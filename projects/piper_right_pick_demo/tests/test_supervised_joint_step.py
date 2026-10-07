"""Software-only tests of one supervised J transaction; all ROS I/O is fake."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from test_fast_ros import telemetry

SPEC = importlib.util.spec_from_file_location("supervised_joint_step", Path(__file__).parents[1]/"scripts/supervised_joint_step.py")
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


def sample(now, seq, second=10000, fourth=0, mode=0):
    raw = telemetry(now, seq)
    raw["raw_q"] = [0, second, -10000, fourth, 0, 0]
    raw["q"] = [v*client.RAD_PER_RAW for v in raw["raw_q"]]
    raw["mode"] = mode
    return raw


def limits():
    return dict(max_speed_percent=1, max_translation_step_m=.03, max_rotation_step_rad=.05,
                workspace_min_m=[-1., -1., 0.], workspace_max_m=[1., 1., 1.], joint_limits_rad=[[-3., 3.]]*6)


class SupervisedJointStepTests(unittest.TestCase):
    def setUp(self):
        for target in ("socket.socket", "subprocess.Popen"):
            guard = patch(target, side_effect=AssertionError("Real device/process forbidden"))
            guard.start(); self.addCleanup(guard.stop)

    def test_relative_single_axis_uses_new_feedback_and_exact_four_frames(self):
        before = sample(100., 10, fourth=4)
        plan = client.make_plan(2, -1., before)
        self.assertEqual(plan["axis"], 1)
        self.assertAlmostEqual(plan["target"][1]-before["q"][1], -math.radians(1.))
        self.assertEqual([plan["target"][i] for i in (0, 2, 3, 4, 5)], [before["q"][i] for i in (0, 2, 3, 4, 5)])
        self.assertEqual(len(plan["expected_frames"]), 4)
        self.assertEqual(plan["message"]["velocity"], [0.]*6+[1.])
        for axis, delta in ((0, -1.), (7, .8), (True, .8), (2, .54), (2, 1.01), (2, math.nan)):
            with self.assertRaises(ValueError): client.make_plan(axis, delta, before)
        with self.assertRaisesRegex(ValueError, "manufacturer"):
            client.make_plan(2, -1., sample(100., 10, second=0))

    def test_original_workspace_and_step_caps_remain(self):
        original = sample(100., 10)
        client.bounds(original, original, limits())
        for changed in (dict(limits(), max_speed_percent=2), dict(limits(), max_translation_step_m=.031),
                        dict(limits(), max_rotation_step_rad=.051)):
            with self.assertRaises(ValueError): client.bounds(original, original, changed)
        for index, value in ((0, .281), (2, -0.001), (3, .051)):
            altered = copy.deepcopy(original); altered["pose"][index] = value
            with self.assertRaises(ValueError): client.bounds(altered, original, limits())

    def test_presend_stability_accepts_existing_driver_feedback_tolerances(self):
        before, current = sample(100., 10), sample(100.02, 11)
        current["q"][5] += math.radians(.111)
        current["pose"][5] += math.radians(.111)
        for axis in range(3):
            current["pose"][axis] += .0004  # Each axis is in bounds; Euclidean norm is larger.
        current["opening_m"] += .0004
        client.assert_no_drift(before, current)
        # Keep the existing sequence/fragment rule: fragments may be equal,
        # but may not regress, and receive sequence must advance.
        current["stamps"] = list(before["stamps"])
        client.assert_no_drift(before, current)

    def test_presend_stability_rejects_excess_motion_jaw_or_stale_feedback(self):
        before = sample(100., 10)
        for field, index, delta in (("q", 5, .00301), ("pose", 5, .00301),
                                    ("pose", 0, .000501), ("pose", 2, -.000501)):
            with self.subTest(field=field, index=index):
                current = sample(100.02, 11)
                current[field][index] += delta
                with self.assertRaises(ValueError): client.assert_no_drift(before, current)
        current = sample(100.02, 11); current["opening_m"] += .000501
        with self.assertRaisesRegex(ValueError, "Gripper drift"): client.assert_no_drift(before, current)
        for current in (sample(100.02, 10), sample(100.02, 9)):
            with self.assertRaisesRegex(ValueError, "New coherent"): client.assert_no_drift(before, current)
        current = sample(100.02, 11); current["stamps"][4] = before["stamps"][4]-.0001
        with self.assertRaisesRegex(ValueError, "New coherent"): client.assert_no_drift(before, current)

    def receipts(self, plan, after):
        return [dict(event="command_intent", kind="joint", sequence=1, speed_percent=1,
                     unix_s=100.041, frames=plan["expected_frames"]),
                dict(event="command_sent_unconfirmed", sequence=1, unix_s=100.042,
                     attempted_frames=4, socket_send_returns=4),
                dict(event="command_observed_stable", kind="joint", sequence=1, unix_s=100.081,
                     arm_target_reached=True, after=after)]

    def test_arrival_uses_existing_point003_tolerance_and_newer_than_stable_receipt(self):
        before = sample(100.04, 12)
        plan = client.make_plan(2, -1., before)
        after = sample(100.08, 14, second=9000, mode=1)
        events = self.receipts(plan, after)
        current = sample(100.10, 15, second=9000, mode=1)
        current["q"][1] += .002  # Normal jitter accepted by the frozen driver.
        self.assertTrue(client.arrived(events, 1, plan, 100.04, current))
        current["q"][1] += .0011
        self.assertFalse(client.arrived(events, 1, plan, 100.04, current))
        self.assertFalse(client.arrived(events, 1, plan, 100.04, after))
        self.assertFalse(client.arrived(events[:2], 1, plan, 100.04, after))
        broken = copy.deepcopy(events); broken[1]["socket_send_returns"] = 3
        with self.assertRaises(RuntimeError): client.arrived(broken, 1, plan, 100.04, after)

    def execute_fake(self, directory, timeout=False):
        initial = sample(100., 10)
        states = [sample(100.02, 11), sample(100.04, 12, fourth=1),
                  sample(100.06, 13, second=9500, fourth=1, mode=1),
                  sample(100.08, 14, second=9000, fourth=1, mode=1),
                  sample(100.10, 15, second=9000, fourth=1, mode=1)]
        clock, seen, published, encoded = [100.], [], [False], [None]
        transport, publisher, arm = Mock(), Mock(), Mock()
        arm.observe.return_value = dict(raw_telemetry=initial, provenance={"speed_percent": 1, "command_sequence": 0})
        arm.ros = {"pose_topic": "/piper/right/pos_cmd", "speed_param": "/piper/right/driver/speed_percent"}
        arm._get_transport.return_value = transport
        transport.rospy.Publisher.return_value = publisher
        transport.rospy.get_name.return_value = "/offline_fixture"
        publisher.get_num_connections.return_value = 1
        transport.master.getParam.return_value = 1
        transport.master.getSystemState.return_value = ([('/piper/right/joint_cmd', ['/offline_fixture'])], [], [])
        def receive(_):
            if timeout and published[0]:
                clock[0] += 121.
                raise TimeoutError("offline timeout")
            value = states[len(seen)]
            seen.append(value); clock[0] = value["stamp"]+.005
            return copy.deepcopy(value)
        transport.receive.side_effect = receive
        def publish(message):
            published[0] = True
            encoded[0] = client.prepare("J", message.position, states[1])
        publisher.publish.side_effect = publish
        transport.events.side_effect = lambda: ([] if not published[0] else
            self.receipts(encoded[0], states[3])[:(3 if len(seen) >= 4 else 2)])
        module = types.ModuleType("sensor_msgs.msg"); module.JointState = lambda **kw: types.SimpleNamespace(**kw)
        result = {"publish_attempts": 0}
        with patch.object(client, "ROSRightArm", return_value=arm), \
                patch.object(client, "check_planned_path", return_value={"scope": "offline fake FK check"}), \
                patch.object(client.time, "time", side_effect=lambda: clock[0]), \
                patch.object(client.time, "monotonic", side_effect=lambda: clock[0]), \
                patch.dict("sys.modules", {"sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": module}):
            if timeout:
                with self.assertRaisesRegex(TimeoutError, "120-second"):
                    client.run({"physical_limits": limits()}, 2, -1., Path(directory), result)
            else:
                client.run({"physical_limits": limits()}, 2, -1., Path(directory), result)
        return result, arm, publisher, states

    def test_whole_transaction_waits_for_arrival_preserves_latest_other_axes_and_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            result, arm, publisher, states = self.execute_fake(directory)
            self.assertTrue(result["arrival_confirmed"])
            self.assertEqual(result["status"], "supervised_joint_step_arrived")
            publisher.publish.assert_called_once()
            self.assertEqual(publisher.publish.call_args.args[0].position[3], states[1]["q"][3])
            self.assertEqual(result["after"]["sequence"], 15)
            trace = [json.loads(s) for s in (Path(directory)/"trajectory.jsonl").read_text().splitlines()]
            self.assertEqual([row["sequence"] for row in trace], [12, 13, 14, 15])
            self.assertTrue((Path(directory)/"before_plan.json").exists())
            publisher.unregister.assert_called_once(); arm.close.assert_called_once()
            arm.execute.assert_not_called(); arm.preflight.assert_not_called()

    def test_timeout_never_publishes_again_and_only_disconnects_client(self):
        with tempfile.TemporaryDirectory() as directory:
            result, arm, publisher, _ = self.execute_fake(directory, timeout=True)
            self.assertEqual(result["publish_attempts"], 1)
            publisher.publish.assert_called_once(); publisher.unregister.assert_called_once(); arm.close.assert_called_once()
            self.assertNotIn("arrival_confirmed", result)


if __name__ == "__main__":
    unittest.main()
