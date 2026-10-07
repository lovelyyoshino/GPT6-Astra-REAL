"""Pure numeric commissioning tests; imports must not connect ROS/CAN."""
import copy
import importlib.util
import math
import json
import struct
from pathlib import Path
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("client_exit", Path(__file__).parents[1] / "scripts/commission_ros_client_exit.py")
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


class CommissionClientNumericTests(unittest.TestCase):
    def setUp(self):
        self.before = {"q": [0., .5, -.5, 0., 0., 0.], "pose": [.1, 0., .2, 0., 0., 0.],
                       "sequence": 10, "stamps": [100.]*14, "stamp": 100., "mode": 1, "motion_status": 0}
        for target in ("socket.socket", "subprocess.Popen"):
            guard = patch(target, side_effect=AssertionError("Hardware/process forbidden"))
            guard.start()
            self.addCleanup(guard.stop)

    def test_joint_limits_and_exactly_one_axis_are_enforced(self):
        target = list(self.before["q"])
        target[3] = math.radians(.8)
        plan = client.prepare("J", target, self.before)
        self.assertEqual(plan["axis"], 3)
        self.assertEqual(len(plan["expected_frames"]), 4)
        self.assertEqual(plan["message"]["velocity"], [0.]*6+[1.])
        for bad in (2., .2):
            altered = list(target)
            altered[3] = math.radians(bad)
            with self.assertRaises(ValueError):
                client.prepare("J", altered, self.before)
        altered = list(target)
        altered[2] += .01
        with self.assertRaisesRegex(ValueError, "more than one"):
            client.prepare("J", altered, self.before)
        shifted = copy.deepcopy(self.before)
        shifted["q"][0] = math.radians(150)
        altered = list(shifted["q"])
        altered[0] += math.radians(.8)
        with self.assertRaisesRegex(ValueError, "manufacturer"):
            client.prepare("J", altered, shifted)

    def test_other_five_joints_use_current_feedback_not_old_target_jitter(self):
        target = list(self.before["q"])
        target[3] += math.radians(.8)
        target[2] += .0001
        plan = client.prepare("J", target, self.before)
        self.assertEqual(plan["target"][2], self.before["q"][2])
        self.assertEqual(plan["requested_target"][2], target[2])

    def test_pose_never_descends_exceeds_six_mm_or_rotates_far(self):
        target = list(self.before["pose"])
        target[2] += .005
        plan = client.prepare("P", target, self.before, .035)
        self.assertEqual(len(plan["expected_frames"]), 7)
        self.assertEqual(plan["message"]["gripper"], .035)
        for axis, delta in ((2, -.005), (0, .007), (3, .02)):
            altered = list(target if axis == 3 else self.before["pose"])
            altered[axis] += delta
            with self.assertRaises(ValueError):
                client.prepare("P", altered, self.before, .035)
        with self.assertRaisesRegex(ValueError, "established"):
            client.prepare("P", target, self.before)

    def advance_clock(self, state, when, sequence, mode=1):
        state.update(stamp=when, stamps=[when]*14, sequence=sequence, mode=mode)
        return state

    def test_exit_requires_fresh_measured_trend_and_remaining_distance_not_status_bit(self):
        target = list(self.before["q"])
        target[3] += math.radians(.8)
        plan = client.prepare("J", target, self.before)
        earlier = self.advance_clock(copy.deepcopy(self.before), 100.02, 11)
        earlier["q"][3] = math.radians(.2)
        current = self.advance_clock(copy.deepcopy(self.before), 100.04, 12)
        current["q"][3] = math.radians(.3)
        self.assertFalse(client.progress(plan, self.before, current)["exit_condition_met"])
        self.assertTrue(client.progress(plan, self.before, current, [earlier])["exit_condition_met"])
        self.assertEqual(client.progress(plan, self.before, current, [earlier])["motion_status"], 0)
        for changed in (dict(earlier, stamp=99.8), dict(earlier, stamps=current["stamps"]),
                        dict(earlier, q=current["q"]), dict(earlier, sequence=12)):
            self.assertFalse(client.progress(plan, self.before, current, [changed])["exit_condition_met"])
        current["q"][3] = -math.radians(.3)
        self.assertFalse(client.progress(plan, self.before, current, [earlier])["exit_condition_met"])
        current["q"][3] = math.radians(.75)
        self.assertFalse(client.progress(plan, self.before, current, [earlier])["exit_condition_met"])
        pose_target = list(self.before["pose"])
        pose_target[2] += .005
        pplan = client.prepare("P", pose_target, self.before, .03)
        earlier = self.advance_clock(copy.deepcopy(self.before), 100.02, 11, mode=0)
        earlier["pose"][2] += .001
        current = self.advance_clock(copy.deepcopy(self.before), 100.04, 12, mode=0)
        current["pose"][2] += .0015
        self.assertTrue(client.progress(pplan, self.before, current, [earlier])["exit_condition_met"])
        current["pose"][2] -= .003
        self.assertFalse(client.progress(pplan, self.before, current, [earlier])["exit_condition_met"])

    def test_replay_genuine_joint_motion_with_status_zero_but_old_late_exit_stays_failed(self):
        fixture = json.loads((Path(__file__).parent / "fixtures/client_exit_J_motion_flag_zero.json").read_text())
        before, plan = fixture["before"], fixture["plan"]
        recent, candidates = [], []
        for index, sample in enumerate(fixture["samples"]):
            frames = {frame["id"]: frame for frame in sample["frames"]}
            q = []
            for identifier in (0x2A5, 0x2A6, 0x2A7):
                q.extend(v * client.RAD_PER_RAW for v in struct.unpack(">ii", bytes.fromhex(frames[identifier]["data_hex"])))
            status = bytes.fromhex(frames[0x2A1]["data_hex"])
            current = dict(q=q, pose=before["pose"], sequence=index+1, stamp=sample["sampled_at"],
                           stamps=[frames[i]["timestamp"] for i in list(range(0x2A1,0x2A9))+list(range(0x261,0x267))],
                           mode=status[2], motion_status=status[4])
            self.assertEqual(current["motion_status"], 0)  # actual firmware bytes, not an invented bit
            result = client.progress(plan, before, current, recent)
            if result["exit_condition_met"]:
                candidates.append((current, result))
            recent.append(current)
        self.assertGreater(len(candidates), 0)
        first, evidence = candidates[0]
        self.assertAlmostEqual(math.degrees(first["q"][0]), .214, places=6)
        self.assertGreaterEqual(evidence["measured_motion_evidence"]["target_error_reduction"], .0005)
        self.assertFalse(client.progress(plan, before, fixture["old_failed_exit_final"], recent)["exit_condition_met"])

    def test_new_feedback_and_small_drift_are_required(self):
        current = copy.deepcopy(self.before)
        with self.assertRaisesRegex(ValueError, "New coherent"):
            client.assert_no_drift(self.before, current)
        current["sequence"] += 1
        client.assert_no_drift(self.before, current)
        current["q"][0] += .002
        with self.assertRaisesRegex(ValueError, "Joint drift"):
            client.assert_no_drift(self.before, current)

    def test_matching_single_command_receipt_is_required(self):
        target = list(self.before["q"])
        target[3] += math.radians(.8)
        plan = client.prepare("J", target, self.before)
        intent = dict(event="command_intent", sequence=1, kind="joint", speed_percent=1,
                      unix_s=100.1, frames=plan["expected_frames"])
        sent = dict(event="command_sent_unconfirmed", sequence=1, attempted_frames=4, socket_send_returns=4)
        self.assertFalse(client.check_receipt([], 1, plan, 100.))
        self.assertTrue(client.check_receipt([intent, sent], 1, plan, 100.))
        with self.assertRaisesRegex(RuntimeError, "additional"):
            client.check_receipt([intent, dict(intent, sequence=2)], 1, plan, 100.)
        with self.assertRaisesRegex(RuntimeError, "mismatch"):
            client.check_receipt([dict(intent, speed_percent=2), sent], 1, plan, 100.)


if __name__ == "__main__":
    unittest.main()
