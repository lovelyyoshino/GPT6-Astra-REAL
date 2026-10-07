"""Actual vendor encoding/decoding; simulated CAN only, no robot sockets."""
import contextlib
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import direct_sdk_pick as pick
from direct_sdk_live_step import LiveFeedback
core = pick.core

START = [36562, 281, -329, -1784, 21222, -9858]


class PickTests(unittest.TestCase):
    def setUp(self):
        self.vendor = core.Vendor()
        self.state = LiveFeedback(self.vendor)
        self.clock = 10.
        self.q = list(START)
        self.width, self.effort, self.gripper_status = 23000, 0, 0
        self.closed_width, self.closed_effort = 30000, 140
        self.reaches = True
        self.joint_tracking_error = [0] * 6
        self.fail_identifier = None
        self.enable_works = True
        self.transport = core.RecordingPort()
        self.transport.Close = mock.Mock()
        recorded_send = self.transport.SendCanMessage

        def send(identifier, data, *args, **kwargs):
            if identifier == self.fail_identifier:
                raise OSError("injected partial target failure")
            return recorded_send(identifier, data, *args, **kwargs)

        self.transport.SendCanMessage = send
        self.attr = "_C_PiperInterface_V2__arm_can"
        self.original = getattr(self.vendor.sdk, self.attr)
        self.report = {"binding": {"ifindex": 8}}
        self.receiver = types.SimpleNamespace(one=self.receive, drain=lambda: None)
        self.populate()
        self.controller = None
        self.stack = contextlib.ExitStack()
        self.stack.enter_context(mock.patch.object(core.time, "monotonic", side_effect=lambda: self.clock))
        self.stack.enter_context(mock.patch.object(core.time, "sleep", side_effect=self.sleep))
        self.stack.enter_context(mock.patch.object(core, "inspect_controllers", return_value=[]))
        self.stack.enter_context(mock.patch.object(core, "inspect_binding", return_value={"ifindex": 8}))
        self.stack.enter_context(mock.patch.object(core, "collect_stationary",
            side_effect=lambda *a, **kw: self.state.snapshot(self.clock)))
        self.stack.enter_context(mock.patch.object(self.vendor.sdk, "CreateCanBus",
            side_effect=lambda *a, **kw: setattr(self.vendor.sdk, self.attr, self.transport)))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.controller = pick.PickController(self.vendor, self.receiver, self.state, self.report)

    def tearDown(self):
        if self.controller:
            self.controller.close()
        setattr(self.vendor.sdk, self.attr, self.original)
        self.stack.close()

    def sleep(self, seconds):
        self.clock += seconds

    def populate(self):
        pose = [round(x) for x in self.vendor.fk_pose_raw(self.q)]
        enabled = self.enable_works and any(i == 0x471 for i, _ in self.transport.frames)
        moving = any(i == 0x151 for i, _ in self.transport.frames)
        frames = {0x2A1: bytes([1 if moving else 0, 0, 1, 0, 0, 0, 0, 0]),
                  0x2A8: struct.pack(">ihBB", self.width, self.effort, self.gripper_status, 0)}
        frames.update({identifier: struct.pack(">ii", *pose[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.POSE_PARTS)})
        frames.update({identifier: struct.pack(">ii", *self.q[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.JOINT_PARTS)})
        frames.update({identifier: struct.pack(">HhbBH", 240, 30, 30, 64 if enabled else 0, 0)
                       for identifier in range(0x261, 0x267)})
        for identifier, data in frames.items():
            self.state.ingest(identifier, data, False, self.clock)

    def receive(self, timeout):
        self.clock += .01
        frames = self.transport.frames
        latest_grip = next((data for identifier, data in reversed(frames) if identifier == 0x159), None)
        if latest_grip:
            commanded = struct.unpack(">i", latest_grip[:4])[0]
            self.width = commanded if commanded else self.closed_width
            self.effort = 0 if commanded else self.closed_effort
            self.gripper_status = 64
        if self.reaches:
            for axis, identifier in enumerate((0x155, 0x156, 0x157)):
                data = next((data for i, data in reversed(frames) if i == identifier), None)
                if data:
                    self.q[axis * 2:axis * 2 + 2] = [q + error for q, error in zip(
                        struct.unpack(">ii", data), self.joint_tracking_error[axis * 2:axis * 2 + 2])]
        self.populate()

    def prepared(self):
        self.controller.enable()
        self.controller.gripper(55000, "open")

    def plan(self):
        first = list(self.q); first[4] += 1000
        second = list(first); second[4] += 1000
        stage = lambda label, q: {"kind": "move", "label": label, "joints_raw": q,
            "pose_raw": [round(x) for x in self.vendor.fk_pose_raw(q)], "clearance_mm": 50.}
        return {"start_joints_raw": list(self.q), "start_pose_raw": self.state.snapshot(self.clock)["pose_raw"],
                "geometry": {"table_base_z_mm": 0}, "limits": {}, "stages": [stage("approach", first),
                    {"kind": "capture", "label": "pregrasp"},
                    {"kind": "gripper", "label": "close", "width_raw": 0, "effort_raw": 300},
                    stage("lift", second), {"kind": "capture", "label": "lifted"},
                    {"kind": "gripper", "label": "release", "width_raw": 55000, "effort_raw": 300},
                    {"kind": "capture", "label": "placed"}]}

    def fake_geometry(self):
        module = types.SimpleNamespace(clearance_for_joints=lambda q, g: {"minimum_clearance_mm": 50.})
        return mock.patch.dict(sys.modules, {"pick_trajectory": module})

    def execute(self, plan):
        with self.fake_geometry(), mock.patch.object(self.controller, "observe", side_effect=lambda fn, **kw: fn()):
            return self.controller.execute(plan, lambda label, sample: {"observation": label + ".png"})

    def test_actual_sdk_gripper_encoding_enable_effort_and_never_zero_calibration(self):
        for opening in (0, 23000, 55000):
            identifier, data = pick.encode_gripper(self.vendor.sdk, opening)[0]
            self.assertEqual(identifier, 0x159)
            self.assertEqual(struct.unpack(">iHBB", data), (opening, 300, 1, 0))
        with self.assertRaises(core.Rejected):
            pick.encode_gripper(self.vendor.sdk, 100000)
        self.assertEqual(self.transport.frames, [])

    def test_observation_opening_verified_once_without_becoming_a_grasp(self):
        self.controller.enable()
        result = self.controller.gripper(23000, "observe")
        self.assertEqual(result["classification"], "observation_opening")
        self.assertEqual(result["width_raw"], 23000)
        self.assertFalse(self.report["close_contact_candidate"])
        self.assertEqual(self.report.get("physical_grasp_attempts", 0), 0)
        self.assertNotIn("grasp_outcome", self.report)
        self.assertEqual(struct.unpack(">iHBB", self.transport.frames[-1][1]), (23000, 300, 1, 0))
        count = len(self.transport.frames)
        with self.assertRaises(core.Rejected):
            self.controller.gripper(23000, "observe")
        self.assertEqual(len(self.transport.frames), count)
        self.controller.gripper(55000, "open")
        result = self.execute(self.plan())
        self.assertTrue(result["protocol_completed"])
        self.assertEqual(result["physical_grasp_attempts"], 1)
        self.assertEqual(sum(i == 0x159 for i, _ in self.transport.frames), 4)

    def test_observation_width_is_not_accepted_for_other_purposes_or_late_in_run(self):
        self.controller.enable()
        count = len(self.transport.frames)
        for width, purpose in ((23000, "open"), (23000, "close"), (23000, "release"),
                               (55000, "observe"), (0, "observe"), (23000, "other")):
            with self.subTest(width=width, purpose=purpose):
                with self.assertRaises(core.Rejected):
                    self.controller.gripper(width, purpose)
                self.assertEqual(len(self.transport.frames), count)
        self.controller.gripper(55000, "open")
        count = len(self.transport.frames)
        with self.assertRaisesRegex(core.Rejected, "before other gripper"):
            self.controller.gripper(23000, "observe")
        self.assertEqual(len(self.transport.frames), count)

    def test_observation_opening_outside_two_mm_never_passes_or_retries(self):
        self.controller.enable()

        def receive_outside_tolerance(timeout):
            self.receive(timeout)
            self.width = 25001
            self.populate()

        self.receiver.one = receive_outside_tolerance
        with self.assertRaisesRegex(core.Rejected, "within five seconds"):
            self.controller.gripper(23000, "observe")
        self.assertEqual(sum(i == 0x159 for i, _ in self.transport.frames), 1)
        self.assertFalse(self.report["stages"][-1]["feedback_verified"])
        self.assertEqual(self.report.get("physical_grasp_attempts", 0), 0)

    def test_orientation_wrap_is_stable_but_real_rotation_is_not(self):
        sample = self.state.snapshot(self.clock)
        first = dict(sample, monotonic_s=10., pose_raw=[1000, 2000, 3000, 179999, 30000, 140000])
        last = dict(sample, monotonic_s=10.3, pose_raw=[1000, 2000, 3000, -179999, 30000, 140000])
        self.assertTrue(pick._stable([first, last]))
        last["pose_raw"][3] = -178000
        self.assertFalse(pick._stable([first, last]))

    def test_single_contact_candidate_protocol_exact_frames_and_no_grasp_proof(self):
        self.prepared()
        result = self.execute(self.plan())
        self.assertTrue(result["enable_verified"])
        self.assertTrue(result["protocol_completed"])
        self.assertTrue(result["close_contact_candidate"])
        self.assertFalse(result["grasp_success_verified"])
        self.assertEqual([i for i, _ in self.transport.frames],
                         [0x471, 0x159, 0x151, 0x155, 0x156, 0x157, 0x159,
                          0x151, 0x155, 0x156, 0x157, 0x159])
        self.assertEqual([c["label"] for c in result["captures"]], ["pregrasp", "lifted", "placed"])
        self.assertTrue(all(s["arrival_verified"] for s in result["stages"] if s["kind"] == "move"))

    def test_empty_closure_finishes_explicit_protocol_without_claiming_success(self):
        self.closed_width, self.closed_effort = 1000, 0
        self.prepared()
        result = self.execute(self.plan())
        self.assertTrue(result["protocol_completed"])
        self.assertEqual(result["grasp_outcome"], "empty")
        self.assertEqual(result["physical_grasp_attempts"], 1)
        self.assertFalse(result["grasp_success_verified"])
        self.assertEqual(sum(i == 0x159 for i, _ in self.transport.frames), 3)

    def test_small_real_tracking_residual_arrives_and_next_segment_remains_valid(self):
        self.joint_tracking_error[5] = 30
        self.prepared()
        result = self.execute(self.plan())
        self.assertTrue(result["protocol_completed"])
        moves = [stage for stage in result["stages"] if stage["kind"] == "move"]
        self.assertEqual(len(moves), 2)
        self.assertTrue(all(stage["arrival_verified"] for stage in moves))
        self.assertTrue(all(stage["final"]["joints_raw"][5] - stage["target_joints_raw"][5] == 30
                            for stage in moves))

    def test_measured_stage60_deadband_arrives_with_point_three_degree_limit(self):
        # Recorded run pick_attempt_20261003T194811_102398: normal controller,
        # stationary at the Cartesian goal; J4/J6 stayed near encoder zero.
        self.prepared()
        target = {"target_joints_raw": [39696, 110447, -72100, 117, 26652, -195],
                  "target_pose_raw": [309155, 256737, 175849, 180000, 30001, -140200],
                  "command_sent": True, "sent_at_monotonic_s": self.clock - 1}
        sample = self.state.snapshot(self.clock, True)
        sample.update(joints_raw=[39700, 110439, -72100, 0, 26611, 0],
                      pose_raw=[309225, 256731, 175937, 180000, 30049, -140299],
                      status={"ctrl_mode": 1, "mode_feed": 1, "motion_status": 0})
        self.assertTrue(self.controller._arrived(sample, target))
        residuals = pick.arrival_residuals(sample, target)
        self.assertEqual(residuals["joint_error_deg"], [.004, -.008, 0., -.117, -.041, .195])
        self.assertAlmostEqual(residuals["max_joint_error_deg"], .195)
        self.assertLess(residuals["position_error_mm"], .114)
        self.assertLess(residuals["rotation_error_deg"], .25)

        sample["joints_raw"][5] = target["target_joints_raw"][5] + 300
        self.assertTrue(self.controller._arrived(sample, target))
        sample["joints_raw"][5] += 1
        self.assertFalse(self.controller._arrived(sample, target))
        sample["joints_raw"] = list(target["target_joints_raw"])
        sample["pose_raw"] = list(target["target_pose_raw"])
        sample["pose_raw"][0] += 501
        self.assertFalse(self.controller._arrived(sample, target))
        sample["pose_raw"] = list(target["target_pose_raw"])
        sample["pose_raw"][5] += 251
        self.assertFalse(self.controller._arrived(sample, target))

    def test_zero_effort_blocked_closure_stops_before_carry_or_release(self):
        self.closed_width, self.closed_effort = 30000, 0
        self.prepared()
        result = self.execute(self.plan())
        self.assertEqual(result["status"], "grasp_ambiguous")
        self.assertFalse(result["protocol_completed"])
        self.assertEqual(sum(i == 0x151 for i, _ in self.transport.frames), 1)
        self.assertEqual(sum(i == 0x159 for i, _ in self.transport.frames), 2)

    def test_whole_plan_invalid_late_segment_rejected_before_any_arm_target(self):
        self.prepared()
        plan = self.plan()
        plan["stages"][3]["pose_raw"][0] += 30000
        with self.assertRaisesRegex(core.Rejected, "segmentation"):
            self.execute(plan)
        self.assertEqual([i for i, _ in self.transport.frames], [0x471, 0x159])
        self.assertGreater(len(self.report["post_failure_observations"]), 20)

    def test_enable_failure_never_sends_gripper_or_motion(self):
        self.enable_works = False
        with self.assertRaisesRegex(core.Rejected, "enables were not verified"):
            self.controller.enable()
        self.assertLessEqual(len(self.transport.frames), 20)
        self.assertTrue(all(i == 0x471 for i, _ in self.transport.frames))

    def test_partial_joint_send_seals_and_observes_without_retries_or_arrival(self):
        self.prepared()
        self.fail_identifier = 0x156
        before = self.clock
        with self.assertRaisesRegex(OSError, "partial target failure"):
            self.execute(self.plan())
        self.assertTrue(self.report["partial_motion_target"])
        self.assertFalse(self.report["post_failure_target_stable"])
        self.assertFalse(self.report["protocol_completed"])
        self.assertGreater(len(self.report["post_failure_observations"]), 20)
        self.assertLess(self.clock - before, 5.05)
        self.assertEqual([i for i, _ in self.transport.frames], [0x471, 0x159, 0x151, 0x155])
        with self.assertRaises(core.Rejected):
            self.controller.gripper(0, "close")

    def test_motion_timeout_single_goal_finite_passive_tail_not_success(self):
        self.prepared()
        self.reaches = False
        before = self.clock
        with self.assertRaisesRegex(core.Rejected, "within five seconds"):
            self.execute(self.plan())
        self.assertLess(self.clock - before, 10.1)
        self.assertEqual(sum(i == 0x151 for i, _ in self.transport.frames), 1)
        self.assertFalse(self.report["protocol_completed"])
        move = next(stage for stage in self.report["stages"] if stage["kind"] == "move")
        self.assertIn("last_arrival_residuals", move)
        self.assertGreater(move["last_arrival_residuals"]["max_joint_error_deg"], .3)
        self.assertIn("position_error_mm", self.report["failure"]["error"])
        self.assertIn("joint_error_deg", self.report["failure"]["error"])


if __name__ == "__main__":
    unittest.main()
