"""Offline controller-contract checks with real vendor CAN codecs, no sockets.

The simulated controller follows Cartesian targets; it does not simulate or
claim to validate the physical arm's firmware IK or collision clearance.
"""
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_direct_sdk_pick as fixtures
import sdk_pose_batch as batch

core = fixtures.core
START_Q = [39700, 110439, -72100, 0, 26611, 0]
START_POSE = [309225, 256731, 175937, 180000, 30049, -140299]
# Measured feedback from pose_batch_20261003T212603_134296/report.json.
DESCEND_REFERENCE = [315506, 262879, 167926, 180000, 30000, -140199]
DESCEND_GOAL = [315504, 262877, 117929, 180000, 30000, -140199]
DESCEND_TRANSIENT = [315995, 263019, 124613, 180000, 30361, -140227]
DESCEND_FINAL = [315727, 262796, 117827, 180000, 30020, -140227]
DESCEND_FINAL_Q = [39772, 118370, -69906, 0, 16516, 0]


class PoseBatchTests(unittest.TestCase):
    # Reuse transport setup only, without inheriting or duplicating pick tests.
    tearDown = fixtures.PickTests.tearDown
    sleep = fixtures.PickTests.sleep
    prepared = fixtures.PickTests.prepared

    def setUp(self):
        self.pose = list(START_POSE)
        self.freeze_frame = None
        self.path_error = None
        self.branch_error = None
        self.orientation_error_raw = None
        self.orientation_error_samples = None
        self.pose_receive_count = 0
        fixtures.PickTests.setUp(self)
        self.q = list(START_Q)
        self.populate()
        self.controller = batch.PoseBatchController(
            self.vendor, self.receiver, self.state, self.report)

    def populate(self):
        enabled = self.enable_works and any(i == 0x471 for i, _ in self.transport.frames)
        mode = next((data[1] for identifier, data in reversed(self.transport.frames)
                     if identifier == 0x151), 1)
        controlled = any(i == 0x151 for i, _ in self.transport.frames)
        frames = {
            0x2A1: bytes([1 if controlled else 0, 0, mode, 0, 0, 0, 0, 0]),
            0x2A8: struct.pack(">ihBB", self.width, self.effort, self.gripper_status, 0),
        }
        frames.update({identifier: struct.pack(">ii", *self.pose[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.POSE_PARTS)})
        frames.update({identifier: struct.pack(">ii", *self.q[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.JOINT_PARTS)})
        frames.update({identifier: struct.pack(">HhbBH", 240, 30, 30, 64 if enabled else 0, 0)
                       for identifier in range(0x261, 0x267)})
        for identifier, payload in frames.items():
            if identifier != self.freeze_frame:
                self.state.ingest(identifier, payload, False, self.clock)

    def receive(self, timeout):
        self.clock += .01
        frames = self.transport.frames
        latest_grip = next((data for identifier, data in reversed(frames) if identifier == 0x159), None)
        if latest_grip:
            opening = struct.unpack(">i", latest_grip[:4])[0]
            self.width = opening if opening else self.closed_width
            self.effort = 0 if opening else self.closed_effort
            self.gripper_status = 64
        pose_parts = [next((data for identifier, data in reversed(frames) if identifier == part), None)
                      for part in (0x152, 0x153, 0x154)]
        if all(pose_parts):
            goal = [value for data in pose_parts for value in struct.unpack(">ii", data)]
            if self.reaches:
                for axis in range(3):
                    delta = goal[axis] - self.pose[axis]
                    self.pose[axis] += min(1000, max(-1000, delta))
                self.pose[3:] = goal[3:]
            if (self.orientation_error_raw is not None and
                    (self.orientation_error_samples is None or
                     self.pose_receive_count < self.orientation_error_samples)):
                self.pose[3:] = [value + error for value, error in zip(goal[3:], self.orientation_error_raw)]
            if self.path_error is not None:
                self.pose[1] = START_POSE[1] + self.path_error
            if self.branch_error is not None:
                self.q[0] = START_Q[0] + self.branch_error
            self.pose_receive_count += 1
        self.populate()

    def plan(self):
        offsets = [(5000, 0, 0), (5000, 0, -20000), (5000, 0, 20000),
                   (5000, 20000, 20000), (5000, 20000, -10000), (5000, 20000, 20000)]
        labels = ("approach", "descend", "lift", "carry", "lower", "retreat")
        previous = list(self.pose)
        stages = []
        for index, (label, offset) in enumerate(zip(labels, offsets)):
            target = [self.pose[axis] + offset[axis] for axis in range(3)] + self.pose[3:]
            stages.append({"kind": "move", "label": label, "pose_raw": target,
                           "reference_start_pose_raw": previous,
                           "speed_percent": 5,
                           "joint_lower_raw": [q - 10000 for q in self.q],
                           "joint_upper_raw": [q + 10000 for q in self.q]})
            previous = target
            if index in (1, 4):
                stages.append({"kind": "gripper", "label": "close" if index == 1 else "release",
                               "width_raw": 0 if index == 1 else 55000, "effort_raw": 300})
        return {"start_joints_raw": list(self.q), "start_pose_raw": list(self.pose),
                "geometry": {"table_base_z_mm": 0}, "stages": stages}

    def execute(self, plan=None):
        geometry = types.SimpleNamespace(clearance_for_joints=lambda q, g: {"minimum_clearance_mm": 50.})
        with mock.patch.dict(sys.modules, {"pick_trajectory": geometry}):
            with mock.patch.object(self.controller, "observe", side_effect=lambda fn, **kw: fn()):
                return self.controller.execute_pose_plan(plan or self.plan())

    def close_first_plan(self):
        stages = [{"kind": "gripper", "label": "close", "width_raw": 0, "effort_raw": 300}]
        previous = list(self.pose)
        for label, offset in zip(("lift", "carry", "lower", "retreat"),
                                 ((0, 0, 40000), (0, 60000, 40000), (0, 60000, 0), (0, 60000, 40000))):
            target = [self.pose[axis] + offset[axis] for axis in range(3)] + self.pose[3:]
            stages.append({"kind": "move", "label": label, "pose_raw": target,
                           "reference_start_pose_raw": previous, "speed_percent": 5,
                           "joint_lower_raw": [q - 10000 for q in self.q],
                           "joint_upper_raw": [q + 10000 for q in self.q]})
            previous = target
            if label == "lower":
                stages.append({"kind": "gripper", "label": "release", "width_raw": 55000, "effort_raw": 300})
        return {"start_joints_raw": list(self.q), "start_pose_raw": list(self.pose),
                "geometry": {"table_base_z_mm": 0}, "stages": stages}

    def assert_no_close_or_later_move(self):
        self.assertEqual(sum(i == 0x151 for i, _ in self.transport.frames), 1)
        self.assertEqual(sum(i == 0x159 for i, _ in self.transport.frames), 1)
        self.assertFalse(self.report["protocol_completed"])
        before = len(self.transport.frames)
        with self.assertRaises(core.Rejected):
            self.controller.gripper(0, "close")
        self.assertEqual(len(self.transport.frames), before)

    def test_real_sdk_pose_encoding_and_six_goal_pick_place(self):
        self.prepared()
        plan = self.plan()
        result = self.execute(plan)
        self.assertTrue(result["protocol_completed"])
        self.assertFalse(result["grasp_success_verified"])
        self.assertEqual(result["physical_grasp_attempts"], 1)
        self.assertGreater(self.pose_receive_count, 80)
        moves = [stage for stage in result["stages"] if stage["kind"] == "move"]
        self.assertEqual(len(moves), 6)
        self.assertTrue(all(stage["arrival_verified"] for stage in moves))
        encoded = [frame for frame in self.transport.frames if frame[0] in (0x151, 0x152, 0x153, 0x154)]
        targets = [stage["pose_raw"] for stage in plan["stages"] if stage["kind"] == "move"]
        self.assertEqual([identifier for identifier, _ in encoded], [0x151, 0x152, 0x153, 0x154] * 6)
        self.assertFalse(any(i in (0x155, 0x156, 0x157) for i, _ in self.transport.frames))
        for index, target in enumerate(targets):
            four_frames = encoded[index * 4:index * 4 + 4]
            self.assertEqual(tuple(four_frames[0][1][:4]), (1, 2, 5, 0))
            decoded = [value for _, payload in four_frames[1:] for value in struct.unpack(">ii", payload)]
            self.assertEqual(decoded, target)
        self.assertEqual([struct.unpack(">i", payload[:4])[0]
                          for identifier, payload in self.transport.frames if identifier == 0x159],
                         [55000, 0, 55000])

    def test_equivalent_pi_command_jump_is_rejected_before_any_motion(self):
        self.prepared()
        plan = self.plan()
        for stage in plan["stages"]:
            if stage["kind"] == "move":
                stage["pose_raw"][3] = -180000
        with self.assertRaisesRegex(core.Rejected, "continuous Euler"):
            self.execute(plan)
        self.assertEqual([identifier for identifier, _ in self.transport.frames], [0x471, 0x159])
        self.assertFalse(self.report["protocol_completed"])

    def test_equivalent_negative_pi_feedback_is_accepted_at_positive_pi_goal(self):
        self.prepared()
        sample = self.state.snapshot(self.clock, True)
        sample["status"].update(ctrl_mode=1, mode_feed=2, motion_status=0)
        target = {"command_sent": True, "target_pose_raw": list(sample["pose_raw"]),
                  "sent_at_monotonic_s": self.clock - .01}
        self.assertEqual(target["target_pose_raw"][3], 180000)
        sample["pose_raw"][3] = -180000
        self.assertTrue(self.controller._arrived(sample, target))

    def test_pose_arrival_does_not_require_historical_joint_target(self):
        self.prepared()
        sample = self.state.snapshot(self.clock, True)
        sample["status"].update(ctrl_mode=1, mode_feed=2, motion_status=0)
        sample["joints_raw"][0] += 5000
        target = {"command_sent": True, "target_pose_raw": list(sample["pose_raw"]),
                  "sent_at_monotonic_s": self.clock - .01}
        # No inverse-solved or historical target_joints_raw exists in this API.
        self.assertTrue(self.controller._arrived(sample, target))
        sample["pose_raw"][0] += 1000
        self.assertTrue(self.controller._arrived(sample, target))
        sample["pose_raw"][0] += 1
        self.assertFalse(self.controller._arrived(sample, target))
        sample["pose_raw"] = list(target["target_pose_raw"])
        sample["pose_raw"][5] += 301
        self.assertFalse(self.controller._arrived(sample, target))

    def test_arrival_requires_post_command_feedback_and_cartesian_mode(self):
        self.prepared()
        sample = self.state.snapshot(self.clock, True)
        sample["status"].update(ctrl_mode=1, mode_feed=2, motion_status=0)
        target = {"command_sent": True, "target_pose_raw": list(sample["pose_raw"]),
                  "sent_at_monotonic_s": self.clock - .01}
        self.assertTrue(self.controller._arrived(sample, target))
        self.state.received[0x2A3] = target["sent_at_monotonic_s"]
        self.assertFalse(self.controller._arrived(sample, target))
        self.state.received[0x2A3] = self.clock
        sample["status"]["mode_feed"] = 1
        self.assertFalse(self.controller._arrived(sample, target))

    def test_late_invalid_pose_prevents_entire_motion_batch(self):
        self.prepared()
        plan = self.plan()
        plan["stages"][-1]["pose_raw"][2] += 1000000
        with self.assertRaises(core.Rejected):
            self.execute(plan)
        self.assertEqual([i for i, _ in self.transport.frames], [0x471, 0x159])

    def test_partial_pose_target_is_not_retried_or_followed_by_closure(self):
        self.prepared()
        self.fail_identifier = 0x153
        with self.assertRaisesRegex(OSError, "partial target failure"):
            self.execute()
        self.assertEqual([i for i, _ in self.transport.frames], [0x471, 0x159, 0x151, 0x152])
        self.assertTrue(self.report["partial_motion_target"])
        self.assertFalse(self.report["post_failure_target_stable"])
        self.assert_no_close_or_later_move()

    def test_one_stale_pose_frame_stops_before_closure(self):
        self.prepared()
        self.freeze_frame = 0x2A3
        with self.assertRaisesRegex(core.Rejected, "stale"):
            self.execute()
        self.assert_no_close_or_later_move()

    def test_nonarriving_target_times_out_without_next_goal(self):
        self.prepared()
        self.reaches = False
        before = self.clock
        with self.assertRaises(core.Rejected):
            self.execute()
        self.assertLess(self.clock - before, 21.)
        self.assert_no_close_or_later_move()

    def test_motion_leaving_cartesian_segment_stops_before_closure(self):
        self.prepared()
        self.path_error = 20000
        with self.assertRaises(core.Rejected):
            self.execute()
        self.assert_no_close_or_later_move()

    def test_joint_branch_box_is_enforced_even_inside_nominal_joint_limits(self):
        self.prepared()
        self.branch_error = 12000
        with self.assertRaises(core.Rejected):
            self.execute()
        self.assert_no_close_or_later_move()

    def test_recorded_descend_transient_is_inside_motion_limits_but_not_arrived(self):
        self.prepared()
        sample = self.state.snapshot(self.clock, True)
        sample["pose_raw"] = list(DESCEND_TRANSIENT)
        sample["status"].update(ctrl_mode=1, mode_feed=2, motion_status=0)
        residuals = batch.motion_residuals(sample, DESCEND_REFERENCE, DESCEND_GOAL)
        self.assertAlmostEqual(residuals["rotation_tracking_error_deg"], .362084, places=5)
        self.assertLess(residuals["position_corridor_error_mm"], 1.)
        self.assertEqual(residuals["motion_limits"], {"position_mm": 5, "rotation_deg": .5})
        target = {"command_sent": True, "target_pose_raw": list(DESCEND_GOAL),
                  "sent_at_monotonic_s": self.clock - .01}
        # Even at the target XYZ, the measured 0.362-degree rotation is not settled.
        sample["pose_raw"][:3] = DESCEND_GOAL[:3]
        self.assertFalse(self.controller._arrived(sample, target))
        sample["pose_raw"] = list(DESCEND_FINAL)
        self.assertTrue(self.controller._arrived(sample, target))

    def test_motion_accepts_transient_orientation_then_requires_precise_arrival(self):
        self.prepared()
        self.orientation_error_raw = (0, 361, -28)
        self.orientation_error_samples = 12
        plan = self.plan()
        result = self.execute(plan)
        self.assertTrue(result["protocol_completed"])
        transient_trace = [sample for sample in result["trace"] if sample.get("stage") == "approach" and
                           core.rotation_distance_deg(sample["pose_raw"], plan["start_pose_raw"]) > .3]
        self.assertTrue(transient_trace)
        self.assertTrue(all(core.rotation_distance_deg(sample["pose_raw"], plan["start_pose_raw"]) <= .5
                            for sample in transient_trace))
        moves = [stage for stage in result["stages"] if stage["kind"] == "move"]
        self.assertTrue(all(stage["arrival_verified"] and
                            stage["last_arrival_residuals"]["rotation_error_deg"] <= .3 for stage in moves))

    def test_half_degree_motion_limit_is_enforced_without_closure(self):
        self.prepared()
        self.orientation_error_raw = (0, 501, 0)
        with self.assertRaises(core.Rejected):
            self.execute()
        self.assert_no_close_or_later_move()
        residuals = self.report["stages"][-1]["last_motion_residuals"]
        self.assertGreater(residuals["rotation_tracking_error_deg"], .5)

    def test_five_mm_motion_limit_is_enforced_without_closure(self):
        self.prepared()
        self.path_error = 5001
        with self.assertRaises(core.Rejected):
            self.execute()
        self.assert_no_close_or_later_move()
        residuals = self.report["stages"][-1]["last_motion_residuals"]
        self.assertGreater(residuals["position_corridor_error_mm"], 5.)

    def test_close_at_current_grasp_pose_then_four_moves_without_repeated_descent(self):
        self.pose, self.q = list(DESCEND_FINAL), list(DESCEND_FINAL_Q)
        self.populate()
        self.prepared()
        plan = self.close_first_plan()
        result = self.execute(plan)
        self.assertTrue(result["protocol_completed"])
        self.assertEqual(result["physical_grasp_attempts"], 1)
        self.assertFalse(result["grasp_success_verified"])
        self.assertEqual([identifier for identifier, _ in self.transport.frames[:4]],
                         [0x471, 0x159, 0x159, 0x151])
        moves = [stage for stage in result["stages"] if stage["kind"] == "move"]
        self.assertEqual([stage["label"] for stage in moves], ["lift", "carry", "lower", "retreat"])
        self.assertEqual(moves[0]["target_pose_raw"][2], DESCEND_FINAL[2] + 40000)
        self.assertTrue(all(stage["target_pose_raw"][2] >= DESCEND_FINAL[2] for stage in moves))
        self.assertEqual(sum(identifier == 0x151 for identifier, _ in self.transport.frames), 4)
        widths = [struct.unpack(">i", payload[:4])[0] for identifier, payload in self.transport.frames
                  if identifier == 0x159]
        self.assertEqual(widths, [55000, 0, 55000])

    def test_invalid_later_goal_prevents_close_first_batch_before_closure(self):
        self.prepared()
        plan = self.close_first_plan()
        plan["stages"][-1]["pose_raw"][2] += 100000
        with self.assertRaises(core.Rejected):
            self.execute(plan)
        self.assertEqual([identifier for identifier, _ in self.transport.frames], [0x471, 0x159])


if __name__ == "__main__":
    unittest.main()
