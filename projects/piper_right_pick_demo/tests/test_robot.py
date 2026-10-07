import sys
import types
import unittest

from right_pick.robot import (MotionUnconfigured, RobotUnavailable, RosRightArm,
    decode_end_pose, decode_joint_state, decode_status, validate_freshness)


class RobotBoundaryTests(unittest.TestCase):
    def test_no_driver_import_or_motion_on_construction(self):
        before = {name for name in sys.modules if name in ("rospy", "piper_sdk")}
        robot = RosRightArm({"arm": "right", "topics": {
            "joint_states": "/right/joints", "end_pose": "/right/pose", "status": "/right/status"}})
        after = {name for name in sys.modules if name in ("rospy", "piper_sdk")}
        self.assertEqual(before, after)
        with self.assertRaises(MotionUnconfigured):
            robot.move({"arm": "right", "position_m": [0.1, 0.2, 0.3]})
        with self.assertRaises(MotionUnconfigured):
            robot.stop()

    def test_wrong_arm_rejected(self):
        with self.assertRaises(RobotUnavailable):
            RosRightArm({"arm": "left"})

    def test_missing_status_does_not_mean_healthy(self):
        with self.assertRaises(RobotUnavailable):
            decode_status(types.SimpleNamespace(ctrl_mode=1))

    def test_ctrl_mode_does_not_imply_enabled(self):
        values = {k: 0 for k in ("arm_status", "mode_feedback", "teach_status",
                                 "motion_status", "trajectory_num", "err_code")}
        values["ctrl_mode"] = 1
        for i in range(1, 7):
            values["joint_%d_angle_limit" % i] = False
            values["communication_status_joint_%d" % i] = False
        decoded = decode_status(types.SimpleNamespace(**values))
        self.assertIsNone(decoded["enabled"])
        values["communication_status_joint_2"] = True
        self.assertTrue(decode_status(types.SimpleNamespace(**values))["fault_reported"])

    def test_stale_pose_rejected_despite_fresh_receipt(self):
        with self.assertRaises(RobotUnavailable):
            validate_freshness({"end_pose": {"received_at_s": 100,
                "data": {"source_stamp_s": 90}}}, 100, 1, .2)

    def test_excessive_feedback_skew_rejected(self):
        samples = {name: {"received_at_s": 100, "data": {"source_stamp_s": stamp}}
                   for name, stamp in (("joint_states", 100), ("end_pose", 99.5))}
        with self.assertRaises(RobotUnavailable):
            validate_freshness(samples, 100, 1, .2)

    def test_joint_missing_or_nonfinite_rejected(self):
        for positions in ([0] * 5, [0, 0, float("nan"), 0, 0, 0]):
            message = types.SimpleNamespace(name=["joint%d" % i for i in range(1, 7)],
                position=positions, velocity=[], effort=[])
            with self.assertRaises(RobotUnavailable):
                decode_joint_state(message)

    def test_quaternion_conversion_and_tcp_not_assumed(self):
        ns = types.SimpleNamespace
        message = ns(header=ns(frame_id="", stamp=ns(to_sec=lambda: 100)),
                     pose=ns(position=ns(x=.1, y=.2, z=.3),
                             orientation=ns(x=1, y=0, z=0, w=0)))
        decoded = decode_end_pose(message)
        self.assertEqual(decoded["quaternion_wxyz"], [0, 1, 0, 0])
        self.assertIn("not_verified", decoded["reference"])


if __name__ == "__main__":
    unittest.main()
