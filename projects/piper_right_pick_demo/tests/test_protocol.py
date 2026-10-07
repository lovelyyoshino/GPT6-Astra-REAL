import math
import unittest
from dataclasses import replace

from right_pick.protocol import Action, Policy, ProtocolError, validate_action


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.move = {"type": "move_tcp", "arm": "right", "issued_at": 99.9,
                     "ttl_s": 1.0, "calibration_version": "fresh-v2", "speed_m_s": 0.01,
                     "pose": {"position_m": [0.25, 0, 0.22], "orientation_wxyz": [1, 0, 0, 0]}}
        self.obs = {"captured_at": 99.9, "robot_state_at": 99.9,
                    "right_tcp_position_m": [0.25, 0, 0.20],
                    "right_tcp_orientation_wxyz": [1, 0, 0, 0],
                    "calibration_version": "fresh-v2", "device_freshness_verified": True,
                    "right_binding_verified": True, "enabled": True, "fault": 0}
        self.policy = Policy(allow_motion=True, calibration_verified=True,
                             calibration_version="fresh-v2", camera_geometry_changed=False,
                             tcp_confirmed=True, gripper_confirmed=True, workspace_confirmed=True,
                             workspace_min_m=(0.1, -0.1, 0.1), workspace_max_m=(0.4, 0.1, 0.4),
                             max_translation_step_m=0.03, max_orientation_step_rad=0.1,
                             max_speed_m_s=0.02, gripper_min_width_m=0, gripper_max_width_m=0.08)

    def check(self, action=None, obs=None, policy=None):
        return validate_action(action or self.move, self.obs if obs is None else obs,
                               self.policy if policy is None else policy, self.now)

    def test_safe_small_move_is_accepted(self):
        self.assertTrue(self.check().accepted)

    def test_camera_tilt_change_invalidates_even_allowed_motion(self):
        result = self.check(policy=replace(self.policy, camera_geometry_changed=True))
        self.assertEqual(result.code, "camera_geometry_changed")

    def test_default_closed_and_only_right(self):
        self.assertEqual(self.check(policy=Policy()).code, "motion_disabled")
        for arm in ("left", "both", None):
            with self.assertRaises(ProtocolError):
                Action.from_dict(dict(self.move, arm=arm))

    def test_unknown_and_nonfinite_fields_are_rejected(self):
        for patch in ({"speed": 1}, {"speed_m_s": math.nan}, {"speed_m_s": True},
                      {"ttl_s": math.inf}, {"issued_at": False}):
            with self.subTest(patch=patch), self.assertRaises(ProtocolError):
                Action.from_dict(dict(self.move, **patch))
        for q in ([0, 0, 0, 0], [2, 0, 0, 0], [1, 0, math.inf, 0], [1, 0, 0]):
            with self.subTest(q=q), self.assertRaises(ProtocolError):
                Action.from_dict(dict(self.move, pose={"position_m": [0.2, 0, 0.2], "orientation_wxyz": q}))
        with self.assertRaises(ProtocolError):
            Action.from_dict({"type": "emergency_stop"})

    def test_direct_dataclass_cannot_bypass_parser(self):
        bad = replace(Action.from_dict(self.move), arm="left")
        self.assertFalse(self.check(action=bad).accepted)

    def test_no_arbitrary_units_or_hidden_pose_fields(self):
        bad = dict(self.move, pose=dict(self.move["pose"], units="mm"))
        self.assertFalse(self.check(action=bad).accepted)

    def test_expiry_version_freshness_and_sensor_skew(self):
        self.assertEqual(self.check(action=dict(self.move, issued_at=98.0)).code, "expired_action")
        self.assertEqual(self.check(action=dict(self.move, issued_at=101.0)).code, "future_action")
        self.assertEqual(self.check(action=dict(self.move, calibration_version="old-v1")).code,
                         "calibration_version_mismatch")
        self.assertEqual(self.check(obs=dict(self.obs, robot_state_at=98.0)).code, "stale_observation")
        self.assertEqual(self.check(obs=dict(self.obs, robot_state_at=99.5)).code, "sensor_skew")

    def test_ros_retimestamp_alone_does_not_prove_device_freshness(self):
        for field in ("device_freshness_verified", "right_binding_verified", "enabled"):
            for value in (False, None, 1, "true"):
                with self.subTest(field=field, value=value):
                    self.assertEqual(self.check(obs=dict(self.obs, **{field: value})).code,
                                     "device_state_unverified")
        for value in (None, False, True, 0.0, "0", 1):
            self.assertEqual(self.check(obs=dict(self.obs, fault=value)).code, "device_fault_or_unknown")

    def test_tcp_workspace_gripper_confirmation_cannot_be_truthy_strings(self):
        for field in ("tcp_confirmed", "gripper_confirmed", "workspace_confirmed"):
            self.assertFalse(self.check(policy=replace(self.policy, **{field: False})).accepted)
            self.assertFalse(self.check(policy=replace(self.policy, **{field: "true"})).accepted)

    def test_translation_orientation_speed_and_gripper_limits(self):
        for position, code in (([0.6, 0, 0.2], "workspace_limit"), ([0.3, 0, 0.2], "step_limit")):
            bad = dict(self.move, pose=dict(self.move["pose"], position_m=position))
            self.assertEqual(self.check(action=bad).code, code)
        q = [math.cos(0.2), 0, 0, math.sin(0.2)]
        self.assertEqual(self.check(action=dict(self.move, pose=dict(self.move["pose"], orientation_wxyz=q))).code,
                         "orientation_limit")
        self.assertEqual(self.check(action=dict(self.move, speed_m_s=0.1)).code, "speed_limit")
        gripper = {"type": "gripper", "width_m": 0.1, "issued_at": 99.9, "ttl_s": 1,
                   "calibration_version": "fresh-v2"}
        self.assertEqual(self.check(action=gripper).code, "gripper_limit")
        self.assertTrue(self.check(action=dict(gripper, width_m=0.05)).accepted)

    def test_observe_and_stop_work_with_default_closed_policy(self):
        for action in ({"type": "observe"}, {"type": "stop"}, {"type": "wait", "duration_s": 0.1}):
            self.assertTrue(validate_action(action, {}, Policy(), self.now).accepted)
        self.assertFalse(validate_action({"type": "wait", "duration_s": 999}, {}, Policy(), self.now).accepted)


if __name__ == "__main__":
    unittest.main()
