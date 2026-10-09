"""Offline single-action contracts; fake feedback is not physical validation.

TakeoverFixture blocks all real sockets, and ActionRobot sends only to fake CAN.
The explicit Piper model avoids silently testing the legacy Piper-X limits.
"""
import copy
import math
import unittest
from unittest.mock import patch

from robot_tools import arms, linear_hold, single_supervised_actions as actions
from robot_tools import supervised_actions, takeover
from test_backend import PROFILE
from test_execution import healthy_arm
from test_single_gripper_prepare import FakeGripper
from test_supervised_actions import ActionRobot
from test_takeover import TakeoverFixture


class SingleActionFixture(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        for config in self.profile["arms"].values():
            config["model"] = "piper"
        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
        self.limits = copy.deepcopy(ROBOT_JOINT_LIMIT_PRESET["piper"])
        for module in (actions, supervised_actions, linear_hold):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.make_robots()

    def make_robots(self, selected="right"):
        self.selected = selected
        self.passive = "left" if selected == "right" else "right"
        self.robots = {side: ActionRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.joints = {side: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
                       for side in takeover.SIDES}
        for side, robot in self.robots.items():
            # Manufacturer effectors share comm, not the blocked arm encoder.
            robot.gripper = FakeGripper(robot)
            robot.driver_enabled = [side == selected] * 6
            robot.gripper_enabled = side == selected
            robot.ctrl_mode = 1 if side == selected else 0
        self.target = self.robots[selected].motion.origin[:]
        self.target[2] += 0.006
        self.hook = None
        self.keep_inputs = False
        self.inputs = []

    def snapshot(self, robot, gripper):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        pose, motion = robot.motion.pose()
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, mode_feedback=robot.motion.mode,
            teach_status=0, motion_status=motion if robot.motion_flag is None else robot.motion_flag,
            arm_status=0, err_code=0))
        state["pose_m_rad"] = pose
        state["joints_rad"] = self.joints[robot.side][:]
        for index, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(index)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"]["width_m"] = robot.width
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        if self.keep_inputs:
            self.inputs.append((state, copy.deepcopy(state)))
        return state

    def run_tool(self, *, grip=False, width=0.012345, journal=None):
        journal = journal or (lambda event, data: self.events.append((event, data)))
        if grip:
            return actions.gripper_once(self.profile, journal, self.selected, width, 0.2)
        return actions.move_once(self.profile, journal, self.selected, self.target)

    def set_observed_boundary_pose(self):
        self.joints[self.selected][1:3] = [math.radians(-5.258), math.radians(2.616)]

    def assert_frame_ids(self, result, expected):
        self.assertEqual([frame.arbitration_id for frame in self.robots[self.selected].sent], expected)
        self.assertEqual(result["hardware_commands_sent"], len(expected))
        self.assertEqual(self.robots[self.passive].sent, [])
        self.assertEqual(result["passive_arm_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["joint_limits_changed"])
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["grasp_verified"])
        self.assertFalse(result["general_stop_validated"])

    def assert_recovery(self, result):
        self.assertTrue(result["boundary_recovery_required"])
        recovery = result["boundary_recovery"]
        nearest = recovery["nearest_legal_joints_rad"]
        expected = [max(low, min(high, value)) for index, value in enumerate(self.joints[self.selected], 1)
                    for low, high in [self.limits["joint%d" % index]]]
        self.assertEqual(nearest, expected)
        self.assertTrue(recovery["candidate_only"])
        self.assertFalse(recovery["automatic_dispatch"])
        self.assertFalse(recovery["existing_recovery_ready"])
        self.assertEqual(recovery["within_existing_recovery_step_limit"],
                         all(abs(a - b) <= 0.05 for a, b in zip(nearest, self.joints[self.selected])))


class SingleSupervisedActionsTests(SingleActionFixture):
    def test_legal_move_sends_one_four_frame_target_with_passive_disabled(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assert_frame_ids(result, [0x151, 0x152, 0x153, 0x154])
        self.assertEqual(bytes(self.robots[self.selected].sent[0].data),
                         bytes((1, 2, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(result["target_calls_sent"], 1)
        self.assertTrue(result["selected_arm_strictly_within_limits"])
        self.assertTrue(result["selected_arm_within_feedback_tolerance"])
        self.assertEqual(result["feedback_boundary_policy"]["observation_band_rad"], 0.003)
        self.assertEqual(result["feedback_boundary_policy"]["static_gripper_offset_cap_rad"], 0.1)
        self.assertTrue(result["feedback_boundary_policy"]["reference_scope"])
        self.assertTrue(result["controller_at_target"])
        self.assertTrue(result["observed_stable"])
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        self.assertGreaterEqual(result["observed_feedback_advances"], 20)
        self.assertTrue(all(value["physically_stopped"] is None
                            for value in result["cleanup"]["arms"].values()))

    def test_legal_left_jaw_sends_exactly_one_159(self):
        self.make_robots(selected="left")
        result = self.run_tool(grip=True)
        self.assertTrue(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertEqual(bytes(self.robots[self.selected].sent[0].data),
                         (12345).to_bytes(4, "big") + bytes((0, 200, 1, 0)))
        self.assertEqual(result["arm_target_commands_sent"], 0)
        self.assertEqual(result["mode_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertTrue(result["selected_arm_strictly_within_limits"])

    def test_static_j2_j3_discrepancy_allows_only_jaw_and_preserves_raw_inputs(self):
        self.set_observed_boundary_pose()
        original_joints, original_target = copy.deepcopy(self.joints), self.target[:]
        self.keep_inputs = True
        result = self.run_tool(grip=True)
        self.assertTrue(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertFalse(result["selected_arm_within_feedback_tolerance"])
        self.assertTrue(result["static_boundary_exception_accepted"])
        self.assertEqual(result["boundary_reference_joints_rad"], original_joints[self.selected])
        violations = result["observed_joint_limit_violations"][self.selected]
        self.assertEqual([item["joint_index"] for item in violations], [2, 3])
        self.assertEqual([item["observed_rad"] for item in violations], original_joints[self.selected][1:3])
        self.assertEqual(result["before"][self.selected]["joints_rad"], original_joints[self.selected])
        self.assertEqual(result["dispatch_feedback"][self.selected]["joints_rad"], original_joints[self.selected])
        self.assertEqual(self.joints, original_joints)
        self.assertEqual(self.target, original_target)
        for state, original in self.inputs:
            self.assertEqual(state, original)

    def test_same_static_discrepancy_move_is_zero_tx_with_nearest_recovery(self):
        self.set_observed_boundary_pose()
        original = copy.deepcopy(self.joints)
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assert_frame_ids(result, [])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertFalse(result["selected_arm_within_feedback_tolerance"])
        self.assert_recovery(result)
        self.assertEqual(result["boundary_recovery"]["nearest_legal_joints_rad"][1:3], [0.0, 0.0])
        self.assertEqual(self.joints, original)

    def test_move_accepts_exact_feedback_boundary_without_hiding_nominal_violation(self):
        for index, observed in ((1, -0.003), (2, 0.003)):
            with self.subTest(joint=index + 1):
                self.make_robots()
                self.joints[self.selected][index] = observed
                result = self.run_tool()
                self.assertTrue(result["ok"], result)
                self.assert_frame_ids(result, [0x151, 0x152, 0x153, 0x154])
                self.assertFalse(result["selected_arm_strictly_within_limits"])
                self.assertTrue(result["selected_arm_within_feedback_tolerance"])
                self.assertEqual(result["observed_joint_limit_violations"][self.selected][0]["observed_rad"], observed)

    def test_move_just_outside_feedback_boundary_is_zero_tx(self):
        for index, observed in ((1, -0.003001), (2, 0.003001)):
            with self.subTest(joint=index + 1):
                self.make_robots()
                self.joints[self.selected][index] = observed
                result = self.run_tool()
                self.assert_no_tx(result)
                self.assertFalse(result["selected_arm_within_feedback_tolerance"])
                self.assert_recovery(result)

    def test_jaw_feedback_tolerance_flag_distinguishes_static_allowance(self):
        for magnitude, within in ((0.003, True), (0.003001, False), (0.1, False)):
            with self.subTest(magnitude=magnitude):
                self.make_robots()
                self.joints[self.selected][1:3] = [-magnitude, magnitude]
                result = self.run_tool(grip=True)
                self.assertTrue(result["ok"], result)
                self.assert_frame_ids(result, [0x159])
                self.assertFalse(result["selected_arm_strictly_within_limits"])
                self.assertIs(result["selected_arm_within_feedback_tolerance"], within)

    def test_mixed_joint_observation_band_and_static_exception_allow_one_jaw_frame(self):
        self.joints[self.selected][0] = self.limits["joint1"][0] - 0.001
        self.joints[self.selected][1] = -0.05
        result = self.run_tool(grip=True)
        self.assertTrue(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertFalse(result["selected_arm_within_feedback_tolerance"])
        self.assertTrue(result["static_boundary_exception_accepted"])
        self.assertTrue(result["static_boundary_exception_ever_used"])
        self.assertEqual(result["static_boundary_exception_joint_indices"], [2])
        self.assertEqual([item["joint_index"] for item in
                          result["observed_joint_limit_violations"][self.selected]], [1, 2])

    def test_return_to_observation_band_preserves_static_exception_history(self):
        self.joints[self.selected][1] = -0.004
        def settle_inside_band(robot, state):
            if robot.side == self.selected and robot.sent:
                state["joints_rad"][1] = -0.002
        self.hook = settle_inside_band
        result = self.run_tool(grip=True)
        self.assertTrue(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertTrue(result["selected_arm_within_feedback_tolerance"])
        self.assertFalse(result["static_boundary_exception_accepted"])
        self.assertTrue(result["static_boundary_exception_ever_used"])
        self.assertEqual(result["static_boundary_exception_joint_indices"], [2])
        self.assertEqual(result["boundary_reference_joints_rad"][1], -0.004)
        self.assertEqual(result["observed_joint_limit_violations"][self.selected][0]["observed_rad"], -0.002)

    def test_jaw_refuses_unlisted_axes_opposite_directions_and_excess_magnitude(self):
        cases = [(index, boundary + sign * 0.01)
                 for index in (0, 3, 4, 5)
                 for boundary, sign in ((self.limits["joint%d" % (index + 1)][0], -1),
                                        (self.limits["joint%d" % (index + 1)][1], 1))]
        cases += [(1, self.limits["joint2"][1] + 0.01),
                  (2, self.limits["joint3"][0] - 0.01),
                  (1, -0.100001), (2, 0.100001)]
        for index, observed in cases:
            with self.subTest(joint=index + 1, observed=observed):
                self.make_robots()
                self.joints[self.selected][index] = observed
                result = self.run_tool(grip=True)
                self.assert_no_tx(result)
                self.assert_frame_ids(result, [])
                self.assertFalse(result["selected_arm_strictly_within_limits"])

    def test_static_boundary_allowance_does_not_relax_baseline_drift(self):
        self.set_observed_boundary_pose()
        start = self.clock.elapsed
        def drift(robot, state):
            if robot.side == self.selected and self.clock.elapsed > start:
                state["joints_rad"][1] += 0.003001
        self.hook = drift
        result = self.run_tool(grip=True)
        self.assert_no_tx(result)
        self.assertIn("envelope", result["errors"][0]["detail"])

    def test_drift_after_allowed_jaw_send_aborts_without_extra_frames(self):
        self.set_observed_boundary_pose()
        def drift(robot, state):
            if robot.side == self.selected and robot.sent:
                state["joints_rad"][1] += 0.003001
        self.hook = drift
        result = self.run_tool(grip=True)
        self.assertFalse(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertIn("envelope", result["errors"][0]["detail"])

    def test_fault_disabled_joint_and_stale_feedback_still_refuse(self):
        for defect in ("fault", "joint_disabled", "stale"):
            with self.subTest(defect=defect):
                self.make_robots()
                self.set_observed_boundary_pose()
                def corrupt(robot, state):
                    if robot.side != self.selected:
                        return
                    if defect == "fault":
                        state["drivers"]["2"]["foc_status"]["collision_status"] = True
                    elif defect == "joint_disabled":
                        state["drivers"]["2"]["foc_status"]["driver_enable_status"] = False
                    elif defect == "jaw_disabled":
                        state["gripper"]["foc_status"]["driver_enable_status"] = False
                    else:
                        state["fragment_timestamps_s"]["joint_34"] -= 0.101
                self.hook = corrupt
                self.assert_no_tx(self.run_tool(grip=True))

    def test_fault_after_jaw_dispatch_preserves_one_send_without_stop_claim(self):
        self.set_observed_boundary_pose()
        def fault(robot, state):
            if robot.side == self.selected and robot.sent:
                state["arm_status"]["arm_status"] = 4
        self.hook = fault
        result = self.run_tool(grip=True)
        self.assertFalse(result["ok"], result)
        self.assert_frame_ids(result, [0x159])
        self.assertIn("Unhealthy", result["errors"][0]["detail"])

    def test_boundary_crossing_after_dispatch_has_no_recovery_candidate(self):
        for grip in (False, True):
            with self.subTest(grip=grip):
                self.make_robots()
                self.joints[self.selected][1] = -0.09 if grip else -0.003
                original = self.joints[self.selected][:]
                def cross_boundary(robot, state):
                    if robot.side == self.selected and robot.sent:
                        state["joints_rad"][1] = -0.100001 if grip else -0.003001
                self.hook = cross_boundary
                result = self.run_tool(grip=grip)
                self.assertFalse(result["ok"], result)
                self.assert_frame_ids(result, [0x159] if grip else [0x151, 0x152, 0x153, 0x154])
                self.assertFalse(result["boundary_recovery_required"])
                self.assertIsNone(result["boundary_recovery"])
                self.assertEqual(result["boundary_reference_joints_rad"], original)
                self.assertFalse(result["selected_arm_within_feedback_tolerance"])

    def test_passive_arm_drift_remains_rejected(self):
        self.set_observed_boundary_pose()
        def drift(robot, state):
            if robot.side == self.passive and self.robots[self.selected].sent:
                state["joints_rad"][0] += 0.003001
        self.hook = drift
        result = self.run_tool(grip=True)
        self.assertFalse(result["ok"], result)
        self.assert_frame_ids(result, [0x159])

    def test_duplicate_jaw_send_is_blocked_after_one_frame(self):
        self.set_observed_boundary_pose()
        self.robots[self.selected].duplicate = True
        result = self.run_tool(grip=True)
        self.assertFalse(result["ok"], result)
        self.assert_frame_ids(result, [0x159])

    def test_corrupt_force_or_zero_byte_cannot_reach_bus(self):
        for index, value in ((5, 201), (7, 1)):
            with self.subTest(byte=index):
                self.make_robots()
                self.set_observed_boundary_pose()
                def corrupt(frame):
                    frame.data[index] = value
                    return frame
                self.robots[self.selected].frame_transform = corrupt
                self.assert_no_tx(self.run_tool(grip=True))


if __name__ == "__main__":
    unittest.main()
