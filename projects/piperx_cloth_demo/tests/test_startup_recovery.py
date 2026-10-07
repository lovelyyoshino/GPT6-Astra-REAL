"""Offline startup-boundary recovery contracts; all physical sockets forbidden.

The fake feedback advances deterministically and does not model robot dynamics.
Manufacturer FK is used only for robot geometry, with no device connection.
"""
import copy
import math
import struct
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, bounded_joint_step, joint_recovery, startup_recovery, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_joint_recovery import Motion, RecoveryRobot, fk
from test_takeover import TakeoverFixture


Q = [math.radians(value) for value in (-16.433, -5.258, 2.616, -4.435, 19.821, -1.809)]
TARGET = [Q[0], 0.0, 0.0, *Q[3:]]


class StartupRecoveryFixture(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        for config in self.profile["arms"].values():
            config["model"] = "piper"
        from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET
        self.limits = copy.deepcopy(ROBOT_JOINT_LIMIT_PRESET["piper"])
        for module in (startup_recovery, bounded_joint_step, joint_recovery):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.make_robots()

    def make_robots(self, selected="left"):
        self.selected = selected
        self.passive = "right" if selected == "left" else "left"
        self.robots = {side: RecoveryRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        for side, robot in self.robots.items():
            robot.motion.origin = Q[:] if side == selected else [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
            robot.driver_enabled = [True] * 6
            robot.gripper_enabled = False
            robot.width = 0.025
        self.hook = None

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        q, motion = robot.motion.feedback()
        state = healthy_arm(self.clock.time())
        state["arm_status"].update(ctrl_mode=robot.ctrl_mode, teach_status=0,
                                  mode_feedback=robot.motion.mode, motion_status=motion)
        state["joints_rad"], state["pose_m_rad"] = q, fk(q)
        for index, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(index)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"]["width_m"] = robot.width
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, *, target=None, attachment=0.3, clearance=0.4, journal=None):
        return joint_recovery.recover_joint_boundary(
            self.profile, journal or (lambda event, data: self.events.append((event, data))),
            self.selected, TARGET[:] if target is None else target,
            recovery_profile="startup_j2_j3", attachment_radius_m=attachment,
            available_clearance_m=clearance)

    def assert_no_fallback(self, result, frame_count=4):
        self.assertEqual([frame.arbitration_id for frame in self.robots[self.selected].sent],
                         [0x151, 0x155, 0x156, 0x157][:frame_count])
        self.assertEqual(self.robots[self.passive].sent, [])
        self.assertEqual(result["hardware_commands_sent"], frame_count)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["general_stop_validated"])
        self.assertFalse(result["joint_zero_calibrated"])
        self.assertFalse(result["limits_changed"])


class StartupRecoveryTests(StartupRecoveryFixture):
    def test_current_nonzero_left_pose_with_both_jaws_disabled_uses_four_frames(self):
        original = self.robots[self.selected].motion.origin[:]
        target = TARGET[:]
        result = self.run_tool(target=target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertTrue(result["recovery_observed"])
        self.assert_no_fallback(result)
        self.assertEqual(self.robots[self.selected].move_calls, 1)
        self.assertEqual(bytes(self.robots[self.selected].sent[0].data), bytes((1, 1, 1, 0, 0, 0, 0, 0)))
        raw = [round(math.degrees(value) * 1000) for value in target]
        for index, frame in enumerate(self.robots[self.selected].sent[1:]):
            self.assertEqual(bytes(frame.data), struct.pack(">ii", *raw[2 * index:2 * index + 2]))
        self.assertEqual(target, TARGET)
        self.assertEqual(self.robots[self.selected].motion.origin, original)
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        self.assertGreaterEqual(result["stable_duration_s"], 3)
        self.assertGreaterEqual(result["stable_feedback_advances"], 20)
        self.assertTrue(result["selected_arm_strictly_within_limits"])
        self.assertLessEqual(result["target_flange_displacement_m"], 0.015)
        for side in takeover.SIDES:
            self.assertFalse(result["before"][side]["gripper"]["foc_status"]["driver_enable_status"])
            self.assertIsNone(result["cleanup"]["arms"][side]["physically_stopped"])

    def test_known_mixed_jaw_enable_states_and_zero_width_are_preserved(self):
        self.robots[self.passive].gripper_enabled = True
        self.robots[self.selected].width = 0.0
        result = self.run_tool()
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)
        self.assertFalse(result["after"][self.selected]["gripper"]["foc_status"]["driver_enable_status"])
        self.assertTrue(result["after"][self.passive]["gripper"]["foc_status"]["driver_enable_status"])
        self.assertEqual(result["after"][self.selected]["gripper"]["width_m"], 0.0)

    def test_standard_default_retains_fifty_milliradian_limit(self):
        for robot in self.robots.values():
            robot.gripper_enabled = True
        result = joint_recovery.recover_joint_boundary(
            self.profile, lambda *args: None, self.selected, TARGET[:])
        self.assert_no_tx(result)
        self.assertEqual(result["recovery_limits"]["joint_excursion_rad"], 0.05)
        self.assertIn("0.05", result["errors"][0]["detail"])

    def test_startup_cap_accepts_point_one_and_refuses_larger_start(self):
        self.robots[self.selected].motion.origin[1:3] = [-0.1, 0.05]
        result = self.run_tool()
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)
        self.make_robots()
        self.robots[self.selected].motion.origin[1] = -0.100001
        self.assert_no_tx(self.run_tool())

    def test_only_original_j2_lower_and_j3_upper_violations_can_recover(self):
        cases = [(0, self.limits["joint1"][0] - 0.01),
                 (1, self.limits["joint2"][1] + 0.01),
                 (2, self.limits["joint3"][0] - 0.01),
                 (3, self.limits["joint4"][1] + 0.01)]
        for index, value in cases:
            with self.subTest(joint=index + 1):
                self.make_robots()
                self.robots[self.selected].motion.origin[index] = value
                target = TARGET[:]
                low, high = self.limits["joint%d" % (index + 1)]
                target[index] = max(low, min(high, value))
                self.assert_no_tx(self.run_tool(target=target))

    def test_no_violation_is_not_general_movement_entry(self):
        self.robots[self.selected].motion.origin = TARGET[:]
        self.assert_no_tx(self.run_tool())

    def test_recovery_axes_require_nearest_exact_boundary(self):
        for index, value in ((1, 0.001), (2, -0.001), (1, -0.001), (2, 0.001)):
            with self.subTest(joint=index + 1, value=value):
                self.make_robots()
                target = TARGET[:]
                target[index] = value
                self.assert_no_tx(self.run_tool(target=target))

    def test_nonrecovery_axis_small_feedback_difference_is_counted_in_sweep(self):
        target = TARGET[:]
        target[0] += 0.001
        result = self.run_tool(target=target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)
        encoded = result["encoded_target_joints_rad"][0]
        expected = abs(encoded - Q[0]) + 0.003 + abs(target[0] - encoded)
        self.assertAlmostEqual(result["sweep_axis_excursions_rad"][0], expected)
        self.assertAlmostEqual(result["target_quantization_error_rad"][0], abs(target[0] - encoded))
        self.assertEqual(result["sweep_reference_joints_rad"], Q)

    def test_encoded_nonrecovery_axis_cannot_round_outside_feedback_tolerance(self):
        self.robots[self.selected].motion.origin[3] = 0.0
        target = TARGET[:]
        target[3] = 0.003
        encoded = math.radians(round(math.degrees(target[3]) * 1000) / 1000)
        self.assertLessEqual(abs(target[3]), 0.003)
        self.assertGreater(encoded, 0.003)
        self.assertAlmostEqual(encoded, 0.003001966313430247)
        result = self.run_tool(target=target)
        self.assert_no_tx(result)
        self.assertIn("0.003", result["errors"][0]["detail"])

    def test_inward_quantization_sweep_still_encloses_caller_monitor_interval(self):
        self.robots[self.selected].motion.origin[3] = 0.0
        target = TARGET[:]
        target[3] = 0.001002
        result = self.run_tool(target=target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)
        encoded = result["encoded_target_joints_rad"][3]
        self.assertLess(encoded, target[3])
        self.assertGreater(result["target_quantization_error_rad"][3], 0)
        self.assertAlmostEqual(result["target_quantization_error_rad"][3], target[3] - encoded)
        self.assertGreaterEqual(result["sweep_axis_excursions_rad"][3], target[3] + 0.003)
        self.assertEqual(result["target_joints_rad"], target)

    def test_nonrecovery_axis_larger_difference_or_fresh_disagreement_refused(self):
        target = TARGET[:]
        target[0] += 0.0031
        self.assert_no_tx(self.run_tool(target=target))
        self.make_robots()
        target = TARGET[:]
        target[0] -= 0.002
        def journal(event, data):
            if event == "joint_recovery_intent":
                self.robots[self.selected].motion.origin[0] += 0.002
        self.assert_no_tx(self.run_tool(target=target, journal=journal))

    def test_unchanged_axis_quantization_must_still_be_legal(self):
        self.robots[self.selected].motion.origin[4] = self.limits["joint5"][1] - 1e-9
        target = TARGET[:]
        target[4] = self.robots[self.selected].motion.origin[4]
        self.assert_no_tx(self.run_tool(target=target))

    def test_relative_clearance_requires_twice_all_axis_sweep_plus_reserve(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result.get("errors"))
        radii = result["sweep_axis_radii_m"]
        excursions = result["sweep_axis_excursions_rad"]
        self.assertEqual(len(radii), 6)
        self.assertEqual(len(excursions), 6)
        self.assertTrue(all(radius > 0.3 for radius in radii))
        self.assertTrue(all(excursion >= 0.003 for excursion in excursions))
        sweep = sum(radius * excursion for radius, excursion in zip(radii, excursions))
        self.assertLess(2 * sweep + 0.005, 0.4)
        self.make_robots()
        # Enough for a single-entity sweep, insufficient for relative surfaces.
        result = self.run_tool(clearance=sweep + 0.006)
        self.assert_no_tx(result)
        self.assertIn("clearance", result["errors"][0]["detail"].lower())

    def test_larger_passive_sweep_requires_its_own_doubled_relative_budget(self):
        original = startup_recovery.tracking_sweep
        def larger_passive_envelope(model, origin, target, attachment, tolerances, body):
            envelope = original(model, origin, target, attachment, tolerances, body)
            if origin == target:
                # Isolate a larger passive geometry without changing its drift.
                scale = 0.3 / envelope["sweep_bound_m"]
                envelope["sweep_axis_radii_m"] = [radius * scale for radius in envelope["sweep_axis_radii_m"]]
                envelope["sweep_bound_m"] = 0.3
            return envelope
        with patch.object(startup_recovery, "tracking_sweep", side_effect=larger_passive_envelope):
            result = self.run_tool(clearance=0.505)
        self.assert_no_tx(result)
        self.assertEqual(result["passive_sweep_bound_m"], 0.3)
        self.assertAlmostEqual(result["relative_sweep_bound_m"], 0.6)
        self.assertLess(2 * result["sweep_bound_m"], 0.5)
        self.assertLess(result["sweep_bound_m"] + result["passive_sweep_bound_m"], 0.5)
        self.assertIn("clearance", result["errors"][0]["detail"].lower())

    def test_invalid_or_missing_corridor_parameters_never_create_robot(self):
        for attachment, clearance in ((None, None), (None, 0.4), (0.3, None),
                                      (0.0, 0.4), (0.3, 0.0), (-0.1, 0.4),
                                      (True, 0.4), (0.3, False), (math.inf, 0.4),
                                      (0.3, math.nan)):
            with self.subTest(attachment=attachment, clearance=clearance):
                with self.assertRaises(ValueError):
                    self.run_tool(attachment=attachment, clearance=clearance)
        with self.assertRaises(ValueError):
            joint_recovery.recover_joint_boundary(
                self.profile, lambda *args: None, self.selected, TARGET[:],
                recovery_profile="startup_j2_j3")
        with self.assertRaises(ValueError):
            joint_recovery.recover_joint_boundary(
                self.profile, lambda *args: None, self.selected, TARGET[:],
                recovery_profile="unknown", attachment_radius_m=0.3, available_clearance_m=0.4)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_fk_disagreement_and_endpoint_larger_than_fifteen_mm_refuse(self):
        def mismatched_fk(q):
            pose = fk(q)
            pose[0] += 0.003
            return pose
        self.robots[self.selected].fk = mismatched_fk
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("FK", result["errors"][0]["detail"])
        self.make_robots()
        self.robots[self.selected].motion.origin[1:3] = [-0.001, 0.05]
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("15 mm", result["errors"][0]["detail"])

    def test_slow_fk_computation_cannot_dispatch_stale_feedback(self):
        def slow_fk(q):
            pose = fk(q)
            self.clock.sleep(0.051)
            return pose
        self.robots[self.selected].fk = slow_fk
        self.assert_no_tx(self.run_tool())

    def test_baseline_health_enables_jaw_range_motion_and_age_remain_required(self):
        cases = ("fault", "joint_disabled", "jaw_unknown", "jaw_range",
                 "jaw_changed", "stale", "passive_pending", "joint_drift", "jaw_drift")
        for condition in cases:
            with self.subTest(condition=condition):
                self.make_robots()
                start = self.clock.elapsed
                def corrupt(robot, state):
                    if condition == "fault":
                        state["drivers"]["2"]["foc_status"]["driver_error_status"] = True
                    elif condition == "joint_disabled":
                        state["drivers"]["2"]["foc_status"]["driver_enable_status"] = False
                    elif condition == "jaw_unknown":
                        state["gripper"]["foc_status"]["driver_enable_status"] = None
                    elif condition == "jaw_range":
                        state["gripper"]["width_m"] = 0.070001
                    elif condition == "jaw_changed" and self.clock.elapsed > start:
                        state["gripper"]["foc_status"]["driver_enable_status"] = True
                    elif condition == "stale":
                        state["fragment_timestamps_s"]["joint_34"] -= 0.051
                    elif condition == "passive_pending" and robot.side == self.passive:
                        state["arm_status"]["motion_status"] = 1
                    elif condition == "joint_drift" and self.clock.elapsed > start:
                        state["joints_rad"][3] += 0.0031
                    elif condition == "jaw_drift" and self.clock.elapsed > start:
                        state["gripper"]["width_m"] += 0.0021
                self.hook = corrupt
                self.assert_no_tx(self.run_tool())

    def test_all_fragments_must_advance_during_baseline(self):
        first = self.clock.time()
        def frozen(robot, state):
            state["fragment_timestamps_s"]["joint_56"] = first
        self.hook = frozen
        self.assert_no_tx(self.run_tool())

    def test_passive_joint_drives_must_also_remain_enabled(self):
        self.robots[self.passive].driver_enabled[2] = False
        self.assert_no_tx(self.run_tool())

    def test_active_fault_or_state_drift_sends_no_fallback(self):
        for condition in ("fault", "selected_fixed_axis", "passive_joint", "jaw_enable",
                          "jaw_width", "stale"):
            with self.subTest(condition=condition):
                self.make_robots()
                def corrupt(robot, state):
                    if self.robots[self.selected].motion.started is None:
                        return
                    if condition == "fault":
                        state["drivers"]["3"]["foc_status"]["collision_status"] = True
                    elif condition == "selected_fixed_axis" and robot.side == self.selected:
                        state["joints_rad"][3] += 0.0031
                    elif condition == "passive_joint" and robot.side == self.passive:
                        state["joints_rad"][3] += 0.0031
                    elif condition == "jaw_enable":
                        state["gripper"]["foc_status"]["driver_enable_status"] = True
                    elif condition == "jaw_width":
                        state["gripper"]["width_m"] += 0.0021
                    elif condition == "stale":
                        state["fragment_timestamps_s"]["joint_34"] -= 0.051
                self.hook = corrupt
                result = self.run_tool()
                self.assertFalse(result["ok"], result.get("errors"))
                self.assertFalse(result["recovery_observed"])
                self.assert_no_fallback(result)

    def test_feedback_tolerance_does_not_claim_strict_nominal_membership(self):
        def residual(robot, state):
            if (robot.side == self.selected and robot.motion.started is not None
                    and state["arm_status"]["motion_status"] == 0):
                state["joints_rad"][1] = -0.001
                state["pose_m_rad"] = fk(state["joints_rad"])
        self.hook = residual
        result = self.run_tool()
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)
        self.assertTrue(result["recovery_observed"])
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertEqual(result["final_boundary_violations"][self.selected][0]["observed_rad"], -0.001)

    def test_right_j4_keeps_three_milliradian_tracking_tolerance(self):
        self.make_robots(selected="right")
        def drift(robot, state):
            if robot.side == self.selected and robot.motion.started is not None:
                state["joints_rad"][3] += 0.0031
        self.hook = drift
        result = self.run_tool()
        self.assertFalse(result["ok"], result.get("errors"))
        self.assert_no_fallback(result)

    def test_post_send_orientation_only_feedback_jump_on_either_arm_is_failure(self):
        for side in takeover.SIDES:
            with self.subTest(side=side):
                self.make_robots()
                def orientation_jump(robot, state):
                    if robot.side == side and self.robots[self.selected].motion.started is not None:
                        state["pose_m_rad"][3] += 0.1
                self.hook = orientation_jump
                result = self.run_tool()
                self.assertFalse(result["ok"], result.get("errors"))
                self.assertFalse(result["recovery_observed"])
                self.assert_no_fallback(result)
                self.assertEqual(self.robots[self.selected].move_calls, 1)
                self.assertIn("FK", result["errors"][0]["detail"])

    def test_cached_mode_target_excursion_is_reported_after_contiguous_dispatch(self):
        robot = self.robots[self.selected]
        original_send = robot.comm.send_bus.send
        def activate_cached_target(frame, *args, **kwargs):
            original_send(frame, *args, **kwargs)
            if frame.arbitration_id == 0x151:
                robot.motion.origin[3] += 0.1
        robot.comm.send_bus.send = activate_cached_target
        result = self.run_tool()
        self.assertFalse(result["ok"], result.get("errors"))
        # There is no feedback interlock between this SDK call's four frames.
        # This proves later rejection, not cancellation or physical stopping.
        self.assert_no_fallback(result)
        self.assertEqual(robot.move_calls, 1)
        self.assertFalse(result["recovery_observed"])
        self.assertFalse(result["hard_path_guarantee"])
        self.assertIsNone(result["cleanup"]["arms"][self.selected]["physically_stopped"])
        self.assertIn("envelope", result["errors"][0]["detail"])

    def test_partial_or_failed_or_duplicate_sequence_never_retries(self):
        for condition, expected in (("partial", 3), ("swallowed_error", 2), ("duplicate", 1)):
            with self.subTest(condition=condition):
                self.make_robots()
                robot = self.robots[self.selected]
                if condition == "partial":
                    robot.partial = True
                elif condition == "swallowed_error":
                    robot.fail_id, robot.swallow_error = 0x156, True
                else:
                    robot.duplicate = True
                result = self.run_tool()
                self.assertFalse(result["ok"], result.get("errors"))
                self.assert_no_fallback(result, expected)
                self.assertEqual(robot.move_calls, 1)

    def test_no_arrival_times_out_without_second_target(self):
        self.robots[self.selected].motion.accept = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result.get("errors"))
        self.assertFalse(result["recovery_observed"])
        self.assert_no_fallback(result)
        self.assertEqual(self.robots[self.selected].move_calls, 1)


class RealSDKStartupRecoveryTests(unittest.TestCase):
    def test_real_vendor_move_j_with_disabled_jaws_and_fake_can_only(self):
        with patch("socket.socket", side_effect=AssertionError("Real sockets forbidden")):
            sdk = arms._load_sdk(PROFILE["sdk_path"])
            import can
            from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
            profile = copy.deepcopy(PROFILE)
            for config in profile["arms"].values():
                config["model"] = "piper"
            selected_channel = profile["arms"]["left"]["channel"]
            clock, sent, channels = Clock(), [], {}
            motions = {config["channel"]: Motion(clock) for config in profile["arms"].values()}
            for channel, motion in motions.items():
                motion.origin = Q[:] if channel == selected_channel else [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]

            class FakeBus:
                def __init__(self, channel):
                    self.channel = channel

                def recv(self, timeout=None):
                    time.sleep(0.001)
                    return None

                def send(self, frame, timeout=None):
                    sent.append((self.channel, frame))
                    motions[self.channel].sent(frame)

                def shutdown(self):
                    pass

            original_factory = sdk.AgxArmFactory.create_arm
            def create_robot(config):
                robot = original_factory(config)
                channels[id(robot)] = config["comm"]["can"]["channel"]
                return robot

            def snapshot(robot, gripper):
                motion = motions[channels[id(robot)]]
                q, moving = motion.feedback()
                state = healthy_arm(clock.time())
                state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                    ctrl_mode=1, teach_status=0, mode_feedback=motion.mode,
                    motion_status=moving, arm_status=0, err_code=0))
                state["joints_rad"], state["pose_m_rad"] = q, robot.fk(q)
                state["gripper"]["width_m"] = 0.025
                state["gripper"]["foc_status"]["driver_enable_status"] = False
                return state

            with patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
                 patch.object(takeover, "time", clock), patch.object(joint_recovery, "time", clock), \
                 patch.object(bounded_joint_step, "time", clock), patch.object(startup_recovery, "time", clock), \
                 patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
                 patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
                 patch.object(arms, "snapshot", side_effect=snapshot):
                result = joint_recovery.recover_joint_boundary(
                    profile, lambda *args: None, "left", TARGET[:],
                    recovery_profile="startup_j2_j3", attachment_radius_m=0.3, available_clearance_m=0.4)

        self.assertTrue(result["ok"], result.get("errors"))
        self.assertTrue(result["recovery_observed"])
        self.assertEqual([channel for channel, frame in sent], [selected_channel] * 4)
        self.assertEqual([frame.arbitration_id for channel, frame in sent], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(bytes(sent[0][1].data), bytes((1, 1, 1, 0, 0, 0, 0, 0)))
        raw = [round(math.degrees(value) * 1000) for value in TARGET]
        for index, (channel, frame) in enumerate(sent[1:]):
            self.assertEqual(bytes(frame.data), struct.pack(">ii", *raw[2 * index:2 * index + 2]))
            self.assertFalse(frame.is_extended_id)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["general_stop_validated"])


if __name__ == "__main__":
    unittest.main()
