"""Single empty-jaw preparation tests: no physical sockets or CAN buses."""
import copy
import errno
import math
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, single_gripper_prepare, takeover
from robot_tools.backend import PyAgxBackend
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture


class FakeGripper:
    def __init__(self, robot):
        self.robot = robot

    def _send_msg(self, frame):
        # Like the vendor effector: own encoder, shared comm; no arm encoder.
        self.robot.comm.send(frame)

    def move_gripper_m(self, *, value, force):
        frame = self.robot.can.Message(
            arbitration_id=0x159, is_extended_id=False,
            data=round(value * 1e6).to_bytes(4, "big", signed=True)
                 + round(force * 1e3).to_bytes(2, "big") + bytes((1, 0)))
        self._send_msg(self.robot.frame_transform(frame))
        if self.robot.duplicate:
            self._send_msg(frame)


class GripperRobot(FakeRobot):
    def __init__(self, side, selected):
        super().__init__(side)
        self.ctrl_mode = 1 if selected else 0
        self.driver_enabled = [selected] * 6
        self.gripper_enabled = False
        self.width = 0.0028 if selected else 0.01141
        self.gripper = FakeGripper(self)

    def _bus_send(self, frame):
        if self.send_error:
            raise self.send_error
        self.sent.append(frame)
        if self.accept:
            self.gripper_enabled = True


class SingleGripperFixture(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        self.make_robots()

    def make_robots(self, selected="right"):
        self.selected = selected
        self.robots = {side: GripperRobot(side, side == selected) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def snapshot(self, robot, gripper):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, mode_feedback=robot.mode_feedback,
            teach_status=0, motion_status=0, arm_status=0, err_code=0))
        for i, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"].update(width_m=robot.width)
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        # Current site's nominal-boundary violations must remain visible.
        state["joints_rad"][1:3] = [-0.026896, 0.062256]
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None, arm=None):
        return single_gripper_prepare.prepare_gripper(
            self.profile, journal or (lambda event, data: self.events.append((event, data))),
            self.selected if arm is None else arm)

    def assert_failure(self, result, sent_count):
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], sent_count)
        passive = "left" if self.selected == "right" else "right"
        self.assertEqual(self.robots[passive].sent, [])
        self.assertEqual(result["passive_arm_commands_sent"], 0)
        self.assertEqual(result["arm_target_commands_sent"], 0)
        self.assertEqual(result["mode_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["motion_gate_unlocked"])


class SingleGripperPrepareTests(SingleGripperFixture):
    def test_current_2_8mm_right_jaw_one_exact_frame_passive_disabled_left(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(result["gripper_enable_commands_sent"], 1)
        self.assertEqual(result["gripper_target_commands_sent"], 1)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["arm_target_commands_sent"], 0)
        self.assertEqual(self.robots["left"].sent, [])
        frame, = self.robots["right"].sent
        self.assertEqual(frame.arbitration_id, 0x159)
        self.assertEqual(bytes(frame.data).hex(), "00000af000c80100")
        self.assertEqual(result["gripper_enabled"], {"left": False, "right": True})
        self.assertEqual(result["passive_initial_enable_flags"], [False] * 7)
        self.assertGreaterEqual(self.clock.elapsed, 3.0)
        self.assertEqual([v["joint_index"] for v in result["observed_joint_limit_violations"]["right"]], [2, 3])
        self.assertFalse(result["joint_limits_changed"])
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["grasp_verified"])
        self.assertTrue(PyAgxBackend(PROFILE).commissioning_errors())

    def test_measured_width_boundaries_and_quantization_are_encoded(self):
        for width, raw in ((0.0, 0), (0.070, 70000), (0.0028004, 2800)):
            with self.subTest(width=width):
                self.make_robots()
                self.robots["right"].width = width
                result = self.run_tool()
                self.assertTrue(result["ok"], result)
                self.assertEqual(result["target_width_raw"], raw)
                self.assertEqual(bytes(self.robots["right"].sent[0].data),
                                 raw.to_bytes(4, "big") + bytes((0, 200, 1, 0)))
                self.assertEqual(self.robots["left"].sent, [])

    def test_raw_out_of_range_nonfinite_boolean_width_rejected_before_rounding(self):
        for width in (-1e-10, 0.0700000001, float("nan"), float("inf"), True):
            with self.subTest(width=width):
                self.make_robots()
                self.robots["right"].width = width
                self.assert_failure(self.run_tool(), 0)

    def test_left_selection_works_with_right_passive_mode2_and_mixed_enable_flags(self):
        self.make_robots(selected="left")
        self.robots["right"].ctrl_mode = 2
        self.robots["right"].driver_enabled = [True, False, True, False, True, False]
        self.robots["right"].gripper_enabled = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual(result["passive_initial_enable_flags"], [True, False, True, False, True, False, True])

    def test_passive_can_mode1_is_allowed_without_transmission(self):
        self.robots["left"].ctrl_mode = 1
        self.robots["left"].driver_enabled = [True] * 6
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.robots["left"].sent, [])

    def test_already_enabled_selected_jaw_is_read_only_after_baseline(self):
        self.robots["right"].gripper_enabled = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNone(result["target_width_raw"])
        self.assertGreaterEqual(self.clock.elapsed, 1.0)
        self.assertEqual(result["arms"]["right"]["status"], "already_enabled_observed")

    def test_selected_wrong_mode_or_disabled_joint_refuses_before_send(self):
        for mode, enabled in ((0, True), (2, True), (1, False)):
            with self.subTest(mode=mode, enabled=enabled):
                self.make_robots()
                self.robots["right"].ctrl_mode = mode
                self.robots["right"].driver_enabled[0] = enabled
                self.assert_failure(self.run_tool(), 0)

    def test_health_width_mode_unknown_flags_teaching_motion_and_fault_refused(self):
        cases = [
            ("arm_status", "teach_status", 1), ("arm_status", "motion_status", 1),
            ("arm_status", "mode_feedback", 3), ("arm_status", "arm_status", 4),
            ("arm_status", "ctrl_mode", True), ("gripper", "mode", "angle"),
        ]
        for side in takeover.SIDES:
            for container, field, value in cases:
                with self.subTest(side=side, field=field):
                    self.make_robots()
                    def hook(robot, state):
                        if robot.side == side:
                            state[container][field] = value
                    self.hook = hook
                    self.assert_failure(self.run_tool(), 0)
        for side in takeover.SIDES:
            for kind in ("unknown_joint_enable", "unknown_jaw_enable", "driver_fault", "jaw_fault"):
                with self.subTest(side=side, kind=kind):
                    self.make_robots()
                    def hook(robot, state):
                        if robot.side != side:
                            return
                        if kind == "unknown_joint_enable":
                            state["drivers"]["2"]["foc_status"]["driver_enable_status"] = 0
                        elif kind == "unknown_jaw_enable":
                            state["gripper"]["foc_status"]["driver_enable_status"] = None
                        elif kind == "driver_fault":
                            state["drivers"]["1"]["foc_status"]["collision_status"] = True
                        else:
                            state["gripper"]["foc_status"]["sensor_status"] = True
                    self.hook = hook
                    self.assert_failure(self.run_tool(), 0)

    def test_passive_mode_or_each_enable_flag_change_after_dispatch_aborts(self):
        for changed in range(8):
            with self.subTest(changed=changed):
                self.make_robots()
                def hook(robot, state):
                    if robot.side != "left" or not self.robots["right"].sent:
                        return
                    if changed < 6:
                        state["drivers"][str(changed + 1)]["foc_status"]["driver_enable_status"] = True
                    elif changed == 6:
                        state["gripper"]["foc_status"]["driver_enable_status"] = True
                    else:
                        state["arm_status"]["ctrl_mode"] = 1
                self.hook = hook
                self.assert_failure(self.run_tool(), 1)

    def test_selected_joint_disable_after_dispatch_aborts(self):
        def hook(robot, state):
            if robot.side == "right" and robot.sent:
                state["drivers"]["5"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        self.assert_failure(self.run_tool(), 1)

    def test_jaw_enable_cannot_happen_before_dispatch(self):
        def journal(event, data):
            if event == "single_gripper_prepare_intent":
                self.robots["right"].gripper_enabled = True
        self.assert_failure(self.run_tool(journal), 0)

    def test_enable_regression_after_first_confirmation_aborts(self):
        first = [True]
        def hook(robot, state):
            if robot.side == "right" and robot.sent:
                state["gripper"]["foc_status"]["driver_enable_status"] = first[0]
                first[0] = False
        self.hook = hook
        result = self.run_tool()
        self.assert_failure(result, 1)
        self.assertIn("regressed", result["errors"][0]["detail"])

    def test_initial_enabled_jaw_cannot_drop_during_readonly_baseline(self):
        self.robots["right"].gripper_enabled = True
        def hook(robot, state):
            if robot.side == "right" and self.clock.elapsed >= 0.1:
                state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        self.assert_failure(self.run_tool(), 0)

    def test_delayed_fresh_enable_can_succeed_but_absent_enable_never_retries(self):
        self.robots["right"].accept = False
        result = self.run_tool()
        self.assert_failure(result, 1)
        self.assertIn("did not confirm enabled", result["errors"][0]["detail"])
        self.make_robots()
        self.robots["right"].accept = False
        start = self.clock.elapsed
        def hook(robot, state):
            if robot.side == "right" and robot.sent and self.clock.elapsed - start > 1.4:
                state["gripper"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)

    def test_strict_joint_xyz_jaw_and_rotation_drift_on_either_arm(self):
        self.profile["gripper_prepare_stationary_joint_tolerance_rad"] = {"right": [0.008] * 6}
        for side in takeover.SIDES:
            for kind in ("joint", "xyz", "jaw", "rotation"):
                with self.subTest(side=side, kind=kind):
                    self.make_robots()
                    start = self.clock.elapsed
                    def hook(robot, state):
                        if robot.side == side and self.clock.elapsed - start >= 0.1:
                            if kind == "joint":
                                state["joints_rad"][3] += 0.004
                            elif kind == "xyz":
                                state["pose_m_rad"][0] += 0.0021
                            elif kind == "jaw":
                                state["gripper"]["width_m"] += 0.0021
                            else:
                                state["pose_m_rad"][3] += 0.0031
                    self.hook = hook
                    self.assert_failure(self.run_tool(), 0)

    def test_rpy_wraparound_is_not_false_drift(self):
        def hook(robot, state):
            state["pose_m_rad"][5] = math.pi - 0.0001 if self.clock.elapsed == 0 else -math.pi + 0.0001
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["drift"]["right"]["rotation_distance_rad"], 0.0002, places=9)

    def test_recorded_euler_amplification_preserves_small_physical_rotation(self):
        # Exact before/after feedback from refused-before-send record
        # single_gripper_prepare_4fd26895a66a46129ba7db7cf5129452.
        before = [0.042283, 0.031753, 0.162079,
                  -3.1111541114350123, 1.2710883876424304, -2.4606873525092454]
        after = [0.042304, 0.031725, 0.16208,
                 -3.1074016535432243, 1.2710534810573906, -2.4571443341276966]
        def hook(robot, state):
            if robot.side == "right":
                changed = self.clock.elapsed >= 0.1
                state["pose_m_rad"] = list(after if changed else before)
                state["joints_rad"][3] = -0.022375121010567305 if changed else -0.02125811028929093
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        drift = result["drift"]["right"]
        self.assertAlmostEqual(drift["pose_rpy_component_rad"], 0.003752457891788019)
        self.assertAlmostEqual(drift["rotation_distance_rad"], 0.001109292748, places=9)
        self.assertLess(drift["joint_rad"], 0.003)
        self.assertEqual(result["rotation_distance_limit_rad"], 0.003)
        self.assertTrue(result["pose_rpy_component_rad_diagnostic_only"])
        self.assertNotIn("additional_pose_rpy_component_limit_rad", result)

    def test_combined_rotation_above_limit_rejected_when_each_euler_change_is_small(self):
        def hook(robot, state):
            state["pose_m_rad"][3:] = [0, 1.27, 0]
            if self.clock.elapsed >= 0.1:
                state["pose_m_rad"][3:] = [0.002, 1.27, -0.002]
        self.hook = hook
        result = self.run_tool()
        self.assert_failure(result, 0)
        self.assertLess(result["drift"]["left"]["pose_rpy_component_rad"], 0.003)
        self.assertGreater(result["drift"]["left"]["rotation_distance_rad"], 0.003)
        self.assertIn("SO(3)", result["errors"][0]["detail"])

    def test_rotation_threshold_below_above_and_near_pi(self):
        for angle, allowed in ((0.002999, True), (0.003001, False), (math.pi - 1e-8, False)):
            with self.subTest(angle=angle):
                self.make_robots()
                start = self.clock.elapsed
                def hook(robot, state):
                    if self.clock.elapsed - start >= 0.1:
                        state["pose_m_rad"][5] = angle
                self.hook = hook
                result = self.run_tool()
                self.assertEqual(result["ok"], allowed, result)
                if allowed:
                    self.assertEqual(result["hardware_commands_sent"], 1)
                else:
                    self.assert_failure(result, 0)
                self.assertAlmostEqual(result["drift"]["left"]["rotation_distance_rad"], angle, places=7)

    def test_equivalent_pi_euler_branch_has_zero_rotation_distance(self):
        def hook(robot, state):
            if self.clock.elapsed >= 0.1:
                state["pose_m_rad"][3:] = [math.pi, math.pi, math.pi]
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["drift"]["right"]["rotation_distance_rad"], 0.0)
        self.assertAlmostEqual(result["drift"]["right"]["pose_rpy_component_rad"], math.pi)

    def test_true_rotation_drift_after_dispatch_aborts_without_another_frame(self):
        for side in takeover.SIDES:
            with self.subTest(side=side):
                self.make_robots()
                def hook(robot, state):
                    if robot.side == side and self.robots["right"].sent:
                        state["pose_m_rad"][3] = 0.0031
                self.hook = hook
                result = self.run_tool()
                self.assert_failure(result, 1)
                self.assertIn("SO(3)", result["errors"][0]["detail"])

    def test_changed_vendor_quaternion_convention_fails_before_transmission(self):
        from pyAgxArm.utiles import tf
        with patch.object(tf, "axes", "rzyx"):
            self.assert_failure(self.run_tool(), 0)

    def test_half_mm_presend_change_is_revalidated_after_journal(self):
        def journal(event, data):
            if event == "single_gripper_prepare_intent":
                self.robots["right"].width += 0.0006
        result = self.run_tool(journal)
        self.assert_failure(result, 0)
        self.assertIn("0.5 mm", result["errors"][0]["detail"])

    def test_one_mm_postsend_width_error_aborts_even_within_drift_envelope(self):
        def hook(robot, state):
            if robot.side == "right" and robot.sent:
                state["gripper"]["width_m"] += 0.0011
        self.hook = hook
        result = self.run_tool()
        self.assert_failure(result, 1)
        self.assertIn("1 mm", result["errors"][0]["detail"])

    def test_stale_or_skewed_or_missing_feedback_on_either_arm_refuses(self):
        for side in takeover.SIDES:
            for kind in ("stale", "skew", "missing"):
                with self.subTest(side=side, kind=kind):
                    self.make_robots()
                    def hook(robot, state):
                        if robot.side != side:
                            return
                        stamps = state["fragment_timestamps_s"]
                        if kind == "missing":
                            del stamps["gripper"]
                        else:
                            stamps["gripper"] -= 0.6 if kind == "stale" else 0.11
                    self.hook = hook
                    self.assert_failure(self.run_tool(), 0)

    def test_postsend_frozen_feedback_cannot_confirm_enable(self):
        frozen = {}
        def hook(robot, state):
            if self.robots["right"].sent:
                state["fragment_timestamps_s"] = dict(frozen[robot.side])
            else:
                frozen[robot.side] = dict(state["fragment_timestamps_s"])
        self.hook = hook
        self.assert_failure(self.run_tool(), 1)

    def test_read_journal_latency_cannot_make_old_feedback_fresh(self):
        def journal(event, data):
            if event == "feedback":
                self.clock.sleep(0.6)
        self.assert_failure(self.run_tool(journal), 0)

    def test_journal_failure_before_and_after_send_never_retries(self):
        for event_name, sent_count in (("operation_started", 0), ("single_gripper_prepare_intent", 0),
                                        ("single_gripper_frame_sent_unconfirmed", 1),
                                        ("single_gripper_enabled_at_observed_width", 1)):
            with self.subTest(event=event_name):
                self.make_robots()
                def journal(event, data):
                    if event == event_name:
                        raise OSError("mock disk full")
                self.assert_failure(self.run_tool(journal), sent_count)

    def test_duplicate_sdk_send_cannot_reach_bus_twice(self):
        self.robots["right"].duplicate = True
        result = self.run_tool()
        self.assert_failure(result, 1)
        self.assertTrue(result["guard_violations"])

    def test_swallowed_bus_failure_keeps_attempted_send_and_never_retries(self):
        self.robots["right"].send_error = OSError(errno.ENOBUFS, "mock queue full")
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assert_failure(result, 0)
        self.assertEqual(result["status"], "aborted_after_dispatch")
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 1)
        self.assertIn("CAN send failed", result["errors"][0]["detail"])

    def test_sdk_no_send_is_not_success(self):
        self.robots["right"].gripper.move_gripper_m = lambda **kwargs: None
        self.assert_failure(self.run_tool(), 0)

    def test_exact_payload_and_standard_frame_guard(self):
        for mutation in ("wrong_id", "force", "status", "zero", "extended", "remote", "fd", "brs", "short"):
            with self.subTest(mutation=mutation):
                self.make_robots()
                def transform(frame):
                    if mutation == "wrong_id":
                        frame.arbitration_id = 0x471
                    elif mutation in ("force", "status", "zero"):
                        index = {"force": 5, "status": 6, "zero": 7}[mutation]
                        frame.data[index] += 1
                    elif mutation == "extended":
                        frame.is_extended_id = True
                    elif mutation == "remote":
                        frame.is_remote_frame = True
                    elif mutation == "fd":
                        frame.is_fd = True
                    elif mutation == "brs":
                        frame.bitrate_switch = True
                    else:
                        frame.dlc = 7
                    return frame
                self.robots["right"].frame_transform = transform
                self.assert_failure(self.run_tool(), 0)

    def test_both_arm_senders_and_passive_gripper_sender_are_blocked(self):
        for side, kind in (("left", "arm"), ("right", "arm"), ("left", "jaw"), ("left", "bus")):
            with self.subTest(side=side, kind=kind):
                self.make_robots()
                def journal(event, data):
                    if event != "single_gripper_prepare_intent":
                        return
                    robot = self.robots[side]
                    if kind == "arm":
                        robot.set_motion_mode("p")
                    elif kind == "jaw":
                        robot.gripper.move_gripper_m(value=robot.width, force=0.2)
                    else:
                        robot.comm.send_bus.send(robot.can.Message(arbitration_id=0x159, data=[0] * 8))
                self.assert_failure(self.run_tool(journal), 0)

    def test_passive_transmission_attempt_during_active_ticket_is_blocked(self):
        original = self.robots["right"].gripper.move_gripper_m
        def command(**kwargs):
            original(**kwargs)
            self.robots["left"].comm.send_bus.send(
                self.robots["left"].can.Message(arbitration_id=0x159, data=[0] * 8))
        self.robots["right"].gripper.move_gripper_m = command
        self.assert_failure(self.run_tool(), 1)

    def test_initialization_tx_is_forbidden(self):
        self.robots["right"].connect = lambda: self.robots["right"].gripper.move_gripper_m(value=0.0028, force=0.2)
        self.assert_failure(self.run_tool(), 0)

    def test_other_sender_command_detected_before_send(self):
        def journal(event, data):
            if event == "single_gripper_prepare_intent":
                robot = self.robots["left"]
                robot.callback(robot.can.Message(arbitration_id=0x151, data=[0] * 8))
        self.assert_failure(self.run_tool(journal), 0)

    def test_cleanup_failure_does_not_claim_success_or_stop(self):
        self.robots["right"].disconnect.side_effect = OSError("mock disconnect failed")
        result = self.run_tool()
        self.assert_failure(result, 1)
        self.assertEqual(result["status"], "cleanup_failed")
        self.assertIsNone(result["cleanup"]["arms"]["right"]["physically_stopped"])

    def test_invalid_arm_journal_and_extra_target_arguments_rejected(self):
        for arm in (None, "", "both", "Right", ["right"], True):
            with self.subTest(arm=repr(arm)):
                with self.assertRaises(ValueError):
                    single_gripper_prepare.prepare_gripper(self.profile, lambda *args: None, arm)
        with self.assertRaises(TypeError):
            single_gripper_prepare.prepare_gripper(self.profile, None, "right")
        with self.assertRaises(TypeError):
            single_gripper_prepare.prepare_gripper(self.profile, lambda *args: None, "right", width_m=0.02)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()


class RealSDKSingleGripperTests(unittest.TestCase):
    def test_actual_vendor_encoder_on_fake_bus_only(self):
        self.run_vendor_case(send_error=False)

    def test_actual_vendor_swallowed_can_error_is_not_success(self):
        self.run_vendor_case(send_error=True)

    def run_vendor_case(self, *, send_error):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, robots = Clock(), [], {}

        class FakeBus:
            def __init__(self, channel):
                self.channel = channel

            def recv(self, timeout=None):
                time.sleep(0.002)
                return None

            def send(self, frame, timeout=None):
                if send_error:
                    raise OSError(errno.ENOBUFS, "mock queue full")
                sent.append((self.channel, frame))

            def shutdown(self):
                pass

        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        profile["arms"]["right"]["channel"] = "can1"
        original_factory = sdk.AgxArmFactory.create_arm

        def create_robot(config):
            robot = original_factory(config)
            robots[config["comm"]["can"]["channel"]] = robot
            return robot

        def snapshot(robot, gripper):
            channel = next(channel for channel, value in robots.items() if value is robot)
            selected = channel == "can1"
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1 if selected else 0, teach_status=0, mode_feedback=0,
                motion_status=0, arm_status=0, err_code=0))
            state["joints_rad"][1:3] = [-0.026896, 0.062256]
            for driver in state["drivers"].values():
                driver["foc_status"]["driver_enable_status"] = selected
            state["gripper"]["width_m"] = 0.0028 if selected else 0.01141
            state["gripper"]["foc_status"]["driver_enable_status"] = any(ch == channel for ch, _ in sent)
            return state

        with patch("socket.socket", side_effect=AssertionError("No real socket")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = single_gripper_prepare.prepare_gripper(profile, lambda event, data: None, "right")
        self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 0)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 1)
        if send_error:
            self.assertFalse(result["ok"], result)
            self.assertEqual(sent, [])
            self.assertIn("CAN send failed", result["errors"][0]["detail"])
        else:
            self.assertTrue(result["ok"], result)
            (channel, frame), = sent
            self.assertEqual(channel, "can1")
            self.assertEqual(frame.arbitration_id, 0x159)
            self.assertEqual(bytes(frame.data).hex(), "00000af000c80100")
            self.assertEqual(frame.dlc, 8)
            self.assertFalse(frame.is_extended_id)
            self.assertFalse(frame.is_fd)


if __name__ == "__main__":
    unittest.main()
