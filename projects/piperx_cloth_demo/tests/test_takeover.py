"""CAN takeover contract tests. Every socket and CAN bus is mocked."""
import copy
import errno
import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms, takeover
from robot_tools.backend import PyAgxBackend
from test_backend import PROFILE
from test_execution import Clock, healthy_arm


class FakeRobot:
    def __init__(self, side):
        import can
        self.side = side
        self.ctrl_mode = 2
        self.mode_feedback = 0
        self.frame_transform = lambda frame: frame
        self.send_error = None
        self.swallow_error = False
        self.duplicate = False
        self.accept = True
        self.sent = []
        self.disconnect = Mock()
        self.callback = None
        self._msg_mode = NS(ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=50,
                            mit_mode=0, installation_pos=0, residence_time=0)
        self.comm = NS(send=self._comm_send, send_bus=NS(send=self._bus_send),
                       get_callback=lambda: None, set_callback=self._set_callback, last_error=None)
        self.gripper = NS(_send_msg=self._send_msg)
        self.can = can

    def _set_callback(self, callback):
        self.callback = callback

    def _send_msg(self, frame):
        self.comm.send(frame)

    def _send_msgs(self, frames):
        for frame in frames:
            self._send_msg(frame)

    def _comm_send(self, frame):
        try:
            self.comm.send_bus.send(frame)
        except Exception as exc:
            self.comm.last_error = exc
            if not self.swallow_error:
                raise
            self.comm.last_error = None  # Simulated RX clearing shared SDK error.

    def _bus_send(self, frame):
        if self.send_error:
            raise self.send_error
        self.sent.append(frame)
        if self.accept:
            self.ctrl_mode = 1

    def init_effector(self, kind):
        return self.gripper

    def create_comm(self):
        return self.comm

    def connect(self):
        pass

    def set_motion_mode(self, mode):
        selected = {"p": 0, "j": 1, "l": 2}[mode]
        self._msg_mode.move_mode = selected
        self._msg_mode.mit_mode = 0
        frame = self.can.Message(arbitration_id=0x151, is_extended_id=False,
                                 data=[self._msg_mode.ctrl_mode, selected,
                                       self._msg_mode.move_spd_rate_ctrl, 0, 0, 0, 0, 0])
        self._send_msg(self.frame_transform(frame))
        if self.duplicate:
            self._send_msg(frame)


class TakeoverFixture(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch("socket.socket", side_effect=AssertionError("No real socket allowed")))
        arms._load_sdk(PROFILE["sdk_path"])  # Import vendor enum definitions only.
        self.clock = Clock()
        self.clock.monotonic = lambda: self.clock.elapsed
        self.stack.enter_context(patch.object(takeover, "time", self.clock))
        self.stack.enter_context(patch.object(arms, "time", self.clock))
        self.stack.enter_context(patch.object(arms, "_preflight"))
        self.robots = {side: FakeRobot(side) for side in takeover.SIDES}
        self.sdk = NS(__version__="fake", create_agx_arm_config=lambda **kwargs: kwargs,
                      AgxArmFactory=NS(create_arm=Mock(side_effect=self.robots.values())))
        self.stack.enter_context(patch.object(arms, "_load_sdk", return_value=self.sdk))
        self.hook = None
        self.snapshot_count = 0
        self.stack.enter_context(patch.object(arms, "snapshot", side_effect=self.snapshot))
        self.events = []

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        result = healthy_arm(self.clock.time())
        result["arm_status"].update(ctrl_mode=robot.ctrl_mode, teach_status=0,
                                    mode_feedback=robot.mode_feedback)
        if self.hook:
            self.hook(robot, result)
        return result

    def run_tool(self, journal=None):
        return takeover.request_can_control(PROFILE, journal or (lambda event, data: self.events.append((event, data))))

    def assert_no_tx(self, result):
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        for robot in self.robots.values():
            self.assertEqual(robot.sent, [])
            robot.disconnect.assert_called_once()


class TakeoverTests(TakeoverFixture):
    def test_exact_one_mode_frame_per_side_and_stationary_windows(self):
        self.robots["right"].mode_feedback = 2
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertGreaterEqual(self.clock.elapsed, 5)
        for side, mode in (("left", 0), ("right", 2)):
            frames = self.robots[side].sent
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0].arbitration_id, 0x151)
            self.assertEqual(bytes(frames[0].data), bytes((1, mode, 1, 0, 0, 0, 0, 0)))
            self.assertFalse(frames[0].is_extended_id)
            self.assertIsNone(result["cleanup"]["arms"][side]["physically_stopped"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertTrue(result["hold_not_validated"])
        self.assertTrue(PyAgxBackend(PROFILE).commissioning_errors())
        self.assertEqual(result["target_commands_sent"], 0)
        requested = [data["side"] for event, data in self.events if event == "mode_request_intent"]
        self.assertEqual(requested, ["left", "right"])

    def test_already_can_control_is_verified_noop(self):
        for robot in self.robots.values():
            robot.ctrl_mode = 1
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertGreaterEqual(self.clock.elapsed, 1)
        self.assertTrue(all(v["status"] == "already_can_control" for v in result["arms"].values()))

    def test_vendor_status_and_cache_enums_accepted_without_coercing_values(self):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.transmit.arm_mode_ctrl import ArmMsgModeCtrl
        enums = ArmMsgModeCtrl.Enums
        for robot in self.robots.values():
            robot._msg_mode.ctrl_mode = enums.CtrlMode.CAN_CTRL
            robot._msg_mode.mit_mode = enums.MitMode.POS_VEL
            robot._msg_mode.installation_pos = enums.InstallationPos.INVALID
        self.robots["right"].mode_feedback = 2
        def hook(robot, state):
            # This reproduces the actual SDK constructor -> snapshot._plain
            # chain: _plain preserves IntEnum instances instead of raw ints.
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=robot.ctrl_mode, mode_feedback=robot.mode_feedback,
                teach_status=0, motion_status=0, arm_status=0, err_code=0))
            self.assertIsNot(type(state["arm_status"]["teach_status"]), int)
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(bytes(self.robots["right"].sent[0].data), bytes((1, 2, 1, 0, 0, 0, 0, 0)))

    def test_boolean_float_and_unknown_vendor_status_values_still_refused(self):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatusEnum as Enums
        for key, unknown in (("teach_status", Enums.TeachingState.UNKNOWN),
                             ("motion_status", Enums.MotionStatus.UNKNOWN),
                             ("mode_feedback", Enums.ModeFeedback.UNKNOWN),
                             ("ctrl_mode", Enums.CtrlMode.UNKNOWN)):
            for value in (False, True, 0.0, 1.0, unknown):
                with self.subTest(field=key, value=repr(value)):
                    state = {side: healthy_arm(self.clock.time()) for side in takeover.SIDES}
                    for arm in state.values():
                        arm["arm_status"].update(teach_status=0, mode_feedback=0)
                    state["left"]["arm_status"][key] = value
                    operation = takeover._Takeover(PROFILE, lambda event, data: None)
                    with self.assertRaises(RuntimeError):
                        operation.checked(state)
                    self.assertEqual(operation.counts["left"]["attempted_frames"], 0)

    def test_boolean_float_and_unknown_cache_values_still_refused(self):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.transmit.arm_mode_ctrl import ArmMsgModeCtrl
        unknown = ArmMsgModeCtrl.Enums.CtrlMode.UNKNOWN
        for field, valid in (("ctrl_mode", 1), ("mit_mode", 0),
                             ("installation_pos", 0), ("residence_time", 0)):
            for value in (bool(valid), float(valid), unknown):
                with self.subTest(field=field, value=repr(value)):
                    self.robots = {side: FakeRobot(side) for side in takeover.SIDES}
                    self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                    setattr(self.robots["left"]._msg_mode, field, value)
                    result = self.run_tool()
                    self.assert_no_tx(result)
                    self.assertIn("Unexpected SDK mode cache field: " + field, result["errors"][0]["detail"])

    def test_active_teaching_refused(self):
        self.hook = lambda robot, state: state["arm_status"].update(teach_status=1)
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("teach_status", result["errors"][0]["detail"])

    def test_motion_status_driver_disabled_or_fault_refused(self):
        for fault in ("moving", "disabled", "fault"):
            with self.subTest(fault=fault):
                def hook(robot, state):
                    if fault == "moving":
                        state["arm_status"]["motion_status"] = 1
                    else:
                        flags = state["drivers"]["1"]["foc_status"]
                        flags["driver_enable_status" if fault == "disabled" else "collision_status"] = fault == "fault"
                self.hook = hook
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["hardware_commands_sent"], 0)
                # Factory return order must be renewed for another independent request.
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_joint_drift_during_initial_window_refused(self):
        def hook(robot, state):
            state["joints_rad"][0] = self.clock.elapsed * 0.01
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("drift", result["errors"][0]["detail"])

    def test_stale_fragment_refused(self):
        def hook(robot, state):
            state["fragment_timestamps_s"]["joint_34"] -= 1
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("stale_feedback", result["errors"][0]["detail"])

    def test_nonadvancing_fragment_rejected_during_baseline(self):
        original = self.clock.time()
        def hook(robot, state):
            state["fragment_timestamps_s"]["gripper"] = original
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_incomplete_warmup_bounded(self):
        self.hook = lambda robot, state: state.update(status="partial")
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertGreaterEqual(self.clock.elapsed, 3)
        self.assertLess(self.clock.elapsed, 3.1)

    def test_sdk_swallowed_send_error_still_aborts_without_second_arm(self):
        left = self.robots["left"]
        left.send_error, left.swallow_error = OSError(errno.ENOBUFS, "queue full"), True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIsNone(left.comm.last_error)
        self.assertEqual(result["status"], "aborted_after_dispatch")
        self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 1)
        self.assertEqual(result["transmission_counts"]["left"]["sent_frames"], 0)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 0)
        self.assertIn("CAN send failed", result["errors"][0]["detail"])

    def test_rejected_mode_never_retries_or_sends_second_arm(self):
        self.robots["left"].accept = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("did not confirm", result["errors"][0]["detail"])

    def test_post_first_request_drift_blocks_second_arm(self):
        def hook(robot, state):
            if self.robots["left"].sent and robot.side == "right":
                state["pose_m_rad"][2] -= 0.01
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertGreater(result["drift"]["right"]["position_m"], 0.009)

    def test_bad_final_encoded_frame_is_blocked_at_bus(self):
        def bad(frame):
            frame.data[2] = 50
            return frame
        self.robots["left"].frame_transform = bad
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("exact standard 0x151", result["errors"][0]["detail"])
        self.assertEqual(result["transmission_counts"]["left"]["blocked_frames"], 1)

    def test_duplicate_sdk_send_is_blocked(self):
        self.robots["left"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("More than one SDK send", result["errors"][0]["detail"])

    def test_other_control_stream_detected_before_any_send(self):
        import can
        observed = False
        def hook(robot, state):
            nonlocal observed
            if robot.side == "left" and self.clock.elapsed >= 0.1 and not observed:
                observed = True
                robot.callback(can.Message(arbitration_id=0x155, is_extended_id=False, data=[0] * 8))
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("ownership", result["errors"][0]["detail"])

    def test_unrequested_arm_control_mode_change_refused(self):
        def hook(robot, state):
            if self.robots["left"].sent and robot.side == "right":
                state["arm_status"]["ctrl_mode"] = 1
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("outside its authorized", result["errors"][0]["detail"])

    def test_journal_intent_failure_prevents_dispatch(self):
        def journal(event, data):
            if event == "mode_request_intent":
                raise OSError("disk full")
        result = self.run_tool(journal)
        self.assert_no_tx(result)
        self.assertIn("disk full", result["errors"][0]["detail"])

    def test_initialization_transmit_is_forbidden(self):
        import can
        def connect():
            self.robots["left"].comm.send(can.Message(arbitration_id=0x471, data=[0] * 8))
        self.robots["left"].connect = connect
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertIn("Unexpected SDK TX", result["errors"][0]["detail"])

    def test_guard_violation_during_cleanup_cannot_report_success(self):
        import can
        self.robots["left"].disconnect.side_effect = lambda: self.robots["left"].callback(
            can.Message(arbitration_id=0x155, is_extended_id=False, data=[0] * 8))
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["status"], "guard_violation_during_cleanup")
        self.assertTrue(result["guard_violations"])

    def test_confirmed_can_control_reverting_to_teach_aborts(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["left"].sent and self.clock.elapsed > 1.2:
                state["arm_status"]["ctrl_mode"] = 2
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIn("outside its authorized", result["errors"][0]["detail"])


class RealSDKTakeoverTests(unittest.TestCase):
    def test_actual_sdk_encoding_on_fake_can_only(self):
        self.run_vendor_case(startup=False)

    def test_actual_piper_sdk_startup_encoding_and_status_enums_on_fake_can(self):
        self.run_vendor_case(startup=True)

    def run_vendor_case(self, *, startup):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, events, robots = Clock(), [], [], {}
        class FakeBus:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(0.002)
                return None
            def send(self, frame, timeout=None):
                sent.append((self.channel, frame))
            def shutdown(self):
                pass
        original_factory = sdk.AgxArmFactory.create_arm
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"  # Actual hardware model confirmed by user.
        def create_robot(config):
            robot = original_factory(config)
            robots[config["comm"]["can"]["channel"]] = robot
            return robot
        def snapshot(robot, gripper):
            channel = next(channel for channel, value in robots.items() if value is robot)
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1 if any(ch == channel for ch, _ in sent) else (0 if startup else 2),
                teach_status=0, mode_feedback=0, motion_status=0, arm_status=0, err_code=0))
            if startup:
                enabled = any(ch == channel and frame.arbitration_id == 0x471 for ch, frame in sent)
                for driver in state["drivers"].values():
                    driver["foc_status"]["driver_enable_status"] = enabled
                state["gripper"]["foc_status"]["driver_enable_status"] = False
            return state
        with patch("socket.socket", side_effect=AssertionError("No real socket")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            function = takeover.startup_arms if startup else takeover.request_can_control
            result = function(profile, lambda event, data: events.append((event, data)))
        self.assertTrue(result["ok"], result)
        expected_ids = [0x151, 0x151] + ([0x471, 0x471] if startup else [])
        self.assertEqual([frame.arbitration_id for channel, frame in sent], expected_ids)
        for (channel, frame), expected_channel in zip(sent, ("can0", "can2", "can0", "can2")):
            self.assertEqual(channel, expected_channel)
            expected = (1, 0, 1, 0, 0, 0, 0, 0) if frame.arbitration_id == 0x151 else (7, 2, 0, 0, 0, 0, 0, 0)
            self.assertEqual(bytes(frame.data), bytes(expected))
            self.assertEqual(frame.dlc, 8)
            self.assertFalse(frame.is_extended_id)
            self.assertFalse(frame.is_fd)


class FakeStartupRobot(FakeRobot):
    def __init__(self, side):
        super().__init__(side)
        self.ctrl_mode = 0
        self.driver_enabled = [False] * 6
        self.gripper_enabled = False
        self.accept_enable = True
        self.enable_error = None
        self.enable_gripper_too = False
        self.enable_transform = lambda frame: frame

    def _bus_send(self, frame):
        if frame.arbitration_id == 0x471:
            if self.enable_error:
                raise self.enable_error
            self.sent.append(frame)
            if self.accept_enable:
                self.driver_enabled = [True] * 6
                self.gripper_enabled = self.enable_gripper_too
        else:
            super()._bus_send(frame)

    def enable(self, joint_index):
        if joint_index != 255:
            raise AssertionError("Expected manufacturer's all-joint enable API")
        frame = self.can.Message(arbitration_id=0x471, is_extended_id=False,
                                 data=[7, 2, 0, 0, 0, 0, 0, 0])
        self._send_msg(self.enable_transform(frame))
        return False  # Deliberately stale vendor return; fresh feedback decides.


class StartupTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: FakeStartupRobot(side) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def snapshot(self, robot, gripper):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, teach_status=0, mode_feedback=robot.mode_feedback,
            motion_status=0, arm_status=0, err_code=0))
        for i, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None):
        return takeover.startup_arms(PROFILE, journal or (lambda event, data: self.events.append((event, data))))

    def has_enabled_frame(self, side):
        return any(frame.arbitration_id == 0x471 for frame in self.robots[side].sent)

    def test_two_can_confirmations_precede_either_enable_and_false_sdk_return_ignored(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        sequence = [(event, data["side"]) for event, data in self.events
                    if event in ("mode_request_intent", "enable_request_intent")]
        self.assertEqual(sequence, [("mode_request_intent", "left"), ("mode_request_intent", "right"),
                                    ("enable_request_intent", "left"), ("enable_request_intent", "right")])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["enable_commands_sent"], 2)
        self.assertEqual(result["enabled_arms"], ["left", "right"])
        self.assertEqual(result["gripper_enabled"], {"left": False, "right": False})
        self.assertFalse(result["fold_ready"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertTrue(result["hold_not_validated"])
        self.assertGreaterEqual(self.clock.elapsed, 9)
        for side, robot in self.robots.items():
            self.assertEqual([frame.arbitration_id for frame in robot.sent], [0x151, 0x471])
            self.assertEqual(bytes(robot.sent[1].data), bytes((7, 2, 0, 0, 0, 0, 0, 0)))
            for kind in ("mode", "enable"):
                self.assertEqual(result["transmission_counts_by_kind"][side][kind]["sent_frames"], 1)

    def test_original_takeover_still_refuses_standby_disabled(self):
        result = takeover.request_can_control(PROFILE, lambda event, data: None)
        self.assert_no_tx(result)

    def test_initial_enabled_joint_refused(self):
        self.robots["left"].driver_enabled[0] = True
        self.assert_no_tx(self.run_tool())

    def test_initial_enabled_gripper_refused(self):
        self.robots["right"].gripper_enabled = True
        self.assert_no_tx(self.run_tool())

    def test_initial_can_mode_refused_as_partial_replay(self):
        self.robots["left"].ctrl_mode = 1
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("no partial replay", result["errors"][0]["detail"])

    def test_second_mode_not_confirmed_means_no_enable_to_either_arm(self):
        self.robots["right"].accept = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertFalse(self.has_enabled_frame("left"))

    def test_partial_enable_can_advance_to_all_six_without_retransmission(self):
        self.robots["left"].accept_enable = False
        samples = 0
        def hook(robot, state):
            nonlocal samples
            if robot.side == "left" and self.has_enabled_frame("left"):
                samples += 1
                for i in range(1, 7):
                    state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = samples >= i
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 4)

    def test_enabled_bit_regression_stops_before_second_arm_enable(self):
        self.robots["left"].accept_enable = False
        samples = 0
        def hook(robot, state):
            nonlocal samples
            if robot.side == "left" and self.has_enabled_frame("left"):
                samples += 1
                state["drivers"]["1"]["foc_status"]["driver_enable_status"] = samples < 3
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("regressed", result["errors"][0]["detail"])
        self.assertEqual(result["enable_commands_sent"], 1)
        self.assertFalse(self.has_enabled_frame("right"))

    def test_partial_enable_timeout_no_retry_and_no_second_arm(self):
        self.robots["left"].accept_enable = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["enable_commands_sent"], 1)
        self.assertIn("all six", result["errors"][0]["detail"])
        self.assertFalse(self.has_enabled_frame("right"))

    def test_gripper_enable_side_effect_reported_without_gripper_target(self):
        self.robots["left"].enable_gripper_too = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["gripper_enabled"], {"left": True, "right": False})
        self.assertEqual(result["gripper_target_commands_sent"], 0)
        self.assertFalse(result["grasp_verified"])
        self.assertEqual([frame.arbitration_id for frame in self.robots["left"].sent], [0x151, 0x471])

    def test_inactive_arm_gripper_enable_change_refused(self):
        def hook(robot, state):
            if robot.side == "right" and self.has_enabled_frame("left"):
                state["gripper"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["enable_commands_sent"], 1)
        self.assertFalse(self.has_enabled_frame("right"))

    def test_gripper_enabled_bit_cannot_regress(self):
        self.robots["left"].enable_gripper_too = True
        samples = 0
        def hook(robot, state):
            nonlocal samples
            if robot.side == "left" and self.has_enabled_frame("left"):
                samples += 1
                if samples >= 3:
                    state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("regressed", result["errors"][0]["detail"])
        self.assertFalse(self.has_enabled_frame("right"))

    def test_gripper_width_drift_stops_remaining_enable(self):
        def hook(robot, state):
            if robot.side == "left" and self.has_enabled_frame("left"):
                state["gripper"]["width_m"] += 0.003
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("gripper_m", result["errors"][0]["detail"])
        self.assertFalse(self.has_enabled_frame("right"))

    def test_swallowed_enable_error_is_not_success_and_no_second_arm(self):
        self.robots["left"].enable_error = OSError(errno.ENOBUFS, "enable queue full")
        self.robots["left"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["transmission_counts_by_kind"]["left"]["enable"],
                         {"attempted_frames": 1, "sent_frames": 0})
        self.assertFalse(self.has_enabled_frame("right"))
        self.assertIn("CAN send failed", result["errors"][0]["detail"])

    def test_gripper_target_disguised_as_enable_is_blocked_at_bus(self):
        import can
        self.robots["left"].enable_transform = lambda frame: can.Message(
            arbitration_id=0x159, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 1, 0])
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertIn("exact standard 0x471", result["errors"][0]["detail"])

    def test_enable_journal_intent_error_leaves_both_disabled(self):
        def journal(event, data):
            if event == "enable_request_intent":
                raise OSError("disk full before enable")
        result = self.run_tool(journal)
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["last_enable_feedback"]["left"]["driver_enabled"], [False] * 6)


if __name__ == "__main__":
    unittest.main()
