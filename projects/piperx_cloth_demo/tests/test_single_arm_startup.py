"""Independent single-arm startup checks; every socket and CAN bus is mocked.

These verify the generic commissioning contract, not physical hold, collision
clearance, task readiness, or successful grasping.
"""
import copy
import errno
import math
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, single_arm_startup, takeover
from robot_tools.backend import PyAgxBackend
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeStartupRobot, TakeoverFixture


class SingleArmStartupFixture(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: FakeStartupRobot(side) for side in takeover.SIDES}
        self.robots["left"].ctrl_mode = 2
        self.robots["left"].driver_enabled = [True] * 6
        self.robots["left"].gripper_enabled = True
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        if hasattr(single_arm_startup, "time"):
            self.stack.enter_context(patch.object(single_arm_startup, "time", self.clock))

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        self.assertIs(robot.gripper, gripper)
        state = healthy_arm(self.clock.time())
        state["arm_status"].update(ctrl_mode=robot.ctrl_mode, teach_status=0,
                                   mode_feedback=robot.mode_feedback)
        for i, enabled in enumerate(robot.driver_enabled, 1):
            state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = enabled
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None, arm="right"):
        return single_arm_startup.startup_arm(
            PROFILE, journal or (lambda event, data: self.events.append((event, data))), arm)

    def has_enable(self):
        return any(frame.arbitration_id == 0x471 for frame in self.robots["right"].sent)

    def assert_bounded_failure(self, result, expected_ids):
        self.assertFalse(result["ok"], result)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual([frame.arbitration_id for frame in self.robots["right"].sent], expected_ids)
        self.assertEqual(result["target_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["motion_gate_unlocked"])
        for side in takeover.SIDES:
            self.robots[side].disconnect.assert_called_once()
            self.assertIsNone(result["cleanup"]["arms"][side]["physically_stopped"])


class SingleArmStartupTests(SingleArmStartupFixture):
    def test_only_selected_arm_receives_exact_mode_and_enable_frames(self):
        self.robots["right"].mode_feedback = 2
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.robots["left"].sent, [])
        frames = self.robots["right"].sent
        self.assertEqual([frame.arbitration_id for frame in frames], [0x151, 0x471])
        self.assertEqual(bytes(frames[0].data), bytes((1, 2, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(bytes(frames[1].data), bytes((7, 2, 0, 0, 0, 0, 0, 0)))
        self.assertTrue(all(frame.dlc == 8 and not frame.is_extended_id and not frame.is_fd for frame in frames))
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["enable_commands_sent"], 1)
        self.assertEqual(result["target_commands_sent"], 0)
        self.assertEqual(result["gripper_target_commands_sent"], 0)
        self.assertEqual(result["enabled_arms"], ["right"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertTrue(result["hold_not_validated"])
        self.assertFalse(result["grasp_verified"])
        self.assertTrue(PyAgxBackend(PROFILE).commissioning_errors())
        self.assertGreaterEqual(self.clock.elapsed, 5)
        intents = [(event, data["side"]) for event, data in self.events
                   if event in ("mode_request_intent", "enable_request_intent")]
        self.assertEqual(intents, [("mode_request_intent", "right"), ("enable_request_intent", "right")])
        self.assertEqual(result["after"]["left"]["arm_status"]["ctrl_mode"], 2)

    def test_left_can_be_the_selected_arm_without_right_transmission(self):
        self.robots["left"].ctrl_mode = 0
        self.robots["left"].driver_enabled = [False] * 6
        self.robots["left"].gripper_enabled = False
        self.robots["right"].ctrl_mode = 2
        self.robots["right"].driver_enabled = [True] * 6
        self.robots["right"].gripper_enabled = True
        result = self.run_tool(arm="left")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual([frame.arbitration_id for frame in self.robots["left"].sent], [0x151, 0x471])

    def test_selected_initial_can_mode_refused_without_partial_replay(self):
        self.robots["right"].ctrl_mode = 1
        self.assert_no_tx(self.run_tool())

    def test_selected_initial_teaching_mode_refused(self):
        self.robots["right"].ctrl_mode = 2
        self.assert_no_tx(self.run_tool())

    def test_selected_initial_enabled_joint_refused(self):
        self.robots["right"].driver_enabled[4] = True
        self.assert_no_tx(self.run_tool())

    def test_selected_initial_enabled_gripper_refused(self):
        self.robots["right"].gripper_enabled = True
        self.assert_no_tx(self.run_tool())

    def test_passive_arm_unknown_enable_flag_refused(self):
        self.robots["left"].driver_enabled[2] = None
        self.assert_no_tx(self.run_tool())

    def test_passive_arm_mode_change_after_selected_mode_aborts_before_enable(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["arm_status"]["ctrl_mode"] = 1
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_passive_arm_enable_change_after_selected_mode_aborts_before_enable(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["drivers"]["3"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_passive_arm_gripper_enable_change_after_enable_aborts(self):
        def hook(robot, state):
            if robot.side == "left" and self.has_enable():
                state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151, 0x471])

    def test_passive_arm_motion_mode_change_refused(self):
        def hook(robot, state):
            if robot.side == "left" and self.clock.elapsed >= 0.1:
                state["arm_status"]["mode_feedback"] = 1
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_passive_arm_drift_before_startup_refused(self):
        def hook(robot, state):
            if robot.side == "left":
                state["joints_rad"][0] += self.clock.elapsed * 0.01
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("drift", result["errors"][0]["detail"])

    def test_passive_arm_drift_after_mode_prevents_enable(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["pose_m_rad"][2] += 0.01
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_pure_passive_arm_orientation_drift_refused_before_dispatch(self):
        def hook(robot, state):
            if robot.side == "left":
                state["pose_m_rad"][4] += self.clock.elapsed * 0.01
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("RPY drift", result["errors"][0]["detail"])

    def test_pure_selected_arm_orientation_drift_after_enable_aborts(self):
        def hook(robot, state):
            if robot.side == "right" and self.has_enable():
                state["pose_m_rad"][3] += 0.01
        self.hook = hook
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertIn("RPY drift", result["errors"][0]["detail"])

    def test_orientation_wrapping_at_pi_is_not_false_full_turn_drift(self):
        def hook(robot, state):
            state["pose_m_rad"][5] = math.pi - 0.0001 if self.clock.elapsed == 0 else -math.pi + 0.0001
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["drift"]["left"]["pose_rpy_component_rad"], 0.0002)

    def test_passive_arm_fault_after_mode_prevents_enable(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["drivers"]["2"]["foc_status"]["collision_status"] = True
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_passive_arm_stale_fragment_refused(self):
        def hook(robot, state):
            if robot.side == "left":
                state["fragment_timestamps_s"]["joint_34"] -= 1
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_passive_arm_active_teaching_refused(self):
        def hook(robot, state):
            if robot.side == "left":
                state["arm_status"]["teach_status"] = 1
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_mode_not_confirmed_never_enables_or_retries(self):
        self.robots["right"].accept = False
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertIn("did not confirm", result["errors"][0]["detail"])

    def test_mode_feedback_must_advance_after_mode_send_before_enable(self):
        frozen = None
        def hook(robot, state):
            nonlocal frozen
            if robot.side == "right" and self.robots["right"].sent:
                if frozen is None:
                    frozen = self.clock.time()
                state["fragment_timestamps_s"]["arm_status"] = frozen
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_selected_cannot_enable_spontaneously_during_mode_confirmation(self):
        def hook(robot, state):
            if robot.side == "right" and self.robots["right"].sent and not self.has_enable():
                state["drivers"]["1"]["foc_status"]["driver_enable_status"] = True
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151])

    def test_partial_enable_can_advance_to_all_six_with_one_frame(self):
        self.robots["right"].accept_enable = False
        samples = 0
        def hook(robot, state):
            nonlocal samples
            if robot.side == "right" and self.has_enable():
                samples += 1
                for i in range(1, 7):
                    state["drivers"][str(i)]["foc_status"]["driver_enable_status"] = samples >= i
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["enable_commands_sent"], 1)

    def test_single_stale_driver_cannot_confirm_enable(self):
        frozen = None
        def hook(robot, state):
            nonlocal frozen
            if robot.side == "right" and self.has_enable():
                if frozen is None:
                    frozen = self.clock.time()
                state["fragment_timestamps_s"]["driver_state_6"] = frozen
        self.hook = hook
        self.assert_bounded_failure(self.run_tool(), [0x151, 0x471])

    def test_partial_enable_timeout_has_no_retry_or_fallback(self):
        self.robots["right"].accept_enable = False
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertIn("all six", result["errors"][0]["detail"])

    def test_enable_feedback_regression_aborts(self):
        samples = 0
        def hook(robot, state):
            nonlocal samples
            if robot.side == "right" and self.has_enable():
                samples += 1
                if samples >= 3:
                    state["drivers"]["4"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertIn("regressed", result["errors"][0]["detail"])

    def test_gripper_enable_side_effect_does_not_send_gripper_target(self):
        self.robots["right"].enable_gripper_too = True
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["gripper_enabled"]["right"])
        self.assertEqual(result["gripper_target_commands_sent"], 0)
        self.assertFalse(result["grasp_verified"])

    def test_swallowed_enable_error_not_hidden_by_sdk_cached_return(self):
        self.robots["right"].enable_error = OSError(errno.ENOBUFS, "enable queue full")
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertEqual(result["status"], "aborted_after_dispatch")
        self.assertEqual(result["transmission_counts_by_kind"]["right"]["enable"],
                         {"attempted_frames": 1, "sent_frames": 0})
        self.assertIn("CAN send failed", result["errors"][0]["detail"])

    def test_swallowed_mode_error_never_retries_or_enables(self):
        self.robots["right"].send_error = OSError(errno.ENOBUFS, "mode queue full")
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assert_bounded_failure(result, [])
        self.assertEqual(result["status"], "aborted_after_dispatch")
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 1)

    def test_disguised_target_frame_rejected_at_underlying_bus(self):
        import can
        self.robots["right"].enable_transform = lambda frame: can.Message(
            arbitration_id=0x159, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 1, 0])
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertIn("exact standard 0x471", result["errors"][0]["detail"])

    def test_unreviewed_mode_speed_rejected_at_underlying_bus(self):
        def bad(frame):
            frame.data[2] = 50
            return frame
        self.robots["right"].frame_transform = bad
        self.assert_bounded_failure(self.run_tool(), [])

    def test_duplicate_enable_send_is_blocked(self):
        original = self.robots["right"].enable
        def duplicate(index):
            original(index)
            return original(index)
        self.robots["right"].enable = duplicate
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertEqual(result["transmission_counts_by_kind"]["right"]["enable"]["attempted_frames"], 1)

    def test_selected_sdk_attempt_to_send_other_arm_is_blocked(self):
        self.robots["right"].enable = lambda index: self.robots["left"].enable(index)
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertEqual(result["transmission_counts"]["left"]["blocked_frames"], 1)

    def test_other_arm_direct_comm_send_is_blocked(self):
        import can
        self.robots["right"].enable = lambda index: self.robots["left"].comm.send(
            can.Message(arbitration_id=0x471, is_extended_id=False, data=[7, 2, 0, 0, 0, 0, 0, 0]))
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertEqual(result["transmission_counts"]["left"]["blocked_frames"], 1)

    def test_other_arm_direct_bus_send_is_blocked(self):
        import can
        self.robots["right"].enable = lambda index: self.robots["left"].comm.send_bus.send(
            can.Message(arbitration_id=0x471, is_extended_id=False, data=[7, 2, 0, 0, 0, 0, 0, 0]))
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151])
        self.assertEqual(result["transmission_counts"]["left"]["blocked_frames"], 1)

    def test_journal_enable_intent_failure_prevents_enable(self):
        def journal(event, data):
            if event == "enable_request_intent":
                raise OSError("disk full before enable")
        result = self.run_tool(journal)
        self.assert_bounded_failure(result, [0x151])
        self.assertIn("disk full", result["errors"][0]["detail"])

    def test_feedback_rechecked_after_journal_enable_intent(self):
        def journal(event, data):
            if event == "enable_request_intent":
                self.robots["left"].ctrl_mode = 1
        self.assert_bounded_failure(self.run_tool(journal), [0x151])

    def test_cleanup_guard_violation_cannot_claim_success_or_stop(self):
        import can
        self.robots["left"].disconnect.side_effect = lambda: self.robots["left"].callback(
            can.Message(arbitration_id=0x155, is_extended_id=False, data=[0] * 8))
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertEqual(result["status"], "guard_violation_during_cleanup")

    def test_disconnect_error_cannot_claim_success_or_physical_stop(self):
        self.robots["right"].disconnect.side_effect = OSError("disconnect failed")
        result = self.run_tool()
        self.assert_bounded_failure(result, [0x151, 0x471])
        self.assertEqual(result["status"], "cleanup_failed")

    def test_invalid_arm_rejected_without_constructing_devices(self):
        for arm in (None, "", "both", "Right", ["right"], True):
            with self.subTest(arm=repr(arm)):
                with self.assertRaises((ValueError, TypeError)):
                    self.run_tool(arm=arm)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()


class RealSDKSingleArmStartupTests(unittest.TestCase):
    def test_actual_sdk_piper_encoding_and_status_enums_on_fake_can_only(self):
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
                sent.append((self.channel, frame))

            def shutdown(self):
                pass

        profile = copy.deepcopy(PROFILE)
        for config in profile["arms"].values():
            config["model"] = "piper"
        original_factory = sdk.AgxArmFactory.create_arm

        def create_robot(config):
            robot = original_factory(config)
            robots[config["comm"]["can"]["channel"]] = robot
            return robot

        def snapshot(robot, gripper):
            channel = next(channel for channel, value in robots.items() if value is robot)
            selected = channel == profile["arms"]["right"]["channel"]
            state = healthy_arm(clock.time())
            mode = 1 if any(ch == channel for ch, _ in sent) else (0 if selected else 2)
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=mode, teach_status=0, mode_feedback=0,
                motion_status=0, arm_status=0, err_code=0))
            enabled = not selected or any(ch == channel and frame.arbitration_id == 0x471
                                          for ch, frame in sent)
            for driver in state["drivers"].values():
                driver["foc_status"]["driver_enable_status"] = enabled
            state["gripper"]["foc_status"]["driver_enable_status"] = not selected
            return state

        with patch("socket.socket", side_effect=AssertionError("No real socket")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = single_arm_startup.startup_arm(profile, lambda event, data: None, "right")
        self.assertTrue(result["ok"], result)
        self.assertEqual([channel for channel, _ in sent], ["can2", "can2"])
        self.assertEqual([frame.arbitration_id for _, frame in sent], [0x151, 0x471])
        self.assertEqual(bytes(sent[0][1].data), bytes((1, 0, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(bytes(sent[1][1].data), bytes((7, 2, 0, 0, 0, 0, 0, 0)))
        self.assertTrue(all(frame.dlc == 8 and not frame.is_extended_id and not frame.is_fd
                            for _, frame in sent))


if __name__ == "__main__":
    unittest.main()
