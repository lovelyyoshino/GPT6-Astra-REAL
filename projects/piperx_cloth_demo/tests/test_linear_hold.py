"""Offline protocol/state-machine evidence only; every real socket is forbidden."""
import copy
import errno
import math
import struct
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, linear_hold, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture


class Motion:
    """Deterministic fake feedback, deliberately not a robot dynamics model."""
    def __init__(self, clock):
        self.clock, self.mode, self.started = clock, 0, None
        self.origin = [0.2, 0.1, 0.3, 0.1, 0.2, -0.1]
        self.target, self.hold, self.raw = None, None, {}
        self.pose_calls, self.accept_mode, self.accept_hold = 0, True, True
        self.speed = 0.0015
        self.rejected = self.retain_rejection = False

    def sent(self, frame):
        if frame.arbitration_id == 0x151:
            if self.accept_mode:
                self.mode = frame.data[1]
        elif 0x152 <= frame.arbitration_id <= 0x154:
            self.raw[frame.arbitration_id] = struct.unpack(">ii", bytes(frame.data))
            if frame.arbitration_id == 0x154:
                values = sum((list(self.raw[i]) for i in (0x152, 0x153, 0x154)), [])
                pose = [value / 1e6 for value in values[:3]] + [math.radians(value / 1000) for value in values[3:]]
                self.pose_calls += 1
                if self.pose_calls == 1:
                    self.started, self.target = self.clock.elapsed, pose
                    self.rejected = self.retain_rejection
                elif self.accept_hold:
                    self.hold = pose

    def pose(self):
        if self.hold is not None:
            return self.hold[:], 0
        pose = self.origin[:]
        if self.started is None:
            return pose, int(self.rejected)
        distance = min(0.006, (self.clock.elapsed - self.started) * self.speed)
        pose[2] += distance
        return pose, int(distance < 0.006)


class HoldRobot(FakeRobot):
    def __init__(self, side, clock):
        super().__init__(side)
        self.ctrl_mode = 1
        self.motion = Motion(clock)
        self.auto_mode = True
        self.fail_id = None
        self.partial = False
        self.gripper_enabled = False

    def _bus_send(self, frame):
        if frame.arbitration_id == self.fail_id:
            raise OSError(errno.ENOBUFS, "fake queue full")
        super()._bus_send(frame)
        self.motion.sent(frame)

    def set_auto_set_motion_mode_enabled(self, value):
        self.auto_mode = value

    def fk(self, joints):
        return self.motion.origin[:]

    def move_l(self, pose):
        if self.auto_mode:
            self.set_motion_mode("l")
        values = [round(x * 1e6) for x in pose[:3]] + [round(math.degrees(x) * 1000) for x in pose[3:]]
        for index in range(2 if self.partial else 3):
            frame = self.can.Message(arbitration_id=0x152 + index, is_extended_id=False,
                                     data=struct.pack(">ii", *values[index * 2:index * 2 + 2]))
            self._send_msg(self.frame_transform(frame))


class LinearHoldTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: HoldRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.stack.enter_context(patch.object(linear_hold, "time", self.clock))

    def snapshot(self, robot, gripper):
        self.snapshot_count += 1
        state = healthy_arm(self.clock.time())
        pose, motion = robot.motion.pose()
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, teach_status=0, motion_status=motion,
            mode_feedback=robot.motion.mode, arm_status=4 if robot.motion.rejected else 0, err_code=0))
        state["pose_m_rad"] = pose
        state["joints_rad"] = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None, arm="right", prior=None):
        return linear_hold.qualify_linear_hold(PROFILE,
            journal or (lambda event, data: self.events.append((event, data))), arm=arm,
            prior_mode_only_record=prior)

    def rejection_receipt(self):
        robot = self.robots["right"]
        robot.motion.mode, robot.motion.rejected = 2, True
        robot.gripper_enabled = True
        state = self.snapshot(robot, robot.gripper)
        run_id = "linear_hold_" + "a" * 32
        result = {"run_id": run_id, "operation": "qualify_linear_hold", "arm": "right",
                  "ok": False, "status": "aborted_after_dispatch", "qualified": False,
                  "guard_violations": [], "hardware_commands_sent": 1,
                  "target_commands_sent": 0, "target_calls_sent": 0, "enable_commands_sent": 0,
                  "stop_commands_sent": 0, "retries": 0, "after": {"right": state},
                  "transmission_counts": {}, "transmission_counts_by_kind": {}}
        for side in takeover.SIDES:
            count = int(side == "right")
            result["transmission_counts"][side] = {"attempted_frames": count, "sent_frames": count, "blocked_frames": 0}
            result["transmission_counts_by_kind"][side] = {
                kind: {"attempted_frames": count if kind == "mode" else 0,
                       "sent_frames": count if kind == "mode" else 0} for kind in ("mode", "travel", "overwrite")}
        return {"source_run_id": run_id, "result": result,
                "request": {"run_id": run_id, "arms": copy.deepcopy(PROFILE["arms"]), "arguments": {"arm": "right"}},
                "events": [{"event": "probe_send_intent", "kind": "mode", "arm": "right",
                            "target_pose_m_rad": None, "frames": [{"id": 0x151, "data_hex": "0102010000000000"}]},
                           {"event": "probe_send_complete_unconfirmed", "kind": "mode"}]}

    def assert_frames(self, side, ids):
        self.assertEqual([f.arbitration_id for f in self.robots[side].sent], ids)

    def test_exact_seven_frames_and_three_second_local_hold_only(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["qualified"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154, 0x152, 0x153, 0x154])
        self.assert_frames("left", [])
        self.assertFalse(self.robots["right"].auto_mode)
        self.assertEqual(bytes(self.robots["right"].sent[0].data), bytes((1, 2, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(result["hardware_commands_sent"], 7)
        self.assertEqual(result["target_commands_sent"], 6)
        self.assertEqual(result["target_calls_sent"], 2)
        self.assertGreaterEqual(result["stable_duration_s"], 3)
        self.assertGreater(result["minimum_original_target_distance_m"], 0.0015)
        self.assertLessEqual(result["stable_position_span_m"], 0.0005)
        self.assertGreaterEqual(result["stable_feedback_advances"], 20)
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertFalse(result["general_stop_validated"])
        self.assertTrue(result["hold_not_validated"])
        self.assertTrue(all(value["physically_stopped"] is None for value in result["cleanup"]["arms"].values()))

    def test_explicit_left_selection_still_only_selected_arm_sends(self):
        result = self.run_tool(arm="left")
        self.assertTrue(result["ok"], result)
        self.assert_frames("right", [])
        self.assertEqual(len(self.robots["left"].sent), 7)

    def test_known_joint_limit_violation_refused_before_any_mode(self):
        def hook(robot, state):
            if robot.side == "right":
                state["joints_rad"][1] = -0.03
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("joint limit", result["errors"][0]["detail"])

    def test_inactive_static_limit_violation_is_reported_with_no_motion_permission(self):
        def hook(robot, state):
            if robot.side == "left":
                state["joints_rad"][1] = -0.03
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["inactive_known_limit_violations"]["left"][0]["joint"], 2)
        self.assertEqual(result["inactive_known_limit_violations"]["left"][0]["actual_rad"], -0.03)
        self.assertFalse(result["inactive_arm_motion_qualified"])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assert_frames("left", [])

    def test_inactive_static_violation_does_not_waive_drift_guard(self):
        def hook(robot, state):
            if robot.side == "left":
                state["joints_rad"][1] = -0.03
                if self.robots["right"].motion.started is not None:
                    state["joints_rad"][1] -= 0.004
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("joint_rad", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])
        self.assert_frames("left", [])

    def test_inactive_static_violation_does_not_waive_fault_guard(self):
        def hook(robot, state):
            if robot.side == "left":
                state["joints_rad"][1] = -0.03
                if self.robots["right"].motion.started is not None:
                    state["arm_status"]["arm_status"] = 4
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("Unhealthy left feedback", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])
        self.assert_frames("left", [])

    def test_initial_teaching_refused(self):
        def hook(robot, state):
            state["arm_status"]["teach_status"] = 1
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("teaching/motion", result["errors"][0]["detail"])

    def test_initial_drift_refused(self):
        def hook(robot, state):
            if self.clock.elapsed > 0.1:
                state["joints_rad"][0] += 0.004
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("joint_rad", result["errors"][0]["detail"])

    def test_mode_not_confirmed_after_complete_target_prevents_overwrite(self):
        self.robots["right"].motion.accept_mode = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("mode not confirmed", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_no_intermediate_read_or_wait_between_mode_and_three_pose_frames(self):
        def hook(robot, state):
            self.assertNotEqual(len(robot.sent), 1)
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertNotIn("mode", [event[1]["kind"] for event in self.events if event[0] == "probe_send_intent"])

    def test_confirmed_mode_cannot_regress_after_pose_dispatch(self):
        def hook(robot, state):
            if robot.sent and self.clock.elapsed > 1.5:
                state["arm_status"]["mode_feedback"] = 0
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("mode changed unexpectedly", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_pose_sequence_partial_is_not_retried(self):
        self.robots["right"].partial = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("Incomplete", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153])
        self.assertFalse(result["overwrite_sent"])

    def test_swallowed_second_pose_frame_bus_error_is_reported_no_fallback(self):
        self.robots["right"].fail_id = 0x153
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("CAN send failed", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152])
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 3)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)

    def test_injected_extra_mode_or_gripper_frame_is_blocked(self):
        import can
        self.robots["right"].frame_transform = lambda frame: can.Message(
            arbitration_id=0x159, is_extended_id=False, data=bytes(8)) if frame.arbitration_id == 0x152 else frame
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assert_frames("right", [0x151])
        self.assertTrue(result["guard_violations"])

    def test_mode_duplicate_never_reaches_bus(self):
        self.robots["right"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assert_frames("right", [0x151])

    def test_stale_active_fragment_refuses_overwrite(self):
        fragment = arms.PARTS[0]
        def hook(robot, state):
            if robot.motion.started is not None:
                state["fragment_timestamps_s"][fragment] = self.clock.time() - 0.06
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("50 ms", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_no_motion_is_unproven_without_retry(self):
        self.robots["right"].motion.speed = 0
        result = self.run_tool()
        self.assertFalse(result["qualified"])
        self.assertIn("No qualifying motion", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_skipped_trigger_window_does_not_send_late_hold(self):
        self.robots["right"].motion.speed = 0.4
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("Missed replacement window", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_logging_delay_expiring_sample_blocks_overwrite(self):
        def journal(event, data):
            if event == "probe_send_intent" and data["kind"] == "overwrite":
                self.clock.sleep(0.4)
        result = self.run_tool(journal)
        self.assertFalse(result["ok"], result)
        self.assertIn("window/sample expired", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_ignored_replacement_cannot_claim_hold_at_original_target(self):
        self.robots["right"].motion.accept_hold = False
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["qualified"])
        self.assertTrue(result["original_target_approached"])
        self.assertEqual(result["hardware_commands_sent"], 7)

    def test_other_arm_drift_prevents_overwrite(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].motion.started is not None:
                state["pose_m_rad"][0] += 0.0021
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])
        self.assert_frames("left", [])

    def test_active_lateral_motion_leaving_envelope_prevents_overwrite(self):
        def hook(robot, state):
            if robot.motion.started is not None:
                state["pose_m_rad"][0] += 0.0021
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("probe envelope", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_settle_jitter_fails_three_second_requirement(self):
        def hook(robot, state):
            if robot.motion.hold is not None:
                state["pose_m_rad"][0] += 0.0004 if int(self.clock.elapsed * 20) % 2 else -0.0004
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["qualified"])
        self.assertIn("No separated stable", result["errors"][0]["detail"])
        self.assertEqual(result["hardware_commands_sent"], 7)

    def test_cleanup_external_command_cannot_preserve_success(self):
        robot = self.robots["right"]
        robot.disconnect.side_effect = lambda: robot.callback(robot.can.Message(
            arbitration_id=0x151, is_extended_id=False, data=bytes(8)))
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["qualified"])
        self.assertTrue(result["guard_violations"])

    def test_invalid_pose_is_rejected_without_clamping(self):
        self.robots["right"].motion.origin[4] = math.pi
        self.assert_no_tx(self.run_tool())

    def test_exact_prior_rejection_receipt_allows_one_full_trial_without_reset(self):
        prior = self.rejection_receipt()
        result = self.run_tool(prior=prior)
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["prior_rejection_recovery_observed"])
        self.assertEqual(result["before"]["right"]["arm_status"]["arm_status"], 4)
        self.assertTrue(result["fresh_normal_feedback_observed"])
        self.assertEqual(result["hardware_commands_sent"], 7)
        self.assertEqual(result["stop_commands_sent"], 0)

    def test_status_four_without_exact_saved_receipt_remains_refused(self):
        self.rejection_receipt()
        self.assert_no_tx(self.run_tool())

    def test_receipt_cannot_waive_different_current_mode(self):
        prior = self.rejection_receipt()
        self.robots["right"].motion.mode = 1
        result = self.run_tool(prior=prior)
        self.assert_no_tx(result)
        self.assertIn("no longer matches", result["errors"][0]["detail"])

    def test_receipt_wrong_bytes_target_history_or_identity_refused_before_connect(self):
        prior = self.rejection_receipt()
        variants = []
        changed = copy.deepcopy(prior)
        changed["events"][0]["frames"][0]["data_hex"] = "0102050000000000"
        variants.append(changed)
        changed = copy.deepcopy(prior)
        changed["result"]["target_commands_sent"] = 1
        variants.append(changed)
        changed = copy.deepcopy(prior)
        changed["request"]["arms"]["right"]["model"] = "piper"
        variants.append(changed)
        for value in variants:
            with self.subTest(receipt=value):
                with self.assertRaises(ValueError):
                    self.run_tool(prior=value)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()
        self.assert_frames("right", [])

    def test_persisting_status_four_times_out_without_overwrite(self):
        prior = self.rejection_receipt()
        self.robots["right"].motion.retain_rejection = True
        result = self.run_tool(prior=prior)
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["prior_rejection_recovery_observed"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])
        self.assertLessEqual(result["prior_rejection_wait_observed_s"], 0.2)

    def test_new_status_four_can_clear_within_bounded_acceptance_window(self):
        prior = self.rejection_receipt()
        motion = self.robots["right"].motion
        def hook(robot, state):
            if robot.side == "right" and motion.started is not None and self.clock.elapsed-motion.started < 0.15:
                state["arm_status"]["arm_status"] = 4
        self.hook = hook
        result = self.run_tool(prior=prior)
        self.assertTrue(result["ok"], result)
        self.assertGreater(result["prior_rejection_wait_observed_s"], 0.1)
        self.assertTrue(result["prior_rejection_recovery_observed"])

    def test_pending_rejection_with_significant_motion_aborts_before_overwrite(self):
        prior = self.rejection_receipt()
        motion = self.robots["right"].motion
        motion.retain_rejection, motion.speed = True, 0.01
        result = self.run_tool(prior=prior)
        self.assertFalse(result["ok"], result)
        self.assertIn("moved before normal", result["errors"][0]["detail"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_old_status_four_cache_may_wait_but_cannot_trigger_overwrite(self):
        prior = self.rejection_receipt()
        motion = self.robots["right"].motion
        def hook(robot, state):
            if robot.side == "right" and motion.started is not None and self.clock.elapsed-motion.started < 0.025:
                state["arm_status"]["arm_status"] = 4
                state["fragment_timestamps_s"]["arm_status"] = 1_800_000_000 + motion.started
        self.hook = hook
        result = self.run_tool(prior=prior)
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["prior_rejection_recovery_observed"])

    def test_normal_feedback_cannot_regress_to_rejection_even_with_old_timestamp(self):
        prior = self.rejection_receipt()
        motion = self.robots["right"].motion
        def hook(robot, state):
            if robot.side == "right" and motion.started is not None and self.clock.elapsed-motion.started >= 0.02:
                state["arm_status"]["arm_status"] = 4
                state["fragment_timestamps_s"]["arm_status"] = 1_800_000_000 + motion.started
        self.hook = hook
        result = self.run_tool(prior=prior)
        self.assertFalse(result["ok"], result)
        self.assertTrue(result["fresh_normal_feedback_observed"])
        self.assertFalse(result["overwrite_sent"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])

    def test_receipt_cannot_waive_current_driver_fault_or_pose_change(self):
        prior = self.rejection_receipt()
        def hook(robot, state):
            if robot.side == "right":
                state["drivers"]["1"]["foc_status"]["driver_error_status"] = True
        self.hook = hook
        result = self.run_tool(prior=prior)
        self.assert_no_tx(result)
        self.assertIn("driver_error_status", result["errors"][0]["detail"])

    def test_receipt_changed_start_pose_refused(self):
        prior = self.rejection_receipt()
        self.robots["right"].motion.origin[0] += 0.003
        result = self.run_tool(prior=prior)
        self.assert_no_tx(result)
        self.assertIn("differs from saved", result["errors"][0]["detail"])

    def test_fk_mismatch_prevents_dispatch(self):
        self.robots["right"].fk = lambda q: [0.3, 0.1, 0.3, 0.1, 0.2, -0.1]
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("Manufacturer FK", result["errors"][0]["detail"])

    def test_first_journal_motion_cannot_be_mistaken_for_probe_progress(self):
        motion = self.robots["right"].motion
        def journal(event, data):
            if event == "probe_send_intent" and data["kind"] == "travel":
                motion.origin[2] += 0.0012
                motion.speed = 0
        result = self.run_tool(journal)
        self.assert_no_tx(result)
        self.assertIn("Initial pose changed", result["errors"][0]["detail"])

    def test_small_initial_slip_does_not_count_as_post_dispatch_motion(self):
        motion = self.robots["right"].motion
        def journal(event, data):
            if event == "probe_send_intent" and data["kind"] == "travel":
                motion.origin[2] += 0.0004
                motion.speed = 0
        result = self.run_tool(journal)
        self.assertFalse(result["qualified"])
        self.assertFalse(result["overwrite_sent"])
        self.assert_frames("right", [0x151, 0x152, 0x153, 0x154])


class RealSDKLinearHoldTests(unittest.TestCase):
    def test_actual_piper_sdk_mode_and_two_move_l_encodings_with_no_auto_mode_frame(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, robots = Clock(), [], {}
        motions = {channel: Motion(clock) for channel in ("can0", "can2")}
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
        profile = copy.deepcopy(PROFILE)
        for config in profile["arms"].values():
            config["model"] = "piper"
        def create_robot(config):
            robot = original_factory(config)
            robot.get_fps = lambda: 100.0
            robots[id(robot)] = config["comm"]["can"]["channel"]
            motions[robots[id(robot)]].origin = robot.fk([0, 0.5, -0.5, 0, 0, 0])
            return robot
        def snapshot(robot, gripper):
            motion = motions[robots[id(robot)]]
            pose, status = motion.pose()
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1, teach_status=0, mode_feedback=motion.mode,
                motion_status=status, arm_status=0, err_code=0))
            state["pose_m_rad"] = pose
            state["joints_rad"] = [0, 0.5, -0.5, 0, 0, 0]
            return state
        with patch("socket.socket", side_effect=AssertionError("Real sockets prohibited")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), patch.object(linear_hold, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = linear_hold.qualify_linear_hold(profile, lambda event, data: None)
        self.assertTrue(result["ok"], result)
        self.assertEqual(len(sent), 7)
        self.assertEqual([frame.arbitration_id for channel, frame in sent],
                         [0x151, 0x152, 0x153, 0x154, 0x152, 0x153, 0x154])
        self.assertTrue(all(channel == "can2" and frame.dlc == 8 and not frame.is_extended_id
                            for channel, frame in sent))
        self.assertEqual(bytes(sent[0][1].data), bytes((1, 2, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(motions["can2"].pose_calls, 2)


if __name__ == "__main__":
    unittest.main()
