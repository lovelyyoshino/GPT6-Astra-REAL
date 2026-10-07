"""Same-connection preparation tests; sockets and all physical CAN are blocked."""
import copy
import time
import unittest
from contextlib import ExitStack
from unittest.mock import patch

from robot_tools import arms, linear_hold, pair_device, pair_preparation
from robot_tools import single_supervised_actions, supervised_actions, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_single_gripper_prepare import SingleGripperFixture


class PairPreparationTests(SingleGripperFixture):
    def setUp(self):
        super().setUp()
        for module in (pair_device, pair_preparation, linear_hold,
                       single_supervised_actions, supervised_actions):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.guard_hook = None
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)), self.guard)
        self.addCleanup(self.device.close)

    def guard(self):
        if self.guard_hook:
            self.guard_hook()

    def ready_joints(self):
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True] * 6

    def test_connect_disabled_peer_is_zero_tx_with_frozen_three_second_baseline(self):
        sample = self.device.connect_for_preparation()
        self.assertFalse(sample["task_ready"])
        self.assertTrue(sample["connected_for_preparation"])
        self.assertGreaterEqual(sample["baseline_duration_s"], 3.)
        self.assertGreaterEqual(sample["feedback_advances"], 20)
        self.assertEqual(sample["readiness"]["left"]["enable_flags"], [False] * 7)
        self.assertEqual(sample["readiness"]["left"]["ctrl_mode"], 0)
        for side in takeover.SIDES:
            binding = self.device.joint_binding(side)
            self.assertEqual(binding["model"], self.profile["arms"][side]["model"])
            self.assertTrue(binding["connection_id"])
            self.assertIsNone(binding["cached_target"])
        self.assertEqual([x["joint_index"] for x in sample["observed_joint_limit_violations"]["right"]], [2, 3])
        observed = self.device.observe()
        self.assertGreater(observed["sequence"], sample["sequence"])
        self.assertTrue(all(not r.sent for r in self.robots.values()))
        self.assertIsNone(self.device._fault)
        with self.assertRaisesRegex(RuntimeError, "task readiness"):
            self.device.execute("right", "gripper", .003)
        self.assertIsNone(self.device._fault)

    def test_one_exact_selected_frame_reuses_two_connections_and_preserves_anchor(self):
        first = self.device.connect_for_preparation()
        anchor = copy.deepcopy(self.device._preparation.anchor)
        result = self.device.prepare_gripper("right")
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["task_ready"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(result["target_width_raw"], 2800)
        self.assertGreater(result["samples"], 3)
        self.assertFalse(result["before"]["right"]["gripper"]["foc_status"]["driver_enable_status"])
        self.assertTrue(result["after"]["right"]["gripper"]["foc_status"]["driver_enable_status"])
        self.assertEqual(result["arm_target_commands_sent"], 0)
        self.assertEqual(result["mode_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertFalse(result["grasp_verified"])
        self.assertIsNone(result["physical_stop_verified"])
        frame, = self.robots["right"].sent
        self.assertEqual(frame.arbitration_id, 0x159)
        self.assertEqual(bytes(frame.data).hex(), "00000af000c80100")
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertTrue(all(not r.disconnect.called for r in self.robots.values()))
        self.assertEqual(self.device._preparation.anchor, anchor)
        self.assertGreater(result["sample"]["sequence"], first["sequence"])
        repeated = self.device.prepare_gripper("right")
        self.assertEqual(repeated["status"], "already_enabled_observed")
        self.assertEqual(repeated["hardware_commands_sent"], 0)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_known_missing_enable_or_mode_is_nonfault_zero_tx(self):
        self.device.connect_for_preparation()
        result = self.device.prepare_gripper("left")
        self.assertEqual(result["status"], "preparation_required")
        self.assertEqual(len(result["requirements"]), 2)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertFalse(result["fault_latched"])
        result = self.device.promote_ready()
        self.assertFalse(result["task_ready"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNone(self.device._fault)
        self.assertTrue(all(not r.sent for r in self.robots.values()))
        self.device.observe()

    def test_two_jaws_then_promote_share_connection_without_arm_enable_or_targets(self):
        self.ready_joints()
        self.device.connect_for_preparation()
        left = self.device.prepare_gripper("left")
        right = self.device.prepare_gripper("right")
        self.assertFalse(left["task_ready"])
        self.assertFalse(right["task_ready"])
        result = self.device.promote_ready()
        self.assertTrue(result["task_ready"])
        self.assertTrue(self.device._task_ready)
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        for side in takeover.SIDES:
            self.assertEqual([f.arbitration_id for f in self.robots[side].sent], [0x159])
            self.assertEqual(self.device._action.totals()[side]["sent_frames"], 1)
        self.device.observe()

    def test_default_open_still_refuses_disabled_jaws(self):
        with self.assertRaisesRegex(RuntimeError, "both CAN arms"):
            self.device.open()
        self.assertTrue(all(not r.sent for r in self.robots.values()))

    def test_width_boundary_70mm_and_zero_preserve_preparation_contract(self):
        for width in (0., .07):
            with self.subTest(width=width):
                # New offline fixture driver, never a second live connection.
                self.make_robots()
                self.robots["right"].width = width
                device = pair_device.GuardedPairDevice(self.profile, lambda *a: None)
                try:
                    device.connect_for_preparation()
                    result = device.prepare_gripper("right")
                    self.assertTrue(result["ok"])
                    self.assertEqual(result["target_width_raw"], round(width * 1e6))
                    self.assertEqual(len(self.robots["right"].sent), 1)
                finally:
                    device.close()

    def test_stale_regressed_alarm_and_unknown_flags_fault_before_tx(self):
        self.device.connect_for_preparation()
        prior = copy.deepcopy(self.device._previous)
        def stale(robot, state):
            state.update(copy.deepcopy(prior[robot.side]))
            state["fragment_timestamps_s"]["joint_12"] -= .2
        self.hook = stale
        with self.assertRaises(RuntimeError):
            self.device.prepare_gripper("right")
        self.assertIsNotNone(self.device._fault)
        self.assertTrue(all(not r.sent for r in self.robots.values()))

    def test_each_health_enable_or_pose_change_faults_connection(self):
        changes = (
            lambda s: s["drivers"]["1"]["foc_status"].update(driver_enable_status=None),
            lambda s: s["drivers"]["1"]["foc_status"].update(collision_status=True),
            lambda s: s["arm_status"].update(teach_status=1),
            lambda s: s["pose_m_rad"].__setitem__(5, .004),
        )
        for change in changes:
            with self.subTest(change=change):
                self.make_robots()
                self.hook = None
                device = pair_device.GuardedPairDevice(self.profile, lambda *a: None)
                try:
                    device.connect_for_preparation()
                    self.hook = lambda robot, state: change(state) if robot.side == "left" else None
                    with self.assertRaises(RuntimeError):
                        device.observe()
                    self.assertIsNotNone(device._fault)
                    self.assertTrue(all(not r.sent for r in self.robots.values()))
                finally:
                    device.close()

    def test_cumulative_drift_cannot_reanchor_across_observations_or_jaw_preparation(self):
        self.device.connect_for_preparation()
        displacement = [.0015]
        self.hook = lambda robot, s: s["joints_rad"].__setitem__(0, displacement[0])
        self.device.observe()
        self.device.prepare_gripper("right")
        displacement[0] = .0031
        with self.assertRaisesRegex(RuntimeError, "drift"):
            self.device.observe()
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_promotion_preserves_original_arm_anchor_against_cumulative_drift(self):
        self.ready_joints()
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.connect_for_preparation()
        original = copy.deepcopy(self.device._preparation.anchor)
        delta = [.0029]
        self.hook = lambda robot, state: state["joints_rad"].__setitem__(0, delta[0])
        self.device.observe()
        self.assertTrue(self.device.promote_ready()["task_ready"])
        self.assertEqual(self.device._action.idle_anchor, original)
        self.assertEqual(self.device._action.anchor, original)
        delta[0] = .0058
        with self.assertRaises(RuntimeError):
            self.device.observe()
        self.assertIsNotNone(self.device._fault)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_promotion_rechecks_completion_against_original_preparation_anchor(self):
        self.ready_joints()
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.connect_for_preparation()
        original_prepare = self.device._action.prepare
        def drift_after_baseline():
            original_prepare()
            self.device._action.report["after"]["left"]["joints_rad"][0] += .0031
        with patch.object(self.device._action, "prepare", side_effect=drift_after_baseline):
            with self.assertRaisesRegex(RuntimeError, "drift"):
                self.device.promote_ready()
        self.assertFalse(self.device._task_ready)
        self.assertIsNotNone(self.device._fault)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_promotion_cannot_absorb_mode_change_at_start_of_new_baseline(self):
        self.ready_joints()
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.connect_for_preparation()
        original_prepare = self.device._action.prepare
        def change_mode_before_baseline():
            self.robots["left"].mode_feedback = 2
            original_prepare()
        with patch.object(self.device._action, "prepare", side_effect=change_mode_before_baseline):
            with self.assertRaisesRegex(RuntimeError, "motion mode changed"):
                self.device.promote_ready()
        self.assertFalse(self.device._task_ready)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_promotion_keeps_prepared_width_tolerance_through_new_baseline(self):
        self.ready_joints()
        self.device.connect_for_preparation()
        self.device.prepare_gripper("left")
        self.device.prepare_gripper("right")
        original_prepare = self.device._action.prepare
        def change_width_before_baseline():
            self.robots["left"].width += .0011
            original_prepare()
        with patch.object(self.device._action, "prepare", side_effect=change_width_before_baseline):
            with self.assertRaisesRegex(RuntimeError, "original measured-width target"):
                self.device.promote_ready()
        self.assertFalse(self.device._task_ready)
        self.assertTrue(all(len(robot.sent) == 1 for robot in self.robots.values()))

    def test_slow_guard_width_change_is_zero_tx(self):
        self.device.connect_for_preparation()
        def guard():
            if self.device._action.ticket is not None:
                self.clock.sleep(.2)
                self.robots["right"].width = .0034
        self.guard_hook = guard
        with self.assertRaisesRegex(RuntimeError, "0.5 mm"):
            self.device.prepare_gripper("right")
        self.assertEqual(self.robots["right"].sent, [])
        self.assertIsNotNone(self.device._fault)

    def test_unsolicited_jaw_enable_before_actual_frame_is_zero_tx(self):
        self.device.connect_for_preparation()
        self.guard_hook = lambda: setattr(self.robots["right"], "gripper_enabled", True) \
            if self.device._action.ticket is not None else None
        with self.assertRaisesRegex(RuntimeError, "before authorized jaw response"):
            self.device.prepare_gripper("right")
        self.assertEqual(self.robots["right"].sent, [])

    def test_no_enable_response_one_frame_fault_and_no_retry(self):
        self.device.connect_for_preparation()
        self.robots["right"].accept = False
        with self.assertRaisesRegex(RuntimeError, "did not confirm enabled"):
            self.device.prepare_gripper("right")
        self.assertEqual(len(self.robots["right"].sent), 1)
        with self.assertRaises(RuntimeError):
            self.device.prepare_gripper("right")
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertIsNotNone(self.device._fault)

    def test_swallowed_bus_error_is_uncertain_and_not_retried(self):
        self.device.connect_for_preparation()
        self.robots["right"].send_error = OSError("uncertain CAN write")
        self.robots["right"].swallow_error = True
        with self.assertRaisesRegex(RuntimeError, "uncertain CAN write"):
            self.device.prepare_gripper("right")
        self.assertEqual(self.device._action.counts["right"]["attempted_frames"], 1)
        self.assertEqual(self.device._action.counts["right"]["sent_frames"], 0)
        with self.assertRaises(RuntimeError):
            self.device.prepare_gripper("right")
        self.assertEqual(self.device._action.counts["right"]["attempted_frames"], 1)

    def test_duplicate_or_modified_frame_is_blocked(self):
        self.device.connect_for_preparation()
        self.robots["right"].duplicate = True
        with self.assertRaises(RuntimeError):
            self.device.prepare_gripper("right")
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertIsNotNone(self.device._fault)

    def test_wrong_encoder_or_peer_sender_during_ticket_is_blocked(self):
        self.device.connect_for_preparation()
        self.robots["right"].frame_transform = lambda frame: (
            self.robots["left"].set_motion_mode("j") or frame)
        with self.assertRaises(RuntimeError):
            self.device.prepare_gripper("right")
        self.assertTrue(all(not r.sent for r in self.robots.values()))

    def test_prepared_target_remains_monitored_after_receipt(self):
        self.device.connect_for_preparation()
        self.device.prepare_gripper("right")
        self.robots["right"].width += .0011
        with self.assertRaisesRegex(RuntimeError, "original measured-width target"):
            self.device.observe()
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_prepared_peer_width_is_monitored_during_other_jaw_preparation(self):
        self.ready_joints()
        self.device.connect_for_preparation()
        self.device.prepare_gripper("left")
        old_width = self.robots["left"].width
        def guard():
            if self.device._action.ticket is not None:
                self.robots["left"].width = old_width + .0011
        self.guard_hook = guard
        with self.assertRaisesRegex(RuntimeError, "original measured-width target"):
            self.device.prepare_gripper("right")
        self.assertEqual(len(self.robots["left"].sent), 1)
        self.assertEqual(self.robots["right"].sent, [])


class RealSDKPairPreparationTests(unittest.TestCase):
    def test_actual_vendor_one_159_per_jaw_same_connection_without_sockets(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, created, bindings = Clock(), [], [], {}
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper_x"
        widths = {cfg["channel"]: .0028 for cfg in profile["arms"].values()}
        enabled = {key: False for key in widths}
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def shutdown(self):
                pass
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == 0x159:
                    enabled[self.channel] = True
                    widths[self.channel] = int.from_bytes(bytes(frame.data[:4]), "big", signed=True)/1e6
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            bindings[id(robot)] = config["comm"]["can"]["channel"]
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            channel = bindings[id(robot)]
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1,
                mode_feedback=1, teach_status=0, motion_status=0, arm_status=0, err_code=0))
            state["gripper"]["width_m"] = widths[channel]
            state["gripper"]["foc_status"]["driver_enable_status"] = enabled[channel]
            state["joints_rad"][1:3] = [-.091769, .045658]
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("Physical socket forbidden")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, linear_hold, supervised_actions,
                           single_supervised_actions, pair_device, pair_preparation):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile, lambda *a: None)
            try:
                device.connect_for_preparation()
                self.assertEqual(sent, [])
                for side in takeover.SIDES:
                    result = device.prepare_gripper(side)
                    self.assertTrue(result["ok"], result)
                    self.assertFalse(result["task_ready"])
                self.assertTrue(device.promote_ready()["task_ready"])
                self.assertEqual(len(created), 2)
                self.assertEqual([frame.arbitration_id for _, frame in sent], [0x159, 0x159])
                self.assertEqual([channel for channel, _ in sent],
                                 [profile["arms"][s]["channel"] for s in takeover.SIDES])
                self.assertTrue(all(bytes(frame.data).hex() == "00000af000c80100" for _, frame in sent))
                self.assertEqual(device.prepare_gripper("right")["hardware_commands_sent"], 0)
            finally:
                device.close()
            self.assertEqual(len(sent), 2)


if __name__ == "__main__":
    unittest.main()
