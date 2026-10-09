"""Protocol/feedback contracts only; fake feedback is not robot dynamics."""
import copy
import errno
import math
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, linear_hold, supervised_actions as actions, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_gripper_prepare import FakeGripper
from test_linear_hold import HoldRobot, Motion
from test_takeover import TakeoverFixture


class ActionRobot(HoldRobot):
    def __init__(self, side, clock):
        super().__init__(side, clock)
        self.gripper = FakeGripper(self)
        self.width = 0.050
        self.gripper_enabled = True
        self.motion_flag = None

    def _bus_send(self, frame):
        super()._bus_send(frame)
        if frame.arbitration_id == 0x159 and self.accept:
            self.width = int.from_bytes(frame.data[:4], "big", signed=True)/1e6


class SupervisedActionsTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots = {side: ActionRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.stack.enter_context(patch.object(actions, "time", self.clock))
        self.stack.enter_context(patch.object(linear_hold, "time", self.clock))
        self.target = self.robots["right"].motion.origin[:]
        self.target[2] += 0.006

    def snapshot(self, robot, gripper):
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.snapshot_count += 1
        state = healthy_arm(self.clock.time())
        pose, motion = robot.motion.pose()
        state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
            ctrl_mode=robot.ctrl_mode, mode_feedback=robot.motion.mode, teach_status=0,
            motion_status=motion if robot.motion_flag is None else robot.motion_flag,
            arm_status=0, err_code=0))
        state["pose_m_rad"], state["joints_rad"] = pose, [0, 0.5, -0.5, 0, 0, 0]
        state["gripper"]["width_m"] = robot.width
        state["gripper"]["foc_status"]["driver_enable_status"] = robot.gripper_enabled
        if self.hook:
            self.hook(robot, state)
        return state

    def run_tool(self, journal=None, *, grip=False, arm="right", width=0.0):
        journal = journal or (lambda event, data: self.events.append((event, data)))
        if grip:
            return actions.gripper_once(PROFILE, journal, arm, width, 0.2)
        return actions.move_once(PROFILE, journal, arm, self.target)

    def test_move_exact_four_contiguous_frames_and_stability_without_stop_claim(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151, 0x152, 0x153, 0x154])
        self.assertEqual(bytes(self.robots["right"].sent[0].data), bytes((1, 2, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.robots["right"].motion.pose_calls, 1)
        self.assertEqual(result["target_calls_sent"], 1)
        self.assertTrue(result["controller_at_target"])
        self.assertTrue(result["observed_stable"])
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["observed_stable_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        for field in ("general_stop_validated", "grasp_verified", "motion_gate_unlocked", "manufacturer_ik_path_verified"):
            self.assertFalse(result[field])
        self.assertTrue(all(v["physically_stopped"] is None for v in result["cleanup"]["arms"].values()))

    def test_one_jaw_159_zero_width_is_not_zero_calibration_or_grasp(self):
        result = self.run_tool(grip=True, arm="left")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.robots["right"].sent, [])
        frames = self.robots["left"].sent
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].arbitration_id, 0x159)
        self.assertEqual(bytes(frames[0].data), bytes((0, 0, 0, 0, 0, 200, 1, 0)))
        self.assertEqual(result["width_error_m"], 0)
        self.assertFalse(result["grasp_verified"])
        self.assertFalse(result["force_calibrated"])

    def test_stable_motion_one_is_reported_not_misclassified_as_moving(self):
        for robot in self.robots.values():
            robot.motion_flag = 1
        self.robots["right"].motion.speed = 0
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["observed_stable"])
        self.assertFalse(result["controller_at_target"])
        self.assertEqual(result["raw_motion_status"], {"left": 1, "right": 1})
        self.assertAlmostEqual(result["pose_error"]["position_m"], 0.006)
        self.assertAlmostEqual(result["pose_error"]["rotation_rad"], 0.)
        self.assertNotIn("joint_error", result)

    def test_stable_jaw_short_of_target_is_observation_not_contact_proof(self):
        self.robots["right"].accept = False
        result = self.run_tool(grip=True)
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(result["width_error_m"], 0.050)
        self.assertFalse(result["grasp_verified"])

    def test_invalid_arguments_fail_without_connect(self):
        for width, force in ((-1, .2), (.055001, .2), (False, .2), (float("nan"), .2), (.01, .3), (.01, True)):
            with self.assertRaises(ValueError):
                actions.gripper_once(PROFILE, lambda *a: None, "right", width, force)
        for target in ([0]*5, [0, 0, 0, 0, 2, 0], [0, 0, float("nan"), 0, 0, 0]):
            with self.assertRaises(ValueError):
                actions.move_once(PROFILE, lambda *a: None, "right", target)
        self.assertTrue(all(not r.sent for r in self.robots.values()))

    def test_oversized_translation_refused_before_mode(self):
        self.target[2] += .025
        self.assert_no_tx(self.run_tool())

    def test_oversized_rotation_refused_before_mode(self):
        self.target[3] += .06
        self.assert_no_tx(self.run_tool())

    def test_either_arm_outside_limits_prevents_dispatch(self):
        self.hook = lambda robot, state: state["joints_rad"].__setitem__(1, -.001) if robot.side == "left" else None
        self.assert_no_tx(self.run_tool())

    def test_disabled_jaw_prevents_all_commands(self):
        self.robots["left"].gripper_enabled = False
        self.assert_no_tx(self.run_tool(grip=True))

    def test_fk_mismatch_prevents_mode_frame(self):
        self.robots["right"].fk = lambda q: [0.3, 0.1, 0.3, 0.1, 0.2, -0.1]
        self.assert_no_tx(self.run_tool())

    def test_baseline_three_second_span_rejects_noise_within_anchor_tolerance(self):
        def hook(robot, state):
            if self.clock.elapsed:
                state["joints_rad"][0] = .002 if int(self.clock.elapsed*10) % 2 else -.002
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_journal_latency_or_motion_revalidated_before_dispatch(self):
        def journal(event, data):
            if event == "supervised_action_intent":
                self.robots["right"].motion.origin[0] += .0006
        self.assert_no_tx(self.run_tool(journal))

    def test_fault_after_dispatch_no_retry_or_stop(self):
        def hook(robot, state):
            if self.robots["right"].sent:
                state["arm_status"]["arm_status"] = 4
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(len(self.robots["right"].sent), 4)
        self.assertEqual(result["stop_commands_sent"], 0)

    def test_inactive_arm_drift_detected_after_one_dispatch(self):
        def hook(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["joints_rad"][0] += .004
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 4)

    def test_selected_joint_branch_excursion_detected(self):
        def hook(robot, state):
            if robot.side == "right" and robot.sent:
                state["joints_rad"][0] = .151
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("envelope", result["errors"][0]["detail"])

    def test_partial_pose_has_no_retry_or_cleanup_frames(self):
        self.robots["right"].partial = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 3)
        self.assertIn("Incomplete", result["errors"][0]["detail"])

    def test_swallowed_bus_failure_persists_through_sdk_error_clear(self):
        robot = self.robots["right"]
        robot.fail_id, robot.swallow_error = 0x153, True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 3)

    def test_wrong_jaw_zero_byte_is_blocked(self):
        def corrupt(frame):
            frame.data[7] = 1
            return frame
        self.robots["right"].frame_transform = corrupt
        self.assert_no_tx(self.run_tool(grip=True))

    def test_no_new_status_after_send_cannot_be_counted_stable(self):
        frozen = self.clock.time()+3
        def hook(robot, state):
            if self.robots["right"].sent:
                state["fragment_timestamps_s"] = dict.fromkeys(state["fragment_timestamps_s"], frozen)
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["observed_stable"])

    def test_unknown_motion_and_teaching_values_refused(self):
        self.hook = lambda robot, state: state["arm_status"].update(teach_status=1)
        self.assert_no_tx(self.run_tool())

    def test_partial_fragment_advance_excursions_prevent_false_stability_and_timeout(self):
        robot = self.robots["right"]
        robot.motion.speed = 0
        def hook(current, state):
            if robot.motion.started is not None and current is robot:
                tick = round((self.clock.elapsed-robot.motion.started)/.01)
                # A fresh pose excursion occurs between complete fragment advances.
                group = tick//3
                key = next(iter(state["fragment_timestamps_s"]))
                state["fragment_timestamps_s"][key] = 1_800_000_000+robot.motion.started+group*.03
                if tick % 3 == 1:
                    state["pose_m_rad"][2] += .0006
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertIn("Observation timeout", result["errors"][0]["detail"])
        self.assertFalse(result["observed_stable"])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertGreaterEqual(self.clock.elapsed-robot.motion.started, 20)

    def test_unknown_motion_value_refused_without_coercion(self):
        self.hook = lambda robot, state: state["arm_status"].update(motion_status=True)
        self.assert_no_tx(self.run_tool())

    def test_wrong_jaw_force_byte_blocked(self):
        def corrupt(frame):
            frame.data[5] = 201
            return frame
        self.robots["right"].frame_transform = corrupt
        self.assert_no_tx(self.run_tool(grip=True))

    def test_duplicate_sdk_jaw_send_cannot_reach_bus(self):
        self.robots["right"].duplicate = True
        result = self.run_tool(grip=True)
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 1)


class RealSDKSupervisedTests(unittest.TestCase):
    def vendor_case(self, kind, fail=False):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, robots = Clock(), [], {}
        motions = {channel: Motion(clock) for channel in ("can0", "can2")}
        widths = {"can0": .05, "can2": .05}
        class FakeBus:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def send(self, frame, timeout=None):
                if fail:
                    raise OSError(errno.ENOBUFS, "mock bus failure")
                sent.append((self.channel, frame))
                motions[self.channel].sent(frame)
                if frame.arbitration_id == 0x159:
                    widths[self.channel] = int.from_bytes(frame.data[:4], "big", signed=True)/1e6
            def shutdown(self):
                pass
        factory, profile = sdk.AgxArmFactory.create_arm, copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.0
            channel = config["comm"]["can"]["channel"]
            robots[id(robot)] = channel
            motions[channel].origin = robot.fk([0, .5, -.5, 0, 0, 0])
            return robot
        def snapshot(robot, gripper):
            channel = robots[id(robot)]
            pose, motion = motions[channel].pose()
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1,
                teach_status=0, motion_status=motion, mode_feedback=motions[channel].mode, arm_status=0, err_code=0))
            state["pose_m_rad"], state["joints_rad"] = pose, [0, .5, -.5, 0, 0, 0]
            state["gripper"]["width_m"] = widths[channel]
            return state
        # Compute expected flange with the same pure manufacturer FK, before any connection.
        with patch("socket.socket", side_effect=AssertionError("Real sockets prohibited")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), patch.object(actions, "time", clock), \
             patch.object(linear_hold, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeBus(kw["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            if kind == "gripper":
                result = actions.gripper_once(profile, lambda *args: None, "right", .012345, .2)
            else:
                config = sdk.create_agx_arm_config(robot="piper", firmeware_version="default", channel="can2", auto_connect=False)
                passive = factory(config)
                target = passive.fk([0, .5, -.5, 0, 0, 0])
                target[2] += .006
                result = actions.move_once(profile, lambda *args: None, "right", target)
        self.assertEqual(result["ok"], not fail, result)
        self.assertTrue(all(channel == "can2" for channel, frame in sent))
        if not fail:
            self.assertEqual([f.arbitration_id for c, f in sent], [0x159] if kind == "gripper" else [0x151, 0x152, 0x153, 0x154])
            if kind == "gripper":
                self.assertEqual(bytes(sent[0][1].data), (12345).to_bytes(4, "big")+bytes((0, 200, 1, 0)))
            else:
                self.assertEqual([(f.arbitration_id, bytes(f.data)) for c, f in sent[1:]], linear_hold._pose_frames(target))
        return result

    def test_real_sdk_move_l_four_frame_encoding(self):
        self.vendor_case("move")

    def test_real_sdk_effector_encoder(self):
        self.vendor_case("gripper")

    def test_real_sdk_swallowed_bus_error_no_retry(self):
        self.vendor_case("gripper", fail=True)


if __name__ == "__main__":
    unittest.main()
