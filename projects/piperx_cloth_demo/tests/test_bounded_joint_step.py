"""Offline contracts for one explicit J2/J3 adjustment; sockets forbidden."""
import copy
import math
import struct
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, bounded_joint_step as step, joint_recovery, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
import test_joint_recovery as recovery_fixture
from test_joint_recovery import Motion, RecoveryRobot
from test_takeover import TakeoverFixture

Q = [-0.04290019301402062, 0.0, -0.0011519173063162576, 0.0,
     0.39020326086837226, -0.15358897417550102]
TARGET = [Q[0], .015, -.015, *Q[3:]]


class BoundedStepTests(TakeoverFixture):
    snapshot = recovery_fixture.RecoveryTests.snapshot

    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        for cfg in self.profile["arms"].values(): cfg["model"] = "piper"
        self.robots = {side: RecoveryRobot(side, self.clock) for side in takeover.SIDES}
        for robot in self.robots.values(): robot.motion.origin = Q[:]
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.stack.enter_context(patch.object(joint_recovery, "time", self.clock))
        self.stack.enter_context(patch.object(step, "time", self.clock))

    def run_tool(self, journal=None, target=None, arm="right", clearance=.05, attachment=.3):
        return step.bounded_joint_step(self.profile,
            journal or (lambda event, data: self.events.append((event, data))), arm,
            TARGET[:] if target is None else target, attachment, clearance)

    def test_four_frames_pending_flag_preserved_baseline_and_completion_stable(self):
        def hook(robot, state):
            if robot.side == "right" and robot.motion.started is None:
                state["arm_status"]["motion_status"] = 1
                state["arm_status"]["mode_feedback"] = 2
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["step_observed"])
        self.assertEqual(result["baseline_motion_status"]["right"], 1)
        self.assertEqual(result["pending_target_state"], "unknown")
        self.assertFalse(result["previous_target_cancelled"])
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["stable_duration_s"], 3)
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151,0x155,0x156,0x157])
        self.assertEqual(bytes(self.robots["right"].sent[0].data), bytes((1,1,1,0,0,0,0,0)))
        self.assertEqual(self.robots["right"].move_calls, 1)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertFalse(result["motion_gate_unlocked"])
        self.assertTrue(result["hold_not_validated"])
        self.assertFalse(result["general_stop_validated"])
        self.assertEqual(result["stop_commands_sent"], 0)

    def test_sweep_includes_all_axes_attachment_body_and_anchor(self):
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        radii = [1.13176,1.00876,.72373,.70175,.451,.451]
        for actual, expected in zip(result["sweep_axis_radii_m"], radii):
            self.assertAlmostEqual(actual, expected, places=7)
        excursions = [abs(a-b)+.003 for a,b in zip(result["encoded_target_joints_rad"],Q)]
        self.assertEqual(result["sweep_reference_joints_rad"], Q)
        self.assertAlmostEqual(result["sweep_bound_m"], sum(a*b for a,b in zip(radii,excursions)))
        self.assertLess(result["sweep_bound_m"], .045)

    def test_small_endpoint_does_not_bypass_whole_chain_clearance(self):
        result = self.run_tool(clearance=.02)
        self.assert_no_tx(result)
        self.assertIn("sweep bound", result["errors"][0]["detail"])

    def test_only_j2_j3_change_and_margin_and_amplitude_enforced(self):
        cases = ((0,Q[0]+.00002),(3,.00002),(4,Q[4]+.001),(5,Q[5]+.001),
                 (1,.026),(2,-.027),(1,.009),(2,-.009))
        for index,value in cases:
            with self.subTest(index=index,value=value):
                target = TARGET[:]; target[index] = value
                result = self.run_tool(target=target)
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["hardware_commands_sent"], 0)
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_unchanged_axis_rounding_half_lsb_and_fresh_journal_check(self):
        target = TARGET[:]; target[0] += 1e-7
        self.assertTrue(self.run_tool(target=target)["ok"])

    def test_journal_changes_fixed_axis_refuses_before_send(self):
        def journal(event, data):
            if event == "joint_recovery_intent":
                self.robots["right"].motion.origin[0] += .0001
        self.assert_no_tx(self.run_tool(journal))

    def test_baseline_motion_or_stale_or_inactive_pending_refused(self):
        for condition in ("xyz", "joint", "stale", "inactive_pending", "fault"):
            with self.subTest(condition=condition):
                def hook(robot, state):
                    if condition == "xyz" and self.clock.elapsed > .5: state["pose_m_rad"][0] += .00051
                    if condition == "joint" and self.clock.elapsed > .5: state["joints_rad"][3] += .0031
                    if condition == "stale": state["fragment_timestamps_s"][arms.PARTS[0]] -= .051
                    if condition == "inactive_pending" and robot.side == "left": state["arm_status"]["motion_status"] = 1
                    if condition == "fault": state["arm_status"]["arm_status"] = 4
                self.hook = hook
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(result["hardware_commands_sent"], 0)
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_selected_out_of_bounds_refused_but_static_inactive_reported(self):
        self.robots["left"].motion.origin[1] = -.02
        result = self.run_tool()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["observed_boundary_violations"]["left"][0]["joint_index"], 2)
        self.assertEqual(self.robots["left"].sent, [])

    def test_selected_out_of_bounds_refused(self):
        self.robots["right"].motion.origin[1] = -.001
        self.assert_no_tx(self.run_tool())

    def test_fk_mismatch_refused(self):
        original = self.robots["right"].fk
        def bad_fk(q):
            pose = original(q); pose[0] += .003; return pose
        self.robots["right"].fk = bad_fk
        self.assert_no_tx(self.run_tool())

    def test_active_drift_fault_or_inactive_motion_never_sends_again(self):
        for condition in ("joint", "inactive", "fault", "jaw"):
            with self.subTest(condition=condition):
                self.robots = {side: RecoveryRobot(side,self.clock) for side in takeover.SIDES}
                for robot in self.robots.values(): robot.motion.origin = Q[:]
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                def hook(robot, state):
                    if self.robots["right"].motion.started is not None:
                        if condition == "joint" and robot.side == "right": state["joints_rad"][3] += .0031
                        if condition == "inactive" and robot.side == "left": state["joints_rad"][3] += .0031
                        if condition == "fault": state["arm_status"]["arm_status"] = 4
                        if condition == "jaw": state["gripper"]["width_m"] += .0021
                self.hook = hook
                result = self.run_tool()
                self.assertFalse(result["ok"], result)
                self.assertEqual(len(self.robots["right"].sent), 4)
                self.assertEqual(self.robots["left"].sent, [])
                self.assertFalse(result["step_observed"])

    def test_swallowed_partial_send_error_never_retried(self):
        self.robots["right"].fail_id = 0x156
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(len(self.robots["right"].sent), 2)
        self.assertEqual(self.robots["right"].move_calls, 1)
        self.assertEqual(self.robots["left"].sent, [])

    def test_unexpected_or_duplicate_frame_blocked_without_retry(self):
        self.robots["right"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertEqual(self.robots["right"].move_calls, 1)
        self.assertEqual(self.robots["left"].sent, [])

    def test_persistent_not_arrived_cannot_succeed_or_retry(self):
        def hook(robot, state):
            if robot.side == "right": state["arm_status"]["motion_status"] = 1
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"], result)
        self.assertFalse(result["step_observed"])
        self.assertEqual(self.robots["right"].move_calls, 1)

    def test_cleanup_failure_revokes_success(self):
        self.robots["right"].disconnect.side_effect = OSError("cleanup")
        result = self.run_tool()
        self.assertFalse(result["ok"])
        self.assertFalse(result["step_observed"])

    def test_invalid_physical_inputs_or_nonfinite_target_no_connection(self):
        for attachment,clearance in ((True,.05),(.3,False),(-.3,.05),(.3,math.nan),(math.inf,.05)):
            with self.subTest(attachment=attachment,clearance=clearance):
                with self.assertRaises(ValueError): self.run_tool(attachment=attachment,clearance=clearance)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()


class RealSDKStepTests(unittest.TestCase):
    def run_real_sdk(self, target):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, channels = Clock(), [], {}
        motions = {channel: Motion(clock) for channel in ("can0","can2")}
        for motion in motions.values(): motion.origin = Q[:]
        class FakeBus:
            def __init__(self,channel): self.channel = channel
            def recv(self,timeout=None): time.sleep(.001); return None
            def send(self,frame,timeout=None):
                sent.append((self.channel,frame)); motions[self.channel].sent(frame)
            def shutdown(self): pass
        original_factory = sdk.AgxArmFactory.create_arm
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values(): cfg["model"] = "piper"
        def create(config):
            robot = original_factory(config)
            channels[id(robot)] = config["comm"]["can"]["channel"]
            return robot
        def snapshot(robot,gripper):
            motion = motions[channels[id(robot)]]
            q,moving = motion.feedback()
            if channels[id(robot)] == "can2" and motion.started is None: moving = 1
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1,teach_status=0,
                mode_feedback=motion.mode,motion_status=moving,arm_status=0,err_code=0))
            state["joints_rad"],state["pose_m_rad"] = q,robot.fk(q)
            return state
        with patch("socket.socket",side_effect=AssertionError("Real sockets forbidden")), \
             patch.object(arms,"_preflight"),patch.object(arms,"time",clock), \
             patch.object(takeover,"time",clock),patch.object(joint_recovery,"time",clock),patch.object(step,"time",clock), \
             patch.object(can.interface,"Bus",side_effect=lambda **kwargs:FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory,"create_arm",side_effect=create),patch.object(arms,"snapshot",side_effect=snapshot):
            result = step.bounded_joint_step(profile,lambda event,data:None,"right",target,.3,.05)
        return result, sent

    def test_manufacturer_move_j_encoding_fk_enums_with_fake_bus_only(self):
        result, sent = self.run_real_sdk(TARGET[:])
        self.assertTrue(result["ok"], result)
        self.assertEqual([channel for channel,frame in sent], ["can2"]*4)
        self.assertEqual([frame.arbitration_id for channel,frame in sent], [0x151,0x155,0x156,0x157])
        raw = [round(math.degrees(value)*1000) for value in TARGET]
        for i,(channel,frame) in enumerate(sent[1:]):
            self.assertEqual(bytes(frame.data),struct.pack(">ii",*raw[2*i:2*i+2]))
            self.assertFalse(frame.is_extended_id)
        self.assertEqual(bytes(sent[0][1].data),bytes((1,1,1,0,0,0,0,0)))
        self.assertTrue(result["hold_not_validated"])

    def test_half_millidegree_expression_disagreement_refused_before_any_frame(self):
        target = TARGET[:]
        target[1] = 0.010306169233026517
        self.assertNotEqual(round(target[1]*180/math.pi*1000), round(target[1]*(180/math.pi)*1000))
        result, sent = self.run_real_sdk(target)
        self.assertFalse(result["ok"], result)
        self.assertEqual(sent, [])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("encodes inconsistently", result["errors"][0]["detail"])


if __name__ == "__main__": unittest.main()
