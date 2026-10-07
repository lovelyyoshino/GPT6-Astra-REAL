"""Offline boundary-recovery protocol tests. Real sockets are forbidden."""
import copy
import errno
import math
import struct
import time
import unittest
from unittest.mock import patch

from robot_tools import arms, joint_recovery, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_takeover import FakeRobot, TakeoverFixture

Q = [-0.0429002, -0.0311541, 0.0419403, 0.0, 0.3905174, -0.153589]
TARGET = [Q[0], 0.0, 0.0, *Q[3:]]


def fk(q):
    from pyAgxArm.utiles.mdh_kinematics import get_mdh, fk_from_mdh
    return fk_from_mdh(get_mdh("piper"), q)


class Motion:
    def __init__(self, clock):
        self.clock, self.origin, self.target = clock, Q[:], None
        self.mode, self.started, self.raw = 0, None, {}
        self.accept, self.duration = True, 0.2

    def sent(self, frame):
        if frame.arbitration_id == 0x151:
            self.mode = frame.data[1]
        elif 0x155 <= frame.arbitration_id <= 0x157:
            self.raw[frame.arbitration_id] = struct.unpack(">ii", bytes(frame.data))
            if frame.arbitration_id == 0x157 and self.accept:
                raw = sum((list(self.raw[i]) for i in (0x155,0x156,0x157)), [])
                self.target = [math.radians(v/1000) for v in raw]
                self.started = self.clock.elapsed

    def feedback(self):
        if self.started is None:
            return self.origin[:], 0
        progress = min(1.0, (self.clock.elapsed-self.started)/self.duration)
        return [a+(b-a)*progress for a,b in zip(self.origin,self.target)], int(progress < 1)


class RecoveryRobot(FakeRobot):
    def __init__(self, side, clock):
        super().__init__(side)
        self.ctrl_mode = 1
        self.motion, self.auto_mode = Motion(clock), True
        self.fail_id, self.partial, self.move_calls = None, False, 0
        self.fk = fk

    def _bus_send(self, frame):
        if frame.arbitration_id == self.fail_id:
            raise OSError(errno.ENOBUFS,"simulated queue full")
        super()._bus_send(frame)
        self.motion.sent(frame)

    def set_auto_set_motion_mode_enabled(self, value):
        self.auto_mode = value

    def move_j(self, joints):
        self.move_calls += 1
        if self.auto_mode:
            self.set_motion_mode("j")
        raw = [round(math.degrees(v)*1000) for v in joints]
        for i in range(2 if self.partial else 3):
            frame = self.can.Message(arbitration_id=0x155+i, is_extended_id=False,
                                     data=struct.pack(">ii",*raw[2*i:2*i+2]))
            self._send_msg(self.frame_transform(frame))


class RecoveryTests(TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.profile = copy.deepcopy(PROFILE)
        for cfg in self.profile["arms"].values():
            cfg["model"] = "piper"
        self.robots = {side:RecoveryRobot(side,self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.stack.enter_context(patch.object(joint_recovery,"time",self.clock))

    def snapshot(self,robot,gripper):
        self.snapshot_count += 1
        q,motion = robot.motion.feedback()
        state = healthy_arm(self.clock.time())
        state["arm_status"].update(ctrl_mode=robot.ctrl_mode,teach_status=0,
                                  mode_feedback=robot.motion.mode,motion_status=motion)
        state["joints_rad"],state["pose_m_rad"] = q,fk(q)
        if self.hook:
            self.hook(robot,state)
        return state

    def run_tool(self,journal=None,target=None,arm="right"):
        return joint_recovery.recover_joint_boundary(self.profile,
            journal or (lambda event,data:self.events.append((event,data))),arm,TARGET[:] if target is None else target)

    def assert_no_fallback(self,result,count=4):
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151,0x155,0x156,0x157][:count])
        self.assertEqual(self.robots["left"].sent,[])
        self.assertEqual(result["enable_commands_sent"],0)
        self.assertEqual(result["stop_commands_sent"],0)
        self.assertFalse(result["motion_gate_unlocked"])

    def test_exact_four_frames_single_call_three_seconds_and_no_hold_claim(self):
        result = self.run_tool()
        self.assertTrue(result["ok"],result)
        self.assertTrue(result["recovery_observed"])
        self.assert_no_fallback(result)
        self.assertEqual(self.robots["right"].move_calls,1)
        self.assertEqual(bytes(self.robots["right"].sent[0].data),bytes((1,1,1,0,0,0,0,0)))
        self.assertEqual(result["target_calls_sent"],1)
        self.assertEqual(result["target_commands_sent"],3)
        self.assertGreaterEqual(result["stable_duration_s"],3)
        self.assertFalse(result["general_stop_validated"])
        self.assertFalse(result["limits_changed"])
        self.assertFalse(result["hard_path_guarantee"])
        self.assertTrue(result["hold_not_validated"])
        self.assertEqual(len(result["observed_boundary_violations"]["left"]),2)
        self.assertTrue(result["selected_arm_strictly_within_limits"])

    def test_left_selection_sends_only_left(self):
        result = self.run_tool(arm="left")
        self.assertTrue(result["ok"],result)
        self.assertEqual(len(self.robots["left"].sent),4)
        self.assertEqual(self.robots["right"].sent,[])

    def test_no_violation_is_not_general_movement_entry(self):
        self.robots["right"].motion.origin = TARGET[:]
        self.assert_no_tx(self.run_tool())

    def test_wrong_boundary_illegal_target_or_legal_joint_change_refused(self):
        for index,value in ((1,.001),(2,-.001),(1,-.001),(0,Q[0]+.004)):
            with self.subTest(index=index,value=value):
                target = TARGET[:]; target[index] = value
                result = self.run_tool(target=target)
                self.assertFalse(result["ok"],result)
                self.assertEqual(result["hardware_commands_sent"],0)
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_large_violation_and_large_fk_target_refused(self):
        self.robots["right"].motion.origin[1] = -.051
        self.assert_no_tx(self.run_tool())

    def test_fk_position_or_orientation_disagreement_refused(self):
        for index,delta in ((0,.0021),(3,.03)):
            with self.subTest(index=index):
                def bad_fk(q):
                    pose = fk(q); pose[index] += delta; return pose
                self.robots["right"].fk = bad_fk
                result = self.run_tool()
                self.assertFalse(result["ok"],result)
                self.assertEqual(result["hardware_commands_sent"],0)
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def test_target_fk_above_fifteen_mm_refused(self):
        self.robots["right"].motion.origin[2] = .05
        self.robots["right"].motion.origin[1] = -.001
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("15 mm",result["errors"][0]["detail"])

    def test_millidegree_quantization_must_not_cross_preset_boundary(self):
        self.robots["right"].motion.origin = [Q[0],0.0,0.0,0.0,1.222,-.153589]
        target = self.robots["right"].motion.origin[:]
        target[4] = 1.221730
        result = self.run_tool(target=target)
        self.assert_no_tx(result)
        self.assertIn("quantization",result["errors"][0]["detail"])

    def test_fresh_legal_joint_change_after_journal_refused(self):
        def journal(event,data):
            if event == "joint_recovery_intent":
                self.robots["right"].motion.origin[0] += .0031
        self.assert_no_tx(self.run_tool(journal))

    def test_j4_baseline_noise_does_not_relax_target_freshness(self):
        def hook(robot,state):
            if robot.side == "right" and self.clock.elapsed > .1:
                state["joints_rad"][3] += .006
        self.hook = hook
        result = self.run_tool()
        self.assert_no_tx(result)
        self.assertIn("Originally legal",result["errors"][0]["detail"])

    def test_disabled_gripper_or_driver_fault_refused(self):
        def hook(robot,state):
            state["gripper"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        self.assert_no_tx(self.run_tool())

    def test_partial_sequence_or_swallowed_bus_error_no_fallback(self):
        self.robots["right"].fail_id = 0x156
        self.robots["right"].swallow_error = True
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result,2)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"],3)
        self.assertFalse(result["recovery_observed"])

    def test_incomplete_sdk_sequence_not_retried(self):
        self.robots["right"].partial = True
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result,3)
        self.assertIn("Incomplete",result["errors"][0]["detail"])

    def test_duplicate_mode_or_wrong_target_frame_rejected(self):
        self.robots["right"].duplicate = True
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result,1)

    def test_injected_zero_calibration_or_other_frame_rejected(self):
        def transform(frame):
            if frame.arbitration_id == 0x155:
                frame.arbitration_id = 0x150
            return frame
        self.robots["right"].frame_transform = transform
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result,1)

    def test_active_envelope_or_other_arm_motion_ends_observation(self):
        for condition in ("joint","pose","other_arm","jaw","fault","disabled"):
            with self.subTest(condition=condition):
                self.robots = {side:RecoveryRobot(side,self.clock) for side in takeover.SIDES}
                self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
                def hook(robot,state):
                    if self.robots["right"].motion.started is not None:
                        if condition == "joint" and robot.side == "right": state["joints_rad"][0] += .004
                        if condition == "pose" and robot.side == "right": state["pose_m_rad"][0] += .021
                        if condition == "other_arm" and robot.side == "left": state["joints_rad"][0] += .004
                        if condition == "jaw": state["gripper"]["width_m"] += .003
                        if condition == "fault": state["drivers"]["2"]["foc_status"]["driver_error_status"] = True
                        if condition == "disabled": state["drivers"]["2"]["foc_status"]["driver_enable_status"] = False
                self.hook = hook
                result = self.run_tool()
                self.assertFalse(result["ok"],result)
                self.assert_no_fallback(result)

    def test_stale_active_feedback_is_not_success(self):
        def hook(robot,state):
            if self.robots["right"].motion.started is not None:
                state["fragment_timestamps_s"][arms.PARTS[0]] -= .06
        self.hook = hook
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result)
        self.assertIn("50 ms",result["errors"][0]["detail"])

    def test_right_j4_observation_tolerance_recorded_without_hold_claim(self):
        def hook(robot,state):
            if robot.side == "right" and robot.motion.started is not None:
                state["joints_rad"][3] += .006
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"],result)
        self.assertGreater(result["drift"]["right"]["joint_rad"],.006)
        self.assertTrue(result["hold_not_validated"])

    def test_tracking_tolerance_does_not_disguise_residual_limit_violation(self):
        def hook(robot,state):
            if robot.side == "right" and robot.motion.started is not None and state["arm_status"]["motion_status"] == 0:
                state["joints_rad"][1] = -.001
        self.hook = hook
        result = self.run_tool()
        self.assertTrue(result["ok"],result)
        self.assertFalse(result["selected_arm_strictly_within_limits"])
        self.assertEqual(result["final_boundary_violations"]["right"][0]["joint_index"],2)

    def test_no_motion_times_out_without_second_target(self):
        self.robots["right"].motion.accept = False
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assert_no_fallback(result)
        self.assertEqual(self.robots["right"].move_calls,1)

    def test_journal_failure_before_target_prevents_all_frames(self):
        def journal(event,data):
            if event == "joint_recovery_intent": raise OSError("disk full")
        self.assert_no_tx(self.run_tool(journal))

    def test_cleanup_failure_cannot_preserve_recovery_claim(self):
        self.robots["right"].disconnect.side_effect = OSError("cleanup failed")
        result = self.run_tool()
        self.assertFalse(result["ok"],result)
        self.assertFalse(result["recovery_observed"])


class RealSDKRecoveryTests(unittest.TestCase):
    def test_real_sdk_four_frame_move_j_and_fk_without_socket(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock,sent,robots = Clock(),[],{}
        motions = {channel:Motion(clock) for channel in ("can0","can2")}
        class FakeBus:
            def __init__(self,channel): self.channel = channel
            def recv(self,timeout=None): time.sleep(.001); return None
            def send(self,frame,timeout=None):
                sent.append((self.channel,frame)); motions[self.channel].sent(frame)
            def shutdown(self): pass
        original_factory = sdk.AgxArmFactory.create_arm
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values(): cfg["model"] = "piper"
        def create_robot(config):
            robot = original_factory(config)
            robots[id(robot)] = config["comm"]["can"]["channel"]
            return robot
        def snapshot(robot,gripper):
            motion = motions[robots[id(robot)]]
            q,moving = motion.feedback()
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1,teach_status=0,
                mode_feedback=motion.mode,motion_status=moving,arm_status=0,err_code=0))
            state["joints_rad"],state["pose_m_rad"] = q,robot.fk(q)
            return state
        with patch("socket.socket",side_effect=AssertionError("Real sockets forbidden")), \
             patch.object(arms,"_preflight"),patch.object(arms,"time",clock), \
             patch.object(takeover,"time",clock),patch.object(joint_recovery,"time",clock), \
             patch.object(can.interface,"Bus",side_effect=lambda **kwargs:FakeBus(kwargs["channel"])), \
             patch.object(sdk.AgxArmFactory,"create_arm",side_effect=create_robot), \
             patch.object(arms,"snapshot",side_effect=snapshot):
            result = joint_recovery.recover_joint_boundary(profile,lambda event,data:None,"right",TARGET[:])
        self.assertTrue(result["ok"],result)
        self.assertEqual([channel for channel,frame in sent],["can2"]*4)
        self.assertEqual([frame.arbitration_id for channel,frame in sent],[0x151,0x155,0x156,0x157])
        self.assertEqual(bytes(sent[0][1].data),bytes((1,1,1,0,0,0,0,0)))
        raw = [round(math.degrees(x)*1000) for x in TARGET]
        for i,(channel,frame) in enumerate(sent[1:]):
            self.assertEqual(bytes(frame.data),struct.pack(">ii",*raw[2*i:2*i+2]))
            self.assertEqual(frame.dlc,8)
            self.assertFalse(frame.is_extended_id)
        self.assertTrue(result["hold_not_validated"])
        self.assertFalse(result["motion_gate_unlocked"])


if __name__ == "__main__": unittest.main()
