"""Optional motor RX diagnostics; no sockets, queries or robot construction."""
import copy
import json
import math
import struct
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms
from robot_tools.fault_feedback import FaultFeedback
from test_arms import FakeRobot, SDK_PATH


class MotorFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        for patcher in (patch("socket.socket", side_effect=AssertionError("No hardware or network")),
                        patch.object(arms.time, "time", return_value=self.now)):
            patcher.start()
            self.addCleanup(patcher.stop)
        self.robot = FakeRobot()
        for name in ("get_motor_states", "get_joint_angle_vel_limits", "connect", "init_effector"):
            setattr(self.robot, name, Mock(side_effect=AssertionError("No getter/query/construction")))

    def motors(self):
        for index in range(1, 7):
            setattr(self.robot._parser, "motor_state_%d" % index,
                    NS(timestamp=self.now-index*.01,
                       msg=NS(current=-.1*index, velocity=.02*index,
                              position=-.3*index, torque=-.4*index)))

    def snapshot(self):
        result = arms.snapshot(self.robot)
        json.dumps(result, allow_nan=False)
        return result

    def test_optional_absence_does_not_change_required_snapshot_or_health(self):
        state = self.snapshot()
        self.assertEqual(state["status"], "complete")
        self.assertTrue(arms.control_health(state, now_s=self.now)["healthy"])
        self.assertEqual(state["motor_feedback"]["status"], "unavailable")
        self.assertEqual(set(state["fragment_timestamps_s"]), set(arms.PARTS+arms.DRIVERS+("gripper",)))
        for motor in state["motor_feedback"]["motors"].values():
            self.assertEqual(motor["status"], "unavailable")
            for field in ("timestamp_s", "age_s", "current_A", "velocity_rad_s", "position_rad", "estimated_torque_Nm"):
                self.assertIsNone(motor[field])

    def test_six_decoded_values_and_independent_timestamps_are_copied_without_getters(self):
        self.motors()
        state = self.snapshot()
        feedback = state["motor_feedback"]
        self.assertEqual(feedback["status"], "complete")
        self.assertTrue(feedback["diagnostic_only"])
        self.assertFalse(feedback["contact_force_measured"])
        self.assertFalse(feedback["motion_permitted"])
        self.assertEqual(feedback["hardware_commands_sent"], 0)
        for index in range(1, 7):
            motor = feedback["motors"][str(index)]
            self.assertEqual(motor["status"], "fresh")
            self.assertEqual(motor["timestamp_s"], self.now-index*.01)
            self.assertAlmostEqual(motor["age_s"], index*.01)
            self.assertEqual(motor["current_A"], -.1*index)
            self.assertEqual(motor["velocity_rad_s"], .02*index)
            self.assertEqual(motor["position_rad"], -.3*index)
            self.assertEqual(motor["estimated_torque_Nm"], -.4*index)
        for name in ("get_motor_states", "get_joint_angle_vel_limits", "connect", "init_effector"):
            getattr(self.robot, name).assert_not_called()
        self.robot.comm.send.assert_not_called()
        self.robot.comm.send_bus.send.assert_not_called()

    def test_one_old_motor_is_stale_despite_five_fresh_motors(self):
        self.motors()
        self.robot._parser.motor_state_1.timestamp = 98.
        state = self.snapshot()
        feedback = state["motor_feedback"]
        self.assertEqual(feedback["status"], "partial")
        self.assertEqual(feedback["motors"]["1"]["status"], "stale")
        self.assertEqual(feedback["motors"]["1"]["age_s"], 2.)
        self.assertEqual(feedback["motors"]["6"]["status"], "fresh")
        self.assertEqual(state["stale_fragments"], [])
        self.assertEqual(state["status"], "complete")
        self.assertTrue(arms.control_health(state, now_s=self.now)["healthy"])

    def test_future_timestamp_is_stale_and_missing_motor_stays_unavailable(self):
        self.motors()
        self.robot._parser.motor_state_2.timestamp = self.now+1.
        self.robot._parser.motor_state_4 = None
        motor = self.snapshot()["motor_feedback"]["motors"]
        self.assertEqual(motor["2"]["status"], "stale")
        self.assertFalse(motor["2"]["fresh"])
        self.assertEqual(motor["4"]["status"], "unavailable")
        self.assertIsNone(motor["4"]["estimated_torque_Nm"])

    def test_invalid_timestamp_is_not_silently_refreshed(self):
        for value in (float("nan"), float("inf"), True, "99.9", None, 0, 10**1000):
            with self.subTest(value=repr(value)[:24]):
                self.motors()
                self.robot._parser.motor_state_3.timestamp = value
                state = self.snapshot()
                motor = state["motor_feedback"]["motors"]["3"]
                self.assertEqual(motor["status"], "partial")
                self.assertIsNone(motor["timestamp_s"])
                self.assertIsNone(motor["age_s"])
                self.assertIn("timestamp_s", motor["invalid_fields"])
                self.assertEqual(state["status"], "complete")

    def test_invalid_decoded_values_do_not_break_basic_snapshot_or_fault_diagnostics(self):
        self.motors()
        self.robot._parser.motor_state_2.msg.current = float("nan")
        self.robot._parser.motor_state_3.msg.velocity = True
        self.robot._parser.motor_state_4.msg.position = "unknown"
        self.robot._parser.motor_state_5.msg.torque = float("inf")
        del self.robot._parser.motor_state_6.msg.current
        state = self.snapshot()
        self.assertEqual(state["status"], "complete")
        for index, field in ((2, "current_A"), (3, "velocity_rad_s"), (4, "position_rad"),
                             (5, "estimated_torque_Nm"), (6, "current_A")):
            motor = state["motor_feedback"]["motors"][str(index)]
            self.assertEqual(motor["status"], "partial")
            self.assertIsNone(motor[field])
            self.assertIn(field, motor["invalid_fields"])
        report = FaultFeedback().capture({"left": state, "right": copy.deepcopy(state)}, self.now)
        json.dumps(report, allow_nan=False)
        self.assertEqual(report["arms"]["left"]["motor_feedback"], state["motor_feedback"])
        self.assertTrue(report["diagnostics"]["left"]["health"]["healthy"])
        self.assertIsNone(report["physical_stop_verified"])

    def test_optional_fragment_copy_and_property_errors_are_isolated(self):
        class BadCopy:
            def __deepcopy__(self, memo):
                raise RuntimeError("Synthetic malformed SDK fragment")
        class BadMessage:
            @property
            def current(self):
                raise ValueError("Synthetic malformed SDK attribute")
        self.motors()
        self.robot._parser.motor_state_1 = BadCopy()
        self.robot._parser.motor_state_2.msg = BadMessage()
        state = self.snapshot()
        self.assertEqual(state["status"], "complete")
        motors = state["motor_feedback"]["motors"]
        self.assertEqual(motors["1"]["read_error"], "RuntimeError")
        self.assertEqual(motors["2"]["read_error"], "ValueError")
        self.assertEqual(motors["6"]["status"], "fresh")

    def test_snapshot_does_not_share_mutable_motor_fragments(self):
        self.motors()
        before = self.snapshot()
        self.robot._parser.motor_state_1.msg.current = 9.
        after = self.snapshot()
        after["motor_feedback"]["motors"]["1"]["estimated_torque_Nm"] = 123.
        self.assertEqual(before["motor_feedback"]["motors"]["1"]["current_A"], -.1)
        self.assertEqual(self.robot._parser.motor_state_1.msg.torque, -.4)

    def test_real_official_piper_x_parser_scales_once_and_preserves_model_torque(self):
        sdk = arms._load_sdk(SDK_PATH)
        import can
        from pyAgxArm.api.constants import ROBOT_JOINT_TORQUE_K, ROBOT_JOINT_TORQUE_B, ROBOT_JOINT_TORQUE_C
        from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Parser
        from pyAgxArm.utiles.fps import FPSManager
        config = {"joint_torque_k": ROBOT_JOINT_TORQUE_K["piper_x"],
                  "joint_torque_b": ROBOT_JOINT_TORQUE_B["piper_x"],
                  "joint_torque_c": ROBOT_JOINT_TORQUE_C["piper_x"]}
        with patch.object(sdk.AgxArmFactory, "create_arm", side_effect=AssertionError("No SDK robot construction")):
            parser = Parser(FPSManager(), config=config)
            for index in range(1, 7):
                payload = struct.pack(">hhi", -100*index, 250*index, -1000*index)
                parser.parse_packet(can.Message(arbitration_id=0x250+index, data=payload,
                    timestamp=self.now-index*.01, is_extended_id=False))
                setattr(self.robot._parser, "motor_state_%d" % index,
                        getattr(parser, "motor_state_%d" % index))
            state = self.snapshot()
        self.assertEqual(state["motor_feedback"]["status"], "complete")
        for index in range(1, 7):
            row = state["motor_feedback"]["motors"][str(index)]
            self.assertAlmostEqual(row["velocity_rad_s"], -.1*index)
            self.assertAlmostEqual(row["current_A"], .25*index)
            self.assertAlmostEqual(row["position_rad"], -float(index))
            expected = .25*index * math.prod(config[key][index-1] for key in
                                           ("joint_torque_k", "joint_torque_b", "joint_torque_c"))
            self.assertAlmostEqual(row["estimated_torque_Nm"], expected)
            self.assertEqual(row["estimated_torque_Nm"], getattr(parser, "motor_state_%d" % index).msg.torque)


if __name__ == "__main__":
    unittest.main()
