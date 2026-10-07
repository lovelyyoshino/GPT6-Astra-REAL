"""Offline arm adapter checks. These tests must never open a real CAN socket."""
import math
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms

SDK_PATH = "/home/agilex/pyAgxArm"
CONFIG = {"left": {"model": "piper_x", "firmware": "default", "channel": "can0", "usb_interface": "1-6.1:1.0"},
          "right": {"model": "piper_x", "firmware": "default", "channel": "can2", "usb_interface": "1-6.3:1.0"}}


class FakeRobot:
    def __init__(self, missing=None, stale=None, transmit_on_connect=False):
        self._parser = NS()
        stamp = time.time() - 0.01
        fields = {"arm_status": {"ctrl_mode": 1, "arm_status": 0, "motion_status": 0,
                                  "err_code": 0, "err_status": NS(**dict.fromkeys(arms.ARM_ERRORS, False))},
                  "joint_12": {"joint_1": 0.1, "joint_2": 0.2},
                  "joint_34": {"joint_3": -0.3, "joint_4": 0.4},
                  "joint_56": {"joint_5": 0.5, "joint_6": 0.6},
                  "end_pose_xy": {"X_axis": 0.2, "Y_axis": 0.1},
                  "end_pose_zrx": {"Z_axis": 0.3, "RX_axis": 0.4},
                  "end_pose_ryrz": {"RY_axis": 0.5, "RZ_axis": 0.6}}
        fields.update({name: {"foc_status": NS(driver_enable_status=True,
                      **dict.fromkeys(arms.DRIVER_ERRORS, False))} for name in arms.DRIVERS})
        for name, values in fields.items():
            if name != missing:
                setattr(self._parser, name, NS(msg=NS(**values), timestamp=stamp - (2 if name == stale else 0)))
        self._effector = NS(_parser=NS())
        if missing != "gripper":
            self._effector._parser.gripper = NS(
                timestamp=stamp - (2 if stale == "gripper" else 0),
                msg=NS(value=0.055, force=0.3, mode="width", status_code=0xC0,
                       foc_status=NS(driver_enable_status=True, homing_status=True,
                                     **dict.fromkeys(arms.GRIPPER_ERRORS, False))))
        self.comm = NS(send=Mock(), send_bus=NS(send=Mock()))
        self.disconnected = False
        self.transmit_on_connect = transmit_on_connect

    def connect(self):
        if self.transmit_on_connect:
            self._send_msg("would transmit")

    def init_effector(self, kind):
        assert kind == "agx_gripper"
        return self._effector

    def get_context(self):
        return NS(get_comm=lambda: self.comm)

    def has_comm_error(self):
        return False

    def disconnect(self):
        self.disconnected = True


class ArmsTests(unittest.TestCase):
    def read_fake(self, robots):
        sdk = NS(create_agx_arm_config=lambda **kw: kw,
                 AgxArmFactory=NS(create_arm=Mock(side_effect=robots)))
        with patch.object(arms, "_preflight"), patch.object(arms, "_load_sdk", return_value=sdk):
            return arms.read_arms(CONFIG, SDK_PATH, timeout_s=0.05)

    def test_complete_units_and_no_transmit(self):
        robots = [FakeRobot(), FakeRobot()]
        report = self.read_fake(robots)
        self.assertEqual(report["status"], "complete")
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertEqual(report["arms"]["left"]["pose_m_rad"], [0.2, 0.1, 0.3, 0.4, 0.5, 0.6])
        self.assertEqual(report["arms"]["left"]["joints_rad"], [0.1, 0.2, -0.3, 0.4, 0.5, 0.6])
        self.assertEqual(report["arms"]["left"]["gripper"]["status"], "complete")
        self.assertEqual(report["arms"]["left"]["gripper"]["width_m"], 0.055)
        for robot in robots:
            self.assertTrue(robot.disconnected)
            with self.assertRaises(RuntimeError):
                robot.comm.send("forbidden")
            with self.assertRaises(RuntimeError):
                robot._effector._send_msg("forbidden gripper TX")

    def test_missing_pose_fragment_cannot_be_complete(self):
        report = self.read_fake([FakeRobot(missing="end_pose_xy"), FakeRobot()])
        self.assertEqual(report["status"], "partial")
        self.assertIsNone(report["arms"]["left"]["pose_m_rad"])

    def test_missing_motor_state_cannot_be_complete(self):
        report = self.read_fake([FakeRobot(missing="driver_state_6"), FakeRobot()])
        self.assertEqual(report["arms"]["left"]["status"], "partial")

    def test_missing_gripper_keeps_arm_completion_separate(self):
        report = self.read_fake([FakeRobot(missing="gripper"), FakeRobot()])
        state = report["arms"]["left"]
        self.assertTrue(state["arm_telemetry_complete"])
        self.assertEqual(state["status"], "partial")
        self.assertEqual(state["gripper"]["status"], "unavailable")
        self.assertIn("gripper", state["missing_fragments"])
        self.assertFalse(arms.control_health(state)["healthy"])

    def test_stale_gripper_cannot_hide_behind_fresh_arm(self):
        state = arms.snapshot(FakeRobot(stale="gripper"))
        self.assertTrue(state["arm_telemetry_complete"])
        self.assertEqual(state["gripper"]["status"], "stale")
        self.assertEqual(state["status"], "partial")

    def test_healthy_feedback_is_not_motion_permission(self):
        health = arms.control_health(arms.snapshot(FakeRobot()))
        self.assertTrue(health["healthy"], health)
        self.assertEqual(health["reasons"], [])
        self.assertFalse(health["motion_permitted"])

    def test_complete_cached_snapshot_is_rechecked_at_current_time(self):
        state = arms.snapshot(FakeRobot())
        self.assertEqual(state["status"], "complete")
        health = arms.control_health(state, now_s=state["timestamp"] + 2)
        self.assertFalse(health["healthy"])
        self.assertIn("stale_feedback", [r["code"] for r in health["reasons"]])

    def test_future_feedback_timestamp_is_unhealthy(self):
        state = arms.snapshot(FakeRobot())
        state["fragment_timestamps_s"]["driver_state_6"] = state["timestamp"] + 2
        health = arms.control_health(state)
        self.assertFalse(health["healthy"])
        self.assertIn("driver_state_6", [r["field"] for r in health["reasons"]])

    def test_rejected_arm_status_overrides_reached_and_zero_error_code(self):
        robot = FakeRobot()
        robot._parser.arm_status.msg.arm_status = 4
        state = arms.snapshot(robot)
        self.assertEqual(state["arm_status"]["motion_status"], 0)
        self.assertEqual(state["arm_status"]["err_code"], 0)
        health = arms.control_health(state)
        self.assertFalse(health["healthy"])
        self.assertIn("arm_status.arm_status", [r["field"] for r in health["reasons"]])

    def test_not_reached_motion_flag_alone_is_not_hardware_fault(self):
        robot = FakeRobot()
        robot._parser.arm_status.msg.motion_status = 1
        self.assertTrue(arms.control_health(arms.snapshot(robot))["healthy"])

    def test_wrong_control_mode_and_disabled_driver_rejected(self):
        robot = FakeRobot()
        robot._parser.arm_status.msg.ctrl_mode = 2
        robot._parser.driver_state_5.msg.foc_status.driver_enable_status = False
        health = arms.control_health(arms.snapshot(robot))
        self.assertFalse(health["healthy"])
        codes = [r["code"] for r in health["reasons"]]
        self.assertIn("control_mode", codes)
        self.assertIn("driver_disabled_or_unknown", codes)

    def test_each_manufacturer_driver_fault_flag_is_checked(self):
        for field in arms.DRIVER_ERRORS:
            with self.subTest(field=field):
                robot = FakeRobot()
                setattr(robot._parser.driver_state_6.msg.foc_status, field, True)
                health = arms.control_health(arms.snapshot(robot))
                self.assertFalse(health["healthy"])
                self.assertIn("drivers.6.foc_status." + field, [r["field"] for r in health["reasons"]])

    def test_missing_driver_error_flags_are_unknown_not_false(self):
        robot = FakeRobot()
        del robot._parser.driver_state_1.msg.foc_status.collision_status
        self.assertFalse(arms.control_health(arms.snapshot(robot))["healthy"])

    def test_gripper_homing_true_is_normal_but_sensor_fault_is_not(self):
        robot = FakeRobot()
        self.assertTrue(arms.control_health(arms.snapshot(robot))["healthy"])
        robot._effector._parser.gripper.msg.foc_status.sensor_status = True
        self.assertFalse(arms.control_health(arms.snapshot(robot))["healthy"])

    def test_gripper_angle_never_mislabeled_as_width(self):
        robot = FakeRobot()
        robot._effector._parser.gripper.msg.mode = "angle"
        robot._effector._parser.gripper.msg.value = 25.0
        state = arms.snapshot(robot)
        self.assertEqual(state["gripper"]["value_unit"], "deg")
        self.assertIsNone(state["gripper"]["width_m"])
        self.assertEqual(state["gripper"]["angle_deg"], 25.0)
        self.assertFalse(arms.control_health(state)["healthy"])

    def test_actual_vendor_gripper_parser_units_and_status_bits(self):
        arms._load_sdk(SDK_PATH)
        import can
        from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.parser import Parser
        from pyAgxArm.utiles.fps import FPSManager
        parser = Parser(FPSManager())
        data = (55000).to_bytes(4, "big", signed=True) + (300).to_bytes(2, "big", signed=True) + bytes([0xC0, 0])
        parser.parse_packet(can.Message(arbitration_id=0x2A8, data=data,
                                       timestamp=time.time(), is_extended_id=False))
        robot = FakeRobot()
        state = arms.snapshot(robot, NS(_parser=parser))
        self.assertAlmostEqual(state["gripper"]["width_m"], 0.055)
        self.assertAlmostEqual(state["gripper"]["force_N"], 0.3)
        self.assertTrue(state["gripper"]["foc_status"]["driver_enable_status"])
        self.assertTrue(state["gripper"]["foc_status"]["homing_status"])
        self.assertTrue(arms.control_health(state)["healthy"])

    def test_stale_first_joint_fragment_not_hidden_by_last(self):
        report = self.read_fake([FakeRobot(stale="joint_12"), FakeRobot()])
        self.assertEqual(report["status"], "partial")
        self.assertIn("joint_12", report["arms"]["left"]["stale_fragments"])

    def test_cross_arm_skew_is_checked(self):
        robots = [FakeRobot(), FakeRobot()]
        for fragment in vars(robots[0]._parser).values():
            fragment.timestamp -= 0.2
        report = self.read_fake(robots)
        self.assertEqual(report["status"], "partial")
        self.assertGreater(report["max_cross_arm_fragment_skew_s"], 0.1)

    def test_factory_failure_cleans_up_other_arm(self):
        robot = FakeRobot()
        report = self.read_fake([robot, RuntimeError("factory failed")])
        self.assertEqual(report["status"], "partial")
        self.assertEqual(report["arms"]["right"]["status"], "failed")
        self.assertTrue(robot.disconnected)

    def test_cleanup_failure_downgrades_complete(self):
        robot = FakeRobot()
        robot.disconnect = Mock(side_effect=RuntimeError("cleanup failed"))
        report = self.read_fake([robot, FakeRobot()])
        self.assertEqual(report["status"], "partial")
        self.assertEqual(report["arms"]["left"]["status"], "partial")
        self.assertEqual(report["arms"]["left"]["cleanup_error"], "cleanup failed")

    def test_connect_send_intercepted_before_command(self):
        robots = [FakeRobot(transmit_on_connect=True), FakeRobot()]
        report = self.read_fake(robots)
        self.assertEqual(report["arms"]["left"]["status"], "failed")
        self.assertIn("TX is forbidden", report["arms"]["left"]["error"])
        self.assertTrue(robots[0].disconnected)

    def test_distinct_channels_required_before_sdk(self):
        config = {"left": CONFIG["left"], "right": CONFIG["left"]}
        with patch.object(arms, "_load_sdk") as load, self.assertRaises(ValueError):
            arms.read_arms(config, SDK_PATH)
        load.assert_not_called()

    def test_wrong_usb_binding_rejected_before_sockets_or_sdk(self):
        with patch.object(arms.Path, "read_text", return_value="280"), \
             patch.object(arms.Path, "resolve", return_value=arms.Path("/fake/9-9:1.0")), \
             patch.object(arms.socket, "socket") as socket_open, \
             patch.object(arms, "_load_sdk") as sdk:
            result = arms.read_arms(CONFIG, SDK_PATH)
        self.assertEqual(result["status"], "failed")
        self.assertIn("binding mismatch", result["error"])
        socket_open.assert_not_called()
        sdk.assert_not_called()

    def test_vendor_fk_does_not_create_bus_or_arm(self):
        sdk = arms._load_sdk(SDK_PATH)
        with patch.object(arms.socket, "socket", side_effect=AssertionError("No socket")), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=AssertionError("No instance")):
            result = arms.vendor_fk("piper_x", [0.0] * 6, SDK_PATH)
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(len(result["pose_m_rad"]), 6)
        self.assertTrue(all(math.isfinite(v) for v in result["pose_m_rad"]))
        self.assertEqual(result["reference"], "flange")

    def test_non_finite_fk_rejected(self):
        with self.assertRaises(ValueError):
            arms.vendor_fk("piper_x", [float("nan")] * 6, SDK_PATH)

    def test_actual_sdk_connect_read_disconnect_never_transmits(self):
        arms._load_sdk(SDK_PATH)
        import can
        sent = []

        class FakeBus:
            def recv(self, timeout=None):
                time.sleep(0.002)
                return None

            def send(self, frame, timeout=None):
                sent.append(frame)

            def shutdown(self):
                pass

        with patch.object(arms, "_preflight"), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus()), \
             patch.object(arms.socket, "socket", side_effect=AssertionError("No real socket")):
            report = arms.read_arms(CONFIG, SDK_PATH, timeout_s=0.05)
        self.assertEqual(report["status"], "partial")
        self.assertEqual(sent, [])
        self.assertEqual(report["arms"]["right"]["status"], "partial")


if __name__ == "__main__":
    unittest.main()
