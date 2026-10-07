"""Offline tests: vendor parsing only; CAN creation and hardware sends forbidden."""

import contextlib
import importlib.util
import logging
from pathlib import Path
import struct
import time
import unittest
from unittest import mock


ADAPTER_PATH = Path(__file__).resolve().parents[1] / "scripts" / "vendor_sdk_feedback.py"
SPEC = importlib.util.spec_from_file_location("vendor_sdk_feedback_under_test", ADAPTER_PATH)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)

try:
    import can
    from piper_sdk import C_PiperInterface_V2
    from piper_sdk.hardware_port import C_STD_CAN
except ImportError:
    C_PiperInterface_V2 = None


@unittest.skipIf(C_PiperInterface_V2 is None, "Run with the installed vendor SDK Python 3.8 environment")
class VendorSDKFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.forbidden = contextlib.ExitStack()
        self.addCleanup(self.forbidden.close)
        for owner, name in ((can.interface, "Bus"),
                            (C_PiperInterface_V2, "CreateCanBus"),
                            (C_PiperInterface_V2, "ConnectPort"),
                            (C_STD_CAN, "SendCanMessage")):
            self.forbidden.enter_context(mock.patch.object(
                owner, name, side_effect=AssertionError("Hardware access forbidden: " + name)))
        self.arm = ADAPTER.create_arm()
        self.addCleanup(self.arm.close_error_capture)
        self.assertIsNone(self.arm.GetCanBus())
        self.stamp = time.time() - 5.0

    def feed(self, ident, data, stamp=None):
        self.arm.ParseCANFrame(can.Message(
            arbitration_id=ident, data=data,
            timestamp=self.stamp if stamp is None else stamp, is_extended_id=False))

    def populate(self):
        self.feed(0x2A1, bytes([1, 4, 0, 0, 0, 0, 0, 0]))
        for ident in range(0x2A2, 0x2A8):
            self.feed(ident, struct.pack(">ii", 110000, 69982))
        self.feed(0x2A8, struct.pack(">ihBB", 54320, 300, 192, 0))
        for ident in range(0x261, 0x267):
            self.feed(ident, struct.pack(">HhbBH", 240, 20, 20, 64, 100))
        self.assertIsNone(self.arm.fatal_error)

    def test_factory_never_opens_hardware(self):
        self.assertEqual(self.arm.GetCanName(), "can2")
        self.assertFalse(self.arm.get_connect_status())
        self.assertEqual(self.arm.snapshot()["rx"], {})

    def test_pose_and_joint_fragments_remain_independently_stale(self):
        self.populate()
        old = self.arm.snapshot()
        fresh = time.time()
        self.feed(0x2A2, struct.pack(">ii", 112000, 0), fresh)
        self.feed(0x2A5, struct.pack(">ii", 1000, 2000), fresh)
        current = self.arm.snapshot()
        for ident in (0x2A2, 0x2A5):
            self.assertEqual(current["rx_wall"][str(ident)], fresh)
        for ident in (0x2A3, 0x2A4, 0x2A6, 0x2A7):
            self.assertEqual(current["rx_wall"][str(ident)], self.stamp)
            self.assertEqual(current["rx"][str(ident)], old["rx"][str(ident)])
            self.assertGreater(current["time_s"] - current["rx_wall"][str(ident)], 4.0)

    def test_each_motor_has_its_own_freshness(self):
        self.populate()
        old = self.arm.snapshot()
        fresh = time.time()
        self.feed(0x261, struct.pack(">HhbBH", 240, 20, 20, 64, 100), fresh)
        current = self.arm.snapshot()
        self.assertEqual(current["rx_wall"][str(0x261)], fresh)
        for ident in range(0x262, 0x267):
            self.assertEqual(current["rx_wall"][str(ident)], self.stamp)
            self.assertEqual(current["rx"][str(ident)], old["rx"][str(ident)])

    def test_snapshot_is_detached_and_preserves_rejected_status(self):
        self.populate()
        old = self.arm.snapshot()
        self.feed(0x2A1, bytes([1, 0, 0, 0, 0, 0, 0, 0]), time.time())
        self.feed(0x2A2, struct.pack(">ii", 112000, 0), time.time())
        new = self.arm.snapshot()
        self.assertEqual(old["status"]["arm_status"], 4)
        self.assertEqual(old["status"]["motion_status"], 0)
        self.assertEqual(old["status"]["err_code"], 0)
        self.assertEqual(new["status"]["arm_status"], 0)
        self.assertEqual(old["pose_mm_deg"][0], 110.0)
        self.assertEqual(new["pose_mm_deg"][0], 112.0)
        self.assertEqual(old["gripper"], {
            "opening_mm": 54.32, "effort_nm": 0.3, "status_code": 192})
        new["rx_wall"].clear()
        new["motor_codes"][0] = 0
        self.assertEqual(len(self.arm.snapshot()["rx_wall"]), 14)
        self.assertEqual(self.arm.snapshot()["motor_codes"][0], 64)

    def test_filtered_frame_sets_fatal_without_refreshing_cached_values(self):
        self.populate()
        old = self.arm.snapshot()
        self.feed(0x2A2, struct.pack(">ii", 1000001, 0), time.time())
        current = self.arm.snapshot()
        self.assertIn("SDK rejected feedback", self.arm.fatal_error)
        self.assertEqual(current["rx_wall"][str(0x2A2)], old["rx_wall"][str(0x2A2)])
        self.assertEqual(current["pose_mm_deg"], old["pose_mm_deg"])

    def test_error_frame_sets_fatal_before_vendor_parsing(self):
        self.arm.ParseCANFrame(can.Message(
            arbitration_id=0x2A1, data=bytes(8), timestamp=time.time(),
            is_extended_id=False, is_error_frame=True))
        self.assertIn("Invalid feedback CAN frame", self.arm.fatal_error)
        self.assertEqual(self.arm.snapshot()["rx"], {})

    def test_sdk_error_log_makes_send_checked_fail(self):
        def rejected_send(*args):
            self.arm.logger.error("simulated vendor send failure")
            return None
        with mock.patch.object(C_PiperInterface_V2, "MotionCtrl_2", side_effect=rejected_send):
            with self.assertRaisesRegex(RuntimeError, "simulated vendor send failure"):
                self.arm.send_checked("MotionCtrl_2", 1, 0, 5, 0)
        self.assertEqual(len(self.arm.send_errors), 1)

    def test_hardware_error_log_is_captured_once(self):
        logging.getLogger("can.interfaces.socketcan.socketcan").error("simulated CAN failure")
        self.assertEqual(len(self.arm.send_errors), 1)
        self.assertEqual(self.arm.send_errors[0]["message"], "simulated CAN failure")

    def test_reset_joint_commands_and_non_stop_motionctrl1_are_forbidden(self):
        for method, args in (("ResetPiper", ()), ("JointCtrl", (0,) * 6),
                             ("EnablePiper", ()), ("MotionCtrl_1", (2, 0, 0)),
                             ("MotionCtrl_1", (1, 1, 0))):
            with self.subTest(method=method, args=args):
                with self.assertRaises(ValueError):
                    self.arm.send_checked(method, *args)

    def test_successful_sdk_return_value_is_not_fabricated(self):
        with mock.patch.object(C_PiperInterface_V2, "MotionCtrl_1", return_value=None) as method:
            self.assertIsNone(self.arm.send_checked("MotionCtrl_1", 1, 0, 0))
            method.assert_called_once_with(1, 0, 0)


if __name__ == "__main__":
    unittest.main()
