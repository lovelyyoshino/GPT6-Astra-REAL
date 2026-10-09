"""Failure receipts use real temporary SQLite and memory-only device doubles."""
from enum import IntEnum
import math
import unittest
from unittest.mock import patch

import numpy as np

from robot_tools.host_recovery import SupportedRecovery
import test_pair_host as host_tests


class Mode(IntEnum):
    CAN = 1


class PairFailurePersistenceTests(unittest.TestCase):
    def setUp(self):
        self.fixture = host_tests.PairHostTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.host = self.fixture.host
        self.host.open()

    def preparation_failure(self, value, *, finish_error=None):
        event_id = "synthetic-failed-opening"
        original = {"ok": False, "hardware_commands_sent": 1,
                    "pose_error": {"rotation_rad": value},
                    "before": {"arm_status": {"ctrl_mode": Mode.CAN}},
                    "candidate_measurement": {"anchor": {"joints_rad": [0.] * 6}},
                    "errors": [{"type": "RuntimeError", "detail": "Synthetic body drift"}]}
        request = {"operation": "supported_recovery_open"}
        payload = {"kind": "supported_recovery_open", "request": request}
        with patch.object(SupportedRecovery, "execute", return_value=original) as execute:
            if finish_error is None:
                self.host._start_preparation(event_id, request, payload)
                result = self.host.wait(event_id)
            else:
                with patch.object(self.host.ledger, "finish", side_effect=finish_error) as finish:
                    self.host._start_preparation(event_id, request, payload)
                    result = self.host.wait(event_id)
                    self.assertEqual(finish.call_count, 1)
            self.assertEqual(execute.call_count, 1)
        self.assertEqual(self.fixture.devices[0].frame_attempts, 0)
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertFalse(result["receipt"]["automatic_retry"])
        self.assertIsNone(result["receipt"]["physical_stop_verified"])
        return event_id, result, original

    def test_failed_preparation_normalizes_sdk_enum_and_finite_numpy_scalar(self):
        event_id, result, original = self.preparation_failure(np.float64(.001718403219517419))
        event = self.host.ledger.event(event_id)
        self.assertEqual(event["status"], "complete")
        self.assertEqual(event["success"], 0)
        device = event["receipt"]["device_receipt"]
        self.assertIs(type(device["pose_error"]["rotation_rad"]), float)
        self.assertIs(type(device["before"]["arm_status"]["ctrl_mode"]), int)
        self.assertEqual(device["candidate_measurement"], original["candidate_measurement"])
        self.assertEqual(device["hardware_commands_sent"], 1)
        self.assertIsInstance(original["before"]["arm_status"]["ctrl_mode"], Mode)
        self.assertIsNone(result["failure_receipt_persistence_error"])
        self.assertIsNone(self.host.status()["failure_receipt_persistence_error"])

    def test_nonfinite_failure_remains_pending_with_visible_normalization_error(self):
        event_id, result, _ = self.preparation_failure(np.float64("nan"))
        event = self.host.ledger.event(event_id)
        self.assertEqual(event["status"], "pending")
        self.assertIsNone(event["receipt"])
        self.assertTrue(math.isnan(result["receipt"]["device_receipt"]["pose_error"]["rotation_rad"]))
        error = result["failure_receipt_persistence_error"]
        self.assertEqual((error["event_id"], error["stage"], error["type"]),
                         (event_id, "normalize", "ValueError"))
        self.assertEqual(self.host.status()["failure_receipt_persistence_error"], error)

    def test_failed_durable_write_is_visible_and_not_retried(self):
        event_id, result, _ = self.preparation_failure(.001, finish_error=OSError("Synthetic storage failure"))
        self.assertEqual(self.host.ledger.event(event_id)["status"], "pending")
        error = result["failure_receipt_persistence_error"]
        self.assertEqual((error["event_id"], error["stage"], error["type"]),
                         (event_id, "ledger_finish", "OSError"))
        self.assertIn("Synthetic storage failure", error["detail"])

    def test_dispatch_nonfinite_failure_also_exposes_error_without_resend(self):
        device = self.fixture.devices[0]
        device.receipt_changes = {"ok": False, "extra": float("inf")}
        arguments = self.fixture.arguments(self.fixture.observation())
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"])
        self.assertEqual(result["status"], "fault")
        self.assertEqual(result["failure_receipt_persistence_error"]["stage"], "normalize")
        self.assertEqual(self.host.ledger.event(arguments["event_id"])["status"], "pending")
        self.assertEqual(len(device.calls), 1)
        self.assertEqual(device.frame_attempts, 1)
        self.assertTrue(math.isinf(result["receipt"]["device_receipt"]["extra"]))


if __name__ == "__main__":
    unittest.main()
