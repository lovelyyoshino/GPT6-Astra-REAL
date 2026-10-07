"""Fault diagnosis remains readable without mutating the dispatch ledger."""
from pathlib import Path
import sqlite3
import tempfile
import unittest

from robot_tools.pair_ledger import PairLedger


class PairLedgerPeekTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/"pair.sqlite"
        self.now = 100.
        self.ledger = PairLedger(self.path, "run", {}, clock=lambda: self.now)
        self.ledger.claim("owner")

    def rows(self):
        with sqlite3.connect(self.path) as db:
            return {name: db.execute("SELECT * FROM "+name).fetchall()
                    for name in ("pair_scope", "pair_runs", "pair_events", "pair_faults")}

    def test_fault_clock_failures_do_not_append_faults_or_renew_ownership(self):
        self.ledger.fault("owner", "cancelled")
        before = self.rows()
        for stamp in (99., float("nan"), float("inf"), True, None):
            self.now = stamp
            with self.subTest(stamp=stamp):
                for _ in range(2):
                    result = self.ledger.peek_status()
                    self.assertTrue(result["fault_latched"])
                    self.assertFalse(result["time_valid"])
                    self.assertFalse(result["dispatch_authorized"])
                    self.assertEqual(result["owner"], "owner")
                self.assertEqual(self.rows(), before)

    def test_healthy_clock_peek_never_performs_admission_or_latches_expiration(self):
        before = self.rows()
        self.now = 2000.
        result = self.ledger.peek_status()
        self.assertEqual(result["remaining_s"], 0)
        self.assertTrue(result["time_valid"])
        self.assertFalse(result["dispatch_authorized"])
        self.assertEqual(self.rows(), before)
        self.assertEqual(self.ledger.status()["fault"]["reason"], "duration_budget_exhausted")

    def test_pending_attempt_is_preserved_and_not_made_replayable(self):
        self.ledger.begin("owner", "step", {"kind": "move"})
        before = self.rows()
        result = self.ledger.peek_status()
        self.assertEqual(result["pending_event_id"], "step")
        self.assertEqual(result["steps"], 1)
        self.assertIsNone(result["physical_stop_verified"])
        self.assertEqual(self.rows(), before)


if __name__ == "__main__":
    unittest.main()
