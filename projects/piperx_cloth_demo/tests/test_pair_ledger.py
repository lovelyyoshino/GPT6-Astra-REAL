"""Offline SQLite transactions, process exclusion and immutable-budget tests."""
import copy
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from robot_tools.pair_ledger import PairLedger, PairLedgerFault, platform_fault, platform_state


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class PairLedgerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "pair.sqlite"
        self.clock = Clock()
        self.contract = {"task": "offline-test", "arms": ["left", "right"], "limit": 0.003}
        self.ledger = self.open()

    def open(self, run_id="run-1", **overrides):
        return PairLedger(self.path, run_id, self.contract, clock=self.clock, **overrides)

    def complete(self, event="event-1", payload=None, receipt=None):
        payload = {"arm": "left"} if payload is None else payload
        receipt = {"ok": True, "frames": 4} if receipt is None else receipt
        begun = self.ledger.begin("owner-1", event, payload)
        self.assertFalse(begun["replayed"])
        self.assertEqual(self.ledger.finish("owner-1", event, receipt), receipt)
        return receipt

    def test_completed_replay_after_reopen_and_clean_detach_does_not_spend_budget(self):
        self.ledger.claim("owner-1")
        original = self.complete(payload={"b": 2, "a": 1})
        self.clock.now += 7
        detached = self.ledger.release("owner-1")
        self.assertEqual(detached["status"], "detached")
        self.assertIsNone(detached["physical_stop_verified"])
        reopened = self.open()
        state = reopened.claim("owner-2")
        self.assertEqual(state["steps"], 1)
        self.assertEqual(state["elapsed_s"], 7)
        replay = reopened.begin("owner-2", "event-1", {"a": 1, "b": 2})
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], original)
        self.assertEqual(reopened.status()["steps"], 1)

    def test_contract_and_budget_cannot_be_changed_by_reinitialization(self):
        changed = copy.deepcopy(self.contract)
        changed["limit"] = 1.0
        for kwargs in ({"contract": changed}, {"max_steps": 129}, {"max_duration_s": 901}):
            with self.subTest(kwargs=kwargs):
                arguments = dict(contract=self.contract, clock=self.clock)
                arguments.update(kwargs)
                with self.assertRaises(ValueError):
                    PairLedger(self.path, "run-1", **arguments)
        self.assertEqual(self.ledger.status()["contract"], self.contract)
        self.assertEqual(self.ledger.status()["max_steps"], 128)

    def test_pending_reopen_claim_latches_fault_across_runs(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {"command": "one"})
        reopened = self.open()
        with self.assertRaises(PairLedgerFault):
            reopened.claim("owner-2")
        state = reopened.status()
        self.assertEqual(state["pending_event_id"], "event-1")
        self.assertTrue(state["fault_latched"])
        alternative = self.open(run_id="new-run")
        with self.assertRaises(PairLedgerFault):
            alternative.claim("new-owner")
        self.assertEqual(alternative.status()["global_fault"], state["global_fault"])
        # The original owner can still record a late receipt without clearing fault.
        self.ledger.finish("owner-1", "event-1", {"ok": True})
        self.assertTrue(self.ledger.status()["fault_latched"])

    def test_repeated_pending_event_never_returns_dispatch_permission(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {"x": 1})
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {"x": 1})
        self.assertEqual(self.ledger.status()["steps"], 1)
        self.assertIn("pending", self.ledger.status()["fault"]["reason"])

    def test_same_event_different_payload_latches_without_second_attempt(self):
        self.ledger.claim("owner-1")
        self.complete(payload={"x": 1})
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {"x": 2})
        self.assertEqual(self.ledger.status()["steps"], 1)
        self.assertEqual(self.ledger.status()["fault"]["reason"], "event_payload_conflict")

    def test_claim_is_not_reentrant_even_for_same_owner(self):
        self.ledger.claim("owner-1")
        with self.assertRaises(PairLedgerFault):
            self.ledger.claim("owner-1")
        self.assertTrue(self.ledger.status()["fault_latched"])

    def test_existing_owner_blocks_claim_from_another_process(self):
        self.ledger.claim("owner-1")
        code = """import json,sys
from robot_tools.pair_ledger import PairLedger,PairLedgerFault
ledger=PairLedger(sys.argv[1],'run-1',json.loads(sys.argv[2]),clock=lambda:100.0)
try:
    ledger.claim('child-owner')
except PairLedgerFault:
    print(json.dumps(ledger.status()))
else:
    raise SystemExit('Second process unexpectedly acquired owner')
"""
        result = subprocess.run([sys.executable, "-B", "-c", code, str(self.path), json.dumps(self.contract)],
                                check=True, capture_output=True, text=True,
                                cwd=Path(__file__).resolve().parents[1])
        child = json.loads(result.stdout)
        self.assertTrue(child["fault_latched"])
        self.assertEqual(child["owner"], "owner-1")
        self.assertEqual(self.ledger.status()["global_fault"], child["global_fault"])

    def test_concurrent_begins_reserve_only_one_attempt_and_latch_conflict(self):
        other = self.open()
        self.ledger.claim("owner-1")
        barrier, results = threading.Barrier(2), []
        def begin(ledger, event):
            barrier.wait(timeout=5)
            try:
                results.append(ledger.begin("owner-1", event, {"event": event}))
            except PairLedgerFault as error:
                results.append(error)
        threads = [threading.Thread(target=begin, args=(ledger, event))
                   for ledger, event in ((self.ledger, "event-a"), (other, "event-b"))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=10)
            self.assertFalse(thread.is_alive())
        self.assertEqual(sum(isinstance(result, dict) for result in results), 1)
        self.assertEqual(sum(isinstance(result, PairLedgerFault) for result in results), 1)
        self.assertEqual(self.ledger.status()["steps"], 1)
        self.assertTrue(self.ledger.status()["fault_latched"])

    def test_step_budget_survives_detach_reopen_and_replayed_receipts(self):
        self.path = Path(self.directory.name) / "small.sqlite"
        self.ledger = self.open(max_steps=2)
        self.ledger.claim("owner-1")
        self.complete("event-1")
        self.complete("event-2")
        self.ledger.release("owner-1")
        resumed = self.open(max_steps=2)
        resumed.claim("owner-2")
        self.assertTrue(resumed.begin("owner-2", "event-2", {"arm": "left"})["replayed"])
        with self.assertRaises(PairLedgerFault):
            resumed.begin("owner-2", "event-3", {"arm": "left"})
        self.assertEqual(resumed.status()["steps"], 2)
        self.assertEqual(resumed.status()["remaining_steps"], 0)

    def test_elapsed_budget_continues_while_detached(self):
        self.path = Path(self.directory.name) / "short.sqlite"
        self.ledger = self.open(max_duration_s=10)
        self.ledger.claim("owner-1")
        self.clock.now += 4
        self.ledger.release("owner-1")
        self.clock.now += 6
        reopened = self.open(max_duration_s=10)
        self.assertEqual(reopened.status()["fault"]["reason"], "duration_budget_exhausted")
        with self.assertRaises(PairLedgerFault):
            reopened.claim("owner-2")

    def test_deadline_is_frozen_across_detach_new_owner_and_status_calls(self):
        before = self.ledger.status()
        self.assertEqual(before["started_at"], 100.0)
        self.assertEqual(before["deadline_s"], 1000.0)
        self.ledger.claim("owner-1")
        self.clock.now += 12.5
        self.ledger.release("owner-1")
        self.clock.now += 5
        resumed = self.open()
        after = resumed.claim("owner-2")
        self.assertEqual(after["started_at"], before["started_at"])
        self.assertEqual(after["deadline_s"], before["deadline_s"])
        self.assertEqual(after["remaining_s"], before["deadline_s"] - self.clock.now)

    def test_clock_rollback_is_durable_and_cannot_be_reset_with_new_run(self):
        self.ledger.claim("owner-1")
        self.clock.now += 1
        self.ledger.status()
        self.clock.now -= 2
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {})
        self.clock.now += 10
        self.assertEqual(self.open("new-run").status()["fault"]["reason"], "clock_rollback")

    def test_invalid_clock_after_initialization_latches(self):
        self.ledger.claim("owner-1")
        self.clock.now = float("nan")
        state = self.ledger.status()
        self.assertTrue(state["fault_latched"])
        self.assertIn("clock_invalid", state["fault"]["reason"])

    def test_failed_receipt_completes_record_and_permanently_latches(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {})
        receipt = {"ok": False, "physical_stop_verified": None}
        self.assertEqual(self.ledger.finish("owner-1", "event-1", receipt, success=False), receipt)
        state = self.ledger.status()
        self.assertIsNone(state["pending_event_id"])
        self.assertEqual(state["fault"]["reason"], "execution_receipt_failed")
        with self.assertRaises(PairLedgerFault):
            self.ledger.release("owner-1")
        with self.assertRaises(PairLedgerFault):
            self.open("other-run").claim("owner-2")

    def test_finish_is_idempotent_but_different_receipt_latches(self):
        self.ledger.claim("owner-1")
        receipt = self.complete()
        self.assertEqual(self.ledger.finish("owner-1", "event-1", receipt), receipt)
        with self.assertRaises(PairLedgerFault):
            self.ledger.finish("owner-1", "event-1", {"ok": True, "frames": 3})
        self.assertEqual(self.ledger.status()["fault"]["reason"], "event_receipt_conflict")

    def test_pending_release_and_wrong_owner_cannot_detach(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {})
        with self.assertRaises(PairLedgerFault):
            self.ledger.release("owner-1")
        self.assertEqual(self.ledger.status()["owner"], "owner-1")
        with self.assertRaises(PairLedgerFault):
            self.ledger.finish("owner-2", "event-1", {})
        self.assertEqual(self.ledger.status()["pending_event_id"], "event-1")

    def test_first_fault_reason_is_preserved(self):
        self.ledger.claim("owner-1")
        first = self.ledger.fault("owner-1", "feedback lost")["fault"]
        self.ledger.fault("owner-1", "later shutdown uncertainty")
        self.assertEqual(self.open().status()["fault"], first)
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {})

    def test_platform_fault_read_is_noncreating_and_does_not_renew_budget(self):
        absent = Path(self.directory.name) / "absent.sqlite"
        self.assertIsNone(platform_fault(absent))
        self.assertFalse(absent.exists())
        self.assertIsNone(platform_fault(self.path))
        self.ledger.claim("owner-1")
        fault = self.ledger.fault("owner-1", "uncertain send")["fault"]
        content, modified = self.path.read_bytes(), self.path.stat().st_mtime_ns
        self.clock.now += 100
        self.assertEqual(platform_fault(self.path), fault)
        self.assertEqual(self.path.read_bytes(), content)
        self.assertEqual(self.path.stat().st_mtime_ns, modified)

    def test_platform_state_exposes_crash_owner_and_pending_without_latching(self):
        absent = Path(self.directory.name) / "absent.sqlite"
        self.assertIsNone(platform_state(absent))
        self.assertFalse(absent.exists())
        self.assertEqual(platform_state(self.path), {"owner": None, "active_run_id": None,
                                                     "pending_events": [], "fault": None})
        self.ledger.claim("owner-1")
        claimed = platform_state(self.path)
        self.assertEqual(claimed["owner"], "owner-1")
        self.assertEqual(claimed["pending_events"], [])
        self.ledger.begin("owner-1", "event-1", {"command": "one"})
        content, modified = self.path.read_bytes(), self.path.stat().st_mtime_ns
        self.clock.now += 10000
        state = platform_state(self.path)
        self.assertEqual(state["owner"], "owner-1")
        self.assertEqual(state["active_run_id"], "run-1")
        self.assertEqual(state["pending_events"], [{"run_id": "run-1", "event_id": "event-1",
                                                  "owner": "owner-1", "step": 1, "began_at": 100.0}])
        self.assertIsNone(state["fault"])
        self.assertIsNone(platform_fault(self.path))
        self.assertEqual(self.path.read_bytes(), content)
        self.assertEqual(self.path.stat().st_mtime_ns, modified)

    def test_platform_state_exposes_fault_and_clean_detach(self):
        self.ledger.claim("owner-1")
        self.complete()
        self.ledger.release("owner-1")
        self.assertEqual(platform_state(self.path), {"owner": None, "active_run_id": None,
                                                     "pending_events": [], "fault": None})
        self.ledger.claim("owner-2")
        fault = self.ledger.fault("owner-2", "new fault")["fault"]
        self.assertEqual(platform_state(self.path)["fault"], fault)

    def test_existing_bad_platform_database_fails_closed(self):
        corrupt = Path(self.directory.name) / "bad.sqlite"
        corrupt.write_bytes(b"not a SQLite database")
        with self.assertRaises(sqlite3.DatabaseError):
            platform_fault(corrupt)
        with self.assertRaises(sqlite3.DatabaseError):
            platform_state(corrupt)
        empty = Path(self.directory.name) / "empty.sqlite"
        empty.touch()
        with self.assertRaises(sqlite3.DatabaseError):
            platform_fault(empty)

    def test_event_read_does_not_claim_touch_clock_or_modify_frozen_state(self):
        self.ledger.claim("owner-1")
        receipt = self.complete(payload={"command": "move", "nested": {"value": 1}})
        self.ledger.release("owner-1")
        content, modified = self.path.read_bytes(), self.path.stat().st_mtime_ns
        self.clock.now = 10000.0
        with patch.object(self.ledger, "clock", side_effect=AssertionError("Read must not consult clock")):
            self.assertIsNone(self.ledger.event("missing"))
            event = self.ledger.event("event-1")
        self.assertEqual(event, {"event_id": "event-1", "status": "complete",
                                 "payload": {"command": "move", "nested": {"value": 1}},
                                 "receipt": receipt, "success": True, "owner": "owner-1", "step": 1})
        event["receipt"]["frames"] = 999
        event["payload"]["nested"]["value"] = 2
        self.assertEqual(self.ledger.event("event-1")["receipt"], receipt)
        self.assertEqual(self.path.read_bytes(), content)
        self.assertEqual(self.path.stat().st_mtime_ns, modified)
        self.assertIsNone(platform_fault(self.path))

    def test_read_pending_event_does_not_authorize_retry(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {"command": "one"})
        event = self.ledger.event("event-1")
        self.assertEqual(event["status"], "pending")
        self.assertIsNone(event["receipt"])
        self.assertIsNone(event["success"])
        self.assertEqual(event["owner"], "owner-1")
        self.assertIsNone(platform_fault(self.path))
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {"command": "one"})

    def test_event_query_is_scoped_to_run_and_failed_receipt_remains_readable(self):
        other = self.open("other-run")
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {})
        self.ledger.finish("owner-1", "event-1", {"ok": False}, success=False)
        event = self.ledger.event("event-1")
        self.assertFalse(event["success"])
        self.assertEqual(event["receipt"], {"ok": False})
        self.assertIsNone(other.event("event-1"))

    def test_strict_types_and_nonfinite_json_are_rejected_before_reservation(self):
        self.ledger.claim("owner-1")
        for event in ("", " leading", "newline\n", "x" * 129, True, 1):
            with self.subTest(event=event), self.assertRaises(ValueError):
                self.ledger.begin("owner-1", event, {})
        for payload in ({"x": float("nan")}, {"x": float("inf")}, {1: "bad-key"},
                        {"x": (1, 2)}, {"x": 2**64}, []):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.ledger.begin("owner-1", "event-1", payload)
        self.assertEqual(self.ledger.status()["steps"], 0)
        self.assertFalse(self.ledger.status()["fault_latched"])

    def test_invalid_constructor_budgets_do_not_create_database(self):
        for options in ({"max_steps": True}, {"max_steps": 0}, {"max_steps": 1.0},
                        {"max_duration_s": False}, {"max_duration_s": float("nan")},
                        {"max_duration_s": -1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                PairLedger(Path(self.directory.name) / "invalid.sqlite", "run", {}, **options)
        self.assertFalse((Path(self.directory.name) / "invalid.sqlite").exists())

    def test_commit_error_never_returns_dispatch_permission_and_rolls_back_step(self):
        self.ledger.claim("owner-1")
        connect = self.ledger._connect
        class FailCommit:
            def __init__(self):
                self.db = connect()
            def execute(self, sql, *args):
                if sql == "COMMIT":
                    raise sqlite3.OperationalError("injected journal commit failure")
                return self.db.execute(sql, *args)
            def __getattr__(self, name):
                return getattr(self.db, name)
        dispatched = False
        with patch.object(self.ledger, "_connect", side_effect=FailCommit):
            with self.assertRaises(sqlite3.OperationalError):
                self.ledger.begin("owner-1", "event-1", {})
                dispatched = True
        self.assertFalse(dispatched)
        self.assertEqual(self.ledger.status()["steps"], 0)
        self.assertIsNone(self.ledger.status()["pending_event_id"])

    def test_failed_receipt_storage_leaves_pending_and_forbids_replay(self):
        self.ledger.claim("owner-1")
        self.ledger.begin("owner-1", "event-1", {})
        with patch.object(self.ledger, "_connect", side_effect=sqlite3.OperationalError("disk full")):
            with self.assertRaises(sqlite3.OperationalError):
                self.ledger.finish("owner-1", "event-1", {"ok": True})
        self.assertEqual(self.ledger.status()["pending_event_id"], "event-1")
        with self.assertRaises(PairLedgerFault):
            self.ledger.begin("owner-1", "event-1", {})

    def test_connections_use_full_synchronous_and_immediate_transactions(self):
        statements = []
        connect = self.ledger._connect
        def traced():
            db = connect()
            self.assertEqual(db.execute("PRAGMA synchronous").fetchone()[0], 2)
            db.set_trace_callback(statements.append)
            return db
        with patch.object(self.ledger, "_connect", side_effect=traced):
            self.ledger.claim("owner-1")
            self.complete()
        self.assertEqual(sum(sql == "BEGIN IMMEDIATE" for sql in statements), 3)
        self.assertEqual(sum(sql == "COMMIT" for sql in statements), 3)


if __name__ == "__main__":
    unittest.main()
