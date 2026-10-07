"""Offline code repair of query-only SQLite history; no hardware or SDK."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_tools.execution import ExclusiveExecution, ExecutionFault
from robot_tools.pair_ledger import PairLedger, PairLedgerError, revise_query_only_code_contract
import test_joint_sources as fixtures


class QueryOnlyCodeRevisionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        sockets = patch("socket.socket", side_effect=AssertionError("Hardware/network forbidden"))
        sockets.start()
        self.addCleanup(sockets.stop)
        self.root = Path(self.temp.name)
        self.project = self.root / "project"
        self.sources = self.project / "robot_tools"
        self.sources.mkdir(parents=True)
        self.path = self.root / "pair_sessions.sqlite"
        self.now = 80.
        self.bindings = {s: {"connection_id": "old-"+s, "model": "piper_x", "firmware_profile": "default",
                            "channel": "can"+str(i), "usb_interface": "port-"+s}
                         for i, s in enumerate(("left", "right"))}
        self.contract = {"task": {"task_id": "plug", "roles": {"left": "task", "right": "task"}},
            "arms": {s: {"model": "piper_x", "firmware": "default", "channel": b["channel"],
                         "usb_interface": b["usb_interface"]} for s, b in self.bindings.items()},
            "cameras": {"front": "a", "left_wrist": "b", "right_wrist": "c"},
            "sdk_commit_audited": "a"*40,
            "code": {"joint_path.py": hashlib.sha256(b"old source").hexdigest(),
                     "pair_ledger.py": hashlib.sha256(b"ledger").hexdigest()}}
        (self.sources / "joint_path.py").write_bytes(b"repaired source")
        (self.sources / "pair_ledger.py").write_bytes(b"ledger")
        self.ledger = PairLedger(self.path, "run", self.contract, max_steps=8, max_duration_s=900, clock=lambda:self.now)
        self.ledger.claim("owner")
        payload = {"kind":"query", "request":{"operation":"inspect_joint_limits"}, "bindings":self.bindings}
        self.ledger.begin("owner", "limits", payload)
        capture = fixtures.JointSourcesTests.make_capture(SimpleNamespace(bindings=self.bindings))
        capture.update(ok=True, fault_latched=False, event_id="limits", pair_owner="owner", execution_mode="inspect_joint_limits",
            hardware_commands_sent=12, joint_limit_queries_attempted=12,
            target_commands_sent=0, mode_commands_sent=0, enable_commands_sent=0, stop_commands_sent=0,
            transmission_counts={s:{"attempted_frames":6,"sent_frames":6,"blocked_frames":0} for s in self.bindings},
            session_transmission_counts={s:{"attempted_frames":6,"sent_frames":6,"blocked_frames":0} for s in self.bindings})
        self.now = 95.
        self.ledger.finish("owner", "limits", capture)
        self.ledger.release("owner")
        self.before = self.rows()
        self.digest = hashlib.sha256(self.before["run"]["contract_json"].encode()).hexdigest()

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            return {"run":dict(db.execute("SELECT * FROM pair_runs WHERE run_id='run'").fetchone()),
                    "scope":dict(db.execute("SELECT * FROM pair_scope").fetchone()),
                    "events":[dict(r) for r in db.execute("SELECT * FROM pair_events ORDER BY step")]}

    def revise(self, **kwargs):
        return revise_query_only_code_contract(self.path, "run", expected_contract_sha256=self.digest,
            project_root=self.project, reason="Repair source reference length; no control-limit changes", clock=lambda:self.now, **kwargs)

    def edit_receipt(self, change):
        with sqlite3.connect(self.path) as db:
            value = json.loads(db.execute("SELECT receipt_json FROM pair_events").fetchone()[0])
            change(value)
            db.execute("UPDATE pair_events SET receipt_json=?", (json.dumps(value),))

    def assert_refused(self):
        before = self.rows()
        with self.assertRaises((PairLedgerError, ValueError, OSError)):
            self.revise()
        self.assertEqual(self.rows(), before)

    def test_code_revision_preserves_exact_events_budget_and_full_old_contract(self):
        result = self.revise()
        after = self.rows()
        self.assertEqual(after["events"], self.before["events"])
        self.assertEqual({k:v for k,v in after["scope"].items() if k!="last_time"},
                         {k:v for k,v in self.before["scope"].items() if k!="last_time"})
        self.assertEqual(after["scope"]["last_time"],self.now)
        self.assertEqual({k:v for k,v in after["run"].items() if k!="contract_json"},
                         {k:v for k,v in self.before["run"].items() if k!="contract_json"})
        self.assertEqual(result["deadline_s"], 980.)
        self.assertEqual(result["budget"]["steps"], 1)
        new = json.loads(after["run"]["contract_json"])
        self.assertEqual({k:v for k,v in new.items() if k!="code"},
                         {k:v for k,v in self.contract.items() if k!="code"})
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            audit=dict(db.execute("SELECT * FROM pair_code_revisions").fetchone())
        self.assertEqual(audit["old_contract_json"],self.before["run"]["contract_json"])
        self.assertEqual(audit["new_contract_json"],after["run"]["contract_json"])
        self.assertEqual(json.loads(audit["source_hashes_json"]),new["code"])
        reopened=PairLedger(self.path,"run",new,max_steps=8,max_duration_s=900,clock=lambda:self.now)
        self.assertEqual(reopened.claim("new-owner")["steps"],1)
        original=self.before["events"][0]
        replay=reopened.begin("new-owner","limits",json.loads(original["payload_json"]))
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"],json.loads(original["receipt_json"]))
        self.assertEqual(reopened.status()["deadline_s"],980.)

    def test_old_constructor_and_stale_digest_cannot_replace_repaired_contract(self):
        self.revise()
        self.assert_refused()
        with self.assertRaises(ValueError):
            PairLedger(self.path,"run",self.contract,max_steps=8,max_duration_s=900,clock=lambda:self.now)

    def test_late_revision_retains_latest_time_and_rollback_latches(self):
        self.now=970.
        result=self.revise()
        self.assertEqual(self.rows()["scope"]["last_time"],970.)
        self.now=100.
        reopened=PairLedger(self.path,"run",result["contract"],max_steps=8,max_duration_s=900,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):
            reopened.claim("new-owner")
        status=reopened.peek_status()
        self.assertTrue(status["fault_latched"])
        self.assertEqual(status["fault"]["reason"],"clock_rollback")
        self.assertEqual(status["deadline_s"],980.)
        self.assertEqual(status["remaining_s"],10.)

    def test_clean_owner_and_shared_execution_lock_are_required(self):
        self.ledger.claim("active")
        self.assert_refused()
        self.ledger.release("active")
        with ExclusiveExecution(self.root), self.assertRaises(ExecutionFault):
            self.revise()

    def test_pending_or_fault_cannot_be_revised_or_cleared(self):
        self.ledger.claim("active")
        self.ledger.begin("active","pending",{"kind":"query"})
        self.assert_refused()
        self.ledger.fault("active","real fault")
        self.assert_refused()

    def test_original_deadline_and_clock_rollback_refuse_without_mutation(self):
        for now in (94.,980.,float("nan"),False):
            with self.subTest(now=now):
                self.now=now
                self.assert_refused()

    def test_wrong_frame_or_missing_raw_response_blocks_revision(self):
        before=self.before["events"][0]["receipt_json"]
        for change in (lambda r:r["query_receipts"]["left"]["1"].update(arbitration_id=0x151),
                       lambda r:r["query_receipts"]["right"]["6"].update(outcome="unknown"),
                       lambda r:r["joint_limits"]["left"]["1"]["response_evidence"].update(response_frames=[])):
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE pair_events SET receipt_json=?",(before,))
            self.edit_receipt(change)
            self.assert_refused()

    def test_nonzero_or_boolean_counters_and_extra_lifetime_frames_refuse(self):
        before=self.before["events"][0]["receipt_json"]
        for change in (lambda r:r.update(actuator_commands_sent=1),lambda r:r.update(mode_commands_sent=False),
                       lambda r:r["session_transmission_counts"]["left"].update(sent_frames=7),
                       lambda r:r.pop("stop_commands_sent")):
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE pair_events SET receipt_json=?",(before,))
            self.edit_receipt(change)
            self.assert_refused()

    def test_nonquery_payload_and_budget_mismatch_refuse(self):
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET payload_json=?",(json.dumps({"kind":"joint"}),))
        self.assert_refused()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET payload_json=?",(self.before["events"][0]["payload_json"],))
            db.execute("UPDATE pair_runs SET steps=2")
        self.assert_refused()

    def test_any_grasp_history_refuses_even_empty(self):
        with sqlite3.connect(self.path) as db:
            db.execute("CREATE TABLE pair_grasp_episodes(state_json TEXT)")
            db.execute("INSERT INTO pair_grasp_episodes VALUES(?)",('{"status":"empty"}',))
        self.assert_refused()

    def test_missing_symlink_and_unchanged_sources_refuse(self):
        source=self.sources/"joint_path.py"
        source.unlink()
        self.assert_refused()
        external=self.root/"external.py"
        external.write_bytes(b"repaired source")
        source.symlink_to(external)
        self.assert_refused()
        source.unlink()
        source.write_bytes(b"old source")
        self.assert_refused()

    def test_io_crossing_original_deadline_refuses_before_commit(self):
        times=iter((95.,979.,980.))
        before=self.rows()
        with self.assertRaises(PairLedgerError):
            revise_query_only_code_contract(self.path,"run",expected_contract_sha256=self.digest,
                project_root=self.project,reason="Source ref repair",clock=lambda:next(times))
        self.assertEqual(self.rows(),before)

    def test_sqlite_revision_writes_cannot_renew_expired_budget(self):
        times=iter((95.,96.,97.,980.))
        before=self.rows()
        with self.assertRaises(PairLedgerError):
            revise_query_only_code_contract(self.path,"run",expected_contract_sha256=self.digest,
                project_root=self.project,reason="Source ref repair",clock=lambda:next(times))
        self.assertEqual(self.rows(),before)
        with sqlite3.connect(self.path) as db:
            self.assertIsNone(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_code_revisions'").fetchone())


if __name__ == "__main__":
    unittest.main()
