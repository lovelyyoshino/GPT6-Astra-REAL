"""Unopened-round repair through the real ledger, host and tool service.

Only archived copies in a temporary directory are changed. Socket, SDK and
camera entrypoints are blocked by the parent fixture; the device alone is an
in-memory double. These tests establish software routing, not real CAN health,
motion, readiness, contact, or physical stopping.
"""
from contextlib import contextmanager
import copy
import hashlib
import json
import sqlite3
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_tools import pair_ledger, pair_round
from robot_tools.pair_host import PairHost
from robot_tools.service import ToolService
import test_pair_initial_rx_round as initial_rx
from test_pair_host import FakePairDevice


RUN_ID = "repaired-new-round"
REVISION_TABLE = "pair_unopened_round_revisions"


class UnopenedRoundTests(unittest.TestCase):
    def setUp(self):
        self.f = initial_rx.InitialRXRoundTests("runTest")
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.root, self.path = self.f.root, self.f.path
        originals = {}
        # Freeze an earlier administrative implementation in the copied source
        # tree, then return to the production version under test. No real source
        # file, authorization, budget or prior receipt is rewritten.
        for name in ("pair_round.py", "pair_ledger.py", "pair_host.py"):
            path = self.root / "robot_tools" / name
            originals[name] = path.read_bytes()
            path.write_bytes(originals[name] + b"\n# synthetic previous enrollment implementation\n")
        self.proposal = self.f.prepare()
        self.activation = self.f.activate(self.proposal)
        for name, data in originals.items():
            (self.root / "robot_tools" / name).write_bytes(data)
        self.f.now += 1
        self.before = self.f.rows()
        self.old_run = next(row for row in self.before["pair_runs"] if row["run_id"] == RUN_ID)
        self.expected = hashlib.sha256(self.old_run["contract_json"].encode()).hexdigest()
        self.devices = []
        self.clock = SimpleNamespace(time=lambda: self.f.now,
                                     sleep=lambda dt: setattr(self.f, "now", self.f.now + dt))

    def revise(self, **changes):
        arguments = dict(expected_contract_sha256=self.expected, project_root=self.root,
                         reason="Offline test of the diagnosed unopened registration repair",
                         clock=lambda: self.f.now)
        arguments.update(changes)
        return pair_round.revise_unopened_round(self.path, RUN_ID, **arguments)

    def factory(self, *args):
        class PreparationDevice(FakePairDevice):
            def connect_for_preparation(inner):
                return {**inner.open(), "task_ready": False,
                        "readiness": {"synthetic_fixture": True}}
        device = PreparationDevice(*args, self.clock)
        self.devices.append(device)
        return device

    def make_host(self, contract, **changes):
        profile = json.loads((self.root / "configs/robot.json").read_text())
        arguments = dict(device_factory=self.factory, clock=self.clock.time,
                         background=False, connection_mode="prepare")
        arguments.update(changes)
        host = PairHost(self.root / "runs", profile, RUN_ID, contract["task"],
                        1000, 10800, **arguments)
        self.addCleanup(host.close)
        return host

    def assert_no_devices(self):
        self.assertEqual(self.devices, [])

    def assert_original_tables_unchanged(self):
        after = self.f.rows()
        for table, rows in self.before.items():
            self.assertEqual(rows, after[table], table)
        self.assertEqual(set(after) - set(self.before), {REVISION_TABLE})
        self.assertEqual(len(after[REVISION_TABLE]), 1)

    @contextmanager
    def changed_database(self, statement, arguments=()):
        """Restore this disposable fixture after one committed mutation."""
        memory = sqlite3.connect(":memory:")
        with sqlite3.connect(self.path) as db:
            db.backup(memory)
            db.execute(statement, arguments)
        try:
            yield
        finally:
            with sqlite3.connect(self.path) as db:
                memory.backup(db)
            memory.close()

    def budget_is_activated(self, **changes):
        arguments = dict(max_steps=1000, max_duration_s=10800)
        arguments.update(changes)
        return pair_ledger.activated_execution_budget(self.path, RUN_ID, **arguments)

    def test_real_host_open_preserves_history_budget_and_requires_fresh_preparation(self):
        with self.assertRaises((ValueError, pair_ledger.PairLedgerError)):
            self.make_host(self.activation["new_contract"])
        self.assert_no_devices()
        result = self.revise()
        self.assert_original_tables_unchanged()
        self.assertEqual(result["old_run"], self.old_run)
        self.assertEqual(result["old_contract"], json.loads(self.old_run["contract_json"]))
        self.assertFalse(result["new_budget_allocated"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNone(result["physical_stop_verified"])
        self.assertTrue(self.budget_is_activated())
        host = self.make_host(result["new_contract"])
        self.assertEqual(host.ledger.status()["contract"], result["new_contract"])
        opened = host.open()
        self.assertEqual(opened["status"], "owned")
        self.assertFalse(opened["task_ready"])
        self.assertEqual(host.deadline, self.old_run["started_at"] + self.old_run["max_duration"])
        self.assertEqual(host.ledger.status()["steps"], 0)
        self.assertEqual(self.devices[0].calls, [])
        self.assertEqual(self.devices[0].frame_attempts, 0)
        self.assertEqual(host.ledger.status()["execution_lineage"]["cumulative_steps"],
                         self.proposal["snapshot"]["cumulative_prior_steps"])
        host.close()
        self.assertEqual(self.before["pair_faults"], self.f.rows()["pair_faults"])
        self.assertEqual(self.before["pair_events"], self.f.rows()["pair_events"])
        with self.assertRaises(pair_ledger.PairLedgerError):
            self.revise()

    def test_tool_service_routes_to_real_host_without_patching_admission(self):
        result = self.revise()
        service = ToolService(self.root)
        service.persistent = True
        context = result["new_contract"]["task"]["site_context"]
        arguments = dict(run_id=RUN_ID, task_id=result["new_contract"]["task"]["task_id"],
                         workspace_clearance_statement=context["workspace_clearance"]["statement"],
                         max_steps=1000, max_duration_s=10800, connection_mode="prepare")
        if context.get("feedback_observation") is not None:
            policy = context["feedback_observation"]
            arguments.update(feedback_observation_profile=policy["profile"],
                             feedback_observation_statement=policy["statement"])
        outer = self

        class HostWithMemoryDevice(PairHost):
            def __init__(inner, *args, **kwargs):
                super().__init__(*args, **kwargs, device_factory=outer.factory,
                                 clock=outer.clock.time, background=False)

        # Dependency injection changes only the device/clock/thread policy.
        # The service, actual host constructor, budget and contract validators
        # all run normally, unlike a FakePairHost-only schema routing test.
        with patch("robot_tools.pair_host.PairHost", HostWithMemoryDevice):
            try:
                opened = service.call("robot_pair_open", arguments)
                self.assertIsInstance(service.pair_host, PairHost)
                self.assertEqual(opened["status"], "owned")
                self.assertFalse(opened["task_ready"])
                self.assertEqual(service.pair_host.ledger.status()["contract"], result["new_contract"])
                status = service.call("robot_pair_status", {})
                self.assertEqual(status["owner"], opened["owner"])
                self.assertEqual(self.devices[0].calls, [])
                self.assertEqual(self.devices[0].frame_attempts, 0)
            finally:
                if service.pair_host is not None:
                    service.call("robot_pair_close", {})
        self.assertEqual(self.before["pair_faults"], self.f.rows()["pair_faults"])

    def test_reopened_preparation_host_keeps_budget_and_reaudits_parent_history(self):
        result = self.revise()
        first = self.make_host(result["new_contract"])
        first_open = first.open()
        first_budget = first.ledger.status()
        self.assertEqual(first_open["status"], "owned")
        self.assertFalse(first_open["task_ready"])
        first.close()
        self.f.now += 3
        # The real budget recognizer replays the parent's historical audit
        # after the child has acquired and released ownership. No audit or
        # source/contract validation is mocked in either opening.
        self.assertTrue(self.budget_is_activated())
        second = self.make_host(result["new_contract"])
        second_open = second.open()
        second_budget = second.ledger.status()
        self.assertEqual(second_open["status"], "owned")
        self.assertFalse(second_open["task_ready"])
        self.assertNotEqual(first.owner, second.owner)
        for key in ("started_at", "deadline_s", "max_steps", "max_duration_s", "steps"):
            self.assertEqual(first_budget[key], second_budget[key], key)
        self.assertLess(second_budget["remaining_s"], first_budget["remaining_s"])
        second.close()
        after = self.f.rows()
        scope = next(row for row in after["pair_rounds"] if row["run_id"] == RUN_ID)
        self.assertGreater(scope["last_time"], json.loads(scope["record_json"])["activated_at"])
        for table in ("pair_runs", "pair_events", "pair_faults"):
            self.assertEqual(self.before[table], after[table], table)
        for device in self.devices:
            self.assertEqual(device.calls, [])
            self.assertEqual(device.frame_attempts, 0)

    def test_previously_claimed_and_released_empty_round_cannot_be_revised(self):
        ledger = pair_ledger.PairLedger(self.path, RUN_ID, self.activation["new_contract"],
                                        max_steps=1000, max_duration_s=10800, clock=self.clock.time)
        claimed = ledger.claim("previous-offline-owner")
        self.assertEqual(claimed["status"], "owned")
        self.f.now += .1
        released = ledger.release("previous-offline-owner")
        self.assertEqual(released["status"], "detached")
        self.assertEqual(released["steps"], 0)
        before = self.f.rows()
        scope = next(row for row in before["pair_rounds"] if row["run_id"] == RUN_ID)
        self.assertIsNone(scope["owner"])
        self.assertIsNone(scope["active_run_id"])
        self.assertIsNone(scope["fault_id"])
        self.assertGreater(scope["last_time"], json.loads(scope["record_json"])["activated_at"])
        self.assertFalse(any(row["run_id"] == RUN_ID for row in before["pair_events"]))
        with self.assertRaises(pair_ledger.PairLedgerError):
            self.revise()
        self.assertEqual(before, self.f.rows())
        self.assertNotIn(REVISION_TABLE, before)
        self.assert_no_devices()

    def test_ready_connection_cannot_skip_post_revision_preparation(self):
        result = self.revise()
        with self.assertRaisesRegex((ValueError, pair_ledger.PairLedgerError), "prepar"):
            self.make_host(result["new_contract"], connection_mode="ready")
        self.assert_no_devices()

    def test_owner_active_scope_fault_or_steps_blocks_revision(self):
        statements = (
            "UPDATE pair_rounds SET owner='active' WHERE run_id=?",
            "UPDATE pair_rounds SET active_run_id=run_id WHERE run_id=?",
            "UPDATE pair_rounds SET fault_id=21 WHERE run_id=?",
            "UPDATE pair_rounds SET last_time=last_time+0.001 WHERE run_id=?",
            "UPDATE pair_runs SET steps=1 WHERE run_id=?",
        )
        for statement in statements:
            with self.subTest(statement=statement), self.changed_database(statement, (RUN_ID,)):
                before = self.f.rows()
                with self.assertRaises(pair_ledger.PairLedgerError):
                    self.revise()
                self.assertEqual(before, self.f.rows())
        self.assert_no_devices()

    def test_any_current_event_blocks_revision_even_with_zero_step_counter(self):
        original = copy.deepcopy(self.before["pair_events"][0])
        for status in ("pending", "complete"):
            event = {**original, "run_id": RUN_ID, "event_id": "unexpected-event", "step": 1,
                     "status": status}
            statement = ("INSERT INTO pair_events (" + ",".join(event) + ") VALUES (" +
                         ",".join("?" for _ in event) + ")")
            with self.subTest(status=status), self.changed_database(statement, tuple(event.values())):
                before = self.f.rows()
                with self.assertRaises(pair_ledger.PairLedgerError):
                    self.revise()
                self.assertEqual(before, self.f.rows())

    def test_expired_window_or_clock_rollback_cannot_reset_time(self):
        scope = next(row for row in self.before["pair_rounds"] if row["run_id"] == RUN_ID)
        for now in (self.old_run["started_at"] + self.old_run["max_duration"], scope["last_time"] - .001):
            with self.subTest(now=now), self.assertRaises(pair_ledger.PairLedgerError):
                self.revise(clock=lambda: now)
            self.assertEqual(self.before, self.f.rows())

    def test_budget_or_original_authorization_change_blocks_revision(self):
        statements = (
            "UPDATE pair_runs SET max_steps=max_steps+1 WHERE run_id=?",
            "UPDATE pair_runs SET max_duration=max_duration+1 WHERE run_id=?",
            "UPDATE pair_runs SET started_at=started_at+1 WHERE run_id=?",
        )
        for statement in statements:
            with self.subTest(statement=statement), self.changed_database(statement, (RUN_ID,)):
                with self.assertRaises(pair_ledger.PairLedgerError):
                    self.revise()
        scope = next(row for row in self.before["pair_rounds"] if row["run_id"] == RUN_ID)
        record = json.loads(scope["record_json"])
        record["authorization"]["statement"] = "Changed historical authorization"
        with self.changed_database("UPDATE pair_rounds SET record_json=? WHERE run_id=?",
                                   (json.dumps(record), RUN_ID)):
            with self.assertRaises(pair_ledger.PairLedgerError):
                self.revise()

    def test_live_process_wrong_digest_or_empty_reason_blocks_revision(self):
        with patch.object(pair_round, "_live_control_processes", return_value=["active"]):
            with self.assertRaises(pair_ledger.PairLedgerError):
                self.revise()
        for changes in ({"expected_contract_sha256": "0" * 64}, {"reason": " "}):
            with self.subTest(changes=changes), self.assertRaises(pair_ledger.PairLedgerError):
                self.revise(**changes)
        self.assertEqual(self.before, self.f.rows())

    def test_unrelated_control_code_or_device_binding_change_blocks_revision(self):
        source = self.root / "robot_tools/joint_path.py"
        original = source.read_bytes()
        source.write_bytes(original + b"\n# unrelated runtime change\n")
        with self.assertRaises(pair_ledger.PairLedgerError):
            self.revise()
        source.write_bytes(original)
        profile_path = self.root / "configs/robot.json"
        profile = json.loads(profile_path.read_text())
        profile["cameras"]["front"] = "different-camera"
        profile_path.write_text(json.dumps(profile))
        with self.assertRaises(pair_ledger.PairLedgerError):
            self.revise()
        self.assertEqual(self.before, self.f.rows())

    def test_historical_parent_fault_tampering_blocks_revision(self):
        with self.changed_database("UPDATE pair_faults SET reason='rewritten' WHERE id=21"):
            with self.assertRaises(pair_ledger.PairLedgerError):
                self.revise()

    def test_revision_is_single_use_and_does_not_grant_a_different_budget(self):
        self.revise()
        after = self.f.rows()
        with self.assertRaises(pair_ledger.PairLedgerError):
            self.revise()
        self.assertEqual(after, self.f.rows())
        self.assertFalse(self.budget_is_activated(max_steps=999))
        self.assertFalse(self.budget_is_activated(max_duration_s=10801))

    def test_tampered_revision_refuses_before_device_construction(self):
        result = self.revise()
        raw = self.f.rows()[REVISION_TABLE][0]["record_json"]
        mutations = (
            lambda record: record.update(events_at_revision=[{"sent": 1}]),
            lambda record: record["old_run"].update(max_steps=999),
            lambda record: record["new_contract"]["task"].update(task_id="other-task"),
            lambda record: record.update(new_budget_allocated=True),
            lambda record: record.update(hardware_commands_sent=1),
            lambda record: record["historical_audit"].update(fresh_host_admission_required=False),
        )
        for mutate in mutations:
            record = json.loads(raw)
            mutate(record)
            with self.subTest(mutate=mutate), self.changed_database(
                    "UPDATE " + REVISION_TABLE + " SET record_json=?", (json.dumps(record),)):
                with self.assertRaises(pair_ledger.PairLedgerError):
                    self.budget_is_activated()
                with self.assertRaises((ValueError, pair_ledger.PairLedgerError)):
                    self.make_host(result["new_contract"])
                self.assert_no_devices()

    def test_source_change_after_revision_refuses_before_device_construction(self):
        result = self.revise()
        source = self.root / "robot_tools/joint_path.py"
        source.write_bytes(source.read_bytes() + b"\n# later unreviewed control change\n")
        self.assertTrue(self.budget_is_activated())
        # Redirect only the host's source root to the disposable copied tree;
        # its real hash computation and frozen-contract checks remain intact.
        with patch("robot_tools.pair_host.__file__", str(self.root / "robot_tools/pair_host.py")):
            with self.assertRaisesRegex(ValueError, "contract"):
                self.make_host(result["new_contract"])
        self.assert_no_devices()


if __name__ == "__main__":
    unittest.main()
