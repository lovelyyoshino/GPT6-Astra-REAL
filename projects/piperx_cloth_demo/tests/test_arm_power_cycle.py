"""Real service/startup executors with FakeCAN; real sockets are forbidden."""
from contextlib import nullcontext
import json
from pathlib import Path
import sqlite3
import tempfile
import sys
from unittest.mock import patch

from robot_tools import arm_power_cycle as cycle
from robot_tools.pair_ledger import PairLedger
from robot_tools.service import ToolService
from test_backend import PROFILE
import test_takeover as fixtures


class ArmPowerCycleTests(fixtures.TakeoverFixture):
    snapshot = fixtures.StartupTests.snapshot

    def setUp(self):
        super().setUp()
        self.robots = {s: fixtures.FakeStartupRobot(s) for s in ("left", "right")}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "configs").mkdir()
        (self.root / "configs/robot.json").write_text(json.dumps(PROFILE))
        (self.root / "runs").mkdir()
        self.path = self.root / "runs/pair_sessions.sqlite"
        self.ledger = PairLedger(self.path, "old", {"task": "old"}, clock=lambda: 100.)
        self.ledger.claim("old-owner")
        self.ledger.fault("old-owner", "old-fault")
        self.service = ToolService(self.root)
        self.boot = {"boot_id": "same-boot", "started_at": 50.}
        self.processes = self.stack.enter_context(patch.object(cycle, "check_processes"))
        self.stack.enter_context(patch.object(cycle, "boot_identity", side_effect=lambda: dict(self.boot)))
        self.stack.enter_context(patch.object(cycle, "project_roots", return_value=[self.root]))
        self.stack.enter_context(patch.object(cycle, "ExclusiveExecution", side_effect=lambda p: nullcontext()))

    def call(self, arm="left", identifier="cycle-1", statement=None):
        return self.service.call("robot_startup_after_arm_power_cycle", {
            "arm": arm, "power_cycle_id": identifier,
            "power_cycle_and_clearance_statement": cycle.confirmation(arm) if statement is None else statement})

    def original_rows(self):
        with sqlite3.connect(self.path) as db:
            names = [r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")
                     if r[0].startswith("pair_") and r[0] != cycle.TABLE]
            return {name: db.execute('SELECT * FROM "' + name + '"').fetchall() for name in names}

    def audit_history(self):
        with sqlite3.connect(self.path.as_uri() + '?mode=ro', uri=True) as db:
            db.row_factory = sqlite3.Row
            db.execute('PRAGMA query_only=ON')
            return cycle.audit_completed_startup_history(db)

    def history_call(self):
        # Executor, claim and journal use the same fake wall clock. A mixed
        # real/fake chronology must never be accepted by the audit.
        with patch.object(cycle, 'time', self.clock), patch('robot_tools.execution.time', self.clock):
            return self.call()

    def test_completed_history_audit_keeps_fault_and_all_original_rows(self):
        result = self.history_call()
        self.assertTrue(result['ok'])
        before = self.original_rows()
        audit = self.audit_history()
        self.assertEqual(audit['cycle_id'], 'cycle-1')
        self.assertTrue(audit['historical_projection_only'])
        self.assertFalse(audit['task_motion_authorized'])
        self.assertEqual(before, self.original_rows())
        self.assertIsNotNone(self.ledger.peek_status()['fault'])
        with self.assertRaises(RuntimeError):
            self.ledger.claim('new-owner-after-startup')

    def test_history_audit_rejects_missing_partial_extra_and_changed_evidence(self):
        result = self.history_call()
        path = Path(result['record_path'])
        original = path.read_text()
        for mutate in [lambda r:r.update(target_commands_sent=1),
                       lambda r:r['transmission_counts']['left'].update(sent_frames=1),
                       lambda r:r.update(task_motion_authorized=True),
                       lambda r:r['after']['left']['joints_rad'].__setitem__(0, 1.),
                       lambda r:r['before']['left']['arm_status'].update(ctrl_mode=1)]:
            body = json.loads(original); mutate(body); path.write_text(json.dumps(body))
            with self.subTest(mutation=mutate), self.assertRaises(RuntimeError):
                self.audit_history()
        path.write_text(original)
        journal = path.with_name('events.jsonl'); saved = journal.read_text()
        journal.write_text(saved + json.dumps({'event':'unaccounted_target','unix_s':1}) + '\n')
        with self.assertRaisesRegex(RuntimeError, 'Unaccounted'):
            self.audit_history()
        journal.write_text(saved)
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            self.audit_history()

    def test_history_audit_rejects_changed_task_or_pending_startup(self):
        self.history_call()
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_arm_power_cycles SET status="pending"')
        with self.assertRaisesRegex(RuntimeError, 'complete single-arm'):
            self.audit_history()
        with sqlite3.connect(self.path) as db:
            db.execute('UPDATE pair_arm_power_cycles SET status="complete"')
            db.execute('UPDATE pair_faults SET reason="changed"')
        with self.assertRaisesRegex(RuntimeError, 'Task history changed'):
            self.audit_history()

    def test_history_audit_rejects_an_unknown_added_pair_table(self):
        self.history_call()
        with sqlite3.connect(self.path) as db:
            db.execute('CREATE TABLE pair_unreviewed_extra (x TEXT)')
        with self.assertRaisesRegex(RuntimeError, 'Task history changed'):
            self.audit_history()

    def reset_fake_arms(self):
        self.robots = {s: fixtures.FakeStartupRobot(s) for s in ("left", "right")}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()

    def add_failed_event(self, *, uncertain=False):
        counts = {s: dict(attempted_frames=0, sent_frames=0, blocked_frames=0) for s in ("left", "right")}
        counts["left"] = dict(attempted_frames=4, sent_frames=3 if uncertain else 4, blocked_frames=0)
        result = {"ok": False, "device_receipt": {"transmission_counts": counts,
                  "original_event": {"send_state": "partial" if uncertain else "all_frames_returned",
                                     "frame_receipts": [{"outcome": "returned"}] * counts["left"]["sent_frames"]}}}
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                       ("old", "failed-event", "{}", "hash", 1, "complete", "old-owner", 100., 101.,
                        json.dumps(result), 0))

    def test_same_boot_fault_only_selected_arm_started_history_preserved(self):
        self.add_failed_event()
        self.robots["right"].ctrl_mode = 1
        self.robots["right"].driver_enabled = [True] * 6
        self.robots["right"].gripper_enabled = True
        before = self.original_rows()
        result = self.call()
        self.assertTrue(result["ok"], result)
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x151, 0x471])
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["target_commands_sent"], 0)
        self.assertFalse(result["task_motion_authorized"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertEqual(self.original_rows(), before)
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT arm,status FROM " + cycle.TABLE).fetchall(), [("left", "complete")])
        with self.assertRaisesRegex(RuntimeError, "no restart bypass"):
            self.service.call("robot_startup_arm", {"arm": "left"})

    def test_both_disabled_reuses_exact_pair_mode_enable_executor(self):
        r = self.call("both")
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["hardware_commands_sent"], 4)
        for robot in self.robots.values():
            self.assertEqual([f.arbitration_id for f in robot.sent], [0x151, 0x471])

    def test_wrong_confirmation_side_or_empty_event_refused_before_connection(self):
        for kwargs in ({"statement": cycle.confirmation("both")}, {"identifier": ""}, {"identifier": "../x"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.call(**kwargs)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_live_host_blocks_before_connection(self):
        self.processes.side_effect = RuntimeError("live host")
        with self.assertRaisesRegex(RuntimeError, "live host"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_service_pair_host_blocks_before_connection(self):
        self.service.pair_host = object()
        with self.assertRaisesRegex(RuntimeError, "双臂宿主"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_pending_send_blocks(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                       ("old", "pending", "{}", "hash", 1, "pending", "old-owner", 100., None, None, None))
        with self.assertRaisesRegex(RuntimeError, "未决发送"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_partial_failed_send_is_not_bypassed_by_power_cycle_statement(self):
        self.add_failed_event(uncertain=True)
        with self.assertRaisesRegex(RuntimeError, "部分完成"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_unknown_failed_receipt_refused(self):
        self.add_failed_event()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET receipt_json='{}'")
        with self.assertRaisesRegex(RuntimeError, "完整逐臂发送计数"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_unresolved_grasp_blocks(self):
        with sqlite3.connect(self.path) as db:
            db.execute("CREATE TABLE pair_grasp_episodes (state_json TEXT)")
            db.execute("INSERT INTO pair_grasp_episodes VALUES (?)", (json.dumps({"status": "retained_static"}),))
        with self.assertRaisesRegex(RuntimeError, "未解除持物"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_already_enabled_selected_arm_no_claim_no_send(self):
        self.robots["left"].ctrl_mode = 1
        self.robots["left"].driver_enabled = [True] * 6
        result = self.call()
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        with sqlite3.connect(self.path) as db:
            self.assertIsNone(db.execute("SELECT 1 FROM sqlite_master WHERE name=?", (cycle.TABLE,)).fetchone())

    def test_claim_durable_before_wire_send(self):
        def check(frame):
            with sqlite3.connect(self.path) as db:
                self.assertEqual(db.execute("SELECT status FROM " + cycle.TABLE).fetchall(), [("pending",)])
            return frame
        self.robots["left"].frame_transform = check
        self.assertTrue(self.call()["ok"])

    def test_same_cycle_cannot_be_replayed_after_success(self):
        self.assertTrue(self.call()["ok"])
        self.reset_fake_arms()
        self.sdk.AgxArmFactory.create_arm.reset_mock()
        with self.assertRaisesRegex(RuntimeError, "已经尝试"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_another_actual_reset_with_new_confirmation_can_start(self):
        self.assertTrue(self.call()["ok"])
        self.reset_fake_arms()  # Newly disabled hardware; same host boot.
        self.assertTrue(self.call(identifier="cycle-2")["ok"])
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM " + cycle.TABLE).fetchone()[0], 2)

    def test_failed_cycle_latches_even_with_new_identifier(self):
        self.robots["left"].enable_error = OSError("unknown wire result")
        self.assertFalse(self.call()["ok"])
        self.reset_fake_arms()
        self.sdk.AgxArmFactory.create_arm.reset_mock()
        with self.assertRaisesRegex(RuntimeError, "未决或失败"):
            self.call(identifier="different-id")
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_lost_process_claim_cannot_be_replaced(self):
        enrollment = cycle.inspect(self.root)
        cycle._reserve(self.path, "lost", "left", cycle.confirmation("left"), enrollment, self.root / "missing.json")
        with self.assertRaisesRegex(RuntimeError, "未决或失败"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_host_reboot_does_not_bypass_pending_arm_cycle(self):
        from robot_tools.reboot_startup import ledger_snapshot
        enrollment = cycle.inspect(self.root)
        cycle._reserve(self.path, "lost", "left", cycle.confirmation("left"), enrollment, self.root / "missing.json")
        with self.assertRaisesRegex(RuntimeError, "不能通过电脑重启"):
            ledger_snapshot(self.path, {"boot_id": "another-boot", "started_at": 500.})

    def test_cycle_history_selected_after_success(self):
        self.assertFalse(cycle.has_startup_history(self.root))
        self.assertTrue(self.call()["ok"])
        self.assertTrue(cycle.has_startup_history(self.root))

    def test_ledger_change_during_baseline_prevents_first_send(self):
        changed = False
        def change(robot, state):
            nonlocal changed
            if not changed and self.clock.elapsed > .5:
                with sqlite3.connect(self.path) as db:
                    db.execute("UPDATE pair_scope SET last_time=102")
                changed = True
        self.hook = change
        r = self.call()
        self.assertFalse(r["ok"])
        self.assertEqual(r["hardware_commands_sent"], 0)
        self.assertIn("账本变化", str(r["errors"]))

    def test_current_round_fault_audited_not_original_scope(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_runs VALUES ('new','{}',128,900,102,0)")
            db.execute("CREATE TABLE pair_rounds (ordinal INTEGER,run_id TEXT,active_run_id TEXT,owner TEXT,fault_id INTEGER,last_time REAL)")
            db.execute("INSERT INTO pair_rounds VALUES (1,'new','new','new-owner',1,102)")
            db.execute("INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                       ("new", "bad", "{}", "hash", 1, "complete", "new-owner", 102., 103., "{}", 0))
        with self.assertRaisesRegex(RuntimeError, "完整逐臂发送计数"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_lock_blocks_device_connection(self):
        from robot_tools.execution import ExclusiveExecution
        def lock(runs):
            return ExclusiveExecution(runs) if runs == self.root / "runs" else nullcontext()
        with ExclusiveExecution(self.root / "runs"), patch.object(cycle, "ExclusiveExecution", side_effect=lock):
            with self.assertRaises(RuntimeError):
                self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_pending_hold_blocks(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_hold_requests VALUES (?,?,?,?,?,?)",
                       ("old", "orig", "old-owner", 1, "reason", 101.))
        with self.assertRaisesRegex(RuntimeError, "保持"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_untracked_contact_probe_blocks(self):
        self.add_failed_event()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_events SET payload_json=?", (json.dumps({"operation": "grip_supported"}),))
        with self.assertRaisesRegex(RuntimeError, "未绑定物体释放"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_public_helper_to_real_service_native_startup_integration(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
        try:
            import can_motor_enable as helper
        finally:
            sys.path.pop(0)
        self.robots["right"].ctrl_mode = 1
        self.robots["right"].driver_enabled = [True] * 6
        self.robots["right"].gripper_enabled = True
        # The helper state read uses its normal read_state entry; only that
        # RX cache read is supplied here. All startup/service/wire code is real.
        before = {s: self.snapshot(r, r.gripper) for s, r in self.robots.items()}
        with patch.object(self.service, "read_state", return_value={
                "ok": True, "state": {"status": "complete", "arms": before}}):
            helper.execute(self.service, {"selected_arms": ["left", "right"]},
                           ownership_check=lambda: cycle.inspect(self.root),
                           arm_cycle_confirmation=lambda arm: ("cli-cycle", cycle.confirmation(arm)))
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x151, 0x471])
        self.assertEqual(self.robots["right"].sent, [])

    def test_equal_counts_do_not_hide_incomplete_startup_prefix(self):
        counts = {s: dict(attempted_frames=0, sent_frames=0, blocked_frames=0) for s in ("left", "right")}
        counts["left"].update(attempted_frames=1, sent_frames=1)
        with self.assertRaisesRegex(RuntimeError, "部分 mode/enable"):
            cycle._known_sends({"operation": "startup_arm", "transmission_counts": counts})

    def test_missing_native_frame_returns_cannot_be_relabelled_complete(self):
        self.add_failed_event()
        with sqlite3.connect(self.path) as db:
            r = json.loads(db.execute("SELECT receipt_json FROM pair_events").fetchone()[0])
            r["device_receipt"]["original_event"]["frame_receipts"].pop()
            db.execute("UPDATE pair_events SET receipt_json=?", (json.dumps(r),))
        with self.assertRaisesRegex(RuntimeError, "未完整返回"):
            self.call()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()
