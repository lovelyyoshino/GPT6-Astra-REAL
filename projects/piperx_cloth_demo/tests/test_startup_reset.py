"""Real reset transactions and existing startup executor; FakeCAN only."""
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
from unittest.mock import patch

from robot_tools import startup_reset as reset
from robot_tools.pair_ledger import PairLedger, platform_state
from robot_tools.service import ToolService
from test_backend import PROFILE
import test_takeover as fixtures


class StartupResetTests(fixtures.TakeoverFixture):
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
        with sqlite3.connect(self.path) as db:
            db.execute("CREATE TABLE pair_grasp_episodes (state_json TEXT)")
            db.execute("INSERT INTO pair_grasp_episodes VALUES (?)", (json.dumps({
                "status": "retained_static", "identity": {"object_id": "white_charger", "arm": "left"},
                "physical_stop_verified": None}),))
        self.service = ToolService(self.root)
        self.boot = {"boot_id": "new-boot", "started_at": 200.}
        self.processes = self.stack.enter_context(patch.object(reset, "check_processes"))
        self.stack.enter_context(patch.object(reset, "boot_identity", side_effect=lambda: dict(self.boot)))
        self.stack.enter_context(patch.object(reset, "project_roots", return_value=[self.root]))
        self.stack.enter_context(patch.object(reset, "ExclusiveExecution", side_effect=lambda p: nullcontext()))

    def rows(self, path=None):
        with sqlite3.connect(path or self.path) as db:
            return {n: db.execute('SELECT * FROM "'+n+'"').fetchall() for (n,) in
                    db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'")}

    def startup(self, arm="both", statement=reset.CONFIRMATION):
        return self.service.call("robot_startup_after_state_reset", {
            "arm": arm, "power_cycle_and_clearance_statement": statement})

    def test_archive_preserves_original_grasp_fault_owner_budget_and_stays_readable(self):
        before = self.rows()
        result = reset.reset(self.root)
        self.assertTrue(result["reset_performed"])
        self.assertEqual(self.rows(result["archive_path"]), before)
        self.assertEqual(hashlib.sha256(Path(result["archive_path"]).read_bytes()).hexdigest(), result["archive_sha256"])
        self.assertEqual(self.rows()["pair_grasp_episodes"], [])
        self.assertEqual(self.rows()["pair_runs"], [])
        state = platform_state(self.path)
        self.assertIsNone(state["owner"])
        self.assertEqual(state["fault"]["reason"], "software_reset_requires_new_startup")
        self.sdk.AgxArmFactory.create_arm.assert_not_called()
        with self.assertRaises(RuntimeError):
            self.service.call("robot_startup_arms", {})
        with self.assertRaises(RuntimeError):
            PairLedger(self.path, "new-task", {"task":"new"}).claim("new-owner")

    def test_repeat_reset_reuses_pending_scene_without_another_archive(self):
        first = reset.reset(self.root)
        before = self.rows()
        second = reset.reset(self.root)
        self.assertEqual(second["reset_id"], first["reset_id"])
        self.assertFalse(second["reset_performed"])
        self.assertEqual(self.rows(), before)

    def test_live_process_refuses_without_touching_history(self):
        before = self.rows()
        self.processes.side_effect = RuntimeError("live host")
        with self.assertRaisesRegex(RuntimeError, "live host"):
            reset.reset(self.root)
        self.assertEqual(self.rows(), before)

    def test_current_boot_task_and_invalid_timestamps_refuse(self):
        for stamp in (201., -1.):
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE pair_scope SET last_time=?", (stamp,))
            before = self.rows()
            with self.assertRaises(RuntimeError):
                reset.reset(self.root)
            self.assertEqual(before, self.rows())

    def test_pending_send_is_not_erased(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                       ("old", "pending", "{}", "hash", 1, "pending", "old-owner", 100., None, None, None))
        before = self.rows()
        with self.assertRaisesRegex(RuntimeError, "未决发送"):
            reset.reset(self.root)
        self.assertEqual(before, self.rows())

    def test_archive_failure_rolls_back_without_clearing_original(self):
        before = self.rows()
        with patch("robot_tools.service._write", side_effect=OSError("disk full")), \
                self.assertRaisesRegex(OSError, "disk full"):
            reset.reset(self.root)
        self.assertEqual(before, self.rows())

    def test_full_startup_uses_existing_exact_frames_and_leaves_archive_unchanged(self):
        enrollment = reset.reset(self.root)
        result = self.startup()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["target_commands_sent"], 0)
        self.assertIsNone(result["physical_release_verified"])
        self.assertIsNone(platform_state(self.path)["fault"])
        self.assertEqual(reset.inspect(self.root)["status"], "complete")
        self.assertEqual(hashlib.sha256(Path(enrollment["archive_path"]).read_bytes()).hexdigest(), enrollment["archive_sha256"])
        for robot in self.robots.values():
            self.assertEqual([f.arbitration_id for f in robot.sent], [0x151, 0x471])

    def cycle_context(self):
        from robot_tools import arm_power_cycle as cycle
        self.stack.enter_context(patch.object(cycle, "check_processes"))
        self.stack.enter_context(patch.object(cycle, "boot_identity", side_effect=lambda: dict(self.boot)))
        self.stack.enter_context(patch.object(cycle, "project_roots", return_value=[self.root]))
        self.stack.enter_context(patch.object(cycle, "ExclusiveExecution", side_effect=lambda p: nullcontext()))
        return cycle

    def test_cli_all_after_completed_reset_uses_new_cycle_without_rewriting_history(self):
        reset.reset(self.root)
        self.assertTrue(self.startup()["ok"])
        before = self.rows()
        cycle = self.cycle_context()
        # New physical power cycle is simulated, with the host boot unchanged.
        self.robots = {s: fixtures.FakeStartupRobot(s) for s in ("left", "right")}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        states = {s: self.snapshot(r, r.gripper) for s, r in self.robots.items()}
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
        try:
            import can_motor_enable as helper
        finally:
            sys.path.pop(0)
        with patch.object(self.service, "read_state", return_value={
                "ok": True, "state": {"status": "complete", "arms": states}}), \
                patch("robot_tools.reboot_startup.check_processes"):
            helper.execute(self.service, {"selected_arms": ["left", "right"]},
                           ownership_check=lambda: helper.check_ownership(self.root),
                           arm_cycle_confirmation=lambda arm: ("after-reset", cycle.confirmation(arm)))
        after = self.rows()
        for name, rows in before.items():
            self.assertEqual(after[name], rows, name)
        self.assertEqual(len(after[cycle.TABLE]), 1)
        for robot in self.robots.values():
            self.assertEqual([f.arbitration_id for f in robot.sent], [0x151, 0x471])
        self.assertEqual(reset.inspect(self.root)["status"], "complete")

    def test_cycle_cannot_replace_first_pending_or_failed_reset_startup(self):
        reset.reset(self.root)
        cycle = self.cycle_context()
        for status in ("awaiting_startup", "pending", "failed"):
            with self.subTest(status=status), sqlite3.connect(self.path) as db:
                db.execute("UPDATE " + reset.TABLE + " SET status=?", (status,))
            with self.assertRaises(RuntimeError):
                cycle.inspect(self.root)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_completed_reset_does_not_bypass_failed_or_pending_cycle(self):
        reset.reset(self.root)
        self.assertTrue(self.startup()["ok"])
        cycle = self.cycle_context()
        enrollment = cycle.inspect(self.root)
        self.assertIn("completed_reset_startup", enrollment)
        cycle._reserve(self.path, "incomplete", "both", cycle.confirmation("both"),
                       enrollment, self.root / "missing.json")
        for status in ("pending", "failed"):
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE " + cycle.TABLE + " SET status=?", (status,))
            with self.subTest(status=status), self.assertRaisesRegex(RuntimeError, "未决或失败"):
                cycle.inspect(self.root)

    def test_confirmation_rejection_does_not_connect_or_change_new_latch(self):
        reset.reset(self.root)
        with self.assertRaises(ValueError):
            self.startup(statement="both rebooted but object remains between jaws")
        self.sdk.AgxArmFactory.create_arm.assert_not_called()
        self.assertEqual(reset.inspect(self.root)["status"], "awaiting_startup")

    def test_enabled_passive_arm_blocks_reset_startup_before_any_frame(self):
        reset.reset(self.root)
        self.robots["right"].ctrl_mode = 1
        self.robots["right"].driver_enabled = [True]*6
        result = self.startup("left")
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(reset.inspect(self.root)["status"], "awaiting_startup")

    def test_claim_exists_before_first_can_send(self):
        reset.reset(self.root)
        def inspect_frame(frame):
            with sqlite3.connect(self.path) as db:
                self.assertEqual(db.execute("SELECT status FROM "+reset.TABLE).fetchone()[0], "pending")
            return frame
        self.robots["left"].frame_transform = inspect_frame
        self.assertTrue(self.startup()["ok"])

    def test_partial_send_failure_cannot_be_reset_or_retried_in_same_boot(self):
        reset.reset(self.root)
        self.robots["left"].enable_error = OSError("uncertain send")
        self.assertFalse(self.startup()["ok"])
        before = self.rows()
        counts = {s:len(r.sent) for s,r in self.robots.items()}
        for action in (lambda: reset.reset(self.root), lambda: self.startup()):
            with self.assertRaisesRegex(RuntimeError, "未决或失败"):
                action()
        self.assertEqual(before, self.rows())
        self.assertEqual(counts, {s:len(r.sent) for s,r in self.robots.items()})

    def test_success_cannot_be_replayed_even_with_disabled_fake_hardware(self):
        reset.reset(self.root)
        self.assertTrue(self.startup()["ok"])
        self.assertFalse(reset.reset(self.root)["reset_performed"])
        with self.assertRaisesRegex(RuntimeError, "不重发"):
            self.startup()

    def test_success_allows_a_new_task_but_does_not_resume_old_run(self):
        reset.reset(self.root)
        self.assertTrue(self.startup()["ok"])
        fresh = PairLedger(self.path, "explicit-new-task", {"task":"new"})
        fresh.claim("new-owner")
        self.assertEqual(platform_state(self.path)["owner"], "new-owner")
        self.assertNotIn("old", [r[0] for r in self.rows()["pair_runs"]])

    def test_modified_archive_blocks_startup(self):
        state = reset.reset(self.root)
        Path(state["archive_path"]).write_bytes(b"changed")
        with self.assertRaisesRegex(RuntimeError, "归档缺失或已改变"):
            self.startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_pending_reset_cannot_be_erased_even_after_another_boot(self):
        reset.reset(self.root)
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE "+reset.TABLE+" SET status='pending'")
        self.boot = {"boot_id":"another-boot", "started_at":9999999999.}
        with self.assertRaisesRegex(RuntimeError, "仍未决"):
            reset.reset(self.root)

    def test_unreadable_or_foreign_schema_is_not_cleared(self):
        with sqlite3.connect(self.path) as db:
            db.execute("CREATE TABLE foreign_data(value TEXT)")
        with self.assertRaisesRegex(RuntimeError, "非 pair 表"):
            reset.reset(self.root)
