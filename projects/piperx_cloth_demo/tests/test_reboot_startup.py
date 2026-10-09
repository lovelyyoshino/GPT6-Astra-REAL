"""Reboot enrollment through real service/startup logic and FakeCAN only."""
from contextlib import nullcontext
import json
from pathlib import Path
import sqlite3
import tempfile
from unittest.mock import patch

from robot_tools import reboot_startup as reboot
from robot_tools.pair_ledger import PairLedger
from robot_tools.service import ToolService
from test_backend import PROFILE
import test_takeover as fixtures


class RebootStartupTests(fixtures.TakeoverFixture):
    snapshot = fixtures.StartupTests.snapshot

    def setUp(self):
        super().setUp()
        self.robots = {side: fixtures.FakeStartupRobot(side) for side in ("left", "right")}
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
        self.boot = {"boot_id": "new-boot", "started_at": 200.}
        self.processes = self.stack.enter_context(patch.object(reboot, "check_processes"))
        self.stack.enter_context(patch.object(reboot, "boot_identity", side_effect=lambda: dict(self.boot)))
        self.stack.enter_context(patch.object(reboot, "project_roots", return_value=[self.root]))
        self.stack.enter_context(patch.object(reboot, "ExclusiveExecution", side_effect=lambda p: nullcontext()))

    def run_startup(self, arm="both", statement=reboot.CONFIRMATION):
        return self.service.call("robot_startup_after_host_reboot", {
            "arm": arm, "power_cycle_and_clearance_statement": statement})

    def original_rows(self):
        with sqlite3.connect(self.path) as db:
            return {table: db.execute("SELECT * FROM " + table).fetchall()
                    for table in ("pair_scope", "pair_runs", "pair_events", "pair_faults")}

    def test_original_executor_sends_once_and_old_fault_budget_owner_unchanged(self):
        before = self.original_rows()
        result = self.run_startup()
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["target_commands_sent"], 0)
        self.assertEqual(self.original_rows(), before)
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT arm,status FROM pair_reboot_startups ORDER BY arm").fetchall(),
                             [("left", "complete"), ("right", "complete")])
        with self.assertRaises(RuntimeError):
            self.service.call("robot_startup_arms", {})  # Legacy still faults.
        with self.assertRaises(RuntimeError):
            self.service.call("robot_single_arm_move_once", {
                "arm": "left", "target_pose_m_rad": [.2, .1, .3, 0., 0., 0.]})

    def test_single_startup_preserves_passive_arm(self):
        result = self.run_startup("right")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151, 0x471])

    def test_operator_confirmation_required_before_device_connection(self):
        with self.assertRaises(ValueError):
            self.run_startup(statement="unconfirmed scene and robot power state")
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_active_host_is_not_retired(self):
        self.processes.side_effect = RuntimeError("live controller")
        with self.assertRaisesRegex(RuntimeError, "live controller"):
            self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_current_boot_task_activity_refuses(self):
        self.boot["started_at"] = 99.
        with self.assertRaisesRegex(RuntimeError, "本次电脑启动后的任务活动"):
            self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_invalid_history_time_is_not_a_same_boot_route_hint(self):
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_scope SET last_time=-1")
        with self.assertRaisesRegex(RuntimeError, "时间无效") as raised:
            self.run_startup()
        self.assertNotIsInstance(raised.exception, reboot.HostBootRequired)
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_latest_scope_checked_instead_of_only_original(self):
        with sqlite3.connect(self.path) as db:
            db.execute("CREATE TABLE pair_rounds (ordinal INTEGER,run_id TEXT,owner TEXT,fault_id INTEGER,last_time REAL)")
            db.execute("INSERT INTO pair_rounds VALUES(1,'newer','newer-owner',1,201)")
        with self.assertRaisesRegex(RuntimeError, "本次电脑启动后的任务活动"):
            self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_pending_send_is_not_bypassed_after_reboot(self):
        with sqlite3.connect(self.path) as db:
            db.execute("INSERT INTO pair_events VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                       ("old", "pending", "{}", "hash", 1, "pending", "old-owner", 100., None, None, None))
        with self.assertRaisesRegex(RuntimeError, "未决发送"):
            self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_live_enabled_arm_refuses_without_claim_or_send(self):
        self.robots["left"].driver_enabled[0] = True
        result = self.run_startup()
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        with sqlite3.connect(self.path) as db:
            self.assertIsNone(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reboot_startups'").fetchone())

    def test_claim_exists_before_first_wire_send(self):
        def inspect_frame(frame):
            with sqlite3.connect(self.path) as db:
                self.assertEqual(db.execute("SELECT COUNT(*) FROM pair_reboot_startups WHERE status='pending'").fetchone()[0], 2)
            return frame
        self.robots["left"].frame_transform = inspect_frame
        self.assertTrue(self.run_startup()["ok"])

    def test_send_error_latches_this_boot_and_never_retries(self):
        self.robots["left"].enable_error = OSError("fake uncertain send")
        result = self.run_startup()
        self.assertFalse(result["ok"])
        self.assertEqual(result["enable_commands_sent"], 0)
        before = {s: len(r.sent) for s, r in self.robots.items()}
        with self.assertRaisesRegex(RuntimeError, "未决或失败"):
            self.run_startup()
        self.assertEqual({s: len(r.sent) for s, r in self.robots.items()}, before)

    def test_process_loss_after_claim_blocks_reentry(self):
        enrollment = reboot.inspect(self.root)
        reboot.reserve(self.path, enrollment, ["left", "right"], "lost-process", self.root / "absent.json")
        with self.assertRaisesRegex(RuntimeError, "未决或失败"):
            self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_completed_boot_arm_claim_cannot_send_again(self):
        self.assertTrue(self.run_startup()["ok"])
        self.robots = {side: fixtures.FakeStartupRobot(side) for side in ("left", "right")}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        result = self.run_startup()
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("已使用过", str(result["errors"]))

    def test_existing_execution_lock_prevents_hardware_connection(self):
        from robot_tools.execution import ExclusiveExecution
        def lock(runs):
            return ExclusiveExecution(runs) if runs == self.root / "runs" else nullcontext()
        with ExclusiveExecution(self.root / "runs"), patch.object(reboot, "ExclusiveExecution", side_effect=lock):
            with self.assertRaises(RuntimeError):
                self.run_startup()
        self.sdk.AgxArmFactory.create_arm.assert_not_called()

    def test_new_task_activity_during_baseline_refuses_before_first_send(self):
        changed = False
        def change_ledger(robot, state):
            nonlocal changed
            if not changed and self.clock.elapsed > .5:
                with sqlite3.connect(self.path) as db:
                    db.execute("UPDATE pair_scope SET last_time=201")
                changed = True
        self.hook = change_ledger
        result = self.run_startup()
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("本次电脑启动后的任务活动", str(result["errors"]))
