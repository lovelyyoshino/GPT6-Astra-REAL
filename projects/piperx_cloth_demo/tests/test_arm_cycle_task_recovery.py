"""A new preparation round after a real-shaped audited startup; no hardware."""
import json
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools import arm_power_cycle, pair_round, reboot_startup
from robot_tools.pair_ledger import activated_execution_budget, PairLedgerError
from robot_tools.service import ToolService
import test_arm_power_cycle as startup_fixtures
import test_pair_tracking_round as tracking_fixtures


class ArmCycleTaskRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.f = tracking_fixtures.TrackingRoundTests('runTest')
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.c = startup_fixtures.ArmPowerCycleTests('runTest')
        self.c.setUp(); self.addCleanup(self.c.doCleanups)
        self.c.root, self.c.path = self.f.root, self.f.path
        self.c.service = ToolService(self.f.root)
        self.c.service.profile['sdk_path'] = startup_fixtures.PROFILE['sdk_path']
        self.c.clock.time = lambda: 7900. + self.c.clock.elapsed
        self.c.stack.enter_context(patch.object(arm_power_cycle, 'project_roots', return_value=[self.f.root]))
        self.c.stack.enter_context(patch.object(reboot_startup, 'boot_identity', side_effect=lambda: dict(self.c.boot)))
        self.startup = self.c.history_call()
        self.assertTrue(self.startup['ok'], self.startup)
        self.move_current_joint('left', 'joint_5', 73559)

    def move_current_joint(self, side, key, value):
        path = self.f.passive[side]
        data = json.loads(path.read_text())
        for sample in data['pose_trace']:
            sample['joints_raw'][key] = value
        path.write_text(json.dumps(data))

    def prepare(self, **kwargs):
        return self.f.prepare(recovery_origin='arm_cycle_current_pose', **kwargs)

    def test_new_pose_accepted_only_as_preparation_preserving_old_history(self):
        before = self.f.rows()
        proposal = self.prepare()
        self.assertEqual(before, self.f.rows())
        evidence = proposal['recovery_evidence']
        self.assertFalse(evidence['current_target_proximity_observed'])
        self.assertTrue(evidence['controller_limits_pending_live_query'])
        self.assertFalse(evidence['new_target_replay_authorized'])
        result = self.f.activate(proposal)
        self.assertEqual(result['required_connection_mode'], 'prepare')
        self.assertFalse(result['cache_or_limits_transferred'])
        after = self.f.rows()
        for name, rows in before.items():
            self.assertTrue(all(row in after[name] for row in rows), name)
        self.assertTrue(activated_execution_budget(self.f.path, 'tracking-new-round', max_steps=500, max_duration_s=3600))
        self.assertEqual(before['pair_faults'], after['pair_faults'])

    def test_default_target_recovery_does_not_silently_switch_origin(self):
        with self.assertRaises(PairLedgerError):
            self.f.prepare()

    def test_missing_or_incomplete_startup_refused(self):
        with sqlite3.connect(self.f.path) as db:
            db.execute("UPDATE pair_arm_power_cycles SET status='pending'")
        with self.assertRaises(RuntimeError):
            self.prepare()

    def test_missing_startup_cannot_be_replaced_by_an_origin_flag(self):
        with sqlite3.connect(self.f.path) as db:
            db.execute('DROP TABLE pair_arm_power_cycles')
        with self.assertRaisesRegex(PairLedgerError, 'audited completed startup'):
            self.prepare()

    def test_startup_mutation_between_prepare_and_activate_refused(self):
        proposal = self.prepare()
        path = self.c.root / 'runs' / self.startup['run_id'] / 'result.json'
        data = json.loads(path.read_text()); data['target_commands_sent'] = 1
        path.write_text(json.dumps(data))
        with self.assertRaises(RuntimeError):
            self.f.activate(proposal)

    def test_changed_host_boot_refused(self):
        self.c.boot['boot_id'] = 'another-boot'
        with self.assertRaisesRegex(PairLedgerError, 'current host boot'):
            self.prepare()

    def test_wrong_startup_side_cannot_rebase_the_failed_arm(self):
        with sqlite3.connect(self.f.path) as db:
            db.execute("UPDATE pair_arm_power_cycles SET arm='right'")
        with self.assertRaises(RuntimeError):
            self.prepare()

    def test_peer_moved_since_old_complete_send_refused(self):
        self.move_current_joint('right', 'joint_1', 15000)
        with self.assertRaisesRegex(PairLedgerError, 'unchanged peer'):
            self.prepare()

    def test_new_pose_beyond_piper_x_nominal_range_refused(self):
        self.move_current_joint('left', 'joint_5', 90000)
        with self.assertRaisesRegex(PairLedgerError, 'absolute joint limits'):
            self.prepare()

    def test_evidence_before_startup_and_mutated_history_refused(self):
        path = self.f.passive['left']; data = json.loads(path.read_text())
        data['started_at_s'] = 7890.; path.write_text(json.dumps(data))
        with self.assertRaises(PairLedgerError):
            self.prepare()
        with sqlite3.connect(self.f.path) as db:
            db.execute("UPDATE pair_faults SET reason=reason || ' changed' WHERE id=(SELECT MAX(id) FROM pair_faults)")
        with self.assertRaises((RuntimeError, PairLedgerError)):
            self.prepare()

    def test_no_recovery_mode_on_clean_round(self):
        with self.assertRaisesRegex(PairLedgerError, 'audited unloaded'):
            self.prepare(parent_kind='clean')


if __name__ == '__main__':
    unittest.main()
