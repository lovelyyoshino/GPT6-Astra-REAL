"""Post-reset initial budget enrollment using the real admin/ledger and FakeCAN."""
import copy
import json
from pathlib import Path
import shutil
import sqlite3
import time
import unittest
from unittest.mock import patch

from robot_tools import startup_reset
from robot_tools.pair_ledger import PairLedger, activated_execution_budget
from robot_tools.pair_host import PairHost
from robot_tools.pair_task_enrollment import preparation_only
import test_startup_reset
from test_pair_host import TASK


class ResetTaskBudgetTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_startup_reset.StartupResetTests('test_success_allows_a_new_task_but_does_not_resume_old_run')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.f = self.fixture
        profile = json.loads((self.f.root / 'configs/robot.json').read_text())
        profile.update(cameras={'front':'test-front', 'left_wrist':'test-left', 'right_wrist':'test-right'},
                       sdk_commit_audited=None)
        (self.f.root / 'configs/robot.json').write_text(json.dumps(profile))
        self.f.service.profile = profile
        shutil.copytree(Path(startup_reset.__file__).parent, self.f.root / 'robot_tools',
                        ignore=shutil.ignore_patterns('__pycache__'))
        startup_reset.reset(self.f.root)
        self.assertTrue(self.f.startup()['ok'])
        # The reset fixture stores a deliberately minimal historical grasp
        # table. Host integration needs the real empty post-reset schema.
        with sqlite3.connect(self.f.path) as db:
            db.execute('DROP TABLE pair_grasp_episodes')
        self.task = copy.deepcopy(TASK)
        self.task.update(worker_arm='left', support_arm='right')
        self.auth = dict(source='user_message', message_id='explicit-current-task',
                         statement='Start this new plug task with the increased budget', received_at=1.,
                         decision='authorize_explicit_new_task', max_steps=1000, max_duration_s=10800)

    def enroll(self, **kw):
        return startup_reset.enroll_task_budget(self.f.root, 'new-plug', self.task, self.auth, **kw)

    def test_enroll_and_reconnect_preserve_start_and_do_not_send(self):
        before = {s: len(r.sent) for s, r in self.f.robots.items()}
        record = self.enroll()
        self.assertEqual(before, {s: len(r.sent) for s, r in self.f.robots.items()})
        self.assertTrue(activated_execution_budget(self.f.path, 'new-plug', max_steps=1000, max_duration_s=10800))
        self.assertTrue(preparation_only(self.f.path, 'new-plug'))
        for _ in range(2):
            ledger = PairLedger(self.f.path, 'new-plug', record['contract'], max_steps=1000, max_duration_s=10800)
            self.assertEqual(ledger.status()['started_at'], record['budget']['started_at'])
        self.assertEqual(record['hardware_commands_sent'], 0)
        self.assertIsNone(record['physical_stop_verified'])

    def test_second_enrollment_or_different_run_cannot_reset_clock(self):
        self.enroll()
        before = self.f.rows()
        with self.assertRaisesRegex(RuntimeError, 'Existing task activity'):
            self.enroll()
        with self.assertRaisesRegex(RuntimeError, 'post-reset task'):
            PairLedger(self.f.path, 'bypass', {'task':'another'})
        self.assertEqual(before, self.f.rows())

    def test_used_initial_scope_is_not_a_new_round(self):
        PairLedger(self.f.path, 'already-opened', {'task':'existing'})
        with self.assertRaisesRegex(RuntimeError, 'Existing task activity'):
            self.enroll()

    def test_pending_or_failed_startup_refuses_without_mutation(self):
        for status in ('pending', 'failed'):
            with sqlite3.connect(self.f.path) as db:
                db.execute('UPDATE pair_startup_resets SET status=?', (status,))
            before = self.f.rows()
            with self.assertRaisesRegex(RuntimeError, 'successful reset startup'):
                self.enroll()
            self.assertEqual(before, self.f.rows())

    def test_budget_and_authorization_are_exact(self):
        for changes in ({'decision':'guess'}, {'source':'model'}, {'statement':''},
                        {'max_steps':True}, {'max_duration_s':900}, {'received_at':1e99}):
            saved = self.auth.copy(); self.auth.update(changes)
            with self.assertRaises((RuntimeError, ValueError)):
                self.enroll()
            self.auth = saved
        with self.assertRaisesRegex(RuntimeError, '1000'):
            self.enroll(max_steps=1001)

    def test_foreign_boot_archive_or_receipt_is_rejected(self):
        record = self.enroll()
        result = Path(record['startup']['result']['path'])
        data = json.loads(result.read_text()); data['ok'] = False
        result.write_text(json.dumps(data))
        with self.assertRaisesRegex(RuntimeError, 'startup receipt'):
            activated_execution_budget(self.f.path, 'new-plug', max_steps=1000, max_duration_s=10800)

    def test_ready_connection_is_rejected_before_constructing_device(self):
        self.enroll()
        factory = unittest.mock.Mock()
        with self.assertRaisesRegex(ValueError, 'preparation connection'):
            PairHost(self.f.root/'runs', self.f.service.profile, 'new-plug', self.task,
                     max_steps=1000, max_duration_s=10800, device_factory=factory)
        factory.assert_not_called()

    def test_current_fault_is_not_cleared_by_enrollment(self):
        with sqlite3.connect(self.f.path) as db:
            db.execute('UPDATE pair_scope SET fault_id=1')
        before = self.f.rows()
        with self.assertRaisesRegex(RuntimeError, 'Clean unowned'):
            self.enroll()
        self.assertEqual(before, self.f.rows())

    def test_changed_frozen_budget_is_not_recognized(self):
        self.enroll()
        with sqlite3.connect(self.f.path) as db:
            db.execute('UPDATE pair_runs SET max_steps=999')
        with self.assertRaisesRegex(RuntimeError, 'Frozen initial task budget'):
            activated_execution_budget(self.f.path, 'new-plug', max_steps=999, max_duration_s=10800)

    def test_new_boot_and_archive_changes_do_not_grant_budget(self):
        record = self.enroll()
        self.f.boot['boot_id'] = 'later-boot'
        with self.assertRaisesRegex(RuntimeError, 'Current successful'):
            activated_execution_budget(self.f.path, 'new-plug', max_steps=1000, max_duration_s=10800)
        self.f.boot['boot_id'] = record['startup']['boot_id']
        Path(record['startup']['archive_path']).write_bytes(b'corrupted archive')
        with self.assertRaisesRegex(RuntimeError, 'archive changed'):
            activated_execution_budget(self.f.path, 'new-plug', max_steps=1000, max_duration_s=10800)

    def test_service_opens_real_host_in_prepare_with_exact_frozen_budget(self):
        from robot_tools.service import ToolService
        from test_host_preparation import PreparationDevice
        record = self.enroll()
        devices = []
        class Clock:
            sleep = staticmethod(time.sleep)
            time = staticmethod(time.time)
        def make_device(profile, journal, guard):
            device = PreparationDevice(profile, journal, guard, Clock())
            devices.append(device)
            return device
        def make_host(*args, **kwargs):
            return PairHost(*args, **kwargs, device_factory=make_device, background=False)
        service = ToolService(self.f.root)
        service.persistent = True
        with patch('robot_tools.pair_host.PairHost', side_effect=make_host):
            for _ in range(2):
                result = service.call('robot_pair_open', dict(run_id='new-plug', task_id='plug_transfer_left',
                    workspace_clearance_statement=self.task['site_context']['workspace_clearance']['statement'],
                    worker_arm='left', support_arm='right', connection_mode='prepare',
                    max_steps=1000, max_duration_s=10800))
                self.assertTrue(result['open'])
                self.assertFalse(result['task_ready'])
                self.assertEqual(result['ledger']['started_at'], record['budget']['started_at'])
                self.assertEqual(result['ledger']['steps'], 0)
                service.call('robot_pair_close', {})
        self.assertTrue(all(device.calls == [] for device in devices))


if __name__ == '__main__':
    unittest.main()
