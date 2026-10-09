"""Offline evidence that arm reassignment preserves the left socket goal."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runner


class RolePlanTests(unittest.TestCase):
    def test_mirrored_arms_preserve_socket_identity_and_canonical_source(self):
        source = runner.RECIPE.read_bytes()
        original = runner.plan()
        mirrored = runner.plan('left', 'right')
        self.assertEqual(source, runner.RECIPE.read_bytes())
        self.assertEqual(original['recipe'], json.loads(source))
        self.assertEqual(mirrored['source_recipe_sha256'], hashlib.sha256(source).hexdigest())
        self.assertEqual(mirrored['recipe_sha256'],
                         hashlib.sha256((runner.encode(mirrored['recipe']) + '\n').encode()).hexdigest())
        self.assertEqual(mirrored['roles'], {'left': 'plug_worker', 'right': 'power_strip_stabilizer'})
        for before, after in zip(original['recipe']['steps'], mirrored['recipe']['steps']):
            self.assertEqual(after['id'], before['id'])
            self.assertEqual(after['operation'], before['operation'])
            self.assertNotEqual(after['arm'], before['arm'])
            for evidence in before.get('evidence', []):
                if evidence.startswith('left_target_socket') or evidence == 'plug_seated_in_left_socket':
                    self.assertIn(evidence, after['evidence'])
        by_id = {step['id']: step for step in mirrored['recipe']['steps']}
        self.assertIn('left_gripper_clear', by_id['release_plug']['evidence'])
        self.assertIn('right_gripper_clear', by_id['release_strip']['evidence'])
        self.assertIn('frozen left target socket', by_id['insert_left_socket']['goal'])
        self.assertIn('right arm', by_id['transfer_left']['goal'])
        self.assertFalse(mirrored['dispatch_authorized'])

    def test_session_role_mismatch_cannot_construct_or_call_service(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = Mock()
            make_service = Mock(return_value=backend)
            session = runner.Session(Path(directory) / 'session', worker_arm='left', support_arm='right',
                                     make_service=make_service, make_rig=Mock())
            self.addCleanup(session.close)
            opening = {'run_id': 'offline-test', 'task_id': 'plug_transfer_left'}
            for extra in ({}, {'worker_arm': 'right', 'support_arm': 'left'},
                          {'worker_arm': 'left'}, {'worker_arm': 'left', 'support_arm': 'left'}):
                with self.assertRaises(ValueError):
                    session.request({'id': 'refused', 'op': 'robot_pair_open',
                                     'arguments': {**opening, **extra}})
            make_service.assert_not_called()
            self.assertFalse(session.open_attempted)
            backend.call.return_value = {'status': 'owned', 'run_id': 'offline-test',
                                         'open': True, 'ledger': {}, 'fault_latched': False}
            valid = {**opening, 'worker_arm': 'left', 'support_arm': 'right'}
            before = copy.deepcopy(valid)
            session.request({'id': 'valid', 'op': 'robot_pair_open', 'arguments': valid})
            backend.call.assert_called_once_with('robot_pair_open', before)
            self.assertEqual(valid, before)
            task = json.loads((session.directory / 'task.json').read_text())
            recipe = json.loads((session.directory / 'recipe.json').read_text())
            self.assertEqual(task['rendered_recipe_sha256'], hashlib.sha256(
                Path(task['rendered_recipe_path']).read_bytes()).hexdigest())
            self.assertEqual(task['recipe_sha256'], hashlib.sha256(runner.RECIPE.read_bytes()).hexdigest())
            self.assertEqual(task['worker_arm'], 'left')
            self.assertEqual(next(step for step in recipe['steps'] if step['id'] == 'extract_plug')['arm'], 'left')


if __name__ == '__main__':
    unittest.main()
