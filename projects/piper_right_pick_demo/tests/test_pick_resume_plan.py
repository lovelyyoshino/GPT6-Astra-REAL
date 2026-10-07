"""Resume changes no goals and rejects consumed, partial, or unsafe sources."""
import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import pick_trajectory as trajectory
import pick_resume_plan as resume

Q = [36562, 281, -329, -1784, 21222, -9858]
POSE = [43201, 30764, 181596, -145000, 70132, -107442]


def synthetic_source(plan, completed=60):
    prep = [dict(kind='gripper', label=label, width_raw=width, effort_raw=300,
                 command_sent=True, feedback_verified=True)
            for label, width in [('observe', 23000), ('open', 55000)]]
    tx = [('0x471', '0702000000000000'), ('0x159', '000059d8012c0100'),
          ('0x159', '0000d6d8012c0100')]
    history = list(prep)
    for i, target in enumerate(plan['stages'][:completed]):
        assert target['kind'] == 'move'
        history.append(dict(kind='move', label=target['label'], target_joints_raw=target['joints_raw'],
                            target_pose_raw=target['pose_raw'], command_sent=True,
                            target_frame_attempted=True, arrival_verified=i < completed - 1))
        tx.append(('0x151', '0101050000000000'))
        for identifier, offset in [(0x155, 0), (0x156, 2), (0x157, 4)]:
            tx.append(('0x%03X' % identifier, struct.pack('>ii', *target['joints_raw'][offset:offset+2]).hex()))
    error, phase = 'Waypoint arrival was not verified within five seconds', 'move_' + history[-1]['label']
    return dict(status='failed_or_stopped', error=error, error_type='Rejected',
                failure=dict(error=error, type='Rejected', phase=phase), phase=phase,
                physical_grasp_attempts=0, partial_motion_target=False, plan_validated=True,
                enable_transmissions=1, plan=plan, geometry=plan['geometry'], stages=history,
                captures=[], transmissions=[dict(id=i, data_hex=d, send_result='accepted_by_host_socket') for i, d in tx])


class ResumePlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fk = trajectory._Builder(np.asarray(Q)/1000., np.asarray(POSE)/1000., {}).fk(np.asarray(Q)/1000.)
        geometry = dict(tcp_base_mm=(fk[:3] + trajectory._rotation(fk).apply([0, 0, 135.8])).tolist(),
                        cube_grasp_base_mm=[387.586, 276.111, -20.045], table_base_z_mm=-40.045,
                        uncertainty=dict(depth_mm=3, finger_feature_mm=12),
                        place_candidates=[dict(tcp_base_mm=[438.140, 214.109, -20.045], empty_patch_radius_mm=25)])
        with mock.patch('socket.socket', side_effect=AssertionError('hardware access')):
            cls.source = synthetic_source(trajectory.plan_pick(Q, POSE, geometry))
        target = cls.source['stages'][-1]
        cls.live = dict(joints_raw=target['target_joints_raw'], pose_raw=target['target_pose_raw'],
                        status=dict(ctrl_mode=1, arm_status=0, mode_feed=1, teach_status=0,
                                    motion_status=0, err_code=0), motor_enabled=[True]*6,
                        frame_age_s={k: .01 for k in resume._REQUIRED_AGES})

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'source' / 'report.json'
        self.path.parent.mkdir()
        self.source_copy = copy.deepcopy(self.source)
        self.write()

    def write(self):
        self.path.write_text(json.dumps(self.source_copy))

    def test_suffix_targets_unchanged_and_all_joint_boxes_rechecked_without_ik_or_hardware(self):
        with mock.patch('socket.socket', side_effect=AssertionError('hardware access')), \
                mock.patch.object(trajectory._Builder, 'solve', side_effect=AssertionError('new IK')):
            plan = resume.resume_plan(self.path, self.live)
        original = self.source['plan']['stages'][60:]
        for old, new in zip(original, plan['stages']):
            self.assertEqual(old['kind'], new['kind'])
            if old['kind'] == 'move':
                self.assertEqual(old['joints_raw'], new['joints_raw'])
                self.assertEqual(old['pose_raw'], new['pose_raw'])
                self.assertGreaterEqual(new['clearance_mm'], 10)
            else:
                self.assertEqual(old, new)
        count = sum(s['kind'] == 'move' for s in original)
        self.assertEqual(plan['assessment']['checked_states'], 73*count)
        self.assertEqual(plan['resume']['skipped_verified_move_count'], 59)
        self.assertFalse(plan['resume']['interrupted_target_reissued'])
        self.assertEqual(plan['start_joints_raw'], self.live['joints_raw'])
        self.assertEqual(plan['limits']['uncertainty_reserve_mm'], 15)
        self.assertEqual(plan['limits']['tracking_allowance_deg'], .5)

    def test_wrong_failure_grasp_history_partial_send_or_unverified_prefix_rejected(self):
        mutations = [lambda r: r.update(error='Motor fault'),
                     lambda r: r.update(physical_grasp_attempts=1),
                     lambda r: r.update(partial_motion_target=True),
                     lambda r: r['transmissions'].pop(),
                     lambda r: r['transmissions'][-1].update(data_hex='0000000000000000'),
                     lambda r: r['stages'][-1].update(command_sent=False),
                     lambda r: r['stages'][5].update(arrival_verified=False),
                     lambda r: r['stages'].append(dict(kind='gripper', label='close'))]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.source_copy = copy.deepcopy(self.source)
                mutation(self.source_copy); self.write()
                with self.assertRaises(trajectory.PlanRejected):
                    resume.resume_plan(self.path, self.live)

    def test_live_wrong_pose_moving_disabled_or_stale_rejected(self):
        for field, replacement in [('pose_raw', [self.live['pose_raw'][0]+1000]+self.live['pose_raw'][1:]),
                                   ('joints_raw', [self.live['joints_raw'][0]+301]+self.live['joints_raw'][1:]),
                                   ('status', dict(self.live['status'], motion_status=1)),
                                   ('motor_enabled', [False]+[True]*5),
                                   ('frame_age_s', {k: .2 for k in resume._REQUIRED_AGES})]:
            with self.subTest(field=field), self.assertRaises(trajectory.PlanRejected):
                resume.resume_plan(self.path, dict(self.live, **{field: replacement}))

    def test_collision_reserve_and_assets_cannot_be_relaxed(self):
        for mutation in [lambda r: r['geometry'].update(table_base_z_mm=100.),
                         lambda r: r['plan']['limits'].update(uncertainty_reserve_mm=3.),
                         lambda r: r['plan']['source_sha'].update(manufacturer_geometry={})]:
            self.source_copy = copy.deepcopy(self.source)
            mutation(self.source_copy); self.write()
            with self.assertRaises(trajectory.PlanRejected):
                resume.resume_plan(self.path, self.live)

    def test_sibling_close_consumes_source_even_if_command_unverified(self):
        prior = self.path.parent.parent / 'prior_resume'; prior.mkdir()
        (prior / 'report.json').write_text(json.dumps(dict(
            resume_source=str(self.path), physical_grasp_attempts=0,
            stages=[dict(kind='gripper', label='close', command_sent=False)])))
        with self.assertRaisesRegex(trajectory.PlanRejected, 'already consumed'):
            resume.resume_plan(self.path, self.live)

    def test_actual_failed_run_keeps_exact_43_moves_when_regression_artifact_available(self):
        path = Path(__file__).resolve().parents[1] / 'runs/pick_attempt_20261003T194811_102398/report.json'
        if not path.exists():
            self.skipTest('local recorded hardware regression is not bundled')
        source = json.loads(path.read_text())
        # Copy into an isolated run directory: this is a historical offline
        # regression, irrespective of whether a later real attempt consumed it.
        self.path.write_text(json.dumps(source))
        plan = resume.resume_plan(self.path, source['trace'][-1])
        self.assertEqual(plan['resume']['remaining_move_count'], 43)
        self.assertEqual(plan['assessment']['checked_states'], 3139)
        self.assertAlmostEqual(plan['resume']['live_arrival_residuals']['max_joint_deg'], .195)


if __name__ == '__main__':
    unittest.main()
