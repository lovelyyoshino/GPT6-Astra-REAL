"""Saved real RGBD -> real planner -> real SDK encoding, simulated actuators."""
from pathlib import Path
import json
import sys
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'tests'))
import test_direct_sdk_pick as fixture
from pick_scene_geometry import estimate_scene
from pick_trajectory import plan_pick
from pick_resume_plan import resume_plan


class CompletePickIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        q=[36562,281,-329,-1784,21222,-9858]
        pose=[43201,30764,181596,-145000,70132,-107442]
        path=ROOT/'runs/scene_bcc9cfee00364048aa89e330558ab646/observations/20261003T184759527238Z_7b6d74f9/observation.json'
        if not path.exists():
            raise unittest.SkipTest('Saved physical-scene fixture is not available')
        with mock.patch('socket.socket',side_effect=AssertionError('hardware access')):
            cls.plan=plan_pick(q,pose,estimate_scene(path,q,pose))

    def test_saved_scene_finishes_contact_and_empty_sequences_with_real_clearance(self):
        for closed_width,effort,outcome in ((30000,140,'contact_candidate'),(1000,0,'empty')):
            with self.subTest(outcome=outcome):
                rig=fixture.PickTests()
                rig.setUp()
                try:
                    rig.closed_width,rig.closed_effort=closed_width,effort
                    rig.joint_tracking_error[5]=30
                    rig.controller.enable()
                    observed=rig.controller.gripper(23000,'observe')
                    self.assertEqual(observed['classification'],'observation_opening')
                    self.assertEqual(rig.report.get('physical_grasp_attempts',0),0)
                    rig.controller.gripper(55000,'open')
                    with mock.patch.object(rig.controller,'observe',side_effect=lambda fn,**kw:fn()):
                        result=rig.controller.execute(self.plan,lambda label,sample:{'observation':label})
                    self.assertTrue(result['protocol_completed'])
                    self.assertEqual(result['grasp_outcome'],outcome)
                    self.assertFalse(result['grasp_success_verified'])
                    moves=[s for s in result['stages'] if s['kind']=='move']
                    self.assertEqual(len(moves),self.plan['assessment']['move_count'])
                    self.assertTrue(all(s['arrival_verified'] for s in moves))
                    self.assertEqual(moves[-1]['label'],'retract_after_release')
                    self.assertEqual(sum(i==0x159 for i,_ in rig.transport.frames),4)
                finally:
                    rig.tearDown()

    def test_real_stopped_plan_resumes_remaining_43_moves_through_release(self):
        source=ROOT/'runs/pick_attempt_20261003T194811_102398/report.json'
        if not source.exists():
            self.skipTest('Saved real partial run is unavailable')
        r=json.loads(source.read_text());d=r['final_diagnostic']
        sample={'joints_raw':[d['joints_raw']['joint_%d'%i] for i in range(1,7)],
                'pose_raw':[d['pose_raw'][k] for k in ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')],
                'status':d['status'], 'motor_enabled':[True]*6,
                'frame_age_s':d['required_frame_age_s']}
        plan=resume_plan(source,sample)
        self.assertEqual(plan['assessment']['move_count'],43)
        rig=fixture.PickTests();rig.setUp()
        try:
            rig.q=list(sample['joints_raw']);rig.populate()
            rig.prepared()
            with mock.patch.object(rig.controller,'observe',side_effect=lambda fn,**kw:fn()):
                result=rig.controller.execute(plan,lambda label,sample:{'observation':label})
            self.assertTrue(result['protocol_completed'])
            moves=[s for s in result['stages'] if s['kind']=='move']
            self.assertEqual(len(moves),43)
            self.assertTrue(all(s['arrival_verified'] for s in moves))
            self.assertEqual(moves[-1]['label'],'retract_after_release')
            self.assertEqual(result['physical_grasp_attempts'],1)
            self.assertFalse(result['grasp_success_verified'])
        finally:
            rig.tearDown()


if __name__=='__main__':
    unittest.main()
