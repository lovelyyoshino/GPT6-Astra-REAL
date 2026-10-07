"""Meaningful offline checks against recorded feedback; no robot/socket access."""
import importlib.util
from pathlib import Path
import types
import unittest
from unittest import mock

PATH = Path(__file__).resolve().parents[1]/'scripts'/'live_lift_plan.py'
SPEC = importlib.util.spec_from_file_location('live_lift_plan_test', PATH)
planner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(planner)

CASES = [
    ([36562,-1420,2940,3243,20027,-3262], [41443,32932,165866,179615,73485,-142680]),
    ([36562,-1420,2961,-6295,19667,-3052], [44620,28912,166421,-149980,71334,-114259]),
    ([36562,-1420,2871,-1949,19739,-9688], [43210,30745,166674,-145015,70117,-107467]),
]


class LiveLiftPlanTests(unittest.TestCase):
    def test_three_distinct_recorded_poses_plan_only_fixed_lift_without_sockets(self):
        import numpy as np
        from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
        from scipy.spatial.transform import Rotation
        fk = C_PiperForwardKinematics(1)
        for q, pose in CASES:
            with self.subTest(q=q), mock.patch('socket.socket', side_effect=AssertionError('hardware access')):
                result = planner.plan_lift(q, pose)
            self.assertEqual(result['start_joints_raw'], q)
            self.assertEqual(result['start_pose_raw'], pose)
            expected = list(pose); expected[2] += 15000
            self.assertEqual(result['target_pose_raw'], expected)
            target = result['target_joints_raw']
            self.assertLessEqual(max(abs(a-b) for a,b in zip(q,target)),4000)
            for raw, (lo,hi) in zip(target,planner.LIMITS_DEG):
                self.assertGreaterEqual(raw,lo*1000); self.assertLessEqual(raw,hi*1000)
            achieved = np.asarray(fk.CalFK(np.deg2rad(np.asarray(target)/1000).tolist())[-1])
            expected = np.asarray(expected)/1000
            self.assertLess(np.linalg.norm(achieved[:3]-expected[:3]),.1)
            delta = (Rotation.from_euler('xyz',expected[3:],degrees=True).inv()*
                     Rotation.from_euler('xyz',achieved[3:],degrees=True)).as_rotvec()
            self.assertLess(np.rad2deg(np.linalg.norm(delta)),.03)
            b=result['model_bounds']; self.assertEqual(b['sample_count'],793)
            self.assertEqual(b['joint_lower_raw'],[min(a,c)-150 for a,c in zip(q,target)])
            self.assertEqual(b['joint_upper_raw'],[max(a,c)+150 for a,c in zip(q,target)])
            self.assertLessEqual(b['end_xy_max_mm'],3)
            self.assertGreaterEqual(b['end_z_delta_min_mm'],-7)
            self.assertLessEqual(b['end_z_delta_max_mm'],23)
            self.assertLessEqual(b['orientation_max_deg'],5)
            self.assertLessEqual(b['gripper_point_dip_max_mm'],15)
            self.assertTrue(all(len(v)==64 for v in result['source_sha'].values()))

    def test_historical_zero_pose_rejected_for_excess_horizontal_progress_box(self):
        with self.assertRaisesRegex(ValueError,'movement envelope'):
            planner.plan_lift([23,0,0,0,0,0],[56127,23,213266,0,85000,23])

    def test_wrong_pose_units_or_fk_mismatch_cannot_plan(self):
        q,p=CASES[0]; wrong=list(p);wrong[0]+=1000
        with self.assertRaisesRegex(ValueError,'FK does not match'):
            planner.plan_lift(q,wrong)
        wrong=list(p);wrong[3]+=1000
        with self.assertRaisesRegex(ValueError,'FK does not match'):
            planner.plan_lift(q,wrong)

    def test_only_specific_small_current_overrun_is_accepted(self):
        q,p=CASES[0]
        for index,value in [(0,150001),(1,-3001),(2,4001),(3,-100001)]:
            bad=list(q);bad[index]=value
            with self.subTest(index=index), self.assertRaisesRegex(ValueError,'input range'):
                planner.plan_lift(bad,p)

    def test_raw_inputs_are_exact_six_signed_integers(self):
        q,p=CASES[0]
        for bad in [q[:5],[True]+q[1:],[1.5]+q[1:],[float('nan')]+q[1:],[2**40]+q[1:]]:
            with self.subTest(bad=bad),self.assertRaisesRegex(ValueError,'six integer'):
                planner.plan_lift(bad,p)

    def test_solver_output_must_meet_endpoint_and_nominal_limits(self):
        import numpy as np
        q,p=CASES[0]
        for endpoint in [[v/1000 for v in q], [36.562,90,-90,3.243,20.027,-3.262]]:
            fake=types.SimpleNamespace(x=np.asarray(endpoint),success=True)
            with mock.patch('scipy.optimize.least_squares',return_value=fake):
                with self.assertRaisesRegex(ValueError,'endpoint tolerance'):
                    planner.plan_lift(q,p)


if __name__ == '__main__':
    unittest.main()
