"""Offline saved-scene regression plus meaningful bad-depth/pose rejections."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('pick_geometry_test',ROOT/'scripts/pick_scene_geometry.py')
geometry=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(geometry)
OBS=ROOT/'runs/scene_bcc9cfee00364048aa89e330558ab646/observations/20261003T184759527238Z_7b6d74f9/observation.json'
Q=[36562,281,-329,-1784,21222,-9858]
POSE=[43201,30764,181596,-145000,70132,-107442]
OPEN_RUN=ROOT/'runs/pick_attempt_20261003T193639_98200'


@unittest.skipUnless(OBS.exists(),'Recorded local scene unavailable')
class PickSceneGeometryTests(unittest.TestCase):
    def test_real_scene_auto_fingers_cube_and_rotation_without_sockets(self):
        import numpy as np
        from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
        with mock.patch('socket.socket',side_effect=AssertionError('hardware access')):
            r=geometry.estimate_scene(OBS,Q,POSE)
        json.dumps(r,allow_nan=False)
        self.assertAlmostEqual(r['cube_height_mm'],30,delta=3)
        self.assertEqual(len(r['fingers']),2)
        self.assertTrue(all(f['valid_near_depth_pixels']>=12 for f in r['fingers']))
        self.assertAlmostEqual(r['finger_spacing_mm'],30.7,delta=4)
        R=np.array(r['R_base_camera_approx']);n=np.array(r['table_plane_camera']['normal'])
        self.assertTrue(np.allclose(R.T@R,np.eye(3),atol=1e-6))
        self.assertAlmostEqual(np.linalg.det(R),1,places=6)
        self.assertTrue(np.allclose(R@n,[0,0,1],atol=1e-6))
        self.assertGreater(r['forward_direction_agreement'],.8)
        self.assertAlmostEqual(r['cube_grasp_base_mm'][2]-r['table_base_z_mm'],20,places=5)
        self.assertGreater(r['uncertainty']['finger_feature_mm'],9)
        self.assertGreaterEqual(len(r['place_candidates']),1)
        for p in r['place_candidates']:
            self.assertGreaterEqual(p['valid_fraction'],.85)
            self.assertLessEqual(p['p95_plane_residual_mm'],7)

    def altered_observation(self, directory, modify):
        obj=json.loads(OBS.read_text());modify(obj)
        p=Path(directory)/'observation.json';p.write_text(json.dumps(obj));return p

    def test_missing_depth_and_background_depth_cannot_be_finger_points(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as folder:
            o=json.loads(OBS.read_text());shape=np.load(o['cameras']['right_hand']['depth_m_path']).shape
            path=Path(folder)/'invalid.npy';np.save(path,np.full(shape,65.535))
            obs=self.altered_observation(folder,lambda o:o['cameras']['right_hand'].update(depth_m_path=str(path)))
            with self.assertRaisesRegex(ValueError,'red cube depth'):
                geometry.estimate_scene(obs,Q,POSE)
        with tempfile.TemporaryDirectory() as folder:
            o=json.loads(OBS.read_text());d=np.load(o['cameras']['right_hand']['depth_m_path']).copy()
            d[int(.8*d.shape[0]):,:]=.6
            path=Path(folder)/'no_fingers.npy';np.save(path,d)
            obs=self.altered_observation(folder,lambda o:o['cameras']['right_hand'].update(depth_m_path=str(path)))
            with self.assertRaisesRegex(ValueError,'two separate visible fingers'):
                geometry.estimate_scene(obs,Q,POSE)

    def test_wrong_robot_pose_and_unaligned_depth_reject(self):
        wrong=list(POSE);wrong[0]+=2000
        with self.assertRaisesRegex(ValueError,'FK and provided pose'):
            geometry.estimate_scene(OBS,Q,wrong)
        with tempfile.TemporaryDirectory() as folder:
            obs=self.altered_observation(folder,lambda o:o['cameras']['right_hand'].update(depth_aligned_to='depth'))
            with self.assertRaisesRegex(ValueError,'aligned to color'):
                geometry.estimate_scene(obs,Q,POSE)

    @unittest.skipUnless((OPEN_RUN/'camera_recording/00_initial/observation.json').exists(),
                         'Recorded 55 mm opening scene unavailable')
    def test_real_55mm_open_right_tip_background_is_rejected(self):
        import numpy as np
        report=json.loads((OPEN_RUN/'report.json').read_text())
        state=report['initial_scene']['robot_before_capture']
        obs=OPEN_RUN/'camera_recording/00_initial/observation.json'
        frame=json.loads(obs.read_text())['cameras']['right_hand']
        depth=np.load(frame['depth_m_path'])*1000
        # This independently inspected actual RGB tip ROI contains tabletop depth,
        # despite valid near-depth finger pixels farther down its body.
        right_tip=depth[405:420,535:553]
        valid=right_tip[np.isfinite(right_tip)]
        self.assertGreater(np.median(valid),350)
        self.assertEqual(int(((valid>120)&(valid<300)).sum()),0)
        with self.assertRaisesRegex(ValueError,'background depth is not a finger') as caught:
            geometry.estimate_scene(obs,state['joints_raw'],state['pose_raw'])
        self.assertIn('"near_tip_depth_pixels": 0',str(caught.exception))

    def test_shifted_finger_components_detect_without_fixed_pixels(self):
        import cv2
        import numpy as np
        o=json.loads(OBS.read_text());c=o['cameras']['right_hand'];rgb=cv2.imread(c['rgb_path']);z=np.load(c['depth_m_path'])*1000
        v,u=np.indices(z.shape);k=c['intrinsics']
        xyz=np.stack(((u-k['ppx'])*z/k['fx'],(v-k['ppy'])*z/k['fy'],z),-1)
        pair,_=geometry._fingers(rgb,xyz,z,cv2,np)
        self.assertEqual(len(pair),2)
        # Shift entire lower image cyclically 30 pixels; both fingers remain visible.
        rgb2=rgb.copy();z2=z.copy();start=int(.8*z.shape[0])
        rgb2[start:]=np.roll(rgb[start:],30,axis=1);z2[start:]=np.roll(z[start:],30,axis=1)
        xyz2=np.stack(((u-k['ppx'])*z2/k['fx'],(v-k['ppy'])*z2/k['fy'],z2),-1)
        shifted,_=geometry._fingers(rgb2,xyz2,z2,cv2,np)
        self.assertEqual(len(shifted),2)
        for a,b in zip(pair,shifted):self.assertEqual(b['bbox'][0]-a['bbox'][0],30)


if __name__=='__main__':unittest.main()
