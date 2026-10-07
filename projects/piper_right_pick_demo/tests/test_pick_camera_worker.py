"""Exercise the finite recording child with fake devices, never real cameras."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('pick_camera_test', ROOT/'scripts/pick_camera_worker.py')
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)


class CameraWorkerTests(unittest.TestCase):
    def test_two_serials_record_and_capture_without_left_or_robot(self):
        import numpy as np
        bindings, pipes, writers = [], [], []
        intr = types.SimpleNamespace(width=640, height=480, fx=600., fy=600., ppx=320., ppy=240.,
                                     model='none', coeffs=[0.]*5)
        profile = types.SimpleNamespace(as_video_stream_profile=lambda: types.SimpleNamespace(intrinsics=intr))
        class Frame:
            def __init__(self, depth=False): self.depth = depth; self.profile = profile
            def get_data(self):
                if self.depth:
                    array = np.full((480,640), 450, np.uint16); array[0,0]=65535; return array
                return np.zeros((480,640,3),np.uint8)
            def get_timestamp(self): return 1.
            def get_frame_number(self): return 1
        frame_set = types.SimpleNamespace(get_color_frame=lambda: Frame(), get_depth_frame=lambda: Frame(True))
        class Config:
            def enable_device(self, serial): self.serial=serial; bindings.append(serial)
            def enable_stream(self, *a): pass
        class Pipeline:
            def __init__(self): self.stopped=False; pipes.append(self)
            def start(self,cfg):
                device = types.SimpleNamespace(get_info=lambda key:cfg.serial,
                    first_depth_sensor=lambda:types.SimpleNamespace(get_depth_scale=lambda:.001))
                return types.SimpleNamespace(get_device=lambda:device)
            def wait_for_frames(self, timeout): return frame_set
            def stop(self): self.stopped=True
        class Writer:
            def __init__(self,*a): self.count=0;self.closed=False;writers.append(self)
            def isOpened(self): return True
            def write(self,array):self.count+=1
            def release(self):self.closed=True
        rs=types.SimpleNamespace(pipeline=Pipeline,config=Config,
            stream=types.SimpleNamespace(color=1,depth=2),format=types.SimpleNamespace(bgr8=1,z16=2),
            camera_info=types.SimpleNamespace(serial_number=1),align=lambda _:types.SimpleNamespace(process=lambda f:f))
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict('sys.modules',{'pyrealsense2':rs}), \
                mock.patch('cv2.VideoWriter',Writer), \
                mock.patch.object(worker.sys,'stdin',io.StringIO('{"op":"capture","label":"initial"}\n{"op":"stop"}\n')), \
                mock.patch.object(worker,'emit') as output:
            target=Path(tmp)/'camera'
            self.assertEqual(worker.main(['--config',str(ROOT/'configs/site.local.json'),'--output-dir',str(target)]),0)
            report=json.loads((target/'report.json').read_text())
            obs=json.loads(Path(report['snapshots'][0]).read_text())
            self.assertEqual(bindings,['243622070374','244222070415'])
            self.assertEqual(set(obs['cameras']),{'front','right_hand'})
            depth=np.load(obs['cameras']['right_hand']['depth_m_path'])
            self.assertTrue(np.isnan(depth[0,0]))
            self.assertAlmostEqual(float(depth[1,1]),.45,places=5)
            self.assertEqual(report['robot_commands_sent'],0)
            self.assertTrue(all(p.stopped for p in pipes))
            self.assertTrue(all(w.count>0 and w.closed for w in writers))
            self.assertIn('captured',[c.args[0]['event'] for c in output.call_args_list])


if __name__ == '__main__':
    unittest.main()
