import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from pick_resume_scene import verify_resume_scene
SOURCE = ROOT / 'runs/pick_attempt_20261003T194811_102398/report.json'


@unittest.skipUnless(SOURCE.exists(), 'Saved real partial run is unavailable')
class ResumeSceneTests(unittest.TestCase):
    def setUp(self):
        import cv2
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        report = json.loads(SOURCE.read_text())
        self.obs = json.loads(Path(report['initial_scene']['observation']).read_text())
        self.images = {}
        for name in ('front', 'right_hand'):
            cap = cv2.VideoCapture(str(SOURCE.parent / 'camera_recording' / (name + '.avi')))
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1)
            ok, frame = cap.read()
            cap.release()
            self.assertTrue(ok)
            self.images[name] = frame
            path = Path(self.tmp.name) / (name + '.png')
            cv2.imwrite(str(path), frame)
            self.obs['cameras'][name]['rgb_path'] = str(path)
        self.path = Path(self.tmp.name) / 'observation.json'

    def check(self):
        self.path.write_text(json.dumps(self.obs))
        return verify_resume_scene(SOURCE, self.path)

    def test_same_held_scene_and_red_cube_pass(self):
        result = self.check()
        self.assertTrue(result['passed'])
        self.assertLess(result['cameras']['right_hand']['cube_displacement_px'], .01)

    def test_shifted_cube_rejects_even_when_robot_state_is_unchanged(self):
        import cv2
        image = self.images['right_hand'].copy()
        # The recorded cube occupies this patch; moving only this patch changes
        # the grasp target while leaving the camera and most table texture held.
        patch = image[230:355, 385:520].copy()
        image[230:355, 385:520] = image[95:220, 385:520]
        image[230:355, 450:585] = patch
        cv2.imwrite(self.obs['cameras']['right_hand']['rgb_path'], image)
        with self.assertRaisesRegex(ValueError, 'Red cube moved|Camera/scene changed'):
            self.check()

    def test_changed_camera_rejects(self):
        self.obs['cameras']['right_hand']['serial'] = 'wrong-camera'
        with self.assertRaisesRegex(ValueError, 'serial changed'):
            self.check()


if __name__ == '__main__':
    unittest.main()
