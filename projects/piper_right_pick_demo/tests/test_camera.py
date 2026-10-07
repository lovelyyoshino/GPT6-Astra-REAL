"""Offline camera contracts; no device access or SDK required."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from right_pick.camera import (RealSenseRig, depth_pixel_query, depth_to_meters,
                               detect_red_candidates, detect_red_candidates_file)


class CameraTests(unittest.TestCase):
    def test_sensor_scale_and_invalid_depth(self):
        raw = np.array([[0, 250, 2000]], dtype=np.uint16)
        converted = depth_to_meters(raw, 0.0005)
        self.assertTrue(np.isnan(converted[0, 0]))
        self.assertAlmostEqual(float(converted[0, 1]), 0.125)
        self.assertAlmostEqual(float(converted[0, 2]), 1.0)
        self.assertEqual(raw[0, 1], 250)
        for invalid_scale in (0.0, -1.0, float("nan")):
            with self.assertRaises(ValueError):
                depth_to_meters(raw, invalid_scale)

    def test_surface_query_is_not_grasp_target(self):
        raw = np.array([[1000, 1000], [1000, 1000]], dtype=np.uint16)
        result = depth_pixel_query(raw, 0.0005, (1, 1), {
            "fx": 100.0, "fy": 200.0, "ppx": 0.0, "ppy": 0.0, "model": "none",
        })
        self.assertEqual(result["camera_point_m"], [0.005, 0.0025, 0.5])
        self.assertFalse(result["is_grasp_target"])
        self.assertFalse(result["extrinsics_applied"])

    def test_invalid_pixel_not_replaced_by_neighbour(self):
        raw = np.array([[1000, 0]], dtype=np.uint16)
        result = depth_pixel_query(raw, 0.001, (1, 0))
        self.assertFalse(result["valid"])
        self.assertIsNone(result["depth_z_m"])
        with self.assertRaises(ValueError):
            depth_pixel_query(raw, 0.001, (2, 0))
        with self.assertRaises(ValueError):
            depth_pixel_query(raw, 0.001, (0.3, 0))

    def test_nonzero_distortion_not_silently_ignored(self):
        result = depth_pixel_query(np.ones((1, 1)), 0.1, (0, 0), {
            "fx": 100, "fy": 100, "ppx": 0, "ppy": 0,
            "model": "brown_conrady", "coeffs": [0.1, 0, 0, 0, 0],
        })
        self.assertTrue(result["valid"])
        self.assertIsNone(result["camera_point_m"])
        self.assertIn("deprojection required", result["reason"])

    def test_red_hue_wrap_and_other_colors(self):
        import cv2
        hsv = np.zeros((80, 100, 3), dtype=np.uint8)
        hsv[10:20, 20:35] = (2, 255, 255)
        hsv[40:60, 60:80] = (178, 255, 255)
        hsv[1:6, 1:6] = (60, 255, 255)
        hsv[70:72, 1:3] = (0, 255, 255)  # noise below minimum area
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        candidates = detect_red_candidates(bgr)
        self.assertEqual([c["area_px"] for c in candidates], [400, 150])
        self.assertEqual(candidates[0]["bbox_xywh"], [60, 40, 20, 20])
        self.assertEqual(candidates[0]["center_uv"], [69.5, 49.5])
        self.assertFalse(candidates[0]["object_identity_verified"])
        self.assertIsNone(candidates[0]["grasp_success"])
        self.assertEqual(candidates, detect_red_candidates(bgr[:, :, ::-1], "rgb"))

    def test_file_detection_is_offline(self):
        import cv2
        image = np.zeros((30, 30, 3), dtype=np.uint8)
        image[5:15, 6:16] = (0, 0, 255)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "red.png"
            self.assertTrue(cv2.imwrite(str(path), image))
            self.assertEqual(detect_red_candidates_file(path)[0]["area_px"], 100)

    def test_constructor_lazy_and_binding_validation(self):
        with patch("right_pick.camera.importlib.import_module", side_effect=AssertionError("SDK import")):
            rig = RealSenseRig({"front": {"serial": "serial-1"}})
            rig.close()
        with self.assertRaises(ValueError):
            RealSenseRig({"front": {"serial": "a"}, "wrist": {"serial": "a"}})
        with self.assertRaises(ValueError):
            RealSenseRig({"../bad": {"serial": "a"}})

    def test_close_releases_all_resources_even_after_one_failure(self):
        rig = RealSenseRig({"front": {"serial": "a"}})
        writer = Mock()
        writer.release.side_effect = RuntimeError("writer release failed")
        pipeline = Mock()
        rig._writers["front"] = writer
        rig._streams["front"] = {"pipeline": pipeline}
        rig.close()
        pipeline.stop.assert_called_once_with()
        self.assertEqual(rig.close_errors[0]["operation"], "video_release")

    def test_sampled_recording_enable_does_not_open_hardware(self):
        rig = RealSenseRig({"front": {"serial": "a"}})
        with tempfile.TemporaryDirectory() as folder:
            with patch("right_pick.camera.importlib.import_module", side_effect=AssertionError("SDK import")):
                rig.start_recording(Path(folder) / "video")
                result = rig.stop_recording()
                self.assertEqual(result["frames"], {})
                self.assertIn("not_continuous", result["recording_kind"])
            rig.close()


if __name__ == "__main__":
    unittest.main()
