"""Offline adapter contracts; fake devices only, never import a real SDK."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from robot_tools.cameras import (
    CameraCaptureError, DEFAULT_CAMERA_SERIALS, _fresh_batch, capture_cameras,
)


class FakeFrame:
    def __init__(self, number, depth=False, depth_data=None):
        self.number, self.depth = number, depth
        self.depth_data = depth_data
        self.profile = NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: NS(
            width=640, height=480, fx=600.0, fy=600.0, ppx=320.0, ppy=240.0,
            model="none", coeffs=[0.0] * 5)))

    def get_frame_number(self):
        return self.number

    def get_timestamp(self):
        return self.number * 66.667

    def get_frame_timestamp_domain(self):
        return "hardware_clock"

    def get_data(self):
        import numpy as np
        if self.depth and self.depth_data is not None:
            return self.depth_data
        return np.array([[0, 1000]], dtype=np.uint16) if self.depth else np.zeros(
            (480, 640, 3), dtype=np.uint8)


class FakePipeline:
    def __init__(self, owner):
        self.owner, self.stopped, self.number, self.serial = owner, False, 0, None

    def start(self, config):
        self.serial = config.serial
        if self.serial == self.owner.start_failure:
            raise RuntimeError("start failed")
        actual = "wrong" if self.owner.mismatch else self.serial
        return NS(get_device=lambda: self.owner.device(actual))

    def stop(self):
        self.stopped = True
        if self.owner.stop_failure:
            raise RuntimeError("stop failed")

    def poll_for_frames(self):
        return None

    def wait_for_frames(self, timeout):
        self.number += 1
        return NS(get_color_frame=lambda: FakeFrame(self.number),
                  get_depth_frame=lambda: FakeFrame(self.number, depth=True,
                                                   depth_data=self.owner.depth_data.get(self.serial)))


class FakeConfig:
    def enable_device(self, serial):
        self.serial = serial

    def enable_stream(self, *args):
        pass


class FakeRS:
    def __init__(self):
        self.available = list(DEFAULT_CAMERA_SERIALS.values())
        self.pipelines = []
        self.depth_data = {}
        self.start_failure, self.stop_failure, self.mismatch = None, False, False
        self.camera_info, self.stream = NS(serial_number="serial"), NS(color=1, depth=2)
        self.format = NS(bgr8=1, z16=2)
        self.config = FakeConfig

    def device(self, serial):
        return NS(get_info=lambda key: serial,
                  first_depth_sensor=lambda: NS(get_depth_scale=lambda: 0.001))

    def context(self):
        return NS(query_devices=lambda: [self.device(s) for s in self.available])

    def pipeline(self):
        pipeline = FakePipeline(self)
        self.pipelines.append(pipeline)
        return pipeline

    def align(self, stream):
        return NS(process=lambda frames: frames)


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / "new_capture"
        self.rs = FakeRS()
        self.cv = NS(imwrite=self.write_image)
        modules = patch.dict(sys.modules, {"pyrealsense2": self.rs, "cv2": self.cv})
        modules.start()
        self.addCleanup(modules.stop)

    @staticmethod
    def write_image(path, image):
        Path(path).write_bytes(b"fake-png-for-contract-test")
        return True

    def capture(self, depth=False):
        return capture_cameras(self.output, DEFAULT_CAMERA_SERIALS, depth)

    def test_missing_device_starts_nothing(self):
        self.rs.available.pop()
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        self.assertEqual(self.rs.pipelines, [])
        self.assertFalse(caught.exception.report["complete"])
        self.assertEqual(caught.exception.report["cameras"], {})

    def test_serial_preflight_exception_starts_nothing(self):
        self.rs.context = lambda: NS(query_devices=lambda: [
            NS(get_info=lambda key: (_ for _ in ()).throw(RuntimeError("USB disconnected")))])
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        self.assertEqual(self.rs.pipelines, [])
        self.assertEqual(caught.exception.report["phase"], "serial_preflight")
        self.assertFalse(caught.exception.report["complete"])

    def test_duplicate_serial_rejected_before_directory_creation(self):
        bindings = dict.fromkeys(DEFAULT_CAMERA_SERIALS, "123")
        with self.assertRaises(ValueError):
            capture_cameras(self.output, bindings)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.rs.pipelines, [])

    def test_existing_directory_never_overwritten(self):
        self.output.mkdir()
        with self.assertRaises(FileExistsError):
            self.capture()
        self.assertEqual(self.rs.pipelines, [])

    def test_complete_rgb_capture_is_explicitly_unsynchronized(self):
        report = self.capture()
        self.assertTrue(report["complete"])
        self.assertFalse(report["hardware_synchronized"])
        self.assertIsNone(report["exposure_skew_s"])
        self.assertIsNone(report["exposure_age_s"])
        self.assertEqual(set(report["cameras"]), set(DEFAULT_CAMERA_SERIALS))
        self.assertTrue(all(p.stopped for p in self.rs.pipelines))
        for name, item in report["cameras"].items():
            self.assertEqual(item["serial"], DEFAULT_CAMERA_SERIALS[name])
            self.assertTrue(Path(item["rgb_path"]).exists())
            self.assertIsNone(item["depth_path"])
            self.assertGreater(item["color"]["frame_number"], 5)
            self.assertGreaterEqual(item["host_receive_age_at_report_s"], 0)
        self.assertTrue(json.loads((self.output / "observation.json").read_text())["complete"])

    def test_depth_is_float32_metres_and_invalid_nan(self):
        import numpy as np
        report = self.capture(depth=True)
        depth = np.load(report["cameras"]["front"]["depth_path"], allow_pickle=False)
        self.assertEqual(depth.dtype, np.dtype("float32"))
        self.assertTrue(np.isnan(depth[0, 0]))
        self.assertAlmostEqual(float(depth[0, 1]), 1.0)
        self.assertEqual(report["cameras"]["front"]["depth_aligned_to"], "color")

    def test_zero_and_encoding_limit_are_nan_with_separate_counts(self):
        self.rs.depth_data[DEFAULT_CAMERA_SERIALS["front"]] = np.array(
            [[0, 65535, 1000, 2000]], dtype=np.uint16)
        report = self.capture(depth=True)
        item = report["cameras"]["front"]
        depth = np.load(item["depth_path"], allow_pickle=False)
        np.testing.assert_array_equal(np.isnan(depth), [[True, True, False, False]])
        np.testing.assert_allclose(depth[0, 2:], [1.0, 2.0])
        quality = item["depth_quality"]
        self.assertEqual(quality["total_pixels"], 4)
        self.assertEqual(quality["valid_pixels"], 2)
        self.assertEqual(quality["zero_pixels"], 1)
        self.assertEqual(quality["at_encoding_limit_pixels"], 1)
        self.assertEqual(quality["valid_fraction"], 0.5)
        self.assertEqual(quality["status"], "partial")
        self.assertIn("at_encoding_limit", quality["quality_flags"])
        self.assertFalse(quality["unknown_depth_is_free_space"])

    def test_all_unusable_depth_keeps_every_rgb_and_depth_file(self):
        self.rs.depth_data = {serial: np.array([[0, 65535]], dtype=np.uint16)
                              for serial in DEFAULT_CAMERA_SERIALS.values()}
        report = self.capture(depth=True)
        self.assertTrue(report["complete"])
        self.assertEqual(report["depth_quality_status"], "unavailable")
        self.assertEqual(len(report["cameras"]), 3)
        for item in report["cameras"].values():
            self.assertTrue(Path(item["rgb_path"]).is_file())
            self.assertTrue(np.isnan(np.load(item["depth_path"], allow_pickle=False)).all())
            self.assertEqual(item["depth_quality"]["status"], "unavailable")
            self.assertEqual(item["depth_quality"]["valid_fraction"], 0.0)
            self.assertIn("no_usable_depth", item["depth_quality"]["quality_flags"])
            self.assertFalse(item["depth_quality"]["unknown_depth_is_free_space"])
        saved = json.loads((self.output / "observation.json").read_text())
        self.assertEqual(saved["depth_quality_status"], "unavailable")
        self.assertTrue(all(p.stopped for p in self.rs.pipelines))

    def test_available_depth_reports_encoding_quality_not_range_certification(self):
        self.rs.depth_data = {serial: np.array([[1, 1000, 65534]], dtype=np.uint16)
                              for serial in DEFAULT_CAMERA_SERIALS.values()}
        report = self.capture(depth=True)
        self.assertEqual(report["depth_quality_status"], "available")
        quality = report["cameras"]["front"]["depth_quality"]
        self.assertEqual(quality["valid_fraction"], 1.0)
        self.assertEqual(quality["quality_flags"], [])
        self.assertIn("range and accuracy unverified", quality["validity_scope"])

    def test_second_start_failure_cleans_first_pipeline(self):
        self.rs.start_failure = DEFAULT_CAMERA_SERIALS["left_wrist"]
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        self.assertTrue(self.rs.pipelines[0].stopped)
        self.assertTrue(self.rs.pipelines[1].stopped)
        self.assertEqual(caught.exception.report["started_cameras"], ["front"])
        self.assertEqual(caught.exception.report["startup_attempted_cameras"], ["front", "left_wrist"])
        self.assertEqual(caught.exception.report["cleanup_results"]["left_wrist"], "stop_returned")

    def test_failed_start_and_failed_stop_report_unknown_resource_state(self):
        self.rs.start_failure = DEFAULT_CAMERA_SERIALS["front"]
        self.rs.stop_failure = True
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        report = caught.exception.report
        self.assertEqual(report["started_cameras"], [])
        self.assertEqual(report["cleanup_results"]["front"], "stop_failed_resource_state_unknown")
        self.assertFalse(report["complete"])

    def test_wrong_serial_cleans_started_pipeline(self):
        self.rs.mismatch = True
        with self.assertRaises(CameraCaptureError):
            self.capture()
        self.assertTrue(self.rs.pipelines[0].stopped)

    def test_image_write_failure_reports_partial_state_and_releases_all(self):
        self.cv.imwrite = lambda path, image: False
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        self.assertFalse(caught.exception.report["complete"])
        self.assertIsNone(caught.exception.report["cameras"]["front"]["rgb_path"])
        self.assertTrue(all(p.stopped for p in self.rs.pipelines))

    def test_cleanup_failure_is_not_reported_as_complete(self):
        self.rs.stop_failure = True
        with self.assertRaises(CameraCaptureError) as caught:
            self.capture()
        self.assertFalse(caught.exception.report["complete"])
        self.assertEqual(len(caught.exception.report["cleanup_errors"]), 3)

    def test_report_write_failure_is_not_success_and_still_cleans_devices(self):
        with patch("robot_tools.cameras.Path.write_text", side_effect=OSError("disk full")):
            with self.assertRaises(CameraCaptureError) as caught:
                self.capture()
        self.assertFalse(caught.exception.report["complete"])
        self.assertIn("disk full", caught.exception.report["report_write_error"])
        self.assertTrue(all(p.stopped for p in self.rs.pipelines))

    def test_excessive_receive_skew_has_bounded_retries(self):
        pipelines = {name: self.rs.pipeline() for name in DEFAULT_CAMERA_SERIALS}
        with patch("robot_tools.cameras.time.monotonic", side_effect=[0, 1, 2] * 3):
            with self.assertRaisesRegex(RuntimeError, "three attempts"):
                _fresh_batch(pipelines)
        self.assertEqual([p.number for p in pipelines.values()], [3, 3, 3])

    def test_receive_skew_retry_takes_new_frames(self):
        pipelines = {name: self.rs.pipeline() for name in DEFAULT_CAMERA_SERIALS}
        with patch("robot_tools.cameras.time.monotonic", side_effect=[0, 1, 2, 3, 3.01, 3.02]):
            batch, attempts = _fresh_batch(pipelines)
        self.assertEqual(attempts, 2)
        self.assertEqual([v[0].get_color_frame().get_frame_number() for v in batch.values()], [2] * 3)


if __name__ == "__main__":
    unittest.main()
