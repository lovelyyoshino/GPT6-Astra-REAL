"""ROS metadata and snapshot tests using messages only; no device or sockets."""
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

from right_pick.camera import RealSenseRig, depth_pixel_query
from right_pick.ros_camera import (RosCameraRig, camera_info_metadata,
                                    configured_streams, snapshot_is_fresh)


def message(stamp=100.0, frame_id="color_optical"):
    return NS(width=10, height=8,
              header=NS(stamp=NS(to_sec=lambda: stamp), frame_id=frame_id),
              array=np.zeros((8, 10, 3), dtype=np.uint8))


def info(stamp=100.0, frame_id="color_optical"):
    result = message(stamp, frame_id)
    result.K = [100, 0, 5, 0, 110, 4, 0, 0, 1]
    result.D = [0, 0, 0, 0, 0]
    result.distortion_model = "plumb_bob"
    return result


class RosCameraTests(unittest.TestCase):
    def test_depth_flag_controls_subscriptions(self):
        config = dict(rgb_topic="rgb", info_topic="info", aligned_depth_topic="depth", depth_enabled=False)
        self.assertEqual(configured_streams(config), [("rgb", "rgb"), ("info", "info")])
        config["depth_enabled"] = True
        self.assertIn(("depth", "depth"), configured_streams(config))

    def test_normalized_intrinsics_keep_ros_distortion_namespace(self):
        calibration = info()
        calibration.D[0] = 0.1
        intrinsics = camera_info_metadata(calibration, message())
        self.assertEqual([intrinsics[k] for k in ("fx", "fy", "ppx", "ppy")], [100, 110, 5, 4])
        self.assertEqual(intrinsics["timestamp"], 100)
        self.assertEqual(intrinsics["model"], "plumb_bob")
        point = depth_pixel_query(np.ones((8, 10)), 1.0, (4, 4), intrinsics)
        self.assertIsNone(point["camera_point_m"])
        self.assertEqual(point["deprojection_status"], "unsupported_distortion")

    def test_bad_intrinsics_and_frame_pairs_rejected(self):
        for field, value in (("width", 9), ("K", [0] * 9), ("D", [float("nan")])):
            calibration = info()
            setattr(calibration, field, value)
            with self.assertRaises(ValueError):
                camera_info_metadata(calibration, message())
        with self.assertRaises(ValueError):
            camera_info_metadata(info(frame_id="other_camera"), message())
        result = camera_info_metadata(info(frame_id=""), message())
        self.assertEqual(result["frame_id_match"], "unknown_empty_frame_id")
        self.assertIsNone(result["frame_id"])

    def test_freshness_requires_exposure_and_real_receipt(self):
        snapshot = {("front", "rgb"): (message(100), 100.05), ("front", "info"): (info(), 100.04)}
        self.assertTrue(snapshot_is_fresh(snapshot, ["front"], 100.1, 0.8, 0.15))
        snapshot[("front", "rgb")] = (message(99), 100.05)
        self.assertFalse(snapshot_is_fresh(snapshot, ["front"], 100.1, 0.8, 0.15))
        snapshot[("front", "rgb")] = (message(100), 99)
        self.assertFalse(snapshot_is_fresh(snapshot, ["front"], 100.1, 0.8, 0.15))

    def test_capture_preserves_timestamps_and_excludes_disabled_depth(self):
        rig = RosCameraRig({"front": dict(serial="a", rgb_topic="rgb", info_topic="info", depth_enabled=False)})
        rig.subscribers = [Mock()]
        rig.frames = {("front", "rgb"): (message(), 100.05),
                      ("front", "info"): (info(), 100.04),
                      ("front", "depth"): (object(), 100.05)}
        bridge = NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda msg, desired_encoding: msg.array))
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(sys.modules, {"cv_bridge": bridge}), patch("right_pick.ros_camera.time.time", return_value=100.1):
                observation = rig.capture(folder)
        self.assertEqual(observation["captured_at"], 100)
        self.assertEqual(observation["capture_completed_at"], 100.1)
        frame = observation["cameras"]["front"]
        self.assertEqual(frame["host_received_at"], 100.05)
        self.assertEqual(frame["timestamp_kind"], "ros_header_stamp_wall_clock")
        self.assertNotIn("depth_m_path", frame)
        rig.close()

    def test_realsense_timestamp_is_earliest_receipt_not_save_time(self):
        rig = RealSenseRig({"front": {"serial": "a"}}, depth_enabled=False)
        intrinsics = NS(width=10, height=8, fx=100, fy=100, ppx=5, ppy=4, model="none", coeffs=[0] * 5)
        color = Mock()
        color.get_data.return_value = np.zeros((8, 10, 3), dtype=np.uint8)
        color.profile.as_video_stream_profile.return_value.intrinsics = intrinsics
        color.get_timestamp.return_value = 500
        color.get_frame_timestamp_domain.return_value = "hardware_clock"
        color.get_frame_number.return_value = 10
        frames = Mock()
        frames.get_color_frame.return_value = color
        pipeline = Mock()
        pipeline.wait_for_frames.return_value = frames
        rig._streams = {"front": {"pipeline": pipeline, "align": None, "device_model": "mock_device"}}
        with tempfile.TemporaryDirectory() as folder:
            with patch("right_pick.camera.time.time", side_effect=[100.05, 100.1]):
                observation = rig.capture(folder)
        self.assertEqual(observation["captured_at"], 100.05)
        self.assertEqual(observation["capture_completed_at"], 100.1)
        self.assertEqual(observation["cameras"]["front"]["timestamp_kind"], "host_receipt_wall_clock")
        rig.close()


if __name__ == "__main__":
    unittest.main()
