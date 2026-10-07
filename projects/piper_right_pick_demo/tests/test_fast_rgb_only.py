"""RGB-only camera mode must not even run the old color detector."""
import tempfile
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from right_pick.camera import RealSenseRig
from right_pick.fast_observation import PersistentRGBCameras
from right_pick.ros_camera import RosCameraRig, configured_streams


class RGBOnlyTests(unittest.TestCase):
    def test_no_depth_or_detection_and_persistent_rig(self):
        config = {"front": {"serial": "test", "depth_enabled": True}}
        rig = RealSenseRig(config, rgb_only=True)
        self.assertFalse(rig.configs["front"]["depth_enabled"])
        color = Mock()
        color.get_data.return_value = np.zeros((8, 10, 3), dtype=np.uint8)
        color.get_timestamp.return_value = 500
        color.get_frame_timestamp_domain.return_value = "hardware_clock"
        color.get_frame_number.return_value = 10
        frames = Mock()
        frames.get_color_frame.return_value = color
        pipeline = Mock()
        pipeline.wait_for_frames.return_value = frames
        rig._streams = {"front": {"pipeline": pipeline, "align": None, "device_model": "mock"}}
        with tempfile.TemporaryDirectory() as root, patch("right_pick.camera.detect_red_candidates", side_effect=AssertionError("detector called")):
            first, second = rig.capture(root), rig.capture(root)
        for obs in (first, second):
            frame = obs["cameras"]["front"]
            self.assertNotIn("intrinsics", frame)
            self.assertNotIn("red_candidates", frame)
            self.assertFalse(frame["depth_enabled"])
        self.assertEqual(pipeline.wait_for_frames.call_count, 2)
        frames.get_depth_frame.assert_not_called()
        pipeline.start.assert_not_called()

    def test_all_camera_roles_required(self):
        with self.assertRaises(ValueError):
            PersistentRGBCameras({"backend": "realsense", "cameras": {"front": {"serial": "a"}}}, "/tmp/not_opened")

    def test_ros_rgb_needs_no_camera_info_or_calibration(self):
        config = {"front": {"rgb_topic": "/rgb", "depth_enabled": True}}
        self.assertEqual(configured_streams(config["front"], rgb_only=True), [("rgb", "/rgb")])
        rig = RosCameraRig(config, rgb_only=True)
        rig.subscribers = [Mock()]
        msg = NS(width=10, height=8, header=NS(frame_id="rgb", stamp=NS(to_sec=lambda: 100.0)),
                 array=np.zeros((8, 10, 3), dtype=np.uint8))
        rig.frames = {("front", "rgb"): (msg, 100.05)}
        bridge = NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda message, desired_encoding: message.array))
        with tempfile.TemporaryDirectory() as root, patch.dict(sys.modules, {"cv_bridge": bridge}), \
                patch("right_pick.ros_camera.time.time", return_value=100.1), \
                patch("right_pick.ros_camera.camera_info_metadata", side_effect=AssertionError("intrinsics called")), \
                patch("right_pick.camera.detect_red_candidates", side_effect=AssertionError("detector called")):
            obs = rig.capture(root)
        self.assertNotIn("intrinsics", obs["cameras"]["front"])
        self.assertFalse(obs["cameras"]["front"]["depth_enabled"])


if __name__ == "__main__":
    unittest.main()
