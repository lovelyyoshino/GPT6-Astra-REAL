"""Offline checks of time pairing and motion rejection; no hardware opened."""
import copy
import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    "alignment_capture", Path(__file__).resolve().parents[1] / "scripts/capture_alignment_pose.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def trace():
    result = []
    for index in range(39):
        received = 100.0 + (index - 19) * .02
        joints = {"joint_%d" % n: 0 for n in range(1, 7)}
        end = dict(X_axis=56000, Y_axis=0, Z_axis=213000,
                   RX_axis=0, RY_axis=85000, RZ_axis=0)
        result.append({"received_monotonic_s": received, "joints_raw": joints,
                       "end_pose_raw": end,
                       "field_received_monotonic_s": {k: received - .003 for k in list(joints) + list(end)}})
    return result


class AlignmentCaptureTests(unittest.TestCase):
    def test_stationary_evidence_is_never_motion_authority(self):
        value = module.assess_trace(trace(), 100.0)
        self.assertTrue(value["stationary_window_candidate"])
        self.assertFalse(value["calibration_accepted"])
        self.assertFalse(value["motion_authorized"])
        self.assertEqual(value["translation_span_m"], 0)

    def test_image_must_be_bracketed_not_just_near_one_pose(self):
        self.assertFalse(module.assess_trace(trace(), 100.3)["stationary_window_candidate"])
        self.assertFalse(module.assess_trace(trace()[:10], 100.0)["stationary_window_candidate"])

    def test_moving_arm_rejected(self):
        value = trace()
        for i, sample in enumerate(value):
            sample["end_pose_raw"]["X_axis"] += i * 1000
        self.assertIn("moved", module.assess_trace(value, 100.0)["reason"])

    def test_stale_split_feedback_rejected_even_with_recent_sample(self):
        value = trace()
        value[19]["field_received_monotonic_s"]["RY_axis"] -= .5
        self.assertIn("stale", module.assess_trace(value, 100.0)["reason"])

    def test_missing_and_nonfinite_data_rejected(self):
        value = trace()
        del value[19]["joints_raw"]["joint_6"]
        self.assertFalse(module.assess_trace(value, 100.0)["stationary_window_candidate"])
        value = trace()
        value[19]["end_pose_raw"]["Y_axis"] = float("nan")
        self.assertFalse(module.assess_trace(value, 100.0)["stationary_window_candidate"])

    def test_rotation_change_and_trace_gaps_rejected(self):
        value = trace()
        value[19]["end_pose_raw"]["RZ_axis"] = 5000
        self.assertIn("moved", module.assess_trace(value, 100.0)["reason"])
        value = trace()
        del value[10:20]
        self.assertIn("Gap", module.assess_trace(value, 100.0)["reason"])

    def test_wrap_at_180_does_not_falsely_imply_full_rotation(self):
        value = trace()
        for i, sample in enumerate(value):
            sample["end_pose_raw"]["RZ_axis"] = 179900 if i % 2 else -179900
        before = copy.deepcopy(value)
        result = module.assess_trace(value, 100.0)
        self.assertTrue(result["stationary_window_candidate"])
        self.assertAlmostEqual(result["largest_euler_component_span_deg"], .2)
        self.assertEqual(value, before)


if __name__ == "__main__":
    unittest.main()
