"""Offline diagnostics: independent SDK FK reference, no robot/socket creation."""
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import arms, model_compatibility as mc


SDK_PATH = "/home/agilex/pyAgxArm"
ROOT = Path(__file__).resolve().parents[3]
COMMIT = "841a625f5f4920e776f20b934eb13048b747e6d0"
CONSTANTS = Path(SDK_PATH) / "pyAgxArm/api/constants.py"


class ModelCompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch("socket.socket", side_effect=AssertionError("Hardware socket forbidden")):
            cls.sdk = arms._load_sdk(SDK_PATH)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.addCleanup(patch.stopall)
        patch("socket.socket", side_effect=AssertionError("Socket forbidden")).start()
        patch.object(self.sdk.AgxArmFactory, "create_arm",
                     side_effect=AssertionError("Robot construction forbidden")).start()
        self.catalog = {"constants_path": str(CONSTANTS), "sha256": hashlib.sha256(CONSTANTS.read_bytes()).hexdigest(),
                        "commit": COMMIT, "source_url": "https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/" +
                        COMMIT + "/pyAgxArm/api/constants.py"}
        self.binding = {"channel": "can0", "usb_interface": "usb-test", "firmware_profile": "default", "firmware_version": "test-v1"}
        self.models, _ = mc.load_model_catalog(self.catalog)
        self.counter = 0

    def vendor_pose(self, model, q):
        return arms.vendor_fk(model, q, SDK_PATH)["pose_m_rad"]

    def sample(self, q=None, model="piper_x", pose=None, binding=None):
        q = list(q or [0., .5, -.5, .1, .2, .1])
        binding = copy.deepcopy(binding or self.binding)
        pose = self.vendor_pose(model, q) if pose is None else pose
        self.counter += 1
        stamp = 1000. + self.counter
        raw = {**binding, "configured_model": "piper", "joints_rad": q, "pose_m_rad": pose,
               "timestamp": stamp, "fragment_timestamps_s": dict.fromkeys(mc.PARTS, stamp - .01)}
        path = self.path / ("source_%d.json" % self.counter)
        path.write_text(json.dumps({"state": {"arms": {"left": raw}}}))
        return {"sample_id": "sample_%d" % self.counter, "binding": binding,
                "units": dict(mc.UNITS), "joint_order": list(mc.JOINT_ORDER),
                "rpy_convention": mc.RPY_CONVENTION, "joint_source": "controller_feedback",
                "pose_source": "controller_feedback", "pose_frame": "arm_base", "pose_reference": "flange",
                "joints_rad": q, "pose_m_rad": pose,
                "source": {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                           "json_pointer": "/state/arms/left"}}

    def document(self, samples, selected="piper_x"):
        return {"schema": mc.SCHEMA, "model_catalog": self.catalog,
                "arms": {"left": {"selected_model": selected, "declared_physical_model": "piper_x",
                "physical_model_source": "Test declaration, not hardware identification",
                "binding": copy.deepcopy(self.binding), "samples": samples}}}

    def replace_source(self, sample, change):
        path = Path(sample["source"]["path"])
        data = json.loads(path.read_text())
        change(data["state"]["arms"]["left"])
        path.write_text(json.dumps(data))
        sample["source"]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()

    def test_fk_independent_vendor_reference_all_models_and_saved_two_arms(self):
        rng = random.Random(871)
        qs = [[0.] * 6, [math.pi / 2, .7, -1.1, -.4, .3, .6]]
        qs += [[rng.uniform(-1., 1.) for _ in range(6)] for _ in range(16)]
        saved = ROOT / "artifacts/plug_transfer_diagnostic_20261007T103243_8ff158/resume_after_pair_host_update_1791345474396806512/final_read_state.json"
        states = json.loads(saved.read_text())["state"]["arms"]
        qs += [states[side]["joints_rad"] for side in ("left", "right")]
        for model in mc.MODELS:
            for q in qs:
                expected = mc.pose_matrix(self.vendor_pose(model, q))
                error = mc.matrix_error(mc.fk_matrix(self.models[model]["mdh"], q), expected)
                self.assertLess(error["position_error_m"], 1e-12)
                self.assertLess(error["so3_error_rad"], 1e-12)
        for state in states.values():
            actual = mc.pose_matrix(state["pose_m_rad"])
            self.assertLess(mc.matrix_error(mc.fk_matrix(self.models["piper"]["mdh"], state["joints_rad"]), actual)["position_error_m"], 3e-6)
            self.assertGreater(mc.matrix_error(mc.fk_matrix(self.models["piper_x"]["mdh"], state["joints_rad"]), actual)["position_error_m"], .05)

    def test_single_sample_candidate_does_not_relabel_hardware_or_confirm_transform(self):
        result = mc.diagnose(self.document([self.sample(model="piper")]))
        arm = result["arms"]["left"]
        self.assertIn("piper", [c["model"] for c in arm["controller_model_candidates"]])
        self.assertEqual(arm["declared_physical_model"], "piper_x")
        self.assertEqual(arm["next_action"], "reconcile_controller_frame_firmware_and_joint_mapping_before_motion")
        self.assertEqual(arm["candidate_model_comparisons"]["piper_x"]["fixed_base_transform_hypothesis"]["status"], "insufficient_pose_diversity")
        self.assertFalse(result["qualification_granted"])
        self.assertFalse(arm["fixed_transform_verified"])

    def test_many_identical_snapshots_do_not_create_pose_diversity(self):
        result = mc.diagnose(self.document([self.sample() for _ in range(5)]))["arms"]["left"]
        self.assertEqual(result["diversity"]["distinct_pose_count"], 1)
        self.assertEqual(result["controller_model_candidates"][0]["evidence"], "limited_pose_numerical_match")

    def test_varied_poses_matching_selected_model_remain_diagnostic(self):
        rows = [self.sample(q=[.1 * i, .5, -.5 - .15 * i, .1, .2, .1]) for i in range(5)]
        arm = mc.diagnose(self.document(rows))["arms"]["left"]
        self.assertTrue(arm["diversity"]["descriptive_diversity_sufficient"])
        self.assertEqual(arm["next_action"], "plan_separate_bounded_physical_validation_not_automatic_admission")
        self.assertFalse(arm["motion_permitted"])

    def test_fixed_base_hypothesis_is_tested_against_other_poses_never_verified(self):
        rows = []
        # Translation in the reported base; use independent vendor poses.
        for i in range(5):
            q = [.1 * i, .5, -.5 - .15 * i, .1, .2, .1]
            pose = self.vendor_pose("piper_x", q)
            pose[0] += .02
            rows.append(self.sample(q=q, pose=pose))
        arm = mc.diagnose(self.document(rows))["arms"]["left"]
        hypothesis = arm["candidate_model_comparisons"]["piper_x"]["fixed_base_transform_hypothesis"]
        self.assertEqual(hypothesis["status"], "numerically_consistent_hypothesis")
        self.assertFalse(hypothesis["transform_verified"])
        self.assertEqual(len(hypothesis["comparison_errors"]), 4)
        rows[-1]["pose_m_rad"][0] += .04
        self.replace_source(rows[-1], lambda raw: raw.update(pose_m_rad=rows[-1]["pose_m_rad"]))
        hypothesis = mc.diagnose(self.document(rows))["arms"]["left"]["candidate_model_comparisons"]["piper_x"]["fixed_base_transform_hypothesis"]
        self.assertEqual(hypothesis["status"], "inconsistent_hypothesis")

    def test_wraparound_equivalent_euler_values_and_pi_rotation(self):
        a = mc.pose_matrix([0., 0., 0., .3, .2, math.pi - .001])
        b = mc.pose_matrix([0., 0., 0., .3, .2, -math.pi - .001])
        self.assertLess(mc.matrix_error(a, b)["so3_error_rad"], 1e-12)
        self.assertAlmostEqual(mc.matrix_error(mc._identity(), mc.pose_matrix([0., 0., 0., math.pi, 0., 0.]))["so3_error_rad"], math.pi)

    def test_two_poses_can_falsify_transform_without_claiming_identifiability(self):
        rows = []
        for i in range(2):
            q = [.2 * i, .5, -.5 - .2 * i, .1, .2, .1]
            pose = self.vendor_pose("piper_x", q)
            pose[0] += .02 + .04 * i
            rows.append(self.sample(q=q, pose=pose))
        arm = mc.diagnose(self.document(rows))["arms"]["left"]
        self.assertFalse(arm["diversity"]["descriptive_diversity_sufficient"])
        model = arm["candidate_model_comparisons"]["piper_x"]
        for key in ("fixed_base_transform_hypothesis", "fixed_tool_transform_hypothesis"):
            self.assertEqual(model[key]["status"], "inconsistent_hypothesis")
            self.assertFalse(model[key]["transform_verified"])

    def test_changed_binding_cannot_be_merged_or_award_candidates(self):
        for field, value in (("channel", "can1"), ("usb_interface", "different"), ("firmware_version", "test-v2")):
            with self.subTest(field=field):
                row = self.sample(); row["binding"][field] = value
                arm = mc.diagnose(self.document([self.sample(), row]))["arms"]["left"]
                self.assertEqual(len(arm["rejected_samples"]), 1)
                self.assertEqual(arm["controller_model_candidates"], [])

    def test_both_arms_compared_separately(self):
        left = self.sample(model="piper")
        right_binding = {**self.binding, "channel": "can1", "usb_interface": "right-test"}
        right = self.sample(binding=right_binding)
        document = self.document([left])
        document["arms"]["right"] = {**copy.deepcopy(document["arms"]["left"]), "binding": right_binding, "samples": [right]}
        report = mc.diagnose(document)
        self.assertTrue(report["arms"]["left"]["candidate_model_comparisons"]["piper"]["all_accepted_samples_numerically_match"])
        self.assertTrue(report["arms"]["right"]["candidate_model_comparisons"]["piper_x"]["all_accepted_samples_numerically_match"])

    def test_unknown_firmware_retained_as_unknown(self):
        self.binding["firmware_version"] = None
        arm = mc.diagnose(self.document([self.sample()]))["arms"]["left"]
        self.assertFalse(arm["firmware_version_observed_in_samples"])

    def test_controller_match_cannot_silently_select_wrong_physical_model(self):
        arm = mc.diagnose(self.document([self.sample(model="piper")], selected="piper"))["arms"]["left"]
        self.assertEqual(arm["next_action"], "reconcile_selected_model_with_declared_physical_model_without_relabeling_hardware")

    def test_malformed_sample_or_pointer_is_rejected(self):
        arm = mc.diagnose(self.document([None]))["arms"]["left"]
        self.assertEqual(len(arm["rejected_samples"]), 1)
        row = self.sample(); row["source"]["json_pointer"] = "/state/arms/left/joints_rad/999"
        arm = mc.diagnose(self.document([row]))["arms"]["left"]
        self.assertEqual(len(arm["rejected_samples"]), 1)

    def test_reject_incorrect_units_order_convention_or_derived_pose(self):
        changes = {"units": {"joints": "deg", "xyz": "m", "rpy": "rad"},
                   "joint_order": list(reversed(mc.JOINT_ORDER)), "rpy_convention": "intrinsic XYZ",
                   "pose_source": "sdk_fk", "pose_reference": "tcp", "pose_frame": "world"}
        for key, value in changes.items():
            with self.subTest(key=key):
                row = self.sample(); row[key] = value
                arm = mc.diagnose(self.document([row]))["arms"]["left"]
                self.assertEqual(len(arm["rejected_samples"]), 1)
                self.assertEqual(arm["controller_model_candidates"], [])

    def test_nonfinite_bool_or_tampered_values_rejected(self):
        for value in (True, math.nan, math.inf, 999.):
            row = self.sample(); row["joints_rad"][0] = value
            arm = mc.diagnose(self.document([row]))["arms"]["left"]
            self.assertEqual(len(arm["rejected_samples"]), 1)

    def test_source_hash_and_source_binding_are_checked(self):
        row = self.sample(); row["source"]["sha256"] = "0" * 64
        self.assertEqual(len(mc.diagnose(self.document([row]))["arms"]["left"]["rejected_samples"]), 1)
        row = self.sample()
        self.replace_source(row, lambda raw: raw.update(channel="other"))
        self.assertEqual(len(mc.diagnose(self.document([row]))["arms"]["left"]["rejected_samples"]), 1)

    def test_stale_missing_or_future_historical_fragments_rejected(self):
        for kind in ("stale", "missing", "future"):
            row = self.sample()
            def change(raw):
                if kind == "missing": del raw["fragment_timestamps_s"]["joint_12"]
                else: raw["fragment_timestamps_s"]["joint_12"] = raw["timestamp"] + (.01 if kind == "future" else -.2)
            self.replace_source(row, change)
            self.assertEqual(len(mc.diagnose(self.document([row]))["arms"]["left"]["rejected_samples"]), 1)

    def test_catalog_hash_commit_url_and_literal_extraction(self):
        for field, value in (("sha256", "0" * 64), ("source_url", "https://example.com/constants.py")):
            spec = {**self.catalog, field: value}
            with self.assertRaises(ValueError): mc.load_model_catalog(spec)
        path = self.path / "constants.py"
        path.write_bytes(CONSTANTS.read_bytes() + b'\nraise RuntimeError("must never execute")\n')
        spec = {**self.catalog, "constants_path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        with self.assertRaises(ValueError): mc.load_model_catalog(spec)  # Cannot impersonate known official commit.
        spec.update(commit="0" * 40, source_url="https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/" + "0" * 40 + "/pyAgxArm/api/constants.py")
        _, source = mc.load_model_catalog(spec)
        self.assertFalse(source["recognized_official_snapshot"])

    def test_cli_writes_new_report_refuses_overwrite_and_duplicate_json(self):
        document = self.document([self.sample()])
        path, output = self.path / "input.json", self.path / "report.json"
        path.write_text(json.dumps(document))
        self.assertEqual(mc.main([str(path), "--output", str(output)]), 0)
        content = output.read_bytes()
        with self.assertRaises(SystemExit): mc.main([str(path), "--output", str(output)])
        self.assertEqual(output.read_bytes(), content)
        with self.assertRaises(ValueError): mc._json('{"a":1,"a":2}')
        with self.assertRaises(ValueError): mc._json('{"a":NaN}')


if __name__ == "__main__":
    unittest.main()
