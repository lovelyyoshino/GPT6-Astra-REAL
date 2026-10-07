"""Pinned model-space hold checks; deliberately different raw pose telemetry."""
import copy
import unittest
from unittest.mock import patch

from robot_tools.hold_transaction import JointHoldTransaction, HoldTransactionError
from robot_tools.joint_geometry import OfficialJointModel
from test_hold_transaction import (event_and_claim, sample, digest, complete_frames,
                                   RAW_HOLD, IDENTITY, envelope_for)
from test_joint_path import context


class ModelHoldTests(unittest.TestCase):
    def setUp(self):
        block = patch("socket.socket", side_effect=AssertionError("No hardware/network"))
        block.start()
        self.addCleanup(block.stop)
        self.model = OfficialJointModel(context()["model_catalog"], "piper_x")

    def inputs(self):
        original, claim = event_and_claim()
        original["geometry_source"] = self.model.source
        claim["original_event_sha256"] = digest(original)
        return original, claim

    def envelope(self, state):
        return {"sample_id": state["sample_id"], "model": "piper_x",
                "source_ref": "offline-test-only", "source_sha256": "a"*64,
                "geometry_mode": "model_joint_geometry_v1", "attachment_radius_m": .02,
                "link_body_allowance_m": .06}

    def prepared(self):
        tx = JointHoldTransaction(*self.inputs(), model_geometry=self.model)
        state = sample(100.21, raw=RAW_HOLD)
        tx.prepare(state, self.envelope(state), current_identity=IDENTITY, now=100.21)
        return tx

    def test_separate_geometry_preserves_large_raw_difference_and_no_io_after_construction(self):
        original, claim = self.inputs()
        before = copy.deepcopy(original)
        with patch("pathlib.Path.read_bytes", side_effect=AssertionError("No frame-loop file I/O")):
            tx = JointHoldTransaction(original, claim, model_geometry=self.model)
            state = sample(100.21, raw=RAW_HOLD)
            tx.prepare(state, self.envelope(state), current_identity=IDENTITY, now=100.21)
            complete_frames(tx)
            for i in range(63):
                at = 100.25+i*.05
                report = tx.observe(sample(at, raw=RAW_HOLD), now=at)
        self.assertTrue(report["hold_observed"])
        self.assertEqual(original, before)
        geometry = report["geometry"]
        self.assertEqual(geometry["controller_pose_m_rad"], state["arms"]["right"]["pose_m_rad"])
        self.assertGreater(geometry["model_controller_difference_diagnostic"]["position_error_m"], .02)
        self.assertFalse(geometry["model_pose_is_independent_measurement"])
        self.assertIsNone(report["physical_stop_verified"])

    def test_old_contract_still_rejects_absolute_fk_difference(self):
        tx = JointHoldTransaction(*event_and_claim())
        state = sample(100.21, raw=RAW_HOLD)
        envelope = envelope_for(state)
        envelope["fk_pose_m_rad"][0] += .01
        with self.assertRaises(HoldTransactionError) as error:
            tx.prepare(state, envelope, current_identity=IDENTITY, now=100.21)
        self.assertEqual(error.exception.code, "model_controller_pose_mismatch")

    def test_source_is_bound_to_original_digest_and_real_pinned_model(self):
        original, claim = self.inputs()
        original["geometry_source"]["constants_sha256"] = "f"*64
        claim["original_event_sha256"] = digest(original)
        with self.assertRaises(HoldTransactionError):
            JointHoldTransaction(original, claim, model_geometry=self.model)
        with self.assertRaises(HoldTransactionError):
            JointHoldTransaction(*self.inputs())
        with self.assertRaises(HoldTransactionError):
            JointHoldTransaction(*event_and_claim(), model_geometry=self.model)

    def test_model_workspace_applies_to_both_models_not_raw_absolute_coordinates(self):
        original, claim = self.inputs()
        # Raw x is outside this model-space box, but both X FK positions fit.
        original["limits"]["workspace_min_m"] = [.09, -.05, .25]
        original["limits"]["workspace_max_m"] = [.16, .05, .36]
        claim["original_event_sha256"] = digest(original)
        tx = JointHoldTransaction(original, claim, model_geometry=self.model)
        state = sample(100.21, raw=RAW_HOLD)
        self.assertGreater(state["arms"]["right"]["pose_m_rad"][0], .16)
        self.assertEqual(tx.prepare(state, self.envelope(state), current_identity=IDENTITY,
                                    now=100.21)["status"], "prepared")
        original["reference"]["arms"]["left"]["joints_rad"][0] = .8
        claim["original_event_sha256"] = digest(original)
        with self.assertRaises(HoldTransactionError) as error:
            JointHoldTransaction(original, claim, model_geometry=self.model)
        self.assertEqual(error.exception.code, "model_workspace_limit")

    def test_model_drift_detected_even_if_raw_pose_does_not_change(self):
        tx = self.prepared()
        moved = list(RAW_HOLD)
        moved[1] += 150
        with self.assertRaises(HoldTransactionError) as error:
            tx.before_frame(tx.report()["expected_frames"][0], sample(100.22, raw=moved),
                            current_identity=IDENTITY, now=100.22)
        self.assertEqual(error.exception.code, "model_hold_plan_slip")
        self.assertEqual(tx.report()["frame_attempts"], [])

    def test_attachment_does_not_redefine_flange_budget_but_remains_sourced(self):
        for radius in (.02, .2):
            tx = JointHoldTransaction(*self.inputs(), model_geometry=self.model)
            state = sample(100.21, raw=RAW_HOLD)
            envelope = self.envelope(state)
            envelope["attachment_radius_m"] = radius
            report = tx.prepare(state, envelope, current_identity=IDENTITY, now=100.21)
            self.assertEqual(report["geometry"]["flange_joint_radius_bounds_m"], self.model.flange_radii())
            self.assertEqual(report["geometry"]["body_joint_radius_bounds_m"], self.model.radii(radius, .06))
        for field, value in (("attachment_radius_m", 0), ("attachment_radius_m", True),
                             ("link_body_allowance_m", 0), ("link_body_allowance_m", .01)):
            tx = JointHoldTransaction(*self.inputs(), model_geometry=self.model)
            envelope = self.envelope(state)
            envelope[field] = value
            with self.assertRaises(HoldTransactionError) as error:
                tx.prepare(state, envelope, current_identity=IDENTITY, now=100.21)
            self.assertEqual(error.exception.code, "invalid_geometry")

    def test_durable_frozen_target_accepts_small_new_feedback_error_without_retargeting(self):
        tx = JointHoldTransaction(*self.inputs(), model_geometry=self.model)
        latest = list(RAW_HOLD)
        latest[1] += 1  # An integer millidegree during the durable claim.
        state = sample(100.21, raw=latest)
        report = tx.prepare(state, self.envelope(state), current_identity=IDENTITY,
                            now=100.21, frozen_target_raw=list(RAW_HOLD))
        self.assertEqual(report["target_raw"], RAW_HOLD)
        self.assertNotEqual(report["target_raw"], latest)
        tx = JointHoldTransaction(*self.inputs(), model_geometry=self.model)
        invalid = list(RAW_HOLD)
        invalid[1] += 300
        with self.assertRaises(HoldTransactionError) as error:
            tx.prepare(state, self.envelope(state), current_identity=IDENTITY,
                       now=100.21, frozen_target_raw=invalid)
        self.assertEqual(error.exception.code, "frozen_hold_target_drift")


if __name__ == "__main__":
    unittest.main()
