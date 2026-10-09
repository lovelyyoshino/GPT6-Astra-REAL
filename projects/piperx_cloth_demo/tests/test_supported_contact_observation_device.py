"""Existing-target contact observation: synthetic RX/FakeCAN, never hardware."""
import copy
import unittest
from unittest.mock import patch

from robot_tools import pair_device
from robot_tools.retention_receipt import digest, measured_anchor
from robot_tools.supported_contact_measurement import validate_source, measure
from test_single_supervised_actions import SingleActionFixture


class SupportedContactObservationDeviceTests(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.stack.enter_context(patch.object(pair_device, "time", self.clock))
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True]*6
            robot.gripper_enabled = True
        self.robots["left"].width = .03325
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)))
        self.addCleanup(self.device.close)

    def source(self):
        sample = self.device.open()
        now = self.clock.time()
        anchor = measured_anchor(sample["arms"]["left"])
        anchor["width_m"] = .032
        anchor["pose_m_rad"][0] -= .0003
        source = {"candidate_measurement": {"anchor": anchor, "observed": {"width_m": .032}},
                  "candidate_probe": {"requested_width_m": .030, "sent_at": now-50,
                                      "completed_at": now-40, "trace_sha256": "a"*64},
                  "before": copy.deepcopy(sample["arms"])}
        source["audited_existing_contact"] = {
            "schema": "piper_existing_supported_contact_admission_v1", "arm": "left",
            "source_receipt_sha256": digest(source), "failed_event_id": "failed-third-closure",
            "failed_receipt_sha256": "b"*64, "prior_sent_target_m": .0295,
            "failed_sent_at": now-10, "failed_finished_at": now-5, "audit_proposal_sha256": "c"*64,
            "consumed_probe_count": 3, "remaining_probe_count": 0,
            "residual_jaw_anchor": {"width_m": self.robots["left"].width, "observed_at": now-.01,
                "source": {"path": "/synthetic/passive.json", "sha256": "d"*64}},
            "bilateral_contact_source": {"path": "/synthetic/user-bilateral.json", "sha256": "e"*64}}
        return source

    def observe(self, source):
        return self.device.observe_supported_contact("left", source_receipt=source)

    def assert_zero_tx(self):
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual(self.device._joint_cache, {"left": None, "right": None})

    def test_observed_contact_has_distinct_evidence_and_common_zero_tx_retention(self):
        source = self.source()
        original = copy.deepcopy(source)
        result = self.observe(source)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["status"], "observed_supported_contact_candidate")
        self.assertEqual(result["completion_mode"], "supported_contact_observe")
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(result["target_calls_sent"], 0)
        self.assertIsNone(result["nominal_force_N"])
        self.assertNotIn("candidate_probe", result)
        self.assertNotIn("conservative_closure_displacement_m", result["current_contact_candidate"])
        self.assertEqual(result["current_contact_candidate"]["existing_target_ref"]["event_id"], "failed-third-closure")
        self.assertAlmostEqual(result["current_contact_candidate"]["minimum_target_gap_m"], .00375)
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        self.assertEqual(result["candidate_measurement"]["anchor"]["pose_m_rad"], source["candidate_measurement"]["anchor"]["pose_m_rad"])
        self.assertEqual(result["candidate_measurement"]["anchor"]["width_m"], .03325)
        self.assertNotIn("identity", result["candidate_measurement"])
        self.assertNotIn("probe_event_id", result["candidate_measurement"])
        self.assertEqual(source, original)
        identity = {"episode_id": "new-observation", "arm": "left", "run_id": "run",
                    "owner": "owner", "epoch": "epoch", "object_id": "charger"}
        retained = self.device.retain_grasp("left", identity=identity, probe_event_id="new-zero-tx-event",
            probe_trace_sha256=result["current_contact_candidate"]["trace_sha256"], deadline_at=self.clock.time()+100)
        self.assertTrue(retained["ok"], retained)
        self.assertEqual(retained["status"], "retained_static")
        self.assertEqual(retained["candidate_basis"], "existing_target_observation")
        self.assertEqual(retained["hardware_commands_sent"], 0)
        self.assertFalse(retained["loaded"])
        self.assertEqual(self.device.grasp_states["left"]["probe_event_id"], "new-zero-tx-event")
        self.assertEqual(self.device.grasp_states["left"]["current_contact_candidate"]["existing_target_ref"]["event_id"], "failed-third-closure")
        self.assert_zero_tx()

    def test_negative_uncalibrated_force_is_not_used_as_grip_proof(self):
        source = self.source()
        def negative(robot, state):
            state["gripper"]["force_N"] = -.205
        self.hook = negative
        result = self.observe(source)
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["force_calibrated"])
        self.assertFalse(result["grasp_verified"])
        self.assertFalse(result["contact_support_verified"])
        self.assert_zero_tx()

    def test_original_body_reference_is_used_across_fresh_baseline_offset(self):
        source = self.source()
        def across(robot, state):
            if robot.side == "left" and self.device._action.supported_contact_observation is not None:
                state["pose_m_rad"][0] -= .0006
        self.hook = across
        result = self.observe(source)
        self.assertTrue(result["ok"], result)
        self.assert_zero_tx()

    def test_true_original_body_drift_is_rejected(self):
        source = self.source()
        def drift(robot, state):
            if robot.side == "left" and self.device._action.supported_contact_observation is not None:
                state["pose_m_rad"][0] += .0003
        self.hook = drift
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("stationary arm exceeded", result["error"])
        self.assert_zero_tx()

    def test_new_window_must_be_stable_even_inside_the_original_body_bounds(self):
        source = self.source()
        anchor = source["candidate_measurement"]["anchor"]
        anchor["joints_rad"][3] -= .0016
        audit = source.pop("audited_existing_contact")
        audit["source_receipt_sha256"] = digest(source)
        source["audited_existing_contact"] = audit
        def drift(robot, state):
            trace = self.device._action.retention_trace
            if robot.side == "left" and trace:
                state["joints_rad"][3] -= .0032
        self.hook = drift
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("baseline exceeded stable feedback spans", result["error"])
        self.assert_zero_tx()

    def test_jaw_reference_is_not_the_old_candidate_width(self):
        source = self.source()
        source["audited_existing_contact"]["residual_jaw_anchor"]["width_m"] -= .000501
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("fixed observed anchor", result["error"])
        self.assert_zero_tx()

    def test_arrived_jaw_without_remaining_shortfall_is_not_this_candidate(self):
        self.robots["left"].width = .0314
        source = self.source()
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("persistent jaw shortfall", result["error"])
        self.assertIsNone(self.device.grasp_states["left"])
        self.assert_zero_tx()

    def test_missing_bilateral_provenance_is_not_a_boolean_contact_claim(self):
        source = self.source()
        source["audited_existing_contact"]["bilateral_contact_source"] = True
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("source provenance", result["error"])
        self.assert_zero_tx()

    def test_source_schema_consumed_budget_and_chronology_remain_strict(self):
        source = self.source()
        for key, value in (("schema", "other"), ("arm", "right"), ("consumed_probe_count", 2),
                           ("remaining_probe_count", 1), ("remaining_probe_count", False),
                           ("prior_sent_target_m", float("nan")), ("failed_finished_at", self.clock.time()),
                           ("failed_receipt_sha256", "x"*64), ("failed_event_id", "same\n")):
            with self.subTest(key=key):
                bad = copy.deepcopy(source)
                bad["audited_existing_contact"][key] = value
                with self.assertRaises(ValueError):
                    validate_source(bad, "left", self.clock.time())
        self.assert_zero_tx()

    def test_changed_original_source_rejected_without_sends(self):
        source = self.source()
        source["candidate_measurement"]["anchor"]["joints_rad"][3] += .0001
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("source receipt changed", result["error"])
        self.assert_zero_tx()

    def test_new_candidate_blocks_selected_passive_jaw_and_body_sends(self):
        source = self.source()
        self.assertTrue(self.observe(source)["ok"])
        # Exercise the final sender guard directly as well as public operation
        # restrictions: even a new executor cannot gain a frame ticket here.
        for side, kind in (("left", "gripper"), ("right", "gripper"), ("left", "move"), ("right", "move")):
            with self.subTest(side=side, kind=kind):
                with self.assertRaises(RuntimeError):
                    self.device._action.require_supported_reacquisition_tx(side, kind)
        self.assert_zero_tx()

    def test_stale_current_feedback_does_not_become_new_evidence(self):
        source = self.source()
        def stale(robot, state):
            state["fragment_timestamps_s"]["joint_34"] -= .101
        self.hook = stale
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assert_zero_tx()

    def test_final_feedback_after_recording_still_checks_original_body(self):
        source = self.source()
        def drift(robot, state):
            if robot.side == "left" and any(event == "existing_supported_contact_trace" for event, _ in self.events):
                state["pose_m_rad"][0] += .0003
        self.hook = drift
        result = self.observe(source)
        self.assertFalse(result["ok"])
        self.assertIn("stationary arm exceeded", result["error"])
        self.assert_zero_tx()

    def test_classification_uses_full_new_window_not_only_the_last_sample(self):
        source = self.source()
        result = self.observe(source)
        self.assertTrue(result["ok"])
        record = next(data for event, data in self.events if event == "existing_supported_contact_trace")
        samples = copy.deepcopy(record["trace"])
        samples[20]["arms"]["left"]["joints_rad"][3] += .004
        with self.assertRaisesRegex(ValueError, "original body reference"):
            measure(arm="left", source=source, samples=samples, trace_id="bad-window", trace_sha256=digest(samples),
                    now=self.clock.time())
        self.assert_zero_tx()


if __name__ == "__main__":
    unittest.main()
