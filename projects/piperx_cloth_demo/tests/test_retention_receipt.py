"""Pure retention summaries over synthetic feedback, never physical proof."""
import copy
import unittest
from unittest.mock import patch

from robot_tools.retention_receipt import (digest, measured_anchor, summarize_retention_trace,
                                           summarize_release_confirmation, grasp_body_anchor)
from test_contact_receipt import sample


class RetentionReceiptTests(unittest.TestCase):
    def setUp(self):
        guard = patch("socket.socket", side_effect=AssertionError("No real sockets"))
        guard.start()
        self.addCleanup(guard.stop)
        self.samples = [sample(100+i*.05, width=.039) for i in range(61)]
        self.identity = {"episode_id": "episode-right", "arm": "right", "run_id": "run-1",
                         "owner": "owner-1", "epoch": "epoch-1", "object_id": "plug"}
        self.args = dict(arm="right", identity=self.identity, probe_event_id="probe-right",
                         trace_id="trace-1", trace_sha256=digest(self.samples), samples=self.samples,
                         original_anchor=measured_anchor(self.samples[0]["arms"]["right"]), now=103.)

    def test_complete_dual_trace_has_bound_identity_original_anchor_and_measured_spans(self):
        before = copy.deepcopy(self.args)
        result = summarize_retention_trace(**self.args)
        self.assertEqual(self.args, before)
        self.assertEqual(result["identity"], self.identity)
        self.assertEqual(result["anchor"], self.args["original_anchor"])
        self.assertEqual(result["feedback_advances"], 60)
        self.assertEqual(result["sample_count"], 61)
        self.assertEqual(result["ended_at"]-result["started_at"], 3.)
        self.assertNotIn("dispatch_authorized", result)
        self.assertNotIn("physical_stop_verified", result)
        result["anchor"]["width_m"] = 0
        self.assertEqual(self.args["original_anchor"]["width_m"], .039)

    def test_device_can_supply_unbound_historical_probe_metadata_without_inventing_identity(self):
        result = summarize_retention_trace(**{**self.args, "identity": None, "probe_event_id": None})
        self.assertNotIn("identity", result)
        self.assertNotIn("probe_event_id", result)
        self.assertEqual(result["ended_at"], 103.)

    def test_incomplete_or_stale_window_is_rejected(self):
        for changes in ({"samples": self.samples[1:]}, {"now": 103.101}, {"now": float("nan")},
                        {"identity": {**self.identity, "arm": "left"}}):
            with self.subTest(changes=tuple(changes)), self.assertRaises(ValueError):
                summarize_retention_trace(**{**self.args, **changes})

    def test_peer_fault_missing_fragment_and_regression_are_not_hidden_by_final_stability(self):
        for kind in ("fault", "missing", "regression"):
            samples = copy.deepcopy(self.samples)
            state = samples[20]["arms"]["left"]
            if kind == "fault":
                state["drivers"]["1"]["foc_status"]["collision_status"] = True
            elif kind == "missing":
                del state["fragment_timestamps_s"]["joint_12"]
            else:
                state["fragment_timestamps_s"]["joint_12"] -= .06
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                summarize_retention_trace(**{**self.args, "samples": samples})

    def test_uncommanded_peer_window_motion_is_rejected(self):
        self.samples[20]["arms"]["left"]["pose_m_rad"][0] += .000501
        with self.assertRaises(ValueError):
            summarize_retention_trace(**self.args)

    def test_stable_offset_is_measured_against_original_anchor(self):
        for item in self.samples:
            item["arms"]["right"]["gripper"]["width_m"] += .0003
        result = summarize_retention_trace(**self.args)
        self.assertAlmostEqual(result["anchor_deviation"]["jaw_m"], .0003)
        self.assertEqual(result["spans"]["jaw_m"], 0.)
        for item in self.samples:
            item["arms"]["right"]["gripper"]["width_m"] += .0003
        with self.assertRaises(ValueError):
            summarize_retention_trace(**self.args)

    def test_completed_local_body_is_explicit_without_rewriting_probe_or_jaw_history(self):
        original = copy.deepcopy(self.args["original_anchor"])
        local = copy.deepcopy(original)
        local["joints_rad"][0] += .006
        record = {"status": "retained_local", "original_anchor": original, "local_anchor": local}
        for item in self.samples:
            item["arms"]["right"]["joints_rad"][0] += .006
        with self.assertRaises(ValueError):
            summarize_retention_trace(**self.args)
        result = summarize_retention_trace(**{**self.args, "original_anchor": grasp_body_anchor(record)})
        self.assertEqual(result["anchor"], local)
        self.assertEqual(record["original_anchor"], self.args["original_anchor"])
        for state in ("loaded_pending_visual", "release_pending", "release_opened"):
            record["status"] = state
            self.assertEqual(grasp_body_anchor(record), local)
        record["status"] = "retained_static"
        self.assertEqual(grasp_body_anchor(record), original)
        detached = grasp_body_anchor({**record, "status": "retained_local"})
        detached["joints_rad"][0] += 1
        self.assertEqual(record["local_anchor"], local)
        # Explicit local body reference does not relax the measured closed jaw.
        self.samples[20]["arms"]["right"]["gripper"]["width_m"] += .000501
        with self.assertRaises(ValueError):
            summarize_retention_trace(**{**self.args, "original_anchor": local})

    def test_release_summary_allows_measured_opening_but_keeps_body_and_stable_span_checks(self):
        for item in self.samples:
            item["arms"]["right"]["gripper"]["width_m"] += .003
        with self.assertRaises(ValueError):
            summarize_retention_trace(**self.args)
        result = summarize_retention_trace(**self.args, release=True)
        self.assertAlmostEqual(result["anchor_deviation"]["jaw_m"], .003)
        self.samples[20]["arms"]["right"]["joints_rad"][1] += .0031
        with self.assertRaises(ValueError):
            summarize_retention_trace(**self.args, release=True)

    def test_release_confirmation_has_original_schema_and_a_new_fixed_jaw_anchor(self):
        for item in self.samples:
            item["arms"]["right"]["gripper"]["width_m"] = .042
        opening = {"trace_sha256": "a"*64, "finished_at": 99.9,
                   "observed_width_m": .042, "target_width_m": .043}
        result = summarize_release_confirmation(release_opening=opening, **self.args)
        self.assertEqual(set(result), set(summarize_retention_trace(**self.args, release=True)))
        self.assertEqual(result["anchor"]["width_m"], .039)
        self.assertEqual(result["observed"]["width_m"], .042)
        self.assertAlmostEqual(result["anchor_deviation"]["jaw_m"], .003)
        for item in self.samples:
            item["arms"]["right"]["gripper"]["width_m"] = .042501
        with self.assertRaisesRegex(ValueError, "opening anchor"):
            summarize_release_confirmation(release_opening=opening, **self.args)

    def test_release_confirmation_rejects_old_trace_target_loss_and_bad_opening_binding(self):
        opening = {"trace_sha256": "b"*64, "finished_at": 99.9,
                   "observed_width_m": .039, "target_width_m": .04}
        for change in ({"finished_at": 100.}, {"trace_sha256": "bad"},
                       {"finished_at": float("nan")}, {"target_width_m": .045}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                summarize_release_confirmation(release_opening={**opening, **change}, **self.args)
        # Observation labels alone do not make the underlying fragments new.
        opening["finished_at"] = 99.99
        for side in ("left", "right"):
            state = self.samples[0]["arms"][side]
            state["fragment_timestamps_s"]["joint_12"] = 99.99
        with self.assertRaisesRegex(ValueError, "new dual-arm feedback"):
            summarize_release_confirmation(release_opening=opening, **self.args)


if __name__ == "__main__":
    unittest.main()
