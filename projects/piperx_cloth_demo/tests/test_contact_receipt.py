"""Pure recorded-feedback tests; no SDK, socket, camera or robot connection."""
import copy
import math
import unittest
from enum import IntEnum
from unittest.mock import patch

from robot_tools.contact_receipt import MAX_PROBE_CLOSURE_M, classify_gripper_probe, probe_closure_within_bound
from test_execution import healthy_arm


def sample(at, width=.030, force=.0):
    states = {side: healthy_arm(at-.001) for side in ("left", "right")}
    for state in states.values():
        state["arm_status"].update(teach_status=0, mode_feedback=0)
        state["joints_rad"] = [0, .5, -.5, 0, 0, 0]
        state["gripper"].update(width_m=.040, force_N=0)
    states["right"]["gripper"].update(width_m=width, force_N=force)
    return {"observed_at_s": at, "arms": states}


class ContactReceiptTests(unittest.TestCase):
    def setUp(self):
        self.socket_guard = patch("socket.socket", side_effect=AssertionError("No physical sockets"))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)
        self.args = dict(arm="right", requested_width_m=.025, sent_at=103.02,
                         baseline_samples=[sample(100+i*.05) for i in range(61)],
                         post_samples=[sample(103.07+i*.05, .03-.0015*min(1, i/10)) for i in range(81)])

    def classify(self):
        return classify_gripper_probe(**self.args)

    def assert_unconfirmed(self):
        result = self.classify()
        self.assertEqual(result["outcome"], "unconfirmed", result)
        self.assertIsNone(result["accepted"])
        self.assertFalse(result["grasp_verified"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertTrue(result["reasons"])
        return result

    def test_partial_closing_then_stable_is_candidate_only(self):
        before = copy.deepcopy(self.args)
        result = self.classify()
        self.assertEqual(result["outcome"], "settled_contact_candidate", result)
        self.assertEqual(result["completion"], "observation_only")
        self.assertFalse(result["arrival_confirmed"])
        self.assertFalse(result["contact_support_verified"])
        self.assertIsNone(result["contact_verified"])
        self.assertIsNone(result["accepted"])
        self.assertFalse(result["grasp_verified"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertFalse(result["target_cancellation_verified"])
        self.assertAlmostEqual(result["closure_displacement_m"], .0015)
        self.assertAlmostEqual(result["observed_width_m"], .0285)
        self.assertEqual(result["max_probe_closure_m"], MAX_PROBE_CLOSURE_M)
        self.assertEqual(before, self.args)

    def test_width_arrival_does_not_claim_grasp_or_acceptance(self):
        for entry in self.args["post_samples"]:
            entry["arms"]["right"]["gripper"]["width_m"] = .025
        result = self.classify()
        self.assertEqual(result["outcome"], "target_arrived", result)
        self.assertTrue(result["arrival_confirmed"])
        self.assertIsNone(result["accepted"])
        self.assertFalse(result["grasp_verified"])

    def test_no_width_response_with_force_increase_is_unconfirmed(self):
        for entry in self.args["post_samples"]:
            entry["arms"]["right"]["gripper"].update(width_m=.030, force_N=5.0)
        result = self.assert_unconfirmed()
        self.assertEqual(result["observed_force_N"], 5.0)
        self.assertEqual(result["completion"], "observation_only")
        self.assertEqual(result["closure_displacement_m"], 0)

    def test_sub_span_width_response_is_not_contact_candidate(self):
        for entry in self.args["post_samples"]:
            entry["arms"]["right"]["gripper"]["width_m"] = .030-.00049
        self.assert_unconfirmed()

    def test_every_baseline_sample_is_within_five_mm_closing_bound(self):
        self.args["baseline_samples"][10]["arms"]["right"]["gripper"]["width_m"] = .030001
        self.assert_unconfirmed()

    def test_exact_five_mm_machine_roundoff_is_not_a_larger_physical_limit(self):
        self.assertTrue(probe_closure_within_bound(.05, .045))
        self.assertFalse(probe_closure_within_bound(.05+1e-9, .045))
        self.assertFalse(probe_closure_within_bound(.045, .045))
        self.assertFalse(probe_closure_within_bound(True, .045))
        self.args["requested_width_m"] = .045
        for entry in self.args["baseline_samples"]:
            entry["arms"]["right"]["gripper"]["width_m"] = .05
        for i, entry in enumerate(self.args["post_samples"]):
            entry["arms"]["right"]["gripper"]["width_m"] = .05-.0015*min(1, i/10)
        self.assertEqual(self.classify()["outcome"], "settled_contact_candidate")

    def test_sdk_int_enums_are_accepted_but_boolean_and_float_statuses_are_not(self):
        class SDKStatus(IntEnum):
            ZERO = 0
        for trace in (self.args["baseline_samples"], self.args["post_samples"]):
            for entry in trace:
                for state in entry["arms"].values():
                    for field in ("motion_status", "teach_status", "mode_feedback"):
                        state["arm_status"][field] = SDKStatus.ZERO
        self.assertEqual(self.classify()["outcome"], "settled_contact_candidate")
        for value in (False, 0.0):
            with self.subTest(value=repr(value)):
                self.args["post_samples"][5]["arms"]["right"]["arm_status"]["motion_status"] = value
                self.assert_unconfirmed()

    def test_opening_or_zero_closure_request_is_unconfirmed(self):
        for target in (.030, .031):
            with self.subTest(target=target):
                self.args["requested_width_m"] = target
                self.assert_unconfirmed()

    def test_nonfinite_and_boolean_requests_are_unconfirmed(self):
        for target in (True, float("nan"), float("inf"), -.001, .056):
            with self.subTest(target=target):
                self.args["requested_width_m"] = target
                self.assert_unconfirmed()

    def test_nonfinite_feedback_is_unconfirmed(self):
        self.args["post_samples"][20]["arms"]["left"]["gripper"]["force_N"] = math.nan
        self.assert_unconfirmed()

    def test_complete_future_stale_and_repeated_traces_are_not_new_receipts(self):
        original = copy.deepcopy(self.args)
        for defect in ("future", "stale", "all_repeated", "out_of_order", "partial"):
            with self.subTest(defect=defect):
                self.args = copy.deepcopy(original)
                entry = self.args["post_samples"][20]
                if defect == "future":
                    entry["arms"]["left"]["fragment_timestamps_s"]["joint_12"] = entry["observed_at_s"]+.001
                elif defect == "stale":
                    entry["arms"]["left"]["fragment_timestamps_s"]["joint_12"] -= .101
                elif defect == "all_repeated":
                    for item in self.args["post_samples"]:
                        item["arms"] = copy.deepcopy(self.args["post_samples"][0]["arms"])
                elif defect == "out_of_order":
                    entry["observed_at_s"] = self.args["post_samples"][19]["observed_at_s"]
                else:
                    entry["arms"]["left"]["status"] = "partial"
                self.assert_unconfirmed()

    def test_old_postsend_fragments_do_not_prove_response(self):
        first = self.args["post_samples"][0]["arms"]["right"]
        first["fragment_timestamps_s"]["joint_12"] = self.args["sent_at"]
        self.assert_unconfirmed()

    def test_omitted_initial_postsend_interval_is_unconfirmed(self):
        self.args["post_samples"] = self.args["post_samples"][4:]
        self.assert_unconfirmed()

    def test_orientation_wrap_is_equivalent_but_actual_rotation_is_rejected(self):
        for entry in self.args["baseline_samples"]:
            entry["arms"]["left"]["pose_m_rad"][5] = math.pi
        for entry in self.args["post_samples"]:
            entry["arms"]["left"]["pose_m_rad"][5] = -math.pi
        self.assertEqual(self.classify()["outcome"], "settled_contact_candidate")
        self.args["post_samples"][20]["arms"]["left"]["pose_m_rad"][5] += .0031
        self.assert_unconfirmed()

    def test_final_window_straddling_arrival_boundary_is_unconfirmed(self):
        for i, entry in enumerate(self.args["post_samples"]):
            entry["arms"]["right"]["gripper"]["width_m"] = .0271 if i%2 else .0269
        self.assert_unconfirmed()

    def test_short_baseline_and_short_settled_window_are_unconfirmed(self):
        original = copy.deepcopy(self.args)
        self.args["baseline_samples"] = self.args["baseline_samples"][1:]
        self.assert_unconfirmed()
        self.args = original
        self.args["post_samples"] = self.args["post_samples"][:40]
        self.assert_unconfirmed()

    def test_sparse_final_quiet_snapshots_cannot_hide_missing_continuous_evidence(self):
        self.args["post_samples"] = self.args["post_samples"][::4]
        self.assert_unconfirmed()

    def test_transient_arm_peer_jaw_fault_and_mode_change_are_not_hidden_by_settling(self):
        original = copy.deepcopy(self.args)
        for defect in ("arm_move", "jaw_move", "fault", "mode", "disabled"):
            with self.subTest(defect=defect):
                self.args = copy.deepcopy(original)
                state = self.args["post_samples"][4]["arms"]["left"]
                if defect == "arm_move":
                    state["joints_rad"][0] += .003001
                elif defect == "jaw_move":
                    state["gripper"]["width_m"] += .002001
                elif defect == "fault":
                    state["gripper"]["foc_status"]["sensor_status"] = True
                elif defect == "mode":
                    state["arm_status"]["mode_feedback"] = 1
                else:
                    state["drivers"]["3"]["foc_status"]["driver_enable_status"] = False
                self.assert_unconfirmed()

    def test_selected_jaw_escape_then_settle_remains_unconfirmed(self):
        self.args["post_samples"][4]["arms"]["right"]["gripper"]["width_m"] = .020
        self.assert_unconfirmed()

    def test_settled_span_above_existing_bound_is_unconfirmed(self):
        for i, entry in enumerate(self.args["post_samples"]):
            entry["arms"]["right"]["gripper"]["width_m"] = .0285 + (i%2)*.000501
        self.assert_unconfirmed()

    def test_repeated_fragments_count_advances_against_last_complete_advance(self):
        self.args["baseline_samples"] = [sample(100+i*.01) for i in range(306)]
        self.args["sent_at"] = 103.07
        self.args["post_samples"] = [sample(103.12+i*.01, .0285) for i in range(306)]
        for trace in (self.args["baseline_samples"], self.args["post_samples"]):
            for i, entry in enumerate(trace):
                stamp = trace[0]["observed_at_s"]+(i//3)*.03-.001
                for state in entry["arms"].values():
                    for key in state["fragment_timestamps_s"]:
                        state["fragment_timestamps_s"][key] = stamp
                    state["gripper"]["timestamp"] = stamp
        result = self.classify()
        self.assertEqual(result["outcome"], "settled_contact_candidate", result)
        self.assertGreaterEqual(result["settled_feedback_advances"], 20)


if __name__ == "__main__":
    unittest.main()
