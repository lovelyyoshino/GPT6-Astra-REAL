"""Audited supported re-closing in FakeCAN only; no physical qualification."""
import copy
import math
import unittest
from unittest.mock import patch

from robot_tools import pair_device
from robot_tools.contact_receipt import classify_gripper_probe
from robot_tools.feedback_tolerance import RIGHT_J4_RAD, validate_policy
from robot_tools.retention_receipt import anchor_deviation, digest, measured_anchor
from test_single_supervised_actions import SingleActionFixture


class SupportedReacquisitionDeviceTests(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.stack.enter_context(patch.object(pair_device, "time", self.clock))
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True]*6
            robot.gripper_enabled = True
        self.robots["left"].width = .03836
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)))
        self.addCleanup(self.device.close)

    def source(self):
        sample = self.device.open()
        anchor = measured_anchor(sample["arms"]["left"])
        anchor["width_m"] = .032
        anchor["pose_m_rad"][0] -= .0003
        at = self.clock.time()
        source = {"candidate_measurement": {"anchor": anchor, "observed": {"width_m": .032}},
                  "candidate_probe": {"requested_width_m": .030, "sent_at": at-40,
                                      "completed_at": at-30, "trace_sha256": "a"*64},
                  "before": copy.deepcopy(sample["arms"])}
        source["audited_supported_reacquisition"] = {
            "schema": "piper_supported_reacquisition_v1", "arm": "left",
            "source_receipt_sha256": digest(source), "opening_event_id": "successful-opening",
            "opening_receipt_sha256": "b"*64, "opening_finished_at": at-10,
            "audit_proposal_sha256": "c"*64,
            "opening_jaw_anchor": {"width_m": .03836, "observed_at": at-10.001,
                "source": {"path": "/synthetic/successful-opening.json", "sha256": "d"*64}}}
        return source

    def contact_response(self, width):
        self.robots["left"].accept = False
        previous = len(self.robots["left"].sent)
        def contact(robot, state):
            if robot.side == "left" and robot.sent[previous:]:
                robot.width = width
                state["gripper"]["width_m"] = width
        self.hook = contact

    @staticmethod
    def rehash_source(source):
        envelope = source.pop("audited_supported_reacquisition")
        envelope["source_receipt_sha256"] = digest(source)
        source["audited_supported_reacquisition"] = envelope

    def crossed_body_source(self):
        source = self.source()
        anchor = source["candidate_measurement"]["anchor"]
        # The fresh baseline and later feedback sit on opposite sides of
        # the original body origin, just as in the recorded J4 fault.
        anchor["joints_rad"][3] -= .0015
        anchor["pose_m_rad"][3] -= .0015
        self.rehash_source(source)
        return source

    def continued_source(self, width=.03388):
        self.robots["left"].width = width
        source = self.source()
        at = self.clock.time()
        source["audited_supported_reacquisition"]["completed_probe_continuation"] = {
            "schema": "piper_completed_contact_continuation_v1", "arm": "left",
            "failed_event_id": "fully-sent-probe-failed", "failed_receipt_sha256": "e"*64,
            "prior_sent_target_m": .034, "failed_finished_at": at-5,
            "consumed_probe_count": 2, "remaining_probe_count": 1,
            "residual_jaw_anchor": {"width_m": width, "observed_at": at-.01,
                "source": {"path": "/synthetic/new-passive-observation.json", "sha256": "f"*64}}}
        return source

    def post_send_body(self, *, joint=-.0039, position=-.0006, rotation=-.0039):
        def change(robot, state):
            if robot.side == "left" and robot.sent:
                state["joints_rad"][3] += joint
                state["pose_m_rad"][0] += position
                state["pose_m_rad"][3] += rotation
        self.hook = change

    def test_body_crosses_fresh_baseline_but_stays_within_immutable_original(self):
        source = self.crossed_body_source()
        original = copy.deepcopy(source)
        idle = copy.deepcopy(self.device._action.idle_anchor)
        self.post_send_body()
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        before = measured_anchor(result["before"]["left"])
        after = result["after"]["left"]
        fresh_difference = anchor_deviation(before, after)
        original_difference = anchor_deviation(source["candidate_measurement"]["anchor"], after)
        for key, limit in (("joint_rad", .003), ("position_m", .0005), ("rotation_rad", .003)):
            self.assertGreater(fresh_difference[key], limit)
            self.assertLess(original_difference[key], limit)
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        self.assertTrue(result["observed_stable"])
        for side in ("left", "right"):
            for key in ("joints_rad", "pose_m_rad"):
                self.assertEqual(self.device._action.anchor[side][key], result["before"][side][key])
                self.assertEqual(self.device._action.idle_anchor[side][key], idle[side][key])
        self.assertEqual(source, original)
        self.assert_only_left_jaw(1)
        trace = next(data for event, data in self.events if event == "bounded_probe_trace")
        legacy = classify_gripper_probe(arm="left", requested_width_m=.034, sent_at=trace["sent_at"],
            baseline_samples=trace["trace"]["baseline"], post_samples=trace["trace"]["post"])
        self.assertEqual(legacy["outcome"], "unconfirmed")
        self.assertIn("pre-send stationary arm anchor", legacy["reasons"][-1])

    def test_completed_probe_continuation_uses_residual_jaw_and_one_last_candidate(self):
        source = self.continued_source()
        original = copy.deepcopy(source)
        self.contact_response(.0325)
        result = self.device.reacquire_supported_gripper("left", .0289, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["status"], "settled_contact_candidate")
        self.assertEqual(result["supported_probe_index"], 3)
        self.assertEqual(result["before"]["left"]["gripper"]["width_m"], .03388)
        self.assertEqual(result["audited_supported_reacquisition"]["opening_jaw_anchor"]["width_m"], .03836)
        self.assertEqual(source, original)
        retained = self.device.retain_grasp("left",
            identity={"episode_id": "last-left-probe", "arm": "left", "run_id": "run",
                      "owner": "owner", "epoch": "epoch", "object_id": "charger"},
            probe_event_id="last-probe", probe_trace_sha256=result["candidate_probe"]["trace_sha256"],
            deadline_at=self.clock.time()+120)
        self.assertTrue(retained["ok"], retained)
        self.assertEqual(retained["hardware_commands_sent"], 0)
        with self.assertRaisesRegex(RuntimeError, "at most three"):
            self.device.reacquire_supported_gripper("left", .028, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_completed_probe_arrival_does_not_grant_a_fourth_probe(self):
        source = self.continued_source()
        result = self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["status"], "target_arrived")
        with self.assertRaisesRegex(RuntimeError, "at most three"):
            self.device.reacquire_supported_gripper("left", .029, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_completed_probe_old_target_is_rejected_even_if_current_jaw_is_wider(self):
        source = self.continued_source(width=.035)
        with self.assertRaisesRegex(RuntimeError, "strictly smaller new target; no replay"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_completed_probe_old_encoded_target_is_also_rejected(self):
        source = self.continued_source(width=.035)
        with self.assertRaisesRegex(RuntimeError, "strictly smaller new target; no replay"):
            self.device.reacquire_supported_gripper("left", .0339996, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_completed_probe_residual_jaw_must_match_current_feedback(self):
        source = self.continued_source()
        source["audited_supported_reacquisition"]["completed_probe_continuation"]["residual_jaw_anchor"]["width_m"] -= .000501
        with self.assertRaisesRegex(RuntimeError, "jaw left its observed anchor"):
            self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_completed_probe_requires_new_residual_provenance(self):
        source = self.continued_source()
        del source["audited_supported_reacquisition"]["completed_probe_continuation"]["residual_jaw_anchor"]["source"]
        with self.assertRaisesRegex(ValueError, "residual jaw provenance"):
            self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_completed_probe_schema_budget_and_time_are_strict(self):
        source = self.continued_source()
        for key, value in (("schema", "other"), ("arm", "right"), ("consumed_probe_count", 1),
                           ("remaining_probe_count", 2), ("remaining_probe_count", True),
                           ("failed_receipt_sha256", "z"*64), ("failed_event_id", "successful-opening"),
                           ("failed_finished_at", self.clock.time()), ("prior_sent_target_m", float("nan"))):
            with self.subTest(key=key, value=value):
                bad = copy.deepcopy(source)
                bad["audited_supported_reacquisition"]["completed_probe_continuation"][key] = value
                with self.assertRaises(ValueError):
                    pair_device._validated_supported_reacquisition(bad, "left", self.clock.time())
        self.assert_only_left_jaw(0)

    def test_actual_original_joint_boundary_is_still_enforced(self):
        source = self.crossed_body_source()
        self.post_send_body(joint=-.004501)
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("stationary arm exceeded", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(1)

    def test_actual_original_position_boundary_is_still_enforced(self):
        source = self.crossed_body_source()
        self.post_send_body(position=-.000801)
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("stationary arm exceeded", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(1)

    def test_actual_original_rotation_boundary_is_still_enforced(self):
        source = self.crossed_body_source()
        self.post_send_body(rotation=-.004501)
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("stationary arm exceeded", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(1)

    def test_new_three_second_baseline_still_rejects_span_inside_old_body_box(self):
        source = self.crossed_body_source()
        def change(robot, state):
            action = self.device._action
            if robot.side == "left" and action.probe_mode == "close" and action.baseline_window is not None:
                state["joints_rad"][3] -= .0039
                state["pose_m_rad"][3] -= .0039
        self.hook = change
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("baseline exceeded stable feedback spans", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(0)

    def test_final_window_must_still_settle_inside_original_body_box(self):
        source = self.crossed_body_source()
        reads = 0
        def change(robot, state):
            nonlocal reads
            if robot.side == "left" and robot.sent:
                reads += 1
                # Every sample fits the original .003 rad body bound, but
                # the continuing .0032 rad span never forms a stable window.
                state["joints_rad"][3] -= .0015 + (.0016 if reads % 2 else -.0016)
        self.hook = change
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("Observation timeout", result["errors"][-1]["detail"])
        self.assertFalse(result["observed_stable"])
        self.assert_only_left_jaw(1)

    def test_selected_jaw_interval_does_not_use_original_body_jaw(self):
        source = self.crossed_body_source()
        self.post_send_body()
        body_hook = self.hook
        def change(robot, state):
            body_hook(robot, state)
            if robot.side == "left" and robot.sent:
                # Fits the old candidate jaw, but not this closure's target
                # interval [34, 38.36] mm with the unchanged 2 mm margin.
                state["gripper"]["width_m"] = .0319
        self.hook = change
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("Selected jaw left requested width interval", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(1)

    def test_passive_jaw_remains_fixed_during_crossed_body_closure(self):
        source = self.crossed_body_source()
        self.post_send_body()
        body_hook = self.hook
        def change(robot, state):
            body_hook(robot, state)
            if robot.side == "right" and self.robots["left"].sent:
                state["gripper"]["width_m"] += .000501
        self.hook = change
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("jaw left its observed anchor: right", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(1)

    def test_right_j4_profile_keeps_same_bound_about_its_original_body(self):
        source = self.source()
        # This fixture supplies native synthetic snapshots, not PiPER X CAN
        # fragment assembly. Exercise the already validated policy's guards.
        self.device._action.feedback_policy = validate_policy({"profile": "right_j4_bounded_v1", "source": "user",
                                     "statement": "Synthetic test: permit bounded right J4 feedback."})
        original = source["before"]["right"]
        original["joints_rad"][3] -= .006
        original["pose_m_rad"][3] -= .006
        self.rehash_source(source)
        def change(robot, state):
            if robot.side == "right" and self.robots["left"].sent:
                state["joints_rad"][3] -= .012
                state["pose_m_rad"][3] -= .012
        self.hook = change
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertAlmostEqual(RIGHT_J4_RAD, math.radians(.5))
        self.assertGreater(abs(result["after"]["right"]["joints_rad"][3]
                               - result["before"]["right"]["joints_rad"][3]), RIGHT_J4_RAD)
        self.assertAlmostEqual(anchor_deviation(measured_anchor(original), result["after"]["right"])["joint_rad"], .006)
        self.assert_only_left_jaw(1)

    def assert_only_left_jaw(self, count):
        self.assertEqual([frame.arbitration_id for frame in self.robots["left"].sent], [0x159]*count)
        self.assertEqual(self.robots["right"].sent, [])
        self.assertEqual(self.device._joint_cache, {"left": None, "right": None})
        for frame in self.robots["left"].sent:
            self.assertEqual(bytes(frame.data[4:]), bytes((0, 200, 1, 0)))

    def test_left_candidate_keeps_old_body_and_can_be_retained_without_tx(self):
        source = self.source()
        original = copy.deepcopy(source)
        self.contact_response(.036)
        result = self.device.reacquire_supported_gripper("left", .0335, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["contact_observation"]["outcome"], "settled_contact_candidate")
        self.assertGreaterEqual(result["baseline_duration_s"], 3)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        for key in ("joints_rad", "pose_m_rad"):
            self.assertEqual(result["candidate_measurement"]["anchor"][key], source["candidate_measurement"]["anchor"][key])
        self.assertEqual(result["candidate_measurement"]["anchor"]["width_m"], .036)
        self.assertEqual(result["reacquisition_original_anchor"]["width_m"], .032)
        self.assertFalse(result["grasp_verified"])
        self.assertIsNone(result["empty_jaw_verified"])
        self.assertIsNone(result["object_release_verified"])
        self.assertIsNone(self.device._supported_recovery_opening)
        identity = {"episode_id": "new-left-probe", "arm": "left", "run_id": "run",
                    "owner": "owner", "epoch": "epoch", "object_id": "charger"}
        retained = self.device.retain_grasp("left", identity=identity, probe_event_id="new-probe",
            probe_trace_sha256=result["candidate_probe"]["trace_sha256"], deadline_at=self.clock.time()+120)
        self.assertTrue(retained["ok"], retained)
        self.assertEqual(retained["hardware_commands_sent"], 0)
        self.assertFalse(retained["loaded"])
        self.assert_only_left_jaw(1)
        self.assertEqual(source, original)

    def test_target_arrival_allows_new_smaller_probe_then_candidate(self):
        source = self.source()
        first = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertTrue(first["ok"], first.get("errors"))
        self.assertEqual(first["status"], "target_arrived")
        self.assertEqual(self.device.grasp_states, {"left": None, "right": None})
        self.contact_response(.0325)
        second = self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assertTrue(second["ok"], second.get("errors"))
        self.assertEqual(second["status"], "settled_contact_candidate")
        self.assertEqual(second["supported_probe_index"], 2)
        self.assertEqual(second["reacquisition_original_anchor"], first["reacquisition_original_anchor"])
        self.assertEqual(second["before"]["left"]["gripper"]["width_m"], .034)
        self.assert_only_left_jaw(2)
        with self.assertRaisesRegex(RuntimeError, "without a candidate"):
            self.device.reacquire_supported_gripper("left", .028, source_receipt=source)
        self.assert_only_left_jaw(2)

    def test_at_most_three_complete_probes(self):
        source = self.source()
        for target in (.034, .030, .026):
            result = self.device.reacquire_supported_gripper("left", target, source_receipt=source)
            self.assertTrue(result["ok"], result.get("errors"))
        with self.assertRaisesRegex(RuntimeError, "at most three"):
            self.device.reacquire_supported_gripper("left", .022, source_receipt=source)
        self.assert_only_left_jaw(3)

    def test_target_arrival_does_not_allow_old_target_replay(self):
        source = self.source()
        self.assertTrue(self.device.reacquire_supported_gripper("left", .034, source_receipt=source)["ok"])
        with self.assertRaisesRegex(RuntimeError, "no replay"):
            self.device.reacquire_supported_gripper("left", .0339996, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_later_probe_cannot_change_audit_or_rebase_body(self):
        source = self.source()
        self.assertTrue(self.device.reacquire_supported_gripper("left", .034, source_receipt=source)["ok"])
        source["audited_supported_reacquisition"]["audit_proposal_sha256"] = "e"*64
        with self.assertRaisesRegex(RuntimeError, "same source"):
            self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_missing_provenance_is_zero_tx(self):
        source = self.source()
        del source["audited_supported_reacquisition"]["opening_jaw_anchor"]["source"]
        with self.assertRaisesRegex(ValueError, "provenance"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_changed_original_body_hash_is_zero_tx(self):
        source = self.source()
        source["candidate_measurement"]["anchor"]["pose_m_rad"][0] += .0001
        with self.assertRaisesRegex(ValueError, "receipt changed"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_opening_jaw_mismatch_is_zero_tx(self):
        source = self.source()
        source["audited_supported_reacquisition"]["opening_jaw_anchor"]["width_m"] -= .000501
        with self.assertRaisesRegex(RuntimeError, "jaw left its observed anchor"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_more_than_five_mm_is_rejected_without_tx(self):
        source = self.source()
        result = self.device.reacquire_supported_gripper("left", .033359, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("within 5 mm", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(0)

    def test_opening_target_is_rejected_without_tx(self):
        source = self.source()
        result = self.device.reacquire_supported_gripper("left", .040, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("strictly directional", result["errors"][-1]["detail"])
        self.assert_only_left_jaw(0)

    def test_rehashed_source_cannot_substitute_an_incompatible_old_body(self):
        source = self.source()
        source["candidate_measurement"]["anchor"]["pose_m_rad"][0] -= .001
        envelope = source.pop("audited_supported_reacquisition")
        envelope["source_receipt_sha256"] = digest(source)
        source["audited_supported_reacquisition"] = envelope
        with self.assertRaisesRegex(RuntimeError, "stationary arm exceeded|body left its original anchor"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_not_a_first_physical_operation_cannot_install_reacquisition(self):
        source = self.source()
        self.assertTrue(self.device.execute("left", "gripper", .038)["ok"])
        with self.assertRaisesRegex(RuntimeError, "first physical operation"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_old_body_is_checked_during_baseline(self):
        source = self.source()
        def drift(robot, state):
            if robot.side == "left" and self.device._action.supported_reacquisition is not None:
                state["pose_m_rad"][0] += .0003
        self.hook = drift
        with self.assertRaisesRegex(RuntimeError, "stationary arm exceeded|body left its original anchor"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_final_fresh_feedback_keeps_original_body_reference(self):
        source = self.source()
        def drift(robot, state):
            context = self.device._action.supported_reacquisition
            if robot.side == "left" and context is not None and context["closure_finished"]:
                state["pose_m_rad"][0] += .0003
        self.hook = drift
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertRegex(result["errors"][-1]["detail"], "stationary arm exceeded|body left its original anchor")
        self.assert_only_left_jaw(1)

    def test_right_jaw_cannot_send_after_left_arrival(self):
        source = self.source()
        self.assertTrue(self.device.reacquire_supported_gripper("left", .034, source_receipt=source)["ok"])
        result = self.device.execute("right", "gripper", .049)
        self.assertFalse(result["ok"])
        self.assert_only_left_jaw(1)

    def test_body_tx_is_not_granted_by_target_arrival(self):
        source = self.source()
        self.assertTrue(self.device.reacquire_supported_gripper("left", .034, source_receipt=source)["ok"])
        pose = self.robots["left"].motion.origin[:]
        pose[2] += .003
        result = self.device.execute("left", "move", pose)
        self.assertFalse(result["ok"])
        self.assert_only_left_jaw(1)

    def test_failure_never_allows_a_second_probe(self):
        source = self.source()
        self.robots["left"].accept = False
        result = self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assertFalse(result["ok"])
        with self.assertRaises(RuntimeError):
            self.device.reacquire_supported_gripper("left", .030, source_receipt=source)
        self.assert_only_left_jaw(1)

    def test_source_arm_checked_before_tx(self):
        source = self.source()
        source["audited_supported_reacquisition"]["arm"] = "right"
        with self.assertRaisesRegex(ValueError, "schema and arm"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)

    def test_opening_chronology_checked_before_tx(self):
        source = self.source()
        source["audited_supported_reacquisition"]["opening_jaw_anchor"]["observed_at"] = self.clock.time()
        with self.assertRaisesRegex(ValueError, "chronology"):
            self.device.reacquire_supported_gripper("left", .034, source_receipt=source)
        self.assert_only_left_jaw(0)


if __name__ == "__main__":
    unittest.main()
