"""Synthetic evidence validation only; no model, simulator or robot calls."""
import copy
import unittest

from right_pick.fast_task_evaluation import (SCHEMA_VERSION, articulated_goal_evaluation_contract,
    evaluate_task_evidence, placement_evaluation_contract)


def document(*, mode="physical", claims=None):
    return dict(schema_version=SCHEMA_VERSION, run_id="trial-1", task_id="declared-object-and-goal",
                mode=mode, claims=claims or placement_evaluation_contract(), observations=[], events=[])


def visual(doc, name, at, facts, *, source="model_visual_report", origin="physical_rgb", observation_id=None):
    oid = observation_id or "rgb-" + name
    if not any(o["id"] == oid for o in doc["observations"]):
        doc["observations"].append(dict(id=oid, run_id=doc["run_id"], at=at, origin=origin,
                                       rgb_references=["fixtures/" + oid + ".png"]))
    event = dict(id=name, run_id=doc["run_id"], at=at, source=source,
                 reference="review/" + name + ".json", observation_id=oid, facts=facts)
    if source == "independent_rgb_review":
        event["reviewer"] = "independent-reviewer"
    doc["events"].append(event)
    return event


def command(doc, name, at, outcome="completed"):
    doc["events"].append(dict(id=name, run_id=doc["run_id"], at=at, source="robot_receipt",
                              reference="robot/" + name + ".json", outcome=outcome))


def placement(*, mode="physical", source="model_visual_report", origin="physical_rgb"):
    doc = document(mode=mode)
    command(doc, "release_command", 3)
    command(doc, "retreat_command", 5)
    for name, at, facts in [
        ("grasp", 1, {"object_retained": True}),
        ("lift", 2, {"object_retained": True, "original_support_separated": True}),
        ("release", 4, {"object_released": True, "destination_support_established": True}),
        ("retreat", 6, {"gripper_clear": True}),
        ("stable-first", 7, {"goal_satisfied": True, "independent_support": True, "gripper_clear": True}),
        ("stable-second", 10, {"goal_satisfied": True, "independent_support": True, "gripper_clear": True}),
    ]:
        visual(doc, name, at, facts, source=source, origin=origin)
    return doc


class TaskEvaluationTests(unittest.TestCase):
    def test_model_success_is_never_independent_physical_success(self):
        result = evaluate_task_evidence(placement())
        self.assertTrue(result["model_reported_success"])
        self.assertIsNone(result["task_success"])
        self.assertIsNone(result["independently_reviewed_success"])
        self.assertFalse(result["execution_available"])
        self.assertFalse(result["phase_transition_applied"])

    def test_complete_independent_sequence_can_verify_placement(self):
        result = evaluate_task_evidence(placement(source="independent_rgb_review"))
        self.assertTrue(result["task_success"])
        self.assertIsNone(result["model_reported_success"])
        self.assertIsNone(result["simulator_oracle_success"])
        self.assertEqual(result["channels"]["independent_rgb_review"]["claims"]["stability"]["completed_at"], 10)

    def test_robot_arrival_alone_cannot_satisfy_any_object_claim(self):
        doc = document()
        command(doc, "release_command", 1)
        command(doc, "retreat_command", 2)
        result = evaluate_task_evidence(doc)
        self.assertIsNone(result["task_success"])
        for channel in result["channels"].values():
            self.assertTrue(all(c["satisfied"] is not True for c in channel["claims"].values()))
        doc["events"][0]["facts"] = {"goal_satisfied": True}
        with self.assertRaises(ValueError):
            evaluate_task_evidence(doc)

    def test_independent_and_model_channels_cannot_complete_each_other(self):
        doc = placement()
        for event in doc["events"]:
            if event["id"].startswith("stable"):
                event.update(source="independent_rgb_review", reviewer="reviewer")
        result = evaluate_task_evidence(doc)
        self.assertIsNone(result["model_reported_success"])
        self.assertIsNone(result["task_success"])

    def test_observation_must_follow_completed_release_and_retreat(self):
        for event_name, changed_at in (("release_command", 4), ("retreat_command", 6)):
            doc = placement(source="independent_rgb_review")
            next(e for e in doc["events"] if e["id"] == event_name)["at"] = changed_at
            self.assertIsNone(evaluate_task_evidence(doc)["task_success"])

    def test_rejected_or_missing_command_is_not_release_evidence(self):
        for outcome in ("rejected", "missing"):
            doc = placement(source="independent_rgb_review")
            if outcome == "missing":
                doc["events"] = [e for e in doc["events"] if e["id"] != "release_command"]
            else:
                doc["events"][0]["outcome"] = outcome
            self.assertIsNone(evaluate_task_evidence(doc)["task_success"])

    def test_single_frame_and_short_gap_cannot_verify_stability(self):
        doc = placement(source="independent_rgb_review")
        doc["events"] = [e for e in doc["events"] if e["id"] != "stable-second"]
        self.assertIsNone(evaluate_task_evidence(doc)["task_success"])
        doc = placement(source="independent_rgb_review")
        next(o for o in doc["observations"] if o["id"] == "rgb-stable-second")["at"] = 8
        self.assertIsNone(evaluate_task_evidence(doc)["task_success"])

    def test_repeated_frame_cannot_be_counted_as_two_samples(self):
        doc = placement(source="independent_rgb_review")
        doc["events"][-1]["observation_id"] = "rgb-stable-first"
        with self.assertRaisesRegex(ValueError, "same observation"):
            evaluate_task_evidence(doc)

    def test_structural_review_explicitly_does_not_authenticate_frame_content(self):
        doc = placement(source="independent_rgb_review")
        for observation in doc["observations"]:
            observation["rgb_references"] = ["video.mp4#host-selected-frame"]
        result = evaluate_task_evidence(doc)
        self.assertTrue(result["task_success"])
        self.assertTrue(any("do not prove distinct decoded frames" in text for text in result["limitations"]))

    def test_old_rgb_cannot_complete_a_physical_model_report(self):
        result = evaluate_task_evidence(placement(origin="historical_rgb"))
        self.assertIsNone(result["model_reported_success"])
        self.assertIsNone(result["task_success"])

    def test_late_refutation_or_occlusion_invalidates_terminal_success(self):
        for fact, expected in ((False, False), (None, None)):
            doc = placement(source="independent_rgb_review")
            visual(doc, "late", 11, {"independent_support": fact}, source="independent_rgb_review")
            self.assertIs(evaluate_task_evidence(doc)["task_success"], expected)

    def test_refutation_requires_new_stability_window_before_success(self):
        doc = placement(source="independent_rgb_review")
        visual(doc, "occluded", 11, {"goal_satisfied": None}, source="independent_rgb_review")
        facts = {"goal_satisfied": True, "independent_support": True, "gripper_clear": True}
        visual(doc, "visible-again", 12, facts, source="independent_rgb_review")
        self.assertIsNone(evaluate_task_evidence(doc)["task_success"])
        visual(doc, "stable-again", 15, facts, source="independent_rgb_review")
        self.assertTrue(evaluate_task_evidence(doc)["task_success"])

    def test_replay_or_historical_images_never_establish_physical_success(self):
        for mode, origin in (("offline_replay", "physical_rgb"), ("offline_replay", "historical_rgb"),
                             ("physical", "historical_rgb"), ("physical", "simulation_rgb")):
            with self.subTest(mode=mode, origin=origin):
                result = evaluate_task_evidence(placement(mode=mode, source="independent_rgb_review", origin=origin))
                self.assertIsNone(result["task_success"])

    def test_articulated_goal_is_generic_and_simulator_oracle_stays_separate(self):
        doc = document(mode="simulation", claims=articulated_goal_evaluation_contract())
        for name, at, facts in (("initial", 1, {"initial_condition_visible": True}),
                                ("goal-first", 2, {"articulation_goal_visible": True}),
                                ("goal-second", 5, {"articulation_goal_visible": True})):
            visual(doc, name, at, facts, source="simulator_oracle", origin="simulation_rgb")
        result = evaluate_task_evidence(doc)
        self.assertTrue(result["simulator_oracle_success"])
        self.assertIsNone(result["model_reported_success"])
        self.assertIsNone(result["task_success"])
        doc["events"][-1]["facts"]["angle_dist"] = -0.1
        with self.assertRaisesRegex(ValueError, "metric object state"):
            evaluate_task_evidence(doc)

    def test_simulator_claims_cannot_relabel_physical_observation(self):
        doc = placement(mode="simulation", source="simulator_oracle")
        with self.assertRaisesRegex(ValueError, "simulation origin"):
            evaluate_task_evidence(doc)

    def test_custom_goal_predicates_do_not_require_pen_or_can_names(self):
        doc = document(claims=articulated_goal_evaluation_contract())
        doc["task_id"] = "drawer-defined-goal"
        doc["claims"][-1]["required_facts"] = ["drawer_at_declared_visible_goal"]
        for name, at, facts in (("initial", 1, {"initial_condition_visible": True}),
                                ("goal-a", 2, {"drawer_at_declared_visible_goal": True}),
                                ("goal-b", 5, {"drawer_at_declared_visible_goal": True})):
            visual(doc, name, at, facts, source="independent_rgb_review")
        self.assertTrue(evaluate_task_evidence(doc)["task_success"])

    def test_human_assistance_is_retained_and_autonomy_not_inferred(self):
        doc = placement(source="independent_rgb_review")
        doc["events"].append(dict(id="assistance", run_id=doc["run_id"], at=2.5, source="human_intervention",
                                 reference="eventlog:assist", scope="task", description="human repositioned the support"))
        result = evaluate_task_evidence(doc)
        self.assertTrue(result["task_success"])
        self.assertEqual(result["human_interventions"][0]["scope"], "task")
        self.assertEqual(result["autonomy_assessment"], "not_inferred_from_an_incomplete_intervention_log")

    def test_unknown_execution_outcome_blocks_physical_success(self):
        doc = placement(source="independent_rgb_review")
        command(doc, "late-command", 11, "uncertain")
        result = evaluate_task_evidence(doc)
        self.assertTrue(result["channels"]["independent_rgb_review"]["success"])
        self.assertTrue(result["execution_outcome_uncertain"])
        self.assertIsNone(result["task_success"])

    def test_later_completed_motion_requires_new_terminal_observations(self):
        doc = placement(source="independent_rgb_review")
        command(doc, "later-motion", 11)
        self.assertIsNone(evaluate_task_evidence(doc)["task_success"])
        facts = {"goal_satisfied": True, "independent_support": True, "gripper_clear": True}
        visual(doc, "after-motion-first", 12, facts, source="independent_rgb_review")
        self.assertIsNone(evaluate_task_evidence(doc)["task_success"])
        visual(doc, "after-motion-stable", 15, facts, source="independent_rgb_review")
        self.assertTrue(evaluate_task_evidence(doc)["task_success"])

    def test_late_intervention_in_any_scope_invalidates_old_terminal_view(self):
        for scope in ("setup", "task", "return"):
            doc = placement(source="independent_rgb_review")
            doc["events"].append(dict(id="later-human", run_id=doc["run_id"], at=11,
                source="human_intervention", reference="events:11", scope=scope,
                description="human changed the scene"))
            self.assertIsNone(evaluate_task_evidence(doc)["task_success"], scope)

    def test_wrong_run_duplicate_ids_missing_rgb_and_invalid_types_are_rejected(self):
        changes = (
            lambda d: d["observations"][0].update(run_id="another-run"),
            lambda d: d["events"][0].update(run_id="another-run"),
            lambda d: d["observations"][0].update(rgb_references=[]),
            lambda d: d["observations"][0].update(at=float("nan")),
            lambda d: d["events"].append(copy.deepcopy(d["events"][0])),
            lambda d: d["claims"][0].update(min_observations=True),
            lambda d: d["events"][-1]["facts"].update(goal_satisfied="true"),
            lambda d: d["events"][-1].update(source="robot_arrival_visual_success"),
        )
        for change in changes:
            doc = placement()
            change(doc)
            with self.assertRaises(ValueError):
                evaluate_task_evidence(doc)

    def test_cycles_missing_dependencies_and_no_terminal_goal_are_rejected(self):
        for change in (lambda d: d["claims"][0].update(after_claims=["stability"]),
                       lambda d: d["claims"][0].update(after_claims=["missing"]),
                       lambda d: d["claims"][-1].update(terminal=False)):
            doc = placement()
            change(doc)
            with self.assertRaises(ValueError):
                evaluate_task_evidence(doc)

    def test_input_document_is_unchanged_and_evaluation_repeatable(self):
        doc = placement(source="independent_rgb_review")
        frozen = copy.deepcopy(doc)
        self.assertEqual(evaluate_task_evidence(doc), evaluate_task_evidence(doc))
        self.assertEqual(doc, frozen)


if __name__ == "__main__":
    unittest.main()
