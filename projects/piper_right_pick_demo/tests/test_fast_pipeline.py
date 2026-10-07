import unittest

from right_pick.fast_pipeline import (
    ATOMIC_SKILLS,
    DUAL_ARM_PIPELINE_ID,
    WORKER_OBSERVER_PIPELINE_ID,
    SINGLE_ARM_PIPELINE_ID,
    compact_pipeline,
    decision_stage,
    observer_action_allowed,
    pipeline_contract,
    validate_mode_config,
    validate_runtime_config,
    validate_pipeline,
    validate_fresh_observation,
    validate_preflight,
    validate_swept_corridors,
    synchronize_duration,
    validate_pair_admission,
    validate_pair_receipt,
    latch_pair_fault,
    validate_session_renewal,
    validate_hold_handoff,
    validate_return,
    validate_task_evidence,
)


class FastPipelineTests(unittest.TestCase):
    def test_catalog_is_atomic_and_bounded(self):
        names = [skill.name for skill in ATOMIC_SKILLS]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("observe_scene", names)
        self.assertIn("coordinate_pair", names)
        self.assertIn("dispatch_pair_once", names)
        self.assertIn("coordinate_observer", names)
        self.assertTrue(all(skill.max_invocations == 1 for skill in ATOMIC_SKILLS))

    def test_single_pipeline_is_complete_and_compact(self):
        contract = pipeline_contract("single_arm")
        self.assertEqual(contract["id"], SINGLE_ARM_PIPELINE_ID)
        self.assertEqual(contract["stages"][0], "observe_scene")
        self.assertEqual(contract["stages"][-1], "verify_visual")
        self.assertEqual(compact_pipeline()["stage"], "decide_one")
        self.assertEqual(compact_pipeline()["id"], SINGLE_ARM_PIPELINE_ID)

    def test_dual_pipeline_has_one_pair_barrier(self):
        contract = pipeline_contract("dual_arm")
        self.assertEqual(contract["id"], DUAL_ARM_PIPELINE_ID)
        self.assertEqual(contract["stages"], [
            "observe_scene", "decide_pair", "coordinate_pair", "dispatch_pair_once",
            "read_receipt", "verify_visual",
        ])
        self.assertEqual(contract["stages"].count("coordinate_pair"), 1)
        self.assertEqual(contract["stages"].count("dispatch_pair_once"), 1)
        self.assertNotIn("admit_action", contract["stages"])
        self.assertNotIn("dispatch_once", contract["stages"])
        self.assertEqual(contract["max_dispatches_per_cycle"], 1)
        self.assertTrue(contract["stop_on_uncertain_receipt"])

    def test_worker_observer_contract_is_explicit(self):
        contract = pipeline_contract("observer")
        self.assertEqual(contract["id"], WORKER_OBSERVER_PIPELINE_ID)
        self.assertTrue(contract["observer_requires_worker_hold"])
        self.assertFalse(contract["observer_can_execute_task_action"])
        self.assertEqual(contract["stages"][2], "coordinate_observer")
        self.assertEqual(decision_stage(WORKER_OBSERVER_PIPELINE_ID), "decide_one")

    def test_compact_pipeline_has_no_repeated_full_stage_list(self):
        compact = compact_pipeline("dual_arm", stage="decide_pair")
        self.assertEqual(compact, {
            "id": DUAL_ARM_PIPELINE_ID,
            "mode": "dual_arm",
            "stage": "decide_pair",
            "next": "coordinate_pair",
        })
        self.assertNotIn("stages", compact)
        self.assertEqual(decision_stage("dual_arm"), "decide_pair")

    def test_repeated_or_unknown_stage_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_pipeline(("observe_scene", "observe_scene"))
        with self.assertRaises(ValueError):
            validate_pipeline(("missing_skill",))

    def test_mode_wiring_is_validated_before_runtime(self):
        self.assertEqual(validate_mode_config({
            "pipeline_id": SINGLE_ARM_PIPELINE_ID,
            "execution_mode": "single_arm",
            "worker_arm": "right",
            "observer_arm": None,
        })['execution_mode'], "single_arm")
        with self.assertRaises(ValueError):
            validate_mode_config({
                "pipeline_id": WORKER_OBSERVER_PIPELINE_ID,
                "execution_mode": "worker_with_observer",
                "worker_arm": "left",
                "observer_arm": "left",
            })

    def test_task_peer_cannot_be_confused_with_view_only_observer(self):
        roles = validate_mode_config(dict(pipeline_id=DUAL_ARM_PIPELINE_ID,
                                          worker_arm="left", peer_arm="right"))
        self.assertEqual(roles["peer_arm"], "right")
        self.assertIsNone(roles["observer_arm"])
        for config in (
                dict(pipeline_id=DUAL_ARM_PIPELINE_ID, worker_arm="left", observer_arm="right"),
                dict(pipeline_id=DUAL_ARM_PIPELINE_ID, worker_arm="left", peer_arm="left"),
                dict(pipeline_id=WORKER_OBSERVER_PIPELINE_ID, worker_arm="left", peer_arm="right"),
                dict(pipeline_id=SINGLE_ARM_PIPELINE_ID, worker_arm="left", peer_arm="right")):
            with self.assertRaises(ValueError):
                validate_mode_config(config)

    def test_observer_can_only_move_for_view_and_requires_worker_hold(self):
        view_move = {"role": "observer", "intent": "view_only",
                     "task_effect": False, "action": "move_eef"}
        self.assertTrue(observer_action_allowed(view_move, worker_held=True))
        self.assertFalse(observer_action_allowed(view_move, worker_held=False))
        self.assertFalse(observer_action_allowed({
            "role": "observer", "intent": "grasp", "task_effect": True,
            "action": "gripper",
        }, worker_held=True))

    def test_arx5_lifecycle_is_explicit_but_model_packet_stays_compact(self):
        contract = pipeline_contract("dual_arm")
        self.assertEqual(contract["lifecycle"]["preflight"], ["preflight_pair"])
        self.assertIn("synchronize_duration", contract["required_atomic_skills"])
        self.assertIn("verify_return", contract["required_atomic_skills"])
        self.assertNotIn("required_atomic_skills", compact_pipeline("dual_arm", "decide_pair"))

    def test_fresh_observation_rejects_stale_duplicate_and_skewed_views(self):
        now = 100.0
        observation = {
            "observation_id": "scene-2",
            "cameras": {
                "front": {"timestamp": 99.9, "sequence": 2, "device": "f"},
                "left_hand": {"timestamp": 99.91, "sequence": 2, "device": "l"},
                "right_hand": {"timestamp": 99.92, "sequence": 2, "device": "r"},
            },
        }
        receipt = validate_fresh_observation(observation, now=now,
            previous_observation={"observation_id": "scene-1", "sequences": {"front": 1, "left_hand": 1, "right_hand": 1}})
        self.assertTrue(receipt["fresh"])
        with self.assertRaises(ValueError):
            validate_fresh_observation(dict(observation, observation_id="scene-1"), now=now,
                previous_observation={"observation_id": "scene-1", "sequences": {}})
        with self.assertRaises(ValueError):
            validate_fresh_observation(dict(observation, cameras=dict(observation["cameras"], front={"timestamp": 98.0})), now=now)

    def test_preflight_and_corridor_contracts_require_evidence(self):
        ready = validate_preflight({
            "physical_execution_ready": True, "timeout_policy_verified": True,
            "blockers": [], "qualification": {"scope": "offline"},
        })
        self.assertTrue(ready["qualified"])
        with self.assertRaises(ValueError):
            validate_preflight(dict(ready, blockers=["heartbeat_stale"]))
        self.assertTrue(validate_swept_corridors({
            "left": {"scene_version": "s-1", "clear": True},
            "right": {"scene_version": "s-1", "clear": True},
        }, scene_version="s-1", arms=("left", "right"))["clear"])
        with self.assertRaises(ValueError):
            validate_swept_corridors({"right": {"scene_version": "s-0", "clear": True}}, scene_version="s-1")

    def test_pair_admission_requires_common_duration_and_held_null(self):
        duration = synchronize_duration({"duration_s": .4}, {"duration_s": .7})
        self.assertEqual(duration["common_duration_s"], .7)
        admission = validate_pair_admission(
            {"action": "move_eef", "observation_id": "s-1"}, None,
            observation_id="s-1", duration_receipt=duration, held_sides=("right",),
            held_receipts={"right": {"arm": "right", "observation_id": "s-1", "sequence": 7,
                                      "owner": "host", "stationary": True, "hold_verified": True}},
            corridor_receipt={"scene_version": "s-1", "clear": True, "arms": ["left", "right"]})
        self.assertEqual(admission["dispatch_barrier"], "shared")
        with self.assertRaises(ValueError):
            validate_pair_admission({"action": "move_eef", "observation_id": "s-0"}, None,
                observation_id="s-1", duration_receipt=duration, held_sides=("right",))

    def test_pair_does_not_admit_two_simultaneous_task_moves(self):
        with self.assertRaisesRegex(ValueError, "exactly one moving arm"):
            validate_pair_admission(
                {"action": "move_eef", "observation_id": "s-1"},
                {"action": "move_eef", "observation_id": "s-1"},
                observation_id="s-1", duration_receipt={"common_duration_s": .7},
                corridor_receipt={"scene_version": "s-1", "clear": True, "arms": ["left", "right"]})

    def test_pair_receipt_and_fault_latch_are_independent_and_irreversible(self):
        receipt = validate_pair_receipt({
            "cycle_id": "c-1", "arms": {
                "left": {"sequence": 1, "accepted": True, "arrival_confirmed": True, "tracking_confirmed": True},
                "right": {"sequence": 2, "accepted": True, "stability_confirmed": True, "tracking_confirmed": True},
            },
        })
        self.assertEqual(receipt["sequences"], {"left": 1, "right": 2})
        fault = latch_pair_fault({}, "right receipt uncertain")
        self.assertTrue(fault["latched"])
        self.assertIs(latch_pair_fault({"fault_latch": fault}, "new reason"), fault)
        with self.assertRaises(ValueError):
            validate_pair_receipt({"cycle_id": "c-1", "arms": {
                "left": {"sequence": 1, "accepted": True, "arrival_confirmed": True, "tracking_confirmed": True},
                "right": {"sequence": 2, "accepted": True, "arrival_confirmed": True, "tracking_confirmed": False},
            }})

    def test_session_handoff_return_and_visual_evidence(self):
        self.assertTrue(validate_session_renewal({"renewed": True, "heartbeat": True, "arms": {"right": True}})["renewed"])
        hold = {"right": {"moving": False, "control_state": "holding", "owner": "host",
                           "pose_m_rad": [0] * 6, "joints_rad": [0] * 6, "commanded_joints_rad": [0] * 6}}
        self.assertTrue(validate_hold_handoff(hold, hold, owner="host")["handoff_ready"])
        self.assertTrue(validate_return(
            hold, {"right": dict(hold["right"], pose_m_rad=[.01, 0, 0, 0, 0, 0])},
        )["return_verified"])
        evidence = [{"stage": stage, "confirmed": True, "source": "model_visual_report",
                     "observation_id": "o-" + str(index), "at": index + 1.0}
                    for index, stage in enumerate(("grasp", "lift", "release", "stable"))]
        self.assertTrue(validate_task_evidence(evidence)["task_evidence_verified"])
        with self.assertRaises(ValueError):
            validate_task_evidence(evidence[:2])

    def test_runtime_rejects_unimplemented_modes_arms_and_tasks(self):
        self.assertEqual(validate_runtime_config({})["worker_arm"], "right")
        for config in ({"task_id": "charger"}, {"worker_arm": "left"},
                       {"pipeline_id": DUAL_ARM_PIPELINE_ID, "observer_arm": "left"}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                validate_runtime_config(config)

    def test_invalid_duration_and_missing_pair_guards_are_rejected(self):
        for value in (0, float("nan"), float("inf"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                synchronize_duration({"duration_s": value}, {"duration_s": 1})
        with self.assertRaises(ValueError):
            validate_pair_admission({"action": "move_eef", "observation_id": "s"}, None,
                                    observation_id="s", held_sides=("right",))

    def test_visual_evidence_cannot_reuse_frame_or_timestamp(self):
        good = [{"stage": stage, "confirmed": True, "source": "model_visual_report",
                 "observation_id": str(i), "at": i + 1.0}
                for i, stage in enumerate(("grasp", "lift", "release", "stable"))]
        for field, value in (("observation_id", "0"), ("at", 1.0), ("at", float("nan"))):
            bad = [dict(item) for item in good]
            bad[1][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_task_evidence(bad)

    def test_hold_and_return_require_full_measured_and_commanded_joints(self):
        arm = dict(moving=False, control_state="holding", owner="host", pose_m_rad=[0] * 6,
                   joints_rad=[0] * 6, commanded_joints_rad=[0] * 6)
        for field, value in (("pose_m_rad", [0] * 3), ("joints_rad", [0] * 5),
                             ("commanded_joints_rad", [.2] * 6)):
            for validate in (validate_hold_handoff, validate_return):
                with self.subTest(field=field, validate=validate.__name__), self.assertRaises(ValueError):
                    validate({"right": arm}, {"right": dict(arm, **{field: value})})

    def test_hold_position_and_rotation_use_separate_units(self):
        arm = dict(moving=False, control_state="holding", owner="host", pose_m_rad=[0] * 6,
                   joints_rad=[0] * 6, commanded_joints_rad=[0] * 6)
        self.assertTrue(validate_hold_handoff({"right": arm},
            {"right": dict(arm, pose_m_rad=[0, 0, 0, .04, 0, 0])})["handoff_ready"])
        with self.assertRaises(ValueError):
            validate_hold_handoff({"right": arm}, {"right": dict(arm, pose_m_rad=[.04, 0, 0, 0, 0, 0])})

    def test_held_side_requires_an_independent_current_receipt(self):
        args = dict(observation_id="s", held_sides=("right",),
                    corridor_receipt={"scene_version": "s", "clear": True, "arms": ["left", "right"]},
                    duration_receipt={"common_duration_s": .5})
        for held in (None, {"right": {"arm": "right", "observation_id": "old", "sequence": 1,
                                     "owner": "host", "stationary": True, "hold_verified": True}}):
            with self.subTest(held=held), self.assertRaises(ValueError):
                validate_pair_admission({"action": "move_eef", "observation_id": "s"}, None,
                                        held_receipts=held, **args)


if __name__ == "__main__":
    unittest.main()
