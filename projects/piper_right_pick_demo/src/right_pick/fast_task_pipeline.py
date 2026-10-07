"""L2 compositions and L3 ARX5-derived task contracts, with no device imports.

The bounded ledger validates offline host events, not physical qualification.
Only the existing pen phase runner has an adapter; recipes never send commands.
"""
import argparse
from dataclasses import dataclass
import json
import math
import time

from .fast_pipeline import (ATOMIC_SKILLS, PipelineContractError, atomic_skill,
                            pipeline_contract, validate_mode_config)


@dataclass(frozen=True)
class Composition:
    name: str
    evidence: tuple
    max_cycles: int = 4
    requirements: tuple = ()


COMPOSITIONS = (
    Composition("inspect", ("scene_identified",), 2, ("initial_condition_frozen", "run_reference_saved")),
    Composition("approach", ("approach_clear",)),
    Composition("align", ("alignment_visible",)),
    Composition("grip_test", ("object_retained", "support_separated"), 6),
    Composition("grip_supported", ("bilateral_contact", "support_unchanged")),
    Composition("validate_preheld", ("preheld_object_retained",), 2),
    Composition("transport", ("object_retained", "destination_visible"), 6),
    Composition("lower_to_support", ("object_on_original_support", "support_established")),
    Composition("hang_supported", ("hat_on_specified_hook", "rack_stable", "support_established"), 8),
    Composition("insert_segment", ("axial_progress", "support_established"), 8,
                ("contact_step_supported",)),
    Composition("rotate_segment", ("object_relative_rotation", "thread_progress"), 16,
                ("axis_and_support_visible", "contact_step_supported")),
    Composition("extract_segment", ("object_relative_progress", "peer_stable"), 8,
                ("axis_and_support_visible",)),
    Composition("pour_segment", ("flow_received", "receiver_stable"), 8,
                ("spill_and_residue_policy_frozen",)),
    Composition("wipe_segment", ("tool_surface_sliding",), 8,
                ("tool_contact_visible",)),
    Composition("sweep_segment", ("all_targets_inside",), 8,
                ("extended_tool_corridor_checked",)),
    Composition("push_segment", ("object_relative_progress",), 8,
                ("contact_surface_visible",)),
    Composition("release_retreat", ("support_established", "empty_gripper_clear"), 6),
    Composition("stable_verify", ("independent_stability",), 3),
    Composition("return_reference", ("run_reference_verified", "stationary"), 8,
                ("immutable_run_reference",)),
    Composition("observer_reposition", ("view_improved", "worker_stationary"), 2,
                ("worker_hold_verified", "observer_view_only", "pair_host_qualified")),
)
_COMPOSITIONS = {item.name: item for item in COMPOSITIONS}
_COMPOSITION_PRIMITIVES = {
    "inspect": ("observe_scene",), "approach": ("move_eef_once",),
    "align": ("move_eef_once",), "grip_test": ("set_gripper_once", "move_eef_once"),
    "grip_supported": ("set_gripper_once",), "validate_preheld": ("observe_scene",),
    "transport": ("move_eef_once",), "lower_to_support": ("move_eef_once",),
    "hang_supported": ("move_eef_once",), "insert_segment": ("move_eef_once",),
    "rotate_segment": ("move_eef_once",), "extract_segment": ("move_eef_once",),
    "pour_segment": ("move_eef_once",), "wipe_segment": ("move_eef_once",),
    "sweep_segment": ("move_eef_once",), "push_segment": ("move_eef_once",),
    "release_retreat": ("set_gripper_once", "move_eef_once"),
    "stable_verify": ("sample_stability",),
    "return_reference": ("move_eef_once", "verify_return"),
    "observer_reposition": ("prepare_held_side", "move_eef_once"),
}


@dataclass(frozen=True)
class TaskStep:
    skill: str
    arm: str = "worker"
    evidence: tuple = ()


@dataclass(frozen=True)
class TaskRecipe:
    task_id: str
    steps: tuple
    initial_condition: str
    goal: str
    constraints: tuple = ()
    dual_required: bool = False
    source_task: str = None


def _step(skill, arm="worker", evidence=()):
    return TaskStep(skill, arm, evidence)


def _pick_place(middle="insert_segment", *, preheld=False, lift_range=False):
    grasp = (_step("validate_preheld"),) if preheld else (
        _step("approach"), _step("align"), _step("grip_test", evidence=
            ("object_retained", "support_separated", "lift_range_verified") if lift_range else ()))
    return (_step("inspect"),) + grasp + (_step("transport"), _step("align"),
        _step(middle), _step("release_retreat"), _step("stable_verify"), _step("return_reference"))


TASK_RECIPES = (
    TaskRecipe("cups", (
        _step("inspect"), _step("approach", "left"), _step("align", "left"),
        _step("grip_test", "left"), _step("transport", "left"),
        _step("release_retreat", "left"), _step("stable_verify", "left"),
        _step("return_reference", "left", ("run_reference_verified", "peer_corridor_clear")),
        _step("approach", "right"), _step("align", "right"), _step("grip_test", "right"),
        _step("transport", "right"), _step("align", "right"), _step("insert_segment", "right"),
        _step("release_retreat", "right"), _step("stable_verify", "right"),
        _step("return_reference", "right")),
        "two identified compatible cups on support", "independent stable nested cups",
        ("one_arm_moves_at_a_time", "left_full_arm_clears_right_corridor"), True),
    TaskRecipe("pen", _pick_place(), "pen on visible support", "pen retained in specified holder",
        ("do_not_replace_thin_pen_with_marker", "do_not_replace_holder_with_wide_cup")),
    TaskRecipe("charger", _pick_place(), "charger on support, specified socket disconnected from mains",
        "correct mechanical insertion, released and stable", ("power_disconnected_confirmed",)),
    TaskRecipe("charger-insert-only", (_step("inspect"), _step("approach"), _step("align"),
        _step("grip_supported"), _step("insert_segment"), _step("release_retreat"),
        _step("stable_verify"), _step("return_reference")),
        "manually aligned or shallowly inserted charger", "robot-added insertion relative to initial state",
        ("power_disconnected_confirmed", "no_test_lift_or_claim_of_autonomous_pick")),
    TaskRecipe("flower", _pick_place(preheld=True), "manually preheld flower, specified receiver",
        "flower independently supported in specified receiver",
        ("preheld_not_autonomous_pick", "freeze_receiver_and_stem_orientation")),
    TaskRecipe("hat", _pick_place("hang_supported"), "hat on support and identified lowest actual hook",
        "hat independently hangs on specified hook", ("rack_must_not_move",)),
    TaskRecipe("pen-uncapping", (_step("inspect"), _step("validate_preheld", "right"),
        _step("approach", "left"), _step("align", "left"), _step("grip_supported", "left"),
        _step("extract_segment", "right", ("cap_body_separated", "cap_retained", "body_retained")),
        _step("transport", "left"), _step("release_retreat", "left"), _step("stable_verify", "left"),
        _step("transport", "right"), _step("release_retreat", "right"), _step("stable_verify", "right"),
        _step("return_reference", "left"), _step("return_reference", "right")),
        "right preholds pen; initial cap tightness and manual help recorded", "cap and body independently separated",
        ("one_arm_moves_at_a_time", "supported_cleanup_after_separation"), True),
    TaskRecipe("pearl-pouring", (_step("inspect"), _step("approach"), _step("align"),
        _step("grip_test"), _step("transport"), _step("align"), _step("pour_segment"),
        _step("transport"), _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "identified source contents and receiver", "transfer per frozen spill and residue policy",
        ("stop_flow_before_reposition", "do_not_infer_received_quantity_from_tilt")),
    TaskRecipe("orange-pick-place", _pick_place("lower_to_support", lift_range=True), "specified whole fruit or segment on original support",
        "lift per frozen range, then replace stably", ("soft_object_no_force_escalation",)),
    TaskRecipe("drawer-push-pull", (_step("inspect"), _step("approach"), _step("align"),
        _step("grip_supported"), _step("extract_segment", evidence=("drawer_relative_travel", "cabinet_stable")),
        _step("push_segment", evidence=("initial_opening_restored", "cabinet_stable")),
        _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "specified partly open drawer, initial opening saved", "pull specified travel then restore initial opening",
        ("drawer_motion_not_tcp_motion", "do_not_force_fully_closed")),
    TaskRecipe("jelly-pick-place", _pick_place("lower_to_support", lift_range=True), "specified soft jelly on original support",
        "specified lift and stable replacement", ("soft_object_no_force_escalation",)),
    TaskRecipe("bottle-unscrewing", (_step("inspect"), _step("approach", "left"),
        _step("align", "left"), _step("grip_test", "left"), _step("approach", "right"),
        _step("align", "right"), _step("grip_supported", "right"), _step("rotate_segment", "right"),
        _step("extract_segment", "right", ("threads_disengaged", "cap_separation_verified", "bottle_stable")),
        _step("transport", "right"), _step("release_retreat", "right"), _step("stable_verify", "right"),
        _step("transport", "left"), _step("release_retreat", "left"), _step("stable_verify", "left"),
        _step("return_reference", "right"), _step("return_reference", "left")),
        "left stabilizes bottle, right rotates cap", "threads disengaged and cap separated per frozen goal",
        ("one_arm_moves_at_a_time", "support_grasp_does_not_prove_torque_capacity"), True),
    TaskRecipe("fixed-bottle-unscrewing", (_step("inspect"), _step("approach"), _step("align"),
        _step("grip_supported"), _step("rotate_segment"),
        _step("extract_segment", evidence=("threads_disengaged", "cap_separation_verified", "fixture_stable")),
        _step("transport"), _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "bottle externally fixed; worker rotates cap; peer view only", "cap separated per frozen goal",
        ("fixture_resists_rotation_verified", "observer_must_not_grip_bottle")),
    TaskRecipe("bolt-screwing", (_step("inspect"), _step("validate_preheld", "right"),
        _step("approach", "left"), _step("align", "left"), _step("grip_supported", "left"),
        _step("rotate_segment", "left", ("three_actual_thread_turns", "axial_feed_verified", "bolt_stable")),
        _step("release_retreat", "left"), _step("stable_verify", "left"),
        _step("transport", "right"), _step("release_retreat", "right"), _step("stable_verify", "right"),
        _step("return_reference", "left"), _step("return_reference", "right")),
        "right preholds bolt, left turns nut", "three actual nut turns relative to bolt",
        ("one_arm_moves_at_a_time", "wrist_angle_is_not_thread_turn_count"), True),
    TaskRecipe("book-extraction", (_step("inspect"), _step("approach"), _step("align"),
        _step("grip_supported"), _step("extract_segment", evidence=("new_book_travel_verified", "other_books_stable")),
        _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "identified middle book with initial protrusion saved", "additional specified book extraction",
        ("initial_protrusion_not_new_progress", "observer_must_not_support_stack")),
    TaskRecipe("blackboard-wiping", (_step("inspect"), _step("validate_preheld"),
        _step("transport"), _step("align"), _step("wipe_segment"),
        _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "preheld wiping tool and identified board; autonomous pickup requires a separate variant",
        "frozen wiping-motion or erasure criterion", ("motion_not_erasure", "observer_must_not_hold_board")),
    TaskRecipe("blue-blocks-sweeping", (_step("inspect"), _step("validate_preheld"),
        _step("transport"), _step("align"), _step("sweep_segment"), _step("stable_verify"),
        _step("transport"), _step("release_retreat"), _step("stable_verify"), _step("return_reference")),
        "preplaced broom, all target blocks counted, identified dustpan", "all blocks inside after broom withdrawal",
        ("preplaced_not_autonomous_pick", "observer_must_not_hold_dustpan")),
    TaskRecipe("blue-block-triangle-push", (_step("inspect"), _step("approach"), _step("align"),
        _step("push_segment", evidence=("block_inside_region", "edge_push_without_grasp")),
        _step("stable_verify"), _step("return_reference")),
        "block and red triangle boundary visible", "block stable inside after empty gripper withdrawal",
        ("edge_push_only_no_pick_place", "no_extra_centering_requirement")),
)
_TASKS = {recipe.task_id: recipe for recipe in TASK_RECIPES}


def composition_contract(name, mode="single_arm"):
    try:
        item = _COMPOSITIONS[name]
    except (KeyError, TypeError):
        raise PipelineContractError("Unknown composition: " + str(name))
    cycle = pipeline_contract(mode)
    primitives = _COMPOSITION_PRIMITIVES[name]
    for primitive in primitives:
        atomic_skill(primitive)
    return {"id": name, "layer": 2, "atomic_cycle": cycle["stages"],
            "primitives": list(primitives),
            "guards": cycle["lifecycle"]["guards"], "evidence": list(item.evidence),
            "requirements": list(item.requirements), "max_cycles": item.max_cycles,
            "max_dispatches_per_cycle": 1, "min_stability_gap_s": 2 if name == "stable_verify" else None,
            "implementation": "offline_contract_only"}


def task_contract(task_id, *, mode="single_arm", worker_arm="right"):
    try:
        recipe = _TASKS[task_id]
    except (KeyError, TypeError):
        raise PipelineContractError("Unknown task: " + str(task_id))
    cycle = pipeline_contract(mode)
    mode = cycle["execution_mode"]
    if recipe.dual_required and mode != "dual_arm":
        raise PipelineContractError("This task needs two task arms, not a view-only observer")
    peer = "left" if worker_arm == "right" else "right"
    roles = validate_mode_config(dict(pipeline_id=cycle["id"], worker_arm=worker_arm,
        observer_arm=peer if mode == "worker_with_observer" else None,
        peer_arm=peer if mode == "dual_arm" else None))
    steps = []
    for index, step in enumerate(recipe.steps):
        composite = composition_contract(step.skill, mode)
        steps.append({"id": str(index) + ":" + step.skill, "skill": step.skill,
            "arm": worker_arm if step.arm == "worker" else step.arm,
            "evidence": list(dict.fromkeys(tuple(composite["evidence"]) + step.evidence)),
            "requirements": composite["requirements"],
            "max_cycles": composite["max_cycles"]})
    return {"id": task_id + "_v1", "task_id": task_id, "layer": 3, "roles": roles,
            "initial_condition": recipe.initial_condition, "goal": recipe.goal,
            "constraints": list(recipe.constraints), "steps": steps,
            "source": "GPT6-ARX5/skills/arx-r5-tabletop-tasks/references/" + (recipe.source_task or task_id) + ".md",
            "dispatch_policy": "one_arm_moves_at_a_time",
            "implementation": "offline_contract_only", "execution_available": False,
            "budget": {"max_cycles": 128, "max_model_calls": 64, "max_elapsed_s": 900,
                       "max_no_progress": 2, "max_rejections": 1, "max_observer_moves": 2},
            "observer_branch": "observer_reposition" if mode == "worker_with_observer" else None,
            "finish": "task_evidence_and_run_reference_separately_verified"}


class BoundedTaskPipeline:
    """Offline event ledger. Host receipts enter here; no command can leave it."""
    def __init__(self, task_id, *, mode="single_arm", worker_arm="right", budget=None, clock=time.monotonic):
        self.contract = task_contract(task_id, mode=mode, worker_arm=worker_arm)
        self.budget = dict(self.contract["budget"])
        self.budget.update(budget or {})
        if set(self.budget) != set(self.contract["budget"]) or any(
                type(v) is not int or v <= 0 for v in self.budget.values()):
            raise PipelineContractError("Budgets must be known positive integer fields")
        self.clock, self.started_at = clock, clock()
        self.index = self.cycles = self.model_calls = self.stage_cycles = 0
        self.no_progress = self.rejections = 0
        self.last_at = self.last_observation_id = None
        self.observations, self.evidence = set(), []
        self.termination_reason = None
        self.observing = False
        self.observer_moves = self.observer_cycles = 0
        self.stage_measurements, self.observer_measurements = {}, {}

    def _check_deadline(self):
        if self.termination_reason is None and self.clock() - self.started_at >= self.budget["max_elapsed_s"]:
            self.termination_reason = "wall_time_budget_exhausted"

    def current(self):
        self._check_deadline()
        if self.termination_reason is not None or self.index == len(self.contract["steps"]):
            return None
        step = self.contract["steps"][self.index]
        used_cycles = self.stage_cycles
        if self.observing:
            step = {"id": "view:" + str(self.observer_moves), "skill": "observer_reposition",
                    "arm": self.contract["roles"]["observer_arm"],
                    "evidence": list(_COMPOSITIONS["observer_reposition"].evidence), "max_cycles": 2,
                    "requirements": list(_COMPOSITIONS["observer_reposition"].requirements)}
            used_cycles = self.observer_cycles
        return {"task": self.contract["id"], "stage": step["id"], "skill": step["skill"],
                "arm": step["arm"], "expect": list(step["evidence"]),
                "needs": list(step["requirements"]),
                "cycles_left": min(step["max_cycles"] - used_cycles, self.budget["max_cycles"] - self.cycles),
                "model_calls_left": self.budget["max_model_calls"] - self.model_calls}

    def current_operation(self):
        """Expand only the active L2 operation and its L1 names on demand."""
        stage = self.current()
        if stage is None:
            return None
        return dict(stage=stage["stage"], arm=stage["arm"],
                    cycles_left=stage["cycles_left"],
                    contract=composition_contract(stage["skill"], self.contract["roles"]["execution_mode"]))

    def _measured_progress(self, receipt, observation):
        change = receipt.get("progress_measurement")
        if not isinstance(change, dict) or set(change) != {"metric", "unit", "before", "after", "observation_id"}:
            return False
        metric, unit = change["metric"], change["unit"]
        before, after = change["before"], change["after"]
        if (not isinstance(metric, str) or not 0 < len(metric) <= 64
                or not isinstance(unit, str) or not 0 < len(unit) <= 32
                or change["observation_id"] != observation
                or type(before) not in (int, float) or type(after) not in (int, float)
                or not math.isfinite(before) or not math.isfinite(after)
                or math.isclose(before, after, rel_tol=1e-6, abs_tol=1e-9)):
            return False
        measurements = self.observer_measurements if self.observing else self.stage_measurements
        previous = measurements.get((metric, unit))
        direction = 1 if after > before else -1
        if previous is not None and (not math.isclose(before, previous[0], rel_tol=1e-6, abs_tol=1e-9)
                                     or direction != previous[1]):
            return False
        measurements[(metric, unit)] = (after, direction)
        return True

    def request_observer_view(self, hold_receipt):
        """Branch only after the host proves a stationary worker in the latest scene."""
        if self.current() is None or self.contract["observer_branch"] is None or self.observing:
            raise PipelineContractError("Observer branch is unavailable")
        if (not isinstance(hold_receipt, dict)
                or hold_receipt.get("arm") != self.contract["roles"]["worker_arm"]
                or hold_receipt.get("stationary") is not True or hold_receipt.get("hold_verified") is not True
                or self.last_at is None or hold_receipt.get("observation_id") != self.last_observation_id
                or hold_receipt.get("at") != self.last_at):
            raise PipelineContractError("Observer motion requires latest verified worker hold")
        if self.observer_moves >= self.budget["max_observer_moves"]:
            self.termination_reason = "observer_budget_exhausted"
            return self.report()
        self.observer_moves += 1
        self.observer_cycles = 0
        self.observer_measurements = {}
        self.observing = True
        return self.report()

    def record_cycle(self, receipt):
        current = self.current()
        if current is None:
            raise PipelineContractError("Pipeline is terminated")
        if not isinstance(receipt, dict):
            raise PipelineContractError("Host receipt must be an object")
        if any(receipt.get(key) != current[key] for key in ("task", "stage", "arm")):
            raise PipelineContractError("Host receipt does not bind the active task/stage/arm")
        status = receipt.get("status")
        if status not in ("complete", "progress", "unknown", "rejected", "fault"):
            raise PipelineContractError("Unknown host status")
        if type(receipt.get("model_called")) is not bool:
            raise PipelineContractError("model_called must be a host boolean")
        observation = receipt.get("observation_id")
        stamp = receipt.get("at")
        if (not isinstance(observation, str) or not observation or observation in self.observations
                or type(stamp) not in (int, float) or not math.isfinite(stamp)
                or self.last_at is not None and stamp <= self.last_at):
            raise PipelineContractError("Cycle needs a distinct observation and advancing finite timestamp")
        for flag in ("partial_send", "outcome_uncertain"):
            if flag in receipt and type(receipt[flag]) is not bool:
                raise PipelineContractError(flag + " must be a host boolean")
        if receipt.get("partial_send") is True or receipt.get("outcome_uncertain") is True or status == "fault":
            # A bound fault is terminal even when no usable success evidence exists.
            self.termination_reason = "execution_failed_latched"
            self.observations.add(observation)
            self.last_at, self.last_observation_id = stamp, observation
            self.cycles += 1
            self.model_calls += int(receipt["model_called"])
            return self.report()
        facts = receipt.get("evidence", [])
        if not isinstance(facts, list) or any(not isinstance(f, str) for f in facts):
            raise PipelineContractError("Evidence must be host-validated fact names")
        if status == "complete" and not set(current["expect"]).issubset(facts):
            raise PipelineContractError("Cannot advance without expected evidence")
        prerequisites = receipt.get("prerequisites", [])
        if (not isinstance(prerequisites, list) or any(not isinstance(f, str) for f in prerequisites)
                or not set(current["needs"]).issubset(prerequisites)):
            raise PipelineContractError("Composition prerequisites need host validation")
        sample_ids = []
        if status == "complete" and current["skill"] == "stable_verify":
            samples = receipt.get("stability_samples")
            if not isinstance(samples, list) or len(samples) != 2 or any(not isinstance(s, dict) for s in samples):
                raise PipelineContractError("Stability requires two new host samples")
            first, last = samples
            sample_ids = [s.get("observation_id") for s in samples]
            times = [s.get("at") for s in samples]
            if (any(not isinstance(s, str) or not s or s in self.observations for s in sample_ids)
                    or sample_ids[0] == sample_ids[1] or sample_ids[-1] != observation
                    or any(type(t) not in (int, float) or not math.isfinite(t) for t in times)
                    or times[-1] != stamp or times[-1] - times[0] < 2
                    or self.last_at is not None and times[0] <= self.last_at
                    or any(s.get("support_stable") is not True or s.get("gripper_clear") is not True for s in samples)):
                raise PipelineContractError("Stability samples must be distinct, separated, post-release and supported")
        measured_progress = status == "progress" and self._measured_progress(receipt, observation)
        self.observations.update(sample_ids)
        self.observations.add(observation)
        self.last_at, self.last_observation_id = stamp, observation
        self.cycles += 1
        if self.observing:
            self.observer_cycles += 1
        else:
            self.stage_cycles += 1
        self.model_calls += int(receipt["model_called"])
        self.no_progress = 0 if status == "complete" or measured_progress else self.no_progress + 1
        self.rejections += int(status == "rejected")
        if self.termination_reason is None and status == "complete":
            self.evidence.append({"stage": current["stage"], "observation_id": observation,
                                  "at": stamp, "facts": list(facts)})
            if self.observing:
                self.observing = False
                self.observer_measurements = {}
            else:
                self.index += 1
                self.stage_cycles = 0
                self.stage_measurements = {}
                if self.index == len(self.contract["steps"]):
                    self.termination_reason = "offline_contract_completed"
        if self.termination_reason is None:
            limits = ((self.no_progress >= self.budget["max_no_progress"], "no_progress_budget_exhausted"),
                      (self.rejections >= self.budget["max_rejections"], "rejection_budget_exhausted"),
                      (self.cycles >= self.budget["max_cycles"], "cycle_budget_exhausted"),
                      (self.model_calls >= self.budget["max_model_calls"], "model_call_budget_exhausted"),
                      (self.observing and self.observer_cycles >= 2, "observer_stage_budget_exhausted"),
                      (not self.observing and self.stage_cycles >= self.contract["steps"][self.index]["max_cycles"], "stage_budget_exhausted"))
            self.termination_reason = next((label for exceeded, label in limits if exceeded), None)
        return self.report()

    def report(self):
        self._check_deadline()
        return {"task": self.contract["id"], "next": self.current(), "cycles": self.cycles,
                "model_calls": self.model_calls, "termination_reason": self.termination_reason,
                "contract_completed": self.termination_reason == "offline_contract_completed",
                "physical_success_measurable": False, "execution_available": False}


_PEN_COMPOSITIONS = dict(INIT="inspect", APPROACH_PEN="approach", ALIGN_PEN="align",
    PREGRASP="align", GRASP="grip_test", VERIFY_GRASP="grip_test", LIFT="grip_test",
    APPROACH_HOLDER="transport", ALIGN_HOLDER="align", INSERT="insert_segment",
    RELEASE="release_retreat", VERIFY_SUCCESS="stable_verify", DONE="stable_verify", RECOVERY="inspect")


def pen_phase_contract(phase):
    """Project the existing pen executor onto L2 without replacing its phase guards."""
    try:
        skill = _PEN_COMPOSITIONS[phase]
    except (KeyError, TypeError):
        raise PipelineContractError("Unknown pen phase")
    return {"task": "pen_v1", "skill": skill, "phase": phase}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline L2/L3 planner; no robot, camera or model calls")
    parser.add_argument("--task", choices=tuple(_TASKS), default="pen")
    parser.add_argument("--mode", choices=("single_arm", "dual_arm", "worker_with_observer"), default="single_arm")
    parser.add_argument("--worker-arm", choices=("left", "right"), default="right")
    parser.add_argument("--catalog", action="store_true")
    parser.add_argument("--compact", action="store_true")
    parser.add_argument("--composition", choices=tuple(_COMPOSITIONS))
    parser.add_argument("--atomic", choices=tuple(s.name for s in ATOMIC_SKILLS))
    args = parser.parse_args(argv)
    try:
        if args.catalog:
            value = {"tasks": list(_TASKS), "compositions": list(_COMPOSITIONS),
                     "atomic_skills": [s.name for s in ATOMIC_SKILLS], "execution_available": False}
        elif args.atomic:
            value = dict(atomic_skill(args.atomic).compact(), implementation="contract_only")
        elif args.composition:
            value = composition_contract(args.composition, args.mode)
        elif args.compact:
            value = BoundedTaskPipeline(args.task, mode=args.mode, worker_arm=args.worker_arm).report()
        else:
            value = task_contract(args.task, mode=args.mode, worker_arm=args.worker_arm)
    except PipelineContractError as exc:
        parser.error(str(exc))
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
