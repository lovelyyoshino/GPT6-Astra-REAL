"""Declarative, portable task recipes. This module cannot dispatch actions.

Recipes describe local goals and evidence requirements; they never contain
robot poses, executable Python, or site qualification. New recipes reuse the
same bounded task ledger instead of creating another object-specific runner.
"""
import copy
import json
import re
from pathlib import Path

from .fast_pipeline import PipelineContractError, pipeline_contract, validate_mode_config


def _text(value, field, limit=400):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise PipelineContractError(field + " must be nonempty bounded text")
    return value


def _names(value, field, limit=16):
    if not isinstance(value, list) or len(value) > limit:
        raise PipelineContractError(field + " must be a bounded list")
    result = [_text(item, field, 160) for item in value]
    if len(set(result)) != len(result):
        raise PipelineContractError(field + " contains duplicates")
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PipelineContractError("Duplicate recipe field: " + key)
        result[key] = value
    return result


def load_recipe(path, *, mode="single_arm", worker_arm="right"):
    path = Path(path)
    if path.stat().st_size > 65536:
        raise PipelineContractError("Recipe exceeds 64 KiB")
    value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    # Validate now; the caller still freezes the returned source in its journal.
    recipe_contract(value, mode=mode, worker_arm=worker_arm)
    return value


def recipe_contract(definition, *, mode="single_arm", worker_arm="right"):
    from .fast_task_pipeline import composition_contract

    if not isinstance(definition, dict):
        raise PipelineContractError("Recipe must be a JSON object")
    allowed = {"schema_version", "task_id", "initial_condition", "goal", "constraints", "steps"}
    if set(definition) != allowed or definition.get("schema_version") != "astra_recipe_v1":
        raise PipelineContractError("Recipe requires the exact astra_recipe_v1 fields")
    task_id = definition["task_id"]
    if not isinstance(task_id, str) or not re.fullmatch(r"[a-z][a-z0-9_-]{0,63}", task_id):
        raise PipelineContractError("task_id must be a lowercase identifier")
    goal = _text(definition["goal"], "goal", 800)
    initial = _text(definition["initial_condition"], "initial_condition", 800)
    constraints = _names(definition["constraints"], "constraints")
    cycle = pipeline_contract(mode)
    mode = cycle["execution_mode"]
    peer = "left" if worker_arm == "right" else "right"
    roles = validate_mode_config(dict(pipeline_id=cycle["id"], worker_arm=worker_arm,
        observer_arm=peer if mode == "worker_with_observer" else None,
        peer_arm=peer if mode == "dual_arm" else None))
    source_steps = definition["steps"]
    if not isinstance(source_steps, list) or not 2 <= len(source_steps) <= 64:
        raise PipelineContractError("Recipe needs 2..64 bounded operation stages")
    steps, identifiers = [], set()
    for index, step in enumerate(source_steps):
        required = {"id", "operation", "goal"}
        optional = {"arm", "evidence", "max_cycles"}
        if not isinstance(step, dict) or not required <= set(step) or set(step) - required - optional:
            raise PipelineContractError("Unknown or missing stage fields; action arguments/code are forbidden")
        identifier = step["id"]
        if (not isinstance(identifier, str) or not re.fullmatch(r"[a-z][a-z0-9_-]{0,63}", identifier)
                or identifier in identifiers):
            raise PipelineContractError("Stage identifiers must be unique lowercase identifiers")
        identifiers.add(identifier)
        operation = step["operation"]
        contract = composition_contract(operation, mode)
        if operation == "observer_reposition":
            raise PipelineContractError("Observer motion uses the existing verified-hold branch")
        arm = step.get("arm", "worker")
        arm = worker_arm if arm == "worker" else arm
        if arm not in ("left", "right") or (mode != "dual_arm" and arm != worker_arm):
            raise PipelineContractError("Stage cannot assign task work to the passive or observer arm")
        maximum = step.get("max_cycles", contract["max_cycles"])
        if type(maximum) is not int or not 1 <= maximum <= contract["max_cycles"]:
            raise PipelineContractError("A recipe may reduce, never enlarge, operation cycle limits")
        facts = _names(step.get("evidence", []), "evidence")
        steps.append(dict(id=identifier, skill=operation, arm=arm,
            goal=_text(step["goal"], "stage goal", 400),
            evidence=list(dict.fromkeys(contract["evidence"] + facts)),
            requirements=contract["requirements"], max_cycles=maximum))
    if steps[0]["skill"] != "inspect":
        raise PipelineContractError("First stage must inspect the current scene")
    if not any(step["skill"] == "stable_verify" for step in steps):
        raise PipelineContractError("Task requires an explicit stable verification stage")
    for index, step in enumerate(steps):
        if step["skill"] == "release_retreat" and not any(
                later["skill"] == "stable_verify" for later in steps[index + 1:]):
            raise PipelineContractError("Every release requires a later independent stability stage")
    if steps[-1]["skill"] not in ("stable_verify", "return_reference"):
        raise PipelineContractError("Last stage must verify stability or separately verify return")
    last_verify = max(i for i, step in enumerate(steps) if step["skill"] == "stable_verify")
    if any(step["skill"] != "return_reference" for step in steps[last_verify + 1:]):
        raise PipelineContractError("Object operations after final stability require re-verification")
    return dict(id=task_id + "_v1", task_id=task_id, layer=3, roles=roles,
        initial_condition=initial, goal=goal, constraints=constraints, steps=steps,
        source="user_declarative_recipe", dispatch_policy="one_arm_moves_at_a_time",
        implementation="offline_contract_only", execution_available=False,
        budget=dict(max_cycles=128, max_model_calls=64, max_elapsed_s=900,
                    max_no_progress=2, max_rejections=1, max_observer_moves=2),
        observer_branch="observer_reposition" if mode == "worker_with_observer" else None,
        finish="task_evidence_and_run_reference_separately_verified")


def frozen_definition(definition, *, mode="single_arm", worker_arm="right"):
    recipe_contract(definition, mode=mode, worker_arm=worker_arm)
    return copy.deepcopy(definition)
