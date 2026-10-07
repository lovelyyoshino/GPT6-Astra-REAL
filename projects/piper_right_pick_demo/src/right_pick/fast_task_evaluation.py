"""Declarative, offline task evidence review; never a motion admission gate.

RGB judgments remain judgments. This module checks provenance labels, ordering,
distinct observations and declared success predicates, not image truth. It
does not open images, call a model, infer object coordinates or authenticate a
reviewer. Caller-supplied claims and reports must themselves be reviewed.
"""
import math

from .fast_pipeline import PipelineContractError


SCHEMA_VERSION = "task_evaluation_v1"
_VISUAL_SOURCES = ("model_visual_report", "independent_rgb_review")
_CHANNELS = _VISUAL_SOURCES + ("simulator_oracle",)
_SOURCES = _CHANNELS + ("robot_receipt", "human_intervention")
_MODES = ("physical", "offline_replay", "simulation")


def _text(value, name, limit=240):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise PipelineContractError(name + " must be a nonempty bounded string")
    return value


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise PipelineContractError(name + " must be finite and nonnegative")
    return value


def _strings(value, name, *, nonempty=False):
    if not isinstance(value, list) or nonempty and not value:
        raise PipelineContractError(name + " must be a list")
    for item in value:
        _text(item, name + " item")
    if len(set(value)) != len(value):
        raise PipelineContractError(name + " must not repeat entries")
    return value


def _object(value, name, *, required, optional=()):
    if (not isinstance(value, dict) or not set(required).issubset(value)
            or set(value) - set(required) - set(optional)):
        raise PipelineContractError(name + " has missing or unknown fields")
    return value


def _claim(claim_id, facts, *, after=(), events=(), samples=1, span=0, terminal=False):
    return dict(id=claim_id, required_facts=list(facts), after_claims=list(after),
                after_events=list(events), min_observations=samples,
                min_span_s=span, terminal=terminal)


def placement_evaluation_contract():
    """Example only: the caller must freeze the actual object/goal variant."""
    return [
        _claim("grasp", ("object_retained",)),
        _claim("lift", ("object_retained", "original_support_separated"), after=("grasp",)),
        _claim("release", ("object_released", "destination_support_established"),
               after=("lift",), events=("release_command",)),
        _claim("retreat", ("gripper_clear",), after=("release",), events=("retreat_command",)),
        _claim("stability", ("goal_satisfied", "independent_support", "gripper_clear"),
               after=("retreat",), samples=2, span=2, terminal=True),
    ]


def articulated_goal_evaluation_contract():
    """RGB-visible articulation change; simulated joint angles stay in oracle."""
    return [
        _claim("initial_condition", ("initial_condition_visible",)),
        _claim("articulated_goal", ("articulation_goal_visible",),
               after=("initial_condition",), samples=2, span=2, terminal=True),
    ]


def _validate(document):
    _object(document, "evaluation", required=("schema_version", "run_id", "task_id", "mode",
                                              "claims", "observations", "events"))
    if document["schema_version"] != SCHEMA_VERSION or document["mode"] not in _MODES:
        raise PipelineContractError("Unsupported evaluation schema or mode")
    for key in ("run_id", "task_id"):
        _text(document[key], key)
    if not isinstance(document["claims"], list) or not document["claims"]:
        raise PipelineContractError("Claims must be a nonempty list")
    claims = {}
    for item in document["claims"]:
        _object(item, "claim", required=("id", "required_facts", "after_claims", "after_events",
                                         "min_observations", "min_span_s", "terminal"))
        key = _text(item["id"], "claim id")
        if key in claims:
            raise PipelineContractError("Duplicate claim id")
        for name in ("required_facts", "after_claims", "after_events"):
            _strings(item[name], name, nonempty=name == "required_facts")
        if type(item["min_observations"]) is not int or item["min_observations"] < 1:
            raise PipelineContractError("min_observations must be a positive integer")
        _number(item["min_span_s"], "min_span_s")
        if type(item["terminal"]) is not bool:
            raise PipelineContractError("terminal must be boolean")
        claims[key] = item
    order, active = [], set()
    def visit(key):
        if key not in claims or key in active:
            raise PipelineContractError("Claims have a missing dependency or dependency cycle")
        if key in order:
            return
        active.add(key)
        for prior in claims[key]["after_claims"]:
            visit(prior)
        active.remove(key)
        order.append(key)
    for key in claims:
        visit(key)
    if not any(c["terminal"] for c in claims.values()):
        raise PipelineContractError("At least one terminal goal claim is required")
    if not isinstance(document["observations"], list) or not isinstance(document["events"], list):
        raise PipelineContractError("Observations and events must be lists")
    observations = {}
    for item in document["observations"]:
        _object(item, "observation", required=("id", "run_id", "at", "origin", "rgb_references"))
        key = _text(item["id"], "observation id")
        if key in observations or item["run_id"] != document["run_id"]:
            raise PipelineContractError("Duplicate observation or mismatched run")
        _number(item["at"], "observation at")
        if item["origin"] not in ("physical_rgb", "historical_rgb", "simulation_rgb"):
            raise PipelineContractError("Unknown observation origin")
        _strings(item["rgb_references"], "rgb_references")
        observations[key] = item
    events, used_frames = {}, set()
    for item in document["events"]:
        if not isinstance(item, dict) or item.get("source") not in _SOURCES:
            raise PipelineContractError("Unknown evidence source")
        source = item["source"]
        base = ("id", "run_id", "at", "source", "reference")
        extra = (("observation_id", "facts", "reviewer") if source == "independent_rgb_review" else
                 ("observation_id", "facts") if source in _CHANNELS else
                 ("outcome",) if source == "robot_receipt" else ("scope", "description"))
        _object(item, "event", required=base + extra)
        key = _text(item["id"], "event id")
        if key in events or item["run_id"] != document["run_id"]:
            raise PipelineContractError("Duplicate event or mismatched run")
        _text(item["reference"], "event reference", limit=2048)
        _number(item["at"], "event at")
        if source in _CHANNELS:
            _text(item["observation_id"], "event observation_id")
            observation = observations.get(item["observation_id"])
            if observation is None or item["at"] < observation["at"]:
                raise PipelineContractError("Evidence needs an existing observation captured before the report")
            frame_key = (source, item["observation_id"])
            if frame_key in used_frames:
                raise PipelineContractError("A source cannot count the same observation twice")
            used_frames.add(frame_key)
            if not isinstance(item["facts"], dict) or not item["facts"]:
                raise PipelineContractError("Visual/oracle facts must be nonempty")
            for fact, value in item["facts"].items():
                _text(fact, "fact name")
                if value is not None and type(value) is not bool:
                    raise PipelineContractError("Facts must be boolean or unknown; metric object state is not accepted")
            if source in _VISUAL_SOURCES and not observation["rgb_references"]:
                raise PipelineContractError("Visual evidence needs RGB references")
            if source == "independent_rgb_review":
                _text(item["reviewer"], "reviewer")
            if source == "simulator_oracle" and observation["origin"] != "simulation_rgb":
                raise PipelineContractError("Simulator oracle needs simulation origin")
        elif source == "robot_receipt":
            if item["outcome"] not in ("completed", "rejected", "uncertain", "fault"):
                raise PipelineContractError("Unknown robot receipt outcome")
        else:
            if item["scope"] not in ("setup", "task", "return"):
                raise PipelineContractError("Unknown human intervention scope")
            _text(item["description"], "intervention description")
        events[key] = item
    return claims, order, observations, events


def _evaluate_channel(source, claims, order, observations, events, *, physical_only=False):
    rows = [e for e in events.values() if e["source"] == source
            and (not physical_only or observations[e["observation_id"]]["origin"] == "physical_rgb")]
    rows.sort(key=lambda e: (observations[e["observation_id"]]["at"], e["at"], e["id"]))
    # Terminal truth must be re-observed after later physical changes. These
    # generic receipts have no trustworthy "could not affect the object" flag;
    # a later completed command or intervention therefore invalidates old views.
    scene_changes = [e["at"] for e in events.values()
                     if e["source"] == "human_intervention"
                     or e["source"] == "robot_receipt" and e["outcome"] == "completed"]
    evaluated = {}
    for key in order:
        claim = claims[key]
        result = dict(satisfied=None, observation_ids=[], event_ids=[], completed_at=None, reason="missing_evidence")
        evaluated[key] = result
        dependencies = [evaluated[p] for p in claim["after_claims"]]
        if any(p["satisfied"] is not True for p in dependencies):
            result["reason"] = "claim_dependency_unmet"
            continue
        required_events = [events.get(k) for k in claim["after_events"]]
        if any(e is None or e["source"] != "robot_receipt" or e["outcome"] != "completed"
               for e in required_events):
            result["reason"] = "execution_event_dependency_unmet"
            continue
        bounds = [p["completed_at"] for p in dependencies] + [e["at"] for e in required_events]
        if claim["terminal"]:
            bounds.extend(scene_changes)
        lower_bound = max(bounds) if bounds else -1
        candidates = []
        for row in rows:
            stamp = observations[row["observation_id"]]["at"]
            if stamp <= lower_bound or not set(claim["required_facts"]).intersection(row["facts"]):
                continue
            facts = [row["facts"].get(f) for f in claim["required_facts"]]
            if not all(v is True for v in facts):
                candidates = []
                result.update(satisfied=False if any(v is False for v in facts) else None,
                              observation_ids=[row["observation_id"]], event_ids=[row["id"]],
                              completed_at=None, reason="refuted" if any(v is False for v in facts) else "unknown")
                continue
            candidates.append(row)
            first = observations[candidates[0]["observation_id"]]["at"]
            complete = len(candidates) >= claim["min_observations"] and stamp - first >= claim["min_span_s"]
            result.update(satisfied=True if complete else None,
                          observation_ids=[e["observation_id"] for e in candidates],
                          event_ids=[e["id"] for e in candidates], completed_at=stamp if complete else None,
                          reason="evidence_complete" if complete else "insufficient_distinct_samples_or_span")
            if complete and not claim["terminal"]:
                break
    complete = all(item["satisfied"] is True for item in evaluated.values())
    refuted = any(claims[k]["terminal"] and v["satisfied"] is False for k, v in evaluated.items())
    return dict(success=True if complete else (False if refuted else None), claims=evaluated)


def evaluate_task_evidence(document):
    """Review an explicit run document. Missing evidence yields null, not failure.

    Evidence paths and reviewer labels are references, not authentication. No
    input or result produced here grants hardware execution or changes a phase.
    """
    claims, order, observations, events = _validate(document)
    model = _evaluate_channel("model_visual_report", claims, order, observations, events,
                              physical_only=document["mode"] == "physical")
    independent = _evaluate_channel("independent_rgb_review", claims, order, observations, events,
                                    physical_only=document["mode"] == "physical")
    oracle = _evaluate_channel("simulator_oracle", claims, order, observations, events)
    uncertain = any(e["source"] == "robot_receipt" and e["outcome"] in ("uncertain", "fault")
                    for e in events.values())
    reviewed_success = independent["success"] if document["mode"] == "physical" and not uncertain else None
    interventions = [dict(event_id=e["id"], scope=e["scope"], description=e["description"])
                     for e in events.values() if e["source"] == "human_intervention"]
    return dict(schema_version=SCHEMA_VERSION, run_id=document["run_id"], task_id=document["task_id"],
                mode=document["mode"], execution_available=False, phase_transition_applied=False,
                model_reported_success=model["success"], independently_reviewed_success=reviewed_success,
                simulator_oracle_success=oracle["success"] if document["mode"] == "simulation" else None,
                task_success=reviewed_success, execution_outcome_uncertain=uncertain,
                human_interventions=interventions,
                autonomy_assessment="not_inferred_from_an_incomplete_intervention_log",
                channels=dict(model_visual_report=model, independent_rgb_review=independent, simulator_oracle=oracle),
                limitations=["Declarative evidence validation does not verify image truth or reviewer identity.",
                             "Distinct observation IDs do not prove distinct decoded frames; the capture/review host owns that check.",
                             "Claims must come from the frozen task definition; changing predicates changes the assessed goal.",
                             "Model reports, independent RGB review and simulator oracle are separate evidence channels.",
                             "Robot arrival cannot satisfy object facts. Replay and simulator results are not physical success.",
                             "Task success does not imply controlled return, safety qualification or video completeness."])
