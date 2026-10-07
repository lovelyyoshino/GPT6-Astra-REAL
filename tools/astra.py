"""Task composition, bounded replay and evidence review without hardware imports."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "projects/piper_right_pick_demo/src"))

from right_pick.fast_pipeline import PipelineContractError
from right_pick.fast_task_pipeline import BoundedTaskPipeline, COMPOSITIONS, TASK_RECIPES
from right_pick.fast_task_spec import load_recipe


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 16 * 1024 * 1024:
        raise PipelineContractError("Input JSON exceeds 16 MiB")
    from right_pick.fast_task_spec import _unique_object
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)


def ledger_for(args):
    definition = load_recipe(args.recipe, mode=args.mode, worker_arm=args.worker_arm) if args.recipe else None
    return BoundedTaskPipeline(definition["task_id"] if definition else args.task,
        mode=args.mode, worker_arm=args.worker_arm, task_definition=definition)


def synthetic_receipt(current, index):
    """Contract plumbing fixture only. These facts are invented, never evidence."""
    stamp = float(index * 10)
    event = dict(task=current["task"], stage=current["stage"], arm=current["arm"],
        status="complete", observation_id="synthetic-" + str(index), at=stamp,
        model_called=False, prerequisites=current["needs"], evidence=current["expect"])
    if current["skill"] == "stable_verify":
        event["stability_samples"] = [
            dict(observation_id="synthetic-first-" + str(index), at=stamp - 2,
                 support_stable=True, gripper_clear=True),
            dict(observation_id=event["observation_id"], at=stamp,
                 support_stable=True, gripper_clear=True)]
    return event


def replay(ledger, receipts=None):
    """No model/image interpretation; existing receipts or explicit synthetic fixtures."""
    if receipts is not None and (not isinstance(receipts, list) or len(receipts) > 128):
        raise PipelineContractError("Replay expects at most 128 cycle receipts")
    events = []
    count = len(receipts) if receipts is not None else len(ledger.contract["steps"])
    for index in range(count):
        before = ledger.current()
        if before is None:
            if receipts is not None:
                raise PipelineContractError("Replay contains events after termination")
            break
        event = receipts[index] if receipts is not None else synthetic_receipt(before, index + 1)
        result = ledger.record_cycle(event)
        events.append(dict(current=before, receipt=event, result=result))
    encoded = json.dumps(ledger.contract, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    return dict(schema_version="astra_offline_replay_v1", mode="offline_replay",
        input_kind="historical_receipts_unverified" if receipts is not None else "synthetic_contract_fixtures",
        contract_sha256=hashlib.sha256(encoded.encode()).hexdigest(),
        contract=ledger.contract, events=events, summary=ledger.report(),
        hardware_accessed=False, dispatched_action_count=0, actual_model_call_count=0,
        task_success=None, image_interpretation_performed=False,
        limitation="Checks bounded contract transitions only; neither synthetic nor historical facts prove current physical success.")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "session":
        from right_pick.fast_task_session import main as session_main
        return session_main(argv[1:])
    parser = argparse.ArgumentParser(description="GPT6-Astra-REAL offline tools; no robot, camera or model connections")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("catalog", help="List reusable tasks and operations, including execution boundaries")
    for name in ("plan", "replay"):
        child = commands.add_parser(name, help="Compile recipe" if name == "plan" else "Offline contract replay; defaults to synthetic fixtures")
        choice = child.add_mutually_exclusive_group(required=True)
        choice.add_argument("--recipe")
        choice.add_argument("--task", choices=[t.task_id for t in TASK_RECIPES])
        child.add_argument("--mode", choices=("single_arm", "dual_arm", "worker_with_observer"), default="single_arm")
        child.add_argument("--worker-arm", choices=("left", "right"), default="right")
        if name == "replay":
            child.add_argument("--receipts", help="Optional historical cycle-receipt JSON list; does not execute actions")
            child.add_argument("--out", help="Create a new report; existing output is never overwritten")
    evaluate = commands.add_parser("evaluate", help="Review declared evidence structure and timing; does not authenticate pixels")
    evaluate.add_argument("document")
    experiences = commands.add_parser("experiences", help="View source-checked historical advice for the current operation")
    experiences.add_argument("--task", default="pen")
    experiences.add_argument("--phase", required=True)
    references = commands.add_parser("references", help="Trace reference ideas to local implementation, tests and remaining work")
    references.add_argument("--source", help="Reference id such as arx5, zetta or maniskill")
    references.add_argument("--task", choices=[t.task_id for t in TASK_RECIPES],
                            help="Inspect one reviewed ARX5 task variant and its evidence requirements")
    recovery = commands.add_parser("recovery-plan", help="Offline recovery diagnosis; never applies a phase transition or action")
    recovery.add_argument("document", help="JSON containing current_stage, execution_result and remaining_budget")
    recording = commands.add_parser("recording-plan", help="Describe task recording requirements; does not start cameras or create videos")
    recording.add_argument("--run-id", required=True)
    recording.add_argument("--task", required=True)
    recording.add_argument("--include-left", action="store_true")
    recording_audit = commands.add_parser("recording-audit", help="Audit declared video/frame metadata coverage; does not verify video pixels")
    recording_audit.add_argument("document")
    commands.add_parser("session", help="Persistent offline ledger: session --store PATH init/current/record/... (see session --help)")
    args = parser.parse_args(argv)
    try:
        if args.command == "catalog":
            value = dict(project=str(ROOT), built_in_tasks=[t.task_id for t in TASK_RECIPES],
                recipes=[str(p.relative_to(ROOT)) for p in sorted((ROOT / "tasks").glob("*.json"))],
                operations=[c.name for c in COMPOSITIONS], execution_available=False,
                physical_adapter="Existing single-right pen source only; current field qualification withdrawn")
        elif args.command == "plan":
            value = ledger_for(args).contract
        elif args.command == "replay":
            value = replay(ledger_for(args), read_json(args.receipts) if args.receipts else None)
            if args.out:
                path = Path(args.out)
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("x", encoding="utf-8") as stream:
                    json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
                    stream.write("\n")
                value = dict(report=str(path.resolve()), **{k: value[k] for k in (
                    "input_kind", "summary", "hardware_accessed", "task_success", "actual_model_call_count", "dispatched_action_count")})
        elif args.command == "evaluate":
            from right_pick.fast_task_evaluation import evaluate_task_evidence
            value = dict(scope="declared_evidence_structure_review", authenticated_visual_truth=False,
                         result=evaluate_task_evidence(read_json(args.document)))
        elif args.command == "experiences":
            from right_pick.fast_experience import build_historical_advisories
            value = dict(historical_advisories=build_historical_advisories(
                dict(phase=args.phase), task_id=args.task), execution_available=False)
        elif args.command == "references":
            value = read_json(ROOT / "research/reference_adoption.json")
            if args.task and args.source not in (None, "arx5"):
                raise PipelineContractError("Task-specific experiment mapping belongs to source arx5")
            if args.task:
                matrix = read_json(ROOT / "research/arx5_experiment_patterns_round2.json")
                row = next(item for item in matrix["experiments"] if item["task_id"] == args.task)
                value = dict(source=next(item for item in value["sources"] if item["id"] == "arx5"),
                    experiment=row, common_limits=value["common_limits"], hardware_accessed=False)
            elif args.source:
                matches = [item for item in value["sources"] if item["id"] == args.source]
                if not matches:
                    raise PipelineContractError("Unknown reference id: " + args.source)
                value = dict(source=matches[0], common_limits=value["common_limits"], hardware_accessed=False)
        elif args.command == "recovery-plan":
            from right_pick.fast_task_recovery import build_recovery_contract
            document = read_json(args.document)
            fields = {"current_stage", "execution_result", "remaining_budget"}
            if not isinstance(document, dict) or not fields <= set(document) or set(document) - fields - {"fixture_kind"}:
                raise PipelineContractError("Recovery input requires current_stage, execution_result and remaining_budget")
            value = build_recovery_contract(**{key: document[key] for key in fields})
        elif args.command == "recording-plan":
            from right_pick.fast_recording_contract import make_recording_contract
            value = make_recording_contract(args.run_id, args.task, include_left=args.include_left)
        elif args.command == "recording-audit":
            from right_pick.fast_recording_contract import audit_recording
            value = audit_recording(read_json(args.document))
        else:
            parser.error("Use session --store PATH <command>")
        print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (PipelineContractError, ValueError, TypeError, OSError) as exc:
        print(json.dumps(dict(error=type(exc).__name__, message=str(exc), execution_available=False), ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
