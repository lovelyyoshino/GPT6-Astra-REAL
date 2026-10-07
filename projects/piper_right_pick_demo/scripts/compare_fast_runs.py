#!/usr/bin/env python3
"""Build an honest comparison; unavailable physical measurements remain null."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline-review", required=True)
    p.add_argument("--optimized-summary", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    baseline = json.loads(Path(args.baseline_review).read_text())
    fast = json.loads(Path(args.optimized_summary).read_text())
    comparable = fast.get("nonphysical") is False and fast.get("task_success") is not None
    table = {
        "baseline": {
            "description": "Previous supervised ROS pen placement with the undivided holder",
            "source": str(Path(args.baseline_review).resolve()),
            "total_elapsed_s": None,
            "recording_wall_time_s": baseline["recording_timing"]["recording_process_wall_duration_s"],
            "model_call_count": baseline["model_usage_and_cost"]["model_api_call_count"],
            "model_total_s": None, "robot_motion_s": None,
            "command_and_feedback_time_s": baseline["intent_to_observed_stable_latency"]["statistics"]["all"]["sum_s"],
            "command_timing_coverage": "48/49; includes dispatch and stability feedback, excludes one failed monitor",
            "task_success": baseline.get("task_success"),
            "autonomous_success_rate": None,
        },
        "optimized": {
            "source": str(Path(args.optimized_summary).resolve()),
            "physical_trial_executed": comparable,
            "total_elapsed_s": fast.get("total_elapsed_s") if comparable else None,
            "model_call_count": fast.get("model_call_count") if comparable else None,
            "model_total_s": fast.get("model_total_s") if comparable else None,
            "robot_execute_total_s": fast.get("robot_execute_total_s") if comparable else None,
            "task_success": fast.get("task_success") if comparable else None,
            "autonomous_success_rate": None,
        },
        "offline_validation": ({key: fast.get(key) for key in
                               ("nonphysical", "replay_completed", "step_count", "scripted_decision_count", "model_call_count")}
                               if fast.get("nonphysical") is True else None),
        "live_integration_only": ({key: fast.get(key) for key in
                                  ("live_check_passed", "total_elapsed_s", "model_call_count", "model_total_s",
                                   "mean_model_latency_s", "control_commands_sent", "task_success")}
                                  if fast.get("live_check") is True else None),
        "kpi": {"successful_task_under_1800_s": None, "model_calls_reduced_at_least_30_percent": None,
                "mean_model_latency_reduced": None, "status": "not_measurable_from_available_physical_evidence"},
        "notes": ["Recording duration is a window, not a reconstructed exact task clock.",
                  "49 robot commands are not 49 Astra calls.",
                  "Scripted replay timings/counts are not physical task performance.",
                  "Model request/response wall time is not isolated internal reasoning time.",
                  "A single successful supervised trial cannot establish autonomous success rate."]}
    Path(args.output).write_text(json.dumps(table, ensure_ascii=False, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
