"""Non-overlapping controller timings; replay never masquerades as a real trial."""
import json
import statistics
import time


class FastRunMetrics:
    def __init__(self, recorder):
        self.recorder = recorder
        self.started = time.monotonic()
        self.steps = []

    def step(self, row):
        row = self.recorder._sanitize(row)
        with (self.recorder.run_dir / "steps.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.recorder.event("controller_step", row)
        self.steps.append(row)

    def finish(self, *, termination_reason, phase, nonphysical, replay_completed=False,
               live_check=False, live_check_passed=False, physical_loop=False,
               task_evidence=(), model_reported_success=False, target_uncertain=False):
        if physical_loop and (nonphysical or live_check or replay_completed):
            raise ValueError("Physical loop metrics cannot be labelled replay or live-check")
        elapsed = time.monotonic() - self.started
        def total(key):
            return sum(row.get(key) or 0.0 for row in self.steps)
        latencies = [row["agent_decide_s"] for row in self.steps
                     if row.get("model_request_start") is not None]
        model_s = sum(latencies)
        camera_s = total("image_capture_s") + total("image_encode_s")
        execute_s, wait_s = total("robot_execute_s"), total("robot_wait_s")
        phases = {}
        for row in self.steps:
            phases[row["phase"]] = phases.get(row["phase"], 0.0) + row["total_step_s"]
        recoveries = sum(bool(row.get("phase_transition")) and
                         row["phase_transition"].get("to") == "RECOVERY" and
                         row["phase_transition"].get("from") != "RECOVERY"
                         for row in self.steps)
        dispatched = sum(bool(row.get("action_dispatched")) for row in self.steps)
        attempted = sum(bool(row.get("dispatch_attempted")) for row in self.steps)
        receipts = sum(bool(row.get("command_receipt_confirmed")) for row in self.steps)
        # ROS transaction receipts are commands, not CAN frame counts. An
        # interrupted send may have reached firmware without a receipt.
        commands_sent = None if target_uncertain else (receipts if physical_loop else 0)
        report = {
            "mode": "astra_fast_closed_loop", "nonphysical": nonphysical,
            "termination_reason": termination_reason, "final_phase": phase,
            "total_elapsed_s": elapsed, "model_total_s": model_s,
            "model_time_definition": "Decision backend wall time including transport/CLI startup, not isolated provider compute",
            "decision_backends": sorted(set(row.get("decision_source", "unknown") for row in self.steps)),
            "robot_execute_total_s": execute_s + wait_s,
            "robot_dispatch_total_s": execute_s, "robot_wait_total_s": wait_s,
            "camera_total_s": camera_s,
            "other_total_s": max(0.0, elapsed - model_s - execute_s - wait_s - camera_s),
            "model_call_count": len(self.recorder.model_calls),
            "scripted_decision_count": sum(row.get("decision_source") == "scripted" and row.get("decision_requested", False) for row in self.steps),
            "mean_model_latency_s": statistics.mean(latencies) if latencies else None,
            "median_model_latency_s": statistics.median(latencies) if latencies else None,
            "max_model_latency_s": max(latencies) if latencies else None,
            "action_count": dispatched,
            "action_count_definition": "Motion/chunk/gripper dispatches only; excludes observe/advance/pause. Nonphysical actions are not robot commands.",
            "step_count": len(self.steps), "phase_durations": phases,
            "recovery_count": recoveries,
            "task_success": None,
            "model_reported_task_success": bool(model_reported_success),
            "replay_completed": bool(replay_completed),
            "live_check": bool(live_check), "live_check_passed": bool(live_check_passed),
            "evaluation_scope": "live_sensor_model_integration_no_motion" if live_check else
                                ("nonphysical_replay" if nonphysical else
                                 ("physical_closed_loop" if physical_loop else "physical_preparation")),
            "control_commands_sent": commands_sent,
            "control_commands_sent_definition": "Confirmed ROS transactions, not CAN frames; null when a dispatch outcome is uncertain",
            "dispatch_attempt_count": attempted,
            "confirmed_command_receipt_count": receipts,
            "target_uncertain": bool(target_uncertain),
            "usage": self.recorder.usage_summary(),
            "limitations": ["Live sensor/model integration is not a physical grasp or hold qualification." if live_check else
                            ("Offline/scripted replay is neither visual validation nor a latency/success benchmark."
                            if nonphysical else ("Success requires ordered recorded visual/physical evidence; arrival is not grasp verification."
                                                 if physical_loop else "Physical execution is not commissioned.")),
                            "Unknown provider token counts and absent physical timings are not imputed."]}
        outcome = "not_evaluated" if nonphysical or live_check else "blocked"
        if physical_loop:
            # Recorder independently checks the evidence order and sources;
            # neither DONE nor model_reported_success alone is sufficient.
            outcome = None if model_reported_success and not target_uncertain else (
                "not_evaluated" if attempted else "blocked")
        detailed = self.recorder.finish(termination_reason=termination_reason,
                             evidence=task_evidence if physical_loop else (),
                             outcome=outcome,
                             physical_attempts=int(attempted > 0) if physical_loop else 0,
                             metrics={"control_commands_sent": commands_sent,
                                      "physical_motion_time_s": execute_s + wait_s if physical_loop else None})
        report["task_success"] = detailed["success"]
        report["task_evidence_order_valid"] = detailed["evidence_order_valid"]
        self.recorder._write_json("fast_summary.json", report)
        return report
