"""Read-only commissioning commands and explicitly nonphysical replay."""
import argparse
import importlib.util
import json
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path

from .config import configuration_blockers, load_config
from .tasks import get_task


def _dump(value, path=None):
    body = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
    if path:
        Path(path).write_text(body + "\n", encoding="utf-8")
    else:
        print(body)


def preflight(config):
    from .robot import inspect_environment
    from .camera import environment_probe
    system = inspect_environment()
    camera = environment_probe()
    environment_blockers = []
    if not system["socket_probes"]["tcp_socket"]["available"]:
        environment_blockers.append("tcp_socket_creation_denied: cannot reach ROS or a model endpoint")
    if config["backend"] == "realsense" and not importlib.util.find_spec("pyrealsense2"):
        environment_blockers.append("pyrealsense2_missing_in_selected_python")
    if config["backend"] == "ros1":
        for module in ("rospy", "cv_bridge", "sensor_msgs", "piper_msgs"):
            try:
                found = importlib.util.find_spec(module) is not None
            except (ImportError, ValueError):
                found = False
            if not found:
                environment_blockers.append("module_missing:" + module)
    return {"checked_at": time.time(), "physical_motion_ready": False,
            "environment": system, "camera_environment": camera,
            "environment_blockers": environment_blockers,
            "commissioning_blockers": configuration_blockers(config),
            "control_commands_sent": 0,
            "prior_extrinsic_status": "INVALIDATED_BY_USER_REPORTED_FRONT_CAMERA_PITCH_CHANGE"}


def _record(config, args, mode="physical"):
    from .recording import Recorder
    task = get_task(config["task"]["name"])
    return Recorder(args.runs, config, task.instruction,
                    model_id=config.get("model", {}).get("model_id"), mode=mode)


def _finish(recorder, reason, physical_attempts=0):
    # No physical success is inferred from observing, rejecting, or stopping.
    return recorder.finish(termination_reason=reason,
                           outcome="blocked" if "blocked" in reason else None,
                           physical_attempts=physical_attempts,
                           metrics={"physical_attempts": physical_attempts,
                                    "physical_motion_time_s": 0, "control_commands_sent": 0})


def observe(config, recorder, timeout_s=8):
    from .robot import RosRightArm, check_ros_master
    check_ros_master(timeout_s=min(timeout_s, 2))
    # Register once in main thread before concurrent callbacks.
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node("right_pick_observer", anonymous=True, disable_signals=True)
    robot = RosRightArm(config["robot"])
    if config["backend"] == "ros1":
        from .ros_camera import RosCameraRig
        rig = RosCameraRig(config["cameras"], max_age_s=config["observation"]["max_age_s"],
                           max_skew_s=config["observation"]["max_skew_s"])
    elif config["backend"] == "realsense":
        from .camera import RealSenseRig
        rig = RealSenseRig(config["cameras"], depth_enabled=True)
    else:
        raise ValueError("observe requires a physical camera backend; use replay for synthetic state")
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            robot_future = executor.submit(robot.observe, timeout_s)
            camera_future = executor.submit(rig.capture, recorder.run_dir / "observations")
            frames = camera_future.result()
            state = robot_future.result()
        result = dict(frames)
        result.update(robot=state, robot_state_at=state["observed_at_s"],
                      device_freshness_verified=state["device_freshness_verified"],
                      right_binding_verified=state["right_binding_verified"],
                      enabled=state["enabled"], fault=None,
                      calibration_version=None, calibration_verified=False,
                      right_tcp_position_m=None, right_tcp_orientation_wxyz=None,
                      camera_geometry_changed=config["calibration"]["camera_geometry_changed"])
        sensor_times = [f.get("timestamp", f.get("host_received_at")) for f in result["cameras"].values()]
        robot_times = [s["received_at_s"] for s in state["samples"].values()]
        times = [t for t in sensor_times + robot_times if t is not None]
        result["robot_camera_skew_s"] = max(times)-min(times)
        if result["robot_camera_skew_s"] > config["observation"]["max_skew_s"]:
            raise RuntimeError("Robot/camera observations exceed allowed skew; no action can use them")
        _dump(result, recorder.run_dir / "observation.json")
        recorder.event("observation", result)
        return result
    finally:
        rig.close()


def policy_from_config(config):
    from .protocol import Policy
    r, c, s = config["robot"], config["calibration"], config["safety"]
    return Policy(allow_motion=s["allow_motion"], calibration_verified=c["verified"],
                  calibration_version=c["version"], camera_geometry_changed=c["camera_geometry_changed"],
                  tcp_confirmed=r["tcp_verified"], gripper_confirmed=r["gripper_verified"],
                  workspace_confirmed=r["workspace_verified"], workspace_min_m=r["workspace_min_m"],
                  workspace_max_m=r["workspace_max_m"], max_translation_step_m=s["max_translation_step_m"],
                  max_orientation_step_rad=s["max_orientation_step_rad"], max_speed_m_s=s["max_speed_m_s"],
                  gripper_min_width_m=r.get("gripper_min_width_m"), gripper_max_width_m=r.get("gripper_max_width_m"),
                  max_observation_age_s=config["observation"]["max_age_s"],
                  max_sensor_skew_s=config["observation"]["max_skew_s"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/site.local.json")
    parser.add_argument("--runs", default="runs")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("preflight", help="Local diagnostics only; sends no control commands")
    sub.add_parser("attempt", help="Record commissioning gates; never skips missing calibration or actuator validation")
    sub.add_parser("observe", help="Read existing ROS feedback and three actual camera views")
    sub.add_parser("replay", help="Nonphysical observe/wait/stop interface smoke test")
    check = sub.add_parser("check-action", help="Check one action without dispatch")
    check.add_argument("action")
    check.add_argument("observation")
    decide = sub.add_parser("decide", help="Explicit paid model request, proposal only; no actuator dispatch")
    decide.add_argument("observation")
    fast = sub.add_parser("astra_fast_closed_loop", help="Compact phase controller; offline replay, live sensor/model test or physical preparation")
    fast.add_argument("--execution", choices=("prepare", "replay", "live-check", "live"), required=True)
    fast.add_argument("--observation", action="append", default=[], help="Historical observation JSON (replay only)")
    fast.add_argument("--model", choices=("scripted", "responses", "codex"), default="scripted",
                      help="responses/codex invoke the real selected model; scripted is offline only")
    from .fast_policy import PHASES
    fast.add_argument("--phase", choices=PHASES, default="INIT", help="Current local goal for live-check only")
    fast.add_argument("--fast-config", default="configs/astra_fast_closed_loop.example.json")
    args = parser.parse_args(argv)
    recorder = None
    try:
        config = load_config(args.config)
        if args.command == "astra_fast_closed_loop":
            from .fast_cli import run_fast
            return run_fast(args, config)
        if args.command == "preflight":
            result = preflight(config)
            _dump(result)
            return 2 if result["environment_blockers"] or result["commissioning_blockers"] else 0
        mode = "nonphysical_replay" if args.command == "replay" else "physical"
        recorder = _record(config, args, mode)
        if args.command == "replay":
            from .offline import ReplayBackend
            from .protocol import Action, Policy, validate_action
            backend = ReplayBackend()
            for payload in ({"type": "observe"}, {"type": "wait", "duration_s": 0}, {"type": "stop"}):
                observation = backend.observe()
                action = Action.from_dict(payload)
                validate_action(action, observation, Policy()).require()
                recorder.event("observation", observation)
                recorder.event("feedback", backend.execute(action))
            report = _finish(recorder, "nonphysical_interface_smoke_completed")
            _dump({"run_dir": str(recorder.run_dir), "report": report})
            return 0
        if args.command == "check-action":
            from .protocol import validate_action
            action = json.loads(Path(args.action).read_text())
            observation = json.loads(Path(args.observation).read_text())
            validation = validate_action(action, observation, policy_from_config(config))
            recorder.event("action_proposal", action)
            recorder.event("validation", asdict(validation))
            _finish(recorder, "action_check_only")
            _dump({"run_dir": str(recorder.run_dir), "validation": asdict(validation), "dispatched": False})
            return 0 if validation.accepted else 2
        if args.command == "decide":
            from .model import ResponsesClient
            from .protocol import validate_action
            from .observation import require_fresh
            observation = json.loads(Path(args.observation).read_text())
            require_fresh(observation, config["observation"]["max_age_s"], config["observation"]["max_skew_s"])
            task = get_task(config["task"]["name"])
            model_config = dict(config["model"], max_observation_age_s=config["observation"]["max_age_s"],
                                max_sensor_skew_s=config["observation"]["max_skew_s"])
            action = ResponsesClient(model_config, recorder).decide(task.instruction, observation)
            validation = validate_action(action, observation, policy_from_config(config))
            recorder.event("action_proposal", action)
            recorder.event("validation", asdict(validation))
            _finish(recorder, "model_proposal_only")
            _dump({"run_dir": str(recorder.run_dir), "action": action, "validation": asdict(validation), "dispatched": False})
            return 0
        diagnostics = preflight(config)
        recorder.event("preflight", diagnostics)
        _dump(diagnostics, recorder.run_dir / "preflight.json")
        if diagnostics["environment_blockers"]:
            report = _finish(recorder, "infrastructure_blocked_before_observation")
            _dump({"run_dir": str(recorder.run_dir), "blockers": diagnostics["environment_blockers"], "report": report})
            return 2
        frames = observe(config, recorder)
        if args.command == "observe":
            report = _finish(recorder, "read_only_observation_completed")
            _dump({"run_dir": str(recorder.run_dir), "observation_file": str(recorder.run_dir/"observation.json"), "report": report})
            return 0
        # Calibration changed and physical dispatch is not commissioned. No code
        # path in this release turns a failed check into a motion command.
        report = _finish(recorder, "commissioning_blocked_after_observation")
        _dump({"run_dir": str(recorder.run_dir), "blockers": diagnostics["commissioning_blockers"], "report": report})
        return 2
    except KeyboardInterrupt:
        if recorder is not None and not recorder._finished:
            _finish(recorder, "operator_interrupted_read_only_process")
        print("Interrupted. This process sent no actuator commands.", file=sys.stderr)
        return 130
    except Exception as exc:
        if recorder is not None and not recorder._finished:
            recorder.event("error", {"type": type(exc).__name__, "message": str(exc)})
            _finish(recorder, "configuration_or_observation_error")
        _dump({"error": type(exc).__name__, "message": str(exc), "run_dir": str(recorder.run_dir) if recorder else None,
               "control_commands_sent": 0})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
