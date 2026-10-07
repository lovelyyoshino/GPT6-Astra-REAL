"""Separate CLI routes; only the independent ROS gate can admit live execution."""
import json
import os
from pathlib import Path

from .recording import Recorder


def run_fast(args, site_config):
    from .fast_safety import physical_blockers
    from .fast_recording import FastRunMetrics
    from .fast_pipeline import validate_runtime_config
    fast_config = json.loads(Path(args.fast_config).read_text(encoding="utf-8"))
    if fast_config.get("mode") != "astra_fast_closed_loop":
        raise ValueError("Explicit astra_fast_closed_loop config required")
    validate_runtime_config(fast_config)
    if args.execution not in ("prepare", "replay", "live-check", "live"):
        raise ValueError("Unknown fast execution mode")
    expected_protocol = {"codex": "codex_cli", "responses": "responses"}.get(args.model)
    if expected_protocol is not None and fast_config.get("model", {}).get("protocol") != expected_protocol:
        raise ValueError("Requested model backend does not match config protocol")
    instruction = "Single right arm: use only current RGB and measured robot state to pick the pen and put it into the holder; no camera calibration, object coordinates or traditional detection."
    nonphysical = args.execution == "replay"
    model_config = dict(fast_config.get("model", {}))
    model_config.setdefault("pipeline_id", fast_config.get("pipeline_id", "single_arm_closed_loop_v1"))
    recorder = Recorder(args.runs, {"site": site_config, "fast": fast_config}, instruction,
                        model_id=model_config.get("model_id"),
                        mode="nonphysical_replay" if nonphysical else "physical")
    try:
        if args.execution == "live":
            from .fast_ros import ROSRightArm
            from .fast_observation import SubprocessRGBCameras
            from .fast_live_loop import FastLiveClosedLoop, verify_preflight_contract
            if args.model not in ("codex", "responses"):
                raise ValueError("live requires explicit --model codex or --model responses")
            # Fail before constructing a model or camera adapter. This call
            # reads independent qualification evidence; it cannot commission
            # the robot or override FastSafetyGuard with a configuration flag.
            robot = ROSRightArm(site_config, recorder=recorder, proposal_only=False)
            try:
                readiness = verify_preflight_contract(robot.preflight())
                recorder.event("cli_live_preflight", readiness)
            except BaseException as exc:
                robot.close()
                if not isinstance(exc, Exception):
                    raise
                recorder.event("live_preflight_blocked", {"type": type(exc).__name__,
                    "message": str(exc)[:1000], "control_commands_sent": 0,
                    "model_calls": 0, "camera_captures": 0})
                report = FastRunMetrics(recorder).finish(termination_reason="preflight_blocked",
                    phase="INIT", nonphysical=False, physical_loop=True)
                print(json.dumps({"run_dir": str(recorder.run_dir), "report": report}, ensure_ascii=False, indent=2))
                return 2
            cameras, handed_to_loop = None, False
            try:
                if args.model == "codex":
                    from .fast_codex import CodexDecisionClient
                    model = CodexDecisionClient(model_config, recorder)
                else:
                    from .fast_model import FastResponsesClient
                    model = FastResponsesClient(model_config, recorder)
                cameras = SubprocessRGBCameras(site_config, recorder.run_dir / "observations")
                loop = FastLiveClosedLoop(model=model, robot=robot, cameras=cameras, recorder=recorder,
                    limits=site_config.get("physical_limits"), options=fast_config.get("controller"),
                    observation_contract=fast_config.get("observation_contract"))
                handed_to_loop = True
                report = loop.run()  # Rechecks admission immediately before sensor/model work.
            finally:
                if not handed_to_loop:
                    try:
                        if cameras is not None:
                            cameras.close()
                    finally:
                        robot.close()
            print(json.dumps({"run_dir": str(recorder.run_dir), "report": report}, ensure_ascii=False, indent=2))
            return 0 if report["task_success"] is True else 2
        if args.execution == "live-check":
            from .fast_ros import ROSRightArm
            from .fast_observation import SubprocessRGBCameras
            from .fast_live_check import run_live_check
            if args.model == "scripted":
                raise ValueError("live-check requires explicit --model codex or --model responses")
            if args.model == "codex":
                from .fast_codex import CodexDecisionClient
                model = CodexDecisionClient(model_config, recorder)
            else:
                from .fast_model import FastResponsesClient
                model = FastResponsesClient(model_config, recorder)
            robot = ROSRightArm(site_config, recorder=recorder, proposal_only=True)
            cameras = SubprocessRGBCameras(site_config, recorder.run_dir / "observations")
            report = run_live_check(model=model, robot=robot, cameras=cameras, recorder=recorder,
                                    phase=args.phase)
            print(json.dumps({"run_dir": str(recorder.run_dir), "report": report}, ensure_ascii=False, indent=2))
            return 0 if report["live_check_passed"] else 2
        if not nonphysical:
            blockers = physical_blockers(site_config)
            if args.model == "responses" and not os.environ.get(model_config.get("api_key_env", "OPENAI_API_KEY")):
                blockers.append("model_api_credential_environment_variable_unset")
            recorder.event("physical_preparation", {"blockers": blockers, "control_commands_sent": 0,
                                                     "right_home_executed": False})
            report = FastRunMetrics(recorder).finish(termination_reason="physical_safety_blocked",
                                                    phase="INIT", nonphysical=False)
            print(json.dumps({"run_dir": str(recorder.run_dir), "blockers": blockers,
                              "right_home_executed": False, "control_commands_sent": 0,
                              "report": report}, ensure_ascii=False, indent=2))
            return 2
        from .fast_observation import HistoricalRGBSource
        from .fast_replay import MockRobot, ScriptedModel, mock_limits
        from .fast_loop import FastClosedLoop
        if not args.observation:
            raise ValueError("Replay requires --observation with existing three-view RGB; images are not fabricated")
        cameras = HistoricalRGBSource(args.observation, recorder.run_dir / "observations")
        limits = mock_limits()
        robot = MockRobot(limits)
        if args.model == "responses":
            from .fast_model import FastResponsesClient
            model = FastResponsesClient(dict(model_config, allow_historical=True), recorder)
        elif args.model == "codex":
            from .fast_codex import CodexDecisionClient
            model = CodexDecisionClient(dict(model_config, allow_historical=True), recorder)
        else:
            model = ScriptedModel(recorder)
        recorder.event("replay_contract", {"historical_images": True, "mock_state_only": True,
                                           "visual_consequences_simulated": False,
                                           "task_success_measurable": False, "mock_limits": limits})
        report = FastClosedLoop(model=model, robot=robot, cameras=cameras, recorder=recorder,
                                limits=limits, options=fast_config.get("controller")).run()
        print(json.dumps({"run_dir": str(recorder.run_dir), "report": report}, ensure_ascii=False, indent=2))
        return 0 if report["replay_completed"] else 2
    except BaseException as exc:
        if not recorder._finished:
            recorder.event("fast_mode_error", {"type": type(exc).__name__})
            FastRunMetrics(recorder).finish(termination_reason="fast_mode_setup_or_runtime_error",
                                           phase="INIT", nonphysical=nonphysical)
        raise
