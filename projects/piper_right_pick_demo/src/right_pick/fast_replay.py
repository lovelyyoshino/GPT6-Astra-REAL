"""Explicit mock state machine; no imports of ROS, CAN, or manufacturer SDK."""
import copy
import time


def mock_limits():
    """Fictional bounds for software tests ONLY; never a physical site config."""
    return {"workspace_min_m": [-0.5, -0.5, 0.05],
            "workspace_max_m": [0.5, 0.5, 0.6], "joint_limits_rad": [[-3.0, 3.0]] * 6,
            "max_speed_percent": 20,
            "max_translation_step_m": 0.03, "max_rotation_step_rad": 0.1,
            "max_chunk_translation_m": 0.03, "max_chunk_rotation_rad": 0.1,
            "max_waypoints": 3, "gripper_min_m": 0.0,
            "gripper_max_m": 0.07, "max_effort_parameter_nm": 1.0,
            "max_state_age_s": 0.8}


class MockRobot:
    nonphysical = True

    def __init__(self, limits=None):
        from .fast_safety import MockMotionGuard
        self.limits = copy.deepcopy(limits or mock_limits())
        self.guard = MockMotionGuard(self.limits, nonphysical=True)
        self.robot_state = {"pose_m_rad": [0.25, 0.0, 0.25, 0.0, 0.0, 0.0],
                            "joints_rad": [0.0] * 6, "arm_status": 0, "err_code": 0,
                            "enabled": True, "moving": False, "binding_verified": True}
        self.gripper_state = {"opening_m": 0.04, "effort": 0.0}
        self.executions = []

    def observe(self):
        self.robot_state["sampled_at"] = time.time()
        return {"nonphysical": True, "robot_state": copy.deepcopy(self.robot_state),
                "gripper_state": copy.deepcopy(self.gripper_state)}

    def validate(self, decision, measured):
        self.guard.validate(decision, measured)

    def execute(self, decision):
        ticket = self.guard.begin(decision, self.observe())
        args = decision.arguments
        if decision.action == "move_eef":
            self.robot_state["pose_m_rad"] = list(args["pose_m_rad"])
        elif decision.action == "move_eef_chunk":
            self.robot_state["pose_m_rad"] = list(args["waypoints"][-1])
        elif decision.action == "gripper":
            self.gripper_state = {"opening_m": args["opening_m"], "effort": args["effort_parameter_nm"]}
        self.executions.append(decision.to_dict())
        self.guard.finish(ticket, self.observe())
        result = {"status": "completed", "nonphysical": True, "robot_wait_s": 0.0,
                  "simulation_note": "State assignment only; no dynamics, contact, grasp or placement validation"}
        evidence = args.get("evidence")
        if evidence == "no_progress":
            result["visual_progress"] = False
        elif evidence == "target_lost":
            result["target_visible"] = False
        elif evidence == "grasp_failed":
            result["grasp_verified"] = False
        return result

    def latch_failure(self, reason):
        self.guard.latch_failure(reason)

    def close(self):
        pass


def demonstration_decisions():
    """Scripted phase coverage, not coordinates or a plan for the real pen."""
    def advance(phase, next_phase, evidence="phase_complete"):
        return dict(phase=phase, action="advance", arguments=dict(next_phase=next_phase, evidence=evidence), confidence=0.9)
    def move(phase, xyz, next_phase=None):
        return dict(phase=phase, action="move_eef",
                    arguments=dict(pose_m_rad=list(xyz) + [0.0, 0.0, 0.0], speed_percent=3, next_phase=next_phase), confidence=0.9)
    def jaw(phase, width):
        return dict(phase=phase, action="gripper", arguments=dict(opening_m=width, effort_parameter_nm=0.5), confidence=0.9)
    return [advance("INIT", "APPROACH_PEN"),
            move("APPROACH_PEN", [.255, 0, .25], "ALIGN_PEN"),
            move("ALIGN_PEN", [.258, 0, .25], "PREGRASP"),
            move("PREGRASP", [.258, 0, .249], "GRASP"),
            jaw("GRASP", .005), advance("VERIFY_GRASP", "LIFT"),
            move("LIFT", [.258, 0, .254], "APPROACH_HOLDER"),
            move("APPROACH_HOLDER", [.263, 0, .254], "ALIGN_HOLDER"),
            move("ALIGN_HOLDER", [.266, 0, .254], "INSERT"),
            move("INSERT", [.266, 0, .253]), advance("INSERT", "RELEASE"),
            jaw("RELEASE", .04), move("VERIFY_SUCCESS", [.266, 0, .258]),
            advance("VERIFY_SUCCESS", "DONE", "success")]


class ScriptedModel:
    """Build the real compact payload but return fixture decisions, no API call."""
    source = "scripted"

    def __init__(self, recorder, decisions=None):
        self.recorder = recorder
        self.decisions = list(demonstration_decisions() if decisions is None else decisions)
        self.calls = 0
        self.last_metrics = {}

    def decide(self, state, observation):
        from .fast_model import build_fast_payload
        from .fast_policy import select_camera_views, reasoning_effort
        started = time.monotonic()
        payload = build_fast_payload({"protocol": "responses", "model_id": "gpt-6-astra",
                                      "allow_historical": True}, state, observation)
        # Save the exact textual packet/schema and image roles, not huge inline
        # base64 copies. Original files are retained in the run observations.
        content = payload["input"][-1]["content"]
        packet = {"model": payload["model"], "input_text": [p["text"] for p in content if p["type"] == "input_text"],
                  "selected_camera_views": list(select_camera_views(state)),
                  "text": payload["text"], "reasoning_effort": reasoning_effort(state),
                  "store": payload["store"], "decision_source": self.source}
        self.recorder._write_json("prompt_%04d.json" % (self.calls + 1), packet)
        self.last_metrics = {"image_encode_s": time.monotonic() - started,
                             "agent_decide_s": 0.0, "model_request_start": None,
                             "model_response_end": None,
                             "selected_camera_views": list(select_camera_views(state)),
                             "reasoning_effort": reasoning_effort(state)}
        self.calls += 1
        if not self.decisions:
            raise RuntimeError("scripted_decisions_exhausted")
        return self.decisions.pop(0)
