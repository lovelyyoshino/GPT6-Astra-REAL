"""Nonphysical replay backend; never imports or falls back to a robot SDK."""
import time
from .protocol import Action, ProtocolError


class ReplayBackend:
    mode = "nonphysical_replay"
    calibration_version = "NONPHYSICAL-REPLAY-ONLY"

    def __init__(self, clock=None):
        self.clock = clock or time.time
        self.position_m = [0.25, 0.0, 0.20]
        self.orientation_wxyz = [1.0, 0.0, 0.0, 0.0]
        self.width_m = 0.05
        self.last_feedback = None
        self.commands = []

    def observe(self):
        now = self.clock()
        return {
            "mode": self.mode, "nonphysical": True, "captured_at": now,
            "robot_state_at": now, "calibration_version": self.calibration_version,
            "camera_geometry_changed": False,
            "device_freshness_verified": True, "right_binding_verified": True,
            "enabled": True, "fault": 0,
            "right_tcp_position_m": list(self.position_m),
            "right_tcp_orientation_wxyz": list(self.orientation_wxyz),
            "right_gripper_width_m": self.width_m,
            "images": [], "previous_feedback": self.last_feedback,
            "warning": "Synthetic state, no physical dynamics, no camera or task-success evidence.",
        }

    def execute(self, action):
        action = Action.from_dict(action.to_dict() if isinstance(action, Action) else action)
        if action.type in ("move_tcp", "gripper") and action.calibration_version != self.calibration_version:
            raise ProtocolError("replay requires NONPHYSICAL-REPLAY-ONLY calibration version")
        self.commands.append(action.to_dict())
        if action.type == "move_tcp":
            self.position_m = list(action.pose.position_m)
            self.orientation_wxyz = list(action.pose.orientation_wxyz)
        elif action.type == "gripper":
            self.width_m = action.width_m
        feedback = {"status": "nonphysical_applied", "action": action.type,
                    "at": self.clock(), "nonphysical": True,
                    "physical_motion_time_s": None, "task_success": None}
        if action.type == "stop":
            feedback["status"] = "decision_stopped"
        self.last_feedback = feedback
        return feedback

    def close(self):
        pass
