"""One model-specified small J2/J3 adjustment, never a cancellation or stop.

The caller establishes physical clearance and attachment extent. The MDH
bound encloses the monitored independent-joint box, not cached/partial target
activation or an industrial collision guarantee. No task target is generated.
"""
import copy
import math
import time

from . import arms
from .joint_recovery import RECOVERY, _JointRecovery
from .takeover import SIDES

STEP = {"joint_change_rad": 0.025, "joint_margin_rad": 0.010,
        "monitor_tolerance_rad": 0.003, "stable_span_m": 0.0005,
        "stable_s": 3.0, "clearance_reserve_m": 0.005,
        "link_body_allowance_m": 0.060}


def _positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be a finite positive number")
    return float(value)


class _BoundedJointStep(_JointRecovery):
    SCOPE = "One caller-specified bounded J2/J3 step; no retry, cancel or hold qualification"

    def __init__(self, profile, journal_callback, arm, target, attachment_radius_m, available_clearance_m):
        super().__init__(profile, journal_callback, arm, target)
        self.attachment_radius = attachment_radius_m
        self.clearance = available_clearance_m
        self.report.pop("recovery_limits")
        self.report.update(operation="bounded_joint_step", step_observed=False,
                           step_limits=dict(STEP), pending_target_state="unknown",
                           previous_target_cancelled=False, baseline_motion_status={},
                           attachment_radius_m=attachment_radius_m,
                           available_clearance_m=available_clearance_m,
                           sweep_scope="MDH remaining chain plus caller-confirmed attachment/body envelope; independent-axis tracking box only",
                           physical_envelope_confirmed_by_tool=False,
                           completion_scope="Fresh motion_status=0 and stable observed joint target; not stop qualification")

    def tolerance(self, side, index):
        return STEP["monitor_tolerance_rad"]

    def stable_position_span(self):
        return STEP["stable_span_m"]

    def allowed_motion_status(self, side, active):
        return (0, 1) if side == self.arm else (0,)

    def checked(self, states):
        states = super().checked(states)
        stamps = [stamp for side in SIDES for stamp in states[side]["fragment_timestamps_s"].values()]
        if time.time() - min(stamps) > RECOVERY["feedback_age_s"]:
            raise RuntimeError("Bounded step requires all feedback fragments within 50 ms")
        return states

    def observe_window(self, duration, **unused):
        # _Takeover.prepare calls this only for its passive initial window.
        start, advances = time.monotonic(), 0
        previous = self.checked(self.read())
        bounds = {side: {"low": previous[side]["pose_m_rad"][:3],
                         "high": previous[side]["pose_m_rad"][:3],
                         "qlow": previous[side]["joints_rad"][:],
                         "qhigh": previous[side]["joints_rad"][:]} for side in SIDES}
        self.report["baseline_motion_status"] = {
            side: previous[side]["arm_status"]["motion_status"] for side in SIDES}
        while time.monotonic() - start < STEP["stable_s"]:
            time.sleep(RECOVERY["poll_s"])
            current = self.checked(self.read())
            for side in SIDES:
                box, state = bounds[side], current[side]
                for name, values, fn in (("low", state["pose_m_rad"][:3], min),
                                         ("high", state["pose_m_rad"][:3], max),
                                         ("qlow", state["joints_rad"], min),
                                         ("qhigh", state["joints_rad"], max)):
                    box[name] = [fn(a, b) for a, b in zip(box[name], values)]
                if (math.dist(box["low"], box["high"]) > STEP["stable_span_m"]
                        or any(b-a > STEP["monitor_tolerance_rad"] for a, b in zip(box["qlow"], box["qhigh"]))):
                    raise RuntimeError(side + " lacks the required three-second stationary baseline")
            if self.advanced(previous, current):
                previous, advances = current, advances + 1
        if advances < 20:
            raise RuntimeError("Insufficient advancing complete feedback for three-second baseline")
        self.report.update(baseline_duration_s=time.monotonic()-start,
                           baseline_feedback_advances=advances,
                           baseline_spans={side: {"position_m": math.dist(box["low"], box["high"]),
                               "joints_rad": [b-a for a, b in zip(box["qlow"], box["qhigh"])]}
                               for side, box in bounds.items()})
        return current

    def validate_target(self, states):
        from pyAgxArm.utiles.mdh_kinematics import get_mdh
        state, origin = states[self.arm], self.anchor[self.arm]
        q = state["joints_rad"]
        raw = [round(math.degrees(value)*1000) for value in self.target]
        vendor_raw = [round(value*(180/math.pi)*1000) for value in self.target]
        guard_raw = [round(value*180/math.pi*1000) for value in self.target]
        if raw != vendor_raw or raw != guard_raw:
            raise RuntimeError("Floating half-millidegree target encodes inconsistently; refuse before mode TX")
        encoded = [math.radians(value/1000) for value in raw]
        if self.outside(self.arm, q) or self.outside(self.arm, encoded):
            raise RuntimeError("Current and encoded selected-arm joints must be within manufacturer limits")
        for i, (current, target) in enumerate(zip(q, encoded)):
            if i in (1, 2):
                low, high = self.joint_limits[self.arm]["joint%d" % (i+1)]
                if abs(target-current) > STEP["joint_change_rad"]:
                    raise RuntimeError("J2/J3 step exceeds 0.025 rad")
                if min(target-low, high-target) < STEP["joint_margin_rad"]:
                    raise RuntimeError("Encoded J2/J3 target must leave at least 0.010 rad to both limits")
            elif raw[i] != round(math.degrees(current)*1000):
                raise RuntimeError("Only J2/J3 may change; other axes must encode identically to fresh feedback")
        if all(raw[i] == round(math.degrees(q[i])*1000) for i in (1, 2)):
            raise RuntimeError("Bounded step requires an actual J2/J3 target change")
        robot = self.robots[self.arm]
        current_fk, target_fk = arms._six_finite(robot.fk(q[:])), arms._six_finite(robot.fk(encoded[:]))
        position_error = math.dist(current_fk[:3], state["pose_m_rad"][:3])
        rotation_error = self.rotation_distance(current_fk, state["pose_m_rad"])
        target_distance = math.dist(target_fk[:3], state["pose_m_rad"][:3])
        if position_error > 0.002 or rotation_error > 0.02:
            raise RuntimeError("Manufacturer FK does not agree with current flange feedback")
        if target_distance > RECOVERY["target_displacement_m"]:
            raise RuntimeError("Manufacturer FK bounded target exceeds 15 mm displacement")
        mdh = get_mdh(self.profile["arms"][self.arm]["model"])
        if len(mdh) != 6 or any(not math.isfinite(v) for row in mdh for v in row):
            raise RuntimeError("A finite six-axis manufacturer MDH chain is required")
        # Modified DH places a_i before joint i's rotation: exclude it from
        # radius_i, retain d_i and every subsequent translation conservatively.
        radii = [abs(row[0]) + sum(abs(link[0])+abs(link[1]) for link in mdh[i+1:])
                 + self.attachment_radius + STEP["link_body_allowance_m"] for i, row in enumerate(mdh)]
        excursions = [abs(target-start)+self.tolerance(self.arm, i)
                      for i, (start, target) in enumerate(zip(origin["joints_rad"], encoded))]
        sweep = sum(radius*delta for radius, delta in zip(radii, excursions))
        self.report.update(sweep_bound_m=sweep, sweep_axis_radii_m=radii,
                           sweep_axis_excursions_rad=excursions,
                           sweep_reference_joints_rad=origin["joints_rad"][:],
                           mdh_source="Manufacturer get_mdh(model), modified DH; sum of absolute remaining translations",
                           sweep_clearance_budget_m=self.clearance-STEP["clearance_reserve_m"])
        if sweep > self.clearance-STEP["clearance_reserve_m"]:
            raise RuntimeError("Whole-chain tracking-box sweep bound exceeds clearance minus 5 mm reserve")
        self.target_pose = target_fk
        self.report.update(fresh_dispatch_state=copy.deepcopy(state), target_flange_m_rad=target_fk,
                           encoded_target_joints_rad=encoded, fk_current_flange_m_rad=current_fk,
                           fk_feedback_position_error_m=position_error,
                           fk_feedback_rotation_error_rad=rotation_error,
                           target_flange_displacement_m=target_distance,
                           observed_boundary_violations={side: self.outside(side, states[side]["joints_rad"]) for side in SIDES})

    def perform(self):
        super().perform()
        if not self.report["selected_arm_strictly_within_limits"]:
            raise RuntimeError("Observed target retains a selected-arm joint-limit violation")
        return "bounded_joint_target_observed_not_hold_qualified"

    def run(self):
        report = super().run()
        report["step_observed"] = report.pop("recovery_observed")
        report["step_attempted"] = report.pop("recovery_attempted")
        return report


def bounded_joint_step(profile, journal_callback, arm, target_joints_rad,
                       attachment_radius_m, available_clearance_m):
    """One explicit model-selected J2/J3 target; service owns device exclusion."""
    if arm not in SIDES:
        raise ValueError("arm must be left or right")
    if not callable(journal_callback):
        raise TypeError("A synchronous journal_callback(event, data) is required")
    target = arms._six_finite(target_joints_rad)
    attachment = _positive(attachment_radius_m, "attachment_radius_m")
    clearance = _positive(available_clearance_m, "available_clearance_m")
    return _BoundedJointStep(profile, journal_callback, arm, target, attachment, clearance).run()
