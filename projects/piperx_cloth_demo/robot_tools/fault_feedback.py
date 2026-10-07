"""RX diagnostics only: no driver calls, admission, hold or stop verdicts."""
import copy
import math

from . import arms


def diagnostic_value(value):
    """Keep invalid numeric evidence explicit while producing strict JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_numeric": repr(value)}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): diagnostic_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [diagnostic_value(item) for item in value]
    return {"unparsed_type": type(value).__name__, "representation": repr(value)}


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


class FaultFeedback:
    """Retain per-fragment high watermarks even across missing/bad samples.

    Freshness describes the time of this read only. It is not a renewed task
    receipt. Existing task snapshots may seed the tracker without rechecking
    hardware or creating any new physical qualification.
    """

    def __init__(self, max_age_s=0.1, max_skew_s=0.1):
        self.max_age_s, self.max_skew_s = max_age_s, max_skew_s
        self.watermarks = {side: {} for side in ("left", "right")}
        self.clock_high_watermark = None
        self.sequence = 0

    def seed(self, states, observed_at_s):
        for side, state in states.items():
            if side not in self.watermarks or not isinstance(state, dict):
                continue
            stamps = state.get("fragment_timestamps_s", {})
            if isinstance(stamps, dict):
                for name, stamp in stamps.items():
                    if _finite(stamp) and stamp > 0:
                        self.watermarks[side][name] = max(stamp, self.watermarks[side].get(name, stamp))
        if _finite(observed_at_s) and observed_at_s >= 0:
            self.clock_high_watermark = max(observed_at_s, self.clock_high_watermark or observed_at_s)

    def capture(self, states, observed_at_s, read_errors=None):
        self.sequence += 1
        clock_valid = _finite(observed_at_s) and observed_at_s >= 0
        clock_regressed = (clock_valid and self.clock_high_watermark is not None
                           and observed_at_s < self.clock_high_watermark)
        diagnosis, all_stamps = {}, []
        for side in ("left", "right"):
            state = states.get(side)
            stamps = state.get("fragment_timestamps_s", {}) if isinstance(state, dict) else {}
            if not isinstance(stamps, dict):
                stamps = {}
            fragments = {}
            for name in arms.PARTS + arms.DRIVERS + ("gripper",):
                stamp, previous = stamps.get(name), self.watermarks[side].get(name)
                valid = _finite(stamp) and stamp > 0
                age = observed_at_s - stamp if clock_valid and valid else None
                if name not in stamps:
                    progress = "missing"
                elif not valid:
                    progress = "invalid"
                elif previous is None:
                    progress = "first_observation"
                else:
                    progress = "advanced" if stamp > previous else ("regressed" if stamp < previous else "repeated")
                issues = []
                if not valid:
                    issues.append(progress)
                if not clock_valid:
                    issues.append("invalid_observation_clock")
                if clock_regressed:
                    issues.append("observation_clock_regressed")
                if age is not None and not 0 <= age <= self.max_age_s:
                    issues.append("future_timestamp" if age < 0 else "stale")
                if progress == "regressed":
                    issues.append("receive_timestamp_regressed")
                fragments[name] = {"timestamp_s": diagnostic_value(stamp), "age_s": age,
                                   "progress": progress, "issues": issues, "fresh": not issues}
                if valid:
                    all_stamps.append(stamp)
            try:
                health = (arms.control_health(state, now_s=observed_at_s,
                          allowed_control_modes=(0, 1, 2), require_enabled=False)
                          if clock_valid else {"healthy": None, "error": "Invalid observation clock"})
            except Exception as exc:
                health = {"healthy": None, "error": type(exc).__name__ + ": " + str(exc)}
            diagnosis[side] = {"fragments": fragments, "health": diagnostic_value(health)}
        self.seed(states, observed_at_s)
        skew = max(all_stamps) - min(all_stamps) if all_stamps else None
        complete_reads = all(isinstance(states.get(side), dict) for side in ("left", "right"))
        status = ("observed" if complete_reads and not read_errors else
                  "partial" if any(isinstance(value, dict) for value in states.values()) else "read_error")
        result = {"status": status, "sequence": self.sequence,
                  "observed_at_s": diagnostic_value(observed_at_s),
                  "observation_clock_valid": clock_valid, "observation_clock_regressed": clock_regressed,
                  "arms": diagnostic_value(copy.deepcopy(states)), "diagnostics": diagnosis,
                  "read_errors": diagnostic_value(read_errors or {}),
                  "max_cross_arm_fragment_skew_s": skew,
                  "within_fragment_skew": skew is not None and skew <= self.max_skew_s,
                  "freshness_limits_s": {"max_age": self.max_age_s, "max_skew": self.max_skew_s},
                  "hardware_commands_sent": 0, "motion_permitted": False,
                  "stationary_observed": None, "physical_stop_verified": None,
                  "scope": "RX cache at observed_at_s only; no task admission, renewed hold, arrival or stop evidence"}
        return diagnostic_value(result)
