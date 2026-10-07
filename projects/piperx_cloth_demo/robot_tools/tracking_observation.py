"""Per-event RX diagnostics and conservative outside-band elapsed time.

This helper has no device, clock, dispatch ticket or permission mechanism. Its
caller must establish complete sending and run every applicable hard feedback
guard before accounting a settling observation. Starting this counter proves
nothing about transmission, target acceptance, arrival or physical stopping.
"""
import copy
import math


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class TrackingObservation:
    def __init__(self, label):
        self.label = label
        self.previous_at = None
        self.previous_outside = False
        self.cumulative_s = 0.
        self.last_attempted_sample = None
        self.last_tracking = None
        self.first_failure = None
        self.report = {"mode": "strict", "cumulative_outside_nominal_band_s": 0.,
            "interval_rule": "count_complete_interval_when_either_observation_endpoint_is_outside",
            "max_excess_rad": 0., "max_origin_excursion_rad": 0.,
            "peak_sample": None, "first_outside_nominal_band": None, "first_failure": None}

    def start(self, sent_at):
        """Set an accounting origin once; this is not a dispatch attestation."""
        if self.previous_at is not None or not _finite(sent_at):
            raise RuntimeError("Tracking accounting origin is invalid or already set")
        self.previous_at = sent_at

    def track(self, plan, sample):
        """Capture diagnostics even when a later hard guard rejects the sample."""
        self.last_attempted_sample = copy.deepcopy(sample)
        arm = plan["identity"]["arm"]
        q = sample["arms"][arm]["joints_rad"]
        start = plan["origin"]["arms"][arm]["joints_rad"]
        envelope = plan["joint_envelope"]
        outside = [{"joint_index": i+1, "observed_rad": value, "low_rad": low, "high_rad": high,
                    "excess_rad": max(low-value, value-high)}
                   for i, (value, low, high) in enumerate(zip(q, envelope["low_rad"], envelope["high_rad"]))
                   if not low <= value <= high]
        tracking = {"within_nominal_band": not outside, "outside_nominal_band": outside,
                    "max_excess_rad": max((row["excess_rad"] for row in outside), default=0.)}
        self.last_tracking = copy.deepcopy(tracking)
        report = self.report
        report["mode"] = plan["tracking_policy"]["mode"]
        report["max_origin_excursion_rad"] = max(report["max_origin_excursion_rad"],
                                                max(abs(a-b) for a,b in zip(q,start)))
        if tracking["max_excess_rad"] > report["max_excess_rad"]:
            report["max_excess_rad"] = tracking["max_excess_rad"]
            report["peak_sample"] = {"sample_id": sample["sample_id"], "captured_at": sample["captured_at"],
                                     "joints_rad": copy.deepcopy(q), "outside_nominal_band": copy.deepcopy(outside)}
        if outside and report["first_outside_nominal_band"] is None:
            report["first_outside_nominal_band"] = {"sample": copy.deepcopy(sample), "tracking": copy.deepcopy(tracking)}
        return tracking

    def account(self, sample, tracking, *, maximum_s):
        """Count the whole interval when either endpoint is outside the old band."""
        at = sample["captured_at"]
        if (not _finite(at) or not _finite(self.previous_at) or at < self.previous_at
                or not _finite(maximum_s) or maximum_s <= 0):
            raise RuntimeError("Tracking accounting requires an advancing time and finite budget")
        outside = not tracking["within_nominal_band"]
        if outside or self.previous_outside:
            self.cumulative_s += at-self.previous_at
        self.previous_at, self.previous_outside = at, outside
        self.report["cumulative_outside_nominal_band_s"] = self.cumulative_s
        if self.cumulative_s > maximum_s:
            raise RuntimeError(self.label + " cumulative outside-band settling budget exceeded")

    def record_failure(self, exc, sample=None, tracking=None):
        if self.first_failure is None:
            self.first_failure = {"type": type(exc).__name__, "detail": str(exc),
                "code": getattr(exc, "code", None),
                "sample_role": "rejected_observation" if sample is not None else "latest_observation_before_failure",
                "sample": copy.deepcopy(sample if sample is not None else self.last_attempted_sample),
                "tracking": copy.deepcopy(tracking if tracking is not None else self.last_tracking)}
            self.report["first_failure"] = copy.deepcopy(self.first_failure)
