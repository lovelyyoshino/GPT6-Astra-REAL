"""Pure source consistency tests. Synthetic records never grant physical scope."""
import copy
import json
import math
import unittest
from unittest.mock import patch

from robot_tools import joint_path as jp
from robot_tools.joint_initialization import plan_joint_initialization
from robot_tools.joint_ingress import resolve_initialization_ingress
from test_joint_initialization import context, commit_origin, fresh_sample
from test_pair_initialization import InitializationFixture, initialization_context
from test_joint_path import RAW


def source(side="left", *, boundary=True, seed=False):
    ctx = context(side, historical=not seed)
    ctx["identity"]["worker_id"] = "initialize-" + side
    for label in ("origin", "current"):
        ctx[label]["identity"] = copy.deepcopy(ctx["identity"])
        if seed and boundary:
            ctx[label]["arms"][side]["joints_rad"][1:3] = [0., 0.]
    commit_origin(ctx)
    plan = plan_joint_initialization(ctx, now=100.01)
    complete = fresh_sample(ctx, 104.1)
    state = complete["arms"][side]
    state["joints_rad"] = plan["encoded_target_joints_rad"][:]
    if boundary:
        state["joints_rad"][1:3] = [-.001, .001]
    state["arm_status"]["mode_feedback"] = 1
    limits = plan["effective_joint_limits_rad"]
    result = {"schema": "piper_pair_joint_initialization_receipt_v1",
        "event_id": ctx["identity"]["worker_id"], "identity": ctx["identity"],
        "purpose": plan["purpose"], "origin": ctx["origin"],
        "target_raw": plan["target_raw"], "encoded_target_joints_rad": plan["encoded_target_joints_rad"],
        "completion_sample": complete, "plan_sha256": plan["plan_sha256"],
        "effective_joint_limits_rad": limits,
        "frame_receipts": [{"frame": frame, "outcome": "returned", "returned_at": 101.+i*.001}
                           for i, frame in enumerate(plan["frames"])],
        "strict_nominal": all(lo <= q <= hi for q, (lo, hi) in zip(state["joints_rad"], limits[side])),
        "within_feedback_tolerance": True, "completed_at": 104.11,
        "ordinary_motion_authorized": False, "physical_stop_verified": None}
    return result


def evidence(sources=None, *, selected="left"):
    sources = sources or {"left": source(), "right": None}
    record = sources[selected] or next(s for s in sources.values() if s is not None)
    identity = {**record["identity"], "arm": selected, "worker_id": "ordinary-next"}
    current = copy.deepcopy(record["completion_sample"])
    current.update(identity=identity, sample_id="current-105", captured_at=105.)
    for side, state in current["arms"].items():
        if sources[side] is not None:
            current["arms"][side] = state = copy.deepcopy(sources[side]["completion_sample"]["arms"][side])
        state["fragment_timestamps_s"] = {key: 105. for key in state["fragment_timestamps_s"]}
        state["gripper"]["foc_status"]["driver_enable_status"] = True
    cache = ({key: copy.deepcopy(sources[selected][key]) for key in
              ("event_id", "identity", "target_raw", "frame_receipts")} if sources[selected] else None)
    return {"initialization_sources": sources, "identity": identity, "origin": copy.deepcopy(current),
            "current": current, "effective_joint_limits_rad": copy.deepcopy(record["effective_joint_limits_rad"]),
            "cached_target": cache}


class JointIngressTests(unittest.TestCase):
    def setUp(self):
        blocker = patch("socket.socket", side_effect=AssertionError("offline; socket forbidden"))
        blocker.start()
        self.addCleanup(blocker.stop)

    def rejected(self, data, code=None):
        with self.assertRaises(jp.JointPathError) as raised:
            resolve_initialization_ingress(**data)
        if code:
            self.assertEqual(raised.exception.code, code)

    def test_absent_sources_leave_nominal_limits_and_inputs_unchanged(self):
        data = evidence()
        data["initialization_sources"] = None
        before = copy.deepcopy(data)
        result = resolve_initialization_ingress(**data)
        self.assertEqual(result["feedback_limits_rad"], data["effective_joint_limits_rad"])
        self.assertEqual(result["evidence"], {"left": None, "right": None})
        self.assertEqual(data, before)
        self.assertFalse(result["motion_permitted"])

    def test_completed_recovery_only_adds_j2_lower_j3_upper_observation_band(self):
        data = evidence()
        before = copy.deepcopy(data)
        result = resolve_initialization_ingress(**data)
        expected = copy.deepcopy(data["effective_joint_limits_rad"])
        expected["left"][1][0] -= .003
        expected["left"][2][1] += .003
        self.assertEqual(result["feedback_limits_rad"], expected)
        self.assertEqual(result["evidence"]["left"]["joint_indices"], [2, 3])
        self.assertFalse(result["nominal_limits_changed"])
        self.assertFalse(result["source_truth_authenticated"])
        self.assertLess(len(json.dumps(result)), 2000)
        self.assertNotIn("completion_sample", result["evidence"]["left"])
        self.assertEqual(data, before)
        result["feedback_limits_rad"]["left"][1][0] = -1.
        self.assertEqual(data, before)

    def test_exact_observation_band_and_true_excess(self):
        for axis, sign in ((1, -1), (2, 1)):
            data = evidence()
            for label in ("origin", "current"):
                data[label]["arms"]["left"]["joints_rad"][axis] = sign * .003
            resolve_initialization_ingress(**data)
            data["current"]["arms"]["left"]["joints_rad"][axis] = sign * (.003 + 1e-9)
            self.rejected(data, "ingress_left_initial_target_band")

    def test_nonboundary_axes_and_drift_are_not_excused(self):
        for axis in (0, 3, 4, 5):
            data = evidence()
            data["current"]["arms"]["left"]["joints_rad"][axis] += .00301
            self.rejected(data, "ingress_left_initial_target_band")
        data = evidence()
        data["current"]["arms"]["left"]["joints_rad"][1] = .00301
        self.rejected(data, "ingress_left_initial_target_band")

    def test_selected_cache_exactly_matches_source_and_old_event_is_not_renamed(self):
        for key, value in (("event_id", "new-event"), ("target_raw", [0]*6), ("frame_receipts", [])):
            data = evidence()
            data["cached_target"][key] = value
            self.rejected(data, "initialization_source_cache_changed")
        data = evidence()
        data["initialization_sources"]["left"]["event_id"] = "renamed"
        self.rejected(data, "initialization_source_event")

    def test_owner_epoch_connection_model_and_profile_cannot_migrate(self):
        for field in ("run_id", "owner", "epoch", "connection_id", "model", "firmware_profile"):
            data = evidence()
            data["initialization_sources"]["left"]["identity"][field] = "different"
            self.rejected(data)

    def test_two_arm_sources_allow_prior_arm_worker_and_disabled_completion_jaws(self):
        for selected in ("left", "right"):
            data = evidence({"left": source("left"), "right": source("right")}, selected=selected)
            for record in data["initialization_sources"].values():
                self.assertFalse(record["completion_sample"]["arms"][record["identity"]["arm"]]
                                 ["gripper"]["foc_status"]["driver_enable_status"])
            result = resolve_initialization_ingress(**data)
            self.assertTrue(all(result["evidence"].values()))
            self.assertTrue(all(x["joint_indices"] == [2, 3] for x in result["evidence"].values()))

    def test_seed_at_exact_boundary_can_bind_band_but_interior_seed_cannot(self):
        for boundary in (True, False):
            data = evidence({"left": source(seed=True, boundary=boundary), "right": None})
            result = resolve_initialization_ingress(**data)
            self.assertEqual(result["evidence"]["left"]["joint_indices"], [2, 3] if boundary else [])
            if not boundary:
                self.assertEqual(result["feedback_limits_rad"], data["effective_joint_limits_rad"])

    def test_missing_limits_changed_limits_and_partial_frames_refuse(self):
        for mutation in (lambda s: s.pop("effective_joint_limits_rad"),
                         lambda s: s["effective_joint_limits_rad"]["right"][0].__setitem__(0, -2.),
                         lambda s: s["frame_receipts"].pop(),
                         lambda s: s["frame_receipts"][2].update(outcome="exception"),
                         lambda s: s["frame_receipts"][2]["frame"].update(data_hex="00")):
            data = evidence()
            mutation(data["initialization_sources"]["left"])
            self.rejected(data)

    def test_completion_must_be_after_complete_send_and_truthful(self):
        for mutation in (lambda s: s["completion_sample"].update(captured_at=103.),
                         lambda s: s.update(completed_at=103.),
                         lambda s: s.update(strict_nominal=True),
                         lambda s: s.update(within_feedback_tolerance=False),
                         lambda s: s["completion_sample"]["arms"]["left"]["arm_status"].update(mode_feedback=True),
                         lambda s: s["completion_sample"]["arms"]["right"]["arm_status"].update(motion_status=False),
                         lambda s: s["completion_sample"]["arms"]["right"]["fragment_timestamps_s"].update(joint_12=101.)):
            data = evidence()
            mutation(data["initialization_sources"]["left"])
            self.rejected(data)

    def test_origin_and_every_current_fragment_must_follow_completion(self):
        for label in ("origin", "current"):
            data = evidence()
            data[label]["captured_at"] = 104.11
            self.rejected(data, "ingress_sample_precedes_completion")
            data = evidence()
            data[label]["arms"]["left"]["fragment_timestamps_s"]["joint_12"] = 104.11
            self.rejected(data, "ingress_feedback_precedes_completion")
        data = evidence()
        data["current"]["arms"]["left"]["arm_status"]["mode_feedback"] = True
        self.rejected(data, "ingress_mode_changed")

    def test_unknown_nonfinite_or_mislabeled_source_fails_closed(self):
        for mutate in (lambda s: s.update(purpose="ordinary"),
                       lambda s: s.update(ordinary_motion_authorized=True),
                       lambda s: s.update(physical_stop_verified=True),
                       lambda s: s.update(completed_at=float("nan")),
                       lambda s: s["origin"]["arms"]["left"]["joints_rad"].__setitem__(0, 4.),
                       lambda s: s["identity"].update(arm="right")):
            data = evidence()
            mutate(data["initialization_sources"]["left"])
            self.rejected(data)


class IngressCacheLifetimeTests(InitializationFixture):
    def ready_initialized_pair(self):
        self.joints = {side: [math.radians(v/1000) for v in RAW] for side in ("left", "right")}
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.open()
        for side in ("left", "right"):
            event = "init-" + side
            report = self.execute(initialization_context(self.device, side, event_id=event), event=event)
            self.assertTrue(report["ok"], report.get("errors"))
        self.assertTrue(all(self.device.joint_initialization_sources().values()))

    def test_cartesian_complete_send_invalidates_only_selected_initialization_cache(self):
        self.ready_initialized_pair()
        old_right = self.device.joint_initialization_sources()["right"]
        target = self.robots["left"].motion.origin[:]
        target[2] += .006
        result = self.device.execute("left", "move", target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids()[-4:], [0x151, 0x152, 0x153, 0x154])
        self.assertIsNone(self.device.joint_binding("left")["cached_target"])
        self.assertIsNone(self.device._joint_initializations["left"])
        self.assertEqual(self.device.joint_initialization_sources()["right"], old_right)

    def test_cartesian_partial_send_invalidates_history_and_never_retries(self):
        self.ready_initialized_pair()
        self.robots["left"].fail_id = 0x153
        target = self.robots["left"].motion.origin[:]
        target[2] += .006
        result = self.device.execute("left", "move", target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids()[-2:], [0x151, 0x152])
        self.assertIsNone(self.device.joint_binding("left")["cached_target"])
        self.assertIsNone(self.device.joint_initialization_sources()["left"])
        count = len(self.ids())
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "move", target)
        self.assertEqual(len(self.ids()), count)

    def test_jaw_action_preserves_joint_source_and_returned_sources_are_copies(self):
        self.ready_initialized_pair()
        before = self.device.joint_initialization_sources()
        result = self.device.execute("left", "gripper", .03)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids()[-1:], [0x159])
        self.assertEqual(self.device.joint_initialization_sources(), before)
        before["left"]["target_raw"][0] += 1
        self.assertNotEqual(self.device.joint_initialization_sources(), before)


if __name__ == "__main__":
    unittest.main()
