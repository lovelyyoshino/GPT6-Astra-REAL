"""Pure model/encoding checks; no sockets, robot construction or TX."""
import copy
import hashlib
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import joint_path as jp
from robot_tools.hold_transaction import joint_hold_frames
from robot_tools.model_compatibility import fk_matrix, matrix_error
from test_execution import healthy_arm


ROOT = Path(__file__).resolve().parents[3]
ART = ROOT / "artifacts/piper_x_capability_review_1791347348345247360"
IDENTITY = {"run_id": "run", "owner": "owner", "epoch": "epoch", "worker_id": "worker",
            "arm": "right", "connection_id": "connection", "model": "piper_x", "firmware_profile": "default"}
SOURCE = {"ref": "offline-test-evidence-not-live", "sha256": "a"*64}
RAW = [10000, 20000, -20000, 10000, 10000, 10000]


def sample(now, raw=None, identity=None):
    identity = copy.deepcopy(identity or IDENTITY)
    states = {s: healthy_arm(now) for s in ("left", "right")}
    for side, state in states.items():
        state["joints_rad"] = [math.radians(x/1000) for x in (raw or RAW)]
        state["arm_status"].update(teach_status=0, mode_feedback=1, motion_status=0)
    return {"sample_id": "sample-"+str(now), "identity": identity, "captured_at": now, "arms": states}


def context(raw=None, arm="right"):
    raw = raw or RAW
    identity = {**IDENTITY, "arm": arm}
    origin = sample(100., raw, identity)
    return {"schema": jp.SCHEMA, "identity": identity,
        "model_catalog": {"constants_path": str(ART / "official_sdk_constants.py"),
            "commit": jp.SDK_COMMIT, "sha256": "b72cd2a0e7499483e1313acf0e98781510cb151de592c3fe1291df712ab23c5b",
            "source_url": "https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/"+jp.SDK_COMMIT+"/pyAgxArm/api/constants.py"},
        "urdf_source": {"path": str(ART / "official_urdf/piper_x/urdf/piper_x_description.urdf"),
            "commit": jp.URDF_COMMIT, "sha256": jp.URDF_SHA256,
            "source_url": "https://raw.githubusercontent.com/agilexrobotics/agx_arm_urdf/"+jp.URDF_COMMIT+"/piper_x/urdf/piper_x_description.urdf"},
        "origin": origin, "origin_sha256": jp.evidence_sha256(origin), "current": sample(100.01, raw, identity),
        "cached_target": {"identity": identity, "event_id": "previous-complete-target", "target_raw": raw[:],
            "frame_receipts": [{"frame": frame, "outcome": "returned", "returned_at": 99.9+i*.001}
                               for i, frame in enumerate(joint_hold_frames(raw))]},
        "controller_limits": {"left": [[-3., 3.], [0., 3.2], [-3.1, 0.], [-2., 2.], [-2., 2.], [-3.2, 3.2]],
            "right": [[-3., 3.], [0., 3.2], [-3.1, 0.], [-2., 2.], [-2., 2.], [-3.2, 3.2]], "source": SOURCE},
        "geometry": {"attachment_radius_m": {"left": .02, "right": .02}, "available_clearance_m": .2,
            "workspace_min_m": [-2., -2., -2.], "workspace_max_m": [2., 2., 2.],
            "origin_sample_id": origin["sample_id"], "source": SOURCE},
        "budget": {"max_translation_m": .020, "max_rotation_rad": .05}, "unloaded_evidence": None}


def target(ctx, axis=0, delta=.001):
    q = ctx["current"]["arms"][ctx["identity"]["arm"]]["joints_rad"][:]
    q[axis] += delta
    return q


def visual_joint_context(ctx=None, requested=None, *, operation="approach"):
    """Synthetic saved RGB references, never a measured scene or permission."""
    ctx=copy.deepcopy(ctx if ctx is not None else context())
    requested=copy.deepcopy(requested if requested is not None else target(ctx))
    stamp=ctx["current"]["captured_at"]-.01
    evidence={"identity":copy.deepcopy(ctx["identity"]),"operation":operation,
        "target_raw":jp.encode_joint_target(requested)[0],"observation_id":"synthetic-joint-scene",
        "capture_id":"synthetic-joint-capture","rgb_received_at":stamp,
        "saved_rgb_evidence":{view:{"rgb_path":"/synthetic/"+view+".png",
            "artifact_sha256":str(i+1)*64,"frame_number":1,"host_received_at":stamp}
            for i,view in enumerate(("front","left_hand","right_hand"))},
        "unloaded_observation":"Synthetic: selected working gripper is empty.",
        "corridor_observation":"Synthetic: full local arm and attachment corridor visibly clear.",
        "workspace_clearance_statement":"Synthetic user statement; no measured geometry."}
    ctx["geometry"]={"schema":jp.VISUAL_GEOMETRY_SCHEMA,
        "origin_sample_id":ctx["origin"]["sample_id"],
        "source":{"ref":"pair_event:synthetic:ordinary-rgb","sha256":jp.evidence_sha256(evidence)},
        "evidence":evidence}
    return ctx,requested


def loaded_joint_context(ctx=None, requested=None, *, operation="extract_segment"):
    """Synthetic retention references, not physical grip or load qualification."""
    ctx, requested = visual_joint_context(ctx, requested)
    from robot_tools.rgb_supervision import LOADED_CONTEXT_SCHEMA
    identities = {side: {"episode_id": side+"-episode", "arm": side,
        "run_id": ctx["identity"]["run_id"], "owner": ctx["identity"]["owner"],
        "epoch": ctx["identity"]["epoch"], "object_id": side+"-object"} for side in ("left", "right")}
    roles = {}
    for role, side in (("worker", "right"), ("peer", "left")):
        state = ctx["current"]["arms"][side]
        anchor = {"joints_rad": copy.deepcopy(state["joints_rad"]),
                  "pose_m_rad": copy.deepcopy(state["pose_m_rad"]), "width_m": state["gripper"]["width_m"]}
        roles[role] = {"identity": identities[side], "revision": 2,
            "probe_event_id": side+"-probe", "probe_trace_sha256": ("a" if side == "left" else "b")*64,
            "requested_width_m": .025, "original_anchor": anchor, "local_anchor": copy.deepcopy(anchor)}
    loaded = {"schema": LOADED_CONTEXT_SCHEMA, "event_id": "loaded-event", "operation": operation,
        **roles, "object_scene": {"source_id": "source-hole", "target_id": "left-hole",
            "observation_id": ctx["geometry"]["evidence"]["observation_id"],
            "description": "Synthetic plug/socket relation; no claimed load success."}}
    evidence = ctx["geometry"]["evidence"]
    del evidence["unloaded_observation"]
    evidence.update(operation=operation, loaded_observation="Synthetic currently retained object observation",
                    loaded_context_sha256=jp.evidence_sha256(loaded))
    ctx["loaded_context"] = loaded
    ctx["geometry"]["schema"] = jp.LOADED_JOINT_PATH_SCHEMA
    ctx["geometry"]["source"]["sha256"] = jp.evidence_sha256(evidence)
    return ctx, requested


def coarse_joint_context(ctx=None, requested=None):
    """Synthetic explicit coarse RGB source; not a measured clearance."""
    base = copy.deepcopy(ctx if ctx is not None else context())
    requested = requested if requested is not None else target(base, 2, -math.radians(10))
    ctx, requested = visual_joint_context(base, requested)
    ctx["geometry"]["schema"] = jp.COARSE_JOINT_PATH_SCHEMA
    evidence = ctx["geometry"]["evidence"]
    evidence["far_from_target_observation"] = "Synthetic: both empty jaws are far from the object and contact."
    ctx["geometry"]["source"]["sha256"] = jp.evidence_sha256(evidence)
    ctx["budget"] = {"max_translation_m": .120, "max_rotation_rad": .40}
    return ctx, requested


def recover_context():
    ctx = context()
    for name in ("origin", "current"):
        ctx[name]["arms"]["right"]["joints_rad"][1:3] = [-.002, .001]
        for state in ctx[name]["arms"].values():
            state["gripper"]["foc_status"]["driver_enable_status"] = False
    ctx["origin_sha256"] = jp.evidence_sha256(ctx["origin"])
    old = RAW[:]
    old[1:3] = [0, 0]
    ctx["cached_target"]["target_raw"] = old
    ctx["cached_target"]["frame_receipts"] = [
        {"frame": frame, "outcome": "returned", "returned_at": 99.9+i*.001}
        for i, frame in enumerate(joint_hold_frames(old))]
    ctx["unloaded_evidence"] = {"origin_sample_id": ctx["origin"]["sample_id"], "source": SOURCE}
    q = ctx["current"]["arms"]["right"]["joints_rad"][:]
    q[1:3] = [0., 0.]
    return ctx, q


class JointPathTests(unittest.TestCase):
    def setUp(self):
        self.no_socket = patch("socket.socket", side_effect=AssertionError("hardware/network forbidden"))
        self.no_socket.start()
        self.addCleanup(self.no_socket.stop)

    def assert_code(self, code, call, *args, **kwargs):
        with self.assertRaises(jp.JointPathError) as caught:
            call(*args, **kwargs)
        self.assertEqual(caught.exception.code, code)

    def test_all_six_individual_axes_and_both_arms(self):
        for side in ("left", "right"):
            for axis in range(6):
                with self.subTest(side=side, axis=axis):
                    ctx = context(arm=side)
                    before = copy.deepcopy(ctx)
                    plan = jp.plan_joint_path(ctx, target(ctx, axis), now=100.01)
                    self.assertTrue(plan["candidate_valid"])
                    self.assertFalse(plan["dispatch_authorized"])
                    self.assertFalse(plan["motion_permitted"])
                    self.assertEqual(plan["hardware_commands_sent"], 0)
                    self.assertIsNone(plan["physical_stop_verified"])
                    self.assertEqual(plan["frames"], joint_hold_frames(plan["target_raw"]))
                    self.assertEqual(ctx, before)

    def test_six_axis_combined_target_and_all_mixed_prefixes(self):
        ctx = context()
        q = [v+.001 for v in target(ctx, delta=0.)]
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        self.assertEqual(len(plan["mixed_targets"]), 4)
        for row in plan["mixed_targets"]:
            n = 2*row["joint_pairs_updated"]
            self.assertEqual(row["joints_rad"][:n], plan["encoded_target_joints_rad"][:n])
            for v, lo, hi in zip(row["joints_rad"], plan["joint_envelope"]["low_rad"], plan["joint_envelope"]["high_rad"]):
                self.assertLessEqual(lo, v)
                self.assertLessEqual(v, hi)

    def test_raw_controller_pose_preserved_and_never_replaced_with_fk(self):
        ctx = context()
        raw = copy.deepcopy(ctx["current"]["arms"]["right"]["pose_m_rad"])
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        self.assertEqual(plan["controller_flange_pose"]["right"], raw)
        self.assertGreater(matrix_error(plan["model_original_flange_transform"]["right"], jp.pose_matrix(raw))["position_error_m"], .002)
        observed = jp.validate_joint_path_sample(plan, ctx["current"], now=100.01)
        self.assertEqual(observed["arms"]["right"]["controller_flange_pose"], raw)

    def test_exact_boundary_inward_fits_tighter_box_with_original_hold_budget(self):
        raw = RAW[:]
        raw[1:3] = [0, 0]
        ctx = context(raw)
        q = target(ctx, delta=0.)
        inward = math.radians(573/1000)  # Smallest encoded margin >= .010 rad.
        q[1:3] = [inward, -inward]
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        bound, hold = plan["flange_box_bound"], plan["remaining_hold_budget"]
        self.assertLess(bound["translation_m"]+hold["required_translation_m"], .020)
        self.assertLess(bound["rotation_rad"]+hold["required_rotation_rad"], .05)
        self.assertEqual(plan["budget"], ctx["budget"])
        self.assertEqual(bound["parallel_axis_rotation_group"], [2, 3, 4])
        self.assertFalse(bound["includes_attachment_geometry"])
        self.assertFalse(plan["motion_permitted"])
        # Geometry improvement does not grant a residual exception without
        # a completed, source-bound initialization on this actual connection.
        state = sample(100.02, raw)
        state["arms"]["right"]["joints_rad"][1] = -.0001
        self.assert_code("feedback_joint_limit", jp.validate_joint_path_sample,
                         plan, state, now=100.02)
        # Nor may the new geometry silently enlarge a caller's smaller budget.
        ctx["budget"]["max_rotation_rad"] = .04
        self.assert_code("insufficient_remaining_hold_budget", jp.plan_joint_path,
                         ctx, q, now=100.01)

    def test_sdk_urdf_j6_conflict_is_explicit_and_intersection_not_widened(self):
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        self.assertFalse(plan["j6_source_disagreement"]["resolved_by_widening"])
        self.assertGreater(plan["j6_source_disagreement"]["sdk_rad"][1], 3.)
        self.assertLess(plan["effective_joint_limits_rad"]["right"][5][1], 2.1)
        for raw, rad in zip(plan["effective_joint_limits_raw"]["right"], plan["effective_joint_limits_rad"]["right"]):
            self.assertGreaterEqual(raw[0]*jp.RAD_PER_RAW, rad[0])
            self.assertLessEqual(raw[1]*jp.RAD_PER_RAW, rad[1])

    def test_controller_limit_narrowing_is_used(self):
        ctx = context()
        ctx["controller_limits"]["right"][0][1] = .175
        self.assert_code("target_joint_limit", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_target_limits_step_limits_margin_and_zero_action(self):
        ctx = context()
        for axis in range(6):
            self.assert_code("joint_step_limit", jp.plan_joint_path, ctx, target(ctx, axis, .026), now=100.01)
        self.assert_code("zero_joint_action", jp.plan_joint_path, ctx, target(ctx, delta=0.), now=100.01)
        q = target(ctx)
        q[5] = 2.2
        self.assert_code("target_joint_limit", jp.plan_joint_path, ctx, q, now=100.01)
        raw = RAW[:]
        raw[1] = 500
        ctx = context(raw)
        self.assert_code("target_margin", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_source_hashes_and_physical_model_cannot_be_relabelled(self):
        for area, field, value, code in (("model_catalog", "sha256", "0"*64, "model_source_invalid"),
                ("urdf_source", "sha256", "0"*64, "unpinned_urdf_model"),
                ("identity", "model", "piper", "physical_model_required"),
                ("identity", "firmware_profile", "v189", "unsupported_firmware_profile")):
            ctx = context()
            ctx[area][field] = value
            self.assert_code(code, jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_long_controller_limit_file_ref_survives_both_planners(self):
        from robot_tools import joint_initialization as ji

        # Same run/epoch/content-addressed layout as the real provider, but an
        # isolated synthetic file. No controller or scene evidence is claimed.
        payload = b'{"synthetic_controller_limits":true}\n'
        digest = hashlib.sha256(payload).hexdigest()
        with tempfile.TemporaryDirectory() as temporary:
            path = (Path(temporary) / "GPT6-Astra-REAL" / "projects" / "piperx_cloth_demo" / "runs"
                    / "pair_plug_live_validation_1791357436415587512"
                    / "joint_sources" / "epochs" / ("e"*64)
                    / ("controller_limits_"+digest+".json"))
            path.parent.mkdir(parents=True)
            path.write_bytes(payload)
            self.assertGreater(len(str(path)), 273)
            source = {"ref": str(path), "sha256": digest}
            ctx = context()
            ctx["controller_limits"]["source"] = source
            plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
            self.assertEqual(plan["controller_limits"]["source"], source)

            ctx["schema"] = ji.SCHEMA
            ctx.pop("cached_target")
            ctx.pop("budget")
            for label in ("origin", "current"):
                for state in ctx[label]["arms"].values():
                    state["arm_status"]["mode_feedback"] = 0
                    state["gripper"]["foc_status"]["driver_enable_status"] = False
            ctx["origin_sha256"] = jp.evidence_sha256(ctx["origin"])
            ctx["unloaded_evidence"] = {
                "origin_sample_id": ctx["origin"]["sample_id"], "source": SOURCE}
            initial = ji.plan_joint_initialization(ctx, now=100.01)
            self.assertEqual(initial["purpose"], "seed_current")
            self.assertEqual(initial["sources"]["controller_limits"], source)
            self.assertEqual(initial["hardware_commands_sent"], 0)
            self.assertFalse(initial["motion_permitted"])

    def test_source_ref_validation_does_not_relax_identity_or_hash(self):
        for ref in (None, 12, "", " ref", "ref ", "r"*4097,
                    "a\x00b", "a\tb", "a\nb", "a\x7fb", "a\x85b"):
            with self.subTest(ref=repr(ref)):
                self.assert_code("invalid_identifier", jp._source,
                                 {"ref": ref, "sha256": "a"*64})
        source = {"ref": "r"*4096, "sha256": "a"*64}
        self.assertEqual(jp._source(source), source)
        self.assert_code("invalid_sha256", jp._source,
                         {"ref": "r"*273, "sha256": "not-a-sha256"})
        ctx = context()
        ctx["identity"]["run_id"] = "r"*257
        self.assert_code("invalid_identifier", jp.plan_joint_path,
                         ctx, target(ctx), now=100.01)

    def test_missing_source_geometry_or_limits_never_get_defaults(self):
        for key, code in (("geometry", "invalid_input_schema"), ("controller_limits", "invalid_input_schema"),
                          ("cached_target", "invalid_input_schema")):
            ctx = context()
            del ctx[key]
            self.assert_code(code, jp.plan_joint_path, ctx, target(ctx), now=100.01)
        ctx = context()
        ctx["geometry"]["source"] = None
        self.assert_code("missing_evidence_source", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_cached_incomplete_unknown_wrong_worker_and_far_goal_reject(self):
        for change, code in (("partial", "cached_target_partial_or_unknown"),
                ("unknown", "cached_target_partial_or_unknown"), ("connection", "cached_target_identity_changed"),
                ("far", "cached_target_not_at_original_anchor")):
            ctx = context()
            cache = ctx["cached_target"]
            if change == "partial":
                cache["frame_receipts"].pop()
            elif change == "unknown":
                cache["frame_receipts"][1]["outcome"] = "unknown"
            elif change == "connection":
                cache["identity"] = {**cache["identity"], "connection_id": "other"}
            else:
                raw = cache["target_raw"][:]
                raw[0] += 1000
                cache["target_raw"] = raw
                for receipt, frame in zip(cache["frame_receipts"], joint_hold_frames(raw)):
                    receipt["frame"] = frame
            self.assert_code(code, jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_old_origin_commitment_and_later_plan_tampering_reject(self):
        ctx = context()
        ctx["origin"]["arms"]["right"]["joints_rad"][0] += .001
        self.assert_code("original_anchor_changed", jp.plan_joint_path, ctx, target(ctx), now=100.01)
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        plan["origin"]["arms"]["right"]["joints_rad"][0] += .001
        self.assert_code("plan_changed", jp.validate_joint_path_sample, plan, ctx["current"], now=100.01)

    def test_prior_completed_worker_cache_is_preserved_not_relabelled(self):
        ctx = context()
        ctx["cached_target"]["identity"] = {**ctx["identity"], "worker_id": "prior-completed-worker"}
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        self.assertEqual(plan["cached_target"]["identity"]["worker_id"], "prior-completed-worker")
        self.assertEqual(plan["identity"]["worker_id"], "worker")
        ctx["cached_target"]["identity"]["epoch"] = "old-owner-epoch"
        self.assert_code("cached_target_identity_changed", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_cached_frame_type_and_protocol_flags_are_exact(self):
        for field, value in (("arbitration_id", float(0x151)), ("is_extended_id", 0), ("is_fd", True)):
            ctx = context()
            ctx["cached_target"]["frame_receipts"][0]["frame"][field] = value
            self.assert_code("cached_target_partial_or_unknown", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_no_reanchoring_on_successive_observations(self):
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        before = copy.deepcopy(plan)
        first = sample(100.02)
        first["arms"]["left"]["joints_rad"][5] += .002
        jp.validate_joint_path_sample(plan, first, now=100.02)
        second = sample(100.03)
        second["arms"]["left"]["joints_rad"][5] += .004
        self.assert_code("stationary_joint_anchor", jp.validate_joint_path_sample, plan, second, now=100.03)
        self.assertEqual(plan, before)

    def test_stale_feedback_and_pre_dispatch_peer_drift_reject(self):
        ctx = context()
        self.assert_code("stale_sample", jp.plan_joint_path, ctx, target(ctx), now=100.1)
        ctx["current"]["arms"]["left"]["pose_m_rad"][0] += .001
        self.assert_code("controller_relative_pose_envelope", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_hold_reserve_cannot_be_spent_by_the_requested_step(self):
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        reserved = plan["remaining_hold_budget"]
        self.assertGreaterEqual(reserved["translation_m"], reserved["required_translation_m"])
        self.assertGreaterEqual(reserved["rotation_rad"], reserved["required_rotation_rad"])
        ctx["budget"]["max_translation_m"] = .005
        self.assert_code("insufficient_remaining_hold_budget", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_clearance_relative_sweep_and_workspace(self):
        ctx = context()
        ctx["geometry"]["available_clearance_m"] = .006
        self.assert_code("relative_sweep_exceeds_clearance", jp.plan_joint_path, ctx, target(ctx), now=100.01)
        ctx = context()
        ctx["geometry"]["workspace_min_m"] = [.9, .9, .9]
        self.assert_code("model_workspace_envelope", jp.plan_joint_path, ctx, target(ctx), now=100.01)

    def test_explicit_startup_recovery_keeps_nominal_limits_and_disabled_jaws(self):
        ctx, q = recover_context()
        self.assert_code("jaw_enable_unknown_or_disabled", jp.plan_joint_path, ctx, q, now=100.01)
        plan = jp.plan_joint_path(ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)
        self.assertEqual(plan["encoded_target_joints_rad"][1:3], [0., 0.])
        self.assertEqual(plan["effective_joint_limits_rad"]["right"][1][0], 0.)
        self.assertEqual(plan["origin"]["arms"]["right"]["joints_rad"][1], -.002)
        self.assertFalse(plan["motion_permitted"])

    def test_recovery_does_not_auto_select_or_accept_wrong_boundary(self):
        ctx, q = recover_context()
        ctx["unloaded_evidence"] = None
        self.assert_code("explicit_unloaded_evidence_required", jp.plan_joint_path, ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)
        ctx, q = recover_context()
        q[1] = .001
        self.assert_code("recovery_requires_exact_nearest_boundary", jp.plan_joint_path, ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)

    def test_recovery_observation_band_is_frozen_not_reanchored(self):
        ctx, q = recover_context()
        plan = jp.plan_joint_path(ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)
        current = copy.deepcopy(ctx["current"])
        current["arms"]["right"]["joints_rad"][1] -= .00001
        jp.validate_joint_path_sample(plan, current, now=100.01)
        current["arms"]["right"]["joints_rad"][1] = -.002-.00301
        self.assert_code("feedback_joint_limit", jp.validate_joint_path_sample, plan, current, now=100.01)

    def test_recovery_non_recovery_axes_requested_and_encoded_change_capped(self):
        ctx, q = recover_context()
        q[5] += .0031
        self.assert_code("joint_step_limit", jp.plan_joint_path, ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)

    def test_jaw_drift_enable_change_raw_pose_and_joint_tracking_are_independent(self):
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        for change, code in (("jaw", "jaw_anchor_drift"), ("raw", "controller_relative_pose_envelope"),
                             ("joint", "joint_tracking_envelope"), ("mode", "same_move_j_mode_required")):
            state = sample(100.02)
            if change == "jaw":
                state["arms"]["right"]["gripper"]["width_m"] += .001
            elif change == "raw":
                state["arms"]["right"]["pose_m_rad"][0] += .021
            elif change == "joint":
                state["arms"]["right"]["joints_rad"][5] += .004
            else:
                state["arms"]["right"]["arm_status"]["mode_feedback"] = 2
            self.assert_code(code, jp.validate_joint_path_sample, plan, state, now=100.02)

    def test_finite_explicit_six_values_and_encoding_range(self):
        for value, code in ((float("nan"), "invalid_number"), (True, "invalid_number"), (1e308, "joint_encoding_overflow")):
            q = [0.] * 6
            q[0] = value
            self.assert_code(code, jp.encode_joint_target, q)
        self.assert_code("invalid_vector", jp.encode_joint_target, [0.] * 5)

    def test_half_millidegree_disagreement_and_encoded_margin_reject(self):
        q = [0.] * 6
        q[1] = 0.010306169233026517
        self.assert_code("ambiguous_joint_quantization", jp.encode_joint_target, q)
        raw = RAW[:]
        raw[3] = -87854
        ctx = context(raw)
        ctx["controller_limits"]["right"][3][0] = -1.55334
        q = target(ctx, delta=0.)
        q[3] = -1.55334+.010
        self.assertLess(jp.encode_joint_target(q)[1][3]-(-1.55334), .010)
        self.assert_code("target_margin", jp.plan_joint_path, ctx, q, now=100.01)

    def test_fifteen_mm_model_endpoint_cap_is_independent_of_clearance(self):
        raw = [5730, 5730, -160428, 5730, 5730, 5730]
        ctx = context(raw)
        self.assert_code("model_endpoint_displacement", jp.plan_joint_path, ctx, target(ctx, delta=.024), now=100.01)

    def test_boundary_recovery_limit_is_not_a_generic_all_axis_exception(self):
        ctx, q = recover_context()
        for name in ("origin", "current"):
            ctx[name]["arms"]["right"]["joints_rad"][1] = -.10001
        ctx["origin_sha256"] = jp.evidence_sha256(ctx["origin"])
        self.assert_code("origin_joint_limit", jp.plan_joint_path, ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)
        ctx, q = recover_context()
        for name in ("origin", "current"):
            ctx[name]["arms"]["right"]["joints_rad"][3] = 1.6
        ctx["origin_sha256"] = jp.evidence_sha256(ctx["origin"])
        self.assert_code("origin_joint_limit", jp.plan_joint_path, ctx, q, now=100.01, recovery_mode=jp.RECOVERY_MODE)

    def test_official_sdk_fk_numerical_comparison_no_device_constructed(self):
        from robot_tools import arms
        from test_backend import PROFILE
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        from pyAgxArm.utiles.mdh_kinematics import fk_from_mdh, get_mdh
        ctx = context()
        plan = jp.plan_joint_path(ctx, target(ctx), now=100.01)
        for q in ([0.] * 6, [v*.01 for v in (1, 50, -40, 20, -30, 10)], plan["encoded_target_joints_rad"]):
            vendor = fk_from_mdh(get_mdh("piper_x"), q)
            observed = matrix_error(fk_matrix(plan["model"]["mdh"], q), jp.pose_matrix(list(vendor)))
            self.assertLess(observed["position_error_m"], 1e-12)
            self.assertLess(observed["so3_error_rad"], 1e-12)


class VisualJointPathTests(unittest.TestCase):
    setUp=JointPathTests.setUp
    assert_code=JointPathTests.assert_code

    def test_rgb_empty_jaw_release_retreat_keeps_ordinary_bounds_without_metric_geometry(self):
        for arm in ("left", "right"):
            base = context(arm=arm)
            ctx, q = visual_joint_context(base, operation="release_retreat")
            plan = jp.plan_joint_path(ctx, q, now=100.01)
            reference = jp.plan_joint_path(base, q, now=100.01)
            for key in ("target_raw", "frames", "joint_envelope", "flange_box_bound",
                        "remaining_hold_budget", "budget", "effective_joint_limits_rad"):
                self.assertEqual(plan[key], reference[key])
            self.assertEqual(plan["geometry"]["evidence"]["operation"], "release_retreat")
            self.assertIsNone(plan["loaded_context"])
            self.assertFalse(plan["loaded_observation_only"])
            self.assertFalse(plan["hold_supported"])
            self.assertFalse(plan["dispatch_authorized"])
            self.assertFalse(plan["metric_clearance_checked"])
            self.assertEqual(plan["hold_policy"], "latch_only")
            # Release tokens and actual empty-finger evidence belong to the
            # host/device boundary; this pure plan grants neither one.
            self.assertNotIn("release_confirmed", plan)

    def test_rgb_release_retreat_cannot_import_loaded_context_or_relax_step(self):
        ctx, q = visual_joint_context(operation="release_retreat")
        ctx["loaded_context"] = loaded_joint_context()[0]["loaded_context"]
        self.assert_code("loaded_context_requires_loaded_schema", jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = visual_joint_context(operation="release_retreat")
        q[0] += .025
        ctx["geometry"]["evidence"]["target_raw"] = jp.encode_joint_target(q)[0]
        ctx["geometry"]["source"]["sha256"] = jp.evidence_sha256(ctx["geometry"]["evidence"])
        self.assert_code("joint_step_limit", jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = loaded_joint_context(operation="release_retreat")
        self.assert_code("visual_joint_operation", jp.plan_joint_path, ctx, q, now=100.01)

    def test_visual_all_axes_keep_exact_targets_boxes_and_numeric_hold_reserve(self):
        for arm in ("left","right"):
            for axis in range(6):
                base=context(arm=arm)
                q=target(base,axis)
                ctx,q=visual_joint_context(base,q,operation="align")
                before=copy.deepcopy(ctx)
                plan=jp.plan_joint_path(ctx,q,now=100.01)
                metric=jp.plan_joint_path(base,q,now=100.01)
                self.assertEqual(ctx,before)
                for key in ("target_raw","frames","joint_envelope","flange_box_bound",
                            "remaining_hold_budget","budget","mixed_targets","effective_joint_limits_rad"):
                    self.assertEqual(plan[key],metric[key])
                self.assertEqual(plan["spatial_admission_mode"],"rgb_supervised")
                self.assertEqual(plan["hold_policy"],"latch_only")
                for key in ("hold_supported","metric_clearance_checked","metric_collision_checked",
                            "absolute_workspace_checked","dispatch_authorized","motion_permitted"):
                    self.assertFalse(plan[key])
                for key in ("sweep_axis_radii_m","active_sweep_bound_m","passive_sweep_bound_m",
                            "relative_sweep_with_hold_bound_m"):
                    self.assertIsNone(plan[key])
                for key in ("attachment_radius_m","available_clearance_m","workspace_min_m","workspace_max_m"):
                    self.assertNotIn(key,plan["geometry"])
                self.assertEqual(plan["controller_flange_pose"],metric["controller_flange_pose"])
                self.assertIsNone(plan["physical_stop_verified"])
                self.assertEqual(plan["hardware_commands_sent"],0)

    def test_visual_target_operation_identity_and_source_cannot_be_substituted(self):
        for mutation,code in (
            (lambda e:e.update(operation="extract"),"visual_joint_operation"),
            (lambda e:e["target_raw"].__setitem__(0,e["target_raw"][0]+1),"visual_joint_target_changed"),
            (lambda e:e["target_raw"].__setitem__(0,True),"visual_joint_target_changed"),
            (lambda e:e["identity"].update(owner="other"),"visual_identity_mismatch"),
            (lambda e:e.update(corridor_observation="Changed without hash"),"visual_evidence_hash_mismatch")):
            ctx,q=visual_joint_context()
            mutation(ctx["geometry"]["evidence"])
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=visual_joint_context()
        ctx["geometry"]["source"]["sha256"]="0"*64
        self.assert_code("visual_evidence_hash_mismatch",jp.plan_joint_path,ctx,q,now=100.01)

    def test_visual_requires_actual_cache_limits_and_unchanged_original(self):
        for mutation,code in (
            (lambda c:c.update(cached_target=None),"known_cached_target_required"),
            (lambda c:c["cached_target"]["frame_receipts"].pop(),"cached_target_partial_or_unknown"),
            (lambda c:c["cached_target"].update(identity={**c["identity"],"connection_id":"other"}),"cached_target_identity_changed"),
            (lambda c:c["controller_limits"].update(source=None),"missing_evidence_source"),
            (lambda c:c["origin"]["arms"]["right"]["joints_rad"].__setitem__(0,.2),"original_anchor_changed")):
            ctx,q=visual_joint_context()
            mutation(ctx)
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)

    def test_no_metric_or_initialization_schema_fallback_and_no_rgb_recovery(self):
        for geometry in (None,{}, {"schema":"piper_rgb_supervised_initialization_v1"}):
            ctx=context()
            ctx["geometry"]=geometry
            self.assert_code("geometry_schema",jp.plan_joint_path,ctx,target(ctx),now=100.01)
        ctx,q=visual_joint_context()
        ctx["geometry"]["available_clearance_m"]=1.
        self.assert_code("visual_geometry_schema",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=recover_context()
        ctx,q=visual_joint_context(ctx,q)
        self.assert_code("visual_joint_recovery_unsupported",jp.plan_joint_path,
                         ctx,q,now=100.01,recovery_mode=jp.RECOVERY_MODE)

    def test_rgb_ingress_retains_exact_completed_initialization_sources(self):
        from test_joint_ingress import source, evidence
        records={side:source(side) for side in ("left","right")}
        data=evidence(records,selected="right")
        ctx=context(arm="right")
        for key in ("identity","origin","current","cached_target","initialization_sources"):
            ctx[key]=copy.deepcopy(data[key])
        ctx["origin_sha256"]=jp.evidence_sha256(ctx["origin"])
        q=ctx["current"]["arms"]["right"]["joints_rad"][:]
        q[1:3]=[math.radians(.573),-math.radians(.573)]
        ctx,q=visual_joint_context(ctx,q)
        plan=jp.plan_joint_path(ctx,q,now=105.)
        self.assertIsNotNone(plan["initialization_ingress"])
        self.assertFalse(plan["hold_reference_within_nominal_limits"])
        self.assertFalse(plan["hold_supported"])
        self.assertEqual(plan["hold_policy"],"latch_only")
        self.assertEqual(plan["initialization_ingress"]["feedback_limits_rad"]["right"][1][0],-.003)
        changed=copy.deepcopy(ctx)
        changed["cached_target"]["target_raw"][0]+=1
        self.assert_code("initialization_source_cache_changed",jp.plan_joint_path,
                         changed,q,now=105.)
        changed=copy.deepcopy(ctx)
        changed.pop("initialization_sources")
        self.assert_code("origin_joint_limit",jp.plan_joint_path,changed,q,now=105.)

    def test_visual_scene_complete_frames_freshness_skew_and_descriptions(self):
        for mutation,code in (
            (lambda e:e["saved_rgb_evidence"].pop("front"),"visual_rgb_views_required"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(frame_number=True),"visual_rgb_frame_number"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(artifact_sha256="bad"),"invalid_sha256"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(host_received_at=100.02),"visual_rgb_expired"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(host_received_at=99.7),"visual_rgb_scene_mismatch"),
            (lambda e:e.update(unloaded_observation=""),"visual_description_required")):
            ctx,q=visual_joint_context()
            mutation(ctx["geometry"]["evidence"])
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        observed=sample(129.99)
        jp.validate_joint_path_sample(plan,observed,now=129.99)
        observed=sample(130.01)
        self.assert_code("visual_rgb_expired",jp.validate_joint_path_sample,plan,observed,now=130.01)

    def test_visual_preserves_all_numeric_limits_and_strict_active_phase(self):
        for change,code in (
            (lambda c,q:q.__setitem__(0,q[0]+.025),"joint_step_limit"),
            (lambda c,q:q.__setitem__(1,.009),"joint_step_limit"),
            (lambda c,q:c["budget"].update(max_translation_m=.021),"budget_enlarged"),
            (lambda c,q:c["budget"].update(max_rotation_rad=.01),"insufficient_remaining_hold_budget")):
            ctx,q=visual_joint_context()
            change(ctx,q)
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)
        raw=RAW[:]
        raw[1]=500
        base=context(raw)
        ctx,q=visual_joint_context(base,target(base))
        self.assert_code("target_margin",jp.plan_joint_path,ctx,q,now=100.01)
        raw=[5730,5730,-160428,5730,5730,5730]
        base=context(raw)
        ctx,q=visual_joint_context(base,target(base,delta=.024))
        self.assert_code("model_endpoint_displacement",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        for mutate,code in (
            (lambda s:s["arms"]["right"]["joints_rad"].__setitem__(4,s["arms"]["right"]["joints_rad"][4]+.015743),"joint_tracking_envelope"),
            (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(0,.2),"stationary_joint_anchor"),
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(0,s["arms"]["right"]["pose_m_rad"][0]+.0201),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(5,s["arms"]["right"]["pose_m_rad"][5]+.0501),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["left"]["gripper"].update(width_m=.03),"jaw_anchor_drift")):
            observed=sample(100.02)
            mutate(observed)
            self.assert_code(code,jp.validate_joint_path_sample,plan,observed,now=100.02)
        self.assert_code("invalid_phase",jp.validate_joint_path_sample,
                         plan,sample(100.02),now=100.02,phase="startup_settling")
        self.assert_code("stale_sample",jp.validate_joint_path_sample,plan,sample(100.02),now=100.08)


class VisualJointSettlingTests(unittest.TestCase):
    setUp=JointPathTests.setUp
    assert_code=JointPathTests.assert_code

    def test_new_rx_policy_leaves_strict_box_and_hold_reserve_unchanged(self):
        base=context()
        q=target(base,5,.01)
        ctx,q=visual_joint_context(base,q)
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        metric=jp.plan_joint_path(base,q,now=100.01)
        self.assertEqual(plan["tracking_policy"],{"mode":"bounded_postsend_settling",
            "transient_band_rad":.025,"max_cumulative_outside_band_s":1.,
            "max_origin_excursion_rad":.025+.003,"settle_tolerance_rad":.003})
        for key in ("joint_envelope","flange_box_bound","remaining_hold_budget","budget","frames","target_raw"):
            self.assertEqual(plan[key],metric[key])
        self.assertEqual(plan["flange_box_bound_scope"],"strict_sending_and_nominal_tracking_only")
        self.assertFalse(plan["postsend_entire_box_admitted"])
        self.assertGreater(plan["postsend_flange_box_bound"]["rotation_rad"],.05)
        q0=plan["origin"]["arms"]["right"]["joints_rad"]
        for i,(a,b) in enumerate(zip(q0,plan["encoded_target_joints_rad"])):
            self.assertEqual(plan["postsend_joint_envelope"]["low_rad"][i],max(min(a,b)-.025,a-.028))
            self.assertEqual(plan["postsend_joint_envelope"]["high_rad"][i],min(max(a,b)+.025,a+.028))
        self.assertEqual(metric["tracking_policy"]["mode"],"strict")
        self.assertEqual(metric["postsend_joint_envelope"],metric["joint_envelope"])

    def test_short_j5_observation_has_no_time_arrival_or_dispatch_authority(self):
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        observed=sample(100.02)
        observed["arms"]["right"]["joints_rad"][4]+=.015743
        before=copy.deepcopy((plan,observed))
        result=jp.validate_joint_path_sample(plan,observed,now=100.02,phase="settling")
        self.assertEqual((plan,observed),before)
        tracking=result["tracking"]
        self.assertFalse(tracking["within_nominal_band"])
        self.assertFalse(tracking["cumulative_time_checked"])
        self.assertEqual(tracking["outside_nominal_band"][0]["joint_index"],5)
        self.assertAlmostEqual(tracking["max_excess_rad"],.015743-.003)
        self.assertFalse(result["motion_permitted"])
        self.assertFalse(result["qualification_granted"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assert_code("joint_tracking_envelope",jp.validate_joint_path_sample,
                         plan,observed,now=100.02,phase="active")
        self.assert_code("stationary_joint_anchor",jp.validate_joint_path_sample,
                         plan,observed,now=100.02,phase="pre_dispatch")

    def test_return_to_strict_band_does_not_establish_stability_or_reset_any_budget(self):
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        observed=sample(100.02)
        result=jp.validate_joint_path_sample(plan,observed,now=100.02,phase="settling")
        self.assertEqual(result["tracking"],{"within_nominal_band":True,"outside_nominal_band":[],
            "max_excess_rad":0.,"cumulative_time_checked":False})
        self.assertNotIn("arrival_confirmed",result)
        self.assertNotIn("observed_stable",result)
        self.assertEqual(plan["tracking_policy"]["max_cumulative_outside_band_s"],1.)

    def test_ordinary_origin_cap_is_028_and_interval_halfwidth_is_025(self):
        ctx=context()
        ctx,q=visual_joint_context(ctx,target(ctx,5,.01))
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        origin=plan["origin"]["arms"]["right"]["joints_rad"][5]
        for value in (origin+.028,origin-.025):
            observed=sample(100.02)
            observed["arms"]["right"]["joints_rad"][5]=value
            jp.validate_joint_path_sample(plan,observed,now=100.02,phase="settling")
        for value in (origin+.028000001,origin-.025000001):
            observed=sample(100.02)
            observed["arms"]["right"]["joints_rad"][5]=value
            self.assert_code("joint_tracking_envelope",jp.validate_joint_path_sample,
                             plan,observed,now=100.02,phase="settling")

    def test_wider_rx_box_is_not_a_whole_box_pose_admission(self):
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        observed=sample(100.02)
        observed["arms"]["right"]["joints_rad"]=[v+.0249 for v in observed["arms"]["right"]["joints_rad"]]
        for v,lo,hi in zip(observed["arms"]["right"]["joints_rad"],
                           plan["postsend_joint_envelope"]["low_rad"],plan["postsend_joint_envelope"]["high_rad"]):
            self.assertLessEqual(lo,v)
            self.assertLessEqual(v,hi)
        self.assert_code("model_relative_pose_envelope",jp.validate_joint_path_sample,
                         plan,observed,now=100.02,phase="settling")

    def test_settling_keeps_hard_feedback_peer_jaw_raw_pose_and_time_guards(self):
        ctx,q=visual_joint_context()
        # A narrow controller limit catches a transient that the wider RX
        # interval alone would allow; numerical limits remain authoritative.
        ctx["controller_limits"]["right"][5][1]=ctx["origin"]["arms"]["right"]["joints_rad"][5]+.015
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        for mutate,code in (
            (lambda s:s["arms"]["right"]["joints_rad"].__setitem__(5,s["arms"]["right"]["joints_rad"][5]+.016),"feedback_joint_limit"),
            (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(0,.2),"stationary_joint_anchor"),
            (lambda s:s["arms"]["left"]["gripper"].update(width_m=.03),"jaw_anchor_drift"),
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(0,s["arms"]["right"]["pose_m_rad"][0]+.0201),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(5,s["arms"]["right"]["pose_m_rad"][5]+.0501),"controller_relative_pose_envelope")):
            observed=sample(100.02)
            mutate(observed)
            self.assert_code(code,jp.validate_joint_path_sample,plan,observed,now=100.02,phase="settling")
        self.assert_code("stale_sample",jp.validate_joint_path_sample,plan,sample(100.02),now=100.08,phase="settling")
        self.assert_code("visual_rgb_expired",jp.validate_joint_path_sample,plan,sample(130.01),now=130.01,phase="settling")

    def test_metric_settling_and_altered_policy_are_rejected(self):
        ctx=context()
        plan=jp.plan_joint_path(ctx,target(ctx),now=100.01)
        self.assert_code("settling_requires_rgb_supervision",jp.validate_joint_path_sample,
                         plan,sample(100.02),now=100.02,phase="settling")
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        plan["tracking_policy"]["max_origin_excursion_rad"]=.1
        self.assert_code("plan_changed",jp.validate_joint_path_sample,
                         plan,sample(100.02),now=100.02,phase="settling")


class LoadedJointPathTests(unittest.TestCase):
    setUp = JointPathTests.setUp
    assert_code = JointPathTests.assert_code

    def rehash(self, ctx):
        ctx["geometry"]["evidence"]["loaded_context_sha256"] = jp.evidence_sha256(ctx["loaded_context"])
        ctx["geometry"]["source"]["sha256"] = jp.evidence_sha256(ctx["geometry"]["evidence"])

    def test_first_contact_candidate_needs_refs_but_no_previous_success_claim(self):
        for operation in ("extract_segment", "transport", "insert_segment"):
            with self.subTest(operation=operation):
                ctx, q = loaded_joint_context(operation=operation)
                before = copy.deepcopy(ctx)
                plan = jp.plan_joint_path(ctx, q, now=100.01)
                self.assertEqual(ctx, before)
                self.assertEqual(plan["loaded_context"], ctx["loaded_context"])
                self.assertTrue(plan["loaded_observation_only"])
                self.assertFalse(plan["hold_supported"])
                self.assertEqual(plan["hold_policy"], "latch_only")
                for key in ("contact_force_verified", "object_progress_verified", "dispatch_authorized", "motion_permitted"):
                    self.assertFalse(plan[key])
                self.assertEqual(plan["contact_target_limit"], None if operation == "transport" else
                                 {"translation_m": .002, "rotation_rad": .01})
                self.assertNotIn("workspace_min_m", plan["geometry"])
                self.assertNotIn("grasp_verified", ctx["loaded_context"])

    def test_contact_translation_is_capped_but_transport_uses_existing_bound(self):
        base = context()
        q = target(base, axis=2, delta=.006)
        for operation in ("extract_segment", "insert_segment"):
            ctx, q = loaded_joint_context(base, q, operation=operation)
            self.assert_code("loaded_contact_target_translation", jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = loaded_joint_context(base, q, operation="transport")
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        self.assertGreater(plan["model_endpoint_displacement_m"], .002)
        self.assertLessEqual(plan["model_endpoint_displacement_m"], .015)

    def test_contact_rotation_caps_requested_and_quantized_targets(self):
        for delta in (.0101, .0099999):
            base = context()
            q = target(base, axis=5, delta=delta)
            ctx, q = loaded_joint_context(base, q)
            if delta < .01:
                encoded = jp.encode_joint_target(q)[1]
                self.assertGreater(encoded[5]-base["origin"]["arms"]["right"]["joints_rad"][5], .01)
            self.assert_code("loaded_contact_target_rotation", jp.plan_joint_path, ctx, q, now=100.01)

    def test_contact_limit_uses_current_as_well_as_frozen_origin(self):
        base = context()
        q = target(base, axis=5, delta=.009)
        base["current"]["arms"]["right"]["joints_rad"][5] -= .0015
        ctx, q = loaded_joint_context(base, q)
        self.assert_code("loaded_contact_target_rotation", jp.plan_joint_path, ctx, q, now=100.01)

    def test_episode_identity_revision_finite_anchors_and_schema_are_strict(self):
        for mutation, code in (
            (lambda c: c["worker"]["identity"].update(owner="old-owner"), "loaded_episode_binding"),
            (lambda c: c["peer"]["identity"].update(arm="right"), "loaded_episode_binding"),
            (lambda c: c["worker"].update(revision=True), "loaded_episode_revision"),
            (lambda c: c["peer"]["local_anchor"]["joints_rad"].__setitem__(0, float("nan")), "invalid_number"),
            (lambda c: c["peer"]["original_anchor"].update(width_m=.071), "loaded_anchor_width"),
            (lambda c: c["worker"].update(requested_width_m=.056), "loaded_jaw_target"),
            (lambda c: c.update(grasp_verified=True), "loaded_context_schema"),
            (lambda c: c["worker"].update(contact_support_verified=True), "loaded_episode_schema"),
            (lambda c: c["object_scene"].update(observation_id="old-scene"), "loaded_object_scene_mismatch")):
            with self.subTest(code=code):
                ctx, q = loaded_joint_context()
                mutation(ctx["loaded_context"])
                # Invalid NaN cannot be hashed; shape/numeric checks occur
                # before the context hash comparison and still reject it.
                if code != "invalid_number":
                    self.rehash(ctx)
                self.assert_code(code, jp.plan_joint_path, ctx, q, now=100.01)

    def test_hash_scene_operation_and_target_bind_one_loaded_event(self):
        for mutation, code in (
            (lambda c: c["loaded_context"].update(event_id="another-event"), "loaded_context_hash_mismatch"),
            (lambda c: c["loaded_context"].update(operation="insert_segment"), "loaded_operation_mismatch"),
            (lambda c: c["geometry"]["evidence"]["target_raw"].__setitem__(0, 99), "visual_joint_target_changed")):
            ctx, q = loaded_joint_context()
            mutation(ctx)
            ctx["geometry"]["source"]["sha256"] = jp.evidence_sha256(ctx["geometry"]["evidence"])
            self.assert_code(code, jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = loaded_joint_context()
        del ctx["loaded_context"]
        self.assert_code("loaded_context_schema", jp.plan_joint_path, ctx, q, now=100.01)

    def test_loaded_schema_cannot_be_used_for_ordinary_left_or_recovery(self):
        ctx, q = loaded_joint_context(operation="approach")
        self.assert_code("visual_joint_operation", jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = loaded_joint_context(context(arm="left"))
        self.assert_code("loaded_worker_arm", jp.plan_joint_path, ctx, q, now=100.01)
        ctx, q = loaded_joint_context()
        ctx["geometry"]["schema"] = jp.VISUAL_GEOMETRY_SCHEMA
        self.assert_code("loaded_context_requires_loaded_schema", jp.plan_joint_path, ctx, q, now=100.01)

    def test_loaded_path_still_requires_real_cache_and_retains_numeric_reserve(self):
        ctx, q = loaded_joint_context()
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        ordinary, _ = visual_joint_context(context(), q)
        previous = jp.plan_joint_path(ordinary, q, now=100.01)
        for key in ("target_raw", "frames", "joint_envelope", "remaining_hold_budget", "budget", "tracking_policy"):
            self.assertEqual(plan[key], previous[key])
        ctx["cached_target"] = None
        self.assert_code("known_cached_target_required", jp.plan_joint_path, ctx, q, now=100.01)

    def test_loaded_settling_retains_hard_feedback_and_current_rgb_without_proving_contact(self):
        ctx, q = loaded_joint_context()
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        observed = sample(100.02)
        observed["arms"]["right"]["joints_rad"][4] += .015
        result = jp.validate_joint_path_sample(plan, observed, now=100.02, phase="settling")
        self.assertTrue(result["loaded_observation_only"])
        self.assertFalse(result["contact_force_verified"])
        self.assertFalse(result["object_progress_verified"])
        self.assertFalse(result["tracking"]["cumulative_time_checked"])
        self.assert_code("joint_tracking_envelope", jp.validate_joint_path_sample,
                         plan, observed, now=100.02, phase="active")
        observed["arms"]["right"]["pose_m_rad"][0] += .0201
        self.assert_code("controller_relative_pose_envelope", jp.validate_joint_path_sample,
                         plan, observed, now=100.02, phase="settling")
        self.assert_code("visual_rgb_expired", jp.validate_joint_path_sample,
                         plan, sample(130.01), now=130.01, phase="settling")


class CoarseJointPathTests(unittest.TestCase):
    setUp = JointPathTests.setUp
    assert_code = JointPathTests.assert_code

    @staticmethod
    def rehash(plan):
        plan["plan_sha256"] = jp.evidence_sha256({k:v for k,v in plan.items() if k != "plan_sha256"})

    def test_three_to_ten_cm_requested_and_encoded_bounds(self):
        # Solve only synthetic test targets from the pinned robot FK. No IK,
        # object location or trajectory is introduced into the runtime.
        base = context()
        model, _, _ = jp._load_model(base["model_catalog"], base["urdf_source"])
        start = target(base, delta=0.)
        origin = fk_matrix(model["mdh"], start)
        for distance, accepted in ((.0299, False), (.0301, True),
                                   (.0999, True), (.1001, False)):
            low, high = 0., math.radians(20)
            for _ in range(50):
                amount = (low + high) / 2
                wanted = start[:]
                wanted[1] -= .02
                wanted[2] -= amount
                actual = matrix_error(origin, fk_matrix(model["mdh"], wanted))["position_error_m"]
                if actual < distance:
                    low = amount
                else:
                    high = amount
            ctx, wanted = coarse_joint_context(base, wanted)
            with self.subTest(distance=distance):
                if not accepted:
                    self.assert_code("model_endpoint_displacement", jp.plan_joint_path,
                                     ctx, wanted, now=100.01)
                else:
                    plan = jp.plan_joint_path(ctx, wanted, now=100.01)
                    self.assertGreaterEqual(plan["model_endpoint_displacement_m"], .03)
                    self.assertLessEqual(plan["model_endpoint_displacement_m"], .10)
                    self.assertEqual(plan["profile_limits"]["speed_percent"], 1)
                    measured = sample(100.02)
                    measured["arms"]["right"]["joints_rad"] = plan["encoded_target_joints_rad"][:]
                    self.assertTrue(jp.validate_joint_path_sample(plan, measured, now=100.02)["within_joint_path_envelope"])

    def test_quantization_cannot_round_a_target_across_either_translation_boundary(self):
        base = context()
        model, _, _ = jp._load_model(base["model_catalog"], base["urdf_source"])
        start = target(base, delta=0.)
        origin = fk_matrix(model["mdh"], start)

        def distance(q):
            return matrix_error(origin, fk_matrix(model["mdh"], q))["position_error_m"]

        for boundary, inward in ((.030, 1), (.100, -1)):
            for requested_inside in (True, False):
                found = False
                # Vary the other joint to find both rounding directions on the
                # real SDK grid; no mocked encoder or enlarged epsilon.
                for offset in range(10, 31):
                    desired = boundary + inward * (1e-9 if requested_inside else -1e-9)
                    low, high = 0., math.radians(20)
                    for _ in range(50):
                        amount = (low + high) / 2
                        wanted = start[:]
                        wanted[1] -= offset / 1000
                        wanted[2] -= amount
                        if distance(wanted) < desired:
                            low = amount
                        else:
                            high = amount
                    _, encoded = jp.encode_joint_target(wanted)
                    inside = .030 <= distance(encoded) <= .100
                    if inside == requested_inside:
                        continue
                    found = True
                    ctx, wanted = coarse_joint_context(base, wanted)
                    with self.subTest(boundary=boundary, requested_inside=requested_inside):
                        self.assertEqual(.030 <= distance(wanted) <= .100, requested_inside)
                        self.assert_code("model_endpoint_displacement", jp.plan_joint_path,
                                         ctx, wanted, now=100.01)
                    break
                self.assertTrue(found, (boundary, requested_inside))

    def test_six_centimetre_ten_degree_both_arms_with_exact_four_frames_and_no_hold(self):
        for side in ("left", "right"):
            ctx, q = coarse_joint_context(context(arm=side))
            before = copy.deepcopy(ctx)
            plan = jp.plan_joint_path(ctx, q, now=100.01)
            self.assertEqual(ctx, before)
            self.assertGreaterEqual(plan["model_endpoint_displacement_m"], .030)
            self.assertLessEqual(plan["model_endpoint_displacement_m"], .100)
            self.assertAlmostEqual(plan["model_endpoint_rotation_rad"], math.radians(10))
            self.assertEqual(plan["motion_profile"], "coarse_approach")
            self.assertEqual(plan["profile_limits"], jp.COARSE_PROFILE_LIMITS)
            self.assertEqual(plan["frames"], joint_hold_frames(plan["target_raw"]))
            self.assertEqual(plan["hold_policy"], "latch_only")
            self.assertFalse(plan["hold_supported"])
            self.assertEqual(plan["remaining_hold_budget"]["required_translation_m"], 0.)
            self.assertEqual(plan["remaining_hold_budget"]["required_rotation_rad"], 0.)
            for key in ("motion_permitted", "dispatch_authorized", "metric_clearance_checked",
                        "absolute_workspace_checked", "contact_force_verified"):
                self.assertFalse(plan[key])
            self.assertEqual(plan["hardware_commands_sent"], 0)
            self.assertIsNone(plan["physical_stop_verified"])
            for mixed in plan["mixed_targets"]:
                for v, lo, hi in zip(mixed["joints_rad"], plan["joint_envelope"]["low_rad"], plan["joint_envelope"]["high_rad"]):
                    self.assertLessEqual(lo, v)
                    self.assertLessEqual(v, hi)

    def test_twenty_degrees_limit_and_original_profile_are_distinct(self):
        base = context()
        ctx, q = coarse_joint_context(base, target(base, 0, math.radians(20)))
        plan = jp.plan_joint_path(ctx, q, now=100.01)
        self.assertGreater(plan["encoded_target_joints_rad"][0]-base["origin"]["arms"]["right"]["joints_rad"][0], .05)
        ctx, q = coarse_joint_context(base, target(base, 0, math.radians(20)+.0001))
        self.assert_code("joint_step_limit", jp.plan_joint_path, ctx, q, now=100.01)
        regular, q = visual_joint_context(base, target(base, 0, math.radians(10)))
        self.assert_code("joint_step_limit", jp.plan_joint_path, regular, q, now=100.01)
        regular, q = visual_joint_context()
        old = jp.plan_joint_path(regular, q, now=100.01)
        self.assertEqual(old["budget"], {"max_translation_m":.020,"max_rotation_rad":.05})
        self.assertGreater(old["remaining_hold_budget"]["required_translation_m"], 0.)
        self.assertAlmostEqual(old["remaining_hold_budget"]["required_rotation_rad"], .018)

    def test_exact_positive_negative_20000_millidegrees_and_off_grid_overrun(self):
        for initial, direction in ((10000,1),(20000,1),(120000,-1),(-10000,-1),(-120000,1)):
            with self.subTest(initial=initial,direction=direction):
                raw=RAW[:];raw[0]=initial
                base=context(raw)
                goal=target(base,delta=0.)
                goal[0]=math.radians((initial+direction*20000)/1000)
                ctx,q=coarse_joint_context(base,goal)
                plan=jp.plan_joint_path(ctx,q,now=100.01)
                self.assertEqual(plan["target_raw"][0]-initial,direction*20000)
                for amount in (20001.,20000.0001):
                    goal[0]=math.radians((initial+direction*amount)/1000)
                    ctx,q=coarse_joint_context(base,goal)
                    self.assert_code("joint_step_limit",jp.plan_joint_path,ctx,q,now=100.01)

    def test_saved_ordinary_v1_plan_without_new_profile_fields_still_validates(self):
        ctx,q=visual_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        for key in ("motion_profile","profile_limits","planning_joint_feedback_rad"):
            plan.pop(key)
        self.rehash(plan)
        self.assertTrue(jp.validate_joint_path_sample(plan,sample(100.02),now=100.02)["within_joint_path_envelope"])
        ctx,q=coarse_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        plan.pop("motion_profile")
        self.rehash(plan)
        self.assert_code("motion_profile_mismatch",jp.validate_joint_path_sample,plan,sample(100.02),now=100.02)

    def test_far_evidence_operation_hash_and_target_are_bound(self):
        for change, code in (
            (lambda e:e.pop("far_from_target_observation"), "visual_evidence_schema"),
            (lambda e:e.update(far_from_target_observation="  "), "visual_description_required"),
            (lambda e:e.update(operation="align"), "visual_joint_operation"),
            (lambda e:e.update(operation="release_retreat"), "visual_joint_operation"),
            (lambda e:e.update(operation="extract_segment"), "visual_joint_operation"),
            (lambda e:e["identity"].update(owner="other"), "visual_identity_mismatch"),
            (lambda e:e["target_raw"].__setitem__(2,123), "visual_joint_target_changed")):
            ctx,q=coarse_joint_context()
            change(ctx["geometry"]["evidence"])
            ctx["geometry"]["source"]["sha256"]=jp.evidence_sha256(ctx["geometry"]["evidence"])
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=coarse_joint_context()
        ctx["geometry"]["evidence"]["far_from_target_observation"]="Changed without hash"
        self.assert_code("visual_evidence_hash_mismatch",jp.plan_joint_path,ctx,q,now=100.01)

    def test_unknown_tags_and_loaded_or_recovery_cannot_fall_back(self):
        for tag in ("piper_rgb_supervised_coarse_approach_v1", "piper_rgb_supervised_coarse_approach_v3", "unknown", None):
            ctx,q=coarse_joint_context()
            ctx["geometry"]["schema"]=tag
            self.assert_code("geometry_schema",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=coarse_joint_context()
        ctx["loaded_context"]={"pretend":"retained"}
        self.assert_code("loaded_context_requires_loaded_schema",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=coarse_joint_context()
        self.assert_code("visual_joint_recovery_unsupported",jp.plan_joint_path,ctx,q,now=100.01,recovery_mode=jp.RECOVERY_MODE)

    def test_both_arm_origin_and_current_must_be_nominal_even_with_initialization_source(self):
        for side in ("left","right"):
            for label in ("origin","current"):
                ctx,q=coarse_joint_context()
                ctx["initialization_sources"]={"not_used_to_grant_an_exception":True}
                ctx[label]["arms"][side]["joints_rad"][1]=-.0001
                ctx["origin_sha256"]=jp.evidence_sha256(ctx["origin"])
                self.assert_code("origin_joint_limit" if label=="origin" else "coarse_current_joint_limit",
                                 jp.plan_joint_path,ctx,q,now=100.01)

    def test_fixed_budget_and_rehashed_plan_cannot_enlarge_profile(self):
        for budget in ({"max_translation_m":.036,"max_rotation_rad":.08},
                       {"max_translation_m":.035,"max_rotation_rad":.081},
                       {"max_translation_m":.020,"max_rotation_rad":.05}):
            ctx,q=coarse_joint_context()
            ctx["budget"]=budget
            self.assert_code("coarse_fixed_budget_required",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=coarse_joint_context()
        original=jp.plan_joint_path(ctx,q,now=100.01)
        for change,code in (
            (lambda p:p["budget"].update(max_translation_m=.1),"coarse_fixed_budget_required"),
            (lambda p:p["budget"].update(max_rotation_rad=.2),"coarse_fixed_budget_required"),
            (lambda p:p["profile_limits"].update(max_joint_change_rad=.1),"profile_limits_changed"),
            (lambda p:p["profile_limits"].update(speed_percent=True),"profile_limits_changed"),
            (lambda p:p["tracking_policy"].update(max_origin_excursion_rad=.1),"tracking_policy_changed"),
            (lambda p:p["tracking_policy"].update(max_cumulative_outside_band_s=2.),"tracking_policy_changed"),
            (lambda p:p["joint_envelope"]["high_rad"].__setitem__(0,1.),"coarse_envelope_changed"),
            (lambda p:p.update(motion_profile="ordinary"),"motion_profile_mismatch"),
            (lambda p:p["geometry"].update(schema="unknown"),"geometry_schema")):
            altered=copy.deepcopy(original)
            change(altered)
            self.rehash(altered)
            self.assert_code(code,jp.validate_joint_path_sample,altered,sample(100.02),now=100.02)

    def test_coarse_keeps_true_cache_and_manufacturer_margin_requirements(self):
        for change,code in (
            (lambda c:c.update(cached_target=None),"known_cached_target_required"),
            (lambda c:c["cached_target"]["frame_receipts"].pop(),"cached_target_partial_or_unknown"),
            (lambda c:c["cached_target"].update(identity={**c["identity"],"connection_id":"other"}),"cached_target_identity_changed"),
            (lambda c:c["controller_limits"]["right"][2].__setitem__(0,-.35),"target_joint_limit")):
            ctx,q=coarse_joint_context()
            change(ctx)
            self.assert_code(code,jp.plan_joint_path,ctx,q,now=100.01)

    def test_rehashed_underreported_box_cannot_hide_multiple_axis_rotation(self):
        ctx,q=coarse_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        origin=plan["origin"]["arms"]["right"]["joints_rad"]
        requested=origin[:]
        for i in (0,4,5):requested[i]+=.14
        raw,encoded=jp.encode_joint_target(requested)
        plan.update(requested_target_joints_rad=requested,encoded_target_joints_rad=encoded,
                    target_raw=raw,frames=joint_hold_frames(raw))
        plan["geometry"]["evidence"]["target_raw"]=raw
        plan["geometry"]["source"]["sha256"]=jp.evidence_sha256(plan["geometry"]["evidence"])
        plan["joint_envelope"]={"low_rad":[min(v)-.003 for v in zip(origin,requested,encoded)],
                                "high_rad":[max(v)+.003 for v in zip(origin,requested,encoded)]}
        cap=math.radians(20)+.003
        plan["postsend_joint_envelope"]={
            "low_rad":[max(min(a,b)-.025,a-cap) for a,b in zip(origin,encoded)],
            "high_rad":[min(max(a,b)+.025,a+cap) for a,b in zip(origin,encoded)]}
        # Retain the old plan's small reported box, but recompute the outer hash.
        self.rehash(plan)
        self.assert_code("coarse_independent_box_budget",jp.validate_joint_path_sample,
                         plan,sample(100.02),now=100.02)

    def test_hundred_mm_endpoint_from_current_as_well_as_origin(self):
        base=context()
        # Find the endpoint boundary using official FK; the current sample
        # moves a legal .002 rad away, making the SAME target >100 mm from it.
        model,_,_=jp._load_model(base["model_catalog"],base["urdf_source"])
        start=target(base,delta=0.)
        a=fk_matrix(model["mdh"],start)
        lo,hi=0.,math.radians(20)
        for _ in range(45):
            mid=(lo+hi)/2
            wanted=start[:];wanted[2]-=mid;wanted[1]-=.02
            if matrix_error(a,fk_matrix(model["mdh"],wanted))["position_error_m"]<.0998:lo=mid
            else:hi=mid
        wanted=start[:];wanted[2]-=lo;wanted[1]-=.02
        ctx,q=coarse_joint_context(base,wanted)
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        self.assertLess(plan["model_endpoint_displacement_m"],.100)
        ctx["current"]["arms"]["right"]["joints_rad"][2]+=.002
        self.assert_code("model_endpoint_displacement",jp.plan_joint_path,ctx,q,now=100.01)
        ctx,q=coarse_joint_context(base,wanted)
        q[2]-=.002
        ctx["geometry"]["evidence"]["target_raw"]=jp.encode_joint_target(q)[0]
        ctx["geometry"]["source"]["sha256"]=jp.evidence_sha256(ctx["geometry"]["evidence"])
        self.assert_code("model_endpoint_displacement",jp.plan_joint_path,ctx,q,now=100.01)

    def test_postsend_only_band_and_same_original_stability_tolerance(self):
        ctx,q=coarse_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        policy=plan["tracking_policy"]
        self.assertEqual(policy,{"mode":"bounded_postsend_settling","transient_band_rad":.025,
            "max_cumulative_outside_band_s":1.,"max_origin_excursion_rad":math.radians(20)+.003,
            "settle_tolerance_rad":.003})
        observed=sample(100.02)
        observed["arms"]["right"]["joints_rad"][5]+=.015743
        self.assert_code("joint_tracking_envelope",jp.validate_joint_path_sample,plan,observed,now=100.02,phase="active")
        result=jp.validate_joint_path_sample(plan,observed,now=100.02,phase="settling")
        self.assertFalse(result["tracking"]["within_nominal_band"])
        self.assertFalse(result["tracking"]["cumulative_time_checked"])
        self.assertFalse(result["motion_permitted"])
        self.assertFalse(plan["postsend_entire_box_admitted"])

    def test_process_feedback_peer_jaw_mode_and_deadline_remain_hard(self):
        ctx,q=coarse_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        for change,code in (
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(0,s["arms"]["right"]["pose_m_rad"][0]+.1201),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["right"]["pose_m_rad"].__setitem__(5,s["arms"]["right"]["pose_m_rad"][5]+.4001),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(0,.2),"stationary_joint_anchor"),
            (lambda s:s["arms"]["right"]["gripper"].update(width_m=.03),"jaw_anchor_drift"),
            (lambda s:s["arms"]["right"]["arm_status"].update(mode_feedback=2),"same_move_j_mode_required")):
            observed=sample(100.02);change(observed)
            self.assert_code(code,jp.validate_joint_path_sample,plan,observed,now=100.02,phase="settling")
        self.assert_code("stale_sample",jp.validate_joint_path_sample,plan,sample(100.02),now=100.08,phase="settling")
        self.assert_code("visual_rgb_expired",jp.validate_joint_path_sample,plan,sample(130.01),now=130.01,phase="settling")

    def test_geometry_reuse_never_caches_feedback_or_hides_changed_model(self):
        from robot_tools.joint_model_bounds import flange_box_bounds
        jp._fixed_flange_box.cache_clear()
        ctx,q=coarse_joint_context()
        plan=jp.plan_joint_path(ctx,q,now=100.01)
        misses=jp._fixed_flange_box.cache_info().misses
        with patch('robot_tools.joint_model_bounds.flange_box_bounds',wraps=flange_box_bounds) as bound:
            self.assertTrue(jp.validate_joint_path_sample(plan,sample(100.02),now=100.02)['within_joint_path_envelope'])
            self.assertEqual(bound.call_count,0)
            self.assert_code('stale_sample',jp.validate_joint_path_sample,plan,sample(100.02),now=100.08)
            changed=copy.deepcopy(plan);changed['model']['mdh'][1][2]+=10.
            self.rehash(changed)
            with self.assertRaises(jp.JointPathError):
                jp.validate_joint_path_sample(changed,sample(100.02),now=100.02)
            self.assertEqual(bound.call_count,1)
        self.assertEqual(jp._fixed_flange_box.cache_info().misses,misses+1)


if __name__ == "__main__":
    unittest.main()
