"""Pure first-target planner tests; synthetic geometry is never scene evidence."""
import copy
from enum import IntEnum
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from robot_tools import joint_initialization as ji
from robot_tools import joint_path as jp
from robot_tools.hold_transaction import joint_hold_frames
from test_joint_path import context as ordinary_context, SOURCE


ROOT = Path(__file__).resolve().parents[3]
SAVED = ROOT / "artifacts/plug_transfer_diagnostic_20261007T103243_8ff158/resume_after_pair_host_update_1791345474396806512/final_read_state.json"


def context(arm="right", historical=False):
    ctx=ordinary_context(arm=arm)
    ctx["schema"]=ji.SCHEMA
    ctx.pop("cached_target")
    ctx.pop("budget")
    for label in ("origin","current"):
        if historical:
            ctx[label]["arms"]=json.loads(SAVED.read_text())["state"]["arms"]
        for state in ctx[label]["arms"].values():
            state["arm_status"].update(ctrl_mode=1,mode_feedback=0,motion_status=0)
            state["gripper"]["foc_status"]["driver_enable_status"]=False
            stamp=ctx[label]["captured_at"]
            state["fragment_timestamps_s"]={k:stamp for k in state["fragment_timestamps_s"]}
    ctx["origin_sha256"]=jp.evidence_sha256(ctx["origin"])
    ctx["geometry"]["available_clearance_m"]=.5  # Synthetic, ample fixture only.
    ctx["unloaded_evidence"]={"origin_sample_id":ctx["origin"]["sample_id"],"source":SOURCE}
    return ctx


def fresh_sample(ctx,now=100.02):
    sample=copy.deepcopy(ctx["current"])
    sample.update(sample_id="sample-"+str(now),captured_at=now)
    for state in sample["arms"].values():
        state["fragment_timestamps_s"]={k:now for k in state["fragment_timestamps_s"]}
    return sample


def commit_origin(ctx):
    ctx["origin_sha256"]=jp.evidence_sha256(ctx["origin"])


def visual_context(arm="right", historical=False):
    """Synthetic image references only; never a real scene or dispatch permit."""
    ctx=context(arm,historical)
    evidence={"identity":copy.deepcopy(ctx["identity"]),"observation_id":"synthetic-scene",
        "capture_id":"synthetic-capture","rgb_received_at":99.9,
        "saved_rgb_evidence":{view:{"rgb_path":"/synthetic/"+view+".png",
            "artifact_sha256":str(index+1)*64,"frame_number":index,"host_received_at":99.9}
            for index,view in enumerate(("front","left_hand","right_hand"))},
        "unloaded_observation":"Synthetic test: both jaws empty and no object contact.",
        "corridor_observation":"Synthetic test: local startup corridor visually clear.",
        "workspace_clearance_statement":"Synthetic user statement only, not a metric survey."}
    ctx["geometry"]={"schema":ji.VISUAL_GEOMETRY_SCHEMA,
        "origin_sample_id":ctx["origin"]["sample_id"],
        "source":{"ref":"pair_event:test:visual_initialization","sha256":jp.evidence_sha256(evidence)},
        "evidence":evidence}
    return ctx


def rehash_visual(ctx):
    ctx["geometry"]["source"]["sha256"]=jp.evidence_sha256(ctx["geometry"]["evidence"])


class JointInitializationTests(unittest.TestCase):
    def setUp(self):
        self.no_socket=patch("socket.socket",side_effect=AssertionError("No hardware/network socket"))
        self.no_socket.start()
        self.addCleanup(self.no_socket.stop)

    def assert_code(self,code,fn,*args,**kwargs):
        with self.assertRaises(ji.JointInitializationError) as caught:
            fn(*args,**kwargs)
        self.assertEqual(caught.exception.code,code)

    def test_legal_p_mode_zero_quantization_seed_requires_no_prior_cache(self):
        for side in ("left","right"):
            ctx=context(side)
            before=copy.deepcopy(ctx)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            self.assertEqual(plan["purpose"],"seed_current")
            self.assertEqual(plan["requested_target_joints_rad"],ctx["origin"]["arms"][side]["joints_rad"])
            self.assertEqual(plan["target_raw"],jp.encode_joint_target(plan["requested_target_joints_rad"])[0])
            self.assertEqual(plan["frames"],joint_hold_frames(plan["target_raw"]))
            self.assertEqual(plan["cached_target_prior"],"unknown")
            self.assertNotIn("budget",plan)
            self.assertNotIn("cached_target",plan)
            self.assertFalse(plan["motion_permitted"])
            self.assertTrue(plan["mode_frame_can_activate_cached_target"])
            self.assertFalse(plan["atomic_update_proven"])
            self.assertEqual(ctx,before)

    def test_historical_both_p_boundary_starts_make_candidates_with_synthetic_clearance(self):
        expected={"left":(.010441422919277505,.046111598837690115),
                  "right":(.012142023678753522,.010297442586766578)}
        for side,(position,rotation) in expected.items():
            ctx=context(side,historical=True)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            self.assertEqual(plan["purpose"],"startup_j2_j3")
            self.assertEqual(plan["encoded_target_joints_rad"][1:3],[0.,0.])
            self.assertAlmostEqual(plan["model_endpoint_displacement_m"],position)
            self.assertAlmostEqual(plan["model_endpoint_rotation_rad"],rotation)
            self.assertGreater(plan["relative_sweep_bound_m"],.05)
            self.assertFalse(plan["hard_path_guarantee"])
            self.assertTrue(all(len(v)==2 for v in plan["initial_boundary_violations"].values()))

    def test_successive_selected_arms_keep_peer_boundary_and_modes(self):
        left=context("left",historical=True)
        first=ji.plan_joint_initialization(left,now=100.01)
        right=context("right",historical=True)
        for label in ("origin","current"):
            right[label]["arms"]["left"]["joints_rad"]=first["encoded_target_joints_rad"][:]
            right[label]["arms"]["left"]["arm_status"]["mode_feedback"]=1
        commit_origin(right)
        second=ji.plan_joint_initialization(right,now=100.01)
        self.assertEqual(second["purpose"],"startup_j2_j3")
        sample=fresh_sample(right)
        ji.validate_joint_initialization_sample(second,sample,now=100.02,phase="pre_dispatch")
        sample["arms"]["left"]["arm_status"]["mode_feedback"]=0
        self.assert_code("movement_mode_changed",ji.validate_joint_initialization_sample,second,sample,now=100.02)

    def test_mode_transition_p_or_l_to_j_then_no_regression(self):
        for mode in (0,1,2):
            ctx=context()
            for label in ("origin","current"):
                ctx[label]["arms"]["right"]["arm_status"]["mode_feedback"]=mode
            commit_origin(ctx)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            sample=fresh_sample(ctx)
            ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase="active")
            sample["arms"]["right"]["arm_status"]["mode_feedback"]=1
            observed=ji.validate_joint_initialization_sample(plan,sample,now=100.02,mode_confirmed=True)
            self.assertTrue(observed["selected_target_observed"])
            self.assertFalse(observed["postsend_and_stability_verified"])
            sample["arms"]["right"]["arm_status"]["mode_feedback"]=0
            self.assert_code("movement_mode_changed",ji.validate_joint_initialization_sample,plan,sample,now=100.02,mode_confirmed=True)

    def test_selected_mixed_boundary_progress_does_not_inherit_ordinary_rotation_gate(self):
        ctx=context("left",historical=True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        sample["arms"]["left"]["joints_rad"][1]=0.
        result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase="active")
        self.assertGreater(result["arms"]["left"]["model_rotation_rad"],.05)
        self.assertFalse(result["selected_target_observed"])
        self.assertFalse(result["motion_permitted"])

    def test_raw_pose_remains_independent_and_relative_20mm_still_applies(self):
        ctx=context("right",historical=True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertEqual(plan["controller_flange_pose"]["right"],ctx["origin"]["arms"]["right"]["pose_m_rad"])
        sample=fresh_sample(ctx)
        sample["arms"]["right"]["joints_rad"]=plan["encoded_target_joints_rad"][:]
        sample["arms"]["right"]["arm_status"]["mode_feedback"]=1
        result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,mode_confirmed=True)
        self.assertTrue(result["selected_target_observed"])
        self.assertEqual(result["arms"]["right"]["controller_displacement_m"],0.)
        sample["arms"]["right"]["pose_m_rad"][0]+=.0201
        self.assert_code("controller_relative_pose_envelope",ji.validate_joint_initialization_sample,plan,sample,now=100.02,mode_confirmed=True)

    def test_target_never_reanchors_to_later_current_observation(self):
        ctx=context()
        first=ji.plan_joint_initialization(ctx,now=100.01)
        ctx["current"]["arms"]["right"]["joints_rad"][5]+=.0001
        second=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertEqual(first["target_raw"],second["target_raw"])
        self.assertEqual(first["requested_target_joints_rad"],second["requested_target_joints_rad"])
        self.assertEqual(first["plan_sha256"],second["plan_sha256"])

    def test_fixed_origin_envelope_is_not_the_three_second_stability_window(self):
        ctx=context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        for phase in ("pre_dispatch","active"):
            for side in ("left","right"):
                sample=fresh_sample(ctx)
                sample["arms"][side]["pose_m_rad"][0]+=.0015
                sample["arms"][side]["gripper"]["width_m"]+=.0015
                result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase=phase)
                self.assertTrue(result["within_initialization_envelope"])
                self.assertFalse(result["postsend_and_stability_verified"])
                sample["arms"][side]["gripper"]["width_m"]+=.0006
                self.assert_code("jaw_anchor_drift",ji.validate_joint_initialization_sample,
                                 plan,sample,now=100.02,phase=phase)
                if phase=="pre_dispatch" or side=="left":
                    sample=fresh_sample(ctx)
                    sample["arms"][side]["pose_m_rad"][0]+=.0021
                    self.assert_code("controller_relative_pose_envelope",ji.validate_joint_initialization_sample,
                                     plan,sample,now=100.02,phase=phase)

    def test_feedback_tolerance_does_not_become_strict_nominal(self):
        ctx=context("left",historical=True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        sample["arms"]["left"]["joints_rad"]=plan["encoded_target_joints_rad"][:]
        sample["arms"]["left"]["joints_rad"][2]=.001
        sample["arms"]["left"]["arm_status"]["mode_feedback"]=1
        result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,mode_confirmed=True)
        self.assertTrue(result["within_feedback_tolerance"])
        self.assertFalse(result["strict_nominal"]["left"])
        self.assertTrue(result["selected_target_observed"])
        self.assertIsNone(result["physical_stop_verified"])

    def test_initial_other_axis_wrong_direction_and_large_boundary_refuse(self):
        for axis,value in ((0,3.),(1,-.10001),(2,.10001),(1,3.2),(2,-3.1),(5,2.2)):
            ctx=context()
            for label in ("origin","current"):
                ctx[label]["arms"]["right"]["joints_rad"][axis]=value
            commit_origin(ctx)
            self.assert_code("origin_joint_limit",ji.plan_joint_initialization,ctx,now=100.01)

    def test_clearance_sources_workspace_and_unloaded_are_required(self):
        for mutation,code in ((lambda c:c.update(unloaded_evidence=None),"explicit_unloaded_evidence_required"),
                (lambda c:c["unloaded_evidence"].update(origin_sample_id="old"),"explicit_unloaded_evidence_required"),
                (lambda c:c["geometry"].update(source=None),"missing_evidence_source"),
                (lambda c:c["geometry"].update(available_clearance_m=.01),"relative_sweep_exceeds_clearance"),
                (lambda c:c["geometry"]["attachment_radius_m"].update(left=0.),"invalid_attachment_radius"),
                (lambda c:c["geometry"].update(workspace_max_m=[0.,0.,0.]),"model_workspace_envelope"),
                (lambda c:c.update(cached_target={}),"context_schema")):
            ctx=context(historical=True)
            mutation(ctx)
            self.assert_code(code,ji.plan_joint_initialization,ctx,now=100.01)

    def test_freshness_health_enable_and_enum_types(self):
        self.assert_code("stale_sample",ji.plan_joint_initialization,context(),now=100.1)
        cases=((lambda s:s["arm_status"].update(ctrl_mode=2),"unhealthy_feedback"),
               (lambda s:s["arm_status"].update(mode_feedback=True),"unknown_movement_mode"),
               (lambda s:s["drivers"]["2"]["foc_status"].update(driver_enable_status=False),"joint_disabled"),
               (lambda s:s["gripper"]["foc_status"].update(driver_enable_status=None),"jaw_enable_unknown"),
               (lambda s:s["gripper"].update(force_N=float("nan")),"unhealthy_feedback"),
               (lambda s:s["fragment_timestamps_s"].update(joint_12=99.95),"stale_feedback"))
        for mutate,code in cases:
            ctx=context()
            mutate(ctx["current"]["arms"]["left"])
            self.assert_code(code,ji.plan_joint_initialization,ctx,now=100.01)
        class Mode(IntEnum):
            P=0
        ctx=context()
        for label in ("origin","current"):
            ctx[label]["arms"]["right"]["arm_status"]["mode_feedback"]=Mode.P
        commit_origin(ctx)
        self.assertTrue(ji.plan_joint_initialization(ctx,now=100.01)["candidate_valid"])

    def test_plan_origin_and_model_source_tampering_refuse(self):
        ctx=context()
        ctx["origin"]["arms"]["left"]["joints_rad"][0]+=.001
        self.assert_code("original_anchor_changed",ji.plan_joint_initialization,ctx,now=100.01)
        ctx=context()
        ctx["model_catalog"]["sha256"]="0"*64
        self.assert_code("model_source_invalid",ji.plan_joint_initialization,ctx,now=100.01)
        ctx=context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        plan["encoded_target_joints_rad"][0]+=.001
        self.assert_code("plan_changed",ji.validate_joint_initialization_sample,plan,fresh_sample(ctx),now=100.02)

    def test_peer_drift_jaw_change_and_boundary_deepening_are_frozen(self):
        ctx=context("left",historical=True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        for mutate,code in ((lambda s:s["arms"]["right"]["joints_rad"].__setitem__(0,1.),"stationary_joint_anchor"),
                (lambda s:s["arms"]["left"]["gripper"].update(width_m=s["arms"]["left"]["gripper"]["width_m"]+.0021),"jaw_anchor_drift"),
                (lambda s:s["arms"]["right"]["gripper"]["foc_status"].update(driver_enable_status=True),"jaw_enable_changed"),
                (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(1,-.095),"feedback_joint_limit")):
            sample=fresh_sample(ctx)
            mutate(sample)
            self.assert_code(code,ji.validate_joint_initialization_sample,plan,sample,now=100.02)

    def test_controller_limit_intersection_cannot_widen_sdk_or_urdf(self):
        ctx=context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertLess(plan["effective_joint_limits_rad"]["right"][5][1],2.1)
        self.assertFalse(plan["j6_source_disagreement"]["resolved_by_widening"])
        ctx["controller_limits"]["right"][0]=[-.1,.1]
        self.assert_code("origin_joint_limit",ji.plan_joint_initialization,ctx,now=100.01)


class VisualJointInitializationTests(unittest.TestCase):
    setUp=JointInitializationTests.setUp
    assert_code=JointInitializationTests.assert_code

    def test_explicit_visual_legal_seed_preserves_metric_branch_target_and_nonmetric_limits(self):
        for side in ("left","right"):
            ctx=visual_context(side)
            before=copy.deepcopy(ctx)
            visual=ji.plan_joint_initialization(ctx,now=100.01)
            metric=ji.plan_joint_initialization(context(side),now=100.01)
            for key in ("purpose","target_raw","frames","joint_envelope","effective_joint_limits_rad",
                        "model_target_flange_transform","startup_limits"):
                self.assertEqual(visual[key],metric[key])
            self.assertEqual(ctx,before)
            self.assertEqual(visual["spatial_admission_mode"],"rgb_supervised")
            for key in ("metric_clearance_checked","metric_collision_checked","absolute_workspace_checked",
                        "hard_path_guarantee","atomic_update_proven","motion_permitted","dispatch_authorized"):
                self.assertFalse(visual[key])
            for key in ("sweep_axis_radii_m","active_sweep_bound_m","passive_sweep_bound_m",
                        "relative_sweep_bound_m","clearance_budget_m"):
                self.assertIsNone(visual[key])
            for key in ("attachment_radius_m","available_clearance_m","workspace_min_m","workspace_max_m"):
                self.assertNotIn(key,visual["geometry"])
            self.assertTrue(metric["metric_clearance_checked"])
            self.assertTrue(metric["absolute_workspace_checked"])
            self.assertFalse(metric["metric_collision_checked"])

    def test_historical_p_j2_j3_disabled_jaws_have_identical_frozen_boundary_targets(self):
        for side in ("left","right"):
            ctx=visual_context(side,True)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            metric=ji.plan_joint_initialization(context(side,True),now=100.01)
            self.assertEqual(plan["purpose"],"startup_j2_j3")
            self.assertEqual(plan["target_raw"],metric["target_raw"])
            self.assertEqual(plan["encoded_target_joints_rad"][1:3],[0.,0.])
            self.assertLessEqual(plan["model_endpoint_displacement_m"],.015)
            sample=fresh_sample(ctx)
            sample["arms"][side]["joints_rad"]=plan["encoded_target_joints_rad"][:]
            sample["arms"][side]["arm_status"]["mode_feedback"]=1
            result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,mode_confirmed=True)
            self.assertTrue(result["selected_target_observed"])
            self.assertFalse(result["absolute_workspace_checked"])
            self.assertFalse(result["metric_clearance_checked"])
            self.assertFalse(result["qualification_granted"])
            self.assertIsNone(result["physical_stop_verified"])

    def test_missing_or_malformed_metric_does_not_fallback_to_visual(self):
        for geometry in (None,{}, {"schema":"unknown_rgb_mode"}):
            ctx=context()
            ctx["geometry"]=geometry
            self.assert_code("geometry_schema",ji.plan_joint_initialization,ctx,now=100.01)
        ctx=visual_context()
        ctx["geometry"]["available_clearance_m"]=1.
        self.assert_code("visual_geometry_schema",ji.plan_joint_initialization,ctx,now=100.01)
        ctx=context()
        ctx["geometry"]["available_clearance_m"]=.0001
        self.assert_code("clearance_reserve_missing",ji.plan_joint_initialization,ctx,now=100.01)

    def test_visual_identity_original_anchor_and_evidence_hash_cannot_change(self):
        for mutation,code in (
                (lambda c:c["geometry"]["evidence"]["identity"].update(owner="different"),"visual_identity_mismatch"),
                (lambda c:c["geometry"].update(origin_sample_id="later-origin"),"geometry_anchor_mismatch"),
                (lambda c:c["geometry"]["source"].update(sha256="0"*64),"visual_evidence_hash_mismatch"),
                (lambda c:c["geometry"]["evidence"].update(corridor_observation="Changed without source hash"),"visual_evidence_hash_mismatch")):
            ctx=visual_context()
            mutation(ctx)
            self.assert_code(code,ji.plan_joint_initialization,ctx,now=100.01)
        ctx=visual_context()
        ctx["current"]["arms"]["right"]["joints_rad"][5]+=.0001
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertEqual(plan["requested_target_joints_rad"],ctx["origin"]["arms"]["right"]["joints_rad"])

    def test_visual_scene_requires_complete_exact_frames_finite_current_times_and_descriptions(self):
        changes=(
            (lambda e:e["saved_rgb_evidence"].pop("left_hand"),"visual_rgb_views_required"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(frame_number=True),"visual_rgb_frame_number"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(artifact_sha256="bad"),"invalid_sha256"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(host_received_at=100.02),"visual_rgb_expired"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(host_received_at=69.),"visual_rgb_expired"),
            (lambda e:e["saved_rgb_evidence"]["front"].update(host_received_at=99.7),"visual_rgb_scene_mismatch"),
            (lambda e:e.update(rgb_received_at=99.8),"visual_rgb_scene_mismatch"),
            (lambda e:e.update(workspace_clearance_statement=" "),"visual_description_required"),
            (lambda e:e.update(unloaded_observation=""),"visual_description_required"),
            (lambda e:e.update(corridor_observation=""),"visual_description_required"))
        for change,code in changes:
            ctx=visual_context()
            change(ctx["geometry"]["evidence"])
            rehash_visual(ctx)
            self.assert_code(code,ji.plan_joint_initialization,ctx,now=100.01)
        ctx=visual_context()
        ctx["geometry"]["evidence"]["saved_rgb_evidence"]["front"]["host_received_at"]=float("nan")
        self.assert_code("invalid_number",ji.plan_joint_initialization,ctx,now=100.01)

    def test_scene_expires_during_monitor_and_new_samples_do_not_renew_it(self):
        ctx=visual_context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        self.assertAlmostEqual(plan["visual_rgb_deadline"],129.9)
        sample=fresh_sample(ctx,129.89)
        ji.validate_joint_initialization_sample(plan,sample,now=129.89)
        sample=fresh_sample(ctx,129.91)
        self.assert_code("visual_rgb_expired",ji.validate_joint_initialization_sample,plan,sample,now=129.91)
        altered=copy.deepcopy(plan)
        altered["geometry"]["evidence"]["rgb_received_at"]=129.91
        self.assert_code("plan_changed",ji.validate_joint_initialization_sample,altered,sample,now=129.91)

    def test_visual_keeps_endpoint_cap_and_cannot_request_free_or_excessive_joint_target(self):
        ctx=visual_context()
        for label in ("origin","current"):
            ctx[label]["arms"]["right"]["joints_rad"]=[0.,-.1,.1,0.,0.,0.]
        commit_origin(ctx)
        self.assert_code("model_endpoint_displacement",ji.plan_joint_initialization,ctx,now=100.01)
        ctx=visual_context()
        ctx["target_joints_rad"]=[0.]*6
        self.assert_code("context_schema",ji.plan_joint_initialization,ctx,now=100.01)
        for axis,value in ((1,-.10001),(2,.10001),(0,3.)):
            ctx=visual_context()
            for label in ("origin","current"):
                ctx[label]["arms"]["right"]["joints_rad"][axis]=value
            commit_origin(ctx)
            self.assert_code("origin_joint_limit",ji.plan_joint_initialization,ctx,now=100.01)

    def test_visual_preserves_peer_jaw_mode_and_relative_raw_motion_guards(self):
        ctx=visual_context("left",True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        changes=(
            (lambda s:s["arms"]["right"]["joints_rad"].__setitem__(0,1.),"stationary_joint_anchor"),
            (lambda s:s["arms"]["left"]["gripper"].update(width_m=s["arms"]["left"]["gripper"]["width_m"]+.0021),"jaw_anchor_drift"),
            (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(1,-.095),"feedback_joint_limit"),
            (lambda s:s["arms"]["right"]["arm_status"].update(mode_feedback=1),"movement_mode_changed"),
            (lambda s:s["arms"]["left"]["pose_m_rad"].__setitem__(0,s["arms"]["left"]["pose_m_rad"][0]+.0201),"controller_relative_pose_envelope"),
            (lambda s:s["arms"]["right"]["drivers"]["1"]["foc_status"].update(driver_enable_status=False),"joint_disabled"))
        for change,code in changes:
            sample=fresh_sample(ctx)
            change(sample)
            self.assert_code(code,ji.validate_joint_initialization_sample,plan,sample,now=100.02)
        self.assert_code("stale_sample",ji.validate_joint_initialization_sample,plan,fresh_sample(ctx),now=100.08)


class SettlingJointInitializationTests(unittest.TestCase):
    setUp=JointInitializationTests.setUp
    assert_code=JointInitializationTests.assert_code

    def test_policy_freezes_distinct_sending_and_postsend_bounds(self):
        for visual in (False,True):
            ctx=visual_context() if visual else context()
            for label in ("origin","current"):
                ctx[label]["arms"]["right"]["joints_rad"][4]+=.000005
            commit_origin(ctx)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            self.assertEqual(plan["tracking_policy"],{
                "mode":"bounded_postsend_settling" if visual else "strict",
                "transient_band_rad":.025 if visual else .003,
                "max_cumulative_outside_band_s":1. if visual else 0.,
                "max_origin_excursion_rad":.1,"settle_tolerance_rad":.003})
            endpoint=plan["encoded_target_joints_rad"] if visual else plan["requested_target_joints_rad"]
            for i,(q,t) in enumerate(zip(ctx["origin"]["arms"]["right"]["joints_rad"],endpoint)):
                band=plan["tracking_policy"]["transient_band_rad"]
                self.assertEqual(plan["postsend_joint_envelope"]["low_rad"][i],min(q,t)-band)
                self.assertEqual(plan["postsend_joint_envelope"]["high_rad"][i],max(q,t)+band)
            self.assertEqual(plan["flange_tracking_sweep_bound_scope"],
                             "strict_band_only_excludes_postsend_settling")
            if visual:
                self.assertGreater(plan["postsend_flange_tracking_sweep_bound_m"]["right"],
                                   plan["flange_tracking_sweep_bound_m"]["right"])
                self.assertEqual(plan["postsend_flange_tracking_sweep_bound_m"]["left"],
                                 plan["flange_tracking_sweep_bound_m"]["left"])
            else:
                self.assertEqual(plan["postsend_joint_envelope"],plan["joint_envelope"])
                self.assertEqual(plan["postsend_flange_tracking_sweep_bound_m"],
                                 plan["flange_tracking_sweep_bound_m"])
            altered=copy.deepcopy(plan)
            altered["tracking_policy"]["max_cumulative_outside_band_s"]=2.
            self.assert_code("plan_changed",ji.validate_joint_initialization_sample,
                             altered,fresh_sample(ctx),now=100.02)

    def test_j5_transient_is_classified_without_arrival_or_time_authority(self):
        ctx=visual_context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        state=sample["arms"]["right"]
        state["arm_status"]["mode_feedback"]=1
        state["joints_rad"][4]+=.015743
        before=copy.deepcopy((plan,sample))
        result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,
                                                       phase="settling",mode_confirmed=True)
        self.assertEqual((plan,sample),before)
        tracking=result["tracking"]
        self.assertFalse(tracking["within_nominal_band"])
        self.assertEqual(len(tracking["outside_nominal_band"]),1)
        row=tracking["outside_nominal_band"][0]
        self.assertEqual(row["joint_index"],5)
        self.assertEqual(row["observed_rad"],state["joints_rad"][4])
        self.assertEqual(row["low_rad"],plan["joint_envelope"]["low_rad"][4])
        self.assertAlmostEqual(row["excess_rad"],.015743-.003)
        self.assertEqual(tracking["max_excess_rad"],row["excess_rad"])
        for field in ("selected_target_observed","postsend_and_stability_verified",
                      "motion_permitted","qualification_granted"):
            self.assertFalse(result[field])
        self.assertFalse(tracking["cumulative_time_checked"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assert_code("joint_tracking_envelope",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="active")
        self.assert_code("stationary_joint_anchor",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="pre_dispatch")

    def test_returning_to_original_tolerance_is_only_a_single_sample_arrival(self):
        ctx=visual_context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        sample["arms"]["right"]["arm_status"]["mode_feedback"]=1
        sample["arms"]["right"]["joints_rad"][4]+=.002
        result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,
                                                       phase="settling",mode_confirmed=True)
        self.assertTrue(result["selected_target_observed"])
        self.assertEqual(result["tracking"],{"within_nominal_band":True,
            "outside_nominal_band":[],"max_excess_rad":0.,"cumulative_time_checked":False})
        self.assertFalse(result["postsend_and_stability_verified"])
        self.assertFalse(result["motion_permitted"])
        sample["arms"]["right"]["arm_status"]["mode_feedback"]=0
        self.assert_code("movement_mode_changed",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="settling",mode_confirmed=True)

    def test_metric_cannot_select_settling_or_tolerate_transient(self):
        ctx=context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        self.assert_code("settling_requires_rgb_supervision",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="settling")
        sample["arms"]["right"]["joints_rad"][4]+=.015743
        self.assert_code("joint_tracking_envelope",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="active")

    def test_total_transient_halfwidth_is_025_not_028(self):
        ctx=visual_context()
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        for field,sign in (("low_rad",-1),("high_rad",1)):
            sample=fresh_sample(ctx)
            # J6 rotates the flange without moving its origin, isolating the
            # joint-band rule from the independent 20 mm translation guard.
            sample["arms"]["right"]["joints_rad"][5]=plan["postsend_joint_envelope"][field][5]
            result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase="settling")
            self.assertEqual(result["tracking"]["outside_nominal_band"][0]["joint_index"],6)
            sample["arms"]["right"]["joints_rad"][5]+=sign*1e-9
            self.assert_code("joint_tracking_envelope",ji.validate_joint_initialization_sample,
                             plan,sample,now=100.02,phase="settling")

    def test_boundary_progress_uses_path_interval_and_frozen_origin_cap(self):
        for side in ("left","right"):
            ctx=visual_context(side,True)
            plan=ji.plan_joint_initialization(ctx,now=100.01)
            sample=fresh_sample(ctx)
            # Its initial recovery error may exceed .025: that is commanded
            # progress inside the original interval, not a tracking excursion.
            result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase="settling")
            self.assertTrue(result["tracking"]["within_nominal_band"])
            sample["arms"][side]["joints_rad"]=plan["encoded_target_joints_rad"][:]
            sample["arms"][side]["joints_rad"][4]+=.015743
            result=ji.validate_joint_initialization_sample(plan,sample,now=100.02,phase="settling")
            self.assertFalse(result["tracking"]["within_nominal_band"])
        ctx=visual_context("left",True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        sample["arms"]["left"]["joints_rad"]=plan["encoded_target_joints_rad"][:]
        sample["arms"]["left"]["joints_rad"][1]=.012
        self.assert_code("recovery_excursion_limit",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="settling")

    def test_settling_preserves_peer_jaw_hard_boundary_and_freshness(self):
        ctx=visual_context("left",True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        changes=(
            (lambda s:s["arms"]["right"]["joints_rad"].__setitem__(0,1.),"stationary_joint_anchor"),
            (lambda s:s["arms"]["left"]["gripper"].update(width_m=s["arms"]["left"]["gripper"]["width_m"]+.0021),"jaw_anchor_drift"),
            (lambda s:s["arms"]["left"]["joints_rad"].__setitem__(1,-.095),"feedback_joint_limit"),
            (lambda s:s["arms"]["right"]["arm_status"].update(mode_feedback=1),"movement_mode_changed"),
            (lambda s:s["arms"]["right"]["drivers"]["1"]["foc_status"].update(driver_enable_status=False),"joint_disabled"),
            (lambda s:s["arms"]["left"]["gripper"]["foc_status"].update(driver_enable_status=True),"jaw_enable_changed"))
        for change,code in changes:
            sample=fresh_sample(ctx)
            change(sample)
            self.assert_code(code,ji.validate_joint_initialization_sample,
                             plan,sample,now=100.02,phase="settling")
        self.assert_code("stale_sample",ji.validate_joint_initialization_sample,
                         plan,fresh_sample(ctx),now=100.08,phase="settling")
        self.assert_code("visual_rgb_expired",ji.validate_joint_initialization_sample,
                         plan,fresh_sample(ctx,129.91),now=129.91,phase="settling")

    def test_both_raw_and_model_20mm_limits_still_intersect_transient_band(self):
        ctx=visual_context("right",True)
        plan=ji.plan_joint_initialization(ctx,now=100.01)
        sample=fresh_sample(ctx)
        sample["arms"]["right"]["pose_m_rad"][0]+=.0201
        self.assert_code("controller_relative_pose_envelope",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="settling")
        sample=fresh_sample(ctx)
        sample["arms"]["right"]["joints_rad"]=[q+s*.0249 for q,s in
            zip(plan["encoded_target_joints_rad"],(-1,-1,-1,-1,1,-1))]
        self.assert_code("model_relative_pose_envelope",ji.validate_joint_initialization_sample,
                         plan,sample,now=100.02,phase="settling")


if __name__=="__main__":
    unittest.main()
