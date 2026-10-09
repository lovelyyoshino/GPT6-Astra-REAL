"""Offline same-connection initialization; fake feedback is no physical proof."""
import copy
import math
import struct
import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import arms, pair_device, pair_preparation, pair_initialization as adapter
from robot_tools import joint_initialization as planner, joint_path
from robot_tools import takeover, linear_hold, supervised_actions, single_supervised_actions
from robot_tools.tracking_observation import TrackingObservation
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_joint_path import context as source_context, RAW, SOURCE
from test_single_supervised_actions import SingleActionFixture


LEFT_RAW = [-16433, -5258, 2616, -4435, 19821, -1809]
RIGHT_RAW = [10000, -1669, 2259, 10000, 10000, 10000]


def initialization_context(device, arm="left", *, event_id="init-1"):
    ctx = source_context(arm=arm)
    ctx["schema"] = planner.SCHEMA
    del ctx["cached_target"], ctx["budget"]
    ctx["identity"].update({k:v for k,v in device.joint_binding(arm).items() if k != "cached_target"})
    ctx["identity"]["worker_id"] = event_id
    ctx["origin"] = device.observe_initialization(ctx["identity"])
    ctx["current"] = copy.deepcopy(ctx["origin"])
    ctx["origin_sha256"] = joint_path.evidence_sha256(ctx["origin"])
    ctx["geometry"].update(attachment_radius_m={"left": .3, "right": .3}, available_clearance_m=.4,
                           origin_sample_id=ctx["origin"]["sample_id"])
    ctx["unloaded_evidence"] = {"origin_sample_id": ctx["origin"]["sample_id"], "source": SOURCE}
    return ctx


class InitializationFixture(SingleActionFixture):
    def setUp(self):
        super().setUp()
        for module in (pair_device, pair_preparation, adapter):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.guard_hook = self.journal_hook = self.bus_hook = None
        self.residual = [0.]*6
        for side, robot in self.robots.items():
            self.profile["arms"][side]["model"] = "piper_x"
            self.joints[side] = [math.radians(v/1000) for v in (LEFT_RAW if side == "left" else RIGHT_RAW)]
            robot.ctrl_mode, robot.motion.mode = 1, 0
            robot.driver_enabled, robot.gripper_enabled = [True]*6, False
            raw_target = [round(math.degrees(v)*1000) for v in self.joints[side]]
            original = robot._bus_send
            def bus(frame, *, _side=side, _original=original, _target=raw_target):
                _original(frame)
                if frame.arbitration_id == 0x159 and self.robots[_side].accept:
                    self.robots[_side].gripper_enabled = True
                if 0x155 <= frame.arbitration_id <= 0x157:
                    i = 2*(frame.arbitration_id-0x155)
                    _target[i:i+2] = struct.unpack(">ii", bytes(frame.data))
                    if frame.arbitration_id == 0x157 and self.robots[_side].accept:
                        self.joints[_side] = [math.radians(v/1000)+r for v,r in zip(_target,self.residual)]
                self.clock.sleep(.0001)
                if self.bus_hook:
                    self.bus_hook(_side, frame)
            robot.comm.send_bus.send = bus
            def move_j(target, *, _robot=robot):
                if _robot.auto_mode:
                    _robot.set_motion_mode("j")
                raw = [round(v*(180/math.pi)*1000) for v in target]
                for i in range(2 if _robot.partial else 3):
                    frame = _robot.can.Message(arbitration_id=0x155+i, is_extended_id=False,
                        data=struct.pack(">ii", *raw[2*i:2*i+2]))
                    _robot._send_msg(_robot.frame_transform(frame))
            robot.move_j = move_j
        self.device = pair_device.GuardedPairDevice(self.profile, self.journal, self.guard)
        self.addCleanup(self.device.close)

    def guard(self):
        if self.guard_hook:
            self.guard_hook()

    def journal(self, event, data):
        self.events.append((event, copy.deepcopy(data)))
        if self.journal_hook:
            self.journal_hook(event, data)

    def opened_context(self, arm="left"):
        self.device.connect_for_preparation()
        return initialization_context(self.device, arm)

    def execute(self, ctx, *, event="init-1", arm=None):
        return self.device.initialize_joint_target(arm or ctx["identity"]["arm"], context=ctx,
            event_id=event, deadline_at=self.clock.time()+60)

    def ids(self, side="left"):
        return [frame.arbitration_id for frame in self.robots[side].sent]


class PairInitializationTests(InitializationFixture):
    def test_slow_feedback_journal_preserves_three_second_rx_window_and_single_dispatch(self):
        ctx=self.opened_context()
        def delayed(event,data):
            if event=='feedback' and self.device._action.auxiliary_executor is not None:
                self.clock.sleep(.08)
        self.journal_hook=delayed
        result=self.execute(ctx)
        self.assertTrue(result['ok'],result.get('errors'))
        self.assertEqual(self.ids(),[0x151,0x155,0x156,0x157])
        self.assertEqual(self.ids('right'),[])
        self.assertGreaterEqual(result['baseline_duration_s'],3.)
        self.assertGreaterEqual(result['observed_stable_duration_s'],3.)
        self.assertAlmostEqual(result['initialization_observation_timing']['maximum_s']['journal_s'],.08)
        self.assertGreaterEqual(result['observed_feedback_advances'],20)

    def test_slow_validation_still_fails_without_retry_or_timestamp_renewal(self):
        ctx=self.opened_context();original=adapter.validate_joint_initialization_sample
        def delayed(*args,**kwargs):
            result=original(*args,**kwargs);self.clock.sleep(.051);return result
        with patch.object(adapter,'validate_joint_initialization_sample',side_effect=delayed):
            result=self.execute(ctx)
        self.assertFalse(result['ok']);self.assertEqual(self.ids(),[])
        self.assertIn('50 ms',result['errors'][0]['detail'])
        failure=result['tracking_observation']['first_failure']
        self.assertIsNotNone(failure['sample'])
        self.assertEqual(len([event for event,data in self.events if event=='initialization_feedback_rejected']),1)
        self.assertFalse(result['cache_established'])

    def test_peer_drift_during_log_is_observed_on_next_sample_without_sending(self):
        ctx=self.opened_context();changed=[]
        def drift(event,data):
            if event=='feedback' and self.device._action.auxiliary_executor is not None and not changed:
                changed.append(True);self.clock.sleep(.08);self.joints['right'][0]+=.02
        self.journal_hook=drift
        result=self.execute(ctx)
        self.assertFalse(result['ok']);self.assertEqual(self.ids(),[]);self.assertEqual(self.ids('right'),[])
        self.assertFalse(result['cache_established'])

    def test_cancellation_during_feedback_log_is_not_hidden_by_reordering(self):
        ctx=self.opened_context();cancelled=[]
        def journal(event,data):
            if event=='feedback' and self.device._action.auxiliary_executor is not None:
                cancelled.append(True);self.clock.sleep(.08)
        def guard():
            if cancelled:raise RuntimeError('cancelled during journal')
        self.journal_hook=journal;self.guard_hook=guard
        result=self.execute(ctx)
        self.assertFalse(result['ok']);self.assertEqual(self.ids(),[])
        self.assertIn('cancelled during journal',result['errors'][0]['detail'])

    def test_p_startup_disabled_jaws_complete_once_preserves_peer_and_jaws(self):
        ctx = self.opened_context()
        original = copy.deepcopy(ctx)
        anchor = copy.deepcopy(self.device._preparation.anchor)
        flags = copy.deepcopy(self.device._preparation.flags)
        result = self.execute(ctx)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["status"], "joint_target_initialized")
        self.assertEqual(result["initialization_plan"]["purpose"], "startup_j2_j3")
        self.assertEqual(self.ids(), [0x151,0x155,0x156,0x157])
        self.assertEqual(self.ids("right"), [])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["gripper_commands_sent"], 0)
        self.assertIsNone(result["accepted"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["unknown_cache_activation_bounded"])
        self.assertGreaterEqual(result["baseline_duration_s"], 3.)
        self.assertGreaterEqual(result["observed_stable_duration_s"], 3.)
        self.assertGreaterEqual(result["observed_feedback_advances"], 20)
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertEqual(self.device._preparation.anchor["right"], anchor["right"])
        self.assertEqual(self.device._preparation.flags, flags)
        for side in takeover.SIDES:
            self.assertEqual(self.device._preparation.anchor[side]["gripper"], anchor[side]["gripper"])
        self.assertEqual(self.device._action.boundary_origins["left"], anchor["left"]["joints_rad"])
        self.assertEqual(self.device._preparation.expected_modes, {"left":1,"right":0})
        self.assertEqual(ctx, original)
        self.assertEqual(result["cached_target"], self.device.joint_binding("left")["cached_target"])
        self.device.observe()

    def test_legal_nonzero_j_seed_and_l_to_j_seed_are_valid(self):
        self.joints = {side:[math.radians(v/1000) for v in RAW] for side in takeover.SIDES}
        self.robots["left"].motion.mode = 1
        self.robots["right"].motion.mode = 2
        ctx = self.opened_context()
        first = self.execute(ctx)
        self.assertTrue(first["ok"], first.get("errors"))
        self.assertEqual(first["initialization_plan"]["purpose"], "seed_current")
        ctx = initialization_context(self.device, "right", event_id="init-right")
        second = self.execute(ctx, event="init-right")
        self.assertTrue(second["ok"], second.get("errors"))
        self.assertEqual(self.ids("right"), [0x151,0x155,0x156,0x157])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)

    def test_ready_connection_uses_same_initialization_path(self):
        self.joints = {side:[math.radians(v/1000) for v in RAW] for side in takeover.SIDES}
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.open()
        result = self.execute(initialization_context(self.device))
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertTrue(result["sample"]["task_ready"])
        self.device.observe()

    def test_two_boundary_initializations_then_two_jaws_and_promote_reuse_connection(self):
        left = self.execute(self.opened_context())
        self.assertTrue(left["ok"], left.get("errors"))
        right = self.execute(initialization_context(self.device, "right", event_id="init-right"), event="init-right")
        self.assertTrue(right["ok"], right.get("errors"))
        cache = copy.deepcopy(self.device._joint_cache)
        self.assertTrue(self.device.prepare_gripper("left")["ok"])
        self.assertTrue(self.device.prepare_gripper("right")["ok"])
        initialized_anchor = copy.deepcopy(self.device._preparation.anchor)
        self.assertTrue(self.device.promote_ready()["task_ready"])
        self.assertEqual(self.device._joint_cache, cache)
        self.assertEqual(self.device._action.idle_anchor, initialized_anchor)
        self.assertEqual(self.device._preparation.expected_modes, {"left":1, "right":1})
        for side in takeover.SIDES:
            self.assertEqual(self.ids(side), [0x151,0x155,0x156,0x157,0x159])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.device.observe()

    def test_ready_peer_keeps_stricter_persistent_half_mm_anchor(self):
        self.joints = {side:[math.radians(v/1000) for v in RAW] for side in takeover.SIDES}
        for robot in self.robots.values():
            robot.gripper_enabled = True
        self.device.open()
        ctx = initialization_context(self.device)
        def change(robot, state):
            if robot.side == "right" and self.ids():
                state["pose_m_rad"][0] += .0007
        self.hook = change
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151])
        self.assertIn("persistent arm anchor", result["errors"][0]["detail"])

    def test_preparation_peer_retains_existing_two_mm_anchor_policy(self):
        ctx = self.opened_context()
        before = copy.deepcopy(self.device._preparation.anchor["right"])
        def change(robot, state):
            if robot.side == "right" and self.ids():
                state["pose_m_rad"][0] += .0015
        self.hook = change
        result = self.execute(ctx)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.device._preparation.anchor["right"], before)
        self.device.observe()

    def test_existing_cache_returns_original_source_without_context_or_resend(self):
        first = self.execute(self.opened_context())
        self.assertTrue(first["ok"], first.get("errors"))
        cache = self.device.joint_binding("left")["cached_target"]
        source = copy.deepcopy(self.device._joint_initializations["left"])
        result = self.execute(None, arm="left", event="different-event")
        self.assertTrue(result["ok"])
        self.assertEqual(result["status"], "already_initialized")
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(result["cached_target"], cache)
        self.assertEqual(result["initialization_source"], source)
        self.assertEqual(len(self.ids()), 4)

    def test_existing_cache_does_not_hide_new_drift(self):
        self.assertTrue(self.execute(self.opened_context())["ok"])
        self.joints["right"][0] += .0031
        with self.assertRaises(RuntimeError):
            self.execute(None, arm="left")
        self.assertEqual(len(self.ids()), 4)
        self.assertIsNotNone(self.device._fault)
        self.assertIsNone(self.device._joint_cache["left"])

    def test_origin_must_be_device_issued_and_cache_cannot_be_invented(self):
        ctx = self.opened_context()
        ctx["origin"]["arms"]["left"]["joints_rad"][0] += .0001
        ctx["origin_sha256"] = joint_path.evidence_sha256(ctx["origin"])
        with self.assertRaisesRegex(RuntimeError, "not issued"):
            self.execute(ctx)
        with self.assertRaisesRegex(ValueError, "context required"):
            self.execute(None, arm="left")
        self.assertEqual(self.ids(), [])

    def test_device_issued_different_worker_cannot_initialize_this_event(self):
        ctx = self.opened_context()
        with self.assertRaisesRegex(ValueError, "worker identity"):
            self.execute(ctx, event="another-worker-event")
        self.assertEqual(self.ids(), [])
        self.assertIsNone(self.device._joint_cache["left"])

    def test_partial_sequence_never_establishes_cache_or_retries(self):
        ctx = self.opened_context()
        self.robots["left"].partial = True
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151,0x155,0x156])
        self.assertIsNone(self.device.joint_binding("left")["cached_target"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx)
        self.assertEqual(len(self.ids()), 3)
        self.assertEqual(result["stop_commands_sent"], 0)

    def test_bus_exception_preserves_actual_attempts_and_no_peer_send(self):
        ctx = self.opened_context()
        self.robots["left"].fail_id = 0x156
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 3)
        self.assertEqual(self.ids("right"), [])
        self.assertIsNone(self.device._joint_initializations["left"])

    def test_no_postsend_joint_response_times_out_without_cache(self):
        ctx = self.opened_context()
        self.robots["left"].accept = False
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4)
        self.assertIsNone(self.device._joint_cache["left"])
        self.assertIn("arrival", result["errors"][0]["detail"])

    def test_micro_boundary_residual_is_reported_without_rezero(self):
        ctx = self.opened_context()
        self.residual[1] = -.001
        self.residual[2] = .001
        result = self.execute(ctx)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertFalse(result["strict_nominal"])
        self.assertTrue(result["within_feedback_tolerance"])
        self.assertEqual(result["cached_target"]["target_raw"][1:3], [0,0])
        self.assertAlmostEqual(result["after"]["left"]["joints_rad"][1], -.001)
        self.assertTrue(self.execute(None, arm="left")["ok"])
        self.assertEqual(len(self.ids()), 4)

    def test_residual_above_band_fails_without_cache(self):
        ctx = self.opened_context()
        self.residual[1] = -.0031
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertIsNone(self.device._joint_cache["left"])
        self.assertEqual(len(self.ids()), 4)

    def test_slow_guard_checks_selected_before_first_mode_frame(self):
        ctx = self.opened_context()
        def guard():
            if self.device._action.ticket is not None:
                self.clock.sleep(.2)
                self.joints["left"][1] += .01
        self.guard_hook = guard
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [])

    def test_stale_or_alarm_between_frames_stops_remaining_tx(self):
        ctx = self.opened_context()
        def hook(robot, state):
            if len(self.ids()) >= 2:
                state["fragment_timestamps_s"]["joint_12"] -= .06
        self.hook = hook
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151,0x155])
        self.assertIsNone(self.device._joint_cache["left"])

    def test_unsolicited_jaw_enable_between_frames_fails(self):
        ctx = self.opened_context()
        self.bus_hook = lambda side, frame: setattr(self.robots["right"], "gripper_enabled", True)
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151])

    def test_confirmed_new_j_mode_cannot_regress_during_arrival_window(self):
        ctx = self.opened_context()
        def change(robot, state):
            sent_at = self.device._action.sent_at
            if robot.side == "left" and sent_at is not None and self.clock.time()-sent_at > .04:
                state["arm_status"]["mode_feedback"] = 0
        self.hook = change
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(result["errors"][0]["code"], "movement_mode_changed")
        self.assertEqual(len(self.ids()), 4)
        self.assertIsNone(self.device._joint_cache["left"])

    def test_clock_rollback_after_first_return_is_fault_without_retry(self):
        ctx = self.opened_context()
        self.bus_hook = lambda side, frame: self.clock.sleep(-.01)
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIsNone(self.device._joint_cache["left"])

    def test_completion_journal_delay_and_drift_do_not_publish_cache(self):
        ctx = self.opened_context()
        def journal(event, data):
            if event == "pair_joint_initialization_observed":
                self.clock.sleep(.2)
                self.joints["right"][0] += .0031
        self.journal_hook = journal
        result = self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4)
        self.assertIsNone(self.device._joint_cache["left"])
        self.assertIsNone(self.device._joint_initializations["left"])

    def test_boundary_and_preparation_peer_anchor_never_expand_after_success(self):
        ctx = self.opened_context()
        before = copy.deepcopy(self.device._preparation.anchor["right"])
        self.assertTrue(self.execute(ctx)["ok"])
        self.assertEqual(self.device._preparation.anchor["right"], before)
        self.joints["right"][1] -= .0031
        with self.assertRaises(RuntimeError):
            self.device.observe()

    def test_later_origin_cannot_enlarge_original_boundary_reference(self):
        self.device.connect_for_preparation()
        original = copy.deepcopy(self.device._preparation.anchor)
        self.joints["right"][1] -= .0005
        result = self.execute(initialization_context(self.device))
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.device._action.boundary_origins["right"], original["right"]["joints_rad"])


class BoundedSettlingTests(InitializationFixture):
    def visual_context(self, arm="right"):
        ctx = self.opened_context(arm)
        stamp = self.clock.time()
        evidence = {"identity": copy.deepcopy(ctx["identity"]), "observation_id": "fixture-scene",
            "capture_id": "fixture-capture", "rgb_received_at": stamp,
            "saved_rgb_evidence": {view: {"rgb_path": "/fixture/"+view+".png",
                "artifact_sha256": str(i+1)*64, "frame_number": 1, "host_received_at": stamp}
                for i,view in enumerate(("front", "left_hand", "right_hand"))},
            "unloaded_observation": "Synthetic fixture: both jaws empty and no object contact",
            "corridor_observation": "Synthetic fixture: visually clear startup corridor",
            "workspace_clearance_statement": "Synthetic user statement, not measured geometry"}
        ctx["geometry"] = {"schema": planner.VISUAL_GEOMETRY_SCHEMA,
            "origin_sample_id": ctx["origin"]["sample_id"], "evidence": evidence,
            "source": {"ref": "fixture-rgb", "sha256": joint_path.evidence_sha256(evidence)}}
        return ctx

    def response(self, function):
        def hook(robot, state):
            sent = self.device._action.sent_at
            if robot.side == "right" and sent is not None:
                function(self.clock.time()-sent, state)
        self.hook = hook

    def assert_one_dispatch(self, result, *, ok):
        self.assertEqual(result["ok"], ok, result.get("errors"))
        self.assertEqual(self.ids("right"), [0x151,0x155,0x156,0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["gripper_commands_sent"], 0)
        self.assertIsNone(result["physical_stop_verified"])
        self.assertEqual(result["cache_established"], ok)

    def test_short_postsend_deviation_settles_without_any_additional_target(self):
        ctx = self.visual_context()
        self.response(lambda dt,s: s["joints_rad"].__setitem__(4,s["joints_rad"][4]+(.015 if dt<.6 else 0.)))
        result = self.execute(ctx)
        self.assert_one_dispatch(result, ok=True)
        tracking = result["tracking_observation"]
        self.assertEqual(tracking["mode"], "bounded_postsend_settling")
        self.assertGreaterEqual(tracking["cumulative_outside_nominal_band_s"], .59)
        self.assertLess(tracking["cumulative_outside_nominal_band_s"], .7)
        self.assertAlmostEqual(tracking["max_excess_rad"], .012)
        self.assertEqual(tracking["first_outside_nominal_band"]["tracking"]["outside_nominal_band"][0]["joint_index"],5)
        self.assertIsNone(tracking["first_failure"])
        self.assertGreaterEqual(result["observed_stable_duration_s"],3.)
        self.assertGreaterEqual(result["observed_feedback_advances"],20)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(result["after"]["right"]["joints_rad"],
            result["initialization_plan"]["encoded_target_joints_rad"])),.003)

    def test_sustained_deviation_exhausts_one_event_budget_without_cache_or_retry(self):
        ctx = self.visual_context()
        self.response(lambda dt,s:s["joints_rad"].__setitem__(4,s["joints_rad"][4]+.008))
        result = self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        tracking=result["tracking_observation"]
        self.assertGreater(tracking["cumulative_outside_nominal_band_s"],1.)
        self.assertIn("cumulative",result["errors"][0]["detail"])
        self.assertEqual(tracking["first_failure"]["tracking"]["outside_nominal_band"][0]["joint_index"],5)
        self.assertIsNotNone(tracking["first_failure"]["sample"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx)
        self.assertEqual(len(self.ids("right")),4)

    def test_reentry_axis_changes_and_arrival_resets_do_not_renew_total(self):
        ctx = self.visual_context()
        def pulses(dt,state):
            if dt<.35 or 1.0<=dt<1.35:
                state["joints_rad"][4]+=.008
            elif .5<=dt<.85:
                state["joints_rad"][5]+=.008
        self.response(pulses)
        result=self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        self.assertIn("cumulative",result["errors"][0]["detail"])
        self.assertGreater(result["tracking_observation"]["cumulative_outside_nominal_band_s"],1.)

    def test_visual_sending_phase_never_uses_postsend_tolerance(self):
        ctx=self.visual_context()
        def during(robot,state):
            if robot.side=="right" and self.ids("right"):
                state["joints_rad"][4]+=.0031
        self.hook=during
        result=self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids("right"),[0x151])
        self.assertEqual(result["errors"][0]["code"],"joint_tracking_envelope")
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"],0.)
        self.assertIsNone(self.device._joint_cache["right"])

    def test_partial_visual_sequence_never_enters_settling(self):
        ctx=self.visual_context()
        self.robots["right"].partial=True
        result=self.execute(ctx)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids("right"),[0x151,0x155,0x156])
        self.assertIn("incomplete",result["errors"][0]["detail"])
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"],0.)
        self.assertIsNone(self.device._joint_cache["right"])

    def test_metric_initialization_retains_immediate_strict_band(self):
        ctx=self.opened_context("right")
        self.response(lambda dt,s:s["joints_rad"].__setitem__(4,s["joints_rad"][4]+.004))
        result=self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        self.assertEqual(result["errors"][0]["code"],"joint_tracking_envelope")
        self.assertEqual(result["tracking_observation"]["mode"],"strict")
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"],0.)

    def test_postsend_hard_transient_cap_still_refuses_immediately(self):
        ctx=self.visual_context()
        self.response(lambda dt,s:s["joints_rad"].__setitem__(4,s["joints_rad"][4]+.0251))
        result=self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        self.assertEqual(result["errors"][0]["code"],"joint_tracking_envelope")
        self.assertAlmostEqual(result["tracking_observation"]["max_excess_rad"],.0221)

    def test_postsend_raw_displacement_and_freshness_guards_are_not_tolerated(self):
        ctx=self.visual_context()
        self.response(lambda dt,s:s["pose_m_rad"].__setitem__(0,s["pose_m_rad"][0]+.0201))
        result=self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        self.assertEqual(result["errors"][0]["code"],"controller_relative_pose_envelope")

    def test_postsend_stale_feedback_still_refuses_immediately(self):
        ctx=self.visual_context()
        self.response(lambda dt,s:s["fragment_timestamps_s"].update(joint_56=self.clock.time()-.051))
        result=self.execute(ctx)
        self.assert_one_dispatch(result,ok=False)
        self.assertEqual(result["errors"][0]["code"],"stale_feedback")


class SharedTrackingObservationTests(unittest.TestCase):
    def test_reentry_counts_both_endpoint_intervals_and_cannot_restart(self):
        tracking = TrackingObservation("test")
        tracking.start(100.)
        tracking.account({"captured_at": 100.2}, {"within_nominal_band": False}, maximum_s=1.)
        tracking.account({"captured_at": 100.4}, {"within_nominal_band": True}, maximum_s=1.)
        tracking.account({"captured_at": 100.6}, {"within_nominal_band": True}, maximum_s=1.)
        tracking.account({"captured_at": 100.8}, {"within_nominal_band": False}, maximum_s=1.)
        self.assertAlmostEqual(tracking.cumulative_s, .6)
        with self.assertRaisesRegex(RuntimeError, "already set"):
            tracking.start(100.8)
        with self.assertRaisesRegex(RuntimeError, "cumulative"):
            tracking.account({"captured_at": 101.3}, {"within_nominal_band": True}, maximum_s=1.)
        self.assertAlmostEqual(tracking.report["cumulative_outside_nominal_band_s"], 1.1)

    def test_first_failure_sample_is_copied_and_never_overwritten(self):
        tracking = TrackingObservation("test")
        sample = {"sample_id": "rejected", "captured_at": 100., "arms": {"right": {"joints_rad": [.1]*6}}}
        outside = {"within_nominal_band": False, "outside_nominal_band": [{"joint_index": 5}]}
        tracking.record_failure(RuntimeError("first"), sample, outside)
        sample["arms"]["right"]["joints_rad"][0] = 10
        outside["outside_nominal_band"].clear()
        tracking.record_failure(RuntimeError("later"), {"sample_id": "later"}, {})
        first = tracking.report["first_failure"]
        self.assertEqual(first["detail"], "first")
        self.assertEqual(first["sample"]["arms"]["right"]["joints_rad"][0], .1)
        self.assertEqual(first["tracking"]["outside_nominal_band"], [{"joint_index": 5}])

    def test_invalid_accounting_clock_does_not_reduce_existing_elapsed_time(self):
        for invalid in (False, float("nan"), 99.):
            with self.subTest(invalid=invalid):
                tracking = TrackingObservation("test")
                tracking.start(100.)
                tracking.account({"captured_at": 100.5}, {"within_nominal_band": False}, maximum_s=1.)
                with self.assertRaises(RuntimeError):
                    tracking.account({"captured_at": invalid}, {"within_nominal_band": True}, maximum_s=1.)
                self.assertEqual(tracking.cumulative_s, .5)
                self.assertEqual(tracking.previous_at, 100.5)


class RealSDKInitializationTests(unittest.TestCase):
    def test_real_default_sdk_four_frames_no_new_connection_or_jaw_command(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.drivers.core import driver_context
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, created, bindings = Clock(), [], [], {}
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper_x"
        channels = {s:cfg["channel"] for s,cfg in profile["arms"].items()}
        joints = {channels["left"]:LEFT_RAW[:], channels["right"]:RIGHT_RAW[:]}
        pending = copy.deepcopy(joints)
        modes = dict.fromkeys(channels.values(), 0)
        class FakeCAN:
            def __init__(self, channel): self.channel = channel
            def recv(self, timeout=None): time.sleep(.001); return None
            def shutdown(self): pass
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == 0x151:
                    modes[self.channel] = frame.data[1]
                if 0x155 <= frame.arbitration_id <= 0x157:
                    i = 2*(frame.arbitration_id-0x155)
                    pending[self.channel][i:i+2] = struct.unpack(">ii", bytes(frame.data))
                    if frame.arbitration_id == 0x157:
                        joints[self.channel] = pending[self.channel][:]
                clock.sleep(.0001)
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda:100.
            created.append(robot)
            bindings[id(robot)] = config["comm"]["can"]["channel"]
            return robot
        def snapshot(robot, jaw):
            channel = bindings[id(robot)]
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1, teach_status=0,
                mode_feedback=modes[channel], motion_status=0, arm_status=0, err_code=0))
            state["joints_rad"] = [math.radians(v/1000) for v in joints[channel]]
            state["gripper"]["width_m"] = .05
            state["gripper"]["foc_status"]["driver_enable_status"] = False
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("Physical socket forbidden")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms,takeover,linear_hold,supervised_actions,single_supervised_actions,
                           pair_device,pair_preparation,adapter):
                stack.enter_context(patch.object(module,"time",clock))
            stack.enter_context(patch.object(driver_context,"time",SimpleNamespace(
                monotonic=clock.monotonic,time=clock.time,sleep=time.sleep)))
            stack.enter_context(patch.object(can.interface,"Bus",side_effect=lambda **kw:FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory,"create_arm",side_effect=create))
            stack.enter_context(patch.object(arms,"snapshot",side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile,lambda *a:None)
            try:
                device.connect_for_preparation()
                self.assertEqual(sent, [])
                ctx = initialization_context(device, event_id="real-sdk-init")
                result = device.initialize_joint_target("left",context=ctx,event_id="real-sdk-init",deadline_at=clock.time()+60)
                self.assertTrue(result["ok"],result.get("errors"))
                self.assertEqual([f.arbitration_id for _,f in sent],[0x151,0x155,0x156,0x157])
                self.assertTrue(all(ch==channels["left"] for ch,_ in sent))
                self.assertEqual(len(created),2)
                self.assertTrue(all(r.get_joint_limits_enabled() is False for r in created))
                self.assertIsNone(result["physical_stop_verified"])
                device.observe()
            finally:
                device.close()
            self.assertEqual(len(sent),4)


if __name__ == "__main__":
    unittest.main()
