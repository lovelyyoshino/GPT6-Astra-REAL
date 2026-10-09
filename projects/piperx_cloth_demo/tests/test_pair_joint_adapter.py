"""Offline transport integration: sockets blocked, actual CAN replaced by fakes."""
import copy
import math
import struct
import threading
import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import pair_device, pair_joint_adapter as adapter, joint_path
from robot_tools import arms, takeover, linear_hold, supervised_actions, single_supervised_actions
from robot_tools.hold_transaction import joint_hold_frames
from robot_tools.retention_receipt import measured_anchor
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_single_supervised_actions import SingleActionFixture
from test_joint_path import context as context_fixture, visual_joint_context, RAW


class Bridge:
    def __init__(self, fixture, *, cancel=False):
        self.fixture, self.cancel = fixture, cancel
        self.original = None
        self.events = []
        self.begin_hook = self.return_hook = None
        self.receipt = None
        self.aborted = None

    def check_active(self):
        if self.aborted:
            raise RuntimeError(self.aborted)

    def record_original(self, event):
        self.original = copy.deepcopy(event)
        self.events.append("original")

    def cancellation_request(self):
        if self.cancel:
            return {"hold_event_id": "hold-1", "reason": "explicit user cancellation"}

    def claim_hold(self, event_id, hold_id, payload):
        self.fixture.clock.sleep(.001)
        original = copy.deepcopy(self.original)
        original["fault"] = {"kind": "explicit_joint_cancel", "reason": "user cancel"}
        self.events.append("claim")
        self.payload = copy.deepcopy(payload)
        return {"original_event": original, "claim": {
            "hold_event_id": hold_id, "original_event_id": event_id,
            "identity": copy.deepcopy(original["identity"]), "claimed_at": self.fixture.clock.time(),
            "original_event_sha256": joint_path.evidence_sha256(original)}, "replayed": False, "step": 2}

    def record_frame_begin(self, hold_id, index, frame):
        self.events.append(("begin", index))
        self.fixture.clock.sleep(.001)
        if self.begin_hook:
            self.begin_hook(index)

    def record_frame_return(self, hold_id, index, outcome, error=None):
        self.events.append(("return", index, outcome))
        if self.return_hook:
            self.return_hook(index)

    def finish_hold(self, hold_id, receipt):
        self.events.append("finish")
        self.receipt = copy.deepcopy(receipt)


class JointFixture(SingleActionFixture):
    def setUp(self):
        super().setUp()
        for module in (pair_device, adapter):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        for side, robot in self.robots.items():
            self.profile["arms"][side]["model"] = "piper_x"
            robot.ctrl_mode = 1
            robot.driver_enabled = [True]*6
            robot.gripper_enabled = True
            robot.motion.mode = 1
            self.joints[side] = [math.radians(value/1000) for value in RAW]
            before = robot._bus_send
            def bus(frame, *, _side=side, _before=before):
                _before(frame)
                if 0x155 <= frame.arbitration_id <= 0x157 and self.robots[_side].accept:
                    index = 2*(frame.arbitration_id-0x155)
                    values = struct.unpack(">ii", bytes(frame.data))
                    self.joints[_side][index:index+2] = [math.radians(v/1000) for v in values]
                self.clock.sleep(.0001)
            robot.comm.send_bus.send = bus
            def move_j(target, *, _robot=robot):
                if _robot.auto_mode:
                    _robot.set_motion_mode("j")
                raw = [round(v*(180/math.pi)*1000) for v in target]
                for index in range(2 if _robot.partial else 3):
                    frame = _robot.can.Message(arbitration_id=0x155+index, is_extended_id=False,
                        data=struct.pack(">ii", *raw[2*index:2*index+2]))
                    _robot._send_msg(_robot.frame_transform(frame))
            robot.move_j = move_j
        self.guard_error = None
        self.guard_hook = None
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)), self.guard)
        self.addCleanup(self.device.close)
        self.device.open()

    def guard(self):
        if self.guard_hook:
            self.guard_hook()
        if self.guard_error:
            raise RuntimeError(self.guard_error)

    def make_context(self, arm="right", *, known_cache=True):
        ctx = context_fixture(arm=arm)
        binding = self.device.joint_binding(arm)
        ctx["identity"].update({key: binding[key] for key in ("connection_id", "model", "firmware_profile")})
        origin = self.device.observe_joint(ctx["identity"])
        ctx["origin"], ctx["current"] = origin, copy.deepcopy(origin)
        ctx["origin_sha256"] = joint_path.evidence_sha256(origin)
        ctx["geometry"]["origin_sample_id"] = origin["sample_id"]
        raw, _ = joint_path.encode_joint_target(origin["arms"][arm]["joints_rad"])
        ctx["cached_target"] = {"event_id": "test-prior-complete-send", "identity": copy.deepcopy(ctx["identity"]),
            "target_raw": raw, "frame_receipts": [
                {"frame": frame, "outcome": "returned", "returned_at": origin["captured_at"]-.1+index*.001}
                for index, frame in enumerate(joint_hold_frames(raw))]}
        if known_cache:
            # Test precondition only: simulate a complete command produced by
            # the same already-live connection. There is NO public cache setter.
            self.device._joint_cache[arm] = copy.deepcopy(ctx["cached_target"])
        target = origin["arms"][arm]["joints_rad"][:]
        target[5] += .001
        return ctx, target

    def execute(self, ctx, target, bridge=None, **kwargs):
        return self.device.execute_joint(ctx["identity"]["arm"], target, context=ctx,
            event_id="joint-1", deadline_at=self.clock.time()+60, hold_bridge=bridge, **kwargs)

    def ids(self, arm="right"):
        return [f.arbitration_id for f in self.robots[arm].sent]


class JointObservationSchedulingTests(JointFixture):
    def visual(self):
        ctx, target = self.make_context()
        return visual_joint_context(ctx, target, operation="approach")

    def zero_tx_fault(self, result):
        self.assertFalse(result["ok"], result)
        self.assertEqual(self.ids(), [])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNotNone(self.device._fault)
        self.assertEqual(result["retries"], 0)

    def test_planning_yields_to_receiver_before_first_baseline_snapshot(self):
        ctx, target = self.visual()
        pending = [False]
        original_plan, original_sleep = adapter.plan_joint_path, self.clock.sleep
        def plan(*args, **kwargs):
            result = original_plan(*args, **kwargs)
            pending[0] = True
            return result
        def sleep(duration):
            original_sleep(duration)
            if duration >= adapter.BOUNDS['poll_s']:
                pending[0] = False
        def snapshot(robot, state):
            if pending[0] and robot.side == 'right':
                state['fragment_timestamps_s']['driver_state_1'] -= .061
        self.hook = snapshot
        with patch.object(adapter, 'plan_joint_path', side_effect=plan), patch.object(self.clock, 'sleep', side_effect=sleep):
            result = self.execute(ctx, target)
        self.assertTrue(result['ok'], result.get('errors'))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result['initial_rx_schedule']['rejected_samples_retried'], 0)
        self.assertFalse(result['initial_rx_schedule']['timestamps_renewed'])

    def test_cancel_during_initial_receiver_yield_is_zero_tx(self):
        ctx, target = self.visual()
        original = self.clock.sleep
        def sleep(duration):
            original(duration)
            if self.device._action.joint_executor is not None:
                self.guard_error = 'cancel during receiver scheduling'
        with patch.object(self.clock, 'sleep', side_effect=sleep):
            result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertIsNone(result['before'])
        self.assertIn('cancel during receiver scheduling', str(result['errors']))

    def test_slow_guard_and_durable_journal_precede_next_fresh_sample(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                self.clock.sleep(.03)
        self.device._action.journal = journal
        self.guard_hook = lambda: self.clock.sleep(.03)
        result = self.execute(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertGreaterEqual(result["baseline_duration_s"], 3.)
        self.assertGreaterEqual(result["observed_stable_duration_s"], 3.)
        self.assertGreaterEqual(result["baseline_feedback_advances"], 20)
        self.assertGreaterEqual(result["observed_feedback_advances"], 20)
        self.assertGreaterEqual(result["joint_observation_timing"]["maximum_s"]["guard_s"], .029)
        self.assertGreaterEqual(result["joint_observation_timing"]["maximum_s"]["journal_s"], .029)
        self.assertLessEqual(result["joint_validation_timing"]["oldest_fragment_age_at_exit_s"], .05)

    def test_stale_sample_is_rejected_once_not_replaced_with_new_feedback(self):
        ctx, target = self.visual()
        poisoned = []
        def hook(robot, state):
            if self.device._action.joint_executor is not None and robot.side == "left":
                poisoned.append(self.clock.time())
                state["fragment_timestamps_s"] = {k:v-.051 for k,v in state["fragment_timestamps_s"].items()}
        self.hook = hook
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertEqual(len(poisoned), 1)
        self.assertEqual(result["tracking_observation"]["first_failure"]["code"], "stale_feedback")
        self.assertIn("rejected_joint_feedback", result)

    def test_health_failure_before_pure_validator_keeps_the_rejected_raw_sample(self):
        ctx, target = self.visual()
        def hook(robot, state):
            if self.device._action.joint_executor is not None and robot.side == "left":
                state["drivers"]["1"]["foc_status"]["driver_enable_status"] = False
        self.hook = hook
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        failed = result["tracking_observation"]["first_failure"]
        self.assertFalse(failed["sample"]["arms"]["left"]["drivers"]["1"]["foc_status"]["driver_enable_status"])
        self.assertTrue(any(event == "joint_feedback_rejected" for event, _ in self.events))

    def test_peer_drift_during_journal_is_detected_before_any_send(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                self.joints["left"][0] += .004
                self.clock.sleep(.03)
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertEqual(result["tracking_observation"]["first_failure"]["code"], "stationary_joint_anchor")

    def test_validation_and_later_body_check_time_remain_inside_50ms(self):
        ctx, target = self.visual()
        original = self.device._action._checked_after_guard
        def slow(states):
            result = original(states)
            self.clock.sleep(.051)
            return result
        with patch.object(self.device._action, "_checked_after_guard", side_effect=slow):
            result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertIn("50 ms", str(result["errors"]))
        self.assertIn("rejected_joint_feedback", result)

    def test_cancel_during_durable_feedback_logging_blocks_first_frame(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                self.guard_error = "Cancellation during durable feedback record"
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertIn("Cancellation during durable", str(result["errors"]))

    def test_rgb_deadline_during_durable_logging_is_not_renewed(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                self.clock.sleep(31.)
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertIn("visual_rgb_expired", str(result["errors"]))

    def test_original_task_deadline_during_logging_is_not_renewed(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                self.clock.sleep(61.)
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.zero_tx_fault(result)
        self.assertIn("original task deadline", str(result["errors"]))

    def test_wallclock_jump_does_not_shorten_either_monotonic_stable_window(self):
        ctx, target = self.visual()
        original = self.device._action.journal
        base_time, offset = self.clock.time, [0.]
        self.clock.time = lambda: base_time()+offset[0]
        counts = {"baseline": 0, "post": 0}
        first_at = {}
        def journal(event, data):
            original(event, data)
            if event == "feedback":
                phase = "post" if len(self.ids()) == 4 else "baseline"
                first_at.setdefault(phase, self.clock.monotonic())
                counts[phase] += 1
                if counts[phase] == 20:
                    offset[0] += 2.
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertGreaterEqual(result["baseline_duration_s"], 3.)
        self.assertGreaterEqual(result["observed_stable_duration_s"], 3.)
        self.assertGreaterEqual(first_at["post"]-first_at["baseline"], 3.)
        self.assertGreaterEqual(self.clock.monotonic()-first_at["post"], 3.)


class RGBJointAdapterTests(JointFixture):
    def make_visual(self, arm="right", *, operation="approach"):
        ctx, target = self.make_context(arm)
        return visual_joint_context(ctx, target, operation=operation)

    def assert_latched_once(self, result, expected):
        self.assertFalse(result["ok"], result)
        self.assertEqual(self.ids(), expected)
        self.assertEqual(self.ids("left"), [])
        self.assertIsNotNone(self.device._fault)
        self.assertIsNone(result["hold_receipt"])
        self.assertFalse(result["explicit_cancel_hold_bridge_bound"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)

    def test_both_rgb_operations_create_typed_send_and_real_cache_without_metric_geometry(self):
        for arm, operation in (("right", "approach"), ("left", "align")):
            ctx, target = self.make_visual(arm, operation=operation)
            previous = copy.deepcopy(ctx)
            result = self.execute(ctx, target, operation=operation)
            self.assertTrue(result["ok"], result.get("errors"))
            self.assertEqual(self.ids(arm), [0x151, 0x155, 0x156, 0x157])
            self.assertEqual(result["passive_arm_commands_sent"], 0)
            self.assertEqual(result["hardware_commands_sent"], 4)
            self.assertEqual(result["spatial_admission_mode"], "rgb_supervised")
            self.assertEqual(result["hold_policy"], "latch_only")
            self.assertFalse(result["hold_supported"])
            self.assertIsNone(result["hold_receipt"])
            event = result["original_event"]
            self.assertEqual(event["schema"], adapter.RGB_JOINT_SEND_SCHEMA)
            self.assertEqual(event["operation"], operation)
            self.assertEqual(event["rgb_admission"], ctx["geometry"])
            self.assertEqual(event["plan_sha256"], result["joint_path_plan"]["plan_sha256"])
            self.assertEqual(event["target_raw"], ctx["geometry"]["evidence"]["target_raw"])
            self.assertNotIn("workspace_min_m", event["limits"])
            self.assertNotIn("workspace_max_m", event["limits"])
            self.assertNotIn("geometry_source", event)
            self.assertEqual(self.device.joint_binding(arm)["cached_target"],
                {key: event[key] for key in ("event_id", "identity", "target_raw", "frame_receipts")})
            self.assertIsNone(result["accepted"])
            self.assertIsNone(result["physical_stop_verified"])
            self.assertEqual(ctx, previous)
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)

    def test_rgb_hold_bridge_is_rejected_before_any_dispatch_or_cache_change(self):
        ctx, target = self.make_visual()
        bridge = Bridge(self, cancel=True)
        old_cache = self.device.joint_binding("right")["cached_target"]
        with self.assertRaisesRegex(ValueError, "cannot bind"):
            self.execute(ctx, target, bridge)
        self.assertEqual(self.ids(), [])
        self.assertEqual(bridge.events, [])
        self.assertEqual(self.device.joint_binding("right")["cached_target"], old_cache)

    def test_rgb_empty_jaw_release_retreat_uses_ordinary_bounds_and_same_transport(self):
        # The host owns the durable visual-release token. The adapter continues
        # to require no selected grasp and verifies its ordinary typed path.
        ctx, target = self.make_visual(operation="release_retreat")
        result = self.execute(ctx, target, operation="release_retreat")
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["spatial_admission_mode"], "rgb_supervised")
        self.assertFalse(result["hold_supported"])
        self.assertIsNone(result["joint_path_plan"]["loaded_context"])
        self.assertIsNone(result["physical_stop_verified"])

    def test_rgb_operation_must_match_evidence_and_cannot_cover_retreat_or_recovery(self):
        ctx, target = self.make_visual()
        for operation in ("align", "release_retreat", "recover"):
            with self.subTest(operation=operation):
                with self.assertRaisesRegex(ValueError, "must match"):
                    self.execute(ctx, target, operation=operation)
                self.assertEqual(self.ids(), [])

    def test_rgb_target_swap_is_rejected_before_tx(self):
        ctx, target = self.make_visual()
        target[5] += .0001
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertIn("visual_joint_target_changed", result["errors"][-1]["detail"])
        self.assertIsNone(result["original_event"])

    def test_rgb_evidence_hash_swap_is_rejected_before_tx(self):
        ctx, target = self.make_visual()
        ctx["geometry"]["evidence"]["corridor_observation"] = "Changed without bound source"
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertIn("visual_evidence_hash_mismatch", result["errors"][-1]["detail"])

    def test_expired_rgb_has_zero_tx_without_metric_fallback(self):
        ctx, target = self.make_visual()
        evidence = ctx["geometry"]["evidence"]
        evidence["rgb_received_at"] -= 31
        for frame in evidence["saved_rgb_evidence"].values():
            frame["host_received_at"] -= 31
        ctx["geometry"]["source"]["sha256"] = joint_path.evidence_sha256(evidence)
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertEqual(result["errors"][-1], {"type": "JointPathError", "detail": "front"})

    def test_rgb_expiry_between_frames_blocks_remainder_and_clears_cache(self):
        ctx, target = self.make_visual()
        delayed = False
        def guard():
            nonlocal delayed
            if len(self.ids()) == 1 and not delayed:
                delayed = True
                self.clock.sleep(31)
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151])
        self.assertIn("visual_rgb_expired", result["errors"][-1]["detail"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_rgb_unexpired_but_short_postsend_window_blocks_first_frame(self):
        ctx, target = self.make_visual()
        evidence = ctx["geometry"]["evidence"]
        evidence["rgb_received_at"] -= 25.
        for frame in evidence["saved_rgb_evidence"].values():
            frame["host_received_at"] -= 25.
        ctx["geometry"]["source"]["sha256"] = joint_path.evidence_sha256(evidence)
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("insufficient_rgb_postsend_window", result["errors"][-1]["detail"])
        check = result["rgb_dispatch_window"]
        self.assertEqual(check["stage"], "after_baseline_before_dispatch")
        self.assertGreater(check["remaining_rgb_window_s"], 0.)
        self.assertLess(check["remaining_rgb_window_s"], 3.)
        self.assertFalse(check["claimed_event_or_budget_released"])

    def test_real_four_point_nine_second_first_bus_window_is_refused_without_tx(self):
        # The real failed segment had 4.905 s left before its first frame:
        # the old >3 s guard sent, then RGB expired during final observation.
        self.assert_first_bus_window_refused(4.905)

    def test_exact_nine_second_first_bus_window_is_refused_without_tx(self):
        self.assert_first_bus_window_refused(9.)

    def assert_first_bus_window_refused(self, remaining):
        ctx, target = self.make_visual()
        delayed = False
        def guard():
            nonlocal delayed
            if self.device._action.ticket is not None and not delayed:
                delayed = True
                deadline = ctx["geometry"]["evidence"]["rgb_received_at"] + 30.
                self.clock.sleep(deadline-self.clock.time()-remaining)
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("insufficient_rgb_postsend_window", result["errors"][-1]["detail"])
        self.assertEqual(result["rgb_dispatch_window"]["stage"], "before_first_bus_frame")
        self.assertEqual(result["rgb_dispatch_window"]["required_postsend_window_s"], 9.)
        self.assertAlmostEqual(result["rgb_dispatch_window"]["remaining_rgb_window_s"], remaining)
        self.assertFalse(result["rgb_dispatch_window"]["claimed_event_or_budget_released"])

    def test_first_bus_window_above_minimum_still_sends_and_retains_deadline(self):
        ctx, target = self.make_visual()
        deadline = ctx["geometry"]["evidence"]["rgb_received_at"] + 30.
        delayed = False
        def guard():
            nonlocal delayed
            if self.device._action.ticket is not None and not delayed:
                delayed = True
                self.clock.sleep(deadline-self.clock.time()-9.01)
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["rgb_dispatch_window"]["rgb_deadline"], deadline)
        self.assertEqual(result["rgb_dispatch_window"]["required_postsend_window_s"], 9.)
        self.assertFalse(result["rgb_dispatch_window"]["window_is_completion_guarantee"])
        self.assertEqual(result["joint_validation_timing"]["static_geometry_policy"],
                         "coarse_box_memoization_not_applicable")

    def test_rgb_expiry_during_final_sample_validation_prevents_first_frame(self):
        ctx, target = self.make_visual()
        original = adapter.validate_joint_path_sample
        def slow_validation(plan, sample, **kwargs):
            result = original(plan, sample, **kwargs)
            ticket = self.device._action.ticket
            if ticket is not None and ticket["bus_calls"] == 0:
                self.clock.sleep(plan["visual_rgb_deadline"]-self.clock.time()+.001)
            return result
        with patch.object(adapter, "validate_joint_path_sample", side_effect=slow_validation):
            result = self.execute(ctx, target)
        self.assert_latched_once(result, [])
        self.assertIn("visual_rgb_expired", result["errors"][-1]["detail"])

    def test_original_deadline_between_frames_does_not_allow_completion(self):
        ctx, target = self.make_visual()
        def guard():
            if len(self.ids()) == 1:
                self.clock.sleep(61)
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151])
        self.assertIn("original task deadline", result["errors"][-1]["detail"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_rgb_partial_send_never_builds_complete_fact_cache_or_retries(self):
        ctx, target = self.make_visual()
        self.robots["right"].partial = True
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156])
        self.assertIsNone(result["original_event"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target)
        self.assertEqual(len(self.ids()), 3)

    def test_rgb_send_exception_never_completes_or_replaces_original(self):
        ctx, target = self.make_visual()
        self.robots["right"].fail_id = 0x156
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151, 0x155])
        self.assertIsNone(result["original_event"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_rgb_cancellation_or_eof_guard_after_full_send_has_no_hold(self):
        ctx, target = self.make_visual()
        def guard():
            if len(self.ids()) == 4:
                self.guard_error = "client EOF/cancellation"
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result["original_event"]["schema"], adapter.RGB_JOINT_SEND_SCHEMA)
        self.assertFalse(result["arrival_confirmed"])
        # The complete target history is factual; it cannot admit a later
        # action while the same connection's fault remains latched.
        self.assertIsNotNone(self.device.joint_binding("right")["cached_target"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target)
        self.assertEqual(len(self.ids()), 4)

    def test_rgb_four_frame_sending_remains_strict(self):
        ctx, target = self.make_visual()
        def hook(robot, state):
            if robot.side == "right" and self.ids():
                state["joints_rad"][4] += .0031
        self.hook = hook
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151])
        self.assertIn("joint_tracking_envelope", result["errors"][-1]["detail"])
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"], 0.)
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_rgb_peer_drift_between_frames_is_not_visual_permission(self):
        ctx, target = self.make_visual()
        def hook(robot, state):
            if robot.side == "left" and self.ids():
                state["joints_rad"][0] += .004
        self.hook = hook
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_rgb_raw_controller_displacement_guard_survives_model_geometry_split(self):
        ctx, target = self.make_visual()
        def hook(robot, state):
            if robot.side == "right" and len(self.ids()) == 4:
                state["pose_m_rad"][0] += .0201
        self.hook = hook
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(result["errors"][-1], {"type": "JointPathError", "detail": "right"})


class RGBJointSettlingTests(JointFixture):
    make_visual = RGBJointAdapterTests.make_visual
    assert_latched_once = RGBJointAdapterTests.assert_latched_once

    def response(self, function):
        def hook(robot, state):
            sent = self.device._action.sent_at
            if robot.side == "right" and sent is not None:
                function(self.clock.time()-sent, state)
        self.hook = hook

    def assert_four_frames(self, result):
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertIsNone(result["hold_receipt"])
        self.assertIsNone(result["physical_stop_verified"])

    def test_short_postsend_transient_settles_then_requires_original_final_window(self):
        ctx, target = self.make_visual()
        self.response(lambda dt, s: s["joints_rad"].__setitem__(4,
            s["joints_rad"][4]+(.015 if dt < .6 else 0.)))
        result = self.execute(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assert_four_frames(result)
        report = result["tracking_observation"]
        self.assertEqual(report["mode"], "bounded_postsend_settling")
        self.assertGreaterEqual(report["cumulative_outside_nominal_band_s"], .59)
        self.assertLess(report["cumulative_outside_nominal_band_s"], .7)
        self.assertAlmostEqual(report["max_excess_rad"], .012)
        self.assertEqual(report["first_outside_nominal_band"]["tracking"]["outside_nominal_band"][0]["joint_index"], 5)
        self.assertIsNone(report["first_failure"])
        self.assertGreaterEqual(result["observed_stable_duration_s"], 3.)
        self.assertGreaterEqual(result["observed_feedback_advances"], 20)
        self.assertTrue(result["joint_path_observation"]["tracking"]["cumulative_time_checked"])
        self.assertLessEqual(max(abs(a-b) for a,b in zip(result["after"]["right"]["joints_rad"],
            result["joint_path_plan"]["encoded_target_joints_rad"])), .003)

    def test_sustained_postsend_deviation_latches_after_one_cumulative_second(self):
        ctx, target = self.make_visual()
        self.response(lambda dt, s: s["joints_rad"].__setitem__(4, s["joints_rad"][4]+.008))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        report = result["tracking_observation"]
        self.assertGreater(report["cumulative_outside_nominal_band_s"], 1.)
        self.assertIn("cumulative", result["errors"][-1]["detail"])
        self.assertEqual(report["first_failure"]["sample_role"], "rejected_observation")
        self.assertEqual(report["first_failure"]["tracking"]["outside_nominal_band"][0]["joint_index"], 5)
        self.assertFalse(result["arrival_confirmed"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target)
        self.assertEqual(len(self.ids()), 4)

    def test_reentry_axis_change_and_arrival_window_reset_do_not_renew_time(self):
        ctx, target = self.make_visual()
        def pulses(dt, state):
            if dt < .35 or 1.0 <= dt < 1.35:
                state["joints_rad"][4] += .008
            elif .5 <= dt < .85:
                state["joints_rad"][5] += .008
        self.response(pulses)
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertGreater(result["tracking_observation"]["cumulative_outside_nominal_band_s"], 1.)
        self.assertIn("cumulative", result["errors"][-1]["detail"])

    def test_metric_postsend_observation_keeps_original_strict_band(self):
        ctx, target = self.make_context()
        self.response(lambda dt, s: s["joints_rad"].__setitem__(4, s["joints_rad"][4]+.004))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assertFalse(result["ok"])
        self.assertIn("joint_tracking_envelope", result["errors"][-1]["detail"])
        self.assertEqual(result["tracking_observation"]["mode"], "strict")
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"], 0.)

    def test_partial_dispatch_never_enters_settling_or_preserves_old_cache(self):
        ctx, target = self.make_visual()
        self.robots["right"].partial = True
        result = self.execute(ctx, target)
        self.assert_latched_once(result, [0x151, 0x155, 0x156])
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"], 0.)
        self.assertIsNone(result["original_event"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_full_returned_frames_with_corrupt_comm_count_cannot_enter_settling(self):
        ctx, target = self.make_visual()
        original = self.robots["right"].move_j
        def corrupt_count(target):
            original(target)
            self.device._action.ticket["comm_calls"] = 3
        self.robots["right"].move_j = corrupt_count
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assertFalse(result["ok"])
        self.assertIn("Incomplete", result["errors"][-1]["detail"])
        self.assertEqual(result["tracking_observation"]["cumulative_outside_nominal_band_s"], 0.)
        self.assertIsNone(result["original_event"])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_postsend_origin_excursion_cap_is_point_zero_two_eight_not_startup_point_one(self):
        ctx, target = self.make_context()
        target[5] = ctx["origin"]["arms"]["right"]["joints_rad"][5]+.01
        ctx, target = visual_joint_context(ctx, target)
        self.response(lambda dt, s: s["joints_rad"].__setitem__(5,
            ctx["origin"]["arms"]["right"]["joints_rad"][5]+.0281))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertAlmostEqual(result["joint_path_plan"]["tracking_policy"]["max_origin_excursion_rad"], .028)
        self.assertAlmostEqual(result["tracking_observation"]["max_origin_excursion_rad"], .0281)
        self.assertIn(result["tracking_observation"]["first_failure"]["code"],
            ("joint_tracking_envelope", "joint_origin_excursion"))

    def test_postsend_joint_nominal_limit_remains_hard(self):
        ctx, target = self.make_visual()
        self.response(lambda dt, s: s["joints_rad"].__setitem__(1, -.0001))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertEqual(result["tracking_observation"]["first_failure"]["code"], "feedback_joint_limit")

    def test_postsend_stale_feedback_is_rejected_without_settling_allowance(self):
        ctx, target = self.make_visual()
        self.response(lambda dt, s: s["fragment_timestamps_s"].update(joint_56=self.clock.time()-.051))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertEqual(result["tracking_observation"]["first_failure"]["code"], "stale_feedback")

    def test_postsend_jaw_drift_remains_hard(self):
        ctx, target = self.make_visual()
        self.response(lambda dt, s: s["gripper"].update(width_m=s["gripper"]["width_m"]+.000501))
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertEqual(result["tracking_observation"]["first_failure"]["code"], "jaw_anchor_drift")

    def test_postsend_original_deadline_is_not_extended_by_receiving(self):
        ctx, target = self.make_visual()
        delayed = False
        def guard():
            nonlocal delayed
            if len(self.ids()) == 4 and not delayed:
                delayed = True
                self.clock.sleep(61)
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assert_four_frames(result)
        self.assert_latched_once(result, self.ids())
        self.assertIn("original task deadline", result["errors"][-1]["detail"])


class PairJointAdapterTests(JointFixture):
    def test_joint_diagnostics_use_radians_after_arrival_without_cartesian_target_error(self):
        ctx, target = self.make_context()
        result = self.execute(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertIsNone(result["pose_error"])
        expected = [abs(actual-requested) for actual, requested in
                    zip(result["after"]["right"]["joints_rad"], target)]
        self.assertEqual(result["joint_error"], {"absolute_rad": expected,
                         "max_abs_rad": max(expected), "reference": "requested_target"})
        self.assertLess(result["joint_error"]["max_abs_rad"], .00001)
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(result["hardware_commands_sent"], 4)

    def test_failed_joint_send_preserves_angle_baseline_diagnostic_without_fake_meters(self):
        ctx, target = self.make_context()
        self.robots["right"].fail_id = 0x151
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertFalse(result["arrival_confirmed"])
        self.assertIsNone(result["pose_error"])
        expected = [abs(actual-requested) for actual, requested in
                    zip(result["before"]["right"]["joints_rad"], target)]
        self.assertEqual(result["joint_error"]["absolute_rad"], expected)
        self.assertAlmostEqual(result["joint_error"]["max_abs_rad"], .001)
        self.assertEqual(self.ids(), [])
        self.assertEqual(self.ids("left"), [])

    def test_unestablished_cache_is_explicit_zero_tx_bootstrap_gap(self):
        ctx, target = self.make_context(known_cache=False)
        with self.assertRaisesRegex(RuntimeError, "bootstrap gap"):
            self.execute(ctx, target)
        self.assertEqual(self.ids(), [])
        self.assertEqual(self.ids("left"), [])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_exact_joint_sequence_and_model_raw_frames_remain_separate(self):
        ctx, target = self.make_context()
        original = copy.deepcopy(ctx)
        bridge = Bridge(self)
        result = self.execute(ctx, target, bridge)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(bytes(self.robots["right"].sent[0].data), bytes((1, 1, 1, 0, 0, 0, 0, 0)))
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertEqual(bridge.original["worker_thread_id"], threading.get_ident())
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["joint_path_plan"]["controller_flange_pose"]["right"],
                         original["origin"]["arms"]["right"]["pose_m_rad"])
        self.assertIn("model_target_flange_transform", result["joint_path_plan"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertEqual(ctx, original)
        self.assertTrue(self.device.observe()["stationary_observed"])

    def test_both_arms_reuse_connections_and_only_selected_can_transmit(self):
        for side in ("left", "right"):
            ctx, target = self.make_context(side)
            result = self.execute(ctx, target)
            self.assertTrue(result["ok"], result.get("errors"))
            self.assertEqual(result["passive_arm_commands_sent"], 0)
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), self.ids())
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)

    def test_forged_origin_or_cache_has_zero_tx(self):
        ctx, target = self.make_context()
        ctx["origin"]["arms"]["right"]["joints_rad"][0] += .0001
        ctx["origin_sha256"] = joint_path.evidence_sha256(ctx["origin"])
        with self.assertRaisesRegex(RuntimeError, "not issued"):
            self.execute(ctx, target)
        ctx, target = self.make_context()
        ctx["cached_target"]["event_id"] = "invented"
        with self.assertRaisesRegex(RuntimeError, "returned frame history"):
            self.execute(ctx, target)
        self.assertEqual(self.ids(), [])

    def test_joint_partial_send_is_never_completed_or_replaced(self):
        ctx, target = self.make_context()
        self.robots["right"].partial = True
        bridge = Bridge(self, cancel=True)
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156])
        self.assertEqual(bridge.events, [])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 3)

    def test_failed_original_frame_no_hold_or_retry(self):
        ctx, target = self.make_context()
        self.robots["right"].fail_id = 0x156
        bridge = Bridge(self, cancel=True)
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151, 0x155])
        self.assertEqual(bridge.events, [])
        self.assertEqual(result["hardware_commands_sent"], 2)

    def test_alarm_after_original_send_is_not_cancel_permission(self):
        ctx, target = self.make_context()
        bridge = Bridge(self)
        def hook(robot, state):
            if robot.side == "left" and len(self.ids()) == 4:
                state["arm_status"]["arm_status"] = 4
        self.hook = hook
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(bridge.events, ["original"])

    def test_slow_host_guard_rechecks_fresh_feedback_before_first_frame(self):
        ctx, target = self.make_context()
        def guard():
            ticket = self.device._action.ticket
            if ticket is not None and ticket["comm_calls"] == 1:
                self.clock.sleep(.08)
                self.joints["left"][0] += .004
        self.guard_hook = guard
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [])

    def test_wrong_joint_frame_flags_denied_by_existing_bus_whitelist(self):
        ctx, target = self.make_context()
        def corrupt(frame):
            frame.is_extended_id = True
            return frame
        self.robots["right"].frame_transform = corrupt
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [])
        self.assertTrue(result["guard_violations"])

    def test_cancel_full_original_then_one_hold_preserves_fault_and_no_stop_claim(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157]*2, result.get("errors"))
        self.assertTrue(result["hold_receipt"]["hold_observed"], result.get("errors"))
        self.assertTrue(result["hold_receipt"]["frames_complete"])
        self.assertEqual(result["hardware_commands_sent"], 8)
        self.assertEqual(result["session_transmission_counts"]["right"]["sent_frames"], 8)
        self.assertIsNotNone(self.device._fault)
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertIsNone(result["hold_receipt"]["original_target_cancelled"])
        self.assertEqual(bridge.events[:2], ["original", "claim"])
        self.assertEqual(bridge.events[-1], "finish")
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .049)
        self.assertEqual(len(self.ids()), 8)

    def test_hold_durable_begin_delay_rechecks_peer_before_any_hold_frame(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        def delayed(index):
            self.clock.sleep(.08)
            self.joints["left"][0] += .004
        bridge.begin_hook = delayed
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4, result.get("errors"))
        self.assertFalse(result["hold_receipt"]["frames_complete"])
        self.assertIsNotNone(self.device._fault)

    def test_hold_partial_failure_keeps_original_and_hold_fault_no_retry(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        def fail_later(index):
            if index == 1:
                self.robots["right"].fail_id = 0x156
        bridge.begin_hook = fail_later
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 6, result.get("errors"))
        self.assertEqual(result["hold_receipt"]["status"], "fault")
        self.assertIsNotNone(result["hold_receipt"]["original_fault"])
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 6)

    def test_durable_original_journal_failure_never_grants_hold(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        bridge.record_original = lambda event: (_ for _ in ()).throw(OSError("journal unavailable"))
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4)
        self.assertEqual(bridge.events, [])
        self.assertIsNone(result["hold_receipt"])

    def test_joint_actions_keep_retained_peer_anchor_and_never_move_retained_arm(self):
        ctx, target = self.make_context()
        origin = ctx["origin"]["arms"]["left"]
        self.device._action.grasps["left"] = {"status": "retained_static", "arm": "left",
            "original_anchor": measured_anchor(origin),
            "retention_contract": {"valid_until": self.clock.time()+60}}
        result = self.execute(ctx, target, operation="approach")
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertIsNotNone(self.device.grasp_states["left"])
        ctx, target = self.make_context("left")
        with self.assertRaisesRegex(RuntimeError, "retained selected arm"):
            self.execute(ctx, target)
        self.assertEqual(self.ids("left"), [])

    def test_unresolved_peer_candidate_blocks_joint_motion(self):
        ctx, target = self.make_context()
        self.device._action.grasps["left"] = {"status": "contact_candidate", "arm": "left"}
        with self.assertRaisesRegex(RuntimeError, "candidate"):
            self.execute(ctx, target)
        self.assertEqual(self.ids(), [])

    def test_cleared_worker_can_retreat_with_static_peer_but_opened_or_held_worker_cannot(self):
        ctx, target = self.make_context()
        self.device._action.grasps["left"] = {"status": "retained_static", "arm": "left",
            "original_anchor": measured_anchor(ctx["origin"]["arms"]["left"]),
            "retention_contract": {"valid_until": self.clock.time()+60}}
        result = self.execute(ctx, target, operation="release_retreat")
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        # These private records are negative admission fixtures, not evidence.
        for side, status in (("right", "retained_static"), ("right", "release_opened"),
                             ("left", "release_opened")):
            with self.subTest(side=side, status=status):
                self.device._action.grasps["right"] = None
                self.device._action.grasps[side] = {"status": status, "arm": side}
                with self.assertRaisesRegex(RuntimeError, "candidate or retained"):
                    self.execute(ctx, target, operation="release_retreat")
                self.assertEqual(len(self.ids()), 4)

    def test_cartesian_target_attempt_invalidates_old_joint_cache(self):
        ctx, target = self.make_context()
        target = self.robots["right"].motion.origin[:]
        target[2] += .006
        result = self.device.execute("right", "move", target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_retained_peer_drift_during_joint_motion_still_faults(self):
        ctx, target = self.make_context()
        self.device._action.grasps["left"] = {"status": "retained_static", "arm": "left",
            "original_anchor": measured_anchor(ctx["origin"]["arms"]["left"]),
            "retention_contract": {"valid_until": self.clock.time()+60}}
        def hook(robot, state):
            if robot.side == "left" and self.ids():
                state["gripper"]["width_m"] += .000501
        self.hook = hook
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x151])
        self.assertEqual(self.ids("left"), [])

    def test_feedback_regression_after_complete_send_faults_without_retry(self):
        ctx, target = self.make_context()
        def hook(robot, state):
            if robot.side == "left" and len(self.ids()) == 4:
                state["fragment_timestamps_s"]["joint_12"] -= .002
        self.hook = hook
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4)
        self.assertIn("regressed", result["errors"][-1]["detail"])

    def test_clock_rollback_after_intent_is_zero_tx(self):
        ctx, target = self.make_context()
        old_journal = self.device._action.journal
        def journal(event, data):
            old_journal(event, data)
            if event == "pair_joint_intent":
                self.clock.sleep(-.1)
        self.device._action.journal = journal
        result = self.execute(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [])

    def test_explicit_cancel_between_poll_and_guard_still_has_one_hold(self):
        ctx, target = self.make_context()
        bridge = Bridge(self)
        def guard():
            if len(self.ids()) == 4 and bridge.original is not None:
                bridge.cancel = True
                self.guard_error = "explicit joint cancellation"
        self.guard_hook = guard
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 8, result.get("errors"))
        self.assertTrue(result["hold_receipt"]["hold_observed"])

    def test_boundary_recovery_transport_gap_rejects_before_any_dispatch(self):
        ctx, target = self.make_context()
        with self.assertRaisesRegex(ValueError, "boundary-origin"):
            self.execute(ctx, target, recovery_mode="unloaded_startup_j2_j3", operation="recover")
        self.assertEqual(self.ids(), [])

    def test_hold_journal_begin_failure_sends_no_hold_frame(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        bridge.record_frame_begin = lambda *args: (_ for _ in ()).throw(OSError("pending fsync failed"))
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 4)
        self.assertEqual(result["hold_receipt"]["status"], "fault")
        self.assertFalse(result["hold_receipt"]["frames_complete"])

    def test_hold_journal_return_failure_never_repeats_returned_frame(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        bridge.record_frame_return = lambda *args, **kwargs: (_ for _ in ()).throw(OSError("return fsync failed"))
        result = self.execute(ctx, target, bridge)
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.ids()), 5)
        self.assertFalse(result["hold_receipt"]["frames_complete"])
        self.assertEqual(result["hardware_commands_sent"], 5)
        with self.assertRaises(RuntimeError):
            self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 5)

    def test_hold_claim_latency_small_feedback_change_keeps_frozen_legal_target(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        original = bridge.claim_hold
        def claim(*args):
            result = original(*args)
            self.joints["right"][5] += math.radians(.001)
            return result
        bridge.claim_hold = claim
        result = self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 8, result.get("errors"))
        self.assertTrue(result["hold_receipt"]["hold_observed"])
        self.assertEqual(result["hold_receipt"]["target_raw"], bridge.payload["target_raw"])

    def test_hold_first_frame_waits_for_real_fragment_advancement(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        def delay_rx(index):
            if index != 0:
                return
            prepared = copy.deepcopy(self.device._action.joint_executor.hold_prepare_sample)
            until = self.clock.time()+.025
            def hook(robot, state):
                if self.clock.time() < until:
                    state["fragment_timestamps_s"] = copy.deepcopy(prepared["arms"][robot.side]["fragment_timestamps_s"])
            self.hook = hook
        bridge.begin_hook = delay_rx
        result = self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 8, result.get("errors"))
        self.assertTrue(result["hold_receipt"]["hold_observed"])

    def test_hold_nonadvancing_fragments_cannot_authorize_first_frame(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        def stale_rx(index):
            prepared = copy.deepcopy(self.device._action.joint_executor.hold_prepare_sample)
            def hook(robot, state):
                state["fragment_timestamps_s"] = copy.deepcopy(prepared["arms"][robot.side]["fragment_timestamps_s"])
            self.hook = hook
        bridge.begin_hook = stale_rx
        result = self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 4)
        self.assertFalse(result["hold_receipt"]["frames_complete"])
        self.assertIsNotNone(self.device._fault)

    def test_incomplete_bridge_rejects_before_any_target(self):
        ctx, target = self.make_context()
        with self.assertRaisesRegex(ValueError, "Complete durable"):
            self.execute(ctx, target, SimpleNamespace())
        self.assertEqual(self.ids(), [])

    def test_independent_abort_during_durable_begin_blocks_all_hold_tx(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        bridge.begin_hook = lambda index: setattr(bridge, "aborted", "EOF during hold")
        result = self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 4)
        self.assertFalse(result["hold_receipt"]["frames_complete"])
        self.assertEqual(result["hold_receipt"]["status"], "fault")

    def test_independent_abort_after_final_rx_stops_current_hold_frame(self):
        ctx, target = self.make_context()
        bridge = Bridge(self, cancel=True)
        def hook(robot, state):
            runner = self.device._action.joint_executor
            if runner and runner.phase == "hold" and self.device._action.ticket is not None:
                bridge.aborted = "watchdog after frame preflight snapshot"
        self.hook = hook
        result = self.execute(ctx, target, bridge)
        self.assertEqual(len(self.ids()), 4)
        self.assertFalse(result["hold_receipt"]["frames_complete"])


class RealSDKJointAdapterTests(unittest.TestCase):
    def run_sdk(self, *, cancel=False, wait_drift=False, visual=False):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.drivers.core import driver_context
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, created, bindings, send_times = Clock(), [], [], {}, []
        clock.monotonic = lambda: clock.elapsed
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper_x"
        channels = [cfg["channel"] for cfg in profile["arms"].values()]
        joints = {channel: RAW[:] for channel in channels}
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def shutdown(self):
                pass
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                send_times.append(clock.time())
                if 0x155 <= frame.arbitration_id <= 0x157:
                    index = 2*(frame.arbitration_id-0x155)
                    joints[self.channel][index:index+2] = struct.unpack(">ii", bytes(frame.data))
                clock.sleep(.0001)
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            bindings[id(robot)] = config["comm"]["can"]["channel"]
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1, teach_status=0,
                mode_feedback=1, motion_status=0, arm_status=0, err_code=0))
            state["joints_rad"] = [math.radians(v/1000) for v in joints[bindings[id(robot)]]]
            if (wait_drift and len(sent) == 4 and clock.time()-send_times[-1] > .025
                    and bindings[id(robot)] == profile["arms"]["left"]["channel"]):
                state["joints_rad"][0] += .004
            state["gripper"]["width_m"] = .05
            # Raw controller geometry deliberately differs from X-model FK.
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("Real socket forbidden")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, linear_hold, supervised_actions,
                           single_supervised_actions, pair_device, adapter):
                stack.enter_context(patch.object(module, "time", clock))
            # Vendor duplicate filtering and adapter must use the SAME clock.
            # Keep background-thread sleep real so it cannot advance fake time.
            stack.enter_context(patch.object(driver_context, "time", SimpleNamespace(
                monotonic=clock.monotonic, time=clock.time, sleep=time.sleep)))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile, lambda *args: None)
            try:
                device.open()
                self.assertEqual(sent, [], "SDK open must not send implicitly")
                ctx = context_fixture()
                ctx["identity"].update({k: v for k, v in device.joint_binding("right").items() if k != "cached_target"})
                ctx["origin"] = device.observe_joint(ctx["identity"])
                ctx["current"] = copy.deepcopy(ctx["origin"])
                ctx["origin_sha256"] = joint_path.evidence_sha256(ctx["origin"])
                ctx["geometry"]["origin_sample_id"] = ctx["origin"]["sample_id"]
                ctx["cached_target"]["identity"] = copy.deepcopy(ctx["identity"])
                for index, receipt in enumerate(ctx["cached_target"]["frame_receipts"]):
                    receipt["returned_at"] = clock.time()-.1+index*.001
                device._joint_cache["right"] = copy.deepcopy(ctx["cached_target"])
                target = [math.radians(v/1000)+.0005 for v in RAW]
                if visual:
                    ctx, target = visual_joint_context(ctx, target)
                bridge = None if visual else Bridge(SimpleNamespace(clock=clock), cancel=cancel)
                result = device.execute_joint("right", target, context=ctx, event_id="joint-sdk",
                    deadline_at=clock.time()+60, hold_bridge=bridge)
                expected = [0x151, 0x155, 0x156, 0x157]*(2 if cancel and not wait_drift else 1)
                self.assertEqual([frame.arbitration_id for _, frame in sent], expected, result.get("errors"))
                self.assertTrue(all(channel == profile["arms"]["right"]["channel"] for channel, _ in sent))
                self.assertEqual(len(created), 2)
                self.assertEqual(result["ok"], not cancel, result.get("errors"))
                if visual:
                    self.assertEqual(result["original_event"]["schema"], adapter.RGB_JOINT_SEND_SCHEMA)
                    self.assertEqual(result["hold_policy"], "latch_only")
                    self.assertFalse(result["hold_supported"])
                    self.assertFalse(result["explicit_cancel_hold_bridge_bound"])
                    self.assertIsNone(result["hold_receipt"])
                    self.assertEqual(device.joint_binding("right")["cached_target"]["target_raw"],
                                     joint_path.encode_joint_target(target)[0])
                if cancel and not wait_drift:
                    self.assertTrue(result["hold_receipt"]["hold_observed"], result.get("errors"))
                    self.assertIsNotNone(device._fault)
                    self.assertGreater(result["sdk_mode_repeat_wait_s"], .09)
                if wait_drift:
                    self.assertIsNone(result["hold_receipt"])
                    self.assertEqual(bridge.events, ["original"])
                self.assertTrue(all(robot._ctx._tx_repeat_min_interval[0x151] == .1 for robot in created))
                self.assertIsNone(result["physical_stop_verified"])
            finally:
                device.close()
            self.assertEqual(len(sent), len(expected))

    def test_real_vendor_move_j_uses_only_expected_frames_no_reconnect(self):
        self.run_sdk()

    def test_real_vendor_rgb_uses_same_four_frames_without_metric_hold_contract(self):
        self.run_sdk(visual=True)

    def test_real_vendor_explicit_cancel_uses_one_same_mode_hold(self):
        self.run_sdk(cancel=True)

    def test_real_vendor_repeat_wait_monitors_peer_and_never_sends_on_drift(self):
        self.run_sdk(cancel=True, wait_drift=True)


if __name__ == "__main__":
    unittest.main()
