"""Offline persistent transport tests; all real sockets and CAN buses blocked."""
import copy
import hashlib
import json
import math
import struct
import threading
import time
import unittest
from contextlib import ExitStack
from unittest.mock import patch

from robot_tools import arms, linear_hold, pair_device, single_supervised_actions
from robot_tools import supervised_actions, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_linear_hold import Motion
from test_single_supervised_actions import SingleActionFixture


class PairDeviceTests(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.stack.enter_context(patch.object(pair_device, "time", self.clock))
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True] * 6
            robot.gripper_enabled = True
        self.guard_error = None
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)), self.guard)
        self.addCleanup(self.device.close)

    def guard(self):
        if self.guard_error:
            raise RuntimeError(self.guard_error)

    def test_open_observe_close_zero_tx_and_fresh_baseline(self):
        first = self.device.open()
        second = self.device.observe()
        self.assertGreater(second["sequence"], first["sequence"])
        self.assertGreater(second["arms"]["left"]["timestamp"], first["arms"]["left"]["timestamp"])
        self.assertGreaterEqual(first["baseline_duration_s"], 3)
        self.assertGreaterEqual(first["feedback_advances"], 20)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))
        self.assertTrue(all(not robot.disconnect.called for robot in self.robots.values()))
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        result = self.device.close()
        self.assertIsNone(result["physical_stop_verified"])
        self.assertTrue(all(robot.disconnect.call_count == 1 for robot in self.robots.values()))

    def test_both_arms_and_both_jaws_must_be_enabled(self):
        self.robots["left"].gripper_enabled = False
        with self.assertRaisesRegex(RuntimeError, "both CAN arms"):
            self.device.open()
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_fault_feedback_retains_disabled_alarm_states_without_guard_or_tx(self):
        opened = self.device.open()
        self.guard_error = "latched cancellation"
        with self.assertRaises(RuntimeError):
            self.device.observe()
        fault = self.device._fault
        self.robots["left"].driver_enabled = [False] * 6
        self.robots["left"].gripper_enabled = False
        self.robots["left"].ctrl_mode = 0
        def alarm(robot, state):
            if robot.side == "left":
                state["arm_status"]["arm_status"] = 4
        self.hook = alarm
        self.clock.sleep(.02)
        with patch.object(self.device._action, "guard", side_effect=AssertionError("RX called dispatch guard")), \
             patch.object(self.device._action, "read", side_effect=AssertionError("RX called guarded action reader")):
            report = self.device.observe_fault_feedback()
        self.assertGreater(report["arms"]["left"]["timestamp"], opened["arms"]["left"]["timestamp"])
        self.assertFalse(report["arms"]["left"]["drivers"]["1"]["foc_status"]["driver_enable_status"])
        self.assertEqual(report["arms"]["left"]["arm_status"]["arm_status"], 4)
        self.assertFalse(report["diagnostics"]["left"]["health"]["healthy"])
        self.assertIsNone(report["physical_stop_verified"])
        self.assertEqual(self.device._fault, fault)
        self.guard_error = None
        with self.assertRaises(RuntimeError):
            self.device.execute("right", "gripper", .03)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_fault_feedback_stale_missing_regression_are_not_renewed(self):
        opened = self.device.open()
        stale = copy.deepcopy(opened["arms"])
        self.clock.sleep(.3)
        def old(robot, state):
            state.update(copy.deepcopy(stale[robot.side]))
            if robot.side == "left":
                del state["fragment_timestamps_s"]["joint_12"]
                state["fragment_timestamps_s"]["joint_34"] -= .01
        self.hook = old
        report = self.device.observe_fault_feedback()
        fragments = report["diagnostics"]["left"]["fragments"]
        self.assertEqual(fragments["joint_12"]["progress"], "missing")
        self.assertEqual(fragments["joint_34"]["progress"], "regressed")
        self.assertEqual(fragments["gripper"]["progress"], "repeated")
        self.assertIn("stale", fragments["gripper"]["issues"])
        self.assertEqual(report["arms"]["left"]["timestamp"], stale["left"]["timestamp"])
        self.assertIsNone(report["stationary_observed"])
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_fault_feedback_decoder_failure_retains_other_arm(self):
        self.device.open()
        def read(robot, jaw):
            if robot.side == "left":
                raise ValueError("malformed timestamp")
            return self.snapshot(robot, jaw)
        with patch.object(arms, "snapshot", side_effect=read):
            report = self.device.observe_fault_feedback()
        self.assertIsNone(report["arms"]["left"])
        self.assertIn("malformed timestamp", report["read_errors"]["left"])
        self.assertIsNotNone(report["arms"]["right"])
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_fault_feedback_does_not_wait_for_operation_or_reconnect_after_close(self):
        self.device.open()
        with self.device._operation:
            result = []
            thread = threading.Thread(target=lambda: result.append(self.device.observe_fault_feedback()))
            thread.start()
            thread.join(.5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result[0]["status"], "deferred")
        self.device.close()
        with patch.object(arms, "snapshot", side_effect=AssertionError("read after close")):
            self.assertEqual(self.device.observe_fault_feedback()["status"], "unavailable")
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_motion_effect_uses_dispatch_feedback_and_reports_no_response_within_arrival_tolerance(self):
        self.device.open()
        robot = self.robots["right"]
        robot.motion.speed = 0
        robot.motion_flag = 0
        target = robot.motion.origin[:]
        target[2] += .003
        result = self.device.execute("right", "move", target)
        self.assertTrue(result["ok"], result)
        effect = result["motion_effect"]
        self.assertEqual(effect["translation"]["response"], "no_discriminable_response")
        self.assertEqual(effect["translation"]["measured_norm"], 0.)
        self.assertAlmostEqual(effect["translation"]["requested_norm"], .003)
        self.assertEqual(effect["before_fragment_timestamps_s"],
                         result["dispatch_feedback"]["right"]["fragment_timestamps_s"])
        self.assertIsNone(effect["object_progress_measurement"])
        self.assertEqual(result["hardware_commands_sent"], 4)

    def test_open_does_not_prepare_or_takeover(self):
        self.robots["left"].ctrl_mode = 2
        with self.assertRaises(RuntimeError):
            self.device.open()
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_initialization_sender_is_blocked_before_connection(self):
        self.robots["left"].connect = lambda: self.robots["left"].set_motion_mode("l")
        with self.assertRaisesRegex(RuntimeError, "Initialization TX"):
            self.device.open()
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_close_cannot_send_or_claim_stop(self):
        self.device.open()
        self.robots["left"].disconnect.side_effect = lambda: self.robots["left"].set_motion_mode("l")
        result = self.device.close()
        self.assertEqual(result["arms"]["left"]["status"], "cleanup_failed")
        self.assertIsNone(result["physical_stop_verified"])
        self.assertTrue(result["guard_violations"])
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_left_right_jaw_rotation_reuses_same_connections(self):
        self.device.open()
        for side, width in (("left", .01), ("right", .02), ("left", .03), ("right", .04)):
            result = self.device.execute(side, "gripper", width)
            self.assertTrue(result["ok"], result)
            self.assertTrue(result["arrival_confirmed"])
            self.assertEqual(result["target_calls_sent"], 1)
            self.assertEqual(result["hardware_commands_sent"], 1)
            self.assertEqual(result["passive_arm_commands_sent"], 0)
            self.assertIsNone(result["accepted"])
            self.assertIsNone(result["physical_stop_verified"])
            self.assertFalse(result["grasp_verified"])
            self.assertFalse(result["capabilities"]["contact_step_supported"])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertTrue(all(not robot.disconnect.called for robot in self.robots.values()))
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x159, 0x159])
        self.assertEqual(result["session_transmission_counts"]["left"]["sent_frames"], 2)
        self.assertEqual(result["session_transmission_counts"]["right"]["sent_frames"], 2)

    def test_left_move_right_move_then_left_jaw(self):
        self.device.open()
        for side in ("left", "right"):
            target = self.robots[side].motion.origin[:]
            target[2] += .006
            result = self.device.execute(side, "move", target)
            self.assertTrue(result["ok"], result)
            self.assertEqual(result["hardware_commands_sent"], 4)
            self.assertEqual([f.arbitration_id for f in self.robots[side].sent], [0x151, 0x152, 0x153, 0x154])
        result = self.device.execute("left", "gripper", .02)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["session_transmission_counts"]["left"]["sent_frames"], 5)

    def test_stable_unarrived_jaw_is_failure_and_cannot_retry(self):
        self.device.open()
        self.robots["right"].accept = False
        result = self.device.execute("right", "gripper", .0)
        self.assertFalse(result["ok"])
        self.assertTrue(result["observed_stable"])
        self.assertFalse(result["arrival_confirmed"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("contact-limited", result["errors"][-1]["detail"])
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .02)
        self.assertEqual(self.robots["left"].sent, [])

    def test_stable_motion_one_is_failure(self):
        self.device.open()
        self.robots["right"].motion.speed = 0
        result = self.device.execute("right", "move", self.target)
        self.assertFalse(result["ok"])
        self.assertTrue(result["observed_stable"])
        self.assertFalse(result["controller_at_target"])
        self.assertEqual(result["hardware_commands_sent"], 4)

    def test_guard_between_frames_stops_remaining_frames_and_latches(self):
        self.device.open()
        old = self.robots["right"].frame_transform
        def transform(frame):
            if frame.arbitration_id == 0x153:
                self.guard_error = "host pair fault"
            return old(frame)
        self.robots["right"].frame_transform = transform
        result = self.device.execute("right", "move", self.target)
        self.assertFalse(result["ok"])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151, 0x152])
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.guard_error = None
        with self.assertRaises(RuntimeError):
            self.device.observe()

    def _delay_bus_guard(self, frame_index, mutate):
        action = self.device._action
        done = []
        def guard():
            self.guard()
            ticket = action.ticket
            if (ticket is not None and ticket["comm_calls"] == ticket["bus_calls"] + 1
                    and ticket["bus_calls"] == frame_index and not done):
                done.append(True)
                before = copy.deepcopy(action.report["dispatch_feedback"])
                self.clock.sleep(.15)  # e.g. synchronous SQLite contention
                mutate(before)
        action.guard = guard
        return done

    def test_guard_delay_before_first_frame_rejects_expired_feedback_zero_tx(self):
        self.device.open()
        def freeze(before):
            self.hook = lambda robot, state: state.update(copy.deepcopy(before[robot.side]))
        delayed = self._delay_bus_guard(0, freeze)
        result = self.device.execute("right", "move", self.target)
        self.assertTrue(delayed)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_guard_delay_between_frames_rejects_expired_feedback_no_completion_send(self):
        self.device.open()
        def freeze(before):
            self.hook = lambda robot, state: state.update(copy.deepcopy(before[robot.side]))
        self._delay_bus_guard(1, freeze)
        result = self.device.execute("right", "move", self.target)
        self.assertFalse(result["ok"])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151])
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 1)
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .02)

    def test_guard_delay_reads_new_peer_drift_before_next_frame(self):
        self.device.open()
        def drift(before):
            self.robots["left"].motion.origin[0] += .000501
        delayed = self._delay_bus_guard(1, drift)
        result = self.device.execute("right", "move", self.target)
        self.assertTrue(delayed)
        self.assertFalse(result["ok"])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151])
        self.assertIn("envelope", result["errors"][-1]["detail"])

    def test_guard_delay_accepts_current_healthy_feedback_without_recursive_guard(self):
        self.device.open()
        delayed = self._delay_bus_guard(1, lambda before: None)
        result = self.device.execute("right", "move", self.target)
        self.assertTrue(delayed)
        self.assertTrue(result["ok"], result)
        frames = result["frame_preflight_feedback"]
        self.assertEqual([frame["frame_index"] for frame in frames], [0, 1, 2, 3])
        self.assertGreater(frames[1]["arms"]["left"]["fragment_timestamps_s"]["joint_12"],
                           result["dispatch_feedback"]["left"]["fragment_timestamps_s"]["joint_12"])

    def test_idle_sender_denied_and_violation_persists(self):
        self.device.open()
        with self.assertRaises(RuntimeError):
            self.robots["left"].gripper.move_gripper_m(value=.01, force=.2)
        with self.assertRaises(RuntimeError):
            self.device.execute("right", "gripper", .02)
        self.assertTrue(self.device._action.violations)
        self.assertEqual(self.robots["left"].sent, [])

    def test_duplicate_encoder_frame_blocked(self):
        self.device.open()
        self.robots["left"].duplicate = True
        result = self.device.execute("left", "gripper", .02)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertTrue(result["guard_violations"])

    def test_swallowed_bus_error_is_failure(self):
        self.device.open()
        self.robots["right"].fail_id = 0x153
        self.robots["right"].swallow_error = True
        result = self.device.execute("right", "move", self.target)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 2)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 3)

    def test_idle_drift_is_rejected(self):
        self.device.open()
        self.robots["left"].motion.origin[0] += .000501
        with self.assertRaisesRegex(RuntimeError, "envelope|anchor"):
            self.device.observe()

    def test_disabled_driver_cannot_reset_at_next_action(self):
        self.device.open()
        self.robots["left"].driver_enabled[0] = False
        with self.assertRaises(RuntimeError):
            self.device.execute("right", "gripper", .02)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_changed_mode_cannot_reset_at_next_action(self):
        self.device.open()
        self.robots["left"].motion.mode = 1
        with self.assertRaises(RuntimeError):
            self.device.execute("right", "gripper", .02)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_stale_fragment_cannot_renew_observation(self):
        self.device.open()
        def stale(robot, state):
            state["fragment_timestamps_s"]["joint_34"] -= .101
        self.hook = stale
        with self.assertRaises(RuntimeError):
            self.device.observe()

    def test_guard_refusal_before_action_remains_latched(self):
        self.device.open()
        self.guard_error = "cancelled"
        with self.assertRaises(RuntimeError):
            self.device.execute("right", "gripper", .02)
        self.guard_error = None
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .02)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_observe_cannot_retimestamp_frozen_feedback(self):
        sample = self.device.open()
        frozen = copy.deepcopy(sample["arms"])
        with patch.object(arms, "snapshot", side_effect=lambda robot, jaw: copy.deepcopy(frozen[robot.side])):
            with self.assertRaises(RuntimeError):
                self.device.observe()
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_uncommanded_peer_anchor_does_not_walk_across_actions(self):
        self.device.open()
        self.robots["left"].motion.origin[0] += .0003
        self.assertTrue(self.device.execute("right", "gripper", .02)["ok"])
        self.robots["left"].motion.origin[0] += .0003
        with self.assertRaisesRegex(RuntimeError, "envelope|anchor"):
            self.device.execute("right", "gripper", .03)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def contact_response(self, width=.048, side="right"):
        self.robots[side].accept = False
        before = len(self.robots[side].sent)
        def contact(robot, state):
            if robot.side == side and any(frame.arbitration_id == 0x159 for frame in robot.sent[before:]):
                robot.width = width
                state["gripper"]["width_m"] = width
        self.hook = contact

    def candidate(self):
        self.device.open()
        self.contact_response()
        result = self.device.execute_gripper_probe("right", .0455)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["contact_observation"]["outcome"], "settled_contact_candidate")
        return result

    @staticmethod
    def grasp_identity(side):
        return {"episode_id": "episode-"+side, "arm": side, "run_id": "run-1",
                "owner": "owner-1", "epoch": "epoch-1", "object_id": "strip" if side == "left" else "plug"}

    def retain(self, side="right", deadline=None):
        pending = self.device.grasp_states[side]
        result = self.device.retain_grasp(side, identity=self.grasp_identity(side),
            probe_event_id="probe-"+side, probe_trace_sha256=pending["trace_sha256"],
            deadline_at=self.clock.time()+120 if deadline is None else deadline)
        self.assertTrue(result["ok"], result)
        return result

    def test_candidate_measurement_and_probe_bind_original_trace_without_retiming(self):
        result = self.candidate()
        probe, measured = result["candidate_probe"], result["candidate_measurement"]
        record = next(data for event, data in self.events if event == "bounded_probe_trace")
        self.assertEqual(probe["trace_sha256"], record["sha256"])
        self.assertEqual(measured["trace_sha256"], record["sha256"])
        self.assertGreater(measured["started_at"], probe["sent_at"])
        self.assertEqual(measured["ended_at"], probe["completed_at"])
        self.assertEqual(measured["ended_at"], record["trace"]["post"][-1]["observed_at_s"])
        self.assertGreaterEqual(measured["ended_at"]-measured["started_at"], 3.)
        self.assertGreaterEqual(measured["feedback_advances"], 20)
        self.assertNotIn("identity", measured)
        self.assertNotIn("probe_event_id", measured)
        self.assertNotIn("identity", probe)
        self.assertEqual(measured["anchor"], self.device.grasp_states["right"]["original_anchor"])

    def test_retain_issues_auditable_current_static_contract_without_tx_or_new_target(self):
        self.candidate()
        before = self.device.grasp_states["right"]
        counts = self.device._action.totals()
        report = self.retain()
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertEqual(report["target_calls_sent"], 0)
        self.assertEqual(self.device._action.totals(), counts)
        self.assertFalse(report["original_target_resent"])
        self.assertEqual(report["status"], "retained_static")
        self.assertIsNone(self.device.unresolved_gripper_probe)
        self.assertFalse(report["loaded"])
        self.assertFalse(report["contact_support_verified"])
        self.assertIsNone(report["physical_stop_verified"])
        self.assertEqual(self.device.grasp_states["right"]["original_anchor"], before["original_anchor"])
        self.assertEqual(self.device.grasp_states["right"]["trace_sha256"], before["trace_sha256"])
        trace = next(data for event, data in self.events if event == "static_grasp_trace")
        issuance = next(data for event, data in self.events if event == "static_grasp_contract")
        self.assertEqual(pair_device.digest(trace["trace"]), report["measurement"]["trace_sha256"])
        contract = copy.deepcopy(report["retention_contract"])
        contract_hash = contract.pop("artifact_sha256")
        self.assertEqual(pair_device.digest(contract), contract_hash)
        self.assertEqual(pair_device.digest(issuance["source"]), contract["source"]["artifact_sha256"])
        self.assertEqual(contract["source"]["trace_id"], report["measurement"]["trace_id"])
        self.assertGreaterEqual(contract["issued_at"], report["measurement"]["ended_at"])
        self.assertNotIn('"trace":', json.dumps(report))
        self.assertLess(len(json.dumps(report)), 1024*1024)
        self.assertTrue(self.device.close()["requires_fault_latch"])

    def test_observe_grasp_is_fresh_zero_tx_and_does_not_upgrade_candidate(self):
        probe = self.candidate()
        before = self.device._action.totals()
        report = self.device.observe_grasp("right", identity=self.grasp_identity("right"), probe_event_id="probe-right")
        self.assertTrue(report["ok"], report)
        self.assertEqual(report["status"], "contact_candidate")
        self.assertIsNone(report["retention_contract"])
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertEqual(self.device._action.totals(), before)
        self.assertGreater(report["measurement"]["started_at"], probe["candidate_probe"]["completed_at"])
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "move", self.robots["left"].motion.origin, operation="approach")

    def test_retention_cannot_change_identity_probe_hash_or_frozen_deadline(self):
        self.candidate()
        retained = self.retain()
        values = dict(identity=self.grasp_identity("right"), probe_event_id="probe-right",
                      probe_trace_sha256=self.device.grasp_states["right"]["trace_sha256"],
                      deadline_at=retained["retention_contract"]["valid_until"])
        for key, bad in (("identity", {**values["identity"], "object_id": "another"}),
                         ("probe_event_id", "other-probe"), ("probe_trace_sha256", "0"*64),
                         ("deadline_at", values["deadline_at"]+1)):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.device.retain_grasp("right", **{**values, key: bad})
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_retained_arm_cannot_move_or_receive_new_width_and_peer_needs_explicit_unloaded_operation(self):
        self.candidate()
        self.retain()
        for side, kind, operation in (("right", "move", "approach"), ("right", "gripper", "align"),
                                      ("left", "gripper", "approach"), ("left", "move", None),
                                      ("left", "move", "transport"), ("left", "move", "extract_segment")):
            target = .05 if kind == "gripper" else self.robots[side].motion.origin
            with self.subTest(side=side, kind=kind, operation=operation), self.assertRaises(RuntimeError):
                self.device.execute(side, kind, target, operation=operation)
        with self.assertRaises(RuntimeError):
            self.device.execute_gripper_probe("right", .046)
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertEqual(self.robots["left"].sent, [])

    def test_left_static_retention_allows_right_approach_probe_and_independent_release(self):
        self.device.open()
        self.contact_response(side="left")
        left = self.device.execute_gripper_probe("left", .0455)
        self.assertTrue(left["ok"], left)
        self.retain("left")
        self.hook = None
        target = self.robots["right"].motion.origin[:]
        target[2] += .006
        move = self.device.execute("right", "move", target, operation="approach")
        self.assertTrue(move["ok"], move)
        self.contact_response(side="right")
        right = self.device.execute_gripper_probe("right", .0455)
        self.assertTrue(right["ok"], right)
        self.assertEqual(self.device.grasp_states["left"]["status"], "retained_static")
        self.assertEqual(self.device.grasp_states["right"]["status"], "contact_candidate")
        self.retain("right")
        peer = self.device.grasp_states["right"]
        self.hook = None
        self.robots["left"].accept = True
        release = self.device.release_gripper_probe("left", .05)
        self.assertTrue(release["ok"], release)
        self.assertEqual(release["release_measurement"]["probe_event_id"], "probe-left")
        self.assertEqual(release["release_measurement"]["identity"], self.grasp_identity("left"))
        self.assertEqual(self.device.grasp_states["left"]["status"], "release_opened")
        self.assertEqual(self.device.grasp_states["right"], peer)
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x159, 0x159])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151,0x152,0x153,0x154,0x159])
        self.assertTrue(self.device.close()["requires_fault_latch"])

    def test_retained_peer_jaw_drift_between_frames_faults_without_resending_closure(self):
        self.device.open()
        self.contact_response(side="left")
        self.assertTrue(self.device.execute_gripper_probe("left", .0455)["ok"])
        self.retain("left")
        self.hook = None
        self._delay_bus_guard(1, lambda before: setattr(self.robots["left"], "width", .0474))
        result = self.device.execute("right", "move", self.target, operation="align")
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x159])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x151])
        with self.assertRaises(RuntimeError):
            self.device.retain_grasp("left", identity=self.grasp_identity("left"), probe_event_id="probe-left",
                probe_trace_sha256=self.device.grasp_states["left"]["trace_sha256"], deadline_at=self.clock.time()+100)

    def test_static_observation_cannot_rebase_cumulative_jaw_drift(self):
        self.candidate()
        self.retain()
        origin = self.device.grasp_states["right"]["original_anchor"]
        self.hook = None
        self.robots["right"].width = .0483
        result = self.device.observe_grasp("right", identity=self.grasp_identity("right"), probe_event_id="probe-right")
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.device.grasp_states["right"]["original_anchor"], origin)
        self.robots["right"].width = .0486
        result = self.device.observe_grasp("right", identity=self.grasp_identity("right"), probe_event_id="probe-right")
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_retention_journal_delay_cannot_issue_stale_contract_or_clear_candidate(self):
        self.candidate()
        original = self.device._action.journal
        def delayed(event, data):
            original(event, data)
            if event == "static_grasp_trace":
                self.clock.sleep(.15)
        self.device._action.journal = delayed
        result = self.device.retain_grasp("right", identity=self.grasp_identity("right"), probe_event_id="probe-right",
            probe_trace_sha256=self.device.grasp_states["right"]["trace_sha256"], deadline_at=self.clock.time()+100)
        self.assertFalse(result["ok"])
        self.assertIn("stale", result["error"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.device.grasp_states["right"]["status"], "contact_candidate")
        with self.assertRaises(RuntimeError):
            self.device.observe_grasp("right", identity=self.grasp_identity("right"), probe_event_id="probe-right")

    def test_first_probe_observes_candidate_with_one_frame_without_grasp_claim(self):
        result = self.candidate()
        self.assertEqual(result["completion_mode"], "contact_probe")
        self.assertTrue(result["controller_at_target"])
        self.assertFalse(result["arrival_confirmed"])
        self.assertGreater(result["width_error_m"], .002)
        self.assertIsNone(result["accepted"])
        self.assertIsNone(result["physical_stop_verified"])
        self.assertFalse(result["grasp_verified"])
        self.assertTrue(result["capabilities"]["gripper_contact_observation"])
        self.assertFalse(result["capabilities"]["contact_support_verified"])
        self.assertFalse(result["capabilities"]["contact_step_supported"])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x159])
        self.assertEqual(bytes(self.robots["right"].sent[0].data)[4:], bytes((0, 200, 1, 0)))
        self.assertEqual(self.robots["left"].sent, [])
        summary = result["contact_observation"]["trace_summary"]
        self.assertEqual(len(summary["sha256"]), 64)
        self.assertGreaterEqual(summary["baseline_samples"], 20)
        self.assertGreaterEqual(summary["post_samples"], 20)
        self.assertLess(len(json.dumps(result)), 1024 * 1024)
        self.assertNotIn("baseline_samples", result)
        pending = self.device.unresolved_gripper_probe
        pending["arm"] = "left"
        self.assertEqual(self.device.unresolved_gripper_probe["arm"], "right")

    def test_probe_and_release_trace_hashes_recompute_from_journal_not_ledger_payload(self):
        probe = self.candidate()
        self.hook = None
        self.robots["right"].accept = True
        release = self.device.release_gripper_probe("right", .05)
        self.assertTrue(release["ok"], release)
        records = [data for event, data in self.events if event == "bounded_probe_trace"]
        self.assertEqual(len(records), 2)
        for index, (record, result) in enumerate(zip(records, (probe, release))):
            digest = hashlib.sha256(json.dumps(record["trace"], sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            self.assertEqual(digest, record["sha256"])
            self.assertEqual(digest, result["contact_observation"]["trace_summary"]["sha256"])
            self.assertEqual(record["release"], bool(index))
            self.assertEqual(record["arm"], "right")
            self.assertEqual(set(record["trace"]), {"baseline", "post"})
            self.assertGreater(record["trace"]["post"][0]["observed_at_s"], record["sent_at"])
            encoded = json.dumps(result)
            self.assertNotIn('"trace":', encoded)
            self.assertNotIn('"observed_at_s":', encoded)
            self.assertLess(len(encoded.encode()), 1024*1024)

    def test_blocked_trace_journal_rechecks_fresh_drift_and_faults_without_extra_tx(self):
        self.device.open()
        original = self.device._action.journal
        def blocking(event, data):
            original(event, data)
            if event == "bounded_probe_trace":
                self.clock.sleep(.2)
                self.robots["left"].motion.origin[0] += .000501
        self.device._action.journal = blocking
        result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("envelope", result["errors"][-1]["detail"])
        self.assertEqual([frame.arbitration_id for frame in self.robots["right"].sent], [0x159])
        self.assertEqual(self.robots["left"].sent, [])
        with self.assertRaises(RuntimeError):
            self.device.execute_gripper_probe("right", .044)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_candidate_keeps_monitoring_and_blocks_all_ordinary_actions_and_reprobe(self):
        self.candidate()
        self.assertTrue(self.device.observe()["stationary_observed"])
        for side in ("left", "right"):
            for kind, target in (("gripper", .049), ("move", self.target)):
                with self.assertRaisesRegex(RuntimeError, "Unresolved"):
                    self.device.execute(side, kind, target)
            with self.assertRaisesRegex(RuntimeError, "Unresolved"):
                self.device.execute_gripper_probe(side, .047)
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertEqual(self.robots["left"].sent, [])
        closed = self.device.close()
        self.assertTrue(closed["requires_fault_latch"])
        self.assertEqual(closed["unresolved_gripper_probe"]["arm"], "right")
        self.assertIsNone(closed["physical_stop_verified"])
        with self.assertRaises(RuntimeError):
            self.device.open()

    def test_candidate_later_jaw_drift_faults_without_resend(self):
        self.candidate()
        self.hook = None
        self.robots["right"].width = .047499
        with self.assertRaisesRegex(RuntimeError, "Unresolved probe jaw"):
            self.device.observe()
        with self.assertRaises(RuntimeError):
            self.device.release_gripper_probe("right", .05)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_no_response_even_with_force_is_fault_not_contact_or_ack(self):
        self.device.open()
        self.robots["right"].accept = False
        self.hook = lambda robot, state: state["gripper"].update(force_N=3.0)
        result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"], result)
        self.assertEqual(result["contact_observation"]["outcome"], "unconfirmed")
        self.assertIsNone(result["accepted"])
        self.assertFalse(result["grasp_verified"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        with self.assertRaises(RuntimeError):
            self.device.execute_gripper_probe("right", .0455)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_probe_health_fault_is_not_hidden_by_a_quiet_final_window(self):
        self.device.open()
        def unhealthy(robot, state):
            if robot.side == "left" and self.robots["right"].sent:
                state["drivers"]["1"]["foc_status"]["driver_enable_status"] = False
        self.hook = unhealthy
        result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .049)

    def test_probe_exact_five_mm_closure_arrives_but_does_not_verify_grasp(self):
        self.device.open()
        result = self.device.execute_gripper_probe("right", .045)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["contact_observation"]["outcome"], "target_arrived")
        self.assertTrue(result["arrival_confirmed"])
        self.assertFalse(result["grasp_verified"])
        self.assertIsNone(self.device.unresolved_gripper_probe)
        self.assertTrue(self.device.execute("left", "gripper", .049)["ok"])

    def test_probe_larger_than_five_mm_rejects_before_send(self):
        self.device.open()
        result = self.device.execute_gripper_probe("right", .044999)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertTrue(all(not robot.sent for robot in self.robots.values()))

    def test_probe_opening_rejected_before_send(self):
        self.device.open()
        result = self.device.execute_gripper_probe("right", .051)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_probe_zero_delta_rejected_before_send(self):
        self.device.open()
        result = self.device.execute_gripper_probe("right", .05)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_probe_classification_delay_reads_current_feedback_again(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        def delayed(**kw):
            result = classifier(**kw)
            self.clock.sleep(.15)
            self.robots["left"].driver_enabled[0] = False
            return result
        with patch.object(pair_device, "classify_gripper_probe", side_effect=delayed):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("both CAN arms", result["errors"][-1]["detail"])

    def test_probe_completion_reads_after_slow_guard_without_resend(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        original_guard = self.device._action.guard
        pending_delay = [False]
        def classify(**kw):
            result = classifier(**kw)
            pending_delay[0] = True
            return result
        def guard():
            original_guard()
            if pending_delay[0]:
                self.clock.sleep(.15)
        self.device._action.guard = guard
        with patch.object(pair_device, "classify_gripper_probe", side_effect=classify):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual(result["completion_feedback_order"], "host_guard_then_read_then_validate")
        self.assertLessEqual(result["last_checked_feedback_age_s"], .1)

    def test_probe_final_completion_guard_still_honors_cancellation(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        completed = [False]
        calls = [0]
        original_guard = self.device._action.guard
        def classify(**kw):
            result = classifier(**kw)
            completed[0] = True
            return result
        def guard():
            original_guard()
            if completed[0]:
                calls[0] += 1
                self.clock.sleep(.15)
                if calls[0] == 2:
                    raise RuntimeError("operator cancellation at final completion")
        self.device._action.guard = guard
        with patch.object(pair_device, "classify_gripper_probe", side_effect=classify):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("operator cancellation", result["errors"][-1]["detail"])

    def test_probe_final_completion_checks_new_health_after_slow_guard(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        completed = [False]
        calls = [0]
        original_guard = self.device._action.guard
        def classify(**kw):
            result = classifier(**kw)
            completed[0] = True
            return result
        def guard():
            original_guard()
            if completed[0]:
                calls[0] += 1
                self.clock.sleep(.15)
                if calls[0] == 2:
                    self.robots["left"].driver_enabled[0] = False
        self.device._action.guard = guard
        with patch.object(pair_device, "classify_gripper_probe", side_effect=classify):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("both CAN arms", result["errors"][-1]["detail"])

    def test_probe_completion_still_rejects_actually_stale_feedback(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        original_read = self.device._action.read
        stale = [None]
        def classify(**kw):
            result = classifier(**kw)
            stale[0] = copy.deepcopy(kw["post_samples"][-1]["arms"])
            self.clock.sleep(.15)
            return result
        def read():
            return copy.deepcopy(stale[0]) if stale[0] is not None else original_read()
        self.device._action.read = read
        with patch.object(pair_device, "classify_gripper_probe", side_effect=classify):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("100 ms", result["errors"][-1]["detail"])

    def test_probe_slow_completion_guard_does_not_hide_new_health_fault(self):
        self.device.open()
        classifier = pair_device.classify_gripper_probe
        original_guard = self.device._action.guard
        pending_delay = [False]
        def classify(**kw):
            result = classifier(**kw)
            pending_delay[0] = True
            return result
        def guard():
            original_guard()
            if pending_delay[0]:
                pending_delay[0] = False
                self.clock.sleep(.15)
                self.robots["left"].driver_enabled[0] = False
        self.device._action.guard = guard
        with patch.object(pair_device, "classify_gripper_probe", side_effect=classify):
            result = self.device.execute_gripper_probe("right", .0455)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertIn("both CAN arms", result["errors"][-1]["detail"])

    def recovery_source(self):
        sample = self.device.open()
        from robot_tools.retention_receipt import measured_anchor
        return {"candidate_measurement": {"anchor": measured_anchor(sample["arms"]["right"]),
                    "observed": {"width_m": sample["arms"]["right"]["gripper"]["width_m"]}},
                "candidate_probe": {"requested_width_m": .046, "sent_at": self.clock.time()-10,
                                    "trace_sha256": "a"*64}, "before": copy.deepcopy(sample["arms"])}

    def test_supported_recovery_opens_once_and_confirms_without_grasp_or_joint_cache(self):
        source = self.recovery_source()
        result = self.device.recover_supported_gripper("right", .054, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x159])
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.device.grasp_states, {"left": None, "right": None})
        self.assertEqual(self.device._joint_cache, {"left": None, "right": None})
        confirmation = self.device.confirm_supported_recovery_release()
        self.assertTrue(confirmation["ok"])
        self.assertEqual(confirmation["hardware_commands_sent"], 0)
        self.assertGreaterEqual(confirmation["measurement"]["ended_at"]-confirmation["measurement"]["started_at"], 3)
        self.assertIsNone(confirmation["physical_stop_verified"])
        self.assertEqual(len(self.robots["right"].sent), 1)
        with self.assertRaises(RuntimeError):
            self.device.recover_supported_gripper("right", .055, source_receipt=source)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_supported_recovery_rejects_closing_without_tx(self):
        source = self.recovery_source()
        result = self.device.recover_supported_gripper("right", .049, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.robots["right"].sent, [])

    def test_supported_recovery_rejects_over_five_mm_without_tx(self):
        self.robots["right"].width = .040
        source = self.recovery_source()
        result = self.device.recover_supported_gripper("right", .046, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.robots["right"].sent, [])

    def test_supported_recovery_rejects_historical_body_mismatch_without_tx(self):
        source = self.recovery_source()
        source["candidate_measurement"]["anchor"]["pose_m_rad"][0] += .01
        with self.assertRaisesRegex(RuntimeError, "differs"):
            self.device.recover_supported_gripper("right", .054, source_receipt=source)
        self.assertEqual(self.robots["right"].sent, [])

    def test_supported_recovery_needs_its_own_opening_before_confirmation(self):
        self.device.open()
        with self.assertRaisesRegex(RuntimeError, "completed recovery opening"):
            self.device.confirm_supported_recovery_release()
        self.assertEqual(self.robots["right"].sent, [])

    def test_supported_recovery_open_failure_is_not_retried(self):
        source = self.recovery_source()
        self.robots["right"].fail_id = 0x159
        result = self.device.recover_supported_gripper("right", .054, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 1)
        self.assertEqual(result["transmission_counts"]["right"]["sent_frames"], 0)
        with self.assertRaises(RuntimeError):
            self.device.recover_supported_gripper("right", .055, source_receipt=source)
        self.assertLessEqual(len(self.robots["right"].sent), 1)

    def test_probe_final_frame_rechecks_five_mm_after_slow_host_guard(self):
        self.device.open()
        def expand(before):
            self.robots["right"].width = .0504
        self._delay_bus_guard(0, expand)
        result = self.device.execute_gripper_probe("right", .0453)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_explicit_release_opens_once_then_clears_pending_and_allows_next_action(self):
        self.candidate()
        self.hook = None
        self.robots["right"].accept = True
        result = self.device.release_gripper_probe("right", .05)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completion_mode"], "contact_probe_release")
        self.assertTrue(result["arrival_confirmed"])
        self.assertIsNone(result["object_release_verified"])
        self.assertGreater(result["actual_opening_increase_m"], .0005)
        self.assertIsNone(self.device.unresolved_gripper_probe)
        self.assertIsNone(result["physical_stop_verified"])
        self.assertFalse(result["contact_observation"]["target_cancellation_verified"])
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x159, 0x159])
        self.assertTrue(self.device.execute("left", "gripper", .049)["ok"])
        self.assertFalse(self.device.close()["requires_fault_latch"])

    def tracked_opening(self):
        self.candidate()
        bound = self.device.observe_grasp("right", identity=self.grasp_identity("right"),
                                          probe_event_id="probe-right")
        self.assertTrue(bound["ok"], bound)
        self.hook = None
        self.robots["right"].accept = True
        opened = self.device.release_gripper_probe("right", .05)
        self.assertTrue(opened["ok"], opened)
        self.assertEqual(self.device.grasp_states["right"]["status"], "release_opened")
        return opened

    def observe_opening(self):
        opening = self.device.grasp_states["right"]["release_opening"]
        return self.device.observe_release("right", identity=self.grasp_identity("right"),
            probe_event_id="probe-right", release_trace_sha256=opening["trace_sha256"])

    def finish_opening(self, observation, **changes):
        args = dict(identity=self.grasp_identity("right"), probe_event_id="probe-right",
            release_trace_sha256=observation["release_opening"]["trace_sha256"],
            confirmation_trace_sha256=observation["measurement"]["trace_sha256"])
        return self.device.finalize_release("right", **{**args, **changes})

    def test_bound_candidate_opens_repeatedly_then_zero_tx_confirms_without_rebasing_body(self):
        self.tracked_opening()
        original = copy.deepcopy(self.device.grasp_states["right"])
        peer = copy.deepcopy(self.device._action.idle_anchor["left"])
        self.assertTrue(self.device.observe()["stationary_observed"])
        for kind in ("move", "gripper"):
            with self.assertRaises(RuntimeError):
                self.device.execute("left", kind, self.target if kind == "move" else .049,
                                    operation="approach")
        with self.assertRaises(RuntimeError):
            self.device.execute_gripper_probe("left", .048)
        before_second = self.device.observe_grasp("right", identity=self.grasp_identity("right"),
                                                  probe_event_id="probe-right")
        self.assertTrue(before_second["ok"], before_second)
        second = self.device.release_gripper_probe("right", .054)
        self.assertTrue(second["ok"], second)
        current = self.device.grasp_states["right"]
        self.assertEqual(current["original_anchor"], original["original_anchor"])
        self.assertEqual(current["requested_width_m"], original["requested_width_m"])
        self.assertEqual(current["trace_sha256"], original["trace_sha256"])
        self.assertNotEqual(current["release_opening"]["trace_sha256"], original["release_opening"]["trace_sha256"])
        self.assertEqual(current["release_opening"]["finished_at"], second["release_measurement"]["ended_at"])
        self.assertEqual(current["release_opening"]["observed_width_m"], second["release_measurement"]["observed"]["width_m"])
        counts = self.device._action.totals()
        observation = self.observe_opening()
        self.assertTrue(observation["ok"], observation)
        self.assertGreater(observation["measurement"]["started_at"], current["release_opening"]["finished_at"])
        self.assertGreaterEqual(observation["measurement"]["ended_at"]-observation["measurement"]["started_at"], 3.)
        self.assertEqual(observation["measurement"]["anchor"], original["original_anchor"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        finalized = self.finish_opening(observation)
        self.assertTrue(finalized["ok"], finalized)
        self.assertEqual(finalized["hardware_commands_sent"], 0)
        self.assertEqual(finalized["target_calls_sent"], 0)
        self.assertIsNone(finalized["physical_stop_verified"])
        self.assertIsNone(finalized["object_release_verified"])
        self.assertIsNone(self.device.grasp_states["right"])
        self.assertEqual(self.device._action.totals(), counts)
        self.assertEqual(self.device._action.idle_anchor["left"], peer)
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x159]*3)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertFalse(self.device.close()["requires_fault_latch"])

    def test_release_opened_cannot_retain_or_rebase_cumulative_jaw_drift(self):
        self.tracked_opening()
        current = self.device.grasp_states["right"]
        with self.assertRaisesRegex(RuntimeError, "opened release"):
            self.device.retain_grasp("right", identity=self.grasp_identity("right"),
                probe_event_id="probe-right", probe_trace_sha256=current["trace_sha256"],
                deadline_at=self.clock.time()+60)
        self.robots["right"].width = .0503
        self.assertTrue(self.observe_opening()["ok"])
        self.assertEqual(self.device.grasp_states["right"]["release_opening"], current["release_opening"])
        self.robots["right"].width = .0506
        report = self.observe_opening()
        self.assertFalse(report["ok"])
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertIsNotNone(self.device.grasp_states["right"])

    def test_finalization_without_device_token_cannot_clear_record(self):
        self.tracked_opening()
        current = self.device.grasp_states["right"]
        result = self.device.finalize_release("right", identity=self.grasp_identity("right"),
            probe_event_id="probe-right", release_trace_sha256=current["release_opening"]["trace_sha256"],
            confirmation_trace_sha256="f"*64)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.device.grasp_states["right"], current)
        with self.assertRaises(RuntimeError):
            self.observe_opening()

    def test_old_confirmation_token_cannot_clear_new_opening(self):
        self.tracked_opening()
        observation = self.observe_opening()
        self.assertTrue(observation["ok"], observation)
        self.assertTrue(self.device.release_gripper_probe("right", .052)["ok"])
        result = self.finish_opening(observation)
        self.assertFalse(result["ok"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        self.assertEqual(len(self.robots["right"].sent), 3)

    def test_finalization_keeps_record_on_stale_confirmation(self):
        self.tracked_opening()
        observation = self.observe_opening()
        self.assertTrue(observation["ok"], observation)
        self.clock.sleep(.101)
        result = self.finish_opening(observation)
        self.assertFalse(result["ok"])
        self.assertIn("stale", result["error"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_finalization_keeps_record_on_peer_drift(self):
        self.tracked_opening()
        observation = self.observe_opening()
        self.assertTrue(observation["ok"], observation)
        self.robots["left"].motion.origin[0] += .000501
        result = self.finish_opening(observation)
        self.assertFalse(result["ok"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_finalization_identity_mismatch_faults_without_clearing_record(self):
        self.tracked_opening()
        observation = self.observe_opening()
        self.assertTrue(observation["ok"], observation)
        result = self.finish_opening(observation, identity={**self.grasp_identity("right"), "object_id": "other"})
        self.assertFalse(result["ok"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_second_release_slow_guard_cannot_rebase_opened_jaw_before_send(self):
        self.tracked_opening()
        self._delay_bus_guard(0, lambda before: setattr(self.robots["right"], "width", .0506))
        result = self.device.release_gripper_probe("right", .052)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(self.device.grasp_states["right"]["release_opening"]["target_width_m"], .05)
        self.assertEqual(len(self.robots["right"].sent), 2)

    def test_release_confirmation_journal_delay_rechecks_new_feedback(self):
        self.tracked_opening()
        original = self.device._action.journal
        def delayed(event, data):
            original(event, data)
            if event == "static_grasp_trace":
                self.clock.sleep(.15)
                self.robots["right"].width += .0006
        self.device._action.journal = delayed
        result = self.observe_opening()
        self.assertFalse(result["ok"])
        self.assertIsNone(self.device._release_confirmations["right"])
        self.assertIsNotNone(self.device.grasp_states["right"])
        self.assertEqual(result["hardware_commands_sent"], 0)

    def test_release_without_pending_and_wrong_arm_send_nothing(self):
        self.device.open()
        with self.assertRaises(RuntimeError):
            self.device.release_gripper_probe("right", .051)
        self.contact_response()
        self.assertTrue(self.device.execute_gripper_probe("right", .0455)["ok"])
        with self.assertRaises(RuntimeError):
            self.device.release_gripper_probe("left", .051)
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_release_no_opening_cannot_clear_pending_via_two_mm_arrival_tolerance(self):
        self.candidate()
        self.hook = None  # accept remains False; measured width stays .048.
        result = self.device.release_gripper_probe("right", .049)
        self.assertFalse(result["ok"])
        self.assertLess(result["actual_opening_increase_m"], .0005)
        self.assertIsNotNone(self.device.unresolved_gripper_probe)
        self.assertEqual(len(self.robots["right"].sent), 2)
        with self.assertRaises(RuntimeError):
            self.device.release_gripper_probe("right", .05)

    def test_release_indistinguishable_opening_cannot_clear_pending(self):
        self.candidate()
        def barely_open(robot, state):
            if robot.side == "right" and len(robot.sent) >= 2:
                robot.width = .0484
                state["gripper"]["width_m"] = .0484
        self.hook = barely_open
        result = self.device.release_gripper_probe("right", .049)
        self.assertFalse(result["ok"])
        self.assertLess(result["actual_opening_increase_m"], .0005)
        self.assertIsNotNone(self.device.unresolved_gripper_probe)
        self.assertEqual(len(self.robots["right"].sent), 2)

    def test_release_latest_feedback_must_still_show_distinguishable_opening(self):
        self.candidate()
        self.hook = None
        self.robots["right"].accept = True
        hasher = pair_device.hashlib.sha256
        def drift_after_settling(data):
            result = hasher(data)
            self.robots["right"].width = .0484
            return result
        with patch.object(pair_device.hashlib, "sha256", side_effect=drift_after_settling):
            result = self.device.release_gripper_probe("right", .0488)
        self.assertFalse(result["ok"])
        self.assertLess(result["actual_opening_increase_m"], .0005)
        self.assertIsNotNone(self.device.unresolved_gripper_probe)
        self.assertEqual(len(self.robots["right"].sent), 2)

    def test_release_more_than_five_mm_is_zero_tx_and_keeps_pending(self):
        self.candidate()
        result = self.device.release_gripper_probe("right", .053001)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNotNone(self.device.unresolved_gripper_probe)
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_release_closing_is_zero_tx_and_keeps_pending(self):
        self.candidate()
        result = self.device.release_gripper_probe("right", .047)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIsNotNone(self.device.unresolved_gripper_probe)

    def test_release_slow_guard_cannot_hide_pending_drift_before_actual_frame(self):
        self.candidate()
        self.hook = None
        self.robots["right"].accept = True
        self._delay_bus_guard(0, lambda before: setattr(self.robots["right"], "width", .0474))
        result = self.device.release_gripper_probe("right", .05)
        self.assertFalse(result["ok"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertIn("Unresolved probe jaw", result["errors"][-1]["detail"])
        self.assertIsNotNone(self.device.unresolved_gripper_probe)
        self.assertEqual(len(self.robots["right"].sent), 1)


class RealSDKPairDeviceTests(unittest.TestCase):
    def test_fault_feedback_real_sdk_parsers_and_fake_can_send_no_frames(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, created = Clock(), [], []
        factory, profile = sdk.AgxArmFactory.create_arm, copy.deepcopy(PROFILE)
        for config in profile["arms"].values():
            config["model"] = "piper"
        class FakeCAN:
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def send(self, frame, timeout=None):
                sent.append(copy.deepcopy(frame))
                raise AssertionError("RX diagnostics attempted CAN TX")
            def shutdown(self):
                pass
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            created.append(robot)
            return robot
        def baseline(robot, jaw):
            state = healthy_arm(clock.time())
            state["joints_rad"] = [0, .5, -.5, 0, 0, 0]
            state["pose_m_rad"] = robot.fk(state["joints_rad"])
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1, teach_status=0,
                motion_status=0, mode_feedback=0, arm_status=0, err_code=0))
            return state
        def deliver(robot, stamp, joint2=1234):
            # Feed actual manufacturer decoders deterministic fake RX frames;
            # the SDK connections still own FakeCAN buses with tracked TX.
            packets = {0x2A1: bytes((0, 4, 0, 0, 1, 0, 0, 0)),
                       0x2A2: struct.pack(">ii", 200000, 100000),
                       0x2A3: struct.pack(">ii", 300000, 0),
                       0x2A4: bytes(8), 0x2A5: struct.pack(">ii", 0, joint2),
                       0x2A6: struct.pack(">ii", -1000, 0), 0x2A7: bytes(8)}
            packets.update({0x260 + i: bytes((0, 240, 0, 20, 20, 0x10, 0, 0)) for i in range(1, 7)})
            for identifier, data in packets.items():
                robot._parser.parse_packet(can.Message(arbitration_id=identifier, data=data,
                                                       timestamp=stamp, is_extended_id=False))
            robot._effector._parser.parse_packet(can.Message(arbitration_id=0x2A8,
                data=struct.pack(">ihBB", 43000, 200, 0, 0), timestamp=stamp, is_extended_id=False))
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("No real sockets")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, supervised_actions, single_supervised_actions, linear_hold, pair_device):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN()))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            device = pair_device.GuardedPairDevice(profile, lambda *args: None)
            try:
                with patch.object(arms, "snapshot", side_effect=baseline):
                    device.open()
                with patch.object(device._action, "guard", side_effect=RuntimeError("latched fault")):
                    with self.assertRaises(RuntimeError):
                        device.observe()
                    clock.sleep(.02)
                    for robot in created:
                        deliver(robot, clock.time())
                    first = device.observe_fault_feedback()
                    self.assertEqual(first["read_errors"], {})
                    state = first["arms"]["left"]
                    self.assertAlmostEqual(state["joints_rad"][1], math.radians(1.234))
                    self.assertEqual(state["arm_status"]["arm_status"], 4)
                    self.assertFalse(state["drivers"]["1"]["foc_status"]["driver_enable_status"])
                    self.assertTrue(state["drivers"]["1"]["foc_status"]["collision_status"])
                    self.assertFalse(state["gripper"]["foc_status"]["driver_enable_status"])
                    clock.sleep(.02)
                    for robot in created:
                        deliver(robot, clock.time(), joint2=1300)
                    second = device.observe_fault_feedback()
                    self.assertEqual(second["diagnostics"]["left"]["fragments"]["joint_12"]["progress"], "advanced")
                    self.assertGreater(second["arms"]["left"]["joints_rad"][1], state["joints_rad"][1])
                    self.assertIsNone(second["physical_stop_verified"])
                    self.assertEqual(device._fault, "latched fault")
                self.assertEqual(sent, [])
                self.assertEqual(len(created), 2)
            finally:
                device.close()
            self.assertEqual(sent, [])

    def test_real_sdk_can_left_right_switch_without_reconnect_or_extra_frames(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, identities, created = Clock(), [], {}, []
        channels = [cfg["channel"] for cfg in PROFILE["arms"].values()]
        motions = {channel: Motion(clock) for channel in channels}
        widths = dict.fromkeys(channels, .05)
        standoffs = dict.fromkeys(channels, 0.0)
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                motions[self.channel].sent(frame)
                if frame.arbitration_id == 0x159:
                    widths[self.channel] = int.from_bytes(frame.data[:4], "big", signed=True)/1e6 + standoffs[self.channel]
            def shutdown(self):
                pass
        factory, profile = sdk.AgxArmFactory.create_arm, copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.0
            channel = config["comm"]["can"]["channel"]
            identities[id(robot)] = channel
            motions[channel].origin = robot.fk([0, .5, -.5, 0, 0, 0])
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            channel = identities[id(robot)]
            pose, motion = motions[channel].pose()
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1, teach_status=0,
                motion_status=motion, mode_feedback=motions[channel].mode, arm_status=0, err_code=0))
            state["pose_m_rad"], state["joints_rad"] = pose, [0, .5, -.5, 0, 0, 0]
            state["gripper"]["width_m"] = widths[channel]
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("No real sockets")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, supervised_actions, single_supervised_actions, linear_hold, pair_device):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile, lambda *args: None)
            try:
                device.open()
                self.assertEqual(sent, [])
                expected = []
                for side, kind in (("left", "move"), ("right", "gripper"), ("right", "move"), ("left", "gripper")):
                    channel = profile["arms"][side]["channel"]
                    target = motions[channel].origin[:]
                    target[2] += .006
                    result = device.execute(side, kind, target if kind == "move" else .02)
                    self.assertTrue(result["ok"], result)
                    expected.extend((channel, item) for item in ([0x151, 0x152, 0x153, 0x154] if kind == "move" else [0x159]))
                channel = profile["arms"]["left"]["channel"]
                standoffs[channel] = .0025
                probe = device.execute_gripper_probe("left", .0155)
                self.assertTrue(probe["ok"], probe)
                self.assertEqual(probe["contact_observation"]["outcome"], "settled_contact_candidate")
                self.assertEqual(probe["hardware_commands_sent"], 1)
                self.assertFalse(probe["arrival_confirmed"])
                before_retention = [(name, frame.arbitration_id, bytes(frame.data)) for name, frame in sent]
                identity = {"episode_id": "real-sdk-left", "arm": "left", "run_id": "run-1",
                            "owner": "owner-1", "epoch": "epoch-1", "object_id": "strip"}
                retention = device.retain_grasp("left", identity=identity, probe_event_id="probe-left",
                    probe_trace_sha256=probe["candidate_probe"]["trace_sha256"], deadline_at=clock.time()+60)
                self.assertTrue(retention["ok"], retention)
                self.assertEqual(retention["hardware_commands_sent"], 0)
                self.assertEqual([(name, frame.arbitration_id, bytes(frame.data)) for name, frame in sent], before_retention)
                self.assertEqual(device.grasp_states["left"]["status"], "retained_static")
                self.assertIsNone(device.unresolved_gripper_probe)
                standoffs[channel] = 0.0
                release = device.release_gripper_probe("left", .02)
                self.assertTrue(release["ok"], release)
                self.assertGreater(release["actual_opening_increase_m"], .0005)
                self.assertEqual(device.unresolved_gripper_probe["status"], "release_opened")
                opening = device.grasp_states["left"]["release_opening"]
                before_confirmation = len(sent)
                observed = device.observe_release("left", identity=identity, probe_event_id="probe-left",
                    release_trace_sha256=opening["trace_sha256"])
                self.assertTrue(observed["ok"], observed)
                finalized = device.finalize_release("left", identity=identity, probe_event_id="probe-left",
                    release_trace_sha256=opening["trace_sha256"],
                    confirmation_trace_sha256=observed["measurement"]["trace_sha256"])
                self.assertTrue(finalized["ok"], finalized)
                self.assertEqual(len(sent), before_confirmation)
                self.assertIsNone(device.unresolved_gripper_probe)
                expected.extend([(channel, 0x159), (channel, 0x159)])
                self.assertEqual([(channel, frame.arbitration_id) for channel, frame in sent], expected)
                self.assertEqual(len(created), 2)
            finally:
                device.close()


if __name__ == "__main__":
    unittest.main()
