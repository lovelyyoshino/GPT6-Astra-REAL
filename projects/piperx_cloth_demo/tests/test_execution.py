"""Offline execution state-machine tests; fake feedback is not robot simulation."""
import copy
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_tools import arms, execution


def healthy_arm(stamp):
    return {
        "status": "complete", "timestamp": stamp,
        "fragment_timestamps_s": {name: stamp for name in arms.PARTS + arms.DRIVERS + ("gripper",)},
        "pose_m_rad": [0.2, 0.1, 0.3, 0.0, 0.0, 0.0], "joints_rad": [0.0] * 6,
        "arm_status": {"ctrl_mode": 1, "arm_status": 0, "motion_status": 0,
                       "err_code": 0, "err_status": dict.fromkeys(arms.ARM_ERRORS, False)},
        "drivers": {str(i): {"foc_status": {**dict.fromkeys(arms.DRIVER_ERRORS, False),
                                             "driver_enable_status": True}} for i in range(1, 7)},
        "gripper": {"status": "complete", "timestamp": stamp, "mode": "width",
                    "width_m": 0.055, "force_N": 0.3,
                    "foc_status": {**dict.fromkeys(arms.GRIPPER_ERRORS, False),
                                   "driver_enable_status": True, "homing_status": True}},
    }


class Clock:
    def __init__(self):
        self.elapsed = 0.0

    def time(self):
        return 1_800_000_000.0 + self.elapsed

    def monotonic(self):
        return self.elapsed

    def sleep(self, duration):
        self.elapsed += duration


class FakeBackend:
    def __init__(self, clock):
        self.clock = clock
        self.state = {"arms": {side: healthy_arm(clock.time()) for side in ("left", "right")}}
        self.events, self.pending, self.accepted_targets = [], {}, {}
        self.sent = {"left": 0, "right": 0}
        self.settle_delays = {"left": 1, "right": 4}
        self.moves, self.grips, self.holds, self.closes = [], [], 0, 0
        self.connected = False
        self.commissioning = []
        self.never_reach = self.stale_after_dispatch = self.fault_after_dispatch = False
        self.fail_send_side, self.send_exception = None, None
        self.hold_response, self.hold_exception = {"all_stopped": False}, None
        self.cleanup_errors = []
        self.last_snapshot = None
        self.stale_source = None
        self.feedback_hook = None

    def commissioning_errors(self):
        return self.commissioning

    def connect(self):
        self.connected = True
        self.events.append(("connect",))

    def snapshot(self):
        self.clock.sleep(0.002)
        stamp = self.clock.time()
        for side, state in self.state["arms"].items():
            state["timestamp"] = stamp
            state["fragment_timestamps_s"] = dict.fromkeys(state["fragment_timestamps_s"], stamp)
            state["gripper"]["timestamp"] = stamp
            if side in self.pending:
                target, remaining = self.pending[side]
                if not self.never_reach:
                    remaining -= 1
                self.pending[side] = (target, remaining)
                if remaining <= 0:
                    if target.get("action", "move") == "gripper":
                        state["gripper"]["width_m"] = target["gripper_width_m"]
                    else:
                        state["pose_m_rad"] = target["pose_m_rad"][:]
                    state["arm_status"]["motion_status"] = 0
                    del self.pending[side]
        if self.fault_after_dispatch and any(self.sent.values()):
            self.state["arms"]["left"]["arm_status"].update(arm_status=4, motion_status=0, err_code=0)
        result = copy.deepcopy(self.state)
        if self.stale_after_dispatch and any(self.sent.values()):
            # Old 'reached' feedback with an apparently exact target is still old.
            result = copy.deepcopy(self.stale_source)
            for side, target in self.accepted_targets.items():
                result["arms"][side]["pose_m_rad"] = target["pose_m_rad"][:]
                result["arms"][side]["arm_status"]["motion_status"] = 0
        if self.feedback_hook is not None:
            self.feedback_hook(result)
        self.last_snapshot = copy.deepcopy(result)
        self.events.append(("snapshot", copy.deepcopy(result)))
        return result

    def _send(self, target, kind):
        side = target["arm"]
        if not any(self.sent.values()):
            self.stale_source = copy.deepcopy(self.last_snapshot)
        self.sent[side] += 1
        self.events.append((kind, side, copy.deepcopy(target)))
        if side == self.fail_send_side:
            raise self.send_exception or RuntimeError("SDK partial CAN write on " + side)
        self.accepted_targets[side] = copy.deepcopy(target)
        self.pending[side] = (copy.deepcopy(target), self.settle_delays[side])
        if kind == "move":
            self.state["arms"][side]["arm_status"]["motion_status"] = 1

    def move(self, target):
        self.moves.append(copy.deepcopy(target))
        self._send(target, "move")

    def grip(self, target):
        self.grips.append(copy.deepcopy(target))
        assert "gripper_force_N" in target
        self._send(target, "grip")

    @staticmethod
    def pose_error(actual, expected):
        return (math.sqrt(sum((a - b) ** 2 for a, b in zip(actual[:3], expected[:3]))),
                max(abs(a - b) for a, b in zip(actual[3:], expected[3:])))

    def request_hold_all(self):
        self.holds += 1
        self.events.append(("hold_request",))
        if self.hold_exception:
            raise self.hold_exception
        return self.hold_response

    def close(self):
        self.closes += 1
        self.events.append(("close",))
        return self.cleanup_errors

    def transmission_counts(self):
        return dict(self.sent)

    def emergency_stop(self):
        raise AssertionError("A timeout must not invoke the damped-descent emergency stop")

    reset = emergency_stop
    disable = emergency_stop


class FakeJournal:
    def __init__(self, events, fail_event=None):
        self.events, self.fail_event = events, fail_event

    def append(self, event, **data):
        if event == self.fail_event:
            raise OSError("journal write failed: " + event)
        self.events.append(("journal", event, copy.deepcopy(data)))


def move_target(side, x=0.21):
    return {"arm": side, "action": "move", "pose_m_rad": [x, 0.1, 0.3, 0.0, 0.0, 0.0],
            "mode": "p", "speed_percent": 5}


def gripper_target(side, width=0.03, force_N=0.4):
    return {"arm": side, "action": "gripper", "pose_m_rad": [0.2, 0.1, 0.3, 0.0, 0.0, 0.0],
            "gripper_width_m": width, "gripper_force_N": force_N}


def plan(stages=None):
    return {"stages": stages or [{"id": "paired_approach", "coordination": "paired",
                                  "targets": [move_target("left"), move_target("right")]}]}


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        for name in ("time", "monotonic", "sleep"):
            patcher = patch.object(execution.time, name, getattr(self.clock, name))
            patcher.start()
            self.addCleanup(patcher.stop)
        limits = patch.dict(execution.LIMITS, {"poll_s": 0.001, "stable_s": 0.004,
                                              "stage_timeout_s": 0.03, "total_timeout_s": 1.0})
        limits.start()
        self.addCleanup(limits.stop)
        self.backend = FakeBackend(self.clock)
        self.observation = {"state": copy.deepcopy(self.backend.state)}
        self.journal = FakeJournal(self.backend.events)

    def run_it(self, candidate=None, cancelled=lambda: False):
        return execution.run_plan(candidate or plan(), self.observation,
                                  self.backend, self.journal, cancelled)

    def test_normal_paired_barrier_precedes_next_gripper_stage(self):
        candidate = plan()
        targets = [{"arm": side, "action": "gripper", "pose_m_rad": move_target(side)["pose_m_rad"],
                    "gripper_width_m": 0.03, "gripper_force_N": 0.3} for side in ("left", "right")]
        candidate["stages"].append({"id": "paired_grip", "coordination": "paired", "targets": targets})
        result = self.run_it(candidate)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["completed_stages"], ["paired_approach", "paired_grip"])
        events = self.backend.events
        barrier = next(i for i, e in enumerate(events) if e[0:2] == ("journal", "targets_reached"))
        first_grip = next(i for i, e in enumerate(events) if e[0] == "grip")
        self.assertLess(barrier, first_grip)
        feedback = events[barrier][2]["feedback"]
        for side in ("left", "right"):
            self.assertEqual(feedback["arms"][side]["pose_m_rad"], move_target(side)["pose_m_rad"])
        self.assertEqual(result["transmissions"], {"left": 2, "right": 2})
        self.assertFalse(result["grasp_verified"])
        self.assertEqual(result["task_success"], "not_assessed")
        self.assertEqual(self.backend.holds, 0)
        self.assertEqual(self.backend.closes, 1)

    def test_sequential_targets_wait_before_dispatching_other_arm(self):
        candidate = plan()
        candidate["stages"][0]["coordination"] = "sequential"
        result = self.run_it(candidate)
        self.assertTrue(result["ok"], result)
        events = self.backend.events
        first_barrier = next(i for i, e in enumerate(events) if e[0:2] == ("journal", "targets_reached"))
        right_move = next(i for i, e in enumerate(events) if e[0:2] == ("move", "right"))
        self.assertLess(first_barrier, right_move)

    def test_initial_async_feedback_has_bounded_warmup(self):
        count = 0
        def warming(snapshot):
            nonlocal count
            count += 1
            if count <= 3:
                snapshot["arms"]["left"]["status"] = "partial"
                snapshot["arms"]["left"]["arm_status"] = None
        self.backend.feedback_hook = warming
        result = self.run_it()
        self.assertTrue(result["ok"], result)
        self.assertGreater(count, 3)

    def test_missing_initial_feedback_times_out_without_motion(self):
        def missing(snapshot):
            snapshot["arms"]["left"]["status"] = "partial"
            snapshot["arms"]["left"]["arm_status"] = None
        self.backend.feedback_hook = missing
        with patch.dict(execution.LIMITS, {"startup_timeout_s": 0.012}):
            result = self.run_it()
        self.assertEqual(result["status"], "rejected_before_motion")
        self.assertIn("Initial feedback timeout", result["error"])
        self.assertEqual(self.backend.moves, [])
        self.assertEqual(self.backend.holds, 0)

    def test_gripper_only_actions_preserve_pose_and_forward_force_units(self):
        candidate = plan([{"id": "grip", "coordination": "paired",
                           "targets": [gripper_target("left"), gripper_target("right", 0.025, 0.35)]}])
        result = self.run_it(candidate)
        self.assertTrue(result["ok"], result)
        self.assertEqual(self.backend.moves, [])
        self.assertEqual([t["gripper_force_N"] for t in self.backend.grips], [0.4, 0.35])
        self.assertEqual(result["transmissions"], {"left": 1, "right": 1})
        self.assertAlmostEqual(result["final_feedback"]["arms"]["right"]["gripper"]["width_m"], 0.025)
        self.assertEqual(result["final_feedback"]["arms"]["left"]["pose_m_rad"],
                         self.observation["state"]["arms"]["left"]["pose_m_rad"])
        self.assertFalse(result["grasp_verified"])

    def test_second_gripper_target_pose_mismatch_rejects_whole_group_before_tx(self):
        right = gripper_target("right")
        right["pose_m_rad"][0] += 0.03
        candidate = plan([{"id": "grip", "coordination": "paired",
                           "targets": [gripper_target("left"), right]}])
        result = self.run_it(candidate)
        self.assertEqual(result["status"], "rejected_before_motion")
        self.assertIn("Gripper-only target must match", result["error"])
        self.assertEqual(result["transmissions"], {"left": 0, "right": 0})
        self.assertEqual(self.backend.grips, [])
        self.assertEqual(self.backend.holds, 0)

    def test_status_only_updates_cannot_prove_stable_pose_feedback(self):
        self.backend.settle_delays = {"left": 1, "right": 1}
        first_stamps = {}
        def freeze_other_fragments(result):
            if not any(self.backend.sent.values()):
                return
            for side, state in result["arms"].items():
                first_stamps.setdefault(side, state["fragment_timestamps_s"].copy())
                for name, stamp in first_stamps[side].items():
                    if name != "arm_status":
                        state["fragment_timestamps_s"][name] = stamp
                state["gripper"]["timestamp"] = first_stamps[side]["gripper"]
        self.backend.feedback_hook = freeze_other_fragments
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertIn("Target timeout", result["error"])
        self.assertEqual(result["completed_stages"], [])
        for state in result["final_feedback"]["arms"].values():
            self.assertTrue(arms.control_health(state)["healthy"])
            self.assertEqual(state["arm_status"]["motion_status"], 0)
            self.assertGreater(state["fragment_timestamps_s"]["arm_status"],
                               state["fragment_timestamps_s"]["end_pose_xy"])

    def test_unexpected_gripper_change_during_active_move_aborts(self):
        def change_width(result):
            if any(self.backend.sent.values()):
                result["arms"]["left"]["gripper"]["width_m"] = 0.02
        self.backend.feedback_hook = change_width
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertIn("Gripper changed during arm motion", result["error"])
        self.assertEqual(self.backend.holds, 1)

    def test_same_flange_joint_drift_during_grip_aborts(self):
        def change_joint(result):
            if any(self.backend.sent.values()):
                result["arms"]["left"]["joints_rad"][3] = 0.1
        self.backend.feedback_hook = change_joint
        candidate = plan([{"id": "grip", "coordination": "paired",
                           "targets": [gripper_target("left"), gripper_target("right")]}])
        result = self.run_it(candidate)
        self.assertFalse(result["ok"])
        self.assertIn("joint configuration drifted", result["error"])
        self.assertEqual(result["final_feedback"]["arms"]["left"]["pose_m_rad"],
                         self.observation["state"]["arms"]["left"]["pose_m_rad"])
        self.assertEqual(self.backend.holds, 1)

    def test_inactive_arm_motion_flag_aborts_even_without_pose_drift(self):
        def idle_arm_moves(result):
            if any(self.backend.sent.values()):
                result["arms"]["right"]["arm_status"]["motion_status"] = 1
        self.backend.feedback_hook = idle_arm_moves
        candidate = plan([{"id": "left_only", "coordination": "sequential",
                           "targets": [move_target("left")]}])
        result = self.run_it(candidate)
        self.assertFalse(result["ok"])
        self.assertIn("Inactive arm started moving: right", result["error"])
        self.assertEqual(result["transmissions"], {"left": 1, "right": 0})
        self.assertEqual(self.backend.holds, 1)

    def test_old_cached_motion_zero_cannot_complete(self):
        self.backend.stale_after_dispatch = True
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertEqual(result["completed_stages"], [])
        self.assertIn("Target timeout", result["error"])
        self.assertEqual(self.backend.holds, 1)

    def test_controller_status_four_rejects_even_motion_zero(self):
        self.backend.fault_after_dispatch = True
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertIn("arm_fault_or_unknown", result["error"])
        self.assertEqual(self.backend.holds, 1)
        self.assertEqual(result["completed_stages"], [])

    def test_partial_second_arm_send_is_uncertain_and_holds_both(self):
        self.backend.fail_send_side = "right"
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertEqual(result["status"], "motion_state_unknown")
        self.assertEqual(result["transmissions"], {"left": 1, "right": 1})
        self.assertEqual(self.backend.holds, 1)
        self.assertFalse(result["physical_stop_verified"])

    def test_target_timeout_requests_hold_without_emergency_stop(self):
        self.backend.never_reach = True
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertIn("Target timeout", result["error"])
        self.assertEqual(self.backend.holds, 1)
        self.assertNotIn("hold_error", result)

    def test_ctrl_c_during_send_requests_hold_and_closes(self):
        self.backend.fail_send_side = "left"
        self.backend.send_exception = KeyboardInterrupt("operator interrupted")
        result = self.run_it()
        self.assertIn("KeyboardInterrupt", result["error"])
        self.assertEqual(self.backend.holds, 1)
        self.assertEqual(self.backend.closes, 1)
        self.assertFalse(result["physical_stop_verified"])

    def test_journal_intent_failure_sends_nothing(self):
        self.journal.fail_event = "dispatch_intent"
        result = self.run_it()
        self.assertEqual(result["status"], "rejected_before_motion")
        self.assertEqual(result["transmissions"], {"left": 0, "right": 0})
        self.assertEqual(self.backend.holds, 0)

    def test_journal_failure_after_send_requests_only_hold(self):
        self.journal.fail_event = "sdk_send_returned"
        result = self.run_it()
        self.assertIn("journal write failed", result["error"])
        self.assertEqual(result["transmissions"], {"left": 1, "right": 0})
        self.assertEqual(self.backend.holds, 1)
        self.assertNotIn("hold_error", result)

    def test_final_completion_journal_failure_cannot_return_ok(self):
        self.journal.fail_event = "plan_targets_reached"
        result = self.run_it()
        self.assertFalse(result["ok"])
        self.assertEqual(result["completed_stages"], ["paired_approach"])
        self.assertIn("journal write failed: plan_targets_reached", result["error"])
        self.assertNotEqual(result["status"], "targets_reached")
        self.assertEqual(self.backend.holds, 1)

    def test_startup_joint_drift_rejected_before_dispatch(self):
        self.backend.state["arms"]["left"]["joints_rad"][3] = 0.2
        result = self.run_it()
        self.assertIn("joint configuration drifted", result["error"])
        self.assertEqual(result["status"], "rejected_before_motion")
        self.assertEqual(result["transmissions"], {"left": 0, "right": 0})
        self.assertEqual(self.backend.holds, 0)

    def test_cancel_before_dispatch_sends_nothing(self):
        result = self.run_it(cancelled=lambda: True)
        self.assertIn("Cancellation requested", result["error"])
        self.assertEqual(result["transmissions"], {"left": 0, "right": 0})
        self.assertEqual(self.backend.holds, 0)

    def test_cancel_between_paired_sends_holds_partially_dispatched_group(self):
        result = self.run_it(cancelled=lambda: any(self.backend.sent.values()))
        self.assertIn("Cancellation requested", result["error"])
        self.assertEqual(result["transmissions"], {"left": 1, "right": 0})
        self.assertEqual(self.backend.holds, 1)

    def test_hold_failure_never_claims_physical_stop(self):
        self.backend.never_reach = True
        self.backend.hold_exception = RuntimeError("hold feedback unavailable")
        result = self.run_it()
        self.assertEqual(result["status"], "motion_state_unknown")
        self.assertFalse(result["physical_stop_verified"])
        self.assertIn("hold feedback unavailable", result["hold_error"])

    def test_verified_hold_is_distinct_from_request_returned(self):
        self.backend.never_reach = True
        self.backend.hold_response = {"all_stopped": True, "evidence": "fake verified feedback"}
        result = self.run_it()
        self.assertEqual(result["status"], "aborted_and_hold_verified")
        self.assertTrue(result["physical_stop_verified"])
        self.assertFalse(result["ok"])

    def test_commissioning_rejection_never_connects(self):
        self.backend.commissioning = [{"code": "hold_unverified"}]
        result = self.run_it()
        self.assertFalse(self.backend.connected)
        self.assertEqual(result["transmissions"], {"left": 0, "right": 0})
        self.assertEqual(result["status"], "rejected_before_motion")

    def test_exclusive_execution_lock_rejects_second_owner_then_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            with execution.ExclusiveExecution(Path(directory)):
                with self.assertRaises(execution.ExecutionFault):
                    with execution.ExclusiveExecution(Path(directory)):
                        self.fail("Second owner acquired the same two arms")
            with execution.ExclusiveExecution(Path(directory)):
                pass


if __name__ == "__main__":
    unittest.main()
