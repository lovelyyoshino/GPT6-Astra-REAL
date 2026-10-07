"""Pair-host contract tests use memory-only devices; never physical qualification."""
import copy
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robot_tools.pair_host import PairHost
from robot_tools.fault_feedback import FaultFeedback
from robot_tools.pair_ledger import platform_fault as ledger_platform_fault
from test_backend import PROFILE
from test_execution import Clock, healthy_arm


TASK = {
    "task_id": "plug_transfer_left",
    "roles": {"left": "task", "right": "task"},
    "site_context": {"workspace_clearance": {
        "source": "user", "statement": "current workspace clear"}},
}


class InjectableClock(Clock):
    def __init__(self):
        super().__init__()
        self.invalid = None

    def time(self):
        return super().time() if self.invalid is None else self.invalid


class FakePairDevice:
    """Trusted-adapter test double, with observable lifetime and failure injection."""

    capabilities = {
        "retained_target_stationary": True,
        "contact_support_verified": False,
        "contact_step_supported": False,
        "physical_stop_verified": False,
    }

    def __init__(self, profile, journal, guard, clock):
        self.profile, self.journal, self.guard, self.clock = profile, journal, guard, clock
        self.opens = self.closes = self.observations = 0
        self.calls = []
        self.observe_error = self.execute_error = None
        self.receipt_changes = {}
        self.execution_gate = None
        self.execution_started = threading.Event()
        self.open_gate = None
        self.open_started = threading.Event()
        self.before_frame = lambda: None
        self.frame_attempts = 0
        self.close_result = {"physical_stop_verified": None}
        self.states = {side: healthy_arm(clock.time()) for side in ("left", "right")}
        self.fault_tracker = FaultFeedback()
        self.fault_observations = 0
        self.fault_observed = threading.Event()
        self.fault_observe_error = None
        self.fault_observe_gate = None

    def open(self):
        self.opens += 1
        self.open_started.set()
        if self.open_gate is not None and not self.open_gate.wait(2):
            raise RuntimeError("Synthetic test opening gate timed out")
        return self.observe()

    def observe(self):
        self.observations += 1
        if self.observe_error is not None:
            raise self.observe_error
        self.clock.sleep(.002)
        stamp = self.clock.time()
        for state in self.states.values():
            state["timestamp"] = stamp
            state["fragment_timestamps_s"] = dict.fromkeys(state["fragment_timestamps_s"], stamp)
            state["gripper"]["timestamp"] = stamp
        return {"arms": copy.deepcopy(self.states), "stationary_observed": True,
                "baseline_duration_s": 3., "feedback_advances": 20}

    def execute(self, arm, kind, target):
        self.calls.append({"arm": arm, "kind": kind, "target": copy.deepcopy(target)})
        self.execution_started.set()
        if self.execution_gate is not None:
            if not self.execution_gate.wait(2):
                raise RuntimeError("Synthetic test execution gate timed out")
        self.before_frame()
        self.guard()
        self.frame_attempts += 1
        if self.execute_error is not None:
            raise self.execute_error
        if kind == "move":
            self.states[arm]["pose_m_rad"] = list(target)
        else:
            self.states[arm]["gripper"]["width_m"] = target
        return {"ok": True, "observed_stable": True, "controller_at_target": True,
                "feedback_all_after_send": True,
                "pose_error": {"position_m": 0., "rotation_rad": 0.},
                "width_error_m": 0., "hardware_commands_sent": 4 if kind == "move" else 1,
                "target_calls_sent": 1, "passive_arm_commands_sent": 0,
                **copy.deepcopy(self.receipt_changes)}

    def observe_fault_feedback(self):
        self.fault_observations += 1
        self.fault_observed.set()
        if self.fault_observe_gate is not None and not self.fault_observe_gate.wait(2):
            raise RuntimeError("Synthetic diagnostic gate timed out")
        if self.fault_observe_error is not None:
            raise self.fault_observe_error
        self.clock.sleep(.002)
        stamp = self.clock.time()
        for state in self.states.values():
            state["timestamp"] = stamp
            state["fragment_timestamps_s"] = dict.fromkeys(state["fragment_timestamps_s"], stamp)
        return self.fault_tracker.capture(self.states, stamp)

    def close(self):
        self.closes += 1
        return copy.deepcopy(self.close_result)


class FakeContactPairDevice(FakePairDevice):
    """Adapter receipts for host routing, never simulated contact dynamics."""

    capabilities = {**FakePairDevice.capabilities, "gripper_contact_observation": True}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        for state in self.states.values():
            state["gripper"]["width_m"] = .04
        self.unresolved_gripper_probe = None
        self.probe_outcome = "settled_contact_candidate"
        self.probe_receipt_changes = {}
        self.release_receipt_changes = {}
        self.release_clears_pending = True

    def _probe_send(self, arm, target, api):
        self.calls.append({"arm": arm, "kind": "gripper", "target": target, "api": api})
        self.execution_started.set()
        if self.execution_gate is not None and not self.execution_gate.wait(2):
            raise RuntimeError("Synthetic contact gate timed out")
        self.before_frame()
        self.guard()
        self.frame_attempts += 1
        if self.execute_error is not None:
            raise self.execute_error
        return {"ok": True, "observed_stable": True, "controller_at_target": True,
                "feedback_all_after_send": True,
                "pose_error": {"position_m": 0., "rotation_rad": 0.},
                "target_calls_sent": 1, "hardware_commands_sent": 1,
                "passive_arm_commands_sent": 0, "accepted": None,
                "grasp_verified": False, "physical_stop_verified": None}

    def execute_gripper_probe(self, arm, target):
        receipt = self._probe_send(arm, target, "execute_gripper_probe")
        arrived = self.probe_outcome == "target_arrived"
        actual_width = target if arrived else .039
        self.states[arm]["gripper"]["width_m"] = actual_width
        self.unresolved_gripper_probe = None if arrived else {
            "arm": arm, "requested_width_m": target, "observed_width_m": actual_width,
            "target_may_remain_active": True, "grasp_verified": False,
            "physical_stop_verified": None}
        return {**receipt, "completion_mode": "contact_probe", "arrival_confirmed": arrived,
                "width_error_m": abs(actual_width - target),
                "contact_observation": {"outcome": self.probe_outcome,
                                        "completion": "observation_only"},
                **copy.deepcopy(self.probe_receipt_changes)}

    def release_gripper_probe(self, arm, target):
        receipt = self._probe_send(arm, target, "release_gripper_probe")
        previous = self.states[arm]["gripper"]["width_m"]
        self.states[arm]["gripper"]["width_m"] = target
        if self.release_clears_pending:
            self.unresolved_gripper_probe = None
        return {**receipt, "completion_mode": "contact_probe_release",
                "arrival_confirmed": True, "width_error_m": 0.,
                "actual_opening_increase_m": target - previous,
                **copy.deepcopy(self.release_receipt_changes)}

    def close(self):
        receipt = super().close()
        return {**receipt, "requires_fault_latch": self.unresolved_gripper_probe is not None}


class PairHostTests(unittest.TestCase):
    def setUp(self):
        self.socket_guard = patch("socket.socket", side_effect=AssertionError("Real sockets forbidden"))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.runs = Path(self.directory.name)
        self.clock = InjectableClock()
        self.profile = copy.deepcopy(PROFILE)
        for arm in self.profile["arms"].values():
            arm["model"] = "piper"
        self.profile["cameras"] = {"front": "fake-front", "left_wrist": "fake-left",
                                   "right_wrist": "fake-right"}
        self.devices = []
        self.device_class = FakePairDevice
        self.configure_device = lambda device: None
        self.frame = 0
        self.host = self.make_host()

    def make_host(self, run_id="pair-test"):
        def factory(profile, journal, guard):
            device = self.device_class(profile, journal, guard, self.clock)
            self.configure_device(device)
            self.devices.append(device)
            return device
        host = PairHost(self.runs, self.profile, run_id, copy.deepcopy(TASK),
                        device_factory=factory, clock=self.clock.time, background=False)
        self.addCleanup(host.close)
        return host

    def rgb(self):
        self.frame += 1
        return {"capture_id": "fake-capture-" + str(self.frame), "cameras": {
            view: {"serial": self.profile["cameras"][key], "frame_number": self.frame,
                   "host_received_at": self.clock.time()}
            for view, key in (("front", "front"), ("left_hand", "left_wrist"),
                              ("right_hand", "right_wrist"))}}

    def observation(self):
        return self.host.observe(self.rgb())

    def arguments(self, observation, *, event="action-1", arm="left", kind="move",
                  operation="approach"):
        peer = "right" if arm == "left" else "left"
        return dict(event_id=event, observation_id=observation["observation_id"],
                    peer_receipt_id=observation["peer_receipts"][peer]["receipt_id"],
                    arm=arm, kind=kind,
                    target=[.2, .1, .306, 0., 0., 0.] if kind == "move" else .03,
                    operation=operation)

    def complete(self, arguments):
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "completed", result)
        return result

    def assert_pair_fault(self):
        status = self.host.status()
        self.assertTrue(status["fault_latched"], status)
        self.assertEqual(status["status"], "fault")

    def test_constructor_is_inert_and_both_actions_reuse_one_device_lifetime(self):
        self.assertEqual(sum(device.opens for device in self.devices), 0)
        self.host.open()
        self.complete(self.arguments(self.observation(), arm="left", event="left-move"))
        self.complete(self.arguments(self.observation(), arm="right", kind="gripper", event="right-jaw"))
        self.assertEqual(len(self.devices), 1)
        device = self.devices[0]
        self.assertEqual((device.opens, device.closes), (1, 0))
        self.assertEqual([(call["arm"], call["kind"]) for call in device.calls],
                         [("left", "move"), ("right", "gripper")])

    def test_direct_constructor_rejects_budget_bypass_before_creating_files(self):
        cases = [("max_steps", value) for value in (0, -1, 129, 500, 501, 10000, True, False, 1.0)]
        cases += [("max_duration_s", value) for value in
                  (0, -1, 901, 3600, 3601, 10**1000, True, False, float("nan"), float("inf"))]
        for index, (field, value) in enumerate(cases):
            path = self.runs / ("invalid-budget-" + str(index))
            with self.subTest(field=field, value=str(value)), self.assertRaises(ValueError):
                PairHost(path, self.profile, "invalid", copy.deepcopy(TASK),
                         **{field: value}, device_factory=lambda *args: self.fail("Device factory called"),
                         clock=self.clock.time, background=False)
            self.assertFalse(path.exists())

    def test_direct_constructor_accepts_positive_reduced_and_maximum_budgets(self):
        for index, (steps, duration) in enumerate(((1, .5), (128, 900))):
            host = PairHost(self.runs / ("valid-budget-" + str(index)), self.profile, "bounded",
                            copy.deepcopy(TASK), steps, duration,
                            device_factory=lambda *args: self.fail("Device factory called"),
                            clock=self.clock.time, background=False)
            self.addCleanup(host.close)
            self.assertEqual(host.status()["ledger"]["max_steps"], steps)
            self.assertEqual(host.status()["ledger"]["max_duration_s"], duration)

    def run_open_race(self, contender_method):
        opening = threading.Event()
        release = threading.Event()
        contender_started = threading.Event()
        contender_done = threading.Event()
        failures = []
        def configure(device):
            device.open_started, device.open_gate = opening, release
        self.configure_device = configure
        def invoke(method, contender=False):
            if contender:
                contender_started.set()
            try:
                method()
            except BaseException as exc:
                failures.append(exc)
            finally:
                if contender:
                    contender_done.set()
        opener = threading.Thread(target=invoke, args=(self.host.open,), daemon=True)
        contender = threading.Thread(target=invoke, args=(contender_method, True), daemon=True)
        opener.start()
        try:
            self.assertTrue(opening.wait(1))
            contender.start()
            self.assertTrue(contender_started.wait(1))
            self.assertFalse(contender_done.wait(.05), "Lifecycle transition overtook unfinished open")
            self.assertEqual(self.devices[0].closes, 0)
        finally:
            release.set()
            opener.join(2)
            if contender.ident is not None:
                contender.join(2)
        self.assertFalse(opener.is_alive())
        self.assertFalse(contender.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(self.devices), 1)
        self.assertEqual(self.devices[0].opens, 1)

    def test_close_waits_for_initial_open_and_closed_instance_cannot_reconnect(self):
        self.run_open_race(self.host.close)
        self.assertEqual(self.devices[0].closes, 1)
        self.assertFalse(self.host.status()["open"])
        with self.assertRaises(RuntimeError):
            self.host.open()
        self.assertEqual(len(self.devices), 1)
        self.assertEqual(self.devices[0].opens, 1)

    def test_concurrent_open_reuses_the_original_initialization(self):
        self.run_open_race(self.host.open)
        self.assertTrue(self.host.status()["open"])
        self.assertEqual(self.devices[0].closes, 0)

    def test_close_before_open_permanently_prevents_device_creation(self):
        self.host.close()
        with self.assertRaises(RuntimeError):
            self.host.open()
        self.assertEqual(self.devices, [])

    def test_caller_hold_assertion_cannot_replace_host_issued_peer_receipt(self):
        self.host.open()
        arguments = self.arguments(self.observation())
        arguments["peer_receipt_id"] = "caller-says-peer-held"
        with self.assertRaises(RuntimeError):
            self.host.submit(**arguments)
        with self.assertRaises(TypeError):
            self.host.submit(**arguments, peer_held=True)
        self.assertEqual(self.devices[0].calls, [])

    def test_old_receipt_cannot_admit_new_observation_or_wrong_peer(self):
        self.host.open()
        old = self.observation()
        current = self.observation()
        arguments = self.arguments(current)
        for receipt in (old["peer_receipts"]["right"]["receipt_id"],
                        current["peer_receipts"]["left"]["receipt_id"]):
            arguments["peer_receipt_id"] = receipt
            with self.subTest(receipt=receipt), self.assertRaises(RuntimeError):
                self.host.submit(**arguments)
        self.assertEqual(self.devices[0].calls, [])

    def test_higher_unseen_frame_from_before_action_cannot_be_the_next_scene(self):
        self.host.open()
        observation = self.observation()
        before_action = self.rgb()  # Higher sequence, never previously submitted to the host.
        self.complete(self.arguments(observation))
        with self.assertRaises(RuntimeError):
            self.host.observe(before_action)
        fresh = self.rgb()
        for camera in fresh["cameras"]:
            mixed = copy.deepcopy(fresh)
            mixed["cameras"][camera]["host_received_at"] = before_action["cameras"][camera]["host_received_at"]
            with self.subTest(stale_view=camera), self.assertRaises(RuntimeError):
                self.host.observe(mixed)
        accepted = self.host.observe(fresh)
        self.assertEqual(accepted["capture_id"], fresh["capture_id"])
        self.assertEqual(len(self.devices[0].calls), 1)

    def test_new_owner_requires_rgb_captured_after_its_own_open(self):
        self.host.open()
        old = self.rgb()
        self.host.close()
        self.clock.sleep(1.)
        self.host = self.make_host()
        self.host.open()
        with self.assertRaises(RuntimeError):
            self.host.observe(old)
        current = self.rgb()
        accepted = self.host.observe(current)
        self.assertEqual(accepted["capture_id"], current["capture_id"])
        self.assertEqual(sum(len(device.calls) for device in self.devices), 0)

    def test_expired_observation_cannot_send(self):
        self.host.open()
        arguments = self.arguments(self.observation())
        self.clock.sleep(30.)
        with self.assertRaises(RuntimeError):
            self.host.submit(**arguments)
        self.assertEqual(self.devices[0].calls, [])

    def test_new_owner_rejects_old_scene_and_peer_receipt_without_resetting_steps(self):
        self.host.open()
        self.complete(self.arguments(self.observation(), event="before-detach"))
        old = self.observation()
        old_arguments = self.arguments(old, event="old-owner-scene")
        self.host.close()
        self.host = self.make_host()
        self.host.open()
        current = self.observation()
        self.assertNotEqual(current["peer_receipts"]["right"]["owner"],
                            old["peer_receipts"]["right"]["owner"])
        self.assertEqual(self.host.status()["ledger"]["steps"], 1)
        with self.assertRaises(RuntimeError):
            self.host.submit(**old_arguments)
        mixed = self.arguments(current, event="old-owner-receipt")
        mixed["peer_receipt_id"] = old["peer_receipts"]["right"]["receipt_id"]
        with self.assertRaises(RuntimeError):
            self.host.submit(**mixed)
        self.assertEqual(self.devices[-1].calls, [])

    def test_same_event_replay_does_not_resend_even_after_observation_expires(self):
        self.host.open()
        arguments = self.arguments(self.observation())
        self.complete(arguments)
        self.clock.sleep(30.)
        replay = self.host.submit(**arguments)
        self.assertEqual(replay["status"], "completed")
        self.assertTrue(replay["replayed"])
        self.assertEqual(len(self.devices[0].calls), 1)
        changed = copy.deepcopy(arguments)
        changed["target"][2] += .001
        with self.assertRaises(RuntimeError):
            self.host.submit(**changed)
        self.assertEqual(len(self.devices[0].calls), 1)

    def test_completed_event_replays_original_receipt_after_clean_owner_restart(self):
        self.host.open()
        arguments = self.arguments(self.observation())
        completed = self.complete(arguments)
        previous_owner = self.host.status()["owner"]
        self.host.close()
        self.clock.sleep(31.)
        self.host = self.make_host()
        self.host.open()
        self.assertNotEqual(self.host.status()["owner"], previous_owner)
        replay = self.host.submit(**arguments)
        self.assertEqual(replay["status"], "completed")
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], completed["receipt"])
        self.assertEqual(self.devices[-1].calls, [])
        self.assertEqual(sum(len(device.calls) for device in self.devices), 1)
        self.assertEqual(self.host.status()["ledger"]["steps"], 1)

    def test_rgb_expiry_inside_device_blocks_its_next_frame(self):
        self.host.open()
        device = self.devices[0]
        device.before_frame = lambda: self.clock.sleep(31.)
        arguments = self.arguments(self.observation())
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        self.assertEqual(len(device.calls), 1)
        self.assertEqual(device.frame_attempts, 0)

    def io_change_cannot_authorize_frame(self, change):
        self.host.open()
        device = self.devices[0]
        at_frame = threading.Event()
        changed = threading.Event()
        device.before_frame = at_frame.set
        def slow_fault_read(path, *, run_id=None):
            if at_frame.is_set() and not changed.is_set():
                changed.set()
                change()
                # Emulate a read-only SQLite snapshot from before the change.
                return None
            return ledger_platform_fault(path, run_id=run_id)
        arguments = self.arguments(self.observation())
        with patch("robot_tools.pair_host.platform_fault", side_effect=slow_fault_read):
            self.host.submit(**arguments)
            result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertTrue(changed.is_set())
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        self.assertEqual(len(device.calls), 1)
        self.assertEqual(device.frame_attempts, 0)

    def test_guard_rechecks_rgb_expiry_after_persistent_fault_read(self):
        self.io_change_cannot_authorize_frame(lambda: self.clock.sleep(31.))

    def test_guard_rechecks_total_budget_after_persistent_fault_read(self):
        self.io_change_cannot_authorize_frame(lambda: self.clock.sleep(901.))

    def test_guard_rechecks_cancel_after_persistent_fault_read(self):
        self.io_change_cannot_authorize_frame(lambda: self.host.cancel("Synthetic cancel during SQLite read"))

    def invalid_clock_cannot_emit_frame(self, value):
        self.host.open()
        device = self.devices[0]
        device.before_frame = lambda: setattr(self.clock, "invalid", value)
        arguments = self.arguments(self.observation())
        self.host.submit(**arguments)
        try:
            result = self.host.wait(arguments["event_id"], timeout=5)
        finally:
            self.clock.invalid = None
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        self.assertEqual(device.frame_attempts, 0)

    def test_nonfinite_clock_cannot_pass_frame_guard(self):
        self.invalid_clock_cannot_emit_frame(float("nan"))

    def test_boolean_clock_cannot_pass_frame_guard(self):
        self.invalid_clock_cannot_emit_frame(False)

    def test_pending_duplicate_keeps_one_worker_and_prevents_peer_dispatch(self):
        self.host.open()
        device = self.devices[0]
        device.execution_gate = threading.Event()
        observation = self.observation()
        arguments = self.arguments(observation)
        self.host.submit(**arguments)
        try:
            self.assertTrue(device.execution_started.wait(1))
            replay = self.host.submit(**arguments)
            self.assertEqual(replay["status"], "pending")
            self.assertTrue(replay["replayed"])
            with self.assertRaises(RuntimeError):
                self.host.submit(**self.arguments(observation, event="peer-pending", arm="right"))
            self.assertEqual(len(device.calls), 1)
        finally:
            device.execution_gate.set()
            result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(len(device.calls), 1)

    def test_cancel_pending_latches_dispatch_without_claiming_stop(self):
        self.host.open()
        device = self.devices[0]
        device.execution_gate = threading.Event()
        arguments = self.arguments(self.observation())
        self.host.submit(**arguments)
        try:
            self.assertTrue(device.execution_started.wait(1))
            cancellation = self.host.cancel("Synthetic operator cancellation")
            self.assertIsNone(cancellation["physical_stop_verified"])
            self.assertTrue(cancellation["software_cancelled"])
            self.assertEqual(self.host.poll()["fault_feedback_read_state"], "deferred_active_action")
            self.assertEqual(device.fault_observations, 0)
        finally:
            device.execution_gate.set()
            result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault")
        self.assert_pair_fault()
        self.assertEqual(len(device.calls), 1)
        self.assertEqual(self.host.poll()["fault_feedback"]["status"], "observed")
        self.assertEqual(device.fault_observations, 1)
        self.assertEqual(device.frame_attempts, 0)

    def test_cancel_then_repeated_new_feedback_without_scene_or_fault_rewrites(self):
        self.host.open()
        observation = self.observation()
        device = self.devices[0]
        with patch.object(self.host.ledger, "fault", wraps=self.host.ledger.fault) as latch:
            self.host.cancel("user cancellation")
            device.states["left"]["drivers"]["1"]["foc_status"]["driver_enable_status"] = False
            device.states["left"]["arm_status"]["arm_status"] = 4
            previous = observation["sample"]["arms"]["left"]["timestamp"]
            for _ in range(3):
                status = self.host.poll()
                feedback = status["fault_feedback"]
                self.assertGreater(feedback["arms"]["left"]["timestamp"], previous)
                previous = feedback["arms"]["left"]["timestamp"]
                self.assertEqual(feedback["arms"]["left"]["arm_status"]["arm_status"], 4)
                self.assertIsNone(feedback["physical_stop_verified"])
                self.assertTrue(status["fault_latched"])
            self.host.cancel("second request must not rewrite the first fault")
            self.assertEqual(latch.call_count, 1)
        self.assertEqual(self.host.sequence, 1)
        self.assertEqual(device.calls, [])
        self.assertEqual(device.frame_attempts, 0)
        with self.assertRaises(RuntimeError):
            self.host.observe(self.rgb())
        state = self.host.read_state()
        self.assertEqual(state["state"]["status"], "diagnostic")
        self.assertFalse(state["motion_permitted"])
        self.assertTrue(state["fault_latched"])
        self.assertGreater(state["state"]["arms"]["left"]["timestamp"], previous)

    def test_fault_monitor_continues_and_close_prevents_further_reads(self):
        self.host.background = True
        self.host.open()
        device = self.devices[0]
        self.host.cancel()
        self.assertTrue(device.fault_observed.wait(1))
        self.host.close()
        self.assertFalse(self.host.monitor.is_alive())
        count = device.fault_observations
        self.assertEqual(self.host.poll()["fault_feedback_read_state"], "closed")
        self.assertEqual(device.fault_observations, count)
        self.assertEqual(device.closes, 1)
        self.assertEqual(device.calls, [])

    def test_fault_read_state_marks_cached_feedback_after_close(self):
        self.host.open()
        self.host.cancel()
        first = self.host.read_state()
        self.assertTrue(first["ok"])
        self.assertFalse(first["cached"])
        self.host.close()
        cached = self.host.read_state()
        self.assertFalse(cached["ok"])
        self.assertFalse(cached["open"])
        self.assertTrue(cached["cached"])
        self.assertEqual(cached["fault_feedback_read_state"], "closed")
        self.assertEqual(cached["fault_feedback"], first["fault_feedback"])

    def test_fault_read_state_continues_while_pending_action_only_blocks_on_bookkeeping(self):
        self.host.open()
        self.host.cancel()
        first = self.host.read_state()
        self.host.active_event_id = "synthetic-active-worker"
        try:
            observed = self.host.read_state()
            self.assertTrue(observed["ok"])
            self.assertFalse(observed["cached"])
            self.assertEqual(observed["fault_feedback_read_state"], "observed")
            self.assertGreater(observed["fault_feedback"]["sequence"], first["fault_feedback"]["sequence"])
            self.assertEqual(observed["fault_feedback"]["action_event_pending"], "synthetic-active-worker")
            self.assertFalse(observed["motion_permitted"])
            self.assertEqual(self.devices[0].frame_attempts, 0)
        finally:
            self.host.active_event_id = None

    def test_pending_fault_read_error_is_reported_as_attempted_read(self):
        self.host.open()
        self.host.cancel()
        self.host.active_event_id = "pending-bookkeeping"
        self.devices[0].fault_observe_error = RuntimeError("Synthetic RX error")
        try:
            observed = self.host.read_state()
            self.assertFalse(observed["ok"])
            self.assertEqual(observed["fault_feedback_read_state"], "read_error")
            self.assertEqual(observed["fault_feedback"]["action_event_pending"], "pending-bookkeeping")
            self.assertEqual(self.devices[0].frame_attempts, 0)
        finally:
            self.host.active_event_id = None

    def test_fault_read_state_busy_device_does_not_claim_another_read(self):
        self.host.open()
        self.host.cancel()
        first = self.host.read_state()
        entered, finish = threading.Event(), threading.Event()
        def occupy():
            with self.host.device_lock:
                entered.set()
                finish.wait(2)
        worker = threading.Thread(target=occupy)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            cached = self.host.read_state()
            self.assertFalse(cached["ok"])
            self.assertEqual(cached["fault_feedback_read_state"], "cached_no_new_read")
            self.assertEqual(cached["fault_feedback"], first["fault_feedback"])
        finally:
            finish.set()
            worker.join(2)

    def test_fault_reader_and_journal_failures_keep_diagnostics_and_do_not_retry_fault(self):
        self.host.open()
        device = self.devices[0]
        with patch.object(self.host.ledger, "fault", wraps=self.host.ledger.fault) as latch:
            self.host.cancel()
            device.fault_observe_error = ValueError("broken parser cache")
            first = self.host.poll()
            self.assertIn("broken parser cache", first["fault_feedback"]["error"])
            device.fault_observe_error = None
            with patch.object(self.host.journal, "append", side_effect=OSError("disk full")):
                second = self.host.poll()
            self.assertEqual(second["fault_feedback"]["status"], "observed")
            self.assertIn("disk full", second["fault_feedback_journal_error"])
            self.assertEqual(latch.call_count, 1)
        self.assertEqual(device.calls, [])

    def test_fault_poll_and_status_use_readonly_ledger_even_for_bad_clock(self):
        self.host.open()
        self.host.cancel()
        self.clock.invalid = float("nan")
        with patch.object(self.host.ledger, "status", side_effect=AssertionError("mutating status after fault")), \
             patch.object(self.host.ledger, "fault", side_effect=AssertionError("repeated fault write")):
            for _ in range(3):
                status = self.host.poll()
                self.assertTrue(status["fault_latched"])
                self.assertFalse(status["fault_feedback"]["observation_clock_valid"])
                self.assertIsNone(status["physical_stop_verified"])
        self.clock.invalid = None

    def test_close_serializes_with_fault_read_without_state_device_deadlock(self):
        self.host.open()
        self.host.cancel()
        device = self.devices[0]
        device.fault_observe_gate = threading.Event()
        errors = []
        def run(call):
            try:
                call()
            except Exception as exc:
                errors.append(exc)
        poller = threading.Thread(target=lambda: run(self.host.poll))
        closer = threading.Thread(target=lambda: run(self.host.close))
        poller.start()
        try:
            self.assertTrue(device.fault_observed.wait(1))
            closer.start()
            self.assertTrue(self.host.quit_event.wait(1))
            self.assertIsNone(self.host.cancel()["physical_stop_verified"])
        finally:
            device.fault_observe_gate.set()
            poller.join(2)
            if closer.ident is not None:
                closer.join(2)
        self.assertFalse(poller.is_alive())
        self.assertFalse(closer.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(device.closes, 1)

    def test_contact_declarations_cannot_override_adapter_capability(self):
        self.profile["contact_step_supported"] = True
        self.profile["verification"] = {"contact_support_verified": True}
        self.host = self.make_host(run_id="contact-declarations")
        self.host.open()
        for operation in ("grip_supported", "extract_segment", "insert_segment"):
            arguments = self.arguments(self.observation(), event=operation, operation=operation,
                                       kind="gripper" if operation == "grip_supported" else "move")
            with self.subTest(operation=operation), self.assertRaises(RuntimeError):
                self.host.submit(**arguments)
            with self.subTest(caller_flag=operation), self.assertRaises(TypeError):
                self.host.submit(**arguments, contact_step_supported=True)
        self.assertEqual(self.devices[0].calls, [])

    def open_contact_host(self):
        self.device_class = FakeContactPairDevice
        self.host.open()
        return self.devices[-1]

    def probe_arguments(self, *, event="contact-1", arm="left"):
        return {**self.arguments(self.observation(), event=event, arm=arm,
                                 kind="gripper", operation="grip_supported"), "target": .0355}

    def release_arguments(self, *, event="release-1", arm="left"):
        return {**self.arguments(self.observation(), event=event, arm=arm,
                                 kind="gripper", operation="release_retreat"), "target": .043}

    def test_contact_candidate_uses_probe_api_without_claiming_arrival_or_grasp(self):
        device = self.open_contact_host()
        arguments = self.probe_arguments()
        result = self.complete(arguments)
        receipt = result["receipt"]
        self.assertEqual([call["api"] for call in device.calls], ["execute_gripper_probe"])
        self.assertEqual(receipt["completion_mode"], "contact_probe")
        self.assertEqual(receipt["contact_observation"], {
            "outcome": "settled_contact_candidate", "completion": "observation_only"})
        self.assertGreater(receipt["width_error_m"], .002)
        self.assertFalse(receipt["arrival_confirmed"])
        self.assertFalse(receipt["grasp_verified"])
        self.assertIsNone(receipt["accepted"])
        self.assertIsNone(receipt["object_task_success"])
        self.assertIsNone(receipt["physical_stop_verified"])
        self.assertEqual(device.unresolved_gripper_probe["arm"], "left")
        replay = self.host.submit(**arguments)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], receipt)
        self.assertEqual(device.frame_attempts, 1)
        self.assertFalse(self.host.status()["fault_latched"])

    def test_arrived_contact_probe_retains_no_residual_target_and_allows_next_action(self):
        device = self.open_contact_host()
        device.probe_outcome = "target_arrived"
        receipt = self.complete(self.probe_arguments())["receipt"]
        self.assertTrue(receipt["arrival_confirmed"])
        self.assertEqual(receipt["width_error_m"], 0.)
        self.assertFalse(receipt["grasp_verified"])
        self.assertIsNone(device.unresolved_gripper_probe)
        self.complete(self.arguments(self.observation(), event="after-arrived-probe", arm="right"))
        self.assertEqual(len(device.calls), 2)
        self.assertFalse(self.host.close()["fault_latched"])

    def test_contact_probe_requires_positive_closing_step_at_most_five_mm(self):
        device = self.open_contact_host()
        for index, target in enumerate((.04, .041, .034, 0.)):
            arguments = {**self.probe_arguments(event="invalid-probe-" + str(index)), "target": target}
            with self.subTest(target=target), self.assertRaises(RuntimeError):
                self.host.submit(**arguments)
        self.assertEqual(device.calls, [])
        self.assertFalse(self.host.status()["fault_latched"])

    def test_contact_probe_step_bound_uses_current_feedback_not_scene_width(self):
        device = self.open_contact_host()
        arguments = self.probe_arguments()
        # The new adapter snapshot sees >5mm closure even though the scene saw 4.5mm.
        device.states["left"]["gripper"]["width_m"] = .045
        with self.assertRaises(RuntimeError):
            self.host.submit(**arguments)
        self.assertEqual(device.calls, [])

    def test_gripper_observation_capability_does_not_enable_loaded_contact_actions(self):
        device = self.open_contact_host()
        operations = ("extract_segment", "insert_segment", "grip_test", "rotate_segment",
                      "push_segment", "wipe_segment", "sweep_segment", "grip_supported")
        for operation in operations:
            arguments = self.arguments(self.observation(), event=operation, operation=operation)
            with self.subTest(operation=operation), self.assertRaises(RuntimeError):
                self.host.submit(**arguments)
        self.assertEqual(device.calls, [])
        self.assertFalse(self.host.capabilities["contact_support_verified"])
        self.assertFalse(self.host.capabilities["contact_step_supported"])

    def test_unresolved_probe_blocks_both_arms_until_explicit_same_jaw_release(self):
        device = self.open_contact_host()
        self.complete(self.probe_arguments())
        cases = (("left", "move", "approach", [.2, .1, .306, 0., 0., 0.]),
                 ("right", "move", "transport", [.2, .1, .306, 0., 0., 0.]),
                 ("right", "gripper", "release_retreat", .043),
                 ("left", "gripper", "approach", .043),
                 ("left", "gripper", "grip_supported", .0355),
                 ("left", "gripper", "release_retreat", .039),
                 ("left", "gripper", "release_retreat", .038),
                 ("left", "gripper", "release_retreat", .045))
        for index, (arm, kind, operation, target) in enumerate(cases):
            arguments = {**self.arguments(self.observation(), event="blocked-" + str(index),
                                          arm=arm, kind=kind, operation=operation), "target": target}
            with self.subTest(arm=arm, kind=kind, operation=operation, target=target), \
                    self.assertRaises(RuntimeError):
                self.host.submit(**arguments)
        self.assertEqual(device.frame_attempts, 1)
        result = self.complete(self.release_arguments())
        self.assertEqual(device.calls[-1]["api"], "release_gripper_probe")
        self.assertEqual(result["receipt"]["completion_mode"], "contact_probe_release")
        self.assertGreater(result["receipt"]["actual_opening_increase_m"], .0005)
        self.assertIsNone(device.unresolved_gripper_probe)
        self.complete(self.arguments(self.observation(), event="after-release", arm="right"))
        self.assertEqual(device.frame_attempts, 3)

    def test_probe_release_requires_new_scene_and_peer_receipt(self):
        device = self.open_contact_host()
        arguments = self.probe_arguments()
        self.complete(arguments)
        old_scene_release = {**arguments, "event_id": "stale-release", "target": .043,
                             "operation": "release_retreat"}
        with self.assertRaises(RuntimeError):
            self.host.submit(**old_scene_release)
        self.assertEqual(device.frame_attempts, 1)
        self.complete(self.release_arguments())
        self.assertEqual(device.frame_attempts, 2)

    def test_probe_receipt_cannot_upgrade_candidate_to_acknowledgement_grasp_or_stop(self):
        cases = ({"accepted": True}, {"grasp_verified": True}, {"physical_stop_verified": True},
                 {"arrival_confirmed": True}, {"controller_at_target": False},
                 {"feedback_all_after_send": False}, {"width_error_m": .001},
                 {"completion_mode": "position"},
                 {"contact_observation": {"outcome": "unconfirmed", "completion": "observation_only"}})
        base = self.runs
        for index, changes in enumerate(cases):
            with self.subTest(changes=changes):
                self.runs = base / ("invalid-probe-receipt-" + str(index))
                self.host = self.make_host()
                device = self.open_contact_host()
                device.probe_receipt_changes = changes
                arguments = self.probe_arguments()
                self.host.submit(**arguments)
                result = self.host.wait(arguments["event_id"], timeout=5)
                self.assertEqual(result["status"], "fault", result)
                self.assert_pair_fault()
                self.assertEqual(device.frame_attempts, 1)

    def test_release_must_show_actual_opening_arrival_and_clear_residual_target(self):
        cases = ({"actual_opening_increase_m": 0.}, {"actual_opening_increase_m": .0005},
                 {"actual_opening_increase_m": float("nan")}, {"actual_opening_increase_m": True},
                 {"arrival_confirmed": False}, {"width_error_m": .004},
                 {"feedback_all_after_send": False}, {"observed_stable": False},
                 {"completion_mode": "position"}, {"keep_pending": True})
        base = self.runs
        for index, changes in enumerate(cases):
            with self.subTest(changes=changes):
                self.runs = base / ("invalid-release-receipt-" + str(index))
                self.host = self.make_host()
                device = self.open_contact_host()
                self.complete(self.probe_arguments())
                device.release_receipt_changes = {k: v for k, v in changes.items() if k != "keep_pending"}
                device.release_clears_pending = not changes.get("keep_pending", False)
                arguments = self.release_arguments()
                self.host.submit(**arguments)
                result = self.host.wait(arguments["event_id"], timeout=5)
                self.assertEqual(result["status"], "fault", result)
                self.assert_pair_fault()
                self.assertEqual(device.frame_attempts, 2)

    def test_ordinary_gripper_stability_with_width_error_still_faults(self):
        device = self.open_contact_host()
        device.receipt_changes = {"width_error_m": .004, "arrival_confirmed": False}
        arguments = self.arguments(self.observation(), kind="gripper", operation="approach")
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        self.assertNotIn("api", device.calls[0])

    def test_close_with_unresolved_probe_persists_fault_instead_of_clean_detach(self):
        device = self.open_contact_host()
        self.complete(self.probe_arguments())
        report = self.host.close()
        self.assertTrue(report["fault_latched"])
        self.assertIsNone(report["physical_stop_verified"])
        self.assert_pair_fault()
        self.assertNotEqual(self.host.status()["ledger"]["status"], "detached")
        self.assertTrue(ledger_platform_fault(self.runs / "pair_sessions.sqlite"))
        self.assertEqual(device.frame_attempts, 1)
        self.assertEqual(device.closes, 1)

    def test_device_close_fault_requirement_is_honored_without_visible_pending_probe(self):
        self.host.open()
        device = self.devices[0]
        device.close_result = {"requires_fault_latch": True, "physical_stop_verified": None}
        report = self.host.close()
        self.assertTrue(report["fault_latched"])
        self.assert_pair_fault()
        self.assertNotEqual(self.host.status()["ledger"]["status"], "detached")
        self.assertEqual(device.frame_attempts, 0)
        self.assertEqual(device.closes, 1)

    def test_baseline_failure_latches_pair_before_any_dispatch(self):
        def broken(device):
            device.observe_error = RuntimeError("Synthetic baseline drift on left arm")
        self.configure_device = broken
        for device in self.devices:
            broken(device)
        with self.assertRaises(RuntimeError):
            self.host.open()
        self.assert_pair_fault()
        self.assertEqual(self.devices[0].calls, [])

    def test_idle_monitor_fault_blocks_actions_on_either_arm(self):
        self.host.open()
        observation = self.observation()
        self.devices[0].observe_error = RuntimeError("Synthetic right feedback fault")
        try:
            self.host.poll()
        except RuntimeError:
            pass
        self.assert_pair_fault()
        for arm in ("left", "right"):
            with self.subTest(arm=arm), self.assertRaises(RuntimeError):
                self.host.submit(**self.arguments(observation, arm=arm, event="after-fault-" + arm))
        self.assertEqual(self.devices[0].calls, [])

    def test_failed_shared_scene_feedback_latches_pair_without_dispatch(self):
        self.host.open()
        self.devices[0].observe_error = RuntimeError("Synthetic feedback fault during shared scene")
        with self.assertRaises(RuntimeError):
            self.observation()
        self.assert_pair_fault()
        self.assertEqual(self.devices[0].calls, [])

    def test_preclaim_peer_recheck_fault_latches_pair_without_dispatch(self):
        self.host.open()
        arguments = self.arguments(self.observation())
        self.devices[0].observe_error = RuntimeError("Synthetic peer drift immediately before claim")
        with self.assertRaises(RuntimeError):
            self.host.submit(**arguments)
        self.assert_pair_fault()
        self.assertEqual(self.devices[0].calls, [])

    def test_uncertain_dispatch_is_claimed_once_and_latches_both_arms(self):
        self.host.open()
        observation = self.observation()
        arguments = self.arguments(observation)
        self.devices[0].execute_error = RuntimeError("Synthetic partial write")
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        replay = self.host.submit(**arguments)
        self.assertEqual(replay["status"], "fault")
        with self.assertRaises(RuntimeError):
            self.host.submit(**self.arguments(observation, arm="right", event="peer-after-fault"))
        self.assertEqual(len(self.devices[0].calls), 1)

    def test_stability_without_target_arrival_faults_instead_of_completing(self):
        self.host.open()
        self.devices[0].receipt_changes = {
            "pose_error": {"position_m": .1, "rotation_rad": 0.}}
        arguments = self.arguments(self.observation())
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()
        self.assertEqual(len(self.devices[0].calls), 1)

    def test_missing_fresh_send_feedback_cannot_complete(self):
        self.host.open()
        self.devices[0].receipt_changes = {"feedback_all_after_send": False}
        arguments = self.arguments(self.observation())
        self.host.submit(**arguments)
        result = self.host.wait(arguments["event_id"], timeout=5)
        self.assertEqual(result["status"], "fault", result)
        self.assert_pair_fault()

    def test_close_disconnects_without_claiming_physical_stop(self):
        self.host.open()
        self.complete(self.arguments(self.observation()))
        report = self.host.close()
        self.assertIsNone(report["physical_stop_verified"])
        self.assertEqual(self.devices[0].closes, 1)
        self.assertEqual(len(self.devices[0].calls), 1)
        self.assertFalse(self.host.status()["open"])

    def test_repeated_close_does_not_repeat_device_cleanup_or_owner_release(self):
        self.host.open()
        with patch.object(self.host.ledger, "release", wraps=self.host.ledger.release) as release:
            first = self.host.close()
            repeated = self.host.close()
        self.assertEqual(repeated, first)
        self.assertEqual(self.devices[0].closes, 1)
        release.assert_called_once()
        self.assertFalse(self.host.status()["fault_latched"])
        self.assertEqual(self.host.status()["ledger"]["status"], "detached")

    def test_concurrent_close_cannot_release_already_detached_owner_twice(self):
        self.host.open()
        committed = threading.Event()
        allow_return = threading.Event()
        contender_started = threading.Event()
        contender_done = threading.Event()
        failures, results = [], []
        original_release = self.host.ledger.release
        def release_then_wait(owner):
            result = original_release(owner)
            committed.set()
            if not allow_return.wait(2):
                raise RuntimeError("Synthetic release return gate timed out")
            return result
        def close(contender=False):
            if contender:
                contender_started.set()
            try:
                results.append(self.host.close())
            except BaseException as exc:
                failures.append(exc)
            finally:
                if contender:
                    contender_done.set()
        first = threading.Thread(target=close, daemon=True)
        second = threading.Thread(target=close, args=(True,), daemon=True)
        with patch.object(self.host.ledger, "release", side_effect=release_then_wait) as release:
            first.start()
            try:
                self.assertTrue(committed.wait(1))
                second.start()
                self.assertTrue(contender_started.wait(1))
                self.assertFalse(contender_done.wait(.05))
                self.assertEqual(self.devices[0].closes, 1)
            finally:
                allow_return.set()
                first.join(2)
                if second.ident is not None:
                    second.join(2)
            release.assert_called_once()
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0], results[1])
        self.assertEqual(self.devices[0].closes, 1)
        self.assertFalse(self.host.status()["fault_latched"])
        self.assertEqual(self.host.status()["ledger"]["status"], "detached")

    def invalid_cleanup_latches(self, result):
        self.host.open()
        device = self.devices[0]
        device.close_result = result
        try:
            with self.assertRaises(RuntimeError):
                self.host.close()
        finally:
            # Cleanup may be invoked again by unittest; preserve the host latch.
            device.close_result = {"physical_stop_verified": None}
        self.assert_pair_fault()
        self.assertNotEqual(self.host.status()["ledger"]["status"], "detached")
        self.assertEqual(device.frame_attempts, 0)
        repeated = self.host.close()
        self.assertTrue(repeated["fault_latched"])
        self.assertEqual(device.closes, 1)

    def test_failed_device_cleanup_latches_instead_of_clean_detach(self):
        self.invalid_cleanup_latches({"physical_stop_verified": None,
                                      "arms": {"left": {"status": "cleanup_failed"}}})

    def test_cleanup_guard_violation_latches_instead_of_clean_detach(self):
        self.invalid_cleanup_latches({"physical_stop_verified": None,
                                      "guard_violations": ["unexpected transmit on close"]})


if __name__ == "__main__":
    unittest.main()
