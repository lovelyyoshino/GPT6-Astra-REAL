"""Raw-feedback replay and concurrent fake transport; no device access."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

from test_ros_joint_stop_probe_entry import limits
import test_ros_guarded_task_entry as guarded_fixtures
import test_ros_interruptible_joint_entry as fixtures

SPEC = importlib.util.spec_from_file_location(
    "rx_latch_under_test", Path(__file__).parents[1]/"scripts/guarded_rx_latch.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
RX_SPEC = importlib.util.spec_from_file_location(
    "rx_entry_under_test", Path(__file__).parents[1]/"scripts/ros_guarded_rx_entry.py")
rx_entry = importlib.util.module_from_spec(RX_SPEC)
RX_SPEC.loader.exec_module(rx_entry)


ORIGIN = [67, 1000, -1000, 0, 7144, -1937]
TARGET = [67, 1000, -1000, 0, 7144, -339]


def joint_fragment(axis, raw):
    start = 2*(axis//2)
    return 0x2a5+axis//2, struct.pack(">ii", *raw[start:start+2])


class ReplayTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        self.latch = entry.RawFeedbackLatch(limits())
        self.stamp = 100.

    def observe(self, ident, data):
        self.stamp += .001
        return self.latch.observe(ident, data, self.stamp, self.stamp+.001)

    def arm(self, method="arm_joint"):
        return getattr(self.latch, method)(1, 0, ORIGIN, TARGET, .03444, jaw_code=64, token="offline")

    def test_nominal_bad_then_good_never_clears_first_fault(self):
        raw = list(ORIGIN); raw[2] = 21
        self.observe(*joint_fragment(2, raw))
        first = copy.deepcopy(self.latch.first_fault)
        self.assertIsNotNone(first)
        self.assertEqual(first["id"], 0x2a6)
        self.assertEqual(first["data_hex"], joint_fragment(2, raw)[1].hex())
        self.assertEqual((first["axis"], first["raw"]), (3, 21))
        self.observe(*joint_fragment(2, ORIGIN))
        self.assertEqual(self.latch.first_fault, first)
        with self.assertRaises(RuntimeError): self.latch.assert_clean()
        for method in ("arm_joint", "arm_jaw"):
            with self.subTest(method=method), self.assertRaises(RuntimeError): self.arm(method)
        self.assertEqual(self.latch.first_fault, first)

    def test_transient_zero_j6_outside_original_box_is_not_hidden_by_good_return(self):
        self.arm()
        good = list(TARGET); good[5] = -335
        self.observe(*joint_fragment(5, good)); self.latch.assert_clean()
        bad = list(good); bad[5] = 0
        self.observe(*joint_fragment(5, bad)); first = copy.deepcopy(self.latch.first_fault)
        self.assertIsNotNone(first)
        self.observe(*joint_fragment(5, good))
        self.assertEqual(self.latch.first_fault, first)
        with self.assertRaises(RuntimeError): self.latch.assert_clean()

    def test_original_point_zero_zero_is_nominal_but_one_raw_outside_is_not(self):
        raw = list(ORIGIN); raw[1:3] = [0, 0]
        self.observe(*joint_fragment(1, raw)); self.observe(*joint_fragment(2, raw))
        self.latch.assert_clean()
        raw[1] = -1
        self.observe(*joint_fragment(1, raw))
        self.assertIsNotNone(self.latch.first_fault)

    def test_good_frames_and_exact_original_point_zero_zero_three_margin_stay_clean(self):
        self.arm()
        # floor(.003 / RAD_PER_RAW) = 171 raw; 172 must reject.
        for raw in (ORIGIN, TARGET, [67, 1000, -1000, 0, 7144, -168]):
            for axis in (0, 2, 4): self.observe(*joint_fragment(axis, raw))
        self.observe(0x2a1, bytes.fromhex("0100010001000000"))
        self.observe(0x2a8, struct.pack(">iHBB", 34440, 0, 64, 0))
        for ident in range(0x261, 0x267): self.observe(ident, bytes([0, 0, 0, 0, 0, 64, 0, 0]))
        self.latch.assert_clean(); self.assertIsNone(self.latch.first_fault)
        bad = list(TARGET); bad[5] += 172
        self.observe(*joint_fragment(5, bad))
        with self.assertRaises(RuntimeError): self.latch.assert_clean()

    def test_jaw_transaction_keeps_arm_box_guard(self):
        self.arm("arm_jaw")
        bad = list(TARGET); bad[0] += 172
        self.observe(*joint_fragment(0, bad))
        with self.assertRaises(RuntimeError): self.latch.assert_clean()

    def test_hold_box_is_additional_and_never_replaces_original_box(self):
        for case in ("original", "hold"):
            latch = entry.RawFeedbackLatch(limits())
            held = list(TARGET); held[5] = -900 if case == "hold" else 0
            latch.arm_joint(1, 0, ORIGIN, TARGET, .03444,
                            hold_box=dict(origin_raw=held, target_raw=held))
            sample = list(TARGET); sample[5] = -335 if case == "hold" else 0
            ident, data = joint_fragment(5, sample)
            latch.observe(ident, data, 100., 100.001)
            with self.subTest(case=case), self.assertRaisesRegex(RuntimeError, "Outside "+case):
                latch.assert_clean()


class FakeParent:
    def __init__(self):
        self.rx_lock = threading.RLock()
        self.received = {}; self.sequence = 0; self.broken = None; self.ticket = None
        self.sent = []
        self.comm = types.SimpleNamespace(send_bus=types.SimpleNamespace(send=self.raw_send))

    def raw_send(self, frame, *args, **kwargs): self.sent.append(frame)
    def GetCanBus(self): return self.comm
    def ParseCANFrame(self, message):
        with self.rx_lock:
            self.received[message.arbitration_id] = (message.timestamp, bytes(message.data))
            self.sequence += 1
    def snapshot(self): return dict(sequence=self.sequence)
    def healthy(self, snapshot, allow_moving=False): return snapshot


def message(ident, data, stamp=100.):
    return types.SimpleNamespace(arbitration_id=ident, data=data, timestamp=stamp,
                                 is_extended_id=False, is_remote_frame=False,
                                 is_error_frame=False, is_rx=True, dlc=8)


class OverlayTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        clock = patch.object(entry.time, "time", return_value=100.001)
        clock.start(); self.addCleanup(clock.stop)
        self.piper = entry.sdk_overlay(FakeParent, limits())()
        self.piper.rx_context_provider = lambda: dict(kind="joint", sequence=1, generation=0,
            origin_raw=ORIGIN, target_raw=TARGET, jaw_m=.03444, jaw_code=64, token="offline")

    def bad_then_good(self):
        raw = list(ORIGIN); raw[2] = 21
        ident, data = joint_fragment(2, raw)
        self.piper.ParseCANFrame(message(ident, data))
        ident, data = joint_fragment(2, ORIGIN)
        self.piper.ParseCANFrame(message(ident, data))

    def test_overlay_initialization_is_zero_tx_and_valid_feedback_keeps_send_available(self):
        self.assertEqual(self.piper.sent, [])
        ident, data = joint_fragment(2, ORIGIN)
        self.piper.ParseCANFrame(message(ident, data))
        self.piper.rx_latch.assert_clean()
        self.piper.GetCanBus().send_bus.send(message(0x151, bytes.fromhex("0101010000000000")))
        self.assertEqual(len(self.piper.sent), 1)

    def test_no_bound_context_means_zero_tx_even_with_no_rx_fault(self):
        self.piper.rx_context_provider = None
        with self.assertRaisesRegex(RuntimeError, "context unavailable"):
            self.piper.GetCanBus().send_bus.send(message(0x151, bytes(8)))
        self.assertIsNone(self.piper.rx_latch.first_fault)
        self.assertEqual(self.piper.sent, [])

    def test_malformed_received_flags_latch_but_local_echo_does_not(self):
        for flag in ("is_extended_id", "is_remote_frame", "is_error_frame"):
            piper = entry.sdk_overlay(FakeParent, limits())()
            ident, data = joint_fragment(2, ORIGIN)
            frame = message(ident, data); frame.is_rx = False; setattr(frame, flag, True)
            piper.ParseCANFrame(frame)
            self.assertIsNone(piper.rx_latch.first_fault)
            frame.is_rx = True
            piper.ParseCANFrame(frame)
            with self.subTest(flag=flag), self.assertRaisesRegex(RuntimeError, "Malformed"):
                piper.rx_latch.assert_clean()
            self.assertEqual(piper.sent, [])

    def test_bad_then_good_blocks_each_normal_hold_jaw_and_remaining_fragment_id(self):
        self.bad_then_good()
        first = copy.deepcopy(self.piper.rx_latch.first_fault)
        for name, ident in (("normal_mode", 0x151), ("hold_mode", 0x151),
                            ("joint12", 0x155), ("joint34", 0x156),
                            ("joint56", 0x157), ("jaw", 0x159)):
            with self.subTest(path=name), self.assertRaises(RuntimeError):
                self.piper.GetCanBus().send_bus.send(message(ident, bytes(8)))
        with self.assertRaises(RuntimeError): self.piper.healthy({})
        self.assertEqual(self.piper.rx_latch.first_fault, first)
        self.assertEqual(self.piper.sent, [])
        self.assertTrue(self.piper.broken)
        self.assertEqual(self.piper.snapshot()["raw_feedback_fault"], first)

    def test_rx_callback_preserves_fault_without_file_io_or_commands(self):
        with patch("builtins.open", side_effect=AssertionError("RX callback must not perform file I/O")), \
                patch.object(Path, "open", side_effect=AssertionError("RX callback must not perform file I/O")):
            self.bad_then_good()
        self.assertEqual(self.piper.sent, [])
        self.assertIsNotNone(self.piper.rx_latch.first_fault)

    def test_fault_after_first_frame_blocks_remaining_three_without_retry(self):
        sender = self.piper.GetCanBus().send_bus.send
        sender(message(0x151, bytes.fromhex("0101010000000000")))
        self.bad_then_good()
        for ident in (0x155, 0x156, 0x157):
            with self.subTest(ident=ident), self.assertRaises(RuntimeError):
                sender(message(ident, bytes(8)))
        self.assertEqual([f.arbitration_id for f in self.piper.sent], [0x151])

    def test_waiting_tx_thread_sees_fault_latched_under_same_rx_lock(self):
        entered, finished = threading.Event(), threading.Event()
        errors = []
        def transmit():
            entered.set()
            try: self.piper.GetCanBus().send_bus.send(message(0x159, bytes(8)))
            except Exception as exc: errors.append(exc)
            finally: finished.set()
        with self.piper.rx_lock:
            worker = threading.Thread(target=transmit, daemon=True)
            worker.start()
            self.assertTrue(entered.wait(1.))
            self.assertEqual(self.piper.sent, [])
            self.bad_then_good()
        self.assertTrue(finished.wait(1.)); worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        self.assertEqual(self.piper.sent, [])


class BusPiper(guarded_fixtures.MultiPiper):
    """Route the existing fake executor's exact ticket through the deep bus."""
    def __init__(self, clock):
        super().__init__(clock)
        self.comm = types.SimpleNamespace(send_bus=types.SimpleNamespace(send=self.raw_send))

    def GetCanBus(self): return self.comm
    def raw_send(self, frame):
        return super().tx((frame.arbitration_id, bytes(frame.data)))
    def tx(self, frame):
        return self.comm.send_bus.send(message(frame[0], frame[1], self.clock.time()))
    def ParseCANFrame(self, frame):
        self.received[frame.arbitration_id] = (frame.timestamp, bytes(frame.data))


class ReceiptIntegrationTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        tick = patch.object(entry.time, "time", side_effect=self.clock.time)
        tick.start(); self.addCleanup(tick.stop)
        self.piper = entry.sdk_overlay(BusPiper, limits())(self.clock)
        origin = list(self.piper.q); target = list(origin); target[1] -= 500
        self.piper.rx_context_provider = lambda: dict(kind="joint", sequence=1, generation=0,
            origin_raw=origin, target_raw=target, jaw_m=.03444, jaw_code=64, token="rx_receipt")
        self.node.piper = self.piper
        self.session.update(stage="task", generation=0, generations=[],
                            commissioning_attempted=True, held_raw=list(self.piper.q),
                            first_segment_verified=False)
        def register(name, callback): self.services[name] = callback; return name
        self.control = rx_entry.RXGuardedTask(
            self.node, types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            fixtures.limits(), fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_driver", token="rx_receipt")
        self.addCleanup(self.control.feedback.close)

    def test_constructor_and_stable_adoption_baseline_send_no_frames(self):
        self.assertEqual(self.piper.frames, [])
        first = self.clock.time()
        self.control.baseline()
        self.assertGreaterEqual(self.clock.time()-first, 3.)
        self.assertEqual(self.piper.frames, [])
        self.assertFalse(self.session["first_segment_verified"])
        with self.assertRaisesRegex(RuntimeError, "small J segment"):
            self.control.gripper(types.SimpleNamespace())
        self.assertEqual(self.piper.frames, [])

    def test_first_frame_rx_fault_persists_partial_receipt_and_blocks_retry_and_jaw(self):
        def first_frame(_):
            if len(self.piper.frames) != 1: return
            original = list(self.piper.q); bad = list(original); bad[2] = 21
            for raw in (bad, original):
                ident, data = joint_fragment(2, raw)
                self.piper.ParseCANFrame(message(ident, data, self.clock.time()))
            if getattr(self, "synchronize_during_send", False):
                self.control.synchronize_fault()
                self.early_failure = copy.deepcopy(self.session["failure"])
        self.piper.on_frame = first_frame
        with self.assertRaisesRegex(RuntimeError, "latched"):
            self.control.execute(self.message(500))
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (1, 1))
        self.assertEqual(self.session["pending"]["attempted_frames"], 1)
        self.assertIsNotNone(self.session["failure"])
        self.assertIsNone(self.piper.ticket)
        self.assertEqual([f[0] for f in self.piper.frames], [0x151])
        with self.assertRaises(RuntimeError): self.control.execute(self.message(500))
        with self.assertRaises(RuntimeError): self.control.gripper(types.SimpleNamespace())
        self.assertEqual(len(self.piper.frames), 1)

    def test_timer_fault_before_send_finally_preserves_first_fault_and_final_partial_receipt(self):
        self.synchronize_during_send = True
        self.test_first_frame_rx_fault_persists_partial_receipt_and_blocks_retry_and_jaw()
        result = json.loads((self.path/"action_000001_result.json").read_text())
        self.assertEqual(result["phase"], "failed")
        self.assertEqual(result["receipts"][0]["socket_send_returns"], 1)
        self.assertEqual(result["raw_feedback_fault"]["raw"], 21)
        persisted = self.store.load()
        self.assertEqual(persisted["pending"]["attempted_frames"], 1)
        self.assertEqual(persisted["raw_feedback_fault"]["raw"], 21)
        self.assertFalse(self.early_failure["accepted_target_may_continue"])
        self.assertTrue(persisted["failure"]["accepted_target_may_continue"])
        self.assertEqual(persisted["failure"]["error"], self.early_failure["error"])
        self.assertEqual(persisted["failure"]["unix_s"], self.early_failure["unix_s"])


class HandoffTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)
        self.reviewed = json.loads((rx_entry.ROOT/
            "runs/cola_on_cup_rxguard_20261006_180445/reviewed_handoff.json").read_text())
        self.parent = json.loads((rx_entry.guard.v1.SESSION_ROOT/rx_entry.predecessor.CHILD_NAME).read_text())

    def test_real_completed_parent_review_accepts_only_explicit_bound_evidence(self):
        endpoint = rx_entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertEqual(endpoint["raw_q"], self.reviewed["completed_actual_raw"])
        for key, value in (("reviewed", False), ("current_completed_state_reviewed", False),
                           ("first_segment_max_joint_deg", 2), ("allow_limit_relaxation", True),
                           ("home_replay_allowed", True), ("parent_sequence", 37)):
            reviewed = copy.deepcopy(self.reviewed); reviewed[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                rx_entry.reviewed_evidence(reviewed, self.parent)

    def test_five_parent_files_unchanged_and_boot_child_cannot_retry(self):
        boot = rx_entry.guard.predecessor.PARENT_BOOT
        names = [boot+".json", rx_entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
                 rx_entry.predecessor.predecessor.CHILD_NAME, rx_entry.predecessor.CHILD_NAME]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (rx_entry.guard.v1.SESSION_ROOT/name).read_bytes() for name in names}
            for name, data in originals.items(): (root/name).write_bytes(data)
            with rx_entry.reserve(boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(endpoint["raw_q"], self.reviewed["completed_actual_raw"])
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertFalse(session["feedback_excursion_physical_cause_resolved"])
                self.assertFalse(session["first_segment_verified"])
                self.assertEqual(store.load()["generation"], 6)
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with rx_entry.reserve(boot, self.reviewed, session_root=root): pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in ReceiptIntegrationTests.__dict__:
        setattr(ReceiptIntegrationTests, _name, None)


if __name__ == "__main__": unittest.main()
