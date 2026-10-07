"""Offline profile/receipt tests only; no physical motion or device access."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

from test_ros_joint_stop_probe_entry import fk, limits
import test_ros_interruptible_joint_entry as fixtures
import test_guarded_rx_latch as rx_fixtures

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location("j5_entry_under_test", ROOT/"scripts/ros_guarded_j5_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
profile = entry.profile
RAD = profile.support.RAD_PER_RAW


def configured_limits():
    result = limits()
    result["joint_tracking_margin_rad"] = list(profile.MARGINS)
    return result


def sample(raw):
    q = [v*RAD for v in raw]
    return dict(raw_q=list(raw), q=q, pose=fk(q), jaw_code=64, opening_m=.03444)


class OfflineTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)


class ProfileTests(OfflineTests):
    def latch(self, origin, target=None, *, jaw=False, hold_box=None):
        latch = profile.ProfileLatch(configured_limits())
        method = latch.arm_jaw if jaw else latch.arm_joint
        method(1, 7, origin, origin if target is None else target, .03444,
               jaw_code=64, token="offline", hold_box=hold_box)
        return latch

    def feed(self, latch, axis, raw):
        latch.observe(*rx_fixtures.joint_fragment(axis, raw), 100., 100.001)

    def test_exact_integer_j5_boundaries_at_positive_and_negative_angles(self):
        for center in (-69000, -32000, -1000, 0, 1000, 69000):
            origin = [67, 1000, -1000, 0, center, 0]
            for delta in (-301, -300, 300, 301):
                latch = self.latch(origin)
                raw = list(origin); raw[4] += delta
                self.feed(latch, 4, raw)
                with self.subTest(center=center, delta=delta):
                    self.assertEqual(latch.first_fault is None, abs(delta) <= 300)

    def test_other_axes_and_jaw_j5_retain_original_integer_boundary(self):
        origin = [67, 1000, -1000, 0, -32000, 0]
        for axis, jaw in [(0, False), (1, False), (2, False), (3, False), (5, False), (4, True)]:
            for delta in (-172, -171, 171, 172):
                latch = self.latch(origin, jaw=jaw)
                raw = list(origin); raw[axis] += delta
                self.feed(latch, axis, raw)
                with self.subTest(axis=axis, jaw=jaw, delta=delta):
                    self.assertEqual(latch.first_fault is None, abs(delta) <= 171)

    def test_nominal_boundary_and_bad_then_good_remain_latched(self):
        for axis, center, bad in ((1, 0, -1), (2, 0, 1), (4, 70000, 70001), (4, -70000, -70001)):
            origin = [67, 1000, -1000, 0, 0, 0]; origin[axis] = center
            latch = self.latch(origin)
            raw = list(origin); raw[axis] = bad
            self.feed(latch, axis, raw)
            first = copy.deepcopy(latch.first_fault)
            self.assertIsNotNone(first)
            self.feed(latch, axis, origin)
            self.assertEqual(latch.first_fault, first)
            with self.assertRaises(RuntimeError): latch.assert_clean()
            with self.assertRaises(RuntimeError): latch.arm_joint(2, 7, origin, origin, .03444)

    def test_original_and_hold_boxes_are_both_enforced(self):
        origin = [67, 1000, -1000, 0, -32000, 0]
        target = list(origin); target[4] = -31000
        for label, held_value, measured in (("hold", -31500, -31000), ("original", -30500, -30500)):
            held = list(origin); held[4] = held_value
            latch = self.latch(origin, target, hold_box=dict(origin_raw=held, target_raw=held))
            raw = list(origin); raw[4] = measured
            self.feed(latch, 4, raw)
            with self.subTest(box=label), self.assertRaisesRegex(RuntimeError, "Outside "+label):
                latch.assert_clean()

    def test_monitor_uses_same_integer_boundary_and_retains_jaw_drift_limit(self):
        origin = sample([67, 1000, -1000, 0, -32000, 0])
        for delta in (-301, -300, 300, 301):
            raw = list(origin["raw_q"]); raw[4] += delta
            current = sample(raw)
            if abs(delta) <= 300:
                profile.monitor(current, origin, origin["q"], configured_limits())
            else:
                with self.assertRaises(RuntimeError):
                    profile.monitor(current, origin, origin["q"], configured_limits())
        current = copy.deepcopy(origin); current["opening_m"] += .000501
        with self.assertRaisesRegex(RuntimeError, "Jaw changed"):
            profile.monitor(current, origin, origin["q"], configured_limits())

    def test_explicit_profile_required_and_original_physical_caps_cannot_increase(self):
        approved = configured_limits()
        self.assertEqual(profile.checked_limits(dict(physical_limits=approved)), approved)
        for kind in ("missing", "other_axis", "j5", "nan", "speed", "translation", "rotation"):
            modified = copy.deepcopy(approved)
            if kind == "missing": modified.pop("joint_tracking_margin_rad")
            elif kind == "other_axis": modified["joint_tracking_margin_rad"][0] = .004
            elif kind == "j5": modified["joint_tracking_margin_rad"][4] += 1e-9
            elif kind == "nan": modified["joint_tracking_margin_rad"][4] = math.nan
            elif kind == "speed": modified["max_speed_percent"] = 2
            elif kind == "translation": modified["max_translation_step_m"] = .03001
            elif kind == "rotation": modified["max_rotation_step_rad"] = .05001
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                profile.checked_limits(dict(physical_limits=modified))

    def test_path_proof_adds_exact_j5_box_budget_without_changing_caps(self):
        before = sample([67, 1000, -1000, 0, -32000, 0])
        target = list(before["raw_q"]); target[1] += 500
        original = profile.support.path_check(before, target, limits(), fk)
        approved = configured_limits(); snapshot = copy.deepcopy(approved)
        actual = profile.path_check(before, target, approved, fk)
        extra = math.pi/600-.003
        self.assertAlmostEqual(actual["position_bound_m"], original["position_bound_m"]+.091*extra)
        self.assertAlmostEqual(actual["rotation_bound_rad"], original["rotation_bound_rad"]+extra)
        self.assertEqual(actual["jaw_tracking_margin_rad"], [.003]*6)
        self.assertEqual(approved, snapshot)
        # Old box fits .05 rad; the newly enlarged box must reject this target.
        target[1] = before["raw_q"][1]+1750
        self.assertLess(profile.support.path_check(before, target, limits(), fk)["rotation_bound_rad"], .05)
        with self.assertRaisesRegex(RuntimeError, "original motion bounds"):
            profile.path_check(before, target, approved, fk)

    def test_private_function_and_three_executor_bindings_do_not_patch_old_globals(self):
        source = entry.guard.GuardedTask.execute
        old_support = source.__globals__["support"]
        clone = profile.private_function(source, support=entry.support)
        self.assertIs(clone.__code__, source.__code__)
        self.assertIsNot(clone.__globals__, source.__globals__)
        self.assertIs(source.__globals__["support"], old_support)
        for changed, original in ((entry.J5Task.execute, source),
                (entry.J5Task.send_hold, entry.guard.GuardedTask.send_hold),
                (entry.J5Task._send_once, entry.guard.v1.Interruptible.send_once)):
            self.assertIs(changed.__code__, original.__code__)
            self.assertIs(changed.__globals__["support"], entry.support)
            self.assertIsNot(original.__globals__["support"], entry.support)
        self.assertIs(entry.support.path_check, profile.path_check)
        self.assertIsNot(profile.support.path_check, profile.path_check)


class HandoffTests(OfflineTests):
    def setUp(self):
        super().setUp()
        self.reviewed = json.loads((ROOT/"runs/cola_on_cup_j5margin_20261006_185000/reviewed_handoff.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        self.parent = json.loads((entry.guard.v1.SESSION_ROOT/entry.frozen.child_name(boot)).read_text())

    def test_real_review_accepts_but_wrong_profile_receipt_or_authorization_rejects(self):
        endpoint = entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertLessEqual(max(abs(q-t)*RAD for q,t in zip(endpoint["raw_q"], entry.PARENT_TARGET)), .003)
        for key, value in (("reviewed", False), ("parent_sequence", 61),
                           ("first_segment_max_joint_deg", 1.), ("user_authorization", ""),
                           ("joint_tracking_margin_rad", [.003]*6)):
            reviewed = copy.deepcopy(self.reviewed); reviewed[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                entry.reviewed_evidence(reviewed, self.parent)
        parent = copy.deepcopy(self.parent)
        parent["status"]["receipts"][0]["socket_send_returns"] = 3
        with self.assertRaises(RuntimeError): entry.reviewed_evidence(self.reviewed, parent)

    def test_six_parent_bytes_unchanged_and_fixed_child_forbids_restart(self):
        boot = entry.guard.predecessor.PARENT_BOOT
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            entry.predecessor.predecessor.CHILD_NAME, entry.predecessor.CHILD_NAME, entry.frozen.child_name(boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT/name).read_bytes() for name in names}
            for name, data in originals.items(): (root/name).write_bytes(data)
            with entry.reserve(boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["generation"], 7)
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertFalse(session["feedback_excursion_physical_cause_resolved"])
                self.assertFalse(session["first_segment_verified"])
                self.assertEqual(store.load()["joint_tracking_margin_rad"], list(profile.MARGINS))
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, self.reviewed, session_root=root): pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


class IntegrationTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp(); self.control.feedback.close()
        tick = patch.object(profile.frozen.overlay.time, "time", side_effect=self.clock.time)
        tick.start(); self.addCleanup(tick.stop)
        self.piper = profile.sdk_overlay(rx_fixtures.BusPiper, configured_limits())(self.clock)
        self.piper.q = list(entry.PARENT_TARGET); self.node.piper = self.piper
        self.session.update(stage="task", generation=7, generations=[], commissioning_attempted=True,
                            held_raw=list(self.piper.q), first_segment_verified=False)
        def register(name, callback): self.services[name] = callback; return name
        self.control = entry.J5Task(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a,k))),
            configured_limits(), fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_driver", token="j5_offline")
        self.addCleanup(self.control.feedback.close)

    def test_fresh_handoff_baseline_zero_tx_and_first_half_degree_completes_four_frames(self):
        self.control.baseline(); self.assertEqual(self.piper.frames, [])
        result = self.control.execute(self.message(500))
        self.assertEqual(result["phase"], "completed")
        self.assertEqual([f[0] for f in self.piper.frames], [0x151, 0x155, 0x156, 0x157])
        self.assertTrue(self.session["first_segment_verified"])
        self.assertGreaterEqual(result["stable_window"]["duration_s"], 3.)

    def test_first_segment_above_half_degree_rejected_zero_tx_and_cannot_retry(self):
        with self.assertRaisesRegex(RuntimeError, "ceiling0.5"):
            self.control.execute(self.message(501))
        self.assertEqual(self.piper.frames, [])
        self.assertIsNotNone(self.session["failure"])
        with self.assertRaises(RuntimeError): self.control.execute(self.message(500))
        self.assertEqual(self.piper.frames, [])

    def test_arrival_handoff_still_point_zero_zero_three_not_j5_tracking_margin(self):
        self.piper.q[4] += 200
        with self.assertRaisesRegex(RuntimeError, "original target arrival"):
            self.control.baseline()
        self.assertEqual(self.piper.frames, [])

    def test_fault_after_first_frame_blocks_remaining_frames_and_preserves_receipt(self):
        def after_frame(_):
            if len(self.piper.frames) != 1: return
            raw = list(self.piper.q); raw[4] += 301
            for current in (raw, self.piper.q):
                ident, data = rx_fixtures.joint_fragment(4, current)
                self.piper.ParseCANFrame(rx_fixtures.message(ident, data, self.clock.time()))
        self.piper.on_frame = after_frame
        with self.assertRaisesRegex(RuntimeError, "latched"):
            self.control.execute(self.message(500))
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (1,1))
        self.assertEqual(len(self.piper.frames), 1)
        with self.assertRaises(RuntimeError): self.control.execute(self.message(500))
        with self.assertRaises(RuntimeError): self.control.gripper(types.SimpleNamespace())
        self.assertEqual(len(self.piper.frames), 1)


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_") and _name not in IntegrationTests.__dict__:
        setattr(IntegrationTests, _name, None)


if __name__ == "__main__": unittest.main()
