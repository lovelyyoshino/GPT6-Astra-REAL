"""Offline single 67 mm relief scope; fake bus, no physical release claim."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import types
import unittest

import test_guarded_wide_entry as wide

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_relief67_20261006_205300"
SPEC = importlib.util.spec_from_file_location(
    "opening_relief_under_test", ROOT / "scripts/ros_guarded_relief_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
SPEC_CLIENT = importlib.util.spec_from_file_location(
    "opening_relief_client_under_test", ROOT / "scripts/ros_guarded_relief_client.py")
client = importlib.util.module_from_spec(SPEC_CLIENT)
SPEC_CLIENT.loader.exec_module(client)


class ReliefTests(wide.OpeningTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.piper.jaw = .06706
        self.session.update(regrasp_stage="opening67_ready", wide_open_attempted=False,
                            generation=11, wide_opening_reviews=[])
        self.node.adopted = True
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.ReliefTask(
            self.node, types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            wide.wide_limits(), wide.fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_relief", token="relief_offline")
        self.addCleanup(self.control.feedback.close)

    def request(self, width=.067):
        return super().request(width)

    def test_only67_once_exact_frozen_sdk_frame_and_no_joint_permission(self):
        self.assertEqual(self.piper.frames, [])
        for width in (.065, .070):
            with self.subTest(width=width), self.assertRaises(RuntimeError):
                self.control.gripper(self.request(width))
        with self.assertRaises(RuntimeError):
            self.control.execute(self.message(100))
        result = self.control.gripper(self.request())
        self.assertEqual(self.piper.frames, [(0x159, struct.pack(">iHBB", 67000, 200, 1, 0))])
        self.assertEqual(result["phase"], "completed")
        self.assertTrue(result["jaw_target_reached"])
        self.assertFalse(result["release_verified"])
        self.assertFalse(self.node.adopted)
        for operation in (lambda: self.control.gripper(self.request()),
                          lambda: self.control.execute(self.message(100))):
            with self.assertRaises(RuntimeError):
                operation()
        self.assertEqual(len(self.piper.frames), 1)

    def test_review_requires_bound_evidence_and_fresh_three_seconds_before_retreat(self):
        self.control.gripper(self.request())
        retired = self.review_args()
        with self.assertRaises(FileNotFoundError):
            self.control.review_opening(*retired)
        path, material, _ = self.review_file("opening70_ready")
        with self.assertRaises(RuntimeError):
            self.control.review_opening(*retired)
        material["next_stage"] = "retreat_ready"
        material["actual_opening_reviewed"] = False
        path.write_text(json.dumps(material))
        with self.assertRaises(RuntimeError):
            self.control.review_opening(*retired)
        self.approve("retreat_ready")
        self.assertEqual(self.session["regrasp_stage"], "retreat_ready")
        self.assertTrue(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 1)
        with self.assertRaises(RuntimeError):
            self.control.review_opening(*retired)

    def test_partial67_latches_without_review_or_reissue(self):
        self.piper.failure = "jaw_send_failure"
        with self.assertRaisesRegex(RuntimeError, "jaw send failed"):
            self.control.gripper(self.request())
        receipt = self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"], receipt["socket_send_returns"]), (1, 0))
        self.assertIsNotNone(self.session["failure"])
        for operation in (lambda: self.control.review_opening(*self.review_args()),
                          lambda: self.control.gripper(self.request())):
            with self.assertRaises(RuntimeError):
                operation()
        self.assertEqual(self.piper.frames, [])

    def test_raw_bad_then_good_before67_never_clears_or_sends(self):
        good = list(self.piper.q)
        bad = list(good)
        bad[2] = 1
        for raw in (bad, good):
            self.piper.ParseCANFrame(wide.rx_fixtures.message(
                *wide.rx_fixtures.joint_fragment(2, raw), self.clock.time()))
        first = copy.deepcopy(self.piper.rx_latch.first_fault)
        self.assertIsNotNone(first)
        for _ in range(2):
            with self.assertRaises(RuntimeError):
                self.control.gripper(self.request())
        self.assertEqual(self.piper.rx_latch.first_fault, first)
        self.assertEqual(self.piper.frames, [])


class ClientTests(wide.ClientTests):
    def setUp(self):
        super().setUp()
        self.transport = client.ReliefTransport.__new__(client.ReliefTransport)
        before = dict(stage="task", regrasp_stage="opening67_ready", phase="idle",
            active=False, stop_latched=False, failure=None, adoption_token="a" * 32,
            generation=11, sequence=0)
        after = dict(before, regrasp_stage="opening67_review", phase="completed",
                     sequence=1, result={"kind": "gripper"})
        self.transport.status = iter([before, before, after]).__next__
        self.transport.observe = lambda: None
        self.transport.master = types.SimpleNamespace(getSystemState=lambda: (
            [], [], [("/piper/right/gripper_srv", [wide.client.frozen.NODE])]))
        def call(*args):
            self.calls.append(args)
            return types.SimpleNamespace(status=True, code=15900)
        self.transport.rospy = types.SimpleNamespace(
            wait_for_service=lambda *a, **k: None, ServiceProxy=lambda *a: call)

    def test_client67mm_is_one_unchanged_force_request(self):
        result = self.transport.gripper(67.)
        self.assertEqual(self.calls, [(.067, .2, 1, 0)])
        self.assertTrue(result["feedback_stable"])
        self.assertFalse(result["grasp_verified"])


class HandoffTests(wide.OfflineTests):
    def setUp(self):
        super().setUp()
        self.boot = entry.guard.predecessor.PARENT_BOOT
        self.reviewed = json.loads((RUN / "reviewed_relief.json").read_text())
        path = entry.guard.v1.SESSION_ROOT / entry.wide.child_name(self.boot)
        self.parent = json.loads(path.read_text())

    def test_real_unmet70_admission_rejects_rx_fault_partial_and_different_target(self):
        endpoint = entry.reviewed_evidence(self.reviewed, self.parent)
        self.assertEqual(endpoint["opening_m"], .06706)
        self.assertEqual(self.parent["status"]["phase"], "failed")
        self.assertEqual(self.parent["status"]["sequence"], 2)
        parent = copy.deepcopy(self.parent)
        parent["raw_feedback_fault"] = {"reason": "synthetic test fault"}
        with self.assertRaisesRegex(RuntimeError, "RX safety"):
            entry.reviewed_evidence(self.reviewed, parent)
        parent = copy.deepcopy(self.parent)
        parent["pending"]["attempted_frames"] = 0
        with self.assertRaisesRegex(RuntimeError, "Incomplete/different"):
            entry.reviewed_evidence(self.reviewed, parent)
        reviewed = copy.deepcopy(self.reviewed)
        reviewed["relief_target_m"] = .070
        with self.assertRaises(RuntimeError):
            entry.reviewed_evidence(reviewed, self.parent)

    def test_nine_failed_parents_unchanged_and_fixed_child_cannot_restart(self):
        previous = entry.previous
        interior = previous.motion.predecessor
        names = [self.boot + ".json", entry.guard.predecessor.CHILD_NAME,
            "guarded_task_" + self.boot + ".json", interior.predecessor.CHILD_NAME,
            interior.CHILD_NAME, previous.motion.frozen.child_name(self.boot),
            previous.motion.child_name(self.boot), previous.release.child_name(self.boot),
            entry.wide.child_name(self.boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (entry.guard.v1.SESSION_ROOT / name).read_bytes() for name in names}
            for name, data in originals.items():
                (root / name).write_bytes(data)
            with entry.reserve(self.boot, self.reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["regrasp_stage"], "opening67_ready")
                self.assertFalse(session["wide_open_attempted"])
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertEqual(session["prior_unmet_width_failure"], self.parent["failure"])
                self.assertEqual(endpoint["opening_m"], .06706)
                self.assertEqual(store.load()["actual_target_changed_from_m"], .07)
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(self.boot, self.reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root / name).read_bytes() for name in names}, originals)


for _base, _child in ((wide.OpeningTests, ReliefTests), (wide.ClientTests, ClientTests)):
    for _name in dir(_base):
        if _name.startswith("test_") and _name not in _child.__dict__:
            setattr(_child, _name, None)
del _base, _child


if __name__ == "__main__":
    unittest.main()
