"""Real service/host/ledger/device with fake CAN and test-only cache history.

No physical bootstrap, scene geometry authentication or stop qualification is
inferred from these fixtures. All sockets are blocked by JointFixture.
"""
import copy
import json
from pathlib import Path
import tempfile
import threading
from unittest.mock import patch

from robot_tools import pair_device, joint_path
from robot_tools.hold_transaction import joint_hold_frames
from robot_tools.pair_host import PairHost
from robot_tools.service import ToolService
from test_pair_host import TASK
from test_pair_joint_adapter import JointFixture
from test_joint_path import context as context_fixture


class HostJointFixture(JointFixture):
    def setUp(self):
        # Reuse fake SDK/feedback fixtures but let ONLY the actual PairHost open
        # its real GuardedPairDevice. The temporary fixture device stays closed.
        with patch.object(pair_device.GuardedPairDevice, "open", return_value=None):
            super().setUp()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.workspace = Path(self.temp.name)
        self.root = self.workspace / "projects/piperx_cloth_demo"
        (self.root / "configs").mkdir(parents=True)
        self.profile["cameras"] = {"front": "fake-front", "left_wrist": "fake-left", "right_wrist": "fake-right"}
        (self.root / "configs/robot.json").write_text(json.dumps(self.profile))
        self.service = ToolService(self.root)
        self.host = PairHost(self.service.runs, self.profile, "joint-run", TASK,
                            clock=self.clock.time, background=False, joint_sources_provider=self.sources)
        self.service.pair_host, self.service.persistent = self.host, True
        self.addCleanup(self.host.close)
        self.host.open()
        self.device = self.host.device
        self.frame = 0

    def snapshot(self, robot, gripper):
        # Synthetic receive clock, not a claim of actual 1 kHz device feedback.
        self.clock.sleep(.001)
        return super().snapshot(robot, gripper)

    def sources(self, scene, arm):
        ctx = context_fixture(arm=arm)
        return {key: copy.deepcopy(ctx[key]) for key in
                ("model_catalog", "urdf_source", "controller_limits", "geometry")}

    def seed_test_cache(self, arm="right"):
        binding = self.device.joint_binding(arm)
        identity = {"run_id": self.host.run_id, "owner": self.host.owner, "epoch": self.host.owner,
                    "worker_id": "test-only-prior-worker", "arm": arm,
                    **{key: binding[key] for key in ("connection_id", "model", "firmware_profile")}}
        raw, _ = joint_path.encode_joint_target(self.joints[arm])
        # Explicit TEST PRECONDITION. Production has no public cache setter and
        # this fixture bypasses the separate initialization entry. This never
        # constitutes hardware history.
        self.device._joint_cache[arm] = {"event_id": "test-only-prior-send", "identity": identity,
            "target_raw": raw, "frame_receipts": [{"frame": frame, "outcome": "returned",
                "returned_at": self.clock.time()-.1+index*.001}
                for index, frame in enumerate(joint_hold_frames(raw))]}

    def observe(self):
        self.frame += 1
        self.clock.sleep(.01)
        directory = self.workspace / "artifacts" / ("capture-"+str(self.frame))
        directory.mkdir(parents=True)
        rgb = {"capture_id": "capture-"+str(self.frame), "cameras": {}}
        for view, key in (("front", "front"), ("left_hand", "left_wrist"), ("right_hand", "right_wrist")):
            path = directory / (view+".png")
            path.write_bytes(("synthetic RGB "+str(self.frame)+view).encode())
            rgb["cameras"][view] = {"serial": self.profile["cameras"][key], "rgb_path": str(path),
                "frame_number": self.frame, "host_received_at": self.clock.time(), "depth_enabled": False}
        path = directory / "observation.json"
        path.write_text(json.dumps(rgb))
        return self.service.call("robot_pair_observe", {"rgb_observation_path": str(path)})

    def request(self, *, arm="right", event="joint-1"):
        scene = self.observe()
        target = self.joints[arm][:]
        target[5] += .001
        peer = "left" if arm == "right" else "right"
        return {"event_id": event, "observation_id": scene["observation_id"],
            "peer_receipt_id": scene["peer_receipts"][peer]["receipt_id"], "arm": arm,
            "kind": "joint", "operation": "approach", "target_joints_rad": target}

    def submit(self, request):
        return self.service.call("robot_pair_submit_once", request)

class HostJointIntegrationTests(HostJointFixture):
    def test_normal_joint_receipt_passes_real_service_host_ledger_and_device(self):
        self.seed_test_cache()
        request = self.request()
        self.assertEqual(self.submit(request)["status"], "pending")
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        receipt = result["receipt"]
        self.assertTrue(receipt["ok"])
        self.assertEqual(receipt["execution_mode"], "joint")
        self.assertEqual(receipt["hardware_commands_sent"], 4)
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(self.host.ledger.status()["steps"], 1)
        self.assertIsNone(receipt["physical_stop_verified"])
        self.assertIsNone(receipt["object_task_success"])

    def test_completed_event_replay_needs_no_new_scene_and_sends_nothing(self):
        self.seed_test_cache()
        request = self.request()
        self.submit(request)
        self.assertEqual(self.host.wait(request["event_id"], 10)["status"], "completed")
        before = len(self.ids())
        replay = self.submit(request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(len(self.ids()), before)
        self.assertEqual(self.host.ledger.status()["steps"], 1)

    def test_missing_source_provider_refuses_before_budget_or_tx(self):
        self.seed_test_cache()
        self.host.joint_sources_provider = None
        request = self.request()
        with self.assertRaisesRegex(RuntimeError, "source adapter unavailable"):
            self.submit(request)
        self.assertEqual(self.ids(), [])
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.ledger.status()["steps"], 0)

    def test_unknown_real_connection_cache_is_not_filled_from_feedback(self):
        request = self.request()
        with self.assertRaisesRegex(RuntimeError, "bootstrap required"):
            self.submit(request)
        self.assertEqual(self.ids(), [])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.ledger.status()["steps"], 0)

    def test_caller_cannot_supply_context_cache_or_geometry(self):
        self.seed_test_cache()
        request = self.request()
        for key in ("context", "cached_target", "geometry", "hold_bridge", "verified"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.submit({**request, key: True})
        self.assertEqual(self.ids(), [])
        self.assertEqual(self.host.ledger.status()["steps"], 0)

    def test_next_worker_consumes_prior_real_adapter_receipt_without_reseeding(self):
        self.seed_test_cache()
        first = self.request(event="joint-first")
        self.submit(first)
        self.assertEqual(self.host.wait(first["event_id"], 10)["status"], "completed")
        cache = self.device.joint_binding("right")["cached_target"]
        self.assertEqual(cache["event_id"], "joint-first")
        self.assertEqual(cache["identity"]["worker_id"], "joint-first")
        second = self.request(event="joint-next-worker")
        self.submit(second)
        result = self.host.wait(second["event_id"], 10)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        self.assertEqual(result["receipt"]["joint_path_plan"]["cached_target"], cache)
        self.assertEqual(len(self.ids()), 8)
        self.assertEqual(self.host.ledger.status()["steps"], 2)

    def test_partial_original_send_latches_host_without_any_hold(self):
        self.seed_test_cache()
        request = self.request()
        self.robots["right"].fail_id = 0x156
        self.submit(request)
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "fault")
        receipt = result["receipt"]["device_receipt"]
        self.assertIsNone(receipt["hold_receipt"])
        self.assertEqual(self.ids(), [0x151, 0x155])
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)
        self.service.call("robot_pair_cancel", {"reason": "cancel after partial failure"})
        self.assertEqual(self.ids(), [0x151, 0x155])

    def test_explicit_cancel_after_complete_original_sends_one_hold_and_keeps_fault(self):
        self.seed_test_cache()
        request = self.request()
        recorded, release = threading.Event(), threading.Event()
        original_record = self.host.ledger.record_original_send
        def gate(*args):
            result = original_record(*args)
            recorded.set()
            if not release.wait(5):
                raise RuntimeError("Synthetic original receipt gate timed out")
            return result
        self.host.ledger.record_original_send = gate
        try:
            self.submit(request)
            self.assertTrue(recorded.wait(5))
            cancel = self.service.call("robot_pair_cancel", {"reason": "Explicit test user cancellation"})
            self.assertTrue(cancel["software_cancelled"])
            self.assertIsNone(cancel["physical_stop_verified"])
        finally:
            release.set()
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "fault")
        receipt = result["receipt"]["device_receipt"]
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157]*2, receipt.get("errors"))
        self.assertTrue(receipt["hold_receipt"]["hold_observed"], receipt.get("errors"))
        self.assertEqual(receipt["hardware_commands_sent"], 8)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 2)
        self.assertTrue(self.host.status()["fault_latched"])
        hold_id = receipt["hold_receipt"]["hold_event_id"]
        persisted = self.host.ledger.hold_event(hold_id)
        self.assertTrue(persisted["receipt"]["hold_observed"])
        self.assertIsNone(persisted["receipt"]["physical_stop_verified"])
        self.assertEqual(self.ids("left"), [])
        with self.assertRaises(RuntimeError):
            self.device.execute("left", "gripper", .049)
        self.assertEqual(len(self.ids()), 8)

    def test_slow_hold_frame_claim_cannot_send_after_original_rgb_expiry(self):
        # Real host/ledger/device; this fixture supplies a mocked SDK and CAN.
        self.seed_test_cache()
        request = self.request()
        recorded, release = threading.Event(), threading.Event()
        original_record = self.host.ledger.record_original_send
        original_begin = self.host.ledger.begin_hold_frame
        def gate(*args):
            result = original_record(*args)
            recorded.set()
            if not release.wait(5):
                raise RuntimeError("Synthetic original receipt gate timed out")
            return result
        def slow_frame_claim(*args):
            result = original_begin(*args)
            self.clock.sleep(31)
            return result
        with patch.object(self.host.ledger, "record_original_send", gate), patch.object(
                self.host.ledger, "begin_hold_frame", slow_frame_claim):
            try:
                self.submit(request)
                self.assertTrue(recorded.wait(5))
                rgb_deadline = self.host.dispatch_rgb_deadline
                cancel = self.service.call("robot_pair_cancel", {"reason": "Explicit test cancellation"})
                original_fault = self.host.ledger.peek_status()["fault"]
                self.assertTrue(cancel["software_cancelled"])
            finally:
                release.set()
            result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "fault")
        receipt = result["receipt"]["device_receipt"]
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157], receipt.get("errors"))
        self.assertEqual(self.ids("left"), [])
        self.assertFalse(receipt["hold_receipt"]["hold_observed"])
        self.assertEqual(receipt["hardware_commands_sent"], 4)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.host.ledger.peek_status()["fault"], original_fault)
        persisted = self.host.ledger.hold_event(receipt["hold_receipt"]["hold_event_id"])
        self.assertEqual(persisted["status"], "complete")
        self.assertEqual(len(persisted["frame_receipts"]), 1)
        self.assertEqual(persisted["frame_receipts"][0]["outcome"], "exception")
        self.assertFalse(persisted["receipt"]["hold_observed"])
        self.assertIsNone(persisted["receipt"]["physical_stop_verified"])
        self.assertGreater(self.host.clock(), rgb_deadline)
