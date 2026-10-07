"""Offline native-SDK ingress integration; no manually seeded target cache.

Only the inherited CAN/time/RGB/feedback and source geometry fixtures are fake.
All target history below is produced by actual host/device initialization and
native SDK encoders. This does not certify physical paths or object handling.
"""
import copy
import json
import math
import threading
import unittest

from robot_tools import joint_path
from robot_tools.bounded_joint_step import STEP
from robot_tools.pair_host import PairHost
from test_pair_host import TASK
import test_host_initialization_integration as initialization_fixture


class HostIngressIntegrationTests(unittest.TestCase):
    # Reuse fixture functions without inheriting/rerunning its test methods.
    setUp = initialization_fixture.HostInitializationIntegrationTests.setUp
    check_async_errors = initialization_fixture.HostInitializationIntegrationTests.check_async_errors
    snapshot = initialization_fixture.HostInitializationIntegrationTests.snapshot
    sources = initialization_fixture.HostInitializationIntegrationTests.sources
    open = initialization_fixture.HostInitializationIntegrationTests.open
    observe = initialization_fixture.HostInitializationIntegrationTests.observe
    request = initialization_fixture.HostInitializationIntegrationTests.request
    submit = initialization_fixture.HostInitializationIntegrationTests.submit
    ids = initialization_fixture.HostInitializationIntegrationTests.ids
    initialize = initialization_fixture.HostInitializationIntegrationTests.initialize

    def prepared_boundary(self):
        history = json.loads(initialization_fixture.SAVED.read_text())["state"]["arms"]
        self.joints = {side: history[side]["joints_rad"][:] for side in self.channels}
        self.jaws = dict.fromkeys(self.channels, False)
        # Actual measurement residuals stay visible through initialization and
        # ingress; the fixture never clips feedback or seeds a target cache.
        self.residual[1:3] = [-.001, .001]
        self.open("prepare")
        owner, deadline = self.host.owner, self.host.deadline
        original_jaws = {side: copy.deepcopy(self.device._preparation.anchor[side]["gripper"])
                         for side in self.channels}
        for side in ("left", "right"):
            peer = "right" if side == "left" else "left"
            peer_count = len(self.ids(peer))
            result = self.initialize(side, "init-"+side)
            self.assertEqual(result["status"], "completed", result.get("receipt"))
            receipt = result["receipt"]
            self.assertEqual(receipt["hardware_commands_sent"], 4)
            self.assertEqual(receipt["passive_arm_commands_sent"], 0)
            self.assertEqual(receipt["gripper_commands_sent"], 0)
            self.assertFalse(receipt["strict_nominal"])
            self.assertTrue(receipt["within_feedback_tolerance"])
            self.assertEqual(receipt["cached_target"]["target_raw"][1:3], [0, 0])
            self.assertEqual(len(self.ids(peer)), peer_count)
        for side in ("left", "right"):
            self.assertEqual(self.device._preparation.anchor[side]["gripper"], original_jaws[side])
            scene = self.observe()
            event = "prepare-"+side
            peer = "right" if side == "left" else "left"
            peer_count = len(self.ids(peer))
            self.host.prepare_gripper(event, scene["observation_id"], side,
                "Offline RGB testimony: selected jaw empty, finger clearance visible")
            result = self.host.wait(event, 10)
            self.assertEqual(result["status"], "completed", result.get("receipt"))
            self.assertEqual(result["receipt"]["hardware_commands_sent"], 1)
            self.assertEqual(len(self.ids(peer)), peer_count)
        self.assertTrue(self.host.promote_ready()["task_ready"])
        self.assertEqual(self.host.ledger.peek_status()["steps"], 4)
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        for side in self.channels:
            self.assertEqual(self.ids(side), [0x151, 0x155, 0x156, 0x157, 0x159])

    def inward_request(self, arm="left", event=None):
        scene = self.observe()
        peer = "right" if arm == "left" else "left"
        current = self.device.joint_binding(arm)["cached_target"]
        target = [math.radians(v/1000) for v in current["target_raw"]]
        inward = math.ceil(STEP["joint_margin_rad"] / joint_path.RAD_PER_RAW) * joint_path.RAD_PER_RAW
        target[1:3] = [inward, -inward]
        return {"event_id": event or "ingress-"+arm, "observation_id": scene["observation_id"],
                "peer_receipt_id": scene["peer_receipts"][peer]["receipt_id"], "arm": arm,
                "kind": "joint", "operation": "approach", "target_joints_rad": target}

    def ordinary_request(self, arm="left", event=None):
        request = self.inward_request(arm, event or "ordinary-"+arm)
        cached = self.device.joint_binding(arm)["cached_target"]
        request["target_joints_rad"] = [math.radians(v/1000) for v in cached["target_raw"]]
        request["target_joints_rad"][5] += .001
        return request

    def dispatch(self, request):
        arm = request["arm"]
        peer = "right" if arm == "left" else "left"
        peer_before = self.ids(peer)
        active_before = len(self.ids(arm))
        previous_cache = self.device.joint_binding(arm)["cached_target"]
        self.assertEqual(self.submit(request)["status"], "pending")
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        receipt = result["receipt"]
        self.assertEqual(receipt["hardware_commands_sent"], 4)
        self.assertEqual(receipt["passive_arm_commands_sent"], 0)
        self.assertEqual(self.ids(peer), peer_before)
        self.assertEqual(self.ids(arm)[active_before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(receipt["joint_path_plan"]["cached_target"], previous_cache)
        self.assertEqual(self.device.joint_binding(arm)["cached_target"]["event_id"], request["event_id"])
        self.assertIsNone(receipt["physical_stop_verified"])
        self.assertIsNone(receipt["accepted"])
        self.assertIsNone(receipt["object_task_success"])
        return receipt

    def test_native_dual_boundary_init_prepare_ingress_then_ordinary_same_owner(self):
        self.prepared_boundary()
        owner, deadline = self.host.owner, self.host.deadline
        for side in ("left", "right"):
            receipt = self.dispatch(self.inward_request(side))
            self.assertFalse(receipt["joint_path_plan"]["hold_reference_within_nominal_limits"])
            self.assertGreater(self.joints[side][1], 0)
            self.assertLess(self.joints[side][2], 0)
        for side in ("left", "right"):
            receipt = self.dispatch(self.ordinary_request(side))
            # Necessary reference fact only: this does not assert an applicable
            # hold was sent/observed, or a physical stop has been established.
            self.assertTrue(receipt["joint_path_plan"]["hold_reference_within_nominal_limits"])
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 8)
        self.assertEqual(len(self.sent), 26)
        self.assertFalse(self.host.fault_event.is_set())

    def test_completed_ingress_replay_and_changed_request_do_not_send(self):
        self.prepared_boundary()
        request = self.inward_request()
        receipt = self.dispatch(request)
        sent = len(self.sent)
        replay = self.submit(request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], receipt)
        changed = copy.deepcopy(request)
        changed["target_joints_rad"][5] += .001
        with self.assertRaises(RuntimeError):
            self.submit(changed)
        self.assertEqual(len(self.sent), sent)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 5)

    def test_missing_initialization_source_refuses_before_claim_without_new_send(self):
        self.prepared_boundary()
        # Simulate lost device provenance; do not invent a target or receipt.
        self.device._joint_initializations["left"] = None
        request = self.inward_request()
        sent = len(self.sent)
        with self.assertRaises(joint_path.JointPathError) as caught:
            self.submit(request)
        self.assertEqual(caught.exception.code, "origin_joint_limit")
        self.assertEqual(len(self.sent), sent)
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 4)

    def test_changed_initialization_event_source_refuses_before_claim(self):
        self.prepared_boundary()
        # A source copied under a different event must not authorize old cache.
        self.device._joint_initializations["left"]["event_id"] = "unrelated-initialization"
        request = self.inward_request()
        sent = len(self.sent)
        with self.assertRaises(joint_path.JointPathError) as caught:
            self.submit(request)
        self.assertEqual(caught.exception.code, "origin_joint_limit")
        self.assertEqual(len(self.sent), sent)
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 4)

    def test_new_owner_has_no_initialization_cache_or_reusable_ingress_source(self):
        self.prepared_boundary()
        old_request = self.inward_request()
        owner = self.host.owner
        self.host.close()
        self.host = PairHost(self.service.runs, self.profile, "first-target-integration", TASK,
            clock=self.clock.time, background=False, joint_sources_provider=self.sources)
        self.service.pair_host = self.host
        self.addCleanup(self.host.close)
        self.host.open()
        self.device = self.host.device
        self.assertNotEqual(self.host.owner, owner)
        self.assertIsNone(self.device.joint_binding("left")["cached_target"])
        self.assertIsNone(self.device._joint_initializations["left"])
        sent = len(self.sent)
        with self.assertRaises(RuntimeError):
            self.submit(old_request)
        scene = self.observe()
        fresh = {**old_request, "event_id": "new-owner-ingress", "observation_id": scene["observation_id"],
                 "peer_receipt_id": scene["peer_receipts"]["right"]["receipt_id"]}
        with self.assertRaises(RuntimeError):
            self.submit(fresh)
        self.assertEqual(len(self.sent), sent)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 4)

    def test_partial_native_ingress_faults_and_replay_never_retries(self):
        self.prepared_boundary()
        request = self.inward_request()
        old_right_cache = self.device.joint_binding("right")["cached_target"]
        before, peer_before = len(self.ids("left")), self.ids("right")
        self.fail_id = 0x156
        self.assertEqual(self.submit(request)["status"], "pending")
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "fault", result)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.ids("left")[before:], [0x151, 0x155])
        self.assertEqual(self.ids("right"), peer_before)
        self.assertIsNone(self.device.joint_binding("left")["cached_target"])
        self.assertEqual(self.device.joint_binding("right")["cached_target"], old_right_cache)
        self.assertTrue(self.submit(request)["replayed"])
        self.assertEqual(self.ids("left")[before:], [0x151, 0x155])
        self.assertEqual(self.host.ledger.peek_status()["steps"], 5)
        self.assertIsNone(result["receipt"]["physical_stop_verified"])

    def test_cancelled_ingress_with_residual_origin_does_not_inherit_hold_permission(self):
        self.prepared_boundary()
        request = self.inward_request()
        before, peer_before = len(self.ids("left")), self.ids("right")
        recorded, release = threading.Event(), threading.Event()
        original_record = self.host.ledger.record_original_send
        def gate(*args):
            result = original_record(*args)
            recorded.set()
            if not release.wait(5):
                raise RuntimeError("Offline original-send gate timed out")
            return result
        self.host.ledger.record_original_send = gate
        try:
            self.assertEqual(self.submit(request)["status"], "pending")
            self.assertTrue(recorded.wait(5))
            # Selected q is now nominal, but this action's original q and its
            # passive peer include a source-bound residual. The unmodified hold
            # contract must reject it, rather than treating ingress as a token.
            self.assertGreater(self.joints["left"][1], 0)
            self.service.call("robot_pair_cancel", {"reason": "Explicit offline ingress cancellation"})
        finally:
            release.set()
        result = self.host.wait(request["event_id"], 10)
        self.assertEqual(result["status"], "fault", result)
        hold = result["receipt"]["device_receipt"]["hold_receipt"]
        self.assertFalse(hold["hold_observed"])
        self.assertFalse(hold["frames_complete"])
        self.assertEqual(hold["frame_attempts"], [])
        self.assertIn("nominal_joint_limit", hold["hold_fault"]["detail"])
        self.assertEqual(self.ids("left")[before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("right"), peer_before)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(hold["physical_stop_verified"])


if __name__ == "__main__":
    unittest.main()
