"""Offline host routing for measured-record publication; no device sends."""
import copy
import unittest
from unittest.mock import patch

from robot_tools.joint_sources import JointSourcesError
from robot_tools.pair_host import PairHostError
import test_host_preparation as prep_fixture


class Publisher:
    def __init__(self):
        self.calls = []
        self.hook = lambda: None

    def publish_geometry(self, scene, record_set_id):
        self.calls.append((copy.deepcopy(scene), record_set_id))
        self.hook()
        return {"source": {"path": "synthetic.json", "sha256": "0"*64},
                "hardware_commands_sent": 0, "dispatch_authorized": False,
                "source_truth_authenticated": False, "replayed": len(self.calls) > 1}

    def diagnose(self, scene, arm):
        return {"ready": False, "gaps": [{"code": "synthetic_source_only"}]}


class HostGeometryPublicationTests(unittest.TestCase):
    def setUp(self):
        # Share setup helpers without inheriting/rerunning preparation tests.
        self.fixture = prep_fixture.HostPreparationTests(methodName="runTest")
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.setUp()
        self.host = self.fixture.host
        self.publisher = Publisher()
        self.host.joint_sources_provider = self.publisher
        self.host.open()
        self.scene = self.fixture.scene()
        self.device = self.fixture.devices[-1]

    def publish(self, **overrides):
        args = {"observation_id": self.scene["observation_id"], "record_set_id": "actual-record-id"}
        return self.host.publish_geometry(**{**args, **overrides})

    def test_host_binds_scene_connections_without_tx_budget_or_readiness_changes(self):
        initial = self.host.ledger.status()
        ready = self.host.task_ready
        with patch.object(self.device, "observe", side_effect=AssertionError("No extra device read")):
            result = self.publish()
            replay = self.publish()
        scene, record_id = self.publisher.calls[0]
        self.assertEqual(record_id, "actual-record-id")
        self.assertEqual(scene["observation_id"], self.scene["observation_id"])
        self.assertEqual(scene["joint_source_bindings"], self.host._joint_bindings())
        self.assertTrue(all(p["owner"] == self.host.owner for p in scene["peer_receipts"].values()))
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertFalse(result["dispatch_authorized"])
        self.assertTrue(replay["replayed"])
        self.assertEqual(self.host.task_ready, ready)
        self.assertEqual(self.device.frame_attempts, 0)
        current = self.host.ledger.status()
        for field in ("steps", "started_at", "deadline_s", "owner", "pending_event_id"):
            self.assertEqual(current[field], initial[field])

    def test_missing_actual_record_is_specific_nonfault_zero_tx_failure(self):
        def missing():
            raise JointSourcesError("geometry_records_missing", "No actual installation records")
        self.publisher.hook = missing
        with self.assertRaisesRegex(JointSourcesError, "geometry_records_missing"):
            self.publish()
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual(self.host.ledger.status()["steps"], 0)
        self.assertEqual(self.device.frame_attempts, 0)

    def test_other_scene_stale_scene_or_invalid_id_never_reaches_publisher(self):
        with self.assertRaises(PairHostError):
            self.publish(observation_id="another-owner-scene")
        for value in ("../source", "/tmp/source", "", True, "a"*97):
            with self.assertRaises(ValueError):
                self.publish(record_set_id=value)
        self.fixture.clock.sleep(31)
        with self.assertRaises(PairHostError):
            self.publish()
        self.assertEqual(self.publisher.calls, [])
        self.assertEqual(self.device.frame_attempts, 0)

    def test_pending_and_unresolved_grasp_reject_without_entering_publisher(self):
        self.host.active_event_id = "synthetic-pending"
        try:
            with self.assertRaises(PairHostError):
                self.publish()
        finally:
            self.host.active_event_id = None
        self.device.grasp_states["left"] = {"status": "contact_candidate"}
        try:
            with self.assertRaises(PairHostError):
                self.publish()
        finally:
            self.device.grasp_states["left"] = None
        self.assertEqual(self.publisher.calls, [])

    def test_expiry_during_io_does_not_return_motion_or_fresh_source_permission(self):
        self.publisher.hook = lambda: self.fixture.clock.sleep(31)
        with self.assertRaises(PairHostError):
            self.publish()
        self.assertFalse(self.host.task_ready)
        self.assertEqual(self.device.frame_attempts, 0)
        self.assertEqual(self.host.ledger.status()["steps"], 0)

    def test_fault_during_publication_prevents_success_response(self):
        self.publisher.hook = lambda: self.host.cancel("offline injected fault", allow_hold=False)
        with self.assertRaises(Exception):
            self.publish()
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.device.frame_attempts, 0)

    def test_expiry_during_final_diagnosis_cannot_return_a_current_scene(self):
        def slow_diagnostic(scene, arm):
            self.fixture.clock.sleep(31)
            return {"ready": True, "gaps": []}
        self.publisher.diagnose = slow_diagnostic
        with self.assertRaises(PairHostError):
            self.publish()
        self.assertEqual(self.device.frame_attempts, 0)
        self.assertEqual(self.host.ledger.status()["steps"], 0)


if __name__ == "__main__":
    unittest.main()
