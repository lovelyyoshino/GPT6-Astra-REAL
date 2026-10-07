"""Offline first-target host/service contracts with a memory-only device.

The real source planner and durable ledger run; feedback/frames here are fake
adapter receipts, never firmware behavior or evidence of physical acceptance.
"""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robot_tools import joint_initialization, joint_path
from robot_tools.hold_transaction import joint_hold_frames
from robot_tools.pair_host import PairHost, PairHostError
from robot_tools.service import TOOL_SCHEMAS, ToolService
from test_backend import PROFILE
from test_execution import Clock
from test_pair_host import FakePairDevice, TASK
import test_host_preparation as preparation_fixture
import test_joint_path as path_fixture


TOOL = "robot_pair_initialize_joint_target"
UNLOADED = "All three current RGB views show both jaws empty and no object contact"


class InitializationDevice(preparation_fixture.PreparationDevice):
    def __init__(self, *args):
        super().__init__(*args)
        self.caches = {"left": None, "right": None}
        self.initialization_hook = lambda: None
        self.result_mutator = lambda result: result
        self.initialization_result = None
        self.origin_error = None
        self.origins = []
        for state in self.states.values():
            state["arm_status"].update(teach_status=0, mode_feedback=0)

    def open(self):
        self.ready = True
        for state in self.states.values():
            state["gripper"]["foc_status"]["driver_enable_status"] = True
        return FakePairDevice.open(self)

    def joint_binding(self, arm):
        return {**super().joint_binding(arm), "cached_target": copy.deepcopy(self.caches[arm])}

    def observe_initialization(self, identity):
        if self.origin_error:
            raise self.origin_error
        sample = self.observe()
        origin = {"sample_id": "origin-"+str(self.observations), "identity": copy.deepcopy(identity),
                  "captured_at": self.clock.time(), "arms": sample["arms"]}
        self.origins.append(copy.deepcopy(origin))
        return origin

    def initialize_joint_target(self, arm, *, context, event_id, deadline_at):
        self.calls.append({"arm": arm, "event_id": event_id, "context": copy.deepcopy(context),
                           "deadline_at": deadline_at})
        self.execution_started.set()
        if self.execution_gate is not None and not self.execution_gate.wait(3):
            raise RuntimeError("Offline initialization gate timed out")
        self.initialization_hook()
        self.guard()
        if self.execute_error:
            raise self.execute_error
        if self.initialization_result is not None:
            self.frame_attempts += self.initialization_result["hardware_commands_sent"]
            return copy.deepcopy(self.initialization_result)
        result = {"ok": True, "cache_established": True, "hardware_commands_sent": 0,
                  "passive_arm_commands_sent": 0, "gripper_commands_sent": 0,
                  "accepted": None, "physical_stop_verified": None}
        if context is None:
            result.update(status="already_initialized", cached_target=copy.deepcopy(self.caches[arm]))
        else:
            plan = joint_initialization.plan_joint_initialization(context, now=self.clock.time())
            rows = []
            for frame in plan["frames"]:
                self.guard()
                self.frame_attempts += 1
                self.clock.sleep(.001)
                rows.append({"frame": copy.deepcopy(frame), "outcome": "returned",
                             "returned_at": self.clock.time()})
            self.states[arm]["joints_rad"] = plan["encoded_target_joints_rad"][:]
            self.states[arm]["arm_status"]["mode_feedback"] = 1
            cache = {"event_id": event_id, "identity": copy.deepcopy(context["identity"]),
                     "target_raw": plan["target_raw"][:], "frame_receipts": rows}
            self.caches[arm] = copy.deepcopy(cache)
            result.update(status="joint_target_initialized", hardware_commands_sent=4,
                          cached_target=cache, frame_receipts=rows, initialization_plan=plan)
        return self.result_mutator(result)


class HostInitializationTests(unittest.TestCase):
    scene = preparation_fixture.HostPreparationTests.scene

    def setUp(self):
        for name in ("socket.socket", "robot_tools.arms._load_sdk", "robot_tools.cameras.capture_cameras"):
            blocker = patch(name, side_effect=AssertionError("Offline test forbids hardware/camera construction"))
            blocker.start()
            self.addCleanup(blocker.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory, self.clock = Path(self.temp.name), Clock()
        self.profile = copy.deepcopy(PROFILE)
        self.profile["cameras"] = {"front": "f", "left_wrist": "l", "right_wrist": "r"}
        for cfg in self.profile["arms"].values():
            cfg["model"] = "piper_x"
        self.frame, self.devices, self.source_calls = 0, [], []
        self.host = self.make_host()

    def sources(self, scene, arm):
        self.source_calls.append((copy.deepcopy(scene), arm))
        ctx = path_fixture.context(arm=arm)
        stable = Path(__file__).resolve().parents[1]/"data/piper_x_official"
        ctx["model_catalog"]["constants_path"] = str(stable/"sdk_constants.py")
        ctx["urdf_source"]["path"] = str(stable/"piper_x_description.urdf")
        return {key: copy.deepcopy(ctx[key]) for key in
                ("model_catalog", "urdf_source", "controller_limits", "geometry")}

    def make_host(self, *, mode="prepare", max_steps=128, max_duration_s=900, run_id="initialize-run"):
        def factory(profile, journal, guard):
            device = InitializationDevice(profile, journal, guard, self.clock)
            self.devices.append(device)
            return device
        host = PairHost(self.directory/"runs", self.profile, run_id, TASK,
                        max_steps=max_steps, max_duration_s=max_duration_s,
                        device_factory=factory, clock=self.clock.time, background=False,
                        connection_mode=mode, joint_sources_provider=self.sources)
        self.addCleanup(host.close)
        return host

    def request(self, event="init-1", arm="right", *, saved=True):
        scene = self.scene(saved=saved)
        return {"event_id": event, "observation_id": scene["observation_id"], "arm": arm,
                "unloaded_observation": UNLOADED}

    def initialize(self, request=None):
        request = request or self.request()
        self.host.initialize_joint_target(**request)
        return self.host.wait(request["event_id"], 5)

    def assert_no_claim(self, event="init-1"):
        self.assertIsNone(self.host.ledger.event(event))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 0)
        self.assertEqual(self.devices[-1].frame_attempts, 0)
        self.assertFalse(self.host.fault_event.is_set())

    def test_prepare_first_target_is_host_bound_four_frames_before_task_readiness(self):
        self.host.open()
        device = self.devices[-1]
        deadline = self.host.deadline
        device.initialization_hook = lambda: self.assertEqual(self.host.ledger.event("init-1")["status"], "pending")
        result = self.initialize()
        self.assertEqual(result["status"], "completed", result)
        receipt = result["receipt"]
        self.assertEqual(receipt["hardware_commands_sent"], 4)
        self.assertIsNone(receipt["accepted"])
        self.assertIsNone(receipt["object_task_success"])
        self.assertIsNone(receipt["physical_stop_verified"])
        self.assertFalse(self.host.task_ready)
        self.assertIsNone(self.host.latest)
        self.assertEqual(device.opens, 1)
        self.assertEqual(self.host.ledger.status()["steps"], 1)
        self.assertEqual(self.host.deadline, deadline)
        payload = self.host.ledger.event("init-1")["payload"]
        ctx = payload["context"]
        self.assertEqual(ctx["identity"]["owner"], self.host.owner)
        self.assertEqual(ctx["identity"]["epoch"], self.host.owner)
        self.assertEqual(ctx["identity"]["worker_id"], "init-1")
        self.assertEqual(ctx["identity"]["connection_id"], device.connection_id)
        self.assertEqual(ctx["origin"], device.origins[0])
        self.assertEqual(ctx["origin_sha256"], joint_path.evidence_sha256(device.origins[0]))
        self.assertEqual(ctx["geometry"]["origin_sample_id"], device.origins[0]["sample_id"])
        self.assertEqual(ctx["unloaded_evidence"]["source"]["sha256"],
                         joint_path.evidence_sha256(payload["unloaded_evidence"]))
        self.assertIn("not_sensor_verification", payload["unloaded_evidence"]["kind"])
        self.assertEqual(payload["peer_receipt"]["arm"], "left")
        self.assertEqual(self.source_calls[0][0]["joint_source_bindings"], payload["bindings"])
        self.assertEqual(receipt["frame_receipts"], device.joint_binding("right")["cached_target"]["frame_receipts"])
        self.assertEqual([row["frame"] for row in receipt["frame_receipts"]],
                         joint_hold_frames(receipt["initialization_plan"]["target_raw"]))

    def test_ready_mode_uses_same_initialization_entry_and_connection(self):
        self.host.close()
        self.host = self.make_host(mode="ready")
        self.host.open()
        self.assertTrue(self.host.task_ready)
        result = self.initialize()
        self.assertEqual(result["status"], "completed", result)
        self.assertTrue(self.host.task_ready)
        self.assertEqual(self.devices[-1].opens, 1)

    def test_real_planner_derives_boundary_target_without_caller_cache(self):
        self.host.open()
        self.devices[-1].states["right"]["joints_rad"][1:3] = [-.002, .001]
        result = self.initialize()
        self.assertEqual(result["status"], "completed", result)
        plan = result["receipt"]["initialization_plan"]
        self.assertEqual(plan["purpose"], "startup_j2_j3")
        self.assertEqual(plan["target_raw"][1:3], [0, 0])
        self.assertEqual(plan["cached_target_prior"], "unknown")
        self.assertFalse(plan["hard_path_guarantee"])

    def test_completed_replay_uses_durable_receipt_and_changed_request_refuses(self):
        self.host.open()
        request = self.request()
        result = self.initialize(request)
        self.assertEqual(result["status"], "completed")
        replay = self.host.initialize_joint_target(**request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], result["receipt"])
        self.assertEqual(len(self.devices[-1].calls), 1)
        with self.assertRaisesRegex(PairHostError, "different request"):
            self.host.initialize_joint_target(**{**request, "unloaded_observation": "changed"})
        self.assertEqual(self.host.ledger.status()["steps"], 1)

    def test_new_owner_can_read_completed_replay_but_does_not_inherit_local_cache(self):
        self.host.open()
        request = self.request()
        result = self.initialize(request)
        old_owner = self.host.owner
        self.host.close()
        self.host = self.make_host()
        self.host.open()
        self.assertNotEqual(self.host.owner, old_owner)
        self.assertTrue(self.host.initialize_joint_target(**request)["replayed"])
        self.assertEqual(self.host.initialize_joint_target(**request)["receipt"], result["receipt"])
        self.assertIsNone(self.devices[-1].joint_binding("right")["cached_target"])
        self.assertEqual(self.devices[-1].frame_attempts, 0)
        self.assertEqual(self.host.ledger.status()["steps"], 1)

    def test_existing_cache_noop_keeps_exact_cache_without_geometry_or_new_scene(self):
        self.host.open()
        self.assertEqual(self.initialize()["status"], "completed")
        before = self.devices[-1].joint_binding("right")["cached_target"]
        self.host.joint_sources_provider = lambda *_: self.fail("No-op must not resolve a new geometry")
        result = self.initialize({"event_id": "noop", "observation_id": "unused-old-scene", "arm": "right",
                                  "unloaded_observation": UNLOADED})
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["receipt"]["status"], "already_initialized")
        self.assertEqual(result["receipt"]["hardware_commands_sent"], 0)
        self.assertEqual(self.devices[-1].joint_binding("right")["cached_target"], before)
        self.assertEqual(self.devices[-1].frame_attempts, 4)
        self.assertEqual(self.host.ledger.status()["steps"], 2)

    def test_missing_provider_or_geometry_is_preclaim_nonfault(self):
        self.host.open()
        request = self.request()
        self.host.joint_sources_provider = None
        with self.assertRaisesRegex(RuntimeError, "host-resolved"):
            self.host.initialize_joint_target(**request)
        self.assert_no_claim()
        def missing(scene, arm):
            raise PairHostError("Current scene geometry source missing")
        self.host.joint_sources_provider = missing
        with self.assertRaisesRegex(RuntimeError, "geometry source missing"):
            self.host.initialize_joint_target(**request)
        self.assert_no_claim()

    def test_missing_changed_and_superseded_saved_scene_refuse_before_claim(self):
        self.host.open()
        request = self.request(saved=False)
        with self.assertRaisesRegex(PairHostError, "saved RGB"):
            self.host.initialize_joint_target(**request)
        request = self.request()
        Path(self.host.latest["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"changed")
        with self.assertRaisesRegex(PairHostError, "artifact/frame changed"):
            self.host.initialize_joint_target(**request)
        request = self.request()
        self.scene()
        with self.assertRaisesRegex(PairHostError, "current host-issued"):
            self.host.initialize_joint_target(**request)
        self.assert_no_claim()

    def test_partial_send_preserves_raw_receipt_fault_and_never_retries(self):
        self.host.open()
        device = self.devices[-1]
        device.initialization_result = {"ok": False, "status": "partial_send", "cache_established": False,
            "hardware_commands_sent": 2, "frame_receipts": [{"frame_index": 2, "outcome": "unknown"}],
            "accepted": None, "physical_stop_verified": None}
        request = self.request()
        result = self.initialize(request)
        self.assertEqual(result["status"], "fault", result)
        self.assertEqual(result["receipt"]["device_receipt"], device.initialization_result)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(device.joint_binding("right")["cached_target"])
        self.assertTrue(self.host.initialize_joint_target(**request)["replayed"])
        self.assertEqual(device.frame_attempts, 2)
        self.assertEqual(len(device.calls), 1)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)

    def test_pending_excludes_new_work_and_cancel_before_send_leaves_zero_frames(self):
        self.host.open()
        device = self.devices[-1]
        device.execution_gate = threading.Event()
        request = self.request()
        self.host.initialize_joint_target(**request)
        self.assertTrue(device.execution_started.wait(2))
        try:
            for call in (lambda: self.host.initialize_joint_target(**{**request, "event_id": "init-2"}),
                         lambda: self.host.inspect_joint_limits("limits"), self.host.promote_ready):
                with self.assertRaises(PairHostError):
                    call()
            self.assertTrue(self.host.initialize_joint_target(**request)["replayed"])
            self.host.cancel("offline explicit cancellation")
        finally:
            device.execution_gate.set()
        result = self.host.wait(request["event_id"], 5)
        self.assertEqual(result["status"], "fault")
        self.assertEqual(device.frame_attempts, 0)
        self.assertIsNone(result["receipt"]["physical_stop_verified"])

    def test_original_budget_applies_even_to_cached_noop(self):
        self.host.close()
        self.host = self.make_host(max_steps=1, run_id="limited-run")
        self.host.open()
        self.assertEqual(self.initialize()["status"], "completed")
        with self.assertRaises(RuntimeError):
            self.host.initialize_joint_target("noop", "unused", "right", UNLOADED)
        self.assertEqual(self.devices[-1].frame_attempts, 4)
        self.assertEqual(len(self.devices[-1].calls), 1)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)
        self.assertTrue(self.host.fault_event.is_set())

    def test_delayed_claim_and_binding_change_block_before_frame(self):
        self.host.open()
        request = self.request()
        begin = self.host.ledger.begin
        def changed(*args, **kwargs):
            claim = begin(*args, **kwargs)
            self.devices[-1].connection_id = "replacement-connection"
            return claim
        with patch.object(self.host.ledger, "begin", side_effect=changed):
            result = self.initialize(request)
        self.assertEqual(result["status"], "fault", result)
        self.assertEqual(self.devices[-1].frame_attempts, 0)
        self.assertEqual(self.devices[-1].calls, [])

    def test_stale_rgb_after_durable_claim_blocks_before_adapter(self):
        self.host.open()
        request = self.request()
        begin = self.host.ledger.begin
        def delayed(*args, **kwargs):
            claim = begin(*args, **kwargs)
            self.clock.sleep(31.)
            return claim
        with patch.object(self.host.ledger, "begin", side_effect=delayed):
            result = self.initialize(request)
        self.assertEqual(result["status"], "fault", result)
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_frozen_run_deadline_is_not_renewed_by_initialization(self):
        self.host.close()
        self.host = self.make_host(max_duration_s=10, run_id="short-run")
        self.host.open()
        request = self.request()
        deadline = self.host.deadline
        self.clock.sleep(11.)
        with self.assertRaises(RuntimeError):
            self.host.initialize_joint_target(**request)
        self.assertEqual(self.host.deadline, deadline)
        self.assertEqual(self.devices[-1].frame_attempts, 0)
        self.assertIsNone(self.host.ledger.event("init-1"))

    def test_host_rejects_report_and_cache_agreeing_on_unplanned_target(self):
        self.host.open()
        device = self.devices[-1]
        def substitute(result):
            bad = copy.deepcopy(result["cached_target"])
            bad["target_raw"][0] += 1
            device.caches["right"] = bad
            result["cached_target"] = copy.deepcopy(bad)
            return result
        device.result_mutator = substitute
        result = self.initialize()
        self.assertEqual(result["status"], "fault", result)
        self.assertEqual(result["receipt"]["device_receipt"]["hardware_commands_sent"], 4)
        self.assertTrue(self.host.fault_event.is_set())

    def test_preclaim_device_feedback_failure_latches_without_claim(self):
        self.host.open()
        request = self.request()
        self.devices[-1].origin_error = RuntimeError("Lost independent peer feedback")
        with self.assertRaisesRegex(RuntimeError, "peer feedback"):
            self.host.initialize_joint_target(**request)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(self.host.ledger.event("init-1"))
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_current_grasp_prevents_initialization_without_consuming_budget(self):
        self.host.open()
        request = self.request()
        self.devices[-1].grasp_states["left"] = {"status": "contact_candidate"}
        with self.assertRaisesRegex(PairHostError, "unresolved or retained grasp"):
            self.host.initialize_joint_target(**request)
        self.assert_no_claim()

    def test_service_exact_schema_and_same_host_route(self):
        root = self.directory/"service"
        (root/"configs").mkdir(parents=True)
        (root/"configs/robot.json").write_text(json.dumps(self.profile))
        service = ToolService(root)
        service.persistent, service.pair_host = True, self.host
        self.host.open()
        request = self.request()
        schema = next(item["inputSchema"] for item in TOOL_SCHEMAS if item["name"] == TOOL)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(set(schema["properties"]), set(request) | {"admission_mode", "corridor_observation"})
        self.assertEqual(set(schema["required"]), set(request))
        for key in ("cache", "cached_target", "target_joints_rad", "context", "geometry", "controller_limits",
                    "verified", "unloaded_verified", "accepted", "physical_stop_verified"):
            with self.subTest(field=key), self.assertRaises(ValueError):
                service.call(TOOL, {**request, key: True})
        self.assert_no_claim()
        self.assertEqual(service.call(TOOL, request)["status"], "pending")
        self.assertEqual(self.host.wait("init-1", 5)["status"], "completed")
        self.assertTrue(service.call(TOOL, request)["replayed"])
        self.assertEqual(self.devices[-1].opens, 1)
        self.assertEqual(self.devices[-1].frame_attempts, 4)


if __name__ == "__main__":
    unittest.main()
