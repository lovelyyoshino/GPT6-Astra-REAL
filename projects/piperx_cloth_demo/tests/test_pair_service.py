"""Service/stdio integration with fake pair hosts; all device entrypoints forbidden."""
from contextlib import nullcontext
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_tools import server
from robot_tools.pair_ledger import PairLedger
from robot_tools.service import TOOL_SCHEMAS, ToolService
from test_backend import PROFILE


OPEN = {"run_id": "offline-test", "task_id": "plug_transfer_left",
        "workspace_clearance_statement": "Current test workspace is clear"}
STEP = {"event_id": "once-1", "observation_id": "fake-scene",
        "peer_receipt_id": "fake-peer", "arm": "left", "kind": "move", "operation": "approach"}
POSE = [.2, .1, .3, 0., 0., 0.]


class FakePairHost:
    def __init__(self):
        self.events = []
        self.active_event_id = None

    def open(self):
        self.events.append(("open",))
        return {"status": "owned", "physical_stop_verified": None}

    def observe(self, rgb, *, saved_rgb_evidence=None):
        self.events.append(("observe", copy.deepcopy(rgb)))
        self.saved_rgb_evidence = copy.deepcopy(saved_rgb_evidence)
        return {"observation_id": "fake-scene", "peer_receipts": {}}

    def submit(self, *args):
        self.events.append(("submit", copy.deepcopy(args)))
        return {"status": "pending", "event_id": args[0]}

    def prepare_gripper(self, *args):
        self.events.append(("prepare_gripper", copy.deepcopy(args)))
        return {"status": "pending", "event_id": args[0]}

    def inspect_joint_limits(self, event_id):
        self.events.append(("inspect_joint_limits", event_id))
        return {"status": "pending", "event_id": event_id}

    def publish_geometry(self, observation_id, record_set_id):
        self.events.append(("publish_geometry", observation_id, record_set_id))
        return {"hardware_commands_sent": 0, "dispatch_authorized": False}

    def promote_ready(self):
        self.events.append(("promote_ready",))
        return {"status": "preparation_required", "task_ready": False,
                "fault_latched": False, "hardware_commands_sent": 0}

    def status(self, event_id=None):
        self.events.append(("status", event_id))
        return {"status": "owned", "physical_stop_verified": None}

    def read_state(self):
        self.events.append(("read_state",))
        return {"state": "same-host-fake-feedback", "hardware_commands_sent": 0}

    def cancel(self, reason, *, allow_hold=True):
        self.last_cancel_allows_hold = allow_hold
        self.events.append(("cancel", reason))
        return {"fault_latched": True, "physical_stop_verified": None}

    def wait(self, event_id, timeout=5):
        self.events.append(("wait", event_id, timeout))
        self.active_event_id = None
        return {"status": "fault", "physical_stop_verified": None}

    def close(self):
        self.events.append(("close",))
        return {"status": "closed", "physical_stop_verified": None}


class PairServiceTests(unittest.TestCase):
    def setUp(self):
        for target in ("socket.socket", "robot_tools.arms._load_sdk",
                       "robot_tools.cameras.capture_cameras"):
            blocker = patch(target, side_effect=AssertionError("Real devices/cameras forbidden"))
            blocker.start()
            self.addCleanup(blocker.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.workspace = Path(self.directory.name) / "workspace"
        self.root = self.workspace / "projects" / "piperx_cloth_demo"
        (self.root / "configs").mkdir(parents=True)
        (self.root / "runs").mkdir()
        profile = copy.deepcopy(PROFILE)
        profile["cameras"] = {"front": "fake-front", "left_wrist": "fake-left", "right_wrist": "fake-right"}
        (self.root / "configs" / "robot.json").write_text(json.dumps(profile))
        self.service = ToolService(self.root)
        self.host = FakePairHost()

    def bind_host(self):
        self.service.persistent = True
        self.service.pair_host = self.host

    def test_geometry_import_uses_owned_host_and_rejects_numeric_or_path_inputs(self):
        args = {"observation_id": "fake-scene", "record_set_id": "measured-installation"}
        with self.assertRaisesRegex(RuntimeError, "No persistent pair host"):
            self.service.call("robot_pair_publish_geometry", args)
        self.bind_host()
        receipt = self.service.call("robot_pair_publish_geometry", args)
        self.assertEqual(receipt["hardware_commands_sent"], 0)
        self.assertFalse(receipt["dispatch_authorized"])
        self.assertEqual(self.host.events, [("publish_geometry", "fake-scene", "measured-installation")])
        for extra in ({"bounds": {}}, {"owner": "other"}, {"verified": True}):
            with self.assertRaises(ValueError):
                self.service.call("robot_pair_publish_geometry", {**args, **extra})
        for record_id in ("../site", "/tmp/site", "", "a"*97):
            with self.assertRaises(ValueError):
                self.service.call("robot_pair_publish_geometry", {**args, "record_set_id": record_id})
        self.assertEqual(len(self.host.events), 1)

    def test_pair_tools_have_strict_schemas_and_no_caller_hold_or_contact_flags(self):
        specs = {s["name"]: s for s in TOOL_SCHEMAS if s["name"].startswith("robot_pair_")}
        self.assertEqual(set(specs), {"robot_pair_" + suffix for suffix in
                         ("open", "observe", "submit_once", "retain_grasp", "confirm_release", "status", "cancel", "close",
                          "prepare_gripper", "inspect_joint_limits", "promote_ready",
                          "initialize_joint_target", "publish_geometry", "confirm_loaded_response",
                          "recover_supported_gripper", "confirm_recovery_release", "observe_supported_contact")})
        for spec in specs.values():
            self.assertFalse(spec["inputSchema"]["additionalProperties"])
        for field in ("peer_held", "contact_step_supported", "physical_stop_verified",
                      "gripper_contact_observation", "grasp_verified", "arrival_confirmed"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.service.call("robot_pair_submit_once", {**STEP, "target_pose_m_rad": POSE, field: True})

    def test_one_shot_pair_open_rejects_before_constructing_device_host(self):
        output = io.StringIO()
        with patch("robot_tools.pair_host.PairHost") as constructor, \
                patch("robot_tools.service.ToolService", return_value=self.service), \
                patch.object(server, "_protocol_stdout", return_value=nullcontext(output)):
            code = server.main(["--root", str(self.root), "--call", "robot_pair_open",
                                "--arguments", json.dumps(OPEN)])
        self.assertEqual(code, 2)
        self.assertFalse(json.loads(output.getvalue())["ok"])
        self.assertIn("long-running", json.loads(output.getvalue())["error"])
        constructor.assert_not_called()
        self.assertIsNone(self.service.pair_host)

    def test_explicit_budget_schema_reaches_host_but_does_not_grant_a_new_epoch(self):
        self.service.persistent = True
        args = {**OPEN, "max_steps":1000, "max_duration_s":10800}
        with patch("robot_tools.pair_host.PairHost", return_value=self.host) as constructor:
            self.service.call("robot_pair_open", args)
        self.assertEqual(constructor.call_args.args[4:6], (1000,10800))
        self.service.pair_host = None
        with self.assertRaisesRegex(ValueError, "explicitly activated"):
            self.service.call("robot_pair_open", args)
        self.assertIsNone(self.service.pair_host)
        self.assertFalse((self.root/"runs/pair_sessions.sqlite").exists())
        for changed in ({"max_steps":1001}, {"max_duration_s":10801}):
            with patch("robot_tools.pair_host.PairHost") as constructor, self.assertRaises(ValueError):
                self.service.call("robot_pair_open", {**args, **changed})
            constructor.assert_not_called()

    def test_long_running_service_keeps_one_host_and_frozen_task_roles(self):
        self.service.persistent = True
        with patch("robot_tools.pair_host.PairHost", return_value=self.host) as constructor:
            self.service.call("robot_pair_open", OPEN)
            self.service.call("robot_pair_status", {})
            with self.assertRaises(RuntimeError):
                self.service.call("robot_pair_open", {**OPEN, "run_id": "other-run"})
        constructor.assert_called_once()
        task = constructor.call_args.args[3]
        self.assertEqual(task["roles"], {"left": "task", "right": "task"})
        self.assertEqual(task["site_context"]["workspace_clearance"]["statement"],
                         OPEN["workspace_clearance_statement"])
        self.assertIs(self.service.pair_host, self.host)
        self.assertEqual(self.host.events, [("open",), ("status", None)])
        self.assertEqual(constructor.call_args.kwargs["connection_mode"], "ready")
        from robot_tools.joint_sources import JointSourcesProvider
        self.assertIsInstance(constructor.call_args.kwargs["joint_sources_provider"], JointSourcesProvider)

    def test_prepare_connection_is_explicit_and_installs_only_an_internal_provider(self):
        self.service.persistent = True
        with patch("robot_tools.pair_host.PairHost", return_value=self.host) as constructor:
            self.service.call("robot_pair_open", {**OPEN, "connection_mode": "prepare"})
        self.assertEqual(constructor.call_args.kwargs["connection_mode"], "prepare")
        provider = constructor.call_args.kwargs["joint_sources_provider"]
        self.assertEqual(provider.run_id, OPEN["run_id"])
        self.assertEqual(provider.runs, self.service.runs)
        self.assertFalse(provider.directory.exists())  # Construction invents no evidence.
        for fields in ({"connection_mode": "force"}, {"task_ready": True},
                       {"joint_sources_provider": {}}, {"geometry": {}}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.service.call("robot_pair_open", {**OPEN, **fields})

    def test_preparation_tools_use_the_current_host_without_legacy_calls(self):
        self.bind_host()
        request = {"event_id": "prepare-1", "observation_id": "current-scene", "arm": "left",
                   "empty_jaw_observation": "Current saved RGB shows no object between the fingers"}
        with patch.object(self.service, "prepare_gripper", side_effect=AssertionError("No legacy reconnection")), \
             patch.object(self.service, "inspect_joint_limits", side_effect=AssertionError("No legacy query")):
            self.assertEqual(self.service.call("robot_pair_prepare_gripper", request)["event_id"], "prepare-1")
            self.assertEqual(self.service.call("robot_pair_inspect_joint_limits", {"event_id": "limits-1"})["event_id"], "limits-1")
            promotion = self.service.call("robot_pair_promote_ready", {})
        self.assertEqual(self.host.events, [
            ("prepare_gripper", ("prepare-1", "current-scene", "left", request["empty_jaw_observation"])),
            ("inspect_joint_limits", "limits-1"), ("promote_ready",)])
        self.assertEqual(promotion["status"], "preparation_required")
        self.assertFalse(promotion["fault_latched"])
        self.assertFalse(promotion["task_ready"])

    def test_preparation_schemas_cannot_supply_width_force_sources_or_verified_flags(self):
        self.bind_host()
        request = {"event_id": "prepare-1", "observation_id": "scene", "arm": "right",
                   "empty_jaw_observation": "Empty in current RGB"}
        for field, value in (("width_m", .01), ("nominal_force_N", .2), ("empty_verified", True),
                             ("physical_stop_verified", True), ("peer_held", True)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.service.call("robot_pair_prepare_gripper", {**request, field: value})
        for name, args in (("robot_pair_inspect_joint_limits", {"event_id": "limits", "capture": {}}),
                           ("robot_pair_promote_ready", {"task_ready": True})):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.service.call(name, args)
        self.assertEqual(self.host.events, [])

    def test_submit_requires_exactly_the_target_matching_action_kind(self):
        self.bind_host()
        invalid = ({}, {"width_m": .01}, {"target_pose_m_rad": POSE, "width_m": .01},
                   {"kind": "gripper", "target_pose_m_rad": POSE})
        for fields in invalid:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.service.call("robot_pair_submit_once", {**STEP, **fields})
        self.assertEqual(self.host.events, [])
        self.service.call("robot_pair_submit_once", {**STEP, "target_pose_m_rad": POSE})
        self.service.call("robot_pair_submit_once", {**STEP, "kind": "gripper", "width_m": .02})
        self.assertEqual(self.host.events[0], ("submit", ("once-1", "fake-scene", "fake-peer",
                         "left", "move", POSE, "approach")))
        self.assertEqual(self.host.events[1], ("submit", ("once-1", "fake-scene", "fake-peer",
                         "left", "gripper", .02, "approach")))

    def test_joint_target_schema_does_not_accept_source_or_permission_inputs(self):
        self.bind_host()
        request = {**STEP, "kind": "joint", "target_joints_rad": [0., .3, -.3, 0., 0., 0.]}
        result = self.service.call("robot_pair_submit_once", request)
        self.assertEqual(result["status"], "pending")
        self.assertEqual(self.host.events[-1][1][4:6], ("joint", request["target_joints_rad"]))
        for extra in ({"width_m": .02}, {"target_pose_m_rad": POSE}, {"context": {}},
                      {"joint_sources_provider": {}}, {"hold_verified": True}):
            with self.assertRaises((ValueError, TypeError)):
                self.service.call("robot_pair_submit_once", {**request, **extra})

    def test_coarse_profile_public_schema_forwards_only_explicit_semantic_fields(self):
        self.bind_host()
        request = {**STEP, "kind": "joint", "target_joints_rad": [0., .3, -.3, 0., 0., 0.],
            "admission_mode": "rgb_supervised", "motion_profile": "coarse_approach",
            "unloaded_observation": "Both jaws currently empty", "corridor_observation": "Whole-arm corridor visible",
            "far_from_target_observation": "Both arms remain far from contact"}
        with patch.object(self.host, "submit", return_value={"status": "pending"}) as submit:
            self.assertEqual(self.service.call("robot_pair_submit_once", request)["status"], "pending")
            self.assertEqual(submit.call_args.kwargs, {key: request[key] for key in
                ("admission_mode", "motion_profile", "unloaded_observation", "corridor_observation",
                 "far_from_target_observation")})
            for extra in ({"motion_profile": "default"}, {"motion_profile": True},
                          {"far_from_target_observation": ""}, {"far_from_target_observation": True},
                          {"coarse_verified": True}, {"max_joint_change_rad": .1}, {"geometry": {}}):
                with self.subTest(extra=extra), self.assertRaises(ValueError):
                    self.service.call("robot_pair_submit_once", {**request, **extra})
            self.assertEqual(submit.call_count, 1)

    def open_contact_service(self):
        """Exercise the real host/service boundary with a receipt-only fake adapter."""
        from robot_tools.pair_host import PairHost
        from test_pair_host import FakeContactPairDevice, InjectableClock
        clock, devices = InjectableClock(), []
        for arm in self.service.profile["arms"].values():
            arm["model"] = "piper"
        def device_factory(profile, journal, guard):
            device = FakeContactPairDevice(profile, journal, guard, clock)
            devices.append(device)
            return device
        def host_factory(*args, **kwargs):
            return PairHost(*args, **kwargs, device_factory=device_factory,
                            clock=clock.time, background=False)
        self.service.persistent = True
        with patch("robot_tools.pair_host.PairHost", side_effect=host_factory):
            self.service.call("robot_pair_open", OPEN)
        host = self.service.pair_host
        self.addCleanup(host.close)
        directory = self.workspace / "artifacts" / "contact-service-rgb"
        directory.mkdir(parents=True)
        frame = [0]
        def observe():
            frame[0] += 1
            rgb = {"capture_id": "contact-fixture-" + str(frame[0]), "cameras": {}}
            for view, key in (("front", "front"), ("left_hand", "left_wrist"),
                              ("right_hand", "right_wrist")):
                png = directory / (view + ".png")
                png.write_bytes(b"synthetic service fixture; not visual evidence")
                rgb["cameras"][view] = {"rgb_path": str(png), "frame_number": frame[0],
                    "serial": self.service.profile["cameras"][key], "host_received_at": clock.time()}
            metadata = directory / "observation.json"
            metadata.write_text(json.dumps(rgb))
            return self.service.call("robot_pair_observe", {"rgb_observation_path": str(metadata)})
        return host, devices[0], observe

    def contact_step(self, scene, event="service-probe", operation="grip_supported", width=.0355):
        return {"event_id": event, "observation_id": scene["observation_id"],
                "peer_receipt_id": scene["peer_receipts"]["right"]["receipt_id"],
                "arm": "left", "kind": "gripper", "operation": operation, "width_m": width}

    def test_service_contact_candidate_remains_observation_until_explicit_measured_release(self):
        host, device, observe = self.open_contact_service()
        probe = self.contact_step(observe())
        self.service.call("robot_pair_submit_once", probe)
        self.assertEqual(host.wait(probe["event_id"])["status"], "completed")
        result = self.service.call("robot_pair_status", {"event_id": probe["event_id"]})
        self.assertEqual(result["receipt"]["contact_observation"]["outcome"], "settled_contact_candidate")
        self.assertFalse(result["receipt"]["arrival_confirmed"])
        self.assertFalse(result["receipt"]["grasp_verified"])
        self.assertIsNone(result["receipt"]["physical_stop_verified"])
        state = self.service.call("robot_pair_status", {})
        self.assertEqual(state["unresolved_gripper_probe"]["arm"], "left")
        scene = observe()
        for operation in ("extract_segment", "insert_segment", "approach"):
            request = {**self.contact_step(scene, event=operation, operation=operation),
                       "kind": "move", "target_pose_m_rad": POSE}
            del request["width_m"]
            with self.subTest(operation=operation), self.assertRaises(RuntimeError):
                self.service.call("robot_pair_submit_once", request)
        self.assertEqual(device.frame_attempts, 1)
        release = self.contact_step(scene, event="service-release", operation="release_retreat", width=.043)
        self.service.call("robot_pair_submit_once", release)
        released = host.wait(release["event_id"])
        self.assertEqual(released["status"], "completed")
        self.assertGreater(released["receipt"]["actual_opening_increase_m"], .0005)
        self.assertIsNone(self.service.call("robot_pair_status", {})["unresolved_gripper_probe"])
        self.assertEqual([call["api"] for call in device.calls],
                         ["execute_gripper_probe", "release_gripper_probe"])

    def test_explicit_close_after_candidate_keeps_durable_fault_and_blocks_legacy(self):
        host, device, observe = self.open_contact_service()
        probe = self.contact_step(observe())
        self.service.call("robot_pair_submit_once", probe)
        self.assertEqual(host.wait(probe["event_id"])["status"], "completed")
        closed = self.service.call("robot_pair_close", {})
        self.assertTrue(closed["fault_latched"])
        self.assertIsNone(closed["physical_stop_verified"])
        self.assertIsNone(self.service.pair_host)
        with patch.object(self.service, "single_arm_move_once") as move:
            with self.assertRaises(RuntimeError):
                self.service.call("robot_single_arm_move_once", {"arm": "right", "target_pose_m_rad": POSE})
        move.assert_not_called()
        self.assertEqual(device.frame_attempts, 1)
        self.assertEqual(device.closes, 1)

    def test_stdio_eof_after_candidate_never_sends_automatic_jaw_release(self):
        host, device, observe = self.open_contact_service()
        probe = self.contact_step(observe())
        self.service.call("robot_pair_submit_once", probe)
        self.assertEqual(host.wait(probe["event_id"])["status"], "completed")
        with patch("robot_tools.service.ToolService", return_value=self.service), \
                patch.object(server, "_protocol_stdout", return_value=nullcontext(io.StringIO())), \
                patch.object(server.sys, "stdin", io.StringIO("")):
            server.main(["--root", str(self.root)])
        self.assertTrue(host.status()["fault_latched"])
        self.assertIsNone(host.status()["physical_stop_verified"])
        self.assertIsNotNone(device.unresolved_gripper_probe)
        self.assertEqual([call["api"] for call in device.calls], ["execute_gripper_probe"])
        self.assertEqual(device.frame_attempts, 1)
        self.assertEqual(device.closes, 1)

    def test_failed_open_retains_host_for_status_and_resource_cleanup(self):
        self.service.persistent = True
        with patch("robot_tools.pair_host.PairHost", return_value=self.host), \
                patch.object(self.host, "open", side_effect=RuntimeError("Device open failed")):
            with self.assertRaises(RuntimeError):
                self.service.call("robot_pair_open", OPEN)
        self.assertIs(self.service.pair_host, self.host)
        self.service.call("robot_pair_status", {})
        self.service.call("robot_pair_close", {})
        self.assertEqual(self.host.events, [("status", None), ("close",)])
        self.assertIsNone(self.service.pair_host)

    def test_active_pair_blocks_legacy_commands_and_reuses_host_feedback(self):
        self.bind_host()
        with patch.object(self.service, "single_arm_move_once") as move, \
                patch.object(self.service, "startup_arms") as startup, \
                patch.object(self.service, "_read_arms") as reconnect:
            for name, args in (("robot_single_arm_move_once", {"arm": "left", "target_pose_m_rad": POSE}),
                               ("robot_startup_arms", {})):
                with self.subTest(tool=name), self.assertRaises(RuntimeError):
                    self.service.call(name, args)
            result = self.service.call("robot_read_state", {})
        move.assert_not_called()
        startup.assert_not_called()
        reconnect.assert_not_called()
        self.assertEqual(result["state"], "same-host-fake-feedback")

    def test_saved_rgb_metadata_is_read_without_cameras_or_second_host(self):
        self.bind_host()
        directory = self.workspace / "artifacts" / "synthetic-rgb"
        directory.mkdir(parents=True)
        rgb = {"capture_id": "synthetic-capture", "cameras": {}}
        for camera in ("front", "left_hand", "right_hand"):
            image = directory / (camera + ".png")
            image.write_bytes(b"synthetic fixture, not visual evidence")
            rgb["cameras"][camera] = {"rgb_path": str(image), "frame_number": 1}
        path = directory / "observation.json"
        path.write_text(json.dumps(rgb))
        with patch("robot_tools.pair_host.PairHost") as constructor:
            result = self.service.call("robot_pair_observe", {"rgb_observation_path": str(path)})
        constructor.assert_not_called()
        self.assertEqual(self.host.events, [("observe", rgb)])
        self.assertEqual(result["observation_id"], "fake-scene")
        self.assertEqual(result["rgb_metadata_path"], str(path))
        self.assertEqual(len(result["rgb_metadata_sha256"]), 64)

    def test_metadata_hash_binds_bytes_used_when_recorder_updates_during_observe(self):
        self.bind_host()
        directory = self.workspace / "artifacts" / "updating-rgb"
        directory.mkdir(parents=True)
        path = directory / "observation.json"
        metadata = json.dumps({"capture_id": "earlier-fixture", "cameras": {}}).encode()
        path.write_bytes(metadata)
        def observe(rgb, *, saved_rgb_evidence=None):
            self.assertEqual(rgb["capture_id"], "earlier-fixture")
            path.write_text(json.dumps({"capture_id": "new-fixture", "cameras": {}}))
            return {"observation_id": "bound-earlier-fixture"}
        with patch.object(self.host, "observe", side_effect=observe):
            result = self.service.call("robot_pair_observe", {"rgb_observation_path": str(path)})
        self.assertEqual(result["rgb_metadata_sha256"], hashlib.sha256(metadata).hexdigest())
        self.assertNotEqual(result["rgb_metadata_sha256"], hashlib.sha256(path.read_bytes()).hexdigest())

    def persistent_state_blocks_legacy(self, mode):
        now = [100.]
        ledger = PairLedger(self.service.runs / "pair_sessions.sqlite", "other-process",
                            {"scope": "synthetic service exclusion test"}, clock=lambda: now[0])
        if mode == "fault":
            # A durable budget fault with no owner exercises the independent latch branch.
            now[0] += 901.
            self.assertTrue(ledger.status()["fault_latched"])
            self.assertIsNone(ledger.status()["global_owner"])
        else:
            ledger.claim("other-owner")
        if mode == "pending":
            ledger.begin("other-owner", "uncertain-once", {"synthetic_target": True})
        with patch.object(self.service, "single_arm_move_once") as move:
            with self.assertRaises(RuntimeError):
                self.service.call("robot_single_arm_move_once", {"arm": "left", "target_pose_m_rad": POSE})
        move.assert_not_called()
        self.assertIsNone(self.service.pair_host)

    def test_another_persistent_owner_blocks_legacy_without_local_host(self):
        self.persistent_state_blocks_legacy("owned")

    def test_pending_send_blocks_legacy_after_process_loss(self):
        self.persistent_state_blocks_legacy("pending")

    def test_durable_pair_fault_blocks_legacy_without_local_host(self):
        self.persistent_state_blocks_legacy("fault")

    def test_stdio_eof_latches_pending_pair_and_never_calls_stop(self):
        self.bind_host()
        self.host.active_event_id = "synthetic-pending"
        output = io.StringIO()
        with patch("robot_tools.service.ToolService", return_value=self.service), \
                patch.object(server, "_protocol_stdout", return_value=nullcontext(output)), \
                patch.object(server.sys, "stdin", io.StringIO("")):
            server.main(["--root", str(self.root)])
        self.assertTrue(self.service.persistent)
        self.assertEqual([event[0] for event in self.host.events], ["cancel", "wait", "close"])
        self.assertIn("disconnected", self.host.events[0][1])
        self.assertFalse(self.host.last_cancel_allows_hold)


if __name__ == "__main__":
    unittest.main()
