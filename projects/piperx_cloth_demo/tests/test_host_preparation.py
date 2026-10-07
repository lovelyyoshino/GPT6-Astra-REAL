"""Offline persistent preparation, real ledger and actual adapter integration."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack
from unittest.mock import patch

from robot_tools import arms, linear_hold, pair_device, pair_preparation, pair_limits
from robot_tools import single_supervised_actions, supervised_actions, takeover
from robot_tools.pair_host import PairHost, PairHostError
from test_backend import PROFILE
from test_execution import Clock
from test_pair_host import FakePairDevice, TASK
from test_single_gripper_prepare import SingleGripperFixture
from test_pair_limits import QueryRobot
from test_joint_limits import reply_bytes
from test_execution import healthy_arm


class PreparationDevice(FakePairDevice):
    def __init__(self, *args):
        super().__init__(*args)
        self.ready = False
        self.prepare_hook = lambda: None
        self.preparation_result = None
        self.grasp_states = {"left": None, "right": None}
        self.connection_id = "offline-test-connection"
        for state in self.states.values():
            state["gripper"]["foc_status"]["driver_enable_status"] = False

    def connect_for_preparation(self):
        return super().open()

    def open(self):
        raise AssertionError("prepare mode must not invoke ready-only device.open")

    def observe(self):
        sample = super().observe()
        sample.update(task_ready=self.ready, readiness={side: {"enable_flags":
            [True]*6 + [self.states[side]["gripper"]["foc_status"]["driver_enable_status"]]}
            for side in self.states})
        return sample

    def prepare_gripper(self, arm):
        self.prepare_hook()
        self.execution_started.set()
        if self.execution_gate is not None and not self.execution_gate.wait(2):
            raise RuntimeError("test preparation gate timed out")
        self.guard()
        if self.execute_error is not None:
            raise self.execute_error
        if self.preparation_result is not None:
            return copy.deepcopy(self.preparation_result)
        enabled = self.states[arm]["gripper"]["foc_status"]["driver_enable_status"]
        if not enabled:
            self.calls.append(("prepare_gripper", arm))
            self.frame_attempts += 1
            self.states[arm]["gripper"]["foc_status"]["driver_enable_status"] = True
        return {"ok": True, "status": "already_enabled_observed" if enabled else "selected_gripper_prepared_not_task_ready",
                "hardware_commands_sent": 0 if enabled else 1, "task_ready": False,
                "gripper_enable_commands_sent": 0 if enabled else 1, "passive_arm_commands_sent": 0,
                "arm_target_commands_sent": 0, "mode_commands_sent": 0, "enable_commands_sent": 0,
                "grasp_verified": False, "physical_stop_verified": None, "accepted": None}

    def promote_ready(self):
        if not all(s["gripper"]["foc_status"]["driver_enable_status"] for s in self.states.values()):
            return {"ok": False, "status": "preparation_required", "requirements": ["jaws_enabled"],
                    "hardware_commands_sent": 0, "fault_latched": False, "task_ready": False,
                    "readiness": self.observe()["readiness"]}
        self.ready = True
        return self.observe()

    def joint_binding(self, arm):
        cfg = self.profile["arms"][arm]
        return {"connection_id": self.connection_id, "model": cfg["model"],
                "firmware_profile": cfg["firmware"], "cached_target": None}


class HostPreparationTests(unittest.TestCase):
    def setUp(self):
        self.socket = patch("socket.socket", side_effect=AssertionError("No hardware sockets"))
        self.socket.start()
        self.addCleanup(self.socket.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory, self.clock = Path(self.temp.name), Clock()
        self.profile = copy.deepcopy(PROFILE)
        self.profile["cameras"] = {"front": "f", "left_wrist": "l", "right_wrist": "r"}
        self.frame = 0
        self.devices = []
        self.host = self.make_host()

    def make_host(self, **kwargs):
        def factory(profile, journal, guard):
            device = PreparationDevice(profile, journal, guard, self.clock)
            self.devices.append(device)
            return device
        host = PairHost(self.directory/"runs", self.profile, "prepare-run", TASK,
                        device_factory=factory, clock=self.clock.time, background=False,
                        connection_mode="prepare", **kwargs)
        self.addCleanup(host.close)
        return host

    def scene(self, saved=True):
        self.frame += 1
        self.clock.sleep(.01)
        rgb = {"capture_id": "capture-"+str(self.frame), "cameras": {}}
        evidence = {}
        for view, key in (("front", "front"), ("left_hand", "left_wrist"), ("right_hand", "right_wrist")):
            raw = b"\x89PNG\r\n\x1a\n" + (view+str(self.frame)).encode()
            path = self.directory/(view+str(self.frame)+".png")
            path.write_bytes(raw)
            rgb["cameras"][view] = {"serial": self.profile["cameras"][key], "frame_number": self.frame,
                "host_received_at": self.clock.time(), "depth_enabled": False}
            evidence[view] = {"rgb_path": str(path), "artifact_sha256": hashlib.sha256(raw).hexdigest(),
                             "frame_number": self.frame, "host_received_at": self.clock.time()}
        return self.host.observe(rgb, saved_rgb_evidence=evidence if saved else None)

    def prepare(self, arm="right", event="prepare-1"):
        scene = self.scene()
        request = dict(event_id=event, observation_id=scene["observation_id"], arm=arm,
                       empty_jaw_observation="Current RGB shows the selected jaw empty with finger clearance")
        return request, self.host.prepare_gripper(**request)

    def test_prepare_connection_is_readable_but_submit_and_retain_are_preclaim_refused(self):
        opened = self.host.open()
        self.assertFalse(opened["task_ready"])
        self.assertEqual(opened["connection_mode"], "prepare")
        self.assertTrue(self.host.read_state()["ok"])
        scene = self.scene()
        with self.assertRaisesRegex(PairHostError, "task readiness"):
            self.host.submit("motion", scene["observation_id"], scene["peer_receipts"]["left"]["receipt_id"],
                             "right", "gripper", .03)
        with self.assertRaisesRegex(PairHostError, "task readiness"):
            self.host.retain_grasp("retain", "episode", scene["observation_id"], "object", "between_fingers",
                                   "original_support_present")
        self.assertEqual(self.host.ledger.status()["steps"], 0)
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_prepare_claim_precedes_one_frame_and_replay_needs_no_new_rgb(self):
        self.host.open()
        self.devices[-1].prepare_hook = lambda: self.assertEqual(self.host.ledger.event("prepare-1")["status"], "pending")
        request, result = self.prepare()
        self.assertEqual(result["status"], "pending")
        completed = self.host.wait("prepare-1", 3)
        self.assertEqual(completed["status"], "completed", completed)
        self.assertEqual(completed["receipt"]["hardware_commands_sent"], 1)
        self.assertFalse(self.host.task_ready)
        self.assertEqual(self.host.ledger.status()["steps"], 1)
        payload = self.host.ledger.event("prepare-1")["payload"]
        self.assertEqual(payload["peer_receipt"]["arm"], "left")
        self.assertEqual(payload["peer_receipt"]["owner"], self.host.owner)
        self.assertIn("model_RGB_semantic", payload["empty_jaw_evidence_kind"])
        self.assertTrue(self.host.prepare_gripper(**request)["replayed"])
        self.assertEqual(self.devices[-1].frame_attempts, 1)
        with self.assertRaises(PairHostError):
            self.host.prepare_gripper(**{**request, "empty_jaw_observation": "different statement"})
        self.assertIsNone(self.host.latest)

    def test_missing_or_changed_saved_rgb_refuses_before_claim(self):
        self.host.open()
        scene = self.scene(saved=False)
        with self.assertRaisesRegex(PairHostError, "saved RGB"):
            self.host.prepare_gripper("prep", scene["observation_id"], "right", "empty jaw")
        scene = self.scene()
        Path(scene["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"changed")
        with self.assertRaisesRegex(PairHostError, "artifact/frame changed"):
            self.host.prepare_gripper("prep", scene["observation_id"], "right", "empty jaw")
        self.assertEqual(self.host.ledger.status()["steps"], 0)
        self.assertFalse(self.host.fault_event.is_set())

    def test_known_missing_preparation_remains_nonfault_with_same_budget(self):
        self.host.open()
        self.assertEqual(self.host.promote_ready()["status"], "preparation_required")
        self.devices[-1].preparation_result = {"ok": False, "status": "preparation_required",
            "requirements": ["selected_arm_six_joint_drivers_enabled"],
            "hardware_commands_sent": 0, "fault_latched": False}
        self.prepare()
        result = self.host.wait("prepare-1", 3)
        self.assertEqual(result["status"], "completed", result)
        self.assertFalse(result["receipt"]["ok"])
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual(self.host.ledger.status()["steps"], 1)

    def test_unknown_failure_latches_once_and_no_replay_send(self):
        self.host.open()
        self.devices[-1].execute_error = OSError("uncertain write")
        request, _ = self.prepare()
        result = self.host.wait("prepare-1", 3)
        self.assertEqual(result["status"], "fault")
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.host.prepare_gripper(**request)["replayed"])
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_guard_after_delayed_claim_refuses_stale_rgb_without_frame(self):
        self.host.open()
        begin = self.host.ledger.begin
        def delayed(*args, **kwargs):
            result = begin(*args, **kwargs)
            self.clock.sleep(31.)
            return result
        with patch.object(self.host.ledger, "begin", side_effect=delayed):
            self.prepare()
            result = self.host.wait("prepare-1", 3)
        self.assertEqual(result["status"], "fault")
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_pending_action_excludes_other_prepare_query_and_promotion(self):
        self.host.open()
        device = self.devices[-1]
        device.execution_gate = threading.Event()
        request, _ = self.prepare()
        self.assertTrue(device.execution_started.wait(1))
        try:
            with self.assertRaises(PairHostError):
                self.host.promote_ready()
            with self.assertRaises(PairHostError):
                self.host.inspect_joint_limits("query")
            with self.assertRaises(PairHostError):
                self.host.prepare_gripper(**{**request, "event_id": "prep2"})
            self.assertTrue(self.host.prepare_gripper(**request)["replayed"])
        finally:
            device.execution_gate.set()
        self.assertEqual(self.host.wait("prepare-1", 3)["status"], "completed")

    def test_cancel_before_preparation_frame_leaves_pending_fault_and_zero_tx(self):
        self.host.open()
        device = self.devices[-1]
        device.execution_gate = threading.Event()
        self.prepare()
        self.assertTrue(device.execution_started.wait(1))
        self.host.cancel()
        device.execution_gate.set()
        self.assertEqual(self.host.wait("prepare-1", 3)["status"], "fault")
        self.assertEqual(device.frame_attempts, 0)

    def test_missing_query_implementation_does_not_consume_step_or_need_rgb(self):
        self.host.open()
        with self.assertRaisesRegex(PairHostError, "query implementation"):
            self.host.inspect_joint_limits("query")
        self.assertEqual(self.host.ledger.status()["steps"], 0)
        self.assertFalse(self.host.fault_event.is_set())

    def test_known_missing_query_preparation_is_nonfault_without_source_publication(self):
        self.host.open()
        device = self.devices[-1]
        device.inspect_joint_limits = lambda: {"ok": False, "status": "preparation_required",
            "hardware_commands_sent": 0, "fault_latched": False, "requirements": ["known_condition"]}
        self.host.inspect_joint_limits("query")
        result = self.host.wait("query", 3)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["receipt"]["status"], "preparation_required")
        self.assertNotIn("source_publication", result["receipt"])
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual(self.host.ledger.status()["steps"], 1)

    def test_query_cannot_run_with_retained_or_candidate_grasp(self):
        self.host.open()
        device = self.devices[-1]
        device.inspect_joint_limits = lambda: (_ for _ in ()).throw(AssertionError("Unexpected query"))
        device.grasp_states["left"] = {"status": "retained_static"}
        with self.assertRaisesRegex(PairHostError, "grasp"):
            self.host.inspect_joint_limits("query")
        self.assertIsNone(self.host.ledger.event("query"))

    def test_preparation_replay_after_clean_owner_resume_keeps_budget_and_old_receipt(self):
        self.host.open()
        request, _ = self.prepare()
        first = self.host.wait("prepare-1", 3)
        deadline = self.host.deadline
        self.host.close()
        second = self.make_host()
        second.open()
        replay = second.prepare_gripper(**request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], first["receipt"])
        self.assertEqual(second.deadline, deadline)
        self.assertEqual(second.ledger.status()["steps"], 1)
        self.assertEqual(self.devices[-1].frame_attempts, 0)

    def test_source_diagnostic_is_explicit_and_status_does_no_provider_io(self):
        self.host.open()
        class Provider:
            calls = 0
            def diagnose(self, scene, arm):
                self.calls += 1
                self.scene = scene
                return {"ready": False, "gaps": ["site geometry absent"]}
        provider = Provider()
        self.host.joint_sources_provider = provider
        scene = self.scene()
        self.assertEqual(provider.calls, 1)
        self.assertIn("inspection_started_at", scene["joint_sources_diagnostic"])
        for _ in range(3):
            self.host.poll()
        self.assertEqual(provider.calls, 1)
        result = self.host._joint_sources_diagnose()
        self.assertEqual(provider.calls, 2)
        self.assertEqual(provider.scene["joint_source_bindings"], self.host._joint_bindings())
        self.assertEqual(self.host.status()["joint_sources_diagnostic"], result)
        self.assertFalse(self.host.fault_event.is_set())

    def test_source_diagnostic_failure_and_delay_do_not_retimestamp_scene(self):
        self.host.open()
        class Provider:
            def diagnose(inner, scene, arm):
                self.clock.sleep(.25)
                raise OSError("missing site file")
        self.host.joint_sources_provider = Provider()
        scene = self.scene()
        diag = scene["joint_sources_diagnostic"]
        self.assertFalse(diag["ready"])
        self.assertIn("missing site file", diag["gaps"][0])
        self.assertGreaterEqual(diag["inspection_finished_at"]-diag["inspection_started_at"], .25)
        self.assertLess(scene["issued_at"], diag["inspection_finished_at"])
        self.assertFalse(self.host.fault_event.is_set())


class ActualAdapterPreparationHostTests(SingleGripperFixture):
    def setUp(self):
        super().setUp()
        for module in (pair_device, pair_preparation, pair_limits, linear_hold,
                       single_supervised_actions, supervised_actions):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.profile["cameras"] = {"front": "f", "left_wrist": "l", "right_wrist": "r"}
        self.robots = {side: QueryRobot(side, self.clock) for side in ("left", "right")}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True] * 6
        self.host = PairHost(self.directory/"runs", self.profile, "actual-prep", TASK,
                            clock=self.clock.time, background=False, connection_mode="prepare")
        self.addCleanup(self.host.close)
        self.frame = 0

    scene = HostPreparationTests.scene

    def test_real_adapter_ledger_both_jaws_then_promote_same_connections_anchor_budget(self):
        status = self.host.open()
        self.assertFalse(status["task_ready"])
        device = self.host.device
        anchor = copy.deepcopy(device._preparation.anchor)
        bindings = {side: device.joint_binding(side) for side in ("left", "right")}
        owner, deadline = self.host.owner, self.host.deadline
        for side in ("left", "right"):
            scene = self.scene()
            event = "prepare-"+side
            self.host.prepare_gripper(event, scene["observation_id"], side, "Current RGB: jaw empty and clear")
            result = self.host.wait(event, 5)
            self.assertEqual(result["status"], "completed", result)
            self.assertEqual(result["receipt"]["hardware_commands_sent"], 1)
            self.assertEqual([f.arbitration_id for f in self.robots[side].sent], [0x159])
        promoted = self.host.promote_ready()
        self.assertTrue(promoted["task_ready"])
        self.assertEqual(self.host.owner, owner)
        self.assertEqual(self.host.deadline, deadline)
        self.assertEqual(self.host.ledger.status()["steps"], 2)
        self.assertEqual(device._action.idle_anchor, anchor)
        self.assertEqual({side: device.joint_binding(side) for side in bindings}, bindings)
        self.assertTrue(all(binding["cached_target"] is None for binding in bindings.values()))
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assertTrue(all(not robot.disconnect.called for robot in self.robots.values()))
        self.assertEqual(sum(len(robot.sent) for robot in self.robots.values()), 2)
        self.assertIsNone(promoted["task_success"])

    def test_twelve_query_frames_publish_current_connection_source_without_rgb_or_motion(self):
        self.host.open()
        anchor = copy.deepcopy(self.host.device._preparation.anchor)
        for robot in self.robots.values():
            robot.on_query = lambda robot, joint: self.assertEqual(self.host.ledger.event("limits")["status"], "pending")
        self.host.inspect_joint_limits("limits")
        result = self.host.wait("limits", 5)
        self.assertEqual(result["status"], "completed", result)
        receipt = result["receipt"]
        self.assertEqual(receipt["hardware_commands_sent"], 12)
        self.assertEqual(receipt["actuator_commands_sent"], 0)
        self.assertFalse(self.host.task_ready)
        publication = receipt["source_publication"]
        index = json.loads(Path(publication["index_path"]).read_text())
        capture_path = Path(publication["index_path"]).parent / index["controller_limits"]["path"]
        capture = json.loads(capture_path.read_text())
        self.assertEqual(capture["run_id"], self.host.run_id)
        self.assertEqual(capture["owner"], self.host.owner)
        self.assertEqual(capture["bindings"], self.host._joint_bindings())
        self.assertEqual(index["geometry"], {})
        self.assertEqual(self.host.ledger.status()["steps"], 1)
        self.assertTrue(self.host.inspect_joint_limits("limits")["replayed"])
        for side in ("left", "right"):
            self.assertEqual([f.arbitration_id for f in self.robots[side].sent], [0x472]*6)
        self.assertEqual(self.host.device._preparation.anchor, anchor)
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)

    def test_partial_query_fault_preserves_raw_receipt_without_publication_or_retry(self):
        self.host.open()
        self.robots["left"].no_reply_joint = 3
        self.host.inspect_joint_limits("limits")
        result = self.host.wait("limits", 5)
        self.assertEqual(result["status"], "fault")
        raw = result["receipt"]["device_receipt"]
        self.assertEqual(raw["hardware_commands_sent"], 3)
        self.assertIn("1", raw["joint_limits"]["left"])
        self.assertFalse((self.host.directory/"joint_sources/index.json").exists())
        self.assertTrue(self.host.inspect_joint_limits("limits")["replayed"])
        self.assertEqual([f.arbitration_id for f in self.robots["left"].sent], [0x472]*3)
        self.assertEqual(self.robots["right"].sent, [])


class RealSDKHostPreparationTests(unittest.TestCase):
    scene = HostPreparationTests.scene

    def test_vendor_host_ledger_query_publication_two_jaws_then_ready_one_connection(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        self.clock, self.frame, sent, created, bindings = Clock(), 0, [], [], {}
        self.profile = copy.deepcopy(PROFILE)
        for cfg in self.profile["arms"].values():
            cfg["model"] = "piper_x"
        self.profile["cameras"] = {"front": "f", "left_wrist": "l", "right_wrist": "r"}
        enabled = {cfg["channel"]: False for cfg in self.profile["arms"].values()}
        clock = self.clock
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def shutdown(self):
                pass
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == 0x472:
                    reply = can.Message(arbitration_id=0x473, is_extended_id=False,
                                        timestamp=clock.time(), data=reply_bytes(frame.data[0]))
                    bindings[self.channel]._ctx.comm.get_callback()(reply)
                elif frame.arbitration_id == 0x159:
                    enabled[self.channel] = True
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            bindings[config["comm"]["can"]["channel"]] = robot
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            channel = next(key for key, value in bindings.items() if value is robot)
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1,
                mode_feedback=1, teach_status=0, motion_status=0, arm_status=0, err_code=0))
            state["gripper"]["width_m"] = .003
            state["gripper"]["foc_status"]["driver_enable_status"] = enabled[channel]
            state["joints_rad"][1:3] = [-.091769, .045658]
            return state
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            self.directory = Path(directory)
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("No physical socket")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, linear_hold, supervised_actions, single_supervised_actions,
                           pair_device, pair_preparation, pair_limits):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            self.host = PairHost(self.directory/"runs", self.profile, "sdk-preparation", TASK,
                                 clock=clock.time, background=False, connection_mode="prepare")
            try:
                self.assertFalse(self.host.open()["task_ready"])
                self.assertEqual(sent, [])
                before = self.host._joint_bindings()
                self.host.inspect_joint_limits("limits")
                query = self.host.wait("limits", 5)
                self.assertEqual(query["status"], "completed", query)
                self.assertTrue(Path(query["receipt"]["source_publication"]["index_path"]).is_file())
                for side in ("left", "right"):
                    scene = self.scene()
                    event = "prepare-"+side
                    self.host.prepare_gripper(event, scene["observation_id"], side, "RGB shows empty clear jaw")
                    result = self.host.wait(event, 5)
                    self.assertEqual(result["status"], "completed", result)
                self.assertTrue(self.host.promote_ready()["task_ready"])
                self.assertEqual(self.host.ledger.status()["steps"], 3)
                self.assertEqual(self.host._joint_bindings(), before)
                self.assertEqual(len(created), 2)
                self.assertEqual([frame.arbitration_id for _, frame in sent], [0x472]*12+[0x159]*2)
                self.assertTrue(all(self.host.device.joint_binding(s)["cached_target"] is None for s in before))
            finally:
                self.host.close()
            self.assertEqual(len(sent), 14)


if __name__ == "__main__":
    unittest.main()
