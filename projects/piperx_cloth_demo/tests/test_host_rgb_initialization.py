"""Native-SDK RGB-supervised initialization on fake CAN, without seeded cache.

Production host/service/device/planners and the actual disk source provider run.
RGB and independently-windowed controller-limit receipts are explicit synthetic
fixtures. No measured site geometry is installed, and no physical path, force,
driver acceptance or stopping claim follows from these offline tests.
"""
import copy
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_tools.joint_path import SDK_COMMIT, evidence_sha256
from robot_tools.joint_sources import JointSourcesError, JointSourcesProvider, publish_controller_limits
import test_host_initialization_integration as native
import test_joint_sources as source_fixture


TOOL = "robot_pair_initialize_joint_target"
UNLOADED = "Synthetic current three-view report: both jaws empty; neither arm contacts an object"
CORRIDOR = ("Synthetic current three-view report: selected whole-arm startup segment, attached "
            "camera/bracket, cables, table and stationary peer have a visibly clear corridor")


class HostRGBInitializationTests(unittest.TestCase):
    # Bind fixture methods only; do not inherit/rerun the previous test class.
    open = native.HostInitializationIntegrationTests.open
    ids = native.HostInitializationIntegrationTests.ids

    def setUp(self):
        self.feedback_mutator = None
        self.expect_guard_disconnect = False
        native.HostInitializationIntegrationTests.setUp(self)
        self.profile["sdk_commit_audited"] = SDK_COMMIT
        self.service.profile = copy.deepcopy(self.profile)
        (self.root / "configs/robot.json").write_text(json.dumps(self.profile))
        official = Path(native.__file__).resolve().parents[1] / "data/piper_x_official"
        installed = self.root / "data/piper_x_official"
        installed.mkdir(parents=True)
        for name in ("sdk_constants.py", "piper_x_description.urdf"):
            shutil.copyfile(official / name, installed / name)
        self.sources = JointSourcesProvider(self.workspace, self.profile, "first-target-integration",
            runs_root=self.service.runs, clock=self.clock.time)
        history = json.loads(native.SAVED.read_text())["state"]["arms"]
        self.joints = {side: history[side]["joints_rad"][:] for side in self.channels}
        for side, expected in (("left", (-5.258, 2.616)), ("right", (-1.669, 2.259))):
            for actual, degrees in zip(self.joints[side][1:3], expected):
                self.assertAlmostEqual(math.degrees(actual), degrees, places=6)

    def check_async_errors(self):
        # Native CANComm also closes when our final-send guard rejects the next
        # frame. Its already-running receive thread may observe that closure.
        for error in self.thread_errors:
            self.assertTrue(self.fail_id is not None or self.expect_guard_disconnect)
            self.assertTrue(self.host.fault_event.is_set())
            self.assertIs(error.exc_type, RuntimeError)
            self.assertEqual(str(error.exc_value), "CAN bus is not connected.")

    def snapshot(self, robot, gripper):
        state = native.HostInitializationIntegrationTests.snapshot(self, robot, gripper)
        if self.feedback_mutator is not None:
            self.feedback_mutator(self.bindings[id(robot)], state)
        return state

    def start(self, mode="ready"):
        if mode == "prepare":
            self.jaws = dict.fromkeys(self.channels, False)
        self.open(mode)
        # A persisted synthetic source, not twelve actual queries or a claim
        # of hardware capture. All per-joint raw windows retain their ordering.
        capture = source_fixture.JointSourcesTests.make_capture(SimpleNamespace(bindings=self.host._joint_bindings()))
        capture.update(run_id=self.host.run_id, owner=self.host.owner)
        offset = self.clock.time() - capture["began_at"]
        time_keys = {"began_at", "ended_at", "request_started_unix_s", "finished_unix_s",
                     "sent_at", "returned_at", "timestamp", "received_unix_s"}

        def shift(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    if key in time_keys:
                        value[key] = item + offset
                    else:
                        shift(item)
            elif isinstance(value, list):
                for item in value:
                    shift(item)

        shift(capture)
        self.clock.sleep(4.01)
        self.publication = publish_controller_limits(self.service.runs, self.profile, self.host.run_id,
            self.host.owner, self.host._joint_bindings(), capture, clock=self.clock.time)
        self.assertEqual(self.sent, [])
        self.assertEqual(json.loads(Path(self.publication["index_path"]).read_text())["geometry"], {})

    def observe(self):
        self.frame += 1
        self.clock.sleep(.01)
        directory = self.workspace / "artifacts" / ("rgb-" + str(self.frame))
        directory.mkdir(parents=True)
        rgb = {"capture_id": "rgb-" + str(self.frame), "cameras": {}}
        for view, key in (("front", "front"), ("left_hand", "left_wrist"), ("right_hand", "right_wrist")):
            path = directory / (view + ".png")
            path.write_bytes(b"\x89PNG\r\n\x1a\n" + ("synthetic RGB " + str(self.frame) + view).encode())
            rgb["cameras"][view] = {"serial": self.profile["cameras"][key], "rgb_path": str(path),
                "frame_number": self.frame, "host_received_at": self.clock.time(), "depth_enabled": False}
        path = directory / "observation.json"
        path.write_text(json.dumps(rgb))
        return self.service.call("robot_pair_observe", {"rgb_observation_path": str(path)})

    def request(self, arm="right", event="rgb-init"):
        scene = self.observe()
        return {"event_id": event, "observation_id": scene["observation_id"], "arm": arm,
                "unloaded_observation": UNLOADED, "admission_mode": "rgb_supervised",
                "corridor_observation": CORRIDOR}

    def initialize(self, request=None):
        request = request or self.request()
        self.assertEqual(self.service.call(TOOL, request)["status"], "pending")
        return self.host.wait(request["event_id"], 10)

    def no_claim(self, event="rgb-init", *, fault=False):
        self.assertEqual(self.sent, [])
        self.assertIsNone(self.host.ledger.event(event))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 0)
        self.assertEqual(self.host.fault_event.is_set(), fault)
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])

    def test_native_dual_boundary_initialization_without_metric_geometry_or_seeded_cache(self):
        self.start("prepare")
        owner, deadline = self.host.owner, self.host.deadline
        original_jaws = {side: copy.deepcopy(self.device._preparation.anchor[side]["gripper"])
                         for side in self.channels}
        for side in ("left", "right"):
            peer = "right" if side == "left" else "left"
            before_peer = self.ids(peer)
            self.assertIsNone(self.device.joint_binding(side)["cached_target"])
            request = self.request(side, "rgb-init-" + side)
            result = self.initialize(request)
            self.assertEqual(result["status"], "completed", result.get("receipt"))
            receipt = result["receipt"]
            plan = receipt["initialization_plan"]
            self.assertEqual(plan["purpose"], "startup_j2_j3")
            self.assertEqual(plan["spatial_admission_mode"], "rgb_supervised")
            self.assertFalse(plan["metric_clearance_checked"])
            self.assertFalse(plan["absolute_workspace_checked"])
            self.assertFalse(plan["hard_path_guarantee"])
            self.assertEqual(plan["target_raw"][1:3], [0, 0])
            self.assertLessEqual(plan["model_endpoint_displacement_m"], .015)
            self.assertEqual(set(plan["geometry"]), {"schema", "origin_sample_id", "source", "evidence"})
            evidence = plan["geometry"]["evidence"]
            self.assertEqual(plan["geometry"]["source"]["sha256"], evidence_sha256(evidence))
            self.assertEqual(evidence["identity"]["owner"], owner)
            self.assertEqual(evidence["observation_id"], request["observation_id"])
            self.assertEqual(evidence["corridor_observation"], CORRIDOR)
            self.assertEqual(evidence["workspace_clearance_statement"],
                             self.host.task["site_context"]["workspace_clearance"]["statement"])
            self.assertEqual(receipt["hardware_commands_sent"], 4)
            self.assertEqual(receipt["passive_arm_commands_sent"], 0)
            self.assertEqual(receipt["gripper_commands_sent"], 0)
            self.assertIsNone(receipt["accepted"])
            self.assertIsNone(receipt["physical_stop_verified"])
            self.assertEqual(self.ids(side), [0x151, 0x155, 0x156, 0x157])
            self.assertEqual(self.ids(peer), before_peer)
            mode = [frame for arm, frame in self.sent if arm == side and frame.arbitration_id == 0x151][0]
            self.assertEqual(bytes(mode.data).hex(), "0101010000000000")
            cache = self.device.joint_binding(side)["cached_target"]
            self.assertEqual(cache["event_id"], request["event_id"])
            self.assertEqual(cache["frame_receipts"], receipt["frame_receipts"])
            self.assertEqual(self.device._preparation.anchor[side]["gripper"], original_jaws[side])
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 2)
        self.assertFalse(self.host.task_ready)
        self.assertFalse(any(self.jaws.values()))
        self.assertEqual(json.loads(Path(self.publication["index_path"]).read_text())["geometry"], {})

    def test_replay_keeps_original_deadline_and_never_sends_again(self):
        self.start()
        request = self.request()
        deadline = self.host.deadline
        result = self.initialize(request)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        frames = self.ids()
        self.clock.sleep(31.)
        replay = self.service.call(TOOL, request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["receipt"], result["receipt"])
        with self.assertRaises(RuntimeError):
            self.service.call(TOOL, {**request, "corridor_observation": "Changed corridor description"})
        self.assertEqual(self.ids(), frames)
        self.assertEqual(self.host.deadline, deadline)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)

    def test_default_metric_mode_does_not_fall_back_to_visual(self):
        self.start()
        request = self.request()
        request.pop("admission_mode")
        request.pop("corridor_observation")
        with self.assertRaises(JointSourcesError):
            self.service.call(TOOL, request)
        self.no_claim()

    def test_postsend_transient_converges_without_retransmission(self):
        self.start("prepare")
        request = self.request()
        owner, deadline = self.host.owner, self.host.deadline
        pulse_started = []

        def pulse(side, state):
            if side == "right" and len(self.ids()) == 4:
                if not pulse_started:
                    pulse_started.append(self.clock.time())
                if self.clock.time() - pulse_started[0] < .3:
                    state["joints_rad"][4] += math.radians(.902)

        self.feedback_mutator = pulse
        result = self.initialize(request)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        self.assertGreaterEqual(self.clock.time() - pulse_started[0], 3.3)
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertEqual(self.device.joint_binding("right")["cached_target"]["event_id"], request["event_id"])
        self.assertFalse(self.host.fault_event.is_set())
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)

    def test_postsend_persistent_deviation_faults_without_cache_or_retry(self):
        self.start("prepare")
        request = self.request()

        def drift(side, state):
            if side == "right" and len(self.ids()) == 4:
                state["joints_rad"][4] += math.radians(.902)

        self.feedback_mutator = drift
        result = self.initialize(request)
        self.assertEqual(result["status"], "fault", result.get("receipt"))
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertTrue(self.host.fault_event.is_set())
        replay = self.service.call(TOOL, request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(self.ids(), [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)

    def test_missing_corridor_and_injected_metric_or_permission_fields_are_zero_tx(self):
        self.start()
        request = self.request()
        missing = {key: value for key, value in request.items() if key != "corridor_observation"}
        with self.assertRaises(ValueError):
            self.service.call(TOOL, missing)
        for key, value in (("attachment_radius_m", .01), ("available_clearance_m", 1.),
                           ("geometry", {}), ("qualified", True), ("cached_target", {})):
            with self.subTest(field=key), self.assertRaises(ValueError):
                self.service.call(TOOL, {**request, key: value})
        self.no_claim()

    def test_expired_rgb_refuses_before_claim(self):
        self.start()
        request = self.request()
        self.clock.sleep(31.)
        with self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.no_claim()

    def test_rgb_tampered_during_source_read_refuses_before_claim(self):
        self.start()
        request = self.request()
        resolve = self.sources.initialization_basis
        def changed(scene, arm):
            result = resolve(scene, arm)
            Path(scene["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"replaced image")
            return result
        with patch.object(self.sources, "initialization_basis", side_effect=changed), self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.no_claim()

    def test_corrupt_persisted_controller_source_is_not_bypassed_by_visual_mode(self):
        self.start()
        request = self.request()
        index = Path(self.publication["index_path"])
        data = json.loads(index.read_text())
        path = index.parent / data["controller_limits"]["path"]
        value = json.loads(path.read_text())
        value["owner"] = "another-owner"
        path.write_text(json.dumps(value))  # Deliberately no matching integrity digest.
        with self.assertRaises(JointSourcesError):
            self.service.call(TOOL, request)
        self.no_claim()

    def test_original_task_deadline_during_source_io_blocks_claim(self):
        self.start()
        request = self.request()
        resolve = self.sources.initialization_basis
        def delayed(scene, arm):
            result = resolve(scene, arm)
            self.clock.sleep(self.host.deadline - self.clock.time() + .001)
            return result
        with patch.object(self.sources, "initialization_basis", side_effect=delayed), self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.no_claim(fault=True)

    def test_initialization_short_rgb_window_refreshes_without_claim(self):
        self.start()
        request = self.request()
        self.clock.sleep(24.1)
        with patch.object(self.sources, "initialization_basis", side_effect=AssertionError("No source IO")):
            result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assertEqual(result["required_rgb_window_s"], 12.)
        self.no_claim()

    def test_initialization_slow_sources_refresh_without_claim(self):
        self.start()
        request = self.request()
        original = self.sources.initialization_basis
        def delayed(scene, arm):
            result = original(scene, arm)
            self.clock.sleep(24.1)
            return result
        with patch.object(self.sources, "initialization_basis", side_effect=delayed):
            result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assertEqual(result["stage"], "after_source_resolution")
        self.no_claim()

    def test_visual_initialization_does_not_supply_ordinary_joint_geometry(self):
        self.start()
        result = self.initialize()
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        before = self.ids()
        scene = self.observe()
        target = self.joints["right"][:]
        target[5] += .001
        with self.assertRaises(JointSourcesError):
            self.service.call("robot_pair_submit_once", {"event_id": "ordinary-without-geometry",
                "observation_id": scene["observation_id"], "peer_receipt_id": scene["peer_receipts"]["left"]["receipt_id"],
                "arm": "right", "kind": "joint", "operation": "approach", "target_joints_rad": target})
        self.assertEqual(self.ids(), before)
        self.assertIsNone(self.host.ledger.event("ordinary-without-geometry"))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)
        self.assertFalse(self.host.fault_event.is_set())

    def assert_postmode_fault(self, mutate):
        self.start()
        request = self.request()
        self.expect_guard_disconnect = True
        def changed(side, state):
            if side == "right" and self.ids():
                mutate(state)
        self.feedback_mutator = changed
        result = self.initialize(request)
        self.assertEqual(result["status"], "fault", result.get("receipt"))
        self.assertEqual(self.ids(), [0x151])
        self.assertEqual(self.ids("left"), [])
        self.assertTrue(self.host.fault_event.is_set())
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        replay = self.service.call(TOOL, request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(self.ids(), [0x151])

    def test_mode_change_after_first_frame_blocks_remaining_target_frames(self):
        self.assert_postmode_fault(lambda state: state["arm_status"].update(mode_feedback=2))

    def test_feedback_older_than_50ms_after_first_frame_blocks_remaining_frames(self):
        def old(state):
            state["fragment_timestamps_s"] = {key: value-.051 for key, value in state["fragment_timestamps_s"].items()}
        self.assert_postmode_fault(old)

    def test_relative_controller_motion_over_20mm_blocks_remaining_frames(self):
        self.assert_postmode_fault(lambda state: state["pose_m_rad"].__setitem__(0, .220001))

    def test_partial_native_send_faults_and_does_not_establish_cache_or_retry(self):
        self.start()
        request = self.request()
        self.fail_id = 0x156
        result = self.initialize(request)
        self.assertEqual(result["status"], "fault")
        self.assertEqual(self.ids(), [0x151, 0x155])
        self.assertEqual(self.ids("left"), [])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.service.call(TOOL, request)["replayed"])
        self.assertEqual(self.ids(), [0x151, 0x155])
        self.assertEqual(self.host.ledger.peek_status()["steps"], 1)


if __name__ == "__main__":
    unittest.main()
