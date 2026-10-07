"""Real host/store/adapter/service against fake CAN; never physical grasp proof."""
import copy
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

from robot_tools import pair_device
from robot_tools.pair_host import PairHost
from robot_tools.service import ToolService
from test_pair_host import TASK
from test_single_supervised_actions import SingleActionFixture


class HostGraspFixture(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.stack.enter_context(patch.object(pair_device, "time", self.clock))
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True] * 6
            robot.gripper_enabled = True
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.workspace = Path(self.temp.name)
        self.root = self.workspace / "projects" / "piperx_cloth_demo"
        (self.root / "configs").mkdir(parents=True)
        self.profile["cameras"] = {"front": "fake-front", "left_wrist": "fake-left", "right_wrist": "fake-right"}
        (self.root / "configs" / "robot.json").write_text(json.dumps(self.profile))
        self.service = ToolService(self.root)
        self.host = PairHost(self.service.runs, self.profile, "grasp-run", TASK,
                             clock=self.clock.time, background=False)
        self.service.pair_host = self.host
        self.service.persistent = True
        self.addCleanup(self.host.close)
        self.host.open()
        self.frame = 0
        self.contacts = {}
        def contact(robot, state):
            if robot.side in self.contacts and any(frame.arbitration_id == 0x159 for frame in robot.sent):
                robot.width = self.contacts[robot.side]
                state["gripper"]["width_m"] = robot.width
        self.hook = contact

    def observe(self):
        self.frame += 1
        self.clock.sleep(.01)
        directory = self.workspace / "artifacts" / ("capture-" + str(self.frame))
        directory.mkdir(parents=True)
        rgb = {"capture_id": "capture-" + str(self.frame), "cameras": {}}
        for view, key in (("front", "front"), ("left_hand", "left_wrist"), ("right_hand", "right_wrist")):
            path = directory / (view + ".png")
            path.write_bytes(("synthetic RGB fixture " + str(self.frame) + view).encode())
            rgb["cameras"][view] = {"serial": self.profile["cameras"][key], "rgb_path": str(path),
                "frame_number": self.frame, "host_received_at": self.clock.time(), "depth_enabled": False}
        path = directory / "observation.json"
        path.write_text(json.dumps(rgb))
        return self.service.call("robot_pair_observe", {"rgb_observation_path": str(path)})

    def submit(self, side, kind, operation, target, event_id, *, object_id=None):
        scene = self.observe()
        peer = "right" if side == "left" else "left"
        request = {"event_id": event_id, "observation_id": scene["observation_id"],
            "peer_receipt_id": scene["peer_receipts"][peer]["receipt_id"], "arm": side,
            "kind": kind, "operation": operation,
            {"gripper": "width_m", "move": "target_pose_m_rad", "joint": "target_joints_rad"}[kind]: target}
        if object_id:
            request["grasp_object_id"] = object_id
        if operation == "release_retreat":
            if kind == "gripper":
                request.update(release_support_observation="Synthetic current independent support report",
                               release_support_relation="independent_support_present")
            elif kind == "joint":
                request["release_retreat_observation"] = "Synthetic empty gripper clear of the object in current RGB"
        self.service.call("robot_pair_submit_once", request)
        result = self.host.wait(event_id, 10)
        receipt = result.get("receipt", {})
        self.assertEqual(result["status"], "completed", (receipt.get("error"),
                         (receipt.get("device_receipt") or {}).get("errors")))
        return result

    def candidate(self, side="left"):
        self.robots[side].accept = False
        self.contacts[side] = .048
        self.submit(side, "gripper", "grip_supported", .0455, "probe-" + side,
                    object_id="strip" if side == "left" else "plug")
        state = self.host.grasps.active(side)
        self.assertEqual(state["status"], "contact_candidate")
        return state

    def retain(self, side="left"):
        state = self.host.grasps.active(side) or self.candidate(side)
        scene = self.observe()
        request = {"event_id": "retain-" + side, "episode_id": state["identity"]["episode_id"],
            "observation_id": scene["observation_id"], "visual_description": "Synthetic test testimony only",
            "object_relation": "between_fingers", "support_relation": "original_support_present"}
        result = self.service.call("robot_pair_retain_grasp", request)
        return result, request

    def confirm(self, side="left", *, event_id=None):
        state = self.host.grasps.active(side)
        scene = self.observe()
        request = {"event_id": event_id or "confirm-" + side, "episode_id": state["identity"]["episode_id"],
                   "observation_id": scene["observation_id"], "visual_description": "Synthetic separation report only",
                   "object_relation": "object_clear_of_fingers", "support_relation": "independent_support_present"}
        return self.service.call("robot_pair_confirm_release", request), request


class HostGraspIntegrationTests(HostGraspFixture):
    def test_retention_zero_tx_then_peer_move_and_two_independent_grasps(self):
        self.candidate("left")
        before = [len(r.sent) for r in self.robots.values()]
        result, request = self.retain("left")
        self.assertEqual(result["episode"]["status"], "retained_static")
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)
        self.assertFalse(result["loaded_contact_available"])
        old_trace = result["episode"]["measurement"]["trace_id"]
        replay = self.service.call("robot_pair_retain_grasp", request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["episode"]["measurement"]["trace_id"], old_trace)
        target = self.robots["right"].motion.origin[:]
        target[2] += .006  # Existing Motion fixture reports exactly this 6 mm response.
        self.submit("right", "move", "approach", target, "peer-approach")
        self.assertEqual(len(self.robots["left"].sent), 1)
        self.candidate("right")
        self.retain("right")
        self.assertEqual([self.host.grasps.active(s)["status"] for s in ("left", "right")],
                         ["retained_static", "retained_static"])
        self.contacts.pop("left")
        self.robots["left"].accept = True
        left_episode = self.host.grasps.active("left")["identity"]["episode_id"]
        self.submit("left", "gripper", "release_retreat", .05, "release-left")
        self.assertEqual(self.host.grasps.store.read(left_episode)["status"], "release_opened")
        self.confirm("left")
        self.assertEqual(self.host.grasps.store.read(left_episode)["status"], "released")
        self.assertEqual(self.host.grasps.active("right")["status"], "retained_static")
        self.assertIsNotNone(self.host.device.grasp_states["right"])
        self.assertFalse(self.host.status()["fault_latched"])

    def test_retained_arm_and_loaded_peer_motion_remain_blocked(self):
        self.candidate()
        self.retain()
        before = [len(r.sent) for r in self.robots.values()]
        for side, operation in (("left", "approach"), ("right", "extract_segment"), ("right", "transport")):
            with self.subTest(side=side, operation=operation), self.assertRaises((ValueError, RuntimeError)):
                self.submit(side, "move", operation, self.robots[side].motion.origin[:], operation + side)
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)

    def test_rgb_replacement_cannot_create_retention(self):
        state = self.candidate()
        scene = self.observe()
        Path(scene["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"changed image")
        with self.assertRaisesRegex(ValueError, "artifact changed"):
            self.host.retain_grasp("retain-bad", state["identity"]["episode_id"], scene["observation_id"],
                                   "Synthetic report", "between_fingers", "original_support_present")
        self.assertEqual(self.host.grasps.active("left")["status"], "contact_candidate")
        self.assertEqual(len(self.robots["left"].sent), 1)

    def test_cancel_blocks_replayed_retention_and_no_new_sends(self):
        self.candidate()
        _, request = self.retain()
        self.host.cancel("Synthetic user cancellation")
        with self.assertRaises(RuntimeError):
            self.service.call("robot_pair_retain_grasp", request)
        self.assertEqual(len(self.robots["left"].sent), 1)
        self.assertEqual(len(self.robots["right"].sent), 0)

    def test_mismatched_adapter_contract_cannot_borrow_durable_retention(self):
        self.candidate()
        self.retain()
        self.host.device._action.grasps["left"]["probe_event_id"] = "other-probe"
        with self.assertRaisesRegex(ValueError, "exact durable"):
            self.submit("right", "move", "approach", self.robots["right"].motion.origin[:], "wrong-contract")
        self.assertEqual(len(self.robots["right"].sent), 0)

    def test_caller_cannot_supply_hardware_contract_to_retention(self):
        self.candidate()
        _, request = self.retain()
        for key in ("retention_contract", "measurement", "contact_support_verified", "grasp_verified"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.service.call("robot_pair_retain_grasp", {**request, key: True})

    def test_missing_live_grasp_cannot_turn_durable_retention_into_an_empty_arm(self):
        self.candidate()
        self.retain()
        self.host.device._action.grasps["left"] = None
        before = len(self.robots["left"].sent)
        with self.assertRaisesRegex(ValueError, "missing or changed"):
            self.submit("left", "move", "approach", self.robots["left"].motion.origin[:], "lost-state")
        self.assertEqual(len(self.robots["left"].sent), before)
        self.assertIsNone(self.host.ledger.event("lost-state"))
        self.assertTrue(self.host.status()["fault_latched"])

    def test_close_with_lost_adapter_state_preserves_durable_unreleased_grasp(self):
        self.candidate()
        self.retain()
        self.host.device._action.grasps["left"] = None
        result = self.host.close()
        self.assertTrue(result["fault_latched"])
        self.assertEqual(self.host.grasps.active("left")["status"], "retained_static")

    def test_failed_release_observation_latches_host_before_any_physical_claim(self):
        self.candidate()
        with patch.object(self.host.device, "observe_grasp", return_value={"ok": False,
                 "hardware_commands_sent": 0, "error": "synthetic drift"}):
            with self.assertRaisesRegex(ValueError, "Release preparation"):
                self.submit("left", "gripper", "release_retreat", .05, "release-fault")
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertIsNone(self.host.ledger.event("release-fault"))
        self.assertEqual(len(self.robots["left"].sent), 1)
