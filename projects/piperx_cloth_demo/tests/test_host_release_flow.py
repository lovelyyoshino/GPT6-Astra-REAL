"""Actual host/store/device release flow on synthetic RGB and fake CAN only."""
import copy
from unittest.mock import patch

from test_host_grasp_integration import HostGraspFixture
from test_host_joint_integration import HostJointFixture


class ReleaseFlowTests(HostGraspFixture):
    def opened(self, side="left"):
        state = self.candidate(side)
        self.contacts.pop(side)
        self.robots[side].accept = True
        self.submit(side, "gripper", "release_retreat", .05, "open-" + side)
        return self.host.grasps.store.read(state["identity"]["episode_id"])

    def test_two_openings_remain_unresolved_then_confirm_without_tx_and_replay(self):
        opened = self.opened()
        deadline = opened["deadline_at"]
        self.assertEqual(opened["status"], "release_opened")
        self.assertEqual(self.host.device.grasp_states["left"]["status"], "release_opened")
        self.submit("left", "gripper", "release_retreat", .052, "open-left-again")
        state = self.host.grasps.active("left")
        self.assertEqual(state["release_opening"]["action_event_id"], "open-left-again")
        self.assertEqual(state["original_anchor"], opened["original_anchor"])
        before = [len(r.sent) for r in self.robots.values()]
        result, request = self.confirm()
        self.assertEqual(result["episode"]["status"], "released")
        self.assertEqual(result["episode"]["deadline_at"], deadline)
        self.assertEqual(self.host.ledger.status()["steps"], 3)
        self.assertIsNone(self.host.device.grasp_states["left"])
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)
        replay = self.service.call("robot_pair_confirm_release", request)
        self.assertTrue(replay["replayed"])
        self.assertEqual(replay["episode"], result["episode"])
        with self.assertRaises(ValueError):
            self.service.call("robot_pair_confirm_release", {**request, "visual_description": "changed"})
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)

    def test_opening_requires_current_support_before_claim(self):
        self.candidate()
        scene = self.observe()
        with self.assertRaisesRegex(ValueError, "independent support"):
            self.host.submit("missing-support", scene["observation_id"],
                scene["peer_receipts"]["right"]["receipt_id"], "left", "gripper", .05, "release_retreat")
        self.assertIsNone(self.host.ledger.event("missing-support"))
        self.assertFalse(self.host.status()["fault_latched"])
        self.assertEqual(len(self.robots["left"].sent), 1)

    def test_opened_episode_blocks_other_arm_motion_and_clean_close(self):
        self.opened()
        before = [len(r.sent) for r in self.robots.values()]
        with self.assertRaises(ValueError):
            self.submit("right", "move", "approach", self.robots["right"].motion.origin, "wrong-move")
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)
        closed = self.host.close()
        self.assertTrue(closed["fault_latched"])
        self.assertEqual(self.host.grasps.active("left")["status"], "release_opened")

    def test_old_opening_scene_cannot_confirm_new_opening(self):
        self.opened()
        old = self.observe()
        self.submit("left", "gripper", "release_retreat", .052, "second-open")
        state = self.host.grasps.active("left")
        with self.assertRaisesRegex(RuntimeError, "current host-issued RGB scene"):
            self.host.confirm_release("old-scene", state["identity"]["episode_id"], old["observation_id"],
                "Synthetic stale report", "object_clear_of_fingers", "independent_support_present")
        self.assertEqual(self.host.grasps.active("left")["status"], "release_opened")

    def test_failed_finalization_keeps_unresolved_device_and_latches_after_durable_fact(self):
        self.opened()
        before = [len(r.sent) for r in self.robots.values()]
        with patch.object(self.host.device, "finalize_release", return_value={"ok": False,
                "hardware_commands_sent": 0, "physical_stop_verified": None}):
            with self.assertRaises(ValueError):
                self.confirm()
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertEqual(self.host.device.grasp_states["left"]["status"], "release_opened")
        self.assertEqual(self.host.grasps.states()[0]["status"], "released")
        self.assertEqual(self.host.grasps.release_tokens, {})
        self.assertEqual([len(r.sent) for r in self.robots.values()], before)

    def test_finalizer_wrong_source_hash_cannot_create_retreat_token(self):
        self.opened()
        finalize = self.host.device.finalize_release
        def wrong(*args, **kwargs):
            result = finalize(*args, **kwargs)
            result["confirmation_trace_sha256"] = "0" * 64
            return result
        with patch.object(self.host.device, "finalize_release", side_effect=wrong):
            with self.assertRaises(ValueError):
                self.confirm()
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertEqual(self.host.grasps.release_tokens, {})

    def test_cancel_during_final_rgb_io_cannot_publish_current_release_token(self):
        self.opened()
        final_recorded = [False]
        append, saved = self.host.journal.append, self.host._saved_preparation_scene
        def journal(event, **kwargs):
            result = append(event, **kwargs)
            if event == "pair_release_confirmed":
                final_recorded[0] = True
            return result
        def io(*args, **kwargs):
            result = saved(*args, **kwargs)
            if final_recorded[0]:
                self.host.fault_event.set()  # Synthetic concurrent cancel/EOF.
            return result
        with patch.object(self.host.journal, "append", side_effect=journal), \
                patch.object(self.host, "_saved_preparation_scene", side_effect=io):
            with self.assertRaises(RuntimeError):
                self.confirm()
        self.assertTrue(final_recorded[0])
        self.assertTrue(self.host.status()["fault_latched"])
        self.assertEqual(self.host.grasps.release_tokens, {})

    def test_caller_cannot_inject_measurement_or_release_boolean(self):
        state = self.opened()
        scene = self.observe()
        request = {"event_id": "injected", "episode_id": state["identity"]["episode_id"],
                   "observation_id": scene["observation_id"], "visual_description": "Synthetic report",
                   "object_relation": "object_clear_of_fingers", "support_relation": "independent_support_present"}
        for name in ("measurement", "release_verified", "release_trace_sha256", "confirmation_trace_sha256"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.service.call("robot_pair_confirm_release", {**request, name: True})


class ReleaseJointFlowTests(HostJointFixture):
    submit = HostGraspFixture.submit
    candidate = HostGraspFixture.candidate
    retain = HostGraspFixture.retain
    confirm = HostGraspFixture.confirm

    def setUp(self):
        super().setUp()
        self.contacts = {}
        def contact(robot, state):
            if robot.side in self.contacts and any(frame.arbitration_id == 0x159 for frame in robot.sent):
                robot.width = self.contacts[robot.side]
                state["gripper"]["width_m"] = robot.width
        self.hook = contact

    def open_side(self, side, event):
        self.contacts.pop(side, None)
        self.robots[side].accept = True
        return self.submit(side, "gripper", "release_retreat", .05, event)

    def retreat(self, side, event):
        self.seed_test_cache(side)  # Explicit synthetic prior complete MOVE_J history only.
        target = self.joints[side][:]
        target[5] += .001
        return self.submit(side, "joint", "release_retreat", target, event)

    def test_pair_right_then_left_release_confirm_and_joint_retreat(self):
        self.candidate("left")
        self.retain("left")
        self.candidate("right")
        self.retain("right")
        original = copy.deepcopy(self.host.device.grasp_states["left"]["original_anchor"])
        deadline = self.host.deadline
        self.open_side("right", "open-right")
        self.submit("right", "gripper", "release_retreat", .052, "open-right-more")
        self.confirm("right")
        left_before = len(self.ids("left"))
        self.retreat("right", "retreat-right")
        self.assertEqual(len(self.ids("left")), left_before)
        self.assertEqual(self.host.device.grasp_states["left"]["original_anchor"], original)
        self.open_side("left", "open-left")
        self.confirm("left")
        right_before = len(self.ids("right"))
        self.retreat("left", "retreat-left")
        self.assertEqual(len(self.ids("right")), right_before)
        self.assertEqual(self.host.grasp_states, {"left": None, "right": None})
        self.assertEqual(self.host.ledger.status()["steps"], 7)
        self.assertEqual(self.host.deadline, deadline)
        self.assertIsNone(self.host.status()["task_success"])

    def test_post_confirm_jaw_command_invalidates_old_release_for_retreat(self):
        self.candidate("right")
        self.open_side("right", "open-right")
        self.confirm("right")
        self.submit("right", "gripper", "approach", .049, "ordinary-jaw")
        before = len(self.ids("right"))
        with self.assertRaisesRegex(ValueError, "current confirmation"):
            self.retreat("right", "stale-retreat")
        self.assertEqual(len(self.ids("right")), before)
        self.assertIsNone(self.host.ledger.event("stale-retreat"))

    def test_opened_without_confirmation_cannot_retreat(self):
        self.candidate("right")
        self.open_side("right", "open-right")
        before = len(self.ids("right"))
        with self.assertRaises(ValueError):
            self.retreat("right", "unconfirmed-retreat")
        self.assertEqual(len(self.ids("right")), before)
        self.assertIsNone(self.host.ledger.event("unconfirmed-retreat"))
