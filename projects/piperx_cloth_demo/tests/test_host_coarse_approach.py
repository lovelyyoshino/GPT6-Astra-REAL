"""Production host/service/native SDK on FakeCAN for explicit coarse approach.

All RGB, controller-limit capture and feedback are synthetic. Real planners and
the normal initialization path establish the cache; no physical result follows.
"""
import copy
import math
import unittest

from robot_tools.rgb_supervision import COARSE_JOINT_PATH_SCHEMA
from test_joint_path import RAW
import test_host_rgb_joint as fixture

TOOL = fixture.TOOL


class HostCoarseApproachTests(unittest.TestCase):
    setUp = fixture.HostRGBJointTests.setUp
    open = fixture.HostRGBJointTests.open
    ids = fixture.HostRGBJointTests.ids
    check_async_errors = fixture.HostRGBJointTests.check_async_errors
    snapshot = fixture.HostRGBJointTests.snapshot
    start = fixture.HostRGBJointTests.start
    observe = fixture.HostRGBJointTests.observe
    request = fixture.HostRGBJointTests.request
    initialize = fixture.HostRGBJointTests.initialize
    execute = fixture.HostRGBJointTests.execute
    frame_record = fixture.HostRGBJointTests.frame_record
    assert_no_claim = fixture.HostRGBJointTests.assert_no_claim

    def prepared(self):
        # Legal nonzero initial feedback, still P mode and no known target.
        self.joints = {side: [math.radians(v / 1000) for v in RAW] for side in self.channels}
        fixture.HostRGBJointTests.prepared(self)

    def coarse_step(self, event="coarse-step", arm="right", degrees=2.5):
        request = fixture.HostRGBJointTests.step(self, arm, event, inward=False)
        target = [math.radians(v / 1000) for v in
                  self.device.joint_binding(arm)["cached_target"]["target_raw"]]
        target[1] += math.radians(degrees)
        request.update(operation="approach", target_joints_rad=target,
            motion_profile="coarse_approach",
            far_from_target_observation="Synthetic current RGB: both empty jaws remain far from contact and the target")
        return request

    def test_native_two_and_half_degree_step_is_once_bound_and_default_unchanged(self):
        self.prepared()
        request = self.coarse_step()
        frames = self.frame_record()
        ordinary = {key: value for key, value in request.items()
                    if key not in ("motion_profile", "far_from_target_observation")}
        with self.assertRaises((RuntimeError, ValueError)):
            self.service.call(TOOL, ordinary)
        self.assert_no_claim(request, frames)
        self.assertFalse(self.host.fault_event.is_set())
        before = copy.deepcopy(self.joints)
        budget = self.host.ledger.peek_status()
        owner, deadline = self.host.owner, self.host.deadline
        right_before, left_before = len(self.ids()), self.ids("left")
        result = self.execute(request)
        self.assertEqual(result["status"], "completed", result.get("receipt"))
        receipt = result["receipt"]
        plan = receipt["joint_path_plan"]
        self.assertEqual(plan["motion_profile"], "coarse_approach")
        self.assertEqual(plan["geometry"]["schema"], COARSE_JOINT_PATH_SCHEMA)
        self.assertEqual(plan["geometry"]["evidence"]["far_from_target_observation"],
                         request["far_from_target_observation"])
        self.assertAlmostEqual(math.degrees(self.joints["right"][1] - before["right"][1]), 2.5)
        self.assertEqual(self.joints["left"], before["left"])
        self.assertEqual(self.ids()[right_before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), left_before)
        self.assertEqual(receipt["hardware_commands_sent"], 4)
        self.assertEqual(receipt["passive_arm_commands_sent"], 0)
        self.assertFalse(plan["hold_supported"])
        self.assertEqual(plan["hold_policy"], "latch_only")
        self.assertIsNone(receipt["hold_receipt"])
        self.assertIsNone(receipt["object_task_success"])
        self.assertIsNone(receipt["physical_stop_verified"])
        self.assertEqual((owner, deadline), (self.host.owner, self.host.deadline))
        now = self.host.ledger.peek_status()
        self.assertEqual(now["steps"], budget["steps"] + 1)
        self.assertEqual(now["deadline_s"], budget["deadline_s"])
        stored = self.host.ledger.event(request["event_id"])
        self.assertEqual(stored["payload"]["motion_profile"], "coarse_approach")
        self.assertEqual(stored["payload"]["far_from_target_observation"], request["far_from_target_observation"])
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        sent = self.frame_record()
        self.assertTrue(self.service.call(TOOL, request)["replayed"])
        for changed in (ordinary, {**request, "far_from_target_observation": "Changed current distance testimony"}):
            with self.assertRaises(RuntimeError):
                self.service.call(TOOL, changed)
        self.assertEqual(self.frame_record(), sent)
        self.assertEqual(self.host.ledger.peek_status()["steps"], now["steps"])

    def test_wrong_scope_or_missing_semantics_refuses_before_claim(self):
        self.prepared()
        request = self.coarse_step()
        before = self.frame_record()
        changes = ({"operation": "align"}, {"operation": "release_retreat"},
            {"operation": "transport", "loaded_observation": "Carrying", "source_object_id": "a", "target_object_id": "b"},
            {"admission_mode": "metric_geometry"}, {"motion_profile": "other"},
            {"far_from_target_observation": " "}, {"far_from_target_observation": "\x00"},
            {"unloaded_observation": " "}, {"corridor_observation": " "},
            {"release_retreat_observation": "Empty"})
        for change in changes:
            with self.subTest(change=change), self.assertRaises((RuntimeError, ValueError)):
                self.service.call(TOOL, {**request, **change})
            self.assert_no_claim(request, before)
        for absent in ("motion_profile", "far_from_target_observation", "unloaded_observation", "corridor_observation"):
            changed = {key: value for key, value in request.items() if key != absent}
            with self.subTest(absent=absent), self.assertRaises((RuntimeError, ValueError)):
                self.service.call(TOOL, changed)
            self.assert_no_claim(request, before)
        for kind, field, target in (("move", "target_pose_m_rad", [0.] * 6), ("gripper", "width_m", .04)):
            changed = {key: value for key, value in request.items() if key != "target_joints_rad"}
            changed.update(kind=kind, **{field: target})
            with self.assertRaises((RuntimeError, ValueError)):
                self.service.call(TOOL, changed)
            self.assert_no_claim(request, before)
        with self.assertRaises(ValueError):
            self.service.call("robot_pair_initialize_joint_target", {
                **self.request(event="not-coarse-init"), "motion_profile": "coarse_approach"})
        self.assertFalse(self.host.fault_event.is_set())

    def test_either_local_grasp_or_active_durable_episode_blocks_coarse(self):
        self.prepared()
        request = self.coarse_step()
        before = self.frame_record()
        for side in ("left", "right"):
            for status in ("contact_candidate", "retained_static", "release_opened", "loaded_pending_visual"):
                with self.subTest(side=side, status=status):
                    self.device._action.grasps[side] = {"arm": side, "status": status}
                    with self.assertRaises(RuntimeError):
                        self.service.call(TOOL, request)
                    self.assert_no_claim(request, before)
                    self.device._action.grasps[side] = None
        # Even an empty allocated episode is active and cannot authorize this profile.
        self.host.grasps.store.create(self.host.owner, episode_id="empty-reservation", arm="left",
                                      object_id="test-strip", epoch=self.host.owner)
        with self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.assert_no_claim(request, before)

    def test_native_partial_send_latches_without_retry_or_extra_hold(self):
        self.prepared()
        request = self.coarse_step()
        before, peer = len(self.ids()), self.ids("left")
        self.fail_id = 0x156
        result = self.execute(request)
        self.assertEqual(result["status"], "fault", result.get("receipt"))
        self.assertEqual(self.ids()[before:], [0x151, 0x155])
        self.assertEqual(self.ids("left"), peer)
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.service.call(TOOL, request)["replayed"])
        self.assertEqual(self.ids()[before:], [0x151, 0x155])

    def test_coarse_profile_preserves_rgb_headroom_and_original_task_deadline(self):
        self.prepared()
        request = self.coarse_step()
        before, deadline = self.frame_record(), self.host.deadline
        self.clock.sleep(24.1)
        result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assert_no_claim(request, before)
        self.assertFalse(self.host.fault_event.is_set())
        request = self.coarse_step(event="coarse-after-deadline")
        self.clock.sleep(deadline - self.clock.time() + .001)
        with self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.assertEqual(self.host.deadline, deadline)
        self.assertTrue(self.host.fault_event.is_set())
        self.assert_no_claim(request, before)


if __name__ == "__main__":
    unittest.main()
