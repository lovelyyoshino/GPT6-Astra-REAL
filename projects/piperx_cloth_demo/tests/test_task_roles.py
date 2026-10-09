"""Frozen role contracts and the native SDK/FakeCAN left-worker chain."""
import copy
import json
import math
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_tools.task_roles import PROFILE_KEY, resolve_task_roles, role_fields
from robot_tools.grasp_episode import apply_event, GraspEpisodeError
from robot_tools.loaded_episode import allowed_next
from test_grasp_episode import retained, event, scene, HASH
import test_host_loaded_joint as loaded
import test_host_initialization_integration as native
from robot_tools.joint_sources import JointSourcesProvider


class TaskRoleTests(unittest.TestCase):
    def test_legacy_bytes_remain_unchanged_and_explicit_pair_is_strict(self):
        task = {"task_id": "plug_transfer_left", "roles": {"left": "task", "right": "task"}}
        before = json.dumps(task, sort_keys=True)
        self.assertEqual(resolve_task_roles(task), ("right", "left"))
        self.assertEqual(role_fields(task), {})
        self.assertEqual(json.dumps(task, sort_keys=True), before)
        for worker, support in (("left", "right"), ("right", "left")):
            explicit = {**task, "worker_arm": worker, "support_arm": support}
            self.assertEqual(resolve_task_roles(explicit), (worker, support))
        for bad in ({"worker_arm": "left"}, {"support_arm": "right"},
                    {"worker_arm": "left", "support_arm": "left"},
                    {"worker_arm": "other", "support_arm": "right"},
                    {"worker_arm": None, "support_arm": "right"}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                resolve_task_roles(bad)

    def test_left_episode_requires_explicit_roles_and_cannot_change_them(self):
        state = retained("left")
        evidence = {"action_event_id": "extract", "operation": "extract_segment", "source_id": "source",
                    "target_id": "left-target", "target_raw": [1]*6, "context_sha256": HASH,
                    "scene": scene(state, at=110., frame=110)}
        with self.assertRaises(GraspEpisodeError):
            apply_event(state, event(state, "begin", "begin_loaded", evidence), now=110.)
        roles = {"worker_arm": "left", "support_arm": "right"}
        begun = apply_event(state, event(state, "begin", "begin_loaded", {**evidence, **roles}), now=110.)
        self.assertEqual(role_fields(begun["loaded"]), roles)
        with self.assertRaises(GraspEpisodeError):
            allowed_next(begun, "extract_segment", "source", "left-target",
                         task_roles={"worker_arm": "right", "support_arm": "left"})

    def test_service_freezes_explicit_roles_and_rejects_incomplete_pair_before_host(self):
        from test_pair_service import PairServiceTests, OPEN
        fixture = PairServiceTests("runTest")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.service.persistent = True
        with patch("robot_tools.pair_host.PairHost", return_value=fixture.host) as constructor:
            with self.assertRaises(ValueError):
                fixture.service.call("robot_pair_open", {**OPEN, "worker_arm": "left"})
            constructor.assert_not_called()
            fixture.service.call("robot_pair_open", {**OPEN, "worker_arm": "left", "support_arm": "right"})
        self.assertEqual(resolve_task_roles(constructor.call_args.args[3]), ("left", "right"))
        self.assertEqual(constructor.call_args.args[1][PROFILE_KEY],
                         {"worker_arm": "left", "support_arm": "right"})

    def test_enrollment_requires_exact_rendered_recipe_for_explicit_roles(self):
        from robot_tools.pair_task_enrollment import _task
        from robot_tools.plug_recipe import render_plug_recipe
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/"bundle/projects/piperx_cloth_demo"
            root.mkdir(parents=True)
            canonical_path = root.parent.parent/"tasks/plug_transfer_left.json"
            canonical_path.parent.mkdir()
            source = Path(__file__).resolve().parents[3]/"tasks/plug_transfer_left.json"
            canonical_path.write_bytes(source.read_bytes())
            canonical = json.loads(canonical_path.read_text())
            task = {"task_id": "plug_transfer_left", "roles": {"left": "task", "right": "task"},
                    "site_context": {"workspace_clearance": {"source": "user", "statement": "Synthetic clearance"}}}
            envelope = {"schema": "piper_supervised_plug_task_v1", "task": task,
                        "recipe_path": str(canonical_path),
                        "recipe_sha256": hashlib.sha256(canonical_path.read_bytes()).hexdigest(),
                        "on_site_supervision": {"source": "user", "statement": "Synthetic supervision"}}
            task_path = root/"task.json"
            task_path.write_text(json.dumps(envelope))
            self.assertNotIn("rendered_recipe", _task(task_path, root))
            task.update(worker_arm="left", support_arm="right")
            task_path.write_text(json.dumps(envelope))
            with self.assertRaisesRegex(RuntimeError, "rendered recipe"):
                _task(task_path, root)
            rendered_path = root/"rendered.json"
            rendered = render_plug_recipe(canonical, worker_arm="left", support_arm="right")
            rendered_path.write_text(json.dumps(rendered))
            envelope.update(rendered_recipe_path=str(rendered_path),
                            rendered_recipe_sha256=hashlib.sha256(rendered_path.read_bytes()).hexdigest())
            task_path.write_text(json.dumps(envelope))
            self.assertIn("rendered_recipe", _task(task_path, root))
            rendered["steps"][7]["arm"] = "right"
            rendered_path.write_text(json.dumps(rendered))
            envelope["rendered_recipe_sha256"] = hashlib.sha256(rendered_path.read_bytes()).hexdigest()
            task_path.write_text(json.dumps(envelope))
            with self.assertRaisesRegex(RuntimeError, "Rendered"):
                _task(task_path, root)


class LeftWorkerIntegrationTests(unittest.TestCase):
    """Synthetic RGB/contact only; no real devices or object success claim."""
    worker_arm, support_arm = "left", "right"
    setUp = loaded.HostLoadedJointTests.setUp
    snapshot = loaded.HostLoadedJointTests.snapshot
    ids = loaded.HostLoadedJointTests.ids
    check_async_errors = loaded.HostLoadedJointTests.check_async_errors
    start = loaded.HostLoadedJointTests.start
    observe = loaded.HostLoadedJointTests.observe
    request = loaded.HostLoadedJointTests.request
    initialize = loaded.HostLoadedJointTests.initialize
    prepared = loaded.HostLoadedJointTests.prepared
    step = loaded.HostLoadedJointTests.step
    execute = loaded.HostLoadedJointTests.execute
    frame_record = loaded.HostLoadedJointTests.frame_record
    jaw = loaded.HostLoadedJointTests.jaw
    retained_pair = loaded.HostLoadedJointTests.retained_pair
    loaded_request = loaded.HostLoadedJointTests.loaded_request
    confirm_loaded = loaded.HostLoadedJointTests.confirm_loaded
    release_confirm = loaded.HostLoadedJointTests.release_confirm

    def open(self, mode="ready"):
        roles = {"worker_arm": self.worker_arm, "support_arm": self.support_arm}
        self.profile[PROFILE_KEY] = roles
        self.sources = JointSourcesProvider(self.workspace, self.profile, "first-target-integration",
            runs_root=self.service.runs, clock=self.clock.time)
        with patch.object(native, "TASK", {**copy.deepcopy(native.TASK), **roles}):
            loaded.HostLoadedJointTests.open(self, mode)

    def test_left_extract_transport_insert_confirm_release_keeps_right_static(self):
        self.retained_pair()
        original = copy.deepcopy(self.device.grasp_states["left"]["original_anchor"])
        support = copy.deepcopy(self.device.grasp_states["right"])
        owner, deadline = self.host.owner, self.host.deadline
        for operation, relation in (("extract_segment", "source_separated"),
                                   ("transport", "target_aligned"), ("insert_segment", "target_seated")):
            peer_frames = self.ids("right")[:]
            req = self.loaded_request(operation, operation)
            result = self.execute(req)
            self.assertEqual(result["status"], "completed", result.get("receipt"))
            self.assertEqual(result["receipt"]["hardware_commands_sent"], 4)
            self.assertEqual(self.ids("right"), peer_frames)
            context = result["receipt"]["joint_path_plan"]["loaded_context"]
            self.assertEqual(resolve_task_roles(context), ("left", "right"))
            self.assertEqual(context["worker"]["identity"]["arm"], "left")
            self.assertEqual(context["peer"]["identity"]["arm"], "right")
            self.assertEqual(self.host.grasps.active("left")["status"], "loaded_pending_visual")
            count = self.frame_record()
            response, confirm = self.confirm_loaded(operation, relation)
            self.assertEqual(response["hardware_commands_sent"], 0)
            self.assertTrue(self.service.call("robot_pair_confirm_loaded_response", confirm)["replayed"])
            self.assertEqual(self.frame_record(), count)
            self.assertEqual(self.device.grasp_states["right"], support)
            self.assertEqual(self.device.grasp_states["left"]["original_anchor"], original)
        self.assertEqual(len(self.host.grasps.active("left")["loaded"]["history"]), 3)
        for side in ("left", "right"):
            self.assertEqual(self.jaw(side, "open-"+side, opening=True)["status"], "completed")
            self.assertEqual(self.release_confirm(side)["episode"]["status"], "released")
        self.assertEqual(self.host.grasp_states, {"left": None, "right": None})
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))

    def test_wrong_worker_and_changed_peer_binding_are_zero_claim_zero_tx(self):
        self.retained_pair()
        req = self.loaded_request(event="wrong-worker")
        req["arm"] = "right"
        before = self.frame_record()
        with self.assertRaisesRegex(RuntimeError, "frozen-worker"):
            self.service.call("robot_pair_submit_once", req)
        self.assertEqual(self.frame_record(), before)
        self.assertIsNone(self.host.ledger.event("wrong-worker"))
        req = self.loaded_request(event="wrong-peer")
        resolve = self.host._joint_context
        def changed(*args, **kwargs):
            context = resolve(*args, **kwargs)
            context["loaded_context"]["peer"]["identity"]["arm"] = "left"
            return context
        with patch.object(self.host, "_joint_context", side_effect=changed), self.assertRaises(ValueError):
            self.service.call("robot_pair_submit_once", req)
        self.assertEqual(self.frame_record(), before)
        self.assertIsNone(self.host.ledger.event("wrong-peer"))

    def test_adapter_rejects_rehashed_cross_role_context_against_frozen_profile(self):
        self.retained_pair()
        from robot_tools.pair_joint_adapter import _loaded_binding
        from robot_tools.joint_path import evidence_sha256
        req = self.loaded_request(event="cross-role")
        payload = {"arm": "left", "operation": "extract_segment", "target": req["target_joints_rad"],
                   "observation_id": req["observation_id"], "loaded_observation": req["loaded_observation"],
                   "source_object_id": req["source_object_id"], "target_object_id": req["target_object_id"],
                   "admission_mode": "rgb_supervised", "corridor_observation": req["corridor_observation"]}
        scene = self.host.latest
        context = self.host._joint_context("cross-role", payload, scene)
        bad = copy.deepcopy(context)
        bad["identity"]["arm"] = "right"
        bound = bad["loaded_context"]
        bound.update(worker_arm="right", support_arm="left")
        bound["worker"], bound["peer"] = bound["peer"], bound["worker"]
        bad["geometry"]["evidence"]["loaded_context_sha256"] = evidence_sha256(bound)
        before = self.frame_record()
        with self.assertRaisesRegex(ValueError, "selected worker"):
            _loaded_binding(self.device, bad, "cross-role", "extract_segment", bad["identity"])
        self.assertEqual(self.frame_record(), before)


if __name__ == "__main__":
    unittest.main()
