"""Native SDK/FakeCAN chain from RGB initialization through ordinary joint steps.

The host, file source provider, ledger, model and encoders are production code.
Images, limits capture and feedback are explicit synthetic fixtures. No metric
site geometry or target cache is seeded; these tests do not prove real motion.
"""
import copy
import json
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

from robot_tools import joint_path, pair_joint_adapter
from robot_tools.bounded_joint_step import STEP
from robot_tools.joint_sources import JointSourcesError
import test_host_rgb_initialization as fixture

TOOL = 'robot_pair_submit_once'


class HostRGBJointTests(unittest.TestCase):
    setUp = fixture.HostRGBInitializationTests.setUp
    open = fixture.HostRGBInitializationTests.open
    ids = fixture.HostRGBInitializationTests.ids
    check_async_errors = fixture.HostRGBInitializationTests.check_async_errors
    snapshot = fixture.HostRGBInitializationTests.snapshot
    start = fixture.HostRGBInitializationTests.start
    observe = fixture.HostRGBInitializationTests.observe
    request = fixture.HostRGBInitializationTests.request
    initialize = fixture.HostRGBInitializationTests.initialize

    def prepared(self):
        self.start('prepare')
        for side in ('left', 'right'):
            result = self.initialize(self.request(side, 'init-' + side))
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
        for side in ('left', 'right'):
            scene = self.observe()
            request = {'event_id': 'jaw-' + side, 'observation_id': scene['observation_id'],
                       'arm': side, 'empty_jaw_observation': fixture.UNLOADED}
            self.assertEqual(self.service.call('robot_pair_prepare_gripper', request)['status'], 'pending')
            result = self.host.wait(request['event_id'], 10)
            self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertTrue(self.service.call('robot_pair_promote_ready', {})['task_ready'])
        self.assertEqual(self.host.ledger.peek_status()['steps'], 4)

    def step(self, arm='right', event='visual-step', *, inward=True):
        scene = self.observe()
        peer = 'left' if arm == 'right' else 'right'
        cached = self.device.joint_binding(arm)['cached_target']
        target = [math.radians(v / 1000) for v in cached['target_raw']]
        if inward:
            margin = math.ceil(STEP['joint_margin_rad'] / joint_path.RAD_PER_RAW) * joint_path.RAD_PER_RAW
            target[1:3] = [margin, -margin]
        else:
            target[5] += .001
        return {'event_id': event, 'observation_id': scene['observation_id'],
                'peer_receipt_id': scene['peer_receipts'][peer]['receipt_id'], 'arm': arm,
                'kind': 'joint', 'operation': 'approach' if inward else 'align',
                'target_joints_rad': target, 'admission_mode': 'rgb_supervised',
                'unloaded_observation': 'Synthetic current RGB: selected arm is empty, not in object contact',
                'corridor_observation': fixture.CORRIDOR}

    def execute(self, request):
        self.assertEqual(self.service.call(TOOL, request)['status'], 'pending')
        return self.host.wait(request['event_id'], 10)

    def frame_record(self):
        return [(side, frame.arbitration_id, bytes(frame.data), frame.dlc,
                 frame.is_extended_id, frame.is_remote_frame, frame.is_error_frame)
                for side, frame in self.sent]

    def assert_no_claim(self, request, before, steps=4):
        self.assertEqual(self.frame_record(), before)
        self.assertIsNone(self.host.ledger.event(request['event_id']))
        self.assertEqual(self.host.ledger.peek_status()['steps'], steps)

    def test_dual_rgb_init_prepare_ingress_then_ordinary_without_geometry(self):
        self.prepared()
        owner, deadline = self.host.owner, self.host.deadline
        for inward in (True, False):
            for side in ('left', 'right'):
                peer = 'right' if side == 'left' else 'left'
                request = self.step(side, ('inward-' if inward else 'align-') + side, inward=inward)
                before, peer_before = len(self.ids(side)), self.ids(peer)
                old_cache = copy.deepcopy(self.device.joint_binding(side)['cached_target'])
                result = self.execute(request)
                self.assertEqual(result['status'], 'completed', result.get('receipt'))
                receipt = result['receipt']
                plan = receipt['joint_path_plan']
                self.assertEqual(plan['spatial_admission_mode'], 'rgb_supervised')
                self.assertFalse(plan['metric_clearance_checked'])
                self.assertFalse(plan['absolute_workspace_checked'])
                self.assertFalse(plan['hold_supported'])
                self.assertEqual(plan['hold_policy'], 'latch_only')
                self.assertNotIn('workspace_min_m', plan['geometry'])
                self.assertEqual(plan['cached_target'], old_cache)
                self.assertEqual(plan['geometry']['evidence']['operation'], request['operation'])
                self.assertEqual(plan['geometry']['evidence']['target_raw'], plan['target_raw'])
                self.assertEqual(receipt['original_event']['schema'], 'piper_rgb_supervised_joint_send_v1')
                self.assertIsNone(receipt['hold_receipt'])
                self.assertFalse(receipt['explicit_cancel_hold_bridge_bound'])
                self.assertEqual(self.ids(side)[before:], [0x151, 0x155, 0x156, 0x157])
                self.assertEqual(self.ids(peer), peer_before)
                self.assertEqual(receipt['hardware_commands_sent'], 4)
                self.assertEqual(receipt['passive_arm_commands_sent'], 0)
                self.assertIsNone(receipt['physical_stop_verified'])
                self.assertIsNone(receipt['object_task_success'])
                self.assertEqual(self.device.joint_binding(side)['cached_target']['event_id'], request['event_id'])
        self.assertEqual((self.host.owner, self.host.deadline), (owner, deadline))
        self.assertEqual(self.host.ledger.peek_status()['steps'], 8)
        self.assertEqual((len(self.created), len(self.buses)), (2, 2))
        self.assertEqual(json.loads(Path(self.publication['index_path']).read_text())['geometry'], {})
        frames = self.frame_record()
        self.assertTrue(self.service.call(TOOL, request)['replayed'])
        self.assertEqual(self.frame_record(), frames)
        with self.assertRaises(RuntimeError):
            self.service.call(TOOL, {**request, 'corridor_observation': 'Different scene testimony'})
        self.assertEqual(self.frame_record(), frames)

    def test_default_metric_does_not_fall_back_to_rgb(self):
        self.prepared()
        request = self.step()
        for key in ('admission_mode', 'unloaded_observation', 'corridor_observation'):
            del request[key]
        before = self.frame_record()
        with self.assertRaises(JointSourcesError):
            self.service.call(TOOL, request)
        self.assert_no_claim(request, before)
        self.assertFalse(self.host.fault_event.is_set())

    def test_old_but_unexpired_rgb_refreshes_without_claim_and_new_scene_can_continue(self):
        self.prepared()
        request = self.step()
        before = self.frame_record()
        cached = copy.deepcopy(self.device.joint_binding("right")["cached_target"])
        self.clock.sleep(24.1)
        with patch.object(self.sources, "rgb_joint_basis", side_effect=AssertionError("Must reject before IO")):
            result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assertEqual(result["required_rgb_window_s"], 6.)
        self.assertFalse(result["event_claimed"])
        self.assertFalse(result["fault_latched"])
        self.assert_no_claim(request, before)
        self.assertEqual(self.device.joint_binding("right")["cached_target"], cached)
        self.assertFalse(self.host.fault_event.is_set())
        fresh = self.step(event=request["event_id"])
        completed = self.execute(fresh)
        self.assertEqual(completed["status"], "completed", completed.get("receipt"))
        self.assertEqual(self.host.ledger.peek_status()["steps"], 5)

    def test_source_cost_rechecks_window_before_claim(self):
        self.prepared()
        request, before = self.step(), self.frame_record()
        original = self.sources.rgb_joint_basis
        def delayed(scene, arm):
            result = original(scene, arm)
            self.clock.sleep(24.1)
            return result
        with patch.object(self.sources, "rgb_joint_basis", side_effect=delayed):
            result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assertEqual(result["stage"], "after_source_resolution")
        self.assert_no_claim(request, before)
        self.assertFalse(self.host.fault_event.is_set())

    def test_plan_cost_rechecks_window_before_claim(self):
        self.prepared()
        request, before = self.step(), self.frame_record()
        original = self.host._joint_context
        def delayed(*args):
            result = original(*args)
            self.clock.sleep(24.1)
            return result
        with patch.object(self.host, "_joint_context", side_effect=delayed):
            result = self.service.call(TOOL, request)
        self.assertEqual(result["status"], "refresh_required")
        self.assertEqual(result["stage"], "before_claim")
        self.assert_no_claim(request, before)
        self.assertFalse(self.host.fault_event.is_set())

    def test_slow_claim_keeps_claimed_zero_tx_fault_without_refunding_budget(self):
        self.prepared()
        request, before = self.step(), self.frame_record()
        original = self.host.ledger.begin
        def delayed(*args):
            result = original(*args)
            self.clock.sleep(24.1)
            return result
        with patch.object(self.host.ledger, "begin", side_effect=delayed):
            result = self.execute(request)
        self.assertEqual(result["status"], "fault", result.get("receipt"))
        self.assertEqual(self.frame_record(), before)
        self.assertEqual(self.host.ledger.peek_status()["steps"], 5)
        self.assertTrue(self.host.fault_event.is_set())
        receipt = result["receipt"]["device_receipt"]
        self.assertEqual(receipt["hardware_commands_sent"], 0)
        self.assertIn("insufficient_rgb_postsend_window", str(receipt["errors"]))
        self.assertFalse(receipt["rgb_dispatch_window"]["claimed_event_or_budget_released"])
        self.assertTrue(self.service.call(TOOL, request)["replayed"])
        self.assertEqual(self.frame_record(), before)

    def test_visual_scope_and_descriptions_refuse_before_claim(self):
        self.prepared()
        request = self.step()
        before = self.frame_record()
        variants = [dict(request, operation='extract_segment'), dict(request, operation='release_retreat'),
                    dict(request, unloaded_observation=' '), dict(request, admission_mode='metric_geometry'),
                    {k: v for k, v in request.items() if k != 'corridor_observation'},
                    {k: v for k, v in request.items() if k != 'admission_mode'}]
        for candidate in variants:
            # release_retreat is now a supported typed RGB operation; its
            # missing current release token is rejected by HostGrasps.
            expected = ValueError if candidate["operation"] == "release_retreat" else RuntimeError
            with self.subTest(candidate=candidate), self.assertRaises(expected):
                self.service.call(TOOL, candidate)
            self.assert_no_claim(request, before)
        with self.assertRaises(ValueError):
            self.service.call(TOOL, {**request, 'geometry': {}})
        self.assert_no_claim(request, before)

    def test_image_changed_during_source_resolution_refuses_before_claim(self):
        self.prepared()
        request = self.step()
        before = self.frame_record()
        original = self.sources.rgb_joint_basis
        def changed(scene, arm):
            sources = original(scene, arm)
            Path(scene['saved_rgb_evidence']['front']['rgb_path']).write_bytes(b'Changed synthetic image')
            return sources
        with patch.object(self.sources, 'rgb_joint_basis', side_effect=changed), self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.assert_no_claim(request, before)

    def test_current_limits_corruption_cannot_be_bypassed_with_visual_text(self):
        self.prepared()
        request = self.step()
        before = self.frame_record()
        index = Path(self.publication['index_path'])
        source = index.parent / json.loads(index.read_text())['controller_limits']['path']
        source.write_bytes(b'Invalid capture')
        with self.assertRaises(JointSourcesError):
            self.service.call(TOOL, request)
        self.assert_no_claim(request, before)

    def test_source_io_cannot_renew_original_task_deadline(self):
        self.prepared()
        request = self.step()
        before = self.frame_record()
        deadline = self.host.deadline
        original = self.sources.rgb_joint_basis

        def delayed(scene, arm):
            sources = original(scene, arm)
            self.clock.sleep(deadline - self.clock.time() + .001)
            return sources

        with patch.object(self.sources, 'rgb_joint_basis', side_effect=delayed), self.assertRaises(RuntimeError):
            self.service.call(TOOL, request)
        self.assert_no_claim(request, before)
        self.assertEqual(self.host.deadline, deadline)
        self.assertTrue(self.host.fault_event.is_set())

    def test_partial_native_send_latches_without_retry_or_selected_cache(self):
        self.prepared()
        request = self.step()
        before, peer_before = len(self.ids()), self.ids('left')
        self.fail_id = 0x156
        result = self.execute(request)
        self.assertEqual(result['status'], 'fault', result.get('receipt'))
        self.assertEqual(self.ids()[before:], [0x151, 0x155])
        self.assertEqual(self.ids('left'), peer_before)
        self.assertIsNone(self.device.joint_binding('right')['cached_target'])
        self.assertTrue(self.host.fault_event.is_set())
        self.assertTrue(self.service.call(TOOL, request)['replayed'])
        self.assertEqual(self.ids()[before:], [0x151, 0x155])

    def test_short_postsend_tracking_transient_converges_with_one_dispatch(self):
        self.prepared()
        request = self.step()
        before, peer_before = len(self.ids()), self.ids('left')
        started = []

        def pulse(side, state):
            if side == 'right' and len(self.ids()) == before + 4:
                if not started:
                    started.append(self.clock.time())
                if self.clock.time() - started[0] < .35:
                    state['joints_rad'][4] += math.radians(.902)

        self.feedback_mutator = pulse
        result = self.execute(request)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        receipt = result['receipt']
        tracking = receipt['tracking_observation']
        self.assertGreater(tracking['cumulative_outside_nominal_band_s'], 0.)
        self.assertLessEqual(tracking['cumulative_outside_nominal_band_s'], 1.)
        self.assertIsNone(tracking['first_failure'])
        self.assertGreaterEqual(self.clock.time() - started[0], 3.35)
        self.assertEqual(self.ids()[before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids('left'), peer_before)
        self.assertFalse(self.host.fault_event.is_set())

    def test_persistent_postsend_tracking_deviation_latches_without_resend(self):
        self.prepared()
        request = self.step()
        before, peer_before = len(self.ids()), self.ids('left')

        def drift(side, state):
            if side == 'right' and len(self.ids()) == before + 4:
                state['joints_rad'][4] += math.radians(.902)

        self.feedback_mutator = drift
        result = self.execute(request)
        self.assertEqual(result['status'], 'fault', result.get('receipt'))
        receipt = result['receipt']['device_receipt']
        self.assertGreater(receipt['tracking_observation']['cumulative_outside_nominal_band_s'], 1.)
        self.assertFalse(receipt['arrival_confirmed'])
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.ids()[before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids('left'), peer_before)
        self.assertTrue(self.service.call(TOOL, request)['replayed'])
        self.assertEqual(self.ids()[before:], [0x151, 0x155, 0x156, 0x157])

    def test_explicit_cancel_after_original_is_latch_only_with_zero_hold_frames(self):
        self.prepared()
        request = self.step()
        before, peer_before = len(self.ids()), self.ids('left')
        recorded, release = threading.Event(), threading.Event()
        original = pair_joint_adapter._JointExecutor.original
        def gate(runner):
            original(runner)
            recorded.set()
            if not release.wait(5):
                raise RuntimeError('Synthetic completion gate timed out')
        with patch.object(pair_joint_adapter._JointExecutor, 'original', gate):
            try:
                self.assertEqual(self.service.call(TOOL, request)['status'], 'pending')
                self.assertTrue(recorded.wait(5))
                self.assertIsNone(self.host.active_joint_bridge)
                cancel = self.service.call('robot_pair_cancel', {'reason': 'Explicit synthetic cancellation'})
                self.assertTrue(cancel['software_cancelled'])
            finally:
                release.set()
            result = self.host.wait(request['event_id'], 10)
        self.assertEqual(result['status'], 'fault', result.get('receipt'))
        receipt = result['receipt']['device_receipt']
        self.assertIsNone(receipt['hold_receipt'])
        self.assertFalse(receipt['explicit_cancel_hold_bridge_bound'])
        self.assertEqual(self.ids()[before:], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids('left'), peer_before)
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.host.ledger.peek_status()['steps'], 5)


if __name__ == '__main__':
    unittest.main()
