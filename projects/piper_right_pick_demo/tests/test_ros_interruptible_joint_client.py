"""Offline client cancellation tests; no physical stopping claim."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
SPEC = importlib.util.spec_from_file_location('interruptible_client', Path(__file__).parents[1]/'scripts/ros_interruptible_joint_client.py')
client = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(client)


class Clock:
    now = 100.
    def time(self): return self.now
    def monotonic(self): return self.now
    def sleep(self, value): self.now += value


class IO:
    def __init__(self, clock):
        self.clock = clock; self.published = []; self.held = []; self.polls = 0; self.mode = 'hold'
        self.token = 'a'*32
        self.state = dict(adoption_token=self.token, sequence=0, phase='idle', active=False,
                          failure=None, stop_latched=False, command_sent_unix_s=None,
                          before=dict(raw_q=[0, 77000, -59000, 0, 16000, -7000]),
                          receipts=[], last_refusal=None, result={})
    def status(self):
        s = copy.deepcopy(self.state)
        if self.published:
            self.polls += 1
            s.update(sequence=1, phase='moving', active=True,
                     hold_service=client.NODE+'/hold_current/seq_1_'+self.token,
                     command_sent_unix_s=100., receipts=[dict(attempted_frames=4, socket_send_returns=4)])
            raw = [0, 76700, -59000, 0, 16000, -7000]
            s['latest_state'] = dict(stamps=[self.clock.time()-.001]*14, raw_q=raw,
                q=[x*(math.pi/180000) for x in raw], pose=[.3, 0., .3, 0., 0., 0.],
                opening_m=.034, ctrl_mode=1, arm_status=0, fault=0, teach_status=0,
                mode=1, motion_status=1, driver_codes=[64]*6, jaw_code=64)
            if self.mode == 'normal': s.update(phase='completed', active=False)
            if self.mode == 'stale': s['latest_state']['stamps'][0] -= 1
            if self.mode == 'different_action': s['sequence'] = 2
            if self.mode == 'different_adoption': s['adoption_token'] = 'b'*32
            if self.mode == 'partial': s['receipts'][0]['socket_send_returns'] = 2
            if self.mode == 'failed': s.update(phase='failed', active=False, failure='send failed')
            if self.mode == 'wrong_service': s['hold_service'] += '_wrong'
            if self.mode == 'preflight': s.update(phase='preflight', command_sent_unix_s=None, receipts=[])
            if self.held and self.mode != 'unconfirmed' and self.polls >= 8:
                s.update(phase='cancelled_before_dispatch' if self.mode=='preflight' else 'hold_confirmed', active=False)
        return s
    def publish(self, raw): self.published.append(raw)
    def hold(self, path):
        self.held.append(path)
        return True, dict(adoption_token=self.token, sequence=1, accepted=True)


class ClientTests(unittest.TestCase):
    def setUp(self):
        guard = patch('socket.socket', side_effect=AssertionError('No hardware I/O'))
        guard.start(); self.addCleanup(guard.stop)
        self.clock = Clock(); self.io = IO(self.clock)
        self.rows = []
    def run_client(self, **kwargs):
        return client.run_goal(self.io, [0, 76000, -59000, 0, 16000, -7000],
                               clock=self.clock, audit=self.rows.append, **kwargs)
    def test_timeout_requests_bound_hold_once_and_waits_for_confirmation(self):
        result = self.run_client(motion_timeout_s=.02)
        self.assertTrue(result['hold_confirmed']); self.assertTrue(result['hold_accepted'])
        self.assertEqual(result['hold_reason'], 'motion_timeout')
        self.assertEqual(len(self.io.published), 1); self.assertEqual(len(self.io.held), 1)
        self.assertGreaterEqual(self.io.polls, 8)
        self.assertFalse(result['target_reached']); self.assertFalse(result['task_success'])
    def test_normal_arrival_sends_no_hold(self):
        self.io.mode = 'normal'
        result = self.run_client()
        self.assertTrue(result['target_reached']); self.assertFalse(result['hold_confirmed'])
        self.assertEqual(self.io.held, [])
    def test_accepted_request_without_confirmed_feedback_is_not_stop(self):
        self.io.mode = 'unconfirmed'
        result = self.run_client(motion_timeout_s=.02)
        self.assertTrue(result['hold_accepted']); self.assertFalse(result['hold_confirmed'])
        self.assertTrue(result['accepted_target_may_continue'])
        self.assertIn('confirmation timed out', result['error'])
        self.assertEqual(len(self.io.held), 1)
    def test_old_action_and_adoption_cannot_cancel_new_action(self):
        for mode in ('different_action', 'different_adoption', 'wrong_service'):
            with self.subTest(mode=mode):
                self.io = IO(self.clock); self.io.mode = mode
                result = self.run_client(cancelled=lambda: bool(self.io.published))
                self.assertIn('error', result); self.assertEqual(self.io.held, [])
    def test_partial_failed_or_stale_feedback_sends_no_hold(self):
        for mode in ('partial', 'failed', 'stale'):
            with self.subTest(mode=mode):
                self.io = IO(self.clock); self.io.mode = mode
                result = self.run_client(motion_timeout_s=.01)
                self.assertIn('error', result); self.assertEqual(self.io.held, [])
                self.assertFalse(result['hold_confirmed'])
    def test_progress_trigger_uses_same_bound_interface(self):
        result = self.run_client(cancel_after_progress_deg=.25)
        self.assertTrue(result['hold_confirmed'])
        self.assertEqual(result['hold_reason'], 'validation_progress_trigger')
    def test_preflight_cancel_is_reported_as_no_dispatch(self):
        self.io.mode = 'preflight'
        result = self.run_client(cancelled=lambda: bool(self.io.published))
        self.assertEqual(result['driver_phase'], 'cancelled_before_dispatch')
        self.assertFalse(result['hold_confirmed']); self.assertFalse(result['target_reached'])
    def test_no_service_retry_after_ambiguous_response(self):
        def ambiguous(path):
            self.io.held.append(path); raise RuntimeError('response lost')
        self.io.hold = ambiguous
        result = self.run_client(motion_timeout_s=.02)
        self.assertEqual(result['error'], 'response lost'); self.assertEqual(len(self.io.held), 1)
    def test_cancel_before_publication_sends_nothing(self):
        result = self.run_client(cancelled=lambda: True)
        self.assertEqual(result['driver_phase'], 'cancelled_before_publication')
        self.assertEqual(self.io.published, []); self.assertEqual(self.io.held, [])


if __name__ == '__main__': unittest.main()
