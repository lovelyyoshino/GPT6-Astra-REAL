"""Offline maintenance contract tests. No hardware/ROS/process may be opened."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('ros_home_step', Path(__file__).parents[1]/'scripts/ros_home_step.py')
home = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(home)


def sample(now=100., seq=1, q=None):
    q = list(q if q is not None else [2494, -1732, 2906, -4229, 21874, 4963])
    return dict(sequence=seq, stamps=[now-.005+i*.00001 for i in range(14)],
                raw_q=q, q=[v*home.RAD for v in q], pose=[.1, .01, .25, 0., 1.4, 0.],
                opening_m=.02, jaw_code=0, ctrl_mode=1, arm_status=0, mode=1,
                teach_status=0, motion_status=0, fault=0, driver_codes=[64]*6)


def legal_sample():
    return sample(q=[2494, 1732, -2906, -4229, 21874, 4963])


class Clock:
    def __init__(self): self.now = 100.
    def time(self): return self.now
    def monotonic(self): return self.now


class FakeIO:
    def __init__(self, directory, *, fail=None, jitter=False):
        self.driver_log = Path(directory)/'driver.log'; self.driver_log.write_text('')
        self.clock = Clock(); self.received = 0; self.published = []; self.connected = 0
        self.fail, self.jitter = fail, jitter
        self.base = legal_sample(); self.owner = {'offline': True, 'driver_pid': 123}
        self.checked = 0

    def identity(self): return self.owner
    def verify_health_message(self, s): self.checked += 1
    def connect_publisher(self): self.connected += 1

    def receive(self, timeout_s=1.):
        self.clock.now += .051; self.received += 1
        q = list(self.base['raw_q'])
        if self.jitter: q[3] += 205 if self.received % 2 else 0
        if self.published:
            if self.fail == 'timeout': raise TimeoutError('offline receive timed out')
            q = list(self.published[0]['target_raw'])
            if self.fail == 'other_axis': q[0] += 173
        return sample(self.clock.now, self.received, q)

    def publish_once(self, plan):
        self.published.append(copy.deepcopy(plan))
        # A real durable pending marker must already exist when publishing.
        persisted = json.loads(self.state_path.read_text())
        assert persisted['pending'] is not None and persisted['attempts'] == 1
        if self.fail == 'publish': raise OSError('offline ambiguous publish failure')
        if self.fail != 'missing_intent':
            self.driver_log.write_text(json.dumps(dict(source='ros_low_speed_entry',
                event='motion_mode_before_sdk_call', move_mode=1, ctrl_mode=1,
                requested_speed_percent=1, effective_speed_percent=1, is_mit_mode=0))+'\n')


class HomeStepTests(unittest.TestCase):
    def setUp(self):
        for target in ('socket.socket', 'subprocess.Popen'):
            guard = patch(target, side_effect=AssertionError('Hardware/process forbidden in offline test'))
            guard.start(); self.addCleanup(guard.stop)

    def test_raw_fourteen_frame_decode_preserves_every_field_timestamp(self):
        s = sample()
        payloads = {i: bytes(8) for i in home.IDS}
        payloads[0x2A1] = bytes([1, 0, 1, 0, 0, 0, 0, 0])
        for i in range(3): payloads[0x2A5+i] = struct.pack('>ii', *s['raw_q'][2*i:2*i+2])
        p = [100000, 10000, 250000, 0, 80000, 0]
        for i in range(3): payloads[0x2A2+i] = struct.pack('>ii', *p[2*i:2*i+2])
        payloads[0x2A8] = struct.pack('>ihBB', 20000, 50, 0, 0)
        for identifier in range(0x261, 0x267): payloads[identifier] = bytes([0, 0, 0, 0, 0, 64, 0, 0])
        frames = {i: (99.99, payloads[i]) for i in home.IDS}
        actual = home.decode(frames, 14)
        self.assertEqual(actual['raw_q'], s['raw_q'])
        self.assertEqual(actual['driver_codes'], [64]*6)
        self.assertEqual(actual['jaw_code'], 0)
        self.assertEqual(actual['stamps'], [99.99]*14)
        home.health(actual, 100.)
        del frames[0x266]
        with self.assertRaisesRegex(RuntimeError, 'Incomplete'): home.decode(frames, 15)

    def test_live_receiver_uses_only_kernel_stamped_nonlocal_fragments_and_no_send(self):
        packets = []
        ancillary = [(home.socket.SOL_SOCKET, getattr(home.socket, 'SO_TIMESTAMPNS', 35), struct.pack('@ll', 100, 0))]
        for identifier in home.IDS:
            packets.append((struct.pack('=IB3x8s', identifier, 8, bytes(8)), ancillary, 0, ('can1',)))
        # Local loopback cannot stand in for a received driver/status frame.
        local = (packets[0][0], packets[0][1], home.socket.MSG_DONTROUTE, ('can1',))
        class Receiver:
            def __init__(self): self.packets = iter([local]+packets); self.bound = None
            def setsockopt(self, *args): pass
            def bind(self, address): self.bound = address
            def settimeout(self, value): pass
            def recvmsg(self, *args): return next(self.packets)
            def close(self): pass
            # Deliberately no send/sendto API in this fake.
        receiver = Receiver()
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory)/'driver.log'; log.write_text('')
            io = home.LiveIO(log)
            with patch.object(home.socket, 'socket', return_value=receiver), patch.object(home.time, 'monotonic', return_value=100.):
                state = io.receive()
            self.assertEqual(receiver.bound, ('can1',))
            self.assertEqual(state['sequence'], 14)
            self.assertEqual(state['stamps'], [100.]*14)
            self.assertIsNone(io.publisher)
            io.close()

    def test_initial_scope_and_nominal_exceptions_are_narrow(self):
        initial = sample(); session = home.new_session({}, initial)
        self.assertEqual(session['exception_caps_raw'], [1732, 2906])
        for axis, value in ((0, 45001), (1, -5001), (2, 5001), (3, 15001), (4, 30001), (5, 15001)):
            q = list(initial['raw_q']); q[axis] = value
            with self.subTest(axis=axis):
                with self.assertRaises(RuntimeError): home.new_session({}, sample(q=q))

    def test_plan_is_only_one_selected_axis_toward_zero_and_preserves_latest_others(self):
        initial = legal_sample(); session = home.new_session({}, initial)
        for axis in range(1, 7):
            plan = home.make_plan(axis, 1., initial, session)
            expected = list(initial['raw_q']); i = axis-1
            expected[i] -= 1000 if expected[i] > 0 else -1000
            self.assertEqual(plan['target_raw'], expected)
            self.assertEqual(plan['message']['velocity'], [0.]*6+[1.])
            self.assertEqual(len(plan['message']['position']), 6)
            self.assertEqual(plan['expected_jaw_frames'], 0)
        for axis, step in ((0, 1), (True, 1), (7, 1), (1, 0), (1, 1.01), (1, math.nan), (1, .0001)):
            with self.assertRaises(RuntimeError): home.make_plan(axis, step, initial, session)

    def test_tail_step_can_be_smaller_than_old_point55_degree_floor(self):
        initial = sample(q=[3, 12, -23, -4, 5, 6]); session = home.new_session({}, initial)
        plan = home.make_plan(2, 1, initial, session)
        self.assertEqual(plan['target_raw'], [3, 0, -23, -4, 5, 6])
        self.assertEqual(plan['step_deg'], .012)
        with self.assertRaisesRegex(RuntimeError, 'already exactly zero'):
            home.make_plan(1, 1, sample(q=[0, 12, -23, -4, 5, 6]), session)

    def test_transient_exception_jitter_is_bounded_and_legal_cannot_be_reexempted(self):
        initial = sample(); session = home.new_session({}, initial)
        q = list(initial['raw_q']); q[1] -= 171
        home.check_session(sample(q=q), session)
        q[1] -= 2
        with self.assertRaisesRegex(RuntimeError, 'deepened'): home.check_session(sample(q=q), session)
        session['exception_caps_raw'][0] = 0
        q[1] = -1
        with self.assertRaisesRegex(RuntimeError, 'revoked'): home.check_session(sample(q=q), session)

    def test_other_exception_feedback_jitter_cannot_become_deeper_command(self):
        initial = sample(); session = home.new_session({}, initial)
        q = list(initial['raw_q']); q[2] += 100
        measured = sample(q=q)
        home.check_session(measured, session)  # Original monitoring tolerance remains.
        with self.assertRaisesRegex(RuntimeError, 'manufacturer nominal'):
            home.make_plan(2, 1, measured, session)

    def test_global_initial_to_zero_box_prevents_cumulative_drift_between_steps(self):
        initial = sample(); session = home.new_session({}, initial)
        q = list(initial['raw_q']); q[0] += 173
        with self.assertRaisesRegex(RuntimeError, 'initial-to-zero'):
            home.check_session(sample(q=q), session)

    def test_stable_xyz_span_is_vector_norm_and_jaw_feedback_keeps_55mm_limit(self):
        a, b = sample(100., 1), sample(100.1, 2)
        b['pose'][0] += .0004; b['pose'][1] += .0004
        with self.assertRaisesRegex(RuntimeError, 'reference drift'): home.no_drift(a, b)
        window = home.StableWindow(); window.add(a, 100.)
        self.assertFalse(window.add(b, 100.1)); self.assertEqual(len(window.samples), 1)
        a['opening_m'] = .055001
        with self.assertRaisesRegex(RuntimeError, 'jaw'): home.health(a, 100.)

    def test_stable_endpoint_contracts_or_revokes_exception(self):
        # One exceptional measured joint may reach legal zero in <=1 degree;
        # its intermediate outside-nominal targets remain forbidden.
        initial = sample(q=[2494, -732, -2906, -4229, 21874, 4963])
        session = home.new_session({}, initial)
        plan = home.make_plan(2, 1, initial, session)
        end = sample(q=plan['target_raw']); window = home.StableWindow()
        window.samples.append((1., end))
        home.commit_step(session, plan, window, end)
        self.assertEqual(session['exception_caps_raw'], [0, 0])
        self.assertEqual(session['completed_steps'], 1)

    def test_real_failed_step_outside_targets_now_refused_before_publication(self):
        # Recorded request: J2=-1732 and selected J3 target=+1906 mdeg.
        # Do not assert an SDK/firmware clipping cause; only the failed contract.
        before = sample(q=[2494, -1732, 2906, -4227, 21874, 4977])
        session = home.new_session({}, before)
        home.check_session(before, session)  # Observation exception is retained.
        with self.assertRaisesRegex(RuntimeError, 'manufacturer nominal'):
            home.make_plan(3, 1, before, session)
        # Selected-only outside target and preserved-other outside target both reject.
        for q, axis in (([2494, -1732, -2906, -4229, 21874, 4963], 2),
                        ([2494, -732, -2906, -4229, 21874, 4963], 3),
                        ([2494, 1732, 1906, -4229, 21874, 4963], 3)):
            measured = sample(q=q)
            with self.assertRaisesRegex(RuntimeError, 'manufacturer nominal'):
                home.make_plan(axis, 1, measured, home.new_session({}, measured))

    def test_health_staleness_driver_fault_and_jaw_changes_reject(self):
        initial = sample(); session = home.new_session({}, initial)
        for key, value in (('fault', 1), ('arm_status', 4), ('driver_codes', [64]*5+[0]),
                           ('jaw_code', 1), ('teach_status', 1), ('ctrl_mode', 0)):
            s = copy.deepcopy(initial); s[key] = value
            with self.assertRaises(RuntimeError): home.health(s, 100.)
        with self.assertRaisesRegex(RuntimeError, '100ms'): home.health(initial, 100.101)
        with self.assertRaises(RuntimeError): home.health(initial, 99.)
        for key, value in (('jaw_code', 64), ('opening_m', .020501)):
            s = copy.deepcopy(initial); s[key] = value
            with self.assertRaisesRegex(RuntimeError, 'Jaw'): home.check_session(s, session)

    def test_single_axis_observed_envelope_does_not_allow_other_joint_motion(self):
        initial = legal_sample(); session = home.new_session({}, initial); plan = home.make_plan(3, 1, initial, session)
        s = sample(q=plan['target_raw']); home.moving_check(s, initial, plan, session)
        s['raw_q'][3] += 173; s['q'] = [v*home.RAD for v in s['raw_q']]
        with self.assertRaisesRegex(RuntimeError, 'Unselected'): home.moving_check(s, initial, plan, session)
        s = sample(q=plan['target_raw']); s['pose'][2] += .030001
        with self.assertRaisesRegex(RuntimeError, '30mm'): home.moving_check(s, initial, plan, session)

    def test_stability_requires_three_seconds_all_new_fragments_and_original_span(self):
        window = home.StableWindow()
        self.assertFalse(window.add(sample(100., 1), 100.))
        with self.assertRaisesRegex(RuntimeError, 'advance'): window.add(sample(100., 1), 100.1)
        window = home.StableWindow()
        for i in range(60): self.assertFalse(window.add(sample(100+i*.05, i+1), 100+i*.05))
        self.assertTrue(window.add(sample(103., 61), 103.))
        window = home.StableWindow()
        for i in range(100):
            q = [2494, -1732, 2906, -4229+(205 if i % 2 else 0), 21874, 4963]
            self.assertFalse(window.add(sample(100+i*.05, i+1, q), 100+i*.05))

    def test_fk_checks_are_pure_and_preserve_original_workspace_caps(self):
        before = legal_sample(); session = home.new_session({}, before); plan = home.make_plan(3, 1, before, session)
        result = home.check_path(plan, before, fk=lambda _: list(before['pose']))
        self.assertEqual(len(result['samples']), 21)
        with self.assertRaisesRegex(RuntimeError, 'disagrees'):
            home.check_path(plan, before, fk=lambda _: [.2]+before['pose'][1:])
        def crosses(q):
            pose = list(before['pose']); pose[0] += abs(q[2]-before['q'][2])*2
            return pose
        with self.assertRaisesRegex(RuntimeError, '30mm'): home.check_path(plan, before, fk=crosses)

    def test_budget_pending_and_failure_are_fail_closed(self):
        s = sample(); session = home.new_session({}, s)
        for key, value in (('attempts', 64), ('pending', {'axis': 2}), ('failure', {'reason': 'prior'})):
            altered = copy.deepcopy(session); altered[key] = value
            with self.assertRaises(RuntimeError): home.make_plan(2, 1, s, altered)

    def transaction(self, *, fail=None, jitter=False, outside=False):
        with tempfile.TemporaryDirectory() as directory:
            io = FakeIO(directory, fail=fail, jitter=jitter)
            if outside: io.base = sample(q=[2494, -1732, 2906, -4227, 21874, 4977])
            io.state_path = Path(directory)/'session.json'
            result, trace = {'publish_attempts': 0}, []
            with home.SessionFile(io.state_path) as store:
                if fail or jitter or outside:
                    with self.assertRaises((RuntimeError, TimeoutError, OSError)):
                        home.run_step(io, store, io.owner, 3, 1., trace.append, result,
                                      clock=io.clock, path_check=lambda *a: {'offline_fake': True})
                else:
                    home.run_step(io, store, io.owner, 3, 1., trace.append, result,
                                  clock=io.clock, path_check=lambda *a: {'offline_fake': True})
                persisted = store.load()
                if fail or jitter or outside:
                    with self.assertRaisesRegex(RuntimeError, 'latched'):
                        home.run_step(io, store, io.owner, 1, 1., trace.append, {},
                                      clock=io.clock, path_check=lambda *a: {})
            return io, result, persisted, trace

    def test_transaction_persists_before_single_publish_and_waits_real_three_seconds(self):
        io, result, persisted, trace = self.transaction()
        self.assertEqual(len(io.published), 1)
        self.assertEqual(result['status'], 'single_home_step_arrived')
        self.assertTrue(result['arrival_confirmed'])
        self.assertIsNone(persisted['pending']); self.assertIsNone(persisted['failure'])
        self.assertEqual(persisted['attempts'], 1)
        self.assertEqual(persisted['exception_caps_raw'], [0, 0])
        self.assertGreaterEqual(result['after']['stamps'][0]-result['command_at'], 3.)
        self.assertFalse(result['can_delivery_receipt'])

    def test_timeout_or_ambiguous_publish_or_drift_latches_without_retry(self):
        for fail in ('timeout', 'publish', 'other_axis', 'missing_intent'):
            with self.subTest(fail=fail):
                io, result, persisted, trace = self.transaction(fail=fail)
                self.assertEqual(len(io.published), 1)
                self.assertEqual(result['publish_attempts'], 1)
                self.assertIsNotNone(persisted['pending'])
                self.assertTrue(persisted['failure']['target_uncertain'])

    def test_observed_205_mdeg_stationary_jitter_refuses_before_publication(self):
        io, result, persisted, trace = self.transaction(jitter=True)
        self.assertEqual(io.published, [])
        self.assertEqual(result['publish_attempts'], 0)
        self.assertEqual(persisted['attempts'], 0)
        self.assertFalse(persisted['failure']['target_uncertain'])

    def test_failed_real_start_replay_in_offline_transport_sends_zero_messages(self):
        io, result, persisted, trace = self.transaction(outside=True)
        self.assertEqual(io.published, [])
        self.assertEqual(result['publish_attempts'], 0)
        self.assertEqual(persisted['attempts'], 0)
        self.assertIsNone(persisted['pending'])
        self.assertIn('manufacturer nominal', persisted['failure']['reason'])

    def test_session_lock_prevents_parallel_clients_and_atomic_save_roundtrips(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'session.json'
            with home.SessionFile(path) as first:
                first.save({'pending': {'axis': 3}})
                self.assertEqual(first.load(), {'pending': {'axis': 3}})
                with self.assertRaises(BlockingIOError):
                    with home.SessionFile(path): pass

    def test_driver_log_intent_is_never_promoted_to_can_receipt(self):
        good = dict(event='motion_mode_before_sdk_call', move_mode=1, ctrl_mode=1,
                    requested_speed_percent=1, effective_speed_percent=1, is_mit_mode=0)
        home.check_intent([good], require_seen=True)
        for events in ([], [good, good], [dict(good, move_mode=0)], [dict(good, effective_speed_percent=2)],
                       [dict(good, event='explicit_enable_before_sdk_call')]):
            with self.assertRaises(RuntimeError): home.check_intent(events, require_seen=True)

    def test_log_cursor_does_not_skip_appends_after_audit_or_incomplete_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'driver.log'
            first = json.dumps({'source': 'ros_low_speed_entry', 'event': 'first'})+'\n'
            path.write_text(first)
            events, cursor = home.mode_events(path, 0)
            self.assertEqual([e['event'] for e in events], ['first'])
            with path.open('a') as out:
                out.write('{"source":"ros_low_speed_entry","event":"second"')
            events, partial_cursor = home.mode_events(path, cursor)
            self.assertEqual(events, []); self.assertEqual(partial_cursor, cursor)
            with path.open('a') as out: out.write('}\n')
            events, next_cursor = home.mode_events(path, cursor)
            self.assertEqual([e['event'] for e in events], ['second'])
            self.assertGreater(next_cursor, cursor)
            # The caller commits next_cursor, never a later stat().st_size.
            with path.open('a') as out:
                out.write(json.dumps({'source': 'ros_low_speed_entry', 'event': 'third'})+'\n')
            events, _ = home.mode_events(path, next_cursor)
            self.assertEqual([e['event'] for e in events], ['third'])


if __name__ == '__main__': unittest.main()
