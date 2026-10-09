"""Explicit PiPER X wrist-limit maintenance, with no motion targets.

Uses the canonical pair ledger for ownership, once-only intent, failures and
the existing budget. The task contract remains immutable; the separately
hashed maintenance implementation is recorded in the event payload. This is
not a task-host continuation and does not transfer any target/cache/hold.
"""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
import uuid
from contextlib import ExitStack

from . import arms
from .execution import ExclusiveExecution, Journal
from .feedback_tolerance import PROFILE_KEY, task_policy
from .joint_path import SDK_COMMIT
from .model_compatibility import KNOWN_OFFICIAL_CONSTANTS
from .pair_device import GuardedPairDevice
from .pair_ledger import PairLedger, _execution_scope
from .pair_limits import _LimitsExecutor, _ACTION_FIELDS
from .pair_preparation import observe_preparation
from .pair_task_enrollment import _lock_roots, _observations
from .reboot_startup import check_processes

SCHEMA = 'piper_x_wrist_limit_maintenance_v1'
SIDES = ('left', 'right')
WRISTS = (4, 5)
CONFIG_ACK_TIMEOUT_S = 1.0  # Same bound as pinned SDK set_joint_angle_vel_limits.


def require(condition, detail):
    if not condition:
        raise RuntimeError(detail)


def manufacturer_limits(profile):
    """Extract exact degree values from the pinned manufacturer file, not a caller."""
    require(profile.get('sdk_commit_audited') == SDK_COMMIT, 'Pinned SDK commit required')
    require(all(profile['arms'][s]['model'] == 'piper_x' and
                profile['arms'][s]['firmware'] == 'default' for s in SIDES),
            'This maintenance requires confirmed PiPER X/default profiles')
    path = Path(profile['sdk_path']) / 'pyAgxArm/api/constants.py'
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == KNOWN_OFFICIAL_CONSTANTS[SDK_COMMIT],
            'Manufacturer constants hash differs')
    tree = ast.parse(raw)
    values = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'ROBOT_JOINT_LIMIT_PRESET_DEG'
                          for t in n.targets))['piper_x']
    result = {j: [round(v * 10) for v in values['joint%d' % j]] for j in WRISTS}
    require(all(v == [-890, 890] for v in result.values()), 'Unreviewed manufacturer wrist limits')
    return result, {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(),
                    'commit': SDK_COMMIT, 'units': 'tenth_degree'}


def payload(joint, limits):
    require(type(joint) is int and joint in WRISTS and limits == [-890, 890],
            'Only the pinned PiPER X J4/J5 range is available')
    return bytes([joint]) + limits[1].to_bytes(2, 'big', signed=True) + \
        limits[0].to_bytes(2, 'big', signed=True) + b'\x7f\xff\x00'


class _MaintenanceExecutor(_LimitsExecutor):
    def __init__(self, device, initial, report):
        super().__init__(device, initial, report)
        self.writing = None
        self.ack_window = None

    def collect(self, side, original, frame):
        if getattr(frame, 'arbitration_id', None) != 0x476:
            return super().collect(side, original, frame)
        with self.lock:
            window = self.ack_window
            if window is not None:
                now = time.time()
                stamp = getattr(frame, 'timestamp', None)
                raw = bytes(frame.data)
                if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp > now:
                    self.rx_error = 'Invalid configuration ACK timestamp'
                elif stamp >= window['started_at']:
                    valid = (side == window['side'] and frame.dlc == len(raw) == 8
                             and raw == b'\x74' + bytes(7) and frame.is_rx
                             and not any((frame.is_extended_id, frame.is_remote_frame, frame.is_error_frame,
                                          frame.is_fd, frame.bitrate_switch, frame.error_state_indicator)))
                    if not valid:
                        self.rx_error = 'Unexpected configuration ACK'
                    else:
                        if len(window['frames']) >= 32:
                            self.rx_error = 'Configuration ACK budget exceeded'
                        else:
                            window['frames'].append({'timestamp': stamp, 'received_at': now,
                                                     'payload_hex': raw.hex()})
        if original is not None:
            return original(frame)

    def await_configuration_ack(self, row):
        """RX-only while firmware applies a setting. No stale sample permits TX.

        The manufacturer setter also waits for 0x476/0x74 before querying.
        Original health, age/skew and fixed stationary anchors are checked
        again on fresh post-ACK feedback before the next query/write.
        """
        started = time.monotonic()
        row['ack_wait'] = {'started_at': time.time(), 'timeout_s': CONFIG_ACK_TIMEOUT_S,
                           'observation_kind': 'RX_only_no_dispatch'}
        while True:
            self.encoder_guard()
            with self.lock:
                require(not self.rx_error, self.rx_error or 'Configuration RX error')
                ack = copy.deepcopy(self.ack_window['frames'])
            states = self.action.read()
            stamps = [v for s in SIDES for v in states[s].get('fragment_timestamps_s', {}).values()]
            row['ack_wait']['latest_fragment_range'] = [min(stamps), max(stamps)] if stamps else None
            # Do not feed an old parser cache into motion admission while the
            # controller is busy. Keep it in the raw journal as gap evidence.
            if ack and stamps and all(states[s].get('status') == 'complete' for s in SIDES):
                received = ack[0]['timestamp']
                if min(stamps) >= received:
                    self.check_states(states)
                    row['configuration_ack'] = ack
                    row['ack_wait']['finished_at'] = time.time()
                    with self.lock:
                        self.ack_window = None
                    return
            require(time.monotonic() - started < CONFIG_ACK_TIMEOUT_S,
                    'Configuration ACK or fresh post-ACK feedback missing; no retry')
            time.sleep(.01)

    def send_frame(self, side, original, frame, *args, **kwargs):
        if self.writing is None:
            return super().send_frame(side, original, frame, *args, **kwargs)
        self.encoder_guard()
        states = {s: arms.snapshot(self.action.robots[s], self.action.grippers[s]) for s in SIDES}
        self.check_states(states)
        row = self.writing
        require(side == row['side'] and frame.arbitration_id == 0x474
                and bytes(frame.data).hex() == row['data_hex']
                and row['outcome'] == 'not_attempted', 'Unexpected or repeated maintenance frame')
        row.update(outcome='uncertain', attempted_at=time.time())
        with self.lock:
            self.ack_window = {'side': side, 'started_at': row['attempted_at'], 'frames': []}
        self.action.check_freshness(states)
        result = original(frame, *args, **kwargs)
        row.update(outcome='returned', returned_at=time.time())
        return result

    def set_once(self, side, joint, limits):
        self.read()
        wire = payload(joint, limits)
        action = self.action
        action.reset_action(side, 'move', [0.] * 6)
        action.frame_limit = 1
        row = {'side': side, 'joint': joint, 'arbitration_id': 0x474,
               'data_hex': wire.hex(), 'outcome': 'not_attempted',
               'requested_limits_tenth_deg': limits, 'speed_field': 'unchanged_0x7fff'}
        self.report['writes'].append(row)
        action.emit('manufacturer_limit_write_intent', copy.deepcopy(row))
        self.read()
        with action.lock:
            require(action.ticket is None and not action.violations, 'Outstanding TX or CAN violation')
            ticket = {'side': side, 'thread': threading.get_ident(), 'kind': 'action',
                      'frames': [(0x474, wire)], 'comm_calls': 0, 'bus_calls': 0, 'error': None}
            action.ticket, self.writing = ticket, row
            try:
                # Manufacturer message/encoder, enclosed by the existing exact
                # comm AND bus guards. Public set_* may query again after an
                # early ACK; this explicit message permits exactly one write.
                robot = action.robots[side]
                robot._send_msg(robot._MSG_MotorAngleLimitMaxSpdSet(
                    joint, limits[1], limits[0], 0x7FFF))
                require(ticket['error'] is None and ticket['comm_calls'] == ticket['bus_calls'] == 1,
                        'Configuration send incomplete or uncertain')
            finally:
                action.ticket, self.writing = None, None
        action.emit('manufacturer_limit_write_returned', copy.deepcopy(row))
        self.await_configuration_ack(row)
        self.query(side, joint)
        reply = copy.deepcopy(self.report['joint_limits'][side][str(joint)])
        row['readback'] = reply
        require([reply['raw_min_angle_tenth_deg'], reply['raw_max_angle_tenth_deg']] == limits,
                'Controller did not confirm the manufacturer limit; no retry')


def reconcile(device):
    """One fixed maintenance sequence: 12 queries, up to 4 writes, readbacks.

    Both arms remain under the original stationary/health/freshness guards.
    No task-ready state or controller-limit source is installed by this tool.
    """
    action, runner, saved = device._action, None, None
    initial_totals = action.totals()
    report = {'schema': SCHEMA, 'ok': False, 'began_at': time.time(), 'errors': [],
              'joint_limits': {s: {} for s in SIDES}, 'query_receipts': {s: {} for s in SIDES},
              'controller_limits_rad': {s: [] for s in SIDES},
              'pre_or_post_window_limit_frames': dict.fromkeys(SIDES, 0), 'writes': [],
              'arm_target_commands_sent': 0, 'gripper_commands_sent': 0,
              'speed_changes_requested': False, 'zero_changes_requested': False,
              'task_motion_authorized': False, 'physical_stop_verified': None}
    try:
        limits, report['manufacturer_source'] = manufacturer_limits(action.profile)
        device._connected_usable()
        require(not device._task_ready and action.ticket is None and action._executor() is None
                and not any(action.grasps.values()), 'Idle unloaded preparation connection required')
        initial = observe_preparation(device)['arms']
        report['before_arms'] = copy.deepcopy(initial)
        # All joints must already lie within the confirmed model's bounds.
        # This is configuration reconciliation, not recovery of an illegal pose.
        for side in SIDES:
            for i, q in enumerate(initial[side]['joints_rad']):
                lo, hi = action.joint_limits[side]['joint%d' % (i + 1)]
                require(lo <= q <= hi, side + ' pose outside manufacturer model bounds')
        saved = {k: copy.deepcopy(getattr(action, k)) for k in _ACTION_FIELDS}
        runner = _MaintenanceExecutor(device, initial, report)
        action.auxiliary_executor = runner
        runner.install()
        for side in SIDES:
            for joint in range(1, 7):
                runner.query(side, joint)
        report['before_limits'] = copy.deepcopy(report['joint_limits'])
        for side in SIDES:
            for joint in WRISTS:
                old = report['before_limits'][side][str(joint)]
                bounds = [old['raw_min_angle_tenth_deg'], old['raw_max_angle_tenth_deg']]
                legacy = [-1000, 1000] if joint == 4 else [-700, 700]
                require(bounds in (limits[joint], legacy), 'Unexpected installed limit; diagnose before changing')
        for side in SIDES:
            for joint in WRISTS:
                old = report['before_limits'][side][str(joint)]
                if [old['raw_min_angle_tenth_deg'], old['raw_max_angle_tenth_deg']] != limits[joint]:
                    runner.set_once(side, joint, limits[joint])
                    current = report['joint_limits'][side][str(joint)]
                    require(current['raw_max_joint_spd'] == old['raw_max_joint_spd'],
                            'Controller speed changed unexpectedly')
        for side in SIDES:
            for joint in range(1, 7):
                runner.query(side, joint)
                old = report['before_limits'][side][str(joint)]
                new = report['joint_limits'][side][str(joint)]
                expected = limits[joint] if joint in WRISTS else [old['raw_min_angle_tenth_deg'], old['raw_max_angle_tenth_deg']]
                require([new['raw_min_angle_tenth_deg'], new['raw_max_angle_tenth_deg']] == expected
                        and new['raw_max_joint_spd'] == old['raw_max_joint_spd'],
                        'Final readback differs from the exact maintenance scope')
        report['after_arms'] = runner.read()
        report.update(ok=True, status='manufacturer_wrist_limits_readback_verified')
    except BaseException as exc:
        device._fault = str(exc)
        report['errors'].append({'type': type(exc).__name__, 'detail': str(exc)})
    finally:
        if runner is not None:
            try:
                runner.restore()
            except BaseException as exc:
                device._fault = str(exc)
                report['errors'].append({'type': type(exc).__name__, 'detail': str(exc)})
        action.auxiliary_executor, action.ticket = None, None
        if saved is not None:
            action.reset_action(saved['arm'], saved['kind'], saved['target'])
            for k, v in saved.items():
                setattr(action, k, v)
        totals = action.totals()
        report.update(ended_at=time.time(), session_transmission_counts=totals,
            transmission_counts={s: {k: v - initial_totals[s][k] for k, v in row.items()} for s, row in totals.items()},
            guard_violations=copy.deepcopy(action.violations), fault_latched=device._fault is not None)
        if report['errors'] or device._fault is not None:
            report.update(ok=False, status='maintenance_failed_no_retry')
    return copy.deepcopy(report)


def inspect_task(root):
    """Read-only, current effective scope; historical faults remain preserved."""
    path = root / 'runs/pair_sessions.sqlite'
    with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute('PRAGMA query_only=ON')
        db.execute('BEGIN')
        run = dict(db.execute('SELECT * FROM pair_runs ORDER BY started_at DESC LIMIT 1').fetchone())
        _, _, _, scope = _execution_scope(db, run['run_id'])
        require(scope['fault_id'] is None and scope['owner'] is None, 'Current scope must be cleanly detached')
        require(not db.execute("SELECT 1 FROM pair_events WHERE status='pending'").fetchone(), 'Pending task send')
        require(not db.execute("SELECT 1 FROM pair_holds WHERE status='pending'").fetchone(), 'Pending hold')
        from .grasp_episode import is_resolved_release
        for row in db.execute('SELECT state_json FROM pair_grasp_episodes'):
            state = json.loads(row[0])
            require(state['status'] == 'empty' or is_resolved_release(state), 'Unresolved object support/grasp')
        require(run['started_at'] <= time.time() < run['started_at'] + run['max_duration'], 'Original task budget expired')
        return run


def configuration_fault(root, event_id):
    """Audit a closed configuration attempt for diagnostic queries ONLY.

    The fault and owner stay in the canonical ledger. This grants no motion,
    configuration retry, enable, budget, or new task registration.
    """
    root = Path(root).resolve()
    path = root / 'runs/pair_sessions.sqlite'
    with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
        run = dict(db.execute('SELECT * FROM pair_runs ORDER BY started_at DESC LIMIT 1').fetchone())
        _, _, _, scope = _execution_scope(db, run['run_id'])
        row = db.execute('SELECT * FROM pair_events WHERE run_id=? AND event_id=?',
                         (run['run_id'], event_id)).fetchone()
        require(row is not None and row['status'] == 'complete' and row['success'] == 0
                and scope['fault_id'] is not None and scope['owner'] == row['owner'],
                'Current completed failed maintenance with its preserved owner/fault required')
        request, result = json.loads(row['payload_json']), json.loads(row['receipt_json'])
        require(request.get('kind') == 'manufacturer_configuration' and request.get('schema') == SCHEMA
                and result.get('schema') == SCHEMA and result.get('ok') is False,
                'Only configuration-maintenance diagnosis is covered')
        require(all(result.get(k) == 0 for k in ('arm_target_commands_sent', 'gripper_commands_sent'))
                and result.get('guard_violations') == [], 'Unexpected motion or CAN ownership violation')
        cleanup = result.get('cleanup', {})
        require(cleanup.get('requires_fault_latch') is False
                and cleanup.get('unresolved_gripper_probe') is None
                and cleanup.get('grasp_states') == dict.fromkeys(SIDES)
                and all(cleanup.get('arms', {}).get(s, {}).get('status') == 'disconnected' for s in SIDES),
                'Original adapter must have completed resource cleanup without grasp or pending transport')
        require(not db.execute("SELECT 1 FROM pair_events WHERE status!='complete'").fetchone(), 'Pending task event')
        require(not db.execute("SELECT 1 FROM pair_holds WHERE status!='complete'").fetchone(), 'Pending hold')
        from .grasp_episode import is_resolved_release
        for episode in db.execute('SELECT state_json FROM pair_grasp_episodes'):
            state = json.loads(episode[0])
            require(state['status'] == 'empty' or is_resolved_release(state), 'Unresolved grasp blocks independent diagnostic')
        saved = root / 'runs' / row['owner'] / 'result.json'
        require(saved.resolve().is_relative_to((root / 'runs').resolve())
                and json.loads(saved.read_text()) == result, 'Saved and canonical failed receipts must agree')
        fault = dict(db.execute('SELECT * FROM pair_faults WHERE id=?', (scope['fault_id'],)).fetchone())
        require(fault['owner'] == row['owner'] and fault['reason'] == 'execution_receipt_failed',
                'Unrelated active fault prohibits this diagnostic')
        return {'run': run, 'scope': dict(scope), 'event': dict(row), 'fault': fault,
                'result_path': str(saved), 'result_sha256': hashlib.sha256(saved.read_bytes()).hexdigest(),
                'fault_cleared': False, 'task_dispatch_authorized': False}


def inspect_after_configuration_fault(root, event_id):
    """Existing twelve-query adapter under all project locks; NEVER a write."""
    root = Path(root).resolve()
    check_processes()
    with ExitStack() as stack:
        for project in _lock_roots(root):
            stack.enter_context(ExclusiveExecution(project / 'runs'))
        check_processes()
        source = configuration_fault(root, event_id)
        contract = json.loads(source['run']['contract_json'])
        profile = json.loads((root / 'configs/robot.json').read_text())
        require(profile['arms'] == contract['arms'] and profile['cameras'] == contract['cameras'],
                'Diagnostic device binding changed')
        profile[PROFILE_KEY] = task_policy(contract['task'])
        name = 'configuration_diagnostic_' + uuid.uuid4().hex
        directory = root / 'runs' / name; directory.mkdir()
        journal = Journal(directory)
        journal.append('configuration_diagnostic_admitted', source=source,
                       scope='Twelve 0x472 queries only; fault, owner and task budget unchanged')
        started = time.monotonic()
        def guard():
            require(time.monotonic() - started < 30., 'Configuration diagnostic deadline')
            require(configuration_fault(root, event_id) == source, 'Canonical maintenance state changed')
        device = GuardedPairDevice(profile, lambda event, data: journal.append(event, **data), guard)
        result = None
        try:
            device.connect_for_preparation()
            result = device.inspect_joint_limits()
            require(result.get('ok') is True and result.get('joint_limit_queries_sent') == 12
                    and result.get('actuator_commands_sent') == 0, 'Diagnostic limit capture incomplete')
        except BaseException as exc:
            result = {'ok': False, 'error': str(exc), 'device_receipt': result}
        finally:
            cleanup = device.close()
            result = json.loads(json.dumps(result, allow_nan=False))
            result.update(source_event_id=event_id, source_fault=source['fault'], cleanup=cleanup,
                          record_path=str(directory / 'result.json'),
                          task_dispatch_authorized=False, fault_cleared=False, physical_stop_verified=None)
            result['canonical_ledger_unchanged'] = configuration_fault(root, event_id) == source
            journal.append('configuration_diagnostic_result', result=result)
            (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        return result


def execute(root, request):
    root = Path(root).resolve()
    require(set(request) == {'event_id', 'authorization', 'passive_paths', 'rgb_observation', 'visual_observation'},
            'Explicit maintenance request and current observations required')
    require(type(request['authorization']) is str and 1 <= len(request['authorization'].strip()) <= 4000,
            'Actual user maintenance instruction required')
    check_processes()
    with ExitStack() as stack:
        for project in _lock_roots(root):
            stack.enter_context(ExclusiveExecution(project / 'runs'))
        check_processes()
        run = inspect_task(root)
        contract = json.loads(run['contract_json'])
        profile = json.loads((root / 'configs/robot.json').read_text())
        require(profile['arms'] == contract['arms'] and profile['cameras'] == contract['cameras'], 'Device binding changed')
        profile[PROFILE_KEY] = task_policy(contract['task'])
        manufacturer_limits(profile)
        evidence = _observations(request['passive_paths'], request['rgb_observation'], request['visual_observation'],
                                 contract, after=run['started_at'], now=time.time())
        deadline = min(v['host_received_at'] for v in evidence['rgb']['images'].values()) + 30.
        # Preserve the task's frozen contract for accounting only. The active
        # maintenance implementation is explicitly hashed in this event.
        source_files = list(contract['code']) + ['controller_limits.py']
        code = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in source_files}
        require(all(code[k] == v for k, v in contract['code'].items()), 'Existing task source changed')
        ledger = PairLedger(root / 'runs/pair_sessions.sqlite', run['run_id'], contract,
                            max_steps=run['max_steps'], max_duration_s=run['max_duration'])
        require(ledger.event(request['event_id']) is None, 'Maintenance event already exists; never resend')
        owner = 'maintenance_' + uuid.uuid4().hex
        directory = root / 'runs' / owner
        directory.mkdir()
        journal = Journal(directory)
        ledger.claim(owner)
        event = {**request, 'kind': 'manufacturer_configuration', 'schema': SCHEMA,
                 'maintenance_code_sha256': code, 'evidence': evidence,
                 'scope': 'PiPER X J4/J5 only; no task target/cache/hold or new budget'}
        result, device, claimed = None, None, False
        try:
            ledger.begin(owner, request['event_id'], event)
            claimed = True
            def guard():
                state = ledger.status()
                require(state['owner'] == owner and not state['fault_latched'], 'Lost maintenance ownership or fault')
                require(time.time() < deadline, 'Current RGB maintenance window expired')
            journal.append('maintenance_claimed', payload=event)
            device = GuardedPairDevice(profile, lambda event, data: journal.append(event, **data), guard)
            device.connect_for_preparation()
            result = reconcile(device)
        except BaseException as exc:
            result = {'ok': False, 'status': 'maintenance_failed_no_retry', 'error': str(exc)}
        finally:
            if device is not None:
                try:
                    result['cleanup'] = device.close()
                    if result['cleanup'].get('requires_fault_latch'):
                        result['ok'] = False
                except BaseException as exc:
                    result.update(ok=False, cleanup_error=str(exc))
            if result is not None:
                result.update(run_id=run['run_id'], owner=owner, event_id=request['event_id'],
                              record_path=str(directory / 'result.json'), physical_stop_verified=None)
                # Vendor feedback contains IntEnum values. Persist exactly the
                # same JSON-normalized receipt in the journal, file and ledger;
                # PairLedger intentionally rejects non-plain Python JSON types.
                result = json.loads(json.dumps(result, ensure_ascii=False, allow_nan=False))
                journal.append('maintenance_result', result=result)
                (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
            if claimed:
                ledger.finish(owner, request['event_id'], result, success=result['ok'])
            if result and result['ok']:
                ledger.release(owner)
            elif not claimed:
                ledger.fault(owner, 'Maintenance claim failed')
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--request', type=Path)
    group.add_argument('--inspect-after', help='Query only after this preserved failed configuration event')
    args = parser.parse_args()
    result = (inspect_after_configuration_fault(Path(__file__).parents[1], args.inspect_after)
              if args.inspect_after else execute(Path(__file__).parents[1], json.loads(args.request.read_text())))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
