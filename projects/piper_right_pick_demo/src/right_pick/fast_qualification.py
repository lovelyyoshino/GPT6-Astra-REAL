"""Offline validation of a narrowly scoped, recorded ROS commissioning trial.

No hardware imports or operations. Raw frames are decoded again here rather than
trusting reported qualification flags. A valid record supports only 1% bounded
goal completion after client exit and stationary retention after driver exit;
it does not demonstrate cancellation, instantaneous stopping or power-loss hold.
The caller must independently establish provenance, ownership and path clearance.
"""
import hashlib
import json
import math
import re
import struct


class QualificationError(ValueError):
    pass


SCOPE = 'bounded_goal_completion_on_client_exit_and_static_driver_exit'
FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
RAD_PER_RAW = math.pi / 180000
CAPS = dict(speed_percent=1, max_translation_m=.03, max_rotation_rad=.05,
            max_feedback_age_s=.1)
JOINT_LIMITS_RAW = ((-150000, 150000), (0, 180000), (-170000, 0),
                    (-100000, 100000), (-70000, 70000), (-120000, 120000))


def _fail(message):
    raise QualificationError(message)


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        _fail(name + ' must be finite numeric')
    return float(value)


def _vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        _fail(name + ' must contain six numbers')
    return [_number(x, name) for x in value]


def _rotation(a, b):
    def quat(p):
        r, p, y = (x / 2 for x in p[3:])
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
    qa, qb = quat(a), quat(b)
    if sum(x*y for x, y in zip(qa, qb)) < 0:
        qb = [-x for x in qb]
    return 4 * math.atan2(math.sqrt(sum((x-y)**2 for x, y in zip(qa, qb))),
                         math.sqrt(sum((x+y)**2 for x, y in zip(qa, qb))))


def _distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a[:3], b[:3])))


def _frame(frame):
    if not isinstance(frame, dict) or type(frame.get('id')) is not int:
        _fail('Raw frame requires an integer id')
    stamp = _number(frame.get('timestamp'), 'frame timestamp')
    payload = frame.get('data_hex')
    if (stamp <= 0 or not isinstance(payload, str)
            or re.fullmatch(r'[0-9a-fA-F]{16}', payload) is None):
        _fail('Raw frame requires positive timestamp and exactly eight bytes')
    return frame['id'], stamp, bytes.fromhex(payload)


def _sample(value):
    if not isinstance(value, dict) or not isinstance(value.get('frames'), list):
        _fail('Each sample requires frames')
    sampled = _number(value.get('sampled_at'), 'sampled_at')
    frames = {}
    for frame in value['frames']:
        frame_id, stamp, data = _frame(frame)
        if frame_id in frames:
            _fail('Duplicate feedback frame')
        frames[frame_id] = (stamp, data)
    if set(frames) != set(FEEDBACK_IDS):
        _fail('Exactly fourteen raw feedback frames required')
    stamps = [frames[k][0] for k in FEEDBACK_IDS]
    if (max(stamps) > sampled or sampled-min(stamps) > .1 + 1e-9
            or max(stamps)-min(stamps) > .1 + 1e-9):
        _fail('Feedback stale, future dated or skewed')
    data = lambda key: frames[key][1]
    status, jaw = data(0x2A1), data(0x2A8)
    if (status[0] != 1 or status[1] != 0 or status[2] not in (0, 1)
            or status[3] != 0 or status[4] not in (0, 1)
            or int.from_bytes(status[6:8], 'big') != 0
            or any(data(k)[5] != 64 for k in range(0x261, 0x267))
            or jaw[6] != 64):
        _fail('Raw feedback is not healthy, enabled ordinary CAN arm and jaw')
    raw_q = sum((list(struct.unpack('>ii', data(k))) for k in range(0x2A5, 0x2A8)), [])
    raw_p = sum((list(struct.unpack('>ii', data(k))) for k in range(0x2A2, 0x2A5)), [])
    # Target bounds remain exact. Feedback limits here reject actual nominal
    # excursions too; no SDK clamping or noise tolerance silently changes them.
    if any(not lo <= q <= hi for q, (lo, hi) in zip(raw_q, JOINT_LIMITS_RAW)):
        _fail('Raw joint feedback outside nominal joint limits')
    opening = struct.unpack('>i', jaw[:4])[0] / 1e6
    if not 0 <= opening <= .055:
        _fail('Jaw outside commissioned width envelope')
    pose = [v/1e6 for v in raw_p[:3]] + [v*RAD_PER_RAW for v in raw_p[3:]]
    if any(abs(v) > 1 for v in pose[:3]):
        _fail('Implausible position feedback')
    return dict(t=sampled, stamps=stamps, q=[v*RAD_PER_RAW for v in raw_q],
                pose=pose, opening=opening, mode=status[2], moving=status[4],
                feedback_age_s=sampled-min(stamps))


def _samples(raw):
    if not isinstance(raw, list) or not 30 <= len(raw) <= 100000:
        _fail('A bounded, sufficiently dense raw sample history is required')
    result = [_sample(s) for s in raw]
    for a, b in zip(result, result[1:]):
        if not 0 < b['t']-a['t'] <= .1 + 1e-9:
            _fail('Sample order or monitoring gap exceeds 100 ms')
        if any(y < x for x, y in zip(a['stamps'], b['stamps'])):
            _fail('Raw feedback receive timestamps regressed')
    return result


def _stable(values, duration):
    if not values or values[-1]['t']-values[0]['t'] < duration-1e-9:
        _fail('Insufficient stable observation duration')
    if any(s['moving'] != 0 for s in values) or len({s['mode'] for s in values}) != 1:
        _fail('Stable interval contains motion or mode changes')
    needed = 20 if duration >= 3 else 3
    if any(len({s['stamps'][i] for s in values}) < needed for i in range(14)):
        _fail('Stable interval does not contain enough fresh feedback advances')
    for field, count, limit in (('q', 6, .003), ('pose', 3, .0005)):
        if any(max(s[field][i] for s in values)-min(s[field][i] for s in values) > limit
               for i in range(count)):
            _fail('Stationary interval drift exceeds ' + field + ' envelope')
    if (any(_rotation(s['pose'], values[0]['pose']) > .003 for s in values)
            or max(s['opening'] for s in values)-min(s['opening'] for s in values) > .0005):
        _fail('Stationary orientation or jaw drift exceeds envelope')
    return values[-1]['t']-values[0]['t']


def _tail(values, duration):
    # Include the sample immediately preceding the start of the required span.
    start = len(values)-1
    while start > 0 and values[-1]['t']-values[start]['t'] < duration:
        start -= 1
    return values[start:]


def _transmissions(kind, trial, target, samples):
    raw = trial.get('control_frames')
    if not isinstance(raw, list):
        _fail('Recorded control_frames required, including an empty static list')
    observed = [_frame(f) for f in raw]
    if kind == 'static_driver_exit':
        if observed:
            _fail('Static exit trial contained a controller transmission')
        return None, 'passive_trace_with_local_loopback_visibility_limitation'
    frames = observed
    receipt = trial.get('driver_receipt')
    source = 'passively_observed_control_frames'
    if receipt is not None:
        if not isinstance(receipt, dict):
            _fail('Malformed driver receipt')
        intent, sent = receipt.get('intent'), receipt.get('sent')
        if not isinstance(intent, dict) or not isinstance(sent, dict):
            _fail('Driver intent and socket send receipt required')
        if (intent.get('source') != 'ros_resume_entry' or sent.get('source') != 'ros_resume_entry'
                or intent.get('event') != 'command_intent'
                or sent.get('event') != 'command_sent_unconfirmed'
                or intent.get('kind') != ('joint' if kind == 'J' else 'pose')
                or type(intent.get('sequence')) is not int or intent['sequence'] <= 0
                or sent.get('sequence') != intent['sequence']
                or type(sent.get('sequence')) is not int
                or type(intent.get('speed_percent')) is not int or intent['speed_percent'] != 1):
            _fail('Driver receipt identity, kind, sequence or speed mismatch')
        started = _number(intent.get('unix_s'), 'intent unix_s')
        finished = _number(sent.get('unix_s'), 'sent unix_s')
        if not 0 <= finished-started <= .1 or not isinstance(intent.get('frames'), list):
            _fail('Invalid driver transaction timing or frame intent')
        frames = [_frame(dict(f, timestamp=finished)) for f in intent['frames']]
        if any(type(sent.get(k)) is not int or sent[k] != len(frames)
               for k in ('attempted_frames', 'socket_send_returns')):
            _fail('Incomplete or partial driver socket send receipt')
        # SDK disables local loopback. Missing side-channel TX is not evidence
        # of absent TX. Any frames that *are* visible still must match in order.
        cursor = 0
        for frame_id, stamp, data in observed:
            while cursor < len(frames) and (frames[cursor][0], frames[cursor][2]) != (frame_id, data):
                cursor += 1
            if cursor == len(frames) or not started <= stamp <= finished:
                _fail('Unexpected visible control transmission outside driver receipt')
            cursor += 1
        source = 'driver_socket_send_receipt_not_bus_delivery'
    ids = [f[0] for f in frames]
    expected = ([0x151, 0x155, 0x156, 0x157] if kind == 'J' else
                [0x150, 0x151, 0x152, 0x153, 0x154, 0x159, 0x151])
    if ids != expected:
        _fail('Expected exactly one pinned vendor transaction; no retries or other sends')
    stamps = [f[1] for f in frames]
    command = _number(trial.get('command_unix_s'), 'command_unix_s')
    exited = _number(trial.get('client_exit_unix_s'), 'client_exit_unix_s')
    if (stamps != sorted(stamps) or command > min(stamps) or max(stamps) >= exited
            or max(stamps)-min(stamps) > .1 or command < samples[0]['t']
            or max(stamps) > samples[-1]['t']):
        _fail('Command frames and client exit timing do not establish a finite transaction')
    mode = 1 if kind == 'J' else 0
    by_id = {f[0]: f[2] for f in frames}
    for frame_id, _, data in frames:
        if frame_id == 0x151 and data != bytes((1, mode, 1, 0, 0, 0, 0, 0)):
            _fail('Actual mode frame must use ordinary requested mode and exactly 1% speed')
    if kind == 'P':
        if by_id[0x150] != bytes(8):
            _fail('Quick stop/reset/trajectory commands are forbidden')
        width, effort, code, zero = struct.unpack('>iHBB', by_id[0x159])
        if not 0 <= width <= 55000 or (effort, code, zero) != (200, 1, 0):
            _fail('Pose transaction jaw command exceeds commissioned envelope')
        if abs(width/1e6-samples[0]['opening']) > .0015:
            _fail('Commissioning pose transaction must preserve empty jaw opening')
    first = 0x155 if kind == 'J' else 0x152
    actual = sum((list(struct.unpack('>ii', by_id[k])) for k in range(first, first+3)), [])
    if kind == 'J':
        expected_raw = [round(v/RAD_PER_RAW) for v in target]
    else:
        expected_raw = [round(v*1000)*1000 for v in target[:3]] + [
            round(v*1000*180/3.1415926) for v in target[3:]]
    if actual != expected_raw:
        _fail('Raw commanded target does not match declared target')
    # Assess arrival against the actual transmitted quantized goal.
    return (([v*RAD_PER_RAW for v in actual] if kind == 'J' else
             [v/1e6 for v in actual[:3]] + [v*RAD_PER_RAW for v in actual[3:]]), source)


def validate_trial(kind, trial):
    """Validate one raw trial without connecting to ROS/CAN or issuing commands."""
    if kind not in ('static_driver_exit', 'J', 'P') or not isinstance(trial, dict):
        _fail('Unknown trial kind')
    samples = _samples(trial.get('samples'))
    target = None if kind == 'static_driver_exit' else _vector(trial.get('target'), 'target')
    target, receipt_source = _transmissions(kind, trial, target, samples)
    event = _number(trial.get('event_unix_s' if kind == 'static_driver_exit' else 'command_unix_s'), 'event time')
    before = [s for s in samples if max(s['stamps']) < event and s['t'] < event]
    if not before or event-before[-1]['t'] > .1:
        _fail('Missing fresh pre-event stationary baseline')
    _stable(_tail(before, .3), .3)
    initial = before[-1]
    for s in samples:
        if (_distance(s['pose'], initial['pose']) > .03
                or _rotation(s['pose'], initial['pose']) > .05
                or max(abs(a-b) for a, b in zip(s['q'], initial['q'])) > .05
                or abs(s['opening']-initial['opening']) > .0015):
            _fail('Observed trial exceeded its bounded movement envelope')
        if initial['pose'][2]-s['pose'][2] > .001:
            _fail('Commissioning trace contains more than 1 mm downward excursion')
    if kind == 'static_driver_exit':
        after = [s for s in samples if min(s['stamps']) > event]
        _stable(after, 3.)
        # A stable shifted post-exit pose is still a failed static hold.
        if any(_distance(s['pose'], initial['pose']) > .001
               or max(abs(a-b) for a, b in zip(s['q'], initial['q'])) > .003
               or _rotation(s['pose'], initial['pose']) > .003 for s in after):
            _fail('Static exit changed position before the stable observation window')
        progress = None
    else:
        if type(trial.get('speed_percent')) is not int or trial['speed_percent'] != 1:
            _fail('Only exactly 1% speed is commissioned')
        exit_time = _number(trial.get('client_exit_unix_s'), 'client_exit_unix_s')
        if exit_time <= event:
            _fail('Client must exit after command dispatch')
        during = [s for s in samples if min(s['stamps']) > event and s['t'] <= exit_time]
        if not during or exit_time-during[-1]['t'] > .1:
            _fail('Fresh motion feedback immediately before client exit is missing')
        at_exit = during[-1]
        if kind == 'J':
            if any(not lo <= round(q/RAD_PER_RAW) <= hi for q, (lo, hi) in zip(target, JOINT_LIMITS_RAW)):
                _fail('Joint target outside nominal joint limits')
            progress = max(abs(a-b) for a, b in zip(at_exit['q'], initial['q']))
            goal_error = lambda s: max(abs(a-b) for a, b in zip(s['q'], target))
            arrived = lambda s: goal_error(s) <= .003
            progressed = progress >= .003
            trend_required, continued_required = .0005, .001
            if max(abs(a-b) for a, b in zip(initial['q'], target)) > .05:
                _fail('Joint target exceeds commissioning displacement')
        else:
            if _distance(initial['pose'], target) > .03 or _rotation(initial['pose'], target) > .05:
                _fail('Pose target exceeds commissioning displacement')
            progress = _distance(at_exit['pose'], initial['pose'])
            goal_error = lambda s: _distance(s['pose'], target)
            arrived = lambda s: (goal_error(s) <= .002
                                  and _rotation(s['pose'], target) <= .02)
            progressed = progress >= .001
            trend_required, continued_required = .0003, .0005
        mode = 1 if kind == 'J' else 0
        if at_exit['mode'] != mode or arrived(at_exit) or not progressed:
            _fail('Client exit was not observed during significant unfinished motion')
        # Firmware may report motion_status=0 while raw position advances. Use
        # independently timed progress toward the actual goal, not that bit, to
        # establish that exit happened during the still-unfinished movement.
        earlier = [s for s in during if s['t'] <= at_exit['t']-.1+1e-9]
        if (not earlier or at_exit['t']-earlier[-1]['t'] > .2
                or goal_error(earlier[-1])-goal_error(at_exit) < trend_required):
            _fail('No sufficient progress toward goal in the last 100 ms before exit')
        after = [s for s in samples if min(s['stamps']) > exit_time]
        if not after or goal_error(at_exit)-min(goal_error(s) for s in after) < continued_required:
            _fail('Insufficient additional progress toward original goal after client exit')
        suffix = _tail(after, 3.)
        if any(not arrived(s) or s['mode'] != mode for s in suffix):
            _fail('Original goal was not retained throughout the final 3-second interval')
        _stable(suffix, 3.)
        after = suffix
    return dict(kind=kind, evidence_valid=True, sample_count=len(samples),
                stable_s=after[-1]['t']-after[0]['t'], progress_at_client_exit=progress,
                max_feedback_age_s=max(s['feedback_age_s'] for s in samples),
                observed_max_downward_excursion_m=max(0., max(initial['pose'][2]-s['pose'][2] for s in samples)),
                transmission_evidence_source=receipt_source,
                external_transmitter_absence_verified=False,
                target_cancelled=False, instantaneous_hold_verified=False,
                general_stop_validated=False, power_loss_hold_verified=False)


def validate_qualification(evidence, *, boot_id, adapter_sha256, vendor_sha256):
    """Require independent static/J/P evidence bound to this boot and source.

    Expected identity arguments must come from the current runtime, not the
    evidence document itself. Success never authorizes a pose/path or motion.
    """
    if not isinstance(evidence, dict) or type(evidence.get('schema_version')) is not int or evidence['schema_version'] != 1:
        _fail('Qualification schema_version must be integer 1')
    for name, expected in (('boot_id', boot_id), ('adapter_sha256', adapter_sha256),
                           ('vendor_sha256', vendor_sha256)):
        if not isinstance(expected, str) or not expected or evidence.get(name) != expected:
            _fail('Current runtime identity mismatch: ' + name)
        if name.endswith('sha256') and re.fullmatch(r'[0-9a-f]{64}', expected) is None:
            _fail('Invalid source hash')
    if (evidence.get('qualification_scope') != SCOPE or evidence.get('can_interface') != 'can1'
            or evidence.get('usb_interface') != '1-6.3:1.0'):
        _fail('Qualification scope or physical interface binding mismatch')
    for name, expected in CAPS.items():
        if type(evidence.get(name)) is bool or _number(evidence.get(name), name) != expected:
            _fail('Qualification cannot widen fixed limit ' + name)
    if type(evidence['speed_percent']) is not int:
        _fail('speed_percent must be an integer')
    trials = evidence.get('trials')
    if not isinstance(trials, dict) or set(trials) != {'static_driver_exit', 'J', 'P'}:
        _fail('Separate static_driver_exit, J and P trials are required')
    results = {kind: validate_trial(kind, trials[kind]) for kind in ('static_driver_exit', 'J', 'P')}
    digest = hashlib.sha256(json.dumps(evidence, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return dict(evidence_valid=True, qualification_scope=SCOPE, qualified_modes=['J', 'P'],
                boot_id=boot_id, adapter_sha256=adapter_sha256, vendor_sha256=vendor_sha256,
                evidence_sha256=digest, limits=dict(CAPS), trials=results,
                physical_motion_authorized=False, target_cancelled=False,
                instantaneous_hold_verified=False, general_stop_validated=False,
                power_loss_hold_verified=False,
                scope_limitations=['Recorded local trials at 1%, not a stopping guarantee',
                                   'Execution still requires independent ownership, limits and path review',
                                   'Does not qualify higher speed, CAN failure, power loss or human contact'])
