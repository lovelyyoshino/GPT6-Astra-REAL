"""One manufacturer zero calibration of an operator-confirmed closed empty jaw.

The selected jaw stays disabled. No position, force, enable, body or peer command
is available. A durable per-boot claim precedes the sole exact calibration frame.
Unknown results latch the shared platform; this is not a task fault recovery.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
import time

from . import arms
from .takeover import _Takeover, _Startup, SIDES, LIMITS
from .single_gripper_prepare import _SingleGripperPrepare
from .linear_hold import _LinearHold
from .feedback_tolerance import PROFILE_KEY, validate_policy, joints_within, rotation_tolerance
from .startup_reset import locks, boot_identity

TABLE = "pair_gripper_zero_calibrations"
CONFIRMATION = "已完全合拢、两指接触"
ACK_TRACE_LIMIT = 64


def classify_ack(arm, frames, overflow, send_started_at, send_returned_at, sdk_returned_at):
    """Describe raw observations, without inferring a missing frame was not sent."""
    matching = []
    rejected = []
    for frame in frames:
        data = bytes.fromhex(frame['data_hex'])
        stamps = (frame['source_timestamp_s'], frame['host_received_at_s'])
        valid = (frame['arm'] == arm and frame['arbitration_id'] == 0x476
                 and frame['dlc'] == 8 and len(data) == 8
                 and not any(frame[k] for k in ('is_extended_id', 'is_remote_frame',
                     'is_error_frame', 'is_fd', 'bitrate_switch', 'error_state_indicator'))
                 and send_started_at is not None and send_returned_at is not None
                 and sdk_returned_at is not None
                 and send_started_at <= send_returned_at <= sdk_returned_at
                 and all(type(t) in (int, float) and math.isfinite(t)
                         and send_started_at <= t <= sdk_returned_at for t in stamps)
                 and stamps[0] <= stamps[1])
        if valid and data[0] == 0x75 and data[1] in (0, 1):
            matching.append(data[1])
        else:
            rejected.append(frame)
    if overflow:
        status = 'ack_trace_overflow'
    elif 0 in matching and 1 in matching:
        status = 'contradictory_matching_ack_observed'
    elif 0 in matching:
        status = 'matching_negative_ack_observed'
    elif 1 in matching:
        status = 'matching_success_ack_observed'
    else:
        status = 'no_matching_ack_observed'
    return dict(status=status, matching_count=len(matching), rejected_frames=rejected,
                expected_can_id=0x476, expected_instruction_index=0x75,
                interpretation='SDK protocol ACK interpretation; does not prove physical motion')


def provenance(profile):
    """Freeze current configuration and sources for future receipts only."""
    driver = Path(profile['sdk_path'])/'pyAgxArm/protocols/can_protocol/drivers/effector/agx_gripper/default/driver.py'
    paths = (Path(__file__), Path(__file__).with_name('takeover.py'), driver)
    return dict(profile=copy.deepcopy(profile),
        source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})


class _Zero(_Takeover):
    CONTROL_MODES = (1,)
    REQUIRE_ENABLED = False
    FRAME_KINDS = ("calibrate",)
    SCOPE = "One disabled, empty, physically closed jaw zero calibration; no movement or enable"

    def __init__(self, profile, journal, arm):
        super().__init__(profile, journal)
        self.arm = arm
        self.flags = {}
        self.zero_observed = False
        self.feedback_policy = validate_policy(profile.get(PROFILE_KEY))
        self.quaternion = _SingleGripperPrepare._manufacturer_quaternion
        # The SDK waits for RX while holding the dispatch lock. Never acquire
        # that lock here or write the journal from the receive callback.
        self.ack_lock = threading.Lock()
        self.ack_frames = []
        self.ack_overflow = 0
        self.send_started_at = self.send_returned_at = self.sdk_returned_at = None
        self.report.update(operation="calibrate_empty_gripper_zero", arm=arm,
            ack_evidence_version=1,
            calibration_acknowledged=False, gripper_enabled_after=False,
            physical_stop_verified=None, grasp_verified=False,
            mode_frame_can_activate_cached_target=False,
            calibration_is_coordinate_change_not_measured_motion=True)

    def _wrap_comm(self, side, comm):
        super()._wrap_comm(side, comm)
        inherited_callback = comm.get_callback()
        def observed(frame):
            if frame.arbitration_id == 0x476:
                record = dict(arm=side, channel=self.profile['arms'][side]['channel'],
                    arbitration_id=frame.arbitration_id, data_hex=bytes(frame.data).hex(),
                    dlc=frame.dlc, source_timestamp_s=frame.timestamp,
                    host_received_at_s=time.time(), **{k:getattr(frame, k) for k in
                        ('is_extended_id', 'is_remote_frame', 'is_error_frame',
                         'is_fd', 'bitrate_switch', 'error_state_indicator')})
                with self.ack_lock:
                    if len(self.ack_frames) < ACK_TRACE_LIMIT:
                        self.ack_frames.append(record)
                    else:
                        self.ack_overflow += 1
            inherited_callback(frame)
        comm.set_callback(observed)

    def ack_evidence(self):
        with self.ack_lock:
            frames, overflow = copy.deepcopy(self.ack_frames), self.ack_overflow
        return dict(raw_frames=frames, overflow_count=overflow,
            guarded_send_started_at_s=self.send_started_at,
            guarded_send_returned_at_s=self.send_returned_at,
            sdk_returned_at_s=self.sdk_returned_at,
            timestamps_are_host_observations_not_wire_transmission=True,
            classification=classify_ack(self.arm, frames, overflow,
                self.send_started_at, self.send_returned_at, self.sdk_returned_at))

    def connect(self):
        super().connect()
        for side, robot in self.robots.items():
            if self.feedback_policy is not None:
                from .coherent_feedback import install
                if self.profile['arms'][side]['model'] != 'piper_x':
                    raise RuntimeError('Bounded feedback profile requires PiPER X')
                install(robot)
            robot._send_msg = lambda *a, _s=side, **kw: self._deny(_s, 'Arm TX forbidden during jaw zero calibration')
            robot._send_msgs = robot._send_msg
        jaw = self.grippers[self.arm]
        jaw._send_msg = type(jaw)._send_msg.__get__(jaw, type(jaw))

    def check_enable_state(self, side, state):
        flags = _Startup.enable_flags(state)
        if any(type(v) is not bool for v in flags) or not all(flags[:6]):
            raise RuntimeError('Six enabled joints and seven known flags required')
        self.flags.setdefault(side, flags[:])
        if flags != self.flags[side] or side == self.arm and flags[6]:
            raise RuntimeError('Calibration requires selected jaw disabled and both arms enable flags unchanged')
        width = state['gripper']['width_m']
        if type(width) not in (int, float) or not math.isfinite(width):
            raise RuntimeError('Finite actual jaw feedback required')
        if side == self.arm and not self.counts[side]['attempted_frames'] and not -.010 <= width <= .001:
            raise RuntimeError('Closed-jaw zero correction exceeds the dedicated 10 mm calibration envelope')

    def check_drift(self, side, current, origin):
        q = max(abs(a-b) for a,b in zip(current['joints_rad'], origin['joints_rad']))
        position = math.dist(current['pose_m_rad'][:3], origin['pose_m_rad'][:3])
        jaw = abs(current['gripper']['width_m']-origin['gripper']['width_m'])
        for key, value in (('joint_rad',q),('position_m',position),('gripper_m',jaw)):
            self.max_drift[side][key] = max(self.max_drift[side][key], value)
        rotation = _LinearHold.rotation_distance(self, current['pose_m_rad'], origin['pose_m_rad'])
        if (not joints_within(self.feedback_policy, side, current['joints_rad'], origin['joints_rad'])
                or position > LIMITS['position_m'] or rotation > rotation_tolerance(self.feedback_policy, side)):
            raise RuntimeError(side+' body drift during calibration')
        width = current['gripper']['width_m']
        if side == self.arm and self.counts[side]['attempted_frames']:
            if abs(width) <= .001:
                self.zero_observed = True
            elif self.zero_observed or jaw > .0005:
                raise RuntimeError('Calibration feedback is neither the original offset nor the new zero')
        elif jaw > (.0005 if side == self.arm else LIMITS['gripper_m']):
            raise RuntimeError(side+' jaw changed outside its calibration')

    def frame_spec(self, kind, side):
        if side != self.arm or kind != 'calibrate':
            raise RuntimeError('Only the selected jaw calibration frame is available')
        return 0x159, bytes.fromhex('00000000000000ae')

    def _wrap_bus(self, side, comm):
        super()._wrap_bus(side, comm)
        guarded = comm.send_bus.send
        def fresh_send(frame, *args, **kwargs):
            # Runs after the durable claim and immediately before the final TX
            # whitelist. The snapshots retain the original received timestamps.
            state = {s:arms.snapshot(self.robots[s], self.grippers[s]) for s in SIDES}
            self.checked(state)
            now = time.time()
            if any(now-t > .05 or t > now for s in SIDES for t in state[s]['fragment_timestamps_s'].values()):
                raise RuntimeError('Calibration feedback exceeds 50 ms before send')
            if self.send_started_at is None:
                self.send_started_at = time.time()
            result = guarded(frame, *args, **kwargs)
            self.send_returned_at = time.time()
            return result
        comm.send_bus.send = fresh_send

    def prepare(self):
        super().prepare()
        self.observe_window(2.)  # Three seconds total before one calibration.

    def perform(self):
        self.emit('gripper_zero_intent', dict(arm=self.arm, before_width_m=self.anchor[self.arm]['gripper']['width_m'],
            arbitration_id=0x159, data_hex='00000000000000ae', sdk_api='calibrate_gripper(timeout=1.0)'))
        self.checked(self.read())
        sdk_returned_at, acknowledged = self.send_one(self.arm, 'calibrate',
            lambda:self.grippers[self.arm].calibrate_gripper(timeout=1.))
        self.sdk_returned_at = sdk_returned_at
        evidence = self.ack_evidence()
        status = evidence['classification']['status']
        accepted = acknowledged is True and status == 'matching_success_ack_observed'
        self.report.update(calibration_acknowledged=accepted, sdk_ack_result=acknowledged,
                           ack_evidence=evidence)
        self.emit('gripper_zero_returned', dict(arm=self.arm, sdk_returned_at_s=sdk_returned_at,
            sdk_ack_result=acknowledged, calibration_acknowledged=accepted,
            ack_evidence=evidence, transmission_counts=self.counts))
        if not accepted:
            raise RuntimeError('Calibration not acknowledged: SDK result=%r; raw ACK=%s; no retry'
                               % (acknowledged, status))
        final = self.observe_window(3., requested_side=self.arm, sent_at=sdk_returned_at)
        if not self.zero_observed or abs(final[self.arm]['gripper']['width_m']) > .001:
            raise RuntimeError('Fresh zero feedback not observed after acknowledged calibration')
        self.report['after_width_m'] = final[self.arm]['gripper']['width_m']
        return 'zero_calibration_acknowledged_and_observed_jaw_still_disabled'

    def run(self):
        result = super().run()
        # Preserve diagnostics even when send/SDK raises before returning.
        # The decision snapshot above remains immutable; late RX is separate.
        result['ack_trace_at_close'] = self.ack_evidence()
        if result['ok'] and result['ack_trace_at_close']['overflow_count']:
            result.update(ok=False, status='ack_trace_overflow')
            result['errors'].append(dict(type='ack_trace_overflow', detail='Raw ACK trace incomplete; no retry'))
        return result


def audit_completed(db):
    """Return completed maintenance evidence for a subsequent initial task."""
    if not db.execute("SELECT 1 FROM sqlite_master WHERE name=?", (TABLE,)).fetchone():
        return []
    evidence = []
    for row in db.execute('SELECT * FROM '+TABLE+' ORDER BY started_at'):
        if row['status'] != 'complete' or row['boot_id'] != boot_identity()['boot_id']:
            raise RuntimeError('Unresolved or foreign-boot jaw calibration cannot admit a new task')
        path = Path(row['record_path']); raw = path.read_bytes(); result = json.loads(raw)
        if (hashlib.sha256(raw).hexdigest() != row['result_sha256'] or result.get('ok') is not True
                or result.get('calibration_acknowledged') is not True or result.get('hardware_commands_sent') != 1
                or result.get('gripper_enabled_after') is not False or result.get('run_id') != row['run_id']
                or result.get('arm') != row['arm']):
            raise RuntimeError('Completed jaw calibration receipt differs')
        arm = row['arm']
        if result.get('ack_evidence_version') == 1:
            ack = result.get('ack_evidence', {})
            observed = classify_ack(arm, ack.get('raw_frames', []), ack.get('overflow_count', 0),
                ack.get('guarded_send_started_at_s'), ack.get('guarded_send_returned_at_s'),
                ack.get('sdk_returned_at_s'))
            if result.get('sdk_ack_result') is not True or observed['status'] != 'matching_success_ack_observed':
                raise RuntimeError('Completed calibration lacks fresh raw ACK evidence')
        expected = {s:dict(attempted_frames=int(s == arm),sent_frames=int(s == arm),blocked_frames=0) for s in SIDES}
        if (result.get('transmission_counts') != expected or result.get('errors') != []
                or result.get('guard_violations') != []
                or any(result.get(k) != 0 for k in ('target_commands_sent','enable_commands_sent','stop_commands_sent','retries'))
                or result.get('status') != 'zero_calibration_acknowledged_and_observed_jaw_still_disabled'
                or abs(result.get('after_width_m', float('inf'))) > .001
                or result.get('after',{}).get(arm,{}).get('gripper',{}).get('foc_status',{}).get('driver_enable_status') is not False):
            raise RuntimeError('Calibration send scope or disabled zero feedback differs')
        evidence.append(dict(row))
    return evidence


def run(service, arm, closed_empty_jaw_statement, empty_jaw_observation,
        feedback_observation_statement=None):
    if arm not in SIDES or closed_empty_jaw_statement != CONFIRMATION:
        raise ValueError('Current operator confirmation of fully closed empty fingers required')
    if not isinstance(empty_jaw_observation,str) or not 1 <= len(empty_jaw_observation.strip()) <= 4000:
        raise ValueError('Describe current RGB evidence that the selected jaw is empty')
    if service.pair_host is not None:
        raise RuntimeError('Close the idle task host before calibration; no simultaneous controller')
    from .service import _write
    from .execution import Journal
    from .pair_ledger import platform_state
    profile = copy.deepcopy(service.profile)
    if feedback_observation_statement is not None:
        profile[PROFILE_KEY] = validate_policy(dict(profile='right_j4_bounded_v1', source='user',
                                                  statement=feedback_observation_statement))
    database = service.runs/'pair_sessions.sqlite'
    with locks(service.root):
        state = platform_state(database)
        if state is None or state['owner'] or state['fault'] or state['pending_events']:
            raise RuntimeError('Clean unowned platform required for calibration')
        with sqlite3.connect(database) as db:
            if db.execute('SELECT COUNT(*) FROM pair_runs').fetchone()[0]:
                raise RuntimeError('Calibration entry is limited to before the first task on this clean platform')
            if db.execute("SELECT 1 FROM sqlite_master WHERE name=?", (TABLE,)).fetchone():
                if db.execute('SELECT 1 FROM '+TABLE+' WHERE boot_id=? AND arm=?',
                              (boot_identity()['boot_id'],arm)).fetchone():
                    raise RuntimeError('Calibration already attempted in this boot; never repeat')
        boot = boot_identity()['boot_id']
        run_id, directory = service._new_run('gripper_zero')
        result_path = directory/'result.json'
        frozen_provenance = provenance(profile)
        _write(directory/'request.json', dict(run_id=run_id, arm=arm, boot_id=boot,
            closed_empty_jaw_statement=closed_empty_jaw_statement, empty_jaw_observation=empty_jaw_observation,
            feedback_observation=profile.get(PROFILE_KEY), provenance=frozen_provenance,
            ack_evidence_version=1))
        journal = Journal(directory); claimed = False
        def record(event, data):
            nonlocal claimed
            if event == 'gripper_zero_intent':
                with sqlite3.connect(database) as db:
                    db.execute('PRAGMA synchronous=FULL'); db.execute('BEGIN IMMEDIATE')
                    db.execute('CREATE TABLE IF NOT EXISTS '+TABLE+' (boot_id TEXT, arm TEXT, run_id TEXT UNIQUE, '
                               'status TEXT, record_path TEXT, started_at REAL, result_sha256 TEXT, fault_id INTEGER, PRIMARY KEY(boot_id,arm))')
                    if db.execute('SELECT 1 FROM '+TABLE+' WHERE boot_id=? AND arm=?',(boot,arm)).fetchone():
                        raise RuntimeError('Calibration already attempted in this boot; never repeat')
                    cur = db.execute('INSERT INTO pair_faults(run_id,owner,reason,at) VALUES(?,NULL,?,?)',
                                     (run_id,'gripper_zero_in_progress',time.time()))
                    if db.execute('UPDATE pair_scope SET fault_id=? WHERE id=1 AND fault_id IS NULL AND owner IS NULL',
                                  (cur.lastrowid,)).rowcount != 1:
                        raise RuntimeError('Platform changed before calibration claim')
                    db.execute('INSERT INTO '+TABLE+' VALUES(?,?,?,?,?,?,NULL,?)',
                               (boot,arm,run_id,'pending',str(result_path),time.time(),cur.lastrowid))
                claimed = True
            journal.append(event, **data)
        result = _Zero(profile, record, arm).run()
        result.update(run_id=run_id, record_path=str(result_path),
                      request_sha256=hashlib.sha256((directory/'request.json').read_bytes()).hexdigest(),
                      provenance=frozen_provenance,
                      task_motion_authorized=False, physical_stop_verified=None)
        _write(result_path,result)
        if claimed:
            with sqlite3.connect(database) as db:
                db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
                fault_id = db.execute('SELECT fault_id FROM '+TABLE+' WHERE boot_id=? AND arm=?',(boot,arm)).fetchone()[0]
                if result['ok']:
                    if db.execute('UPDATE pair_scope SET fault_id=NULL WHERE id=1 AND fault_id=?', (fault_id,)).rowcount != 1:
                        raise RuntimeError('Calibration completed but platform fault changed; pending state retained')
                db.execute('UPDATE '+TABLE+' SET status=?,result_sha256=? WHERE boot_id=? AND arm=?',
                    ('complete' if result['ok'] else 'failed',hashlib.sha256(result_path.read_bytes()).hexdigest(),boot,arm))
                if not result['ok']:
                    cur=db.execute('INSERT INTO pair_faults(run_id,owner,reason,at) VALUES(?,NULL,?,?)',
                                   (run_id,'gripper_zero_calibration_failed_or_uncertain',time.time()))
                    db.execute('UPDATE pair_scope SET fault_id=COALESCE(fault_id,?) WHERE id=1',(cur.lastrowid,))
        return result
