#!/usr/bin/env python3
"""One supervised near-zero maintenance step through the existing ROS owner.

This is not a task executor, an enable tool, an autonomous homing sequence, or a
hold/collision qualification. Each invocation publishes at most ONE six-joint
JointState: one caller-selected axis moves toward its existing zero by <=1 deg.
The other five targets are the latest measured joints; the jaw is never sent.
Every published target must satisfy manufacturer nominal joint limits. Initial
measurement exceptions do NOT authorize outside-limit intermediate targets;
single-axis recovery that would preserve/send such targets is unsupported.
CAN is receive-only. Timeout/fault leaves any accepted finite target with the
controller, latches the persistent session, and sends no physical recovery.
Importing this module performs no ROS, CAN, SDK, or filesystem I/O.
"""
import argparse
from collections import deque
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
ENTRY = Path('/home/agilex/piperx_cloth_demo/robot_tools/ros_low_speed_entry.py')
HEALTH = Path('/home/agilex/GPT6_bash_jiang/gpt_robot_eval_jiang/scripts/driver_with_health.py')
VENDOR = Path('/home/agilex/piper_gpt/src/piper_ros-noetic/src/piper/scripts/piper_ctrl_single_node.py')
HASHES = ((ENTRY, '3e1c6c592482f16967555a2ff3ffa37406ff0c40ebe27f3e67872cb0e3faa09b'),
          (HEALTH, 'cb60c162958c01948dd9ee0fae3b504aca1837e28b1986eb96a41cd5c85992ca'),
          (VENDOR, 'ecff6823dc6bbf55fe51708024ef88d993fead97572ad7c06e951020e83bc200'))
NODE = '/piper/right/driver'
TOPIC = '/piper/right/joint_cmd'
TELEMETRY = '/piper/right/eval_telemetry'
COMMAND_TOPICS = (TOPIC, '/piper/right/pos_cmd', '/piper/right/enable_flag')
IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
RAD = math.pi / 180000
JOINT_BOUNDS = ((-150000, 150000), (0, 180000), (-170000, 0),
                (-100000, 100000), (-70000, 70000), (-120000, 120000))
START_ABS = (45000, 10000, 10000, 15000, 30000, 15000)
JITTER_RAW = .003 / RAD
SCHEMA = 'ros_single_axis_near_zero_maintenance_v1'


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def rotation(a, b):
    def quat(pose):
        r, p, y = (v / 2 for v in pose[3:])
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    qa, qb = quat(a), quat(b)
    return 2*math.acos(min(1., abs(sum(x*y for x, y in zip(qa, qb)))))


def decode(frames, sequence):
    """Pure raw decoding identical to the reviewed resume adapter snapshot.

    Values are individual received fragments, never SDK filtered/default zeros.
    Kernel UNIX timestamps are provided by the receive-only transport below.
    """
    require(set(frames) == set(IDS), 'Incomplete fourteen-fragment feedback')
    data = lambda k: frames[k][1]
    require(all(len(data(k)) == 8 for k in IDS), 'Malformed feedback frame')
    q = sum((list(struct.unpack('>ii', data(k))) for k in range(0x2A5, 0x2A8)), [])
    pose = sum((list(struct.unpack('>ii', data(k))) for k in range(0x2A2, 0x2A5)), [])
    status, jaw = data(0x2A1), data(0x2A8)
    return dict(sequence=sequence, stamps=[frames[k][0] for k in IDS], raw_q=q,
                q=[v*RAD for v in q], pose=[v/1e6 for v in pose[:3]]+[v*RAD for v in pose[3:]],
                opening_m=struct.unpack('>i', jaw[:4])[0]/1e6, jaw_code=jaw[6],
                ctrl_mode=status[0], arm_status=status[1], mode=status[2], teach_status=status[3],
                motion_status=status[4], fault=int.from_bytes(status[6:8], 'big'),
                driver_codes=[data(k)[5] for k in range(0x261, 0x267)])


def health(s, now, *, idle=False):
    stamps = s['stamps']
    require(len(stamps) == 14 and all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in stamps),
            'Invalid kernel timestamps')
    require(max(stamps) <= now and now-min(stamps) <= .1 and max(stamps)-min(stamps) <= .1,
            'Complete kernel feedback age/skew exceeds 100ms')
    require(s['ctrl_mode'] == 1 and s['arm_status'] == 0 and s['fault'] == 0
            and s['teach_status'] == 0 and s['mode'] in (0, 1, 2)
            and s['motion_status'] in ((0,) if idle else (0, 1))
            and s['driver_codes'] == [64]*6 and s['jaw_code'] in (0, 64),
            'Requires six enabled healthy motors; jaw may be disabled but healthy')
    require(len(s['raw_q']) == 6 and all(type(v) is int for v in s['raw_q']), 'Invalid raw joints')
    require(s['q'] == [v*RAD for v in s['raw_q']], 'Raw joint conversion mismatch')
    require(len(s['pose']) == 6 and all(type(v) in (int, float) and math.isfinite(v) for v in s['pose']),
            'Invalid finite end-reference feedback')
    require(-.6 <= s['pose'][0] <= .6 and -.6 <= s['pose'][1] <= .6
            and .05 <= s['pose'][2] <= .65, 'End reference outside original site workspace')
    require(all(abs(v) <= 2*math.pi for v in s['pose'][3:]) and 0 <= s['opening_m'] <= .055,
            'Invalid pose/jaw encoding range')


def new_session(identity, before):
    q = before['raw_q']
    require(all(abs(v) <= limit for v, limit in zip(q, START_ABS)), 'Outside fixed near-home start bounds')
    for i, (v, (lo, hi)) in enumerate(zip(q, JOINT_BOUNDS)):
        require(lo <= v <= hi or (i == 1 and -5000 <= v < 0) or (i == 2 and 0 < v <= 5000),
                'Only initial J2-negative/J3-positive exceptions up to five degrees are permitted')
    return dict(schema=SCHEMA, identity=identity, initial=copy.deepcopy(before), last=copy.deepcopy(before),
                exception_caps_raw=[max(0, -q[1]), max(0, q[2])], attempts=0,
                pending=None, failure=None, completed_steps=0, history=[],
                collision_path_verified=False, hold_verified=False)


def check_session(s, session):
    require(session['schema'] == SCHEMA and not session['failure'] and session['pending'] is None,
            'Persistent session latched or an earlier publication outcome is unresolved')
    require(0 <= session['attempts'] < 64, 'Maintenance publication budget exhausted')
    for i, (v, (lo, hi)) in enumerate(zip(s['raw_q'], JOINT_BOUNDS)):
        cap = session['exception_caps_raw'][i-1] if i in (1, 2) else 0
        if lo <= v <= hi:
            continue
        require(cap > 0 and ((i == 1 and v < 0 and -v <= cap+JITTER_RAW)
                             or (i == 2 and v > 0 and v <= cap+JITTER_RAW)),
                'Joint limit exception absent, revoked, or deepened beyond .003rad')
    initial = session['initial']
    require(all(min(start, 0)-JITTER_RAW <= v <= max(start, 0)+JITTER_RAW
                for v, start in zip(s['raw_q'], initial['raw_q'])),
            'Joint left fixed initial-to-zero maintenance interval')
    require(s['jaw_code'] == initial['jaw_code'] and abs(s['opening_m']-initial['opening_m']) <= .0005,
            'Jaw state/opening changed during maintenance session')


def no_drift(a, b):
    require(max(abs(x-y) for x, y in zip(a['q'], b['q'])) <= .003, 'Joint drift exceeds .003rad')
    require(math.dist(a['pose'][:3], b['pose'][:3]) <= .0005
            and rotation(a['pose'], b['pose']) <= .003, 'End reference drift exceeds stable bounds')
    require(a['jaw_code'] == b['jaw_code'] and abs(a['opening_m']-b['opening_m']) <= .0005,
            'Jaw drift exceeds stable bounds')


def make_plan(axis, max_step_deg, before, session):
    require(type(axis) is int and 1 <= axis <= 6, 'Axis must be integer 1..6')
    require(type(max_step_deg) in (int, float) and math.isfinite(max_step_deg)
            and 0 < max_step_deg <= 1, 'Step magnitude must be within (0, 1] degrees')
    check_session(before, session)
    q = list(before['raw_q']); i = axis-1
    require(q[i] != 0, 'Selected joint is already exactly zero; no publication')
    amount = min(abs(q[i]), int(math.floor(max_step_deg*1000+1e-9)))
    require(amount > 0, 'Requested step is below one raw millidegree')
    q[i] -= (1 if q[i] > 0 else -1)*amount
    if i in (1, 2) and session['exception_caps_raw'][i-1]:
        violation = max(0, -q[i]) if i == 1 else max(0, q[i])
        require(violation < session['exception_caps_raw'][i-1], 'Exception target must strictly shrink')
    # Real home_step_001 showed an unselected joint moving when the six-target
    # request contained outside-nominal intermediates. Its downstream cause is
    # unresolved (the inspected SDK clamp is disabled by default). Measurement
    # exceptions therefore never grant permission to publish such targets.
    require(all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_BOUNDS)),
            'Every published target must be manufacturer nominal; outside-limit intermediate targets are unsupported')
    target = [v*RAD for v in q]
    # Exact pinned vendor arithmetic order; quantization may not alter a raw value.
    encoded = [round(v*(1000*180/math.pi)) for v in target]
    require(encoded == q, 'Joint message does not round-trip to intended raw target')
    return dict(axis=axis, start_raw=list(before['raw_q']), target_raw=q, target=target,
                step_deg=amount/1000, speed_percent=1,
                message={'position': target, 'velocity': [0.]*6+[1.]},
                expected_can_frames=4, expected_jaw_frames=0,
                receipt_scope='one ROS publish plus mode intent and measured motion; no CAN send receipt')


def check_path(plan, before, fk=None):
    if fk is None:
        # Pure manufacturer kinematics, never a C_PiperInterface/SDK control object.
        from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
        engine = C_PiperForwardKinematics(1)
        def fk(q):
            v = engine.CalFK(q)[-1]
            return [x/1000 for x in v[:3]]+[math.radians(x) for x in v[3:]]
    start = fk(before['q'])
    require(math.dist(start[:3], before['pose'][:3]) <= .002
            and rotation(start, before['pose']) <= .02, 'Manufacturer FK disagrees with measured end reference')
    samples = []
    for n in range(21):
        q = [a+(b-a)*n/20 for a, b in zip(before['q'], plan['target'])]
        p = fk(q)
        require(-.6 <= p[0] <= .6 and -.6 <= p[1] <= .6 and .05 <= p[2] <= .65,
                'FK interpolation leaves original workspace')
        require(math.dist(p[:3], before['pose'][:3]) <= .03 and rotation(p, before['pose']) <= .05,
                'FK interpolation exceeds original 30mm/.05rad step cap')
        samples.append(dict(q=q, pose=p))
    return dict(samples=samples, scope='21-point joint interpolation FK envelope, not collision proof')


def moving_check(s, before, plan, session):
    check_session(s, session)
    require(math.dist(s['pose'][:3], before['pose'][:3]) <= .03
            and rotation(s['pose'], before['pose']) <= .05, 'Measured EEF step exceeds 30mm/.05rad')
    for i, (v, start, end) in enumerate(zip(s['q'], before['q'], plan['target'])):
        if i == plan['axis']-1:
            require(min(start, end)-.003 <= v <= max(start, end)+.003, 'Selected joint left bounded step interval')
        else:
            require(abs(v-start) <= .003, 'Unselected joint drift exceeds .003rad')


class StableWindow:
    """A contiguous 3s window with >=20 complete advancing fragment groups."""
    def __init__(self):
        self.samples = deque()

    def add(self, s, monotonic):
        if self.samples:
            previous = self.samples[-1][1]
            require(s['sequence'] > previous['sequence']
                    and all(a > b for a, b in zip(s['stamps'], previous['stamps'])),
                    'All fourteen feedback fragments must advance')
        self.samples.append((monotonic, copy.deepcopy(s)))
        while len(self.samples) > 1 and monotonic-self.samples[1][0] >= 3.:
            self.samples.popleft()
        rows = [row for _, row in self.samples]
        # Range checks are O(n); SO(3) pair checks only for this bounded 3s window.
        for key, width, cap in (('q', 6, .003),):
            if any(max(r[key][i] for r in rows)-min(r[key][i] for r in rows) > cap for i in range(width)):
                self.samples = deque([(monotonic, copy.deepcopy(s))]); return False
        xyz_span = [max(r['pose'][i] for r in rows)-min(r['pose'][i] for r in rows) for i in range(3)]
        if math.sqrt(sum(v*v for v in xyz_span)) > .0005:
            self.samples = deque([(monotonic, copy.deepcopy(s))]); return False
        if max(r['opening_m'] for r in rows)-min(r['opening_m'] for r in rows) > .0005:
            self.samples = deque([(monotonic, copy.deepcopy(s))]); return False
        if any(rotation(s['pose'], r['pose']) > .003 for r in rows):
            self.samples = deque([(monotonic, copy.deepcopy(s))]); return False
        return len(rows) >= 20 and monotonic-self.samples[0][0] >= 3.


def commit_step(session, plan, window, after):
    rows = [row for _, row in window.samples]
    i = plan['axis']-1
    require(max(abs(row['raw_q'][i]) for row in rows) < abs(plan['start_raw'][i]),
            'No verified reduction in selected joint distance to zero')
    caps = list(session['exception_caps_raw'])
    for axis in (1, 2):
        observed = max(max(0, -r['raw_q'][axis]) if axis == 1 else max(0, r['raw_q'][axis]) for r in rows)
        if caps[axis-1]:
            if axis == i:
                require(observed < caps[axis-1], 'Stable endpoint did not strictly reduce exception')
            caps[axis-1] = min(caps[axis-1], observed)
    session.update(exception_caps_raw=caps, last=copy.deepcopy(after), pending=None,
                   completed_steps=session['completed_steps']+1)
    session['history'].append(dict(axis=plan['axis'], target_raw=plan['target_raw'],
                                  final_raw=after['raw_q'], exception_caps_raw=caps, completed_at=time.time()))


class SessionFile:
    def __init__(self, path):
        self.path = Path(path); self.lock = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = self.path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.lock.close(); raise
        return self

    def load(self):
        return json.loads(self.path.read_text()) if self.path.exists() else None

    def save(self, state):
        temp = self.path.with_suffix('.tmp')
        with temp.open('w') as out:
            json.dump(state, out, allow_nan=False, indent=2); out.write('\n'); out.flush(); os.fsync(out.fileno())
        os.replace(temp, self.path)
        fd = os.open(str(self.path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)

    def __exit__(self, *args):
        self.lock.close()


def mode_events(path, offset):
    with Path(path).open('rb') as source:
        require(source.seek(0, 2) >= offset, 'Driver log truncated')
        source.seek(offset); raw = source.read()
    # Only consume complete lines actually inspected. Appends after read and a
    # partially written final line remain visible to the next audit.
    length = raw.rfind(b'\n')+1
    text = raw[:length].decode('utf-8', 'replace')
    result = []
    for line in text.splitlines():
        start = line.find('{')
        if start < 0: continue
        try: item = json.JSONDecoder().raw_decode(line[start:])[0]
        except ValueError: continue
        if item.get('source') == 'ros_low_speed_entry': result.append(item)
    return result, offset+length


def check_intent(events, require_seen=False):
    require(len(events) <= 1, 'Additional driver control intent observed')
    if events:
        e = events[0]
        require(e.get('event') == 'motion_mode_before_sdk_call' and e.get('move_mode') == 1
                and e.get('ctrl_mode') == 1 and e.get('requested_speed_percent') == 1
                and e.get('effective_speed_percent') == 1 and e.get('is_mit_mode') == 0,
                'Unexpected low-speed wrapper control intent')
    require(not require_seen or bool(events), 'No matching driver J-mode intent after ROS publication')


class LiveIO:
    """Lazy ROS topic publisher and a socket with recvmsg only; no CAN send API."""
    def __init__(self, driver_log):
        self.driver_log = Path(driver_log).resolve(strict=True)
        self.receiver = self.publisher = self.subscriber = None
        self.rospy = self.master = None
        self.latest_health = None
        self.frames = {}; self.last_stamps = [0.]*14; self.sequence = 0

    def identity(self):
        for path, expected in HASHES:
            require(hashlib.sha256(path.read_bytes()).hexdigest() == expected, 'Pinned source changed: '+str(path))
        interface = Path('/sys/class/net/can1'); usb = (interface/'device').resolve(strict=True)
        require(usb.name == '1-6.3:1.0' and (interface/'type').read_text().strip() == '280', 'Right CAN/USB binding changed')
        import importlib.metadata
        require(importlib.metadata.version('piper-sdk') == '0.6.2', 'SDK package version changed')
        if self.rospy is None:
            import rospy, rosgraph
            from std_msgs.msg import String
            if not rospy.core.is_initialized(): rospy.init_node('ros_home_step', anonymous=True, disable_signals=True)
            self.rospy, self.master = rospy, rosgraph.Master(rospy.get_name())
            def receive(msg): self.latest_health = json.loads(msg.data)
            self.subscriber = rospy.Subscriber(TELEMETRY, String, receive, queue_size=1)
        pubs, subs, services = self.master.getSystemState()
        require(dict(pubs).get(TELEMETRY) == [NODE] and dict(subs).get(TOPIC) == [NODE], 'ROS driver ownership/subscriber mismatch')
        for topic in COMMAND_TOPICS:
            expected = [self.rospy.get_name()] if topic == TOPIC and self.publisher is not None else []
            require(dict(pubs).get(topic, []) == expected, 'Unexpected command publisher: '+topic)
        for suffix, expected in (('can_port', 'can1'), ('auto_enable', False), ('exit_teaching_mode', False),
                                 ('gripper_exist', False), ('gripper_val_mutiple', 1)):
            v = self.master.getParam(NODE+'/'+suffix)
            require(v == expected and (not isinstance(expected, bool) or v is expected), 'Driver private parameter mismatch: '+suffix)
        import xmlrpc.client
        uri = self.master.lookupNode(NODE)
        code, message, pid = xmlrpc.client.ServerProxy(uri).getPid(self.rospy.get_name())
        require(code == 1 and type(pid) is int and pid > 0, 'Cannot verify ROS driver PID')
        proc = Path('/proc')/str(pid)
        argv = [x.decode() for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        require(str(ENTRY) in argv and str(VENDOR) in argv and '__name:=driver' in argv, 'ROS owner is not pinned low-speed entry')
        owners = []
        for p in Path('/proc').iterdir():
            if not p.name.isdigit(): continue
            try: arguments = (p/'cmdline').read_bytes().split(b'\0')
            except OSError: continue
            if str(ENTRY).encode() in arguments: owners.append(int(p.name))
        require(owners == [pid], 'Low-speed entry process ownership is not unique')
        live = {n for _, nodes in pubs+subs+services for n in nodes}
        for key in self.master.getParamNames():
            if key.endswith('/can_port') and key.rsplit('/', 1)[0] in live and key != NODE+'/can_port':
                require(self.master.getParam(key) != 'can1', 'Another live ROS owner uses can1')
        stat = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        return dict(driver_pid=pid, process_start_ticks=stat[19], ros_uri=uri,
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    ifindex=(interface/'ifindex').read_text().strip(), usb_path=str(usb),
                    entry_sha256=HASHES[0][1], health_sha256=HASHES[1][1], vendor_sha256=HASHES[2][1],
                    speed_cap_percent=1, driver_log=str(self.driver_log),
                    driver_log_device=self.driver_log.stat().st_dev, driver_log_inode=self.driver_log.stat().st_ino)

    def verify_health_message(self, raw):
        h = self.latest_health
        require(h is not None and 0 <= time.time()-h.get('stamp', 0) <= .1
                and h.get('source') == 'sdk_receive_frames' and h.get('can_interface') == 'can1'
                and h.get('driver_sha256') == HASHES[2][1] and h.get('sdk_version') == '0.6.2'
                and h.get('driver_accepts_commands') is True and h.get('enabled') == [True]*6
                and h.get('arm_status') == 0 and h.get('fault') == 0
                and h.get('feedback_max_age_s', 99) <= .1, 'Fresh enabled ROS internal/health state required')
        require(len(h.get('q', [])) == 6 and max(abs(a-b) for a, b in zip(h['q'], raw['q'])) <= .003,
                'ROS health and raw CAN joints disagree')

    def receive(self, timeout_s=1.):
        if self.receiver is None:
            self.receiver = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # Linux SO_TIMESTAMPNS: kernel receive time, never refreshed at parsing.
            self.receiver.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
            filters = b''.join(struct.pack('=II', i, socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG) for i in IDS)
            self.receiver.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.receiver.bind(('can1',))
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            self.receiver.settimeout(max(.001, deadline-time.monotonic()))
            try: packet, ancillary, flags, _ = self.receiver.recvmsg(16, socket.CMSG_SPACE(struct.calcsize('@ll')))
            except socket.timeout: raise TimeoutError('No new complete raw CAN feedback')
            require(not flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC), 'Truncated CAN frame/timestamp')
            if flags & socket.MSG_DONTROUTE: continue  # Reject local loopback as feedback.
            require(len(packet) == 16, 'Malformed CAN frame')
            identifier, length, payload = struct.unpack('=IB3x8s', packet)
            if identifier not in IDS or length != 8: continue
            stamps = [struct.unpack('@ll', data[:struct.calcsize('@ll')]) for level, kind, data in ancillary
                      if level == socket.SOL_SOCKET and kind == getattr(socket, 'SO_TIMESTAMPNS', 35)]
            require(len(stamps) == 1, 'Missing unique kernel receive timestamp')
            stamp = stamps[0][0]+stamps[0][1]/1e9
            self.frames[identifier] = (stamp, payload); self.sequence += 1
            if len(self.frames) == 14:
                s = decode(self.frames, self.sequence)
                if all(a > b for a, b in zip(s['stamps'], self.last_stamps)):
                    self.last_stamps = s['stamps']; return s
        raise TimeoutError('No new complete raw CAN feedback')

    def connect_publisher(self):
        from sensor_msgs.msg import JointState
        self.publisher = self.rospy.Publisher(TOPIC, JointState, queue_size=1, latch=False)
        deadline = time.monotonic()+3
        while self.publisher.get_num_connections() != 1:
            require(time.monotonic() < deadline, 'Exactly one connected ROS joint subscriber required')
            time.sleep(.01)

    def publish_once(self, plan):
        from sensor_msgs.msg import JointState
        self.publisher.publish(JointState(**plan['message']))

    def close(self):
        if self.publisher is not None: self.publisher.unregister()
        if self.subscriber is not None: self.subscriber.unregister()
        if self.receiver is not None: self.receiver.close()


def run_step(io, store, identity, axis, max_step_deg, trace, result, *, clock=time, path_check=check_path):
    """Injected I/O supports offline tests; live CLI selects only LiveIO."""
    session = store.load()
    if session is not None:
        require(session['identity'] == identity, 'Persistent session identity changed')
        require(not session['failure'] and session['pending'] is None, 'Earlier session failure/pending publication is latched')
    try:
        if session is not None:
            events, _ = mode_events(io.driver_log, session['log_offset'])
            require(not events, 'Driver control intent occurred between maintenance steps')
        baseline, window = None, StableWindow()
        deadline = clock.monotonic()+10
        while clock.monotonic() < deadline:
            s = io.receive(); health(s, clock.time(), idle=True)
            if session is None:
                session = new_session(identity, s)
                session['log_offset'] = io.driver_log.stat().st_size
                store.save(session)
            check_session(s, session); no_drift(session['last'], s)
            trace(s)
            if window.add(s, clock.monotonic()): baseline = s; break
        require(baseline is not None, 'No stable fresh three-second pre-step window')
        io.verify_health_message(baseline)
        io.connect_publisher()
        require(io.identity() == identity, 'Ownership/binding changed before preparation')
        before = io.receive(); health(before, clock.time(), idle=True); no_drift(baseline, before)
        plan = make_plan(axis, max_step_deg, before, session)
        checked = path_check(plan, before)
        # The import/FK/identity work may be slow. Re-read and recompute from NEW q.
        fresh = io.receive(); health(fresh, clock.time(), idle=True); no_drift(before, fresh)
        before, plan = fresh, make_plan(axis, max_step_deg, fresh, session)
        checked = path_check(plan, before)
        io.verify_health_message(before)
        require(io.identity() == identity, 'Ownership/binding changed immediately before publication')
        events, log_offset = mode_events(io.driver_log, session['log_offset'])
        require(not events, 'Unexpected driver control during preparation')
        result.update(before=before, plan=plan, path_check=checked, driver_log_offset=log_offset)
        # Durable attempt BEFORE publish: interruption anywhere below cannot replay.
        session['attempts'] += 1
        session['pending'] = dict(axis=axis, plan=plan, at=clock.time())
        store.save(session)
        health(before, clock.time(), idle=True)
        result.update(publish_attempts=1, command_at=clock.time())
        io.publish_once(plan)  # Sole actuator request; four CAN frames are expected, not independently counted.
        # Monitoring uses the same fixed exception caps, while pending remains durable.
        monitor_session = copy.deepcopy(session); monitor_session['pending'] = None
        monitor_session['attempts'] -= 1
        window, deadline, previous = StableWindow(), clock.monotonic()+120, before
        while clock.monotonic() < deadline:
            s = io.receive(); trace(s); result['after'] = s
            health(s, clock.time())
            require(s['sequence'] > previous['sequence'] and all(a > b for a, b in zip(s['stamps'], previous['stamps'])),
                    'Raw feedback fragments failed to advance')
            previous = s
            moving_check(s, before, plan, monitor_session)
            events, _ = mode_events(io.driver_log, log_offset)
            check_intent(events)
            arrived = (s['mode'] == 1 and s['motion_status'] == 0 and min(s['stamps']) > result['command_at']
                       and max(abs(a-b) for a, b in zip(s['q'], plan['target'])) <= .003)
            if not arrived:
                window = StableWindow(); continue
            if window.add(s, clock.monotonic()):
                io.verify_health_message(s)
                require(io.identity() == identity, 'Ownership/binding changed after publication')
                events, checked_log_offset = mode_events(io.driver_log, log_offset)
                check_intent(events, require_seen=True)
                health(s, clock.time(), idle=True)
                commit_step(session, plan, window, s)
                session['log_offset'] = checked_log_offset
                store.save(session)
                result.update(status='single_home_step_arrived', arrival_confirmed=True,
                              exception_caps_raw=session['exception_caps_raw'], attempts=session['attempts'],
                              can_delivery_receipt=False, hold_verified=False)
                return
        raise TimeoutError('120s finite-target arrival timeout; no physical cancellation or retry')
    except BaseException as exc:
        if session is not None:
            session['failure'] = dict(type=type(exc).__name__, reason=str(exc), at=clock.time(),
                                      target_uncertain=session['pending'] is not None,
                                      physical_recovery_sent=False, hold_verified=False)
            store.save(session)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', required=True, type=int)
    parser.add_argument('--max-step-deg', type=float, default=1.)
    parser.add_argument('--driver-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    require(1 <= args.axis <= 6 and math.isfinite(args.max_step_deg) and 0 < args.max_step_deg <= 1,
            'One axis and a positive step <=1 degree are required')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    io = LiveIO(args.driver_log)
    result = dict(status='starting', publish_attempts=0, arrival_confirmed=False,
                  scope='one supervised ROS near-zero maintenance step, not autonomous task permission',
                  can_frames_sent_by_this_client=0, started_at=time.time())
    try:
        identity = io.identity()
        # Caller cannot reset a failed session by choosing a different output path/axis.
        key = '%s_%s_%s' % (identity['boot_id'], identity['driver_pid'], identity['process_start_ticks'])
        path = ROOT/'runs'/'ros_home_maintenance_sessions'/(key+'.json')
        result.update(identity=identity, session_path=str(path))
        with SessionFile(path) as store, (args.output_dir/'trajectory.jsonl').open('x') as output:
            def trace(s):
                output.write(json.dumps(s, allow_nan=False)+'\n'); output.flush()
            run_step(io, store, identity, args.axis, args.max_step_deg, trace, result)
        code = 0
    except BaseException as exc:
        result.update(status='failed', error_type=type(exc).__name__, error=str(exc)); code = 2
    finally:
        io.close()  # Disconnect this client only. Driver and finite target remain untouched.
    result.update(finished_at=time.time(), target_uncertain=result['publish_attempts'] > 0 and not result['arrival_confirmed'],
                  physical_stop_sent=False, disable_sent=False, retry_sent=False)
    (args.output_dir/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'status': result['status'], 'output': str(args.output_dir), 'publish_attempts': result['publish_attempts']}))
    return code


if __name__ == '__main__':
    sys.exit(main())
