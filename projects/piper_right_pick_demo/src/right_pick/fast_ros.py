"""Pinned right ROS adapter: passive observation and gated single transactions.

Import/construction opens no ROS connection. No SDK import or driver lifecycle
operations. FastSafetyGuard requires independently recorded commissioning,
the same driver session, explicit limits and fresh feedback before dispatch.
Pose coordinates are the driver end reference in right_base, not finger TCP.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import threading
import time

from .fast_safety import FastSafetyGuard, FastSafetyError, physical_blockers, rotation_distance


class FastROSError(RuntimeError):
    pass


ADAPTER_PATH = '/home/agilex/piperx_cloth_demo/robot_tools/ros_resume_entry.py'
ADAPTER_SHA256 = '3cd2fcc87b5444a4e21419f33a4f38169b7ef2b7645fe329abba1aed70a992c8'
VENDOR_PATH = '/home/agilex/piper_gpt/src/piper_ros-noetic/src/piper/scripts/piper_ctrl_single_node.py'
VENDOR_SHA256 = 'ecff6823dc6bbf55fe51708024ef88d993fead97572ad7c06e951020e83bc200'
FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
JOINT_LIMITS_RAW = ((-150000, 150000), (0, 180000), (-170000, 0),
                    (-100000, 100000), (-70000, 70000), (-120000, 120000))
RAD_PER_RAW = math.pi / 180000
PINNED = dict(telemetry_topic='/piper/right/eval_telemetry', pose_topic='/piper/right/pos_cmd',
              gripper_service='/piper/right/gripper_srv', speed_param='/piper/right/driver/speed_percent',
              driver_node='/piper/right/driver', can_interface='can1', usb_interface='1-6.3:1.0',
              adapter_path=ADAPTER_PATH, source_sha256=ADAPTER_SHA256, adapter_sha256=ADAPTER_SHA256,
              vendor_path=VENDOR_PATH, vendor_sha256=VENDOR_SHA256)


def _finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise FastROSError(name + ' must be finite numeric')
    return float(value)


def _vector(value, count, name):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise FastROSError(name + ' has incorrect length')
    return [_finite(v, name) for v in value]


def _int(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise FastROSError(name + ' must be a bounded integer')
    return value


def _decision(raw):
    from .fast_policy import Decision, parse_response, phase_spec
    explain = (isinstance(raw, Decision) and raw.explanation is not None
               or isinstance(raw, dict) and 'explanation' in raw)
    result = parse_response(raw, require_explanation=explain)
    if result.action not in phase_spec(result.phase)['allowed_actions']:
        raise FastROSError('Action is forbidden in this phase')
    return result


def _events(path):
    """Only this adapter's JSON lines, bounded to the existing local session log."""
    path = Path(path)
    if path.stat().st_size > 16 * 1024 * 1024:
        raise FastROSError('Driver log too large; explicit review needed')
    result = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.startswith('{'):
            continue
        try:
            value = json.loads(line)
        except ValueError:
            continue  # Writer may have an incomplete final line.
        if isinstance(value, dict) and value.get('source') == 'ros_resume_entry':
            result.append(value)
    if not any(e.get('event') == 'read_only_adoption_complete' for e in result):
        raise FastROSError('Current session adoption receipt is missing')
    return result


def _last_command_sequence(events):
    sequences = [e['sequence'] for e in events if e.get('event') == 'command_intent']
    for seq in sequences:
        _int(seq, 1, 2**63-1, 'command sequence')
    if sequences != list(range(1, len(sequences)+1)):
        raise FastROSError('Driver command history is incomplete or out of order')
    return sequences[-1] if sequences else 0


def _last_jaw_target(events):
    # Keep the commanded contact target, not measured opening under load.
    target = None
    for event in events:
        if event.get('event') != 'command_intent':
            continue
        for frame in event.get('frames', []):
            if frame.get('id') == 0x159:
                try:
                    data = bytes.fromhex(frame['data_hex'])
                    width, effort, enabled, zero = struct.unpack('>iHBB', data)
                except (KeyError, ValueError, struct.error) as exc:
                    raise FastROSError('Invalid jaw command receipt') from exc
                if not 0 <= width <= 55000 or (effort, enabled, zero) != (200, 1, 0):
                    raise FastROSError('Unexpected jaw command in pinned session log')
                target = width / 1e6
    return target


def check_telemetry(raw, now, *, require_idle=True):
    """Recheck raw fragment age and health; driver_accepts alone is insufficient."""
    if not isinstance(raw, dict):
        raise FastROSError('Telemetry must be an object')
    if (raw.get('source') != 'sdk_receive_raw_frames' or raw.get('can_interface') != 'can1'
            or raw.get('driver_sha256') != VENDOR_SHA256 or raw.get('sdk_version') != '0.6.2'):
        raise FastROSError('Telemetry source/version/binding mismatch')
    stamps = _vector(raw.get('stamps'), 14, 'kernel fragment stamps')
    pubstamp = _finite(raw.get('stamp'), 'telemetry publish stamp')
    if min(stamps) <= 0 or max(stamps) > now or now-min(stamps) > .1 or max(stamps)-min(stamps) > .1:
        raise FastROSError('Kernel feedback age/skew exceeds 100ms or is future dated')
    if not max(stamps) <= pubstamp <= now or now-pubstamp > .1:
        raise FastROSError('Telemetry publish timestamp is stale/inconsistent')
    seq = _int(raw.get('sequence'), 1, 2**63-1, 'raw receive sequence')
    if type(raw.get('source_sequence')) is not int or raw['source_sequence'] != seq:
        raise FastROSError('Raw receive sequence mismatch')
    for field, allowed in [('ctrl_mode', (1,)), ('arm_status', (0,)), ('fault', (0,)),
                           ('teach_status', (0,)), ('mode', (0, 1, 2)),
                           ('motion_status', (0,) if require_idle else (0, 1)), ('jaw_code', (64,))]:
        if type(raw.get(field)) is not int or raw[field] not in allowed:
            raise FastROSError('Unhealthy telemetry: ' + field)
    if (raw.get('driver_codes') != [64]*6 or any(type(v) is not int for v in raw['driver_codes'])
            or raw.get('enabled') != [True]*6 or any(v is not True for v in raw['enabled'])):
        raise FastROSError('Six enabled healthy driver flags required')
    if type(raw.get('active_command')) is not bool or type(raw.get('driver_accepts_commands')) is not bool:
        raise FastROSError('Missing driver action-state flags')
    if raw.get('failure') is not None:
        raise FastROSError('Driver failure latch: ' + str(raw['failure']))
    if require_idle and (raw['active_command'] or not raw['driver_accepts_commands']):
        raise FastROSError('Driver active or not accepting commands')
    q = _vector(raw.get('q'), 6, 'joint feedback')
    raw_q = raw.get('raw_q')
    if not isinstance(raw_q, list) or len(raw_q) != 6:
        raise FastROSError('Missing raw joint feedback')
    for measured, encoded, (lo, hi) in zip(q, raw_q, JOINT_LIMITS_RAW):
        _int(encoded, lo, hi, 'manufacturer nominal raw joint')
        if abs(measured-encoded*RAD_PER_RAW) > 1e-10:
            raise FastROSError('Raw/converted joint feedback mismatch')
    pose = _vector(raw.get('pose'), 6, 'driver end feedback')
    if any(abs(v) > 1 for v in pose[:3]) or any(abs(v) > 2*math.pi for v in pose[3:]):
        raise FastROSError('Pose outside vendor feedback encoding envelope')
    opening = _finite(raw.get('opening_m'), 'gripper opening')
    if not 0 <= opening <= .07:
        raise FastROSError('Gripper feedback outside manufacturer envelope')
    _finite(raw.get('gripper_torque_sdk_units'), 'gripper effort feedback')
    return {'sampled_at': min(stamps), 'max_feedback_age_s': now-min(stamps),
            'feedback_skew_s': max(stamps)-min(stamps)}


def _check_command_publishers(pubs, config, own_node, owns_pose_publisher):
    command_publishers = {topic: nodes for topic, nodes in pubs
                          if topic in (config['pose_topic'], '/piper/right/joint_cmd', '/piper/right/enable_flag')}
    allowed = ({config['pose_topic']: [own_node]} if owns_pose_publisher else {})
    if command_publishers and command_publishers != allowed:
        raise FastROSError('Another command publisher exists: ' + str(command_publishers))
    return command_publishers


class _ROSTransport:
    """All ROS imports lazy. Public operations are read-only; send is private."""
    def __init__(self, config):
        self.config = config
        self.rospy = self.master = self.subscriber = None
        self.condition = threading.Condition()
        self.latest = None
        self.generation = self.delivered = 0
        self.publishers = []

    def _connect(self):
        if self.rospy is not None:
            return
        import rospy
        import rosgraph
        from std_msgs.msg import String
        if not rospy.core.is_initialized():
            rospy.init_node('right_pick_passive_observer', anonymous=True, disable_signals=True)
        self.rospy, self.master = rospy, rosgraph.Master(rospy.get_name())
        def received(msg):
            with self.condition:
                self.latest = msg.data
                self.generation += 1
                self.condition.notify_all()
        self.subscriber = rospy.Subscriber(self.config['telemetry_topic'], String, received, queue_size=1)

    def identity(self):
        # Verify local identities before any subscription; no CAN socket is opened.
        c = self.config
        for path, expected in ((c['adapter_path'], ADAPTER_SHA256), (c['vendor_path'], VENDOR_SHA256)):
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
                raise FastROSError('Frozen driver source hash mismatch: ' + path)
        interface = Path('/sys/class/net') / c['can_interface']
        device = (interface / 'device').resolve(strict=True)
        if device.name != c['usb_interface'] or (interface / 'type').read_text().strip() != '280':
            raise FastROSError('Current Linux CAN/USB binding mismatch')
        self._connect()
        pubs, subs, services = self.master.getSystemState()
        if dict(pubs).get(c['telemetry_topic']) != [c['driver_node']]:
            raise FastROSError('Telemetry topic must have the sole pinned driver publisher')
        if dict(services).get(c['gripper_service']) != [c['driver_node']]:
            raise FastROSError('Gripper service owner mismatch')
        if c['driver_node'] not in dict(subs).get(c['pose_topic'], []):
            raise FastROSError('Pinned driver does not subscribe to PosCmd')
        command_publishers = _check_command_publishers(pubs, c, self.rospy.get_name(), bool(self.publishers))
        for suffix, expected in (('can_port', 'can1'), ('auto_enable', False),
                                 ('exit_teaching_mode', False), ('gripper_exist', True), ('gripper_val_mutiple', 1)):
            value = self.master.getParam(c['driver_node'] + '/' + suffix)
            if value != expected or (isinstance(expected, bool) and value is not expected):
                raise FastROSError('Pinned driver parameter mismatch: ' + suffix)
        speed = _int(self.master.getParam(c['speed_param']), 1, 50, 'driver speed_percent')
        import xmlrpc.client
        uri = self.master.lookupNode(c['driver_node'])
        code, message, pid = xmlrpc.client.ServerProxy(uri).getPid(self.rospy.get_name())
        if code != 1:
            raise FastROSError('Cannot verify ROS driver PID: ' + message)
        cmdline = Path('/proc') / str(_int(pid, 1, 2**31-1, 'driver PID')) / 'cmdline'
        argv = [v.decode() for v in cmdline.read_bytes().split(b'\0') if v]
        if c['adapter_path'] not in argv or c['vendor_path'] not in argv or '__name:=driver' not in argv:
            raise FastROSError('ROS owner PID is not the frozen resume process')
        # Check unique running instance, not just the ROS graph name.
        owners = []
        for item in Path('/proc').iterdir():
            if not item.name.isdigit():
                continue
            try:
                arguments = (item / 'cmdline').read_bytes().split(b'\0')
            except (OSError, PermissionError):
                continue
            if c['adapter_path'].encode() in arguments:
                owners.append(int(item.name))
        if owners != [pid]:
            raise FastROSError('Resume process ownership is not unique')
        return dict(binding_verified=True, source_verified=True, driver_pid=pid,
                    can_interface='can1', usb_interface=device.name, adapter_sha256=ADAPTER_SHA256,
                    vendor_sha256=VENDOR_SHA256, driver_node=c['driver_node'], speed_percent=speed,
                    command_publishers=command_publishers,
                    source_scope='current source hashes plus live ROS owner PID; not memory attestation')

    def receive(self, timeout_s):
        self._connect()
        end = time.monotonic() + timeout_s
        with self.condition:
            while self.generation <= self.delivered:
                remaining = end-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('No new ROS telemetry; no retry or physical recovery')
                self.condition.wait(remaining)
            self.delivered = self.generation
            value = self.latest
        return json.loads(value)

    def events(self):
        return _events(self.config['command_log'])

    def _dispatch_once(self, envelope, before_send):
        # Only called by ROSRightArm after formal gate + numeric bounds.
        # These private routes are not exposed by CLI/tool registration.
        if envelope['route'] == 'topic':
            from piper_msgs.msg import PosCmd
            publisher = self.rospy.Publisher(self.config['pose_topic'], PosCmd, queue_size=1, latch=False)
            self.publishers.append(publisher)
            deadline = time.monotonic()+3
            while publisher.get_num_connections() != 1:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Expected one PosCmd subscriber; not sent')
                time.sleep(.01)
            before_send()  # Connection setup is not allowed to age the authorization snapshot.
            publisher.publish(PosCmd(**envelope['message']))  # Exactly once, never retry.
        else:
            from piper_msgs.srv import Gripper
            self.rospy.wait_for_service(self.config['gripper_service'], timeout=3)
            service = self.rospy.ServiceProxy(self.config['gripper_service'], Gripper, persistent=False)
            # This service waits for the driver action; bound the caller wait without
            # pretending timeout cancels an already accepted firmware goal.
            done, results = threading.Event(), []
            def request():
                try:
                    before_send()  # Service discovery may have taken seconds.
                    results.append(service(**envelope['request']))
                except BaseException as error:
                    results.append(error)
                finally:
                    done.set()
            worker = threading.Thread(target=request, daemon=True)
            worker.start()
            if not done.wait(125):
                raise TimeoutError('Gripper service outcome uncertain; request is not cancelled')
            if isinstance(results[0], BaseException):
                raise results[0]
            if results[0].status is not True:
                raise FastROSError('Gripper service refused; no retry')

    def close(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
        for publisher in self.publishers:
            publisher.unregister()
        self.publishers.clear()
        # Never shutdown/restart the shared driver or send a physical stop.


class ROSRightArm:
    nonphysical = False

    def __init__(self, config, recorder=None, proposal_only=True, transport=None,
                 *, clock=time.time, monotonic=time.monotonic):
        if not isinstance(config, dict) or type(proposal_only) is not bool:
            raise FastROSError('Explicit configuration and boolean proposal_only required')
        self.config = copy.deepcopy(config)
        ros = config.get('ros', {})
        if not isinstance(ros, dict):
            raise FastROSError('ros configuration must be an object')
        self.ros = dict(PINNED)
        for key, expected in PINNED.items():
            if key in ros and ros[key] != expected:
                raise FastROSError('Only the pinned right ROS route is supported: ' + key)
        self.ros.update(ros)
        if not isinstance(self.ros.get('command_log'), str) or not Path(self.ros['command_log']).is_absolute():
            raise FastROSError('Absolute current session command_log is required')
        self.recorder, self.proposal_only = recorder, proposal_only
        self._transport = transport
        self.clock, self.monotonic = clock, monotonic
        self.failure = None
        self.target_uncertain = False
        self.closed = False
        self._action_lock = threading.Lock()
        self._last_raw = None

    def _open(self):
        if self.closed or self.failure:
            raise FastROSError('Adapter closed or failure latched; no retry: ' + str(self.failure))

    def _get_transport(self):
        if self._transport is None:
            self._transport = _ROSTransport(self.ros)
        return self._transport

    def authorization_status(self):
        from .fast_safety import qualification_status
        status = qualification_status(self.config)
        status['proposal_only'] = self.proposal_only
        return status

    def preflight(self):
        """Read current identity/state and ask the independent physical gate.

        Readiness is never inferred from a successful home or a config flag.
        The commissioning gate remains responsible for real evidence.
        """
        # Parse the immutable, potentially large raw evidence before acquiring
        # the short-lived current feedback. Later guards reuse the checked cache.
        status = self.authorization_status()
        if self.proposal_only:
            status['blockers'] = list(status['blockers']) + ['proposal_only_adapter']
            status['physical_execution_ready'] = False
            return status
        state = self.observe()
        status = FastSafetyGuard(self.config).preflight(state)
        status['state'] = state
        return status

    def _observe(self, timeout_s, require_idle):
        self._open()
        _finite(timeout_s, 'observe timeout')
        if not 0 < timeout_s <= 125:
            raise FastROSError('Observation timeout must be within 0..125 seconds')
        transport = self._get_transport()
        identity = transport.identity()
        if identity.get('binding_verified') is not True or identity.get('source_verified') is not True:
            raise FastROSError('Live identity verification failed')
        events = transport.events()
        command_sequence = _last_command_sequence(events)
        raw = transport.receive(timeout_s)
        checks = check_telemetry(raw, self.clock(), require_idle=require_idle)
        if self._last_raw is not None:
            if raw['sequence'] <= self._last_raw['sequence'] or any(a < b for a, b in zip(raw['stamps'], self._last_raw['stamps'])):
                raise FastROSError('Feedback sequence regressed/repeated or fragment timestamp regressed')
        self._last_raw = copy.deepcopy(raw)
        return dict(nonphysical=False, robot_state=dict(pose_m_rad=list(raw['pose']), joints_rad=list(raw['q']),
                    sampled_at=checks['sampled_at'], enabled=True, moving=raw['motion_status'] != 0,
                    binding_verified=True, arm_status=raw['arm_status'], err_code=raw['fault']),
                    gripper_state=dict(opening_m=raw['opening_m']),
                    raw_telemetry=copy.deepcopy(raw), provenance=dict(identity, **checks,
                    command_sequence=command_sequence, jaw_command_target_m=_last_jaw_target(events),
                    command_log=self.ros['command_log'],
                    current_driver_adoption_unix_s=next(e.get('unix_s') for e in events if e.get('event') == 'read_only_adoption_complete'),
                    command_sequence_source='local_driver_log_not_CAN_receive_counter',
                    feedback_ids=list(FEEDBACK_IDS), effort_units='SDK raw telemetry units, not measured newton metres',
                    physical_motion_authorized=False, hold_verified=False))

    def observe(self, timeout_s=3.0):
        return self._observe(timeout_s, require_idle=True)

    def prepare_command(self, decision, state):
        """Pure encoding/routing only; no ROS, filesystem, parameter or send I/O."""
        self._open()
        decision = _decision(decision)
        args = decision.arguments
        raw = state.get('raw_telemetry', {})
        # A model proposal may refer to an older valid observation. Encoding it
        # grants no freshness/dispatch authority; validate rechecks wall-clock age.
        check_telemetry(raw, _finite(raw.get('stamp'), 'observation publication time'))
        provenance = state.get('provenance', {})
        sequence = _int(provenance.get('command_sequence'), 0, 2**63-2, 'last command sequence')
        envelope = dict(action=decision.action, expected_command_sequence=sequence+1,
                        physical_motion_authorized=False, numeric_limits_verified=False,
                        requires_fresh_revalidation=True,
                        source_state_stale_at_preparation=self.clock()-min(raw['stamps']) > .1,
                        pose_reference='right_base_driver_end_reference', pose_units='metres_radians',
                        hold_verified=False, safety_scope='encoding only, not workspace/IK/path clearance')
        def frame(can_id, data):
            return dict(id=can_id, data_hex=data.hex())
        if decision.action == 'move_eef':
            pose = args['pose_m_rad']
            if any(abs(v) > 1 for v in pose[:3]) or any(abs(v) > 2*math.pi for v in pose[3:]):
                raise FastROSError('Pose outside pinned vendor encoding envelope')
            speed = _int(args['speed_percent'], 1, 50, 'speed_percent')
            width = provenance.get('jaw_command_target_m')
            if width is None:
                raise FastROSError('Known commanded jaw target required to preserve contact during P motion')
            width = _finite(width, 'last commanded jaw target')
            if not 0 <= width <= .055:
                raise FastROSError('Commanded opening cannot be preserved by the pinned 0..55mm route')
            factor = 180 / 3.1415926
            encoded = [round(v*1000)*1000 for v in pose[:3]] + [round(v*1000*factor) for v in pose[3:]]
            mode = frame(0x151, bytes((1, 0, speed, 0, 0, 0, 0, 0)))
            frames = [frame(0x150, bytes(8)), mode]
            frames += [frame(0x152+i, struct.pack('>ii', *encoded[2*i:2*i+2])) for i in range(3)]
            frames += [frame(0x159, struct.pack('>iHBB', round(width*1000*1000), 200, 1, 0)), mode]
            envelope.update(route='topic', topic=self.ros['pose_topic'], message_type='piper_msgs/PosCmd', kind='pose',
                            message=dict(zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pose),
                                         gripper=width, mode1=0, mode2=0),
                            speed_percent=speed, speed_param=self.ros['speed_param'],
                            speed_rule='must already equal requested speed; adapter never writes parameters',
                            encoded_target_m_rad=[v/1e6 for v in encoded[:3]]+[v*RAD_PER_RAW for v in encoded[3:]],
                            jaw_target_m=width, jaw_preservation='last_explicit_command_not_measured_contact_width',
                            expected_frames=frames)
        elif decision.action == 'gripper':
            if not 0 <= args['opening_m'] <= .055 or args['effort_parameter_nm'] != .2:
                raise FastROSError('Pinned gripper route requires 0..55mm and effort parameter exactly .2')
            envelope.update(route='service', service=self.ros['gripper_service'], service_type='piper_msgs/Gripper',
                            kind='gripper', speed_percent=provenance['speed_percent'],
                            request=dict(gripper_angle=args['opening_m'], gripper_effort=.2, gripper_code=1, set_zero=0),
                            jaw_target_m=round(args['opening_m']*1e6)/1e6,
                            expected_frames=[frame(0x159, struct.pack('>iHBB', round(args['opening_m']*1e6), 200, 1, 0))])
        else:
            raise FastROSError('Only one move_eef or gripper action has a dispatch route; no chunks or automatic phase advance')
        return envelope

    def _numeric_limits(self, decision, state, envelope):
        """Explicit site limits are additionally necessary, never sufficient authority."""
        from .fast_policy import phase_translation_fraction, phase_rotation_fraction
        limits = self.config.get('physical_limits')
        if not isinstance(limits, dict):
            raise FastROSError('Qualified physical_limits are missing; numeric bounds unknown')
        pose = state['raw_telemetry']['pose']
        low = _vector(limits.get('workspace_min_m'), 3, 'workspace minimum')
        high = _vector(limits.get('workspace_max_m'), 3, 'workspace maximum')
        if any(a >= b for a, b in zip(low, high)):
            raise FastROSError('Invalid explicit workspace')
        points = [pose] + ([decision.arguments['pose_m_rad'], envelope['encoded_target_m_rad']]
                           if decision.action == 'move_eef' else [])
        if any(not a <= v <= b for p in points for a, v, b in zip(low, p[:3], high)):
            raise FastROSError('Current/target driver end outside explicit workspace')
        joints = limits.get('joint_limits_rad')
        if not isinstance(joints, list) or len(joints) != 6:
            raise FastROSError('Explicit six joint feedback bounds required; target IK not validated')
        for q, pair in zip(state['raw_telemetry']['q'], joints):
            a, b = _vector(pair, 2, 'joint limits')
            if not a < b or not a <= q <= b:
                raise FastROSError('Measured joint outside explicit bounds')
        age = _finite(limits.get('max_state_age_s'), 'maximum state age')
        if not 0 < age <= .1 or self.clock()-min(state['raw_telemetry']['stamps']) > age:
            raise FastROSError('Explicit state freshness limit rejected')
        jaw_low = _finite(limits.get('gripper_min_m'), 'jaw minimum')
        jaw_high = _finite(limits.get('gripper_max_m'), 'jaw maximum')
        effort = _finite(limits.get('max_effort_parameter_nm'), 'maximum effort parameter')
        if not 0 <= jaw_low < jaw_high <= .055 or not .2 <= effort:
            raise FastROSError('Invalid explicit gripper limits')
        if not jaw_low <= state['raw_telemetry']['opening_m'] <= jaw_high:
            raise FastROSError('Current jaw outside explicit bounds')
        if decision.action == 'move_eef':
            distance = _finite(limits.get('max_translation_step_m'), 'translation limit')
            angle = _finite(limits.get('max_rotation_step_rad'), 'rotation limit')
            if distance <= 0 or angle <= 0:
                raise FastROSError('Positive explicit movement limits required')
            cap = _int(limits.get('max_speed_percent'), 1, 50, 'speed cap')
            _int(decision.arguments['speed_percent'], 1, cap, 'command speed')
            if any(math.dist(pose[:3], target[:3]) > distance*phase_translation_fraction(decision.phase)
                   or rotation_distance(pose, target) > angle*phase_rotation_fraction(decision.phase)
                   for target in points[1:]):
                raise FastROSError('Phase scaled step exceeds explicit site bounds')
        elif not jaw_low <= decision.arguments['opening_m'] <= jaw_high:
            raise FastROSError('Jaw goal outside explicit bounds')

    def validate(self, decision, state):
        self._open()
        decision = _decision(decision)
        envelope = self.prepare_command(decision, state)
        check_telemetry(state['raw_telemetry'], self.clock())
        # No injected gate and no config boolean can authorize this release.
        FastSafetyGuard(self.config).validate(decision, state)
        self._numeric_limits(decision, state, envelope)
        return envelope

    def _match_receipt(self, envelope, events, before, dispatch_started):
        seq = envelope['expected_command_sequence']
        if _last_command_sequence(events) > seq:
            raise FastROSError('Unexpected additional driver command; ownership uncertain')
        relevant = [e for e in events if e.get('sequence') == seq]
        intent = next((e for e in relevant if e.get('event') == 'command_intent'), None)
        if intent is None:
            return None
        if (intent.get('kind') != envelope['kind'] or intent.get('frames') != envelope['expected_frames']
                or intent.get('speed_percent') != envelope['speed_percent']
                or intent.get('unix_s', 0) < dispatch_started):
            raise FastROSError('Expected command receipt mismatch')
        baseline = before['raw_telemetry']
        captured = intent.get('before', {})
        if (len(captured.get('stamps', [])) != 14
                or any(a < b for a, b in zip(captured['stamps'], baseline['stamps']))
                or captured.get('sequence', 0) < baseline['sequence']):
            raise FastROSError('Command before-state is not newer than reviewed state')
        sent = next((e for e in relevant if e.get('event') == 'command_sent_unconfirmed'), None)
        stable = next((e for e in relevant if e.get('event') == 'command_observed_stable'), None)
        if sent is None or stable is None:
            return None
        count = len(envelope['expected_frames'])
        if sent.get('attempted_frames') != count or sent.get('socket_send_returns') != count:
            raise FastROSError('Partial/unconfirmed dispatch; never retry')
        if sent.get('unix_s', 0) < intent['unix_s'] or stable.get('unix_s', 0) < sent['unix_s']:
            raise FastROSError('Receipt timestamps are inconsistent')
        after = stable.get('after', {})
        if len(after.get('stamps', [])) != 14 or min(after['stamps']) <= sent['unix_s']:
            raise FastROSError('Receipt has no complete post-send feedback')
        return stable

    def execute(self, decision):
        self._open()
        if not self._action_lock.acquire(False):
            raise FastROSError('Another action is active; no concurrent dispatch')
        dispatched = False
        try:
            before = self.observe()
            decision = _decision(decision)
            envelope = self.prepare_command(decision, before)
            if self.proposal_only:
                return dict(status='not_dispatched', shadow=True, completed=False,
                            physical_motion_authorized=False, envelope=envelope,
                            blockers=self.authorization_status()['blockers'])
            self.validate(decision, before)  # Independent evidence gate and numeric bounds.
            transport = self._get_transport()
            identity = transport.identity()
            if identity['speed_percent'] != envelope['speed_percent']:
                raise FastROSError('Requested speed differs from live private parameter; not modified')
            if _last_command_sequence(transport.events())+1 != envelope['expected_command_sequence']:
                raise FastROSError('Another command appeared before dispatch')
            check_telemetry(before['raw_telemetry'], self.clock())
            started = self.clock()
            action_started = self.monotonic()
            deadline = self.monotonic()+125
            dispatched = True  # Any exception from here means outcome uncertain, even if zero TX.
            def before_send():
                fresh = self.observe()
                if (fresh['provenance']['command_sequence']+1 != envelope['expected_command_sequence']
                        or fresh['provenance']['speed_percent'] != envelope['speed_percent']
                        or fresh['provenance']['jaw_command_target_m'] != before['provenance']['jaw_command_target_m']):
                    raise FastROSError('Command/parameter/jaw intent changed during connection setup')
                a, b = fresh['raw_telemetry'], before['raw_telemetry']
                if (not all(x > y for x, y in zip(a['stamps'], b['stamps']))
                        or max(abs(x-y) for x, y in zip(a['q'], b['q'])) > .003
                        or max(abs(x-y) for x, y in zip(a['pose'][:3], b['pose'][:3])) > .0005
                        or rotation_distance(a['pose'], b['pose']) > .003
                        or abs(a['opening_m']-b['opening_m']) > .0005):
                    raise FastROSError('Reviewed state changed or lacks complete new feedback before send')
                self.validate(decision, fresh)
                check_telemetry(a, self.clock())
            transport._dispatch_once(envelope, before_send)
            while self.monotonic() < deadline:
                state = self._observe(min(3.0, max(.001, deadline-self.monotonic())), require_idle=False)
                self._numeric_limits(decision, state, envelope)  # Monitor explicit bounds throughout the wait.
                receipt = self._match_receipt(envelope, transport.events(), before, started)
                raw = state['raw_telemetry']
                if receipt and not raw['active_command'] and raw['driver_accepts_commands'] and raw['motion_status'] == 0:
                    check_telemetry(raw, self.clock())
                    if min(raw['stamps']) <= max(receipt['after']['stamps']):
                        continue
                    if envelope['kind'] == 'pose':
                        goal = envelope['encoded_target_m_rad']
                        if max(abs(a-b) for a, b in zip(raw['pose'][:3], goal[:3])) > .002 or rotation_distance(raw['pose'], goal) > .02:
                            raise FastROSError('Fresh pose does not confirm encoded target')
                    else:
                        if (max(abs(a-b) for a, b in zip(raw['q'], before['raw_telemetry']['q'])) > .003
                                or math.dist(raw['pose'][:3], before['raw_telemetry']['pose'][:3]) > .002
                                or rotation_distance(raw['pose'], before['raw_telemetry']['pose']) > .003):
                            raise FastROSError('Arm changed during gripper action')
                    width_error = raw['opening_m']-envelope['jaw_target_m']
                    return dict(status='command_observed_stable', completed=False, command_sequence=envelope['expected_command_sequence'],
                                arrival_confirmed=envelope['kind'] == 'pose', stability_confirmed=True,
                                robot_wait_s=max(0.0, self.monotonic()-action_started),
                                arm_target_reached=envelope['kind'] == 'pose', jaw_width_error_m=width_error,
                                jaw_target_reached=abs(width_error) <= .0015, grasp_verified=False,
                                task_success=False, hold_verified=False, state=state, receipt=receipt)
            raise TimeoutError('Action confirmation timed out; firmware target not cancelled')
        except BaseException as error:
            if dispatched:
                self.target_uncertain = True
                self.latch_failure(str(error))
            raise
        finally:
            self._action_lock.release()

    def latch_failure(self, reason):
        if self.failure is None:
            self.failure = str(reason)

    def close(self):
        if self._action_lock.locked():
            self.latch_failure('Close requested during an action; target outcome uncertain')
            self.target_uncertain = True
            raise FastROSError('Active action is not cancelled by close; no physical recovery sent')
        self.closed = True
        if self._transport is not None:
            self._transport.close()
