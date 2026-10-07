"""Independent runtime guards and narrowly scoped commissioning evidence gate.

No SDK, ROS, CAN, camera or network operations. Mock checks grant no physical
authority. The physical gate additionally requires raw recorded commissioning,
the same boot/source/driver session, fresh healthy feedback and explicit limits.
It never interprets bounded goal completion as cancellation or immediate hold.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time


class FastSafetyError(RuntimeError):
    pass


def _default_blockers():
    return ["hold_unqualified: no validated interruption/hold lifecycle",
            "live_adapter_uncommissioned: ROS observation and command encoding are implemented; physical dispatch/lifecycle remains unqualified",
            "physical_binding_scope_unqualified: read-only identity checks do not establish exclusive command ownership throughout physical execution",
            "physical_limits_unqualified: no qualified site motion limits and tool/path clearance scope; replay bounds are not physical evidence"]


_EVIDENCE_LOCK = threading.RLock()
_JSON_CACHE = {}
_QUALIFICATION_CACHE = {}
_BOOT_ID_PATH = Path('/proc/sys/kernel/random/boot_id')


def _file_signature(path):
    stat = path.stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)


def _absolute_path(value, name):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise FastSafetyError(name + ' must be an absolute file path')
    path = Path(value)
    if not path.is_file():
        raise FastSafetyError(name + ' must identify an existing file')
    return path


def _cached_json(path):
    """Cache immutable parses by inode/size/mtime/ctime plus verified content hash.

    Any normal write, replacement or timestamp edit invalidates the stat key.
    The content hash is computed on every cache miss; stat is checked again
    after reading to reject concurrent mutation. No config boolean bypasses it.
    """
    signature = _file_signature(path)
    key = str(path)
    cached = _JSON_CACHE.get(key)
    if cached is not None and cached[0] == signature:
        return cached
    if signature[2] > 128*1024*1024:
        raise FastSafetyError('Qualification artifact exceeds 128 MiB bound')
    payload = path.read_bytes()
    if _file_signature(path) != signature:
        raise FastSafetyError('Qualification artifact changed while reading')
    value = json.loads(payload)
    digest = hashlib.sha256(payload).hexdigest()
    result = (signature, digest, value)
    _JSON_CACHE[key] = result
    return result


def _physical_limits(config):
    from .fast_qualification import JOINT_LIMITS_RAW, RAD_PER_RAW
    limits = config.get('physical_limits') if isinstance(config, dict) else None
    if not isinstance(limits, dict):
        raise FastSafetyError('Explicit physical_limits are required')
    low = _vector(limits.get('workspace_min_m'), 3, 'workspace_min_m')
    high = _vector(limits.get('workspace_max_m'), 3, 'workspace_max_m')
    if any(not -1 <= a < b <= 1 for a, b in zip(low, high)):
        raise FastSafetyError('Workspace command_envelope must lie inside vendor encoding bounds')
    joints = limits.get('joint_limits_rad')
    if not isinstance(joints, list) or len(joints) != 6:
        raise FastSafetyError('Six explicit joint limits are required')
    for pair, (raw_low, raw_high) in zip(joints, JOINT_LIMITS_RAW):
        a, b = _vector(pair, 2, 'joint_limits_rad')
        if not raw_low*RAD_PER_RAW <= a < b <= raw_high*RAD_PER_RAW:
            raise FastSafetyError('Joint limits cannot exceed manufacturer nominal limits')
    _integer(limits.get('max_speed_percent'), 1, 1, 'max_speed_percent')
    _integer(limits.get('max_waypoints'), 1, 1, 'max_waypoints')
    for key, cap in (('max_translation_step_m', .03), ('max_rotation_step_rad', .05),
                     ('max_state_age_s', .1)):
        if not 0 < _number(limits.get(key), key) <= cap:
            raise FastSafetyError(key + ' exceeds commissioning scope')
    jaw_min = _number(limits.get('gripper_min_m'), 'gripper_min_m')
    jaw_max = _number(limits.get('gripper_max_m'), 'gripper_max_m')
    if not 0 <= jaw_min < jaw_max <= .055:
        raise FastSafetyError('Jaw command envelope must remain within 0..55 mm')
    if _number(limits.get('max_effort_parameter_nm'), 'max_effort_parameter_nm') != .2:
        raise FastSafetyError('Gripper effort parameter must remain exactly 0.2')
    return copy.deepcopy(limits)


def _collector(trial):
    """Return the complete original trace and its immutable dependency key."""
    reference = trial.get('collector_artifact')
    embedded = trial.get('collector_report')
    if (reference is None) == (embedded is None):
        raise FastSafetyError('One original collector_artifact or collector_report is required per trial')
    if reference is not None:
        if not isinstance(reference, dict):
            raise FastSafetyError('Malformed collector_artifact reference')
        path = _absolute_path(reference.get('path'), 'collector_artifact.path')
        signature, digest, report = _cached_json(path)
        if reference.get('sha256') != digest:
            raise FastSafetyError('Original collector artifact hash mismatch')
        key = (str(path), signature, digest)
    else:
        report, key = embedded, None
    if not isinstance(report, dict):
        raise FastSafetyError('Original collector report must be an object')
    return report, key


def _verify_collector(report, trial):
    if (report.get('mode') != 'passive_right_can_qualification_recording'
            or report.get('trace_transport_clean') is not True
            or any(report.get(k) != [] for k in ('bad_frames', 'sample_gaps', 'timestamp_backwards'))
            or type(report.get('socket_dropped_total')) is not int or report['socket_dropped_total'] != 0
            or type(report.get('frames_sent_by_this_script')) is not int or report['frames_sent_by_this_script'] != 0
            or report.get('sdk_used') is not False or report.get('interface_changed') is not False):
        raise FastSafetyError('Original collector transport report is not clean receive-only evidence')
    binding = report.get('binding', {})
    if (binding.get('channel') != 'can1' or binding.get('expected_usb_interface') != '1-6.3:1.0'
            or binding.get('binding_verified') is not True):
        raise FastSafetyError('Original collector binding mismatch')
    if report.get('samples') != trial.get('samples') or report.get('control_frames') != trial.get('control_frames'):
        raise FastSafetyError('Trial omits or changes original collector samples/control frames')


def _session_events(command_log):
    path = _absolute_path(command_log, 'session command_log')
    if path.stat().st_size > 16*1024*1024:
        raise FastSafetyError('Current driver log exceeds bounded review size')
    events = []
    for line in path.read_text().splitlines():
        if not line.startswith('{'):
            continue
        try:
            event = json.loads(line)
        except ValueError:
            continue  # Flushed receipts cannot be replaced by a partial last line.
        if isinstance(event, dict) and event.get('source') == 'ros_resume_entry':
            events.append(event)
    return events


def _verify_session(evidence, config):
    session = evidence.get('session')
    if not isinstance(session, dict):
        raise FastSafetyError('Current commissioning session identity is missing')
    command_log = str(_absolute_path(session.get('command_log'), 'session.command_log'))
    if config.get('ros', {}).get('command_log') != command_log:
        raise FastSafetyError('Configured command_log is not the commissioning session')
    adopted = _number(session.get('driver_adoption_unix_s'), 'driver adoption time')
    _integer(session.get('driver_pid'), 1, 2**31-1, 'commissioned driver PID')
    events = _session_events(command_log)
    adoptions = [e for e in events if e.get('event') == 'read_only_adoption_complete']
    if len(adoptions) != 1 or adoptions[0].get('unix_s') != adopted:
        raise FastSafetyError('Current log adoption does not match the commissioning session')
    for kind in ('J', 'P'):
        trial = evidence['trials'][kind]
        receipt = trial.get('driver_receipt')
        if not isinstance(receipt, dict):
            raise FastSafetyError('J/P qualification requires current-session source receipts')
        for key in ('intent', 'sent'):
            recorded = receipt.get(key)
            if (not isinstance(recorded, dict) or recorded not in events
                    or _number(recorded.get('unix_s'), 'receipt unix_s') <= adopted):
                raise FastSafetyError('Commissioning receipt is absent from current adopted driver log')
    intents = [e for e in events if e.get('event') == 'command_intent']
    sequences = [e.get('sequence') for e in intents]
    if any(type(seq) is not int for seq in sequences) or sequences != list(range(1, len(sequences)+1)):
        raise FastSafetyError('Current driver command sequence is incomplete or changed')
    if any(e.get('event') == 'command_refused_or_failed' and e.get('further_commands_blocked') is True for e in events):
        raise FastSafetyError('Current session contains a latched driver failure')
    result = copy.deepcopy(session)
    result['command_sequence'] = sequences[-1] if sequences else 0
    return result


def qualification_status(config=None):
    """Read/validate physical evidence; this report alone grants no live action.

    Prime this before acquiring short-lived feedback. Subsequent calls reuse
    immutable artifact parses but recheck their filesystem identity and the
    current driver's append-only session log.
    """
    status = dict(evidence_valid=False, qualified=False, physical_motion_authorized=False,
                  timeout_policy_verified=False, hold_verified=False, target_cancelled=False,
                  general_stop_validated=False, blockers=_default_blockers())
    try:
        if not isinstance(config, dict):
            raise FastSafetyError('No commissioning configuration')
        reference = config.get('physical_qualification')
        if not isinstance(reference, dict) or not reference.get('evidence_file'):
            raise FastSafetyError('No raw commissioning evidence_file')
        from .fast_ros import PINNED
        from .fast_qualification import validate_qualification
        path = _absolute_path(reference['evidence_file'], 'physical_qualification.evidence_file')
        boot_id = _BOOT_ID_PATH.read_text().strip()
        for name, expected in (('adapter_path', PINNED['adapter_sha256']), ('vendor_path', PINNED['vendor_sha256'])):
            if hashlib.sha256(Path(PINNED[name]).read_bytes()).hexdigest() != expected:
                raise FastSafetyError('Frozen physical driver source changed')
        with _EVIDENCE_LOCK:
            signature, digest, evidence = _cached_json(path)
            if not isinstance(evidence, dict) or not isinstance(evidence.get('trials'), dict):
                raise FastSafetyError('Malformed qualification evidence')
            collectors, dependencies = {}, []
            for kind in ('static_driver_exit', 'J', 'P'):
                trial = evidence['trials'].get(kind)
                if not isinstance(trial, dict):
                    raise FastSafetyError('Missing separate commissioning trial: ' + kind)
                collectors[kind], key = _collector(trial)
                dependencies.append(key)
            cache_key = (str(path), signature, digest, boot_id, tuple(dependencies))
            accepted = _QUALIFICATION_CACHE.get(cache_key)
            if accepted is None:
                for kind, collector in collectors.items():
                    _verify_collector(collector, evidence['trials'][kind])
                accepted = validate_qualification(evidence, boot_id=boot_id,
                    adapter_sha256=PINNED['adapter_sha256'], vendor_sha256=PINNED['vendor_sha256'])
                _QUALIFICATION_CACHE[cache_key] = accepted
            session = _verify_session(evidence, config)
        limits = _physical_limits(config)
        status.update(evidence_valid=True, qualified=True, blockers=[], error=None,
                      qualification=copy.deepcopy(accepted), evidence_file=str(path),
                      evidence_file_sha256=digest, session=session, limits=limits,
                      workspace_scope='command_envelope_not_clearance_or_collision_proof',
                      timeout_policy_verified=True,
                      timeout_policy='retain_driver_and_enable_allow_accepted_bounded_goal_to_complete_then_no_next_goal')
    except (OSError, ValueError, KeyError, TypeError, FastSafetyError) as error:
        status['error'] = str(error)
    return status


def physical_blockers(config=None):
    """Booleans never replace raw commissioning and current-session evidence."""
    return qualification_status(config)['blockers']


class FastSafetyGuard:
    """Scope-limited evidence gate plus independent live command constraints."""
    def __init__(self, config=None, *, clock=time.time):
        self.config = copy.deepcopy(config)
        self.clock = clock

    def preflight(self, measured_state):
        status = qualification_status(self.config)
        if not status['qualified']:
            raise FastSafetyError('Physical execution unavailable; ' + '; '.join(status['blockers'])
                                  + '; ' + str(status.get('error')))
        from .fast_ros import PINNED, check_telemetry
        if not isinstance(measured_state, dict) or measured_state.get('nonphysical') is not False:
            raise FastSafetyError('Fresh physical ROS state is required')
        provenance = measured_state.get('provenance', {})
        session = status['session']
        for key, expected in (('binding_verified', True), ('source_verified', True),
                              ('driver_pid', session['driver_pid']), ('driver_node', PINNED['driver_node']),
                              ('adapter_sha256', PINNED['adapter_sha256']), ('vendor_sha256', PINNED['vendor_sha256']),
                              ('can_interface', 'can1'), ('usb_interface', '1-6.3:1.0'), ('speed_percent', 1),
                              ('command_log', session['command_log']),
                              ('current_driver_adoption_unix_s', session['driver_adoption_unix_s']),
                              ('command_sequence', session['command_sequence'])):
            value = provenance.get(key)
            if value != expected or (type(expected) in (int, bool) and type(value) is not type(expected)):
                raise FastSafetyError('Current physical provenance mismatch: ' + key)
        if 'command_publishers' not in provenance or not isinstance(provenance['command_publishers'], dict):
            raise FastSafetyError('Current command ownership check is missing')
        publishers = provenance['command_publishers']
        if publishers:
            owners = publishers.get(PINNED['pose_topic'])
            if (set(publishers) != {PINNED['pose_topic']} or not isinstance(owners, list)
                    or len(owners) != 1 or not isinstance(owners[0], str) or not owners[0]):
                raise FastSafetyError('Unexpected concurrent command publishers')
        raw = measured_state.get('raw_telemetry')
        try:
            checks = check_telemetry(raw, self.clock())
        except (ValueError, RuntimeError, TypeError) as error:
            raise FastSafetyError('Current raw telemetry rejected: ' + str(error)) from error
        limits = status['limits']
        if self.clock()-checks['sampled_at'] > limits['max_state_age_s']:
            raise FastSafetyError('Current state exceeds explicit freshness limit')
        if raw['mode'] not in (0, 1):
            raise FastSafetyError('Only commissioned ordinary P/J feedback modes supported')
        if any(not a <= q <= b for q, (a, b) in zip(raw['q'], limits['joint_limits_rad'])):
            raise FastSafetyError('Measured joints outside explicit physical limits')
        self._workspace(raw['pose'], limits)
        if not limits['gripper_min_m'] <= raw['opening_m'] <= limits['gripper_max_m']:
            raise FastSafetyError('Current jaw outside physical command envelope')
        state = measured_state.get('robot_state', {})
        if (state.get('pose_m_rad') != raw['pose'] or state.get('joints_rad') != raw['q']
                or state.get('enabled') is not True or state.get('moving') is not False
                or measured_state.get('gripper_state', {}).get('opening_m') != raw['opening_m']):
            raise FastSafetyError('Compact state and raw telemetry disagree')
        status.update(physical_execution_ready=True, physical_motion_authorized=False,
                      path_or_ik_verified=False)
        return status

    @staticmethod
    def _workspace(pose, limits):
        if any(not a <= v <= b for a, v, b in zip(limits['workspace_min_m'], pose[:3], limits['workspace_max_m'])):
            raise FastSafetyError('Current/target reference outside explicit command_envelope')

    def validate(self, decision, measured_state):
        from .fast_policy import Decision, parse_response, phase_spec, phase_translation_fraction, phase_rotation_fraction
        status = self.preflight(measured_state)
        try:
            decision = parse_response(decision, require_explanation=(
                isinstance(decision, Decision) and decision.explanation is not None
                or isinstance(decision, dict) and 'explanation' in decision))
        except ValueError as error:
            raise FastSafetyError('Invalid physical decision: ' + str(error)) from error
        if decision.action not in ('move_eef', 'gripper') or decision.action not in phase_spec(decision.phase)['allowed_actions']:
            raise FastSafetyError('Only one phase-appropriate EEF or gripper action is commissioned; no chunks')
        args, limits = decision.arguments, status['limits']
        raw = measured_state['raw_telemetry']
        if decision.action == 'move_eef':
            _integer(args['speed_percent'], 1, 1, 'command speed_percent')
            target = _vector(args['pose_m_rad'], 6, 'pose_m_rad')
            if any(abs(v) > 2*math.pi for v in target[3:]):
                raise FastSafetyError('Target orientation outside vendor encoding envelope')
            encoded = [round(v*1000)/1000 for v in target[:3]] + [
                round(v*1000*180/3.1415926)*math.pi/180000 for v in target[3:]]
            for pose in (target, encoded):
                self._workspace(pose, limits)
                if (math.dist(raw['pose'][:3], pose[:3]) > limits['max_translation_step_m']*phase_translation_fraction(decision.phase)
                        or rotation_distance(raw['pose'], pose) > limits['max_rotation_step_rad']*phase_rotation_fraction(decision.phase)):
                    raise FastSafetyError('Phase-scaled physical movement exceeds commissioned bound')
        else:
            if not limits['gripper_min_m'] <= args['opening_m'] <= limits['gripper_max_m']:
                raise FastSafetyError('Jaw target outside commissioned envelope')
            if args['effort_parameter_nm'] != .2:
                raise FastSafetyError('Jaw effort must equal commissioned 0.2 parameter')
        # Include validation and artifact/session inspection time in freshness.
        if self.clock()-min(raw['stamps']) > limits['max_state_age_s']:
            raise FastSafetyError('Feedback aged during physical decision validation')
        status.update(physical_motion_authorized=True, accepted=True, nonphysical=False)
        return status


def require_nonphysical_runtime(robot, cameras):
    # Import only the known, hardware-free replay types. Flags/subclasses alone
    # cannot introduce another implementation through the executable runner.
    from .fast_replay import MockRobot
    from .fast_observation import HistoricalRGBSource
    if (type(robot) is not MockRobot or type(cameras) is not HistoricalRGBSource
            or robot.nonphysical is not True or cameras.nonphysical is not True):
        raise FastSafetyError("Only exact MockRobot + HistoricalRGBSource runtime is available; physical execution blocked")


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise FastSafetyError(name + " must be a finite number")
    return float(value)


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise FastSafetyError(name + " has wrong length")
    return [_number(v, name) for v in value]


def _positive(value, name):
    value = _number(value, name)
    if value <= 0:
        raise FastSafetyError(name + " must be positive")
    return value


def _integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise FastSafetyError(name + " must be an integer within its bound")
    return value


def rotation_distance(a, b):
    """SO(3) rotation distance for driver Rz(yaw) Ry(pitch) Rx(roll)."""
    def q(pose):
        r, p, y = (x / 2 for x in pose[3:])
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
    qa, qb = q(a), q(b)
    # Chord/atan2 avoids acos cancellation close to equal rotations.
    na, nb = math.sqrt(sum(x*x for x in qa)), math.sqrt(sum(x*x for x in qb))
    qa, qb = [x/na for x in qa], [x/nb for x in qb]
    if sum(x*y for x, y in zip(qa, qb)) < 0:
        qb = [-x for x in qb]
    minus = math.sqrt(sum((x-y)**2 for x, y in zip(qa, qb)))
    plus = math.sqrt(sum((x+y)**2 for x, y in zip(qa, qb)))
    return 4 * math.atan2(minus, plus)


_LIMITS = frozenset(("workspace_min_m", "workspace_max_m", "joint_limits_rad",
                    "max_speed_percent", "max_translation_step_m", "max_rotation_step_rad",
                    "max_chunk_translation_m", "max_chunk_rotation_rad", "max_waypoints",
                    "max_state_age_s", "gripper_min_m", "gripper_max_m", "max_effort_parameter_nm"))


class MockMotionGuard:
    """Check explicitly supplied mock limits; a ticket represents one mock action.

    Joint checks cover supplied feedback only; no IK or trajectory is computed.
    begin/finish serialise actions. Failure permanently seals this instance.
    """
    nonphysical = True

    def __init__(self, limits, *, nonphysical, clock=time.time):
        if nonphysical is not True:
            raise FastSafetyError("MockMotionGuard requires explicit nonphysical=True")
        if not isinstance(limits, dict):
            raise FastSafetyError("Explicit mock limits dictionary required")
        if set(limits) != _LIMITS:
            raise FastSafetyError("Explicit mock limits required; missing/unknown keys: " + str(sorted(_LIMITS ^ set(limits))))
        self.limits = copy.deepcopy(limits)
        lo = _vector(limits["workspace_min_m"], 3, "workspace_min_m")
        hi = _vector(limits["workspace_max_m"], 3, "workspace_max_m")
        if any(a >= b for a, b in zip(lo, hi)):
            raise FastSafetyError("Invalid workspace bounds")
        joints = limits["joint_limits_rad"]
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            raise FastSafetyError("Six explicit joint bounds required")
        for bound in joints:
            a, b = _vector(bound, 2, "joint_limits_rad")
            if a >= b:
                raise FastSafetyError("Invalid joint bounds")
        _integer(limits["max_speed_percent"], 1, 100, "max_speed_percent")
        _integer(limits["max_waypoints"], 1, 100, "max_waypoints")
        for key in ("max_translation_step_m", "max_rotation_step_rad", "max_chunk_translation_m",
                    "max_chunk_rotation_rad", "max_state_age_s", "max_effort_parameter_nm"):
            _positive(limits[key], key)
        low, high = (_number(limits[k], k) for k in ("gripper_min_m", "gripper_max_m"))
        if not 0 <= low < high:
            raise FastSafetyError("Invalid gripper bounds")
        self.clock, self.failure = clock, None
        self._ticket = None
        self.actions_started = 0
        self._lock = threading.RLock()

    def _open(self):
        if self.failure is not None:
            raise FastSafetyError("Failure latched; no retry: " + self.failure)

    def latch_failure(self, reason):
        with self._lock:
            if self.failure is None:
                self.failure = str(reason)
            self._ticket = None

    def check_state(self, measured_state):
        self._open()
        if not isinstance(measured_state, dict):
            raise FastSafetyError("Measured state must be an object")
        state = copy.deepcopy(measured_state.get("robot_state", measured_state))
        if not isinstance(state, dict):
            raise FastSafetyError("robot_state must be an object")
        if "opening_m" not in state:
            state["opening_m"] = measured_state.get("gripper_state", {}).get("opening_m")
        stamp = _number(state.get("sampled_at"), "sampled_at")
        age = _number(self.clock(), "clock") - stamp
        if not 0 <= age <= self.limits["max_state_age_s"]:
            raise FastSafetyError("Robot state stale or future-dated")
        if (state.get("enabled") is not True or state.get("binding_verified") is not True
                or state.get("moving") is not False):
            raise FastSafetyError("Requires enabled, stationary, bound mock state")
        for key in ("arm_status", "err_code"):
            if type(state.get(key)) is not int or state[key] != 0:
                raise FastSafetyError("Unhealthy or missing " + key)
        pose = _vector(state.get("pose_m_rad"), 6, "pose_m_rad")
        joints = _vector(state.get("joints_rad"), 6, "joints_rad")
        self._workspace(pose)
        if any(not lo <= q <= hi for q, (lo, hi) in zip(joints, self.limits["joint_limits_rad"])):
            raise FastSafetyError("Measured joints outside explicit mock bounds")
        self._opening(_number(state.get("opening_m"), "opening_m"))
        return state

    def _workspace(self, pose):
        if any(not a <= x <= b for x, a, b in zip(pose[:3], self.limits["workspace_min_m"], self.limits["workspace_max_m"])):
            raise FastSafetyError("Driver end reference outside mock workspace")

    def _opening(self, value):
        if not self.limits["gripper_min_m"] <= value <= self.limits["gripper_max_m"]:
            raise FastSafetyError("Gripper outside mock bounds")

    def validate(self, decision, measured_state):
        from .fast_policy import (parse_response, Decision, FastPolicyError, phase_translation_fraction,
                                  phase_rotation_fraction)
        self._open()
        if self._ticket is not None:
            raise FastSafetyError("An action is already active; no concurrent dispatch")
        explanation = (isinstance(decision, Decision) and decision.explanation is not None
                       or isinstance(decision, dict) and "explanation" in decision)
        try:
            decision = parse_response(decision, require_explanation=explanation)
        except FastPolicyError as exc:
            raise FastSafetyError("Invalid decision: " + str(exc)) from exc
        state = self.check_state(measured_state)
        args = decision.arguments
        total_distance = total_rotation = 0.0
        if decision.action in ("move_eef", "move_eef_chunk"):
            distance_fraction = phase_translation_fraction(decision.phase)
            rotation_fraction = phase_rotation_fraction(decision.phase)
            if not (0 < distance_fraction <= 1 and 0 < rotation_fraction <= 1):
                raise FastSafetyError("Invalid phase limit fractions")
            distance_limit = self.limits["max_translation_step_m"] * distance_fraction
            rotation_limit = self.limits["max_rotation_step_rad"] * rotation_fraction
            _integer(args["speed_percent"], 1, self.limits["max_speed_percent"], "speed_percent")
            points = [args["pose_m_rad"]] if decision.action == "move_eef" else args["waypoints"]
            if len(points) > min(self.limits["max_waypoints"], 3):
                raise FastSafetyError("Too many waypoints")
            previous = state["pose_m_rad"]
            for pose in points:
                self._workspace(pose)
                distance = math.sqrt(sum((x-y)**2 for x, y in zip(previous[:3], pose[:3])))
                angle = rotation_distance(previous, pose)
                if distance > distance_limit:
                    raise FastSafetyError("Translation step exceeds mock limit")
                if angle > rotation_limit:
                    raise FastSafetyError("Rotation step exceeds mock limit")
                total_distance += distance
                total_rotation += angle
                previous = pose
            if (total_distance > min(self.limits["max_chunk_translation_m"], self.limits["max_translation_step_m"]) * distance_fraction
                    or total_rotation > min(self.limits["max_chunk_rotation_rad"], self.limits["max_rotation_step_rad"]) * rotation_fraction):
                raise FastSafetyError("Cumulative chunk movement exceeds mock limit")
        elif decision.action == "gripper":
            self._opening(args["opening_m"])
            if args["effort_parameter_nm"] > self.limits["max_effort_parameter_nm"]:
                raise FastSafetyError("Gripper effort parameter exceeds mock limit")
        return {"accepted": True, "nonphysical": True, "physical_motion_authorized": False,
                "hold_verified": False, "path_or_ik_verified": False,
                "cumulative_translation_m": total_distance, "cumulative_rotation_rad": total_rotation}

    def begin(self, decision, measured_state):
        with self._lock:
            self.validate(decision, measured_state)
            self.check_state(measured_state)  # Include validation time in age.
            self._ticket = object()
            self.actions_started += 1
            return self._ticket

    def finish(self, ticket, measured_state):
        with self._lock:
            return self._finish(ticket, measured_state)

    def _finish(self, ticket, measured_state):
        self._open()
        if ticket is not self._ticket or ticket is None:
            self.latch_failure("Unknown or already consumed action ticket")
            raise FastSafetyError(self.failure)
        try:
            state = self.check_state(measured_state)
        except BaseException:
            self.latch_failure("Post-action state rejected")
            raise
        self._ticket = None
        return {"nonphysical": True, "feedback_ranges_checked": True,
                "physical_arrival_verified": False, "hold_verified": False, "state": state}
