#!/usr/bin/env python3
"""Dispatch a model-authored pose table through vendor IK. No task planner or IK here."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = ROOT / 'runs/model_direct_sdk_20261004/continuation_plan.json'
CAMERA_PYTHON = '/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10'
IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
MODES = {'MOVE_P': 0, 'MOVE_L': 2}
JOINT_LIMITS = [(-150, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120)]
EXECUTION_HOLD = ROOT / 'runs/vendor_execution_hold.json'


def require_execution_available():
    # The historical pause cannot be lifted merely by deleting its marker.
    # No qualified physical interruption/hold lifecycle exists in this executor.
    reason = 'Execution paused by ' + str(EXECUTION_HOLD) if EXECUTION_HOLD.exists() else 'Physical execution unavailable'
    raise RuntimeError(reason + '; hold_unverified; no qualified physical executor')


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def finite_vector(value, count):
    return (isinstance(value, list) and len(value) == count and
            all(type(x) in (int, float) and math.isfinite(x) for x in value))


def near_pose(actual, target, xyz=2.0, angle=1.0):
    return (all(abs(a-b) <= xyz for a, b in zip(actual[:3], target[:3])) and
            all(abs((a-b+180) % 360-180) <= angle for a, b in zip(actual[3:], target[3:])))


def validate_plan(plan):
    require(plan['channel'] == 'can2' and plan['usb_interface'] == '1-6.3:1.0', 'Only the bound right arm is allowed')
    require(plan['inverse_kinematics_owner'] == 'manufacturer_arm_controller', 'Manufacturer IK required')
    require(finite_vector(plan['expected_start_pose_mm_deg'], 6), 'Invalid start pose')
    require(finite_vector(plan['expected_start_joints_deg'], 6), 'Invalid start joints')
    require(plan['allowed_initial_arm_status'] in ([0], [0, 4]), 'Unsupported initial status exception')
    bounds = plan['workspace_mm']
    require(len(bounds) == 3 and all(finite_vector(b, 2) and b[0] < b[1] for b in bounds), 'Invalid workspace')
    require(1 <= len(plan['stages']) <= 16, 'Invalid stage count')
    names, closes = set(), 0
    for stage in plan['stages']:
        require(isinstance(stage['id'], str) and stage['id'].replace('_', '').isalnum() and
                0 < len(stage['id']) <= 30 and stage['id'] not in names, 'Invalid/duplicate stage id')
        names.add(stage['id'])
        if stage['operation'] == 'move':
            pose = stage['sdk_pose_mm_deg']
            require(finite_vector(pose, 6), 'Invalid pose')
            require(all(lo+2 <= v <= hi-2 for v, (lo, hi) in zip(pose, bounds)), 'Target outside workspace')
            require(all(abs(v) <= 180 for v in pose[3:]), 'Euler angle out of range')
            require(stage['mode'] in MODES and type(stage['speed_percent']) is int and
                    1 <= stage['speed_percent'] <= 5, 'Unsupported mode/speed')
            require(stage['mode'] != 'MOVE_P' or pose[2] >= 240, 'MOVE_P only allowed at selected high points')
        else:
            require(stage['operation'] == 'gripper', 'Unknown operation')
            width, torque = stage['opening_mm'], stage['torque_parameter_nm']
            require(finite_vector([width, torque], 2) and 0 <= width <= 60 and 0 < torque <= 0.3, 'Invalid gripper command')
            closes += width < 5
    require(closes <= 1, 'At most one closing attempt')
    if 4 in plan['allowed_initial_arm_status']:
        require(plan['stages'][0]['operation'] == 'move' and plan['stages'][0]['mode'] == 'MOVE_P', 'First command must replace rejected target')


def freshness(state, after_mono=0, after_wall=0):
    for can_id in IDS:
        key = str(can_id)
        mono, wall = state['rx'].get(key, 0), state['rx_wall'].get(key, 0)
        require(0 <= state['mono_s']-mono <= 0.75 and
                -0.05 <= state['time_s']-wall <= 0.75, 'Stale/missing feedback: ' + hex(can_id))
        if mono <= after_mono or wall <= after_wall:
            return False
    return True


def healthy(state, plan, allowed_status=(0,)):
    freshness(state)
    s = state['status']
    require(s['arm_status'] in allowed_status, 'Controller rejected/faulted: arm_status=%s' % s['arm_status'])
    require(s['err_code'] == 0 and s['ctrl_mode'] == 1 and s['teach_status'] == 0, 'Error/control/teach mode changed')
    require(all(code & 0x40 and not code & 0xBF for code in state['motor_codes']), 'Joint motor disabled/faulted')
    grip = state['gripper']
    require(grip['status_code'] & 0x40 and not grip['status_code'] & 0x3F, 'Gripper disabled/faulted')
    require(-1 <= grip['opening_mm'] <= 65, 'Abnormal gripper feedback')
    require(finite_vector(state['pose_mm_deg'], 6) and finite_vector(state['joints_deg'], 6), 'Invalid measured pose/joints')
    require(all(lo <= x <= hi for x, (lo, hi) in zip(state['pose_mm_deg'], plan['workspace_mm'])), 'Measured end reference outside workspace')
    require(all(lo-0.5 <= x <= hi+0.5 for x, (lo, hi) in zip(state['joints_deg'], JOINT_LIMITS)), 'Measured joint outside nominal limits')


class Cameras:
    def __init__(self, output):
        self.output, self.events = output, queue.Queue()
        self.stderr = (output.parent / 'camera_stderr.log').open('x')
        try:
            self.process = subprocess.Popen([CAMERA_PYTHON, '-B', str(Path(__file__).with_name('vendor_camera_record.py')),
                '--output-dir', str(output)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=self.stderr, text=True, bufsize=1)
        except Exception:
            self.stderr.close()
            raise
        def read_events():
            for line in self.process.stdout:
                try:
                    self.events.put(json.loads(line))
                except ValueError:
                    self.events.put({'event': 'error', 'error': 'Invalid camera event: ' + line})
        threading.Thread(target=read_events, daemon=True).start()

    def wait(self, wanted, timeout=20):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                event = self.events.get(timeout=0.2)
            except queue.Empty:
                require(self.process.poll() is None, 'Camera worker exited; see camera_stderr.log')
                continue
            require(event['event'] not in ('error', 'request_error', 'closed'), 'Camera error: ' + str(event))
            if event['event'] == wanted:
                return event
        raise TimeoutError('Camera did not report ' + wanted)

    def check(self):
        require(self.process.poll() is None, 'Camera worker exited')
        require(time.time() - (self.output / 'frames.jsonl').stat().st_mtime < 3, 'Camera frames stopped')

    def capture(self, label):
        self.process.stdin.write(json.dumps({'op': 'capture', 'label': label}) + '\n')
        self.process.stdin.flush()
        return self.wait('captured', 4)

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.stdin.write('{"op":"stop"}\n')
                self.process.stdin.flush()
                self.process.wait(timeout=8)
        except (OSError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        finally:
            self.stderr.close()
        return self.process.returncode


class Executor:
    def __init__(self, arm, plan, output, cameras, *, nonphysical=False):
        self.arm, self.plan, self.output, self.cameras = arm, plan, output, cameras
        self.nonphysical = nonphysical
        self._execution_gate()  # Before opening the trace, even outside main().
        self.failed = False
        self.pending = False
        self.sent_mono, self.sent_wall = 0, 0
        self.trace = (output / 'feedback.jsonl').open('x', buffering=1)
        self.report = {'status': 'preflight', 'stages': [], 'grasp_result': 'not_visually_evaluated',
                       'manufacturer_ik': True, 'started_at_s': time.time(),
                       'nonphysical': nonphysical, 'hold_unverified': True,
                       'physical_execution_available': False}
        self.started = time.monotonic()

    def _execution_gate(self):
        if self.nonphysical is not True:
            require_execution_available()
        # Explicit injected offline doubles only; never accept the SDK or the
        # real camera worker simply because a caller toggled a config boolean.
        require(getattr(self.arm, 'nonphysical', None) is True and
                getattr(self.cameras, 'nonphysical', None) is True,
                'Explicit nonphysical test doubles required')
        require(not any(hasattr(self.arm, name) for name in ('CreateCanBus', 'ConnectPort', 'GetCanBus'))
                and not isinstance(self.cameras, Cameras), 'Hardware adapters cannot use the offline test gate')

    def _seal_failure(self, reason):
        self.failed = True
        self.report.update(failure_latched=True, further_tx_blocked=True,
                           hold_unverified=True, physical_target_cancelled=False,
                           onsite_intervention_required=True)
        self.report.setdefault('failure_reason', str(reason))

    def _send(self, method, *args):
        self._execution_gate()
        require(not self.failed, 'Failure latched; no subsequent transmission or retry')
        try:
            self.arm.send_checked(method, *args)
        except BaseException as exc:
            self._seal_failure(exc)
            raise

    def read(self):
        self.cameras.check()
        require(not self.arm.fatal_error, 'SDK receive failure: ' + str(self.arm.fatal_error))
        require(not self.arm.send_errors, 'SDK error: ' + str(self.arm.send_errors))
        require(time.monotonic() - self.started < 240, 'Execution time limit')
        state = self.arm.snapshot()
        self.trace.write(json.dumps(state) + '\n')
        return state

    def preflight(self):
        self._execution_gate()
        require(not self.failed, 'Failure latched; no restart')
        try:
            return self._preflight()
        except BaseException as exc:
            self._seal_failure(exc)
            raise

    def _preflight(self):
        # Receive-only startup; all split feedback and all six motors must appear.
        end = time.monotonic() + 4
        while True:
            state = self.read()
            try:
                freshness(state)
                break
            except RuntimeError:
                if time.monotonic() >= end:
                    raise
                time.sleep(0.05)
        anchor = state['joints_deg']
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            state = self.read()
            healthy(state, self.plan, self.plan['allowed_initial_arm_status'])
            require(state['status']['motion_status'] == 0, 'Arm still moving')
            require(near_pose(state['pose_mm_deg'], self.plan['expected_start_pose_mm_deg'], 3, 1), 'Start pose changed; do not execute this plan')
            require(max(abs(a-b) for a,b in zip(state['joints_deg'], self.plan['expected_start_joints_deg'])) <= 1, 'Start joints changed')
            require(max(abs(a-b) for a,b in zip(state['joints_deg'], anchor)) <= 0.15, 'Arm drifting/touched during preflight')
            require(state['gripper']['opening_mm'] >= 50, 'Expected open, empty gripper')
            time.sleep(0.05)
        self.report['initial'] = state
        return state

    def step(self, stage, first=False):
        self._execution_gate()
        require(not self.failed and not self.pending, 'Failure latched or previous action pending; no retry')
        try:
            return self._step(stage, first)
        except BaseException as exc:
            self._seal_failure(exc)
            raise

    def _step(self, stage, first=False):
        before = self.read()
        allowed = self.plan['allowed_initial_arm_status'] if first else (0,)
        healthy(before, self.plan, allowed)
        require(before['status']['motion_status'] == 0, 'Previous motion not settled')
        record = {'stage': stage, 'before': before, 'status': 'sending'}
        self.report['stages'].append(record)
        self.pending = True  # Even a partial three-frame send must enter abort handling.
        self.sent_mono, self.sent_wall = time.monotonic(), time.time()
        if stage['operation'] == 'move':
            mode = MODES[stage['mode']]
            self._send('MotionCtrl_2', 1, mode, stage['speed_percent'], 0)
            self._send('EndPoseCtrl', *[int(round(v*1000)) for v in stage['sdk_pose_mm_deg']])
        else:
            self._send('GripperCtrl', int(round(stage['opening_mm']*1000)),
                                  int(round(stage['torque_parameter_nm']*1000)), 1, 0)
        sent_mono, sent_wall = time.monotonic(), time.time()
        self.sent_mono, self.sent_wall = sent_mono, sent_wall
        record.update(sent_at_s=sent_wall, status='waiting_for_feedback')
        stable_since, anchor, normal_seen = None, None, False
        while time.monotonic() - sent_mono < (40 if stage['operation'] == 'move' else 8):
            state = self.read()
            age = time.monotonic() - sent_mono
            # The known rejected target is latched. Only the first replacement gets a short grace.
            status_allow = (0, 4) if first and before['status']['arm_status'] == 4 and age < 0.6 and not normal_seen else (0,)
            healthy(state, self.plan, status_allow)
            if not freshness(state, sent_mono, sent_wall):
                time.sleep(0.05)
                continue
            s = state['status']
            normal_seen |= s['arm_status'] == 0
            ok = s['arm_status'] == 0 and s['motion_status'] == 0
            if stage['operation'] == 'move':
                ok &= s['mode_feed'] == mode and near_pose(state['pose_mm_deg'], stage['sdk_pose_mm_deg'])
                value, tolerance, dwell = state['joints_deg'], 0.12, 0.4
            else:
                require(near_pose(state['pose_mm_deg'], before['pose_mm_deg'], 2, 1), 'Arm moved during gripper operation')
                value, tolerance, dwell = [state['gripper']['opening_mm']], 0.25, 0.8
                ok &= age >= 1 and (stage['opening_mm'] < 5 or abs(value[0]-stage['opening_mm']) <= 1.5)
                if stage['opening_mm'] < 5:
                    ok &= value[0] <= stage['opening_mm']+1.5 or value[0] <= before['gripper']['opening_mm']-1
            if not ok:
                stable_since, anchor = None, None
            elif anchor is None or max(abs(a-b) for a,b in zip(value, anchor)) > tolerance:
                stable_since, anchor = time.monotonic(), value
            elif time.monotonic() - stable_since >= dwell:
                self.pending = False
                record.update(status='target_reached' if stage['operation'] == 'move' else 'gripper_settled', after=state)
                if stage['operation'] == 'gripper' and stage['opening_mm'] < 5:
                    record['object_held'] = 'unknown; settled width does not prove a grasp'
                record['camera'] = self.cameras.capture(stage['id'])
                save(self.output / 'report.json', self.report)
                return
            time.sleep(0.05)
        raise TimeoutError('Target not reached: ' + stage['id'])

    def abort(self):
        self._seal_failure('abort requested; no physical recovery is qualified')
        if 'stop' in self.report:
            return  # Never restart a completed passive tail or any action.
        stop = {'requested': False, 'confirmed': False, 'action': 'tx_latched_passive_observation_only',
                'hold_unverified': True, 'physical_target_cancelled': False,
                'onsite_intervention_required': True, 'observations': [], 'observation_errors': [],
                'note': 'No stop, reset, disable, hold target or retry is sent. Accepted firmware targets may continue.'}
        self.report['stop'] = stop
        if not self.pending:
            return
        print('已封锁后续发送；保持与目标取消均未验证，需要现场介入。仅继续记录反馈。', flush=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                state = self.arm.snapshot()
                stop['last_feedback'] = state
                stop['observations'].append(state)
                self.trace.write(json.dumps(state) + '\n')
                # Qualification failure is diagnostic; it never triggers TX.
                require(freshness(state, self.sent_mono, self.sent_wall), 'No post-command feedback')
            except KeyboardInterrupt:
                stop['observation_interrupted'] = True
                break
            except Exception as exc:
                if len(stop['observation_errors']) < 20:
                    stop['observation_errors'].append(str(exc))
            time.sleep(0.05)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--check-only', action='store_true', help='Validate data only; no SDK/camera/device access')
    args = parser.parse_args(argv)
    plan = json.loads(args.plan.read_text())
    validate_plan(plan)
    if args.check_only:
        print('Plan valid; no hardware accessed, no IK/path/grasp verification.')
        return 0
    try:
        require_execution_available()
    except RuntimeError as exc:
        print(str(exc), flush=True)
        return 2
    output = ROOT / 'runs' / ('vendor_sequence_' + time.strftime('%Y%m%dT%H%M%S') + '_%d' % os.getpid())
    output.mkdir()
    save(output / 'plan.json', plan)
    save(output / 'source_hashes.json', {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__), Path(__file__).with_name('vendor_sdk_feedback.py'), Path(__file__).with_name('vendor_camera_record.py')]})
    print('结果目录：' + str(output), flush=True)
    arm = cameras = executor = None
    lock = (ROOT / 'runs/direct_sdk_step.lock').open('a')
    failed, error = False, None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt('SIGTERM')))
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        device = Path('/sys/class/net/can2/device').resolve()
        require(device.name == plan['usb_interface'], 'CAN2 physical USB binding changed')
        # Read-only capability probe: an EPERM here occurs before SDK or camera startup.
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as probe:
            probe.bind(('can2',))
        cameras = Cameras(output / 'cameras')
        cameras.wait('ready')
        from vendor_sdk_feedback import create_arm
        arm = create_arm()
        arm.CreateCanBus('can2', judge_flag=False)
        arm.ConnectPort(piper_init=False)
        executor = Executor(arm, plan, output, cameras)
        executor.preflight()
        print('将按预先给定的位姿表执行一次抓放；5%速度，厂家逆解，无自动重试。', flush=True)
        input('确认旧SDK交互终端已退出、区域无人、可立即断电，按 Enter 开始：')
        executor.preflight()  # Recheck after an arbitrarily long operator wait.
        executor.report['status'] = 'running'
        for index, stage in enumerate(plan['stages']):
            print('[%d/%d] %s' % (index+1, len(plan['stages']), stage['id']), flush=True)
            executor.step(stage, first=index == 0)
        executor.report['status'] = 'sequence_completed'
    except (Exception, KeyboardInterrupt) as exc:
        failed, error = True, type(exc).__name__ + ': ' + str(exc)
        print('未完成：' + error, flush=True)
        if executor:
            executor.report.update(status='failed', error=error)
            executor.abort()
    finally:
        if cameras:
            try:
                camera_exit = cameras.close()
                require(camera_exit == 0, 'Camera recording failed during cleanup')
            except Exception as exc:
                failed, error = True, str(exc)
        if arm:
            try:
                arm.DisconnectPort()
            except Exception as exc:
                failed, error = True, 'SDK disconnect: ' + str(exc)
            finally:
                arm.close_error_capture()
        report = executor.report if executor else {'status': 'startup_failed', 'error': error, 'motion_commands_sent': False}
        if failed:
            report.update(status='failed', error=error)
        report['finished_at_s'] = time.time()
        save(output / 'report.json', report)
        if executor:
            executor.trace.close()
        lock.close()
        print('已保存报告：' + str(output / 'report.json'), flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
