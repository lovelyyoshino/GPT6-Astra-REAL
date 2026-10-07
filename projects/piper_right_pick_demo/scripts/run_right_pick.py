#!/usr/bin/env python3
"""One locally supervised red-cube pick attempt, using only the right arm."""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


class CameraClient:
    """Own a finite camera-only recording child; no external command interface."""
    def __init__(self, python, config, directory):
        self.python, self.config, self.directory = python, config, directory
        self.process = None
        self.events = queue.Queue()
        self.stderr = None

    def start(self):
        self.stderr = (self.directory.parent / 'camera_stderr.log').open('x')
        self.process = subprocess.Popen([self.python, str(ROOT / 'scripts/pick_camera_worker.py'),
            '--config', str(self.config), '--output-dir', str(self.directory)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
            text=True, bufsize=1)
        def read():
            for line in self.process.stdout:
                try:
                    item = json.loads(line)
                    if isinstance(item, dict):
                        self.events.put(item)
                except ValueError:
                    continue
            self.events.put({'event': 'closed'})
        threading.Thread(target=read, daemon=True).start()
        return self.wait_for('ready', 25.)

    def wait_for(self, event, seconds, label=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                item = self.events.get(timeout=min(.2, max(.001, deadline-time.monotonic())))
            except queue.Empty:
                continue
            if item.get('event') in ('failed', 'closed', 'request_error'):
                raise RuntimeError('Camera recording failed: ' + str(item))
            if item.get('event') == event and (label is None or item.get('label') == label):
                return item
        raise RuntimeError('Camera ' + event + ' timed out')

    def capture(self, label):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('Camera recorder is not running')
        self.process.stdin.write(json.dumps({'op': 'capture', 'label': label}) + '\n')
        self.process.stdin.flush()
        reply = self.wait_for('captured', 8., label)
        path = Path(reply['observation']).resolve()
        if self.directory.resolve() not in path.parents:
            raise RuntimeError('Unexpected camera output location')
        obs = json.loads(path.read_text())
        if set(obs['cameras']) != {'front', 'right_hand'} or time.time()-obs['captured_at'] > 2:
            raise RuntimeError('Unexpected cameras or stale snapshot')
        return str(path)

    def close(self):
        if self.process and self.process.poll() is None:
            try:
                self.process.stdin.write('{"op":"stop"}\n')
                self.process.stdin.flush()
                self.process.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                self.process.terminate()  # This child only owns cameras, never the robot.
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        if self.stderr:
            self.stderr.close()


def write_report(path, report):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/site.local.json')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--resume-from', type=Path, help='Continue a fully sent, pre-close arrival-timeout plan')
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.output_dir / 'report.json'
    if report_path.exists():
        parser.error('Use a new run directory')
    # Imports are offline. All dependencies must work before any motor command.
    from pick_scene_geometry import estimate_scene
    from pick_trajectory import plan_pick
    from direct_sdk_pick import PickController
    from direct_sdk_live_step import LiveFeedback, require_no_drift
    if args.resume_from is not None:
        from pick_resume_plan import resume_plan
        from pick_resume_scene import verify_resume_scene
    import direct_sdk_step as core
    config = json.loads(args.config.read_text())
    report = {'operation': 'one_right_red_cube_pick_attempt', 'status': 'preparing',
              'started_at_s': time.time(), 'transmissions': [], 'trace': [],
              'captures': [], 'physical_grasp_attempts': 0, 'task_success': 'not_evaluated',
              'strategy': 'estimate complete path, execute finite stages with actual feedback',
              'estimated_geometry_not_calibration': True, 'exit_does_not_cancel_target': True}
    camera = CameraClient(config['host_capture']['camera_python'], args.config,
                          args.output_dir / 'camera_recording')
    receiver, state, controller = None, None, None
    exit_code = 2
    try:
        report['binding'] = core.inspect_binding()
        core.inspect_controllers()
        if config['host_capture']['right_can_interface'] != 'can2':
            raise core.Rejected('Right arm binding differs from can2')
        if args.resume_from is None:
            print('一次右臂抓放尝试：23 mm 开口观测 → 张开 55 mm → 调整姿态 → 接近红块 → 夹持 → 抬起 → 侧移 → 放下。', flush=True)
            print('只操作右臂；预计向桌面前伸约 30 cm 并转腕。请清空整段右臂运动区域，移开手，准备现场断电。', flush=True)
        else:
            print('续接尚未闭爪的抓放：核对当前位置和画面 → 最后接近 → 闭爪 → 抬升 → 侧移 → 释放 → 撤离。', flush=True)
            print('保持右臂、方块和相机位置；清空剩余运动区域，移开手，准备现场断电。', flush=True)
            report['resume_source'] = str(args.resume_from.resolve())
        print('使用未完全标定的几何估计，可能抓空；不会自动再抓。退出程序不能取消已发送目标。', flush=True)
        if not sys.stdin.isatty() or input('准备好后按 Enter 开始；输入文字取消：').strip():
            raise core.Rejected('Operator cancelled or noninteractive input')
        vendor = core.Vendor()
        state = LiveFeedback(vendor)
        receiver = core.Receiver(state)
        controller = PickController(vendor, receiver, state, report)
        controller.enable()
        controller.observe(camera.start, timeout=30.)
        if args.resume_from is None:
            # 55 mm loses one fingertip's stereo depth. Symmetric opening after
            # narrow-aperture measurement preserves the midpoint TCP.
            controller.gripper(23000, 'observe')
        def capture(label, sample=None):
            # This callback runs in a worker; receive/cache mutation remains
            # exclusively in the main thread. The surrounding observe() checks
            # the held pose both before and after image acquisition.
            before = sample if sample is not None else controller.held
            observation = camera.capture(label)
            item = {'label': label, 'observation': observation, 'robot_before_capture': before}
            return item
        initial = controller.observe(lambda: capture('initial'), timeout=10.)
        initial_sample = state.snapshot(time.monotonic(), True)
        report['initial_scene'] = initial
        report['captures'].append({'label': 'initial', 'result': initial, 'sample': initial_sample})
        if args.resume_from is None:
            geometry = controller.observe(lambda: estimate_scene(initial['observation'],
                initial_sample['joints_raw'], initial_sample['pose_raw']), timeout=20.)
            geometry['provenance']['measurement_gripper_width_raw'] = state.gripper['angle_raw']
            geometry['provenance']['aperture_change_assumption'] = (
                'Symmetric jaw opening preserves the central TCP and camera mount; '
                'held arm feedback must agree before and after opening.')
            opened = controller.gripper(55000, 'open')
            require_no_drift(controller.held, initial_sample)
            report['arm_pose_held_across_gripper_opening'] = True
            geometry['provenance']['post_measurement_open_width_raw'] = opened['width_raw']
            prepared = controller.observe(lambda: capture('prepared'), timeout=10.)
            require_no_drift(controller.held, initial_sample)
            report['captures'].append({'label': 'prepared', 'result': prepared,
                                       'sample': controller.held})
            print('实时右腕估计已取得，正在求解并检查整条抓放路径。', flush=True)
            plan = controller.observe(lambda: plan_pick(initial_sample['joints_raw'],
                initial_sample['pose_raw'], geometry), timeout=30.)
        else:
            # The enable/stationary and camera observe steps establish a held
            # current pose. Check the recorded unfinished plan and current scene
            # before sending any gripper or arm target. Never narrow above cube.
            plan = controller.observe(lambda: resume_plan(args.resume_from, initial_sample), timeout=30.)
            report['resume_scene_check'] = controller.observe(lambda: verify_resume_scene(
                args.resume_from, initial['observation']), timeout=20.)
            geometry = plan['geometry']
            controller.gripper(55000, 'open')
            require_no_drift(controller.held, initial_sample)
            print('当前姿态、画面与原计划一致，剩余路径检查通过。', flush=True)
        report['geometry'] = geometry
        report['plan'] = plan
        write_report(args.output_dir / 'planned_attempt.json', report)
        print('整条路径检查通过，开始逐段执行；每段都确认反馈。', flush=True)
        controller.execute(plan, capture)
        if report.get('status') == 'protocol_completed':
            final_capture = controller.observe(lambda: capture('final'), timeout=10.)
            report['captures'].append({'label': 'final', 'result': final_capture,
                                       'sample': state.snapshot(time.monotonic(), True)})
        if report.get('grasp_outcome') == 'empty' and report.get('status') == 'protocol_completed':
            report['status'] = 'attempt_sequence_completed_empty'
            report['task_success'] = False
            exit_code = 0
        elif report.get('status') == 'grasp_empty':
            report['task_success'] = False
            exit_code = 3
        elif report.get('status') == 'grasp_ambiguous':
            report['task_success'] = 'not_verified'
            exit_code = 3
        else:
            report['status'] = 'attempt_sequence_completed'
            report['task_success'] = 'pending_visual_review'
            exit_code = 0
        print('动作流程结果：' + report['status'] + '；夹持结果：' + str(report.get('grasp_outcome', '待复核')), flush=True)
    except KeyboardInterrupt:
        report.update(status='interrupted', error='Operator interrupted', error_type='KeyboardInterrupt')
        exit_code = 130
    except Exception as exc:
        report.update(status='failed_or_stopped', error=str(exc), error_type=type(exc).__name__)
        print('本次结果：' + str(exc), file=sys.stderr, flush=True)
        if controller is not None:
            try:
                controller.fail(exc)
            except Exception as monitor_error:
                report['failure_monitor_error'] = str(monitor_error)
    finally:
        if controller is not None:
            controller.close()
        if state is not None:
            report['final_diagnostic'] = state.diagnostic(time.monotonic())
        if receiver is not None:
            receiver.close()
        camera.close()
        report['finished_at_s'] = time.time()
        write_report(report_path, report)
        print('本次记录：' + str(report_path), flush=True)
        if report['transmissions']:
            print('程序不会自动失能或复位；退出不代表已撤销机械臂目标。', flush=True)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
