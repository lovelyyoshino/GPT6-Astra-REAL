#!/usr/bin/env python3
"""Execute an already selected pose table through the SDK; no perception or IK planner."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import direct_sdk_step as core
from direct_sdk_live_step import LiveFeedback, require_no_drift
from run_right_pick import CameraClient, write_report
from sdk_pose_batch import PoseBatchController

ROOT = Path(__file__).resolve().parents[1]


def load_plan(path):
    plan = json.loads(path.read_text())
    if (plan.get('schema_version') != 1 or plan.get('arm') != 'right' or
            plan.get('channel') != 'can2' or plan.get('motion_mode') != 'MOVE_L'):
        raise core.Rejected('Expected a right-arm can2 MOVE_L pose table')
    source = Path(plan['source_report'])
    if hashlib.sha256(source.read_bytes()).hexdigest() != plan['source_report_sha256']:
        raise core.Rejected('Referenced scene and motion evidence changed')
    if plan.get('stages', [{}])[0].get('kind') == 'gripper':
        previous = json.loads(source.read_text())
        last = previous.get('stages', [{}])[-1]
        if (plan['stages'][0].get('label') != 'close' or
                previous.get('post_failure_target_stable') is not True or
                previous.get('partial_motion_target') is not False or
                previous.get('physical_grasp_attempts') != 0 or
                previous.get('gripper_close_commands_attempted') != 0 or
                last.get('kind') != 'move' or last.get('command_sent') is not True):
            raise core.Rejected('Close-first continuation lacks an ungrasped, fully sent, arrived source')
        final = previous['final_diagnostic']
        recorded = {'pose_raw': [final['pose_raw'][k] for k in core.POSE_NAMES],
                    'joints_raw': [final['joints_raw'][k] for k in core.JOINT_NAMES]}
        require_no_drift(recorded, {'pose_raw': plan['start_pose_raw'],
                                   'joints_raw': plan['start_joints_raw']})
        if (core.pose_difference(recorded['pose_raw'], last['target_pose_raw'])[0] > 1 or
                core.rotation_distance_deg(recorded['pose_raw'], last['target_pose_raw']) > .3):
            raise core.Rejected('Continuation source final pose did not reach its sent target')
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--review-only', action='store_true')
    args = parser.parse_args(argv)
    plan = load_plan(args.plan)
    vendor = core.Vendor()  # SDK construction with can_auto_init=False: no connection.
    reference = {k: plan[k] for k in ('start_pose_raw', 'start_joints_raw')}
    reference = {'pose_raw': reference['start_pose_raw'], 'joints_raw': reference['start_joints_raw']}
    if args.review_only:
        controller = PoseBatchController(vendor, None, None, {})
        controller.validate_pose_plan(plan, reference)
        for stage in plan['stages']:
            if stage['kind'] == 'move':
                core.encoded_plan(vendor.sdk, stage['pose_raw'])
        print('离线数据和厂家 SDK 编码检查通过；未连接设备，未验证固件逆解或真实路径。')
        return 0
    if args.output_dir is None:
        parser.error('--output-dir is required for device execution')
    try:
        held_lock, expected_lock = os.fstat(9), (ROOT / 'runs/direct_sdk_step.lock').stat()
        if (held_lock.st_dev, held_lock.st_ino) != (expected_lock.st_dev, expected_lock.st_ino):
            raise ValueError('Wrong execution lock')
        fcntl.flock(9, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (OSError, ValueError):
        parser.error('Start device execution through try_right_pose_batch.sh so the shared lock is held')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path = args.output_dir / 'report.json'
    if path.exists():
        parser.error('Use a new output directory')
    config = json.loads((ROOT / 'configs/site.local.json').read_text())
    report = {'operation': 'model_selected_sdk_cartesian_batch', 'status': 'preparing',
              'started_at_s': time.time(), 'plan': plan, 'plan_path': str(args.plan.resolve()),
              'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest(),
              'physical_grasp_attempts': 0, 'task_success': 'not_evaluated',
              'online_model_replanning': False, 'runtime_inverse_solver': 'arm_controller',
              'external_ik_calls': 0, 'home_commands': 0, 'trace': [], 'transmissions': [],
              'stages': [], 'captures': [], 'new_controller_path_previously_verified': False}
    camera = CameraClient(config['host_capture']['camera_python'], ROOT / 'configs/site.local.json',
                          args.output_dir / 'camera_recording')
    receiver = controller = state = None
    result = 2
    write_report(path, report)
    try:
        report['binding'] = core.inspect_binding()
        core.inspect_controllers()
        if plan['stages'][0]['kind'] == 'gripper':
            print('从当前已下探的抓取高度续接：核对状态和画面后闭爪，再抬升、侧移、释放和撤离。', flush=True)
        else:
            print('从当前接近抓取姿态执行这份末端位姿表，不回零、不展开、不重复前60段。', flush=True)
        for stage in plan['stages']:
            if stage['kind'] == 'move':
                values = [v / 1000. for v in stage['pose_raw']]
                print(stage['label'], 'XYZ(mm):', values[:3], 'RPY(deg):', values[3:], flush=True)
            else:
                print(stage['label'], '夹爪开口(mm):', stage['width_raw'] / 1000., flush=True)
        print('清空右臂运动区域并准备现场断电。中断程序不保证取消已发送目标。', flush=True)
        if not sys.stdin.isatty() or input('按一次 Enter 执行整份计划；输入文字取消：').strip():
            raise core.Rejected('Operator cancelled or noninteractive input')
        state = LiveFeedback(vendor)
        receiver = core.Receiver(state)
        controller = PoseBatchController(vendor, receiver, state, report)
        held = core.collect_stationary(receiver, state, report['trace'])
        require_no_drift(held, reference)
        controller.validate_pose_plan(plan, held)
        controller.enable()
        controller.observe(camera.start, timeout=30.)

        def capture(label):
            observation = controller.observe(lambda: camera.capture(label), timeout=10.)
            item = {'label': label, 'observation': observation, 'sample': controller.held}
            report['captures'].append(item)
            return item

        initial = capture('initial')
        # Reuse the existing scene-persistence check; it chooses no targets.
        from pick_resume_scene import verify_resume_scene
        report['launch_scene_check'] = controller.observe(
            lambda: verify_resume_scene(plan['source_report'], initial['observation']), timeout=20.)
        require_no_drift(controller.held, reference)
        controller.gripper(55000, 'open')
        write_report(path, report)
        controller.execute_pose_plan(plan)
        if report.get('protocol_completed'):
            capture('final')
            empty = report.get('grasp_outcome') == 'empty'
            report['status'] = 'sequence_completed_empty' if empty else 'sequence_completed_pending_visual_review'
            report['task_success'] = False if empty else 'pending_visual_review'
            result = 0
        else:
            result = 3
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='failed_or_stopped', error=str(exc), error_type=type(exc).__name__)
        if controller is not None:
            try:
                controller.fail(exc)
            except BaseException as monitor_error:
                report['failure_monitor_error'] = str(monitor_error)
        print('已停止后续下发：' + str(exc), file=sys.stderr, flush=True)
        result = 130 if isinstance(exc, KeyboardInterrupt) else 2
    finally:
        report['cleanup_errors'] = []
        if state is not None:
            try:
                report['final_diagnostic'] = state.diagnostic(time.monotonic())
            except BaseException as exc:
                report['cleanup_errors'].append({'operation': 'final_diagnostic', 'error': str(exc)})
        for name, resource in (('sdk', controller), ('receiver', receiver), ('camera', camera)):
            if resource is not None:
                try:
                    resource.close()
                except BaseException as exc:
                    report['cleanup_errors'].append({'operation': name + '_close', 'error': str(exc)})
        if report['cleanup_errors'] and result == 0:
            report['status'] += '_cleanup_error'
            result = 2
        report['finished_at_s'] = time.time()
        report['gripper_close_commands_attempted'] = sum(
            s.get('kind') == 'gripper' and s.get('label') == 'close' for s in report['stages'])
        write_report(path, report)
        print('本次记录：' + str(path), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
