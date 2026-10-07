#!/usr/bin/env python3
"""Concurrent receive-only master targets and right-arm actual joint evidence."""
import argparse
import json
import math
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from passive_can_snapshot import collect

ROOT = Path(__file__).resolve().parents[1]


def compare(master, right):
    result = {"comparison_available": False, "motion_authorized": False,
              "follow_ready": False, "master_motion_response_verified": False,
              "note": "Receipt-time comparison only; no follow command or trajectory safety validation."}
    try:
        required_master = ["PiperMsgJointCtrl_" + pair for pair in ("12", "34", "56")]
        missing = [key for key in required_master if not master.get("command_feedback", {}).get(
            key, {}).get("latest_by_origin", {}).get("nonlocal")]
        result["missing_master_joint_messages"] = missing
        if missing:
            raise ValueError("主臂未收到完整的非本机关节目标帧：" + ", ".join(missing))
        master_values, right_values, times = {}, {}, []
        for pair in ("12", "34", "56"):
            m = master["command_feedback"]["PiperMsgJointCtrl_" + pair]["latest_by_origin"]["nonlocal"]
            r = right["feedback"]["PiperMsgJointFeedBack_" + pair]
            for item in (m, r):
                age = item["age_s_at_finish"]
                if not math.isfinite(age) or not 0 <= age <= .1:
                    raise ValueError("Joint input missing or older than 100 ms at collection end")
                if item.get("origin") != "nonlocal":
                    raise ValueError("Joint source is not a nonlocal CAN frame")
                stamp = float(item["received_monotonic_s"])
                if not math.isfinite(stamp):
                    raise ValueError("Invalid receipt timestamp")
                times.append(stamp)
            for name in ("joint_" + n for n in pair):
                master_values[name] = m["fields"][name] * .001
                right_values[name] = r["fields"][name] * .001
        if max(times) - min(times) > .1:
            raise ValueError("Master and right feedback have excessive receipt-time skew")
        m = [master_values["joint_%d" % n] for n in range(1, 7)]
        r = [right_values["joint_%d" % n] for n in range(1, 7)]
        if not all(math.isfinite(v) for v in m + r):
            raise ValueError("Non-finite joint values")
        delta = [a - b for a, b in zip(m, r)]
        result.update(comparison_available=True, master_target_deg=m,
                      right_actual_deg=r, master_minus_right_deg=delta,
                      max_absolute_difference_deg=max(abs(x) for x in delta),
                      receipt_skew_s=max(times) - min(times))
    except (KeyError, TypeError, ValueError) as exc:
        result["reason"] = str(exc)
    # Raw driver positions: no assumption that this is calibrated total opening.
    result["master_gripper_command"] = master.get("command_feedback", {}).get(
        "PiperMsgGripperCtrl", {}).get("latest_by_origin", {}).get("nonlocal")
    result["right_gripper_feedback"] = right.get("feedback", {}).get("PiperMsgGripperFeedBack")
    result["right_status"] = right.get("feedback", {}).get("PiperMsgStatusFeedback")
    result["right_motor_feedback"] = {str(n): right.get("feedback", {}).get(
        "PiperMsgLowSpdFeed_%d" % n) for n in range(1, 7)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-master-motion', action='store_true',
                        help='12-second passive recording with prompts for human master movement; never drives an arm')
    args = parser.parse_args()
    config = json.loads((ROOT / 'configs/site.local.json').read_text())['host_capture']
    bindings = {config['master_can_interface']: config['master_usb_port'],
                config['right_can_interface']: config['right_usb_port']}
    if set(bindings) != {'can1', 'can2'}:
        raise RuntimeError('Expected inspected can1/master and can2/right bindings')
    for channel, usb in bindings.items():
        device = Path('/sys/class/net') / channel / 'device'
        if not device.exists() or device.resolve().name != usb:
            raise RuntimeError('CAN USB identity mismatch: ' + channel)
    run = ROOT / 'runs' / ('master_pair_' + uuid.uuid4().hex)
    run.mkdir(parents=True, exist_ok=False)
    report = {"operation": "passive_master_right_comparison", "started_at_s": time.time(),
              "status": "collecting", "can_frames_sent": 0, "motion_authorized": False,
              "controller_started": False, "can_bindings": bindings,
              "physical_grasp_attempts": 0,
              "interface_activation": "Parent shell may activate master can1 only; this Python process only receives."}
    report['human_motion_observation_requested'] = args.observe_master_motion
    report['operator_compliance_verified'] = False
    report['operator_prompt_times'] = []
    try:
        gateways = subprocess.run(['cangw', '-L'], capture_output=True, text=True, timeout=2)
        report['kernel_gateway_readonly_listing'] = {
            'exit_code': gateways.returncode, 'stdout': gateways.stdout,
            'stderr': gateways.stderr, 'command': ['cangw', '-L']}
    except (OSError, subprocess.TimeoutExpired) as exc:
        report['kernel_gateway_readonly_listing'] = {'error': str(exc)}
    path = run / 'report.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    snapshots = {}
    seconds = 12.0 if args.observe_master_motion else 3.0
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = {name: pool.submit(collect, channel, seconds,
                                    include_pose_trace=(name == 'right' and args.observe_master_motion))
                   for name, channel in (('master', 'can1'), ('right', 'can2'))}
        if args.observe_master_motion:
            print('开始记录：前 3 秒保持主臂和右臂不动。', flush=True)
            report['operator_prompt_times'].append({'phase': 'stationary_before', 'monotonic_s': time.monotonic()})
            time.sleep(3)
            print('现在仅轻微摆动可自由拖动的主臂，并轻动主臂夹爪手柄，持续约 5 秒；右臂若意外动作立即停止拖动。', flush=True)
            report['operator_prompt_times'].append({'phase': 'human_master_movement', 'monotonic_s': time.monotonic()})
            time.sleep(5)
            print('现在停止拖动，保持两臂不动，等待保存。', flush=True)
            report['operator_prompt_times'].append({'phase': 'stationary_after', 'monotonic_s': time.monotonic()})
        for name, future in futures.items():
            try:
                snapshots[name] = future.result()
            except Exception as exc:
                snapshots[name] = {"error": type(exc).__name__, "reason": str(exc),
                                   "frames_sent_by_this_script": 0}
            (run / (name + '.json')).write_text(json.dumps(
                snapshots[name], ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    report['comparison'] = compare(snapshots['master'], snapshots['right'])
    if args.observe_master_motion:
        report['master_nonlocal_observed_ranges'] = {
            key: {'count': item['latest_by_origin']['nonlocal'].get('observed_count'),
                  'field_min': item['latest_by_origin']['nonlocal'].get('field_min'),
                  'field_max': item['latest_by_origin']['nonlocal'].get('field_max')}
            for key, item in snapshots['master'].get('command_feedback', {}).items()
            if 'nonlocal' in item.get('latest_by_origin', {})}
        report['master_frame_counts'] = snapshots['master'].get('frame_id_counts', {})
        report['motion_observation_note'] = 'Observed command variation is evidence only; no controller or automatic follow authorization is present.'
    report['status'] = ('comparison_saved' if report['comparison']['comparison_available']
                        else 'incomplete_input_saved')
    report['finished_at_s'] = time.time()
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print('诊断已保存：' + str(path), flush=True)
    c = report['comparison']
    if args.observe_master_motion:
        print('主臂实际收到的帧：', report['master_frame_counts'])
        print('动态记录已保存；即使静止后不再输出导致角度比较不可用，原始报文及目标变化范围也会保留。')
    if c['comparison_available']:
        print('主臂目标角（度）：', c['master_target_deg'])
        print('右臂实际角（度）：', c['right_actual_deg'])
        print('最大关节差（度）：', c['max_absolute_difference_deg'])
    else:
        print('尚不能比较：' + c.get('reason', 'unknown'))
    print('没有启动桥接、使能/失能、闭爪或发送运动指令。')
    return 0 if c['comparison_available'] else 2


if __name__ == '__main__':
    sys.exit(main())
