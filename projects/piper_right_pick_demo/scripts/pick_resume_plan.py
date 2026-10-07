"""Offline continuation of an ungrasped, fully transmitted arrival-timeout plan.

The caller must acquire a fresh, continuously stationary SDK snapshot and verify
that the scene is unchanged. This module never connects to the robot, sends a
command, infers scene persistence, or repeats the interrupted target.
"""
import copy
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np

from pick_trajectory import (PlanRejected, LIMITS_DEG, _Builder, _six, _rotation,
                             _angle, _reserve, _sha, ARM_GEOMETRY, GRIPPER_HULL)

_ARRIVAL_ERROR = 'Waypoint arrival was not verified within five seconds'
_REQUIRED_AGES = ['0x%03X' % i for i in
                  (0x2A1, 0x2A2, 0x2A3, 0x2A4, 0x2A5, 0x2A6, 0x2A7,
                   0x261, 0x262, 0x263, 0x264, 0x265, 0x266)]


def _reject_unless(condition, message):
    if not condition:
        raise PlanRejected('Cannot resume: ' + message)


def _joint_frames(raw):
    _six(raw, 'source target joints')
    return [('0x151', '0101050000000000')] + [
        ('0x%03X' % (0x155 + i), struct.pack('>ii', *raw[i * 2:i * 2 + 2]).hex())
        for i in range(3)]


def _normal_live(sample):
    q, pose = _six(sample.get('joints_raw'), 'live joints'), _six(sample.get('pose_raw'), 'live pose')
    status = sample.get('status', {})
    _reject_unless(all(status.get(k) == v for k, v in dict(
        ctrl_mode=1, arm_status=0, mode_feed=1, teach_status=0,
        motion_status=0, err_code=0).items()), 'live arm is not stationary in normal SDK joint mode')
    _reject_unless(sample.get('motor_enabled') == [True] * 6 and not sample.get('fault'),
                   'live motor enable/fault state is invalid')
    ages = sample.get('frame_age_s', {})
    _reject_unless(all(isinstance(ages.get(k), (int, float)) and
                      math.isfinite(ages[k]) and 0 <= ages[k] < .1 for k in _REQUIRED_AGES),
                   'live feedback frames are missing or stale')
    _reject_unless(np.all(q >= LIMITS_DEG[:, 0]) and np.all(q <= LIMITS_DEG[:, 1]),
                   'live joints are outside nominal ranges')
    return q, pose


def _source_prefix(source):
    failure = source.get('failure', {})
    _reject_unless(source.get('status') == 'failed_or_stopped' and
                   source.get('error_type') == 'Rejected' and failure.get('type') == 'Rejected' and
                   source.get('error', '').startswith(_ARRIVAL_ERROR) and
                   failure.get('error', '').startswith(_ARRIVAL_ERROR), 'source is not an arrival timeout')
    _reject_unless(source.get('physical_grasp_attempts') == 0 and
                   not source.get('close_contact_candidate') and
                   not source.get('grasp_contact_classification') and
                   not source.get('protocol_completed') and not source.get('latched_fault'),
                   'source has already attempted a grasp or recorded a fault')
    _reject_unless(source.get('partial_motion_target') is False and source.get('plan_validated') is True,
                   'source contains a partial target or lacks a validated plan')
    plan = source.get('plan', {})
    _reject_unless(not plan.get('resume'), 'a resumed attempt cannot be resumed again')
    _reject_unless(plan.get('geometry') == source.get('geometry') and isinstance(plan.get('geometry'), dict),
                   'source geometry and plan disagree')
    stages, history = plan.get('stages', []), source.get('stages', [])
    _reject_unless(isinstance(stages, list) and isinstance(history, list) and len(history) >= 3,
                   'source stages are missing')
    _reject_unless([(s.get('kind'), s.get('label')) for s in history[:2]] ==
                   [('gripper', 'observe'), ('gripper', 'open')], 'source preparation sequence differs')
    for stage, width in zip(history[:2], (23000, 55000)):
        _reject_unless(stage.get('command_sent') is True and stage.get('feedback_verified') is True and
                       stage.get('width_raw') == width and stage.get('effort_raw') == 300,
                       'source preparation was not completely verified')
    executed = history[2:]
    _reject_unless(all(s.get('kind') == 'move' for s in executed),
                   'source already reached a grasp/capture stage or has unknown history')
    _reject_unless(len(executed) < len(stages), 'source has no remaining trajectory')
    for index, event in enumerate(executed):
        target = stages[index]
        _reject_unless(target.get('kind') == 'move' and event.get('label') == target.get('label') and
                       event.get('target_joints_raw') == target.get('joints_raw') and
                       event.get('target_pose_raw') == target.get('pose_raw'),
                       'executed targets do not match the original plan prefix')
        _reject_unless(event.get('command_sent') is True and event.get('target_frame_attempted') is True,
                       'source contains an incomplete target send')
        _reject_unless(event.get('arrival_verified') is (index < len(executed) - 1),
                       'only the last transmitted target may lack arrival verification')
    _reject_unless(failure.get('phase') == source.get('phase') == 'move_' + executed[-1]['label'],
                   'failure phase differs from the interrupted target')
    enable_count = source.get('enable_transmissions')
    _reject_unless(type(enable_count) is int and 1 <= enable_count <= 100,
                   'source enable transmission count is invalid')
    expected_tx = [('0x471', '0702000000000000')] * enable_count
    expected_tx += [('0x159', '000059d8012c0100'), ('0x159', '0000d6d8012c0100')]
    for event in executed:
        expected_tx += _joint_frames(event['target_joints_raw'])
    actual_tx = source.get('transmissions', [])
    _reject_unless(len(actual_tx) == len(expected_tx) and all(
        frame.get('send_result') == 'accepted_by_host_socket' and
        (frame.get('id'), frame.get('data_hex')) == expected
        for frame, expected in zip(actual_tx, expected_tx)),
        'source CAN history is partial, has extra commands, or disagrees with exact targets')
    _reject_unless(not any(c.get('label') in ('pregrasp', 'lifted', 'placed')
                           for c in source.get('captures', [])), 'source already entered grasp checkpoints')
    return plan, executed


def _remaining_events(stages):
    expected = [('capture', 'pregrasp'), ('gripper', 'close'), ('capture', 'lifted'),
                ('gripper', 'release'), ('capture', 'placed')]
    _reject_unless([(s.get('kind'), s.get('label')) for s in stages if s.get('kind') != 'move'] == expected,
                   'remaining plan is not the original single grasp/place sequence')
    for s in stages:
        if s.get('kind') == 'gripper':
            _reject_unless(s.get('width_raw') == (0 if s['label'] == 'close' else 55000) and
                           s.get('effort_raw') == 300, 'remaining gripper endpoint/effort changed')


def _reject_consumed_source(path, digest):
    # Reports are sibling run directories. Check the exact source identity, not
    # a run-name convention; even an unverified/partial close consumes it.
    candidates = set(path.parent.glob('*.json')) | set(path.parent.parent.glob('*/report.json'))
    for candidate in candidates:
        if candidate.resolve() == path:
            continue
        try:
            previous = json.loads(candidate.read_text())
        except (OSError, ValueError):
            continue
        if not isinstance(previous, dict):
            continue
        metadata = previous.get('plan', {}).get('resume', {})
        ref = previous.get('resume_source') or metadata.get('source_report_path')
        same_source = (isinstance(ref, str) and Path(ref).resolve() == path or
                       metadata.get('source_report_sha256') == digest)
        if same_source:
            _reject_unless(not previous.get('physical_grasp_attempts') and not any(
                stage.get('kind') == 'gripper' and stage.get('label') in ('close', 'release')
                for stage in previous.get('stages', [])),
                'source already consumed by a close/release in ' + str(candidate))


def resume_plan(source_report_path, live_sample):
    """Return only the untouched suffix after a now-arrived timeout target.

    ``live_sample`` uses the fresh SDK snapshot schema. The caller must check a
    >=0.2 s stable window and scene persistence before calling, and maintain
    fresh held-pose monitoring during this offline calculation. No old live
    samples or the source report's final diagnostic authorize a real movement.
    """
    path = Path(source_report_path).resolve()
    payload = path.read_bytes()
    try:
        digest = hashlib.sha256(payload).hexdigest()
        _reject_consumed_source(path, digest)
        source = json.loads(payload)
        plan, executed = _source_prefix(source)
        q, pose = _normal_live(live_sample)
        old_target = executed[-1]
        target_q = _six(old_target['target_joints_raw'], 'interrupted target joints')
        target_pose = _six(old_target['target_pose_raw'], 'interrupted target pose')
        residual = dict(position_mm=float(np.linalg.norm(pose[:3] - target_pose[:3])),
                        orientation_deg=_angle(_rotation(pose), _rotation(target_pose)),
                        max_joint_deg=float(np.max(np.abs(q - target_q))))
        _reject_unless(residual['position_mm'] <= .5 and residual['orientation_deg'] <= .25 and
                       residual['max_joint_deg'] <= .3, 'live arm has not arrived at the interrupted target')
        geometry = copy.deepcopy(plan['geometry'])
        builder = _Builder(q, pose, geometry)
        fk = builder.fk(q)
        _reject_unless(np.linalg.norm(fk[:3] - pose[:3]) <= .5 and
                       _angle(_rotation(fk), _rotation(pose)) <= .1, 'live feedback disagrees with official FK')
        limits = plan.get('limits', {})
        for key, value in dict(speed_percent=5, tracking_allowance_deg=.5,
                               minimum_table_clearance_mm=10., max_joint_segment_deg=7.5,
                               max_flange_segment_mm=14., max_orientation_segment_deg=4.5).items():
            _reject_unless(limits.get(key) == value, 'source planning limits changed: ' + key)
        _reject_unless(limits.get('uncertainty_reserve_mm') == _reserve(geometry),
                       'source uncertainty reserve changed')
        hashes = plan.get('source_sha', {}).get('manufacturer_geometry', {})
        for asset in (ARM_GEOMETRY, GRIPPER_HULL):
            matches = [v for k, v in hashes.items() if Path(k).name == asset.name]
            _reject_unless(matches == [_sha(asset)], 'manufacturer collision asset changed: ' + asset.name)
        remaining = copy.deepcopy(plan['stages'][len(executed):])
        _remaining_events(remaining)
        count = sum(s.get('kind') == 'move' for s in remaining)
        _reject_unless(1 <= count <= 120 and len(remaining) <= 140, 'remaining motion budget invalid')
        phases = []
        for stage in remaining:
            if stage['kind'] != 'move':
                continue
            target = _six(stage.get('joints_raw'), 'remaining target joints')
            p = _six(stage.get('pose_raw'), 'remaining target pose')
            _reject_unless(np.all(target >= LIMITS_DEG[:, 0]) and np.all(target <= LIMITS_DEG[:, 1]),
                           'remaining target is outside nominal ranges')
            computed = builder.fk(target)
            _reject_unless(np.linalg.norm(computed[:3] - p[:3]) <= .3 and
                           _angle(_rotation(computed), _rotation(p)) <= .1,
                           'remaining target disagrees with official FK')
            _reject_unless(np.linalg.norm(p[:3] - builder.pose[:3]) <= 14.01 and
                           _angle(_rotation(p), _rotation(builder.pose)) <= 4.51 and
                           np.max(np.abs(target - builder.q)) <= 7.501,
                           'remaining segment exceeds original planning bounds')
            _reject_unless(stage.get('speed_percent') == 5, 'remaining speed changed')
            stage['clearance_mm'], stage['clearance_detail'] = builder.segment_clearance(target)
            builder.q, builder.pose = target, p
            if not phases or phases[-1]['label'] != stage['label']:
                phases.append(dict(label=stage['label'], moves=0, minimum_clearance_mm=float('inf')))
            phases[-1]['moves'] += 1
            phases[-1]['minimum_clearance_mm'] = min(phases[-1]['minimum_clearance_mm'], stage['clearance_mm'])
            phases[-1]['final_pose_raw'], phases[-1]['final_joints_raw'] = stage['pose_raw'], stage['joints_raw']
        result = copy.deepcopy(plan)
        result.update(start_joints_raw=list(live_sample['joints_raw']),
                      start_pose_raw=list(live_sample['pose_raw']), stages=remaining, geometry=geometry)
        result['assessment'].update(move_count=count, checked_states=builder.checked_states,
                                    minimum_sampled_clearance_mm=builder.min_clearance, phase_summary=phases)
        result['resume'] = dict(source_report_path=str(path), source_report_sha256=digest,
                              skipped_original_move_count=len(executed),
                              skipped_verified_move_count=len(executed) - 1,
                              interrupted_move_number=len(executed),
                              interrupted_target_reissued=False,
                              interrupted_target_joints_raw=old_target['target_joints_raw'],
                              interrupted_target_pose_raw=old_target['target_pose_raw'],
                              live_arrival_residuals=residual, remaining_move_count=count,
                              remaining_capture_count=3, remaining_gripper_commands=2,
                              source_geometry_reused=True,
                              caller_must_verify_scene_unchanged=True,
                              caller_must_verify_continuously_stationary_live_feedback=True)
        result['source_sha']['resume_planner'] = _sha(__file__)
        result['source_sha']['resume_source_report'] = result['resume']['source_report_sha256']
        return result
    except (KeyError, TypeError, IndexError, json.JSONDecodeError) as exc:
        raise PlanRejected('Cannot resume: malformed source report or live sample: ' + str(exc)) from exc
