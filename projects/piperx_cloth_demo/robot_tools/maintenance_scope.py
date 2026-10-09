"""Confine a terminal, fully sent jaw-coordinate maintenance failure to its arm.

This is not a successful calibration or a fault reset. Only the existing
single-arm executors, whose peer TX is blocked, may use the other arm. Unknown
sends, task faults, active owners and subsequent failed actions still block.
"""
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

from .reboot_startup import boot_identity

PREFIXES = {'single_gripper_prepare', 'single_supervised_gripper', 'single_supervised_move'}
TOOLS = {'robot_prepare_gripper', 'robot_single_arm_gripper_once', 'robot_single_arm_move_once'}
TABLE = 'pair_single_arm_maintenance_actions'


def _need(value, detail):
    if not value:
        raise RuntimeError('Single-arm maintenance scope refused: ' + detail)


def _review_model_refusal(runs, action, compatibility):
    """Read a specific zero-TX model rejection; keep its failed row unchanged."""
    review = (compatibility or {}).get('reviewed_start_reference_refusal', {})
    reference_refusal = action['run_id'] == review.get('run_id')
    age_review = (compatibility or {}).get('reviewed_prediction_age_refusal', {})
    prediction_refusal = action['run_id'] == age_review.get('run_id')
    live_review = (compatibility or {}).get('reviewed_live_start_refusal', {})
    live_refusal = action['run_id'] == live_review.get('run_id')
    if prediction_refusal:
        review=age_review
    expected_id = review.get('run_id') if reference_refusal else (compatibility or {}).get('refused_run_id')
    expected_hash = review.get('sha256') if reference_refusal else (compatibility or {}).get('refused_sha256')
    if prediction_refusal:
        expected_id,expected_hash=review['run_id'],review['sha256']
    if live_refusal:
        expected_id,expected_hash=live_review['run_id'],live_review['sha256']
    expected_error = ('Legacy compatibility permits only <=1 mm +Z with unchanged controller orientation'
                      if reference_refusal else 'Manufacturer FK does not agree with current flange feedback')
    _need(compatibility is not None and action['status'] == 'failed'
          and action['run_id'] == expected_id and action['result_sha256'] == expected_hash,
          'prior single-arm action failed or pending')
    path = Path(action['record_path']).resolve()
    _need(path == runs/action['run_id']/'result.json', 'model refusal path differs')
    raw = path.read_bytes(); result = json.loads(raw)
    if live_refusal:
        from .legacy_controller import same_authorization
        expected_error='Legacy +Z target exceeds selected step or existing 0.5 mm / 0.003 rad start-reference bands'
        _need(same_authorization(result.get('legacy_controller_compatibility'),compatibility)
              and result['legacy_controller_compatibility'].get('continuation_window')==compatibility.get('continuation_window'),
              'live reference refusal belongs to a different window')
        start=result['before']['left']['pose_m_rad'];target=result['requested_target']
        _need(.0045<=target[2]-start[2]<=.0055 and math.dist(start[:2],target[:2])<=.0005
              and sum(abs(math.remainder(a-b,2*math.pi)) for a,b in zip(start[3:],target[3:]))<=.01,
              'old target is not the bounded five-mm reference mismatch')
        for side in ('left','right'):
            a,b=result['before'][side],result['after'][side]
            _need(max(abs(x-y) for x,y in zip(a['joints_rad'],b['joints_rad']))<=.003
                  and math.dist(a['pose_m_rad'][:3],b['pose_m_rad'][:3])<=.0005
                  and abs(a['gripper']['width_m']-b['gripper']['width_m'])<=.0005
                  and b['arm_status']['motion_status']==0,'live reference refusal was not stationary')
    if prediction_refusal:
        import re
        from .legacy_controller import same_authorization
        errors=result.get('errors',[])
        _need(len(errors)==1 and errors[0].get('type')=='RuntimeError', 'one prediction-age error required')
        expected_error=errors[0].get('detail','')
        match=re.fullmatch(r'Single action requires receive age/skew within 100 ms including processing; age=([0-9.]+) skew=([0-9.]+)',expected_error)
        _need(match is not None and .1<float(match[1])<1 and 0<=float(match[2])<=.1
              and isinstance(result.get('pre_send_endpoint_prediction'),dict)
              and same_authorization(result.get('legacy_controller_compatibility'),compatibility)
              and result['legacy_controller_compatibility'].get('continuation_window')==compatibility.get('continuation_window')
              and result.get('verified_wrist_recovery',{}).get('original_failure_run_id')==compatibility.get('reviewed_wrist_limit_failure',{}).get('run_id'),
              'not the reviewed prediction processing delay in this window')
        for side in ('left','right'):
            before,after=result['before'][side],result['after'][side]
            _need(max(abs(a-b) for a,b in zip(before['joints_rad'],after['joints_rad']))<=.003
                  and math.dist(before['pose_m_rad'][:3],after['pose_m_rad'][:3])<=.0005
                  and abs(before['gripper']['width_m']-after['gripper']['width_m'])<=.0005
                  and after['arm_status']['motion_status']==0,'prediction refusal has changed physical feedback')
    _need(hashlib.sha256(raw).hexdigest() == action['result_sha256']
          and result.get('run_id') == action['run_id']
          and result.get('operation') == 'single_supervised_move'
          and result.get('selected_arm') == compatibility['arm']
          and result.get('ok') is False and result.get('status') == 'refused_before_send'
          and result.get('errors') == [dict(type='RuntimeError', detail=expected_error)]
          and result.get('guard_violations') == []
          and result.get('transmission_counts') == {
              s:dict(attempted_frames=0, sent_frames=0, blocked_frames=0) for s in ('left','right')}
          and all(result.get(k) == 0 for k in ('hardware_commands_sent','target_calls_sent',
              'target_commands_sent','enable_commands_sent','stop_commands_sent','retries')),
          'model refusal is not exact, complete zero-TX evidence')
    if reference_refusal:
        from .legacy_controller import check_up_target, same_authorization
        _need(same_authorization(result.get('legacy_controller_compatibility'), compatibility),
              'start-reference refusal came from another authorization')
        def angle(a,b):
            # Conservative SO(3) triangle bound; this read-only ledger review
            # must not load a device driver before service admission.
            return sum(abs((x-y+math.pi)%(2*math.pi)-math.pi) for x,y in zip(a[3:],b[3:]))
        check_up_target(result['before'][compatibility['arm']]['pose_m_rad'], result['requested_target'], angle)
    _need(all(result.get('cleanup', {}).get('arms', {}).get(s, {}).get('status') == 'disconnected'
              for s in ('left','right')), 'model refusal cleanup incomplete')
    events = [json.loads(x) for x in path.with_name('events.jsonl').read_text().splitlines()]
    _need(not any(e['event'] in ('single_supervised_action_sent_unconfirmed',
        'probe_send_complete_unconfirmed','can_send_attempt','can_send_returned') for e in events),
        'model refusal journal contains a send')


def inspect(runs, arm, compatibility=None):
    """Read-only admission; called again under the execution lock before claim."""
    runs = Path(runs).resolve()
    _need(arm in ('left', 'right'), 'explicit working arm required')
    with sqlite3.connect((runs/'pair_sessions.sqlite').as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute('BEGIN')
        names = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        _need('pair_gripper_zero_calibrations' in names, 'fault is not isolated jaw maintenance')
        scope = db.execute('SELECT * FROM pair_scope WHERE id=1').fetchone()
        _need(scope and scope['owner'] is None and scope['active_run_id'] is None,
              'active task or owner remains')
        _need(db.execute('SELECT COUNT(*) FROM pair_runs').fetchone()[0] == 0
              and db.execute('SELECT COUNT(*) FROM pair_events').fetchone()[0] == 0,
              'task history requires its own continuation')
        rows = list(db.execute('SELECT * FROM pair_gripper_zero_calibrations'))
        _need(len(rows) == 1, 'ambiguous maintenance history')
        row = dict(rows[0]); peer = row['arm']
        _need(peer in ('left', 'right') and peer != arm and row['status'] == 'failed'
              and row['boot_id'] == boot_identity()['boot_id'], 'selected arm, pending or foreign-boot maintenance')
        faults = [dict(f) for f in db.execute('SELECT * FROM pair_faults')]
        relevant = [f for f in faults if f['run_id'] == row['run_id']]
        _need(relevant and scope['fault_id'] == row['fault_id']
              and any(f['id'] == row['fault_id'] for f in relevant)
              and all(f['owner'] is None and f['reason'] in
                  ('gripper_zero_in_progress', 'gripper_zero_calibration_failed_or_uncertain') for f in relevant)
              and all(f in relevant or (f['reason'] == 'software_reset_requires_new_startup'
                  and f['owner'] is None and f['id'] < row['fault_id']) for f in faults),
              'unrelated or newer fault remains')
        path = Path(row['record_path']).resolve()
        _need(path == runs/row['run_id']/'result.json', 'receipt path differs')
        raw = path.read_bytes(); result = json.loads(raw)
        request = json.loads(path.with_name('request.json').read_text())
        _need(request.get('run_id') == row['run_id'] and request.get('arm') == peer
              and request.get('boot_id') == row['boot_id'], 'maintenance request identity differs')
        _need(hashlib.sha256(raw).hexdigest() == row['result_sha256'], 'receipt hash differs')
        expected = {s:dict(attempted_frames=int(s == peer), sent_frames=int(s == peer), blocked_frames=0)
                    for s in ('left', 'right')}
        _need(result.get('run_id') == row['run_id'] and result.get('arm') == peer
              and result.get('operation') == 'calibrate_empty_gripper_zero'
              and result.get('status') == 'aborted_after_dispatch' and result.get('ok') is False
              and result.get('transmission_counts') == expected
              and result.get('hardware_commands_sent') == 1
              and result.get('guard_violations') == []
              and all(result.get(k) == 0 for k in ('target_commands_sent', 'enable_commands_sent',
                                                  'stop_commands_sent', 'retries')),
              'send scope is unknown, partial, or includes actuation')
        errors = result.get('errors', [])
        _need(len(errors) == 1 and errors[0]['type'] == 'RuntimeError'
              and (errors[0]['detail'] == 'Manufacturer zero ACK missing; no retry'
                   or errors[0]['detail'].startswith('Calibration not acknowledged:')
                   or errors[0]['detail'] == 'Fresh zero feedback not observed after acknowledged calibration'),
              'failure is not solely calibration confirmation')
        for side in ('left', 'right'):
            _need(result.get('cleanup', {}).get('arms', {}).get(side, {}).get('status') == 'disconnected',
                  'original controller cleanup incomplete')
        _need(all(result[phase][peer]['gripper']['foc_status']['driver_enable_status'] is False
                  for phase in ('before', 'after')), 'maintenance jaw was enabled')
        journal = [json.loads(line) for line in path.with_name('events.jsonl').read_text().splitlines()]
        intent = [e for e in journal if e['event'] == 'gripper_zero_intent']
        returned = [e for e in journal if e['event'] == 'gripper_zero_returned']
        _need(len(intent) == len(returned) == 1 and intent[0]['arm'] == returned[0]['arm'] == peer
              and intent[0]['arbitration_id'] == 0x159 and intent[0]['data_hex'] == '00000000000000ae'
              and returned[0]['transmission_counts'] == expected
              and intent[0]['unix_s'] <= returned[0]['unix_s'], 'sole coordinate command not accounted for')
        budget_receipt = None
        wrist_review = None
        if TABLE in names:
            from .legacy_controller import task_budget
            budget = task_budget(compatibility)
            compatibility_steps = 0
            task_steps = 0
            reviewed = False
            for action in db.execute('SELECT * FROM '+TABLE):
                task_steps += 1
                if budget is not None:
                    _need(action['started_at'] >= budget['started_at_unix_s'],
                          'original task budget starts after recorded task activity')
                _need(action['arm'] == arm and action['source_sha256'] == row['result_sha256'],
                      'prior action arm or original source differs')
                if action['status'] != 'complete':
                    from .legacy_wrist_review import KEY, review as review_wrist
                    if action['run_id'] == (compatibility or {}).get(KEY,{}).get('run_id'):
                        wrist_review = review_wrist(runs, action, compatibility)
                        continue
                    _review_model_refusal(runs, action, compatibility)
                    reviewed = True
                    continue
                raw_action = Path(action['record_path']).read_bytes()
                action_result = json.loads(raw_action)
                _need(hashlib.sha256(raw_action).hexdigest() == action['result_sha256']
                      and action_result.get('ok') is True, 'prior action receipt differs')
                if action_result.get('legacy_controller_compatibility') is not None:
                    from .legacy_controller import same_authorization
                    prior_grant = action_result['legacy_controller_compatibility']
                    _need(same_authorization(prior_grant, compatibility),
                          'legacy compatibility changed within current sequence')
                    if prior_grant.get('task_budget') is not None:
                        _need(prior_grant['task_budget'] == budget,
                              'frozen original task budget cannot be changed or removed')
                    if prior_grant.get('continuation_window') is not None:
                        _need(prior_grant['continuation_window']==compatibility.get('continuation_window'),
                              'frozen continuation window cannot be changed or removed')
                    compatibility_steps += 1
            if compatibility is not None:
                if compatibility.get('reviewed_wrist_limit_failure') is not None:
                    _need(wrist_review is not None, 'specific wrist failure was not reviewed')
                from .legacy_controller import MAX_STEPS, effective_deadline
                used = task_steps if budget is not None else compatibility_steps
                maximum = budget['max_steps'] if budget is not None else MAX_STEPS
                _need(reviewed and used < maximum,
                      'missing original model refusal or legacy step budget exhausted')
                budget_receipt = dict(used_steps=used, max_steps=maximum,
                    successful_legacy_moves=compatibility_steps,
                    count_basis='all existing single-arm action rows' if budget else 'legacy moves',
                    original_task_budget=budget, expires_at_unix_s=effective_deadline(compatibility),
                    original_expires_at_unix_s=compatibility['expires_at_unix_s'],
                    continuation_window=compatibility.get('continuation_window'))
        return dict(selected_arm=arm, excluded_arm=peer, fault_id=row['fault_id'],
                    source_run_id=row['run_id'], source_sha256=row['result_sha256'],
                    feedback_observation=request.get('feedback_observation'),
                    task_budget_receipt=budget_receipt,
                    verified_wrist_recovery=wrist_review,
                    calibration_required=False, calibration_resolved=False,
                    excluded_arm_must_remain_disabled=True, original_fault_preserved=True)


def claim(runs, run_id, arm, prefix, record_path, compatibility=None):
    _need(prefix in PREFIXES, 'executor lacks a passive-arm TX block')
    _need(compatibility is None or prefix == 'single_supervised_move', 'model continuation is MOVE_L only')
    evidence = inspect(runs, arm, compatibility)
    if compatibility is not None:
        evidence['reviewed_zero_tx_model_refusal'] = dict(compatibility)
    with sqlite3.connect(Path(runs)/'pair_sessions.sqlite') as db:
        db.execute('PRAGMA synchronous=FULL'); db.execute('BEGIN IMMEDIATE')
        scope = db.execute('SELECT owner,active_run_id,fault_id FROM pair_scope WHERE id=1').fetchone()
        _need(scope == (None, None, evidence['fault_id']), 'platform changed before claim')
        db.execute('CREATE TABLE IF NOT EXISTS '+TABLE+' (run_id TEXT PRIMARY KEY, arm TEXT, '
                   'source_sha256 TEXT, status TEXT, record_path TEXT, result_sha256 TEXT, started_at REAL)')
        allowed = compatibility['refused_run_id'] if compatibility is not None else ''
        reference_allowed = (compatibility or {}).get('reviewed_start_reference_refusal', {}).get('run_id','')
        wrist_allowed = (compatibility or {}).get('reviewed_wrist_limit_failure', {}).get('run_id','')
        age_allowed = (compatibility or {}).get('reviewed_prediction_age_refusal', {}).get('run_id','')
        live_allowed = (compatibility or {}).get('reviewed_live_start_refusal', {}).get('run_id','')
        _need(not db.execute("SELECT 1 FROM "+TABLE+" WHERE status!='complete' AND run_id NOT IN (?,?,?,?,?)", (allowed,reference_allowed,wrist_allowed,age_allowed,live_allowed)).fetchone(),
              'previous single-arm action not complete')
        db.execute('INSERT INTO '+TABLE+' VALUES(?,?,?,?,?,NULL,?)',
                   (run_id, arm, evidence['source_sha256'], 'pending', str(record_path), time.time()))
    return evidence


def finish(runs, run_id, result, record_path):
    # A crash, exception or failed action never becomes permission to retry.
    with sqlite3.connect(Path(runs)/'pair_sessions.sqlite') as db:
        db.execute('PRAGMA synchronous=FULL')
        db.execute('UPDATE '+TABLE+" SET status=?,result_sha256=? WHERE run_id=? AND status='pending'",
                   ('complete' if result.get('ok') is True else 'failed',
                    hashlib.sha256(Path(record_path).read_bytes()).hexdigest(), run_id))
