"""Explicit round enrollment after audited closure; never hardware recovery.

Trusted administrative API, never a robot dispatch tool. prepare_round is
read-only; activate_round appends a new run/scope under a matching user-message
authorization. started_at is frozen before preparation, so repair time consumes
the new budget. Old rows, faults, targets and source evidence are never changed.

Operator API (Python 3.10, from this project): call prepare_round with the
authoritative runs/pair_sessions.sqlite, parent/new run IDs, final close log,
fixed started_at and budget. Then activate_round(proposal, authorization,
project_root=...). Authorization has source='user_message', message_id, actual
statement/received_at, decision='authorize_explicit_new_round', proposal_sha256
and the exact proposal.new_budget. Only a new explicit user instruction to
start timing after repair permits budget_start_policy=
'after_repair_before_online_execution' in BOTH proposal and authorization;
the operator chooses started_at after repair and before opening the new host.
Activation never moves that timestamp. Neither API opens devices or grants
cached targets, source records, readiness, task success or physical stopping.

Diagnosed fault entries retain old fault rows. A zero-TX freshness continuation
preserves the old deadline; post-send RGB expiry requires an expired parent and
separate explicit new-round authorization, plus archived post-failure evidence.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import _execution_scope, _hold_frames, _identifier, _json_object, _number
from .pair_restart import _current_contract as _restart_current_contract
from .pair_restart import _file, _live_control_processes, _need, _sha


TABLE = "pair_rounds"


def revise_unopened_round(path, run_id, *, expected_contract_sha256, project_root,
                          reason, clock=time.time):
    """Append one reviewed code revision to an unopened, enrolled round.

    This administrative repair does not create a run, change its original
    contract/budget rows, or restore an execution owner. Historical evidence
    is rechecked, and a subsequent host must open in preparation mode and
    acquire new feedback/RGB before any physical operation.
    """
    from .pair_ledger import effective_contract_json, UNOPENED_REPAIR_SOURCES
    root = Path(project_root).resolve(strict=True)
    path = Path(path).resolve(strict=True)
    run_id = _identifier(run_id, 'run_id')
    _need(path == root / 'runs/pair_sessions.sqlite', 'Authoritative project database required')
    _need(type(reason) is str and 1 <= len(reason.strip()) <= 2000, 'Actual repair reason required')
    _need(not _live_control_processes(), 'Live control host blocks unopened revision')
    with ExclusiveExecution(path.parent):
        db = sqlite3.connect(path.as_uri()+'?mode=rw', uri=True, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            db.execute('PRAGMA synchronous=FULL')
            db.execute('BEGIN IMMEDIATE')
            table, _, _, scope = _execution_scope(db, run_id, writable=True)
            run = db.execute('SELECT * FROM pair_runs WHERE run_id=?', (run_id,)).fetchone()
            _need(table == 'pair_rounds' and run is not None and scope['run_id'] == run_id
                  and scope['owner'] is None and scope['active_run_id'] is None
                  and scope['fault_id'] is None and run['steps'] == 0,
                  'Only the clean never-opened current round may receive this revision')
            registration = json.loads(scope['record_json'])
            _need(scope['last_time'] == registration['activated_at'],
                  'A round observed or claimed after registration is not unopened')
            for name in ('pair_events', 'pair_faults', 'pair_joint_sends', 'pair_hold_requests',
                         'pair_holds', 'pair_hold_frames', 'pair_grasp_episodes'):
                _need(not db.execute('SELECT 1 FROM '+name+' WHERE run_id=?', (run_id,)).fetchone(),
                      'Current-round activity blocks unopened revision: '+name)
            _need(not db.execute("SELECT 1 FROM pair_events WHERE status='pending'").fetchone(),
                  'Any pending send blocks revision')
            exists = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_unopened_round_revisions'").fetchone()
            _need(not exists or not db.execute('SELECT 1 FROM pair_unopened_round_revisions WHERE run_id=?',
                                               (run_id,)).fetchone(), 'Unopened revision already used')
            old = json.loads(run['contract_json'])
            _need(hashlib.sha256(run['contract_json'].encode()).hexdigest() == expected_contract_sha256
                  and old == registration['new_contract'] == registration['proposal']['reviewed_contract'],
                  'Original frozen contract differs from the reviewed repair input')
            now = _number(clock(), 'clock')
            deadline = run['started_at'] + run['max_duration']
            _need(scope['last_time'] <= now < deadline, 'Original deadline or clock blocks revision')
            audit = audit_unopened_enrollment(db, scope, run)
            new = _current_contract(root, old)
            _need(set(new['code']) == set(old['code'])
                  and {name for name in old['code'] if old['code'][name] != new['code'][name]}
                      == UNOPENED_REPAIR_SOURCES,
                  'Only the reviewed round, ledger and preparation routing code may change')
            revision = dict(schema='piper_unopened_round_code_repair_v2', project_root=str(root), reason=reason,
                old_run=dict(run), old_contract=old, new_contract=new,
                original_round_record_sha256=_sha(registration), historical_audit=audit,
                owner_at_revision=None, events_at_revision=[], hardware_commands_sent=0,
                new_budget_allocated=False, required_connection_mode='prepare', deadline_s=deadline,
                physical_stop_verified=None)
            _need(_current_contract(root, old) == new and not _live_control_processes(),
                  'Code or control owner changed during revision')
            final = _number(clock(), 'clock')
            _need(now <= final < deadline, 'Original deadline reached or clock regressed')
            revision['revised_at'] = final
            db.execute('CREATE TABLE IF NOT EXISTS pair_unopened_round_revisions '
                       '(run_id TEXT PRIMARY KEY, at REAL NOT NULL, record_json TEXT NOT NULL)')
            db.execute('INSERT INTO pair_unopened_round_revisions VALUES(?,?,?)',
                       (run_id, final, _json_object(revision, 'unopened revision')))
            _need(effective_contract_json(db, run, scope) == _json_object(new, 'new contract'),
                  'Effective contract reader did not recognize the appended revision')
            db.execute('COMMIT')
            return revision
        except BaseException:
            if db.in_transaction:
                db.execute('ROLLBACK')
            raise
        finally:
            db.close()


def _current_contract(root, old):
    contract = _restart_current_contract(root, old, require_change=False)
    _need("pair_round.py" in contract["code"], "New-round manager must be in the host source contract")
    return contract


def _counts(value):
    _need(type(value) is dict and set(value) == {"left", "right"}, "Exact pair counters required")
    for row in value.values():
        _need(type(row) is dict and set(row) == {"attempted_frames", "sent_frames", "blocked_frames"}
              and all(type(n) is int and n >= 0 for n in row.values())
              and row["attempted_frames"] == row["sent_frames"] and row["blocked_frames"] == 0,
              "Unknown, blocked or partial CAN attempts forbid new-round enrollment")
    return value


def _frames(frames, raw, event):
    _need(type(frames) is list and len(frames) == 4
          and [f.get("frame") for f in frames] == _hold_frames(raw)
          and all(f.get("outcome") == "returned" for f in frames), "Four complete frame returns required")
    times = [_number(f.get("returned_at"), "frame return") for f in frames]
    _need(times == sorted(times) and event["began_at"] <= times[0] <= times[-1] <= event["finished_at"],
          "Frame chronology must match its original event")


def _unloaded_opening(event, previous, following, run, bindings):
    """Audit one completed position-mode opening, never a grasp or release.

    Ordinary gripper receipts do not carry a visual empty-jaw claim or raw
    frame receipts. Keep those limitations explicit: the immutable adjacent
    RGB admissions supply the model semantics, while the guarded transport's
    counters (incremented only after its exact frame returns) supply TX facts.
    Neither source is a new physical qualification or a replay permission.
    """
    from .joint_path import evidence_sha256
    from .single_supervised_actions import BOUNDS
    p, r = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    arm = p.get('arm'); peer = 'right' if arm == 'left' else 'left'
    _need(set(p) == {'arm','kind','observation_id','operation','peer_receipt_id','target'}
          and arm in ('left','right') and p['kind'] == 'gripper' and p['operation'] == 'approach'
          and r.get('kind') == 'gripper' and r.get('operation') == 'single_supervised_gripper'
          and r.get('execution_mode') == 'position' and r.get('status') == 'target_arrived_and_stationary_observed'
          and r.get('arm') == r.get('selected_arm') == arm and r.get('passive_arm') == peer
          and r.get('pair_owner') == event['owner'] and r.get('event_id') == event['event_id']
          and r.get('observation_id') == p['observation_id'] and r.get('automatic_retry') is False
          and r.get('observed_stable') is True and r.get('arrival_confirmed') is True
          and r.get('feedback_all_after_send') is True and r.get('grasp_verified') is False
          and all(r.get(k) is None for k in ('contact_observation','completion_mode','hold_receipt',
              'loaded_context','release_opening','unresolved_gripper_probe','grasp_states','retention_contract')),
          'Only the original completed position-mode empty-jaw approach is covered')
    for key, expected in (('hardware_commands_sent',1),('target_calls_sent',1),('target_commands_sent',0),
                          ('enable_commands_sent',0),('stop_commands_sent',0),('retries',0),('passive_arm_commands_sent',0)):
        _need(type(r.get(key)) is int and r[key] == expected, 'Exact opening counter required: '+key)
    counts = {s:dict(attempted_frames=int(s == arm),sent_frames=int(s == arm),blocked_frames=0)
              for s in ('left','right')}
    _need(_counts(r.get('transmission_counts')) == counts, 'Exactly one returned selected-jaw frame required')
    _need(_number(r.get('observed_stable_duration_s'),'opening stability duration') >= BOUNDS['stable_s']
          and type(r.get('observed_feedback_advances')) is int
          and r['observed_feedback_advances'] >= BOUNDS['minimum_feedback_advances'],
          'Opening must retain the complete stable window and advancing feedback')
    spans = r.get('observed_spans')
    span_limits = {'joint_rad':BOUNDS['joint_span_rad'],'position_m':BOUNDS['position_span_m'],
                   'rotation_rad':BOUNDS['rotation_span_rad'],'jaw_m':BOUNDS['jaw_span_m']}
    _need(type(spans) is dict and set(spans) == {'left','right'}, 'Both original stability spans are required')
    for side in spans:
        _need(type(spans[side]) is dict and set(spans[side]) == set(span_limits)
              and all(0 <= _number(spans[side][key],'opening stability span') <= bound
                      for key,bound in span_limits.items()),
              'Opening feedback exceeds the existing stability bounds')
    frames = r.get('frame_preflight_feedback')
    _need(type(frames) is list and len(frames) == 1 and type(frames[0]) is dict and frames[0].get('side') == arm
          and type(frames[0].get('frame_index')) is int and frames[0]['frame_index'] == 0,
          'The original single-frame preflight feedback is required')
    target = _number(p['target'],'opening target'); encoded = round(target*1e6)/1e6
    widths = []
    stamps = {side:{'arm':[],'jaw':[]} for side in ('left','right')}
    for sample in (r.get('before',{}),frames[0].get('arms',{}),r.get('after',{})):
        for side in stamps:
            state = sample.get(side,{}) if type(sample) is dict else {}
            jaw = state.get('gripper',{})
            _need(state.get('status') == jaw.get('status') == 'complete' and jaw.get('mode') == 'width',
                  'Complete pair width feedback is required for opening direction')
            arm_time = _number(state.get('timestamp'),'opening arm feedback time')
            jaw_time = _number(jaw.get('timestamp'),'opening jaw feedback time')
            _need(0 <= arm_time-jaw_time <= BOUNDS['feedback_age_s'],
                  'Opening jaw feedback must be fresh within its original arm sample')
            stamps[side]['arm'].append(arm_time); stamps[side]['jaw'].append(jaw_time)
            if side == arm:
                widths.append(_number(jaw.get('width_m'),'observed opening width'))
    for side in stamps:
        for times in stamps[side].values():
            _need(event['began_at'] <= times[0] <= times[1] < times[2] <= event['finished_at'],
                  'Both arms and jaws need ordered advancing pre/post opening feedback')
    _need(0 <= widths[0] < target <= .055 and 0 <= widths[1] < encoded <= .055
          and target-widths[0] > BOUNDS['jaw_span_m'] and encoded-widths[1] > BOUNDS['jaw_span_m']
          and widths[2]-max(widths[:2]) > BOUNDS['jaw_span_m'] and r.get('requested_target') == target
          and r.get('observed_width_m') == widths[2] and r.get('nominal_force_N') == .2
          and abs(target-widths[2]) <= .002 and r.get('width_error_m') == abs(target-widths[2]),
          'Requested, preflight and observed jaw widths must prove a completed opening')
    # An adjacent successful unloaded RGB segment on either side of the jaw
    # event preserves the historical semantic claim without manufacturing a
    # new same-frame unloaded field in its old payload.
    _need(previous['finished_at'] <= event['began_at'] < event['finished_at'] <= following['began_at'],
          'Opening and adjacent unloaded events must retain their original order')
    for adjacent, step in ((previous,event['step']-1),(following,event['step']+1)):
        q, receipt = json.loads(adjacent['payload_json']), json.loads(adjacent['receipt_json'])
        plan = receipt.get('joint_path_plan',{}); geometry = plan.get('geometry',{})
        evidence = geometry.get('evidence',{}); identity = plan.get('identity',{})
        _need(adjacent['step'] == step and adjacent['success'] == 1
              and adjacent['run_id'] == run['run_id'] and adjacent['owner'] == event['owner']
              and q.get('kind') == 'joint' and q.get('arm') == arm and q.get('admission_mode') == 'rgb_supervised'
              and identity.get('run_id') == run['run_id'] and identity.get('owner') == event['owner']
              and identity.get('worker_id') == adjacent['event_id'] and identity.get('arm') == arm
              and bindings is not None and all(identity.get(k) == bindings[arm][k]
                  for k in ('connection_id','model','firmware_profile'))
              and isinstance(q.get('unloaded_observation'),str) and q['unloaded_observation'].strip()
              and evidence.get('unloaded_observation') == q['unloaded_observation']
              and evidence.get('observation_id') == q.get('observation_id') and evidence.get('identity') == identity
              and evidence_sha256(evidence) == geometry.get('source',{}).get('sha256')
              and evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'}) == plan.get('plan_sha256'),
              'Opening must be bracketed by same-arm completed unloaded RGB admissions')
        images = evidence.get('saved_rgb_evidence')
        _need(type(images) is dict and set(images) == {'front','left_hand','right_hand'},
              'Archived three-view unloaded RGB sources are required')
        for image in images.values():
            _need(type(image) is dict, 'Original RGB source required')
            raw, _ = _file(image.get('rgb_path'))
            received = _number(image.get('host_received_at'),'original image reception')
            _need(hashlib.sha256(raw).hexdigest() == image.get('artifact_sha256')
                  and received <= adjacent['began_at']
                  and (step < event['step'] or received > event['finished_at']),
                  'Archived unloaded images must retain their hashes and before/after order')


def _query_duplicate_failure(event, run, scope, faults):
    """Narrow terminal RX failure; never permits an uncertain actuator send."""
    from .joint_sources import _validate_bindings
    p, receipt = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    r = receipt.get('device_receipt', {})
    _need(event['step'] == run['steps'] == 1 and event['success'] == 0
          and event['status'] == 'complete' and event['finished_at'] is not None
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and p.get('kind') == 'query' and p.get('request') == {'operation':'inspect_joint_limits'}
          and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and receipt.get('event_id') == event['event_id'], 'Only one terminal, non-actuating query is covered')
    _need(set(p) == {'bindings','kind','request'}, 'Exact query payload required')
    _validate_bindings(json.loads(run['contract_json']), p['bindings'])
    _need(r.get('schema') == 'piper_pair_controller_limits_capture_v1'
          and r.get('operation') == 'inspect_joint_limits' and r.get('ok') is False
          and r.get('status') == 'joint_limits_capture_failed' and r.get('fault_latched') is True
          and r.get('controller_limits_changed') is False and r.get('sdk_joint_limits_changed') is False,
          'Exact failed limit capture required')
    for field in ('actuator_commands_sent','target_commands_sent','mode_commands_sent','enable_commands_sent','stop_commands_sent'):
        _need(type(r.get(field)) is int and r[field] == 0, 'Actuator or unknown command blocks query restart')
    for field in ('hardware_commands_sent','joint_limit_queries_sent','joint_limit_queries_attempted'):
        _need(type(r.get(field)) is int and r[field] == 1, 'Exactly one completed query required')
    counts = {s:{'attempted_frames':int(s=='left'),'sent_frames':int(s=='left'),'blocked_frames':0} for s in ('left','right')}
    _need(_counts(r.get('transmission_counts')) == counts == _counts(r.get('session_transmission_counts')),
          'Unknown lifetime transmission blocks query restart')
    error = {'type':'RuntimeError','detail':'Unexpected or duplicate joint-limit response'}
    _need(r.get('errors') == [error,error] and r.get('guard_violations') == [
        {'side':'left','detail':error['detail'],'arbitration_id':0x473}]
        and r.get('pre_or_post_window_limit_frames') == {'left':0,'right':0}
        and r.get('controller_limits_rad') == {'left':[],'right':[]}, 'Other query anomaly requires separate diagnosis')
    _need(set(r['joint_limits']) == {'left','right'} and set(r['joint_limits']['left']) == {'1'}
          and r['joint_limits']['right'] == {} and set(r['query_receipts']) == {'left','right'}
          and set(r['query_receipts']['left']) == {'1'} and r['query_receipts']['right'] == {}, 'First-query evidence required')
    row, q = r['joint_limits']['left']['1'], r['query_receipts']['left']['1']
    _need(q.get('arbitration_id') == 0x472 and q.get('data_hex') == '0101000000000000'
          and q.get('outcome') == 'returned', 'Returned query, never a target, required')
    w = row['response_evidence']
    _need(row.get('status') == 'unconfirmed' and w.get('active') is False
          and w.get('ignored_stale_frames') == [] and len(w.get('response_frames',[])) == 1
          and len(w.get('rejected_frames',[])) == 1, 'Exactly two original RX records required')
    first, second = w['response_frames'][0], w['rejected_frames'][0]
    raw = bytes.fromhex(first['payload_hex'])
    _need(len(raw) == 8 and raw[0] == 1 and raw[7] == 0
          and int.from_bytes(raw[3:5],'big',signed=True) < int.from_bytes(raw[1:3],'big',signed=True)
          and first['dlc'] == second['dlc'] == 8
          and first['payload_hex'] == second['payload_hex'] == row.get('raw_response_hex'), 'Identical valid value records required')
    _need(event['began_at'] <= r['began_at'] <= w['request_started_unix_s'] == q['sent_at']
          <= q['returned_at'] <= first['timestamp'] <= second['timestamp']
          and first['timestamp'] <= first['received_unix_s'] <= second['received_unix_s']
          and second['timestamp'] <= second['received_unix_s'] <= w['finished_unix_s']
          <= r['ended_at'] <= event['finished_at']
          and second['received_unix_s']-q['sent_at'] <= 1.0, 'Original query/RX chronology required')
    own = [f for f in faults if f['run_id'] == run['run_id'] or f['owner'] == event['owner']]
    _need(len(own) == 2 and scope['fault_id'] in [f['id'] for f in own]
          and {f['reason'] for f in own} == {'Claimed preparation/query failed or uncertain: Controller limit query incomplete or uncertain','execution_receipt_failed'}
          and all(f['run_id'] == run['run_id'] and f['owner'] == event['owner'] and f['at'] >= r['ended_at'] for f in own),
          'Additional or unrelated faults block this entry')
    return counts


def _zero_tx_freshness_failure(event, run, scope, faults, totals):
    """Only an empty-jaw target rejected before the first bus attempt."""
    from .joint_path import evidence_sha256
    p, receipt = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    r = receipt.get('device_receipt', {})
    plan = r.get('joint_path_plan', {}); identity = plan.get('identity', {})
    error = {'type':'RuntimeError','detail':'Joint feedback exceeded 50 ms including validation'}
    _need(event['success'] == 0 and event['finished_at'] is not None
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and p.get('kind') == 'joint' and p.get('operation') in ('approach','align')
          and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and receipt.get('event_id') == event['event_id']
          and receipt.get('error') == 'Stable feedback alone is not an arrived single dispatch',
          'Exact terminal predispatch failure required')
    _need(r.get('errors') == [error] and r.get('guard_violations') == []
          and r.get('ok') is False and r.get('automatic_retry') is False
          and r.get('original_event') is None and r.get('original_action_report') is None
          and r.get('hold_receipt') is None and r.get('kind') == 'joint'
          and r.get('status') == 'pair_device_fault' and r.get('hold_supported') is False
          and r.get('hold_policy') == 'latch_only' and r.get('explicit_cancel_hold_bridge_bound') is False
          and r.get('motion_gate_unlocked') is False and r.get('joint_limits_changed') is False,
          'Any other fault, hold or prior send forbids this continuation')
    for key in ('hardware_commands_sent','target_commands_sent','target_calls_sent',
                'enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'):
        _need(type(r.get(key)) is int and r[key] == 0, 'Zero device command required: '+key)
    zero = {s:{'attempted_frames':0,'sent_frames':0,'blocked_frames':0} for s in totals}
    _need(_counts(r.get('transmission_counts')) == zero
          and _counts(r.get('session_transmission_counts')) == totals, 'No missing lifetime or attempted frame allowed')
    _need(identity.get('run_id') == run['run_id'] and identity.get('owner') == scope['owner']
          and identity.get('epoch') == scope['owner'] and identity.get('worker_id') == event['event_id']
          and identity.get('arm') == p.get('arm') and p.get('arm') in totals
          and plan.get('loaded_context') is None and plan.get('loaded_observation_only') is False
          and plan.get('recovery_mode') is None and plan.get('hold_supported') is False
          and plan.get('hold_policy') == 'latch_only'
          and plan.get('spatial_admission_mode') == r.get('spatial_admission_mode') == 'rgb_supervised'
          and plan.get('geometry',{}).get('schema') in
              ('piper_rgb_supervised_joint_path_v1','piper_rgb_supervised_coarse_approach_v1','piper_rgb_supervised_coarse_approach_v2')
          and plan['geometry'].get('evidence',{}).get('operation') == p['operation'], 'Unloaded RGB identity required')
    target = p.get('target')
    _need(type(target) is list and len(target) == 6
          and all(type(v) in (int,float) and math.isfinite(v) for v in target)
          and target == r.get('target_joints_rad') == plan.get('requested_target_joints_rad')
          and [round(v*180000/math.pi) for v in target] == plan.get('target_raw')
          and plan.get('frames') == _hold_frames(plan['target_raw'])
          and evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'}) == plan.get('plan_sha256'),
          'Rejected target and immutable plan must match')
    failure = r.get('tracking_observation',{}).get('first_failure',{})
    _need(failure.get('type') == error['type'] and failure.get('detail') == error['detail']
          and failure.get('sample_role') == 'rejected_observation'
          and failure.get('sample',{}).get('identity') == identity
          and event['began_at'] <= _number(failure['sample'].get('captured_at'),'feedback time') <= event['finished_at'],
          'Original rejected feedback required')
    own = [f for f in faults if f['run_id'] == run['run_id'] or f['owner'] == event['owner']]
    _need(len(own) == 2 and scope['fault_id'] in [f['id'] for f in own]
          and {f['reason'] for f in own} == {'execution_receipt_failed',
              'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch'}
          and all(f['owner'] == event['owner'] and f['run_id'] == run['run_id']
                  and f['at'] >= event['began_at'] for f in own), 'Unrelated faults require separate diagnosis')


def _postsend_rgb_expiry_failure(event, run, scope, faults, totals, bindings):
    return _completed_unloaded_joint_failure(event,run,scope,faults,totals,bindings,tracking=False)


def _completed_unloaded_joint_failure(event, run, scope, faults, totals, bindings, *, tracking):
    """Audit one complete empty-jaw send with a diagnosed terminal guard fault.

    The target is never replayed and arrival is deliberately not inferred from
    frame returns. RGB expiry and a reproduced tracking-envelope rejection have
    separate predicates. New recovery observations are required separately.
    """
    from .joint_path import evidence_sha256
    p, receipt = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    r = receipt.get('device_receipt', {})
    plan, original = r.get('joint_path_plan', {}), r.get('original_event', {})
    identity = plan.get('identity', {}); arm = p.get('arm')
    error = ({'type':'JointPathError','detail':'joint_tracking_envelope'} if tracking else
             {'type':'RuntimeError','detail':'visual_rgb_expired: original joint RGB deadline reached'})
    _need(event['success'] == 0 and event['finished_at'] is not None
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and p.get('kind') == 'joint' and p.get('operation') in ('approach','align')
          and arm in totals and p.get('admission_mode') == 'rgb_supervised'
          and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and receipt.get('event_id') == event['event_id']
          and receipt.get('error') == 'Stable feedback alone is not an arrived single dispatch',
          'Exact terminal complete-send unloaded dispatch failure required')
    _need(r.get('errors') == [error] and r.get('guard_violations') == []
          and r.get('ok') is False and r.get('automatic_retry') is False
          and r.get('hold_receipt') is None and r.get('kind') == 'joint'
          and r.get('status') == 'pair_device_fault' and r.get('hold_supported') is False
          and r.get('hold_policy') == 'latch_only' and r.get('explicit_cancel_hold_bridge_bound') is False
          and r.get('motion_gate_unlocked') is False and r.get('joint_limits_changed') is False
          and r.get('arrival_confirmed') is False,
          'Other faults, holds or claimed arrival are outside complete-send unloaded recovery')
    for key, expected in (('hardware_commands_sent',4),('target_calls_sent',1),
                          ('target_commands_sent',0),('enable_commands_sent',0),
                          ('stop_commands_sent',0),('retries',0),('passive_arm_commands_sent',0)):
        _need(type(r.get(key)) is int and r[key] == expected, 'Exact device counter required: '+key)
    counts = {s:{'attempted_frames':4 if s == arm else 0,'sent_frames':4 if s == arm else 0,
                 'blocked_frames':0} for s in totals}
    _need(_counts(r.get('transmission_counts')) == counts, 'Exactly four selected-arm frames required')
    cumulative = {s:{key:totals[s][key]+counts[s][key] for key in totals[s]} for s in totals}
    _need(_counts(r.get('session_transmission_counts')) == cumulative, 'Lifetime transmissions differ')
    _need(identity.get('run_id') == run['run_id'] and identity.get('owner') == scope['owner']
          and identity.get('epoch') == scope['owner'] and identity.get('worker_id') == event['event_id']
          and identity.get('arm') == arm and bindings is not None
          and all(identity.get(k) == bindings[arm][k] for k in ('connection_id','model','firmware_profile'))
          and plan.get('loaded_context') is None and plan.get('loaded_observation_only') is False
          and plan.get('recovery_mode') is None and plan.get('hold_supported') is False
          and plan.get('hold_policy') == 'latch_only'
          and plan.get('spatial_admission_mode') == r.get('spatial_admission_mode') == 'rgb_supervised',
          'Unloaded same-connection RGB identity required')
    geometry = plan.get('geometry', {}); evidence = geometry.get('evidence', {})
    _need(geometry.get('schema') in ('piper_rgb_supervised_joint_path_v1',
              'piper_rgb_supervised_coarse_approach_v1','piper_rgb_supervised_coarse_approach_v2')
          and evidence.get('identity') == identity and evidence.get('operation') == p['operation']
          and evidence.get('observation_id') == p.get('observation_id')
          and isinstance(p.get('unloaded_observation'),str) and p['unloaded_observation'].strip()
          and evidence.get('unloaded_observation') == p['unloaded_observation']
          and evidence_sha256(evidence) == geometry.get('source',{}).get('sha256'),
          'Original unloaded image admission must match its evidence digest')
    target = p.get('target')
    _need(type(target) is list and len(target) == 6
          and all(type(v) in (int,float) and math.isfinite(v) for v in target)
          and target == r.get('target_joints_rad') == plan.get('requested_target_joints_rad')
          and [round(v*180000/math.pi) for v in target] == plan.get('target_raw')
          and evidence.get('target_raw') == plan['target_raw']
          and plan.get('frames') == _hold_frames(plan['target_raw'])
          and evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'}) == plan.get('plan_sha256'),
          'Sent target and immutable plan must match')
    _need(original.get('schema') == 'piper_rgb_supervised_joint_send_v1'
          and original.get('send_state') == 'all_frames_returned' and original.get('fault') is None
          and original.get('event_id') == event['event_id'] and original.get('identity') == identity
          and original.get('operation') == p['operation']
          and original.get('rgb_admission') == geometry and original.get('target_raw') == plan['target_raw']
          and original.get('plan_sha256') == plan['plan_sha256']
          and original.get('spatial_admission_mode') == 'rgb_supervised'
          and original.get('hold_policy') == 'latch_only' and original.get('hold_supported') is False
          and original.get('deadline_at') == run['started_at']+run['max_duration'],
          'Original complete send identity, plan and deadline required')
    _frames(original.get('frame_receipts'),plan['target_raw'],event)
    rgb_deadline = _number(plan.get('visual_rgb_deadline'),'original RGB deadline')
    _need(rgb_deadline == _number(evidence.get('rgb_received_at'),'original RGB reception')+30.
          and original['frame_receipts'][-1]['returned_at'] < rgb_deadline
          and event['finished_at'] <= original['deadline_at']
          and (tracking or rgb_deadline <= event['finished_at']),
          'Original RGB and execution deadlines must bind the completed send')
    action = r.get('original_action_report', {})
    _need(action.get('errors') == [] and action.get('automatic_retry') is False
          and action.get('target_calls_sent') == 1 and action.get('requested_target') == target
          and all(type(action.get(k)) is int and action[k] == 0 for k in
              ('target_commands_sent','enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent')),
          'Earlier action errors or additional dispatches require separate diagnosis')
    failure = r.get('tracking_observation',{}).get('first_failure',{})
    _need(failure.get('type') == error['type'] and failure.get('detail') == error['detail']
          and failure.get('sample_role') == ('rejected_observation' if tracking else 'latest_observation_before_failure')
          and failure.get('sample',{}).get('identity') == identity
          and original['frame_receipts'][-1]['returned_at'] <=
              _number(failure['sample'].get('captured_at'),'last feedback time') <= event['finished_at'],
          'Original post-send failure observation required')
    own = [f for f in faults if f['run_id'] == run['run_id'] or f['owner'] == event['owner']]
    _need(len(own) == 2 and scope['fault_id'] in [f['id'] for f in own]
          and {f['reason'] for f in own} == {'execution_receipt_failed',
              'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch'}
          and all(f['owner'] == event['owner'] and f['run_id'] == run['run_id']
                  and (failure['sample']['captured_at'] if tracking else rgb_deadline)
                      <= f['at'] <= event['finished_at'] for f in own),
          'Only the original dispatch failure and failed-receipt latch are covered')
    if tracking:
        _need(failure.get('code') == 'joint_tracking_envelope'
              and failure['sample']['identity'] == r.get('rejected_joint_feedback',{}).get('identity')
              and failure['sample']['arms'] == r.get('rejected_joint_feedback',{}).get('arms') == r.get('after')
              and 0 <= failure['sample']['captured_at']
                  - _number(r['rejected_joint_feedback'].get('captured_at'),'rejected read time') <= .05
              and failure['sample']['captured_at'] < rgb_deadline,
              'Original rejected post-send tracking sample required')
        _audit_tracking_feedback(plan,failure['sample'])
    return cumulative


def _audit_tracking_feedback(plan, sample):
    """Preserve a failed process envelope, reject another device/hard-limit fault.

    Reproducing this historical rejection grants no movement and changes no
    bound. A new independent stationary observation is required separately.
    """
    from . import joint_path as jp
    from .feedback_tolerance import joints_within, model_still_limits, rotation_tolerance, validate_policy
    now = _number(sample.get('captured_at'),'rejected sample time')
    try:
        jp.validate_joint_path_sample(plan,sample,now=now,phase='settling')
    except jp.JointPathError as exc:
        _need(exc.code == 'joint_tracking_envelope','Original rejection is not solely the tracking-envelope guard')
    else:
        _need(False,'Original tracking-envelope rejection must remain reproducible')
    identity, origin = plan['identity'], plan['origin']
    states,_ = jp._pair(sample,identity,now,jaw_enabled=True)
    policy = validate_policy(plan.get('feedback_observation'))
    for side,state in states.items():
        base = origin['arms'][side]; active = side == identity['arm']
        _need(all(lo <= v <= hi for v,(lo,hi) in zip(state['joints_rad'],plan['effective_joint_limits_rad'][side])),
              'Absolute joint limit violation requires separate recovery')
        assembly = state.get('feedback_assembly')
        _need(assembly is None or assembly.get('error') is None and assembly.get('timestamps_renewed') is False,
              'Malformed or renewed feedback cannot establish a recoverable history')
        _need(state['arm_status']['mode_feedback'] == base['arm_status']['mode_feedback']
              and state['gripper']['foc_status']['driver_enable_status'] is base['gripper']['foc_status']['driver_enable_status']
              and abs(state['gripper']['width_m']-base['gripper']['width_m']) <= jp.POSITION_STILL_M
              and all(state['fragment_timestamps_s'][k] >= base['fragment_timestamps_s'][k]
                      for k in state['fragment_timestamps_s']),
              'Another mode, jaw or regressing-feedback fault forbids recovery')
        if not active:
            _need(joints_within(policy,side,state['joints_rad'],base['joints_rad'])
                  and state['arm_status']['motion_status'] == 0,'The peer must retain its original stationary anchor')
        raw_m,raw_r = jp._error(jp.pose_matrix(base['pose_m_rad']),jp.pose_matrix(state['pose_m_rad']))
        model_m,model_r = jp._error(plan['model_original_flange_transform'][side],
                                  jp.fk_matrix(plan['model']['mdh'],state['joints_rad']))
        bound_m = plan['budget']['max_translation_m'] if active else jp.POSITION_STILL_M
        bound_r = plan['budget']['max_rotation_rad'] if active else rotation_tolerance(policy,side)
        model_bounds = ((bound_m,bound_r) if active else model_still_limits(policy,side,
                        plan['model']['mdh'],jp.POSITION_STILL_M,jp.ROTATION_STILL_RAD))
        _need(raw_m <= bound_m and raw_r <= bound_r and model_m <= model_bounds[0] and model_r <= model_bounds[1],
              'Another whole-arm process pose violation requires separate recovery')


def _initial_rx_parent_history(tables, run, scope, owned):
    """Audit this unloaded endpoint successor against its frozen predecessor.

    The new window is an append-only administrative action, not removal of the
    old fault or permission to replay its rejected target.
    """
    from .preparation_continuation import _table_sha
    record = json.loads(scope['record_json'])
    parent = record['proposal']
    _need(_sha({k:v for k,v in parent.items() if k != 'proposal_sha256'}) == parent.get('proposal_sha256')
          == scope['proposal_sha256']
          and parent.get('schema') == 'piper_zero_tx_endpoint_continuation_v1'
          and parent['run_id'] == run['run_id'] and scope['owner'] == owned[-1]['owner']
          and record.get('old_rows_preserved') is True and record.get('new_budget_allocated') is False
          and record.get('hardware_commands_sent') == 0 and record.get('cache_or_limits_transferred') is False
          and record.get('physical_stop_verified') is None
          and json.loads(scope['contract_json']) == record['new_contract'] == parent['reviewed_contract']
          and parent['budget'] == {'started_at':run['started_at'],'max_steps':run['max_steps'],
              'max_duration_s':run['max_duration'],'steps':owned[0]['step']-1,
              'deadline_s':run['started_at']+run['max_duration']}
          and parent['created_at'] <= record['activated_at'] < owned[0]['began_at'],
          'Unchanged endpoint predecessor required')
    kinds = [json.loads(e['payload_json']).get('kind') for e in owned]
    _need(len(owned) >= 4 and kinds[:3] == ['query','initialization','initialization']
          and all(k == 'joint' for k in kinds[3:])
          and {json.loads(e['receipt_json']).get('arm') for e in owned[1:3]} == {'left','right'},
          'Only one query, both initialized empty arms and unloaded joint segments are covered')
    _need(len(tables.get('pair_endpoint_continuations', [])) == 1
          and all(a['finished_at'] <= b['began_at'] for a,b in zip(owned,owned[1:])),
          'One endpoint successor and ordered nonoverlapping events required')
    for name in ('pair_joint_sends','pair_hold_requests','pair_holds','pair_hold_frames','pair_grasp_episodes'):
        _need(not any(r['run_id'] == run['run_id'] for r in tables.get(name, [])),
              'Any send bridge, hold or grasp episode excludes this repair entry')
    prior = copy.deepcopy(tables)
    prior.pop('pair_endpoint_continuations')
    prior['pair_events'] = [e for e in prior['pair_events'] if e not in owned]
    # A later CLI open was refused by the already-latched ledger before the
    # device factory ran. Its shutdown attempted cancel under an unclaimed
    # owner, recording owner_mismatch. Audit its actual CLI log separately;
    # retain that fault too, rather than treating it as actuator uncertainty.
    refused = [f for f in prior['pair_faults'] if f['at'] > owned[-1]['finished_at']]
    _need(len(refused) == 1 and refused[0]['run_id'] == run['run_id']
          and refused[0]['reason'] == 'owner_mismatch' and refused[0]['owner'] != scope['owner']
          and not any(e['owner'] == refused[0]['owner'] for e in tables['pair_events']),
          'Only the diagnosed rejected-open cleanup record is covered')
    prior['pair_faults'] = [f for f in prior['pair_faults'] if f['owner'] != scope['owner'] and f not in refused]
    next(r for r in prior['pair_runs'] if r['run_id'] == run['run_id'])['steps'] -= len(owned)
    expected = parent['snapshot']['table_sha256']
    _need(set(prior) == set(expected), 'Unexpected historical table changes')
    for name, digest in expected.items():
        _need(_table_sha(prior[name]) == digest, 'Historical rows changed: '+name)


def _initial_rx_failure(event, run, scope, faults, totals, bindings):
    from .joint_path import evidence_sha256
    p, receipt = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    r = receipt.get('device_receipt', {})
    plan = r.get('joint_path_plan', {}); identity = plan.get('identity', {})
    failure = r.get('tracking_observation', {}).get('first_failure', {})
    sample = failure.get('sample', {}); timing = r.get('joint_validation_timing', {})
    error = {'type':'JointPathError', 'detail':'right/driver_state_1'}
    _need(event['success'] == 0 and event['status'] == 'complete' and event['step'] == run['steps']
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and p.get('kind') == 'joint' and p.get('operation') in ('approach','align')
          and p.get('arm') == 'left' and p.get('admission_mode') == 'rgb_supervised'
          and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and receipt.get('event_id') == event['event_id']
          and receipt.get('error') == 'Stable feedback alone is not an arrived single dispatch',
          'Exact completed initial RX failure required')
    _need(r.get('errors') == [error] and r.get('guard_violations') == [] and r.get('ok') is False
          and r.get('automatic_retry') is False and r.get('arrival_confirmed') is False
          and r.get('before') is None and r.get('samples') == 1
          and all(r.get(k) is None for k in ('original_event','original_action_report','hold_receipt','sample'))
          and r.get('hold_policy') == 'latch_only' and r.get('hold_supported') is False
          and r.get('status') == 'pair_device_fault' and r.get('kind') == 'joint'
          and r.get('motion_gate_unlocked') is False and r.get('joint_limits_changed') is False
          and r.get('explicit_cancel_hold_bridge_bound') is False,
          'Failure must precede the first accepted baseline and every send')
    for key in ('hardware_commands_sent','target_calls_sent','target_commands_sent','enable_commands_sent',
                'stop_commands_sent','retries','passive_arm_commands_sent'):
        _need(type(r.get(key)) is int and r[key] == 0, 'Zero dispatch counter required: '+key)
    _need(_counts(r.get('transmission_counts')) == {s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in totals}
          and _counts(r.get('session_transmission_counts')) == totals, 'Exact unchanged lifetime TX required')
    _need(identity.get('run_id') == run['run_id'] and identity.get('owner') == scope['owner']
          and identity.get('epoch') == scope['owner'] and identity.get('worker_id') == event['event_id']
          and identity.get('arm') == p['arm'] and bindings is not None
          and all(identity.get(k) == bindings[p['arm']][k] for k in ('connection_id','model','firmware_profile'))
          and plan.get('loaded_context') is None and plan.get('recovery_mode') is None
          and plan.get('loaded_observation_only') is False and plan.get('hold_supported') is False
          and plan.get('hold_policy') == 'latch_only' and plan.get('spatial_admission_mode') == 'rgb_supervised'
          and evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'}) == plan.get('plan_sha256'),
          'Original unloaded same-connection plan required')
    geometry = plan.get('geometry', {}); evidence = geometry.get('evidence', {})
    _need(geometry.get('schema') == 'piper_rgb_supervised_joint_path_v1'
          and evidence.get('identity') == identity and evidence.get('operation') == p['operation']
          and evidence.get('observation_id') == p.get('observation_id')
          and isinstance(p.get('unloaded_observation'), str) and p['unloaded_observation'].strip()
          and evidence.get('unloaded_observation') == p['unloaded_observation']
          and evidence_sha256(evidence) == geometry.get('source', {}).get('sha256')
          and type(p.get('target')) is list and len(p['target']) == 6
          and all(type(v) in (int,float) and math.isfinite(v) for v in p['target'])
          and p.get('target') == r.get('target_joints_rad') == plan.get('requested_target_joints_rad')
          and [round(v*180000/math.pi) for v in p['target']] == plan.get('target_raw')
          and evidence.get('target_raw') == plan['target_raw']
          and plan.get('frames') == _hold_frames(plan['target_raw']), 'Immutable original image/target admission required')
    _need({k:failure.get(k) for k in ('type','detail')} == error and failure.get('code') == 'stale_feedback'
          and failure.get('sample_role') == 'rejected_observation' and sample.get('identity') == identity
          and sample.get('arms') == r.get('rejected_joint_feedback', {}).get('arms') == r.get('after')
          and timing.get('live_feedback_age_limit_s') == .05
          and all(timing.get(k) is None for k in ('checked_at','oldest_fragment_age_at_exit_s','validation_elapsed_s')),
          'Original rejected sample and entry-only freshness failure required')
    at = _number(timing.get('entry_at'), 'validation entry')
    from .arms import control_health, PARTS, DRIVERS
    _need(type(sample.get('arms')) is dict and set(sample['arms']) == {'left','right'},
          'Complete rejected pair feedback required')
    for side, state in sample['arms'].items():
        _need(type(state) is dict and state.get('status') == 'complete'
              and control_health(state,now_s=at)['healthy']
              and state['arm_status'].get('teach_status') == 0
              and state['arm_status'].get('mode_feedback') == 1
              and state['arm_status'].get('motion_status') == 0
              and 0 <= state['gripper']['width_m'] <= .070
              and set(state.get('fragment_timestamps_s',{})) == set(PARTS+DRIVERS+('gripper',))
              and state.get('feedback_assembly',{}).get('error') is None
              and state.get('feedback_assembly',{}).get('timestamps_renewed') is False,
              'Another feedback, device or grouping fault excludes initial RX repair')
        limits = plan.get('effective_joint_limits_rad',{}).get(side)
        _need(type(limits) is list and len(limits) == 6
              and all(low <= q <= high for q,(low,high) in zip(state['joints_rad'],limits)),
              'Rejected feedback outside the original hard joint bounds')
    stamps = [_number(t,'original fragment time') for state in sample['arms'].values()
              for t in state['fragment_timestamps_s'].values()]
    _need(.05 < at-sample['arms']['right']['fragment_timestamps_s']['driver_state_1'] <= .1
          and timing.get('oldest_fragment_age_at_entry_s') == max(at-t for t in stamps)
          and all(0 <= at-t <= .1 for t in stamps)
          and event['began_at'] <= sample['captured_at'] == at <= event['finished_at'],
          'Only the diagnosed bounded initial cache delay is covered; no timestamps are renewed')
    own = [f for f in faults if f['owner'] == event['owner']]
    _need(len(own) == 2 and scope['fault_id'] in [f['id'] for f in own]
          and {f['reason'] for f in own} == {'execution_receipt_failed',
              'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch'}
          and all(f['run_id'] == run['run_id'] and at <= f['at'] <= event['finished_at'] for f in own),
          'Additional or unrelated faults forbid this repair entry')


def _initial_rx_closed(path, snapshot, evidence_sources=None):
    """Actual CLI finally/shutdown evidence, explicitly not a pair_close RPC."""
    raw, ref = _file(path); manifest = json.loads(raw)
    names = {'before','shutdown_intent','shutdown_signal_result','close_verification','session_log','rejected_open_log'}
    _need(set(manifest) == names | {'schema'} and manifest['schema'] == 'piper_cli_shutdown_evidence_v1',
          'Exact CLI shutdown evidence manifest required')
    _need(evidence_sources is None or type(evidence_sources) is dict and set(evidence_sources) == names,
          'Exact original closure source references required')
    values, refs = {}, {}
    for name in names:
        source = manifest[name]
        if evidence_sources is not None:
            archived = evidence_sources[name]['path']
            relative, absolute = Path(source), Path(archived)
            _need(absolute.is_absolute() and (relative == absolute if relative.is_absolute() else
                  bool(relative.parts) and '..' not in relative.parts
                  and absolute.parts[-len(relative.parts):] == relative.parts),
                  'Original manifest entry differs from its frozen absolute reference')
            source = archived
        data, refs[name] = _file(source)
        _need(evidence_sources is None or refs[name] == evidence_sources[name],
              'Original closure source bytes changed')
        values[name] = ([json.loads(line) for line in data.decode().splitlines()] if name.endswith('_log') else json.loads(data))
    b, intent, signal, closed = (values[k] for k in ('before','shutdown_intent','shutdown_signal_result','close_verification'))
    rows = values['session_log']; owner = snapshot['retired_owner']; run = snapshot['run']
    _need(rows and [r['sequence'] for r in rows] == list(range(1,len(rows)+1))
          and [r['at'] for r in rows] == sorted(r['at'] for r in rows)
          and rows[0]['kind'] == 'session_started' and rows[-1] == closed.get('session_end')
          and rows[-1]['kind'] == 'session_ended' and rows[-1]['cleanup_errors'] == []
          and rows[-1]['automatic_retry'] is False and rows[-1]['physical_stop_verified'] is None,
          'Complete ordered CLI log and successful finally cleanup required')
    requests = [r for r in rows if r['kind'] == 'request']
    allowed = {'capture','note','robot_pair_open','robot_pair_inspect_joint_limits','robot_pair_status',
               'robot_pair_observe','robot_pair_initialize_joint_target','robot_pair_promote_ready','robot_pair_submit_once'}
    _need(all(r['request']['op'] in allowed for r in requests)
          and all(r['kind'] in ('session_started','request','result','session_ended') for r in rows),
          'Unexpected command, error or incomplete request excludes CLI shutdown audit')
    def result(request):
        found = [r for r in rows if r['kind'] == 'result' and r.get('request_id') == request['request']['id']]
        _need(len(found) == 1 and request['at'] <= found[0]['at'], 'Exactly one ordered RPC result required')
        return found[0]['result']
    _need(len({r['request']['id'] for r in requests}) == len(requests)
          and len([r for r in rows if r['kind'] == 'result']) == len(requests), 'Unique complete CLI requests required')
    opened = [result(r) for r in requests if r['request']['op'] == 'robot_pair_open']
    _need(len(opened) == 1 and opened[0].get('owner') == owner and opened[0].get('run_id') == run['run_id']
          and opened[0].get('connection_mode') == 'prepare', 'Original owner binding required')
    claimed = []
    for request in requests:
        response = result(request); op = request['request']['op']; args = request['request']['arguments']
        if op not in ('robot_pair_inspect_joint_limits','robot_pair_initialize_joint_target','robot_pair_submit_once'):
            continue
        if response.get('status') == 'refresh_required':
            _need(response.get('event_claimed') is False and response.get('fault_latched') is False
                  and response.get('hardware_commands_sent') == response.get('steps_consumed') == 0,
                  'Only explicitly unclaimed RGB refreshes may precede dispatch')
            continue
        event = next((e for e in snapshot['retired_events'] if e['event_id'] == args.get('event_id')), None)
        _need(event is not None and response.get('status') == 'pending', 'Every actual claim must match the ledger')
        payload = event['payload']
        if op == 'robot_pair_inspect_joint_limits':
            expected = {'event_id':event['event_id']}
            _need(payload.get('kind') == 'query', 'Query kind differs')
        elif op == 'robot_pair_initialize_joint_target':
            expected = {'event_id':event['event_id'], **{k:v for k,v in payload['request'].items() if k != 'operation'}}
            _need(payload.get('kind') == 'initialization', 'Initialization kind differs')
        else:
            expected = {'event_id':event['event_id'], **{k:v for k,v in payload.items() if k != 'target'},
                        'target_joints_rad':payload['target']}
            _need(payload.get('kind') == 'joint', 'Joint kind differs')
        _need(args == expected, 'RPC arguments must match the original immutable payload')
        claimed.append(event['event_id'])
        completions = [r['result'] for r in rows if r['kind'] == 'result'
            and r['result'].get('event_id') == event['event_id'] and r['result'].get('status') in ('completed','fault')]
        _need(len(completions) == 1 and _sha(completions[0].get('receipt')) == event['receipt_sha256'],
              'Original RPC completion and immutable ledger must agree')
    _need(claimed == [e['event_id'] for e in snapshot['retired_events']], 'Missing, duplicate or extra dispatched event')
    _need(b['pid'] == intent['pid'] == closed['original_pid'] and b['stat_start_ticks'] == intent['start_ticks']
          and intent.get('signal') == 'SIGINT' and intent.get('signal_count') == 1
          and intent.get('no_active_or_pending_action') is True and intent.get('grasp_episode_count') == 0
          and intent.get('explicit_pair_close_rpc') is False and signal.get('signal_sent_once') is True
          and signal.get('original_process_still_present') is False
          and closed.get('original_process_exited') is True and closed.get('execution_lock_available') is True
          and closed.get('runtime_sources_unchanged') is True and closed.get('fault_cleared') is False
          and closed.get('physical_stop_verified') is None
          and snapshot['last_finished_at'] < b['at_s'] <= intent['at_s'] <= rows[-1]['at'] <= signal['at_s'] <= closed['at_s'],
          'Identified single-signal zero-pending finally shutdown required; not a physical stop claim')
    _need(closed.get('scope_after') == {k:snapshot['scope'][k] for k in ('run_id','owner','active_run_id','fault_id')}
          and closed.get('run_after') == {k:run[k] for k in ('run_id','steps','started_at','max_steps','max_duration')},
          'Closure must preserve the original fault, history and budget')
    channels = {v['channel'] for v in snapshot['effective_contract']['arms'].values()}
    _need(set(b['tx_packets']) == set(closed['tx_packets_after']) == set(closed['tx_packet_change']) == channels
          and all(int(b['tx_packets'][c]) == closed['tx_packets_after'][c] and closed['tx_packet_change'][c] == 0 for c in channels),
          'Both independent interface counters must remain unchanged through cleanup')
    for name in ('service.py','pair_host.py','pair_device.py'):
        _need(b['source_sha256'].get('projects/piperx_cloth_demo/robot_tools/'+name) == snapshot['effective_contract']['code'][name],
              'Original shutdown source must match the retired host contract')
    rejected = values['rejected_open_log']; fault = snapshot['rejected_open_fault']
    _need(len(rejected) == 8 and [r['sequence'] for r in rejected] == list(range(1,9))
          and [r['kind'] for r in rejected] == ['session_started','request','result','request','request_error','request','result','session_ended']
          and [r['at'] for r in rejected] == sorted(r['at'] for r in rejected)
          and [rejected[i]['request']['op'] for i in (1,3,5)] == ['capture','robot_pair_open','note']
          and rejected[3]['request']['arguments'].get('run_id') == run['run_id']
          and rejected[4].get('request_id') == rejected[3]['request']['id']
          and rejected[4].get('error') == 'PairLedgerFault: Database dispatch fault is latched'
          and rejected[4].get('automatic_retry') is False
          and rejected[2]['result'].get('hardware_commands_sent') == 0
          and rejected[-1].get('cleanup_errors') == [] and rejected[-1].get('physical_stop_verified') is None
          and closed['at_s'] < rejected[0]['at'] < rejected[4]['at'] <= fault['at'] <= rejected[-1]['at'],
          'Exact later claim refusal before device construction and completed cleanup required')
    return {'source':ref,'evidence_sources':refs,'retired_owner':owner,'exit_observed_at':closed['at_s'],
            'closure_kind':'cli_finally_shutdown','explicit_pair_close_rpc':False,'physical_stop_verified':None}


def _initial_rx_observations(evidence, snapshot, now):
    from .pair_task_enrollment import _observations
    _need(type(evidence) is dict and set(evidence) == {'passive_paths','rgb_observation','visual_observation'},
          'Fresh independent pair/RGB observations required')
    return _observations(**evidence, contract=snapshot['effective_contract'], after=snapshot['last_finished_at'], now=now)


def _snapshot(db, run_id, parent_kind='clean'):
    if parent_kind == 'supported_contact_zero_tx_fault':
        from .supported_gripper_recovery import contact_zero_tx_snapshot
        return contact_zero_tx_snapshot(db,run_id)
    from .grasp_episode import is_resolved_release
    from .joint_sources import _validated_limits
    table, key, ordinal, scope = _execution_scope(db, run_id, writable=True)
    run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
    query_fault = parent_kind == 'query_duplicate_fault'
    freshness_fault = parent_kind == 'zero_tx_freshness_fault'
    rgb_fault = parent_kind == 'postsend_rgb_expiry_fault'
    initial_rx_fault = parent_kind == 'initial_rx_zero_tx_fault'
    tracking_fault = parent_kind == 'completed_unloaded_joint_fault'
    configuration_fault = parent_kind == 'configuration_maintenance_fault'
    _need(parent_kind in ('clean','query_duplicate_fault','zero_tx_freshness_fault','postsend_rgb_expiry_fault',
                         'initial_rx_zero_tx_fault','completed_unloaded_joint_fault','configuration_maintenance_fault'), 'Unknown parent state')
    _need(run is not None and scope is not None, 'Current run required')
    if query_fault or freshness_fault or rgb_fault or initial_rx_fault or tracking_fault or configuration_fault:
        _need(table == ('pair_endpoint_continuations' if initial_rx_fault else 'pair_rounds')
              and scope['active_run_id'] == run_id and scope['owner'] is not None
              and scope['fault_id'] is not None, 'Explicit current round query fault required')
    else:
        _need(scope['owner'] is None and scope['active_run_id'] is None and scope['fault_id'] is None,
              'Current effective run must be cleanly detached without an active fault')
    latest = db.execute("SELECT run_id FROM pair_runs ORDER BY started_at DESC,run_id DESC LIMIT 1").fetchone()
    _need(latest is not None and latest[0] == run_id, "Only the latest effective round may authorize its successor")
    tables = {}
    for item in db.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"):
        name = item[0]
        if name.startswith("pair_"):
            _need(name.replace("_", "").isalnum(), "Unexpected ledger table")
            tables[name] = sorted([dict(row) for row in db.execute('SELECT * FROM "'+name+'"')],
                                 key=lambda row: json.dumps(row, sort_keys=True))
    _need(not any(e["status"] != "complete" for e in tables["pair_events"]), "Pending event blocks enrollment")
    for name in ("pair_holds", "pair_hold_requests", "pair_hold_frames"):
        _need(not tables.get(name), "Hold transactions are outside this healthy new-round entry")
    for row in tables.get("pair_grasp_episodes", []):
        state = json.loads(row["state_json"])
        _need(state["status"] == "empty" or is_resolved_release(state), "Unresolved grasp blocks new round")
    events = sorted([e for e in tables["pair_events"] if e["run_id"] == run_id], key=lambda e:e["step"])
    _need(events and events[-1]["step"] == run["steps"], "Current run needs its complete final event")
    for historical in tables["pair_runs"]:
        steps=sorted(e["step"] for e in tables["pair_events"] if e["run_id"] == historical["run_id"])
        _need(steps == list(range(1,historical["steps"]+1)), "All prior run event history must remain contiguous")
    owner = events[-1]["owner"]
    owned = [e for e in events if e["owner"] == owner]
    _need([e["step"] for e in owned] == list(range(owned[0]["step"], run["steps"]+1)),
          "Retired owner's events must form the latest uninterrupted suffix")
    if not (query_fault or freshness_fault or rgb_fault or initial_rx_fault or tracking_fault or configuration_fault):
        _need(not any(f["owner"] == owner for f in tables["pair_faults"]), "Latest owner had a fault; not healthy closure")
    totals = {s:{"attempted_frames":0,"sent_frames":0,"blocked_frames":0} for s in ("left","right")}
    bindings = None
    maintenance_prefix = None
    if rgb_fault and len(events) != len(owned):
        from .configuration_recovery import completed_prefix
        maintenance_prefix = completed_prefix([e for e in events if e['owner'] != owner], run, owned[0])
    if initial_rx_fault:
        _initial_rx_parent_history(tables, dict(run), dict(scope), owned)
    if rgb_fault or tracking_fault:
        kinds = [json.loads(e['payload_json']).get('kind') for e in owned]
        _need((len(events) == len(owned) or maintenance_prefix is not None)
              and owner == scope['owner'] and len(owned) >= 4
              and kinds[:3] == ['query','initialization','initialization']
              and all(k in (('joint',) if tracking_fault else ('joint','gripper')) for k in kinds[3:])
              and kinds.count('gripper') <= 1,
              'One query, both initializations, unloaded joints and at most one empty opening are covered')
        if 'gripper' in kinds:
            _need('pair_grasp_episodes' in tables
                  and not any(row['run_id'] == run_id for row in tables['pair_grasp_episodes']),
                  'Any grasp episode excludes this empty-opening recovery history')
        _need({json.loads(e['receipt_json']).get('arm') for e in owned[1:3]} == {'left','right'},
              'Each arm must have its own initialization')
    if tracking_fault:
        _need(not any(row['run_id'] == run_id for name in ('pair_joint_sends','pair_grasp_episodes')
                      for row in tables.get(name,[])),
              'Grasp or hold-send history cannot use unloaded joint recovery')
        registration = json.loads(scope['record_json']); enrolled = registration['proposal']
        _need(enrolled['new_run_id'] == run_id and enrolled['parent_run_id'] == scope['parent_run_id']
              and _sha({k:v for k,v in enrolled.items() if k != 'proposal_sha256'})
                  == enrolled['proposal_sha256'] == scope['proposal_sha256']
              and _sha(registration['authorization']) == scope['authorization_sha256']
              and json.loads(run['contract_json']) == registration['new_contract'] == enrolled['reviewed_contract']
              and enrolled['new_budget'] == {'started_at':run['started_at'],'max_steps':run['max_steps'],
                                            'max_duration_s':run['max_duration']}
              and registration['activated_at'] <= owned[0]['began_at'],
              'The failed run must preserve its exact original enrollment and budget')
        if enrolled.get('parent_kind') == 'initial_rx_zero_tx_fault':
            audit_unopened_enrollment(db,scope,run)
        elif enrolled.get('parent_kind') == 'completed_unloaded_joint_fault':
            audit_completed_unloaded_round(db,scope,run)
    for e in owned:
        if configuration_fault:
            from .configuration_recovery import failure
            _need(len(owned) == 1, 'Only one maintenance event may belong to this retired owner')
            totals = failure(e, run, scope, tables['pair_faults'], totals)
            continue
        if tracking_fault and e == owned[-1]:
            totals = _completed_unloaded_joint_failure(e,run,scope,tables['pair_faults'],totals,bindings,tracking=True)
            continue
        if initial_rx_fault and e == owned[-1]:
            _initial_rx_failure(e, dict(run), dict(scope), tables['pair_faults'], totals, bindings)
            continue
        if rgb_fault and e == owned[-1]:
            totals = _postsend_rgb_expiry_failure(e, run, scope, tables['pair_faults'], totals, bindings)
            continue
        if freshness_fault and e == owned[-1]:
            _need(len(events) == len(owned) and owner == scope['owner'], 'Only the current single owner is covered')
            _zero_tx_freshness_failure(e, run, scope, tables['pair_faults'], totals)
            continue
        if query_fault:
            _need(len(events) == len(owned) == 1 and e['owner'] == scope['owner'], 'No other owner or event allowed')
            totals = _query_duplicate_failure(e, run, scope, tables['pair_faults'])
            continue
        _need(e["success"] == 1 and e["finished_at"] is not None
              and hashlib.sha256(e["payload_json"].encode()).hexdigest() == e["payload_digest"],
              "Successful immutable event receipts required for the retired owner")
        p, r = json.loads(e["payload_json"]), json.loads(e["receipt_json"])
        _need(r.get("ok") is True and r.get("guard_violations") == [] and not r.get("errors")
              and r.get("hold_receipt") is None, "Uncertain, failed or hold action blocks enrollment")
        counts = _counts(r.get("transmission_counts"))
        sent = sum(v["sent_frames"] for v in counts.values())
        _need(type(r.get("hardware_commands_sent")) is int and r["hardware_commands_sent"] == sent,
              "Event transmission count mismatch")
        for name in ("enable_commands_sent", "stop_commands_sent"):
            _need(type(r.get(name)) is int and r[name] == 0, "Only existing target/query routes are covered")
        kind = p.get("kind")
        if kind == "query":
            bindings = p.get('bindings')
            _need(p.get("request") == {"operation":"inspect_joint_limits"} and sent == 12
                  and type(r.get("actuator_commands_sent")) is int and r["actuator_commands_sent"] == 0,
                  "Only complete non-actuating limit queries are covered")
            for name, expected in (("joint_limit_queries_attempted",12),("joint_limit_queries_sent",12),
                                   ("target_commands_sent",0),("mode_commands_sent",0)):
                _need(type(r.get(name)) is int and r[name] == expected, "Query counter mismatch: "+name)
            _validated_limits({**r,"run_id":run_id,"owner":owner,"bindings":p["bindings"]},
                run_id=run_id,owner=owner,bindings=p["bindings"],now=e["finished_at"])
            _need(e["began_at"] <= r["began_at"] <= r["ended_at"] <= e["finished_at"],
                  "Raw query evidence must lie inside its claimed event")
        elif kind == "initialization":
            plan=r.get("initialization_plan",{})
            _need(sent == 4 and r.get("cache_established") is True
                  and p.get("expected_target_raw") == plan.get("target_raw"), "Complete initialization required")
            _frames(r.get("frame_receipts"),plan.get("target_raw"),e)
            if rgb_fault or initial_rx_fault or tracking_fault:
                identity = plan.get('identity', {}); arm = r.get('arm')
                _need(identity.get('run_id') == run_id and identity.get('owner') == owner
                      and identity.get('epoch') == owner and identity.get('worker_id') == e['event_id']
                      and identity.get('arm') == arm and bindings is not None
                      and all(identity.get(k) == bindings[arm][k] for k in ('connection_id','model','firmware_profile')),
                      'Initializations must bind this run and queried connections')
        elif kind == "joint":
            original=r.get("original_event",{})
            _need(sent == 4 and original.get("send_state") == "all_frames_returned"
                  and original.get("event_id") == e["event_id"] and r.get("arrival_confirmed") is True,
                  "Only completely returned and observed-arrived joint events are covered")
            _frames(original.get("frame_receipts"),original.get("target_raw"),e)
            if rgb_fault or initial_rx_fault or tracking_fault:
                plan = r.get('joint_path_plan', {}); identity = plan.get('identity', {})
                _need(p.get('operation') in ('approach','align') and p.get('admission_mode') == 'rgb_supervised'
                      and plan.get('loaded_context') is None and plan.get('loaded_observation_only') is False
                      and plan.get('recovery_mode') is None and plan.get('hold_supported') is False
                      and plan.get('hold_policy') == 'latch_only' and plan.get('spatial_admission_mode') == 'rgb_supervised'
                      and identity.get('run_id') == run_id and identity.get('owner') == owner
                      and identity.get('epoch') == owner and identity.get('worker_id') == e['event_id']
                      and identity.get('arm') == p.get('arm'), 'Earlier loaded or foreign actions forbid RGB recovery')
        elif kind == "gripper":
            _need(sent == 1 and r.get("target_calls_sent") == 1 and r.get("arrival_confirmed") is True
                  and r.get("feedback_all_after_send") is True, "Complete observed jaw target required")
            if rgb_fault:
                index = owned.index(e)
                _need(3 < index < len(owned)-2, 'Empty opening requires completed adjacent unloaded segments')
                _unloaded_opening(e,owned[index-1],owned[index+1],run,bindings)
        else:
            _need(False,"Unsupported retired-owner event kind")
        for side in totals:
            for field in totals[side]: totals[side][field] += counts[side][field]
        _need(_counts(r.get("session_transmission_counts")) == totals, "Retired-owner lifetime mismatch")
    hashes={name:hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()
            for name,rows in tables.items()}
    owners = {row[field] for rows in tables.values() for row in rows for field in ("owner", "previous_owner")
              if row.get(field)}
    for rows in tables.values():
        for row in rows:
            if row.get("retired_owners_json"): owners.update(json.loads(row["retired_owners_json"]))
    owners=sorted(owners)
    from .pair_ledger import effective_contract_json
    summary={"run":dict(run),"scope_table":table,"scope_key":key,"scope_ordinal":ordinal,
        "scope":{k:scope[k] for k in scope.keys() if k not in ("record_json", "contract_json")},
        "effective_contract":json.loads(effective_contract_json(db,run,scope)),
        "retired_owner":owner,"retired_owners":owners,"session_transmission_counts":totals,
        "last_finished_at":owned[-1]["finished_at"],"cumulative_prior_steps":sum(r["steps"] for r in tables["pair_runs"]),
        "table_sha256":hashes,"table_rows":{name:len(rows) for name,rows in tables.items()}}
    if query_fault:
        summary['query_fault_event'] = events[0]
        summary['parent_kind'] = parent_kind
    if freshness_fault:
        summary['freshness_fault_event'] = events[-1]
        summary['parent_kind'] = parent_kind
    if rgb_fault:
        summary['rgb_expiry_fault_event'] = events[-1]
        summary['parent_kind'] = parent_kind
        if maintenance_prefix is not None:
            summary['completed_maintenance_prefix'] = maintenance_prefix
    if initial_rx_fault:
        # Keep original bodies in the authoritative ledger. Bind their digests
        # here rather than duplicating megabytes into every successor record.
        summary['retired_events'] = [{**{k:v for k,v in e.items() if k not in ('payload_json','receipt_json')},
            'payload':json.loads(e['payload_json']), 'receipt_sha256':_sha(json.loads(e['receipt_json']))} for e in owned]
        summary['initial_rx_fault_event'] = summary['retired_events'][-1]
        summary['rejected_open_fault'] = next(f for f in tables['pair_faults'] if f['at'] > owned[-1]['finished_at'])
        summary['parent_kind'] = parent_kind
    if tracking_fault:
        event = events[-1]; receipt = json.loads(event['receipt_json'])['device_receipt']
        plan = receipt['joint_path_plan']; arm = plan['identity']['arm']; peer = 'right' if arm == 'left' else 'left'
        summary['parent_kind'] = parent_kind
        summary['terminal_unloaded_dispatch'] = {'event_id':event['event_id'],'event_sha256':_sha(event),
            'arm':arm,'peer':peer,'plan_sha256':plan['plan_sha256'],'target_raw':plan['target_raw'],
            'settle_tolerances_rad':(plan['tracking_policy']['settle_tolerances_rad']
                if 'settle_tolerances_rad' in plan['tracking_policy'] else [plan['tracking_policy']['settle_tolerance_rad']]*6),
            'peer_origin_joints_rad':plan['origin']['arms'][peer]['joints_rad'],
            'joint_limits_rad':plan['effective_joint_limits_rad'],
            'last_frame_returned_at':receipt['original_event']['frame_receipts'][-1]['returned_at'],
            'original_failed_result_preserved':True,'physical_stop_verified':None}
        if 'pair_arm_power_cycles' in tables:
            from .arm_power_cycle import audit_completed_startup_history
            summary['post_fault_startup'] = audit_completed_startup_history(db)
    if configuration_fault:
        summary['parent_kind'] = parent_kind
        e = owned[0]
        summary['configuration_failure'] = {'event_id':e['event_id'], 'event_sha256':_sha(e),
            'receipt':json.loads(e['receipt_json']),
            'fault':next(f for f in tables['pair_faults'] if f['id'] == scope['fault_id'])}
    return summary


def _closed(path, snapshot, evidence_sources=None):
    if snapshot.get('parent_kind') == 'supported_contact_zero_tx_fault':
        from .supported_gripper_recovery import contact_zero_tx_closed
        return contact_zero_tx_closed(path,snapshot)
    if snapshot.get('parent_kind') == 'configuration_maintenance_fault':
        from .configuration_recovery import closed
        return closed(path, snapshot)
    if snapshot.get('parent_kind') == 'initial_rx_zero_tx_fault':
        return _initial_rx_closed(path, snapshot, evidence_sources)
    if snapshot.get('parent_kind') == 'query_duplicate_fault':
        return _query_closed(path, snapshot)
    raw, ref = _file(path); values=[]
    try: rows=[json.loads(raw)]
    except ValueError: rows=[json.loads(line) for line in raw.decode().splitlines()]
    for row in rows:
        if row.get("status") == "closed":values.append(row)
        if row.get('result',{}).get('status') == 'closed':values.append(row['result'])
        for item in row.get("result",{}).get("content",[]):
            if item.get("type") == "text":
                value=json.loads(item["text"])
                if value.get("status") == "closed":values.append(value)
    _need(values,"Normal close receipt required")
    value=values[-1];cleanup=value.get("cleanup",{})
    expected_fault = snapshot.get('parent_kind') in ('zero_tx_freshness_fault','postsend_rgb_expiry_fault',
                                                    'completed_unloaded_joint_fault')
    _need(value.get("fault_latched") is expected_fault and cleanup.get("requires_fault_latch") is False
          and cleanup.get("guard_violations") == [] and cleanup.get("unresolved_gripper_probe") is None
          and cleanup.get("grasp_states") == {"left":None,"right":None}
          and all(cleanup.get("arms",{}).get(s,{}).get("status") == "disconnected" for s in ("left","right"))
          and _counts(cleanup.get("session_transmission_counts")) == snapshot["session_transmission_counts"],
          "Healthy close must account for the exact retired-owner lifetime")
    return {"source":ref,"receipt":value,"retired_owner":snapshot["retired_owner"],"physical_stop_verified":None}


def _query_closed(path, snapshot):
    raw, ref = _file(path)
    manifest = json.loads(raw)
    _need(set(manifest) == {'schema','failed_query','close_error','closed_status','process_exit'}
          and manifest['schema'] == 'piper_query_fault_exit_evidence_v1', 'Exact query exit evidence manifest required')
    values, refs = {}, {}
    for name in ('failed_query','close_error','closed_status','process_exit'):
        data, refs[name] = _file(manifest[name]); values[name] = json.loads(data)
    event = snapshot['query_fault_event']; run = snapshot['run']; owner = snapshot['retired_owner']
    _need(values['failed_query'] == {'event_id':event['event_id'],'status':'fault','receipt':json.loads(event['receipt_json'])},
          'Actual failed query receipt must match the immutable ledger')
    _need(values['close_error'] == {'ok':False,'error':'PairHostError: Device cleanup failed or attempted an unexpected transmission'},
          'Preserve the actual cleanup error; do not relabel it healthy')
    status, exited = values['closed_status'], values['process_exit']
    _need(status.get('run_id') == run['run_id'] and status.get('owner') == owner
          and status.get('open') is False and status.get('active_event_id') is None
          and status.get('fault_latched') is True and status.get('fault_feedback_read_state') == 'closed'
          and status.get('unresolved_gripper_probe') is None and status.get('grasp_states') == {'left':None,'right':None}
          and status.get('fault_record_error') is None and status.get('grasp_read_error') is None
          and status['ledger']['steps'] == 1 and status['ledger']['pending_event_id'] is None
          and status['ledger']['fault']['id'] == snapshot['scope']['fault_id'], 'Closed, idle same-owner status required')
    _need(exited.get('schema') == 'piper_control_process_exit_v1' and exited.get('run_id') == run['run_id']
          and exited.get('owner') == owner and type(exited.get('exit_code')) is int and exited['exit_code'] == 0
          and isinstance(exited.get('source_ref'),str) and exited['source_ref'].strip()
          and _number(exited.get('observed_at'),'process exit observation') >= event['finished_at'],
          'Outer operator must archive actual process exit, never physical stopping')
    return {'source':ref,'evidence_sources':refs,'retired_owner':owner,
            'exit_observed_at':exited['observed_at'],'cleanup_error_preserved':True,
            'resource_closed_status_observed':True,'physical_stop_verified':None}


def _round_task_contract(root, snapshot, task_file=None):
    contract = _current_contract(root,snapshot['effective_contract'])
    if task_file is None:
        return contract,None
    from .pair_task_enrollment import _task
    task = _task(task_file,root)
    desired, old = task['envelope']['task'], snapshot['effective_contract']['task']
    _need(desired.get('task_id') == old.get('task_id') and desired.get('roles') == old.get('roles')
          and desired.get('site_context') == old.get('site_context')
          and {'worker_arm','support_arm'} <= set(desired),
          'Recovery may explicitly remap roles, not replace the task or its current site statements')
    contract['task'] = copy.deepcopy(desired)
    return contract,task


def _completed_unloaded_observations(evidence, snapshot, contract, now, recovery_origin='completed_target'):
    """Fresh stationary recovery evidence; never success of the failed action."""
    from .pair_task_enrollment import _observations
    from .feedback_tolerance import joint_tolerances, task_policy
    _need(type(evidence) is dict and set(evidence) == {'passive_paths','rgb_observation','visual_observation'},
          'New independent stationary pair samples and empty-jaw RGB required')
    terminal = snapshot['terminal_unloaded_dispatch']; arm,peer = terminal['arm'],terminal['peer']
    _need(recovery_origin in ('completed_target','arm_cycle_current_pose'), 'Unknown recovery origin')
    startup = None
    after = snapshot['last_finished_at']
    if recovery_origin == 'arm_cycle_current_pose':
        startup = snapshot.get('post_fault_startup')
        _need(type(startup) is dict and startup['arm'] == arm and startup['finished_at'] > after,
              'The failed arm needs its own audited completed post-fault power-cycle startup')
        after = startup['finished_at']
        # Model bounds qualify only this preparation observation. Controller
        # limits and target caches must be acquired on the NEW live connection.
        from .model_compatibility import load_model_catalog, KNOWN_OFFICIAL_CONSTANTS
        from .joint_path import SDK_COMMIT
        source = {'commit':SDK_COMMIT,
            'source_url':'https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/'+SDK_COMMIT+'/pyAgxArm/api/constants.py',
            'constants_path':str(Path(__file__).parents[1]/'data/piper_x_official/sdk_constants.py'),
            'sha256':KNOWN_OFFICIAL_CONSTANTS[SDK_COMMIT]}
        models,_ = load_model_catalog(source)
        _need(all(contract['arms'][s]['model'] == 'piper_x' for s in ('left','right')),
              'Current-pose startup admission is limited to confirmed PiPER X profiles')
    result = _observations(**evidence,contract=contract,after=after,now=now)
    target = [v*math.pi/180000 for v in terminal['target_raw']]
    for side,path in evidence['passive_paths'].items():
        raw,_ = _file(path); data = json.loads(raw)
        reference = target if side == arm else terminal['peer_origin_joints_rad']
        tolerances = (terminal['settle_tolerances_rad'] if side == arm else
                      joint_tolerances(task_policy(contract['task']),side))
        for sample in data['pose_trace']:
            q = [sample['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            limits = (models['piper_x']['joint_limits_rad'] if startup and side == arm
                      else terminal['joint_limits_rad'][side])
            _need(all(lo <= value <= hi for value,(lo,hi) in zip(q,limits)),
                  'New preparation feedback lies outside its applicable absolute joint limits')
            _need((startup is not None and side == arm) or
                  all(abs(value-want) <= tolerance for value,want,tolerance in zip(q,reference,tolerances)),
                  'New stable feedback has not settled near the complete target and unchanged peer')
    if startup is not None:
        return {**result,'recovery_origin':recovery_origin,'startup':startup,
                'current_target_proximity_observed':False,'current_pose_is_new_preparation_origin':True,
                'controller_limits_pending_live_query':True,'original_failed_result_preserved':True,
                'physical_stop_verified':None,'new_target_replay_authorized':False}
    return {**result,'current_target_proximity_observed':True,'original_failed_result_preserved':True,
            'physical_stop_verified':None,'new_target_replay_authorized':False}


def prepare_round(path, parent_run_id, *, close_log, new_run_id, started_at,
                  max_steps=500, max_duration_s=3600, budget_start_policy="include_repair_time",
                  parent_kind='clean', recovery_evidence=None, task_file=None,
                  recovery_origin='completed_target', clock=time.time):
    _need(recovery_origin in ('completed_target','arm_cycle_current_pose') and
          (recovery_origin == 'completed_target' or parent_kind == 'completed_unloaded_joint_fault'),
          'Current-pose recovery belongs only to the audited unloaded complete-send fault')
    parent_run_id=_identifier(parent_run_id,"parent run");new_run_id=_identifier(new_run_id,"new run")
    _need(parent_run_id != new_run_id,"New round requires a distinct run ID")
    _need(budget_start_policy in ("include_repair_time", "after_repair_before_online_execution", "preserve_parent_deadline"),
          "Unknown new-round time policy")
    _need(type(max_steps) is int and 1 <= max_steps <= 1000,"New round maximum is 1000 steps")
    duration=_number(max_duration_s,"duration",positive=True);start=_number(started_at,"started_at")
    _need(duration <= 10800,"New round maximum is 10800 seconds")
    now=_number(clock(),"clock");deadline=_number(start+duration,"deadline")
    source=Path(path).resolve(strict=True)
    db=sqlite3.connect(source.as_uri()+"?mode=ro",uri=True);db.row_factory=sqlite3.Row
    try:
        db.execute("PRAGMA query_only=ON");db.execute("BEGIN")
        snap=_snapshot(db,parent_run_id,parent_kind)
        _need(not db.execute("SELECT 1 FROM pair_runs WHERE run_id=?",(new_run_id,)).fetchone(),"New round already exists")
    finally:db.close()
    prior=snap["run"]
    close = _closed(close_log,snap)
    if parent_kind == 'zero_tx_freshness_fault':
        _need(budget_start_policy == 'preserve_parent_deadline'
              and max_steps == prior['max_steps']-prior['steps']
              and deadline == prior['started_at']+prior['max_duration'],
              'Freshness repair must preserve every unused step and the original deadline')
        not_before = snap['scope']['last_time']
    elif parent_kind == 'query_duplicate_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution'
              and max_steps == prior['max_steps']-prior['steps'] and duration == prior['max_duration'],
              'Explicit timing restart preserves the unused dispatch allowance')
        not_before = max(snap['scope']['last_time'], close['exit_observed_at'])
    elif parent_kind == 'initial_rx_zero_tx_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution',
              'Initial RX repair needs an explicitly authorized new round after repair')
        not_before = max(snap['scope']['last_time'], close['exit_observed_at'])
    elif parent_kind == 'postsend_rgb_expiry_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution',
              'RGB-expiry recovery requires a separately authorized new round after repair')
        not_before = max(prior['started_at']+prior['max_duration'],snap['scope']['last_time'])
    elif parent_kind == 'completed_unloaded_joint_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution' and task_file is not None,
              'Complete-send recovery requires an explicit new budget and frozen task roles')
        not_before = max(prior['started_at']+prior['max_duration'],snap['scope']['last_time'])
    elif parent_kind == 'supported_contact_zero_tx_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution','Contact recovery needs an explicit new budget after repair')
        not_before=max(prior['started_at']+prior['max_duration'],snap['scope']['last_time'],close['exit_observed_at'])
    elif parent_kind == 'configuration_maintenance_fault':
        _need(budget_start_policy == 'after_repair_before_online_execution',
              'Configuration recovery requires the explicit new-task timing authorization')
        not_before = max(snap['scope']['last_time'], snap['last_finished_at'])
    else:
        _need(budget_start_policy != 'preserve_parent_deadline', 'Deadline preservation is a distinct repair entry')
        not_before=max(prior["started_at"]+prior["max_duration"],snap["scope"]["last_time"])
    if recovery_origin == 'arm_cycle_current_pose':
        from .reboot_startup import boot_identity
        startup = snap.get('post_fault_startup')
        _need(type(startup) is dict and startup['boot'] == boot_identity(),
              'An audited completed startup in the current host boot is required')
        not_before = max(not_before,startup['finished_at'])
    _need(not_before <= start <= now < deadline,"Expired parent, fixed begun new window and unexpired new deadline required")
    _need(task_file is None or parent_kind == 'completed_unloaded_joint_fault',
          'Task remapping belongs only to its explicit supervised recovery entry')
    contract,task = _round_task_contract(source.parent.parent,snap,task_file)
    proposal={"schema":"piper_explicit_clean_round_v1","database":str(source),"created_at":now,
        "parent_run_id":parent_run_id,"new_run_id":new_run_id,"snapshot":snap,"snapshot_sha256":_sha(snap),
        "parent_run":prior,"close":close,"authorization_not_before":not_before,
        "new_budget":{"max_steps":max_steps,"max_duration_s":duration,"started_at":start},"deadline_s":deadline,
        "budget_policy":"explicit_user_new_round","budget_start_policy":budget_start_policy,
        "cumulative_step_ceiling":snap["cumulative_prior_steps"]+max_steps,
        "reviewed_contract":contract,
        "hardware_commands_sent":0,"dispatch_authorized":False,"cache_or_limits_transferred":False,
        "physical_stop_verified":None,"fresh_host_admission_required":True}
    if parent_kind != 'clean':
        proposal['parent_kind'] = parent_kind
    if parent_kind == 'query_duplicate_fault':
        _need(all(proposal['reviewed_contract']['code'][name] != snap['effective_contract']['code'][name]
                  for name in ('pair_limits.py','joint_sources.py')), 'Query receiver and evidence reader repair required')
    if parent_kind == 'completed_unloaded_joint_fault':
        proposal['task'] = task
        proposal['required_connection_mode'] = 'prepare'
        if recovery_origin != 'completed_target':
            proposal['recovery_origin'] = recovery_origin
        proposal['recovery_evidence'] = _completed_unloaded_observations(recovery_evidence,snap,contract,now,recovery_origin)
    elif parent_kind == 'supported_contact_zero_tx_fault':
        from .supported_gripper_recovery import contact_zero_tx_observations, CONTACT_ROUND_REPAIR_FILES
        proposal['required_connection_mode']='prepare'
        proposal['contact_route']=snap['contact_attempts']
        old_code,new_code=snap['effective_contract']['code'],contract['code']
        _need(set(old_code)==set(new_code) and {k for k,v in new_code.items() if old_code[k]!=v}<=CONTACT_ROUND_REPAIR_FILES,
              'Only reviewed contact renewal/RGB reserve sources may change')
        proposal['recovery_evidence']=contact_zero_tx_observations(recovery_evidence,snap,contract,now,close['exit_observed_at'])
    elif parent_kind == 'configuration_maintenance_fault':
        from .configuration_recovery import observations
        proposal['required_connection_mode'] = 'prepare'
        _need('controller_limits.py' in contract['code'] and 'configuration_recovery.py' in contract['code'],
              'Maintenance runtime and audited recovery must be frozen')
        proposal['recovery_evidence'] = observations(recovery_evidence,snap,contract,now)
    elif parent_kind == 'initial_rx_zero_tx_fault':
        proposal['recovery_evidence'] = _initial_rx_observations(recovery_evidence, snap, now)
        old_code, new_code = snap['effective_contract']['code'], proposal['reviewed_contract']['code']
        changed = {name for name in old_code if old_code[name] != new_code.get(name)}
        _need(set(old_code) == set(new_code)
              and {'pair_joint_adapter.py','pair_round.py','pair_ledger.py'} <= changed
              <= {'pair_joint_adapter.py','pair_round.py','pair_ledger.py','pair_host.py'},
              'Only the reviewed initial RX runtime/round/prepare-entry repair is covered')
    elif parent_kind in ('zero_tx_freshness_fault','postsend_rgb_expiry_fault'):
        from .pair_continuation import _observations
        _need(type(recovery_evidence) is dict and set(recovery_evidence) ==
              {'passive_paths','rgb_observation','visual_observation'}, 'Archived post-failure observations required')
        proposal['recovery_evidence'] = _observations(**recovery_evidence,
            contract=snap['effective_contract'], failed_at=snap['last_finished_at'], now=now,
            orientation_metric='so3_diameter' if parent_kind == 'postsend_rgb_expiry_fault' else 'euler_span_bound')
        repaired = ('joint_path.py','pair_joint_adapter.py') if parent_kind == 'zero_tx_freshness_fault' else (
            'pair_host.py','pair_joint_adapter.py')
        _need(all(proposal['reviewed_contract']['code'][name] != snap['effective_contract']['code'][name]
                  for name in repaired), 'Diagnosed runtime repair required')
    else:
        _need(recovery_evidence is None, 'Recovery evidence belongs only to its diagnosed entry')
    return {**proposal,"proposal_sha256":_sha(proposal)}


def audit_unopened_enrollment(db, scope, run):
    return _audit_round_enrollment(db,scope,run,parent_kind='initial_rx_zero_tx_fault')


def audit_completed_unloaded_round(db, scope, run):
    return _audit_round_enrollment(db,scope,run,parent_kind='completed_unloaded_joint_fault')


def _audit_round_enrollment(db, scope, run, *, parent_kind):
    """Recheck original enrollment without changing the authoritative ledger.

    The caller checks whether this child may open or receive a code revision.
    This function also works after the child has opened: only rows belonging to
    this exact child are isolated in an in-memory historical replay. Every
    other row must still match the original parent audit. Archived observations
    are checked at registration time, never promoted to current admission.
    """
    record = json.loads(scope['record_json'])
    proposal, auth = record['proposal'], record['authorization']
    required_auth = {'source','message_id','statement','received_at','decision',
                     'proposal_sha256','new_budget','budget_start_policy'}
    _need(proposal.get('schema') == 'piper_explicit_clean_round_v1'
          and proposal.get('parent_kind') == parent_kind
          and scope['run_id'] == run['run_id'] == proposal['new_run_id']
          and scope['parent_run_id'] == proposal['parent_run_id']
          and proposal['parent_run_id'] != run['run_id']
          and _sha({k:v for k,v in proposal.items() if k != 'proposal_sha256'})
              == proposal['proposal_sha256'] == scope['proposal_sha256']
          and type(auth) is dict and set(auth) == required_auth
          and _sha(auth) == scope['authorization_sha256']
          and auth['source'] == 'user_message' and auth['decision'] == 'authorize_explicit_new_round'
          and auth['proposal_sha256'] == proposal['proposal_sha256']
          and auth['new_budget'] == proposal['new_budget']
          and auth['budget_start_policy'] == proposal['budget_start_policy'] == 'after_repair_before_online_execution'
          and record['new_contract'] == proposal['reviewed_contract'],
          'Original exact enrollment and user authorization required')
    _identifier(auth['message_id'],'original user message')
    _need(type(auth['statement']) is str and 1 <= len(auth['statement'].strip()) <= 8000,
          'Original actual user statement required')
    at = _number(auth['received_at'],'original authorization time')
    created = _number(proposal['created_at'],'original proposal time')
    activated = _number(record['activated_at'],'original activation time')
    started = _number(run['started_at'],'original budget start')
    duration = _number(run['max_duration'],'original budget duration',positive=True)
    _need(type(run['max_steps']) is int and 1 <= run['max_steps'] <= 1000
          and duration <= 10800
          and proposal['new_budget'] == {'started_at':started,'max_steps':run['max_steps'],'max_duration_s':duration}
          and proposal['deadline_s'] == started+duration
          and proposal['authorization_not_before'] <= at <= started <= created <= activated < started+duration
          and activated <= scope['last_time']
          and record.get('old_rows_preserved') is True and record.get('new_budget_allocated') is True
          and type(record.get('hardware_commands_sent')) is int and record['hardware_commands_sent'] == 0
          and record.get('cache_or_limits_transferred') is False
          and record.get('dispatch_authorized') is False and record.get('physical_stop_verified') is None
          and record.get('fresh_host_admission_required') is True
          and proposal.get('hardware_commands_sent') == 0 and proposal.get('dispatch_authorized') is False
          and proposal.get('cache_or_limits_transferred') is False
          and proposal.get('physical_stop_verified') is None and proposal.get('fresh_host_admission_required') is True,
          'Original budget, chronology and zero-dispatch registration facts required')
    shadow = sqlite3.connect(':memory:'); shadow.row_factory = sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()))
        # A later independently recorded startup is not part of the old
        # parent's enrollment. Verify it against the full current history
        # before projecting it out of this in-memory historical replay only.
        # This does not remove the task fault or admit a changed current pose.
        from .arm_power_cycle import audit_completed_startup_history
        if 'pair_arm_power_cycles' not in proposal['snapshot']['table_rows']:
            startup = audit_completed_startup_history(shadow)
            if startup is not None:
                shadow.execute('DROP TABLE pair_arm_power_cycles')
        names = [row[0] for row in shadow.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'")]
        rgb_table = 'pair_round_rgb_continuations'
        if rgb_table in names:
            # Its audit is performed by activated_execution_budget. Projection
            # removes only this child's append-only administrative enrollment.
            if rgb_table not in proposal['snapshot']['table_rows']:
                _need(not shadow.execute('SELECT 1 FROM '+rgb_table+' WHERE run_id != ?', (run['run_id'],)).fetchone(),
                      'Another round RGB successor is outside the original parent history')
                shadow.execute('DROP TABLE '+rgb_table)
            else:
                shadow.execute('DELETE FROM '+rgb_table+' WHERE run_id=?',(run['run_id'],))
        # Only these child runtime rows may have accumulated after registration.
        child_tables = {'pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests',
                        'pair_hold_frames','pair_grasp_episodes'}
        for name in child_tables & set(names):
            shadow.execute('DELETE FROM '+name+' WHERE run_id=?',(run['run_id'],))
        if 'pair_unopened_round_revisions' in names:
            if 'pair_unopened_round_revisions' not in proposal['snapshot']['table_rows']:
                _need(not shadow.execute('SELECT 1 FROM pair_unopened_round_revisions WHERE run_id != ?',
                                         (run['run_id'],)).fetchone(),
                      'Another run revision is not part of the original parent history')
                shadow.execute('DROP TABLE pair_unopened_round_revisions')
            else:
                shadow.execute('DELETE FROM pair_unopened_round_revisions WHERE run_id=?',(run['run_id'],))
        shadow.execute('DELETE FROM pair_rounds WHERE ordinal=? AND run_id=?',(scope['ordinal'],run['run_id']))
        shadow.execute('DELETE FROM pair_runs WHERE run_id=?',(run['run_id'],))
        original = _snapshot(shadow,proposal['parent_run_id'],proposal['parent_kind'])
    finally:
        shadow.close()
    _need(original == proposal['snapshot'] and _sha(original) == proposal['snapshot_sha256']
          and proposal['parent_run'] == original['run']
          and json.loads(scope['retired_owners_json']) == original['retired_owners'],
          'Parent history differs from the original audited registration')
    close = _closed(proposal['close']['source']['path'],original,proposal['close'].get('evidence_sources'))
    not_before = (max(original['scope']['last_time'],close['exit_observed_at'])
                  if parent_kind in ('initial_rx_zero_tx_fault','configuration_maintenance_fault') else
                  max(original['run']['started_at']+original['run']['max_duration'],original['scope']['last_time']))
    if parent_kind == 'supported_contact_zero_tx_fault':
        not_before=max(not_before,close['exit_observed_at'])
    recovery_origin = proposal.get('recovery_origin','completed_target')
    if recovery_origin == 'arm_cycle_current_pose':
        _need(parent_kind == 'completed_unloaded_joint_fault' and original.get('post_fault_startup'),
              'Current-pose enrollment must retain its audited startup')
        not_before = max(not_before,original['post_fault_startup']['finished_at'])
    _need(close == proposal['close']
          and proposal['authorization_not_before'] == not_before
          and proposal['cumulative_step_ceiling'] == original['cumulative_prior_steps']+run['max_steps'],
          'Original closure evidence, authorization boundary or cumulative allowance changed')
    evidence = proposal['recovery_evidence']
    recovery = {'passive_paths':{side:value['source']['path'] for side,value in evidence['passive'].items()},
                'rgb_observation':evidence['rgb']['source']['path'],'visual_observation':evidence['visual_observation']}
    if parent_kind == 'configuration_maintenance_fault':
        recovery['configuration_diagnostic'] = evidence['configuration_diagnostic']['source']['path']
    if parent_kind == 'completed_unloaded_joint_fault':
        from .pair_task_enrollment import _task
        task = _task(proposal['task']['source']['path'],Path(proposal['database']).parent.parent)
        old,new = original['effective_contract'],proposal['reviewed_contract']
        _need(task == proposal['task'] and new['task'] == task['envelope']['task']
              and {k:v for k,v in new.items() if k not in ('task','code')}
                  == {k:v for k,v in old.items() if k not in ('task','code')}
              and new['task']['site_context'] == old['task']['site_context']
              and new['task']['task_id'] == old['task']['task_id']
              and {'worker_arm','support_arm'} <= set(new['task'])
              and record.get('required_connection_mode') == proposal.get('required_connection_mode') == 'prepare',
              'Frozen role/recipe binding and preparation-only scope required')
        observed = _completed_unloaded_observations(recovery,original,new,created,recovery_origin)
    elif parent_kind == 'supported_contact_zero_tx_fault':
        from .supported_gripper_recovery import contact_zero_tx_observations, contact_round_proposal, CONTACT_ROUND_REPAIR_FILES
        old,new=original['effective_contract'],proposal['reviewed_contract']
        _need(record.get('required_connection_mode')==proposal.get('required_connection_mode')=='prepare'
              and {k:v for k,v in old.items() if k!='code'}=={k:v for k,v in new.items() if k!='code'}
              and set(old['code'])==set(new['code'])
              and {k for k,v in new['code'].items() if old['code'][k]!=v}<=CONTACT_ROUND_REPAIR_FILES,
              'Contact round must retain original roles/task and only reviewed source repair')
        contact_round_proposal(proposal)
        observed=contact_zero_tx_observations(recovery,original,new,created,close['exit_observed_at'])
    elif parent_kind == 'configuration_maintenance_fault':
        from .configuration_recovery import observations
        _need(record.get('required_connection_mode') == proposal.get('required_connection_mode') == 'prepare'
              and {k:v for k,v in original['effective_contract'].items() if k != 'code'}
                  == {k:v for k,v in proposal['reviewed_contract'].items() if k != 'code'},
              'Configuration recovery must preserve task, roles, devices and preparation-only entry')
        observed = observations(recovery,original,proposal['reviewed_contract'],created)
    else:
        observed = _initial_rx_observations(recovery,original,created)
    _need(observed == evidence,
          'Archived admission evidence changed; it cannot supply current feedback')
    return {'original_snapshot_sha256':proposal['snapshot_sha256'],
            'original_proposal_sha256':proposal['proposal_sha256'],
            'historical_observations_only':True,'fresh_host_admission_required':True}


def activate_round(proposal, authorization, *, project_root, clock=time.time):
    _need(_sha({k:v for k,v in proposal.items() if k != "proposal_sha256"}) == proposal.get("proposal_sha256"),"Proposal digest mismatch")
    budget=proposal["new_budget"]
    evidence = proposal.get('recovery_evidence')
    recovery = None if evidence is None else {
        'passive_paths':{s:v['source']['path'] for s,v in evidence['passive'].items()},
        'rgb_observation':evidence['rgb']['source']['path'], 'visual_observation':evidence['visual_observation']}
    if proposal.get('parent_kind') == 'configuration_maintenance_fault':
        recovery['configuration_diagnostic'] = evidence['configuration_diagnostic']['source']['path']
    canonical=prepare_round(proposal["database"],proposal["parent_run_id"],close_log=proposal["close"]["source"]["path"],
        new_run_id=proposal["new_run_id"],clock=lambda:proposal["created_at"],
        budget_start_policy=proposal["budget_start_policy"],parent_kind=proposal.get('parent_kind','clean'),
        recovery_evidence=recovery,task_file=proposal.get('task',{}).get('source',{}).get('path'),
        recovery_origin=proposal.get('recovery_origin','completed_target'),**budget)
    _need(canonical == proposal,"Canonical unchanged proposal required")
    required={"source","message_id","statement","received_at","decision","proposal_sha256","new_budget"}
    preserved = proposal['budget_start_policy'] == 'preserve_parent_deadline'
    after_repair=proposal["budget_start_policy"] in ("after_repair_before_online_execution", 'preserve_parent_deadline')
    if after_repair: required.add("budget_start_policy")
    _need(type(authorization) is dict and set(authorization)==required
          and authorization["source"]=="user_message" and authorization["decision"]==
              ('authorize_repaired_continuation' if preserved else 'authorize_explicit_new_round')
          and authorization["proposal_sha256"]==proposal["proposal_sha256"]
          and (not after_repair or authorization["budget_start_policy"] == proposal["budget_start_policy"])
          and _json_object(authorization["new_budget"],"authorized budget")==_json_object(budget,"budget"),
          "Exact new user-message budget authorization required")
    _identifier(authorization["message_id"],"user message reference")
    _need(type(authorization["statement"]) is str and 1 <= len(authorization["statement"].strip()) <= 8000,
          "Actual user statement required")
    at=_number(authorization["received_at"],"authorization time");now=_number(clock(),"clock")
    _need(proposal["authorization_not_before"] <= at <= budget["started_at"]
          and (after_repair or at == budget["started_at"])
          and proposal["created_at"] <= now < proposal["deadline_s"],"New authorization/window chronology differs")
    root=Path(project_root).resolve(strict=True);path=Path(proposal["database"])
    _need(path==root/"runs/pair_sessions.sqlite","Authoritative project database required")
    _need(not _live_control_processes(),"Live control process blocks enrollment")
    with ExclusiveExecution(path.parent):
        db=sqlite3.connect(path.as_uri()+"?mode=rw",uri=True,isolation_level=None);db.row_factory=sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL");db.execute("BEGIN IMMEDIATE")
            snap=_snapshot(db,proposal["parent_run_id"],proposal.get('parent_kind','clean'))
            _need(_sha(snap)==proposal["snapshot_sha256"],"Old history changed after proposal")
            _need(_closed(proposal["close"]["source"]["path"],snap)==proposal["close"],"Close evidence changed")
            task_path = proposal.get('task',{}).get('source',{}).get('path')
            contract,task = _round_task_contract(root,snap,task_path)
            _need(contract==proposal["reviewed_contract"] and task==proposal.get('task'),"Reviewed code/task changed")
            if recovery is not None:
                if proposal.get('parent_kind') == 'completed_unloaded_joint_fault':
                    if proposal.get('recovery_origin') == 'arm_cycle_current_pose':
                        from .reboot_startup import boot_identity
                        _need(snap['post_fault_startup']['boot'] == boot_identity(), 'Host boot changed after proposal')
                    observed = _completed_unloaded_observations(recovery,snap,contract,now,
                                                               proposal.get('recovery_origin','completed_target'))
                elif proposal.get('parent_kind') == 'initial_rx_zero_tx_fault':
                    observed = _initial_rx_observations(recovery, snap, now)
                elif proposal.get('parent_kind') == 'supported_contact_zero_tx_fault':
                    from .supported_gripper_recovery import contact_zero_tx_observations
                    observed=contact_zero_tx_observations(recovery,snap,contract,now,proposal['close']['exit_observed_at'])
                elif proposal.get('parent_kind') == 'configuration_maintenance_fault':
                    from .configuration_recovery import observations
                    observed = observations(recovery, snap, contract, now)
                else:
                    from .pair_continuation import _observations
                    observed = _observations(**recovery, contract=snap['effective_contract'],
                        failed_at=snap['last_finished_at'],now=now,
                        orientation_metric='so3_diameter' if proposal.get('parent_kind') == 'postsend_rgb_expiry_fault'
                            else 'euler_span_bound')
                _need(observed == evidence, 'Recovery evidence changed')
            _need(not _live_control_processes(),"Control process appeared during enrollment")
            db.execute("CREATE TABLE IF NOT EXISTS pair_rounds (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,"
                "parent_run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,"
                "retired_owners_json TEXT NOT NULL,proposal_sha256 TEXT UNIQUE NOT NULL,authorization_sha256 TEXT UNIQUE NOT NULL,"
                "record_json TEXT NOT NULL)")
            ordinal=db.execute("SELECT COALESCE(MAX(ordinal),0)+1 FROM pair_rounds").fetchone()[0]
            record={"proposal":proposal,"authorization":authorization,"activated_at":now,
                "new_contract":proposal["reviewed_contract"],"old_rows_preserved":True,"new_budget_allocated":not preserved,
                "hardware_commands_sent":0,"dispatch_authorized":False,"physical_stop_verified":None,
                "cache_or_limits_transferred":False,"fresh_host_admission_required":True}
            if proposal.get('parent_kind') in ('completed_unloaded_joint_fault','configuration_maintenance_fault','supported_contact_zero_tx_fault'):
                record['required_connection_mode'] = 'prepare'
            db.execute("INSERT INTO pair_runs VALUES(?,?,?,?,?,0)",(proposal["new_run_id"],
                _json_object(record["new_contract"],"new contract"),budget["max_steps"],budget["max_duration_s"],budget["started_at"]))
            db.execute("INSERT INTO pair_rounds VALUES(?,?,?,NULL,NULL,NULL,?,?,?,?,?)",(ordinal,proposal["new_run_id"],
                proposal["parent_run_id"],now,json.dumps(snap["retired_owners"]),proposal["proposal_sha256"],_sha(authorization),_json_object(record,"new round")))
            _need(_round_task_contract(root,snap,task_path)==(record['new_contract'],task),"Code/task changed during enrollment")
            final=_number(clock(),"final clock")
            _need(now <= final < proposal["deadline_s"],"Clock rollback or new deadline reached")
            record["activated_at"]=final
            db.execute("UPDATE pair_rounds SET last_time=?,record_json=? WHERE ordinal=?",(final,_json_object(record,"new round"),ordinal))
            check=_number(clock(),"commit clock")
            _need(final <= check < proposal["deadline_s"],"Clock/deadline changed during durable write")
            record["activated_at"]=check
            db.execute("UPDATE pair_rounds SET last_time=?,record_json=? WHERE ordinal=?",(check,_json_object(record,"new round"),ordinal))
            db.execute("COMMIT");return record
        except BaseException:
            if db.in_transaction:db.execute("ROLLBACK")
            raise
        finally:db.close()
