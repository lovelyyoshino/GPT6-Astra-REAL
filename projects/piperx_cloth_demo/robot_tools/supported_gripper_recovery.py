"""Once-only supported mechanical opening after a completed jaw frame whose
contact classification succeeded but final feedback freshness failed.

No grasp/cache transfer and no replay. A new owner is restricted by the ledger
to one bounded opening, then fresh visual/feedback separation confirmation,
before ordinary preparation. A separately audited contact route permits at most
three new left closures under original support, then zero-TX retention only.
Original failure, history and budget are retained.
"""
from contextlib import ExitStack
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_restart import _need, _sha, _file, _current_contract
from .pair_ledger import _json_object, _number, _identifier, _activated_execution_budget_db
from .pair_task_enrollment import _observations, _lock_roots
from .preparation_continuation import _table_sha
from .reboot_startup import check_processes
from .retention_receipt import digest as _receipt_digest

TABLE = 'pair_supported_gripper_recoveries'
OPENING_TABLE = 'pair_supported_gripper_opening_continuations'
OPENING_SCHEMA = 'piper_supported_opening_continuation_v1'
CONTACT_TABLE = 'pair_supported_contact_reacquisitions'
CONTACT_SCHEMA = 'piper_supported_contact_reacquisition_v1'
SCHEMA = 'piper_supported_gripper_recovery_v1'
REPAIR_FILES = {'pair_host.py', 'pair_ledger.py', 'pair_device.py', 'service.py',
                'supported_gripper_recovery.py', 'host_recovery.py'}
REASON = 'No distinguishable closing-and-settling response or no unambiguous width outcome'
SIDES = ('left', 'right')


def tables(db):
    result = {}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_','').isalnum(), 'Unexpected table name')
        result[name] = [dict(r) for r in db.execute('SELECT * FROM '+name)]
    return result


def validate_failed_probe(event, run, owner):
    import re
    p,r=json.loads(event['payload_json']),json.loads(event['receipt_json'])
    d=r.get('device_receipt',{});c=d.get('contact_observation',{})
    _need(event['run_id']==run['run_id'] and event['owner']==owner and event['step']==run['steps']
          and event['status']=='complete' and event['success']==0
          and hashlib.sha256(event['payload_json'].encode()).hexdigest()==event['payload_digest']
          and set(p)=={'arm','kind','operation','target','grasp_object_id','observation_id','peer_receipt_id'}
          and p['kind']=='gripper' and p['operation']=='grip_supported' and p['arm'] in SIDES
          and r.get('event_id')==event['event_id'] and r.get('ok') is False
          and r.get('automatic_retry') is False, 'Exact final failed supported probe required')
    _identifier(p['grasp_object_id'],'original object')
    errors=d.get('errors',[])
    _need(len(errors)==1 and errors[0].get('type')=='RuntimeError', 'Only final freshness failure is covered')
    match=re.fullmatch(r'Single action requires receive age/skew within 100 ms including processing; age=([0-9]+[.][0-9]+) skew=([0-9]+[.][0-9]+)',errors[0].get('detail',''))
    _need(match is not None and max(map(float,match.groups()))>.1, 'Only actual final age/skew excess is covered')
    arm=p['arm'];counts={s:dict(attempted_frames=int(s==arm),sent_frames=int(s==arm),blocked_frames=0) for s in SIDES}
    _need(d.get('transmission_counts')==counts and d.get('hardware_commands_sent')==1
          and d.get('target_calls_sent')==1 and d.get('nominal_force_N')==.2
          and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('guard_violations')==[] and d.get('ok') is False
          and c.get('outcome')=='settled_contact_candidate' and c.get('completion')=='observation_only'
          and c.get('observation_window_complete') is True and c.get('requested_width_m')==p['target']
          and c.get('target_may_remain_active') is True
          and d.get('unresolved_gripper_probe') is None and d.get('grasp_states')==dict.fromkeys(SIDES),
          'Only one fully returned frame and classified supported candidate are covered')
    probe=d.get('candidate_probe',{});m=d.get('candidate_measurement',{})
    _need(probe.get('requested_width_m')==p['target'] and probe.get('outcome')==c['outcome']
          and probe.get('observed_width_m')==m.get('observed',{}).get('width_m')
          and probe.get('trace_sha256')==c.get('trace_summary',{}).get('sha256')
          and m.get('identity') is None and m.get('probe_event_id') is None,
          'Original anonymous candidate evidence required')
    return p,d


def snapshot(db, run_id):
    all_rows = tables(db)
    _need(TABLE not in all_rows, 'This supported recovery has already been consumed')
    scopes = all_rows.get('pair_manual_gripper_continuations',[])
    _need(len(scopes)==1, 'One original manual continuation required')
    scope=scopes[0]; run=next((r for r in all_rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id']
          and max(all_rows['pair_rounds'],key=lambda r:r['ordinal'])['ordinal']==scope['round_ordinal'],
          'Current faulted parent round/owner required')
    _need(_activated_execution_budget_db(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Full original lineage and budget audit required')
    owner=scope['owner']; events=sorted([r for r in all_rows['pair_events'] if r['run_id']==run_id and r['owner']==owner],key=lambda r:r['step'])
    _need(not any(r['status']=='pending' for r in all_rows['pair_events']), 'Pending send blocks continuation')
    _need(len(events)==4 and [json.loads(e['payload_json'])['kind'] for e in events]==['query','initialization','initialization','gripper'],
          'Only a query, both current initializations and one terminal probe are covered')
    prior=json.loads(scope['record_json'])['proposal']['budget']['steps']
    _need([e['step'] for e in events]==list(range(prior+1,prior+5)) and run['steps']==prior+4, 'Contiguous original steps required')
    totals={s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in SIDES}; initialized=set()
    for e in events[:-1]:
        p=json.loads(e['payload_json']);r=json.loads(e['receipt_json'])
        _need(hashlib.sha256(e['payload_json'].encode()).hexdigest()==e['payload_digest']
              and e['status']=='complete' and e['success']==1 and r.get('ok') is True
              and r.get('errors')==r.get('guard_violations')==[] and r.get('event_id')==e['event_id']
              and r.get('pair_owner')==owner and r.get('fault_latched') is False, 'Uncertain preparation history')
        if p['kind']=='query':
            from .joint_sources import _validate_bindings, _validated_limits
            contract=json.loads(scope['contract_json']);_validate_bindings(contract,p['bindings'])
            _validated_limits({**r,'run_id':run_id,'owner':owner,'bindings':p['bindings']},run_id=run_id,owner=owner,bindings=p['bindings'],now=e['finished_at'])
            expected={s:dict(attempted_frames=6,sent_frames=6,blocked_frames=0) for s in SIDES}
            _need(r.get('actuator_commands_sent')==0 and p['request']=={'operation':'inspect_joint_limits'}, 'Only nonactuating query allowed')
        else:
            from .pair_round import _frames
            arm=r.get('arm');_need(arm in SIDES and arm not in initialized, 'Distinct current initialization required');initialized.add(arm)
            plan=r['initialization_plan'];_frames(r['frame_receipts'],plan['target_raw'],e)
            _need(r.get('arrival_confirmed') is True and r.get('observed_stable') is True and r.get('cache_established') is True
                  and r.get('gripper_commands_sent')==0 and r.get('enable_commands_sent')==r.get('stop_commands_sent')==0,
                  'Completed unchanged-jaw initialization required')
            expected={s:dict(attempted_frames=4*int(s==arm),sent_frames=4*int(s==arm),blocked_frames=0) for s in SIDES}
        _need(r['transmission_counts']==expected,'Unexpected preparation sends')
        for s in SIDES:
            for k in totals[s]:totals[s][k]+=expected[s][k]
        _need(r['session_transmission_counts']==totals,'Unrecorded preparation sends')
    event=events[-1];p,d=validate_failed_probe(event,run,owner)
    for s in SIDES:
        for k in totals[s]:totals[s][k]+=d['transmission_counts'][s][k]
    _need(d['session_transmission_counts']==totals,'Unrecorded lifetime sends')
    faults=[r for r in all_rows['pair_faults'] if r['run_id']==run_id and r['owner']==owner]
    _need(len(faults)==2 and {r['reason'] for r in faults}=={
          'Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch','execution_receipt_failed'}
          and min(r['id'] for r in faults)==scope['fault_id'] and all(r['at']>=event['began_at'] for r in faults),
          'Only the original unconfirmed-probe fault is covered')
    episodes=[r for r in all_rows.get('pair_grasp_episodes',[]) if r['run_id']==run_id and r['owner']==owner]
    _need(len(episodes)==1,'Exactly one empty probe bookkeeping episode required')
    episode=episodes[0];state=json.loads(episode['state_json'])
    _need(episode['owner']==owner and episode['revision']==state['revision']==0 and state['status']=='empty'
          and state['events']=={} and state['identity']['object_id']==p['grasp_object_id']
          and state['identity']['arm']==p['arm'] and all(state[k] is None for k in
          ('measurement','original_anchor','pending','probe_ref','residual_target','retention_contract','visual_evidence')),
          'Existing candidate or held object cannot be reclassified as empty')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends'):
        _need(not any(r['run_id']==run_id and r.get('owner')==owner for r in all_rows.get(name,[])), 'Hold or uncertain send history blocks this entry')
    retired=sorted({r['owner'] for rows in all_rows.values() for r in rows if r.get('owner')})
    return dict(run=run,scope=scope,event=event,contract=json.loads(scope['contract_json']),retired_owners=retired,
                table_sha256={k:_table_sha(v) for k,v in all_rows.items()})


def history(session_log, probe_journal, s):
    raw,ref=_file(session_log);rows=[json.loads(l) for l in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows)
          and rows[-1]['kind']=='session_ended' and rows[-1]['cleanup_errors']==[], 'Complete normally ended session required')
    results=[r for r in rows if r['kind']=='result'];owner=s['scope']['owner'];e=s['event']
    _need(any(r['result'].get('owner')==owner and r['result'].get('run_id')==s['run']['run_id'] for r in results), 'Original host binding required')
    close=[r['result'] for r in results if r['result'].get('status')=='closed'];_need(len(close)==1,'One normal close required')
    c=close[0]['cleanup'];d=json.loads(e['receipt_json'])['device_receipt']
    _need(close[0]['fault_latched'] is True and c.get('session_transmission_counts')==d['session_transmission_counts']
          and c.get('guard_violations')==[] and c.get('unresolved_gripper_probe') is None
          and c.get('grasp_states')==dict.fromkeys(SIDES) and c.get('requires_fault_latch') is False
          and all(c['arms'][side]['status']=='disconnected' for side in SIDES)
          and rows[-1]['at']>e['finished_at'], 'Exact detached empty-grasp closure required')
    raw,jref=_file(probe_journal);j=json.loads(raw)['rows']
    _need([r['event'] for r in j]==['pair_dispatch_claimed','single_supervised_action_intent','single_supervised_action_sent_unconfirmed','bounded_probe_trace'], 'Actual original claim/intent/return/trace required')
    p=json.loads(e['payload_json']);intent,sent,tr=j[1:]
    expected=round(p['target']*1e6).to_bytes(4,'big',signed=True)+bytes.fromhex('00c80100')
    _need(j[0]['event_id']==e['event_id'] and j[0]['payload']==p
          and intent['arm']==p['arm'] and intent['kind']=='gripper' and intent['target']==p['target']
          and intent['frames']==[{'id':0x159,'data_hex':expected.hex()}]
          and sent['kind']=='gripper' and tr['arm']==p['arm'] and tr['release'] is False
          and tr['requested_width_m']==p['target'] and tr['sha256']==hashlib.sha256(json.dumps(tr['trace'],sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
          and e['began_at']<=j[0]['unix_s']<=intent['unix_s']<=sent['finished_unix_s']<=sent['unix_s']<=tr['unix_s']<=e['finished_at'], 'Original known single-frame provenance differs')
    from .contact_receipt import classify_gripper_probe
    from .feedback_tolerance import task_policy
    classified=classify_gripper_probe(arm=p['arm'],requested_width_m=p['target'],sent_at=tr['sent_at'],
        baseline_samples=tr['trace']['baseline'],post_samples=tr['trace']['post'],feedback_policy=task_policy(s['contract']['task']))
    _need(classified=={k:v for k,v in d['contact_observation'].items() if k!='trace_summary'}
          and d['contact_observation']['trace_summary']['sha256']==tr['sha256'], 'Original raw response classification must remain failed')
    from .retention_receipt import measured_anchor, summarize_retention_trace
    cutoff=tr['trace']['post'][-1]['observed_at_s']-3.
    start=max(i for i,item in enumerate(tr['trace']['post']) if item['observed_at_s']<=cutoff)
    stable=tr['trace']['post'][start:]
    measured=summarize_retention_trace(arm=p['arm'],identity=None,probe_event_id=None,
        trace_id='probe_'+tr['sha256'][:32],trace_sha256=tr['sha256'],
        original_anchor=measured_anchor(stable[0]['arms'][p['arm']]),samples=stable,
        now=stable[-1]['observed_at_s'],feedback_policy=task_policy(s['contract']['task']))
    _need(measured==d['candidate_measurement'] and d['candidate_probe']['sent_at']==tr['sent_at']
          and d['candidate_probe']['completed_at']==stable[-1]['observed_at_s'],
          'Classified measurement must derive from the unchanged original raw trace')
    return dict(session=ref,probe_journal=jref,ended_at=rows[-1]['at'],physical_stop_verified=None)


def runtime(db,run_id,owner=None):
    """Small read-only phase check; full immutable audit occurs at enrollment."""
    row = None
    if db.execute("SELECT 1 FROM sqlite_master WHERE name=?",(OBSERVATION_TABLE,)).fetchone():
        observed=db.execute('SELECT * FROM '+OBSERVATION_TABLE+' ORDER BY ordinal DESC LIMIT 1').fetchone()
        current=db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if observed is not None and current is not None and observed['round_ordinal']==current['ordinal']:
            _need(observed['run_id']==run_id,'Current contact observation belongs to its exact run')
            if owner is not None:_need(observed['owner']==owner,'Contact observation belongs to its exact new owner')
            proposal=json.loads(observed['record_json'])['proposal']
            events=list(db.execute('SELECT * FROM pair_events WHERE run_id=? AND step>? ORDER BY step',(run_id,proposal['budget']['steps'])))
            return _existing_contact_runtime(observed,proposal,events)
    if db.execute("SELECT 1 FROM sqlite_master WHERE name=?",(CONTACT_TABLE,)).fetchone():
        sent=db.execute('SELECT * FROM '+CONTACT_TABLE+' ORDER BY ordinal DESC LIMIT 1').fetchone()
        current_round=db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if sent is not None and current_round is not None and sent['run_id']==run_id and sent['round_ordinal']==current_round['ordinal']:
            registration=json.loads(sent['record_json'])['proposal']
            if registration.get('schema')==SENT_CONTACT_SCHEMA:
                if owner is not None:_need(sent['owner']==owner,'Sent contact continuation belongs to exact new owner')
                proposal=_sent_contact_runtime_proposal(registration)
                events=list(db.execute('SELECT * FROM pair_events WHERE run_id=? AND step>? ORDER BY step',(run_id,proposal['budget']['steps'])))
                return _contact_runtime(db,run_id,sent,proposal,events)
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_rounds'").fetchone():
        head=db.execute('SELECT * FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and head['run_id']==run_id:
            enrolled=json.loads(head['record_json'])['proposal']
            if enrolled.get('parent_kind')==CONTACT_ZERO_TX_KIND:
                if owner is not None:_need(head['owner']==owner,'Contact round belongs to its exact current owner')
                proposal=contact_round_proposal(enrolled)
                events=list(db.execute('SELECT * FROM pair_events WHERE run_id=? ORDER BY step',(run_id,)))
                return _contact_runtime(db,run_id,head,proposal,events)
    for table in (CONTACT_TABLE, OPENING_TABLE, TABLE):
        if db.execute("SELECT 1 FROM sqlite_master WHERE name=?",(table,)).fetchone():
            row=db.execute('SELECT * FROM '+table+' WHERE run_id=?',(run_id,)).fetchone()
            if row is not None:break
    if row is None:return None
    parent=db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
    if parent is None or parent['ordinal']!=row['round_ordinal']:return None
    if owner is not None:_need(row['owner']==owner,'Recovery belongs to this exact live owner')
    p=json.loads(row['record_json'])['proposal'];s=p['snapshot']
    events=list(db.execute('SELECT * FROM pair_events WHERE run_id=? AND step>? ORDER BY step',
                           (run_id,p['budget']['steps'])))
    if p.get('route')=='audited_contact_reacquisition':return _contact_runtime(db,run_id,row,p,events)
    phase='opening_required'
    for i,expected in enumerate(('supported_recovery_open','supported_recovery_confirm')):
        if len(events)<=i:break
        event=events[i];payload=json.loads(event['payload_json'])
        _need(event['owner']==row['owner'] and payload.get('kind')==expected,
              'Only bounded opening then fresh separation are admitted before preparation')
        if event['status']!='complete' or event['success']!=1:
            return dict(phase='unresolved',proposal=p,events=[dict(e) for e in events])
        receipt=json.loads(event['receipt_json'])
        _need(receipt.get('ok') is True and receipt.get('physical_stop_verified') is None
              and receipt.get('hardware_commands_sent')==(1 if i==0 else 0),
              'Recovery receipt has unexpected transmission count')
        phase='confirmation_required' if i==0 else 'resolved'
    return dict(phase=phase,proposal=p,events=[dict(e) for e in events])


def check_request(db,run_id,owner,payload):
    current=runtime(db,run_id,owner)
    if current is None:return
    if current['proposal'].get('route')==OBSERVATION_ROUTE:
        _need(current['phase']=='contact_observation_required','Only one new zero-TX contact observation then retention is admitted')
        _check_observation_payload(payload,current['proposal'])
        return
    if current['proposal'].get('route')=='audited_contact_reacquisition':
        _need(current['phase']=='reacquire_required' and current['probe_count']<current['proposal']['max_probes'],'Only bounded contact reacquisition before a candidate; no ordinary target/query')
        _check_contact_payload(payload,current['proposal'],current['current_width_m'],current['prior_target_m'])
        return
    allowed={'opening_required':'supported_recovery_open','confirmation_required':'supported_recovery_confirm'}
    phase=current['phase'];kind=payload.get('kind')
    _need((phase=='resolved' and kind not in allowed.values()) or allowed.get(phase)==kind,
          'Supported jaw recovery must finish before any ordinary target/query; no opening replay')
    if kind == 'supported_recovery_open' and current['proposal'].get('route') == 'audited_opening_continuation':
        target = _number(payload.get('request', {}).get('width_m'), 'new opening width')
        prior = current['proposal']['opening_continuation']['prior_opening_target_m']
        prior = max(prior, round(prior*1e6)/1e6)
        _need(target > prior and round(target*1e6)/1e6 > prior,
              'A further opening must strictly exceed the prior sent target; no replay')


def recovery_source(state):
    """Bind a device-only residual envelope; never transfer an old grasp/cache."""
    proposal = state['proposal']; source = proposal['snapshot']
    continued = proposal.get('route') == 'audited_opening_continuation'
    if continued:
        source = json.loads(source['scope']['record_json'])['proposal']['snapshot']
    payload, receipt = validate_failed_probe(source['event'], source['run'], source['scope']['owner'])
    if continued:
        receipt = copy.deepcopy(receipt)
        receipt['audited_opening_continuation'] = {
            **proposal['opening_continuation'], 'audit_proposal_sha256': proposal['proposal_sha256']}
    return payload, receipt


def authorization(value,s,h,now):
    _need(type(value) is dict and set(value)=={'source','message_id','statement','recorded_at','decision'}
          and value['source']=='user_message' and value['decision']=='authorize_supported_gripper_recovery'
          and type(value['statement']) is str and 1<=len(value['statement'].strip())<=4000,
          'An explicit repair instruction is required')
    _identifier(value['message_id'],'user message')
    _need(h['ended_at']<_number(value['recorded_at'],'instruction recorded time')<=now, 'Instruction must be recorded after the failed closed session')
    return value


def observed(evidence,s,contract,after,now):
    result=_observations(**evidence,contract=contract,after=after,now=now)
    from .feedback_tolerance import joint_tolerances,task_policy,rotation_tolerance
    from .contact_receipt import _rotation_span
    policy=task_policy(contract['task']);p,d=validate_failed_probe(s['event'],s['run'],s['scope']['owner'])
    # Manual object repositioning does not grant a different body pose or joint-limit exception.
    for side,path in evidence['passive_paths'].items():
        data=json.loads(_file(path)[0]);origin=d['before'][side]
        for item in data['pose_trace']:
            q=[item['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            _need(all(abs(a-b)<=limit for a,b,limit in zip(q,origin['joints_rad'],joint_tolerances(policy,side))), 'Body pose changed beyond original observation tolerance')
            pose=[item['end_pose_raw'][key]*(1e-6 if i<3 else math.pi/180000) for i,key in enumerate(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'))]
            _need(math.dist(pose[:3],origin['pose_m_rad'][:3])<=.0005 and _rotation_span([pose,origin['pose_m_rad']])<=rotation_tolerance(policy,side), 'Manual body displacement requires another recovery')
        width=result['passive'][side]['jaw_width_m'];reference=d['candidate_probe']['observed_width_m'] if side==p['arm'] else origin['gripper']['width_m']
        _need(abs(width-reference)<=.0005,'Current jaw must remain near the classified candidate width; peer unchanged')
        _need(result['passive'][side]['jaw_enabled'] is True,'Both healthy enabled jaws required')
    result.update(target_proximity_only=True,old_failed_result_preserved=True,grasp_verified=False,
                  target_replay_authorized=False,physical_stop_verified=None)
    return result


def prepare(path,run_id,*,session_log,probe_journal,user_instruction,passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');s=snapshot(db,run_id)
    now=_number(clock(),'clock');r=s['run'];deadline=r['started_at']+r['max_duration']
    _need(s['scope']['last_time']<=now<deadline and r['steps']<r['max_steps'],'Original task budget must remain')
    h=history(session_log,probe_journal,s);a=authorization(user_instruction,s,h,now)
    new=_current_contract(path.parent.parent,s['contract'],require_change=False)
    changes={k for k,v in new['code'].items() if s['contract']['code'].get(k)!=v}
    _need(changes<=REPAIR_FILES and set(s['contract']['code'])<=set(new['code']),'Only reviewed recovery and completion-feedback code may change')
    evidence=dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    p=dict(schema=SCHEMA,database=str(path),run_id=run_id,created_at=now,snapshot=s,history=h,authorization=a,
           reviewed_contract=new,evidence=observed(evidence,s,new,a['recorded_at'],now),
           budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
           hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,required_connection_mode='prepare',
           old_target_replay_authorized=False,grasp_transferred=False,physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def activate(p,*,project_root,clock=time.time):
    _need(p.get('schema')==SCHEMA and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p.get('proposal_sha256'),'Exact reviewed proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(p['database']);ev=p['evidence']
    _need(path==root/'runs/pair_sessions.sqlite','Canonical ledger required')
    evidence=dict(passive_paths={s:v['source']['path'] for s,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare(path,p['run_id'],session_log=p['history']['session']['path'],probe_journal=p['history']['probe_journal']['path'],
          user_instruction=p['authorization'],clock=lambda:p['created_at'],**evidence)==p,'Original history, sources or evidence changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(snapshot(db,p['run_id'])==p['snapshot'],'Original history changed')
                now=_number(clock(),'clock');_need(p['created_at']<=now<p['budget']['deadline_s'],'Original deadline reached')
                _need(observed(evidence,p['snapshot'],p['reviewed_contract'],p['authorization']['recorded_at'],now)==ev,'Admission observations expired')
                _need(_current_contract(root,p['snapshot']['contract'],require_change=False)==p['reviewed_contract'],'Reviewed sources changed')
                record=dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],old_rows_preserved=True,new_budget_allocated=False,
                            hardware_commands_sent=0,required_connection_mode='prepare',old_target_replay_authorized=False,grasp_transferred=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,'
                    'last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,'
                    'contract_json TEXT NOT NULL,proposal_sha256 TEXT NOT NULL,record_json TEXT NOT NULL)')
                s=p['snapshot'];db.execute('INSERT INTO '+TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (p['run_id'],now,s['scope']['owner'],json.dumps(s['retired_owners']),s['scope']['round_ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'continuation')))
                check_processes();_need(now<=clock()<p['budget']['deadline_s'],'Original deadline reached')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise


def audit_budget(db,run_id,*,max_steps,max_duration_s):
    rows=list(db.execute('SELECT * FROM '+TABLE));_need(len(rows)==1,'Exactly one manual successor required');row=rows[0]
    if row['run_id']!=run_id:return False
    r=db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone();record=json.loads(row['record_json']);p=record['proposal'];s=p['snapshot'];b=p['budget']
    _need(p.get('schema')==SCHEMA and p['run_id']==run_id and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p['proposal_sha256']==row['proposal_sha256']
          and record['new_contract']==p['reviewed_contract']==json.loads(row['contract_json'])
          and row['previous_owner']==s['scope']['owner'] and json.loads(row['retired_owners_json'])==s['retired_owners']
          and row['round_ordinal']==s['scope']['round_ordinal'] and p['created_at']<=record['activated_at']<=row['last_time']
          and record['activated_at']<b['deadline_s'] and record['hardware_commands_sent']==0
          and record['old_rows_preserved'] is True and record['new_budget_allocated'] is False
          and record['required_connection_mode']=='prepare' and record['old_target_replay_authorized'] is False and record['grasp_transferred'] is False
          and all(r[k]==s['run'][k] for k in ('run_id','started_at','max_steps','max_duration','contract_json'))
          and b==dict(started_at=r['started_at'],deadline_s=r['started_at']+r['max_duration'],max_steps=r['max_steps'],steps=s['run']['steps'],max_duration_s=r['max_duration'])
          and r['max_steps']==max_steps and r['max_duration']==max_duration_s,'Original exact run, budget and retirement required')
    changes={k for k,v in p['reviewed_contract']['code'].items() if s['contract']['code'].get(k)!=v}
    _need(changes<=REPAIR_FILES and {k:v for k,v in s['contract'].items() if k!='code'}=={k:v for k,v in p['reviewed_contract'].items() if k!='code'},'Unreviewed source/task change')
    shadow=sqlite3.connect(':memory:');shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()));shadow.execute('DROP TABLE '+TABLE)
        for name,items in tables(shadow).items():
            if name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
                for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run_id,))):
                    if 'owner' in item.keys() and item['owner'] not in s['retired_owners']:
                        _need(item['owner']==row['owner'],'Unexpected later owner')
                        shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run_id))
        _need(snapshot(shadow,run_id)==s,'Original history differs from manual enrollment')
        valid=_activated_execution_budget_db(shadow,run_id,max_steps=max_steps,max_duration_s=max_duration_s)
    finally:shadow.close()
    h=history(p['history']['session']['path'],p['history']['probe_journal']['path'],s);_need(h==p['history'],'Original closure/trace changed')
    authorization(p['authorization'],s,h,p['created_at']);ev=p['evidence']
    evidence=dict(passive_paths={side:v['source']['path'] for side,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(observed(evidence,s,p['reviewed_contract'],p['authorization']['recorded_at'],p['created_at'])==ev,'Archived admission changed')
    return valid


def _opening_failure_at(event):
    receipt = json.loads(event['receipt_json'])
    archive = receipt.get('archival_reconciliation')
    if archive is None:
        return event['finished_at']
    pending = archive['original_pending_event']
    _need(archive.get('schema') == 'piper_failed_opening_archival_v1'
          and _sha(pending) == archive['original_pending_sha256']
          and pending['status'] == 'pending' and pending['finished_at'] is None
          and pending['success'] is None and pending['receipt_json'] is None
          and all(event[k] == pending[k] for k in pending if k not in ('status','finished_at','success','receipt_json'))
          and archive['original_failure_receipt_sha256'] == _receipt_digest({k:v for k,v in receipt.items() if k != 'archival_reconciliation'})
          and pending['began_at'] <= archive['original_fault']['at'] < archive['recorded_at'] == event['finished_at']
          and archive['original_fault']['owner'] == event['owner']
          and archive['hardware_commands_sent'] == 0 and archive['old_failure_preserved'] is True,
          'Late failure archival must preserve its original pending row and real time')
    return archive['original_fault']['at']


def validate_failed_opening(event, run, scope):
    """Only a known single opening whose original body-anchor guard failed."""
    original = json.loads(scope['record_json'])['proposal']
    src = original['snapshot']
    old, candidate = validate_failed_probe(src['event'], src['run'], src['scope']['owner'])
    p, r = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    d = r.get('device_receipt', {}); arm = old['arm']; peer = 'right' if arm == 'left' else 'left'
    counts = {s: dict(attempted_frames=int(s == arm), sent_frames=int(s == arm), blocked_frames=0) for s in SIDES}
    _need(event['run_id'] == run['run_id'] == scope['run_id'] and event['owner'] == scope['owner']
          and event['step'] == run['steps'] and event['status'] == 'complete' and event['success'] == 0
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and set(p) == {'kind','request','arm','source_event_id','recovery_proposal_sha256','saved_rgb_evidence'}
          and p['kind'] == 'supported_recovery_open' and p['arm'] == arm
          and p['source_event_id'] == src['event']['event_id']
          and p['recovery_proposal_sha256'] == original['proposal_sha256']
          and r.get('event_id') == event['event_id'] and r.get('ok') is False
          and r.get('automatic_retry') is False and r.get('physical_stop_verified') is None,
          'Exact terminal failed recovery opening and original enrollment required')
    request = p['request']; target = _number(request.get('width_m'), 'original opening target')
    _need(set(request) == {'operation','observation_id','visual_description','support_relation','width_m','object_relation'}
          and request['operation'] == p['kind'] and request['support_relation'] == 'independent_support_present'
          and request['object_relation'] is None and 0 < target <= .055
          and d.get('requested_target') == target and d.get('completion_mode') == 'contact_probe_release'
          and d.get('errors') == [{'type':'RuntimeError','detail':'Grasp arm changed from its original candidate anchor: '+arm}]
          and d.get('ok') is False and d.get('arrival_confirmed') is False
          and d.get('transmission_counts') == d.get('session_transmission_counts') == counts
          and d.get('hardware_commands_sent') == d.get('target_calls_sent') == 1
          and d.get('nominal_force_N') == .2 and d.get('guard_violations') == []
          and all(d.get(k) == 0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('unresolved_gripper_probe') is None and d.get('grasp_verified') is False
          and d.get('physical_stop_verified') is None and 'contact_observation' not in d,
          'Only one complete bounded opening and original-anchor failure are covered')
    residual = d.get('grasp_states', {}).get(arm)
    _need(type(residual) is dict and d['grasp_states'].get(peer) is None
          and residual.get('status') == 'recovery_residual' and residual.get('arm') == arm
          and residual.get('original_anchor') == candidate['candidate_measurement']['anchor']
          and residual.get('requested_width_m') == candidate['candidate_probe']['requested_width_m']
          and residual.get('observed_width_m') == candidate['candidate_measurement']['observed']['width_m']
          and residual.get('sent_at') == candidate['candidate_probe']['sent_at']
          and residual.get('trace_sha256') == candidate['candidate_probe']['trace_sha256']
          and residual.get('identity') is None and residual.get('probe_event_id') is None
          and residual.get('failed_receipt_preserved') is True and residual.get('grasp_verified') is False
          and residual.get('target_may_remain_active') is True,
          'Original anonymous candidate body anchor and residual identity must remain unchanged')
    from . import arms
    from .feedback_tolerance import task_policy, joints_within, rotation_tolerance
    from .retention_receipt import anchor_deviation
    from .single_supervised_actions import BOUNDS
    from .contact_receipt import probe_closure_within_bound
    policy = task_policy(json.loads(scope['contract_json'])['task'])
    for side in SIDES:
        _need(arms.control_health(d['after'][side], now_s=_opening_failure_at(event),
              allowed_control_modes=(1,), require_enabled=True)['healthy'], 'Hard device fault is not an anchor-only failure')
    anchor = candidate['candidate_measurement']['anchor']; observed = d['after'][arm]
    deviation = anchor_deviation(anchor, observed)
    _need(not joints_within(policy, arm, anchor['joints_rad'], observed['joints_rad'])
          or deviation['position_m'] > BOUNDS['position_span_m']
          or deviation['rotation_rad'] > rotation_tolerance(policy, arm),
          'The recorded sample must actually violate its unchanged body anchor')
    _need(probe_closure_within_bound(target, d['dispatch_feedback'][arm]['gripper']['width_m']),
          'Original command must be a bounded opening')
    return p, d


def opening_snapshot(db, run_id):
    all_rows = tables(db)
    _need(OPENING_TABLE not in all_rows, 'This further-opening enrollment has already been consumed')
    scopes = all_rows.get(TABLE, []); _need(len(scopes) == 1, 'One original supported recovery required')
    scope = scopes[0]; run = next((r for r in all_rows['pair_runs'] if r['run_id'] == run_id), None)
    _need(run and scope['run_id'] == scope['active_run_id'] == run_id and scope['owner'] and scope['fault_id']
          and max(all_rows['pair_rounds'], key=lambda r:r['ordinal'])['ordinal'] == scope['round_ordinal'],
          'Latest faulted supported-recovery owner required')
    _need(audit_budget(db, run_id, max_steps=run['max_steps'], max_duration_s=run['max_duration']),
          'Original full supported-recovery budget lineage required')
    _need(not any(r['status'] == 'pending' for r in all_rows['pair_events']), 'Pending send blocks continuation')
    events = [r for r in all_rows['pair_events'] if r['run_id'] == run_id and r['owner'] == scope['owner']]
    prior = json.loads(scope['record_json'])['proposal']['budget']['steps']
    _need(len(events) == 1 and run['steps'] == prior+1 and events[0]['step'] == prior+1,
          'Exactly one terminal opening without later operations required')
    event = events[0]; validate_failed_opening(event, run, scope)
    faults = [r for r in all_rows['pair_faults'] if r['run_id'] == run_id and r['owner'] == scope['owner']]
    faults.sort(key=lambda f:f['id'])
    archive = json.loads(event['receipt_json']).get('archival_reconciliation')
    _need(len(faults) == (2 if archive else 1) and faults[0]['id'] == scope['fault_id']
          and faults[0]['reason'] == 'Claimed preparation/query failed or uncertain: Supported jaw recovery failed; no automatic retry'
          and event['began_at'] <= faults[0]['at'] <= event['finished_at'], 'Only the retained opening failure is covered')
    if archive:
        _need(archive['original_fault'] == faults[0] and faults[1]['reason'] == 'execution_receipt_failed'
              and faults[1]['at'] == event['finished_at'], 'Late archival must retain the original fault and append only its failed completion')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends','pair_grasp_episodes'):
        _need(not any(r['run_id'] == run_id and r.get('owner') == scope['owner'] for r in all_rows.get(name, [])),
              'Held object, joint/cache or uncertain history blocks further opening')
    return dict(run=run, scope=scope, event=event, contract=json.loads(scope['contract_json']),
                retired_owners=sorted({r['owner'] for rows in all_rows.values() for r in rows if r.get('owner')}),
                table_sha256={k:_table_sha(v) for k,v in all_rows.items()})


def opening_history(session_log, opening_journal, s, *, source_prefix=None):
    raw, ref = _file(session_log); rows = [json.loads(l) for l in raw.decode().splitlines()]
    e = s['event']; p, d = validate_failed_opening(e, s['run'], s['scope'])
    failure_at = _opening_failure_at(e)
    _need(rows and [r['sequence'] for r in rows] == list(range(1,len(rows)+1))
          and [r['at'] for r in rows] == sorted(r['at'] for r in rows)
          and rows[-1]['kind'] == 'session_ended' and rows[-1]['cleanup_errors'] == []
          and rows[-1]['at'] > failure_at, 'Normally ended original recovery session required')
    results = [r['result'] for r in rows if r['kind'] == 'result']
    _need(any(r.get('owner') == s['scope']['owner'] and r.get('run_id') == s['run']['run_id'] for r in results),
          'Original recovery host binding required')
    closes = [r for r in results if r.get('status') == 'closed']; _need(len(closes) == 1, 'One original close required')
    c = closes[0]['cleanup']
    _need(closes[0]['fault_latched'] is True and c.get('session_transmission_counts') == d['session_transmission_counts']
          and c.get('guard_violations') == [] and c.get('unresolved_gripper_probe') is None
          and c.get('grasp_states') == d['grasp_states'] and c.get('requires_fault_latch') is True
          and all(c['arms'][side]['status'] == 'disconnected' for side in SIDES),
          'Closure must preserve the unresolved residual and exact one-frame totals')
    raw, jref = _file(opening_journal); index = json.loads(raw); journal = index['rows']
    _need(len(journal) == 3 and [r['event'] for r in journal] == [
          'pair_preparation_claimed','single_supervised_action_intent','single_supervised_action_sent_unconfirmed'],
          'Exact original claim, opening intent and returned frame required')
    claim, intent, sent = journal; target = p['request']['width_m']
    expected = round(target*1e6).to_bytes(4,'big',signed=True)+bytes.fromhex('00c80100')
    _need(claim['event_id'] == e['event_id'] and claim['payload'] == p
          and intent['arm'] == p['arm'] and intent['kind'] == sent['kind'] == 'gripper'
          and intent['target'] == target and intent['frames'] == [{'id':0x159,'data_hex':expected.hex()}]
          and e['began_at'] <= claim['unix_s'] <= intent['unix_s'] <= sent['finished_unix_s'] <= sent['unix_s'] <= failure_at,
          'Original once-only opening frame/time binding differs')
    # The same journal is append-only across hosts. Bind the archived byte prefix,
    # not the later file size/hash, while independently locating the three rows.
    source = Path(index.get('source', index.get('source_journal', ''))).resolve(strict=True)
    prefix = source_prefix or dict(path=str(source), size_bytes=source.stat().st_size,
                                  sha256=index['source_sha256'])
    _need(prefix['path'] == str(source) and type(prefix['size_bytes']) is int and prefix['size_bytes'] > 0,
          'Original journal prefix binding required')
    hasher = hashlib.sha256(); remaining = prefix['size_bytes']; found = []
    names = tuple(name.encode() for name in ('pair_preparation_claimed','single_supervised_action_intent','single_supervised_action_sent_unconfirmed'))
    with source.open('rb') as stream:
        while remaining:
            line = stream.readline(remaining); _need(line, 'Original journal prefix was truncated')
            remaining -= len(line); hasher.update(line)
            if any(name in line for name in names):
                row = json.loads(line)
                if e['began_at'] <= row.get('unix_s', -1) <= failure_at and row.get('event','').encode() in names:
                    found.append(row)
    _need(hasher.hexdigest() == prefix['sha256'] == index['source_sha256'] and found == journal,
          'Original journal prefix or extracted transmission provenance changed')
    archive = json.loads(e['receipt_json']).get('archival_reconciliation')
    if archive:
        _need(archive['history']['session'] == ref and archive['history']['opening_journal'] == jref
              and archive['history']['source_prefix'] == prefix,
              'Reconciled failure sources must match original closure and frame audit')
        observed = [row for row in rows if row['kind'] == 'result' and row.get('result',{}).get('event_id') == e['event_id']
                    and row['result'].get('status') == 'fault']
        _need(len(observed) == 1 and observed[0]['sequence'] == archive['fault_result_sequence']
              and observed[0]['result']['receipt'] == {k:v for k,v in json.loads(e['receipt_json']).items() if k != 'archival_reconciliation'},
              'Late failure receipt must equal the original host fault result')
    return dict(session=ref, opening_journal=jref, source_prefix=prefix,
                ended_at=rows[-1]['at'], physical_stop_verified=None)


def opening_observed(evidence, s, contract, after, now):
    result = _observations(**evidence, contract=contract, after=max(after,s['event']['finished_at']), now=now)
    original = json.loads(s['scope']['record_json'])['proposal']['snapshot']
    old, candidate = validate_failed_probe(original['event'], original['run'], original['scope']['owner'])
    _, failed = validate_failed_opening(s['event'], s['run'], s['scope'])
    from .feedback_tolerance import joint_tolerances, task_policy, rotation_tolerance
    from .contact_receipt import _rotation_span
    from .retention_receipt import measured_anchor
    from .takeover import LIMITS
    policy = task_policy(contract['task']); arm = old['arm']
    for side, path in evidence['passive_paths'].items():
        data = json.loads(_file(path)[0])
        origin = candidate['candidate_measurement']['anchor'] if side == arm else measured_anchor(candidate['before'][side])
        for item in data['pose_trace']:
            q = [item['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            pose = [item['end_pose_raw'][key]*(1e-6 if i<3 else math.pi/180000)
                    for i,key in enumerate(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'))]
            _need(all(abs(a-b) <= limit for a,b,limit in zip(q,origin['joints_rad'],joint_tolerances(policy,side)))
                  and math.dist(pose[:3],origin['pose_m_rad'][:3]) <= .0005
                  and _rotation_span([pose,origin['pose_m_rad']]) <= rotation_tolerance(policy,side),
                  'Every new body sample must remain inside the unchanged original candidate/peer anchor')
        width = result['passive'][side]['jaw_width_m']
        reference = failed['requested_target'] if side == arm else origin['width_m']
        _need(abs(width-reference) <= (LIMITS['gripper_m'] if side == arm else .0005)
              and result['passive'][side]['jaw_enabled'] is True,
              'Latest residual jaw must be near the sent opening target; peer jaw unchanged')
    result.update(old_failed_result_preserved=True, grasp_verified=False, object_release_verified=False,
                  target_replay_authorized=False, physical_stop_verified=None,
                  jaw_whole_window_observed=False, fresh_device_jaw_baseline_required=True)
    return result


def prepare_opening_continuation(path, run_id, *, session_log, opening_journal, user_instruction,
                                 passive_paths, rgb_observation, visual_observation, clock=time.time):
    path = Path(path).resolve(strict=True); check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory=sqlite3.Row; db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
        s = opening_snapshot(db, run_id)
    now = _number(clock(),'clock'); r = s['run']; deadline = r['started_at']+r['max_duration']
    _need(s['scope']['last_time'] <= now < deadline and r['steps']+2 <= r['max_steps'],
          'Original budget must cover one further opening and its new confirmation')
    h = opening_history(session_log, opening_journal, s); a = authorization(user_instruction,s,h,now)
    new = _current_contract(path.parent.parent,s['contract'],require_change=False)
    _need({k for k,v in new['code'].items() if s['contract']['code'].get(k) != v} <= REPAIR_FILES
          and set(s['contract']['code']) <= set(new['code']), 'Only reviewed supported recovery code may change')
    evidence = dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    ev = opening_observed(evidence,s,new,a['recorded_at'],now)
    original = json.loads(s['scope']['record_json'])['proposal']['snapshot']
    old, candidate = validate_failed_probe(original['event'],original['run'],original['scope']['owner'])
    arm = old['arm']; passive = ev['passive'][arm]
    data = json.loads(_file(passive_paths[arm])[0]); jaw_time = data['feedback']['PiperMsgGripperFeedBack']['received_at_s']
    env = dict(schema=OPENING_SCHEMA,arm=arm,source_receipt_sha256=_receipt_digest(candidate),
               prior_opening_event_id=s['event']['event_id'],prior_opening_receipt_sha256=_receipt_digest(json.loads(s['event']['receipt_json'])),
               prior_opening_target_m=json.loads(s['event']['payload_json'])['request']['width_m'],
               prior_opening_finished_at=s['event']['finished_at'],
               residual_jaw_anchor=dict(width_m=passive['jaw_width_m'],observed_at=jaw_time,source=passive['source']))
    p = dict(schema=OPENING_SCHEMA,route='audited_opening_continuation',database=str(path),run_id=run_id,created_at=now,
             snapshot=s,history=h,authorization=a,reviewed_contract=new,evidence=ev,opening_continuation=env,
             budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
             hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,required_connection_mode='prepare',
             old_target_replay_authorized=False,grasp_transferred=False,physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def activate_opening_continuation(p, *, project_root, clock=time.time):
    _need(p.get('schema') == OPENING_SCHEMA and p.get('route') == 'audited_opening_continuation'
          and _sha({k:v for k,v in p.items() if k != 'proposal_sha256'}) == p.get('proposal_sha256'), 'Exact reviewed continuation required')
    root = Path(project_root).resolve(strict=True); path = Path(p['database']); ev = p['evidence']
    _need(path == root/'runs/pair_sessions.sqlite', 'Canonical ledger required')
    evidence = dict(passive_paths={s:v['source']['path'] for s,v in ev['passive'].items()},
                    rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare_opening_continuation(path,p['run_id'],session_log=p['history']['session']['path'],
          opening_journal=p['history']['opening_journal']['path'],user_instruction=p['authorization'],
          clock=lambda:p['created_at'],**evidence) == p, 'Original reviewed continuation changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory == root or (directory/'runs').is_dir(): locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row; db.execute('PRAGMA synchronous=FULL'); db.execute('BEGIN IMMEDIATE')
            try:
                _need(opening_snapshot(db,p['run_id']) == p['snapshot'], 'Original recovery history changed')
                now = _number(clock(),'clock'); _need(p['created_at'] <= now < p['budget']['deadline_s'], 'Original deadline reached')
                _need(opening_observed(evidence,p['snapshot'],p['reviewed_contract'],p['authorization']['recorded_at'],now) == ev,
                      'Current continuation observations expired')
                _need(_current_contract(root,p['snapshot']['contract'],require_change=False) == p['reviewed_contract'], 'Reviewed sources changed')
                record = dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],old_rows_preserved=True,
                              new_budget_allocated=False,hardware_commands_sent=0,required_connection_mode='prepare',
                              old_target_replay_authorized=False,grasp_transferred=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+OPENING_TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,'
                    'last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,'
                    'contract_json TEXT NOT NULL,proposal_sha256 TEXT NOT NULL,record_json TEXT NOT NULL)')
                s = p['snapshot']; db.execute('INSERT INTO '+OPENING_TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (p['run_id'],now,s['scope']['owner'],json.dumps(s['retired_owners']),s['scope']['round_ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'continuation')))
                check_processes(); _need(now <= clock() < p['budget']['deadline_s'], 'Original deadline reached')
                db.execute('COMMIT'); return record
            except BaseException:
                if db.in_transaction: db.execute('ROLLBACK')
                raise


def audit_opening_budget(db, run_id, *, max_steps, max_duration_s):
    rows = list(db.execute('SELECT * FROM '+OPENING_TABLE)); _need(len(rows) == 1, 'One further-opening scope required')
    row = rows[0]
    if row['run_id'] != run_id: return False
    r = db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone()
    record = json.loads(row['record_json']); p = record['proposal']; s = p['snapshot']; b = p['budget']
    _need(p.get('schema') == OPENING_SCHEMA and p.get('route') == 'audited_opening_continuation'
          and p['run_id'] == run_id and _sha({k:v for k,v in p.items() if k != 'proposal_sha256'}) == p['proposal_sha256'] == row['proposal_sha256']
          and record['new_contract'] == p['reviewed_contract'] == json.loads(row['contract_json'])
          and row['previous_owner'] == s['scope']['owner'] and json.loads(row['retired_owners_json']) == s['retired_owners']
          and row['round_ordinal'] == s['scope']['round_ordinal'] and p['created_at'] <= record['activated_at'] <= row['last_time']
          and record['activated_at'] < b['deadline_s'] and record['hardware_commands_sent'] == 0
          and record['old_rows_preserved'] is True and record['new_budget_allocated'] is False
          and record['required_connection_mode'] == 'prepare' and record['old_target_replay_authorized'] is False
          and record['grasp_transferred'] is False and record.get('physical_stop_verified') is None
          and all(r[k] == s['run'][k] for k in ('run_id','started_at','max_steps','max_duration','contract_json'))
          and b == dict(started_at=r['started_at'],deadline_s=r['started_at']+r['max_duration'],max_steps=r['max_steps'],steps=s['run']['steps'],max_duration_s=r['max_duration'])
          and r['max_steps'] == max_steps and r['max_duration'] == max_duration_s, 'Exact original recovery budget and retirement required')
    _need({k for k,v in p['reviewed_contract']['code'].items() if s['contract']['code'].get(k) != v} <= REPAIR_FILES
          and {k:v for k,v in s['contract'].items() if k != 'code'} == {k:v for k,v in p['reviewed_contract'].items() if k != 'code'},
          'Unreviewed recovery source/task change')
    shadow = sqlite3.connect(':memory:'); shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump())); shadow.execute('DROP TABLE '+OPENING_TABLE)
        for name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
            if not shadow.execute('SELECT 1 FROM sqlite_master WHERE name=?',(name,)).fetchone(): continue
            for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run_id,))):
                if 'owner' in item.keys() and item['owner'] not in s['retired_owners']:
                    _need(item['owner'] == row['owner'], 'Unexpected later owner')
                    shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run_id))
        _need(opening_snapshot(shadow,run_id) == s, 'Original failed opening history differs')
    finally: shadow.close()
    h = opening_history(p['history']['session']['path'],p['history']['opening_journal']['path'],s,source_prefix=p['history']['source_prefix'])
    _need(h == p['history'], 'Original opening closure/provenance changed')
    authorization(p['authorization'],s,h,p['created_at']); ev = p['evidence']
    evidence = dict(passive_paths={side:v['source']['path'] for side,v in ev['passive'].items()},
                    rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(opening_observed(evidence,s,p['reviewed_contract'],p['authorization']['recorded_at'],p['created_at']) == ev,
          'Archived opening admission changed')
    _, source = recovery_source(dict(proposal=p)); envelope = source['audited_opening_continuation']; arm = envelope['arm']
    original = json.loads(s['scope']['record_json'])['proposal']['snapshot']
    old, candidate = validate_failed_probe(original['event'],original['run'],original['scope']['owner'])
    passive = json.loads(_file(ev['passive'][arm]['source']['path'])[0])
    expected = dict(schema=OPENING_SCHEMA,arm=old['arm'],source_receipt_sha256=_receipt_digest(candidate),
        prior_opening_event_id=s['event']['event_id'],prior_opening_receipt_sha256=_receipt_digest(json.loads(s['event']['receipt_json'])),
        prior_opening_target_m=json.loads(s['event']['payload_json'])['request']['width_m'],prior_opening_finished_at=s['event']['finished_at'],
        residual_jaw_anchor=dict(width_m=ev['passive'][arm]['jaw_width_m'],
            observed_at=passive['feedback']['PiperMsgGripperFeedBack']['received_at_s'],source=ev['passive'][arm]['source']))
    _need(p['opening_continuation'] == expected and envelope['audit_proposal_sha256'] == p['proposal_sha256'],
          'Residual jaw envelope must derive from the unchanged opening and new raw feedback')
    return True


def _pending_opening_snapshot(db, run_id):
    """Read a failed host's still-pending ledger claim without excusing it."""
    rows = tables(db)
    _need(OPENING_TABLE not in rows and len(rows.get(TABLE, [])) == 1,
          'Pending archival belongs to the original supported-recovery scope')
    scope = rows[TABLE][0]; run = next((r for r in rows['pair_runs'] if r['run_id'] == run_id), None)
    _need(run and scope['run_id'] == scope['active_run_id'] == run_id and scope['owner'] and scope['fault_id']
          and max(rows['pair_rounds'], key=lambda r:r['ordinal'])['ordinal'] == scope['round_ordinal'],
          'Original faulted recovery owner required')
    _need(audit_budget(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Full original budget lineage required before archival')
    pending = [r for r in rows['pair_events'] if r['status'] == 'pending']
    events = [r for r in rows['pair_events'] if r['run_id'] == run_id and r['owner'] == scope['owner']]
    prior = json.loads(scope['record_json'])['proposal']['budget']['steps']
    _need(len(events) == len(pending) == 1 and events == pending and events[0]['step'] == run['steps'] == prior+1
          and events[0]['receipt_json'] is None and events[0]['finished_at'] is None and events[0]['success'] is None,
          'One exact pending opening without any later attempt required')
    faults = [f for f in rows['pair_faults'] if f['run_id'] == run_id and f['owner'] == scope['owner']]
    _need(len(faults) == 1 and faults[0]['id'] == scope['fault_id']
          and faults[0]['reason'] == 'Claimed preparation/query failed or uncertain: Supported jaw recovery failed; no automatic retry',
          'Original known failure must already be latched')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends','pair_grasp_episodes'):
        _need(not any(r['run_id'] == run_id and r.get('owner') == scope['owner'] for r in rows.get(name, [])),
              'Unrelated held, joint or uncertain history blocks archival')
    return dict(run=run,scope=scope,event=events[0],fault=faults[0],contract=json.loads(scope['contract_json']),
                table_sha256={k:_table_sha(v) for k,v in rows.items()})


def prepare_pending_opening_reconciliation(path, run_id, *, session_log, opening_journal,
                                           user_instruction, clock=time.time):
    """Propose a late FAILED receipt from closed-host evidence; zero device IO."""
    path = Path(path).resolve(strict=True); check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row; db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
        s = _pending_opening_snapshot(db,run_id)
    raw, _ = _file(session_log); rows = [json.loads(line) for line in raw.decode().splitlines()]
    observed = [r for r in rows if r.get('kind') == 'result'
                and r.get('result',{}).get('event_id') == s['event']['event_id'] and r['result'].get('status') == 'fault']
    _need(len(observed) == 1, 'One actual closed-host fault receipt required')
    result = observed[0]; receipt = result['result']['receipt']
    _need('archival_reconciliation' not in receipt and s['fault']['at'] <= result['at'], 'Original fault result chronology required')
    # This projection exists only for pure validators; it is never written to DB.
    projected = {**s['event'],'status':'complete','success':0,'finished_at':s['fault']['at'],
                 'receipt_json':_json_object(receipt,'archived failed receipt')}
    validate_failed_opening(projected,s['run'],s['scope'])
    h = opening_history(session_log,opening_journal,{**s,'event':projected})
    _need(any(r.get('kind') == 'result' and r.get('result',{}).get('fault') == s['fault'] for r in rows),
          'The original session must independently contain this exact latched fault')
    now = _number(clock(),'clock'); _need(now >= s['scope']['last_time'] and now > h['ended_at'], 'Real post-close archival time required')
    a = authorization(user_instruction,s,h,now)
    root = path.parent.parent
    code = {name:_file(root/'robot_tools'/name)[1] for name in ('supported_gripper_recovery.py','pair_ledger.py')}
    p = dict(schema='piper_failed_opening_reconciliation_proposal_v1',database=str(path),run_id=run_id,
             created_at=now,snapshot=s,history=h,authorization=a,failure_receipt=receipt,
             fault_result_sequence=result['sequence'],reviewed_archival_code=code,
             hardware_commands_sent=0,dispatch_authorized=False,new_budget_allocated=False)
    return {**p,'proposal_sha256':_sha(p)}


def reconcile_pending_opening(p, *, project_root, clock=time.time):
    """Normal ledger.finish(success=False), with provenance and actual late time.

    No pending-row exemption, success rewrite, owner takeover or fault clearing.
    Repeated/stale proposals are rejected before constructing a ledger writer.
    """
    _need(p.get('schema') == 'piper_failed_opening_reconciliation_proposal_v1'
          and _sha({k:v for k,v in p.items() if k != 'proposal_sha256'}) == p.get('proposal_sha256'),
          'Exact reviewed failure-reconciliation proposal required')
    root = Path(project_root).resolve(strict=True); path = Path(p['database'])
    _need(path == root/'runs/pair_sessions.sqlite','Canonical ledger required')
    _need(prepare_pending_opening_reconciliation(path,p['run_id'],session_log=p['history']['session']['path'],
          opening_journal=p['history']['opening_journal']['path'],user_instruction=p['authorization'],
          clock=lambda:p['created_at']) == p, 'Original pending claim, sources or authorization changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory == root or (directory/'runs').is_dir(): locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
            db.row_factory=sqlite3.Row; db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
            _need(_pending_opening_snapshot(db,p['run_id']) == p['snapshot'], 'Pending history changed before archival')
        _need({name:_file(root/'robot_tools'/name)[1] for name in p['reviewed_archival_code']} == p['reviewed_archival_code'],
              'Reviewed archival sources changed')
        from .pair_ledger import PairLedger
        s=p['snapshot']; now=_number(clock(),'clock'); _need(now >= p['created_at'], 'Archival clock regressed')
        ledger=PairLedger(path,p['run_id'],s['contract'],max_steps=s['run']['max_steps'],
                          max_duration_s=s['run']['max_duration'],clock=lambda:now)
        recorded=_number(clock(),'clock'); _need(recorded >= now, 'Archival clock regressed')
        # Freeze the actual just-read time for this single normal transaction.
        ledger.clock=lambda:recorded
        provenance=dict(schema='piper_failed_opening_archival_v1',original_pending_event=s['event'],
                        original_pending_sha256=_sha(s['event']),original_failure_receipt_sha256=_receipt_digest(p['failure_receipt']),
                        original_fault=s['fault'],fault_result_sequence=p['fault_result_sequence'],history=p['history'],
                        before_table_sha256=s['table_sha256'],reconciliation_proposal_sha256=p['proposal_sha256'],
                        recorded_at=recorded,hardware_commands_sent=0,old_failure_preserved=True)
        receipt={**p['failure_receipt'],'archival_reconciliation':provenance}
        ledger.finish(s['scope']['owner'],s['event']['event_id'],receipt,success=False)
        return dict(status='failed_receipt_archived',run_id=p['run_id'],event_id=s['event']['event_id'],
                    recorded_at=recorded,hardware_commands_sent=0,dispatch_authorized=False,
                    new_budget_allocated=False,original_fault_preserved=s['fault']['id'],
                    original_pending_sha256=_sha(s['event']),receipt_sha256=_receipt_digest(receipt),
                    physical_stop_verified=None)

def _contact_original(s):
    if 'parent_binding' in s['scope']:
        source=s['scope']['parent_binding']['original_probe']
        return validate_failed_probe(source['event'],source['run'],source['owner'])
    parent = json.loads(s['scope']['record_json'])['proposal']
    payload, receipt = recovery_source(dict(proposal=parent))
    return payload, {k:v for k,v in receipt.items() if k != 'audited_opening_continuation'}


def validate_successful_opening(event, run, scope):
    old, candidate = _contact_original(dict(scope=scope))
    p, d = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    parent = scope['parent_binding']; arm = old['arm']
    counts = {side:dict(attempted_frames=int(side==arm),sent_frames=int(side==arm),blocked_frames=0) for side in SIDES}
    _need(arm == 'left' and event['run_id'] == run['run_id'] == scope['run_id']
          and event['owner'] == scope['owner'] and event['step'] == run['steps']
          and event['status'] == 'complete' and event['success'] == 1
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and set(p) == {'kind','request','arm','source_event_id','recovery_proposal_sha256','saved_rgb_evidence'}
          and p['kind'] == 'supported_recovery_open' and p['arm'] == arm
          and p['source_event_id'] == parent['source_event_id']
          and p['recovery_proposal_sha256'] == parent['proposal_sha256']
          and d.get('event_id') == event['event_id'] and d.get('pair_owner') == scope['owner'],
          'Exact latest successful left opening and its audited source required')
    request = p['request']; target = _number(request.get('width_m'),'opening width')
    _need(request.get('operation') == p['kind'] and request.get('support_relation') == 'independent_support_present'
          and request.get('object_relation') is None and d.get('requested_target') == target
          and d.get('ok') is True and d.get('status') == 'release_arrived'
          and d.get('arrival_confirmed') is True and d.get('observed_stable') is True
          and d.get('completion_mode') == 'contact_probe_release'
          and d.get('hardware_commands_sent') == d.get('target_calls_sent') == 1
          and d.get('transmission_counts') == d.get('session_transmission_counts') == counts
          and d.get('nominal_force_N') == .2 and d.get('errors') == d.get('guard_violations') == []
          and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('grasp_states') == dict.fromkeys(SIDES) and d.get('unresolved_gripper_probe') is None
          and d.get('grasp_transferred') is False and d.get('grasp_verified') is False
          and d.get('physical_stop_verified') is None and not d.get('object_release_verified')
          and d.get('release_measurement',{}).get('anchor') == candidate['candidate_measurement']['anchor'],
          'Only a complete mechanical opening with unchanged original body anchor is covered')
    c=d.get('contact_observation',{}); jaw=d['sample']['arms'][arm]['gripper']
    _need(c.get('outcome') == 'release_arrived' and c.get('completion') == 'observation_only'
          and c.get('arrival_confirmed') is True and c.get('requested_width_m') == target
          and c.get('observed_width_m') == d.get('observed_width_m') == jaw['width_m']
          and d['after'][arm]['gripper']['width_m'] == jaw['width_m']
          and candidate['candidate_probe']['completed_at'] < jaw['timestamp'] <= event['finished_at']
          and event['began_at'] < d['release_measurement']['ended_at'] <= event['finished_at'],
          'Opening feedback and original anchor chronology must agree')
    return p,d


def contact_snapshot(db, run_id):
    rows=tables(db);_need(CONTACT_TABLE not in rows,'Contact reacquisition scope has already been consumed')
    scopes=rows.get(OPENING_TABLE,[]);_need(len(scopes)==1,'One completed audited further opening required')
    scope=scopes[0];run=next((r for r in rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id']
          and max(rows['pair_rounds'],key=lambda r:r['ordinal'])['ordinal']==scope['round_ordinal'],
          'Latest closed opening owner and retained separation fault required')
    _need(audit_opening_budget(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Full opening lineage and original budget required')
    _need(not any(e['status']=='pending' for e in rows['pair_events']),'Any pending send blocks reacquisition')
    events=[e for e in rows['pair_events'] if e['run_id']==run_id and e['owner']==scope['owner']]
    floor=json.loads(scope['record_json'])['proposal']['budget']['steps']
    _need(len(events)==1 and run['steps']==events[0]['step']==floor+1,'Exactly one completed opening without later action required')
    event=events[0]
    parent=json.loads(scope['record_json'])['proposal']
    original=json.loads(parent['snapshot']['scope']['record_json'])['proposal']['snapshot']
    scope={**{k:v for k,v in scope.items() if k!='record_json'},
           'record_json_sha256':hashlib.sha256(scope['record_json'].encode()).hexdigest(),
           'parent_binding':dict(proposal_sha256=parent['proposal_sha256'],source_event_id=parent['snapshot']['event']['event_id'],
                                 original_probe=dict(event=original['event'],run=original['run'],owner=original['scope']['owner']))}
    validate_successful_opening(event,run,scope)
    faults=[f for f in rows['pair_faults'] if f['run_id']==run_id and f['owner']==scope['owner']]
    _need(len(faults)==1 and faults[0]['id']==scope['fault_id']
          and faults[0]['reason']=='Cannot establish clean grasp release during cleanup: Supported jaw recovery must finish before ordinary preparation or motion'
          and event['finished_at']<faults[0]['at']<=scope['last_time'],
          'Only unresolved visual separation at close is covered; old faults stay intact')
    for name in ('pair_grasp_episodes','pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends'):
        _need(not any(r['run_id']==run_id and r.get('owner')==scope['owner'] for r in rows.get(name,[])),
              'Held/loaded/unknown history cannot enter contact reacquisition')
    return dict(run=run,scope=scope,event=event,contract=json.loads(scope['contract_json']),
                retired_owners=sorted({r['owner'] for items in rows.values() for r in items if r.get('owner')}),
                table_sha256={k:_table_sha(v) for k,v in rows.items()})


def contact_history(session_log, opening_journal, s, *, source_prefix=None):
    raw,ref=_file(session_log);rows=[json.loads(line) for line in raw.decode().splitlines()]
    e=s['event'];p,d=validate_successful_opening(e,s['run'],s['scope'])
    _need(rows and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows)
          and rows[-1]['kind']=='session_ended' and rows[-1]['cleanup_errors']==[]
          and rows[-1]['at']>s['scope']['last_time'],'Normally ended latest opening session required')
    results=[r['result'] for r in rows if r['kind']=='result']
    _need(any(r.get('owner')==s['scope']['owner'] and r.get('run_id')==s['run']['run_id'] for r in results),
          'Latest opening host binding required')
    closes=[r for r in results if r.get('status')=='closed'];_need(len(closes)==1,'One normal resource close required')
    cleanup=closes[0]['cleanup']
    _need(closes[0]['fault_latched'] is True and cleanup.get('session_transmission_counts')==d['session_transmission_counts']
          and cleanup.get('guard_violations')==[] and cleanup.get('requires_fault_latch') is False
          and cleanup.get('unresolved_gripper_probe') is None and cleanup.get('grasp_states')==dict.fromkeys(SIDES)
          and all(cleanup['arms'][side]['status']=='disconnected' for side in SIDES),
          'No unresolved send or local grasp may be hidden by resource cleanup')
    raw,jref=_file(opening_journal);index=json.loads(raw);journal=index['rows']
    names=('pair_preparation_claimed','single_supervised_action_intent','single_supervised_action_sent_unconfirmed','bounded_probe_trace')
    _need([r['event'] for r in journal]==list(names),'Exact successful opening claim/intent/return/trace required')
    claim,intent,sent,tr=journal;target=p['request']['width_m'];arm=p['arm']
    expected=round(target*1e6).to_bytes(4,'big',signed=True)+bytes.fromhex('00c80100')
    _need(claim['event_id']==e['event_id'] and claim['payload']==p and intent['arm']==arm
          and intent['kind']==sent['kind']=='gripper' and intent['target']==target
          and intent['frames']==[{'id':0x159,'data_hex':expected.hex()}]
          and tr['release'] is True and tr['arm']==arm and tr['requested_width_m']==target
          and _receipt_digest(tr['trace'])==tr['sha256']==d['contact_observation']['trace_summary']['sha256']
          and e['began_at']<=claim['unix_s']<=intent['unix_s']<=sent['finished_unix_s']<=sent['unix_s']<=tr['unix_s']<=e['finished_at'],
          'Complete successful opening source, payload or times differ')
    source=Path(index.get('source',index.get('source_journal',''))).resolve(strict=True)
    prefix=source_prefix or dict(path=str(source),size_bytes=source.stat().st_size,sha256=index['source_sha256'])
    _need(prefix['path']==str(source) and type(prefix['size_bytes']) is int and prefix['size_bytes']>0,'Frozen journal prefix required')
    remaining=prefix['size_bytes'];hasher=hashlib.sha256();found=[];encoded=tuple(name.encode() for name in names)
    with source.open('rb') as stream:
        while remaining:
            line=stream.readline(remaining);_need(line,'Opening source prefix was truncated');remaining-=len(line);hasher.update(line)
            if any(name in line for name in encoded):
                item=json.loads(line)
                if e['began_at']<=item.get('unix_s',-1)<=e['finished_at'] and item.get('event') in names:found.append(item)
    _need(hasher.hexdigest()==prefix['sha256']==index['source_sha256'] and found==journal,'Opening source prefix/extracted trace changed')
    from .retention_receipt import summarize_retention_trace
    from .feedback_tolerance import task_policy
    old,candidate=_contact_original(s);samples=tr['trace']['post'];cutoff=samples[-1]['observed_at_s']-3.
    stable=samples[max(i for i,item in enumerate(samples) if item['observed_at_s']<=cutoff):]
    measured=summarize_retention_trace(arm=arm,identity=None,probe_event_id=None,trace_id='release_'+tr['sha256'][:32],
        trace_sha256=tr['sha256'],original_anchor=candidate['candidate_measurement']['anchor'],samples=stable,
        now=stable[-1]['observed_at_s'],release=True,feedback_policy=task_policy(s['contract']['task']))
    baseline=[item['arms'][arm]['gripper']['width_m'] for item in tr['trace']['baseline']]
    widths=[item['arms'][arm]['gripper']['width_m'] for item in stable]
    _need(_same_derived_measurement(measured,d['release_measurement']) and min(widths)-max(baseline)==d['actual_opening_increase_m']
          and all(abs(w-target)<=.002 for w in widths) and min(widths)-max(baseline)>.0005,
          'Successful opening must rederive from its actual stable response trace')
    return dict(session=ref,opening_journal=jref,source_prefix=prefix,ended_at=rows[-1]['at'],physical_stop_verified=None)


def contact_authorization(value,s,h,now):
    _need(type(value) is dict and set(value)=={'source','message_id','statement','recorded_at','decision'}
          and value['source']=='user_message' and value['decision']=='authorize_supported_contact_reacquisition'
          and type(value['statement']) is str and 1<=len(value['statement'].strip())<=4000,
          'Explicit instruction for supported left contact reacquisition required')
    _identifier(value['message_id'],'user message')
    _need(s['event']['finished_at']<_number(value['recorded_at'],'instruction time')<=now,
          'Reacquisition instruction must follow the completed opening')
    return value


def contact_observed(evidence,s,contract,after,now):
    result=_observations(**evidence,contract=contract,after=max(after,s['scope']['last_time']),now=now)
    old,candidate=_contact_original(s);_,opened=validate_successful_opening(s['event'],s['run'],s['scope'])
    from .feedback_tolerance import task_policy,joint_tolerances,rotation_tolerance
    from .retention_receipt import measured_anchor
    from .contact_receipt import _rotation_span
    policy=task_policy(contract['task']);arm=old['arm']
    for side,path in evidence['passive_paths'].items():
        data=json.loads(_file(path)[0]);origin=candidate['candidate_measurement']['anchor'] if side==arm else measured_anchor(candidate['before'][side])
        for item in data['pose_trace']:
            q=[item['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            pose=[item['end_pose_raw'][key]*(1e-6 if i<3 else math.pi/180000) for i,key in enumerate(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'))]
            _need(all(abs(a-b)<=limit for a,b,limit in zip(q,origin['joints_rad'],joint_tolerances(policy,side)))
                  and math.dist(pose[:3],origin['pose_m_rad'][:3])<=.0005
                  and _rotation_span([pose,origin['pose_m_rad']])<=rotation_tolerance(policy,side),
                  'Every new body sample must remain within original candidate and peer body anchors')
        reference=opened['sample']['arms'][arm]['gripper']['width_m'] if side==arm else origin['width_m']
        _need(abs(result['passive'][side]['jaw_width_m']-reference)<=.0005 and result['passive'][side]['jaw_enabled'] is True,
              'Current jaw must remain at latest successful opening; peer jaw unchanged')
    result.update(object_release_verified=False,empty_jaw_verified=False,grasp_verified=False,physical_stop_verified=None,
                  jaw_whole_window_observed=False,fresh_device_jaw_baseline_required=True)
    return result


def _reacquisition_envelope(s,h):
    old,candidate=_contact_original(s);e=s['event'];d=json.loads(e['receipt_json']);jaw=d['sample']['arms'][old['arm']]['gripper']
    return dict(schema='piper_supported_reacquisition_v1',arm=old['arm'],source_receipt_sha256=_receipt_digest(candidate),
                opening_event_id=e['event_id'],opening_receipt_sha256=_receipt_digest(d),opening_finished_at=e['finished_at'],
                opening_jaw_anchor=dict(width_m=jaw['width_m'],observed_at=jaw['timestamp'],source=h['opening_journal']))


def reacquisition_source(state):
    p=state['proposal'];_need(p.get('route')=='audited_contact_reacquisition','Contact reacquisition scope required')
    old,candidate=_contact_original(p['snapshot']);source=copy.deepcopy(candidate)
    source['audited_supported_reacquisition']={**p['reacquisition'],'audit_proposal_sha256':p['proposal_sha256']}
    return old,source


def prepare_contact_reacquisition(path,run_id,*,session_log,opening_journal,user_instruction,
                                  passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');s=contact_snapshot(db,run_id)
    now=_number(clock(),'clock');r=s['run'];deadline=r['started_at']+r['max_duration']
    _need(s['scope']['last_time']<=now<deadline and r['steps']+3<=r['max_steps'],'Original budget must cover at most three new closures')
    h=contact_history(session_log,opening_journal,s);a=contact_authorization(user_instruction,s,h,now)
    new=_current_contract(path.parent.parent,s['contract'],require_change=False)
    _need({k for k,v in new['code'].items() if s['contract']['code'].get(k)!=v}<=REPAIR_FILES
          and set(s['contract']['code'])==set(new['code']),'Only reviewed contact reacquisition sources may change')
    evidence=dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    p=dict(schema=CONTACT_SCHEMA,route='audited_contact_reacquisition',database=str(path),run_id=run_id,created_at=now,
           snapshot=s,history=h,authorization=a,reviewed_contract=new,evidence=contact_observed(evidence,s,new,a['recorded_at'],now),
           reacquisition=_reacquisition_envelope(s,h),max_probes=3,nominal_force_N=.2,
           budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
           hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,required_connection_mode='prepare',
           old_target_replay_authorized=False,grasp_transferred=False,object_release_verified=False,empty_jaw_verified=False,
           physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def activate_contact_reacquisition(p,*,project_root,clock=time.time):
    _need(p.get('schema')==CONTACT_SCHEMA and p.get('route')=='audited_contact_reacquisition'
          and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p.get('proposal_sha256'),'Exact contact proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(p['database']);ev=p['evidence']
    _need(path==root/'runs/pair_sessions.sqlite','Canonical ledger required')
    evidence=dict(passive_paths={s:v['source']['path'] for s,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare_contact_reacquisition(path,p['run_id'],session_log=p['history']['session']['path'],opening_journal=p['history']['opening_journal']['path'],
          user_instruction=p['authorization'],clock=lambda:p['created_at'],**evidence)==p,'Reviewed contact proposal sources changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(contact_snapshot(db,p['run_id'])==p['snapshot'],'Original completed opening history changed')
                now=_number(clock(),'clock');_need(p['created_at']<=now<p['budget']['deadline_s'],'Original deadline reached')
                _need(contact_observed(evidence,p['snapshot'],p['reviewed_contract'],p['authorization']['recorded_at'],now)==ev,'Contact admission expired')
                _need(_current_contract(root,p['snapshot']['contract'],require_change=False)==p['reviewed_contract'],'Reviewed sources changed')
                record=dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],old_rows_preserved=True,new_budget_allocated=False,
                            hardware_commands_sent=0,required_connection_mode='prepare',old_target_replay_authorized=False,grasp_transferred=False,
                            object_release_verified=False,empty_jaw_verified=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+CONTACT_TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,'
                    'last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,'
                    'contract_json TEXT NOT NULL,proposal_sha256 TEXT NOT NULL,record_json TEXT NOT NULL)')
                s=p['snapshot'];db.execute('INSERT INTO '+CONTACT_TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (p['run_id'],now,s['scope']['owner'],json.dumps(s['retired_owners']),s['scope']['round_ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'contact reacquisition')))
                check_processes();_need(now<=clock()<p['budget']['deadline_s'],'Original deadline reached')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise


def audit_contact_budget(db,run_id,*,max_steps,max_duration_s):
    latest=db.execute('SELECT * FROM '+CONTACT_TABLE+' ORDER BY ordinal DESC LIMIT 1').fetchone()
    if latest is not None and json.loads(latest['record_json'])['proposal'].get('schema')==SENT_CONTACT_SCHEMA:
        return audit_sent_contact_budget(db,run_id,max_steps=max_steps,max_duration_s=max_duration_s)
    rows=list(db.execute('SELECT * FROM '+CONTACT_TABLE));_need(len(rows)==1,'One contact reacquisition scope required')
    row=rows[0]
    if row['run_id']!=run_id:return False
    r=db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone();record=json.loads(row['record_json']);p=record['proposal'];s=p['snapshot'];b=p['budget']
    _need(p.get('schema')==CONTACT_SCHEMA and p.get('route')=='audited_contact_reacquisition'
          and p['run_id']==run_id and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p['proposal_sha256']==row['proposal_sha256']
          and p['max_probes']==3 and p['nominal_force_N']==.2
          and record['new_contract']==p['reviewed_contract']==json.loads(row['contract_json'])
          and row['previous_owner']==s['scope']['owner'] and json.loads(row['retired_owners_json'])==s['retired_owners']
          and row['round_ordinal']==s['scope']['round_ordinal'] and p['created_at']<=record['activated_at']<=row['last_time']
          and record['activated_at']<b['deadline_s'] and record['hardware_commands_sent']==0
          and record['old_rows_preserved'] is True and record['new_budget_allocated'] is False
          and record['required_connection_mode']=='prepare' and record['old_target_replay_authorized'] is False
          and record['grasp_transferred'] is False and record['empty_jaw_verified'] is False and record['object_release_verified'] is False
          and record.get('physical_stop_verified') is None
          and all(r[k]==s['run'][k] for k in ('run_id','started_at','max_steps','max_duration','contract_json'))
          and b==dict(started_at=r['started_at'],deadline_s=r['started_at']+r['max_duration'],max_steps=r['max_steps'],steps=s['run']['steps'],max_duration_s=r['max_duration'])
          and r['max_steps']==max_steps and r['max_duration']==max_duration_s,'Exact original budget, retirement and restricted route required')
    _need({k for k,v in p['reviewed_contract']['code'].items() if s['contract']['code'].get(k)!=v}<=REPAIR_FILES
          and {k:v for k,v in s['contract'].items() if k!='code'}=={k:v for k,v in p['reviewed_contract'].items() if k!='code'},'Unreviewed task or source change')
    shadow=sqlite3.connect(':memory:');shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()));shadow.execute('DROP TABLE '+CONTACT_TABLE)
        for name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
            if not shadow.execute('SELECT 1 FROM sqlite_master WHERE name=?',(name,)).fetchone():continue
            for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run_id,))):
                if 'owner' in item.keys() and item['owner'] not in s['retired_owners']:
                    _need(item['owner']==row['owner'],'Unexpected later owner');shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run_id))
        _need(contact_snapshot(shadow,run_id)==s,'Original successful opening/fault history changed')
    finally:shadow.close()
    h=contact_history(p['history']['session']['path'],p['history']['opening_journal']['path'],s,source_prefix=p['history']['source_prefix'])
    _need(h==p['history'],'Original opening closure/trace changed')
    contact_authorization(p['authorization'],s,h,p['created_at']);ev=p['evidence']
    evidence=dict(passive_paths={side:v['source']['path'] for side,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(contact_observed(evidence,s,p['reviewed_contract'],p['authorization']['recorded_at'],p['created_at'])==ev
          and p['reacquisition']==_reacquisition_envelope(s,h),'Archived contact admission or original body source changed')
    return True


def _contact_runtime(db,run_id,row,p,events):
    phase='reacquire_required';count=0;maximum=p['max_probes'];consumed=p.get('consumed_probe_count',0)
    _need(type(maximum) is int and type(consumed) is int and 0<=consumed<3 and maximum==3-consumed,'Original contact attempt limit must remain fixed')
    old,_=reacquisition_source(dict(proposal=p));width=p['evidence']['passive']['left']['jaw_width_m'];prior=p.get('prior_sent_target_m')
    for event in events:
        payload=json.loads(event['payload_json'])
        _need(event['owner']==row['owner'] and event['step']==p['budget']['steps']+count+1
              and phase=='reacquire_required' and count<maximum,'Only up to three independently admitted left closures are covered')
        _check_contact_payload(payload,p,width,prior,old)
        count+=1
        if event['status']!='complete' or event['success']!=1:return dict(phase='unresolved',proposal=p,events=[dict(e) for e in events],probe_count=count,consumed_probe_count=consumed)
        d=json.loads(event['receipt_json']);c=d.get('contact_observation',{})
        counts={s:dict(attempted_frames=int(s=='left'),sent_frames=int(s=='left'),blocked_frames=0) for s in SIDES}
        total={s:{k:v*count for k,v in cts.items()} for s,cts in counts.items()}
        _need(d.get('ok') is True and d.get('hardware_commands_sent')==d.get('target_calls_sent')==1
              and d.get('nominal_force_N')==.2 and d.get('errors')==d.get('guard_violations')==[]
              and d.get('transmission_counts')==counts and d.get('session_transmission_counts')==total
              and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
              and c.get('outcome') in ('target_arrived','settled_contact_candidate') and c.get('completion')=='observation_only'
              and d.get('physical_stop_verified') is None and d.get('grasp_verified') is False,
              'Contact probe must retain complete one-frame and unverified contact facts')
        width=_number(d['after']['left']['gripper']['width_m'],'latest jaw');prior=payload['target']
        phase='contact_candidate' if c['outcome']=='settled_contact_candidate' else ('exhausted' if count==maximum else 'reacquire_required')
    return dict(phase=phase,proposal=p,events=[dict(e) for e in events],probe_count=count,consumed_probe_count=consumed,current_width_m=width,prior_target_m=prior)


def _check_contact_payload(payload,p,width,prior,old=None):
    if old is None:old,_=reacquisition_source(dict(proposal=p))
    _need(payload.get('kind')=='gripper' and payload.get('operation')=='grip_supported'
          and payload.get('arm')=='left' and payload.get('grasp_object_id')==old['grasp_object_id']
          and payload.get('reacquisition_proposal_sha256')==p['proposal_sha256']
          and payload.get('probe_support_relation')=='independent_support_present'
          and type(payload.get('probe_support_observation')) is str and 1<=len(payload['probe_support_observation'].strip())<=4000,
          'Only the same supported object and a newly observed left contact probe are admitted')
    target=_number(payload.get('target'),'new closure width');encoded=round(target*1e6)/1e6
    _need(0<=target<=.055 and 0<width-target<=.005+1e-12 and 0<width-encoded<=.005+1e-12,
          'Closure must narrow the current jaw by at most five millimeters')
    if prior is not None:
        floor=min(prior,round(prior*1e6)/1e6)
        _need(target<floor and encoded<floor,'New closure must be strictly below prior requested and encoded targets; no replay')


def _same_derived_measurement(actual, recorded):
    """Only floating arithmetic roundoff in a rederived summary, not a control bound."""
    if isinstance(actual,dict) and isinstance(recorded,dict):
        return set(actual)==set(recorded) and all(_same_derived_measurement(actual[k],recorded[k]) for k in actual)
    if isinstance(actual,list) and isinstance(recorded,list):
        return len(actual)==len(recorded) and all(_same_derived_measurement(a,b) for a,b in zip(actual,recorded))
    if type(actual) is float and type(recorded) is float:
        return math.isclose(actual,recorded,rel_tol=1e-12,abs_tol=1e-15)
    return type(actual) is type(recorded) and actual==recorded


CONTACT_ZERO_TX_KIND = 'supported_contact_zero_tx_fault'
CONTACT_ROUND_REPAIR_FILES = REPAIR_FILES | {'pair_round.py','pair_task_enrollment.py'}


def contact_round_proposal(proposal):
    """Adapt an audited new round to the same restricted runtime; never reset attempts."""
    route=proposal.get('contact_route',{});source=proposal['snapshot']['contact_source']
    expected=proposal['snapshot']['contact_attempts']
    _need(proposal.get('parent_kind')==CONTACT_ZERO_TX_KIND and route==expected
          and route['route']=='audited_contact_reacquisition' and route['stage_attempt_limit']==3
          and 1<=route['consumed_probe_count']<3 and route['max_probes']==3-route['consumed_probe_count'],
          'New round must preserve the exact remaining contact attempts')
    return dict(schema=CONTACT_SCHEMA,route=route['route'],snapshot=source['snapshot'],
                reacquisition=source['reacquisition'],evidence=proposal['recovery_evidence'],
                proposal_sha256=proposal['proposal_sha256'],budget={'steps':0},max_probes=route['max_probes'],
                consumed_probe_count=route['consumed_probe_count'],nominal_force_N=.2)


def contact_zero_tx_snapshot(db,run_id):
    """Audits one known pre-send RGB-expiry attempt, retaining its consumed stage slot."""
    from .pair_ledger import _execution_scope
    table,key,ordinal,scope=_execution_scope(db,run_id,writable=True)
    rows=tables(db);run=next((r for r in rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and scope and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id']
          and table in (CONTACT_TABLE,'pair_rounds'),'Current faulted contact scope required')
    _need(max(rows['pair_runs'],key=lambda r:(r['started_at'],r['run_id']))['run_id']==run_id,
          'Only latest contact run may enroll its successor')
    registration=json.loads(scope['record_json']);parent=registration['proposal']
    if table=='pair_rounds':
        _need(parent.get('parent_kind')==CONTACT_ZERO_TX_KIND,'Only a restricted contact round may continue')
        parent=contact_round_proposal(parent)
    _need(parent.get('route')=='audited_contact_reacquisition'
          and _activated_execution_budget_db(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Full original contact enrollment and budget lineage required')
    _need(not any(e['status']!='complete' for e in rows['pair_events']),'Pending/unknown event blocks contact continuation')
    for historical in rows['pair_runs']:
        _need(sorted(e['step'] for e in rows['pair_events'] if e['run_id']==historical['run_id'])==list(range(1,historical['steps']+1)),
              'All existing run steps must remain contiguous')
    events=[e for e in rows['pair_events'] if e['run_id']==run_id and e['owner']==scope['owner']]
    _need(len(events)==1 and run['steps']==events[0]['step']==parent['budget']['steps']+1,
          'Exactly one terminal local attempt without any earlier send required')
    event=events[0];payload=json.loads(event['payload_json']);receipt=json.loads(event['receipt_json']);device=receipt.get('device_receipt',{})
    old,candidate=reacquisition_source(dict(proposal=parent));arm=old['arm'];zero={side:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for side in SIDES}
    _check_contact_payload(payload,parent,parent['evidence']['passive'][arm]['jaw_width_m'],None,old)
    _need(event['success']==0 and event['owner']==scope['owner']
          and hashlib.sha256(event['payload_json'].encode()).hexdigest()==event['payload_digest']
          and receipt.get('ok') is False and receipt.get('event_id')==event['event_id'] and receipt.get('automatic_retry') is False
          and device.get('ok') is False and device.get('errors')==[{'type':'PairHostError','detail':'Pair RGB scene is stale'}]
          and device.get('hardware_commands_sent')==device.get('target_calls_sent')==0
          and device.get('transmission_counts')==device.get('session_transmission_counts')==zero
          and device.get('guard_violations')==[] and device.get('unresolved_gripper_probe') is None
          and device.get('grasp_states')==dict.fromkeys(SIDES) and device.get('nominal_force_N')==.2
          and all(device.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and not device.get('dispatch_feedback') and not device.get('contact_observation')
          and not device.get('candidate_probe') and not device.get('candidate_measurement')
          and receipt.get('physical_stop_verified') is None,'Only fully known zero-attempt/zero-send RGB expiry is covered')
    from . import arms
    from .retention_receipt import measured_anchor,anchor_deviation
    from .feedback_tolerance import task_policy,joints_within,rotation_tolerance
    policy=task_policy(registration['new_contract']['task'])
    for side in SIDES:
        origin=candidate['candidate_measurement']['anchor'] if side==arm else measured_anchor(candidate['before'][side])
        reference=parent['reacquisition']['opening_jaw_anchor']['width_m'] if side==arm else origin['width_m']
        for label in ('before','after'):
            observed=device[label][side];deviation=anchor_deviation(origin,observed)
            _need(event['began_at']<=observed['timestamp']<=event['finished_at']
                  and (label!='after' or event['finished_at']-observed['timestamp']<=.1)
                  and arms.control_health(observed,now_s=observed['timestamp'],allowed_control_modes=(1,),require_enabled=True)['healthy']
                  and joints_within(policy,side,origin['joints_rad'],observed['joints_rad'])
                  and deviation['position_m']<=.0005 and deviation['rotation_rad']<=rotation_tolerance(policy,side)
                  and abs(observed['gripper']['width_m']-reference)<=.0005,
                  'Pre-send feedback must retain original body/opening-jaw and healthy peer anchors')
    faults=sorted([f for f in rows['pair_faults'] if f['run_id']==run_id and f['owner']==scope['owner']],key=lambda f:f['id'])
    _need(len(faults)==2 and faults[0]['id']==scope['fault_id']
          and [f['reason'] for f in faults]==['RGB shared scene expired before the next guarded operation','execution_receipt_failed']
          and event['began_at']<=faults[0]['at']<=faults[1]['at']==event['finished_at']<=scope['last_time'],
          'Only the original RGB-expiry and failed-receipt faults are covered')
    episodes=[e for e in rows.get('pair_grasp_episodes',[]) if e['run_id']==run_id and e['owner']==scope['owner']]
    _need(len(episodes)==1,'One empty bookkeeping episode required')
    episode=episodes[0];state=json.loads(episode['state_json'])
    from .grasp_episode import new_episode
    empty=new_episode(episode_id=episode['episode_id'],arm=arm,run_id=run_id,owner=scope['owner'],
                      epoch=scope['owner'],object_id=old['grasp_object_id'],created_at=state['created_at'],
                      deadline_at=run['started_at']+run['max_duration'],feedback_policy=policy)
    _need(type(episode['revision']) is int and episode['revision']==0 and episode['arm']==arm
          and run['started_at']<=state['created_at']<=event['began_at']
          and _json_object(state,'old empty episode')==_json_object(empty,'expected empty episode'),
          'Only the exact unbound empty episode, identity and original deadline are covered')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends'):
        _need(not any(r['run_id']==run_id and r.get('owner')==scope['owner'] for r in rows.get(name,[])),
              'Hold, loaded or uncertain sends block contact continuation')
    consumed=parent.get('consumed_probe_count',0)+1
    _need(0<consumed<3 and parent['max_probes']>1,'No contact attempts remain after the failed local attempt')
    retired={r[field] for items in rows.values() for r in items for field in ('owner','previous_owner') if r.get(field)}
    for items in rows.values():
        for r in items:
            if r.get('retired_owners_json'):retired.update(json.loads(r['retired_owners_json']))
    return dict(parent_kind=CONTACT_ZERO_TX_KIND,run=run,scope_table=table,scope_key=key,scope_ordinal=ordinal,
        scope={k:scope[k] for k in scope.keys() if k not in ('record_json','contract_json')},
        scope_record_sha256=hashlib.sha256(scope['record_json'].encode()).hexdigest(),effective_contract=registration['new_contract'],
        retired_owner=scope['owner'],retired_owners=sorted(retired),session_transmission_counts=zero,last_finished_at=event['finished_at'],
        cumulative_prior_steps=sum(r['steps'] for r in rows['pair_runs']),table_sha256={k:_table_sha(v) for k,v in rows.items()},table_rows={k:len(v) for k,v in rows.items()},
        zero_tx_event=event,contact_source={k:parent[k] for k in ('snapshot','reacquisition')},
        contact_attempts=dict(route='audited_contact_reacquisition',stage_attempt_limit=3,consumed_probe_count=consumed,max_probes=3-consumed))


def contact_zero_tx_closed(path,snapshot):
    """Bind closed CLI and a fixed journal prefix; a stopped process is not physical stop."""
    raw,ref=_file(path);manifest=json.loads(raw)
    _need(set(manifest)=={'schema','session_log','dispatch_journal'} and manifest['schema']=='piper_contact_zero_tx_closure_v1',
          'Contact zero-TX closure manifest required')
    raw,sref=_file(manifest['session_log']);rows=[json.loads(line) for line in raw.decode().splitlines()]
    e=snapshot['zero_tx_event'];owner=snapshot['retired_owner'];run=snapshot['run']['run_id']
    _need(rows and rows[0]['kind']=='session_started' and rows[-1]['kind']=='session_ended'
          and rows[-1]['cleanup_errors']==[] and rows[-1].get('automatic_retry') is False
          and [r['sequence'] for r in rows]==list(range(1,len(rows)+1)) and [r['at'] for r in rows]==sorted(r['at'] for r in rows)
          and rows[0]['at']<e['began_at']<e['finished_at']<rows[-1]['at'],'Normally ended failed contact session required')
    results=[r['result'] for r in rows if r['kind']=='result']
    _need(any(r.get('owner')==owner and r.get('run_id')==run and r.get('connection_mode')=='prepare' for r in results),
          'Exact passive-open contact owner required')
    failures=[r for r in results if r.get('event_id')==e['event_id'] and r.get('status')=='fault']
    _need(len(failures)==1 and failures[0]['receipt']==json.loads(e['receipt_json'])
          and failures[0].get('failure_receipt_persistence_error') is None,'Original durable zero-TX failure must equal CLI evidence')
    closes=[r for r in results if r.get('status')=='closed'];_need(len(closes)==1,'One complete normal close required')
    c=closes[0]['cleanup'];zero=snapshot['session_transmission_counts']
    _need(closes[0]['fault_latched'] is True and c.get('requires_fault_latch') is False and c.get('guard_violations')==[]
          and c.get('unresolved_gripper_probe') is None and c.get('grasp_states')==dict.fromkeys(SIDES)
          and c.get('session_transmission_counts')==zero and all(c['arms'][side]['status']=='disconnected' for side in SIDES),
          'Closure must retain known zero lifetime sends and no new contact candidate')
    raw,jref=_file(manifest['dispatch_journal']);index=json.loads(raw);source=Path(index['source']).resolve(strict=True)
    opens=[r for r in rows if r['kind']=='request' and r.get('request',{}).get('op')=='robot_pair_open']
    _need(len(opens)==1 and opens[0]['sequence']==2 and opens[0]['request']['arguments']['run_id']==run
          and opens[0]['request']['arguments']['connection_mode']=='prepare','Exactly one original preparation owner-open request required')
    window=dict(started_at=rows[0]['at'],ended_at=rows[-1]['at'])
    _need(index.get('excluded_telemetry_events')==['feedback','pair_fault_feedback'],'Only named receive-only telemetry may be omitted from the index')
    _need(index['owner_window']==window and type(index['source_size_bytes']) is int and index['source_size_bytes']>0,
          'Whole original owner window and frozen source byte length required')
    remaining=index['source_size_bytes'];hasher=hashlib.sha256();found=[]
    import re
    with source.open('rb') as stream:
        while remaining:
            line=stream.readline(remaining);_need(line,'Original journal prefix truncated');remaining-=len(line);hasher.update(line)
            name=re.search(rb'"event"\s*:\s*"([^"]+)"',line[:200]);_need(name,'Journal event header missing')
            if name.group(1) not in (b'feedback',b'pair_fault_feedback'):
                item=json.loads(line)
                if window['started_at']<=item.get('unix_s',-1)<=window['ended_at']:found.append(item)
    _need(hasher.hexdigest()==index['source_sha256'] and found==index['rows'], 'Frozen owner journal provenance changed')
    _need([r['event'] for r in found]==['connected_passively','pair_ready_observed','pair_shared_scene','pair_dispatch_claimed']
          and found[0]['transmission_counts']==zero and found[1]['hardware_commands_sent']==0
          and found[-1]['event_id']==e['event_id'] and found[-1]['payload']==json.loads(e['payload_json'])
          and e['began_at']<=found[-1]['unix_s']<=e['finished_at'],
          'Only one claim and no transmission intent, return, trace or other action is covered')
    return dict(source=ref,session=sref,dispatch_journal=jref,source_prefix=dict(path=str(source),size_bytes=index['source_size_bytes'],sha256=index['source_sha256']),
                owner_window=window,retired_owner=owner,exit_observed_at=rows[-1]['at'],physical_stop_verified=None)


def contact_zero_tx_observations(evidence,snapshot,contract,now,after=None):
    _need(type(evidence) is dict and set(evidence)=={'passive_paths','rgb_observation','visual_observation'},
          'Current passive pair and RGB evidence required')
    return contact_observed(evidence,snapshot['contact_source']['snapshot'],contract,
                            snapshot['last_finished_at'] if after is None else after,now)


SENT_CONTACT_SCHEMA = 'piper_supported_sent_contact_continuation_v1'
SENT_CONTACT_REPAIR_FILES = REPAIR_FILES | {'single_supervised_actions.py','contact_receipt.py'}


def _sent_contact_body(sample,candidate,contract,*,now,selected='left'):
    from . import arms
    from .retention_receipt import measured_anchor,anchor_deviation
    from .feedback_tolerance import task_policy,joints_within,rotation_tolerance
    policy=task_policy(contract['task'])
    for side in SIDES:
        current=sample[side];anchor=candidate['candidate_measurement']['anchor'] if side==selected else measured_anchor(candidate['before'][side])
        deviation=anchor_deviation(anchor,current)
        _need(arms.control_health(current,now_s=now,allowed_control_modes=(1,),require_enabled=True)['healthy']
              and joints_within(policy,side,anchor['joints_rad'],current['joints_rad'])
              and deviation['position_m']<=.0005 and deviation['rotation_rad']<=rotation_tolerance(policy,side)
              and (side==selected or deviation['jaw_m']<=.0005),
              'Original body or peer jaw/health boundary cannot use a duplicate-reference repair')


def sent_contact_snapshot(db,run_id):
    from .pair_ledger import _execution_scope
    from .grasp_episode import new_episode
    from .feedback_tolerance import task_policy,joints_within,rotation_tolerance
    from .retention_receipt import measured_anchor,anchor_deviation
    table,key,ordinal,scope=_execution_scope(db,run_id,writable=True);rows=tables(db)
    run=next((r for r in rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and table=='pair_rounds' and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id'],
          'Current faulted restricted contact round required')
    registration=json.loads(scope['record_json']);enrolled=registration['proposal']
    _need(enrolled.get('parent_kind')==CONTACT_ZERO_TX_KIND
          and _activated_execution_budget_db(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Full existing restricted contact round and original budget required')
    parent=contact_round_proposal(enrolled);old,candidate=reacquisition_source(dict(proposal=parent))
    _need(parent['consumed_probe_count']==1 and parent['max_probes']==2,'Exactly one prior consumed contact attempt required')
    _need(not any(e['status']!='complete' for e in rows['pair_events']),'Pending/unknown send blocks continuation')
    events=[e for e in rows['pair_events'] if e['run_id']==run_id]
    _need(len(events)==1 and run['steps']==events[0]['step']==1 and events[0]['owner']==scope['owner'],
          'Exactly one failed closure in the current round is covered')
    event=events[0];payload=json.loads(event['payload_json']);receipt=json.loads(event['receipt_json']);d=receipt.get('device_receipt',{})
    _check_contact_payload(payload,parent,parent['evidence']['passive']['left']['jaw_width_m'],None,old)
    counts={side:dict(attempted_frames=int(side=='left'),sent_frames=int(side=='left'),blocked_frames=0) for side in SIDES}
    _need(event['success']==0 and hashlib.sha256(event['payload_json'].encode()).hexdigest()==event['payload_digest']
          and receipt.get('event_id')==event['event_id'] and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and d.get('ok') is False and d.get('errors')==[{'type':'RuntimeError','detail':'left stationary arm exceeded joint/XYZ/SO(3) envelope'}]
          and d.get('hardware_commands_sent')==d.get('target_calls_sent')==1 and d.get('nominal_force_N')==.2
          and d.get('transmission_counts')==d.get('session_transmission_counts')==counts and d.get('guard_violations')==[]
          and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('requested_target')==payload['target'] and d.get('grasp_states')==dict.fromkeys(SIDES)
          and d.get('unresolved_gripper_probe') is None and not d.get('candidate_probe') and not d.get('candidate_measurement')
          and not d.get('contact_observation') and receipt.get('physical_stop_verified') is None,
          'Only one completely returned closure frame with duplicate-body-reference failure is covered')
    for label in ('before','dispatch_feedback','after'):
        _need(all(event['began_at']<=d[label][side]['timestamp']<=event['finished_at'] for side in SIDES),
              'Original action feedback chronology differs')
        _sent_contact_body(d[label],candidate,registration['new_contract'],now=max(d[label][side]['timestamp'] for side in SIDES))
    fresh=measured_anchor(d['before']['left']);deviation=anchor_deviation(fresh,d['after']['left']);policy=task_policy(registration['new_contract']['task'])
    _need(not joints_within(policy,'left',fresh['joints_rad'],d['after']['left']['joints_rad'])
          or deviation['position_m']>.0005 or deviation['rotation_rad']>rotation_tolerance(policy,'left'),
          'The actual failure must conflict only with the duplicate fresh body reference')
    faults=sorted([f for f in rows['pair_faults'] if f['run_id']==run_id],key=lambda f:f['id'])
    _need(len(faults)==2 and all(f['owner']==scope['owner'] for f in faults) and faults[0]['id']==scope['fault_id']
          and [f['reason'] for f in faults]==['Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch','execution_receipt_failed']
          and event['began_at']<=faults[0]['at']<=faults[1]['at']==event['finished_at']<=scope['last_time'],
          'Only the original dispatch failure and failed receipt are covered')
    episodes=[r for r in rows.get('pair_grasp_episodes',[]) if r['run_id']==run_id]
    _need(len(episodes)==1,'Exactly one empty local bookkeeping episode required');episode=episodes[0];state=json.loads(episode['state_json'])
    empty=new_episode(episode_id=episode['episode_id'],arm='left',run_id=run_id,owner=scope['owner'],epoch=scope['owner'],
                      object_id=old['grasp_object_id'],created_at=state['created_at'],deadline_at=run['started_at']+run['max_duration'],feedback_policy=policy)
    _need(episode['owner']==scope['owner'] and episode['arm']=='left' and type(episode['revision']) is int and episode['revision']==0
          and run['started_at']<=state['created_at']<=event['began_at'] and _json_object(empty,'expected empty')==_json_object(state,'original empty'),
          'A sent failure cannot transfer a grasp, candidate or held episode')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends'):
        _need(not any(r['run_id']==run_id for r in rows.get(name,[])),'Hold/loaded/uncertain send cannot use this continuation')
    retired={r[field] for items in rows.values() for r in items for field in ('owner','previous_owner') if r.get(field)}
    for items in rows.values():
        for row in items:
            if row.get('retired_owners_json'):retired.update(json.loads(row['retired_owners_json']))
    return dict(run=run,scope={k:scope[k] for k in scope.keys() if k!='record_json'},scope_record_sha256=hashlib.sha256(scope['record_json'].encode()).hexdigest(),
        event=event,contract=registration['new_contract'],authorization=registration['authorization'],retired_owners=sorted(retired),
        contact_source={k:parent[k] for k in ('snapshot','reacquisition')},
        table_sha256={k:_table_sha(v) for k,v in rows.items()},table_rows={k:len(v) for k,v in rows.items()})


def sent_contact_history(session_log,journal_index,s,*,existing_observation=False):
    raw,sref=_file(session_log);rows=[json.loads(line) for line in raw.decode().splitlines()];e=s['event'];d=json.loads(e['receipt_json'])['device_receipt']
    _need(rows and rows[0]['kind']=='session_started' and rows[-1]['kind']=='session_ended' and rows[-1]['cleanup_errors']==[]
          and rows[-1].get('automatic_retry') is False and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows) and rows[-1]['at']>e['finished_at'], 'Complete closed sent-failure session required')
    results=[r['result'] for r in rows if r['kind']=='result']
    _need(any(r.get('owner')==s['scope']['owner'] and r.get('run_id')==s['run']['run_id'] and r.get('connection_mode')=='prepare' for r in results),
          'Original prepare-only owner binding required')
    failures=[r for r in results if r.get('event_id')==e['event_id'] and r.get('status')=='fault']
    _need(len(failures)==1 and failures[0]['receipt']==json.loads(e['receipt_json']) and failures[0].get('failure_receipt_persistence_error') is None,
          'Durable failed receipt must match original CLI result')
    closes=[r for r in results if r.get('status')=='closed'];_need(len(closes)==1,'One actual resource close required');c=closes[0]['cleanup']
    _need(closes[0]['fault_latched'] is True and c.get('requires_fault_latch') is False and c.get('guard_violations')==[]
          and c.get('unresolved_gripper_probe') is None and c.get('grasp_states')==dict.fromkeys(SIDES)
          and c.get('session_transmission_counts')==d['session_transmission_counts'] and all(c['arms'][side]['status']=='disconnected' for side in SIDES),
          'Exact known single-frame lifetime and no local grasp at closure required')
    later=[r for r in results if r.get('owner')==s['scope']['owner'] and r.get('run_id')==s['run']['run_id'] and r.get('fault_feedback')]
    _need(len(later)==1,'One independent later same-owner feedback receipt required');feedback=later[0]['fault_feedback']
    _need(later[0]['fault_feedback_read_state']=='observed' and later[0].get('fault_feedback_journal_error') is None
          and feedback.get('status')=='observed' and feedback.get('read_errors')=={} and feedback.get('hardware_commands_sent')==0
          and feedback.get('action_event_pending') is None and e['finished_at']<feedback['observed_at_s']<rows[-1]['at'],
          'Later feedback must remain zero-TX independent observation, never retroactive success')
    old,candidate=_contact_original(s['contact_source']['snapshot'])
    _sent_contact_body(feedback['arms'],candidate,s['contract'],now=feedback['observed_at_s'])
    target=json.loads(e['payload_json'])['target'];later_width=feedback['arms']['left']['gripper']['width_m']
    if existing_observation:
        _need(later_width-target>.002,'Existing supported contact must remain distinct from target arrival')
    else:
        _need(abs(later_width-target)<=.002,'Later feedback must remain near the completely sent target')
    raw,jref=_file(journal_index);index=json.loads(raw);source=Path(index['source']).resolve(strict=True)
    window=dict(started_at=rows[0]['at'],ended_at=rows[-1]['at'])
    _need(index['owner_window']==window and type(index['source_size_bytes']) is int and index['source_size_bytes']>0,
          'Frozen complete owner journal window required')
    remaining=index['source_size_bytes'];hasher=hashlib.sha256();found=[];saw_later=False;action_samples=0;later_samples=0
    with source.open('rb') as stream:
        while remaining:
            line=stream.readline(remaining);_need(line,'Original journal prefix truncated');remaining-=len(line);hasher.update(line)
            row=json.loads(line);at=row.get('unix_s',-1)
            if not window['started_at']<=at<=window['ended_at']:continue
            if row['event']=='feedback':
                if e['began_at']<=at<=e['finished_at']:
                    _sent_contact_body({side:row[side] for side in SIDES},candidate,s['contract'],now=at);action_samples+=1
            elif row['event']=='pair_fault_feedback':
                f=row['feedback']
                _need(e['began_at']<=f['observed_at_s']<=rows[-1]['at'] and f.get('read_errors')=={} and f.get('hardware_commands_sent')==0,
                      'Uncertain post-fault feedback cannot qualify body-reference continuation')
                _sent_contact_body(f['arms'],candidate,s['contract'],now=f['observed_at_s']);later_samples+=1
                if f==feedback:saw_later=True
            else:found.append(row)
    _need(hasher.hexdigest()==index['source_sha256'] and found==index['rows'] and saw_later and action_samples>=21 and later_samples>=21,
          'Complete original frames and independent receive history must rederive from frozen journal prefix')
    trace=None
    if existing_observation:
        _need(found[-1]['event']=='bounded_probe_trace','Complete raw failed-probe trace required')
        trace=found[-1];found=found[:-1]
    events=[r['event'] for r in found]
    _need(events[0]=='connected_passively' and events.count('connected_passively')==1
          and events[-3:]==['pair_dispatch_claimed','single_supervised_action_intent','single_supervised_action_sent_unconfirmed']
          and all(name in ('pair_ready_observed','pair_shared_scene') for name in events[1:-3]),
          'Only passive observation followed by one returned closure frame is covered')
    claim,intent,sent=found[-3:];payload=json.loads(e['payload_json']);rawtarget=round(target*1e6)
    _need(claim['event_id']==e['event_id'] and claim['payload']==payload and intent['arm']=='left' and intent['kind']==sent['kind']=='gripper'
          and intent['target']==target and intent['frames']==[{'id':0x159,'data_hex':(rawtarget.to_bytes(4,'big',signed=True)+bytes.fromhex('00c80100')).hex()}]
          and e['began_at']<=claim['unix_s']<=intent['unix_s']<=sent['finished_unix_s']<=sent['unix_s']<=e['finished_at'],
          'Exact known once-only frame and failed event binding required')
    result=dict(session=sref,journal_index=jref,source_prefix=dict(path=str(source),size_bytes=index['source_size_bytes'],sha256=index['source_sha256']),
                ended_at=rows[-1]['at'],later_feedback_sha256=_receipt_digest(feedback),later_feedback_at=feedback['observed_at_s'],
                later_jaw_width_m=later_width,original_body_action_samples=action_samples,original_body_post_fault_samples=later_samples,physical_stop_verified=None)
    if existing_observation:
        from .contact_receipt import classify_gripper_probe
        from .retention_receipt import measured_anchor
        from .feedback_tolerance import task_policy
        _need(trace['arm']=='left' and trace['release'] is False and trace['requested_width_m']==target
              and trace['sha256']==_receipt_digest(trace['trace'])==d['contact_observation']['trace_summary']['sha256']
              and trace['sent_at']==sent['finished_unix_s'] and sent['unix_s']<=trace['unix_s']<=e['finished_at'],
              'Failed full trace must bind the exact prior sent frame and failed receipt')
        refs={side:{k:v for k,v in (candidate['candidate_measurement']['anchor'] if side=='left' else measured_anchor(candidate['before'][side])).items()
                    if k in ('joints_rad','pose_m_rad')} for side in SIDES}
        classified=classify_gripper_probe(arm='left',requested_width_m=target,sent_at=trace['sent_at'],
            baseline_samples=trace['trace']['baseline'],post_samples=trace['trace']['post'],
            feedback_policy=task_policy(s['contract']['task']),stationary_body_reference=refs)
        _need(_same_derived_measurement(classified,{k:v for k,v in d['contact_observation'].items() if k!='trace_summary'}),
              'Original unconfirmed classification must rederive unchanged from complete raw trace')
        for sample in trace['trace']['baseline']+trace['trace']['post']:
            _sent_contact_body(sample['arms'],candidate,s['contract'],now=sample['observed_at_s'])
        result.update(failed_sent_at=trace['sent_at'],failed_trace_sha256=trace['sha256'],original_classification_preserved=True)
    return result


def sent_contact_authorization(path,s):
    raw,ref=_file(path);value=json.loads(raw);original=s['authorization'];run=s['run']
    _need(all(value.get(k)==original[k] for k in ('source','message_id','statement','received_at','decision','budget_start_policy'))
          and value.get('source')=='user_message' and value.get('max_steps')==run['max_steps'] and value.get('max_duration_s')==run['max_duration']
          and original['new_budget']==dict(started_at=run['started_at'],max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Existing actual user authorization for this exact current round/budget required')
    return dict(source=ref,instruction=value,current_round_authorization_sha256=_sha(original),
                scope='Explicit offline repair and newly admitted remaining contact attempt in the same unchanged round')


def sent_contact_observed(evidence,s,contract,after,now,history):
    result=_observations(**evidence,contract=contract,after=after,now=now)
    old,candidate=_contact_original(s['contact_source']['snapshot'])
    from .feedback_tolerance import task_policy,joint_tolerances,rotation_tolerance
    from .retention_receipt import measured_anchor
    from .contact_receipt import _rotation_span
    policy=task_policy(contract['task'])
    for side,path in evidence['passive_paths'].items():
        data=json.loads(_file(path)[0]);anchor=candidate['candidate_measurement']['anchor'] if side=='left' else measured_anchor(candidate['before'][side])
        for row in data['pose_trace']:
            q=[row['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            pose=[row['end_pose_raw'][k]*(1e-6 if i<3 else math.pi/180000) for i,k in enumerate(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'))]
            _need(all(abs(a-b)<=limit for a,b,limit in zip(q,anchor['joints_rad'],joint_tolerances(policy,side)))
                  and math.dist(pose[:3],anchor['pose_m_rad'][:3])<=.0005 and _rotation_span([pose,anchor['pose_m_rad']])<=rotation_tolerance(policy,side),
                  'Every new body sample must retain the original candidate and peer anchors')
        reference=history['later_jaw_width_m'] if side=='left' else anchor['width_m']
        _need(result['passive'][side]['jaw_enabled'] is True and abs(result['passive'][side]['jaw_width_m']-reference)<=.0005,
              'New residual jaw must match independently observed later width; peer unchanged')
    result.update(object_release_verified=False,empty_jaw_verified=False,grasp_verified=False,physical_stop_verified=None,
                  jaw_whole_window_observed=False,fresh_device_jaw_baseline_required=True)
    return result


def _sent_contact_envelope(s,evidence):
    envelope=copy.deepcopy(s['contact_source']['reacquisition']);arm='left';passive=evidence['passive'][arm]
    raw=json.loads(_file(passive['source']['path'])[0]);event=s['event']
    envelope['completed_probe_continuation']=dict(schema='piper_completed_contact_continuation_v1',arm=arm,
        failed_event_id=event['event_id'],failed_receipt_sha256=_receipt_digest(json.loads(event['receipt_json'])),
        prior_sent_target_m=json.loads(event['payload_json'])['target'],failed_finished_at=event['finished_at'],consumed_probe_count=2,remaining_probe_count=1,
        residual_jaw_anchor=dict(width_m=passive['jaw_width_m'],observed_at=raw['feedback']['PiperMsgGripperFeedBack']['received_at_s'],source=passive['source']))
    return envelope


def prepare_sent_contact_continuation(path,run_id,*,session_log,journal_index,user_instruction,
                                      passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');s=sent_contact_snapshot(db,run_id)
    now=_number(clock(),'clock');r=s['run'];deadline=r['started_at']+r['max_duration']
    _need(s['scope']['last_time']<=now<deadline and r['steps']<r['max_steps'],'Same original task budget must remain')
    h=sent_contact_history(session_log,journal_index,s);a=sent_contact_authorization(user_instruction,s)
    new=_current_contract(path.parent.parent,s['contract'],require_change=False)
    _need(set(new['code'])==set(s['contract']['code']) and {k for k,v in new['code'].items() if s['contract']['code'][k]!=v}<=SENT_CONTACT_REPAIR_FILES,
          'Only reviewed original-body-reference/contact-continuation code may change')
    evidence=dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    ev=sent_contact_observed(evidence,s,new,h['ended_at'],now,h)
    p=dict(schema=SENT_CONTACT_SCHEMA,route='audited_contact_reacquisition',database=str(path),run_id=run_id,created_at=now,
           snapshot=s,history=h,authorization=a,reviewed_contract=new,evidence=ev,reacquisition=_sent_contact_envelope(s,ev),
           max_probes=1,consumed_probe_count=2,stage_attempt_limit=3,nominal_force_N=.2,
           budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
           hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,required_connection_mode='prepare',
           old_target_replay_authorized=False,grasp_transferred=False,object_release_verified=False,empty_jaw_verified=False,physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def _sent_contact_runtime_proposal(p):
    _need(p.get('schema')==SENT_CONTACT_SCHEMA and p['max_probes']==1 and p['consumed_probe_count']==2 and p['stage_attempt_limit']==3,
          'Exactly one original remaining contact attempt required')
    return {**p,'snapshot':p['snapshot']['contact_source']['snapshot'],
            'prior_sent_target_m':p['reacquisition']['completed_probe_continuation']['prior_sent_target_m']}


def activate_sent_contact_continuation(p,*,project_root,clock=time.time):
    _need(p.get('schema')==SENT_CONTACT_SCHEMA and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p.get('proposal_sha256'),
          'Exact reviewed completed-send continuation proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(p['database']);ev=p['evidence']
    _need(path==root/'runs/pair_sessions.sqlite','Canonical ledger required')
    evidence=dict(passive_paths={s:v['source']['path'] for s,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare_sent_contact_continuation(path,p['run_id'],session_log=p['history']['session']['path'],journal_index=p['history']['journal_index']['path'],
          user_instruction=p['authorization']['source']['path'],clock=lambda:p['created_at'],**evidence)==p,'Original reviewed history or evidence changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(sent_contact_snapshot(db,p['run_id'])==p['snapshot'],'Original sent failure history changed')
                now=_number(clock(),'clock');_need(p['created_at']<=now<p['budget']['deadline_s'],'Original deadline reached')
                _need(sent_contact_observed(evidence,p['snapshot'],p['reviewed_contract'],p['history']['ended_at'],now,p['history'])==ev,'Current admission expired')
                _need(_current_contract(root,p['snapshot']['contract'],require_change=False)==p['reviewed_contract'],'Reviewed sources changed')
                record=dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],old_rows_preserved=True,new_budget_allocated=False,
                            hardware_commands_sent=0,required_connection_mode='prepare',old_target_replay_authorized=False,grasp_transferred=False,
                            object_release_verified=False,empty_jaw_verified=False,physical_stop_verified=None)
                ordinal=db.execute('SELECT COALESCE(MAX(ordinal),0)+1 FROM '+CONTACT_TABLE).fetchone()[0];s=p['snapshot']
                db.execute('INSERT INTO '+CONTACT_TABLE+' VALUES(?,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (ordinal,p['run_id'],now,s['scope']['owner'],json.dumps(s['retired_owners']),s['scope']['ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'sent contact continuation')))
                check_processes();_need(now<=clock()<p['budget']['deadline_s'],'Original deadline reached')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise


def audit_sent_contact_budget(db,run_id,*,max_steps,max_duration_s):
    row=db.execute('SELECT * FROM '+CONTACT_TABLE+' ORDER BY ordinal DESC LIMIT 1').fetchone()
    if row is None or row['run_id']!=run_id:return False
    record=json.loads(row['record_json']);p=record['proposal'];s=p['snapshot'];b=p['budget'];r=db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone()
    _need(p.get('schema')==SENT_CONTACT_SCHEMA and p['route']=='audited_contact_reacquisition'
          and p['run_id']==run_id and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p['proposal_sha256']==row['proposal_sha256']
          and record['new_contract']==p['reviewed_contract']==json.loads(row['contract_json'])
          and row['previous_owner']==s['scope']['owner'] and row['round_ordinal']==s['scope']['ordinal']
          and json.loads(row['retired_owners_json'])==s['retired_owners'] and p['created_at']<=record['activated_at']<=row['last_time'] and record['activated_at']<b['deadline_s']
          and all(record.get(k) is False for k in ('new_budget_allocated','old_target_replay_authorized','grasp_transferred','object_release_verified','empty_jaw_verified'))
          and record.get('old_rows_preserved') is True and record.get('hardware_commands_sent')==0 and record.get('physical_stop_verified') is None
          and record.get('required_connection_mode')=='prepare' and p['max_probes']==1 and p['consumed_probe_count']==2 and p['stage_attempt_limit']==3
          and all(r[k]==s['run'][k] for k in ('run_id','contract_json','started_at','max_duration','max_steps'))
          and b==dict(started_at=r['started_at'],deadline_s=r['started_at']+r['max_duration'],max_steps=r['max_steps'],steps=s['run']['steps'],max_duration_s=r['max_duration'])
          and r['max_steps']==max_steps and r['max_duration']==max_duration_s,'Exact original budget, failed attempt and remaining contact allowance required')
    _need({k:v for k,v in s['contract'].items() if k!='code'}=={k:v for k,v in p['reviewed_contract'].items() if k!='code'}
          and set(s['contract']['code'])==set(p['reviewed_contract']['code'])
          and {k for k,v in p['reviewed_contract']['code'].items() if s['contract']['code'][k]!=v}<=SENT_CONTACT_REPAIR_FILES,
          'Unreviewed task or source repair')
    shadow=sqlite3.connect(':memory:');shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()));shadow.execute('DELETE FROM '+CONTACT_TABLE+' WHERE ordinal=?',(row['ordinal'],))
        for name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
            if not shadow.execute('SELECT 1 FROM sqlite_master WHERE name=?',(name,)).fetchone():continue
            for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run_id,))):
                if item['owner'] not in s['retired_owners']:
                    _need(item['owner']==row['owner'],'Unknown successor owner');shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run_id))
        _need(sent_contact_snapshot(shadow,run_id)==s,'Original one-frame failure and previous scope must stay exact')
    finally:shadow.close()
    h=sent_contact_history(p['history']['session']['path'],p['history']['journal_index']['path'],s)
    _need(h==p['history'] and sent_contact_authorization(p['authorization']['source']['path'],s)==p['authorization'],'Original closure or current-round authorization changed')
    ev=p['evidence'];evidence=dict(passive_paths={side:v['source']['path'] for side,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(sent_contact_observed(evidence,s,p['reviewed_contract'],h['ended_at'],p['created_at'],h)==ev
          and _sent_contact_envelope(s,ev)==p['reacquisition'],'Original source body and separate residual jaw admission changed')
    return True


OBSERVATION_TABLE = 'pair_supported_contact_observations'
OBSERVATION_SCHEMA = 'piper_existing_supported_contact_scope_v1'
OBSERVATION_ROUTE = 'audited_supported_contact_observation'
OBSERVATION_REPAIR_FILES = SENT_CONTACT_REPAIR_FILES | {'host_grasp.py','grasp_episode.py','supported_contact_measurement.py'}


def existing_contact_snapshot(db,run_id):
    """Audit an exhausted, completely sent supported closure; never promote its result."""
    from .pair_ledger import _execution_scope
    from .grasp_episode import new_episode
    from .feedback_tolerance import task_policy
    rows=tables(db)
    _need(OBSERVATION_TABLE not in rows,'Existing-contact observation enrollment already consumed')
    table,key,ordinal,scope=_execution_scope(db,run_id,writable=True)
    run=next((r for r in rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and table==CONTACT_TABLE and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id'],
          'Current faulted sent-contact continuation required')
    record=json.loads(scope['record_json']);parent=record['proposal']
    _need(parent.get('schema')==SENT_CONTACT_SCHEMA and parent['max_probes']==1 and parent['consumed_probe_count']==2
          and _activated_execution_budget_db(db,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Original same-budget contact lineage and three consumed attempts required')
    _need(not any(e['status']!='complete' for e in rows['pair_events']),'Pending or unknown sends block contact observation')
    events=[e for e in rows['pair_events'] if e['run_id']==run_id and e['step']>parent['budget']['steps']]
    _need(len(events)==1 and events[0]['step']==run['steps']==parent['budget']['steps']+1 and events[0]['owner']==scope['owner'],
          'Exactly the final consumed closure is covered')
    event=events[0];payload=json.loads(event['payload_json']);receipt=json.loads(event['receipt_json']);d=receipt.get('device_receipt',{});c=d.get('contact_observation',{})
    normalized=_sent_contact_runtime_proposal(parent);old,candidate=reacquisition_source(dict(proposal=normalized))
    candidate={k:v for k,v in candidate.items() if k!='audited_supported_reacquisition'}
    _check_contact_payload(payload,normalized,parent['evidence']['passive']['left']['jaw_width_m'],normalized['prior_sent_target_m'],old)
    counts={side:dict(attempted_frames=int(side=='left'),sent_frames=int(side=='left'),blocked_frames=0) for side in SIDES}
    _need(event['success']==0 and hashlib.sha256(event['payload_json'].encode()).hexdigest()==event['payload_digest']
          and receipt.get('event_id')==event['event_id'] and receipt.get('ok') is False and receipt.get('automatic_retry') is False
          and receipt.get('physical_stop_verified') is None and d.get('ok') is False
          and d.get('errors')==[{'type':'RuntimeError','detail':"Probe response unconfirmed: ['"+REASON+"']"}]
          and d.get('hardware_commands_sent')==d.get('target_calls_sent')==1 and d.get('nominal_force_N')==.2
          and d.get('transmission_counts')==d.get('session_transmission_counts')==counts and d.get('guard_violations')==[]
          and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('requested_target')==payload['target'] and d.get('grasp_states')==dict.fromkeys(SIDES)
          and d.get('unresolved_gripper_probe') is None and d.get('candidate_probe') is None and d.get('candidate_measurement') is None
          and c.get('outcome')=='unconfirmed' and c.get('reasons')==[REASON] and c.get('completion')=='observation_only'
          and c.get('observation_window_complete') is True and c.get('target_may_remain_active') is True
          and c.get('arrival_confirmed') is False and c.get('requested_width_m')==payload['target'],
          'Only a known once-sent complete window with unconfirmed response and no grasp is covered')
    for label in ('before','dispatch_feedback','after'):
        _need(all(event['began_at']<=d[label][side]['timestamp']<=event['finished_at'] for side in SIDES),'Original feedback chronology changed')
        _sent_contact_body(d[label],candidate,record['new_contract'],now=max(d[label][side]['timestamp'] for side in SIDES))
    faults=sorted([f for f in rows['pair_faults'] if f['run_id']==run_id and f['owner']==scope['owner']],key=lambda f:f['id'])
    _need(len(faults)==2 and faults[0]['id']==scope['fault_id']
          and [f['reason'] for f in faults]==['Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch','execution_receipt_failed']
          and event['began_at']<=faults[0]['at']<=faults[1]['at']==event['finished_at']<=scope['last_time'],
          'Only the complete unconfirmed response and its durable failed receipt are covered')
    episodes=[e for e in rows.get('pair_grasp_episodes',[]) if e['run_id']==run_id and e['owner']==scope['owner']]
    _need(len(episodes)==1,'One unbound empty bookkeeping episode required');episode=episodes[0];state=json.loads(episode['state_json'])
    empty=new_episode(episode_id=episode['episode_id'],arm='left',run_id=run_id,owner=scope['owner'],epoch=scope['owner'],object_id=old['grasp_object_id'],
                      created_at=state['created_at'],deadline_at=run['started_at']+run['max_duration'],feedback_policy=task_policy(record['new_contract']['task']))
    _need(type(episode['revision']) is int and episode['revision']==0 and episode['arm']=='left'
          and run['started_at']<=state['created_at']<=event['began_at'] and _json_object(empty,'expected empty')==_json_object(state,'old empty'),
          'No inherited candidate, retained grasp or loaded episode is admissible')
    for name in ('pair_holds','pair_hold_requests','pair_hold_frames','pair_joint_sends'):
        _need(not any(r['run_id']==run_id and r.get('owner')==scope['owner'] for r in rows.get(name,[])),'Hold/loaded/uncertain send history is not covered')
    retired={r[field] for items in rows.values() for r in items for field in ('owner','previous_owner') if r.get(field)}
    for items in rows.values():
        for row in items:
            if row.get('retired_owners_json'):retired.update(json.loads(row['retired_owners_json']))
    return dict(run=run,scope={k:scope[k] for k in scope.keys() if k!='record_json'},scope_record_sha256=hashlib.sha256(scope['record_json'].encode()).hexdigest(),
                event=event,contract=record['new_contract'],authorization=parent['snapshot']['authorization'],retired_owners=sorted(retired),
                contact_source=parent['snapshot']['contact_source'],table_sha256={k:_table_sha(v) for k,v in rows.items()},table_rows={k:len(v) for k,v in rows.items()})


def bilateral_contact_authorization(path,s,history,now):
    from datetime import datetime,timezone
    raw,ref=_file(path);value=json.loads(raw)
    _need(value.get('source')=='user_message' and value.get('statement')=='两根夹指都贴住充电器'
          and value.get('scope')==dict(run_id=s['run']['run_id'],after_event_id=s['event']['event_id'],arm='left',object_id='white_charger'),
          'Exact on-site bilateral contact statement for this failed event and object required')
    _identifier(value.get('message_id'),'user contact message')
    at=datetime.strptime(value['received_at_utc'],'%Y-%m-%d %H:%M:%S UTC').replace(tzinfo=timezone.utc).timestamp()
    _need(max(history['ended_at'],s['event']['finished_at'])<at<=now,'Bilateral statement must follow this closed failure and precede current admission')
    return dict(source=ref,instruction=value,recorded_at=at,scope='On-site bilateral contact report, not force calibration, grasp strength or physical stop')


def _existing_contact_contract(old,new):
    _need({k:v for k,v in old.items() if k!='code'}=={k:v for k,v in new.items() if k!='code'}
          and set(old['code'])<=set(new['code']) and set(new['code'])-set(old['code'])<={'supported_contact_measurement.py'}
          and {k for k,v in new['code'].items() if old['code'].get(k)!=v}<=OBSERVATION_REPAIR_FILES,
          'Only reviewed existing-contact observation code may change; task and limits remain frozen')


def _existing_contact_authorization(path,s):
    result=sent_contact_authorization(path,s)
    result['scope']='Explicit offline repair and one newly admitted zero-TX contact observation in the same unchanged round; no further closure'
    return result


def _existing_contact_envelope(s,ev,h,bilateral):
    old,candidate=_contact_original(s['contact_source']['snapshot']);passive=ev['passive']['left'];raw=json.loads(_file(passive['source']['path'])[0]);event=s['event']
    return dict(schema='piper_existing_supported_contact_admission_v1',arm='left',source_receipt_sha256=_receipt_digest(candidate),
                failed_event_id=event['event_id'],failed_receipt_sha256=_receipt_digest(json.loads(event['receipt_json'])),
                prior_sent_target_m=json.loads(event['payload_json'])['target'],failed_sent_at=h['failed_sent_at'],failed_finished_at=event['finished_at'],
                consumed_probe_count=3,remaining_probe_count=0,
                residual_jaw_anchor=dict(width_m=passive['jaw_width_m'],observed_at=raw['feedback']['PiperMsgGripperFeedBack']['received_at_s'],source=passive['source']),
                bilateral_contact_source=bilateral['source'])


def current_contact_source(state):
    p=state['proposal'];_need(p.get('schema')==OBSERVATION_SCHEMA and p.get('route')==OBSERVATION_ROUTE,'Existing-contact observation scope required')
    old,candidate=_contact_original(p['snapshot']['contact_source']['snapshot']);source=copy.deepcopy(candidate)
    source['audited_existing_contact']={**p['existing_contact'],'audit_proposal_sha256':p['proposal_sha256']}
    return old,source


def prepare_existing_contact_observation(path,run_id,*,session_log,journal_index,user_instruction,bilateral_contact,
                                         passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');s=existing_contact_snapshot(db,run_id)
    now=_number(clock(),'clock');r=s['run'];deadline=r['started_at']+r['max_duration']
    _need(s['scope']['last_time']<=now<deadline and r['steps']<r['max_steps'],'Original budget must cover one zero-TX observation')
    h=sent_contact_history(session_log,journal_index,s,existing_observation=True);a=_existing_contact_authorization(user_instruction,s)
    bilateral=bilateral_contact_authorization(bilateral_contact,s,h,now);new=_current_contract(path.parent.parent,s['contract'],require_change=False)
    _existing_contact_contract(s['contract'],new)
    evidence=dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    ev=sent_contact_observed(evidence,s,new,bilateral['recorded_at'],now,h)
    _need(ev['passive']['left']['jaw_width_m']-json.loads(s['event']['payload_json'])['target']>.002,'Residual jaw must remain distinct from prior target arrival')
    p=dict(schema=OBSERVATION_SCHEMA,route=OBSERVATION_ROUTE,database=str(path),run_id=run_id,created_at=now,snapshot=s,history=h,authorization=a,
           bilateral_contact=bilateral,reviewed_contract=new,evidence=ev,existing_contact=_existing_contact_envelope(s,ev,h,bilateral),
           max_observations=1,max_probes=0,consumed_probe_count=3,stage_attempt_limit=3,
           budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
           hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,required_connection_mode='prepare',
           old_target_replay_authorized=False,grasp_transferred=False,object_release_verified=False,empty_jaw_verified=False,physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def _observation_evidence(p):
    ev=p['evidence']
    return dict(passive_paths={side:v['source']['path'] for side,v in ev['passive'].items()},rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])


def activate_existing_contact_observation(p,*,project_root,clock=time.time):
    _need(p.get('schema')==OBSERVATION_SCHEMA and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p.get('proposal_sha256'),
          'Exact reviewed zero-TX observation proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(p['database']);evidence=_observation_evidence(p)
    _need(path==root/'runs/pair_sessions.sqlite','Canonical ledger path required')
    _need(prepare_existing_contact_observation(path,p['run_id'],session_log=p['history']['session']['path'],journal_index=p['history']['journal_index']['path'],
          user_instruction=p['authorization']['source']['path'],bilateral_contact=p['bilateral_contact']['source']['path'],clock=lambda:p['created_at'],**evidence)==p,
          'Reviewed closed failure, bilateral statement or admission changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(existing_contact_snapshot(db,p['run_id'])==p['snapshot'],'Original full failure history changed')
                now=_number(clock(),'clock');_need(p['created_at']<=now<p['budget']['deadline_s'],'Original deadline reached')
                _need(sent_contact_observed(evidence,p['snapshot'],p['reviewed_contract'],p['bilateral_contact']['recorded_at'],now,p['history'])==p['evidence'],
                      'Current passive/RGB contact admission expired')
                _need(_current_contract(root,p['snapshot']['contract'],require_change=False)==p['reviewed_contract'],'Reviewed code changed')
                record=dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],old_rows_preserved=True,new_budget_allocated=False,
                            hardware_commands_sent=0,required_connection_mode='prepare',old_target_replay_authorized=False,grasp_transferred=False,
                            object_release_verified=False,empty_jaw_verified=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+OBSERVATION_TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,'
                    'last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,'
                    'contract_json TEXT NOT NULL,proposal_sha256 TEXT NOT NULL,record_json TEXT NOT NULL)')
                s=p['snapshot'];db.execute('INSERT INTO '+OBSERVATION_TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (p['run_id'],now,s['scope']['owner'],json.dumps(s['retired_owners']),s['scope']['round_ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'existing contact observation')))
                check_processes();_need(now<=clock()<p['budget']['deadline_s'],'Original deadline reached')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise


def audit_existing_contact_budget(db,run_id,*,max_steps,max_duration_s):
    row=db.execute('SELECT * FROM '+OBSERVATION_TABLE+' ORDER BY ordinal DESC LIMIT 1').fetchone()
    if row is None or row['run_id']!=run_id:return False
    record=json.loads(row['record_json']);p=record['proposal'];s=p['snapshot'];b=p['budget'];r=db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone()
    _need(p.get('schema')==OBSERVATION_SCHEMA and p['route']==OBSERVATION_ROUTE and p['run_id']==run_id
          and _sha({k:v for k,v in p.items() if k!='proposal_sha256'})==p['proposal_sha256']==row['proposal_sha256']
          and record['new_contract']==p['reviewed_contract']==json.loads(row['contract_json'])
          and row['previous_owner']==s['scope']['owner'] and row['round_ordinal']==s['scope']['round_ordinal']
          and json.loads(row['retired_owners_json'])==s['retired_owners'] and p['created_at']<=record['activated_at']<=row['last_time']
          and record['activated_at']<b['deadline_s'] and p['max_observations']==1 and p['max_probes']==0
          and p['consumed_probe_count']==p['stage_attempt_limit']==3
          and all(record.get(k) is False for k in ('new_budget_allocated','old_target_replay_authorized','grasp_transferred','object_release_verified','empty_jaw_verified'))
          and record.get('old_rows_preserved') is True and record.get('hardware_commands_sent')==0 and record.get('physical_stop_verified') is None
          and record.get('required_connection_mode')=='prepare'
          and all(r[k]==s['run'][k] for k in ('run_id','contract_json','started_at','max_duration','max_steps'))
          and b==dict(started_at=r['started_at'],deadline_s=r['started_at']+r['max_duration'],max_steps=r['max_steps'],steps=s['run']['steps'],max_duration_s=r['max_duration'])
          and r['max_steps']==max_steps and r['max_duration']==max_duration_s,'Original budget, consumed attempts and one zero-TX observation must remain fixed')
    _existing_contact_contract(s['contract'],p['reviewed_contract'])
    shadow=sqlite3.connect(':memory:');shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()));shadow.execute('DROP TABLE '+OBSERVATION_TABLE)
        for name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
            if not shadow.execute('SELECT 1 FROM sqlite_master WHERE name=?',(name,)).fetchone():continue
            for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run_id,))):
                if item['owner'] not in s['retired_owners']:
                    _need(item['owner']==row['owner'],'Unknown observation successor owner');shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run_id))
        _need(existing_contact_snapshot(shadow,run_id)==s,'Original failed classification, exhausted stage or prior scope changed')
    finally:shadow.close()
    h=sent_contact_history(p['history']['session']['path'],p['history']['journal_index']['path'],s,existing_observation=True)
    _need(h==p['history'] and _existing_contact_authorization(p['authorization']['source']['path'],s)==p['authorization'],'Historical source or original budget authorization changed')
    _need(bilateral_contact_authorization(p['bilateral_contact']['source']['path'],s,h,p['created_at'])==p['bilateral_contact'],'Bilateral contact source changed')
    _need(sent_contact_observed(_observation_evidence(p),s,p['reviewed_contract'],p['bilateral_contact']['recorded_at'],p['created_at'],h)==p['evidence']
          and _existing_contact_envelope(s,p['evidence'],h,p['bilateral_contact'])==p['existing_contact'],'Archived admission or zero-TX source changed')
    return True


def _check_observation_payload(payload,p):
    old,_=current_contact_source(dict(proposal=p));request=payload.get('request',{})
    _need(payload.get('arm')=='left' and payload.get('kind')=='supported_contact_observe'
          and payload.get('source_event_id')==p['snapshot']['event']['event_id']
          and payload.get('observation_proposal_sha256')==p['proposal_sha256']
          and request.get('operation')=='supported_contact_observe' and request.get('object_id')==old['grasp_object_id']
          and request.get('contact_relation')=='bilateral_finger_contact' and request.get('support_relation')=='independent_support_present'
          and type(request.get('visual_description')) is str and 1<=len(request['visual_description'].strip())<=4000,
          'Only a new bilateral supported-contact observation for the same object is admitted')
    _identifier(request.get('observation_id'),'current RGB observation')


def _existing_contact_runtime(row,p,events):
    _need(p.get('schema')==OBSERVATION_SCHEMA and p.get('route')==OBSERVATION_ROUTE
          and type(p['max_observations']) is int and p['max_observations']==1 and type(p['max_probes']) is int and p['max_probes']==0
          and type(p['consumed_probe_count']) is int and p['consumed_probe_count']==p['stage_attempt_limit']==3,
          'Existing contact route cannot restore closure attempts')
    _need(len(events)<=1,'Only one zero-TX observation is admitted')
    result=dict(phase='contact_observation_required',proposal=p,events=[dict(e) for e in events],probe_count=0,consumed_probe_count=3)
    if not events:return result
    event=events[0];payload=json.loads(event['payload_json']);_check_observation_payload(payload,p)
    _need(event['owner']==row['owner'] and event['step']==p['budget']['steps']+1,'Exact current observation owner and next step required')
    if event['status']!='complete' or event['success']!=1:return {**result,'phase':'unresolved'}
    d=json.loads(event['receipt_json']);zero={side:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for side in SIDES}
    c=d.get('current_contact_candidate',{});m=d.get('candidate_measurement',{});source=p['existing_contact']
    old,original=current_contact_source(dict(proposal=p));anchor=original['candidate_measurement']['anchor']
    _need(d.get('ok') is True and d.get('status')=='observed_supported_contact_candidate' and d.get('completion_mode')=='supported_contact_observe'
          and d.get('candidate_basis')=='existing_target_observation' and d.get('nominal_force_N') is None
          and d.get('audited_existing_contact')==original['audited_existing_contact'] and d.get('loaded') is False
          and d.get('hardware_commands_sent')==d.get('target_calls_sent')==0 and d.get('transmission_counts')==d.get('session_transmission_counts')==zero
          and all(type(d.get(k)) is int for k in ('hardware_commands_sent','target_calls_sent','enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and all(type(v) is int for key in ('transmission_counts','session_transmission_counts') for counts in d[key].values() for v in counts.values())
          and d.get('errors')==d.get('guard_violations')==[] and d.get('physical_stop_verified') is None and d.get('grasp_verified') is False
          and all(d.get(k)==0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('candidate_probe') is None and c.get('schema')=='piper_existing_supported_contact_candidate_v1'
          and c.get('basis')=='existing_target_observation' and c.get('completion')=='observation_only' and c.get('target_may_remain_active') is True
          and c.get('existing_target_ref')==dict(event_id=source['failed_event_id'],receipt_sha256=source['failed_receipt_sha256'],
              sent_at=source['failed_sent_at'],finished_at=source['failed_finished_at'],requested_width_m=source['prior_sent_target_m'])
          and c.get('trace_sha256')==m.get('trace_sha256') and c.get('observed_width_m')==m.get('observed',{}).get('width_m')
          and c.get('started_at')==m.get('started_at') and c.get('completed_at')==m.get('ended_at')
          and event['began_at']<=m['started_at']<m['ended_at']<=event['finished_at'] and m['ended_at']-m['started_at']>=3
          and m.get('health')=='healthy' and m.get('mode')=='stationary' and m.get('sample_count',0)>=21 and m.get('feedback_advances',0)>=20
          and all(m.get('anchor',{}).get(key)==anchor[key] for key in ('joints_rad','pose_m_rad'))
          and _number(c.get('minimum_target_gap_m'),'observed target gap')>.002,
          'New candidate must arise from fresh zero-TX observation; original failed target remains separately identified')
    return {**result,'phase':'contact_candidate'}
