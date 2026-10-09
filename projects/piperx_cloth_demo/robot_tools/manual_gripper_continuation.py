"""Explicit operator continuation after a fully returned, unconfirmed jaw probe.

The old failure remains failed. Requires a new reposition/retry instruction,
normal closure, original trace, and fresh supported-scene evidence with the
jaw now near the old target. It grants preparation only, no replay or grasp.
"""
from contextlib import ExitStack
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

TABLE = 'pair_manual_gripper_continuations'
SCHEMA = 'piper_manual_repositioned_gripper_continuation_v1'
REPAIR_FILES = {'pair_host.py', 'pair_ledger.py', 'manual_gripper_continuation.py'}
REASON = 'No distinguishable closing-and-settling response or no unambiguous width outcome'
SIDES = ('left', 'right')


def tables(db):
    result = {}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_','').isalnum(), 'Unexpected table name')
        result[name] = [dict(r) for r in db.execute('SELECT * FROM '+name)]
    return result


def validate_failed_probe(event, run, owner):
    p, r = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    d = r.get('device_receipt',{}); c = d.get('contact_observation',{})
    _need(event['run_id'] == run['run_id'] and event['owner'] == owner
          and event['step'] == run['steps'] and event['status'] == 'complete' and event['success'] == 0
          and hashlib.sha256(event['payload_json'].encode()).hexdigest() == event['payload_digest']
          and set(p) == {'arm','kind','operation','target','grasp_object_id','observation_id','peer_receipt_id'}
          and p['kind'] == 'gripper' and p['operation'] == 'grip_supported' and p['arm'] in SIDES
          and r.get('event_id') == event['event_id'] and r.get('ok') is False
          and r.get('automatic_retry') is False, 'Exact final failed supported probe required')
    _identifier(p['grasp_object_id'],'original object')
    arm = p['arm']
    counts = {s:dict(attempted_frames=int(s==arm),sent_frames=int(s==arm),blocked_frames=0) for s in SIDES}
    _need(d.get('transmission_counts') == counts and d.get('hardware_commands_sent') == 1
          and d.get('target_calls_sent') == 1 and d.get('nominal_force_N') == .2
          and all(d.get(k) == 0 for k in ('enable_commands_sent','stop_commands_sent','retries','passive_arm_commands_sent'))
          and d.get('guard_violations') == [] and d.get('ok') is False
          and d.get('errors') == [{'type':'RuntimeError','detail':'Probe response unconfirmed: '+str([REASON])}]
          and c.get('outcome') == 'unconfirmed' and c.get('reasons') == [REASON]
          and c.get('observation_window_complete') is True and c.get('completion') == 'observation_only'
          and c.get('requested_width_m') == p['target'] and c.get('target_may_remain_active') is True
          and d.get('unresolved_gripper_probe') is None and d.get('grasp_states') == dict.fromkeys(SIDES),
          'Partial/unknown send, different fault or retained grasp cannot use this entry')
    return p,d


def snapshot(db, run_id):
    all_rows = tables(db)
    _need(TABLE not in all_rows, 'This one manual continuation has already been consumed')
    scopes = all_rows.get('pair_round_rgb_continuations',[])
    _need(len(scopes)==1, 'One original RGB continuation required')
    scope=scopes[0]; run=next((r for r in all_rows['pair_runs'] if r['run_id']==run_id),None)
    _need(run and scope['run_id']==scope['active_run_id']==run_id and scope['owner'] and scope['fault_id']
          and max(all_rows['pair_rounds'],key=lambda r:r['ordinal'])['ordinal']==scope['round_ordinal'],
          'Current faulted original round/owner required')
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
    episodes=[r for r in all_rows.get('pair_grasp_episodes',[]) if r['run_id']==run_id]
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
    return dict(session=ref,probe_journal=jref,ended_at=rows[-1]['at'],physical_stop_verified=None)


def authorization(value,s,h,now):
    _need(type(value) is dict and set(value)=={'source','message_id','statement','recorded_at','decision'}
          and value['source']=='user_message' and value['decision']=='authorize_repositioned_gripper_attempt'
          and type(value['statement']) is str and 1<=len(value['statement'].strip())<=4000,
          'A new explicit user reposition/retry instruction is required')
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
        width=result['passive'][side]['jaw_width_m'];reference=p['target'] if side==p['arm'] else origin['gripper']['width_m']
        _need(abs(width-reference)<=.0005,'Current jaw must resolve near the old returned target; peer unchanged')
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
    _need(changes<=REPAIR_FILES and set(s['contract']['code'])<=set(new['code']),'Only the reviewed administrative continuation code may change')
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
