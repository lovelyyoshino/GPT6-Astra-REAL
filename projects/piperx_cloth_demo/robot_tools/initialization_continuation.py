"""Same-budget administrative successor for a zero-TX seed freshness failure.

This entry audits a complete query then one failed seed-current initialization.
No robot I/O, target replay, new budget, historical rewrite or fault clearing.
"""
from contextlib import ExitStack
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import _json_object, _number, activated_execution_budget
from .pair_restart import _current_contract, _file, _need, _sha
from .pair_task_enrollment import _lock_roots, _observations
from .preparation_continuation import _counts, _same_counts, _json_sort, _table_sha
from .reboot_startup import check_processes

TABLE = 'pair_initialization_continuations'
SCHEMA = 'piper_zero_tx_seed_schedule_continuation_v1'
ERROR = 'Initialization feedback exceeds 50 ms including processing'
FAULT = 'Claimed preparation/query failed or uncertain: First-target initialization incomplete, uncertain or inconsistent'
SIDES = ('left','right')


def _snapshot(db,run_id):
    from .joint_sources import _validate_bindings, _validated_limits
    from .joint_initialization import validate_joint_initialization_sample
    tables={}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_','').isalnum(),'Unexpected table name')
        tables[name]=sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort)
    _need(TABLE not in tables,'Initialization successor already consumed')
    parents=tables.get('pair_feedback_continuations',[])
    _need(len(parents)==1,'One current feedback successor required')
    scope=parents[0];run=next((r for r in tables['pair_runs'] if r['run_id']==run_id),None)
    head=max(tables['pair_rounds'],key=lambda r:r['ordinal'])
    _need(run and scope['run_id']==scope['active_run_id']==head['run_id']==run_id and scope['owner']
          and scope['round_ordinal']==head['ordinal'],'Current task scope differs')
    record=json.loads(scope['record_json']);parent=record['proposal'];contract=json.loads(scope['contract_json'])
    _need(parent.get('schema')=='piper_query_only_mixed_feedback_continuation_v1'
          and parent['proposal_sha256']==scope['proposal_sha256']
          and _sha({k:v for k,v in parent.items() if k!='proposal_sha256'})==scope['proposal_sha256']
          and contract==record['new_contract']==parent['reviewed_contract']
          and record.get('old_rows_preserved') is True and record.get('new_budget_allocated') is False,
          'Original audited feedback successor required')
    events=sorted([r for r in tables['pair_events'] if r['run_id']==run_id and r['owner']==scope['owner']],key=lambda r:r['step'])
    faults=sorted([r for r in tables['pair_faults'] if r['run_id']==run_id and r['owner']==scope['owner']],key=lambda r:r['at'])
    _need(len(events)==len(faults)==2 and [r['reason'] for r in faults]==[FAULT,'execution_receipt_failed']
          and faults[0]['id']==scope['fault_id'],'Exact seed failure and its failed-receipt latch required')
    _need([e['step'] for e in events]==[parent['budget']['steps']+1,parent['budget']['steps']+2]
          and run['steps']==events[-1]['step'],'Original cumulative step accounting differs')
    query,failed=events
    for e in events:
        _need(e['status']=='complete' and hashlib.sha256(e['payload_json'].encode()).hexdigest()==e['payload_digest'],
              'Complete immutable event required')
    p=json.loads(query['payload_json']);q=json.loads(query['receipt_json'])
    _need(query['success']==1 and p.get('kind')=='query' and p.get('request')=={'operation':'inspect_joint_limits'}
          and q.get('ok') is True and q.get('fault_latched') is False and q.get('errors')==q.get('guard_violations')==[]
          and q.get('event_id')==query['event_id'] and q.get('pair_owner')==scope['owner'],'Query receipt differs')
    _validate_bindings(contract,p['bindings'])
    _validated_limits({**q,'run_id':run_id,'owner':scope['owner'],'bindings':p['bindings']},
                      run_id=run_id,owner=scope['owner'],bindings=p['bindings'],now=query['finished_at'])
    for key,value in (('hardware_commands_sent',12),('joint_limit_queries_sent',12),('joint_limit_queries_attempted',12),
                      ('actuator_commands_sent',0),('target_commands_sent',0),('mode_commands_sent',0),('enable_commands_sent',0),('stop_commands_sent',0)):
        _need(type(q.get(key)) is int and q[key]==value,'Query transmission differs: '+key)
    _need(_same_counts(q.get('transmission_counts'),_counts(6,6))
          and _same_counts(q.get('session_transmission_counts'),_counts(6,6)),'Query counts differ')
    p=json.loads(failed['payload_json']);r=json.loads(failed['receipt_json']);d=r.get('device_receipt',{});plan=d.get('initialization_plan',{})
    _need(failed['success']==0 and r.get('ok') is False and r.get('event_id')==failed['event_id']
          and r.get('error')=='First-target initialization incomplete, uncertain or inconsistent'
          and r.get('automatic_retry') is False and p.get('kind')=='initialization'
          and p.get('existing_cached_target') is None and p['request'].get('operation')=='initialize_joint_target'
          and p['request'].get('admission_mode')=='rgb_supervised' and p['request'].get('arm')=='left'
          and d.get('ok') is False and d.get('status')=='initialization_failed' and d.get('cache_established') is False
          and d.get('errors')==[{'type':'RuntimeError','detail':ERROR,'code':None}]
          and d.get('frame_receipts')==d.get('guard_violations')==[] and d.get('fault_latched') is True,
          'Only this diagnosed pre-dispatch seed freshness failure is covered')
    for key in ('hardware_commands_sent','target_commands_sent','target_calls_sent','enable_commands_sent',
                'stop_commands_sent','gripper_commands_sent','passive_arm_commands_sent','retries'):
        _need(type(d.get(key)) is int and d[key]==0,'Unknown/partial initialization send: '+key)
    _need(_same_counts(d.get('transmission_counts'),_counts(0,0))
          and _same_counts(d.get('session_transmission_counts'),_counts(6,6)),'No actuator attempts permitted')
    _validate_bindings(contract,p['bindings'])
    identity=plan.get('identity',{})
    _need(plan.get('purpose')=='seed_current' and plan.get('cached_target_prior')=='unknown'
          and identity.get('run_id')==run_id and identity.get('owner')==scope['owner']
          and identity.get('worker_id')==failed['event_id'] and identity.get('arm')=='left'
          and plan.get('target_raw')==p.get('expected_target_raw'),'Seed plan identity/target differs')
    failure=d.get('tracking_observation',{}).get('first_failure',{})
    sample=failure.get('sample')
    _need(failure.get('detail')==ERROR and isinstance(sample,dict),'Original failure sample required')
    # Fresh and numerically valid at capture, but rejected after processing.
    validate_joint_initialization_sample(plan,sample,now=sample['captured_at'],phase='pre_dispatch')
    _need(record['activated_at']<query['began_at']<query['finished_at']<failed['began_at']
          <=sample['captured_at']<=faults[0]['at']<=faults[1]['at']==failed['finished_at'],
          'Query, sample and failure chronology differs')
    _need(not any(e['status']=='pending' for e in tables['pair_events']),'Pending event blocks continuation')
    for name in ('pair_joint_sends','pair_hold_requests','pair_holds','pair_hold_frames','pair_grasp_episodes'):
        _need(not any(r['run_id']==run_id for r in tables.get(name,[])),'Target/hold/grasp history is outside this scope')
    prior=copy.deepcopy(tables)
    prior['pair_events']=[v for v in prior['pair_events'] if v not in events]
    prior['pair_faults']=[v for v in prior['pair_faults'] if v not in faults]
    next(v for v in prior['pair_runs'] if v['run_id']==run_id)['steps']-=2
    for name,digest in parent['snapshot']['table_sha256'].items():
        _need(_table_sha(prior.get(name,[]))==digest,'Historical rows changed: '+name)
    return dict(run=run,scope={k:v for k,v in scope.items() if k!='record_json'},
                parent_record_sha256=hashlib.sha256(scope['record_json'].encode()).hexdigest(),
                faults=faults,events=events,contract=contract,
                retired_owners=sorted({r['owner'] for rows in tables.values() for r in rows if r.get('owner')}),
                table_sha256={k:_table_sha(v) for k,v in tables.items()})


def _history(session_log,journal_path,snapshot):
    raw,ref=_file(session_log);rows=[json.loads(line) for line in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows),'Complete ordered session required')
    requests=[r for r in rows if r['kind']=='request']
    allowed={'capture','robot_pair_open','robot_pair_inspect_joint_limits','robot_pair_status',
             'robot_pair_observe','robot_pair_initialize_joint_target','robot_pair_close'}
    _need(all(r['request']['op'] in allowed for r in requests) and not any(r['kind']=='request_error' for r in rows),
          'Unexpected request or additional failure')
    def one(op):
        found=[r for r in requests if r['request']['op']==op];_need(len(found)==1,'Exactly one '+op+' required');return found[0]
    def result(req):
        found=[r['result'] for r in rows if r['kind']=='result' and r.get('request_id')==req['request']['id']]
        _need(len(found)==1,'Exactly one result required');return found[0]
    opening=one('robot_pair_open');opened=result(opening)
    _need(opened.get('run_id')==snapshot['run']['run_id'] and opened.get('owner')==snapshot['scope']['owner']
          and opened.get('connection_mode')=='prepare','Owner/connection mode mismatch')
    query,failed=snapshot['events']
    _need(one('robot_pair_inspect_joint_limits')['request']['arguments']=={'event_id':query['event_id']},'Query event differs')
    init=one('robot_pair_initialize_joint_target');args=init['request']['arguments'];payload=json.loads(failed['payload_json'])
    _need(args=={'event_id':failed['event_id'],**{k:v for k,v in payload['request'].items() if k!='operation'}},'Original initialization request differs')
    observed=[r['result'] for r in rows if r['kind']=='result' and r.get('result',{}).get('event_id')==failed['event_id'] and r['result'].get('status')=='fault']
    _need(len(observed)==1 and observed[0].get('receipt')==json.loads(failed['receipt_json']),'Original failed receipt required')
    close_request=one('robot_pair_close');closed=result(close_request);c=closed.get('cleanup',{})
    _need(close_request['at']>failed['finished_at'] and closed.get('status')=='closed' and closed.get('fault_latched') is True
          and c.get('requires_fault_latch') is False and c.get('guard_violations')==[]
          and c.get('unresolved_gripper_probe') is None and c.get('grasp_states')==dict.fromkeys(SIDES)
          and _same_counts(c.get('session_transmission_counts'),_counts(6,6))
          and all(c.get('arms',{}).get(s,{}).get('status')=='disconnected' for s in SIDES),'Known query-only normal cleanup required')
    ended=rows[-1];_need(ended['kind']=='session_ended' and ended.get('cleanup_errors')==[]
                         and ended['at']>close_request['at'],'Completed session exit required')
    path=Path(journal_path).absolute();_need(path==path.resolve() and path.is_file(),'Original nonsymlink journal required')
    before=path.stat();_need(before.st_size<=256*1024*1024,'Journal exceeds bounded streaming audit')
    digest=hashlib.sha256();previous=0.;metadata=[];failure_sample_found=False
    failure=json.loads(failed['receipt_json'])['device_receipt']['tracking_observation']['first_failure']['sample']
    with path.open('rb') as stream:
        for line in stream:
            _need(len(line)<=1024*1024,'Oversized journal row');digest.update(line)
            row=json.loads(line);at=_number(row.get('unix_s'),'journal timestamp');_need(at>=previous,'Regressing journal');previous=at
            if at<opening['at']:continue
            _need(at<=ended['at'],'Unexpected activity after session close')
            event=row.get('event')
            if event=='feedback':
                if all(row.get(s)==failure['arms'][s] for s in SIDES):
                    _need(failed['began_at']<=at<=snapshot['faults'][0]['at'],'Failure sample outside claimed event')
                    failure_sample_found=True
            elif event!='pair_fault_feedback':
                _need(event in {'connected_passively','pair_preparation_claimed','pair_shared_scene',
                               'pair_joint_limit_query_intent','pair_joint_limit_reply'},'Unexpected initialization intent, target or other event')
                metadata.append(row)
    _need(path.stat()==before and failure_sample_found,'Journal changed or original failure sample missing')
    claims=[r for r in metadata if r['event']=='pair_preparation_claimed']
    _need(len(claims)==2 and all(c.get('event_id')==e['event_id'] and c.get('payload')==json.loads(e['payload_json'])
                               for c,e in zip(claims,snapshot['events'])),'Durable claims differ')
    _need(len([r for r in metadata if r['event']=='connected_passively'])==1,'One passive connection required')
    intents=[r for r in metadata if r['event']=='pair_joint_limit_query_intent'];replies=[r for r in metadata if r['event']=='pair_joint_limit_reply']
    _need(len(intents)==len(replies)==12,'Complete twelve-query journal required')
    receipt=json.loads(query['receipt_json'])
    for intent,reply,(side,joint) in zip(intents,replies,((s,j) for s in SIDES for j in range(1,7))):
        sent=receipt['query_receipts'][side][str(joint)]
        _need(intent['side']==reply['side']==side and intent['joint_index']==reply['joint_index']==joint
              and intent['arbitration_id']==0x472 and intent['data_hex']==sent['data_hex']==bytes((joint,1,0,0,0,0,0,0)).hex()
              and sent['outcome']=='returned' and reply['result']['status']=='confirmed'
              and query['began_at']<=intent['unix_s']<=sent['sent_at']<=sent['returned_at']<=reply['unix_s']<=query['finished_at'],
              'Query wire evidence differs')
    return dict(session=ref,journal={'path':str(path),'sha256':digest.hexdigest()},ended_at=ended['at'],close_receipt=closed,
                original_failure_sample_sha256=_sha(failure),initialization_intent_seen=False)


def prepare(path,run_id,*,session_log,journal_path,passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);root=path.parent.parent;check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');snapshot=_snapshot(db,run_id)
    history=_history(session_log,journal_path,snapshot);now=_number(clock(),'clock');run=snapshot['run'];deadline=run['started_at']+run['max_duration']
    _need(activated_execution_budget(path,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),'Original budget authorization changed')
    _need(history['ended_at']<=now<deadline and run['steps']<run['max_steps'],'Original remaining budget exhausted')
    contract=_current_contract(root,snapshot['contract'])
    changes={k for k,v in contract['code'].items() if snapshot['contract']['code'].get(k)!=v}
    _need(changes<={'pair_initialization.py','initialization_continuation.py','pair_host.py','pair_ledger.py','reboot_startup.py'}
          and {'pair_initialization.py','initialization_continuation.py'}<=changes,'Only reviewed initialization scheduling and scope code changes are covered')
    p=dict(schema=SCHEMA,database=str(path),run_id=run_id,created_at=now,snapshot=snapshot,history=history,reviewed_contract=contract,
        evidence=_observations(passive_paths,rgb_observation,visual_observation,contract,history['ended_at'],now),
        budget=dict(started_at=run['started_at'],deadline_s=deadline,max_steps=run['max_steps'],steps=run['steps'],max_duration_s=run['max_duration']),
        hardware_commands_sent=0,new_budget_allocated=False,cache_or_limits_transferred=False,required_connection_mode='prepare',physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def activate(proposal,*,project_root,clock=time.time):
    _need(proposal.get('schema')==SCHEMA and _sha({k:v for k,v in proposal.items() if k!='proposal_sha256'})==proposal.get('proposal_sha256'),'Exact reviewed proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(proposal['database']);_need(path==root/'runs/pair_sessions.sqlite','Authoritative ledger required')
    ev=proposal['evidence'];history=proposal['history']
    kwargs=dict(session_log=history['session']['path'],journal_path=history['journal']['path'],passive_paths={s:ev['passive'][s]['source']['path'] for s in SIDES},
                rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare(path,proposal['run_id'],clock=lambda:proposal['created_at'],**kwargs)==proposal,'Evidence, ledger or code changed after review')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(_snapshot(db,proposal['run_id'])==proposal['snapshot'],'Ledger changed before activation')
                now=_number(clock(),'clock');_need(proposal['created_at']<=now<proposal['budget']['deadline_s'],'Original deadline reached')
                _need(_observations(kwargs['passive_paths'],kwargs['rgb_observation'],kwargs['visual_observation'],proposal['reviewed_contract'],history['ended_at'],now)==ev,'Current evidence expired or changed')
                _need(_current_contract(root,proposal['snapshot']['contract'])==proposal['reviewed_contract'],'Reviewed code changed')
                snapshot=proposal['snapshot'];record=dict(proposal=proposal,activated_at=now,new_contract=proposal['reviewed_contract'],hardware_commands_sent=0,
                    new_budget_allocated=False,old_rows_preserved=True,cache_or_limits_transferred=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,'
                           'previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,feedback_ordinal INTEGER NOT NULL,'
                           'contract_json TEXT NOT NULL,proposal_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL)')
                db.execute('INSERT INTO '+TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?,?)',
                    (proposal['run_id'],now,snapshot['scope']['owner'],json.dumps(snapshot['retired_owners']),snapshot['scope']['round_ordinal'],snapshot['scope']['ordinal'],
                     _json_object(record['new_contract'],'contract'),proposal['proposal_sha256'],_json_object(record,'record')))
                for name,digest in snapshot['table_sha256'].items():
                    rows=sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort);_need(_table_sha(rows)==digest,'Historical rows changed: '+name)
                check_processes();_need(now<=_number(clock(),'commit clock')<proposal['budget']['deadline_s'],'Deadline reached before commit')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise
