"""Audit a complete stationary left seed and a zero-TX right endpoint refusal.

Administrative, same run and budget; no device I/O, replay or fault clearing.
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
from .pair_ledger import _json_object, _number, activated_execution_budget
from .pair_restart import _current_contract, _file, _need, _sha
from .pair_task_enrollment import _lock_roots, _observations
from .preparation_continuation import _counts, _same_counts, _json_sort, _table_sha
from .reboot_startup import check_processes

TABLE = 'pair_endpoint_continuations'
SCHEMA = 'piper_zero_tx_endpoint_continuation_v1'
FAULT = 'Claimed preparation/query failed or uncertain: First-target initialization incomplete, uncertain or inconsistent'
SIDES = ('left','right')


def _snapshot(db, run_id):
    from .joint_sources import _validate_bindings, _validated_limits
    from .joint_initialization import validate_joint_initialization_sample, JointInitializationError
    from .hold_transaction import joint_hold_frames
    tables={}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_','').isalnum(),'Unexpected table name')
        tables[name]=sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort)
    _need(TABLE not in tables,'Endpoint successor already consumed')
    parents=tables.get('pair_initialization_continuations',[])
    _need(len(parents)==1,'One prior initialization successor required')
    scope=parents[0];run=next((r for r in tables['pair_runs'] if r['run_id']==run_id),None)
    head=max(tables['pair_rounds'],key=lambda r:r['ordinal'])
    _need(run and scope['run_id']==scope['active_run_id']==head['run_id']==run_id and scope['owner']
          and scope['round_ordinal']==head['ordinal'],'Current scope differs')
    record=json.loads(scope['record_json']);parent=record['proposal'];contract=json.loads(scope['contract_json'])
    _need(parent.get('schema')=='piper_zero_tx_seed_schedule_continuation_v1'
          and parent['proposal_sha256']==scope['proposal_sha256']
          and _sha({k:v for k,v in parent.items() if k!='proposal_sha256'})==scope['proposal_sha256']
          and record.get('old_rows_preserved') is True and record.get('new_budget_allocated') is False
          and contract==record['new_contract']==parent['reviewed_contract'],'Original audited parent required')
    events=sorted([r for r in tables['pair_events'] if r['run_id']==run_id and r['owner']==scope['owner']],key=lambda r:r['step'])
    faults=sorted([r for r in tables['pair_faults'] if r['run_id']==run_id and r['owner']==scope['owner']],key=lambda r:r['at'])
    _need(len(events)==3 and len(faults)==2 and [f['reason'] for f in faults]==[FAULT,'execution_receipt_failed']
          and faults[0]['id']==scope['fault_id'],'Exact right failure and failed-receipt latch required')
    _need([e['step'] for e in events]==list(range(parent['budget']['steps']+1,parent['budget']['steps']+4))
          and run['steps']==events[-1]['step'],'Cumulative steps differ')
    for event in events:
        _need(event['status']=='complete' and hashlib.sha256(event['payload_json'].encode()).hexdigest()==event['payload_digest'],
              'Complete immutable events required')
        _validate_bindings(contract,json.loads(event['payload_json'])['bindings'])
    query,seed,failed=events
    p=json.loads(query['payload_json']);r=json.loads(query['receipt_json'])
    _need(query['success']==1 and p.get('kind')=='query' and p.get('request')=={'operation':'inspect_joint_limits'}
          and r.get('ok') is True and r.get('fault_latched') is False and r.get('errors')==r.get('guard_violations')==[]
          and r.get('event_id')==query['event_id'] and r.get('pair_owner')==scope['owner'],'Query differs')
    _validated_limits({**r,'run_id':run_id,'owner':scope['owner'],'bindings':p['bindings']},
                      run_id=run_id,owner=scope['owner'],bindings=p['bindings'],now=query['finished_at'])
    for key,value in (('hardware_commands_sent',12),('joint_limit_queries_sent',12),('joint_limit_queries_attempted',12),
                      ('actuator_commands_sent',0),('target_commands_sent',0),('mode_commands_sent',0),('enable_commands_sent',0),('stop_commands_sent',0)):
        _need(type(r.get(key)) is int and r[key]==value,'Query counts differ: '+key)
    _need(_same_counts(r.get('transmission_counts'),_counts(6,6))
          and _same_counts(r.get('session_transmission_counts'),_counts(6,6)),'Query accounting differs')
    for event,side,success in ((seed,'left',1),(failed,'right',0)):
        p=json.loads(event['payload_json']);r=json.loads(event['receipt_json']);d=r if success else r.get('device_receipt',{})
        plan=d.get('initialization_plan',{});identity=plan.get('identity',{})
        _need(event['success']==success and p.get('kind')=='initialization' and p.get('existing_cached_target') is None
              and p['request'].get('operation')=='initialize_joint_target' and p['request'].get('arm')==side
              and p['request'].get('admission_mode')=='rgb_supervised' and plan.get('cached_target_prior')=='unknown'
              and identity.get('run_id')==run_id and identity.get('owner')==scope['owner']
              and identity.get('worker_id')==event['event_id'] and identity.get('arm')==side
              and plan.get('target_raw')==p['expected_target_raw'],'Initialization identity differs')
        for key in ('enable_commands_sent','stop_commands_sent','gripper_commands_sent','passive_arm_commands_sent','retries'):
            _need(type(d.get(key)) is int and d[key]==0,'Extra commands: '+key)
        _need(_same_counts(d.get('session_transmission_counts'),_counts(10,6)),'Session counts differ')
        if success:
            _need(d.get('ok') is True and d.get('fault_latched') is False and d.get('cache_established') is True
                  and d.get('errors')==d.get('guard_violations')==[] and plan.get('purpose')=='seed_current'
                  and d.get('controller_at_target') is True and d.get('observed_stable') is True
                  and d.get('feedback_all_after_send') is True and d.get('observed_stable_duration_s',0)>=3.
                  and d.get('observed_feedback_advances',0)>=20,'Complete stable left seed required')
            # This legacy report counts move_j via target_calls_sent; its
            # target_commands_sent field stays zero. Four wire returns below
            # and hardware_commands_sent are the actual send evidence.
            for key,value in (('hardware_commands_sent',4),('target_commands_sent',0),('target_calls_sent',1)):
                _need(type(d.get(key)) is int and d[key]==value,'Seed accounting differs: '+key)
            _need(_same_counts(d.get('transmission_counts'),_counts(4,0)),'Left seed send counts differ')
            frames=d['frame_receipts'];expected=joint_hold_frames(plan['target_raw'])
            _need(len(frames)==4 and all(f.get('frame')==x and f.get('outcome')=='returned' for f,x in zip(frames,expected))
                  and [f['returned_at'] for f in frames]==sorted(f['returned_at'] for f in frames),'Complete exact returned seed frames required')
            source=d['initialization_source'];sample=source['completion_sample']
            _need(source['frame_receipts']==frames and source['target_raw']==plan['target_raw']
                  and source['plan_sha256']==plan['plan_sha256'] and source['identity']==identity
                  and sample['captured_at']-frames[-1]['returned_at']>=3.
                  and d['before']['left']['joints_rad']==d['after']['left']['joints_rad']
                  and d['before']['left']['pose_m_rad']==d['after']['left']['pose_m_rad']
                  and d['before']['left']['gripper']['width_m']==d['after']['left']['gripper']['width_m'],
                  'Stationary completed seed source differs')
            validate_joint_initialization_sample(plan,sample,now=sample['captured_at'],phase='active',mode_confirmed=True)
        else:
            _need(r.get('ok') is False and r.get('automatic_retry') is False and r.get('event_id')==event['event_id']
                  and r.get('error')=='First-target initialization incomplete, uncertain or inconsistent'
                  and d.get('ok') is False and d.get('cache_established') is False and d.get('fault_latched') is True
                  and d.get('errors')==[{'type':'JointPathError','detail':'model_endpoint_displacement','code':'model_endpoint_displacement'}]
                  and d.get('frame_receipts')==d.get('guard_violations')==[] and plan.get('purpose')=='startup_j2_j3',
                  'Only the exact pre-dispatch endpoint refusal is covered')
            for key in ('hardware_commands_sent','target_commands_sent','target_calls_sent'):
                _need(type(d.get(key)) is int and d[key]==0,'Unknown/partial right send: '+key)
            _need(_same_counts(d.get('transmission_counts'),_counts(0,0)),'Zero right target attempts required')
            failure=d.get('tracking_observation',{}).get('first_failure',{});sample=failure.get('sample')
            _need(failure.get('code')=='model_endpoint_displacement' and isinstance(sample,dict),'Original failed sample required')
            observed=sample['arms']['right']['joints_rad'];original=plan['origin']['arms']['right']['joints_rad']
            _need(all(observed[i]==original[i] for i in (0,1,2,4,5))
                  and 0<abs(observed[3]-original[3])<=math.radians(.5)
                  and plan['model_endpoint_displacement_m']<=.015,'Only accepted J4 change may explain endpoint refusal')
            try:
                validate_joint_initialization_sample(plan,sample,now=sample['captured_at'],phase='pre_dispatch')
            except JointInitializationError as exc:
                _need(exc.code=='model_endpoint_displacement','Another historical sample violation')
            else:
                raise ValueError('Original endpoint failure not reproduced')
    _need(record['activated_at']<query['began_at']<query['finished_at']<seed['began_at']<seed['finished_at']
          <failed['began_at']<=sample['captured_at']<=faults[0]['at']<=faults[1]['at']==failed['finished_at'],
          'Historical chronology differs')
    _need(not any(e['status']=='pending' for e in tables['pair_events']),'Pending event blocks continuation')
    for name in ('pair_joint_sends','pair_hold_requests','pair_holds','pair_hold_frames','pair_grasp_episodes'):
        _need(not any(r['run_id']==run_id for r in tables.get(name,[])),'Other motion/hold/grasp history blocks continuation')
    prior=copy.deepcopy(tables)
    prior['pair_events']=[v for v in prior['pair_events'] if v not in events]
    prior['pair_faults']=[v for v in prior['pair_faults'] if v not in faults]
    next(v for v in prior['pair_runs'] if v['run_id']==run_id)['steps']-=3
    for name,digest in parent['snapshot']['table_sha256'].items():
        _need(_table_sha(prior.get(name,[]))==digest,'Historical rows changed: '+name)
    return dict(run=run,scope={k:v for k,v in scope.items() if k!='record_json'},contract=contract,events=events,faults=faults,
        parent_record_sha256=hashlib.sha256(scope['record_json'].encode()).hexdigest(),
        retired_owners=sorted({r['owner'] for rows in tables.values() for r in rows if r.get('owner')}),
        table_sha256={k:_table_sha(v) for k,v in tables.items()})


def _history(session_log,journal_path,snapshot):
    raw,ref=_file(session_log);rows=[json.loads(line) for line in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows),'Complete ordered session required')
    requests=[r for r in rows if r['kind']=='request']
    allowed={'capture','robot_pair_open','robot_pair_inspect_joint_limits','robot_pair_status',
             'robot_pair_observe','robot_pair_initialize_joint_target','robot_pair_close'}
    _need(all(r['request']['op'] in allowed for r in requests) and not any(r['kind']=='request_error' for r in rows),'Unexpected requests/errors')
    def result(req):
        found=[r['result'] for r in rows if r['kind']=='result' and r.get('request_id')==req['request']['id']]
        _need(len(found)==1,'Exactly one result required');return found[0]
    def one(op):
        found=[r for r in requests if r['request']['op']==op];_need(len(found)==1,'Exactly one '+op+' required');return found[0]
    opening=one('robot_pair_open');opened=result(opening)
    _need(opened.get('owner')==snapshot['scope']['owner'] and opened.get('run_id')==snapshot['run']['run_id']
          and opened.get('connection_mode')=='prepare','Original owner/open required')
    query,seed,failed=snapshot['events']
    _need(one('robot_pair_inspect_joint_limits')['request']['arguments']=={'event_id':query['event_id']},'Query event differs')
    inits=[r for r in requests if r['request']['op']=='robot_pair_initialize_joint_target'];matched=[]
    for request in inits:
        args=request['request']['arguments'];response=result(request)
        if response.get('status')=='refresh_required':
            _need(args.get('event_id')==seed['event_id'] and response.get('event_claimed') is False
                  and response.get('hardware_commands_sent')==response.get('steps_consumed')==0
                  and response.get('fault_latched') is False,'Only zero-claim left RGB refresh permitted')
            continue
        event=next((e for e in (seed,failed) if e['event_id']==args.get('event_id')),None)
        _need(event is not None,'Unknown initialization request')
        p=json.loads(event['payload_json']);_need(args=={'event_id':event['event_id'],**{k:v for k,v in p['request'].items() if k!='operation'}},'Claimed initialization request differs')
        matched.append(event['event_id'])
        observed=[r['result'] for r in rows if r['kind']=='result' and r.get('result',{}).get('event_id')==event['event_id']
                  and r['result'].get('status') in ('completed','fault')]
        _need(len(observed)==1 and observed[0].get('receipt')==json.loads(event['receipt_json']),'Original completion receipt required')
    _need(matched==[seed['event_id'],failed['event_id']],'Exactly one claim per arm required')
    close_request=one('robot_pair_close');closed=result(close_request);c=closed.get('cleanup',{});ended=rows[-1]
    _need(close_request['at']>failed['finished_at'] and closed.get('status')=='closed' and closed.get('fault_latched') is True
          and c.get('requires_fault_latch') is False and c.get('guard_violations')==[]
          and c.get('unresolved_gripper_probe') is None and c.get('grasp_states')==dict.fromkeys(SIDES)
          and _same_counts(c.get('session_transmission_counts'),_counts(10,6))
          and all(c.get('arms',{}).get(s,{}).get('status')=='disconnected' for s in SIDES)
          and ended['kind']=='session_ended' and ended.get('cleanup_errors')==[] and ended['at']>close_request['at'],
          'Known normal cleanup and completed exit required')
    path=Path(journal_path).absolute();_need(path==path.resolve() and path.is_file(),'Original nonsymlink journal required')
    before=path.stat();_need(before.st_size<=512*1024*1024,'Journal exceeds bounded streaming audit')
    digest=hashlib.sha256();previous=0.;metadata=[];stable=[]
    receipt=json.loads(seed['receipt_json']);frames=receipt['frame_receipts'];failure=json.loads(failed['receipt_json'])['device_receipt']['tracking_observation']['first_failure']['sample']
    with path.open('rb') as stream:
        for line in stream:
            _need(len(line)<=1024*1024,'Oversized journal row');digest.update(line)
            row=json.loads(line);at=_number(row.get('unix_s'),'journal time');_need(at>=previous,'Regressing journal');previous=at
            if at<opening['at']:continue
            _need(at<=ended['at'],'Unexpected activity after close')
            event=row.get('event')
            if event=='feedback':
                if frames[-1]['returned_at']<at<seed['finished_at']:
                    stable.append(row)
            elif event!='pair_fault_feedback':
                _need(event in {'connected_passively','pair_preparation_claimed','pair_shared_scene','pair_joint_limit_query_intent',
                    'pair_joint_limit_reply','pair_joint_initialization_intent','pair_joint_initialization_dispatched_unconfirmed',
                    'pair_joint_initialization_observed','initialization_feedback_rejected'},'Unexpected target/journal event')
                metadata.append(row)
    _need(path.stat()==before,'Journal changed')
    claims=[r for r in metadata if r['event']=='pair_preparation_claimed']
    _need(len(claims)==3 and all(c.get('event_id')==e['event_id'] and c.get('payload')==json.loads(e['payload_json']) for c,e in zip(claims,snapshot['events'])),'Durable claims differ')
    def one_event(name):
        found=[r for r in metadata if r['event']==name];_need(len(found)==1,'Exactly one '+name+' required');return found[0]
    connected=one_event('connected_passively');_need(_same_counts(connected.get('transmission_counts'),_counts(0,0)),'Connection was not zero TX')
    intent=one_event('pair_joint_initialization_intent');sent=one_event('pair_joint_initialization_dispatched_unconfirmed');observed=one_event('pair_joint_initialization_observed')
    _need(intent.get('event_id')==sent.get('event_id')==observed.get('event_id')==seed['event_id']
          and intent.get('plan')==receipt['initialization_plan'] and sent.get('frame_receipts')==frames
          and seed['began_at']<intent['unix_s']<=frames[0]['returned_at']<=frames[-1]['returned_at']<=sent['unix_s']<observed['unix_s']<seed['finished_at']
          and observed['sample']['captured_at']-frames[-1]['returned_at']>=3.
          and observed.get('duration_s',0)>=3. and observed.get('feedback_advances',0)>=20,'Exact complete left seed journal required')
    _need(len(stable)>=21 and stable[-1]['unix_s']-stable[0]['unix_s']>=2.9
          and all(r['left']['joints_rad']==receipt['after']['left']['joints_rad']
                  and r['left']['pose_m_rad']==receipt['after']['left']['pose_m_rad'] for r in stable),
          'Recorded left seed must remain stationary during arrival window')
    rejected=one_event('initialization_feedback_rejected')
    rejected_sample=rejected.get('sample',{})
    _need(rejected_sample.get('arms')==failure['arms'] and rejected_sample.get('identity')==failure['identity']
          and 0<=failure['captured_at']-rejected_sample.get('captured_at',0)<=.05
          and rejected_sample['captured_at']<=failure['captured_at']<=rejected['unix_s']
          and rejected.get('error')=='model_endpoint_displacement'
          and failed['began_at']<rejected['unix_s']<snapshot['faults'][0]['at'],'Original right rejected sample required')
    intents=[r for r in metadata if r['event']=='pair_joint_limit_query_intent'];replies=[r for r in metadata if r['event']=='pair_joint_limit_reply'];q=json.loads(query['receipt_json'])
    _need(len(intents)==len(replies)==12,'Complete twelve-query journal required')
    for intent,reply,(side,joint) in zip(intents,replies,((s,j) for s in SIDES for j in range(1,7))):
        sent=q['query_receipts'][side][str(joint)]
        _need(intent['side']==reply['side']==side and intent['joint_index']==reply['joint_index']==joint
              and intent['arbitration_id']==0x472 and intent['data_hex']==sent['data_hex']==bytes((joint,1,0,0,0,0,0,0)).hex()
              and sent['outcome']=='returned' and reply['result']['status']=='confirmed'
              and query['began_at']<=intent['unix_s']<=sent['sent_at']<=sent['returned_at']<=reply['unix_s']<=query['finished_at'],'Query wire evidence differs')
    return dict(session=ref,journal={'path':str(path),'sha256':digest.hexdigest()},ended_at=ended['at'],close_receipt=closed,
        left_seed_frames=4,right_target_frames=0,original_failure_sample_sha256=_sha(failure))
def prepare(path,run_id,*,session_log,journal_path,passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);root=path.parent.parent;check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');snapshot=_snapshot(db,run_id)
    history=_history(session_log,journal_path,snapshot);now=_number(clock(),'clock');run=snapshot['run'];deadline=run['started_at']+run['max_duration']
    _need(activated_execution_budget(path,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),'Original budget authorization changed')
    _need(history['ended_at']<=now<deadline and run['steps']<run['max_steps'],'Original remaining budget exhausted')
    contract=_current_contract(root,snapshot['contract'])
    changes={k for k,v in contract['code'].items() if snapshot['contract']['code'].get(k)!=v}
    _need(changes<={'joint_initialization.py','joint_ingress.py','pair_initialization.py','endpoint_continuation.py','pair_host.py','pair_ledger.py','reboot_startup.py'}
          and {'joint_initialization.py','joint_ingress.py','pair_initialization.py','endpoint_continuation.py'}<=changes,'Only reviewed bounded endpoint target selection and scope changes are covered')
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
                           'previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,initialization_ordinal INTEGER NOT NULL,'
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
