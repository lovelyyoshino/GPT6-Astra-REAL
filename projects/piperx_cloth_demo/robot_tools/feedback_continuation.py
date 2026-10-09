"""Audited same-budget successor for the query-only mixed-pose RX fault.

No device I/O, automatic retry, new budget, or rewrite of historical rows.
The former preparation successor remains latched. Its current successor must
use the repaired receive assembler and establish a fresh preparation connection.
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

TABLE = 'pair_feedback_continuations'
SCHEMA = 'piper_query_only_mixed_feedback_continuation_v1'
FAULT = 'Idle monitor: right pose SO(3) rotation drift exceeds task observation bound'
SIDES = ('left', 'right')


def _snapshot(db, run_id):
    from .joint_sources import _validate_bindings, _validated_limits
    tables = {}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_','').isalnum(), 'Invalid table name')
        tables[name] = sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort)
    _need(TABLE not in tables, 'Mixed-feedback continuation already consumed')
    parents = tables.get('pair_preparation_continuations', [])
    _need(len(parents)==1, 'Exact prior preparation successor required')
    scope=parents[0];run=next((r for r in tables['pair_runs'] if r['run_id']==run_id),None)
    current_round=max(tables['pair_rounds'],key=lambda r:r['ordinal'])
    _need(run and scope['run_id']==scope['active_run_id']==run_id and scope['owner']
          and scope['round_ordinal']==current_round['ordinal'] and current_round['run_id']==run_id,
          'Current preparation scope/round mismatch')
    record=json.loads(scope['record_json']);parent=record['proposal']
    _need(parent.get('schema')=='piper_preparation_close_continuation_v1'
          and parent.get('proposal_sha256')==scope['proposal_sha256']
          and _sha({k:v for k,v in parent.items() if k!='proposal_sha256'})==scope['proposal_sha256']
          and record.get('new_budget_allocated') is False and record.get('old_rows_preserved') is True,
          'Original reviewed preparation activation required')
    old=json.loads(scope['contract_json'])
    _need(old==record['new_contract']==parent['reviewed_contract']
          and old['task']['task_id']=='plug_transfer_left'
          and old['task']['site_context']['feedback_observation']['profile']=='right_j4_bounded_v1',
          'Frozen bounded plug task required')
    events=[r for r in tables['pair_events'] if r['run_id']==run_id and r['owner']==scope['owner']]
    faults=[r for r in tables['pair_faults'] if r['run_id']==run_id and r['owner']==scope['owner']]
    _need(len(events)==len(faults)==1 and faults[0]['id']==scope['fault_id'] and faults[0]['reason']==FAULT,
          'Only one complete query and the diagnosed idle RX fault are covered')
    e=events[0];p=json.loads(e['payload_json']);r=json.loads(e['receipt_json'])
    _need(e['status']=='complete' and e['success']==1 and e['step']==run['steps']==parent['budget']['steps']+1
          and p.get('kind')=='query' and p.get('request')=={'operation':'inspect_joint_limits'}
          and hashlib.sha256(e['payload_json'].encode()).hexdigest()==e['payload_digest']
          and r.get('ok') is True and r.get('fault_latched') is False
          and r.get('errors')==r.get('guard_violations')==[]
          and r.get('event_id')==e['event_id'] and r.get('pair_owner')==scope['owner']
          and record['activated_at']<e['began_at']<e['finished_at']<faults[0]['at'],
          'Query receipt, identity or chronology differs')
    _validate_bindings(old,p['bindings'])
    _validated_limits({**r,'run_id':run_id,'owner':scope['owner'],'bindings':p['bindings']},
                      run_id=run_id,owner=scope['owner'],bindings=p['bindings'],now=e['finished_at'])
    for key,value in (('hardware_commands_sent',12),('joint_limit_queries_sent',12),
                      ('joint_limit_queries_attempted',12),('actuator_commands_sent',0),
                      ('target_commands_sent',0),('mode_commands_sent',0),('enable_commands_sent',0),('stop_commands_sent',0)):
        _need(type(r.get(key)) is int and r[key]==value,'Unexpected/unknown query transmission: '+key)
    for key in ('transmission_counts','session_transmission_counts'):
        _need(_same_counts(r.get(key),_counts(6,6)),'Unexpected session transmission')
    _need(not any(e['status']=='pending' for e in tables['pair_events']),'Pending event prevents continuation')
    for name in ('pair_joint_sends','pair_hold_requests','pair_holds','pair_hold_frames','pair_grasp_episodes'):
        _need(not any(r['run_id']==run_id for r in tables.get(name,[])),'Target, grasp or hold outside query-only scope')
    # Reconstruct only the three known append/count changes since the prior
    # activation and verify every original table hash. Old history is immutable.
    prior=copy.deepcopy(tables)
    prior['pair_events']=[v for v in prior['pair_events'] if v!=e]
    prior['pair_faults']=[v for v in prior['pair_faults'] if v!=faults[0]]
    next(v for v in prior['pair_runs'] if v['run_id']==run_id)['steps']-=1
    for name,digest in parent['snapshot']['table_sha256'].items():
        _need(_table_sha(prior.get(name,[]))==digest,'History changed since prior activation: '+name)
    retired=sorted({r['owner'] for rows in tables.values() for r in rows if r.get('owner')})
    return dict(run=run,scope=scope,fault=faults[0],event=e,contract=old,retired_owners=retired,
                table_sha256={k:_table_sha(v) for k,v in tables.items()})


def _history(session_log,journal_path,snapshot):
    raw,ref=_file(session_log);rows=[json.loads(line) for line in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows]==list(range(1,len(rows)+1))
          and [r['at'] for r in rows]==sorted(r['at'] for r in rows),'Complete ordered session required')
    requests=[r for r in rows if r['kind']=='request']
    allowed={'capture','robot_pair_open','robot_pair_inspect_joint_limits','robot_pair_status','robot_pair_observe','robot_pair_close'}
    _need(all(r['request']['op'] in allowed for r in requests),'Unexpected action request')
    def one(op):
        found=[r for r in requests if r['request']['op']==op]
        _need(len(found)==1,'Exactly one '+op+' required')
        return found[0]
    def result(req):
        found=[r for r in rows if r['kind']=='result' and r.get('request_id')==req['request']['id']]
        _need(len(found)==1,'Exactly one request result required')
        return found[0]['result']
    opening=one('robot_pair_open');opened=result(opening)
    _need(opened.get('run_id')==snapshot['run']['run_id'] and opened.get('owner')==snapshot['scope']['owner']
          and opened.get('connection_mode')=='prepare','Session owner/mode mismatch')
    query=one('robot_pair_inspect_joint_limits')
    _need(query['request']['arguments']=={'event_id':snapshot['event']['event_id']},'Query identity differs')
    errors=[r for r in rows if r['kind']=='request_error']
    for error in errors:
        req=next((r for r in requests if r['request']['id']==error['request_id']),None)
        _need(req and req['request']['op']=='robot_pair_observe' and error['at']>snapshot['fault']['at']
              and error.get('error')=='PairHostError: Pair latched or closing; no further target frames',
              'Other session failure is outside this continuation')
    close_request=one('robot_pair_close');closed=result(close_request);c=closed.get('cleanup',{})
    _need(close_request['at']>snapshot['fault']['at'] and closed.get('status')=='closed'
          and closed.get('fault_latched') is True and c.get('requires_fault_latch') is False
          and c.get('guard_violations')==[] and c.get('unresolved_gripper_probe') is None
          and c.get('grasp_states')==dict.fromkeys(SIDES) and _same_counts(c.get('session_transmission_counts'),_counts(6,6))
          and all(c.get('arms',{}).get(s,{}).get('status')=='disconnected' for s in SIDES),
          'Normal empty-arm cleanup with known query-only counts required')
    ended=rows[-1]
    _need(ended['kind']=='session_ended' and ended.get('cleanup_errors')==[] and ended['at']>close_request['at'],
          'Session exit evidence required')
    path=Path(journal_path).absolute();_need(path==path.resolve() and path.is_file(),'Original nonsymlink journal required')
    before=path.stat();_need(before.st_size<=128*1024*1024,'Journal too large for bounded audit')
    digest=hashlib.sha256();previous=0.;metadata=[];anchor=mixed=after=None
    with path.open('rb') as stream:
        for line in stream:
            _need(len(line)<=1024*1024,'Oversized journal row');digest.update(line)
            row=json.loads(line);at=_number(row.get('unix_s'),'journal timestamp')
            _need(at>=previous,'Journal clock regressed');previous=at
            if at<opening['at']:continue
            _need(at<=ended['at'],'Unexpected journal activity after close')
            kind=row.get('event')
            if kind=='feedback':
                if anchor is None and all(row.get(s,{}).get('status')=='complete' for s in SIDES):anchor=row
                if at<snapshot['fault']['at']:mixed=row
            elif kind=='pair_fault_feedback':
                if after is None and at>=snapshot['fault']['at']:
                    feedback=row.get('feedback',{});after={'unix_s':at,**feedback.get('arms',{})}
            else:
                _need(kind in {'connected_passively','pair_preparation_claimed','pair_joint_limit_query_intent','pair_joint_limit_reply'},
                      'Unexpected target or preparation event in journal')
                metadata.append(row)
    _need(path.stat()==before,'Journal changed during audit')
    claims=[r for r in metadata if r['event']=='pair_preparation_claimed']
    _need(len(claims)==1 and claims[0].get('event_id')==snapshot['event']['event_id']
          and claims[0].get('payload')==json.loads(snapshot['event']['payload_json']),'Journal claim mismatch')
    _need(len([r for r in metadata if r['event']=='connected_passively'])==1,'Single passive connection required')
    intents=[r for r in metadata if r['event']=='pair_joint_limit_query_intent']
    replies=[r for r in metadata if r['event']=='pair_joint_limit_reply']
    _need(len(intents)==len(replies)==12,'Complete query journal required')
    receipt=json.loads(snapshot['event']['receipt_json'])
    for intent,reply,(side,joint) in zip(intents,replies,((s,j) for s in SIDES for j in range(1,7))):
        sent=receipt['query_receipts'][side][str(joint)]
        _need(intent['side']==reply['side']==side and intent['joint_index']==reply['joint_index']==joint
              and intent['arbitration_id']==0x472 and intent['data_hex']==bytes((joint,1,0,0,0,0,0,0)).hex()
              and intent['data_hex']==sent['data_hex'] and sent['outcome']=='returned'
              and snapshot['event']['began_at']<=intent['unix_s']<=sent['sent_at']<=sent['returned_at']<=reply['unix_s']<=snapshot['event']['finished_at']
              and reply['result']['status']=='confirmed','Query wire accounting differs')
    diagnosis=_diagnose(anchor,mixed,after,snapshot['fault']['at'])
    return dict(session=ref,journal={'path':str(path),'sha256':digest.hexdigest()},ended_at=ended['at'],
                close_receipt=closed,diagnosis=diagnosis)


def _diagnose(anchor,mixed,after,fault_at):
    from .arms import control_health
    from .contact_receipt import _rotation_span
    from .coherent_feedback import MAX_GROUP_SPAN_S
    _need(all(type(r) is dict for r in (anchor,mixed,after)),'Original pre/post-fault samples required')
    keys=('end_pose_xy','end_pose_zrx','end_pose_ryrz')
    for row in (anchor,mixed,after):
        _need(all(control_health(row[s],now_s=row[s]['timestamp'])['healthy'] for s in SIDES),
              'Historical feedback has another health error')
    _need(anchor['unix_s']<mixed['unix_s']<fault_at<=after['unix_s'] and fault_at-mixed['unix_s']<.1
          and after['unix_s']-fault_at<.2,'Exact adjacent fault samples required')
    stamps=[mixed['right']['fragment_timestamps_s'][k] for k in keys]
    _need(stamps[0]<=stamps[1] and stamps[2]<stamps[0] and max(stamps)-min(stamps)>MAX_GROUP_SPAN_S,
          'Failure is not the diagnosed old/new Cartesian mixture')
    for row in (anchor,after):
        times=[row['right']['fragment_timestamps_s'][k] for k in keys]
        _need(times==sorted(times) and max(times)-min(times)<=MAX_GROUP_SPAN_S,'Complete neighboring pose groups required')
    origin=anchor['right']['pose_m_rad']
    bad=_rotation_span([origin,mixed['right']['pose_m_rad']]);good=_rotation_span([origin,after['right']['pose_m_rad']])
    from .feedback_tolerance import rotation_tolerance
    limit=rotation_tolerance({'profile':'right_j4_bounded_v1','source':'user','statement':'diagnostic profile'},'right')
    _need(good<=limit<bad,'Historical diagnosis does not match the bounded RX fault')
    return dict(anchor=anchor,mixed=mixed,following=after,mixed_rotation_rad=bad,following_rotation_rad=good,
                physical_stop_verified=None,all_fluctuation_is_sensor_noise=False)


def prepare(path,run_id,*,session_log,journal_path,passive_paths,rgb_observation,visual_observation,clock=time.time):
    path=Path(path).resolve(strict=True);root=path.parent.parent;check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row;db.execute('PRAGMA query_only=ON');db.execute('BEGIN');snapshot=_snapshot(db,run_id)
    history=_history(session_log,journal_path,snapshot);now=_number(clock(),'clock');run=snapshot['run']
    deadline=run['started_at']+run['max_duration']
    _need(activated_execution_budget(path,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Original authorized budget changed')
    _need(history['ended_at']<=now<deadline and run['steps']<run['max_steps'],'Original budget exhausted')
    contract=_current_contract(root,snapshot['contract'])
    changes={k for k,v in contract['code'].items() if snapshot['contract']['code'].get(k)!=v}
    _need(changes<={'arms.py','pair_device.py','coherent_feedback.py','pair_host.py','pair_ledger.py','reboot_startup.py','feedback_continuation.py'}
          and {'arms.py','pair_device.py','coherent_feedback.py','feedback_continuation.py'}<=changes,
          'Only reviewed complete-frame RX repair and its successor are covered')
    proposal=dict(schema=SCHEMA,database=str(path),run_id=run_id,created_at=now,snapshot=snapshot,history=history,
        reviewed_contract=contract,evidence=_observations(passive_paths,rgb_observation,visual_observation,contract,history['ended_at'],now),
        budget=dict(started_at=run['started_at'],deadline_s=deadline,max_steps=run['max_steps'],steps=run['steps'],max_duration_s=run['max_duration']),
        hardware_commands_sent=0,new_budget_allocated=False,cache_or_limits_transferred=False,required_connection_mode='prepare',physical_stop_verified=None)
    return {**proposal,'proposal_sha256':_sha(proposal)}


def activate(proposal,*,project_root,clock=time.time):
    _need(proposal.get('schema')==SCHEMA and _sha({k:v for k,v in proposal.items() if k!='proposal_sha256'})==proposal.get('proposal_sha256'),
          'Exact reviewed proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(proposal['database'])
    _need(path==root/'runs/pair_sessions.sqlite','Authoritative project ledger required')
    ev=proposal['evidence'];history=proposal['history']
    kwargs=dict(session_log=history['session']['path'],journal_path=history['journal']['path'],
                passive_paths={s:ev['passive'][s]['source']['path'] for s in SIDES},
                rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare(path,proposal['run_id'],clock=lambda:proposal['created_at'],**kwargs)==proposal,'Evidence, source or ledger changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(_snapshot(db,proposal['run_id'])==proposal['snapshot'],'Ledger changed before activation')
                now=_number(clock(),'clock');_need(proposal['created_at']<=now<proposal['budget']['deadline_s'],'Original deadline reached')
                _need(_observations(kwargs['passive_paths'],kwargs['rgb_observation'],kwargs['visual_observation'],proposal['reviewed_contract'],history['ended_at'],now)==ev,
                      'Current evidence expired or changed')
                _need(_current_contract(root,proposal['snapshot']['contract'])==proposal['reviewed_contract'],'Reviewed source changed')
                snapshot=proposal['snapshot']
                record=dict(proposal=proposal,activated_at=now,new_contract=proposal['reviewed_contract'],hardware_commands_sent=0,
                            new_budget_allocated=False,old_rows_preserved=True,cache_or_limits_transferred=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,'
                           'last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,round_ordinal INTEGER NOT NULL,'
                           'preparation_ordinal INTEGER NOT NULL,contract_json TEXT NOT NULL,proposal_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL)')
                db.execute('INSERT INTO '+TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?,?)',
                           (proposal['run_id'],now,snapshot['scope']['owner'],json.dumps(snapshot['retired_owners']),snapshot['scope']['round_ordinal'],
                            snapshot['scope']['ordinal'],_json_object(record['new_contract'],'contract'),proposal['proposal_sha256'],_json_object(record,'record')))
                for name,digest in snapshot['table_sha256'].items():
                    rows=sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort)
                    _need(_table_sha(rows)==digest,'Historical rows changed: '+name)
                check_processes();_need(now<=_number(clock(),'commit clock')<proposal['budget']['deadline_s'],'Deadline reached before commit')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise
