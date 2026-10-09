"""One audited continuation after the diagnosed idle-close race, no robot I/O.

Only the successful query + two measured-width jaw preparations are covered.
Append a scope for the SAME run/budget; never edit the old fault, events, run,
or source files. A successor must establish current preparation from scratch.
"""
from contextlib import ExitStack
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
import time

from .execution import ExclusiveExecution
from .pair_ledger import _json_object, _number, activated_execution_budget
from .pair_restart import _current_contract, _file, _need, _sha
from .pair_task_enrollment import _lock_roots, _observations
from .reboot_startup import check_processes

TABLE = 'pair_preparation_continuations'
SCHEMA = 'piper_preparation_close_continuation_v1'
FAULT = 'Idle monitor: Pair latched or closing; no further target frames'
SIDES = ('left', 'right')


def _counts(left, right):
    return {s:dict(attempted_frames=n, sent_frames=n, blocked_frames=0)
            for s,n in zip(SIDES, (left, right))}


def _same_counts(actual, expected):
    return (type(actual) is dict and set(actual)==set(expected)
            and all(type(actual[s]) is dict and set(actual[s])==set(expected[s])
                    and all(type(v) is int for v in actual[s].values()) for s in expected)
            and actual==expected)


def _snapshot(db, run_id):
    from .joint_sources import _validate_bindings, _validated_limits
    tables = {}
    for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'"):
        _need(name.replace('_', '').isalnum(), 'Unexpected table identifier')
        tables[name] = sorted([dict(r) for r in db.execute('SELECT * FROM '+name)], key=_json_sort)
    _need(TABLE not in tables, 'This once-only preparation continuation already exists')
    _need(tables.get('pair_rounds'), 'Existing authorized round required')
    scope = max(tables['pair_rounds'], key=lambda r:r['ordinal'])
    run = next((r for r in tables['pair_runs'] if r['run_id'] == run_id), None)
    _need(run and scope['run_id'] == scope['active_run_id'] == run_id and scope['owner']
          and scope['fault_id'] is not None, 'Current retired owner and round required')
    record = json.loads(scope['record_json'])
    _need(record.get('proposal', {}).get('parent_kind') == 'postreboot_supervised_plug_task'
          and record.get('required_connection_mode') == 'prepare', 'Only the enrolled preparation round is covered')
    old = json.loads(run['contract_json'])
    _need(old == record.get('new_contract') and old['task']['task_id'] == 'plug_transfer_left',
          'Original task contract differs')
    _need(not any(e['status'] == 'pending' for e in tables['pair_events']), 'Pending sends prohibit continuation')
    _need(not any(e.get('status')=='pending' for e in tables.get('pair_holds',[])),
          'Pending hold prohibits continuation')
    from .grasp_episode import is_resolved_release
    for episode in tables.get('pair_grasp_episodes',[]):
        state=json.loads(episode['state_json'])
        _need(state.get('status')=='empty' or is_resolved_release(state),'Unresolved grasp prohibits continuation')
    for name in ('pair_joint_sends', 'pair_hold_requests', 'pair_holds', 'pair_hold_frames', 'pair_grasp_episodes'):
        _need(not any(r['run_id'] == run_id for r in tables.get(name, [])), 'Joint/grasp/hold history is outside this scope')
    events = sorted([e for e in tables['pair_events'] if e['run_id'] == run_id], key=lambda e:e['step'])
    _need(run['steps'] == 3 and [e['step'] for e in events] == [1,2,3], 'Exact query and two-jaw history required')
    faults = [f for f in tables['pair_faults'] if f['run_id'] == run_id]
    _need(len(faults) == 1 and faults[0]['id'] == scope['fault_id']
          and faults[0]['owner'] == scope['owner'] and faults[0]['reason'] == FAULT,
          'Only the diagnosed idle-close guard fault is covered')
    jaws = set()
    total = _counts(0,0)
    for i,e in enumerate(events):
        p,r = json.loads(e['payload_json']), json.loads(e['receipt_json'])
        _need(e['owner'] == scope['owner'] and e['status'] == 'complete' and e['success'] == 1
              and hashlib.sha256(e['payload_json'].encode()).hexdigest() == e['payload_digest']
              and r.get('ok') is True and r.get('fault_latched') is False
              and r.get('errors') == r.get('guard_violations') == []
              and r.get('event_id') == e['event_id'] and r.get('pair_owner') == scope['owner'],
              'Every original preparation must have an exact complete successful receipt')
        _need(e['began_at'] < e['finished_at'] < faults[0]['at']
              and (i == 0 or events[i-1]['finished_at'] < e['began_at']), 'Preparation chronology differs')
        if i == 0:
            _need(p.get('kind') == 'query' and p.get('request') == {'operation':'inspect_joint_limits'},
                  'First event must be the existing limit query')
            _validate_bindings(old, p['bindings'])
            _validated_limits({**r, 'run_id':run_id, 'owner':scope['owner'], 'bindings':p['bindings']},
                              run_id=run_id, owner=scope['owner'], bindings=p['bindings'], now=e['finished_at'])
            expected = _counts(6,6)
            for k,n in (('hardware_commands_sent',12), ('joint_limit_queries_sent',12),
                        ('joint_limit_queries_attempted',12), ('actuator_commands_sent',0),
                        ('target_commands_sent',0), ('mode_commands_sent',0), ('enable_commands_sent',0)):
                _need(type(r.get(k)) is int and r[k] == n, 'Query counter differs: '+k)
        else:
            q = p.get('request', {}); side = q.get('arm')
            _need(p.get('kind') == 'preparation' and q.get('operation') == 'prepare_gripper'
                  and side in SIDES and side not in jaws and r.get('arm') == side
                  and isinstance(q.get('empty_jaw_observation'),str) and q['empty_jaw_observation'].strip()
                  and r.get('status') == 'selected_gripper_prepared_not_task_ready'
                  and r.get('operation') == 'prepare_gripper' and r.get('grasp_verified') is False
                  and r.get('physical_stop_verified') is None and r.get('force_nominal_N') == .2,
                  'Only each empty jaw prepared once at its measured width is covered')
            jaws.add(side)
            expected = _counts(1 if side=='left' else 0, 1 if side=='right' else 0)
            for k,n in (('hardware_commands_sent',1), ('gripper_enable_commands_sent',1),
                        ('gripper_target_commands_sent',1), ('target_commands_sent',1),
                        ('arm_target_commands_sent',0), ('mode_commands_sent',0),
                        ('enable_commands_sent',0), ('stop_commands_sent',0), ('retries',0),
                        ('passive_arm_commands_sent',0)):
                _need(type(r.get(k)) is int and r[k] == n, 'Jaw counter differs: '+k)
            before,after = r['before'][side]['gripper'],r['after'][side]['gripper']
            raw = r.get('target_width_raw')
            _need(type(raw) is int and 5000 <= raw <= 70000
                  and before['foc_status']['driver_enable_status'] is False
                  and after['foc_status']['driver_enable_status'] is True
                  and raw == round(before['width_m']*1e6)
                  and abs(after['width_m']-raw/1e6) <= .001, 'Jaw target/arrival does not match measured-width preparation')
        _need(_same_counts(r.get('transmission_counts'),expected), 'Original frame counters differ')
        for side in SIDES:
            for k,n in expected[side].items(): total[side][k] += n
        _need(_same_counts(r.get('session_transmission_counts'),total), 'Unaccounted lifetime transmission')
    retired = sorted({r['owner'] for rows in tables.values() for r in rows if r.get('owner')})
    return dict(run=run, scope=scope, fault=faults[0], events=events, contract=old, retired_owners=retired,
                totals=total, table_sha256={k:_table_sha(v) for k,v in tables.items()})


def _json_sort(value):
    return json.dumps(value, sort_keys=True)


def _table_sha(rows):
    return hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()


def _journal(path):
    # Long idle sessions have large feedback journals. Hash every original byte
    # while retaining only event metadata, instead of copying the full trace.
    path=Path(path).absolute()
    _need(path==path.resolve() and path.is_file(),'Existing nonsymlink journal required')
    before=path.stat();_need(before.st_size<=128*1024*1024,'Journal exceeds 128 MiB audit bound')
    digest=hashlib.sha256();rows=[];previous=0.
    with path.open('rb') as stream:
        for line in stream:
            _need(len(line)<=1024*1024,'Oversized journal row')
            digest.update(line);row=json.loads(line);stamp=_number(row.get('unix_s'),'journal time')
            _need(stamp>=previous,'Regressing host journal time');previous=stamp
            if row.get('event')!='feedback':rows.append(row)
    _need(path.stat()==before,'Host journal changed during audit')
    return rows,dict(path=str(path),sha256=digest.hexdigest())


def _history(session_log, journal_path, snapshot):
    raw,ref = _file(session_log); rows = [json.loads(l) for l in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows] == list(range(1,len(rows)+1))
          and [r['at'] for r in rows] == sorted(r['at'] for r in rows), 'Complete ordered session journal required')
    requests = [r for r in rows if r['kind']=='request']
    allowed = {'capture','robot_pair_status','robot_pair_observe','robot_pair_open',
               'robot_pair_inspect_joint_limits','robot_pair_prepare_gripper',
               'robot_pair_initialize_joint_target','robot_pair_close'}
    _need(all(r['request']['op'] in allowed for r in requests), 'Unexpected tool request in preparation history')
    def selected(op): return [r for r in requests if r['request']['op']==op]
    _need(len(selected('robot_pair_open')) == len(selected('robot_pair_close'))
          == len(selected('robot_pair_initialize_joint_target')) == 1
          and len(selected('robot_pair_inspect_joint_limits')) == 1
          and len(selected('robot_pair_prepare_gripper')) == 2, 'Unexpected/repeated device requests')
    init = selected('robot_pair_initialize_joint_target')[0]
    errors = [r for r in rows if r['kind']=='request_error']
    _need(len(errors)==1 and errors[0]['request_id']==init['request']['id']
          and errors[0]['error'].startswith('JointSourcesError: joint_sources_unavailable: source_index_missing:')
          and '; controller_limits_capture_missing:' in errors[0]['error'], 'Initialization must have the exact preclaim lookup rejection')
    _need(init['request']['arguments']['event_id'] not in {e['event_id'] for e in snapshot['events']},
          'Rejected initialization must have no claimed event')
    close_request = selected('robot_pair_close')[0]
    closed = [r for r in rows if r.get('request_id')==close_request['request']['id'] and r['kind']=='result']
    _need(len(closed)==1 and close_request['at'] <= snapshot['fault']['at'] <= closed[0]['at'],
          'Fault must occur inside this actual normal close call')
    c = closed[0]['result']; cleanup=c.get('cleanup',{})
    _need(c.get('status')=='closed' and c.get('fault_latched') is True
          and cleanup.get('requires_fault_latch') is False and cleanup.get('guard_violations')==[]
          and cleanup.get('unresolved_gripper_probe') is None and cleanup.get('grasp_states')==dict.fromkeys(SIDES)
          and _same_counts(cleanup.get('session_transmission_counts'),snapshot['totals'])
          and all(cleanup.get('arms',{}).get(s,{}).get('status')=='disconnected' for s in SIDES),
          'Close must account for both empty arms and all known frames')
    ended=rows[-1]
    _need(ended['kind']=='session_ended' and ended.get('cleanup_errors')==[] and ended['at']>=closed[0]['at'],
          'Completed session cleanup required')
    opened=[r for r in rows if r.get('request_id')==selected('robot_pair_open')[0]['request']['id'] and r['kind']=='result']
    _need(len(opened)==1 and opened[0]['result'].get('run_id')==snapshot['run']['run_id']
          and opened[0]['result'].get('owner')==snapshot['scope']['owner'], 'Session owner binding differs')
    journal,jref=_journal(journal_path)
    allowed_events={'feedback','connected_passively','pair_preparation_claimed','pair_shared_scene',
        'pair_joint_limit_query_intent','pair_joint_limit_reply','single_gripper_prepare_intent',
        'single_gripper_frame_sent_unconfirmed','single_gripper_enabled_at_observed_width'}
    _need(all(r.get('event') in allowed_events for r in journal), 'Unexpected host journal event/target')
    claims=[r for r in journal if r['event']=='pair_preparation_claimed']
    _need([r.get('event_id') for r in claims]==[e['event_id'] for e in snapshot['events']]
          and all(r.get('payload')==json.loads(e['payload_json']) for r,e in zip(claims,snapshot['events'])),
          'Host claims differ from durable event history')
    intents=[r for r in journal if r['event']=='single_gripper_prepare_intent']
    returns=[r for r in journal if r['event']=='single_gripper_frame_sent_unconfirmed']
    _need(len(intents)==len(returns)==2, 'Exactly two known jaw frames required')
    for e,intent,sent in zip(snapshot['events'][1:],intents,returns):
        receipt=json.loads(e['receipt_json']);side=receipt['arm']
        expected=(struct.pack('>i',receipt['target_width_raw'])+b'\x00\xc8\x01\x00').hex()
        _need(intent.get('side')==sent.get('side')==side and intent.get('arbitration_id')==0x159
              and intent.get('data_hex')==expected and intent.get('set_zero') is False
              and e['began_at']<=intent['unix_s']<=sent['sent_at']<=sent['unix_s']<=e['finished_at']
              and _same_counts(sent.get('transmission_counts'),receipt['transmission_counts']), 'Jaw wire evidence differs')
    return dict(session=ref, journal=jref, ended_at=ended['at'], close_receipt=c)


def prepare(path, run_id, *, session_log, journal_path, passive_paths, rgb_observation,
            visual_observation, clock=time.time):
    path=Path(path).resolve(strict=True); root=path.parent.parent
    check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row; db.execute('PRAGMA query_only=ON');db.execute('BEGIN')
        snapshot=_snapshot(db,run_id)
    history=_history(session_log,journal_path,snapshot)
    now=_number(clock(),'clock');run=snapshot['run'];deadline=run['started_at']+run['max_duration']
    _need(activated_execution_budget(path,run_id,max_steps=run['max_steps'],max_duration_s=run['max_duration']),
          'Original task and budget authorization must remain unchanged')
    _need(history['ended_at']<=now<deadline and run['steps']<run['max_steps'], 'Original remaining budget exhausted')
    contract=_current_contract(root,snapshot['contract'])
    changes={k for k,v in contract['code'].items() if snapshot['contract']['code'].get(k)!=v}
    _need(changes <= {'service.py','pair_host.py','pair_ledger.py','reboot_startup.py','preparation_continuation.py'},
          'Unrelated controller/threshold changes are outside this reviewed continuation')
    _need(all(contract['code'][k]!=snapshot['contract']['code'][k] for k in ('service.py','pair_host.py'))
          and 'preparation_continuation.py' in contract['code'], 'Reviewed source and close fixes must be frozen')
    proposal=dict(schema=SCHEMA,database=str(path),run_id=run_id,created_at=now,snapshot=snapshot,
        history=history,reviewed_contract=contract,
        evidence=_observations(passive_paths,rgb_observation,visual_observation,contract,history['ended_at'],now),
        budget=dict(started_at=run['started_at'],deadline_s=deadline,max_steps=run['max_steps'],
                    steps=run['steps'],max_duration_s=run['max_duration']),
        hardware_commands_sent=0,new_budget_allocated=False,cache_or_limits_transferred=False,
        required_connection_mode='prepare',physical_stop_verified=None)
    return {**proposal,'proposal_sha256':_sha(proposal)}


def activate(proposal, *, project_root, clock=time.time):
    _need(proposal.get('schema')==SCHEMA and _sha({k:v for k,v in proposal.items() if k!='proposal_sha256'})
          ==proposal.get('proposal_sha256'), 'Exact reviewed proposal required')
    root=Path(project_root).resolve(strict=True);path=Path(proposal['database'])
    _need(path==root/'runs/pair_sessions.sqlite','Authoritative project ledger required')
    ev=proposal['evidence'];history=proposal['history']
    kwargs=dict(session_log=history['session']['path'],journal_path=history['journal']['path'],
                passive_paths={s:ev['passive'][s]['source']['path'] for s in SIDES},
                rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare(path,proposal['run_id'],clock=lambda:proposal['created_at'],**kwargs)==proposal,
          'Source, observations or ledger changed after review')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory==root or (directory/'runs').is_dir():locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                _need(_snapshot(db,proposal['run_id'])==proposal['snapshot'],'Ledger changed before activation')
                now=_number(clock(),'clock')
                _need(proposal['created_at']<=now<proposal['budget']['deadline_s'],'Original deadline reached')
                _need(_observations(kwargs['passive_paths'],kwargs['rgb_observation'],kwargs['visual_observation'],
                                    proposal['reviewed_contract'],history['ended_at'],now)==ev,'Fresh evidence changed')
                _need(_current_contract(root,proposal['snapshot']['contract'])==proposal['reviewed_contract'],
                      'Reviewed code changed')
                snapshot=proposal['snapshot'];owner=snapshot['scope']['owner']
                record=dict(proposal=proposal,activated_at=now,new_contract=proposal['reviewed_contract'],
                            hardware_commands_sent=0,new_budget_allocated=False,old_rows_preserved=True,
                            cache_or_limits_transferred=False,physical_stop_verified=None)
                db.execute('CREATE TABLE '+TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,'
                    'owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,previous_owner TEXT NOT NULL,retired_owners_json TEXT NOT NULL,'
                    'round_ordinal INTEGER NOT NULL,contract_json TEXT NOT NULL,proposal_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL)')
                db.execute('INSERT INTO '+TABLE+' VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (proposal['run_id'],now,owner,json.dumps(snapshot['retired_owners']),snapshot['scope']['ordinal'],_json_object(record['new_contract'],'contract'),
                     proposal['proposal_sha256'],_json_object(record,'record')))
                # Verify the operation only added its dedicated scope.
                for name,digest in snapshot['table_sha256'].items():
                    rows=sorted([dict(r) for r in db.execute('SELECT * FROM '+name)],key=_json_sort)
                    _need(_table_sha(rows)==digest,'Historical rows changed: '+name)
                check_processes()
                _need(now<=_number(clock(),'commit clock')<proposal['budget']['deadline_s'],'Deadline reached before commit')
                db.execute('COMMIT');return record
            except BaseException:
                if db.in_transaction:db.execute('ROLLBACK')
                raise
