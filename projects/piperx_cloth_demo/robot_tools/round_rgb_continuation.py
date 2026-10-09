"""One audited RGB-expiry successor per explicit round, same run and budget.

The old failed send, owner and fault stay intact. This administrative entry
never transmits or replays targets; current preparation remains compulsory.
"""
from contextlib import ExitStack
import json
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import _json_object, _number
from .pair_restart import _current_contract, _file, _need, _sha
from .pair_task_enrollment import _lock_roots, _observations
from .reboot_startup import check_processes

TABLE = 'pair_round_rgb_continuations'
SCHEMA = 'piper_round_rgb_expiry_continuation_v1'
KIND = 'postsend_rgb_expiry_fault'
REPAIR_FILES = {'pair_host.py','pair_joint_adapter.py','pair_ledger.py','pair_round.py',
                'pair_task_enrollment.py','configuration_recovery.py','round_rgb_continuation.py'}


def snapshot(db, run_id):
    from .pair_round import _snapshot
    if db.execute("SELECT 1 FROM sqlite_master WHERE name=?",(TABLE,)).fetchone():
        _need(not db.execute('SELECT 1 FROM '+TABLE+' WHERE run_id=?',(run_id,)).fetchone(),
              'This round already consumed its one RGB continuation')
    s = _snapshot(db,run_id,KIND)
    _need(s['scope_table'] == 'pair_rounds', 'Only the original current round can use this successor')
    return s


def history(path, s):
    from .pair_round import _closed
    closed = _closed(path,s)
    raw, ref = _file(path); rows = [json.loads(l) for l in raw.decode().splitlines()]
    _need(rows and [r['sequence'] for r in rows] == list(range(1,len(rows)+1))
          and [r['at'] for r in rows] == sorted(r['at'] for r in rows)
          and rows[-1].get('kind') == 'session_ended' and rows[-1].get('cleanup_errors') == [],
          'Full normally ended original session required')
    opens = [r for r in rows if r.get('kind') == 'result' and r.get('result',{}).get('status') == 'owned'
             and r['result'].get('open') is True and r['result'].get('owner') == s['retired_owner']]
    _need(opens and all(r['result'].get('run_id') == s['run']['run_id'] for r in opens)
          and s['last_finished_at'] < rows[-1]['at'], 'Original run/owner and exit chronology required')
    return {'close':closed,'session':ref,'ended_at':rows[-1]['at']}


def observed(evidence, s, contract, after, now):
    from .feedback_tolerance import task_policy, joint_tolerances, rotation_tolerance
    from .contact_receipt import _rotation_span
    import math
    result = _observations(**evidence,contract=contract,after=after,now=now)
    receipt = json.loads(s['rgb_expiry_fault_event']['receipt_json'])['device_receipt']
    plan = receipt['joint_path_plan']; arm = plan['identity']['arm']; policy = task_policy(contract['task'])
    for side,path in evidence['passive_paths'].items():
        raw,_ = _file(path); d = json.loads(raw)
        reference = (plan['encoded_target_joints_rad'] if side == arm else plan['origin']['arms'][side]['joints_rad'])
        for sample in d['pose_trace']:
            q = [sample['joints_raw']['joint_'+str(i)]*math.pi/180000 for i in range(1,7)]
            _need(all(lo <= v <= hi for v,(lo,hi) in zip(q,plan['effective_joint_limits_rad'][side]))
                  and all(abs(v-t) <= limit for v,t,limit in zip(q,reference,joint_tolerances(policy,side))),
                  'Fresh stationary pose must match the fully returned target and original peer, within hard limits')
            if side != arm:
                keys = ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')
                pose = [sample['end_pose_raw'][k]*(1e-6 if i < 3 else math.pi/180000) for i,k in enumerate(keys)]
                origin = receipt['before'][side]['pose_m_rad']
                _need(math.dist(pose[:3],origin[:3]) <= .0005
                      and _rotation_span([pose,origin]) <= rotation_tolerance(policy,side), 'Uncommanded peer moved')
        jaw = d['feedback']['PiperMsgGripperFeedBack']['fields']['grippers_angle']*1e-6
        _need(abs(jaw-receipt['before'][side]['gripper']['width_m']) <= .0005,'Uncommanded jaw changed')
    result.update(target_proximity_only=True, original_failed_result_preserved=True,
                  target_replay_authorized=False, physical_stop_verified=None)
    return result


def prepare(path, run_id, *, session_log, passive_paths, rgb_observation, visual_observation, clock=time.time):
    path = Path(path).resolve(strict=True); check_processes()
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row; db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
        s = snapshot(db,run_id)
    now = _number(clock(),'clock'); r=s['run']; deadline=r['started_at']+r['max_duration']
    _need(s['scope']['last_time'] <= now < deadline and r['steps'] < r['max_steps'], 'Original budget must remain')
    h=history(session_log,s); old=s['effective_contract']; new=_current_contract(path.parent.parent,old)
    changes={k for k,v in new['code'].items() if old['code'].get(k) != v}
    _need({'pair_joint_adapter.py','pair_host.py'} <= changes <= REPAIR_FILES,
          'Only the reviewed RGB timing repair and same-budget enrollment are covered')
    ev=dict(passive_paths=passive_paths,rgb_observation=rgb_observation,visual_observation=visual_observation)
    p=dict(schema=SCHEMA,database=str(path),run_id=run_id,created_at=now,snapshot=s,history=h,
        reviewed_contract=new,evidence=observed(ev,s,new,h['ended_at'],now),
        budget=dict(started_at=r['started_at'],deadline_s=deadline,max_steps=r['max_steps'],steps=r['steps'],max_duration_s=r['max_duration']),
        hardware_commands_sent=0,new_budget_allocated=False,cache_or_limits_transferred=False,
        required_connection_mode='prepare',physical_stop_verified=None)
    return {**p,'proposal_sha256':_sha(p)}


def activate(p, *, project_root, clock=time.time):
    _need(p.get('schema') == SCHEMA and _sha({k:v for k,v in p.items() if k != 'proposal_sha256'}) == p.get('proposal_sha256'),
          'Reviewed same-budget proposal required')
    root=Path(project_root).resolve(strict=True); path=Path(p['database']); ev=p['evidence']
    _need(path == root/'runs/pair_sessions.sqlite','Canonical ledger required')
    evidence=dict(passive_paths={s:v['source']['path'] for s,v in ev['passive'].items()},
        rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(prepare(path,p['run_id'],session_log=p['history']['session']['path'],clock=lambda:p['created_at'],**evidence) == p,
          'Evidence or source changed')
    with ExitStack() as locks:
        for directory in _lock_roots(root):
            if directory == root or (directory/'runs').is_dir(): locks.enter_context(ExclusiveExecution(directory/'runs'))
        check_processes()
        with sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None) as db:
            db.row_factory=sqlite3.Row;db.execute('PRAGMA synchronous=FULL');db.execute('BEGIN IMMEDIATE')
            try:
                s=snapshot(db,p['run_id']); _need(s == p['snapshot'],'History changed')
                now=_number(clock(),'clock');_need(p['created_at'] <= now < p['budget']['deadline_s'],'Original deadline reached')
                _need(history(p['history']['session']['path'],s) == p['history'],'Closure changed')
                _need(observed(evidence,s,p['reviewed_contract'],p['history']['ended_at'],now) == ev,'Current evidence expired or changed')
                _need(_current_contract(root,s['effective_contract']) == p['reviewed_contract'],'Reviewed code changed')
                record=dict(proposal=p,activated_at=now,new_contract=p['reviewed_contract'],hardware_commands_sent=0,
                    new_budget_allocated=False,old_rows_preserved=True,cache_or_limits_transferred=False,
                    required_connection_mode='prepare',physical_stop_verified=None)
                db.execute('CREATE TABLE IF NOT EXISTS '+TABLE+' (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,'
                    'owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,previous_owner TEXT NOT NULL,'
                    'retired_owners_json TEXT NOT NULL,round_ordinal INTEGER UNIQUE NOT NULL,contract_json TEXT NOT NULL,'
                    'proposal_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL)')
                ordinal=db.execute('SELECT COALESCE(MAX(ordinal),0)+1 FROM '+TABLE).fetchone()[0]
                db.execute('INSERT INTO '+TABLE+' VALUES(?,?,NULL,NULL,NULL,?,?,?,?,?,?,?)',
                    (ordinal,p['run_id'],now,s['retired_owner'],json.dumps(s['retired_owners']),s['scope_ordinal'],
                     _json_object(record['new_contract'],'contract'),p['proposal_sha256'],_json_object(record,'continuation')))
                check_processes();_need(now <= _number(clock(),'commit clock') < p['budget']['deadline_s'],'Original deadline reached')
                _need(_current_contract(root,s['effective_contract']) == p['reviewed_contract'],'Code changed during commit')
                db.execute('COMMIT'); return record
            except BaseException:
                if db.in_transaction: db.execute('ROLLBACK')
                raise


def audit_row(db, row, run):
    """Verify append-only enrollment after later child runtime events as well."""
    record=json.loads(row['record_json']);p=record['proposal'];s=p['snapshot'];b=p['budget']
    changes={k for k,v in p['reviewed_contract']['code'].items() if s['effective_contract']['code'].get(k) != v}
    _need({'pair_joint_adapter.py','pair_host.py'} <= changes <= REPAIR_FILES, 'Unreviewed continuation code changes')
    _need(p.get('schema') == SCHEMA and p['run_id'] == row['run_id'] == run['run_id']
          and _sha({k:v for k,v in p.items() if k != 'proposal_sha256'}) == row['proposal_sha256'] == p['proposal_sha256']
          and record['new_contract'] == p['reviewed_contract'] == json.loads(row['contract_json'])
          and record.get('new_budget_allocated') is False and record.get('old_rows_preserved') is True
          and record.get('hardware_commands_sent') == 0 and record.get('cache_or_limits_transferred') is False
          and record.get('required_connection_mode') == p.get('required_connection_mode') == 'prepare'
          and {k:v for k,v in s['effective_contract'].items() if k!='code'} == {k:v for k,v in record['new_contract'].items() if k!='code'}
          and all(run[k] == s['run'][k] for k in ('started_at','max_steps','max_duration','contract_json'))
          and b == dict(started_at=run['started_at'],deadline_s=run['started_at']+run['max_duration'],max_steps=run['max_steps'],steps=s['run']['steps'],max_duration_s=run['max_duration'])
          and p['created_at'] <= record['activated_at'] <= row['last_time']
          and record['activated_at'] < b['deadline_s'] and row['round_ordinal'] == s['scope_ordinal']
          and row['previous_owner'] == s['retired_owner'] and json.loads(row['retired_owners_json']) == s['retired_owners'],
          'Same-run original budget, owner retirement and exact enrollment required')
    shadow=sqlite3.connect(':memory:');shadow.row_factory=sqlite3.Row
    try:
        shadow.executescript('\n'.join(db.iterdump()))
        for name in ('pair_events','pair_faults','pair_joint_sends','pair_holds','pair_hold_requests','pair_hold_frames','pair_grasp_episodes'):
            if shadow.execute('SELECT 1 FROM sqlite_master WHERE name=?',(name,)).fetchone():
                if name=='pair_events': shadow.execute('DELETE FROM '+name+' WHERE run_id=? AND step>?',(run['run_id'],b['steps']))
                else:
                    for item in list(shadow.execute('SELECT rowid AS audit_rowid,* FROM '+name+' WHERE run_id=?',(run['run_id'],))):
                        # Never hide a retired owner's changed historical row.
                        if 'owner' in item.keys() and item['owner'] not in s['retired_owners']:
                            shadow.execute('DELETE FROM '+name+' WHERE rowid=?',(item['audit_rowid'],))
        shadow.execute('UPDATE pair_runs SET steps=? WHERE run_id=?',(b['steps'],run['run_id']))
        if TABLE in s['table_rows']: shadow.execute('DELETE FROM '+TABLE+' WHERE ordinal=?',(row['ordinal'],))
        else:
            _need(shadow.execute('SELECT COUNT(*) FROM '+TABLE).fetchone()[0]==1,'Unexplained continuation rows')
            shadow.execute('DROP TABLE '+TABLE)
        _need(snapshot(shadow,run['run_id']) == s,'Original parent history changed')
    finally: shadow.close()
    _need(history(p['history']['session']['path'],s) == p['history'],'Original closure changed')
    ev=p['evidence'];args=dict(passive_paths={k:v['source']['path'] for k,v in ev['passive'].items()},
        rgb_observation=ev['rgb']['source']['path'],visual_observation=ev['visual_observation'])
    _need(observed(args,s,p['reviewed_contract'],p['history']['ended_at'],p['created_at']) == ev,'Archived admission evidence changed')
    return True
