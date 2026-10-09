"""Append-only admission for a new supervised plug task after this host reboot.

This trusted administrative API sends nothing. It covers only the audited
unloaded RGB-expiry history followed by the known four-frame startup that
failed its right-joint drift observation. It never declares that startup
successful, retries it, or transfers targets. Current observations use the
frozen task profile; historical startup checks and hard limits stay unchanged.
The successor must open in preparation mode and obtain normal live admission.
"""
from contextlib import ExitStack
import copy
from .feedback_tolerance import rotation_tolerance
from .feedback_tolerance import task_policy, joint_tolerances
from .task_roles import resolve_task_roles, role_fields
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import _identifier, _json_object, _number
from .pair_restart import _file, _need, _sha
from .pair_round import _snapshot, _closed, _counts, _current_contract
from .reboot_startup import boot_identity, check_processes, project_roots, SCOPE_TABLES, CONFIRMATION


SCHEMA = 'piper_postreboot_supervised_task_v1'
PARENT_KIND = 'postreboot_supervised_plug_task'
STARTUP_ERROR = 'right drift joint_rad=0.005603 exceeds 0.003000'


def _lock_roots(root):
    return sorted(set(project_roots(root)) | {root.parent/'piper_right_pick_demo',Path('/home/agilex/piper_right_pick_demo')})


def preparation_only(path, run_id):
    """Read-only restriction on the enrolled scope; never an ownership grant."""
    path = Path(path).resolve()
    if not path.exists():
        return False
    with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
        db.execute('PRAGMA query_only=ON')
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reset_task_budgets'").fetchone():
            if db.execute('SELECT 1 FROM pair_reset_task_budgets WHERE run_id=?', (run_id,)).fetchone():
                return True
        if not db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_rounds'").fetchone():
            return False
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_round_rgb_continuations'").fetchone():
            if db.execute('SELECT 1 FROM pair_round_rgb_continuations WHERE run_id=? AND round_ordinal='
                          '(SELECT MAX(ordinal) FROM pair_rounds)',(run_id,)).fetchone():
                return True
        row = db.execute('SELECT run_id,record_json FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        return bool(row and row[0] == run_id and
                    json.loads(row[1]).get('proposal', {}).get('parent_kind') in
                    (PARENT_KIND, 'completed_unloaded_joint_fault', 'configuration_maintenance_fault', 'supported_contact_zero_tx_fault'))


def _startup(db, path, boot, now):
    _need(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reboot_startups'").fetchone(),
          'Existing failed reboot startup required')
    rows = [dict(r) for r in db.execute('SELECT * FROM pair_reboot_startups WHERE boot_id=? ORDER BY arm',
                                      (boot['boot_id'],))]
    _need(len(rows) == 2 and [r['arm'] for r in rows] == ['left','right']
          and all(r['status'] == 'failed' for r in rows)
          and len({r['run_id'] for r in rows}) == len({r['record_path'] for r in rows}) == 1,
          'Exactly the two terminal failed claims from this boot are required')
    finished = [_number(r['finished_at'], 'startup finish') for r in rows]
    started = [_number(r['started_at'], 'startup start') for r in rows]
    _need(len(set(finished)) == 1 and boot['started_at'] <= min(started) <= max(started) < finished[0] <= now,
          'Startup claim chronology differs')
    directory = path.parent / rows[0]['run_id']
    _need(Path(rows[0]['record_path']) == directory/'result.json', 'Canonical startup record path required')
    result_raw, result_ref = _file(directory/'result.json')
    request_raw, request_ref = _file(directory/'request.json')
    journal_raw, journal_ref = _file(directory/'events.jsonl')
    result, request = json.loads(result_raw), json.loads(request_raw)
    enrollment = request.get('enrollment', {})
    _need(request.get('run_id') == result.get('run_id') == rows[0]['run_id']
          and request.get('arm') == 'both' and request.get('operator_statement') == CONFIRMATION
          and result.get('record_path') == rows[0]['record_path']
          and enrollment.get('route') == 'reboot_startup' and enrollment.get('boot') == boot
          and result.get('reboot_startup') == enrollment,
          'Actual startup request/result/boot binding required')
    scopes, times = [], []
    for table in SCOPE_TABLES:
        if not db.execute("SELECT 1 FROM sqlite_master WHERE name=?", (table,)).fetchone():
            continue
        for row in db.execute('SELECT * FROM '+table):
            value = dict(row); times.append(value['last_time'])
            scopes.append({'table':table,'ordinal':value.get('ordinal',1),
                'run_id':value.get('run_id',value.get('active_run_id')), 'owner':value['owner'],
                'fault_id':value['fault_id'],
                'sha256':hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()})
    for table, field in (('pair_events','began_at'),('pair_events','finished_at'),
                         ('pair_faults','at'),('pair_runs','started_at')):
        value = db.execute('SELECT MAX('+field+') FROM '+table).fetchone()[0]
        if value is not None: times.append(value)
    _need(times and all(_number(t,'old activity') < boot['started_at'] for t in times),
          'All old task activity must precede this boot')
    _need(enrollment.get('ledgers') == [{'database':str(path),'scopes':scopes,'last_task_activity_at':max(times)}],
          'Startup must bind the unchanged authoritative historical scopes')
    expected_counts = {s:dict(attempted_frames=2,sent_frames=2,blocked_frames=0) for s in ('left','right')}
    _need(result.get('ok') is False and result.get('status') == 'aborted_after_dispatch'
          and result.get('operation') == 'startup_arms'
          and result.get('errors') == [{'type':'RuntimeError','detail':STARTUP_ERROR}]
          and result.get('guard_violations') == [] and _counts(result.get('transmission_counts')) == expected_counts
          and result.get('transmission_counts_by_kind') == {s:{k:dict(attempted_frames=1,sent_frames=1)
              for k in ('mode','enable')} for s in ('left','right')}
          and result.get('old_task_faults_preserved') is True and result.get('task_motion_authorized') is False
          and result.get('motion_gate_unlocked') is False and result.get('grasp_verified') is False,
          'Only the exact known mode/enable drift failure is covered')
    for key, value in (('hardware_commands_sent',4),('enable_commands_sent',2),('target_commands_sent',0),
                        ('gripper_target_commands_sent',0),('stop_commands_sent',0),('retries',0)):
        _need(type(result.get(key)) is int and result[key] == value, 'Startup counter differs: '+key)
    _need(all(type(value) is int for side in result['transmission_counts_by_kind'].values()
              for kind in side.values() for value in kind.values()), 'Startup per-kind counters must be integers')
    _need(result.get('last_enable_feedback') == {s:{'driver_enabled':[True]*6,'gripper_enabled':False}
              for s in ('left','right')}
          and all(result.get('cleanup',{}).get('arms',{}).get(s,{}).get('status') == 'disconnected'
                  for s in ('left','right')),
          'Startup terminal feedback and disconnected cleanup required')
    journal = [json.loads(line) for line in journal_raw.decode().splitlines()]
    stamps = [_number(row.get('unix_s'),'startup journal time') for row in journal]
    _need(stamps and stamps == sorted(stamps) and boot['started_at'] <= stamps[0] <= stamps[-1] <= finished[0],
          'Startup journal chronology differs')
    events = [r for r in journal if r.get('event') != 'feedback']
    names = ['operation_started','connected_passively','mode_request_intent','mode_frame_sent_unconfirmed',
             'can_control_observed','mode_request_intent','mode_frame_sent_unconfirmed','can_control_observed',
             'enable_request_intent','enable_frame_sent_unconfirmed','joints_enabled_observed',
             'enable_request_intent','enable_frame_sent_unconfirmed']
    _need([r.get('event') for r in events] == names, 'Unknown, missing or extra startup journal event')
    totals = {s:dict(attempted_frames=0,sent_frames=0,blocked_frames=0) for s in ('left','right')}
    _need(_counts(events[1].get('transmission_counts')) == totals, 'Startup connection must be zero TX')
    for index, side, kind in ((2,'left','mode'),(5,'right','mode'),(8,'left','enable'),(11,'right','enable')):
        intent, sent = events[index:index+2]
        _need(intent.get('side') == sent.get('side') == side
              and type(intent.get('arbitration_id')) is int
              and intent['arbitration_id'] == (0x151 if kind == 'mode' else 0x471)
              and intent.get('data_hex') == ('0100010000000000' if kind == 'mode' else '0702000000000000')
              and intent.get('cached_target_activation_possible') is True
              and max(started) <= intent['unix_s'] <= _number(sent.get('sent_at'),'startup send time') <= sent['unix_s'],
              'Exact once-only startup frame/side/return chronology required')
        if kind == 'mode':
            _need(type(intent.get('mode_feedback')) is int and intent['mode_feedback'] == 0
                  and type(intent.get('speed_percent')) is int and intent['speed_percent'] == 1,
                  'Original low-speed P mode request required')
        else:
            _need(intent.get('sdk_api') == 'enable(255)' and sent.get('sdk_cached_return') is False,
                  'Original enable API/returned feedback required')
        totals[side]['attempted_frames'] += 1; totals[side]['sent_frames'] += 1
        _need(_counts(sent.get('transmission_counts')) == totals, 'Startup cumulative frame returns differ')
    _startup_feedback(journal,events,result)
    return {'boot':boot,'claims':rows,'result':result_ref,'request':request_ref,'journal':journal_ref,
            'finished_at':finished[0],'known_frames_returned':4,'failed_result_preserved':True}


def _startup_feedback(journal, events, result):
    """Bind the historical failure to its complete feedback, never relabel it stable."""
    from .arms import control_health
    rows = [r for r in journal if r['event'] == 'feedback']
    _need(type(result.get('samples')) is int and len(rows) == result['samples'] and rows,
          'Complete original startup feedback journal required')
    pair = lambda r: {s:r.get(s) for s in ('left','right')}
    anchors = [i for i,r in enumerate(rows) if pair(r) == result.get('before')]
    _need(len(anchors) == 1 and rows[anchors[0]]['unix_s'] < events[2]['unix_s']
          and pair(rows[-1]) == result.get('after'), 'Original before/after must bind actual journal snapshots')
    for row in rows[:anchors[0]]:
        for side in ('left','right'):
            state = row.get(side) or {}; status = state.get('arm_status') or {}
            flags = [v.get('foc_status',{}) for v in (state.get('drivers') or {}).values()]
            flags.append((state.get('gripper') or {}).get('foc_status',{}))
            _need(not any(state.get(k) for k in ('communication_error','cleanup_error','error'))
                  and not any(type(status.get(k)) is int and status[k] != 0 for k in ('arm_status','err_code'))
                  and not any(v is True for v in (status.get('err_status') or {}).values())
                  and not any(v is True for group in flags for k,v in group.items() if k != 'driver_enable_status'),
                  'Known warmup fault cannot be ignored as incomplete feedback')
    anchor = result['before']; previous = {s:[False]*6 for s in ('left','right')}
    drift = {s:dict(joint_rad=0.,position_m=0.,gripper_m=0.) for s in ('left','right')}
    for row in rows[anchors[0]:]:
        for side, mode_sent, enable_sent in (('left',events[3]['sent_at'],events[9]['sent_at']),
                                             ('right',events[6]['sent_at'],events[12]['sent_at'])):
            state = row.get(side); at = row['unix_s']
            _need(type(state) is dict and state.get('status') == 'complete'
                  and control_health(state,now_s=at,require_enabled=False,allowed_control_modes=(0,1))['healthy'],
                  'Other unhealthy startup feedback cannot be hidden by the drift summary')
            status = state['arm_status']
            _need(type(status.get('mode_feedback')) is int and status['mode_feedback'] == 0
                  and type(status.get('teach_status')) is int and status['teach_status'] == 0
                  and type(status.get('motion_status')) is int and status['motion_status'] == 0
                  and (at >= mode_sent or status['ctrl_mode'] == 0), 'Unexpected historical startup mode transition')
            flags = [state['drivers'][str(i)]['foc_status'].get('driver_enable_status') for i in range(1,7)]
            _need(all(type(f) is bool for f in flags) and state['gripper']['foc_status'].get('driver_enable_status') is False
                  and (at >= enable_sent or flags == [False]*6)
                  and all(not old or new for old,new in zip(previous[side],flags)),
                  'Unrequested or regressing startup enable transition')
            previous[side] = flags
            values = {'joint_rad':max(abs(a-b) for a,b in zip(state['joints_rad'],anchor[side]['joints_rad'])),
                      'position_m':math.dist(state['pose_m_rad'][:3],anchor[side]['pose_m_rad'][:3]),
                      'gripper_m':abs(state['gripper']['width_m']-anchor[side]['gripper']['width_m'])}
            for key,value in values.items(): drift[side][key] = max(drift[side][key],value)
    _need(previous == {'left':[True]*6,'right':[True]*6}
          and all(result['after'][s]['arm_status']['ctrl_mode'] == 1 for s in previous)
          and result.get('drift') == drift and drift['left']['joint_rad'] <= .003
          and abs(drift['right']['joint_rad']-.005603) <= .0000005
          and all(drift[s]['position_m'] <= .002 and drift[s]['gripper_m'] <= .002 for s in previous),
          'Recorded startup drift/terminal enabled flags differ from raw feedback history')


def _observations(passive_paths, rgb_observation, visual_observation, contract, after, now):
    """Separate preparation evidence contract; original continuation is unchanged."""
    _need(type(passive_paths) is dict and set(passive_paths) == {'left','right'}, 'Two passive records required')
    _need(type(visual_observation) is str and 1 <= len(visual_observation.strip()) <= 4000,
          'Actual current empty-jaw/no-contact/support interpretation required')
    passive = {}
    for side, path in passive_paths.items():
        raw, ref = _file(path); data = json.loads(raw)
        _need(data.get('mode') == 'passive_receive_only' and data.get('channel') == contract['arms'][side]['channel']
              and type(data.get('frames_sent_by_this_script')) is int and data['frames_sent_by_this_script'] == 0
              and type(data.get('malformed_frames')) is int and data['malformed_frames'] == 0
              and data.get('complete_feedback_received') is True and data.get('missing_feedback_types') == [],
              'Complete bound zero-TX passive evidence required')
        from .takeover import COMMAND_IDS
        origins, ids = data.get('frame_origin_counts',{}), data.get('frame_id_counts',{})
        _need(set(origins) == {'local','nonlocal'} and all(type(v) is int and v >= 0 for v in origins.values())
              and origins['local'] == 0 and origins['nonlocal'] > 0 and data.get('command_feedback') == {}
              and type(ids) is dict and ids
              and all(type(k) is str and k.startswith('0x') and type(v) is int and v > 0 for k,v in ids.items())
              and not any(int(k,16) in COMMAND_IDS for k in ids),
              'Observed local/control traffic or incomplete receive accounting blocks admission')
        began, ended = (_number(data.get(k),k) for k in ('started_at_s','finished_at_s'))
        _need(after < began < ended <= now and now-ended <= 30., 'Fresh post-startup passive evidence required')
        trace = data.get('pose_trace')
        _need(type(trace) is list and len(trace) >= 21, 'At least 21 raw passive samples required')
        stamps = [_number(r.get('received_at_s'),'trace time') for r in trace]
        _need(all(a < b for a,b in zip(stamps,stamps[1:])) and stamps[-1]-stamps[0] >= 3.
              and max(b-a for a,b in zip(stamps,stamps[1:])) <= .1
              and began <= stamps[0] <= stamps[-1] <= ended, 'Complete three-second advancing trace required')
        keys = [('joints_raw','joint_'+str(i),math.pi/180000,limit)
                for i,limit in enumerate(joint_tolerances(task_policy(contract['task']), side),1)]
        pose_keys = ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')
        keys += [('end_pose_raw',k,1e-6 if i < 3 else math.pi/180000,.0005 if i < 3 else None)
                 for i,k in enumerate(pose_keys)]
        spans = {}
        for group,key,unit,limit in keys:
            values = [r.get(group,{}).get(key) for r in trace]
            _need(all(type(v) is int for v in values), 'Complete raw integer trace required')
            spans[key] = (max(values)-min(values))*unit
            _need(limit is None or spans[key] <= limit,
                  'Passive body is not stationary within existing observation bounds: '+side+'/'+key)
            times = [_number(r.get('field_received_at_s',{}).get(key),'field timestamp') for r in trace]
            _need(all(a <= b for a,b in zip(times,times[1:]))
                  and all(0 <= at-t <= .1 for at,t in zip(stamps,times)), 'Stale/regressing passive fields')
        from .contact_receipt import _rotation_span
        poses = [[r['end_pose_raw'][k]*(1e-6 if i < 3 else math.pi/180000)
                  for i,k in enumerate(pose_keys)] for r in trace]
        rotation = _rotation_span(poses)
        _need(math.sqrt(sum(spans[k]**2 for k in pose_keys[:3])) <= .0005 and rotation <= rotation_tolerance(task_policy(contract["task"]),side),
              'Passive whole-position/rotation observation bound exceeded')
        status = data.get('raw_frame_latest',{}).get('0x2A1',{})
        value = bytes.fromhex(status.get('payload_hex',''))
        _need(len(value) == 8 and value[0:2] == b'\x01\x00' and value[2] in (0,1,2)
              and value[3:5] == b'\x00\x00' and value[6:] == b'\x00\x00'
              and 0 <= ended-_number(status.get('received_at_s'),'status time') <= .1,
              'Normal CAN/P,J,L/arrival preparation status required')
        feedback = data.get('feedback',{}); jaw_enabled = None
        for name in ['PiperMsgGripperFeedBack']+['PiperMsgLowSpdFeed_'+str(i) for i in range(1,7)]:
            item = feedback.get(name,{}); flags = item.get('fields',{}).get('foc_status',{})
            jaw = name == 'PiperMsgGripperFeedBack'
            expected = {'voltage_too_low','motor_overheating','driver_overcurrent','driver_overheating',
                        'driver_error_status','driver_enable_status'} | ({'sensor_status','homing_status'}
                        if jaw else {'collision_status','stall_status'})
            _need(set(flags) == expected and all(type(v) is bool for v in flags.values())
                  and all(not v for k,v in flags.items() if k != 'driver_enable_status')
                  and (jaw or flags['driver_enable_status'] is True)
                  and 0 <= ended-_number(item.get('received_at_s'),'health time') <= .1,
                  'Six enabled joint drivers and known healthy jaw flag required')
            if jaw: jaw_enabled = flags['driver_enable_status']
        width = feedback['PiperMsgGripperFeedBack']['fields'].get('grippers_angle')
        _need(type(width) is int and 0 <= width <= 70000, 'Current jaw width required')
        passive[side] = {'source':ref,'started_at':began,'ended_at':ended,'sample_count':len(trace),'spans':spans,
            'rotation_diameter_rad':rotation,'mode_feedback':value[2],'jaw_enabled':jaw_enabled,
            'jaw_width_m':width*1e-6,'jaw_whole_window_observed':False}
    raw, ref = _file(rgb_observation); rgb = json.loads(raw)
    _need(set(rgb.get('cameras',{})) == {'front','left_hand','right_hand'}, 'Actual three-view RGB required')
    pictures = {}
    for view, camera in rgb['cameras'].items():
        at = _number(camera.get('host_received_at'),'RGB time')
        configured = {'front':'front','left_hand':'left_wrist','right_hand':'right_wrist'}[view]
        _need(camera.get('serial') == contract['cameras'][configured] and after < at <= now and now-at <= 30.
              and type(camera.get('frame_number')) is int and camera['frame_number'] > 0
              and camera.get('depth_enabled') is False, 'Fresh post-startup RGB identity/time required')
        _, image_ref = _file(camera['rgb_path'])
        pictures[view] = {**image_ref,'host_received_at':at,'frame_number':camera['frame_number']}
    return {'passive':passive,'rgb':{'source':ref,'images':pictures},'visual_observation':visual_observation,
            'scope':'Archived preparation admission only; live checks required; no stop, cache or motion grant'}


def _task(path, root):
    raw, ref = _file(path); envelope = json.loads(raw); task = envelope.get('task',{})
    _need(envelope.get('schema') == 'piper_supervised_plug_task_v1'
          and set(task) == {'task_id','roles','site_context'} | set(role_fields(task))
          and task['task_id'] == 'plug_transfer_left'
          and task['roles'] == {'left':'task','right':'task'}
          and {'workspace_clearance'} <= set(task['site_context']) <= {'workspace_clearance','feedback_observation'},
          'Exact service-compatible plug task required')
    task_policy(task)
    for statement, limit in ((task['site_context']['workspace_clearance'],2000),(envelope.get('on_site_supervision',{}),8000)):
        _need(type(statement) is dict and statement.get('source') == 'user'
              and type(statement.get('statement')) is str and 1 <= len(statement['statement'].strip()) <= limit,
              'Actual task clearance and on-site supervision statements required')
    recipe_path = root.parent.parent/'tasks/plug_transfer_left.json'
    recipe_raw, recipe_ref = _file(recipe_path)
    _need(envelope.get('recipe_path') == str(recipe_path) and envelope.get('recipe_sha256') == recipe_ref['sha256']
          and json.loads(recipe_raw).get('task_id') == task['task_id'], 'Canonical plug recipe binding required')
    result = {'source':ref,'recipe':recipe_ref,'envelope':envelope}
    rendered_keys = {'rendered_recipe_path', 'rendered_recipe_sha256'}
    if role_fields(task) or rendered_keys.intersection(envelope):
        from .plug_recipe import render_plug_recipe
        _need(rendered_keys <= set(envelope), 'Explicit task roles require their rendered recipe path and hash')
        worker, support = resolve_task_roles(task)
        rendered_raw, rendered_ref = _file(envelope.get('rendered_recipe_path'))
        _need(envelope.get('rendered_recipe_sha256') == rendered_ref['sha256']
              and json.loads(rendered_raw) == render_plug_recipe(json.loads(recipe_raw),
                    worker_arm=worker, support_arm=support), 'Rendered plug recipe must match frozen task roles')
        result['rendered_recipe'] = rendered_ref
    return result


def prepare_task(path, parent_run_id, *, close_log, task_file, new_run_id, started_at,
                 passive_paths, rgb_observation, visual_observation,
                 max_steps=1000, max_duration_s=10800, clock=time.time):
    """Read-only proposal. Failing current evidence never allocates a scope."""
    path = Path(path).resolve(strict=True); root = path.parent.parent
    _need(path == root/'runs/pair_sessions.sqlite', 'Authoritative project database required')
    parent_run_id = _identifier(parent_run_id,'parent run'); new_run_id = _identifier(new_run_id,'new run')
    _need(parent_run_id != new_run_id and type(max_steps) is int and 1 <= max_steps <= 1000,
          'Distinct new run and bounded step allowance required')
    duration = _number(max_duration_s,'duration',positive=True); start = _number(started_at,'start')
    now = _number(clock(),'clock'); boot = boot_identity(); check_processes()
    _need(duration <= 10800 and start <= now < start+duration, 'Fixed current bounded new window required')
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory = sqlite3.Row; db.execute('PRAGMA query_only=ON'); db.execute('BEGIN')
        snapshot = _snapshot(db,parent_run_id,'postsend_rgb_expiry_fault')
        _need(not db.execute('SELECT 1 FROM pair_runs WHERE run_id=?',(new_run_id,)).fetchone(), 'New run already exists')
        startup = _startup(db,path,boot,now)
    old = snapshot['run']; closed = _closed(close_log,snapshot)
    not_before = max(old['started_at']+old['max_duration'], startup['finished_at'])
    _need(not_before <= start, 'Expired parent and completed startup must precede new task authorization/window')
    task = _task(task_file,root); contract = _current_contract(root,snapshot['effective_contract'])
    _need('pair_task_enrollment.py' in contract['code'], 'New manager must be frozen in the host source contract')
    contract['task'] = copy.deepcopy(task['envelope']['task'])
    evidence = _observations(passive_paths,rgb_observation,visual_observation,contract,startup['finished_at'],now)
    proposal = {'schema':SCHEMA,'parent_kind':PARENT_KIND,'database':str(path),'created_at':now,
        'parent_run_id':parent_run_id,'new_run_id':new_run_id,'parent_run':old,
        'snapshot':snapshot,'snapshot_sha256':_sha(snapshot),'close':closed,'startup':startup,'task':task,
        'authorization_not_before':not_before,'new_budget':{'max_steps':max_steps,'max_duration_s':duration,'started_at':start},
        'deadline_s':start+duration,'budget_policy':'explicit_user_new_round',
        'budget_start_policy':'after_repair_before_online_execution',
        'cumulative_step_ceiling':snapshot['cumulative_prior_steps']+max_steps,
        'reviewed_contract':contract,'recovery_evidence':evidence,'required_connection_mode':'prepare',
        'hardware_commands_sent':0,'dispatch_authorized':False,'cache_or_limits_transferred':False,
        'physical_stop_verified':None,'fresh_host_admission_required':True}
    return {**proposal,'proposal_sha256':_sha(proposal)}


def _reprepare(proposal, now):
    evidence = proposal['recovery_evidence']
    return prepare_task(proposal['database'],proposal['parent_run_id'],close_log=proposal['close']['source']['path'],
        task_file=proposal['task']['source']['path'],new_run_id=proposal['new_run_id'],**proposal['new_budget'],
        passive_paths={s:r['source']['path'] for s,r in evidence['passive'].items()},
        rgb_observation=evidence['rgb']['source']['path'],visual_observation=evidence['visual_observation'],clock=lambda:now)


def activate_task(proposal, authorization, *, project_root, clock=time.time):
    """Append one scoped preparation successor after exact new user authorization."""
    _need(proposal.get('schema') == SCHEMA and proposal.get('parent_kind') == PARENT_KIND
          and _sha({k:v for k,v in proposal.items() if k != 'proposal_sha256'}) == proposal.get('proposal_sha256'),
          'Exact canonical supervised-task proposal required')
    _need(_reprepare(proposal,proposal['created_at']) == proposal, 'Canonical unchanged proposal required')
    fields = {'source','message_id','statement','received_at','decision','proposal_sha256','new_budget','budget_start_policy'}
    _need(type(authorization) is dict and set(authorization) == fields
          and authorization['source'] == 'user_message' and authorization['decision'] == 'authorize_explicit_new_round'
          and authorization['proposal_sha256'] == proposal['proposal_sha256']
          and authorization['budget_start_policy'] == proposal['budget_start_policy']
          and _json_object(authorization['new_budget'],'authorized budget') == _json_object(proposal['new_budget'],'budget'),
          'Exact new user-message task/budget authorization required')
    _identifier(authorization['message_id'],'user message reference')
    _need(type(authorization['statement']) is str and 1 <= len(authorization['statement'].strip()) <= 8000,
          'Actual new user statement required')
    authorized = _number(authorization['received_at'],'authorization time')
    _need(proposal['authorization_not_before'] <= authorized <= proposal['new_budget']['started_at'],
          'Authorization must follow this startup and precede the fixed new window')
    root = Path(project_root).resolve(strict=True); path = Path(proposal['database'])
    _need(path == root/'runs/pair_sessions.sqlite', 'Authoritative database required')
    with ExitStack() as locks:
        for candidate in _lock_roots(root):
            if candidate == root or (candidate/'runs').is_dir():
                locks.enter_context(ExclusiveExecution(candidate/'runs'))
        now = _number(clock(),'activation clock')
        _need(proposal['created_at'] <= now < proposal['deadline_s'], 'Activation chronology/deadline changed')
        current = _reprepare(proposal,now)
        current['created_at'] = proposal['created_at']; current.pop('proposal_sha256')
        _need(current == {k:v for k,v in proposal.items() if k != 'proposal_sha256'}, 'Fresh evidence/boot/source changed')
        db = sqlite3.connect(path.as_uri()+'?mode=rw',uri=True,isolation_level=None); db.row_factory = sqlite3.Row
        try:
            db.execute('PRAGMA synchronous=FULL'); db.execute('BEGIN IMMEDIATE')
            snapshot = _snapshot(db,proposal['parent_run_id'],'postsend_rgb_expiry_fault')
            _need(_sha(snapshot) == proposal['snapshot_sha256'], 'Historical ledger changed before activation')
            _need(_startup(db,path,boot_identity(),now) == proposal['startup'], 'Startup/boot evidence changed')
            check_processes()
            budget = proposal['new_budget']
            record = {'proposal':proposal,'authorization':authorization,'activated_at':now,
                'new_contract':proposal['reviewed_contract'],'old_rows_preserved':True,'new_budget_allocated':True,
                'hardware_commands_sent':0,'dispatch_authorized':False,'physical_stop_verified':None,
                'cache_or_limits_transferred':False,'fresh_host_admission_required':True,'required_connection_mode':'prepare'}
            ordinal = db.execute('SELECT MAX(ordinal)+1 FROM pair_rounds').fetchone()[0]
            db.execute('INSERT INTO pair_runs VALUES(?,?,?,?,?,0)',(proposal['new_run_id'],
                _json_object(record['new_contract'],'new contract'),budget['max_steps'],budget['max_duration_s'],budget['started_at']))
            db.execute('INSERT INTO pair_rounds VALUES(?,?,?,NULL,NULL,NULL,?,?,?,?,?)',(ordinal,proposal['new_run_id'],
                proposal['parent_run_id'],now,json.dumps(snapshot['retired_owners']),proposal['proposal_sha256'],
                _sha(authorization),_json_object(record,'new supervised task')))
            # Only this newly appended row may receive its final commit time.
            final = _number(clock(),'commit clock')
            _need(now <= final < proposal['deadline_s'] and boot_identity() == proposal['startup']['boot'],
                  'Boot/clock/deadline changed before commit')
            contract = _current_contract(root,snapshot['effective_contract']); contract['task'] = record['new_contract']['task']
            _need(contract == record['new_contract'] and _task(proposal['task']['source']['path'],root) == proposal['task'],
                  'Code/task changed during enrollment')
            _need(_closed(proposal['close']['source']['path'],snapshot) == proposal['close'],
                  'Close evidence changed during enrollment')
            evidence = proposal['recovery_evidence']
            _need(_observations({s:r['source']['path'] for s,r in evidence['passive'].items()},
                evidence['rgb']['source']['path'],evidence['visual_observation'],record['new_contract'],
                proposal['startup']['finished_at'],final) == evidence, 'Preparation evidence changed or expired before commit')
            for kind in ('request','result','journal'):
                ref = proposal['startup'][kind]
                _need(_file(ref['path'])[1] == ref, 'Startup source changed before commit')
            check_processes(); record['activated_at'] = final
            db.execute('UPDATE pair_rounds SET last_time=?,record_json=? WHERE ordinal=?',
                       (final,_json_object(record,'new supervised task'),ordinal))
            db.execute('COMMIT'); return record
        except BaseException:
            if db.in_transaction: db.execute('ROLLBACK')
            raise
        finally:
            db.close()
