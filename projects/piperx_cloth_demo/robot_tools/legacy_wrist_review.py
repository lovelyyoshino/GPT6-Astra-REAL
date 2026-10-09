"""Audit a known old-SDK J5 mismatch against later non-actuating readback.

No fault deletion, limit write, retransmission or automatic task renewal.
Only the exact fully sent action and six-query left-only receipt are covered.
"""
import ast
import hashlib
import json
import math
from pathlib import Path

KEY = 'reviewed_wrist_limit_failure'
FIELDS = {'run_id', 'sha256', 'limits_run_id', 'limits_sha256', 'source', 'statement'}


def need(value, message):
    if not value:
        raise RuntimeError('Legacy wrist review refused: ' + message)


def validate_reference(value):
    need(isinstance(value,dict) and set(value)==FIELDS
         and value.get('source')=='user'
         and isinstance(value.get('statement'),str) and value['statement'].strip(),
         'explicit bounded review reference required')
    for key,prefix in [('run_id','single_supervised_move_'),('limits_run_id','joint_limits_')]:
        need(isinstance(value[key],str) and value[key].startswith(prefix)
             and all(c.isalnum() or c=='_' for c in value[key]), 'invalid receipt identity')
    for key in ('sha256','limits_sha256'):
        need(isinstance(value[key],str) and len(value[key])==64
             and all(c in '0123456789abcdef' for c in value[key]), 'invalid receipt digest')


def read(runs, run_id, digest):
    path=Path(runs)/run_id/'result.json';raw=path.read_bytes();result=json.loads(raw)
    need(hashlib.sha256(raw).hexdigest()==digest and result.get('run_id')==run_id,
         'receipt identity or hash differs')
    return result,path


def review(runs, action, compatibility):
    from .legacy_controller import same_authorization
    value=compatibility.get(KEY);validate_reference(value)
    need(action['run_id']==value['run_id'] and action['status']=='failed'
         and action['result_sha256']==value['sha256'], 'wrong failed action')
    failed,fp=read(runs,value['run_id'],value['sha256'])
    limits,lp=read(runs,value['limits_run_id'],value['limits_sha256'])
    need(Path(action['record_path']).resolve()==fp.resolve(), 'failed action path differs')
    need(failed.get('ok') is False and failed.get('status')=='aborted_after_dispatch'
         and failed.get('operation')=='single_supervised_move'
         and failed.get('selected_arm')=='left'
         and same_authorization(failed.get('legacy_controller_compatibility'),compatibility),
         'failure is not this original left-arm sequence')
    errors=failed.get('errors',[]);prefix='Selected joint feedback exceeds action-specific boundary allowance: '
    need(len(errors)==1 and errors[0].get('type')=='RuntimeError'
         and errors[0].get('detail','').startswith(prefix), 'failure has another cause')
    try:
        violations=ast.literal_eval(errors[0]['detail'][len(prefix):])
    except (ValueError,SyntaxError):
        raise RuntimeError('Legacy wrist review refused: malformed old-limit violation')
    need(isinstance(violations,list) and len(violations)==1, 'one J5 mismatch only')
    v=violations[0]
    need(set(v)=={'joint_index','observed_rad','minimum_rad','maximum_rad'}
         and v['joint_index']==5 and v['minimum_rad']==-1.22173 and v['maximum_rad']==1.22173
         and 1.22173 < v['observed_rad'] < math.radians(89), 'not old 70-degree J5 mismatch')
    counts=lambda n:{s:dict(attempted_frames=n if s=='left' else 0,
                           sent_frames=n if s=='left' else 0,blocked_frames=0) for s in ('left','right')}
    need(failed.get('transmission_counts')==counts(4)
         and failed.get('guard_violations')==[]
         and all(failed.get(k)==n for k,n in dict(hardware_commands_sent=4,
             target_calls_sent=1,target_commands_sent=3,mode_commands_sent=1,
             arm_target_commands_sent=3,gripper_target_commands_sent=0,
             enable_commands_sent=0,stop_commands_sent=0,retries=0).items()),
         'unknown, partial, extra or non-MOVE_L send')
    events=[json.loads(line) for line in fp.with_name('events.jsonl').read_text().splitlines()]
    sent=[e for e in events if e['event']=='single_supervised_action_sent_unconfirmed']
    need(len(sent)==1, 'exact completed send journal required')
    req=json.loads(lp.with_name('request.json').read_text())
    oldreq=json.loads(fp.with_name('request.json').read_text())
    need(limits.get('ok') is True and limits.get('operation')=='inspect_joint_limits'
         and limits.get('selected_arm')=='left' and limits.get('query_sides')==['left']
         and limits.get('errors')==limits.get('guard_violations')==[]
         and limits.get('transmission_counts')==counts(6)
         and limits.get('joint_limit_queries_sent')==6
         and limits.get('controller_limits_changed') is False
         and limits.get('sdk_joint_limits_changed') is False
         and all(limits.get(k)==0 for k in ('actuator_commands_sent','mode_commands_sent',
             'target_commands_sent','enable_commands_sent','stop_commands_sent','retries'))
         and req.get('arms')==oldreq.get('arms')
         and req.get('arguments')=={'arm':'left'}
         and req['started_unix_s']>failed['after']['left']['timestamp'],
         'later left-only matching readback required')
    need(set(limits['joint_limits']['left'])==set(map(str,range(1,7)))
         and limits['joint_limits']['right']=={}, 'six left joints and no peer query required')
    for joint,row in limits['joint_limits']['left'].items():
        raw=bytes.fromhex(row['raw_response_hex'])
        need(row.get('status')=='confirmed' and len(raw)==8 and raw[0]==int(joint)
             and raw[7]==0
             and int.from_bytes(raw[3:5],'big',signed=True)==row['raw_min_angle_tenth_deg']
             and int.from_bytes(raw[1:3],'big',signed=True)==row['raw_max_angle_tenth_deg'],
             'raw limit evidence differs')
        if joint in ('4','5'):
            need([row['raw_min_angle_tenth_deg'],row['raw_max_angle_tenth_deg']]==[-890,890],
                 'controller wrist is not +/-89 degrees')
    for receipt in (failed,limits):
        need(all(receipt.get('cleanup',{}).get('arms',{}).get(s,{}).get('status')=='disconnected'
                 for s in ('left','right')), 'unfinished controller cleanup')
    state=limits['after']['left'];target=failed['requested_target']
    angle=sum(abs(math.remainder(a-b,2*math.pi)) for a,b in zip(state['pose_m_rad'][3:],target[3:]))
    drift=limits['drift']['left']
    need(math.dist(state['pose_m_rad'][:3],target[:3])<=.0005 and angle<=.003
         and drift['joint_rad']<=.003 and drift['position_m']<=.0005 and drift['gripper_m']<=.0005
         and state['arm_status']['motion_status']==0,
         'later independent stable endpoint is not the original target')
    need(abs(state['gripper']['width_m']-failed['before']['left']['gripper']['width_m'])<=.0005,
         'gripper changed after failed action')
    return dict(original_failure_run_id=value['run_id'],limits_run_id=value['limits_run_id'],
                limits_sha256=value['limits_sha256'],joint5_limits_tenth_deg=[-890,890],
                original_failed_row_preserved=True,original_target_not_replayed=True,
                task_budget_reset=False,controller_limits_written=False)
