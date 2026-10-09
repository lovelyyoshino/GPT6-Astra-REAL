"""Audited new preparation round after a confirmed configuration-only failure.

This module is offline. It preserves the old event/fault and demands a later
independent full limit query resolving the one uncertain setting. No old
target, controller-limit cache, hold or motion permission is transferred.
"""
import json
from pathlib import Path

from .pair_restart import _file, _need, _sha

KIND = 'configuration_maintenance_fault'


def completed_prefix(events, run, following):
    """Recognize one completed configuration owner before a fresh task owner.

    These limits are historical only. The task owner's subsequent independent
    twelve-joint capture remains mandatory and supplies its own live limits.
    """
    from .pair_round import _counts
    from .controller_limits import payload
    _need(len(events) == 1, 'Only one completed maintenance prefix is covered')
    e = events[0]; p = json.loads(e['payload_json']); r = json.loads(e['receipt_json'])
    _need(e['step'] == 1 and e['success'] == 1 and e['status'] == 'complete'
          and e['owner'] != following['owner'] and r.get('owner') == e['owner']
          and r.get('event_id') == e['event_id'] and r.get('run_id') == run['run_id']
          and hashlib_digest(e['payload_json']) == e['payload_digest']
          and p.get('schema') == r.get('schema') == 'piper_x_wrist_limit_maintenance_v1'
          and p.get('kind') == 'manufacturer_configuration'
          and r.get('ok') is True and r.get('status') == 'manufacturer_wrist_limits_readback_verified'
          and r.get('errors') == r.get('guard_violations') == []
          and all(type(r.get(k)) is int and r[k] == 0 for k in ('arm_target_commands_sent','gripper_commands_sent'))
          and r.get('speed_changes_requested') is r.get('zero_changes_requested') is False
          and r.get('task_motion_authorized') is False
          and e['began_at'] <= r['began_at'] <= r['ended_at'] <= e['finished_at'] < following['began_at'],
          'Complete non-actuating manufacturer maintenance prefix required')
    writes = r.get('writes'); seen = set(); counts = {s:dict(attempted_frames=12,sent_frames=12,blocked_frames=0) for s in ('left','right')}
    _need(type(writes) is list and len(writes) <= 4, 'Bounded wrist writes only')
    for w in writes:
        key = (w.get('side'),w.get('joint'))
        _need(key[0] in counts and key[1] in (4,5) and key not in seen
              and w.get('outcome') == 'returned' and w.get('arbitration_id') == 0x474
              and w.get('data_hex') == payload(key[1],[-890,890]).hex()
              and w.get('speed_field') == 'unchanged_0x7fff'
              and w.get('requested_limits_tenth_deg') == [-890,890]
              and r['began_at'] <= w['attempted_at'] <= w['returned_at'] < r['ended_at'],
              'One returned manufacturer frame per distinct wrist required')
        seen.add(key)
        for name in ('attempted_frames','sent_frames'): counts[key[0]][name] += 2
        _need(w.get('readback',{}).get('status') == 'confirmed', 'Every write needs its actual readback')
    _need(_counts(r.get('transmission_counts')) == _counts(r.get('session_transmission_counts')) == counts,
          'Exact query/write lifetime accounting required')
    for side in counts:
        for j in range(1,7):
            b = r['before_limits'][side][str(j)]; a = r['joint_limits'][side][str(j)]
            expected = [-890,890] if j in (4,5) else [b['raw_min_angle_tenth_deg'],b['raw_max_angle_tenth_deg']]
            raw = bytes.fromhex(a['raw_response_hex'])
            _need(a.get('status') == 'confirmed' and len(raw) == 8 and raw[0] == j and raw[7] == 0
                  and [int.from_bytes(raw[3:5],'big',signed=True),int.from_bytes(raw[1:3],'big',signed=True)] == expected
                  and [a['raw_min_angle_tenth_deg'],a['raw_max_angle_tenth_deg']] == expected
                  and int.from_bytes(raw[5:7],'big') == a['raw_max_joint_spd'] == b['raw_max_joint_spd'],
                  'Final manufacturer limits and unchanged speeds required')
    c = r.get('cleanup',{})
    _need(c.get('requires_fault_latch') is False and c.get('unresolved_gripper_probe') is None
          and c.get('grasp_states') == {'left':None,'right':None}
          and c.get('guard_violations') == [] and _counts(c.get('session_transmission_counts')) == counts
          and all(c.get('arms',{}).get(s,{}).get('status') == 'disconnected' for s in counts),
          'Completed maintenance cleanup required')
    return {'event_id':e['event_id'],'event_sha256':_sha(e),'configuration_only':True,
            'historical_limits_not_transferred':True}


def hashlib_digest(text):
    import hashlib
    return hashlib.sha256(text.encode()).hexdigest()


def failure(event, run, scope, faults, totals):
    from .pair_round import _counts
    p, r = json.loads(event['payload_json']), json.loads(event['receipt_json'])
    _need(event['success'] == 0 and event['status'] == 'complete'
          and event['owner'] == scope['owner'] and event['finished_at'] is not None
          and p.get('kind') == 'manufacturer_configuration'
          and p.get('schema') == r.get('schema') == 'piper_x_wrist_limit_maintenance_v1'
          and r.get('ok') is False and r.get('status') == 'maintenance_failed_no_retry'
          and r.get('event_id') == event['event_id'] and r.get('owner') == event['owner']
          and r.get('run_id') == run['run_id'] and _sha(p) == event['payload_digest'],
          'Exact completed configuration-only failure required')
    _need(all(v == 0 for row in totals.values() for v in row.values()),
          'Maintenance owner may have only this one claimed operation')
    _need(all(type(r.get(k)) is int and r[k] == 0 for k in ('arm_target_commands_sent','gripper_commands_sent'))
          and r.get('guard_violations') == []
          and r.get('speed_changes_requested') is False and r.get('zero_changes_requested') is False
          and r.get('task_motion_authorized') is False,
          'Motion, zero, speed changes or uncertain transport are outside this recovery')
    _need(r.get('errors') == [{'type':'RuntimeError','detail':'Cross-arm feedback skew exceeds limit'}],
          'Only the diagnosed post-configuration feedback interruption is covered')
    writes = r.get('writes')
    _need(type(writes) is list and len(writes) == 1, 'Exactly one original configuration write required')
    w = writes[0]
    from .controller_limits import payload
    _need(w.get('side') in ('left','right') and w.get('joint') in (4,5)
          and w.get('arbitration_id') == 0x474 and w.get('outcome') == 'returned'
          and w.get('data_hex') == payload(w['joint'],[-890,890]).hex()
          and w.get('requested_limits_tenth_deg') == [-890,890]
          and w.get('speed_field') == 'unchanged_0x7fff'
          and event['began_at'] <= r['began_at'] <= w['attempted_at'] <= w['returned_at']
              <= r['ended_at'] <= event['finished_at'], 'Original manufacturer frame and timing required')
    expected = {s: dict(attempted_frames=8 if s == w['side'] else 6,
                       sent_frames=8 if s == w['side'] else 6, blocked_frames=0) for s in ('left','right')}
    _need(_counts(r.get('transmission_counts')) == expected
          and _counts(r.get('session_transmission_counts')) == expected,
          'Only twelve initial queries, one write and one unfinished readback are covered')
    query = r.get('joint_limits',{}).get(w['side'],{}).get(str(w['joint']),{})
    _need(query.get('status') == 'unconfirmed' and query.get('raw_response_hex') == ''
          and query.get('response_evidence',{}).get('response_frames') == [],
          'Preserve the missing original configuration readback')
    f = [f for f in faults if f['run_id'] == run['run_id']]
    _need(len(f) == 1 and f[0]['id'] == scope['fault_id'] and f[0]['owner'] == event['owner']
          and f[0]['reason'] == 'execution_receipt_failed' and f[0]['at'] >= event['finished_at'],
          'Original maintenance fault must remain the current sole run fault')
    return expected


def closed(path, snapshot):
    from .pair_round import _counts
    raw, ref = _file(path); receipt = json.loads(raw)
    _need(receipt == snapshot['configuration_failure']['receipt'], 'Original saved maintenance result required')
    c = receipt.get('cleanup',{})
    _need(c.get('requires_fault_latch') is False and c.get('unresolved_gripper_probe') is None
          and c.get('grasp_states') == {'left':None,'right':None}
          and all(c.get('arms',{}).get(s,{}).get('status') == 'disconnected' for s in ('left','right'))
          and _counts(c.get('session_transmission_counts')) == snapshot['session_transmission_counts'],
          'Original no-grasp resource closure and exact counters required')
    return {'source':ref,'receipt':receipt,'retired_owner':snapshot['retired_owner'],
            'exit_observed_at':snapshot['last_finished_at'],'physical_stop_verified':None}


def observations(evidence, snapshot, contract, now):
    from .pair_task_enrollment import _observations
    _need(type(evidence) is dict and set(evidence) ==
          {'passive_paths','rgb_observation','visual_observation','configuration_diagnostic'},
          'Fresh passive/RGB evidence and resolved manufacturer-setting diagnostic required')
    result = _observations(**{k:v for k,v in evidence.items() if k != 'configuration_diagnostic'},
                           contract=contract, after=snapshot['last_finished_at'], now=now)
    raw, ref = _file(evidence['configuration_diagnostic']); capture = json.loads(raw)
    failed = snapshot['configuration_failure']; old = failed['receipt']; w = old['writes'][0]
    _need(capture.get('ok') is True and capture.get('schema') == 'piper_pair_controller_limits_capture_v1'
          and capture.get('source_event_id') == failed['event_id']
          and capture.get('source_fault') == failed['fault']
          and capture.get('fault_cleared') is False and capture.get('task_dispatch_authorized') is False
          and capture.get('canonical_ledger_unchanged') is True
          and capture.get('guard_violations') == capture.get('errors') == []
          and capture.get('joint_limit_queries_sent') == capture.get('hardware_commands_sent') == 12
          and capture.get('actuator_commands_sent') == 0
          and snapshot['last_finished_at'] < capture['began_at'] < capture['ended_at'] <= now,
          'Later complete independent non-actuating diagnostic required')
    for side in ('left','right'):
        _need(set(capture['joint_limits'][side]) == set(str(i) for i in range(1,7)), 'Complete diagnostic joints required')
        for j in range(1,7):
            before = old['before_limits'][side][str(j)]
            current = capture['joint_limits'][side][str(j)]
            expected = ([-890,890] if (side,j) == (w['side'],w['joint']) else
                        [before['raw_min_angle_tenth_deg'],before['raw_max_angle_tenth_deg']])
            rx = current['response_evidence']; frames = rx['response_frames']
            _need(current.get('status') == 'confirmed' and len(frames) == 1
                  and rx['rejected_frames'] == [] and frames[0]['valid_can_data_frame'] is True
                  and current['raw_response_hex'] == frames[0]['payload_hex']
                  and capture['began_at'] <= rx['request_started_unix_s'] <= frames[0]['timestamp']
                      <= frames[0]['received_unix_s'] <= rx['finished_unix_s'] <= capture['ended_at'],
                  'Raw fresh correlated limit reply required')
            data = bytes.fromhex(current['raw_response_hex'])
            _need(len(data) == 8 and data[0] == j and data[7] == 0
                  and [int.from_bytes(data[3:5],'big',signed=True),int.from_bytes(data[1:3],'big',signed=True)] == expected
                  and int.from_bytes(data[5:7],'big') == before['raw_max_joint_spd']
                  and [current['raw_min_angle_tenth_deg'],current['raw_max_angle_tenth_deg']] == expected
                  and current['raw_max_joint_spd'] == before['raw_max_joint_spd'],
                  'Written limit must be confirmed, all other limits/speeds preserved; no automatic retry')
    # Model pose bounds qualify preparation only. Runtime must query limits
    # again and complete the not-yet-attempted settings before any motion.
    from .model_compatibility import load_model_catalog, KNOWN_OFFICIAL_CONSTANTS
    from .joint_path import SDK_COMMIT
    source = {'commit':SDK_COMMIT, 'sha256':KNOWN_OFFICIAL_CONSTANTS[SDK_COMMIT],
              'source_url':'https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/'+SDK_COMMIT+'/pyAgxArm/api/constants.py',
              'constants_path':str(Path(__file__).parents[1]/'data/piper_x_official/sdk_constants.py')}
    models,_ = load_model_catalog(source)
    for side,path in evidence['passive_paths'].items():
        raw,_ = _file(path); data = json.loads(raw)
        _need(contract['arms'][side]['model'] == 'piper_x','Confirmed PiPER X required')
        limits = models['piper_x']['joint_limits_rad']
        import math
        for sample in data['pose_trace']:
            for i in range(1,7):
                q = sample['joints_raw']['joint_'+str(i)]*math.pi/180000
                lo,hi = limits[i-1]
                _need(lo <= q <= hi, 'Current pose outside manufacturer model limits')
    result.update(configuration_diagnostic={'source':ref,'event_id':failed['event_id'],
                  'resolved_write':{'arm':w['side'],'joint':w['joint'],'limits_tenth_deg':[-890,890]},
                  'old_failed_result_preserved':True},
                  controller_limits_pending_live_query=True, preparation_only=True,
                  new_target_replay_authorized=False, physical_stop_verified=None)
    return result
