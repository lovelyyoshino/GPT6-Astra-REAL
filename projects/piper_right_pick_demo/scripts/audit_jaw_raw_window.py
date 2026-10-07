#!/usr/bin/env python3
"""Offline audit of an existing wide70 jaw result and raw JSONL byte window.

No ROS, CAN socket, SDK, process control, or writes to source evidence. This
checks recorded feedback, not physical clearance, grasp, or general stopping.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

IDS=set(range(673,681))|set(range(609,615))
BOUNDS=((-150000,150000),(0,180000),(-170000,0),(-100000,100000),(-70000,70000),(-120000,120000))
RAD=math.pi/180000


def require(value,message):
    if not value:raise RuntimeError(message)


def quaternion(pose):
    r,p,y=[v/2 for v in pose[3:]]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)


def window(path,begin,end):
    """Seek near the requested time, then preserve every contiguous full line."""
    size=path.stat().st_size
    with path.open('rb')as stream:
        offset=max(0,size-4000000)
        while True:
            stream.seek(offset)
            if offset:stream.readline()
            scan_start=stream.tell();stamp=None
            for _ in range(100):
                line=stream.readline()
                if not line.endswith(b'\n'):break
                row=json.loads(line)
                if row['event']=='frame':stamp=row['timestamp'];break
            if offset==0 or(stamp is not None and stamp<=begin):break
            offset=max(0,offset-4000000)
        stream.seek(scan_start);rows=[];chunks=[];first=last=None
        while stream.tell()<size:
            position=stream.tell();line=stream.readline()
            if not line.endswith(b'\n'):break
            row=json.loads(line);stamp=row.get('timestamp',row.get('observed_unix_s',0))
            if stamp<begin:continue
            if stamp>end:break
            if first is None:first=position
            last=stream.tell();rows.append(row);chunks.append(line)
    require(first is not None,'No complete raw window')
    content=b''.join(chunks)
    require(len(content)==last-first,'Raw byte window is not contiguous')
    return rows,dict(path=str(path.resolve()),first_byte=first,last_byte_exclusive=last,
        sha256=hashlib.sha256(content).hexdigest(),read_file_size=size)


def audit(result_path,state_path,raw_path,tail_seconds=.05):
    result_bytes=result_path.read_bytes();result=json.loads(result_bytes)
    state_bytes=state_path.read_bytes();state=json.loads(state_bytes)
    require(state['sequence']==result['sequence'],'State belongs to a different action; use its saved snapshot')
    require(result['kind']=='gripper'and state['kind']=='gripper','Only jaw actions are supported')
    require(len(state['receipts'])==1,'Requires one jaw transaction, no retry')
    receipt=state['receipts'][0];before=state['before'];width=result['jaw_target_m']
    expected=[dict(id=345,data_hex=struct.pack('>iHBB',round(width*1e6),200,1,0).hex())]
    require(0<=width<=.07 and receipt['kind']=='gripper'and receipt['jaw_target_m']==width
        and receipt['attempted_frames']==receipt['socket_send_returns']==1
        and receipt['frames']==expected,'Incomplete or different jaw receipt')
    after=result.get('after')or result.get('latest_state')
    require(after is not None,'No recorded terminal feedback')
    require(.05<=tail_seconds<=30,'Tail must be0.05..30seconds')
    start=receipt['started_unix_s'];end=max(after['stamps'])
    rows,source=window(raw_path,start-.2,end+tail_seconds)
    nominal=[];tracking=[];jaw=[];health=[];transport=[];controls=[]
    last={};cache={};counts={};matched=set();maxgap=maxage=0.;timestamps=[]
    qranges=[[v,v]for v in before['raw_q']];qmax=[0]*6;jrange=[math.inf,-math.inf]
    torques=[];xyzmax=rotmax=0.;poses=0;statuses=set();motors={};qb=quaternion(before['pose'])
    tail_q=[[]for _ in range(6)];tail_jaw=[];tail_torque=[];tail_stamps=[]
    for row in rows:
        if row['event']!='frame':
            if row['event']not in('fresh_ready',):transport.append(row)
            continue
        ident=row['id'];stamp=row['timestamp'];data=bytes.fromhex(row['data_hex'])
        tag=dict(timestamp=stamp,id=ident,data_hex=row['data_hex'])
        age=row['host_received_at']-stamp;maxage=max(maxage,age)
        if(len(data)!=8 or row.get('timestamp_basis')!='kernel_socket_SO_TIMESTAMPNS_unix'
            or not row.get('kernel_timestamp_ns')or row.get('msg_flags',0)
            or not 0<=age<=.1 or row.get('socket_dropped_total',0)):
            transport.append(dict(tag,reason='transport_metadata'))
        if len(data)!=8:continue
        if ident not in IDS:
            controls.append(tag)
            if stamp>=start and dict(id=ident,data_hex=row['data_hex'])not in expected:
                health.append(dict(tag,reason='unexpected_control_frame'))
            continue
        timestamps.append(stamp);counts[ident]=counts.get(ident,0)+1
        if ident in last:
            gap=stamp-last[ident];maxgap=max(maxgap,gap)
            if gap<0 or gap>.1:transport.append(dict(tag,reason='per_id_gap',gap_s=gap))
        last[ident]=stamp;cache[ident]=row
        expected_after=after['raw_feedback'].get(hex(ident))
        if(expected_after and expected_after['kernel_unix_s']==stamp
            and expected_after['data_hex']==row['data_hex']):matched.add(ident)
        if 677<=ident<=679:
            for pair,raw in enumerate(struct.unpack('>ii',data)):
                axis=(ident-677)*2+pair
                if stamp>=end+3:tail_q[axis].append(raw);tail_stamps.append(stamp)
                if not BOUNDS[axis][0]<=raw<=BOUNDS[axis][1]:nominal.append(dict(tag,axis=axis+1,raw=raw))
                if stamp>=start:
                    qranges[axis]=[min(qranges[axis][0],raw),max(qranges[axis][1],raw)]
                    qmax[axis]=max(qmax[axis],abs(raw-before['raw_q'][axis]))
                    if abs(raw-before['raw_q'][axis])*RAD>.003:
                        tracking.append(dict(tag,axis=axis+1,raw=raw,origin_raw=before['raw_q'][axis],margin_rad=.003))
        elif ident==673:
            statuses.add(row['data_hex'])
            if data[:5]!=bytes([1,0,1,0,0])or int.from_bytes(data[6:8],'big'):
                health.append(dict(tag,reason='arm_status'))
        elif 609<=ident<=614:
            motors.setdefault(ident,set()).add(data[5])
            if data[5]!=64:health.append(dict(tag,reason='motor'))
        elif ident==680:
            opening=struct.unpack('>i',data[:4])[0]
            if not 0<=opening<=70000 or data[6]!=64:
                jaw.append(dict(tag,reason='jaw_range_or_code',width_raw=opening))
            if stamp>=start:
                jrange=[min(jrange[0],opening),max(jrange[1],opening)]
                torques.append(struct.unpack('>h',data[4:6])[0])
            if stamp>=end+3:
                tail_jaw.append(opening);tail_torque.append(struct.unpack('>h',data[4:6])[0])
        if stamp>=start and ident in(674,675,676)and all(i in cache for i in(674,675,676)):
            rawpose=[];pose_stamps=[]
            for i in(674,675,676):
                rawpose.extend(struct.unpack('>ii',bytes.fromhex(cache[i]['data_hex'])))
                pose_stamps.append(cache[i]['timestamp'])
            if max(pose_stamps)-min(pose_stamps)>.1:transport.append(dict(tag,reason='pose_skew'))
            pose=[v/1e6 for v in rawpose[:3]]+[v*RAD for v in rawpose[3:]]
            xyz=math.sqrt(sum((a-b)**2 for a,b in zip(pose[:3],before['pose'][:3])))
            q=quaternion(pose);rot=2*math.acos(min(1,abs(sum(a*b for a,b in zip(q,qb)))))
            xyzmax=max(xyzmax,xyz);rotmax=max(rotmax,rot);poses+=1
            if xyz>.002 or rot>.003:jaw.append(dict(tag,reason='arm_pose_drift',translation_m=xyz,rotation_rad=rot))
    require(set(counts)==IDS and matched==IDS,'All14 final raw feedback frames must match source timestamps and bytes')
    require(min(timestamps)<start and max(timestamps)>=end+tail_seconds-.03,'Window does not bracket full action/tail')
    return dict(schema_version=1,sequence=result['sequence'],generation=state['generation'],
        adoption_token=state['adoption_token'],completed_result_sha256=hashlib.sha256(result_bytes).hexdigest(),
        state_snapshot_sha256=hashlib.sha256(state_bytes).hexdigest(),full_raw_reviewed=True,
        transport_clean=not transport,first_kernel_unix_s=min(timestamps),last_kernel_unix_s=max(timestamps),
        source_windows=[source],nominal_violations=nominal,joint_tracking_violations=tracking,
        jaw_guard_violations=jaw,health_violations=health,transport_violations=transport,
        feedback_frame_count=sum(counts.values()),feedback_counts={hex(k):v for k,v in sorted(counts.items())},
        control_frames=controls,control_absence_is_not_zero_tx_proof=True,
        command_evidence_source='frozen_driver_exact_frame_socket_send_receipt_not_bus_delivery',receipt=receipt,
        arm_original_raw_q=before['raw_q'],post_command_joint_ranges_raw=qranges,max_joint_deviation_mdeg=qmax,
        max_observed_translation_m=xyzmax,max_observed_rotation_rad=rotmax,
        pose_fragment_reconstruction_count=poses,
        pose_reconstruction_scope='Actual asynchronous XYZ/RPY fragments; not independent physical measurements',
        post_command_jaw_range_raw=jrange,torque_sdk_range=[min(torques),max(torques)],
        final_opening_m=after['opening_m'],target_opening_m=width,
        terminal_width_error_m=after['opening_m']-width,arrival_report_tolerance_m=.0015,
        result_phase=result['phase'],action_failure=result.get('failure'),stable_window=result.get('stable_window'),
        later_tail=dict(requested_seconds_after_terminal=tail_seconds,
            first_kernel_unix_s=min(tail_stamps)if tail_stamps else None,
            last_kernel_unix_s=max(tail_stamps)if tail_stamps else None,
            joint_ranges_raw=[[min(v),max(v)]if v else None for v in tail_q],
            jaw_range_raw=[min(tail_jaw),max(tail_jaw)]if tail_jaw else None,
            jaw_torque_sdk_range=[min(tail_torque),max(tail_torque)]if tail_torque else None),
        all_14_result_feedback_frames_matched_exactly=True,status_payloads=sorted(statuses),
        motor_codes={hex(k):sorted(v)for k,v in motors.items()},max_per_id_gap_s=maxgap,max_host_frame_age_s=maxage,
        actual_can_fit_verified=False,contact_absent_verified=False,prior_failures_preserved=True,
        scope='Complete-line raw byte window covering jaw action plus0.2s before/requested tail; not whole-recording qualification',
        review_pass=result['phase']=='completed'and not any((nominal,tracking,jaw,health,transport)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in('result','state','raw','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--tail-seconds',type=float,default=.05)
    args=parser.parse_args();report=audit(args.result,args.state,args.raw,args.tail_seconds)
    with args.output.open('x')as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps(dict(path=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        review_pass=report['review_pass'],frames=report['feedback_frame_count'],
        max_joint_deviation_mdeg=report['max_joint_deviation_mdeg'],final_opening_m=report['final_opening_m'])))
    return 0 if report['review_pass']else 1


if __name__=='__main__':raise SystemExit(main())
