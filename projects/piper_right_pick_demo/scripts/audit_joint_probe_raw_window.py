#!/usr/bin/env python3
"""Offline raw audit ONLY for the reviewed separation and small wrist probes.

No robot/camera access. Pure manufacturer FK is geometry, not object localization
or clearance proof. An audit never changes the driver's persistent state.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

from audit_jaw_raw_window import window,quaternion,require,IDS,BOUNDS,RAD

FK_SHA='354d9b1f41271645de127c4c59ae30c86b0debddeba7a5c0ca0b1b2d44f9b93d'
STAGES=('retreat_ready','wrist_ready')


def outside_joint_box(raw,axis0,origin_raw,target_raw):
    excess=max(min(origin_raw,target_raw)-raw,raw-max(origin_raw,target_raw),0)
    return excess>300 if axis0==4 else excess*RAD>.003


def rotation(a,b):
    qa,qb=quaternion(a),quaternion(b)
    return 2*math.acos(min(1,abs(sum(x*y for x,y in zip(qa,qb)))))


def check_rows(rows,before,target_raw,after,limits,fk,start):
    nominal=[];tracking=[];jaw=[];health=[];transport=[];controls=[]
    last={};cache={};counts={};matched=set();stamps=[];maxgap=maxage=0.
    ranges=[[v,v]for v in before['raw_q']];excesses=[0]*6;jrange=[math.inf,-math.inf]
    xyzmax=rotmax=fkxyzmax=fkrotmax=0.;seen_fk={};status_payloads=set();motor_codes={}
    expected_controls={
        (0x151,bytes([1,1,1,0,0,0,0,0]).hex()),
        *((0x155+i,struct.pack('>ii',*target_raw[2*i:2*i+2]).hex())for i in range(3))}

    def geometry(pose,tag,source):
        nonlocal xyzmax,rotmax,fkxyzmax,fkrotmax
        xyz=math.dist(pose[:3],before['pose'][:3]);rot=rotation(pose,before['pose'])
        if source=='feedback':xyzmax=max(xyzmax,xyz);rotmax=max(rotmax,rot)
        else:fkxyzmax=max(fkxyzmax,xyz);fkrotmax=max(fkrotmax,rot)
        if(not all(math.isfinite(v)for v in pose)or xyz>limits['max_translation_step_m']
           or rot>limits['max_rotation_step_rad']or not all(lo<=v<=hi for v,lo,hi in
               zip(pose[:3],limits['workspace_min_m'],limits['workspace_max_m']))):
            health.append(dict(tag,reason=source+'_software_geometry_envelope',translation_m=xyz,rotation_rad=rot))

    for row in rows:
        if row['event']!='frame':
            if row['event']!='fresh_ready':transport.append(row)
            continue
        ident=row['id'];stamp=row['timestamp'];data=bytes.fromhex(row['data_hex'])
        tag=dict(timestamp=stamp,id=ident,data_hex=row['data_hex'])
        age=row['host_received_at']-stamp;maxage=max(maxage,age)
        if(len(data)!=8 or row.get('timestamp_basis')!='kernel_socket_SO_TIMESTAMPNS_unix'
            or not row.get('kernel_timestamp_ns')or row.get('msg_flags',0)
            or not 0<=age<=limits['max_state_age_s']or row.get('socket_dropped_total',0)):
            transport.append(dict(tag,reason='transport_metadata'))
        if len(data)!=8:continue
        if ident not in IDS:
            controls.append(tag)
            if stamp>=start and(ident,row['data_hex'])not in expected_controls:
                health.append(dict(tag,reason='unexpected_control_frame'))
            continue
        stamps.append(stamp);counts[ident]=counts.get(ident,0)+1
        if ident in last:
            gap=stamp-last[ident];maxgap=max(maxgap,gap)
            if gap<0 or gap>.1:transport.append(dict(tag,reason='per_id_gap',gap_s=gap))
        last[ident]=stamp;cache[ident]=row
        expected=after['raw_feedback'].get(hex(ident))
        if expected and expected['kernel_unix_s']==stamp and expected['data_hex']==row['data_hex']:matched.add(ident)
        if 677<=ident<=679:
            for pair,raw in enumerate(struct.unpack('>ii',data)):
                axis=(ident-677)*2+pair
                if not BOUNDS[axis][0]<=raw<=BOUNDS[axis][1]:nominal.append(dict(tag,axis=axis+1,raw=raw))
                if stamp>=start:
                    ranges[axis]=[min(ranges[axis][0],raw),max(ranges[axis][1],raw)]
                    excess=max(min(before['raw_q'][axis],target_raw[axis])-raw,
                        raw-max(before['raw_q'][axis],target_raw[axis]),0)
                    excesses[axis]=max(excesses[axis],excess)
                    if outside_joint_box(raw,axis,before['raw_q'][axis],target_raw[axis]):
                        tracking.append(dict(tag,axis=axis+1,raw=raw,excess_mdeg=excess,
                            margin_rad=math.pi/600 if axis==4 else .003))
        elif ident==673:
            status_payloads.add(row['data_hex'])
            if(data[0]!=1 or data[1]!=0 or data[2]!=1 or data[3]!=0 or data[4]not in(0,1)
                or int.from_bytes(data[6:8],'big')):health.append(dict(tag,reason='arm_status'))
        elif 609<=ident<=614:
            motor_codes.setdefault(ident,set()).add(data[5])
            if data[5]!=64:health.append(dict(tag,reason='motor_flags'))
        elif ident==680:
            opening_raw=struct.unpack('>i',data[:4])[0];opening=opening_raw/1e6
            if not limits['gripper_min_m']<=opening<=limits['gripper_max_m']or data[6]!=64:
                jaw.append(dict(tag,reason='jaw_range_or_code',opening_m=opening))
            if stamp>=start:
                jrange=[min(jrange[0],opening),max(jrange[1],opening)]
                # Exact native micrometres implement the inclusive0.5mm bound.
                # The live controller is unchanged and may conservatively
                # reject an exact500um boundary due to floating arithmetic.
                if abs(opening_raw-round(before['opening_m']*1e6))>500 or data[6]!=before['jaw_code']:
                    jaw.append(dict(tag,reason='jaw_changed_during_joint',opening_m=opening))
        if stamp<start:continue
        if ident in(674,675,676)and all(i in cache for i in(674,675,676)):
            values=[];times=[]
            for i in(674,675,676):
                values.extend(struct.unpack('>ii',bytes.fromhex(cache[i]['data_hex'])))
                times.append(cache[i]['timestamp'])
            if max(times)-min(times)>.1:transport.append(dict(tag,reason='pose_fragment_skew'))
            pose=[v/1e6 for v in values[:3]]+[v*RAD for v in values[3:]]
            geometry(pose,tag,'feedback')
        if ident in(677,678,679)and all(i in cache for i in(677,678,679)):
            q=[];times=[]
            for i in(677,678,679):
                q.extend(struct.unpack('>ii',bytes.fromhex(cache[i]['data_hex'])))
                times.append(cache[i]['timestamp'])
            if max(times)-min(times)>.1:transport.append(dict(tag,reason='joint_fragment_skew'))
            key=tuple(q)
            if key not in seen_fk:
                pose=fk([v*RAD for v in q]);seen_fk[key]=pose;geometry(pose,tag,'manufacturer_fk')
            if all(i in cache for i in(674,675,676)):
                pose_raw=[];pose_times=[]
                for i in(674,675,676):
                    pose_raw.extend(struct.unpack('>ii',bytes.fromhex(cache[i]['data_hex'])))
                    pose_times.append(cache[i]['timestamp'])
                pose=[v/1e6 for v in pose_raw[:3]]+[v*RAD for v in pose_raw[3:]]
                if max(times+pose_times)-min(times+pose_times)>.1:
                    transport.append(dict(tag,reason='joint_pose_fragment_skew'))
                if math.dist(seen_fk[key][:3],pose[:3])>.002 or rotation(seen_fk[key],pose)>.02:
                    health.append(dict(tag,reason='manufacturer_fk_feedback_mismatch'))
    require(set(counts)==IDS and matched==IDS,'All14 terminal feedback frames must match raw timestamps and bytes')
    return dict(nominal_violations=nominal,joint_tracking_violations=tracking,jaw_guard_violations=jaw,
        health_violations=health,health_violations_scope='Hardware flags plus explicitly labelled original execution geometry guards',
        transport_violations=transport,transport_clean=not transport,full_raw_reviewed=True,
        first_kernel_unix_s=min(stamps),last_kernel_unix_s=max(stamps),
        feedback_frame_count=sum(counts.values()),feedback_counts={hex(k):v for k,v in sorted(counts.items())},
        control_frames=controls,post_command_joint_ranges_raw=ranges,max_joint_box_excess_mdeg=excesses,
        post_command_jaw_range_m=jrange,max_observed_translation_m=xyzmax,max_observed_rotation_rad=rotmax,
        max_manufacturer_fk_translation_m=fkxyzmax,max_manufacturer_fk_rotation_rad=fkrotmax,
        unique_manufacturer_fk_states=len(seen_fk),max_per_id_gap_s=maxgap,max_host_frame_age_s=maxage,
        status_payloads=sorted(status_payloads),motor_codes={hex(k):sorted(v)for k,v in motor_codes.items()},
        all_14_result_feedback_frames_matched_exactly=True,
        jaw_boundary_semantics='Exact raw <=500um; unchanged live floating guard can conservatively reject an exact boundary; failed result never passes',
        guard_checks_clean=not any((nominal,tracking,jaw,health,transport)))


def audit(result_path,state_path,raw_path,config_path,session_path):
    # Import only existing pure planning helpers. No C_PiperInterface instance.
    import gripper70_profile as profile
    import piper_sdk.kinematics.piper_fk as manufacturer
    require(hashlib.sha256(Path(manufacturer.__file__).read_bytes()).hexdigest()==FK_SHA,'Manufacturer FK changed')
    rb=result_path.read_bytes();sb=state_path.read_bytes();session_bytes=session_path.read_bytes()
    result=json.loads(rb);state=json.loads(sb);session=json.loads(session_bytes)
    require(result['sequence']==state['sequence']==session['status']['sequence'],'Action/session sequence mismatch')
    stage=result.get('regrasp_probe_stage')or state['regrasp_stage']
    require(stage in STAGES and state['kind']=='joint','Only separation and wrist probes supported')
    require(session['identity']['adoption_token']==state['adoption_token'],'Session adoption mismatch')
    before=state['before'];target=state['target_raw'];anchor=session['scope_anchor_raw']
    delta=[0,-350,350,0,0,0]if stage=='retreat_ready'else[0,0,0,0,-150,0]
    require(target==[a+d for a,d in zip(anchor,delta)],'Not the exact reviewed probe target')
    receipts=state['receipts']
    require(len(receipts)==1 and receipts[0]['kind']=='initial'
        and receipts[0]['attempted_frames']==receipts[0]['socket_send_returns']==4
        and receipts[0]['target_raw']==target,'Probe receipt incomplete or contains an overwrite')
    config_bytes=config_path.read_bytes()
    require(hashlib.sha256(config_bytes).hexdigest()==session['identity']['config_sha256'],'Configuration binding changed')
    limits=profile.checked_limits(json.loads(config_bytes));fk=profile.support_view().manufacturer_fk()
    plan=profile.path_check(before,target,limits,fk)
    after=result.get('after')or result.get('latest_state');require(after is not None,'Terminal feedback absent')
    receipt=receipts[0];start=receipt['started_unix_s'];end=max(after['stamps'])
    rows,source=window(raw_path,start-.2,end+.05)
    report=check_rows(rows,before,target,after,limits,fk,start)
    require(report['first_kernel_unix_s']<start and report['last_kernel_unix_s']>end,'Incomplete probe coverage')
    report.update(schema_version=1,sequence=result['sequence'],generation=state['generation'],
        adoption_token=state['adoption_token'],completed_result_sha256=hashlib.sha256(rb).hexdigest(),
        state_snapshot_sha256=hashlib.sha256(sb).hexdigest(),session_snapshot_sha256=hashlib.sha256(session_bytes).hexdigest(),
        source_windows=[source],result_phase=result['phase'],action_failure=result.get('failure'),
        probe_stage=stage,scope_anchor_raw=anchor,origin_raw=before['raw_q'],target_raw=target,receipt=receipt,
        manufacturer_fk_sha256=FK_SHA,complete_joint_box_proof={k:v for k,v in plan.items()if k!='samples'},
        joint_tracking_margin_rad=list(profile.MARGINS),
        stable_window=result.get('stable_window'),actual_probe_progress=result.get('probe_motion_evidence'),
        command_evidence_source='frozen_driver_socket_send_receipt_not_bus_delivery',
        control_absence_is_not_zero_tx_proof=True,contact_absent_verified=False,prior_failures_preserved=True,
        scope='All received raw fragments in this action window; pure robot geometry only, no clearance or stop guarantee')
    report['review_pass']=(result['phase']=='completed'and report['guard_checks_clean']
        and result.get('probe_motion_evidence',{}).get('sufficient')is True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in('result','state','raw','config','session','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();report=audit(args.result,args.state,args.raw,args.config,args.session)
    with args.output.open('x')as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps(dict(path=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        review_pass=report['review_pass'],frames=report['feedback_frame_count'],
        max_joint_box_excess_mdeg=report['max_joint_box_excess_mdeg'])))
    return 0 if report['review_pass']else 1


if __name__=='__main__':raise SystemExit(main())
