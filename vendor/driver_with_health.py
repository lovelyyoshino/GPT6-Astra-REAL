#!/usr/bin/env python3
"""显式 roslaunch 才运行厂家驱动；在 SDK 接收路径记录逐帧健康度。

只扩展遥测，不复制厂家源码，不创建第二个 CAN 控制者。
直接调用本脚本会启动驱动并产生厂家初始化通信；本轮未执行。
"""

import hashlib
import importlib.util
import json
import os
import sys
import threading
import time


EXPECTED_DRIVER_SHA256='ecff6823dc6bbf55fe51708024ef88d993fead97572ad7c06e951020e83bc200'
EXPECTED_SDK_VERSION='0.6.2'


def main():
    if len(sys.argv)<2 or not os.path.isfile(sys.argv[1]):
        raise SystemExit('Use the generated guarded ROS launch; vendor driver path is required.')
    source=os.path.realpath(sys.argv[1])
    if hashlib.sha256(open(source,'rb').read()).hexdigest()!=EXPECTED_DRIVER_SHA256:
        raise SystemExit('Vendor driver changed: re-audit callbacks before updating the pinned SHA256.')
    import importlib.metadata
    if importlib.metadata.version('piper-sdk')!=EXPECTED_SDK_VERSION:
        raise SystemExit('SDK version mismatch: health adapter audited for piper-sdk 0.6.2.')
    import rospy
    from std_msgs.msg import String
    from piper_sdk import C_PiperInterface
    from piper_sdk.piper_msgs.msg_v2.can_id import CanIDPiper
    ids=[CanIDPiper.ARM_STATUS_FEEDBACK,CanIDPiper.ARM_GRIPPER_FEEDBACK,
         CanIDPiper.ARM_JOINT_FEEDBACK_12,CanIDPiper.ARM_JOINT_FEEDBACK_34,CanIDPiper.ARM_JOINT_FEEDBACK_56]
    ids += [getattr(CanIDPiper,'ARM_INFO_LOW_SPD_FEEDBACK_'+str(i)) for i in range(1,7)]
    monitored={x.value:x.name for x in ids}

    class ObservedPiper(C_PiperInterface):
        def __init__(self,*args,**kwargs):
            import rosgraph
            own=rospy.get_name();can_name=kwargs.get('can_name')
            pubs,subs,services=rosgraph.Master(own).getSystemState()
            live={n for _,nodes in pubs+subs+services for n in nodes}
            for param in rospy.get_param_names():
                owner=param.rsplit('/',1)[0]
                if param.endswith('/can_port') and owner!=own and owner in live and rospy.get_param(param)==can_name:
                    raise RuntimeError('another live ROS driver owns '+str(can_name)+': '+owner)
            self.eval_lock=threading.Lock();self.eval_seen={};self.eval_sequence=0
            super().__init__(*args,**kwargs)
        def ParseCANFrame(self,message):
            result=super().ParseCANFrame(message)
            if message is not None and message.arbitration_id in monitored and len(message.data)==8 and not message.is_error_frame and not message.is_remote_frame and not message.is_extended_id and getattr(message,'is_rx',True):
                with self.eval_lock:
                    self.eval_seen[message.arbitration_id]=(time.monotonic(),rospy.Time.now().to_sec())
                    self.eval_sequence+=1
            return result

    # ROS remap 参数继续交给厂家 init_node；仅替换 SDK 的被动观测子类。
    sys.argv=sys.argv[1:]
    spec=importlib.util.spec_from_file_location('audited_piper_driver',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.C_PiperInterface=ObservedPiper
    node=module.C_PiperRosNode()
    publisher=rospy.Publisher('eval_telemetry',String,queue_size=1)
    def health(_):
        sdk=node.piper;now=time.monotonic()
        with sdk.eval_lock:seen=dict(sdk.eval_seen);sequence=sdk.eval_sequence
        ages={monitored[k]:max(0,now-seen[k][0]) if k in seen else 1e9 for k in monitored}
        joint_ids=[CanIDPiper.ARM_JOINT_FEEDBACK_12.value,CanIDPiper.ARM_JOINT_FEEDBACK_34.value,CanIDPiper.ARM_JOINT_FEEDBACK_56.value]
        joint_stamp=min([seen[k][1] for k in joint_ids if k in seen]) if all(k in seen for k in joint_ids) else 0.0
        joints=sdk.GetArmJointMsgs().joint_state
        grip=sdk.GetArmGripperMsgs().gripper_state
        status=sdk.GetArmStatus().arm_status
        low=sdk.GetArmLowSpdInfoMsgs()
        enabled=[];motor_faults=[]
        for i in range(1,7):
            foc=getattr(low,'motor_'+str(i)).foc_status
            enabled.append(bool(foc.driver_enable_status))
            motor_faults.append(any(bool(getattr(foc,k,False)) for k in
                ('voltage_too_low','motor_overheating','driver_overcurrent','driver_overheating','collision_status','driver_error_status','stall_status')))
        payload={'can_interface':node.can_port,'joint_stamp':joint_stamp,'source_sequence':sequence,
                 'stamp':rospy.Time.now().to_sec(),'q':[getattr(joints,'joint_'+str(i))*3.141592653589793/180000 for i in range(1,7)],
                 'opening_m':grip.grippers_angle/1000000,'gripper_torque_sdk_units':grip.grippers_effort,
                 'ctrl_mode':int(status.ctrl_mode),'arm_status':int(status.arm_status),'teach_status':int(status.teach_status),
                 'motion_status':int(status.motion_status),'enabled':enabled,'driver_accepts_commands':bool(node.GetEnableFlag()),
                 'fault':int(status.err_code) or int(any(motor_faults)),
                 'feedback_max_age_s':max(ages.values()),'frame_ages_s':ages,'source':'sdk_receive_frames',
                 'driver_sha256':EXPECTED_DRIVER_SHA256,'sdk_version':EXPECTED_SDK_VERSION}
        publisher.publish(String(data=json.dumps(payload,separators=(',',':'),allow_nan=False)))
    timer=rospy.Timer(rospy.Duration(0.02),health)
    try:node.Pubilsh()
    finally:timer.shutdown()


if __name__=='__main__':main()
