"""Per-received-frame guards. No ROS, SDK construction, files, or device I/O.

The first observed violation is permanent for this process. This blocks later
transmissions; it does not physically stop or cancel an already accepted goal.
"""
import copy
import math
import struct
import threading
import time

RAD_PER_RAW=math.pi/180000.
JOINT_LIMITS_RAW=((-150000,150000),(0,180000),(-170000,0),(-100000,100000),(-70000,70000),(-120000,120000))
FEEDBACK_IDS=frozenset(range(0x2a1,0x2a9))|frozenset(range(0x261,0x267))


class RawFeedbackLatch:
    def __init__(self,limits,lock=None):
        self.lock=lock if lock is not None else threading.RLock()
        self.limits=copy.deepcopy(limits)
        self.first_fault=None
        self.context=None
        self.frames_checked=0
        self.last_stamps={}

    def assert_clean(self):
        with self.lock:
            if self.first_fault is not None:
                raise RuntimeError("Raw feedback violation latched: "+self.first_fault["reason"])

    def arm_joint(self,sequence,generation,origin_raw,target_raw,jaw_m,jaw_code=64,token="",*,hold_box=None):
        self._arm("joint",sequence,generation,origin_raw,target_raw,jaw_m,jaw_code,token,hold_box)

    def arm_jaw(self,sequence,generation,origin_raw,target_raw,jaw_m,jaw_code=64,token="",*,hold_box=None):
        self._arm("gripper",sequence,generation,origin_raw,target_raw,jaw_m,jaw_code,token,None)

    def _arm(self,kind,sequence,generation,origin,target,jaw_m,jaw_code,token,hold_box):
        with self.lock:
            self.assert_clean()
            if len(origin)!=6 or len(target)!=6:raise RuntimeError("Six-axis RX context required")
            context=dict(kind=kind,sequence=sequence,generation=generation,adoption_token=token,
                origin_raw=list(origin),target_raw=list(target),jaw_m=jaw_m,jaw_code=jaw_code,
                hold_box=copy.deepcopy(hold_box))
            self.context=context

    def _fault(self,reason,ident,data,stamp,host,**details):
        if self.first_fault is None:
            self.first_fault=dict(reason=reason,id=ident,data_hex=bytes(data).hex(),kernel_unix_s=stamp,
                observed_unix_s=host,context=copy.deepcopy(self.context),frame_index=self.frames_checked,
                accepted_target_not_cancelled=True,**details)

    def reject_frame(self,reason,ident,data,stamp,host):
        with self.lock:
            self.frames_checked+=1
            self._fault(reason,ident,data,stamp,host)

    def observe(self,can_id,data,kernel_s,host_s):
        """Check each raw pair immediately, before a later pair can replace it."""
        if can_id not in FEEDBACK_IDS:return
        with self.lock:
            self.frames_checked+=1
            if self.first_fault is not None:return
            bad=lambda why,**kw:self._fault(why,can_id,data,kernel_s,host_s,**kw)
            if (len(data)!=8 or not math.isfinite(kernel_s) or kernel_s<=0 or kernel_s>host_s
                    or host_s-kernel_s>self.limits["max_state_age_s"]):
                bad("Invalid or stale raw feedback frame");return
            old=self.last_stamps.get(can_id)
            if old is not None and kernel_s<old:
                bad("Raw feedback timestamp regressed",previous_kernel_unix_s=old);return
            self.last_stamps[can_id]=kernel_s
            context=self.context
            if 0x2a5<=can_id<=0x2a7:
                for offset,raw in enumerate(struct.unpack(">ii",data)):
                    axis=(can_id-0x2a5)*2+offset;lo,hi=JOINT_LIMITS_RAW[axis]
                    if not lo<=raw<=hi:
                        bad("Manufacturer nominal joint limit",axis=axis+1,raw=raw,lower_raw=lo,upper_raw=hi);return
                    if context is not None:
                        boxes=[("original",context["origin_raw"],context["target_raw"])]
                        if context["hold_box"] is not None:
                            box=context["hold_box"];boxes.append(("hold",box["origin_raw"],box["target_raw"]))
                        for label,start,target in boxes:
                            lower=min(start[axis],target[axis])*RAD_PER_RAW-.003
                            upper=max(start[axis],target[axis])*RAD_PER_RAW+.003
                            if not lower<=raw*RAD_PER_RAW<=upper:
                                bad("Outside "+label+" joint box",axis=axis+1,raw=raw,
                                    lower_rad=lower,upper_rad=upper,tracking_tolerance_rad=.003);return
            elif can_id==0x2a1:
                if (data[0]!=1 or data[1]!=0 or data[2]!=1 or data[3]!=0 or data[4]not in(0,1)
                        or int.from_bytes(data[6:8],"big")!=0):
                    bad("Unhealthy raw arm status")
            elif 0x261<=can_id<=0x266:
                if data[5]!=64:bad("Unhealthy raw motor flags",axis=can_id-0x260,driver_code=data[5])
            elif can_id==0x2a8:
                width=struct.unpack(">i",data[:4])[0]/1e6
                if data[6]!=64 or not self.limits["gripper_min_m"]<=width<=self.limits["gripper_max_m"]:
                    bad("Unhealthy raw jaw",opening_m=width,jaw_code=data[6])
                elif context is not None and context["kind"]!="gripper"and(
                        data[6]!=context["jaw_code"]or abs(width-context["jaw_m"])>.0005):
                    bad("Jaw changed during joint command",opening_m=width,jaw_code=data[6])


def sdk_overlay(Parent,limits):
    """Wrap the already reviewed SDK adapter, before ConnectPort starts RX."""
    class LatchedPiper(Parent):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.rx_latch=RawFeedbackLatch(limits,self.rx_lock)
            self.rx_context_provider=None
            bus=self.GetCanBus().send_bus;send=bus.send
            def latched_send(frame,*a,**k):
                # One frame only: a fault received between transaction frames
                # must block the remainder, leaving an explicit partial receipt.
                with self.rx_lock:
                    self.rx_latch.assert_clean()
                    provider=self.rx_context_provider
                    if provider is None:
                        self.broken="RX transmission context unavailable"
                        raise RuntimeError(self.broken)
                    c=provider()  # Immutable worker-owned data; no locks/I/O.
                    if c["kind"]not in("gripper","joint"):raise RuntimeError("Unknown RX action context")
                    method=self.rx_latch.arm_jaw if c["kind"]=="gripper"else self.rx_latch.arm_joint
                    method(c["sequence"],c["generation"],c["origin_raw"],c["target_raw"],
                           c["jaw_m"],c["jaw_code"],c["token"],hold_box=c.get("hold_box"))
                    self.rx_latch.assert_clean()
                    return send(frame,*a,**k)
            bus.send=latched_send

        def ParseCANFrame(self,message):
            with self.rx_lock:
                if(message is not None and message.arbitration_id in FEEDBACK_IDS
                   and getattr(message,"is_rx",False)):
                    latch=getattr(self,"rx_latch",None)
                    if latch is None:self.broken="Raw guard unavailable before receive"
                    else:
                        if message.is_extended_id or message.is_remote_frame or message.is_error_frame:
                            latch.reject_frame("Malformed raw feedback flags",message.arbitration_id,
                                bytes(message.data),float(message.timestamp),time.time())
                        else:latch.observe(message.arbitration_id,bytes(message.data),float(message.timestamp),time.time())
                        if latch.first_fault is not None:
                            self.broken="Raw feedback violation latched: "+latch.first_fault["reason"]
                return super().ParseCANFrame(message)

        def healthy(self,snapshot,allow_moving=False):
            self.rx_latch.assert_clean()
            return super().healthy(snapshot,allow_moving=allow_moving)

        def snapshot(self):
            result=super().snapshot()
            with self.rx_lock:
                result["raw_feedback_fault"]=copy.deepcopy(self.rx_latch.first_fault)
                result["raw_feedback_frames_checked"]=self.rx_latch.frames_checked
            return result
    return LatchedPiper
