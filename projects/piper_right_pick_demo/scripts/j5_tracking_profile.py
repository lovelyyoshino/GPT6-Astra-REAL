"""One approved tracking profile; pure math and per-instance RX adaptation.

Only joint-motion J5 tracking changes. No module globals in frozen dependencies
are patched; arrival, slip, baseline, jaw-motion and nominal guards stay intact.
"""
import copy
import math
import struct
import types

import ros_guarded_rx_entry as frozen
support=frozen.support;require=frozen.require
MARGINS=(.003,.003,.003,.003,math.pi/600,.003)


def inside_box(raw,origin,target,axis,*,jaw=False):
    excess=max(min(origin,target)-raw,raw-max(origin,target),0)
    # The approved J5 allowance is exactly 300 integer feedback units. Avoid
    # cancellation at negative absolute angles rejecting the exact boundary.
    return excess<=300 if axis==4 and not jaw else excess*support.RAD_PER_RAW<=.003


def checked_limits(config):
    limits=support.checked_probe_limits(config)
    value=limits.get("joint_tracking_margin_rad")
    require(isinstance(value,(list,tuple))and len(value)==6
        and all(type(x)in(int,float)and math.isfinite(x)and x==m for x,m in zip(value,MARGINS)),
        "Explicit approved J5-only tracking profile required")
    return limits


def path_check(before,raw_target,limits,fk):
    require(tuple(limits.get("joint_tracking_margin_rad",()))==MARGINS,"Path/RX tracking profile mismatch")
    path=support.path_check(before,raw_target,limits,fk)
    # The frozen analytic proof already budgets .003 on every axis. Adding
    # exactly the J5 increment is algebraically the same independent-box proof.
    extra=MARGINS[4]-.003
    position=path["position_bound_m"]+fk.joint_radius_bounds_m[4]*extra
    rotation=path["rotation_bound_rad"]+extra
    require(position<=limits["max_translation_step_m"]and rotation<=limits["max_rotation_step_rad"],
        "Profile joint box exceeds original motion bounds")
    require(all(lo<=p-position and p+position<=hi for p,lo,hi in
        zip(before["pose"][:3],limits["workspace_min_m"],limits["workspace_max_m"])),
        "Profile joint box exceeds original workspace")
    return dict(path,position_bound_m=position,rotation_bound_rad=rotation,
                joint_tracking_margin_rad=list(MARGINS),jaw_tracking_margin_rad=[.003]*6)


def monitor(s,origin,target,limits):
    require(tuple(limits.get("joint_tracking_margin_rad",()))==MARGINS,"Monitor/profile mismatch")
    support.envelope(s["q"],s["pose"],origin["pose"],limits)
    target_raw=[round(v/support.RAD_PER_RAW)for v in target]
    require(all(inside_box(q,a,b,i)for i,(q,a,b)in enumerate(zip(s["raw_q"],origin["raw_q"],target_raw))),
            "Outside approved original/hold joint box")
    require(s["jaw_code"]==origin["jaw_code"]and abs(s["opening_m"]-origin["opening_m"])<=.0005,"Jaw changed")


def private_function(function,**overrides):
    """Reuse reviewed bytecode with an isolated dependency dictionary."""
    namespace=dict(function.__globals__);namespace.update(overrides)
    result=types.FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    result.__kwdefaults__=copy.copy(function.__kwdefaults__)
    result.__annotations__=dict(function.__annotations__)
    return result


def support_view():
    namespace=dict(vars(support));namespace.update(path_check=path_check,checked_probe_limits=checked_limits)
    return types.SimpleNamespace(**namespace)


class ProfileLatch(frozen.overlay.RawFeedbackLatch):
    def __init__(self,limits,lock=None):
        require(tuple(limits.get("joint_tracking_margin_rad",()))==MARGINS,"RX/profile mismatch")
        super().__init__(limits,lock)

    def observe(self,can_id,data,kernel_s,host_s):
        if not 0x2a5<=can_id<=0x2a7:
            return super().observe(can_id,data,kernel_s,host_s)
        with self.lock:
            # Retain all frozen transport/nominal checks. Temporarily omit only
            # its hardcoded tracking comparison, under the same exclusive lock.
            context=self.context;previous_fault=self.first_fault
            self.context=None
            try:super().observe(can_id,data,kernel_s,host_s)
            finally:self.context=context
            if self.first_fault is not None:
                if previous_fault is None:self.first_fault["context"]=copy.deepcopy(context)
                return
            if context is None:return
            margins=[.003]*6 if context["kind"]=="gripper"else MARGINS
            boxes=[("original",context["origin_raw"],context["target_raw"])]
            if context["hold_box"]is not None:
                box=context["hold_box"];boxes.append(("hold",box["origin_raw"],box["target_raw"]))
            for offset,raw in enumerate(struct.unpack(">ii",data)):
                axis=(can_id-0x2a5)*2+offset;margin=margins[axis]
                for label,origin,target in boxes:
                    lower=min(origin[axis],target[axis])*support.RAD_PER_RAW-margin
                    upper=max(origin[axis],target[axis])*support.RAD_PER_RAW+margin
                    if not inside_box(raw,origin[axis],target[axis],axis,jaw=context["kind"]=="gripper"):
                        self._fault("Outside "+label+" joint box",can_id,data,kernel_s,host_s,
                            axis=axis+1,raw=raw,lower_rad=lower,upper_rad=upper,tracking_tolerance_rad=margin)
                        return


def sdk_overlay(Parent,limits):
    Wrapped=frozen.overlay.sdk_overlay(Parent,limits)
    class ProfilePiper(Wrapped):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            require(self.rx_latch.frames_checked==0 and self.rx_latch.first_fault is None,
                    "Profile must be installed before reception")
            self.rx_latch=ProfileLatch(limits,self.rx_lock)
    return ProfilePiper
