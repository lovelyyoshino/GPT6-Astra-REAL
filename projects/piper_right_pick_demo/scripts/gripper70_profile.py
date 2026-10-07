"""Private 70 mm command/RX envelope; all frozen motion guards are retained."""
import copy
import types
import j5_tracking_profile as original

require=original.require
MARGINS=original.MARGINS
MAX_OPENING_M=.070
private_function=original.private_function
path_check=original.path_check
monitor=original.monitor
inside_box=original.inside_box
ProfileLatch=original.ProfileLatch


def checked_limits(config):
    value=config['physical_limits']['gripper_max_m']
    require(type(value)in(int,float)and value==MAX_OPENING_M,'Explicit70mm software envelope required')
    narrow=copy.deepcopy(config)
    narrow['physical_limits']['gripper_max_m']=.055
    limits=original.checked_limits(narrow)  # Retain every other original bound.
    limits['gripper_max_m']=MAX_OPENING_M
    return limits


def support_view():
    namespace=dict(vars(original.support_view()))
    namespace.update(checked_probe_limits=checked_limits)
    return types.SimpleNamespace(**namespace)


def widen_gripper_class(Parent):
    """Keep the pinned method body, super closure and ticket checks per class.

    Only its named width integer validation changes. This does not mutate its
    source, globals, any existing class, or the already running process.
    """
    method=Parent.GripperCtrl
    require(55000 in method.__code__.co_consts and 'integer'in method.__globals__,
            'Expected pinned gripper validation method absent')
    old_integer=method.__globals__['integer']
    def integer(value,low,high,name):
        if (low,high,name)==(0,55000,'jaw width'):high=70000
        return old_integer(value,low,high,name)
    class WideGripper(Parent):
        GripperCtrl=private_function(method,integer=integer)
    return WideGripper


def sdk_overlay(Parent,limits):
    require(limits['gripper_max_m']==MAX_OPENING_M,'SDK/RX70mm envelope mismatch')
    # The original overlay still owns every-frame ticket/RX locking and health.
    return original.sdk_overlay(widen_gripper_class(Parent),limits)
