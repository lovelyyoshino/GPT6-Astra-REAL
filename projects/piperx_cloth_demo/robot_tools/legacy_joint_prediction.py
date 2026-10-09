"""Local endpoint prediction before legacy MOVE_L, never a motion dispatch.

Does not certify the controller's IK branch, interpolation or swept clearance.
Measured joints, physical-model envelope and all post-send guards still apply.
"""
import math

from . import arms


def predict(profile, joints, target, limits, compatibility, rotation_distance):
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    from .legacy_controller import model_delta

    def fk(q):
        return arms.vendor_fk('piper',list(q),profile['sdk_path'])['pose_m_rad']
    wanted_rotation=Rotation.from_euler('xyz',target[3:])
    def residual(q):
        pose=fk(q)
        return np.r_[np.array(pose[:3])-target[:3],
                     (wanted_rotation.inv()*Rotation.from_euler('xyz',pose[3:])).as_rotvec()]
    fit=least_squares(residual,np.array(joints),max_nfev=50,xtol=1e-10,ftol=1e-10,gtol=1e-10)
    pose=fk(fit.x)
    if (not fit.success or not np.isfinite(fit.x).all()
            or math.dist(pose[:3],target[:3])>.00005 or rotation_distance(pose,target)>.0002):
        raise RuntimeError('Legacy local endpoint prediction did not converge; no send')
    violations=[i for i,q in enumerate(fit.x,1) if not limits['joint%d'%i][0] <= q <= limits['joint%d'%i][1]]
    if violations:
        raise RuntimeError('Predicted legacy target exceeds applicable joint limits before send: '+repr(violations))
    delta=model_delta(profile,joints,list(fit.x),rotation_distance,compatibility)
    return dict(joints_rad=list(fit.x),physical_model_delta=delta,
                controller_endpoint_error_m=math.dist(pose[:3],target[:3]),
                manufacturer_ik_path_verified=False,collision_path_verified=False,
                scope='Local numerical endpoint prediction, not controller branch or swept-path certification')
