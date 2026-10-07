"""Pure offline planner for exactly +15 mm base Z, preserving reported attitude.

Uses official SDK FK, nearby numerical IK and a sampled independent-joint box.
No CAN/ROS interface is instantiated. Geometry checks are not collision proofs.
"""
import hashlib
import inspect
import itertools
import json
from pathlib import Path

LIMITS_DEG = ((-150, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120))
CHECKS = dict(xy_max_mm=3., z_min_mm=-7., z_max_mm=23., rotation_max_deg=5., gripper_dip_max_mm=15.)


def _raw6(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != 6 or
            any(type(v) is not int or abs(v) > 2147483647 for v in value)):
        raise ValueError(name + ' must contain exactly six integer raw SDK values')
    return list(value)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def _sample_bounds(fk, start, target, hull, np, Rotation):
    lower, upper = np.minimum(start, target)-.15, np.maximum(start, target)+.15
    p0 = fk(start); r0 = Rotation.from_euler('xyz', p0[3:], degrees=True)
    rotation_row0 = _matrix(r0)[2]
    samples = list(itertools.product(*zip(lower, upper)))
    samples += list(itertools.product(*[np.linspace(a, b, 3) for a, b in zip(lower, upper)]))
    xyz, angle, dip = [], [], 0.
    for q in samples:
        p = fk(np.asarray(q)); r = Rotation.from_euler('xyz', p[3:], degrees=True)
        delta = p[:3]-p0[:3]
        xyz.append(delta); angle.append(np.rad2deg(np.linalg.norm((r0.inv()*r).as_rotvec())))
        # Maximum downward displacement of any hull point, conservative for all openings.
        dip = max(dip, float(-(hull@(_matrix(r)[2]-rotation_row0)+delta[2]).min()))
    xyz = np.asarray(xyz)
    bounds = dict(joint_lower_raw=(np.rint(np.minimum(start, target)*1000).astype(int)-150).tolist(),
                  joint_upper_raw=(np.rint(np.maximum(start, target)*1000).astype(int)+150).tolist(),
                  end_xy_max_mm=float(np.linalg.norm(xyz[:, :2], axis=1).max()),
                  end_z_delta_min_mm=float(xyz[:, 2].min()), end_z_delta_max_mm=float(xyz[:, 2].max()),
                  orientation_max_deg=float(max(angle)), gripper_point_dip_max_mm=dip,
                  sample_count=len(samples), checks=dict(CHECKS),
                  note='64 corners + 3^6 grid; sampled geometry estimate, not proven extrema or collision clearance')
    if (bounds['end_xy_max_mm'] > 3 or bounds['end_z_delta_min_mm'] < -7 or
            bounds['end_z_delta_max_mm'] > 23 or bounds['orientation_max_deg'] > 5 or dip > 15):
        raise ValueError('sampled movement envelope exceeds reviewed bounds: '+json.dumps(bounds))
    return bounds


def plan_lift(joints_raw, pose_raw):
    """Return one reviewed lift plan from fresh feedback; raise ValueError on rejection."""
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics

    joints_raw, pose_raw = _raw6(joints_raw, 'joints_raw'), _raw6(pose_raw, 'pose_raw')
    q = np.asarray(joints_raw, dtype=float)/1000.; measured = np.asarray(pose_raw, dtype=float)/1000.
    lower, upper = np.asarray(LIMITS_DEG, dtype=float).T
    allowed_lower, allowed_upper = lower.copy(), upper.copy()
    allowed_lower[1], allowed_upper[2] = -3., 4.
    if np.any(q < allowed_lower) or np.any(q > allowed_upper):
        raise ValueError('current joints exceed limited observed recovery input range')
    if np.any(np.abs(measured[:3]) > 1000) or np.any(np.abs(measured[3:]) > 360):
        raise ValueError('reported pose exceeds plausible raw SDK range')
    official = C_PiperForwardKinematics(1)
    def fk(degrees):
        return np.asarray(official.CalFK(np.deg2rad(degrees).tolist())[-1], dtype=float)
    def rotation(p):
        return Rotation.from_euler('xyz', p[3:], degrees=True)
    p0 = fk(q); r_measured = rotation(measured)
    error_mm = float(np.linalg.norm(p0[:3]-measured[:3]))
    error_deg = float(np.rad2deg(np.linalg.norm((r_measured.inv()*rotation(p0)).as_rotvec())))
    if not np.all(np.isfinite(p0)) or error_mm > .3 or error_deg > .05:
        raise ValueError('official FK does not match fresh reported pose: %.6f mm, %.6f deg' % (error_mm, error_deg))
    wanted = measured.copy(); wanted[2] += 15.
    def residual(degrees):
        p = fk(degrees)
        return np.r_[p[:3]-wanted[:3], (r_measured.inv()*rotation(p)).as_rotvec()*100.]
    # Constrain optimization to the reviewed local branch from the outset.
    local_lower, local_upper = np.maximum(lower, q-4.), np.minimum(upper, q+4.)
    if np.any(local_lower >= local_upper):
        raise ValueError('no nearby nominal joint interval for lift')
    seed = np.clip(q, local_lower+1e-7, local_upper-1e-7)
    solve = least_squares(residual, seed, bounds=(local_lower, local_upper), max_nfev=150,
                          xtol=1e-10, ftol=1e-10, gtol=1e-10)
    target_raw = np.rint(solve.x*1000).astype(int); target = target_raw/1000.
    endpoint = fk(target)
    endpoint_mm = float(np.linalg.norm(endpoint[:3]-wanted[:3]))
    endpoint_deg = float(np.rad2deg(np.linalg.norm((r_measured.inv()*rotation(endpoint)).as_rotvec())))
    if (not solve.success or np.any(target < lower) or np.any(target > upper) or
            np.any(np.abs(target-q) > 4.) or endpoint_mm > .1 or endpoint_deg > .03):
        raise ValueError('no nearby nominal lift IK meeting endpoint tolerance')
    hull_path = Path(__file__).with_name('vendor_gripper_hull.json')
    hull = np.asarray(json.loads(hull_path.read_text())['vertices_mm'], dtype=float)
    if hull.ndim != 2 or hull.shape[1] != 3 or not np.all(np.isfinite(hull)):
        raise ValueError('invalid packaged vendor gripper hull')
    bounds = _sample_bounds(fk, q, target, hull, np, Rotation)
    target_pose = list(pose_raw); target_pose[2] += 15000
    return dict(start_joints_raw=joints_raw, start_pose_raw=pose_raw,
                target_joints_raw=target_raw.tolist(), target_pose_raw=target_pose,
                fk_error_mm=error_mm, fk_error_deg=error_deg,
                endpoint_error_mm=endpoint_mm, endpoint_error_deg=endpoint_deg,
                model_bounds=bounds, source_sha=dict(planner=_sha(__file__),
                    sdk_fk=_sha(inspect.getfile(C_PiperForwardKinematics)), gripper_hull=_sha(hull_path)))
