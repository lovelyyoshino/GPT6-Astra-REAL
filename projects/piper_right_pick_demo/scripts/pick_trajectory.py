"""Offline, scene-relative pick/place planning; never opens a robot connection.

The table check uses installed manufacturer collision meshes, measured scene
geometry and an explicit uncertainty reserve. It samples independent joint
progress and tracking error; this is not a continuous collision proof and does
not model the camera mount, cables, people, or other furniture.
"""
import hashlib
import itertools
import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics

LIMITS_DEG = np.array([[-150, 150], [0, 180], [-170, 0], [-100, 100], [-70, 70], [-120, 120]], float)
TCP_LOCAL_MM = np.array([0., 0., 135.8])
TRACKING_DEG = .5
MIN_CLEARANCE_MM = 10.
ARM_GEOMETRY = Path(__file__).with_name('vendor_arm_collision.json')
GRIPPER_HULL = Path(__file__).with_name('vendor_gripper_hull.json')


class PlanRejected(ValueError):
    pass


def _six(raw, name):
    if not isinstance(raw, (list, tuple)) or len(raw) != 6 or any(type(x) is not int for x in raw):
        raise PlanRejected(name + ' must have six integer SDK raw values')
    return np.array(raw, float) / 1000.


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _rotation(pose):
    return Rotation.from_euler('xyz', pose[3:], degrees=True)


def _angle(a, b):
    return float(np.rad2deg(np.linalg.norm((a.inv() * b).as_rotvec())))


@lru_cache(maxsize=1)
def _model():
    packaged = json.loads(ARM_GEOMETRY.read_text())
    origins = [np.asarray(x, float) for x in packaged['joint_origins']]
    axes = [np.asarray(x, float) for x in packaged['joint_axes']]
    hulls = [np.asarray(x, float) for x in packaged['link_hulls_mm']]
    gripper = np.asarray(json.loads(GRIPPER_HULL.read_text())['vertices_mm'], float)
    if (packaged.get('schema') != 1 or len(origins) != 6 or len(axes) != 6 or len(hulls) != 6 or
            any(x.shape != (4, 4) or not np.all(np.isfinite(x)) for x in origins) or
            any(x.shape != (3,) or not np.all(np.isfinite(x)) or abs(np.linalg.norm(x)-1.) > 1e-6 for x in axes) or
            any(x.ndim != 2 or x.shape[1] != 3 or len(x) < 4 or not np.all(np.isfinite(x)) for x in hulls+[gripper])):
        raise PlanRejected('Packaged manufacturer collision geometry is malformed')
    sources = dict(packaged['source_file_sha256'])
    sources.update({str(ARM_GEOMETRY): _sha(ARM_GEOMETRY), str(GRIPPER_HULL): _sha(GRIPPER_HULL)})
    return origins, axes, hulls, gripper, sources


def _link_transforms(q):
    origins, axes, _, _, _ = _model()
    t = np.eye(4)
    result = []
    for origin, axis, degrees in zip(origins, axes, q):
        turn = np.eye(4)
        turn[:3, :3] = Rotation.from_rotvec(axis * np.deg2rad(degrees)).as_matrix()
        t = t @ origin @ turn
        result.append(t.copy())
    return result


def _reserve(geometry):
    uncertainty = geometry.get('uncertainty', {})
    depth = float(uncertainty.get('depth_mm', 3.))
    feature = float(uncertainty.get('finger_feature_mm', 12.))
    reserve = depth + feature
    if not math.isfinite(reserve) or reserve < 0 or reserve > 30:
        raise PlanRejected('Invalid or excessive vertical geometry uncertainty')
    return max(15., reserve)


def clearance_for_joints(joints_raw, geometry):
    """Return manufacturer-CAD/table clearance after scene uncertainty reserve."""
    q = _six(joints_raw, 'joints_raw')
    table = float(geometry['table_base_z_mm'])
    if not math.isfinite(table):
        raise PlanRejected('Nonfinite table height')
    origins, axes, hulls, gripper, _ = _model()
    transforms = _link_transforms(q)
    minima = {}
    for i, (t, hull) in enumerate(zip(transforms, hulls), 1):
        minima['link%d' % i] = float((hull @ t[2, :3] + t[2, 3]).min() - table)
    # The gripper hull was packaged in official SDK flange coordinates.
    pose = np.asarray(C_PiperForwardKinematics(1).CalFK(np.deg2rad(q).tolist())[-1])
    r = _rotation(pose).as_matrix()
    minima['gripper'] = float((gripper @ r[2] + pose[2]).min() - table)
    reserve = _reserve(geometry)
    return dict(minimum_clearance_mm=min(minima.values()) - reserve,
                gripper_clearance_mm=minima['gripper'] - reserve,
                link_clearance_mm={k: v - reserve for k, v in minima.items() if k != 'gripper'},
                nominal_clearance_mm=minima, uncertainty_reserve_mm=reserve,
                excluded_geometry=['camera and bracket', 'cables', 'people', 'non-table obstacles'])


class _Builder:
    def __init__(self, q, pose, geometry):
        self.q = q.copy()
        self.pose = pose.copy()
        self.geometry = geometry
        self.fk_engine = C_PiperForwardKinematics(1)
        self.stages = []
        self.checked_states = 0
        self.min_clearance = float('inf')

    def fk(self, q):
        return np.asarray(self.fk_engine.CalFK(np.deg2rad(q).tolist())[-1])

    def tcp(self):
        return self.pose[:3] + _rotation(self.pose).apply(TCP_LOCAL_MM)

    def solve(self, tcp, rotation, seed):
        desired = tcp - rotation.apply(TCP_LOCAL_MM)
        def residual(q):
            p = self.fk(q)
            return np.r_[p[:3] - desired, (rotation.inv() * _rotation(p)).as_rotvec() * 100.]
        lo, hi = LIMITS_DEG.T
        solution = least_squares(residual, np.clip(seed, lo + 1e-8, hi - 1e-8), bounds=(lo, hi),
                                 max_nfev=180, xtol=1e-10, ftol=1e-10, gtol=1e-10)
        target = np.rint(solution.x * 1000.) / 1000.
        p = self.fk(target)
        if (not solution.success or np.linalg.norm(p[:3] - desired) > 1. or
                _angle(rotation, _rotation(p)) > .5 or np.any(target < lo) or np.any(target > hi)):
            raise PlanRejected('No nominal nearby IK for requested pick stage')
        return target

    def segment_clearance(self, target):
        lo = np.minimum(self.q, target) - TRACKING_DEG
        hi = np.maximum(self.q, target) + TRACKING_DEG
        # Corners cover independently progressing joints plus observed tracking
        # allowance; intermediate synchronized samples are also inspected.
        samples = list(itertools.product(*zip(lo, hi)))
        samples += [(self.q + (target - self.q) * t).tolist() for t in np.linspace(0., 1., 9)]
        minimum = float('inf')
        worst = None
        for sample in samples:
            raw = np.rint(np.asarray(sample) * 1000.).astype(int).tolist()
            check = clearance_for_joints(raw, self.geometry)
            self.checked_states += 1
            if check['minimum_clearance_mm'] < minimum:
                minimum, worst = check['minimum_clearance_mm'], check
        if minimum < MIN_CLEARANCE_MM:
            raise PlanRejected('Sampled table clearance %.3f mm is below %.1f mm including uncertainty/tracking' %
                               (minimum, MIN_CLEARANCE_MM))
        self.min_clearance = min(self.min_clearance, minimum)
        return minimum, worst

    def joint_move(self, target, label, depth=0):
        target = np.rint(np.asarray(target) * 1000.) / 1000.
        pose = self.fk(target)
        if (np.max(np.abs(target - self.q)) > 7.5 or np.linalg.norm(pose[:3] - self.pose[:3]) > 14 or
                _angle(_rotation(self.pose), _rotation(pose)) > 4.5):
            if depth >= 10:
                raise PlanRejected('Could not subdivide a bounded joint segment')
            self.joint_move((self.q + target) / 2, label, depth + 1)
            self.joint_move(target, label, depth + 1)
            return
        if np.any(target < LIMITS_DEG[:, 0]) or np.any(target > LIMITS_DEG[:, 1]):
            raise PlanRejected('Joint endpoint outside manufacturer nominal range')
        clearance, detail = self.segment_clearance(target)
        self.stages.append(dict(kind='move', label=label, joints_raw=np.rint(target * 1000).astype(int).tolist(),
                                pose_raw=np.rint(pose * 1000).astype(int).tolist(), clearance_mm=clearance,
                                clearance_detail=detail, speed_percent=5))
        self.q, self.pose = target, pose

    def tcp_move(self, target_tcp, target_rotation, label, spacing=12.):
        start_tcp, start_rotation = self.tcp(), _rotation(self.pose)
        steps = max(1, int(math.ceil(np.linalg.norm(target_tcp - start_tcp) / spacing)),
                    int(math.ceil(_angle(start_rotation, target_rotation) / 3.)))
        interpolation = Slerp([0., 1.], Rotation.from_quat([start_rotation.as_quat(), target_rotation.as_quat()]))
        for t in np.linspace(0., 1., steps + 1)[1:]:
            q = self.solve(start_tcp + (target_tcp - start_tcp) * t, interpolation(t), self.q)
            if np.max(np.abs(q - self.q)) > 25:
                raise PlanRejected('IK branch jump exceeds preparation policy')
            self.joint_move(q, label)

    def marker(self, kind, label, **extra):
        self.stages.append(dict(kind=kind, label=label, **extra))


def plan_pick(joints_raw, pose_raw, geometry):
    """Plan complete approach, close, lift, carry, release and retreat offline.

    Capture ``geometry`` at an opening where both real finger depth surfaces
    are visible, then open the empty gripper to 55 mm before executing the
    approach. Verify that the arm stayed at the captured pose while opening.
    The symmetric manufacturer's TCP centre does not depend on opening; the
    packaged collision hull encloses all openings from 0 through 70 mm. This
    assumes the physical jaws match that symmetric CAD model. No movement is
    authorized by a partial or rejected plan.
    """
    q, measured = _six(joints_raw, 'joints_raw'), _six(pose_raw, 'pose_raw')
    if np.any(q < LIMITS_DEG[:, 0]) or np.any(q > LIMITS_DEG[:, 1]):
        raise PlanRejected('Pick starts only inside nominal joint limits')
    initial = _Builder(q, measured, geometry)
    actual = initial.fk(q)
    if np.linalg.norm(actual[:3] - measured[:3]) > .5 or _angle(_rotation(actual), _rotation(measured)) > .1:
        raise PlanRejected('Fresh pose and official SDK FK disagree')
    urdf_pose = _link_transforms(q)[-1]
    if np.linalg.norm(urdf_pose[:3, 3] - actual[:3]) > .5:
        raise PlanRejected('Manufacturer collision URDF does not match SDK flange')
    tcp0 = actual[:3] + _rotation(actual).apply(TCP_LOCAL_MM)
    if np.linalg.norm(tcp0 - np.asarray(geometry['tcp_base_mm'])) > 3.:
        raise PlanRejected('Scene geometry was not derived from this robot pose')
    cube = np.asarray(geometry['cube_grasp_base_mm'], float)
    table = float(geometry['table_base_z_mm'])
    placements = geometry.get('place_candidates', [])
    if not np.all(np.isfinite(cube)) or not placements:
        raise PlanRejected('Missing finite object and visible empty placement geometry')
    yaw = math.degrees(math.atan2(cube[1], cube[0]))
    downward = Rotation.from_euler('xyz', [180., 30., yaw + 180.], degrees=True)
    initial.pose = actual
    initial.tcp_move(tcp0 + [0., 0., 100.], _rotation(actual), 'raise_before_unfolding')
    # Cross the folded wrist-over-base configuration with bounded joint-space
    # steps, not Cartesian IK branch selection. Compensating J5 approximately
    # preserves tool attitude; every resulting pose and mesh is checked.
    lifted_q = initial.q.copy()
    radial_axis = np.array([math.cos(math.radians(q[0])), math.sin(math.radians(q[0]))])
    for angle in np.arange(2., 40.1, 2.):
        target = lifted_q.copy(); target[1] += angle; target[4] -= angle
        if np.any(target < LIMITS_DEG[:, 0]) or np.any(target > LIMITS_DEG[:, 1]):
            raise PlanRejected('No nominal high-clearance unfolding posture')
        initial.joint_move(target, 'unfold_at_height')
        wrist = np.asarray(initial.fk_engine.CalFK(np.deg2rad(target).tolist())[3][:3])
        if float(wrist[:2] @ radial_axis) >= 45.:
            break
    else:
        raise PlanRejected('Unfolding did not clear the base-axis configuration')
    if initial.tcp()[2] - table < 100.:
        raise PlanRejected('Orientation preparation is too close to table')
    initial.tcp_move(initial.tcp(), downward, 'orient_above_table')
    prefix = initial.stages[:]
    preparation_q, preparation_pose = initial.q.copy(), initial.pose.copy()
    rejections = []
    # A raised final attempt deliberately permits missing the cube rather than
    # removing the table uncertainty/tracking reserve to make it reach lower.
    for grasp_height in (30., 33., 36., 39., 42.):
        for placement in placements:
            builder = _Builder(preparation_q, preparation_pose, geometry)
            builder.stages = list(prefix)
            builder.min_clearance = initial.min_clearance
            builder.checked_states = initial.checked_states
            try:
                place = np.asarray(placement['tcp_base_mm'], float)
                distance = float(np.linalg.norm(place[:2] - cube[:2]))
                if not np.all(np.isfinite(place)) or not 50. <= distance <= 120.:
                    raise PlanRejected('Placement must be a separate visible table patch')
                if float(placement.get('empty_patch_radius_mm', 0)) < 25.:
                    raise PlanRejected('Placement empty patch is too small')
                pre = cube.copy(); pre[2] = table + max(90., grasp_height + 50.)
                grasp = cube.copy(); grasp[2] = table + grasp_height
                raised = grasp + [0., 0., 40.]
                carry = place.copy(); carry[2] = raised[2]
                release = place.copy(); release[2] = grasp[2]
                builder.tcp_move(pre, downward, 'approach_above_cube')
                builder.marker('capture', 'pregrasp')
                builder.tcp_move(grasp, downward, 'descend_to_cube', spacing=5.)
                builder.marker('gripper', 'close', width_raw=0, effort_raw=300)
                builder.tcp_move(raised, downward, 'lift_cube', spacing=5.)
                builder.marker('capture', 'lifted')
                builder.tcp_move(carry, downward, 'carry_to_empty_patch')
                builder.tcp_move(release, downward, 'lower_to_place', spacing=5.)
                builder.marker('gripper', 'release', width_raw=55000, effort_raw=300)
                builder.marker('capture', 'placed')
                builder.tcp_move(carry, downward, 'retract_after_release', spacing=5.)
                moves = [s for s in builder.stages if s['kind'] == 'move']
                if len(moves) > 120:
                    raise PlanRejected('Whole pick requires more than 120 bounded moves')
                phase_summary = []
                for stage in moves:
                    if not phase_summary or phase_summary[-1]['label'] != stage['label']:
                        phase_summary.append(dict(label=stage['label'], moves=0,
                                                  minimum_clearance_mm=stage['clearance_mm']))
                    phase_summary[-1]['moves'] += 1
                    phase_summary[-1]['minimum_clearance_mm'] = min(
                        phase_summary[-1]['minimum_clearance_mm'], stage['clearance_mm'])
                    phase_summary[-1]['final_pose_raw'] = stage['pose_raw']
                    phase_summary[-1]['final_joints_raw'] = stage['joints_raw']
                return dict(start_joints_raw=list(joints_raw), start_pose_raw=list(pose_raw),
                            stages=builder.stages, geometry=geometry,
                            limits=dict(speed_percent=5, max_joint_segment_deg=7.5, max_flange_segment_mm=14,
                                        max_orientation_segment_deg=4.5, tracking_allowance_deg=TRACKING_DEG,
                                        max_move_stages=120, whole_execution_timeout_s=450,
                                        minimum_table_clearance_mm=MIN_CLEARANCE_MM,
                                        uncertainty_reserve_mm=_reserve(geometry)),
                            assessment=dict(grasp_tcp_table_height_mm=grasp_height, lift_mm=40., carry_mm=distance,
                                            minimum_sampled_clearance_mm=builder.min_clearance,
                                            checked_states=builder.checked_states,
                                            move_count=len(moves), phase_summary=phase_summary,
                                            may_miss_above_cube=True,
                                            collision_scope='manufacturer arm/gripper meshes against estimated horizontal table only',
                                            not_proven='continuous extrema, self-collision, camera, bracket, cables, other obstacles'),
                            source_sha=dict(planner=_sha(__file__), manufacturer_geometry=_model()[-1]),
                            rejected_alternatives=rejections)
            except PlanRejected as exc:
                rejections.append(dict(grasp_height_mm=grasp_height, placement=place.tolist() if 'place' in locals() else None,
                                       reason=str(exc)))
    raise PlanRejected('No complete grasp/place trajectory meets physical clearance and IK checks: ' + json.dumps(rejections))
