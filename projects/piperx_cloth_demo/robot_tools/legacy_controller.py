"""Explicit, expiring old-Piper controller coordinates on physical Piper X.

The installed X driver already inherits ordinary MOVE_L encoding from Piper.
This selects the controller FK interpretation, never changes physical identity,
flashes firmware or manufactures successful task evidence. The default limits
remain intersected; an audited current controller readback may select the
physical X J5 range through the explicit wrist-failure review.
Default supervised +Z increments are at most 1 mm. An explicit task-bound
2.5 or 5 mm profile has its own X-model envelope and shares the original budget.
This is not force or stopping control.
"""
import copy
import math
import time

from . import arms
from .reboot_startup import boot_identity

KEY = 'legacy_piper_controller_compatibility'
VERSION = 'piper_controller_on_x_up_v1'
MAX_STEPS = 20
LARGER_STEP = 'legacy_up_2p5mm_v1'
FIVE_MM_STEP = 'legacy_up_5mm_v1'
TASK_BUDGET = 'original_task_3h_1000_v1'


def effective_deadline(value):
    """An explicitly granted later window preserves the parent and all counts."""
    window=value.get('continuation_window')
    if window is None:
        return value['expires_at_unix_s']
    required={'schema','source','statement','starts_at_unix_s','expires_at_unix_s','reviewed_failure_run_id'}
    if (not isinstance(window,dict) or set(window)!=required
            or window.get('schema')!='legacy_explicit_continuation_window_v1'
            or window.get('source')!='user' or not isinstance(window.get('statement'),str)
            or len(window['statement'].strip())<10
            or value.get('task_budget') is None
            or window.get('reviewed_failure_run_id')!=value.get('reviewed_wrist_limit_failure',{}).get('run_id')
            or not window.get('reviewed_failure_run_id')
            or any(type(window.get(k)) not in (int,float) or not math.isfinite(window[k])
                   for k in ('starts_at_unix_s','expires_at_unix_s'))
            or not value['expires_at_unix_s'] <= window['starts_at_unix_s'] <= time.time()
            or window['expires_at_unix_s']-window['starts_at_unix_s']!=10800):
        raise RuntimeError('Explicit continuation window invalid; original budget is not reset')
    return window['expires_at_unix_s']


def step_limits(compatibility=None):
    larger = (compatibility or {}).get('step_profile')
    if larger is not None and (not isinstance(larger, dict) or larger.get('name') not in (LARGER_STEP, FIVE_MM_STEP)):
        raise RuntimeError('Unknown legacy step profile')
    five = larger is not None and larger['name'] == FIVE_MM_STEP
    return dict(nominal_step_m=0.005 if five else (0.0025 if larger else 0.001),
                minimum_nominal_step_m=0.005 if five else 0.0,
                start_reference_band_m=0.0005,
                physical_translation_m=0.008 if five else (0.0045 if larger else 0.002),
                physical_rotation_rad=0.024 if five else (0.012 if larger else 0.01),
                physical_lateral_m=0.001, physical_downward_m=0.0005,
                joint_change_rad=0.025)


def policy(profile, arm):
    value = profile.get(KEY)
    if value is None:
        return None
    required = {'version', 'arm', 'source', 'statement', 'boot_id',
                'refused_run_id', 'refused_sha256', 'expires_at_unix_s'}
    if (not isinstance(value, dict) or not required <= set(value)
            or set(value) - required - {'reviewed_start_reference_refusal', 'step_profile', 'task_budget', 'reviewed_wrist_limit_failure', 'continuation_window', 'reviewed_prediction_age_refusal', 'reviewed_live_start_refusal'}
            or value['version'] != VERSION or arm != value['arm']
            or arm != 'left' or value['source'] != 'user'
            or not isinstance(value['statement'], str) or not value['statement'].strip()
            or value['boot_id'] != boot_identity()['boot_id']
            or type(value['expires_at_unix_s']) not in (int, float)
            or not math.isfinite(value['expires_at_unix_s'])
            or not time.time() < effective_deadline(value)
            or not isinstance(value['refused_run_id'], str)
            or not value['refused_run_id'].startswith('single_supervised_move_')
            or not isinstance(value['refused_sha256'], str)
            or len(value['refused_sha256']) != 64
            or any(c not in '0123456789abcdef' for c in value['refused_sha256'])
            or profile.get('physical_model_confirmation', {}).get(arm) != 'piper_x'
            or any(cfg.get('model') != 'piper_x' for cfg in profile['arms'].values())):
        raise RuntimeError('Legacy controller compatibility identity/authorization expired or invalid')
    for reference_key in ('reviewed_start_reference_refusal','reviewed_prediction_age_refusal','reviewed_live_start_refusal'):
        review = value.get(reference_key)
        if review is not None and (not isinstance(review, dict)
                or set(review) != {'run_id','sha256'}
                or not isinstance(review['run_id'], str)
                or not review['run_id'].startswith('single_supervised_move_')
                or not isinstance(review['sha256'], str) or len(review['sha256']) != 64
                or any(c not in '0123456789abcdef' for c in review['sha256'])):
            raise RuntimeError('Invalid zero-TX start-reference review')
    larger = value.get('step_profile')
    if larger is not None and (not isinstance(larger, dict)
            or set(larger) != {'name', 'source', 'statement', 'boot_id', 'expires_at_unix_s'}
            or larger.get('name') not in (LARGER_STEP, FIVE_MM_STEP) or larger.get('source') != 'user'
            or not isinstance(larger.get('statement'), str) or not larger['statement'].strip()
            or larger.get('boot_id') != value['boot_id']
            or type(larger.get('expires_at_unix_s')) not in (int, float)
            or not math.isfinite(larger['expires_at_unix_s'])
            or larger['expires_at_unix_s'] > value['expires_at_unix_s']
            or not (time.time()<larger['expires_at_unix_s']
                    or (larger['expires_at_unix_s']==value['expires_at_unix_s']
                        and time.time()<effective_deadline(value)))):
        raise RuntimeError('Legacy larger-step authorization expired or invalid')
    task_budget(value)
    if value.get('reviewed_wrist_limit_failure') is not None:
        from .legacy_wrist_review import validate_reference
        validate_reference(value['reviewed_wrist_limit_failure'])
    return copy.deepcopy(value)


def task_budget(compatibility):
    """Explicit original task budget; no new clock, counter or automatic renewal."""
    value = (compatibility or {}).get('task_budget')
    if value is None:
        return None
    if (not isinstance(value, dict)
            or set(value) != {'name', 'source', 'statement', 'started_at_unix_s',
                             'max_duration_s', 'max_steps'}
            or value.get('name') != TASK_BUDGET or value.get('source') != 'user'
            or not isinstance(value.get('statement'), str) or not value['statement'].strip()
            or type(value.get('max_steps')) is not int or value['max_steps'] != 1000
            or type(value.get('max_duration_s')) is not int or value['max_duration_s'] != 10800
            or type(value.get('started_at_unix_s')) not in (int, float)
            or not math.isfinite(value['started_at_unix_s'])
            or not value['started_at_unix_s'] <= time.time() < effective_deadline(compatibility)
            or value['started_at_unix_s'] + value['max_duration_s'] != compatibility['expires_at_unix_s']):
        raise RuntimeError('Original legacy task budget invalid or expired')
    return copy.deepcopy(value)


def check_up_target(start, target, rotation_distance, compatibility=None):
    # Absolute targets are proposed from the preceding observed endpoint.
    # Reuse the existing stationary 0.5 mm / 0.003 rad observation bands,
    # rather than requiring unchanged feedback down to encoder precision.
    dz = target[2] - start[2]
    limits = step_limits(compatibility)
    if (not 0 < dz <= limits['nominal_step_m'] + limits['start_reference_band_m']
            or dz < limits['minimum_nominal_step_m'] - limits['start_reference_band_m']
            or math.dist(start[:2], target[:2]) > 0.0005
            or rotation_distance(start, target) > 0.003):
        raise RuntimeError('Legacy +Z target exceeds selected step or existing 0.5 mm / 0.003 rad start-reference bands')


def same_authorization(a, b):
    # A step-profile change never starts a new sequence or erases used steps.
    # Parent identity, original refusal, boot and deadline must still match.
    extensions = {'reviewed_start_reference_refusal', 'step_profile', 'task_budget', 'reviewed_wrist_limit_failure', 'continuation_window', 'reviewed_prediction_age_refusal', 'reviewed_live_start_refusal'}
    return (isinstance(a, dict) and isinstance(b, dict)
            and {k:v for k,v in a.items() if k not in extensions}
            == {k:v for k,v in b.items() if k not in extensions})


def model_delta(profile, start_q, current_q, rotation_distance, compatibility=None):
    start = arms.vendor_fk('piper_x', start_q, profile['sdk_path'])['pose_m_rad']
    current = arms.vendor_fk('piper_x', current_q, profile['sdk_path'])['pose_m_rad']
    delta = [b-a for a,b in zip(start[:3], current[:3])]
    angle = rotation_distance(start, current)
    limits = step_limits(compatibility)
    if (math.dist(start[:3], current[:3]) > limits['physical_translation_m']
            or math.hypot(delta[0], delta[1]) > limits['physical_lateral_m']
            or delta[2] < -limits['physical_downward_m'] or angle > limits['physical_rotation_rad']
            or max(abs(a-b) for a,b in zip(start_q, current_q)) > limits['joint_change_rad']):
        raise RuntimeError('Physical Piper X model left bounded legacy-controller trial envelope; no new target')
    return {'translation_m': delta, 'rotation_rad': angle,
            'source': 'physical-model FK of measured joints; not measured object displacement'}
