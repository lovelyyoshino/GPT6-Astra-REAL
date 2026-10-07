"""Fixed SDK tools. Physical execution is subject to independent commissioning."""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import time
import uuid
from pathlib import Path

from .preview import PLAN_SCHEMA, preview_plan, render_intent_svg, validate
from .execution import ExclusiveExecution, Journal, readiness, run_plan


def _object(properties=None, required=None):
    return {"type": "object", "additionalProperties": False,
            "properties": properties or {}, "required": required or []}


def _tool(name, description, schema, read_only=True):
    return {"name": name, "description": description, "inputSchema": schema,
            "annotations": {"readOnlyHint": read_only, "destructiveHint": False,
                            "idempotentHint": read_only, "openWorldHint": False}}


EXECUTION_REF_SCHEMA = _object({
    "execution_id": {"type": "string", "pattern": "^execution_[0-9a-f]{32}$"},
    "preview_id": {"type": "string", "pattern": "^preview_[0-9a-f]{32}$"},
})


TOOL_SCHEMAS = [
    _tool("robot_single_arm_move_once", "One explicitly planned and attended MOVE_L on one enabled healthy arm and jaw, with the other arm passive in its original mode 0/1/2 and seven enable states. Absolute SDK flange target in that arm base, metres/RPY radians. Fixed 1%, translation <=30 mm and SO(3) rotation <=0.05 rad. Selected joint feedback uses a bounded 0.003 rad observation band around unchanged nominal limits; larger initial offsets refuse before send with a diagnostic recovery candidate, never automatic recovery. This software band is not calibrated accuracy or an IK/path certificate. Passive joint limit violations are recorded, never repaired or granted motion permission. Both arms remain fresh, healthy, teaching OFF; three-second stable baseline and post-dispatch observation. All passive-arm and jaw TX forbidden. One exact 0x151+0x152/153/154 sequence, no retry, enable, reset, stop, disable or fallback. Caller must inspect full link/tool/camera/cable path and remaining motion space. Reports stability, raw arrival and target errors separately; ok does not certify arrival, grasp, collision-free path or holding stop. Full-plan backend remains blocked.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "target_pose_m_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}}}, ["arm", "target_pose_m_rad"]), False),
    _tool("robot_single_arm_gripper_once", "One attended jaw target on one healthy CAN-controlled arm with all six joints and jaw already enabled. Other arm retains its initial mode 0/1/2, seven enable states and stationary pose; its joint-limit violations are recorded without commanding it. Both arms must remain fresh, healthy, stationary and teaching OFF; nominal joint violations remain reported. A 0.003 rad feedback band is allowed; this jaw-only operation may also preserve an initial J2-below-minimum or J3-above-maximum offset up to 0.1 rad, frozen for this single action with the original stationary drift checks. No arm targets or cross-session allowance. Width 0..0.055 m, manufacturer nominal force exactly 0.2. One exact 0x159, enable=1 and set_zero=0, at most one frame; all arm and passive-jaw TX forbidden. Three-second fresh stable baseline and final observation. Caller checks finger/contact clearance. Stable aperture or ok never proves grip/contact; inspect width error and images separately. No retry, reset, stop, disable, zero calibration or full-plan gate unlock.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "width_m": {"type": "number", "minimum": 0, "maximum": 0.055}, "nominal_force_N": {"type": "number", "enum": [0.2]}}, ["arm", "width_m", "nominal_force_N"]), False),
    _tool("robot_move_once", "One explicitly MODEL-planned, operator-supervised finite MOVE_L trial; not a production trajectory or validated stopping service. Absolute SDK flange target in selected arm base, metres and RPY radians. Translation <=30 mm and rotation <=0.05 rad from fresh feedback, fixed speed 1%, one continuous manufacturer mode+three-pose-frame call. Both arms/jaws must already be enabled, healthy, legal, and observed stable for 3 s. Caller must first assess the ENTIRE intended motion including arm links, camera, fingers, cables and possible pending targets; numerical endpoint bounds are not collision or IK path proof. Preserves raw not-at-target flags. Returns observed stability, pose error and controller arrival separately, never calls stability cancellation. No task perception, automatic target repair, enable, retry, fallback target, reset, stop or disable. Failure stops later software dispatch only; disconnect is not a physical stop. Use only for the currently authorized attended experiment; default full-plan backend remains blocked.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "target_pose_m_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}}}, ["arm", "target_pose_m_rad"]), False),
    _tool("robot_gripper_once", "One explicit low-nominal-force jaw target on one arm during the authorized attended experiment. width_m is 0..0.055 m and nominal_force_N is fixed at manufacturer SDK value 0.2, not independently calibrated physical force. Requires both arms/jaws already enabled, healthy, legal and stationary in fresh feedback; no arm command, no zero calibration, no enable/reset/stop/retry. Exactly one manufacturer 0x159 command. Caller establishes finger/contact clearance before closing. Reports width, controller feedback and observed stability separately; fabric contact, nonzero stable width or closure NEVER proves a grasp. Model must inspect images/cloth response before lifting or continuing.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "width_m": {"type": "number", "minimum": 0, "maximum": 0.055}, "nominal_force_N": {"type": "number", "enum": [0.2]}}, ["arm", "width_m", "nominal_force_N"]), False),
    _tool("robot_bounded_joint_step", "One supervised commissioning adjustment, not task execution or cancellation. MODEL supplies six radians: only J2/J3 may change, at most 0.025 rad each; encoded targets must be legal and J2/J3 at least 0.010 rad inside limits. Other axes must encode identically to fresh feedback. Caller confirms unloaded arms, full-chain/attachment clearance and live attendance. attachment_radius_m bounds the entire tool/camera/bracket from the flange; available_clearance_m is minimum surrounding free space. Manufacturer MDH remaining translations plus attachment and fixed 60 mm body allowance conservatively bound ALL six axes' target excursions plus 0.003 rad tracking tolerance; bound must fit clearance minus 5 mm. This assumption-dependent envelope is not collision certification and does not cover cached/partial target activation. Selected arm must be healthy and genuinely stable for 3 s; motion_status=1 may mean an unknown pending target and is preserved, never called stopped. Other arm requires motion_status=0 and receives no TX. Manufacturer FK endpoint <=15 mm. One public move_j at 1% sends exactly 151/155/156/157, no retry/reset/stop/disable. Success requires fresh motion_status=0, near-target feedback and 3 s stability; failure requires site attention. No general hold qualification or task gate unlock.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "target_joints_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}}, "attachment_radius_m": {"type": "number", "exclusiveMinimum": 0}, "available_clearance_m": {"type": "number", "exclusiveMinimum": 0}}, ["arm", "target_joints_rad", "attachment_radius_m", "available_clearance_m"]), False),
    _tool("robot_describe", "Read robot profile, task, embedded robot guide/SDK audit/experiment policy and exact tool capabilities. No device access. Model/firmware/frames are unverified; execution is unavailable. Call before planning; no file reader is needed.", _object()),
    _tool("robot_read_state", "Read passive manufacturer SDK arm AND gripper feedback from BOTH bound arms. No enable, query TX, reset, stop or motion. Returns per-fragment age, cross-arm skew, errors and partial states. Complete means telemetry received, not motion permission.", _object()),
    _tool("robot_request_can_control", "Operator-requested administrative CAN control takeover for both bound arms. May affect physical holding/motion despite sending no target. Requires fresh, stationary, enabled, fault-free feedback and teaching recording OFF. Keeps current P/J/L motion selection and requests CAN control at 1% with at most one exact 0x151 frame per arm, left then right; already-CAN arms are not retransmitted. Monitors both arms before and after each change, saves journal, never retries or sends targets, enable, reset, stop or recovery. Failure stops subsequent dispatch, not proof of physical stopping. Does NOT unlock grasp execution or certify a holding stop. Invoke only for explicit mode-takeover authorization.", _object(), False),
    _tool("robot_startup_arms", "Operator-requested startup after both arms have been power-cycled: requires both in standby ctrl_mode=0, all six joints AND grippers explicitly disabled, fresh fault-free stationary feedback and teaching OFF. First requests CAN mode once per arm at 1% preserving P/J/L, confirms both remain disabled; then calls manufacturer enable(255) once per arm and verifies fresh driver feedback. Only exact 0x151 and 0x471 frames are allowed, at most one of each per arm. No position, joint or gripper targets, no reset/stop/disable/home/retry. Gripper enable may change as a side effect of vendor all-motor enable and is reported, not assumed ready. Activation can cause physical motion; continuous drift checks stop later dispatch but cannot guarantee physical stopping. Partial/uncertain startup must not be replayed. Shares execution lock, saves request/events/result; does NOT unlock grasp execution or validate hold. Requires explicit startup authorization and current scene inspection.", _object(), False),
    _tool("robot_startup_arm", "Operator-authorized startup of ONE explicit arm after reboot. Selected arm must begin in standby ctrl_mode=0 with all six joints AND gripper disabled. Both arms must provide fresh, healthy, stationary feedback, teach_status=0 and motion_status=0. The other arm is observed only: its initial mode (0/1/2), enable flags, pose, joints and jaw must remain unchanged within existing drift limits; ALL transmission to it is forbidden. Selected arm only: one exact 0x151 at 1% preserving P/J/L, fresh CAN-mode confirmation while disabled, then one manufacturer enable(255) encoded as 0x471; verify fresh individual driver feedback. No joint/flange/jaw targets, no 0x159, calibration, reset, stop, disable, recovery or automatic retry. Mode/enable can activate physical motion; failure/disconnect is not a stop. All-motor enable may affect gripper enable status; report actual feedback, never assume grasp readiness. Partial/uncertain startup must not be replayed. Shares execution lock and durable request/events/result. Does not repair joint limits, grant task motion permission or validate holding stop. Requires explicit startup authorization and current scene inspection.", _object({"arm": {"type": "string", "enum": ["left", "right"]}}, ["arm"]), False),
    _tool("robot_prepare_gripper", "Operator-supervised EMPTY-gripper initialization on ONE arm at its measured current width; accepts only arm, never a caller-selected width or force. Selected arm must already be CAN-controlled with six healthy enabled joints. Both arms need fresh healthy feedback, teaching OFF, motion_status=0 and a measured stable baseline. Other arm may retain mode 0/1/2 and any known initial enable flags; its state is monitored and ALL TX to it is forbidden. If selected jaw is disabled, send at most one exact manufacturer 0x159 at initial measured width within 0..0.070 m, fixed nominal force 0.2, status=1, set_zero=0. This is a real position/enable command and may move fingers, not a pure enable. Target must remain within 0.5 mm of fresh pre-send width; require fresh enabled feedback and final width error <=1 mm. Strict 0.003 rad joint drift and 0.003 rad SO(3) rotation distance using manufacturer RPY convention, 2 mm position/jaw envelope; individual Euler component differences are diagnostic only, no right-J4 tolerance exception. Already enabled is observed without retransmission. Joint values outside nominal task limits remain recorded and do not confer motion permission; this tool does not move or recover arm joints. No arm target, mode, enable(255), calibration, retry, reset, stop or disable. Shares lock and durable request/events/result; preserves all existing task gates and the separate dual-gripper contract. Requires clear empty fingers and live site supervision.", _object({"arm": {"type": "string", "enum": ["left", "right"]}}, ["arm"]), False),
    _tool("robot_home_arm", "Explicitly operator-requested return of ONE near-home arm to its existing six zero joint coordinates; NOT encoder zero calibration and not a grasp trajectory. Only arm is accepted; fixed target [0,0,0,0,0,0], fixed speed 1%, one continuous manufacturer MOVE_J call containing exactly 151/155/156/157, no other actuator commands. Caller must inspect the full arm, fingers, wrist camera/bracket, cables and entire intended sweep under current live attendance; endpoint FK and monitored joint boxes are NOT collision/path/stop qualification. Selected arm must be CAN mode 1 with six enabled healthy joints; passive arm may retain mode 0/1/2 with known unchanged seven enable flags, ALL TX to it and both jaws forbidden. Both jaws keep their width and enable state. Requires three-second advancing stable baseline, known teaching/motion state, <=100 ms fragment age including local validation time and FK matching feedback. Near-home start bounds per axis are [45,10,10,15,30,15] degrees, only J2/J3 may begin outside nominal bounds by at most five degrees; record these exceptions without changing controller limits. Observe each joint within its independent initial-to-zero interval plus 0.01 rad, forbid initial boundary violation deepening over 0.003 rad, continually verify FK/telemetry. Maximum observation 120 s; success requires fresh post-send J-mode, motion_status=0, each joint within 0.003 rad of zero, target FK agreement and three seconds stable advancing feedback. Already near zero is observed without retransmission. Reports near-zero arrival and strict nominal-limit membership separately, never grants task-motion or holding-stop qualification. No jaw target, enable, set_zero, retry, automatic home replay, reset, stop or disable, even on partial send or timeout. Shares lock and durable request/events/result.", _object({"arm": {"type": "string", "enum": ["left", "right"]}}, ["arm"]), False),
    _tool("robot_inspect_firmware", "Query each bound arm's actual firmware through manufacturer SDK. Sends at most one exact non-actuating 0x4AF query (DLC=1,data=01) per arm; never sends mode, enable, targets, reset or stop. Requires fresh healthy CAN-controlled enabled joints; grippers may be disabled. Records drift without claiming stationary or motion permission. Saves manufacturer result, fresh raw response fragments, SDK provenance and before/after feedback. Does not automatically change profile. Query failure is unknown, never evidence of a different firmware. Uses exclusive execution lock; no retries.", _object(), False),
    _tool("robot_inspect_joint_limits", "Read both controllers' stored joint angle/velocity limits through manufacturer SDK, sequentially joints 1..6, at most one exact non-actuating 0x472 query per joint. Requires fresh matching 0x473 response; saves raw bytes and vendor-decoded values. No settings, target, mode, enable, stop or automatic retry. Feedback drift is recorded, not interpreted as motion permission. Returned settings are controller configuration, not proof of mechanical clearance, zero calibration or safe reachability. Velocity unit discrepancy between vendor comments and parser is reported, not silently corrected. Uses exclusive execution lock and durable journal.", _object(), False),
    _tool("robot_prepare_grippers", "Operator-requested EMPTY-gripper preparation with both arms already CAN-controlled and enabled. For each disabled gripper, sends its newly observed current width with nominal SDK force=0.2 through move_gripper_m once, then verifies enable and width feedback. This is a real position command and may cause small jaw motion, NOT a read-only or pure-enable operation. Target must be 5..70 mm and stay within 0.5 mm of latest pre-send width. Already-enabled grippers are not retransmitted. Only exact 0x159 frames, one per side; no zeroing, arm target, mode, enable(255), reset, stop, disable or retry. Monitors both arms and jaws, stops later dispatch on drift/error. Does not validate force calibration, grasp or arm holding stop; no grasp gate unlock. Requires empty jaws and clear fingers established by current scene/operator evidence.", _object(), False),
    _tool("robot_recover_joint_boundary", "Commissioning-only, single bounded joint-position recovery for one explicit arm. Caller supplies all six target angles in radians and must establish clearance of the full arm, tool, camera and cables, plus live operator supervision. Only joints slightly outside verified manufacturer/controller limits may return to their nearest boundary; remaining axes stay at observed values within feedback tolerance. Default standard profile retains each change <=0.05 rad and enabled jaws. Explicit startup_j2_j3 profile accepts only J2 below min/J3 above max, <=0.10 rad to the nearest exact boundary; other axes stay within 0.003 rad of frozen and fresh feedback. Requires attachment_radius_m bounding BOTH arms attachments and available_clearance_m as minimum surface gap to obstacles, other arm and non-adjacent bodies; twice the all-axis MDH sweep (including 60 mm body allowance and 0.003 rad tracking tolerance) must fit clearance minus 5 mm. Both arms joints enabled; jaws may be disabled with frozen known enable bits and complete healthy stable feedback. Three-second baseline, >=20 advancing samples, motion_status=0 before dispatch, all feedback <=50 ms including FK computation time. Fixed 1% speed and vendor FK endpoint displacement <=15 mm for both profiles. These bounds are NOT a whole-path collision proof. One manufacturer move_j sends 0x151 plus 0x155/156/157, four non-atomic frames, no zero calibration, reset, stop, disable or retry. Mode/partial sends may activate old or mixed targets; failure requires operator attention. Observes fresh arrival for 3 s, logs actual values; no holding-stop or task-motion qualification. Never use to bypass a failed task plan.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "target_joints_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}}, "recovery_profile": {"type": "string", "enum": ["standard", "startup_j2_j3"], "default": "standard"}, "attachment_radius_m": {"type": "number", "exclusiveMinimum": 0}, "available_clearance_m": {"type": "number", "exclusiveMinimum": 0}}, ["arm", "target_joints_rad"]), False),
    _tool("robot_qualify_linear_hold", "Commissioning-only local experiment on one explicit legal, unloaded arm: at fixed 1% MOVE_L continuously send manufacturer mode+target for +6 mm base Z, unchanged orientation, then once replace it with freshly observed XYZ after >=1 mm progress while >=3 mm remain. Caller must verify the ENTIRE original path clearance and operator supervision. At most seven exact frames. Must demonstrate old target cancelled and replacement stable >=3 s. Optional prior_mode_only_run_id allows one durably claimed trial after THIS platform's proven mode-only failure, same bindings/arm/pose, exact recorded 0x151, no prior pose target, only status4/motion1 with genuinely stationary feedback and all other health checks. In that receipt-bound trial only, after all four initial frames are sent continuously, status4 may persist for at most 200 ms while awaiting target processing: RX timestamps are not target acknowledgements. Feedback must remain within 50 ms, with displacement <=0.5 mm, joint change <=0.003 rad and orientation change <=0.02 rad from the final pre-send sample; all other guards remain. Fresh normal status0 is required before any replacement, and once observed, regression to status4 is refused. This is not general fault suppression. Failure never retries or sends reset/stop/disable. Success is local evidence, not general stopping qualification or task unlock. Inactive arm is observed, never commanded.", _object({"arm": {"type": "string", "enum": ["left", "right"]}, "prior_mode_only_run_id": {"type": "string", "pattern": "^linear_hold_[0-9a-f]{32}$"}}, ["arm"]), False),
    _tool("robot_observe", "Capture front + left wrist + right wrist RGB and optional aligned depth, then read both arms. Returns images, timestamped files and observation id. No object detector or coordinates inferred. Streams are NOT exposure synchronized. Partial failure and invalid depth stay unknown. No arm commands. Saves a new run directory.", _object({"include_depth": {"type": "boolean"}}), False),
    _tool("robot_depth_at_pixels", "Read exact aligned depth samples in metres at MODEL-selected RGB pixels from a saved observation. No detection, interpolation, segmentation or robot-coordinate transform. Invalid depth returns null, never free space. Includes capture age; old samples cannot authorize motion. No device access.", _object({"observation_id": {"type": "string", "pattern": "^obs_[0-9a-f]{32}$"}, "camera": {"type": "string", "enum": ["front", "left_wrist", "right_wrist"]}, "pixels_uv": {"type": "array", "minItems": 1, "maxItems": 32, "items": {"type": "array", "minItems": 2, "maxItems": 2, "items": {"type": "integer", "minimum": 0, "maximum": 639}}}}, ["observation_id", "camera", "pixels_uv"])),
    _tool("robot_fk", "Pure manufacturer's forward kinematics for six joint angles in radians. Model must be explicitly selected; physical model is unverified. Returns flange xyz metres / RPY radians and nominal limit violations. NOT inverse kinematics, reachability test, collision check or motion.", _object({"model": {"type": "string", "enum": ["piper", "piper_x", "piper_h", "piper_l"]}, "joints_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}}}, ["model", "joints_rad"])),
    _tool("robot_preview_plan", "Validate a MODEL-authored whole plan against a saved current observation. Targets use each arm's own base, SDK flange, metres/radians. Paired means coordination intent only. Saves unchanged plan, checks and stage chart. Unknown IK/path/collision/hold checks never become a safe verdict. NO motion and NO target generation or correction.", _object({"plan": PLAN_SCHEMA}, ["plan"]), False),
    _tool("robot_submit_plan", "Execute an unchanged saved MODEL plan through vendor SDK only after commissioning, recent complete observation and fresh healthy stationary feedback checks. Current site is BLOCKED: no verified hold stop. One-shot preview id, exclusive lock, no automatic enable/retry/home/reset. Paired stages mean near-time dispatch + barriers, NOT synchronized paths. Exact-width gripper result does not prove a grasp. Synchronous; another process can request cancellation.", _object({"preview_id": {"type": "string", "pattern": "^preview_[0-9a-f]{32}$"}}, ["preview_id"]), False),
    _tool("robot_check_execution", "Report missing physical execution prerequisites without opening devices or transmitting. Configuration flags cannot override the backend's unvalidated holding stop. Does not grant motion permission.", _object()),
    _tool("robot_execution_status", "Read persisted execution status and last event. Supply exactly one execution_id OR the already-known preview_id, so a second process can check an in-flight blocking submit. Interrupted records are uncertain, never auto-resumed. No hardware access.", EXECUTION_REF_SCHEMA),
    _tool("robot_cancel_execution", "Persist a cancellation request. Supply exactly one execution_id OR known preview_id. Worker polls then requests only a validated holding stop; response is NOT proof of stopping. Serial MCP requires another process during blocking submit; Ctrl-C is caught. No emergency stop/disable/reset fallback.", EXECUTION_REF_SCHEMA, False),
]


# Persistent pair tools are separate from the legacy full-plan backend. The
# default real adapter exposes retained stationary observation, not contact hold.
PAIR_ID_SCHEMA = {"type": "string", "pattern": "^[A-Za-z0-9_-]{1,96}$"}
PAIR_STEP_SCHEMA = _object({
    "event_id": PAIR_ID_SCHEMA, "observation_id": {"type": "string", "minLength": 1},
    "peer_receipt_id": {"type": "string", "minLength": 1},
    "arm": {"type": "string", "enum": ["left", "right"]},
    "kind": {"type": "string", "enum": ["move", "joint", "gripper"]},
    "operation": {"type": "string", "enum": ["approach", "align", "transport", "release_retreat",
        "return_reference", "observer_reposition", "grip_supported", "grip_test", "extract_segment",
        "insert_segment", "rotate_segment", "push_segment", "wipe_segment", "sweep_segment"]},
    "target_pose_m_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}},
    "target_joints_rad": {"type": "array", "minItems": 6, "maxItems": 6, "items": {"type": "number"}},
    "width_m": {"type": "number", "minimum": 0, "maximum": 0.055},
    "grasp_object_id": PAIR_ID_SCHEMA,
    "release_support_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "release_support_relation": {"type": "string", "enum": ["independent_support_present"]},
    "release_retreat_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "admission_mode": {"type": "string", "enum": ["metric_geometry", "rgb_supervised"]},
    "motion_profile": {"type": "string", "enum": ["coarse_approach"]},
    "far_from_target_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "unloaded_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "corridor_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "loaded_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    "source_object_id": PAIR_ID_SCHEMA,
    "target_object_id": PAIR_ID_SCHEMA,
}, ["event_id", "observation_id", "peer_receipt_id", "arm", "kind", "operation"])
TOOL_SCHEMAS.extend([
    _tool("robot_pair_confirm_loaded_response", "Zero-TX confirmation after one complete supervised loaded joint segment. Requires fresh saved RGB after that segment and current dual-arm feedback. Records observed plug retention, the left-held power strip stationary on its table support (not the airborne plug), and source/target relation; it does not prove force, hardware stop or whole-task success. Unknown/adverse response faults; two cumulative no-progress observations block further segments without resetting budgets. A pending response blocks all new targets and openings. Caller cannot supply poses, receipt hashes or permission booleans.", _object({
        "event_id": PAIR_ID_SCHEMA, "action_event_id": PAIR_ID_SCHEMA,
        "observation_id": {"type": "string", "minLength": 1},
        "visual_description": {"type": "string", "minLength": 1, "maxLength": 4000},
        "response": {"type": "string", "enum": ["progress", "no_progress", "unknown", "adverse"]},
        "object_relation": {"type": "string", "enum": ["retained_between_fingers", "slipped", "unknown"]},
        "support_relation": {"type": "string", "enum": ["table_supported_stationary", "moved", "unknown"]},
        "task_relation": {"type": "string", "enum": ["source_engaged", "source_separated", "target_aligned", "target_partly_inserted", "target_seated", "unknown"]},
    }, ["event_id", "action_event_id", "observation_id", "visual_description", "response", "object_relation", "support_relation", "task_relation"]), False),
    _tool("robot_pair_publish_geometry", "Import an existing measured site record set into the current host scene. Accepts only a record-set ID under the fixed local site-record directory, never caller bounds, file paths or permission flags. Host binds its owner, connections, profile and saved RGB; structured installed-attachment/workspace records and current physical clearance measurements supply conservative bounds. Zero CAN TX; does not acquire measurements, renew old clearance, prepare devices, establish target cache or authorize motion. Missing actual records remain a specific source gap.", _object({
        "observation_id": {"type": "string", "minLength": 1},
        "record_set_id": {"type": "string", "pattern": "^[A-Za-z0-9_-]{1,96}$"},
    }, ["observation_id", "record_set_id"]), False),
    _tool("robot_pair_open", "Explicit attended persistent pair connection. connection_mode=ready preserves the enabled healthy CAN-pair entry; prepare connects with zero TX and observes known initial modes/enable flags without granting task motion. Use same-host preparation tools for missing empty-jaw readiness, then promote_ready; neither connection mode enables arm joints or sends targets. Requires long-running stdio, not --call. Keeps one exclusive owner and frozen run budget. Defaults remain 128 steps/900 seconds; larger budgets up to 500/3600 require the exact already activated explicit-user-budget execution epoch. Ordinary new runs cannot request that expansion. User clearance statement is current task context, not a collision certificate. Both roles are task arms. Force-controlled contact and verified physical stopping remain unavailable.", _object({
        "run_id": PAIR_ID_SCHEMA, "task_id": {"type": "string", "minLength": 1, "maxLength": 96},
        "workspace_clearance_statement": {"type": "string", "minLength": 1, "maxLength": 2000},
        "max_steps": {"type": "integer", "minimum": 1, "maximum": 500},
        "max_duration_s": {"type": "number", "minimum": 1, "maximum": 3600},
        "connection_mode": {"type": "string", "enum": ["ready", "prepare"]},
    }, ["run_id", "task_id", "workspace_clearance_statement"]), False),
    _tool("robot_pair_prepare_gripper", "Prepare one currently empty jaw on the same persistent connection. Requires a fresh saved RGB scene and the outer model's empty-jaw observation, stored as semantic testimony rather than machine proof. Selected arm needs CAN control and six enabled joints. At most one exact measured-width 0x159 at fixed nominal force 0.2; may move fingers. Already-enabled jaw sends nothing. No caller width/force, joint target, mode change, reconnect or retry. Durable event uses the original budget; poll pair_status. Task motion still requires explicit promote_ready.", _object({
        "event_id": PAIR_ID_SCHEMA, "observation_id": {"type": "string", "minLength": 1},
        "arm": {"type": "string", "enum": ["left", "right"]},
        "empty_jaw_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    }, ["event_id", "observation_id", "arm", "empty_jaw_observation"]), False),
    _tool("robot_pair_inspect_joint_limits", "Query each of the twelve joint limits once using the persistent host's existing connections. Sends fixed 0x472 queries only, no mode, enable, target or configuration. Requires the existing healthy stationary feedback contract and no unresolved grasp. Disabled jaws are allowed. Records independent fresh raw responses and same-connection source bindings; complete successful capture may populate the host's internal limit source. No caller source dictionaries, retry, reconnect, movement permission or first-target cache. Uses one durable event from the frozen budget; poll pair_status.", _object({
        "event_id": PAIR_ID_SCHEMA,
    }, ["event_id"]), False),
    _tool("robot_pair_promote_ready", "Check task readiness on the existing preparation connection with zero TX. Preserve original stationary anchors, modes and prepared jaw targets. Known missing readiness returns preparation_required; no automatic enable, mode change, reconnect or target. A successful ready observation does not establish first joint-target history, geometry, loaded contact or physical stopping.", _object(), False),
    _tool("robot_pair_initialize_joint_target", "Explicit attended unloaded first joint target on the SAME pair connections, usable in preparation or ready mode. Host resolves current RGB semantics, official PiPER X model and current controller limits; selects a current-position seed or the existing bounded J2/J3 startup boundary target. Default admission_mode=metric_geometry requires actual geometry. Explicit admission_mode=rgb_supervised instead requires corridor_observation describing the current whole-arm, attachment, cable and table corridor plus the recorded user clearance statement; this is attended visual testimony, not metric clearance or absolute-workspace certification. It does not fall back automatically or qualify subsequent ordinary/loaded motion. Caller cannot supply target, cached history, geometry or permission booleans. One durable event, fixed 1 percent and at most the four standard MOVE_J frames, no enable/jaw/reset/stop or retry. Keeps original peer/jaw anchors and all task budgets. RGB-supervised initialization retains the strict 0.003 rad band until all four frames return, then observes bounded RX-only settling: total 0.025 rad band around the origin-to-encoded-target interval, at most one cumulative second outside the original band, with original joint, 0.10 rad origin, 20 mm relative, peer/jaw, freshness and deadline guards. This deployment policy is not manufacturer accuracy or permission to resend; metric mode remains strict. Only complete sending plus new J feedback back within the original 0.003 rad target tolerance and a three-second stable target window establishes the local cache. Unknown old-cache activation remains part of the separately supervised startup scope; this is not the ordinary known-cache path proof, a physical stop, loaded support or completed plug task. Existing same-connection cache yields zero TX without renewing its source. New RGB initialization requests require more than the existing 3-second baseline plus 3-second stable window before claim; insufficient time returns refresh_required with no event, step or fault. This minimum does not guarantee completion within the original RGB deadline. Poll pair_status.", _object({
        "event_id": PAIR_ID_SCHEMA, "observation_id": {"type": "string", "minLength": 1},
        "arm": {"type": "string", "enum": ["left", "right"]},
        "unloaded_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
        "admission_mode": {"type": "string", "enum": ["metric_geometry", "rgb_supervised"]},
        "corridor_observation": {"type": "string", "minLength": 1, "maxLength": 4000},
    }, ["event_id", "observation_id", "arm", "unloaded_observation"]), False),
    _tool("robot_pair_observe", "Combine saved current three-camera RGB metadata with new independent pair feedback from the SAME host. Issues opaque owner/scene-bound peer stationary receipts. Does not start cameras, send targets or certify grasp/contact support. New frame numbers and configured serials required.", _object({"rgb_observation_path": {"type": "string", "minLength": 1}}, ["rgb_observation_path"])),
    _tool("robot_pair_submit_once", "Asynchronously claim and attempt ONE selected-arm move, explicit joint or jaw target while monitoring its peer. Requires task-ready state. Joint approach/align resolves official/current sources internally and requires a same-connection target cache; the separate unloaded initialize_joint_target entry can establish it when applicable. Default metric_geometry keeps the existing measured-source contract. Explicit admission_mode=rgb_supervised supports joint approach/align and confirmed-release retreat with current unloaded_observation for the selected worker and corridor_observation covering both arms and attachments. It binds saved RGB, operation and encoded target, uses the same official/current limits and real cache, and does not invent metric geometry or inherit physical hold support. Only after four complete frame returns, RGB ordinary motion permits bounded RX-only settling: 0.025 rad around the origin-to-encoded-target interval intersected with origin +/-0.028 rad, at most one cumulative second outside the original band, with all joint, relative-pose, peer/jaw, freshness and deadline guards retained. Final arrival still requires the original tolerance and stable window. This branch uses latch_only cancellation; no extra hold frames are sent. This default profile is unchanged when motion_profile is omitted. Explicit motion_profile=coarse_approach is a separate RGB-only empty-arm approach contract: kind joint and operation approach only, both arms free of active grasp/contact episodes, with current far_from_target_observation plus unloaded and whole-arm corridor descriptions. It bounds each requested and encoded joint change to 3 degrees and each model endpoint translation to 20 mm, with a complete independent joint-box process bound of 35 mm and 0.08 rad at the same 1% speed. It reserves no extra hold motion because cancellation remains latch_only; this is not a physical stopping proof. It does not apply to alignment, loaded/contact motion, release retreat, grippers or initialization, and cannot authorize them by changing a profile label. Real scene clearance remains visual testimony, not metric collision certification; host freshness, target cache, once-only and original session budget remain required. Missing numerical sources or unresolved boundary residuals still refuse before dispatch. Tool callers cannot supply sources or permission booleans. Requires current host-issued scene/peer receipt. Same event id never dispatches twice, including after clean owner restart; pending or uncertain records latch both arms. Existing 1%, 30mm/0.05rad MOVE_L or 0..55mm fixed 0.2 nominal jaw limits remain. Exactly one matching target field. grip_supported with gripper uses a dedicated at-most-5mm closing probe at nominal 0.2; a stable contact candidate is observation only, never grasp, target cancellation or loaded support. A probe may include grasp_object_id to register a durable episode; robot_pair_retain_grasp can establish zero-TX static retention for unloaded peer preparation. Otherwise unresolved candidates block arm motion; explicit same-jaw release_retreat opens at most 5mm and requires measured opening/arrival. Tracked opening additionally needs release_support_observation and release_support_relation=independent_support_present from the current RGB; arrival leaves release_opened unresolved. Continue bounded opening as needed, then robot_pair_confirm_release resolves new visual separation and feedback. Subsequent joint release_retreat requires release_retreat_observation describing the empty gripper in each new scene and a current unchanged release token. A separate explicit RGB branch supports right extract_segment/transport/insert_segment with loaded_observation, frozen source_object_id/target_object_id, two retained episodes and left original static anchor. Extract/insert requested and encoded model endpoints are at most 2mm and 0.01rad; these are software target limits, not force bounds. Each segment ends pending_visual until robot_pair_confirm_loaded_response records new object evidence with fresh feedback. Contact/support booleans remain false, jaws are not resent, and unknown/adverse responses fault. Caller booleans cannot enable contact hold. RGB joint requests require more than the existing 3-second baseline plus 3-second final-stability window remaining before claim. A short window returns refresh_required without claiming an event, consuming a step or faulting. Refresh RGB and decide again; the 6 seconds are a necessary minimum, not a completion guarantee. Source/plan costs are rechecked before claim. The first frame independently needs more than 3 seconds remaining; a late post-claim refusal keeps the zero-TX fault and consumed event instead of refunding or retrying. Poll pair_status; cancel stops NEW software sends, not physical motion.", PAIR_STEP_SCHEMA, False),
    _tool("robot_pair_status", "Read persistent pair/action receipts and capability limits; no target or connection change. After fault, an open idle host continues RX-only diagnostics in fault_feedback with capture time and fragment freshness; active actions defer that read. Diagnostics never renew task or hold permission. Move receipts separate measured robot response from object progress. Stability, arrival, object success and physical stopping remain separate.", _object({"event_id": PAIR_ID_SCHEMA})),
    _tool("robot_pair_retain_grasp", "Observe and durably retain one existing supported jaw candidate without sending a target. Requires a probe submitted with grasp_object_id, this host's saved current RGB scene, and the outer model's explicit visual description. The adapter supplies a NEW three-second trace and fixed-anchor existing-target contract; caller input is semantic testimony, not grasp/force/stop proof. Allows only the other unloaded arm to approach, align or probe; loaded extraction/insertion remain unavailable. Same event never renews twice. Does not start cameras.", _object({
        "event_id": PAIR_ID_SCHEMA, "episode_id": PAIR_ID_SCHEMA,
        "observation_id": {"type": "string", "minLength": 1},
        "visual_description": {"type": "string", "minLength": 1, "maxLength": 4000},
        "object_relation": {"type": "string", "enum": ["between_fingers"]},
        "support_relation": {"type": "string", "enum": ["original_support_present"]},
    }, ["event_id", "episode_id", "observation_id", "visual_description", "object_relation", "support_relation"]), False),
    _tool("robot_pair_cancel", "Explicit client cancellation immediately blocks ordinary frames and persists the reason. A fully returned MOVE_J on the same live worker/connection may enter one separately budgeted same-mode bounded hold transaction using new feedback; unknown/partial sends, generic faults and EOF cannot. Further independent faults abort hold. MOVE_L/jaw cancellation only latches. No reset or disable; measured hold and physical stop remain separate and stop state stays unknown.", _object({"reason": {"type": "string", "minLength": 1, "maxLength": 2000}}, ["reason"]), False),
    _tool("robot_pair_confirm_release", "Confirm separation after a tracked supported opening, without sending CAN. Requires this owner's release_opened episode, new saved RGB after the final opening, an explicit description of the object clear of the fingers and its independent support. The adapter supplies a new three-second stable trace and an exact local release token; durable confirmation precedes zero-TX local cleanup. Allows later unloaded joint release_retreat with a fresh empty-gripper RGB description. Semantic testimony is not independent visual verification, target-socket correctness, final stability or physical-stop proof. No automatic opening, retreat or retry.", _object({
        "event_id": PAIR_ID_SCHEMA, "episode_id": PAIR_ID_SCHEMA,
        "observation_id": {"type": "string", "minLength": 1},
        "visual_description": {"type": "string", "minLength": 1, "maxLength": 4000},
        "object_relation": {"type": "string", "enum": ["object_clear_of_fingers"]},
        "support_relation": {"type": "string", "enum": ["independent_support_present"]},
    }, ["event_id", "episode_id", "observation_id", "visual_description", "object_relation", "support_relation"]), False),
    _tool("robot_pair_close", "Close an idle persistent host and release its software lock; pending actions must first finish/cancel. Does not stop or certify the robot. An unreleased candidate or retained grasp latches the pair instead of clean detaching. Faults and frozen budgets survive close; no clear-fault operation is provided.", _object(), False),
])


def _read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"),
                      parse_constant=lambda s: (_ for _ in ()).throw(ValueError("Nonfinite JSON: " + s)))


def _write(path: Path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    # Keep one-shot claims durable across a host crash before any CAN send.
    descriptor = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class ToolService:
    def __init__(self, root: Path):
        self.root = Path(root).resolve()
        self.profile = _read(self.root / "configs/robot.json")
        self.runs = self.root / "runs"
        self.persistent = False
        self.pair_host = None

    def call(self, name: str, args: dict) -> dict:
        spec = next((item for item in TOOL_SCHEMAS if item["name"] == name), None)
        if spec is None:
            raise ValueError("Unknown robot tool: " + str(name))
        validate(args, spec["inputSchema"])
        pair_names = {"robot_pair_open", "robot_pair_observe", "robot_pair_submit_once", "robot_pair_retain_grasp", "robot_pair_confirm_release", "robot_pair_confirm_loaded_response",
                      "robot_pair_status", "robot_pair_cancel", "robot_pair_close", "robot_pair_prepare_gripper",
                      "robot_pair_inspect_joint_limits", "robot_pair_promote_ready", "robot_pair_initialize_joint_target",
                      "robot_pair_publish_geometry"}
        if name not in pair_names and not spec["annotations"]["readOnlyHint"]:
            from .pair_ledger import platform_state
            if self.pair_host is not None:
                raise RuntimeError("Persistent pair host owns both arms; use its one-action interface")
            pair_state = platform_state(self.runs / "pair_sessions.sqlite")
            if pair_state is not None and (pair_state["fault"] or pair_state["owner"] or pair_state["pending_events"]):
                raise RuntimeError("Persistent pair owner, pending send or fault blocks legacy command entrypoints; no restart bypass")
        methods = {"robot_describe": self.describe, "robot_read_state": self.read_state,
                   "robot_single_arm_move_once": self.single_arm_move_once,
                   "robot_single_arm_gripper_once": self.single_arm_gripper_once,
                   "robot_move_once": self.move_once, "robot_gripper_once": self.gripper_once,
                   "robot_request_can_control": self.request_can_control,
                   "robot_startup_arms": self.startup_arms,
                   "robot_startup_arm": self.startup_arm,
                   "robot_prepare_gripper": self.prepare_gripper,
                   "robot_home_arm": self.home_arm,
                   "robot_inspect_firmware": self.inspect_firmware,
                   "robot_inspect_joint_limits": self.inspect_joint_limits,
                   "robot_prepare_grippers": self.prepare_grippers,
                   "robot_recover_joint_boundary": self.recover_joint_boundary,
                   "robot_bounded_joint_step": self.bounded_joint_step,
                   "robot_qualify_linear_hold": self.qualify_linear_hold,
                   "robot_observe": self.observe, "robot_fk": self.fk,
                   "robot_preview_plan": self.preview, "robot_submit_plan": self.submit,
                   "robot_check_execution": self.check_execution, "robot_execution_status": self.execution_status,
                   "robot_cancel_execution": self.cancel_execution, "robot_depth_at_pixels": self.depth_at_pixels,
                   "robot_pair_open": self.pair_open, "robot_pair_observe": self.pair_observe,
                   "robot_pair_submit_once": self.pair_submit_once, "robot_pair_status": self.pair_status,
                   "robot_pair_retain_grasp": self.pair_retain_grasp,
                   "robot_pair_confirm_release": self.pair_confirm_release,
                   "robot_pair_confirm_loaded_response": self.pair_confirm_loaded_response,
                   "robot_pair_prepare_gripper": self.pair_prepare_gripper,
                   "robot_pair_inspect_joint_limits": self.pair_inspect_joint_limits,
                   "robot_pair_publish_geometry": self.pair_publish_geometry,
                   "robot_pair_promote_ready": self.pair_promote_ready,
                   "robot_pair_initialize_joint_target": self.pair_initialize_joint_target,
                   "robot_pair_cancel": self.pair_cancel, "robot_pair_close": self.pair_close}
        return methods[name](**args)

    def _pair(self):
        if self.pair_host is None:
            raise RuntimeError("No persistent pair host in this server process")
        return self.pair_host

    def pair_open(self, run_id, task_id, workspace_clearance_statement, max_steps=128, max_duration_s=900,
                  connection_mode="ready"):
        if not self.persistent:
            raise RuntimeError("Pair open requires the long-running stdio server; --call cannot retain ownership")
        if self.pair_host is not None:
            raise RuntimeError("A pair host already exists; keep the same owner and task budget")
        from .pair_host import PairHost
        from .joint_sources import JointSourcesProvider
        task = {"task_id": task_id, "roles": {"left": "task", "right": "task"},
                "site_context": {"workspace_clearance": {"source": "user", "statement": workspace_clearance_statement}}}
        provider = JointSourcesProvider(self.root.parents[1], self.profile, run_id, runs_root=self.runs)
        host = PairHost(self.runs, self.profile, run_id, task, max_steps, max_duration_s,
                        connection_mode=connection_mode, joint_sources_provider=provider)
        # Retain even a failed open for status and explicit resource cleanup.
        # The durable fault/owner must not leave an unreachable live adapter.
        self.pair_host = host
        return host.open()

    def pair_prepare_gripper(self, event_id, observation_id, arm, empty_jaw_observation):
        return self._pair().prepare_gripper(event_id, observation_id, arm, empty_jaw_observation)

    def pair_publish_geometry(self, observation_id, record_set_id):
        return self._pair().publish_geometry(observation_id, record_set_id)

    def pair_inspect_joint_limits(self, event_id):
        return self._pair().inspect_joint_limits(event_id)

    def pair_promote_ready(self):
        return self._pair().promote_ready()

    def pair_initialize_joint_target(self, event_id, observation_id, arm, unloaded_observation,
                                     admission_mode="metric_geometry", corridor_observation=None):
        return self._pair().initialize_joint_target(event_id, observation_id, arm, unloaded_observation,
            admission_mode=admission_mode, corridor_observation=corridor_observation)

    def pair_observe(self, rgb_observation_path):
        path = Path(rgb_observation_path).resolve()
        # Existing continuous recorder lives in the bundle's artifacts tree.
        # Reading its metadata does not open or restart any camera.
        bundle = self.root.parents[1]
        if not path.is_relative_to(bundle) or path.name != "observation.json":
            raise ValueError("Current recorder observation.json must be inside this workspace")
        metadata = path.read_bytes()
        rgb = json.loads(metadata, parse_constant=lambda s: (_ for _ in ()).throw(
            ValueError("Nonfinite JSON: " + s)))
        evidence = {}
        for camera, view in rgb.get("cameras", {}).items():
            image = Path(view.get("rgb_path", "")).resolve()
            if not image.is_relative_to(path.parent) or image.suffix != ".png" or not image.is_file():
                raise ValueError("RGB metadata must reference its saved current PNGs")
            evidence[camera] = {"rgb_path": str(image), "artifact_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
                               "frame_number": view.get("frame_number"), "host_received_at": view.get("host_received_at")}
        result = self._pair().observe(rgb, saved_rgb_evidence=evidence)
        result["rgb_metadata_sha256"] = hashlib.sha256(metadata).hexdigest()
        result["rgb_metadata_path"] = str(path)
        return result

    def pair_submit_once(self, event_id, observation_id, peer_receipt_id, arm, kind, operation,
                         target_pose_m_rad=None, width_m=None, grasp_object_id=None, target_joints_rad=None,
                         release_support_observation=None, release_support_relation=None,
                         release_retreat_observation=None, admission_mode=None,
                         unloaded_observation=None, corridor_observation=None, loaded_observation=None,
                         source_object_id=None, target_object_id=None, motion_profile=None,
                         far_from_target_observation=None):
        if sum(value is not None for value in (target_pose_m_rad, target_joints_rad, width_m)) != 1:
            raise ValueError("Supply exactly one matching pose, joint or gripper target")
        if kind == "move" and target_pose_m_rad is not None:
            target = target_pose_m_rad
        elif kind == "joint" and target_joints_rad is not None:
            target = target_joints_rad
        elif kind == "gripper" and width_m is not None:
            target = width_m
        else:
            raise ValueError("Supply exactly the target matching move, joint or gripper")
        kwargs = {"grasp_object_id": grasp_object_id} if grasp_object_id is not None else {}
        kwargs.update({key: value for key, value in (
            ("release_support_observation", release_support_observation),
            ("release_support_relation", release_support_relation),
            ("release_retreat_observation", release_retreat_observation),
            ("admission_mode", admission_mode), ("unloaded_observation", unloaded_observation),
            ("corridor_observation", corridor_observation), ("loaded_observation", loaded_observation),
            ("source_object_id", source_object_id), ("target_object_id", target_object_id),
            ("motion_profile", motion_profile),
            ("far_from_target_observation", far_from_target_observation)) if value is not None})
        return self._pair().submit(event_id, observation_id, peer_receipt_id, arm, kind, target, operation, **kwargs)

    def pair_retain_grasp(self, event_id, episode_id, observation_id, visual_description,
                          object_relation, support_relation):
        return self._pair().retain_grasp(event_id, episode_id, observation_id, visual_description,
                                        object_relation, support_relation)

    def pair_confirm_loaded_response(self, event_id, action_event_id, observation_id, visual_description,
                                     response, object_relation, support_relation, task_relation):
        return self._pair().confirm_loaded_response(event_id, action_event_id, observation_id, visual_description,
                                                   response, object_relation, support_relation, task_relation)

    def pair_confirm_release(self, event_id, episode_id, observation_id, visual_description,
                             object_relation, support_relation):
        return self._pair().confirm_release(event_id, episode_id, observation_id, visual_description,
                                           object_relation, support_relation)

    def pair_status(self, event_id=None):
        return self._pair().status(event_id)

    def pair_cancel(self, reason):
        return self._pair().cancel(reason)

    def pair_close(self):
        result = self._pair().close()
        self.pair_host = None
        return result

    def shutdown(self):
        """Unexpected client loss latches dispatch, never claims a physical stop."""
        host = self.pair_host
        if host is None:
            return
        host.cancel("Control client disconnected without explicit pair_close", allow_hold=False)
        if host.active_event_id is not None:
            host.wait(host.active_event_id, timeout=2)
        if host.active_event_id is None:
            host.close()

    def _new_run(self, prefix):
        run_id = prefix + "_" + uuid.uuid4().hex
        directory = self.runs / run_id
        directory.mkdir(parents=True, exist_ok=False)
        return run_id, directory

    def _saved(self, run_id, prefix, filename):
        if not isinstance(run_id, str) or not re.fullmatch(prefix + r"_[0-9a-f]{32}", run_id):
            raise ValueError("Invalid saved run id")
        path = self.runs / run_id / filename
        if not path.exists():
            return None
        if not path.resolve().is_relative_to(self.runs.resolve()):
            raise ValueError("Saved run is outside this project")
        return _read(path)

    def describe(self):
        from .arms import sdk_capabilities
        try:
            sdk = sdk_capabilities(self.profile["sdk_path"])
        except Exception as exc:
            sdk = {"status": "unavailable", "error": f"{type(exc).__name__}: {exc}"}
        task = (self.root / self.profile.get("task_file", "tasks/fold_tshirt/task.json")).resolve()
        if not task.is_relative_to(self.root):
            raise ValueError("Task data must be inside this project")
        return {"ok": True, "platform": "piperx_cloth_demo v0.9", "profile": self.profile,
                "task": _read(task), "sdk": sdk, "execution_readiness": self.check_execution(),
                "execution_available": False, "hardware_commands_sent": 0,
                "supervised_single_action_tools": ["robot_move_once", "robot_gripper_once", "robot_single_arm_move_once", "robot_single_arm_gripper_once"],
                "supervised_scope": "Finite attended trials only; not full-plan commissioning or validated holding stop",
                "device_access_tested_here": False,
                "reference_texts": {name: (self.root / relative).read_text(encoding="utf-8")
                                    for name, relative in (("robot_guide", "docs/ROBOT_GUIDE.md"),
                                                           ("sdk_audit", "docs/SDK_AUDIT.md"),
                                                           ("experiment_policy", "prompts/experiment.md"))},
                "read_first": [str(self.root / "docs/ROBOT_GUIDE.md"), str(self.root / "prompts/experiment.md")],
                "next_step": "Acquire current three-camera views and both arm states; identify the configured task objects, assess the entire motion space, and form a task-specific staged plan"}

    def _read_arms(self):
        from .arms import read_arms
        return read_arms(self.profile["arms"], self.profile["sdk_path"])

    def _backend(self):
        from .backend import PyAgxBackend
        return PyAgxBackend(self.profile)

    def check_execution(self):
        return {"ok": False, "status": "blocked", "implementation_present": True,
                "physical_execution_available": False, "hardware_commands_sent": 0,
                "reasons": readiness(self.profile, self._backend())}

    def read_state(self):
        if self.pair_host is not None:
            return self.pair_host.read_state()
        state = self._read_arms()
        return {"ok": state.get("status") == "complete", "state": state,
                "hardware_commands_sent": 0, "motion_permitted": False}

    def request_can_control(self):
        from .takeover import request_can_control
        with ExclusiveExecution(self.runs):
            run_id, directory = self._new_run("takeover")
            _write(directory / "request.json", {
                "takeover_id": run_id, "started_unix_s": time.time(),
                "arms": self.profile["arms"], "speed_percent": 1,
                "scope": "mode-only administrative takeover; no target dispatch",
                "risk": "Mode changes can affect physical motion; no verified holding stop.",
            })
            journal = Journal(directory)
            result = request_can_control(
                self.profile, lambda event, data: journal.append(event, **data))
            result.update(takeover_id=run_id, record_path=str(directory / "result.json"),
                          trajectory_execution_available=False)
            _write(directory / "result.json", result)
            return result

    def startup_arms(self):
        from .takeover import startup_arms
        with ExclusiveExecution(self.runs):
            run_id, directory = self._new_run("startup")
            _write(directory / "request.json", {
                "startup_id": run_id, "started_unix_s": time.time(),
                "arms": self.profile["arms"], "speed_percent": 1,
                "scope": "standby CAN takeover and arm enable only; no target dispatch",
                "risk": "Mode/enable can activate motion; no verified holding stop. No automatic retry.",
            })
            journal = Journal(directory)
            result = startup_arms(
                self.profile, lambda event, data: journal.append(event, **data))
            result.update(startup_id=run_id, record_path=str(directory / "result.json"),
                          trajectory_execution_available=False)
            _write(directory / "result.json", result)
            return result

    def _administrative_call(self, prefix, operation, scope, arguments=None, prior_receipt_id=None):
        arguments = arguments or {}
        with ExclusiveExecution(self.runs):
            run_id, directory = self._new_run(prefix)
            _write(directory / "request.json", {
                "run_id": run_id, "started_unix_s": time.time(),
                "arms": self.profile["arms"], "scope": scope, "arguments": arguments,
            })
            journal = Journal(directory)
            if prior_receipt_id is not None:
                # Claim BEFORE any hardware access under the same exclusive
                # lock. Even a pre-send failure cannot silently reuse it.
                _write(self.runs / prior_receipt_id / "target_replacement_claim.json",
                       {"claimed_by_run_id": run_id, "claimed_unix_s": time.time(),
                        "scope": "One subsequent complete-target trial; no automatic retry"})
            result = operation(self.profile, lambda event, data: journal.append(event, **data), **arguments)
            result.update(run_id=run_id, record_path=str(directory / "result.json"),
                          trajectory_execution_available=False)
            _write(directory / "result.json", result)
            return result

    def startup_arm(self, arm):
        from .single_arm_startup import startup_arm
        return self._administrative_call("single_startup", startup_arm,
                                         "Explicit single-arm standby CAN mode and joint enable; inactive arm RX only; no targets, retry or task gate unlock",
                                         {"arm": arm})

    def inspect_firmware(self):
        from .commissioning import inspect_firmware
        return self._administrative_call("firmware", inspect_firmware,
                                         "Manufacturer firmware query only; no actuator changes")

    def single_arm_move_once(self, arm, target_pose_m_rad):
        from .single_supervised_actions import move_once
        return self._administrative_call("single_supervised_move", move_once,
                                         "One attended selected-arm MOVE_L; other arm passive; no full-plan or holding-stop qualification",
                                         {"arm": arm, "target_pose_m_rad": target_pose_m_rad})

    def single_arm_gripper_once(self, arm, width_m, nominal_force_N):
        from .single_supervised_actions import gripper_once
        return self._administrative_call("single_supervised_gripper", gripper_once,
                                         "One attended selected-jaw target; both arms and passive jaw receive no targets; grasp requires visual confirmation",
                                         {"arm": arm, "width_m": width_m, "nominal_force_N": nominal_force_N})

    def move_once(self, arm, target_pose_m_rad):
        from .supervised_actions import move_once
        return self._administrative_call("supervised_move", move_once,
                                         "One model-selected finite supervised MOVE_L target; no certified stop or full-plan gate unlock",
                                         {"arm": arm, "target_pose_m_rad": target_pose_m_rad})

    def gripper_once(self, arm, width_m, nominal_force_N):
        from .supervised_actions import gripper_once
        return self._administrative_call("supervised_gripper", gripper_once,
                                         "One explicit low-nominal-force jaw target; contact/grasp remain visual questions",
                                         {"arm": arm, "width_m": width_m, "nominal_force_N": nominal_force_N})

    def prepare_grippers(self):
        from .gripper_prepare import prepare_grippers
        return self._administrative_call("gripper_prepare", prepare_grippers,
                                         "Explicit empty-gripper preparation at fresh current width, nominal force=0.2")

    def prepare_gripper(self, arm):
        from .single_gripper_prepare import prepare_gripper
        return self._administrative_call("single_gripper_prepare", prepare_gripper,
                                         "One empty jaw at measured current width only; passive other arm; no joint or task-motion qualification",
                                         {"arm": arm})

    def home_arm(self, arm):
        from .home_arm import home_arm
        return self._administrative_call("home_arm", home_arm,
                                         "One supervised return to existing six zero joint coordinates; no calibration, jaw command or task-motion qualification",
                                         {"arm": arm})

    def inspect_joint_limits(self):
        from .joint_limits import inspect_joint_limits
        return self._administrative_call("joint_limits", inspect_joint_limits,
                                         "Manufacturer stored-limit queries only; no parameter changes or actuator commands")

    def recover_joint_boundary(self, arm, target_joints_rad, recovery_profile="standard",
                               attachment_radius_m=None, available_clearance_m=None):
        from .joint_recovery import recover_joint_boundary
        arguments = {"arm": arm, "target_joints_rad": target_joints_rad}
        if recovery_profile != "standard" or attachment_radius_m is not None or available_clearance_m is not None:
            arguments.update(recovery_profile=recovery_profile, attachment_radius_m=attachment_radius_m,
                             available_clearance_m=available_clearance_m)
        return self._administrative_call("joint_recovery", recover_joint_boundary,
                                         "Single supervised bounded boundary recovery; no zero calibration, hold claim or task gate unlock",
                                         arguments)

    def bounded_joint_step(self, arm, target_joints_rad, attachment_radius_m, available_clearance_m):
        from .bounded_joint_step import bounded_joint_step
        return self._administrative_call("bounded_joint_step", bounded_joint_step,
                                         "One supervised model-selected J2/J3 step with conservative chain envelope; no hold claim or task gate unlock",
                                         {"arm": arm, "target_joints_rad": target_joints_rad,
                                          "attachment_radius_m": attachment_radius_m,
                                          "available_clearance_m": available_clearance_m})

    def qualify_linear_hold(self, arm, prior_mode_only_run_id=None):
        from .linear_hold import qualify_linear_hold
        arguments = {"arm": arm}
        if prior_mode_only_run_id is not None:
            prior = self._saved(prior_mode_only_run_id, "linear_hold", "result.json")
            request = self._saved(prior_mode_only_run_id, "linear_hold", "request.json")
            if (prior is None or request is None or prior.get("arm") != arm
                    or request.get("arms") != self.profile["arms"]):
                raise ValueError("Prior mode-only trial must match this arm and all current device bindings")
            events_path = self.runs / prior_mode_only_run_id / "events.jsonl"
            if not events_path.resolve().is_relative_to((self.runs / prior_mode_only_run_id).resolve()):
                raise ValueError("Prior journal is outside its run")
            events = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines() if line.strip()]
            arguments["prior_mode_only_record"] = {"result": prior, "request": request,
                                                   "source_run_id": prior_mode_only_run_id,
                                                   "events": [row for row in events if row.get("event") in
                                                              ("probe_send_intent", "probe_send_complete_unconfirmed", "connected_passively")]}
        return self._administrative_call("linear_hold", qualify_linear_hold,
                                         "Single supervised +6 mm MOVE_L target replacement test; no general stop qualification",
                                         arguments, prior_receipt_id=prior_mode_only_run_id)

    def observe(self, include_depth=True):
        from .cameras import CameraCaptureError, capture_cameras
        run_id, directory = self._new_run("obs")
        started = time.time()
        try:
            camera = capture_cameras(directory / "cameras", self.profile["cameras"], include_depth)
        except CameraCaptureError as exc:
            camera = exc.report
        except Exception as exc:
            camera = {"complete": False, "error": f"{type(exc).__name__}: {exc}", "cameras": {}}
        try:
            state = self._read_arms()
        except Exception as exc:
            state = {"status": "failed", "error": f"{type(exc).__name__}: {exc}"}
        complete = camera.get("complete") is True and state.get("status") == "complete"
        report = {"id": run_id, "ok": complete, "complete": complete,
                  "capture_started_unix_s": started, "capture_finished_unix_s": time.time(),
                  "cameras": camera, "state": state, "hardware_commands_sent": 0,
                  "synchronized": False, "sequence": "three camera snapshots, then passive arm snapshots",
                  "motion_permitted": False, "garment_detection": "model responsibility; not performed",
                  "image_paths": [item["rgb_path"] for item in camera.get("cameras", {}).values() if item.get("rgb_path")],
                  "record_path": str(directory / "observation.json")}
        _write(directory / "observation.json", report)
        return report

    def fk(self, model, joints_rad):
        from .arms import vendor_fk
        result = vendor_fk(model, joints_rad, self.profile["sdk_path"])
        return {"ok": True, **result, "physical_model_verified": False,
                "interpretation": "Selected SDK mathematical model only; no assertion it matches either physical arm"}

    def depth_at_pixels(self, observation_id, camera, pixels_uv):
        import numpy as np
        record = self._saved(observation_id, "obs", "observation.json")
        if record is None:
            raise ValueError("Unknown observation id")
        capture = record.get("cameras", {}).get("cameras", {}).get(camera, {})
        value = capture.get("depth_path")
        if not value:
            raise ValueError("This camera has no saved aligned depth")
        path = Path(value).resolve()
        if not path.is_relative_to((self.runs / observation_id).resolve()):
            raise ValueError("Depth file is outside its observation")
        if capture.get("depth_unit") != "metre" or capture.get("depth_aligned_to") != "color":
            raise ValueError("Require depth explicitly aligned to color in metres")
        depth = np.load(path, allow_pickle=False, mmap_mode="r")
        if depth.shape != (480, 640) or depth.dtype.kind != "f":
            raise ValueError("Unexpected depth array format")
        points = []
        scale = capture.get("depth_scale_m")
        if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 < scale < 1:
            raise ValueError("Missing reliable depth scale for encoding-limit check")
        for u, v in pixels_uv:
            if not 0 <= u < 640 or not 0 <= v < 480:
                raise ValueError("Pixel outside captured image")
            d = float(depth[v, u])
            at_limit = math.isfinite(d) and d >= (65535 - 0.5) * scale
            valid = math.isfinite(d) and d > 0 and not at_limit
            points.append({"u": u, "v": v, "depth_m": d if valid else None, "valid": valid,
                           "quality": "at_encoding_limit" if at_limit else ("sample_available" if valid else "unavailable")})
        return {"ok": True, "camera": camera, "observation_id": observation_id, "points": points,
                "capture_age_s": time.time() - capture["host_receive_unix_s"],
                "coordinate_frame": "camera depth axis, not robot base",
                "hardware_commands_sent": 0}

    def preview(self, plan):
        observation = self._saved(plan["observation_id"], "obs", "observation.json")
        report = preview_plan(plan, self.profile, observation)
        run_id, directory = self._new_run("preview")
        _write(directory / "plan.json", plan)
        plan_hash = hashlib.sha256((directory / "plan.json").read_bytes()).hexdigest()
        (directory / "intent.svg").write_text(render_intent_svg(plan), encoding="utf-8")
        report.update(ok=True, preview_id=run_id, plan_sha256=plan_hash,
                      plan_path=str(directory / "plan.json"), diagram_path=str(directory / "intent.svg"),
                      report_path=str(directory / "preview.json"), created_unix_s=time.time())
        _write(directory / "preview.json", report)
        return report

    def submit(self, preview_id):
        previous = self._saved(preview_id, "preview", "preview.json")
        if previous is None:
            raise ValueError("Unknown preview id")
        plan = self._saved(preview_id, "preview", "plan.json")
        path = self.runs / preview_id / "plan.json"
        if hashlib.sha256(path.read_bytes()).hexdigest() != previous["plan_sha256"]:
            raise ValueError("Saved plan changed after preview; create a new preview")
        observation = self._saved(plan["observation_id"], "obs", "observation.json")
        with ExclusiveExecution(self.runs):
            # The preview itself is the idempotency key. Never repeat an uncertain send.
            claim = self._saved(preview_id, "preview", "execution_claim.json")
            if claim:
                return self.execution_status(claim["execution_id"])
            preview_cancel = self.runs / preview_id / "cancel.json"
            if preview_cancel.exists():
                return {"ok": False, "status": "cancelled_before_start", "preview_id": preview_id,
                        "hardware_commands_sent": 0}
            report = preview_plan(plan, self.profile, observation)
            backend = self._backend()
            reasons = readiness(self.profile, backend)
            if not any(c["name"] == "observation" and c["status"] == "passed" for c in report["checks"]):
                reasons.append({"code": "current_complete_observation_required"})
            if reasons:
                run_id, directory = self._new_run("submission")
                report.update(ok=False, status="blocked", submission_id=run_id, preview_id=preview_id,
                              reasons=reasons, record_path=str(directory / "submission.json"))
                _write(directory / "submission.json", report)
                return report
            run_id, directory = self._new_run("execution")
            _write(directory / "request.json", {"execution_id": run_id, "preview_id": preview_id,
                                               "plan_sha256": previous["plan_sha256"], "started_unix_s": time.time()})
            _write(self.runs / preview_id / "execution_claim.json", {"execution_id": run_id})
            journal = Journal(directory)
            journal.append("execution_claimed", preview_id=preview_id)
            result = run_plan(plan, observation, backend, journal,
                              cancelled=lambda: preview_cancel.exists() or (directory / "cancel.json").exists())
            result.update(execution_id=run_id, preview_id=preview_id,
                          record_path=str(directory / "result.json"))
            _write(directory / "result.json", result)
            return result

    def execution_status(self, execution_id=None, preview_id=None):
        if (execution_id is None) == (preview_id is None):
            raise ValueError("Supply exactly one execution_id or preview_id")
        if preview_id is not None:
            preview = self._saved(preview_id, "preview", "preview.json")
            if preview is None:
                raise ValueError("Unknown preview id")
            claim = self._saved(preview_id, "preview", "execution_claim.json")
            if claim is None:
                cancelled = (self.runs / preview_id / "cancel.json").exists()
                return {"ok": not cancelled, "status": "cancelled_before_start" if cancelled else "not_started", "preview_id": preview_id,
                        "hardware_commands_sent": 0}
            execution_id = claim["execution_id"]
        request = self._saved(execution_id, "execution", "request.json")
        if request is None:
            raise ValueError("Unknown execution id")
        result = self._saved(execution_id, "execution", "result.json")
        if result is not None:
            return result
        events = self.runs / execution_id / "events.jsonl"
        last_event = None
        if events.is_file():
            lines = events.read_text(encoding="utf-8").splitlines()
            for line in reversed(lines):
                try:
                    last_event = json.loads(line)
                    break
                except ValueError:
                    continue
        return {"ok": False, "status": "in_flight_or_interrupted", "execution_id": execution_id,
                "physical_state": "unknown_until_fresh_feedback", "last_event": last_event,
                "automatic_resume": False, "cancel_requested": (events.parent / "cancel.json").exists()}

    def cancel_execution(self, execution_id=None, preview_id=None):
        result = self.execution_status(execution_id, preview_id)
        if preview_id and result.get("status") in ("not_started", "cancelled_before_start"):
            # Keep this cancellation even if submit claims execution concurrently.
            try:
                _write(self.runs / preview_id / "cancel.json", {"requested_unix_s": time.time()})
            except FileExistsError:
                pass
            return {"ok": True, "status": "cancellation_recorded_for_preview", "preview_id": preview_id,
                    "physical_stop_verified": False, "hardware_commands_sent": 0}
        if result.get("status") != "in_flight_or_interrupted":
            return {"ok": True, "status": "already_finished", "execution": result}
        execution_id = result["execution_id"]
        marker = self.runs / execution_id / "cancel.json"
        try:
            _write(marker, {"requested_unix_s": time.time()})
        except FileExistsError:
            pass
        return {"ok": True, "status": "cancellation_requested", "execution_id": execution_id,
                "physical_stop_verified": False, "hardware_commands_sent": 0}
