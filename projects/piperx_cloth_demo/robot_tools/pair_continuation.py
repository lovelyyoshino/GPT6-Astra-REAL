"""Separately audited, once-only same-run RGB expiry and zero-TX continuations.

Administrative only: no SDK, camera, transmission, budget renewal or cache
transfer. Archived post-failure observations are evidence, not fresh admission;
the new owner must obtain its own normal feedback/RGB and initialize normally.
The operator supplies its actual empty-gripper image interpretation, not an
executable permission Boolean or a claim of physical stopping.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import PairLedgerError, _hold_frames, _identifier, _json_object, _number
from .pair_restart import _closed, _current_contract, _file, _live_control_processes, _need


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _snapshot(db, run_id, *, zero_tx=False):
    from .joint_sources import _validate_bindings, _validated_limits
    if zero_tx:
        _need(not db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_predispatch_continuations'").fetchone(),
              "Only one audited zero-TX freshness continuation is supported")
        _need(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_continuations'").fetchone(),
              "The previously audited RGB continuation is required")
    else:
        _need(not db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_continuations'").fetchone(),
              "Only one explicit same-run continuation is supported")
    _need(db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_execution_epochs'").fetchone(),
          "An existing explicitly authorized execution epoch is required")
    scope_table = "pair_continuations" if zero_tx else "pair_execution_epochs"
    scope = db.execute("SELECT * FROM "+scope_table+" ORDER BY ordinal DESC LIMIT 1").fetchone()
    run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
    _need(scope is not None and run is not None and scope["run_id"] == run_id
          and scope["active_run_id"] == run_id and scope["owner"] is not None and scope["fault_id"] is not None,
          "The current faulted run and exact retired owner are required")
    tables = {}
    for row in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%' ORDER BY name"):
        name = row[0]
        _need(name.replace("_", "").isalnum(), "Unexpected ledger table name")
        tables[name] = sorted([dict(item) for item in db.execute('SELECT * FROM "'+name+'"')],
                              key=lambda item:json.dumps(item, sort_keys=True))
    _need(not any(e["status"] == "pending" for e in tables["pair_events"]), "Pending sends cannot continue")
    for name in ("pair_holds", "pair_hold_requests", "pair_joint_sends", "pair_grasp_episodes"):
        _need(not tables.get(name), "Grasp, hold or other target replacement history is outside this continuation")
    events = sorted([e for e in tables["pair_events"] if e["run_id"] == run_id], key=lambda e:e["step"])
    _need(events and len(events) == run["steps"] and [e["step"] for e in events] == list(range(1, run["steps"]+1)),
          "Complete contiguous same-run dispatch history required")
    contract = json.loads(scope["contract_json"] if zero_tx else run["contract_json"])
    if zero_tx:
        _need(len(tables["pair_continuations"]) == 1, "Exactly one prior RGB continuation required")
        prior_record = json.loads(scope["record_json"])
        prior = prior_record["proposal"]
        _need(prior.get("schema") == "piper_same_run_rgb_expiry_continuation_v1"
              and prior.get("run_id") == run_id
              and _sha({k:v for k,v in prior.items() if k != "proposal_sha256"}) == prior.get("proposal_sha256")
              and scope["proposal_sha256"] == prior["proposal_sha256"]
              and prior_record.get("new_contract") == contract == prior.get("reviewed_contract")
              and prior_record.get("new_budget_allocated") is False
              and prior_record.get("old_rows_preserved") is True
              and type(prior_record.get("hardware_commands_sent")) is int
              and prior_record["hardware_commands_sent"] == 0,
              "Prior continuation's immutable audit contract differs")
        original_steps = prior["snapshot"]["run"]["steps"]
        _need(type(original_steps) is int and 0 < original_steps < run["steps"]
              and scope["previous_owner"] == prior["previous_owner"]
              and scope["owner"] != scope["previous_owner"], "Prior owner/step lineage differs")
        for name, expected in prior["snapshot"]["table_sha256"].items():
            rows = copy.deepcopy(tables.get(name, []))
            if name == "pair_events":
                rows = [r for r in rows if not (r["run_id"] == run_id and r["step"] > original_steps)]
            elif name == "pair_faults":
                rows = [r for r in rows if not (r["run_id"] == run_id and r["owner"] == scope["owner"])]
            elif name == "pair_runs":
                for r in rows:
                    if r["run_id"] == run_id:r["steps"] = original_steps
            _need(_sha(rows) == expected, "Previously audited history changed: "+name)
        events = [e for e in events if e["step"] > original_steps]
        _need(len(events) == 4, "Only one current limit query, two initializations and this zero-TX failure are covered")
        _need([json.loads(e["payload_json"]).get("kind") for e in events] ==
              ["query", "initialization", "initialization", "joint"], "Current preparation sequence differs")
        _need({json.loads(e["receipt_json"]).get("arm") for e in events[1:3]} == {"left", "right"},
              "Both arms require their own current initialization")
    totals = {side:{"attempted_frames":0,"sent_frames":0,"blocked_frames":0} for side in ("left","right")}
    current_bindings = None
    initialization_identities = {}
    for event in events:
        _need(event["owner"] == scope["owner"] and event["status"] == "complete", "Only one closed owner with complete receipts is covered")
        _need(hashlib.sha256(event["payload_json"].encode()).hexdigest() == event["payload_digest"], "Event payload digest mismatch")
        payload, receipt = json.loads(event["payload_json"]), json.loads(event["receipt_json"])
        device = receipt.get("device_receipt", receipt)
        if event != events[-1]:
            _need(event["success"] == 1 and payload.get("kind") in ("query","initialization")
                  and receipt.get("ok") is True, "Only completed limit queries and initializations may precede this failure")
            if payload["kind"] == "query":
                _need(set(payload) == {"kind","request","bindings"}
                      and payload["request"] == {"operation":"inspect_joint_limits"}, "Only exact limit queries are covered")
                _validate_bindings(contract,payload["bindings"])
                current_bindings = payload["bindings"]
                _need(receipt.get("event_id") == event["event_id"] and receipt.get("pair_owner") == event["owner"]
                      and receipt.get("fault_latched") is False, "Historical query owner or fault differs")
                for key,value in (("hardware_commands_sent",12),("joint_limit_queries_sent",12),
                        ("joint_limit_queries_attempted",12),("actuator_commands_sent",0),("target_commands_sent",0),
                        ("mode_commands_sent",0),("enable_commands_sent",0),("stop_commands_sent",0)):
                    _need(type(receipt.get(key)) is int and receipt[key] == value,"Query transmission mismatch: "+key)
                _validated_limits({**receipt,"run_id":run_id,"owner":event["owner"],"bindings":payload["bindings"]},
                    run_id=run_id,owner=event["owner"],bindings=payload["bindings"],now=event["finished_at"])
                _need(event["began_at"] <= receipt["began_at"] <= receipt["ended_at"] <= event["finished_at"],
                      "Query raw evidence must occur within its own claimed event")
                _need(receipt["transmission_counts"] == {side:{"attempted_frames":6,"sent_frames":6,"blocked_frames":0}
                      for side in totals}, "Twelve query frames required")
            else:
                init = receipt.get("initialization_plan",{}); ident = init.get("identity",{}); side = receipt.get("arm")
                frames = receipt.get("frame_receipts",[])
                _need(receipt.get("operation") == "initialize_joint_target" and receipt.get("status") == "joint_target_initialized"
                      and receipt.get("cache_established") is True and receipt.get("errors") == []
                      and receipt.get("guard_violations") == [] and side in totals
                      and ident.get("run_id") == run_id and ident.get("owner") == event["owner"]
                      and ident.get("worker_id") == event["event_id"] and ident.get("arm") == side
                      and payload.get("expected_target_raw") == init.get("target_raw"), "Historical initialization source differs")
                _need(len(frames) == 4 and [f.get("frame") for f in frames] == _hold_frames(init["target_raw"])
                      and all(f.get("outcome") == "returned" for f in frames), "Historical initialization must have four returned frames")
                times = [_number(f.get("returned_at"),"initialization frame time") for f in frames]
                _need(times == sorted(times) and event["began_at"] <= times[0] <= times[-1] <= event["finished_at"], "Initialization chronology differs")
                for key,value in (("hardware_commands_sent",4),("gripper_commands_sent",0),
                        ("enable_commands_sent",0),("stop_commands_sent",0),("retries",0)):
                    _need(type(receipt.get(key)) is int and receipt[key] == value,"Initialization transmission mismatch: "+key)
                _need(receipt["transmission_counts"] == {s:{"attempted_frames":4 if s==side else 0,
                    "sent_frames":4 if s==side else 0,"blocked_frames":0} for s in totals},"Initialization arm counts differ")
                if zero_tx:
                    _need(current_bindings is not None and ident.get("epoch") == event["owner"]
                          and all(ident.get(k) == current_bindings[side][k]
                                  for k in ("connection_id","model","firmware_profile")),
                          "Initialization identity differs from this owner's actual query bindings")
                    initialization_identities[side] = ident
        counts = device.get("transmission_counts")
        _need(type(counts) is dict and set(counts) == set(totals), "Exact two-arm transmission counters required")
        for side, row in counts.items():
            _need(set(row) == set(totals[side]) and all(type(value) is int and value >= 0 for value in row.values())
                  and row["attempted_frames"] == row["sent_frames"] and row["blocked_frames"] == 0,
                  "Unknown, blocked or partial transmissions cannot continue")
            for key in row:
                totals[side][key] += row[key]
        lifetime = device.get("session_transmission_counts")
        _need(type(lifetime) is dict and set(lifetime) == set(totals)
              and all(type(row) is dict and set(row) == set(totals[side])
                      and all(type(v) is int and v >= 0 for v in row.values()) for side,row in lifetime.items())
              and lifetime == totals, "Unrecorded or non-integer lifetime transmissions")
    event = events[-1]
    payload, receipt = json.loads(event["payload_json"]), json.loads(event["receipt_json"])
    device = receipt.get("device_receipt", {})
    plan, original = device.get("joint_path_plan", {}), device.get("original_event")
    identity = plan.get("identity", {}) if zero_tx else (original or {}).get("identity", {})
    arm = payload.get("arm")
    _need(event["success"] == 0 and receipt.get("ok") is False and payload.get("kind") == "joint"
          and payload.get("operation") in ("approach","align") and arm in totals,
          "Only the last failed unloaded ordinary joint approach/align is covered")
    expected_error = "Joint feedback exceeded 50 ms including validation" if zero_tx else "visual_rgb_expired: original joint RGB deadline reached"
    _need(device.get("errors") == [{"type":"RuntimeError", "detail":expected_error}]
          and device.get("ok") is False and device.get("guard_violations") == []
          and device.get("hold_receipt") is None and device.get("automatic_retry") is False,
          "The only device failure must be the exact diagnosed error")
    _need(all(type(device.get(key)) is int and device[key] == 0 for key in
              ("enable_commands_sent","stop_commands_sent","retries","passive_arm_commands_sent")),
          "No enable, stop, retry or passive command may accompany this action")
    if zero_tx:
        _need(receipt.get("event_id") == event["event_id"] and receipt.get("automatic_retry") is False
              and receipt.get("error") == "Stable feedback alone is not an arrived single dispatch",
              "Exact failed outer receipt is required")
        _need(current_bindings is not None and arm in initialization_identities
              and all(identity.get(k) == current_bindings[arm][k] == initialization_identities[arm][k]
                      for k in ("connection_id","model","firmware_profile")),
              "Rejected action identity differs from this owner's query and initialized connection")
        _need(original is None and device.get("original_action_report") is None
              and device.get("kind") == "joint" and device.get("status") == "pair_device_fault"
              and device.get("spatial_admission_mode") == "rgb_supervised"
              and device.get("hold_supported") is False and device.get("hold_policy") == "latch_only"
              and device.get("explicit_cancel_hold_bridge_bound") is False
              and device.get("motion_gate_unlocked") is False and device.get("joint_limits_changed") is False,
              "No completed original dispatch, hold or motion-gate entry may be recorded")
        for key in ("hardware_commands_sent","target_commands_sent","target_calls_sent",
                    "enable_commands_sent","stop_commands_sent","retries","passive_arm_commands_sent"):
            _need(type(device.get(key)) is int and device[key] == 0, "Exact zero dispatch counter required: "+key)
        _need(device["transmission_counts"] == {s:{"attempted_frames":0,"sent_frames":0,"blocked_frames":0} for s in totals},
              "No bus attempt, return or blocked frame is covered")
        _need(identity.get("run_id") == run_id and identity.get("owner") == scope["owner"]
              and identity.get("epoch") == scope["owner"] and identity.get("worker_id") == event["event_id"]
              and identity.get("arm") == arm and plan.get("loaded_context") is None
              and plan.get("loaded_observation_only") is False and plan.get("recovery_mode") is None
              and plan.get("spatial_admission_mode") == "rgb_supervised"
              and plan.get("geometry",{}).get("schema") == "piper_rgb_supervised_joint_path_v1"
              and plan["geometry"].get("evidence",{}).get("operation") == payload["operation"]
              and plan.get("hold_supported") is False and plan.get("hold_policy") == "latch_only",
              "Exact unloaded ordinary RGB identity and plan required")
        target = payload.get("target")
        _need(type(target) is list and len(target) == 6
              and all(type(x) in (int,float) and math.isfinite(x) for x in target)
              and target == device.get("target_joints_rad") == plan.get("requested_target_joints_rad")
              and [round(x*(180/math.pi)*1000) for x in target] == plan.get("target_raw"),
              "Zero-TX rejected target must bind the claimed ordinary plan")
        _need(plan.get("frames") == _hold_frames(plan["target_raw"])
              and _sha({k:v for k,v in plan.items() if k != "plan_sha256"}) == plan.get("plan_sha256"),
              "Rejected plan integrity differs")
        failure = device.get("tracking_observation",{}).get("first_failure",{})
        _need(failure.get("type") == "RuntimeError" and failure.get("detail") == expected_error
              and failure.get("sample_role") == "rejected_observation"
              and failure.get("sample",{}).get("identity") == identity,
              "The original rejected feedback sample must remain recorded")
        at = _number(failure["sample"].get("captured_at"),"rejected feedback time")
        _need(event["began_at"] <= at <= event["finished_at"], "Rejected sample chronology differs")
    else:
        _need(original.get("schema") == "piper_rgb_supervised_joint_send_v1"
              and original.get("send_state") == "all_frames_returned"
              and original.get("operation") == payload["operation"]
              and original.get("spatial_admission_mode") == "rgb_supervised"
              and original.get("hold_supported") is False and original.get("hold_policy") == "latch_only"
              and original.get("fault") is None and original.get("event_id") == event["event_id"]
              and identity.get("run_id") == run_id and identity.get("owner") == scope["owner"]
              and identity.get("worker_id") == event["event_id"] and identity.get("arm") == arm
              and plan.get("identity") == identity and plan.get("loaded_context") is None
              and plan.get("geometry", {}).get("schema") == "piper_rgb_supervised_joint_path_v1"
              and original.get("rgb_admission") == plan.get("geometry")
              and original.get("plan_sha256") == plan.get("plan_sha256"), "Ordinary RGB source binding differs")
        target = payload.get("target")
        _need(type(target) is list and len(target) == 6
              and all(type(x) in (int,float) and math.isfinite(x) for x in target), "Frozen six-joint target required")
        _need([round(x*(180/math.pi)*1000) for x in target] == original.get("target_raw"), "Original wire target differs from claimed target")
        frames = original.get("frame_receipts", [])
        _need(len(frames) == 4 and [f.get("frame") for f in frames] == _hold_frames(original["target_raw"])
              and all(f.get("outcome") == "returned" for f in frames), "Exactly four complete original frames required")
        stamps = [_number(f.get("returned_at"), "frame time") for f in frames]
        _need(stamps == sorted(stamps) and event["began_at"] <= stamps[0] <= stamps[-1] <= event["finished_at"], "Original send chronology differs")
        _need(device.get("transmission_counts") == {side:{"attempted_frames":4 if side == arm else 0,
              "sent_frames":4 if side == arm else 0,"blocked_frames":0} for side in totals}
              and type(device.get("hardware_commands_sent")) is int and device["hardware_commands_sent"] == 4,
              "Only the expected original four frames are covered")
    faults = [f for f in tables["pair_faults"] if f["run_id"] == run_id
              and (not zero_tx or f["owner"] == scope["owner"])]
    _need(len(faults) == 2 and scope["fault_id"] in [f["id"] for f in faults]
          and all(f["owner"] == scope["owner"] and f["at"] >= event["began_at"] for f in faults)
          and {f["reason"] for f in faults} == {"execution_receipt_failed",
              "Claimed dispatch failed or uncertain: Stable feedback alone is not an arrived single dispatch"},
          "Additional or different faults require their own diagnosis")
    snapshot = {"run":dict(run),"scope":dict(scope),"table_sha256":{name:_sha(rows) for name,rows in tables.items()}}
    if zero_tx:
        snapshot["retired_owners"] = sorted({r["owner"] for name,rows in tables.items()
            for r in rows if name in ("pair_events","pair_scope","pair_execution_epochs","pair_continuations")
            and r.get("owner") is not None})
    return snapshot, event, device


def _observations(passive_paths, rgb_observation, visual_observation, contract, failed_at, now,
                  *, orientation_metric='euler_span_bound'):
    _need(orientation_metric in ('euler_span_bound','so3_diameter'), 'Unknown archived orientation metric')
    _need(type(passive_paths) is dict and set(passive_paths) == {"left","right"}, "Two actual passive records required")
    _need(type(visual_observation) is str and 0 < len(visual_observation.strip()) <= 4000,
          "Preserve the operator's actual current empty-jaw/no-contact image interpretation")
    passive = {}
    for side, path in passive_paths.items():
        raw, ref = _file(path); data = json.loads(raw)
        _need(data.get("mode") == "passive_receive_only" and data.get("channel") == contract["arms"][side]["channel"]
              and type(data.get("frames_sent_by_this_script")) is int and data["frames_sent_by_this_script"] == 0
              and data.get("malformed_frames") == 0 and data.get("complete_feedback_received") is True
              and data.get("missing_feedback_types") == [], "Complete zero-TX passive evidence on the bound channel required")
        began, ended = (_number(data.get(key), key) for key in ("started_at_s","finished_at_s"))
        _need(failed_at < began < ended <= now, "Passive evidence must follow the failed action")
        trace = data.get("pose_trace")
        _need(type(trace) is list and len(trace) >= 21, "At least 21 passive pose samples required")
        stamps = [_number(row.get("received_at_s"), "trace time") for row in trace]
        _need(all(a < b for a,b in zip(stamps,stamps[1:])) and stamps[-1]-stamps[0] >= 3
              and max(b-a for a,b in zip(stamps,stamps[1:])) <= .1
              and began <= stamps[0] <= stamps[-1] <= ended, "A complete three-second advancing passive trace is required")
        keys = [("joints_raw", "joint_"+str(i), math.pi/180000, .003) for i in range(1,7)]
        keys += [("end_pose_raw", key, 1e-6 if i < 3 else math.pi/180000, .0005 if i < 3 else .003)
                 for i,key in enumerate(("X_axis","Y_axis","Z_axis","RX_axis","RY_axis","RZ_axis"))]
        spans = {}
        for group,key,unit,limit in keys:
            values = [row.get(group,{}).get(key) for row in trace]
            _need(all(type(v) is int for v in values), "Complete raw integer feedback trace required")
            spans[key] = (max(values)-min(values))*unit
            _need(spans[key] <= limit, "Passive body is not stationary within existing observation bounds")
            times = [_number(row.get("field_received_at_s",{}).get(key), "field timestamp") for row in trace]
            _need(all(a <= b for a,b in zip(times,times[1:]))
                  and all(0 <= at-t <= .1 for at,t in zip(stamps,times)), "Passive fields are stale or regress")
        euler_span = sum(spans[k] for k in ("RX_axis","RY_axis","RZ_axis"))
        rotation_span = euler_span
        orientation = None
        if orientation_metric == 'so3_diameter':
            from .contact_receipt import _rotation_span
            # Retain every per-axis limit above. The overall angle is the
            # pairwise SO(3) diameter, not the looser sum of Euler-axis spans.
            poses = [[row['end_pose_raw'][key]*(1e-6 if i < 3 else math.pi/180000)
                      for i,key in enumerate(('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'))]
                     for row in trace]
            rotation_span = _rotation_span(poses)
            orientation = {'method':'pairwise_so3_diameter',
                'convention':'Rz(yaw) Ry(pitch) Rx(roll)', 'sample_count':len(trace),
                'euler_span_sum_bound_rad':euler_span, 'so3_diameter_rad':rotation_span,
                'overall_limit_rad':.003, 'individual_euler_axis_limit_rad':.003}
        _need(math.sqrt(sum(spans[k]**2 for k in ("X_axis","Y_axis","Z_axis"))) <= .0005
              and rotation_span <= .003,
              "Passive whole-position/rotation observation bound exceeded")
        status = data.get("raw_frame_latest",{}).get("0x2A1",{})
        status_bytes = bytes.fromhex(status.get("payload_hex", ""))
        _need(len(status_bytes) == 8 and list(status_bytes[:5]) == [1,0,1,0,0]
              and status_bytes[6:] == b"\0\0" and 0 <= ended-_number(status.get("received_at_s"),"status timestamp") <= .1,
              "Latest controller must report normal CAN/J/arrival")
        feedback = data.get("feedback",{})
        for name in ["PiperMsgGripperFeedBack"]+["PiperMsgLowSpdFeed_"+str(i) for i in range(1,7)]:
            record = feedback.get(name,{})
            flags = record.get("fields",{}).get("foc_status",{})
            expected_flags = {"voltage_too_low","motor_overheating","driver_overcurrent","driver_overheating",
                "driver_error_status","driver_enable_status"} | ({"sensor_status","homing_status"}
                if name == "PiperMsgGripperFeedBack" else {"collision_status","stall_status"})
            _need(set(flags) == expected_flags and flags.get("driver_enable_status") is True
                  and all(type(v) is bool and (v if k == "driver_enable_status" else not v) for k,v in flags.items())
                  and 0 <= ended-_number(record.get("received_at_s"),"health timestamp") <= .1,
                  "Complete current driver/jaw health required")
        width = feedback["PiperMsgGripperFeedBack"]["fields"].get("grippers_angle")
        _need(type(width) is int and 0 <= width <= 70000, "Measured current jaw width required")
        passive[side] = {"source":ref,"started_at":began,"ended_at":ended,"sample_count":len(trace),
                         "spans":spans,"jaw_width_m":width*1e-6,"jaw_whole_window_observed":False}
        if orientation is not None:
            passive[side]['orientation_observation'] = orientation
    raw, ref = _file(rgb_observation); rgb = json.loads(raw)
    _need(set(rgb.get("cameras",{})) == {"front","left_hand","right_hand"}, "Actual three-view RGB required")
    pictures = {}
    for view, camera in rgb["cameras"].items():
        at = _number(camera.get("host_received_at"),"RGB receive time")
        configured = {"front":"front","left_hand":"left_wrist","right_hand":"right_wrist"}[view]
        _need(camera.get("serial") == contract["cameras"][configured] and failed_at < at <= now
              and type(camera.get("frame_number")) is int and camera["frame_number"] > 0
              and camera.get("depth_enabled") is False, "Post-failure RGB identity/time mismatch")
        _, image_ref = _file(camera["rgb_path"])
        pictures[view] = {**image_ref,"host_received_at":at,"frame_number":camera["frame_number"]}
    return {"passive":passive,"rgb":{"source":ref,"images":pictures},"visual_observation":visual_observation,
            "scope":"Archived post-failure evidence only; new host must reobserve; no physical stop or task success proof"}


def _prepare_continuation(path, run_id, *, close_log, passive_paths, rgb_observation,
                          visual_observation, clock=time.time, zero_tx=False):
    """Read-only proposal bound to real failure/evidence and unchanged budget."""
    run_id = _identifier(run_id,"run_id"); source = Path(path).resolve(strict=True)
    db = sqlite3.connect(source.as_uri()+"?mode=ro",uri=True); db.row_factory=sqlite3.Row
    try:
        db.execute("PRAGMA query_only=ON");db.execute("BEGIN")
        snapshot,event,device = _snapshot(db,run_id,zero_tx=zero_tx)
    finally:
        db.close()
    now = _number(clock(),"clock");run=snapshot["run"]
    _need(snapshot["scope"]["last_time"] <= now < run["started_at"]+run["max_duration"]
          and run["steps"] < run["max_steps"], "The original time and step budget must remain available")
    old = json.loads(snapshot["scope"]["contract_json"] if zero_tx else run["contract_json"])
    proposal = {"schema":"piper_same_run_zero_tx_freshness_continuation_v1" if zero_tx else "piper_same_run_rgb_expiry_continuation_v1","database":str(source),"run_id":run_id,
        "created_at":now,"snapshot":snapshot,"snapshot_sha256":_sha(snapshot),"failed_event_id":event["event_id"],
        "failed_event_sha256":_sha(event),"previous_owner":event["owner"],"close":_closed(close_log,device),
        "evidence":_observations(passive_paths,rgb_observation,visual_observation,old,event["finished_at"],now),
        "reviewed_contract":_current_contract(source.parent.parent,old),
        "hardware_commands_sent":0,"dispatch_authorized":False,"physical_stop_verified":None,
        "budget":{"started_at":run["started_at"],"deadline_s":run["started_at"]+run["max_duration"],
                  "max_steps":run["max_steps"],"steps":run["steps"],"max_duration_s":run["max_duration"]},
        "cache_or_limits_transferred":False,"new_budget_allocated":False}
    if zero_tx:
        proposal["retired_owners"] = snapshot["retired_owners"]
        _need(proposal["reviewed_contract"]["code"].get("pair_joint_adapter.py") != old["code"].get("pair_joint_adapter.py"),
              "The diagnosed adapter code must actually be repaired before continuation")
    return {**proposal,"proposal_sha256":_sha(proposal)}


def _activate_continuation(proposal, *, project_root, clock=time.time, zero_tx=False):
    """Append once under the original authorization, budget and task scope."""
    _need(_sha({k:v for k,v in proposal.items() if k != "proposal_sha256"}) == proposal.get("proposal_sha256"), "Proposal digest mismatch")
    evidence=proposal["evidence"]
    canonical=_prepare_continuation(proposal["database"],proposal["run_id"],close_log=proposal["close"]["source"]["path"],
        passive_paths={side:item["source"]["path"] for side,item in evidence["passive"].items()},
        rgb_observation=evidence["rgb"]["source"]["path"],visual_observation=evidence["visual_observation"],
        clock=lambda:proposal["created_at"],zero_tx=zero_tx)
    _need(canonical == proposal,"Canonical reviewed continuation required")
    root=Path(project_root).resolve(strict=True);path=Path(proposal["database"])
    _need(path == root/"runs/pair_sessions.sqlite","Use the authoritative project ledger")
    _need(not _live_control_processes(),"An old/live control process blocks continuation")
    with ExclusiveExecution(path.parent):
        db=sqlite3.connect(path.as_uri()+"?mode=rw",uri=True,isolation_level=None);db.row_factory=sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL");db.execute("BEGIN IMMEDIATE")
            snapshot,event,device=_snapshot(db,proposal["run_id"],zero_tx=zero_tx)
            _need(_sha(snapshot)==proposal["snapshot_sha256"],"Ledger changed after proposal")
            _need(_closed(proposal["close"]["source"]["path"],device)==proposal["close"],"Close evidence changed")
            old_contract=json.loads(snapshot["scope"]["contract_json"] if zero_tx else snapshot["run"]["contract_json"])
            contract=_current_contract(root,old_contract)
            _need(contract==proposal["reviewed_contract"],"Reviewed source contract changed")
            now=_number(clock(),"clock")
            _need(proposal["created_at"] <= now < proposal["budget"]["deadline_s"],"Original deadline/chronology prohibits continuation")
            _need(_observations({s:v["source"]["path"] for s,v in evidence["passive"].items()},
                evidence["rgb"]["source"]["path"],evidence["visual_observation"],contract,event["finished_at"],now)==evidence,
                "Post-failure evidence changed")
            _need(not _live_control_processes(),"A control process appeared during continuation")
            table = "pair_predispatch_continuations" if zero_tx else "pair_continuations"
            extra = ",retired_owners_json TEXT NOT NULL" if zero_tx else ""
            db.execute("CREATE TABLE "+table+" (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,"
                "owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,previous_owner TEXT NOT NULL,"
                "failed_event_id TEXT NOT NULL,execution_epoch_ordinal INTEGER NOT NULL,contract_json TEXT NOT NULL,"
                "proposal_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL"+extra+")")
            committed=_number(clock(),"commit clock")
            _need(now <= committed < proposal["budget"]["deadline_s"],"Original deadline reached during continuation")
            record={"proposal":proposal,"activated_at":committed,"new_contract":contract,
                    "hardware_commands_sent":0,"new_budget_allocated":False,"old_rows_preserved":True,
                    "physical_stop_verified":None,"fresh_host_admission_required":True}
            db.execute("INSERT INTO "+table+" VALUES(1,?,NULL,NULL,NULL,?,?,?,?,?,?,?"+(",?" if zero_tx else "")+")",
                (proposal["run_id"],committed,proposal["previous_owner"],proposal["failed_event_id"],
                 snapshot["scope"]["execution_epoch_ordinal"] if zero_tx else snapshot["scope"]["ordinal"],
                 _json_object(contract,"continuation contract"),proposal["proposal_sha256"],
                 _json_object(record,"continuation record"))+((json.dumps(proposal["retired_owners"]),) if zero_tx else ()))
            _need(_current_contract(root,old_contract) == contract,
                  "Code changed during continuation writes")
            final=_number(clock(),"final commit clock")
            _need(committed <= final < proposal["budget"]["deadline_s"],"Original deadline/clock changed before commit")
            record["activated_at"] = final
            db.execute("UPDATE "+table+" SET last_time=?,record_json=? WHERE ordinal=1",
                       (final,_json_object(record,"continuation record")))
            check=_number(clock(),"durable commit clock")
            _need(final <= check < proposal["budget"]["deadline_s"],"Clock/deadline changed during final durable write")
            db.execute("COMMIT")
            return record
        except BaseException:
            if db.in_transaction:db.execute("ROLLBACK")
            raise
        finally:db.close()


def prepare_continuation(path, run_id, *, close_log, passive_paths, rgb_observation,
                         visual_observation, clock=time.time):
    """Original once-only full-four-frame RGB expiry continuation."""
    return _prepare_continuation(path, run_id, close_log=close_log, passive_paths=passive_paths,
        rgb_observation=rgb_observation, visual_observation=visual_observation, clock=clock)


def activate_continuation(proposal, *, project_root, clock=time.time):
    _need(proposal.get("schema") == "piper_same_run_rgb_expiry_continuation_v1", "Wrong continuation branch")
    return _activate_continuation(proposal, project_root=project_root, clock=clock)


def prepare_zero_tx_continuation(path, run_id, *, close_log, passive_paths, rgb_observation,
                                 visual_observation, clock=time.time):
    """Read-only audit of a distinct once-only known predispatch freshness failure."""
    return _prepare_continuation(path, run_id, close_log=close_log, passive_paths=passive_paths,
        rgb_observation=rgb_observation, visual_observation=visual_observation, clock=clock, zero_tx=True)


def activate_zero_tx_continuation(proposal, *, project_root, clock=time.time):
    """Append a separate scope without changing any old row, budget or failed event."""
    _need(proposal.get("schema") == "piper_same_run_zero_tx_freshness_continuation_v1", "Wrong continuation branch")
    return _activate_continuation(proposal, project_root=project_root, clock=clock, zero_tx=True)
