"""Explicit administrative enrollment after one known terminal init failure.

No device imports, hardware, camera startup, fault deletion or retry. Preparing
is read-only. Activation requires a *new* human authorization recorded by the
trusted outer operator, and preserves the old scope verbatim. Authorization
documents are audit records, not cryptographic proof of a human's identity.

The enrolled host must obtain new feedback/RGB and establish its own connection
and target cache through normal entrypoints. Enrollment is not readiness, a
physical-stop receipt, or permission to replay the failed action.
"""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import PairLedgerError, _hold_frames, _identifier, _json_object, _number


def _need(condition, message):
    if not condition:
        raise PairLedgerError(message)


def _sha(value):
    return hashlib.sha256(_json_object(value, "restart record").encode()).hexdigest()


def _file(path):
    path = Path(path).absolute()
    _need(path == path.resolve() and path.is_file(), "Existing nonsymlink evidence file required")
    raw = path.read_bytes()
    _need(len(raw) <= 8*1024*1024, "Evidence file exceeds eight MiB")
    return raw, {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}


def _snapshot(db, run_id):
    from .joint_sources import _validate_bindings, _validated_limits
    run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
    scope = db.execute("SELECT * FROM pair_scope WHERE id=1").fetchone()
    _need(run is not None and scope is not None, "Existing run and platform required")
    _need(scope["active_run_id"] == run_id and scope["fault_id"] is not None,
          "This narrow entry requires the original faulted active run")
    _need(not db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_execution_epochs'").fetchone(),
          "An execution successor already exists; no automatic repeated restart")
    tables = {}
    # Bind all durable pair history, including another run's pending sends or
    # late faults. No attempt to recreate a current hardware cache from rows.
    for row in db.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"):
        name = row[0]
        if name.startswith("pair_"):
            _need(name.replace("_", "").isalnum(), "Unexpected ledger table name")
            tables[name] = sorted([dict(x) for x in db.execute('SELECT * FROM "'+name+'"')],
                                  key=lambda x: json.dumps(x, sort_keys=True))
    _need(not any(e["status"] == "pending" for e in tables["pair_events"]), "Pending send blocks restart")
    for name in ("pair_holds", "pair_hold_requests", "pair_joint_sends", "pair_grasp_episodes"):
        _need(not tables.get(name), "Hold, ordinary movement or grasp history needs a different recovery")
    events = sorted([e for e in tables["pair_events"] if e["run_id"] == run_id], key=lambda e:e["step"])
    _need(len(events) == run["steps"] and events
          and [e["step"] for e in events] == list(range(1,len(events)+1)), "Exact contiguous event/budget history required")
    movements = []
    owner_queries = {}
    contract = json.loads(run["contract_json"])
    for event in events:
        _need(hashlib.sha256(event["payload_json"].encode()).hexdigest() == event["payload_digest"],
              "Historical event payload digest mismatch")
        payload, receipt = json.loads(event["payload_json"]), json.loads(event["receipt_json"])
        if payload.get("kind") == "query":
            _need(set(payload) == {"kind","request","bindings"}
                  and payload["request"] == {"operation":"inspect_joint_limits"}
                  and event["status"] == "complete" and event["success"] == 1
                  and receipt.get("ok") is True and receipt.get("fault_latched") is False
                  and receipt.get("event_id") == event["event_id"] and receipt.get("pair_owner") == event["owner"]
                  and receipt.get("execution_mode") == "inspect_joint_limits",
                  "Only completed non-actuating limit queries may precede initialization")
            _validate_bindings(contract, payload["bindings"])
            for key, count in (("hardware_commands_sent",12),("joint_limit_queries_sent",12),
                    ("joint_limit_queries_attempted",12),("actuator_commands_sent",0),("target_commands_sent",0),
                    ("mode_commands_sent",0),("enable_commands_sent",0),("stop_commands_sent",0)):
                _need(type(receipt.get(key)) is int and receipt[key] == count, "Unknown/extra query transmission: "+key)
            owner_queries[event["owner"]] = owner_queries.get(event["owner"],0)+1
            for key,count in (("transmission_counts",6),("session_transmission_counts",6*owner_queries[event["owner"]])):
                _need(receipt.get(key) == {s:{"attempted_frames":count,"sent_frames":count,"blocked_frames":0}
                                           for s in ("left","right")}, "Query lifetime frame totals differ")
                _need(all(type(v) is int for c in receipt[key].values() for v in c.values()), "Integer counters required")
            capture = {**receipt,"run_id":run_id,"owner":event["owner"],"bindings":payload["bindings"]}
            _validated_limits(capture,run_id=run_id,owner=event["owner"],bindings=payload["bindings"],now=event["finished_at"])
            _need(run["started_at"] <= event["began_at"] <= receipt["began_at"] <= receipt["ended_at"]
                  <= event["finished_at"], "Historical query chronology mismatch")
            continue
        _need(payload.get("kind") == "initialization" and event["success"] == 0,
              "Only one terminal first-target failure is covered")
        device = receipt.get("device_receipt", {})
        _need(device.get("operation") == "initialize_joint_target"
              and device.get("cache_established") is False and device.get("ok") is False
              and device.get("errors") == [{"type": "JointPathError", "detail": "joint_tracking_envelope",
                                            "code": "joint_tracking_envelope"}]
              and device.get("guard_violations") == [] and device.get("automatic_retry") is False,
              "Failure is not the known complete-send tracking-envelope case")
        plan = device.get("initialization_plan", {})
        identity = plan.get("identity", {})
        _need(identity.get("run_id") == run_id and identity.get("owner") == event["owner"]
              and identity.get("worker_id") == event["event_id"]
              and plan.get("spatial_admission_mode") == "rgb_supervised",
              "Initialization identity/visual branch mismatch")
        frames = device.get("frame_receipts", [])
        expected = _hold_frames(plan.get("target_raw"))
        _need(len(frames) == 4 and [f.get("frame") for f in frames] == expected
              and all(f.get("outcome") == "returned" for f in frames), "Complete four-frame returns required")
        stamps = [_number(f.get("returned_at"), "returned_at") for f in frames]
        _need(stamps == sorted(stamps) and event["began_at"] <= stamps[0] <= stamps[-1] <= event["finished_at"],
              "Send timestamps are not within the terminal event")
        side = device.get("arm")
        _need(side in ("left", "right"), "Selected arm missing")
        counts = device.get("transmission_counts", {})
        _need(counts == {s:{"attempted_frames":4 if s == side else 0,
                            "sent_frames":4 if s == side else 0,"blocked_frames":0} for s in ("left", "right")}
              and device.get("hardware_commands_sent") == 4
              and all(type(device.get(k)) is int and device[k] == 0 for k in
                      ("gripper_commands_sent", "enable_commands_sent", "stop_commands_sent", "retries")),
              "Partial/unknown/extra sends cannot be administratively retried")
        _need(all(type(v) is int for c in counts.values() for v in c.values()), "Integer movement counters required")
        prior = 6*owner_queries.get(event["owner"],0)
        lifetime = device.get("session_transmission_counts", {})
        _need(lifetime == {s:{"attempted_frames":prior+(4 if s == side else 0),
                              "sent_frames":prior+(4 if s == side else 0),"blocked_frames":0}
                            for s in ("left","right")}
              and all(type(v) is int for c in lifetime.values() for v in c.values()),
              "Lifetime counts include unrecorded or unknown sends")
        movements.append((event, device))
    _need(len(movements) == 1 and movements[0][0] == events[-1], "Exactly one final failed initialization required")
    event, device = movements[0]
    faults = [f for f in tables["pair_faults"] if f["run_id"] == run_id]
    _need(len(faults) == 2 and faults[0]["id"] == scope["fault_id"]
          and {f["reason"] for f in faults} == {
              "Claimed preparation/query failed or uncertain: First-target initialization incomplete, uncertain or inconsistent",
              "execution_receipt_failed"}
          and all(f["owner"] == event["owner"] and f["at"] >= event["began_at"] for f in faults),
          "Additional or different faults require independent diagnosis")
    return {"run":dict(run), "scope":dict(scope), "tables":tables}, event, device


def _closed(path, device):
    raw, ref = _file(path)
    results = []
    for line in raw.decode().splitlines():
        row = json.loads(line)
        for item in row.get("result", {}).get("content", []):
            if item.get("type") == "text":
                value = json.loads(item["text"])
                if value.get("status") == "closed":
                    results.append(value)
    _need(bool(results), "Original normal close receipt required")
    # A query-only code repair can close an earlier owner in the same log.
    # Only the final close may settle the terminal owner's exact lifetime.
    close = results[-1]
    cleanup = close.get("cleanup", {})
    _need(close.get("fault_latched") is True and cleanup.get("unresolved_gripper_probe") is None
          and cleanup.get("grasp_states") == {"left":None,"right":None}
          and cleanup.get("requires_fault_latch") is False and cleanup.get("guard_violations") == []
          and cleanup.get("session_transmission_counts") == device.get("session_transmission_counts")
          and all(cleanup.get("arms", {}).get(s, {}).get("status") == "disconnected" for s in ("left", "right")),
          "Close receipt must retain the fault and exact empty-grasp lifetime send counts")
    return {"source":ref, "receipt":close, "physical_stop_verified":None}


def prepare_restart(path, run_id, *, close_log, new_run_id, max_steps=125, max_duration_s=900,
                    budget_mode="preserve_parent_ceiling", clock=time.time):
    """Read-only exact proposal; no activation or implicit authorization."""
    run_id, new_run_id = _identifier(run_id, "run_id"), _identifier(new_run_id, "new_run_id")
    _need(run_id != new_run_id, "The original terminal run cannot be overwritten")
    _need(budget_mode in ("preserve_parent_ceiling", "explicit_user_budget_request"), "Unknown budget policy")
    explicit = budget_mode == "explicit_user_budget_request"
    _need(type(max_steps) is int and 1 <= max_steps <= (1000 if explicit else 128), "Bounded new step allocation required")
    duration = _number(max_duration_s, "max_duration_s", positive=True)
    _need(duration <= (10800 if explicit else 900), "New attempt exceeds its budget policy duration cap")
    source = Path(path).resolve(strict=True)
    db = sqlite3.connect(source.as_uri()+"?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    try:
        db.execute("PRAGMA query_only=ON"); db.execute("BEGIN")
        snapshot, event, device = _snapshot(db, run_id)
        _need(not db.execute("SELECT 1 FROM pair_runs WHERE run_id=?", (new_run_id,)).fetchone(), "New run ID already exists")
    finally:
        db.close()
    now = _number(clock(), "clock")
    run = snapshot["run"]
    _need(now >= run["started_at"]+run["max_duration"] and now >= snapshot["scope"]["last_time"],
          "Entry is only for the expired terminal attempt")
    if not explicit:
        _need(run["steps"] + max_steps <= run["max_steps"], "Preserve the original cumulative step ceiling")
    proposal = {"schema":"piper_terminal_restart_proposal_v1", "database":str(source),
        "created_at":now, "parent_run_id":run_id, "new_run_id":new_run_id,
        "parent_snapshot_sha256":_sha(snapshot), "parent_run":run,
        "parent_scope":snapshot["scope"], "failed_event_id":event["event_id"],
        "failed_event_sha256":_sha(event), "original_returned_frames":device["frame_receipts"],
        "close":_closed(close_log, device), "new_budget":{"max_steps":max_steps,"max_duration_s":duration},
        "cumulative_step_ceiling":run["steps"]+max_steps if explicit else run["max_steps"], "new_authorization":None,
        "reviewed_contract":_current_contract(source.parent.parent, json.loads(run["contract_json"])),
        "hardware_commands_sent":0, "dispatch_authorized":False,
        "old_target_cache_transferred":False, "physical_stop_verified":None,
        "fresh_feedback_and_rgb":"Required by the new host before any task dispatch; not established by this proposal"}
    if explicit:
        proposal.update(budget_policy=budget_mode, authorization_not_before=max(
            event["finished_at"], *(f["at"] for f in snapshot["tables"]["pair_faults"] if f["run_id"] == run_id)))
    return {**proposal,"proposal_sha256":_sha(proposal)}


def _live_control_processes():
    """Conservative local process check, not proof against remote CAN senders."""
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry/"cmdline").read_bytes().split(b"\0")
        except FileNotFoundError:
            continue
        except PermissionError:
            raise PairLedgerError("Cannot inspect current process ownership")
        if any(token == b"robot_tools.server" or b"piper_ctrl" in token or b"piper_driver" in token for token in argv):
            found.append({"pid":int(entry.name),"argv":[x.decode(errors="replace") for x in argv if x]})
    return found


def _current_contract(root, old, *, require_change=True):
    profile = json.loads((root/"configs/robot.json").read_text())
    _need(all(profile.get(k) == old[k] for k in ("arms","cameras","sdk_commit_audited")),
          "Model/device/profile changes are outside this restart")
    source, _ = _file(root/"robot_tools/pair_host.py")
    names = [ast.literal_eval(n.value) for n in ast.walk(ast.parse(source))
             if isinstance(n, ast.Assign) and isinstance(n.value, ast.Tuple)
             and any(isinstance(t, ast.Name) and t.id == "sources" for t in n.targets)]
    _need(len(names) == 1 and isinstance(names[0], tuple), "Current host source manifest cannot be resolved")
    _need(set(old["code"]).issubset(names[0]) and "pair_restart.py" in names[0], "Complete old and restart code manifest required")
    code = {}
    for name in names[0]:
        _need(type(name) is str and Path(name).name == name and name.endswith(".py"), "Local Python source basename required")
        _, ref = _file(root/"robot_tools"/name)
        code[name] = ref["sha256"]
    if require_change:
        _need(code != old["code"], "This entry requires the reviewed code repair")
    return {**copy.deepcopy(old),"code":code}


def activate_restart(proposal, authorization, *, project_root, clock=time.time):
    """Enroll once after explicit human recovery/new-budget authorization.

    This is an administrative trusted-operator API, deliberately absent from
    robot tools. It performs no hardware action and cannot establish readiness.
    Never synthesize authorization from silence, a goal continuation or an old
    restart request made before this failure.
    """
    payload = {k:v for k,v in proposal.items() if k != "proposal_sha256"}
    _need(_sha(payload) == proposal.get("proposal_sha256"), "Proposal digest mismatch")
    canonical = prepare_restart(proposal["database"], proposal["parent_run_id"],
        close_log=proposal["close"]["source"]["path"], new_run_id=proposal["new_run_id"],
        max_steps=proposal["new_budget"]["max_steps"], max_duration_s=proposal["new_budget"]["max_duration_s"],
        budget_mode=proposal.get("budget_policy", "preserve_parent_ceiling"),
        clock=lambda:proposal["created_at"])
    _need(canonical == proposal, "Activation requires the complete canonical reviewed proposal")
    required = {"source","message_id","statement","received_at","decision","proposal_sha256","new_budget"}
    explicit = proposal.get("budget_policy") == "explicit_user_budget_request"
    decision = "authorize_explicit_budget_request" if explicit else "authorize_audited_new_attempt"
    _need(type(authorization) is dict and set(authorization) == required
          and authorization["source"] == "user_message"
          and authorization["decision"] == decision
          and authorization["proposal_sha256"] == proposal["proposal_sha256"]
          and authorization["new_budget"] == proposal["new_budget"], "Explicit matching new-attempt authorization required")
    budget = authorization["new_budget"]
    _need(type(budget) is dict and type(budget.get("max_steps")) is int
          and type(budget.get("max_duration_s")) in (int, float), "Exact finite numeric authorization budget required")
    _identifier(authorization["message_id"], "user message reference")
    _need(type(authorization["statement"]) is str and 1 <= len(authorization["statement"].strip()) <= 8000,
          "Preserve the user's actual confirmation text")
    authorized_at = _number(authorization["received_at"], "authorization time")
    now = _number(clock(), "clock")
    _need(now >= proposal["created_at"], "Activation clock cannot precede the reviewed proposal")
    if explicit:
        # The actual user budget request may precede its subsequently prepared
        # exact proposal. It must still postdate this precise terminal failure.
        _need(proposal["authorization_not_before"] < authorized_at <= now,
              "Explicit budget request must follow the actual terminal fault and cannot be future-dated")
    else:
        _need(proposal["created_at"] <= authorized_at <= now, "Old/future authorization cannot restart this failure")
    root = Path(project_root).resolve(strict=True)
    path = Path(proposal["database"])
    _need(path == root/"runs/pair_sessions.sqlite", "Use the authoritative original project database")
    _need(not _live_control_processes(), "A live local control process blocks administrative enrollment")
    with ExclusiveExecution(path.parent):
        db = sqlite3.connect(path.as_uri()+"?mode=rw", uri=True, isolation_level=None)
        db.row_factory = sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL"); db.execute("BEGIN IMMEDIATE")
            snapshot, event, device = _snapshot(db, proposal["parent_run_id"])
            _need(_sha(snapshot) == proposal["parent_snapshot_sha256"], "Ledger changed after proposal; approval is stale")
            _need(_closed(proposal["close"]["source"]["path"], device) == proposal["close"], "Original close evidence changed")
            contract = _current_contract(root, json.loads(snapshot["run"]["contract_json"]))
            _need(contract == proposal.get("reviewed_contract"), "Reviewed code changed after the restart proposal")
            end = _number(clock(), "clock")
            _need(end >= now and end >= snapshot["scope"]["last_time"], "Clock regressed during enrollment")
            _need(not _live_control_processes(), "Control owner appeared during enrollment")
            _need(_current_contract(root, json.loads(snapshot["run"]["contract_json"])) == contract,
                  "Code changed during enrollment")
            record = {"proposal":proposal,"authorization":authorization,"activated_at":end,
                      "new_contract":contract,"hardware_commands_sent":0,"dispatch_authorized":False,
                      "old_rows_preserved":True,"current_physical_state":"not_observed",
                      "fresh_host_admission_required":True,"physical_stop_verified":None}
            db.execute("CREATE TABLE pair_execution_epochs (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,"
                       "parent_run_id TEXT NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,"
                       "proposal_sha256 TEXT UNIQUE NOT NULL,authorization_sha256 TEXT UNIQUE NOT NULL,record_json TEXT NOT NULL)")
            budget = proposal["new_budget"]
            db.execute("INSERT INTO pair_runs VALUES(?,?,?,?,?,0)", (proposal["new_run_id"],
                _json_object(contract,"new contract"),budget["max_steps"],budget["max_duration_s"],end))
            db.execute("INSERT INTO pair_execution_epochs VALUES(1,?,?,NULL,NULL,NULL,?,?,?,?)",
                (proposal["new_run_id"],proposal["parent_run_id"],end,proposal["proposal_sha256"],_sha(authorization),
                 _json_object(record,"activation audit")))
            committed_at = _number(clock(), "final enrollment clock")
            _need(committed_at >= end, "Clock regressed during administrative writes")
            record["activated_at"] = committed_at
            db.execute("UPDATE pair_runs SET started_at=? WHERE run_id=?",(committed_at,proposal["new_run_id"]))
            db.execute("UPDATE pair_execution_epochs SET last_time=?,record_json=? WHERE ordinal=1",
                       (committed_at,_json_object(record,"activation audit")))
            _need(_number(clock(), "commit clock") >= committed_at, "Clock regressed before administrative commit")
            db.execute("COMMIT")
            return record
        except BaseException:
            if db.in_transaction:
                db.execute("ROLLBACK")
            raise
        finally:
            db.close()
