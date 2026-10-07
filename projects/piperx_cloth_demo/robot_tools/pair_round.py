"""Explicit new-budget enrollment after a healthy, detached, expired round.

Trusted administrative API, never a robot dispatch tool. prepare_round is
read-only; activate_round appends a new run/scope under a matching user-message
authorization. started_at is frozen before preparation, so repair time consumes
the new budget. Old rows, faults, targets and source evidence are never changed.

Operator API (Python 3.10, from this project): call prepare_round with the
authoritative runs/pair_sessions.sqlite, parent/new run IDs, final close log,
fixed started_at and budget. Then activate_round(proposal, authorization,
project_root=...). Authorization has source='user_message', message_id, actual
statement/received_at, decision='authorize_explicit_new_round', proposal_sha256
and the exact proposal.new_budget. Only a new explicit user instruction to
start timing after repair permits budget_start_policy=
'after_repair_before_online_execution' in BOTH proposal and authorization;
the operator chooses started_at after repair and before opening the new host.
Activation never moves that timestamp. Neither API opens devices or grants
cached targets, source records, readiness, task success or physical stopping.
"""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import time

from .execution import ExclusiveExecution
from .pair_ledger import _execution_scope, _hold_frames, _identifier, _json_object, _number
from .pair_restart import _current_contract as _restart_current_contract
from .pair_restart import _file, _live_control_processes, _need, _sha


TABLE = "pair_rounds"


def _current_contract(root, old):
    contract = _restart_current_contract(root, old, require_change=False)
    _need("pair_round.py" in contract["code"], "New-round manager must be in the host source contract")
    return contract


def _counts(value):
    _need(type(value) is dict and set(value) == {"left", "right"}, "Exact pair counters required")
    for row in value.values():
        _need(type(row) is dict and set(row) == {"attempted_frames", "sent_frames", "blocked_frames"}
              and all(type(n) is int and n >= 0 for n in row.values())
              and row["attempted_frames"] == row["sent_frames"] and row["blocked_frames"] == 0,
              "Unknown, blocked or partial CAN attempts forbid new-round enrollment")
    return value


def _frames(frames, raw, event):
    _need(type(frames) is list and len(frames) == 4
          and [f.get("frame") for f in frames] == _hold_frames(raw)
          and all(f.get("outcome") == "returned" for f in frames), "Four complete frame returns required")
    times = [_number(f.get("returned_at"), "frame return") for f in frames]
    _need(times == sorted(times) and event["began_at"] <= times[0] <= times[-1] <= event["finished_at"],
          "Frame chronology must match its original event")


def _snapshot(db, run_id):
    from .grasp_episode import is_resolved_release
    from .joint_sources import _validated_limits
    table, key, ordinal, scope = _execution_scope(db, run_id, writable=True)
    run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
    _need(run is not None and scope is not None and scope["owner"] is None
          and scope["active_run_id"] is None and scope["fault_id"] is None,
          "Current effective run must be cleanly detached without an active fault")
    latest = db.execute("SELECT run_id FROM pair_runs ORDER BY started_at DESC,run_id DESC LIMIT 1").fetchone()
    _need(latest is not None and latest[0] == run_id, "Only the latest effective round may authorize its successor")
    tables = {}
    for item in db.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"):
        name = item[0]
        if name.startswith("pair_"):
            _need(name.replace("_", "").isalnum(), "Unexpected ledger table")
            tables[name] = sorted([dict(row) for row in db.execute('SELECT * FROM "'+name+'"')],
                                 key=lambda row: json.dumps(row, sort_keys=True))
    _need(not any(e["status"] != "complete" for e in tables["pair_events"]), "Pending event blocks enrollment")
    for name in ("pair_holds", "pair_hold_requests", "pair_hold_frames"):
        _need(not tables.get(name), "Hold transactions are outside this healthy new-round entry")
    for row in tables.get("pair_grasp_episodes", []):
        state = json.loads(row["state_json"])
        _need(state["status"] == "empty" or is_resolved_release(state), "Unresolved grasp blocks new round")
    events = sorted([e for e in tables["pair_events"] if e["run_id"] == run_id], key=lambda e:e["step"])
    _need(events and events[-1]["step"] == run["steps"], "Current run needs its complete final event")
    for historical in tables["pair_runs"]:
        steps=sorted(e["step"] for e in tables["pair_events"] if e["run_id"] == historical["run_id"])
        _need(steps == list(range(1,historical["steps"]+1)), "All prior run event history must remain contiguous")
    owner = events[-1]["owner"]
    owned = [e for e in events if e["owner"] == owner]
    _need([e["step"] for e in owned] == list(range(owned[0]["step"], run["steps"]+1)),
          "Retired owner's events must form the latest uninterrupted suffix")
    _need(not any(f["owner"] == owner for f in tables["pair_faults"]), "Latest owner had a fault; not healthy closure")
    totals = {s:{"attempted_frames":0,"sent_frames":0,"blocked_frames":0} for s in ("left","right")}
    for e in owned:
        _need(e["success"] == 1 and e["finished_at"] is not None
              and hashlib.sha256(e["payload_json"].encode()).hexdigest() == e["payload_digest"],
              "Successful immutable event receipts required for the retired owner")
        p, r = json.loads(e["payload_json"]), json.loads(e["receipt_json"])
        _need(r.get("ok") is True and r.get("guard_violations") == [] and not r.get("errors")
              and r.get("hold_receipt") is None, "Uncertain, failed or hold action blocks enrollment")
        counts = _counts(r.get("transmission_counts"))
        sent = sum(v["sent_frames"] for v in counts.values())
        _need(type(r.get("hardware_commands_sent")) is int and r["hardware_commands_sent"] == sent,
              "Event transmission count mismatch")
        for name in ("enable_commands_sent", "stop_commands_sent"):
            _need(type(r.get(name)) is int and r[name] == 0, "Only existing target/query routes are covered")
        kind = p.get("kind")
        if kind == "query":
            _need(p.get("request") == {"operation":"inspect_joint_limits"} and sent == 12
                  and type(r.get("actuator_commands_sent")) is int and r["actuator_commands_sent"] == 0,
                  "Only complete non-actuating limit queries are covered")
            for name, expected in (("joint_limit_queries_attempted",12),("joint_limit_queries_sent",12),
                                   ("target_commands_sent",0),("mode_commands_sent",0)):
                _need(type(r.get(name)) is int and r[name] == expected, "Query counter mismatch: "+name)
            _validated_limits({**r,"run_id":run_id,"owner":owner,"bindings":p["bindings"]},
                run_id=run_id,owner=owner,bindings=p["bindings"],now=e["finished_at"])
            _need(e["began_at"] <= r["began_at"] <= r["ended_at"] <= e["finished_at"],
                  "Raw query evidence must lie inside its claimed event")
        elif kind == "initialization":
            plan=r.get("initialization_plan",{})
            _need(sent == 4 and r.get("cache_established") is True
                  and p.get("expected_target_raw") == plan.get("target_raw"), "Complete initialization required")
            _frames(r.get("frame_receipts"),plan.get("target_raw"),e)
        elif kind == "joint":
            original=r.get("original_event",{})
            _need(sent == 4 and original.get("send_state") == "all_frames_returned"
                  and original.get("event_id") == e["event_id"] and r.get("arrival_confirmed") is True,
                  "Only completely returned and observed-arrived joint events are covered")
            _frames(original.get("frame_receipts"),original.get("target_raw"),e)
        elif kind == "gripper":
            _need(sent == 1 and r.get("target_calls_sent") == 1 and r.get("arrival_confirmed") is True
                  and r.get("feedback_all_after_send") is True, "Complete observed jaw target required")
        else:
            _need(False,"Unsupported retired-owner event kind")
        for side in totals:
            for field in totals[side]: totals[side][field] += counts[side][field]
        _need(_counts(r.get("session_transmission_counts")) == totals, "Retired-owner lifetime mismatch")
    hashes={name:hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()
            for name,rows in tables.items()}
    owners = {row[field] for rows in tables.values() for row in rows for field in ("owner", "previous_owner")
              if row.get(field)}
    for rows in tables.values():
        for row in rows:
            if row.get("retired_owners_json"): owners.update(json.loads(row["retired_owners_json"]))
    owners=sorted(owners)
    summary={"run":dict(run),"scope_table":table,"scope_key":key,"scope_ordinal":ordinal,
        "scope":{k:scope[k] for k in scope.keys() if k not in ("record_json", "contract_json")},
        "effective_contract":json.loads(scope["contract_json"] if "contract_json" in scope.keys() else run["contract_json"]),
        "retired_owner":owner,"retired_owners":owners,"session_transmission_counts":totals,
        "last_finished_at":owned[-1]["finished_at"],"cumulative_prior_steps":sum(r["steps"] for r in tables["pair_runs"]),
        "table_sha256":hashes,"table_rows":{name:len(rows) for name,rows in tables.items()}}
    return summary


def _closed(path, snapshot):
    raw, ref = _file(path); values=[]
    try: rows=[json.loads(raw)]
    except ValueError: rows=[json.loads(line) for line in raw.decode().splitlines()]
    for row in rows:
        if row.get("status") == "closed":values.append(row)
        for item in row.get("result",{}).get("content",[]):
            if item.get("type") == "text":
                value=json.loads(item["text"])
                if value.get("status") == "closed":values.append(value)
    _need(values,"Normal close receipt required")
    value=values[-1];cleanup=value.get("cleanup",{})
    _need(value.get("fault_latched") is False and cleanup.get("requires_fault_latch") is False
          and cleanup.get("guard_violations") == [] and cleanup.get("unresolved_gripper_probe") is None
          and cleanup.get("grasp_states") == {"left":None,"right":None}
          and all(cleanup.get("arms",{}).get(s,{}).get("status") == "disconnected" for s in ("left","right"))
          and _counts(cleanup.get("session_transmission_counts")) == snapshot["session_transmission_counts"],
          "Healthy close must account for the exact retired-owner lifetime")
    return {"source":ref,"receipt":value,"retired_owner":snapshot["retired_owner"],"physical_stop_verified":None}


def prepare_round(path, parent_run_id, *, close_log, new_run_id, started_at,
                  max_steps=500, max_duration_s=3600, budget_start_policy="include_repair_time", clock=time.time):
    parent_run_id=_identifier(parent_run_id,"parent run");new_run_id=_identifier(new_run_id,"new run")
    _need(parent_run_id != new_run_id,"New round requires a distinct run ID")
    _need(budget_start_policy in ("include_repair_time", "after_repair_before_online_execution"),
          "Unknown new-round time policy")
    _need(type(max_steps) is int and 1 <= max_steps <= 500,"New round maximum is 500 steps")
    duration=_number(max_duration_s,"duration",positive=True);start=_number(started_at,"started_at")
    _need(duration <= 3600,"New round maximum is 3600 seconds")
    now=_number(clock(),"clock");deadline=_number(start+duration,"deadline")
    source=Path(path).resolve(strict=True)
    db=sqlite3.connect(source.as_uri()+"?mode=ro",uri=True);db.row_factory=sqlite3.Row
    try:
        db.execute("PRAGMA query_only=ON");db.execute("BEGIN")
        snap=_snapshot(db,parent_run_id)
        _need(not db.execute("SELECT 1 FROM pair_runs WHERE run_id=?",(new_run_id,)).fetchone(),"New round already exists")
    finally:db.close()
    prior=snap["run"];not_before=max(prior["started_at"]+prior["max_duration"],snap["scope"]["last_time"])
    _need(not_before <= start <= now < deadline,"Expired parent, fixed begun new window and unexpired new deadline required")
    proposal={"schema":"piper_explicit_clean_round_v1","database":str(source),"created_at":now,
        "parent_run_id":parent_run_id,"new_run_id":new_run_id,"snapshot":snap,"snapshot_sha256":_sha(snap),
        "parent_run":prior,"close":_closed(close_log,snap),"authorization_not_before":not_before,
        "new_budget":{"max_steps":max_steps,"max_duration_s":duration,"started_at":start},"deadline_s":deadline,
        "budget_policy":"explicit_user_new_round","budget_start_policy":budget_start_policy,
        "cumulative_step_ceiling":snap["cumulative_prior_steps"]+max_steps,
        "reviewed_contract":_current_contract(source.parent.parent,snap["effective_contract"]),
        "hardware_commands_sent":0,"dispatch_authorized":False,"cache_or_limits_transferred":False,
        "physical_stop_verified":None,"fresh_host_admission_required":True}
    return {**proposal,"proposal_sha256":_sha(proposal)}


def activate_round(proposal, authorization, *, project_root, clock=time.time):
    _need(_sha({k:v for k,v in proposal.items() if k != "proposal_sha256"}) == proposal.get("proposal_sha256"),"Proposal digest mismatch")
    budget=proposal["new_budget"]
    canonical=prepare_round(proposal["database"],proposal["parent_run_id"],close_log=proposal["close"]["source"]["path"],
        new_run_id=proposal["new_run_id"],clock=lambda:proposal["created_at"],
        budget_start_policy=proposal["budget_start_policy"],**budget)
    _need(canonical == proposal,"Canonical unchanged proposal required")
    required={"source","message_id","statement","received_at","decision","proposal_sha256","new_budget"}
    after_repair=proposal["budget_start_policy"] == "after_repair_before_online_execution"
    if after_repair: required.add("budget_start_policy")
    _need(type(authorization) is dict and set(authorization)==required
          and authorization["source"]=="user_message" and authorization["decision"]=="authorize_explicit_new_round"
          and authorization["proposal_sha256"]==proposal["proposal_sha256"]
          and (not after_repair or authorization["budget_start_policy"] == proposal["budget_start_policy"])
          and _json_object(authorization["new_budget"],"authorized budget")==_json_object(budget,"budget"),
          "Exact new user-message budget authorization required")
    _identifier(authorization["message_id"],"user message reference")
    _need(type(authorization["statement"]) is str and 1 <= len(authorization["statement"].strip()) <= 8000,
          "Actual user statement required")
    at=_number(authorization["received_at"],"authorization time");now=_number(clock(),"clock")
    _need(proposal["authorization_not_before"] <= at <= budget["started_at"]
          and (after_repair or at == budget["started_at"])
          and proposal["created_at"] <= now < proposal["deadline_s"],"New authorization/window chronology differs")
    root=Path(project_root).resolve(strict=True);path=Path(proposal["database"])
    _need(path==root/"runs/pair_sessions.sqlite","Authoritative project database required")
    _need(not _live_control_processes(),"Live control process blocks enrollment")
    with ExclusiveExecution(path.parent):
        db=sqlite3.connect(path.as_uri()+"?mode=rw",uri=True,isolation_level=None);db.row_factory=sqlite3.Row
        try:
            db.execute("PRAGMA synchronous=FULL");db.execute("BEGIN IMMEDIATE")
            snap=_snapshot(db,proposal["parent_run_id"])
            _need(_sha(snap)==proposal["snapshot_sha256"],"Old history changed after proposal")
            _need(_closed(proposal["close"]["source"]["path"],snap)==proposal["close"],"Close evidence changed")
            _need(_current_contract(root,snap["effective_contract"])==proposal["reviewed_contract"],"Reviewed code changed")
            _need(not _live_control_processes(),"Control process appeared during enrollment")
            db.execute("CREATE TABLE IF NOT EXISTS pair_rounds (ordinal INTEGER PRIMARY KEY,run_id TEXT UNIQUE NOT NULL,"
                "parent_run_id TEXT UNIQUE NOT NULL,owner TEXT,active_run_id TEXT,fault_id INTEGER,last_time REAL NOT NULL,"
                "retired_owners_json TEXT NOT NULL,proposal_sha256 TEXT UNIQUE NOT NULL,authorization_sha256 TEXT UNIQUE NOT NULL,"
                "record_json TEXT NOT NULL)")
            ordinal=db.execute("SELECT COALESCE(MAX(ordinal),0)+1 FROM pair_rounds").fetchone()[0]
            record={"proposal":proposal,"authorization":authorization,"activated_at":now,
                "new_contract":proposal["reviewed_contract"],"old_rows_preserved":True,"new_budget_allocated":True,
                "hardware_commands_sent":0,"dispatch_authorized":False,"physical_stop_verified":None,
                "cache_or_limits_transferred":False,"fresh_host_admission_required":True}
            db.execute("INSERT INTO pair_runs VALUES(?,?,?,?,?,0)",(proposal["new_run_id"],
                _json_object(record["new_contract"],"new contract"),budget["max_steps"],budget["max_duration_s"],budget["started_at"]))
            db.execute("INSERT INTO pair_rounds VALUES(?,?,?,NULL,NULL,NULL,?,?,?,?,?)",(ordinal,proposal["new_run_id"],
                proposal["parent_run_id"],now,json.dumps(snap["retired_owners"]),proposal["proposal_sha256"],_sha(authorization),_json_object(record,"new round")))
            _need(_current_contract(root,snap["effective_contract"])==record["new_contract"],"Code changed during enrollment")
            final=_number(clock(),"final clock")
            _need(now <= final < proposal["deadline_s"],"Clock rollback or new deadline reached")
            record["activated_at"]=final
            db.execute("UPDATE pair_rounds SET last_time=?,record_json=? WHERE ordinal=?",(final,_json_object(record,"new round"),ordinal))
            check=_number(clock(),"commit clock")
            _need(final <= check < proposal["deadline_s"],"Clock/deadline changed during durable write")
            record["activated_at"]=check
            db.execute("UPDATE pair_rounds SET last_time=?,record_json=? WHERE ordinal=?",(check,_json_object(record,"new round"),ordinal))
            db.execute("COMMIT");return record
        except BaseException:
            if db.in_transaction:db.execute("ROLLBACK")
            raise
        finally:db.close()
