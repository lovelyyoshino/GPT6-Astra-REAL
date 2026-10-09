"""Operator-requested software reset, with an immutable SQLite archive.

No hardware access in reset(). The new startup latch survives repeat CLI calls;
only a new host boot may retire it. Old physical grasp/stop facts are UNKNOWN,
never changed to released/stopped. Startup reuses the existing bounded executor.
"""
from contextlib import ExitStack, contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import time
import uuid

from .execution import ExclusiveExecution, Journal
from .reboot_startup import boot_identity, check_processes, project_roots, CONFIRMATION

TABLE = "pair_startup_resets"
POLICY = "Operator invocation of can_enable.sh retires prior software task state; archive original evidence"
TASK_BUDGETS = "pair_reset_task_budgets"


def _completed_startup(db, project):
    """Audit the current reset receipt; never infer a release from software reset."""
    from .pair_restart import _need, _file
    head = _head(db)
    _need(head and head["boot_id"] == boot_identity()["boot_id"]
          and head["status"] == "complete", "Current successful reset startup required")
    archive = Path(head["archive_path"])
    _need(hashlib.sha256(archive.read_bytes()).hexdigest() == head["archive_sha256"],
          "Reset archive changed")
    result_path = Path(head["record_path"])
    _need(result_path.parent.parent == project / "runs" and result_path.name == "result.json",
          "Canonical reset startup receipt required")
    raw, result_ref = _file(result_path)
    result = json.loads(raw)
    raw, request_ref = _file(result_path.with_name("request.json"))
    request = json.loads(raw)
    _need(result.get("ok") is True and result.get("errors") == []
          and result.get("guard_violations") == []
          and result.get("software_reset", {}).get("reset_id") == head["reset_id"]
          and request.get("reset", {}).get("reset_id") == head["reset_id"]
          and request.get("operator_statement") == CONFIRMATION
          and request.get("arm") == "both"
          and request.get("run_id") == result.get("run_id") == result_path.parent.name
          and result.get("operation") == "startup_arms", "Complete bilateral startup receipt required")
    _need(result.get("transmission_counts") == {
        s: {"attempted_frames": 2, "sent_frames": 2, "blocked_frames": 0}
        for s in ("left", "right")}
        and result.get("hardware_commands_sent") == 4
        and result.get("target_commands_sent") == 0
        and result.get("gripper_target_commands_sent") == 0
        and result.get("retries") == 0
        and result.get("last_enable_feedback") == {
            s: {"driver_enabled": [True]*6, "gripper_enabled": False} for s in ("left", "right")}
        and all(result.get("cleanup", {}).get("arms", {}).get(s, {}).get("status") == "disconnected"
                for s in ("left", "right")), "Startup sends, enabled feedback or cleanup differs")
    return {"reset_id": head["reset_id"], "boot_id": head["boot_id"],
            "archive_path": str(archive), "archive_sha256": head["archive_sha256"],
            "result": result_ref, "request": request_ref}


def enroll_task_budget(project, run_id, task, authorization, *, max_steps=1000,
                       max_duration_s=10800, clock=time.time):
    """Administrative first-task enrollment after successful reset, zero device I/O.

    This is not a tool exposed to the inner action model. The caller records
    actual user authorization. No existing run, target, fault or budget is
    modified; the first execution clock starts here, after offline repair.
    """
    from .pair_ledger import _identifier, _json_object, _number
    from .pair_restart import _need, _current_contract
    from .task_roles import resolve_task_roles
    from .feedback_tolerance import task_policy
    project = Path(project).resolve(strict=True)
    run_id = _identifier(run_id, "run_id")
    task = json.loads(_json_object(task, "task"))
    _need(task.get("task_id") == "plug_transfer_left"
          and task.get("roles") == {"left": "task", "right": "task"}, "Explicit plug task required")
    resolve_task_roles(task)
    task_policy(task)
    clearance = task.get("site_context", {}).get("workspace_clearance", {})
    _need(clearance.get("source") == "user" and isinstance(clearance.get("statement"), str)
          and bool(clearance["statement"].strip()), "Actual task clearance statement required")
    _need(type(max_steps) is int and 1 <= max_steps <= 1000, "Maximum is 1000 steps")
    duration = _number(max_duration_s, "duration", positive=True)
    _need(duration <= 10800, "Maximum is 10800 seconds")
    authorization = json.loads(_json_object(authorization, "authorization"))
    _need(set(authorization) == {"source", "message_id", "statement", "received_at", "decision",
                                  "max_steps", "max_duration_s"}
          and authorization["source"] == "user_message"
          and authorization["decision"] == "authorize_explicit_new_task"
          and all(isinstance(authorization[k], str) and authorization[k].strip()
                  for k in ("message_id", "statement"))
          and type(authorization["max_steps"]) is int and authorization["max_steps"] == max_steps
          and authorization["max_duration_s"] == duration, "Explicit matching new-task budget authorization required")
    profile = json.loads((project / "configs/robot.json").read_text())
    base = {k: profile[k] for k in ("arms", "cameras", "sdk_commit_audited")}
    base.update(task=task, code={})
    contract = _current_contract(project, base, require_change=False)
    with locks(project), sqlite3.connect(project / "runs/pair_sessions.sqlite") as db:
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA synchronous=FULL")
        db.execute("BEGIN IMMEDIATE")
        startup = _completed_startup(db, project)
        from .gripper_zero import audit_completed
        maintenance = audit_completed(db)
        tables = _tables(db)
        # Reset and its retired startup latch are the only permitted history.
        # A normal host open, even with no TX, consumes this first-task route.
        for name in sorted(tables):
            if name.startswith("pair_") and name not in (TABLE, "pair_scope", "pair_faults", "pair_gripper_zero_calibrations"):
                _need(db.execute("SELECT COUNT(*) FROM " + _quote(name)).fetchone()[0] == 0,
                      "Existing task activity blocks initial enrollment: " + name)
        scope = db.execute("SELECT * FROM pair_scope WHERE id=1").fetchone()
        _need(scope and scope["owner"] is None and scope["active_run_id"] is None
              and scope["fault_id"] is None, "Clean unowned reset scope required")
        faults = [dict(r) for r in db.execute("SELECT * FROM pair_faults")]
        maintenance_faults = {row['fault_id']:row['run_id'] for row in maintenance}
        _need(all(f['id'] not in maintenance_faults or
                  (f['run_id'] == maintenance_faults[f['id']] and f['reason'] == 'gripper_zero_in_progress'
                   and f['owner'] is None) for f in faults), "Calibration fault history differs")
        faults = [f for f in faults if f['id'] not in maintenance_faults]
        _need(len(faults) == 1 and faults[0]["run_id"] == startup["reset_id"]
              and faults[0]["reason"] == "software_reset_requires_new_startup"
              and faults[0]["owner"] is None, "Only the retired reset latch may precede the new task")
        now = _number(clock(), "clock")
        _need(_number(authorization["received_at"], "authorization time") <= now
              and scope["last_time"] <= now, "Authorization or task clock is in the future")
        _need(_current_contract(project, base, require_change=False) == contract,
              "Code changed during task enrollment")
        check_processes()
        record = dict(schema="piper_reset_initial_task_budget_v1", project_root=str(project),
                      startup=startup, completed_maintenance=maintenance, authorization=authorization, contract=contract,
                      budget=dict(max_steps=max_steps, max_duration_s=duration, started_at=now),
                      required_connection_mode="prepare", hardware_commands_sent=0,
                      fresh_host_admission_required=True, physical_release_verified=None,
                      physical_stop_verified=None)
        db.execute("CREATE TABLE IF NOT EXISTS " + TASK_BUDGETS +
                   " (reset_id TEXT PRIMARY KEY, run_id TEXT UNIQUE NOT NULL, record_json TEXT NOT NULL)")
        db.execute("INSERT INTO " + TASK_BUDGETS + " VALUES(?,?,?)",
                   (startup["reset_id"], run_id, _json_object(record, "task budget")))
        db.execute("INSERT INTO pair_runs VALUES(?,?,?,?,?,0)",
                   (run_id, _json_object(contract, "contract"), max_steps, duration, now))
        return record


def audit_task_budget(db, run_id, *, max_steps, max_duration_s):
    """Recognize the frozen first-task budget without renewing its clock."""
    from .pair_restart import _need
    row = db.execute("SELECT * FROM " + TASK_BUDGETS + " WHERE run_id=?", (run_id,)).fetchone()
    if row is None:
        return False
    record = json.loads(row["record_json"])
    run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
    _need(run is not None and record.get("schema") == "piper_reset_initial_task_budget_v1",
          "Initial task budget record missing")
    budget, auth = record["budget"], record["authorization"]
    from .gripper_zero import audit_completed
    _need(audit_completed(db) == record.get('completed_maintenance', []), "Calibration history changed")
    _need(_completed_startup(db, Path(record["project_root"])) == record["startup"]
          and row["reset_id"] == record["startup"]["reset_id"]
          and json.loads(run["contract_json"]) == record["contract"]
          and run["started_at"] == budget["started_at"]
          and run["max_steps"] == budget["max_steps"] == auth["max_steps"]
          and run["max_duration"] == budget["max_duration_s"] == auth["max_duration_s"]
          and auth["source"] == "user_message" and auth["decision"] == "authorize_explicit_new_task"
          and auth["received_at"] <= budget["started_at"]
          and record["required_connection_mode"] == "prepare",
          "Frozen initial task budget, authorization or startup differs")
    return max_steps == run["max_steps"] and max_duration_s == run["max_duration"]


@contextmanager
def locks(project):
    project = Path(project).resolve()
    roots = set(project_roots(project)) | {project.parent / "piper_right_pick_demo",
                                          Path("/home/agilex/piper_right_pick_demo")}
    with ExitStack() as stack:
        for root in sorted(roots):
            if root == project or (root / "runs").is_dir():
                stack.enter_context(ExclusiveExecution(root / "runs"))
        check_processes()
        yield


def _tables(db):
    return {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}


def _quote(name):
    return '"' + name.replace('"', '""') + '"'


def _head(db):
    if TABLE not in _tables(db):
        return None
    row = db.execute("SELECT * FROM " + TABLE + " ORDER BY created_at DESC LIMIT 1").fetchone()
    return dict(row) if row else None


def _route(row):
    if row["status"] not in ("awaiting_startup", "complete"):
        raise RuntimeError("本次重置后的启动未决或失败；重复运行不能清除或重发，请查看：" + row["record_path"])
    return {"route": "state_reset_startup", "reset_id": row["reset_id"],
            "status": row["status"], "archive_path": row["archive_path"],
            "archive_sha256": row["archive_sha256"], "record_path": row["record_path"]}


def inspect(project):
    path = Path(project) / "runs/pair_sessions.sqlite"
    if not path.exists():
        return None
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        head = _head(db)
        if head and head["boot_id"] == boot_identity()["boot_id"]:
            if hashlib.sha256(Path(head["archive_path"]).read_bytes()).hexdigest() != head["archive_sha256"]:
                raise RuntimeError("原账本归档缺失或已改变，不能启动。")
            return _route(head)
    return None


def reset(project):
    """Archive and transactionally clear ONLY a prior-boot task database.

    Live processes, current-boot activity, pending/unknown transactions and
    another project's active ledger are not reset. No database is substituted
    at a new pathname, and a failed archive cannot clear any original rows.
    """
    from .pair_ledger import platform_state
    project = Path(project).resolve()
    path = project / "runs/pair_sessions.sqlite"
    with locks(project):
        for other in project_roots(project):
            if other != project:
                state = platform_state(other / "runs/pair_sessions.sqlite")
                if state and (state["owner"] or state["fault"] or state["pending_events"]):
                    raise RuntimeError("原项目仍有控制状态；本脚本不清除其他项目账本：" + str(other))
        if not path.exists():
            return {"route": "ordinary_startup", "reset_performed": False}
        boot = boot_identity()
        with sqlite3.connect(path, isolation_level=None) as db:
            db.row_factory = sqlite3.Row
            db.execute("PRAGMA synchronous=FULL")
            db.execute("BEGIN IMMEDIATE")
            try:
                head = _head(db)
                if head and head["boot_id"] == boot["boot_id"]:
                    result = _route(head)
                    db.rollback()
                    return dict(result, reset_performed=False)
                if head and head["status"] == "pending":
                    raise RuntimeError("先前重置启动仍未决，不能用再次重启抹掉发送状态。")
                if head and head["status"] == "failed":
                    from .arm_power_cycle import _known_sends
                    _known_sends(json.loads(Path(head["record_path"]).read_text()))
                tables = _tables(db)
                if not {"pair_scope", "pair_runs", "pair_events", "pair_faults"} <= tables:
                    raise RuntimeError("控制账本结构不完整，未重置。")
                if any(not n.startswith("pair_") and n != "sqlite_sequence" for n in tables):
                    raise RuntimeError("账本包含非 pair 表，未重置。")
                if db.execute("SELECT 1 FROM pair_events WHERE status!='complete' OR finished_at IS NULL "
                              "OR success IS NULL LIMIT 1").fetchone():
                    raise RuntimeError("账本存在未决发送，不能通过软件重置重试。")
                for name in ("pair_holds", "pair_reboot_startups", "pair_arm_power_cycles"):
                    if name in tables and db.execute("SELECT 1 FROM " + name +
                            " WHERE status='pending' OR finished_at IS NULL LIMIT 1").fetchone():
                        raise RuntimeError("账本存在未决保持或启动事务，未重置。")
                # Also inspect management scopes and claims, not just pair_runs.
                activity = []
                for name in tables:
                    if not name.startswith("pair_"):
                        continue
                    columns = {r[1] for r in db.execute("PRAGMA table_info(" + _quote(name) + ")")}
                    for column in columns & {"last_time", "started_at", "began_at", "finished_at", "at",
                                             "created_at", "recorded_at", "requested_at", "attempted_at", "returned_at"}:
                        value = db.execute("SELECT MAX(" + _quote(column) + ") FROM " + _quote(name)).fetchone()[0]
                        if value is not None:
                            activity.append(value)
                if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in activity):
                    raise RuntimeError("账本时间无效，未重置。")
                if any(v >= boot["started_at"] for v in activity):
                    raise RuntimeError("账本包含本次电脑启动后的活动；保留当前记录，不自动重置。可用 --keep-state 接续。")
                if not db.execute("SELECT 1 FROM pair_runs LIMIT 1").fetchone() and not head:
                    db.rollback()
                    return {"route": "ordinary_startup", "reset_performed": False}
                reset_id = "state_reset_" + uuid.uuid4().hex
                directory = project / "runs" / reset_id
                directory.mkdir()
                archive = directory / "pair_sessions.before.sqlite"
                # BEGIN IMMEDIATE excludes writers; a separate reader sees the
                # same committed snapshot, including committed WAL contents.
                with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as source:
                    with sqlite3.connect(archive) as target:
                        source.backup(target)
                        if target.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
                            raise RuntimeError("账本备份完整性检查失败，未重置。")
                with archive.open("rb") as stream:
                    os.fsync(stream.fileno())
                digest = hashlib.sha256(archive.read_bytes()).hexdigest()
                from .service import _write
                _write(directory / "request.json", {"reset_id": reset_id, "boot": boot,
                       "policy": POLICY, "database": str(path), "archive_path": str(archive),
                       "archive_sha256": digest, "physical_release_verified": None,
                       "physical_stop_verified": None, "hardware_commands_sent": 0})
                descriptor = os.open(str(directory.parent), os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                # Recheck processes immediately before mutation; locks remain held.
                check_processes()
                for name in sorted(tables):
                    if name.startswith("pair_"):
                        db.execute("DELETE FROM " + _quote(name))
                if "sqlite_sequence" in tables:
                    db.execute("DELETE FROM sqlite_sequence WHERE name LIKE 'pair_%'")
                db.execute("CREATE TABLE IF NOT EXISTS " + TABLE + " (reset_id TEXT PRIMARY KEY, "
                           "boot_id TEXT NOT NULL,created_at REAL NOT NULL,archive_path TEXT NOT NULL, "
                           "archive_sha256 TEXT NOT NULL,status TEXT NOT NULL,record_path TEXT NOT NULL)")
                now = time.time()
                db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES (?,NULL,?,?)",
                           (reset_id, "software_reset_requires_new_startup", now))
                fault = db.execute("SELECT last_insert_rowid()").fetchone()[0]
                db.execute("INSERT INTO pair_scope VALUES(1,NULL,NULL,?,?)", (fault, now))
                db.execute("INSERT INTO " + TABLE + " VALUES(?,?,?,?,?,?,?)",
                           (reset_id, boot["boot_id"], now, str(archive), digest,
                            "awaiting_startup", str(directory / "request.json")))
                result = _route(_head(db))
                db.commit()
                return dict(result, reset_performed=True)
            except BaseException:
                db.rollback()
                raise


def run_startup(service, arm, power_cycle_and_clearance_statement):
    """New startup only; preserve reset latch until a complete live receipt."""
    if arm not in ("left", "right", "both") or power_cycle_and_clearance_statement != CONFIRMATION:
        raise ValueError("重置后的使能需要确认双臂已断电重启、当前空载无接触并现场看护。")
    if service.pair_host is not None:
        raise RuntimeError("当前服务仍持有控制宿主。")
    from .service import _write
    from .single_arm_startup import startup_arm
    from .takeover import startup_arms, _Startup
    with locks(service.root):
        enrollment = inspect(service.root)
        if not enrollment or enrollment["status"] != "awaiting_startup":
            raise RuntimeError("本次重置已尝试启动或缺少重置记录，不重发。")
        run_id, directory = service._new_run("reset_startup")
        result_path = directory / "result.json"
        _write(directory / "request.json", {"run_id": run_id, "arm": arm,
               "operator_statement": power_cycle_and_clearance_statement, "reset": enrollment})
        journal = Journal(directory)
        claimed = False
        latest = None
        database = service.runs / "pair_sessions.sqlite"

        def record(event, data):
            nonlocal claimed, latest
            if event == "feedback":
                latest = data
            if event in ("mode_request_intent", "enable_request_intent"):
                check_processes()
                if not claimed:
                    # Reset retires both task roles, even with a single selected
                    # motor. An enabled passive arm could still be supporting a load.
                    if not latest or any(latest[s]["arm_status"]["ctrl_mode"] != 0 or
                            any(v is not False for v in _Startup.enable_flags(latest[s]))
                            for s in ("left", "right")):
                        raise RuntimeError("全局软件重置后的首次启动要求双臂待机且关节、夹爪全部失能。")
                    if inspect(service.root) != enrollment:
                        raise RuntimeError("重置记录在发送前发生变化。")
                    with sqlite3.connect(database) as db:
                        db.execute("PRAGMA synchronous=FULL")
                        changed = db.execute("UPDATE " + TABLE + " SET status='pending',record_path=? "
                                             "WHERE reset_id=? AND status='awaiting_startup'",
                                             (str(result_path), enrollment["reset_id"])).rowcount
                        if changed != 1:
                            raise RuntimeError("重置启动已经被其他请求占用。")
                    claimed = True
            journal.append(event, **data)

        result = (startup_arms(service.profile, record) if arm == "both" else
                  startup_arm(service.profile, record, arm))
        result.update(run_id=run_id, record_path=str(result_path), software_reset=enrollment,
                      task_motion_authorized=False, physical_release_verified=None,
                      physical_stop_verified=None, old_history_archived=True)
        _write(result_path, result)
        if claimed:
            with sqlite3.connect(database) as db:
                db.execute("PRAGMA synchronous=FULL")
                db.execute("UPDATE " + TABLE + " SET status=? WHERE reset_id=? AND status='pending'",
                           ("complete" if result.get("ok") is True else "failed", enrollment["reset_id"]))
                if result.get("ok") is True:
                    db.execute("UPDATE pair_scope SET fault_id=NULL,last_time=? WHERE id=1", (time.time(),))
        return result
