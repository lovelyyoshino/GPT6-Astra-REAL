"""Attended, startup-only enrollment after a host reboot; old faults stay latched.

No task scope is reset or reactivated. Existing startup owns all physical checks
and exact sends. A durable per-boot/per-arm claim precedes its first mode intent.
The operator must separately attest an arm power cycle and the current scene;
the Linux boot time cannot establish that the robot was power-cycled.
"""
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import time
import uuid

from .execution import ExclusiveExecution, Journal
from .pair_restart import _live_control_processes


CONFIRMATION = "我确认两臂已断电重启，当前空载、无接触、周围净空，并在现场看护"
SCOPE_TABLES = ("pair_endpoint_continuations", "pair_initialization_continuations", "pair_feedback_continuations", "pair_preparation_continuations", "pair_rounds", "pair_predispatch_continuations", "pair_continuations",
                "pair_execution_epochs", "pair_scope")


class HostBootRequired(RuntimeError):
    """Historical host-reboot route cannot cover activity in this Linux boot."""


def boot_identity():
    boot_id = str(uuid.UUID(Path("/proc/sys/kernel/random/boot_id").read_text().strip()))
    started = next(int(line.split()[1]) for line in Path("/proc/stat").read_text().splitlines()
                   if line.startswith("btime "))
    if not 0 < started <= time.time():
        raise RuntimeError("Invalid current host boot time")
    return {"boot_id": boot_id, "started_at": started}


def project_roots(project):
    return sorted({Path(project).resolve(), Path("/home/agilex/piperx_cloth_demo").resolve()})


def check_processes():
    active = _live_control_processes()
    names = {"ros_resume_entry.py", "ros_low_speed_entry.py", "ros_interruptible_joint_entry.py"}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except FileNotFoundError:
            continue
        if any(Path(v.decode(errors="replace")).name in names or
               (Path(v.decode(errors="replace")).name == "runner.py" and
                Path(v.decode(errors="replace")).parent.name == "dual_arm_plug_transfer")
               for v in argv if v):
            active.append({"pid": int(entry.name)})
    if active:
        raise RuntimeError("仍有控制宿主，不能启动独立使能：" + json.dumps(active, ensure_ascii=False))


def ledger_snapshot(path, boot):
    if not path.exists():
        return None
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only=ON")
        db.execute("BEGIN")
        tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if not {"pair_scope", "pair_events", "pair_faults", "pair_runs"} <= tables:
            raise RuntimeError("控制账本结构不完整：" + str(path))
        if "pair_arm_power_cycles" in tables and db.execute(
                "SELECT 1 FROM pair_arm_power_cycles WHERE status!='complete' OR finished_at IS NULL LIMIT 1").fetchone():
            raise RuntimeError("机械臂断电启动仍未决或失败，不能通过电脑重启绕过原回执。")
        if db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone():
            raise RuntimeError("账本有未决发送，不能进行重启后使能。")
        if "pair_holds" in tables and db.execute(
                "SELECT 1 FROM pair_holds WHERE status='pending' LIMIT 1").fetchone():
            raise RuntimeError("账本有未决保持事务。")
        if "pair_grasp_episodes" in tables:
            from .grasp_episode import is_resolved_release
            for row in db.execute("SELECT state_json FROM pair_grasp_episodes"):
                state = json.loads(row[0])
                if state.get("status") != "empty" and not is_resolved_release(state):
                    raise RuntimeError("账本有未解除的持物关系，不能独立重新使能。")
        scopes, times = [], []
        for table in SCOPE_TABLES:
            if table not in tables:
                continue
            for row in db.execute("SELECT * FROM " + table):
                value = dict(row)
                times.append(value["last_time"])
                scopes.append({"table": table, "ordinal": value.get("ordinal", 1),
                               "run_id": value.get("run_id", value.get("active_run_id")),
                               "owner": value["owner"], "fault_id": value["fault_id"],
                               "sha256": hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()})
        for table, column in (("pair_events", "began_at"), ("pair_events", "finished_at"),
                              ("pair_faults", "at"), ("pair_runs", "started_at")):
            value = db.execute("SELECT MAX(" + column + ") FROM " + table).fetchone()[0]
            if value is not None:
                times.append(value)
        if not times or any(type(t) not in (int, float) or not math.isfinite(t) or t < 0 for t in times):
            raise RuntimeError("控制账本时间无效，不能选择重启接续。")
        if any(t >= boot["started_at"] for t in times):
            raise HostBootRequired("控制账本含本次电脑启动后的任务活动；不能按历史会话接续使能。")
        if "pair_reboot_startups" in tables:
            if db.execute("SELECT 1 FROM pair_reboot_startups WHERE boot_id=? AND status!='complete' LIMIT 1",
                          (boot["boot_id"],)).fetchone():
                raise RuntimeError("本次电脑启动已有未决或失败的使能事务，禁止重复发送。")
        return {"database": str(path), "scopes": scopes, "last_task_activity_at": max(times)}


def inspect(project):
    """Metadata only. This is eligibility for live preflight, never a TX permit."""
    check_processes()
    boot = boot_identity()
    ledgers = [s for root in project_roots(project)
               if (s := ledger_snapshot(root / "runs/pair_sessions.sqlite", boot)) is not None]
    if not ledgers:
        raise RuntimeError("没有需要保留的历史账本，请使用普通启动入口。")
    return {"route": "reboot_startup", "boot": boot, "ledgers": ledgers,
            "physical_stop_verified": None, "task_motion_authorized": False}


def reserve(database, enrollment, selected, run_id, record_path):
    with sqlite3.connect(database) as db:
        db.execute("PRAGMA synchronous=FULL")
        db.execute("BEGIN IMMEDIATE")
        db.execute("CREATE TABLE IF NOT EXISTS pair_reboot_startups ("
                   "boot_id TEXT NOT NULL, arm TEXT NOT NULL, run_id TEXT NOT NULL, "
                   "status TEXT NOT NULL, record_path TEXT NOT NULL, started_at REAL NOT NULL, "
                   "finished_at REAL, PRIMARY KEY(boot_id,arm))")
        boot_id = enrollment["boot"]["boot_id"]
        if db.execute("SELECT 1 FROM pair_reboot_startups WHERE boot_id=? AND status!='complete'",
                      (boot_id,)).fetchone():
            raise RuntimeError("本次启动已有未决/失败事务。")
        for side in selected:
            if db.execute("SELECT 1 FROM pair_reboot_startups WHERE boot_id=? AND arm=?", (boot_id, side)).fetchone():
                raise RuntimeError(side + " 本次电脑启动已使用过使能事务，禁止重发。")
            db.execute("INSERT INTO pair_reboot_startups VALUES (?,?,?,?,?,?,NULL)",
                       (boot_id, side, run_id, "pending", str(record_path), time.time()))


def run(service, arm, power_cycle_and_clearance_statement):
    if arm not in ("left", "right", "both") or power_cycle_and_clearance_statement != CONFIRMATION:
        raise ValueError("必须明确选择机械臂并由现场操作者确认机械臂断电重启和当前空载净空。")
    if service.pair_host is not None:
        raise RuntimeError("当前服务仍持有 pair host，不能建立独立启动。")
    from .service import _write
    from .takeover import startup_arms
    from .single_arm_startup import startup_arm
    selected = ["left", "right"] if arm == "both" else [arm]
    with ExitStack() as locks:
        roots = project_roots(service.root)
        lock_roots = set(roots) | {service.root.parent / "piper_right_pick_demo",
                                  Path("/home/agilex/piper_right_pick_demo")}
        for root in sorted(lock_roots):
            if root == service.root or (root / "runs").is_dir():
                locks.enter_context(ExclusiveExecution(root / "runs"))
        enrollment = inspect(service.root)
        run_id, directory = service._new_run("reboot_startup")
        result_path = directory / "result.json"
        database = service.runs / "pair_sessions.sqlite"
        if not database.is_file():
            raise RuntimeError("当前项目无已有账本；不能换数据库接管历史故障。")
        _write(directory / "request.json", {"run_id": run_id, "arm": arm,
               "operator_statement": power_cycle_and_clearance_statement, "enrollment": enrollment,
               "scope": "motor startup only; all old task faults/owners/budgets retained"})
        journal = Journal(directory)
        claimed = False

        def record(event, data):
            nonlocal claimed
            if event in ("mode_request_intent", "enable_request_intent"):
                # Called by the original executor after its new healthy,
                # disabled, stationary baseline and before its TX guard opens.
                if not claimed:
                    if inspect(service.root) != enrollment:
                        raise RuntimeError("启动前账本或电脑启动身份变化。")
                    reserve(database, enrollment, selected, run_id, result_path)
                    claimed = True
                else:
                    # Our own pending receipt is expected after the claim.
                    check_processes()
                    if boot_identity() != enrollment["boot"]:
                        raise RuntimeError("启动身份变化。")
            journal.append(event, **data)

        result = (startup_arms(service.profile, record) if arm == "both" else
                  startup_arm(service.profile, record, arm))
        result.update(run_id=run_id, record_path=str(result_path), reboot_startup=enrollment,
                      task_motion_authorized=False, old_task_faults_preserved=True)
        _write(result_path, result)
        if claimed:
            with sqlite3.connect(database) as db:
                db.execute("PRAGMA synchronous=FULL")
                db.execute("UPDATE pair_reboot_startups SET status=?,finished_at=? WHERE run_id=?",
                           ("complete" if result.get("ok") is True else "failed", time.time(), run_id))
        return result
