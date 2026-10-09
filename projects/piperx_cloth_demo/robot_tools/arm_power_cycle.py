"""Startup after an explicitly attested ARM power cycle, without a host reboot.

Only the existing once-only mode/enable executors touch devices. This module
archives a new startup claim while leaving task owners, faults and budgets
untouched. It cannot resume motion or replay a target from the prior session.
"""
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import time

from .execution import ExclusiveExecution, Journal
from .pair_ledger import _execution_scope
from .reboot_startup import boot_identity, check_processes, project_roots

TABLE = "pair_arm_power_cycles"


def audit_completed_startup_history(db):
    """Validate a later startup without making it a task-recovery grant.

    Used only when reconstructing the historical parent in memory. Every
    other pair table must still equal the startup's pre-send snapshot. The
    real database is never edited, and current fault/target checks stay in
    force. A subsequent task transition needs its own admission contract.
    """
    names = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    if TABLE not in names:
        return None
    rows = [dict(r) for r in db.execute("SELECT * FROM " + TABLE)]
    if len(rows) != 1:
        raise RuntimeError("Exactly one completed arm-cycle startup is covered by historical projection")
    row = rows[0]
    if row['status'] != 'complete' or row['finished_at'] is None or row['arm'] not in ('left', 'right'):
        raise RuntimeError("Only a complete single-arm startup can be projected from parent history")
    path = Path(row['record_path']).resolve(strict=True)
    result_raw = path.read_bytes()
    request_raw = path.with_name('request.json').read_bytes()
    journal_raw = path.with_name('events.jsonl').read_bytes()
    result, request = json.loads(result_raw), json.loads(request_raw)
    journal = [json.loads(line) for line in journal_raw.decode().splitlines()]
    enrollment = request.get('enrollment', {})
    side = row['arm']; peer = 'right' if side == 'left' else 'left'
    required = (path.name == 'result.json' and path.parent.name == result.get('run_id') == request.get('run_id')
        and path.parent.parent.name == 'runs' and result.get('record_path') == str(path)
        and request.get('power_cycle_id') == result.get('power_cycle_id') == row['cycle_id']
        and request.get('arm') == result.get('arm') == result.get('selected_arm') == side
        and result.get('passive_arm') == peer
        and request.get('operator_statement') == row['statement'] == confirmation(side)
        and enrollment.get('route') == 'arm_power_cycle_startup'
        and result.get('arm_power_cycle') == enrollment and _digest(enrollment) == row['enrollment_sha256']
        and result.get('ok') is True and result.get('operation') == 'startup_arm'
        and result.get('status') == 'selected_arm_joints_enabled_observed_not_task_ready'
        and result.get('old_task_faults_preserved') is True and result.get('task_motion_authorized') is False
        and result.get('motion_gate_unlocked') is False and result.get('task_motion_ready') is False
        and result.get('physical_stop_verified') is None
        and result.get('errors') == result.get('guard_violations') == [])
    if not required:
        raise RuntimeError("Arm-cycle request, result and completed claim disagree")
    for key, value in {'hardware_commands_sent': 2, 'enable_commands_sent': 1,
                       'target_commands_sent': 0, 'gripper_target_commands_sent': 0,
                       'stop_commands_sent': 0, 'passive_arm_commands_sent': 0, 'retries': 0}.items():
        if type(result.get(key)) is not int or result[key] != value:
            raise RuntimeError("Unexpected startup command count: " + key)
    counts = {s: {'attempted_frames': 2 if s == side else 0, 'sent_frames': 2 if s == side else 0,
                  'blocked_frames': 0} for s in ('left', 'right')}
    kinds = {s: {k: {'attempted_frames': int(s == side), 'sent_frames': int(s == side)}
                 for k in ('mode', 'enable')} for s in ('left', 'right')}
    if result.get('transmission_counts') != counts or result.get('transmission_counts_by_kind') != kinds:
        raise RuntimeError("Startup per-arm/per-kind counts disagree")
    allowed = {'operation_started', 'connected_passively', 'feedback', 'mode_request_intent',
               'mode_frame_sent_unconfirmed', 'can_control_observed', 'enable_request_intent',
               'enable_frame_sent_unconfirmed', 'joints_enabled_observed', 'passive_arm_state_preserved'}
    if not journal or any(r.get('event') not in allowed for r in journal):
        raise RuntimeError("Unaccounted startup journal events")
    stamps = [r['unix_s'] for r in journal]
    if (any(type(t) not in (int, float) or not math.isfinite(t) for t in stamps)
            or stamps != sorted(stamps) or not stamps[-1] <= row['finished_at']):
        raise RuntimeError("Startup journal chronology differs")
    wire = [r for r in journal if r['event'] in {'mode_request_intent', 'mode_frame_sent_unconfirmed',
                                               'enable_request_intent', 'enable_frame_sent_unconfirmed'}]
    if [r['event'] for r in wire] != ['mode_request_intent', 'mode_frame_sent_unconfirmed',
                                    'enable_request_intent', 'enable_frame_sent_unconfirmed']:
        raise RuntimeError("Exactly one complete mode/enable attempt required")
    for i, (arbitration_id, payload) in enumerate(((0x151, '0100010000000000'), (0x471, '0702000000000000'))):
        intent, sent = wire[2*i:2*i+2]
        expected = {s: {'attempted_frames': i+1 if s == side else 0, 'sent_frames': i+1 if s == side else 0,
                        'blocked_frames': 0} for s in ('left', 'right')}
        if not (intent.get('side') == sent.get('side') == side and intent.get('arbitration_id') == arbitration_id
                and intent.get('data_hex') == payload and sent.get('transmission_counts') == expected
                and row['created_at'] <= intent['unix_s'] <= sent['sent_at'] <= sent['unix_s'] <= row['finished_at']):
            raise RuntimeError("Startup wire identity, chronology or returned counts differ")
    before, after = result['before'], result['after']
    feedback = [{s: r[s] for s in ('left', 'right')} for r in journal if r['event'] == 'feedback']
    if (type(result.get('samples')) is not int or len(feedback) != result['samples']
            or before not in feedback or not feedback or feedback[-1] != after):
        raise RuntimeError("Startup result does not match its original feedback journal")
    if not (before[side]['arm_status']['ctrl_mode'] == 0 and after[side]['arm_status']['ctrl_mode'] == 1
            and before[side]['gripper']['foc_status']['driver_enable_status'] is False
            and all(before[side]['drivers'][str(i)]['foc_status']['driver_enable_status'] is False
                    and after[side]['drivers'][str(i)]['foc_status']['driver_enable_status'] is True for i in range(1, 7))
            and all(result['cleanup']['arms'][s]['status'] == 'disconnected' for s in ('left', 'right'))):
        raise RuntimeError("Startup disabled baseline, enabled result or cleanup evidence differs")
    sources = enrollment.get('ledgers', [])
    canonical = str(path.parent.parent / 'pair_sessions.sqlite')
    matching = [s for s in sources if s.get('database') == canonical]
    if len(matching) != 1 or matching[0].get('prior_arm_power_cycles') != []:
        raise RuntimeError("First startup must bind its authoritative unchanged task ledger")
    source = matching[0]
    current = {}
    for name in sorted(names):
        if not name.startswith('pair_') or name == TABLE:
            continue
        quoted = '"' + name.replace('"', '""') + '"'
        values = [dict(r) for r in db.execute('SELECT * FROM ' + quoted)]
        current[name] = _digest(sorted(values, key=lambda r: json.dumps(r, sort_keys=True)))
    if current != source.get('table_sha256'):
        raise RuntimeError("Task history changed after the arm-cycle startup snapshot")
    run_id = source.get('state', {}).get('active_run_id')
    _, _, _, scope = _execution_scope(db, run_id, writable=True)
    if scope is None or {k: scope[k] for k in ('owner', 'active_run_id', 'fault_id')} != source['state']:
        raise RuntimeError("Arm-cycle startup does not bind the current preserved task fault")
    return {'cycle_id': row['cycle_id'], 'arm': side, 'finished_at': row['finished_at'],
            'boot': enrollment['boot'], 'record_path': str(path),
            'request_sha256': hashlib.sha256(request_raw).hexdigest(),
            'result_sha256': hashlib.sha256(result_raw).hexdigest(),
            'journal_sha256': hashlib.sha256(journal_raw).hexdigest(),
            'historical_projection_only': True, 'task_motion_authorized': False}


def confirmation(arm):
    names = {"left": "左臂", "right": "右臂", "both": "两臂"}
    if arm not in names:
        raise ValueError("必须明确选择 left、right 或 both")
    return ("我确认已对%s重新断电再上电；当前两臂无持物或接触负载、周围净空，"
            "并在现场看护；只启动所选臂，不恢复旧目标" % names[arm])


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False).encode()).hexdigest()


def _known_sends(receipt):
    """A terminal failure is not an unknown or partial send recovery grant."""
    value = receipt.get("device_receipt", receipt)
    counts = value.get("transmission_counts")
    if not isinstance(counts, dict) or set(counts) != {"left", "right"}:
        raise RuntimeError("旧失败缺少完整逐臂发送计数，不能通过断电声明重试。")
    for side in counts.values():
        if (not isinstance(side, dict)
                or any(type(side.get(k)) is not int or side[k] < 0
                       for k in ("attempted_frames", "sent_frames", "blocked_frames"))
                or side["attempted_frames"] != side["sent_frames"] or side["blocked_frames"]):
            raise RuntimeError("旧发送部分完成、未知或被阻挡，不能通过断电声明重试。")
    original = value.get("original_event")
    sent = sum(s["sent_frames"] for s in counts.values())
    if original is not None:
        frames = original.get("frame_receipts", [])
        if (original.get("send_state") != "all_frames_returned" or not frames
                or len(frames) != sent or any(f.get("outcome") != "returned" for f in frames)):
            raise RuntimeError("旧目标发送未完整返回，不能接续使能。")
    elif sent:
        # A completed mode-only prefix is still a partial startup. It cannot
        # be treated as a complete attempt merely because attempted == sent.
        kinds = value.get("transmission_counts_by_kind", {})
        if value.get("operation") not in ("startup_arm", "startup_arms"):
            raise RuntimeError("旧失败缺少完整发送事务证据，不能独立使能。")
        for side, count in counts.items():
            expected = 1 if count["sent_frames"] == 2 else 0
            if (count["sent_frames"] not in (0, 2) or kinds.get(side) != {
                    k: {"attempted_frames": expected, "sent_frames": expected} for k in ("mode", "enable")}):
                raise RuntimeError("旧启动只完成部分 mode/enable，不能独立使能。")


def _ledger(path):
    if not path.exists():
        return None
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only=ON")
        db.execute("BEGIN")
        tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if not {"pair_scope", "pair_events", "pair_faults", "pair_runs"} <= tables:
            raise RuntimeError("控制账本结构不完整。")
        latest = db.execute("SELECT run_id FROM pair_runs ORDER BY started_at DESC,run_id DESC LIMIT 1").fetchone()
        _, _, _, scope = _execution_scope(db, latest[0] if latest else None, writable=True)
        if scope is None:
            raise RuntimeError("缺少当前控制范围。")
        state = {k: scope[k] for k in ("owner", "active_run_id", "fault_id")}
        if state["fault_id"] is not None and not db.execute(
                "SELECT 1 FROM pair_faults WHERE id=?", (state["fault_id"],)).fetchone():
            raise RuntimeError("当前故障记录缺失。")
        if db.execute("SELECT 1 FROM pair_events WHERE status!='complete' OR finished_at IS NULL "
                      "OR success IS NULL OR success NOT IN (0,1) LIMIT 1").fetchone():
            raise RuntimeError("账本有未决发送，不能独立使能。")
        if "pair_holds" in tables and db.execute(
                "SELECT 1 FROM pair_holds WHERE status='pending' LIMIT 1").fetchone():
            raise RuntimeError("账本有未决保持事务。")
        for name in ("pair_hold_requests", "pair_hold_frames"):
            if name in tables and db.execute("SELECT 1 FROM " + name + " LIMIT 1").fetchone():
                raise RuntimeError("存在保持发送记录，需原保持交接审计，不能独立使能。")
        if "pair_grasp_episodes" in tables:
            from .grasp_episode import is_resolved_release
            for row in db.execute("SELECT state_json FROM pair_grasp_episodes"):
                grasp = json.loads(row[0])
                if grasp.get("status") != "empty" and not is_resolved_release(grasp):
                    raise RuntimeError("账本有未解除持物关系，不能独立使能。")
        # Historical terminal failures behind a previously audited successor
        # stay archived. Inspect failed events in the current ownership scope.
        for row in db.execute("SELECT payload_json,receipt_json,success FROM pair_events WHERE run_id=? AND owner=?",
                              (state["active_run_id"], state["owner"])):
            payload = json.loads(row[0])
            if payload.get("operation") == "grip_supported" and not payload.get("grasp_object_id"):
                raise RuntimeError("旧接触试夹未绑定物体释放记录，不能独立使能。")
            if row["success"] == 0:
                _known_sends(json.loads(row["receipt_json"]))
        if "pair_reboot_startups" in tables:
            for row in db.execute("SELECT * FROM pair_reboot_startups"):
                if row["status"] == "pending" or row["finished_at"] is None:
                    raise RuntimeError("旧使能事务仍未决。")
                if row["status"] != "complete":
                    _known_sends(json.loads(Path(row["record_path"]).read_text()))
        claims = []
        if TABLE in tables:
            claims = [dict(r) for r in db.execute("SELECT * FROM " + TABLE + " ORDER BY created_at,cycle_id")]
            if any(r["status"] != "complete" or r["finished_at"] is None for r in claims):
                raise RuntimeError("已有未决或失败的机械臂断电启动事务；禁止重复发送，请检查原回执。")
        hashes = {}
        for table in sorted(tables):
            if not table.startswith("pair_") or table == TABLE:
                continue
            # Identifiers come only from SQLite's schema, then are quoted.
            quoted = '"' + table.replace('"', '""') + '"'
            rows = [dict(r) for r in db.execute("SELECT * FROM " + quoted)]
            hashes[table] = _digest(sorted(rows, key=lambda r: json.dumps(r, sort_keys=True)))
    return {"database": str(path.resolve()), "state": state, "table_sha256": hashes,
            "prior_arm_power_cycles": claims}


def inspect(project):
    """Read-only eligibility; current hardware and operator evidence come later."""
    check_processes()
    from .startup_reset import inspect as inspect_reset
    reset_state = inspect_reset(project)
    if reset_state is not None and reset_state["status"] != "complete":
        raise RuntimeError("软件重置后的首次启动尚未完成，请使用原重置启动入口。")
    ledgers = [s for root in project_roots(project)
               if (s := _ledger(root / "runs/pair_sessions.sqlite")) is not None]
    if not ledgers:
        raise RuntimeError("无历史控制账本，请使用普通启动入口。")
    if reset_state is None and not any(
            s["state"]["owner"] is not None or s["state"]["fault_id"] is not None for s in ledgers):
        raise RuntimeError("当前账本已正常释放，请使用普通启动入口。")
    result = {"route": "arm_power_cycle_startup", "boot": boot_identity(), "ledgers": ledgers,
              "physical_stop_verified": None, "task_motion_authorized": False}
    if reset_state is not None:
        result["completed_reset_startup"] = reset_state
    return result


def has_startup_history(project):
    """Select the event-based route after a prior startup; never grants a send."""
    boot = boot_identity()
    for root in project_roots(project):
        path = root / "runs/pair_sessions.sqlite"
        if not path.exists():
            continue
        with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
            db.execute("PRAGMA query_only=ON")
            names = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            if TABLE in names and db.execute("SELECT 1 FROM " + TABLE + " LIMIT 1").fetchone():
                return True
            if "pair_reboot_startups" in names and db.execute(
                    "SELECT 1 FROM pair_reboot_startups WHERE boot_id=? LIMIT 1", (boot["boot_id"],)).fetchone():
                return True
    return False


def _reserve(path, cycle_id, arm, statement, enrollment, record_path):
    with sqlite3.connect(path) as db:
        db.execute("PRAGMA synchronous=FULL")
        db.execute("BEGIN IMMEDIATE")
        db.execute("CREATE TABLE IF NOT EXISTS " + TABLE + " ("
                   "cycle_id TEXT PRIMARY KEY, arm TEXT NOT NULL, statement TEXT NOT NULL, "
                   "enrollment_sha256 TEXT NOT NULL, status TEXT NOT NULL, record_path TEXT NOT NULL, "
                   "created_at REAL NOT NULL, finished_at REAL)")
        if db.execute("SELECT 1 FROM " + TABLE + " WHERE status!='complete' OR cycle_id=?", (cycle_id,)).fetchone():
            raise RuntimeError("断电事件已尝试或有未决/失败事务，禁止重发。")
        db.execute("INSERT INTO " + TABLE + " VALUES (?,?,?,?,?,?,?,NULL)",
                   (cycle_id, arm, statement, _digest(enrollment), "pending", str(record_path), time.time()))


def run(service, arm, power_cycle_id, power_cycle_and_clearance_statement):
    if (type(power_cycle_id) is not str or not re.fullmatch(r"[A-Za-z0-9_-]{1,120}", power_cycle_id)
            or power_cycle_and_clearance_statement != confirmation(arm)):
        raise ValueError("需要本次实际机械臂断电事件 ID 及对应臂的现场确认。")
    if service.pair_host is not None:
        raise RuntimeError("当前服务仍持有双臂宿主，不能独立使能。")
    from .service import _write
    from .single_arm_startup import startup_arm
    from .takeover import startup_arms
    with ExitStack() as locks:
        roots = set(project_roots(service.root)) | {service.root.parent / "piper_right_pick_demo",
                                                   Path("/home/agilex/piper_right_pick_demo")}
        for root in sorted(roots):
            if root == service.root or (root / "runs").is_dir():
                locks.enter_context(ExclusiveExecution(root / "runs"))
        enrollment = inspect(service.root)
        for ledger in enrollment["ledgers"]:
            if any(r["cycle_id"] == power_cycle_id for r in ledger["prior_arm_power_cycles"]):
                raise RuntimeError("该断电事件已经尝试；请查看原回执，不重发。")
        database = service.runs / "pair_sessions.sqlite"
        if not database.is_file():
            raise RuntimeError("当前项目缺少权威控制账本，不能更换数据库绕过旧状态。")
        run_id, directory = service._new_run("arm_power_cycle")
        result_path = directory / "result.json"
        _write(directory / "request.json", {"run_id": run_id, "arm": arm,
               "power_cycle_id": power_cycle_id, "operator_statement": power_cycle_and_clearance_statement,
               "enrollment": enrollment, "scope": "startup_only; no old task resume or target replay"})
        journal = Journal(directory)
        claimed = False

        def record(event, data):
            nonlocal claimed
            if event in ("mode_request_intent", "enable_request_intent"):
                check_processes()
                if boot_identity() != enrollment["boot"]:
                    raise RuntimeError("电脑启动身份变化。")
                if not claimed:
                    if inspect(service.root) != enrollment:
                        raise RuntimeError("使能前原控制账本变化。")
                    _reserve(database, power_cycle_id, arm, power_cycle_and_clearance_statement,
                             enrollment, result_path)
                    claimed = True
            journal.append(event, **data)

        # The unchanged executor collects its fresh bounded baseline,
        # verifies full standby/disabled state and guards every outgoing frame.
        # If persistence or the process fails after reservation, pending stays.
        result = (startup_arms(service.profile, record) if arm == "both" else
                  startup_arm(service.profile, record, arm))
        result.update(run_id=run_id, record_path=str(result_path), power_cycle_id=power_cycle_id,
                      arm_power_cycle=enrollment, old_task_faults_preserved=True,
                      task_motion_authorized=False, physical_stop_verified=None)
        _write(result_path, result)
        if claimed:
            with sqlite3.connect(database) as db:
                db.execute("PRAGMA synchronous=FULL")
                db.execute("UPDATE " + TABLE + " SET status=?,finished_at=? WHERE cycle_id=? AND status='pending'",
                           ("complete" if result.get("ok") is True else "failed", time.time(), power_cycle_id))
        return result
