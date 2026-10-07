"""Codex 跨命令调用的离线任务账本；不导入或启动模型、相机与执行器。

@author Codex
@date 2026-10-07
@version v1.0.0
@last_modified 2026-10-07
@changelog
  - v1.0.0 (2026-10-07): 持久化任务契约、事件、预算和观察分支，拒绝重复推进。
"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import time

from .fast_pipeline import PipelineContractError
from .fast_task_pipeline import BoundedTaskPipeline


def _json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _identifier(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,95}", value):
        raise PipelineContractError(name + " must be 1..96 letters, digits, dots, underscores or hyphens")
    return value


class TaskSessionStore:
    """事务只保护账本一致性，不能提供硬件互斥、动作准入或回执真实性。"""

    def __init__(self, path, *, clock=time.time):
        self.path = Path(path)
        self.clock = clock

    def _now(self):
        value = self.clock()
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise PipelineContractError("Session clock must be finite and nonnegative")
        return value

    @contextmanager
    def _transaction(self, create=False):
        if create:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        elif not self.path.is_file():
            raise PipelineContractError("Session store does not exist; initialize once")
        connection = sqlite3.connect(str(self.path), timeout=1, isolation_level=None)
        connection.row_factory = sqlite3.Row
        try:
            connection.execute("PRAGMA foreign_keys=ON")
            connection.execute("BEGIN IMMEDIATE")
            if create:
                connection.execute("""CREATE TABLE IF NOT EXISTS task_sessions (
                    run_id TEXT PRIMARY KEY, config TEXT NOT NULL, contract TEXT NOT NULL,
                    created_wall REAL NOT NULL, last_wall REAL NOT NULL, elapsed REAL NOT NULL,
                    clock_fault INTEGER NOT NULL DEFAULT 0)""")
                connection.execute("""CREATE TABLE IF NOT EXISTS task_events (
                    run_id TEXT NOT NULL REFERENCES task_sessions(run_id),
                    revision INTEGER NOT NULL, event_id TEXT NOT NULL, kind TEXT NOT NULL,
                    payload TEXT NOT NULL, elapsed REAL NOT NULL,
                    PRIMARY KEY(run_id, revision), UNIQUE(run_id, event_id))""")
            yield connection
            connection.commit()
        except BaseException:
            connection.rollback()
            raise
        finally:
            connection.close()

    def _load(self, connection, run_id):
        _identifier(run_id, "run_id")
        row = connection.execute("SELECT * FROM task_sessions WHERE run_id=?", (run_id,)).fetchone()
        if row is None:
            raise PipelineContractError("Unknown run_id; initialize once, do not silently replace a run")
        config = json.loads(row["config"])
        replay_clock = [0.0]
        ledger = BoundedTaskPipeline(**config, clock=lambda: replay_clock[0])
        if _json(ledger.contract) != row["contract"]:
            raise PipelineContractError("Task contract changed; existing run requires explicit migration")
        events = connection.execute("SELECT * FROM task_events WHERE run_id=? ORDER BY revision", (run_id,)).fetchall()
        for revision, event in enumerate(events, 1):
            if event["revision"] != revision or event["elapsed"] < replay_clock[0]:
                raise PipelineContractError("Session event sequence is inconsistent")
            replay_clock[0] = event["elapsed"]
            self._apply(ledger, event["kind"], json.loads(event["payload"]))
        now = self._now()
        # 跨进程使用真实累计时间；系统时间倒退时锁住账本，不赠送新的预算。
        elapsed = max(row["elapsed"], now - row["created_wall"])
        fault = bool(row["clock_fault"]) or now < row["last_wall"]
        replay_clock[0] = elapsed
        if fault and ledger.termination_reason is None:
            ledger.termination_reason = "session_clock_regressed"
        ledger.report()
        connection.execute("UPDATE task_sessions SET last_wall=?, elapsed=?, clock_fault=? WHERE run_id=?",
                           (max(now, row["last_wall"]), elapsed, int(fault), run_id))
        return ledger, len(events), elapsed, row["contract"]

    @staticmethod
    def _apply(ledger, kind, payload):
        if kind == "cycle":
            ledger.record_cycle(payload)
        elif kind == "observer":
            ledger.request_observer_view(payload)
        elif kind == "end":
            if ledger.current() is None:
                raise PipelineContractError("Pipeline is terminated")
            ledger.termination_reason = "offline_session_ended"
            ledger.termination_detail = payload["reason"]
        else:
            raise PipelineContractError("Unknown session event kind")

    @staticmethod
    def _report(ledger, run_id, revision, elapsed, contract, *, duplicate=False):
        report = ledger.report()
        report.update(run_id=run_id, revision=revision,
                      elapsed_s=round(elapsed, 3),
                      seconds_left=round(max(0, ledger.budget["max_elapsed_s"] - elapsed), 3),
                      contract_sha256=hashlib.sha256(contract.encode("utf-8")).hexdigest(),
                      duplicate_event=duplicate, scope="offline_host_receipt_ledger")
        if hasattr(ledger, "termination_detail"):
            report["termination_detail"] = ledger.termination_detail
        return report

    def initialize(self, run_id, task_id, *, mode="single_arm", worker_arm="right", budget=None):
        _identifier(run_id, "run_id")
        ledger = BoundedTaskPipeline(task_id, mode=mode, worker_arm=worker_arm, budget=budget)
        config = _json(dict(task_id=task_id, mode=mode, worker_arm=worker_arm, budget=ledger.budget))
        contract = _json(ledger.contract)
        with self._transaction(create=True) as connection:
            old = connection.execute("SELECT config, contract FROM task_sessions WHERE run_id=?", (run_id,)).fetchone()
            if old is not None:
                if old["config"] != config or old["contract"] != contract:
                    raise PipelineContractError("Existing run has a different frozen task, roles or budget")
            else:
                now = self._now()
                connection.execute("INSERT INTO task_sessions VALUES (?, ?, ?, ?, ?, 0, 0)",
                                   (run_id, config, contract, now, now))
            ledger, revision, elapsed, contract = self._load(connection, run_id)
            return self._report(ledger, run_id, revision, elapsed, contract)

    def current(self, run_id, *, include_contract=False, include_operation=False):
        with self._transaction() as connection:
            ledger, revision, elapsed, contract = self._load(connection, run_id)
            report = self._report(ledger, run_id, revision, elapsed, contract)
            if include_contract:
                report["contract"] = json.loads(contract)
            if include_operation:
                report["operation"] = ledger.current_operation()
            return report

    def append(self, run_id, *, expected_revision, event_id, kind, payload):
        _identifier(event_id, "event_id")
        if type(expected_revision) is not int or expected_revision < 0:
            raise PipelineContractError("expected_revision must be a nonnegative integer")
        if not isinstance(payload, dict):
            raise PipelineContractError("Event payload must be a JSON object")
        if kind not in ("cycle", "observer", "end"):
            raise PipelineContractError("Unknown session event kind")
        if kind != "end" and payload.get("run_id") != run_id:
            raise PipelineContractError("Host receipt must bind this exact run_id")
        if kind == "end" and (not isinstance(payload.get("reason"), str) or not payload["reason"].strip()
                              or len(payload["reason"]) > 240 or set(payload) != {"reason"}):
            raise PipelineContractError("Ending an offline session requires a reason of 1..240 characters")
        encoded = _json(payload)
        with self._transaction() as connection:
            ledger, revision, elapsed, contract = self._load(connection, run_id)
            old = connection.execute("SELECT kind, payload FROM task_events WHERE run_id=? AND event_id=?",
                                     (run_id, event_id)).fetchone()
            if old is not None:
                if old["kind"] != kind or old["payload"] != encoded:
                    raise PipelineContractError("event_id was already used for a different payload")
                return self._report(ledger, run_id, revision, elapsed, contract, duplicate=True)
            if revision != expected_revision:
                raise PipelineContractError("Stale revision; read current stage before proposing another event")
            # 到期报告仍提交时钟状态，避免下一次命令误以为本轮尚未开始。
            if ledger.current() is None:
                return dict(self._report(ledger, run_id, revision, elapsed, contract), event_applied=False)
            self._apply(ledger, kind, payload)
            connection.execute("INSERT INTO task_events VALUES (?, ?, ?, ?, ?, ?)",
                               (run_id, revision + 1, event_id, kind, encoded, elapsed))
            return dict(self._report(ledger, run_id, revision + 1, elapsed, contract), event_applied=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Persistent offline pipeline ledger; no model or device calls")
    parser.add_argument("--store", required=True, help="Local SQLite journal, reused for every command")
    commands = parser.add_subparsers(dest="command", required=True)
    initialize = commands.add_parser("init", help="Create a frozen run, or return its existing progress")
    initialize.add_argument("--task", required=True)
    initialize.add_argument("--mode", choices=("single_arm", "dual_arm", "worker_with_observer"), default="single_arm")
    initialize.add_argument("--worker-arm", choices=("left", "right"), default="right")
    initialize.add_argument("--budget", help="JSON file overriding task budget fields")
    current = commands.add_parser("current", help="Read compact stage and remaining total budget")
    current.add_argument("--operation", action="store_true", help="Expand only the current L2 operation")
    contract = commands.add_parser("contract", help="Read the frozen full task once")
    record = commands.add_parser("record", help="Record one host cycle receipt")
    observer = commands.add_parser("observer", help="Enter the view-only branch using a current hold receipt")
    end = commands.add_parser("end", help="End the offline ledger; this is not a hardware stop")
    for child in (initialize, current, contract, record, observer, end):
        child.add_argument("--run-id", required=True)
    for child in (record, observer, end):
        child.add_argument("--revision", type=int, required=True)
        child.add_argument("--event-id", required=True)
    for child in (record, observer):
        child.add_argument("--receipt", required=True, help="JSON receipt from host evidence; not a model claim")
    end.add_argument("--reason", required=True)
    args = parser.parse_args(argv)
    store = TaskSessionStore(args.store)
    try:
        if args.command == "init":
            budget = json.loads(Path(args.budget).read_text()) if args.budget else None
            result = store.initialize(args.run_id, args.task, mode=args.mode, worker_arm=args.worker_arm, budget=budget)
        elif args.command in ("current", "contract"):
            result = store.current(args.run_id, include_contract=args.command == "contract",
                                   include_operation=args.command == "current" and args.operation)
        else:
            payload = {"reason": args.reason} if args.command == "end" else json.loads(Path(args.receipt).read_text())
            result = store.append(args.run_id, expected_revision=args.revision, event_id=args.event_id,
                                  kind={"record": "cycle", "observer": "observer", "end": "end"}[args.command],
                                  payload=payload)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if result.get("event_applied") is False:
            return 2
        return 0 if result["termination_reason"] in (None, "offline_contract_completed", "offline_session_ended") else 2
    except (PipelineContractError, ValueError, TypeError, OSError, sqlite3.Error) as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc), "execution_available": False}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
