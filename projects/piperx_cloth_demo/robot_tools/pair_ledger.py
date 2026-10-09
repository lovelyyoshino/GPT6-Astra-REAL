"""Durable single-owner dispatch ledger; contains no robot or camera imports.

A successful begin reserves one attempt, not proof of transmission. Its caller
may dispatch only after begin returns, and must never replay a pending event.
Use one database for the same physical pair: its fault latch spans all run IDs.
Detaching an owner is bookkeeping and does not certify physical stopping.
"""
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import stat
import struct
import threading
import time
import uuid


class PairLedgerError(RuntimeError):
    """The requested ledger transition was refused."""


class PairLedgerFault(PairLedgerError):
    """A durable fault prohibits further dispatch from this database."""


def _hold_frames(raw):
    """Pure expected wire bytes; never a sender or permission token."""
    if (type(raw) is not list or len(raw) != 6 or
            any(type(value) is not int or not -(2**31) <= value < 2**31 for value in raw)):
        raise ValueError("Six signed millidegree joint integers required")
    flags = {"is_extended_id": False, "is_remote_frame": False, "is_error_frame": False, "is_fd": False}
    return [{"arbitration_id": 0x151, "data_hex": "0101010000000000", **flags}] + [
        {"arbitration_id": 0x155+i, "data_hex": struct.pack(">ii", *raw[2*i:2*i+2]).hex(), **flags}
        for i in range(3)]


def _hold_identity(identity):
    fields = {"run_id", "owner", "epoch", "worker_id", "arm", "connection_id", "model", "firmware_profile"}
    if type(identity) is not dict or set(identity) != fields:
        raise ValueError("Resolved hold identity required")
    for name, value in identity.items():
        _identifier(value, name)
    if identity["arm"] not in ("left", "right"):
        raise ValueError("Hold must select one explicit arm")


def _hold_hash(value):
    if type(value) is not str or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("Lowercase SHA-256 source reference required")


def _hold_frame_equal(frame, expected):
    return (type(frame) is dict and set(frame) == set(expected)
            and type(frame.get("arbitration_id")) is int and type(frame.get("data_hex")) is str
            and all(frame.get(name) is False for name in
                    ("is_extended_id", "is_remote_frame", "is_error_frame", "is_fd")) and frame == expected)


def _identifier(value, name):
    if (type(value) is not str or not 1 <= len(value) <= 128
            or value != value.strip() or any(ord(char) < 32 or ord(char) == 127 for char in value)):
        raise ValueError(name + " must be a nonempty identifier of at most 128 characters")
    return value


def _number(value, name, *, positive=False):
    try:
        valid = (type(value) in (int, float) and math.isfinite(value)
                 and value >= 0 and not (positive and value == 0))
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(name + " must be a finite " + ("positive" if positive else "nonnegative") + " number")
    return float(value)


def _json_object(value, name):
    if type(value) is not dict:
        raise ValueError(name + " must be a JSON object")
    def validate(item, depth=0):
        if depth > 64:
            raise ValueError(name + " exceeds JSON nesting limit")
        kind = type(item)
        if item is None or kind in (str, bool):
            return
        if kind is int:
            if not -(2**63) <= item < 2**63:
                raise ValueError(name + " integer is outside signed 64-bit range")
            return
        if kind is float:
            if not math.isfinite(item):
                raise ValueError(name + " contains a nonfinite number")
            return
        if kind is list:
            for child in item:
                validate(child, depth + 1)
            return
        if kind is dict and all(type(key) is str for key in item):
            for child in item.values():
                validate(child, depth + 1)
            return
        raise ValueError(name + " contains a non-JSON value")
    validate(value)
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    if len(encoded.encode("utf-8")) > 1_048_576:
        raise ValueError(name + " exceeds one MiB")
    return encoded


def _execution_scope(db, run_id=None, *, writable=False):
    """Resolve an explicitly enrolled successor; legacy callers see old fault.

    Enrollment is an offline administrative transition, never a normal claim.
    In particular, an arbitrary new run ID cannot select a fresh fault scope.
    """
    observed = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_contact_observations'").fetchone()
    if observed:
        head = db.execute('SELECT * FROM pair_supported_contact_observations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_supported_contact_observations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited existing-contact observation')
    contact = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_contact_reacquisitions'").fetchone()
    if contact:
        head = db.execute('SELECT * FROM pair_supported_contact_reacquisitions ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_supported_contact_reacquisitions', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited supported contact reacquisition')
    opening = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_gripper_opening_continuations'").fetchone()
    if opening:
        head = db.execute('SELECT * FROM pair_supported_gripper_opening_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_supported_gripper_opening_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited further-opening continuation')
    recovery = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_gripper_recoveries'").fetchone()
    if recovery:
        head = db.execute('SELECT * FROM pair_supported_gripper_recoveries ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_supported_gripper_recoveries', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited supported jaw recovery')
    manual = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_manual_gripper_continuations'").fetchone()
    if manual:
        head = db.execute('SELECT * FROM pair_manual_gripper_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_manual_gripper_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the explicitly authorized manual gripper continuation')
    rgb = db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_round_rgb_continuations'").fetchone()
    if rgb:
        head = db.execute('SELECT * FROM pair_round_rgb_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_round_rgb_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the same-budget RGB continuation')
    endpoint = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_endpoint_continuations'").fetchone()
    if endpoint:
        head = db.execute('SELECT * FROM pair_endpoint_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_endpoint_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited endpoint continuation')
    initialization = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_initialization_continuations'").fetchone()
    if initialization:
        head = db.execute('SELECT * FROM pair_initialization_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_initialization_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited initialization continuation')
    feedback = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_feedback_continuations'").fetchone()
    if feedback:
        head = db.execute('SELECT * FROM pair_feedback_continuations ORDER BY ordinal DESC LIMIT 1').fetchone()
        parent = db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_feedback_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited feedback continuation')
    preparation = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_preparation_continuations'").fetchone()
    if preparation:
        head = db.execute("SELECT * FROM pair_preparation_continuations ORDER BY ordinal DESC LIMIT 1").fetchone()
        parent = db.execute("SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1").fetchone()
        if head is not None and parent is not None and head['round_ordinal'] == parent['ordinal']:
            if run_id == head['run_id']:
                return 'pair_preparation_continuations', 'ordinal', head['ordinal'], head
            if writable:
                raise PairLedgerFault('Run is outside the audited preparation continuation')
    rounds = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_rounds'").fetchone()
    if rounds:
        head = db.execute("SELECT * FROM pair_rounds ORDER BY ordinal DESC LIMIT 1").fetchone()
        if head is not None:
            if run_id == head["run_id"]:
                return "pair_rounds", "ordinal", head["ordinal"], head
            if writable:
                raise PairLedgerFault("Run is outside the explicitly authorized current round")
    predispatch = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_predispatch_continuations'").fetchone()
    if predispatch:
        head = db.execute("SELECT * FROM pair_predispatch_continuations ORDER BY ordinal DESC LIMIT 1").fetchone()
        if head is not None:
            if run_id == head["run_id"]:
                return "pair_predispatch_continuations", "ordinal", head["ordinal"], head
            if writable:
                raise PairLedgerFault("Run is outside the audited predispatch continuation scope")
    continuation = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_continuations'").fetchone()
    if continuation:
        head = db.execute("SELECT * FROM pair_continuations ORDER BY ordinal DESC LIMIT 1").fetchone()
        if head is not None:
            if run_id == head["run_id"]:
                return "pair_continuations", "ordinal", head["ordinal"], head
            if writable:
                raise PairLedgerFault("Run is outside the single same-budget continuation scope")
    exists = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_execution_epochs'").fetchone()
    if exists:
        head = db.execute("SELECT * FROM pair_execution_epochs ORDER BY ordinal DESC LIMIT 1").fetchone()
        if head is not None:
            if run_id == head["run_id"]:
                return "pair_execution_epochs", "ordinal", head["ordinal"], head
            if writable:
                raise PairLedgerFault("Run is outside the explicitly authorized active execution epoch")
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reset_task_budgets'").fetchone():
        initial = db.execute("SELECT run_id FROM pair_reset_task_budgets LIMIT 1").fetchone()
        if initial is not None and writable and run_id != initial["run_id"]:
            raise PairLedgerFault("Run is outside the explicitly enrolled post-reset task")
    row = db.execute("SELECT * FROM pair_scope WHERE id=1").fetchone()
    return "pair_scope", "id", 1, row


def _retired_owner(scope, owner):
    if "previous_owner" in scope.keys() and owner == scope["previous_owner"]:
        return True
    return ("retired_owners_json" in scope.keys()
            and owner in json.loads(scope["retired_owners_json"]))


def activated_execution_budget(path, run_id, *, max_steps, max_duration_s):
    """Read-only recognition of one explicitly activated expanded host budget.

    This never creates a run, grants ownership, changes a clock, or authenticates
    a human. The administrative activation record is the trusted audit source;
    the normal constructor still requires its exact frozen code/task contract.
    """
    source = Path(path).resolve()
    if not source.exists():
        return False
    db = sqlite3.connect(source.as_uri()+"?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    try:
        db.execute("PRAGMA query_only=ON")
        db.execute("BEGIN")
        return _activated_execution_budget_db(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
    finally:
        db.close()


def _activated_execution_budget_db(db, run_id, *, max_steps, max_duration_s):
    """Same immutable budget audit on an existing read-only history snapshot."""
    try:
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reset_task_budgets'").fetchone():
            table, _, _, _ = _execution_scope(db, run_id)
            if table == "pair_scope":
                from .startup_reset import audit_task_budget
                return audit_task_budget(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_contact_observations'").fetchone():
            observed=db.execute('SELECT * FROM pair_supported_contact_observations ORDER BY ordinal DESC LIMIT 1').fetchone()
            current=db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
            if observed is not None and current is not None and observed['round_ordinal']==current['ordinal']:
                from .supported_gripper_recovery import audit_existing_contact_budget
                return audit_existing_contact_budget(db,run_id,max_steps=max_steps,max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_contact_reacquisitions'").fetchone():
            contact=db.execute('SELECT * FROM pair_supported_contact_reacquisitions ORDER BY ordinal DESC LIMIT 1').fetchone()
            current_round=db.execute('SELECT ordinal FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
            if contact is not None and current_round is not None and contact['run_id']==run_id and contact['round_ordinal']==current_round['ordinal']:
                from .supported_gripper_recovery import audit_contact_budget
                return audit_contact_budget(db,run_id,max_steps=max_steps,max_duration_s=max_duration_s)
        # Older contact tables stay intact when a restricted successor round
        # is appended. Select the matching current round before those parents.
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_rounds'").fetchone():
            current=db.execute('SELECT * FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
            if current is not None and current['run_id']==run_id:
                record=json.loads(current['record_json'])
                if record['proposal'].get('parent_kind')=='supported_contact_zero_tx_fault':
                    from .pair_round import _audit_round_enrollment
                    run=db.execute('SELECT * FROM pair_runs WHERE run_id=?',(run_id,)).fetchone()
                    if run is None or run['max_steps']!=max_steps or run['max_duration']!=max_duration_s:return False
                    if (record['proposal'].get('budget_policy')!='explicit_user_new_round'
                            or json.loads(run['contract_json'])!=record['new_contract']):
                        raise PairLedgerError('Restricted contact round must retain its explicit budget and frozen run contract')
                    _audit_round_enrollment(db,current,run,parent_kind='supported_contact_zero_tx_fault')
                    return True
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_contact_reacquisitions'").fetchone():
            from .supported_gripper_recovery import audit_contact_budget
            return audit_contact_budget(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_gripper_opening_continuations'").fetchone():
            from .supported_gripper_recovery import audit_opening_budget
            return audit_opening_budget(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_supported_gripper_recoveries'").fetchone():
            from .supported_gripper_recovery import audit_budget
            return audit_budget(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_manual_gripper_continuations'").fetchone():
            from .manual_gripper_continuation import audit_budget
            return audit_budget(db, run_id, max_steps=max_steps, max_duration_s=max_duration_s)
        if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_rounds'").fetchone():
            scope = db.execute("SELECT * FROM pair_rounds ORDER BY ordinal DESC LIMIT 1").fetchone()
            if scope is not None:
                if scope["run_id"] != run_id:
                    return False
                record = json.loads(scope["record_json"])
                proposal, authorization = record["proposal"], record["authorization"]
                digest = lambda value: hashlib.sha256(_json_object(value, "round evidence").encode()).hexdigest()
                budget = proposal["new_budget"]
                start_policy = proposal.get("budget_start_policy", "include_repair_time")
                run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
                if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_round_rgb_continuations'").fetchone():
                    continuation = db.execute('SELECT * FROM pair_round_rgb_continuations WHERE run_id=? AND round_ordinal=?',
                                              (run_id,scope['ordinal'])).fetchone()
                    if continuation is not None:
                        from .round_rgb_continuation import audit_row
                        audit_row(db,continuation,run)
                parent = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (scope["parent_run_id"],)).fetchone()
                return (proposal.get("budget_policy") == "explicit_user_new_round"
                    and proposal["new_run_id"] == run_id and proposal["parent_run_id"] == scope["parent_run_id"]
                    and digest({k:v for k,v in proposal.items() if k != "proposal_sha256"})
                        == proposal["proposal_sha256"] == scope["proposal_sha256"]
                    and digest(authorization) == scope["authorization_sha256"]
                    and authorization["source"] == "user_message"
                    and authorization["decision"] == ('authorize_repaired_continuation'
                        if start_policy == 'preserve_parent_deadline' else 'authorize_explicit_new_round')
                    and authorization["proposal_sha256"] == scope["proposal_sha256"]
                    and _json_object(authorization["new_budget"], "budget") == _json_object(budget, "budget")
                    and start_policy in ("include_repair_time", "after_repair_before_online_execution", 'preserve_parent_deadline')
                    and proposal["authorization_not_before"] <= authorization["received_at"] <= budget["started_at"]
                    and ((start_policy == "include_repair_time" and authorization["received_at"] == budget["started_at"])
                         or (start_policy in ("after_repair_before_online_execution", 'preserve_parent_deadline')
                             and authorization.get("budget_start_policy") == start_policy))
                    and budget["started_at"] <= proposal["created_at"] <= record["activated_at"] < proposal["deadline_s"]
                    and type(budget["max_steps"]) is int and 1 <= budget["max_steps"] <= 1000
                    and type(budget["max_duration_s"]) in (int,float) and 0 < budget["max_duration_s"] <= 10800
                    and budget["max_steps"] == max_steps and budget["max_duration_s"] == max_duration_s
                    and run is not None and parent is not None
                    and run["started_at"] == budget["started_at"]
                    and proposal["deadline_s"] == budget["started_at"] + budget["max_duration_s"]
                    and run["max_steps"] == max_steps and run["max_duration"] == max_duration_s
                    and json.loads(run["contract_json"]) == record["new_contract"] == proposal["reviewed_contract"]
                    and proposal["parent_run"] == dict(parent)
                    and (proposal.get('parent_kind', 'clean') == 'clean' or
                         (proposal.get('parent_kind') == 'query_duplicate_fault'
                          and start_policy == 'after_repair_before_online_execution'
                          and max_steps == parent['max_steps']-parent['steps']
                          and max_duration_s == parent['max_duration']) or
                         (proposal.get('parent_kind') == 'zero_tx_freshness_fault'
                          and start_policy == 'preserve_parent_deadline'
                          and max_steps == parent['max_steps']-parent['steps']
                          and proposal['deadline_s'] == parent['started_at']+parent['max_duration']
                          and record.get('new_budget_allocated') is False) or
                         (proposal.get('parent_kind') == 'postsend_rgb_expiry_fault'
                          and start_policy == 'after_repair_before_online_execution'
                          and proposal['authorization_not_before'] >= parent['started_at']+parent['max_duration']
                          and record.get('new_budget_allocated') is True) or
                         (proposal.get('parent_kind') == 'initial_rx_zero_tx_fault'
                          and start_policy == 'after_repair_before_online_execution'
                          and record.get('new_budget_allocated') is True
                          and _audited_initial_rx_budget(db, run, scope)) or
                         (proposal.get('parent_kind') == 'completed_unloaded_joint_fault'
                          and start_policy == 'after_repair_before_online_execution'
                          and proposal['authorization_not_before'] >= parent['started_at']+parent['max_duration']
                          and record.get('new_budget_allocated') is True
                          and record.get('required_connection_mode') == proposal.get('required_connection_mode') == 'prepare'
                          and _audited_completed_unloaded_budget(db, run, scope)) or
                         (proposal.get('parent_kind') == 'configuration_maintenance_fault'
                          and start_policy == 'after_repair_before_online_execution'
                          and record.get('new_budget_allocated') is True
                          and record.get('required_connection_mode') == proposal.get('required_connection_mode') == 'prepare'
                          and _audited_configuration_budget(db, run, scope)) or
                         (proposal.get('parent_kind') == 'postreboot_supervised_plug_task'
                          and proposal.get('schema') == 'piper_postreboot_supervised_task_v1'
                          and start_policy == 'after_repair_before_online_execution'
                          and proposal['authorization_not_before'] >= parent['started_at']+parent['max_duration']
                          and record.get('new_budget_allocated') is True
                          and record.get('required_connection_mode') == proposal.get('required_connection_mode') == 'prepare'
                          and proposal.get('startup',{}).get('known_frames_returned') == 4
                          and proposal.get('startup',{}).get('failed_result_preserved') is True
                          and proposal.get('task',{}).get('envelope',{}).get('task') == record['new_contract']['task']))
                    and proposal["cumulative_step_ceiling"] == proposal["snapshot"]["cumulative_prior_steps"]+max_steps)
        if not db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_execution_epochs'").fetchone():
            return False
        scope = db.execute("SELECT * FROM pair_execution_epochs ORDER BY ordinal DESC LIMIT 1").fetchone()
        if scope is None or scope["run_id"] != run_id:
            return False
        record = json.loads(scope["record_json"])
        proposal, authorization = record["proposal"], record["authorization"]
        digest = lambda value: hashlib.sha256(_json_object(value, "activation evidence").encode()).hexdigest()
        budget = proposal["new_budget"]
        run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
        parent = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (scope["parent_run_id"],)).fetchone()
        return (proposal.get("budget_policy") == "explicit_user_budget_request"
            and proposal["new_run_id"] == run_id and proposal["parent_run_id"] == scope["parent_run_id"]
            and digest({key:value for key,value in proposal.items() if key != "proposal_sha256"})
                == proposal["proposal_sha256"] == scope["proposal_sha256"]
            and digest(authorization) == scope["authorization_sha256"]
            and authorization["source"] == "user_message"
            and authorization["decision"] == "authorize_explicit_budget_request"
            and authorization["proposal_sha256"] == scope["proposal_sha256"]
            and authorization["new_budget"] == budget
            and proposal["authorization_not_before"] < authorization["received_at"] <= record["activated_at"]
            and type(budget["max_steps"]) is int and 1 <= budget["max_steps"] <= 1000
            and type(budget["max_duration_s"]) in (int, float) and 0 < budget["max_duration_s"] <= 10800
            and budget == {"max_steps":max_steps, "max_duration_s":max_duration_s}
            and run is not None and parent is not None
            and run["max_steps"] == max_steps and run["max_duration"] == max_duration_s
            and json.loads(run["contract_json"]) == record["new_contract"] == proposal["reviewed_contract"]
            and proposal["parent_run"] == dict(parent)
            and proposal["cumulative_step_ceiling"] == parent["steps"]+max_steps)
    except (KeyError, TypeError, ValueError) as exc:
        raise PairLedgerError("Malformed activated execution budget") from exc


UNOPENED_REPAIR_SOURCES = frozenset({'pair_round.py', 'pair_ledger.py', 'pair_host.py'})


def effective_contract_json(db, run, scope):
    """Resolve a frozen contract, including one append-only unopened repair.

    The original run and enrollment rows remain the source of budget and task
    identity. A revision changes code only; it cannot clear a fault or supply
    targets, feedback, readiness, or a new execution scope.
    """
    if 'contract_json' in scope.keys():
        return scope['contract_json']
    original = run['contract_json']
    if not db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_unopened_round_revisions'").fetchone():
        return original
    row = db.execute('SELECT * FROM pair_unopened_round_revisions WHERE run_id=?', (run['run_id'],)).fetchone()
    if row is None:
        return original
    try:
        revision = json.loads(row['record_json'])
        registration = json.loads(scope['record_json'])
        proposal = registration['proposal']
        old, new = json.loads(original), revision['new_contract']
        digest = lambda value: hashlib.sha256(_json_object(value, 'contract revision').encode()).hexdigest()
        before = revision['old_run']
        expected_audit = {'original_snapshot_sha256': proposal['snapshot_sha256'],
            'original_proposal_sha256': proposal['proposal_sha256'],
            'historical_observations_only': True, 'fresh_host_admission_required': True}
        valid = (revision['schema'] == 'piper_unopened_round_code_repair_v2'
            and proposal.get('parent_kind') == 'initial_rx_zero_tx_fault'
            and old == registration['new_contract'] == proposal['reviewed_contract'] == revision['old_contract']
            and revision['original_round_record_sha256'] == digest(registration)
            and before['contract_json'] == original and before['run_id'] == run['run_id']
            and type(before['steps']) is int and before['steps'] == 0
            and all(before[k] == run[k] for k in ('started_at', 'max_steps', 'max_duration'))
            and revision['owner_at_revision'] is None and revision['events_at_revision'] == []
            and type(revision['hardware_commands_sent']) is int and revision['hardware_commands_sent'] == 0
            and revision['new_budget_allocated'] is False
            and revision['required_connection_mode'] == 'prepare'
            and revision['historical_audit'] == expected_audit
            and registration['activated_at'] <= row['at'] == revision['revised_at'] < proposal['deadline_s']
            and revision['deadline_s'] == run['started_at'] + run['max_duration'] == proposal['deadline_s']
            and {k:v for k,v in old.items() if k != 'code'} == {k:v for k,v in new.items() if k != 'code'}
            and set(old['code']) == set(new['code'])
            and {k for k in old['code'] if old['code'][k] != new['code'][k]} == UNOPENED_REPAIR_SOURCES)
        if not valid:
            raise PairLedgerError('Invalid unopened-round contract revision')
        return _json_object(new, 'effective contract')
    except (KeyError, TypeError, ValueError) as exc:
        raise PairLedgerError('Malformed unopened-round contract revision') from exc


def _audited_initial_rx_budget(db, run, scope):
    # Import only on administrative recognition, never in the feedback loop.
    # Checking a tag or a self-reported zero counter alone is insufficient.
    from .pair_round import audit_unopened_enrollment
    audit_unopened_enrollment(db, scope, run)
    effective_contract_json(db, run, scope)
    return True


def _audited_completed_unloaded_budget(db, run, scope):
    from .pair_round import audit_completed_unloaded_round
    audit_completed_unloaded_round(db, scope, run)
    return True


def _audited_configuration_budget(db, run, scope):
    from .pair_round import _audit_round_enrollment
    _audit_round_enrollment(db, scope, run, parent_kind='configuration_maintenance_fault')
    return True


def initial_rx_round_requires_preparation(path, run_id):
    """Read-only connection-mode routing; grants no recovery or motion."""
    source = Path(path).resolve()
    if not source.exists():
        return False
    with sqlite3.connect(source.as_uri() + '?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        if not db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_rounds'").fetchone():
            return False
        row = db.execute('SELECT * FROM pair_rounds ORDER BY ordinal DESC LIMIT 1').fetchone()
        return (row is not None and row['run_id'] == run_id
                and json.loads(row['record_json'])['proposal'].get('parent_kind') == 'initial_rx_zero_tx_fault')


def platform_state(path, *, run_id=None):
    """Read shared ownership, pending attempts and fault without changing time.

    A malformed/unreadable existing database raises: callers must fail closed.
    This read does not provide exclusion against noncooperating senders.
    """
    if not isinstance(path, (str, os.PathLike)) or not str(path) or str(path) == ":memory:":
        raise ValueError("path must name a persistent SQLite file")
    source = Path(path).resolve()
    try:
        source.stat()
    except FileNotFoundError:
        return None
    db = sqlite3.connect(source.as_uri() + "?mode=ro", uri=True, timeout=5.0)
    try:
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA query_only=ON")
        db.execute("BEGIN")  # One consistent, read-only snapshot of all state.
        _, _, _, scope = _execution_scope(db, run_id)
        if scope is None:
            raise PairLedgerError("Existing pair database has no platform state")
        fault = None
        if scope["fault_id"] is not None:
            fault = db.execute("SELECT * FROM pair_faults WHERE id=?", (scope["fault_id"],)).fetchone()
            if fault is None:
                raise PairLedgerError("Existing pair database has a missing fault record")
        pending = db.execute("SELECT run_id,event_id,owner,step,began_at FROM pair_events "
                             "WHERE status='pending' ORDER BY run_id,step").fetchall()
        return {"owner": scope["owner"], "active_run_id": scope["active_run_id"],
                "pending_events": [dict(row) for row in pending],
                "fault": dict(fault) if fault is not None else None}
    finally:
        db.close()


def platform_fault(path, *, run_id=None):
    """Read only the shared fault; clean owner/pending state is not a fault."""
    state = platform_state(path, run_id=run_id)
    return state["fault"] if state is not None else None


def revise_query_only_code_contract(path, run_id, *, expected_contract_sha256,
                                    project_root, reason, clock=time.time):
    """Offline code-only repair of a detached run with query-only history.

    No device imports/connections, budget renewal, physical qualification or
    source-epoch transfer. The caller must have closed its control host. The
    expected digest is SHA256 of the exact stored contract_json UTF-8 bytes.
    project_root names piperx_cloth_demo; its original robot_tools filename set
    supplies every new hash. Ordinary construction still cannot alter contracts.
    """
    from .execution import ExclusiveExecution
    from .joint_sources import _validate_bindings, _validated_limits

    def need(condition, message):
        if not condition:
            raise PairLedgerError(message)

    run_id = _identifier(run_id, "run_id")
    _hold_hash(expected_contract_sha256)
    need(type(reason) is str and 1 <= len(reason.strip()) <= 2000
         and "\x00" not in reason, "Explicit offline repair reason required")
    database = Path(path).absolute()
    root = Path(project_root).absolute()
    need(database.is_file() and database == database.resolve(), "Existing nonsymlink database required")
    need(root == root.resolve() and (root / "robot_tools").is_dir(), "Actual project root required")

    def read_code(names):
        hashes = {}
        directory = os.open(root / "robot_tools", os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            for name in sorted(names):
                need(type(name) is str and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*\.py", name),
                     "Code source must be a local Python basename")
                fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
                try:
                    info = os.fstat(fd)
                    need(stat.S_ISREG(info.st_mode) and info.st_size <= 4*1024*1024,
                         "Bounded regular code source required")
                    chunks, size = [], 0
                    while True:
                        chunk = os.read(fd, 65536)
                        if not chunk:
                            break
                        size += len(chunk)
                        need(size <= 4*1024*1024, "Code source grew during read")
                        chunks.append(chunk)
                    hashes[name] = hashlib.sha256(b"".join(chunks)).hexdigest()
                finally:
                    os.close(fd)
        finally:
            os.close(directory)
        return hashes

    with ExclusiveExecution(database.parent):
        db = sqlite3.connect(database.as_uri()+"?mode=rw", uri=True, timeout=5, isolation_level=None)
        try:
            db.row_factory = sqlite3.Row
            db.execute("PRAGMA synchronous=FULL")
            db.execute("BEGIN IMMEDIATE")
            need(not db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_execution_epochs'").fetchone(),
                 "Query-only revision cannot alter an administratively enrolled execution epoch")
            run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (run_id,)).fetchone()
            scope = db.execute("SELECT * FROM pair_scope WHERE id=1").fetchone()
            need(run is not None and scope is not None, "Existing run and platform scope required")
            now = _number(clock(), "clock")
            deadline = run["started_at"] + run["max_duration"]
            need(scope["last_time"] <= now < deadline, "Original clock/deadline blocks code revision")
            need(scope["owner"] is None and scope["active_run_id"] is None and scope["fault_id"] is None,
                 "Clean detached nonfault platform required")
            need(db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone() is None,
                 "Pending attempts block code revision")
            for table in ("pair_joint_sends", "pair_hold_requests", "pair_holds", "pair_hold_frames", "pair_grasp_episodes"):
                if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (table,)).fetchone():
                    need(db.execute("SELECT 1 FROM "+table+" LIMIT 1").fetchone() is None,
                         "Joint/hold/grasp history blocks query-only repair: "+table)
            old_json = run["contract_json"]
            old_digest = hashlib.sha256(old_json.encode()).hexdigest()
            need(old_digest == expected_contract_sha256, "Expected contract digest changed")
            old = json.loads(old_json)
            need(type(old) is dict and set(old) == {"task", "arms", "cameras", "sdk_commit_audited", "code"}
                 and type(old["code"]) is dict and bool(old["code"]), "Frozen host contract required")
            for value in old["code"].values():
                _hold_hash(value)
            events = db.execute("SELECT * FROM pair_events WHERE run_id=? ORDER BY step", (run_id,)).fetchall()
            need(bool(events) and run["steps"] == len(events)
                 and [row["step"] for row in events] == list(range(1, len(events)+1)),
                 "Complete contiguous query-only budget history required")
            owner_counts = {}
            for row in events:
                need(row["status"] == "complete" and row["success"] == 1,
                     "Only successful completed queries may precede this repair")
                payload, receipt = json.loads(row["payload_json"]), json.loads(row["receipt_json"])
                need(hashlib.sha256(row["payload_json"].encode()).hexdigest() == row["payload_digest"],
                     "Historical payload digest mismatch")
                need(set(payload) == {"kind", "request", "bindings"} and payload["kind"] == "query"
                     and payload["request"] == {"operation": "inspect_joint_limits"},
                     "Any non-query event blocks this repair")
                _validate_bindings(old, payload["bindings"])
                need(receipt.get("ok") is True and receipt.get("fault_latched") is False
                     and receipt.get("event_id") == row["event_id"] and receipt.get("pair_owner") == row["owner"]
                     and receipt.get("execution_mode") == "inspect_joint_limits", "Historical query binding failed")
                for key, expected in (("hardware_commands_sent", 12), ("joint_limit_queries_sent", 12),
                        ("joint_limit_queries_attempted", 12), ("actuator_commands_sent", 0),
                        ("target_commands_sent", 0), ("mode_commands_sent", 0),
                        ("enable_commands_sent", 0), ("stop_commands_sent", 0)):
                    need(type(receipt.get(key)) is int and receipt[key] == expected,
                         "Non-query/unknown transmission count: "+key)
                owner_counts[row["owner"]] = owner_counts.get(row["owner"], 0) + 1
                for key, count in (("transmission_counts", 6),
                                   ("session_transmission_counts", 6*owner_counts[row["owner"]])):
                    counters = receipt.get(key)
                    need(type(counters) is dict and set(counters) == {"left", "right"}, "Pair counters required")
                    for values in counters.values():
                        need(type(values) is dict and set(values) == {"attempted_frames", "sent_frames", "blocked_frames"}
                             and all(type(v) is int for v in values.values())
                             and values == {"attempted_frames": count, "sent_frames": count, "blocked_frames": 0},
                             "Unknown or extra lifetime sends block code revision")
                capture = {**receipt, "run_id": run_id, "owner": row["owner"], "bindings": payload["bindings"]}
                _validated_limits(capture, run_id=run_id, owner=row["owner"], bindings=payload["bindings"], now=now)
                need(run["started_at"] <= row["began_at"] <= receipt["began_at"] <= receipt["ended_at"]
                     <= row["finished_at"] <= now, "Historical query time mismatch")
            code = read_code(old["code"])
            need(code != old["code"], "No code revision found")
            new = {**old, "code": code}
            new_json = _json_object(new, "new contract")
            new_digest = hashlib.sha256(new_json.encode()).hexdigest()
            history = _json_object({"events": [dict(row) for row in events]}, "unchanged events")
            budget = {key: run[key] for key in ("started_at", "max_duration", "max_steps", "steps")}
            finished = _number(clock(), "clock")
            need(now <= finished < deadline and read_code(old["code"]) == code,
                 "Clock, deadline or source changed during repair")
            # Source rereading is IO too: recheck time after it, before commit.
            final_time = _number(clock(), "clock")
            need(finished <= final_time < deadline, "Original deadline expired during code verification")
            db.execute("CREATE TABLE IF NOT EXISTS pair_code_revisions ("
                       "revision_id TEXT PRIMARY KEY,run_id TEXT NOT NULL,at REAL NOT NULL,reason TEXT NOT NULL,"
                       "project_root TEXT NOT NULL,old_contract_json TEXT NOT NULL,new_contract_json TEXT NOT NULL,"
                       "old_digest TEXT NOT NULL,new_digest TEXT NOT NULL,source_hashes_json TEXT NOT NULL,"
                       "budget_json TEXT NOT NULL,event_history_sha256 TEXT NOT NULL)")
            revision = "code_repair_"+uuid.uuid4().hex
            db.execute("INSERT INTO pair_code_revisions VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                       (revision, run_id, final_time, reason, str(root), old_json, new_json, old_digest, new_digest,
                        _json_object(code, "code hashes"), _json_object(budget, "frozen budget"),
                        hashlib.sha256(history.encode()).hexdigest()))
            updated = db.execute("UPDATE pair_runs SET contract_json=? WHERE run_id=? AND contract_json=?",
                                 (new_json, run_id, old_json))
            need(updated.rowcount == 1, "Contract compare-and-swap failed")
            committed_at = _number(clock(), "clock")
            need(final_time <= committed_at < deadline, "Original deadline expired during revision write")
            db.execute("UPDATE pair_scope SET last_time=? WHERE id=1", (committed_at,))
            db.execute("COMMIT")
            return {"revision_id": revision, "run_id": run_id, "old_contract_sha256": old_digest,
                    "new_contract_sha256": new_digest, "contract": new, "budget": budget,
                    "deadline_s": deadline, "historical_events_unchanged": True,
                    "hardware_commands_sent": 0, "dispatch_authorized": False,
                    "physical_stop_verified": None}
        except BaseException:
            if db.in_transaction:
                db.execute("ROLLBACK")
            raise
        finally:
            db.close()


class PairLedger:
    """SQLite FULL-synchronous at-most-once attempts and immutable run budgets."""

    def __init__(self, path, run_id, contract, *, max_steps=128, max_duration_s=900, clock=time.time):
        if not isinstance(path, (str, os.PathLike)) or not str(path) or str(path) == ":memory:":
            raise ValueError("path must name a persistent SQLite file")
        self.path = str(Path(path).resolve())
        self.run_id = _identifier(run_id, "run_id")
        self.contract_json = _json_object(contract, "contract")
        if type(max_steps) is not int or not 1 <= max_steps <= 2**31 - 1:
            raise ValueError("max_steps must be a positive integer")
        self.max_steps = max_steps
        self.max_duration_s = _number(max_duration_s, "max_duration_s", positive=True)
        if not callable(clock):
            raise ValueError("clock must be callable")
        self.clock = clock
        self._scope_binding = None
        # Deliberately not restorable: a database record cannot revive a crashed
        # action worker or grant a new process its old hold attempt.
        self._hold_instance = uuid.uuid4().hex
        self._hold_pid = os.getpid()
        self._hold_workers = {}
        self._hold_live_actions = set()
        now = _number(clock(), "clock")
        if not math.isfinite(now + self.max_duration_s):
            raise ValueError("Frozen deadline must be finite")
        with self._transaction() as db:
            # Even construction of an unrelated run must not create a budget
            # after an explicit administrative successor has been enrolled.
            if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_scope'").fetchone():
                _execution_scope(db, self.run_id, writable=True)
            db.execute("CREATE TABLE IF NOT EXISTS pair_scope (id INTEGER PRIMARY KEY CHECK(id=1), "
                       "owner TEXT, active_run_id TEXT, fault_id INTEGER, last_time REAL NOT NULL)")
            db.execute("CREATE TABLE IF NOT EXISTS pair_runs (run_id TEXT PRIMARY KEY, contract_json TEXT NOT NULL, "
                       "max_steps INTEGER NOT NULL, max_duration REAL NOT NULL, started_at REAL NOT NULL, "
                       "steps INTEGER NOT NULL DEFAULT 0)")
            db.execute("CREATE TABLE IF NOT EXISTS pair_events (run_id TEXT NOT NULL, event_id TEXT NOT NULL, "
                       "payload_json TEXT NOT NULL, payload_digest TEXT NOT NULL, step INTEGER NOT NULL, "
                       "status TEXT NOT NULL CHECK(status IN ('pending','complete')), owner TEXT NOT NULL, "
                       "began_at REAL NOT NULL, finished_at REAL, receipt_json TEXT, success INTEGER, "
                       "PRIMARY KEY(run_id,event_id), UNIQUE(run_id,step), "
                       "FOREIGN KEY(run_id) REFERENCES pair_runs(run_id))")
            db.execute("CREATE TABLE IF NOT EXISTS pair_faults (id INTEGER PRIMARY KEY AUTOINCREMENT, "
                       "run_id TEXT NOT NULL, owner TEXT, reason TEXT NOT NULL, at REAL NOT NULL)")
            db.execute("CREATE TABLE IF NOT EXISTS pair_joint_sends (run_id TEXT NOT NULL,event_id TEXT NOT NULL, "
                       "owner TEXT NOT NULL,instance_id TEXT NOT NULL,process_id INTEGER NOT NULL,worker_id INTEGER NOT NULL, "
                       "evidence_json TEXT NOT NULL,evidence_digest TEXT NOT NULL,recorded_at REAL NOT NULL, "
                       "PRIMARY KEY(run_id,event_id),FOREIGN KEY(run_id,event_id) REFERENCES pair_events(run_id,event_id))")
            db.execute("CREATE TABLE IF NOT EXISTS pair_hold_requests (run_id TEXT NOT NULL,original_event_id TEXT NOT NULL, "
                       "owner TEXT NOT NULL,fault_id INTEGER NOT NULL,reason TEXT NOT NULL,requested_at REAL NOT NULL, "
                       "PRIMARY KEY(run_id,original_event_id),FOREIGN KEY(fault_id) REFERENCES pair_faults(id))")
            db.execute("CREATE TABLE IF NOT EXISTS pair_holds (run_id TEXT NOT NULL,hold_event_id TEXT NOT NULL, "
                       "original_event_id TEXT NOT NULL,owner TEXT NOT NULL,payload_json TEXT NOT NULL,payload_digest TEXT NOT NULL, "
                       "claim_json TEXT NOT NULL,original_json TEXT NOT NULL,step INTEGER NOT NULL,status TEXT NOT NULL, "
                       "began_at REAL NOT NULL,finished_at REAL,receipt_json TEXT,PRIMARY KEY(run_id,hold_event_id), "
                       "UNIQUE(run_id,original_event_id),FOREIGN KEY(run_id,original_event_id) REFERENCES pair_events(run_id,event_id))")
            db.execute("CREATE TABLE IF NOT EXISTS pair_hold_frames (run_id TEXT NOT NULL,hold_event_id TEXT NOT NULL, "
                       "frame_index INTEGER NOT NULL,frame_json TEXT NOT NULL,outcome TEXT NOT NULL,attempted_at REAL NOT NULL, "
                       "returned_at REAL,error TEXT,PRIMARY KEY(run_id,hold_event_id,frame_index), "
                       "FOREIGN KEY(run_id,hold_event_id) REFERENCES pair_holds(run_id,hold_event_id))")
            db.execute("INSERT OR IGNORE INTO pair_scope(id,last_time) VALUES(1,?)", (now,))
            existing = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (self.run_id,)).fetchone()
            if existing is None:
                db.execute("INSERT INTO pair_runs(run_id,contract_json,max_steps,max_duration,started_at) "
                           "VALUES(?,?,?,?,?)", (self.run_id, self.contract_json, max_steps, self.max_duration_s, now))
            table, key, value, scope = _execution_scope(db, self.run_id, writable=True)
            self._scope_binding = (table, key, value)
            stored_run = existing if existing is not None else db.execute(
                'SELECT * FROM pair_runs WHERE run_id=?', (self.run_id,)).fetchone()
            effective_contract = effective_contract_json(db, stored_run, scope)
            if (effective_contract != self.contract_json or existing is not None and (existing["max_steps"] != max_steps
                  or existing["max_duration"] != self.max_duration_s)):
                raise ValueError("Existing run contract and budgets are frozen")
            self._runtime(db, now=now)

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=5.0, isolation_level=None)
        try:
            db.row_factory = sqlite3.Row
            mode = db.execute("PRAGMA journal_mode").fetchone()[0].lower()
            if mode not in ("delete", "truncate", "persist", "wal"):
                db.execute("PRAGMA journal_mode=DELETE")
            db.execute("PRAGMA synchronous=FULL")
            db.execute("PRAGMA foreign_keys=ON")
            db.execute("PRAGMA busy_timeout=5000")
        except BaseException:
            db.close()
            raise
        return db

    @contextmanager
    def _transaction(self):
        db = self._connect()
        try:
            db.execute("BEGIN IMMEDIATE")
            yield db
            db.execute("COMMIT")
        except BaseException:
            if db.in_transaction:
                db.execute("ROLLBACK")
            raise
        finally:
            db.close()

    def _rows(self, db):
        run = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (self.run_id,)).fetchone()
        if self._scope_binding is not None:
            table, key, value = self._scope_binding
            scope = db.execute("SELECT * FROM " + table + " WHERE " + key + "=?", (value,)).fetchone()
        else:
            _, _, _, scope = _execution_scope(db, self.run_id)
        return run, scope

    def _current_scope(self, db):
        current = _execution_scope(db, self.run_id, writable=True)
        if self._scope_binding is not None and current[:3] != self._scope_binding:
            raise PairLedgerFault("Historical ledger instance cannot enter the new continuation scope")
        return current

    def _scope_update(self, db, assignment, values=()):
        table, key, value, _ = self._current_scope(db)
        db.execute("UPDATE " + table + " SET " + assignment + " WHERE " + key + "=?", (*values, value))

    def _latch(self, db, reason, now, owner=None):
        self._current_scope(db)
        cursor = db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES(?,?,?,?)",
                            (self.run_id, owner, reason, now))
        self._scope_update(db, "fault_id=COALESCE(fault_id,?)", (cursor.lastrowid,))

    def _runtime(self, db, now=None):
        self._current_scope(db)
        run, scope = self._rows(db)
        if now is None:
            try:
                now = _number(self.clock(), "clock")
            except Exception as exc:
                now = scope["last_time"]
                self._latch(db, "clock_invalid: " + str(exc)[:512], now, scope["owner"])
        if now < scope["last_time"]:
            self._latch(db, "clock_rollback", scope["last_time"], scope["owner"])
            now = scope["last_time"]
        else:
            self._scope_update(db, "last_time=?", (now,))
        if now - run["started_at"] >= run["max_duration"] and scope["fault_id"] is None:
            self._latch(db, "duration_budget_exhausted", now, scope["owner"])
        run, scope = self._rows(db)
        return run, scope, now

    def _fault_value(self, db, scope):
        if scope["fault_id"] is None:
            return None
        row = db.execute("SELECT * FROM pair_faults WHERE id=?", (scope["fault_id"],)).fetchone()
        return dict(row)

    def _status(self, db, now):
        run, scope = self._rows(db)
        pending = db.execute("SELECT event_id FROM pair_events WHERE run_id=? AND status='pending'",
                             (self.run_id,)).fetchone()
        fault = self._fault_value(db, scope)
        owner = scope["owner"] if scope["active_run_id"] == self.run_id else None
        elapsed = max(0.0, now - run["started_at"])
        remaining_steps = max(0, run["max_steps"] - run["steps"])
        remaining_s = max(0.0, run["max_duration"] - elapsed)
        result = {"run_id": self.run_id, "status": "fault" if fault else "pending" if pending else "owned" if owner else "detached",
                "owner": owner, "global_owner": scope["owner"], "active_run_id": scope["active_run_id"],
                "steps": run["steps"], "max_steps": run["max_steps"], "remaining_steps": remaining_steps,
                "started_at": run["started_at"], "deadline_s": run["started_at"] + run["max_duration"],
                "elapsed_s": elapsed, "max_duration_s": run["max_duration"],
                "remaining_s": remaining_s, "remaining": {"steps": remaining_steps, "duration_s": remaining_s},
                "fault": fault, "global_fault": fault, "fault_latched": fault is not None,
                "pending_event_id": pending["event_id"] if pending else None,
                "contract": json.loads(effective_contract_json(db, run, scope)), "physical_stop_verified": None}
        if "round_ordinal" in scope.keys():
            field = ('endpoint_continuation' if 'initialization_ordinal' in scope.keys() else
                     'initialization_continuation' if 'feedback_ordinal' in scope.keys() else
                     'feedback_continuation' if 'preparation_ordinal' in scope.keys() else 'preparation_continuation')
            result[field] = {'ordinal':scope['ordinal'],
                'proposal_sha256':scope['proposal_sha256'], 'old_fault_preserved':True,
                'original_run_deadline_preserved':True, 'new_budget_allocated':False}
            scope = db.execute('SELECT * FROM pair_rounds WHERE ordinal=?', (scope['round_ordinal'],)).fetchone()
        elif "previous_owner" in scope.keys():
            result["continuation"] = {"ordinal":scope["ordinal"], "failed_event_id":scope["failed_event_id"],
                "proposal_sha256":scope["proposal_sha256"], "old_fault_preserved":True,
                "original_run_deadline_preserved":True, "new_budget_allocated":False}
            scope = db.execute("SELECT * FROM pair_execution_epochs WHERE ordinal=?",
                               (scope["execution_epoch_ordinal"],)).fetchone()
        if "parent_run_id" in scope.keys():
            parent = db.execute("SELECT * FROM pair_runs WHERE run_id=?", (scope["parent_run_id"],)).fetchone()
            epoch_proposal = json.loads(scope["record_json"])["proposal"]
            result["execution_lineage"] = {
                "parent_run_id": parent["run_id"], "parent_steps": parent["steps"],
                "parent_deadline_s": parent["started_at"]+parent["max_duration"],
                "parent_max_duration_s": parent["max_duration"],
                "cumulative_steps": epoch_proposal.get("snapshot",{}).get("cumulative_prior_steps",parent["steps"])+run["steps"],
                "cumulative_step_ceiling": epoch_proposal["cumulative_step_ceiling"],
                "budget_policy": epoch_proposal.get("budget_policy", "preserve_parent_ceiling"),
                "separately_authorized_duration_s": run["max_duration"],
                "proposal_sha256": scope["proposal_sha256"], "old_fault_preserved": True}
        return result

    def _live_owner(self, db, scope, owner, now, *, allow_fault=False):
        if _retired_owner(scope, owner):
            return PairLedgerFault("The retired owner cannot write into its continuation scope")
        if scope["owner"] != owner or scope["active_run_id"] != self.run_id:
            self._latch(db, "owner_mismatch", now, owner)
            return PairLedgerFault("Owner mismatch; database dispatch fault latched")
        if scope["fault_id"] is not None and not allow_fault:
            return PairLedgerFault("Database dispatch fault is latched")
        return None

    def claim(self, owner):
        owner = _identifier(owner, "owner")
        error = None
        with self._transaction() as db:
            _, initial_scope = self._rows(db)
            if _retired_owner(initial_scope, owner):
                raise PairLedgerFault("The retired owner cannot claim the continuation")
            run, scope, now = self._runtime(db)
            pending = db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone()
            if scope["fault_id"] is not None:
                error = PairLedgerFault("Database dispatch fault is latched")
            elif scope["owner"] is not None or pending is not None:
                self._latch(db, "claim_requires_clean_detach", now, owner)
                error = PairLedgerFault("Existing owner or pending attempt; database fault latched")
            else:
                self._scope_update(db, "owner=?,active_run_id=?", (owner, self.run_id))
            result = self._status(db, now)
        if error:
            raise error
        return result

    def begin(self, owner, event_id, payload):
        owner, event_id = _identifier(owner, "owner"), _identifier(event_id, "event_id")
        encoded = _json_object(payload, "payload")
        digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        error, result = None, None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now)
            event = db.execute("SELECT * FROM pair_events WHERE run_id=? AND event_id=?",
                               (self.run_id, event_id)).fetchone()
            if error is None and event is not None and event["payload_json"] != encoded:
                self._latch(db, "event_payload_conflict", now, owner)
                error = PairLedgerFault("Event ID payload differs; database fault latched")
            elif error is None and event is not None and event["status"] == "complete":
                result = {"replayed": True, "event_id": event_id, "step": event["step"],
                          "payload_digest": event["payload_digest"], "receipt": json.loads(event["receipt_json"])}
            elif error is None:
                pending = db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone()
                if pending:
                    self._latch(db, "pending_attempt_cannot_replay", now, owner)
                    error = PairLedgerFault("Pending attempt is uncertain; database fault latched")
                elif run["steps"] >= run["max_steps"]:
                    self._latch(db, "step_budget_exhausted", now, owner)
                    error = PairLedgerFault("Step budget exhausted; database fault latched")
                else:
                    from .supported_gripper_recovery import check_request
                    check_request(db, self.run_id, owner, payload)
                    step = run["steps"] + 1
                    db.execute("INSERT INTO pair_events(run_id,event_id,payload_json,payload_digest,step,status,owner,began_at) "
                               "VALUES(?,?,?,?,?,'pending',?,?)", (self.run_id, event_id, encoded, digest, step, owner, now))
                    db.execute("UPDATE pair_runs SET steps=? WHERE run_id=?", (step, self.run_id))
                    result = {"replayed": False, "event_id": event_id, "step": step, "payload_digest": digest}
        if error:
            raise error
        if not result["replayed"] and payload.get("kind") == "joint":
            self._hold_live_actions.add(event_id)
        return result

    def _hold_refuse(self, db, reason, now, owner):
        self._latch(db, reason, now, owner)
        return PairLedgerFault(reason + "; original fault and budgets retained")

    def _hold_source(self, db, owner, event_id):
        source = db.execute("SELECT * FROM pair_joint_sends WHERE run_id=? AND event_id=?",
                            (self.run_id, event_id)).fetchone()
        if (source is None or source["owner"] != owner or source["instance_id"] != self._hold_instance
                or source["process_id"] != os.getpid() or self._hold_pid != os.getpid()
                or source["worker_id"] != threading.get_ident()
                or self._hold_workers.get(event_id) is not threading.current_thread()
                or event_id not in self._hold_live_actions):
            raise ValueError("hold_requires_original_live_worker")
        return source

    def _validate_original_send(self, original, event, run, owner, now):
        required = {"event_id", "identity", "worker_thread_id", "send_state", "target_raw", "reference",
                    "frame_receipts", "limits", "deadline_at", "fault"}
        optional = {"geometry_source"} if "geometry_source" in original else set()
        if set(original) != required | optional:
            raise ValueError("original_send_schema")
        identity = original["identity"]
        _hold_identity(identity)
        if (identity["run_id"] != self.run_id or identity["owner"] != owner
                or identity["model"] not in ("piper", "piper_x") or identity["firmware_profile"] != "default"
                or original["event_id"] != event["event_id"] or original["fault"] is not None
                or type(original["worker_thread_id"]) is not int
                or original["worker_thread_id"] != threading.get_ident()
                or original["send_state"] != "all_frames_returned"):
            raise ValueError("original_send_binding")
        if optional:
            geometry = original["geometry_source"]
            if (type(geometry) is not dict or set(geometry) != {"mode", "model", "sdk_commit", "constants_sha256"}
                    or geometry["mode"] != "model_joint_geometry_v1" or geometry["model"] != identity["model"]
                    or type(geometry["sdk_commit"]) is not str or len(geometry["sdk_commit"]) != 40
                    or any(c not in "0123456789abcdef" for c in geometry["sdk_commit"])):
                raise ValueError("original_geometry_binding")
            _hold_hash(geometry["constants_sha256"])
        expected = _hold_frames(original["target_raw"])
        payload = json.loads(event["payload_json"])
        target = payload.get("target")
        if (payload.get("kind") != "joint" or payload.get("arm") != identity["arm"]
                or type(target) is not list or len(target) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1000 for v in target)
                or [round(v * 180 / math.pi * 1000) for v in target] != original["target_raw"]
                or [round(v * (180 / math.pi) * 1000) for v in target] != original["target_raw"]
                or [round(math.degrees(v) * 1000) for v in target] != original["target_raw"]):
            raise ValueError("original_target_binding")
        if _number(original["deadline_at"], "deadline_at") != run["started_at"] + run["max_duration"]:
            raise ValueError("original_deadline_changed")
        reference = original["reference"]
        if (type(reference) is not dict or set(reference) != {"sample_id", "identity", "captured_at", "arms"}
                or reference["identity"] != identity or type(reference["arms"]) is not dict
                or set(reference["arms"]) != {"left", "right"}):
            raise ValueError("original_reference_binding")
        _identifier(reference["sample_id"], "sample_id")
        previous = _number(reference["captured_at"], "captured_at")
        if previous > now:
            raise ValueError("original_reference_future")
        receipts = original["frame_receipts"]
        if type(receipts) is not list or len(receipts) != 4:
            raise ValueError("original_send_incomplete")
        for receipt, frame in zip(receipts, expected):
            if (type(receipt) is not dict or set(receipt) != {"frame", "outcome", "returned_at"}
                    or receipt["outcome"] != "returned" or not _hold_frame_equal(receipt["frame"], frame)):
                raise ValueError("original_send_incomplete")
            at = _number(receipt["returned_at"], "returned_at")
            if not max(previous, event["began_at"]) <= at <= now:
                raise ValueError("original_send_time")
            previous = at
        # Physical limits are frozen verbatim here; the mode-specific helper
        # checks their numerical applicability against live measured feedback.
        limits = original["limits"]
        if (type(limits) is not dict or set(limits) != {"joint_limits_raw", "workspace_min_m", "workspace_max_m",
                                                      "max_translation_m", "max_rotation_rad"}):
            raise ValueError("original_limits_schema")

    def record_original_send(self, owner, event_id, original_event):
        """Persist adapter-resolved full wire facts; never recover a crashed send.

        Only the original live ledger instance may bind its action worker. This
        internal callback does not authenticate a caller's feedback or CAD data.
        Partial/unknown sends have no hold source record and cannot be upgraded.
        """
        owner, event_id = _identifier(owner, "owner"), _identifier(event_id, "event_id")
        encoded = _json_object(original_event, "original_event")
        error, result = None, None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now, allow_fault=True)
            event = db.execute("SELECT * FROM pair_events WHERE run_id=? AND event_id=?",
                               (self.run_id, event_id)).fetchone()
            prior = db.execute("SELECT * FROM pair_joint_sends WHERE run_id=? AND event_id=?",
                               (self.run_id, event_id)).fetchone()
            if error is None:
                try:
                    if (event is None or event["owner"] != owner or event["status"] != "pending"
                            or self._hold_pid != os.getpid() or event_id not in self._hold_live_actions):
                        raise ValueError("original_send_without_live_action")
                    self._validate_original_send(original_event, event, run, owner, now)
                    if prior is not None:
                        self._hold_source(db, owner, event_id)
                        if prior["evidence_json"] != encoded:
                            raise ValueError("original_send_conflict")
                    else:
                        db.execute("INSERT INTO pair_joint_sends VALUES(?,?,?,?,?,?,?,?,?)",
                                   (self.run_id, event_id, owner, self._hold_instance, os.getpid(), threading.get_ident(),
                                    encoded, hashlib.sha256(encoded.encode()).hexdigest(), now))
                    result = {"event_id": event_id, "recorded": True, "replayed": prior is not None}
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    error = self._hold_refuse(db, "hold_source_refused: " + str(exc), now, owner)
        if error:
            raise error
        if not result["replayed"]:
            self._hold_workers[event_id] = threading.current_thread()
        return result

    def request_hold_cancel(self, owner, original_event_id, reason):
        """Latch explicit client cancellation, not a generic fault exemption.

        This internal host route must only be called for explicit user/client
        cancellation. The public normal fault route never creates this record.
        A request can precede the last original frame; it cannot prove full TX.
        """
        owner = _identifier(owner, "owner")
        original_event_id = _identifier(original_event_id, "original_event_id")
        if type(reason) is not str or not reason.strip() or len(reason) > 4096:
            raise ValueError("reason must be a nonempty string of at most 4096 characters")
        error, result = None, None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now, allow_fault=True)
            event = db.execute("SELECT * FROM pair_events WHERE run_id=? AND event_id=?",
                               (self.run_id, original_event_id)).fetchone()
            prior = db.execute("SELECT * FROM pair_hold_requests WHERE run_id=? AND original_event_id=?",
                               (self.run_id, original_event_id)).fetchone()
            if error is None:
                if (event is None or event["owner"] != owner or event["status"] != "pending"
                        or json.loads(event["payload_json"]).get("kind") != "joint"
                        or original_event_id not in self._hold_live_actions or self._hold_pid != os.getpid()):
                    error = self._hold_refuse(db, "hold_cancel_without_live_joint_action", now, owner)
                elif prior is not None and (prior["owner"] != owner or prior["reason"] != reason):
                    error = self._hold_refuse(db, "hold_cancel_conflict", now, owner)
                elif prior is None and scope["fault_id"] is not None:
                    error = PairLedgerFault("Preexisting fault cannot become a hold cancellation")
                elif prior is None:
                    self._latch(db, "explicit_hold_cancel: " + reason, now, owner)
                    fault_id = self._rows(db)[1]["fault_id"]
                    db.execute("INSERT INTO pair_hold_requests VALUES(?,?,?,?,?,?)",
                               (self.run_id, original_event_id, owner, fault_id, reason, now))
                    prior = db.execute("SELECT * FROM pair_hold_requests WHERE run_id=? AND original_event_id=?",
                                       (self.run_id, original_event_id)).fetchone()
                    replayed = False
                else:
                    replayed = True
                if error is None:
                    fault = db.execute("SELECT * FROM pair_faults WHERE id=?", (prior["fault_id"],)).fetchone()
                    result = {"original_event_id": original_event_id, "fault": dict(fault), "reason": reason,
                              "requested_at": prior["requested_at"], "replayed": replayed}
        if error:
            raise error
        return result

    def _hold_gate(self, db, owner, original_event_id, run, scope, now):
        error = self._live_owner(db, scope, owner, now, allow_fault=True)
        if error:
            raise ValueError(str(error))
        source = self._hold_source(db, owner, original_event_id)
        event = db.execute("SELECT * FROM pair_events WHERE run_id=? AND event_id=?",
                           (self.run_id, original_event_id)).fetchone()
        request = db.execute("SELECT * FROM pair_hold_requests WHERE run_id=? AND original_event_id=?",
                             (self.run_id, original_event_id)).fetchone()
        latest = db.execute("SELECT MAX(id) FROM pair_faults").fetchone()[0]
        if event is None or event["owner"] != owner or event["status"] != "pending":
            raise ValueError("hold_original_not_pending")
        if (request is None or request["owner"] != owner or scope["fault_id"] != request["fault_id"]
                or latest != request["fault_id"]):
            raise ValueError("hold_requires_only_explicit_cancel_fault")
        if now >= run["started_at"] + run["max_duration"]:
            raise ValueError("hold_original_deadline_exhausted")
        return source, request

    def begin_hold(self, owner, original_event_id, hold_event_id, payload):
        """Reserve one bounded same-worker J hold; never replay a pending claim."""
        owner = _identifier(owner, "owner")
        original_event_id = _identifier(original_event_id, "original_event_id")
        hold_event_id = _identifier(hold_event_id, "hold_event_id")
        encoded = _json_object(payload, "hold payload")
        error, result = None, None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            try:
                source, request = self._hold_gate(db, owner, original_event_id, run, scope, now)
                original = json.loads(source["evidence_json"])
                if set(payload) != {"operation", "identity", "target_raw", "expected_frames", "sample_ref"}:
                    raise ValueError("hold_payload_schema")
                if payload["operation"] != "joint_hold_current" or payload["identity"] != original["identity"]:
                    raise ValueError("hold_payload_identity")
                frames = _hold_frames(payload["target_raw"])
                if (type(payload["expected_frames"]) is not list or len(payload["expected_frames"]) != 4
                        or not all(_hold_frame_equal(f, e) for f, e in zip(payload["expected_frames"], frames))):
                    raise ValueError("hold_payload_frames")
                existing = db.execute("SELECT * FROM pair_holds WHERE run_id=? AND hold_event_id=?",
                                      (self.run_id, hold_event_id)).fetchone()
                if existing is not None:
                    if existing["payload_json"] != encoded or existing["original_event_id"] != original_event_id:
                        raise ValueError("hold_payload_conflict")
                    if existing["status"] != "complete":
                        raise ValueError("pending_hold_cannot_replay")
                    result = {"replayed": True, "hold_event_id": hold_event_id, "step": existing["step"],
                              "receipt": json.loads(existing["receipt_json"])}
                else:
                    if (hold_event_id == original_event_id or db.execute(
                            "SELECT 1 FROM pair_events WHERE run_id=? AND event_id=?", (self.run_id, hold_event_id)).fetchone()
                            or db.execute("SELECT 1 FROM pair_holds WHERE run_id=? AND original_event_id=?",
                                          (self.run_id, original_event_id)).fetchone()):
                        raise ValueError("hold_already_reserved_or_event_collision")
                    sample = payload["sample_ref"]
                    if type(sample) is not dict or set(sample) != {"sample_id", "captured_at", "sha256"}:
                        raise ValueError("hold_sample_reference")
                    _identifier(sample["sample_id"], "sample_id")
                    _hold_hash(sample["sha256"])
                    captured = _number(sample["captured_at"], "captured_at")
                    if (not 0 <= now-captured <= .1 or captured < original["frame_receipts"][-1]["returned_at"]
                            or captured < request["requested_at"]):
                        raise ValueError("hold_sample_not_current")
                    if run["steps"] >= run["max_steps"]:
                        raise ValueError("hold_step_budget_exhausted")
                    original["fault"] = {"reason": "explicit_hold_cancel: " + request["reason"],
                                         "event_id": "hold-cancel-" + str(request["fault_id"]), "at": request["requested_at"]}
                    # Match the pure helper's specified JSON digest including
                    # default ASCII escaping, independently of storage encoding.
                    digest = hashlib.sha256(json.dumps(original, sort_keys=True, separators=(",", ":"),
                                                       allow_nan=False).encode()).hexdigest()
                    claim = {"hold_event_id": hold_event_id, "original_event_id": original_event_id,
                             "identity": original["identity"], "claimed_at": now, "original_event_sha256": digest}
                    step = run["steps"] + 1
                    db.execute("INSERT INTO pair_holds VALUES(?,?,?,?,?,?,?,?,?,'pending',?,NULL,NULL)",
                               (self.run_id, hold_event_id, original_event_id, owner, encoded,
                                hashlib.sha256(encoded.encode()).hexdigest(), _json_object(claim, "claim"),
                                _json_object(original, "bound original"), step, now))
                    db.execute("UPDATE pair_runs SET steps=? WHERE run_id=?", (step, self.run_id))
                    result = {"replayed": False, "hold_event_id": hold_event_id, "step": step,
                              "claim": claim, "original_event": original}
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                error = self._hold_refuse(db, "hold_claim_refused: " + str(exc), now, owner)
        if error:
            raise error
        return result

    def begin_hold_frame(self, owner, hold_event_id, index, frame):
        """Durably mark the next exact frame pending BEFORE the final RX check.

        A successful return is bookkeeping, not a send token: the bound adapter
        must check fresh feedback and the pure hold helper after this SQLite IO.
        """
        owner, hold_event_id = _identifier(owner, "owner"), _identifier(hold_event_id, "hold_event_id")
        encoded = _json_object(frame, "frame")
        error = None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            try:
                hold = db.execute("SELECT * FROM pair_holds WHERE run_id=? AND hold_event_id=?",
                                  (self.run_id, hold_event_id)).fetchone()
                if hold is None or hold["owner"] != owner or hold["status"] != "pending":
                    raise ValueError("hold_not_pending")
                self._hold_gate(db, owner, hold["original_event_id"], run, scope, now)
                rows = db.execute("SELECT * FROM pair_hold_frames WHERE run_id=? AND hold_event_id=? ORDER BY frame_index",
                                  (self.run_id, hold_event_id)).fetchall()
                expected = json.loads(hold["payload_json"])["expected_frames"]
                if (type(index) is not int or not 0 <= index < 4 or index != len(rows)
                        or any(row["outcome"] != "returned" for row in rows)
                        or not _hold_frame_equal(frame, expected[index])):
                    raise ValueError("hold_frame_order_or_payload")
                db.execute("INSERT INTO pair_hold_frames VALUES(?,?,?,?, 'pending',?,NULL,NULL)",
                           (self.run_id, hold_event_id, index, encoded, now))
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                error = self._hold_refuse(db, "hold_frame_refused: " + str(exc), now, owner)
        if error:
            raise error
        return {"hold_event_id": hold_event_id, "index": index, "outcome": "pending"}

    def finish_hold_frame(self, owner, hold_event_id, index, outcome, error=None):
        """Record one send return even after a subsequent fault/deadline.

        'returned' only means local send returned. Neither it nor a completed
        four-frame sequence is a firmware acceptance or physical stop receipt.
        """
        owner, hold_event_id = _identifier(owner, "owner"), _identifier(hold_event_id, "hold_event_id")
        if (type(index) is not int or not 0 <= index < 4 or outcome not in ("returned", "exception", "unknown")
                or (error is not None and (type(error) is not str or len(error) > 4096))):
            raise ValueError("Invalid hold frame return")
        refusal = None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            try:
                live_error = self._live_owner(db, scope, owner, now, allow_fault=True)
                if live_error:
                    raise ValueError(str(live_error))
                hold = db.execute("SELECT * FROM pair_holds WHERE run_id=? AND hold_event_id=?",
                                  (self.run_id, hold_event_id)).fetchone()
                if hold is None or hold["owner"] != owner or hold["status"] == "complete":
                    raise ValueError("hold_return_without_pending_claim")
                self._hold_source(db, owner, hold["original_event_id"])
                row = db.execute("SELECT * FROM pair_hold_frames WHERE run_id=? AND hold_event_id=? AND frame_index=?",
                                 (self.run_id, hold_event_id, index)).fetchone()
                if row is None or row["outcome"] != "pending":
                    raise ValueError("hold_return_without_pending_frame")
                db.execute("UPDATE pair_hold_frames SET outcome=?,returned_at=?,error=? "
                           "WHERE run_id=? AND hold_event_id=? AND frame_index=?",
                           (outcome, now, error, self.run_id, hold_event_id, index))
                if outcome != "returned":
                    db.execute("UPDATE pair_holds SET status='fault' WHERE run_id=? AND hold_event_id=?",
                               (self.run_id, hold_event_id))
                    self._latch(db, "hold_send_" + outcome, now, owner)
            except (ValueError, TypeError, KeyError) as exc:
                refusal = self._hold_refuse(db, "hold_return_refused: " + str(exc), now, owner)
        if refusal:
            raise refusal
        return {"hold_event_id": hold_event_id, "index": index, "outcome": outcome}

    def finish_hold(self, owner, hold_event_id, receipt):
        """Store bound adapter diagnostics; preserve first fault and all budgets."""
        owner, hold_event_id = _identifier(owner, "owner"), _identifier(hold_event_id, "hold_event_id")
        encoded = _json_object(receipt, "hold receipt")
        error = None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            try:
                live_error = self._live_owner(db, scope, owner, now, allow_fault=True)
                if live_error:
                    raise ValueError(str(live_error))
                hold = db.execute("SELECT * FROM pair_holds WHERE run_id=? AND hold_event_id=?",
                                  (self.run_id, hold_event_id)).fetchone()
                if hold is None or hold["owner"] != owner:
                    raise ValueError("hold_finish_without_claim")
                self._hold_source(db, owner, hold["original_event_id"])
                payload = json.loads(hold["payload_json"])
                if (receipt.get("hold_event_id") != hold_event_id
                        or receipt.get("original_event_id") != hold["original_event_id"]
                        or receipt.get("identity") != payload["identity"]
                        or receipt.get("physical_stop_verified") is not None
                        or receipt.get("original_target_cancelled") is not None
                        or type(receipt.get("frames_complete")) is not bool
                        or type(receipt.get("hold_observed")) is not bool):
                    raise ValueError("hold_receipt_binding")
                rows = db.execute("SELECT * FROM pair_hold_frames WHERE run_id=? AND hold_event_id=? ORDER BY frame_index",
                                  (self.run_id, hold_event_id)).fetchall()
                complete = len(rows) == 4 and all(row["outcome"] == "returned" for row in rows)
                if (receipt["frames_complete"] != complete or (receipt["hold_observed"] and not complete)
                        or ("target_raw" in receipt and receipt["target_raw"] not in (None, payload["target_raw"]))
                        or ("expected_frames" in receipt and receipt["expected_frames"] != payload["expected_frames"]
                            and not (receipt["expected_frames"] in (None, []) and not rows
                                     and not receipt["frames_complete"] and not receipt["hold_observed"]))):
                    raise ValueError("hold_receipt_frame_evidence")
                if hold["receipt_json"] is not None:
                    if hold["receipt_json"] != encoded:
                        raise ValueError("hold_receipt_conflict")
                else:
                    db.execute("UPDATE pair_holds SET status='complete',finished_at=?,receipt_json=? "
                               "WHERE run_id=? AND hold_event_id=?", (now, encoded, self.run_id, hold_event_id))
                    if not receipt["hold_observed"]:
                        self._latch(db, "hold_not_observed", now, owner)
            except (ValueError, TypeError, KeyError) as exc:
                error = self._hold_refuse(db, "hold_finish_refused: " + str(exc), now, owner)
        if error:
            raise error
        return json.loads(encoded)

    def hold_event(self, hold_event_id):
        """Read historical facts only; cannot reconstruct a worker or retry."""
        hold_event_id = _identifier(hold_event_id, "hold_event_id")
        db = sqlite3.connect(Path(self.path).as_uri()+"?mode=ro", uri=True, timeout=5.0)
        try:
            db.row_factory = sqlite3.Row
            db.execute("PRAGMA query_only=ON")
            db.execute("BEGIN")
            row = db.execute("SELECT * FROM pair_holds WHERE run_id=? AND hold_event_id=?",
                             (self.run_id, hold_event_id)).fetchone()
            if row is None:
                return None
            frames = db.execute("SELECT * FROM pair_hold_frames WHERE run_id=? AND hold_event_id=? ORDER BY frame_index",
                                (self.run_id, hold_event_id)).fetchall()
            return {"hold_event_id": hold_event_id, "original_event_id": row["original_event_id"],
                    "owner": row["owner"], "status": row["status"], "step": row["step"],
                    "payload": json.loads(row["payload_json"]), "claim": json.loads(row["claim_json"]),
                    "original_event": json.loads(row["original_json"]),
                    "frame_receipts": [{"index": f["frame_index"], "frame": json.loads(f["frame_json"]),
                                        "outcome": f["outcome"], "attempted_at": f["attempted_at"],
                                        "returned_at": f["returned_at"], "error": f["error"]} for f in frames],
                    "receipt": json.loads(row["receipt_json"]) if row["receipt_json"] is not None else None,
                    "dispatch_authorized": False, "physical_stop_verified": None}
        finally:
            db.close()

    def finish(self, owner, event_id, receipt, success=True):
        owner, event_id = _identifier(owner, "owner"), _identifier(event_id, "event_id")
        encoded = _json_object(receipt, "receipt")
        if type(success) is not bool:
            raise ValueError("success must be a boolean")
        error = None
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now, allow_fault=True)
            event = db.execute("SELECT * FROM pair_events WHERE run_id=? AND event_id=?",
                               (self.run_id, event_id)).fetchone()
            if error is None and (event is None or event["owner"] != owner):
                self._latch(db, "finish_without_owned_attempt", now, owner)
                error = PairLedgerFault("No matching owned attempt; database fault latched")
            elif error is None and event["status"] == "complete":
                if event["receipt_json"] != encoded or bool(event["success"]) is not success:
                    self._latch(db, "event_receipt_conflict", now, owner)
                    error = PairLedgerFault("Completed receipt differs; database fault latched")
            elif error is None:
                db.execute("UPDATE pair_events SET status='complete',receipt_json=?,success=?,finished_at=? "
                           "WHERE run_id=? AND event_id=?", (encoded, int(success), now, self.run_id, event_id))
                if not success:
                    self._latch(db, "execution_receipt_failed", now, owner)
        if error:
            raise error
        return json.loads(encoded)

    def fault(self, owner, reason):
        owner = _identifier(owner, "owner")
        if type(reason) is not str or not reason.strip() or len(reason) > 4096:
            raise ValueError("reason must be a nonempty string of at most 4096 characters")
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now, allow_fault=True)
            if error is None:
                self._latch(db, reason, now, owner)
            result = self._status(db, now)
        if error:
            raise error
        return result

    def release(self, owner):
        from .grasp_episode import is_resolved_release

        owner = _identifier(owner, "owner")
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            error = self._live_owner(db, scope, owner, now)
            pending = db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone()
            unresolved_grasp = False
            if db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pair_grasp_episodes'").fetchone():
                episodes = db.execute("SELECT state_json FROM pair_grasp_episodes WHERE run_id=?",
                                      (self.run_id,)).fetchall()
                for row in episodes:
                    state = json.loads(row["state_json"])
                    # V2 release needs new separation/support RGB and a fresh
                    # stability trace. V1's mechanical opening alone is not a
                    # resolved grasp, nor does either mean physical stopping.
                    if state.get("status") != "empty" and not is_resolved_release(state):
                        unresolved_grasp = True
                        break
            if error is None and pending:
                self._latch(db, "release_with_pending_attempt", now, owner)
                error = PairLedgerFault("Pending attempt cannot detach; database fault latched")
            elif error is None and unresolved_grasp:
                self._latch(db, "release_with_unresolved_grasp", now, owner)
                error = PairLedgerFault("Unresolved grasp cannot cleanly detach; database fault latched")
            elif error is None:
                self._scope_update(db, "owner=NULL,active_run_id=NULL")
            result = self._status(db, now)
        if error:
            raise error
        return result

    def status(self):
        with self._transaction() as db:
            run, scope, now = self._runtime(db)
            return self._status(db, now)

    def peek_status(self):
        """Read after a fault without writing clocks, faults or ownership.

        This snapshot never grants dispatch or refreshes an owner. Normal
        admission must still call the transactional status/begin methods.
        An invalid/regressed clock uses the last recorded time for display.
        """
        db = sqlite3.connect(Path(self.path).as_uri()+"?mode=ro", uri=True, timeout=5)
        try:
            db.row_factory = sqlite3.Row
            db.execute("PRAGMA query_only=ON")
            db.execute("BEGIN")
            run, scope = self._rows(db)
            valid = True
            try:
                now = _number(self.clock(), "clock")
                if now < scope["last_time"]:
                    valid = False
                    now = scope["last_time"]
            except Exception:
                valid = False
                now = scope["last_time"]
            result = self._status(db, now)
            result.update(read_only_snapshot=True, time_valid=valid,
                          dispatch_authorized=False)
            return result
        finally:
            db.close()

    def event(self, event_id):
        """Read one stored attempt without claiming, renewing time or dispatching.

        A completed receipt can be shown before fresh-scene admission. Reading
        a pending record never grants retry or releases its original owner.
        """
        event_id = _identifier(event_id, "event_id")
        db = sqlite3.connect(Path(self.path).as_uri() + "?mode=ro", uri=True, timeout=5.0)
        try:
            db.row_factory = sqlite3.Row
            db.execute("PRAGMA query_only=ON")
            row = db.execute("SELECT event_id,status,payload_json,receipt_json,success,owner,step "
                             "FROM pair_events WHERE run_id=? AND event_id=?",
                             (self.run_id, event_id)).fetchone()
            if row is None:
                return None
            payload = json.loads(row["payload_json"])
            receipt = json.loads(row["receipt_json"]) if row["receipt_json"] is not None else None
            _json_object(payload, "stored payload")
            if receipt is not None:
                _json_object(receipt, "stored receipt")
            if row["success"] not in (None, 0, 1):
                raise PairLedgerError("Stored event success is not a boolean or null")
            return {"event_id": row["event_id"], "status": row["status"], "payload": payload,
                    "receipt": receipt, "success": bool(row["success"]) if row["success"] is not None else None,
                    "owner": row["owner"], "step": row["step"]}
        finally:
            db.close()
