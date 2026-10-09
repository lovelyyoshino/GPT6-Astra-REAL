"""Durable host-side grasp records in the existing pair ledger transaction.

No device imports or action admission. The caller supplies evidence already
resolved by the owning host; persistence and schema checks do not authenticate
RGB or establish a physical grasp. The real device still owns dispatch gates.
"""
import json
from .feedback_tolerance import task_policy
from pathlib import Path
import sqlite3

from .grasp_episode import apply_event, is_resolved_release, new_episode
from .pair_ledger import PairLedgerError, _identifier, _json_object


class GraspStore:
    def __init__(self, ledger):
        self.ledger = ledger
        with ledger._transaction() as db:
            db.execute("CREATE TABLE IF NOT EXISTS pair_grasp_episodes ("
                       "run_id TEXT NOT NULL, episode_id TEXT NOT NULL, arm TEXT NOT NULL, "
                       "owner TEXT NOT NULL, revision INTEGER NOT NULL, state_json TEXT NOT NULL, "
                       "PRIMARY KEY(run_id,episode_id), "
                       "FOREIGN KEY(run_id) REFERENCES pair_runs(run_id))")

    def _mutate(self, owner, operation):
        owner = _identifier(owner, "owner")
        error, result = None, None
        with self.ledger._transaction() as db:
            run, scope, now = self.ledger._runtime(db)
            error = self.ledger._live_owner(db, scope, owner, now)
            if error is None:
                pending = db.execute("SELECT 1 FROM pair_events WHERE status='pending' LIMIT 1").fetchone()
                if pending:
                    error = PairLedgerError("A pending physical attempt cannot change grasp state")
                else:
                    result = operation(db, run, now)
        if error is not None:
            raise error
        return result

    def create(self, owner, *, episode_id, arm, object_id, epoch):
        """Start empty bookkeeping; creation grants no grasp or movement rights."""
        episode_id = _identifier(episode_id, "episode_id")
        object_id, epoch = _identifier(object_id, "object_id"), _identifier(epoch, "epoch")
        if arm not in ("left", "right"):
            raise ValueError("Explicit left/right arm required")

        def operation(db, run, now):
            old = db.execute("SELECT state_json FROM pair_grasp_episodes WHERE run_id=? AND episode_id=?",
                             (self.ledger.run_id, episode_id)).fetchone()
            if old is not None:
                state = json.loads(old["state_json"])
                expected = dict(episode_id=episode_id, arm=arm, run_id=self.ledger.run_id,
                                owner=owner, epoch=epoch, object_id=object_id)
                if state["identity"] != expected:
                    raise PairLedgerError("Existing grasp identity is immutable")
                return state
            rows = db.execute("SELECT state_json FROM pair_grasp_episodes WHERE run_id=? AND arm=?",
                              (self.ledger.run_id, arm)).fetchall()
            for row in rows:
                prior = json.loads(row["state_json"])
                if is_resolved_release(prior):
                    continue
                if prior["status"] == "empty" and prior["identity"]["owner"] != owner:
                    continue  # Zero-TX bookkeeping from a cleanly detached owner.
                raise PairLedgerError("Previous grasp episode on this arm has not been explicitly released")
            state = new_episode(episode_id=episode_id, arm=arm, run_id=self.ledger.run_id,
                                owner=owner, epoch=epoch, object_id=object_id, created_at=now,
                                deadline_at=run["started_at"]+run["max_duration"],
                                feedback_policy=task_policy(json.loads(self.ledger.contract_json).get("task",{})))
            encoded = _json_object(state, "grasp state")
            db.execute("INSERT INTO pair_grasp_episodes VALUES(?,?,?,?,?,?)",
                       (self.ledger.run_id, episode_id, arm, owner, state["revision"], encoded))
            return json.loads(encoded)
        return self._mutate(owner, operation)

    def append(self, owner, episode_id, event, *, expected_revision):
        """Atomically apply one resolved-evidence event without spending/resetting TX budget."""
        episode_id = _identifier(episode_id, "episode_id")
        if type(expected_revision) is not int or expected_revision < 0:
            raise ValueError("Nonnegative expected grasp revision required")
        # Freeze caller-owned data before acquiring the transaction.
        event = json.loads(_json_object(event, "grasp event"))

        def operation(db, run, now):
            row = db.execute("SELECT * FROM pair_grasp_episodes WHERE run_id=? AND episode_id=?",
                             (self.ledger.run_id, episode_id)).fetchone()
            if row is None or row["owner"] != owner:
                raise PairLedgerError("No grasp episode belonging to the current owner")
            state = json.loads(row["state_json"])
            # Replay is recognized by the pure transition function; only a new
            # transition needs the latest revision. A replay cannot renew TTL.
            if row["revision"] != expected_revision and event.get("event_id") not in state["events"]:
                raise PairLedgerError("Stale grasp revision; read the stored episode first")
            updated = apply_event(state, event, now=now)
            if updated["revision"] != state["revision"]:
                if row["revision"] != expected_revision:
                    raise PairLedgerError("Stale grasp revision; read the stored episode first")
                encoded = _json_object(updated, "grasp state")
                db.execute("UPDATE pair_grasp_episodes SET revision=?,state_json=? "
                           "WHERE run_id=? AND episode_id=? AND revision=?",
                           (updated["revision"], encoded, self.ledger.run_id, episode_id, expected_revision))
            return updated
        return self._mutate(owner, operation)

    def read(self, episode_id=None):
        """Read records after fault; never renew or authorize retained grasps.

        The pair platform fault always overrides these historical episode
        fields. A retained_static record is not permission to resume actions.
        """
        if episode_id is not None:
            episode_id = _identifier(episode_id, "episode_id")
        db = sqlite3.connect(Path(self.ledger.path).as_uri()+"?mode=ro", uri=True, timeout=5)
        try:
            db.execute("PRAGMA query_only=ON")
            sql = "SELECT state_json FROM pair_grasp_episodes WHERE run_id=?"
            args = [self.ledger.run_id]
            if episode_id is not None:
                sql += " AND episode_id=?"
                args.append(episode_id)
            rows = db.execute(sql+" ORDER BY episode_id", args).fetchall()
            states = [json.loads(row[0]) for row in rows]
            return (states[0] if states else None) if episode_id is not None else states
        finally:
            db.close()
