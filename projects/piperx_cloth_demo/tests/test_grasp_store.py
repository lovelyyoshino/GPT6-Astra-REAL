"""Persistent grasp bookkeeping shares ownership and budget with the pair."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_tools.grasp_episode import GraspEpisodeError, is_resolved_release
from robot_tools.grasp_store import GraspStore
from robot_tools.pair_ledger import PairLedger, PairLedgerError, PairLedgerFault
from test_grasp_episode import (candidate_event, retention_event, event, measurement, scene,
                                release_visual, release_confirmation_event)


class GraspStoreTests(unittest.TestCase):
    def setUp(self):
        self.guard = patch("socket.socket", side_effect=AssertionError("Offline tests only"))
        self.guard.start()
        self.addCleanup(self.guard.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/"pair.sqlite"
        self.now = 100.
        self.ledger = PairLedger(self.path, "run-1", {}, max_duration_s=100., clock=lambda: self.now)
        self.ledger.claim("owner-1")
        self.store = GraspStore(self.ledger)

    def create(self, arm="left", episode_id=None, owner="owner-1"):
        return self.store.create(owner, episode_id=episode_id or "episode-"+arm, arm=arm,
                                 object_id="strip" if arm == "left" else "plug", epoch="epoch-1")

    def candidate(self, arm="left"):
        state = self.create(arm)
        ev = candidate_event(state)
        self.now = 104.
        return self.store.append("owner-1", state["identity"]["episode_id"], ev, expected_revision=0)

    def test_create_and_reopen_preserve_frozen_budget_and_identity(self):
        initial = self.create()
        self.now = 103.
        reopened = GraspStore(PairLedger(self.path, "run-1", {}, max_duration_s=100., clock=lambda: self.now))
        self.assertEqual(reopened.read("episode-left"), initial)
        self.assertEqual(self.create(), initial)
        self.assertEqual(initial["deadline_at"], 200.)
        self.assertEqual(self.ledger.status()["steps"], 0)
        self.assertEqual(self.ledger.status()["remaining_s"], 97.)

    def test_append_is_atomic_and_replay_neither_advances_revision_nor_resets_deadline(self):
        state = self.candidate()
        ev = retention_event(state)
        self.now = 108.
        retained = self.store.append("owner-1", "episode-left", ev, expected_revision=1)
        self.now = 120.
        replay = self.store.append("owner-1", "episode-left", ev, expected_revision=1)
        self.assertEqual(retained, replay)
        self.assertEqual(replay["revision"], 2)
        self.assertEqual(replay["deadline_at"], 200.)
        self.assertEqual(self.ledger.status()["steps"], 0)

    def test_new_event_with_stale_revision_cannot_overwrite_retained_state(self):
        state = self.candidate()
        self.now = 108.
        retained = self.store.append("owner-1", "episode-left", retention_event(state), expected_revision=1)
        self.now = 112.
        ev = retention_event(retained, event_id="renew-2", now=112., frame=3, renew=True)
        with self.assertRaisesRegex(PairLedgerError, "Stale grasp revision"):
            self.store.append("owner-1", "episode-left", ev, expected_revision=1)
        self.assertEqual(self.store.read("episode-left"), retained)

    def test_same_event_with_changed_payload_is_not_silent_replay(self):
        state = self.candidate()
        original = self.store.read("episode-left")
        ev = candidate_event(state)
        ev["evidence"]["probe"]["requested_width_m"] = .034
        with self.assertRaises(GraspEpisodeError):
            self.store.append("owner-1", "episode-left", ev, expected_revision=0)
        self.assertEqual(self.store.read("episode-left"), original)

    def test_both_arms_have_independent_records(self):
        left, right = self.create("left"), self.create("right")
        self.now = 104.
        changed = self.store.append("owner-1", "episode-left", candidate_event(left), expected_revision=0)
        self.assertEqual(changed["status"], "contact_candidate")
        self.assertEqual(self.store.read("episode-right"), right)
        self.assertEqual(len(self.store.read()), 2)

    def test_pending_send_blocks_changes_without_losing_pending_or_episode(self):
        state = self.create()
        self.ledger.begin("owner-1", "probe-left", {"kind": "gripper"})
        self.now = 104.
        with self.assertRaisesRegex(PairLedgerError, "pending physical"):
            self.store.append("owner-1", "episode-left", candidate_event(state), expected_revision=0)
        self.assertEqual(self.store.read("episode-left"), state)
        self.assertEqual(self.ledger.status()["pending_event_id"], "probe-left")

    def test_fault_keeps_evidence_readable_but_forbids_all_mutations(self):
        state = self.candidate()
        self.ledger.fault("owner-1", "feedback lost")
        self.now = 108.
        with self.assertRaises(PairLedgerFault):
            self.store.append("owner-1", "episode-left", retention_event(state), expected_revision=1)
        self.assertEqual(self.store.read("episode-left"), state)
        self.assertFalse(state["dispatch_authorized"])
        self.assertTrue(self.ledger.peek_status()["fault_latched"])

    def test_unreleased_grasp_prevents_clean_detach_and_reclaim(self):
        state = self.candidate()
        with self.assertRaisesRegex(PairLedgerFault, "Unresolved grasp"):
            self.ledger.release("owner-1")
        with self.assertRaises(PairLedgerFault):
            self.ledger.claim("owner-2")
        self.assertEqual(self.store.read("episode-left"), state)

    def opened(self):
        state = self.candidate()
        self.now = 108.
        rgb = scene(state, at=107., frame=2)
        begin = event(state, "release-begin", "begin_release", {
            "action_event_id": "jaw-open-1", "target_width_m": .042,
            "scene": rgb, "support_visual": release_visual(state, rgb),
            "measurement": measurement(state, now=108., trace_id="release-baseline")})
        pending = self.store.append("owner-1", "episode-left", begin, expected_revision=1)
        self.now = 112.
        done = event(pending, "release-done", "finish_release", {
            "action_event_id": "jaw-open-1",
            "measurement": measurement(pending, now=112., trace_id="release-settled", width=.042),
            "actual_opening_increase_m": .003, "arrival_confirmed": True,
            "target_calls_sent": 1, "passive_arm_commands_sent": 0})
        return self.store.append("owner-1", "episode-left", done, expected_revision=2)

    def test_mechanical_opening_cannot_detach_or_replace_episode(self):
        opened = self.opened()
        self.assertEqual(opened["status"], "release_opened")
        with self.assertRaisesRegex(PairLedgerError, "not been explicitly released"):
            self.create(episode_id="new-left")
        with self.assertRaisesRegex(PairLedgerFault, "Unresolved grasp"):
            self.ledger.release("owner-1")
        self.assertEqual(self.store.read("episode-left"), opened)

    def test_confirmed_separation_can_detach_without_claiming_target_cancelled_or_stopped(self):
        opened = self.opened()
        self.now = 116.
        released = self.store.append("owner-1", "episode-left",
            release_confirmation_event(opened, now=116., frame=3), expected_revision=3)
        self.assertTrue(released["target_may_remain_active"])
        self.assertEqual(released["status"], "released")
        self.assertTrue(is_resolved_release(released))
        self.assertIsNone(self.ledger.release("owner-1")["physical_stop_verified"])
        self.assertEqual(self.ledger.claim("owner-2")["owner"], "owner-2")

    def test_confirmed_release_can_be_followed_by_new_episode_without_budget_reset(self):
        opened = self.opened()
        self.now = 116.
        released = self.store.append("owner-1", "episode-left",
            release_confirmation_event(opened, now=116., frame=3), expected_revision=3)
        newer = self.create(episode_id="new-left")
        self.assertEqual(newer["deadline_at"], released["deadline_at"])
        self.assertEqual(self.ledger.status()["steps"], 0)
        self.assertEqual(self.store.read("episode-left"), released)

    def test_confirmed_left_does_not_resolve_other_arm(self):
        opened = self.opened()
        right = self.create("right")
        # The independent right candidate's observed send precedes this clock;
        # recording historical feedback cannot clear either arm's state.
        self.now = 113.
        right_event = candidate_event(right)
        right_event["evidence"]["probe"].update(sent_at=112.1, completed_at=116.)
        right_event["evidence"]["measurement"] = measurement(right, now=116.)
        self.now = 116.
        right = self.store.append("owner-1", "episode-right", right_event, expected_revision=0)
        self.store.append("owner-1", "episode-left", release_confirmation_event(opened, now=116., frame=3),
                          expected_revision=3)
        self.assertEqual(self.store.read("episode-right"), right)
        with self.assertRaisesRegex(PairLedgerFault, "Unresolved grasp"):
            self.ledger.release("owner-1")

    def test_legacy_mechanical_released_remains_readable_but_cannot_detach(self):
        state = self.opened()
        legacy = copy.deepcopy(state)
        legacy["schema_version"], legacy["status"] = 1, "released"
        del legacy["release_opening"], legacy["release_confirmation"]
        encoded = json.dumps(legacy, sort_keys=True)
        with self.ledger._transaction() as db:
            db.execute("UPDATE pair_grasp_episodes SET state_json=? WHERE episode_id=?", (encoded, "episode-left"))
        self.assertEqual(self.store.read("episode-left"), legacy)
        with self.assertRaisesRegex(PairLedgerError, "not been explicitly released"):
            self.create(episode_id="new-left")
        with self.assertRaisesRegex(PairLedgerFault, "Unresolved grasp"):
            self.ledger.release("owner-1")
        with self.ledger._transaction() as db:
            self.assertEqual(db.execute("SELECT state_json FROM pair_grasp_episodes").fetchone()[0], encoded)

    def test_fault_or_deadline_after_opening_keeps_record_and_blocks_confirmation(self):
        opened = self.opened()
        self.now = 201.
        with self.assertRaises(PairLedgerFault):
            self.store.append("owner-1", "episode-left", release_confirmation_event(opened, now=201., frame=3),
                              expected_revision=3)
        self.assertEqual(self.store.read("episode-left"), opened)
        self.assertEqual(self.ledger.peek_status()["remaining_s"], 0.)

    def test_release_confirmation_replay_does_not_refresh_timestamp_or_revision(self):
        opened = self.opened()
        ev = release_confirmation_event(opened, now=116., frame=3)
        self.now = 116.
        confirmed = self.store.append("owner-1", "episode-left", ev, expected_revision=3)
        self.now = 125.
        replay = self.store.append("owner-1", "episode-left", ev, expected_revision=3)
        self.assertEqual(replay, confirmed)
        self.assertEqual(replay["release_confirmation"]["confirmed_at"], 116.)
        self.assertEqual(replay["deadline_at"], 200.)

    def test_late_or_failed_opening_keeps_physical_receipt_without_mutating_episode(self):
        state = self.candidate()
        self.now = 108.
        rgb = scene(state, at=107., frame=2)
        begin = event(state, "release-begin", "begin_release", {
            "action_event_id": "jaw-open-1", "target_width_m": .042,
            "scene": rgb, "support_visual": release_visual(state, rgb),
            "measurement": measurement(state, now=108., trace_id="baseline")})
        pending = self.store.append("owner-1", "episode-left", begin, expected_revision=1)
        self.ledger.begin("owner-1", "jaw-open-1", {"kind": "gripper", "target": .042})
        self.now = 112.
        receipt = {"action_event_id": "jaw-open-1", "actual_opening_increase_m": .003,
                   "arrival_confirmed": True, "target_calls_sent": 1, "passive_arm_commands_sent": 0,
                   "measurement": measurement(pending, now=112., trace_id="original-opening", width=.042)}
        self.ledger.finish("owner-1", "jaw-open-1", receipt)
        self.now = 201.
        with self.assertRaises(PairLedgerFault):
            self.store.append("owner-1", "episode-left", event(pending, "late-finish", "finish_release", receipt),
                              expected_revision=2)
        self.assertEqual(self.store.read("episode-left"), pending)
        self.assertEqual(self.ledger.event("jaw-open-1")["receipt"], receipt)
        self.assertEqual(self.ledger.peek_status()["steps"], 1)
        self.assertTrue(self.ledger.peek_status()["fault_latched"])

    def test_empty_bookkeeping_does_not_block_clean_owner_handoff(self):
        previous = self.create()
        self.ledger.release("owner-1")
        self.ledger.claim("owner-2")
        with self.assertRaisesRegex(PairLedgerError, "immutable"):
            self.create(owner="owner-2")
        newer = self.create(owner="owner-2", episode_id="new-left")
        self.assertEqual(newer["identity"]["owner"], "owner-2")
        self.assertEqual(self.store.read("episode-left"), previous)

    def test_new_episode_cannot_replace_current_unreleased_grasp(self):
        self.candidate()
        with self.assertRaisesRegex(PairLedgerError, "not been explicitly released"):
            self.create(episode_id="replace-left")
        self.assertEqual(len(self.store.read()), 1)

    def test_expiration_cannot_be_renewed_by_reinitializing_store(self):
        state = self.candidate()
        self.now = 201.
        reopened = GraspStore(self.ledger)
        with self.assertRaises(PairLedgerFault):
            reopened.create("owner-1", episode_id="fresh", arm="right", object_id="plug", epoch="new")
        self.assertEqual(reopened.read("episode-left"), state)
        self.assertEqual(self.ledger.peek_status()["remaining_s"], 0.)


if __name__ == "__main__":
    unittest.main()
