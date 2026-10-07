"""Offline recorded-feedback replay and unchanged stability predicates."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import sliding_stability as sliding


def sample(index):
    return dict(sequence=index+1, stamps=[100.+index*.01]*14,
                q=[0.]*6, pose=[0.]*6, opening_m=0.)


class SlidingTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device or process access"))
            blocker.start()
            self.addCleanup(blocker.stop)

    def test_real195_trace_old_misses_valid_contiguous_windows_new_preserves_them(self):
        path = ROOT / "runs/cola_on_cup_probe_recorded_20261006_212800/runtime/feedback.jsonl"
        rows = []
        with path.open() as stream:
            for line in stream:
                row = json.loads(line)
                if (row.get("event") == "feedback" and row.get("stage") == "baseline"
                        and 1791294383.45 <= row["unix_s"] <= 1791294393.43):
                    rows.append(row)
        self.assertEqual(len(rows), 195)
        old, new = sliding.FrozenStableWindow(), sliding.StableWindow()
        old_passes = new_passes = 0
        original_bytes = json.dumps(rows, sort_keys=True)
        for index, row in enumerate(rows):
            old_passes += old.add(row["state"], row["monotonic_s"])
            accepted = new.add(row["state"], row["monotonic_s"])
            new_passes += accepted
            retained = list(new.samples)
            expected = rows[index-len(retained)+1:index+1]
            self.assertEqual([state["sequence"] for _, state in retained],
                             [item["state"]["sequence"] for item in expected])
            if accepted:
                # The frozen predicate itself must accept each claimed suffix.
                verifier = sliding.FrozenStableWindow()
                for timestamp, state in retained:
                    valid = verifier.add(state, timestamp)
                self.assertTrue(valid)
                self.assertEqual(len(verifier.samples), len(retained))
        self.assertEqual(old_passes, 0)
        self.assertEqual(new_passes, 57)
        self.assertEqual(json.dumps(rows, sort_keys=True), original_bytes)

    def test_internal_bad_point_cannot_be_skipped_or_bridge_earlier_good_samples(self):
        window = sliding.StableWindow()
        first_pass = None
        for index in range(101):
            state = sample(index)
            if index == 30:
                state["q"][2] = .004
            accepted = window.add(state, index*.05)
            retained = [s["sequence"] for _, s in window.samples]
            self.assertEqual(retained, list(range(retained[0], index+2)))
            if index > 30:
                self.assertGreaterEqual(retained[0], 32)
            if accepted and first_pass is None:
                first_pass = index*.05
        self.assertIsNotNone(first_pass)
        self.assertGreaterEqual(first_pass, 31*.05+3.)

    def test_at_least20_groups_and_full_three_seconds_are_both_required(self):
        count = sliding.StableWindow()
        for index in range(19):
            self.assertFalse(count.add(sample(index), index/6.))
        self.assertTrue(count.add(sample(19), 3.001))
        duration = sliding.StableWindow()
        for index in range(20):
            self.assertFalse(duration.add(sample(index), index*.1))
        self.assertFalse(duration.add(sample(20), 2.999))
        self.assertTrue(duration.add(sample(21), 3.))

    def test_sequence_all14_timestamps_and_monotonic_must_advance(self):
        for fault in list(range(14)) + ["sequence", "monotonic"]:
            window = sliding.StableWindow()
            first, second = sample(0), sample(1)
            window.add(first, 1.)
            timestamp = 1.1
            if isinstance(fault, int):
                second["stamps"][fault] = first["stamps"][fault]
            elif fault == "sequence":
                second["sequence"] = first["sequence"]
            else:
                timestamp = .9
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                window.add(second, timestamp)
            self.assertEqual(len(window.samples), 1)

    def test_joint_xyz_jaw_and_so3_caps_match_frozen_predicate_without_relaxation(self):
        cases = []
        for axis in range(6):
            for value, valid in ((.003, True), (.00300001, False)):
                state = sample(1)
                state["q"][axis] = value
                cases.append(("q%d_%r" % (axis, value), state, valid))
        for key, value, valid in (("opening_m", .0005, True), ("opening_m", .00050001, False)):
            state = sample(1)
            state[key] = value
            cases.append((key+str(value), state, valid))
        for xyz, valid in (([.0003, .0004, 0.], True), ([.00030001, .0004, 0.], False)):
            state = sample(1)
            state["pose"][:3] = xyz
            cases.append(("xyz"+str(xyz), state, valid))
        for angle, valid in ((.002999, True), (.003001, False)):
            state = sample(1)
            state["pose"][3] = angle
            cases.append(("so3"+str(angle), state, valid))
        for name, state, valid in cases:
            with self.subTest(case=name):
                old, new = sliding.FrozenStableWindow(), sliding.StableWindow()
                old.add(sample(0), 0.)
                new.add(sample(0), 0.)
                old.add(state, .1)
                new.add(state, .1)
                self.assertEqual(len(new.samples), 2 if valid else 1)
                self.assertEqual(list(new.samples), list(old.samples))


if __name__ == "__main__":
    unittest.main()
