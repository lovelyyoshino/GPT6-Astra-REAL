import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from right_pick.recording import Recorder, normalize_usage


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_missing_usage_stays_unknown_and_subsets_are_not_added(self):
        recorder = Recorder(self.tmp.name, {}, "test")
        recorder.model_call({"input_tokens": 100, "output_tokens": 30,
                             "input_tokens_details": {"cached_tokens": 80},
                             "output_tokens_details": {"reasoning_tokens": 20}}, elapsed_s=1)
        recorder.model_call(None, elapsed_s=2, error="network timeout")
        usage = recorder.usage_summary()
        self.assertIsNone(usage["input_tokens"]["total"])
        self.assertEqual(usage["input_tokens"]["known_subtotal"], 100)
        self.assertEqual(usage["input_tokens"]["coverage"], 0.5)
        self.assertEqual(usage["reasoning_output_tokens"]["known_subtotal"], 20)
        self.assertEqual(usage["calls"], 2)
        self.assertEqual(usage["interface_wait_time_s"], 3)

    def test_provider_usage_aliases_and_invalid_counts(self):
        usage = normalize_usage({"prompt_tokens": 10, "completion_tokens": 3,
                                 "prompt_tokens_details": {"cached_tokens": 4}})
        self.assertEqual(usage["input_tokens"], 10)
        self.assertEqual(usage["output_tokens"], 3)
        self.assertEqual(usage["cached_input_tokens"], 4)
        self.assertIsNone(normalize_usage({"input_tokens": False})["input_tokens"])

    def test_secret_filter_and_no_detailed_thought(self):
        recorder = Recorder(self.tmp.name, {"api_key": "sk-private-example-credential"},
                            "test sk-private-example-credential")
        recorder.event("model_response", {"reasoning_content": "private long thought",
                                          "text": "Bearer other-secret sk-private-example-credential",
                                          "token": "third-secret", "input_tokens": 5})
        recorder.finish(outcome="blocked", reason="infrastructure_blocked", physical_attempts=0)
        text = "\n".join(p.read_text() for p in recorder.run_dir.iterdir())
        for secret in ("sk-private-example-credential", "other-secret", "third-secret", "private long thought"):
            self.assertNotIn(secret, text)
        self.assertIn('"input_tokens": 5', text)

    def test_stop_and_arrival_never_imply_success(self):
        recorder = Recorder(self.tmp.name, {}, "test")
        recorder.event("feedback", {"status": "arrived"})
        result = recorder.finish(termination_reason="model_stop")
        self.assertEqual(result["outcome"], "not_evaluated")
        self.assertIsNone(result["success"])

    def test_infrastructure_block_is_not_a_grasp_failure(self):
        recorder = Recorder(self.tmp.name, {}, "test")
        result = recorder.finish(outcome="blocked", reason="infrastructure_blocked", physical_attempts=0)
        self.assertEqual(result["physical_attempts"], 0)
        self.assertIsNone(result["success"])
        self.assertIsNone(result["failure"])

    def evidence(self, start):
        return [{"stage": stage, "confirmed": True, "source": "manual_review",
                 "reference": "frame_%d.png" % index, "at": start + index + 1}
                for index, stage in enumerate(("lift", "transport", "release", "stable"))]

    def test_all_four_ordered_physical_evidence_stages_are_required(self):
        recorder = Recorder(self.tmp.name, {}, "test")
        with patch("right_pick.recording.time.time", return_value=recorder.started_at + 10):
            result = recorder.finish(evidence=self.evidence(recorder.started_at))
        self.assertTrue(result["success"])
        for missing in range(4):
            recorder = Recorder(self.tmp.name, {}, "test")
            evidence = self.evidence(recorder.started_at)
            del evidence[missing]
            with patch("right_pick.recording.time.time", return_value=recorder.started_at + 10):
                result = recorder.finish(evidence=evidence)
            self.assertIsNone(result["success"])

    def test_replay_or_model_claim_cannot_be_physical_success(self):
        for mode, source in (("nonphysical_replay", "manual_review"), ("physical", "model_claim")):
            recorder = Recorder(self.tmp.name, {}, "test", mode=mode)
            evidence = self.evidence(recorder.started_at)
            for row in evidence:
                row["source"] = source
            with patch("right_pick.recording.time.time", return_value=recorder.started_at + 10):
                result = recorder.finish(evidence=evidence)
            self.assertIsNone(result["success"])

    def test_out_of_order_and_pre_run_evidence_does_not_succeed(self):
        recorder = Recorder(self.tmp.name, {}, "test")
        evidence = self.evidence(recorder.started_at)
        evidence[0]["at"] = evidence[3]["at"]
        with patch("right_pick.recording.time.time", return_value=recorder.started_at + 10):
            result = recorder.finish(evidence=evidence)
        self.assertIsNone(result["success"])

    def test_independent_run_directories_and_no_post_finish_mutation(self):
        one = Recorder(self.tmp.name, {}, "one")
        two = Recorder(self.tmp.name, {}, "two")
        self.assertNotEqual(one.run_dir, two.run_dir)
        one.finish()
        self.assertTrue((one.run_dir / "report.json").is_file())
        with self.assertRaises(RuntimeError):
            one.event("late_write")


if __name__ == "__main__":
    unittest.main()
