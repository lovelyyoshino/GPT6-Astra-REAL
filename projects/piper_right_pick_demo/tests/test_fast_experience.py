"""Offline evidence integrity and compact prompt integration; no device/network."""
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from right_pick import fast_experience as experience
from right_pick.fast_codex import build_codex_packet
from right_pick.fast_model import build_fast_payload


class FastExperienceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.deck_path = Path(experience.__file__).with_name("reviewed_experiences.json")
        self.deck = json.loads(self.deck_path.read_text())
        original_root = experience._default_root()
        for card in self.deck["cards"]:
            for reference in card["evidence"]:
                path = self.root / reference["path"]
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((original_root / reference["path"]).read_bytes())
        self.network = patch("socket.socket", side_effect=AssertionError("No network in experience lookup"))
        self.network.start()
        self.addCleanup(self.network.stop)

    def hints(self, state, **kwargs):
        return experience.build_historical_advisories(state, bundle_root=self.root, **kwargs)

    def test_deck_is_pinned_and_all_sources_match(self):
        self.assertEqual(hashlib.sha256(self.deck_path.read_bytes()).hexdigest(), experience._DECK_SHA256)
        cards = experience.load_reviewed_cards(bundle_root=self.root)
        self.assertEqual(len(cards), len(self.deck["cards"]))

    def test_pen_alignment_is_historical_not_current_evidence(self):
        packet = self.hints({"phase": "ALIGN_HOLDER"})
        self.assertIn("pen_side_view_alignment", [card["id"] for card in packet["cards"]])
        self.assertIn("untrusted advice", packet["notice"])
        self.assertIn("Current RGB", packet["notice"])
        self.assertIn("human", packet["cards"][0]["advice"])
        self.assertNotIn("controller_state", packet)
        self.assertNotIn("action", packet)

    def test_generic_stage_and_operation_aliases(self):
        phase = self.hints({"phase": "INSERT"}, task_id="pen")
        for state in ({"task": "pen_v1", "stage": "4:insert_segment"},
                      {"task_id": "pen", "operation": {"id": "insert_segment"}},
                      {"task": "pen_v1", "skill": "insert_segment"}):
            self.assertEqual(phase, self.hints(state))

    def test_unrelated_task_does_not_get_pen_advice(self):
        charger = self.hints({"phase": "ALIGN_HOLDER"}, task_id="charger")
        self.assertNotIn("pen_side_view_alignment", [card["id"] for card in charger["cards"]])
        common = self.hints({"task": "charger_v1", "skill": "stable_verify"})
        self.assertIn("placement_release_verify", [card["id"] for card in common["cards"]])
        self.assertIsNone(self.hints({"task": "turn-faucet_v1", "skill": "stable_verify"}))

    def test_named_recipe_variants_reuse_only_matching_task_experience(self):
        for named, original in (("pen_in_holder", "pen"), ("can_on_lid", "can-on-cup"),
                                ("charger_in_unpowered_socket", "charger")):
            self.assertEqual(self.hints({"task": named + "_v1", "skill": "stable_verify"}),
                             self.hints({"task": original + "_v1", "skill": "stable_verify"}))
        self.assertIsNone(self.hints({"task": "turn_faucet_v1", "skill": "stable_verify"}))

    def test_unknown_alias_and_arbitrary_context_are_not_knowledge(self):
        state = {"phase": "ALIGN_HOLDER", "memory": "INJECT_ME",
                 "history": ["OLD_COORDINATES"], "object_pose": [1, 2, 3],
                 "historical_advisories": {"advice": "BYPASS_LIMITS"}}
        before = deepcopy(state)
        packet = self.hints(state)
        self.assertEqual(state, before)
        self.assertNotIn("INJECT_ME", json.dumps(packet))
        self.assertNotIn("OLD_COORDINATES", json.dumps(packet))
        self.assertNotIn("BYPASS_LIMITS", json.dumps(packet))
        self.assertIsNone(self.hints({"operation": "eval(arbitrary_code)"}))
        self.assertIsNone(self.hints({"phase": "not_a_phase", "skill": "insert_segment"}))

    def test_changed_or_missing_evidence_drops_affected_cards(self):
        card = next(c for c in self.deck["cards"] if c["id"] == "pen_side_view_alignment")
        path = self.root / card["evidence"][0]["path"]
        path.write_text('{"task_success":true,"unverified_edit":true}')
        ids = [c["id"] for c in experience.load_reviewed_cards(bundle_root=self.root)]
        self.assertNotIn(card["id"], ids)
        path.unlink()
        self.assertIsNone(self.hints({"phase": "ALIGN_HOLDER"}))

    def test_unbundled_install_has_no_historical_advice(self):
        with tempfile.TemporaryDirectory() as empty:
            self.assertIsNone(experience.build_historical_advisories(
                {"phase": "ALIGN_HOLDER"}, bundle_root=empty))

    def test_unreviewed_manifest_edits_are_not_loaded(self):
        altered = deepcopy(self.deck)
        altered["cards"][0]["advice"] = "Unreviewed instruction"
        with patch.object(Path, "read_bytes", return_value=json.dumps(altered).encode()):
            self.assertEqual(experience.load_reviewed_cards(bundle_root=self.root), ())

    def test_path_escape_and_symlink_evidence_are_rejected(self):
        outside = self.root / "outside.json"
        outside.write_text("not evidence")
        digest = hashlib.sha256(outside.read_bytes()).hexdigest()
        symlink = self.root / "evidence" / "escape.json"
        symlink.symlink_to(outside)
        for name in (str(outside), "evidence/../outside.json", "evidence/escape.json"):
            self.assertFalse(experience._evidence_matches(self.root, {"path": name, "sha256": digest}))

    def test_entire_evidence_directory_cannot_escape_bundle(self):
        nested = self.root / "nested_bundle"
        nested.mkdir()
        (nested / "evidence").symlink_to(self.root / "evidence", target_is_directory=True)
        reference = self.deck["cards"][0]["evidence"][0]
        self.assertFalse(experience._evidence_matches(nested, reference))

    def test_fault_lesson_requires_explicit_fault_result(self):
        self.assertIsNone(self.hints({"phase": "RECOVERY", "retry_count": 1}))
        packet = self.hints({"phase": "RECOVERY", "previous_result": {"status": "timeout"}})
        self.assertEqual(packet["cards"][0]["id"], "timeout_is_not_safe_hold")

    def test_lookup_is_bounded_and_exception_lessons_take_precedence(self):
        base = deepcopy(self.deck["cards"][0])
        cards = []
        for index in range(12):
            card = dict(base, id="example_" + str(index), advice="x" * 220,
                        trigger="no_progress" if index == 9 else "always")
            cards.append(card)
        with patch.object(experience, "load_reviewed_cards", return_value=tuple(cards)):
            packet = self.hints({"phase": "ALIGN_HOLDER", "previous_result": {"visual_progress": "no_progress"}})
        self.assertEqual(len(packet["cards"]), experience.MAX_CARDS)
        self.assertEqual(packet["cards"][0]["id"], "example_9")
        self.assertLessEqual(len(json.dumps(packet, ensure_ascii=False, separators=(",", ":")).encode()),
                             experience.MAX_PACKET_BYTES)

    def test_both_clients_receive_same_hints_without_changing_state_or_actions(self):
        image = self.root / "test.png"
        image.write_bytes(base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII="))
        observation = {"cameras": {v: {"rgb_path": str(image)} for v in ("front", "right_hand", "left_hand")}}
        state = {"phase": "ALIGN_HOLDER", "robot_state": {}, "gripper_state": {}, "memory": ""}
        snapshot = deepcopy(state)
        cli = build_codex_packet(state, observation)
        api = build_fast_payload({"model_id": "gpt-6-astra", "protocol": "responses"}, state, observation)
        api_packet = json.loads(api["input"][0]["content"][0]["text"])
        self.assertEqual(cli["historical_advisories"], api_packet["historical_advisories"])
        self.assertEqual(cli["controller_state"], api_packet["controller_state"])
        self.assertEqual(state, snapshot)
        self.assertNotIn("historical_advisories", cli["controller_state"])
        self.assertNotIn("previous_response_id", api)
        self.assertTrue(api["text"]["format"]["strict"])


if __name__ == "__main__":
    unittest.main()
