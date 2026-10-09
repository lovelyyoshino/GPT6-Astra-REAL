"""Synthetic FakeCAN residual-opening continuation; no devices or live DB."""
import copy
import unittest
from unittest.mock import patch

from robot_tools import pair_device
from robot_tools.retention_receipt import digest, measured_anchor
from test_single_supervised_actions import SingleActionFixture


class SupportedOpeningContinuationDeviceTests(SingleActionFixture):
    def setUp(self):
        super().setUp()
        self.stack.enter_context(patch.object(pair_device, "time", self.clock))
        for robot in self.robots.values():
            robot.ctrl_mode = 1
            robot.driver_enabled = [True]*6
            robot.gripper_enabled = True
        self.robots["right"].width = .0363
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)))
        self.addCleanup(self.device.close)

    def source(self):
        sample = self.device.open()
        anchor = measured_anchor(sample["arms"]["right"])
        anchor["width_m"] = .032
        # Deliberate old/body versus new/jaw distinction, still within bounds.
        anchor["pose_m_rad"][0] -= .0003
        at = self.clock.time()
        source = {"candidate_measurement": {"anchor": anchor, "observed": {"width_m": .032}},
                  "candidate_probe": {"requested_width_m": .030, "sent_at": at-40,
                                      "completed_at": at-30, "trace_sha256": "a"*64},
                  "before": copy.deepcopy(sample["arms"])}
        source["audited_opening_continuation"] = {
            "schema": "piper_supported_opening_continuation_v1", "arm": "right",
            "source_receipt_sha256": digest(source),
            "prior_opening_event_id": "synthetic-old-opening", "prior_opening_receipt_sha256": "b"*64,
            "prior_opening_target_m": .037, "prior_opening_finished_at": at-20,
            "audit_proposal_sha256": "c"*64,
            "residual_jaw_anchor": {"width_m": .0363, "observed_at": at-1,
                                    "source": {"path": "/synthetic/passive-right.json", "sha256": "d"*64}}}
        return source

    def assert_zero_tx(self):
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(self.robots["right"].sent, [])

    def test_continuation_sends_one_jaw_frame_preserves_body_then_confirms_zero_tx(self):
        source = self.source()
        before = copy.deepcopy(source)
        installed = []
        def inspect(robot, state):
            pending = self.device._action.grasps["right"]
            if pending is not None and not installed:
                installed.append(copy.deepcopy(pending))
        self.hook = inspect
        result = self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(result["hardware_commands_sent"], 1)
        self.assertEqual([frame.arbitration_id for frame in self.robots["right"].sent], [0x159])
        self.assertEqual(self.robots["left"].sent, [])
        self.assertEqual(installed[0]["original_anchor"], source["candidate_measurement"]["anchor"])
        self.assertEqual(installed[0]["audited_opening_continuation"], source["audited_opening_continuation"])
        self.assertNotEqual(installed[0]["original_anchor"]["width_m"],
                            installed[0]["audited_opening_continuation"]["residual_jaw_anchor"]["width_m"])
        self.assertEqual(self.device._joint_cache, {"left": None, "right": None})
        self.assertEqual(self.device.grasp_states, {"left": None, "right": None})
        opening = self.device._supported_recovery_opening
        self.assertEqual(opening["original_anchor"], source["candidate_measurement"]["anchor"])
        for key in ("joints_rad", "pose_m_rad"):
            self.assertEqual(opening["anchor"][key], source["candidate_measurement"]["anchor"][key])
        confirmation = self.device.confirm_supported_recovery_release()
        self.assertTrue(confirmation["ok"])
        self.assertEqual(confirmation["hardware_commands_sent"], 0)
        self.assertGreaterEqual(confirmation["measurement"]["ended_at"]-confirmation["measurement"]["started_at"], 3)
        self.assertEqual(len(self.robots["right"].sent), 1)
        self.assertIsNone(confirmation["physical_stop_verified"])
        self.assertEqual(source, before)

    def test_missing_passive_provenance_is_zero_tx(self):
        source = self.source()
        del source["audited_opening_continuation"]["residual_jaw_anchor"]["source"]["sha256"]
        with self.assertRaisesRegex(ValueError, "provenance"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_changed_original_source_digest_is_zero_tx(self):
        source = self.source()
        source["candidate_measurement"]["anchor"]["pose_m_rad"][0] += .0001
        with self.assertRaisesRegex(ValueError, "receipt changed"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_rehashed_body_mismatch_cannot_be_replaced_by_new_jaw_anchor(self):
        source = self.source()
        source["candidate_measurement"]["anchor"]["pose_m_rad"][0] -= .01
        envelope = source.pop("audited_opening_continuation")
        envelope["source_receipt_sha256"] = digest(source)
        source["audited_opening_continuation"] = envelope
        with self.assertRaisesRegex(RuntimeError, "Current body/jaw differs"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_old_complete_target_replay_is_zero_tx(self):
        source = self.source()
        with self.assertRaisesRegex(RuntimeError, "no replay"):
            self.device.recover_supported_gripper("right", .037, source_receipt=source)
        self.assert_zero_tx()

    def test_larger_float_that_encodes_to_old_target_is_zero_tx(self):
        source = self.source()
        with self.assertRaisesRegex(RuntimeError, "no replay"):
            self.device.recover_supported_gripper("right", .0370004, source_receipt=source)
        self.assert_zero_tx()

    def test_new_target_still_obeys_five_mm_limit(self):
        source = self.source()
        result = self.device.recover_supported_gripper("right", .041301, source_receipt=source)
        self.assertFalse(result["ok"])
        self.assertIn("within 5 mm", result["errors"][-1]["detail"])
        self.assert_zero_tx()

    def test_residual_width_mismatch_does_not_rebase_live_feedback(self):
        source = self.source()
        source["audited_opening_continuation"]["residual_jaw_anchor"]["width_m"] -= .000501
        with self.assertRaisesRegex(RuntimeError, "Current body/jaw differs"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_missing_envelope_retains_legacy_original_jaw_guard(self):
        source = self.source()
        del source["audited_opening_continuation"]
        with self.assertRaisesRegex(RuntimeError, "Current body/jaw differs"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_old_body_anchor_is_checked_again_during_baseline(self):
        source = self.source()
        def drift(robot, state):
            if robot.side == "right" and self.device._action.grasps["right"] is not None:
                state["pose_m_rad"][0] += .0003
        self.hook = drift
        with self.assertRaisesRegex(RuntimeError, "original candidate anchor"):
            self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assert_zero_tx()

    def test_final_fresh_read_still_checks_original_body_after_residual_clears(self):
        source = self.source()
        drifted = []
        def drift(robot, state):
            action = self.device._action
            if (robot.side == "right" and self.robots["right"].sent
                    and action.grasps["right"] is None and not action.active):
                # 0.3 mm from connection anchor, but 0.6 mm from old body.
                state["pose_m_rad"][0] += .0003
                drifted.append(True)
        self.hook = drift
        result = self.device.recover_supported_gripper("right", .040, source_receipt=source)
        self.assertTrue(drifted)
        self.assertFalse(result["ok"])
        self.assertIn("Final opening body changed", result["errors"][-1]["detail"])
        self.assertEqual([frame.arbitration_id for frame in self.robots["right"].sent], [0x159])
        self.assertEqual(self.robots["left"].sent, [])
        self.assertIsNone(self.device._supported_recovery_opening)
        with self.assertRaises(RuntimeError):
            self.device.confirm_supported_recovery_release()
        self.assertEqual(len(self.robots["right"].sent), 1)

    def test_exact_schema_arm_and_chronology_are_required(self):
        source = self.source()
        for mutation in (
            lambda c: c.update(arm="left"),
            lambda c: c.update(audit_proposal_sha256="missing"),
            lambda c: c["residual_jaw_anchor"].update(observed_at=self.clock.time()+1),
            lambda c: c["residual_jaw_anchor"].update(pose_m_rad=[0.]*6),
            lambda c: c["residual_jaw_anchor"]["source"].update(path="relative.json"),
            lambda c: c.update(prior_opening_finished_at=c["residual_jaw_anchor"]["observed_at"]),
        ):
            changed = copy.deepcopy(source)
            mutation(changed["audited_opening_continuation"])
            with self.subTest(changed=changed["audited_opening_continuation"]), self.assertRaises(ValueError):
                pair_device._validated_opening_continuation(changed, "right", .040, self.clock.time())
        self.assert_zero_tx()


if __name__ == "__main__":
    unittest.main()
