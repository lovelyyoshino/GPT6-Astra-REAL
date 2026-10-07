"""Offline readiness-refusal boundaries; never starts a device or driver."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import sys
import unittest
from unittest.mock import patch

import test_guarded_recorded_probe as recorded
import test_ros_interruptible_joint_client as client_fixture

ROOT = Path(__file__).parents[1]
RUN = ROOT / "runs/cola_on_cup_sliding_20261006_215500"
SPEC = importlib.util.spec_from_file_location("readiness_under_test", ROOT / "scripts/ros_guarded_readiness_entry.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
CLIENT = importlib.util.spec_from_file_location("readiness_client_under_test", ROOT / "scripts/ros_guarded_readiness_client.py")
client = importlib.util.module_from_spec(CLIENT)
CLIENT.loader.exec_module(client)


class ReadinessTests(recorded.RecordedTests):
    def setUp(self):
        super().setUp()
        self.control.feedback.close()
        self.session.update(generation=16, regrasp_stage="task", first_segment_verified=True,
                            preflight_refusals=[], probe_attempted=True)
        def register(name, callback):
            self.services[name] = callback
            return name
        self.control = entry.ReadinessTask(self.node,
            types.SimpleNamespace(emit=lambda *a, **k: self.events.append((a, k))),
            recorded.wide.wide_limits(), recorded.probe.fixtures.fk, self.store, self.session, self.path,
            {"offline": True}, lambda: None, self.clock, service_factory=register,
            namespace="/offline_readiness", token="readiness_offline")
        self.addCleanup(self.control.feedback.close)

    def test_actual_healthy_unstable_baseline_refuses_zero_tx_and_only_new_request_can_send(self):
        def unstable(state):
            state["q"][4] += .002 if state["sequence"] % 28 else -.002
        self.piper.on_read = unstable
        with self.assertRaises(entry.stability.BaselineNotReady):
            self.control.execute(self.probe_message())
        self.assertEqual(self.control.status()["phase"], "rejected_preflight")
        self.assertEqual(self.piper.frames, [])
        self.assertIsNone(self.session["pending"])
        self.assertIsNone(self.session["failure"])
        self.assertTrue(self.node.adopted)
        self.assertFalse(self.node.active)
        self.assertEqual(len(self.session["preflight_refusals"]), 1)
        result = json.loads((self.path / "action_000001_preflight_refusal.json").read_text())
        self.assertEqual(result["actuator_frames"], 0)
        self.assertFalse(result["accepted_target_may_continue"])
        self.assertTrue(result["new_explicit_request_required"])
        self.piper.on_read = None
        self.assertEqual(self.piper.frames, [])
        completed = self.control.execute(self.probe_message())
        self.assertEqual(completed["phase"], "completed")
        self.assertEqual(len(self.piper.frames), 4)
        self.assertEqual(len(self.session["preflight_refusals"]), 1)

    def test_same_message_untyped_baseline_error_is_still_a_hard_failure(self):
        with patch.object(self.control, "baseline", side_effect=RuntimeError("Baseline stability not established")):
            with self.assertRaises(RuntimeError):
                self.control.execute(self.probe_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.session["preflight_refusals"], [])
        self.assertEqual(self.piper.frames, [])

    def test_partial_dispatch_preserves_hard_failure_and_no_retry(self):
        self.piper.failure = "partial"
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            self.control.execute(self.probe_message())
        self.assertEqual(len(self.piper.frames), 1)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.session["preflight_refusals"], [])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.probe_message())
        self.assertEqual(len(self.piper.frames), 1)

    def test_typed_error_after_full_send_cannot_be_reclassified_as_zero_tx(self):
        with patch.object(self.control, "wait_arrival", side_effect=entry.stability.BaselineNotReady("late test exception")):
            with self.assertRaises(entry.stability.BaselineNotReady):
                self.control.execute(self.probe_message())
        self.assertEqual(len(self.piper.frames), 4)
        self.assertIsNotNone(self.session["failure"])
        self.assertTrue(self.session["failure"]["accepted_target_may_continue"])
        self.assertEqual(self.session["preflight_refusals"], [])

    def test_existing_raw_fault_wins_over_typed_readiness_error(self):
        def fault():
            raw = list(self.piper.q)
            raw[2] = 1
            self.piper.ParseCANFrame(recorded.probe.rx_fixtures.message(
                *recorded.probe.rx_fixtures.joint_fragment(2, raw), self.clock.time()))
            raise entry.stability.BaselineNotReady("fault must not become a refusal")
        with patch.object(self.control, "baseline", side_effect=fault):
            with self.assertRaises(entry.stability.BaselineNotReady):
                self.control.execute(self.probe_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertIsNotNone(self.session["raw_feedback_fault"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.session["preflight_refusals"], [])
        self.assertEqual(self.piper.frames, [])

    def test_jaw_typed_zero_tx_refuses_but_untyped_error_still_latches(self):
        request = types.SimpleNamespace(gripper_angle=.066, gripper_effort=.2, gripper_code=1, set_zero=0)
        with patch.object(self.control, "baseline", side_effect=entry.stability.BaselineNotReady("Baseline stability not established")):
            with self.assertRaises(entry.stability.BaselineNotReady):
                self.control.gripper(request)
        self.assertEqual(self.control.status()["phase"], "rejected_preflight")
        self.assertIsNone(self.session["failure"])
        self.assertIsNone(self.session["pending"])
        self.assertTrue(self.node.adopted)
        self.assertEqual(self.piper.frames, [])
        result = self.control.gripper(request)
        self.assertEqual(result["phase"], "completed")
        self.assertEqual(len(self.piper.frames), 1)
        with patch.object(self.control, "baseline", side_effect=RuntimeError("Baseline stability not established")):
            with self.assertRaises(RuntimeError):
                self.control.gripper(request)
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(len(self.piper.frames), 1)

    def test_refusal_journal_error_falls_back_to_hard_failure(self):
        record = self.control.record
        def disk_error(event, **fields):
            if event == "rejected_preflight":
                raise OSError("synthetic refusal journal failure")
            return record(event, **fields)
        with patch.object(self.control, "record", side_effect=disk_error), patch.object(
                self.control, "baseline", side_effect=entry.stability.BaselineNotReady("not ready")):
            with self.assertRaisesRegex(OSError, "journal failure"):
                self.control.execute(self.probe_message())
        self.assertIsNotNone(self.session["failure"])
        self.assertFalse(self.node.adopted)
        self.assertEqual(self.piper.frames, [])
        with self.assertRaises(RuntimeError):
            self.control.execute(self.probe_message())


class ClientTests(recorded.wide.OfflineTests):
    def test_explicit_jaw_request_after_refusal_calls_service_once(self):
        before = dict(stage="task", regrasp_stage="task", adoption_token="a"*32,
            generation=16, sequence=8, phase="rejected_preflight", active=False,
            failure=None, stop_latched=False)
        after = dict(before, sequence=9, phase="completed", result={"kind": "gripper"})
        transport = client.ReadinessTransport.__new__(client.ReadinessTransport)
        transport.status = iter([before, before, before, after]).__next__
        transport.observe = lambda: None
        transport.master = types.SimpleNamespace(getSystemState=lambda: (
            [], [], [("/piper/right/gripper_srv", [client.frozen.NODE])]))
        calls = []
        def service(*args):
            calls.append(args)
            return types.SimpleNamespace(status=True, code=15900)
        transport.rospy = types.SimpleNamespace(wait_for_service=lambda *a, **k: None,
                                                ServiceProxy=lambda *a: service)
        modules = {"piper_msgs": types.ModuleType("piper_msgs"),
                   "piper_msgs.srv": types.SimpleNamespace(Gripper=object)}
        with patch.dict(sys.modules, modules):
            result = transport.gripper(66.)
        self.assertTrue(result["feedback_stable"])
        self.assertFalse(result["grasp_verified"])
        self.assertEqual(calls, [(.066, .2, 1, 0)])

    def test_refusal_is_terminal_no_hold_no_retry_and_requires_zero_tx_proof(self):
        for corrupt in (False, True):
            clock = client_fixture.Clock()
            io = client_fixture.IO(clock)
            def status():
                if not io.published:
                    return copy.deepcopy(io.state)
                return dict(io.state, sequence=1, phase="rejected_preflight", active=False,
                    hold_service=client.frozen.NODE+"/hold_current/seq_1_"+io.token,
                    result=dict(phase="rejected_preflight", actuator_frames=1 if corrupt else 0,
                        actuator_transaction_started=False, error="Baseline stability not established"))
            io.status = status
            with self.subTest(corrupt=corrupt):
                if corrupt:
                    with self.assertRaisesRegex(RuntimeError, "proof missing"):
                        client.run_goal(io, [0, 1, -1, 0, 0, 0], clock=clock)
                else:
                    result = client.run_goal(io, [0, 1, -1, 0, 0, 0], clock=clock)
                    self.assertTrue(result["rejected_preflight"])
                    self.assertEqual(result["actuator_frames"], 0)
                    self.assertFalse(result["execution_started"])
                    self.assertFalse(result["accepted_target_may_continue"])
                    self.assertTrue(result["new_explicit_request_required"])
                self.assertEqual(len(io.published), 1)
                self.assertEqual(io.held, [])

    def test_jaw_service_error_requires_same_action_zero_tx_proof_without_resend(self):
        before = dict(adoption_token="a"*32, generation=16, sequence=8)
        after = dict(before, sequence=9, phase="rejected_preflight", kind="gripper",
            active=False, failure=None, receipts=[], result=dict(phase="rejected_preflight",
                kind="gripper", actuator_frames=0, actuator_transaction_started=False,
                error="Baseline stability not established"))
        for corrupt in (None, "generation", "kind", "actuator_frames"):
            terminal = copy.deepcopy(after)
            if corrupt == "generation":
                terminal["generation"] += 1
            elif corrupt == "kind":
                terminal["kind"] = terminal["result"]["kind"] = "joint"
            elif corrupt:
                terminal["result"][corrupt] = 1
            transport = client.ReadinessTransport.__new__(client.ReadinessTransport)
            transport.status = iter([before, terminal]).__next__
            with self.subTest(corrupt=corrupt), patch.object(
                    client.wide_client.WideTransport, "gripper", side_effect=RuntimeError("service failed")) as service:
                if corrupt:
                    with self.assertRaisesRegex(RuntimeError, "service failed"):
                        transport.gripper(66.)
                else:
                    result = transport.gripper(66.)
                    self.assertTrue(result["rejected_preflight"])
                    self.assertEqual(result["actuator_frames"], 0)
                    self.assertFalse(result["grasp_verified"])
                service.assert_called_once_with(66.)


class HandoffTests(recorded.wide.OfflineTests):
    def test_real_qualification_and_eleven_parent_bytes_preserved_without_replay_or_restart(self):
        reviewed = json.loads((RUN / "reviewed_readiness.json").read_text())
        boot = entry.guard.predecessor.PARENT_BOOT
        session_root = entry.guard.v1.SESSION_ROOT
        parent = json.loads((session_root / entry.predecessor.child_name(boot)).read_text())
        self.assertEqual(entry.reviewed_evidence(reviewed, parent)["raw_q"], parent["status"]["latest_state"]["raw_q"])
        changed = copy.deepcopy(parent)
        changed["raw_feedback_fault"] = {"reason": "synthetic fault"}
        with self.assertRaises(RuntimeError):
            entry.reviewed_evidence(reviewed, changed)
        previous = recorded.wide.entry.previous
        interior = previous.motion.predecessor
        names = [boot+".json", entry.guard.predecessor.CHILD_NAME, "guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME, interior.CHILD_NAME, previous.motion.frozen.child_name(boot),
            previous.motion.child_name(boot), previous.release.child_name(boot),
            recorded.wide.entry.child_name(boot), recorded.relief.child_name(boot), entry.predecessor.child_name(boot)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals = {name: (session_root / name).read_bytes() for name in names}
            for name, raw in originals.items():
                (root / name).write_bytes(raw)
            with entry.reserve(boot, reviewed, session_root=root) as (_, endpoint, store, session):
                self.assertEqual(session["regrasp_stage"], "task")
                self.assertTrue(session["first_segment_verified"])
                self.assertEqual(session["probe_reviews"], parent["probe_reviews"])
                self.assertEqual(session["prior_preflight_refusal"], parent["failure"])
                self.assertEqual(session["preflight_refusals"], [])
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                with entry.reserve(boot, reviewed, session_root=root):
                    pass
            self.assertEqual({name: (root/name).read_bytes() for name in names}, originals)


for _name in dir(recorded.RecordedTests):
    if _name.startswith("test_") and _name not in ReadinessTests.__dict__:
        setattr(ReadinessTests, _name, None)


if __name__ == "__main__":
    unittest.main()
