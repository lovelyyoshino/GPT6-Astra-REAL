"""Safety-boundary tests; real vendor encoding, no socket or robot connection."""
import importlib.util
import io
import json
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest import mock

PATH = Path(__file__).resolve().parents[1] / "scripts" / "direct_sdk_step.py"
SPEC = importlib.util.spec_from_file_location("direct_sdk_step_test_module", PATH)
step = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(step)


class SDKStepTests(unittest.TestCase):
    def setUp(self):
        self.vendor = step.Vendor()
        self.state = step.Feedback(self.vendor)
        self.populate()

    def populate(self, enabled=True, z=213000, now=10.):
        frames = {0x2A1: bytes([1, 0, 1, 0, 0, 0, 0, 0]),
                  0x2A2: struct.pack(">ii", 56000, 0),
                  0x2A3: struct.pack(">ii", z, 0),
                  0x2A4: struct.pack(">ii", 85000, 0),
                  0x2A5: struct.pack(">ii", 0, 0),
                  0x2A6: struct.pack(">ii", 0, 0),
                  0x2A7: struct.pack(">ii", 0, 0)}
        frames.update({identifier: struct.pack(">HhbBH", 240, 30, 30, 64 if enabled else 0, 0)
                       for identifier in range(0x261, 0x267)})
        for identifier, payload in frames.items():
            self.state.ingest(identifier, payload, False, now)

    def test_actual_vendor_decode_and_separate_frame_freshness(self):
        snap = self.state.snapshot(10.01, True)
        self.assertEqual(snap["pose_raw"], [56000, 0, 213000, 0, 85000, 0])
        self.state.received[0x2A2] = 9.5
        with self.assertRaisesRegex(step.Rejected, "0x2A2"):
            self.state.snapshot(10.01)

    def test_missing_one_motor_cannot_look_enabled(self):
        del self.state.received[0x264]
        with self.assertRaisesRegex(step.Rejected, "0x264"):
            self.state.snapshot(10.01, True)

    def test_foreign_control_and_any_local_frame_latch(self):
        for local, identifier in ((False, 0x151), (True, 0x2A1)):
            state = step.Feedback(self.vendor)
            with self.assertRaises(step.Rejected):
                state.ingest(identifier, bytes(8), local, 10.)
            self.assertIsNotNone(state.fault)

    def test_expected_own_echo_consumed_only_once(self):
        key = (0x471, bytes(8))
        self.state.own_echo_budget[key] = 1
        self.state.ingest(*key, True, 10.)
        with self.assertRaises(step.Rejected):
            self.state.ingest(*key, True, 10.)

    def test_teach_and_mit_and_transient_fault_do_not_clear(self):
        for status in (bytes([2, 0, 1, 0, 0, 0, 0, 0]),
                       bytes([1, 0, 4, 0, 0, 0, 0, 0]),
                       bytes([1, 2, 2, 0, 0, 0, 0, 0])):
            state = step.Feedback(self.vendor)
            with self.assertRaises(step.Rejected):
                state.ingest(0x2A1, status, False, 10.)
            state.ingest(0x2A1, bytes([1, 0, 1, 0, 0, 0, 0, 0]), False, 10.01)
            with self.assertRaises(step.Rejected):
                state.snapshot(10.02)

    def test_motor_fault_latches_at_frame_not_at_snapshot(self):
        with self.assertRaisesRegex(step.Rejected, "Motor 2 fault"):
            self.state.ingest(0x262, struct.pack(">HhbBH", 240, 30, 30, 64 | 4, 0), False, 10.)
        self.assertIsNotNone(self.state.fault)

    def test_exact_sdk_encoded_target_units_and_order(self):
        pose = [56000, 0, 228000, 0, 85000, 0]
        plan = step.encoded_plan(self.vendor.sdk, pose)
        self.assertEqual([i for i, _ in plan["motion"]], [0x151, 0x152, 0x153, 0x154])
        self.assertEqual(plan["motion"][0][1][:4], bytes([1, 2, 5, 0]))
        self.assertEqual(struct.unpack(">ii", plan["motion"][2][1]), (228000, 0))
        self.assertEqual([i for i, _ in plan["enable"]], [0x471])

    def guard(self):
        plan = step.encoded_plan(self.vendor.sdk, [56000, 0, 228000, 0, 85000, 0])
        sender = mock.Mock(return_value=step.RecordingPort.CAN_STATUS.SEND_MESSAGE_SUCCESS)
        transport = types.SimpleNamespace(SendCanMessage=sender, CAN_STATUS=step.RecordingPort.CAN_STATUS)
        receiver = types.SimpleNamespace(drain=lambda: None)
        report = {"transmissions": []}
        return step.TransmitGuard(transport, plan, self.state, receiver, report), sender, report

    def test_stale_feedback_rejection_precedes_actual_send(self):
        guard, sender, report = self.guard()
        guard.allow("enable")
        with mock.patch.object(step.time, "monotonic", return_value=11.):
            with self.assertRaises(step.Rejected):
                guard.send(*guard.plan["enable"][0])
        sender.assert_not_called()
        self.assertEqual(report["transmissions"], [])

    def test_modified_payload_or_extra_command_never_sent(self):
        guard, sender, report = self.guard()
        guard.allow("motion")
        with mock.patch.object(step.time, "monotonic", return_value=10.01):
            with self.assertRaises(step.Rejected):
                guard.send(0x151, bytes([1, 2, 100, 0, 0, 0, 0, 0]))
        sender.assert_not_called()
        with self.assertRaises(step.Rejected):
            guard.allow("motion")

    def test_exact_plan_sends_four_frames_and_cannot_repeat(self):
        guard, sender, report = self.guard()
        guard.require_enabled = True
        guard.allow("motion")
        with mock.patch.object(step.time, "monotonic", return_value=10.01):
            for frame in guard.plan["motion"]:
                guard.send(*frame)
        self.assertEqual(sender.call_count, 4)
        self.assertEqual(len(report["transmissions"]), 4)
        with self.assertRaises(step.Rejected):
            guard.allow("motion")

    def test_arrival_requires_new_feedback_actual_pose_and_mode(self):
        target = [56000, 0, 228000, 0, 85000, 0]
        sample = self.state.snapshot(10.01, True)
        sample["status"]["mode_feed"] = 2
        self.assertFalse(step.arrived(sample, target, self.state, 9.9))
        sample["pose_raw"] = target
        self.assertTrue(step.arrived(sample, target, self.state, 9.9))
        self.assertFalse(step.arrived(sample, target, self.state, 10.))

    def test_path_deviation_and_stationary_gate(self):
        origin = self.state.snapshot(10.01, True)
        changed = dict(origin, pose_raw=[60000, 0, 213000, 0, 85000, 0])
        with self.assertRaises(step.Rejected):
            step.check_path(changed, origin)
        self.assertFalse(step.stationary([origin]))
        self.assertTrue(step.stationary([origin, dict(origin, monotonic_s=10.4)]))

    def test_motion_deadline_never_counts_as_arrival(self):
        guard, sender, report = self.guard()
        guard.allow("motion")
        guard.deadline = 10.
        with mock.patch.object(step.time, "monotonic", return_value=10.01):
            with self.assertRaisesRegex(step.Rejected, "deadline"):
                guard.send(*guard.plan["motion"][0])
        sender.assert_not_called()

    def run_full_step(self, reaches):
        clock = [10.]
        origin = self.state.snapshot(10., True)
        report = {"preview": origin, "preview_target_raw": [56000, 0, 228000, 0, 85000, 0],
                  "binding": {"ifindex": 8}, "trace": [], "transmissions": [],
                  "arrival_verified": False}
        transport = step.RecordingPort()
        transport.Close = mock.Mock()
        attribute = "_C_PiperInterface_V2__arm_can"
        sdk = self.vendor.sdk
        original = getattr(sdk, attribute)

        def create(*args, **kwargs):
            setattr(sdk, attribute, transport)

        def receive(timeout):
            clock[0] += .01
            moved = reaches and clock[0] >= 10.05
            self.populate(z=228000 if moved else 213000, now=clock[0])
            self.state.ingest(0x2A1, bytes([1, 0, 2, 0, 0, 0, 0, 0]), False, clock[0])

        receiver = types.SimpleNamespace(one=receive, drain=lambda: None)
        try:
            with mock.patch.object(step, "collect_stationary", return_value=origin), \
                    mock.patch.object(step, "inspect_binding", return_value={"ifindex": 8}), \
                    mock.patch.object(step, "inspect_controllers", return_value=[]), \
                    mock.patch.object(step.time, "monotonic", side_effect=lambda: clock[0]), \
                    mock.patch.object(sdk, "CreateCanBus", side_effect=create):
                if reaches:
                    step.execute_step(self.vendor, receiver, self.state, report)
                else:
                    with self.assertRaisesRegex(step.Rejected, "Five-second"):
                        step.execute_step(self.vendor, receiver, self.state, report)
        finally:
            setattr(sdk, attribute, original)
        transport.Close.assert_called_once()
        self.assertEqual([frame[0] for frame in transport.frames], [0x151, 0x152, 0x153, 0x154])
        return report, clock[0]

    def test_entire_step_timeout_stays_failed_and_sends_no_stop_or_retry(self):
        report, finished = self.run_full_step(False)
        self.assertFalse(report["arrival_verified"])
        self.assertLess(finished, 15.02)

    def test_entire_step_passes_only_after_measured_arrival_settles(self):
        report, finished = self.run_full_step(True)
        self.assertTrue(report["arrival_verified"])
        self.assertEqual(report["status"], "arrived")
        self.assertGreater(finished, 10.25)

    def test_rotated_preview_cannot_be_silently_retargeted(self):
        origin = self.state.snapshot(10.01)
        rotated = dict(origin, pose_raw=[56000, 0, 213000, 1000, 85000, 0])
        with self.assertRaises(step.Rejected):
            step.require_same_pose(rotated, origin, "changed")

    def rejected_preview_report(self, stale=False):
        clock = [10.]

        def receive(timeout):
            clock[0] += .01
            stamp = clock[0] - 1 if stale else clock[0]
            self.populate(now=stamp)
            if not stale:
                self.state.ingest(0x2A5, struct.pack(">ii", 0, -10000), False, stamp)

        receiver = types.SimpleNamespace(one=receive, drain=lambda: None, close=lambda: None)
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(step, "Vendor", return_value=self.vendor), \
                mock.patch.object(step, "Feedback", return_value=self.state), \
                mock.patch.object(step, "Receiver", return_value=receiver), \
                mock.patch.object(step, "inspect_binding", return_value={"ifindex": 8}), \
                mock.patch.object(step, "inspect_controllers", return_value=[]), \
                mock.patch.object(step.time, "monotonic", side_effect=lambda: clock[0]), \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO), \
                mock.patch("builtins.input") as user_input:
            result = step.main(["--output-dir", directory])
            report = json.loads((Path(directory) / "report.json").read_text())
            self.assertEqual(result, 2)
            user_input.assert_not_called()
        self.assertEqual(report["frames_send_attempted"], 0)
        self.assertEqual(report["trace"], [])
        return report

    def test_outside_range_report_preserves_values_and_exact_rejection_without_tx(self):
        report = self.rejected_preview_report()
        self.assertIn("Measured joint outside", report["error"])
        evidence = report["latest_unqualified_feedback"]
        self.assertEqual(evidence["joints_raw"]["joint_2"], -10000)
        self.assertEqual(evidence["raw_frame_latest"]["0x2A5"]["payload_hex"],
                         struct.pack(">ii", 0, -10000).hex())
        self.assertGreater(evidence["snapshot_rejection_counts"]["Measured joint outside inspected manufacturer range"], 0)

    def test_old_kernel_timestamp_report_exposes_ages_and_no_tx(self):
        report = self.rejected_preview_report(stale=True)
        self.assertIn("Missing/stale individual feedback frames", report["error"])
        evidence = report["latest_unqualified_feedback"]
        self.assertGreaterEqual(evidence["required_frame_age_s"]["0x2A2"], 1.)
        self.assertGreaterEqual(evidence["raw_frame_latest"]["0x2A2"]["kernel_age_s_at_report"], 1.)
        self.assertEqual(evidence["pose_raw"]["Z_axis"], 213000)

    def populate_reentry(self, joints=None, enabled=False, now=10., ctrl_mode=0):
        joints = list(step.REENTRY_START if joints is None else joints)
        pose = [round(x) for x in self.vendor.fk_pose_raw(joints)]
        self.populate(enabled=enabled, now=now)
        self.state.ingest(0x2A1, bytes([ctrl_mode, 0, 1, 0, 0, 0, 0, 0]), False, now)
        for index, identifier in enumerate(step.POSE_PARTS):
            self.state.ingest(identifier, struct.pack(">ii", *pose[2 * index:2 * index + 2]), False, now)
        for index, identifier in enumerate(step.JOINT_PARTS):
            self.state.ingest(identifier, struct.pack(">ii", *joints[2 * index:2 * index + 2]), False, now)

    def test_nominal_gate_unchanged_review_exception_only_small_interval(self):
        self.populate_reentry()
        with self.assertRaisesRegex(step.Rejected, "manufacturer range"):
            self.state.snapshot(10.01)
        self.state.reviewed_reentry = True
        sample = self.state.snapshot(10.01)
        step.validate_reviewed_start(sample, self.vendor, True)
        q = list(step.REENTRY_START)
        q[0] += 151
        self.populate_reentry(q)
        with self.assertRaisesRegex(step.Rejected, "reviewed reentry interval"):
            self.state.snapshot(10.01)

    def test_reviewed_target_actual_sdk_joint_payloads_and_units(self):
        plan = step.encoded_plan(self.vendor.sdk, [], reviewed_reentry=True)
        self.assertEqual([i for i, _ in plan["motion"]], [0x151, 0x155, 0x156, 0x157])
        self.assertEqual(plan["motion"][0][1][:4], bytes([1, 1, 5, 0]))
        unpacked = [q for _, data in plan["motion"][1:] for q in struct.unpack(">ii", data)]
        self.assertEqual(unpacked, list(step.REENTRY_TARGET))
        self.assertTrue(step.in_nominal_range(unpacked))

    def test_reentry_cannot_start_after_completion_or_from_wrong_fk(self):
        self.state.reviewed_reentry = True
        self.populate_reentry(step.REENTRY_TARGET)
        sample = self.state.snapshot(10.01)
        with self.assertRaisesRegex(step.Rejected, "starting posture"):
            step.validate_reviewed_start(sample, self.vendor, True)
        self.populate_reentry()
        sample = self.state.snapshot(10.01)
        sample["pose_raw"][0] += 500
        with self.assertRaisesRegex(step.Rejected, "FK disagree"):
            step.validate_reviewed_start(sample, self.vendor, True)
        sample["pose_raw"][0] -= 500
        sample["motor_enabled"] = [True] * 6
        with self.assertRaisesRegex(step.Rejected, "standby state"):
            step.validate_reviewed_start(sample, self.vendor, True)
        step.validate_reviewed_start(sample, self.vendor, False)

    def test_reviewed_arrival_requires_actual_nominal_joints_and_freshness(self):
        self.state.reviewed_reentry = True
        self.populate_reentry(step.REENTRY_TARGET, enabled=True, ctrl_mode=1)
        sample = self.state.snapshot(10.01, True)
        goal = list(sample["pose_raw"])
        self.assertTrue(step.arrived(sample, goal, self.state, 9.99))
        self.assertFalse(step.arrived(sample, goal, self.state, 10.))
        sample["joints_raw"][1] = -1
        self.assertFalse(step.arrived(sample, goal, self.state, 9.99))
        sample["joints_raw"][1] = step.REENTRY_TARGET[1] + 101
        self.assertFalse(step.arrived(sample, goal, self.state, 9.99))

    def run_reviewed_full_step(self, reaches=True, invalid_fk=False):
        clock = [10.]
        self.state.reviewed_reentry = True
        self.populate_reentry()
        origin = self.state.snapshot(10.)
        target = step.validate_reviewed_start(origin, self.vendor, True)
        report = {"preview": origin, "preview_target_raw": target,
                  "binding": {"ifindex": 8}, "trace": [], "transmissions": [], "arrival_verified": False}
        if invalid_fk:
            origin["pose_raw"][0] += 500
        transport = step.RecordingPort()
        transport.Close = mock.Mock()
        attribute = "_C_PiperInterface_V2__arm_can"
        sdk = self.vendor.sdk
        original = getattr(sdk, attribute)

        def create(*args, **kwargs):
            setattr(sdk, attribute, transport)

        def receive(timeout):
            clock[0] += .01
            moving = any(i == 0x157 for i, _ in transport.frames)
            reached = reaches and moving and clock[0] >= 10.06
            self.populate_reentry(step.REENTRY_TARGET if reached else step.REENTRY_START,
                                  enabled=True, now=clock[0], ctrl_mode=1 if moving else 0)

        receiver = types.SimpleNamespace(one=receive, drain=lambda: None)
        try:
            with mock.patch.object(step, "collect_stationary", side_effect=lambda *a, **kw:
                                   origin if invalid_fk else self.state.snapshot(clock[0])), \
                    mock.patch.object(step, "inspect_binding", return_value={"ifindex": 8}), \
                    mock.patch.object(step, "inspect_controllers", return_value=[]), \
                    mock.patch.object(step.time, "monotonic", side_effect=lambda: clock[0]), \
                    mock.patch.object(sdk, "CreateCanBus", side_effect=create) as creator:
                if invalid_fk:
                    with self.assertRaisesRegex(step.Rejected, "FK disagree"):
                        step.execute_step(self.vendor, receiver, self.state, report)
                    creator.assert_not_called()
                elif reaches:
                    step.execute_step(self.vendor, receiver, self.state, report)
                else:
                    with self.assertRaisesRegex(step.Rejected, "Five-second"):
                        step.execute_step(self.vendor, receiver, self.state, report)
        finally:
            setattr(sdk, attribute, original)
        expected = [] if invalid_fk else [0x471, 0x151, 0x155, 0x156, 0x157]
        self.assertEqual([frame[0] for frame in transport.frames], expected)
        return report

    def test_reviewed_complete_enable_joint_target_and_arrival(self):
        self.assertTrue(self.run_reviewed_full_step()["arrival_verified"])

    def test_reviewed_timeout_cannot_pass_and_does_not_retry(self):
        self.assertFalse(self.run_reviewed_full_step(False)["arrival_verified"])

    def test_reviewed_fk_mismatch_rejected_before_any_send(self):
        report = self.run_reviewed_full_step(invalid_fk=True)
        self.assertEqual(report["transmissions"], [])

    def test_joint_progress_corridor_allows_audited_dip_but_not_escape(self):
        self.state.reviewed_reentry = True
        self.populate_reentry()
        origin = self.state.snapshot(10.01)
        sample = dict(origin, pose_raw=list(origin["pose_raw"]))
        sample["pose_raw"][2] -= 6000
        step.check_path(sample, origin, reviewed_reentry=True)
        with self.assertRaises(step.Rejected):
            step.check_path(sample, origin, reviewed_reentry=False)
        sample["pose_raw"][2] -= 2000
        with self.assertRaises(step.Rejected):
            step.check_path(sample, origin, reviewed_reentry=True)

    def run_reviewed_main_flow(self, cancel=False):
        self.state.reviewed_reentry = True
        self.populate_reentry()
        sample = self.state.snapshot(10.01)
        events = []
        receiver = types.SimpleNamespace(close=lambda: None)

        def user_ready(prompt):
            events.append("input")
            return "cancel" if cancel else ""

        def create_receiver(state):
            events.append("receiver")
            return receiver

        def live_preview(*args, **kwargs):
            events.append("live_preview")
            return sample

        def execute(vendor, receiver, state, report):
            events.append("execute")
            report.update(status="arrived", arrival_verified=True)

        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(step, "Vendor", return_value=self.vendor), \
                mock.patch.object(step, "Receiver", side_effect=create_receiver) as creator, \
                mock.patch.object(step, "collect_stationary", side_effect=live_preview) as collector, \
                mock.patch.object(step, "execute_step", side_effect=execute), \
                mock.patch.object(step, "inspect_binding", return_value={"ifindex": 8}), \
                mock.patch.object(step, "inspect_controllers", return_value=[]), \
                mock.patch.object(step.socket, "socket", side_effect=AssertionError("No CAN socket in offline test")), \
                mock.patch.object(step.sys.stdin, "isatty", return_value=True), \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO), \
                mock.patch("builtins.input", side_effect=user_ready) as input_mock:
            code = step.main(["--output-dir", directory, "--reviewed-joint-reentry"])
            report = json.loads((Path(directory) / "report.json").read_text())
            input_mock.assert_called_once()
            if cancel:
                creator.assert_not_called()
                collector.assert_not_called()
        return code, events, report

    def test_reviewed_readiness_precedes_socket_and_live_pose_without_second_pause(self):
        code, events, report = self.run_reviewed_main_flow()
        self.assertEqual(code, 0)
        self.assertEqual(events, ["input", "receiver", "live_preview", "execute"])
        self.assertEqual(report["reviewed_joint_reentry"]["start_joints_raw"], list(step.REENTRY_START))
        self.assertIn(str(step.REENTRY_OBSERVATION), report["reviewed_joint_reentry"]["source_artifacts_sha256"])

    def test_cancelled_reviewed_readiness_opens_no_can_and_sends_nothing(self):
        code, events, report = self.run_reviewed_main_flow(cancel=True)
        self.assertEqual(code, 2)
        self.assertEqual(events, ["input"])
        self.assertEqual(report["frames_send_attempted"], 0)
        self.assertEqual(report["transmissions"], [])


if __name__ == "__main__":
    unittest.main()
