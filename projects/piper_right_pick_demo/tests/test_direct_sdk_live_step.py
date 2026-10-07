"""Live driver integration with real vendor parser/encoder and fake CAN transport."""
import contextlib
import importlib
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
live = importlib.import_module("direct_sdk_live_step")
core = live.core

CASES = ((36562, -1420, 2961, -6295, 19667, -3052),
         (36562, -1420, 2871, -1949, 19739, -9688))


class LiveSDKTests(unittest.TestCase):
    def setUp(self):
        self.vendor = core.Vendor()
        self.state = live.LiveFeedback(self.vendor)

    def populate(self, joints, now, enabled=False, mode=0):
        pose = [round(x) for x in self.vendor.fk_pose_raw(joints)]
        frames = {0x2A1: bytes([mode, 0, 1, 0, 0, 0, 0, 0]),
                  0x2A8: struct.pack(">ihBB", 23000, 0, 64, 0)}
        frames.update({identifier: struct.pack(">ii", *pose[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.POSE_PARTS)})
        frames.update({identifier: struct.pack(">ii", *joints[i * 2:i * 2 + 2])
                       for i, identifier in enumerate(core.JOINT_PARTS)})
        frames.update({identifier: struct.pack(">HhbBH", 240, 30, 30, 64 if enabled else 0, 0)
                       for identifier in range(0x261, 0x267)})
        for identifier, data in frames.items():
            self.state.ingest(identifier, data, False, now)

    def run_case(self, joints=CASES[0], enable_works=True, planner_fails=False, reaches=True, overshoot=False,
                 partial_send=False):
        from live_lift_plan import plan_lift
        clock = [10.]
        self.populate(joints, clock[0])
        report = {"binding": {"ifindex": 8}, "trace": [], "transmissions": [],
                  "enable_verified": False, "enable_transmissions": 0,
                  "motion_target_sent": False, "arrival_verified": False}
        transport = core.RecordingPort()
        transport.Close = mock.Mock()
        recorded_send = transport.SendCanMessage

        def send(identifier, data, *args, **kwargs):
            if partial_send and identifier == 0x156:
                raise OSError("injected failure after first joint pair")
            return recorded_send(identifier, data, *args, **kwargs)

        transport.SendCanMessage = send
        attribute = "_C_PiperInterface_V2__arm_can"
        original = getattr(self.vendor.sdk, attribute)
        plans, planner_inputs = [], []

        def create(*args, **kwargs):
            setattr(self.vendor.sdk, attribute, transport)

        def receive(timeout):
            clock[0] += .01
            enabled = enable_works and any(i == 0x471 for i, _ in transport.frames)
            moving = any(i == 0x157 for i, _ in transport.frames)
            reached = reaches and moving and clock[0] > 10.06
            q = list(plans[-1]["target_joints_raw"] if reached else joints)
            if overshoot and reached and clock[0] < 10.12:
                q[4] += 300
            self.populate(q, clock[0], enabled=enabled, mode=1 if moving else 0)

        def planner(q, pose):
            self.assertTrue(report["enable_verified"])
            self.assertTrue(all(self.state.snapshot(clock[0])["motor_enabled"]))
            self.assertTrue(transport.frames)
            self.assertTrue(all(i == 0x471 for i, _ in transport.frames))
            planner_inputs.append((q, pose))
            if planner_fails:
                raise ValueError("deliberate IK rejection")
            plan = plan_lift(q, pose)
            plans.append(plan)
            return plan

        receiver = types.SimpleNamespace(one=receive, drain=lambda: None)
        try:
            with mock.patch.object(core, "collect_stationary", side_effect=lambda *a, **kw: self.state.snapshot(clock[0])), \
                    mock.patch.object(core, "inspect_binding", return_value={"ifindex": 8}), \
                    mock.patch.object(core, "inspect_controllers", return_value=[]), \
                    mock.patch.object(core.time, "monotonic", side_effect=lambda: clock[0]), \
                    mock.patch.object(self.vendor.sdk, "CreateCanBus", side_effect=create), \
                    mock.patch.object(live, "compute_while_receiving", side_effect=lambda fn, held, *a:
                                      fn(list(held["joints_raw"]), list(held["pose_raw"]))), \
                    contextlib.redirect_stdout(io.StringIO()):
                if not enable_works:
                    with self.assertRaisesRegex(core.Rejected, "enable not verified"):
                        live.run_live(self.vendor, receiver, self.state, report, planner)
                elif planner_fails:
                    with self.assertRaisesRegex(ValueError, "IK rejection"):
                        live.run_live(self.vendor, receiver, self.state, report, planner)
                elif partial_send:
                    with self.assertRaisesRegex(OSError, "injected failure"):
                        live.run_live(self.vendor, receiver, self.state, report, planner)
                elif overshoot:
                    with self.assertRaisesRegex(core.Rejected, "computed small-step interval"):
                        live.run_live(self.vendor, receiver, self.state, report, planner)
                elif not reaches:
                    with self.assertRaisesRegex(core.Rejected, "Five-second"):
                        live.run_live(self.vendor, receiver, self.state, report, planner)
                else:
                    live.run_live(self.vendor, receiver, self.state, report, planner)
        finally:
            setattr(self.vendor.sdk, attribute, original)
        transport.Close.assert_called_once()
        return report, transport.frames, planner_inputs

    def test_two_different_live_postures_enable_then_solve_actual_held_pose(self):
        for joints in CASES:
            with self.subTest(joints=joints):
                self.state = live.LiveFeedback(self.vendor)
                report, frames, inputs = self.run_case(joints)
                self.assertTrue(report["enable_verified"])
                self.assertTrue(report["motion_target_sent"])
                self.assertTrue(report["arrival_verified"])
                self.assertEqual(inputs[0][0], list(joints))
                self.assertEqual([i for i, _ in frames], [0x471, 0x151, 0x155, 0x156, 0x157])
                actual = [q for i, data in frames if i in (0x155, 0x156, 0x157)
                          for q in struct.unpack(">ii", data)]
                self.assertEqual(actual, report["plan"]["target_joints_raw"])

    def test_enable_failure_bounded_no_ik_or_joint_target(self):
        report, frames, inputs = self.run_case(enable_works=False)
        self.assertFalse(report["enable_verified"])
        self.assertFalse(report["motion_target_sent"])
        self.assertEqual(inputs, [])
        self.assertLessEqual(len(frames), 20)
        self.assertGreater(len(frames), 1)
        self.assertTrue(all(i == 0x471 for i, _ in frames))

    def test_ik_failure_reports_enable_verified_and_never_sends_mode(self):
        report, frames, inputs = self.run_case(planner_fails=True)
        self.assertTrue(report["enable_verified"])
        self.assertEqual(report["phase"], "planning")
        self.assertFalse(report["motion_target_sent"])
        self.assertEqual([i for i, _ in frames], [0x471])

    def test_movement_timeout_is_not_success_and_target_not_repeated(self):
        report, frames, inputs = self.run_case(reaches=False)
        self.assertTrue(report["enable_verified"])
        self.assertTrue(report["motion_target_sent"])
        self.assertFalse(report["arrival_verified"])
        self.assertEqual([i for i, _ in frames], [0x471, 0x151, 0x155, 0x156, 0x157])

    def test_live_range_bounded_without_historical_angles(self):
        self.populate(CASES[1], 10.)
        self.state.snapshot(10.01)
        q = list(CASES[1]); q[1] = -3001
        self.populate(q, 10.)
        with self.assertRaisesRegex(core.Rejected, "commissioning range"):
            self.state.snapshot(10.01)

    def test_stale_or_outside_modeled_gripper_rejected(self):
        self.populate(CASES[0], 10.)
        live.require_gripper_geometry(self.state, 10.01)
        with self.assertRaises(core.Rejected):
            live.require_gripper_geometry(self.state, 10.2)
        self.state.gripper["angle_raw"] = 70001
        with self.assertRaises(core.Rejected):
            live.require_gripper_geometry(self.state, 10.01)

    def test_boundary_violation_retained_while_passive_tail_observes_final_arrival(self):
        report, frames, _ = self.run_case(overshoot=True)
        self.assertFalse(report["arrival_verified"])
        self.assertEqual(report["post_failure_trigger"]["phase"], "motion")
        self.assertIn("computed small-step interval", report["post_failure_trigger"]["error"])
        self.assertTrue(any("validation_error" in item for item in report["post_failure_observations"]))
        self.assertTrue(report["post_failure_target_stable"])
        self.assertGreater(len(report["post_failure_observations"]), 20)
        self.assertLessEqual(report["post_failure_monitor_finished_at_monotonic_s"],
                             report["motion_deadline_monotonic_s"] + .02)
        self.assertEqual([i for i, _ in frames], [0x471, 0x151, 0x155, 0x156, 0x157])
        final_q = report["post_failure_final_diagnostic"]["joints_raw"]
        self.assertEqual([final_q[n] for n in core.JOINT_NAMES], report["plan"]["target_joints_raw"])

    def test_passive_tail_receive_errors_are_recorded_and_finite(self):
        clock = [10.]
        self.populate(CASES[0], clock[0], enabled=True)
        report = {"plan": {"target_pose_raw": self.state.snapshot(10.)["pose_raw"],
                           "target_joints_raw": list(CASES[0])},
                  "motion_sent_at_monotonic_s": 9.99, "arrival_verified": False}

        def broken_receive(timeout):
            clock[0] += .01
            raise OSError("disconnected receiver")

        def sleep(seconds):
            clock[0] += seconds

        with mock.patch.object(core.time, "monotonic", side_effect=lambda: clock[0]), \
                mock.patch.object(core.time, "sleep", side_effect=sleep), \
                contextlib.redirect_stdout(io.StringIO()):
            live.observe_after_failure(types.SimpleNamespace(one=broken_receive), self.state, report, 10.5)
        self.assertGreaterEqual(report["post_failure_receive_error_count"], 20)
        self.assertLessEqual(len(report["post_failure_receive_errors"]), 20)
        self.assertLessEqual(clock[0], 10.52)
        self.assertFalse(report["arrival_verified"])

    def test_partial_joint_send_still_observed_without_claiming_target_arrival(self):
        report, frames, _ = self.run_case(partial_send=True)
        self.assertTrue(report["partial_motion_target"])
        self.assertFalse(report["motion_target_sent"])
        self.assertFalse(report["arrival_verified"])
        self.assertFalse(report["post_failure_target_stable"])
        self.assertIn("injected failure", report["post_failure_trigger"]["error"])
        self.assertGreater(len(report["post_failure_observations"]), 20)
        self.assertTrue(all("target_assessment" in item for item in report["post_failure_observations"]))
        self.assertEqual([i for i, _ in frames], [0x471, 0x151, 0x155])
        self.assertEqual([entry["id"] for entry in report["transmissions"]],
                         ["0x471", "0x151", "0x155", "0x156"])
        self.assertLessEqual(report["post_failure_monitor_finished_at_monotonic_s"],
                             report["motion_deadline_monotonic_s"] + .02)


if __name__ == "__main__":
    unittest.main()
