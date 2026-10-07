"""Synthetic raw-frame audits, never hardware or a physical clearance claim."""
import copy
import importlib.util
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "probe_raw_audit_under_test", ROOT / "scripts/audit_joint_probe_raw_window.py")
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class RawProbeAuditTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device or process access"))
            blocker.start()
            self.addCleanup(blocker.stop)
        self.q = [0, 1000, -1000, 0, 0, 0]
        self.pose = [.25, 0., .25, 0., 0., 0.]
        self.before = dict(raw_q=self.q, q=[v * audit.RAD for v in self.q],
                           pose=self.pose, opening_m=.06706, jaw_code=64)
        self.limits = dict(max_state_age_s=.1, gripper_min_m=0., gripper_max_m=.07,
            workspace_min_m=[-.5, -.5, 0.], workspace_max_m=[.5, .5, .5],
            max_translation_step_m=.03, max_rotation_step_rad=.05)

    def payloads(self, q=None, jaw=67060):
        q = self.q if q is None else q
        values = {0x2a1: bytes.fromhex("0100010000000000"),
                  0x2a8: struct.pack(">iHBB", jaw, 0, 64, 0)}
        raw_pose = [250000, 0, 250000, 0, 0, 0]
        for i in range(3):
            values[0x2a2 + i] = struct.pack(">ii", *raw_pose[2*i:2*i+2])
            values[0x2a5 + i] = struct.pack(">ii", *q[2*i:2*i+2])
        for ident in range(0x261, 0x267):
            values[ident] = bytes([0, 0, 0, 0, 0, 64, 0, 0])
        return values

    def rows(self, *, middle_q=None, middle_jaw=67060):
        rows = []
        after = dict(raw_feedback={})
        for stamp, payloads in ((99.99, self.payloads()),
                (100.01, self.payloads(middle_q, middle_jaw)),
                (100.05, self.payloads())):
            for ident, data in sorted(payloads.items()):
                rows.append(dict(event="frame", id=ident, data_hex=data.hex(), timestamp=stamp,
                    host_received_at=stamp+.001, timestamp_basis="kernel_socket_SO_TIMESTAMPNS_unix",
                    kernel_timestamp_ns=round(stamp*1e9), msg_flags=0, socket_dropped_total=0))
                if stamp == 100.05:
                    after["raw_feedback"][hex(ident)] = dict(kernel_unix_s=stamp, data_hex=data.hex())
        return rows, after

    def check(self, rows, after):
        return audit.check_rows(rows, self.before, self.q, after, self.limits,
                                lambda q: list(self.pose), 100.)

    def test_complete_normal_feedback_passes_but_bad_then_good_stays_reported(self):
        clean = self.check(*self.rows())
        self.assertTrue(clean["guard_checks_clean"])
        self.assertEqual(clean["feedback_frame_count"], 42)
        bad = list(self.q)
        bad[2] = 1
        result = self.check(*self.rows(middle_q=bad))
        self.assertFalse(result["guard_checks_clean"])
        self.assertEqual(len(result["nominal_violations"]), 1)
        self.assertEqual(result["nominal_violations"][0]["raw"], 1)
        self.assertTrue(result["joint_tracking_violations"])
        self.assertTrue(result["all_14_result_feedback_frames_matched_exactly"])

    def test_joint_box_keeps_j5_integer300_and_other_axes003_radians(self):
        for axis in range(6):
            boundary = 300 if axis == 4 else 171
            for direction in (-1, 1):
                with self.subTest(axis=axis, direction=direction):
                    self.assertFalse(audit.outside_joint_box(direction*boundary, axis, 0, 0))
                    self.assertTrue(audit.outside_joint_box(direction*(boundary+1), axis, 0, 0))
        for delta, clean in ((300, True), (301, False)):
            q = list(self.q)
            q[4] += delta
            result = self.check(*self.rows(middle_q=q))
            self.assertEqual(not result["joint_tracking_violations"], clean)

    def test_jaw_half_millimetre_boundary_uses_every_raw_frame(self):
        for delta, clean in ((500, True), (501, False), (-500, True), (-501, False)):
            with self.subTest(delta_micrometres=delta):
                result = self.check(*self.rows(middle_jaw=67060+delta))
                self.assertEqual(not result["jaw_guard_violations"], clean)
                self.assertEqual(result["guard_checks_clean"], clean)

    def test_missing_or_different_terminal_feedback_never_qualifies(self):
        rows, after = self.rows()
        for key, value in (("kernel_unix_s", 100.051), ("data_hex", "0000000000000000")):
            changed = copy.deepcopy(after)
            changed["raw_feedback"]["0x2a8"][key] = value
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "All14"):
                self.check(rows, changed)
        missing = [row for row in rows if row["id"] != 0x266]
        with self.assertRaisesRegex(RuntimeError, "All14"):
            self.check(missing, after)


if __name__ == "__main__":
    unittest.main()
