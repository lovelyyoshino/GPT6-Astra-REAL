import unittest

from right_pick.offline import ReplayBackend
from right_pick.protocol import ProtocolError


class ReplayTests(unittest.TestCase):
    def test_readonly_state_is_explicitly_nonphysical(self):
        backend = ReplayBackend(clock=lambda: 100.0)
        state = backend.observe()
        self.assertTrue(state["nonphysical"])
        self.assertEqual(state["images"], [])
        state["right_tcp_position_m"][0] = 999
        self.assertNotEqual(backend.observe()["right_tcp_position_m"][0], 999)

    def test_replay_actions_update_virtual_state_and_never_claim_success(self):
        backend = ReplayBackend(clock=lambda: 100.0)
        action = {"type": "gripper", "width_m": 0.03, "issued_at": 100.0,
                  "ttl_s": 1, "calibration_version": backend.calibration_version}
        feedback = backend.execute(action)
        self.assertEqual(backend.observe()["right_gripper_width_m"], 0.03)
        self.assertIsNone(feedback["task_success"])
        self.assertIsNone(feedback["physical_motion_time_s"])
        stopped = backend.execute({"type": "stop"})
        self.assertEqual(stopped["status"], "decision_stopped")
        self.assertIsNone(stopped["task_success"])

    def test_physical_calibration_cannot_leak_into_replay(self):
        backend = ReplayBackend()
        with self.assertRaises(ProtocolError):
            backend.execute({"type": "gripper", "width_m": 0.03, "issued_at": 1,
                             "ttl_s": 1, "calibration_version": "real-camera-v2"})


if __name__ == "__main__":
    unittest.main()
