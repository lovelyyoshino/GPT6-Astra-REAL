"""Pinned SDK limit semantics on constructed vendor drivers and FakeCAN only.

These tests establish encoding/transport facts, never controller acceptance,
physical arrival, a recoverable path, or qualification of an out-of-limit q.
"""
import copy
import io
import math
import struct
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from robot_tools import arms
from test_arms import SDK_PATH


class SDKJointLimitSemanticsTests(unittest.TestCase):
    def setUp(self):
        sockets = patch("socket.socket", side_effect=AssertionError("Physical sockets forbidden"))
        sockets.start()
        self.addCleanup(sockets.stop)
        sdk = arms._load_sdk(SDK_PATH)
        import can
        self.frames, self.fake_buses = [], []
        frames = self.frames
        class FakeCAN:
            def send(self, frame, timeout=None):
                frames.append(copy.deepcopy(frame))
            def recv(self, timeout=None):
                return None  # Deliberately no controller ACK or feedback.
            def shutdown(self):
                pass
        def bus_factory(**kwargs):
            bus = FakeCAN()
            self.fake_buses.append(bus)
            return bus
        buses = patch.object(can.interface, "Bus", side_effect=bus_factory)
        buses.start()
        self.addCleanup(buses.stop)
        config = sdk.create_agx_arm_config(robot="piper_x", comm="can",
                                          firmeware_version="default", channel="can0", interface="socketcan")
        # Use the real factory/constructor. In particular, never assign the
        # flag being tested or replace its initialization with a fake False.
        self.robot = sdk.AgxArmFactory.create_arm(config)
        self.addCleanup(self.robot.disconnect)
        self.robot.connect(start_read_thread=False)
        self.assertEqual(self.frames, [], "Construction/connect must be zero TX in this fixture")
        self.assertTrue(self.fake_buses, "All communication must terminate in FakeCAN")

    def encoded(self, values):
        with redirect_stdout(io.StringIO()):
            messages = self.robot._deal_move_j_msgs(values)
        return [getattr(message, "joint_%d" % index)
                for message, indexes in zip(messages, ((1, 2), (3, 4), (5, 6))) for index in indexes]

    def send(self, values):
        # Set only the reviewed in-memory speed field; set_speed_percent()
        # would emit an unrelated frame and is intentionally not called.
        self.robot._msg_mode.move_spd_rate_ctrl = 1
        with redirect_stdout(io.StringIO()):
            result = self.robot.move_j(values)
        self.assertIsNone(result)  # SDK call return carries no acceptance ACK.
        self.assertEqual([frame.arbitration_id for frame in self.frames], [0x151, 0x155, 0x156, 0x157])
        self.assertTrue(all(frame.dlc == 8 and not frame.is_extended_id for frame in self.frames))
        self.assertEqual(bytes(self.frames[0].data), bytes((1, 1, 1, 0, 0, 0, 0, 0)))
        self.assertIsNone(self.robot.get_arm_status(), "FakeCAN supplies no physical state or arrival")
        return [value for frame in self.frames[1:] for value in struct.unpack(">ii", bytes(frame.data))]

    def test_real_constructor_disables_model_joint_limits_by_default(self):
        self.assertIs(self.robot.get_joint_limits_enabled(), False)
        self.assertIs(self.robot.get_auto_set_motion_mode_enabled(), True)
        self.assertEqual(self.frames, [])

    def test_default_historical_j2_j3_offsets_encode_without_clamping_to_zero(self):
        q = [0., -.091769, .045658, 0., 0., 0.]
        before = q[:]
        raw = self.encoded(q)
        self.assertEqual(raw, [round(v * (180 / math.pi) * 1000) for v in q])
        self.assertLess(raw[1], 0)
        self.assertGreater(raw[2], 0)
        self.assertEqual(q, before)
        self.assertEqual(self.frames, [], "Pure message construction sends nothing")

    def test_explicit_enabled_uses_piper_x_nominal_joint_boundaries(self):
        self.robot.set_joint_limits_enabled(True)
        self.assertIs(self.robot.get_joint_limits_enabled(), True)
        q = [math.radians(200), -.091769, .045658, 2., -2., 4.]
        before = q[:]
        self.assertEqual(self.encoded(q), [150000, 0, 0, 89000, -89000, 180000])
        self.assertEqual(q, before)
        self.assertEqual(self.frames, [])

    def test_default_still_clamps_to_generic_plus_minus_two_pi(self):
        self.assertIs(self.robot.get_joint_limits_enabled(), False)
        self.assertEqual(self.encoded([3*math.pi, -3*math.pi, 4*math.pi, -4*math.pi, 0., 0.]),
                         [360000, -360000, 360000, -360000, 0, 0])
        self.assertEqual(self.frames, [])

    def test_public_default_move_j_sends_four_frames_with_unclipped_small_offsets_without_ack(self):
        q = [0., -.091769, .045658, 0., 0., 0.]
        self.assertIs(self.robot.get_joint_limits_enabled(), False)
        raw = self.send(q)
        self.assertEqual(raw, [round(v * (180 / math.pi) * 1000) for v in q])
        self.assertLess(raw[1], 0)
        self.assertGreater(raw[2], 0)

    def test_public_explicit_enabled_move_j_sends_four_frames_with_model_clipping_without_ack(self):
        self.robot.set_joint_limits_enabled(True)
        raw = self.send([math.radians(200), -.091769, .045658, 2., -2., 4.])
        self.assertEqual(raw, [150000, 0, 0, 89000, -89000, 180000])


if __name__ == "__main__":
    unittest.main()
