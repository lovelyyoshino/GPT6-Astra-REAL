"""Real vendor encoding on fake CAN only; feedback is not robot dynamics."""
import copy
import errno
import time
import unittest
from unittest.mock import Mock, patch

from robot_tools import arms, home_arm, linear_hold, takeover
from test_backend import PROFILE
from test_execution import Clock, healthy_arm


class RealSDKHomeArmTests(unittest.TestCase):
    def vendor_case(self, fail_id=None):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus

        clock, attempted, sent, robots = Clock(), [], [], {}
        initial = [0.656715, -0.0268955, 0.0622559, -0.0209095, 0.351544, 0.012828]
        # Explicit site binding; neither test channel ever reaches SocketCAN.
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper"
        profile["arms"]["right"]["channel"] = "can1"
        mode = {"can0": 0, "can1": 0}
        joints = {"can0": [0, -0.01, 0.02, 0, 0, 0], "can1": initial[:]}

        class FakeBus:
            def __init__(self, channel):
                self.channel = channel

            def recv(self, timeout=None):
                time.sleep(0.001)
                return None

            def send(self, frame, timeout=None):
                attempted.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == fail_id:
                    # Vendor CAN transport catches this exception; the platform
                    # must preserve the failed attempt and terminate the batch.
                    raise OSError(errno.ENOBUFS, "mock home CAN queue full")
                sent.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == 0x151:
                    mode[self.channel] = 1
                elif frame.arbitration_id == 0x157:
                    joints[self.channel] = [0.0] * 6

            def shutdown(self):
                pass

        original_factory = sdk.AgxArmFactory.create_arm

        def create_robot(config):
            robot = original_factory(config)
            channel = config["comm"]["can"]["channel"]
            robots[channel] = robot
            robot.move_j = Mock(wraps=robot.move_j)
            return robot

        def snapshot(robot, gripper):
            channel = next(key for key, value in robots.items() if value is robot)
            selected = channel == "can1"
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(
                ctrl_mode=1 if selected else 0, mode_feedback=mode[channel],
                teach_status=0, motion_status=0, arm_status=0, err_code=0))
            state["joints_rad"] = joints[channel][:]
            state["pose_m_rad"] = robot.fk(joints[channel][:])
            for driver in state["drivers"].values():
                driver["foc_status"]["driver_enable_status"] = selected
            state["gripper"]["width_m"] = 0.0028 if selected else 0.01141
            state["gripper"]["foc_status"]["driver_enable_status"] = selected
            return state

        with patch("socket.socket", side_effect=AssertionError("Physical sockets forbidden")), \
             patch.object(arms, "_preflight"), patch.object(arms, "time", clock), \
             patch.object(takeover, "time", clock), patch.object(linear_hold, "time", clock), \
             patch.object(home_arm, "time", clock), \
             patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeBus(kw["channel"])), \
             patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create_robot), \
             patch.object(arms, "snapshot", side_effect=snapshot):
            result = home_arm.home_arm(profile, lambda event, data: None, "right")

        robots["can1"].move_j.assert_called_once_with([0.0] * 6)
        robots["can0"].move_j.assert_not_called()
        self.assertTrue(all(channel == "can1" for channel, _ in attempted))
        self.assertFalse(any(frame.arbitration_id == 0x159 for _, frame in attempted))
        self.assertEqual(result["transmission_counts"]["left"]["attempted_frames"], 0)
        self.assertEqual(result["passive_arm_commands_sent"], 0)
        self.assertEqual(result["gripper_target_commands_sent"], 0)
        self.assertEqual(result["enable_commands_sent"], 0)
        self.assertEqual(result["stop_commands_sent"], 0)
        self.assertEqual(result["retries"], 0)
        self.assertFalse(result["task_motion_ready"])
        self.assertFalse(result["joint_zero_calibrated"])
        self.assertFalse(result["general_stop_validated"])
        self.assertFalse(result["path_collision_verified"])
        self.assertFalse(result["motion_gate_unlocked"])
        return result, attempted, sent

    def test_vendor_move_j_encodes_one_percent_mode_then_three_zero_targets(self):
        result, attempted, sent = self.vendor_case()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["zero_target_observed"])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertEqual(result["target_calls_sent"], 1)
        self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], 4)
        expected = [(0x151, "0101010000000000"),
                    (0x155, "0000000000000000"),
                    (0x156, "0000000000000000"),
                    (0x157, "0000000000000000")]
        self.assertEqual([(frame.arbitration_id, bytes(frame.data).hex()) for _, frame in sent], expected)
        self.assertEqual(len(attempted), len(sent))
        for _, frame in sent:
            self.assertEqual(frame.dlc, 8)
            for name in ("is_extended_id", "is_remote_frame", "is_error_frame", "is_fd",
                         "bitrate_switch", "error_state_indicator"):
                self.assertFalse(getattr(frame, name), name)
        self.assertEqual(result["after"]["left"]["arm_status"]["ctrl_mode"], 0)
        self.assertFalse(result["after"]["left"]["gripper"]["foc_status"]["driver_enable_status"])
        self.assertEqual(result["after"]["right"]["gripper"]["width_m"], 0.0028)

    def test_vendor_swallowed_failure_at_each_frame_never_sends_remaining_frames(self):
        frame_ids = [0x151, 0x155, 0x156, 0x157]
        for index, fail_id in enumerate(frame_ids):
            with self.subTest(fail_id=hex(fail_id)):
                result, attempted, sent = self.vendor_case(fail_id)
                self.assertFalse(result["ok"], result)
                self.assertFalse(result["zero_target_observed"])
                self.assertEqual([frame.arbitration_id for _, frame in attempted], frame_ids[:index + 1])
                self.assertEqual([frame.arbitration_id for _, frame in sent], frame_ids[:index])
                self.assertEqual(result["hardware_commands_sent"], index)
                self.assertEqual(result["transmission_counts"]["right"]["attempted_frames"], index + 1)
                self.assertEqual(result["target_calls_sent"], 0)
                self.assertIn("CAN send failed", result["errors"][0]["detail"])


if __name__ == "__main__":
    unittest.main()
