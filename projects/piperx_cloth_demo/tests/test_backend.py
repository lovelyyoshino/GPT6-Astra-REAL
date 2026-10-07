"""Backend contract tests only: CAN sockets are forbidden throughout."""
import copy
import math
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from robot_tools import arms
from robot_tools.backend import PyAgxBackend


PROFILE = {"sdk_path": "/home/agilex/pyAgxArm", "preview_max_speed_percent": 5,
           "arms": {"left": {"model": "piper_x", "firmware": "default", "channel": "can0", "usb_interface": "1-6.1:1.0"},
                    "right": {"model": "piper_x", "firmware": "default", "channel": "can2", "usb_interface": "1-6.3:1.0"}}}
TARGET = {"arm": "right", "frame": "right_base", "reference": "sdk_flange",
          "pose_m_rad": [0.15, 0.01, 0.25, 0.1, 0.2, -0.1], "motion": "move_l", "speed_percent": 3}


class FakeRobot:
    def __init__(self):
        self.raw_send = Mock()
        self.swallow_error = False
        self.clear_error_like_rx = False
        self.comm = NS(send=self._comm_send, send_bus=NS(send=self.raw_send), last_error=None)
        self.calls = []
        self.gripper = NS(move_gripper_m=self.grip)
        self.disconnect = Mock()

    def _comm_send(self, *args, **kwargs):
        try:
            return self.comm.send_bus.send(*args, **kwargs)
        except Exception as exc:
            self.comm.last_error = exc
            if not self.swallow_error:
                raise
            if self.clear_error_like_rx:
                self.comm.last_error = None

    def init_effector(self, kind):
        self.calls.append(("init_effector", kind))
        return self.gripper

    def create_comm(self):
        return self.comm

    def connect(self):
        self.calls.append(("connect",))

    def set_speed_percent(self, value):
        self.calls.append(("speed", value))
        self.comm.send("speed")

    def move_l(self, pose):
        self.calls.append(("move_l", list(pose)))
        self.comm.send("pose_part_1")
        self.comm.send("pose_part_2")
        pose[0] = 999  # Prove an SDK mutation cannot rewrite the authored plan.

    def move_p(self, pose):
        self.calls.append(("move_p", list(pose)))
        self.comm.send("pose")

    def grip(self, **kwargs):
        self.calls.append(("grip", kwargs))
        self.comm.send("grip")


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.socket_guard = patch("socket.socket", side_effect=AssertionError("Real sockets forbidden"))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)

    def prepared(self):
        backend = PyAgxBackend(PROFILE)
        robots = [FakeRobot(), FakeRobot()]
        sdk = NS(create_agx_arm_config=Mock(side_effect=lambda **kwargs: kwargs),
                 AgxArmFactory=NS(create_arm=Mock(side_effect=robots)))
        # Bypassing commissioning is explicit and LOCAL TO TEST MOCKS, never a
        # profile option. Preflight and SDK factory are both replaced by fakes.
        for patcher in (patch.object(backend, "commissioning_errors", return_value=[]),
                        patch.object(arms, "_preflight"),
                        patch.object(arms, "_load_sdk", return_value=sdk)):
            patcher.start()
            self.addCleanup(patcher.stop)
        backend.connect()
        stamp = time.time()
        state = {"status": "complete", "fragment_timestamps_s": {"arm_status": stamp}}
        for patcher in (patch.object(arms, "snapshot", side_effect=lambda *args: copy.deepcopy(state)),
                        patch.object(arms, "control_health", return_value={"healthy": True})):
            patcher.start()
            self.addCleanup(patcher.stop)
        return backend, robots, sdk

    def test_constructor_and_profile_flags_cannot_open_or_unlock(self):
        profile = copy.deepcopy(PROFILE)
        profile["verification"] = dict.fromkeys(("hold_stop_and_recovery", "everything"), True)
        profile["execution_implemented"] = True
        with patch.object(arms, "_load_sdk") as load, patch.object(arms, "_preflight") as preflight:
            backend = PyAgxBackend(profile)
            self.assertTrue(backend.commissioning_errors())
            for action in (backend.connect, lambda: backend.move(TARGET), lambda: backend.grip(TARGET)):
                with self.assertRaisesRegex(RuntimeError, "blocked"):
                    action()
            load.assert_not_called()
            preflight.assert_not_called()
        self.assertEqual(backend.transmission_counts()["sent_frames"], 0)

    def test_both_connections_no_automatic_enable_or_tx(self):
        backend, robots, sdk = self.prepared()
        for robot in robots:
            self.assertEqual(robot.calls, [("init_effector", "agx_gripper"), ("connect",)])
            robot.raw_send.assert_not_called()
        for call in sdk.create_agx_arm_config.call_args_list:
            self.assertIs(call.kwargs["auto_connect"], False)
            self.assertIs(call.kwargs["enable_check_can"], False)
        self.assertEqual(backend.transmission_counts()["attempted_frames"], 0)

    def test_pass_through_move_pose_and_count_actual_send_calls(self):
        backend, robots, _ = self.prepared()
        target = copy.deepcopy(TARGET)
        result = backend.move(target)
        self.assertEqual(target, TARGET)
        self.assertEqual(robots[1].calls[-2:], [("speed", 3), ("move_l", TARGET["pose_m_rad"])])
        self.assertEqual(result["status"], "sent_unconfirmed")
        self.assertIsNone(result["controller_accepted"])
        self.assertEqual(backend.transmission_counts()["sent_frames"], 3)
        self.assertEqual(backend.transmission_counts()["arms"]["left"]["sent_frames"], 0)

    def test_gripper_explicit_meters_force_and_no_default(self):
        backend, robots, _ = self.prepared()
        with self.assertRaises(ValueError):
            backend.grip({"arm": "left", "gripper_width_m": 0.04})
        with self.assertRaises(ValueError):
            backend.grip({"arm": "left", "gripper_width_m": 0.04, "gripper_force_N": 6})
        backend.grip({"arm": "left", "gripper_width_m": 0.04, "gripper_force_N": 0.2})
        self.assertEqual(robots[0].calls[-1], ("grip", {"value": 0.04, "force": 0.2}))
        self.assertEqual(backend.transmission_counts()["sent_frames"], 1)

    def test_validation_happens_before_speed_transmission(self):
        backend, robots, _ = self.prepared()
        for field, value in (("speed_percent", 10), ("motion", "move_j"), ("frame", "left_base"),
                             ("pose_m_rad", [0.2, 0, 0.3, 0, math.pi, 0])):
            with self.subTest(field=field), self.assertRaises(ValueError):
                backend.move({**TARGET, field: value})
        robots[1].raw_send.assert_not_called()

    def test_unhealthy_feedback_or_uncommissioned_again_prevents_send(self):
        backend, robots, _ = self.prepared()
        with patch.object(arms, "control_health", return_value={"healthy": False, "reasons": ["stale"]}):
            with self.assertRaisesRegex(RuntimeError, "Unhealthy"):
                backend.move(TARGET)
        backend.commissioning_errors.return_value = [{"code": "hold_lost"}]
        with self.assertRaisesRegex(RuntimeError, "blocked"):
            backend.move(TARGET)
        robots[1].raw_send.assert_not_called()

    def test_swallowed_vendor_send_error_never_counts_success(self):
        backend, robots, _ = self.prepared()
        robots[1].swallow_error = True
        robots[1].raw_send.side_effect = OSError("ENOBUFS")
        with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
            backend.move(TARGET)
        counts = backend.transmission_counts()
        self.assertEqual(counts["attempted_frames"], 1)
        self.assertEqual(counts["sent_frames"], 0)
        self.assertFalse(backend._transmitting)

    def test_rx_cannot_erase_swallowed_bus_send_failure(self):
        backend, robots, _ = self.prepared()
        robot = robots[1]
        robot.swallow_error = robot.clear_error_like_rx = True
        robot.raw_send.side_effect = OSError("ENOBUFS")
        with self.assertRaisesRegex(RuntimeError, "CAN send failed: ENOBUFS"):
            backend.move(TARGET)
        self.assertIsNone(robot.comm.last_error)
        self.assertEqual(backend.transmission_counts()["attempted_frames"], 1)
        self.assertEqual(backend.transmission_counts()["sent_frames"], 0)

    def test_partial_sdk_transmission_preserves_exact_counts(self):
        backend, robots, _ = self.prepared()
        robots[1].raw_send.side_effect = [None, None, RuntimeError("bus lost")]
        with self.assertRaisesRegex(RuntimeError, "bus lost"):
            backend.move(TARGET)
        self.assertEqual(backend.transmission_counts()["attempted_frames"], 3)
        self.assertEqual(backend.transmission_counts()["sent_frames"], 2)

    def test_unsolicited_sdk_send_is_blocked(self):
        backend, robots, _ = self.prepared()
        with self.assertRaisesRegex(RuntimeError, "outside"):
            robots[0].comm.send("unexpected")
        robots[0].raw_send.assert_not_called()
        self.assertEqual(backend.transmission_counts()["attempted_frames"], 0)

    def test_hold_has_no_substitute_and_cleanup_is_not_stop(self):
        backend, robots, _ = self.prepared()
        hold = backend.request_hold_all()
        self.assertFalse(hold["held"])
        self.assertTrue(all(v["status"] == "unavailable" for v in hold["arms"].values()))
        robots[0].disconnect.side_effect = RuntimeError("cleanup failed")
        result = backend.close()
        self.assertEqual(result["status"], "partial")
        self.assertEqual(result["arms"]["left"]["status"], "cleanup_failed")
        self.assertIsNone(result["stopped"])
        self.assertEqual(backend.transmission_counts()["sent_frames"], 0)
        robots[1].disconnect.assert_called_once()

    def test_second_connection_failure_cleans_both(self):
        backend = PyAgxBackend(PROFILE)
        robots = [FakeRobot(), FakeRobot()]
        robots[1].connect = Mock(side_effect=RuntimeError("no bus"))
        sdk = NS(create_agx_arm_config=lambda **kwargs: kwargs,
                 AgxArmFactory=NS(create_arm=Mock(side_effect=robots)))
        with patch.object(backend, "commissioning_errors", return_value=[]), \
             patch.object(arms, "_preflight"), patch.object(arms, "_load_sdk", return_value=sdk):
            with self.assertRaisesRegex(RuntimeError, "Connection failed"):
                backend.connect()
        self.assertFalse(backend._connected)
        for robot in robots:
            robot.disconnect.assert_called_once()
            robot.raw_send.assert_not_called()

    def test_vendor_quaternion_pose_error_requires_no_bus(self):
        backend = PyAgxBackend(PROFILE)
        position, angle = backend.pose_error([0, 0, 0, 0, 0, math.pi - 0.01],
                                             [0.03, 0.04, 0, 0, 0, -math.pi + 0.01])
        self.assertAlmostEqual(position, 0.05)
        self.assertAlmostEqual(angle, 0.02)
        self.assertEqual(backend.transmission_counts()["sent_frames"], 0)

    def test_actual_vendor_methods_and_encoding_on_fake_bus_only(self):
        arms._load_sdk(PROFILE["sdk_path"])
        import can
        sent = []
        class FakeBus:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(0.002)
                return None
            def send(self, frame, timeout=None):
                sent.append((self.channel, frame.arbitration_id, bytes(frame.data)))
            def shutdown(self):
                pass
        backend = PyAgxBackend(PROFILE)
        with patch.object(backend, "commissioning_errors", return_value=[]), \
             patch.object(arms, "_preflight"), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FakeBus(kwargs["channel"])), \
             patch.object(backend, "_command_guard"):
            try:
                backend.connect()
                self.assertEqual(sent, [])
                backend.move(TARGET)
                backend.grip({"arm": "left", "gripper_width_m": 0.04, "gripper_force_N": 0.2})
                right_ids = {can_id for channel, can_id, data in sent if channel == "can2"}
                self.assertTrue({0x151, 0x152, 0x153, 0x154}.issubset(right_ids))
                left = [(can_id, data) for channel, can_id, data in sent if channel == "can0"]
                self.assertEqual([can_id for can_id, _ in left], [0x159])
                self.assertEqual(backend.transmission_counts()["sent_frames"], len(sent))
            finally:
                self.assertEqual(backend.close()["status"], "complete")

    def test_actual_sdk_swallowed_error_with_simulated_rx_clear(self):
        import errno
        arms._load_sdk(PROFILE["sdk_path"])
        import can
        class FailingBus:
            def recv(self, timeout=None):
                time.sleep(0.002)
                return None
            def send(self, frame, timeout=None):
                raise OSError(errno.ENOBUFS, "simulated transmit queue full")
            def shutdown(self):
                pass
        backend = PyAgxBackend(PROFILE)
        with patch.object(backend, "commissioning_errors", return_value=[]), \
             patch.object(arms, "_preflight"), \
             patch.object(can.interface, "Bus", side_effect=lambda **kwargs: FailingBus()), \
             patch.object(backend, "_command_guard"):
            try:
                backend.connect()
                comm = backend.robots["right"].get_context().get_comm()
                def classify_and_emulate_rx_clear(exc):
                    # Actual vendor send assigned last_error, then another RX
                    # succeeded before it returns through its swallowed branch.
                    comm.last_error = None
                    return errno.ENOBUFS
                with patch.object(comm, "_classify_can_error", side_effect=classify_and_emulate_rx_clear):
                    with self.assertRaisesRegex(RuntimeError, "CAN send failed"):
                        backend.move(TARGET)
                self.assertIsNone(comm.last_error)
                self.assertEqual(backend.transmission_counts()["attempted_frames"], 1)
                self.assertEqual(backend.transmission_counts()["sent_frames"], 0)
            finally:
                self.assertEqual(backend.close()["status"], "complete")

    def test_run_plan_real_backend_health_move_grip_and_successful_close(self):
        from robot_tools import execution
        from test_execution import Clock, FakeJournal, healthy_arm

        # Keep the manufacturer's real quaternion conversion for pose_error.
        arms._load_sdk(PROFILE["sdk_path"])
        clock = Clock()
        class ObservedRobot(FakeRobot):
            def __init__(self):
                super().__init__()
                self.state = healthy_arm(clock.time())
            def move_p(self, pose):
                super().move_p(pose)
                self.state["pose_m_rad"] = list(pose)
            def grip(self, **kwargs):
                super().grip(**kwargs)
                self.state["gripper"]["width_m"] = kwargs["value"]

        robots = [ObservedRobot(), ObservedRobot()]
        sdk = NS(create_agx_arm_config=lambda **kwargs: kwargs,
                 AgxArmFactory=NS(create_arm=Mock(side_effect=robots)))
        backend = PyAgxBackend(PROFILE)
        observation = {"state": {"arms": {side: copy.deepcopy(robot.state)
                          for side, robot in zip(("left", "right"), robots)}}}
        def observed_snapshot(robot, gripper):
            self.assertIs(gripper, robot.gripper)
            clock.sleep(0.001)
            state = copy.deepcopy(robot.state)
            state["timestamp"] = clock.time()
            state["fragment_timestamps_s"] = dict.fromkeys(state["fragment_timestamps_s"], clock.time())
            state["gripper"]["timestamp"] = clock.time()
            return state

        moves = [{**TARGET, "arm": side, "frame": side + "_base", "action": "move",
                  "motion": "move_p", "pose_m_rad": [x, 0.1, 0.3, 0, 0, 0]}
                 for side, x in (("left", 0.21), ("right", 0.23))]
        grips = [{**target, "action": "gripper", "gripper_width_m": width, "gripper_force_N": 0.2}
                 for target, width in zip(moves, (0.03, 0.04))]
        candidate = {"stages": [{"id": "approach", "coordination": "paired", "targets": moves},
                                 {"id": "grip", "coordination": "paired", "targets": grips}]}
        events = []
        with patch.object(backend, "commissioning_errors", return_value=[]), \
             patch.object(arms, "_preflight"), patch.object(arms, "_load_sdk", return_value=sdk), \
             patch.object(arms, "snapshot", side_effect=observed_snapshot), \
             patch.object(execution.time, "time", clock.time), \
             patch.object(execution.time, "monotonic", clock.monotonic), \
             patch.object(execution.time, "sleep", clock.sleep), \
             patch.dict(execution.LIMITS, {"poll_s": 0.001, "stable_s": 0.004,
                                           "stage_timeout_s": 0.03, "total_timeout_s": 1.0}):
            # Real backend guard, control_health, pose_error and run_plan are used.
            result = execution.run_plan(candidate, observation, backend, FakeJournal(events))

        self.assertTrue(result["ok"], result)
        self.assertEqual(result["status"], "targets_reached")
        self.assertEqual(result["completed_stages"], ["approach", "grip"])
        self.assertEqual(result["cleanup_errors"], [])
        self.assertEqual(result["cleanup_result"]["status"], "complete")
        self.assertIsNone(result["cleanup_result"]["stopped"])
        self.assertEqual(result["transmissions"]["sent_frames"], 6)
        self.assertFalse(result["grasp_verified"])
        for side, robot, target in zip(("left", "right"), robots, grips):
            robot.disconnect.assert_called_once()
            self.assertEqual(result["final_feedback"]["arms"][side]["pose_m_rad"], target["pose_m_rad"])
            self.assertEqual(result["final_feedback"]["arms"][side]["gripper"]["width_m"], target["gripper_width_m"])


if __name__ == "__main__":
    unittest.main()
