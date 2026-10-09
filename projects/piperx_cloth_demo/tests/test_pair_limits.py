"""Retained-connection limit capture: all sockets and physical CAN forbidden."""
import copy
import json
import math
import time
import unittest
from contextlib import ExitStack
from unittest.mock import Mock, patch

from robot_tools import arms, linear_hold, pair_device, pair_limits, pair_preparation
from robot_tools import single_supervised_actions, supervised_actions, takeover
from robot_tools.joint_sources import JointSourcesProvider
from test_backend import PROFILE
from test_execution import Clock, healthy_arm
from test_joint_limits import reply_bytes
from test_single_gripper_prepare import GripperRobot, SingleGripperFixture


class QueryRobot(GripperRobot):
    def __init__(self, side, clock):
        super().__init__(side, True)
        self.clock, self.calls = clock, []
        self.reply_transform = lambda frame: frame
        self.extra_reply = False
        self.extra_reply_transform = lambda frame: frame
        self.extra_reply_count = 1
        self.no_reply_joint = None
        self.raw = reply_bytes
        self.on_query = None
        self.reply_side = None
        self.comm.get_callback = lambda: self.callback

    def get_joint_angle_vel_limits(self, *, joint_index, timeout, min_interval):
        self.calls.append((joint_index, timeout, min_interval))
        frame = self.can.Message(arbitration_id=0x472, is_extended_id=False,
                                 data=bytes((joint_index, 1, 0, 0, 0, 0, 0, 0)))
        self._send_msg(self.frame_transform(frame))
        if self.duplicate:
            self._send_msg(frame)
        return {"stale_sdk_cache": True}  # Not a raw request confirmation.

    def _bus_send(self, frame):
        if self.send_error:
            raise self.send_error
        self.sent.append(copy.deepcopy(frame))
        if frame.arbitration_id != 0x472:
            self.gripper_enabled = True
            return
        joint = frame.data[0]
        if self.on_query is not None:
            self.on_query(self, joint)
        if self.no_reply_joint == joint:
            return
        reply = self.reply_transform(self.can.Message(arbitration_id=0x473, is_extended_id=False,
            timestamp=self.clock.time(), data=self.raw(joint)))
        callback = self.reply_side.callback if self.reply_side is not None else self.callback
        callback(reply)
        if self.extra_reply:
            for _ in range(self.extra_reply_count):
                callback(self.extra_reply_transform(copy.deepcopy(reply)))


class PairLimitsTests(SingleGripperFixture):
    def setUp(self):
        super().setUp()
        for module in (pair_device, pair_limits, pair_preparation, linear_hold,
                       single_supervised_actions, supervised_actions):
            self.stack.enter_context(patch.object(module, "time", self.clock))
        self.robots = {side: QueryRobot(side, self.clock) for side in takeover.SIDES}
        self.sdk.AgxArmFactory.create_arm.side_effect = self.robots.values()
        self.guard_hook = None
        self.device = pair_device.GuardedPairDevice(self.profile,
            lambda event, data: self.events.append((event, data)), self.guard)
        self.addCleanup(self.device.close)

    def guard(self):
        if self.guard_hook:
            self.guard_hook()

    def open(self, ready=False):
        if ready:
            for robot in self.robots.values():
                robot.gripper_enabled = True
            return self.device.open()
        return self.device.connect_for_preparation()

    def assert_queries(self, left, right):
        for side, count in (("left", left), ("right", right)):
            self.assertEqual([frame.arbitration_id for frame in self.robots[side].sent], [0x472]*count)

    def test_twelve_raw_windows_schema_no_actuation_reconnect_or_readiness_change(self):
        self.open()
        anchor = copy.deepcopy(self.device._preparation.anchor)
        flags = copy.deepcopy(self.device._preparation.flags)
        callbacks = {side: robot.callback for side, robot in self.robots.items()}
        report = self.device.inspect_joint_limits()
        self.assertTrue(report["ok"], report)
        self.assertEqual(report["status"], "joint_limits_received_no_motion_commands")
        self.assertEqual(report["joint_limit_queries_sent"], 12)
        self.assertEqual(report["hardware_commands_sent"], 12)
        self.assertEqual(report["actuator_commands_sent"], 0)
        self.assertFalse(report["task_ready"])
        self.assertFalse(self.device._task_ready)
        self.assertEqual(self.device._preparation.anchor, anchor)
        self.assertEqual(self.device._preparation.flags, flags)
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)
        self.assert_queries(6, 6)
        for side, robot in self.robots.items():
            self.assertEqual(robot.calls, [(i, 0.0, 0.0) for i in range(1, 7)])
            self.assertIs(robot.callback, callbacks[side])
            self.assertFalse(robot.disconnect.called)
            for i in range(1, 7):
                row, query = report["joint_limits"][side][str(i)], report["query_receipts"][side][str(i)]
                self.assertEqual(row["status"], "confirmed")
                self.assertEqual(row["raw_response_hex"], reply_bytes(i).hex())
                self.assertEqual(query["data_hex"], bytes((i,1,0,0,0,0,0,0)).hex())
                self.assertFalse(row["response_evidence"]["active"])
                self.assertAlmostEqual(row["decoded_limits_rad"][0], -math.pi)
        # Exercise the production provider's complete temporal/raw validator.
        report.update(run_id="run", owner="owner", bindings={"test": "host injects bindings"})
        provider = JointSourcesProvider.__new__(JointSourcesProvider)
        provider.run_id, provider.clock = "run", self.clock.time
        provider._ref = lambda value: (report, {"ref": "capture", "sha256": "a"*64})
        limits = provider._limits({"ref": "ignored"}, "owner", report["bindings"])
        self.assertEqual(limits["left"], [[-math.pi, math.pi]]*6)
        self.assertTrue(self.device.observe()["connected_for_preparation"])

    def test_ready_action_context_and_original_anchors_survive_all_twelve_queries(self):
        self.open(ready=True)
        action = self.device._action
        saved = {key: copy.deepcopy(getattr(action, key)) for key in pair_limits._ACTION_FIELDS}
        original_anchor = copy.deepcopy(action.idle_anchor)
        self.assertTrue(self.device.inspect_joint_limits()["ok"])
        for key, value in saved.items():
            # The real initial observation can refresh diagnostic timestamps.
            if key != "report":
                self.assertEqual(getattr(action, key), value, key)
        self.assertEqual(action.idle_anchor, original_anchor)
        self.assertTrue(self.device.observe()["task_ready"])
        self.assertEqual(sum(v["sent_frames"] for v in action.totals().values()), 12)
        receipt = self.device.execute("right", "gripper", self.robots["right"].width)
        self.assertTrue(receipt["ok"], receipt)
        self.assertEqual(receipt["hardware_commands_sent"], 1)
        self.assertEqual(sum(v["sent_frames"] for v in action.totals().values()), 13)
        self.assertEqual([f.arbitration_id for f in self.robots["right"].sent], [0x472]*6 + [0x159])

    def test_known_unready_returns_preparation_required_without_enabling_or_fault(self):
        self.robots["left"].ctrl_mode = 0
        self.robots["left"].driver_enabled = [False]*6
        self.open()
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"], report)
        self.assertEqual(report["status"], "preparation_required")
        self.assertFalse(report["fault_latched"])
        self.assertEqual(report["requirements"], ["left_arm_CAN_control_mode_1", "left_arm_six_joint_drivers_enabled"])
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertEqual(self.robots["left"].driver_enabled, [False]*6)
        self.assertTrue(all(not r.gripper_enabled for r in self.robots.values()))
        self.assertEqual(self.robots["left"].ctrl_mode, 0)
        self.assert_queries(0, 0)
        self.assertTrue(self.device.observe()["connected_for_preparation"])

    def test_missing_reply_ignores_sdk_cached_value_and_stops_sequence(self):
        self.open()
        self.robots["left"].no_reply_joint = 3
        start = self.clock.elapsed
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assertTrue(report["fault_latched"])
        self.assertLessEqual(self.clock.elapsed-start, 1.6)
        self.assert_queries(3, 0)
        repeated = self.device.inspect_joint_limits()
        self.assertFalse(repeated["ok"])
        self.assertEqual(repeated["hardware_commands_sent"], 0)
        self.assert_queries(3, 0)

    def test_stale_reply_cannot_confirm_but_valid_new_reply_can(self):
        self.open()
        def stale(robot, joint):
            robot.callback(robot.can.Message(arbitration_id=0x473, is_extended_id=False,
                timestamp=self.clock.time()-1., data=reply_bytes(joint)))
        self.robots["left"].on_query = stale
        report = self.device.inspect_joint_limits()
        self.assertTrue(report["ok"], report)
        for row in report["joint_limits"]["left"].values():
            self.assertEqual(len(row["response_evidence"]["ignored_stale_frames"]), 1)
            self.assertEqual(len(row["response_evidence"]["response_frames"]), 1)

    def test_only_stale_reply_times_out_once(self):
        self.open()
        def old(frame):
            frame.timestamp -= 1
            return frame
        self.robots["left"].reply_transform = old
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assert_queries(1, 0)

    def test_identical_reply_preserves_both_records_without_extra_queries(self):
        self.open()
        self.robots["left"].extra_reply = True
        report = self.device.inspect_joint_limits()
        self.assertTrue(report["ok"], report)
        window = report["joint_limits"]["left"]["1"]["response_evidence"]
        self.assertEqual(len(window["response_frames"]), 1)
        self.assertEqual(len(window["identical_duplicate_frames"]), 1)
        self.assertEqual(len(window["rejected_frames"]), 0)
        self.assertFalse(window["active"])
        self.assert_queries(6, 6)

    def test_identical_reply_after_96_microseconds_is_idempotent(self):
        self.open()
        robot = self.robots['left']; robot.extra_reply = True
        def later(frame):
            self.clock.elapsed += .000096
            frame.timestamp = self.clock.time()
            return frame
        robot.extra_reply_transform = later
        report = self.device.inspect_joint_limits()
        self.assertTrue(report['ok'], report)
        self.assert_queries(6, 6)
        self.assertEqual(report['actuator_commands_sent'], 0)
        for row in report['joint_limits']['left'].values():
            w = row['response_evidence']
            self.assertEqual(w['response_frames'][0]['payload_hex'], w['identical_duplicate_frames'][0]['payload_hex'])
            self.assertGreater(w['identical_duplicate_frames'][0]['timestamp'], w['response_frames'][0]['timestamp'])

    def test_duplicate_conflict_flags_late_and_flood_still_fault(self):
        def late(case, frame):
            case.clock.elapsed += 1.001
            frame.timestamp = case.clock.time()
        for kind in ('conflict', 'joint', 'flags', 'late', 'flood'):
            with self.subTest(kind=kind):
                case = PairLimitsTests('runTest'); case.setUp()
                try:
                    case.open(); robot = case.robots['left']; robot.extra_reply = True
                    def transform(frame):
                        if kind == 'conflict': frame.data[6] ^= 1
                        if kind == 'joint': frame.data[0] = 2
                        if kind == 'flags': frame.is_error_frame = True
                        if kind == 'late': late(case, frame)
                        return frame
                    robot.extra_reply_transform = transform
                    if kind == 'flood': robot.extra_reply_count = 32
                    r = case.device.inspect_joint_limits()
                    self.assertFalse(r['ok'], r); case.assert_queries(1, 0)
                finally: case.doCleanups()

    def test_reply_on_peer_connection_is_rejected(self):
        self.open()
        self.robots["left"].reply_side = self.robots["right"]
        self.assertFalse(self.device.inspect_joint_limits()["ok"])
        self.assert_queries(1, 0)

    def test_malformed_reply_types_flags_joint_range_and_timestamp_fail(self):
        transforms = (
            lambda f: setattr(f, "timestamp", float("nan")),
            lambda f: setattr(f, "timestamp", self.clock.time()+1.),
            lambda f: setattr(f, "is_extended_id", True),
            lambda f: setattr(f, "is_rx", False),
            lambda f: setattr(f, "dlc", 7),
            lambda f: f.data.__setitem__(0, 6),
            lambda f: f.data.__setitem__(7, 1),
            lambda f: setattr(f, "data", bytearray(reply_bytes(1, minimum=10, maximum=10))),
        )
        # Each subcase constructs only offline fake objects; no shared live retry.
        for change in transforms:
            with self.subTest(change=change):
                case = PairLimitsTests("test_reply_on_peer_connection_is_rejected")
                case.setUp()
                try:
                    case.open()
                    case.robots["left"].reply_transform = lambda frame: (change(frame) or frame)
                    result = case.device.inspect_joint_limits()
                    self.assertFalse(result["ok"], result)
                    self.assertTrue(result["fault_latched"])
                    json.dumps(result, allow_nan=False)
                    case.assert_queries(1, 0)
                finally:
                    case.doCleanups()

    def test_partial_send_failure_is_faulted_and_not_retried(self):
        self.open()
        callbacks = {s: r.callback for s,r in self.robots.items()}
        self.robots["left"].send_error = OSError("uncertain CAN query")
        self.robots["left"].swallow_error = True
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assertEqual(report["joint_limit_queries_attempted"], 1)
        self.assertEqual(report["joint_limit_queries_sent"], 0)
        self.assertEqual(len(self.robots["left"].calls), 1)
        self.assert_queries(0, 0)
        for side, robot in self.robots.items():
            self.assertIs(robot.callback, callbacks[side])

    def test_duplicate_query_is_blocked_after_one_physical_request(self):
        self.open()
        self.robots["left"].duplicate = True
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assert_queries(1, 0)

    def test_wrong_payload_or_peer_sender_is_zero_tx(self):
        self.open()
        self.robots["left"].frame_transform = lambda frame: (
            self.robots["right"].set_motion_mode("j") or frame)
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assertTrue(report["guard_violations"])
        self.assert_queries(0, 0)

    def test_guard_delay_rechecks_real_feedback_before_opening_window(self):
        self.open()
        previous = copy.deepcopy(self.device._previous)
        def guard():
            if self.device._action.ticket is not None:
                self.clock.sleep(.2)
                self.hook = lambda robot, state: state.update(copy.deepcopy(previous[robot.side]))
        self.guard_hook = guard
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assert_queries(0, 0)
        self.assertEqual(report["joint_limits"]["left"], {})

    def test_peer_drift_before_query_is_zero_tx(self):
        self.open()
        def guard():
            if self.device._action.ticket is not None:
                self.hook = lambda robot, state: state["joints_rad"].__setitem__(0, .004) if robot.side == "right" else None
        self.guard_hook = guard
        self.assertFalse(self.device.inspect_joint_limits()["ok"])
        self.assert_queries(0, 0)

    def test_mode_or_joint_enable_change_after_first_reply_stops_remaining_queries(self):
        for changed in ("ctrl_mode", "driver_enabled"):
            with self.subTest(changed=changed):
                case = PairLimitsTests("test_reply_on_peer_connection_is_rejected")
                case.setUp()
                try:
                    case.open()
                    def alter(robot, joint):
                        if changed == "ctrl_mode":
                            case.robots["right"].ctrl_mode = 2
                        else:
                            case.robots["right"].driver_enabled[2] = False
                    case.robots["left"].on_query = alter
                    result = case.device.inspect_joint_limits()
                    self.assertFalse(result["ok"])
                    self.assertTrue(result["fault_latched"])
                    case.assert_queries(1, 0)
                finally:
                    case.doCleanups()

    def test_late_duplicate_after_closed_window_stops_before_next_query(self):
        self.open()
        def journal(event, data):
            self.events.append((event, data))
            if event == "pair_joint_limit_reply":
                robot = self.robots[data["side"]]
                robot.callback(robot.can.Message(arbitration_id=0x473, is_extended_id=False,
                    timestamp=self.clock.time(), data=reply_bytes(data["joint_index"])))
        self.device._action.journal = journal
        report = self.device.inspect_joint_limits()
        self.assertFalse(report["ok"])
        self.assertTrue(report["fault_latched"])
        self.assert_queries(1, 0)

    def test_prepared_jaw_target_is_preserved_and_enforced_during_queries(self):
        self.open()
        self.device.prepare_gripper("right")
        self.robots["right"].sent.clear()
        targets = copy.deepcopy(self.device._preparation.prepared_targets)
        def guard():
            if self.device._action.ticket is not None:
                self.robots["right"].width += .0011
        self.guard_hook = guard
        self.assertFalse(self.device.inspect_joint_limits()["ok"])
        self.assertEqual(self.device._preparation.prepared_targets, targets)
        self.assert_queries(0, 0)


class RealSDKPairLimitsTests(unittest.TestCase):
    def test_vendor_nonblocking_getter_twelve_requests_two_connections_and_exact_raw_windows(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        clock, sent, created, bindings = Clock(), [], [], {}
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper_x"
        class FakeCAN:
            def __init__(self, channel):
                self.channel = channel
            def recv(self, timeout=None):
                time.sleep(.001)
                return None
            def shutdown(self):
                pass
            def send(self, frame, timeout=None):
                sent.append((self.channel, copy.deepcopy(frame)))
                if frame.arbitration_id == 0x472:
                    reply = can.Message(arbitration_id=0x473, is_extended_id=False,
                        timestamp=clock.time(), data=reply_bytes(frame.data[0]))
                    bindings[self.channel]._ctx.comm.get_callback()(reply)
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            bindings[config["comm"]["can"]["channel"]] = robot
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            state = healthy_arm(clock.time())
            state["arm_status"].update(ctrl_mode=1, mode_feedback=1, teach_status=0, motion_status=0)
            state["gripper"]["width_m"] = .003
            state["gripper"]["foc_status"]["driver_enable_status"] = False
            state["joints_rad"][1:3] = [-.091769, .045658]
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("Physical socket forbidden")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, linear_hold, supervised_actions,
                           single_supervised_actions, pair_device, pair_preparation, pair_limits):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile, lambda *a: None)
            try:
                device.connect_for_preparation()
                self.assertEqual(sent, [])
                report = device.inspect_joint_limits()
                self.assertTrue(report["ok"], report)
                self.assertEqual(len(created), 2)
                self.assertEqual([frame.arbitration_id for _,frame in sent], [0x472]*12)
                self.assertEqual([frame.data[0] for _,frame in sent], list(range(1,7))*2)
                self.assertFalse(device._task_ready)
                for side in takeover.SIDES:
                    self.assertEqual(len(report["joint_limits"][side]), 6)
                    for row in report["joint_limits"][side].values():
                        self.assertEqual(len(row["response_evidence"]["response_frames"]), 1)
                device.observe()
            finally:
                device.close()
            self.assertEqual(len(sent), 12)


if __name__ == "__main__":
    unittest.main()
