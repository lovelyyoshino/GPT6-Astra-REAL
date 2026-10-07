"""Loaded adapter contracts over synthetic RX and FakeCAN, never object proof.

The prior joint cache is the explicitly seeded unit-test precondition inherited
from JointFixture. Grasp candidates/retention are produced by actual device APIs.
"""
import copy
import math
import struct
import time
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import (joint_path, arms, takeover, linear_hold, supervised_actions,
                         single_supervised_actions, pair_device, pair_joint_adapter)
from robot_tools.retention_receipt import grasp_body_anchor
from test_joint_path import context as context_fixture, loaded_joint_context, RAW
from test_pair_joint_adapter import JointFixture
from test_backend import PROFILE
from test_execution import Clock, healthy_arm


class LoadedAdapterTests(JointFixture):
    def setUp(self):
        super().setUp()
        scope = context_fixture()["identity"]
        self.identities = {side: {"episode_id": side+"-episode", "arm": side,
            "run_id": scope["run_id"], "owner": scope["owner"], "epoch": scope["epoch"],
            "object_id": "strip" if side == "left" else "plug"} for side in ("left", "right")}
        for side in ("left", "right"):
            self.robots[side].accept = False
            before = len(self.robots[side].sent)
            def contact(robot, state, *, selected=side, index=before):
                if robot.side == selected and any(f.arbitration_id == 0x159 for f in robot.sent[index:]):
                    robot.width = .048
                    state["gripper"]["width_m"] = .048
            self.hook = contact
            probe = self.device.execute_gripper_probe(side, .0455)
            self.assertTrue(probe["ok"], probe.get("errors"))
            self.assertEqual(probe["contact_observation"]["outcome"], "settled_contact_candidate")
            retained = self.device.retain_grasp(side, identity=self.identities[side],
                probe_event_id="probe-"+side, probe_trace_sha256=probe["candidate_probe"]["trace_sha256"],
                deadline_at=self.clock.time()+300)
            self.assertTrue(retained["ok"], retained)
            self.hook = None
            self.robots[side].accept = True
        self.original = copy.deepcopy(self.device.grasp_states)
        self.first_context, _ = self.make_context()

    def loaded(self, event="loaded-1", operation="transport", delta=.005):
        ctx = copy.deepcopy(self.first_context)
        ctx["identity"]["worker_id"] = event
        origin = self.device.observe_joint(ctx["identity"])
        ctx.update(origin=origin, current=copy.deepcopy(origin), origin_sha256=joint_path.evidence_sha256(origin),
                   cached_target=self.device.joint_binding("right")["cached_target"])
        target = origin["arms"]["right"]["joints_rad"][:]
        target[5] += delta
        ctx, target = loaded_joint_context(ctx, target, operation=operation)
        loaded = ctx["loaded_context"]
        loaded["event_id"] = event
        for role, side in (("worker", "right"), ("peer", "left")):
            record = self.device.grasp_states[side]
            loaded[role] = {"identity": record["identity"], "revision": 2,
                "probe_event_id": record["probe_event_id"], "probe_trace_sha256": record["trace_sha256"],
                "requested_width_m": record["requested_width_m"], "original_anchor": record["original_anchor"],
                "local_anchor": grasp_body_anchor(record)}
        self.rehash(ctx)
        return ctx, target

    @staticmethod
    def rehash(ctx):
        evidence = ctx["geometry"]["evidence"]
        evidence["loaded_context_sha256"] = joint_path.evidence_sha256(ctx["loaded_context"])
        ctx["geometry"]["source"]["sha256"] = joint_path.evidence_sha256(evidence)

    def run_loaded(self, ctx, target, *, event=None, operation=None):
        return self.device.execute_joint("right", target, context=ctx,
            event_id=event or ctx["loaded_context"]["event_id"],
            operation=operation or ctx["loaded_context"]["operation"], deadline_at=self.clock.time()+60)

    def confirm(self, result):
        return self.device.confirm_loaded_response("right", identity=self.identities["right"],
            action_event_id=result["loaded_receipt"]["action_event_id"],
            plan_sha256=result["loaded_receipt"]["plan_sha256"])

    def test_transport_preserves_original_and_peer_and_requires_visual_before_any_more_tx(self):
        ctx, target = self.loaded()
        result = self.run_loaded(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155, 0x156, 0x157])
        self.assertEqual(self.ids("left"), [0x159])
        self.assertEqual(result["hardware_commands_sent"], 4)
        self.assertTrue(result["loaded_response_pending_visual"])
        self.assertFalse(result["contact_support_verified"])
        self.assertIsNone(result["object_progress_measurement"])
        self.assertIsNone(result["physical_stop_verified"])
        record = self.device.grasp_states["right"]
        self.assertEqual(record["status"], "loaded_pending_visual")
        self.assertEqual(record["original_anchor"], self.original["right"]["original_anchor"])
        self.assertEqual(self.device.grasp_states["left"], self.original["left"])
        self.assertGreater(abs(record["local_anchor"]["joints_rad"][5]-record["original_anchor"]["joints_rad"][5]), .003)
        self.assertEqual(result["loaded_receipt"]["local_anchor"], record["local_anchor"])
        self.device.observe()  # New local body anchor is observed, not original-body stasis.
        before = (len(self.ids()), len(self.ids("left")))
        for call in (lambda: self.device.release_gripper_probe("right", .052),
                     lambda: self.device.execute_gripper_probe("left", .044),
                     lambda: self.device.execute("left", "gripper", .052),
                     lambda: self.device.inspect_joint_limits(),
                     lambda: self.run_loaded(ctx, target)):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual((len(self.ids()), len(self.ids("left"))), before)
        confirmed = self.confirm(result)
        self.assertTrue(confirmed["ok"], confirmed)
        self.assertEqual(confirmed["hardware_commands_sent"], 0)
        self.assertEqual(self.device.grasp_states["right"]["status"], "retained_local")
        self.assertIsNone(self.device.grasp_states["right"]["loaded_pending"])
        ctx2, target2 = self.loaded(event="loaded-2", operation="insert_segment", delta=.001)
        second = self.run_loaded(ctx2, target2)
        self.assertTrue(second["ok"], second.get("errors"))
        self.assertEqual(len(self.device.grasp_states["right"]["loaded_history"]), 2)
        self.assertEqual(self.device.grasp_states["right"]["original_anchor"], self.original["right"]["original_anchor"])
        self.assertEqual(self.sdk.AgxArmFactory.create_arm.call_count, 2)

    def test_live_episode_anchor_and_probe_binding_cannot_be_rehashed_into_permission(self):
        ctx, target = self.loaded()
        for role, field, value in (("worker", "probe_event_id", "other-probe"),
                ("peer", "probe_trace_sha256", "a"*64),
                ("worker", "requested_width_m", .044),
                ("worker", "local_anchor", {**ctx["loaded_context"]["worker"]["local_anchor"], "width_m": .047})):
            changed = copy.deepcopy(ctx)
            changed["loaded_context"][role][field] = value
            self.rehash(changed)
            with self.subTest(role=role, field=field), self.assertRaises(RuntimeError):
                self.run_loaded(changed, target)
        with self.assertRaises(ValueError):
            self.run_loaded(ctx, target, event="another-event")
        self.assertEqual(self.ids(), [0x159])
        self.assertEqual(self.ids("left"), [0x159])

    def test_dual_jaw_slip_during_right_motion_faults_without_reanchoring(self):
        ctx, target = self.loaded()
        def slip(robot, state):
            if robot.side == "right" and any(f.arbitration_id == 0x155 for f in robot.sent):
                state["gripper"]["width_m"] = .048501
        self.hook = slip
        result = self.run_loaded(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155])
        self.assertIsNotNone(self.device._fault)
        self.assertNotIn("local_anchor", self.device.grasp_states["right"])
        self.assertIsNone(result["loaded_receipt"])
        self.assertEqual(self.device.grasp_states["left"], self.original["left"])

    def test_peer_body_motion_is_never_delegated_to_worker_runner(self):
        ctx, target = self.loaded()
        def move_peer(robot, state):
            if robot.side == "left" and any(f.arbitration_id == 0x155 for f in self.robots["right"].sent):
                state["joints_rad"][0] += .0031
        self.hook = move_peer
        result = self.run_loaded(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155])
        self.assertEqual(self.ids("left"), [0x159])
        self.assertIsNone(result["loaded_receipt"])

    def test_before_first_returned_frame_worker_keeps_static_body_anchor(self):
        ctx, target = self.loaded()
        def before_send():
            action = self.device._action
            if action.ticket is not None and not action.joint_executor.frames:
                self.joints["right"][5] = self.original["right"]["original_anchor"]["joints_rad"][5]+.004
        self.guard_hook = before_send
        result = self.run_loaded(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x159])
        self.assertIsNone(result["loaded_receipt"])
        self.assertIsNotNone(self.device._fault)

    def test_partial_send_never_creates_local_anchor_or_visual_confirmation(self):
        ctx, target = self.loaded()
        self.robots["right"].partial = True
        result = self.run_loaded(ctx, target)
        self.assertFalse(result["ok"])
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155, 0x156])
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertNotIn("local_anchor", self.device.grasp_states["right"])
        self.assertEqual(self.device.grasp_states["right"]["original_anchor"], self.original["right"]["original_anchor"])

    def test_wrong_plan_confirmation_faults_and_preserves_pending_history(self):
        ctx, target = self.loaded()
        result = self.run_loaded(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        before = copy.deepcopy(self.device.grasp_states)
        confirm = self.device.confirm_loaded_response("right", identity=self.identities["right"],
            action_event_id="loaded-1", plan_sha256="0"*64)
        self.assertFalse(confirm["ok"])
        self.assertIsNotNone(self.device._fault)
        self.assertEqual(self.device.grasp_states, before)
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155, 0x156, 0x157])

    def test_receipt_journal_delay_cannot_hide_worker_jaw_slip(self):
        ctx, target = self.loaded()
        old = self.device._action.journal
        def journal(event, data):
            old(event, data)
            if event == "pair_loaded_segment_observed":
                self.clock.sleep(.06)
                self.robots["right"].width += .000501
        self.device._action.journal = journal
        result = self.run_loaded(ctx, target)
        self.assertFalse(result["ok"])
        self.assertIsNone(result["loaded_receipt"])
        self.assertNotIn("local_anchor", self.device.grasp_states["right"])
        self.assertIsNotNone(self.device._fault)
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155, 0x156, 0x157])

    def test_visual_confirmation_reads_new_feedback_and_cannot_reanchor_drift(self):
        ctx, target = self.loaded()
        result = self.run_loaded(ctx, target)
        self.assertTrue(result["ok"], result.get("errors"))
        before = copy.deepcopy(self.device.grasp_states["right"])
        self.joints["right"][5] += .0031
        confirmation = self.confirm(result)
        self.assertFalse(confirmation["ok"])
        self.assertIsNotNone(self.device._fault)
        self.assertEqual(self.device.grasp_states["right"], before)
        self.assertEqual(self.ids(), [0x159, 0x151, 0x155, 0x156, 0x157])

    def test_release_after_confirmed_segment_uses_local_body_and_preserves_original_probe(self):
        ctx, target = self.loaded()
        moved = self.run_loaded(ctx, target)
        self.assertTrue(moved["ok"], moved.get("errors"))
        self.assertTrue(self.confirm(moved)["ok"])
        local = copy.deepcopy(self.device.grasp_states["right"]["local_anchor"])
        observation = self.device.observe_grasp("right", identity=self.identities["right"], probe_event_id="probe-right")
        self.assertTrue(observation["ok"], observation)
        self.assertEqual(observation["measurement"]["anchor"], local)
        opened = self.device.release_gripper_probe("right", .052)
        self.assertTrue(opened["ok"], opened.get("errors"))
        self.assertEqual(self.device.grasp_states["right"]["status"], "release_opened")
        self.assertEqual(opened["release_measurement"]["anchor"], local)
        record = self.device.grasp_states["right"]
        self.assertEqual(record["original_anchor"], self.original["right"]["original_anchor"])
        self.assertEqual(record["probe_event_id"], "probe-right")
        observed = self.device.observe_release("right", identity=self.identities["right"],
            probe_event_id="probe-right", release_trace_sha256=record["release_opening"]["trace_sha256"])
        self.assertTrue(observed["ok"], observed)
        self.assertEqual(observed["measurement"]["anchor"], local)
        self.assertEqual(self.device.grasp_states["left"], self.original["left"])


class NativeSDKLoadedAdapterTests(unittest.TestCase):
    def test_vendor_two_candidates_retained_then_single_loaded_target_and_zero_tx_confirmation(self):
        sdk = arms._load_sdk(PROFILE["sdk_path"])
        import can
        from pyAgxArm.protocols.can_protocol.drivers.core import driver_context
        from pyAgxArm.protocols.can_protocol.msgs.piper.default.feedback.arm_feedback_status import ArmMsgFeedbackStatus
        clock, sent, created, bindings = Clock(), [], [], {}
        clock.monotonic = lambda: clock.elapsed
        profile = copy.deepcopy(PROFILE)
        for cfg in profile["arms"].values():
            cfg["model"] = "piper_x"
        channels = {side: cfg["channel"] for side, cfg in profile["arms"].items()}
        joints = {channel: RAW[:] for channel in channels.values()}
        widths = {channel: .05 for channel in channels.values()}
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
                if 0x155 <= frame.arbitration_id <= 0x157:
                    index = 2*(frame.arbitration_id-0x155)
                    joints[self.channel][index:index+2] = struct.unpack(">ii", bytes(frame.data))
                elif frame.arbitration_id == 0x159:
                    widths[self.channel] = .048
                clock.sleep(.0001)
        factory = sdk.AgxArmFactory.create_arm
        def create(config):
            robot = factory(config)
            robot.get_fps = lambda: 100.
            bindings[id(robot)] = config["comm"]["can"]["channel"]
            created.append(robot)
            return robot
        def snapshot(robot, jaw):
            state = healthy_arm(clock.time())
            state["arm_status"] = arms._plain(ArmMsgFeedbackStatus(ctrl_mode=1, teach_status=0,
                mode_feedback=1, motion_status=0, arm_status=0, err_code=0))
            channel = bindings[id(robot)]
            state["joints_rad"] = [math.radians(v/1000) for v in joints[channel]]
            state["gripper"]["width_m"] = widths[channel]
            return state
        with ExitStack() as stack:
            stack.enter_context(patch("socket.socket", side_effect=AssertionError("No real sockets")))
            stack.enter_context(patch.object(arms, "_preflight"))
            for module in (arms, takeover, linear_hold, supervised_actions,
                           single_supervised_actions, pair_device, pair_joint_adapter):
                stack.enter_context(patch.object(module, "time", clock))
            stack.enter_context(patch.object(driver_context, "time", SimpleNamespace(
                monotonic=clock.monotonic, time=clock.time, sleep=time.sleep)))
            stack.enter_context(patch.object(can.interface, "Bus", side_effect=lambda **kw: FakeCAN(kw["channel"])))
            stack.enter_context(patch.object(sdk.AgxArmFactory, "create_arm", side_effect=create))
            stack.enter_context(patch.object(arms, "snapshot", side_effect=snapshot))
            device = pair_device.GuardedPairDevice(profile, lambda *args: None)
            try:
                device.open()
                self.assertEqual(sent, [])
                ctx = context_fixture()
                ctx["identity"].update({k: v for k, v in device.joint_binding("right").items() if k != "cached_target"})
                ctx["identity"]["worker_id"] = "native-loaded"
                for side in ("left", "right"):
                    identity = {"episode_id": side+"-episode", "arm": side,
                        **{key: ctx["identity"][key] for key in ("run_id", "owner", "epoch")},
                        "object_id": side+"-object"}
                    probe = device.execute_gripper_probe(side, .0455)
                    self.assertTrue(probe["ok"], probe.get("errors"))
                    retained = device.retain_grasp(side, identity=identity, probe_event_id="probe-"+side,
                        probe_trace_sha256=probe["candidate_probe"]["trace_sha256"], deadline_at=clock.time()+200)
                    self.assertTrue(retained["ok"], retained)
                ctx["origin"] = device.observe_joint(ctx["identity"])
                ctx["current"] = copy.deepcopy(ctx["origin"])
                ctx["origin_sha256"] = joint_path.evidence_sha256(ctx["origin"])
                ctx["cached_target"]["identity"] = copy.deepcopy(ctx["identity"])
                for index, item in enumerate(ctx["cached_target"]["frame_receipts"]):
                    item["returned_at"] = clock.time()-.1+index*.001
                # Unit-test cache precondition only, never advertised as initialization.
                device._joint_cache["right"] = copy.deepcopy(ctx["cached_target"])
                target = ctx["current"]["arms"]["right"]["joints_rad"][:]
                target[5] += .005
                ctx, target = loaded_joint_context(ctx, target, operation="transport")
                ctx["loaded_context"]["event_id"] = "native-loaded"
                for role, side in (("worker", "right"), ("peer", "left")):
                    record = device.grasp_states[side]
                    ctx["loaded_context"][role] = {"identity": record["identity"], "revision": 2,
                        "probe_event_id": record["probe_event_id"], "probe_trace_sha256": record["trace_sha256"],
                        "requested_width_m": record["requested_width_m"], "original_anchor": record["original_anchor"],
                        "local_anchor": grasp_body_anchor(record)}
                LoadedAdapterTests.rehash(ctx)
                result = device.execute_joint("right", target, context=ctx, event_id="native-loaded",
                    deadline_at=clock.time()+60, operation="transport")
                self.assertTrue(result["ok"], result.get("errors"))
                self.assertEqual([(channel, frame.arbitration_id) for channel, frame in sent],
                    [(channels["left"], 0x159), (channels["right"], 0x159)]+
                    [(channels["right"], frame) for frame in (0x151, 0x155, 0x156, 0x157)])
                receipt = result["loaded_receipt"]
                confirmed = device.confirm_loaded_response("right", identity=device.grasp_states["right"]["identity"],
                    action_event_id="native-loaded", plan_sha256=receipt["plan_sha256"])
                self.assertTrue(confirmed["ok"], confirmed)
                self.assertEqual(confirmed["hardware_commands_sent"], 0)
                self.assertEqual(len(sent), 6)
                self.assertEqual(len(created), 2)
                self.assertIsNone(result["accepted"])
                self.assertIsNone(result["physical_stop_verified"])
                self.assertFalse(result["contact_support_verified"])
            finally:
                device.close()
            self.assertEqual(len(sent), 6)


if __name__ == "__main__":
    unittest.main()
