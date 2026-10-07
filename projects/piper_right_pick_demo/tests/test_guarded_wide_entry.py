"""Offline wide-envelope tests using the frozen SDK guard and fake buses."""
import copy
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

import test_ros_interruptible_joint_entry as fixtures
import test_guarded_rx_latch as rx_fixtures
import test_guarded_regrasp_entry as regrasp_fixtures
from test_guarded_j5_profile import configured_limits

ROOT = Path(__file__).parents[1]
RUN = ROOT/"runs/cola_on_cup_wide70_20261006_202700"
SPEC = importlib.util.spec_from_file_location("wide_entry_under_test", ROOT/"scripts/ros_guarded_wide_entry.py")
entry = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(entry)
SPEC_CLIENT = importlib.util.spec_from_file_location("wide_client_under_test", ROOT/"scripts/ros_guarded_wide_client.py")
client = importlib.util.module_from_spec(SPEC_CLIENT); SPEC_CLIENT.loader.exec_module(client)
profile = entry.profile


def wide_limits():
    value = configured_limits(); value["gripper_max_m"] = .070
    return value


class VendorFake(rx_fixtures.BusPiper):
    """Only the vendor I/O is fake; ResumePiper's SDK/bus guards run unchanged."""
    def __init__(self, clock):
        super().__init__(clock)
        bus = types.SimpleNamespace(send=self.raw_send)
        success = object()
        self.comm = types.SimpleNamespace(send_bus=bus, recv_bus=bus,
            CAN_STATUS=types.SimpleNamespace(SEND_MESSAGE_SUCCESS=success))
        def send(ident, data, dlc=8, is_extended_id=False):
            frame = rx_fixtures.message(ident, bytes(data), self.clock.time())
            frame.dlc = dlc; frame.is_extended_id = is_extended_id
            self.comm.send_bus.send(frame)
            return success
        self.comm.SendCanMessage = send

    def raw_send(self, frame, *args, **kwargs):
        if self.failure == "jaw_send_failure": raise RuntimeError("Fake jaw send failed")
        self.frames.append((frame.arbitration_id, bytes(frame.data)))
        if self.on_frame: self.on_frame(frame)

    def GripperCtrl(self, raw, effort, code, zero):
        self.comm.SendCanMessage(0x159, struct.pack(">iHBB", raw, effort, code, zero))
        self.jaw_goal = raw/1e6


FrozenSDK = fixtures.base.sdk_class(VendorFake)


class OfflineSDK(FrozenSDK):
    # Bypass only constructor identity/device discovery; run the real guard
    # __init__ and GripperCtrl, with pure assigned feedback and a fake CAN bus.
    def __new__(cls, *args, **kwargs): return object.__new__(cls)
    snapshot = VendorFake.snapshot
    healthy = VendorFake.healthy
    ParseCANFrame = VendorFake.ParseCANFrame


class OfflineTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket", "subprocess.Popen"):
            blocker = patch(name, side_effect=AssertionError("No device/process access"))
            blocker.start(); self.addCleanup(blocker.stop)


class ProfileTests(OfflineTests):
    def test_only_configured_width_expands_and_other_limits_stay_frozen(self):
        config = json.loads((RUN/"entry_config.json").read_text())
        old = json.loads((entry.previous.PARENT_RUN/"entry_config.json").read_text())
        adjusted = copy.deepcopy(config); adjusted["physical_limits"]["gripper_max_m"] = .055
        self.assertEqual(adjusted, old)
        self.assertEqual(profile.checked_limits(config)["gripper_max_m"], .07)
        for key, value in (("gripper_max_m", .070001), ("max_speed_percent", 2),
                ("max_translation_step_m", .030001), ("max_rotation_step_rad", .050001),
                ("max_state_age_s", .100001)):
            bad = copy.deepcopy(config); bad["physical_limits"][key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError): profile.checked_limits(bad)
        self.assertIs(profile.monitor, profile.original.monitor)
        self.assertIs(profile.path_check, profile.original.path_check)
        self.assertEqual(profile.MARGINS, profile.original.MARGINS)

    def make_sdk(self):
        clock = fixtures.Clock()
        piper = profile.sdk_overlay(OfflineSDK, wide_limits())(clock)
        piper.rx_context_provider = lambda: dict(kind="gripper", sequence=1, generation=9,
            origin_raw=piper.q, target_raw=piper.q, jaw_m=piper.jaw, jaw_code=64, token="fake")
        return piper

    def ticket(self, piper, width):
        frame = (0x159, struct.pack(">iHBB", width, 200, 1, 0))
        piper.ticket = dict(thread=threading.get_ident(), expected=[frame], speed=1,
                            in_comm=False, attempted=0, sent=0)
        return frame

    def test_real_frozen_sdk_method_65000_and_70000_encode_without_55mm_clipping(self):
        original_integer = FrozenSDK.GripperCtrl.__globals__["integer"]
        for raw in (65000, 70000):
            piper = self.make_sdk(); self.assertEqual(piper.frames, [])
            expected = self.ticket(piper, raw)
            piper.GripperCtrl(raw, 999, 1, 0)  # Frozen guard always sends exactly200.
            self.assertEqual(piper.frames, [expected])
            self.assertEqual((piper.ticket["attempted"], piper.ticket["sent"]), (1,1))
            self.assertEqual(piper.ticket["expected"], [])
        self.assertIs(FrozenSDK.GripperCtrl.__globals__["integer"], original_integer)
        with self.assertRaises(ValueError): original_integer(65000, 0, 55000, "jaw width")

    def test_sdk_70001_bad_code_zero_and_missing_or_wrong_ticket_send_nothing(self):
        for kind in ("too_wide", "code", "zero", "no_ticket", "wrong_thread", "wrong_payload"):
            piper = self.make_sdk(); self.ticket(piper, 65000)
            values = [65000, 200, 1, 0]
            if kind == "too_wide": values[0] = 70001
            if kind == "code": values[2] = 3
            if kind == "zero": values[3] = 1
            if kind == "no_ticket": piper.ticket = None
            if kind == "wrong_thread": piper.ticket["thread"] = -1
            if kind == "wrong_payload": self.ticket(piper, 64000)
            expected_error = ValueError if kind == "too_wide" else RuntimeError
            with self.subTest(kind=kind), self.assertRaises(expected_error): piper.GripperCtrl(*values)
            self.assertEqual(piper.frames, [])

    def test_raw_width_boundary_and_original_jaw_arm_tracking_margin(self):
        q = [33118,105639,-31835,0,-52597,0]
        for raw in (70000,70001):
            latch = profile.ProfileLatch(wide_limits()); latch.arm_jaw(1,9,q,q,.05439)
            latch.observe(0x2a8, struct.pack(">iHBB",raw,0,64,0),100.,100.001)
            self.assertEqual(latch.first_fault is None,raw==70000)
        for jaw, delta, accepted in ((True,171,True),(True,172,False),(False,300,True),(False,301,False)):
            latch = profile.ProfileLatch(wide_limits())
            (latch.arm_jaw if jaw else latch.arm_joint)(1,9,q,q,.05439)
            moved = list(q); moved[4] += delta
            latch.observe(*rx_fixtures.joint_fragment(4,moved),100.,100.001)
            with self.subTest(jaw=jaw,delta=delta): self.assertEqual(latch.first_fault is None,accepted)


class OpeningTests(fixtures.InterruptibleTests):
    write_review = regrasp_fixtures.ProbeTests.write_review
    review_args = regrasp_fixtures.ProbeTests.review_args

    def setUp(self):
        super().setUp(); self.control.feedback.close()
        tick = patch.object(entry.motion.frozen.overlay.time,"time",side_effect=self.clock.time)
        tick.start(); self.addCleanup(tick.stop)
        self.piper = profile.sdk_overlay(OfflineSDK,wide_limits())(self.clock)
        self.piper.q = [33118,105639,-31835,0,-52597,0]; self.piper.jaw = .05439
        self.node.piper = self.piper
        self.session.update(stage="task",generation=9,generations=[],commissioning_attempted=True,
            held_raw=list(self.piper.q),first_segment_verified=False,regrasp_stage="opening65_ready",
            scope_anchor_raw=list(self.piper.q),probe_attempted=False,retreat_completed=0,
            wide_open_attempted=False,wide_opening_reviews=[])
        def register(name,callback): self.services[name]=callback;return name
        self.control = entry.WideTask(self.node,
            types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
            wide_limits(),fixtures.fk,self.store,self.session,self.path,{"offline":True},lambda:None,
            self.clock,service_factory=register,namespace="/offline_wide",token="wide_offline")
        self.addCleanup(self.control.feedback.close)

    def request(self,width=.065):
        return types.SimpleNamespace(gripper_angle=width,gripper_effort=.2,gripper_code=1,set_zero=0)

    def review_file(self,next_stage):
        path,material,source=self.write_review(next_stage=next_stage)
        material["actual_opening_reviewed"]=True;path.write_text(json.dumps(material))
        return path,material,source

    def approve(self,next_stage):
        self.review_file(next_stage);count=len(self.piper.frames);started=self.clock.monotonic()
        result=self.control.review_opening(*self.review_args())
        self.assertTrue(result[0]);self.assertGreaterEqual(self.clock.monotonic()-started,3.)
        self.assertEqual(len(self.piper.frames),count)

    def test_first_only_65_then_review_blocks_joint_repeat_and_premature70(self):
        self.assertEqual(self.piper.frames,[])
        for width in (.055,.07,.070001):
            with self.assertRaises(RuntimeError):self.control.gripper(self.request(width))
        with self.assertRaises(RuntimeError):self.control.execute(self.message(100))
        result=self.control.gripper(self.request())
        self.assertEqual(self.piper.frames,[(0x159,struct.pack(">iHBB",65000,200,1,0))])
        self.assertEqual(result["phase"],"completed");self.assertTrue(result["jaw_target_reached"])
        self.assertFalse(result["release_verified"])
        self.assertFalse(self.node.adopted)
        for call in (lambda:self.control.gripper(self.request()),lambda:self.control.execute(self.message(100))):
            with self.assertRaises(RuntimeError):call()
        self.assertEqual(len(self.piper.frames),1)

    def test_65_review_may_allow70_once_and_70_review_only_allows_retreat(self):
        self.control.gripper(self.request());old=self.review_args();self.approve("opening70_ready")
        with self.assertRaises(RuntimeError):self.control.review_opening(*old)
        with self.assertRaises(RuntimeError):self.control.execute(self.message(100))
        self.control.gripper(self.request(.07))
        self.assertEqual(self.piper.frames[-1],(0x159,struct.pack(">iHBB",70000,200,1,0)))
        self.review_file("task")
        with self.assertRaises(RuntimeError):self.control.review_opening(*self.review_args())
        self.approve("retreat_ready")
        self.assertEqual(self.session["regrasp_stage"],"retreat_ready")
        self.assertEqual(len(self.session["wide_opening_reviews"]),2)
        self.assertEqual(len(self.piper.frames),2)
        self.assertTrue(self.node.adopted)

    def test_65_review_can_choose_retreat_without_sending70(self):
        self.control.gripper(self.request());self.approve("retreat_ready")
        self.assertEqual(self.session["regrasp_stage"],"retreat_ready")
        self.assertEqual(len(self.piper.frames),1)

    def test_review_requires_new_raw_rgb_actual_opening_and_fresh_healthy_state(self):
        self.control.gripper(self.request());path,material,source=self.review_file("retreat_ready")
        for key in ("actual_opening_reviewed","table_supported","no_visible_hook_or_tension"):
            changed=dict(material);changed[key]=False;path.write_text(json.dumps(changed))
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                self.control.review_opening(*self.review_args())
        path.write_text(json.dumps(material));self.piper.q[4]+=172;self.piper.goal=list(self.piper.q)
        with self.assertRaisesRegex(RuntimeError,"slip"):
            self.control.review_opening(*self.review_args())
        self.assertIsNotNone(self.session["failure"]);self.assertFalse(self.node.adopted)
        self.assertEqual(len(self.piper.frames),1)

    def test_stable_but_not_at65_is_failed_and_cannot_review_or_retry(self):
        snapshot=self.piper.snapshot
        def short():
            state=snapshot()
            if self.piper.frames:state["opening_m"]=.060
            return state
        self.piper.snapshot=short
        with self.assertRaisesRegex(RuntimeError,"opening not reached"):
            self.control.gripper(self.request())
        with self.assertRaises(RuntimeError):self.control.review_opening(*self.review_args())
        with self.assertRaises(RuntimeError):self.control.gripper(self.request())
        self.assertEqual(len(self.piper.frames),1)

    def test_partial_opening_never_retries(self):
        self.piper.failure="jaw_send_failure"
        with self.assertRaisesRegex(RuntimeError,"jaw send failed"):
            self.control.gripper(self.request())
        receipt=self.control.status()["receipts"][0]
        self.assertEqual((receipt["attempted_frames"],receipt["socket_send_returns"]),(1,0))
        with self.assertRaises(RuntimeError):self.control.gripper(self.request())
        self.assertEqual(self.piper.frames,[])


class HandoffTests(OfflineTests):
    def setUp(self):
        super().setUp();self.reviewed=json.loads((RUN/"reviewed_wide.json").read_text())
        boot=entry.guard.predecessor.PARENT_BOOT
        self.parent=json.loads((entry.guard.v1.SESSION_ROOT/entry.previous.release.child_name(boot)).read_text())

    def test_real_failed_release_and_70_query_bind_while_wrong_query_refuses(self):
        self.assertEqual(entry.reviewed_evidence(self.reviewed,self.parent)["opening_m"],.05439)
        for key,value in (("wide_opening_reviewed",False),("configured_gripper_max_mm",100),("initial_opening_m",.07)):
            reviewed=copy.deepcopy(self.reviewed);reviewed[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):entry.reviewed_evidence(reviewed,self.parent)
        reviewed=copy.deepcopy(self.reviewed);reviewed["range_query"]["sha256"]="0"*64
        with self.assertRaisesRegex(RuntimeError,"Range query evidence"):
            entry.reviewed_evidence(reviewed,self.parent)

    def test_eight_parents_unchanged_and_wide_child_is_once_per_boot(self):
        boot=entry.guard.predecessor.PARENT_BOOT;previous=entry.previous
        interior=previous.motion.predecessor
        names=[boot+".json",entry.guard.predecessor.CHILD_NAME,"guarded_task_"+boot+".json",
            interior.predecessor.CHILD_NAME,interior.CHILD_NAME,previous.motion.frozen.child_name(boot),
            previous.motion.child_name(boot),previous.release.child_name(boot)]
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);originals={name:(entry.guard.v1.SESSION_ROOT/name).read_bytes()for name in names}
            for name,data in originals.items():(root/name).write_bytes(data)
            with entry.reserve(boot,self.reviewed,session_root=root)as(_,endpoint,store,session):
                self.assertEqual(session["regrasp_stage"],"opening65_ready")
                self.assertFalse(session["wide_open_attempted"])
                self.assertTrue(session["parent_failure_chain_preserved"])
                self.assertEqual(store.load()["range_query_sha256"],entry.RANGE_QUERY_SHA)
                self.assertEqual(session["prior_release_failure"],self.parent["failure"])
            with self.assertRaisesRegex(RuntimeError,"already exists"):
                with entry.reserve(boot,self.reviewed,session_root=root):pass
            self.assertEqual({name:(root/name).read_bytes()for name in names},originals)


class ClientTests(OfflineTests):
    def setUp(self):
        super().setUp()
        fake=types.SimpleNamespace(Gripper=object)
        patcher=patch.dict(sys.modules,{'piper_msgs':types.ModuleType('piper_msgs'),'piper_msgs.srv':fake})
        patcher.start();self.addCleanup(patcher.stop)
        self.calls=[]
        before=dict(stage='task',regrasp_stage='opening65_ready',phase='idle',active=False,
            stop_latched=False,failure=None,adoption_token='a'*32,generation=9,sequence=0)
        after=dict(before,regrasp_stage='opening65_review',phase='completed',sequence=1,result={'kind':'gripper'})
        self.transport=client.WideTransport.__new__(client.WideTransport)
        self.transport.status=iter([before,before,after]).__next__
        self.transport.observe=lambda:None
        self.transport.master=types.SimpleNamespace(getSystemState=lambda:([],[],[('/piper/right/gripper_srv',[client.frozen.NODE])]))
        def call(*args):self.calls.append(args);return types.SimpleNamespace(status=True,code=15900)
        self.transport.rospy=types.SimpleNamespace(wait_for_service=lambda *a,**k:None,ServiceProxy=lambda *a:call)

    def test_client_sends_one65mm_service_request_without55_clipping(self):
        result=self.transport.gripper(65.)
        self.assertEqual(self.calls,[(.065,.2,1,0)])
        self.assertTrue(result['feedback_stable']);self.assertFalse(result['grasp_verified'])
        original=client.previous.previous.TaskTransport.gripper
        changed=client.WideTransport._gripper
        expected=tuple(70. if type(x)is float and x==55. else
            'Jaw width must be0..70mm'if x=='Jaw width must be0..55mm'else x for x in original.__code__.co_consts)
        self.assertEqual(changed.__code__.co_consts,expected)
        self.assertEqual(changed.__code__.co_code,original.__code__.co_code)
        self.assertEqual(original.__code__.co_consts.count(55.),1)

    def test_client_ambiguous_service_response_does_not_retry(self):
        def lost(*args):self.calls.append(args);raise RuntimeError('response lost')
        self.transport.rospy.ServiceProxy=lambda *a:lost
        with self.assertRaisesRegex(RuntimeError,'response lost'):self.transport.gripper(65.)
        self.assertEqual(self.calls,[(.065,.2,1,0)])


for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith('test_')and _name not in OpeningTests.__dict__:
        setattr(OpeningTests,_name,None)


if __name__=='__main__':unittest.main()
