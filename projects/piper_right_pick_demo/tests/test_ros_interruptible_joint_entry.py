"""Offline assigned-state/transport tests, not physical brake dynamics."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import threading
import types
import unittest
from unittest.mock import patch
from test_ros_joint_stop_probe_entry import Clock,Piper,base,fk,limits

SPEC=importlib.util.spec_from_file_location("interruptible_entry",Path(__file__).parents[1]/"scripts/ros_interruptible_joint_entry.py")
entry=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(entry)


class MovingPiper(Piper):
    def __init__(self,clock):
        super().__init__(clock);self.on_read=None;self.on_frame=None
    def snapshot(self):
        result=super().snapshot()
        if self.on_read:self.on_read(result)
        return result
    def tx(self,frame):
        super().tx(frame)
        if self.on_frame:self.on_frame(frame)
    def JointCtrl(self,*raw):
        for frame in entry.support.frames_for(list(raw))[1:]:self.tx(frame)
        self.targets.append(list(raw));self.goal=list(raw)


class InterruptibleTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket","subprocess.Popen"):
            guard=patch(name,side_effect=AssertionError("No hardware/process operations in offline tests"))
            guard.start();self.addCleanup(guard.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name);self.clock=Clock()
        patcher=patch.object(base,"time",self.clock);patcher.start();self.addCleanup(patcher.stop)
        self.piper=MovingPiper(self.clock)
        self.node=types.SimpleNamespace(piper=self.piper,adopted=True,failed=None,active=False,command_sequence=0,
                                        action_lock=threading.Lock(),rospy_probe=types.SimpleNamespace(is_shutdown=lambda:False))
        self.store=entry.SessionFile(self.path/"session.json").__enter__();self.addCleanup(lambda:self.store.__exit__())
        self.session=dict(pending=None,failure=None,stop_latched=False,held_raw=list(self.piper.q))
        self.services={};self.events=[]
        def register(name,callback):self.services[name]=callback;return name
        self.control=entry.Interruptible(self.node,types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
                       limits(),fk,self.store,self.session,self.path,{"offline":True},lambda:None,self.clock,
                       service_factory=register,namespace="/offline_driver",token="offline_token")
        self.addCleanup(self.control.feedback.close)
    def message(self,delta=1000):
        q=list(self.piper.q);q[1]-=delta
        return types.SimpleNamespace(position=[v*entry.support.RAD_PER_RAW for v in q],velocity=[0.]*6+[1.],effort=[])
    def ask(self):
        state=self.control.status()
        return self.services[state["hold_service"]]()
    def trigger(self,condition,*,threaded=False):
        called=[]
        def hook(s):
            if not called and condition(s):
                called.append(True)
                if threaded:
                    thread=threading.Thread(target=lambda:called.append(self.ask()))
                    thread.start();thread.join(timeout=2)
                    self.assertFalse(thread.is_alive(),"Request callback blocked behind action worker")
                else:called.append(self.ask())
        self.piper.on_read=hook
        return called

    def test_normal_joint_completes_once_with_three_seconds_and_can_continue(self):
        first=self.control.execute(self.message())
        self.assertEqual(first["phase"],"completed");self.assertTrue(first["target_reached"])
        self.assertGreaterEqual(first["stable_window"]["duration_s"],3.)
        self.assertGreaterEqual(first["stable_window"]["new_feedback_groups"],20)
        self.assertEqual(len(self.piper.frames),4)
        self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),8);self.assertEqual(self.control.status()["sequence"],2)
        self.assertIsNone(self.session["pending"])

    def test_external_request_from_other_thread_is_accepted_then_held_by_worker(self):
        initial=self.piper.q[1]
        called=self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260,threaded=True)
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"hold_confirmed");self.assertTrue(result["interruption_demonstrated"])
        self.assertTrue(called[1][0]);self.assertFalse(called[1][1]["hold_confirmed"])
        self.assertEqual(len(self.piper.frames),8);self.assertEqual(len(self.piper.targets),2)
        first,held=self.piper.targets
        self.assertEqual(first[:1]+first[2:],held[:1]+held[2:])
        self.assertTrue(self.control.status()["stop_latched"])
        self.assertFalse(self.node.adopted);self.assertIsNone(self.node.failed)
        with self.assertRaisesRegex(RuntimeError,"unavailable"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),8)

    def test_hold_before_initial_send_cancels_without_frames(self):
        self.trigger(lambda s:self.control.status()["phase"]=="preflight")
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"cancelled_before_dispatch")
        self.assertEqual(self.piper.frames,[]);self.assertFalse(self.control.status()["hold_confirmed"])
        self.assertTrue(self.session["stop_latched"])

    def test_request_during_initial_frames_never_interleaves_hold(self):
        called=[]
        def hook(frame):
            if len(self.piper.frames)==2 and not called:called.append(self.ask())
        self.piper.on_frame=hook
        result=self.control.execute(self.message())
        self.assertTrue(called[0][0]);self.assertEqual(result["phase"],"hold_confirmed")
        self.assertFalse(result["interruption_demonstrated"])
        self.assertEqual([f[0]for f in self.piper.frames],[0x151,0x155,0x156,0x157]*2)

    def test_retired_seq_or_wrong_adoption_cannot_cancel_new_action(self):
        self.control.execute(self.message());old=list(self.services.values())[0]
        called=[]
        def hook(s):
            if self.control.status()["sequence"]==2 and not called:
                called.append(old());called.append(self.control.request_hold(2,"other_process"))
        self.piper.on_read=hook
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"completed")
        self.assertTrue(all(not item[0]for item in called));self.assertEqual(len(self.piper.frames),8)

    def test_request_after_arrival_does_not_send_or_claim_interruption(self):
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and s["raw_q"][1]==self.piper.goal[1])
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"already_completed");self.assertFalse(result["interruption_demonstrated"])
        self.assertEqual(len(self.piper.frames),4)

    def test_partial_initial_send_never_attempts_hold_or_retry(self):
        self.piper.failure="partial"
        with self.assertRaisesRegex(RuntimeError,"CAN send failed"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),1);self.assertEqual(self.control.status()["phase"],"failed")
        self.assertFalse(self.ask()[0]);self.assertIsNone(self.piper.ticket)
        before=len(self.piper.frames)
        with self.assertRaises(RuntimeError):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),before)

    def test_stale_feedback_on_request_blocks_hold_but_latches(self):
        initial=self.piper.q[1]
        def hook(s):
            if self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260 and not self.control.requested():
                self.ask();self.piper.failure="stale"
        self.piper.on_read=hook
        with self.assertRaisesRegex(RuntimeError,"100ms"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4);self.assertEqual(self.control.status()["phase"],"failed")
        self.assertTrue(self.control.status()["stop_latched"])

    def test_cancel_baseline_error_persists_failure(self):
        def hook(s):
            if self.control.status()["phase"]=="preflight"and not self.control.requested():
                self.ask();self.node.rospy_probe.is_shutdown=lambda:True
        self.piper.on_read=hook
        with self.assertRaisesRegex(RuntimeError,"shutdown"):self.control.execute(self.message())
        self.assertEqual(self.piper.frames,[]);self.assertEqual(self.control.status()["phase"],"failed")
        self.assertIsNotNone(json.loads(self.store.path.read_text())["failure"])

    def test_unsupported_speed_axis_direction_and_range_refused_without_admission(self):
        for case in ("speed","axis","away","large","effort","nan"):
            msg=self.message()
            if case=="speed":msg.velocity[-1]=50
            if case=="axis":msg.position[4]+=.01
            if case=="away":msg.position[1]+=math.radians(2)
            if case=="large":msg.position[1]-=math.radians(.001)
            if case=="effort":msg.effort=[0]*6
            if case=="nan":msg.position[0]=math.nan
            with self.subTest(case=case),self.assertRaises(RuntimeError):self.control.execute(msg)
            self.assertEqual(self.piper.frames,[]);self.assertEqual(self.control.status()["sequence"],0)
            self.assertIsNotNone(self.control.status()["last_refusal"])

    def test_unselected_feedback_noise_is_normalized_to_frozen_held_targets(self):
        msg=self.message();msg.position[4]+=math.radians(.05)
        original=list(self.session["held_raw"])
        result=self.control.execute(msg)
        self.assertEqual(result["phase"],"completed")
        self.assertEqual(self.piper.targets[0][4],original[4])
        self.assertNotEqual(self.control.status()["requested_raw"][4],self.control.status()["target_raw"][4])

    def test_duplicate_stop_is_not_reissued(self):
        initial=self.piper.q[1];responses=[]
        def hook(s):
            if self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260 and not responses:
                responses.extend([self.ask(),self.ask()])
        self.piper.on_read=hook
        self.control.execute(self.message())
        self.assertTrue(responses[0][0]);self.assertFalse(responses[1][0]);self.assertEqual(len(self.piper.frames),8)

    def test_hold_has_its_own_twenty_second_observation_deadline(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        original=self.control.wait_arrival;observed=[]
        def observe(*a,**kw):
            if not kw["allow_request"]:
                observed.append(kw["deadline"]-self.clock.monotonic())
            return original(*a,**kw)
        self.control.wait_arrival=observe
        self.control.execute(self.message())
        self.assertEqual(observed,[20.])

    def test_cancelled_before_dispatch_confirmation_failure_is_latched(self):
        original=self.control.baseline;calls=[]
        def baseline():
            calls.append(1)
            if len(calls)>1:raise RuntimeError("Offline cancelled hold stability failure")
            state=original();self.ask();return state
        self.control.baseline=baseline
        with self.assertRaisesRegex(RuntimeError,"cancelled hold stability"):self.control.execute(self.message())
        self.assertEqual(self.piper.frames,[]);self.assertEqual(self.control.status()["phase"],"failed")
        self.assertIsNotNone(self.store.load()["failure"])

    def test_partial_hold_never_retries_or_sends_legacy_stop(self):
        initial=self.piper.q[1]
        def hook(s):
            if self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260 and not self.control.requested():
                self.ask();self.piper.failure="partial"
        self.piper.on_read=hook
        with self.assertRaisesRegex(RuntimeError,"CAN send failed"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),5);self.assertIsNone(self.piper.ticket)
        self.assertEqual([r["attempted_frames"]for r in self.control.status()["receipts"]],[4,2])
        self.assertFalse(self.ask()[0]);self.assertEqual(len(self.piper.frames),5)


if __name__=="__main__":unittest.main()
