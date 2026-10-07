"""Offline fake transport/state tests; these do not model physical stopping."""
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

SPEC=importlib.util.spec_from_file_location("stop_probe",Path(__file__).parents[1]/"scripts/ros_joint_stop_probe_entry.py")
probe=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(probe)
base=probe.load_base()
Health=base.sdk_class(type("OfflineOriginal",(),{})).healthy


def limits():
    return dict(max_speed_percent=1,max_translation_step_m=.03,max_rotation_step_rad=.05,max_state_age_s=.1,
                workspace_min_m=[-.6,-.6,.05],workspace_max_m=[.6,.6,.65],
                joint_limits_rad=[[a*probe.RAD_PER_RAW,b*probe.RAD_PER_RAW]for a,b in probe.JOINT_LIMITS_RAW],
                gripper_min_m=0.,gripper_max_m=.055)


def fk(q):
    return [.25+.1*q[1],.02*q[0],.25,0.,q[1]+q[2],q[5]]
fk.joint_radius_bounds_m=[.63,.63,.35,.091,.091,0.]


class Clock:
    def __init__(self):self.now=100.
    def time(self):return self.now
    def monotonic(self):return self.now
    def sleep(self,s):self.now+=s


class Piper:
    healthy=Health
    def __init__(self,clock):
        self.clock=clock;self.rx_lock=threading.RLock();self.broken=None;self.ticket=None
        self.q=[69,77806,-59399,0,16711,-7840];self.sequence=0;self.received={}
        self.frames=[];self.targets=[];self.goal=None;self.failure=None;self.step_raw=20
    def snapshot(self):
        self.clock.sleep(.01);self.sequence+=14
        if self.goal is not None:
            self.q[1]=max(self.goal[1],self.q[1]-self.step_raw)
        stamps=[self.clock.time()-.001]*14
        q=[v*probe.RAD_PER_RAW for v in self.q]
        if self.failure=="stale" and self.targets:stamps[0]-=.101
        if self.failure=="old_target" and len(self.targets)>=3:
            self.q[1]=max(self.targets[1][1],self.q[1]-self.step_raw)
            q=[v*probe.RAD_PER_RAW for v in self.q]
        raw=list(self.q)
        if self.failure=="unselected_noise" and self.goal is not None:
            raw[4]+=10;q[4]+=10*probe.RAD_PER_RAW
        self.received={k:(stamp,bytes(8))for k,stamp in zip(base.FEEDBACK_IDS,stamps)}
        return dict(sequence=self.sequence,stamps=stamps,raw_q=raw,q=q,pose=fk(q),
                    opening_m=.03444+(.0006 if self.failure=="jaw"and self.targets else 0),
                    jaw_code=64,ctrl_mode=1,arm_status=0,mode=1,teach_status=0,
                    motion_status=int(self.goal is not None and self.q[1]!=self.goal[1]),
                    fault=0,driver_codes=[64]*6,enabled=[True]*6)
    def tx(self,frame):
        t=self.ticket
        assert t is not None and t["thread"]==threading.get_ident()
        assert t["expected"][0]==frame
        t["attempted"]+=1
        if self.failure=="partial" and t["attempted"]==2:
            self.broken="Fake CAN send failed";raise RuntimeError(self.broken)
        t["expected"].pop(0);t["sent"]+=1;self.frames.append(frame)
    def MotionCtrl_2(self,*args):
        assert args==(1,1,1,0,0,0)
        self.tx(probe.frames_for(self.q)[0])
    def JointCtrl(self,*raw):
        for frame in probe.frames_for(list(raw))[1:]:self.tx(frame)
        self.targets.append(list(raw))
        if len(self.targets)==2:self.goal=list(raw)
        else:
            if not(self.failure=="old_target"and len(self.targets)>=3):
                self.q=list(raw);self.goal=None


class StopProbeTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket","subprocess.Popen"):
            guard=patch(name,side_effect=AssertionError("No hardware/process I/O in offline test"))
            guard.start();self.addCleanup(guard.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name);self.clock=Clock()
        clockpatch=patch.object(base,"time",self.clock);clockpatch.start();self.addCleanup(clockpatch.stop)
        self.piper=Piper(self.clock)
        self.node=types.SimpleNamespace(piper=self.piper,adopted=True,failed=None,active=False,command_sequence=0,
                                        action_lock=threading.Lock(),rospy_probe=types.SimpleNamespace(is_shutdown=lambda:False))
        self.store=probe.SessionFile(self.path/"session.json").__enter__();self.addCleanup(lambda:self.store.__exit__())
        self.identity={"boot":"offline","pid":1}
        self.session=dict(identity=self.identity,pending=None,failure=None,static_attempted=False,
                          dynamic_attempted=False,static_passed=False,dynamic_passed=False)
        self.events=[]
        self.subject=probe.Probe(self.node,types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
                                 limits(),fk,self.store,self.session,self.path,self.identity,lambda:None,self.clock)
        self.addCleanup(self.subject.feedback.close)
    def static(self):
        result=self.subject.perform("static")
        self.assertTrue(result[0],result[1]);return result

    def test_static_four_frames_and_three_second_window(self):
        self.static()
        self.assertEqual([f[0]for f in self.piper.frames],[0x151,0x155,0x156,0x157])
        self.assertEqual(self.piper.frames[0][1].hex(),"0101010000000000")
        result=json.loads((self.path/"static_result.json").read_text())
        self.assertGreaterEqual(result["hold_window"]["duration_s"],3.)
        self.assertGreaterEqual(result["hold_window"]["new_feedback_groups"],20)
        self.assertFalse(result["general_stop_qualified"])
        self.assertIsNone(self.piper.ticket)

    def test_dynamic_cannot_run_before_static_or_repeat_after_pass(self):
        self.assertFalse(self.subject.perform("dynamic")[0]);self.assertEqual(self.piper.frames,[])
        self.static();count=len(self.piper.frames)
        self.assertFalse(self.subject.perform("static")[0]);self.assertEqual(len(self.piper.frames),count)
        self.assertTrue(self.subject.perform("dynamic")[0])
        count=len(self.piper.frames)
        self.assertFalse(self.subject.perform("dynamic")[0]);self.assertEqual(len(self.piper.frames),count)

    def test_dynamic_has_real_progress_then_one_axis_overwrite_and_old_goal_separation(self):
        self.static();self.piper.failure="unselected_noise"
        result=self.subject.perform("dynamic");self.assertTrue(result[0],result[1])
        self.assertEqual(len(self.piper.targets),3);self.assertEqual(len(self.piper.frames),12)
        first,initial,hold=self.piper.targets
        self.assertEqual(first[1]-initial[1],1000)
        self.assertGreaterEqual(first[1]-hold[1],250);self.assertGreaterEqual(hold[1]-initial[1],350)
        self.assertEqual(hold[:1]+hold[2:],initial[:1]+initial[2:])
        result=json.loads((self.path/"dynamic_result.json").read_text())
        self.assertGreaterEqual(result["final_distance_from_old_goal_deg"],.2)
        self.assertIn("max_motion_from_dispatch_sample",result)
        self.assertGreaterEqual(result["hold_window"]["duration_s"],3.)

    def test_motion_that_continues_to_old_target_cannot_pass(self):
        self.static();self.piper.failure="old_target"
        ok,message=self.subject.perform("dynamic")
        self.assertFalse(ok);self.assertIsNotNone(self.session["failure"])
        self.assertEqual(len(self.piper.targets),3)
        self.assertFalse(json.loads((self.path/"dynamic_result.json").read_text())["success"])

    def test_partial_transaction_latches_and_never_sends_cleanup_or_retry(self):
        self.piper.failure="partial"
        ok,message=self.subject.perform("static");self.assertFalse(ok)
        self.assertEqual(len(self.piper.frames),1);self.assertIsNone(self.piper.ticket)
        saved=self.store.path.read_bytes()
        self.assertFalse(self.subject.perform("static")[0]);self.assertFalse(self.subject.perform("dynamic")[0])
        self.assertEqual(len(self.piper.frames),1);self.assertEqual(self.store.path.read_bytes(),saved)

    def test_jaw_failure_blocks_later_probe(self):
        self.piper.failure="jaw"
        self.assertFalse(self.subject.perform("static")[0])
        count=len(self.piper.frames)
        self.assertFalse(self.subject.perform("dynamic")[0]);self.assertEqual(len(self.piper.frames),count)

    def test_stale_failure_blocks_later_probe(self):
        self.piper.failure="stale"
        self.assertFalse(self.subject.perform("static")[0])
        count=len(self.piper.frames)
        self.assertFalse(self.subject.perform("dynamic")[0]);self.assertEqual(len(self.piper.frames),count)

    def test_missed_trigger_does_not_send_late_hold(self):
        self.static();self.piper.step_raw=800
        ok,message=self.subject.perform("dynamic");self.assertFalse(ok)
        self.assertIn("Missed interruption",message);self.assertEqual(len(self.piper.targets),2)

    def test_no_motion_does_not_fake_dynamic_success(self):
        self.static();self.piper.step_raw=0
        ok,message=self.subject.perform("dynamic");self.assertFalse(ok)
        self.assertIn("No timely measured motion",message);self.assertEqual(len(self.piper.targets),2)

    def test_small_remaining_distance_is_rejected_before_dynamic_tx(self):
        self.piper.q[1]=599
        self.static();count=len(self.piper.frames)
        ok,message=self.subject.perform("dynamic");self.assertFalse(ok)
        self.assertIn("0.60deg",message);self.assertEqual(len(self.piper.frames),count)

    def test_original_bounds_and_jaw_limits_cannot_be_relaxed(self):
        probe.checked_probe_limits({"physical_limits":limits()})
        for key,value in (("max_speed_percent",50),("max_speed_percent",True),("max_rotation_step_rad",.051),
                          ("max_translation_step_m",.031),("max_state_age_s",.101),("gripper_max_m",.07)):
            config=limits();config[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):probe.checked_probe_limits({"physical_limits":config})

    def test_complete_box_must_fit_even_when_endpoints_cancel(self):
        before=self.piper.snapshot();target=list(before["raw_q"]);target[1]-=1000
        plan=probe.path_check(before,target,limits(),fk)
        self.assertLessEqual(plan["position_bound_m"],.03);self.assertLessEqual(plan["rotation_bound_rad"],.05)
        target[1]-=4000;target[2]+=5000
        with self.assertRaisesRegex(RuntimeError,"joint box"):probe.path_check(before,target,limits(),fk)

    def test_stale_requested_sample_not_rescued_by_fresh_new_sample(self):
        before=self.piper.snapshot();self.clock.sleep(.11)
        with self.assertRaisesRegex(RuntimeError,"100ms"):
            self.subject.send(before["raw_q"],before,"offline",before,before["q"])
        self.assertEqual(self.piper.frames,[])

    def test_busy_probe_and_shutdown_refuse_before_tx(self):
        self.node.action_lock.acquire()
        self.assertFalse(self.subject.perform("static")[0]);self.node.action_lock.release()
        self.node.rospy_probe.is_shutdown=lambda:True
        self.assertFalse(self.subject.perform("static")[0]);self.assertEqual(self.piper.frames,[])

    def test_trend_uses_recent_joint_fragment_time_and_measured_change(self):
        old=self.piper.snapshot();now=copy.deepcopy(old)
        now["stamps"]=[v+.1 for v in old["stamps"]];now["raw_q"][1]-=29
        self.assertIs(probe.Probe.moving_reference([old],now),old)
        now["raw_q"][1]+=1;self.assertIsNone(probe.Probe.moving_reference([old],now))
        now["raw_q"][1]-=100;now["stamps"][4]+=.1
        self.assertIsNone(probe.Probe.moving_reference([old],now))


if __name__=="__main__":unittest.main()
