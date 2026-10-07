"""Offline transport/state assignment tests; not physical stop validation."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import types
import unittest
from unittest.mock import patch
import test_ros_interruptible_joint_entry as fixtures

SPEC=importlib.util.spec_from_file_location("guarded_task",Path(__file__).parents[1]/"scripts/ros_guarded_task_entry.py")
entry=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(entry)


class MultiPiper(fixtures.MovingPiper):
    def __init__(self,clock):
        super().__init__(clock);self.jaw=.03444;self.jaw_goal=None;self.jaw_contact=None
    def snapshot(self):
        if self.goal is not None:
            for i in range(6):
                if i==1:continue
                delta=self.goal[i]-self.q[i]
                self.q[i]+=max(-self.step_raw,min(self.step_raw,delta))
        result=super().snapshot()
        result["motion_status"]=int(self.goal is not None and self.q!=self.goal)
        if self.jaw_goal is not None:
            target=self.jaw_goal if self.jaw_contact is None else max(self.jaw_goal,self.jaw_contact)
            self.jaw+=max(-.001,min(.001,target-self.jaw))
        result["opening_m"]=self.jaw
        return result
    def GripperCtrl(self,raw,effort,code,zero):
        if self.failure=="jaw_send_failure":
            self.ticket["attempted"]+=1;self.broken="Fake jaw send failed"
            raise RuntimeError(self.broken)
        self.tx((0x159,struct.pack(">iHBB",raw,effort,code,zero)))
        self.jaw_goal=raw/1e6


class GuardedTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp();self.control.feedback.close()
        self.piper=MultiPiper(self.clock);self.node.piper=self.piper
        self.session.update(stage="commissioning",generation=0,generations=[],commissioning_attempted=False,
                            held_raw=list(self.piper.q))
        def register(name,callback):self.services[name]=callback;return name
        self.control=entry.GuardedTask(self.node,types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
            fixtures.limits(),fixtures.fk,self.store,self.session,self.path,{"offline":True},lambda:None,self.clock,
            service_factory=register,namespace="/offline_driver",token="guarded_token")
        self.addCleanup(self.control.feedback.close)

    def message(self,delta=800):
        anchor=self.session["held_raw"];scale=1-delta/max(map(abs,anchor))
        return types.SimpleNamespace(position=[round(v*scale)*entry.support.RAD_PER_RAW for v in anchor],
                                     velocity=[0.]*6+[1.],effort=[])

    def commission(self):
        start=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and start-s["raw_q"][1]>=320)
        result=self.control.execute(self.message())
        self.piper.on_read=None
        return result

    def review_resume(self):
        status=self.control.status()
        return self.services[status["resume_service"]]()

    def test_commission_requires_two_actual_axes_then_reviewed_zero_tx_resume(self):
        result=self.commission()
        self.assertTrue(result["coupled_interruption_demonstrated"])
        self.assertEqual(self.piper.targets[1],result["hold_plan_sample"]["raw_q"])
        self.assertEqual(len(self.piper.frames),8);self.assertFalse(self.node.adopted)
        self.assertTrue(self.review_resume()[0]);self.assertTrue(self.node.adopted)
        self.assertEqual(len(self.piper.frames),8);self.assertEqual(self.session["stage"],"task")
        self.assertEqual(self.session["generation"],1)
        self.assertTrue(self.session["generations"][0]["stop_latched"])
        self.assertEqual(self.session["generations"][0]["result_sha256"],entry.sha(self.path/"action_000001_result.json"))
        result=self.control.execute(self.message(300));self.assertEqual(result["phase"],"completed")
        result=self.control.execute(self.message(300));self.assertEqual(result["phase"],"completed")
        self.assertEqual(len(self.piper.frames),16)

    def test_natural_completion_is_not_commissioning_pass(self):
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"completed");self.assertFalse(self.node.adopted)
        self.assertIsNone(self.control.status()["resume_service"])
        with self.assertRaises(RuntimeError):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4)

    def test_only_one_moving_axis_cannot_promote(self):
        # Preserve a valid coupled target, but external hold occurs before J3
        # has shown the required actual motion. This is state evidence failure.
        start=self.piper.q[1]
        original=self.piper.snapshot
        def snapshot():
            goal=self.piper.goal
            if goal is not None and len(self.piper.targets)==1:
                fake=list(goal);fake[2]=self.piper.q[2];self.piper.goal=fake
            s=original();self.piper.goal=goal
            return s
        self.piper.snapshot=snapshot
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and start-s["raw_q"][1]>=320)
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"hold_confirmed")
        self.assertFalse(result["coupled_interruption_demonstrated"])
        self.assertIsNone(self.control.status()["resume_service"])

    def test_zero_tx_cancel_never_promotes(self):
        self.trigger(lambda s:self.control.status()["phase"]=="preflight")
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"cancelled_before_dispatch")
        self.assertEqual(self.piper.frames,[]);self.assertIsNone(self.control.status()["resume_service"])

    def test_bad_commission_ratio_speed_or_magnitude_refused_without_frames(self):
        for kind in ("ratio","speed","large","nan","nominal"):
            msg=self.message()
            if kind=="ratio":msg.position[2]=self.piper.q[2]*entry.support.RAD_PER_RAW
            if kind=="speed":msg.velocity[-1]=50
            if kind=="large":msg=self.message(1100)
            if kind=="nan":msg.position[0]=math.nan
            if kind=="nominal":msg.position[1]=-.01
            with self.subTest(kind=kind),self.assertRaises(RuntimeError):self.control.execute(msg)
            self.assertEqual(self.piper.frames,[])

    def test_wrong_hash_old_generation_and_duplicate_resume_rejected(self):
        self.commission();state=self.control.status();callback=self.services[state["resume_service"]]
        with self.assertRaisesRegex(RuntimeError,"Retired"):
            self.control.resume(state["sequence"],state["generation"],"wrong")
        self.assertTrue(callback()[0]);self.assertEqual(self.session["generation"],1)
        with self.assertRaisesRegex(RuntimeError,"Retired"):callback()
        self.assertEqual(len(self.piper.frames),8)

    def test_result_changed_cannot_resume(self):
        self.commission();path=self.path/"action_000001_result.json"
        path.write_text(path.read_text()+"\n")
        with self.assertRaisesRegex(RuntimeError,"result changed"):self.review_resume()
        self.assertFalse(self.node.adopted);self.assertEqual(len(self.piper.frames),8)

    def test_resume_pose_changed_latches_without_physical_actions(self):
        self.commission();self.piper.q[0]+=200;self.piper.goal=list(self.piper.q)
        with self.assertRaisesRegex(RuntimeError,"slip"):self.review_resume()
        self.assertIsNotNone(self.session["failure"]);self.assertEqual(len(self.piper.frames),8)

    def test_resume_races_active_worker_refused(self):
        self.commission()
        with self.node.action_lock:
            with self.assertRaisesRegex(RuntimeError,"still active"):self.review_resume()
        self.assertEqual(len(self.piper.frames),8)

    def test_partial_initial_and_hold_cannot_resume_or_retry(self):
        self.piper.failure="partial"
        with self.assertRaisesRegex(RuntimeError,"CAN send failed"):self.control.execute(self.message())
        self.assertIsNotNone(self.session["pending"]);self.assertIsNotNone(self.session["failure"])
        self.assertIsNone(self.control.status()["resume_service"])
        self.assertEqual(len(self.piper.frames),1)

    def test_pending_failure_or_partial_receipt_cannot_use_success_resume(self):
        self.commission();original=copy.deepcopy(self.session);state=copy.deepcopy(self.control.state)
        for reason in ("pending","failure","receipt"):
            self.session.clear();self.session.update(copy.deepcopy(original));self.control.state=copy.deepcopy(state)
            if reason=="pending":self.session["pending"]={"unresolved":True}
            if reason=="failure":self.session["failure"]={"error":"unresolved"}
            if reason=="receipt":self.control.state["receipts"][1]["socket_send_returns"]=3
            with self.subTest(reason=reason),self.assertRaises(RuntimeError):self.review_resume()
            self.assertEqual(len(self.piper.frames),8);self.assertFalse(self.node.adopted)

    def test_old_hold_service_cannot_cancel_next_generation(self):
        self.commission();old=self.services[self.control.status()["hold_service"]]
        self.review_resume();responses=[]
        def hook(s):
            if self.control.status()["active"]and not responses:responses.append(old())
        self.piper.on_read=hook
        result=self.control.execute(self.message(300))
        self.assertEqual(result["phase"],"completed");self.assertFalse(responses[0][0])
        self.assertEqual(len(self.piper.frames),12)

    def test_partial_hold_blocks_more_frames(self):
        start=self.piper.q[1]
        def hook(s):
            if self.control.status()["phase"]=="moving"and start-s["raw_q"][1]>=320 and not self.control.requested():
                self.ask();self.piper.failure="partial"
        self.piper.on_read=hook
        with self.assertRaisesRegex(RuntimeError,"CAN send failed"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),5)
        with self.assertRaises(RuntimeError):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),5)

    def test_original_box_remains_after_hold_local_origin(self):
        origin=self.control.read();target=list(origin["q"]);target[1]-=.01
        self.control.original=(origin,target)
        local=copy.deepcopy(origin);local["q"][0]+=.0029
        s=copy.deepcopy(local);s["q"][0]+=.0029
        with self.assertRaisesRegex(RuntimeError,"original probe joint box"):
            self.control.monitor(s,local,local["q"])

    def test_stale_hold_does_not_send_second_transaction(self):
        start=self.piper.q[1]
        def hook(s):
            if self.control.status()["phase"]=="moving"and start-s["raw_q"][1]>=320 and not self.control.requested():
                self.ask();self.piper.failure="stale"
        self.piper.on_read=hook
        with self.assertRaisesRegex(RuntimeError,"100ms"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4)

    def jaw_request(self,width=.05):
        return types.SimpleNamespace(gripper_angle=width,gripper_effort=.2,gripper_code=1,set_zero=0)

    def task_stage(self):
        # Local fake fixture only. Real promotion has its own commissioning tests.
        self.session["stage"]="task";self.control.state["stage"]="task"

    def test_jaw_single_frame_fresh_stable_and_next_J_allowed(self):
        self.task_stage();result=self.control.gripper(self.jaw_request())
        self.assertEqual(result["phase"],"completed");self.assertEqual(result["kind"],"gripper")
        self.assertEqual(self.piper.frames,[(0x159,struct.pack(">iHBB",50000,200,1,0))])
        self.assertTrue(result["jaw_target_reached"]);self.assertFalse(result["grasp_verified"])
        self.assertGreaterEqual(result["stable_window"]["duration_s"],3.)
        self.assertGreaterEqual(result["stable_window"]["new_feedback_groups"],20)
        self.assertIsNone(self.control.status()["hold_service"])
        self.assertEqual(self.session["jaw_command_target_m"],.05)
        result=self.control.execute(self.message(300));self.assertEqual(result["phase"],"completed")
        self.assertEqual(len(self.piper.frames),5);self.assertEqual(self.control.status()["sequence"],2)

    def test_contact_before_jaw_goal_is_stable_but_not_target_or_grasp_verified(self):
        self.task_stage();self.piper.jaw_contact=.018
        result=self.control.gripper(self.jaw_request(.006))
        self.assertEqual(result["phase"],"completed");self.assertFalse(result["jaw_target_reached"])
        self.assertAlmostEqual(result["jaw_width_error_m"],.012)
        self.assertFalse(result["grasp_verified"]);self.assertFalse(result["release_verified"])
        self.assertEqual(len(self.piper.frames),1)

    def test_jaw_during_commissioning_action_hold_or_failure_zero_frames(self):
        with self.assertRaisesRegex(RuntimeError,"commissioning"):self.control.gripper(self.jaw_request())
        self.task_stage()
        with self.node.action_lock:
            with self.assertRaisesRegex(RuntimeError,"active"):self.control.gripper(self.jaw_request())
        for key,value in (("stop_latched",True),("pending",{"unresolved":True}),("failure",{"error":"fault"})):
            old=self.session[key];self.session[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.control.gripper(self.jaw_request())
            self.session[key]=old
        self.assertEqual(self.piper.frames,[])

    def test_jaw_bad_parameters_and_unenabled_feedback_zero_frames(self):
        self.task_stage()
        for name,value in (("gripper_angle",.055001),("gripper_angle",-.001),("gripper_angle",math.nan),
                           ("gripper_effort",.5),("gripper_code",0),("gripper_code",3),("set_zero",174)):
            req=self.jaw_request();setattr(req,name,value)
            with self.subTest(name=name,value=value),self.assertRaises(RuntimeError):self.control.gripper(req)
        original=self.piper.snapshot
        def disabled():
            s=original();s["jaw_code"]=0;return s
        self.piper.snapshot=disabled
        with self.assertRaisesRegex(RuntimeError,"healthy enabled"):self.control.gripper(self.jaw_request())
        self.assertEqual(self.piper.frames,[])

    def test_jaw_send_failure_is_durable_and_never_retried(self):
        self.task_stage();self.piper.failure="jaw_send_failure"
        with self.assertRaisesRegex(RuntimeError,"jaw send failed"):self.control.gripper(self.jaw_request())
        self.assertEqual(self.control.status()["phase"],"failed")
        self.assertEqual(self.session["pending"]["attempted_frames"],1)
        self.assertEqual(self.control.status()["receipts"][0]["socket_send_returns"],0)
        with self.assertRaises(RuntimeError):self.control.gripper(self.jaw_request())
        self.assertEqual(self.piper.frames,[]);self.assertIsNone(self.piper.ticket)

    def test_jaw_arm_drift_after_send_latches_no_other_command(self):
        self.task_stage();original=self.piper.snapshot
        def drift():
            s=original()
            if self.piper.jaw_goal is not None:
                self.piper.q[0]+=200;s["q"][0]+=200*entry.support.RAD_PER_RAW
                s["raw_q"][0]+=200;s["pose"]=fixtures.fk(s["q"])
            return s
        self.piper.snapshot=drift
        with self.assertRaisesRegex(RuntimeError,"Arm drift"):self.control.gripper(self.jaw_request())
        self.assertEqual(len(self.piper.frames),1);self.assertIsNotNone(self.session["failure"])

    def test_jaw_stale_fragment_after_send_cannot_report_success(self):
        self.task_stage();original=self.piper.snapshot
        def stale():
            s=original()
            if self.piper.jaw_goal is not None:s["stamps"][7]-=.101
            return s
        self.piper.snapshot=stale
        with self.assertRaisesRegex(RuntimeError,"100ms"):self.control.gripper(self.jaw_request())
        self.assertEqual(len(self.piper.frames),1);self.assertEqual(self.control.status()["phase"],"failed")


for name in dir(fixtures.InterruptibleTests):
    if name.startswith("test_")and name not in GuardedTests.__dict__:setattr(GuardedTests,name,None)


class ReservationTests(unittest.TestCase):
    def test_successful_parent_unchanged_and_fixed_new_session_cannot_retry(self):
        import tempfile
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);boot=entry.predecessor.PARENT_BOOT
            for name in (boot+".json",entry.predecessor.CHILD_NAME):
                (root/name).write_bytes((entry.v1.SESSION_ROOT/name).read_bytes())
            before={p.name:p.read_bytes()for p in root.iterdir()}
            reviewed=dict(parent_success_session_sha256=entry.SUCCESS_SESSION_SHA,
                          parent_success_result_sha256=entry.SUCCESS_RESULT_SHA,user_authorization=entry.AUTHORIZATION)
            with entry.reserve(boot,reviewed,session_root=root)as(parent,store,session):
                self.assertEqual(session["stage"],"commissioning")
                self.assertEqual(store.load()["generation"],0)
            self.assertTrue(all((root/name).read_bytes()==content for name,content in before.items()))
            with self.assertRaisesRegex(RuntimeError,"already exists"):
                with entry.reserve(boot,reviewed,session_root=root):pass


if __name__=="__main__":unittest.main()
