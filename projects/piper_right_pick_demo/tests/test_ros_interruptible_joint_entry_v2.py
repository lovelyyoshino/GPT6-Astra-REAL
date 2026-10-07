"""Offline regression including the saved real failed pre-send snapshots."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import types
from unittest.mock import patch
import test_ros_interruptible_joint_entry as fixtures

SPEC=importlib.util.spec_from_file_location("interruptible_v2",Path(__file__).parents[1]/"scripts/ros_interruptible_joint_entry_v2.py")
v2=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(v2)


class CandidateTests(fixtures.InterruptibleTests):
    def setUp(self):
        super().setUp();self.control.feedback.close()
        def register(name,callback):self.services[name]=callback;return name
        self.control=v2.InterruptibleCandidate(self.node,types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
                          fixtures.limits(),fixtures.fk,self.store,self.session,self.path,{"offline":True},lambda:None,self.clock,
                          service_factory=register,namespace="/offline_driver",token="offline_token")
        self.addCleanup(self.control.feedback.close)

    def test_candidate_cannot_start_ros_and_shares_original_session_root(self):
        with self.assertRaisesRegex(SystemExit,"Offline candidate only"):v2.main()
        self.assertEqual(v2.SESSION_ROOT,fixtures.entry.SESSION_ROOT)

    def test_preparation_motion_rebinds_target_instead_of_reusing_old_angle(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        old_angles=[]
        def delayed_ownership():
            if self.control.requested():
                old_angles.append(self.piper.q[1]);self.clock.sleep(.03)
                self.piper.q[1]-=350;self.piper.step_raw=1
        self.control.ownership_check=delayed_ownership
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"hold_confirmed")
        self.assertLess(self.piper.targets[1][1],old_angles[0]-300)
        self.assertEqual(result["hold_target_raw"],self.piper.targets[1])
        self.assertGreater(abs(result["hold_target_raw"][1]-result["request_candidate_raw"][1]),171)
        self.assertEqual(result["after"]["raw_q"][1],result["hold_target_raw"][1])
        self.assertEqual(len(self.piper.frames),8)

    def test_same_slip_limit_still_refuses_motion_after_latest_plan(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        original=v2.support.path_check
        def add_motion(*args):
            result=original(*args)
            if self.control.status()["phase"]=="hold_sending":self.piper.q[1]-=300
            return result
        with patch.object(v2.support,"path_check",side_effect=add_motion):
            with self.assertRaisesRegex(RuntimeError,"slip from latest"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4);self.assertEqual(self.control.status()["phase"],"failed")

    def test_latest_plan_age_still_refuses_slow_computation_without_hold(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        original=v2.support.path_check
        def delayed(*args):
            result=original(*args)
            if self.control.status()["phase"]=="hold_sending":self.clock.sleep(.11)
            return result
        with patch.object(v2.support,"path_check",side_effect=delayed):
            with self.assertRaisesRegex(RuntimeError,"100ms"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4)

    def test_saved_real_failure_replays_latest_binding_without_relaxed_thresholds(self):
        run=Path(__file__).parents[1]/"runs/stop_integration_20261006_152651/runtime"
        events=[json.loads(line)for line in (run/"feedback.jsonl").read_text().splitlines()]
        intent=next(r for r in events if r["event"]=="intent"and r["kind"]=="hold")
        state=json.loads((run/"state.json").read_text())
        old=intent["measured"];latest=state["latest_state"];origin=state["before"]
        self.assertGreater(math.dist(old["pose"][:3],latest["pose"][:3]),.0005)
        self.assertLess(max(abs(a-b)for a,b in zip(old["q"],latest["q"])),.003)
        self.assertLess(v2.support.rotation_distance(old["pose"],latest["pose"]),.003)
        witness=copy.deepcopy(latest);witness["sequence"]+=14
        witness["stamps"]=[s+.001 for s in latest["stamps"]]
        self.clock.now=max(witness["stamps"])+.005
        self.control.state.update(active=True,hold_requested=True,sequence=1,before=origin,
                                  requested_raw=state["requested_raw"],target_raw=state["target_raw"])
        self.session.update(stop_latched=True,pending={})
        original_target=[v*v2.support.RAD_PER_RAW for v in state["target_raw"]]
        self.control.fk=v2.support.manufacturer_fk()  # Pure manufacturer FK only, never C_PiperInterface.
        audit=[];samples=iter([latest,witness])
        def read(**kw):
            audit.append("snapshot");sample=copy.deepcopy(next(samples))
            self.piper.healthy(sample,allow_moving=True)
            return sample
        real_path=v2.support.path_check
        def actual_path(before,raw,limits,fk):
            audit.append("pure_path")
            self.assertEqual(before["raw_q"],latest["raw_q"])
            return real_path(before,raw,limits,fk)
        old_save=self.control.save;old_record=self.control.record
        self.control.save=lambda:(audit.append("durable_save"),old_save())[1]
        self.control.record=lambda *a,**kw:(audit.append("log"),old_record(*a,**kw))[1]
        self.control.ownership_check=lambda:audit.append("ownership")
        with patch.object(self.control,"read",return_value=copy.deepcopy(latest)):
            with self.assertRaisesRegex(RuntimeError,"Pre-send slip"):
                v2.v1.Interruptible.send_once(self.control,intent["target_raw"],old,origin,original_target,"hold",moving=True)
        self.assertEqual(self.piper.frames,[])
        audit.clear()
        with patch.object(self.control,"read",side_effect=read),patch.object(v2.support,"path_check",side_effect=actual_path):
            receipt=self.control.send_once(intent["target_raw"],old,origin,original_target,"hold",moving=True)
        self.assertEqual(receipt["target_raw"][1],77078)
        self.assertNotEqual(receipt["target_raw"][1],77165)
        self.assertEqual(self.piper.targets[0],receipt["target_raw"])
        self.assertEqual(len(self.piper.frames),4)
        positions=[i for i,name in enumerate(audit)if name=="snapshot"]
        self.assertEqual(audit[positions[0]+1:positions[1]],["pure_path"])
        self.assertEqual(receipt["source_v1_sha256"],v2.V1_SHA)
        self.assertLessEqual(receipt["path"]["position_bound_m"],.03)
        self.assertLessEqual(receipt["path"]["rotation_bound_rad"],.05)

    def test_same_group_witness_can_remain_fresh_without_waiting_again(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        original_read=self.control.read;planned=[]
        def read(**kw):
            if self.control.status()["phase"]=="hold_sending":
                if not planned:
                    state=original_read(**kw);planned.append(copy.deepcopy(state));return state
                if len(planned)==1:
                    planned.append(None);return copy.deepcopy(planned[0])
            return original_read(**kw)
        with patch.object(self.control,"read",side_effect=read):result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"hold_confirmed")
        receipt=self.control.status()["receipts"][-1]
        self.assertEqual(receipt["plan_sample"]["sequence"],receipt["dispatch_sample"]["sequence"])

    def test_latest_witness_xyz_over_point_five_mm_alone_still_refuses(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        original=self.control.read;planned=[]
        def read(**kw):
            s=original(**kw)
            if self.control.status()["phase"]=="hold_sending":
                if not planned:planned.append(copy.deepcopy(s))
                else:
                    s["pose"]=list(planned[0]["pose"]);s["pose"][0]+=.000501
                    self.assertLess(max(abs(a-b)for a,b in zip(s["q"],planned[0]["q"])),.003)
            return s
        with patch.object(self.control,"read",side_effect=read):
            with self.assertRaisesRegex(RuntimeError,"slip from latest"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4)
