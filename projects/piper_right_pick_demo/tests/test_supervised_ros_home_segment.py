"""Offline software tests only; fake ROS and synthetic FK cannot qualify motion."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("home_segment", Path(__file__).parents[1]/"scripts/supervised_ros_home_segment.py")
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


def limits():
    return dict(max_speed_percent=50, max_translation_step_m=.03, max_rotation_step_rad=.05,
                max_state_age_s=.1, workspace_min_m=[-.6,-.6,.05], workspace_max_m=[.6,.6,.65],
                joint_limits_rad=[[a*client.RAD_PER_RAW,b*client.RAD_PER_RAW]for a,b in client.JOINT_LIMITS_RAW],
                gripper_min_m=0., gripper_max_m=.055)


def fake_fk(q):
    return [.25+.04*q[1], .02*q[0], .25+.03*q[2], 0., q[1]+q[2], q[5]]
fake_fk.joint_radius_bounds_m = [.65,.65,.35,.091,.091,0.]


def sample(now, sequence, raw_q=None):
    raw_q = list(raw_q if raw_q is not None else [69,78646,-60043,0,16869,-7934])
    q = [v*client.RAD_PER_RAW for v in raw_q]
    return dict(sequence=sequence, source_sequence=sequence, stamps=[now-.01]*14, stamp=now-.001,
                raw_q=raw_q, q=q, pose=fake_fk(q), opening_m=.03444, jaw_code=64,
                ctrl_mode=1, arm_status=0, mode=1, teach_status=0, motion_status=0,
                fault=0, driver_codes=[64]*6, enabled=[True]*6, active_command=False,
                driver_accepts_commands=True, failure=None, can_interface="can1",
                source="sdk_receive_raw_frames", sdk_version="0.6.2",
                driver_sha256=client.check_telemetry.__globals__["VENDOR_SHA256"], gripper_torque_sdk_units=12)


class Clock:
    def __init__(self): self.now=100.
    def time(self): return self.now
    def monotonic(self): return self.now
    def sleep(self, seconds): self.now+=seconds


class FakeArm:
    """Only assigned-state feedback; deliberately not robot dynamics."""
    def __init__(self, clock, speed=50, failure=None):
        self.clock, self.speed, self.failure_case = clock, speed, failure
        self.q=[69,78646,-60043,0,16869,-7934]
        self.sequence=0; self.events_list=[]; self.published=[]; self.pub=False; self.closed=False
        self.created_publishers=0; self.post_reads=0; self.identity_reads=0
        self.ros=dict(pose_topic="/piper/right/pos_cmd", speed_param="/piper/right/driver/speed_percent",
                      driver_node="/piper/right/driver", telemetry_topic="/piper/right/eval_telemetry")
        self.transport=types.SimpleNamespace(
            rospy=types.SimpleNamespace(Publisher=self.publisher,get_name=lambda:"/offline_home"),
            master=types.SimpleNamespace(getParam=lambda key:self.speed, getSystemState=self.graph),
            receive=self.receive, events=lambda:copy.deepcopy(self.events_list))
    def graph(self):
        pubs=[(self.ros["telemetry_topic"],[self.ros["driver_node"]])]
        if self.pub:pubs.append((client.TOPIC,["/offline_home"]))
        if self.failure_case=="other_publisher" and self.pub:pubs.append((self.ros["pose_topic"],["/intruder"]))
        return pubs,[(client.TOPIC,[self.ros["driver_node"]])],[]
    def publisher(self,*args,**kwargs):
        self.created_publishers+=1;self.pub=True
        return types.SimpleNamespace(get_num_connections=lambda:1,publish=self.publish,unregister=self.unregister)
    def unregister(self):self.pub=False
    def publish(self,msg):
        self.published.append(msg)
        raw=[round(v*(1000*180/math.pi))for v in msg.position]
        frames=[{"id":0x151,"data_hex":bytes((1,1,50,0,0,0,0,0)).hex()}]
        frames += [{"id":0x155+i,"data_hex":client.struct.pack(">ii",*raw[2*i:2*i+2]).hex()}for i in range(3)]
        self.events_list=[dict(event="command_intent",sequence=1,kind="joint",speed_percent=50,
                              unix_s=self.clock.time(),frames=frames),
                          dict(event="command_sent_unconfirmed",sequence=1,unix_s=self.clock.time()+.001,
                               attempted_frames=4,socket_send_returns=3 if self.failure_case=="partial" else 4)]
        self.q=raw
    def receive(self, timeout):
        self.clock.sleep(.06);self.sequence+=1
        if self.published:self.post_reads+=1
        if self.published and self.failure_case=="timeout":raise TimeoutError("offline receive timeout")
        raw=sample(self.clock.time(),self.sequence,self.q)
        if self.published and self.failure_case=="stale":raw["stamps"][0]-=.11
        if self.published and self.failure_case=="jaw":raw["opening_m"]+=.00051
        if self.published and self.failure_case=="box":
            raw["raw_q"][3]=200;raw["q"][3]=200*client.RAD_PER_RAW;raw["pose"]=fake_fk(raw["q"])
        if self.published and len(self.events_list)==2:
            self.events_list.append(dict(event="command_observed_stable",sequence=1,kind="joint",
                                        unix_s=self.clock.time(),after=copy.deepcopy(raw),arm_target_reached=True))
        return raw
    def observe(self):
        self.identity_reads+=1
        raw=self.receive(3.)
        return dict(raw_telemetry=raw,provenance=dict(binding_verified=True,source_verified=True,
                    driver_pid=123,current_driver_adoption_unix_s=10.,adapter_sha256="offline-adapter",
                    vendor_sha256="offline-vendor",can_interface="can1",usb_interface="1-6.3:1.0",
                    driver_node="/piper/right/driver",command_log="/offline/log",speed_percent=self.speed,
                    command_sequence=1 if self.published else 0))
    def _get_transport(self):return self.transport
    def close(self):
        self.closed=True
        if self.failure_case=="cleanup":raise RuntimeError("offline cleanup failure")


class HomeSegmentTests(unittest.TestCase):
    def setUp(self):
        for name in ("socket.socket","subprocess.Popen"):
            guard=patch(name,side_effect=AssertionError("Real I/O forbidden in offline tests"))
            guard.start();self.addCleanup(guard.stop)

    def test_config_cannot_relax_original_bounds_or_speed(self):
        client.checked_limits({"physical_limits":limits()},30)
        for key,value in (("max_speed_percent",1),("max_speed_percent",50.),("max_rotation_step_rad",.051),
                          ("max_translation_step_m",.031),("max_state_age_s",.101),
                          ("workspace_min_m",[-.61,-.6,.05]),("gripper_max_m",.07)):
            changed=limits();changed[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):client.checked_limits({"physical_limits":changed},30)
        for cap in (0,30.001,math.nan,True):
            with self.assertRaises(RuntimeError):client.checked_limits({"physical_limits":limits()},cap)

    def test_common_scale_encoded_toward_zero_and_exact_four_frames_at50(self):
        before=sample(100,1)
        plan=client.make_plan(before,30,limits(),fake_fk)
        self.assertEqual(plan["message"]["velocity"],[0.]*6+[50.])
        self.assertEqual(plan["expected_frames"][0],{"id":0x151,"data_hex":"0101320000000000"})
        self.assertEqual([v["id"]for v in plan["expected_frames"]],[0x151,0x155,0x156,0x157])
        self.assertTrue(all(a*b>=0 and abs(b)<=abs(a)for a,b in zip(before["raw_q"],plan["target_raw"])))
        self.assertLessEqual(plan["joint_box_bound"]["rotation_rad"],.05)
        self.assertLessEqual(plan["joint_box_bound"]["position_m"],.03)
        self.assertGreaterEqual(len(plan["path_check"]["samples"]),21)

    def test_large_canceling_joints_do_not_bypass_independent_box_bound(self):
        before=sample(100,1)
        # Common-line orientation cancellation is insufficient: every joint
        # can be at a different progress inside the monitored box.
        with self.assertRaisesRegex(RuntimeError,"joint-box"):
            client.candidate(before,.075,30,limits(),fake_fk)
        plan=client.make_plan(before,30,limits(),fake_fk)
        for bits in range(64):
            q=[plan["target"][i]if bits&(1<<i)else before["q"][i]for i in range(6)]
            client.envelope(q,fake_fk(q),before["pose"],limits())

    def test_small_last_remainder_can_target_exact_zero_without_old_minimum(self):
        before=sample(100,1,[50,100,-100,0,50,-50])
        plan=client.make_plan(before,30,limits(),fake_fk)
        self.assertEqual(plan["target_raw"],[0]*6)
        with self.assertRaisesRegex(RuntimeError,"Already exact"):
            client.make_plan(sample(100,1,[0]*6),30,limits(),fake_fk)

    def test_joint_cap_and_measured_pose_mismatch_refuse(self):
        before=sample(100,1)
        with self.assertRaisesRegex(RuntimeError,"ceiling"):
            client.candidate(before,.1,1,limits(),fake_fk)
        before["pose"][0]+=.003
        with self.assertRaisesRegex(RuntimeError,"FK/feedback"):
            client.make_plan(before,1,limits(),fake_fk)

    def test_monitor_keeps_raw_box_jaw_and_original_caps(self):
        before=sample(100,1);plan=client.make_plan(before,1,limits(),fake_fk)
        for change in ("box","jaw","translation","rotation"):
            after=copy.deepcopy(before)
            if change=="box":after["q"][3]+=.00301
            if change=="jaw":after["opening_m"]+=.000501
            if change=="translation":after["pose"][0]+=.03001
            if change=="rotation":after["pose"][3]+=.051
            with self.subTest(change=change),self.assertRaises(RuntimeError):client.monitor(after,before,plan,limits(),fake_fk)

    def invoke(self,directory,arm,*,execute=True,cap=1):
        output=Path(directory)/("out"+str(len(list(Path(directory).glob('out*')))));output.mkdir()
        result={"publish_attempts":0,"arrival_confirmed":False}
        client.run({"physical_limits":limits()},cap,execute,output,result,arm_factory=lambda *a,**k:arm,
                   fk_factory=lambda:fake_fk,session_root=Path(directory)/"sessions",clock=arm.clock,
                   message_factory=lambda **kw:types.SimpleNamespace(**kw))
        return result

    def test_live1_proposal_has_no_publisher_and_reports_speed_mismatch(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock(),speed=1);result=self.invoke(directory,arm,execute=False,cap=30)
            self.assertEqual(result["status"],"proposal_only");self.assertTrue(result["speed_mismatch"])
            self.assertEqual(arm.created_publishers,0);self.assertEqual(arm.published,[])
            self.assertTrue(arm.closed)

    def test_execute_refuses_live1_without_publisher(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock(),speed=1)
            with self.assertRaisesRegex(RuntimeError,"live speed50"):self.invoke(directory,arm)
            self.assertEqual(arm.created_publishers,0)

    def test_first_execute_requires_pilot_ceiling(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock())
            with self.assertRaisesRegex(RuntimeError,"First executed segment"):self.invoke(directory,arm,cap=30)
            self.assertEqual(arm.published,[])

    def test_one_publication_and_new_three_second_stable_window(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock());result=self.invoke(directory,arm)
            self.assertEqual(len(arm.published),1);self.assertTrue(result["arrival_confirmed"])
            self.assertGreaterEqual(result["stable_duration_s"],3.)
            self.assertGreaterEqual(result["stable_feedback_groups"],20)
            self.assertEqual(arm.identity_reads,3);self.assertFalse(arm.pub);self.assertTrue(arm.closed)
            saved=json.loads(Path(result["session_path"]).read_text())
            self.assertIsNone(saved["pending"]);self.assertIsNone(saved["failure"])
            self.assertEqual(saved["completed_segments"],1)

    def test_failure_after_send_is_latched_across_new_output_directories(self):
        for failure in ("partial","timeout","stale","jaw","box"):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory()as directory:
                arm=FakeArm(Clock(),failure=failure)
                with self.assertRaises((RuntimeError,TimeoutError)):self.invoke(directory,arm)
                self.assertEqual(len(arm.published),1);self.assertTrue(arm.closed)
                path=next((Path(directory)/"sessions").glob('*.json'));original=path.read_bytes()
                latched=json.loads(original);self.assertTrue(latched["failure"]["target_uncertain"])
                again=FakeArm(Clock())
                with self.assertRaisesRegex(RuntimeError,"failed or unresolved"):self.invoke(directory,again)
                self.assertEqual(again.published,[]);self.assertEqual(path.read_bytes(),original)

    def test_competing_publisher_refused_before_send(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock(),failure="other_publisher")
            with self.assertRaisesRegex(RuntimeError,"ownership"):self.invoke(directory,arm)
            self.assertEqual(arm.published,[])

    def test_cleanup_failure_latches_even_after_measured_arrival(self):
        with tempfile.TemporaryDirectory()as directory:
            arm=FakeArm(Clock(),failure="cleanup")
            with self.assertRaisesRegex(RuntimeError,"cleanup failure"):self.invoke(directory,arm)
            saved=json.loads(next((Path(directory)/"sessions").glob('*.json')).read_text())
            self.assertIn("cleanup failed",saved["failure"]["reason"])
            self.assertEqual(len(arm.published),1)

    def test_unsequenced_real_driver_failure_event_is_not_ignored(self):
        before=sample(100,1);plan=client.make_plan(before,1,limits(),fake_fk)
        with self.assertRaisesRegex(RuntimeError,"refusal/failure"):
            client.matching_receipt([dict(event="command_refused_or_failed",unix_s=101.)],1,plan,100.)

    def test_offline_state_file_never_constructs_adapter_and_execute_is_forbidden(self):
        with tempfile.TemporaryDirectory()as directory:
            p=Path(directory)/"state.json";p.write_text(json.dumps({"raw_telemetry":sample(100,1)}))
            with patch.object(client,"manufacturer_fk",return_value=fake_fk),patch.object(client,"ROSRightArm",side_effect=AssertionError("No adapter")):
                result={};client.offline_proposal({"physical_limits":limits()},p,30,result)
                self.assertFalse(result["live_freshness_verified"]);self.assertTrue(result["not_dispatched"])
            with self.assertRaises(SystemExit):
                client.main(["--config","absent.json","--state-file",str(p),"--execute","--output-dir",str(Path(directory)/"unused")])
            self.assertFalse((Path(directory)/"unused").exists())


if __name__ == "__main__":unittest.main()
