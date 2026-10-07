"""Robot geometry and homing input regressions, with all transport forbidden."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
SPEC=importlib.util.spec_from_file_location('guarded_client',ROOT/'scripts/ros_guarded_task_client.py')
client=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(client)
from test_ros_joint_stop_probe_entry import limits


class HomePlannerTests(unittest.TestCase):
    def setUp(self):
        guard=patch('socket.socket',side_effect=AssertionError('No ROS/CAN/network in planner tests'))
        guard.start();self.addCleanup(guard.stop)
        self.fk=client.support.manufacturer_fk();self.limits=limits()
        self.before=json.loads((ROOT/'runs/cola_on_cup_resume_20261006_160005/initial_state.json').read_text())
    def test_recorded_pose_has_nominal_toward_zero_plan_inside_full_joint_box(self):
        plan=client.home_target(self.before,self.limits,self.fk)
        self.assertTrue(all(min(0,a)<=b<=max(0,a)for a,b in zip(self.before['raw_q'],plan['target_raw'])))
        self.assertLessEqual(plan['path']['position_bound_m'],.03)
        self.assertLessEqual(plan['path']['rotation_bound_rad'],.05)
        self.assertEqual(plan['speed_percent'],1)
        self.assertLess(max(abs(a-b)for a,b in zip(self.before['raw_q'],plan['target_raw'])),1000)
    def test_small_pose_goes_to_existing_exact_zeros(self):
        s=copy.deepcopy(self.before);s['raw_q']=[10,100,-100,0,10,-10]
        s['q']=[x*client.support.RAD_PER_RAW for x in s['raw_q']];s['pose']=self.fk(s['q'])
        plan=client.home_target(s,self.limits,self.fk)
        self.assertEqual(plan['target_raw'],[0]*6);self.assertEqual(plan['scale'],1.)
    def test_commissioning_target_uses_fixed_anchor_with_current_feedback_validation(self):
        anchor=list(self.before['raw_q']);anchor[4]+=20
        plan=client.home_target(self.before,self.limits,self.fk,1.,anchor)
        i=max(range(6),key=lambda k:abs(anchor[k]));scale=plan['target_raw'][i]/anchor[i]
        self.assertTrue(all(abs(v-round(a*scale))<=1 for a,v in zip(anchor,plan['target_raw'])))
        self.assertGreaterEqual(sum(abs(a-b)>=200 for a,b in zip(self.before['raw_q'],plan['target_raw'])),2)
    def test_invalid_ceiling_and_bad_fk_state_are_rejected(self):
        for value in (0.,31.,float('nan')):
            with self.assertRaises(RuntimeError):client.home_target(self.before,self.limits,self.fk,value)
        s=copy.deepcopy(self.before);s['pose'][2]+=.01
        with self.assertRaisesRegex(RuntimeError,'FK/feedback'):client.home_target(s,self.limits,self.fk)


class JawClientTests(unittest.TestCase):
    def setUp(self):
        fake=types.SimpleNamespace(Gripper=object)
        patcher=patch.dict(sys.modules,{'piper_msgs':types.ModuleType('piper_msgs'),'piper_msgs.srv':fake})
        patcher.start();self.addCleanup(patcher.stop)
        self.calls=[]
        self.before=dict(stage='task',phase='idle',active=False,stop_latched=False,failure=None,
                         adoption_token='a'*32,generation=1,sequence=3)
        self.after=dict(self.before,phase='completed',sequence=4,result={'kind':'gripper','grasp_verified':False})
        self.transport=client.TaskTransport.__new__(client.TaskTransport)
        self.transport.status=iter([self.before,self.after]).__next__
        self.transport.observe=lambda:None
        self.transport.master=types.SimpleNamespace(getSystemState=lambda:([],[],[('/piper/right/gripper_srv',[client.frozen.NODE])]))
        def call(*args):
            self.calls.append(args);return types.SimpleNamespace(status=True,code=15900)
        self.transport.rospy=types.SimpleNamespace(wait_for_service=lambda *a,**kw:None,ServiceProxy=lambda *a:call)
    def test_one_explicit_jaw_request_and_stable_is_not_grasp(self):
        result=self.transport.gripper(55.)
        self.assertEqual(self.calls,[(.055,.2,1,0)])
        self.assertTrue(result['feedback_stable']);self.assertFalse(result['grasp_verified'])
    def test_latched_state_forbids_jaw_and_no_service_call(self):
        self.before['stop_latched']=True
        with self.assertRaisesRegex(RuntimeError,'idle task'):self.transport.gripper(30.)
        self.assertEqual(self.calls,[])
    def test_ambiguous_jaw_response_is_never_retried(self):
        def ambiguous(*args):self.calls.append(args);raise RuntimeError('response lost')
        self.transport.rospy.ServiceProxy=lambda *a:ambiguous
        with self.assertRaisesRegex(RuntimeError,'response lost'):self.transport.gripper(30.)
        self.assertEqual(len(self.calls),1)


if __name__=='__main__':unittest.main()
