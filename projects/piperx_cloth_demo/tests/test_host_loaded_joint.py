"""Actual SDK/FakeCAN pair chain with synthetic RGB/contact, never real objects.

No cache, grasp record, planner or device execute result is injected. Both
initializations, jaw probes, retention, joint segments and releases use the
production service/host/ledger/adapter and native SDK encoders.
"""
import copy
import math
import unittest
from unittest.mock import patch
from robot_tools.grasp_episode import GraspEpisodeError

import test_host_rgb_joint as ordinary
import test_host_rgb_initialization as initialization

CORRIDOR = initialization.CORRIDOR


class HostLoadedJointTests(unittest.TestCase):
    worker_arm, support_arm = "right", "left"
    open = ordinary.HostRGBJointTests.open
    ids = ordinary.HostRGBJointTests.ids
    check_async_errors = ordinary.HostRGBJointTests.check_async_errors
    start = ordinary.HostRGBJointTests.start
    observe = ordinary.HostRGBJointTests.observe
    request = ordinary.HostRGBJointTests.request
    initialize = ordinary.HostRGBJointTests.initialize
    prepared = ordinary.HostRGBJointTests.prepared
    step = ordinary.HostRGBJointTests.step
    execute = ordinary.HostRGBJointTests.execute
    frame_record = ordinary.HostRGBJointTests.frame_record

    def setUp(self):
        self.contacts = set()
        ordinary.HostRGBJointTests.setUp(self)

    def snapshot(self, robot, gripper):
        state = initialization.HostRGBInitializationTests.snapshot(self, robot, gripper)
        side = self.bindings[id(robot)]
        # Explicit synthetic object response, after an actual native close.
        if side in self.contacts and any(s == side and f.arbitration_id == 0x159
                and int.from_bytes(bytes(f.data[:4]), "big", signed=True) < 49000 for s,f in self.sent):
            self.widths[side] = state["gripper"]["width_m"] = .048
        return state

    def jaw(self, side, event, *, opening=False):
        scene = self.observe()
        peer = "right" if side == "left" else "left"
        req = {"event_id":event,"observation_id":scene["observation_id"],
            "peer_receipt_id":scene["peer_receipts"][peer]["receipt_id"],"arm":side,"kind":"gripper",
            "operation":"release_retreat" if opening else "grip_supported", "width_m":.052 if opening else .0455}
        if opening:
            self.contacts.discard(side)
            req.update(release_support_observation="Synthetic current object resting independently on socket/table",
                       release_support_relation="independent_support_present")
        else:
            self.contacts.add(side)
            req["grasp_object_id"] = "plug" if side == self.worker_arm else "strip"
        return self.execute(req)

    def retained_pair(self):
        self.prepared()
        for side in ("left","right"):
            self.assertEqual(self.execute(self.step(side,"inward-"+side))["status"],"completed")
        for side in ("left","right"):
            result=self.jaw(side,"probe-"+side)
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            state=self.host.grasps.active(side); scene=self.observe()
            result=self.service.call("robot_pair_retain_grasp", {"event_id":"retain-"+side,
                "episode_id":state["identity"]["episode_id"],"observation_id":scene["observation_id"],
                "visual_description":"Synthetic current object between fingers on original support",
                "object_relation":"between_fingers","support_relation":"original_support_present"})
            self.assertEqual(result["episode"]["status"],"retained_static")

    def loaded_request(self, operation="extract_segment", event="extract"):
        scene=self.observe()
        target=[math.radians(v/1000) for v in self.device.joint_binding(self.worker_arm)["cached_target"]["target_raw"]]
        target[0]+=.001  # Synthetic tiny translation/rotation, no contact axis claim.
        return {"event_id":event,"observation_id":scene["observation_id"],
            "peer_receipt_id":scene["peer_receipts"][self.support_arm]["receipt_id"],"arm":self.worker_arm,"kind":"joint",
            "operation":operation,"target_joints_rad":target,"admission_mode":"rgb_supervised",
            "loaded_observation":"Synthetic RGB: retained plug, support fingers and table support fixed strip",
            "corridor_observation":CORRIDOR,"source_object_id":"source-socket","target_object_id":"left-target-socket"}

    def confirm_loaded(self, action_event, relation, *, response="progress", event=None):
        scene=self.observe()
        req={"event_id":event or "response-"+action_event,"action_event_id":action_event,
            "observation_id":scene["observation_id"],"visual_description":"Synthetic new object response only",
            "response":response,"object_relation":"retained_between_fingers",
            "support_relation":"table_supported_stationary","task_relation":relation}
        return self.service.call("robot_pair_confirm_loaded_response",req),req

    def release_confirm(self, side):
        state=self.host.grasps.active(side); scene=self.observe()
        return self.service.call("robot_pair_confirm_release",{"event_id":"release-confirm-"+side,
            "episode_id":state["identity"]["episode_id"],"observation_id":scene["observation_id"],
            "visual_description":"Synthetic object clear of fingers on independent support",
            "object_relation":"object_clear_of_fingers","support_relation":"independent_support_present"})

    def test_native_complete_chain_pending_response_and_local_anchor_release(self):
        self.retained_pair()
        owner,deadline=self.host.owner,self.host.deadline
        original=copy.deepcopy(self.device.grasp_states["right"]["original_anchor"])
        peer=copy.deepcopy(self.device.grasp_states["left"])
        for operation,relation in (("extract_segment","source_separated"),("transport","target_aligned"),
                ("insert_segment","target_seated")):
            before=self.ids("left")[:]; request=self.loaded_request(operation,operation)
            result=self.execute(request)
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            self.assertEqual(result["receipt"]["hardware_commands_sent"],4)
            self.assertEqual(self.ids("left"),before)
            self.assertEqual(self.host.grasps.active("right")["status"],"loaded_pending_visual")
            sent=self.frame_record()
            with self.assertRaises(ValueError):self.service.call("robot_pair_submit_once",self.loaded_request(operation,"pending-bypass"))
            self.assertEqual(self.frame_record(),sent)
            confirmation,req=self.confirm_loaded(operation,relation)
            self.assertEqual(confirmation["hardware_commands_sent"],0)
            self.assertEqual(self.frame_record(),sent)
            self.assertTrue(self.service.call("robot_pair_confirm_loaded_response",req)["replayed"])
            self.assertEqual(self.device.grasp_states["left"],peer)
            self.assertEqual(self.device.grasp_states["right"]["original_anchor"],original)
        state=self.host.grasps.active("right")
        self.assertEqual(len(state["loaded"]["history"]),3)
        self.assertNotEqual(state["loaded"]["local_anchor"],original)
        for side in ("right","left"):
            result=self.jaw(side,"open-"+side,opening=True)
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            self.assertEqual(self.release_confirm(side)["episode"]["status"],"released")
            request=self.step(side,"retreat-"+side,inward=False)
            request.pop("unloaded_observation")
            request.update(operation="release_retreat",release_retreat_observation="Synthetic current empty fingers clear of released object")
            before=self.ids("left" if side=="right" else "right")[:]
            result=self.execute(request)
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            self.assertEqual(self.ids("left" if side=="right" else "right"),before)
        self.assertEqual(self.host.grasp_states,{"left":None,"right":None})
        self.assertEqual(len(self.created),2)
        self.assertEqual(self.host.owner,owner); self.assertEqual(self.host.deadline,deadline)
        self.assertEqual(self.host.ledger.peek_status()["steps"],15)
        self.assertIsNone(self.host.status()["task_success"])

    def test_partial_loaded_send_faults_without_retry_or_visual_promotion(self):
        self.retained_pair(); req=self.loaded_request(); self.fail_id=0x156
        result=self.execute(req)
        self.assertEqual(result["status"],"fault")
        count=len(self.sent)
        self.service.call("robot_pair_submit_once",req)
        self.assertEqual(len(self.sent),count)
        self.assertIsNone(self.device.joint_binding("right")["cached_target"])
        self.assertEqual(self.host.grasps.active("right")["status"],"proof_pending")

    def test_adverse_response_faults_and_preserves_pending_object_history(self):
        self.retained_pair(); result=self.execute(self.loaded_request())
        self.assertEqual(result["status"],"completed",result.get("receipt"))
        count=len(self.sent)
        with self.assertRaises(GraspEpisodeError):self.confirm_loaded("extract","source_engaged",response="unknown")
        self.assertTrue(self.host.fault_event.is_set())
        self.assertEqual(self.device.grasp_states["right"]["status"],"loaded_pending_visual")
        self.assertEqual(len(self.sent),count)

    def test_no_progress_cumulative_budget_and_wrong_phase_are_zero_tx(self):
        self.retained_pair()
        count=len(self.sent)
        with self.assertRaises(GraspEpisodeError):self.service.call("robot_pair_submit_once",self.loaded_request("transport","early"))
        self.assertEqual(len(self.sent),count)
        for index in range(2):
            eid="no-progress-%d"%index
            result=self.execute(self.loaded_request(event=eid))
            self.assertEqual(result["status"],"completed",result.get("receipt"))
            self.confirm_loaded(eid,"source_engaged",response="no_progress")
        count=len(self.sent)
        with self.assertRaises(GraspEpisodeError):self.service.call("robot_pair_submit_once",self.loaded_request(event="third"))
        self.assertEqual(len(self.sent),count)

    def test_peer_revision_changed_between_plan_and_begin_refuses_before_claim(self):
        self.retained_pair()
        request=self.loaded_request(event="peer-revision-change")
        count=len(self.sent)
        resolve=self.host._joint_context
        def changed(*args,**kwargs):
            context=resolve(*args,**kwargs)
            context["loaded_context"]["peer"]["revision"]-=1
            return context
        with patch.object(self.host,"_joint_context",side_effect=changed),self.assertRaisesRegex(ValueError,"revision"):
            self.service.call("robot_pair_submit_once",request)
        self.assertEqual(len(self.sent),count)
        self.assertIsNone(self.host.ledger.event(request["event_id"]))
        self.assertEqual(self.host.grasps.active("right")["status"],"retained_static")

class LoadedRequestTests(unittest.TestCase):
    def test_missing_loaded_scope_refuses_before_any_host_resource_or_claim(self):
        from robot_tools.pair_host import PairHost, PairHostError
        # No device, locks, DB or sources exist: all three malformed requests
        # must be rejected at the public argument boundary, not after a claim.
        host=PairHost.__new__(PairHost)
        host.feedback_policy = None  # Frozen strict-default policy, without opening a host.
        for operation in ("extract_segment","transport","insert_segment"):
            with self.subTest(operation=operation),self.assertRaisesRegex(PairHostError,"explicit current RGB"):
                host.submit("missing-scope","scene","peer","right","joint",[.1]*6,operation)


if __name__=="__main__":unittest.main()
