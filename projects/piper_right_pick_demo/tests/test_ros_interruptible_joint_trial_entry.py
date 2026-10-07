"""Offline parent-review locking and single-child authorization tests."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import test_ros_interruptible_joint_entry as fixtures

SPEC=importlib.util.spec_from_file_location("joint_child_trial",Path(__file__).parents[1]/"scripts/ros_interruptible_joint_trial_entry.py")
trial=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(trial)


def evidence():
    original=(trial.v1.SESSION_ROOT/(trial.PARENT_BOOT+".json")).read_bytes()
    summary=json.loads((trial.PARENT_RUN/"verification_summary.json").read_text())
    can=json.loads((trial.PARENT_RUN/"independent_can_review.json").read_text())
    reviewed=dict(parent_failure_sha256=trial.PARENT_SHA,parent_run=str(trial.PARENT_RUN),
                  parent_adoption_token=trial.PARENT_TOKEN,parent_sequence=1,user_authorization=trial.AUTHORIZATION)
    return original,summary,can,reviewed


class ParentGateTests(unittest.TestCase):
    def setUp(self):
        guard=patch("socket.socket",side_effect=AssertionError("No device/ROS sockets"));guard.start();self.addCleanup(guard.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.original,self.summary,self.can,self.reviewed=evidence()
        self.parent=self.root/(trial.PARENT_BOOT+".json");self.parent.write_bytes(self.original)
    def reserve(self):
        return trial.reserve_trial(trial.PARENT_BOOT,self.reviewed,self.summary,self.can,session_root=self.root)
    def test_parent_bytes_unchanged_and_child_reserved_durably_once(self):
        with self.reserve()as(parent,store,child):
            self.assertEqual(parent["failure"]["error"],"Pre-send slip")
            self.assertEqual(store.path.name,trial.CHILD_NAME)
            self.assertEqual(store.load()["parent_failure_sha256"],trial.PARENT_SHA)
            self.assertFalse(child["trial_attempted"])
        self.assertEqual(self.parent.read_bytes(),self.original)
        with self.assertRaisesRegex(RuntimeError,"already exists"):
            with self.reserve():pass
        self.assertEqual(self.parent.read_bytes(),self.original)
    def test_child_remains_reserved_on_exception_and_parent_not_rewritten(self):
        with self.assertRaisesRegex(RuntimeError,"offline startup failure"):
            with self.reserve():raise RuntimeError("offline startup failure")
        self.assertEqual(self.parent.read_bytes(),self.original)
        with self.assertRaisesRegex(RuntimeError,"already exists"):
            with self.reserve():pass
    def test_live_parent_lock_prevents_child_before_any_reservation(self):
        with trial.SessionFile(self.parent):
            with self.assertRaises(BlockingIOError):
                with self.reserve():pass
        self.assertFalse((self.root/trial.CHILD_NAME).exists())
    def test_changed_parent_or_wrong_new_authorization_rejected(self):
        for case in ("hash","token","authorization","run","sequence"):
            reviewed=copy.deepcopy(self.reviewed);digest=trial.PARENT_SHA
            if case=="hash":digest="wrong"
            if case=="token":reviewed["parent_adoption_token"]="oldother"
            if case=="authorization":reviewed["user_authorization"]=""
            if case=="run":reviewed["parent_run"]="/other"
            if case=="sequence":reviewed["parent_sequence"]=2
            with self.subTest(case=case),self.assertRaises(RuntimeError):
                trial.check_parent(json.loads(self.original),digest,trial.PARENT_BOOT,reviewed,self.summary,self.can)
    def test_nonmatching_failure_partial_tx_and_missing_arrival_evidence_refused(self):
        for case in ("failure","initial_partial","hold_sent","no_arrival"):
            parent=json.loads(self.original);can=copy.deepcopy(self.can)
            if case=="failure":parent["failure"]["error"]="CAN send failed"
            if case=="initial_partial":parent["status"]["receipts"][0]["socket_send_returns"]=3
            if case=="hold_sent":parent["pending"]["attempted_frames"]=1
            if case=="no_arrival":can["last_50s"]["j2_constant"]=False
            with self.subTest(case=case),self.assertRaises(RuntimeError):
                trial.check_parent(parent,trial.PARENT_SHA,trial.PARENT_BOOT,self.reviewed,self.summary,can)


class ChildCommandTests(fixtures.InterruptibleTests):
    # Only this class's specifically applicable cases are installed below;
    # normal multi-action v1 tests intentionally do not apply to a one-shot child.
    def setUp(self):
        super().setUp();self.control.feedback.close()
        self.session.update(trial_attempted=False,trial_closed=False)
        def register(name,callback):self.services[name]=callback;return name
        self.control=trial.OneShotTrial(self.node,types.SimpleNamespace(emit=lambda *a,**k:self.events.append((a,k))),
                  fixtures.limits(),fixtures.fk,self.store,self.session,self.path,{"offline":True},lambda:None,self.clock,
                  service_factory=register,namespace="/offline_driver",token="offline_trial")
        self.addCleanup(self.control.feedback.close)
    def test_one_normal_command_then_no_second_even_after_success(self):
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"completed");self.assertEqual(len(self.piper.frames),4)
        self.assertFalse(self.node.adopted);self.assertTrue(self.store.load()["trial_closed"])
        with self.assertRaisesRegex(RuntimeError,"one admitted-or-refused"):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),4)
    def test_refused_command_also_consumes_child_without_sending(self):
        msg=self.message();msg.velocity[-1]=50
        with self.assertRaises(RuntimeError):self.control.execute(msg)
        self.assertEqual(self.piper.frames,[])
        with self.assertRaisesRegex(RuntimeError,"one admitted-or-refused"):self.control.execute(self.message())
        self.assertTrue(self.store.load()["trial_attempted"])
    def test_external_hold_then_trial_closed_without_extra_goal(self):
        initial=self.piper.q[1]
        self.trigger(lambda s:self.control.status()["phase"]=="moving"and initial-s["raw_q"][1]>=260)
        result=self.control.execute(self.message())
        self.assertEqual(result["phase"],"hold_confirmed");self.assertEqual(len(self.piper.frames),8)
        self.assertFalse(result["candidate_offline_only"]);self.assertTrue(result["physical_trial_wrapper"])
        self.assertFalse(json.loads((self.path/"action_000001_result.json").read_text())["candidate_offline_only"])
        self.assertTrue(self.control.status()["trial_closed"])
        with self.assertRaises(RuntimeError):self.control.execute(self.message())
        self.assertEqual(len(self.piper.frames),8)


# Keep shared fixture helpers while excluding inherited tests that intentionally
# exercise multiple actions. The parent/core suites retain their own coverage.
for _name in dir(fixtures.InterruptibleTests):
    if _name.startswith("test_")and _name not in ChildCommandTests.__dict__:
        setattr(ChildCommandTests,_name,None)


if __name__=="__main__":unittest.main()
