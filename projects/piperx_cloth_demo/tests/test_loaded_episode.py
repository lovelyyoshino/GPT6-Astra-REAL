"""Synthetic event evidence only; no devices or image authenticity claims."""
import copy
import unittest
from robot_tools.grasp_episode import apply_event, GraspEpisodeError
from robot_tools.loaded_episode import allowed_next, body_anchor
from test_grasp_episode import retained, scene, event, HASH, measurement, release_visual


class LoadedEpisodeTests(unittest.TestCase):
    def begin(self, state, operation="extract_segment", at=110., eid="segment", source="source", target="left-target"):
        return apply_event(state, event(state, "begin-"+eid, "begin_loaded", {
            "action_event_id": eid, "operation": operation, "source_id": source, "target_id": target,
            "target_raw": [1]*6, "context_sha256": HASH, "scene": scene(state, at=at, frame=int(at))}), now=at)

    def finish(self, state, at=114.):
        pending=state["loaded"]["pending"]
        anchor=copy.deepcopy(body_anchor(state)); anchor["joints_rad"][5]+=.001
        return apply_event(state, event(state, "finish-"+pending["action_event_id"], "finish_loaded", {
            "action_event_id":pending["action_event_id"], "operation":pending["operation"],
            "finished_at":at, "plan_sha256":HASH, "local_anchor":anchor}), now=at)

    def confirm(self, state, relation="source_separated", response="progress", at=115.):
        eid=state["loaded"]["pending"]["action_event_id"]
        return apply_event(state,event(state,"response-"+eid,"confirm_loaded",{
            "action_event_id":eid,"scene":scene(state,at=at,frame=int(at)), "response":response,
            "object_relation":"retained_between_fingers","support_relation":"table_supported_stationary",
            "task_relation":relation,"artifact_sha256":HASH}),now=at)

    def test_full_extract_transport_insert_retains_original_anchor_and_requires_responses(self):
        state=retained("right"); original=copy.deepcopy(state["original_anchor"])
        for i,(operation,relation) in enumerate((("extract_segment","source_separated"),
                ("transport","target_aligned"),("insert_segment","target_seated"))):
            at=110.+i*7
            state=self.begin(state,operation,at,"seg%d"%i)
            with self.assertRaises(GraspEpisodeError): self.begin(state,operation,at+.1,"bypass")
            state=self.finish(state,at+4)
            self.assertEqual(state["status"],"loaded_pending_visual")
            with self.assertRaises(GraspEpisodeError): self.begin(state,operation,at+4.1,"bypass")
            state=self.confirm(state,relation,at=at+5)
        self.assertEqual(state["status"],"retained_local")
        self.assertEqual(state["original_anchor"],original)
        self.assertEqual(len(state["loaded"]["history"]),3)
        self.assertNotEqual(body_anchor(state),original)
        self.assertIsNone(state["physical_stop_verified"])
        self.assertFalse(state["dispatch_authorized"])

    def test_old_images_and_adverse_response_cannot_confirm(self):
        state=self.finish(self.begin(retained("right")))
        for at,response in ((114.,"progress"),(115.,"adverse"),(115.,"unknown")):
            with self.subTest(at=at,response=response),self.assertRaises(GraspEpisodeError):
                self.confirm(state,response=response,at=at)
        self.assertEqual(state["status"],"loaded_pending_visual")

    def test_no_progress_is_cumulative_and_source_identity_cannot_change(self):
        state=retained("right")
        for i in range(2):
            at=110.+7*i
            state=self.confirm(self.finish(self.begin(state,at=at,eid="n%d"%i),at+4),
                "source_engaged","no_progress",at+5)
        self.assertEqual(state["loaded"]["no_progress_count"],2)
        with self.assertRaisesRegex(GraspEpisodeError,"budget"):
            self.begin(state,at=125.,eid="n3")
        state=self.confirm(self.finish(self.begin(retained("right"))))
        with self.assertRaisesRegex(GraspEpisodeError,"identity"):
            self.begin(state,"transport",116.,"changed",target="different-target")

    def test_first_extract_does_not_require_old_loaded_success_but_transport_does(self):
        state=retained("right")
        self.assertEqual(self.begin(state)["status"],"proof_pending")
        with self.assertRaises(GraspEpisodeError):self.begin(state,"transport")
        with self.assertRaises(GraspEpisodeError):self.begin(state,"insert_segment")

    def test_tampered_completed_anchor_and_count_rejected(self):
        state=self.confirm(self.finish(self.begin(retained("right"))))
        for field in ("local_anchor","no_progress_count"):
            bad=copy.deepcopy(state)
            if field=="local_anchor":bad["loaded"][field]["joints_rad"][0]+=.01
            else:bad["loaded"][field]=0.5
            with self.assertRaises(GraspEpisodeError):self.begin(bad,"transport",116.,"bad")

    def test_supported_release_uses_completed_local_anchor_preserving_probe_anchor(self):
        state=self.confirm(self.finish(self.begin(retained("right"))))
        measured=measurement(state,now=119.,trace_id="post-loaded-release")
        measured["anchor"]=copy.deepcopy(body_anchor(state)); measured["observed"]=copy.deepcopy(body_anchor(state))
        rgb=scene(state,at=118.,frame=118)
        next_state=apply_event(state,event(state,"open","begin_release",{
            "action_event_id":"open-jaw","target_width_m":.043,"scene":rgb,
            "measurement":measured,"support_visual":release_visual(state,rgb)}),now=119.)
        self.assertEqual(next_state["status"],"release_pending")
        self.assertEqual(next_state["original_anchor"],state["original_anchor"])

if __name__=="__main__":unittest.main()
