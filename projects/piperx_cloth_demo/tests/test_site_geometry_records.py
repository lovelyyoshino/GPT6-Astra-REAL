"""Synthetic record import tests; no actual site data or device connections."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from robot_tools import joint_sources as sources
from robot_tools import site_geometry_records as records
import test_joint_sources as source_fixture


class GeometryFixture(unittest.TestCase):
    make_capture = source_fixture.JointSourcesTests.make_capture
    make_geometry = source_fixture.JointSourcesTests.make_geometry
    save_json = source_fixture.JointSourcesTests.save_json
    save_sources = source_fixture.JointSourcesTests.save_sources
    codes = source_fixture.JointSourcesTests.codes

    def setUp(self):
        source_fixture.JointSourcesTests.setUp(self)
        self.index["geometry"] = {}
        self.save_json("index.json", self.index)
        self.record_dir = self.root/"site_records/piper_geometry/measured-set"
        self.record_dir.mkdir(parents=True, mode=0o700)
        for directory in (self.record_dir, self.record_dir.parent, self.record_dir.parent.parent):
            directory.chmod(0o700)
        self.installation = {"schema":records.INSTALLATION_SCHEMA, "installation_id":"fixture-installation",
            "recorded_at_s":90., "recorded_by":"OFFLINE SYNTHETIC TEST ONLY", "devices":{}, "components":{}}
        for side in sources.SIDES:
            self.installation["devices"][side] = {"model":"piper_x", "usb_interface":"test-port-"+side,
                "camera_serial":"test-"+side}
            self.installation["components"][side] = [{"component_id":side+"-"+category,
                "category":category, "kind":"installed_cad_record", "frame":"physical_model_flange",
                "lower_m":[-.01,-.02,-.03], "upper_m":[.02,.03,.1], "error_m":.001,
                "method":"Synthetic complete enclosing box for an offline fixture, not a real measurement"}
                for category in sorted(records.CATEGORIES)]
        self.workspace = {"schema":records.WORKSPACE_SCHEMA, "installation_sha256":None,
            "kind":"installed_workspace_record", "recorded_at_s":92., "recorded_by":"TEST ONLY",
            "frame":"physical_model_arm_base", "method":"Separate synthetic base-frame boxes",
            "bounds_by_arm":{"left":{"lower_m":[-.5,-.4,0.],"upper_m":[.8,.7,1.],"error_m":.01},
                             "right":{"lower_m":[-.4,-.5,-.1],"upper_m":[.7,.8,.9],"error_m":.02}}}
        owner, bindings, scene_ref = self.provider._scene(self.scene, "left")
        self.clearance = {"schema":records.CLEARANCE_SCHEMA, "installation_sha256":None,
            "scope":{"run_id":"run", "owner":owner, "profile_sha256":sources.profile_sha256(self.profile),
                     "bindings":bindings,"scene":scene_ref},
            "measured_at_s":99., "valid_until_s":140., "recorded_by":"TEST ONLY", "method":"Synthetic complete gap survey",
            "measurements":{group:[{"surface_pair":[group+"-surface-a",group+"-surface-b"],"distance_m":.5,"error_m":.01}]
                            for group in sorted(records.GAP_GROUPS)}}
        self.clearance["measurements"]["inter_arm"][0].update(distance_m=.2,error_m=.02)
        self.save_records()

    def save_records(self):
        manifest = {"schema":records.MANIFEST_SCHEMA}
        for name, obj in (("installation",self.installation),("workspace",self.workspace),("clearance",self.clearance)):
            if name != "installation":
                obj["installation_sha256"] = manifest["installation"]["sha256"]
            raw = sources._canonical_bytes(obj)
            path = self.record_dir/(name+".json")
            path.write_bytes(raw)
            path.chmod(0o600)
            manifest[name] = {"path":path.name, "sha256":hashlib.sha256(raw).hexdigest()}
        (self.record_dir/"manifest.json").write_bytes(sources._canonical_bytes(manifest))
        (self.record_dir/"manifest.json").chmod(0o600)

    def publish(self):
        return self.provider.publish_geometry(self.scene, "measured-set")

    def refuse(self, code=None):
        before = self.provider.index_path.read_bytes()
        with self.assertRaises(sources.JointSourcesError) as raised:
            self.publish()
        if code:
            self.assertEqual(raised.exception.code, code)
        self.assertEqual(self.provider.index_path.read_bytes(), before)


class SiteGeometryRecordsTests(GeometryFixture):
    def test_actual_readings_derive_radius_minimum_gap_and_dual_base_intersection(self):
        prior_limits = copy.deepcopy(self.index["controller_limits"])
        report = self.publish()
        derived = report["geometry"]
        self.assertGreaterEqual(derived["attachment_radius_m"]["left"], math.hypot(.021,.031,.101))
        self.assertAlmostEqual(derived["attachment_radius_m"]["left"], math.hypot(.021,.031,.101))
        self.assertLessEqual(derived["available_clearance_m"], .2-.02)
        self.assertAlmostEqual(derived["available_clearance_m"], .18)
        for observed, expected in zip(derived["workspace_min_m"],[-.38,-.39,.01]):
            self.assertGreaterEqual(observed,expected)
            self.assertAlmostEqual(observed,expected)
        for observed, expected in zip(derived["workspace_max_m"],[.68,.69,.88]):
            self.assertLessEqual(observed,expected+1e-16)
            self.assertAlmostEqual(observed,expected)
        self.assertEqual(json.loads(self.provider.index_path.read_bytes())["controller_limits"], prior_limits)
        self.assertTrue(self.provider.diagnose(self.scene,"left")["ready"])
        self.assertFalse(report["dispatch_authorized"])
        self.assertFalse(report["source_truth_authenticated"])
        self.assertEqual(report["hardware_commands_sent"],0)
        self.assertEqual(report["measurement_valid_until_s"],140.)
        self.assertEqual(self.provider.index_path.stat().st_mode & 0o777,0o600)

    def test_same_scene_same_content_replays_without_writes_or_time_renewal(self):
        first = self.publish()
        before = self.provider.index_path.read_bytes()
        self.now = 101.
        with patch.object(sources.os,"write",side_effect=AssertionError("Replay must be read-only")):
            second = self.publish()
        self.assertTrue(second["replayed"])
        self.assertEqual(first["source"],second["source"])
        self.assertEqual(first["measurement_valid_until_s"],second["measurement_valid_until_s"])
        self.assertEqual(before,self.provider.index_path.read_bytes())

    def test_missing_actual_record_reports_gap_without_creating_numeric_template(self):
        shutil.rmtree(self.record_dir)
        self.refuse("geometry_records_missing")
        self.assertFalse(self.record_dir.exists())
        self.assertEqual(list(self.directory.glob("geometry_record_*.json")),[])

    def test_no_arbitrary_bounds_verified_or_surface_samples(self):
        for name, extra in (("installation",{"verified":True}), ("workspace",{"bounds":[-10,10]}),
                            ("clearance",{"available_clearance_m":10.})):
            obj = getattr(self,name)
            obj.update(extra)
            self.save_records()
            self.refuse("geometry_record_schema")
            for key in extra:
                obj.pop(key)
        self.installation["components"]["left"].pop()
        self.save_records()
        self.refuse("geometry_inventory_incomplete")

    def test_recorded_gaps_cover_environment_peer_and_nonadjacent_self_groups(self):
        for group in records.GAP_GROUPS:
            rows = self.clearance["measurements"].pop(group)
            self.save_records()
            self.refuse("geometry_clearance_coverage_missing")
            self.clearance["measurements"][group] = rows
        self.clearance["measurements"]["inter_arm"] = []
        self.save_records()
        self.refuse("geometry_clearance_coverage_missing")

    def test_error_consuming_gap_and_empty_common_workspace_refuse(self):
        self.clearance["measurements"]["inter_arm"][0]["error_m"] = .2
        self.save_records()
        self.refuse("geometry_clearance_invalid")
        self.clearance["measurements"]["inter_arm"][0]["error_m"] = .02
        self.workspace["bounds_by_arm"]["right"]["lower_m"] = [2.,2.,2.]
        self.workspace["bounds_by_arm"]["right"]["upper_m"] = [3.,3.,3.]
        self.save_records()
        self.refuse("geometry_workspace_empty")

    def test_installation_camera_and_physical_flange_frame_are_required(self):
        self.installation["devices"]["left"]["camera_serial"] = "other-camera"
        self.save_records()
        self.refuse("geometry_installation_mismatch")
        self.installation["devices"]["left"]["camera_serial"] = "test-left"
        self.installation["components"]["left"][0]["frame"] = "camera_frame"
        self.save_records()
        self.refuse("geometry_record_schema")

    def test_measurement_owner_connection_scene_and_exact_json_types_bind(self):
        pristine = copy.deepcopy(self.clearance)
        for mutate in (lambda c:c["scope"].update(owner="old-owner"),
                       lambda c:c["scope"]["bindings"]["left"].update(connection_id="old-connection"),
                       lambda c:c["scope"]["scene"]["frames"]["front"].update(frame_number=True),
                       lambda c:c["scope"]["scene"].update(observation_id="old-scene")):
            self.clearance = copy.deepcopy(pristine)
            mutate(self.clearance)
            self.save_records()
            self.refuse("geometry_clearance_scope_mismatch")

    def test_new_rgb_does_not_relabel_or_renew_old_clearance(self):
        self.scene["observation_id"] = "scene-2"
        for peer in self.scene["peer_receipts"].values():
            peer["observation_id"] = "scene-2"
        self.refuse("geometry_clearance_scope_mismatch")

    def test_new_real_measurement_can_reuse_unchanged_installation_and_workspace(self):
        first = self.publish()
        installation = (self.record_dir/"installation.json").read_bytes()
        workspace = (self.record_dir/"workspace.json").read_bytes()
        self.now = 101.
        self.scene.update(observation_id="scene-2",capture_id="capture-2",rgb_received_at=100.9)
        for peer in self.scene["peer_receipts"].values():
            peer["observation_id"] = "scene-2"
        for row in self.scene["saved_rgb_evidence"].values():
            row.update(frame_number=2,host_received_at=100.9)
        self.clearance["scope"]["scene"] = self.provider._scene(self.scene,"left")[2]
        self.clearance["measured_at_s"] = 100.8
        self.save_records()
        second = self.publish()
        self.assertFalse(second["replayed"])
        self.assertNotEqual(first["source"],second["source"])
        self.assertEqual((self.record_dir/"installation.json").read_bytes(),installation)
        self.assertEqual((self.record_dir/"workspace.json").read_bytes(),workspace)
        self.assertEqual(set(json.loads(self.provider.index_path.read_bytes())["geometry"]),{"scene-1","scene-2"})

    def test_short_measurement_cannot_authorize_whole_rgb_action_window(self):
        for expiry in (105., 129.9):
            self.clearance["valid_until_s"] = expiry
            self.save_records()
            self.refuse("geometry_record_time_invalid")

    def test_fixed_expiry_is_rechecked_on_direct_geometry_consumption(self):
        report = self.publish()
        owner, bindings, scene_ref = self.provider._scene(self.scene,"left")
        self.now = 140.
        with self.assertRaises(sources.JointSourcesError) as caught:
            self.provider._geometry(report["source"],owner,bindings,scene_ref)
        self.assertEqual(caught.exception.code,"geometry_record_time_invalid")

    def test_source_io_after_geometry_cannot_cross_expiry(self):
        self.publish()
        original = self.provider._geometry
        def slow(*args):
            result = original(*args)
            self.now = 140.
            return result
        with patch.object(self.provider,"_geometry",slow):
            # The earlier RGB deadline now expires before the recorded survey.
            self.assertIn("current_scene_expired",self.codes())

    def test_published_bounds_are_rederived_not_merely_hash_checked(self):
        report = self.publish()
        data = json.loads((self.directory/report["source"]["path"]).read_bytes())
        data["bounds"]["available_clearance_m"] = 1.
        forged = self.save_json("changed-geometry.json",data)
        index = json.loads(self.provider.index_path.read_bytes())
        index["geometry"]["scene-1"] = forged
        self.save_json("index.json",index)
        self.assertIn("geometry_derived_bounds_mismatch",self.codes())

    def test_failed_index_publication_keeps_limits_and_no_visible_partial_geometry(self):
        before = self.provider.index_path.read_bytes()
        with patch.object(sources.os,"replace",side_effect=OSError("synthetic disk failure")):
            self.refuse("source_publication_failed")
        self.assertEqual(self.provider.index_path.read_bytes(),before)
        self.assertEqual(json.loads(before)["geometry"],{})
        self.assertEqual(list(self.directory.glob(".publish-*")),[])

    def test_expiry_during_record_persistence_never_installs_index(self):
        original = sources._write_temp
        def slow(*args):
            result = original(*args)
            self.now = 141.
            return result
        with patch.object(sources,"_write_temp",slow):
            self.refuse("geometry_record_time_invalid")

    def test_old_owner_index_is_not_migrated_or_overwritten(self):
        self.index["owner"] = "previous-owner"
        self.save_json("index.json",self.index)
        self.refuse("source_index_scope_mismatch")

    def test_concurrent_same_scene_publication_is_one_immutable_result(self):
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda _:self.publish(),range(2)))
        self.assertEqual(sorted(x["replayed"] for x in results),[False,True])
        self.assertEqual(results[0]["source"],results[1]["source"])
        self.assertEqual(len(json.loads(self.provider.index_path.read_bytes())["geometry"]),1)

    def test_commit_io_cannot_return_success_after_original_expiry(self):
        original = sources.os.replace
        def slow(*args,**kwargs):
            result = original(*args,**kwargs)
            self.now = 140.
            return result
        with patch.object(sources.os,"replace",slow), self.assertRaises(sources.JointSourcesError) as caught:
            self.publish()
        self.assertEqual(caught.exception.code,"geometry_record_time_invalid")
        # A complete immutable source may remain; it is not a dispatch permit.
        self.assertIn("current_scene_expired",self.codes())

    def test_bool_nonfinite_and_duplicate_json_values_are_not_measurements(self):
        self.installation["components"]["left"][0]["error_m"] = True
        self.save_records()
        self.refuse("invalid_source_number")
        self.installation["components"]["left"][0]["error_m"] = .001
        self.save_records()
        for raw in (b'{"schema":"x","schema":"y"}', b'{"value":NaN}'):
            path = self.record_dir/"clearance.json"
            path.write_bytes(raw)
            manifest_path = self.record_dir/"manifest.json"
            manifest = json.loads(manifest_path.read_bytes())
            manifest["clearance"]["sha256"] = hashlib.sha256(raw).hexdigest()
            manifest_path.write_bytes(sources._canonical_bytes(manifest))
            self.refuse("invalid_source_json")

    def test_same_scene_different_data_cannot_overwrite_prior_geometry(self):
        self.publish()
        self.clearance["measurements"]["inter_arm"][0]["distance_m"] = .19
        self.save_records()
        self.refuse("geometry_scene_already_published")

    def test_controlled_paths_reject_traversal_symlinks_unsafe_files_and_wrong_hash(self):
        with self.assertRaises(sources.JointSourcesError):
            self.provider.publish_geometry(self.scene,"../escape")
        path = self.record_dir/"clearance.json"
        old = path.read_bytes()
        path.unlink()
        outside = self.root/"outside.json"
        outside.write_bytes(old)
        path.symlink_to(outside)
        self.refuse("geometry_records_unsafe")
        path.unlink()
        path.write_bytes(old)
        path.chmod(0o666)
        self.refuse("geometry_records_unsafe")
        path.chmod(0o600)
        path.write_bytes(old+b" ")
        self.refuse("geometry_record_hash_mismatch")


if __name__ == "__main__":
    unittest.main()
