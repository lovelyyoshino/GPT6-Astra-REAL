"""Offline source epochs. No real DB/SDK, rebinding or session renewal."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch

from robot_tools.joint_sources import JointSourcesProvider, JointSourcesError, publish_controller_limits
import test_joint_sources as source_fixture
from test_site_geometry_records import GeometryFixture


class EpochFixture(unittest.TestCase):
    setUp = source_fixture.JointSourcesTests.setUp
    make_capture = source_fixture.JointSourcesTests.make_capture
    make_geometry = source_fixture.JointSourcesTests.make_geometry
    save_json = source_fixture.JointSourcesTests.save_json
    save_sources = source_fixture.JointSourcesTests.save_sources

    def other_scope(self, *, owner="next-owner", replace_connection=True):
        scene, bindings = copy.deepcopy(self.scene), copy.deepcopy(self.bindings)
        if replace_connection:
            for side in bindings:
                bindings[side]["connection_id"] = "new-"+side
        scene["joint_source_bindings"] = bindings
        for peer in scene["peer_receipts"].values():
            peer["owner"] = owner
        return scene, bindings

    def new_capture(self, owner, bindings, delta=1.):
        # Independent synthetic query windows, not an old receipt relabelled
        # by production code. No publisher here claims physical authenticity.
        capture = self.make_capture()
        capture.update(owner=owner,bindings=copy.deepcopy(bindings),capture_id="fresh-query-"+owner)
        capture["began_at"] += delta
        capture["ended_at"] += delta
        for side in bindings:
            for joint in map(str,range(1,7)):
                query = capture["query_receipts"][side][joint]
                for key in ("sent_at","returned_at"):
                    query[key] += delta
                window = capture["joint_limits"][side][joint]["response_evidence"]
                for key in ("request_started_unix_s","finished_unix_s"):
                    window[key] += delta
                for frame in window["response_frames"]:
                    frame["timestamp"] += delta
                    frame["received_unix_s"] += delta
        return capture

    def publish_scope(self, owner, bindings, capture=None):
        return publish_controller_limits(self.root/"runs",self.profile,"run",owner,bindings,
            capture if capture is not None else self.new_capture(owner,bindings),clock=lambda:self.now)

    @staticmethod
    def bytes_in(directory):
        return {p.relative_to(directory).as_posix():p.read_bytes() for p in directory.rglob("*") if p.is_file()}

    def legacy_only(self):
        base = self.provider.base_directory
        for path in self.directory.iterdir():
            shutil.copyfile(path,base/path.name)
        shutil.rmtree(self.directory)
        return base


class JointSourceEpochTests(EpochFixture):
    def test_new_owner_has_empty_scope_then_only_its_fresh_limits(self):
        old = self.bytes_in(self.directory)
        scene, bindings = self.other_scope()
        missing = self.provider.diagnose(scene,"left")
        self.assertFalse(missing["ready"])
        self.assertEqual(set(missing["available_sources"]),{"model_catalog","urdf_source"})
        self.assertNotEqual(missing["index_path"],str(self.provider.index_path))
        with self.assertRaises(JointSourcesError):
            self.publish_scope("next-owner",bindings,self.capture)
        self.assertFalse(Path(missing["index_path"]).exists())
        result = self.publish_scope("next-owner",bindings)
        current = self.provider.diagnose(scene,"left")
        self.assertEqual(current["index_path"],result["index_path"])
        self.assertEqual({g["code"] for g in current["gaps"]},{"geometry_source_missing"})
        self.assertEqual(json.loads(Path(result["index_path"]).read_bytes())["geometry"],{})
        self.assertEqual(self.bytes_in(self.directory),old)
        self.assertTrue(self.provider.diagnose(self.scene,"left")["ready"])

    def test_one_connection_change_requires_new_source_even_with_same_owner(self):
        scene, bindings = self.other_scope(owner="owner",replace_connection=False)
        bindings["left"]["connection_id"] = "one-reconnected-arm"
        before = self.bytes_in(self.directory)
        diagnostic = self.provider.diagnose(scene,"right")
        self.assertFalse(diagnostic["ready"])
        with self.assertRaises(JointSourcesError):
            self.publish_scope("owner",bindings,self.capture)
        result = self.publish_scope("owner",bindings)
        self.assertNotEqual(result["index_path"],str(self.provider.index_path))
        self.assertEqual(before,self.bytes_in(self.directory))

    def test_legacy_exact_scope_read_only_compatibility_and_new_write_no_copy(self):
        legacy = self.legacy_only()
        before = {p.name:p.read_bytes() for p in legacy.iterdir() if p.is_file()}
        diagnostic = self.provider.diagnose(self.scene,"left")
        self.assertTrue(diagnostic["ready"],diagnostic)
        self.assertEqual(diagnostic["index_path"],str(legacy/"index.json"))
        result = self.publish_scope("owner",self.bindings)
        self.assertIn("/epochs/",result["index_path"])
        self.assertEqual(json.loads(Path(result["index_path"]).read_bytes())["geometry"],{})
        self.assertEqual({p.name:p.read_bytes() for p in legacy.iterdir() if p.is_file()},before)
        self.assertEqual({g["code"] for g in self.provider.diagnose(self.scene,"left")["gaps"]},
                         {"geometry_source_missing"})

    def test_legacy_different_owner_or_one_new_connection_never_becomes_current(self):
        self.legacy_only()
        for owner, changed in (("new-owner",False),("owner",True)):
            scene, _ = self.other_scope(owner=owner,replace_connection=changed)
            diagnostic = self.provider.diagnose(scene,"left")
            self.assertFalse(diagnostic["ready"])
            self.assertIn("/epochs/",diagnostic["index_path"])
            self.assertEqual(set(diagnostic["available_sources"]),{"model_catalog","urdf_source"})

    def test_incomplete_or_corrupt_epoch_cannot_fall_back_to_valid_legacy(self):
        self.legacy_only()
        self.directory.mkdir()
        diagnostic = self.provider.diagnose(self.scene,"left")
        self.assertIn("source_index_missing",{g["code"] for g in diagnostic["gaps"]})
        self.provider.index_path.write_text('{"value":NaN}')
        diagnostic = self.provider.diagnose(self.scene,"left")
        self.assertIn("invalid_source_json",{g["code"] for g in diagnostic["gaps"]})
        self.assertFalse(diagnostic["ready"])

    def test_cross_epoch_or_legacy_absolute_reference_is_rejected(self):
        scene, bindings = self.other_scope()
        result = self.publish_scope("next-owner",bindings)
        path = Path(result["index_path"])
        index = json.loads(path.read_bytes())
        index["controller_limits"] = {**self.index["controller_limits"],
                                      "path":str(self.directory/"controller_limits.json")}
        path.write_text(json.dumps(index))
        diagnostic = self.provider.diagnose(scene,"left")
        self.assertIn("source_path_escape",{g["code"] for g in diagnostic["gaps"]})

    def test_two_publishers_and_shared_reader_never_switch_global_directory(self):
        first_scene, first_bindings = self.other_scope(owner="first")
        second_scene, second_bindings = self.other_scope(owner="second")
        old_directory = self.provider.directory
        with ThreadPoolExecutor(max_workers=2) as pool:
            outputs = list(pool.map(lambda pair:self.publish_scope(*pair),
                                    [("first",first_bindings),("second",second_bindings)]))
            diagnostics = list(pool.map(lambda scene:self.provider.diagnose(scene,"left"),
                                        [first_scene,second_scene]))
        self.assertNotEqual(outputs[0]["index_path"],outputs[1]["index_path"])
        for output, diagnostic, owner in zip(outputs,diagnostics,("first","second")):
            self.assertEqual(output["index_path"],diagnostic["index_path"])
            self.assertEqual(json.loads(Path(output["index_path"]).read_bytes())["owner"],owner)
            self.assertEqual({g["code"] for g in diagnostic["gaps"]},{"geometry_source_missing"})
        self.assertEqual(self.provider.directory,old_directory)

    def test_failed_new_epoch_publication_preserves_prior_epoch_bytes(self):
        before = self.bytes_in(self.directory)
        scene, bindings = self.other_scope()
        with patch("robot_tools.joint_sources.os.replace",side_effect=OSError("synthetic failure")), \
                self.assertRaises(JointSourcesError):
            self.publish_scope("next-owner",bindings)
        self.assertEqual(self.bytes_in(self.directory),before)
        diagnostic = self.provider.diagnose(scene,"left")
        self.assertIn("source_index_missing",{g["code"] for g in diagnostic["gaps"]})
        self.assertTrue(self.provider.diagnose(self.scene,"left")["ready"])

    def test_current_epoch_tampered_owner_and_profile_are_not_accepted(self):
        for field in ("owner","profile_sha256"):
            changed = copy.deepcopy(self.index)
            changed[field] = "altered"
            self.save_json("index.json",changed)
            diagnostic = self.provider.diagnose(self.scene,"left")
            self.assertIn("source_index_scope_mismatch",{g["code"] for g in diagnostic["gaps"]})


class GeometryEpochTests(GeometryFixture):
    def test_same_epoch_parallel_query_capture_and_geometry_preserve_both(self):
        self.provider.index_path.unlink()
        with ThreadPoolExecutor(max_workers=2) as pool:
            limit_future = pool.submit(publish_controller_limits,self.root/"runs",self.profile,"run","owner",
                                       self.bindings,self.capture,clock=lambda:self.now)
            geometry_future = pool.submit(self.publish)
            limits, geometry = limit_future.result(), geometry_future.result()
        self.assertEqual(limits["index_path"],geometry["index_path"])
        self.assertTrue(self.provider.diagnose(self.scene,"left")["ready"])
        index = json.loads(Path(limits["index_path"]).read_bytes())
        self.assertIsNotNone(index["controller_limits"])
        self.assertIn("scene-1",index["geometry"])

    def test_new_scope_measurement_requires_new_limits_and_preserves_old_geometry(self):
        original = self.publish()
        old_directory = self.directory
        before = EpochFixture.bytes_in(old_directory)
        self.now = 101.
        self.scene.update(observation_id="second-scene",capture_id="second-capture",rgb_received_at=100.9)
        for side, binding in self.scene["joint_source_bindings"].items():
            binding["connection_id"] = "replacement-"+side
        for peer in self.scene["peer_receipts"].values():
            peer.update(owner="next-owner",observation_id="second-scene")
        for row in self.scene["saved_rgb_evidence"].values():
            row.update(frame_number=2,host_received_at=100.9)
        with self.assertRaises(JointSourcesError) as caught:
            self.publish()
        self.assertEqual(caught.exception.code,"geometry_clearance_scope_mismatch")
        owner, bindings, scene_ref = self.provider._scene(self.scene,"left")
        # New synthetic survey record. Static installation/workspace bytes are
        # unchanged; production code never relabels the old measurement.
        self.clearance["scope"].update(owner=owner,bindings=bindings,scene=scene_ref)
        self.clearance["measured_at_s"] = 100.8
        self.save_records()
        result = self.publish()
        self.assertNotEqual(result["index_path"],original["index_path"])
        self.assertIsNone(json.loads(Path(result["index_path"]).read_bytes())["controller_limits"])
        diagnostic = self.provider.diagnose(self.scene,"left")
        self.assertEqual({g["code"] for g in diagnostic["gaps"]},{"controller_limits_capture_missing"})
        self.assertEqual(EpochFixture.bytes_in(old_directory),before)
        capture = EpochFixture.new_capture(self,owner,bindings)
        limits = publish_controller_limits(self.root/"runs",self.profile,"run",owner,bindings,capture,clock=lambda:self.now)
        self.assertEqual(limits["index_path"],result["index_path"])
        self.assertTrue(self.provider.diagnose(self.scene,"left")["ready"])
        self.assertEqual(EpochFixture.bytes_in(old_directory),before)


if __name__ == "__main__":
    unittest.main()
