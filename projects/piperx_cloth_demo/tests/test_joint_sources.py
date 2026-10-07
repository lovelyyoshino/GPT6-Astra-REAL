"""Local artifact reader fixtures; no device/socket/network/SDK construction."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from robot_tools.joint_sources import (GEOMETRY_SCHEMA, INDEX_SCHEMA, LIMITS_SCHEMA, JointSourcesError,
                                       JointSourcesProvider, profile_sha256, publish_controller_limits)
from robot_tools.joint_path import SDK_COMMIT, URDF_SHA256


REPO = Path(__file__).resolve().parents[3]
OFFICIAL = REPO / "projects/piperx_cloth_demo/data/piper_x_official"


class JointSourcesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        sockets = patch("socket.socket", side_effect=AssertionError("No hardware/network permitted"))
        sockets.start()
        self.addCleanup(sockets.stop)
        self.root = Path(self.temp.name)
        self.profile = {"sdk_path": str(self.root/"missing-sdk"), "sdk_commit_audited": SDK_COMMIT,
            "arms": {side: {"model": "piper_x", "firmware": "default", "channel": "can"+str(i),
                            "usb_interface": "test-port-"+side} for i, side in enumerate(("left", "right"))},
            "cameras": {"front": "test-front", "left_wrist": "test-left", "right_wrist": "test-right"}}
        self.now = 100.
        self.provider = JointSourcesProvider(self.root, self.profile, "run", runs_root=self.root/"runs", clock=lambda: self.now)
        self.directory = self.provider.directory
        self.directory.mkdir(parents=True)
        self.directory.chmod(0o700)
        official = self.root/"projects/piperx_cloth_demo/data/piper_x_official"
        official.mkdir(parents=True)
        shutil.copyfile(OFFICIAL/"sdk_constants.py", official/"sdk_constants.py")
        shutil.copyfile(OFFICIAL/"piper_x_description.urdf", official/"piper_x_description.urdf")
        self.bindings = {side: {"connection_id": "current-"+side, "model": "piper_x", "firmware_profile": "default",
                               "channel": self.profile["arms"][side]["channel"], "usb_interface": "test-port-"+side}
                         for side in ("left", "right")}
        self.provider = self.provider._scoped("owner", self.bindings, for_write=True)
        self.directory = self.provider.directory
        self.directory.mkdir(parents=True)
        self.directory.chmod(0o700)
        self.scene = {"observation_id": "scene-1", "capture_id": "capture-1", "rgb_received_at": 99.9,
            "joint_source_bindings": copy.deepcopy(self.bindings), "saved_rgb_evidence": {},
            "peer_receipts": {side: {"arm": side, "owner": "owner", "observation_id": "scene-1"} for side in ("left", "right")}}
        for view in ("front", "left_hand", "right_hand"):
            path = self.root/(view+".png")
            path.write_bytes(b"\x89PNG\r\n\x1a\n"+view.encode())
            self.scene["saved_rgb_evidence"][view] = {"rgb_path": str(path), "artifact_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                                    "frame_number": 1, "host_received_at": 99.9}
        self.capture = self.make_capture()
        self.geometry = self.make_geometry()
        self.index = {"schema": INDEX_SCHEMA, "run_id": "run", "owner": "owner", "profile_sha256": profile_sha256(self.profile),
                      "controller_limits": None, "geometry": {}}
        self.save_sources()

    def save_json(self, name, value):
        path = self.directory/name
        path.write_text(json.dumps(value, sort_keys=True, allow_nan=False))
        return {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    def save_sources(self):
        self.index["controller_limits"] = self.save_json("controller_limits.json", self.capture)
        self.index["geometry"]["scene-1"] = self.save_json("geometry.json", self.geometry)
        self.save_json("index.json", self.index)

    def make_capture(self):
        data = {"schema": LIMITS_SCHEMA, "run_id": "run", "owner": "owner", "bindings": copy.deepcopy(self.bindings),
                "began_at": 90., "ended_at": 94., "operation": "inspect_joint_limits",
                "status": "joint_limits_received_no_motion_commands", "joint_limit_queries_sent": 12,
                "actuator_commands_sent": 0, "controller_limits_changed": False, "sdk_joint_limits_changed": False,
                "guard_violations": [], "errors": [], "joint_limits": {}, "query_receipts": {}}
        bounds = [(-1500, 1500), (0, 1800), (-1700, 0), (-1000, 1000), (-700, 700), (-1800, 1800)]
        for side_index, side in enumerate(("left", "right")):
            data["joint_limits"][side], data["query_receipts"][side] = {}, {}
            for index, (low, high) in enumerate(bounds, 1):
                started = 90. + (side_index*6+index)*.25
                raw = (bytes([index])+struct.pack(">hhHB", high, low, 300, 0)).hex()
                data["joint_limits"][side][str(index)] = {"status": "confirmed", "raw_response_hex": raw,
                    "manufacturer_result": {"min_angle_limit": 999., "max_angle_limit": 999.},  # Never used as raw authority.
                    "response_evidence": {"active": False, "request_started_unix_s": started,
                        "finished_unix_s": started+.2, "ignored_stale_frames": [], "rejected_frames": [],
                        "response_frames": [{"timestamp": started+.02, "received_unix_s": started+.03, "dlc": 8, "payload_hex": raw}]}}
                data["query_receipts"][side][str(index)] = {"arbitration_id": 0x472,
                    "data_hex": bytes((index, 1, 0, 0, 0, 0, 0, 0)).hex(), "outcome": "returned",
                    "sent_at": started+.01, "returned_at": started+.015}
        return data

    def make_geometry(self):
        metrics = {}
        for name in ("left_attachment", "right_attachment", "clearance", "workspace"):
            path = self.directory/(name+"-record.txt")
            path.write_text("Synthetic offline test measurement source: " + name)
            metrics[name] = {"kind": "physical_measurement", "path": path.name,
                             "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        return {"schema": GEOMETRY_SCHEMA, "run_id": "run", "owner": "owner", "profile_sha256": profile_sha256(self.profile),
                "bindings": copy.deepcopy(self.bindings), "workspace_frame": "physical_model_arm_base",
                "scene": {"observation_id": "scene-1", "capture_id": "capture-1", "frames": {
                    view: {key: data[key] for key in ("artifact_sha256", "frame_number", "host_received_at")}
                    for view, data in self.scene["saved_rgb_evidence"].items()}},
                "bounds": {"attachment_radius_m": {"left": .17, "right": .18}, "available_clearance_m": .10,
                           "workspace_min_m": [-.5, -.5, .05], "workspace_max_m": [.5, .5, .6]}, "metric_sources": metrics}

    def codes(self):
        return {gap["code"] for gap in self.provider.diagnose(self.scene, "right")["gaps"]}

    def test_official_sources_and_raw_twelve_replies_produce_existing_four_field_contract(self):
        result = self.provider(self.scene, "right")
        self.assertEqual(set(result), {"model_catalog", "urdf_source", "controller_limits", "geometry"})
        self.assertEqual(result["urdf_source"]["sha256"], URDF_SHA256)
        self.assertAlmostEqual(result["controller_limits"]["left"][0][0], math.radians(-150))
        self.assertAlmostEqual(result["controller_limits"]["right"][5][1], math.pi)
        self.assertEqual(result["geometry"]["attachment_radius_m"], {"left": .17, "right": .18})
        diagnostics = self.provider.diagnose(self.scene, "right")
        self.assertTrue(diagnostics["ready"])
        self.assertFalse(diagnostics["dispatch_authorized"])
        self.assertFalse(diagnostics["source_truth_authenticated"])
        self.assertEqual(diagnostics["hardware_commands_sent"], 0)

    def test_source_values_are_copied_not_a_mutable_provider_permission(self):
        result = self.provider(self.scene, "left")
        result["geometry"]["available_clearance_m"] = 100
        self.assertEqual(self.provider(self.scene, "left")["geometry"]["available_clearance_m"], .10)

    def test_explicit_initialization_basis_reads_limits_without_inventing_geometry(self):
        self.index["geometry"] = {}
        self.save_json("index.json", self.index)
        basis = self.provider.initialization_basis(self.scene, "right")
        self.assertEqual(set(basis), {"model_catalog", "urdf_source", "controller_limits"})
        self.assertAlmostEqual(basis["controller_limits"]["right"][0][0], math.radians(-150))
        # The ordinary motion route remains metric and cannot silently switch.
        with self.assertRaises(JointSourcesError) as caught:
            self.provider(self.scene, "right")
        self.assertIn("geometry_source_missing", {g["code"] for g in caught.exception.gaps})
        self.assertEqual(json.loads(self.provider.index_path.read_text())["geometry"], {})

    def test_initialization_basis_still_requires_this_connections_limit_receipts(self):
        self.index["controller_limits"] = None
        self.save_json("index.json", self.index)
        with self.assertRaises(JointSourcesError) as caught:
            self.provider.initialization_basis(self.scene, "right")
        self.assertIn("controller_limits_capture_missing", {g["code"] for g in caught.exception.gaps})
        self.assertNotIn("geometry_source_missing", {g["code"] for g in caught.exception.gaps})

    def test_initialization_basis_rejects_modified_current_rgb_and_changed_connection(self):
        scene = copy.deepcopy(self.scene)
        scene["joint_source_bindings"]["right"]["connection_id"] = "new-connection"
        with self.assertRaises(JointSourcesError):
            self.provider.initialization_basis(scene, "right")
        Path(self.scene["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"modified")
        with self.assertRaises(JointSourcesError) as caught:
            self.provider.initialization_basis(self.scene, "right")
        self.assertIn("rgb_artifact_changed", {g["code"] for g in caught.exception.gaps})

    def test_missing_index_lists_concrete_metric_and_controller_gaps(self):
        self.provider.index_path.unlink()
        self.assertEqual(self.codes(), {"source_index_missing", "controller_limits_capture_missing", "geometry_source_missing"})
        report = self.provider.diagnose(self.scene, "right")
        self.assertEqual(set(report["official_sources"]), {"model_catalog", "urdf_source"})
        with self.assertRaises(JointSourcesError) as caught:
            self.provider(self.scene, "right")
        self.assertEqual(len(caught.exception.gaps), 3)

    def test_missing_official_cache_is_a_gap_without_download_or_sdk_import(self):
        (self.root/"projects/piperx_cloth_demo/data/piper_x_official/sdk_constants.py").unlink()
        self.assertIn("official_sdk_cache_missing", self.codes())

    def test_corrupt_official_urdf_cannot_be_used_with_expected_filename(self):
        (self.root/"projects/piperx_cloth_demo/data/piper_x_official/piper_x_description.urdf").write_text("unreviewed")
        self.assertIn("official_urdf_cache_missing", self.codes())

    def test_profile_sdk_commit_change_is_not_silently_accepted(self):
        profile = copy.deepcopy(self.profile)
        profile["sdk_commit_audited"] = "b"*40
        provider = JointSourcesProvider(self.root, profile, "run", runs_root=self.root/"runs", clock=lambda: self.now)
        codes = {g["code"] for g in provider.diagnose(self.scene, "right")["gaps"]}
        self.assertIn("official_sdk_commit_mismatch", codes)
        self.assertIn("source_index_missing", codes)

    def test_current_scene_saved_png_change_invalidates_geometry(self):
        Path(self.scene["saved_rgb_evidence"]["front"]["rgb_path"]).write_bytes(b"\x89PNG\r\n\x1a\nchanged")
        self.assertIn("rgb_artifact_changed", self.codes())

    def test_profile_camera_metadata_without_saved_images_is_not_metric_evidence(self):
        self.scene["saved_rgb_evidence"] = None
        self.assertIn("saved_rgb_required", self.codes())

    def test_old_scene_and_new_capture_cannot_reuse_exact_geometry_record(self):
        self.scene["capture_id"] = "capture-2"
        self.assertIn("geometry_scene_scope_mismatch", self.codes())

    def test_new_frame_with_same_pixels_cannot_reuse_old_geometry_record(self):
        self.scene["saved_rgb_evidence"]["left_hand"]["frame_number"] += 1
        self.assertIn("geometry_scene_scope_mismatch", self.codes())

    def test_geometry_scene_frame_boolean_is_not_the_integer_frame_identifier(self):
        self.geometry["scene"]["frames"]["front"]["frame_number"] = True
        self.save_sources()
        self.assertIn("geometry_scene_scope_mismatch", self.codes())

    def test_expired_rgb_cannot_be_refreshed_by_fresh_numeric_source_file(self):
        self.now = 131.
        self.assertIn("current_scene_expired", self.codes())

    def test_slow_source_io_cannot_return_an_expired_scene(self):
        original = self.provider._ref
        def slow(reference, **kwargs):
            result = original(reference, **kwargs)
            self.now += 6.
            return result
        with patch.object(self.provider, "_ref", slow):
            self.assertIn("current_scene_expired", self.codes())

    def test_source_io_clock_regression_is_not_a_new_freshness_window(self):
        original = self.provider._geometry
        def regressed(*args):
            result = original(*args)
            self.now = 99.95
            return result
        with patch.object(self.provider, "_geometry", regressed):
            self.assertIn("source_clock_regressed", self.codes())

    def test_missing_internal_binding_is_not_supplied_by_profile_or_source_file(self):
        self.scene.pop("joint_source_bindings")
        self.assertIn("device_binding_missing", self.codes())

    def test_wrong_device_usb_binding_refuses_profile_reuse(self):
        self.scene["joint_source_bindings"]["left"]["usb_interface"] = "other-port"
        self.assertIn("device_profile_mismatch", self.codes())

    def test_new_owner_cannot_reuse_previous_run_index(self):
        for peer in self.scene["peer_receipts"].values():
            peer["owner"] = "new-owner"
        self.assertIn("source_index_missing", self.codes())

    def test_mixed_peer_owners_cannot_select_a_source_owner(self):
        self.scene["peer_receipts"]["left"]["owner"] = "other"
        self.assertIn("scene_owner_mismatch", self.codes())

    def test_new_connection_invalidates_controller_and_geometry_even_same_owner(self):
        self.scene["joint_source_bindings"]["right"]["connection_id"] = "replacement-sdk-connection"
        self.assertEqual(self.codes(), {"source_index_missing", "controller_limits_capture_missing", "geometry_source_missing"})

    def test_wrong_run_index_is_refused(self):
        self.index["run_id"] = "different-run"
        self.save_sources()
        self.assertIn("source_index_scope_mismatch", self.codes())

    def test_controller_proposal_numbers_do_not_substitute_raw_reply(self):
        row = self.capture["joint_limits"]["left"]["1"]
        row.pop("raw_response_hex")
        row["verified"] = True
        self.save_sources()
        self.assertIn("controller_limit_reply_invalid", self.codes())

    def test_shared_parser_timestamp_or_incomplete_side_does_not_make_twelve_replies(self):
        self.capture["joint_limits"]["left"].pop("6")
        self.capture["timestamp"] = 99.99
        self.save_sources()
        self.assertIn("controller_limits_incomplete", self.codes())

    def test_reordered_wrong_joint_reply_is_refused(self):
        row = self.capture["joint_limits"]["left"]["2"]
        raw = "01"+row["raw_response_hex"][2:]
        row["raw_response_hex"] = raw
        row["response_evidence"]["response_frames"][0]["payload_hex"] = raw
        self.save_sources()
        self.assertIn("controller_limit_joint_mismatch", self.codes())

    def test_duplicate_reply_window_is_refused(self):
        window = self.capture["joint_limits"]["right"]["4"]["response_evidence"]
        window["response_frames"].append(copy.deepcopy(window["response_frames"][0]))
        self.save_sources()
        self.assertIn("controller_limit_reply_invalid", self.codes())

    def test_stale_reply_cannot_use_a_recent_capture_end_time(self):
        self.capture["joint_limits"]["right"]["1"]["response_evidence"]["response_frames"][0]["timestamp"] = 80.
        self.save_sources()
        self.assertIn("controller_limit_window_invalid", self.codes())

    def test_unknown_query_return_cannot_become_source_qualification(self):
        self.capture["query_receipts"]["left"]["1"]["outcome"] = "unknown"
        self.save_sources()
        self.assertIn("controller_limit_query_invalid", self.codes())

    def test_limit_parameter_write_is_not_a_read_capture(self):
        self.capture["controller_limits_changed"] = True
        self.save_sources()
        self.assertIn("controller_limits_capture_failed", self.codes())

    def test_capture_with_future_response_is_not_current(self):
        self.capture["ended_at"] = 101.
        self.save_sources()
        self.assertIn("controller_limits_time_invalid", self.codes())

    def test_geometry_verified_boolean_does_not_replace_metric_records(self):
        self.geometry["verified"] = True
        self.save_sources()
        self.assertIn("geometry_source_schema", self.codes())

    def test_model_rgb_estimate_cannot_supply_metric_clearance(self):
        self.geometry["metric_sources"]["clearance"]["kind"] = "rgb_semantic_observation"
        self.save_sources()
        self.assertIn("geometry_metric_source_invalid", self.codes())

    def test_metric_record_bytes_changed_after_index_commit_are_refused(self):
        (self.directory/"left_attachment-record.txt").write_text("different installed tool")
        self.assertIn("source_hash_mismatch", self.codes())

    def test_workspace_frame_must_be_declared_physical_model_base(self):
        self.geometry["workspace_frame"] = "raw_controller_pose_frame"
        self.save_sources()
        self.assertIn("workspace_source_invalid", self.codes())

    def test_boolean_is_not_a_metric_number(self):
        self.geometry["bounds"]["available_clearance_m"] = True
        self.save_sources()
        self.assertIn("invalid_source_number", self.codes())

    def test_index_reference_cannot_escape_run_directory(self):
        self.index["controller_limits"] = {"path": "../somewhere.json", "sha256": "a"*64}
        self.save_json("index.json", self.index)
        self.assertIn("source_path_escape", self.codes())

    def test_symbolic_link_reference_cannot_escape_run_directory(self):
        source = self.directory/"controller_limits.json"
        outside = self.root/"external.json"
        source.rename(outside)
        source.symlink_to(outside)
        self.assertIn("source_path_escape", self.codes())

    def test_duplicate_index_json_key_is_refused(self):
        self.provider.index_path.write_text('{"schema":"x","schema":"y"}')
        self.assertIn("invalid_source_json", self.codes())

    def test_nonfinite_source_json_never_reaches_plan(self):
        self.provider.index_path.write_text('{"value":NaN}')
        self.assertIn("invalid_source_json", self.codes())

    def publish(self, capture=None, **kwargs):
        return publish_controller_limits(self.root/"runs", kwargs.pop("profile", self.profile),
            kwargs.pop("run_id", "run"), kwargs.pop("owner", "owner"),
            kwargs.pop("bindings", self.bindings), self.capture if capture is None else capture,
            clock=lambda: self.now, **kwargs)

    def test_publish_without_geometry_installs_only_raw_limits_with_private_permissions(self):
        shutil.rmtree(self.directory)
        result = self.publish()
        index = json.loads(self.provider.index_path.read_bytes())
        self.assertEqual(index["geometry"], {})
        self.assertFalse(result["replayed"])
        self.assertFalse(result["source_truth_authenticated"])
        self.assertEqual(result["hardware_commands_sent"], 0)
        self.assertEqual(result["controller_limits"]["right"], self.provider._limits(index["controller_limits"], "owner", self.bindings)["right"])
        self.assertEqual(self.directory.stat().st_mode & 0o777, 0o700)
        self.assertEqual(self.provider.index_path.stat().st_mode & 0o777, 0o600)
        self.assertEqual((self.directory/index["controller_limits"]["path"]).stat().st_mode & 0o777, 0o600)
        self.assertEqual(self.codes(), {"geometry_source_missing"})

    def test_publish_same_capture_replays_without_rewriting_index_or_old_file(self):
        before = self.provider.index_path.read_bytes()
        source_before = (self.directory/"controller_limits.json").read_bytes()
        with patch("robot_tools.joint_sources.os.write", side_effect=AssertionError("Replay must not write")):
            report = self.publish()
        self.assertTrue(report["replayed"])
        self.assertEqual(self.provider.index_path.read_bytes(), before)
        self.assertEqual((self.directory/"controller_limits.json").read_bytes(), source_before)

    def test_publish_new_capture_preserves_geometry_and_previous_capture_bytes(self):
        old = (self.directory/"controller_limits.json").read_bytes()
        updated = copy.deepcopy(self.capture)
        updated["capture_id"] = "second-capture"
        report = self.publish(updated)
        installed = json.loads(self.provider.index_path.read_bytes())
        self.assertEqual(installed["geometry"], self.index["geometry"])
        self.assertEqual((self.directory/"controller_limits.json").read_bytes(), old)
        self.assertNotEqual(report["source"]["path"], "controller_limits.json")
        self.assertTrue(self.publish(updated)["replayed"])
        self.assertTrue(self.provider.diagnose(self.scene, "right")["ready"])

    def test_publish_failed_or_missing_raw_windows_never_installs_a_source(self):
        self.provider.index_path.unlink()
        mutations = [lambda c: c.update(status="unconfirmed"),
                     lambda c: c.update(ok=False),
                     lambda c: c.update(hardware_commands_sent=13),
                     lambda c: c.update(joint_limit_queries_sent=11),
                     lambda c: c.update(actuator_commands_sent=True),
                     lambda c: c["query_receipts"]["left"]["1"].update(outcome="unknown"),
                     lambda c: c["joint_limits"]["right"]["6"].pop("response_evidence"),
                     lambda c: c["joint_limits"]["left"]["1"]["response_evidence"]["response_frames"][0].update(timestamp=89.)]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                capture = copy.deepcopy(self.capture)
                mutate(capture)
                with self.assertRaises(JointSourcesError):
                    self.publish(capture)
                self.assertFalse(self.provider.index_path.exists())
                self.assertEqual(list(self.directory.glob("controller_limits_*.json")), [])

    def test_publish_other_owner_profile_or_connection_uses_separate_epoch(self):
        before = self.provider.index_path.read_bytes()
        changed_owner = copy.deepcopy(self.capture)
        changed_owner["owner"] = "other"
        changed_bindings = copy.deepcopy(self.bindings)
        changed_bindings["left"]["connection_id"] = "new-connection"
        changed_capture = copy.deepcopy(self.capture)
        changed_capture["bindings"] = changed_bindings
        profile = copy.deepcopy(self.profile)
        profile["diagnostic_change"] = True
        for capture, kwargs in [(changed_owner, {"owner": "other"}), (self.capture, {"profile": profile}),
                                (changed_capture, {"bindings": changed_bindings})]:
            with self.subTest(kwargs=kwargs):
                result = self.publish(capture, **kwargs)
                self.assertNotEqual(result["index_path"],str(self.provider.index_path))
                self.assertEqual(json.loads(Path(result["index_path"]).read_bytes())["geometry"],{})
            self.assertEqual(self.provider.index_path.read_bytes(), before)

    def test_publish_wrong_device_profile_or_scope_refuses_before_creating_directories(self):
        shutil.rmtree(self.directory)
        bindings = copy.deepcopy(self.bindings)
        bindings["right"]["usb_interface"] = "other-usb"
        with self.assertRaises(JointSourcesError):
            self.publish(bindings=bindings)
        with self.assertRaises(JointSourcesError):
            self.publish(run_id="../escape")
        self.assertFalse(self.directory.exists())

    def test_reader_and_publisher_reject_fault_latched_success_and_reserved_raw_byte(self):
        def reserved(capture):
            row = capture["joint_limits"]["left"]["1"]
            row["raw_response_hex"] = row["raw_response_hex"][:-2] + "01"
            row["response_evidence"]["response_frames"][0]["payload_hex"] = row["raw_response_hex"]
        for mutation in (lambda c: c.update(ok=True, fault_latched=True), reserved):
            with self.subTest(mutation=mutation):
                self.capture = self.make_capture()
                mutation(self.capture)
                self.save_sources()
                self.assertFalse(self.provider.diagnose(self.scene, "left")["ready"])
                before = self.provider.index_path.read_bytes()
                with self.assertRaises(JointSourcesError):
                    self.publish()
                self.assertEqual(self.provider.index_path.read_bytes(), before)

    def test_publish_index_and_source_symlinks_are_never_followed(self):
        before = self.provider.index_path.read_bytes()
        outside = self.root/"outside.json"
        outside.write_bytes(before)
        self.provider.index_path.unlink()
        self.provider.index_path.symlink_to(outside)
        with self.assertRaises(JointSourcesError):
            self.publish()
        self.assertEqual(outside.read_bytes(), before)
        self.provider.index_path.unlink()
        self.provider.index_path.write_bytes(before)
        source = self.directory/"controller_limits.json"
        source.rename(self.directory/"real.json")
        source.symlink_to("real.json")
        with self.assertRaises(JointSourcesError):
            self.publish()

    def test_publish_source_directory_symlink_and_unsafe_mode_refused(self):
        outside = self.root/"outside-sources"
        self.directory.rename(outside)
        self.directory.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(JointSourcesError):
            self.publish()
        self.directory.unlink()
        outside.rename(self.directory)
        self.directory.chmod(0o777)
        with self.assertRaises(JointSourcesError) as caught:
            self.publish()
        self.assertEqual(caught.exception.code, "source_directory_unsafe")

    def test_publish_partial_write_does_not_install_or_replace_any_capture(self):
        before = self.provider.index_path.read_bytes()
        original_write = os.write
        calls = []
        def broken(fd, data):
            calls.append(1)
            if len(calls) == 1:
                return original_write(fd, data[:17])
            raise OSError("Synthetic disk write failure")
        updated = {**self.capture, "capture_id": "new"}
        with patch("robot_tools.joint_sources.os.write", side_effect=broken), self.assertRaises(JointSourcesError):
            self.publish(updated)
        self.assertEqual(self.provider.index_path.read_bytes(), before)
        self.assertEqual(list(self.directory.glob(".publish-*.tmp")), [])
        self.assertEqual(list(self.directory.glob("controller_limits_*.json")), [])

    def test_publish_atomic_index_replace_failure_leaves_old_index_and_complete_orphan(self):
        before = self.provider.index_path.read_bytes()
        with patch("robot_tools.joint_sources.os.replace", side_effect=OSError("Synthetic rename failure")), self.assertRaises(JointSourcesError):
            self.publish({**self.capture, "capture_id": "new"})
        self.assertEqual(self.provider.index_path.read_bytes(), before)
        files = list(self.directory.glob("controller_limits_*.json"))
        self.assertEqual(len(files), 1)
        self.assertEqual(json.loads(files[0].read_bytes())["capture_id"], "new")
        self.assertEqual(list(self.directory.glob(".publish-*.tmp")), [])

    def test_publish_fsync_failure_before_install_preserves_existing_index(self):
        before = self.provider.index_path.read_bytes()
        with patch("robot_tools.joint_sources.os.fsync", side_effect=OSError("Synthetic fsync failure")), self.assertRaises(JointSourcesError):
            self.publish({**self.capture, "capture_id": "new"})
        self.assertEqual(self.provider.index_path.read_bytes(), before)
        self.assertEqual(list(self.directory.glob(".publish-*.tmp")), [])

    def test_publish_existing_content_hash_path_is_immutable(self):
        updated = {**self.capture, "capture_id": "new"}
        raw = json.dumps(updated, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        path = self.directory/("controller_limits_"+hashlib.sha256(raw).hexdigest()+".json")
        path.write_bytes(b"injected wrong bytes")
        before = self.provider.index_path.read_bytes()
        with self.assertRaises(JointSourcesError):
            self.publish(updated)
        self.assertEqual(path.read_bytes(), b"injected wrong bytes")
        self.assertEqual(self.provider.index_path.read_bytes(), before)

    def test_concurrent_publication_is_serialized_and_same_capture_replays(self):
        self.provider.index_path.unlink()
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda _: self.publish(), range(2)))
        self.assertEqual(sorted(r["replayed"] for r in results), [False, True])
        self.assertEqual(len(list(self.directory.glob("controller_limits_*.json"))), 1)
        self.assertEqual(list(self.directory.glob(".publish-*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
