"""Synthetic raw traces and temporary files only; no physical qualification."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from right_pick import fast_ros, fast_safety
from right_pick.fast_qualification import JOINT_LIMITS_RAW, RAD_PER_RAW
from right_pick.fast_safety import FastSafetyError, FastSafetyGuard, qualification_status
from test_fast_qualification import evidence, with_receipt
from test_fast_ros import telemetry


def offset_trial(trial, amount):
    for key in ('command_unix_s', 'client_exit_unix_s'):
        trial[key] += amount
    for sample in trial['samples']:
        sample['sampled_at'] += amount
        for frame in sample['frames']:
            frame['timestamp'] += amount
    for event in trial['driver_receipt'].values():
        event['unix_s'] += amount


def collector(trial):
    return dict(mode='passive_right_can_qualification_recording', trace_transport_clean=True,
                bad_frames=[], sample_gaps=[], timestamp_backwards=[], socket_dropped_total=0,
                frames_sent_by_this_script=0, sdk_used=False, interface_changed=False,
                binding=dict(channel='can1', expected_usb_interface='1-6.3:1.0', binding_verified=True),
                samples=copy.deepcopy(trial['samples']), control_frames=copy.deepcopy(trial['control_frames']))


class PhysicalGateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.adapter = self.path/'adapter.py'; self.adapter.write_text('# frozen fake adapter\n')
        self.vendor = self.path/'vendor.py'; self.vendor.write_text('# frozen fake vendor\n')
        self.adapter_hash = hashlib.sha256(self.adapter.read_bytes()).hexdigest()
        self.vendor_hash = hashlib.sha256(self.vendor.read_bytes()).hexdigest()
        self.boot = self.path/'boot_id'; self.boot.write_text('boot-current\n')
        pinned = dict(adapter_path=str(self.adapter), adapter_sha256=self.adapter_hash,
                      vendor_path=str(self.vendor), vendor_sha256=self.vendor_hash)
        self.patches = [patch.dict(fast_ros.PINNED, pinned),
                        patch.object(fast_ros, 'VENDOR_SHA256', self.vendor_hash),
                        patch.object(fast_safety, '_BOOT_ID_PATH', self.boot)]
        for item in self.patches:
            item.start(); self.addCleanup(item.stop)
        fast_safety._JSON_CACHE.clear(); fast_safety._QUALIFICATION_CACHE.clear()
        self.addCleanup(fast_safety._JSON_CACHE.clear)
        self.addCleanup(fast_safety._QUALIFICATION_CACHE.clear)
        self.doc = evidence()
        self.doc.update(adapter_sha256=self.adapter_hash, vendor_sha256=self.vendor_hash)
        for kind, amount, seq in (('J', 10, 1), ('P', 20, 2)):
            value = with_receipt(self.doc['trials'][kind]); offset_trial(value, amount)
            for event in value['driver_receipt'].values(): event['sequence'] = seq
        for value in self.doc['trials'].values(): value['collector_report'] = collector(value)
        self.log = self.path/'driver.log'
        self.events = [dict(source='ros_resume_entry', event='read_only_adoption_complete', unix_s=105.)]
        for kind in ('J', 'P'):
            self.events.extend(self.doc['trials'][kind]['driver_receipt'].values())
        self.write_log()
        self.doc['session'] = dict(command_log=str(self.log), driver_adoption_unix_s=105., driver_pid=1234)
        self.manifest = self.path/'qualification.json'; self.write_manifest()
        self.config = dict(physical_qualification=dict(evidence_file=str(self.manifest)),
            ros=dict(command_log=str(self.log)), physical_limits=dict(
                workspace_min_m=[-.5, -.5, .1], workspace_max_m=[.5, .5, .5],
                joint_limits_rad=[[lo*RAD_PER_RAW, hi*RAD_PER_RAW] for lo, hi in JOINT_LIMITS_RAW],
                max_speed_percent=1, max_waypoints=1, max_translation_step_m=.03,
                max_rotation_step_rad=.05, max_state_age_s=.1, gripper_min_m=0.,
                gripper_max_m=.055, max_effort_parameter_nm=.2))
        raw = telemetry(now=130.)
        raw.update(driver_sha256=self.vendor_hash, pose=[.256, 0., .25, 0., 0., 0.], opening_m=.03)
        self.state = dict(nonphysical=False, raw_telemetry=raw,
            robot_state=dict(pose_m_rad=raw['pose'][:], joints_rad=raw['q'][:], enabled=True, moving=False),
            gripper_state=dict(opening_m=.03), provenance=dict(
                binding_verified=True, source_verified=True, driver_pid=1234,
                driver_node='/piper/right/driver', adapter_sha256=self.adapter_hash, vendor_sha256=self.vendor_hash,
                can_interface='can1', usb_interface='1-6.3:1.0', speed_percent=1,
                command_log=str(self.log), current_driver_adoption_unix_s=105., command_sequence=2,
                command_publishers={}))

    def write_manifest(self):
        self.manifest.write_text(json.dumps(self.doc))

    def write_log(self):
        self.log.write_text('\n'.join(json.dumps(e) for e in self.events)+'\n')

    def guard(self):
        return FastSafetyGuard(self.config, clock=lambda: 130.)

    @staticmethod
    def move(x=.26, phase='APPROACH_PEN', speed=1):
        return dict(phase=phase, action='move_eef', confidence=.9,
                    arguments=dict(pose_m_rad=[x, 0., .25, 0., 0., 0.], speed_percent=speed, next_phase=None))

    def test_real_raw_fixture_opens_only_narrow_action_gate(self):
        status = qualification_status(self.config)
        self.assertTrue(status['qualified'])
        self.assertFalse(status['physical_motion_authorized'])
        status = self.guard().preflight(self.state)
        self.assertTrue(status['physical_execution_ready'])
        self.assertFalse(status['physical_motion_authorized'])
        result = self.guard().validate(self.move(), self.state)
        self.assertTrue(result['physical_motion_authorized'])
        self.assertTrue(result['timeout_policy_verified'])
        for key in ('hold_verified', 'target_cancelled', 'general_stop_validated', 'path_or_ik_verified'):
            self.assertFalse(result[key])

    def test_boolean_shortcuts_do_not_open_gate(self):
        self.config['physical_qualification'] = dict(qualified=True, timeout_policy_verified=True)
        self.assertFalse(qualification_status(self.config)['qualified'])
        with self.assertRaisesRegex(FastSafetyError, 'Physical execution unavailable'):
            self.guard().validate(self.move(), self.state)

    def test_transport_clean_flag_cannot_hide_bad_raw_transport(self):
        for key, bad in (('bad_frames', [{'reason': 'CAN error'}]), ('sample_gaps', [1]),
                         ('timestamp_backwards', [1]), ('socket_dropped_total', 2),
                         ('frames_sent_by_this_script', 1), ('trace_transport_clean', False)):
            before = copy.deepcopy(self.doc)
            self.doc['trials']['J']['collector_report'][key] = bad; self.write_manifest()
            with self.subTest(key=key): self.assertFalse(qualification_status(self.config)['qualified'])
            self.doc = before

    def test_trial_cannot_drop_bad_original_samples(self):
        del self.doc['trials']['J']['collector_report']['samples'][10]
        self.write_manifest()
        self.assertIn('omits or changes', qualification_status(self.config)['error'])

    def test_hashed_external_collector_and_mutation_revalidation(self):
        value = self.doc['trials']['P']; report = value.pop('collector_report')
        artifact = self.path/'raw.json'; artifact.write_text(json.dumps(report))
        value['collector_artifact'] = dict(path=str(artifact), sha256=hashlib.sha256(artifact.read_bytes()).hexdigest())
        self.write_manifest()
        self.assertTrue(qualification_status(self.config)['qualified'])
        report['socket_dropped_total'] = 1; artifact.write_text(json.dumps(report))
        self.assertFalse(qualification_status(self.config)['qualified'])

    def test_content_replacement_invalidates_success_cache(self):
        self.assertTrue(qualification_status(self.config)['qualified'])
        self.doc['boot_id'] = 'another boot'; self.write_manifest()
        self.assertFalse(qualification_status(self.config)['qualified'])

    def test_unchanged_artifacts_reuse_derived_validation(self):
        from right_pick.fast_qualification import validate_qualification
        with patch('right_pick.fast_qualification.validate_qualification', wraps=validate_qualification) as checked:
            self.assertTrue(qualification_status(self.config)['qualified'])
            self.assertTrue(qualification_status(self.config)['qualified'])
            self.assertEqual(checked.call_count, 1)

    def test_session_restart_or_missing_source_receipt_rejected(self):
        self.assertTrue(qualification_status(self.config)['qualified'])
        self.events[0]['unix_s'] = 106.; self.write_log()
        self.assertFalse(qualification_status(self.config)['qualified'])
        self.events[0]['unix_s'] = 105.; del self.events[2]; self.write_log()
        self.assertFalse(qualification_status(self.config)['qualified'])

    def test_new_latched_driver_failure_invalidates_cached_qualification(self):
        self.assertTrue(qualification_status(self.config)['qualified'])
        self.events.append(dict(source='ros_resume_entry', event='command_refused_or_failed', further_commands_blocked=True))
        self.write_log()
        self.assertFalse(qualification_status(self.config)['qualified'])

    def test_current_provenance_must_match_session_and_fixed_bindings(self):
        for key, bad in (('driver_pid', 1235), ('command_log', '/other/log'),
                         ('current_driver_adoption_unix_s', 106.), ('speed_percent', 50),
                         ('command_sequence', 1), ('source_verified', False), ('binding_verified', 1),
                         ('usb_interface', 'left'), ('adapter_sha256', 'd'*64)):
            value = copy.deepcopy(self.state); value['provenance'][key] = bad
            with self.subTest(key=key), self.assertRaises(FastSafetyError):
                self.guard().preflight(value)

    def test_other_publisher_and_raw_fault_rejected(self):
        value = copy.deepcopy(self.state)
        value['provenance']['command_publishers'] = {'/piper/right/joint_cmd': ['/other']}
        with self.assertRaises(FastSafetyError): self.guard().preflight(value)
        value = copy.deepcopy(self.state); value['raw_telemetry']['arm_status'] = 1
        with self.assertRaises(FastSafetyError): self.guard().preflight(value)

    def test_current_state_and_encoding_are_independently_bounded(self):
        with self.assertRaises(FastSafetyError): self.guard().validate(self.move(speed=2), self.state)
        with self.assertRaises(FastSafetyError): self.guard().validate(self.move(x=.287), self.state)
        with self.assertRaises(FastSafetyError): self.guard().validate(self.move(x=.27, phase='PREGRASP'), self.state)
        value = copy.deepcopy(self.state); value['robot_state']['pose_m_rad'][0] += .001
        with self.assertRaises(FastSafetyError): self.guard().validate(self.move(), value)

    def test_freshness_includes_qualification_and_final_validation(self):
        self.assertTrue(qualification_status(self.config)['qualified'])
        with self.assertRaises(FastSafetyError):
            FastSafetyGuard(self.config, clock=lambda: 130.2).validate(self.move(), self.state)

    def test_chunks_denied_and_gripper_keeps_effort_bounds(self):
        value = dict(phase='APPROACH_PEN', action='move_eef_chunk', confidence=.9,
                     arguments=dict(waypoints=[[.26, 0., .25, 0., 0., 0.]], speed_percent=1, next_phase=None))
        with self.assertRaises(FastSafetyError): self.guard().validate(value, self.state)
        jaw = dict(phase='GRASP', action='gripper', confidence=.9,
                   arguments=dict(opening_m=.006, effort_parameter_nm=.2))
        self.assertTrue(self.guard().validate(jaw, self.state)['physical_motion_authorized'])
        jaw['arguments']['effort_parameter_nm'] = .3
        with self.assertRaises(FastSafetyError): self.guard().validate(jaw, self.state)

    def test_configuration_cannot_widen_fixed_motion_or_manufacturer_limits(self):
        for key, bad in (('max_translation_step_m', .031), ('max_rotation_step_rad', .051),
                         ('max_speed_percent', 2), ('max_state_age_s', .11), ('max_waypoints', 2),
                         ('max_effort_parameter_nm', .3), ('gripper_max_m', .056),
                         ('workspace_min_m', [-1.1, -.5, .1]),
                         ('joint_limits_rad', [[-3., 3.]]*6)):
            config = copy.deepcopy(self.config); config['physical_limits'][key] = bad
            with self.subTest(key=key): self.assertFalse(qualification_status(config)['qualified'])

    def test_source_file_change_invalidates_all_cached_qualifications(self):
        self.assertTrue(qualification_status(self.config)['qualified'])
        self.adapter.write_text('# changed\n')
        self.assertFalse(qualification_status(self.config)['qualified'])


if __name__ == '__main__':
    unittest.main()
