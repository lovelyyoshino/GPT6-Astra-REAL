"""Raw synthetic CAN records only; no SDK/ROS/network/hardware operations."""
import copy
import math
import struct
import unittest

from right_pick.fast_qualification import (CAPS, FEEDBACK_IDS, SCOPE,
                                            QualificationError, validate_qualification,
                                            validate_trial)


def sample(t, q1=0, x=250000, mode=1, moving=0):
    values = {0x2A1: bytes((1, 0, mode, 0, moving, 0, 0, 0)),
              0x2A2: struct.pack('>ii', x, 0),
              0x2A3: struct.pack('>ii', 250000, 0),
              0x2A4: struct.pack('>ii', 0, 0),
              0x2A5: struct.pack('>ii', q1, 0),
              0x2A6: bytes(8), 0x2A7: bytes(8),
              0x2A8: struct.pack('>ihBB', 30000, 0, 64, 0)}
    values.update({k: bytes((0, 0, 0, 0, 0, 64, 0, 0)) for k in range(0x261, 0x267)})
    return dict(sampled_at=t, frames=[dict(id=k, timestamp=t-.001*(i+1), data_hex=values[k].hex())
                                    for i, k in enumerate(FEEDBACK_IDS)])


def trial(kind):
    result = dict(samples=[], control_frames=[])
    if kind == 'static_driver_exit':
        result['event_unix_s'] = 100.5
        result['samples'] = [sample(100+i*.05) for i in range(91)]
        return result
    mode = 1 if kind == 'J' else 0
    result.update(command_unix_s=100.5, client_exit_unix_s=100.86, speed_percent=1,
                  target=([math.pi/180, 0, 0, 0, 0, 0] if kind == 'J' else [.256, 0, .25, 0, 0, 0]))
    for i in range(101):
        t = 100+i*.05
        fraction = min(1., max(0., (t-100.6)/.5))
        result['samples'].append(sample(t, q1=round(1000*fraction) if kind == 'J' else 0,
                                       x=250000+round(6000*fraction) if kind == 'P' else 250000,
                                       mode=mode, moving=int(0 < fraction < 1)))
    if kind == 'J':
        frames = [(0x151, bytes((1, 1, 1, 0, 0, 0, 0, 0))),
                  (0x155, struct.pack('>ii', 1000, 0)), (0x156, bytes(8)), (0x157, bytes(8))]
    else:
        frames = [(0x150, bytes(8)), (0x151, bytes((1, 0, 1, 0, 0, 0, 0, 0))),
                  (0x152, struct.pack('>ii', 256000, 0)), (0x153, struct.pack('>ii', 250000, 0)),
                  (0x154, bytes(8)), (0x159, struct.pack('>iHBB', 30000, 200, 1, 0)),
                  (0x151, bytes((1, 0, 1, 0, 0, 0, 0, 0)))]
    result['control_frames'] = [dict(id=k, timestamp=100.52+i*.001, data_hex=v.hex())
                                for i, (k, v) in enumerate(frames)]
    return result


def evidence():
    return dict(schema_version=1, qualification_scope=SCOPE, boot_id='boot-current',
                adapter_sha256='a'*64, vendor_sha256='b'*64, can_interface='can1',
                usb_interface='1-6.3:1.0', trials={k: trial(k) for k in ('static_driver_exit', 'J', 'P')}, **CAPS)


def check(value):
    return validate_qualification(value, boot_id='boot-current', adapter_sha256='a'*64, vendor_sha256='b'*64)


def frame(value, frame_id):
    return next(f for f in value['frames'] if f['id'] == frame_id)


def with_receipt(value):
    frames = [dict(id=f['id'], data_hex=f['data_hex']) for f in value['control_frames']]
    kind = 'joint' if value['target'][0] < .1 else 'pose'
    value['driver_receipt'] = dict(
        intent=dict(source='ros_resume_entry', event='command_intent', sequence=1, kind=kind,
                    speed_percent=1, unix_s=100.51, frames=frames),
        sent=dict(source='ros_resume_entry', event='command_sent_unconfirmed', sequence=1,
                  unix_s=100.54, attempted_frames=len(frames), socket_send_returns=len(frames)))
    value['control_frames'] = []
    return value


class FastQualificationTests(unittest.TestCase):
    def test_independent_raw_trials_pass_only_narrow_scope(self):
        report = check(evidence())
        self.assertTrue(report['evidence_valid'])
        self.assertEqual(report['qualified_modes'], ['J', 'P'])
        self.assertEqual(report['limits']['speed_percent'], 1)
        self.assertFalse(report['physical_motion_authorized'])
        for key in ('target_cancelled', 'general_stop_validated', 'power_loss_hold_verified', 'instantaneous_hold_verified'):
            self.assertFalse(report[key])

    def test_static_does_not_qualify_p_or_j(self):
        value = evidence()
        del value['trials']['P']
        value['qualified'] = True
        with self.assertRaises(QualificationError):
            check(value)

    def test_identity_scope_binding_and_caps_rejected(self):
        for key, bad in (('boot_id', 'old'), ('adapter_sha256', 'c'*64), ('vendor_sha256', 'c'*64),
                         ('qualification_scope', 'general_hold'), ('can_interface', 'can0'),
                         ('usb_interface', 'wrong'), ('speed_percent', 50), ('speed_percent', True),
                         ('max_translation_m', .031), ('max_rotation_rad', .051), ('max_feedback_age_s', .2),
                         ('schema_version', True)):
            value = evidence()
            value[key] = bad
            with self.subTest(key=key, bad=bad), self.assertRaises(QualificationError):
                check(value)

    def test_no_raw_frames_cannot_be_replaced_by_qualified_boolean(self):
        value = trial('J')
        value['qualified'] = True
        value['samples'][20]['frames'] = []
        with self.assertRaises(QualificationError):
            validate_trial('J', value)

    def test_raw_health_disables_faults_modes_and_jaw_are_checked(self):
        for ident, byte, bad in ((0x2A1, 0, 0), (0x2A1, 1, 1), (0x2A1, 7, 1),
                                 (0x2A1, 2, 4), (0x261, 5, 0), (0x2A8, 6, 0)):
            value = trial('J')
            f = frame(value['samples'][20], ident)
            data = bytearray.fromhex(f['data_hex']); data[byte] = bad; f['data_hex'] = data.hex()
            with self.subTest(ident=ident, byte=byte), self.assertRaises(QualificationError):
                validate_trial('J', value)

    def test_raw_joint_limit_still_enforced(self):
        value = trial('J')
        frame(value['samples'][20], 0x2A5)['data_hex'] = struct.pack('>ii', 100, -1).hex()
        with self.assertRaisesRegex(QualificationError, 'joint limits'):
            validate_trial('J', value)

    def test_stale_future_regressing_and_gapped_feedback_rejected(self):
        for kind in ('stale', 'future', 'regressing', 'gap'):
            value = trial('J')
            if kind == 'gap':
                del value['samples'][20:24]
            elif kind == 'regressing':
                value['samples'][20]['sampled_at'] = value['samples'][19]['sampled_at']
            else:
                value['samples'][20]['frames'][0]['timestamp'] += (-.2 if kind == 'stale' else .2)
            with self.subTest(kind=kind), self.assertRaises(QualificationError):
                validate_trial('J', value)

    def test_duplicate_frames_nan_and_boolean_timestamps_rejected(self):
        for kind in ('duplicate', 'nan', 'bool'):
            value = trial('J')
            if kind == 'duplicate':
                value['samples'][20]['frames'].append(copy.deepcopy(value['samples'][20]['frames'][0]))
            else:
                value['samples'][20]['sampled_at'] = float('nan') if kind == 'nan' else True
            with self.subTest(kind=kind), self.assertRaises(QualificationError):
                validate_trial('J', value)

    def test_exit_after_arrival_or_before_progress_cannot_prove_lifecycle(self):
        for kind in ('J', 'P'):
            for when in (100.4, 100.61, 102.):
                value = trial(kind); value['client_exit_unix_s'] = when
                with self.subTest(kind=kind, when=when), self.assertRaises(QualificationError):
                    validate_trial(kind, value)

    def test_j_only_motion_cannot_qualify_p(self):
        value = trial('P')
        for item in value['samples']:
            frame(item, 0x2A2)['data_hex'] = struct.pack('>ii', 250000, 0).hex()
        with self.assertRaisesRegex(QualificationError, 'significant unfinished motion'):
            validate_trial('P', value)

    def test_position_progress_proves_motion_when_firmware_flag_stays_zero(self):
        for kind in ('J', 'P'):
            value = trial(kind)
            for item in value['samples']:
                f = frame(item, 0x2A1)
                data = bytearray.fromhex(f['data_hex']); data[4] = 0; f['data_hex'] = data.hex()
            self.assertTrue(validate_trial(kind, value)['evidence_valid'])

    def test_no_recent_progress_cannot_pass_even_with_motion_flag_one(self):
        for kind in ('J', 'P'):
            value = trial(kind)
            identifier = 0x2A5 if kind == 'J' else 0x2A2
            frozen = frame(value['samples'][17], identifier)['data_hex']
            for i in (14, 15, 16):
                frame(value['samples'][i], identifier)['data_hex'] = frozen
            with self.subTest(kind=kind), self.assertRaisesRegex(QualificationError, 'last 100 ms'):
                validate_trial(kind, value)

    def test_zero_flag_does_not_rescue_exit_after_arrival(self):
        value = trial('J'); value['client_exit_unix_s'] = 104.
        for item in value['samples']:
            f = frame(item, 0x2A1)
            data = bytearray.fromhex(f['data_hex']); data[4] = 0; f['data_hex'] = data.hex()
        with self.assertRaisesRegex(QualificationError, 'unfinished motion'):
            validate_trial('J', value)

    def test_insufficient_final_stability_and_stopping_short_rejected(self):
        value = trial('J'); value['samples'] = value['samples'][:60]
        with self.assertRaises(QualificationError):
            validate_trial('J', value)
        value = trial('P')
        for item in value['samples'][25:]:
            frame(item, 0x2A2)['data_hex'] = struct.pack('>ii', 253000, 0).hex()
        with self.assertRaises(QualificationError):
            validate_trial('P', value)

    def test_downward_jump_is_not_hidden_by_eventual_stationarity(self):
        value = trial('static_driver_exit')
        for item in value['samples'][11:]:
            frame(item, 0x2A3)['data_hex'] = struct.pack('>ii', 247000, 0).hex()
        with self.assertRaisesRegex(QualificationError, 'downward excursion'):
            validate_trial('static_driver_exit', value)

    def test_stable_post_exit_shift_is_not_hold(self):
        value = trial('static_driver_exit')
        for item in value['samples'][11:]:
            frame(item, 0x2A2)['data_hex'] = struct.pack('>ii', 252000, 0).hex()
        with self.assertRaises(QualificationError):
            validate_trial('static_driver_exit', value)

    def test_actual_speed_frame_must_match_one_percent(self):
        value = trial('P'); value['control_frames'][1]['data_hex'] = bytes((1, 0, 50, 0, 0, 0, 0, 0)).hex()
        with self.assertRaisesRegex(QualificationError, '1% speed'):
            validate_trial('P', value)

    def test_repeated_or_partial_transmissions_rejected(self):
        for mutate in ('extra', 'missing', 'stop'):
            value = trial('P')
            if mutate == 'extra': value['control_frames'].append(copy.deepcopy(value['control_frames'][-1]))
            if mutate == 'missing': del value['control_frames'][2]
            if mutate == 'stop': value['control_frames'][0]['data_hex'] = '0100000000000000'
            with self.subTest(mutate=mutate), self.assertRaises(QualificationError):
                validate_trial('P', value)

    def test_target_metadata_cannot_substitute_different_actual_goal(self):
        value = trial('P'); value['target'][0] = .260
        with self.assertRaisesRegex(QualificationError, 'does not match'):
            validate_trial('P', value)

    def test_loopback_disabled_uses_full_matching_driver_receipt(self):
        for kind in ('J', 'P'):
            value = with_receipt(trial(kind))
            result = validate_trial(kind, value)
            self.assertEqual(result['transmission_evidence_source'], 'driver_socket_send_receipt_not_bus_delivery')
            self.assertFalse(result['external_transmitter_absence_verified'])

    def test_receipt_incomplete_mismatch_or_extra_visible_frames_rejected(self):
        for mutate in ('partial', 'sequence', 'kind', 'time', 'unexpected'):
            value = with_receipt(trial('J'))
            if mutate == 'partial': value['driver_receipt']['sent']['socket_send_returns'] = 3
            if mutate == 'sequence': value['driver_receipt']['sent']['sequence'] = 2
            if mutate == 'kind': value['driver_receipt']['intent']['kind'] = 'pose'
            if mutate == 'time': value['driver_receipt']['sent']['unix_s'] = 101.
            if mutate == 'unexpected': value['control_frames'] = [dict(id=0x150, timestamp=100.53, data_hex='0100000000000000')]
            with self.subTest(mutate=mutate), self.assertRaises(QualificationError):
                validate_trial('J', value)

    def test_deterministic_digest_and_inputs_unchanged(self):
        value = evidence(); before = copy.deepcopy(value)
        self.assertEqual(check(value)['evidence_sha256'], check(value)['evidence_sha256'])
        self.assertEqual(value, before)


if __name__ == '__main__':
    unittest.main()
