"""Offline contract checks; no SDK, CAN, cameras, or robot are instantiated."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/vendor_pose_execute.py'
SPEC = importlib.util.spec_from_file_location('vendor_pose_execute', SOURCE)
CONTROL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTROL)


def plan():
    return {'channel': 'can2', 'usb_interface': '1-6.3:1.0',
            'inverse_kinematics_owner': 'manufacturer_arm_controller',
            'expected_start_pose_mm_deg': [180, 0, 280, 0, 60, 0],
            'expected_start_joints_deg': [0, 50, -50, 0, 10, 0],
            'allowed_initial_arm_status': [0],
            'workspace_mm': [[0, 500], [-300, 400], [80, 400]],
            'stages': [{'id': 'approach', 'operation': 'move', 'mode': 'MOVE_P',
                        'speed_percent': 5, 'sdk_pose_mm_deg': [260, 160, 260, 180, 30, -140]}]}


class Clock:
    def __init__(self):
        self.now = 100.0

    def sleep(self, seconds):
        self.now += seconds


class Cameras:
    nonphysical = True
    def __init__(self):
        self.captures = []

    def check(self):
        pass

    def capture(self, label):
        self.captures.append(label)
        return {'event': 'captured', 'label': label}


class Arm:
    nonphysical = True
    fatal_error = None
    send_errors = []

    def __init__(self, clock, configuration):
        self.clock, self.plan = clock, configuration
        self.commands, self.mutate = [], lambda state: None
        self.arrive = True

    def send_checked(self, name, *values):
        self.commands.append((name, values))

    def snapshot(self):
        stamp = self.clock.now
        state = {'time_s': stamp, 'mono_s': stamp,
                 'rx': {str(i): stamp for i in CONTROL.IDS},
                 'rx_wall': {str(i): stamp for i in CONTROL.IDS},
                 'status': {'arm_status': 0, 'ctrl_mode': 1, 'teach_status': 0,
                            'err_code': 0, 'motion_status': 0, 'mode_feed': 0},
                 'motor_codes': [0x40] * 6, 'gripper': {'opening_mm': 55, 'status_code': 0x40},
                 'pose_mm_deg': list(self.plan['expected_start_pose_mm_deg']),
                 'joints_deg': list(self.plan['expected_start_joints_deg'])}
        if self.commands and self.arrive:
            stage = self.plan['stages'][0]
            if stage['operation'] == 'move':
                state['pose_mm_deg'] = list(stage['sdk_pose_mm_deg'])
            else:
                state['gripper']['opening_mm'] = 27
        self.mutate(state)
        return state


class VendorExecutionTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.plan, self.cameras = Clock(), plan(), Cameras()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.patch = mock.patch.multiple(CONTROL.time, monotonic=lambda: self.clock.now,
                                         time=lambda: self.clock.now, sleep=self.clock.sleep)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.arm = Arm(self.clock, self.plan)
        self.executor_count = 0
        self.new_executor()

    def new_executor(self):
        output = Path(self.temp.name) / str(self.executor_count)
        output.mkdir()
        self.executor_count += 1
        self.executor = CONTROL.Executor(self.arm, self.plan, output, self.cameras, nonphysical=True)
        self.addCleanup(self.executor.trace.close)

    def test_plan_rejects_wrong_arm_and_excess_speed(self):
        CONTROL.validate_plan(self.plan)
        for key, value in [('channel', 'can0'), ('inverse_kinematics_owner', 'external_ik')]:
            changed = copy.deepcopy(self.plan)
            changed[key] = value
            with self.assertRaises(RuntimeError):
                CONTROL.validate_plan(changed)
        self.plan['stages'][0]['speed_percent'] = 6
        with self.assertRaises(RuntimeError):
            CONTROL.validate_plan(self.plan)

    def test_healthy_rejects_disabled_motor(self):
        state = self.arm.snapshot()
        state['motor_codes'][4] = 0
        with self.assertRaises(RuntimeError):
            CONTROL.healthy(state, self.plan)

    def test_split_feedback_missing_or_stale_stops_after_first_command(self):
        for stale in (False, True):
            with self.subTest(stale=stale):
                if self.executor.failed:
                    self.new_executor()  # A fresh explicitly nonphysical test instance.
                self.arm.commands.clear()
                def mutate(state):
                    if self.arm.commands:
                        for can_id in (0x2A3, 0x2A7):
                            if stale:
                                state['rx'][str(can_id)] -= 1.0
                            else:
                                del state['rx'][str(can_id)]
                self.arm.mutate = mutate
                with self.assertRaisesRegex(RuntimeError, 'Stale/missing'):
                    self.executor.step(self.plan['stages'][0])
                self.assertEqual(len(self.arm.commands), 2)
                self.assertFalse(self.cameras.captures)

    def test_status_four_motion_zero_never_counts_as_arrived(self):
        self.plan['allowed_initial_arm_status'] = [0, 4]
        self.arm.mutate = lambda state: state['status'].update(arm_status=4)
        with self.assertRaisesRegex(RuntimeError, 'arm_status=4'):
            self.executor.step(self.plan['stages'][0], first=True)
        self.assertTrue(self.executor.pending)
        self.assertFalse(self.cameras.captures)

    def test_feedback_before_command_cannot_confirm_arrival(self):
        def mutate(state):
            if self.arm.commands:
                for can_id in (0x2A3, 0x2A7):
                    state['rx'][str(can_id)] = 100.0
                    state['rx_wall'][str(can_id)] = 100.0
        self.arm.mutate = mutate
        with self.assertRaisesRegex(RuntimeError, 'Stale/missing'):
            self.executor.step(self.plan['stages'][0])
        self.assertFalse(self.cameras.captures)

    def test_normal_arrival_can_advance(self):
        self.executor.step(self.plan['stages'][0])
        self.assertFalse(self.executor.pending)
        self.assertEqual(self.executor.report['stages'][-1]['status'], 'target_reached')
        self.assertEqual(self.cameras.captures, ['approach'])
        self.assertGreaterEqual(self.clock.now, 100.4)

    def test_unreached_pose_times_out(self):
        self.arm.arrive = False
        with self.assertRaises(TimeoutError):
            self.executor.step(self.plan['stages'][0])
        self.assertTrue(self.executor.pending)
        self.assertFalse(self.cameras.captures)

    def test_closed_gripper_is_not_grasp_success(self):
        stage = {'id': 'close_once', 'operation': 'gripper', 'opening_mm': 0, 'torque_parameter_nm': 0.3}
        self.plan['stages'] = [stage]
        self.executor.step(stage)
        result = self.executor.report['stages'][-1]
        self.assertEqual(result['status'], 'gripper_settled')
        self.assertIn('unknown', result['object_held'])
        self.assertEqual(self.executor.report['grasp_result'], 'not_visually_evaluated')

    def test_closing_without_any_width_change_times_out(self):
        stage = {'id': 'close_once', 'operation': 'gripper', 'opening_mm': 0, 'torque_parameter_nm': 0.3}
        self.plan['stages'], self.arm.arrive = [stage], False
        with self.assertRaises(TimeoutError):
            self.executor.step(stage)
        self.assertFalse(self.cameras.captures)
        self.assertEqual(self.executor.report['stages'][-1]['status'], 'waiting_for_feedback')

    def test_initial_rejection_can_clear_but_later_rejection_is_not_exempt(self):
        self.plan['allowed_initial_arm_status'] = [0, 4]
        self.arm.mutate = lambda state: state['status'].update(arm_status=0 if self.arm.commands else 4)
        self.executor.step(self.plan['stages'][0], first=True)
        self.assertEqual(self.executor.report['stages'][-1]['status'], 'target_reached')
        self.arm.mutate = lambda state: state['status'].update(arm_status=4)
        with self.assertRaisesRegex(RuntimeError, 'arm_status=4'):
            self.executor.step(self.plan['stages'][0])
        self.assertEqual(len(self.arm.commands), 2)

    def prepare_abort(self):
        self.executor.pending = True
        self.executor.sent_mono = self.executor.sent_wall = self.clock.now - 1

    def test_rejected_but_healthy_stationary_arm_does_not_receive_quick_stop(self):
        self.prepare_abort()
        self.arm.mutate = lambda state: state['status'].update(arm_status=4)
        self.executor.abort()
        stop = self.executor.report['stop']
        self.assertFalse(self.arm.commands)
        self.assertFalse(stop['requested'])
        self.assertEqual(stop['action'], 'tx_latched_passive_observation_only')
        self.assertFalse(stop['confirmed'])
        self.assertTrue(stop['hold_unverified'])
        self.assertGreaterEqual(self.clock.now, 103)

    def test_still_moving_arm_never_receives_quick_stop_or_claims_hold(self):
        self.prepare_abort()
        def mutate(state):
            stopping = bool(self.arm.commands)
            state['status'].update(arm_status=1 if stopping else 0,
                                   motion_status=0 if stopping else 1)
        self.arm.mutate = mutate
        self.executor.abort()
        self.assertEqual(self.arm.commands, [])
        self.assertFalse(self.executor.report['stop']['requested'])
        self.assertFalse(self.executor.report['stop']['confirmed'])
        self.assertFalse(self.executor.report['stop']['physical_target_cancelled'])
        self.assertTrue(self.executor.report['onsite_intervention_required'])
        self.assertGreaterEqual(self.clock.now, 103)

    def test_missing_feedback_only_records_failure_without_tx(self):
        self.prepare_abort()
        def mutate(state):
            del state['rx'][str(0x2A7)]
            state['status']['arm_status'] = 1 if self.arm.commands else 0
        self.arm.mutate = mutate
        self.executor.abort()
        self.assertEqual(self.arm.commands, [])
        self.assertFalse(self.executor.report['stop']['requested'])
        self.assertFalse(self.executor.report['stop']['confirmed'])
        self.assertTrue(self.executor.report['stop']['observation_errors'])

    def test_timeout_seals_direct_step_and_send_even_without_main(self):
        self.arm.arrive = False
        with self.assertRaises(TimeoutError):
            self.executor.step(self.plan['stages'][0])
        original = list(self.arm.commands)
        self.executor.abort()
        self.executor.abort()
        for call in (lambda: self.executor.step(self.plan['stages'][0]),
                     lambda: self.executor._send('GripperCtrl', 0, 300, 1, 0),
                     self.executor.preflight):
            with self.assertRaisesRegex(RuntimeError, 'Failure latched'):
                call()
        self.assertEqual(self.arm.commands, original)
        self.assertTrue(self.executor.report['further_tx_blocked'])

    def test_partial_send_failure_does_not_retry_or_send_recovery(self):
        original = self.arm.send_checked
        def broken(name, *args):
            original(name, *args)
            raise OSError('partial transport failure')
        self.arm.send_checked = broken
        with self.assertRaises(OSError):
            self.executor.step(self.plan['stages'][0])
        self.executor.abort()
        self.assertEqual([x[0] for x in self.arm.commands], ['MotionCtrl_2'])
        self.assertTrue(self.executor.failed)

    def test_physical_constructor_refuses_before_trace_or_adapter_use(self):
        output = Path(self.temp.name) / 'blocked'
        with self.assertRaisesRegex(RuntimeError, 'hold_unverified'):
            CONTROL.Executor(self.arm, self.plan, output, self.cameras)
        self.assertFalse(output.exists())
        self.assertEqual(self.arm.commands, [])

    def test_deleting_pause_marker_does_not_qualify_physical_execution(self):
        with mock.patch.object(CONTROL, 'EXECUTION_HOLD', Path(self.temp.name)/'absent'):
            with self.assertRaisesRegex(RuntimeError, 'hold_unverified'):
                CONTROL.require_execution_available()
            with self.assertRaisesRegex(RuntimeError, 'hold_unverified'):
                CONTROL.Executor(self.arm, self.plan, Path(self.temp.name), self.cameras)

    def test_nonphysical_opt_in_rejects_unmarked_or_sdk_capable_adapters(self):
        self.arm.nonphysical = False
        with self.assertRaisesRegex(RuntimeError, 'nonphysical test doubles'):
            CONTROL.Executor(self.arm, self.plan, Path(self.temp.name), self.cameras, nonphysical=True)
        self.arm.nonphysical = True
        self.arm.CreateCanBus = lambda: self.fail('must never be called')
        with self.assertRaisesRegex(RuntimeError, 'Hardware adapters'):
            CONTROL.Executor(self.arm, self.plan, Path(self.temp.name), self.cameras, nonphysical=True)

    def test_step_rechecks_offline_gate_before_sending(self):
        self.executor.nonphysical = False
        with self.assertRaisesRegex(RuntimeError, 'hold_unverified'):
            self.executor.step(self.plan['stages'][0])
        self.assertEqual(self.arm.commands, [])

    def test_camera_failure_during_action_latches_without_automatic_stop(self):
        def check():
            if self.arm.commands:
                raise RuntimeError('camera failed')
        self.cameras.check = check
        with self.assertRaisesRegex(RuntimeError, 'camera failed'):
            self.executor.step(self.plan['stages'][0])
        self.executor.abort()
        self.assertEqual([command[0] for command in self.arm.commands], ['MotionCtrl_2', 'EndPoseCtrl'])
        self.assertTrue(self.executor.failed)

    def test_passive_snapshot_errors_are_bounded_without_any_recovery(self):
        self.prepare_abort()
        self.arm.snapshot = lambda: (_ for _ in ()).throw(OSError('receiver lost'))
        self.executor.abort()
        self.assertEqual(self.arm.commands, [])
        self.assertEqual(len(self.executor.report['stop']['observation_errors']), 20)
        self.assertLess(self.clock.now, 103.1)


if __name__ == '__main__':
    unittest.main()
