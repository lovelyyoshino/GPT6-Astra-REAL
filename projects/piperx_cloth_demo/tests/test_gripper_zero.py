"""Closed empty jaw zero calibration: fake devices and real vendor encoder only."""
import copy
import json
from pathlib import Path
import sqlite3
import tempfile
from unittest.mock import patch
import unittest

from robot_tools import gripper_zero as zero
from robot_tools.pair_ledger import PairLedger, platform_state
from robot_tools.service import ToolService
from test_backend import PROFILE
import test_takeover


class Jaw:
    def __init__(self, robot): self.robot = robot
    def _send_msg(self, frame): self.robot.comm.send(frame)
    def calibrate_gripper(self, timeout):
        r = self.robot
        frame = r.can.Message(arbitration_id=0x159, is_extended_id=False, data=bytes.fromhex('00000000000000ae'))
        self._send_msg(r.frame_transform(frame))
        if r.duplicate: self._send_msg(frame)
        r.emit_ack()
        if r.ack:
            r.width = 0.
        return r.ack


class Robot(test_takeover.FakeRobot):
    def __init__(self, side):
        super().__init__(side)
        self.ctrl_mode=1; self.driver_enabled=[True]*6; self.gripper_enabled=False
        self.width=-.00546 if side=='right' else .03073; self.ack=True
        self.gripper=Jaw(self)
        self.comm.get_callback=lambda:self.callback
        self.emit_ack=lambda:None


class ZeroTests(test_takeover.TakeoverFixture):
    def setUp(self):
        super().setUp()
        self.robots={s:Robot(s) for s in ('left','right')}
        self.sdk.AgxArmFactory.create_arm.side_effect=self.robots.values()
        self.stack.enter_context(patch.object(zero,'time',self.clock))
        self.robots['right'].emit_ack=lambda:self.emit_ack() if self.robots['right'].ack else None

    def emit_ack(self, side='right', data='7501000000000000', offset=0., **kwargs):
        r=self.robots[side]
        r.callback(r.can.Message(arbitration_id=0x476, data=bytes.fromhex(data),
            timestamp=self.clock.time()+offset, is_extended_id=False, **kwargs))

    def snapshot(self, robot, gripper):
        state=super().snapshot(robot,gripper)
        for i in range(1,7): state['drivers'][str(i)]['foc_status']['driver_enable_status']=robot.driver_enabled[i-1]
        state['gripper']['foc_status']['driver_enable_status']=robot.gripper_enabled
        state['gripper']['width_m']=robot.width
        return state

    def execute(self):
        return zero._Zero(PROFILE,lambda e,d:self.events.append((e,d)),'right').run()

    def test_exact_single_frame_ack_zero_and_disabled(self):
        result=self.execute()
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['hardware_commands_sent'],1)
        self.assertEqual(result['before']['right']['gripper']['width_m'],-.00546)
        self.assertEqual(result['after_width_m'],0.)
        self.assertTrue(result['calibration_acknowledged'])
        self.assertFalse(result['after']['right']['gripper']['foc_status']['driver_enable_status'])
        self.assertEqual(self.robots['left'].sent,[])
        self.assertEqual(bytes(self.robots['right'].sent[0].data),bytes.fromhex('00000000000000ae'))
        self.assertGreaterEqual(self.clock.elapsed,6.)
        evidence=result['ack_evidence']
        self.assertEqual(evidence['classification']['status'],'matching_success_ack_observed')
        self.assertEqual(evidence['raw_frames'][0]['data_hex'],'7501000000000000')
        self.assertEqual(evidence['raw_frames'][0]['channel'],PROFILE['arms']['right']['channel'])

    def test_enabled_jaw_refuses_before_send(self):
        self.robots['right'].gripper_enabled=True
        self.assert_no_tx(self.execute())

    def test_offset_outside_calibration_envelope_refuses_before_send(self):
        self.robots['right'].width=-.011
        self.assert_no_tx(self.execute())

    def test_missing_ack_is_one_attempt_and_not_success(self):
        self.robots['right'].ack=False
        result=self.execute()
        self.assertFalse(result['ok']);self.assertEqual(result['hardware_commands_sent'],1)
        self.assertEqual(result['ack_evidence']['classification']['status'],'no_matching_ack_observed')

    def test_negative_ack_is_distinguished_from_unobserved_ack(self):
        r=self.robots['right'];r.ack=False
        r.emit_ack=lambda:self.emit_ack(data='7500000000000000')
        result=self.execute()
        self.assertFalse(result['ok'])
        self.assertEqual(result['ack_evidence']['classification']['status'],'matching_negative_ack_observed')
        self.assertEqual(result['hardware_commands_sent'],1)

    def test_raw_success_cannot_override_sdk_false(self):
        r=self.robots['right'];r.ack=False;r.emit_ack=lambda:self.emit_ack()
        result=self.execute()
        self.assertFalse(result['ok']);self.assertFalse(result['calibration_acknowledged'])
        self.assertEqual(result['ack_evidence']['classification']['status'],'matching_success_ack_observed')

    def test_sdk_true_requires_raw_ack(self):
        self.robots['right'].emit_ack=lambda:None
        result=self.execute()
        self.assertFalse(result['ok']);self.assertEqual(result['hardware_commands_sent'],1)

    def test_peer_ack_cannot_acknowledge_selected_jaw(self):
        self.robots['right'].emit_ack=lambda:self.emit_ack(side='left')
        self.assertFalse(self.execute()['ok'])

    def test_stale_ack_cannot_acknowledge(self):
        self.robots['right'].emit_ack=lambda:self.emit_ack(offset=-1.)
        self.assertFalse(self.execute()['ok'])

    def test_future_ack_cannot_acknowledge(self):
        self.robots['right'].emit_ack=lambda:self.emit_ack(offset=1.)
        self.assertFalse(self.execute()['ok'])

    def test_different_instruction_retained_without_acknowledging(self):
        self.robots['right'].emit_ack=lambda:self.emit_ack(data='7601000000000000')
        result=self.execute()
        self.assertFalse(result['ok'])
        self.assertEqual(len(result['ack_evidence']['classification']['rejected_frames']),1)

    def test_malformed_ack_cannot_acknowledge(self):
        self.robots['right'].emit_ack=lambda:self.emit_ack(data='7501')
        self.assertFalse(self.execute()['ok'])

    def test_conflicting_ack_retains_both_and_fails(self):
        self.robots['right'].emit_ack=lambda:(self.emit_ack(),self.emit_ack(data='7500000000000000'))
        result=self.execute()
        self.assertFalse(result['ok'])
        self.assertEqual(result['ack_evidence']['classification']['status'],'contradictory_matching_ack_observed')

    def test_trace_overflow_is_bounded_and_fails(self):
        self.robots['right'].emit_ack=lambda:[self.emit_ack() for _ in range(zero.ACK_TRACE_LIMIT+1)]
        result=self.execute()
        self.assertFalse(result['ok'])
        self.assertEqual(result['ack_evidence']['overflow_count'],1)
        self.assertEqual(len(result['ack_evidence']['raw_frames']),zero.ACK_TRACE_LIMIT)

    def test_receive_callback_does_not_wait_for_dispatch_lock(self):
        import threading
        def receive():
            thread=threading.Thread(target=self.emit_ack,daemon=True)
            thread.start();thread.join(1.)
            self.assertFalse(thread.is_alive(),'RX blocked on dispatch lock')
        self.robots['right'].emit_ack=receive
        self.assertTrue(self.execute()['ok'])

    def test_sdk_failure_after_ack_retains_trace_without_retry(self):
        def receive():
            self.emit_ack()
            raise RuntimeError('Synthetic SDK failure after reception')
        self.robots['right'].emit_ack=receive
        result=self.execute()
        self.assertFalse(result['ok']);self.assertEqual(result['hardware_commands_sent'],1)
        self.assertEqual(len(result['ack_trace_at_close']['raw_frames']),1)

    def test_source_timestamp_before_host_receipt_required(self):
        def receive():
            self.emit_ack(offset=.001)
            self.clock.sleep(.002)
        self.robots['right'].emit_ack=receive
        self.assertFalse(self.execute()['ok'])

    def test_sdk_completion_time_is_distinguished_from_send_return(self):
        def receive():
            self.emit_ack()
            self.clock.sleep(.02)
        self.robots['right'].emit_ack=receive
        result=self.execute();self.assertTrue(result['ok'],result)
        ack=result['ack_evidence']
        self.assertGreater(ack['sdk_returned_at_s'],ack['guarded_send_returned_at_s'])

    def test_receive_wrapper_preserves_manufacturer_callback(self):
        received=[]
        self.robots['right'].callback=lambda frame:received.append(bytes(frame.data))
        self.assertTrue(self.execute()['ok'])
        self.assertEqual(received,[bytes.fromhex('7501000000000000')])

    def test_duplicate_sdk_send_does_not_reach_bus_twice(self):
        self.robots['right'].duplicate=True
        result=self.execute()
        self.assertFalse(result['ok']);self.assertEqual(len(self.robots['right'].sent),1)

    def test_altered_command_rejected_before_bus(self):
        def change(frame):
            frame.data[6]=1
            return frame
        self.robots['right'].frame_transform=change
        self.assert_no_tx(self.execute())

    def test_peer_body_drift_refuses(self):
        self.hook=lambda r,s:s['joints_rad'].__setitem__(0,self.clock.elapsed*.01)
        self.assert_no_tx(self.execute())

    def test_real_vendor_calibration_encoder_and_ack_parser(self):
        from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.driver import Driver
        from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.parser import Parser
        from types import SimpleNamespace
        import can
        r=self.robots['right']
        parser=Parser(lambda *a,**kw:None)
        def request_and_get(**kw):
            kw['request']()
            # Synthetic controller ACK decoded using the manufacturer codec.
            from pyAgxArm.protocols.can_protocol.msgs.core.msg_abstract import MessageAbstract
            from pyAgxArm.protocols.can_protocol.msgs.piper.default import ArmMsgFeedbackRespSetInstruction
            parser.resp_set_instruction=MessageAbstract()
            parser.resp_set_instruction.msg=ArmMsgFeedbackRespSetInstruction()
            parser._codec.decode_476_resp_set_instruction(parser.resp_set_instruction.msg, bytearray.fromhex('7501000000000000'))
            self.emit_ack()
            r.width=0.
            return kw['get_value']() if kw['is_ready']() else None
        driver=object.__new__(Driver);driver._parser=parser
        driver._ctx=SimpleNamespace(_validate_timeout=lambda t:None,_request_and_get=request_and_get,get_comm=lambda:r.comm)
        r.gripper=driver
        result=self.execute()
        self.assertTrue(result['ok'],result)
        self.assertEqual(len(r.sent),1)


class ZeroManagerTests(ZeroTests):
    def setup_manager(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        (self.root/'runs').mkdir();(self.root/'configs').mkdir()
        (self.root/'configs/robot.json').write_text(json.dumps(PROFILE))
        self.path=self.root/'runs/pair_sessions.sqlite'
        PairLedger(self.path,'seed',{'task':'schema'},clock=self.clock.time)
        with sqlite3.connect(self.path) as db:db.execute('DELETE FROM pair_runs')
        self.service=ToolService(self.root)
        from contextlib import nullcontext
        self.stack.enter_context(patch.object(zero,'locks',lambda p:nullcontext()))
        self.stack.enter_context(patch.object(zero,'boot_identity',lambda:{'boot_id':'boot'}))

    def run_manager(self):
        return self.service.call('robot_calibrate_empty_gripper_zero_once',dict(arm='right',
            closed_empty_jaw_statement=zero.CONFIRMATION,empty_jaw_observation='Synthetic empty right jaw; no object'))

    def test_claim_latches_before_tx_and_success_clears_only_own_pending_gate(self):
        self.setup_manager()
        def check(frame):
            self.assertIsNotNone(platform_state(self.path)['fault'])
            return frame
        self.robots['right'].frame_transform=check
        result=self.run_manager();self.assertTrue(result['ok'],result)
        self.assertEqual(result['provenance']['profile']['arms'],PROFILE['arms'])
        self.assertEqual(len(result['request_sha256']),64)
        self.assertIsNone(platform_state(self.path)['fault'])
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            self.assertEqual(len(zero.audit_completed(db)),1)
        with self.assertRaisesRegex(RuntimeError,'already attempted'):
            self.run_manager()
        self.assertEqual(len(self.robots['right'].sent),1)

    def test_failure_latches_shared_platform_and_cannot_retry(self):
        self.setup_manager();self.robots['right'].ack=False
        self.assertFalse(self.run_manager()['ok'])
        self.assertIsNotNone(platform_state(self.path)['fault'])
        with self.assertRaisesRegex(RuntimeError,'Clean unowned'):
            self.run_manager()
        self.assertEqual(len(self.robots['right'].sent),1)

    def test_sdk_true_without_raw_ack_still_latches_platform(self):
        self.setup_manager();self.robots['right'].emit_ack=lambda:None
        self.assertFalse(self.run_manager()['ok'])
        self.assertIsNotNone(platform_state(self.path)['fault'])
        with self.assertRaisesRegex(RuntimeError,'Clean unowned'):
            self.run_manager()
        self.assertEqual(len(self.robots['right'].sent),1)
