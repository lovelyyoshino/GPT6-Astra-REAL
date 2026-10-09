"""Recorded partial-batch replay through the real vendor decoder, no CAN I/O."""
import copy
import json
import math
from pathlib import Path
import struct
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from robot_tools import arms, coherent_feedback as cf, contact_receipt
from test_arms import FakeRobot, SDK_PATH
from test_contact_receipt import sample

RECORDED=json.loads((Path(__file__).parent/'fixtures/mixed_pose_fragments.json').read_text())
POLICY={'profile':'right_j4_bounded_v1','source':'user','statement':'Synthetic test authorization'}


class CoherentFeedbackTests(unittest.TestCase):
    def setUp(self):
        arms._load_sdk(SDK_PATH)
        import can
        from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Parser
        self.can=can;self.robot=FakeRobot();old=self.robot._parser
        self.robot._parser=Parser(NS(add_variable=lambda *a:None,increment=lambda *a:None))
        for name in ('arm_status',)+arms.DRIVERS:setattr(self.robot._parser,name,getattr(old,name))
        self.now=RECORDED['anchor']['timestamp']+.01
        self.clock=patch.object(arms.time,'time',side_effect=lambda:self.now);self.clock.start();self.addCleanup(self.clock.stop)
        block=patch('socket.socket',side_effect=AssertionError('Offline only'));block.start();self.addCleanup(block.stop)
        self.grouped=cf.install(self.robot)

    def frame(self,group,index,values,stamp):
        can_id,_,_=cf.GROUPS[group][index]
        if group=='joints':raw=[round(v*180000/math.pi) for v in values]
        else:
            offset=index*2
            raw=[round(v*(1e6 if offset+n<3 else 180000/math.pi)) for n,v in enumerate(values)]
        return self.can.Message(arbitration_id=can_id,data=struct.pack('>ii',*raw),
                                timestamp=stamp,is_extended_id=False)

    def feed_record(self,record,groups=('pose','joints')):
        for group in groups:
            values=record['pose_m_rad' if group=='pose' else 'joints_rad']
            for i,(_,name,_) in enumerate(cf.GROUPS[group]):
                self.robot._parser.parse_packet(self.frame(group,i,values[2*i:2*i+2],record['fragment_timestamps_s'][name]))

    def snapshot(self):return arms.snapshot(self.robot,self.robot._effector)

    def test_recorded_mixed_pose_never_becomes_qualified_rotation(self):
        self.feed_record(RECORDED['anchor']);before=self.snapshot()
        for i,(_,name,_) in enumerate(cf.GROUPS['pose'][:2]):
            bad=RECORDED['mixed'];self.robot._parser.parse_packet(self.frame('pose',i,bad['pose_m_rad'][2*i:2*i+2],bad['fragment_timestamps_s'][name]))
        self.now=RECORDED['mixed']['timestamp']+.001
        partial=self.snapshot()
        self.assertEqual(partial['pose_m_rad'],before['pose_m_rad'])
        self.assertEqual(partial['fragment_timestamps_s']['end_pose_ryrz'],before['fragment_timestamps_s']['end_pose_ryrz'])
        raw=partial['feedback_assembly']['raw_fragment_cache']['pose']
        mixed=[v for part in raw for v in part['values']]
        self.assertAlmostEqual(math.degrees(contact_receipt._rotation_span([before['pose_m_rad'],mixed])),.962,places=6)
        self.feed_record(RECORDED['coherent_after']);self.now=RECORDED['coherent_after']['timestamp']+.001
        after=self.snapshot()
        for a,b in zip(after['pose_m_rad'],RECORDED['coherent_after']['pose_m_rad']):self.assertAlmostEqual(a,b,places=10)
        self.assertLess(math.degrees(contact_receipt._rotation_span([before['pose_m_rad'],after['pose_m_rad']])),.5)
        self.assertEqual(len(after['feedback_assembly']['incomplete_groups']),1)
        self.assertEqual(len(after['feedback_assembly']['incomplete_groups'][0]['frames']),2)
        self.assertEqual(after['feedback_assembly']['groups']['pose']['sequence'],2)
        self.robot.comm.send.assert_not_called();self.robot.comm.send_bus.send.assert_not_called()

    def test_missing_or_late_group_stays_old_and_becomes_stale(self):
        self.feed_record(RECORDED['anchor']);before=self.snapshot()
        start=self.now+.1;pose=RECORDED['anchor']['pose_m_rad']
        for i in (0,2):self.robot._parser.parse_packet(self.frame('pose',i,pose[2*i:2*i+2],start+i*.0001))
        self.now=start+1.;after=self.snapshot()
        self.assertEqual(after['pose_m_rad'],before['pose_m_rad'])
        self.assertEqual(after['fragment_timestamps_s']['end_pose_xy'],before['fragment_timestamps_s']['end_pose_xy'])
        self.assertIn('end_pose_xy',after['stale_fragments'])
        self.assertFalse(arms.control_health(after)['healthy'])
        self.assertFalse(after['feedback_assembly']['timestamps_renewed'])
        self.assertEqual(len(after['feedback_assembly']['incomplete_groups']),2)

    def test_group_receive_span_is_bounded(self):
        pose=RECORDED['anchor']['pose_m_rad'];at=self.now
        for i in range(3):self.robot._parser.parse_packet(self.frame('pose',i,pose[2*i:2*i+2],at+i*.005))
        s=self.snapshot();self.assertIsNone(s['pose_m_rad']);self.assertEqual(s['status'],'partial')
        self.assertEqual(s['feedback_assembly']['groups']['pose']['sequence'],0)

    def test_regressing_or_invalid_fragment_latches_reader_error(self):
        self.feed_record(RECORDED['anchor'])
        spec=cf.GROUPS['pose'][0];rec=RECORDED['anchor']
        frame=self.frame('pose',0,rec['pose_m_rad'][:2],rec['fragment_timestamps_s'][spec[1]])
        self.robot._parser.parse_packet(frame)
        self.feed_record(RECORDED['coherent_after'])
        result=self.snapshot()
        self.assertIn('did not advance',result['feedback_assembly']['error'])
        self.assertIsNone(result['pose_m_rad']);self.assertEqual(result['status'],'partial')
        self.assertEqual(result['feedback_assembly']['failure_evidence']['data_hex'],bytes(frame.data).hex())

    def test_invalid_body_frames_are_saved_without_entering_native_decoder(self):
        for mutation in ('short','extended','remote','error','fd','nan','zero'):
            with self.subTest(mutation=mutation):
                parser=type(self.robot._parser)(NS(add_variable=lambda *a:None,increment=lambda *a:None))
                grouped=cf.CoherentFeedback(parser)
                frame=self.frame('pose',0,RECORDED['anchor']['pose_m_rad'][:2],self.now)
                if mutation=='short':frame.data=frame.data[:7];frame.dlc=7
                elif mutation=='nan':frame.timestamp=float('nan')
                elif mutation=='zero':frame.timestamp=0.
                else:setattr(frame,{'extended':'is_extended_id','remote':'is_remote_frame',
                                   'error':'is_error_frame','fd':'is_fd'}[mutation],True)
                with patch.object(grouped,'original',side_effect=AssertionError('Invalid frame reached decoder')):
                    parser.parse_packet(frame)
                _,evidence=grouped.snapshot(arms.PARTS+arms.DRIVERS)
                self.assertIn('Invalid feedback',evidence['error'])
                self.assertEqual(evidence['failure_evidence']['data_hex'],bytes(frame.data).hex())
                json.dumps(evidence,allow_nan=False)

    def test_decoder_exception_keeps_first_error_and_cannot_publish_later_samples(self):
        self.feed_record(RECORDED['anchor'])
        frame=self.frame('pose',0,RECORDED['anchor']['pose_m_rad'][:2],self.now)
        with patch.object(self.grouped,'original',side_effect=ValueError('decode failed')):
            with self.assertRaisesRegex(ValueError,'decode failed'):self.robot._parser.parse_packet(frame)
        self.feed_record(RECORDED['coherent_after'])
        result=self.snapshot()
        self.assertIn('decoder failed: ValueError',result['feedback_assembly']['error'])
        self.assertIsNone(result['pose_m_rad']);self.assertIsNone(result['joints_rad'])

    def test_joint_group_waits_for_six_axes_without_blocking_status_faults(self):
        self.feed_record(RECORDED['anchor']);before=self.snapshot()
        rec=RECORDED['coherent_after']
        for i,(_,name,_) in enumerate(cf.GROUPS['joints'][:2]):
            self.robot._parser.parse_packet(self.frame('joints',i,rec['joints_rad'][2*i:2*i+2],rec['fragment_timestamps_s'][name]))
        partial=self.snapshot()
        self.assertEqual(partial['joints_rad'],before['joints_rad'])
        self.assertEqual(partial['fragment_timestamps_s']['joint_34'],before['fragment_timestamps_s']['joint_34'])
        name=cf.GROUPS['joints'][2][1]
        self.robot._parser.parse_packet(self.frame('joints',2,rec['joints_rad'][4:],rec['fragment_timestamps_s'][name]))
        after=self.snapshot()
        for a,b in zip(after['joints_rad'],rec['joints_rad']):self.assertAlmostEqual(a,b,places=10)
        self.robot._parser.arm_status.msg.err_code=1
        fault=self.snapshot()
        self.assertEqual(fault['arm_status']['err_code'],1)
        self.assertFalse(arms.control_health(fault)['healthy'])

    def test_registered_vendor_callback_uses_the_grouped_decoder(self):
        from pyAgxArm.protocols.can_protocol.drivers.core.submodel_driver_context_abstract import SubmodelDriverContextAbstract
        calls=[]
        ctx=NS(_parser=self.robot._parser,_ctx=NS(fps=NS(increment=calls.append)),FPS_DATA_MONITOR='test')
        record=RECORDED['anchor']
        for i,(_,name,_) in enumerate(cf.GROUPS['pose']):
            frame=self.frame('pose',i,record['pose_m_rad'][2*i:2*i+2],record['fragment_timestamps_s'][name])
            SubmodelDriverContextAbstract.parse_packet(ctx,frame)
        self.assertEqual(self.snapshot()['feedback_assembly']['groups']['pose']['sequence'],1)
        self.assertEqual(calls,['test']*3)

    def test_true_complete_pose_excess_still_fails_frozen_guard(self):
        self.feed_record(RECORDED['anchor']);origin=self.snapshot()
        pose=origin['pose_m_rad'][:];pose[3]+=math.radians(.6)
        at=self.now+.01
        for i in range(3):self.robot._parser.parse_packet(self.frame('pose',i,pose[2*i:2*i+2],at+i*.0001))
        self.now=at+.001;changed=self.snapshot()
        self.assertGreater(math.degrees(contact_receipt._rotation_span([origin['pose_m_rad'],changed['pose_m_rad']])),.5)
        trace=[sample(100.),sample(103.)]
        trace[0]['arms']['right']['pose_m_rad']=origin['pose_m_rad']
        trace[1]['arms']['right']['pose_m_rad']=changed['pose_m_rad']
        with self.assertRaises(ValueError):contact_receipt._stable(trace,feedback_policy=POLICY)

    def test_consumer_cannot_copy_in_place_decoder_mutation(self):
        self.feed_record(RECORDED['anchor']);before=self.snapshot();entered=threading.Event();release=threading.Event()
        real=self.grouped.original;results=[];errors=[]
        def blocked(frame):
            r=real(frame);entered.set()
            if not release.wait(2):raise RuntimeError('test receive timeout')
            return r
        self.grouped.original=blocked
        def run(call):
            try:results.append(call())
            except Exception as e:errors.append(e)
        rec=RECORDED['mixed'];f=self.frame('pose',0,rec['pose_m_rad'][:2],rec['fragment_timestamps_s']['end_pose_xy'])
        writer=threading.Thread(target=lambda:run(lambda:self.robot._parser.parse_packet(f)))
        reader=threading.Thread(target=lambda:run(self.snapshot));writer.start()
        try:
            self.assertTrue(entered.wait(1));reader.start();self.assertEqual(results,[])
        finally:release.set();writer.join(2);reader.join(2)
        self.assertFalse(writer.is_alive());self.assertFalse(reader.is_alive());self.assertEqual(errors,[])
        copied=next(r for r in results if isinstance(r,dict))
        self.assertEqual(copied['pose_m_rad'],before['pose_m_rad'])

    def test_unfinished_evidence_overflow_is_explicit_not_silent_loss(self):
        rec=RECORDED['anchor'];at=self.now
        for i in range(cf.MAX_INCOMPLETE_RECORDS+2):
            self.robot._parser.parse_packet(self.frame('pose',0,rec['pose_m_rad'][:2],at+i*.005))
        result=self.snapshot();self.assertIn('buffer exhausted',result['feedback_assembly']['error'])
        self.assertEqual(len(result['feedback_assembly']['incomplete_groups']),cf.MAX_INCOMPLETE_RECORDS)
        self.assertIsNone(result['pose_m_rad'])

    def test_reader_requires_new_complete_frames_and_cannot_install_twice(self):
        result=self.snapshot();self.assertIsNone(result['pose_m_rad']);self.assertIsNone(result['joints_rad'])
        with self.assertRaises(RuntimeError):cf.install(self.robot)
