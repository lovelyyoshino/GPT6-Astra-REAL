"""Synthetic maintenance-fault enrollment; no hardware or production ledger."""
import copy
import json
import sqlite3
import unittest
from unittest.mock import patch

from robot_tools import pair_round as entry
from robot_tools import configuration_recovery as recovery
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from robot_tools.pair_task_enrollment import preparation_only
import test_pair_tracking_round as fixtures
from test_joint_limits import reply_bytes


class ConfigurationRoundTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.TrackingRoundTests('runTest'); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root, self.path = self.f.root, self.f.path
        self.now = 8000.
        counts = {s:dict(attempted_frames=n,sent_frames=n,blocked_frames=0)
                  for s,n in (('left',8),('right',6))}
        self.receipt = dict(schema='piper_x_wrist_limit_maintenance_v1',ok=False,
            status='maintenance_failed_no_retry',event_id='rgb-failed',owner='maintenance-owner',run_id='round-2',
            arm_target_commands_sent=0,gripper_commands_sent=0,guard_violations=[],
            speed_changes_requested=False,zero_changes_requested=False,task_motion_authorized=False,
            errors=[dict(type='RuntimeError',detail='Cross-arm feedback skew exceeds limit')],
            began_at=4042.1,ended_at=4046.,writes=[dict(side='left',joint=4,arbitration_id=0x474,
                data_hex='04037afc867fff00',outcome='returned',requested_limits_tenth_deg=[-890,890],
                speed_field='unchanged_0x7fff',attempted_at=4044.,returned_at=4044.01)],
            transmission_counts=counts,session_transmission_counts=copy.deepcopy(counts),
            joint_limits={'left':{'4':dict(status='unconfirmed',raw_response_hex='',
                                           response_evidence={'response_frames':[]})}},before_limits={},
            cleanup=dict(requires_fault_latch=False,unresolved_gripper_probe=None,
                grasp_states={'left':None,'right':None},arms={s:{'status':'disconnected'} for s in counts},
                session_transmission_counts=copy.deepcopy(counts)))
        for side in counts:
            self.receipt['before_limits'][side] = {str(j):dict(raw_min_angle_tenth_deg=lo,
                raw_max_angle_tenth_deg=hi,raw_max_joint_spd=300) for j,(lo,hi) in
                enumerate([(-1500,1500),(0,1800),(-1700,0),(-1000,1000),(-700,700),(-1800,1800)],1)}
        payload = dict(schema=self.receipt['schema'],kind='manufacturer_configuration')
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row
            db.execute("UPDATE pair_events SET owner=?,payload_json=?,payload_digest=?,receipt_json=? WHERE event_id='rgb-failed'",
                ('maintenance-owner',json.dumps(payload),entry._sha(payload),json.dumps(self.receipt)))
            db.execute("DELETE FROM pair_faults WHERE run_id='round-2'")
            fault_id=db.execute("INSERT INTO pair_faults(run_id,owner,reason,at) VALUES(?,?,?,?)",
                ('round-2','maintenance-owner','execution_receipt_failed',4047.)).lastrowid
            db.execute("UPDATE pair_rounds SET owner=?,fault_id=? WHERE run_id='round-2'",('maintenance-owner',fault_id))
            fault=dict(db.execute('SELECT * FROM pair_faults WHERE id=?',(fault_id,)).fetchone())
        self.close=self.root/'maintenance-result.json'; self.close.write_text(json.dumps(self.receipt))
        self.capture=dict(ok=True,schema='piper_pair_controller_limits_capture_v1',source_event_id='rgb-failed',
            source_fault=fault,fault_cleared=False,task_dispatch_authorized=False,canonical_ledger_unchanged=True,
            guard_violations=[],errors=[],joint_limit_queries_sent=12,hardware_commands_sent=12,
            actuator_commands_sent=0,began_at=7900.,ended_at=7903.,joint_limits={})
        for side in counts:
            self.capture['joint_limits'][side]={}
            for j in range(1,7):
                b=self.receipt['before_limits'][side][str(j)]
                lo,hi=(-890,890) if (side,j)==('left',4) else (b['raw_min_angle_tenth_deg'],b['raw_max_angle_tenth_deg'])
                raw=reply_bytes(j,lo,hi,300).hex()
                self.capture['joint_limits'][side][str(j)] = dict(status='confirmed',raw_response_hex=raw,
                    raw_min_angle_tenth_deg=lo,raw_max_angle_tenth_deg=hi,raw_max_joint_spd=300,
                    response_evidence=dict(rejected_frames=[],request_started_unix_s=7900.1,
                        finished_unix_s=7902.,response_frames=[dict(valid_can_data_frame=True,payload_hex=raw,
                            timestamp=7901.,received_unix_s=7901.01)]))
        self.diag=self.root/'diagnostic.json';self.diag.write_text(json.dumps(self.capture))
        self.guard=patch('socket.socket',side_effect=AssertionError('No physical sockets in offline tests'))
        self.guard.start();self.addCleanup(self.guard.stop)

    def prepare(self,**overrides):
        args=dict(close_log=self.close,new_run_id='configuration-round',started_at=7990.,max_steps=500,
            max_duration_s=3600,budget_start_policy='after_repair_before_online_execution',
            parent_kind=recovery.KIND,clock=lambda:self.now,recovery_evidence=dict(passive_paths=self.f.passive,
                rgb_observation=self.f.rgb,visual_observation='Synthetic empty jaws and stationary table support.',
                configuration_diagnostic=self.diag))
        args.update(overrides);return entry.prepare_round(self.path,'round-2',**args)

    def activate(self,p):
        a=self.f.f.authorize(p)
        return entry.activate_round(p,a,project_root=self.root,clock=lambda:self.now)

    def test_preserves_every_old_row_requires_preparation_and_independent_readback(self):
        before=self.f.rows();p=self.prepare();self.assertEqual(before,self.f.rows())
        result=self.activate(p);after=self.f.rows()
        for name,rows in before.items():self.assertTrue(all(r in after[name] for r in rows),name)
        self.assertEqual(before['pair_faults'],after['pair_faults'])
        self.assertEqual(result['required_connection_mode'],'prepare')
        self.assertFalse(result['cache_or_limits_transferred'])
        self.assertTrue(preparation_only(self.path,'configuration-round'))
        self.assertTrue(activated_execution_budget(self.path,'configuration-round',max_steps=500,max_duration_s=3600))
        ledger=PairLedger(self.path,'configuration-round',result['new_contract'],max_steps=500,
                          max_duration_s=3600,clock=lambda:self.now)
        with self.assertRaises(PairLedgerError):ledger.claim('maintenance-owner')
        ledger.claim('new-preparation');ledger.begin('new-preparation','new-query',{'kind':'query'})
        ledger.finish('new-preparation','new-query',{'ok':True})
        self.assertTrue(activated_execution_budget(self.path,'configuration-round',max_steps=500,max_duration_s=3600))

    def test_unresolved_limit_changed_speed_and_nonquery_diagnostic_refuse(self):
        for mutation in (lambda d:d['joint_limits']['left']['4'].update(raw_min_angle_tenth_deg=-1000),
                         lambda d:d['joint_limits']['right']['2'].update(raw_max_joint_spd=301),
                         lambda d:d.update(actuator_commands_sent=1),
                         lambda d:d.update(fault_cleared=True),
                         lambda d:d['joint_limits']['left']['4']['response_evidence']['response_frames'][0].update(timestamp=1)):
            d=copy.deepcopy(self.capture);mutation(d);self.diag.write_text(json.dumps(d))
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError):self.prepare()

    def test_motion_other_fault_or_unreturned_setting_cannot_use_recovery(self):
        for mutation in (lambda d:d.update(arm_target_commands_sent=1),
                         lambda d:d['writes'][0].update(outcome='uncertain'),
                         lambda d:d['errors'][0].update(detail='driver fault'),
                         lambda d:d.update(speed_changes_requested=True)):
            d=copy.deepcopy(self.receipt);mutation(d)
            with sqlite3.connect(self.path) as db:
                db.execute("UPDATE pair_events SET receipt_json=? WHERE event_id='rgb-failed'",(json.dumps(d),))
            self.close.write_text(json.dumps(d))
            with self.subTest(mutation=mutation),self.assertRaises(PairLedgerError):self.prepare()

    def test_stale_current_observation_and_changed_historical_fault_refuse(self):
        p=self.prepare();self.now+=31
        with self.assertRaises(PairLedgerError):self.activate(p)
        self.now=8000.;self.activate(p)
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE pair_faults SET reason='altered' WHERE owner='maintenance-owner'")
        with self.assertRaises(PairLedgerError):
            activated_execution_budget(self.path,'configuration-round',max_steps=500,max_duration_s=3600)
