"""Failure admission and ordered mechanical recovery, no physical I/O."""
import copy
import json
import sqlite3
import unittest
from robot_tools import supported_gripper_recovery as recovery
from robot_tools.contact_receipt import classify_gripper_probe
from robot_tools.pair_ledger import PairLedgerError
from test_contact_receipt import sample
import test_manual_gripper_continuation as manual_test


class SupportedRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.f = manual_test.ManualGripperContinuationTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        c = classify_gripper_probe(arm='right', requested_width_m=.025, sent_at=103.02,
            baseline_samples=[sample(100+i*.05) for i in range(61)],
            post_samples=[sample(103.07+i*.05,.028,force=.2) for i in range(81)])
        self.assertEqual(c['outcome'], 'settled_contact_candidate')
        c['trace_summary'] = {'sha256':'a'*64}
        self.f.receipt['device_receipt'].update(contact_observation=c,
            errors=[dict(type='RuntimeError',detail='Single action requires receive age/skew within 100 ms including processing; age=0.148015 skew=0.100401')],
            candidate_probe=dict(requested_width_m=.025,observed_width_m=.028,outcome=c['outcome'],trace_sha256='a'*64),
            candidate_measurement=dict(observed=dict(width_m=.028)))
        self.f.refresh()

    def validate(self):
        self.f.refresh()
        return recovery.validate_failed_probe(self.f.event,self.f.run,'old-owner')

    def test_final_freshness_failure_is_history_not_success(self):
        p,d=self.validate()
        self.assertFalse(d['ok'])
        self.assertFalse(d['contact_observation']['grasp_verified'])
        self.assertEqual(p['target'],.025)

    def test_other_fault_partial_send_retry_force_change_or_success_refused(self):
        original=copy.deepcopy(self.f.receipt)
        for key,value in [('errors',[dict(type='RuntimeError',detail='driver overcurrent')]),
                          ('hardware_commands_sent',0),('retries',1),('nominal_force_N',1),('ok',True)]:
            self.f.receipt=copy.deepcopy(original)
            self.f.receipt['device_receipt'][key]=value
            with self.subTest(key=key),self.assertRaises(PairLedgerError):self.validate()
        self.f.receipt=original;self.f.event['success']=1
        with self.assertRaises(PairLedgerError):self.validate()

    def test_candidate_width_or_trace_cannot_be_replaced(self):
        for key,value in [('observed_width_m',.029),('trace_sha256','b'*64)]:
            before=copy.deepcopy(self.f.receipt)
            self.f.receipt['device_receipt']['candidate_probe'][key]=value
            with self.subTest(key=key),self.assertRaises(PairLedgerError):self.validate()
            self.f.receipt=before

    def test_explicit_repair_instruction_required(self):
        a=dict(source='user_message',message_id='operator-repair',statement='I confirm the grip; fix this and extract.',
               recorded_at=110.,decision='authorize_supported_gripper_recovery')
        self.assertEqual(recovery.authorization(a,{},dict(ended_at=109.),111.),a)
        with self.assertRaises(PairLedgerError):
            recovery.authorization({**a,'decision':'automatic_retry'}, {},dict(ended_at=109.),111.)

    def test_runtime_requires_open_then_confirm_and_blocks_repetition(self):
        db=sqlite3.connect(':memory:');db.row_factory=sqlite3.Row;self.addCleanup(db.close)
        db.executescript("CREATE TABLE pair_rounds(ordinal INTEGER,run_id TEXT,record_json TEXT); INSERT INTO pair_rounds VALUES(1,'run','{\"proposal\":{}}');"
            "CREATE TABLE pair_supported_gripper_recoveries(run_id TEXT,owner TEXT,round_ordinal INTEGER,record_json TEXT);"
            "CREATE TABLE pair_events(run_id TEXT,step INTEGER,owner TEXT,payload_json TEXT,status TEXT,success INTEGER,receipt_json TEXT);")
        p=dict(snapshot={},budget=dict(steps=13))
        db.execute('INSERT INTO pair_supported_gripper_recoveries VALUES(?,?,?,?)',('run','new-owner',1,json.dumps(dict(proposal=p))))
        def allowed(kind):recovery.check_request(db,'run','new-owner',dict(kind=kind))
        with self.assertRaises(PairLedgerError):allowed('joint')
        allowed('supported_recovery_open')
        db.execute('INSERT INTO pair_events VALUES(?,?,?,?,?,?,?)',('run',14,'new-owner',json.dumps(dict(kind='supported_recovery_open')),'pending',None,None))
        with self.assertRaises(PairLedgerError):allowed('supported_recovery_open')
        with self.assertRaises(PairLedgerError):allowed('supported_recovery_confirm')
        r=dict(ok=True,hardware_commands_sent=1,physical_stop_verified=None)
        db.execute("UPDATE pair_events SET status='complete',success=1,receipt_json=?",(json.dumps(r),))
        allowed('supported_recovery_confirm')
        with self.assertRaises(PairLedgerError):allowed('query')
        db.execute('INSERT INTO pair_events VALUES(?,?,?,?,?,?,?)',('run',15,'new-owner',json.dumps(dict(kind='supported_recovery_confirm')),'complete',1,json.dumps({**r,'hardware_commands_sent':0})))
        allowed('query');allowed('joint')
        with self.assertRaises(PairLedgerError):allowed('supported_recovery_open')
        with self.assertRaises(PairLedgerError):recovery.runtime(db,'run','old-owner')
