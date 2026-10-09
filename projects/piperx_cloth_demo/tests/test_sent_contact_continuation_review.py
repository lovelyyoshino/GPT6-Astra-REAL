"""Independent same-budget sent-contact continuation checks, with no hardware.

Host cases use the native adapter and FakeCAN, with synthetic archived source.
Opt-in archive cases only back up the canonical ledger read-only and mutate a
temporary copy; all enrollment, source and budget validators remain real.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_tools import host_recovery
from robot_tools import supported_gripper_recovery as recovery
from robot_tools.pair_ledger import PairLedger, PairLedgerError, activated_execution_budget
from robot_tools.retention_receipt import digest, measured_anchor
from test_host_grasp_integration import HostGraspFixture
import test_supported_contact_reacquisition_review as contact_review


class SentContactHostReviewTests(HostGraspFixture):
    request = contact_review.SupportedContactReacquisitionHostReviewTests.request
    probe = contact_review.SupportedContactReacquisitionHostReviewTests.probe
    assert_only_left_jaw_frames = contact_review.SupportedContactReacquisitionHostReviewTests.assert_only_left_jaw_frames

    def setUp(self):
        super().setUp()
        states = copy.deepcopy(self.host.device._previous)
        anchor = measured_anchor(states['left'])
        anchor['width_m'] = .046
        self.source = dict(before=states,
            candidate_measurement=dict(anchor=anchor, observed=dict(width_m=.046)),
            candidate_probe=dict(requested_width_m=.044, sent_at=self.clock.time()-40,
                                 completed_at=self.clock.time()-30, trace_sha256='a'*64))
        self.source['audited_supported_reacquisition'] = dict(
            schema='piper_supported_reacquisition_v1', arm='left',
            source_receipt_sha256=digest(self.source), opening_event_id='old-opening',
            opening_receipt_sha256='b'*64, opening_finished_at=self.clock.time()-10,
            audit_proposal_sha256='c'*64,
            opening_jaw_anchor=dict(width_m=.054, observed_at=self.clock.time()-11,
                source=dict(path='/synthetic/opening.json', sha256='d'*64)),
            completed_probe_continuation=dict(schema='piper_completed_contact_continuation_v1',
                arm='left', failed_event_id='original-failed-sent-probe',
                failed_receipt_sha256='e'*64, prior_sent_target_m=.05012,
                failed_finished_at=self.clock.time()-5, consumed_probe_count=2,
                remaining_probe_count=1,
                residual_jaw_anchor=dict(width_m=.05, observed_at=self.clock.time()-1,
                    source=dict(path='/synthetic/residual.json', sha256='f'*64))))
        self.proposal = dict(route='audited_contact_reacquisition', proposal_sha256='c'*64,
            snapshot={}, reacquisition=copy.deepcopy(self.source['audited_supported_reacquisition']),
            evidence=dict(passive=dict(left=dict(jaw_width_m=.05))), budget=dict(steps=0),
            max_probes=1, consumed_probe_count=2, prior_sent_target_m=.05012)
        self.old_payload = dict(arm='left', grasp_object_id='charger')
        self.stack.enter_context(patch.object(recovery, 'reacquisition_source',
            return_value=(self.old_payload, self.source)))
        self.stack.enter_context(patch.object(host_recovery.SupportedRecovery, 'state',
            lambda _: self.recovery_state()))
        self.probes = []
        self.contact_hook = self.hook

    def recovery_state(self):
        with sqlite3.connect(self.host.runs/'pair_sessions.sqlite') as db:
            db.row_factory = sqlite3.Row
            events = list(db.execute('SELECT * FROM pair_events WHERE run_id=? ORDER BY step',
                                     (self.host.run_id,)))
            return recovery._contact_runtime(db, self.host.run_id,
                dict(owner=self.host.owner), self.proposal, events)

    def test_new_candidate_retains_zero_tx_without_unlocking_body_or_right(self):
        frozen = copy.deepcopy(self.source)
        _, result = self.probe()
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(result['receipt']['supported_probe_index'], 3)
        self.assertEqual(self.recovery_state()['phase'], 'contact_candidate')
        retained, _ = self.retain('left')
        self.assertEqual(retained['hardware_commands_sent'], 0)
        self.assertFalse(retained['loaded_contact_available'])
        with self.assertRaises(RuntimeError):
            self.host.inspect_joint_limits('forbidden-query')
        for changes in (dict(arm='right'), dict(operation='extract_segment')):
            with self.subTest(changes=changes), self.assertRaises((ValueError, RuntimeError)):
                self.service.call('robot_pair_submit_once', self.request('forbidden', **changes))
        self.assert_only_left_jaw_frames(1)
        self.assertEqual(self.source, frozen)

    def test_last_arrival_exhausts_allowance_and_old_failed_target_never_replays(self):
        for target in (.05012, .0501196):
            with self.subTest(target=target), self.assertRaises(PairLedgerError):
                recovery._check_contact_payload(dict(arm='left', kind='gripper', operation='grip_supported',
                    target=target, grasp_object_id='charger', reacquisition_proposal_sha256='c'*64,
                    probe_support_relation='independent_support_present', probe_support_observation='Synthetic support'),
                    self.proposal, .051, .05012, self.old_payload)
        _, result = self.probe(contact=None)
        self.assertEqual(result['status'], 'completed', result.get('receipt'))
        self.assertEqual(self.recovery_state()['phase'], 'exhausted')
        with self.assertRaises((ValueError, RuntimeError)):
            self.service.call('robot_pair_submit_once', self.request('fourth-probe', target=.041))
        self.assertIsNone(self.host.ledger.event('fourth-probe'))
        self.assert_only_left_jaw_frames(1)

    def test_refresh_precedes_claim_and_preserves_the_last_attempt(self):
        request = self.request()
        self.clock.sleep(27)
        result = self.service.call('robot_pair_submit_once', request)
        self.assertEqual(result['status'], 'refresh_required')
        self.assertIsNone(self.host.ledger.event(request['event_id']))
        self.assertEqual(self.host.ledger.status()['steps'], 0)
        self.assertEqual(self.recovery_state()['probe_count'], 0)
        self.assertIsNone(self.host.grasps.active('left'))
        self.assertFalse(self.host.status()['fault_latched'])
        self.assert_only_left_jaw_frames(0)
        _, completed = self.probe('fresh-last-attempt')
        self.assertEqual(completed['status'], 'completed', completed.get('receipt'))
        self.assert_only_left_jaw_frames(1)

    def test_new_failed_frame_stays_faulted_without_candidate_or_retry(self):
        self.robots['left'].fail_id = 0x159
        _, result = self.probe()
        self.assertEqual(result['status'], 'fault')
        self.assertEqual(self.recovery_state()['phase'], 'unresolved')
        self.assertTrue(self.host.status()['fault_latched'])
        before = len(self.robots['left'].sent)
        with self.assertRaises((ValueError, RuntimeError)):
            self.service.call('robot_pair_submit_once', self.request('retry'))
        self.assertEqual(len(self.robots['left'].sent), before)
        self.assertEqual(self.robots['right'].sent, [])


@unittest.skipUnless(os.environ.get('PIPER_RUN_ARCHIVED_CONTACT_TESTS') == '1',
                     'Explicit opt-in for a read-only archived ledger backup')
class SentContactArchivedReviewTests(unittest.TestCase):
    def setUp(self):
        location = Path('artifacts/contact_anchor_continue_20261008/offline_fixture.py')
        spec = importlib.util.spec_from_file_location('sent_contact_review_fixture', location)
        fixture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fixture)
        context = fixture.copied_fixture()
        self.f = context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        self.path = self.f['path']
        self.assertNotEqual(self.path.resolve(), Path('projects/piperx_cloth_demo/runs/pair_sessions.sqlite').resolve())
        self.before = self.rows()

    def rows(self):
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            return {name:[dict(r) for r in db.execute('SELECT * FROM '+name)]
                for name, in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'pair_%'").fetchall()}

    def activate(self, proposal=None, now=None):
        return recovery.activate_sent_contact_continuation(
            self.f['proposal'] if proposal is None else proposal, project_root=self.f['root'],
            clock=lambda:self.f['now'] if now is None else now)

    def test_full_original_history_prepare_host_budget_and_old_owner_isolation(self):
        from robot_tools.pair_host import PairHost
        from test_pair_host import FakePairDevice
        activated = self.activate()
        after = self.rows()
        for name, rows in self.before.items():
            self.assertTrue(all(row in after[name] for row in rows), name)
            if name != recovery.CONTACT_TABLE:
                self.assertEqual(rows, after[name], name)
        p = self.f['proposal']; run = p['run_id']; budget = p['budget']
        self.assertFalse(activated['new_budget_allocated'])
        self.assertTrue(activated_execution_budget(self.path,run,max_steps=budget['max_steps'],max_duration_s=budget['max_duration_s']))
        ledger = PairLedger(self.path,run,activated['new_contract'],max_steps=budget['max_steps'],
            max_duration_s=budget['max_duration_s'],clock=lambda:self.f['now'])
        with self.assertRaises(PairLedgerError):
            ledger.claim(p['snapshot']['scope']['owner'])
        devices=[]; ticks=[self.f['now']]
        clock=SimpleNamespace(time=lambda:ticks[0],sleep=lambda dt:ticks.__setitem__(0,ticks[0]+dt))
        class PreparationDevice(FakePairDevice):
            def connect_for_preparation(inner):
                return {**inner.open(),'task_ready':False,'readiness':{'synthetic_fixture':True}}
        def factory(*args):
            device=PreparationDevice(*args,clock);devices.append(device);return device
        profile=json.loads((self.f['root']/'configs/robot.json').read_text())
        args=dict(device_factory=factory,clock=clock.time,background=False)
        with self.assertRaises(ValueError):
            PairHost(self.f['root']/'runs',profile,run,activated['new_contract']['task'],
                budget['max_steps'],budget['max_duration_s'],connection_mode='ready',**args)
        self.assertEqual(devices,[])
        host=PairHost(self.f['root']/'runs',profile,run,activated['new_contract']['task'],
            budget['max_steps'],budget['max_duration_s'],connection_mode='prepare',**args)
        self.addCleanup(host.close)
        opened=host.open()
        self.assertTrue(opened['open']);self.assertFalse(opened['task_ready'])
        self.assertEqual(host.deadline,budget['deadline_s'])
        self.assertEqual(host.ledger.status()['steps'],1)
        with sqlite3.connect(self.path) as db:
            db.row_factory=sqlite3.Row; state=recovery.runtime(db,run,host.owner)
        self.assertEqual(state['proposal']['max_probes'],1)
        self.assertEqual(state['consumed_probe_count'],2)
        self.assertEqual(state['prior_target_m'],.034)
        self.assertLess(state['current_width_m'],.034)
        from robot_tools.pair_device import _validated_supported_reacquisition
        _, source=recovery.reacquisition_source(state)
        envelope=_validated_supported_reacquisition(source,'left',clock.time())
        self.assertEqual(envelope['opening_jaw_anchor']['width_m'],.03836)
        self.assertEqual(envelope['completed_probe_continuation']['residual_jaw_anchor']['width_m'],.03388)
        self.assertEqual(envelope['completed_probe_continuation']['consumed_probe_count'],2)
        with self.assertRaises(RuntimeError):host.inspect_joint_limits('no-query')
        self.assertEqual(devices[0].calls,[]);self.assertEqual(devices[0].frame_attempts,0)

    def test_changed_proposal_budget_authority_and_deadline_never_append(self):
        p=self.f['proposal']
        for key, value in (('max_probes',2),('consumed_probe_count',1),('new_budget_allocated',True)):
            changed=copy.deepcopy(p);changed[key]=value
            changed['proposal_sha256']=recovery._sha({k:v for k,v in changed.items() if k!='proposal_sha256'})
            with self.subTest(key=key),self.assertRaises(PairLedgerError):self.activate(changed)
            self.assertEqual(self.rows(),self.before)
        with self.assertRaises(PairLedgerError):self.activate(now=p['budget']['deadline_s'])
        self.assertEqual(self.rows(),self.before)

    def test_original_body_multi_frame_or_candidate_mutations_block_activation(self):
        event=self.f['proposal']['snapshot']['event'];original=json.loads(event['receipt_json'])
        changed=copy.deepcopy(original);changed['device_receipt']['after']['left']['joints_rad'][3]+=.1
        more=copy.deepcopy(original);more['device_receipt']['transmission_counts']['left']['sent_frames']=2
        candidate=copy.deepcopy(original);candidate['device_receipt']['candidate_probe']={'forged':True}
        for receipt in (changed,more,candidate):
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND event_id=?',
                           (json.dumps(receipt),event['run_id'],event['event_id']))
            with self.assertRaises(PairLedgerError):self.activate()
            with sqlite3.connect(self.path) as db:
                db.execute('UPDATE pair_events SET receipt_json=? WHERE run_id=? AND event_id=?',
                           (event['receipt_json'],event['run_id'],event['event_id']))
        self.assertEqual(self.rows(),self.before)
