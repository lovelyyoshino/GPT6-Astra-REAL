"""Narrow recovery of a completed opening: real host/device, fake CAN only.

The jaw receipt comes through PairHost and GuardedPairDevice; adjacent RGB
claims and the surrounding round are synthetic offline fixtures, not object
or physical stopping evidence. All real sockets remain forbidden.
"""
import copy
import hashlib
import json
import sqlite3
import unittest

from robot_tools import pair_device, pair_round
from robot_tools.grasp_store import GraspStore
from robot_tools.joint_path import evidence_sha256
from robot_tools.pair_host import PairHost
from robot_tools.pair_ledger import PairLedgerError
from test_pair_host import TASK
import test_pair_device as device_fixtures
import test_pair_rgb_round as round_fixtures


class RGBOpeningRoundTests(unittest.TestCase):
    def setUp(self):
        self.f = round_fixtures.RGBExpiryRoundTests('runTest'); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        GraspStore(self.f.ledger)
        device = device_fixtures.PairDeviceTests('runTest'); device.setUp(); self.addCleanup(device.doCleanups)
        profile = copy.deepcopy(device.profile)
        profile['cameras'] = dict(front='fake-front',left_wrist='fake-left',right_wrist='fake-right')
        host = PairHost(self.f.root/'jaw-source',profile,'jaw-source',copy.deepcopy(TASK),
            device_factory=lambda p,j,g:pair_device.GuardedPairDevice(p,j,g),
            clock=device.clock.time,background=False)
        self.addCleanup(host.close)
        host.open()
        rgb = dict(capture_id='jaw-test-capture',cameras={view:dict(serial=profile['cameras'][key],
            frame_number=1,host_received_at=device.clock.time()) for view,key in
            (('front','front'),('left_hand','left_wrist'),('right_hand','right_wrist'))})
        scene = host.observe(rgb)
        host.submit('jaw-source-event',scene['observation_id'],scene['peer_receipts']['right']['receipt_id'],
                    'left','gripper',.055,operation='approach')
        result = host.wait('jaw-source-event',timeout=5)
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual([frame.arbitration_id for frame in device.robots['left'].sent],[0x159])
        self.assertEqual(device.robots['right'].sent,[])
        with sqlite3.connect(host.ledger.path) as db:
            db.row_factory = sqlite3.Row
            event = dict(db.execute('SELECT * FROM pair_events WHERE event_id=?',('jaw-source-event',)).fetchone())
        host.close()
        self._insert_actual_opening(event)

    def _rehash(self,event,p,r):
        event['payload_json'] = json.dumps(p,sort_keys=True,separators=(',',':'))
        event['payload_digest'] = hashlib.sha256(event['payload_json'].encode()).hexdigest()
        event['receipt_json'] = json.dumps(r)
        return event

    def _images(self,event,received):
        p,r = json.loads(event['payload_json']),json.loads(event['receipt_json'])
        plan = r['joint_path_plan']; geometry = plan['geometry']; evidence = geometry['evidence']
        images = {}
        for view in ('front','left_hand','right_hand'):
            path = self.f.root/(event['event_id']+'-'+view+'.png')
            path.write_bytes(b'offline synthetic image bytes '+view.encode())
            images[view] = dict(rgb_path=str(path),artifact_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                host_received_at=received)
        evidence.update(saved_rgb_evidence=images,rgb_received_at=received)
        geometry['source']['sha256'] = evidence_sha256(evidence)
        plan['visual_rgb_deadline'] = received+30.
        plan['plan_sha256'] = evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'})
        r['original_event'].update(rgb_admission=copy.deepcopy(geometry),plan_sha256=plan['plan_sha256'])
        return self._rehash(event,p,r)

    def _insert_actual_opening(self,source):
        rows = self.f.f.rows()
        previous = copy.deepcopy(next(e for e in rows['pair_events'] if e['event_id']=='rgb-success'))
        previous = self._images(previous,4039.)
        self.event = copy.deepcopy(source)
        p,r = json.loads(source['payload_json']),json.loads(source['receipt_json'])
        shift = 4042.-source['began_at']; self.f.f.f.shift(r,shift)
        self.event.update(run_id='round-2',event_id='rgb-opening',owner=self.f.owner,step=5,
                          began_at=4042.,finished_at=source['finished_at']+shift)
        r.update(pair_owner=self.f.owner,event_id='rgb-opening')
        for side in ('left','right'):
            r['session_transmission_counts'][side] = copy.deepcopy(json.loads(previous['receipt_json'])['session_transmission_counts'][side])
        for key in ('attempted_frames','sent_frames'):r['session_transmission_counts']['left'][key] += 1
        self._rehash(self.event,p,r)
        # Complete a new unloaded segment after the real-shaped jaw receipt.
        following = json.loads(json.dumps(previous).replace('rgb-success','rgb-after-opening'))
        following.update(step=6,began_at=4052.,finished_at=4053.)
        q,t = json.loads(following['payload_json']),json.loads(following['receipt_json'])
        self.f.f.f.shift(t,12.)
        for key in ('attempted_frames','sent_frames'):t['session_transmission_counts']['left'][key] += 5
        following = self._images(self._rehash(following,q,t),4051.)
        failed = copy.deepcopy(next(e for e in rows['pair_events'] if e['event_id']=='rgb-failed'))
        failed.update(step=7,began_at=4142.,finished_at=4147.)
        q,t = json.loads(failed['payload_json']),json.loads(failed['receipt_json'])
        self.f.f.f.shift(t,100.)
        d = t['device_receipt']; plan = d['joint_path_plan']; geometry = plan['geometry']
        geometry['evidence']['rgb_received_at'] += 100.
        geometry['source']['sha256'] = evidence_sha256(geometry['evidence'])
        plan['visual_rgb_deadline'] += 100.
        plan['plan_sha256'] = evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'})
        d['original_event'].update(rgb_admission=copy.deepcopy(geometry),plan_sha256=plan['plan_sha256'])
        d['tracking_observation']['first_failure']['sample']['captured_at'] += 100.
        for key in ('attempted_frames','sent_frames'):d['session_transmission_counts']['left'][key] += 5
        failed = self._rehash(failed,q,t)
        self.f.receipt = t
        with sqlite3.connect(self.f.path) as db:
            db.execute("DELETE FROM pair_events WHERE run_id='round-2' AND step>=4")
            for event in (previous,self.event,following,failed):
                db.execute('INSERT INTO pair_events ('+','.join(event)+') VALUES ('+','.join('?' for _ in event)+')',tuple(event.values()))
            db.execute("UPDATE pair_runs SET steps=7 WHERE run_id='round-2'")
            db.execute("UPDATE pair_faults SET at=at+100 WHERE run_id='round-2'")
            db.execute("UPDATE pair_rounds SET last_time=last_time+100 WHERE run_id='round-2'")
        close = json.loads(self.f.close.read_text())
        close['cleanup']['session_transmission_counts'] = copy.deepcopy(d['session_transmission_counts'])
        self.f.close.write_text(json.dumps(close))
        for path in [*self.f.passive.values(),self.f.rgb]:
            data = json.loads(path.read_text()); self.f.f.f.shift(data,100.); path.write_text(json.dumps(data))

    def write(self,event):
        with sqlite3.connect(self.f.path) as db:
            db.execute('UPDATE pair_events SET payload_json=?,payload_digest=?,receipt_json=? WHERE event_id=?',
                       (event['payload_json'],event['payload_digest'],event['receipt_json'],event['event_id']))

    def test_actual_host_device_opening_allows_read_only_proposal_preserving_history(self):
        before = self.f.f.rows(); proposal = self.f.prepare()
        self.assertEqual(before,self.f.f.rows())
        self.assertEqual(proposal['snapshot']['run']['steps'],7)
        self.assertEqual(proposal['snapshot']['session_transmission_counts']['left']['sent_frames'],23)
        self.assertEqual(proposal['hardware_commands_sent'],0)
        self.assertFalse(proposal['dispatch_authorized'])
        self.assertIsNone(proposal['physical_stop_verified'])
        # Existing explicit post-repair authorization remains mandatory.
        auth = self.f.authorize(proposal); auth['received_at'] = 7599.
        with self.assertRaises(PairLedgerError):self.f.activate(proposal,auth)
        self.assertEqual(before,self.f.f.rows())

    def test_closing_probe_release_unknown_or_extra_transmissions_refuse(self):
        def indistinguishable_opening(p,r):
            before = r['before']['left']['gripper']['width_m']
            p['target'] = before+.000001
            r.update(requested_target=p['target'],observed_width_m=before+.000002,
                     width_error_m=abs(p['target']-(before+.000002)))
            r['after']['left']['gripper']['width_m'] = before+.000002
        changes = [lambda p,r:p.update(operation='grip_supported'),lambda p,r:p.update(operation='release_retreat'),
            lambda p,r:r.update(execution_mode='contact_probe'),lambda p,r:r.update(completion_mode='contact_probe_release'),
            lambda p,r:r.update(grasp_states={'left':{'status':'retained_static'},'right':None}),
            lambda p,r:r.update(requested_target=.01),lambda p,r:p.update(target=.01),
            lambda p,r:r['after']['left']['gripper'].update(width_m=.01),
            lambda p,r:r['frame_preflight_feedback'][0]['arms']['left']['gripper'].update(width_m=.056),
            lambda p,r:r['frame_preflight_feedback'].clear(),
            lambda p,r:r['frame_preflight_feedback'][0].update(side='right'),
            lambda p,r:r['transmission_counts']['left'].update(attempted_frames=2),
            lambda p,r:r['transmission_counts']['right'].update(attempted_frames=1,sent_frames=1),
            lambda p,r:r.update(arrival_confirmed=False),lambda p,r:r.update(observed_stable=False),
            lambda p,r:r.update(observed_stable_duration_s=0),lambda p,r:r.update(observed_feedback_advances=0),
            lambda p,r:r['observed_spans']['left'].update(jaw_m=.01),
            lambda p,r:r['observed_spans']['right'].update(joint_rad=.1),
            indistinguishable_opening,
            lambda p,r:r['after']['left']['gripper'].update(timestamp=r['before']['left']['gripper']['timestamp']),
            lambda p,r:r['after']['right']['gripper'].update(timestamp=r['before']['right']['gripper']['timestamp']),
            lambda p,r:r['after']['right'].update(timestamp=r['before']['right']['timestamp']),
            lambda p,r:r.update(pair_owner='other'),lambda p,r:r.update(observation_id='other')]
        for change in changes:
            event = copy.deepcopy(self.event); p,r = json.loads(event['payload_json']),json.loads(event['receipt_json'])
            change(p,r); self.write(self._rehash(event,p,r))
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.f.prepare()
        self.write(self.event)

    def test_missing_or_modified_unloaded_rgb_and_any_grasp_episode_refuse(self):
        with sqlite3.connect(self.f.path) as db:
            db.row_factory = sqlite3.Row
            original = dict(db.execute("SELECT * FROM pair_events WHERE event_id='rgb-after-opening'").fetchone())
        for change in (lambda p,r:p.update(unloaded_observation=''),
                       lambda p,r:r['joint_path_plan']['geometry']['evidence'].pop('saved_rgb_evidence'),
                       lambda p,r:r['joint_path_plan']['geometry']['evidence']['saved_rgb_evidence']['front'].update(host_received_at=4040.)):
            event = copy.deepcopy(original); p,r = json.loads(event['payload_json']),json.loads(event['receipt_json'])
            change(p,r)
            plan = r['joint_path_plan']; geometry = plan['geometry']
            geometry['source']['sha256'] = evidence_sha256(geometry['evidence'])
            plan['plan_sha256'] = evidence_sha256({k:v for k,v in plan.items() if k != 'plan_sha256'})
            self.write(self._rehash(event,p,r))
            with self.subTest(change=change),self.assertRaises(PairLedgerError):self.f.prepare()
        self.write(original)
        with sqlite3.connect(self.f.path) as db:
            db.execute('INSERT INTO pair_grasp_episodes VALUES (?,?,?,?,?,?)',
                       ('round-2','historical-grasp','left',self.f.owner,0,json.dumps(dict(status='empty'))))
        with self.assertRaisesRegex(PairLedgerError,'Any grasp episode'):self.f.prepare()


if __name__ == '__main__':unittest.main()
