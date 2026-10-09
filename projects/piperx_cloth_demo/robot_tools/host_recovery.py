"""Host integration for an explicitly audited, supported mechanical recovery.

No old grasp is promoted. Opening/separation and independently supported
reacquisition are distinct routes; neither transfers a joint target cache.
"""
import copy
import json
import sqlite3

from .pair_ledger import _identifier
from .supported_gripper_recovery import runtime, recovery_source


class SupportedRecovery:
    def __init__(self, host):
        self.host = host

    def state(self):
        host = self.host
        path = (host.runs / 'pair_sessions.sqlite').resolve()
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            db.row_factory = sqlite3.Row
            return runtime(db, host.run_id, host.owner)

    def require_resolved(self):
        state = self.state()
        if state is not None and state['phase'] != 'resolved':
            raise RuntimeError('Supported jaw recovery must finish before ordinary preparation or motion')

    def admit_probe(self, arm, kind, operation, object_id, description, relation):
        state = self.state()
        if state is None or state['proposal'].get('route') != 'audited_contact_reacquisition':
            self.require_resolved()
            return None
        from .supported_gripper_recovery import reacquisition_source
        source, _ = reacquisition_source(state)
        if (state['phase'] != 'reacquire_required' or arm != source['arm']
                or kind != 'gripper' or operation != 'grip_supported'
                or object_id != source.get('grasp_object_id')):
            raise RuntimeError('Reacquisition admits only a new supported probe on its frozen arm/object')
        if (type(description) is not str or not 1 <= len(description.strip()) <= 4000
                or '\x00' in description or relation != 'independent_support_present'):
            raise ValueError('Describe the current independent object support before reacquisition')
        return state

    def execute_probe(self, payload):
        state = self.state()
        if state is None or state['proposal'].get('route') != 'audited_contact_reacquisition':
            if payload.get('reacquisition_proposal_sha256') is not None:
                raise RuntimeError('Reacquisition enrollment disappeared after claim')
            return self.host.device.execute_gripper_probe(payload['arm'], payload['target'])
        from .supported_gripper_recovery import reacquisition_source
        source, receipt = reacquisition_source(state)
        if (payload.get('reacquisition_proposal_sha256') != state['proposal']['proposal_sha256']
                or payload['arm'] != source['arm']
                or payload.get('grasp_object_id') != source.get('grasp_object_id')
                or payload['kind'] != 'gripper' or payload['operation'] != 'grip_supported'):
            raise RuntimeError('Reacquisition source changed after claim')
        return self.host.device.reacquire_supported_gripper(payload['arm'], payload['target'],
                                                           source_receipt=receipt)

    def check_retention(self, episode_id):
        state = self.state()
        if state is None or state['proposal'].get('route') not in (
                'audited_contact_reacquisition', 'audited_supported_contact_observation'):
            return
        from .supported_gripper_recovery import reacquisition_source, current_contact_source
        source, _ = (current_contact_source(state) if state['proposal']['route'] == 'audited_supported_contact_observation'
                     else reacquisition_source(state))
        grasp = self.host.grasps.active(source['arm'])
        events = state['events']
        if (state['phase'] != 'contact_candidate' or not events or grasp is None
                or grasp['identity']['episode_id'] != episode_id
                or grasp['identity']['object_id'] != source.get('grasp_object_id')
                or grasp.get('probe_ref', {}).get('event_id') != events[-1]['event_id']):
            raise RuntimeError('Only the new completed reacquisition candidate may be retained')

    def observe_contact(self, event_id, observation_id, object_id, visual_description,
                        contact_relation, support_relation):
        """Claim one RX-only current-contact observation under an audited scope."""
        host = self.host
        event_id = _identifier(event_id, 'contact observation event id')
        object_id = _identifier(object_id, 'contact object id')
        if (type(visual_description) is not str or not 1 <= len(visual_description.strip()) <= 4000
                or '\x00' in visual_description or contact_relation != 'bilateral_finger_contact'
                or support_relation != 'independent_support_present'):
            raise ValueError('Describe current bilateral finger contact and independent object support')
        request = dict(operation='supported_contact_observe', observation_id=observation_id,
            object_id=object_id, visual_description=visual_description, contact_relation=contact_relation,
            support_relation=support_relation)
        with host.state_lock:
            replay = host._preparation_replay(event_id, request)
            if replay is not None:
                return replay
            if not host.opened or host.active_event_id is not None or not host.task_ready:
                raise RuntimeError('Contact observation needs the idle, ready, unique host')
            host._guard()
            state = self.state()
            if (state is None or state['proposal'].get('route') != 'audited_supported_contact_observation'
                    or state['phase'] != 'contact_observation_required'):
                raise RuntimeError('Audited current-contact observation scope required; no repeated observation')
            from .supported_gripper_recovery import current_contact_source
            old, _ = current_contact_source(state)
            if object_id != old['grasp_object_id']:
                raise ValueError('Contact observation must preserve the audited object')
            arm = old['arm']
            scene = host._saved_preparation_scene(observation_id, arm)
            payload = dict(kind='supported_contact_observe', arm=arm, request=request,
                source_event_id=state['proposal']['snapshot']['event']['event_id'],
                observation_proposal_sha256=state['proposal']['proposal_sha256'],
                saved_rgb_evidence=copy.deepcopy(scene['saved_rgb_evidence']))
            refresh = host._rgb_dispatch_window(scene, event_id, 'before_claim')
            if refresh is not None:
                return refresh
            host.grasps.prepare_probe(arm, object_id)
            return host._start_preparation(event_id, request, payload, scene['rgb_received_at']+30.)

    def execute_contact_observation(self, payload):
        state = self.state()
        if state is None or state['proposal'].get('route') != 'audited_supported_contact_observation':
            raise RuntimeError('Current-contact observation enrollment disappeared after claim')
        from .supported_gripper_recovery import current_contact_source
        old, source = current_contact_source(state)
        if (payload['kind'] != 'supported_contact_observe' or payload['arm'] != old['arm']
                or payload['request']['object_id'] != old['grasp_object_id']
                or payload['source_event_id'] != state['proposal']['snapshot']['event']['event_id']
                or payload['observation_proposal_sha256'] != state['proposal']['proposal_sha256']):
            raise RuntimeError('Audited existing-target observation binding changed')
        receipt = self.host.device.observe_supported_contact(old['arm'], source_receipt=source)
        zero = {side:dict(attempted_frames=0, sent_frames=0, blocked_frames=0) for side in ('left','right')}
        if (receipt.get('ok') is not True or receipt.get('status') != 'observed_supported_contact_candidate'
                or receipt.get('completion_mode') != 'supported_contact_observe'
                or receipt.get('candidate_basis') != 'existing_target_observation'
                or any(type(receipt.get(key)) is not int or receipt[key] != 0
                       for key in ('hardware_commands_sent','target_calls_sent'))
                or receipt.get('transmission_counts') != zero or receipt.get('session_transmission_counts') != zero
                or receipt.get('physical_stop_verified') is not None or receipt.get('grasp_verified') is not False
                or receipt.get('loaded') is not False or receipt.get('candidate_probe') is not None
                or receipt.get('audited_existing_contact') != source['audited_existing_contact']):
            return {**receipt, 'ok':False, 'current_contact_receipt_valid':False}
        receipt['sample'] = self.host._sample(receipt['sample'])
        return receipt

    def submit(self, event_id, observation_id, visual_description, support_relation,
               *, width_m=None, object_relation=None, confirm=False):
        host = self.host
        event_id = _identifier(event_id, 'recovery event id')
        operation = 'supported_recovery_confirm' if confirm else 'supported_recovery_open'
        if (type(visual_description) is not str or not 1 <= len(visual_description.strip()) <= 4000
                or support_relation != 'independent_support_present'):
            raise ValueError('Describe the current independent object support in all three RGB views')
        if confirm and (object_relation != 'object_clear_of_fingers' or width_m is not None):
            raise ValueError('Confirmation requires actual separation and cannot send a jaw target')
        request = dict(operation=operation, observation_id=observation_id,
                       visual_description=visual_description, support_relation=support_relation,
                       width_m=width_m, object_relation=object_relation)
        with host.state_lock:
            replay = host._preparation_replay(event_id, request)
            if replay is not None:
                return replay
            if not host.opened or host.active_event_id is not None or not host.task_ready:
                raise RuntimeError('Recovery requires the idle, ready, unique host')
            host._guard()
            state = self.state()
            if state is None or state['phase'] != ('confirmation_required' if confirm else 'opening_required'):
                raise RuntimeError('This recovery phase is unavailable; no repeated opening')
            source = state['proposal']['snapshot']
            old, receipt = recovery_source(state)
            arm = old['arm']
            scene = host._saved_preparation_scene(observation_id, arm)
            if confirm:
                opening = state['events'][0]
                if scene['rgb_received_at'] <= opening['finished_at']:
                    raise ValueError('Separation requires new RGB after the completed opening')
            else:
                from .contact_receipt import probe_closure_within_bound
                current = scene['sample']['arms'][arm]['gripper']['width_m']
                if not probe_closure_within_bound(width_m, current):
                    raise ValueError('Recovery is one strictly increasing jaw opening within 5 mm')
                continuation = receipt.get('audited_opening_continuation')
                if continuation is not None and width_m <= continuation['prior_opening_target_m']:
                    raise ValueError('Continued opening must exceed the previous complete target; no replay')
            payload = dict(kind=operation, request=request, arm=arm,
                           source_event_id=source['event']['event_id'],
                           recovery_proposal_sha256=state['proposal']['proposal_sha256'],
                           saved_rgb_evidence=copy.deepcopy(scene['saved_rgb_evidence']))
            refresh = host._rgb_dispatch_window(scene, event_id, 'before_claim')
            if refresh is not None:
                return refresh
            return host._start_preparation(event_id, request, payload,
                                           scene['rgb_received_at']+30.)

    def execute(self, operation, payload):
        host = self.host
        state = self.state()
        if state is None:
            raise RuntimeError('Missing durable recovery enrollment')
        source = state['proposal']['snapshot']
        old, receipt = recovery_source(state)
        if (payload['source_event_id'] != source['event']['event_id']
                or payload['recovery_proposal_sha256'] != state['proposal']['proposal_sha256']
                or payload['arm'] != old['arm']):
            raise RuntimeError('Recovery source changed after claim')
        if operation == 'supported_recovery_open':
            result = host.device.recover_supported_gripper(old['arm'], payload['request']['width_m'],
                                                          source_receipt=receipt)
            valid = (result.get('status') == 'release_arrived'
                     and result.get('hardware_commands_sent') == 1
                     and result.get('nominal_force_N') == .2
                     and result.get('passive_arm_commands_sent') == 0
                     and result.get('arrival_confirmed') is True)
        else:
            result = host.device.confirm_supported_recovery_release()
            valid = result.get('hardware_commands_sent') == 0 and result.get('status') == 'recovery_release_observed'
        if result.get('ok') is not True or not valid:
            # The entire device receipt is retained by the outer worker.
            result = {**result, 'ok': False, 'recovery_receipt_valid': False}
            return result
        result['sample'] = host._sample(result['sample'])
        result['recovery_source_event_id'] = source['event']['event_id']
        result['old_failed_receipt_preserved'] = True
        result['grasp_transferred'] = False
        return result
