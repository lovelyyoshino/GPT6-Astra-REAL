"""RX-only grouping of PiPER X/default pose and joint fragments.

The SDK updates mutable fragment objects individually. Never derive a pose from
half of the next batch. Publish only complete, ordered three-frame groups and
retain their original timestamps, decoded values and wire bytes. No filtering,
FK substitution, target, query, timestamp renewal, or physical atomicity claim.
"""
import copy
import math
import threading

GROUPS = {
    'pose': ((0x2A2,'end_pose_xy',('X_axis','Y_axis')),
             (0x2A3,'end_pose_zrx',('Z_axis','RX_axis')),
             (0x2A4,'end_pose_ryrz',('RY_axis','RZ_axis'))),
    'joints': ((0x2A5,'joint_12',('joint_1','joint_2')),
               (0x2A6,'joint_34',('joint_3','joint_4')),
               (0x2A7,'joint_56',('joint_5','joint_6'))),
}
# This deployed 1-Mbit/s feedback stream has ~5-ms cycles and ~0.3-ms groups.
# Require a tighter host receive grouping; this is not a firmware cycle ID.
MAX_GROUP_SPAN_S = .002
MAX_INCOMPLETE_RECORDS = 64
LOOKUP = {can_id:(group,index,name) for group,spec in GROUPS.items()
          for index,(can_id,name,_) in enumerate(spec)}


class CoherentFeedback:
    def __init__(self, parser):
        self.parser = parser
        self.lock = threading.RLock()
        self.original = parser.parse_packet
        self.pending = {name:[] for name in GROUPS}
        self.complete = {name:None for name in GROUPS}
        self.sequence = dict.fromkeys(GROUPS,0)
        self.incomplete = []
        self.failure = None
        self.failure_evidence = None
        self.last_stamp = {}
        parser.parse_packet = self.parse_packet

    def _fail(self, reason, evidence):
        # Preserve the first offending bytes even if later feedback is valid.
        if self.failure is None:
            self.failure = reason
            self.failure_evidence = copy.deepcopy(evidence)

    def _discard(self, group, reason):
        if self.pending[group]:
            if len(self.incomplete)>=MAX_INCOMPLETE_RECORDS:
                self._fail('Incomplete feedback evidence buffer exhausted',
                           {'group':group,'frames':[row['frame'] for row in self.pending[group]]})
            else:
                self.incomplete.append({'group':group,'reason':reason,
                    'frames':[copy.deepcopy(row['frame']) for row in self.pending[group]]})
        self.pending[group] = []

    def parse_packet(self, frame):
        with self.lock:
            item = LOOKUP.get(frame.arbitration_id)
            if item is None:
                return self.original(frame)
            group,index,name = item
            stamp = frame.timestamp
            valid_stamp = type(stamp) in (int,float) and math.isfinite(stamp) and stamp>0
            evidence = {'arbitration_id':frame.arbitration_id,'data_hex':bytes(frame.data).hex(),
                        'received_at_s':stamp if valid_stamp else None,'dlc':frame.dlc,
                        'flags':{key:bool(getattr(frame,key,False)) for key in
                                 ('is_extended_id','is_remote_frame','is_error_frame','is_fd')}}
            if not valid_stamp:
                evidence['invalid_timestamp_repr'] = repr(stamp)
            if (not valid_stamp or frame.dlc!=8 or len(frame.data)!=8
                    or any(evidence['flags'].values())):
                self._fail('Invalid feedback fragment identity, shape or timestamp',evidence)
                return None
            if name in self.last_stamp and stamp<=self.last_stamp[name]:
                self._fail('Feedback fragment receive timestamp did not advance',evidence)
                return None
            try:
                result = self.original(frame)
            except Exception as exc:
                self._fail('Manufacturer feedback decoder failed: '+type(exc).__name__,evidence)
                raise
            self.last_stamp[name] = stamp
            if index==0:
                self._discard(group,'new_first_frame_before_group_completed')
            pending = self.pending[group]
            if index!=len(pending):
                self._discard(group,'missing_or_out_of_order_fragment')
                # Preserve this orphan too; it cannot complete a group.
                self.pending[group] = [{'frame':evidence,'fragment':None}]
                self._discard(group,'orphan_fragment')
                return result
            decoded = copy.deepcopy(getattr(self.parser,name,None))
            if decoded is None or decoded.timestamp!=stamp:
                self._fail('Manufacturer decoder did not return the matching fragment',evidence)
                return result
            pending.append({'frame':evidence,'fragment':decoded})
            stamps = [row['frame']['received_at_s'] for row in pending]
            if stamps!=sorted(stamps) or stamps[-1]-stamps[0]>MAX_GROUP_SPAN_S:
                self._discard(group,'fragment_group_receive_span_or_order')
                return result
            if len(pending)==3:
                self.complete[group] = pending
                self.pending[group] = []
                self.sequence[group] += 1
            return result

    def snapshot(self, names):
        with self.lock:
            fragments = {name:copy.deepcopy(getattr(self.parser,name,None)) for name in names}
            raw = {}
            published = {}
            for group,spec in GROUPS.items():
                raw[group] = [{'name':name,'timestamp_s':fragments[name].timestamp,
                               'values':[getattr(fragments[name].msg,key) for key in keys]}
                              for _,name,keys in spec if fragments.get(name) is not None]
                complete = self.complete[group]
                published[group] = {'sequence':self.sequence[group],
                    'frames':[copy.deepcopy(row['frame']) for row in complete] if complete else []}
                for index,(_,name,_) in enumerate(spec):
                    fragments[name] = copy.deepcopy(complete[index]['fragment']) if complete and not self.failure else None
            evidence = {'schema':'piper_rx_coherent_groups_v1','source':'same_connection_manufacturer_decoder',
                'max_group_receive_span_s':MAX_GROUP_SPAN_S,'groups':published,
                'raw_fragment_cache':raw,'pending_frames':{g:[copy.deepcopy(r['frame']) for r in rows]
                                                         for g,rows in self.pending.items()},
                'incomplete_groups':copy.deepcopy(self.incomplete),'error':self.failure,
                'failure_evidence':copy.deepcopy(self.failure_evidence),
                'timestamps_renewed':False,'hardware_commands_sent':0,'firmware_cycle_ids_available':False,
                'whole_snapshot_atomic':False}
            self.incomplete.clear()  # The caller records this evidence with its snapshot.
            return fragments,evidence


def install(robot):
    """Attach once to an already passively connected task-owned SDK parser."""
    if getattr(robot,'_pair_coherent_feedback',None) is not None:
        raise RuntimeError('Feedback grouping already installed on this connection')
    parser=getattr(robot,'_parser',None)
    if not callable(getattr(parser,'parse_packet',None)):
        raise RuntimeError('Manufacturer RX parser unavailable for coherent feedback')
    robot._pair_coherent_feedback = CoherentFeedback(parser)
    return robot._pair_coherent_feedback
