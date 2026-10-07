#!/usr/bin/env python3
"""One ROS J2 goal with action-bound cancellation and verified hold result.

Import is hardware-free. This client never uses CAN or the SDK. Driver failure,
lost communication and process death are not physical-stop confirmations.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
NODE = '/piper/right/interruptible_driver'
TOPIC = '/piper/right/joint_cmd'
TERMINAL = {'completed', 'hold_confirmed', 'cancelled_before_dispatch', 'already_completed', 'failed'}


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def run_goal(io, target_raw, *, motion_timeout_s=20., cancel_after_progress_deg=None,
             clock=time, cancelled=lambda: False, audit=lambda row: None):
    """io.status/publish/hold are bounded transports; publish and hold run once.

    Cancellation is bound to both adoption token and the admitted action number.
    A timeout while waiting for confirmation never reports a successful stop.
    """
    require(math.isfinite(motion_timeout_s) and .01 <= motion_timeout_s <= 60., 'Invalid timeout')
    if cancel_after_progress_deg is not None:
        require(.2 <= cancel_after_progress_deg <= .5, 'Progress trigger outside local probe range')
    initial = io.status()
    require(initial['phase'] in ('idle', 'completed') and not initial['active']
            and not initial['failure'] and not initial['stop_latched'], 'Driver unavailable or latched')
    token, sequence = initial['adoption_token'], initial['sequence'] + 1
    require(re.fullmatch('[0-9a-f]{32}', token) is not None, 'Invalid adoption identity')
    bound = NODE + '/hold_current/seq_%d_%s' % (sequence, token)
    started, published_at = clock.monotonic(), clock.time()
    result = dict(published=False, sequence=sequence, adoption_token=token, hold_requested=False,
                  hold_accepted=False, hold_confirmed=False, target_reached=False,
                  task_success=False, accepted_target_may_continue=False)
    if cancelled():
        result.update(driver_phase='cancelled_before_publication', elapsed_s=clock.monotonic()-started)
        audit(dict(event='client_result', unix_s=clock.time(), **result))
        return result
    audit(dict(event='publication_intent', unix_s=published_at, target_raw=target_raw,
               sequence=sequence, adoption_token=token))
    # An exception from publish can still mean the message was delivered.
    result['accepted_target_may_continue'] = True
    try:
        io.publish(target_raw)
        result['published'] = True
        requested_at, reason = None, None
        while clock.monotonic() - started < 90.:
            try:
                state = io.status()
            except KeyboardInterrupt:
                reason = 'keyboard_interrupt'
                continue
            require(state['adoption_token'] == token, 'Driver adoption changed; no cancellation sent')
            require(state['sequence'] in (sequence - 1, sequence), 'Different action; no cancellation sent')
            if state['sequence'] != sequence:
                refusal = state.get('last_refusal')
                if refusal and refusal['unix_s'] >= published_at:
                    raise RuntimeError('Goal refused: ' + refusal['error'])
                require(clock.monotonic() - started < 15., 'Admission unknown; no blind retry')
                clock.sleep(.01)
                continue
            require(state.get('hold_service') == bound, 'Hold service does not belong to this action')
            result['driver_phase'] = state['phase']
            if state['phase'] in TERMINAL:
                if state['phase'] == 'failed':
                    raise RuntimeError('Driver failure: ' + str(state.get('failure')))
                result.update(hold_confirmed=state['phase'] == 'hold_confirmed',
                              target_reached=state['phase'] in ('completed', 'already_completed'),
                              accepted_target_may_continue=False, driver_result=state.get('result'))
                break
            if requested_at is not None:
                require(clock.monotonic() - requested_at < 20., 'Hold confirmation timed out; stop unconfirmed')
            else:
                if cancelled():
                    reason = reason or 'external_cancel'
                sent = state.get('command_sent_unix_s')
                if sent is not None and clock.time() - sent >= motion_timeout_s:
                    reason = reason or 'motion_timeout'
                elif sent is None and clock.monotonic() - started >= 12.:
                    reason = reason or 'preflight_timeout'
                current = state.get('latest_state')
                if cancel_after_progress_deg is not None and sent is not None and current:
                    progress = (state['before']['raw_q'][1] - current['raw_q'][1]) / 1000.
                    if progress >= cancel_after_progress_deg:
                        reason = reason or 'validation_progress_trigger'
                if reason:
                    # A partial/failed transaction must not provoke another CAN transaction.
                    for receipt in state['receipts']:
                        require(receipt['attempted_frames'] == receipt['socket_send_returns'] == 4,
                                'Partial send receipt; no blind hold')
                    require(state['active'] and not state.get('failure'), 'No healthy active action')
                    from ros_home_step import health
                    require(current is not None, 'No current feedback for cancellation')
                    health(current, clock.time(), idle=False)
                    require(current['jaw_code'] == 64, 'Jaw status changed')
                    requested_at = clock.monotonic()
                    result.update(hold_requested=True, hold_reason=reason)
                    audit(dict(event='hold_request', unix_s=clock.time(), service=bound, reason=reason))
                    # Never retry an ambiguous service result.
                    accepted, answer = io.hold(bound)
                    require(answer['adoption_token'] == token and answer['sequence'] == sequence,
                            'Hold response belongs to another action')
                    result['hold_accepted'] = bool(accepted and answer.get('accepted'))
                    audit(dict(event='hold_response', unix_s=clock.time(), accepted=result['hold_accepted']))
            clock.sleep(.01)
        else:
            raise RuntimeError('Client observation budget exhausted; physical stop unconfirmed')
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = str(error) or type(error).__name__
    result['elapsed_s'] = clock.monotonic() - started
    audit(dict(event='client_result', unix_s=clock.time(), **result))
    return result


class RosTransport:
    """Bounded service waits use daemon readers, never duplicate requests."""
    def __init__(self, config, runtime):
        import rospy
        import rosgraph
        import rosnode
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        self.rospy, self.message, self.trigger = rospy, JointState, Trigger
        self.config, self.runtime = Path(config), Path(runtime)
        self.master = rosgraph.Master(rospy.get_name())
        # No publisher exists yet; verify source, live process, and exclusive ownership.
        session = json.loads((ROOT/'runs/ros_interruptible_joint_sessions'/
                              (Path('/proc/sys/kernel/random/boot_id').read_text().strip()+'.json')).read_text())
        self.identity = session['identity']
        entry = ROOT/'scripts/ros_interruptible_joint_entry.py'
        require(hashlib.sha256(entry.read_bytes()).hexdigest() == self.identity['source_sha256'], 'Entry source changed')
        require(hashlib.sha256(self.config.read_bytes()).hexdigest() == self.identity['config_sha256'], 'Config changed')
        from xmlrpc.client import ServerProxy
        uri = rosnode.get_api_uri(self.master, NODE)
        require(uri, 'Interruptible driver absent')
        pid = ServerProxy(uri).getPid(rospy.get_name())[2]
        require(pid == self.identity['pid'], 'Live PID does not match adoption')
        args = (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        for expected in (str(entry), '--entry-config', str(self.config), '--entry-output-dir', str(self.runtime)):
            require(expected.encode() in args, 'Driver process argument mismatch: '+expected)
        pubs, subs, _ = self.master.getSystemState()
        require(dict(subs).get(TOPIC) == [NODE], 'Joint command subscriber is not exclusive')
        for topic in (TOPIC, '/piper/right/pos_cmd', '/piper/right/enable_flag'):
            require(not dict(pubs).get(topic), 'Competing command publisher')
        require(dict(pubs).get('/piper/right/eval_telemetry') == [NODE], 'Telemetry owner mismatch')
        require(rospy.get_param(NODE+'/speed_percent') == 1, 'Speed must remain one percent')
        self.publisher = rospy.Publisher(TOPIC, JointState, queue_size=1, latch=False)
        deadline = time.monotonic()+3.
        while not self.publisher.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.02)
        require(self.publisher.get_num_connections() == 1, 'Joint transport unavailable')

    def service(self, name):
        self.rospy.wait_for_service(name, timeout=1.)
        output = []
        def invoke():
            try:
                output.append((True, self.rospy.ServiceProxy(name, self.trigger)()))
            except Exception as error:
                output.append((False, error))
        worker = threading.Thread(target=invoke, daemon=True)
        worker.start(); worker.join(2.)
        require(output, 'ROS service response unknown; request will not be retried')
        ok, value = output[0]
        if not ok:
            raise value
        return value.success, json.loads(value.message)

    def status(self):
        ok, result = self.service(NODE+'/hold_status')
        require(ok and result['adoption_token'] == self.identity['adoption_token'], 'Status identity mismatch')
        return result

    def observe(self):
        from std_msgs.msg import String
        from right_pick.fast_ros import check_telemetry
        raw = json.loads(self.rospy.wait_for_message('/piper/right/eval_telemetry', String, timeout=1.).data)
        check_telemetry(raw, time.time(), require_idle=True)
        return raw

    def publish(self, raw):
        msg = self.message()
        msg.header.stamp = self.rospy.Time.now()
        msg.name = ['joint%d'%i for i in range(1, 7)]
        msg.position = [x*math.pi/180000 for x in raw]
        msg.velocity = [0.]*6+[1.]
        self.publisher.publish(msg)

    def hold(self, path):
        return self.service(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--runtime', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--j2-toward-zero-deg', type=float, required=True)
    parser.add_argument('--motion-timeout-s', type=float, default=20.)
    parser.add_argument('--cancel-after-progress-deg', type=float)
    args = parser.parse_args()
    require(math.isfinite(args.j2_toward_zero_deg) and 0 < args.j2_toward_zero_deg <= 1., 'J2 step must be <=1degree')
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(ROOT/'src'))
    import rospy
    rospy.init_node('interruptible_joint_client', anonymous=True, disable_signals=True)
    import signal
    cancel = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: cancel.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel.set())
    io = RosTransport(args.config, args.runtime)
    before = io.observe()
    target = list(before['raw_q'])
    target[1] -= int(round(args.j2_toward_zero_deg*1000))
    require(target[1] >= 0, 'J2 target below existing zero')
    (out/'before.json').write_text(json.dumps(before, indent=2)+'\n')
    with (out/'events.jsonl').open('x', buffering=1) as log:
        def audit(row):
            log.write(json.dumps(row, allow_nan=False)+'\n')
        result = run_goal(io, target, motion_timeout_s=args.motion_timeout_s,
                          cancel_after_progress_deg=args.cancel_after_progress_deg,
                          cancelled=cancel.is_set, audit=audit)
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 1 if result.get('error') else 0


if __name__ == '__main__':
    sys.exit(main())
