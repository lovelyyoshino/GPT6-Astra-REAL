"""Thin attended client for the existing canonical Piper pair host.

Imports, inspect and plan are offline. Only explicit JSONL requests open the
pair host or cameras. This module contains no CAN encoder or task controller.
"""
from __future__ import annotations

import argparse
import contextlib
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import sys
import time

PROJECT = Path(__file__).resolve().parent
BUNDLE = Path('/home/agilex/GPT6-Astra-REAL')
CANONICAL = BUNDLE / 'projects/piperx_cloth_demo'
LEDGER = CANONICAL / 'runs/pair_sessions.sqlite'
RECIPE = BUNDLE / 'tasks/plug_transfer_left.json'
CAMERAS = {'front': '243322070709', 'left_wrist': '336222071115',
           'right_wrist': '244222070415'}
VIEWS = {'front': 'front', 'left_hand': 'left_wrist', 'right_hand': 'right_wrist'}
TOOLS = frozenset({
    'robot_pair_open', 'robot_pair_observe', 'robot_pair_submit_once',
    'robot_pair_status', 'robot_pair_cancel', 'robot_pair_close',
    'robot_pair_prepare_gripper', 'robot_pair_inspect_joint_limits',
    'robot_pair_promote_ready', 'robot_pair_initialize_joint_target',
    'robot_pair_publish_geometry', 'robot_pair_retain_grasp',
    'robot_pair_confirm_release', 'robot_pair_confirm_loaded_response',
    'robot_pair_recover_supported_gripper', 'robot_pair_confirm_recovery_release',
    'robot_pair_observe_supported_contact',
})
FAULT_TOOLS = {'robot_pair_status', 'robot_pair_cancel', 'robot_pair_close'}
MAX_LINE = 1024 * 1024


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def parse(text):
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError('Duplicate JSON key: ' + key)
            value[key] = item
        return value
    return json.loads(text, object_pairs_hook=pairs,
                      parse_constant=lambda s: (_ for _ in ()).throw(ValueError('Nonfinite JSON: ' + s)))


def profile():
    value = parse((CANONICAL / 'configs/robot.json').read_text())
    if value.get('cameras') != CAMERAS:
        raise ValueError('Canonical camera bindings changed; review the actual binding before use')
    for side, channel, usb in (('left', 'can0', '1-6.2:1.0'), ('right', 'can1', '1-6.3:1.0')):
        arm = value.get('arms', {}).get(side, {})
        if (arm.get('model'), arm.get('channel'), arm.get('usb_interface')) != ('piper_x', channel, usb):
            raise ValueError('Canonical arm binding changed: ' + side)
    return value


def service_factory():
    sys.path.insert(0, str(CANONICAL))
    from robot_tools.service import ToolService
    service = ToolService(CANONICAL)
    service.persistent = True
    return service


def rig_factory():
    sys.path.insert(0, str(BUNDLE / 'projects/piper_right_pick_demo/src'))
    from right_pick.camera import RealSenseRig
    return RealSenseRig({view: {'serial': CAMERAS[key]} for view, key in VIEWS.items()},
                        depth_enabled=False, rgb_only=True)


def inspect():
    current = profile()
    ledger = {'path': str(LEDGER), 'exists': LEDGER.exists(), 'latest_run': None}
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    boot = {'boot_id': boot_id, 'startup_records': [], 'startup_is_task_enrollment': False}
    if LEDGER.exists():
        with sqlite3.connect(LEDGER.as_uri() + '?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            db.row_factory = sqlite3.Row
            db.execute('BEGIN')
            row = db.execute('SELECT run_id,max_steps,max_duration,started_at,steps FROM pair_runs '
                             'ORDER BY started_at DESC,run_id DESC LIMIT 1').fetchone()
            ledger['latest_run'] = dict(row) if row else None
            ledger['pending_events'] = db.execute("SELECT count(*) FROM pair_events WHERE status!='complete'").fetchone()[0]
            ledger['preserved_fault_count'] = db.execute('SELECT count(*) FROM pair_faults').fetchone()[0]
            sys.path.insert(0, str(CANONICAL))
            from robot_tools.pair_ledger import _execution_scope
            table, _, ordinal, scope = _execution_scope(db, row['run_id'] if row else None)
            if scope is None:
                raise ValueError('Canonical database lacks its effective scope')
            ledger['effective_scope'] = {'table': table, 'ordinal': ordinal,
                                        **{key: scope[key] for key in ('owner', 'active_run_id', 'fault_id', 'last_time')}}
            fault = db.execute('SELECT id,run_id,owner,reason,at FROM pair_faults WHERE id=?',
                               (scope['fault_id'],)).fetchone()
            if scope['fault_id'] is not None and fault is None:
                raise ValueError('Canonical effective fault record is missing')
            ledger['effective_fault'] = dict(fault) if fault else None
            if db.execute("SELECT 1 FROM sqlite_master WHERE name='pair_reboot_startups'").fetchone():
                boot['startup_records'] = [dict(item) for item in db.execute(
                    'SELECT arm,run_id,status,started_at,finished_at FROM pair_reboot_startups '
                    'WHERE boot_id=? ORDER BY arm', (boot_id,))]
    return {'status': 'offline_inspection', 'canonical_root': str(CANONICAL),
            'ledger': ledger, 'current_boot': boot, 'arms': current['arms'], 'cameras': current['cameras'],
            'allowed_tools': sorted(TOOLS), 'hardware_accessed': False,
            'task_completed': False, 'physical_stop_verified': None}


def role_plan(worker_arm='right', support_arm='left'):
    # Pure contract projection: importing these helpers opens no device.
    sys.path.insert(0, str(CANONICAL))
    from robot_tools.task_roles import resolve_task_roles
    from robot_tools.plug_recipe import render_plug_recipe
    worker, support = resolve_task_roles({'worker_arm': worker_arm, 'support_arm': support_arm})
    return ({worker: 'plug_worker', support: 'power_strip_stabilizer'},
            render_plug_recipe(parse(RECIPE.read_text()), worker_arm=worker, support_arm=support))


def plan(worker_arm='right', support_arm='left'):
    roles, recipe = role_plan(worker_arm, support_arm)
    return {'status': 'offline_plan_only', 'source_recipe_path': str(RECIPE),
            'source_recipe_sha256': hashlib.sha256(RECIPE.read_bytes()).hexdigest(),
            'recipe_sha256': hashlib.sha256((encode(recipe) + '\n').encode()).hexdigest(),
            'recipe': recipe, 'dispatch_authorized': False,
            'roles': roles, 'worker_arm': worker_arm, 'support_arm': support_arm,
            'task_completed': False, 'hardware_accessed': False}


class Session:
    """One process, one service, one camera rig; local logs are not a robot ledger."""

    def __init__(self, directory, *, context='', worker_arm='right', support_arm='left',
                 make_service=service_factory, make_rig=rig_factory):
        roles, recipe = role_plan(worker_arm, support_arm)
        self.worker_arm, self.support_arm = worker_arm, support_arm
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log = (self.directory / 'session.jsonl').open('x', encoding='utf-8')
        self.make_service, self.make_rig = make_service, make_rig
        self.service = self.rig = None
        self.open_attempted = self.fault_observed = self.closed = False
        self.request_ids = set()
        self.sequence = 0
        recipe_bytes = (encode(recipe) + '\n').encode('utf-8')
        rendered_recipe_path = self.directory / 'recipe.json'
        task = {'task_id': 'plug_transfer_left', 'goal': '双臂协作拔出原插头并插入初始正视图左侧相邻插座',
                'roles': roles, 'worker_arm': worker_arm, 'support_arm': support_arm,
                'canonical_root': str(CANONICAL), 'canonical_ledger': str(LEDGER),
                'operator_context': context, 'context_is_semantic_testimony': True,
                'task_completed': False, 'physical_stop_verified': None,
                'recipe_path': str(RECIPE), 'recipe_sha256': hashlib.sha256(RECIPE.read_bytes()).hexdigest(),
                'rendered_recipe_path': str(rendered_recipe_path),
                'rendered_recipe_sha256': hashlib.sha256(recipe_bytes).hexdigest()}
        (self.directory / 'task.json').write_text(encode(task) + '\n', encoding='utf-8')
        rendered_recipe_path.write_bytes(recipe_bytes)
        self.record('session_started', task=task, automatic_tool_calls=0)

    def record(self, kind, **fields):
        self.sequence += 1
        self.log.write(encode({'sequence': self.sequence, 'at': time.time(), 'kind': kind, **fields}) + '\n')
        self.log.flush()
        os.fsync(self.log.fileno())

    @staticmethod
    def known_missing(result):
        return (isinstance(result, dict) and result.get('ok') is False
                and result.get('status') == 'preparation_required' and result.get('fault_latched') is False
                and type(result.get('hardware_commands_sent')) is int and result['hardware_commands_sent'] == 0
                and type(result.get('requirements')) is list and bool(result['requirements'])
                and isinstance(result.get('readiness'), dict))

    @staticmethod
    def adverse(result, tolerated=None):
        """Check only response/receipt chains, not embedded historical diagnostics."""
        if (result.get('fault_latched') is True or result.get('status') in ('fault', 'pair_device_fault')
                or result.get('error') or result.get('isError') or result.get('errors')
                or result.get('guard_violations') or (result.get('ok') is False and result is not tolerated)):
            return True
        return any(Session.adverse(result[key], tolerated) for key in ('receipt', 'device_receipt')
                   if isinstance(result.get(key), dict))

    @staticmethod
    def failed(result, tool=None):
        """Recognize existing wire shapes; unknown results cannot permit another target."""
        if not isinstance(result, dict) or not result:
            return True
        status = result.get('status')
        receipt = result.get('receipt')
        tolerated = None
        if tool == 'robot_pair_promote_ready' and Session.known_missing(result):
            tolerated = result
        elif (status == 'completed' and tool in {'robot_pair_status', 'robot_pair_prepare_gripper',
                                                 'robot_pair_inspect_joint_limits'}
              and Session.known_missing(receipt)
              and receipt.get('execution_mode') in ('prepare_gripper', 'inspect_joint_limits')
              and (tool == 'robot_pair_status' or tool == 'robot_pair_' + receipt['execution_mode'])):
            tolerated = receipt
        if Session.adverse(result, tolerated):
            return True
        # Known zero-TX preparation shortfall is not a fault and can be remedied
        # by another explicit canonical preparation request on this same host.
        if tool == 'robot_pair_promote_ready' and status == 'preparation_required':
            return tolerated is not result
        if status == 'refresh_required' and tool in {'robot_pair_submit_once', 'robot_pair_initialize_joint_target',
                                                   'robot_pair_recover_supported_gripper', 'robot_pair_confirm_recovery_release',
                                                   'robot_pair_observe_supported_contact'}:
            return not (result.get('hardware_commands_sent') == 0 and result.get('event_claimed') is False
                        and result.get('steps_consumed') == 0 and result.get('fault_latched') is False)
        if status in ('pending', 'completed') and isinstance(result.get('event_id'), str):
            asynchronous = {'robot_pair_submit_once', 'robot_pair_prepare_gripper',
                            'robot_pair_recover_supported_gripper', 'robot_pair_confirm_recovery_release',
                            'robot_pair_observe_supported_contact',
                            'robot_pair_initialize_joint_target', 'robot_pair_inspect_joint_limits', 'robot_pair_status'}
            if tool not in asynchronous:
                return True
            if status == 'pending':
                return result.get('receipt') is not None
            receipt = result.get('receipt')
            return not (isinstance(receipt, dict) and (receipt.get('ok') is True or receipt is tolerated))
        if status in ('owned', 'pending', 'detached'):
            return not (tool in {'robot_pair_open', 'robot_pair_status', 'robot_pair_promote_ready', 'robot_pair_cancel'}
                        and isinstance(result.get('run_id'), str) and isinstance(result.get('ledger'), dict)
                        and type(result.get('open')) is bool and result.get('fault_latched') is False)
        if status == 'closed':
            return not (tool == 'robot_pair_close' and isinstance(result.get('cleanup'), dict)
                        and result.get('fault_latched') is False)
        if status is not None:  # Including an unknown status paired with ok=true.
            return True
        if tool == 'robot_pair_observe':
            return not (isinstance(result.get('observation_id'), str) and isinstance(result.get('capture_id'), str)
                        and isinstance(result.get('sample'), dict)
                        and isinstance(result.get('peer_receipts'), dict)
                        and set(result['peer_receipts']) == {'left', 'right'})
        if tool == 'robot_pair_publish_geometry':
            return not (isinstance(result.get('source'), dict) and isinstance(result.get('geometry'), dict)
                        and isinstance(result.get('index_path'), str) and result.get('dispatch_authorized') is False
                        and result.get('hardware_commands_sent') == 0)
        if tool in {'robot_pair_retain_grasp', 'robot_pair_confirm_release', 'robot_pair_confirm_loaded_response'}:
            return not (result.get('ok') is True and isinstance(result.get('event_id'), str)
                        and isinstance(result.get('episode'), dict) and result.get('hardware_commands_sent') == 0)
        return True

    def request(self, request):
        if self.closed:
            raise ValueError('Session is closed')
        if type(request) is not dict or set(request) != {'id', 'op', 'arguments'}:
            raise ValueError('Require exactly id, op and arguments')
        identifier, op, arguments = request['id'], request['op'], request['arguments']
        if type(identifier) is not str or not re.fullmatch(r'[A-Za-z0-9_-]{1,120}', identifier):
            raise ValueError('Request id must be a short safe string')
        if identifier in self.request_ids:
            raise ValueError('Request already attempted; inspect its saved receipt, never replay it')
        if type(arguments) is not dict:
            raise ValueError('arguments must be an object')
        if op not in TOOLS | {'capture', 'note'}:
            raise ValueError('Operation is not in the fixed pair/camera allowlist')
        if op == 'robot_pair_open' and self.open_attempted:
            raise ValueError('This process already attempted pair_open; use status/close on its retained service')
        if op == 'robot_pair_open' and arguments.get('task_id') != parse(RECIPE.read_text())['task_id']:
            raise ValueError('pair_open task_id must match the fixed plug_transfer_left recipe')
        if op == 'robot_pair_open':
            sys.path.insert(0, str(CANONICAL))
            from robot_tools.task_roles import resolve_task_roles
            if resolve_task_roles(arguments) != (self.worker_arm, self.support_arm):
                raise ValueError('pair_open arm roles must match this session role plan')
        if self.fault_observed and op in TOOLS - FAULT_TOOLS:
            raise ValueError('Fault or uncertain result observed; only status/cancel/close remain available')
        if op == 'capture' and arguments:
            raise ValueError('capture takes no arguments and uses the fixed three-camera bindings')
        if op == 'note' and (set(arguments) != {'text'} or type(arguments['text']) is not str
                             or not 1 <= len(arguments['text']) <= 8000):
            raise ValueError('note requires only nonempty text (up to 8000 characters)')
        self.record('request', request=copy.deepcopy(request))  # Durable before any possible device access.
        self.request_ids.add(identifier)
        try:
            if op == 'note':
                result = {'status': 'semantic_note_recorded', 'task_completed': None}
            elif op == 'capture':
                if self.rig is None:
                    self.rig = self.make_rig()
                result = self.rig.capture(self.directory / 'capture')
                path = Path(result['metadata_path']).resolve()
                if path.name != 'observation.json' or not path.is_relative_to(self.directory / 'capture'):
                    raise ValueError('Camera adapter returned an unexpected metadata path')
                saved = parse(path.read_text())
                if saved != result or set(result.get('cameras', {})) != set(VIEWS):
                    raise ValueError('Saved camera report differs or lacks the three required views')
                for view, key in VIEWS.items():
                    camera = result['cameras'][view]
                    image = Path(camera['rgb_path']).resolve()
                    if camera.get('serial') != CAMERAS[key] or camera.get('depth_enabled') is not False:
                        raise ValueError('Camera binding/RGB-only mismatch')
                    if not image.is_relative_to(path.parent) or image.suffix != '.png' or not image.is_file():
                        raise ValueError('Camera RGB file is missing or outside the capture')
                result = {'status': 'captured', 'observation': result,
                          'rgb_observation_path': str(path),
                          'metadata_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                          'hardware_commands_sent': 0, 'automatically_observed_by_pair': False}
            else:
                if self.service is None:
                    self.service = self.make_service()
                if op == 'robot_pair_open':
                    self.open_attempted = True
                result = self.service.call(op, copy.deepcopy(arguments))
                self.fault_observed = self.fault_observed or self.failed(result, op)
            self.record('result', request_id=identifier, result=result,
                        fault_observed=self.fault_observed)
            return {'id': identifier, 'result': result}
        except Exception as exc:
            if op in TOOLS - FAULT_TOOLS:
                self.fault_observed = True
            result = {'error': type(exc).__name__ + ': ' + str(exc),
                      'automatic_retry': False, 'physical_stop_verified': None}
            self.record('request_error', request_id=identifier, **result,
                        fault_observed=self.fault_observed)
            return {'id': identifier, **result}

    def close(self):
        if self.closed:
            return
        errors = []
        # The canonical EOF handler latches unexplained client loss; it never
        # sends a hold on EOF and never discards the durable owner/fault history.
        for name, resource, method in (('service', self.service, 'shutdown'), ('camera', self.rig, 'close')):
            if resource is not None:
                try:
                    getattr(resource, method)()
                    if name == 'camera':
                        errors.extend(getattr(resource, 'close_errors', []))
                except Exception as exc:
                    errors.append({'resource': name, 'error': type(exc).__name__ + ': ' + str(exc)})
        self.record('session_ended', cleanup_errors=errors, task_completed=None,
                    physical_stop_verified=None, automatic_retry=False)
        self.closed = True
        self.log.close()


def serve(session, source, output):
    output.write(encode({'status': 'ready_for_explicit_requests', 'session_directory': str(session.directory),
                         'automatic_tool_calls': 0, 'canonical_ledger': str(LEDGER)}) + '\n')
    output.flush()
    try:
        while True:
            line = source.readline(MAX_LINE + 1)
            if not line:
                break
            try:
                if len(line) > MAX_LINE:
                    raise ValueError('Request exceeds size limit; session will close without reading remaining input')
                with contextlib.redirect_stdout(sys.stderr):
                    response = session.request(parse(line))
            except Exception as exc:
                response = {'error': type(exc).__name__ + ': ' + str(exc), 'automatic_retry': False}
                session.record('request_rejected', **response)
            output.write(encode(response) + '\n')
            output.flush()
            if len(line) > MAX_LINE:
                break
    finally:
        with contextlib.redirect_stdout(sys.stderr):
            session.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('inspect', help='Read fixed bindings and canonical ledger without hardware')
    plan_parser = commands.add_parser('plan', help='Show the task recipe with explicit arm roles; no execution')
    plan_parser.add_argument('--worker-arm', choices=('left', 'right'), default='right')
    session = commands.add_parser('session', help='Retain one service and accept explicit JSONL requests on stdin')
    session.add_argument('--name', required=True, help='New log directory name; never a new robot ledger')
    session.add_argument('--context', default='', help='Actual operator/task statement, logged without granting permission')
    session.add_argument('--worker-arm', choices=('left', 'right'), default='right',
                         help='Plug worker; the other arm stabilizes the strip, frozen for this session')
    args = parser.parse_args(argv)
    if args.command != 'session':
        print(encode(inspect() if args.command == 'inspect' else plan(
            args.worker_arm, 'right' if args.worker_arm == 'left' else 'left')))
        return 0
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,120}', args.name):
        parser.error('--name must contain 1..120 letters, digits, hyphens or underscores')
    profile()  # Fixed identities only; no hardware probe or connection here.
    serve(Session(PROJECT / 'runs' / args.name, context=args.context, worker_arm=args.worker_arm,
                  support_arm='right' if args.worker_arm == 'left' else 'left'), sys.stdin, sys.stdout)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
