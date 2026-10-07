#!/usr/bin/env python3
"""Record two RealSense RGB streams; stdin accepts capture/stop JSON only.

Independent camera-only adaptation of piper_right_pick_demo's recorder. Each
capture uses the same running streams as the video; do not open robot_observe or
another camera owner concurrently. Keep stdin open, wait for ready, request
fresh captures as needed, and send {"op":"stop"} to finalize both AVI files.
No robot imports, task logic, depth stream, automatic restart or device reset.
AVI uses nominal 15 fps; frames.jsonl preserves actual receive/device times.
"""
import argparse
import json
from pathlib import Path
import queue
import re
import signal
import sys
import threading
import time


def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--serials', nargs=2, default=['243322070709', '244222070415'],
                        metavar=('FRONT', 'RIGHT_HAND'))
    parser.add_argument('--duration-s', type=int, default=300,
                        help='Maximum process recording lifetime, 1..3600 seconds including startup (default 300)')
    parser.add_argument('--max-snapshots', type=int, default=32,
                        help='Maximum saved snapshot pairs, 1..256 including initial (default 32)')
    args = parser.parse_args(argv)
    if not 1 <= args.duration_s <= 3600:
        parser.error('--duration-s must be in 1..3600')
    if not 1 <= args.max_snapshots <= 256:
        parser.error('--max-snapshots must be in 1..256')
    if (len(set(args.serials)) != 2
            or any(not re.fullmatch(r'[0-9]+', serial) for serial in args.serials)):
        parser.error('--serials requires two different numeric serials')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    names = ('front', 'right_hand')
    streams, writers, camera_info = {}, {}, {}
    stopped = threading.Event()
    requests = queue.Queue(maxsize=32)
    deadline = time.monotonic() + args.duration_s
    report = {'status': 'starting', 'started_at_s': time.time(), 'cameras': {},
              'snapshots': [], 'frames_recorded': dict.fromkeys(names, 0),
              'robot_commands_sent': 0, 'hardware_synchronized': False,
              'video_fps': 15, 'video_timing': 'nominal fps; timestamps in frames.jsonl',
              'duration_limit_s': args.duration_s, 'snapshot_limit': args.max_snapshots,
              'requested_cameras': dict(zip(names, args.serials)),
              'camera_roles': {'front': 'third_person', 'right_hand': 'first_person_right_wrist'},
              'depth_enabled': False, 'camera_owner_scope': 'These two serials only; no other camera owner concurrently'}
    frame_log = None
    exit_code = 0

    def receive():
        while not stopped.is_set():
            line = sys.stdin.readline(4097)
            if not line:
                stopped.set()
                return
            try:
                if len(line) > 4096:
                    raise ValueError('Command exceeds 4096 characters')
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError('Command must be a JSON object')
                if request.get('op') == 'stop' and set(request) == {'op'}:
                    stopped.set()
                    return
                if request.get('op') != 'capture' or set(request) != {'op', 'label'}:
                    raise ValueError('Only {op:capture,label:...} or {op:stop} is accepted')
                label = request['label']
                if not isinstance(label, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,32}', label):
                    raise ValueError('Label must contain 1-32 letters, digits, underscores or hyphens')
                requests.put_nowait(request)
            except Exception as exc:
                emit({'event': 'request_error', 'error': str(exc)})

    def next_frames(record=True):
        latest = {}
        for name in names:
            if stopped.is_set() or time.monotonic() >= deadline:
                raise TimeoutError('Camera recording stopped or reached its %d-second limit' % args.duration_s)
            frame = streams[name].wait_for_frames(1000).get_color_frame()
            if not frame:
                raise RuntimeError('Missing color frame: ' + name)
            received = time.time()
            bgr = np.asanyarray(frame.get_data()).copy()
            metadata = {'frame_number': int(frame.get_frame_number()),
                        'device_timestamp_ms': float(frame.get_timestamp()),
                        'host_received_at_s': received}
            latest[name] = {'bgr': bgr, 'metadata': metadata}
            if record:
                writers[name].write(bgr)
                frame_log.write(json.dumps(dict(metadata, camera=name,
                    video_index=report['frames_recorded'][name])) + '\n')
                report['frames_recorded'][name] += 1
        if record:
            frame_log.flush()
        return latest

    def capture(label, latest):
        if len(report['snapshots']) >= args.max_snapshots:
            raise RuntimeError('Snapshot limit (%d, including initial) reached' % args.max_snapshots)
        folder = args.output_dir / ('%02d_%s' % (len(report['snapshots']), label))
        folder.mkdir()
        observation = {'label': label, 'cameras': {}, 'hardware_synchronized': False}
        for name in names:
            image_path = folder / (name + '.png')
            if not cv2.imwrite(str(image_path), latest[name]['bgr']):
                raise RuntimeError('Cannot save image: ' + str(image_path))
            observation['cameras'][name] = dict(camera_info[name],
                **latest[name]['metadata'], rgb_path=str(image_path.resolve()))
        path = folder / 'observation.json'
        write_json(path, observation)
        report['snapshots'].append(str(path.resolve()))
        return str(path.resolve())

    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        signal.signal(signal.SIGTERM, lambda *_: stopped.set())
        signal.signal(signal.SIGINT, lambda *_: stopped.set())
        frame_log = (args.output_dir / 'frames.jsonl').open('x')
        for name, serial in zip(names, args.serials):
            pipeline, config = rs.pipeline(), rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            # start() may acquire resources before raising; retain the pipeline
            # first so cleanup always attempts stop and reports its outcome.
            streams[name] = pipeline
            profile = pipeline.start(config)
            actual_serial = profile.get_device().get_info(rs.camera_info.serial_number)
            if actual_serial != serial:
                raise RuntimeError('Camera serial mismatch: ' + name)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            camera_info[name] = {'serial': serial, 'intrinsics': {
                'width': intr.width, 'height': intr.height, 'fx': intr.fx, 'fy': intr.fy,
                'ppx': intr.ppx, 'ppy': intr.ppy, 'model': str(intr.model),
                'coeffs': list(intr.coeffs)}, 'pixel_format': 'BGR8'}
            writers[name] = cv2.VideoWriter(str(args.output_dir / (name + '.avi')),
                cv2.VideoWriter_fourcc(*'MJPG'), 15.0, (640, 480))
            if not writers[name].isOpened():
                raise RuntimeError('Cannot open video writer: ' + name)
        report['cameras'] = camera_info
        for _ in range(15):
            next_frames(record=False)
        latest = next_frames()
        initial = capture('initial', latest)
        report['status'] = 'recording'
        ready_path = args.output_dir / 'ready.json'
        write_json(ready_path, dict(report, initial_observation=initial))
        emit({'event': 'ready', 'ready': str(ready_path.resolve()),
              'cameras': camera_info, 'initial_observation': initial})
        threading.Thread(target=receive, daemon=True).start()
        while not stopped.is_set() and time.monotonic() < deadline:
            latest = next_frames()
            try:
                request = requests.get_nowait()
            except queue.Empty:
                continue
            path = capture(request['label'], latest)
            emit({'event': 'captured', 'label': request['label'], 'observation': path})
        report['status'] = 'stopped' if stopped.is_set() else 'deadline'
    except Exception as exc:
        if isinstance(exc, TimeoutError) and (stopped.is_set() or time.monotonic() >= deadline):
            report['status'] = 'stopped' if stopped.is_set() else 'deadline'
        else:
            exit_code = 1
            report.update(status='failed', error=str(exc), error_type=type(exc).__name__)
            write_json(args.output_dir / 'failure.json', report)
            emit({'event': 'error', 'error': str(exc), 'error_type': type(exc).__name__})
    finally:
        stopped.set()
        report['cleanup_errors'] = []
        for name, pipeline in streams.items():
            try:
                pipeline.stop()
            except Exception as exc:
                report['cleanup_errors'].append({'camera': name, 'error': str(exc)})
        for writer in writers.values():
            writer.release()
        if frame_log is not None:
            frame_log.close()
        report['finished_at_s'] = time.time()
        if report['cleanup_errors']:
            exit_code = 1
            report['status'] = 'failed'
        write_json(args.output_dir / 'report.json', report)
        if exit_code:
            write_json(args.output_dir / 'failure.json', report)
        emit({'event': 'closed', 'status': report['status'],
              'frames_recorded': report['frames_recorded'],
              'report': str((args.output_dir / 'report.json').resolve())})
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
