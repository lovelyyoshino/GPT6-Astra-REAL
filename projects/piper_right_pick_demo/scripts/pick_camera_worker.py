#!/usr/bin/env python3
"""Finite, camera-only child of one supervised pick; records two streams.

No robot, CAN, ROS, network, arbitrary commands or executable input. Stdin only
accepts capture labels and stop, for at most 16 snapshots and 480 seconds.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import re
import sys
import threading
import time


def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
    from right_pick.camera import _intrinsics_dict

    args.output_dir.mkdir(parents=True, exist_ok=False)
    configuration = json.loads(args.config.read_text())['cameras']
    names = ('front', 'right_hand')
    streams, writers = {}, {}
    requests = queue.Queue(maxsize=20)
    report = {'operation': 'finite_pick_two_camera_recording', 'started_at_s': time.time(),
              'robot_commands_sent': 0, 'cameras': list(names), 'snapshots': [],
              'frames_recorded': {n: 0 for n in names}, 'status': 'starting',
              'video_timing': 'nominal 15 fps AVI; actual receipt/device timestamps in video_frames.jsonl'}
    def receive_requests():
        for line in sys.stdin:
            try:
                request = json.loads(line)
                if not isinstance(request, dict) or request.get('op') not in ('capture', 'stop'):
                    raise ValueError('Only capture and stop are supported')
                if request['op'] == 'capture' and not re.fullmatch('[a-z][a-z0-9_]{0,31}', request.get('label', '')):
                    raise ValueError('Invalid capture label')
                requests.put_nowait(request)
            except Exception as exc:
                emit({'event': 'request_error', 'error': str(exc)})
        try:
            requests.put_nowait({'op': 'stop'})
        except queue.Full:
            pass
    def save_capture(label, latest):
        index = len(report['snapshots'])
        folder = args.output_dir / ('%02d_%s' % (index, label))
        folder.mkdir(exist_ok=False)
        obs = {'capture_id': folder.name, 'directory': str(folder.resolve()),
               'cameras': {}, 'hardware_synchronized': False,
               'timestamp_kind': 'host_receipt_wall_clock', 'extrinsics_applied': False,
               'robot_base_transform': None, 'label': label}
        times = []
        for name, frame in latest.items():
            rgb = folder / (name + '.png')
            if not cv2.imwrite(str(rgb), frame['bgr']):
                raise RuntimeError('Could not write RGB image')
            raw_path = folder / (name + '_depth_raw.npy')
            depth_path = folder / (name + '_depth_m.npy')
            raw = frame['raw']
            metres = raw.astype(np.float32) * streams[name]['scale']
            metres[(raw == 0) | (raw == 65535)] = np.nan
            np.save(str(raw_path), raw, allow_pickle=False)
            np.save(str(depth_path), metres, allow_pickle=False)
            metadata = dict(frame['metadata'])
            metadata.update(rgb_path=str(rgb.resolve()), depth_raw_path=str(raw_path.resolve()),
                            depth_m_path=str(depth_path.resolve()), depth_scale_m=streams[name]['scale'],
                            depth_aligned_to='color', depth_enabled=True,
                            usable_depth_fraction_0_1_to_2_m=float(np.mean((metres >= .1) & (metres <= 2.))),
                            depth_definition='camera optical-axis Z metres; invalid raw 0/65535 become NaN')
            obs['cameras'][name] = metadata
            times.append(metadata['host_received_at'])
        obs.update(captured_at=min(times), host_receipt_skew_s=max(times)-min(times),
                   metadata_path=str((folder / 'observation.json').resolve()))
        (folder / 'observation.json').write_text(json.dumps(obs, ensure_ascii=False, indent=2, allow_nan=False))
        report['snapshots'].append(obs['metadata_path'])
        emit({'event': 'captured', 'label': label, 'observation': obs['metadata_path']})

    frame_log = (args.output_dir / 'video_frames.jsonl').open('x')
    try:
        for name in names:
            cfg = configuration[name]
            pipeline, config = rs.pipeline(), rs.config()
            config.enable_device(str(cfg['serial']))
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            profile = pipeline.start(config)
            streams[name] = {'pipeline': pipeline}
            device = profile.get_device()
            if device.get_info(rs.camera_info.serial_number) != str(cfg['serial']):
                raise RuntimeError('Camera serial mismatch: ' + name)
            streams[name].update(align=rs.align(rs.stream.color),
                                 scale=float(device.first_depth_sensor().get_depth_scale()),
                                 serial=str(cfg['serial']))
            writer = cv2.VideoWriter(str(args.output_dir / (name + '.avi')),
                                     cv2.VideoWriter_fourcc(*'MJPG'), 15., (640, 480))
            if not writer.isOpened():
                raise RuntimeError('Could not open video writer: ' + name)
            writers[name] = writer
        # Warm both streams together; initial startup depth/auto-exposure can be poor.
        for _ in range(30):
            for stream in streams.values():
                stream['pipeline'].wait_for_frames(5000)
        threading.Thread(target=receive_requests, daemon=True).start()
        report['status'] = 'recording'
        emit({'event': 'ready', 'cameras': list(names)})
        deadline = time.monotonic() + 480.
        while time.monotonic() < deadline:
            latest = {}
            for name, stream in streams.items():
                frames = stream['pipeline'].wait_for_frames(2000)
                receipt = time.time()
                aligned = stream['align'].process(frames)
                color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
                if not color or not depth:
                    raise RuntimeError('Missing RGB or depth: ' + name)
                bgr, raw = np.asanyarray(color.get_data()).copy(), np.asanyarray(depth.get_data()).copy()
                metadata = {'name': name, 'serial': stream['serial'], 'host_received_at': receipt,
                            'timestamp': receipt, 'frame_number': int(color.get_frame_number()),
                            'device_timestamp_ms': float(color.get_timestamp()),
                            'intrinsics': _intrinsics_dict(color.profile.as_video_stream_profile().intrinsics),
                            'rgb_depth_device_skew_ms': abs(float(color.get_timestamp())-float(depth.get_timestamp()))}
                latest[name] = {'bgr': bgr, 'raw': raw, 'metadata': metadata}
                writers[name].write(bgr)
                report['frames_recorded'][name] += 1
                frame_log.write(json.dumps({'camera': name, 'video_index': report['frames_recorded'][name]-1,
                    'host_received_at': receipt, 'device_timestamp_ms': metadata['device_timestamp_ms']}) + '\n')
            frame_log.flush()
            try:
                request = requests.get_nowait()
            except queue.Empty:
                continue
            if request['op'] == 'stop':
                report['status'] = 'stopped'
                break
            if len(report['snapshots']) >= 16:
                raise RuntimeError('Snapshot budget exhausted')
            save_capture(request['label'], latest)
        else:
            report['status'] = 'deadline'
        return 0
    except Exception as exc:
        report.update(status='failed', error=str(exc), error_type=type(exc).__name__)
        emit({'event': 'failed', 'error': str(exc)})
        return 1
    finally:
        frame_log.close()
        for writer in writers.values():
            writer.release()
        report['cleanup_errors'] = []
        for name, stream in streams.items():
            try:
                stream['pipeline'].stop()
            except Exception as exc:
                report['cleanup_errors'].append({'camera': name, 'error': str(exc)})
        report['finished_at_s'] = time.time()
        (args.output_dir / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    raise SystemExit(main())
