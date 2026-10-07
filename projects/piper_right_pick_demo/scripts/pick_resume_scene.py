"""Check a held, pre-close scene against the previous run's final video frames."""
import hashlib
import json
from pathlib import Path


def verify_resume_scene(source_report_path, observation_path):
    import cv2
    import numpy as np
    source = Path(source_report_path).resolve()
    report = json.loads(source.read_text())
    observation = json.loads(Path(observation_path).read_text())
    initial_path = report.get('initial_scene', {}).get('observation')
    if initial_path is None:
        initial_path = next((capture.get('observation') for capture in report.get('captures', [])
                             if capture.get('label') == 'initial' and capture.get('observation')), None)
    if initial_path is None:
        raise ValueError('Reference report lacks its initial camera identity observation')
    initial = json.loads(Path(initial_path).read_text())
    results = {}
    for name in ('front', 'right_hand'):
        frame = observation['cameras'][name]
        if frame['serial'] != initial['cameras'][name]['serial']:
            raise ValueError('Resume camera serial changed: ' + name)
        video = source.parent / 'camera_recording' / (name + '.avi')
        cap = cv2.VideoCapture(str(video))
        try:
            count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if count < 10:
                raise ValueError('Resume reference video is missing or incomplete')
            cap.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
            ok, previous = cap.read()
        finally:
            cap.release()
        current = cv2.imread(frame['rgb_path'])
        if not ok or current is None or current.shape != previous.shape:
            raise ValueError('Cannot compare previous final frame and current scene')
        def gray(image):
            return cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (160, 120)).astype(float).ravel()
        correlation = float(np.corrcoef(gray(previous), gray(current))[0, 1])
        if not np.isfinite(correlation) or correlation < .92:
            raise ValueError('Camera/scene changed since stopped trajectory: %s correlation %.4f' % (name, correlation))
        result = {'reference_video': str(video), 'reference_frame': count - 1,
                  'current_rgb': frame['rgb_path'], 'image_correlation': correlation}
        if name == 'right_hand':
            def cube(image):
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (0, 85, 5), (30, 255, 255)) | cv2.inRange(hsv, (170, 85, 5), (179, 255, 255))
                n, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
                if n < 2:
                    raise ValueError('Resume requires a visible red cube')
                index = 1 + int(np.argmax(stats[1:, 4]))
                area = int(stats[index, 4])
                if area < 100 or sum(int(s[4]) > .5 * area for s in stats[1:]) != 1:
                    raise ValueError('Resume cube is too small or ambiguous')
                return centers[index], area
            before, area_before = cube(previous)
            after, area_after = cube(current)
            displacement = float(np.linalg.norm(after - before))
            ratio = float(area_after / area_before)
            if displacement > 8 or not .65 <= ratio <= 1.5:
                raise ValueError('Red cube moved since stopped trajectory: %.2f pixels, area ratio %.3f' % (displacement, ratio))
            result.update(cube_centroid_before_px=before.tolist(), cube_centroid_after_px=after.tolist(),
                          cube_displacement_px=displacement, cube_area_ratio=ratio)
        results[name] = result
    return {'passed': True, 'cameras': results,
            'source_report_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'observation': str(Path(observation_path).resolve()),
            'meaning': 'Current images are consistent with the previously stopped pre-close scene; this is not a new calibration.'}
