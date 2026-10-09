"""Offline successful-action edit. Reads recordings; never connects devices."""
import argparse
import shutil
import bisect
import csv
import datetime as dt
import hashlib
import json
import sqlite3
import subprocess
import zipfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'artifacts/plug_extraction_valid_20261009_rebuild'
FFMPEG = 'ffmpeg'
TZ = dt.timezone(dt.timedelta(hours=8))
FPS, W, H = 15, 1920, 648
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for data in iter(lambda: f.read(1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def local(t):
    return dt.datetime.fromtimestamp(t, TZ).strftime('%H:%M:%S')


def prepare():
    old = json.loads((ROOT/'artifacts/charger_extraction_review_20261009/manifest.json').read_text())
    sources = [Path(g['original_directory']) for g in old['groups']]
    later = ROOT/'artifacts/left_lift_10cm_20261009_run1'
    sources += [later/name/'continuous_recording' for name in
                ('recording', 'recording_after_prediction_fix', 'recording_live_reference')]
    groups = []
    for directory in sources:
        report = json.loads((directory/'report.json').read_text())
        assert report['status'] == 'closed' and report['failure'] is None
        frames = {side: [] for side in ('front', 'left_hand')}
        for line in (directory/'frames.jsonl').open():
            row = json.loads(line)
            if row['name'] in frames:
                frames[row['name']].append((row['host_received_at'], row['video_frame_index']))
        groups.append(dict(directory=directory, report=report, frames=frames))
    with sqlite3.connect((ROOT/'projects/piperx_cloth_demo/runs/pair_sessions.sqlite').as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        rows = [dict(r) for r in db.execute('SELECT * FROM pair_single_arm_maintenance_actions ORDER BY started_at')]
    included, excluded = [], []
    for row in rows:
        path = Path(row['record_path'])
        assert sha(path) == row['result_sha256']
        result = json.loads(path.read_text())
        if row['status'] != 'complete' or result['ok'] is not True:
            excluded.append(dict(run_id=row['run_id'], started_at=row['started_at'],
                                 ended_at=(result.get('after') or {}).get('left', {}).get('timestamp', row['started_at']),
                                 reason='Failed or refused action omitted', errors=result['errors']))
            continue
        assert not result['errors'] and result['hardware_commands_sent'] in (1, 4)
        assert result['transmission_counts']['right']['sent_frames'] == 0
        events = (json.loads(line) for line in path.with_name('events.jsonl').open())
        sends = [e['unix_s'] for e in events if e['event'] == 'single_supervised_action_sent_unconfirmed']
        assert len(sends) == 1
        start, end = sends[0]-1.5, result['after']['left']['timestamp']+2.5
        matches = [g for g in groups if g['report']['started_at'] <= start < end <= g['report']['finished_at']]
        assert len(matches) == 1, (row['run_id'], start, end)
        group = matches[0]
        index = len(included)+1
        stage = '夹持充电器' if result['operation'] == 'single_supervised_gripper' else '分段向上拔出'
        if 'left_extract_5mm_' in str(group['directory']):
            stage = '继续上提，插脚脱离插孔'
        if 'left_lift_10cm_' in str(group['directory']):
            stage = '完全拔出后的有效上提'
        a, b = result['before']['left'], result['after']['left']
        item = dict(index=index, run_id=row['run_id'], stage=stage, source_directory=str(group['directory']),
                    source_start_unix_s=start, source_end_unix_s=end, source_time_local=local(start),
                    output_frame_count=round((end-start)*FPS), receipt_sha256=row['result_sha256'],
                    measured_flange_z_change_mm=(b['pose_m_rad'][2]-a['pose_m_rad'][2])*1000,
                    gripper_change_mm=(b['gripper']['width_m']-a['gripper']['width_m'])*1000,
                    _group=group)
        included.append(item)
    for cut in included:
        for failure in excluded:
            assert not (cut['source_start_unix_s'] < failure['ended_at']+1
                        and cut['source_end_unix_s'] > failure['started_at']-1), (cut['run_id'], failure['run_id'])
    assert len(included) == 27 and len(excluded) == 6
    return included, excluded


class Reader:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        assert self.cap.isOpened(), str(path)
        self.index, self.frame = -1, None

    def at(self, index):
        if self.index == index:
            return self.frame
        if self.index+1 != index:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.cap.read()
        assert ok, ('missing source frame', index)
        self.index, self.frame = index, frame
        return frame

    def close(self):
        self.cap.release()


def header(cut, total):
    image = Image.new('RGB', (W, 108), '#102238')
    draw = ImageDraw.Draw(image)
    draw.text((24, 10), '2026-10-09  插头拔出 · 有效片段剪辑', font=ImageFont.truetype(FONT, 32), fill='white')
    draw.text((1120, 15), '%02d/%02d  %s  %s' % (cut['index'], total, cut['source_time_local'], cut['stage']),
              font=ImageFont.truetype(FONT, 24), fill='#a4e4cf')
    for x, label in ((24, '全景'), (984, '左夹爪近景')):
        draw.text((x, 63), label, font=ImageFont.truetype(FONT, 25), fill='#c9d8e8')
    draw.text((1190, 67), '按时间拼接 · 保持原速 · 已删除失败与等待', font=ImageFont.truetype(FONT, 21), fill='#c9d8e8')
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


def fit(frame):
    h, w = frame.shape[:2]
    scale = min(960/w, 540/h)
    resized = cv2.resize(frame, (round(w*scale), round(h*scale)), interpolation=cv2.INTER_AREA)
    panel = np.zeros((540, 960, 3), np.uint8)
    y, x = (540-resized.shape[0])//2, (960-resized.shape[1])//2
    panel[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
    return panel


def render(cuts):
    cv2.setNumThreads(1)
    video = OUT/'plug_extraction_20261009_valid.mp4'
    command = [FFMPEG, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
               '-s', '%dx%d' % (W, H), '-r', str(FPS), '-i', '-', '-an', '-c:v', 'libx264',
               '-preset', 'veryfast', '-crf', '20', '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)]
    timeline = 0
    with (OUT/'encode.log').open('w') as log:
        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            for cut in cuts:
                group = cut.pop('_group')
                readers = {side: Reader(group['directory']/(side+'.avi')) for side in ('front', 'left_hand')}
                cut['output_start_s'] = timeline/FPS
                cut['source_frame_mapping'] = {side: [] for side in readers}
                times = {side: [x[0] for x in group['frames'][side]] for side in readers}
                banner = header(cut, len(cuts))
                try:
                    for i in range(cut['output_frame_count']):
                        t = cut['source_start_unix_s']+i/FPS
                        panels = []
                        for side, reader in readers.items():
                            j = bisect.bisect_left(times[side], t)
                            if j and (j == len(times[side]) or t-times[side][j-1] <= times[side][j]-t):
                                j -= 1
                            stamp, index = group['frames'][side][j]
                            assert abs(stamp-t) <= .2, ('source gap', side, stamp-t)
                            cut['source_frame_mapping'][side].append(dict(output_frame=timeline,
                                                                        source_frame=index, source_host_time=stamp))
                            panels.append(fit(reader.at(index)))
                        frame = np.vstack((banner, np.hstack(panels)))
                        proc.stdin.write(frame.tobytes())
                        if i == 0 or i == cut['output_frame_count']-1:
                            cv2.imwrite(str(OUT/('preview_%02d_%s.jpg' % (cut['index'], 'start' if i == 0 else 'end'))), frame)
                        timeline += 1
                finally:
                    for reader in readers.values():
                        reader.close()
                cut['output_end_s'] = timeline/FPS
                print(json.dumps({'clip':cut['index'], 'of':len(cuts), 'output_seconds':timeline/FPS}), flush=True)
        finally:
            proc.stdin.close()
        assert proc.wait() == 0, (OUT/'encode.log').read_text()
    return video, timeline


def package(cuts, excluded, video, frames):
    subprocess.run([FFMPEG, '-v', 'error', '-i', str(video), '-f', 'null', '-'], check=True, stderr=subprocess.PIPE)
    cap = cv2.VideoCapture(str(video))
    info = dict(frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), fps=cap.get(cv2.CAP_PROP_FPS),
                width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    cap.release()
    assert info == dict(frames=frames, fps=FPS, width=W, height=H)
    manifest = dict(schema='plug_successful_motion_edit_v1', date='2026-10-09', timezone='Asia/Shanghai',
                    successful_actions=len(cuts), omitted_failed_actions=excluded, cuts=cuts,
                    video=dict(path=video.name, bytes=video.stat().st_size, sha256=sha(video), duration_s=frames/FPS, **info),
                    audio='Original recordings have no audio; no synthetic audio added',
                    editing='1.5s before successful send to 2.5s after receipt; failures and long waiting omitted; chronological hard cuts',
                    timing='Each camera sampled nearest host receipt to a common 15fps timeline; not hardware synchronization',
                    result='Charger was extracted and remained held in the last successful footage. Additional 10cm lift was not completed.',
                    subsequent_event='Further motion was refused with J5 motor_overheating/driver_error flags; that failed interval is omitted.',
                    validation='All selected source frames read successfully, excluded intervals do not overlap; final MP4 fully decoded without errors',
                    original_files_modified=False)
    (OUT/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    with (OUT/'timeline.csv').open('w', newline='', encoding='utf-8-sig') as f:
        fields = ['index','stage','source_time_local','output_start_s','output_end_s','run_id','measured_flange_z_change_mm']
        writer = csv.DictWriter(f, fieldnames=fields);writer.writeheader()
        writer.writerows({k:cut[k] for k in fields} for cut in cuts)
    (OUT/'README.txt').write_text('2026-10-09 插头拔出｜有效过程剪辑\n\n'
        '打开 plug_extraction_20261009_valid.mp4，或打开 index.html。\n'
        '保留 27 个成功动作：1 次夹持、26 次上提。删除 6 次失败/拒绝区间和长时间等待；原始录像保留未修改。\n'
        '左画面是全景，右画面是左夹爪近景。按实际帧接收时间对齐后输出 15 fps，保持原速，无音轨。\n'
        '这是按时间拼接的有效片段，不是无间断录像。开口变化/法兰位移和真实物体结果分别记录。\n'
        '结果：充电器已完全拔出，最后有效片段中仍由左爪夹持；额外 10 cm 上提未完成。之后 J5 故障回报导致停止新增动作，故障区间不在本片。\n'
        'timeline.csv 为片段索引；manifest.json 保存来源、实际帧映射、排除记录和校验值。\n', encoding='utf-8')
    rows = ''.join('<tr><td>%02d</td><td>%s</td><td>%s</td><td>%.1f–%.1f 秒</td></tr>' %
                   (c['index'], c['stage'], c['source_time_local'], c['output_start_s'], c['output_end_s']) for c in cuts)
    (OUT/'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>今天的拔插头视频 · 有效片段</title><style>body{font:16px/1.6 system-ui;max-width:1100px;margin:36px auto;padding:0 20px;background:#eef3f8;color:#102238}video{width:100%;background:#000}a{color:#076d80}table{width:100%;border-collapse:collapse}td,th{padding:8px;text-align:left;border-bottom:1px solid #ccd8e2}</style>
<h1>2026-10-09 拔插头 · 有效片段拼接</h1>
<p>按时间保留成功夹持和上提，去掉失败动作、调试与长时间等待。左侧全景，右侧左夹爪近景，保持原速。</p>
<video controls preload="metadata" src="plug_extraction_20261009_valid.mp4"></video>
<p><a download href="plug_extraction_20261009_valid.mp4">下载完整有效过程 MP4</a> · <a href="timeline.csv">片段时间表</a> · <a href="README.txt">说明</a></p>
<p>充电器已拔出；额外 10 cm 上提未完成。本片仅为有效动作剪辑，后续故障区间已删除。</p>
<table><tr><th>片段</th><th>阶段</th><th>实际时间</th><th>成片位置</th></tr>'''+rows+'</table></html>', encoding='utf-8')
    (OUT/'SHA256SUMS.txt').write_text(''.join(sha(OUT/n)+'  '+n+'\n' for n in
        [video.name, 'manifest.json','timeline.csv','README.txt','index.html']))
    zip_path = OUT/'plug_extraction_20261009_valid.zip'
    files = [video.name, 'index.html', 'README.txt','timeline.csv','manifest.json','SHA256SUMS.txt']
    with zipfile.ZipFile(zip_path, 'w', allowZip64=True) as z:
        for name in files:
            z.write(OUT/name, 'plug_extraction_20261009/'+name,
                    compress_type=zipfile.ZIP_STORED if name.endswith('.mp4') else zipfile.ZIP_DEFLATED)
    with zipfile.ZipFile(zip_path) as z:
        assert z.testzip() is None
    final = dict(mp4=str(video), mp4_bytes=video.stat().st_size, zip=str(zip_path), zip_bytes=zip_path.stat().st_size,
                 zip_sha256=sha(zip_path), duration_s=frames/FPS, successful_actions=len(cuts), excluded_failures=len(excluded), validation='passed')
    (OUT/'delivery.json').write_text(json.dumps(final,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(final, ensure_ascii=False), flush=True)


def main():
    global ROOT, OUT, FFMPEG, FONT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT, help='Original evidence workspace; contains the read-only ledger and recordings')
    parser.add_argument('--out', type=Path, default=OUT, help='New, empty output directory')
    parser.add_argument('--ffmpeg', default=FFMPEG)
    parser.add_argument('--font', default=FONT)
    parser.add_argument('--audit-only', action='store_true', help='Verify receipts and cut selection without encoding or writing files')
    args = parser.parse_args()
    ROOT, OUT = args.root.resolve(), args.out.resolve()
    FFMPEG, FONT = args.ffmpeg, args.font
    if not __debug__:
        parser.error('Assertions are evidence checks: do not run with python -O')
    cuts, excluded = prepare()
    if args.audit_only:
        print(json.dumps(dict(successful_actions=len(cuts), excluded_failures=len(excluded),
                             duration_s=sum(c['output_frame_count'] for c in cuts)/FPS,
                             hardware_accessed=False, files_written=False)))
        return
    if not shutil.which(FFMPEG) or not Path(FONT).is_file():
        parser.error('Provide an installed ffmpeg binary and Chinese font')
    if OUT.exists() and (not OUT.is_dir() or any(OUT.iterdir())):
        parser.error('Output must be a new or empty directory; original deliveries are preserved')
    OUT.mkdir(parents=True, exist_ok=True)
    video, frames = render(cuts)
    package(cuts, excluded, video, frames)


if __name__ == '__main__':
    main()
