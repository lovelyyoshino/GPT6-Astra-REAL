#!/usr/bin/env python3
"""Export only finalized local RGB AVI files; never opens cameras or robots.

Preserves all frames and source nominal playback rate, without cropping,
resizing, cutting waits or speeding motion. Actual time remains in frames.jsonl.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

DEFAULT_FFMPEG = "/home/agilex/miniconda3/envs/pi0/lib/python3.11/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2"
ROLES = {"front": "third_person", "right_hand": "first_person_right_wrist", "left_hand": "left_wrist"}


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def signature(path):
    s = path.stat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def metadata(path):
    import cv2  # File decoding only; never VideoCapture(device_index).
    if not path.is_file() or path.suffix.lower() not in (".avi", ".mp4"):
        raise ValueError("Only an existing regular AVI/MP4 file is accepted")
    reader = cv2.VideoCapture(str(path.resolve()))
    try:
        if not reader.isOpened():
            raise RuntimeError("Cannot open finalized video file: " + str(path))
        return dict(path=str(path.resolve()), frames=int(reader.get(cv2.CAP_PROP_FRAME_COUNT)),
                    fps=float(reader.get(cv2.CAP_PROP_FPS)), width=int(reader.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    height=int(reader.get(cv2.CAP_PROP_FRAME_HEIGHT)), bytes=path.stat().st_size)
    finally:
        reader.release()


def final_progress(text):
    result = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def export(recording_dir, output_dir, views, ffmpeg=DEFAULT_FFMPEG):
    recording_dir, output_dir = Path(recording_dir).resolve(), Path(output_dir).resolve()
    report_path = recording_dir / "report.json"
    original = json.loads(report_path.read_text())
    finished = original.get("finished_at", original.get("finished_at_s"))
    if (original.get("status") not in ("closed", "stopped", "deadline") or not finished
            or original.get("cleanup_errors") != []):
        raise ValueError("Recording is not cleanly finalized; do not convert a live/failed stream")
    if not views or len(views) != len(set(views)) or any(view not in ROLES for view in views):
        raise ValueError("Use unique supported camera views")
    ffmpeg = str(Path(ffmpeg).resolve(strict=True))
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {"operation": "closed_local_rgb_video_export", "source_report": str(report_path),
              "source_report_sha256": digest(report_path), "started_at": time.time(),
              "status": "exporting", "frames_added_removed_or_retimed": False,
              "video_timing": "original nominal AVI fps; real receive times remain in original frames.jsonl",
              "hardware_accessed": False, "views": {}}
    try:
        for view in views:
            source = recording_dir / (view + ".avi")
            expected = original.get("frames_recorded", {}).get(view)
            if type(expected) is not int or expected <= 0:
                raise ValueError("Final recording report lacks a positive frame count: " + view)
            initial_signature = signature(source)
            info = metadata(source)
            if info["frames"] != expected or info["fps"] <= 0:
                raise ValueError("Final AVI metadata differs from recorded frame count: " + view)
            target = output_dir / (view + ".mp4")
            print(json.dumps({"event": "exporting", "view": view, "source_frames": expected}), flush=True)
            argv = [ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-n", "-i", str(source),
                    "-map", "0:v:0", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                    "-threads", "2", "-pix_fmt", "yuv420p", "-fps_mode", "passthrough",
                    "-movflags", "+faststart", "-progress", "pipe:1", "-nostats", str(target)]
            encoded = subprocess.run(argv, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, timeout=3600)
            (output_dir / (view + "_encode.log")).write_text(encoded.stdout + encoded.stderr)
            progress = final_progress(encoded.stdout)
            if (progress.get("progress") != "end" or int(progress.get("frame", -1)) != expected
                    or int(progress.get("dup_frames", -1)) != 0 or int(progress.get("drop_frames", -1)) != 0):
                raise RuntimeError("Encoding did not preserve every source frame: " + view)
            converted = metadata(target)
            if any(converted[key] != info[key] for key in ("frames", "width", "height")) or abs(converted["fps"]-info["fps"]) > .0001:
                raise RuntimeError("Export video metadata does not match source: " + view)
            checked = subprocess.run([ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-xerror",
                "-i", str(target), "-map", "0:v:0", "-an", "-progress", "pipe:1", "-nostats", "-f", "null", "-"],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3600)
            decoded = final_progress(checked.stdout)
            (output_dir / (view + "_full_decode.log")).write_text(checked.stdout + checked.stderr)
            if decoded.get("progress") != "end" or int(decoded.get("frame", -1)) != expected:
                raise RuntimeError("Full export decoding did not preserve all frames: " + view)
            if signature(source) != initial_signature or digest(report_path) != report["source_report_sha256"]:
                raise RuntimeError("Source changed during export; original stream was not immutable")
            report["views"][view] = dict(role=ROLES[view], original=info, export=converted,
                encoding_progress=progress, full_decode_progress=decoded, verified_frames=expected,
                exported_sha256=digest(target), nominal_duration_s=expected/info["fps"])
            print(json.dumps({"event": "verified", "view": view, "frames": expected, "path": str(target)}), flush=True)
        report["status"] = "exported_and_verified"
        return report
    except Exception as exc:
        report.update(status="failed", error_type=type(exc).__name__, error=str(exc))
        raise
    finally:
        report["finished_at"] = time.time()
        (output_dir / "video_exports.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--views", nargs="+", choices=tuple(ROLES), default=["front", "right_hand"])
    parser.add_argument("--ffmpeg", default=DEFAULT_FFMPEG)
    args = parser.parse_args()
    export(args.recording_dir, args.output_dir, args.views, args.ffmpeg)


if __name__ == "__main__":
    main()
