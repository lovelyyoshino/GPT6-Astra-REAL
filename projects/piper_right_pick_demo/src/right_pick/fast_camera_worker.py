"""Single owner for continuous three-camera RGB videos and fresh snapshots.

No depth stream, calibration query, object detector or robot import is used.
AVI playback uses the configured nominal FPS; frames.jsonl retains real timing.
The stdin/stdout protocol remains one result per capture request.
"""
import argparse
import json
import queue
import signal
import sys
import threading
import time
import uuid
from pathlib import Path

from .camera import RealSenseRig, _dependency


class ContinuousRGBRecorder:
    """One acquisition loop records all views, including while the model waits."""

    def __init__(self, configs, output):
        # Reuse strict serial/dimension validation only; this does not open USB.
        self.configs = RealSenseRig(configs, rgb_only=True).configs
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.recording_dir = self.output / "continuous_recording"
        self.recording_dir.mkdir(exist_ok=False)
        self.streams, self.writers = {}, {}
        self.cv2 = self.np = self.rs = None
        self.frame_log = None
        self.closed = False
        self.close_errors = []
        self.previous_frame_numbers = {}
        self.report = {
            "status": "starting", "started_at": time.time(),
            "recording_kind": "continuous_rgb_acquisition",
            "video_timing": "nominal configured fps; actual receipts/device times in frames.jsonl",
            "depth_enabled": False, "extrinsics_applied": False,
            "hardware_synchronized": False, "exposure_age_verified": False,
            "requested_cameras": self.configs,
            "frames_recorded": dict.fromkeys(self.configs, 0),
            "device_frame_gaps": dict.fromkeys(self.configs, 0),
            "snapshots": [], "cleanup_errors": self.close_errors,
        }

    def _open(self):
        if self.rs is not None:
            return
        self.np, self.cv2 = _dependency("numpy"), _dependency("cv2")
        self.rs = _dependency("pyrealsense2")
        self.frame_log = (self.recording_dir / "frames.jsonl").open("x", encoding="utf-8")
        for name, conf in self.configs.items():
            pipeline, config = self.rs.pipeline(), self.rs.config()
            # Retain before start: even a partially failed start must be closed.
            self.streams[name] = {"pipeline": pipeline}
            config.enable_device(conf["serial"])
            config.enable_stream(self.rs.stream.color, conf["width"], conf["height"],
                                 self.rs.format.bgr8, conf["fps"])
            device = pipeline.start(config).get_device()
            if device.get_info(self.rs.camera_info.serial_number) != conf["serial"]:
                raise RuntimeError("Camera serial mismatch: " + name)
            self.streams[name]["model"] = device.get_info(self.rs.camera_info.name)
            writer = self.cv2.VideoWriter(str(self.recording_dir / (name + ".avi")),
                self.cv2.VideoWriter_fourcc(*"MJPG"), float(conf["fps"]),
                (conf["width"], conf["height"]))
            self.writers[name] = writer
            if not writer.isOpened():
                raise RuntimeError("Cannot open continuous video: " + name)
        self.report["status"] = "recording"
        self._save_report()

    def next_frames(self):
        """Acquire once and record once, independent of snapshot/model requests."""
        self._open()
        latest = {}
        for name, stream in self.streams.items():
            color = stream["pipeline"].wait_for_frames(1000).get_color_frame()
            if not color:
                raise RuntimeError("Missing RGB frame: " + name)
            received, monotonic = time.time(), time.monotonic()
            pixels = self.np.asanyarray(color.get_data()).copy()
            conf = self.configs[name]
            if pixels.shape != (conf["height"], conf["width"], 3):
                raise RuntimeError("Unexpected RGB shape: " + name)
            number = int(color.get_frame_number())
            previous = self.previous_frame_numbers.get(name)
            if previous is not None and number <= previous:
                raise RuntimeError("Repeated/non-monotonic RGB frame: " + name)
            if previous is not None:
                self.report["device_frame_gaps"][name] += max(0, number - previous - 1)
            self.previous_frame_numbers[name] = number
            metadata = {
                "name": name, "serial": conf["serial"], "model": stream["model"],
                "host_received_at": received, "timestamp": received,
                "timestamp_kind": "host_receipt_wall_clock",
                "host_received_monotonic_s": monotonic,
                "device_timestamp_ms": float(color.get_timestamp()),
                "timestamp_domain": str(color.get_frame_timestamp_domain()),
                "frame_number": number, "depth_enabled": False,
                "exposure_age_verified": False,
                "video_frame_index": self.report["frames_recorded"][name],
                "video_path": str(self.recording_dir / (name + ".avi")),
            }
            self.writers[name].write(pixels)
            self.frame_log.write(json.dumps(metadata, allow_nan=False) + "\n")
            self.report["frames_recorded"][name] += 1
            latest[name] = {"pixels": pixels, "metadata": metadata}
        self.frame_log.flush()
        return latest

    def snapshot(self, latest):
        """Save the fresh frames already recorded by this owner's acquisition loop."""
        capture_id = "capture_" + uuid.uuid4().hex
        folder = self.output / capture_id
        folder.mkdir(exist_ok=False)
        result = {"capture_id": capture_id, "directory": str(folder), "cameras": {},
                  "extrinsics_applied": False, "historical_extrinsics_used": False,
                  "hardware_synchronized": False,
                  "timestamp_kind": "earliest_host_receipt_wall_clock",
                  "recording_report": str(self.recording_dir / "report.json")}
        for name, frame in latest.items():
            path = folder / (name + ".png")
            if not self.cv2.imwrite(str(path), frame["pixels"]):
                raise RuntimeError("Failed to save RGB frame: " + str(path))
            result["cameras"][name] = dict(frame["metadata"], rgb_path=str(path),
                                           image_encoding="PNG RGB (OpenCV arrays BGR)")
        completed = time.monotonic()
        receipts = [frame["host_received_monotonic_s"] for frame in result["cameras"].values()]
        result["captured_at"] = min(frame["timestamp"] for frame in result["cameras"].values())
        result["capture_completed_at"] = time.time()
        result["host_receipt_skew_s"] = max(receipts) - min(receipts)
        result["max_host_receipt_skew_s"] = 0.15
        result["host_receipt_skew_exceeded"] = result["host_receipt_skew_s"] > 0.15
        for frame in result["cameras"].values():
            frame["host_receipt_age_at_capture_complete_s"] = completed - frame["host_received_monotonic_s"]
            frame["max_host_receipt_age_s"] = 0.6
            frame["host_receipt_stale"] = frame["host_receipt_age_at_capture_complete_s"] > 0.6
        result["metadata_path"] = str(folder / "observation.json")
        Path(result["metadata_path"]).write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        self.report["snapshots"].append(result["metadata_path"])
        self._save_report()
        return result

    def _save_report(self):
        target = self.recording_dir / "report.json"
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.report, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(target)

    def close(self, failure=None):
        if self.closed:
            return
        self.closed = True
        for name, stream in self.streams.items():
            try:
                stream["pipeline"].stop()
            except Exception as exc:
                self.close_errors.append({"camera": name, "operation": "pipeline_stop", "error": str(exc)})
        for name, writer in self.writers.items():
            try:
                writer.release()
            except Exception as exc:
                self.close_errors.append({"camera": name, "operation": "video_release", "error": str(exc)})
        if self.frame_log is not None:
            try:
                self.frame_log.close()
            except Exception as exc:
                self.close_errors.append({"operation": "frame_log_close", "error": str(exc)})
        self.report.update(status="failed" if failure or self.close_errors else "closed",
                           finished_at=time.time(), failure=str(failure) if failure else None)
        self._save_report()


def _commands(stream, requests, stopped):
    """Only this thread reads stdin; only the main thread ever accesses cameras."""
    while not stopped.is_set():
        try:
            line = stream.readline(4097)
            if not line:
                command = {"command": "close"}
            else:
                if len(line) > 4096:
                    raise ValueError("Camera command exceeds 4096 characters")
                command = json.loads(line)
                if command not in ({"command": "capture"}, {"command": "close"}):
                    raise ValueError("Only capture/close commands are supported")
        except Exception as exc:
            command = exc
        while not stopped.is_set():
            try:
                requests.put(command, timeout=0.1)
                break
            except queue.Full:
                continue
        if isinstance(command, Exception) or command == {"command": "close"}:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        configs = json.load(stream)
    if set(configs) != {"front", "left_hand", "right_hand"}:
        raise ValueError("All three bound cameras are required")
    recorder = ContinuousRGBRecorder(configs, args.output)
    requests, stopped = queue.Queue(maxsize=32), threading.Event()
    failure = None
    previous_handlers = {}
    try:
        if threading.current_thread() is threading.main_thread():
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[signum] = signal.signal(signum, lambda *_: stopped.set())
        threading.Thread(target=_commands, args=(sys.stdin, requests, stopped), daemon=True).start()
        while not stopped.is_set():
            try:
                command = requests.get_nowait()
            except queue.Empty:
                command = None
            if isinstance(command, Exception):
                raise command
            if command == {"command": "close"}:
                return 0
            # A requested snapshot uses acquisition AFTER dequeuing that request.
            # Model latency therefore never pauses local recording.
            latest = recorder.next_frames()
            if command == {"command": "capture"}:
                observation = recorder.snapshot(latest)
                print(json.dumps({"ok": True, "observation": observation}, allow_nan=False), flush=True)
        failure = "camera worker interrupted by signal"
        return 1
    except Exception as exc:
        failure = exc
        raise
    finally:
        stopped.set()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        recorder.close(failure=failure)
        if recorder.close_errors:
            raise RuntimeError("RGB camera shutdown failed: " + repr(recorder.close_errors))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
