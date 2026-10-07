"""Persistent RGB capture boundary and explicitly historical replay sources.

No object detector, camera-to-base transform, depth query or target coordinate
estimator belongs in this module. Robot state is acquired separately.
"""
import json
import shutil
import time
import os
import select
import signal
import subprocess
from pathlib import Path


CAMERA_ROLES = ("front", "left_hand", "right_hand")


class SubprocessRGBCameras:
    """Keep three RGB streams open in the explicitly configured camera Python.

    The single owner continuously records all three views between capture()
    calls. Snapshots and videos share those streams. Ending this process never
    ends a robot driver; it only finalizes the local camera recordings.
    """
    nonphysical = False

    def __init__(self, config, output_dir):
        if config.get("backend") != "realsense" or set(config.get("cameras", {})) != set(CAMERA_ROLES):
            raise ValueError("Explicit three-view RealSense configuration required")
        executable = config.get("camera_python")
        if not executable or not Path(executable).is_file():
            raise ValueError("Configured camera Python executable is missing")
        self.config = config
        self.output_dir = Path(output_dir).resolve()
        self.recording_dir = self.output_dir / "continuous_recording"
        self.process = None
        self.stderr = None

    def _start(self):
        if self.process is not None:
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        config_path = self.output_dir / "camera_bindings.json"
        config_path.write_text(json.dumps(self.config["cameras"], indent=2), encoding="utf-8")
        argv = [self.config["camera_python"], "-u", "-m", "right_pick.fast_camera_worker",
                "--config", str(config_path), "--output", str(self.output_dir)]
        env = os.environ.copy()
        env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
        self.stderr = (self.output_dir / "camera_worker.stderr").open("w", encoding="utf-8")
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.stderr, text=True, env=env, start_new_session=True)

    def capture(self):
        self._start()
        if self.process.poll() is not None:
            raise RuntimeError("RGB worker exited; no automatic restart")
        self.process.stdin.write('{"command":"capture"}\n')
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 20.0)[0]:
            self.close()
            raise TimeoutError("RGB capture timed out; camera worker stopped")
        line = self.process.stdout.readline(1024 * 1024 + 1)
        if not line or len(line) > 1024 * 1024:
            raise RuntimeError("RGB worker returned an empty/oversized result")
        result = json.loads(line)
        if result.get("ok") is not True or set(result.get("observation", {}).get("cameras", {})) != set(CAMERA_ROLES):
            raise RuntimeError("RGB worker did not return all three views")
        return result["observation"]

    def close(self):
        process = self.process
        cleanup_failed = False
        try:
            if process is not None:
                if process.poll() is None:
                    try:
                        process.stdin.write('{"command":"close"}\n')
                        process.stdin.flush()
                        # At most three in-flight one-second frame waits plus
                        # video finalization; avoid killing a healthy recorder.
                        process.wait(timeout=8)
                    except (OSError, subprocess.TimeoutExpired):
                        for signum in (signal.SIGTERM, signal.SIGKILL):
                            try:
                                os.killpg(process.pid, signum)
                            except ProcessLookupError:
                                pass  # The child can exit between poll and signal.
                            try:
                                process.wait(timeout=2)
                                break
                            except subprocess.TimeoutExpired:
                                if signum == signal.SIGKILL:
                                    raise
                cleanup_failed = process.poll() not in (0, None)
        finally:
            if process is not None:
                for stream in (process.stdin, process.stdout):
                    if stream is not None:
                        try:
                            stream.close()
                        except OSError:
                            cleanup_failed = True
            self.process = None
            if self.stderr is not None:
                try:
                    self.stderr.close()
                except OSError:
                    cleanup_failed = True
                self.stderr = None
        if cleanup_failed:
            raise RuntimeError("RGB worker exited unsuccessfully; inspect local camera_worker.stderr")


class PersistentRGBCameras:
    """Lazy, read-only capture of all three streams for the whole session."""
    nonphysical = False

    def __init__(self, config, output_dir):
        if set(config["cameras"]) != set(CAMERA_ROLES):
            raise ValueError("All three explicit camera bindings are required")
        self.output_dir = Path(output_dir)
        if config["backend"] == "realsense":
            from .camera import RealSenseRig
            self.rig = RealSenseRig(config["cameras"], rgb_only=True)
        elif config["backend"] == "ros1":
            from .ros_camera import RosCameraRig
            self.rig = RosCameraRig(config["cameras"], rgb_only=True,
                                    max_age_s=config["observation"]["max_age_s"],
                                    max_skew_s=config["observation"]["max_skew_s"])
        else:
            raise ValueError("A real RGB backend must be selected explicitly")

    def capture(self):
        return self.rig.capture(self.output_dir)

    def close(self):
        self.rig.close()


class HistoricalRGBSource:
    """Archive images, never a simulated visual consequence of a mock move.

    Each capture copies all original RGB files to the new run. Reusing a source
    snapshot is disclosed. Original observations and times are retained locally;
    no source intrinsics/detections/target positions reach the model packet.
    """
    nonphysical = True

    def __init__(self, observation_paths, output_dir):
        self.sources = [Path(p).resolve() for p in observation_paths]
        if not self.sources:
            raise ValueError("At least one historical observation is required")
        self.output_dir = Path(output_dir)
        self.index = 0

    def capture(self):
        source = self.sources[min(self.index, len(self.sources) - 1)]
        original = json.loads(source.read_text(encoding="utf-8"))
        frames = original.get("cameras", {})
        if "cameras" in frames:
            frames = frames["cameras"]
        frames = { {"left_wrist": "left_hand", "right_wrist": "right_hand"}.get(k, k): v
                   for k, v in frames.items() if isinstance(v, dict)}
        if set(frames) != set(CAMERA_ROLES):
            raise ValueError("Replay requires three actually recorded RGB views; missing views are not fabricated")
        folder = self.output_dir / ("replay_%04d" % self.index)
        folder.mkdir(parents=True, exist_ok=False)
        self.index += 1
        observation = {"nonphysical": True, "historical": True,
                       "capture_id": folder.name, "captured_at": time.time(),
                       "timestamp_kind": "replay_read_time_not_camera_exposure",
                       "source_observation": str(source), "source_reused": self.index > len(self.sources),
                       "robot_state_source": "mock_no_dynamics",
                       "cameras": {}}
        for role in CAMERA_ROLES:
            frame = frames[role]
            path = Path(frame["rgb_path"])
            if not path.is_absolute():
                path = source.parent / path
            target = folder / (role + path.suffix.lower())
            shutil.copyfile(str(path), str(target))
            observation["cameras"][role] = {
                "rgb_path": str(target), "timestamp": observation["captured_at"],
                "historical": True, "source_rgb_path": str(path),
                "source_timestamp": frame.get("timestamp", frame.get("host_receive_unix_s"))}
        (folder / "source_observation.json").write_text(json.dumps(original, ensure_ascii=False, indent=2), encoding="utf-8")
        (folder / "observation.json").write_text(json.dumps(observation, indent=2), encoding="utf-8")
        return observation

    def close(self):
        pass
