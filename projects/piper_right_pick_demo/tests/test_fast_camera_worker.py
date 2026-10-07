"""Camera process/worker lifecycle tests using fakes only, never RealSense IO."""
import io
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, call, patch

from right_pick import fast_camera_worker, fast_observation
from right_pick.camera import RealSenseRig


class FastCameraWorkerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.python = self.root / "fake python"
        self.python.write_text("Never executed by these tests.\n")
        self.bindings = {name: {"serial": str(100 + i), "width": 640, "height": 480, "fps": 15}
                         for i, name in enumerate(fast_observation.CAMERA_ROLES)}
        self.config = {"backend": "realsense", "camera_python": str(self.python), "cameras": self.bindings}
        self.observation = {"capture_id": "original-frame", "cameras": {
            name: {"rgb_path": "/test/" + name + ".png", "host_received_at": 123.}
            for name in fast_observation.CAMERA_ROLES}}
        for target in ("socket.socket", "subprocess.Popen", "right_pick.camera._dependency"):
            guard = patch(target, side_effect=AssertionError("Hardware/process forbidden in this test"))
            guard.start()
            self.addCleanup(guard.stop)

    def cameras(self):
        obj = fast_observation.SubprocessRGBCameras(self.config, self.root / "observations")
        self.addCleanup(obj.close)
        return obj

    def process(self, result=None):
        process = Mock(pid=765431, stdin=Mock(), stdout=Mock())
        process.poll.return_value = None
        process.wait.return_value = 0
        process.stdout.readline.return_value = json.dumps(
            {"ok": True, "observation": result or self.observation}) + "\n"
        return process

    def worker(self, input_text, rig=None, bindings=None):
        config_path = self.root / "bindings.json"
        config_path.write_text(json.dumps(self.bindings if bindings is None else bindings))
        rig = rig or Mock(snapshot=Mock(return_value=self.observation), close_errors=[])
        output = io.StringIO()
        with patch.object(fast_camera_worker, "ContinuousRGBRecorder", return_value=rig) as factory, \
                patch.object(fast_camera_worker.sys, "argv", ["worker", "--config", str(config_path),
                                                              "--output", str(self.root / "out")]), \
                patch.object(fast_camera_worker.sys, "stdin", io.StringIO(input_text)), \
                patch.object(fast_camera_worker.sys, "stdout", output):
            result = fast_camera_worker.main()
        return result, output.getvalue(), factory, rig

    def test_constructor_requires_real_backend_all_three_and_python(self):
        for changed in ({"backend": "mock"}, {"camera_python": "/missing/python"},
                        {"cameras": {"front": {"serial": "1"}}},
                        {"cameras": dict(self.bindings, invented={"serial": "4"})}):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                fast_observation.SubprocessRGBCameras(dict(self.config, **changed), self.root)
        cameras = self.cameras()
        self.assertFalse(cameras.nonphysical)
        self.assertIsNone(cameras.process)

    def test_one_persistent_argv_process_and_original_three_view_packet(self):
        process = self.process()
        cameras = self.cameras()
        with patch.object(fast_observation.subprocess, "Popen", return_value=process) as spawn, \
                patch.object(fast_observation.select, "select", return_value=([process.stdout], [], [])):
            self.assertEqual(cameras.capture(), self.observation)
            self.assertEqual(cameras.capture(), self.observation)
        self.assertEqual(spawn.call_count, 1)
        args, kwargs = spawn.call_args
        self.assertEqual(args[0][:4], [str(self.python), "-u", "-m", "right_pick.fast_camera_worker"])
        self.assertTrue(kwargs["start_new_session"])
        self.assertFalse(kwargs.get("shell", False))
        self.assertEqual(json.loads((cameras.output_dir / "camera_bindings.json").read_text()), self.bindings)
        self.assertEqual(process.stdin.write.call_args_list,
                         [call('{"command":"capture"}\n'), call('{"command":"capture"}\n')])
        stderr = cameras.stderr
        cameras.close()
        self.assertTrue(stderr.closed)
        process.stdin.close.assert_called_once()
        process.stdout.close.assert_called_once()
        self.assertIsNone(cameras.process)

    def test_missing_extra_empty_or_bad_camera_response_is_rejected(self):
        for raw in ("", "not JSON", json.dumps({"ok": False}),
                    json.dumps({"ok": True, "observation": {"cameras": {"front": {}}}}),
                    json.dumps({"ok": True, "observation": {"cameras": dict(self.bindings, invented={})}}),
                    "x" * (1024 * 1024 + 1)):
            with self.subTest(length=len(raw)):
                cameras = self.cameras()
                process = self.process()
                process.stdout.readline.return_value = raw
                cameras.process = process
                with patch.object(fast_observation.select, "select", return_value=([process.stdout], [], [])):
                    with self.assertRaises((RuntimeError, ValueError)):
                        cameras.capture()
                cameras.close()

    def test_dead_worker_does_not_restart_or_fabricate_capture(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        process.poll.return_value = 4
        with self.assertRaisesRegex(RuntimeError, "no automatic restart"):
            cameras.capture()
        process.stdin.write.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "exited unsuccessfully"):
            cameras.close()
        self.assertIsNone(cameras.process)

    def test_capture_timeout_closes_only_camera_worker(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        with patch.object(fast_observation.select, "select", return_value=([], [], [])), \
                patch.object(fast_observation.os, "killpg") as kill:
            with self.assertRaisesRegex(TimeoutError, "camera worker stopped"):
                cameras.capture()
        self.assertIsNone(cameras.process)
        process.wait.assert_called_once_with(timeout=8)
        kill.assert_not_called()

    def test_stuck_worker_gets_own_group_term_then_kill_and_pipes_close(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        process.wait.side_effect = [subprocess.TimeoutExpired("worker", 3),
                                    subprocess.TimeoutExpired("worker", 2), 0]
        with patch.object(fast_observation.os, "killpg") as kill:
            cameras.close()
        self.assertEqual(kill.call_args_list, [call(process.pid, signal.SIGTERM), call(process.pid, signal.SIGKILL)])
        process.stdin.close.assert_called_once()
        process.stdout.close.assert_called_once()
        cameras.close()  # Idempotent: cannot signal any process after ownership ends.

    def test_exit_between_poll_and_signal_still_reaps_and_releases_every_stream(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        process.poll.side_effect = [None, 0]
        process.wait.side_effect = [subprocess.TimeoutExpired("worker", 3), 0]
        stderr = cameras.stderr = io.StringIO()
        with patch.object(fast_observation.os, "killpg", side_effect=ProcessLookupError) as kill:
            cameras.close()
        kill.assert_called_once_with(process.pid, signal.SIGTERM)
        self.assertEqual(process.wait.call_args_list, [call(timeout=8), call(timeout=2)])
        process.stdin.close.assert_called_once()
        process.stdout.close.assert_called_once()
        self.assertTrue(stderr.closed)
        self.assertIsNone(cameras.process)
        self.assertIsNone(cameras.stderr)

    def test_nonzero_exit_after_close_is_reported_after_resources_are_released(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        process.poll.side_effect = [None, 7]
        process.wait.return_value = 7
        stderr = cameras.stderr = io.StringIO()
        with patch.object(fast_observation.os, "killpg") as kill:
            with self.assertRaisesRegex(RuntimeError, "exited unsuccessfully"):
                cameras.close()
        kill.assert_not_called()
        process.stdin.close.assert_called_once()
        process.stdout.close.assert_called_once()
        self.assertTrue(stderr.closed)
        self.assertIsNone(cameras.process)

    def test_pipe_close_failure_still_closes_other_streams_and_clears_ownership(self):
        cameras = self.cameras()
        process = cameras.process = self.process()
        process.poll.return_value = 0
        process.stdin.close.side_effect = BrokenPipeError("fake pipe flush failure")
        stderr = cameras.stderr = io.StringIO()
        with self.assertRaises((RuntimeError, OSError)):
            cameras.close()
        process.stdout.close.assert_called_once()
        self.assertTrue(stderr.closed)
        self.assertIsNone(cameras.process)
        self.assertIsNone(cameras.stderr)

    def test_spawn_failure_can_be_closed_without_leaking_stderr(self):
        cameras = self.cameras()
        with patch.object(fast_observation.subprocess, "Popen", side_effect=OSError("fake spawn failure")):
            with self.assertRaises(OSError):
                cameras.capture()
        stderr = cameras.stderr
        cameras.close()
        self.assertTrue(stderr.closed)

    def test_worker_capture_reuses_rig_rgb_only_and_close_releases(self):
        result, output, factory, rig = self.worker('{"command":"capture"}\n{"command":"capture"}\n{"command":"close"}\n')
        self.assertEqual(result, 0)
        factory.assert_called_once_with(self.bindings, str(self.root / "out"))
        self.assertEqual(rig.snapshot.call_count, 2)
        self.assertGreaterEqual(rig.next_frames.call_count, 2)
        self.assertEqual([json.loads(line)["observation"] for line in output.splitlines()],
                         [self.observation, self.observation])
        rig.close.assert_called_once_with(failure=None)

    def test_worker_eof_bad_command_and_capture_failure_always_close(self):
        _, output, _, rig = self.worker("")
        self.assertEqual(output, "")
        rig.close.assert_called_once()
        for text in ('{"command":"move"}\n', "malformed\n"):
            rig = Mock(close_errors=[])
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.worker(text, rig)
            rig.snapshot.assert_not_called()
            rig.close.assert_called_once()
        rig = Mock(close_errors=[])
        rig.next_frames.side_effect = RuntimeError("fake device failure")
        with self.assertRaisesRegex(RuntimeError, "fake device failure"):
            self.worker('{"command":"capture"}\n', rig)
        rig.close.assert_called_once()

    def test_worker_requires_exact_three_roles_before_rig_creation(self):
        with self.assertRaisesRegex(ValueError, "All three"):
            self.worker("", bindings={"front": {"serial": "1"}})
        # Existing camera constructor is lazy; these checks cannot load SDK/USB.
        duplicate = {name: {"serial": "same"} for name in self.bindings}
        with self.assertRaisesRegex(ValueError, "unique"):
            RealSenseRig(duplicate, rgb_only=True)

    def test_worker_close_errors_prevent_success_even_after_capture_or_eof(self):
        for commands in ("", '{"command":"close"}\n', '{"command":"capture"}\n'):
            rig = Mock(snapshot=Mock(return_value=self.observation),
                       close_errors=[{"operation": "pipeline_stop", "error": "fake stop failure"}])
            with self.subTest(commands=commands), self.assertRaisesRegex(RuntimeError, "shutdown failed"):
                self.worker(commands, rig)
            rig.close.assert_called_once()

    def test_acquisition_continues_while_no_snapshot_is_requested(self):
        rig = Mock(snapshot=Mock(return_value=self.observation), close_errors=[])
        state = {}
        def commands(stream, requests, stopped):
            state["requests"] = requests
            requests.put({"command": "capture"})
        def acquire():
            if rig.next_frames.call_count == 7:
                state["requests"].put({"command": "close"})
            return {"fresh_cycle": rig.next_frames.call_count}
        rig.next_frames.side_effect = acquire
        with patch.object(fast_camera_worker, "_commands", side_effect=commands):
            _, output, _, _ = self.worker("", rig)
        self.assertEqual(rig.next_frames.call_count, 7)
        self.assertEqual(rig.snapshot.call_count, 1)
        self.assertEqual(len(output.splitlines()), 1)


class ContinuousRGBRecorderTests(unittest.TestCase):
    """Real PNG/AVI encode/decode with synthetic pixels and a fake USB SDK."""

    def setUp(self):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bindings = {name: {"serial": str(100 + i), "width": 64, "height": 48, "fps": 15}
                         for i, name in enumerate(fast_observation.CAMERA_ROLES)}
        self.rs = Mock()
        self.rs.stream.color = "color-only"
        self.rs.format.bgr8 = "bgr8"
        self.rs.camera_info.serial_number = "serial"
        self.rs.camera_info.name = "model"
        self.pipelines, self.configs = [], []
        for name, conf in self.bindings.items():
            pipeline, config = Mock(), Mock()
            device = pipeline.start.return_value.get_device.return_value
            device.get_info.side_effect = lambda key, serial=conf["serial"]: serial if key == "serial" else "fake-camera"
            pipeline.wait_for_frames.side_effect = self.frames()
            self.pipelines.append(pipeline)
            self.configs.append(config)
        self.rs.pipeline.side_effect = self.pipelines
        self.rs.config.side_effect = self.configs
        dependency = patch.object(fast_camera_worker, "_dependency", side_effect={
            "cv2": self.cv2, "numpy": self.np, "pyrealsense2": self.rs}.__getitem__)
        dependency.start()
        self.addCleanup(dependency.stop)
        for target in ("socket.socket", "subprocess.Popen"):
            guard = patch(target, side_effect=AssertionError("Hardware/process forbidden"))
            guard.start()
            self.addCleanup(guard.stop)
        self.recorder = fast_camera_worker.ContinuousRGBRecorder(self.bindings, self.root)
        self.addCleanup(self.recorder.close)

    def frames(self):
        for number in range(1, 100):
            color = Mock()
            color.get_frame_number.return_value = number
            color.get_timestamp.return_value = number * 1000 / 15
            color.get_frame_timestamp_domain.return_value = "synthetic-device-clock"
            color.get_data.return_value = self.np.full((48, 64, 3), number * 2, dtype=self.np.uint8)
            frame = Mock()
            frame.get_color_frame.return_value = color
            yield frame

    def test_three_continuous_videos_share_frames_with_snapshots_and_keep_real_timing(self):
        self.rs.pipeline.assert_not_called()  # lazy construction never opens USB
        first = self.recorder.next_frames()
        observation = self.recorder.snapshot(first)
        for _ in range(5):
            self.recorder.next_frames()  # no snapshots/model requests during these cycles
        self.recorder.close()
        report = json.loads((self.root / "continuous_recording/report.json").read_text())
        self.assertEqual(report["status"], "closed")
        self.assertEqual(report["frames_recorded"], dict.fromkeys(self.bindings, 6))
        self.assertEqual(len(report["snapshots"]), 1)
        self.assertFalse(report["depth_enabled"])
        rows = [json.loads(line) for line in (self.root / "continuous_recording/frames.jsonl").read_text().splitlines()]
        self.assertEqual(len(rows), 18)
        for name, conf in self.bindings.items():
            frame = observation["cameras"][name]
            self.assertEqual(frame["video_frame_index"], 0)
            self.assertEqual(frame["frame_number"], 1)
            self.assertNotIn("intrinsics", frame)
            self.assertNotIn("red_candidates", frame)
            self.assertFalse(frame["exposure_age_verified"])
            self.assertTrue(self.np.array_equal(self.cv2.imread(frame["rgb_path"]), first[name]["pixels"]))
            decoder = self.cv2.VideoCapture(str(self.root / "continuous_recording" / (name + ".avi")))
            count = 0
            while decoder.read()[0]:
                count += 1
            decoder.release()
            self.assertEqual(count, 6)
        for config in self.configs:
            config.enable_stream.assert_called_once_with("color-only", 64, 48, "bgr8", 15)
        for pipeline in self.pipelines:
            pipeline.stop.assert_called_once()

    def test_repeated_frames_fail_instead_of_claiming_new_observation(self):
        self.recorder.next_frames()
        self.pipelines[0].wait_for_frames.side_effect = self.frames()  # resets device frame number
        with self.assertRaisesRegex(RuntimeError, "Repeated/non-monotonic") as error:
            self.recorder.next_frames()
        self.recorder.close(failure=error.exception)
        report = json.loads((self.root / "continuous_recording/report.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["frames_recorded"], dict.fromkeys(self.bindings, 1))

    def test_failed_partial_open_is_closed_and_recording_failure_is_explicit(self):
        self.pipelines[1].start.side_effect = RuntimeError("fake USB start failure")
        with self.assertRaisesRegex(RuntimeError, "fake USB") as error:
            self.recorder.next_frames()
        self.recorder.close(failure=error.exception)
        self.pipelines[0].stop.assert_called_once()
        self.pipelines[1].stop.assert_called_once()
        self.pipelines[2].stop.assert_not_called()
        report = json.loads((self.root / "continuous_recording/report.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertIn("fake USB start failure", report["failure"])

    def test_camera_stop_failure_releases_videos_and_is_reported(self):
        self.recorder.next_frames()
        self.pipelines[0].stop.side_effect = RuntimeError("fake stop error")
        self.recorder.close()
        report = json.loads((self.root / "continuous_recording/report.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["cleanup_errors"][0]["operation"], "pipeline_stop")
        self.assertTrue(self.recorder.frame_log.closed)


if __name__ == "__main__":
    unittest.main()
