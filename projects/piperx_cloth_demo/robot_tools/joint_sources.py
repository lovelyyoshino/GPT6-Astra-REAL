"""Host-owned, local-file sources for the bounded pair MOVE_J route.

This module never imports the SDK, queries a device, downloads files, or
estimates metric geometry from RGB. The explicit publisher installs complete
host-owned controller captures; it never creates geometry. The service supplies the
current host scene plus internal ``joint_source_bindings``; public proposals
must not supply those bindings or this provider's file paths.

The fixed per-run index names hashed capture/measurement records. Hashes bind
the bytes that were consumed, not their author or physical truth. Site records
must be established by a host-controlled measurement/install-record workflow;
a model's ``verified`` flag or metric guess is not such a workflow. This module
checks that provenance structure and exact current-scene/device scope, without
claiming to authenticate the original measurements or image contents.
"""
from __future__ import annotations

import copy
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
import re
import stat
import uuid
import time

from .joint_path import SDK_COMMIT, URDF_COMMIT, URDF_SHA256
from .model_compatibility import KNOWN_OFFICIAL_CONSTANTS, load_model_catalog


INDEX_SCHEMA = "piper_pair_joint_sources_index_v1"
LIMITS_SCHEMA = "piper_pair_controller_limits_capture_v1"
GEOMETRY_SCHEMA = "piper_pair_joint_geometry_source_v1"
SIDES = ("left", "right")
VIEWS = {"front": "front", "left_hand": "left_wrist", "right_hand": "right_wrist"}
RGB_MAX_AGE_S = 30.
MODEL_ARTIFACT = "artifacts/piper_x_capability_review_1791347348345247360"
SDK_URL = "https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/" + SDK_COMMIT + "/pyAgxArm/api/constants.py"
URDF_URL = "https://raw.githubusercontent.com/agilexrobotics/agx_arm_urdf/" + URDF_COMMIT + "/piper_x/urdf/piper_x_description.urdf"
BINDING_KEYS = {"connection_id", "model", "firmware_profile", "channel", "usb_interface"}


class JointSourcesError(ValueError):
    def __init__(self, code, detail):
        self.code, self.detail = code, detail
        super().__init__(code + ": " + detail)


def _need(condition, code, detail):
    if not condition:
        raise JointSourcesError(code, detail)


def _text(value, name):
    _need(type(value) is str and 0 < len(value) <= 1024 and value == value.strip()
          and all(ord(c) >= 32 and ord(c) != 127 for c in value), "invalid_source_schema", name)
    return value


def _num(value, name, *, positive=False):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    _need(valid and (not positive or value > 0), "invalid_source_number", name)
    return float(value)


def _hash(value):
    _need(type(value) is str and re.fullmatch("[0-9a-f]{64}", value) is not None,
          "invalid_source_hash", "Expected a lowercase SHA-256")
    return value


def _json(raw):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            _need(key not in result, "invalid_source_json", "Duplicate JSON key: " + key)
            result[key] = value
        return result
    try:
        return json.loads(raw, object_pairs_hook=unique, parse_constant=lambda value: (_ for _ in ()).throw(
            JointSourcesError("invalid_source_json", "Nonfinite JSON: " + value)))
    except (ValueError, UnicodeError) as exc:
        if isinstance(exc, JointSourcesError):
            raise
        raise JointSourcesError("invalid_source_json", str(exc)) from exc


def profile_sha256(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _validate_bindings(profile, bindings):
    _need(type(bindings) is dict and set(bindings) == set(SIDES), "device_binding_missing",
          "Host must supply both live joint connection identities, never public tool arguments")
    _need(type(profile.get("arms")) is dict, "device_profile_mismatch", "Both profile arms required")
    for side in SIDES:
        binding, arm_profile = bindings[side], profile.get("arms", {}).get(side, {})
        _need(type(arm_profile) is dict, "device_profile_mismatch", side)
        _need(type(binding) is dict and set(binding) == BINDING_KEYS, "device_binding_invalid", side)
        for key in BINDING_KEYS:
            _text(binding[key], side+"/"+key)
        _need(binding["model"] == arm_profile.get("model") == "piper_x"
              and binding["firmware_profile"] == arm_profile.get("firmware") == "default"
              and all(binding[key] == arm_profile.get(key) for key in ("channel", "usb_interface")),
              "device_profile_mismatch", side)


def _validated_limits(data, *, run_id, owner, bindings, now):
    """Validate raw receipts identically before publication and on consumption."""
    _need(type(data) is dict and data.get("schema") == LIMITS_SCHEMA
          and data.get("run_id") == run_id and data.get("owner") == owner and data.get("bindings") == bindings,
          "controller_limits_scope_mismatch", "Limit receipt must bind the current owner and both SDK connections")
    _need(data.get("operation") == "inspect_joint_limits" and data.get("status") == "joint_limits_received_no_motion_commands"
          and type(data.get("joint_limit_queries_sent")) is int and data["joint_limit_queries_sent"] == 12
          and type(data.get("actuator_commands_sent")) is int and data["actuator_commands_sent"] == 0
          and data.get("controller_limits_changed") is False and data.get("sdk_joint_limits_changed") is False
          and data.get("guard_violations") == [] and data.get("errors") == [],
          "controller_limits_capture_failed", "Twelve complete read queries without actuator/configuration sends required")
    _need(("ok" not in data or data["ok"] is True)
          and ("fault_latched" not in data or data["fault_latched"] is False)
          and ("hardware_commands_sent" not in data or
               type(data["hardware_commands_sent"]) is int and data["hardware_commands_sent"] == 12),
          "controller_limits_capture_failed", "A failed or contradictory device report cannot install sources")
    began, ended, now = _num(data.get("began_at"), "began_at"), _num(data.get("ended_at"), "ended_at"), _num(now, "clock")
    _need(0 <= began <= ended <= now, "controller_limits_time_invalid", "Capture time must be complete and nonfuture")
    joints, queries = data.get("joint_limits"), data.get("query_receipts")
    _need(type(joints) is dict and set(joints) == set(SIDES) and type(queries) is dict and set(queries) == set(SIDES),
          "controller_limits_incomplete", "Both arms' individual joint receipts required")
    result = {}
    for side in SIDES:
        _need(type(joints[side]) is dict and set(joints[side]) == set(map(str, range(1, 7)))
              and type(queries[side]) is dict and set(queries[side]) == set(joints[side]),
              "controller_limits_incomplete", side)
        result[side] = []
        previous_finished = began
        for joint in range(1, 7):
            row, query = joints[side][str(joint)], queries[side][str(joint)]
            _need(type(row) is dict and row.get("status") == "confirmed" and type(query) is dict,
                  "controller_limit_reply_invalid", side+"/"+str(joint))
            _need(set(query) == {"arbitration_id", "data_hex", "outcome", "sent_at", "returned_at"}
                  and type(query["arbitration_id"]) is int and query["arbitration_id"] == 0x472
                  and query["data_hex"] == bytes((joint, 1, 0, 0, 0, 0, 0, 0)).hex()
                  and query["outcome"] == "returned", "controller_limit_query_invalid", side+"/"+str(joint))
            window = row.get("response_evidence")
            _need(type(window) is dict and window.get("active") is False
                  and window.get("rejected_frames") == [] and type(window.get("ignored_stale_frames")) is list
                  and type(window.get("response_frames")) is list and len(window["response_frames"]) == 1,
                  "controller_limit_reply_invalid", "Exactly one correlated response per joint required")
            started, finished = _num(window.get("request_started_unix_s"), "request start"), _num(window.get("finished_unix_s"), "window end")
            sent, returned = _num(query["sent_at"], "query sent"), _num(query["returned_at"], "query returned")
            frame = window["response_frames"][0]
            _need(type(frame) is dict and type(frame.get("dlc")) is int and frame["dlc"] == 8,
                  "controller_limit_reply_invalid", "One eight-byte response required")
            stamp, received = _num(frame.get("timestamp"), "frame stamp"), _num(frame.get("received_unix_s"), "received time")
            _need(previous_finished <= started <= sent <= returned <= finished <= ended and started < finished
                  and sent <= stamp <= received <= finished,
                  "controller_limit_window_invalid", "Each response must follow its own query within its own closed window")
            previous_finished = finished
            raw_hex = frame.get("payload_hex")
            _need(type(raw_hex) is str and re.fullmatch("[0-9a-f]{16}", raw_hex) is not None
                  and row.get("raw_response_hex") == raw_hex, "controller_limit_reply_invalid", "Original reply bytes required")
            raw = bytes.fromhex(raw_hex)
            _need(raw[0] == joint, "controller_limit_joint_mismatch", side+"/"+str(joint))
            _need(raw[7] == 0, "controller_limit_reply_invalid", "Reserved reply byte must be zero")
            low, high = int.from_bytes(raw[3:5], "big", signed=True), int.from_bytes(raw[1:3], "big", signed=True)
            _need(low < high, "controller_limit_range_invalid", side+"/"+str(joint))
            result[side].append([low*.1*math.pi/180, high*.1*math.pi/180])
    return result


def _canonical_bytes(value):
    try:
        raw = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    except (TypeError, ValueError, OverflowError) as exc:
        raise JointSourcesError("invalid_source_json", "Expected finite JSON source data") from exc
    _need(len(raw) <= 4*1024*1024, "source_file_invalid", "Source exceeds 4 MiB")
    return raw


def _open_source_directory(path):
    """Pin each directory component without following injected symlinks."""
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in path.parts[1:]:
            try:
                os.mkdir(part, 0o700, dir_fd=descriptor)
                os.fsync(descriptor)
            except FileExistsError:
                pass
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        info = os.fstat(descriptor)
        _need(info.st_uid == os.geteuid() and not info.st_mode & 0o022,
              "source_directory_unsafe", "Host source directory must be owned and not group/world writable")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _read_at(directory_fd, name):
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
    try:
        info = os.fstat(fd)
        _need(stat.S_ISREG(info.st_mode) and info.st_size <= 4*1024*1024,
              "source_file_invalid", "Expected a bounded regular source file")
        chunks, size = [], 0
        while True:
            chunk = os.read(fd, 65536)
            if not chunk:
                break
            size += len(chunk)
            _need(size <= 4*1024*1024, "source_file_invalid", "Source grew during read")
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(fd)


def _existing_reference(directory_fd, directory, reference):
    _need(type(reference) is dict and set(reference) == {"path", "sha256"},
          "invalid_source_reference", "Expected an exact path/hash reference")
    path = Path(_text(reference["path"], "source path"))
    if path.is_absolute():
        _need(path.is_relative_to(directory), "source_path_escape", "Capture must stay inside this source directory")
        path = path.relative_to(directory)
    _need(path.parts and not any(p in ("..", ".") for p in path.parts),
          "source_path_escape", "Capture path must stay inside this source directory")
    fd = os.dup(directory_fd)
    try:
        for part in path.parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=fd)
            os.close(fd)
            fd = child
        raw = _read_at(fd, path.name)
    finally:
        os.close(fd)
    _need(hashlib.sha256(raw).hexdigest() == _hash(reference["sha256"]),
          "source_hash_mismatch", str(path))
    return _json(raw)


def _write_temp(directory_fd, raw):
    name = ".publish-" + uuid.uuid4().hex + ".tmp"
    fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=directory_fd)
    try:
        remaining = memoryview(raw)
        while remaining:
            count = os.write(fd, remaining)
            _need(count > 0, "source_write_failed", "No bytes written")
            remaining = remaining[count:]
        os.fsync(fd)
    except BaseException:
        os.unlink(name, dir_fd=directory_fd)
        raise
    finally:
        os.close(fd)
    return name


def _source_epoch_directory(runs_root, profile, run_id, owner, bindings):
    """Pure scope address; this neither connects nor creates a session/budget."""
    _need(type(run_id) is str and re.fullmatch(r"[A-Za-z0-9_-]{1,96}", run_id),
          "invalid_run_id", "Expected a host run identifier")
    _text(owner, "owner")
    _validate_bindings(profile, bindings)
    scope = {"run_id":run_id, "owner":owner, "profile_sha256":profile_sha256(profile),
             "bindings":bindings}
    digest = hashlib.sha256(_canonical_bytes(scope)).hexdigest()
    return Path(os.path.abspath(runs_root))/("pair_"+run_id)/"joint_sources"/"epochs"/digest


def publish_controller_limits(runs_root, profile, run_id, owner, bindings, capture, *, clock=time.time):
    """Publish a complete internal device capture, without geometry or motion.

    The host supplies run/owner/live bindings and the actual device report. This
    is not a public tool accepting a model-provided capture. Source hashes are
    integrity references, not authentication of physical measurements. A failed
    write never retries device queries; an orphan complete capture can remain
    after index publication fails. Readers see either complete index version.
    """
    _need(type(run_id) is str and re.fullmatch(r"[A-Za-z0-9_-]{1,96}", run_id) is not None,
          "invalid_run_id", "Expected a host run identifier")
    _text(owner, "owner")
    _need(type(profile) is dict and callable(clock), "invalid_provider_config", "Profile and clock required")
    profile, bindings = _json(_canonical_bytes(profile)), _json(_canonical_bytes(bindings))
    _validate_bindings(profile, bindings)
    raw = _canonical_bytes(capture)
    capture = _json(raw)
    now = _num(clock(), "clock")
    limits = _validated_limits(capture, run_id=run_id, owner=owner, bindings=bindings, now=now)
    digest = hashlib.sha256(raw).hexdigest()
    reference = {"path": "controller_limits_" + digest + ".json", "sha256": digest}
    directory = _source_epoch_directory(runs_root, profile, run_id, owner, bindings)
    fd = None
    try:
        fd = _open_source_directory(directory)
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            index = _json(_read_at(fd, "index.json"))
        except FileNotFoundError:
            index = {"schema": INDEX_SCHEMA, "run_id": run_id, "owner": owner,
                     "profile_sha256": profile_sha256(profile), "controller_limits": None, "geometry": {}}
        _need(type(index) is dict and set(index) == {"schema", "run_id", "owner", "profile_sha256", "controller_limits", "geometry"}
              and index["schema"] == INDEX_SCHEMA and type(index["geometry"]) is dict,
              "source_index_schema", "Exact host source index required")
        _need(index["run_id"] == run_id and index["owner"] == owner and index["profile_sha256"] == profile_sha256(profile),
              "source_index_scope_mismatch", "Cannot replace another run/owner/profile's sources")
        previous = index["controller_limits"]
        if previous is not None:
            old = _existing_reference(fd, directory, previous)
            _validated_limits(old, run_id=run_id, owner=owner, bindings=bindings, now=now)
            replayed = _canonical_bytes(old) == raw
        else:
            replayed = False
        if replayed:
            reference = previous
        else:
            temporary = _write_temp(fd, raw)
            try:
                try:
                    os.link(temporary, reference["path"], src_dir_fd=fd, dst_dir_fd=fd, follow_symlinks=False)
                except FileExistsError:
                    _need(_read_at(fd, reference["path"]) == raw, "source_hash_mismatch", "Content-addressed capture already differs")
            finally:
                os.unlink(temporary, dir_fd=fd)
            os.fsync(fd)
            index["controller_limits"] = reference
            temporary = _write_temp(fd, _canonical_bytes(index))
            try:
                os.replace(temporary, "index.json", src_dir_fd=fd, dst_dir_fd=fd)
                os.fsync(fd)
            finally:
                try:
                    os.unlink(temporary, dir_fd=fd)
                except FileNotFoundError:
                    pass
        source_path = Path(reference["path"])
        source_path = source_path if source_path.is_absolute() else directory/source_path
        source = {"ref": str(source_path), "sha256": reference["sha256"]}
        return {"index_path": str(directory/"index.json"), "source": copy.deepcopy(reference),
                "controller_limits": {"source": source, **limits}, "replayed": replayed,
                "source_truth_authenticated": False, "hardware_commands_sent": 0}
    except OSError as exc:
        raise JointSourcesError("source_publication_failed", "Local source publication failed: " + str(exc)) from exc
    finally:
        if fd is not None:
            os.close(fd)  # Releases the directory flock as well.


class JointSourcesProvider:
    """Callable ``(scene, arm) ->`` four sources consumed by PairHost.

    New index: runs_root/pair_<run_id>/joint_sources/epochs/<scope_hash>/index.json,
    containing schema,
    run_id, owner, profile_sha256, controller_limits:{path,sha256}|null, and
    geometry:{observation_id:{path,sha256}}. Paths in this index/site records
    must stay inside that same source directory after resolving symlinks.

    The source reader is an internal host boundary, not a tool accepting model
    dictionaries. ``diagnose`` reports precise gaps and never grants motion.
    A legacy root index is only a read fallback for its exact original scope.
    New publications never migrate it or relabel any historical device capture.
    """

    def __init__(self, bundle_root, profile, run_id, *, runs_root, clock=time.time):
        _need(type(run_id) is str and re.fullmatch(r"[A-Za-z0-9_-]{1,96}", run_id) is not None,
              "invalid_run_id", "Expected a host run identifier")
        _need(type(profile) is dict and callable(clock), "invalid_provider_config", "Profile and clock required")
        self.root, self.runs = Path(bundle_root).resolve(), Path(runs_root).resolve()
        self.profile, self.run_id, self.clock = copy.deepcopy(profile), run_id, clock
        self.profile_hash = profile_sha256(self.profile)
        self.directory = self.runs / ("pair_" + run_id) / "joint_sources"
        self.base_directory = self.directory
        self._source_root = self.directory.resolve()
        _need(self.runs.is_relative_to(self.root) and self._source_root.is_relative_to(self.runs),
              "source_path_escape", "Run sources must remain under the host workspace")
        self.index_path = self.directory / "index.json"

    def _scoped(self, owner, bindings, *, for_write=False):
        """Return a per-call view, never mutate a shared provider's directory."""
        directory = _source_epoch_directory(self.runs, self.profile, self.run_id, owner, bindings)
        scoped = copy.copy(self)
        scoped.directory, scoped._source_root = directory, directory
        scoped.index_path = directory/"index.json"
        # Once any epoch directory exists, incomplete/corrupt content must not
        # fall back to historical records and conceal a failed publication.
        if for_write or os.path.lexists(directory):
            return scoped
        legacy = copy.copy(self)
        legacy.directory, legacy._source_root = self.base_directory, self.base_directory
        legacy.index_path = self.base_directory/"index.json"
        if not os.path.lexists(legacy.index_path):
            return scoped
        # Read, never rewrite, the old index. Known different scope means this
        # new epoch is empty; malformed current-scope data remains a hard gap.
        _need(legacy.index_path.resolve().is_relative_to(legacy._source_root),
              "source_path_escape", "Legacy index escaped its source directory")
        index = _json(legacy._bytes(legacy.index_path))
        _need(type(index) is dict and set(index) == {"schema","run_id","owner","profile_sha256","controller_limits","geometry"}
              and index["schema"] == INDEX_SCHEMA and type(index["geometry"]) is dict,
              "source_index_schema", "Exact legacy source index required")
        if (index["run_id"] != self.run_id or index["owner"] != owner
                or index["profile_sha256"] != self.profile_hash):
            return scoped
        references = ([index["controller_limits"]] if index["controller_limits"] is not None else [])
        references += list(index["geometry"].values())
        if not references:
            return scoped  # An empty old index has no connection identity.
        for reference in references:
            data, _ = legacy._ref(reference)
            if (type(data) is not dict or data.get("run_id") != self.run_id or data.get("owner") != owner
                    or _canonical_bytes(data.get("bindings")) != _canonical_bytes(bindings)):
                return scoped
        return legacy

    @staticmethod
    def _bytes(path, *, maximum=4*1024*1024):
        try:
            _need(path.is_file() and path.stat().st_size <= maximum, "source_file_invalid",
                  "Expected bounded regular file: " + str(path))
            raw = path.read_bytes()
            _need(len(raw) <= maximum, "source_file_invalid", "Source grew while reading: " + str(path))
            return raw
        except OSError as exc:
            raise JointSourcesError("source_file_unavailable", str(path)) from exc

    def _ref(self, reference, *, decode=True):
        _need(type(reference) is dict and set(reference) == {"path", "sha256"},
              "invalid_source_reference", "Expected an exact path/hash reference")
        name, expected = _text(reference["path"], "source path"), _hash(reference["sha256"])
        path = Path(name)
        path = (path if path.is_absolute() else self.directory/path).resolve()
        _need(path.is_relative_to(self._source_root), "source_path_escape", "Source must remain in this run's source directory")
        raw = self._bytes(path)
        _need(hashlib.sha256(raw).hexdigest() == expected, "source_hash_mismatch", str(path))
        return (_json(raw) if decode else raw), {"ref": str(path), "sha256": expected}

    def official_sources(self):
        """Resolve previously pinned local bytes; no network or SDK execution."""
        _need(self.profile.get("sdk_commit_audited") == SDK_COMMIT, "official_sdk_commit_mismatch",
              "Current profile must retain the reviewed SDK commit")
        cache = self.root / "artifacts/joint_sources/official"
        archive = self.root / MODEL_ARTIFACT
        stable = self.root / "projects/piperx_cloth_demo/data/piper_x_official"
        sdk_candidates = [stable / "sdk_constants.py", cache / "sdk_constants.py", archive / "official_sdk_constants.py"]
        sdk_path = self.profile.get("sdk_path")
        if type(sdk_path) is str and sdk_path:
            sdk_candidates.append(Path(sdk_path)/"pyAgxArm/api/constants.py")
        def choose(candidates, expected, kind):
            for candidate in candidates:
                if candidate.is_file():
                    raw = self._bytes(candidate, maximum=1024*1024)
                    if hashlib.sha256(raw).hexdigest() == expected:
                        return str(candidate.resolve())
            raise JointSourcesError("official_"+kind+"_cache_missing",
                                    "No local copy matches the pinned manufacturer bytes")
        constants = choose(sdk_candidates, KNOWN_OFFICIAL_CONSTANTS[SDK_COMMIT], "sdk")
        catalog = {"constants_path": constants, "commit": SDK_COMMIT,
                   "sha256": KNOWN_OFFICIAL_CONSTANTS[SDK_COMMIT], "source_url": SDK_URL}
        # Existing literal-only reader also checks the official model shape.
        load_model_catalog(catalog)
        urdf = choose([stable/"piper_x_description.urdf", cache/"piper_x_description.urdf", archive/"official_urdf/piper_x/urdf/piper_x_description.urdf"],
                      URDF_SHA256, "urdf")
        return {"model_catalog": catalog, "urdf_source": {"path": urdf, "commit": URDF_COMMIT,
                                                           "source_url": URDF_URL, "sha256": URDF_SHA256}}

    def _scene(self, scene, arm):
        _need(arm in SIDES and type(scene) is dict, "current_scene_required", "Current host scene and selected arm required")
        observation, capture = _text(scene.get("observation_id"), "observation_id"), _text(scene.get("capture_id"), "capture_id")
        now, stamp = _num(self.clock(), "clock"), _num(scene.get("rgb_received_at"), "rgb_received_at")
        _need(0 <= stamp <= now and now-stamp <= RGB_MAX_AGE_S, "current_scene_expired", "RGB source is not current")
        peers = scene.get("peer_receipts")
        _need(type(peers) is dict and set(peers) == set(SIDES), "scene_owner_missing", "Both host-issued peer receipts required")
        owner = None
        for side in SIDES:
            peer = peers[side]
            _need(type(peer) is dict and peer.get("observation_id") == observation and peer.get("arm") == side,
                  "scene_owner_mismatch", "Peer receipt belongs to another scene or arm")
            observed_owner = _text(peer.get("owner"), "peer owner")
            owner = observed_owner if owner is None else owner
            _need(owner == observed_owner, "scene_owner_mismatch", "One current owner required")
        bindings = scene.get("joint_source_bindings")
        _validate_bindings(self.profile, bindings)
        saved = scene.get("saved_rgb_evidence")
        _need(type(saved) is dict and set(saved) == set(VIEWS), "saved_rgb_required", "Three current saved RGB artifacts required")
        frames, stamps = {}, []
        for view, camera_key in VIEWS.items():
            _text(self.profile.get("cameras", {}).get(camera_key), "camera serial")
            item = saved[view]
            _need(type(item) is dict and set(item) == {"rgb_path", "artifact_sha256", "frame_number", "host_received_at"},
                  "saved_rgb_invalid", view)
            path = Path(_text(item["rgb_path"], "RGB path")).resolve()
            _need(path.is_relative_to(self.root) and path.suffix == ".png", "rgb_path_escape", view)
            raw = self._bytes(path, maximum=32*1024*1024)
            _need(raw.startswith(b"\x89PNG\r\n\x1a\n") and hashlib.sha256(raw).hexdigest() == _hash(item["artifact_sha256"]),
                  "rgb_artifact_changed", view)
            number, received = item["frame_number"], _num(item["host_received_at"], "RGB receive time")
            _need(type(number) is int and number >= 0 and 0 <= received <= now and now-received <= RGB_MAX_AGE_S,
                  "saved_rgb_stale", view)
            stamps.append(received)
            frames[view] = {key: item[key] for key in ("artifact_sha256", "frame_number", "host_received_at")}
        _need(max(stamps)-min(stamps) <= .15 and stamp == min(stamps), "saved_rgb_scene_mismatch",
              "Saved frame timestamps must match the current shared scene")
        return owner, copy.deepcopy(bindings), {"observation_id": observation, "capture_id": capture, "frames": frames}

    def _index(self, owner):
        _need(self.index_path.is_file(), "source_index_missing", "No host source index: " + str(self.index_path))
        _need(self.index_path.resolve().is_relative_to(self._source_root), "source_path_escape", "Index escaped source directory")
        index = _json(self._bytes(self.index_path))
        _need(type(index) is dict and set(index) == {"schema", "run_id", "owner", "profile_sha256", "controller_limits", "geometry"}
              and index["schema"] == INDEX_SCHEMA, "source_index_schema", "Exact host source index required")
        _need(index["run_id"] == self.run_id and index["owner"] == owner
              and index["profile_sha256"] == self.profile_hash, "source_index_scope_mismatch", "Index belongs to another run/owner/profile")
        _need(type(index["geometry"]) is dict, "source_index_schema", "Geometry is keyed by current observation ID")
        return index

    def _limits(self, reference, owner, bindings):
        _need(reference is not None, "controller_limits_capture_missing", "Current owner's per-joint raw query receipt required")
        data, source = self._ref(reference)
        return {"source": source, **_validated_limits(data, run_id=self.run_id, owner=owner,
                                                      bindings=bindings, now=self.clock())}

    def _geometry(self, reference, owner, bindings, scene_ref, validation_info=None):
        _need(reference is not None, "geometry_source_missing",
              "Current-scene attachment bounds, numeric corridor and model-base workspace sources required")
        data, source = self._ref(reference)
        _need(type(data) is dict and set(data) == {"schema", "run_id", "owner", "profile_sha256", "bindings", "scene", "bounds", "metric_sources", "workspace_frame"}
              and data["schema"] == GEOMETRY_SCHEMA, "geometry_source_schema", "Exact sourced geometry record required, not verified flags")
        _need(data["workspace_frame"] == "physical_model_arm_base", "workspace_source_invalid",
              "Workspace measurements must use the physical model arm-base frame")
        _need(data["run_id"] == self.run_id and data["owner"] == owner and data["profile_sha256"] == self.profile_hash
              and data["bindings"] == bindings and profile_sha256(data["scene"]) == profile_sha256(scene_ref),
              "geometry_scene_scope_mismatch", "Geometry must refer to this owner, device installation and exact current RGB artifacts")
        bounds, metrics = data["bounds"], data["metric_sources"]
        _need(type(bounds) is dict and set(bounds) == {"attachment_radius_m", "available_clearance_m", "workspace_min_m", "workspace_max_m"}
              and type(metrics) is dict and set(metrics) == {"left_attachment", "right_attachment", "clearance", "workspace"},
              "geometry_source_schema", "All independent metric source records required")
        metric_bytes = {}
        for name, entry in metrics.items():
            allowed = ("physical_measurement", "installed_cad_record") if name.endswith("attachment") else ("physical_measurement", "installed_workspace_record")
            _need(type(entry) is dict and set(entry) == {"kind", "path", "sha256"} and entry["kind"] in allowed,
                  "geometry_metric_source_invalid", name+": RGB guesses or model assertions cannot supply metric bounds")
            if name == "clearance":
                _need(entry["kind"] == "physical_measurement", "geometry_metric_source_invalid", "Clearance must refer to current physical measurement")
            metric_bytes[name] = self._ref({"path": entry["path"], "sha256": entry["sha256"]}, decode=False)[0]
        attachments = bounds["attachment_radius_m"]
        _need(type(attachments) is dict and set(attachments) == set(SIDES), "attachment_bounds_missing", "Both installed attachments required")
        for side in SIDES:
            _num(attachments[side], side+" attachment radius", positive=True)
        _num(bounds["available_clearance_m"], "clearance", positive=True)
        vectors = []
        for name in ("workspace_min_m", "workspace_max_m"):
            values = bounds[name]
            _need(type(values) is list and len(values) == 3, "workspace_source_invalid", "Workspace must be in physical model arm-base coordinates")
            vectors.append([_num(v, name) for v in values])
        _need(all(a < b for a, b in zip(*vectors)), "workspace_source_invalid", "Workspace limits must be ordered")
        from .site_geometry_records import validate_imported_geometry
        measured_until = validate_imported_geometry(data, metric_bytes, profile=self.profile, now=self.clock())
        if validation_info is not None and measured_until is not None:
            validation_info["valid_until_s"] = measured_until
        if measured_until is not None:
            _need(_num(self.clock(), "clock") < measured_until, "geometry_record_time_invalid",
                  "Geometry validation outlasted the measurement interval")
        # PairHost binds its subsequently acquired joint origin. A source
        # reader has no such sample and must not manufacture an observation ID.
        return {**copy.deepcopy(bounds), "origin_sample_id": None, "source": source}

    def publish_geometry(self, scene, record_set_id):
        """Import actual controlled records into the current scene with zero TX.

        Internal host API only: no supplied bounds, permissions or owner fields.
        The host must serialize against scene/owner changes around this call.
        Old-owner indexes are never migrated, and no ledger budget is modified.
        """
        scene = _json(_canonical_bytes(scene))
        owner, bindings, scene_ref = self._scene(scene, "left")
        return self._scoped(owner, bindings, for_write=True)._publish_geometry_in_scope(scene, record_set_id)

    def _publish_geometry_in_scope(self, scene, record_set_id):
        from .site_geometry_records import load_geometry_records
        started = _num(self.clock(), "clock")
        owner, bindings, scene_ref = self._scene(scene, "left")
        records = load_geometry_records(self.root, record_set_id, profile=self.profile,
            run_id=self.run_id, owner=owner, bindings=bindings, scene_ref=scene_ref, now=started)
        metrics, content = {}, {}
        for name, raw in records["raw_records"].items():
            digest = records["record_sha256"][name]
            filename = "geometry_record_" + name + "_" + digest + ".json"
            content[filename] = raw
            keys = ("left_attachment", "right_attachment") if name == "installation" else (name,)
            for key in keys:
                metrics[key] = {"kind":records["metric_kinds"][key], "path":filename, "sha256":digest}
        data = {"schema":GEOMETRY_SCHEMA, "run_id":self.run_id, "owner":owner,
                "profile_sha256":self.profile_hash, "bindings":bindings, "scene":scene_ref,
                "bounds":records["bounds"], "metric_sources":metrics,
                "workspace_frame":"physical_model_arm_base"}
        raw = _canonical_bytes(data)
        digest = hashlib.sha256(raw).hexdigest()
        reference = {"path":"geometry_"+digest+".json", "sha256":digest}
        content[reference["path"]] = raw
        fd = None
        try:
            fd = _open_source_directory(self.directory)
            fcntl.flock(fd, fcntl.LOCK_EX)
            try:
                index = _json(_read_at(fd, "index.json"))
            except FileNotFoundError:
                index = {"schema":INDEX_SCHEMA, "run_id":self.run_id, "owner":owner,
                         "profile_sha256":self.profile_hash, "controller_limits":None, "geometry":{}}
            _need(type(index) is dict and set(index) == {"schema","run_id","owner","profile_sha256","controller_limits","geometry"}
                  and index["schema"] == INDEX_SCHEMA and type(index["geometry"]) is dict,
                  "source_index_schema", "Exact host source index required")
            _need(index["run_id"] == self.run_id and index["owner"] == owner
                  and index["profile_sha256"] == self.profile_hash,
                  "source_index_scope_mismatch", "Cannot migrate or replace another owner/profile's sources")
            if index["controller_limits"] is not None:
                old_limits = _existing_reference(fd, self.directory, index["controller_limits"])
                _validated_limits(old_limits, run_id=self.run_id, owner=owner, bindings=bindings, now=self.clock())
            previous = index["geometry"].get(scene_ref["observation_id"])
            if previous is not None:
                old = _existing_reference(fd, self.directory, previous)
                _need(_canonical_bytes(old) == raw, "geometry_scene_already_published",
                      "Cannot overwrite a different geometry record for the same scene")
            replayed = previous is not None
            if not replayed:
                for filename, blob in content.items():
                    temporary = _write_temp(fd, blob)
                    try:
                        try:
                            os.link(temporary, filename, src_dir_fd=fd, dst_dir_fd=fd, follow_symlinks=False)
                        except FileExistsError:
                            _need(_read_at(fd, filename) == blob, "source_hash_mismatch", "Existing immutable geometry source differs")
                    finally:
                        os.unlink(temporary, dir_fd=fd)
                os.fsync(fd)
            installed_reference = previous if replayed else reference
            # Reuse the exact reader contract, including content-derived values
            # and fixed measurement expiry, after all record IO.
            geometry = self._geometry(installed_reference, owner, bindings, scene_ref)
            checked = _num(self.clock(), "clock")
            _need(started <= checked < records["valid_until_s"], "geometry_record_time_invalid",
                  "Source IO outlasted measurement validity or clock regressed")
            _need(self._scene(scene, "left") == (owner, bindings, scene_ref),
                  "geometry_scene_scope_mismatch", "Current saved scene changed during source import")
            if not replayed:
                index["geometry"][scene_ref["observation_id"]] = reference
                temporary = _write_temp(fd, _canonical_bytes(index))
                try:
                    os.replace(temporary, "index.json", src_dir_fd=fd, dst_dir_fd=fd)
                    os.fsync(fd)
                finally:
                    try:
                        os.unlink(temporary, dir_fd=fd)
                    except FileNotFoundError:
                        pass
            finished = _num(self.clock(), "clock")
            _need(started <= finished < records["valid_until_s"], "geometry_record_time_invalid",
                  "Index commit outlasted original measurement validity or clock regressed")
            _need(self._scene(scene, "left") == (owner, bindings, scene_ref),
                  "geometry_scene_scope_mismatch", "Saved scene changed during index commit")
            return {"index_path":str(self.index_path), "source":copy.deepcopy(installed_reference),
                    "geometry":geometry, "replayed":replayed, "source_truth_authenticated":False,
                    "measurement_valid_until_s":records["valid_until_s"],
                    "dispatch_authorized":False, "hardware_commands_sent":0}
        except OSError as exc:
            raise JointSourcesError("source_publication_failed", "Local geometry publication failed: "+str(exc)) from exc
        finally:
            if fd is not None:
                os.close(fd)

    def _resolve(self, scene, arm, *, require_geometry=True):
        result, gaps = {}, []
        geometry_info = {}
        index_path = self.index_path
        def collect(function, *args):
            try:
                return function(*args)
            except JointSourcesError as exc:
                gaps.append({"code": exc.code, "detail": exc.detail})
            except (ValueError, OSError, KeyError, TypeError, OverflowError) as exc:
                gaps.append({"code": "invalid_source_schema", "detail": str(exc)})
            return None
        started = collect(lambda: _num(self.clock(), "clock"))
        official = collect(self.official_sources)
        if official:
            result.update(official)
        bound = collect(self._scene, scene, arm)
        if bound:
            owner, bindings, scene_ref = bound
            scoped = collect(self._scoped, owner, bindings)
            if scoped is not None:
                index_path = scoped.index_path
            index = collect(scoped._index, owner) if scoped is not None else None
            if index:
                limits = collect(scoped._limits, index["controller_limits"], owner, bindings)
                geometry = (collect(scoped._geometry, index["geometry"].get(scene_ref["observation_id"]), owner, bindings, scene_ref, geometry_info)
                            if require_geometry else None)
                if limits:
                    result["controller_limits"] = limits
                if geometry:
                    result["geometry"] = geometry
            elif any(g["code"] == "source_index_missing" for g in gaps):
                gaps.append({"code": "controller_limits_capture_missing", "detail": "Capture twelve raw per-joint replies in this persistent owner"})
                if require_geometry:
                    gaps.append({"code": "geometry_source_missing", "detail": "No current attachment/corridor/model-base workspace sources; no metric values inferred from RGB or generic clearance wording"})
            if started is not None:
                def final_time():
                    now = _num(self.clock(), "clock")
                    _need(now >= started, "source_clock_regressed", "Source reading cannot refresh a regressed host clock")
                    _need(0 <= now-scene["rgb_received_at"] <= RGB_MAX_AGE_S, "current_scene_expired",
                          "Source IO outlasted the current RGB scene; acquire a new host scene")
                    _need(now < geometry_info.get("valid_until_s", math.inf), "geometry_record_time_invalid",
                          "Source IO outlasted the recorded measurement interval")
                collect(final_time)
        return result, gaps, index_path

    def diagnose(self, scene=None, arm=None):
        sources, gaps, index_path = self._resolve(scene, arm)
        return {"ready": not gaps, "gaps": gaps, "index_path": str(index_path),
                "available_sources": sorted(sources), "official_sources": {k: v for k, v in sources.items() if k in ("model_catalog", "urdf_source")},
                "source_truth_authenticated": False, "dispatch_authorized": False, "hardware_commands_sent": 0}

    def __call__(self, scene, arm):
        sources, gaps, _ = self._resolve(scene, arm)
        if gaps:
            error = JointSourcesError("joint_sources_unavailable", "; ".join(g["code"]+": "+g["detail"] for g in gaps))
            error.gaps = gaps
            raise error
        return sources

    def initialization_basis(self, scene, arm):
        """Official model and current connection's limits for explicit RGB review.

        This does not resolve, publish or certify geometry. The first-target
        host branch binds its own explicit RGB initialization evidence.
        """
        sources, gaps, _ = self._resolve(scene, arm, require_geometry=False)
        if gaps:
            error = JointSourcesError("joint_sources_unavailable", "; ".join(g["code"]+": "+g["detail"] for g in gaps))
            error.gaps = gaps
            raise error
        return sources

    def rgb_joint_basis(self, scene, arm):
        """Same numerical sources for the separately bound ordinary RGB step.

        Resolving sources alone grants neither an initialized cache nor a
        motion contract; the host binds operation, target and current images.
        """
        return self.initialization_basis(scene, arm)
