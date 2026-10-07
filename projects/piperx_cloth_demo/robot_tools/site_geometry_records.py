"""Import recorded installation/survey data, never manufacture site evidence.

Only the internal provider calls ``load_geometry_records``. Its fixed input is
``bundle_root/site_records/piper_geometry/<record_set_id>/manifest.json``.
The manifest hashes three finite JSON records. Installation component records
describe enclosing boxes (not sampled vertices) in each physical flange frame;
the inventory includes gripper, camera, bracket and cable. Workspace records
give separate surveyed boxes in BOTH physical model base frames. Clearance
records give surveyed surface-gap minima, including both nonadjacent self
groups, both environment groups and the inter-arm group, for the exact current
scene/owner/connections and a fixed measurement interval.

This parser derives conservative numeric bounds from those readings and their
errors. It cannot authenticate who measured them, their coverage, or physical
truth. No real record, template, CAD installation or numeric default is created.
"""
import copy
import hashlib
import math
import os
from pathlib import Path
import re
import stat

from .joint_sources import (JointSourcesError, SIDES, _need, _num, _text, _hash,
                            _json, _read_at, _canonical_bytes, profile_sha256, RGB_MAX_AGE_S)

MANIFEST_SCHEMA = "piper_geometry_record_set_v1"
INSTALLATION_SCHEMA = "piper_geometry_installation_v1"
WORKSPACE_SCHEMA = "piper_geometry_workspace_v1"
CLEARANCE_SCHEMA = "piper_geometry_clearance_v1"
CATEGORIES = {"gripper", "camera", "bracket", "cable"}
GAP_GROUPS = {"left_environment", "right_environment", "inter_arm",
              "left_self_nonadjacent", "right_self_nonadjacent"}


def _exact(value, fields, label):
    _need(type(value) is dict and set(value) == set(fields.split()),
          "geometry_record_schema", label)


def _vector(value):
    _need(type(value) is list and len(value) == 3, "geometry_record_schema", "Three SI coordinates required")
    return [_num(x, "coordinate") for x in value]


def _error(value):
    value = _num(value, "absolute measurement error")
    _need(value >= 0, "geometry_record_invalid", "Negative measurement error")
    return value


def _direct(value, direction):
    _need(math.isfinite(value), "geometry_record_invalid", "Derived bound is not finite")
    result = math.nextafter(value, direction)
    _need(math.isfinite(result), "geometry_record_invalid", "Derived bound overflow")
    return result


def _box(row):
    lower, upper, error = _vector(row["lower_m"]), _vector(row["upper_m"]), _error(row["error_m"])
    _need(all(a < b for a, b in zip(lower, upper)), "geometry_record_invalid", "Enclosing box must have positive extents")
    return lower, upper, error


def _open_records(root, record_set_id):
    _need(type(record_set_id) is str and re.fullmatch(r"[A-Za-z0-9_-]{1,96}", record_set_id),
          "geometry_record_set_invalid", "Expected a registered record-set identifier, not a path")
    path = Path(root) / "site_records" / "piper_geometry" / record_set_id
    fd = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
    try:
        parts = path.absolute().parts[1:]
        for index, part in enumerate(parts):
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=fd)
            os.close(fd)
            fd = child
            if index >= len(parts)-3:
                info = os.fstat(fd)
                _need(info.st_uid == os.geteuid() and not info.st_mode & 0o022,
                      "geometry_records_unsafe", "Record directories must be host-owned and not group/world writable")
        return fd
    except BaseException:
        os.close(fd)
        raise


def _read_record(fd, name):
    _need(type(name) is str and re.fullmatch(r"[A-Za-z0-9_-]+\.json", name),
          "geometry_record_path_invalid", "Only direct JSON files in the controlled record set are accepted")
    info = os.stat(name, dir_fd=fd, follow_symlinks=False)
    _need(stat.S_ISREG(info.st_mode) and info.st_uid == os.geteuid() and not info.st_mode & 0o022,
          "geometry_records_unsafe", "Record must be a host-owned regular, nonwritable-by-others file")
    return _read_at(fd, name)


def load_geometry_records(bundle_root, record_set_id, *, profile, run_id, owner, bindings, scene_ref, now):
    """Read actual records and return derived bounds plus immutable source bytes.

    Host scope is supplied internally. There is no caller-provided bounds object.
    Static installation/workspace records can be reused unchanged; a newer RGB
    scene cannot relabel or renew an old clearance measurement.
    """
    fd = None
    try:
        fd = _open_records(bundle_root, record_set_id)
        manifest = _json(_read_record(fd, "manifest.json"))
        _exact(manifest, "schema installation workspace clearance", "record manifest")
        _need(manifest["schema"] == MANIFEST_SCHEMA, "geometry_record_schema", "Unknown record manifest")
        records, raw_records, hashes = {}, {}, {}
        for name in ("installation", "workspace", "clearance"):
            ref = manifest[name]
            _exact(ref, "path sha256", "record reference")
            raw = _read_record(fd, ref["path"])
            digest = hashlib.sha256(raw).hexdigest()
            _need(digest == _hash(ref["sha256"]), "geometry_record_hash_mismatch", name)
            records[name], raw_records[name], hashes[name] = _json(raw), raw, digest
        return _derive(records, raw_records, hashes, profile=profile, run_id=run_id,
                       owner=owner, bindings=bindings, scene_ref=scene_ref, now=now)
    except JointSourcesError:
        raise
    except FileNotFoundError as exc:
        raise JointSourcesError("geometry_records_missing", "Actual controlled installation/measurement record absent: " + str(exc.filename)) from exc
    except OSError as exc:
        raise JointSourcesError("geometry_records_unavailable", str(exc)) from exc
    except (KeyError, TypeError, IndexError, OverflowError, ValueError) as exc:
        raise JointSourcesError("geometry_record_schema", str(exc)) from exc
    finally:
        if fd is not None:
            os.close(fd)


def _derive(records, raws, hashes, *, profile, run_id, owner, bindings, scene_ref, now):
    now = _num(now, "clock")
    installation, workspace, clearance = (records[k] for k in ("installation", "workspace", "clearance"))
    _exact(installation, "schema installation_id recorded_at_s recorded_by devices components", "installation")
    _need(installation["schema"] == INSTALLATION_SCHEMA, "geometry_record_schema", "Installation schema")
    _text(installation["installation_id"], "installation identifier")
    _text(installation["recorded_by"], "installation recorder")
    installed_at = _num(installation["recorded_at_s"], "installation time")
    _need(0 <= installed_at <= now, "geometry_record_time_invalid", "Installation record is future/invalid")
    _exact(installation["devices"], "left right", "installed devices")
    _exact(installation["components"], "left right", "installed component inventory")
    radii, metric_kinds = {}, {}
    for side in SIDES:
        device = installation["devices"][side]
        _exact(device, "model usb_interface camera_serial", "installed device identity")
        expected = {"model": bindings[side]["model"], "usb_interface": bindings[side]["usb_interface"],
                    "camera_serial": profile["cameras"][side+"_wrist"]}
        _need(device == expected, "geometry_installation_mismatch", side)
        components = installation["components"][side]
        _need(type(components) is list and 4 <= len(components) <= 128,
              "geometry_inventory_incomplete", side)
        categories, identifiers, radius, kinds = set(), set(), 0., set()
        for part in components:
            _exact(part, "component_id category kind frame lower_m upper_m error_m method", "installed component")
            identifier = _text(part["component_id"], "component identifier")
            _need(identifier not in identifiers, "geometry_record_schema", "Duplicate component")
            identifiers.add(identifier)
            _need(part["category"] in CATEGORIES and part["frame"] == "physical_model_flange"
                  and part["kind"] in ("physical_measurement", "installed_cad_record"),
                  "geometry_record_schema", "Enclosing component frame/kind/category")
            _text(part["method"], "enclosing-box measurement/CAD method")
            categories.add(part["category"])
            kinds.add(part["kind"])
            lower, upper, error = _box(part)
            extents = [_direct(max(abs(a), abs(b))+error, math.inf) for a, b in zip(lower, upper)]
            radius = max(radius, _direct(math.hypot(*extents), math.inf))
        _need(categories == CATEGORIES, "geometry_inventory_incomplete", side)
        radii[side] = radius
        metric_kinds[side+"_attachment"] = ("physical_measurement" if "physical_measurement" in kinds else "installed_cad_record")

    _exact(workspace, "schema installation_sha256 kind recorded_at_s recorded_by frame bounds_by_arm method", "workspace")
    _need(workspace["schema"] == WORKSPACE_SCHEMA and workspace["installation_sha256"] == hashes["installation"]
          and workspace["kind"] in ("physical_measurement", "installed_workspace_record")
          and workspace["frame"] == "physical_model_arm_base", "geometry_workspace_scope_mismatch", "Both installed model-base workspaces required")
    _text(workspace["recorded_by"], "workspace recorder")
    _text(workspace["method"], "workspace survey method")
    workspace_at = _num(workspace["recorded_at_s"], "workspace time")
    _need(installed_at <= workspace_at <= now, "geometry_record_time_invalid", "Workspace predates installation or is future")
    _exact(workspace["bounds_by_arm"], "left right", "Both base-frame workspaces")
    boxes = []
    for side in SIDES:
        row = workspace["bounds_by_arm"][side]
        _exact(row, "lower_m upper_m error_m", side+" workspace")
        lower, upper, error = _box(row)
        boxes.append(([_direct(v+error, math.inf) for v in lower],
                      [_direct(v-error, -math.inf) for v in upper]))
    wlo = [max(box[0][i] for box in boxes) for i in range(3)]
    whi = [min(box[1][i] for box in boxes) for i in range(3)]
    _need(all(a < b for a, b in zip(wlo, whi)), "geometry_workspace_empty", "No conservative common box for both base frames")

    _exact(clearance, "schema installation_sha256 scope measured_at_s valid_until_s recorded_by method measurements", "clearance")
    _need(clearance["schema"] == CLEARANCE_SCHEMA and clearance["installation_sha256"] == hashes["installation"],
          "geometry_clearance_scope_mismatch", "Clearance belongs to another installation")
    expected_scope = {"run_id":run_id, "owner":owner, "profile_sha256":profile_sha256(profile),
        "bindings":bindings, "scene":scene_ref}
    _need(_canonical_bytes(clearance["scope"]) == _canonical_bytes(expected_scope),
          "geometry_clearance_scope_mismatch", "Clearance measurement does not cover this exact scene/owner/connections/profile")
    measured, expires = _num(clearance["measured_at_s"], "measurement time"), _num(clearance["valid_until_s"], "measurement expiry")
    image_time = min(item["host_received_at"] for item in scene_ref["frames"].values())
    _need(max(installed_at,workspace_at) <= measured <= image_time <= now < expires,
          "geometry_record_time_invalid", "New RGB cannot renew expired, future or pre-installation clearance")
    # The existing dispatch guard lasts until the original RGB deadline. A
    # published measurement must ALREADY cover that whole interval, including
    # planning/baseline/each-frame checks. Short records cannot be extended here.
    _need(expires > image_time + RGB_MAX_AGE_S, "geometry_record_time_invalid",
          "Original measurement interval must cover the entire existing RGB action window; no expiry is extended")
    _text(clearance["recorded_by"], "clearance recorder")
    _text(clearance["method"], "surface-gap survey method")
    _need(type(clearance["measurements"]) is dict and set(clearance["measurements"]) == GAP_GROUPS,
          "geometry_clearance_coverage_missing", "Environment/inter-arm/nonadjacent self gap groups required")
    gaps = []
    for group, rows in clearance["measurements"].items():
        _need(type(rows) is list and 0 < len(rows) <= 512, "geometry_clearance_coverage_missing", group)
        for row in rows:
            _exact(row, "surface_pair distance_m error_m", "surface-gap reading")
            _need(type(row["surface_pair"]) is list and len(row["surface_pair"]) == 2,
                  "geometry_record_schema", "Two measured surface identifiers required")
            names = [_text(x, "surface identifier") for x in row["surface_pair"]]
            _need(names[0] != names[1], "geometry_record_schema", "Surface pair must be distinct")
            low = _direct(_num(row["distance_m"], "surface gap", positive=True)-_error(row["error_m"]), -math.inf)
            _need(low > 0, "geometry_clearance_invalid", "Measurement error consumes gap")
            gaps.append(low)
    bounds = {"attachment_radius_m":radii, "available_clearance_m":min(gaps),
              "workspace_min_m":wlo, "workspace_max_m":whi}
    metric_kinds.update(clearance="physical_measurement", workspace=workspace["kind"])
    return {"bounds":bounds, "metric_kinds":metric_kinds, "raw_records":raws,
            "record_sha256":hashes, "valid_until_s":expires, "measurement_scene":copy.deepcopy(scene_ref)}


def validate_imported_geometry(data, metric_bytes, *, profile, now):
    """Recheck this publisher's readings/expiry at consumption, not only import.

    Older separately established metric-source formats retain their existing
    reader contract; this function does not create a way to publish them.
    """
    try:
        clearance = _json(metric_bytes["clearance"])
    except JointSourcesError:
        return
    if not isinstance(clearance, dict) or clearance.get("schema") != CLEARANCE_SCHEMA:
        return
    _need(metric_bytes["left_attachment"] == metric_bytes["right_attachment"],
          "geometry_installation_mismatch", "Imported installation must cover both arms")
    raws = {"installation":metric_bytes["left_attachment"], "workspace":metric_bytes["workspace"],
            "clearance":metric_bytes["clearance"]}
    records = {k:_json(v) for k,v in raws.items()}
    hashes = {k:hashlib.sha256(v).hexdigest() for k,v in raws.items()}
    try:
        derived = _derive(records, raws, hashes, profile=profile, run_id=data["run_id"],
                          owner=data["owner"], bindings=data["bindings"], scene_ref=data["scene"], now=now)
        _need(_canonical_bytes(data["bounds"]) == _canonical_bytes(derived["bounds"]),
              "geometry_derived_bounds_mismatch", "Published bounds differ from the actual source readings")
        _need(all(data["metric_sources"][k]["kind"] == v for k,v in derived["metric_kinds"].items()),
              "geometry_metric_source_invalid", "Published source kinds differ from installation readings")
        return derived["valid_until_s"]
    except (KeyError, TypeError, IndexError, OverflowError, ValueError) as exc:
        if isinstance(exc, JointSourcesError):
            raise
        raise JointSourcesError("geometry_record_schema", str(exc)) from exc
