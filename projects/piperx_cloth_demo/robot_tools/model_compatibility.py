"""Offline saved-feedback / SDK-model comparison; never a motion admission.

CLI::

    python -m robot_tools.model_compatibility input.json --output report.json

Input schema ``piper_model_compatibility_input_v1`` has ``model_catalog``
(``constants_path``, ``sha256``, ``commit``, ``source_url``), and ``arms``.
Each left/right arm has ``selected_model``, ``declared_physical_model``,
``physical_model_source``, ``binding`` and ``samples``. Binding contains
``channel``, ``usb_interface``, ``firmware_profile`` and ``firmware_version``
(explicit null is allowed, but cannot establish firmware identity).
Each sample includes its binding, sample_id, joints_rad, pose_m_rad,
joint_order, units, rpy_convention, joint_source, pose_source, pose_frame,
pose_reference, and ``source`` (path, sha256, json_pointer to a saved arm
snapshot). These values must match the referenced snapshot. No source code
is executed: SDK constants are extracted using ast.literal_eval only.

FK implements the official SDK's documented modified-DH equation, using
its literal model parameters. Independent SDK numerical comparison lives
in the tests. Matrix errors avoid Euler-angle subtraction. Fixed-base and
fixed-tool estimates are descriptive hypotheses, never accepted transforms;
the combined two-sided transform and joint-zero calibration are not fitted.
"""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import re


SCHEMA = "piper_model_compatibility_input_v1"
MODELS = ("piper", "piper_x", "piper_h", "piper_l")
JOINT_ORDER = ["joint%d" % i for i in range(1, 7)]
RPY_CONVENTION = "Rz(yaw) Ry(pitch) Rx(roll)"
UNITS = {"joints": "rad", "xyz": "m", "rpy": "rad"}
PARTS = ("joint_12", "joint_34", "joint_56", "end_pose_xy", "end_pose_zrx", "end_pose_ryrz")
# Diagnostic comparators mirror the existing FK agreement check. They neither
# configure nor relax a robot guard. Diversity is an observation descriptor.
POSITION_TOLERANCE_M = .002
ROTATION_TOLERANCE_RAD = .02
POSE_SEPARATION_RAD = .01
MIN_DISTINCT_POSES = 4
MAX_SAMPLES = 128
KNOWN_OFFICIAL_CONSTANTS = {
    "841a625f5f4920e776f20b934eb13048b747e6d0":
        "b72cd2a0e7499483e1313acf0e98781510cb151de592c3fe1291df712ab23c5b",
}


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number, never bool")
    return float(value)


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + " has the wrong dimensions")
    return [_number(x, name) for x in value]


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(name + " must be explicit nonempty text")
    return value


def _json(raw):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON key: " + key)
            result[key] = value
        return result
    def invalid(value):
        raise ValueError("Nonfinite JSON token: " + value)
    return json.loads(raw, object_pairs_hook=unique, parse_constant=invalid)


def _read_hashed(path, expected, maximum):
    if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
        raise ValueError("A lowercase SHA256 is required")
    source = Path(_text(path, "source path")).expanduser().resolve(strict=True)
    if not source.is_file() or source.stat().st_size > maximum:
        raise ValueError("Source must be a bounded regular file")
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError("SHA256 mismatch: " + str(source))
    return raw


def load_model_catalog(spec):
    """Read literal SDK MDH/limits without importing or executing the SDK."""
    commit = spec.get("commit")
    if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("Explicit full SDK commit required")
    expected_url = ("https://raw.githubusercontent.com/agilexrobotics/pyAgxArm/" +
                    commit + "/pyAgxArm/api/constants.py")
    if spec.get("source_url") != expected_url:
        raise ValueError("SDK source URL and commit disagree")
    raw = _read_hashed(spec.get("constants_path"), spec.get("sha256"), 1024 * 1024)
    known_hash = KNOWN_OFFICIAL_CONSTANTS.get(commit)
    if known_hash is not None and known_hash != spec["sha256"]:
        raise ValueError("Recognized official commit has a different constants hash")
    wanted = {"ROBOT_MDH_PRESET", "ROBOT_JOINT_LIMIT_PRESET_RAD"}
    literals = {}
    for node in ast.parse(raw).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    if target.id in literals:
                        raise ValueError("Duplicate SDK constant assignment")
                    literals[target.id] = ast.literal_eval(node.value)
    if set(literals) != wanted:
        raise ValueError("SDK literal MDH and radians limit tables required")
    models = {}
    for model in MODELS:
        table = literals["ROBOT_MDH_PRESET"][model]
        if len(table) != 6:
            raise ValueError("Six MDH rows required")
        mdh = [_vector(row, 4, "MDH row") for row in table]
        limits = [_vector(literals["ROBOT_JOINT_LIMIT_PRESET_RAD"][model][name], 2,
                          "joint limits") for name in JOINT_ORDER]
        if any(low >= high for low, high in limits):
            raise ValueError("Joint lower limit must precede upper limit")
        models[model] = {"mdh": mdh, "joint_limits_rad": limits}
    return models, {**copy.deepcopy(spec), "sha256_verified": True,
                    "recognized_official_snapshot": known_hash is not None,
                    "origin_scope": "Pinned previously reviewed bytes" if known_hash else
                    "Caller-declared new source; hash checked, official origin not authenticated",
                    "extract_method": "AST literals only; no SDK import or execution"}


def _identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def _multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _inverse(t):
    result = _identity()
    for i in range(3):
        for j in range(3):
            result[i][j] = t[j][i]
        result[i][3] = -sum(t[j][i] * t[j][3] for j in range(3))
    return result


def fk_matrix(mdh, joints_rad):
    """SDK MDH: Rx(alpha) Tx(a) Rz(q + offset) Tz(d), no robot object."""
    joints = _vector(joints_rad, 6, "joints_rad")
    if len(mdh) != 6:
        raise ValueError("Six MDH rows required")
    result = _identity()
    for row, q in zip(mdh, joints):
        d, a, alpha, offset = _vector(row, 4, "MDH row")
        ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(q + offset), math.sin(q + offset)
        link = [[ct, -st, 0., a], [ca * st, ca * ct, -sa, -sa * d],
                [sa * st, sa * ct, ca, ca * d], [0., 0., 0., 1.]]
        result = _multiply(result, link)
    return result


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = _vector(pose, 6, "pose_m_rad")
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z], [0., 0., 0., 1.]]


def matrix_error(a, b):
    distance = math.sqrt(sum((a[i][3] - b[i][3]) ** 2 for i in range(3)))
    # atan2 is stable at both 0 and pi; unlike Euler subtraction it respects
    # wraparound and equivalent pitch/roll parameterizations.
    r = [[sum(a[k][i] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    cosine = max(-1., min(1., (sum(r[i][i] for i in range(3)) - 1.) / 2.))
    sine = .5 * math.sqrt(sum(x * x for x in
                             (r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1])))
    angle = math.atan2(sine, cosine)
    return {"position_error_m": distance, "so3_error_rad": angle,
            "within_diagnostic_tolerance": distance <= POSITION_TOLERANCE_M and angle <= ROTATION_TOLERANCE_RAD}


def _binding(value):
    if not isinstance(value, dict):
        raise ValueError("Device/firmware binding is required")
    keys = ("channel", "usb_interface", "firmware_profile", "firmware_version")
    if set(value) != set(keys):
        raise ValueError("Binding requires exactly " + repr(keys))
    for key in keys[:-1]:
        _text(value[key], key)
    if value["firmware_version"] is not None:
        _text(value["firmware_version"], "firmware_version")
    return copy.deepcopy(value)


def _pointer(value, pointer):
    if not isinstance(pointer, str) or not pointer.startswith("/"):
        raise ValueError("Source json_pointer must select a saved arm snapshot")
    for token in pointer[1:].split("/"):
        key = token.replace("~1", "/").replace("~0", "~")
        value = value[int(key)] if isinstance(value, list) else value[key]
    return value


def _sample(sample, binding, source_cache):
    if not isinstance(sample, dict):
        raise ValueError("Each sample must be a JSON object")
    if _binding(sample.get("binding")) != binding:
        raise ValueError("Device or firmware binding changed; split these samples into a separate report")
    if sample.get("joint_order") != JOINT_ORDER or sample.get("units") != UNITS:
        raise ValueError("Explicit joint1..joint6 order and rad/m/rad units required; no guessed conversion")
    if sample.get("rpy_convention") != RPY_CONVENTION:
        raise ValueError("RPY convention unsupported; no guessed conversion")
    for key in ("joint_source", "pose_source"):
        if sample.get(key) != "controller_feedback":
            raise ValueError("Independent controller feedback required; computed FK is not a controller observation")
    if sample.get("pose_frame") != "arm_base" or sample.get("pose_reference") != "flange":
        raise ValueError("Explicit arm_base/flange feedback required; tool/base transforms must not be assumed")
    q = _vector(sample.get("joints_rad"), 6, "joints_rad")
    pose = _vector(sample.get("pose_m_rad"), 6, "pose_m_rad")
    source = sample["source"]
    key = (source["path"], source["sha256"])
    if key not in source_cache:
        source_cache[key] = _json(_read_hashed(*key, maximum=64 * 1024 * 1024))
    raw = _pointer(source_cache[key], source.get("json_pointer"))
    if not isinstance(raw, dict):
        raise ValueError("Source json_pointer must resolve to an arm snapshot object")
    if q != _vector(raw.get("joints_rad"), 6, "saved joints") or pose != _vector(raw.get("pose_m_rad"), 6, "saved pose"):
        raise ValueError("Sample numerical values differ from the hashed source snapshot")
    for field in ("channel", "usb_interface", "firmware_profile"):
        if raw.get(field) != binding[field]:
            raise ValueError("Source snapshot binding disagrees: " + field)
    if raw.get("firmware_version") != binding["firmware_version"]:
        raise ValueError("Firmware version must be supported by this source; use null if absent")
    captured = _number(raw.get("timestamp"), "saved timestamp")
    stamps = [_number(raw["fragment_timestamps_s"].get(part), part) for part in PARTS]
    age, skew = captured - min(stamps), max(stamps) - min(stamps)
    if min(stamps) <= 0 or max(stamps) > captured or age > .1 or skew > .1:
        raise ValueError("Saved joint/pose fragments were not coherent within 100ms at capture")
    return {"sample_id": _text(sample.get("sample_id"), "sample_id"),
            "joints_rad": q, "pose_m_rad": pose, "source": copy.deepcopy(source),
            "captured_at_s": captured, "fragment_age_at_capture_s": age,
            "fragment_skew_s": skew, "source_sha256_and_values_verified": True,
            "configured_model_in_source": raw.get("configured_model"),
            "historical_only": True}


def _diversity(samples):
    representatives = []
    for row in samples:
        if all(max(abs(a - b) for a, b in zip(row["joints_rad"], other)) >= POSE_SEPARATION_RAD
               for other in representatives):
            representatives.append(row["joints_rad"])
    spans = [max(s["joints_rad"][i] for s in samples) - min(s["joints_rad"][i] for s in samples)
             for i in range(6)] if samples else [0.] * 6
    return {"distinct_pose_count": len(representatives), "joint_spans_rad": spans,
            "separation_rad": POSE_SEPARATION_RAD, "minimum_distinct_pose_count": MIN_DISTINCT_POSES,
            "descriptive_diversity_sufficient": len(representatives) >= MIN_DISTINCT_POSES and
            sum(span >= POSE_SEPARATION_RAD for span in spans) >= 2,
            "observability_or_calibration_proven": False}


def _hypothesis(model_matrices, observed, diversity, left):
    # Derive from FIRST pose only, then test the other poses, never solve a
    # transform independently per row and claim that each residual is zero.
    transform = (_multiply(observed[0], _inverse(model_matrices[0])) if left else
                 _multiply(_inverse(model_matrices[0]), observed[0]))
    errors = [matrix_error(_multiply(transform, t) if left else _multiply(t, transform), measured)
              for t, measured in zip(model_matrices, observed)]
    consistent = all(e["within_diagnostic_tolerance"] for e in errors[1:])
    # Two observations can falsify a fixed transform, even though several
    # diverse observations still cannot establish physical correctness.
    status = ("inconsistent_hypothesis" if len(errors) > 1 and not consistent else
              "insufficient_pose_diversity" if not diversity["descriptive_diversity_sufficient"] else
              "numerically_consistent_hypothesis")
    return {"status": status, "transform_4x4_from_first_pose": transform,
            "fit_sample_index": 0, "comparison_sample_indices": list(range(1, len(errors))),
            "comparison_errors": errors[1:], "transform_verified": False,
            "eligible_for_execution": False,
            "limitations": "No joint-zero fit, two-sided AXB fit, observability proof or independent physical validation"}


def diagnose(document):
    """Return a bounded diagnostic report; invalid evidence cannot earn matches."""
    if document.get("schema") != SCHEMA:
        raise ValueError("Unsupported input schema")
    models, catalog_source = load_model_catalog(document["model_catalog"])
    arm_inputs = document.get("arms")
    if not isinstance(arm_inputs, dict) or not arm_inputs or set(arm_inputs) - {"left", "right"}:
        raise ValueError("Explicit left and/or right arm inputs required")
    report = {"schema": "piper_model_compatibility_report_v1", "model_catalog_source": catalog_source,
              "model_parameters": copy.deepcopy(models),
              "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "coordinate_contract": {"joint_order": JOINT_ORDER[:], "units": dict(UNITS),
                                      "rpy_convention": RPY_CONVENTION,
                                      "pose_frame": "arm_base", "pose_reference": "flange",
                                      "source_semantics": "Caller-declared controller-feedback semantics; saved values and hashes checked, original bus acquisition not reauthenticated"},
              "mode": "offline_saved_feedback_diagnostic", "hardware_commands_sent": 0,
              "devices_constructed": 0, "configuration_written": False,
              "motion_permitted": False, "qualification_granted": False,
              "diagnostic_thresholds": {"position_m": POSITION_TOLERANCE_M,
                                        "so3_rad": ROTATION_TOLERANCE_RAD}, "arms": {}}
    cache = {}
    for side, spec in arm_inputs.items():
        selected = spec.get("selected_model")
        physical = spec.get("declared_physical_model")
        if selected not in MODELS or physical not in MODELS:
            raise ValueError("Explicit supported selected and declared physical models required")
        model_source = _text(spec.get("physical_model_source"), "physical_model_source")
        binding = _binding(spec.get("binding"))
        incoming = spec.get("samples")
        if not isinstance(incoming, list) or not 1 <= len(incoming) <= MAX_SAMPLES:
            raise ValueError("Each arm needs 1..128 saved samples")
        rows, rejected, identifiers = [], [], set()
        for sample in incoming:
            try:
                row = _sample(sample, binding, cache)
                if row["sample_id"] in identifiers:
                    raise ValueError("Duplicate sample_id")
                identifiers.add(row["sample_id"])
                rows.append(row)
            except (KeyError, IndexError, TypeError, ValueError, OSError) as error:
                rejected.append({"sample_id": sample.get("sample_id") if isinstance(sample, dict) else None,
                                 "reason": str(error)})
        diversity = _diversity(rows)
        result = {"selected_model": selected, "declared_physical_model": physical,
                  "physical_model_source": model_source, "physical_identity_verified_by_report": False,
                  "binding": binding, "firmware_version_observed_in_samples": binding["firmware_version"] is not None,
                  "current_device_binding_verified": False,
                  "binding_scope": "Historical source fields only; not a current device identity check",
                  "samples": rows, "rejected_samples": rejected, "diversity": diversity,
                  "candidate_model_comparisons": {}, "controller_model_candidates": [],
                  "motion_permitted": False, "fixed_transform_verified": False}
        for model, parameters in models.items():
            predicted, measured = [], []
            for row in rows:
                fk = fk_matrix(parameters["mdh"], row["joints_rad"])
                actual = pose_matrix(row["pose_m_rad"])
                predicted.append(fk); measured.append(actual)
                comparison = {**matrix_error(fk, actual), "model_flange_transform_4x4": fk,
                              "nominal_joint_limit_violations": [i + 1 for i, (q, limits) in
                                  enumerate(zip(row["joints_rad"], parameters["joint_limits_rad"]))
                                  if not limits[0] <= q <= limits[1]]}
                row.setdefault("model_comparisons", {})[model] = comparison
            matches = bool(rows) and all(r["model_comparisons"][model]["within_diagnostic_tolerance"] for r in rows)
            entry = {"all_accepted_samples_numerically_match": matches,
                     "max_position_error_m": max((r["model_comparisons"][model]["position_error_m"] for r in rows), default=None),
                     "max_so3_error_rad": max((r["model_comparisons"][model]["so3_error_rad"] for r in rows), default=None)}
            if rows:
                entry["fixed_base_transform_hypothesis"] = _hypothesis(predicted, measured, diversity, True)
                entry["fixed_tool_transform_hypothesis"] = _hypothesis(predicted, measured, diversity, False)
            result["candidate_model_comparisons"][model] = entry
            if matches and not rejected:
                result["controller_model_candidates"].append({"model": model,
                    "evidence": "multi_pose_numerical_match" if diversity["descriptive_diversity_sufficient"] else "limited_pose_numerical_match",
                    "physical_model_identification": False, "firmware_model_identification": False})
        selected_match = result["candidate_model_comparisons"][selected]["all_accepted_samples_numerically_match"]
        if rejected:
            action = "repair_input_evidence_or_split_device_firmware_groups"
        elif not catalog_source["recognized_official_snapshot"]:
            action = "authenticate_new_catalog_source_before_interpreting_candidates"
        elif selected != physical:
            action = "reconcile_selected_model_with_declared_physical_model_without_relabeling_hardware"
        elif not selected_match:
            action = "reconcile_controller_frame_firmware_and_joint_mapping_before_motion"
        elif not diversity["descriptive_diversity_sufficient"] or binding["firmware_version"] is None:
            action = "obtain_missing_binding_or_diverse_saved_pose_evidence_without_automatic_motion"
        else:
            action = "plan_separate_bounded_physical_validation_not_automatic_admission"
        result["next_action"] = action
        result["interpretation"] = "Numerical compatibility describes saved data; no physical identity, calibration, stop, collision or execution qualification"
        report["arms"][side] = result
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    # Refuse an existing output before evaluating; never replace source or a
    # previous diagnostic, and never execute source-file Python statements.
    if args.output.exists():
        parser.error("Output already exists; choose a new report path")
    raw = args.input.read_bytes()
    report = diagnose(_json(raw))
    report["input"] = {"path": str(args.input.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"report": str(args.output), "motion_permitted": False,
                      "next_actions": {side: row["next_action"] for side, row in report["arms"].items()}}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
