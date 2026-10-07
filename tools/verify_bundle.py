#!/usr/bin/env python3
"""Verify the source snapshot and the local optimization layer separately."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys


RUNTIME_OMISSION_PATHS = frozenset(
    "projects/piper_right_pick_demo/configs/" + name + ".local.json"
    for name in ("site", "gpt_tools", "astra_fast_live", "astra_fast_codex"))
OMISSION_REASON = "machine_private_runtime_configuration_not_imported_or_activated"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _sha(value):
    return isinstance(value, str) and _SHA256.fullmatch(value) is not None


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def verify_bundle(bundle_root):
    """Read files only; absent private configs need a source-bound ledger entry.

    Existing files (including eligible config names) always receive ordinary
    hash verification. This does not load or activate any runtime configuration.
    """
    root = Path(bundle_root).resolve()
    errors, source_errors, optimization_errors, checksum_errors, omission_errors = [], [], [], [], []

    def read_json(relative, target_errors, *, optional=False):
        path = root / relative
        if optional and not path.exists() and not path.is_symlink():
            return {}
        try:
            path.resolve().relative_to(root)
            value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
            if not isinstance(value, dict):
                raise ValueError("JSON root must be an object")
            return value
        except (OSError, ValueError) as exc:
            target_errors.append("Cannot read " + relative + ": " + str(exc))
            return {}

    def safe_path(relative, target_errors):
        if not isinstance(relative, str) or not relative or "\\" in relative:
            target_errors.append("Invalid bundle path: " + str(relative))
            return None
        parts = PurePosixPath(relative)
        if parts.is_absolute() or ".." in parts.parts or parts.as_posix() != relative or relative == ".":
            target_errors.append("Path escapes or is noncanonical: " + relative)
            return None
        path = root / relative
        try:
            path.resolve().relative_to(root)
        except (OSError, ValueError):
            target_errors.append("Path escapes bundle: " + relative)
            return None
        return path

    source = read_json("metadata/source_manifest.json", source_errors)
    optimization = read_json("metadata/optimization_manifest.json", optimization_errors, optional=True)
    ledger = read_json("metadata/runtime_omissions.json", omission_errors, optional=True)
    source_by_path, optimization_by_path, omissions = {}, {}, {}
    files = source.get("files", [])
    if "files" not in source or not isinstance(files, list):
        source_errors.append("Source files must be a list")
        files = []
    for item in files:
        if not isinstance(item, dict) or not _sha(item.get("sha256")):
            source_errors.append("Invalid source entry")
            continue
        relative = item.get("local_path")
        if safe_path(relative, source_errors) is None:
            continue
        if relative in source_by_path:
            source_errors.append("Duplicate source entry: " + relative)
            continue
        source_by_path[relative] = item["sha256"]

    entries = optimization.get("entries", [])
    if not isinstance(entries, list):
        optimization_errors.append("Optimization entries must be a list")
        entries = []
    for item in entries:
        if not isinstance(item, dict) or not _sha(item.get("optimized_sha256")):
            optimization_errors.append("Invalid optimization entry")
            continue
        relative, base = item.get("local_path"), item.get("base_source_sha256")
        if safe_path(relative, optimization_errors) is None:
            continue
        if relative in optimization_by_path:
            optimization_errors.append("Duplicate optimization entry: " + relative)
            continue
        optimization_by_path[relative] = item
        expected_source = source_by_path.get(relative)
        if expected_source is not None and base != expected_source:
            optimization_errors.append("Optimization base does not match source: " + relative)
        if expected_source is None and base is not None:
            optimization_errors.append("New optimization file must have null base hash: " + relative)

    ledger_path = root / "metadata/runtime_omissions.json"
    if ledger_path.exists() or ledger_path.is_symlink():
        if (set(ledger) != {"schema_version", "entries"} or
                ledger.get("schema_version") != "runtime_omissions_v1" or
                not isinstance(ledger.get("entries"), list)):
            omission_errors.append("Invalid runtime omission ledger")
        else:
            for item in ledger["entries"]:
                if not isinstance(item, dict) or set(item) != {"local_path", "source_sha256", "reason"}:
                    omission_errors.append("Invalid runtime omission entry")
                    continue
                relative = item["local_path"]
                if safe_path(relative, omission_errors) is None:
                    continue
                if relative not in RUNTIME_OMISSION_PATHS:
                    omission_errors.append("Unknown runtime omission: " + relative)
                    continue
                if relative in omissions:
                    omission_errors.append("Duplicate runtime omission: " + relative)
                    continue
                if (not _sha(item["source_sha256"]) or
                        source_by_path.get(relative) != item["source_sha256"]):
                    omission_errors.append("Runtime omission source hash mismatch: " + relative)
                    continue
                if item["reason"] != OMISSION_REASON:
                    omission_errors.append("Invalid runtime omission reason: " + relative)
                    continue
                omissions[relative] = item
    if omission_errors:
        omissions = {}  # A malformed ledger grants no exemptions.

    omitted_paths = set()

    def digest(relative, target_errors):
        path = safe_path(relative, target_errors)
        if path is None:
            return None, False
        # Dangling symlinks/directories are invalid entries, not missing files.
        if relative in omissions and not path.exists() and not path.is_symlink():
            omitted_paths.add(relative)
            return None, True
        if not path.is_file():
            target_errors.append("Missing or not a regular file: " + relative)
            return None, False
        try:
            return hashlib.sha256(path.read_bytes()).hexdigest(), False
        except OSError as exc:
            target_errors.append("Cannot hash " + relative + ": " + str(exc))
            return None, False

    source_unmodified = source_overridden = source_omitted = 0
    for relative, expected in source_by_path.items():
        actual, omitted = digest(relative, source_errors)
        if omitted:
            source_omitted += 1
        elif actual == expected:
            source_unmodified += 1
        elif actual is not None and actual == optimization_by_path.get(relative, {}).get("optimized_sha256"):
            source_overridden += 1
        else:
            source_errors.append("Unexpected source change: " + relative)

    optimized_verified = optimized_omitted = 0
    for relative, item in optimization_by_path.items():
        actual, omitted = digest(relative, optimization_errors)
        if omitted:
            optimized_omitted += 1
        elif actual == item["optimized_sha256"]:
            optimized_verified += 1
        else:
            optimization_errors.append("Optimized hash mismatch: " + relative)

    checksum_entries, checksum_omitted = set(), 0
    checksums = safe_path("SHA256SUMS", checksum_errors)
    try:
        lines = checksums.read_text(encoding="utf-8").splitlines() if checksums is not None else []
    except (OSError, ValueError) as exc:
        checksum_errors.append("Cannot read SHA256SUMS: " + str(exc))
        lines = []
    for line in lines:
        if not line.strip():
            continue
        try:
            expected, relative = line.split("  ", 1)
        except ValueError:
            checksum_errors.append("Malformed checksum line")
            continue
        if not _sha(expected) or relative in checksum_entries:
            checksum_errors.append("Invalid or duplicate checksum entry: " + relative)
            continue
        checksum_entries.add(relative)
        actual, omitted = digest(relative, checksum_errors)
        optimized = optimization_by_path.get(relative, {}).get("optimized_sha256")
        if omitted:
            # No bytes are claimed verified; the recorded checksum must still
            # be explained by this source or its declared optimization.
            if expected not in (source_by_path.get(relative), optimized):
                checksum_errors.append("Omitted checksum has unknown hash: " + relative)
            else:
                checksum_omitted += 1
        elif actual is None or not (actual == expected or actual == optimized):
            checksum_errors.append("Checksum mismatch: " + relative)

    errors.extend(source_errors + optimization_errors + checksum_errors + omission_errors)
    source_status = "verified_with_optimization_overrides" if source_overridden else "verified"
    if source_omitted:
        source_status += "_with_runtime_omissions"
    return {
        "status": "failed" if errors else "passed",
        "source_snapshot_status": "failed" if source_errors else source_status,
        "optimization_status": "failed" if optimization_errors else
            ("verified_with_runtime_omissions" if optimized_omitted else "verified"),
        "bundle_checksum_status": "failed" if checksum_errors else
            ("verified_with_runtime_omissions" if checksum_omitted else "verified"),
        "runtime_omission_status": "failed" if omission_errors else "verified",
        "remote_source_files": len(files), "source_unmodified_files": source_unmodified,
        "source_files_explained_by_optimization": source_overridden,
        "source_files_omitted": source_omitted,
        "optimization_entries": len(optimization_by_path),
        "optimized_verified_files": optimized_verified, "optimized_omitted_files": optimized_omitted,
        "bundle_checksum_entries": len(checksum_entries), "bundle_checksum_omitted_files": checksum_omitted,
        "runtime_omission_entries": len(omissions), "omitted_runtime_files": sorted(omitted_paths),
        "hardware_accessed": False, "errors": errors,
    }


def main():
    result = verify_bundle(Path(__file__).resolve().parents[1])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 1 if result["errors"] else 0


if __name__ == "__main__":
    sys.exit(main())
