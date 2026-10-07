#!/usr/bin/env python3
"""Verify the source snapshot and the local optimization layer separately."""
import hashlib
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
errors = []
source = json.loads((root / "metadata/source_manifest.json").read_text(encoding="utf-8"))
optimization_path = root / "metadata/optimization_manifest.json"
optimization = json.loads(optimization_path.read_text(encoding="utf-8")) if optimization_path.is_file() else {}


def safe_path(relative):
    path = root / relative
    try:
        path.resolve().relative_to(root)
    except ValueError:
        errors.append("Path escapes bundle: " + str(relative))
        return None
    return path


def digest(relative):
    path = safe_path(relative)
    if path is None or not path.is_file():
        errors.append("Missing: " + str(relative))
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


source_by_path = {item["local_path"]: item["sha256"] for item in source.get("files", [])}
optimization_by_path = {}
optimization_errors = []
for item in optimization.get("entries", []):
    relative = item.get("local_path")
    optimized = item.get("optimized_sha256")
    base = item.get("base_source_sha256")
    if not isinstance(relative, str) or not isinstance(optimized, str) or len(optimized) != 64:
        optimization_errors.append("Invalid optimization entry")
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


source_unmodified = 0
source_overridden = 0
for item in source.get("files", []):
    relative, expected = item["local_path"], item["sha256"]
    actual = digest(relative)
    if actual == expected:
        source_unmodified += 1
    elif actual == optimization_by_path.get(relative, {}).get("optimized_sha256"):
        source_overridden += 1
    else:
        errors.append("Unexpected source change: " + relative)


optimized_verified = 0
for relative, item in optimization_by_path.items():
    actual = digest(relative)
    if actual == item["optimized_sha256"]:
        optimized_verified += 1
    else:
        optimization_errors.append("Optimized hash mismatch: " + relative)


checksum_entries = []
checksum_errors = []
checksums = root / "SHA256SUMS"
if checksums.exists():
    for line in checksums.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            expected, relative = line.split("  ", 1)
        except ValueError:
            checksum_errors.append("Malformed checksum line")
            continue
        checksum_entries.append(relative)
        actual = digest(relative)
        allowed = actual == expected or actual == optimization_by_path.get(relative, {}).get("optimized_sha256")
        if not allowed:
            checksum_errors.append("Checksum mismatch: " + relative)
else:
    checksum_errors.append("Missing: SHA256SUMS")

errors.extend(optimization_errors)
errors.extend(checksum_errors)
source_status = "verified" if source_overridden == 0 else "verified_with_optimization_overrides"
result = {
    "status": "failed" if errors else "passed",
    "source_snapshot_status": source_status if not any(e.startswith("Unexpected source") for e in errors) else "failed",
    "optimization_status": "verified" if optimization_by_path and optimized_verified == len(optimization_by_path) and not optimization_errors else "failed",
    "bundle_checksum_status": "verified" if not checksum_errors else "failed",
    "remote_source_files": len(source.get("files", [])),
    "source_unmodified_files": source_unmodified,
    "source_files_explained_by_optimization": source_overridden,
    "optimization_entries": len(optimization_by_path),
    "bundle_checksum_entries": len(checksum_entries),
    "hardware_accessed": False,
    "errors": errors,
}
print(json.dumps(result, ensure_ascii=False, indent=2))
sys.exit(1 if errors else 0)
