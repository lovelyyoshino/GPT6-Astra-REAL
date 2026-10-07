"""Run a copied verifier against minimal temporary bundles; no real configs."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


VERIFIER = Path(__file__).resolve().parents[1] / "verify_bundle.py"
PREFIX = "projects/piper_right_pick_demo/configs/"
NAMES = ("site", "gpt_tools", "astra_fast_live", "astra_fast_codex")
REASON = "machine_private_runtime_configuration_not_imported_or_activated"


def sha(data):
    return hashlib.sha256(data).hexdigest()


class VerifyBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.outside = Path(self.temp.name)
        self.root = self.outside / "bundle"
        (self.root / "tools").mkdir(parents=True)
        (self.root / "metadata").mkdir()
        shutil.copyfile(VERIFIER, self.root / "tools/verify_bundle.py")
        self.regular = "projects/demo.py"
        self.content = b"# harmless offline fixture\n"
        path = self.root / self.regular
        path.parent.mkdir(parents=True)
        path.write_bytes(self.content)
        self.private = {PREFIX + n + ".local.json": ("not activated: " + n).encode() for n in NAMES}
        self.optimized = {PREFIX + n + ".local.json": ("optimized snapshot: " + n).encode()
                          for n in ("astra_fast_live", "astra_fast_codex")}
        self.source = {"files": [{"local_path": self.regular, "sha256": sha(self.content)}] +
                       [{"local_path": p, "sha256": sha(data)} for p, data in self.private.items()]}
        self.optimization = {"entries": [{"local_path": p, "base_source_sha256": sha(self.private[p]),
                                            "optimized_sha256": sha(data)} for p, data in self.optimized.items()]}
        self.ledger = {"schema_version": "runtime_omissions_v1", "entries": [
            {"local_path": p, "source_sha256": sha(data), "reason": REASON}
            for p, data in self.private.items()]}
        self.checksums = {self.regular: sha(self.content), **{p: sha(self.optimized.get(p, data))
                                                           for p, data in self.private.items()}}
        self.save()

    def save(self):
        for name, value in (("source_manifest", self.source), ("optimization_manifest", self.optimization),
                            ("runtime_omissions", self.ledger)):
            (self.root / ("metadata/" + name + ".json")).write_text(json.dumps(value))
        (self.root / "SHA256SUMS").write_text("".join(h + "  " + p + "\n" for p, h in self.checksums.items()))

    def run_verifier(self):
        source_before = (self.root / "metadata/source_manifest.json").read_bytes()
        result = subprocess.run([sys.executable, str(self.root / "tools/verify_bundle.py")],
                                text=True, capture_output=True, timeout=10, check=False)
        self.assertEqual((self.root / "metadata/source_manifest.json").read_bytes(), source_before)
        data = json.loads(result.stdout)
        self.assertFalse(data["hardware_accessed"])
        self.assertEqual(result.returncode, 0 if data["status"] == "passed" else 1)
        return data

    def failed(self, fragment):
        result = self.run_verifier()
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any(fragment in error for error in result["errors"]), result["errors"])

    def test_only_listed_absent_configs_are_explained_in_all_layers(self):
        result = self.run_verifier()
        self.assertEqual(result["status"], "passed", result["errors"])
        self.assertEqual(result["source_files_omitted"], 4)
        self.assertEqual(result["optimized_omitted_files"], 2)
        self.assertEqual(result["bundle_checksum_omitted_files"], 4)
        self.assertEqual(result["source_unmodified_files"], 1)
        self.assertEqual(result["omitted_runtime_files"], sorted(self.private))
        for field in ("source_snapshot_status", "optimization_status", "bundle_checksum_status"):
            self.assertIn("runtime_omissions", result[field])

    def test_no_ledger_does_not_exempt_even_known_names(self):
        (self.root / "metadata/runtime_omissions.json").unlink()
        self.failed("Missing or not a regular file")

    def test_arbitrary_missing_source_still_fails(self):
        (self.root / self.regular).unlink()
        self.failed("Unexpected source change: " + self.regular)

    def test_existing_regular_file_wrong_hash_fails(self):
        (self.root / self.regular).write_text("tampered")
        self.failed("Checksum mismatch: " + self.regular)

    def test_existing_private_file_is_verified_not_omitted(self):
        relative = PREFIX + "site.local.json"
        p = self.root / relative
        p.parent.mkdir(parents=True)
        p.write_bytes(self.private[relative])
        result = self.run_verifier()
        self.assertEqual(result["status"], "passed", result["errors"])
        self.assertEqual(result["source_files_omitted"], 3)
        self.assertEqual(result["source_unmodified_files"], 2)
        self.assertNotIn(relative, result["omitted_runtime_files"])

    def test_existing_private_file_wrong_hash_is_not_exempt(self):
        relative = PREFIX + "site.local.json"
        p = self.root / relative
        p.parent.mkdir(parents=True)
        p.write_text("unknown local settings")
        self.failed("Unexpected source change: " + relative)

    def test_existing_optimized_private_file_uses_normal_override(self):
        relative = PREFIX + "astra_fast_live.local.json"
        p = self.root / relative
        p.parent.mkdir(parents=True)
        p.write_bytes(self.optimized[relative])
        result = self.run_verifier()
        self.assertEqual(result["status"], "passed", result["errors"])
        self.assertEqual(result["source_files_explained_by_optimization"], 1)
        self.assertEqual(result["optimized_verified_files"], 1)
        self.assertEqual(result["optimized_omitted_files"], 1)

    def test_unknown_omission_is_rejected_even_if_source_contains_it(self):
        self.ledger["entries"].append({"local_path": self.regular, "source_sha256": sha(self.content), "reason": REASON})
        self.save()
        self.failed("Unknown runtime omission")

    def test_wrong_omission_source_hash_is_rejected(self):
        self.ledger["entries"][0]["source_sha256"] = "0" * 64
        self.save()
        self.failed("Runtime omission source hash mismatch")

    def test_wrong_reason_or_unknown_fields_are_rejected(self):
        self.ledger["entries"][0]["reason"] = "convenience"
        self.save()
        self.failed("Invalid runtime omission reason")
        self.ledger["entries"][0]["reason"] = REASON
        self.ledger["entries"][0]["skip_hash_when_present"] = True
        self.save()
        self.failed("Invalid runtime omission entry")

    def test_duplicate_omission_and_empty_schema_are_rejected(self):
        self.ledger["entries"].append(dict(self.ledger["entries"][0]))
        self.save()
        self.failed("Duplicate runtime omission")
        self.ledger = {}
        self.save()
        self.failed("Invalid runtime omission ledger")

    def test_wrong_checksum_for_omitted_file_is_rejected(self):
        self.checksums[PREFIX + "site.local.json"] = "0" * 64
        self.save()
        self.failed("Omitted checksum has unknown hash")

    def test_wrong_optimization_base_for_omitted_file_is_rejected(self):
        self.optimization["entries"][0]["base_source_sha256"] = "0" * 64
        self.save()
        self.failed("Optimization base does not match source")

    def test_source_and_omission_paths_cannot_escape_bundle(self):
        secret = self.outside / "outside.txt"
        secret.write_bytes(self.content)
        self.source["files"].append({"local_path": "../outside.txt", "sha256": sha(self.content)})
        self.save()
        self.failed("Path escapes")
        self.source["files"].pop()
        self.ledger["entries"][0]["local_path"] = str(secret)
        self.save()
        self.failed("Path escapes")

    def test_symlink_escape_and_dangling_symlink_are_not_omissions(self):
        relative = PREFIX + "site.local.json"
        target = self.outside / "outside.txt"
        target.write_bytes(self.private[relative])
        link = self.root / relative
        link.parent.mkdir(parents=True)
        link.symlink_to(target)
        self.failed("Path escapes bundle")
        link.unlink()
        link.symlink_to(self.root / "missing.txt")
        self.failed("Missing or not a regular file")


if __name__ == "__main__":
    unittest.main()
