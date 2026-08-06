#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from driver_release_manifest import release_identity, reviewed_capabilities_hash, verify_driver_release  # noqa: E402
from evidence_manifest import canonical_json_bytes, manifest_hash, sha256_b64  # noqa: E402
from release_signer_policy import POLICY_SCHEMA, derived_public_key_der  # noqa: E402


class DriverReleaseManifestTests(unittest.TestCase):
    def keypair(self, root: Path, name: str) -> tuple[Path, Path]:
        private = root / f"{name}.pem"; public = root / f"{name}.pub.pem"
        subprocess.run(["openssl", "genpkey", "-algorithm", "ED25519", "-out", str(private)], check=True, capture_output=True)
        subprocess.run(["openssl", "pkey", "-in", str(private), "-pubout", "-out", str(public)], check=True, capture_output=True)
        return private, public

    def fixture(self, root: Path) -> tuple[Path, Path, Path, Path, Path, Path, dict]:
        root_private, root_public = self.keypair(root, "root")
        review_private, _ = self.keypair(root, "review")
        review_der = derived_public_key_der(review_private)
        root_der = subprocess.run(["openssl", "pkey", "-pubin", "-in", str(root_public), "-outform", "DER"], check=True, capture_output=True).stdout
        now = time.time_ns() // 1000
        policy = {"schema": POLICY_SCHEMA, "policy_id": sha256_b64(b"policy"), "policy_version": 1, "created_at_unix_micros": 1, "valid_from_unix_micros": 1, "valid_until_unix_micros": now + 10_000_000_000, "root_public_key_sha256": sha256_b64(root_der), "keys": [{"key_id": sha256_b64(review_der), "public_key_der_b64": base64.b64encode(review_der).decode("ascii"), "roles": ["driver-review"], "valid_from_unix_micros": 1, "valid_until_unix_micros": now + 9_000_000_000, "revoked_at_unix_micros": None, "revocation_mode": "prospective"}]}
        policy_path, policy_sig = root / "policy.json", root / "policy.sig"
        policy_path.write_bytes(canonical_json_bytes(policy) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(root_private), "-in", str(policy_path), "-out", str(policy_sig)], check=True, capture_output=True)
        driver = root / "driver.py"
        build_hash, lock_hash = sha256_b64(b"build"), sha256_b64(b"lock")
        placeholder = sha256_b64(b"placeholder")
        capabilities = {"driver_protocol": "mycelix-real-conductor-driver-v1", "driver_release_hash": placeholder, "conductor_count": 3, "dna_hash": "dna", "happ_hash": "happ", "holochain_version": "0.6.0", "agents": ["verifier", "prover_a", "prover_b"], "conductors": []}
        driver.write_text("#!/usr/bin/env python3\nimport json\nprint(json.dumps(" + repr(capabilities) + ",sort_keys=True,separators=(',',':')))\n")
        driver.chmod(0o755)
        executable_hash = sha256_b64(driver.read_bytes())
        release_hash = release_identity(executable_hash, reviewed_capabilities_hash(capabilities), "commit", "tree", build_hash, lock_hash, "0.6.0")
        capabilities["driver_release_hash"] = release_hash
        # Rebuild until executable and embedded release hash reach a fixed point is impossible; use a wrapper and reviewed payload.
        payload = root / "capabilities.json"
        payload.write_bytes(canonical_json_bytes(capabilities) + b"\n")
        driver.write_text("#!/usr/bin/env python3\nfrom pathlib import Path\nprint((Path(__file__).with_name('capabilities.json')).read_text(),end='')\n")
        driver.chmod(0o755)
        executable_hash = sha256_b64(driver.read_bytes())
        release_hash = release_identity(executable_hash, reviewed_capabilities_hash(capabilities), "commit", "tree", build_hash, lock_hash, "0.6.0")
        capabilities["driver_release_hash"] = release_hash
        payload.write_bytes(canonical_json_bytes(capabilities) + b"\n")
        # One more executable hash is stable because the wrapper no longer embeds the release hash.
        assert executable_hash == sha256_b64(driver.read_bytes())
        manifest = {
            "schema": "mycelix-real-conductor-driver-release-v1", "signature_algorithm": "ed25519",
            "signer_key_id": sha256_b64(review_der), "signer_policy_version": 1,
            "signer_policy_file_sha256": sha256_b64(policy_path.read_bytes()), "signer_policy_canonical_hash": manifest_hash(policy),
            "driver_protocol": "mycelix-real-conductor-driver-v1", "adapter_protocol": "mycelix-conductor-scenario-adapter-v3",
            "scenario_contract_hash": __import__('driver_release_manifest').contract_hash(ROOT / 'settlement_scenarios_v2.json'),
            "driver_release_hash": release_hash, "driver_executable_sha256": executable_hash,
            "capabilities_canonical_hash": reviewed_capabilities_hash(capabilities), "source_git_commit": "commit", "source_git_tree": "tree",
            "build_recipe_sha256": build_hash, "lockfile_sha256": lock_hash, "holochain_version": "0.6.0",
            "created_at_unix_micros": now, "valid_from_unix_micros": now, "valid_until_unix_micros": now + 1_000_000_000,
        }
        manifest_path, manifest_sig = root / "driver-release.json", root / "driver-release.sig"
        manifest_path.write_bytes(canonical_json_bytes(manifest) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(review_private), "-in", str(manifest_path), "-out", str(manifest_sig)], check=True, capture_output=True)
        return driver, manifest_path, manifest_sig, policy_path, policy_sig, root_public, capabilities

    def test_reviewed_driver_verifies(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            driver, manifest, sig, policy, policy_sig, root_public, capabilities = self.fixture(Path(temporary))
            verify_driver_release(manifest, sig, policy, policy_sig, root_public, driver, capabilities)

    def test_driver_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            driver, manifest, sig, policy, policy_sig, root_public, capabilities = self.fixture(Path(temporary))
            driver.write_text(driver.read_text() + "\n# tamper\n")
            with self.assertRaises(Exception):
                verify_driver_release(manifest, sig, policy, policy_sig, root_public, driver, capabilities)

    def test_capability_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            driver, manifest, sig, policy, policy_sig, root_public, capabilities = self.fixture(Path(temporary))
            changed = dict(capabilities); changed["holochain_version"] = "different"
            with self.assertRaises(Exception):
                verify_driver_release(manifest, sig, policy, policy_sig, root_public, driver, changed)


if __name__ == "__main__":
    unittest.main()
