#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import os
import subprocess
import tempfile
import time
import unittest
import sys
from pathlib import Path

CONDUCTOR_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(CONDUCTOR_ROOT))

from evidence_manifest import canonical_json_bytes, sha256_b64
from release_signer_policy import POLICY_SCHEMA, derived_public_key_der
from test_holochain_real_driver import CONTROL_SOURCE

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/create_reviewed_driver_release.sh"
DRIVER = ROOT / "tests/conductor/holochain_real_driver.py"


class ReviewedDriverReleaseWorkflowTests(unittest.TestCase):
    def keypair(self, root: Path, name: str) -> tuple[Path, Path]:
        private = root / f"{name}.pem"; public = root / f"{name}.pub.pem"
        subprocess.run(["openssl", "genpkey", "-algorithm", "ED25519", "-out", str(private)], check=True, capture_output=True)
        subprocess.run(["openssl", "pkey", "-in", str(private), "-pubout", "-out", str(public)], check=True, capture_output=True)
        return private, public

    def policy(self, root: Path) -> tuple[Path, Path, Path, Path]:
        root_private, root_public = self.keypair(root, "root")
        review_private, _ = self.keypair(root, "review")
        review_der = derived_public_key_der(review_private)
        root_der = subprocess.run(["openssl", "pkey", "-pubin", "-in", str(root_public), "-outform", "DER"], check=True, capture_output=True).stdout
        now = time.time_ns() // 1000
        policy = {
            "schema": POLICY_SCHEMA,
            "policy_id": sha256_b64(b"driver-workflow-policy"),
            "policy_version": 1,
            "created_at_unix_micros": 1,
            "valid_from_unix_micros": 1,
            "valid_until_unix_micros": now + 20_000_000_000,
            "root_public_key_sha256": sha256_b64(root_der),
            "keys": [{
                "key_id": sha256_b64(review_der),
                "public_key_der_b64": base64.b64encode(review_der).decode("ascii"),
                "roles": ["driver-review"],
                "valid_from_unix_micros": 1,
                "valid_until_unix_micros": now + 10_000_000_000,
                "revoked_at_unix_micros": None,
                "revocation_mode": "prospective",
            }],
        }
        policy_path = root / "policy.json"; policy_sig = root / "policy.sig"
        policy_path.write_bytes(canonical_json_bytes(policy) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(root_private), "-in", str(policy_path), "-out", str(policy_sig)], check=True, capture_output=True)
        return review_private, policy_path, policy_sig, root_public

    def test_script_creates_a_verifiable_release(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            control = root / "control.py"; control.write_text(CONTROL_SOURCE); control.chmod(0o755)
            review_key, policy, policy_sig, root_public = self.policy(root)
            config = root / "driver-config.json"; manifest = root / "driver-release.json"; signature = root / "driver-release.sig"
            env = os.environ.copy()
            env.update({
                "MYCELIX_CONDUCTOR_DRIVER": str(DRIVER),
                "MYCELIX_HOLOCHAIN_CONTROL": str(control),
                "MYCELIX_HOLOCHAIN_DRIVER_CONFIG_OUTPUT": str(config),
                "MYCELIX_DRIVER_RELEASE_MANIFEST_OUTPUT": str(manifest),
                "MYCELIX_DRIVER_RELEASE_SIGNATURE_OUTPUT": str(signature),
                "MYCELIX_DRIVER_REVIEW_SIGNING_KEY": str(review_key),
                "MYCELIX_SIGNER_POLICY": str(policy),
                "MYCELIX_SIGNER_POLICY_SIGNATURE": str(policy_sig),
                "MYCELIX_POLICY_ROOT_PUBLIC_KEY": str(root_public),
                "MYCELIX_MINIMUM_SIGNER_POLICY_VERSION": "1",
                "MYCELIX_DRIVER_RELEASE_VALIDITY_SECONDS": "600",
                "MYCELIX_CONDUCTOR_STEP_TIMEOUT_SECONDS": "30",
            })
            result = subprocess.run([str(SCRIPT)], cwd=ROOT, env=env, text=True, capture_output=True, timeout=90, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(config.is_file() and manifest.is_file() and signature.is_file())
            value = json.loads(manifest.read_text())
            self.assertEqual(value["holochain_version"], "0.6.1-test")
            self.assertEqual(value["driver_executable_sha256"], sha256_b64(DRIVER.read_bytes()))
            verify_env = env.copy(); verify_env["MYCELIX_HOLOCHAIN_DRIVER_CONFIG"] = str(config)
            verify = subprocess.run([
                "python3", "tests/conductor/driver_release_manifest.py", "--verify",
                "--manifest", str(manifest), "--signature", str(signature), "--driver", str(DRIVER),
                "--signer-policy", str(policy), "--signer-policy-signature", str(policy_sig),
                "--policy-root-public-key", str(root_public), "--minimum-policy-version", "1",
            ], cwd=ROOT, env=verify_env, text=True, capture_output=True, timeout=60, check=False)
            self.assertEqual(verify.returncode, 0, verify.stderr)
            self.assertIn("driver release manifest: PASS", verify.stdout)


if __name__ == "__main__":
    unittest.main()
