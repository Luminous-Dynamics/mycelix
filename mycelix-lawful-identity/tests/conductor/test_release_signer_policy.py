#!/usr/bin/env python3
from __future__ import annotations

import base64
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from evidence_manifest import canonical_json_bytes, sha256_b64  # noqa: E402
from release_signer_policy import (  # noqa: E402
    POLICY_SCHEMA,
    PolicyError,
    authorized_key,
    derived_public_key_der,
    verify_policy,
)


class ReleaseSignerPolicyTests(unittest.TestCase):
    def keypair(self, root: Path, name: str) -> tuple[Path, Path]:
        private = root / f"{name}.pem"
        public = root / f"{name}.pub.pem"
        subprocess.run(["openssl", "genpkey", "-algorithm", "ED25519", "-out", str(private)], check=True, capture_output=True)
        subprocess.run(["openssl", "pkey", "-in", str(private), "-pubout", "-out", str(public)], check=True, capture_output=True)
        return private, public

    def fixture(self, root: Path, *, revoked: int | None = None, mode: str = "prospective") -> tuple[Path, Path, Path, Path]:
        root_private, root_public = self.keypair(root, "root")
        signer_private, _signer_public = self.keypair(root, "release")
        der = derived_public_key_der(signer_private)
        policy = {
            "schema": POLICY_SCHEMA,
            "policy_id": sha256_b64(b"policy-id"),
            "policy_version": 1,
            "created_at_unix_micros": 1,
            "valid_from_unix_micros": 2,
            "valid_until_unix_micros": 10_000,
            "root_public_key_sha256": sha256_b64(subprocess.run(["openssl", "pkey", "-pubin", "-in", str(root_public), "-outform", "DER"], check=True, capture_output=True).stdout),
            "keys": [{
                "key_id": sha256_b64(der),
                "public_key_der_b64": base64.b64encode(der).decode("ascii"),
                "roles": ["release-evidence", "driver-review"],
                "valid_from_unix_micros": 2,
                "valid_until_unix_micros": 9_000,
                "revoked_at_unix_micros": revoked,
                "revocation_mode": mode,
            }],
        }
        policy_path = root / "policy.json"
        signature = root / "policy.sig"
        policy_path.write_bytes(canonical_json_bytes(policy) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(root_private), "-in", str(policy_path), "-out", str(signature)], check=True, capture_output=True)
        return policy_path, signature, root_public, signer_private

    def test_policy_and_roles_verify(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy_path, signature, root_public, signer_private = self.fixture(Path(temporary))
            policy = verify_policy(policy_path, signature, root_public)
            authorized_key(policy, "release-evidence", sha256_b64(derived_public_key_der(signer_private)), 100)

    def test_prospective_revocation_rejects_late_signature(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy_path, signature, root_public, signer_private = self.fixture(Path(temporary), revoked=100)
            policy = verify_policy(policy_path, signature, root_public)
            authorized_key(policy, "release-evidence", sha256_b64(derived_public_key_der(signer_private)), 99)
            with self.assertRaises(Exception):
                authorized_key(policy, "release-evidence", sha256_b64(derived_public_key_der(signer_private)), 100)

    def test_retroactive_revocation_rejects_earlier_signature(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy_path, signature, root_public, signer_private = self.fixture(Path(temporary), revoked=100, mode="retroactive")
            policy = verify_policy(policy_path, signature, root_public)
            with self.assertRaises(PolicyError):
                authorized_key(policy, "release-evidence", sha256_b64(derived_public_key_der(signer_private)), 50)

    def test_policy_tampering_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy_path, signature, root_public, _signer_private = self.fixture(Path(temporary))
            policy_path.write_bytes(policy_path.read_bytes() + b" ")
            with self.assertRaises(Exception):
                verify_policy(policy_path, signature, root_public)


if __name__ == "__main__":
    unittest.main()
