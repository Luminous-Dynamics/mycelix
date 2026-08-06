#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from evidence_manifest import canonical_json_bytes, sha256_b64
from interface_signer_policy import (
    INTERFACE_KEY_POLICY_SCHEMA,
    InterfaceKeyPolicyError,
    authorized_certificate,
    certificate_hash,
    policy_id_payload,
    validate_policy_shape,
    verify_interface_key_policy,
)
from release_signer_policy import POLICY_SCHEMA, derived_public_key_der


class InterfaceSignerPolicyTests(unittest.TestCase):
    def keypair(self, root: Path, name: str) -> tuple[Path, Path]:
        private = root / f"{name}.pem"
        public = root / f"{name}.pub.pem"
        subprocess.run(["openssl", "genpkey", "-algorithm", "ED25519", "-out", str(private)], check=True, capture_output=True)
        subprocess.run(["openssl", "pkey", "-in", str(private), "-pubout", "-out", str(public)], check=True, capture_output=True)
        return private, public

    def fixture(self, root: Path, *, key_epoch: int = 2, revoked: int | None = None, mode: str = "prospective"):
        root_private, root_public = self.keypair(root, "root")
        review_private, _ = self.keypair(root, "review")
        review_der = derived_public_key_der(review_private)
        root_der = subprocess.run(["openssl", "pkey", "-pubin", "-in", str(root_public), "-outform", "DER"], check=True, capture_output=True).stdout
        signer_policy = {
            "schema": POLICY_SCHEMA,
            "policy_id": sha256_b64(b"release-policy"),
            "policy_version": 4,
            "created_at_unix_micros": 1,
            "valid_from_unix_micros": 2,
            "valid_until_unix_micros": 100_000,
            "root_public_key_sha256": sha256_b64(root_der),
            "keys": [{
                "key_id": sha256_b64(review_der),
                "public_key_der_b64": base64.b64encode(review_der).decode("ascii"),
                "roles": ["driver-review"],
                "valid_from_unix_micros": 2,
                "valid_until_unix_micros": 90_000,
                "revoked_at_unix_micros": None,
                "revocation_mode": "prospective",
            }],
        }
        signer_path = root / "signer-policy.json"
        signer_sig = root / "signer-policy.sig"
        signer_path.write_bytes(canonical_json_bytes(signer_policy) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(root_private), "-in", str(signer_path), "-out", str(signer_sig)], check=True, capture_output=True)
        signer_file_hash = sha256_b64(signer_path.read_bytes())
        signer_canonical_hash = sha256_b64(canonical_json_bytes(signer_policy))
        current_public_key = base64.b64encode(bytes(range(32))).decode("ascii")
        previous_public_key = base64.b64encode(bytes(range(32, 64))).decode("ascii")
        if key_epoch == 1:
            keys = [{
                "conductor_id": "verifier",
                "agent_pub_key": "uhCAk-verifier",
                "attestation_signer_pub_key_b64": current_public_key,
                "key_epoch": 1,
                "valid_from_unix_micros": 40,
                "valid_until_unix_micros": 70_000,
                "revoked_at_unix_micros": revoked,
                "revocation_mode": mode,
                "predecessor_certificate_hash": None,
                "certificate_hash": "",
            }]
        else:
            previous = {
                "conductor_id": "verifier",
                "agent_pub_key": "uhCAk-verifier",
                "attestation_signer_pub_key_b64": previous_public_key,
                "key_epoch": 1,
                "valid_from_unix_micros": 40,
                "valid_until_unix_micros": 90,
                "revoked_at_unix_micros": None,
                "revocation_mode": "prospective",
                "predecessor_certificate_hash": None,
                "certificate_hash": "",
            }
            previous["certificate_hash"] = certificate_hash({}, previous)
            current = {
                "conductor_id": "verifier",
                "agent_pub_key": "uhCAk-verifier",
                "attestation_signer_pub_key_b64": current_public_key,
                "key_epoch": 2,
                "valid_from_unix_micros": 90,
                "valid_until_unix_micros": 70_000,
                "revoked_at_unix_micros": revoked,
                "revocation_mode": mode,
                "predecessor_certificate_hash": previous["certificate_hash"],
                "certificate_hash": "",
            }
            current["certificate_hash"] = certificate_hash({}, current)
            keys = [previous, current]
        policy = {
            "schema": INTERFACE_KEY_POLICY_SCHEMA,
            "policy_id": "",
            "policy_version": 7,
            "created_at_unix_micros": 10,
            "valid_from_unix_micros": 20,
            "valid_until_unix_micros": 80_000,
            "signer_policy_file_sha256": signer_file_hash,
            "signer_policy_canonical_hash": signer_canonical_hash,
            "signer_policy_version": 4,
            "driver_review_key_id": sha256_b64(review_der),
            "issued_at_unix_micros": 30,
            "keys": keys,
        }
        for entry in policy["keys"]:
            if not entry["certificate_hash"]:
                entry["certificate_hash"] = certificate_hash(policy, entry)
        policy["policy_id"] = sha256_b64(canonical_json_bytes(policy_id_payload(policy)))
        policy_path = root / "interface-policy.json"
        policy_sig = root / "interface-policy.sig"
        policy_path.write_bytes(canonical_json_bytes(policy) + b"\n")
        subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(review_private), "-in", str(policy_path), "-out", str(policy_sig)], check=True, capture_output=True)
        return policy_path, policy_sig, signer_path, signer_sig, root_public, policy

    def test_root_governed_policy_verifies(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            paths = self.fixture(Path(temporary))
            policy = verify_interface_key_policy(*paths[:5], 4, 7)
            cert = authorized_certificate(policy, "verifier", "uhCAk-verifier", 100, 2)
            self.assertEqual(cert["key_epoch"], 2)

    def test_epoch_rollback_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            paths = self.fixture(Path(temporary), key_epoch=2)
            policy = verify_interface_key_policy(*paths[:5], 4, 7)
            with self.assertRaises(Exception):
                authorized_certificate(policy, "verifier", "uhCAk-verifier", 100, 3)

    def test_retroactive_revocation_invalidates_old_use(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            paths = self.fixture(Path(temporary), revoked=200, mode="retroactive")
            policy = verify_interface_key_policy(*paths[:5], 4, 7)
            with self.assertRaises(InterfaceKeyPolicyError):
                authorized_certificate(policy, "verifier", "uhCAk-verifier", 100, 2)

    def test_policy_tampering_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy_path, sig, signer, signer_sig, root_public, _ = self.fixture(Path(temporary))
            value = json.loads(policy_path.read_text())
            value["keys"][-1]["key_epoch"] = 3
            policy_path.write_bytes(canonical_json_bytes(value) + b"\n")
            with self.assertRaises(Exception):
                verify_interface_key_policy(policy_path, sig, signer, signer_sig, root_public, 4, 7)

    def test_certificate_hash_detects_key_substitution(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            *_, policy = self.fixture(Path(temporary))
            policy = json.loads(json.dumps(policy))
            policy["keys"][-1]["attestation_signer_pub_key_b64"] = base64.b64encode(b"z" * 32).decode("ascii")
            with self.assertRaises(Exception):
                validate_policy_shape(policy)

    def test_rotation_chain_authorizes_old_then_new_epoch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            paths = self.fixture(Path(temporary), key_epoch=2)
            policy = verify_interface_key_policy(*paths[:5], 4, 7)
            self.assertEqual(authorized_certificate(policy, "verifier", "uhCAk-verifier", 50, 1)["key_epoch"], 1)
            self.assertEqual(authorized_certificate(policy, "verifier", "uhCAk-verifier", 100, 2)["key_epoch"], 2)

    def test_broken_predecessor_chain_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            *_, policy = self.fixture(Path(temporary), key_epoch=2)
            policy = json.loads(json.dumps(policy))
            policy["keys"][1]["predecessor_certificate_hash"] = sha256_b64(b"wrong-predecessor")
            policy["keys"][1]["certificate_hash"] = certificate_hash(policy, policy["keys"][1])
            policy["policy_id"] = sha256_b64(canonical_json_bytes(policy_id_payload(policy)))
            with self.assertRaises(Exception):
                validate_policy_shape(policy)

    def test_overlapping_key_epochs_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            *_, policy = self.fixture(Path(temporary), key_epoch=2)
            policy = json.loads(json.dumps(policy))
            policy["keys"][0]["valid_until_unix_micros"] = 100
            policy["keys"][0]["certificate_hash"] = certificate_hash(policy, policy["keys"][0])
            policy["keys"][1]["predecessor_certificate_hash"] = policy["keys"][0]["certificate_hash"]
            policy["keys"][1]["certificate_hash"] = certificate_hash(policy, policy["keys"][1])
            policy["policy_id"] = sha256_b64(canonical_json_bytes(policy_id_payload(policy)))
            with self.assertRaises(Exception):
                validate_policy_shape(policy)


if __name__ == "__main__":
    unittest.main()
