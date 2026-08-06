#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from holochain_real_driver import CONFIG_SCHEMA
from interface_signer_policy import verify_interface_key_policy
from test_interface_signer_policy import InterfaceSignerPolicyTests
from verify_interface_policy_binding import verify_binding


class InterfacePolicyBindingTests(unittest.TestCase):
    def fixture(self, root: Path, *, revoked: int | None = None, mode: str = "prospective") -> dict:
        policy_path, policy_sig, signer_path, signer_sig, root_public, _ = InterfaceSignerPolicyTests().fixture(
            root, key_epoch=2, revoked=revoked, mode=mode,
        )
        policy = verify_interface_key_policy(policy_path, policy_sig, signer_path, signer_sig, root_public, 4, 7)
        certificate = policy["keys"][-1]
        conductor = {
            "id": "verifier",
            "agent_pub_key": "uhCAk-verifier",
            "attestation_signer_pub_key_b64": certificate["attestation_signer_pub_key_b64"],
            "attestation_key_policy_id": policy["policy_id"],
            "attestation_key_policy_version": policy["policy_version"],
            "attestation_key_epoch": certificate["key_epoch"],
            "attestation_key_certificate_hash": certificate["certificate_hash"],
            "attestation_key_valid_from_unix_micros": certificate["valid_from_unix_micros"],
            "attestation_key_valid_until_unix_micros": certificate["valid_until_unix_micros"],
            "attestation_key_revoked_at_unix_micros": certificate["revoked_at_unix_micros"],
            "attestation_key_revocation_mode": certificate["revocation_mode"],
        }
        config = root / "driver-config.json"
        config.write_text(json.dumps({"schema": CONFIG_SCHEMA, "conductors": [conductor]}, sort_keys=True), encoding="utf-8")
        floors = root / "minimum-epochs.json"
        floors.write_text(json.dumps({"verifier": 2}, sort_keys=True), encoding="utf-8")
        return {
            "driver_config": config,
            "interface_policy": policy_path,
            "interface_policy_signature": policy_sig,
            "signer_policy": signer_path,
            "signer_policy_signature": signer_sig,
            "policy_root_public_key": root_public,
            "minimum_signer_policy_version": 4,
            "minimum_interface_policy_version": 7,
            "minimum_key_epochs": floors,
            "evaluation_time_unix_micros": 100,
        }

    def test_current_policy_binding_verifies(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            value = verify_binding(**self.fixture(Path(temporary)))
            self.assertEqual(value["active_certificates"][0]["key_epoch"], 2)
            self.assertEqual(len(value["binding_hash"]), 44)

    def test_protected_epoch_floor_rejects_rollback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self.fixture(root)
            inputs["minimum_key_epochs"].write_text(json.dumps({"verifier": 3}), encoding="utf-8")
            with self.assertRaises(Exception):
                verify_binding(**inputs)

    def test_driver_key_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self.fixture(root)
            value = json.loads(inputs["driver_config"].read_text())
            value["conductors"][0]["attestation_key_epoch"] = 1
            inputs["driver_config"].write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
            with self.assertRaises(Exception):
                verify_binding(**inputs)

    def test_retroactively_revoked_certificate_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            inputs = self.fixture(Path(temporary), revoked=200, mode="retroactive")
            with self.assertRaises(Exception):
                verify_binding(**inputs)

    def test_floor_file_must_name_exact_conductor_set(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self.fixture(root)
            inputs["minimum_key_epochs"].write_text(json.dumps({"other": 2}), encoding="utf-8")
            with self.assertRaises(Exception):
                verify_binding(**inputs)


if __name__ == "__main__":
    unittest.main()
