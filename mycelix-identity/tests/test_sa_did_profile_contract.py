"""Regression tests for the SA-DID-001 structural contract validator."""
from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path
import sys

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import validate_sa_did_profile  # noqa: E402


class SouthAfricanDigitalIdProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = json.loads(
            (IDENTITY_DIR / "SA_DID_PROFILE_V1.json").read_text(encoding="utf-8")
        )

    def test_checked_in_profile_is_valid_research_contract(self) -> None:
        self.assertEqual(validate_sa_did_profile.validate_profile(self.profile), [])

    def test_production_activation_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["production_enabled"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("production_enabled must be false" in e for e in errors))

    def test_dha_route_cannot_be_enabled_by_metadata_only(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][0]["production_enabled"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("routes[0].production_enabled must be false" in e for e in errors))

    def test_a_real_or_non_synthetic_credential_cannot_enter_corpus(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][0]["synthetic_fixture"] = False
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("synthetic_fixture=true" in e for e in errors))

    def test_verified_result_requires_test_only_anchor_and_known_scenario(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][0]["trust_anchor_ref"] = "dha-production"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("test-only trust anchor" in e for e in errors))

    def test_unknown_status_cannot_be_mapped_to_verified(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["reason_disposition"]["status_unknown"] = "verified"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("wrong or missing disposition mapping for reason: status_unknown" in e for e in errors))

    def test_sensitive_receipt_fields_cannot_be_removed_from_prohibition_list(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["receipt_policy"]["prohibited_output_keys"].remove("national_identity_number")
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("prohibited_output_keys" in e for e in errors))

    def test_identity_does_not_grant_authority(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["receipt_policy"]["identity_implies_authority"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("identity_implies_authority" in e for e in errors))

    def test_required_adversarial_vector_cannot_disappear(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"] = [
            vector for vector in mutated["test_vectors"]
            if vector["scenario"] != "nonce_replay"
        ]
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("missing required scenarios" in e for e in errors))

    def test_duplicate_vector_ids_are_rejected(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][1]["id"] = mutated["test_vectors"][0]["id"]
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("duplicate vector id" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
