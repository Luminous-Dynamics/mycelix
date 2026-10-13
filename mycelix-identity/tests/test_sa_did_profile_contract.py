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


    def test_boolean_schema_version_is_not_accepted_as_integer_one(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["schema_version"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("schema_version must be the integer 1" in e for e in errors))

    def test_malformed_reason_disposition_type_returns_errors_not_type_error(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["reason_disposition"]["status_unknown"] = []
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("unsupported disposition for reason status_unknown" in e for e in errors))

    def test_non_string_prohibited_receipt_key_is_rejected_without_crashing(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["receipt_policy"]["prohibited_output_keys"].append({"unexpected": "object"})
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("prohibited_output_keys" in e for e in errors))

    def test_unhashable_vector_disposition_is_rejected_without_crashing(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][1]["expected_disposition"] = []
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("expected_disposition is invalid" in e for e in errors))


    def test_ovs_and_wallet_routes_cannot_exchange_method_semantics(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][0]["method_kind"] = "holder_presented_digital_credential"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("method_kind must preserve the distinct route semantics" in e for e in errors))

    def test_wallet_profile_cannot_be_populated_before_official_review(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][1]["protocol_profile_id"] = "openid4vp-assumed"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("protocol_profile_id must remain null" in e for e in errors))

    def test_unrecognized_receipt_policy_fields_require_schema_version_change(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["receipt_policy"]["store_face_template"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("closed v1 policy schema" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
