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


    def test_malformed_route_identifier_returns_errors_not_type_error(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][0]["route_id"] = []
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("route_id must be a non-empty string" in e for e in errors))


    def test_unknown_top_level_field_requires_schema_version_change(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["allow_real_credentials"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("top-level fields must match the closed v1 schema" in e for e in errors))

    def test_reason_enum_cannot_gain_an_unreviewed_verified_reason(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["reason_disposition"]["unreviewed_reason"] = "verified"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("reason_disposition keys must match the closed v1 reason enum" in e for e in errors))

    def test_allowed_disposition_enum_is_closed(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["allowed_dispositions"].append("soft_verified")
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("allowed_dispositions must enumerate exactly" in e for e in errors))

    def test_duplicate_json_object_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            validate_sa_did_profile._reject_duplicate_object_keys(
                [("production_enabled", False), ("production_enabled", True)]
            )


    def test_every_never_enable_field_is_fail_closed(self) -> None:
        for path in validate_sa_did_profile.NEVER_ENABLE_FIELDS:
            with self.subTest(path=path):
                mutated = copy.deepcopy(self.profile)
                target = mutated
                for part in path[:-1]:
                    target = target[part]
                target[path[-1]] = True
                errors = validate_sa_did_profile.validate_profile(mutated)
                self.assertTrue(any(".".join(path) in error for error in errors))

    def test_rejected_vector_cannot_reference_a_production_trust_anchor(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][1]["trust_anchor_ref"] = "dha-production"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("trust_anchor_ref must be none or an explicit test-only reference" in e for e in errors))


    def test_adversarial_scenario_requires_its_exact_reason_not_just_same_disposition(self) -> None:
        mutated = copy.deepcopy(self.profile)
        nonce_replay = next(
            vector for vector in mutated["test_vectors"]
            if vector["scenario"] == "nonce_replay"
        )
        nonce_replay["expected_reason"] = "issuer_not_trusted"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("scenario does not match its required disposition and reason" in e for e in errors))


    def test_smartphone_route_cannot_claim_a_production_ready_status(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][1]["status"] = "production_ready"
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("status must preserve the reviewed research status" in e for e in errors))

    def test_ovs_route_cannot_drop_the_dha_approval_nonclaim(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][0]["evidence_nonclaims"].remove("requires_dha_approval_and_terms")
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("evidence_nonclaims must preserve the v1 nonclaims" in e for e in errors))


    def test_policy_inputs_are_closed_and_type_checked(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["test_vectors"][0]["policy_inputs"]["issuer_trusted"] = "true"
        mutated["test_vectors"][0]["policy_inputs"]["allow_production"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("policy_inputs must match the closed synthetic input schema" in e for e in errors))
        self.assertTrue(any("policy_inputs.issuer_trusted must be a Boolean" in e for e in errors))


    def test_research_sources_are_pinned_to_official_evidence(self) -> None:
        mutated = copy.deepcopy(self.profile)
        mutated["routes"][1]["source_urls"] = [
            "https://www.gov.za/not-a-dha-digital-id-source"
        ]
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any("pinned official research sources" in e for e in errors))


    def test_trusted_synthetic_issuer_requires_test_only_anchor(self) -> None:
        mutated = copy.deepcopy(self.profile)
        vector = next(
            item for item in mutated["test_vectors"]
            if item["scenario"] == "unsupported_unpublished_profile"
        )
        self.assertEqual(vector["trust_anchor_ref"], "none")
        vector["policy_inputs"]["issuer_trusted"] = True
        errors = validate_sa_did_profile.validate_profile(mutated)
        self.assertTrue(any(
            "trusted synthetic issuer requires a test-only anchor" in error
            for error in errors
        ))


if __name__ == "__main__":
    unittest.main()
