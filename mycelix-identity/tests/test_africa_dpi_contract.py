"""Adversarial tests for the Africa DPI sovereignty contract."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import validate_africa_dpi_contract  # noqa: E402


class AfricaDpiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = json.loads(
            (IDENTITY_DIR / "AFRICA_DPI_INTEROP_CONTRACT_V1.json").read_text(
                encoding="utf-8"
            )
        )

    def test_checked_in_contract_is_valid_and_unqualified(self) -> None:
        self.assertEqual(validate_africa_dpi_contract.validate_contract(self.contract), [])
        self.assertEqual(self.contract["country_profiles"], [])
        self.assertFalse(self.contract["production_enabled"])

    def test_continental_production_cannot_be_enabled(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["production_enabled"] = True
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("production_enabled must be false" in error for error in errors))

    def test_no_continental_identity_database_or_root_of_trust(self) -> None:
        for field in (
            "central_continental_identity_database",
            "single_continental_root_of_trust",
        ):
            with self.subTest(field=field):
                mutated = copy.deepcopy(self.contract)
                mutated["hard_invariants"][field] = True
                errors = validate_africa_dpi_contract.validate_contract(mutated)
                self.assertTrue(any(field in error for error in errors))

    def test_sensitive_identity_material_cannot_be_written_to_public_dht(self) -> None:
        for field in (
            "store_raw_credential_in_public_dht",
            "store_national_identity_number_in_public_dht",
            "store_biometric_template_in_public_dht",
            "store_replayable_presentation_in_public_dht",
        ):
            with self.subTest(field=field):
                mutated = copy.deepcopy(self.contract)
                mutated["hard_invariants"][field] = True
                errors = validate_africa_dpi_contract.validate_contract(mutated)
                self.assertTrue(any(field in error for error in errors))

    def test_identity_does_not_create_entitlement_or_membership(self) -> None:
        for field in (
            "identity_implies_citizenship",
            "identity_implies_residency_or_migration_permission",
            "identity_implies_service_eligibility",
            "identity_implies_mycelix_membership",
            "identity_implies_governance_or_execution_authority",
        ):
            with self.subTest(field=field):
                mutated = copy.deepcopy(self.contract)
                mutated["hard_invariants"][field] = True
                errors = validate_africa_dpi_contract.validate_contract(mutated)
                self.assertTrue(any(field in error for error in errors))

    def test_country_entries_cannot_implicitly_claim_qualification(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["country_profiles"] = [{"country_code": "ZA", "status": "production_authorized"}]
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("country_profiles must remain empty" in error for error in errors))

    def test_unknown_country_profile_status_is_not_added_silently(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["country_profile_status_values"].append("mostly_ready")
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("closed v1 status enum" in error for error in errors))

    def test_all_release_gates_must_remain_closed_until_authorized(self) -> None:
        for gate in sorted(self.contract["release_gates"]):
            with self.subTest(gate=gate):
                mutated = copy.deepcopy(self.contract)
                mutated["release_gates"][gate] = True
                errors = validate_africa_dpi_contract.validate_contract(mutated)
                self.assertTrue(any(gate in error for error in errors))

    def test_offline_access_and_data_protection_review_cannot_be_dropped(self) -> None:
        for field in (
            "non_digital_or_assisted_alternative_required",
            "country_level_data_protection_review_required",
            "accountable_grievance_and_correction_path_required",
            "minimum_disclosure_required",
        ):
            with self.subTest(field=field):
                mutated = copy.deepcopy(self.contract)
                mutated["hard_invariants"][field] = False
                errors = validate_africa_dpi_contract.validate_contract(mutated)
                self.assertTrue(any(field in error for error in errors))

    def test_boolean_schema_version_is_not_accepted_as_integer_one(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["schema_version"] = True
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("schema_version must be the integer 1" in error for error in errors))

    def test_required_country_review_sections_cannot_disappear(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["required_country_profile_sections"].remove("data_protection_and_retention")
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("preserve all required review areas" in error for error in errors))

    def test_unknown_top_level_field_requires_contract_revision(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["allow_unverified_country_profiles"] = True
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("closed v1 contract" in error for error in errors))

    def test_reference_set_cannot_drop_au_or_regional_governance_sources(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["references"] = [
            item for item in mutated["references"] if item["id"] != "au_digital_id_interoperability_framework"
        ]
        errors = validate_africa_dpi_contract.validate_contract(mutated)
        self.assertTrue(any("required official reference missing or changed" in error for error in errors))

    def test_duplicate_json_object_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            validate_africa_dpi_contract._reject_duplicate_object_keys(
                [("production_enabled", False), ("production_enabled", True)]
            )


if __name__ == "__main__":
    unittest.main()
