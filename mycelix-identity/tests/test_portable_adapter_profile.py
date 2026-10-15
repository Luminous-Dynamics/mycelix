"""Adversarial contract tests for portable sovereign adapter profiles."""
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

IDENTITY_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(IDENTITY_DIR))
import validate_portable_adapter_profile  # noqa: E402


class PortableAdapterProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = json.loads(
            (IDENTITY_DIR / "PORTABLE_ADAPTER_PROFILE_TEMPLATE_V1.json").read_text(encoding="utf-8")
        )

    def test_checked_in_template_is_valid_but_unqualified(self) -> None:
        self.assertEqual(validate_portable_adapter_profile.validate_profile(self.profile), [])
        self.assertEqual(self.profile["maturity"], "research_only")
        self.assertIs(self.profile["production_enabled"], False)
        self.assertFalse(any(self.profile["release_gates"].values()))

    def test_missing_top_level_field_fails_closed(self) -> None:
        changed = copy.deepcopy(self.profile)
        del changed["privacy"]
        self.assertTrue(any("top-level fields" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_unknown_top_level_field_fails_closed(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["trusted_by_default"] = True
        self.assertTrue(any("top-level fields" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_boolean_is_not_accepted_as_schema_version_integer(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["schema_version"] = True
        self.assertTrue(any("schema_version must be the integer 1" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_production_flag_cannot_disagree_with_maturity(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = "production_authorized"
        self.assertTrue(any("production_enabled must be true exactly" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_open_production_flag_with_closed_gates_fails(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = "production_authorized"
        changed["production_enabled"] = True
        errors = validate_portable_adapter_profile.validate_profile(changed)
        self.assertTrue(any("every non-compensatory release gate" in e for e in errors))

    def test_all_open_gates_do_not_substitute_for_authority_and_protocol_evidence(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = "production_authorized"
        changed["production_enabled"] = True
        changed["release_gates"] = {key: True for key in validate_portable_adapter_profile.REQUIRED_RELEASE_GATES}
        errors = validate_portable_adapter_profile.validate_profile(changed)
        self.assertTrue(any("requires an identified authority" in e for e in errors))
        self.assertTrue(any("requires reviewed and authorized protocol" in e for e in errors))

    def test_sandbox_maturity_cannot_be_claimed_without_written_authority(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = "sandbox_authorized"
        errors = validate_portable_adapter_profile.validate_profile(changed)
        self.assertTrue(any("sandbox_authorized requires written authorization evidence" in e for e in errors))
        self.assertTrue(any("sandbox_authorized requires legal" in e for e in errors))

    def test_synthetic_route_cannot_be_promoted_to_production(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = "production_authorized"
        changed["production_enabled"] = True
        changed["release_gates"] = {key: True for key in validate_portable_adapter_profile.REQUIRED_RELEASE_GATES}
        errors = validate_portable_adapter_profile.validate_profile(changed)
        self.assertTrue(any("synthetic_only route cannot be production_authorized" in e for e in errors))

    def test_release_gates_must_be_booleans_not_truthy_values(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["release_gates"]["legal_basis_reviewed"] = 1
        self.assertTrue(any("release gate must be boolean" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_unknown_route_kind_is_rejected(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["protocol"]["route_kind"] = "guess_and_scrape"
        self.assertTrue(any("route_kind" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_non_https_verifier_endpoint_is_rejected(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["protocol"]["verifier_endpoint"] = "http://example.invalid/verify"
        self.assertTrue(any("verifier_endpoint" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_credential_material_must_not_be_stored(self) -> None:
        for field in validate_portable_adapter_profile.FORBIDDEN_IDENTITY_MATERIAL_FIELDS:
            with self.subTest(field=field):
                changed = copy.deepcopy(self.profile)
                changed["privacy"][field] = "allowed"
                self.assertTrue(any(f"{field} must remain prohibited" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_identity_cannot_become_entitlement_evidence(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["service"]["identity_is_not_entitlement_evidence"] = False
        self.assertTrue(any("identity_is_not_entitlement_evidence" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_all_prohibited_decision_boundaries_are_required(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["service"]["prohibited_decisions"].remove("benefit_or_service_eligibility")
        self.assertTrue(any("non-transferable decision boundaries" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_non_digital_and_assisted_paths_cannot_be_disabled(self) -> None:
        for field in ("non_digital_fallback_available", "assisted_path_available"):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.profile)
                changed["accessibility"][field] = False
                self.assertTrue(any(field.replace("_", " ") in e or "must be available" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_negative_retention_is_rejected(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["privacy"]["receipt_retention_days"] = -1
        self.assertTrue(any("non-negative integer" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_release_approval_gate_cannot_be_open_without_production_maturity(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["release_gates"]["human_approval_for_exact_release_commit"] = True
        self.assertTrue(any("human release approval cannot be open" in e for e in validate_portable_adapter_profile.validate_profile(changed)))

    def test_unhashable_enum_values_fail_closed_without_crashing(self) -> None:
        mutations = []
        changed = copy.deepcopy(self.profile)
        changed["maturity"] = []
        mutations.append(changed)
        changed = copy.deepcopy(self.profile)
        changed["protocol"]["route_kind"] = {}
        mutations.append(changed)
        changed = copy.deepcopy(self.profile)
        changed["service"]["lawful_basis_status"] = []
        mutations.append(changed)
        for mutated in mutations:
            with self.subTest(maturity=mutated.get("maturity")):
                self.assertTrue(validate_portable_adapter_profile.validate_profile(mutated))

    def test_non_string_prohibited_decision_value_fails_closed_without_crashing(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["service"]["prohibited_decisions"][0] = {"citizenship": True}
        self.assertTrue(validate_portable_adapter_profile.validate_profile(changed))

    def test_duplicate_json_object_keys_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate JSON object key"):
            json.loads('{"production_enabled": false, "production_enabled": true}', object_pairs_hook=validate_portable_adapter_profile._reject_duplicate_object_keys)


if __name__ == "__main__":
    unittest.main()
