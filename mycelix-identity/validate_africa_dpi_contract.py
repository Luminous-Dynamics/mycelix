#!/usr/bin/env python3
"""Validate the sovereignty-first Africa DPI interoperability contract.

This checks architecture invariants and release-gate closure only. It is not a
country-specific legal review, identity verifier, certification, or production
integration.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

CONTRACT_PATH = Path(__file__).with_name("AFRICA_DPI_INTEROP_CONTRACT_V1.json")

REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_id",
    "schema_version",
    "profile_id",
    "title",
    "snapshot_date",
    "maturity",
    "production_enabled",
    "role",
    "jurisdiction_model",
    "national_profiles_are_separate_artifacts",
    "country_profiles",
    "hard_invariants",
    "country_profile_status_values",
    "required_country_profile_sections",
    "release_gates",
    "references",
    "notes",
}

REQUIRED_HARD_INVARIANTS = {
    "production_enabled": False,
    "central_continental_identity_database": False,
    "single_continental_root_of_trust": False,
    "store_raw_credential_in_public_dht": False,
    "store_national_identity_number_in_public_dht": False,
    "store_biometric_template_in_public_dht": False,
    "store_replayable_presentation_in_public_dht": False,
    "identity_implies_citizenship": False,
    "identity_implies_residency_or_migration_permission": False,
    "identity_implies_service_eligibility": False,
    "identity_implies_mycelix_membership": False,
    "identity_implies_governance_or_execution_authority": False,
    "undocumented_government_endpoint_allowed": False,
    "screen_scraping_allowed": False,
    "reverse_engineered_app_protocol_allowed": False,
    "mandatory_smartphone_only_access": False,
    "non_digital_or_assisted_alternative_required": True,
    "purpose_and_legal_basis_required": True,
    "minimum_disclosure_required": True,
    "explicit_route_and_issuer_separation_required": True,
    "unknown_trust_or_status_fails_closed": True,
    "country_level_data_protection_review_required": True,
    "accountable_grievance_and_correction_path_required": True,
}

REQUIRED_COUNTRY_PROFILE_SECTIONS = {
    "national_authority_and_legal_basis",
    "issuer_and_trust_metadata",
    "credential_formats_and_protocol_versions",
    "verification_and_holder_binding",
    "validity_status_suspension_and_revocation",
    "purpose_limitation_and_minimum_disclosure",
    "relying_party_onboarding_and_contract_terms",
    "data_protection_and_retention",
    "offline_assisted_and_non_digital_access",
    "accessibility_languages_and_low_connectivity",
    "data_correction_appeal_and_grievance",
    "incident_reporting_and_key_compromise",
    "conformance_vectors_and_negative_controls",
    "evidence_commit_and_release_approval",
}

REQUIRED_STATUS_VALUES = {
    "not_assessed",
    "research_only",
    "partner_review",
    "sandbox_authorized",
    "production_authorized",
}

REQUIRED_REFERENCES = {
    "au_digital_id_interoperability_framework":
        "https://au.int/en/documents/20231211/au-interoperability-framework-digital-id",
    "au_data_policy_framework":
        "https://www.au.int/en/documents/20220728/au-data-policy-framework",
    "afcfta_digital_trade_protocol":
        "https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade",
    "smart_africa_trust_alliance":
        "https://smartafrica.org/service-post/digital-identity-for-africa/",
    "south_africa_dha_digital_id":
        "https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct",
}

REQUIRED_RELEASE_GATES = {
    "reviewed_primary_law_and_regulations",
    "written_authority_or_partner_approval",
    "official_protocol_and_trust_metadata_verified",
    "status_lifecycle_and_key_rotation_specified",
    "privacy_impact_assessment_approved",
    "data_retention_and_erasure_verified",
    "offline_and_non_digital_path_tested",
    "accessibility_and_language_review_complete",
    "security_threat_model_and_independent_review_complete",
    "positive_and_negative_conformance_tests_passed",
    "operational_incident_and_grievance_process_agreed",
    "human_approval_recorded_for_exact_release_commit",
}


def validate_contract(data: Any) -> list[str]:
    """Return structural and sovereignty-boundary errors."""
    if not isinstance(data, dict):
        return ["contract must be a JSON object"]

    errors: list[str] = []
    if set(data) != REQUIRED_TOP_LEVEL_FIELDS:
        errors.append("top-level fields must match the closed v1 contract")
    if data.get("schema_id") != "org.luminousdynamics.africa-dpi-interoperability-contract":
        errors.append("unexpected schema_id")
    if type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        errors.append("schema_version must be the integer 1")
    if data.get("profile_id") != "AFRICA-DPI-001":
        errors.append("profile_id must be AFRICA-DPI-001")
    if data.get("maturity") != "research_only":
        errors.append("maturity must remain research_only")
    if data.get("production_enabled") is not False:
        errors.append("production_enabled must be false")
    if data.get("role") != "country-adapter-and-evidence-layer-not-a-government-or-continental-trust-authority":
        errors.append("role must preserve the non-authoritative adapter boundary")
    if data.get("jurisdiction_model") != "sovereign-national-profiles-with-optional-bilateral-or-regional-federation":
        errors.append("jurisdiction_model must preserve sovereign national profiles")
    if data.get("national_profiles_are_separate_artifacts") is not True:
        errors.append("national_profiles_are_separate_artifacts must be true")

    # An aggregate architecture contract must never itself imply that a
    # national adapter has been qualified. Keep national artifacts separate.
    if data.get("country_profiles") != []:
        errors.append("country_profiles must remain empty in the continent-level contract")

    invariants = data.get("hard_invariants")
    if not isinstance(invariants, dict):
        errors.append("hard_invariants must be an object")
    else:
        if set(invariants) != set(REQUIRED_HARD_INVARIANTS):
            errors.append("hard_invariants keys must match the closed v1 invariant set")
        for key, expected in REQUIRED_HARD_INVARIANTS.items():
            if invariants.get(key) is not expected:
                errors.append(f"hard invariant must remain {str(expected).lower()}: {key}")

    statuses = data.get("country_profile_status_values")
    if (
        not isinstance(statuses, list)
        or any(not isinstance(value, str) for value in statuses)
        or len(statuses) != len(REQUIRED_STATUS_VALUES)
        or set(statuses) != REQUIRED_STATUS_VALUES
    ):
        errors.append("country_profile_status_values must match the closed v1 status enum")

    sections = data.get("required_country_profile_sections")
    if (
        not isinstance(sections, list)
        or any(not isinstance(value, str) for value in sections)
        or len(sections) != len(REQUIRED_COUNTRY_PROFILE_SECTIONS)
        or set(sections) != REQUIRED_COUNTRY_PROFILE_SECTIONS
    ):
        errors.append("required_country_profile_sections must preserve all required review areas")

    gates = data.get("release_gates")
    if not isinstance(gates, dict):
        errors.append("release_gates must be an object")
    else:
        if set(gates) != REQUIRED_RELEASE_GATES:
            errors.append("release_gates keys must match the closed v1 gate set")
        for gate in REQUIRED_RELEASE_GATES:
            if gates.get(gate) is not False:
                errors.append(f"release gate must remain closed in this contract: {gate}")

    references = data.get("references")
    if not isinstance(references, list):
        errors.append("references must be an array")
    else:
        found: dict[str, str] = {}
        for index, item in enumerate(references):
            if not isinstance(item, dict):
                errors.append(f"references[{index}] must be an object")
                continue
            if set(item) != {"id", "url", "use"}:
                errors.append(f"references[{index}] fields must match the v1 reference schema")
                continue
            ref_id, url = item.get("id"), item.get("url")
            if not isinstance(ref_id, str) or not isinstance(url, str):
                errors.append(f"references[{index}] id and url must be strings")
            elif ref_id in found:
                errors.append(f"duplicate reference id: {ref_id}")
            else:
                found[ref_id] = url
            if not isinstance(url, str) or not url.startswith("https://"):
                errors.append(f"references[{index}].url must use HTTPS")
            if not isinstance(item.get("use"), str) or not item["use"].strip():
                errors.append(f"references[{index}].use must explain the reference purpose")
        for ref_id, expected_url in REQUIRED_REFERENCES.items():
            if found.get(ref_id) != expected_url:
                errors.append(f"required official reference missing or changed: {ref_id}")

    notes = data.get("notes")
    if not isinstance(notes, list) or not notes or any(not isinstance(note, str) for note in notes):
        errors.append("notes must be a non-empty array of strings")
    else:
        combined_notes = " ".join(notes).lower()
        for phrase in (
            "not a continental identity registry",
            "consent alone is not automatically a legal basis",
            "not a digital signature",
        ):
            if phrase not in combined_notes:
                errors.append(f"notes must preserve nonclaim: {phrase}")

    return errors


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=CONTRACT_PATH)
    args = parser.parse_args()
    try:
        data = json.loads(
            args.contract.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_object_keys,
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: could not load Africa DPI contract: {exc}", file=sys.stderr)
        return 2

    errors = validate_contract(data)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"AFRICA_DPI_CONTRACT_VALIDATION=FAIL errors={len(errors)}", file=sys.stderr)
        return 1

    gates = data["release_gates"]
    print(
        "AFRICA_DPI_CONTRACT_VALIDATION=PASS "
        f"gates_closed={sum(value is False for value in gates.values())} "
        f"country_profiles={len(data['country_profiles'])} "
        "production_enabled=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
