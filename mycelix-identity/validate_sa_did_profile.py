#!/usr/bin/env python3
"""Validate the SA-DID-001 research contract and synthetic expected-outcome corpus.

This checks contract metadata and adversarial-corpus completeness only. It does
not parse credentials, verify signatures, contact DHA, or qualify production use.
Uses Python's standard library only.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PROFILE_PATH = Path(__file__).with_name("SA_DID_PROFILE_V1.json")

REQUIRED_ROUTES = {"dha_ovs", "dha_smartphone_digital_id"}
REQUIRED_ROUTE_METHOD_BY_ID = {
    "dha_ovs": "server_side_register_verification",
    "dha_smartphone_digital_id": "holder_presented_digital_credential",
}
REQUIRED_ROUTE_STATUS_BY_ID = {
    "dha_ovs": "registration_and_contract_required",
    "dha_smartphone_digital_id": "official_relying_party_profile_not_established_in_reviewed_public_materials",
}
REQUIRED_ROUTE_NONCLAIMS_BY_ID = {
    "dha_ovs": [
        "not_a_user_held_wallet_credential",
        "does_not_establish_mycelix_membership",
        "requires_dha_approval_and_terms",
    ],
    "dha_smartphone_digital_id": [
        "not_implemented",
        "not_dha_certified",
        "candidate_standards_are_not_evidence_of_dha_protocol_support",
    ],
}
REQUIRED_ROUTE_SOURCE_URLS_BY_ID = {
    "dha_ovs": [
        "https://www.gov.za/news/media-statements/home-affairs-corrects-unsustainable-under-pricing-national-population",
        "https://www.gov.za/news/media-statements/minister-leon-schreiber-welcomes-withdrawal-act-court-application-against",
    ],
    "dha_smartphone_digital_id": [
        "https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct",
        "https://www.gov.za/news/media-statements/minister-leon-schreiber-invites-public-comment-draft-digital-identity",
        "https://www.dha.gov.za/images/AnnualReports/DHA-APP-2026-27_260326.pdf",
        "https://www.parliament.gov.za/storage/app/media/Docs/atc/01ls62wghipk7yjq7tmng272dk6bqkozmm.pdf",
    ],
}
OFFICIAL_SOURCE_PREFIXES = (
    "https://www.gov.za/",
    "https://www.dha.gov.za/",
    "https://www.parliament.gov.za/",
)
REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_id", "schema_version", "profile_id", "title", "snapshot_date",
    "maturity", "production_enabled", "authority", "routes",
    "allowed_dispositions", "reason_disposition", "receipt_policy",
    "test_vectors", "notes",
}
REQUIRED_AUTHORITY_FIELDS = {
    "government_issuer",
    "official_issuer_trust_anchor_configured",
    "published_wallet_relying_party_profile_reviewed",
    "dha_relying_party_authorization_confirmed",
    "notes",
}
REQUIRED_ROUTE_FIELDS = {
    "dha_ovs": {
        "route_id", "method_kind", "status", "production_enabled",
        "public_protocol_spec_in_this_repository", "source_urls",
        "public_onboarding_contact", "evidence_nonclaims",
    },
    "dha_smartphone_digital_id": {
        "route_id", "method_kind", "status", "production_enabled",
        "protocol_profile_id", "official_issuer_trust_anchor_configured",
        "status_or_revocation_profile_configured", "source_urls",
        "evidence_nonclaims",
    },
}
REQUIRED_VECTOR_FIELDS = {
    "id", "scenario", "synthetic_fixture", "production_eligible",
    "trust_anchor_ref", "expected_disposition", "expected_reason", "policy_inputs",
}
REQUIRED_POLICY_INPUT_FIELDS = {
    "user_approved", "alternative_route_required", "source_state",
    "profile_supported", "issuer_trusted", "proof_valid", "schema_valid",
    "audience_matches", "nonce_fresh", "holder_binding_valid", "time_state",
    "credential_status", "privacy_safe", "authorization_scope_valid",
}
POLICY_INPUT_BOOLEAN_FIELDS = {
    "user_approved", "alternative_route_required", "profile_supported",
    "issuer_trusted", "proof_valid", "schema_valid", "audience_matches",
    "nonce_fresh", "holder_binding_valid", "privacy_safe",
    "authorization_scope_valid",
}
POLICY_INPUT_ENUM_FIELDS = {
    "source_state": {"available", "unavailable"},
    "time_state": {"valid", "expired", "unknown"},
    "credential_status": {"valid", "revoked", "unknown", "unavailable", "stale"},
}
REQUIRED_RECEIPT_POLICY_FIELDS = {
    "short_lived",
    "purpose_scoped",
    "exact_verifier_bound",
    "nonce_or_transaction_bound",
    "store_raw_credential",
    "store_national_identity_number",
    "store_biometric_template",
    "store_document_image",
    "store_replayable_presentation",
    "publish_government_identity_link_to_dht",
    "identity_implies_membership",
    "identity_implies_authority",
    "non_digital_alternative_required",
    "prohibited_output_keys",
}
REQUIRED_SCENARIOS = {
    "synthetic_valid_reference",
    "issuer_display_name_spoof",
    "unknown_issuer_key",
    "wrong_audience",
    "nonce_replay",
    "holder_binding_mismatch",
    "expired_credential",
    "revoked_credential",
    "tampered_claim_or_schema",
    "status_service_unavailable",
    "status_unknown_or_malformed",
    "unsupported_unpublished_profile",
    "credential_or_identifier_leak_to_receipt",
    "identity_upgraded_to_membership_or_authority",
    "user_declines_presentation",
    "no_smartphone_or_verifier_offline",
    "invalid_cryptographic_proof",
    "stale_credential_status",
    "credential_validity_time_unknown",
    "unsupported_algorithm_or_profile_downgrade",
}
EXPECTED_OUTCOME_BY_SCENARIO = {
    "synthetic_valid_reference": ("verified", "none"),
    "issuer_display_name_spoof": ("rejected", "issuer_not_trusted"),
    "unknown_issuer_key": ("rejected", "issuer_not_trusted"),
    "wrong_audience": ("rejected", "wrong_audience"),
    "nonce_replay": ("rejected", "nonce_replay"),
    "holder_binding_mismatch": ("rejected", "holder_binding_failed"),
    "expired_credential": ("rejected", "credential_expired"),
    "revoked_credential": ("rejected", "credential_revoked"),
    "tampered_claim_or_schema": ("rejected", "schema_or_profile_mismatch"),
    "status_service_unavailable": ("indeterminate", "status_unavailable"),
    "status_unknown_or_malformed": ("indeterminate", "status_unknown"),
    "unsupported_unpublished_profile": ("indeterminate", "unsupported_profile"),
    "credential_or_identifier_leak_to_receipt": ("rejected", "privacy_boundary_violation"),
    "identity_upgraded_to_membership_or_authority": ("rejected", "authorization_scope_violation"),
    "user_declines_presentation": ("indeterminate", "user_declined"),
    "no_smartphone_or_verifier_offline": ("indeterminate", "alternative_route_required"),
    "invalid_cryptographic_proof": ("rejected", "invalid_proof"),
    "stale_credential_status": ("indeterminate", "status_stale"),
    "credential_validity_time_unknown": ("indeterminate", "credential_time_unknown"),
    "unsupported_algorithm_or_profile_downgrade": ("indeterminate", "unsupported_profile"),
}
FORBIDDEN_RECEIPT_KEYS = {
    "national_identity_number",
    "sa_id_number",
    "biometric_template",
    "face_template",
    "fingerprint_template",
    "full_credential",
    "document_image",
    "replayable_presentation",
    "raw_presentation_token",
}
ALLOWED_DISPOSITIONS = {"verified", "rejected", "indeterminate"}
ALLOWED_EXPECTED_DISPOSITION_BY_REASON = {
    "none": "verified",
    "issuer_not_trusted": "rejected",
    "invalid_proof": "rejected",
    "wrong_audience": "rejected",
    "nonce_replay": "rejected",
    "holder_binding_failed": "rejected",
    "credential_expired": "rejected",
    "credential_revoked": "rejected",
    "schema_or_profile_mismatch": "rejected",
    "user_declined": "indeterminate",
    "privacy_boundary_violation": "rejected",
    "authorization_scope_violation": "rejected",
    "unsupported_profile": "indeterminate",
    "status_unavailable": "indeterminate",
    "status_unknown": "indeterminate",
    "status_stale": "indeterminate",
    "credential_time_unknown": "indeterminate",
    "source_unavailable": "indeterminate",
    "alternative_route_required": "indeterminate",
}
NEVER_ENABLE_FIELDS = (
    ("receipt_policy", "store_raw_credential"),
    ("receipt_policy", "store_national_identity_number"),
    ("receipt_policy", "store_biometric_template"),
    ("receipt_policy", "store_document_image"),
    ("receipt_policy", "store_replayable_presentation"),
    ("receipt_policy", "publish_government_identity_link_to_dht"),
    ("receipt_policy", "identity_implies_membership"),
    ("receipt_policy", "identity_implies_authority"),
    ("authority", "official_issuer_trust_anchor_configured"),
    ("authority", "published_wallet_relying_party_profile_reviewed"),
    ("authority", "dha_relying_party_authorization_confirmed"),
)


def _get_nested(data: dict[str, Any], path: tuple[str, ...]) -> Any:
    value: Any = data
    for part in path:
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def validate_profile(data: Any) -> list[str]:
    """Return all structural, safety-boundary, and corpus errors found."""
    errors: list[str] = []
    if not isinstance(data, dict):
        return ["profile must be a JSON object"]
    if set(data) != REQUIRED_TOP_LEVEL_FIELDS:
        errors.append("top-level fields must match the closed v1 schema")

    if data.get("schema_id") != "mycelix.sa-digital-id-interoperability-profile":
        errors.append("unexpected schema_id")
    # bool is a subclass of int in Python; reject true as schema_version=1.
    if type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        errors.append("schema_version must be the integer 1")
    if data.get("profile_id") != "SA-DID-001":
        errors.append("profile_id must be SA-DID-001")
    if data.get("maturity") != "research_only":
        errors.append("maturity must remain research_only")
    if data.get("production_enabled") is not False:
        errors.append("production_enabled must be false")

    for path in NEVER_ENABLE_FIELDS:
        if _get_nested(data, path) is not False:
            errors.append("safety boundary must remain false: " + ".".join(path))

    authority = data.get("authority")
    if not isinstance(authority, dict):
        errors.append("authority must be an object")
    elif set(authority) != REQUIRED_AUTHORITY_FIELDS:
        errors.append("authority fields must match the closed v1 schema")

    routes = data.get("routes")
    route_ids: set[str] = set()
    if not isinstance(routes, list):
        errors.append("routes must be an array")
        routes = []
    for idx, route in enumerate(routes):
        prefix = f"routes[{idx}]"
        if not isinstance(route, dict):
            errors.append(f"{prefix} must be an object")
            continue
        route_id = route.get("route_id")
        if isinstance(route_id, str) and route_id in REQUIRED_ROUTE_FIELDS:
            if set(route) != REQUIRED_ROUTE_FIELDS[route_id]:
                errors.append(f"{prefix} fields must match the closed v1 route schema")
        if not isinstance(route_id, str) or not route_id:
            errors.append(f"{prefix}.route_id must be a non-empty string")
        elif route_id in route_ids:
            errors.append(f"duplicate route_id: {route_id}")
        else:
            route_ids.add(route_id)
        if route.get("production_enabled") is not False:
            errors.append(f"{prefix}.production_enabled must be false")
        expected_method = (
            REQUIRED_ROUTE_METHOD_BY_ID.get(route_id)
            if isinstance(route_id, str)
            else None
        )
        if expected_method is not None and route.get("method_kind") != expected_method:
            errors.append(f"{prefix}.method_kind must preserve the distinct route semantics")
        if isinstance(route_id, str) and route_id in REQUIRED_ROUTE_STATUS_BY_ID:
            if route.get("status") != REQUIRED_ROUTE_STATUS_BY_ID[route_id]:
                errors.append(f"{prefix}.status must preserve the reviewed research status")
            if route.get("evidence_nonclaims") != REQUIRED_ROUTE_NONCLAIMS_BY_ID[route_id]:
                errors.append(f"{prefix}.evidence_nonclaims must preserve the v1 nonclaims")
        if route_id == "dha_ovs":
            if route.get("public_protocol_spec_in_this_repository") is not False:
                errors.append(f"{prefix}.public_protocol_spec_in_this_repository must remain false")
            if route.get("public_onboarding_contact") != "Verifications@dha.gov.za":
                errors.append(f"{prefix}.public_onboarding_contact must match the documented DHA channel")
        if route_id == "dha_smartphone_digital_id":
            if route.get("protocol_profile_id") is not None:
                errors.append(f"{prefix}.protocol_profile_id must remain null until DHA profile review")
            for key in ("official_issuer_trust_anchor_configured", "status_or_revocation_profile_configured"):
                if route.get(key) is not False:
                    errors.append(f"{prefix}.{key} must remain false until official profile review")
        urls = route.get("source_urls")
        if not isinstance(urls, list) or not urls or any(
            not isinstance(url, str) or not url.startswith(OFFICIAL_SOURCE_PREFIXES)
            for url in urls
        ):
            errors.append(f"{prefix}.source_urls must contain official HTTPS government sources")
        expected_urls = (
            REQUIRED_ROUTE_SOURCE_URLS_BY_ID.get(route_id)
            if isinstance(route_id, str)
            else None
        )
        if expected_urls is not None and urls != expected_urls:
            errors.append(f"{prefix}.source_urls must match the pinned official research sources")
    if route_ids != REQUIRED_ROUTES:
        errors.append(
            "routes must distinguish exactly dha_ovs and dha_smartphone_digital_id"
        )

    allowed = data.get("allowed_dispositions")
    if (
        not isinstance(allowed, list)
        or any(not isinstance(item, str) for item in allowed)
        or len(allowed) != len(ALLOWED_DISPOSITIONS)
        or set(allowed) != ALLOWED_DISPOSITIONS
    ):
        errors.append("allowed_dispositions must enumerate exactly verified, rejected, and indeterminate")

    receipt = data.get("receipt_policy")
    if not isinstance(receipt, dict):
        errors.append("receipt_policy must be an object")
    else:
        if set(receipt) != REQUIRED_RECEIPT_POLICY_FIELDS:
            errors.append("receipt_policy fields must match the closed v1 policy schema")
        for key in (
            "short_lived",
            "purpose_scoped",
            "exact_verifier_bound",
            "nonce_or_transaction_bound",
            "non_digital_alternative_required",
        ):
            if receipt.get(key) is not True:
                errors.append(f"receipt_policy.{key} must be true")
        keys = receipt.get("prohibited_output_keys")
        if (
            not isinstance(keys, list)
            or any(not isinstance(key, str) for key in keys)
            or not FORBIDDEN_RECEIPT_KEYS.issubset(set(keys))
        ):
            errors.append("prohibited_output_keys must include every required sensitive field as strings")

    mappings = data.get("reason_disposition")
    if not isinstance(mappings, dict):
        errors.append("reason_disposition must be an object")
        mappings = {}
    elif set(mappings) != set(ALLOWED_EXPECTED_DISPOSITION_BY_REASON):
        errors.append("reason_disposition keys must match the closed v1 reason enum")
    for reason, expected in ALLOWED_EXPECTED_DISPOSITION_BY_REASON.items():
        if mappings.get(reason) != expected:
            errors.append(f"wrong or missing disposition mapping for reason: {reason}")
    for reason, disposition in mappings.items():
        if not isinstance(disposition, str) or disposition not in ALLOWED_DISPOSITIONS:
            errors.append(f"unsupported disposition for reason {reason}: {disposition}")

    vectors = data.get("test_vectors")
    if not isinstance(vectors, list):
        errors.append("test_vectors must be an array")
        vectors = []
    ids: set[str] = set()
    scenarios: set[str] = set()
    verified_vectors: list[dict[str, Any]] = []
    for idx, vector in enumerate(vectors):
        prefix = f"test_vectors[{idx}]"
        if not isinstance(vector, dict):
            errors.append(f"{prefix} must be an object")
            continue
        if set(vector) != REQUIRED_VECTOR_FIELDS:
            errors.append(f"{prefix} fields must match the closed v1 vector schema")
        vector_id = vector.get("id")
        scenario = vector.get("scenario")
        if not isinstance(vector_id, str) or not vector_id:
            errors.append(f"{prefix}.id must be a non-empty string")
        elif vector_id in ids:
            errors.append(f"duplicate vector id: {vector_id}")
        else:
            ids.add(vector_id)
        if not isinstance(scenario, str) or not scenario:
            errors.append(f"{prefix}.scenario must be a non-empty string")
        elif scenario in scenarios:
            errors.append(f"duplicate scenario: {scenario}")
        else:
            scenarios.add(scenario)
        if vector.get("synthetic_fixture") is not True:
            errors.append(f"{prefix} must be explicitly marked synthetic_fixture=true")
        if vector.get("production_eligible") is not False:
            errors.append(f"{prefix} must have production_eligible=false")
        anchor_ref = vector.get("trust_anchor_ref")
        if not isinstance(anchor_ref, str) or not (
            anchor_ref == "none" or anchor_ref.startswith("test-only:")
        ):
            errors.append(f"{prefix}.trust_anchor_ref must be none or an explicit test-only reference")

        policy_inputs = vector.get("policy_inputs")
        if not isinstance(policy_inputs, dict):
            errors.append(f"{prefix}.policy_inputs must be an object")
        else:
            if set(policy_inputs) != REQUIRED_POLICY_INPUT_FIELDS:
                errors.append(f"{prefix}.policy_inputs must match the closed synthetic input schema")
            for field in POLICY_INPUT_BOOLEAN_FIELDS:
                if type(policy_inputs.get(field)) is not bool:
                    errors.append(f"{prefix}.policy_inputs.{field} must be a Boolean")
            for field, allowed_values in POLICY_INPUT_ENUM_FIELDS.items():
                value = policy_inputs.get(field)
                if not isinstance(value, str) or value not in allowed_values:
                    errors.append(f"{prefix}.policy_inputs.{field} has an invalid value")
            if policy_inputs.get("issuer_trusted") is True and not (
                isinstance(anchor_ref, str) and anchor_ref.startswith("test-only:")
            ):
                errors.append(f"{prefix}: trusted synthetic issuer requires a test-only anchor")

        disposition = vector.get("expected_disposition")
        reason = vector.get("expected_reason")
        if not isinstance(disposition, str) or disposition not in ALLOWED_DISPOSITIONS:
            errors.append(f"{prefix}.expected_disposition is invalid")
        if not isinstance(reason, str) or not reason:
            errors.append(f"{prefix}.expected_reason must be a non-empty string")
        elif mappings.get(reason) != disposition:
            errors.append(f"{prefix}: expected disposition disagrees with reason mapping")
        if isinstance(scenario, str):
            expected_outcome = EXPECTED_OUTCOME_BY_SCENARIO.get(scenario)
            if expected_outcome is not None and (disposition, reason) != expected_outcome:
                errors.append(f"{prefix}: scenario does not match its required disposition and reason")

        if disposition == "verified":
            verified_vectors.append(vector)
            if reason != "none":
                errors.append(f"{prefix}: verified outcome must use reason none")
            if not str(vector.get("trust_anchor_ref", "")).startswith("test-only:"):
                errors.append(f"{prefix}: verified fixture must use test-only trust anchor")
            if scenario != "synthetic_valid_reference":
                errors.append(f"{prefix}: only the synthetic valid reference may be verified")

    missing_scenarios = REQUIRED_SCENARIOS - scenarios
    if missing_scenarios:
        errors.append("missing required scenarios: " + ", ".join(sorted(missing_scenarios)))
    if len(verified_vectors) != 1:
        errors.append("corpus must contain exactly one verified synthetic reference vector")
    if len(ids) != len(REQUIRED_SCENARIOS):
        errors.append(f"expected exactly {len(REQUIRED_SCENARIOS)} unique test vectors")

    notes = data.get("notes")
    if not isinstance(notes, list) or not notes or not any(
        isinstance(note, str) and "not a credential parser" in note.lower()
        for note in notes
    ):
        errors.append("notes must state that this is not a credential parser/verifier")

    return errors


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """Reject duplicate JSON keys instead of silently accepting the last value."""
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    args = parser.parse_args()
    try:
        data = json.loads(
            args.profile.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_object_keys,
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: could not load profile: {exc}", file=sys.stderr)
        return 2

    errors = validate_profile(data)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"SA_DID_PROFILE_VALIDATION=FAIL errors={len(errors)}", file=sys.stderr)
        return 1

    print(
        "SA_DID_PROFILE_VALIDATION=PASS "
        f"vectors={len(data['test_vectors'])} production_enabled=false "
        "scope=structural_contract_only"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
