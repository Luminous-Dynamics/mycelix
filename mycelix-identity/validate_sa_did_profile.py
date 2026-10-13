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
        if not isinstance(route_id, str) or not route_id:
            errors.append(f"{prefix}.route_id must be a non-empty string")
        elif route_id in route_ids:
            errors.append(f"duplicate route_id: {route_id}")
        else:
            route_ids.add(route_id)
        if route.get("production_enabled") is not False:
            errors.append(f"{prefix}.production_enabled must be false")
        urls = route.get("source_urls")
        if not isinstance(urls, list) or not urls or any(
            not isinstance(url, str) or not url.startswith("https://www.gov.za/")
            for url in urls
        ):
            errors.append(f"{prefix}.source_urls must contain official HTTPS gov.za sources")
    if route_ids != REQUIRED_ROUTES:
        errors.append(
            "routes must distinguish exactly dha_ovs and dha_smartphone_digital_id"
        )

    receipt = data.get("receipt_policy")
    if not isinstance(receipt, dict):
        errors.append("receipt_policy must be an object")
    else:
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

        disposition = vector.get("expected_disposition")
        reason = vector.get("expected_reason")
        if not isinstance(disposition, str) or disposition not in ALLOWED_DISPOSITIONS:
            errors.append(f"{prefix}.expected_disposition is invalid")
        if not isinstance(reason, str) or not reason:
            errors.append(f"{prefix}.expected_reason must be a non-empty string")
        elif mappings.get(reason) != disposition:
            errors.append(f"{prefix}: expected disposition disagrees with reason mapping")

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH)
    args = parser.parse_args()
    try:
        data = json.loads(args.profile.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
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
