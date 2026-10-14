#!/usr/bin/env python3
"""Validate a portable sovereign adapter profile without granting trust.

The validator enforces a closed v1 schema and non-compensatory release gates.
It does not verify credentials, signatures, legal authority, external endpoints,
independent-review claims, or deployment security. A passing research template
is not an integration certification.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

PROFILE_PATH = Path(__file__).with_name("PORTABLE_ADAPTER_PROFILE_TEMPLATE_V1.json")
SCHEMA_ID = "org.luminousdynamics.portable-sovereign-adapter-profile"
MATURITY_VALUES = {
    "not_assessed", "research_only", "partner_review", "sandbox_authorized", "production_authorized"
}
ROUTE_KINDS = {"synthetic_only", "server_side_verification", "holder_presented_credential", "federated_identity_provider"}
JURISDICTION_SCOPES = {"national", "regional", "bilateral", "service"}
LAWFUL_BASIS_VALUES = {"not_assessed", "review_required", "reviewed", "approved"}
REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_id", "schema_version", "profile_id", "title", "maturity", "production_enabled",
    "jurisdiction", "authority", "protocol", "trust", "service", "privacy", "accessibility",
    "evidence", "release_gates", "nonclaims",
}
NESTED_FIELDS = {
    "jurisdiction": {"scope_type", "code", "name"},
    "authority": {"authority_name", "authority_id", "official_sources", "written_authorization_ref"},
    "protocol": {"route_kind", "name", "version", "verifier_endpoint", "official_specification_url", "official_spec_sha256", "reviewed", "externally_authorized"},
    "trust": {"issuer_ids", "metadata_url", "key_rotation_policy_reviewed", "status_method", "revocation_method"},
    "service": {"service_id", "purpose", "lawful_basis_status", "purpose_grant_id", "permitted_claims", "prohibited_decisions", "identity_is_not_entitlement_evidence"},
    "privacy": {"data_minimization_reviewed", "raw_credentials_storage", "national_identifier_storage", "biometrics_storage", "replayable_presentation_storage", "public_dht_identity_material", "purpose_bound_receipts_required", "receipt_retention_days", "deletion_policy_reviewed"},
    "accessibility": {"non_digital_fallback_available", "fallback_tested", "assisted_path_available", "languages", "accessibility_reviewed"},
    "evidence": {"source_commit", "conformance_suite_id", "positive_control_count", "negative_control_count", "independent_review_ref", "release_approval_ref", "release_approval_commit"},
}
REQUIRED_RELEASE_GATES = {
    "legal_basis_reviewed", "written_authority_or_partner_approval",
    "official_protocol_and_trust_metadata_verified", "status_lifecycle_and_key_rotation_reviewed",
    "purpose_minimization_and_data_protection_approved", "retention_and_erasure_verified",
    "offline_assisted_and_non_digital_fallback_tested", "accessibility_and_localization_reviewed",
    "independent_security_review_complete", "positive_and_negative_conformance_passed",
    "incident_correction_and_appeal_agreed", "human_approval_for_exact_release_commit",
}
FORBIDDEN_IDENTITY_MATERIAL_FIELDS = (
    "raw_credentials_storage", "national_identifier_storage", "biometrics_storage",
    "replayable_presentation_storage", "public_dht_identity_material",
)
REQUIRED_PROHIBITED_DECISIONS = {
    "citizenship", "residency_or_migration_permission", "benefit_or_service_eligibility",
    "entitlement_or_legal_rights", "mycelix_membership", "governance_or_execution_authority",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
PROFILE_ID_RE = re.compile(r"^[A-Z0-9][A-Z0-9._-]{2,63}$")


def _is_optional_text(value: Any) -> bool:
    return value is None or (isinstance(value, str) and bool(value.strip()))


def _is_https_url(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    parsed = urlparse(value)
    return parsed.scheme == "https" and bool(parsed.netloc) and not parsed.username and not parsed.password


def validate_profile(data: Any) -> list[str]:
    """Return closed-schema, privacy and release-gate validation errors."""
    if not isinstance(data, dict):
        return ["profile must be a JSON object"]
    errors: list[str] = []
    if set(data) != REQUIRED_TOP_LEVEL_FIELDS:
        errors.append("top-level fields must match the closed v1 profile schema")
    if data.get("schema_id") != SCHEMA_ID:
        errors.append("unexpected schema_id")
    if type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        errors.append("schema_version must be the integer 1")
    profile_id = data.get("profile_id")
    if not isinstance(profile_id, str) or not PROFILE_ID_RE.fullmatch(profile_id):
        errors.append("profile_id must be a 3-64 character uppercase stable identifier")
    if not isinstance(data.get("title"), str) or not data["title"].strip():
        errors.append("title must be a non-empty string")
    maturity = data.get("maturity")
    if not isinstance(maturity, str) or maturity not in MATURITY_VALUES:
        errors.append("maturity must use the closed v1 enum")
    if type(data.get("production_enabled")) is not bool:
        errors.append("production_enabled must be a boolean")
    if data.get("production_enabled") is not (maturity == "production_authorized"):
        errors.append("production_enabled must be true exactly for production_authorized maturity")

    for section, required_fields in NESTED_FIELDS.items():
        obj = data.get(section)
        if not isinstance(obj, dict):
            errors.append(f"{section} must be an object")
            continue
        if set(obj) != required_fields:
            errors.append(f"{section} fields must match the closed v1 schema")

    jurisdiction = data.get("jurisdiction")
    if isinstance(jurisdiction, dict):
        if not isinstance(jurisdiction.get("scope_type"), str) or jurisdiction.get("scope_type") not in JURISDICTION_SCOPES:
            errors.append("jurisdiction.scope_type must use the closed v1 enum")
        for key in ("code", "name"):
            if not isinstance(jurisdiction.get(key), str) or not jurisdiction[key].strip():
                errors.append(f"jurisdiction.{key} must be a non-empty string")

    authority = data.get("authority")
    if isinstance(authority, dict):
        for key in ("authority_name", "authority_id", "written_authorization_ref"):
            if not _is_optional_text(authority.get(key)):
                errors.append(f"authority.{key} must be null or non-empty text")
        sources = authority.get("official_sources")
        if not isinstance(sources, list):
            errors.append("authority.official_sources must be an array")
        else:
            for index, source in enumerate(sources):
                if not isinstance(source, dict) or set(source) != {"url", "supports"}:
                    errors.append(f"authority.official_sources[{index}] must contain exactly url and supports")
                    continue
                if not _is_https_url(source.get("url")):
                    errors.append(f"authority.official_sources[{index}].url must be an HTTPS URL")
                if not isinstance(source.get("supports"), str) or not source["supports"].strip():
                    errors.append(f"authority.official_sources[{index}].supports must be non-empty text")

    protocol = data.get("protocol")
    if isinstance(protocol, dict):
        if not isinstance(protocol.get("route_kind"), str) or protocol.get("route_kind") not in ROUTE_KINDS:
            errors.append("protocol.route_kind must use the closed v1 enum")
        for key in ("name", "version", "verifier_endpoint", "official_specification_url", "official_spec_sha256"):
            if not _is_optional_text(protocol.get(key)):
                errors.append(f"protocol.{key} must be null or non-empty text")
        for key in ("reviewed", "externally_authorized"):
            if type(protocol.get(key)) is not bool:
                errors.append(f"protocol.{key} must be a boolean")
        if protocol.get("verifier_endpoint") is not None and not _is_https_url(protocol.get("verifier_endpoint")):
            errors.append("protocol.verifier_endpoint must be null or an HTTPS URL")
        if protocol.get("official_specification_url") is not None and not _is_https_url(protocol.get("official_specification_url")):
            errors.append("protocol.official_specification_url must be null or an HTTPS URL")
        spec_hash = protocol.get("official_spec_sha256")
        if spec_hash is not None and (not isinstance(spec_hash, str) or not SHA256_RE.fullmatch(spec_hash)):
            errors.append("protocol.official_spec_sha256 must be null or a lowercase SHA-256 digest")

    trust = data.get("trust")
    if isinstance(trust, dict):
        issuer_ids = trust.get("issuer_ids")
        if not isinstance(issuer_ids, list) or any(not isinstance(v, str) or not v.strip() for v in issuer_ids):
            errors.append("trust.issuer_ids must be an array of non-empty strings")
        for key in ("metadata_url", "status_method", "revocation_method"):
            if not _is_optional_text(trust.get(key)):
                errors.append(f"trust.{key} must be null or non-empty text")
        if trust.get("metadata_url") is not None and not _is_https_url(trust.get("metadata_url")):
            errors.append("trust.metadata_url must be null or an HTTPS URL")
        if type(trust.get("key_rotation_policy_reviewed")) is not bool:
            errors.append("trust.key_rotation_policy_reviewed must be a boolean")

    service = data.get("service")
    if isinstance(service, dict):
        for key in ("service_id", "purpose"):
            if not isinstance(service.get(key), str) or not service[key].strip():
                errors.append(f"service.{key} must be non-empty text")
        if not isinstance(service.get("lawful_basis_status"), str) or service.get("lawful_basis_status") not in LAWFUL_BASIS_VALUES:
            errors.append("service.lawful_basis_status must use the closed v1 enum")
        if not _is_optional_text(service.get("purpose_grant_id")):
            errors.append("service.purpose_grant_id must be null or non-empty text")
        claims = service.get("permitted_claims")
        if not isinstance(claims, list) or not claims or any(not isinstance(v, str) or not v.strip() for v in claims):
            errors.append("service.permitted_claims must be a non-empty array of non-empty strings")
        decisions = service.get("prohibited_decisions")
        if (
            not isinstance(decisions, list)
            or any(not isinstance(value, str) for value in decisions)
            or len(decisions) != len(REQUIRED_PROHIBITED_DECISIONS)
            or set(decisions) != REQUIRED_PROHIBITED_DECISIONS
        ):
            errors.append("service.prohibited_decisions must preserve all non-transferable decision boundaries")
        if service.get("identity_is_not_entitlement_evidence") is not True:
            errors.append("identity_is_not_entitlement_evidence must remain true")

    privacy = data.get("privacy")
    if isinstance(privacy, dict):
        for key in ("data_minimization_reviewed", "purpose_bound_receipts_required", "deletion_policy_reviewed"):
            if type(privacy.get(key)) is not bool:
                errors.append(f"privacy.{key} must be a boolean")
        for key in FORBIDDEN_IDENTITY_MATERIAL_FIELDS:
            if privacy.get(key) != "prohibited":
                errors.append(f"privacy.{key} must remain prohibited")
        retention = privacy.get("receipt_retention_days")
        if type(retention) is not int or retention < 0:
            errors.append("privacy.receipt_retention_days must be a non-negative integer")
        if privacy.get("purpose_bound_receipts_required") is not True:
            errors.append("purpose-bound receipts must remain required")

    accessibility = data.get("accessibility")
    if isinstance(accessibility, dict):
        for key in ("non_digital_fallback_available", "fallback_tested", "assisted_path_available", "accessibility_reviewed"):
            if type(accessibility.get(key)) is not bool:
                errors.append(f"accessibility.{key} must be a boolean")
        if accessibility.get("non_digital_fallback_available") is not True:
            errors.append("a non-digital fallback must be available")
        if accessibility.get("assisted_path_available") is not True:
            errors.append("an assisted path must be available")
        languages = accessibility.get("languages")
        if not isinstance(languages, list) or any(not isinstance(v, str) or not v.strip() for v in languages):
            errors.append("accessibility.languages must be an array of non-empty strings")

    evidence = data.get("evidence")
    if isinstance(evidence, dict):
        for key in ("source_commit", "conformance_suite_id", "independent_review_ref", "release_approval_ref", "release_approval_commit"):
            if not _is_optional_text(evidence.get(key)):
                errors.append(f"evidence.{key} must be null or non-empty text")
        for key in ("positive_control_count", "negative_control_count"):
            value = evidence.get(key)
            if type(value) is not int or value < 0:
                errors.append(f"evidence.{key} must be a non-negative integer")
        if evidence.get("source_commit") is not None and (not isinstance(evidence.get("source_commit"), str) or not COMMIT_RE.fullmatch(evidence["source_commit"])):
            errors.append("evidence.source_commit must be null or a full lowercase commit SHA")
        if evidence.get("release_approval_commit") is not None and (not isinstance(evidence.get("release_approval_commit"), str) or not COMMIT_RE.fullmatch(evidence["release_approval_commit"])):
            errors.append("evidence.release_approval_commit must be null or a full lowercase commit SHA")

    gates = data.get("release_gates")
    if not isinstance(gates, dict):
        errors.append("release_gates must be an object")
    else:
        if set(gates) != REQUIRED_RELEASE_GATES:
            errors.append("release_gates keys must match the closed v1 gate set")
        for gate in REQUIRED_RELEASE_GATES:
            if type(gates.get(gate)) is not bool:
                errors.append(f"release gate must be boolean: {gate}")

    nonclaims = data.get("nonclaims")
    if not isinstance(nonclaims, list) or len(nonclaims) < 3 or any(not isinstance(v, str) or not v.strip() for v in nonclaims):
        errors.append("nonclaims must contain at least three non-empty caveats")
    else:
        combined = " ".join(nonclaims).lower()
        for phrase in ("does not itself grant identity", "does not imply external permission", "not a digital signature"):
            if phrase not in combined:
                errors.append(f"nonclaims must preserve: {phrase}")

    if maturity == "production_authorized":
        # Gates are conjunctive and cannot be replaced by a weighted score.
        if not isinstance(gates, dict) or set(gates) != REQUIRED_RELEASE_GATES or any(gates.get(k) is not True for k in REQUIRED_RELEASE_GATES):
            errors.append("production_authorized requires every non-compensatory release gate to be true")
        if isinstance(authority, dict):
            if not _is_optional_text(authority.get("authority_name")) or not isinstance(authority.get("authority_name"), str):
                errors.append("production_authorized requires an identified authority")
            if not _is_optional_text(authority.get("written_authorization_ref")) or not isinstance(authority.get("written_authorization_ref"), str):
                errors.append("production_authorized requires written authorization evidence")
            if not isinstance(authority.get("official_sources"), list) or not authority["official_sources"]:
                errors.append("production_authorized requires official source references")
        if isinstance(protocol, dict):
            if protocol.get("route_kind") == "synthetic_only":
                errors.append("synthetic_only route cannot be production_authorized")
            required_protocol_fields = ("name", "version", "verifier_endpoint", "official_specification_url", "official_spec_sha256")
            if any(not isinstance(protocol.get(k), str) or not protocol[k].strip() for k in required_protocol_fields):
                errors.append("production_authorized requires official protocol identity, endpoint, specification URL and digest")
            if protocol.get("reviewed") is not True or protocol.get("externally_authorized") is not True:
                errors.append("production_authorized requires reviewed and authorized protocol")
        if isinstance(trust, dict):
            if not trust.get("issuer_ids") or not isinstance(trust.get("metadata_url"), str) or not trust.get("status_method") or not trust.get("revocation_method") or trust.get("key_rotation_policy_reviewed") is not True:
                errors.append("production_authorized requires issuer metadata, status/revocation methods and key-rotation review")
        if isinstance(service, dict):
            if service.get("lawful_basis_status") != "approved":
                errors.append("production_authorized requires approved lawful basis")
            if not isinstance(service.get("purpose_grant_id"), str) or not service["purpose_grant_id"].strip():
                errors.append("production_authorized requires a purpose-grant reference")
        if isinstance(privacy, dict) and (privacy.get("data_minimization_reviewed") is not True or privacy.get("deletion_policy_reviewed") is not True):
            errors.append("production_authorized requires data-minimization and deletion-policy review")
        if isinstance(accessibility, dict) and (accessibility.get("fallback_tested") is not True or accessibility.get("accessibility_reviewed") is not True):
            errors.append("production_authorized requires tested fallback and accessibility review")
        if isinstance(evidence, dict):
            if not isinstance(evidence.get("source_commit"), str) or not COMMIT_RE.fullmatch(evidence["source_commit"]):
                errors.append("production_authorized requires evidence.source_commit")
            if not isinstance(evidence.get("release_approval_commit"), str) or evidence.get("release_approval_commit") != evidence.get("source_commit"):
                errors.append("release approval must bind the exact source commit")
            for key in ("conformance_suite_id", "independent_review_ref", "release_approval_ref"):
                if not isinstance(evidence.get(key), str) or not evidence[key].strip():
                    errors.append(f"production_authorized requires evidence.{key}")
            if evidence.get("positive_control_count", 0) < 1 or evidence.get("negative_control_count", 0) < 1:
                errors.append("production_authorized requires positive and negative controls")
    elif maturity == "sandbox_authorized":
        # Sandbox authorization is still a real external permission, not a maturity label
        # that can be selected by filling in booleans. It must not imply production approval.
        required_sandbox_gates = {
            "legal_basis_reviewed",
            "written_authority_or_partner_approval",
            "official_protocol_and_trust_metadata_verified",
            "status_lifecycle_and_key_rotation_reviewed",
        }
        if not isinstance(gates, dict) or any(gates.get(k) is not True for k in required_sandbox_gates):
            errors.append("sandbox_authorized requires legal, written-authority, official-protocol and trust-lifecycle gates")
        if isinstance(authority, dict):
            if not isinstance(authority.get("authority_name"), str) or not authority["authority_name"].strip():
                errors.append("sandbox_authorized requires an identified authority")
            if not isinstance(authority.get("written_authorization_ref"), str) or not authority["written_authorization_ref"].strip():
                errors.append("sandbox_authorized requires written authorization evidence")
            if not isinstance(authority.get("official_sources"), list) or not authority["official_sources"]:
                errors.append("sandbox_authorized requires official source references")
        if isinstance(protocol, dict):
            if protocol.get("route_kind") == "synthetic_only":
                errors.append("synthetic_only route cannot be sandbox_authorized")
            required_protocol_fields = ("name", "version", "verifier_endpoint", "official_specification_url", "official_spec_sha256")
            if any(not isinstance(protocol.get(k), str) or not protocol[k].strip() for k in required_protocol_fields):
                errors.append("sandbox_authorized requires reviewed official protocol identity, endpoint, specification URL and digest")
            if protocol.get("reviewed") is not True or protocol.get("externally_authorized") is not True:
                errors.append("sandbox_authorized requires reviewed and externally authorized protocol")
        if isinstance(trust, dict):
            if not trust.get("issuer_ids") or not isinstance(trust.get("metadata_url"), str) or not trust.get("status_method") or not trust.get("revocation_method") or trust.get("key_rotation_policy_reviewed") is not True:
                errors.append("sandbox_authorized requires issuer metadata, status/revocation methods and key-rotation review")
        if isinstance(service, dict) and (
            not isinstance(service.get("lawful_basis_status"), str)
            or service.get("lawful_basis_status") not in {"reviewed", "approved"}
            or not isinstance(service.get("purpose_grant_id"), str)
            or not service["purpose_grant_id"].strip()
        ):
            errors.append("sandbox_authorized requires reviewed lawful basis and a purpose-grant reference")
        if isinstance(evidence, dict):
            if not isinstance(evidence.get("source_commit"), str) or not COMMIT_RE.fullmatch(evidence["source_commit"]):
                errors.append("sandbox_authorized requires evidence.source_commit")
            if not isinstance(evidence.get("release_approval_commit"), str) or evidence.get("release_approval_commit") != evidence.get("source_commit"):
                errors.append("sandbox authorization must bind the exact source commit")
            if not isinstance(evidence.get("release_approval_ref"), str) or not evidence["release_approval_ref"].strip():
                errors.append("sandbox_authorized requires evidence.release_approval_ref")
    elif isinstance(gates, dict) and gates.get("human_approval_for_exact_release_commit") is True:
        # Approval is the final gate and must never appear open on a non-production profile.
        errors.append("human release approval cannot be open on a non-production profile")

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
    parser.add_argument("--profile", type=Path, default=PROFILE_PATH, help="profile JSON to validate")
    args = parser.parse_args()
    try:
        profile = json.loads(args.profile.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_object_keys)
    except (OSError, ValueError) as exc:
        print(f"ERROR: could not load portable adapter profile: {exc}", file=sys.stderr)
        return 2
    errors = validate_profile(profile)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"PORTABLE_ADAPTER_PROFILE_VALIDATION=FAIL errors={len(errors)}", file=sys.stderr)
        return 1
    gates = profile["release_gates"]
    print(
        "PORTABLE_ADAPTER_PROFILE_VALIDATION=PASS "
        f"profile_id={profile['profile_id']} maturity={profile['maturity']} "
        f"gates_open={sum(value is True for value in gates.values())} "
        f"production_enabled={str(profile['production_enabled']).lower()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
