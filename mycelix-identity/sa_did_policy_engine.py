#!/usr/bin/env python3
"""Synthetic-only SA-DID policy oracle.

This module evaluates synthetic policy signals, not credentials. It does not
parse credential formats, verify signatures, retrieve issuer keys, query DHA,
or establish legal identity. Its only positive result is a test-corpus outcome
when a vector is explicitly synthetic and production-ineligible.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import validate_sa_did_profile

POLICY_INPUT_FIELDS = {
    "user_approved",
    "alternative_route_required",
    "source_state",
    "profile_supported",
    "issuer_trusted",
    "proof_valid",
    "schema_valid",
    "audience_matches",
    "nonce_fresh",
    "holder_binding_valid",
    "time_state",
    "credential_status",
    "privacy_safe",
    "authorization_scope_valid",
}
BOOLEAN_FIELDS = {
    "user_approved",
    "alternative_route_required",
    "profile_supported",
    "issuer_trusted",
    "proof_valid",
    "schema_valid",
    "audience_matches",
    "nonce_fresh",
    "holder_binding_valid",
    "privacy_safe",
    "authorization_scope_valid",
}
ENUM_FIELDS = {
    "source_state": {"available", "unavailable"},
    "time_state": {"valid", "expired", "unknown"},
    "credential_status": {"valid", "revoked", "unknown", "unavailable", "stale"},
}
DISPOSITIONS = {"verified", "rejected", "indeterminate"}


def _decision(disposition: str, reason: str) -> tuple[str, str]:
    """Return the policy disposition and stable reason code."""
    return disposition, reason


def evaluate_synthetic_vector(vector: Any) -> tuple[str, str]:
    """Evaluate a frozen synthetic policy vector; never a real credential.

    Raises ValueError for malformed or production-eligible fixtures. Callers
    must not turn exceptions into a VERIFIED result.
    """
    if not isinstance(vector, dict):
        raise ValueError("vector must be a JSON-like object")
    if vector.get("synthetic_fixture") is not True:
        raise ValueError("only explicitly synthetic fixtures are supported")
    if vector.get("production_eligible") is not False:
        raise ValueError("synthetic policy oracle refuses production-eligible vectors")

    anchor_ref = vector.get("trust_anchor_ref")
    if not isinstance(anchor_ref, str) or not (
        anchor_ref == "none" or anchor_ref.startswith("test-only:")
    ):
        raise ValueError("trust_anchor_ref must be none or an explicit test-only reference")

    evidence = vector.get("policy_inputs")
    if not isinstance(evidence, dict) or set(evidence) != POLICY_INPUT_FIELDS:
        raise ValueError("policy_inputs must match the closed synthetic input schema")

    for field in BOOLEAN_FIELDS:
        if type(evidence.get(field)) is not bool:
            raise ValueError(f"policy_inputs.{field} must be a Boolean")

    for field, allowed_values in ENUM_FIELDS.items():
        value = evidence.get(field)
        if not isinstance(value, str) or value not in allowed_values:
            raise ValueError(f"policy_inputs.{field} must be one of {sorted(allowed_values)}")

    if evidence["issuer_trusted"] and not anchor_ref.startswith("test-only:"):
        raise ValueError("a trusted synthetic issuer must use a test-only trust reference")

    # Preserve alternative access before attempting identity checks.
    if evidence["alternative_route_required"]:
        return _decision("indeterminate", "alternative_route_required")
    if not evidence["user_approved"]:
        return _decision("indeterminate", "user_declined")
    if evidence["source_state"] == "unavailable":
        return _decision("indeterminate", "source_unavailable")
    if not evidence["profile_supported"]:
        return _decision("indeterminate", "unsupported_profile")
    if not evidence["issuer_trusted"]:
        return _decision("rejected", "issuer_not_trusted")
    if not evidence["proof_valid"]:
        return _decision("rejected", "invalid_proof")
    if not evidence["schema_valid"]:
        return _decision("rejected", "schema_or_profile_mismatch")
    if not evidence["audience_matches"]:
        return _decision("rejected", "wrong_audience")
    if not evidence["nonce_fresh"]:
        return _decision("rejected", "nonce_replay")
    if not evidence["holder_binding_valid"]:
        return _decision("rejected", "holder_binding_failed")
    if evidence["time_state"] == "expired":
        return _decision("rejected", "credential_expired")
    if evidence["time_state"] == "unknown":
        return _decision("indeterminate", "credential_time_unknown")

    status_outcomes = {
        "revoked": ("rejected", "credential_revoked"),
        "unknown": ("indeterminate", "status_unknown"),
        "unavailable": ("indeterminate", "status_unavailable"),
        "stale": ("indeterminate", "status_stale"),
    }
    if evidence["credential_status"] in status_outcomes:
        return status_outcomes[evidence["credential_status"]]

    if not evidence["privacy_safe"]:
        return _decision("rejected", "privacy_boundary_violation")
    if not evidence["authorization_scope_valid"]:
        return _decision("rejected", "authorization_scope_violation")

    # "verified" here means only that the synthetic policy inputs satisfy
    # this test oracle. It is not a cryptographic or real-world identity result.
    return _decision("verified", "none")


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """Reject duplicate JSON keys instead of accepting ambiguous input."""
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def build_report(
    profile: Any,
    *,
    profile_source_bytes: bytes | None = None,
    source_commit_sha: str | None = None,
) -> dict[str, Any]:
    """Validate the contract and bind results to exact input and source hashes."""
    if not isinstance(profile, dict):
        raise ValueError("profile must be an object")
    if profile.get("production_enabled") is not False:
        raise ValueError("policy oracle refuses a production-enabled profile")

    # Do not let a report pass simply because expected labels and synthetic
    # inputs were altered together. Enforce the independent, closed contract
    # (including the scenario-to-outcome table) before executing the oracle.
    contract_errors = validate_sa_did_profile.validate_profile(profile)
    if contract_errors:
        summary = "; ".join(contract_errors[:8])
        if len(contract_errors) > 8:
            summary += f"; and {len(contract_errors) - 8} more contract errors"
        raise ValueError("profile contract validation failed: " + summary)

    vectors = profile.get("test_vectors")
    if not isinstance(vectors, list):
        raise ValueError("profile.test_vectors must be an array")

    results: list[dict[str, Any]] = []
    for vector in vectors:
        vector_id = vector.get("id") if isinstance(vector, dict) else None
        scenario = vector.get("scenario") if isinstance(vector, dict) else None
        expected = {
            "disposition": vector.get("expected_disposition"),
            "reason": vector.get("expected_reason"),
        } if isinstance(vector, dict) else {"disposition": None, "reason": None}
        try:
            actual_disposition, actual_reason = evaluate_synthetic_vector(vector)
            actual = {"disposition": actual_disposition, "reason": actual_reason}
            error = None
        except (TypeError, ValueError) as exc:
            actual = None
            error = str(exc)
        results.append({
            "vector_id": vector_id,
            "scenario": scenario,
            "expected": expected,
            "actual": actual,
            "matched": actual == expected,
            "error": error,
            "synthetic_fixture": (
                vector.get("synthetic_fixture") is True if isinstance(vector, dict) else False
            ),
        })

    profile_source_sha256: str | None = None
    if profile_source_bytes is not None:
        if not isinstance(profile_source_bytes, bytes):
            raise ValueError("profile_source_bytes must be bytes")
        try:
            parsed_source = json.loads(
                profile_source_bytes.decode("utf-8"),
                object_pairs_hook=_reject_duplicate_object_keys,
            )
        except (UnicodeDecodeError, ValueError) as exc:
            raise ValueError("profile_source_bytes must contain valid UTF-8 JSON") from exc
        if parsed_source != profile:
            raise ValueError("profile_source_bytes do not match the supplied profile object")
        profile_source_sha256 = hashlib.sha256(profile_source_bytes).hexdigest()

    if source_commit_sha is not None and (
        not isinstance(source_commit_sha, str)
        or len(source_commit_sha) != 40
        or any(char not in "0123456789abcdef" for char in source_commit_sha)
    ):
        raise ValueError("source_commit_sha must be a lowercase 40-character Git commit SHA")

    normalized_profile = json.dumps(
        profile, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    engine_source = Path(__file__).read_bytes()
    validator_source = Path(validate_sa_did_profile.__file__).read_bytes()
    evidence_fingerprints = {
        "profile_file_sha256": profile_source_sha256,
        "profile_normalized_sha256": hashlib.sha256(normalized_profile).hexdigest(),
        "policy_engine_source_sha256": hashlib.sha256(engine_source).hexdigest(),
        "contract_validator_source_sha256": hashlib.sha256(validator_source).hexdigest(),
    }

    matched = sum(1 for item in results if item["matched"])
    return {
        "report_schema": "mycelix.sa-did-synthetic-policy-report",
        "schema_version": 3,
        "profile_id": profile.get("profile_id"),
        "source_commit_sha": source_commit_sha,
        "profile_maturity": profile.get("maturity"),
        "production_enabled": profile.get("production_enabled"),
        "purpose": "synthetic_policy_oracle_only",
        "cryptographic_verification_performed": False,
        "dha_endpoint_contacted": False,
        "evidence_fingerprints": evidence_fingerprints,
        "vectors_total": len(results),
        "vectors_matched": matched,
        "passed": bool(results) and matched == len(results),
        "results": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path(__file__).with_name("SA_DID_PROFILE_V1.json"),
    )
    parser.add_argument("--report", type=Path, help="write a machine-readable JSON report")
    parser.add_argument(
        "--source-commit",
        help="exact 40-character source commit SHA supplied by the CI workflow",
    )
    args = parser.parse_args()
    try:
        profile_bytes = args.profile.read_bytes()
        profile = json.loads(
            profile_bytes.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_object_keys,
        )
        report = build_report(
            profile,
            profile_source_bytes=profile_bytes,
            source_commit_sha=args.source_commit,
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: could not evaluate synthetic profile: {exc}", file=sys.stderr)
        return 2

    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.report is not None:
        try:
            args.report.write_text(serialized, encoding="utf-8")
        except OSError as exc:
            print(f"ERROR: could not write policy report: {exc}", file=sys.stderr)
            return 2

    status = "PASS" if report["passed"] else "FAIL"
    print(
        f"SA_DID_POLICY_ORACLE={status} "
        f"vectors={report['vectors_total']} matched={report['vectors_matched']} "
        "synthetic_only=true production_enabled=false "
        "cryptographic_verification=false"
    )
    return 0 if report["passed"] else 1


__all__ = ["evaluate_synthetic_vector", "build_report", "POLICY_INPUT_FIELDS"]


if __name__ == "__main__":
    raise SystemExit(main())
