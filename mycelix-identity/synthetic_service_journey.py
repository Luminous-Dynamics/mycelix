#!/usr/bin/env python3
"""Execute a deterministic, synthetic-only public-service identity journey.

This harness exercises policy sequencing and minimal-disclosure outcomes. It
does not parse credentials, verify cryptographic signatures, contact external
verifiers, make eligibility decisions, or persist operational appeal records.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

CORPUS_PATH = Path(__file__).with_name("SYNTHETIC_SERVICE_JOURNEY_V1.json")
SCHEMA_ID = "org.luminousdynamics.synthetic-service-journey"
EXPECTED_CASE_COUNT = 18
FROZEN_EXPECTATIONS = {
    "SVC-001-MINIMAL-SUCCESS": ("MINIMAL_DISCLOSURE", "PURPOSE_SCOPE_VALID", "digital_minimal_disclosure", ("synthetic_subject_reference",)),
    "SVC-002-ENTITLEMENT-INFERENCE": ("DENIED", "IDENTITY_NOT_ENTITLEMENT", "manual_review_required", ()),
    "SVC-003-PURPOSE-MISMATCH": ("DENIED", "PURPOSE_GRANT_INVALID", "manual_review_required", ()),
    "SVC-004-EXCESSIVE-CLAIM": ("DENIED", "CLAIM_OUTSIDE_PURPOSE_SCOPE", "manual_review_required", ()),
    "SVC-005-VERIFIER-OUTAGE": ("INDETERMINATE", "VERIFICATION_SOURCE_UNAVAILABLE", "assisted_or_non_digital", ()),
    "SVC-006-STALE-STATUS": ("INDETERMINATE", "CREDENTIAL_STATUS_NOT_CURRENT", "assisted_or_non_digital", ()),
    "SVC-007-INVALID-PROOF": ("REJECTED", "PROOF_INVALID", "correction_and_appeal", ()),
    "SVC-008-AUDIENCE-MISMATCH": ("REJECTED", "AUDIENCE_MISMATCH", "correction_and_appeal", ()),
    "SVC-009-REPLAYED-REQUEST": ("REJECTED", "REPLAY_DETECTED", "correction_and_appeal", ()),
    "SVC-010-HOLDER-BINDING-MISMATCH": ("REJECTED", "HOLDER_BINDING_MISMATCH", "correction_and_appeal", ()),
    "SVC-011-UNTRUSTED-ISSUER": ("REJECTED", "ISSUER_NOT_TRUSTED", "correction_and_appeal", ()),
    "SVC-012-UNKNOWN-ISSUER-TRUST": ("INDETERMINATE", "ISSUER_TRUST_UNKNOWN", "assisted_or_non_digital", ()),
    "SVC-013-UNKNOWN-PROOF-STATE": ("INDETERMINATE", "PROOF_STATUS_UNKNOWN", "assisted_or_non_digital", ()),
    "SVC-014-REVOKED-CREDENTIAL": ("REJECTED", "CREDENTIAL_REVOKED", "correction_and_appeal", ()),
    "SVC-015-NO-DIGITAL-ACCESS": ("INDETERMINATE", "DIGITAL_ROUTE_UNAVAILABLE", "assisted_or_non_digital", ()),
    "SVC-016-NO-FALLBACK-AVAILABLE": ("INDETERMINATE", "VERIFICATION_SOURCE_UNAVAILABLE", "manual_review_required", ()),
    "SVC-017-REQUIRED-CLAIM-MISSING": ("REJECTED", "REQUESTED_CLAIM_UNAVAILABLE", "correction_and_appeal", ()),
    "SVC-018-UNKNOWN-PURPOSE-GRANT": ("DENIED", "PURPOSE_GRANT_INVALID", "manual_review_required", ()),
}
SOURCE_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
DISPOSITIONS = {"MINIMAL_DISCLOSURE", "DENIED", "REJECTED", "INDETERMINATE"}
ROUTES = {
    "digital_minimal_disclosure",
    "assisted_or_non_digital",
    "correction_and_appeal",
    "manual_review_required",
}
REQUIRED_CORPUS_FIELDS = {
    "schema_id", "schema_version", "maturity", "production_enabled",
    "no_external_calls", "no_real_identity_data", "service_model",
    "permitted_claim_names", "forbidden_persisted_material", "route_values",
    "disposition_values", "nonclaims", "cases",
}
REQUEST_FIELDS = {
    "service_id", "purpose", "requested_outcome", "requested_claims",
    "purpose_grant", "evidence", "access",
}
GRANT_FIELDS = {"grant_id", "service_id", "purpose", "status", "allowed_claims"}
EVIDENCE_FIELDS = {
    "source_state", "proof_state", "issuer_trust", "status_state",
    "audience_match", "nonce_fresh", "holder_binding_match", "available_claims",
}
ACCESS_FIELDS = {"digital_available", "assisted_available", "non_digital_available"}
EXPECTED_FIELDS = {"disposition", "reason_code", "route", "disclosed_claim_names", "appeal_receipt_required"}
CASE_FIELDS = {"case_id", "request", "expected"}
FORBIDDEN_KEYS = {
    "raw_credential", "credential_bytes", "credential_payload", "national_identifier_value",
    "national_id_number", "biometric_template", "biometric_data", "replayable_presentation",
    "claim_values", "subject_identifier", "person_name", "date_of_birth_value",
}


def _reject_duplicate_object_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key}")
        result[key] = value
    return result


def _is_string_list(value: Any, *, non_empty: bool = False) -> bool:
    return (
        isinstance(value, list)
        and (not non_empty or len(value) > 0)
        and all(isinstance(item, str) and bool(item.strip()) for item in value)
        and len(value) == len(set(value))
    )


def _contains_forbidden_key(value: Any) -> bool:
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str):
                return True
            normalized = key.strip().lower()
            if normalized in FORBIDDEN_KEYS or _contains_forbidden_key(child):
                return True
    elif isinstance(value, list):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def validate_request(request: Any) -> list[str]:
    """Check the closed synthetic input shape before evaluating policy."""
    if not isinstance(request, dict):
        return ["request must be an object"]
    errors: list[str] = []
    if set(request) != REQUEST_FIELDS:
        errors.append("request fields must match the closed v1 schema")
    for field in ("service_id", "purpose", "requested_outcome"):
        if not isinstance(request.get(field), str) or not request[field].strip():
            errors.append(f"request.{field} must be non-empty text")
    outcome_value = request.get("requested_outcome")
    if not isinstance(outcome_value, str) or outcome_value not in {"proof_only", "benefits_eligibility", "citizenship_decision", "residency_decision"}:
        errors.append("request.requested_outcome must use the closed v1 enum")
    if not _is_string_list(request.get("requested_claims"), non_empty=True):
        errors.append("request.requested_claims must be a non-empty list of unique claim names")

    grant = request.get("purpose_grant")
    if not isinstance(grant, dict):
        errors.append("request.purpose_grant must be an object")
    else:
        if set(grant) != GRANT_FIELDS:
            errors.append("purpose_grant fields must match the closed v1 schema")
        for field in ("grant_id", "service_id", "purpose", "status"):
            if not isinstance(grant.get(field), str) or not grant[field].strip():
                errors.append(f"purpose_grant.{field} must be non-empty text")
        if not _is_string_list(grant.get("allowed_claims"), non_empty=True):
            errors.append("purpose_grant.allowed_claims must be a non-empty list of unique claim names")

    evidence = request.get("evidence")
    if not isinstance(evidence, dict):
        errors.append("request.evidence must be an object")
    else:
        if set(evidence) != EVIDENCE_FIELDS:
            errors.append("evidence fields must match the closed v1 schema")
        for key, choices in {
            "source_state": {"available", "unavailable", "unknown"},
            "proof_state": {"valid", "invalid", "unknown"},
            "issuer_trust": {"trusted", "untrusted", "unknown"},
            "status_state": {"fresh", "stale", "unknown", "revoked"},
        }.items():
            value = evidence.get(key)
            if not isinstance(value, str) or value not in choices:
                errors.append(f"evidence.{key} must use the closed v1 enum")
        for key in ("audience_match", "nonce_fresh", "holder_binding_match"):
            if type(evidence.get(key)) is not bool:
                errors.append(f"evidence.{key} must be a boolean")
        if not _is_string_list(evidence.get("available_claims")):
            errors.append("evidence.available_claims must be a list of unique claim names")

    access = request.get("access")
    if not isinstance(access, dict):
        errors.append("request.access must be an object")
    else:
        if set(access) != ACCESS_FIELDS:
            errors.append("access fields must match the closed v1 schema")
        for key in ACCESS_FIELDS:
            if type(access.get(key)) is not bool:
                errors.append(f"access.{key} must be a boolean")

    if _contains_forbidden_key(request):
        errors.append("request must not contain raw identity or credential material")
    return errors


def validate_corpus(corpus: Any) -> list[str]:
    """Validate closed corpus metadata and every expected synthetic case."""
    if not isinstance(corpus, dict):
        return ["corpus must be a JSON object"]
    errors: list[str] = []
    if set(corpus) != REQUIRED_CORPUS_FIELDS:
        errors.append("top-level fields must match the closed v1 corpus schema")
    if corpus.get("schema_id") != SCHEMA_ID:
        errors.append("unexpected schema_id")
    if type(corpus.get("schema_version")) is not int or corpus.get("schema_version") != 1:
        errors.append("schema_version must be the integer 1")
    if corpus.get("maturity") != "synthetic_only":
        errors.append("maturity must remain synthetic_only")
    if corpus.get("production_enabled") is not False:
        errors.append("production_enabled must be false")
    for key in ("no_external_calls", "no_real_identity_data"):
        if corpus.get(key) is not True:
            errors.append(f"{key} must preserve the research boundary")
    if corpus.get("service_model") != "appointment-intake-proof-only":
        errors.append("service_model must remain appointment-intake-proof-only")

    claim_names = corpus.get("permitted_claim_names")
    if not _is_string_list(claim_names, non_empty=True) or set(claim_names) != {"synthetic_subject_reference"}:
        errors.append("permitted_claim_names must contain only synthetic_subject_reference")
    forbidden = corpus.get("forbidden_persisted_material")
    if not _is_string_list(forbidden, non_empty=True):
        errors.append("forbidden_persisted_material must be a non-empty list")
    route_values = corpus.get("route_values")
    if not isinstance(route_values, list) or any(not isinstance(value, str) for value in route_values) or set(route_values) != ROUTES:
        errors.append("route_values must match the closed v1 enum")
    disposition_values = corpus.get("disposition_values")
    if not isinstance(disposition_values, list) or any(not isinstance(value, str) for value in disposition_values) or set(disposition_values) != DISPOSITIONS:
        errors.append("disposition_values must match the closed v1 enum")
    notes = corpus.get("nonclaims")
    if not isinstance(notes, list) or len(notes) < 3 or any(not isinstance(note, str) or not note.strip() for note in notes):
        errors.append("nonclaims must be at least three non-empty strings")

    cases = corpus.get("cases")
    if not isinstance(cases, list) or len(cases) != EXPECTED_CASE_COUNT:
        errors.append(f"cases must contain exactly {EXPECTED_CASE_COUNT} frozen vectors")
        return errors
    seen: set[str] = set()
    for index, case in enumerate(cases):
        if not isinstance(case, dict):
            errors.append(f"cases[{index}] must be an object")
            continue
        if set(case) != CASE_FIELDS:
            errors.append(f"cases[{index}] fields must match the closed v1 schema")
        case_id = case.get("case_id")
        if not isinstance(case_id, str) or not case_id.strip():
            errors.append(f"cases[{index}].case_id must be non-empty text")
        elif case_id in seen:
            errors.append(f"duplicate case_id: {case_id}")
        else:
            seen.add(case_id)
        if isinstance(case_id, str) and case_id not in FROZEN_EXPECTATIONS:
            errors.append(f"cases[{index}].case_id is not in the frozen scenario registry")
        errors.extend(f"cases[{index}]: {message}" for message in validate_request(case.get("request")))
        expected = case.get("expected")
        if not isinstance(expected, dict):
            errors.append(f"cases[{index}].expected must be an object")
        else:
            if set(expected) != EXPECTED_FIELDS:
                errors.append(f"cases[{index}].expected fields must match the closed v1 schema")
            expected_disposition = expected.get("disposition")
            if not isinstance(expected_disposition, str) or expected_disposition not in DISPOSITIONS:
                errors.append(f"cases[{index}].expected.disposition must use the closed v1 enum")
            if not isinstance(expected.get("reason_code"), str) or not expected["reason_code"].strip():
                errors.append(f"cases[{index}].expected.reason_code must be non-empty text")
            expected_route = expected.get("route")
            if not isinstance(expected_route, str) or expected_route not in ROUTES:
                errors.append(f"cases[{index}].expected.route must use the closed v1 enum")
            if not _is_string_list(expected.get("disclosed_claim_names")):
                errors.append(f"cases[{index}].expected.disclosed_claim_names must be a unique string list")
            if expected.get("appeal_receipt_required") is not True:
                errors.append(f"cases[{index}] must require an appeal receipt")
            if isinstance(case_id, str) and case_id in FROZEN_EXPECTATIONS:
                frozen = FROZEN_EXPECTATIONS[case_id]
                observed = (
                    expected_disposition,
                    expected.get("reason_code"),
                    expected_route,
                    tuple(expected.get("disclosed_claim_names", [])) if _is_string_list(expected.get("disclosed_claim_names")) else (),
                )
                if observed != frozen:
                    errors.append(f"cases[{index}] expected outcome differs from frozen scenario contract")
            disclosed = expected.get("disclosed_claim_names")
            if expected_disposition == "MINIMAL_DISCLOSURE" and (
                not _is_string_list(disclosed, non_empty=True)
                or set(disclosed) != {"synthetic_subject_reference"}
            ):
                errors.append(f"cases[{index}] successful outcome must disclose only the synthetic subject reference")
            if expected.get("disposition") != "MINIMAL_DISCLOSURE" and expected.get("disclosed_claim_names") != []:
                errors.append(f"cases[{index}] unsuccessful outcome must disclose no claims")
    if seen != set(FROZEN_EXPECTATIONS):
        errors.append("case IDs must match the complete frozen scenario registry")
    if _contains_forbidden_key(corpus):
        errors.append("corpus must not include raw identity, credential, or claim-value fields")
    return errors


def _fallback_route(request: dict[str, Any]) -> str:
    access = request["access"]
    return "assisted_or_non_digital" if access["assisted_available"] or access["non_digital_available"] else "manual_review_required"


def _result(disposition: str, reason_code: str, route: str, disclosed_claim_names: list[str] | None = None) -> dict[str, Any]:
    return {
        "disposition": disposition,
        "reason_code": reason_code,
        "route": route,
        "disclosed_claim_names": disclosed_claim_names or [],
        "appeal_receipt_required": True,
    }


def evaluate_request(case_id: str, request: Any) -> dict[str, Any]:
    """Evaluate a proof-only request and return only minimal outcome metadata."""
    errors = validate_request(request)
    if errors:
        outcome = _result("INDETERMINATE", "MALFORMED_SYNTHETIC_REQUEST", "manual_review_required")
    else:
        grant = request["purpose_grant"]
        evidence = request["evidence"]

        # Identity proof must never be reused as a benefits, citizenship,
        # residency, entitlement, or other legal-status decision.
        if request["requested_outcome"] != "proof_only":
            outcome = _result("DENIED", "IDENTITY_NOT_ENTITLEMENT", "manual_review_required")
        elif (
            grant["status"] != "synthetic_test_approved"
            or grant["service_id"] != request["service_id"]
            or grant["purpose"] != request["purpose"]
            or not grant["grant_id"].startswith("SYNTHETIC-")
        ):
            outcome = _result("DENIED", "PURPOSE_GRANT_INVALID", "manual_review_required")
        elif not set(request["requested_claims"]).issubset(set(grant["allowed_claims"])):
            outcome = _result("DENIED", "CLAIM_OUTSIDE_PURPOSE_SCOPE", "manual_review_required")
        elif not set(request["requested_claims"]).issubset({"synthetic_subject_reference"}):
            outcome = _result("DENIED", "CLAIM_OUTSIDE_PURPOSE_SCOPE", "manual_review_required")
        elif not request["access"]["digital_available"]:
            outcome = _result("INDETERMINATE", "DIGITAL_ROUTE_UNAVAILABLE", _fallback_route(request))
        elif evidence["source_state"] != "available":
            reason = "VERIFICATION_SOURCE_UNAVAILABLE" if evidence["source_state"] == "unavailable" else "VERIFICATION_SOURCE_UNKNOWN"
            outcome = _result("INDETERMINATE", reason, _fallback_route(request))
        elif evidence["issuer_trust"] == "unknown":
            outcome = _result("INDETERMINATE", "ISSUER_TRUST_UNKNOWN", _fallback_route(request))
        elif evidence["issuer_trust"] == "untrusted":
            outcome = _result("REJECTED", "ISSUER_NOT_TRUSTED", "correction_and_appeal")
        elif evidence["proof_state"] == "unknown":
            outcome = _result("INDETERMINATE", "PROOF_STATUS_UNKNOWN", _fallback_route(request))
        elif evidence["proof_state"] == "invalid":
            outcome = _result("REJECTED", "PROOF_INVALID", "correction_and_appeal")
        elif evidence["status_state"] in {"stale", "unknown"}:
            outcome = _result("INDETERMINATE", "CREDENTIAL_STATUS_NOT_CURRENT", _fallback_route(request))
        elif evidence["status_state"] == "revoked":
            outcome = _result("REJECTED", "CREDENTIAL_REVOKED", "correction_and_appeal")
        elif not evidence["audience_match"]:
            outcome = _result("REJECTED", "AUDIENCE_MISMATCH", "correction_and_appeal")
        elif not evidence["nonce_fresh"]:
            outcome = _result("REJECTED", "REPLAY_DETECTED", "correction_and_appeal")
        elif not evidence["holder_binding_match"]:
            outcome = _result("REJECTED", "HOLDER_BINDING_MISMATCH", "correction_and_appeal")
        elif not set(request["requested_claims"]).issubset(set(evidence["available_claims"])):
            outcome = _result("REJECTED", "REQUESTED_CLAIM_UNAVAILABLE", "correction_and_appeal")
        else:
            outcome = _result("MINIMAL_DISCLOSURE", "PURPOSE_SCOPE_VALID", "digital_minimal_disclosure", request["requested_claims"])

    # Receipt content is an allow-list, not a copy of the request, credential,
    # returned claim values, or verifier response. References are synthetic.
    receipt = {
        "receipt_id": f"SYNTHETIC-RECEIPT-{case_id}",
        "service_id": request.get("service_id") if isinstance(request, dict) and isinstance(request.get("service_id"), str) else "unknown-service",
        "purpose": request.get("purpose") if isinstance(request, dict) and isinstance(request.get("purpose"), str) else "unknown-purpose",
        "disposition": outcome["disposition"],
        "reason_code": outcome["reason_code"],
        "disclosed_claim_names": list(outcome["disclosed_claim_names"]),
        "appeal_reference": f"SYNTHETIC-APPEAL-{case_id}",
        "receipt_persistence": "ephemeral_output_only",
        "contains_identity_material": False,
    }
    return {"case_id": case_id, **outcome, "receipt": receipt}


def run_corpus(corpus: dict[str, Any], source_commit: str) -> dict[str, Any]:
    """Return deterministic evidence bound to the exact source and corpus bytes."""
    errors = validate_corpus(corpus)
    if errors:
        raise ValueError("invalid synthetic service-journey corpus: " + "; ".join(errors))
    if not isinstance(source_commit, str) or not SOURCE_COMMIT_RE.fullmatch(source_commit):
        raise ValueError("source_commit must be a 40-character lowercase commit SHA")
    actual_cases = []
    for case in corpus["cases"]:
        actual = evaluate_request(case["case_id"], case["request"])
        expected = case["expected"]
        matched = all(actual[key] == expected[key] for key in EXPECTED_FIELDS)
        actual_cases.append({
            "case_id": case["case_id"],
            "expected": expected,
            "actual": {key: actual[key] for key in EXPECTED_FIELDS},
            "matched": matched,
            "receipt": actual["receipt"],
        })
    engine_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    canonical_corpus = json.dumps(corpus, sort_keys=True, separators=(",", ":")).encode("utf-8")
    corpus_sha = hashlib.sha256(canonical_corpus).hexdigest()
    return {
        "report_schema": "org.luminousdynamics.synthetic-service-journey-report",
        "report_version": 1,
        "source_commit": source_commit,
        "engine_sha256": engine_sha,
        "corpus_sha256": corpus_sha,
        "maturity": "synthetic_only",
        "production_enabled": False,
        "external_endpoint_contacted": False,
        "real_identity_data_processed": False,
        "all_expected_outcomes_match": all(row["matched"] for row in actual_cases),
        "case_count": len(actual_cases),
        "case_results": actual_cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", type=Path, default=CORPUS_PATH)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        corpus = json.loads(args.corpus.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_object_keys)
        # run_corpus pins the checked-in corpus bytes; custom corpus paths are
        # useful for validation tests, not for release evidence.
        errors = validate_corpus(corpus)
        if errors:
            raise ValueError("invalid corpus: " + "; ".join(errors))
        report = run_corpus(corpus, args.source_commit)
    except (OSError, ValueError) as exc:
        print(f"SYNTHETIC_SERVICE_JOURNEY=FAIL {exc}", file=sys.stderr)
        return 2
    if args.report:
        try:
            args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except OSError as exc:
            print(f"SYNTHETIC_SERVICE_JOURNEY=FAIL could not write report: {exc}", file=sys.stderr)
            return 2
    if not report["all_expected_outcomes_match"]:
        failed = [row["case_id"] for row in report["case_results"] if not row["matched"]]
        print(f"SYNTHETIC_SERVICE_JOURNEY=FAIL mismatched_cases={','.join(failed)}", file=sys.stderr)
        return 1
    print(
        "SYNTHETIC_SERVICE_JOURNEY=PASS "
        f"cases={report['case_count']} production_enabled=false "
        "external_endpoint_contacted=false real_identity_data_processed=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
