#!/usr/bin/env python3
"""Fail-closed schema and semantic checks for synthetic civic work packages.

Reference/CI glue only. This is not an authorization, payroll, procurement,
legal-classification, identity-authentication, or real-world outcome service.
"""
from __future__ import annotations

import hashlib
import json
import sys
from datetime import date, datetime
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "docs/architecture/civic-work-package-v0.1.schema.json"
MANIFEST_PATH = ROOT / "docs/architecture/civic-work-package-fixtures/manifest.v0.1.json"

_ALLOWED_TRANSITIONS = {
    None: {"discovery", "proposed"},
    "discovery": {"proposed", "closed_or_cancelled"},
    "proposed": {"classification_review", "closed_or_cancelled"},
    "classification_review": {"proposed", "funding_committed", "closed_or_cancelled"},
    "funding_committed": {"classification_review", "authorized", "closed_or_cancelled"},
    "authorized": {"funding_committed", "announced", "closed_or_cancelled"},
    "announced": {"authorized", "active", "closed_or_cancelled"},
    "active": {"completion_claimed", "closed_or_cancelled"},
    "completion_claimed": {"active", "independently_verified", "closed_or_cancelled"},
    "independently_verified": {"active", "closed_or_cancelled"},
    "closed_or_cancelled": set(),
}
_FUNDED_STATUSES = {
    "funding_committed", "authorized", "announced", "active",
    "completion_claimed", "independently_verified",
}
_AUTHORIZED_STATUSES = {
    "authorized", "announced", "active", "completion_claimed", "independently_verified",
}
_EXECUTION_STATUSES = {"announced", "active", "completion_claimed", "independently_verified"}


def fail(message: str) -> None:
    raise ValueError(message)


def _unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            fail(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    fail(f"non-standard JSON numeric constant: {value}")


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(b'blob ' + str(len(data)).encode('ascii') + b'\0' + data).hexdigest()


def load_json(path: Path) -> Any:
    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_unique_pairs,
            parse_constant=_reject_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"INPUT_INVALID {path}: {exc}") from exc


def semantic_errors(record: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    status = record.get("status")
    events = record.get("lifecycle_events", [])
    seen_event_ids: set[str] = set()
    previous_status = None
    previous_time: datetime | None = None

    for event in events:
        event_id = event.get("event_id")
        if event_id in seen_event_ids:
            errors.append("CWP_E_DUPLICATE_EVENT_ID")
        seen_event_ids.add(event_id)
        from_status = event.get("from_status")
        to_status = event.get("to_status")
        if from_status != previous_status:
            errors.append("CWP_E_LIFECYCLE_FROM_STATUS_MISMATCH")
        if to_status not in _ALLOWED_TRANSITIONS.get(previous_status, set()):
            errors.append("CWP_E_INVALID_LIFECYCLE_TRANSITION")
        if event.get("actor_kind") == "automated" and to_status in (
            _FUNDED_STATUSES | _AUTHORIZED_STATUSES | _EXECUTION_STATUSES | {"closed_or_cancelled"}
        ):
            errors.append("CWP_E_AUTOMATED_GATED_TRANSITION")
        if to_status in {"funding_committed", "authorized", "announced", "active", "completion_claimed", "independently_verified"} and not event.get("evidence_refs"):
            errors.append("CWP_E_GATED_TRANSITION_WITHOUT_EVIDENCE")
        try:
            event_time = datetime.fromisoformat(event["happened_at"].replace("Z", "+00:00"))
            if previous_time is not None and event_time < previous_time:
                errors.append("CWP_E_NON_MONOTONIC_EVENT_TIME")
            previous_time = event_time
        except (KeyError, TypeError, ValueError, AttributeError):
            errors.append("CWP_E_INVALID_EVENT_TIME")
        previous_status = to_status

    if not events or previous_status != status:
        errors.append("CWP_E_FINAL_STATUS_MISMATCH")
    if events and events[0].get("from_status") is not None:
        errors.append("CWP_E_INITIAL_STATUS_NOT_NULL")

    funding = record.get("funding", {})
    commissioning = record.get("commissioning", {})
    if status in _FUNDED_STATUSES:
        amount = funding.get("committed_amount")
        estimate = funding.get("full_period_cost_estimate")
        if funding.get("status") != "committed" or not funding.get("commitment_evidence_ref"):
            errors.append("CWP_E_FUNDING_NOT_COMMITTED")
        if not isinstance(amount, (int, float)) or isinstance(amount, bool) or not isinstance(estimate, (int, float)) or isinstance(estimate, bool) or amount < estimate:
            errors.append("CWP_E_COMMITMENT_DOES_NOT_COVER_FULL_PERIOD_COST")
        if funding.get("payer_ref") != commissioning.get("payer_ref") or not funding.get("payer_ref"):
            errors.append("CWP_E_PAYER_REFERENCE_MISMATCH")
        if funding.get("whole_period_fully_costed") is not True:
            errors.append("CWP_E_PERIOD_COSTING_INCOMPLETE")
        for role in record.get("labor_plan", {}).get("roles", []):
            if role.get("paid") is True:
                cls = role.get("classification", {})
                if cls.get("status") != "reviewed" or not all(cls.get(k) for k in ("classification_code", "wage_basis_ref", "source_ref", "reviewer_ref", "reviewed_at")):
                    errors.append("CWP_E_PAID_ROLE_CLASSIFICATION_UNREVIEWED")
                if not role.get("terms_ref"):
                    errors.append("CWP_E_PAID_ROLE_TERMS_MISSING")

    if status in _AUTHORIZED_STATUSES:
        auth = record.get("authorization", {})
        if auth.get("status") != "approved" or auth.get("approved_by_human") is not True or not all(auth.get(k) for k in ("authorizer_ref", "authority_basis", "decision_ref", "decided_at")):
            errors.append("CWP_E_HUMAN_AUTHORIZATION_MISSING")
        if commissioning.get("status") != "authorized" or not all(commissioning.get(k) for k in ("commissioning_body_ref", "payer_ref", "authority_basis", "decision_ref")):
            errors.append("CWP_E_COMMISSIONING_AUTHORITY_MISSING")
        procurement = record.get("procurement", {})
        if procurement.get("lifecycle_stage") not in {"contracted", "implementation", "completed", "not_applicable"}:
            errors.append("CWP_E_PROCUREMENT_NOT_AUTHORIZED_FOR_WORK")
        if not procurement.get("source_ref"):
            errors.append("CWP_E_PROCUREMENT_EVIDENCE_MISSING")

    if status in _EXECUTION_STATUSES:
        if record.get("community_review", {}).get("status") not in {"completed", "not_required"}:
            errors.append("CWP_E_COMMUNITY_REVIEW_GATE_UNMET")
        protection = record.get("protections", {})
        for key in ("written_terms_ref", "payment_schedule", "safety_plan_ref", "grievance_route_ref", "accessibility_review_ref"):
            if not protection.get(key):
                errors.append("CWP_E_WORKER_PROTECTION_MISSING")
        if protection.get("no_adverse_automated_decisions") is not True:
            errors.append("CWP_E_AUTOMATED_ADVERSE_DECISION_GUARD_MISSING")

    if status == "independently_verified":
        verification = record.get("verification", {})
        reviewer = verification.get("reviewer_ref")
        execution_refs = set(record.get("labor_plan", {}).get("execution_actor_refs", []))
        if verification.get("status") != "completed" or verification.get("reviewer_is_independent") is not True or not reviewer or not verification.get("evidence"):
            errors.append("CWP_E_INDEPENDENT_VERIFICATION_INCOMPLETE")
        if reviewer in execution_refs:
            errors.append("CWP_E_PROVIDER_SELF_VERIFICATION")
        if not any(item.get("reviewed_by_ref") == reviewer for item in verification.get("evidence", [])):
            errors.append("CWP_E_REVIEWER_EVIDENCE_IDENTITY_MISMATCH")
        if any(event.get("actor_ref") == reviewer and event.get("to_status") in {"active", "completion_claimed"} for event in events):
            errors.append("CWP_E_REVIEWER_WAS_EXECUTION_ACTOR")

    if status == "closed_or_cancelled" and (not events or not events[-1].get("evidence_refs")):
        errors.append("CWP_E_CLOSURE_WITHOUT_EVIDENCE")

    if record.get("timestamps", {}).get("updated_at") and events:
        try:
            updated = datetime.fromisoformat(record["timestamps"]["updated_at"].replace("Z", "+00:00"))
            last = datetime.fromisoformat(events[-1]["happened_at"].replace("Z", "+00:00"))
            if updated < last:
                errors.append("CWP_E_UPDATED_BEFORE_LAST_EVENT")
        except (TypeError, ValueError, AttributeError):
            errors.append("CWP_E_INVALID_TIMESTAMP")

    try:
        scope_period = record["scope"]["period"]
        start = date.fromisoformat(scope_period["start_date"]) if scope_period.get("start_date") else None
        end = date.fromisoformat(scope_period["end_date"]) if scope_period.get("end_date") else None
        if start and end and end < start:
            errors.append("CWP_E_INVALID_SCOPE_PERIOD")
        funded_start = date.fromisoformat(funding["funded_period_start"]) if funding.get("funded_period_start") else None
        funded_end = date.fromisoformat(funding["funded_period_end"]) if funding.get("funded_period_end") else None
        if funded_start and funded_end and funded_end < funded_start:
            errors.append("CWP_E_INVALID_FUNDED_PERIOD")
    except (KeyError, TypeError, ValueError):
        errors.append("CWP_E_INVALID_DATE")

    return sorted(set(errors))


def validate_record(record: Any, schema: dict[str, Any], label: str = "record") -> list[str]:
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    errors = []
    for error in validator.iter_errors(record):
        path = "$" + "".join(f"[{part}]" if isinstance(part, int) else f".{part}" for part in error.absolute_path)
        errors.append(f"SCHEMA {label}{path}: {error.message}")
    if not errors and isinstance(record, dict):
        errors.extend(semantic_errors(record))
    return sorted(errors)


def main() -> int:
    schema = load_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    manifest = load_json(MANIFEST_PATH)
    expected_schema_sha = manifest.get('schema_git_blob_sha')
    actual_schema_sha = git_blob_sha(SCHEMA_PATH.read_bytes())
    if actual_schema_sha != expected_schema_sha:
        raise ValueError(f'SCHEMA_BLOB_PIN_MISMATCH expected={expected_schema_sha} actual={actual_schema_sha}')
    entries = manifest.get("fixtures")
    if manifest.get("manifest_version") != "civic-work-package-fixture-manifest/0.1" or not isinstance(entries, list) or not entries:
        raise ValueError("MANIFEST_INVALID: missing versioned fixture list")
    seen_paths = set()
    failures = []
    for entry in entries:
        path_text = entry.get("path")
        if not isinstance(path_text, str) or path_text in seen_paths:
            failures.append(f"MANIFEST_DUPLICATE_OR_INVALID_PATH: {path_text!r}")
            continue
        seen_paths.add(path_text)
        path = (ROOT / path_text).resolve()
        if ROOT.resolve() not in path.parents:
            failures.append(f"MANIFEST_PATH_ESCAPE: {path_text}")
            continue
        expected_blob_sha = entry.get('git_blob_sha')
        actual_blob_sha = git_blob_sha(path.read_bytes())
        if actual_blob_sha != expected_blob_sha:
            failures.append(f'FIXTURE_BLOB_PIN_MISMATCH {path_text}: expected={expected_blob_sha} actual={actual_blob_sha}')
        record = load_json(path)
        errors = validate_record(record, schema, path_text)
        if entry.get("expect_valid") is True and errors:
            failures.append(f"EXPECTED_VALID_FAILED {path_text}: " + "; ".join(errors))
        elif entry.get("expect_valid") is False and not errors:
            failures.append(f"EXPECTED_INVALID_PASSED {path_text}")
        elif entry.get("status") != record.get("status"):
            failures.append(f"FIXTURE_STATUS_MISMATCH {path_text}")
    expected_paths = {
        "docs/architecture/civic-work-package-fixtures/proposed.synthetic.v0.1.json",
        "docs/architecture/civic-work-package-fixtures/authorized.synthetic.v0.1.json",
    }
    if seen_paths != expected_paths:
        failures.append("MANIFEST_FIXTURE_CENSUS_MISMATCH")
    if failures:
        print("\n".join(f"FAIL: {item}" for item in failures), file=sys.stderr)
        return 1
    print(f"PASS: schema valid; {len(entries)} positive fixtures; {len(manifest.get('required_negative_controls', []))} required mutation controls registered.")
    print("Claim ceiling: synthetic contract and semantic consistency only; no legal, funding, identity, procurement, or operational qualification.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
