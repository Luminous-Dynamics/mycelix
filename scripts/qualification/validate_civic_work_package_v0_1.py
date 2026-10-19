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

# ISO 4217 minor-unit scales for the deliberately bounded initial allowlist.
# Do not guess for unlisted currency codes: update against the current ISO 4217
# Maintenance Agency data before supporting another currency.
_SUPPORTED_CURRENCY_MINOR_UNIT_DIGITS = {
    "AUD": 2,
    "BHD": 3,
    "CAD": 2,
    "CHF": 2,
    "CNY": 2,
    "EUR": 2,
    "GBP": 2,
    "INR": 2,
    "JPY": 0,
    "KRW": 0,
    "KWD": 3,
    "NZD": 2,
    "OMR": 3,
    "USD": 2,
    "ZAR": 2,
}


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

    # Lifecycle gates remain binding after closure/cancellation. Otherwise a record
    # could reach authorization or execution and then evade its historical
    # funding, labor-protection, or independent-verification requirements by
    # changing only its current status to closed_or_cancelled.
    reached_statuses = {event.get("to_status") for event in events}
    funding_gate_reached = status in _FUNDED_STATUSES or bool(reached_statuses & _FUNDED_STATUSES)
    authorization_gate_reached = status in _AUTHORIZED_STATUSES or bool(reached_statuses & _AUTHORIZED_STATUSES)
    execution_gate_reached = status in _EXECUTION_STATUSES or bool(reached_statuses & _EXECUTION_STATUSES)
    verification_gate_reached = status == "independently_verified" or "independently_verified" in reached_statuses

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
    scope_period = record.get("scope", {}).get("period", {})
    cost_breakdown = funding.get("cost_breakdown", {})

    # Declared integer amounts only mean something when their scale matches the
    # currency. Reject unknown scales rather than silently applying a 10^n factor.
    currency = funding.get("currency")
    minor_unit_digits = funding.get("minor_unit_digits")
    if currency is not None or minor_unit_digits is not None:
        expected_digits = _SUPPORTED_CURRENCY_MINOR_UNIT_DIGITS.get(currency)
        if expected_digits is None:
            errors.append("CWP_E_UNSUPPORTED_CURRENCY_MINOR_UNIT_RULE")
        elif (
            not isinstance(minor_unit_digits, int)
            or isinstance(minor_unit_digits, bool)
            or minor_unit_digits != expected_digits
        ):
            errors.append("CWP_E_CURRENCY_MINOR_UNIT_DIGITS_MISMATCH")
    cost_items = cost_breakdown.get("cost_items", [])
    category_coverage = cost_breakdown.get("category_coverage", [])
    required_cost_categories = {
        "wages", "employer_statutory_costs", "supervision", "training", "tools_equipment",
        "protective_equipment", "accessibility", "transport", "insurance", "materials",
        "maintenance", "independent_verification", "administration_overheads",
        "community_engagement", "environmental_mitigation", "end_of_life_disposal",
        "permits_taxes_fees", "contingency", "other",
    }

    # Money is represented as integer minor units. Sum and compare exactly—never use
    # binary floating-point for the cost-coverage decision.
    line_ids: set[str] = set()
    costs_by_category: dict[str, int] = {}
    computed_total = 0
    scope_start = scope_end = None
    try:
        scope_start = date.fromisoformat(scope_period["start_date"]) if scope_period.get("start_date") else None
        scope_end = date.fromisoformat(scope_period["end_date"]) if scope_period.get("end_date") else None
        if scope_start and scope_end and scope_end < scope_start:
            errors.append("CWP_E_INVALID_SCOPE_PERIOD")
    except (TypeError, ValueError):
        errors.append("CWP_E_INVALID_SCOPE_PERIOD")

    for item in cost_items:
        line_id = item.get("line_item_id")
        if not line_id or line_id in line_ids:
            errors.append("CWP_E_DUPLICATE_COST_ITEM_ID")
        line_ids.add(line_id)
        amount_minor = item.get("amount_minor_units")
        if not isinstance(amount_minor, int) or isinstance(amount_minor, bool) or amount_minor < 0:
            errors.append("CWP_E_INVALID_COST_AMOUNT")
            continue
        computed_total += amount_minor
        category = item.get("category")
        costs_by_category[category] = costs_by_category.get(category, 0) + 1
        try:
            item_start = date.fromisoformat(item["period_start"])
            item_end = date.fromisoformat(item["period_end"])
            if item_end < item_start:
                errors.append("CWP_E_INVALID_COST_PERIOD")
            if scope_start and item_start < scope_start or scope_end and item_end > scope_end:
                errors.append("CWP_E_COST_PERIOD_OUT_OF_SCOPE")
        except (KeyError, TypeError, ValueError):
            errors.append("CWP_E_INVALID_COST_PERIOD")

    covered_categories: set[str] = set()
    for assessment in category_coverage:
        category = assessment.get("category")
        if category in covered_categories:
            errors.append("CWP_E_DUPLICATE_COST_CATEGORY_ASSESSMENT")
        covered_categories.add(category)
        line_count = costs_by_category.get(category, 0)
        disposition = assessment.get("disposition")
        if disposition == "included" and line_count == 0:
            errors.append("CWP_E_INCLUDED_CATEGORY_HAS_NO_COST_LINE")
        if disposition == "not_applicable" and line_count > 0:
            errors.append("CWP_E_NOT_APPLICABLE_CATEGORY_HAS_COST_LINE")
    if covered_categories != required_cost_categories:
        errors.append("CWP_E_COST_COVERAGE_INCOMPLETE")

    estimate = funding.get("full_period_cost_estimate_minor_units")
    amount = funding.get("committed_amount_minor_units")
    if isinstance(estimate, int) and not isinstance(estimate, bool):
        if computed_total != estimate:
            errors.append("CWP_E_COST_TOTAL_MISMATCH")
    elif funding.get("whole_period_fully_costed") is True or status in _FUNDED_STATUSES:
        errors.append("CWP_E_COST_TOTAL_MISSING")
    if funding.get("whole_period_fully_costed") is True:
        if covered_categories != required_cost_categories:
            errors.append("CWP_E_PERIOD_COSTING_INCOMPLETE")
        if not cost_breakdown.get("reviewed_by_ref") or not cost_breakdown.get("reviewed_at"):
            errors.append("CWP_E_COST_COVERAGE_REVIEW_MISSING")
    if funding_gate_reached:
        if funding.get("status") != "committed" or not funding.get("commitment_evidence_ref"):
            errors.append("CWP_E_FUNDING_NOT_COMMITTED")
        if not isinstance(amount, int) or isinstance(amount, bool) or not isinstance(estimate, int) or isinstance(estimate, bool) or amount < estimate:
            errors.append("CWP_E_COMMITMENT_DOES_NOT_COVER_FULL_PERIOD_COST")
        if funding.get("payer_ref") != commissioning.get("payer_ref") or not funding.get("payer_ref"):
            errors.append("CWP_E_PAYER_REFERENCE_MISMATCH")
        if funding.get("whole_period_fully_costed") is not True:
            errors.append("CWP_E_PERIOD_COSTING_INCOMPLETE")
        funded_start = funded_end = None
        try:
            funded_start = date.fromisoformat(funding["funded_period_start"])
            funded_end = date.fromisoformat(funding["funded_period_end"])
            if funded_end < funded_start:
                errors.append("CWP_E_INVALID_FUNDED_PERIOD")
            for item in cost_items:
                item_start = date.fromisoformat(item["period_start"])
                item_end = date.fromisoformat(item["period_end"])
                if item_start < funded_start or item_end > funded_end:
                    errors.append("CWP_E_COST_PERIOD_OUTSIDE_FUNDED_PERIOD")
        except (KeyError, TypeError, ValueError):
            errors.append("CWP_E_INVALID_FUNDED_PERIOD")
        if scope_start is None or scope_end is None:
            errors.append("CWP_E_FUNDED_SCOPE_PERIOD_MISSING")
        elif funded_start is not None and funded_end is not None and (
            funded_start > scope_start or funded_end < scope_end
        ):
            errors.append("CWP_E_FUNDED_PERIOD_DOES_NOT_COVER_SCOPE")
        paid_roles = [role for role in record.get("labor_plan", {}).get("roles", []) if role.get("paid") is True]
        if paid_roles and sum(item.get("amount_minor_units", 0) for item in cost_items if item.get("category") == "wages") <= 0:
            errors.append("CWP_E_PAID_ROLES_WITHOUT_POSITIVE_WAGES")
        for role in record.get("labor_plan", {}).get("roles", []):
            if role.get("paid") is True:
                cls = role.get("classification", {})
                if cls.get("status") != "reviewed" or not all(cls.get(k) for k in ("classification_code", "wage_basis_ref", "source_ref", "reviewer_ref", "reviewed_at")):
                    errors.append("CWP_E_PAID_ROLE_CLASSIFICATION_UNREVIEWED")
                if not role.get("terms_ref"):
                    errors.append("CWP_E_PAID_ROLE_TERMS_MISSING")

    if authorization_gate_reached:
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

    if execution_gate_reached:
        if record.get("community_review", {}).get("status") not in {"completed", "not_required"}:
            errors.append("CWP_E_COMMUNITY_REVIEW_GATE_UNMET")
        protection = record.get("protections", {})
        for key in ("written_terms_ref", "payment_schedule", "safety_plan_ref", "grievance_route_ref", "accessibility_review_ref"):
            if not protection.get(key):
                errors.append("CWP_E_WORKER_PROTECTION_MISSING")
        if protection.get("no_adverse_automated_decisions") is not True:
            errors.append("CWP_E_AUTOMATED_ADVERSE_DECISION_GUARD_MISSING")

    if verification_gate_reached:
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
        if not path.is_file():
            failures.append(f'FIXTURE_MISSING: {path_text}')
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
