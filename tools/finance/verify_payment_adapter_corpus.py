#!/usr/bin/env python3
"""Deterministic, stdlib-only verifier for Mycelix's payment adapter corpus.

This validates the corpus as a specification artifact. It does not call
providers, execute payments, or qualify a live integration.
"""
from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from pathlib import Path
from typing import Any

SCHEMA = "mycelix.payment-adapter-qualification/v1"
MIN_CASES = 27
CASE_ID_RE = re.compile(r"^PAY-[A-Z0-9]+-\d{3}$")
CASE_FIELDS = {
    "id", "adapter_class", "condition", "event_sequence",
    "expected_disposition", "external_dispatch_allowed",
    "expected_financial_effect", "required_evidence", "forbidden_claims",
}

# These contracts protect the highest-risk scenarios against accidental
# weakening while allowing non-critical cases to grow independently.
CRITICAL_CASES: dict[str, dict[str, Any]] = {
    "PAY-ISO-002": {
        "expected_disposition": "ProviderStatusObserved",
        "external_dispatch_allowed": None,
        "event_sequence_contains": ["positive_status_report_received"],
        "required_evidence_contains": ["status_code", "finality_profile_id"],
        "forbidden_claims_contains": ["provider_acknowledgement_equals_finality"],
    },
    "PAY-ISO-003": {
        "expected_disposition": "AppendSuccessorEffect",
        "event_sequence_contains": ["return_or_reversal_received"],
        "forbidden_claims_contains": ["history_may_be_overwritten"],
    },
    "PAY-ISO-004": {
        "expected_disposition": "PreDispatchValidationFailure",
        "external_dispatch_allowed": False,
        "condition_contains": ["2026-11-14", "Swift CBPR+"],
        "forbidden_claims_contains": ["rule_applies_to_every_iso20022_market"],
    },
    "PAY-VISA-001": {
        "expected_disposition": "IndeterminateUntilQueried",
        "external_dispatch_allowed": False,
        "event_sequence_contains": ["transport_timeout", "read_after_timeout"],
        "forbidden_claims_contains": [
            "timeout_means_provider_rejected",
            "retry_may_create_new_logical_payment",
        ],
    },
    "PAY-VISA-003": {
        "expected_disposition": "AppendReturnAndReconcile",
        "event_sequence_contains": ["return_notification_received"],
        "forbidden_claims_contains": ["prior_payout_evidence_is_deleted"],
    },
    "PAY-VISA-005": {
        "expected_disposition": "IdempotencyConflict",
        "external_dispatch_allowed": False,
        "event_sequence_contains": ["request_identity_comparison"],
        "required_evidence_contains": [
            "client_reference_id", "prior_request_digest", "new_request_digest",
            "prior_amount_and_currency", "new_amount_and_currency", "provider_error_or_query_result",
        ],
        "forbidden_claims_contains": [
            "changed_payload_under_same_client_reference_is_a_new_payment",
            "provider_conflict_can_be_bypassed_without_new_authorized_intent",
        ],
    },
    "PAY-IDEM-001": {
        "expected_disposition": "IndeterminateAfterProviderKeyExpiry",
        "external_dispatch_allowed": False,
        "event_sequence_contains": [
            "provider_key_retention_expired_or_unknown",
            "provider_query_or_reconciliation",
        ],
        "required_evidence_contains": [
            "provider_profile_id", "key_namespace_scope", "retention_policy_version",
            "first_submission_at", "idempotency_key_digest",
            "provider_reference_or_client_reference", "query_result_or_reconciliation_record",
        ],
        "forbidden_claims_contains": [
            "local_key_persistence_extends_provider_key_retention",
            "expired_or_unknown_provider_key_is_safe_to_replay",
            "same_key_is_globally_deduplicated_across_regions",
        ],
    },
    "PAY-CARD-001": {
        "expected_disposition": "AuthorizedNotSettled",
        "forbidden_claims_contains": ["authorization_equals_settlement"],
    },
    "PAY-CARD-002": {
        "expected_disposition": "IndeterminateCapture",
        "external_dispatch_allowed": False,
        "event_sequence_contains": ["transport_timeout", "processor_query_or_reconciliation"],
        "forbidden_claims_contains": ["timeout_means_capture_failed"],
    },
    "PAY-REGIONAL-001": {
        "expected_disposition": "RouteNotAdmitted",
        "external_dispatch_allowed": False,
        "forbidden_claims_contains": ["region_or_political_initiative_is_an_api"],
    },
    "PAY-REGIONAL-002": {
        "expected_disposition": "PartialEffectOrIndeterminate",
        "external_dispatch_allowed": False,
        "forbidden_claims_contains": ["one_leg_success_means_whole_route_settled"],
    },
    "PAY-MULTI-001": {
        "expected_disposition": "PartialEffectWithRecoveryPlan",
        "external_dispatch_allowed": False,
        "forbidden_claims_contains": ["saga_equals_atomicity", "partial_effect_may_be_hidden"],
    },
    "PAY-PANDA-001": {
        "expected_disposition": "UnqualifiedProvider",
        "external_dispatch_allowed": False,
        "required_evidence_contains": [
            "official_product_url", "legal_provider_identity", "settlement_counterparty",
        ],
    },
    "PAY-MYC-001": {
        "expected_disposition": "SingleOperationResolved",
        "event_sequence_contains": ["concurrent_payment_submissions", "claim_index_conflict_resolution"],
        "required_evidence_contains": ["winning_claim_hash", "outbox_effect_id"],
        "forbidden_claims_contains": ["each_retry_is_a_new_payment"],
    },
    "PAY-MYC-002": {
        "expected_disposition": "IdempotencyConflict",
        "external_dispatch_allowed": False,
        "event_sequence_contains": ["request_fingerprint_comparison"],
        "forbidden_claims_contains": [
            "changed_payload_under_same_sender_key_is_a_new_payment",
            "conflicting_retry_may_update_the_original_payment",
        ],
    },
    "PAY-TREASURY-001": {
        "expected_disposition": "ExistingReceiptResolved",
        "event_sequence_contains": ["treasury_receipt_committed", "payment_status_write_failed", "receipt_lookup"],
        "required_evidence_contains": ["destination_scoped_receipt_id", "balance_delta"],
        "forbidden_claims_contains": ["receipt_replay_adds_another_balance_credit"],
    },
    "PAY-TREASURY-002": {
        "expected_disposition": "UnanchoredEffectRejected",
        "external_dispatch_allowed": False,
        "required_evidence_contains": ["source_outbox_lookup_result", "payment_record_lookup_result", "balance_before", "balance_after"],
        "forbidden_claims_contains": ["effect_id_alone_authorizes_credit"],
    },
    "PAY-MYC-004": {
        "expected_disposition": "ChainHeadMovedRetryResolvesOriginal",
        "event_sequence_contains": ["two_zome_calls_read_same_snapshot", "second_transaction_returns_chain_head_moved", "losing_transaction_scratch_is_discarded"],
        "required_evidence_contains": ["chain_head_moved_error", "losing_transaction_rollback_evidence", "winning_claim_hash", "outbox_effect_id"],
        "forbidden_claims_contains": [
            "chain_head_moved_means_the_original_payment_did_not_commit",
            "losing_transaction_may_partially_commit",
            "retry_may_use_a_fresh_idempotency_key",
        ],
    },
    "PAY-MYC-003": {
        "expected_disposition": "SenderScopedOperationsIsolated",
        "event_sequence_contains": ["sender_scoped_operation_identity_resolution"],
        "required_evidence_contains": ["sender_A_identity", "sender_B_identity", "operation_id_A", "operation_id_B"],
        "forbidden_claims_contains": [
            "same_key_is_globally_unique_across_senders",
            "one_senders_retry_may_resolve_another_senders_payment",
        ],
    },
    "PAY-TREASURY-003": {
        "expected_disposition": "ConcurrentCreditsSerialized",
        "event_sequence_contains": ["concurrent_credit_attempts", "destination_balance_checked_arithmetically"],
        "required_evidence_contains": ["balance_before", "balance_after", "maximum_representable_balance", "destination_scoped_receipts"],
        "forbidden_claims_contains": ["competing_credits_may_overflow_shared_pool", "every_competing_credit_must_succeed"],
    },
}


def _contains_all(actual: Any, required: list[Any]) -> bool:
    return isinstance(actual, list) and all(item in actual for item in required)


def validate_corpus(document: Any) -> list[str]:
    """Return deterministic validation failures; empty means structural pass."""
    errors: list[str] = []
    if not isinstance(document, dict):
        return ["root must be a JSON object"]

    if document.get("schema") != SCHEMA:
        errors.append(f"schema must equal {SCHEMA!r}")
    if document.get("status") != "reference-corpus":
        errors.append("status must remain 'reference-corpus'")
    scope = document.get("scope")
    if not isinstance(scope, dict) or scope.get("mode") != "effects-disabled":
        errors.append("scope.mode must remain 'effects-disabled'")
    if not isinstance(document.get("profile_rule"), str) or not document["profile_rule"].strip():
        errors.append("profile_rule must be a non-empty string")

    source_refs = document.get("source_refs")
    if not isinstance(source_refs, list) or not source_refs:
        errors.append("source_refs must be a non-empty list")
    else:
        for i, source in enumerate(source_refs):
            if not isinstance(source, dict):
                errors.append(f"source_refs[{i}] must be an object")
                continue
            for field in ("id", "url", "relevance"):
                if not isinstance(source.get(field), str) or not source[field].strip():
                    errors.append(f"source_refs[{i}].{field} must be a non-empty string")
            if isinstance(source.get("url"), str) and not source["url"].startswith("https://"):
                errors.append(f"source_refs[{i}].url must use HTTPS")

    cases = document.get("cases")
    if not isinstance(cases, list):
        errors.append("cases must be a list")
        return errors
    if len(cases) < MIN_CASES:
        errors.append(f"cases must contain at least {MIN_CASES} scenarios; found {len(cases)}")

    by_id: dict[str, dict[str, Any]] = {}
    for i, case in enumerate(cases):
        prefix = f"cases[{i}]"
        if not isinstance(case, dict):
            errors.append(f"{prefix} must be an object")
            continue
        missing = sorted(CASE_FIELDS - set(case))
        if missing:
            errors.append(f"{prefix} missing fields: {', '.join(missing)}")
        case_id = case.get("id")
        if not isinstance(case_id, str) or not CASE_ID_RE.fullmatch(case_id):
            errors.append(f"{prefix}.id must match PAY-<FAMILY>-NNN")
        elif case_id in by_id:
            errors.append(f"duplicate case id: {case_id}")
        else:
            by_id[case_id] = case

        for field in ("adapter_class", "condition", "expected_disposition", "expected_financial_effect"):
            if not isinstance(case.get(field), str) or not case[field].strip():
                errors.append(f"{prefix}.{field} must be a non-empty string")
        for field in ("event_sequence", "required_evidence", "forbidden_claims"):
            value = case.get(field)
            if not isinstance(value, list) or not value:
                errors.append(f"{prefix}.{field} must be a non-empty list")
            elif any(not isinstance(item, str) or not item.strip() for item in value):
                errors.append(f"{prefix}.{field} must contain only non-empty strings")
        dispatch = case.get("external_dispatch_allowed")
        if dispatch is not None and not isinstance(dispatch, bool):
            errors.append(f"{prefix}.external_dispatch_allowed must be boolean or null")

    for case_id, contract in CRITICAL_CASES.items():
        case = by_id.get(case_id)
        if case is None:
            errors.append(f"missing critical scenario: {case_id}")
            continue
        for field, expected in contract.items():
            if field.endswith("_contains"):
                actual_field = field.removesuffix("_contains")
                actual = case.get(actual_field)
                if actual_field == "condition":
                    ok = isinstance(actual, str) and all(v in actual for v in expected)
                else:
                    ok = _contains_all(actual, expected)
                if not ok:
                    errors.append(f"{case_id}.{actual_field} must contain {expected!r}")
            elif case.get(field) != expected:
                errors.append(f"{case_id}.{field} must equal {expected!r}")
    return errors


def _mutate_case(document: dict[str, Any], case_id: str, field: str, value: Any) -> None:
    for case in document.get("cases", []):
        if isinstance(case, dict) and case.get("id") == case_id:
            case[field] = value
            return
    raise KeyError(f"case not found in test mutation: {case_id}")


def run_self_tests(document: dict[str, Any]) -> list[str]:
    """Mutation tests proving that known unsafe corpus changes are rejected."""
    baseline = validate_corpus(document)
    if baseline:
        return ["baseline corpus unexpectedly failed: " + "; ".join(baseline)]
    mutations: list[tuple[str, dict[str, Any]]] = []

    def changed(label: str, mutator: Any) -> None:
        altered = copy.deepcopy(document)
        mutator(altered)
        mutations.append((label, altered))

    changed("duplicate ID", lambda d: d["cases"].__setitem__(-1, {**d["cases"][-1], "id": d["cases"][0]["id"]}))
    changed("timeout blind retry regression", lambda d: _mutate_case(d, "PAY-VISA-001", "external_dispatch_allowed", True))
    changed("status-to-finality regression", lambda d: _mutate_case(d, "PAY-ISO-002", "expected_disposition", "Settled"))
    changed("return history rewrite regression", lambda d: _mutate_case(d, "PAY-ISO-003", "forbidden_claims", ["some-other-claim"]))
    changed("effects-enabled regression", lambda d: d["scope"].__setitem__("mode", "live-effects"))
    changed("missing-source lineage", lambda d: d.__setitem__("source_refs", []))
    changed("scenario removed", lambda d: d["cases"].pop())
    changed("missing required evidence", lambda d: _mutate_case(d, "PAY-VISA-001", "required_evidence", []))
    changed("conflicting local payload dispatch regression", lambda d: _mutate_case(d, "PAY-MYC-002", "external_dispatch_allowed", True))
    changed("Treasury effect-ID-as-authority regression", lambda d: _mutate_case(d, "PAY-TREASURY-002", "forbidden_claims", ["effect_id_is_authority"]))
    changed("receipt replay duplicate-credit regression", lambda d: _mutate_case(d, "PAY-TREASURY-001", "forbidden_claims", ["some-other-claim"]))
    changed("cross-sender idempotency scope regression", lambda d: _mutate_case(d, "PAY-MYC-003", "forbidden_claims", ["same_key_is_globally_unique_across_senders"]))
    changed("chain-head retry fresh-key regression", lambda d: _mutate_case(d, "PAY-MYC-004", "forbidden_claims", ["retry_may_use_a_fresh_idempotency_key"]))

    failures = [
        f"mutation was not rejected: {label}"
        for label, altered in mutations
        if not validate_corpus(altered)
    ]
    return failures


def main(argv: list[str] | None = None) -> int:
    script_path = Path(__file__).resolve()
    repo_root = script_path.parents[2] if len(script_path.parents) > 2 else script_path.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--corpus",
        type=Path,
        default=repo_root / "docs/finance/payment-adapter-qualification-v1.json",
        help="path to payment-adapter-qualification-v1.json",
    )
    parser.add_argument("--self-test", action="store_true", help="run mutation tests after corpus validation")
    args = parser.parse_args(argv)

    try:
        with args.corpus.open("r", encoding="utf-8") as handle:
            document = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"FAIL: cannot load corpus {args.corpus}: {exc}", file=sys.stderr)
        return 2

    errors = validate_corpus(document)
    if errors:
        print(f"FAIL: {len(errors)} validation error(s)")
        for error in errors:
            print(f" - {error}")
        return 1

    print(f"PASS: {len(document['cases'])} scenarios; unique IDs; critical safety contracts verified")
    if args.self_test:
        test_errors = run_self_tests(document)
        if test_errors:
            print(f"FAIL: {len(test_errors)} self-test error(s)")
            for error in test_errors:
                print(f" - {error}")
            return 1
        print("PASS: 13 negative mutation tests rejected unsafe corpus changes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
