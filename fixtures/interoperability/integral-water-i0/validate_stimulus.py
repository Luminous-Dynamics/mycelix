#!/usr/bin/env python3
"""Validate MYC-INT-006E executable I0 stimulus without exposing the oracle."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

STIMULUS_PROFILE = "myc-int-006e-i0-stimulus-validator-v1"
STIMULUS_ID = "myc-int-006e-water-i0-stimulus"
STIMULUS_VERSION = "1.0.0"
PROFILE = "runtime-neutral-executable-stimulus-v1"
CORPUS_ID = "myc-int-006c-water-i0"
CORPUS_VERSION = "1.0.0"

EXPECTED_CASE_IDS = frozenset({"I0-POS-001", *(f"I0-HOSTILE-{i:03d}" for i in range(1, 19))})
OPERATIONS = frozenset({
    "RecordLineage",
    "ExecuteRecommendation",
    "ImportForeignDecisionAsLocalAuthority",
    "UseCredentialAsAuthority",
    "DeliverImplementationAttempt",
    "ClassifyTimeoutOutcome",
    "AdmitStaleSchema",
    "AcceptExternalCertificationLocally",
    "TreatDerivedSummaryAsSourceFact",
    "ExecuteWithExpiredAuthorization",
    "RewriteHistoricalDecisionFromOutcome",
    "ProcessReorderedTransport",
    "CollapseSchemaQualifiedIdentity",
    "TranslateWithUndeclaredLoss",
    "ImportForeignAuthorityAsLocal",
    "ResolvePartitionByLastArrival",
    "PromotePredictionToObservation",
    "PromoteDeliveryReceiptToImplementationReceipt",
    "PromoteExecutionReceiptToDesiredOutcome",
})
FORBIDDEN_KEYS = frozenset({
    "expected_disposition", "expected_result", "expected", "oracle",
    "reason_code", "assertion", "assertions", "predicate", "predicates",
})
ROOT_KEYS = frozenset({
    "stimulus_id", "stimulus_version", "profile", "corpus_id",
    "corpus_version", "cases", "nonclaims",
})
CASE_KEYS = frozenset({"operation", "subjects", "transport", "authority", "schema", "translation"})
CONDITION_KEYS = {
    "transport": frozenset({
        "delivery_count", "semantic_attempt_count", "receiver_persistence",
        "acknowledgement", "sender_result", "delivery_order",
        "total_order_promised", "partitioned", "reconnected", "proposed_resolution",
    }),
    "authority": frozenset({"status"}),
    "schema": frozenset({
        "supplied_generation", "required_generation",
        "visible_identifier_equal", "source_schema_equal",
    }),
    "translation": frozenset({"declared_loss", "actual_loss"}),
}


class StimulusValidationError(ValueError):
    pass


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise StimulusValidationError(f"duplicate JSON object member: {key!r}")
        out[key] = value
    return out


def parse_strict_json(raw: bytes, *, label: str) -> Any:
    try:
        return json.loads(raw.decode("utf-8", errors="strict"), object_pairs_hook=_strict_object)
    except StimulusValidationError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise StimulusValidationError(f"{label}: invalid JSON/UTF-8: {exc}") from exc


def _walk_forbidden(value: Any, ctx: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_KEYS or key.startswith("expected_"):
                raise StimulusValidationError(f"{ctx}: forbidden oracle-bearing key {key!r}")
            _walk_forbidden(child, f"{ctx}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_forbidden(child, f"{ctx}[{index}]")


def _mapping(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise StimulusValidationError(f"{ctx}: expected object")
    return value


def _nonempty_string(value: Any, ctx: str) -> str:
    if not isinstance(value, str) or not value:
        raise StimulusValidationError(f"{ctx}: expected non-empty string")
    return value


def validate_documents(stimulus: Any, corpus: Any) -> dict[str, int | str]:
    root = _mapping(stimulus, "stimulus")
    _walk_forbidden(root)
    if set(root) != ROOT_KEYS:
        raise StimulusValidationError(
            f"stimulus: field set drifted; missing={sorted(ROOT_KEYS-set(root))}, "
            f"extra={sorted(set(root)-ROOT_KEYS)}"
        )
    constants = {
        "stimulus_id": STIMULUS_ID,
        "stimulus_version": STIMULUS_VERSION,
        "profile": PROFILE,
        "corpus_id": CORPUS_ID,
        "corpus_version": CORPUS_VERSION,
    }
    for key, expected in constants.items():
        if root[key] != expected:
            raise StimulusValidationError(f"stimulus.{key}: expected {expected!r}, got {root[key]!r}")

    corpus_root = _mapping(corpus, "corpus")
    if corpus_root.get("corpus_id") != CORPUS_ID or corpus_root.get("corpus_version") != CORPUS_VERSION:
        raise StimulusValidationError("corpus identity/version does not match stimulus binding")
    corpus_cases = _mapping(corpus_root.get("cases"), "corpus.cases")
    corpus_subjects = _mapping(corpus_root.get("subjects"), "corpus.subjects")

    cases = _mapping(root["cases"], "stimulus.cases")
    if set(cases) != EXPECTED_CASE_IDS:
        raise StimulusValidationError("stimulus.cases: I0 v1 case set drifted")
    if set(cases) != set(corpus_cases):
        raise StimulusValidationError("stimulus.cases: does not exactly match bound corpus case set")

    used_operations: set[str] = set()
    for case_id, raw_case in cases.items():
        case = _mapping(raw_case, f"stimulus.cases.{case_id}")
        unknown = set(case) - CASE_KEYS
        missing = {"operation", "subjects"} - set(case)
        if missing or unknown:
            raise StimulusValidationError(
                f"stimulus.cases.{case_id}: missing={sorted(missing)}, extra={sorted(unknown)}"
            )
        operation = _nonempty_string(case["operation"], f"stimulus.cases.{case_id}.operation")
        if operation not in OPERATIONS:
            raise StimulusValidationError(f"stimulus.cases.{case_id}: unregistered operation {operation!r}")
        used_operations.add(operation)

        subjects = case["subjects"]
        if not isinstance(subjects, list) or not subjects:
            raise StimulusValidationError(f"stimulus.cases.{case_id}.subjects: expected non-empty array")
        for subject in subjects:
            if not isinstance(subject, str) or subject not in corpus_subjects:
                raise StimulusValidationError(
                    f"stimulus.cases.{case_id}.subjects: dangling subject {subject!r}"
                )

        for family in ("transport", "authority", "schema", "translation"):
            if family not in case:
                continue
            condition = _mapping(case[family], f"stimulus.cases.{case_id}.{family}")
            extra = set(condition) - CONDITION_KEYS[family]
            if extra:
                raise StimulusValidationError(
                    f"stimulus.cases.{case_id}.{family}: unknown fields {sorted(extra)}"
                )
            if not condition:
                raise StimulusValidationError(
                    f"stimulus.cases.{case_id}.{family}: empty condition object"
                )
            if family == "transport" and "delivery_order" in condition:
                order = condition["delivery_order"]
                if not isinstance(order, list) or not order:
                    raise StimulusValidationError(
                        f"stimulus.cases.{case_id}.transport.delivery_order: expected non-empty array"
                    )
                for ref in order:
                    if not isinstance(ref, str) or ref not in corpus_subjects:
                        raise StimulusValidationError(
                            f"stimulus.cases.{case_id}.transport.delivery_order: dangling subject {ref!r}"
                        )

    nonclaims = root["nonclaims"]
    if not isinstance(nonclaims, list) or not nonclaims:
        raise StimulusValidationError("stimulus.nonclaims: expected non-empty array")
    if len(nonclaims) != len(set(nonclaims)):
        raise StimulusValidationError("stimulus.nonclaims: duplicate entries")
    for index, value in enumerate(nonclaims):
        _nonempty_string(value, f"stimulus.nonclaims[{index}]")

    if used_operations != OPERATIONS:
        missing = sorted(OPERATIONS - used_operations)
        raise StimulusValidationError(f"stimulus: registered operations not exercised: {missing}")

    return {"cases": len(cases), "operations": len(used_operations), "subjects": len(corpus_subjects)}


def validate_bytes(stimulus_raw: bytes, corpus_raw: bytes, *, expected_sha256: str | None = None) -> dict[str, int | str]:
    digest = hashlib.sha256(stimulus_raw).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise StimulusValidationError(
            f"stimulus commitment mismatch: expected {expected_sha256}, got {digest}"
        )
    stimulus = parse_strict_json(stimulus_raw, label="stimulus")
    corpus = parse_strict_json(corpus_raw, label="corpus")
    counts = validate_documents(stimulus, corpus)
    return {
        "validator_profile": STIMULUS_PROFILE,
        "stimulus_id": STIMULUS_ID,
        "stimulus_version": STIMULUS_VERSION,
        "corpus_id": CORPUS_ID,
        "corpus_version": CORPUS_VERSION,
        "cases": counts["cases"],
        "operations": counts["operations"],
        "sha256": digest,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve().parent
    parser.add_argument(
        "stimulus",
        nargs="?",
        type=Path,
        default=here / "myc-int-006e-water-i0-stimulus.v1.json",
    )
    parser.add_argument(
        "--corpus",
        type=Path,
        default=here / "myc-int-006c-water-i0-corpus.v1.json",
    )
    parser.add_argument("--expected-sha256")
    args = parser.parse_args(argv)
    try:
        summary = validate_bytes(
            args.stimulus.read_bytes(),
            args.corpus.read_bytes(),
            expected_sha256=args.expected_sha256,
        )
    except (OSError, StimulusValidationError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
