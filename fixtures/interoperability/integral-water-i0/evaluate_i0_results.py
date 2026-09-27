#!/usr/bin/env python3
"""Separate semantic evaluator for MYC-INT-006G."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

EVALUATOR_PROFILE = "myc-int-006g-semantic-evaluator-v1"
RESULT_PROFILE = "oracle-blind-candidate-results-v1"
CORPUS_ID = "myc-int-006c-water-i0"
CORPUS_VERSION = "1.0.0"
FORBIDDEN_CANDIDATE_KEYS = frozenset({
    "expected_disposition", "expected_result", "expected", "oracle",
    "reason_code", "assertion", "assertions", "predicate", "predicates",
})

PREDICATE_FACTS: dict[str, tuple[str, Any]] = {
    "DistinctSemanticSubjects": ("distinct_semantic_subjects", True),
    "NoEffectAuthority": ("effect_authority_granted", False),
    "BindsExactSubject": ("authorization_subject_bound", True),
    "PreservesSourceSchema": ("source_schema_preserved", True),
    "PreservesProvenance": ("provenance_preserved", True),
    "PreservesUnknownState": ("unknown_state_preserved", True),
    "PreservesConflict": ("conflict_preserved", True),
    "NoHistoricalMutation": ("historical_mutation", False),
    "CreatesReviewCandidate": ("review_candidate_created", True),
    "NoLocalAuthority": ("local_authority_granted", False),
    "NoObservationPromotion": ("observation_promotion", False),
    "NoReceiptPromotion": ("receipt_promotion", False),
    "NoOutcomePromotion": ("outcome_promotion", False),
    "DeclaresTranslationLoss": ("translation_loss_declared", True),
    "IdempotentReplay": ("idempotent_replay", True),
    "RejectsExpiredAuthority": ("expired_authority_rejected", True),
    "RejectsStaleSchema": ("stale_schema_rejected", True),
}


class EvaluationError(ValueError):
    pass


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in pairs:
        if key in out:
            raise EvaluationError(f"duplicate JSON object member: {key!r}")
        out[key] = value
    return out


def parse_strict_json(raw: bytes, *, label: str) -> Any:
    try:
        return json.loads(raw.decode("utf-8", errors="strict"), object_pairs_hook=_strict_object)
    except EvaluationError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise EvaluationError(f"{label}: invalid JSON/UTF-8: {exc}") from exc


def _mapping(value: Any, ctx: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EvaluationError(f"{ctx}: expected object")
    return value


def _walk_candidate_forbidden(value: Any, ctx: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_CANDIDATE_KEYS or key.startswith("expected_"):
                raise EvaluationError(f"{ctx}: candidate copied oracle-bearing key {key!r}")
            _walk_candidate_forbidden(child, f"{ctx}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_candidate_forbidden(child, f"{ctx}[{index}]")


def _assertion_check(predicate: str, result: dict[str, Any]) -> tuple[bool, str, Any, Any]:
    facts = _mapping(result.get("facts"), f"candidate.{result.get('case_id')}.facts")
    if predicate == "SingleLogicalEffect":
        observed_fact = facts.get("logical_effect_count")
        observed_top = result.get("effect_count")
        passed = observed_fact == 1 and observed_top == 1
        return passed, "logical_effect_count/effect_count", {"fact": observed_fact, "top": observed_top}, 1

    if predicate not in PREDICATE_FACTS:
        raise EvaluationError(f"unsupported oracle predicate: {predicate!r}")
    fact_key, expected = PREDICATE_FACTS[predicate]
    if fact_key not in facts:
        return False, fact_key, None, expected
    observed = facts[fact_key]
    return observed == expected, fact_key, observed, expected


def evaluate_documents(oracle: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    if oracle.get("corpus_id") != CORPUS_ID or oracle.get("corpus_version") != CORPUS_VERSION:
        raise EvaluationError("unexpected oracle corpus identity/version")

    _walk_candidate_forbidden(candidate)
    if candidate.get("result_profile") != RESULT_PROFILE:
        raise EvaluationError("unexpected candidate result profile")
    adapter_profile = candidate.get("adapter_profile")
    if not isinstance(adapter_profile, str) or not adapter_profile:
        raise EvaluationError("candidate adapter profile missing")
    if not isinstance(candidate.get("candidate_input_id"), str) or not candidate["candidate_input_id"]:
        raise EvaluationError("candidate input identity missing")
    if not isinstance(candidate.get("candidate_input_version"), str) or not candidate["candidate_input_version"]:
        raise EvaluationError("candidate input version missing")

    oracle_cases = _mapping(oracle.get("cases"), "oracle.cases")
    raw_results = candidate.get("results")
    if not isinstance(raw_results, list):
        raise EvaluationError("candidate.results: expected array")

    candidate_by_case: dict[str, dict[str, Any]] = {}
    for index, raw in enumerate(raw_results):
        result = _mapping(raw, f"candidate.results[{index}]")
        case_id = result.get("case_id")
        if not isinstance(case_id, str) or not case_id:
            raise EvaluationError(f"candidate.results[{index}]: missing case_id")
        if case_id in candidate_by_case:
            raise EvaluationError(f"candidate.results: duplicate case result {case_id}")
        candidate_by_case[case_id] = result

    if set(candidate_by_case) != set(oracle_cases):
        missing = sorted(set(oracle_cases) - set(candidate_by_case))
        extra = sorted(set(candidate_by_case) - set(oracle_cases))
        raise EvaluationError(f"candidate case set mismatch; missing={missing}, extra={extra}")

    report_cases: list[dict[str, Any]] = []
    assertion_total = 0
    assertion_passes = 0
    disposition_matches = 0

    for case_id in sorted(oracle_cases):
        oracle_case = _mapping(oracle_cases[case_id], f"oracle.cases.{case_id}")
        result = candidate_by_case[case_id]
        expected_disposition = _mapping(
            oracle_case.get("expected_disposition"), f"oracle.cases.{case_id}.expected_disposition"
        ).get("class")
        observed_disposition = result.get("disposition")
        disposition_match = observed_disposition == expected_disposition
        disposition_matches += int(disposition_match)

        assertions = oracle_case.get("assertions")
        if not isinstance(assertions, list) or not assertions:
            raise EvaluationError(f"oracle.cases.{case_id}.assertions: expected non-empty array")

        checks: list[dict[str, Any]] = []
        for assertion in assertions:
            assertion_obj = _mapping(assertion, f"oracle.cases.{case_id}.assertion")
            predicate = assertion_obj.get("predicate")
            if not isinstance(predicate, str):
                raise EvaluationError(f"oracle.cases.{case_id}: assertion predicate missing")
            passed, fact_key, observed, expected = _assertion_check(predicate, result)
            assertion_total += 1
            assertion_passes += int(passed)
            checks.append({
                "predicate": predicate,
                "fact": fact_key,
                "observed": observed,
                "expected": expected,
                "passed": passed,
            })

        case_pass = disposition_match and all(check["passed"] for check in checks)
        report_cases.append({
            "case_id": case_id,
            "observed_disposition": observed_disposition,
            "expected_disposition": expected_disposition,
            "disposition_match": disposition_match,
            "assertions": checks,
            "pass": case_pass,
        })

    overall = disposition_matches == len(oracle_cases) and assertion_passes == assertion_total and all(row["pass"] for row in report_cases)
    return {
        "evaluator_profile": EVALUATOR_PROFILE,
        "corpus_id": CORPUS_ID,
        "corpus_version": CORPUS_VERSION,
        "candidate_result_profile": candidate["result_profile"],
        "candidate_adapter_profile": adapter_profile,
        "candidate_input_id": candidate["candidate_input_id"],
        "candidate_input_version": candidate["candidate_input_version"],
        "case_count": len(oracle_cases),
        "disposition_matches": disposition_matches,
        "assertion_checks": assertion_total,
        "assertion_passes": assertion_passes,
        "status": "PASS" if overall else "FAIL",
        "cases": report_cases,
    }


def evaluate_bytes(oracle_raw: bytes, candidate_raw: bytes, *, schema_raw: bytes | None = None) -> dict[str, Any]:
    if schema_raw is not None:
        import validate_i0
        validate_i0.validate_bytes(oracle_raw, schema_raw=schema_raw)
    oracle = parse_strict_json(oracle_raw, label="oracle")
    candidate = parse_strict_json(candidate_raw, label="candidate")
    report = evaluate_documents(oracle, candidate)
    report["oracle_sha256"] = hashlib.sha256(oracle_raw).hexdigest()
    report["candidate_sha256"] = hashlib.sha256(candidate_raw).hexdigest()
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve().parent
    parser.add_argument("--oracle", type=Path, default=here / "myc-int-006c-water-i0-corpus.v1.json")
    parser.add_argument("--schema", type=Path, default=here / "myc-int-006c-water-i0-corpus.schema.json")
    parser.add_argument("candidate_results", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        report = evaluate_bytes(args.oracle.read_bytes(), args.candidate_results.read_bytes(), schema_raw=args.schema.read_bytes())
    except (OSError, EvaluationError, ValueError) as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        return 2
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
