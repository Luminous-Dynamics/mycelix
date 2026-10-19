#!/usr/bin/env python3
"""Independent structural/consistency verifier for the COS reference report."""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
REPORT_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("cos-reference-conformance-report.json")
if not REPORT_PATH.is_absolute():
    REPORT_PATH = ROOT / REPORT_PATH

EXPECTED_IDS = [f"COS-N-{number:03d}" for number in range(1, 17)]
EXPECTED_OBLIGATIONS = {f"COS-FV-{number:03d}" for number in range(1, 11)}
EXPECTED_CLAIM_CEILINGS = {
    "COS-N-001": "Plan/execution semantic conformance only.",
    "COS-N-002": "Planned-vs-consumed evidence separation only.",
    "COS-N-003": "Temporal availability binding only.",
    "COS-N-004": "Machine capability/availability separation only.",
    "COS-N-005": "Freshness semantics only.",
    "COS-N-006": "Output qualification boundary only.",
    "COS-N-007": "Single-run/general-capability separation only.",
    "COS-N-008": "Historical failure immutability only.",
    "COS-N-009": "External dependency provenance only.",
    "COS-N-010": "FRS projection/source-observation separation only.",
    "COS-N-011": "Production/ITC projection separation only.",
    "COS-N-012": "ITC/physical-work separation only.",
    "COS-N-013": "Foreign/local provenance separation only.",
    "COS-N-014": "Authorization/effect separation only.",
    "COS-N-015": "Distribution/quality separation only.",
    "COS-N-016": "Explicit-denominator arithmetic claim only.",
}
ALLOWED_DECISIONS = {
    "Accepted", "Rejected", "Unknown", "Conflicting",
    "Stale", "Superseded", "Unauthorized", "Unbound",
}
REQUIRED_RECORD_FIELDS = {
    "test_id", "formal_obligations", "expected_negative", "actual_negative",
    "expected_positive", "actual_positive", "input_evidence_classes",
    "provenance_origin", "temporal_validity", "refinement_status",
    "refinement_subject", "claim_ceiling",
}


def unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def reject_constant(token: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {token}")


def load_json(path: Path) -> Any:
    return json.loads(
        path.read_text(encoding="utf-8"),
        object_pairs_hook=unique_pairs,
        parse_constant=reject_constant,
    )


def require_nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"REQUIRED_STRING_MISSING {label}")
    return value


def main() -> int:
    report = load_json(REPORT_PATH)
    if not isinstance(report, dict):
        raise ValueError("REPORT_ROOT_NOT_OBJECT")
    if report.get("corpus_id") != "COS-CONF-001":
        raise ValueError("REPORT_CORPUS_ID_MISMATCH")
    if report.get("case_count") != 32:
        raise ValueError("REPORT_CASE_COUNT_MISMATCH")
    if not isinstance(report.get("records"), list) or len(report["records"]) != 16:
        raise ValueError("REPORT_RECORD_COUNT_MISMATCH")
    if report.get("untested_formal_obligations") != []:
        raise ValueError("REPORT_UNTESTED_FORMAL_OBLIGATIONS_NOT_EMPTY")
    require_nonempty_string(report.get("claim_ceiling"), "root.claim_ceiling")
    if "no physical" not in report["claim_ceiling"].lower() or "integral-validation" not in report["claim_ceiling"].lower():
        raise ValueError("REPORT_ROOT_CLAIM_CEILING_WEAKENED")

    seen_ids: list[str] = []
    covered: set[str] = set()
    for index, record in enumerate(report["records"]):
        label = f"records[{index}]"
        if not isinstance(record, dict):
            raise ValueError(f"REPORT_RECORD_NOT_OBJECT {label}")
        missing = REQUIRED_RECORD_FIELDS - set(record)
        if missing:
            raise ValueError(f"REPORT_FIELDS_MISSING {label}: {sorted(missing)}")
        test_id = require_nonempty_string(record["test_id"], f"{label}.test_id")
        seen_ids.append(test_id)
        if record["expected_negative"] not in ALLOWED_DECISIONS or record["expected_positive"] not in ALLOWED_DECISIONS:
            raise ValueError(f"REPORT_EXPECTED_DECISION_INVALID {test_id}")
        if record["actual_negative"] not in ALLOWED_DECISIONS or record["actual_positive"] not in ALLOWED_DECISIONS:
            raise ValueError(f"REPORT_ACTUAL_DECISION_INVALID {test_id}")
        if record["actual_negative"] != record["expected_negative"]:
            raise ValueError(f"REPORT_NEGATIVE_DECISION_DRIFT {test_id}")
        if record["actual_positive"] != record["expected_positive"]:
            raise ValueError(f"REPORT_POSITIVE_DECISION_DRIFT {test_id}")
        if record["expected_positive"] != "Accepted":
            raise ValueError(f"REPORT_POSITIVE_EXPECTATION_WEAKENED {test_id}")
        obligations = record["formal_obligations"]
        if not isinstance(obligations, list) or not obligations or any(not isinstance(x, str) for x in obligations):
            raise ValueError(f"REPORT_FORMAL_MAPPING_INVALID {test_id}")
        if len(obligations) != len(set(obligations)) or not set(obligations) <= EXPECTED_OBLIGATIONS:
            raise ValueError(f"REPORT_FORMAL_MAPPING_UNKNOWN_OR_DUPLICATE {test_id}")
        covered.update(obligations)
        for field in ("input_evidence_classes", "provenance_origin", "temporal_validity", "refinement_subject", "claim_ceiling"):
            require_nonempty_string(record[field], f"{label}.{field}")
        if record["refinement_status"] != "ReferenceModelOnly":
            raise ValueError(f"REPORT_REFINEMENT_CLAIM_EXCEEDS_REFERENCE_MODEL {test_id}")
        if record["claim_ceiling"] != EXPECTED_CLAIM_CEILINGS.get(test_id):
            raise ValueError(f"REPORT_ROW_CLAIM_CEILING_MISMATCH {test_id}")
        if test_id in {"COS-N-009", "COS-N-013"}:
            if "foreign" not in record["provenance_origin"].lower():
                raise ValueError(f"REPORT_FOREIGN_ORIGIN_BOUNDARY_MISSING {test_id}")
        if test_id == "COS-N-005" and not all(token in record["temporal_validity"].lower() for token in ("stale", "current")):
            raise ValueError("REPORT_STALE_EVIDENCE_BOUNDARY_MISSING")

    if seen_ids != EXPECTED_IDS:
        raise ValueError("REPORT_CASE_ID_CENSUS_OR_ORDER_MISMATCH")
    if covered != EXPECTED_OBLIGATIONS:
        raise ValueError(f"REPORT_FORMAL_OBLIGATION_COVERAGE_MISMATCH missing={sorted(EXPECTED_OBLIGATIONS - covered)}")

    print("PASS: COS-CONF-001 report has 16 ordered cases / 32 expected outcomes, complete COS-FV coverage, matching actual decisions, and required provenance/refinement metadata.")
    print("Claim ceiling: report structure and reference-model self-consistency only; independent executable test suites remain required.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
