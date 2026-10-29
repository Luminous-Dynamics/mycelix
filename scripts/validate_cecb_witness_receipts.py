#!/usr/bin/env python3
"""Validate CECB witness receipt schemas, fixtures, and cross-field invariants.

This is CI/research tooling, not a production benchmark runner. Example fixture
hashes are syntactically valid placeholders and do not constitute run evidence.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    ROOT
    / "docs/research/schemas/CECB_VERIFIED_WITNESS_RECEIPT_V0_1.schema.json"
)
FIXTURE_DIR = ROOT / "docs/research/fixtures"
EXPECTED = {
    "CECB_WITNESS_RECEIPT_VALID_COUNTEREXAMPLE_V0_1.json":
        "valid_counterexample",
    "CECB_WITNESS_RECEIPT_NO_COUNTEREXAMPLE_FOUND_V0_1.json":
        "no_counterexample_found_within_budget",
    "CECB_WITNESS_RECEIPT_BOUNDED_DOMAIN_EXHAUSTIVE_V0_1.json":
        "verified_no_counterexample_in_bounded_domain",
}


class ReceiptValidationError(ValueError):
    """An invalid receipt or a contradiction between receipt fields."""


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ReceiptValidationError(f"{path.name}: top-level JSON must be an object")
    return value


def validate_receipt(
    receipt: dict[str, Any], validator: Draft202012Validator, label: str
) -> None:
    errors = sorted(validator.iter_errors(receipt), key=lambda error: list(error.path))
    if errors:
        first = errors[0]
        location = "/".join(str(part) for part in first.path) or "<root>"
        raise ReceiptValidationError(
            f"{label}: schema violation at {location}: {first.message}"
        )

    budgets = receipt["budgets"]
    trace = receipt["query_trace"]
    if budgets["label_queries_used"] != len(trace):
        raise ReceiptValidationError(
            f"{label}: query count disagrees with trusted trace length"
        )
    if budgets["label_queries_used"] > budgets["max_label_queries"]:
        raise ReceiptValidationError(f"{label}: query budget exceeded")
    if [item["query_index"] for item in trace] != list(range(len(trace))):
        raise ReceiptValidationError(
            f"{label}: query indices must be contiguous and zero-based"
        )

    outcome = receipt["outcome"]
    verification = receipt["verification"]
    witness = receipt.get("witness")
    if outcome == "valid_counterexample":
        if witness is None:
            raise ReceiptValidationError(f"{label}: accepted counterexample lacks witness")
        if not witness["feasible"]:
            raise ReceiptValidationError(f"{label}: witness violates feasibility")
        if witness["candidate_prediction"] == witness["reference_prediction"]:
            raise ReceiptValidationError(f"{label}: witness does not reproduce a mismatch")
        if not verification["witness_feasible"] or not verification["mismatch_reproduced"]:
            raise ReceiptValidationError(f"{label}: verifier did not confirm the witness")
    elif outcome == "no_counterexample_found_within_budget":
        if witness is not None:
            raise ReceiptValidationError(f"{label}: search-failure outcome must not carry witness")
        if verification["complete_domain_checked"]:
            raise ReceiptValidationError(
                f"{label}: budget-limited search cannot claim a complete domain check"
            )
        if verification["mode"] != "budgeted_search":
            raise ReceiptValidationError(
                f"{label}: no-witness search must use budgeted_search verification mode"
            )
    elif outcome == "verified_no_counterexample_in_bounded_domain":
        if witness is not None:
            raise ReceiptValidationError(
                f"{label}: no-counterexample proof cannot include a counterexample witness"
            )
        if not verification["complete_domain_checked"]:
            raise ReceiptValidationError(f"{label}: complete-domain claim lacks completion")
        if verification["mode"] == "bounded_domain_exhaustive":
            domain = receipt["domain"]
            if domain["kind"] != "finite_discrete":
                raise ReceiptValidationError(
                    f"{label}: exhaustive enumeration requires a finite discrete domain"
                )
            if verification["checked_domain_size"] != domain["size"]:
                raise ReceiptValidationError(
                    f"{label}: checked domain size differs from declared domain size"
                )
        elif verification["mode"] == "independent_proof_artifact":
            if not verification["proof_artifact_sha256"]:
                raise ReceiptValidationError(
                    f"{label}: proof outcome lacks a digest-bound proof artifact"
                )
        else:
            raise ReceiptValidationError(
                f"{label}: verified no-witness outcome requires exhaustive checking or a proof"
            )
    else:
        raise ReceiptValidationError(f"{label}: unknown outcome {outcome!r}")


def expect_rejected(
    receipt: dict[str, Any],
    validator: Draft202012Validator,
    label: str,
) -> None:
    try:
        validate_receipt(receipt, validator, label)
    except ReceiptValidationError:
        return
    raise ReceiptValidationError(f"{label}: invalid mutation was unexpectedly accepted")


def main() -> int:
    schema = load_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema)

    loaded: dict[str, dict[str, Any]] = {}
    for filename, expected_outcome in EXPECTED.items():
        receipt = load_json(FIXTURE_DIR / filename)
        if receipt["outcome"] != expected_outcome:
            raise ReceiptValidationError(
                f"{filename}: expected outcome {expected_outcome!r}"
            )
        validate_receipt(receipt, validator, filename)
        loaded[filename] = receipt

    # Negative controls: self-asserted mismatches, contradictory outcomes, and
    # false exhaustive claims must be rejected, even if the JSON parses.
    valid = copy.deepcopy(
        loaded["CECB_WITNESS_RECEIPT_VALID_COUNTEREXAMPLE_V0_1.json"]
    )
    valid["witness"]["reference_prediction"] = valid["witness"]["candidate_prediction"]
    expect_rejected(valid, validator, "equal-prediction witness mutation")

    false_budget_claim = copy.deepcopy(
        loaded["CECB_WITNESS_RECEIPT_VALID_COUNTEREXAMPLE_V0_1.json"]
    )
    false_budget_claim["outcome"] = "no_counterexample_found_within_budget"
    expect_rejected(false_budget_claim, validator, "witness attached to no-witness outcome")

    false_domain_size = copy.deepcopy(
        loaded["CECB_WITNESS_RECEIPT_BOUNDED_DOMAIN_EXHAUSTIVE_V0_1.json"]
    )
    false_domain_size["verification"]["checked_domain_size"] -= 1
    expect_rejected(false_domain_size, validator, "incomplete exhaustive-domain receipt")

    over_budget = copy.deepcopy(
        loaded["CECB_WITNESS_RECEIPT_NO_COUNTEREXAMPLE_FOUND_V0_1.json"]
    )
    over_budget["budgets"]["max_label_queries"] -= 1
    expect_rejected(over_budget, validator, "over-budget receipt")

    print(
        "CECB witness receipt validation passed: "
        f"{len(EXPECTED)} valid fixtures and 4 rejected negative controls."
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, json.JSONDecodeError, ReceiptValidationError) as error:
        print(f"CECB witness receipt validation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
