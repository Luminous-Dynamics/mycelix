#!/usr/bin/env python3
"""Mutation guard proving the differential corpus manifest cannot be weakened silently."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

from differential_compound_subsumption import validate_matrix


EXPECTED_MUTATIONS = (
    "remove-request-value",
    "reorder-request-values",
    "widen-amount-domain",
    "weaken-atom-target",
    "widen-atom-numeric-bound",
    "remove-atom",
    "change-generation-rule",
    "lower-expression-count",
    "lower-pair-count",
    "lower-pair-request-count",
    "remove-compound-operator",
    "remove-required-invariant",
    "enable-randomized-generation",
    "duplicate-atom-identifier",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def rejected(name: str, candidate: dict[str, Any]) -> None:
    try:
        validate_matrix(candidate)
    except (AssertionError, KeyError, TypeError, ValueError):
        return
    raise AssertionError(f"matrix mutation unexpectedly accepted: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    matrix_bytes = args.matrix.read_bytes()
    baseline = json.loads(matrix_bytes.decode("utf-8"))
    validate_matrix(baseline)

    mutations: list[tuple[str, Any]] = [
        ("remove-request-value", lambda m: m["finite_universe"]["targets"].pop()),
        ("reorder-request-values", lambda m: m["finite_universe"]["targets"].reverse()),
        ("widen-amount-domain", lambda m: m["finite_universe"]["amounts"].append(4)),
        ("weaken-atom-target", lambda m: m["atoms"][6]["target"].append("bob")),
        ("widen-atom-numeric-bound", lambda m: m["atoms"][6].update({"max_amount": 3})),
        ("remove-atom", lambda m: m["atoms"].pop()),
        ("change-generation-rule", lambda m: m["corpus"].update({"generation": "unordered combinations"})),
        ("lower-expression-count", lambda m: m["corpus"].update({"expression_count": 127})),
        ("lower-pair-count", lambda m: m["corpus"].update({"ordered_parent_child_pairs": 16383})),
        ("lower-pair-request-count", lambda m: m["corpus"].update({"ordered_pair_request_combinations": 524287})),
        ("remove-compound-operator", lambda m: m["corpus"].update({"compound_kinds": ["all"]})),
        ("remove-required-invariant", lambda m: m["required_invariants"].pop()),
        ("enable-randomized-generation", lambda m: m["corpus"].update({"randomness": "seeded"})),
        ("duplicate-atom-identifier", lambda m: m["atoms"][1].update({"id": m["atoms"][0]["id"]})),
    ]
    rejected_names = []
    for name, mutate in mutations:
        candidate = copy.deepcopy(baseline)
        mutate(candidate)
        rejected(name, candidate)
        rejected_names.append(name)

    require(tuple(rejected_names) == EXPECTED_MUTATIONS,
            "mutation guard coverage differs from the frozen set of 14 mutations")

    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
        check=True, timeout=15,
    ).stdout.strip()
    receipt = {
        "schema": "mycelix.differential-matrix-mutation-guard-receipt.v1",
        "status": "PASS",
        "source_head": head,
        "matrix_sha256": hashlib.sha256(matrix_bytes).hexdigest(),
        "checker_sha256": hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest(),
        "mutation_count": len(rejected_names),
        "mutations_rejected": rejected_names,
        "baseline_manifest_validation": "PASS",
        "qualification": "NOT_CLAIMED",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(f"DIFFERENTIAL MATRIX MUTATION GUARD PASS: {len(rejected_names)} weakening mutations rejected")
    for name in rejected_names:
        print(f"MATRIX MUTATION REJECTED: {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
