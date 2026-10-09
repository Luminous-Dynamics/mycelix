#!/usr/bin/env python3
"""Mutation guard proving the differential corpus manifest cannot be weakened silently."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any

from differential_compound_subsumption import validate_matrix


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
    args = parser.parse_args()
    baseline = json.loads(args.matrix.read_text(encoding="utf-8"))
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

    print(f"DIFFERENTIAL MATRIX MUTATION GUARD PASS: {len(rejected_names)} weakening mutations rejected")
    for name in rejected_names:
        print(f"MATRIX MUTATION REJECTED: {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
