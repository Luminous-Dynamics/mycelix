#!/usr/bin/env python3
"""Prove selected oracle regressions are observable to the independent checker.

This is a deterministic mutation-sensitivity check, not a claim that a finite
mutant set proves the checker free of defects. Mutations are applied only to
in-memory module functions and are always restored before exit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compound_subsumption_counterexamples as oracle  # noqa: E402
import differential_compound_subsumption as differential  # noqa: E402

UNIVERSE = {
    "targets": ["alice", "bob"],
    "purposes": ["business", "refund"],
    "contexts": ["trusted", "untrusted"],
    "amounts": [0, 1, 2, 3],
}
EXPECTED_MUTANTS = {
    "atom-subsumption-opened": "structural-matcher-disagrees-with-bruteforce",
    "denotation-forced-empty": "denotational-containment-flag-disagrees-with-independent-evaluator",
    "injective-matcher-reuses-child": "structural-witness-map-invalid",
    "witness-core-keeps-redundant-clause": "expansion-diagnostic-core-incorrect",
}


def atom(
    atom_id: str,
    target: list[str],
    purpose: list[str],
    context: list[str],
    max_amount: int = 3,
) -> dict[str, Any]:
    return {
        "id": atom_id,
        "target": target,
        "purpose": purpose,
        "context": context,
        "max_amount": max_amount,
        "extension": "none",
    }


def compound(kind: str, *clauses: dict[str, Any]) -> dict[str, Any]:
    return {"kind": kind, "clauses": list(clauses)}


def scenario(parent: dict[str, Any], child: dict[str, Any]) -> dict[str, Any]:
    return {"schema": oracle.SCENARIO_SCHEMA, "universe": UNIVERSE,
            "mode": "compound", "parent": parent, "child": child}


def cases() -> dict[str, dict[str, Any]]:
    parent_alice = atom("parent-alice", ["alice"], UNIVERSE["purposes"], UNIVERSE["contexts"])
    parent_business = atom("parent-business", UNIVERSE["targets"], ["business"], UNIVERSE["contexts"])
    child_bob = atom("child-bob", ["bob"], UNIVERSE["purposes"], UNIVERSE["contexts"])
    child_refund = atom("child-refund", UNIVERSE["targets"], ["refund"], UNIVERSE["contexts"])
    repeated_parent = compound(
        "all",
        atom("parent-narrow-1", ["alice"], ["business"], ["trusted"], 1),
        atom("parent-narrow-2", ["alice"], ["business"], ["trusted"], 1),
    )
    repeated_child = compound(
        "all",
        atom("child-narrow-1", ["alice"], ["business"], ["trusted"], 1),
        atom("child-narrow-2", ["alice"], ["business"], ["trusted"], 1),
    )
    return {
        "expansion": scenario(
            compound("all", parent_alice, parent_business),
            compound("all", child_bob, child_refund),
        ),
        "injective": scenario(repeated_parent, repeated_child),
    }


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    fixtures = cases()
    mutations: list[tuple[str, dict[str, Any], str, Callable[..., Any]]] = [
        (
            "atom-subsumption-opened",
            fixtures["expansion"],
            "atom_subsumes",
            lambda child, parent: True,
        ),
        (
            "denotation-forced-empty",
            fixtures["expansion"],
            "denotation",
            lambda expression, universe: frozenset(),
        ),
        (
            "injective-matcher-reuses-child",
            fixtures["injective"],
            "maximum_injective_matching",
            lambda parent, child: {
                parent.clauses[0].id: child.clauses[0].id,
                parent.clauses[1].id: child.clauses[0].id,
            },
        ),
        (
            "witness-core-keeps-redundant-clause",
            fixtures["expansion"],
            "minimize_core",
            lambda expression, request, must_admit: sorted(atom.id for atom in expression.clauses),
        ),
    ]
    receipt: dict[str, Any] = {
        "schema": "mycelix.oracle-mutation-sensitivity-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "mutations": [],
    }

    try:
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        oracle_path = HERE / "compound_subsumption_counterexamples.py"
        checker_path = HERE / "differential_compound_subsumption.py"
        receipt["oracle_sha256"] = hashlib.sha256(oracle_path.read_bytes()).hexdigest()
        receipt["differential_checker_sha256"] = hashlib.sha256(checker_path.read_bytes()).hexdigest()
        receipt["mutation_guard_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()

        for name, raw, attribute, mutant in mutations:
            baseline = oracle.evaluate_scenario(raw)
            baseline_mismatch = differential.mismatch_for(raw, baseline)
            require(baseline_mismatch is None,
                    name + ": unmutated baseline is not independently consistent: " + str(baseline_mismatch))

            original = getattr(oracle, attribute)
            try:
                setattr(oracle, attribute, mutant)
                observed = oracle.evaluate_scenario(raw)
                mismatch = differential.mismatch_for(raw, observed)
            finally:
                setattr(oracle, attribute, original)

            require(mismatch is not None, name + ": injected regression escaped independent detection")
            expected_kind = EXPECTED_MUTANTS[name]
            require(mismatch.get("kind") == expected_kind,
                    name + ": unexpected mismatch category " + str(mismatch.get("kind")))
            receipt["mutations"].append({
                "id": name,
                "target_function": attribute,
                "baseline": "INDEPENDENTLY_CONSISTENT",
                "mutant_detected": True,
                "mismatch_kind": mismatch["kind"],
            })

        observed_names = tuple(row["id"] for row in receipt["mutations"])
        require(observed_names == tuple(EXPECTED_MUTANTS),
                "executed mutation set differs from the frozen expected mutation set")
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "mutants_injected": len(receipt["mutations"]),
            "mutants_detected": sum(row["mutant_detected"] for row in receipt["mutations"]),
            "independent_checker": "differential_compound_subsumption.mismatch_for",
            "scope": "four deterministic in-memory mutants covering subsumption, denotation, injectivity, and diagnostic minimization",
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("ORACLE MUTATION SENSITIVITY PASS: 4 of 4 injected mutants detected")
        print("QUALIFICATION NOT CLAIMED: finite mutation set; not a proof of checker completeness")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("ORACLE MUTATION SENSITIVITY FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
