#!/usr/bin/env python3
"""Independent effective-policy replay and deterministic policy-layer mutation tests.

The finite request evaluator below works over raw JSON and deliberately does not
call oracle.Request, oracle.Atom.matches, oracle.denotation, or oracle policy
classification helpers. This proves sensitivity to selected policy-layer defects;
it is not a proof of completeness for arbitrary authorization languages.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compound_subsumption_counterexamples as oracle  # noqa: E402

EXPECTED_MUTANTS = {
    "deny-set-erased": "policy-status-disagrees-with-independent-replay",
    "deny-overrides-skipped": "policy-status-disagrees-with-independent-replay",
    "allow-overrides-misread": "policy-status-disagrees-with-independent-replay",
    "masked-allow-expansion-accepted": "policy-status-disagrees-with-independent-replay",
    "deny-removal-gate-bypassed": "policy-status-disagrees-with-independent-replay",
    "conflict-rule-flag-forged": "policy-field-disagrees-with-independent-replay",
}


def atom(
    atom_id: str,
    target: list[str],
    purpose: list[str],
    context: list[str],
    max_amount: int = 3,
    extension: str = "none",
) -> dict[str, Any]:
    return {
        "id": atom_id, "target": target, "purpose": purpose,
        "context": context, "max_amount": max_amount, "extension": extension,
    }


def compound(kind: str, *clauses: dict[str, Any]) -> dict[str, Any]:
    return {"kind": kind, "clauses": list(clauses)}


TARGETS = ["alice", "bob"]
PURPOSES = ["business", "refund"]
CONTEXTS = ["trusted", "untrusted"]
AMOUNTS = [0, 1, 2, 3]
UNIVERSE = {
    "targets": TARGETS, "purposes": PURPOSES,
    "contexts": CONTEXTS, "amounts": AMOUNTS,
}
BROAD = compound("any", atom("allow-all", TARGETS, PURPOSES, CONTEXTS))
ALICE = compound("any", atom("allow-alice", ["alice"], PURPOSES, CONTEXTS))
DENY_BOB = compound("all", atom("deny-bob", ["bob"], PURPOSES, CONTEXTS))


def scenario(parent_allow: dict[str, Any], parent_deny: list[dict[str, Any]],
             parent_rule: str, child_allow: dict[str, Any],
             child_deny: list[dict[str, Any]], child_rule: str) -> dict[str, Any]:
    return {
        "schema": oracle.SCENARIO_SCHEMA,
        "mode": "effective-policy",
        "universe": UNIVERSE,
        "parent_policy": {
            "allow": parent_allow, "deny": parent_deny, "conflict_rule": parent_rule,
        },
        "child_policy": {
            "allow": child_allow, "deny": child_deny, "conflict_rule": child_rule,
        },
    }


def fixtures() -> dict[str, dict[str, Any]]:
    return {
        "deny-removal-expands-effective-access": scenario(
            BROAD, [DENY_BOB], "deny-overrides", BROAD, [], "deny-overrides"),
        "allow-expansion-masked-by-deny": scenario(
            ALICE, [], "deny-overrides", BROAD, [DENY_BOB], "deny-overrides"),
        "deny-removal-without-effective-expansion": scenario(
            ALICE, [DENY_BOB], "deny-overrides", ALICE, [], "deny-overrides"),
        "conflict-rule-substitution": scenario(
            BROAD, [DENY_BOB], "deny-overrides", BROAD, [DENY_BOB], "allow-overrides"),
    }


def request_tuples(universe: dict[str, Any]) -> list[tuple[str, str, str, int]]:
    return list(itertools.product(
        universe["targets"], universe["purposes"], universe["contexts"], universe["amounts"]
    ))


def atom_matches_raw(raw_atom: dict[str, Any], request: tuple[str, str, str, int]) -> bool:
    target, purpose, context, amount = request
    extension = raw_atom.get("extension", "none")
    if extension != "none":
        raise ValueError("independent replay refuses unsupported extension: " + str(extension))
    return (
        target in raw_atom["target"]
        and purpose in raw_atom["purpose"]
        and context in raw_atom["context"]
        and amount <= raw_atom["max_amount"]
    )


def compound_denotation_raw(raw_expr: dict[str, Any],
                            requests: list[tuple[str, str, str, int]]) -> set[tuple[str, str, str, int]]:
    clauses = raw_expr["clauses"]
    matches = {
        request: [atom_matches_raw(clause, request) for clause in clauses]
        for request in requests
    }
    if raw_expr["kind"] == "all":
        return {request for request, outcomes in matches.items() if all(outcomes)}
    if raw_expr["kind"] == "any":
        return {request for request, outcomes in matches.items() if any(outcomes)}
    raise ValueError("independent replay refuses unsupported compound kind")


def independent_policy_replay(raw: dict[str, Any]) -> dict[str, Any]:
    requests = request_tuples(raw["universe"])
    parent, child = raw["parent_policy"], raw["child_policy"]
    p_allow = compound_denotation_raw(parent["allow"], requests)
    c_allow = compound_denotation_raw(child["allow"], requests)
    p_deny: set[tuple[str, str, str, int]] = set()
    c_deny: set[tuple[str, str, str, int]] = set()
    for expression in parent["deny"]:
        p_deny.update(compound_denotation_raw(expression, requests))
    for expression in child["deny"]:
        c_deny.update(compound_denotation_raw(expression, requests))

    def effective(allowed: set[tuple[str, str, str, int]],
                  denied: set[tuple[str, str, str, int]], rule: str) -> set[tuple[str, str, str, int]]:
        if rule == "deny-overrides":
            return allowed - denied
        if rule == "allow-overrides":
            return set(allowed)
        raise ValueError("independent replay refuses unknown conflict rule")

    p_effective = effective(p_allow, p_deny, parent["conflict_rule"])
    c_effective = effective(c_allow, c_deny, child["conflict_rule"])
    expansion = c_effective - p_effective
    allow_expansion = c_allow - p_allow
    removed_denies = p_deny - c_deny
    allow_containment = not allow_expansion
    deny_preservation = p_deny <= c_deny
    conflict_preserved = parent["conflict_rule"] == child["conflict_rule"]

    if expansion:
        expected_status = "AUTHORITY_EXPANSION"
    elif not allow_containment or not deny_preservation or not conflict_preserved:
        expected_status = "POLICY_ATTENUATION_VIOLATION"
    else:
        expected_status = "EFFECTIVE_POLICY_CONTAINMENT_PASS"

    return {
        "expected_status": expected_status,
        "effective_denotational_containment": not expansion,
        "allow_denotational_containment": allow_containment,
        "deny_preservation": deny_preservation,
        "conflict_rule_preserved": conflict_preserved,
        "parent_effective_size": len(p_effective),
        "child_effective_size": len(c_effective),
        "parent_deny_size": len(p_deny),
        "child_deny_size": len(c_deny),
        "allow_denotations_equal": p_allow == c_allow,
        "first_expansion_witness": min(expansion) if expansion else None,
        "first_allow_expansion": min(allow_expansion) if allow_expansion else None,
        "first_removed_deny": min(removed_denies) if removed_denies else None,
    }


def audit_policy_result(raw: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any] | None:
    expected = independent_policy_replay(raw)
    if observed.get("status") != expected["expected_status"]:
        return {
            "kind": "policy-status-disagrees-with-independent-replay",
            "expected": expected["expected_status"],
            "observed": observed.get("status"),
        }

    fields = (
        "effective_denotational_containment", "allow_denotational_containment",
        "deny_preservation", "conflict_rule_preserved", "parent_effective_size",
        "child_effective_size", "parent_deny_size", "child_deny_size",
        "allow_denotations_equal",
    )
    for field in fields:
        if observed.get(field) != expected[field]:
            return {
                "kind": "policy-field-disagrees-with-independent-replay",
                "field": field, "expected": expected[field], "observed": observed.get(field),
            }

    witness = observed.get("counterexample", {}).get("request")
    if expected["first_expansion_witness"] is not None:
        tuple_observed = (
            witness.get("target"), witness.get("purpose"),
            witness.get("context"), witness.get("amount"),
        ) if isinstance(witness, dict) else None
        if tuple_observed != expected["first_expansion_witness"]:
            return {
                "kind": "policy-expansion-witness-disagrees-with-independent-replay",
                "expected": expected["first_expansion_witness"],
                "observed": tuple_observed,
            }
    return None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    casebook = fixtures()
    receipt: dict[str, Any] = {
        "schema": "mycelix.effective-policy-mutation-sensitivity-receipt.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "baseline_controls": [],
        "mutations": [],
    }
    try:
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        oracle_path = HERE / "compound_subsumption_counterexamples.py"
        receipt["oracle_sha256"] = hashlib.sha256(oracle_path.read_bytes()).hexdigest()
        receipt["mutation_guard_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()

        baseline_results: dict[str, dict[str, Any]] = {}
        for name, raw in casebook.items():
            observed = oracle.evaluate_scenario(raw)
            mismatch = audit_policy_result(raw, observed)
            require(mismatch is None, name + ": baseline disagrees with independent replay: " + str(mismatch))
            baseline_results[name] = observed
            receipt["baseline_controls"].append({
                "id": name,
                "status": observed["status"],
                "independent_replay": "PASS",
                "finite_universe": len(request_tuples(raw["universe"])),
            })

        def deny_overrides_skipped(policy: Any, universe: Any) -> frozenset[Any]:
            return oracle.denotation(policy.allow, universe)

        def allow_overrides_misread(policy: Any, universe: Any) -> frozenset[Any]:
            allowed = oracle.denotation(policy.allow, universe)
            denied = oracle.deny_denotation(policy, universe)
            return frozenset(allowed - denied)

        def mask_allow_gate(original: Callable[..., dict[str, Any]]) -> Callable[..., dict[str, Any]]:
            def mutant(parent: Any, child: Any, universe: Any) -> dict[str, Any]:
                result = original(parent, child, universe)
                result["status"] = "EFFECTIVE_POLICY_CONTAINMENT_PASS"
                return result
            return mutant

        def mask_deny_gate(original: Callable[..., dict[str, Any]]) -> Callable[..., dict[str, Any]]:
            def mutant(parent: Any, child: Any, universe: Any) -> dict[str, Any]:
                result = original(parent, child, universe)
                result["status"] = "EFFECTIVE_POLICY_CONTAINMENT_PASS"
                return result
            return mutant

        def forge_conflict_flag(original: Callable[..., dict[str, Any]]) -> Callable[..., dict[str, Any]]:
            def mutant(parent: Any, child: Any, universe: Any) -> dict[str, Any]:
                result = original(parent, child, universe)
                result["conflict_rule_preserved"] = True
                return result
            return mutant

        mutations: list[tuple[str, str, str, Callable[..., Any]]] = [
            ("deny-set-erased", "deny-removal-expands-effective-access", "deny_denotation",
             lambda policy, universe: frozenset()),
            ("deny-overrides-skipped", "deny-removal-expands-effective-access", "effective_denotation",
             deny_overrides_skipped),
            ("allow-overrides-misread", "conflict-rule-substitution", "effective_denotation",
             allow_overrides_misread),
            ("masked-allow-expansion-accepted", "allow-expansion-masked-by-deny", "classify_policies",
             mask_allow_gate(oracle.classify_policies)),
            ("deny-removal-gate-bypassed", "deny-removal-without-effective-expansion", "classify_policies",
             mask_deny_gate(oracle.classify_policies)),
            ("conflict-rule-flag-forged", "conflict-rule-substitution", "classify_policies",
             forge_conflict_flag(oracle.classify_policies)),
        ]

        for name, fixture_name, attribute, mutant in mutations:
            raw = casebook[fixture_name]
            original = getattr(oracle, attribute)
            try:
                setattr(oracle, attribute, mutant)
                observed = oracle.evaluate_scenario(raw)
                mismatch = audit_policy_result(raw, observed)
            finally:
                setattr(oracle, attribute, original)

            require(mismatch is not None, name + ": injected policy regression escaped independent replay")
            expected_kind = EXPECTED_MUTANTS[name]
            require(mismatch.get("kind") == expected_kind,
                    name + ": unexpected mismatch kind " + str(mismatch.get("kind")))
            receipt["mutations"].append({
                "id": name, "fixture": fixture_name, "target_function": attribute,
                "mutant_detected": True, "mismatch_kind": mismatch["kind"],
            })

        require(tuple(row["id"] for row in receipt["mutations"]) == tuple(EXPECTED_MUTANTS),
                "observed mutation set differs from the frozen mutation set")
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "independent_baseline_controls": len(receipt["baseline_controls"]),
            "mutants_injected": len(receipt["mutations"]),
            "mutants_detected": sum(bool(row["mutant_detected"]) for row in receipt["mutations"]),
            "request_tuples_per_control": 32,
            "independent_replay": "RAW_JSON_FINITE_DENOTATIONS",
            "coverage": [
                "effective deny semantics",
                "allow containment even when effective access is unchanged",
                "deny preservation even when effective access is unchanged",
                "conflict-rule substitution and its reported component flag",
            ],
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("EFFECTIVE-POLICY MUTATION SENSITIVITY PASS: 6 of 6 injected mutants detected")
        print("INDEPENDENT POLICY REPLAY PASS: 4 baseline scenarios × 32 request tuples")
        print("QUALIFICATION NOT CLAIMED: bounded research/specification evidence only")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("EFFECTIVE-POLICY MUTATION SENSITIVITY FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
