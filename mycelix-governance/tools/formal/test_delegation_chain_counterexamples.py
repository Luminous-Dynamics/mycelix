#!/usr/bin/env python3
"""Independent finite-universe controls for multi-hop authorization attenuation."""
from __future__ import annotations

import argparse
import copy
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
import delegation_chain_counterexamples as chain_checker  # noqa: E402

CHAIN_SCHEMA = chain_checker.CHAIN_SCHEMA
FROZEN_MAX_DELEGATION_DEPTH = 8
FROZEN_MAX_HOPS = FROZEN_MAX_DELEGATION_DEPTH + 1
INDEPENDENT_FAILURE_STATUSES = {
    "AUTHORITY_EXPANSION", "POLICY_ATTENUATION_VIOLATION", "UNSUPPORTED_OR_UNDECIDABLE",
}
TARGETS = ["alice", "bob"]
PURPOSES = ["business", "refund"]
CONTEXTS = ["trusted", "untrusted"]
AMOUNTS = [0, 1, 2, 3]
UNIVERSE = {
    "targets": TARGETS, "purposes": PURPOSES,
    "contexts": CONTEXTS, "amounts": AMOUNTS,
}
EXPECTED_MUTANTS = {
    "adjacent-edge-validation-skipped": "chain-relation-set-disagrees-with-independent-replay",
    "root-anchor-validation-skipped": "chain-relation-set-disagrees-with-independent-replay",
    "masked-attenuation-violation-accepted": "policy-pair-status-disagrees-with-independent-replay",
    "chain-expansion-status-downgraded": "chain-status-disagrees-with-independent-replay",
}


def atom(atom_id: str, target: list[str], purpose: list[str], context: list[str],
         max_amount: int = 3, extension: str = "none") -> dict[str, Any]:
    return {"id": atom_id, "target": target, "purpose": purpose, "context": context,
            "max_amount": max_amount, "extension": extension}


def compound(kind: str, *clauses: dict[str, Any]) -> dict[str, Any]:
    return {"kind": kind, "clauses": list(clauses)}


BROAD = compound("any", atom("allow-all", TARGETS, PURPOSES, CONTEXTS))
ALICE = compound("any", atom("allow-alice", ["alice"], PURPOSES, CONTEXTS))
ALICE_BUSINESS = compound(
    "any", atom("allow-alice-business-trusted", ["alice"], ["business"], ["trusted"], 2)
)
ALICE_BUSINESS_TIGHT = compound(
    "any", atom("allow-alice-business-trusted-tight", ["alice"], ["business"], ["trusted"], 1)
)
DENY_BOB = compound("all", atom("deny-bob", ["bob"], PURPOSES, CONTEXTS))


def policy(allow: dict[str, Any], deny: list[dict[str, Any]] | None = None,
           conflict_rule: str = "deny-overrides") -> dict[str, Any]:
    return {"allow": copy.deepcopy(allow), "deny": copy.deepcopy(deny or []),
            "conflict_rule": conflict_rule}


def hop(hop_id: str, policy_value: dict[str, Any]) -> dict[str, Any]:
    return {"id": hop_id, "policy": policy_value}


def fixturebook() -> dict[str, dict[str, Any]]:
    return {
        "monotonic-four-edge-chain": {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [
                hop("root", policy(BROAD)),
                hop("agent-a", policy(ALICE)),
                hop("agent-b", policy(ALICE, [DENY_BOB])),
                hop("agent-c", policy(ALICE_BUSINESS, [DENY_BOB])),
                hop("tool-leaf", policy(ALICE_BUSINESS_TIGHT, [DENY_BOB])),
            ],
        },
        "allow-expansion-masked-at-hop-two": {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [
                hop("root", policy(BROAD)),
                hop("agent-a", policy(ALICE)),
                hop("agent-b", policy(BROAD, [DENY_BOB])),
            ],
        },
        "authority-reintroduced-at-hop-two": {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [
                hop("root", policy(BROAD)),
                hop("agent-a", policy(ALICE)),
                hop("agent-b", policy(BROAD)),
            ],
        },
        "deny-preservation-lost-without-effective-expansion": {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [
                hop("root", policy(ALICE, [DENY_BOB])),
                hop("agent-a", policy(ALICE)),
                hop("tool-leaf", policy(ALICE)),
            ],
        },
        "unsupported-extension-at-leaf": {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [
                hop("root", policy(BROAD)),
                hop("agent-a", policy(ALICE)),
                hop("tool-leaf", policy(ALICE, [
                    compound("all", atom("future-deny", ["bob"], PURPOSES, CONTEXTS,
                                         extension="future-deny-v1"))
                ])),
            ],
        },
    }


def request_tuples(universe: dict[str, Any]) -> list[tuple[str, str, str, int]]:
    return list(itertools.product(universe["targets"], universe["purposes"],
                                  universe["contexts"], universe["amounts"]))


def atom_matches_raw(raw_atom: dict[str, Any], request: tuple[str, str, str, int]) -> bool:
    target, purpose, context, amount = request
    if raw_atom.get("extension", "none") != "none":
        raise ValueError("unsupported extension")
    return (
        target in raw_atom["target"] and purpose in raw_atom["purpose"]
        and context in raw_atom["context"] and amount <= raw_atom["max_amount"]
    )


def compound_denotation_raw(
    raw_expr: dict[str, Any], requests: list[tuple[str, str, str, int]]
) -> set[tuple[str, str, str, int]]:
    results: dict[tuple[str, str, str, int], list[bool]] = {}
    for request in requests:
        results[request] = [atom_matches_raw(clause, request) for clause in raw_expr["clauses"]]
    if raw_expr["kind"] == "all":
        return {request for request, matches in results.items() if all(matches)}
    if raw_expr["kind"] == "any":
        return {request for request, matches in results.items() if any(matches)}
    raise ValueError("unsupported compound kind")


def independent_policy_replay(parent: dict[str, Any], child: dict[str, Any],
                              universe: dict[str, Any]) -> dict[str, Any]:
    if parent["conflict_rule"] not in {"deny-overrides", "allow-overrides"} or \
            child["conflict_rule"] not in {"deny-overrides", "allow-overrides"}:
        return {"status": "UNSUPPORTED_OR_UNDECIDABLE"}
    requests = request_tuples(universe)
    try:
        parent_allow = compound_denotation_raw(parent["allow"], requests)
        child_allow = compound_denotation_raw(child["allow"], requests)
        parent_deny: set[tuple[str, str, str, int]] = set()
        child_deny: set[tuple[str, str, str, int]] = set()
        for expr in parent["deny"]:
            parent_deny.update(compound_denotation_raw(expr, requests))
        for expr in child["deny"]:
            child_deny.update(compound_denotation_raw(expr, requests))
    except (ValueError, KeyError, TypeError):
        return {"status": "UNSUPPORTED_OR_UNDECIDABLE"}

    def effective(allow_set: set[tuple[str, str, str, int]],
                  deny_set: set[tuple[str, str, str, int]], rule: str) -> set[tuple[str, str, str, int]]:
        return allow_set - deny_set if rule == "deny-overrides" else set(allow_set)

    p_effective = effective(parent_allow, parent_deny, parent["conflict_rule"])
    c_effective = effective(child_allow, child_deny, child["conflict_rule"])
    expansion = c_effective - p_effective
    allow_expansion = child_allow - parent_allow
    removed_denies = parent_deny - child_deny
    flags = {
        "effective_denotational_containment": not expansion,
        "allow_denotational_containment": not allow_expansion,
        "deny_preservation": parent_deny <= child_deny,
        "conflict_rule_preserved": parent["conflict_rule"] == child["conflict_rule"],
        "parent_effective_size": len(p_effective),
        "child_effective_size": len(c_effective),
        "parent_deny_size": len(parent_deny),
        "child_deny_size": len(child_deny),
        "allow_denotations_equal": parent_allow == child_allow,
    }
    if expansion:
        status = "AUTHORITY_EXPANSION"
        witness = min((requests.index(request), request) for request in expansion)[1]
        flags["first_expansion_witness"] = witness
    elif (not flags["allow_denotational_containment"] or not flags["deny_preservation"]
          or not flags["conflict_rule_preserved"]):
        status = "POLICY_ATTENUATION_VIOLATION"
    else:
        status = "EFFECTIVE_POLICY_CONTAINMENT_PASS"
    return {"status": status, **flags}


def expected_relations(raw: dict[str, Any]) -> list[tuple[str, int, int, str, str]]:
    hops = raw["hops"]
    expected: list[tuple[str, int, int, str, str]] = []
    for child_index in range(1, len(hops)):
        expected.append(("adjacent", child_index - 1, child_index,
                         hops[child_index - 1]["id"], hops[child_index]["id"]))
    for descendant_index in range(1, len(hops)):
        expected.append(("root-anchored", 0, descendant_index,
                         hops[0]["id"], hops[descendant_index]["id"]))
    return expected


def expected_chain_status(statuses: list[str]) -> str:
    if "AUTHORITY_EXPANSION" in statuses:
        return "AUTHORITY_EXPANSION"
    if "UNSUPPORTED_OR_UNDECIDABLE" in statuses:
        return "UNSUPPORTED_OR_UNDECIDABLE"
    if "POLICY_ATTENUATION_VIOLATION" in statuses:
        return "POLICY_ATTENUATION_VIOLATION"
    return "DELEGATION_CHAIN_ATTENUATION_PASS"


def audit_chain(raw: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any] | None:
    hops = raw["hops"]
    expected = expected_relations(raw)
    rows = observed.get("relations")
    if not isinstance(rows, list):
        return {"kind": "chain-relations-missing"}
    observed_keys = [
        (row.get("relation"), row.get("parent_index"), row.get("child_index"),
         row.get("parent_id"), row.get("child_id")) for row in rows
    ]
    if observed_keys != expected:
        return {"kind": "chain-relation-set-disagrees-with-independent-replay",
                "expected_relation_count": len(expected), "observed_relation_count": len(rows),
                "expected_relations": expected, "observed_relations": observed_keys}

    statuses: list[str] = []
    for row, (_, parent_i, child_i, parent_id, child_id) in zip(rows, expected):
        independent = independent_policy_replay(
            hops[parent_i]["policy"], hops[child_i]["policy"], raw["universe"]
        )
        observed_result = row.get("result", {})
        if observed_result.get("status") != independent["status"]:
            return {
                "kind": "policy-pair-status-disagrees-with-independent-replay",
                "parent_id": parent_id, "child_id": child_id,
                "expected": independent["status"], "observed": observed_result.get("status"),
            }
        for field, value in independent.items():
            if field == "status":
                continue
            if field == "first_expansion_witness":
                observed_witness = observed_result.get("counterexample", {}).get("request")
                expected_witness = {
                    "target": value[0], "purpose": value[1], "context": value[2], "amount": value[3],
                }
                if observed_witness != expected_witness:
                    return {
                        "kind": "policy-pair-witness-disagrees-with-independent-replay",
                        "parent_id": parent_id, "child_id": child_id,
                        "expected": expected_witness, "observed": observed_witness,
                    }
                continue
            expected_value = list(value) if isinstance(value, tuple) else value
            if observed_result.get(field) != expected_value:
                return {
                    "kind": "policy-pair-field-disagrees-with-independent-replay",
                    "parent_id": parent_id, "child_id": child_id,
                    "field": field, "expected": expected_value,
                    "observed": observed_result.get(field),
                }
        statuses.append(independent["status"])

    expected_status = expected_chain_status(statuses)
    if observed.get("status") != expected_status:
        return {"kind": "chain-status-disagrees-with-independent-replay",
                "expected": expected_status, "observed": observed.get("status")}
    expected_failures = sum(status in INDEPENDENT_FAILURE_STATUSES for status in statuses)
    if observed.get("failure_count") != expected_failures:
        return {"kind": "chain-failure-count-disagrees-with-independent-replay",
                "expected": expected_failures, "observed": observed.get("failure_count")}
    if observed.get("adjacent_edge_count") != len(hops) - 1:
        return {"kind": "chain-adjacent-edge-count-incorrect",
                "expected": len(hops) - 1, "observed": observed.get("adjacent_edge_count")}
    if observed.get("root_anchored_relation_count") != len(hops) - 1:
        return {"kind": "chain-root-anchor-count-incorrect",
                "expected": len(hops) - 1, "observed": observed.get("root_anchored_relation_count")}
    if observed.get("hop_count") != len(hops):
        return {"kind": "chain-hop-count-incorrect",
                "expected": len(hops), "observed": observed.get("hop_count")}
    return None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    book = fixturebook()
    receipt: dict[str, Any] = {
        "schema": "mycelix.delegation-chain-differential-receipt.v1",
        "status": "RUNNING", "qualification": "NOT_CLAIMED",
        "baseline_cases": [], "mutations": [],
    }
    try:
        require(chain_checker.MAX_DELEGATION_DEPTH == FROZEN_MAX_DELEGATION_DEPTH,
                "maximum delegation depth differs from the independently frozen value")
        require(chain_checker.MAX_HOPS == FROZEN_MAX_HOPS,
                "maximum token count differs from the independently frozen value")
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        chain_path = HERE / "delegation_chain_counterexamples.py"
        oracle_path = HERE / "compound_subsumption_counterexamples.py"
        receipt["chain_evaluator_sha256"] = hashlib.sha256(chain_path.read_bytes()).hexdigest()
        receipt["policy_oracle_sha256"] = hashlib.sha256(oracle_path.read_bytes()).hexdigest()
        receipt["test_harness_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()

        baseline_results: dict[str, dict[str, Any]] = {}
        for name, raw in book.items():
            result = chain_checker.evaluate_chain(raw)
            mismatch = audit_chain(raw, result)
            require(mismatch is None, name + ": independent baseline mismatch: " + str(mismatch))
            baseline_results[name] = result
            receipt["baseline_cases"].append({
                "id": name, "status": result["status"], "hop_count": result["hop_count"],
                "adjacent_edges": result["adjacent_edge_count"],
                "root_anchored_relations": result["root_anchored_relation_count"],
                "independent_replay": "PASS",
                "request_tuples_per_relation": len(request_tuples(raw["universe"])),
            })

        # The chain input is explicitly bounded; oversized input must fail closed.
        oversized = {
            "schema": CHAIN_SCHEMA, "universe": UNIVERSE,
            "hops": [hop(f"hop-{index}", policy(BROAD)) for index in range(FROZEN_MAX_HOPS + 1)],
        }
        oversized_result = chain_checker.evaluate_chain(oversized)
        require(oversized_result.get("status") == "UNSUPPORTED_OR_UNDECIDABLE",
                "over-depth chain was not rejected fail-closed")
        require("maximum" in oversized_result.get("reason", ""),
                "over-depth rejection lacks an explicit boundedness reason")
        duplicate_ids = copy.deepcopy(book["monotonic-four-edge-chain"])
        duplicate_ids["hops"][2]["id"] = duplicate_ids["hops"][1]["id"]
        try:
            chain_checker.evaluate_chain(duplicate_ids)
        except ValueError as error:
            require("unique" in str(error), "duplicate-id chain rejected for an unexpected reason")
            duplicate_ids_rejected = True
        else:
            duplicate_ids_rejected = False
        require(duplicate_ids_rejected, "duplicate delegation-hop IDs were accepted")
        receipt["input_guards"] = {
            "maximum_delegation_depth": FROZEN_MAX_DELEGATION_DEPTH,
            "maximum_tokens_including_root": FROZEN_MAX_HOPS,
            "over_depth_rejected": True,
            "duplicate_hop_ids_rejected": True,
        }

        # Mutant 1: check only root-to-descendant containment and omit immediate edges.
        raw = book["allow-expansion-masked-at-hop-two"]
        original_evaluate = chain_checker.evaluate_chain
        def skip_adjacent(candidate: dict[str, Any]) -> dict[str, Any]:
            result = original_evaluate(candidate)
            result["relations"] = [r for r in result["relations"] if r["relation"] != "adjacent"]
            result["adjacent_edge_count"] = 0
            observed_statuses = [r["result"].get("status") for r in result["relations"]]
            result["failure_count"] = sum(s in chain_checker.FAILURE_STATUSES for s in observed_statuses)
            result["failures"] = [r for r in result["relations"]
                                  if r["result"].get("status") in chain_checker.FAILURE_STATUSES]
            result["status"] = expected_chain_status(observed_statuses)
            return result
        try:
            chain_checker.evaluate_chain = skip_adjacent
            observed = chain_checker.evaluate_chain(raw)
            mismatch = audit_chain(raw, observed)
        finally:
            chain_checker.evaluate_chain = original_evaluate
        require(mismatch is not None and mismatch["kind"] == EXPECTED_MUTANTS["adjacent-edge-validation-skipped"],
                "adjacent-edge mutant escaped independent detection")
        receipt["mutations"].append({"id": "adjacent-edge-validation-skipped",
                                     "detected": True, "mismatch_kind": mismatch["kind"]})

        # Mutant 2: check local edges only; omit the independent root anchor records.
        raw = book["monotonic-four-edge-chain"]
        def skip_root(candidate: dict[str, Any]) -> dict[str, Any]:
            result = original_evaluate(candidate)
            result["relations"] = [r for r in result["relations"] if r["relation"] != "root-anchored"]
            result["root_anchored_relation_count"] = 0
            observed_statuses = [r["result"].get("status") for r in result["relations"]]
            result["failure_count"] = sum(s in chain_checker.FAILURE_STATUSES for s in observed_statuses)
            result["failures"] = [r for r in result["relations"]
                                  if r["result"].get("status") in chain_checker.FAILURE_STATUSES]
            result["status"] = expected_chain_status(observed_statuses)
            return result
        try:
            chain_checker.evaluate_chain = skip_root
            observed = chain_checker.evaluate_chain(raw)
            mismatch = audit_chain(raw, observed)
        finally:
            chain_checker.evaluate_chain = original_evaluate
        require(mismatch is not None and mismatch["kind"] == EXPECTED_MUTANTS["root-anchor-validation-skipped"],
                "root-anchor mutant escaped independent detection")
        receipt["mutations"].append({"id": "root-anchor-validation-skipped",
                                     "detected": True, "mismatch_kind": mismatch["kind"]})

        # Mutant 3: convert policy attenuation violations into passes.
        raw = book["allow-expansion-masked-at-hop-two"]
        original_classify = oracle.classify_policies
        def accept_attenuation_violation(parent: Any, child: Any, universe: Any) -> dict[str, Any]:
            result = original_classify(parent, child, universe)
            if result.get("status") == "POLICY_ATTENUATION_VIOLATION":
                result["status"] = "EFFECTIVE_POLICY_CONTAINMENT_PASS"
            return result
        try:
            oracle.classify_policies = accept_attenuation_violation
            observed = original_evaluate(raw)
            mismatch = audit_chain(raw, observed)
        finally:
            oracle.classify_policies = original_classify
        require(mismatch is not None and mismatch["kind"] == EXPECTED_MUTANTS["masked-attenuation-violation-accepted"],
                "masked attenuation mutant escaped independent detection")
        receipt["mutations"].append({"id": "masked-attenuation-violation-accepted",
                                     "detected": True, "mismatch_kind": mismatch["kind"]})

        # Mutant 4: a true expansion is reported only as a weaker policy violation.
        raw = book["authority-reintroduced-at-hop-two"]
        def downgrade_expansion(candidate: dict[str, Any]) -> dict[str, Any]:
            result = original_evaluate(candidate)
            if any(r["result"].get("status") == "AUTHORITY_EXPANSION" for r in result["relations"]):
                result["status"] = "POLICY_ATTENUATION_VIOLATION"
            return result
        try:
            chain_checker.evaluate_chain = downgrade_expansion
            observed = chain_checker.evaluate_chain(raw)
            mismatch = audit_chain(raw, observed)
        finally:
            chain_checker.evaluate_chain = original_evaluate
        require(mismatch is not None and mismatch["kind"] == EXPECTED_MUTANTS["chain-expansion-status-downgraded"],
                "chain status priority mutant escaped independent detection")
        receipt["mutations"].append({"id": "chain-expansion-status-downgraded",
                                     "detected": True, "mismatch_kind": mismatch["kind"]})

        require(tuple(row["id"] for row in receipt["mutations"]) == tuple(EXPECTED_MUTANTS),
                "executed mutation set differs from frozen mutation set")
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "baseline_chains": len(receipt["baseline_cases"]),
            "baseline_relations": sum(row["adjacent_edges"] + row["root_anchored_relations"]
                                      for row in receipt["baseline_cases"]),
            "request_tuples_per_relation": 32,
            "injected_mutants": len(receipt["mutations"]),
            "mutants_detected": sum(bool(row["detected"]) for row in receipt["mutations"]),
            "independent_evaluator": "RAW_JSON_FINITE_REQUEST_SETS",
            "qualification": "NOT_CLAIMED",
        }
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION-CHAIN DIFFERENTIAL PASS: 5 baseline chains independently replayed")
        print("CHAIN MUTATION SENSITIVITY PASS: 4 of 4 injected mutants detected")
        print("QUALIFICATION NOT CLAIMED: finite policy semantics only; no token cryptography/lifecycle claims")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("DELEGATION-CHAIN DIFFERENTIAL FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
