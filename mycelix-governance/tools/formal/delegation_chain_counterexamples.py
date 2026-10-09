#!/usr/bin/env python3
"""Bounded semantic evaluator for effective-policy delegation chains.

The chain check evaluates every adjacent delegation edge and every descendant
against the root. It models policy semantics only; token signatures, parent
commitments, proof-of-possession, revocation, lifetime, and transport verification
are deliberately outside this module's qualification boundary.
"""
from __future__ import annotations

from typing import Any

import compound_subsumption_counterexamples as oracle


CHAIN_SCHEMA = "mycelix.effective-policy-delegation-chain.v1"
RESULT_SCHEMA = "mycelix.effective-policy-delegation-chain-result.v1"
MAX_DELEGATION_DEPTH = 8
MAX_HOPS = MAX_DELEGATION_DEPTH + 1

FAILURE_STATUSES = {
    "AUTHORITY_EXPANSION",
    "POLICY_ATTENUATION_VIOLATION",
    "UNSUPPORTED_OR_UNDECIDABLE",
}


def _policy_pair(parent: dict[str, Any], child: dict[str, Any],
                 universe: dict[str, Any]) -> dict[str, Any]:
    return oracle.evaluate_scenario({
        "schema": oracle.SCENARIO_SCHEMA,
        "mode": "effective-policy",
        "universe": universe,
        "parent_policy": parent,
        "child_policy": child,
    })


def evaluate_chain(raw: dict[str, Any]) -> dict[str, Any]:
    """Evaluate all adjacent edges and root-to-descendant relations fail-closed."""
    if raw.get("schema") != CHAIN_SCHEMA:
        raise ValueError(f"chain schema must be {CHAIN_SCHEMA}")
    universe = raw.get("universe")
    hops = raw.get("hops")
    if not isinstance(universe, dict):
        raise ValueError("chain requires a finite universe object")
    if not isinstance(hops, list) or len(hops) < 2:
        raise ValueError("chain requires a root and at least one delegated hop")
    if len(hops) > MAX_HOPS:
        return {
            "schema": RESULT_SCHEMA,
            "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "hop_count": len(hops),
            "failure_count": 1,
            "failures": [{"status": "UNSUPPORTED_OR_UNDECIDABLE",
                          "reason": f"chain exceeds maximum delegation depth of {MAX_DELEGATION_DEPTH} edges ({MAX_HOPS} tokens including root)"}],
            "reason": f"chain exceeds maximum delegation depth of {MAX_DELEGATION_DEPTH} edges ({MAX_HOPS} tokens including root)",
            "qualification": "NOT_CLAIMED",
        }
    ids = [hop.get("id") if isinstance(hop, dict) else None for hop in hops]
    if any(not isinstance(item, str) or not item for item in ids):
        raise ValueError("each chain hop requires a non-empty id")
    if len(set(ids)) != len(ids):
        raise ValueError("chain hop ids must be unique")

    relations: list[dict[str, Any]] = []
    for child_index in range(1, len(hops)):
        parent, child = hops[child_index - 1], hops[child_index]
        relations.append({
            "relation": "adjacent",
            "parent_id": parent["id"],
            "child_id": child["id"],
            "parent_index": child_index - 1,
            "child_index": child_index,
            "result": _policy_pair(parent["policy"], child["policy"], universe),
        })

    root = hops[0]
    for descendant_index in range(1, len(hops)):
        descendant = hops[descendant_index]
        # The adjacent root edge appears twice semantically by design: once as
        # an edge invariant and once as a root-anchored transitive invariant.
        relations.append({
            "relation": "root-anchored",
            "parent_id": root["id"],
            "child_id": descendant["id"],
            "parent_index": 0,
            "child_index": descendant_index,
            "result": _policy_pair(root["policy"], descendant["policy"], universe),
        })

    failures = [
        {
            "relation": row["relation"],
            "parent_id": row["parent_id"],
            "child_id": row["child_id"],
            "status": row["result"].get("status"),
            "reason": row["result"].get("reason"),
            "counterexample": row["result"].get("counterexample"),
        }
        for row in relations
        if row["result"].get("status") in FAILURE_STATUSES
    ]

    if any(row["status"] == "AUTHORITY_EXPANSION" for row in failures):
        status = "AUTHORITY_EXPANSION"
    elif any(row["status"] == "UNSUPPORTED_OR_UNDECIDABLE" for row in failures):
        status = "UNSUPPORTED_OR_UNDECIDABLE"
    elif failures:
        status = "POLICY_ATTENUATION_VIOLATION"
    else:
        status = "DELEGATION_CHAIN_ATTENUATION_PASS"

    return {
        "schema": RESULT_SCHEMA,
        "status": status,
        "hop_count": len(hops),
        "adjacent_edge_count": len(hops) - 1,
        "root_anchored_relation_count": len(hops) - 1,
        "relations": relations,
        "failure_count": len(failures),
        "failures": failures,
        "qualification": "NOT_CLAIMED",
        "scope": (
            "bounded effective-policy semantics only; no cryptographic linkage, "
            "token signature, proof-of-possession, revocation, or lifetime validation"
        ),
    }
