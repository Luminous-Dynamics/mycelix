#!/usr/bin/env python3
"""Generate the bounded continual-adaptation ledger property corpus.

Research fixture only. Deterministic by construction: seed + generator version
are part of the output identity. No external randomness or network access.
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

SEED = 0x46480001
GENERATOR_VERSION = "ledger-property-generator-v1"


def rng_step(state: int) -> int:
    return (1664525 * state + 1013904223) & 0xFFFFFFFF


def token(state: int, prefix: str) -> tuple[int, str]:
    state = rng_step(state)
    return state, f"{prefix}{state:08x}"


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: generate_continual_adaptation_ledger.py BASE_GRAPH.json OUTPUT.json",
            file=sys.stderr,
        )
        return 2

    base = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    state = SEED
    cases: list[dict] = []

    # 32 node-order permutations.
    for i in range(32):
        state = rng_step(state)
        cases.append(
            {
                "case_id": f"GEN-NODE-{i:03d}",
                "property": "representation_invariance",
                "mutation": [["rotate_collection", "nodes", state % len(base["nodes"])]],
            }
        )

    # 32 edge-order permutations.
    for i in range(32):
        state = rng_step(state)
        cases.append(
            {
                "case_id": f"GEN-EDGE-{i:03d}",
                "property": "representation_invariance",
                "mutation": [["rotate_collection", "edges", state % len(base["edges"])]],
            }
        )

    # 32 unrelated-node additions.
    for i in range(32):
        state, commitment = token(state, "N")
        cases.append(
            {
                "case_id": f"GEN-UNRELATED-{i:03d}",
                "property": "claim_local_invariance",
                "mutation": [
                    [
                        "add_node",
                        {
                            "id": f"noise-{i:03d}",
                            "type": "UnrelatedEvidence",
                            "commitment": commitment,
                        },
                    ]
                ],
            }
        )

    # 32 claim-relevant identity corruptions.
    identity_targets = [
        ("subject", "commitment"),
        ("evaluator", "commitment"),
        ("evaluator", "state"),
        ("intervention", "semantic_id"),
        ("measurement", "semantic_id"),
    ]
    for i in range(32):
        state, value = token(state, "X")
        node, field = identity_targets[i % len(identity_targets)]
        cases.append(
            {
                "case_id": f"GEN-IDENTITY-{i:03d}",
                "property": "identity_sensitivity",
                "mutation": [["set", node, field, value]],
            }
        )

    required_edges = [
        ["result", "claim", "qualifies"],
        ["result", "evaluator", "generated_by"],
        ["evaluator", "reference", "uses"],
        ["evaluator", "attempts", "uses"],
        ["attempts", "campaign", "derived_from"],
        ["campaign", "subject", "applies_to"],
        ["campaign", "intervention", "uses"],
        ["campaign", "observation", "uses"],
        ["campaign", "measurement", "uses"],
        ["claim", "transport", "requires"],
        ["claim", "freshness", "requires"],
        ["claim", "subject", "applies_to"],
    ]
    # 32 missing-edge cases, cycling the required edge set.
    for i in range(32):
        edge = required_edges[i % len(required_edges)]
        cases.append(
            {
                "case_id": f"GEN-MISSING-{i:03d}",
                "property": "structural_rejection",
                "mutation": [["remove_edge", edge]],
            }
        )

    # 32 duplicate-edge cases.
    for i in range(32):
        edge = required_edges[i % len(required_edges)]
        cases.append(
            {
                "case_id": f"GEN-DUPEDGE-{i:03d}",
                "property": "structural_rejection",
                "mutation": [["add_edge", edge]],
            }
        )

    # 32 dangling-edge cases.
    for i in range(32):
        state, target = token(state, "missing-")
        cases.append(
            {
                "case_id": f"GEN-DANGLING-{i:03d}",
                "property": "structural_rejection",
                "mutation": [["add_edge", ["claim", target, "requires"]]],
            }
        )

    # 32 semantic relation cases:
    # 0..9 ordered-array sensitivity,
    # 10..19 explicit derivation dependence,
    # 20..31 competing qualifying result contradiction.
    for i in range(32):
        if i < 10:
            first = "first" if i % 2 == 0 else "second"
            second = "second" if i % 2 == 0 else "first"
            cases.append(
                {
                    "case_id": f"GEN-ORDERED-{i:03d}",
                    "property": "ordered_array_sensitivity",
                    "mutation": [["set", "claim", "ordered_probe", [first, second]]],
                }
            )
        elif i < 20:
            rid = f"derived-{i:03d}"
            cases.append(
                {
                    "case_id": f"GEN-DERIVED-{i:03d}",
                    "property": "provenance_dependence",
                    "mutation": [
                        ["add_node", {"id": rid, "type": "Result", "commitment": "X1"}],
                        ["add_edge", [rid, "result", "derived_from"]],
                    ],
                }
            )
        else:
            rid = f"competing-{i:03d}"
            cases.append(
                {
                    "case_id": f"GEN-CONFLICT-{i:03d}",
                    "property": "result_conflict",
                    "mutation": [
                        ["add_node", {"id": rid, "type": "Result", "commitment": f"X{i}"}],
                        ["add_edge", [rid, "claim", "qualifies"]],
                    ],
                }
            )

    corpus = {
        "schema": "mycelix.continual-adaptation.evidence-ledger-generated-properties.v1",
        "status": "research-fixture-only",
        "generator": {
            "version": GENERATOR_VERSION,
            "seed": SEED,
            "mutation_count": len(cases),
        },
        "base_graph": base,
        "cases": cases,
    }

    Path(sys.argv[2]).write_text(
        json.dumps(corpus, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"generated={len(cases)} seed={SEED} version={GENERATOR_VERSION}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
