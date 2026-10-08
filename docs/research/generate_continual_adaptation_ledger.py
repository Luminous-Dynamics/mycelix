#!/usr/bin/env python3
"""Generate the bounded continual-adaptation ledger property corpus.

Research fixture only. Deterministic by construction: seed + generator
version are part of the output identity. The canonical fixture supplies
the sole base graph so the generator cannot silently create a second model.
"""
from __future__ import annotations

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


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    header = f"blob {len(data)}\\0".encode("ascii")
    return __import__("hashlib").sha1(header + data).hexdigest()


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: generate_continual_adaptation_ledger.py "
            "EXPECTED_POLICY_BLOB_SHA POLICY.json FIXED_FIXTURES.json OUTPUT.json",
            file=sys.stderr,
        )
        return 2

    expected_policy_blob_sha = sys.argv[1]
    policy_path = Path(sys.argv[2])
    fixed = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
    output_path = Path(sys.argv[4])

    actual_policy_blob_sha = git_blob_sha(policy_path)
    binding = fixed.get("policy_binding", {})
    if (
        expected_policy_blob_sha != actual_policy_blob_sha
        or binding.get("git_blob_sha") != actual_policy_blob_sha
    ):
        print("policy binding mismatch", file=sys.stderr)
        return 1
    base = fixed["base_graph"]
    state = SEED
    cases: list[dict] = []

    for i in range(32):
        state = rng_step(state)
        cases.append({
            "case_id": f"GEN-NODE-{i:03d}",
            "property": "representation_invariance",
            "mutation": [["rotate_collection", "nodes", state % len(base["nodes"])]],
        })

    for i in range(32):
        state = rng_step(state)
        cases.append({
            "case_id": f"GEN-EDGE-{i:03d}",
            "property": "representation_invariance",
            "mutation": [["rotate_collection", "edges", state % len(base["edges"])]],
        })

    for i in range(32):
        state, commitment = token(state, "N")
        cases.append({
            "case_id": f"GEN-UNRELATED-{i:03d}",
            "property": "claim_local_invariance",
            "mutation": [[
                "add_node",
                {"id": f"noise-{i:03d}", "type": "UnrelatedEvidence", "commitment": commitment},
            ]],
        })

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
        cases.append({
            "case_id": f"GEN-IDENTITY-{i:03d}",
            "property": "identity_sensitivity",
            "mutation": [["set", node, field, value]],
        })

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
    for i in range(32):
        cases.append({
            "case_id": f"GEN-MISSING-{i:03d}",
            "property": "required_dependency_rejection",
            "mutation": [["remove_edge", required_edges[i % len(required_edges)]]],
        })

    for i in range(32):
        cases.append({
            "case_id": f"GEN-DUPEDGE-{i:03d}",
            "property": "structural_rejection",
            "mutation": [["add_edge", required_edges[i % len(required_edges)]]],
        })

    for i in range(32):
        state, target = token(state, "missing-")
        cases.append({
            "case_id": f"GEN-DANGLING-{i:03d}",
            "property": "structural_rejection",
            "mutation": [["add_edge", ["claim", target, "requires"]]],
        })

    for i in range(32):
        edge = required_edges[i % len(required_edges)]
        cases.append({
            "case_id": f"GEN-ENDPOINT-{i:03d}",
            "property": "endpoint_rejection",
            "mutation": [["add_edge", [edge[1], edge[0], edge[2]]]],
        })

    for i in range(32):
        if i < 10:
            first = "first" if i % 2 == 0 else "second"
            second = "second" if i % 2 == 0 else "first"
            cases.append({
                "case_id": f"GEN-ORDERED-{i:03d}",
                "property": "ordered_array_sensitivity",
                "mutation": [["set", "claim", "ordered_probe", [first, second]]],
            })
        elif i < 20:
            result_id = f"derived-{i:03d}"
            cases.append({
                "case_id": f"GEN-DERIVED-{i:03d}",
                "property": "provenance_dependence",
                "mutation": [
                    ["add_node", {"id": result_id, "type": "Result", "commitment": "X1"}],
                    ["add_edge", [result_id, "result", "derived_from"]],
                ],
            })
        else:
            result_id = f"competing-{i:03d}"
            cases.append({
                "case_id": f"GEN-CONFLICT-{i:03d}",
                "property": "result_conflict",
                "mutation": [
                    ["add_node", {"id": result_id, "type": "Result", "commitment": f"X{i}"}],
                    ["add_edge", [result_id, "claim", "qualifies"]],
                ],
            })

    corpus = {
        "schema": "mycelix.continual-adaptation.evidence-ledger-generated-properties.v1",
        "status": "research-fixture-only",
        "generator": {
            "version": GENERATOR_VERSION,
            "seed": SEED,
            "mutation_count": len(cases),
            "endpoint_attack_count": 32,
            "base_fixture_schema": fixed.get("schema"),
        },
        "policy_binding": fixed["policy_binding"],
        "base_graph": base,
        "cases": cases,
    }

    output_path.write_text(
        json.dumps(corpus, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"generated={len(cases)} seed={SEED} version={GENERATOR_VERSION} "
        f"base_schema={fixed.get('schema')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
