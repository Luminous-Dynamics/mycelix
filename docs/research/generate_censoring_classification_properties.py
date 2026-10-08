#!/usr/bin/env python3
"""Generate deterministic compositional censoring-provenance mutations.

Research fixture only. The generated corpus is a bounded metamorphic campaign.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

SEED = 0x43505601
VERSION = "censoring-classification-provenance-v3"

FIELDS = (
    "attempt_id",
    "censoring_reason",
    "classification_epoch",
    "frozen_epoch",
    "policy_blob_sha",
    "basis_id",
    "revision",
)


def canonical(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def classification_commitment(node: dict) -> str:
    return digest({field: node[field] for field in FIELDS})


def rng_step(state: int) -> int:
    state ^= (state << 13) & 0xFFFFFFFF
    state ^= state >> 17
    state ^= (state << 5) & 0xFFFFFFFF
    return state & 0xFFFFFFFF


def revision_node(policy_sha: str, revision: str = "1", classification_epoch: str = "t0",
                  frozen_epoch: str = "t0") -> dict:
    node = {
        "id": "c02",
        "type": "CensoringClassification",
        "attempt_id": "a01",
        "censoring_reason": "ActionInducedCensoring",
        "classification_epoch": classification_epoch,
        "frozen_epoch": frozen_epoch,
        "policy_blob_sha": policy_sha,
        "basis_id": "B1",
        "revision": revision,
    }
    node["commitment"] = classification_commitment(node)
    return node


def add_valid_revision(policy_sha: str) -> list[list[object]]:
    return [
        ["add_node", revision_node(policy_sha)],
        ["add_edge", ["attempts", "c02", "uses"]],
        ["add_edge", ["c02", "a01", "classifies"]],
        ["add_edge", ["c02", "p01", "frozen_by"]],
        ["add_edge", ["c02", "b01", "supported_by"]],
        ["add_edge", ["c02", "c01", "supersedes"]],
    ]


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: generate_censoring_classification_properties.py POLICY.json FIXTURES.json OUTPUT.json", file=sys.stderr)
        return 2

    policy_path, fixture_path, output_path = map(Path, sys.argv[1:4])
    policy_bytes = policy_path.read_bytes()
    policy_sha = hashlib.sha1(
        f"blob {len(policy_bytes)}\0".encode("ascii") + policy_bytes
    ).hexdigest()

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    if fixture["policy_binding"]["git_blob_sha"] != policy_sha:
        print("policy binding mismatch", file=sys.stderr)
        return 1

    base = fixture["base_graph"]
    cases: list[dict] = []
    state = SEED

    for i in range(16):
        state = rng_step(state)
        mutation = [["reverse_collection", "nodes"]]
        if state & 1:
            mutation.append(["reverse_collection", "edges"])
        cases.append({
            "case_id": f"CPV-GEN-REP-{i:03d}",
            "property": "representation_invariance",
            "mutation": mutation,
            "expected_verdict": "qualified",
        })

    lineage_mutations = [
        (
            [["remove_edge", ["attempts", "c01", "uses"]], ["add_node", {**revision_node(policy_sha, revision="0" )}],
             ["add_edge", ["attempts", "c02", "uses"]],
             ["add_edge", ["c02", "a01", "classifies"]],
             ["add_edge", ["c02", "p01", "frozen_by"]],
             ["add_edge", ["c02", "b01", "supported_by"]]],
            "unresolved",
        ),
        (
            [["remove_edge", ["c01", "a01", "classifies"]], ["add_node", {**revision_node(policy_sha, revision="0" )}],
             ["add_edge", ["attempts", "c02", "uses"]],
             ["add_edge", ["c02", "a01", "classifies"]],
             ["add_edge", ["c02", "p01", "frozen_by"]],
             ["add_edge", ["c02", "b01", "supported_by"]]],
            "unresolved",
        ),
        (
            [["remove_edge", ["attempts", "c01", "uses"]]] + add_valid_revision(policy_sha),
            "qualified",
        ),
        (
            [["remove_edge", ["attempts", "a01", "uses"]]],
            "unresolved",
        ),
        (
            [["remove_edge", ["attempts", "c01", "uses"], ["remove_edge", ["c01", "a01", "classifies"]]] + [
                ["add_node", revision_node(policy_sha, revision="0")],
                ["add_edge", ["attempts", "c02", "uses"]],
                ["add_edge", ["c02", "a01", "classifies"]],
                ["add_edge", ["c02", "p01", "frozen_by"]],
                ["add_edge", ["c02", "b01", "supported_by"]],
            ],
            "unresolved",
        ),
        (
            [["remove_edge", ["c01", "a01", "classifies"]]] + add_valid_revision(policy_sha),
            "unresolved",
        ),
        (
            [["remove_edge", ["attempts", "a01", "uses"]]] + add_valid_revision(policy_sha),
            "unresolved",
        ),
        (
            [["remove_edge", ["attempts", "c01", "uses"]]] + add_valid_revision(policy_sha) + [["reverse_collection", "nodes"], ["reverse_collection", "edges"]],
            "qualified",
        ),
    ]
    for i, (mutation, expected) in enumerate(lineage_mutations):
        cases.append({
            "case_id": f"CPV-GEN-LINEAGE-LOCAL-{i:03d}",
            "property": "anchor_claim_local_lineage",
            "mutation": mutation,
            "expected_verdict": expected,
        })
    for i in range(8):
        state = rng_step(state)
        base_lineage = lineage_mutations[i][0]
        mutation = copy.deepcopy(base_lineage)
        if state & 1:
            mutation.append(["reverse_collection", "nodes"])
        if state & 2:
            mutation.append(["reverse_collection", "edges"])
        cases.append({
            "case_id": f"CPV-GEN-LINEAGE-LOCAL-REP-{i:03d}",
            "property": "anchor_claim_local_lineage_representation",
            "mutation": mutation,
            "expected_verdict": lineage_mutations[i][1],
        })


    def off_claim_scope_attack(policy_sha: str, replacement_revision: str = "0") -> list[list[object]]:
        base_node = revision_node(policy_sha, revision=replacement_revision)
        base_node["id"] = "c02"
        base_node["commitment"] = classification_commitment(base_node)
        foreign = revision_node(policy_sha, revision="1")
        foreign["id"] = "c99"
        foreign["commitment"] = classification_commitment(foreign)
        mutation = [
            ["add_node", base_node],
            ["add_edge", ["attempts", "c02", "uses"]],
            ["add_edge", ["c02", "a01", "classifies"]],
            ["add_edge", ["c02", "p01", "frozen_by"]],
            ["add_edge", ["c02", "b01", "supported_by"]],
            ["add_node", foreign],
            ["add_edge", ["c99", "a01", "classifies"]],
            ["add_edge", ["c99", "p01", "frozen_by"]],
            ["add_edge", ["c99", "b01", "supported_by"]],
            ["add_edge", ["c99", "c01", "supersedes"]],
        ]
        return mutation

    scope_mutations = [
        (off_claim_scope_attack(policy_sha, "0"), "unresolved"),
        (off_claim_scope_attack(policy_sha, "0") + [["reverse_collection", "nodes"], ["reverse_collection", "edges"]], "unresolved"),
        (off_claim_scope_attack(policy_sha, "1") + [
            ["add_edge", ["c02", "c01", "supersedes"]]
        ], "unresolved"),
        (off_claim_scope_attack(policy_sha, "0") + [
            ["add_node", {"id": "i01", "type": "InvalidationRecord", "reason": "historical invalidation", "epoch": "t1"}],
            ["add_edge", ["c01", "i01", "invalidated_by"]]
        ], "unresolved"),
        (off_claim_scope_attack(policy_sha, "0") + [["reverse_collection", "nodes"]], "unresolved"),
        (off_claim_scope_attack(policy_sha, "0") + [["reverse_collection", "edges"]], "unresolved"),
        (off_claim_scope_attack(policy_sha, "0") + [["reverse_collection", "nodes"], ["reverse_collection", "edges"], ["reverse_collection", "nodes"]], "unresolved"),
        (off_claim_scope_attack(policy_sha, "1") + [["reverse_collection", "nodes"], ["reverse_collection", "edges"]], "unresolved"),
    ]
    for i, (mutation, expected) in enumerate(scope_mutations):
        cases.append({
            "case_id": f"CPV-GEN-SCOPE-{i:03d}",
            "property": "claim_local_supersession_scope",
            "mutation": mutation,
            "expected_verdict": expected,
        })

    omission_mutations = [
        [["remove_node", "c01"]],
        [["remove_node", "c01"], ["add_node", revision_node(policy_sha, revision="0")],
         ["add_edge", ["attempts", "c02", "uses"]],
         ["add_edge", ["c02", "a01", "classifies"]],
         ["add_edge", ["c02", "p01", "frozen_by"]],
         ["add_edge", ["c02", "b01", "supported_by"]]],
        [["remove_node", "claim"]],
        [["remove_node", "claim"], ["add_node", {"id": "claim", "type": "Result"}]],
        [["remove_node", "claim"], ["add_node", {"id": "claim", "type": "Claim"}]],
        [["remove_node", "attempts"]],
        [["remove_node", "p01"]],
        [["remove_node", "b01"]],
        [["remove_node", "a01"]],
        [["remove_node", "c01"], ["add_node", revision_node(policy_sha, revision="1")]],
        [["remove_node", "c01"], ["add_node", revision_node(policy_sha, revision="1")],
         ["add_edge", ["attempts", "c02", "uses"]],
         ["add_edge", ["c02", "a01", "classifies"]],
         ["add_edge", ["c02", "p01", "frozen_by"]],
         ["add_edge", ["c02", "b01", "supported_by"]]],
        [["remove_node", "claim"], ["remove_node", "c01"]],
        [["remove_node", "p01"], ["remove_node", "b01"]],
        [["remove_node", "a01"], ["remove_node", "c01"]],
        [["remove_node", "c01"], ["remove_node", "claim"], ["add_node", {"id": "claim", "type": "Claim"}]],
    ]
    for i, mutation in enumerate(omission_mutations):
        cases.append({
            "case_id": f"CPV-GEN-OMIT-{i:03d}",
            "property": "omission_and_anchor_presence",
            "mutation": mutation,
            "expected_verdict": "unresolved",
        })

    for i in range(24):
        mode = i % 6
        if mode == 0:
            mutation = [["set_node", "c01", "censoring_reason", "InfrastructureCensoring"]]
        elif mode == 1:
            mutation = [["set_node", "c01", "policy_blob_sha", "P2"]]
        elif mode == 2:
            mutation = [["set_node", "c01", "basis_id", "B2"]]
        elif mode == 3:
            mutation = [["set_node", "c01", "commitment", "sha256:bad"]]
        elif mode == 4:
            altered = copy.deepcopy(next(n for n in base["nodes"] if n["id"] == "c01"))
            altered["frozen_epoch"] = "t1"
            mutation = [
                ["set_node", "c01", "frozen_epoch", "t1"],
                ["set_node", "c01", "commitment", altered.pop("commitment")],
            ]
            altered["commitment"] = classification_commitment(altered)
            mutation[-1][3] = altered["commitment"]
        else:
            altered = copy.deepcopy(next(n for n in base["nodes"] if n["id"] == "c01"))
            altered["classification_epoch"] = "t2"
            altered["commitment"] = classification_commitment(altered)
            mutation = [
                ["set_node", "c01", "classification_epoch", "t2"],
                ["set_node", "c01", "commitment", altered["commitment"]],
            ]
        cases.append({
            "case_id": f"CPV-GEN-ID-{i:03d}",
            "property": "content_or_temporal_binding",
            "mutation": mutation,
            "expected_verdict": "unqualified" if mode < 4 else "unresolved",
        })

    structural_edges = [
        ["claim", "attempts", "requires"],
        ["attempts", "a01", "uses"],
        ["attempts", "c01", "uses"],
        ["c01", "p01", "frozen_by"],
        ["c01", "b01", "supported_by"],
    ]
    for i in range(24):
        mode = i % 6
        if mode < 5:
            mutation = [["remove_edge", structural_edges[mode]]]
        elif i % 2:
            mutation = [["add_edge", ["c01", "a01", "classifies"]]]
        else:
            mutation = [["add_edge", ["a01", "c01", "classifies"]]]
        cases.append({
            "case_id": f"CPV-GEN-STRUCT-{i:03d}",
            "property": "structural_or_claim_local_rejection",
            "mutation": mutation,
            "expected_verdict": "unresolved",
        })

    for i in range(24):
        mode = i % 6
        if mode == 0:
            mutation = add_valid_revision(policy_sha)
            expected = "qualified"
        elif mode == 1:
            mutation = add_valid_revision(policy_sha)
            mutation[0][1]["classification_epoch"] = "t2"
            mutation[0][1]["commitment"] = classification_commitment(mutation[0][1])
            expected = "unresolved"
        elif mode == 2:
            mutation = add_valid_revision(policy_sha)
            mutation[0][1]["frozen_epoch"] = "t1"
            mutation[0][1]["commitment"] = classification_commitment(mutation[0][1])
            expected = "unresolved"
        elif mode == 3:
            mutation = add_valid_revision(policy_sha)
            mutation[0][1]["revision"] = "3"
            mutation[0][1]["commitment"] = classification_commitment(mutation[0][1])
            expected = "unresolved"
        elif mode == 4:
            first = revision_node(policy_sha)
            first["id"] = "c02"
            first["commitment"] = classification_commitment(first)
            second = revision_node(policy_sha)
            second["id"] = "c03"
            second["commitment"] = classification_commitment(second)
            mutation = [
                ["add_node", first],
                ["add_node", second],
                ["add_edge", ["attempts", "c02", "uses"]],
                ["add_edge", ["c02", "a01", "classifies"]],
                ["add_edge", ["c02", "p01", "frozen_by"]],
                ["add_edge", ["c02", "b01", "supported_by"]],
                ["add_edge", ["attempts", "c03", "uses"]],
                ["add_edge", ["c03", "a01", "classifies"]],
                ["add_edge", ["c03", "p01", "frozen_by"]],
                ["add_edge", ["c03", "b01", "supported_by"]],
                ["add_edge", ["c02", "c01", "supersedes"]],
                ["add_edge", ["c03", "c01", "supersedes"]],
            ]
            expected = "unresolved"
        else:
            mutation = add_valid_revision(policy_sha)
            mutation[0][1]["attempt_id"] = "a02"
            mutation[0][1]["commitment"] = classification_commitment(mutation[0][1])
            mutation.append([
                "add_node",
                {"id":"a02","type":"Attempt","censoring_reason":"ActionInducedCensoring","outcome_epoch":"t1","action":"UPDATE"},
            ])
            expected = "unqualified"
        cases.append({
            "case_id": f"CPV-GEN-LINEAGE-{i:03d}",
            "property": "revision_and_supersession_integrity",
            "mutation": mutation,
            "expected_verdict": expected,
        })

    for i in range(16):
        mutation = [
            ["add_node", {"id": "r01", "type": "Result"}],
            ["add_edge", ["c01", "r01", "supported_by"]],
        ]
        if i % 2:
            mutation.extend(add_valid_revision(policy_sha))
        cases.append({
            "case_id": f"CPV-GEN-RESULT-{i:03d}",
            "property": "result_independence_boundary",
            "mutation": mutation,
            "expected_verdict": "unqualified",
        })

    for i in range(24):
        mode = i % 4
        if mode == 0:
            mutation = [
                ["set_node", "c01", "censoring_reason", "InfrastructureCensoring"],
                ["add_node", {"id": "r01", "type": "Result"}],
                ["add_edge", ["c01", "r01", "supported_by"]],
            ]
            expected = "unqualified"
        elif mode == 1:
            mutation = [
                ["set_node", "c01", "commitment", "sha256:bad"],
            ]
            expected = "unqualified"
        elif mode == 2:
            mutation = [
                ["remove_edge", ["attempts", "c01", "uses"]],
            ]
            expected = "unresolved"
        else:
            mutation = add_valid_revision(policy_sha) + [
                ["add_node", {"id": "r01", "type": "Result"}],
                ["add_edge", ["c02", "r01", "supported_by"]],
            ]
            expected = "unqualified"
        cases.append({
            "case_id": f"CPV-GEN-COMPOSE-{i:03d}",
            "property": "compositional_nonpositive",
            "mutation": mutation,
            "expected_verdict": expected,
        })

    assert len(cases) == 168

    corpus = {
        "schema": "mycelix.continual-adaptation.censoring-classification-provenance-generated-properties.v1",
        "status": "research-fixture-only",
        "generator": {
            "version": VERSION,
            "seed": SEED,
            "mutation_count": len(cases),
        },
        "policy_binding": fixture["policy_binding"],
        "history_anchors": fixture["history_anchors"],
        "base_graph": base,
        "cases": cases,
    }
    payload = json.dumps(corpus, ensure_ascii=False, indent=2) + "\n"
    output_path.write_text(payload, encoding="utf-8")
    raw = output_path.read_bytes()
    print(f"generated={len(cases)} seed=0x{SEED:08x} sha256={hashlib.sha256(raw).hexdigest()} bytes={len(raw)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
