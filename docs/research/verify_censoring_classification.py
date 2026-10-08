#!/usr/bin/env python3
"""Research-only verifier for claim-local censoring-classification provenance."""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

IDENTITY_FIELDS = (
    "attempt_id",
    "censoring_reason",
    "classification_epoch",
    "frozen_epoch",
    "policy_blob_sha",
    "basis_id",
    "revision",
)
RELATIONS = (
    "requires",
    "uses",
    "classifies",
    "frozen_by",
    "supported_by",
    "supersedes",
    "invalidated_by",
)


def canonical(value: object) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(
        f"blob {len(data)}\0".encode("ascii") + data
    ).hexdigest()


def node_index(graph: dict) -> dict[str, dict] | None:
    nodes = graph.get("nodes")
    if not isinstance(nodes, list):
        return None
    out: dict[str, dict] = {}
    for node in nodes:
        if (
            not isinstance(node, dict)
            or not isinstance(node.get("id"), str)
            or node["id"] in out
        ):
            return None
        out[node["id"]] = node
    return out


def contains_non_ascii(value: object) -> bool:
    if isinstance(value, str):
        return any(ord(char) > 0x7F for char in value)
    if isinstance(value, list):
        return any(contains_non_ascii(item) for item in value)
    if isinstance(value, dict):
        return any(contains_non_ascii(item) for item in value.values())
    return False


def classification_commitment(node: dict) -> str:
    return digest({field: node[field] for field in IDENTITY_FIELDS})


def semantic_normalize(graph: dict) -> dict:
    out = copy.deepcopy(graph)
    out["nodes"] = sorted(out["nodes"], key=lambda n: n["id"])
    out["edges"] = sorted(out["edges"], key=canonical)
    return out


def validate_structure(graph: dict, policy: dict) -> tuple[bool, str]:
    if contains_non_ascii(graph):
        return False, "non-ascii"
    nodes = node_index(graph)
    if nodes is None:
        return False, "node-structure"

    claim_roots = [node for node in nodes.values() if node.get("type") == "Claim"]
    if (
        len(claim_roots) != 1
        or "claim" not in nodes
        or nodes["claim"].get("type") != "Claim"
    ):
        return False, "claim-root"

    edges = graph.get("edges")
    if not isinstance(edges, list):
        return False, "edge-structure"

    seen: set[tuple[str, str, str]] = set()
    for edge in edges:
        if (
            not isinstance(edge, list)
            or len(edge) != 3
            or not all(isinstance(v, str) for v in edge)
            or edge[0] not in nodes
            or edge[1] not in nodes
        ):
            return False, "dangling-edge"
        key = tuple(edge)
        if key in seen:
            return False, "duplicate-edge"
        seen.add(key)

        relation = edge[2]
        rule = policy.get("relations", {}).get(relation)
        if relation not in RELATIONS or rule is None:
            return False, "unknown-relation"
        if (
            nodes[edge[0]].get("type") not in rule["source_types"]
            or nodes[edge[1]].get("type") not in rule["target_types"]
        ):
            return False, "typed-endpoint"

    claim_entry_edges = [
        edge
        for edge in edges
        if edge[0] == "claim"
        and edge[2] == "requires"
        and nodes[edge[1]].get("type") == "AttemptCensus"
    ]
    if len(claim_entry_edges) != 1:
        return False, "claim-root-edge"

    order = policy["epoch_order"]
    cfg = policy["classification"]
    for node in nodes.values():
        if node.get("type") != "CensoringClassification":
            continue

        outgoing = [e for e in edges if e[0] == node["id"]]
        classifies = [e for e in outgoing if e[2] == "classifies"]
        frozen_by = [e for e in outgoing if e[2] == "frozen_by"]
        supported_by = [e for e in outgoing if e[2] == "supported_by"]
        basis_edges = [
            e for e in supported_by
            if nodes[e[1]].get("type") == "ClassificationBasis"
        ]
        supersedes = [e for e in outgoing if e[2] == "supersedes"]

        if cfg["one_target_per_classification"] and len(classifies) != 1:
            return False, "target-cardinality"
        if cfg["one_policy_per_classification"] and len(frozen_by) != 1:
            return False, "policy-cardinality"
        if cfg["one_basis_per_classification"] and len(basis_edges) != 1:
            return False, "basis-cardinality"
        if len(classifies) != 1 or len(frozen_by) != 1:
            return False, "required-provenance-missing"
        if len(basis_edges) < 1:
            return False, "basis-missing"

        attempt = nodes[classifies[0][1]]
        policy_node = nodes[frozen_by[0][1]]
        basis_node = nodes[basis_edges[0][1]]

        if node.get("attempt_id") != attempt.get("id"):
            return False, "attempt-binding"
        if cfg["class_must_match_attempt_reason"] and (
            node.get("censoring_reason") != attempt.get("censoring_reason")
        ):
            return False, "reason-mismatch"
        if node.get("basis_id") != basis_node.get("basis_commitment"):
            return False, "basis-binding"
        if node.get("policy_blob_sha") != policy_node.get("policy_blob_sha"):
            return False, "policy-binding"
        if node.get("frozen_epoch") != policy_node.get("frozen_epoch"):
            return False, "freeze-binding"

        if (
            node.get("classification_epoch") not in order
            or node.get("frozen_epoch") not in order
            or attempt.get("outcome_epoch") not in order
        ):
            return False, "epoch-unknown"

        if cfg["policy_must_be_frozen_before_outcome"] and (
            order.index(node["frozen_epoch"]) >= order.index(attempt["outcome_epoch"])
        ):
            return False, "policy-late"

        if cfg["classification_recorded_at_or_before_outcome"] and (
            order.index(node["classification_epoch"])
            > order.index(attempt["outcome_epoch"])
        ):
            return False, "classification-late"

        if node.get("commitment") != classification_commitment(node):
            return False, "commitment-mismatch"

        revision = node.get("revision")
        if not isinstance(revision, str) or not revision.isdigit():
            return False, "revision-format"
        if revision == "0":
            if supersedes:
                return False, "base-supersedes"
        elif cfg["require_supersedes_for_nonzero_revision"] and not supersedes:
            return False, "revision-without-supersession"
        if len(supersedes) > 1:
            return False, "multiple-predecessors"

        for edge in supersedes:
            old = nodes[edge[1]]
            if old.get("type") != "CensoringClassification":
                return False, "supersedes-type"
            old_targets = [
                e[1] for e in edges
                if e[0] == old["id"] and e[2] == "classifies"
            ]
            if old_targets != [attempt["id"]]:
                return False, "supersedes-target"
            old_revision = old.get("revision")
            if (
                cfg.get("require_sequential_revision", True)
                and isinstance(old_revision, str)
                and old_revision.isdigit()
                and int(revision) != int(old_revision) + 1
            ):
                return False, "revision-gap"

        if cfg["result_cannot_support_classification"]:
            if any(nodes[e[1]].get("type") == "Result" for e in supported_by):
                return False, "result-support"

    if policy["classification"].get("history_is_immutable", True):
        # Base history anchors are checked against the immutable fixture binding
        # in verify(); this policy flag controls whether that anchor is enforced.
        pass

    return True, "ok"


def claim_local_nodes(graph: dict, policy: dict) -> set[str]:
    nodes = node_index(graph)
    if (
        nodes is None
        or "claim" not in nodes
        or nodes["claim"].get("type") != "Claim"
    ):
        return set()

    allowed = set(policy["relations"])
    included = {"claim"}
    changed = True
    while changed:
        changed = False
        for edge in graph["edges"]:
            if edge[2] not in allowed or edge[0] not in included:
                continue
            if edge[1] not in included:
                included.add(edge[1])
                changed = True
    return included


def superseded_ids(graph: dict) -> set[str]:
    return {
        edge[1]
        for edge in graph["edges"]
        if edge[2] == "supersedes"
    }


def has_supersedes_cycle(graph: dict) -> bool:
    adjacency: dict[str, str] = {}
    for edge in graph["edges"]:
        if edge[2] == "supersedes":
            if edge[0] in adjacency:
                return True
            adjacency[edge[0]] = edge[1]

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> bool:
        if node in visiting:
            return True
        if node in visited:
            return False
        visiting.add(node)
        nxt = adjacency.get(node)
        if nxt is not None and visit(nxt):
            return True
        visiting.remove(node)
        visited.add(node)
        return False

    return any(visit(node) for node in adjacency)


def verify(
    graph: dict,
    policy: dict,
    actual_policy_sha: str,
    history_anchors: dict,
) -> str:
    ok, reason = validate_structure(graph, policy)
    if not ok:
        return (
            "unqualified"
            if reason in {
                "reason-mismatch",
                "basis-binding",
                "policy-binding",
                "freeze-binding",
                "commitment-mismatch",
                "attempt-binding",
                "result-support",
                "revision-format",
            }
            else "unresolved"
        )

    nodes = node_index(graph)
    assert nodes is not None
    claim_nodes = claim_local_nodes(graph, policy)

    if has_supersedes_cycle(graph):
        return "unresolved"

    if policy["classification"].get("active_classification_required", True):
        all_attempts = [
            n for n in nodes.values()
            if n.get("type") == "Attempt" and n["id"] in claim_nodes
        ]
        superseded = superseded_ids(graph)
        for attempt in all_attempts:
            classifications = [
                c for c in nodes.values()
                if (
                    c.get("type") == "CensoringClassification"
                    and c["id"] in claim_nodes
                    and any(
                        e[0] == c["id"]
                        and e[1] == attempt["id"]
                        and e[2] == "classifies"
                        for e in graph["edges"]
                    )
                )
            ]
            active = [c for c in classifications if c["id"] not in superseded]
            if len(active) != 1:
                return "unresolved"

    if policy["classification"].get("history_is_immutable", True) and policy["classification"].get("base_revision_anchor_required", True):
        for cid, anchor in history_anchors.items():
            node = nodes.get(cid)
            if node is None:
                return "unresolved"
            if node.get("type") != "CensoringClassification":
                return "unresolved"
            if node.get("revision") != "0":
                return "unresolved"
            if node.get("commitment") != anchor:
                return "unqualified"
            if cid not in claim_nodes:
                return "unresolved"
            anchor_targets = [
                edge[1]
                for edge in graph["edges"]
                if edge[0] == cid and edge[2] == "classifies"
            ]
            if len(anchor_targets) != 1:
                return "unresolved"
            anchor_attempt = nodes.get(anchor_targets[0])
            if (
                anchor_attempt is None
                or anchor_attempt.get("type") != "Attempt"
                or anchor_targets[0] not in claim_nodes
                or node.get("attempt_id") != anchor_targets[0]
            ):
                return "unresolved"

    # Exact policy identity is external to the graph and cannot be substituted.
    for node in nodes.values():
        if node.get("type") == "ClassificationPolicy" and node["id"] in claim_nodes:
            if node.get("policy_blob_sha") != actual_policy_sha:
                return "unqualified"

    # An invalidated active classification blocks the claim; historical invalidation
    # can remain visible when a valid superseding classification replaces it.
    superseded = superseded_ids(graph)
    for node in nodes.values():
        if node.get("type") != "CensoringClassification" or node["id"] not in claim_nodes:
            continue
        invalidated = any(
            e[0] == node["id"] and e[2] == "invalidated_by"
            for e in graph["edges"]
        )
        if invalidated and node["id"] not in superseded:
            return "unresolved"

    return "qualified"


def apply_mutations(base: dict, mutations: list[list[object]]) -> dict:
    graph = copy.deepcopy(base)
    for op in mutations:
        kind = op[0]
        if kind == "set_node":
            node = next(n for n in graph["nodes"] if n["id"] == op[1])
            node[op[2]] = op[3]
        elif kind == "add_node":
            graph["nodes"].append(copy.deepcopy(op[1]))
        elif kind == "add_edge":
            graph["edges"].append(copy.deepcopy(op[1]))
        elif kind == "remove_edge":
            graph["edges"].remove(op[1])
        elif kind == "remove_node":
            node_id = op[1]
            graph["nodes"] = [n for n in graph["nodes"] if n["id"] != node_id]
            graph["edges"] = [
                e for e in graph["edges"] if e[0] != node_id and e[1] != node_id
            ]
        elif kind == "reverse_collection":
            graph[op[1]].reverse()
        else:
            raise ValueError(f"unknown mutation: {kind}")
    return graph


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: verify_censoring_classification.py EXPECTED_POLICY_SHA "
            "POLICY.json FIXTURES.json REPORT.json",
            file=sys.stderr,
        )
        return 2

    expected_sha, policy_path, fixture_path, report_path = sys.argv[1:5]
    actual_sha = git_blob_sha(Path(policy_path))
    if actual_sha != expected_sha:
        print("policy binding mismatch", file=sys.stderr)
        return 1

    policy = json.loads(Path(policy_path).read_text(encoding="utf-8"))
    fixture = json.loads(Path(fixture_path).read_text(encoding="utf-8"))
    if fixture["policy_binding"]["git_blob_sha"] != actual_sha:
        print("fixture binding mismatch", file=sys.stderr)
        return 1

    failures = []
    rows = []
    for case in fixture["cases"]:
        graph = apply_mutations(fixture["base_graph"], case["mutation"])
        verdict = verify(
            graph, policy, actual_sha, fixture["history_anchors"]
        )
        rows.append({
            "actual_verdict": verdict,
            "case_id": case["case_id"],
            "expected_verdict": case["expected_verdict"],
            "claim_local_graph_digest_sha256": digest({
                "nodes": [
                    copy.deepcopy(n)
                    for n in semantic_normalize(graph)["nodes"]
                    if n["id"] in claim_local_nodes(graph, policy)
                ],
                "edges": [
                    copy.deepcopy(e)
                    for e in semantic_normalize(graph)["edges"]
                    if e[0] in claim_local_nodes(graph, policy)
                    and e[1] in claim_local_nodes(graph, policy)
                ],
            }),
        })
        if verdict != case["expected_verdict"]:
            failures.append(
                [case["case_id"], case["expected_verdict"], verdict]
            )

    report = {
        "cases": rows,
        "failures": failures,
        "policy_blob_sha": actual_sha,
        "schema": "mycelix.continual-adaptation.censoring-classification-provenance-report.v1",
        "status": "research-evidence-only",
    }
    Path(report_path).write_text(
        json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n",
        encoding="utf-8",
    )
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
