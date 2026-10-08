#!/usr/bin/env python3
"""Independent property evaluator for the generated continual-adaptation ledger corpus.

Research fixture only. The assertions are metamorphic relations over the declared
policy rather than an oracle of scientific truth.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path


def has_number(value: object) -> bool:
    if isinstance(value, bool) or value is None:
        return False
    if isinstance(value, (int, float)):
        return True
    if isinstance(value, list):
        return any(has_number(v) for v in value)
    if isinstance(value, dict):
        return any(has_number(v) for v in value.values())
    return False


def canonical(value: object) -> bytes:
    if has_number(value):
        raise ValueError("numeric scalar")
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def node_index(graph: dict) -> dict[str, dict] | None:
    out: dict[str, dict] = {}
    for node in graph["nodes"]:
        node_id = node.get("id")
        if not isinstance(node_id, str) or node_id in out:
            return None
        out[node_id] = node
    return out


def normalize(graph: dict, policy: dict) -> dict | None:
    nodes = node_index(graph)
    if nodes is None:
        return None
    seen: set[tuple[str, str, str]] = set()
    for edge in graph["edges"]:
        if not isinstance(edge, list) or len(edge) != 3 or edge[0] not in nodes or edge[1] not in nodes:
            return None
        key = tuple(edge)
        if policy["graph_canonicalization"]["reject_duplicate_edges"] and key in seen:
            return None
        seen.add(key)

    out = copy.deepcopy(graph)
    if policy["graph_canonicalization"]["node_collection"] == "unordered-by-id":
        out["nodes"].sort(key=lambda n: n["id"])
    if policy["graph_canonicalization"]["edge_collection"] == "unordered-by-tuple":
        out["edges"].sort(key=canonical)
    return out


def semantic_digest(graph: dict, policy: dict) -> str:
    out = normalize(graph, policy)
    return "invalid" if out is None else digest(out)


def claim_local_digest(graph: dict, policy: dict) -> str:
    normalized = normalize(graph, policy)
    if normalized is None:
        return "invalid"

    root = policy["claim_local_projection"]["root"]
    allowed = set(policy["claim_local_projection"]["relation_allowlist"])
    included = {root}
    changed = True
    while changed:
        changed = False
        for edge in normalized["edges"]:
            if edge[2] not in allowed:
                continue
            left, right = edge
            if left in included and right not in included:
                included.add(right)
                changed = True
            elif right in included and left not in included:
                included.add(left)
                changed = True

    projection = {
        "nodes": [copy.deepcopy(n) for n in normalized["nodes"] if n["id"] in included],
        "edges": [
            copy.deepcopy(e)
            for e in normalized["edges"]
            if e[0] in included and e[1] in included and e[2] in allowed
        ],
    }
    return digest(projection)


def verify(graph: dict, policy: dict) -> str:
    if normalize(graph, policy) is None:
        return "unresolved"
    nodes = node_index(graph)
    if nodes is None:
        return "unresolved"
    edges = {tuple(e) for e in graph["edges"]}

    for spec in policy["required_nodes"]:
        if spec["id"] not in nodes or nodes[spec["id"]].get("type") != spec["type"]:
            return "unresolved"
    for edge in policy["required_edges"]:
        if tuple(edge) not in edges:
            return "unresolved"
    for rule in policy["equality_constraints"]:
        lnode, lfield = rule["left"]
        rnode, rfield = rule["right"]
        if nodes[lnode].get(lfield) != nodes[rnode].get(rfield):
            return rule["failure_verdict"]
    for rule in policy["fixed_fields"]:
        if nodes[rule["node"]].get(rule["field"]) != rule["value"]:
            return rule["failure_verdict"]

    conflict = policy["result_conflict"]
    qualifying = [
        n for n in graph["nodes"]
        if n.get("type") == conflict["node_type"]
        and (
            n.get("id") == "result"
            or (n.get("id"), conflict["qualifies_edge_to"], "qualifies") in edges
        )
    ]
    if len(qualifying) > conflict["max_qualifying_results"]:
        return conflict["overflow_verdict"]

    dependence = policy["derived_dependence"]
    if any(
        n.get("type") == dependence["node_type"]
        and (n.get("id"), dependence["source_node"], dependence["edge_relation"]) in edges
        for n in graph["nodes"]
    ):
        return dependence["verdict"]
    return "qualified"


def rotate(items: list, offset: int) -> None:
    if items:
        offset %= len(items)
        items[:] = items[offset:] + items[:offset]


def apply_mutations(base: dict, mutations: list[list[object]]) -> dict:
    graph = copy.deepcopy(base)
    for op in mutations:
        kind = op[0]
        nodes = node_index(graph)
        if nodes is None:
            raise ValueError("invalid node structure")
        if kind == "rotate_collection":
            collection, offset = op[1], int(op[2])
            if collection not in ("nodes", "edges"):
                raise ValueError("unsupported collection")
            rotate(graph[collection], offset)
        elif kind == "set":
            _, node_id, field, value = op
            if node_id not in nodes:
                raise ValueError("unknown node")
            nodes[node_id][field] = value
        elif kind == "add_node":
            node = op[1]
            if node["id"] in nodes:
                raise ValueError("duplicate node")
            graph["nodes"].append(node)
        elif kind == "add_edge":
            graph["edges"].append(op[1])
        elif kind == "remove_edge":
            graph["edges"].remove(op[1])
        else:
            raise ValueError(f"unknown mutation: {kind}")
    return graph


def check_case(base: dict, policy: dict, case: dict) -> dict:
    graph = apply_mutations(base, case["mutation"])
    base_semantic = semantic_digest(base, policy)
    base_claim = claim_local_digest(base, policy)
    base_verdict = verify(base, policy)
    actual = {
        "case_id": case["case_id"],
        "property": case["property"],
        "verdict": verify(graph, policy),
        "semantic_digest": semantic_digest(graph, policy),
        "claim_local_digest": claim_local_digest(graph, policy),
        "base_verdict": base_verdict,
        "base_semantic_digest": base_semantic,
        "base_claim_local_digest": base_claim,
    }
    prop = case["property"]

    if prop == "representation_invariance":
        ok = (
            actual["verdict"] == base_verdict
            and actual["semantic_digest"] == base_semantic
            and actual["claim_local_digest"] == base_claim
        )
    elif prop == "claim_local_invariance":
        ok = (
            actual["verdict"] == base_verdict
            and actual["claim_local_digest"] == base_claim
            and actual["semantic_digest"] != base_semantic
        )
    elif prop == "identity_sensitivity":
        ok = actual["verdict"] != "qualified" and actual["semantic_digest"] != base_semantic
    elif prop == "structural_rejection":
        ok = actual["verdict"] == "unresolved" and actual["semantic_digest"] == "invalid"
    elif prop == "ordered_array_sensitivity":
        ok = actual["verdict"] == "qualified" and actual["semantic_digest"] != base_semantic and actual["claim_local_digest"] != base_claim
    elif prop == "provenance_dependence":
        ok = actual["verdict"] == "qualified-with-dependence" and actual["claim_local_digest"] != base_claim
    elif prop == "result_conflict":
        ok = actual["verdict"] == "unresolved" and actual["claim_local_digest"] != base_claim
    else:
        raise ValueError(f"unknown property: {prop}")

    actual["status"] = "pass" if ok else "fail"
    return actual


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: verify_generated_properties.py POLICY.json GENERATED.json REPORT.json", file=sys.stderr)
        return 2
    policy = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    corpus = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    report = [check_case(corpus["base_graph"], policy, case) for case in corpus["cases"]]
    Path(sys.argv[3]).write_text(
        json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    failures = [r for r in report if r["status"] != "pass"]
    print(f"cases={len(report)} failures={len(failures)}")
    if failures:
        for failure in failures[:10]:
            print("FAIL", failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
