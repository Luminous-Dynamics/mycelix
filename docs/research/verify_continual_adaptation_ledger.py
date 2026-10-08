#!/usr/bin/env python3
"""Policy-driven dependency-free reference verifier for the ledger fixtures.

Research fixture only. This is not a production trust root.
The fixture currently uses an RFC 8785-compatible JSON subset that rejects
numeric scalars before hashing.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path


def contains_number(value: object) -> bool:
    if isinstance(value, bool) or value is None:
        return False
    if isinstance(value, (int, float)):
        return True
    if isinstance(value, list):
        return any(contains_number(item) for item in value)
    if isinstance(value, dict):
        return any(contains_number(item) for item in value.values())
    return False


def canonical(value: object) -> bytes:
    if contains_number(value):
        raise ValueError("numeric scalar found")
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def node_index(graph: dict) -> dict[str, dict] | None:
    index: dict[str, dict] = {}
    for node in graph["nodes"]:
        node_id = node.get("id")
        if not isinstance(node_id, str) or node_id in index:
            return None
        index[node_id] = node
    return index


def contains_non_ascii(value: object) -> bool:
    if isinstance(value, str):
        return any(ord(char) > 0x7F for char in value)
    if isinstance(value, list):
        return any(contains_non_ascii(item) for item in value)
    if isinstance(value, dict):
        return any(contains_non_ascii(item) for item in value.values())
    return False


def semantic_normalize(graph: dict, policy: dict) -> dict | None:
    if policy["graph_canonicalization"].get("string_policy") == "ASCII-only" and contains_non_ascii(graph):
        return None
    nodes = node_index(graph)
    if nodes is None:
        return None

    seen_edges: set[tuple[str, str, str]] = set()
    for edge in graph["edges"]:
        if (
            not isinstance(edge, list)
            or len(edge) != 3
            or edge[0] not in nodes
            or edge[1] not in nodes
        ):
            return None
        key = tuple(edge)
        if policy["graph_canonicalization"]["reject_duplicate_edges"] and key in seen_edges:
            return None
        seen_edges.add(key)

    normalized = copy.deepcopy(graph)
    if policy["graph_canonicalization"]["node_collection"] == "unordered-by-id":
        normalized["nodes"] = sorted(normalized["nodes"], key=lambda n: n["id"])
    if policy["graph_canonicalization"]["edge_collection"] == "unordered-by-tuple":
        normalized["edges"] = sorted(normalized["edges"], key=canonical)
    return normalized


def semantic_digest(graph: dict, policy: dict) -> str:
    normalized = semantic_normalize(graph, policy)
    return "invalid" if normalized is None else digest(normalized)


def claim_local_projection(graph: dict, policy: dict) -> dict | None:
    normalized = semantic_normalize(graph, policy)
    if normalized is None:
        return None

    root = policy["claim_local_projection"]["root"]
    allowed = set(policy["claim_local_projection"]["relation_allowlist"])
    included = {root}

    changed = True
    while changed:
        changed = False
        for edge in normalized["edges"]:
            if edge[2] not in allowed:
                continue
            left, right = edge[0], edge[1]
            if left in included and right not in included:
                included.add(right)
                changed = True
            elif right in included and left not in included:
                included.add(left)
                changed = True

    return {
        "nodes": [copy.deepcopy(n) for n in normalized["nodes"] if n["id"] in included],
        "edges": [
            copy.deepcopy(e)
            for e in normalized["edges"]
            if e[0] in included and e[1] in included and e[2] in allowed
        ],
    }


def claim_local_digest(graph: dict, policy: dict) -> str:
    projection = claim_local_projection(graph, policy)
    return "invalid" if projection is None else digest(projection)


def validate_graph_structure(graph: dict, policy: dict) -> bool:
    nodes = node_index(graph)
    if nodes is None:
        return False
    seen_edges: set[tuple[str, str, str]] = set()
    for edge in graph["edges"]:
        if (
            not isinstance(edge, list)
            or len(edge) != 3
            or edge[0] not in nodes
            or edge[1] not in nodes
        ):
            return False
        key = tuple(edge)
        if policy["graph_canonicalization"]["reject_duplicate_edges"] and key in seen_edges:
            return False
        seen_edges.add(key)
    return True


def edge_set(graph: dict) -> set[tuple[str, str, str]]:
    return {tuple(edge) for edge in graph["edges"]}


def apply_mutations(base: dict, mutations: list[list[object]]) -> dict:
    graph = copy.deepcopy(base)
    for operation in mutations:
        kind = operation[0]
        nodes = node_index(graph)
        if nodes is None:
            raise ValueError("invalid or duplicate node id")

        if kind == "set":
            _, node_id, field, value = operation
            if node_id not in nodes:
                raise ValueError(f"unknown node: {node_id}")
            nodes[node_id][field] = value
        elif kind == "remove_edge":
            graph["edges"].remove(operation[1])
        elif kind == "add_node":
            node = operation[1]
            if node["id"] in nodes:
                raise ValueError(f"duplicate node: {node['id']}")
            graph["nodes"].append(node)
        elif kind == "add_edge":
            graph["edges"].append(operation[1])
        elif kind == "reverse_collection":
            collection = operation[1]
            if collection not in ("nodes", "edges"):
                raise ValueError(f"unsupported collection: {collection}")
            graph[collection].reverse()
        else:
            raise ValueError(f"unknown mutation operation: {kind}")

    return graph


def verify(graph: dict, policy: dict) -> str:
    if not validate_graph_structure(graph, policy):
        return "unresolved"
    nodes = node_index(graph)
    if nodes is None:
        return "unresolved"
    edges = edge_set(graph)

    for spec in policy["required_nodes"]:
        node_id, node_type = spec["id"], spec["type"]
        if node_id not in nodes or nodes[node_id].get("type") != node_type:
            return "unresolved"

    for edge in policy["required_edges"]:
        edge_tuple = tuple(edge)
        if edge_tuple not in edges:
            return "unresolved"
        if edge_tuple[0] not in nodes or edge_tuple[1] not in nodes:
            return "unresolved"

    for constraint in policy["equality_constraints"]:
        left_node, left_field = constraint["left"]
        right_node, right_field = constraint["right"]
        if nodes[left_node].get(left_field) != nodes[right_node].get(right_field):
            return constraint["failure_verdict"]

    for rule in policy["fixed_fields"]:
        if nodes[rule["node"]].get(rule["field"]) != rule["value"]:
            return rule["failure_verdict"]

    conflict = policy["result_conflict"]
    qualifying = [
        node
        for node in graph["nodes"]
        if node.get("type") == conflict["node_type"]
        and (
            node.get("id") == "result"
            or (
                node.get("id"),
                conflict["qualifies_edge_to"],
                "qualifies",
            )
            in edges
        )
    ]
    if len(qualifying) > conflict["max_qualifying_results"]:
        return conflict["overflow_verdict"]

    dependence = policy["derived_dependence"]
    if any(
        node.get("type") == dependence["node_type"]
        and (
            node.get("id"),
            dependence["source_node"],
            dependence["edge_relation"],
        ) in edges
        for node in graph["nodes"]
    ):
        return dependence["verdict"]

    return "qualified"


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: verify_continual_adaptation_ledger.py "
            "EXPECTED_POLICY_BLOB_SHA POLICY.json CORPUS.json",
            file=sys.stderr,
        )
        return 2

    expected_policy_blob_sha = sys.argv[1]
    policy = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    corpus = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))

    binding = corpus.get("policy_binding", {})
    if binding.get("git_blob_sha") != expected_policy_blob_sha:
        print("policy binding mismatch", file=sys.stderr)
        return 1
    failures: list[tuple[str, str, str, str]] = []

    for case in corpus["cases"]:
        graph = apply_mutations(corpus["base_graph"], case["mutation"])
        serialized = digest(graph)
        semantic = semantic_digest(graph, policy)
        claim_local = claim_local_digest(graph, policy)
        verdict = verify(graph, policy)

        for field, actual in (
            ("expected_graph_digest_sha256", serialized),
            ("expected_semantic_graph_digest_sha256", semantic),
            ("expected_claim_local_graph_digest_sha256", claim_local),
            ("expected_verdict", verdict),
        ):
            expected = case[field]
            if actual != expected:
                failures.append((case["case_id"], field, expected, actual))

    print(f"cases={len(corpus['cases'])} failures={len(failures)}")
    for failure in failures:
        print("FAIL", failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
