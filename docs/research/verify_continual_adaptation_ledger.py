#!/usr/bin/env python3
"""Dependency-free, policy-driven reference verifier for the ledger fixtures.

Research fixture only. This is not a production trust root.
The declared policy and corpus use an RFC 8785-compatible restricted subset
with numeric scalars excluded until full JCS number handling is implemented.
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
        raise ValueError("numeric scalar found; extend RFC 8785-compatible number handling first")
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def graph_digest(graph: dict) -> str:
    return "sha256:" + hashlib.sha256(canonical(graph)).hexdigest()


def node_index(graph: dict) -> dict[str, dict] | None:
    index: dict[str, dict] = {}
    for node in graph["nodes"]:
        node_id = node.get("id")
        if not isinstance(node_id, str) or node_id in index:
            return None
        index[node_id] = node
    return index


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
            target = operation[1]
            graph["edges"].remove(target)
        elif kind == "add_node":
            node = operation[1]
            if node["id"] in nodes:
                raise ValueError(f"duplicate node: {node['id']}")
            graph["nodes"].append(node)
        elif kind == "add_edge":
            graph["edges"].append(operation[1])
        else:
            raise ValueError(f"unknown mutation operation: {kind}")

    return graph


def verify(graph: dict, policy: dict) -> str:
    nodes = node_index(graph)
    if nodes is None:
        return "unresolved"
    edges = edge_set(graph)

    required_ids = set()
    for spec in policy["required_nodes"]:
        node_id = spec["id"]
        required_ids.add(node_id)
        if node_id not in nodes or nodes[node_id].get("type") != spec["type"]:
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
        node = nodes[rule["node"]]
        if node.get(rule["field"]) != rule["value"]:
            return rule["failure_verdict"]

    conflict = policy["result_conflict"]
    qualifying_results = [
        node for node in graph["nodes"]
        if node.get("type") == conflict["node_type"]
        and (
            node.get("id") == "result"
            or (
                node.get("id"),
                conflict["qualifies_edge_to"],
                "qualifies",
            ) in edges
        )
    ]
    if len(qualifying_results) > conflict["max_qualifying_results"]:
        return conflict["overflow_verdict"]

    dependence = policy["derived_dependence"]
    for node in graph["nodes"]:
        if (
            node.get("type") == dependence["node_type"]
            and node.get(dependence["derived_from_field"]) == dependence["source_node"]
        ):
            return dependence["verdict"]

    return "qualified"


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: verify_continual_adaptation_ledger.py POLICY.json CORPUS.json",
            file=sys.stderr,
        )
        return 2

    policy = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    corpus = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    failures: list[tuple[str, str, str, str]] = []

    for case in corpus["cases"]:
        graph = apply_mutations(corpus["base_graph"], case["mutation"])
        actual_digest = graph_digest(graph)
        actual_verdict = verify(graph, policy)

        if actual_digest != case["expected_graph_digest_sha256"]:
            failures.append(
                (
                    case["case_id"],
                    "digest",
                    case["expected_graph_digest_sha256"],
                    actual_digest,
                )
            )
        if actual_verdict != case["expected_verdict"]:
            failures.append(
                (case["case_id"], "verdict", case["expected_verdict"], actual_verdict)
            )

    print(f"cases={len(corpus['cases'])} failures={len(failures)}")
    for failure in failures:
        print("FAIL", failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
