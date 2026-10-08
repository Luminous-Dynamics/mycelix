#!/usr/bin/env python3
"""Dependency-free reference verifier for the continual-adaptation ledger fixtures.

Research fixture only. This is not a production trust root.
The fixture corpus intentionally restricts scalar data to I-JSON-safe strings,
booleans, arrays, and objects; extend JCS number handling before adding numeric cases.
"""

from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

REQUIRED_EDGES = {
    ("result", "claim", "qualifies"),
    ("result", "evaluator", "generated_by"),
    ("evaluator", "reference", "uses"),
    ("evaluator", "attempts", "uses"),
    ("attempts", "campaign", "derived_from"),
    ("campaign", "subject", "applies_to"),
    ("campaign", "intervention", "uses"),
    ("campaign", "observation", "uses"),
    ("campaign", "measurement", "uses"),
    ("claim", "transport", "requires"),
    ("claim", "freshness", "requires"),
    ("claim", "subject", "applies_to"),
}


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


def node_map(graph: dict) -> dict[str, dict]:
    return {node["id"]: node for node in graph["nodes"]}


def edge_set(graph: dict) -> set[tuple[str, str, str]]:
    return {tuple(edge) for edge in graph["edges"]}


def apply_mutations(base: dict, mutations: list[list[object]]) -> dict:
    graph = copy.deepcopy(base)
    for operation in mutations:
        kind = operation[0]
        nodes = node_map(graph)

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
        else:
            raise ValueError(f"unknown mutation operation: {kind}")

    return graph


def verify(graph: dict) -> str:
    nodes = node_map(graph)
    edges = edge_set(graph)
    required_ids = {
        "claim",
        "subject",
        "campaign",
        "attempts",
        "evaluator",
        "reference",
        "intervention",
        "observation",
        "measurement",
        "result",
        "transport",
        "freshness",
    }

    if not required_ids <= set(nodes):
        return "unresolved"

    if not REQUIRED_EDGES <= edges:
        return "unresolved"

    claim = nodes["claim"]
    subject = nodes["subject"]
    evaluator = nodes["evaluator"]
    intervention = nodes["intervention"]
    measurement = nodes["measurement"]
    transport = nodes["transport"]

    if claim.get("subject") != subject.get("commitment"):
        return "unqualified"
    if claim.get("target") != transport.get("target"):
        return "unqualified"
    if evaluator.get("commitment") != "E1":
        return "unqualified"
    if evaluator.get("state") != "fresh":
        return "unqualified"
    if intervention.get("semantic_id") != "U1":
        return "unqualified"
    if measurement.get("semantic_id") != "M1":
        return "unqualified"

    qualifying_results = [
        node
        for node in graph["nodes"]
        if node.get("type") == "Result"
        and (node["id"] == "result" or (node["id"], "claim", "qualifies") in edges)
    ]
    if len(qualifying_results) > 1:
        return "unresolved"

    derived_copies = [
        node
        for node in graph["nodes"]
        if node.get("type") == "Result" and node.get("derived_from") == "result"
    ]
    if derived_copies:
        return "qualified-with-dependence"

    return "qualified"


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: verify_continual_adaptation_ledger.py CORPUS.json", file=sys.stderr)
        return 2

    corpus = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    failures: list[tuple[str, str, str, str]] = []

    for case in corpus["cases"]:
        graph = apply_mutations(corpus["base_graph"], case["mutation"])
        actual_digest = graph_digest(graph)
        actual_verdict = verify(graph)

        if actual_digest != case["expected_graph_digest_sha256"]:
            failures.append(
                (case["case_id"], "digest", case["expected_graph_digest_sha256"], actual_digest)
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
