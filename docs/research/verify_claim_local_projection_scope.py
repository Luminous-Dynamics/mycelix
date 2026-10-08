#!/usr/bin/env python3
"""Claim-local projection scope verifier for shared-claim fixtures.

Research fixture only. It specifically tests that directed provenance closure
does not cross from Claim A into Claim B merely because subjects/evaluators are shared.
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


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def node_index(graph: dict) -> dict[str, dict] | None:
    out: dict[str, dict] = {}
    for node in graph["nodes"]:
        node_id = node.get("id")
        if not isinstance(node_id, str) or node_id in out:
            return None
        out[node_id] = node
    return out


def edge_schema_valid(edge: list[object], nodes: dict[str, dict], policy: dict) -> bool:
    rules = [
        rule
        for rule in policy.get("edge_schema_constraints", [])
        if rule["relation"] == edge[2]
    ]
    return any(
        nodes[edge[0]].get("type") in rule["source_types"]
        and nodes[edge[1]].get("type") in rule["target_types"]
        for rule in rules
    )


def normalize(graph: dict, policy: dict) -> dict | None:
    nodes = node_index(graph)
    if nodes is None:
        return None
    seen: set[tuple[str, str, str]] = set()
    for edge in graph["edges"]:
        if not isinstance(edge, list) or len(edge) != 3:
            return None
        if edge[0] not in nodes or edge[1] not in nodes:
            return None
        if not edge_schema_valid(edge, nodes, policy):
            return None
        key = tuple(edge)
        if key in seen:
            return None
        seen.add(key)
    out = copy.deepcopy(graph)
    out["nodes"].sort(key=lambda n: n["id"])
    out["edges"].sort(key=canonical)
    return out


def project(graph: dict, policy: dict, claim_id: str) -> dict | None:
    normalized = normalize(graph, policy)
    if normalized is None:
        return None
    ids = {node["id"] for node in normalized["nodes"]}
    if claim_id not in ids:
        return None
    allowed = set(policy["claim_local_projection"]["relation_allowlist"])
    directions = policy["claim_local_projection"]["relation_directions"]
    included = {claim_id}
    changed = True
    while changed:
        changed = False
        for edge in normalized["edges"]:
            if edge[2] not in allowed:
                continue
            ds = set(directions.get(edge[2], []))
            if "outgoing" in ds and edge[0] in included and edge[1] not in included:
                included.add(edge[1])
                changed = True
            if "incoming" in ds and edge[1] in included and edge[0] not in included:
                included.add(edge[0])
                changed = True
    return {
        "nodes": [copy.deepcopy(n) for n in normalized["nodes"] if n["id"] in included],
        "edges": [
            copy.deepcopy(e)
            for e in normalized["edges"]
            if e[0] in included and e[1] in included and e[2] in allowed
        ],
    }


def apply(base: dict, mutations: list[list[object]]) -> dict:
    graph = copy.deepcopy(base)
    for op in mutations:
        kind = op[0]
        nodes = node_index(graph)
        if nodes is None:
            raise ValueError("invalid nodes")
        if kind == "add_node":
            if op[1]["id"] in nodes:
                raise ValueError("duplicate node")
            graph["nodes"].append(op[1])
        elif kind == "add_edge":
            graph["edges"].append(op[1])
        elif kind == "reverse_collection":
            collection = op[1]
            if collection not in ("nodes", "edges"):
                raise ValueError("bad collection")
            graph[collection].reverse()
        else:
            raise ValueError(f"unsupported mutation: {kind}")
    return graph


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: verify_claim_projection_scope.py "
            "EXPECTED_POLICY_BLOB_SHA POLICY.json FIXTURES.json",
            file=sys.stderr,
        )
        return 2

    expected_policy = sys.argv[1]
    policy_path = Path(sys.argv[2])
    fixture_path = Path(sys.argv[3])
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))

    actual_policy = git_blob_sha(policy_path)
    if actual_policy != expected_policy:
        print("policy binding mismatch")
        return 1
    if fixture.get("policy_binding", {}).get("git_blob_sha") != actual_policy:
        print("fixture policy binding mismatch")
        return 1

    base = fixture["base_graph"]
    baseline_projection = None
    failures = []

    for case in fixture["cases"]:
        graph = apply(base, case["mutation"])
        projection = project(graph, policy, case["claim"])
        if projection is None:
            failures.append((case["case_id"], "invalid-projection"))
            continue
        current_digest = digest(projection)

        if baseline_projection is None and case["claim"] == "claim-a":
            baseline_projection = current_digest

        if case["expected"] == "qualified":
            if projection is None:
                failures.append((case["case_id"], "expected-qualified"))
        elif case["expected"] == "projection-unchanged":
            if case["claim"] == "claim-a" and current_digest != baseline_projection:
                failures.append((case["case_id"], baseline_projection, current_digest))
        elif case["expected"] == "projection-changed":
            # Locate the immediately preceding baseline for the same claim.
            base_projection = digest(project(base, policy, case["claim"]))
            if current_digest == base_projection:
                failures.append((case["case_id"], "expected-projection-change", current_digest))
        else:
            failures.append((case["case_id"], "unknown expectation", case["expected"]))

    print(f"cases={len(fixture['cases'])} failures={len(failures)}")
    for failure in failures:
        print("FAIL", failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
