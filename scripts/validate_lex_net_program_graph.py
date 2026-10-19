#!/usr/bin/env python3
"""Validate the LEX-NET program graph without network access.

A PASS from this script establishes coordination-metadata consistency only. It does
not qualify any LEX-NET semantic theorem, implementation head, legal claim, pilot,
or external effect.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any

PROFILE_ID = "lex-net-program-graph-v1"
AUTHORITY = "CoordinationOnly"
RELATIONS = {
    "semantic_requires",
    "composition_requires",
    "qualification_requires",
    "pilot_requires",
    "historical_predecessor",
    "supersedes",
    "reference_only",
}
STATES = {
    "ResearchSpec",
    "CandidateAuthored",
    "QualificationQueued",
    "QualificationInProgress",
    "QualifiedExactHead",
    "ExecutedFail",
    "NonExecuted",
    "Superseded",
    "DiagnosticOnly",
    "IntegratedQualified",
    "PilotEligible",
}
HARD_RELATIONS = {"semantic_requires", "qualification_requires"}
RECEIPT_POLICIES = {"Required", "NotRequired"}


class ValidationError(ValueError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def nonempty(value: Any, field: str) -> str:
    require(isinstance(value, str) and bool(value.strip()), f"{field} must be a non-empty string")
    return value


def exact_head(entry: dict[str, Any], key: str, field: str) -> str:
    head = nonempty(entry.get(key), field)
    require(
        len(head) == 40 and all(c in "0123456789abcdef" for c in head),
        f"{field} must be lowercase 40-hex",
    )
    return head


def positive_int(entry: dict[str, Any], key: str, field: str) -> int:
    value = entry.get(key)
    require(isinstance(value, int) and value > 0, f"{field} must be positive")
    return value


def validate_execution_identity(entry: dict[str, Any], prefix: str) -> str:
    exact_head(entry, "head_sha", f"{prefix}.head_sha")
    nonempty(entry.get("workflow_profile"), f"{prefix}.workflow_profile")
    positive_int(entry, "run_id", f"{prefix}.run_id")
    positive_int(entry, "run_attempt", f"{prefix}.run_attempt")
    positive_int(entry, "job_id", f"{prefix}.job_id")
    receipt_policy = entry.get("receipt_policy")
    require(
        receipt_policy in RECEIPT_POLICIES,
        f"{prefix}.receipt_policy must be Required or NotRequired",
    )
    return receipt_policy


def validate_status(entry: Any, nodes: set[str], index: int) -> None:
    prefix = f"status_entries[{index}]"
    require(isinstance(entry, dict), f"{prefix} must be an object")
    theorem = nonempty(entry.get("theorem"), f"{prefix}.theorem")
    require(theorem in nodes, f"{prefix} references unknown theorem {theorem!r}")
    state = nonempty(entry.get("state"), f"{prefix}.state")
    require(state in STATES, f"{prefix} has unknown state {state!r}")
    nonempty(entry.get("observed_at"), f"{prefix}.observed_at")

    if state == "QualifiedExactHead":
        receipt_policy = validate_execution_identity(entry, prefix)
        require(
            entry.get("conclusion") == "success",
            f"{prefix} QualifiedExactHead requires conclusion=success",
        )
        require(
            entry.get("postflight_immutable") is True,
            f"{prefix} QualifiedExactHead requires postflight_immutable=true",
        )
        if receipt_policy == "Required":
            nonempty(entry.get("receipt_id"), f"{prefix}.receipt_id")

    if state == "QualificationQueued":
        validate_execution_identity(entry, prefix)
        require(
            entry.get("run_status") == "queued",
            f"{prefix} QualificationQueued requires run_status=queued",
        )
        require(
            entry.get("conclusion") is None,
            f"{prefix} QualificationQueued requires conclusion=null",
        )


def reject_hard_cycles(nodes: set[str], relations: list[list[str]]) -> None:
    graph = {node: [] for node in nodes}
    for source, relation, target in relations:
        if relation in HARD_RELATIONS:
            graph[source].append(target)

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str, stack: list[str]) -> None:
        if node in visited:
            return
        if node in visiting:
            start = stack.index(node) if node in stack else 0
            cycle = stack[start:] + [node]
            raise ValidationError("hard dependency cycle: " + " -> ".join(cycle))
        visiting.add(node)
        stack.append(node)
        for target in graph[node]:
            visit(target, stack)
        stack.pop()
        visiting.remove(node)
        visited.add(node)

    for node in sorted(nodes):
        visit(node, [])


def validate_pilot_edges(
    pilots: list[dict[str, Any]],
    normalized_relations: list[list[str]],
    nodes: set[str],
) -> int:
    """Require pilot profile prerequisites and typed pilot edges to be identical."""
    declared_by_theorem: dict[str, set[str]] = {}

    for index, pilot in enumerate(pilots):
        prefix = f"pilot_profiles[{index}]"
        require(isinstance(pilot, dict), f"{prefix} must be an object")
        nonempty(pilot.get("id"), f"{prefix}.id")
        theorem = nonempty(pilot.get("theorem"), f"{prefix}.theorem")
        require(theorem in nodes, f"{prefix} references unknown theorem {theorem!r}")
        require(
            theorem not in declared_by_theorem,
            f"multiple pilot profiles declare theorem {theorem!r}",
        )
        required = pilot.get("requires")
        require(
            isinstance(required, list) and required,
            f"{prefix}.requires must be non-empty",
        )
        require(
            len(required) == len(set(required)),
            f"{prefix}.requires contains duplicates",
        )
        for dependency in required:
            require(dependency in nodes, f"{prefix} unknown prerequisite {dependency!r}")
            require(dependency != theorem, f"{prefix} theorem cannot require itself")
        require(
            pilot.get("effect_policy") == "EffectNotAttempted",
            f"{prefix} v0 must remain effects-disabled",
        )
        declared_by_theorem[theorem] = set(required)

    edge_by_theorem: dict[str, set[str]] = {}
    edge_count = 0
    for source, relation, target in normalized_relations:
        if relation != "pilot_requires":
            continue
        edge_count += 1
        require(
            source in declared_by_theorem,
            f"pilot_requires source {source!r} is not a declared pilot theorem",
        )
        edge_by_theorem.setdefault(source, set()).add(target)

    require(
        set(edge_by_theorem) == set(declared_by_theorem),
        "pilot_requires sources must exactly match declared pilot theorems",
    )

    for theorem, declared in declared_by_theorem.items():
        graph_edges = edge_by_theorem.get(theorem, set())
        require(
            graph_edges == declared,
            f"{theorem} pilot_requires edges differ from pilot profile requires: "
            f"missing_edges={sorted(declared - graph_edges)} "
            f"extra_edges={sorted(graph_edges - declared)}",
        )

    expected_edge_count = sum(len(required) for required in declared_by_theorem.values())
    require(
        edge_count == expected_edge_count,
        "pilot_requires edge census differs from pilot prerequisite census",
    )
    return edge_count


def validate(data: Any) -> dict[str, Any]:
    require(isinstance(data, dict), "manifest root must be an object")
    require(data.get("profile_id") == PROFILE_ID, "unexpected profile_id")
    require(data.get("profile_version") == 1, "profile_version must be 1")
    require(data.get("meta_issue") == 1370, "meta_issue must be 1370")
    require(data.get("authority") == AUTHORITY, "authority must be CoordinationOnly")

    relation_types = data.get("relation_types")
    require(
        isinstance(relation_types, list) and set(relation_types) == RELATIONS,
        "relation_types must match the closed v1 vocabulary",
    )
    require(len(relation_types) == len(set(relation_types)), "relation_types contains duplicates")

    states = data.get("qualification_states")
    require(
        isinstance(states, list) and set(states) == STATES,
        "qualification_states must match the closed v1 vocabulary",
    )
    require(len(states) == len(set(states)), "qualification_states contains duplicates")

    hard = data.get("hard_cycle_relation_types")
    require(
        isinstance(hard, list) and set(hard) == HARD_RELATIONS,
        "hard_cycle_relation_types must match the v1 hard relation set",
    )

    node_map = data.get("nodes")
    require(isinstance(node_map, dict) and node_map, "nodes must be a non-empty object")
    nodes = set(node_map)
    issues: set[int] = set()
    for node, issue in node_map.items():
        nonempty(node, "node id")
        require(isinstance(issue, int) and issue > 0, f"{node} issue must be positive")
        require(issue not in issues, f"duplicate issue mapping #{issue}")
        issues.add(issue)

    layers = data.get("layers")
    require(isinstance(layers, dict) and layers, "layers must be a non-empty object")
    layer_members: list[str] = []
    for layer, members in layers.items():
        nonempty(layer, "layer name")
        require(isinstance(members, list) and members, f"layer {layer!r} must be non-empty")
        require(len(members) == len(set(members)), f"layer {layer!r} contains duplicates")
        for member in members:
            require(member in nodes, f"layer {layer!r} references unknown node {member!r}")
        layer_members.extend(members)
    require(set(layer_members) == nodes, "layers must cover every node exactly")
    require(len(layer_members) == len(nodes), "a node appears in more than one layer")

    relations = data.get("relations")
    require(isinstance(relations, list), "relations must be a list")
    seen: set[tuple[str, str, str]] = set()
    normalized: list[list[str]] = []
    for i, edge in enumerate(relations):
        require(isinstance(edge, list) and len(edge) == 3, f"relations[{i}] must be [from,type,to]")
        source, relation, target = edge
        require(source in nodes, f"relations[{i}] has unknown source {source!r}")
        require(target in nodes, f"relations[{i}] has unknown target {target!r}")
        require(source != target, f"relations[{i}] is a self-dependency")
        require(relation in RELATIONS, f"relations[{i}] has unknown type {relation!r}")
        key = (source, relation, target)
        require(key not in seen, f"duplicate relation {key!r}")
        seen.add(key)
        normalized.append([source, relation, target])
    reject_hard_cycles(nodes, normalized)

    pilots = data.get("pilot_profiles")
    require(isinstance(pilots, list), "pilot_profiles must be a list")
    pilot_ids = [p.get("id") for p in pilots if isinstance(p, dict)]
    require(len(pilot_ids) == len(set(pilot_ids)), "pilot profile ids must be unique")
    pilot_edge_count = validate_pilot_edges(pilots, normalized, nodes)

    status_entries = data.get("status_entries")
    require(isinstance(status_entries, list), "status_entries must be a list")
    for i, entry in enumerate(status_entries):
        validate_status(entry, nodes, i)

    policy = data.get("status_registry_policy")
    require(isinstance(policy, dict), "status_registry_policy must be an object")
    require(policy.get("issue_state_is_qualification") is False, "issue state must not be qualification")
    require(
        policy.get("branch_or_pr_is_proof_identity") is False,
        "branch/PR must not be proof identity",
    )
    require(policy.get("queued_is_pass") is False, "queued must never be PASS")

    nonclaims = data.get("nonclaims")
    require(isinstance(nonclaims, list) and len(nonclaims) >= 4, "at least four explicit nonclaims are required")
    joined = "\n".join(str(item).lower() for item in nonclaims)
    for phrase in ("coordination evidence only", "does not qualify", "do not grant authority"):
        require(phrase in joined, f"nonclaims must include phrase {phrase!r}")

    return {
        "result": "PASS",
        "profile_id": PROFILE_ID,
        "nodes": len(nodes),
        "relations": len(relations),
        "pilot_profiles": len(pilots),
        "pilot_requires_edges": pilot_edge_count,
        "status_entries": len(status_entries),
        "authority": AUTHORITY,
        "grants_semantic_authority": False,
        "grants_legal_authority": False,
        "grants_external_effect_authority": False,
    }


def expect_reject(data: dict[str, Any], label: str) -> None:
    try:
        validate(data)
    except ValidationError:
        return
    raise ValidationError(f"self-test expected rejection: {label}")


def self_test(data: dict[str, Any]) -> None:
    validate(data)

    bad = copy.deepcopy(data)
    bad["relations"].append(["LEX-NET-001", "qualification_requires", "LEX-NET-025"])
    bad["relations"].append(["LEX-NET-025", "qualification_requires", "LEX-NET-001"])
    expect_reject(bad, "hard-cycle")

    bad = copy.deepcopy(data)
    bad["status_entries"] = [{
        "theorem": "LEX-NET-018",
        "state": "QualifiedExactHead",
        "observed_at": "2026-09-16T20:00:00Z",
        "head_sha": "0" * 40,
        "workflow_profile": "lex-net-018-r3-v1",
        "run_id": 1,
        "run_attempt": 1,
        "job_id": 1,
        "receipt_policy": "NotRequired",
        "conclusion": None,
        "postflight_immutable": False,
    }]
    expect_reject(bad, "false QualifiedExactHead")

    bad = copy.deepcopy(data)
    edge = next(edge for edge in bad["relations"] if edge == ["LEX-NET-005A", "pilot_requires", "LEX-NET-005"])
    bad["relations"].remove(edge)
    expect_reject(bad, "pilot prerequisite missing graph edge")

    bad = copy.deepcopy(data)
    bad["relations"].append(["LEX-NET-005A", "pilot_requires", "LEX-NET-002"])
    expect_reject(bad, "extra pilot edge")

    bad = copy.deepcopy(data)
    bad["relations"].append(["LEX-NET-022", "pilot_requires", "LEX-NET-001"])
    expect_reject(bad, "non-pilot theorem owns pilot edge")

    bad = copy.deepcopy(data)
    duplicate = copy.deepcopy(bad["pilot_profiles"][0])
    duplicate["id"] = "duplicate-pilot-record"
    bad["pilot_profiles"].append(duplicate)
    expect_reject(bad, "duplicate pilot theorem")

    queued = copy.deepcopy(data)
    queued["status_entries"] = [{
        "theorem": "LEX-NET-032",
        "state": "QualificationQueued",
        "observed_at": "2026-09-17T07:54:00Z",
        "head_sha": "1" * 40,
        "workflow_profile": "lex-net-032-v1",
        "run_id": 1,
        "run_attempt": 1,
        "job_id": 1,
        "receipt_policy": "NotRequired",
        "run_status": "queued",
        "conclusion": None,
    }]
    validate(queued)

    qualified = copy.deepcopy(data)
    qualified["status_entries"] = [{
        "theorem": "LEX-NET-018",
        "state": "QualifiedExactHead",
        "observed_at": "2026-09-17T07:54:00Z",
        "head_sha": "2" * 40,
        "workflow_profile": "lex-net-018-r3-v1",
        "run_id": 2,
        "run_attempt": 1,
        "job_id": 2,
        "receipt_policy": "NotRequired",
        "conclusion": "success",
        "postflight_immutable": True,
    }]
    validate(qualified)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", nargs="?", default="docs/lex-net/lex_net_program_graph_v1.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    data = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    summary = validate(data)
    if args.self_test:
        self_test(data)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
