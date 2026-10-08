#!/usr/bin/env python3
"""Research-only policy liveness campaign for classification provenance."""
from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

FIELDS = (
    "attempt_id",
    "censoring_reason",
    "classification_epoch",
    "frozen_epoch",
    "policy_blob_sha",
    "basis_id",
    "revision",
    "claim_scope_anchor",
)


def canonical(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()


def blob_sha(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def commitment(node: dict) -> str:
    return digest({field: node[field] for field in FIELDS})


def set_path(document: dict, path: list[str], value: object) -> None:
    target = document
    for part in path[:-1]:
        target = target[part]
    target[path[-1]] = value


def apply_mutations(graph: dict, mutations: list[list[object]]) -> dict:
    out = copy.deepcopy(graph)
    for op in mutations:
        if op[0] == "set_node":
            node = next(n for n in out["nodes"] if n["id"] == op[1])
            node[op[2]] = op[3]
        elif op[0] == "add_node":
            out["nodes"].append(copy.deepcopy(op[1]))
        elif op[0] == "add_edge":
            out["edges"].append(copy.deepcopy(op[1]))
        elif op[0] == "remove_edge":
            out["edges"].remove(op[1])
        elif op[0] == "reverse_collection":
            out[op[1]].reverse()
        else:
            raise ValueError(f"unknown mutation: {op[0]}")
    return out


def rebind_graph(graph: dict, policy_sha: str) -> dict:
    out = copy.deepcopy(graph)
    for node in out["nodes"]:
        if node.get("type") == "ClassificationPolicy":
            node["policy_blob_sha"] = policy_sha
        if node.get("type") == "CensoringClassification":
            node["policy_blob_sha"] = policy_sha
            node["commitment"] = commitment(node)
    return out


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: verify_censoring_classification_policy_liveness.py "
            "POLICY.json LIVENESS.json VERIFIER.py VERIFIER.mjs",
            file=sys.stderr,
        )
        return 2

    policy_path, liveness_path, py_verifier, node_verifier = map(Path, sys.argv[1:5])
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    liveness = json.loads(liveness_path.read_text(encoding="utf-8"))

    fixture_path = liveness_path.parent / "CONTINUAL_ADAPTATION_CENSORING_CLASSIFICATION_FIXTURES.json"
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    fixed_cases = {case["case_id"]: case for case in fixture["cases"]}

    original_sha = blob_sha(policy_path)
    if fixture["policy_binding"]["git_blob_sha"] != original_sha:
        print("fixture policy binding mismatch", file=sys.stderr)
        return 1

    failures = []
    with tempfile.TemporaryDirectory(prefix="mycelix-cpvp-liveness-") as tmp:
        root = Path(tmp)
        for spec in liveness["mutations"]:
            mutated_policy = copy.deepcopy(policy)
            set_path(mutated_policy, spec["policy_path"], spec["value"])
            mutated_policy_path = root / f"{spec['case_id']}.policy.json"
            mutated_policy_path.write_text(
                json.dumps(mutated_policy, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            mutated_sha = blob_sha(mutated_policy_path)

            source = fixed_cases[spec["source_case_id"]]
            graph = apply_mutations(fixture["base_graph"], source["mutation"])
            graph = apply_mutations(graph, spec.get("extra_mutations", []))
            graph = rebind_graph(graph, mutated_sha)
            anchors = dict(fixture["history_anchors"])
            if not spec.get("preserve_history_anchor", False):
                for node in graph["nodes"]:
                    if node.get("type") == "CensoringClassification" and node.get("revision") == "0":
                        if node["id"] in anchors:
                            anchors[node["id"]] = commitment(node)

            corpus = {
                "schema": "mycelix.continual-adaptation.censoring-classification-policy-liveness.single-case.v2",
                "status": "research-fixture-only",
                "policy_binding": {"git_blob_sha": mutated_sha},
                "history_anchors": anchors,
                "base_graph": graph,
                "cases": [{
                    "case_id": spec["case_id"],
                    "mutation": [],
                    "expected_verdict": spec["expected_verdict"],
                }],
            }
            corpus_path = root / f"{spec['case_id']}.fixture.json"
            corpus_path.write_text(
                json.dumps(corpus, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            py_report = root / f"{spec['case_id']}.py.json"
            node_report = root / f"{spec['case_id']}.node.json"
            py = subprocess.run(
                [sys.executable, str(py_verifier), mutated_sha, str(mutated_policy_path),
                 str(corpus_path), str(py_report)],
                capture_output=True, text=True, check=False,
            )
            node = subprocess.run(
                ["node", str(node_verifier), mutated_sha, str(mutated_policy_path),
                 str(corpus_path), str(node_report)],
                capture_output=True, text=True, check=False,
            )

            py_ok = py.returncode == 0
            node_ok = node.returncode == 0
            if not (py_ok and node_ok):
                failures.append(spec["case_id"])

            print(
                f"{spec['case_id']}: expected={spec['expected_verdict']} "
                f"python_rc={py.returncode} node_rc={node.returncode} "
                f"policy_blob={mutated_sha}"
            )

    print(f"mutations={len(liveness['mutations'])} failures={len(failures)}")
    if failures:
        print("FAILURES", failures)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
