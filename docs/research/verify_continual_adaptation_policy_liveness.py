#!/usr/bin/env python3
"""Research-only declarative policy liveness campaign.

Mutates individual policy rules, binds each mutated policy to its exact Git blob
identity, and invokes the current Python and Node ledger verifiers against a
single fixed case. A mutation is successful only when both implementations
return the declared changed verdict.
"""
from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def git_blob_sha(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def mutate(policy: dict, spec: dict) -> dict:
    out = copy.deepcopy(policy)
    operation = spec["operation"]

    if operation == "remove_required_edge":
        out["required_edges"].remove(spec["edge"])
    elif operation == "remove_equality_constraint":
        target = next(
            c for c in out["equality_constraints"]
            if c["left"] == spec["left"] and c["right"] == spec["right"]
        )
        out["equality_constraints"].remove(target)
    elif operation == "remove_fixed_field":
        target = next(
            r for r in out["fixed_fields"]
            if r["node"] == spec["node"] and r["field"] == spec["field"]
        )
        out["fixed_fields"].remove(target)
    elif operation == "set_max_qualifying_results":
        out["result_conflict"]["max_qualifying_results"] = spec["value"]
    elif operation == "set_fixed_field_value":
        target = next(
            r for r in out["fixed_fields"]
            if r["node"] == spec["node"] and r["field"] == spec["field"]
        )
        target["value"] = spec["value"]
    elif operation == "replace_relation_rules":
        out["edge_schema_constraints"] = [
            r for r in out["edge_schema_constraints"]
            if r["relation"] != spec["relation"]
        ]
        out["edge_schema_constraints"].append({
            "relation": spec["relation"],
            "source_types": spec["source_types"],
            "target_types": spec["target_types"],
        })
    else:
        raise ValueError(f"unknown mutation: {operation}")

    return out


def run_verifier(verifier: Path, policy: Path, corpus: Path, expected_sha: str) -> int:
    if verifier.suffix == ".py":
        cmd = [sys.executable, str(verifier), expected_sha, str(policy), str(corpus)]
    else:
        cmd = ["node", str(verifier), expected_sha, str(policy), str(corpus)]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if result.stdout:
        print(result.stdout.rstrip())
    if result.stderr:
        print(result.stderr.rstrip(), file=sys.stderr)
    return result.returncode


def main() -> int:
    if len(sys.argv) != 5:
        print(
            "usage: verify_policy_liveness.py POLICY.json "
            "FIXTURES.json LEDGER_VERIFIER_PY LEDGER_VERIFIER_MJS",
            file=sys.stderr,
        )
        return 2

    policy_path = Path(sys.argv[1])
    fixtures_path = Path(sys.argv[2])
    py_verifier = Path(sys.argv[3])
    node_verifier = Path(sys.argv[4])

    original_policy = json.loads(policy_path.read_text(encoding="utf-8"))
    fixtures = json.loads(fixtures_path.read_text(encoding="utf-8"))

    original_sha = git_blob_sha(policy_path)
    if fixtures.get("policy_binding", {}).get("git_blob_sha") != original_sha:
        print("source policy binding mismatch", file=sys.stderr)
        return 1

    # Load the fixed corpus from the sibling evidence-ledger file.
    fixture_path = fixtures_path.parent / "CONTINUAL_ADAPTATION_EVIDENCE_LEDGER_V1_FIXTURES.json"
    fixed = json.loads(fixture_path.read_text(encoding="utf-8"))
    fixed_cases = {case["case_id"]: case for case in fixed["cases"]}

    failures = []

    with tempfile.TemporaryDirectory(prefix="mycelix-policy-liveness-") as tmp:
        tmp_dir = Path(tmp)

        for spec in fixtures["mutations"]:
            source = fixed_cases[spec["source_case_id"]]
            mutated = mutate(original_policy, spec)

            mutated_path = tmp_dir / f"{spec['case_id']}.policy.json"
            mutated_path.write_text(
                json.dumps(mutated, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            mutated_sha = git_blob_sha(mutated_path)

            corpus = {
                "schema": "mycelix.continual-adaptation.policy-liveness.single-case.v1",
                "status": "research-fixture-only",
                "policy_binding": {
                    "path": str(policy_path),
                    "git_blob_sha": mutated_sha,
                },
                "base_graph": fixed["base_graph"],
                "cases": [{
                    "case_id": spec["case_id"],
                    "mutation": source["mutation"],
                    "expected_verdict": spec["expected_verdict"],
                    "expected_graph_digest_sha256": source["expected_graph_digest_sha256"],
                    "expected_semantic_graph_digest_sha256": spec.get("expected_semantic_graph_digest_sha256", source["expected_semantic_graph_digest_sha256"]),
                    "expected_claim_local_graph_digest_sha256": spec.get("expected_claim_local_graph_digest_sha256", source["expected_claim_local_graph_digest_sha256"]),
                }],
            }
            corpus_path = tmp_dir / f"{spec['case_id']}.corpus.json"
            corpus_path.write_text(
                json.dumps(corpus, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

            py_rc = run_verifier(py_verifier, mutated_path, corpus_path, mutated_sha)
            node_rc = run_verifier(node_verifier, mutated_path, corpus_path, mutated_sha)

            ok = py_rc == 0 and node_rc == 0
            print(
                f"{spec['case_id']}: "
                f"expected={spec['expected_verdict']} "
                f"python_rc={py_rc} node_rc={node_rc} "
                f"policy_blob={mutated_sha}"
            )
            if not ok:
                failures.append(spec["case_id"])

    print(f"mutations={len(fixtures['mutations'])} failures={len(failures)}")
    if failures:
        print("FAILURES", failures)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
