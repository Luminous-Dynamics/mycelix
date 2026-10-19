#!/usr/bin/env python3
"""Research-only liveness campaign for the selection/censoring policy.

Each mutation changes one policy control and reruns both independent reference
implementations. A mutation is useful only when the expected verdict changes.
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
    target = out
    for key in spec["path"][:-1]:
        target = target[key]
    target[spec["path"][-1]] = spec["value"]
    return out


def main() -> int:
    if len(sys.argv) != 5:
        print("usage: verify_selection_censoring_policy_liveness.py POLICY.json FIXTURES.json VERIFIER.py VERIFIER.mjs", file=sys.stderr)
        return 2

    policy_path, liveness_path, py_verifier, node_verifier = map(Path, sys.argv[1:5])
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    liveness = json.loads(liveness_path.read_text(encoding="utf-8"))

    fixed_path = liveness_path.parent / "CONTINUAL_ADAPTATION_SELECTION_CENSORING_FIXTURES.json"
    fixed = json.loads(fixed_path.read_text(encoding="utf-8"))
    fixed_cases = {case["case_id"]: case for case in fixed["cases"]}

    original_sha = git_blob_sha(policy_path)
    if fixed.get("policy_binding", {}).get("git_blob_sha") != original_sha:
        print("source policy binding mismatch", file=sys.stderr)
        return 1

    failures = []
    with tempfile.TemporaryDirectory(prefix="mycelix-selection-policy-liveness-") as tmp:
        tmp_dir = Path(tmp)
        for spec in liveness["mutations"]:
            mutated_policy = mutate(policy, spec)
            mutated_path = tmp_dir / f"{spec['case_id']}.policy.json"
            mutated_path.write_text(json.dumps(mutated_policy, indent=2) + "\n", encoding="utf-8")
            mutated_sha = git_blob_sha(mutated_path)

            source = fixed_cases[spec["source_case_id"]]
            corpus = {
                "schema": "mycelix.continual-adaptation.selection-censoring-policy-liveness.single-case.v1",
                "status": "research-fixture-only",
                "policy_binding": {"git_blob_sha": mutated_sha},
                "protocol": fixed["protocol"],
                "base_case": fixed["base_case"],
                "cases": [{
                    "case_id": spec["case_id"],
                    "mutation": source["mutation"],
                    "expected_verdict": spec["expected_verdict"],
                }],
            }
            corpus_path = tmp_dir / f"{spec['case_id']}.corpus.json"
            corpus_path.write_text(json.dumps(corpus, indent=2) + "\n", encoding="utf-8")

            py = subprocess.run(
                [sys.executable, str(py_verifier), mutated_sha, str(mutated_path), str(corpus_path), str(tmp_dir / "py.json")],
                capture_output=True, text=True, check=False,
            )
            node = subprocess.run(
                ["node", str(node_verifier), mutated_sha, str(mutated_path), str(corpus_path), str(tmp_dir / "node.json")],
                capture_output=True, text=True, check=False,
            )
            ok = py.returncode == 0 and node.returncode == 0
            print(
                f"{spec['case_id']}: source={spec['source_case_id']} "
                f"expected={spec['expected_verdict']} python_rc={py.returncode} "
                f"node_rc={node.returncode} policy_blob={mutated_sha}"
            )
            if not ok:
                failures.append(spec["case_id"])

    print(f"mutations={len(liveness['mutations'])} failures={len(failures)}")
    if failures:
        print("FAILURES", failures)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
