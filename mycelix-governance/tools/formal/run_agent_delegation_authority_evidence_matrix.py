#!/usr/bin/env python3
"""Execute the three-way evidence provenance control matrix."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def fail(msg: str) -> "NoReturn":
    raise RuntimeError("AUTHORITY_EVIDENCE_FORMAL_FAIL: " + msg)


def main() -> int:
    p = argparse.ArgumentParser()
    for name in (
        "matrix", "tla", "cfg", "negative-tla", "negative-cfg",
        "alloy", "runner-class-dir", "tla-jar", "alloy-jar",
        "alloy-runner-class", "reference", "evidence-dir",
    ):
        p.add_argument("--" + name, type=Path, required=True)
    a = p.parse_args()
    a.evidence_dir.mkdir(parents=True, exist_ok=True)

    matrix = json.loads(a.matrix.read_text(encoding="utf-8"))
    if matrix.get("schema") != "mycelix.agent-delegation-authority-evidence-control-matrix.v1":
        fail("matrix schema mismatch")
    controls = matrix.get("controls", [])
    if controls != [{
        "id": "evidence-transition",
        "boundary": "recording evidence must not change authority",
        "tla_invariant": "EvidenceDoesNotMintAuthority",
        "tla_control": "evidence-mint",
        "alloy_witness": "EvidenceGrantBackedAuthorityDeltaWitness",
        "reference_marker": "ISOLATION PASS: evidence delta leaves steady-state grant provenance valid -> NEGATIVE PASS: evidence-only transition attribution detected"
    }]:
        fail("matrix bytes/semantics mismatch")

    ref = run(["python3", str(a.reference)])
    (a.evidence_dir / "reference.log").write_text(ref.stdout, encoding="utf-8")
    if ref.returncode != 0:
        fail("reference explorer returned nonzero")
    for marker in (
        "CANONICAL PASS: evidence recording preserves authority",
        "ISOLATION PASS: evidence delta leaves steady-state grant provenance valid",
        "NEGATIVE PASS: evidence-only transition attribution detected",
    ):
        if marker not in ref.stdout:
            fail("reference marker missing: " + marker)

    tla = run([
        "java", "-cp", str(a.tla_jar), "tlc2.TLC",
        "-workers", "1", "-config", str(a.cfg), str(a.tla),
    ])
    (a.evidence_dir / "tla-canonical.log").write_text(tla.stdout, encoding="utf-8")
    if tla.returncode != 0 or "Model checking completed. No error has been found." not in tla.stdout:
        fail("canonical TLA did not complete cleanly")

    negative = run([
        "java", "-cp", str(a.tla_jar), "tlc2.TLC",
        "-workers", "1", "-config", str(a.negative_cfg), str(a.negative_tla),
    ])
    (a.evidence_dir / "tla-negative.log").write_text(negative.stdout, encoding="utf-8")
    violations = set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.", negative.stdout))
    if violations != {"EvidenceDoesNotMintAuthority"}:
        fail("evidence negative did not isolate exactly one invariant: " + repr(sorted(violations)))

    cp = f"{a.runner_class_dir}:{a.alloy_jar}"
    alloy = run(["java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(a.alloy)])
    (a.evidence_dir / "alloy.log").write_text(alloy.stdout, encoding="utf-8")
    if alloy.returncode != 0:
        fail("Alloy runner returned nonzero")

    rows = [
        json.loads(line) for line in alloy.stdout.splitlines()
        if line.startswith("{") and '"label"' in line and '"actual"' in line
    ]
    by_label = {row["label"]: row for row in rows}
    for label in ("EvidenceGrantBackedAuthorityDeltaWitness", "EvidenceCannotMintAuthority"):
        if label not in by_label:
            fail("missing Alloy command: " + label)
    if by_label["EvidenceGrantBackedAuthorityDeltaWitness"]["actual"] != "SAT":
        fail("grant-backed evidence delta witness is not SAT")
    if by_label["EvidenceCannotMintAuthority"]["actual"] != "UNSAT":
        fail("evidence non-mint assertion is not UNSAT")

    for path in (
        a.matrix, a.tla, a.cfg, a.negative_tla, a.negative_cfg,
        a.alloy, a.reference, a.alloy_runner_class
    ):
        if not path.exists() or path.stat().st_size == 0:
            fail("missing/empty evidence input: " + str(path))

    receipt = {
        "receipt_schema": "agent-delegation-authority-evidence-formal-receipt-v1",
        "result": "ExecutedFail",
        "control": controls[0],
        "reference": {"returncode": ref.returncode, "sha256": hashlib.sha256(ref.stdout.encode()).hexdigest()},
        "tla": {
            "canonical": {"returncode": tla.returncode, "sha256": hashlib.sha256(tla.stdout.encode()).hexdigest()},
            "negative": {"returncode": negative.returncode, "violated_invariants": sorted(violations),
                         "sha256": hashlib.sha256(negative.stdout.encode()).hexdigest()},
        },
        "alloy": {"rows": rows},
    }
    (a.evidence_dir / "agent-delegation-authority-evidence-formal-receipt-v1.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "
", encoding="utf-8"
    )
    receipt["result"] = "ExecutedPass"
    (a.evidence_dir / "agent-delegation-authority-evidence-formal-receipt-v1.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "
", encoding="utf-8"
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
