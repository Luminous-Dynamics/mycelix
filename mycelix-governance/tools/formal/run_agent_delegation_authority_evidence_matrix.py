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


def remove_named_fact(source: str, name: str) -> str:
    marker = f"fact {name} {{"
    start = source.find(marker)
    if start < 0:
        fail("Alloy mutation fact not found: " + name)
    brace = source.find("{", start)
    depth = 0
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[:start] + source[i + 1 :]
    fail("unterminated Alloy fact: " + name)


def alloy_rows(output: str) -> list[dict]:
    return [
        json.loads(line)
        for line in output.splitlines()
        if line.startswith("{") and '"label"' in line and '"actual"' in line
    ]


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
    expected_control = {
        "id": "evidence-transition",
        "boundary": "recording evidence must not change authority",
        "tla_invariant": "EvidenceDoesNotMintAuthority",
        "tla_control": "evidence-mint",
        "alloy_witness": "EvidenceGrantBackedAuthorityDeltaWitness",
        "alloy_mutant_fact": "EvidenceRecordingDoesNotChangeGrants",
        "reference_marker": "ISOLATION PASS: evidence delta leaves steady-state grant provenance valid -> NEGATIVE PASS: evidence-only transition attribution detected",
    }
    if controls != [expected_control]:
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
    canonical = run(["java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(a.alloy)])
    (a.evidence_dir / "alloy-canonical.log").write_text(canonical.stdout, encoding="utf-8")
    if canonical.returncode != 0:
        fail("canonical Alloy runner returned nonzero")
    canonical_rows = alloy_rows(canonical.stdout)
    canonical_by_label = {row["label"]: row for row in canonical_rows}

    required_labels = {
        "EvidenceGrantBackedAuthorityDeltaWitness",
        "EvidenceCannotMintAuthority",
    }
    if not required_labels.issubset(canonical_by_label):
        fail("canonical Alloy evidence commands missing")
    if canonical_by_label["EvidenceGrantBackedAuthorityDeltaWitness"]["actual"] != "UNSAT":
        fail("canonical Alloy evidence-delta witness was satisfiable")
    if canonical_by_label["EvidenceCannotMintAuthority"]["actual"] != "UNSAT":
        fail("canonical Alloy evidence non-mint assertion was satisfiable")

    source = a.alloy.read_text(encoding="utf-8")
    mutant_path = a.evidence_dir / "alloy-negative-evidence.als"
    mutant_path.write_text(
        remove_named_fact(source, expected_control["alloy_mutant_fact"]),
        encoding="utf-8",
    )
    mutant = run(["java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(mutant_path)])
    (a.evidence_dir / "alloy-negative.log").write_text(mutant.stdout, encoding="utf-8")
    if mutant.returncode != 0:
        fail("Alloy negative runner returned nonzero")
    mutant_rows = alloy_rows(mutant.stdout)
    mutant_by_label = {row["label"]: row for row in mutant_rows}
    if set(mutant_by_label) != set(canonical_by_label):
        fail("Alloy negative command set changed")
    if mutant_by_label["EvidenceGrantBackedAuthorityDeltaWitness"]["actual"] != "SAT":
        fail("Alloy negative evidence-delta witness did not become SAT")
    if mutant_by_label["EvidenceCannotMintAuthority"]["actual"] != "SAT":
        fail("Alloy negative evidence non-mint assertion did not become SAT")

    changed = {
        label
        for label in canonical_by_label
        if canonical_by_label[label]["actual"] != mutant_by_label[label]["actual"]
    }
    if changed - required_labels:
        fail("Alloy negative changed unrelated outcomes: " + repr(sorted(changed - required_labels)))

    for path in (
        a.matrix, a.tla, a.cfg, a.negative_tla, a.negative_cfg,
        a.alloy, a.reference, a.alloy_runner_class
    ):
        if not path.exists() or path.stat().st_size == 0:
            fail("missing/empty evidence input: " + str(path))

    receipt = {
        "receipt_schema": "agent-delegation-authority-evidence-formal-receipt-v1",
        "result": "ExecutedPass",
        "control": expected_control,
        "reference": {
            "returncode": ref.returncode,
            "sha256": hashlib.sha256(ref.stdout.encode()).hexdigest(),
        },
        "tla": {
            "canonical": {
                "returncode": tla.returncode,
                "sha256": hashlib.sha256(tla.stdout.encode()).hexdigest(),
            },
            "negative": {
                "returncode": negative.returncode,
                "violated_invariants": sorted(violations),
                "sha256": hashlib.sha256(negative.stdout.encode()).hexdigest(),
            },
        },
        "alloy": {
            "canonical_rows": canonical_rows,
            "negative_rows": mutant_rows,
            "changed_outcomes": sorted(changed),
        },
    }
    (a.evidence_dir / "agent-delegation-authority-evidence-formal-receipt-v1.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
