#!/usr/bin/env python3
"""Execute the TLA+/Alloy/reference attestation control matrix."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def alloy_rows(output: str) -> dict[str, str]:
    rows: dict[str, str] = {}
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            row = json.loads(line)
            rows[row["label"]] = row["actual"]
    return rows


def remove_fact(source: str, name: str) -> str:
    marker = f"fact {name} {{"
    start = source.find(marker)
    if start < 0:
        raise RuntimeError("mutation fact not found: " + name)
    brace = source.find("{", start)
    depth = 0
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[:start] + source[i + 1 :]
    raise RuntimeError("unterminated fact: " + name)


def main() -> int:
    p = argparse.ArgumentParser()
    for name in (
        "matrix", "tla", "canonical-cfg", "negative-tla",
        "negative-untrusted-cfg", "negative-subject-cfg", "alloy",
        "runner-class-dir", "alloy-jar", "tla-jar", "reference",
        "runtime", "evidence-dir",
    ):
        p.add_argument("--" + name, type=Path, required=True)
    a = p.parse_args()
    a.evidence_dir.mkdir(parents=True, exist_ok=True)

    matrix = json.loads(a.matrix.read_text(encoding="utf-8"))
    expected_order = ["untrusted-attestation", "subject-mismatch"]
    if matrix.get("schema") != "mycelix.evidence-attestation-subject-control-matrix.v1":
        raise RuntimeError("matrix schema mismatch")
    controls = matrix.get("controls", [])
    if [c["id"] for c in controls] != expected_order:
        raise RuntimeError("control order/identity mismatch")

    expected = {
        "untrusted-attestation": (
            "UntrustedAttestationCannotChangeAuthority",
            "UntrustedGrantBackedAuthorityDeltaWitness",
            "EvidenceAuthenticityCannotMintAuthority",
            "UntrustedAttestationNoAuthorityDelta",
        ),
        "subject-mismatch": (
            "SubjectMismatchCannotChangeAuthority",
            "SubjectMismatchGrantBackedAuthorityDeltaWitness",
            "SubjectSubstitutionCannotMintAuthority",
            "SubjectMismatchNoAuthorityDelta",
        ),
    }
    for c in controls:
        got = (
            c["tla_invariant"], c["alloy_witness"],
            c["alloy_assertion"], c["alloy_mutant_fact"],
        )
        if got != expected[c["id"]]:
            raise RuntimeError(f"matrix semantics mismatch for {c['id']}: {got!r}")

    head = run(["git", "rev-parse", "HEAD"])
    tree = run(["git", "rev-parse", "HEAD^{tree}"])
    if head.returncode or tree.returncode:
        raise RuntimeError("unable to resolve exact head/tree")

    runtime = json.loads(a.runtime.read_text(encoding="utf-8"))
    if runtime.get("schema") != "mycelix.evidence-attestation-subject-formal-runtime.v1":
        raise RuntimeError("runtime schema mismatch")

    receipt = {
        "receipt_schema": "mycelix.evidence-attestation-subject-formal-receipt.v1",
        "result": "ExecutedFail",
        "repository": {"head": head.stdout.strip(), "tree": tree.stdout.strip()},
        "runtime": runtime,
        "input_sha256": {
            str(path): sha256_file(path)
            for path in (
                a.matrix, a.tla, a.canonical_cfg, a.negative_tla,
                a.negative_untrusted_cfg, a.negative_subject_cfg,
                a.alloy, a.reference, a.runtime,
            )
        },
    }

    ref = run(["python3", str(a.reference)])
    (a.evidence_dir / "reference.log").write_text(ref.stdout, encoding="utf-8")
    if ref.returncode:
        raise RuntimeError("reference model failed")
    required_markers = [
        "CANONICAL PASS: trusted, subject-bound evidence may accompany a grant-backed authority transition",
        "ISOLATION PASS: untrusted signer leaves steady-state grant provenance valid",
        "NEGATIVE PASS: untrusted attestation authority delta detected",
        "ISOLATION PASS: subject substitution leaves steady-state grant provenance valid",
        "NEGATIVE PASS: subject-mismatch authority delta detected",
    ]
    for marker in required_markers:
        if marker not in ref.stdout:
            raise RuntimeError("reference marker missing: " + marker)
    receipt["reference"] = {"returncode": 0, "stdout_sha256": sha256_bytes(ref.stdout.encode())}

    def tlc(cfg: Path, label: str) -> set[str]:
        r = run(["java", "-cp", str(a.tla_jar), "tlc2.TLC", "-workers", "1", "-config", str(cfg), str(a.tla)])
        (a.evidence_dir / f"{label}.log").write_text(r.stdout, encoding="utf-8")
        return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.", r.stdout))

    if tlc(a.canonical_cfg, "tla-canonical"):
        raise RuntimeError("canonical TLA violated an invariant")
    vu = tlc(a.negative_untrusted_cfg, "tla-negative-untrusted")
    vs = tlc(a.negative_subject_cfg, "tla-negative-subject")
    if vu != {"UntrustedAttestationCannotChangeAuthority"}:
        raise RuntimeError(f"untrusted TLA isolation mismatch: {sorted(vu)}")
    if vs != {"SubjectMismatchCannotChangeAuthority"}:
        raise RuntimeError(f"subject TLA isolation mismatch: {sorted(vs)}")
    receipt["tla"] = {
        "canonical": "PASS",
        "negative_untrusted": sorted(vu),
        "negative_subject": sorted(vs),
    }

    cp = f"{a.runner_class_dir}:{a.alloy_jar}"
    canonical = run(["java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(a.alloy)])
    (a.evidence_dir / "alloy-canonical.log").write_text(canonical.stdout, encoding="utf-8")
    if canonical.returncode:
        raise RuntimeError("canonical Alloy runner failed")
    canonical_rows = alloy_rows(canonical.stdout)
    expected_canonical = {
        "ValidTrustedBoundedAuthorityDeltaWitness": "SAT",
        "UntrustedGrantBackedAuthorityDeltaWitness": "UNSAT",
        "SubjectMismatchGrantBackedAuthorityDeltaWitness": "UNSAT",
        "EvidenceAuthenticityCannotMintAuthority": "UNSAT",
        "SubjectSubstitutionCannotMintAuthority": "UNSAT",
    }
    if {k: canonical_rows.get(k) for k in expected_canonical} != expected_canonical:
        raise RuntimeError("canonical Alloy outcome mismatch")
    receipt["alloy"] = {"canonical": canonical_rows, "mutations": {}}

    source = a.alloy.read_text(encoding="utf-8")
    for control_id, (_, witness, assertion, fact) in expected.items():
        mutant_path = a.evidence_dir / f"alloy-negative-{control_id}.als"
        mutant_path.write_text(remove_fact(source, fact), encoding="utf-8")
        mutant = run(["java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(mutant_path)])
        (a.evidence_dir / f"alloy-negative-{control_id}.log").write_text(mutant.stdout, encoding="utf-8")
        if mutant.returncode:
            raise RuntimeError(f"Alloy mutant runner failed: {control_id}")
        rows = alloy_rows(mutant.stdout)
        if rows.get(witness) != "SAT" or rows.get(assertion) != "SAT":
            raise RuntimeError(f"target Alloy mutation did not become SAT: {control_id}")
        changed = {
            label for label in set(canonical_rows) | set(rows)
            if canonical_rows.get(label) != rows.get(label)
        }
        if changed != {witness, assertion}:
            raise RuntimeError(f"unrelated Alloy outcomes changed for {control_id}: {sorted(changed)}")
        receipt["alloy"]["mutations"][control_id] = {
            "removed_fact": fact,
            "changed_outcomes": sorted(changed),
            "outcomes": rows,
        }

    receipt["result"] = "ExecutedPass"
    (a.evidence_dir / "evidence-attestation-subject-formal-receipt-v1.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "
", encoding="utf-8"
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
