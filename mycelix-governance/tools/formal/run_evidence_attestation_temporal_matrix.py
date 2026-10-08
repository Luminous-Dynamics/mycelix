#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def run(cmd):
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def fsha(path):
    return sha(path.read_bytes())


def alloy_rows(output):
    rows = {}
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            obj = json.loads(line)
            rows[obj["label"]] = obj["actual"]
    return rows


def remove_fact(source, name):
    marker = "fact " + name + " {"
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
                return source[:start] + source[i + 1:]
    raise RuntimeError("unterminated fact: " + name)


parser = argparse.ArgumentParser()
for name in (
    "matrix", "tla", "canonical-cfg", "negative-tla", "negative-cfg",
    "alloy", "runner-class-dir", "alloy-jar", "tla-jar", "reference",
    "runtime", "evidence-dir",
):
    parser.add_argument("--" + name, type=Path, required=True)
a = parser.parse_args()
a.evidence_dir.mkdir(parents=True, exist_ok=True)

matrix = json.loads(a.matrix.read_text(encoding="utf-8"))
control = matrix["control"]
if matrix.get("schema") != "mycelix.evidence-attestation-temporal-expiration-control-matrix.v1":
    raise RuntimeError("matrix schema mismatch")
if control["id"] != "expiry-persistence":
    raise RuntimeError("control id mismatch")
if control["alloy_mutant_fact"] != "ExpiredGrantNotEffective":
    raise RuntimeError("unexpected temporal mutation target")

head = run(["git", "rev-parse", "HEAD"])
tree = run(["git", "rev-parse", "HEAD^{tree}"])
if head.returncode or tree.returncode:
    raise RuntimeError("exact head/tree resolution failed")

runtime = json.loads(a.runtime.read_text(encoding="utf-8"))
if runtime.get("schema") != "mycelix.evidence-attestation-temporal-formal-runtime.v1":
    raise RuntimeError("runtime schema mismatch")

receipt = {
    "receipt_schema": "mycelix.evidence-attestation-temporal-formal-receipt.v1",
    "result": "ExecutedFail",
    "repository": {"head": head.stdout.strip(), "tree": tree.stdout.strip()},
    "runtime": runtime,
    "input_sha256": {
        str(p): fsha(p)
        for p in (
            a.matrix, a.tla, a.canonical_cfg, a.negative_tla,
            a.negative_cfg, a.alloy, a.reference, a.runtime
        )
    },
}

reference = run(["python3", str(a.reference)])
(a.evidence_dir / "reference.log").write_text(reference.stdout, encoding="utf-8")
if reference.returncode:
    raise RuntimeError("reference execution failed")
for marker in (
    "CANONICAL PASS: fresh grant remains effective before expiry",
    "ISOLATION PASS: expired grant remains structurally grant-backed",
    "NEGATIVE PASS: expired authority persistence detected",
):
    if marker not in reference.stdout:
        raise RuntimeError("reference marker missing: " + marker)
receipt["reference"] = {
    "returncode": reference.returncode,
    "stdout_sha256": sha(reference.stdout.encode()),
}

def tlc(cfg, module_path, label):
    r = run([
        "java", "-cp", str(a.tla_jar),
        "tlc2.TLC", "-workers", "1",
        "-config", str(cfg), str(module_path),
    ])
    (a.evidence_dir / (label + ".log")).write_text(r.stdout, encoding="utf-8")
    if r.returncode == 0 and "Model checking completed. No error has been found." in r.stdout:
        return set()
    violations = set(re.findall(
        r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",
        r.stdout,
    ))
    if violations:
        return violations
    raise RuntimeError(
        "TLC execution failed without a classified invariant violation: " +
        " | ".join(r.stdout.splitlines()[-8:])
    )

canonical_violations = tlc(a.canonical_cfg, a.tla, "tla-canonical")
if canonical_violations:
    raise RuntimeError("canonical TLA failed: " + repr(sorted(canonical_violations)))

negative_violations = tlc(a.negative_cfg, a.negative_tla, "tla-negative")
if negative_violations != {"ExpiredGrantNotEffective"}:
    raise RuntimeError(
        "temporal TLA negative did not isolate target: " +
        repr(sorted(negative_violations))
    )

receipt["tla"] = {
    "canonical": "PASS",
    "negative": sorted(negative_violations),
}

cp = f"{a.runner_class_dir}:{a.alloy_jar}"
canonical = run([
    "java", "-cp", cp,
    "AgentDelegationAuthorityAlloyRunner", str(a.alloy),
])
(a.evidence_dir / "alloy-canonical.log").write_text(
    canonical.stdout, encoding="utf-8"
)
if canonical.returncode:
    raise RuntimeError("canonical Alloy runner failed")
canonical_rows = alloy_rows(canonical.stdout)
expected = {
    "FreshGrantEffectiveWitness": "SAT",
    "ExpiredGrantPersistenceWitness": "UNSAT",
    "EffectiveAuthorityMatchesCurrentTime": "UNSAT",
}
if {k: canonical_rows.get(k) for k in expected} != expected:
    raise RuntimeError("canonical Alloy mismatch: " + repr(canonical_rows))

mutant_path = a.evidence_dir / "alloy-negative-expiry-persistence.als"
mutant_path.write_text(
    remove_fact(a.alloy.read_text(encoding="utf-8"), "ExpiredGrantNotEffective"),
    encoding="utf-8",
)
mutant = run([
    "java", "-cp", cp,
    "AgentDelegationAuthorityAlloyRunner", str(mutant_path),
])
(a.evidence_dir / "alloy-negative.log").write_text(
    mutant.stdout, encoding="utf-8"
)
if mutant.returncode:
    raise RuntimeError("Alloy negative runner failed")
mutant_rows = alloy_rows(mutant.stdout)
if mutant_rows.get("ExpiredGrantPersistenceWitness") != "SAT":
    raise RuntimeError("expired persistence witness did not become SAT")
if mutant_rows.get("EffectiveAuthorityMatchesCurrentTime") != "SAT":
    raise RuntimeError("temporal consistency assertion did not become SAT")
changed = {
    label
    for label in set(canonical_rows) | set(mutant_rows)
    if canonical_rows.get(label) != mutant_rows.get(label)
}
allowed = {
    "ExpiredGrantPersistenceWitness",
    "EffectiveAuthorityMatchesCurrentTime",
}
if changed != allowed:
    raise RuntimeError(
        "unrelated Alloy outcomes changed: " + repr(sorted(changed))
    )

receipt["alloy"] = {
    "canonical": canonical_rows,
    "negative": mutant_rows,
    "changed_outcomes": sorted(changed),
    "removed_fact": "ExpiredGrantNotEffective",
}
receipt["result"] = "ExecutedPass"
(a.evidence_dir / "evidence-attestation-temporal-formal-receipt-v1.json").write_text(
    json.dumps(receipt, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(receipt, sort_keys=True))
