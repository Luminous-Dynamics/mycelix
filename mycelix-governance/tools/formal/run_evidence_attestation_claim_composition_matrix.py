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
if matrix.get("schema") != "mycelix.evidence-attestation-claim-composition-control-matrix.v1":
    raise RuntimeError("matrix schema mismatch")
if control["id"] != "composition-amplification":
    raise RuntimeError("control id mismatch")

expected = {
    "tla_invariant": "CompositionOnlyUsesAtomicAuthorizedCapabilities",
    "tla_control": "composition-amplification",
    "alloy_witness": "CartesianAmplificationWitness",
    "alloy_assertion": "CompositionContainsOnlyAuthorizedAtoms",
    "alloy_mutant_fact": "CompositionOnlyUsesAtomicAuthorizedCapabilities",
}
for key, value in expected.items():
    if control.get(
        "tla_invariant" if key == "tla_invariant"
        else "tla_control" if key == "tla_control"
        else "alloy_witness" if key == "alloy_witness"
        else "alloy_assertion" if key == "alloy_assertion"
        else "alloy_mutant_fact"
    ) != value:
        raise RuntimeError("matrix semantic mismatch: " + key)

head = run(["git", "rev-parse", "HEAD"])
tree = run(["git", "rev-parse", "HEAD^{tree}"])
if head.returncode or tree.returncode:
    raise RuntimeError("exact head/tree resolution failed")

runtime = json.loads(a.runtime.read_text(encoding="utf-8"))
if runtime.get("schema") != "mycelix.evidence-attestation-composition-formal-runtime.v1":
    raise RuntimeError("runtime schema mismatch")

receipt = {
    "receipt_schema": "mycelix.evidence-attestation-composition-formal-receipt.v1",
    "result": "ExecutedFail",
    "repository": {
        "head": head.stdout.strip(),
        "tree": tree.stdout.strip(),
    },
    "runtime": runtime,
    "input_sha256": {
        str(path): fsha(path)
        for path in (
            a.matrix,
            a.tla,
            a.canonical_cfg,
            a.negative_tla,
            a.negative_cfg,
            a.alloy,
            a.reference,
            a.runtime,
        )
    },
}

reference = run(["python3", str(a.reference)])
(a.evidence_dir / "reference.log").write_text(reference.stdout, encoding="utf-8")
if reference.returncode:
    raise RuntimeError("reference execution failed")
for marker in control["reference_markers"]:
    if marker not in reference.stdout:
        raise RuntimeError("reference marker missing: " + marker)
receipt["reference"] = {
    "returncode": reference.returncode,
    "stdout_sha256": sha(reference.stdout.encode()),
}

def tlc(cfg, label):
    result = run([
        "java",
        "-cp",
        str(a.tla_jar),
        "tlc2.TLC",
        "-workers",
        "1",
        "-config",
        str(cfg),
        str(a.tla),
    ])
    (a.evidence_dir / f"{label}.log").write_text(result.stdout, encoding="utf-8")
    if result.returncode == 0 and "Model checking completed. No error has been found." in result.stdout:
        return set()
    return set(
        re.findall(
            r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.",
            result.stdout,
        )
    )

canonical_violations = tlc(a.canonical_cfg, "tla-canonical")
if canonical_violations:
    raise RuntimeError("canonical TLA violated: " + repr(sorted(canonical_violations)))

negative_violations = tlc(a.negative_cfg, "tla-negative")
if negative_violations != {expected["tla_invariant"]}:
    raise RuntimeError(
        "composition TLA negative not isolated: " + repr(sorted(negative_violations))
    )

receipt["tla"] = {
    "canonical": "PASS",
    "negative": sorted(negative_violations),
}

cp = f"{a.runner_class_dir}:{a.alloy_jar}"
canonical = run([
    "java",
    "-cp",
    cp,
    "AgentDelegationAuthorityAlloyRunner",
    str(a.alloy),
])
(a.evidence_dir / "alloy-canonical.log").write_text(
    canonical.stdout, encoding="utf-8"
)
if canonical.returncode:
    raise RuntimeError("canonical Alloy runner failed")

canonical_rows = alloy_rows(canonical.stdout)
expected_canonical = {
    "AtomicCompositionWitness": "SAT",
    "CartesianAmplificationWitness": "UNSAT",
    "CompositionContainsOnlyAuthorizedAtoms": "UNSAT",
}
if {key: canonical_rows.get(key) for key in expected_canonical} != expected_canonical:
    raise RuntimeError(
        "canonical Alloy mismatch: " + repr(canonical_rows)
    )

source = a.alloy.read_text(encoding="utf-8")
mutant_path = a.evidence_dir / "alloy-negative-composition-amplification.als"
mutant_path.write_text(
    remove_fact(source, expected["alloy_mutant_fact"]),
    encoding="utf-8",
)

mutant = run([
    "java",
    "-cp",
    cp,
    "AgentDelegationAuthorityAlloyRunner",
    str(mutant_path),
])
(a.evidence_dir / "alloy-negative.log").write_text(
    mutant.stdout, encoding="utf-8"
)
if mutant.returncode:
    raise RuntimeError("Alloy negative runner failed")

mutant_rows = alloy_rows(mutant.stdout)
if mutant_rows.get("CartesianAmplificationWitness") != "SAT":
    raise RuntimeError("Cartesian witness did not become SAT")
if mutant_rows.get("CompositionContainsOnlyAuthorizedAtoms") != "SAT":
    raise RuntimeError("composition assertion did not become SAT")

changed = {
    label
    for label in set(canonical_rows) | set(mutant_rows)
    if canonical_rows.get(label) != mutant_rows.get(label)
}
allowed = {
    "CartesianAmplificationWitness",
    "CompositionContainsOnlyAuthorizedAtoms",
}
if changed != allowed:
    raise RuntimeError(
        "unrelated Alloy outcomes changed: " + repr(sorted(changed))
    )

receipt["alloy"] = {
    "canonical": canonical_rows,
    "negative": mutant_rows,
    "changed_outcomes": sorted(changed),
    "removed_fact": expected["alloy_mutant_fact"],
}
receipt["result"] = "ExecutedPass"
(a.evidence_dir / "evidence-attestation-composition-formal-receipt-v1.json").write_text(
    json.dumps(receipt, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(receipt, sort_keys=True))
