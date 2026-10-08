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
    "alloy", "runner-class-dir", "alloy-jar", "tla-jar",
    "reference", "runtime", "evidence-dir",
):
    parser.add_argument("--" + name, type=Path, required=True)
a = parser.parse_args()
a.evidence_dir.mkdir(parents=True, exist_ok=True)

matrix = json.loads(a.matrix.read_text(encoding="utf-8"))
if matrix.get("schema") != "mycelix.evidence-attestation-full-dimensional-composition-control-matrix.v1":
    raise RuntimeError("matrix schema mismatch")
controls = matrix.get("controls", [])
if [c["id"] for c in controls] != ["full-dimensional-hybrid"]:
    raise RuntimeError("control ordering mismatch")

control = controls[0]
expected = {
    "tla_invariant": "CompositionAtomAndProvenanceExact",
    "tla_control": "hybrid-synthesis",
    "witness": "HybridSynthesisWitness",
    "assertion": "CompositionAtomAndProvenanceExact",
    "fact": "CompositionAtomicAndProvenance",
}
if (
    control["tla_invariant"],
    control["tla_control"],
    control["alloy_witness"],
    control["alloy_assertion"],
    control["alloy_mutant_fact"],
) != (
    expected["tla_invariant"],
    expected["tla_control"],
    expected["witness"],
    expected["assertion"],
    expected["fact"],
):
    raise RuntimeError("matrix semantics mismatch")

head = run(["git", "rev-parse", "HEAD"])
tree = run(["git", "rev-parse", "HEAD^{tree}"])
if head.returncode or tree.returncode:
    raise RuntimeError("exact head/tree resolution failed")

runtime = json.loads(a.runtime.read_text(encoding="utf-8"))
if runtime.get("schema") != "mycelix.evidence-attestation-capability-formal-runtime.v1":
    raise RuntimeError("runtime schema mismatch")

receipt = {
    "receipt_schema": "mycelix.evidence-attestation-full-dimensional-composition-formal-receipt.v1",
    "result": "ExecutedFail",
    "repository": {"head": head.stdout.strip(), "tree": tree.stdout.strip()},
    "control": control,
    "runtime": runtime,
    "input_sha256": {
        str(p): fsha(p)
        for p in (
            a.matrix, a.tla, a.canonical_cfg, a.negative_tla, a.negative_cfg,
            a.alloy, a.reference, a.runtime,
        )
    },
}

reference = run(["python3", str(a.reference)])
(a.evidence_dir / "reference.log").write_text(reference.stdout, encoding="utf-8")
if reference.returncode:
    raise RuntimeError("reference model returned nonzero")
for marker in control["reference_markers"]:
    if marker not in reference.stdout:
        raise RuntimeError("reference marker missing: " + marker)
receipt["reference"] = {
    "returncode": reference.returncode,
    "stdout_sha256": sha(reference.stdout.encode()),
}

def tlc(cfg, module_path, label):
    result = run([
        "java", "-cp", str(a.tla_jar), "tlc2.TLC",
        "-workers", "1", "-config", str(cfg), str(module_path),
    ])
    (a.evidence_dir / (label + ".log")).write_text(result.stdout, encoding="utf-8")
    if result.returncode == 0 and "Model checking completed. No error has been found." in result.stdout:
        return set()
    return set(re.findall(
        r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.", result.stdout
    ))

canonical = tlc(a.canonical_cfg, a.tla, "tla-canonical")
if canonical:
    raise RuntimeError("canonical TLA violated: " + repr(sorted(canonical)))

negative = tlc(a.negative_cfg, a.negative_tla, "tla-negative-hybrid")
if negative != {"CompositionAtomAndProvenanceExact"}:
    raise RuntimeError(
        "TLA negative isolation mismatch: " + repr(sorted(negative))
    )
receipt["tla"] = {
    "canonical": "PASS",
    "negative": {"hybrid-synthesis": sorted(negative)},
}

cp = f"{a.runner_class_dir}:{a.alloy_jar}"
canonical_run = run([
    "java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(a.alloy)
])
(a.evidence_dir / "alloy-canonical.log").write_text(canonical_run.stdout, encoding="utf-8")
if canonical_run.returncode:
    raise RuntimeError("canonical Alloy runner failed")
rows = alloy_rows(canonical_run.stdout)

required = {
    "ValidFourDimensionalCompositionWitness": "SAT",
    "HybridSynthesisWitness": "UNSAT",
    "CompositionAtomAndProvenanceExact": "UNSAT",
    "DimensionSourcesRemainValid": "UNSAT",
}
if {k: rows.get(k) for k in required} != required:
    raise RuntimeError("canonical Alloy outcomes mismatch: " + repr(rows))
receipt["alloy"] = {"canonical": rows, "mutations": {}}

source = a.alloy.read_text(encoding="utf-8")
mutant_path = a.evidence_dir / "alloy-negative-full-dimensional-hybrid.als"
mutant_path.write_text(
    remove_fact(source, "CompositionAtomicAndProvenance"),
    encoding="utf-8",
)
mutant = run([
    "java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(mutant_path)
])
(a.evidence_dir / "alloy-negative-full-dimensional-hybrid.log").write_text(
    mutant.stdout, encoding="utf-8"
)
if mutant.returncode:
    raise RuntimeError("Alloy mutant failed")
mrows = alloy_rows(mutant.stdout)
if (
    mrows.get("ValidFourDimensionalCompositionWitness") != "SAT"
    or mrows.get("HybridSynthesisWitness") != "SAT"
    or mrows.get("CompositionAtomAndProvenanceExact") != "SAT"
    or mrows.get("DimensionSourcesRemainValid") != "UNSAT"
):
    raise RuntimeError("Alloy differential outcomes mismatch: " + repr(mrows))

changed = {
    label for label in set(rows) | set(mrows)
    if rows.get(label) != mrows.get(label)
}
if changed != {"HybridSynthesisWitness", "CompositionAtomAndProvenanceExact"}:
    raise RuntimeError(
        "unrelated Alloy outcomes changed: " + repr(sorted(changed))
    )

receipt["alloy"]["mutations"]["full-dimensional-hybrid"] = {
    "removed_fact": "CompositionAtomicAndProvenance",
    "changed_outcomes": sorted(changed),
    "outcomes": mrows,
}

receipt["result"] = "ExecutedPass"
(a.evidence_dir / "evidence-attestation-full-dimensional-composition-formal-receipt-v1.json").write_text(
    json.dumps(receipt, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(receipt, sort_keys=True))
