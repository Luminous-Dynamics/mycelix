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
    "matrix", "tla", "canonical-cfg", "negative-tla",
    "negative-resource-cfg", "negative-action-cfg",
    "negative-audience-cfg", "negative-expiry-cfg",
    "alloy", "runner-class-dir", "alloy-jar", "tla-jar",
    "reference", "runtime", "evidence-dir",
):
    parser.add_argument("--" + name, type=Path, required=True)
a = parser.parse_args()
a.evidence_dir.mkdir(parents=True, exist_ok=True)

matrix = json.loads(a.matrix.read_text(encoding="utf-8"))
if matrix.get("schema") != "mycelix.evidence-attestation-capability-attenuation-control-matrix.v1":
    raise RuntimeError("matrix schema mismatch")
controls = matrix.get("controls", [])
if [c["id"] for c in controls] != [
    "resource-expansion", "action-expansion",
    "audience-expansion", "expiry-expansion",
]:
    raise RuntimeError("control ordering mismatch")

expected = {
    "resource-expansion": {
        "tla_invariant": "ResourceScopeAttenuated",
        "tla_control": "resource-expansion",
        "witness": "ResourceExpansionWitness",
        "assertion": "ResourceScopeNeverExceedsClaim",
        "fact": "ClaimResourceBounded",
    },
    "action-expansion": {
        "tla_invariant": "ActionScopeAttenuated",
        "tla_control": "action-expansion",
        "witness": "ActionExpansionWitness",
        "assertion": "ActionScopeNeverExceedsClaim",
        "fact": "ClaimActionBounded",
    },
    "audience-expansion": {
        "tla_invariant": "AudienceScopeAttenuated",
        "tla_control": "audience-expansion",
        "witness": "AudienceExpansionWitness",
        "assertion": "AudienceScopeNeverExceedsClaim",
        "fact": "ClaimAudienceBounded",
    },
    "expiry-expansion": {
        "tla_invariant": "ExpiryScopeAttenuated",
        "tla_control": "expiry-expansion",
        "witness": "ExpiryExpansionWitness",
        "assertion": "ExpiryScopeNeverExceedsClaim",
        "fact": "ClaimExpiryBounded",
    },
}
for c in controls:
    e = expected[c["id"]]
    if (
        c["tla_invariant"],
        c["tla_control"],
        c["alloy_witness"],
        c["alloy_assertion"],
        c["alloy_mutant_fact"],
    ) != (
        e["tla_invariant"], e["tla_control"], e["witness"], e["assertion"], e["fact"]
    ):
        raise RuntimeError("matrix semantics mismatch: " + c["id"])

head = run(["git", "rev-parse", "HEAD"])
tree = run(["git", "rev-parse", "HEAD^{tree}"])
if head.returncode or tree.returncode:
    raise RuntimeError("exact head/tree resolution failed")

runtime = json.loads(a.runtime.read_text(encoding="utf-8"))
if runtime.get("schema") != "mycelix.evidence-attestation-capability-formal-runtime.v1":
    raise RuntimeError("runtime schema mismatch")

receipt = {
    "receipt_schema": "mycelix.evidence-attestation-capability-formal-receipt.v1",
    "result": "ExecutedFail",
    "repository": {"head": head.stdout.strip(), "tree": tree.stdout.strip()},
    "runtime": runtime,
    "input_sha256": {
        str(p): fsha(p)
        for p in (
            a.matrix, a.tla, a.canonical_cfg, a.negative_tla,
            a.negative_resource_cfg, a.negative_action_cfg,
            a.negative_audience_cfg, a.negative_expiry_cfg,
            a.alloy, a.reference, a.runtime,
        )
    },
}

reference = run(["python3", str(a.reference)])
(a.evidence_dir / "reference.log").write_text(reference.stdout, encoding="utf-8")
if reference.returncode:
    raise RuntimeError("reference model returned nonzero")
for c in controls:
    for marker in c["reference_markers"]:
        if marker not in reference.stdout:
            raise RuntimeError("reference marker missing: " + marker)
receipt["reference"] = {
    "returncode": reference.returncode,
    "stdout_sha256": sha(reference.stdout.encode()),
}

def tlc(cfg, label):
    result = run([
        "java", "-cp", str(a.tla_jar), "tlc2.TLC",
        "-workers", "1", "-config", str(cfg), str(a.tla),
    ])
    (a.evidence_dir / (label + ".log")).write_text(result.stdout, encoding="utf-8")
    if result.returncode == 0 and "Model checking completed. No error has been found." in result.stdout:
        return set()
    return set(re.findall(
        r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.", result.stdout
    ))

canonical = tlc(a.canonical_cfg, "tla-canonical")
if canonical:
    raise RuntimeError("canonical TLA violated: " + repr(sorted(canonical)))

negative_cfgs = {
    "resource-expansion": a.negative_resource_cfg,
    "action-expansion": a.negative_action_cfg,
    "audience-expansion": a.negative_audience_cfg,
    "expiry-expansion": a.negative_expiry_cfg,
}
tla_negative = {}
for control_id, cfg in negative_cfgs.items():
    violations = tlc(cfg, "tla-negative-" + control_id)
    expected_invariant = expected[control_id]["tla_invariant"]
    if violations != {expected_invariant}:
        raise RuntimeError(
            f"TLA negative isolation mismatch for {control_id}: {sorted(violations)}"
        )
    tla_negative[control_id] = sorted(violations)
receipt["tla"] = {"canonical": "PASS", "negative": tla_negative}

cp = f"{a.runner_class_dir}:{a.alloy_jar}"
alloy_canonical = run([
    "java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(a.alloy)
])
(a.evidence_dir / "alloy-canonical.log").write_text(alloy_canonical.stdout, encoding="utf-8")
if alloy_canonical.returncode:
    raise RuntimeError("canonical Alloy runner failed")
rows = alloy_rows(alloy_canonical.stdout)

required_canonical = {"ValidCapabilityDeltaWitness": "SAT"}
for c in controls:
    required_canonical[c["alloy_witness"]] = "UNSAT"
    required_canonical[c["alloy_assertion"]] = "UNSAT"

# The multidimensional model uses a single positive witness plus four negative witnesses.
if {k: rows.get(k) for k in required_canonical} != required_canonical:
    raise RuntimeError("canonical Alloy outcomes mismatch: " + repr(rows))

receipt["alloy"] = {"canonical": rows, "mutations": {}}
source = a.alloy.read_text(encoding="utf-8")

for control_id in expected:
    fact = expected[control_id]["fact"]
    witness = expected[control_id]["witness"]
    assertion = expected[control_id]["assertion"]
    mutant_path = a.evidence_dir / f"alloy-negative-{control_id}.als"
    mutant_path.write_text(remove_fact(source, fact), encoding="utf-8")
    mutant = run([
        "java", "-cp", cp, "AgentDelegationAuthorityAlloyRunner", str(mutant_path)
    ])
    (a.evidence_dir / f"alloy-negative-{control_id}.log").write_text(mutant.stdout, encoding="utf-8")
    if mutant.returncode:
        raise RuntimeError("Alloy mutant failed: " + control_id)
    mrows = alloy_rows(mutant.stdout)
    if mrows.get(witness) != "SAT" or mrows.get(assertion) != "SAT":
        raise RuntimeError("target Alloy mutant did not become SAT: " + control_id)
    changed = {
        label for label in set(rows) | set(mrows)
        if rows.get(label) != mrows.get(label)
    }
    allowed = {witness, assertion}
    if changed != allowed:
        raise RuntimeError(
            f"unrelated Alloy outcomes changed for {control_id}: {sorted(changed)}"
        )
    receipt["alloy"]["mutations"][control_id] = {
        "removed_fact": fact,
        "changed_outcomes": sorted(changed),
        "outcomes": mrows,
    }

receipt["result"] = "ExecutedPass"
(a.evidence_dir / "evidence-attestation-capability-formal-receipt-v1.json").write_text(
    json.dumps(receipt, indent=2, sort_keys=True) + "
",
    encoding="utf-8",
)
print(json.dumps(receipt, sort_keys=True))
