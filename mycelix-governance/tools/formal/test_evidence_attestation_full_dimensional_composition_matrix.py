#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--matrix", type=Path, required=True)
parser.add_argument("--tla", type=Path, required=True)
parser.add_argument("--negative-tla", type=Path, required=True)
parser.add_argument("--alloy", type=Path, required=True)
parser.add_argument("--reference", type=Path, required=True)
args = parser.parse_args()

matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
assert matrix["schema"] == "mycelix.evidence-attestation-full-dimensional-composition-control-matrix.v1"
controls = matrix["controls"]
assert [c["id"] for c in controls] == ["full-dimensional-hybrid"]
assert len(controls) == 1

tla = args.tla.read_text(encoding="utf-8")
negative_tla = args.negative_tla.read_text(encoding="utf-8")
alloy = args.alloy.read_text(encoding="utf-8")
reference = args.reference.read_text(encoding="utf-8")

control = controls[0]
assert control["tla_invariant"] in tla
assert control["tla_control"] in negative_tla
for token in (
    control["alloy_witness"],
    control["alloy_assertion"],
    control["alloy_mutant_fact"],
):
    assert token in alloy
for marker in control["reference_markers"]:
    assert marker in reference

for token in (
    "Claim1", "Claim2", "Claim3", "Claim4",
    "HybridCapability",
    "CompositionAtomAndProvenanceExact",
    "DimensionSourcesRemainIndividuallyValid",
):
    assert token in tla, token

print("FULL-DIMENSIONAL COMPOSITION MATRIX PASS: 4D atom, exact provenance, and independent dimension sources are represented")
print("NON-AUTHORITATIVE: static alignment only")
