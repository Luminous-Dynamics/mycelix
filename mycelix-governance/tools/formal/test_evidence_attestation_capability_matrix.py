#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

parser = argparse.ArgumentParser()
parser.add_argument("--matrix", type=Path, required=True)
parser.add_argument("--tla", type=Path, required=True)
parser.add_argument("--negative-tla", type=Path, required=True)
parser.add_argument("--alloy", type=Path, required=True)
parser.add_argument("--reference", type=Path, required=True)
args = parser.parse_args()

matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
assert matrix["schema"] == "mycelix.evidence-attestation-capability-attenuation-control-matrix.v1"
controls = matrix["controls"]
assert [c["id"] for c in controls] == [
    "resource-expansion",
    "action-expansion",
    "audience-expansion",
    "expiry-expansion",
]
assert len({c["id"] for c in controls}) == 4

tla = args.tla.read_text(encoding="utf-8")
negative = args.negative_tla.read_text(encoding="utf-8")
alloy = args.alloy.read_text(encoding="utf-8")
reference = args.reference.read_text(encoding="utf-8")

for control in controls:
    for token in (control["tla_invariant"], control["tla_control"]):
        assert token in (tla if token == control["tla_invariant"] else negative), token
    for token in (
        control["alloy_witness"],
        control["alloy_assertion"],
        control["alloy_mutant_fact"],
    ):
        assert token in alloy, token
    for marker in control["reference_markers"]:
        assert marker in reference, marker

print("CAPABILITY MATRIX PASS: four independent attenuation dimensions are represented across TLA+, Alloy, and reference semantics")
print("NON-AUTHORITATIVE: static alignment only")
