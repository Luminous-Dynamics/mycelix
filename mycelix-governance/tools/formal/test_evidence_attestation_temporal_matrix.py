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

m = json.loads(args.matrix.read_text(encoding="utf-8"))
c = m["control"]
assert m["schema"] == "mycelix.evidence-attestation-temporal-expiration-control-matrix.v1"
assert c["id"] == "expiry-persistence"

tla = args.tla.read_text(encoding="utf-8")
negative = args.negative_tla.read_text(encoding="utf-8")
alloy = args.alloy.read_text(encoding="utf-8")
reference = args.reference.read_text(encoding="utf-8")

assert c["tla_invariant"] in tla
assert c["tla_control"] in negative
assert c["alloy_witness"] in alloy
assert c["alloy_assertion"] in alloy
assert c["alloy_mutant_fact"] in alloy
for marker in c["reference_markers"]:
    assert marker in reference

print("TEMPORAL MATRIX PASS: expiration seam is represented across TLA+, Alloy, and reference semantics")
print("NON-AUTHORITATIVE: static alignment only")
