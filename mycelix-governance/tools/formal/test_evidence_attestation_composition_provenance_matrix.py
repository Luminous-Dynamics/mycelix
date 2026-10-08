#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument("--matrix",type=Path,required=True)
parser.add_argument("--tla",type=Path,required=True)
parser.add_argument("--negative-tla",type=Path,required=True)
parser.add_argument("--alloy",type=Path,required=True)
parser.add_argument("--reference",type=Path,required=True)
args=parser.parse_args()

m=json.loads(args.matrix.read_text(encoding="utf-8"))
c=m["control"]
assert m["schema"]=="mycelix.evidence-attestation-composition-provenance-control-matrix.v1"
assert c["id"]=="contributor-substitution"
assert c["tla_invariant"] in args.tla.read_text(encoding="utf-8")
assert c["tla_control"] in args.negative_tla.read_text(encoding="utf-8")
a=args.alloy.read_text(encoding="utf-8")
assert c["alloy_witness"] in a and c["alloy_assertion"] in a and c["alloy_mutant_fact"] in a
r=args.reference.read_text(encoding="utf-8")
for marker in c["reference_markers"]:
    assert marker in r
print("COMPOSITION PROVENANCE MATRIX PASS: exact contributor identity is represented across TLA+, Alloy, and reference semantics")
print("NON-AUTHORITATIVE: static alignment only")
