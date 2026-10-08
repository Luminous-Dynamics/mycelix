#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

IDS = [
    "uncovered-field",
    "unknown-field",
    "value-substitution",
    "source-substitution",
    "implicit-default",
    "schema-profile",
    "required-omission",
]

p = argparse.ArgumentParser()
for n in ("matrix", "tla", "negative-tla", "alloy", "reference"):
    p.add_argument("--" + n, type=Path, required=True)
a = p.parse_args()

m = json.loads(a.matrix.read_text())
assert m["schema"] == "mycelix.evidence-attestation-semantic-projection-control-matrix.v1"
controls = m["controls"]
assert [c["id"] for c in controls] == IDS

t = a.tla.read_text()
n = a.negative_tla.read_text()
al = a.alloy.read_text()
ref = a.reference.read_text()

for c in controls:
    assert c["tla_invariant"] in t
    assert c["alloy_witness"] in al
    assert c["alloy_assertion"] in al
    assert c["alloy_mutant_fact"] in al
    assert c["reference_marker"] in ref

assert "CANONICAL PASS" in ref
assert "NEGATIVE PASS" in ref
print("SEMANTIC PROJECTION MATRIX PASS: seven exact projection controls are represented")
print("NON-AUTHORITATIVE: static alignment only")
