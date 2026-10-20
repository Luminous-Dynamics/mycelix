#!/usr/bin/env python3
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser()
for n in ("matrix","tla","negative-tla","alloy","reference"): p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args()
m=json.loads(a.matrix.read_text())
expected=["decision-identity","intent-identity","operation-commitment","use-commitment","target","adapter","invocation-identity"]
assert m["schema"]=="mycelix.evidence-attestation-effect-handoff-identity-control-matrix.v1"
assert [c["id"] for c in m["controls"]]==expected
t=a.tla.read_text(); n=a.negative_tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t and c["tla_control"] in n
    assert c["alloy_witness"] in al and c["alloy_assertion"] in al and c["alloy_mutant_fact"] in al
assert "CANONICAL PASS" in ref and "NEGATIVE PASS" in ref
print("EFFECT HANDOFF IDENTITY MATRIX PASS: seven exact causal bindings are represented")
print("NON-AUTHORITATIVE: static alignment only")
