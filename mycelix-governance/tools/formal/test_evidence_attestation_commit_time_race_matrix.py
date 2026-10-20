#!/usr/bin/env python3
import argparse, json
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument("--matrix",type=Path,required=True)
p.add_argument("--tla",type=Path,required=True)
p.add_argument("--negative-tla",type=Path,required=True)
p.add_argument("--alloy",type=Path,required=True)
p.add_argument("--reference",type=Path,required=True)
a=p.parse_args()

m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-commit-time-race-control-matrix.v1"
expected=["authority-epoch","request-commitment","target","policy-epoch","adapter-profile","invocation-identity","capability-expiry-change","time-expiry"]
assert [c["id"] for c in m["controls"]]==expected
t=a.tla.read_text(); n=a.negative_tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t
    assert c["tla_control"] in n
    assert c["alloy_witness"] in al
    assert c["alloy_assertion"] in al
    assert c["alloy_mutant_fact"] in al
assert "unsafe check-then-commit" in ref
print("COMMIT-TIME RACE MATRIX PASS: eight independent TOCTOU controls are represented")
print("NON-AUTHORITATIVE: static alignment only")
