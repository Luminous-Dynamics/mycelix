#!/usr/bin/env python3
import argparse,json
from pathlib import Path

IDS=["profile-ceiling-widening","undeclared-derivation","privilege-widening","cross-field-contamination","profile-substitution","implementation-hash-substitution","conversion-rule-substitution","default-reconstruction","external-enrichment","unbound-context"]

p=argparse.ArgumentParser()
for n in ("matrix","tla","negative-tla","alloy","reference"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args()
m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-nonexpanding-semantic-transform-control-matrix.v1"
assert [c["id"] for c in m["controls"]]==IDS
t=a.tla.read_text(); n=a.negative_tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t
    assert c["alloy_witness"] in al
    assert c["alloy_assertion"] in al
    assert c["alloy_mutant_fact"] in al
    assert c["reference_marker"] in ref
assert "EffectScopeNarrowed" in t
assert "ProfileCeilingNarrowed" in t
assert "scopeLeq" in al
assert "CANONICAL PASS" in ref and "NEGATIVE PASS" in ref
print("NON-EXPANDING TRANSFORM MATRIX PASS: nine exact transformation controls are represented")
print("NON-AUTHORITATIVE: static alignment only")
