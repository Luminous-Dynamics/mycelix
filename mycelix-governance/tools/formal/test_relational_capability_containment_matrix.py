import argparse,json
from pathlib import Path
IDS=["cartesian-recombination","target-currency-correlation","operation-argument-correlation","audience-target-correlation","wildcard-expansion","capability-set-identity-substitution","tuple-normalization-substitution","profile-widening-unchanged-marginals","effect-outside-relation-inside-marginals","downstream-relational-subset-bypass"]
p=argparse.ArgumentParser()
for n in ("matrix","tla","negative-tla","alloy","reference"): p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-relational-capability-containment-control-matrix.v1"
assert [c["id"] for c in m["controls"]]==IDS
t=a.tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t and c["alloy_assertion"] in al and c["reference_marker"] in ref
assert "TupleLeq" in t and "SetLeq" in t and "semEquivalent" in al
print("RELATIONAL MATRIX PASS: 10 controls aligned across TLA+/Alloy/reference")
print("NON-AUTHORITATIVE: structural/static checks only")
