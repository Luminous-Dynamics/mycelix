import argparse,json
from pathlib import Path
IDS=["cartesian-recombination","target-currency-correlation","operation-argument-correlation","audience-target-correlation","wildcard-expansion","capability-set-identity-substitution","tuple-normalization-substitution","profile-widening-unchanged-marginals","effect-outside-relation-inside-marginals","downstream-relational-subset-bypass"]
EXPECTED={"cartesian-recombination":{"ChildCapabilities"},"target-currency-correlation":{"ChildCapabilities"},"operation-argument-correlation":{"ChildCapabilities"},"audience-target-correlation":{"ChildCapabilities"},"wildcard-expansion":{"ChildCapabilities"},"capability-set-identity-substitution":{"ParentSetId"},"tuple-normalization-substitution":{"ChildCapabilities"},"profile-widening-unchanged-marginals":{"ChildCapabilities"},"effect-outside-relation-inside-marginals":{"ChildCapabilities"},"downstream-relational-subset-bypass":{"DownstreamCapabilities"}}
def cfg(path):
    out={}
    for line in path.read_text().splitlines():
        line=line.strip()
        if " = " in line and not line.startswith("CONSTANTS"):
            k,v=line.split(" = ",1); out[k]=v
    return out
p=argparse.ArgumentParser()
for n in ("matrix","tla","negative-tla","alloy","reference"): p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-relational-capability-containment-control-matrix.v1"
assert [c["id"] for c in m["controls"]]==IDS
t=a.tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t and c["alloy_assertion"] in al and c["reference_marker"] in ref
assert "TupleLeq" in t and "SetLeq" in t and "semEquivalent" in al
base=cfg(a.tla.with_suffix(".cfg"))
for c in m["controls"]:
    neg=cfg(a.tla.parent/f"EvidenceAttestationRelationalCapabilityContainmentV1Negative-{c['id']}.cfg")
    changed={k for k in base if neg.get(k)!=base[k]}
    assert changed==EXPECTED[c["id"]],(c["id"],changed,EXPECTED[c["id"]])
print("RELATIONAL MATRIX PASS: 10 controls aligned across TLA+/Alloy/reference")
print("NEGATIVE FIXTURE PASS: each CFG changes only its declared semantic mutation key(s)")
print("SEMANTIC ORDER PASS: antisymmetry is stated modulo semantic equivalence")
print("NON-AUTHORITATIVE: structural/static checks only")
