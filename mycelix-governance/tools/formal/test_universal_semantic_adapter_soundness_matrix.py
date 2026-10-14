import argparse
import json
from pathlib import Path

IDS=[
"monotonicity-sampled-only","monotonicity-violation-outside-sample","undeclared-adapter-domain",
"schema-semantic-type-substitution","unit-conversion-expansion","lossy-redaction-reconstitution",
"adapter-implementation-substitution","external-dependency-substitution","partial-adapter-success",
"domain-precondition-bypass"]

MUTATIONS={
"monotonicity-sampled-only":{"MonotonicityDomainMax"},
"monotonicity-violation-outside-sample":{"TransformVariant"},
"undeclared-adapter-domain":{"DeclaredDomainMax"},
"schema-semantic-type-substitution":{"SourceSemanticType"},
"unit-conversion-expansion":{"UnitRule"},
"lossy-redaction-reconstitution":{"RemovedFields","ReconstitutedFields"},
"adapter-implementation-substitution":{"ImplementationHash"},
"external-dependency-substitution":{"ExternalDependencyState"},
"partial-adapter-success":{"AdapterStatus"},
"domain-precondition-bypass":{"PreconditionSatisfied"}}

def parse_cfg(path):
    result={}
    for line in path.read_text().splitlines():
        if " = " not in line or line.startswith("CONSTANTS"): continue
        key,value=line.split(" = ",1); result[key]=value
    return result

parser=argparse.ArgumentParser()
for name in ("matrix","tla","negative-tla","alloy","reference"):
    parser.add_argument("--"+name,type=Path,required=True)
args=parser.parse_args()
matrix=json.loads(args.matrix.read_text())
assert matrix["schema"]=="mycelix.evidence-attestation-universal-semantic-adapter-soundness-control-matrix.v1"
assert [c["id"] for c in matrix["controls"]]==IDS
tla=args.tla.read_text(); alloy=args.alloy.read_text(); reference=args.reference.read_text()
for c in matrix["controls"]:
    assert c["tla_invariant"] in tla
    assert c["alloy_assertion"] in alloy
    assert c["alloy_witness"] in alloy
    assert reference.count(c["reference_marker"])==1
assert "MutantAggregate_" in alloy
assert "fact Environment" in alloy
assert "UniversalTransformSound" in tla
assert "MonotoneUniversal" in tla
base=parse_cfg(args.tla.with_suffix(".cfg"))
for cid,keys in MUTATIONS.items():
    neg=parse_cfg(args.tla.parent/f"EvidenceAttestationUniversalSemanticAdapterSoundnessV1Negative-{cid}.cfg")
    assert set(neg)==set(base)|{"Control"}
    changed={k for k in base if neg.get(k)!=base[k]}
    assert changed==keys,(cid,changed,keys)
print("UNIVERSAL ADAPTER MATRIX PASS: 10 controls structurally aligned")
print("MUTATION SEMANTICS PASS: Alloy mutant assertions are explicit and independently scoped")
print("NON-AUTHORITATIVE: static alignment only")
