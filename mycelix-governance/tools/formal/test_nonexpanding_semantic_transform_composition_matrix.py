import argparse,json
from pathlib import Path
IDS=["upstream-decision-identity-substitution","derivation-rule-substitution","source-amount-substitution","audience-widening","second-hop-profile-ceiling-widening","second-hop-effect-widening","chain-link-substitution","missing-monotonicity-declaration","monotonicity-violation","downstream-implementation-substitution"]
OVERRIDES={
"upstream-decision-identity-substitution":{"Step1SourceDecisionId"},
"derivation-rule-substitution":{"Profile2DerivationRuleId","UsedDerivationRule2"},
"source-amount-substitution":{"Step2SourceAmountField"},
"audience-widening":{"Profile2Audiences"},
"second-hop-profile-ceiling-widening":{"Profile2MaxAmount"},
"second-hop-effect-widening":{"Effect2MaxAmount","Step2RenderedAmount"},
"chain-link-substitution":{"Step2SourceEffectId","Step2InputMaxAmount"},
"missing-monotonicity-declaration":{"MonotoneDeclared"},
"monotonicity-violation":{"MonoEffectBMaxAmount"},
"downstream-implementation-substitution":{"ImplementationHash2"},
}
def parse_cfg(path):
    values={}
    for line in path.read_text().splitlines():
        line=line.strip()
        if " = " not in line: continue
        if " = " in line and not line.startswith("CONSTANTS"):
            key,val=line.split(" = ",1); values[key]=val
    return values
p=argparse.ArgumentParser()
for n in ("matrix","tla","negative-tla","alloy","reference"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-nonexpanding-semantic-transform-composition-control-matrix.v1"
assert [c["id"] for c in m["controls"]]==IDS
t=a.tla.read_text(); n=a.negative_tla.read_text(); al=a.alloy.read_text(); ref=a.reference.read_text()
for c in m["controls"]:
    assert c["tla_invariant"] in t
    assert c["alloy_witness"] in al
    assert c["alloy_assertion"] in al
    assert ref.count(c["reference_marker"]) == 1
assert "ScopeLeq" in t
assert "ScopeLeq" in al or "scopeLeq" in al
assert "NonExpandingSemanticTransformCompositionExact" in al
assert "CANONICAL PASS" in ref and "COMPOSITION PASS" in ref
canon_cfg=Path(str(a.tla).replace(".tla",".cfg"))
canonical_values=parse_cfg(canon_cfg)
negative_dir=canon_cfg.parent
for cid,allowed in OVERRIDES.items():
    vals=parse_cfg(negative_dir/f"EvidenceAttestationNonExpandingSemanticTransformCompositionV1Negative-{cid}.cfg")
    assert set(vals)==set(canonical_values)|{"Control"}
    changed={k for k in canonical_values if vals.get(k)!=canonical_values[k]}
    assert changed==allowed,(cid,changed,allowed)
print("NON-EXPANDING COMPOSITION MATRIX PASS: 10 exact controls are represented across TLA+/Alloy/reference")
print("NON-AUTHORITATIVE: static alignment only")
