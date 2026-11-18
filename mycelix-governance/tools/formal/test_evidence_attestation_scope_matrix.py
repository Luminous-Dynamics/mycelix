#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
m=json.loads((ROOT/"docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_SCOPE_CONTROL_MATRIX_V1.json").read_text())
t=(ROOT/"mycelix-governance/specs/EvidenceAttestationScopeAttenuationV1.tla").read_text()
n=(ROOT/"mycelix-governance/specs/EvidenceAttestationScopeAttenuationV1NegativeControls.tla").read_text()
a=(ROOT/"mycelix-governance/specs/alloy/EvidenceAttestationScopeAttenuationV1.als").read_text()
r=(ROOT/"mycelix-governance/tools/formal/evidence_attestation_scope_reference.py").read_text()
c=m["control"]
assert m["schema"]=="mycelix.evidence-attestation-scope-attenuation-control-matrix.v1"
assert c["id"]=="scope-expansion"
assert c["tla_invariant"] in t and c["tla_control"] in n
assert c["alloy_witness"] in a and c["alloy_assertion"] in a and c["alloy_mutant_fact"] in a
for marker in c["reference_markers"]:
    assert marker in r
print("SCOPE MATRIX PASS: attenuation control is represented across TLA+, Alloy, and reference semantics")
print("NON-AUTHORITATIVE: static alignment only")
