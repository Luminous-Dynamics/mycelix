#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
m=json.loads((ROOT/"docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_CLAIM_AUTHORIZATION_CONTROL_MATRIX_V1.json").read_text())
t=(ROOT/"mycelix-governance/specs/EvidenceAttestationClaimAuthorizationV1.tla").read_text()
n=(ROOT/"mycelix-governance/specs/EvidenceAttestationClaimAuthorizationV1NegativeControls.tla").read_text()
a=(ROOT/"mycelix-governance/specs/alloy/EvidenceAttestationClaimAuthorizationV1.als").read_text()
r=(ROOT/"mycelix-governance/tools/formal/evidence_attestation_claim_authorization_reference.py").read_text()
c=m["control"]
assert m["schema"]=="mycelix.evidence-attestation-claim-authorization-control-matrix.v1"
assert c["id"]=="unauthorized-claim"
assert c["tla_invariant"] in t
assert c["tla_control"] in n
assert c["alloy_witness"] in a
assert c["alloy_assertion"] in a
assert c["alloy_mutant_fact"] in a
for marker in c["reference_markers"]:
    assert marker in r
print("CLAIM MATRIX PASS: claim authorization is independently represented across TLA+, Alloy, and reference semantics")
print("NON-AUTHORITATIVE: static alignment only")
