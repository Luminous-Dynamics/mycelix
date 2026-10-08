#!/usr/bin/env python3
"""Static matrix checks for the evidence attestation trust/subject seam."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
M = ROOT / "docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_SUBJECT_CONTROL_MATRIX_V1.json"
T = ROOT / "mycelix-governance/specs/EvidenceAttestationProvenanceV1.tla"
N = ROOT / "mycelix-governance/specs/EvidenceAttestationProvenanceV1NegativeControls.tla"
A = ROOT / "mycelix-governance/specs/alloy/EvidenceAttestationProvenanceV1.als"
R = ROOT / "mycelix-governance/tools/formal/evidence_attestation_provenance_reference.py"

matrix = json.loads(M.read_text(encoding="utf-8"))
assert matrix["schema"] == "mycelix.evidence-attestation-subject-control-matrix.v1"
assert [c["id"] for c in matrix["controls"]] == ["untrusted-attestation", "subject-mismatch"]

tla = T.read_text(encoding="utf-8")
neg = N.read_text(encoding="utf-8")
alloy = A.read_text(encoding="utf-8")
reference = R.read_text(encoding="utf-8")

for c in matrix["controls"]:
    assert c["tla_invariant"] in tla
    assert c["tla_control"] in neg
    assert c["alloy_witness"] in alloy
    assert c["alloy_assertion"] in alloy
    assert c["alloy_mutant_fact"] in alloy
    for marker in c["reference_markers"]:
        assert marker in reference

print("STATIC PASS: two independent attestation controls mapped across all three models")
print("NON-AUTHORITATIVE: static alignment evidence only")
