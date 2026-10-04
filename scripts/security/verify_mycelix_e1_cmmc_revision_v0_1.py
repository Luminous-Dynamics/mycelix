#!/usr/bin/env python3
"""Dependency-free validator for the E1/CMMC revision boundary contract."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-e1-cmmc-revision-boundary-v0.1.json"

EXPECTED_STATUSES = {
    "AlreadyQualified",
    "ImplementedUnqualified",
    "Designed",
    "CandidateMapping",
    "Gap",
    "OrganizationDecisionRequired",
    "ContractDecisionRequired",
    "IndependentAssessorRequired",
    "NotApplicable",
}


def main() -> int:
    c = json.loads(CONTRACT.read_text(encoding="utf-8"))
    failures: list[str] = []

    if c["claim_ceiling"] != "ReferenceModelOnly":
        failures.append("claim ceiling was widened")

    if c["current_external_state"]["cmmc_phase_two"] != "suspended-2026-07-13":
        failures.append("CMMC Phase II state drifted")

    if c["current_external_state"]["operational_level_two_basis"].find("Rev.2") < 0:
        failures.append("operative Level 2 basis no longer records Rev.2")

    if c["current_external_state"]["engineering_target"].find("Rev.3") < 0:
        failures.append("engineering target no longer records Rev.3")

    families = c["control_families"]
    for family in families:
        if family["status"] not in EXPECTED_STATUSES:
            failures.append(f'unknown status: {family["family"]}')
    if not any(f["status"] == "OrganizationDecisionRequired" for f in families):
        failures.append("organizational boundary disappeared")
    if not any(f["status"] == "Gap" for f in families):
        failures.append("technical/operational gaps disappeared")

    vector_expectations = {
        "CMMC-BOUNDARY-001": "DENY",
        "CMMC-BOUNDARY-002": "DENY",
        "CMMC-BOUNDARY-003": "PASS",
        "CMMC-BOUNDARY-004": "ORGANIZATION_DECISION_REQUIRED",
        "CMMC-BOUNDARY-005": "INDEPENDENT_ASSESSOR_REQUIRED",
        "CMMC-BOUNDARY-006": "PASS",
        "CMMC-BOUNDARY-007": "ORGANIZATION_DECISION_REQUIRED",
        "CMMC-BOUNDARY-008": "CONTRACT_DECISION_REQUIRED",
        "CMMC-BOUNDARY-009": "DENY",
        "CMMC-BOUNDARY-010": "DENY",
        "CMMC-BOUNDARY-011": "DENY",
        "CMMC-BOUNDARY-012": "PASS",
    }
    actual = {v["id"]: v["expected"] for v in c["qualification_vectors"]}
    for vector_id, expected in vector_expectations.items():
        if actual.get(vector_id) != expected:
            failures.append(f"{vector_id}: expected {expected}")

    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1

    print(f"E1/CMMC revision-boundary qualification: {len(vector_expectations)}/{len(vector_expectations)} vectors structurally valid")
    print("Claim ceiling: ReferenceModelOnly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
