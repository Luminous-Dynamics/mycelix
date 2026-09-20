#!/usr/bin/env python3
"""Validate CIV-RES-000A Civic Resilience composition boundary v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_000A_COMPOSITION_BOUNDARY_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_000a_manifest.json"

EXPECTED_TOP_LEVEL = {
    "schema",
    "program",
    "source_base",
    "authority",
    "runtime_changes",
    "planes",
    "semantic_owners",
    "required_non_equivalences",
    "composition_rules",
    "forbidden_redefinitions",
    "continuation",
    "nonclaims",
}

EXPECTED_PLANES = ["Legitimacy", "Authority", "Evidence", "Procedure", "Effect"]
EXPECTED_OWNER_CONCERNS = [
    "institutional_legitimacy",
    "administrative_procedure_review_finality",
    "anti_capture_civic_standing",
    "person_linked_access_accountability",
    "governance_authority_provenance_conservation",
    "municipal_public_institution_integration",
    "mycelix_symthaea_state_model_ownership",
    "evidence_currentness_qualification",
]
EXPECTED_NON_EQUIVALENCES = [
    "CivicObservation != AdministrativeDecision",
    "CivicSignal != AdjudicatedFinding",
    "ServiceRequest != Entitlement",
    "Commitment != CompletionEvidence",
    "CompletionEvidence != IndependentVerification",
    "IndependentVerification != OutcomeEffect",
    "ModelEstimate != Observation",
    "ModelRecommendation != CivicAuthority",
    "ProcurementOpportunity != AwardAuthority",
    "Award != Contract",
    "Contract != Payment",
    "ReportedCrime != TrueCrimeIncidence",
    "LowReportedCrime != ImprovedSafety",
    "AggregateTrend != IndividualRisk",
    "AccessReceipt != PermissionGrant",
    "SoftwareCapability != InstitutionalLegitimacy",
    "QualifiedUpstream != QualifiedComposition",
    "CrossRepoSchema != SharedMutableTruth",
    "HistoricalPass != CurrentQualifiedEvidence",
    "EmergencyContext != PermanentRetentionAuthority",
]
EXPECTED_RULES = [
    "composition_must_not_widen_scope",
    "composition_must_not_extend_validity",
    "composition_must_not_strengthen_consequence_class",
    "composition_must_not_create_disclosure_authority",
    "composition_must_not_create_redelegation_rights",
    "composition_must_not_create_external_effect_authority",
    "upstream_qualification_does_not_qualify_composition",
    "draft_reference_does_not_create_executable_dependency",
    "symthaea_output_is_model_evidence_not_adopted_state",
]
EXPECTED_FORBIDDEN = [
    "civic_sovereign",
    "universal_government_superuser",
    "master_resident_database",
    "universal_reputation_score",
    "individual_criminality_score",
    "autonomous_rights_affecting_ai",
    "police_targeting_authority",
    "procurement_award_authority",
    "payment_authority",
    "entitlement_authority",
    "generic_person_level_cross_domain_join",
    "generic_emergency_bypass",
    "permanent_emergency_data_retention",
    "symthaea_civic_state_mutation_authority",
    "parallel_administrative_procedure_system",
    "implicit_qualification_inheritance",
]
EXPECTED_CONTINUATION = ["CIV-RES-000B", "CIV-RES-001A", "CIV-RES-001B", "CIV-RES-001C"]
EXPECTED_NONCLAIMS = [
    "runtime_enforcement",
    "upstream_dependency_qualification",
    "legal_validity",
    "popia_compliance",
    "johannesburg_adoption",
    "municipal_legitimacy",
    "public_sector_readiness",
    "privacy_or_security_adequacy",
    "causal_effectiveness",
    "crime_reduction",
    "procurement_authority",
    "law_enforcement_authority",
    "deployment_readiness",
]


def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-000A FAIL: {message}")


def require_exact_list(name: str, actual: object, expected: list[str]) -> None:
    if actual != expected:
        fail(f"{name} must equal the canonical ordered registry")


def main() -> int:
    if not DOC.is_file():
        fail(f"missing document: {DOC}")
    if not MANIFEST.is_file():
        fail(f"missing manifest: {MANIFEST}")

    text = DOC.read_text(encoding="utf-8")
    try:
        data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        fail(f"manifest is not valid JSON: {exc}")

    if set(data) != EXPECTED_TOP_LEVEL:
        fail("manifest top-level schema drift")
    if data["schema"] != "mycelix.civic-resilience.composition-boundary.v1":
        fail("wrong schema identity")
    if data["program"] != "CIV-RES-000A":
        fail("wrong program identity")
    if data["source_base"] != "a85369699099d4c7524e502e531735eed4ab36f4":
        fail("source base changed without a new lineage")
    if data["authority"] != "architecture_only":
        fail("CIV-RES-000A may not carry runtime authority")
    if data["runtime_changes"] is not False:
        fail("runtime_changes must remain false")

    require_exact_list("planes", data["planes"], EXPECTED_PLANES)
    require_exact_list("required_non_equivalences", data["required_non_equivalences"], EXPECTED_NON_EQUIVALENCES)
    require_exact_list("composition_rules", data["composition_rules"], EXPECTED_RULES)
    require_exact_list("forbidden_redefinitions", data["forbidden_redefinitions"], EXPECTED_FORBIDDEN)
    require_exact_list("continuation", data["continuation"], EXPECTED_CONTINUATION)
    require_exact_list("nonclaims", data["nonclaims"], EXPECTED_NONCLAIMS)

    owners = data["semantic_owners"]
    if not isinstance(owners, list) or len(owners) != len(EXPECTED_OWNER_CONCERNS):
        fail("semantic owner census changed")
    concerns = [entry.get("concern") for entry in owners]
    if concerns != EXPECTED_OWNER_CONCERNS:
        fail("semantic owner concerns must remain canonical and ordered")
    for entry in owners:
        if set(entry) != {"concern", "owner", "reference", "dependency_state"}:
            fail(f"closed owner schema violated for {entry.get('concern')!r}")
        if not entry["owner"] or not entry["reference"]:
            fail(f"owner/reference missing for {entry['concern']!r}")
        if entry["dependency_state"] != "architectural_reference_only":
            fail(f"dependency {entry['concern']!r} was silently promoted")

    for boundary in EXPECTED_NON_EQUIVALENCES:
        if boundary not in text:
            fail(f"document missing non-equivalence: {boundary!r}")

    required_phrases = [
        "composition != authority mint",
        "reference to upstream work != qualification inheritance",
        "qualified(A) AND qualified(B) != qualified(compose(A, B))",
        "draft dependency reference != executable dependency",
        "Symthaea analysis receipt -> evidence/proposal",
        "AccessReceipt != PermissionGrant",
        "documentation + machine validation only",
        "CIV-RES-000B",
    ]
    for phrase in required_phrases:
        if phrase not in text:
            fail(f"document missing required composition phrase: {phrase!r}")

    lower = text.lower()
    forbidden_positive_phrases = [
        "civic resilience is the sovereign",
        "symthaea may authorize sanctions",
        "model output is an observation",
        "access receipt grants permission",
        "award authorizes payment",
        "qualified upstream automatically qualifies",
    ]
    for phrase in forbidden_positive_phrases:
        if phrase in lower:
            fail(f"forbidden authority shortcut found: {phrase!r}")

    if text.count("!=") < len(EXPECTED_NON_EQUIVALENCES) + 8:
        fail("composition boundary lost explicit non-equivalence density")

    print("CIV-RES-000A PASS: ownership, non-equivalence, and non-inheritance boundaries preserved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
