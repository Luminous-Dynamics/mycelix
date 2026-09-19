#!/usr/bin/env python3
"""Validate CIV-RES-000B Johannesburg deployment/threat/privacy profile v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_000B_JOHANNESBURG_PROFILE_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_000b_johannesburg_profile.json"

EXPECTED_TOP_LEVEL = {
    "schema",
    "program",
    "parent_subject",
    "deployment_profile",
    "deployment_readiness",
    "real_world_processing_authorized",
    "official_source_anchors",
    "city_context_domains",
    "service_channel_references",
    "data_planes",
    "sensitivity_classes",
    "geographic_precision",
    "temporal_precision",
    "legal_review_flags",
    "required_unbound_deployment_bindings",
    "default_denials",
    "threat_coverage_classes",
    "threat_census",
    "threats_mitigated",
    "pilot_stages",
    "current_max_stage",
    "continuation",
    "nonclaims",
}

EXPECTED_SOURCE_IDS = [
    "POPIA_ACT_4_2013",
    "COJ_2026_27_IDP_INDEX",
    "COJ_2026_27_IDP_FAQ",
    "COJ_JOBURG_CONNECT",
]
EXPECTED_DATA_PLANES = [
    "PublicProjection",
    "ProtectedOperations",
    "ResearchEnclave",
    "EmergencyEphemeral",
]
EXPECTED_SENSITIVITY = [
    "PublicCivic",
    "CommunitySensitive",
    "Restricted",
    "EmergencyEphemeral",
]
EXPECTED_GEO = ["City", "Region", "Ward", "CoarseCell", "ExactProtected"]
EXPECTED_TIME = [
    "HistoricalAggregate",
    "DelayedAggregate",
    "OperationalCurrentProtected",
    "EmergencyCurrentProtected",
]
EXPECTED_LEGAL_FLAGS = [
    "POPIA_S13_SPECIFIC_PURPOSE",
    "POPIA_S14_RETENTION_RESTRICTION",
    "POPIA_S15_FURTHER_PROCESSING_COMPATIBILITY",
    "POPIA_S18_NOTIFICATION_OPENNESS",
    "POPIA_S19_SECURITY_SAFEGUARDS",
    "POPIA_S20_21_OPERATOR_SECURITY",
    "POPIA_S22_SECURITY_COMPROMISE_NOTIFICATION",
    "POPIA_S26_35_SPECIAL_AND_CHILD_INFORMATION",
    "POPIA_S57_59_PRIOR_AUTHORISATION",
    "POPIA_S71_AUTOMATED_DECISION_MAKING",
    "POPIA_S72_TRANSBORDER_FLOW",
]
EXPECTED_BINDINGS = [
    "responsible_party",
    "operators",
    "information_officer_or_accountable_privacy_role",
    "data_stewards",
    "purpose_and_legal_policy_basis",
    "retention_restriction_deletion_schedule",
    "disclosure_release_policy",
    "cross_border_transfer_policy",
    "security_safeguard_risk_review_plan",
    "compromise_incident_response_plan",
    "independent_legal_privacy_review",
    "pilot_governance_oversight_authority",
    "accessibility_language_profile",
    "exit_rollback_plan",
]
EXPECTED_DENIALS = [
    "biometric_processing",
    "facial_recognition_identification_or_search",
    "criminal_behaviour_personal_information_processing",
    "child_personal_information_processing",
    "solely_automated_rights_affecting_decisions",
    "law_enforcement_target_generation",
    "public_exact_person_location",
    "routine_person_level_cross_domain_join",
    "cross_border_personal_information_transfer",
    "permanent_emergency_context_retention",
]
EXPECTED_THREAT_CLASSES = [
    "future_executable_attack",
    "deployment_control_required",
    "external_review_blocker",
    "residual_research_gap",
]
EXPECTED_THREAT_IDS = [f"JHB-T-{n:03d}" for n in range(1, 23)]
EXPECTED_STAGES = [
    "J0_SyntheticProfileOnly",
    "J1_PublicReferenceInteroperability",
    "J2_ProtectedServiceOperations",
    "J3_ResearchEnclaveShadowEvaluation",
    "J4_SensitiveSafetyOperations",
]
EXPECTED_NONCLAIMS = [
    "legal_advice",
    "popia_compliance",
    "municipal_adoption",
    "city_endorsement",
    "live_service_channel_integration",
    "privacy_security_adequacy",
    "prior_authorisation_determination",
    "cross_border_transfer_legality",
    "law_enforcement_authority",
    "crime_reduction",
    "safety_effectiveness",
    "deployment_readiness",
]


def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-000B FAIL: {message}")


def exact(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{name} drifted from the canonical profile")


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
    exact("schema", data["schema"], "mycelix.civic-resilience.johannesburg-profile.v1")
    exact("program", data["program"], "CIV-RES-000B")
    exact("parent subject", data["parent_subject"], "c6a96d40895e71642292b0e2c433b13cc58ecd4d")
    exact("deployment profile", data["deployment_profile"], "JohannesburgGautengSouthAfrica")
    exact("deployment readiness", data["deployment_readiness"], "ProfileTemplateOnly")
    if data["real_world_processing_authorized"] is not False:
        fail("real-world processing must remain unauthorized")
    if data["threats_mitigated"] is not False:
        fail("threat census must not be relabeled as mitigated")

    anchors = data["official_source_anchors"]
    if [x.get("id") for x in anchors] != EXPECTED_SOURCE_IDS:
        fail("official source-anchor census/order changed")
    for anchor in anchors:
        if set(anchor) != {"id", "url", "authority", "grants_deployment_authority"}:
            fail(f"source anchor schema drift: {anchor.get('id')!r}")
        if not anchor["url"].startswith("https://"):
            fail(f"non-HTTPS source anchor: {anchor['id']!r}")
        if anchor["grants_deployment_authority"] is not False:
            fail(f"source anchor cannot grant deployment authority: {anchor['id']!r}")

    exact("data planes", data["data_planes"], EXPECTED_DATA_PLANES)
    exact("sensitivity classes", data["sensitivity_classes"], EXPECTED_SENSITIVITY)
    exact("geographic precision", data["geographic_precision"], EXPECTED_GEO)
    exact("temporal precision", data["temporal_precision"], EXPECTED_TIME)
    exact("legal review flags", data["legal_review_flags"], EXPECTED_LEGAL_FLAGS)
    exact("required deployment bindings", data["required_unbound_deployment_bindings"], EXPECTED_BINDINGS)
    exact("default denials", data["default_denials"], EXPECTED_DENIALS)
    exact("threat coverage classes", data["threat_coverage_classes"], EXPECTED_THREAT_CLASSES)
    exact("pilot stages", data["pilot_stages"], EXPECTED_STAGES)
    exact("current max stage", data["current_max_stage"], "J0_SyntheticProfileOnly")
    exact("nonclaims", data["nonclaims"], EXPECTED_NONCLAIMS)

    threats = data["threat_census"]
    if not isinstance(threats, list) or len(threats) != len(EXPECTED_THREAT_IDS):
        fail("threat census cardinality changed")
    if [x.get("id") for x in threats] != EXPECTED_THREAT_IDS:
        fail("JHB-T-001..022 must appear exactly once and in canonical order")
    names = set()
    classes_seen = set()
    for threat in threats:
        if set(threat) != {"id", "name", "coverage"}:
            fail(f"closed threat schema violated at {threat.get('id')!r}")
        if not threat["name"] or threat["name"] in names:
            fail(f"empty/duplicate threat name: {threat.get('name')!r}")
        names.add(threat["name"])
        if threat["coverage"] not in EXPECTED_THREAT_CLASSES:
            fail(f"unknown coverage class for {threat['id']}")
        classes_seen.add(threat["coverage"])
    if classes_seen != set(EXPECTED_THREAT_CLASSES):
        fail("every threat coverage class must remain represented")

    required_doc_phrases = [
        "Johannesburg profile exists\n!= Johannesburg deployment authorised",
        "review flag != legal determination",
        "absence of flag != compliance",
        "legal approval != technical security",
        "ModelEstimate != AdministrativeDecision",
        "Cross-border personal-information transfer is **default denied**",
        "ExactProtected` must never appear in a public projection",
        "Disconnection may narrow authority; it must never increase authority.",
        "Passing this census means only that threats remain visible. It does not mean they are mitigated.",
        "J0 PASS != J1 authority",
    ]
    for phrase in required_doc_phrases:
        if phrase not in text:
            fail(f"document missing required boundary: {phrase!r}")

    for threat_id in EXPECTED_THREAT_IDS:
        if text.count(threat_id) != 1:
            fail(f"{threat_id} must appear exactly once in the narrative")

    forbidden_shortcuts = [
        "popia compliant = true",
        "deployment ready = true",
        "facial recognition enabled",
        "criminal risk score",
        "automated police target",
        "cross-border transfer allowed by default",
        "child data allowed by default",
    ]
    lower = text.lower()
    for phrase in forbidden_shortcuts:
        if phrase in lower:
            fail(f"forbidden deployment shortcut found: {phrase!r}")

    print("CIV-RES-000B PASS: Johannesburg profile remains blocked, explicit, and threat-accountable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
