#!/usr/bin/env python3
"""Validate CIV-RES-001A domain-neutral civic semantic waist v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_001A_SEMANTIC_WAIST_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_001a_semantic_waist.json"

EXPECTED_TOP_LEVEL = {
    "schema",
    "program",
    "parent_subject",
    "topology",
    "runtime_owner",
    "subject_classes",
    "artifact_kinds",
    "opaque_reference_kinds",
    "required_non_equivalences",
    "forbidden_universal_subjects",
    "forbidden_authority_states",
    "evidence_primitive_policy",
    "candidate_runtime_owners",
    "continuation",
    "nonclaims",
}
EXPECTED_SUBJECTS = [
    "Place",
    "Service",
    "InfrastructureAsset",
    "InstitutionalProcess",
    "Programme",
    "Resource",
    "Aggregate",
]
EXPECTED_ARTIFACTS = [
    "CivicObservation",
    "CivicNeedStatement",
    "CivicInterventionProposal",
    "OutcomeTarget",
    "OutcomeObservation",
]
EXPECTED_REFS = [
    "EvidenceRef",
    "MeasurementRef",
    "MethodRef",
    "TimeEvidenceRef",
    "AuthorityDecisionRef",
    "ExternalEffectRef",
    "VerificationRef",
    "StudyRef",
]
EXPECTED_NON_EQUIVALENCES = [
    "Observation != Need",
    "Need != Priority",
    "Need != Entitlement",
    "NeedStatement != AdministrativeFinding",
    "InterventionProposal != AdoptedIntervention",
    "AdoptedIntervention != ExternalEffect",
    "OutcomeTarget != OutcomeObservation",
    "OutcomeObservation != CausalEffect",
    "CausalEffectEstimate != OutcomeObservation",
    "ReportedCondition != VerifiedCondition",
    "Verification != CausalAttribution",
    "CommunityPreference != InstitutionalAuthority",
    "ModelRecommendation != InterventionAuthority",
    "CommunityInput != RepresentativeMandate",
    "PlatformActivity != PriorityAuthority",
]
EXPECTED_FORBIDDEN_SUBJECTS = [
    "Person",
    "HouseholdRisk",
    "Suspect",
    "Offender",
    "VictimScore",
    "CitizenScore",
    "TrustScore",
]
EXPECTED_FORBIDDEN_STATES = [
    "Approved",
    "Official",
    "Authorised",
    "Funded",
    "Awarded",
    "Entitled",
    "Sanctioned",
    "Executed",
    "Paid",
]
EXPECTED_RUNTIME_CANDIDATES = [
    "civic-types-module-after-convergence",
    "dependency-light-civic-resilience-types",
    "shared-cross-domain-evidence-core",
]
EXPECTED_CONTINUATION = [
    "CIV-RES-001B",
    "CIV-RES-001C",
    "CIV-RES-002A",
    "SYM-CIVIC-000A",
]
EXPECTED_NONCLAIMS = [
    "runtime_api",
    "evidence_validity",
    "observation_truth",
    "need_validity",
    "representative_mandate",
    "priority_authority",
    "entitlement",
    "intervention_authority",
    "external_effect",
    "outcome_improvement",
    "causal_effect",
    "municipal_legitimacy",
    "johannesburg_deployment",
    "deployment_readiness",
]


def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-001A FAIL: {message}")


def exact(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{name} drifted from the canonical semantic contract")


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

    exact("schema", data["schema"], "mycelix.civic-resilience.semantic-waist.v1")
    exact("program", data["program"], "CIV-RES-001A")
    exact("parent subject", data["parent_subject"], "c6a96d40895e71642292b0e2c433b13cc58ecd4d")
    exact("topology", data["topology"], "sibling_of_civ_res_000b")
    exact("runtime owner", data["runtime_owner"], "Deferred")
    exact("subject classes", data["subject_classes"], EXPECTED_SUBJECTS)
    exact("artifact kinds", data["artifact_kinds"], EXPECTED_ARTIFACTS)
    exact("opaque references", data["opaque_reference_kinds"], EXPECTED_REFS)
    exact("non-equivalences", data["required_non_equivalences"], EXPECTED_NON_EQUIVALENCES)
    exact("forbidden subjects", data["forbidden_universal_subjects"], EXPECTED_FORBIDDEN_SUBJECTS)
    exact("forbidden authority states", data["forbidden_authority_states"], EXPECTED_FORBIDDEN_STATES)
    exact(
        "evidence primitive policy",
        data["evidence_primitive_policy"],
        "opaque_refs_until_shared_evidence_convergence",
    )
    exact("runtime candidates", data["candidate_runtime_owners"], EXPECTED_RUNTIME_CANDIDATES)
    exact("continuation", data["continuation"], EXPECTED_CONTINUATION)
    exact("nonclaims", data["nonclaims"], EXPECTED_NONCLAIMS)

    if set(EXPECTED_SUBJECTS) & set(EXPECTED_FORBIDDEN_SUBJECTS):
        fail("canonical subject and forbidden-subject sets overlap")
    if set(data["subject_classes"]) & set(data["forbidden_universal_subjects"]):
        fail("person/risk subject leaked into the universal subject vocabulary")
    if set(data["artifact_kinds"]) & set(data["forbidden_authority_states"]):
        fail("authority state leaked into semantic artifact kinds")

    for boundary in EXPECTED_NON_EQUIVALENCES:
        if boundary not in text:
            fail(f"document missing required non-equivalence: {boundary!r}")

    required_phrases = [
        "Johannesburg is one deployment profile",
        "EvidenceRef != evidence valid",
        "MeasurementRef != measurement correct",
        "MethodRef != method qualified",
        "Observation recorded != observation true",
        "observation -> need statement -> intervention proposal -> outcome target",
        "candidate runtime location != selected runtime owner",
        "There is deliberately no `Person`",
        "001A deliberately has no universal priority score",
        "OutcomeObservation != CausalEffectEstimate",
        "exact qualified CIV-RES universal semantics",
        "exact qualified Johannesburg deployment profile",
    ]
    for phrase in required_phrases:
        if phrase not in text:
            fail(f"document missing required semantic boundary: {phrase!r}")

    for artifact in EXPECTED_ARTIFACTS:
        if text.count(artifact) < 1:
            fail(f"artifact kind missing from narrative: {artifact}")

    if "CIV-RES-000B" not in text or "CIV-RES-001A" not in text:
        fail("sibling topology is not visible in the narrative")

    print("CIV-RES-001A PASS: universal civic semantics remain non-personal, non-authorizing, and evidence-waist neutral")
    return 0


if __name__ == "__main__":
    sys.exit(main())
