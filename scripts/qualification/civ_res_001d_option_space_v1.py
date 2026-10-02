#!/usr/bin/env python3
"""Validate CIV-RES-001D reachable option-space semantic contract v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_001D_OPTION_SPACE_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_001d_option_space.json"

EXPECTED_ARTIFACTS = [
    "OptionPathway",
    "OptionAccessEvidence",
    "OptionCapacityObservation",
    "BarrierObservation",
    "ResolutionAttempt",
    "ResolutionOutcomeObservation",
]

EXPECTED_DIMENSIONS = [
    "breadth",
    "accessibility",
    "independence_redundancy",
    "eligibility_constraints",
    "temporal_availability",
    "provider_dependency_concentration",
    "credibility_perceived_availability",
    "scope",
    "currentness",
    "uncertainty",
]

EXPECTED_FORBIDDEN = [
    "PersonRiskScore",
    "SuicideRiskScore",
    "ViolenceRiskScore",
    "CriminalityScore",
    "SocialValueScore",
    "PolicingPriority",
    "ServiceWorthinessScore",
]

def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-001D FAIL: {message}")

def main() -> int:
    if not DOC.is_file(): fail(f"missing document: {DOC}")
    if not MANIFEST.is_file(): fail(f"missing manifest: {MANIFEST}")
    text = DOC.read_text(encoding="utf-8")
    try:
        data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        fail(f"manifest is not valid JSON: {exc}")
    required = {
        "schema","program","parent_subject","topology","runtime_owner",
        "artifact_kinds","option_dimensions","opaque_reference_kinds",
        "required_non_equivalences","forbidden_universal_person_risk_surfaces",
        "forbidden_scalar_models","evidence_primitive_policy","privacy_rule",
        "operational_rule","continuation","nonclaims"
    }
    if set(data) != required: fail("manifest top-level schema drift")
    if data["schema"] != "mycelix.civic-resilience.option-space.v1": fail("schema drift")
    if data["program"] != "CIV-RES-001D": fail("program drift")
    if data["parent_subject"] != "bf4f63099e68bd67c02dc741ac4eff340d321b0b": fail("parent drift")
    if data["topology"] != "child_of_civ_res_001a": fail("topology drift")
    if data["runtime_owner"] != "Deferred": fail("runtime owner became selected")
    if data["artifact_kinds"] != EXPECTED_ARTIFACTS: fail("artifact vocabulary drift")
    if data["option_dimensions"] != EXPECTED_DIMENSIONS: fail("option dimension drift")
    if data["forbidden_universal_person_risk_surfaces"] != EXPECTED_FORBIDDEN: fail("forbidden person-risk surface drift")
    if data["forbidden_scalar_models"] != ["resilience","capability","risk","safety","option_score"]: fail("scalar-model guard drift")
    if data["evidence_primitive_policy"] != "opaque_refs_until_shared_evidence_convergence": fail("evidence ownership policy drift")
    if data["privacy_rule"] != "compose_with_civ_res_001b_release_boundary": fail("privacy composition drift")
    if data["operational_rule"] != "compose_with_sup_civ_owner_lines": fail("operational ownership drift")
    if set(EXPECTED_ARTIFACTS) & set(data["forbidden_scalar_models"]): fail("artifact/scalar collision")
    for artifact in EXPECTED_ARTIFACTS:
        if artifact not in text: fail(f"document missing artifact: {artifact}")
    boundaries = [
        "resource exists != resource reachable",
        "resource reachable != resource usable",
        "stale capacity != current capacity",
        "missing capacity != zero capacity",
        "nominal option count != independent option count",
        "event time != effectivity time",
        "objective availability != perceived availability",
        "aggregate perception trend != individual state",
        "model recommendation != pathway authority",
        "completion != outcome improvement",
        "outcome improvement != causal effect",
        "aggregate option capacity != individual risk",
    ]
    for boundary in boundaries:
        if boundary not in text: fail(f"missing non-equivalence: {boundary!r}")
    required_phrases = [
        "Do not create a universal scalar called `resilience`, `capability`, `risk`, `safety`, or `option_score`.",
        "Universal CIV-RES must not infer a person's perceived option set from aggregate data.",
        "001D does not create another generic provenance or uncertainty stack.",
        "Runtime selection must be an explicit architecture decision.",
        "It does not establish:",
    ]
    for phrase in required_phrases:
        if phrase not in text: fail(f"missing semantic guard: {phrase!r}")
    print("CIV-RES-001D PASS: reachable option-space semantics remain descriptive, scoped, non-person-risk, non-authorizing, and evidence-waist neutral")
    return 0

if __name__ == "__main__":
    sys.exit(main())
