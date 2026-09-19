#!/usr/bin/env python3
"""Validate CIV-RES-001B privacy projection/accountability composition v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_001B_PRIVACY_PROJECTION_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_001b_privacy_projection.json"

EXPECTED = {
    "existing_external_owners": [
        "AccessReceipt", "SubjectNotice", "QueryBudgetCharge", "SubjectRights", "NotificationDirective"
    ],
    "projection_classes": ["PublicAggregate", "CommunityScopedAggregate", "ResearchOutput"],
    "artifact_kinds": ["CivicProjectionRequest", "CivicProjectionPlan", "CivicProjectionEvidence"],
    "required_lineage_refs": [
        "SourceSnapshotRef", "SourceSensitivityRef", "PurposeRef", "ProjectionPolicyRef",
        "TransformationRef", "GeographicPrecisionRef", "TemporalPrecisionRef",
        "SuppressionOrThresholdPolicyRef", "AccessReceiptRef", "ReleaseBudgetRef",
        "ReleaseHistoryRef", "DisclosureReviewRef", "ReleaseDecisionRef", "OutputCommitmentRef"
    ],
    "required_non_equivalences": [
        "ProtectedAccess != PublicRelease",
        "AccessReceipt != PermissionGrant",
        "AccessReceipt != PublicationAuthority",
        "SubjectNotice != PublicProjection",
        "Aggregate != Anonymous",
        "Coarsened != Safe",
        "Redacted != Deidentified",
        "NoDirectIdentifier != NonIdentifiable",
        "OneSafeRelease != SafeReleaseSequence",
        "QueryAccessBudget != PublicReleaseBudget",
        "ValidAccess != SafeAggregate",
        "ValidAggregateTransformation != LegitimateAccess",
        "AnalysisCompleted != OutputReleasable",
        "StatisticalResult != DisclosureSafeResult",
        "ModelOutput != AnonymousOutput",
        "ProjectionEvidence != ReleaseAuthority",
        "ReleaseDecisionRef != ReleaseDecisionValid",
        "PublicProjection != SourceTruth",
    ],
    "mosaic_requirements": [
        "release_context_must_include_prior_release_history_when_policy_requires",
        "required_release_history_unavailable_fails_closed",
        "auxiliary_information_risk_must_not_be_assumed_zero",
        "single_release_safety_does_not_imply_sequence_safety",
    ],
    "externalized_policy_parameters": [
        "minimum_cohort_size", "geographic_cell_size", "time_delay", "retention_period",
        "differential_privacy_epsilon", "suppression_threshold", "query_budget", "release_budget"
    ],
    "forbidden_projection_classes": ["SubjectNotice", "FullPersonRecord", "ExactProtectedLocationPublic"],
    "continuation": ["CIV-RES-001C", "CIV-RES-002A", "CIV-RES-002C", "SYM-CIVIC-000A"],
    "nonclaims": [
        "runtime_release_gate", "accountability_core_qualification", "anonymity", "deidentification",
        "reidentification_resistance", "safe_threshold", "safe_privacy_budget", "legal_permission",
        "popia_compliance", "publication_authority", "johannesburg_deployment", "deployment_readiness"
    ],
}

EXPECTED_TOP_LEVEL = {
    "schema", "program", "parent_subject", "runtime_owner", "architectural_dependencies",
    *EXPECTED.keys(),
}


def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-001B FAIL: {message}")


def exact(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{name} drifted from the canonical projection contract")


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

    exact("schema", data["schema"], "mycelix.civic-resilience.privacy-projection.v1")
    exact("program", data["program"], "CIV-RES-001B")
    exact("parent subject", data["parent_subject"], "bf4f63099e68bd67c02dc741ac4eff340d321b0b")
    exact("runtime owner", data["runtime_owner"], "Deferred")

    deps = [{
        "owner": "mycelix-accountability-core",
        "reference": "PR #28",
        "dependency_state": "architectural_reference_only",
    }]
    exact("architectural dependency", data["architectural_dependencies"], deps)

    for key, expected in EXPECTED.items():
        exact(key, data[key], expected)

    if set(data["projection_classes"]) & set(data["forbidden_projection_classes"]):
        fail("forbidden projection class entered universal vocabulary")
    if "SubjectNotice" not in data["existing_external_owners"]:
        fail("subject-facing notice ownership must remain external")
    if "SubjectNotice" in data["projection_classes"]:
        fail("CIV-RES must not duplicate SubjectNotice")

    for boundary in EXPECTED["required_non_equivalences"]:
        if boundary not in text:
            fail(f"document missing required non-equivalence: {boundary!r}")

    required_phrases = [
        "required release history unavailable -> fail closed",
        "history unavailable -> assume current release is isolated",
        "There is deliberately no `SubjectNotice` projection class",
        "CIV-RES-001B deliberately does not define the numeric release budget itself",
        "The universal protocol does not freeze one value for any of these",
        "architectural composition contract != runtime dependency convergence",
        "Johannesburg policy value -> universal privacy constant",
        "valid access receipt != safe aggregate",
        "valid aggregate transformation != legitimate access",
    ]
    for phrase in required_phrases:
        if phrase not in text:
            fail(f"document missing required projection boundary: {phrase!r}")

    # Numeric privacy choices must remain named external policy parameters rather
    # than universal values. Their exact census is already checked above; do not
    # infer safety from prose formatting (e.g. hyphenated 'differential-privacy').
    if data["runtime_owner"] != "Deferred":
        fail("runtime ownership was selected before dependency convergence")

    print("CIV-RES-001B PASS: access, projection, mosaic, and release-authority boundaries preserved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
