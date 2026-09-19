#!/usr/bin/env python3
"""Validate CIV-RES-001C completion evidence / verification separation v1."""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "CIV_RES_001C_COMPLETION_VERIFICATION_V1.md"
MANIFEST = ROOT / "mycelix-workspace" / "docs" / "civic-resilience" / "civ_res_001c_completion_verification.json"

EXPECTED_TOP_LEVEL = {
    "schema",
    "program",
    "parent_subject",
    "runtime_owner",
    "architectural_dependencies",
    "artifact_kinds",
    "required_reference_kinds",
    "required_non_equivalences",
    "forbidden_caller_selected_positive_fields",
    "verification_scope_rules",
    "adversarial_case_ids",
    "continuation",
    "nonclaims",
}
EXPECTED_DEPS = [
    {"owner":"planetary_response","reference":"PR #483","dependency_state":"architectural_reference_only"},
    {"owner":"planetary_outcome","reference":"PR #484","dependency_state":"architectural_reference_only"},
    {"owner":"mycelix-business-core","reference":"PR #165","dependency_state":"architectural_reference_only"},
]
EXPECTED_ARTIFACTS = [
    "CompletionClaim",
    "CompletionEvidenceBundle",
    "CompletionVerificationEvidence",
]
EXPECTED_REFS = [
    "ExternalCommitmentRef",
    "EvidenceRef",
    "CollectionSourceRef",
    "TimeEvidenceRef",
    "LocationScopeRef",
    "MethodRef",
    "VerificationPredicateRef",
    "VerificationMethodRef",
    "VerifierQualificationRef",
    "VerifierIndependenceProfileRef",
    "InputEvidenceCutRef",
    "VerificationResultRef",
    "OutcomeObservationRef",
    "ExecutionReceiptRef",
    "AuthorityDecisionRef",
]
EXPECTED_NON_EQUIVALENCES = [
    "Commitment != AuthorityToExecute",
    "Commitment != CompletionClaim",
    "CompletionClaim != CompletionEvidence",
    "CompletionEvidence != CompletionVerification",
    "CompletionVerification != OutcomeObservation",
    "VerifiedCompletion != OutcomeImprovement",
    "OutcomeObservation != CausalEffect",
    "ExecutionReceipt != CompletionVerification",
    "ProviderAcknowledgement != Completion",
    "Payment != Completion",
    "PhotoOrDocument != Completion",
    "DifferentVerifierId != IndependentVerification",
    "Verification != BeneficiaryAcceptance",
    "Verification != QualityBeyondVerifiedPredicate",
    "CompletionVerification != PaymentAuthority",
    "VerifiedCompletion != PaymentDue",
]
EXPECTED_FORBIDDEN_FIELDS = [
    "verified",
    "official",
    "paid",
    "accepted",
    "successful",
    "outcome_improved",
    "independent",
]
EXPECTED_SCOPE_RULES = [
    "verification_establishes_only_exact_predicate",
    "verifier_identity_difference_does_not_establish_independence",
    "execution_receipt_may_be_evidence_but_not_completion_verification",
    "payment_may_be_evidence_but_not_completion",
    "outcome_observation_remains_separate_from_completion_verification",
    "causal_attribution_remains_external",
]
EXPECTED_ATTACKS = [
    "CIV-COMP-T-001_WRONG_COMMITMENT_REVISION",
    "CIV-COMP-T-002_REUSED_EVIDENCE",
    "CIV-COMP-T-003_OUT_OF_SCOPE_EVIDENCE",
    "CIV-COMP-T-004_PROVIDER_ACK_WITHOUT_RESULT",
    "CIV-COMP-T-005_PAYMENT_WITHOUT_COMPLETION",
    "CIV-COMP-T-006_EXECUTION_RECEIPT_PREDICATE_MISMATCH",
    "CIV-COMP-T-007_CLAIMANT_VERIFIER_CONFLICT",
    "CIV-COMP-T-008_SHARED_VERIFIER_FAULT_DOMAIN",
    "CIV-COMP-T-009_PREDICATE_UI_OVERCLAIM",
    "CIV-COMP-T-010_LATER_CONTRADICTORY_EVIDENCE",
    "CIV-COMP-T-011_COMPLETED_BUT_OUTCOME_WORSE",
    "CIV-COMP-T-012_OUTCOME_IMPROVES_WITHOUT_ATTRIBUTION",
    "CIV-COMP-T-013_STALE_VERIFICATION_PRESENTED_CURRENT",
]
EXPECTED_CONTINUATION = ["CIV-RES-002A", "CIV-RES-002B", "CIV-RES-002C", "SYM-CIVIC-000A"]
EXPECTED_NONCLAIMS = [
    "runtime_commitment_registry",
    "obligation_engine",
    "execution_authority",
    "evidence_authenticity",
    "completion_truth",
    "verifier_independence",
    "service_quality",
    "beneficiary_acceptance",
    "payment_authority",
    "outcome_improvement",
    "causal_effect",
    "municipal_legitimacy",
    "johannesburg_deployment",
    "deployment_readiness",
]


def fail(message: str) -> None:
    raise SystemExit(f"CIV-RES-001C FAIL: {message}")


def exact(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{name} drifted from the canonical completion contract")


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

    exact("schema", data["schema"], "mycelix.civic-resilience.completion-verification.v1")
    exact("program", data["program"], "CIV-RES-001C")
    exact("parent subject", data["parent_subject"], "c7b6f3ac3113d2eba83b01eeb12ab66c3598ee9f")
    exact("runtime owner", data["runtime_owner"], "Deferred")
    exact("dependencies", data["architectural_dependencies"], EXPECTED_DEPS)
    exact("artifact kinds", data["artifact_kinds"], EXPECTED_ARTIFACTS)
    exact("reference kinds", data["required_reference_kinds"], EXPECTED_REFS)
    exact("non-equivalences", data["required_non_equivalences"], EXPECTED_NON_EQUIVALENCES)
    exact("forbidden caller fields", data["forbidden_caller_selected_positive_fields"], EXPECTED_FORBIDDEN_FIELDS)
    exact("scope rules", data["verification_scope_rules"], EXPECTED_SCOPE_RULES)
    exact("adversarial cases", data["adversarial_case_ids"], EXPECTED_ATTACKS)
    exact("continuation", data["continuation"], EXPECTED_CONTINUATION)
    exact("nonclaims", data["nonclaims"], EXPECTED_NONCLAIMS)

    for dep in data["architectural_dependencies"]:
        if dep["dependency_state"] != "architectural_reference_only":
            fail(f"dependency was silently promoted: {dep['owner']}")

    for boundary in EXPECTED_NON_EQUIVALENCES:
        if boundary not in text:
            fail(f"document missing required non-equivalence: {boundary!r}")

    required_phrases = [
        "There is deliberately no universal `completed=true` or `independent=true` field.",
        "DifferentVerifierId != IndependentVerification",
        "A photo, document, provider acknowledgement, payment record, sensor reading, external reference, resident report, or execution receipt may be evidence. None is a universal completion oracle.",
        "verified completion + better outcome != causal effect",
        "opaque ref != referenced proposition valid",
        "CIV-RES-001C defines no scalar completion confidence",
        "ExecutionReceipt != CompletionVerification",
        "CompletionVerification != PaymentAuthority",
    ]
    for phrase in required_phrases:
        if phrase not in text:
            fail(f"document missing required completion boundary: {phrase!r}")

    for i in range(1, 14):
        marker = f"CIV-COMP-T-{i:03d}"
        if not any(item.startswith(marker) for item in EXPECTED_ATTACKS):
            fail(f"internal adversarial census construction error: {marker}")

    if data["runtime_owner"] != "Deferred":
        fail("runtime ownership was selected before dependency convergence")

    print("CIV-RES-001C PASS: commitment, completion evidence, scoped verification, and outcome boundaries preserved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
