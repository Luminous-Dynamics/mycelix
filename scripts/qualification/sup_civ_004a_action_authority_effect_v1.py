#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sup_civ_004a_action_authority_effect.json"
DOC = ROOT / "mycelix-workspace/docs/civic-resilience/SUP_CIV_004A_ACTION_AUTHORITY_EFFECT_V1.md"

d = json.loads(MANIFEST.read_text())
doc = DOC.read_text()

assert d["schema"] == "mycelix.support-civic.action-authority-effect.v1"
assert d["program"] == "SUP-CIV-004A"
assert d["parent_subject"] == "a85369699099d4c7524e502e531735eed4ab36f4"
assert d["runtime_owner"] == "Deferred"
assert d["artifact_vocabulary"] == [
    "SupportActionProposal",
    "SupportActionAuthorization",
    "SupportExecutionRequest",
    "SupportExecutionReceipt",
    "SupportEffectObservation",
    "SupportRollbackRequest",
    "SupportRollbackReceipt",
    "LegacySupportActionState",
]
for key in [
    "proposal_requirement_is_approval_authority",
    "proposal_requirement_is_execution_authority",
    "legacy_approved_is_institutional_authorization",
    "legacy_executed_is_external_effect",
    "legacy_success_is_outcome_effect",
    "full_autonomy_is_civic_authority",
]:
    assert d[key] is False

required = {
    "civic_requirement_proposal() != AuthorityToApprove",
    "civic_requirement_proposal() != AuthorityToExecute",
    "ActionProposal != AuthorityToExecute",
    "LegacyApprovedBool != InstitutionalAuthorization",
    "ExecutionRequest != ExecutionOccurred",
    "LegacyExecutedBool != ConfirmedExternalEffect",
    "ExecutionReceipt != CompletionVerification",
    "ExecutionReceipt != OutcomeEffect",
    "SupportEffectObservation != CivicCompletionVerification",
    "LegacySuccessBool != OutcomeEffect",
    "AutonomyLevel::FullAutonomous != CivicAuthority",
    "PredictionConfidence != IncidentTruth",
    "LegacyRolledBackBool != RollbackVerified",
    "RollbackReceipt != OriginalEffectErased",
    "SameAuthorizedRequestReplayed != NewAuthorization",
}
assert required.issubset(set(d["non_equivalences"]))

expected_refusals = {
    "proposal_eligibility_as_authorization",
    "authorization_action_mismatch",
    "authorization_target_mismatch",
    "expired_or_revoked_authorization",
    "executor_mismatch",
    "parameters_outside_authorized_scope",
    "duplicate_request_without_idempotency_policy",
    "execution_receipt_as_effect_success_without_observation",
    "legacy_success_as_outcome_improvement",
    "rollback_request_as_verified_rollback",
    "partial_or_failed_rollback_coerced_to_success",
    "full_autonomous_bypasses_authorization_scope",
    "prediction_confidence_as_incident_truth",
    "legacy_booleans_upgraded_without_evidence",
}
assert set(d["required_runtime_refusals"]) == expected_refusals

for phrase in [
    "civic_requirement_proposal() != AuthorityToApprove",
    "civic_requirement_proposal() != AuthorityToExecute",
    "ExecutionReceipt != CompletionVerification",
    "AutonomyLevel::FullAutonomous != CivicAuthority",
    "PredictionConfidence != IncidentTruth",
    "RollbackReceipt != OriginalEffectErased",
]:
    assert phrase in doc, phrase

print("SUP-CIV-004A semantic contract: PASS")
