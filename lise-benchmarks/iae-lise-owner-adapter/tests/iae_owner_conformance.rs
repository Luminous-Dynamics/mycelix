use ipb_finality_reference::{
    evaluate_finality, ApprovalBinding, ApprovalState, AuthorityState, ConstitutionRef,
    CurrentProtectedState, DenialCode, FinalityDecision, FinalityInput, HoldCode,
    OperationBinding, PriorEffectState, ResourceState, RetrySafety,
};
use serde_json::Value;

const PACKET: &str = include_str!("../fixtures/iae-v0.2-finality-projection.json");

fn packet() -> Value {
    serde_json::from_str(PACKET).expect("IAE projection fixture parses")
}

fn required_string<'a>(value: &'a Value, field: &str) -> &'a str {
    value[field].as_str().unwrap_or_else(|| panic!("{field} must be a string"))
}

fn operation(value: &Value) -> OperationBinding {
    OperationBinding {
        logical_operation_id: required_string(value, "logical_operation_id").to_owned(),
        canonical_operation_digest: required_string(value, "canonical_operation_digest").to_owned(),
        action_digest: required_string(value, "action_digest").to_owned(),
        target_ref: required_string(value, "target_ref").to_owned(),
        destination_or_audience: value["destination_or_audience"].as_str().map(str::to_owned),
        amount_minor: value["amount_minor"].as_u64(),
        currency: value["currency"].as_str().map(str::to_owned),
        purpose: value["purpose"].as_str().map(str::to_owned),
        resource_revision: value["resource_revision"].as_str().map(str::to_owned),
        idempotency_ref: value["idempotency_ref"].as_str().map(str::to_owned),
    }
}

fn constitution(value: &Value) -> ConstitutionRef {
    ConstitutionRef {
        institution_ref: required_string(value, "institution_ref").to_owned(),
        epoch: value["epoch"].as_u64().expect("constitution epoch is unsigned"),
        constitution_digest: required_string(value, "constitution_digest").to_owned(),
    }
}

fn approval_state(value: &Value) -> ApprovalState {
    match required_string(value, "approval_state") {
        "authorized" => ApprovalState::Authorized,
        "denied" => ApprovalState::Denied,
        "revoked" => ApprovalState::Revoked,
        "historical" => ApprovalState::Historical,
        "pending" => ApprovalState::Pending,
        "unknown" => ApprovalState::Unknown,
        other => panic!("unsupported IAE approval state: {other}"),
    }
}

fn base_input(packet: &Value) -> FinalityInput {
    let binding = &packet["operation_finality"];
    let approved = operation(&binding["approved_operation"]);
    let pending = operation(&binding["pending_operation"]);
    let constitution_ref = constitution(&binding["constitution"]);
    let policy_revision = required_string(binding, "policy_revision").to_owned();
    FinalityInput {
        approval: ApprovalBinding {
            operation: approved,
            constitution: constitution_ref.clone(),
            policy_revision: policy_revision.clone(),
            state: approval_state(binding),
        },
        actual_operation: pending,
        current: CurrentProtectedState {
            constitution: constitution_ref,
            policy_revision,
        },
        authority: AuthorityState::Current,
        resource: ResourceState::Matches,
        prior_effect: PriorEffectState::NeverAttempted,
        retry_safety: RetrySafety::Unknown,
    }
}

#[test]
fn intent_baseline_acceptance_is_not_operation_approval() {
    let p = packet();
    assert_ne!(
        required_string(&p["intent_baseline_approval"], "approval_ref"),
        required_string(&p["operation_finality"], "approval_ref")
    );
}

#[test]
fn iae_exact_operation_projection_maps_to_lise_original_preflight_permit() {
    let p = packet();
    let mut input = base_input(&p);
    assert_eq!(
        input.approval.operation,
        input.actual_operation,
        "the approved operation projection must equal the pending operation"
    );
    assert_eq!(evaluate_finality(&input), FinalityDecision::Permit);
}

#[test]
fn possible_external_effect_remains_unknown_and_requires_reconciliation() {
    let p = packet();
    let mut input = base_input(&p);
    input.prior_effect = PriorEffectState::OutcomeUnknownAfterDispatch;
    assert_eq!(
        evaluate_finality(&input),
        FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown)
    );
}

#[test]
fn later_revocation_denies_reattempt_without_resolving_original_unknown() {
    let p = packet();
    let mut original = base_input(&p);
    original.prior_effect = PriorEffectState::OutcomeUnknownAfterDispatch;
    assert_eq!(
        evaluate_finality(&original),
        FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown)
    );

    let mut reattempt = original;
    reattempt.authority = AuthorityState::Revoked;
    assert_eq!(
        evaluate_finality(&reattempt),
        FinalityDecision::Deny(DenialCode::AuthorityRevoked)
    );

    assert_eq!(
        required_string(&p["operation_finality"], "prior_effect_after_timeout"),
        "outcome_unknown_after_dispatch"
    );
    assert_eq!(
        required_string(&p["operation_finality"], "expected_reattempt_disposition"),
        "DENY"
    );
    assert_eq!(p["execution_control"]["retry_count"].as_u64(), Some(0));
    assert_eq!(p["execution_control"]["reattempt_permitted"].as_bool(), Some(false));
}

#[test]
fn temporal_and_effect_lineage_supports_separate_original_and_retry_decisions() {
    let p = packet();
    let binding = &p["operation_finality"];
    let approval_time = required_string(binding, "approved_at");
    let action_time = required_string(binding, "action_at_before_dispatch");
    let knowledge_time = required_string(binding, "known_at_before_dispatch");
    let submit = p["operation_history"]
        .as_array().unwrap()
        .iter().find(|e| e["event_type"] == "submit_attempt").unwrap();
    let timeout = p["operation_history"]
        .as_array().unwrap()
        .iter().find(|e| e["event_type"] == "timeout").unwrap();
    let revoke = p["authority"]["status_events"]
        .as_array().unwrap()
        .iter().find(|e| e["status"] == "revoked").unwrap();

    assert!(approval_time <= action_time);
    assert!(knowledge_time <= action_time);
    assert_eq!(action_time, required_string(submit, "occurred_at"));
    assert!(required_string(timeout, "occurred_at") > action_time);
    assert!(required_string(revoke, "effective_at") < required_string(binding, "reattempt_action_at"));
    assert!(required_string(revoke, "observed_at") < required_string(binding, "reattempt_known_at"));
    assert_eq!(
        required_string(binding, "authority_state_after_timeout"),
        "revoked"
    );
}
