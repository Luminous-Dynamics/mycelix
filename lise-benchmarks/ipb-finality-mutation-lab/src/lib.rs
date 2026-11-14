#![forbid(unsafe_code)]

//! Deterministic mutation suite for the LISE reference relation.
//!
//! Candidate-visible inputs and expected answers are emitted separately.
//! This lab tests the reference model; it is not a production authorizer,
//! hidden confirmatory benchmark, or population safety estimate.

use ipb_finality_reference::{
    evaluate_finality, ApprovalBinding, ApprovalState, AuthorityState, ConstitutionRef,
    CurrentProtectedState, DenialCode, FinalityDecision, FinalityInput, HoldCode,
    OperationBinding, PriorEffectState, ResourceState, RetrySafety,
};

pub const GENERATOR_VERSION: &str = "lise-mutation-lab-v0.1";
pub const SUITE_ID: &str = "LISE-TYPED-MUTATIONS-v0.1";

#[derive(Clone, Debug)]
pub struct MutationCase {
    pub id: String,
    pub family: &'static str,
    pub mutation: &'static str,
    pub is_control: bool,
    pub input: FinalityInput,
    pub expected: FinalityDecision,
}

#[derive(Clone, Debug)]
pub struct MutationObservation {
    pub id: String,
    pub family: &'static str,
    pub mutation: &'static str,
    pub is_control: bool,
    pub expected: FinalityDecision,
    pub observed: FinalityDecision,
    pub killed: bool,
}

#[derive(Clone, Debug)]
pub struct MutationReport {
    pub observations: Vec<MutationObservation>,
    pub control_count: usize,
    pub mutant_count: usize,
    pub killed_count: usize,
    pub survivor_count: usize,
    pub false_permit_count: usize,
}

fn operation() -> OperationBinding {
    OperationBinding {
        logical_operation_id: "op-001".to_owned(),
        canonical_operation_digest: "fixture:operation-digest-001".to_owned(),
        action_digest: "fixture:action-digest-transfer".to_owned(),
        target_ref: "asset:treasury-a".to_owned(),
        destination_or_audience: Some("party-b".to_owned()),
        amount_minor: Some(10_000),
        currency: Some("USD".to_owned()),
        purpose: Some("approved-grant".to_owned()),
        resource_revision: Some("revision-7".to_owned()),
        idempotency_ref: Some("idem-001".to_owned()),
    }
}

fn constitution() -> ConstitutionRef {
    ConstitutionRef {
        institution_ref: "institution-a".to_owned(),
        epoch: 7,
        constitution_digest: "fixture:constitution-digest-7".to_owned(),
    }
}

pub fn baseline_input() -> FinalityInput {
    FinalityInput {
        approval: ApprovalBinding {
            operation: operation(),
            constitution: constitution(),
            policy_revision: "policy-r4".to_owned(),
            state: ApprovalState::Authorized,
        },
        actual_operation: operation(),
        current: CurrentProtectedState {
            constitution: constitution(),
            policy_revision: "policy-r4".to_owned(),
        },
        authority: AuthorityState::Current,
        resource: ResourceState::Matches,
        prior_effect: PriorEffectState::NeverAttempted,
        retry_safety: RetrySafety::Unknown,
    }
}

fn push_case(
    cases: &mut Vec<MutationCase>,
    id: &str,
    family: &'static str,
    mutation: &'static str,
    is_control: bool,
    mutate: impl FnOnce(&mut FinalityInput),
    expected: FinalityDecision,
) {
    let mut input = baseline_input();
    mutate(&mut input);
    cases.push(MutationCase {
        id: id.to_owned(),
        family,
        mutation,
        is_control,
        input,
        expected,
    });
}

/// Stable, deterministic mutation order. No random seed is required because
/// every mutation is explicitly named and applied to the same frozen baseline.
pub fn generate_cases() -> Vec<MutationCase> {
    let mut cases = Vec::new();
    let permit = FinalityDecision::Permit;

    push_case(&mut cases, "C000", "control", "exact-current-operation", true, |_| {}, permit);

    let operation_mutations: [(&str, fn(&mut OperationBinding)); 10] = [
        ("logical-operation-id", |op| op.logical_operation_id.push_str("-substitute")),
        ("canonical-operation-digest", |op| op.canonical_operation_digest.push_str("-substitute")),
        ("action-digest", |op| op.action_digest.push_str("-substitute")),
        ("target-reference", |op| op.target_ref.push_str("-substitute")),
        ("destination-or-audience", |op| op.destination_or_audience = Some("party-c".to_owned())),
        ("amount-minor", |op| op.amount_minor = Some(10_001)),
        ("currency", |op| op.currency = Some("EUR".to_owned())),
        ("purpose", |op| op.purpose = Some("different-purpose".to_owned())),
        ("resource-revision", |op| op.resource_revision = Some("revision-8".to_owned())),
        ("idempotency-reference", |op| op.idempotency_ref = Some("idem-002".to_owned())),
    ];
    for (index, (name, mutate)) in operation_mutations.into_iter().enumerate() {
        push_case(
            &mut cases,
            &format!("O{:03}", index + 1),
            "exact_operation",
            name,
            false,
            move |input| mutate(&mut input.actual_operation),
            FinalityDecision::Deny(DenialCode::OperationMismatch),
        );
    }

    push_case(
        &mut cases, "B001", "binding", "empty-actual-canonical-digest", false,
        |input| input.actual_operation.canonical_operation_digest.clear(),
        FinalityDecision::Hold(HoldCode::MissingRequiredBinding),
    );
    push_case(
        &mut cases, "B002", "binding", "amount-without-currency", false,
        |input| input.actual_operation.currency = None,
        FinalityDecision::Hold(HoldCode::MissingRequiredBinding),
    );
    push_case(
        &mut cases, "B003", "binding", "empty-current-constitution-digest", false,
        |input| input.current.constitution.constitution_digest.clear(),
        FinalityDecision::Hold(HoldCode::MissingRequiredBinding),
    );

    let approval_states = [
        ("denied", ApprovalState::Denied, FinalityDecision::Deny(DenialCode::ApprovalDenied)),
        ("revoked", ApprovalState::Revoked, FinalityDecision::Deny(DenialCode::ApprovalRevoked)),
        ("historical", ApprovalState::Historical, FinalityDecision::Deny(DenialCode::HistoricalApproval)),
        ("pending", ApprovalState::Pending, FinalityDecision::Hold(HoldCode::ApprovalPending)),
        ("unknown", ApprovalState::Unknown, FinalityDecision::Hold(HoldCode::ApprovalUnknown)),
    ];
    for (index, (name, state, expected)) in approval_states.into_iter().enumerate() {
        push_case(
            &mut cases,
            &format!("A{:03}", index + 1),
            "approval_state",
            name,
            false,
            move |input| input.approval.state = state,
            expected,
        );
    }

    let authority_states = [
        ("revoked", AuthorityState::Revoked, FinalityDecision::Deny(DenialCode::AuthorityRevoked)),
        ("expired", AuthorityState::Expired, FinalityDecision::Deny(DenialCode::AuthorityExpired)),
        ("not-yet-valid", AuthorityState::NotYetValid, FinalityDecision::Deny(DenialCode::AuthorityNotYetValid)),
        ("unknown", AuthorityState::Unknown, FinalityDecision::Hold(HoldCode::AuthorityUnknown)),
    ];
    for (index, (name, state, expected)) in authority_states.into_iter().enumerate() {
        push_case(
            &mut cases,
            &format!("H{:03}", index + 1),
            "authority_state",
            name,
            false,
            move |input| input.authority = state,
            expected,
        );
    }

    push_case(
        &mut cases, "C001", "constitution", "epoch-and-digest-change", false,
        |input| {
            input.current.constitution.epoch = 8;
            input.current.constitution.constitution_digest = "fixture:constitution-digest-8".to_owned();
        },
        FinalityDecision::Deny(DenialCode::ConstitutionMismatch),
    );
    push_case(
        &mut cases, "C002", "constitution", "same-epoch-different-digest", false,
        |input| input.current.constitution.constitution_digest = "fixture:substituted-digest".to_owned(),
        FinalityDecision::Deny(DenialCode::ConstitutionMismatch),
    );
    push_case(
        &mut cases, "P001", "policy", "policy-revision-change", false,
        |input| input.current.policy_revision = "policy-r5".to_owned(),
        FinalityDecision::Deny(DenialCode::PolicyRevisionMismatch),
    );
    push_case(
        &mut cases, "R001", "resource", "resource-precondition-mismatch", false,
        |input| input.resource = ResourceState::Mismatch,
        FinalityDecision::Deny(DenialCode::ResourcePreconditionMismatch),
    );
    push_case(
        &mut cases, "R002", "resource", "resource-precondition-unknown", false,
        |input| input.resource = ResourceState::Unknown,
        FinalityDecision::Hold(HoldCode::ResourceStateUnknown),
    );

    push_case(
        &mut cases, "E001", "prior_effect", "unknown-after-dispatch", false,
        |input| input.prior_effect = PriorEffectState::OutcomeUnknownAfterDispatch,
        FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown),
    );
    push_case(
        &mut cases, "E002", "prior_effect", "partially-applied", false,
        |input| input.prior_effect = PriorEffectState::PartiallyApplied,
        FinalityDecision::Hold(HoldCode::PartialOutcomeRequiresReconciliation),
    );
    push_case(
        &mut cases, "E003", "prior_effect", "already-applied", false,
        |input| input.prior_effect = PriorEffectState::ConfirmedApplied,
        FinalityDecision::Deny(DenialCode::EffectAlreadyApplied),
    );
    push_case(
        &mut cases, "E004", "retry", "confirmed-no-effect-with-unknown-safety", false,
        |input| input.prior_effect = PriorEffectState::ConfirmedNotApplied,
        FinalityDecision::Hold(HoldCode::RetrySafetyUnproven),
    );
    push_case(
        &mut cases, "E005", "retry", "confirmed-no-effect-unsupported-safety", false,
        |input| {
            input.prior_effect = PriorEffectState::ConfirmedNotApplied;
            input.retry_safety = RetrySafety::Unsupported;
        },
        FinalityDecision::Hold(HoldCode::RetrySafetyUnproven),
    );
    push_case(
        &mut cases, "C003", "control", "confirmed-no-effect-qualified-same-key-retry", true,
        |input| {
            input.prior_effect = PriorEffectState::ConfirmedNotApplied;
            input.retry_safety = RetrySafety::SameOperationSameIdempotencyKeyQualified;
        },
        permit,
    );
    push_case(
        &mut cases, "E006", "retry", "qualified-safety-without-idempotency-identity", false,
        |input| {
            input.prior_effect = PriorEffectState::ConfirmedNotApplied;
            input.retry_safety = RetrySafety::SameOperationSameIdempotencyKeyQualified;
            input.approval.operation.idempotency_ref = None;
            input.actual_operation.idempotency_ref = None;
        },
        FinalityDecision::Hold(HoldCode::RetrySafetyUnproven),
    );

    push_case(
        &mut cases, "X001", "composition", "operation-substitution-plus-revoked-authority", false,
        |input| {
            input.actual_operation.target_ref = "asset:substituted".to_owned();
            input.authority = AuthorityState::Revoked;
        },
        FinalityDecision::Deny(DenialCode::OperationMismatch),
    );
    push_case(
        &mut cases, "X002", "composition", "unknown-authority-plus-resource-mismatch", false,
        |input| {
            input.authority = AuthorityState::Unknown;
            input.resource = ResourceState::Mismatch;
        },
        FinalityDecision::Hold(HoldCode::AuthorityUnknown),
    );
    push_case(
        &mut cases, "X003", "composition", "unknown-outcome-plus-qualified-retry-safety", false,
        |input| {
            input.prior_effect = PriorEffectState::OutcomeUnknownAfterDispatch;
            input.retry_safety = RetrySafety::SameOperationSameIdempotencyKeyQualified;
        },
        FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown),
    );

    cases
}

impl MutationReport {
    pub fn from_cases(cases: &[MutationCase]) -> Self {
        let observations = cases.iter().map(|case| {
            let observed = evaluate_finality(&case.input);
            MutationObservation {
                id: case.id.clone(),
                family: case.family,
                mutation: case.mutation,
                is_control: case.is_control,
                expected: case.expected,
                observed,
                killed: observed == case.expected,
            }
        }).collect::<Vec<_>>();
        let control_count = observations.iter().filter(|row| row.is_control).count();
        let mutants = observations.iter().filter(|row| !row.is_control).collect::<Vec<_>>();
        let mutant_count = mutants.len();
        let killed_count = mutants.iter().filter(|row| row.killed).count();
        let survivor_count = mutant_count.saturating_sub(killed_count);
        let false_permit_count = mutants.iter()
            .filter(|row| row.expected != FinalityDecision::Permit && row.observed == FinalityDecision::Permit)
            .count();
        Self {
            observations,
            control_count,
            mutant_count,
            killed_count,
            survivor_count,
            false_permit_count,
        }
    }

    pub fn mutation_kill_rate(&self) -> f64 {
        if self.mutant_count == 0 { 1.0 } else { self.killed_count as f64 / self.mutant_count as f64 }
    }

    /// Candidate-visible input manifest. Expected decisions are deliberately omitted.
    pub fn input_manifest_json(cases: &[MutationCase]) -> String {
        let rows = cases.iter().map(|case| format!(
            "{{\"case_id\":{},\"family\":{},\"mutation\":{},\"input\":{}}}",
            quote(&case.id),
            quote(case.family),
            quote(case.mutation),
            input_json(&case.input)
        )).collect::<Vec<_>>().join(",");
        format!(
            "{{\"suite_id\":{},\"generator_version\":{},\"input_only\":true,\"case_count\":{},\"cases\":[{}]}}",
            quote(SUITE_ID), quote(GENERATOR_VERSION), cases.len(), rows
        )
    }

    /// Evaluator-only expected answers. Never send this artifact to a candidate.
    pub fn oracle_json(cases: &[MutationCase]) -> String {
        let rows = cases.iter().map(|case| format!(
            "{{\"case_id\":{},\"expected_decision\":{}}}",
            quote(&case.id), quote(&decision_label(case.expected))
        )).collect::<Vec<_>>().join(",");
        format!(
            "{{\"suite_id\":{},\"generator_version\":{},\"case_count\":{},\"rows\":[{}]}}",
            quote(SUITE_ID), quote(GENERATOR_VERSION), cases.len(), rows
        )
    }

    pub fn report_json(&self) -> String {
        let rows = self.observations.iter().map(|row| format!(
            "{{\"case_id\":{},\"family\":{},\"mutation\":{},\"control\":{},\"expected\":{},\"observed\":{},\"killed\":{}}}",
            quote(&row.id),
            quote(row.family),
            quote(row.mutation),
            row.is_control,
            quote(&decision_label(row.expected)),
            quote(&decision_label(row.observed)),
            row.killed
        )).collect::<Vec<_>>().join(",");
        format!(
            "{{\"suite_id\":{},\"generator_version\":{},\"control_count\":{},\"mutant_count\":{},\"killed_count\":{},\"survivor_count\":{},\"false_permit_count\":{},\"mutation_kill_rate\":{:.6},\"cases\":[{}]}}",
            quote(SUITE_ID), quote(GENERATOR_VERSION), self.control_count, self.mutant_count,
            self.killed_count, self.survivor_count, self.false_permit_count,
            self.mutation_kill_rate(), rows
        )
    }
}

fn quote(value: &str) -> String {
    let mut out = String::from("\"");
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn option_string(value: Option<&str>) -> String {
    value.map(quote).unwrap_or_else(|| "null".to_owned())
}
fn option_number(value: Option<u64>) -> String {
    value.map(|n| n.to_string()).unwrap_or_else(|| "null".to_owned())
}
fn operation_json(op: &OperationBinding) -> String {
    format!(
        "{{\"logical_operation_id\":{},\"canonical_operation_digest\":{},\"action_digest\":{},\"target_ref\":{},\"destination_or_audience\":{},\"amount_minor\":{},\"currency\":{},\"purpose\":{},\"resource_revision\":{},\"idempotency_ref\":{}}}",
        quote(&op.logical_operation_id),
        quote(&op.canonical_operation_digest),
        quote(&op.action_digest),
        quote(&op.target_ref),
        option_string(op.destination_or_audience.as_deref()),
        option_number(op.amount_minor),
        option_string(op.currency.as_deref()),
        option_string(op.purpose.as_deref()),
        option_string(op.resource_revision.as_deref()),
        option_string(op.idempotency_ref.as_deref()),
    )
}
fn constitution_json(value: &ConstitutionRef) -> String {
    format!(
        "{{\"institution_ref\":{},\"epoch\":{},\"constitution_digest\":{}}}",
        quote(&value.institution_ref), value.epoch, quote(&value.constitution_digest)
    )
}
fn approval_state_label(value: ApprovalState) -> &'static str {
    match value {
        ApprovalState::Authorized => "authorized",
        ApprovalState::Denied => "denied",
        ApprovalState::Revoked => "revoked",
        ApprovalState::Historical => "historical",
        ApprovalState::Pending => "pending",
        ApprovalState::Unknown => "unknown",
    }
}
fn authority_state_label(value: AuthorityState) -> &'static str {
    match value {
        AuthorityState::Current => "current",
        AuthorityState::Revoked => "revoked",
        AuthorityState::Expired => "expired",
        AuthorityState::NotYetValid => "not_yet_valid",
        AuthorityState::Unknown => "unknown",
    }
}
fn resource_state_label(value: ResourceState) -> &'static str {
    match value {
        ResourceState::Matches => "matches",
        ResourceState::Mismatch => "mismatch",
        ResourceState::Unknown => "unknown",
        ResourceState::NotRequired => "not_required",
    }
}
fn prior_effect_label(value: PriorEffectState) -> &'static str {
    match value {
        PriorEffectState::NeverAttempted => "never_attempted",
        PriorEffectState::OutcomeUnknownAfterDispatch => "outcome_unknown_after_dispatch",
        PriorEffectState::ConfirmedNotApplied => "confirmed_not_applied",
        PriorEffectState::ConfirmedApplied => "confirmed_applied",
        PriorEffectState::PartiallyApplied => "partially_applied",
    }
}
fn retry_safety_label(value: RetrySafety) -> &'static str {
    match value {
        RetrySafety::SameOperationSameIdempotencyKeyQualified => "same_operation_same_idempotency_key_qualified",
        RetrySafety::Unsupported => "unsupported",
        RetrySafety::Unknown => "unknown",
    }
}
fn decision_label(value: FinalityDecision) -> String {
    format!("{value:?}")
}
fn input_json(input: &FinalityInput) -> String {
    format!(
        "{{\"approval\":{{\"operation\":{},\"constitution\":{},\"policy_revision\":{},\"state\":{}}},\"actual_operation\":{},\"current\":{{\"constitution\":{},\"policy_revision\":{}}},\"authority\":{},\"resource\":{},\"prior_effect\":{},\"retry_safety\":{}}}",
        operation_json(&input.approval.operation),
        constitution_json(&input.approval.constitution),
        quote(&input.approval.policy_revision),
        quote(approval_state_label(input.approval.state)),
        operation_json(&input.actual_operation),
        constitution_json(&input.current.constitution),
        quote(&input.current.policy_revision),
        quote(authority_state_label(input.authority)),
        quote(resource_state_label(input.resource)),
        quote(prior_effect_label(input.prior_effect)),
        quote(retry_safety_label(input.retry_safety))
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generator_is_deterministic_and_ids_are_unique() {
        let a = generate_cases();
        let b = generate_cases();
        assert_eq!(a.len(), b.len());
        assert_eq!(a.iter().map(|c| &c.id).collect::<Vec<_>>(), b.iter().map(|c| &c.id).collect::<Vec<_>>());
        let ids = a.iter().map(|case| case.id.as_str()).collect::<std::collections::BTreeSet<_>>();
        assert_eq!(ids.len(), a.len());
    }

    #[test]
    fn baseline_controls_are_permitted() {
        for case in generate_cases().into_iter().filter(|case| case.is_control) {
            assert_eq!(evaluate_finality(&case.input), FinalityDecision::Permit, "{}", case.id);
            assert_eq!(case.expected, FinalityDecision::Permit);
        }
    }

    #[test]
    fn every_named_mutant_is_killed_by_the_reference() {
        let cases = generate_cases();
        let report = MutationReport::from_cases(&cases);
        assert_eq!(report.survivor_count, 0);
        assert_eq!(report.false_permit_count, 0);
        assert_eq!(report.killed_count, report.mutant_count);
    }

    #[test]
    fn candidate_input_manifest_never_contains_oracle_decisions() {
        let cases = generate_cases();
        let input = MutationReport::input_manifest_json(&cases);
        assert!(input.contains("\"input_only\":true"));
        assert!(!input.contains("expected_decision"));
        assert!(!input.contains("Deny("));
        assert!(!input.contains("Hold("));
        let oracle = MutationReport::oracle_json(&cases);
        assert!(oracle.contains("expected_decision"));
    }

    #[test]
    fn generated_cases_include_all_exact_operation_dimensions() {
        let cases = generate_cases();
        for dimension in [
            "logical-operation-id",
            "canonical-operation-digest",
            "action-digest",
            "target-reference",
            "destination-or-audience",
            "amount-minor",
            "currency",
            "purpose",
            "resource-revision",
            "idempotency-reference",
        ] {
            assert!(cases.iter().any(|case| case.mutation == dimension));
        }
    }

    #[test]
    fn report_has_stable_denominators_and_distinguishes_controls() {
        let cases = generate_cases();
        let report = MutationReport::from_cases(&cases);
        assert!(report.control_count >= 1);
        assert!(report.mutant_count >= 30);
        assert_eq!(report.killed_count + report.survivor_count, report.mutant_count);
        assert_eq!(report.mutation_kill_rate(), 1.0);
    }
}
