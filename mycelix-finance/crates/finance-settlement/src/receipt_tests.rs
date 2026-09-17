use std::collections::BTreeSet;

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::{AssetAmount, AssetId};

use super::*;

fn reference(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static reference")
}

fn attempt(value: &str) -> ExecutionAttemptRef {
    ExecutionAttemptRef::new(value).expect("static attempt")
}

fn amount(units: u64) -> AssetAmount {
    AssetAmount::new(
        units,
        AssetId::new("USD.micro").expect("static asset identifier"),
    )
}

fn effect() -> Digest32 {
    Digest32::repeat(5)
}

fn profile() -> FinalityProfile {
    FinalityProfile::new(
        reference("finality:bank:v1"),
        1,
        reference("rail:bank"),
        reference("network:test-bank"),
        BTreeSet::from([reference("evidence:provider-settlement")]),
        1,
        60_000,
        ReversalModel::MayReverse,
    )
    .expect("valid profile")
}

fn context(class: EvaluationContextClass, time: u64) -> SettlementEvaluationContext {
    match class {
        EvaluationContextClass::DeterministicSupplied => {
            SettlementEvaluationContext::deterministic_supplied(
                time,
                reference("clock:test"),
                1,
                Digest32::repeat(4),
            )
        }
        EvaluationContextClass::HistoricalReplay => SettlementEvaluationContext::historical_replay(
            time,
            reference("clock:test"),
            1,
            Digest32::repeat(4),
        ),
    }
    .expect("valid context")
}

fn subject(profile: &FinalityProfile) -> SettlementSubject {
    SettlementSubject {
        id: reference("settlement:subject:1"),
        financial_effect_commitment: effect(),
        attempt: attempt("attempt:1"),
        rail: profile.rail().clone(),
        network: profile.network().clone(),
        amount: amount(100),
        required_profile: profile.profile_ref().clone(),
    }
}

fn observation(
    operation: &str,
    revision: u64,
    units: u64,
    state: ObservedSettlementState,
    observed_at: u64,
) -> SettlementObservation {
    let profile = profile();
    let subject = subject(&profile);
    let observation_id = reference(format!("observation:{operation}:{revision}").as_str());
    SettlementObservation {
        observation_id: observation_id.clone(),
        subject: subject.id,
        financial_effect_commitment: subject.financial_effect_commitment,
        attempt: subject.attempt,
        rail: subject.rail,
        network: subject.network,
        operation_id: reference(operation),
        revision,
        amount: amount(units),
        state,
        observed_at_unix_ms: observed_at,
        evidence: vec![FinalityEvidence {
            evidence_id: reference(format!("evidence:{operation}:{revision}:provider-a").as_str()),
            subject: reference("settlement:subject:1"),
            operation_id: reference(operation),
            operation_revision: revision,
            observation_id,
            kind: reference("evidence:provider-settlement"),
            source: reference("source:provider-a"),
            digest: Digest32::repeat(7),
        }],
    }
}

#[test]
fn receipt_preserves_non_applied_current_observation_commitments() {
    let profile = profile();
    let subject = subject(&profile);
    let context = context(EvaluationContextClass::DeterministicSupplied, 1_100);
    let applied = observation(
        "op:applied",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
    );
    let rejected = observation(
        "op:rejected",
        1,
        10,
        ObservedSettlementState::Rejected,
        1_010,
    );

    let receipt =
        qualify_settlement_with_receipt(&subject, &profile, &[applied, rejected], &context)
            .expect("qualification receipt");

    assert_eq!(receipt.settlement().operations().len(), 1);
    assert_eq!(receipt.selected_observation_commitments().len(), 2);
}

#[test]
fn exact_duplicate_observation_multiplicity_does_not_change_receipt_selection() {
    let profile = profile();
    let subject = subject(&profile);
    let context = context(EvaluationContextClass::DeterministicSupplied, 1_100);
    let current = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);

    let single = qualify_settlement_with_receipt(
        &subject,
        &profile,
        std::slice::from_ref(&current),
        &context,
    )
    .expect("single observation");
    let duplicate =
        qualify_settlement_with_receipt(&subject, &profile, &[current.clone(), current], &context)
            .expect("exact duplicate observation");

    assert_eq!(
        single.selected_observation_commitments(),
        duplicate.selected_observation_commitments()
    );
    assert_eq!(
        single.settlement().evidence_frontier(),
        duplicate.settlement().evidence_frontier()
    );
}

#[test]
fn invalidation_receipt_preserves_temporal_context_class_and_commitment() {
    let profile = profile();
    let subject = subject(&profile);
    let prior_context = context(EvaluationContextClass::DeterministicSupplied, 1_100);
    let prior = qualify_settlement(
        &subject,
        &profile,
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &prior_context,
    )
    .expect("prior settlement");
    let reversal = observation("op:1", 2, 100, ObservedSettlementState::Reversed, 1_200);
    let deterministic = context(EvaluationContextClass::DeterministicSupplied, 1_250);
    let historical = context(EvaluationContextClass::HistoricalReplay, 1_250);

    let deterministic_receipt =
        derive_invalidation_with_receipt(&prior, &profile, &reversal, &deterministic)
            .expect("deterministic invalidation receipt");
    let historical_receipt =
        derive_invalidation_with_receipt(&prior, &profile, &reversal, &historical)
            .expect("historical invalidation receipt");

    assert_eq!(
        deterministic_receipt.evaluation_context_class(),
        EvaluationContextClass::DeterministicSupplied
    );
    assert_eq!(
        historical_receipt.evaluation_context_class(),
        EvaluationContextClass::HistoricalReplay
    );
    assert_ne!(
        deterministic_receipt.evaluation_context_commitment(),
        historical_receipt.evaluation_context_commitment()
    );
    assert_ne!(
        deterministic_receipt.receipt_commitment(),
        historical_receipt.receipt_commitment()
    );
}

#[test]
fn invalidation_receipt_commitment_reconstructs_from_sealed_fields() {
    let profile = profile();
    let subject = subject(&profile);
    let prior = qualify_settlement(
        &subject,
        &profile,
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &context(EvaluationContextClass::DeterministicSupplied, 1_100),
    )
    .expect("prior settlement");
    let later_context = context(EvaluationContextClass::DeterministicSupplied, 1_250);
    let receipt = derive_invalidation_with_receipt(
        &prior,
        &profile,
        &observation("op:1", 2, 100, ObservedSettlementState::Reversed, 1_200),
        &later_context,
    )
    .expect("invalidation receipt");

    let reconstructed = settlement_invalidation_receipt_commitment(
        receipt.invalidation(),
        receipt.evaluation_context_commitment(),
        receipt.evaluation_context_class(),
    )
    .expect("reconstructed commitment");
    assert_eq!(reconstructed, receipt.receipt_commitment());
}
