use std::collections::BTreeSet;
use std::panic::{AssertUnwindSafe, catch_unwind};

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

fn context(time: u64) -> SettlementEvaluationContext {
    SettlementEvaluationContext::deterministic_supplied(
        time,
        reference("clock:panic-safety:test"),
        1,
        Digest32::repeat(4),
    )
    .expect("valid context")
}

fn subject(profile: &FinalityProfile) -> SettlementSubject {
    SettlementSubject {
        id: reference("settlement:subject:panic-safety"),
        financial_effect_commitment: Digest32::repeat(5),
        attempt: attempt("attempt:panic-safety"),
        rail: profile.rail().clone(),
        network: profile.network().clone(),
        amount: amount(100),
        required_profile: profile.profile_ref().clone(),
    }
}

fn observation(
    subject: &SettlementSubject,
    operation: &str,
    revision: u64,
    units: u64,
    state: ObservedSettlementState,
    observed_at_unix_ms: u64,
    suffix: &str,
) -> SettlementObservation {
    let observation_id = reference(&format!("observation:{operation}:{revision}:{suffix}"));
    SettlementObservation {
        observation_id: observation_id.clone(),
        subject: subject.id.clone(),
        financial_effect_commitment: subject.financial_effect_commitment,
        attempt: subject.attempt.clone(),
        rail: subject.rail.clone(),
        network: subject.network.clone(),
        operation_id: reference(operation),
        revision,
        amount: amount(units),
        state,
        observed_at_unix_ms,
        evidence: vec![FinalityEvidence {
            evidence_id: reference(&format!("evidence:{operation}:{revision}:{suffix}")),
            subject: subject.id.clone(),
            operation_id: reference(operation),
            operation_revision: revision,
            observation_id,
            kind: reference("evidence:provider-settlement"),
            source: reference(&format!("source:{suffix}")),
            digest: Digest32::repeat(7),
        }],
    }
}

#[test]
fn qualification_entry_points_return_without_panicking_for_degenerate_inputs() {
    let profile = profile();
    let subject = subject(&profile);
    let context = context(1_100);

    let zero_revision = observation(
        &subject,
        "op:zero-revision",
        0,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "zero",
    );
    let conflicting = vec![
        observation(
            &subject,
            "op:conflict",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
            "a",
        ),
        observation(
            &subject,
            "op:conflict",
            1,
            90,
            ObservedSettlementState::Applied,
            1_010,
            "b",
        ),
    ];
    let future = observation(
        &subject,
        "op:future",
        1,
        100,
        ObservedSettlementState::Applied,
        1_200,
        "future",
    );
    let uncertain = observation(
        &subject,
        "op:pending",
        1,
        100,
        ObservedSettlementState::Pending,
        1_000,
        "pending",
    );

    let cases = vec![
        Vec::new(),
        vec![zero_revision],
        conflicting,
        vec![future],
        vec![uncertain],
    ];

    for observations in cases {
        let qualification = catch_unwind(AssertUnwindSafe(|| {
            qualify_settlement(&subject, &profile, &observations, &context)
        }));
        assert!(qualification.is_ok(), "qualify_settlement panicked");

        let receipt = catch_unwind(AssertUnwindSafe(|| {
            qualify_settlement_with_receipt(&subject, &profile, &observations, &context)
        }));
        assert!(receipt.is_ok(), "qualify_settlement_with_receipt panicked");
    }
}

#[test]
fn invalidation_entry_points_return_without_panicking_for_degenerate_inputs() {
    let profile = profile();
    let subject = subject(&profile);
    let prior_context = context(1_100);
    let prior_observation = observation(
        &subject,
        "op:qualified",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "prior",
    );
    let prior = qualify_settlement(&subject, &profile, &[prior_observation], &prior_context)
        .expect("valid prior qualification");
    let later_context = context(1_250);

    let same_revision = observation(
        &subject,
        "op:qualified",
        1,
        100,
        ObservedSettlementState::Reversed,
        1_200,
        "same-revision",
    );
    let wrong_operation = observation(
        &subject,
        "op:not-qualified",
        2,
        100,
        ObservedSettlementState::Reversed,
        1_200,
        "wrong-operation",
    );
    let mut missing_evidence = observation(
        &subject,
        "op:qualified",
        2,
        100,
        ObservedSettlementState::Reversed,
        1_200,
        "missing-evidence",
    );
    missing_evidence.evidence.clear();
    let future = observation(
        &subject,
        "op:qualified",
        2,
        100,
        ObservedSettlementState::Reversed,
        1_300,
        "future",
    );

    for candidate in [same_revision, wrong_operation, missing_evidence, future] {
        let invalidation = catch_unwind(AssertUnwindSafe(|| {
            derive_invalidation(&prior, &profile, &candidate, &later_context)
        }));
        assert!(invalidation.is_ok(), "derive_invalidation panicked");

        let receipt = catch_unwind(AssertUnwindSafe(|| {
            derive_invalidation_with_receipt(&prior, &profile, &candidate, &later_context)
        }));
        assert!(receipt.is_ok(), "derive_invalidation_with_receipt panicked");
    }
}
