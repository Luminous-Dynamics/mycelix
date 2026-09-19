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

fn profile_with(
    kinds: &[&str],
    min_distinct_sources: u16,
    max_observation_age_ms: u64,
) -> FinalityProfile {
    FinalityProfile::new(
        reference("finality:bank:v1"),
        1,
        reference("rail:bank"),
        reference("network:test-bank"),
        kinds.iter().map(|kind| reference(kind)).collect(),
        min_distinct_sources,
        max_observation_age_ms,
        ReversalModel::MayReverse,
    )
    .expect("valid profile")
}

fn profile() -> FinalityProfile {
    profile_with(&["evidence:provider-settlement"], 1, 60_000)
}

fn context(class: EvaluationContextClass, time: u64) -> SettlementEvaluationContext {
    match class {
        EvaluationContextClass::DeterministicSupplied => {
            SettlementEvaluationContext::deterministic_supplied(
                time,
                reference("clock:metamorphic:test"),
                1,
                Digest32::repeat(4),
            )
        }
        EvaluationContextClass::HistoricalReplay => SettlementEvaluationContext::historical_replay(
            time,
            reference("clock:metamorphic:test"),
            1,
            Digest32::repeat(4),
        ),
    }
    .expect("valid context")
}

fn deterministic_context(time: u64) -> SettlementEvaluationContext {
    context(EvaluationContextClass::DeterministicSupplied, time)
}

fn subject_for(profile: &FinalityProfile) -> SettlementSubject {
    SettlementSubject {
        id: reference("settlement:subject:metamorphic"),
        financial_effect_commitment: Digest32::repeat(5),
        attempt: attempt("attempt:metamorphic"),
        rail: profile.rail().clone(),
        network: profile.network().clone(),
        amount: amount(100),
        required_profile: profile.profile_ref().clone(),
    }
}

fn evidence(
    subject: &SettlementSubject,
    observation_id: &ReferenceId,
    operation: &str,
    revision: u64,
    suffix: &str,
) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: reference(&format!("evidence:{operation}:{revision}:{suffix}")),
        subject: subject.id.clone(),
        operation_id: reference(operation),
        operation_revision: revision,
        observation_id: observation_id.clone(),
        kind: reference("evidence:provider-settlement"),
        source: reference(&format!("source:{suffix}")),
        digest: Digest32::repeat(7),
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
        evidence: vec![evidence(
            subject,
            &observation_id,
            operation,
            revision,
            suffix,
        )],
    }
}

fn qualified_three_operation_corpus(subject: &SettlementSubject) -> Vec<SettlementObservation> {
    vec![
        observation(
            subject,
            "op:a",
            1,
            20,
            ObservedSettlementState::Applied,
            1_000,
            "a",
        ),
        observation(
            subject,
            "op:b",
            1,
            30,
            ObservedSettlementState::Applied,
            1_010,
            "b",
        ),
        observation(
            subject,
            "op:c",
            1,
            50,
            ObservedSettlementState::Applied,
            1_020,
            "c",
        ),
    ]
}

#[test]
fn bounded_input_permutations_preserve_the_complete_qualified_result() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let corpus = qualified_three_operation_corpus(&subject);
    let baseline = qualify_settlement(&subject, &profile, &corpus, &context)
        .expect("baseline corpus qualifies");

    let permutations = [
        [0_usize, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    for permutation in permutations {
        let permuted = permutation
            .into_iter()
            .map(|index| corpus[index].clone())
            .collect::<Vec<_>>();
        let actual = qualify_settlement(&subject, &profile, &permuted, &context)
            .expect("permutation qualifies");
        assert_eq!(actual, baseline);
    }
}

#[test]
fn exact_duplicate_observations_are_idempotent_for_value_and_frontier() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let current = observation(
        &subject,
        "op:only",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "provider-a",
    );

    let single = qualify_settlement(&subject, &profile, std::slice::from_ref(&current), &context)
        .expect("single qualifies");
    let duplicate = qualify_settlement(
        &subject,
        &profile,
        &[current.clone(), current.clone(), current],
        &context,
    )
    .expect("duplicates qualify");

    assert_eq!(duplicate, single);
    assert_eq!(duplicate.operations().len(), 1);
    assert_eq!(duplicate.amount().atomic_units(), 100);
}

#[test]
fn higher_current_revision_never_falls_back_to_lower_applied_revision() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let lower = observation(
        &subject,
        "op:revision",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "lower",
    );

    for state in [
        ObservedSettlementState::Rejected,
        ObservedSettlementState::Reversed,
    ] {
        let higher = observation(&subject, "op:revision", 2, 100, state, 1_050, "higher");
        assert_eq!(
            qualify_settlement(&subject, &profile, &[lower.clone(), higher], &context,),
            Err(SettlementQualificationError::PartialSettlement)
        );
    }

    let uncertain = observation(
        &subject,
        "op:revision",
        2,
        100,
        ObservedSettlementState::Pending,
        1_050,
        "pending",
    );
    assert_eq!(
        qualify_settlement(&subject, &profile, &[lower, uncertain], &context),
        Err(SettlementQualificationError::OutstandingUncertainty)
    );
}

#[test]
fn stale_same_revision_evidence_cannot_be_improved_by_fresh_peer_or_input_order() {
    let profile = profile_with(&["evidence:provider-settlement"], 1, 50);
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let stale = observation(
        &subject,
        "op:stale",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "stale",
    );
    let fresh = observation(
        &subject,
        "op:stale",
        1,
        100,
        ObservedSettlementState::Applied,
        1_090,
        "fresh",
    );

    for observations in [vec![stale.clone(), fresh.clone()], vec![fresh, stale]] {
        assert_eq!(
            qualify_settlement(&subject, &profile, &observations, &context),
            Err(SettlementQualificationError::StaleObservation)
        );
    }
}

#[test]
fn same_revision_conflict_is_monotonic_under_additional_agreement() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let first = observation(
        &subject,
        "op:conflict",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "a",
    );
    let disagreeing = observation(
        &subject,
        "op:conflict",
        1,
        90,
        ObservedSettlementState::Applied,
        1_010,
        "b",
    );
    let agreeing = observation(
        &subject,
        "op:conflict",
        1,
        100,
        ObservedSettlementState::Applied,
        1_020,
        "c",
    );

    for observations in [
        vec![first.clone(), disagreeing.clone()],
        vec![first, disagreeing, agreeing],
    ] {
        assert_eq!(
            qualify_settlement(&subject, &profile, &observations, &context),
            Err(SettlementQualificationError::ConflictingCurrentRevision)
        );
    }
}

#[test]
fn evidence_id_conflict_is_monotonic_under_additional_copies() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let first = observation(
        &subject,
        "op:evidence-id",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "a",
    );
    let mut conflicting = observation(
        &subject,
        "op:evidence-id",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "b",
    );
    conflicting.evidence[0].evidence_id = first.evidence[0].evidence_id.clone();

    for observations in [
        vec![first.clone(), conflicting.clone()],
        vec![first.clone(), conflicting, first],
    ] {
        assert_eq!(
            qualify_settlement(&subject, &profile, &observations, &context),
            Err(SettlementQualificationError::EvidenceIdConflict)
        );
    }
}

#[test]
fn observation_id_conflict_is_monotonic_under_additional_copies() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let first = observation(
        &subject,
        "op:observation-id",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "a",
    );
    let mut conflicting = observation(
        &subject,
        "op:observation-id",
        1,
        90,
        ObservedSettlementState::Applied,
        1_000,
        "b",
    );
    conflicting.observation_id = first.observation_id.clone();
    conflicting.evidence[0].observation_id = first.observation_id.clone();

    for observations in [
        vec![first.clone(), conflicting.clone()],
        vec![first.clone(), conflicting, first],
    ] {
        assert_eq!(
            qualify_settlement(&subject, &profile, &observations, &context),
            Err(SettlementQualificationError::ObservationIdConflict)
        );
    }
}

#[test]
fn qualified_operation_amounts_conserve_the_exact_subject_amount() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);

    for (left, right) in [(1_u64, 99_u64), (20, 80), (40, 60), (50, 50), (99, 1)] {
        let observations = vec![
            observation(
                &subject,
                "op:left",
                1,
                left,
                ObservedSettlementState::Applied,
                1_000,
                "left",
            ),
            observation(
                &subject,
                "op:right",
                1,
                right,
                ObservedSettlementState::Applied,
                1_010,
                "right",
            ),
        ];
        let qualified = qualify_settlement(&subject, &profile, &observations, &context)
            .expect("partition qualifies");
        let sum = qualified
            .operations()
            .values()
            .map(|operation| operation.amount().atomic_units())
            .sum::<u64>();
        assert_eq!(sum, subject.amount.atomic_units());
    }

    let partial = observation(
        &subject,
        "op:partial",
        1,
        99,
        ObservedSettlementState::Applied,
        1_000,
        "partial",
    );
    assert_eq!(
        qualify_settlement(&subject, &profile, &[partial], &context),
        Err(SettlementQualificationError::PartialSettlement)
    );

    let over = observation(
        &subject,
        "op:over",
        1,
        101,
        ObservedSettlementState::Applied,
        1_000,
        "over",
    );
    assert_eq!(
        qualify_settlement(&subject, &profile, &[over], &context),
        Err(SettlementQualificationError::OverSettlement)
    );
}

#[test]
fn changing_only_financial_effect_commitment_isolation_fails_closed() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let mut changed = observation(
        &subject,
        "op:effect",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "effect",
    );
    changed.financial_effect_commitment = Digest32::repeat(99);

    assert_eq!(
        qualify_settlement(&subject, &profile, &[changed], &context),
        Err(SettlementQualificationError::EffectCommitmentMismatch)
    );
}

#[test]
fn context_class_changes_proof_identity_even_at_the_same_timestamp() {
    let profile = profile();
    let subject = subject_for(&profile);
    let observation = observation(
        &subject,
        "op:context",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "context",
    );
    let deterministic = context(EvaluationContextClass::DeterministicSupplied, 1_100);
    let historical = context(EvaluationContextClass::HistoricalReplay, 1_100);

    let deterministic_result = qualify_settlement(
        &subject,
        &profile,
        std::slice::from_ref(&observation),
        &deterministic,
    )
    .expect("deterministic qualifies");
    let historical_result = qualify_settlement(&subject, &profile, &[observation], &historical)
        .expect("historical qualifies");

    assert_ne!(deterministic.commitment(), historical.commitment());
    assert_ne!(
        deterministic_result.evidence_frontier(),
        historical_result.evidence_frontier()
    );
    assert_ne!(
        deterministic_result.evaluation_context_class(),
        historical_result.evaluation_context_class()
    );
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_reference(out: &mut Vec<u8>, value: &str) {
    push_u32(
        out,
        u32::try_from(value.len()).expect("bounded test reference"),
    );
    out.extend_from_slice(value.as_bytes());
}

#[test]
fn canonical_reference_set_order_matches_independent_raw_utf8_sort() {
    let physical = ["kind:zz", "kind:a", "kind:bbb", "kind:aa", "kind:a"];
    let profile = profile_with(&physical, 1, 60_000);
    let actual = canonical_finality_profile_bytes(&profile).expect("canonical profile");

    let mut unique = physical.to_vec();
    unique.sort_by(|left, right| left.as_bytes().cmp(right.as_bytes()));
    unique.dedup();

    let mut expected = Vec::new();
    expected.extend_from_slice(b"MYCELIX_FINANCE_SETTLEMENT_FINALITY_PROFILE_V1\0");
    push_u16(&mut expected, SETTLEMENT_COMMITMENT_PROFILE_REVISION);
    push_reference(&mut expected, "finality:bank:v1");
    push_u64(&mut expected, 1);
    push_reference(&mut expected, "rail:bank");
    push_reference(&mut expected, "network:test-bank");
    push_u32(
        &mut expected,
        u32::try_from(unique.len()).expect("bounded test set"),
    );
    for kind in unique {
        push_reference(&mut expected, kind);
    }
    push_u16(&mut expected, 1);
    push_u64(&mut expected, 60_000);
    expected.push(0x00);

    assert_eq!(actual, expected);
}

#[test]
fn receipt_selection_reconstructs_the_exact_qualified_frontier() {
    let profile = profile();
    let subject = subject_for(&profile);
    let context = deterministic_context(1_100);
    let observations = qualified_three_operation_corpus(&subject);
    let receipt = qualify_settlement_with_receipt(&subject, &profile, &observations, &context)
        .expect("receipt qualifies");

    let reconstructed = selected_evidence_frontier_commitment(
        &subject,
        &profile,
        context.commitment(),
        receipt.selected_observation_commitments(),
    )
    .expect("reconstruct frontier");

    assert_eq!(reconstructed, receipt.settlement().evidence_frontier());
}
