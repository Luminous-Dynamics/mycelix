use std::collections::BTreeSet;

use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId};
use mycelix_finance_exact::{AssetAmount, AssetId};
use serde_json::Value;

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

fn profile_with(
    kinds: &[&str],
    min_distinct_sources: u16,
    max_observation_age_ms: u64,
    reversal_model: ReversalModel,
) -> FinalityProfile {
    FinalityProfile::new(
        reference("finality:bank:v1"),
        1,
        reference("rail:bank"),
        reference("network:test-bank"),
        kinds.iter().map(|kind| reference(kind)).collect(),
        min_distinct_sources,
        max_observation_age_ms,
        reversal_model,
    )
    .expect("valid test profile")
}

fn profile() -> FinalityProfile {
    profile_with(
        &["evidence:provider-settlement"],
        1,
        60_000,
        ReversalModel::MayReverse,
    )
}

fn context_at(time: u64) -> SettlementEvaluationContext {
    SettlementEvaluationContext::deterministic_supplied(
        time,
        reference("clock:deterministic:test"),
        1,
        Digest32::repeat(4),
    )
    .expect("valid deterministic context")
}

fn context() -> SettlementEvaluationContext {
    context_at(1_100)
}

fn subject_for(profile: &FinalityProfile) -> SettlementSubject {
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

fn subject() -> SettlementSubject {
    subject_for(&profile())
}

fn evidence_for(
    observation_id: &str,
    operation: &str,
    revision: u64,
    suffix: &str,
) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: reference(&format!("evidence:{operation}:{revision}:{suffix}")),
        subject: reference("settlement:subject:1"),
        operation_id: reference(operation),
        operation_revision: revision,
        observation_id: reference(observation_id),
        kind: reference("evidence:provider-settlement"),
        source: reference(&format!("source:{suffix}")),
        digest: Digest32::repeat(7),
    }
}

fn observation_with_id(
    observation_id: &str,
    operation: &str,
    revision: u64,
    units: u64,
    state: ObservedSettlementState,
    observed_at: u64,
    suffix: &str,
) -> SettlementObservation {
    let subject = subject();
    SettlementObservation {
        observation_id: reference(observation_id),
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
        evidence: vec![evidence_for(observation_id, operation, revision, suffix)],
    }
}

fn observation(
    operation: &str,
    revision: u64,
    units: u64,
    state: ObservedSettlementState,
    observed_at: u64,
) -> SettlementObservation {
    let id = format!("observation:{operation}:{revision}");
    observation_with_id(
        &id,
        operation,
        revision,
        units,
        state,
        observed_at,
        "provider-a",
    )
}

fn bytes_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        write!(&mut out, "{byte:02x}").expect("writing to string cannot fail");
    }
    out
}

fn digest_hex(digest: Digest32) -> String {
    bytes_hex(&digest.0)
}

fn fixture() -> Value {
    serde_json::from_str(include_str!("../test-vectors/settlement-v1.json"))
        .expect("checked-in settlement vector must parse")
}

#[test]
fn exact_applied_settlement_qualifies_with_derived_frontier() {
    let qualified = qualify_settlement(
        &subject(),
        &profile(),
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &context(),
    )
    .expect("qualification should succeed");

    assert_eq!(qualified.amount().atomic_units(), 100);
    assert_eq!(qualified.operations().len(), 1);
    assert_eq!(
        digest_hex(qualified.evidence_frontier()),
        fixture()["frontier"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        qualified.evaluation_context_class(),
        EvaluationContextClass::DeterministicSupplied
    );
}

#[test]
fn canonical_vector_reproduces_profile_context_evidence_observation_and_frontier() {
    let vector = fixture();
    let profile = profile();
    let context = context();
    let subject = subject();
    let observation = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    let evidence = &observation.evidence[0];

    let profile_bytes = canonical_finality_profile_bytes(&profile).expect("canonical profile");
    assert_eq!(
        profile_bytes.len() as u64,
        vector["profile"]["canonical_length"]
            .as_u64()
            .expect("vector integer")
    );
    assert_eq!(
        bytes_hex(&profile_bytes),
        vector["profile"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(profile.profile_ref().digest),
        vector["profile"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let context_bytes =
        canonical_evaluation_context_bytes(&context).expect("canonical evaluation context");
    assert_eq!(
        bytes_hex(&context_bytes),
        vector["evaluation_context"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(context.commitment()),
        vector["evaluation_context"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let evidence_bytes = canonical_evidence_bytes(evidence).expect("canonical evidence");
    assert_eq!(
        bytes_hex(&evidence_bytes),
        vector["evidence"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(evidence_commitment(evidence).expect("evidence commitment")),
        vector["evidence"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let observation_bytes =
        canonical_observation_bytes(&observation).expect("canonical observation");
    assert_eq!(
        bytes_hex(&observation_bytes),
        vector["observation"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    let observation_digest = observation_commitment(&observation).expect("observation commitment");
    assert_eq!(
        digest_hex(observation_digest),
        vector["observation"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );

    let selected = BTreeSet::from([observation_digest]);
    let frontier_bytes = canonical_selected_evidence_frontier_bytes(
        &subject,
        &profile,
        context.commitment(),
        &selected,
    )
    .expect("canonical frontier");
    assert_eq!(
        bytes_hex(&frontier_bytes),
        vector["frontier"]["canonical_hex"]
            .as_str()
            .expect("vector string")
    );
    assert_eq!(
        digest_hex(
            selected_evidence_frontier_commitment(
                &subject,
                &profile,
                context.commitment(),
                &selected,
            )
            .expect("frontier commitment")
        ),
        vector["frontier"]["commitment_hex"]
            .as_str()
            .expect("vector string")
    );
}

#[test]
fn profile_body_mutation_with_old_digest_is_rejected() {
    let mut mutated = profile();
    mutated.max_observation_age_ms = 1;
    assert_eq!(
        mutated.validate(),
        Err(FinalityProfileError::DigestMismatch)
    );
}

#[test]
fn required_evidence_set_order_does_not_change_profile_commitment() {
    let first = profile_with(
        &["evidence:provider-settlement", "evidence:witness"],
        2,
        60_000,
        ReversalModel::MayReverse,
    );
    let second = profile_with(
        &["evidence:witness", "evidence:provider-settlement"],
        2,
        60_000,
        ReversalModel::MayReverse,
    );
    assert_eq!(first.profile_ref().digest, second.profile_ref().digest);
}

#[test]
fn input_order_does_not_change_selected_evidence_frontier() {
    let observations = vec![
        observation_with_id(
            "observation:op:1:a",
            "op:1",
            1,
            40,
            ObservedSettlementState::Applied,
            1_000,
            "provider-a",
        ),
        observation_with_id(
            "observation:op:2:a",
            "op:2",
            1,
            60,
            ObservedSettlementState::Applied,
            1_010,
            "provider-b",
        ),
    ];
    let forward =
        qualify_settlement(&subject(), &profile(), &observations, &context()).expect("forward");
    let reversed = qualify_settlement(
        &subject(),
        &profile(),
        &observations.into_iter().rev().collect::<Vec<_>>(),
        &context(),
    )
    .expect("reversed");
    assert_eq!(forward.evidence_frontier(), reversed.evidence_frontier());
}

#[test]
fn stale_same_revision_duplicate_cannot_launder_evidence_freshness() {
    let strict = profile_with(
        &["evidence:provider-settlement"],
        1,
        50,
        ReversalModel::MayReverse,
    );
    let old = observation_with_id(
        "observation:op:1:old",
        "op:1",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "provider-a",
    );
    let fresh = observation_with_id(
        "observation:op:1:fresh",
        "op:1",
        1,
        100,
        ObservedSettlementState::Applied,
        1_090,
        "provider-b",
    );
    assert_eq!(
        qualify_settlement(&subject_for(&strict), &strict, &[old, fresh], &context()),
        Err(SettlementQualificationError::StaleObservation)
    );
}

#[test]
fn revision_one_evidence_cannot_be_replayed_onto_revision_two() {
    let mut replay = observation("op:1", 2, 100, ObservedSettlementState::Applied, 1_000);
    replay.evidence[0].operation_revision = 1;
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[replay], &context()),
        Err(SettlementQualificationError::EvidenceRevisionMismatch)
    );
}

#[test]
fn evidence_must_bind_exact_observation_id() {
    let mut replay = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    replay.evidence[0].observation_id = reference("observation:other");
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[replay], &context()),
        Err(SettlementQualificationError::EvidenceObservationMismatch)
    );
}

#[test]
fn same_observation_id_with_changed_evidence_set_is_conflict() {
    let first = observation_with_id(
        "observation:op:1:same",
        "op:1",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "provider-a",
    );
    let second = observation_with_id(
        "observation:op:1:same",
        "op:1",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "provider-b",
    );
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[first, second], &context()),
        Err(SettlementQualificationError::ObservationIdConflict)
    );
}

#[test]
fn cumulative_operation_revisions_use_only_current_amount() {
    let observations = vec![
        observation("op:1", 1, 30, ObservedSettlementState::Applied, 900),
        observation("op:1", 2, 70, ObservedSettlementState::Applied, 950),
        observation("op:1", 3, 100, ObservedSettlementState::Applied, 1_000),
    ];
    let qualified =
        qualify_settlement(&subject(), &profile(), &observations, &context()).expect("current 100");
    assert_eq!(
        qualified.operations()[&reference("op:1")]
            .amount()
            .atomic_units(),
        100
    );
}

#[test]
fn same_attempt_amount_and_profile_still_require_exact_effect_commitment() {
    let mut wrong_effect = observation("op:1", 1, 100, ObservedSettlementState::Applied, 1_000);
    wrong_effect.financial_effect_commitment = Digest32::repeat(99);
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[wrong_effect], &context()),
        Err(SettlementQualificationError::EffectCommitmentMismatch)
    );
}

#[test]
fn same_revision_state_or_amount_disagreement_fails_closed() {
    let first = observation_with_id(
        "observation:op:1:a",
        "op:1",
        1,
        100,
        ObservedSettlementState::Applied,
        1_000,
        "provider-a",
    );
    let second = observation_with_id(
        "observation:op:1:b",
        "op:1",
        1,
        90,
        ObservedSettlementState::Applied,
        1_010,
        "provider-b",
    );
    assert_eq!(
        qualify_settlement(&subject(), &profile(), &[first, second], &context()),
        Err(SettlementQualificationError::ConflictingCurrentRevision)
    );
}

#[test]
fn pending_or_unknown_current_state_preserves_uncertainty() {
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation(
                "op:1",
                1,
                100,
                ObservedSettlementState::Unknown,
                1_000,
            )],
            &context(),
        ),
        Err(SettlementQualificationError::OutstandingUncertainty)
    );
}

#[test]
fn partial_and_over_settlement_are_distinct() {
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation(
                "op:1",
                1,
                80,
                ObservedSettlementState::Applied,
                1_000,
            )],
            &context(),
        ),
        Err(SettlementQualificationError::PartialSettlement)
    );
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation(
                "op:1",
                1,
                120,
                ObservedSettlementState::Applied,
                1_000,
            )],
            &context(),
        ),
        Err(SettlementQualificationError::OverSettlement)
    );
}

#[test]
fn multiple_operations_can_exactly_satisfy_subject() {
    qualify_settlement(
        &subject(),
        &profile(),
        &[
            observation("op:1", 1, 40, ObservedSettlementState::Applied, 1_000),
            observation("op:2", 1, 60, ObservedSettlementState::Applied, 1_010),
        ],
        &context(),
    )
    .expect("two current operation amounts should compose exactly");
}

#[test]
fn future_observation_fails_relative_to_explicit_evaluation_context() {
    assert_eq!(
        qualify_settlement(
            &subject(),
            &profile(),
            &[observation(
                "op:1",
                1,
                100,
                ObservedSettlementState::Applied,
                1_200,
            )],
            &context(),
        ),
        Err(SettlementQualificationError::FutureObservation)
    );
}

#[test]
fn historical_context_is_distinct_from_deterministic_supplied_context() {
    let historical = SettlementEvaluationContext::historical_replay(
        1_100,
        reference("clock:historical:test"),
        1,
        Digest32::repeat(4),
    )
    .expect("historical context");
    let qualified = qualify_settlement(
        &subject(),
        &profile(),
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &historical,
    )
    .expect("historical relative proof");
    assert_eq!(
        qualified.evaluation_context_class(),
        EvaluationContextClass::HistoricalReplay
    );
    assert_ne!(
        qualified.evaluation_context_commitment(),
        context().commitment()
    );
}

#[test]
fn qualified_settlement_can_be_explicitly_invalidated_by_fresh_higher_revision_evidence() {
    let prior = qualify_settlement(
        &subject(),
        &profile(),
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &context(),
    )
    .expect("initial qualification");

    let reversal = observation("op:1", 2, 100, ObservedSettlementState::Reversed, 1_200);
    let later_context = context_at(1_250);
    let invalidation = derive_invalidation(&prior, &profile(), &reversal, &later_context)
        .expect("fresh higher revision reversal should invalidate");

    assert_eq!(invalidation.prior_revision(), 1);
    assert_eq!(invalidation.operation_id(), &reference("op:1"));
    assert_eq!(invalidation.financial_effect_commitment(), effect());
}

#[test]
fn same_applied_amount_at_higher_revision_does_not_invalidate() {
    let prior = qualify_settlement(
        &subject(),
        &profile(),
        &[observation(
            "op:1",
            1,
            100,
            ObservedSettlementState::Applied,
            1_000,
        )],
        &context(),
    )
    .expect("initial qualification");
    let same = observation("op:1", 2, 100, ObservedSettlementState::Applied, 1_200);
    assert_eq!(
        derive_invalidation(&prior, &profile(), &same, &context_at(1_250)),
        Err(SettlementInvalidationError::ObservationDoesNotInvalidate)
    );
}
