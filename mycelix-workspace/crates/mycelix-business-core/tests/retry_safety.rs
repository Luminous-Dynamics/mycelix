// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_business_core::causal::{
    AttemptOutcomeBinding, AttemptOutcomeError, CommittedIntent, RetrySafety, RetrySafetyError,
};
use mycelix_business_core::{
    AttemptOutcome, AttemptRef, DomainRef, LogicalIntentRef, OperationCommitment,
    OrganizationContextRef, QualificationCut, QualifiedInputRef, RecordRef, SemanticProfileId,
    TimestampMs, ValidityEnd, ValidityWindow,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn committed(material_commitment: &str) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("pay:invoice:1").expect("valid intent"),
        profile("finance.pay", 1),
        OperationCommitment::new(material_commitment).expect("valid material commitment"),
    )
}

fn source(record_name: &str, profile_name: &str, valid_until: i64) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: DomainRef::new("finance").expect("valid domain"),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: profile(profile_name, 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
        )
        .expect("valid window"),
    }
}

fn outcome_source(valid_until: i64) -> QualifiedInputRef {
    source("attempt-result:1", "finance.attempt-result", valid_until)
}

fn safety_source(valid_until: i64) -> QualifiedInputRef {
    source("retry-safety:payment:1", "finance.retry-safety", valid_until)
}

fn cut(at: i64) -> QualificationCut {
    QualificationCut::new(
        OrganizationContextRef::new("org:acme").expect("valid context"),
        profile("business-qualification", 1),
        TimestampMs::new(at),
    )
}

fn bind_outcome(
    logical_intent: CommittedIntent,
    outcome: AttemptOutcome,
    source: QualifiedInputRef,
    qualification_cut: &QualificationCut,
) -> AttemptOutcomeBinding {
    AttemptOutcomeBinding::bind(
        AttemptRef::new("attempt:1").expect("valid attempt"),
        logical_intent,
        outcome,
        source,
        qualification_cut,
    )
    .expect("exact attempt-outcome binding")
}

#[test]
fn naked_outcome_source_cannot_authorize_retry() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let source = outcome_source(100);
    let qualification_cut = cut(50);

    let result = AttemptOutcomeBinding::bind(
        AttemptRef::new("attempt:1").unwrap(),
        logical_intent,
        AttemptOutcome::ProvenNotApplied,
        source.clone(),
        &qualification_cut,
    );

    assert!(matches!(
        result,
        Err(AttemptOutcomeError::SourceMissingFromCut {
            source: missing_source,
            ..
        }) if missing_source == source
    ));
}

#[test]
fn retry_safety_source_must_be_present_in_exact_cut() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let source = safety_source(100);
    let qualification_cut = cut(50);

    let result = RetrySafety::bind(
        logical_intent.clone(),
        source.clone(),
        &qualification_cut,
    );

    assert!(matches!(
        result,
        Err(RetrySafetyError::SourceMissingFromCut {
            logical_intent: missing_intent,
            source: missing_source,
        }) if missing_intent == logical_intent && missing_source == source
    ));
}

#[test]
fn only_proven_not_applied_is_retryable_without_additional_safety() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");

    let proven_source = outcome_source(100);
    let mut proven_cut = cut(50);
    assert!(proven_cut.insert_input(proven_source.clone()).unwrap());
    let proven = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::ProvenNotApplied,
        proven_source,
        &proven_cut,
    );
    assert!(proven.retry_permitted_without_additional_safety(&proven_cut));

    let unknown_source = outcome_source(100);
    let mut unknown_cut = cut(50);
    assert!(unknown_cut.insert_input(unknown_source.clone()).unwrap());
    let unknown = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::OutcomeUnknown,
        unknown_source,
        &unknown_cut,
    );
    assert!(!unknown.retry_permitted_without_additional_safety(&unknown_cut));

    let applied_source = outcome_source(100);
    let mut applied_cut = cut(50);
    assert!(applied_cut.insert_input(applied_source.clone()).unwrap());
    let applied = bind_outcome(
        logical_intent,
        AttemptOutcome::Applied,
        applied_source,
        &applied_cut,
    );
    assert!(!applied.retry_permitted_without_additional_safety(&applied_cut));
}

#[test]
fn exact_current_safety_can_allow_exact_unknown_attempt_retry() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let attempt_source = outcome_source(100);
    let safety_source = safety_source(100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_input(attempt_source.clone()).unwrap());
    assert!(qualification_cut.insert_input(safety_source.clone()).unwrap());

    let outcome = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::OutcomeUnknown,
        attempt_source,
        &qualification_cut,
    );
    let safety = RetrySafety::bind(
        logical_intent.clone(),
        safety_source.clone(),
        &qualification_cut,
    )
    .expect("exact retry-safety binding");

    assert_eq!(outcome.logical_intent(), &logical_intent);
    assert_eq!(safety.logical_intent(), &logical_intent);
    assert_eq!(safety.source(), &safety_source);
    assert!(outcome.retry_permitted_with_safety(&safety, &qualification_cut));
}

#[test]
fn retry_safety_cannot_float_to_changed_material_operation() {
    let original_intent = committed("material:invoice-1:alice:usd:500");
    let changed_intent = committed("material:invoice-1:alice:usd:600");
    let attempt_source = outcome_source(100);
    let safety_source = safety_source(100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_input(attempt_source.clone()).unwrap());
    assert!(qualification_cut.insert_input(safety_source.clone()).unwrap());

    let changed_outcome = bind_outcome(
        changed_intent,
        AttemptOutcome::OutcomeUnknown,
        attempt_source,
        &qualification_cut,
    );
    let original_safety = RetrySafety::bind(
        original_intent,
        safety_source,
        &qualification_cut,
    )
    .expect("exact retry-safety binding");

    assert!(!changed_outcome.retry_permitted_with_safety(
        &original_safety,
        &qualification_cut,
    ));
}

#[test]
fn retry_safety_cannot_float_to_a_cut_without_its_source() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let attempt_source = outcome_source(100);
    let safety_source = safety_source(100);
    let mut first_cut = cut(50);
    assert!(first_cut.insert_input(attempt_source.clone()).unwrap());
    assert!(first_cut.insert_input(safety_source.clone()).unwrap());

    let outcome = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::OutcomeUnknown,
        attempt_source.clone(),
        &first_cut,
    );
    let safety = RetrySafety::bind(logical_intent, safety_source, &first_cut)
        .expect("exact retry-safety binding");

    let mut replacement_cut = cut(50);
    assert!(replacement_cut.insert_input(attempt_source).unwrap());
    assert!(!outcome.retry_permitted_with_safety(&safety, &replacement_cut));
}

#[test]
fn expired_retry_safety_cannot_authorize_unknown_retry() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let attempt_source = outcome_source(100);
    let safety_source = safety_source(49);
    let mut original_cut = cut(40);
    assert!(original_cut.insert_input(attempt_source.clone()).unwrap());
    assert!(original_cut.insert_input(safety_source.clone()).unwrap());

    let outcome = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::OutcomeUnknown,
        attempt_source.clone(),
        &original_cut,
    );
    let safety = RetrySafety::bind(logical_intent, safety_source.clone(), &original_cut)
        .expect("safety is current at original qualification time");

    let mut later_cut = cut(50);
    assert!(later_cut.insert_input(attempt_source).unwrap());
    assert!(later_cut.insert_input(safety_source).unwrap());

    assert!(!outcome.retry_permitted_with_safety(&safety, &later_cut));
}

#[test]
fn positive_retry_safety_does_not_make_applied_attempt_retryable() {
    let logical_intent = committed("material:invoice-1:alice:usd:500");
    let attempt_source = outcome_source(100);
    let safety_source = safety_source(100);
    let mut qualification_cut = cut(50);
    assert!(qualification_cut.insert_input(attempt_source.clone()).unwrap());
    assert!(qualification_cut.insert_input(safety_source.clone()).unwrap());

    let outcome = bind_outcome(
        logical_intent.clone(),
        AttemptOutcome::Applied,
        attempt_source,
        &qualification_cut,
    );
    let safety = RetrySafety::bind(logical_intent, safety_source, &qualification_cut)
        .expect("exact retry-safety binding");

    assert!(!outcome.retry_permitted_with_safety(&safety, &qualification_cut));
}
