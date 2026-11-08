// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::causal::CommittedIntent;
use mycelix_business_core::closure::{
    ClosureClass, ClosureError, ClosureRequirements, CompensationBinding, WorkflowClosureReceipt,
};
use mycelix_business_core::{
    ClosurePolicyRef, CutError, DomainRef, LogicalIntentRef, OperationCommitment,
    OrganizationContextRef, QualificationCut, QualifiedInputRef, RecordRef, SemanticProfileId,
    TimestampMs, ValidityEnd, ValidityWindow, WorkflowRef,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn committed(material_commitment: &str) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("refund:payment:1").expect("valid intent"),
        profile("finance.refund", 1),
        OperationCommitment::new(material_commitment).expect("valid commitment"),
    )
}

fn compensation_result(valid_until: i64) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: DomainRef::new("finance").expect("valid domain"),
        record: RecordRef::new("refund-result:1").expect("valid record"),
        version: 1,
        semantic_profile: profile("finance.refund-result", 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
        )
        .expect("valid window"),
    }
}

fn cut(at: i64) -> QualificationCut {
    QualificationCut::new(
        OrganizationContextRef::new("org:acme").expect("valid context"),
        profile("business-qualification", 1),
        TimestampMs::new(at),
    )
}

fn compensated_receipt(
    qualification_cut: QualificationCut,
    compensating_intents: BTreeSet<CompensationBinding>,
) -> Result<WorkflowClosureReceipt, ClosureError> {
    WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:refund").expect("valid workflow"),
        ClosurePolicyRef::new("closure:refund").expect("valid policy"),
        profile("refund-close", 1),
        qualification_cut,
        ClosureRequirements::default(),
        ClosureClass::Compensated,
        Vec::new(),
        Vec::new(),
        compensating_intents,
    )
}

#[test]
fn compensation_source_must_be_present_in_exact_cut() {
    let qualification_cut = cut(50);
    let intent = committed("material:payment-1:alice:usd:500");
    let source = compensation_result(100);

    let result = CompensationBinding::bind(
        intent.clone(),
        source.clone(),
        &qualification_cut,
    );

    assert!(matches!(
        result,
        Err(ClosureError::CompensationSourceMissingFromCut {
            intent: missing_intent,
            source: missing_source,
        }) if missing_intent == intent && missing_source == source
    ));
}

#[test]
fn compensated_closure_retains_exact_committed_operation_and_result() {
    let mut qualification_cut = cut(50);
    let intent = committed("material:payment-1:alice:usd:500");
    let source = compensation_result(100);
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    let binding = CompensationBinding::bind(
        intent.clone(),
        source.clone(),
        &qualification_cut,
    )
    .expect("exact compensation binding");

    let receipt = compensated_receipt(
        qualification_cut,
        BTreeSet::from([binding]),
    )
    .expect("compensated closure with exact provenance");

    assert_eq!(receipt.compensating_intents().len(), 1);
    let retained = receipt
        .compensating_intents()
        .iter()
        .next()
        .expect("one compensation");
    assert_eq!(retained.intent(), &intent);
    assert_eq!(retained.source(), &source);
}

#[test]
fn compensation_binding_cannot_float_to_another_cut() {
    let mut first_cut = cut(50);
    let intent = committed("material:payment-1:alice:usd:500");
    let source = compensation_result(100);
    assert!(first_cut.insert_input(source.clone()).unwrap());

    let binding = CompensationBinding::bind(intent.clone(), source.clone(), &first_cut)
        .expect("binding qualifies against first cut");

    let result = compensated_receipt(cut(50), BTreeSet::from([binding]));

    assert!(matches!(
        result,
        Err(ClosureError::CompensationSourceMissingFromCut {
            intent: missing_intent,
            source: missing_source,
        }) if missing_intent == intent && missing_source == source
    ));
}

#[test]
fn stale_compensation_result_cannot_close_workflow() {
    let mut qualification_cut = cut(50);
    let intent = committed("material:payment-1:alice:usd:500");
    let source = compensation_result(49);
    assert!(qualification_cut.insert_input(source.clone()).unwrap());

    let binding = CompensationBinding::bind(intent, source, &qualification_cut)
        .expect("membership binding is structural; freshness is cut-qualified");

    let result = compensated_receipt(qualification_cut, BTreeSet::from([binding]));

    assert!(matches!(
        result,
        Err(ClosureError::InvalidQualification(
            CutError::InputNotValidAtQualification { .. }
        ))
    ));
}

#[test]
fn changed_compensation_semantics_do_not_collapse_to_same_logical_id() {
    let original = committed("material:payment-1:alice:usd:500");
    let changed = committed("material:payment-1:alice:usd:600");

    assert_eq!(original.intent_ref, changed.intent_ref);
    assert_ne!(original, changed);
}
