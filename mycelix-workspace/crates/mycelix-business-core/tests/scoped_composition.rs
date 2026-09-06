// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, CutError,
    DomainObligationRef, DomainRef, DomainScopedRef, GenerationBinding, GenerationNamespace,
    ObligationDisposition, ObligationDispositionBinding, ObligationRef, OrganizationContextRef,
    QualificationCut, QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd,
    ValidityWindow, WorkflowClosureReceipt, WorkflowRef,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn obligation(domain_name: &str, local_id: &str) -> DomainObligationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ObligationRef::new(local_id).expect("valid obligation"),
    )
}

fn input(domain_name: &str, record_name: &str, generation: Option<u64>) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: profile("fixture.input", 1),
        generation: generation.map(|generation| GenerationBinding {
            namespace: GenerationNamespace::new("policy").expect("valid namespace"),
            generation,
        }),
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(100)),
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

fn bind(
    obligation: DomainObligationRef,
    disposition: ObligationDisposition,
    source: QualifiedInputRef,
    cut: &QualificationCut,
) -> ObligationDispositionBinding {
    ObligationDispositionBinding::bind(obligation, disposition, source, cut)
        .expect("valid exact disposition binding")
}

#[test]
fn equal_generation_namespace_names_from_different_domains_do_not_conflict() {
    let mut cut = cut(50);
    cut.insert_input(input("finance", "policy:finance", Some(3)))
        .expect("finance generation should qualify");
    cut.insert_input(input("governance", "policy:governance", Some(9)))
        .expect("unrelated governance generation must not conflict");

    assert_eq!(cut.inputs().len(), 2);
}

#[test]
fn mixed_generation_in_same_domain_local_namespace_is_rejected() {
    let mut cut = cut(50);
    cut.insert_input(input("finance", "policy:a", Some(3)))
        .unwrap();

    let err = cut
        .insert_input(input("finance", "policy:b", Some(4)))
        .unwrap_err();

    assert!(matches!(
        err,
        CutError::GenerationConflict {
            domain: conflict_domain,
            existing: 3,
            incoming: 4,
            ..
        } if conflict_domain == domain("finance")
    ));
}

#[test]
fn disposition_source_must_be_owned_by_same_domain_as_obligation() {
    let finance_obligation = obligation("finance", "obligation:payment");
    let commerce_source = input("commerce", "payment-disposition:1", None);
    let mut cut = cut(50);
    assert!(cut.insert_input(commerce_source.clone()).unwrap());

    let result = ObligationDispositionBinding::bind(
        finance_obligation.clone(),
        ObligationDisposition::Satisfied,
        commerce_source,
        &cut,
    );

    assert!(matches!(
        result,
        Err(ClosureError::DispositionSourceDomainMismatch {
            obligation: mismatch,
            source_domain,
        }) if mismatch == finance_obligation && source_domain == domain("commerce")
    ));
}

#[test]
fn disposition_source_must_be_present_in_exact_cut() {
    let finance_obligation = obligation("finance", "obligation:payment");
    let finance_source = input("finance", "payment-disposition:1", None);
    let cut = cut(50);

    let result = ObligationDispositionBinding::bind(
        finance_obligation.clone(),
        ObligationDisposition::Satisfied,
        finance_source.clone(),
        &cut,
    );

    assert!(matches!(
        result,
        Err(ClosureError::DispositionSourceMissingFromCut {
            obligation: missing,
            source,
        }) if missing == finance_obligation && source == finance_source
    ));
}

#[test]
fn disposition_binding_cannot_be_replayed_against_different_cut() {
    let payment = obligation("finance", "obligation:payment");
    let source = input("finance", "payment-disposition:1", None);

    let mut original_cut = cut(50);
    assert!(original_cut.insert_input(source.clone()).unwrap());
    let binding = bind(
        payment.clone(),
        ObligationDisposition::Satisfied,
        source.clone(),
        &original_cut,
    );

    let replacement_cut = cut(50);
    let result = WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:cross-cut-replay").unwrap(),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("service-sale-close", 1),
        replacement_cut,
        ClosureRequirements::new(BTreeSet::from([payment.clone()])),
        ClosureClass::Satisfied,
        vec![binding],
        Vec::new(),
        BTreeSet::new(),
    );

    assert!(matches!(
        result,
        Err(ClosureError::DispositionSourceMissingFromCut {
            obligation: missing,
            source: missing_source,
        }) if missing == payment && missing_source == source
    ));
}

#[test]
fn stale_disposition_source_cannot_close_workflow() {
    let payment = obligation("finance", "obligation:payment");
    let mut stale_source = input("finance", "payment-disposition:stale", None);
    stale_source.validity = ValidityWindow::new(
        TimestampMs::new(0),
        ValidityEnd::At(TimestampMs::new(49)),
    )
    .unwrap();

    let mut cut = cut(50);
    assert!(cut.insert_input(stale_source.clone()).unwrap());
    let binding = bind(
        payment.clone(),
        ObligationDisposition::Satisfied,
        stale_source,
        &cut,
    );

    let result = WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:stale-disposition").unwrap(),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("service-sale-close", 1),
        cut,
        ClosureRequirements::new(BTreeSet::from([payment])),
        ClosureClass::Satisfied,
        vec![binding],
        Vec::new(),
        BTreeSet::new(),
    );

    assert!(matches!(
        result,
        Err(ClosureError::InvalidQualification(
            CutError::InputNotValidAtQualification { .. }
        ))
    ));
}

#[test]
fn equal_local_obligation_ids_from_different_domains_remain_distinct() {
    let commerce = obligation("commerce", "obligation:1");
    let finance = obligation("finance", "obligation:1");
    assert_ne!(commerce, finance);

    let commerce_source = input("commerce", "disposition:obligation:1", None);
    let finance_source = input("finance", "disposition:obligation:1", None);
    let mut cut = cut(50);
    assert!(cut.insert_input(commerce_source.clone()).unwrap());
    assert!(cut.insert_input(finance_source.clone()).unwrap());

    let requirements = ClosureRequirements::new(BTreeSet::from([
        commerce.clone(),
        finance.clone(),
    ]));
    let bindings = vec![
        bind(
            commerce,
            ObligationDisposition::Satisfied,
            commerce_source,
            &cut,
        ),
        bind(
            finance,
            ObligationDisposition::Satisfied,
            finance_source,
            &cut,
        ),
    ];

    let receipt = WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:service-sale").unwrap(),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("service-sale-close", 1),
        cut,
        requirements,
        ClosureClass::Satisfied,
        bindings,
        Vec::new(),
        BTreeSet::new(),
    )
    .expect("both independently owned obligations are satisfied by exact sources");

    assert_eq!(receipt.obligations().len(), 2);
}

#[test]
fn omitting_one_domain_obligation_cannot_be_hidden_by_equal_local_id() {
    let commerce = obligation("commerce", "obligation:1");
    let finance = obligation("finance", "obligation:1");
    let commerce_source = input("commerce", "disposition:obligation:1", None);
    let mut cut = cut(50);
    assert!(cut.insert_input(commerce_source.clone()).unwrap());

    let requirements = ClosureRequirements::new(BTreeSet::from([
        commerce.clone(),
        finance.clone(),
    ]));
    let bindings = vec![bind(
        commerce,
        ObligationDisposition::Satisfied,
        commerce_source,
        &cut,
    )];

    let result = WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:service-sale").unwrap(),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("service-sale-close", 1),
        cut,
        requirements,
        ClosureClass::Satisfied,
        bindings,
        Vec::new(),
        BTreeSet::new(),
    );

    assert!(matches!(
        result,
        Err(ClosureError::MissingRequiredObligation { obligation })
            if obligation == finance
    ));
}

#[test]
fn duplicate_disposition_bindings_for_same_obligation_are_rejected() {
    let payment = obligation("finance", "obligation:payment");
    let source = input("finance", "disposition:payment", None);
    let mut cut = cut(50);
    assert!(cut.insert_input(source.clone()).unwrap());

    let first = bind(
        payment.clone(),
        ObligationDisposition::Satisfied,
        source.clone(),
        &cut,
    );
    let second = bind(
        payment.clone(),
        ObligationDisposition::Satisfied,
        source,
        &cut,
    );

    let result = WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:duplicate-disposition").unwrap(),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("service-sale-close", 1),
        cut,
        ClosureRequirements::new(BTreeSet::from([payment.clone()])),
        ClosureClass::Satisfied,
        vec![first, second],
        Vec::new(),
        BTreeSet::new(),
    );

    assert!(matches!(
        result,
        Err(ClosureError::DuplicateObligationDisposition { obligation })
            if obligation == payment
    ));
}
