// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::closure::ExceptionBinding;
use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, DomainExceptionRef,
    DomainRef, DomainScopedRef, ExceptionRef, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
    WorkflowClosureReceipt, WorkflowRef,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

fn source(domain_name: &str, record_name: &str, valid_until: i64) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: profile("fixture.exception", 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
        )
        .expect("valid window"),
    }
}

fn cut() -> QualificationCut {
    QualificationCut::new(
        OrganizationContextRef::new("org:acme").expect("valid context"),
        profile("business-qualification", 1),
        TimestampMs::new(50),
    )
}

fn close(
    qualification_cut: QualificationCut,
    class: ClosureClass,
    exceptions: Vec<ExceptionBinding>,
) -> Result<WorkflowClosureReceipt, ClosureError> {
    WorkflowClosureReceipt::new(
        WorkflowRef::new("workflow:month-close").expect("valid workflow"),
        ClosurePolicyRef::new("closure:month-end").expect("valid policy"),
        profile("month-end-close", 1),
        qualification_cut,
        ClosureRequirements::default(),
        class,
        Vec::new(),
        exceptions,
        BTreeSet::new(),
    )
}

#[test]
fn equal_local_exception_ids_from_different_domains_do_not_collapse() {
    let finance = exception("finance", "exception:1");
    let commerce = exception("commerce", "exception:1");
    assert_ne!(finance, commerce);

    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_exception(finance));
    assert!(qualification_cut.insert_exception(commerce));
    assert_eq!(qualification_cut.exceptions().len(), 2);
}

#[test]
fn exception_source_must_be_owned_by_same_domain() {
    let retained = exception("finance", "exception:bank-item:1");
    let wrong_source = source("commerce", "exception-result:1", 100);
    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_input(wrong_source.clone()).unwrap());

    let result = ExceptionBinding::bind(
        retained.clone(),
        wrong_source,
        &qualification_cut,
    );

    assert!(matches!(
        result,
        Err(ClosureError::ExceptionSourceDomainMismatch {
            exception: mismatch,
            source_domain,
        }) if mismatch == retained && source_domain == domain("commerce")
    ));
}

#[test]
fn exception_source_must_be_present_in_exact_cut() {
    let retained = exception("finance", "exception:bank-item:1");
    let missing_source = source("finance", "exception-result:1", 100);

    let result = ExceptionBinding::bind(
        retained.clone(),
        missing_source.clone(),
        &cut(),
    );

    assert!(matches!(
        result,
        Err(ClosureError::ExceptionSourceMissingFromCut {
            exception: missing,
            source,
        }) if missing == retained && source == missing_source
    ));
}

#[test]
fn receipt_cannot_hide_exception_retained_by_cut() {
    let retained = exception("finance", "exception:bank-item:1");
    let exact_source = source("finance", "exception-result:1", 100);
    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_exception(retained.clone()));
    assert!(qualification_cut.insert_input(exact_source).unwrap());

    let result = close(qualification_cut, ClosureClass::Satisfied, Vec::new());

    assert!(matches!(
        result,
        Err(ClosureError::ExceptionSetMismatch {
            qualification_cut: cut_exceptions,
            receipt,
        }) if cut_exceptions == BTreeSet::from([retained]) && receipt.is_empty()
    ));
}

#[test]
fn receipt_cannot_invent_exception_absent_from_cut() {
    let retained = exception("finance", "exception:invented");
    let exact_source = source("finance", "exception-result:invented", 100);
    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_input(exact_source.clone()).unwrap());
    let binding = ExceptionBinding::bind(
        retained.clone(),
        exact_source,
        &qualification_cut,
    )
    .expect("source provenance is exact even though exception is absent from cut");

    let result = close(
        qualification_cut,
        ClosureClass::ResolvedWithExceptions,
        vec![binding],
    );

    assert!(matches!(
        result,
        Err(ClosureError::ExceptionSetMismatch {
            qualification_cut,
            receipt,
        }) if qualification_cut.is_empty() && receipt == BTreeSet::from([retained])
    ));
}

#[test]
fn duplicate_exception_bindings_are_rejected() {
    let retained = exception("finance", "exception:bank-item:1");
    let exact_source = source("finance", "exception-result:1", 100);
    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_exception(retained.clone()));
    assert!(qualification_cut.insert_input(exact_source.clone()).unwrap());
    let first = ExceptionBinding::bind(
        retained.clone(),
        exact_source.clone(),
        &qualification_cut,
    )
    .unwrap();
    let second = ExceptionBinding::bind(
        retained.clone(),
        exact_source,
        &qualification_cut,
    )
    .unwrap();

    let result = close(
        qualification_cut,
        ClosureClass::ResolvedWithExceptions,
        vec![first, second],
    );

    assert!(matches!(
        result,
        Err(ClosureError::DuplicateExceptionBinding { exception })
            if exception == retained
    ));
}

#[test]
fn resolved_with_exceptions_retains_exact_bound_cut_set() {
    let retained = exception("finance", "exception:bank-item:1");
    let exact_source = source("finance", "exception-result:1", 100);
    let mut qualification_cut = cut();
    assert!(qualification_cut.insert_exception(retained.clone()));
    assert!(qualification_cut.insert_input(exact_source.clone()).unwrap());
    let binding = ExceptionBinding::bind(
        retained.clone(),
        exact_source.clone(),
        &qualification_cut,
    )
    .expect("exact exception binding");

    let receipt = close(
        qualification_cut,
        ClosureClass::ResolvedWithExceptions,
        vec![binding],
    )
    .expect("exact retained exception binding qualifies");

    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
    assert_eq!(receipt.exceptions().len(), 1);
    let retained_binding = receipt
        .exceptions()
        .get(&retained)
        .expect("exception retained by exact identity");
    assert_eq!(retained_binding.source(), &exact_source);
}
