// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! GP-005 refund/dispute qualification through reusable terminal-role schema.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, CommittedIntent, CompensationBinding, DerivationNodeRef,
    DomainExceptionRef, DomainObligationRef, DomainReconciliationRef, DomainRef, DomainScopedRef,
    ExceptionBinding, ExceptionRef, LogicalIntentRef, ObligationDisposition,
    ObligationDispositionBinding, ObligationRef, OperationCommitment, OrganizationContextRef,
    QualificationCut, QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId,
    TimestampMs, ValidityEnd, ValidityWindow, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationError, ClosureQualificationProfile,
    QualifiedBoundaryRef, QualifiedBoundaryRequirement,
};
use mycelix_business_terminal_qualification::{
    CompensationRoleRequirement, TerminalClosureQualificationBasis,
    TerminalClosureQualificationError, TerminalClosureQualificationProfile,
};

fn semantic(name: &str) -> SemanticProfileId {
    SemanticProfileId::new(name, 1).expect("valid semantic profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("valid derivation node")
}

fn input(
    domain_name: &str,
    record_name: &str,
    semantic_name: &str,
    valid_until: i64,
) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: semantic(semantic_name),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
        )
        .expect("valid validity window"),
    }
}

fn obligation(domain_name: &str, local_id: &str) -> DomainObligationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ObligationRef::new(local_id).expect("valid obligation"),
    )
}

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

fn reconciliation(domain_name: &str, local_id: &str) -> DomainReconciliationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ReconciliationRef::new(local_id).expect("valid reconciliation"),
    )
}

fn refund_intent(profile: &str, commitment: &str) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("refund:order-1").unwrap(),
        semantic(profile),
        OperationCommitment::new(commitment).unwrap(),
    )
}

struct CompensatedCase {
    profile: TerminalClosureQualificationProfile,
    basis: TerminalClosureQualificationBasis,
    refund_role: DerivationNodeRef,
    refund_source: QualifiedInputRef,
    disposition_source: QualifiedInputRef,
    original_transaction: QualifiedInputRef,
}

fn compensated_case(resolution_valid_until: i64) -> CompensatedCase {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.refund-dispute.compensated.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:refund-dispute:compensated:v1",
        "governance.closure-policy-active",
        100,
    );
    let original_transaction = input(
        "commerce",
        "order:original-1",
        "commerce.accepted-order-history",
        100,
    );
    let resolution = input(
        "governance",
        "resolution:dispute-1",
        "governance.authorized-dispute-resolution",
        resolution_valid_until,
    );
    let disposition_source = input(
        "finance",
        "refund-obligation-disposition:1",
        "finance.obligation-disposition",
        100,
    );
    let refund_source = input(
        "finance",
        "refund-effect:1",
        "finance.accepted-refund-effect",
        100,
    );
    let accounting = input(
        "accounting",
        "compensating-event:refund-1",
        "accounting.accepted-compensating-event",
        100,
    );
    let appeal = input(
        "governance",
        "appeal-window:dispute-1:final",
        "governance.appeal-window-final",
        100,
    );
    let finance_reconciliation = reconciliation("finance", "refund-reconciliation:1");
    let refund_obligation = obligation("finance", "refund:order-1");

    let root = node("business:refund-dispute-compensated-close");
    let original_role = node("commerce:original-transaction");
    let resolution_role = node("governance:authorized-resolution");
    let disposition_role = node("finance:refund-obligation-disposition");
    let refund_role = node("finance:accepted-refund-effect");
    let reconciliation_role = node("finance:refund-reconciliation");
    let accounting_role = node("accounting:accepted-compensating-event");
    let appeal_role = node("governance:appeal-window-final");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), original_role.clone());
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), disposition_role.clone());
    graph.add_dependency(root.clone(), accounting_role.clone());
    graph.add_dependency(root.clone(), appeal_role.clone());
    graph.add_dependency(accounting_role.clone(), refund_role.clone());
    graph.add_dependency(refund_role.clone(), reconciliation_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:refund-dispute:compensated").unwrap(),
        semantic("business.refund-dispute.compensated.close"),
        ClosureClass::Compensated,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                original_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("commerce"),
                    semantic("commerce.accepted-order-history"),
                ),
            ),
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-dispute-resolution"),
                ),
            ),
            (
                disposition_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.obligation-disposition"),
                ),
            ),
            (
                refund_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.accepted-refund-effect"),
                ),
            ),
            (
                reconciliation_role.clone(),
                QualifiedBoundaryRequirement::reconciliation(domain("finance")),
            ),
            (
                accounting_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("accounting"),
                    semantic("accounting.accepted-compensating-event"),
                ),
            ),
            (
                appeal_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.appeal-window-final"),
                ),
            ),
        ]),
        BTreeMap::from([(disposition_role.clone(), domain("finance"))]),
    );
    let profile = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::from([(
            refund_role.clone(),
            CompensationRoleRequirement::new(semantic("finance.refund")),
        )]),
        BTreeSet::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    for exact in [
        policy_source,
        original_transaction.clone(),
        resolution.clone(),
        disposition_source.clone(),
        refund_source.clone(),
        accounting.clone(),
        appeal.clone(),
    ] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_reconciliation(finance_reconciliation.clone()));

    let disposition_binding = ObligationDispositionBinding::bind(
        refund_obligation.clone(),
        ObligationDisposition::Satisfied,
        disposition_source.clone(),
        &cut,
    )
    .unwrap();
    let compensation = CompensationBinding::bind(
        refund_intent("finance.refund", "USD-cent:5000:customer-a:order-1"),
        refund_source.clone(),
        &cut,
    )
    .unwrap();

    CompensatedCase {
        profile,
        basis: TerminalClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([
                (
                    original_role,
                    QualifiedBoundaryRef::Input(original_transaction.clone()),
                ),
                (resolution_role, QualifiedBoundaryRef::Input(resolution)),
                (
                    disposition_role.clone(),
                    QualifiedBoundaryRef::Input(disposition_source.clone()),
                ),
                (
                    refund_role.clone(),
                    QualifiedBoundaryRef::Input(refund_source.clone()),
                ),
                (
                    reconciliation_role,
                    QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
                ),
                (accounting_role, QualifiedBoundaryRef::Input(accounting)),
                (appeal_role, QualifiedBoundaryRef::Input(appeal)),
            ]),
            obligations: BTreeMap::from([(disposition_role, refund_obligation)]),
            disposition_bindings: vec![disposition_binding],
            compensation_bindings: BTreeMap::from([(refund_role.clone(), compensation)]),
            exception_bindings: BTreeMap::new(),
        },
        refund_role,
        refund_source,
        disposition_source,
        original_transaction,
    }
}

#[test]
fn gp005_compensated_path_qualifies_and_preserves_original_history() {
    let case = compensated_case(100);
    let original = case.original_transaction.clone();
    let receipt = case
        .profile
        .qualify(WorkflowRef::new("workflow:refund-dispute:compensated:1").unwrap(), case.basis)
        .unwrap();

    assert_eq!(receipt.class(), ClosureClass::Compensated);
    assert!(receipt.qualification_cut().inputs().contains(&original));
    assert_eq!(receipt.compensating_intents().len(), 1);
}

#[test]
fn gp005_compensated_path_rejects_other_declared_finance_source() {
    let mut case = compensated_case(100);
    let wrong_binding = CompensationBinding::bind(
        refund_intent("finance.refund", "USD-cent:5000:customer-a:order-1"),
        case.disposition_source.clone(),
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis
        .compensation_bindings
        .insert(case.refund_role.clone(), wrong_binding);

    assert!(matches!(
        case.profile.qualify(
            WorkflowRef::new("workflow:refund-dispute:compensated:2").unwrap(),
            case.basis,
        ),
        Err(TerminalClosureQualificationError::CompensationBoundaryMismatch { .. })
    ));
}

#[test]
fn gp005_compensated_path_rejects_wrong_operation_profile() {
    let mut case = compensated_case(100);
    let wrong_binding = CompensationBinding::bind(
        refund_intent("finance.credit", "USD-cent:5000:customer-a:order-1"),
        case.refund_source.clone(),
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis
        .compensation_bindings
        .insert(case.refund_role.clone(), wrong_binding);

    assert!(matches!(
        case.profile.qualify(
            WorkflowRef::new("workflow:refund-dispute:compensated:3").unwrap(),
            case.basis,
        ),
        Err(TerminalClosureQualificationError::CompensationOperationProfileMismatch { .. })
    ));
}

#[test]
fn gp005_stale_authorized_resolution_fails_closed() {
    let case = compensated_case(70);
    assert!(matches!(
        case.profile.qualify(
            WorkflowRef::new("workflow:refund-dispute:stale-resolution").unwrap(),
            case.basis,
        ),
        Err(TerminalClosureQualificationError::ClosureQualification(
            ClosureQualificationError::InvalidCut(_)
        ))
    ));
}

struct ExceptionCase {
    profile: TerminalClosureQualificationProfile,
    basis: TerminalClosureQualificationBasis,
    exception_role: DerivationNodeRef,
    intended_source: QualifiedInputRef,
    alternate_source: QualifiedInputRef,
    retained_exception: DomainExceptionRef,
    original_transaction: QualifiedInputRef,
}

fn exception_case() -> ExceptionCase {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.refund-dispute.exception.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:refund-dispute:exceptions:v1",
        "governance.closure-policy-active",
        100,
    );
    let original_transaction = input(
        "commerce",
        "order:original-1",
        "commerce.accepted-order-history",
        100,
    );
    let resolution = input(
        "governance",
        "resolution:dispute-1:exception",
        "governance.authorized-dispute-resolution",
        100,
    );
    let intended_source = input(
        "finance",
        "chargeback:pending:1",
        "finance.chargeback-pending",
        100,
    );
    let alternate_source = input(
        "finance",
        "dispute-note:accepted:1",
        "finance.dispute-note-accepted",
        100,
    );
    let accounting = input(
        "accounting",
        "dispute-event:1",
        "accounting.accepted-dispute-resolution-event",
        100,
    );
    let appeal = input(
        "governance",
        "appeal-window:dispute-1:active",
        "governance.appeal-window-active",
        100,
    );
    let retained_exception = exception("finance", "chargeback:pending:1");

    let root = node("business:refund-dispute-exception-close");
    let original_role = node("commerce:original-transaction");
    let resolution_role = node("governance:authorized-resolution");
    let exception_role = node("finance:required-retained-exception");
    let alternate_role = node("finance:other-dispute-result");
    let accounting_role = node("accounting:accepted-dispute-resolution-event");
    let appeal_role = node("governance:appeal-window-active");

    let mut graph = ClosureDependencyGraph::default();
    for role in [
        original_role.clone(),
        resolution_role.clone(),
        exception_role.clone(),
        alternate_role.clone(),
        accounting_role.clone(),
        appeal_role.clone(),
    ] {
        graph.add_dependency(root.clone(), role);
    }

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:refund-dispute:exceptions").unwrap(),
        semantic("business.refund-dispute.exception.close"),
        ClosureClass::ResolvedWithExceptions,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                original_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("commerce"),
                    semantic("commerce.accepted-order-history"),
                ),
            ),
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-dispute-resolution"),
                ),
            ),
            (
                exception_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.chargeback-pending"),
                ),
            ),
            (
                alternate_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.dispute-note-accepted"),
                ),
            ),
            (
                accounting_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("accounting"),
                    semantic("accounting.accepted-dispute-resolution-event"),
                ),
            ),
            (
                appeal_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.appeal-window-active"),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let profile = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::new(),
        BTreeSet::from([exception_role.clone()]),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    for exact in [
        policy_source,
        original_transaction.clone(),
        resolution.clone(),
        intended_source.clone(),
        alternate_source.clone(),
        accounting.clone(),
        appeal.clone(),
    ] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_exception(retained_exception.clone()));

    let exception_binding = ExceptionBinding::bind(
        retained_exception.clone(),
        intended_source.clone(),
        &cut,
    )
    .unwrap();

    ExceptionCase {
        profile,
        basis: TerminalClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([
                (
                    original_role,
                    QualifiedBoundaryRef::Input(original_transaction.clone()),
                ),
                (resolution_role, QualifiedBoundaryRef::Input(resolution)),
                (
                    exception_role.clone(),
                    QualifiedBoundaryRef::Input(intended_source.clone()),
                ),
                (
                    alternate_role,
                    QualifiedBoundaryRef::Input(alternate_source.clone()),
                ),
                (accounting_role, QualifiedBoundaryRef::Input(accounting)),
                (appeal_role, QualifiedBoundaryRef::Input(appeal)),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            compensation_bindings: BTreeMap::new(),
            exception_bindings: BTreeMap::from([(exception_role.clone(), exception_binding)]),
        },
        exception_role,
        intended_source,
        alternate_source,
        retained_exception,
        original_transaction,
    }
}

#[test]
fn gp005_resolved_with_exceptions_qualifies_and_retains_exception_and_history() {
    let case = exception_case();
    let original = case.original_transaction.clone();
    let retained_exception = case.retained_exception.clone();
    let receipt = case
        .profile
        .qualify(WorkflowRef::new("workflow:refund-dispute:exception:1").unwrap(), case.basis)
        .unwrap();

    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
    assert!(receipt.qualification_cut().inputs().contains(&original));
    assert!(receipt.exceptions().contains_key(&retained_exception));
}

#[test]
fn gp005_exception_path_rejects_other_declared_same_domain_source() {
    let mut case = exception_case();
    let wrong_binding = ExceptionBinding::bind(
        case.retained_exception.clone(),
        case.alternate_source.clone(),
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis
        .exception_bindings
        .insert(case.exception_role.clone(), wrong_binding);

    assert!(matches!(
        case.profile.qualify(
            WorkflowRef::new("workflow:refund-dispute:exception:2").unwrap(),
            case.basis,
        ),
        Err(TerminalClosureQualificationError::ExceptionBoundaryMismatch { .. })
    ));
}

#[test]
fn gp005_exception_path_rejects_undeclared_compensation() {
    let mut case = exception_case();
    let compensation = CompensationBinding::bind(
        refund_intent("finance.refund", "USD-cent:5000:customer-a:order-1"),
        case.alternate_source.clone(),
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis
        .compensation_bindings
        .insert(case.exception_role.clone(), compensation);

    assert!(matches!(
        case.profile.qualify(
            WorkflowRef::new("workflow:refund-dispute:exception:3").unwrap(),
            case.basis,
        ),
        Err(TerminalClosureQualificationError::CompensationRoleSetMismatch { .. })
    ));
}
