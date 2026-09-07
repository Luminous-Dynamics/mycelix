// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, CommittedIntent, CompensationBinding, DerivationNodeRef,
    DomainExceptionRef, DomainRef, DomainScopedRef, ExceptionBinding, ExceptionRef,
    LogicalIntentRef, OperationCommitment, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
    WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationProfile, QualifiedBoundaryRef,
    QualifiedBoundaryRequirement,
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

fn input(domain_name: &str, record_name: &str, semantic_name: &str) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: semantic(semantic_name),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(100)),
        )
        .expect("valid validity window"),
    }
}

fn intent(profile_name: &str, commitment: &str) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("refund:order-1").unwrap(),
        semantic(profile_name),
        OperationCommitment::new(commitment).unwrap(),
    )
}

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

struct CompensationFixture {
    profile: TerminalClosureQualificationProfile,
    cut: QualificationCut,
    intended_role: DerivationNodeRef,
    alternate_role: DerivationNodeRef,
    intended_source: QualifiedInputRef,
    alternate_source: QualifiedInputRef,
}

fn compensation_fixture() -> CompensationFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.terminal-role.compensated.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:terminal-role:compensated:v1",
        "governance.closure-policy-active",
    );
    let intended_source = input(
        "finance",
        "refund-effect:required",
        "finance.accepted-refund-effect",
    );
    let alternate_source = input(
        "finance",
        "refund-audit-result:other",
        "finance.accepted-refund-audit",
    );
    let root = node("business:compensated-close");
    let intended_role = node("finance:required-refund-effect");
    let alternate_role = node("finance:other-qualified-result");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), intended_role.clone());
    graph.add_dependency(root.clone(), alternate_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:terminal-role:compensated").unwrap(),
        semantic("business.terminal-role.compensated.close"),
        ClosureClass::Compensated,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                intended_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.accepted-refund-effect"),
                ),
            ),
            (
                alternate_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.accepted-refund-audit"),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let profile = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::from([(
            intended_role.clone(),
            CompensationRoleRequirement::new(semantic("finance.refund")),
        )]),
        BTreeSet::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(intended_source.clone()).unwrap());
    assert!(cut.insert_input(alternate_source.clone()).unwrap());

    CompensationFixture {
        profile,
        cut,
        intended_role,
        alternate_role,
        intended_source,
        alternate_source,
    }
}

fn compensation_basis(
    fixture: &CompensationFixture,
    binding: CompensationBinding,
) -> TerminalClosureQualificationBasis {
    TerminalClosureQualificationBasis {
        qualification_cut: fixture.cut.clone(),
        boundary_results: BTreeMap::from([
            (
                fixture.intended_role.clone(),
                QualifiedBoundaryRef::Input(fixture.intended_source.clone()),
            ),
            (
                fixture.alternate_role.clone(),
                QualifiedBoundaryRef::Input(fixture.alternate_source.clone()),
            ),
        ]),
        obligations: BTreeMap::new(),
        disposition_bindings: Vec::new(),
        compensation_bindings: BTreeMap::from([(fixture.intended_role.clone(), binding)]),
        exception_bindings: BTreeMap::new(),
    }
}

#[test]
fn compensated_role_accepts_exact_declared_source_and_operation_profile() {
    let fixture = compensation_fixture();
    let binding = CompensationBinding::bind(
        intent("finance.refund", "USD-cent:5000:customer-a"),
        fixture.intended_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let basis = compensation_basis(&fixture, binding);

    let receipt = fixture
        .profile
        .qualify(WorkflowRef::new("workflow:terminal:compensated:1").unwrap(), basis)
        .unwrap();
    assert_eq!(receipt.class(), ClosureClass::Compensated);
}

#[test]
fn compensated_role_rejects_other_declared_same_domain_source() {
    let fixture = compensation_fixture();
    let binding = CompensationBinding::bind(
        intent("finance.refund", "USD-cent:5000:customer-a"),
        fixture.alternate_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let basis = compensation_basis(&fixture, binding);

    assert!(matches!(
        fixture
            .profile
            .qualify(WorkflowRef::new("workflow:terminal:compensated:2").unwrap(), basis),
        Err(TerminalClosureQualificationError::CompensationBoundaryMismatch { .. })
    ));
}

#[test]
fn compensated_role_rejects_wrong_operation_semantic_profile() {
    let fixture = compensation_fixture();
    let binding = CompensationBinding::bind(
        intent("finance.credit", "USD-cent:5000:customer-a"),
        fixture.intended_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let basis = compensation_basis(&fixture, binding);

    assert!(matches!(
        fixture
            .profile
            .qualify(WorkflowRef::new("workflow:terminal:compensated:3").unwrap(), basis),
        Err(TerminalClosureQualificationError::CompensationOperationProfileMismatch { .. })
    ));
}

struct ExceptionFixture {
    profile: TerminalClosureQualificationProfile,
    cut: QualificationCut,
    intended_role: DerivationNodeRef,
    alternate_role: DerivationNodeRef,
    intended_source: QualifiedInputRef,
    alternate_source: QualifiedInputRef,
    retained_exception: DomainExceptionRef,
}

fn exception_fixture() -> ExceptionFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.terminal-role.exception.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:terminal-role:exception:v1",
        "governance.closure-policy-active",
    );
    let intended_source = input(
        "finance",
        "chargeback:pending:required",
        "finance.chargeback-pending",
    );
    let alternate_source = input(
        "finance",
        "dispute-note:other",
        "finance.dispute-note-accepted",
    );
    let retained_exception = exception("finance", "chargeback:pending:1");
    let root = node("business:resolved-with-exception-close");
    let intended_role = node("finance:required-retained-exception");
    let alternate_role = node("finance:other-dispute-result");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), intended_role.clone());
    graph.add_dependency(root.clone(), alternate_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:terminal-role:exception").unwrap(),
        semantic("business.terminal-role.exception.close"),
        ClosureClass::ResolvedWithExceptions,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                intended_role.clone(),
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
        ]),
        BTreeMap::new(),
    );
    let profile = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::new(),
        BTreeSet::from([intended_role.clone()]),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(intended_source.clone()).unwrap());
    assert!(cut.insert_input(alternate_source.clone()).unwrap());
    assert!(cut.insert_exception(retained_exception.clone()));

    ExceptionFixture {
        profile,
        cut,
        intended_role,
        alternate_role,
        intended_source,
        alternate_source,
        retained_exception,
    }
}

fn exception_basis(
    fixture: &ExceptionFixture,
    binding: ExceptionBinding,
) -> TerminalClosureQualificationBasis {
    TerminalClosureQualificationBasis {
        qualification_cut: fixture.cut.clone(),
        boundary_results: BTreeMap::from([
            (
                fixture.intended_role.clone(),
                QualifiedBoundaryRef::Input(fixture.intended_source.clone()),
            ),
            (
                fixture.alternate_role.clone(),
                QualifiedBoundaryRef::Input(fixture.alternate_source.clone()),
            ),
        ]),
        obligations: BTreeMap::new(),
        disposition_bindings: Vec::new(),
        compensation_bindings: BTreeMap::new(),
        exception_bindings: BTreeMap::from([(fixture.intended_role.clone(), binding)]),
    }
}

#[test]
fn exception_role_accepts_exact_declared_source() {
    let fixture = exception_fixture();
    let binding = ExceptionBinding::bind(
        fixture.retained_exception.clone(),
        fixture.intended_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let basis = exception_basis(&fixture, binding);

    let receipt = fixture
        .profile
        .qualify(WorkflowRef::new("workflow:terminal:exception:1").unwrap(), basis)
        .unwrap();
    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
}

#[test]
fn exception_role_rejects_other_declared_same_domain_source() {
    let fixture = exception_fixture();
    let binding = ExceptionBinding::bind(
        fixture.retained_exception.clone(),
        fixture.alternate_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let basis = exception_basis(&fixture, binding);

    assert!(matches!(
        fixture
            .profile
            .qualify(WorkflowRef::new("workflow:terminal:exception:2").unwrap(), basis),
        Err(TerminalClosureQualificationError::ExceptionBoundaryMismatch { .. })
    ));
}
