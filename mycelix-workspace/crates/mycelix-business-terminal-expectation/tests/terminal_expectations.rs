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
use mycelix_business_terminal_expectation::{
    CompensationExpectationBinding, CompensationExpectationRequirement,
    ExceptionExpectationBinding, ExceptionExpectationRequirement, ExpectedTerminalClosureBasis,
    ExpectedTerminalClosureProfile, TerminalExpectationError,
};
use mycelix_business_terminal_qualification::{
    CompensationRoleRequirement, TerminalClosureQualificationBasis,
    TerminalClosureQualificationProfile,
};

fn semantic(name: &str) -> SemanticProfileId {
    SemanticProfileId::new(name, 1).expect("valid semantic profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("valid node")
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
        .expect("valid validity"),
    }
}

fn refund_intent(commitment: &str) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("refund:order-1").unwrap(),
        semantic("finance.refund"),
        OperationCommitment::new(commitment).unwrap(),
    )
}

fn exception(local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain("finance"),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

struct CompensationFixture {
    profile: ExpectedTerminalClosureProfile,
    cut: QualificationCut,
    resolution_role: DerivationNodeRef,
    refund_role: DerivationNodeRef,
    resolution: QualifiedInputRef,
    refund_source: QualifiedInputRef,
}

fn compensation_fixture(expectation_depends_on_refund: bool) -> CompensationFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.gp005.expectation.compensated.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:gp005:compensated:v1",
        "governance.closure-policy-active",
    );
    let resolution = input(
        "governance",
        "resolution:refund-5000",
        "governance.authorized-refund-resolution",
    );
    let refund_source = input(
        "finance",
        "refund-effect:1",
        "finance.accepted-refund-effect",
    );

    let root = node("business:gp005-compensated-close");
    let resolution_role = node("governance:authorized-refund-resolution");
    let refund_role = node("finance:accepted-refund-effect");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), refund_role.clone());
    if expectation_depends_on_refund {
        graph.add_dependency(resolution_role.clone(), refund_role.clone());
    } else {
        graph.add_dependency(refund_role.clone(), resolution_role.clone());
    }

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:gp005:compensated").unwrap(),
        semantic("business.gp005.compensated.close"),
        ClosureClass::Compensated,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-refund-resolution"),
                ),
            ),
            (
                refund_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.accepted-refund-effect"),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let terminal = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::from([(
            refund_role.clone(),
            CompensationRoleRequirement::new(semantic("finance.refund")),
        )]),
        BTreeSet::new(),
    );
    let profile = ExpectedTerminalClosureProfile::new(
        terminal,
        BTreeMap::from([(
            refund_role.clone(),
            CompensationExpectationRequirement::new(resolution_role.clone()),
        )]),
        BTreeMap::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(resolution.clone()).unwrap());
    assert!(cut.insert_input(refund_source.clone()).unwrap());

    CompensationFixture {
        profile,
        cut,
        resolution_role,
        refund_role,
        resolution,
        refund_source,
    }
}

fn compensation_basis(
    fixture: &CompensationFixture,
    expected: CommittedIntent,
    actual: CommittedIntent,
    expectation_source: QualifiedInputRef,
) -> ExpectedTerminalClosureBasis {
    let compensation = CompensationBinding::bind(
        actual,
        fixture.refund_source.clone(),
        &fixture.cut,
    )
    .unwrap();
    let expectation = CompensationExpectationBinding::bind(
        expected,
        expectation_source,
        &fixture.cut,
    )
    .unwrap();

    ExpectedTerminalClosureBasis {
        terminal_basis: TerminalClosureQualificationBasis {
            qualification_cut: fixture.cut.clone(),
            boundary_results: BTreeMap::from([
                (
                    fixture.resolution_role.clone(),
                    QualifiedBoundaryRef::Input(fixture.resolution.clone()),
                ),
                (
                    fixture.refund_role.clone(),
                    QualifiedBoundaryRef::Input(fixture.refund_source.clone()),
                ),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            compensation_bindings: BTreeMap::from([(
                fixture.refund_role.clone(),
                compensation,
            )]),
            exception_bindings: BTreeMap::new(),
        },
        compensation_expectations: BTreeMap::from([(
            fixture.refund_role.clone(),
            expectation,
        )]),
        exception_expectations: BTreeMap::new(),
    }
}

#[test]
fn exact_gp005_refund_expectation_qualifies() {
    let fixture = compensation_fixture(false);
    let exact = refund_intent("USD-cent:5000:customer-a:order-1");
    let basis = compensation_basis(
        &fixture,
        exact.clone(),
        exact,
        fixture.resolution.clone(),
    );

    let receipt = fixture
        .profile
        .qualify(WorkflowRef::new("workflow:gp005:expected-refund:1").unwrap(), basis)
        .unwrap();
    assert_eq!(receipt.class(), ClosureClass::Compensated);
}

#[test]
fn same_refund_role_profile_and_source_cannot_drift_material_commitment() {
    let fixture = compensation_fixture(false);
    let basis = compensation_basis(
        &fixture,
        refund_intent("USD-cent:5000:customer-a:order-1"),
        refund_intent("USD-cent:6000:customer-a:order-1"),
        fixture.resolution.clone(),
    );

    assert!(matches!(
        fixture.profile.qualify(
            WorkflowRef::new("workflow:gp005:expected-refund:2").unwrap(),
            basis,
        ),
        Err(TerminalExpectationError::CompensationExpectationMismatch { .. })
    ));
}

#[test]
fn compensation_expectation_must_use_exact_authoritative_boundary_source() {
    let fixture = compensation_fixture(false);
    let exact = refund_intent("USD-cent:5000:customer-a:order-1");
    let basis = compensation_basis(
        &fixture,
        exact.clone(),
        exact,
        fixture.refund_source.clone(),
    );

    assert!(matches!(
        fixture.profile.qualify(
            WorkflowRef::new("workflow:gp005:expected-refund:3").unwrap(),
            basis,
        ),
        Err(TerminalExpectationError::CompensationExpectationSourceMismatch { .. })
    ));
}

#[test]
fn expectation_source_cannot_depend_on_terminal_result() {
    let fixture = compensation_fixture(true);
    let exact = refund_intent("USD-cent:5000:customer-a:order-1");
    let basis = compensation_basis(
        &fixture,
        exact.clone(),
        exact,
        fixture.resolution.clone(),
    );

    assert!(matches!(
        fixture.profile.qualify(
            WorkflowRef::new("workflow:gp005:expected-refund:4").unwrap(),
            basis,
        ),
        Err(TerminalExpectationError::ExpectationDependsOnTerminalRole { .. })
    ));
}

#[test]
fn terminal_result_cannot_be_its_own_expectation_role() {
    let fixture = compensation_fixture(false);
    let terminal = fixture.profile.base().clone();
    let self_referential = ExpectedTerminalClosureProfile::new(
        terminal,
        BTreeMap::from([(
            fixture.refund_role.clone(),
            CompensationExpectationRequirement::new(fixture.refund_role.clone()),
        )]),
        BTreeMap::new(),
    );
    let exact = refund_intent("USD-cent:5000:customer-a:order-1");
    let basis = compensation_basis(
        &fixture,
        exact.clone(),
        exact,
        fixture.refund_source.clone(),
    );

    assert!(matches!(
        self_referential.qualify(
            WorkflowRef::new("workflow:gp005:expected-refund:5").unwrap(),
            basis,
        ),
        Err(TerminalExpectationError::ExpectationRoleIsTerminal { .. })
    ));
}

struct ExceptionFixture {
    profile: ExpectedTerminalClosureProfile,
    cut: QualificationCut,
    resolution_role: DerivationNodeRef,
    exception_role: DerivationNodeRef,
    resolution: QualifiedInputRef,
    exception_source: QualifiedInputRef,
}

fn exception_fixture() -> ExceptionFixture {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.gp005.expectation.exception.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:gp005:exception:v1",
        "governance.closure-policy-active",
    );
    let resolution = input(
        "governance",
        "resolution:retain-chargeback-1",
        "governance.authorized-exception-resolution",
    );
    let exception_source = input(
        "finance",
        "chargeback:pending:source",
        "finance.chargeback-pending",
    );

    let root = node("business:gp005-exception-close");
    let resolution_role = node("governance:authorized-exception-resolution");
    let exception_role = node("finance:required-retained-exception");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), exception_role.clone());
    graph.add_dependency(exception_role.clone(), resolution_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:gp005:exception").unwrap(),
        semantic("business.gp005.exception.close"),
        ClosureClass::ResolvedWithExceptions,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([
            (
                resolution_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("governance"),
                    semantic("governance.authorized-exception-resolution"),
                ),
            ),
            (
                exception_role.clone(),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.chargeback-pending"),
                ),
            ),
        ]),
        BTreeMap::new(),
    );
    let terminal = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::new(),
        BTreeSet::from([exception_role.clone()]),
    );
    let profile = ExpectedTerminalClosureProfile::new(
        terminal,
        BTreeMap::new(),
        BTreeMap::from([(
            exception_role.clone(),
            ExceptionExpectationRequirement::new(resolution_role.clone()),
        )]),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(resolution.clone()).unwrap());
    assert!(cut.insert_input(exception_source.clone()).unwrap());

    ExceptionFixture {
        profile,
        cut,
        resolution_role,
        exception_role,
        resolution,
        exception_source,
    }
}

fn exception_basis(
    fixture: &ExceptionFixture,
    expected: DomainExceptionRef,
    actual: DomainExceptionRef,
) -> ExpectedTerminalClosureBasis {
    let mut cut = fixture.cut.clone();
    assert!(cut.insert_exception(actual.clone()));
    let binding = ExceptionBinding::bind(
        actual,
        fixture.exception_source.clone(),
        &cut,
    )
    .unwrap();
    let expectation = ExceptionExpectationBinding::bind(
        expected,
        fixture.resolution.clone(),
        &cut,
    )
    .unwrap();

    ExpectedTerminalClosureBasis {
        terminal_basis: TerminalClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([
                (
                    fixture.resolution_role.clone(),
                    QualifiedBoundaryRef::Input(fixture.resolution.clone()),
                ),
                (
                    fixture.exception_role.clone(),
                    QualifiedBoundaryRef::Input(fixture.exception_source.clone()),
                ),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            compensation_bindings: BTreeMap::new(),
            exception_bindings: BTreeMap::from([(
                fixture.exception_role.clone(),
                binding,
            )]),
        },
        compensation_expectations: BTreeMap::new(),
        exception_expectations: BTreeMap::from([(
            fixture.exception_role.clone(),
            expectation,
        )]),
    }
}

#[test]
fn exact_gp005_exception_expectation_qualifies() {
    let fixture = exception_fixture();
    let exact = exception("chargeback:pending:1");
    let basis = exception_basis(&fixture, exact.clone(), exact);

    let receipt = fixture
        .profile
        .qualify(WorkflowRef::new("workflow:gp005:expected-exception:1").unwrap(), basis)
        .unwrap();
    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
}

#[test]
fn same_exception_source_role_cannot_substitute_instance_identity() {
    let fixture = exception_fixture();
    let basis = exception_basis(
        &fixture,
        exception("chargeback:pending:1"),
        exception("chargeback:pending:2"),
    );

    assert!(matches!(
        fixture.profile.qualify(
            WorkflowRef::new("workflow:gp005:expected-exception:2").unwrap(),
            basis,
        ),
        Err(TerminalExpectationError::ExceptionExpectationMismatch { .. })
    ));
}
