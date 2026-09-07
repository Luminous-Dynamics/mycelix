// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Precision companion to the Governance resolution adapter fixture.
//!
//! The cut contains only the *actual* retained exception identity. This ensures
//! that substitution reaches the terminal-expectation equality check instead of
//! failing earlier because both expected and actual exception identities were
//! present in the exact cut.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, DerivationNodeRef, DomainExceptionRef, DomainRef,
    DomainScopedRef, ExceptionBinding, ExceptionRef, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
    WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationProfile, QualifiedBoundaryRef,
    QualifiedBoundaryRequirement,
};
use mycelix_business_terminal_expectation::{
    ExceptionExpectationBinding, ExceptionExpectationRequirement, ExpectedTerminalClosureBasis,
    ExpectedTerminalClosureProfile, TerminalExpectationError,
};
use mycelix_business_terminal_qualification::{
    TerminalClosureQualificationBasis, TerminalClosureQualificationProfile,
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

fn exception(local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain("finance"),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

fn governance_v1_expectation(
    resolution: &QualifiedInputRef,
    cut: &QualificationCut,
) -> ExceptionExpectationBinding {
    assert_eq!(resolution.domain, domain("governance"));
    assert_eq!(
        resolution.semantic_profile,
        semantic("governance.authorized-exception-resolution")
    );
    assert_eq!(resolution.record.as_str(), "resolution:retain-chargeback-order-1");

    ExceptionExpectationBinding::bind(
        exception("chargeback:pending:1"),
        resolution.clone(),
        cut,
    )
    .expect("exact Governance resolution is present in the cut")
}

#[test]
fn substituted_exception_reaches_named_expectation_mismatch() {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.gp005.adapter.precise-exception.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:gp005:adapter:precise-exception:v1",
        "governance.closure-policy-active",
    );
    let resolution = input(
        "governance",
        "resolution:retain-chargeback-order-1",
        "governance.authorized-exception-resolution",
    );
    let exception_source = input(
        "finance",
        "chargeback:pending:order-1",
        "finance.chargeback-pending",
    );

    let root = node("business:gp005-adapter-precise-exception-close");
    let resolution_role = node("governance:authorized-exception-resolution");
    let exception_role = node("finance:required-retained-exception");
    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), exception_role.clone());
    graph.add_dependency(exception_role.clone(), resolution_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:gp005:adapter:precise-exception").unwrap(),
        semantic("business.gp005.adapter.precise-exception.close"),
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

    let actual = exception("chargeback:pending:2");
    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(resolution.clone()).unwrap());
    assert!(cut.insert_input(exception_source.clone()).unwrap());
    assert!(cut.insert_exception(actual.clone()));

    let expectation = governance_v1_expectation(&resolution, &cut);
    let actual_binding = ExceptionBinding::bind(actual, exception_source.clone(), &cut).unwrap();

    let result = profile.qualify(
        WorkflowRef::new("workflow:gp005:adapter-precise-exception:1").unwrap(),
        ExpectedTerminalClosureBasis {
            terminal_basis: TerminalClosureQualificationBasis {
                qualification_cut: cut,
                boundary_results: BTreeMap::from([
                    (
                        resolution_role,
                        QualifiedBoundaryRef::Input(resolution),
                    ),
                    (
                        exception_role.clone(),
                        QualifiedBoundaryRef::Input(exception_source),
                    ),
                ]),
                obligations: BTreeMap::new(),
                disposition_bindings: Vec::new(),
                compensation_bindings: BTreeMap::new(),
                exception_bindings: BTreeMap::from([(
                    exception_role.clone(),
                    actual_binding,
                )]),
            },
            compensation_expectations: BTreeMap::new(),
            exception_expectations: BTreeMap::from([(exception_role, expectation)]),
        },
    );

    assert!(matches!(
        result,
        Err(TerminalExpectationError::ExceptionExpectationMismatch { .. })
    ));
}
