// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Regression target for #300.
//!
//! Terminal source-role correspondence does not by itself bind the exact
//! instance-specific effect identity required by an authoritative resolution.

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
    TerminalClosureQualificationProfile,
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

fn compensation_result(commitment: &str) -> Result<(), ()> {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.expectation.compensation.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:expectation:compensation:v1",
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
    let root = node("business:expectation-compensated-close");
    let resolution_role = node("governance:authorized-refund-resolution");
    let refund_role = node("finance:required-refund-effect");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), refund_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:expectation:compensated").unwrap(),
        semantic("business.expectation.compensated.close"),
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
    for exact in [policy_source, resolution.clone(), refund_source.clone()] {
        assert!(cut.insert_input(exact).unwrap());
    }
    let compensation = CompensationBinding::bind(
        refund_intent(commitment),
        refund_source.clone(),
        &cut,
    )
    .unwrap();

    profile
        .qualify(
            WorkflowRef::new("workflow:expectation:compensated").unwrap(),
            TerminalClosureQualificationBasis {
                qualification_cut: cut,
                boundary_results: BTreeMap::from([
                    (
                        resolution_role,
                        QualifiedBoundaryRef::Input(resolution),
                    ),
                    (
                        refund_role.clone(),
                        QualifiedBoundaryRef::Input(refund_source),
                    ),
                ]),
                obligations: BTreeMap::new(),
                disposition_bindings: Vec::new(),
                compensation_bindings: BTreeMap::from([(refund_role, compensation)]),
                exception_bindings: BTreeMap::new(),
            },
        )
        .map(|_| ())
        .map_err(|_| ())
}

#[test]
fn exact_expected_refund_commitment_control_is_valid() {
    assert!(compensation_result("USD-cent:5000:customer-a:order-1").is_ok());
}

#[test]
fn same_refund_profile_and_source_cannot_change_material_commitment() {
    assert!(
        compensation_result("USD-cent:6000:customer-a:order-1").is_err(),
        "terminal source-role correspondence must not let an instance-specific refund commitment drift"
    );
}

fn exception_result(exception_id: &str) -> Result<(), ()> {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.expectation.exception.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:expectation:exception:v1",
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
    let root = node("business:expectation-exception-close");
    let resolution_role = node("governance:authorized-exception-resolution");
    let exception_role = node("finance:required-retained-exception");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_role.clone());
    graph.add_dependency(root.clone(), exception_role.clone());

    let base = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:expectation:exception").unwrap(),
        semantic("business.expectation.exception.close"),
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
    let profile = TerminalClosureQualificationProfile::new(
        base,
        BTreeMap::new(),
        BTreeSet::from([exception_role.clone()]),
    );

    let retained_exception = exception(exception_id);
    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    for exact in [policy_source, resolution.clone(), exception_source.clone()] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_exception(retained_exception.clone()));
    let binding = ExceptionBinding::bind(
        retained_exception,
        exception_source.clone(),
        &cut,
    )
    .unwrap();

    profile
        .qualify(
            WorkflowRef::new("workflow:expectation:exception").unwrap(),
            TerminalClosureQualificationBasis {
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
                exception_bindings: BTreeMap::from([(exception_role, binding)]),
            },
        )
        .map(|_| ())
        .map_err(|_| ())
}

#[test]
fn exact_expected_exception_identity_control_is_valid() {
    assert!(exception_result("chargeback:pending:1").is_ok());
}

#[test]
fn same_exception_source_role_cannot_substitute_instance_identity() {
    assert!(
        exception_result("chargeback:pending:2").is_err(),
        "terminal source-role correspondence must not let the exact retained exception identity drift"
    );
}
