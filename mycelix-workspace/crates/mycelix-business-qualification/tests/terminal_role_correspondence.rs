// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Regression target for #276.
//!
//! Exact compensation/exception provenance is necessary but not sufficient.
//! A reusable closure policy must eventually prove that the supplied terminal
//! binding occupies the exact terminal role required by that policy. These tests
//! deliberately keep both the intended and substituted sources as declared
//! boundary roles, so proof-manifest accounting cannot explain the denial.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, CommittedIntent, CompensationBinding, DerivationNodeRef,
    DomainExceptionRef, DomainRef, DomainScopedRef, ExceptionBinding, ExceptionRef,
    LogicalIntentRef, OperationCommitment, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
    WorkflowClosureReceipt, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationError,
    ClosureQualificationProfile, QualifiedBoundaryRef, QualifiedBoundaryRequirement,
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

fn compensation_intent(profile_name: &str, commitment: &str) -> CommittedIntent {
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

struct CompensatedFixture {
    profile: ClosureQualificationProfile,
    cut: QualificationCut,
    intended_source: QualifiedInputRef,
    alternate_source: QualifiedInputRef,
}

fn compensated_fixture() -> CompensatedFixture {
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

    let profile = ClosureQualificationProfile::new(
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

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(intended_source.clone()).unwrap());
    assert!(cut.insert_input(alternate_source.clone()).unwrap());

    CompensatedFixture {
        profile,
        cut,
        intended_source,
        alternate_source,
    }
}

fn qualify_compensated(
    fixture: CompensatedFixture,
    intent: CommittedIntent,
    source: QualifiedInputRef,
) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
    let compensation = CompensationBinding::bind(intent, source, &fixture.cut).unwrap();

    fixture.profile.qualify(
        WorkflowRef::new("workflow:terminal-role:compensated:1").unwrap(),
        ClosureQualificationBasis {
            qualification_cut: fixture.cut,
            boundary_results: BTreeMap::from([
                (
                    node("finance:required-refund-effect"),
                    QualifiedBoundaryRef::Input(fixture.intended_source),
                ),
                (
                    node("finance:other-qualified-result"),
                    QualifiedBoundaryRef::Input(fixture.alternate_source),
                ),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::from([compensation]),
        },
    )
}

#[test]
fn compensated_control_with_intended_source_is_structurally_valid() {
    let fixture = compensated_fixture();
    let intended_source = fixture.intended_source.clone();
    let result = qualify_compensated(
        fixture,
        compensation_intent("finance.refund", "USD-cent:5000:customer-a"),
        intended_source,
    );
    assert!(result.is_ok());
}

#[test]
fn compensated_profile_cannot_accept_other_declared_finance_source_as_terminal_evidence() {
    let fixture = compensated_fixture();
    let alternate_source = fixture.alternate_source.clone();
    let result = qualify_compensated(
        fixture,
        compensation_intent("finance.refund", "USD-cent:5000:customer-a"),
        alternate_source,
    );

    assert!(
        result.is_err(),
        "exact cut membership plus a declared boundary is not sufficient terminal-role correspondence"
    );
}

#[test]
fn compensated_profile_cannot_accept_wrong_operation_profile_for_required_terminal_role() {
    let fixture = compensated_fixture();
    let intended_source = fixture.intended_source.clone();
    let result = qualify_compensated(
        fixture,
        compensation_intent("finance.credit", "USD-cent:5000:customer-a"),
        intended_source,
    );

    assert!(
        result.is_err(),
        "a reusable refund-compensation role must not accept an arbitrary operation semantic profile"
    );
}

struct ExceptionFixture {
    profile: ClosureQualificationProfile,
    cut: QualificationCut,
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

    let profile = ClosureQualificationProfile::new(
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
        intended_source,
        alternate_source,
        retained_exception,
    }
}

fn qualify_exception(
    fixture: ExceptionFixture,
    source: QualifiedInputRef,
) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
    let binding = ExceptionBinding::bind(
        fixture.retained_exception,
        source,
        &fixture.cut,
    )
    .unwrap();

    fixture.profile.qualify(
        WorkflowRef::new("workflow:terminal-role:exception:1").unwrap(),
        ClosureQualificationBasis {
            qualification_cut: fixture.cut,
            boundary_results: BTreeMap::from([
                (
                    node("finance:required-retained-exception"),
                    QualifiedBoundaryRef::Input(fixture.intended_source),
                ),
                (
                    node("finance:other-dispute-result"),
                    QualifiedBoundaryRef::Input(fixture.alternate_source),
                ),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            exception_bindings: vec![binding],
            compensating_intents: BTreeSet::new(),
        },
    )
}

#[test]
fn resolved_exception_control_with_intended_source_is_structurally_valid() {
    let fixture = exception_fixture();
    let intended_source = fixture.intended_source.clone();
    assert!(qualify_exception(fixture, intended_source).is_ok());
}

#[test]
fn resolved_exception_profile_cannot_accept_other_declared_same_domain_source() {
    let fixture = exception_fixture();
    let alternate_source = fixture.alternate_source.clone();
    let result = qualify_exception(fixture, alternate_source);

    assert!(
        result.is_err(),
        "same-domain exact provenance must still correspond to the policy-required terminal role"
    );
}
