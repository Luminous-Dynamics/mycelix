// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Regression target for #268.
//!
//! Two distinct policy-required obligation roles must not silently collapse onto
//! one exact obligation merely because they share an authoritative domain and
//! exact disposition result. Evidence reuse and normative-duty identity reuse
//! are different semantics.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, DerivationNodeRef, DomainObligationRef, DomainRef,
    DomainScopedRef, ObligationDisposition, ObligationDispositionBinding, ObligationRef,
    OrganizationContextRef, QualificationCut, QualifiedInputRef, RecordRef, SemanticProfileId,
    TimestampMs, ValidityEnd, ValidityWindow, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationProfile,
    QualifiedBoundaryRef, QualifiedBoundaryRequirement,
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

fn obligation(domain_name: &str, local_id: &str) -> DomainObligationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ObligationRef::new(local_id).expect("valid obligation"),
    )
}

#[test]
fn distinct_required_roles_cannot_alias_one_exact_obligation() {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.alias-regression.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:alias-regression:v1",
        "governance.closure-policy-active",
    );
    let shared_disposition = input(
        "finance",
        "batch-disposition:shared",
        "finance.obligation-disposition",
    );
    let shared_obligation = obligation("finance", "obligation:shared");

    let root = node("business:alias-regression-close");
    let first_role = node("finance:first-required-duty");
    let second_role = node("finance:second-required-duty");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), first_role.clone());
    graph.add_dependency(root.clone(), second_role.clone());

    let boundary_requirements = BTreeMap::from([
        (
            first_role.clone(),
            QualifiedBoundaryRequirement::input(
                domain("finance"),
                semantic("finance.obligation-disposition"),
            ),
        ),
        (
            second_role.clone(),
            QualifiedBoundaryRequirement::input(
                domain("finance"),
                semantic("finance.obligation-disposition"),
            ),
        ),
    ]);

    let profile = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:alias-regression:satisfied").unwrap(),
        semantic("business.alias-regression.close"),
        ClosureClass::Satisfied,
        policy_source.clone(),
        root,
        graph,
        boundary_requirements,
        BTreeMap::from([
            (first_role.clone(), domain("finance")),
            (second_role.clone(), domain("finance")),
        ]),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(shared_disposition.clone()).unwrap());

    let disposition = ObligationDispositionBinding::bind(
        shared_obligation.clone(),
        ObligationDisposition::Satisfied,
        shared_disposition.clone(),
        &cut,
    )
    .unwrap();

    let basis = ClosureQualificationBasis {
        qualification_cut: cut,
        boundary_results: BTreeMap::from([
            (
                first_role.clone(),
                QualifiedBoundaryRef::Input(shared_disposition.clone()),
            ),
            (
                second_role.clone(),
                QualifiedBoundaryRef::Input(shared_disposition),
            ),
        ]),
        obligations: BTreeMap::from([
            (first_role, shared_obligation.clone()),
            (second_role, shared_obligation),
        ]),
        disposition_bindings: vec![disposition],
        exception_bindings: Vec::new(),
        compensating_intents: BTreeSet::new(),
    };

    let result = profile.qualify(
        WorkflowRef::new("workflow:alias-regression:1").unwrap(),
        basis,
    );

    assert!(
        result.is_err(),
        "distinct policy-required obligation roles must not collapse onto one exact obligation"
    );
}
