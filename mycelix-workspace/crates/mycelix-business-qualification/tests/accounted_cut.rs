// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Regression target for #277.
//!
//! A closure qualification cut is an exact declared proof basis, not an arbitrary
//! evidence superset. Policy provenance and declared factual boundaries are
//! accounted inputs; unrelated baggage must not silently ride along.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, DerivationNodeRef, DomainRef, OrganizationContextRef,
    QualificationCut, QualifiedInputRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd,
    ValidityWindow, WorkflowClosureReceipt, WorkflowRef,
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

fn qualify_month_end_like_cut(
    include_unaccounted_input: bool,
) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.accounted-cut.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:accounted-cut:v1",
        "governance.closure-policy-active",
    );
    let accounting_close = input(
        "accounting",
        "period-close:2026-08",
        "accounting.accepted-period-close",
    );
    let unrelated_finance_input = input(
        "finance",
        "unrelated-bank-observation:1",
        "finance.unrelated-observation",
    );

    let root = node("business:month-end-orchestration-close");
    let accounting_role = node("accounting:accepted-period-close");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), accounting_role.clone());

    let profile = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:accounted-cut:satisfied").unwrap(),
        semantic("business.accounted-cut.close"),
        ClosureClass::Satisfied,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([(
            accounting_role.clone(),
            QualifiedBoundaryRequirement::input(
                domain("accounting"),
                semantic("accounting.accepted-period-close"),
            ),
        )]),
        BTreeMap::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(accounting_close.clone()).unwrap());
    if include_unaccounted_input {
        assert!(cut.insert_input(unrelated_finance_input).unwrap());
    }

    profile.qualify(
        WorkflowRef::new("workflow:accounted-cut:1").unwrap(),
        ClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([(
                accounting_role,
                QualifiedBoundaryRef::Input(accounting_close),
            )]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    )
}

#[test]
fn declared_policy_and_boundary_inputs_form_valid_minimal_cut() {
    assert!(qualify_month_end_like_cut(false).is_ok());
}

#[test]
fn unrelated_exact_input_cannot_ride_along_in_successful_closure_cut() {
    assert!(matches!(
        qualify_month_end_like_cut(true),
        Err(ClosureQualificationError::UnaccountedCutInputs { .. })
    ));
}
