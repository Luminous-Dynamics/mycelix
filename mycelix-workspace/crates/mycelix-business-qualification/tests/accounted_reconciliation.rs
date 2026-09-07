// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reconciliation-accounting regression target for #277.
//!
//! A declared reconciliation boundary is part of the exact closure proof. An
//! unrelated reconciliation identity must not silently ride along in the same
//! cut merely because it is domain-scoped and otherwise well formed.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, DerivationNodeRef, DomainReconciliationRef, DomainRef,
    DomainScopedRef, OrganizationContextRef, QualificationCut, QualifiedInputRef,
    ReconciliationRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
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

fn reconciliation(domain_name: &str, local_id: &str) -> DomainReconciliationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ReconciliationRef::new(local_id).expect("valid reconciliation"),
    )
}

fn qualify_reconciliation_cut(
    include_unaccounted_reconciliation: bool,
) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
    let organization = OrganizationContextRef::new("org:acme").unwrap();
    let qualification_profile = semantic("business.reconciliation-cut.qualification");
    let policy_source = input(
        "governance",
        "closure-policy:reconciliation-cut:v1",
        "governance.closure-policy-active",
    );
    let required_reconciliation = reconciliation("finance", "settlement:required");
    let unrelated_reconciliation = reconciliation("finance", "settlement:unrelated");

    let root = node("business:reconciliation-close");
    let reconciliation_role = node("finance:required-reconciliation");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), reconciliation_role.clone());

    let profile = ClosureQualificationProfile::new(
        organization.clone(),
        qualification_profile.clone(),
        ClosurePolicyRef::new("closure:reconciliation-cut:satisfied").unwrap(),
        semantic("business.reconciliation-cut.close"),
        ClosureClass::Satisfied,
        policy_source.clone(),
        root,
        graph,
        BTreeMap::from([(
            reconciliation_role.clone(),
            QualifiedBoundaryRequirement::reconciliation(domain("finance")),
        )]),
        BTreeMap::new(),
    );

    let mut cut = QualificationCut::new(
        organization,
        qualification_profile,
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_reconciliation(required_reconciliation.clone()));
    if include_unaccounted_reconciliation {
        assert!(cut.insert_reconciliation(unrelated_reconciliation));
    }

    profile.qualify(
        WorkflowRef::new("workflow:reconciliation-cut:1").unwrap(),
        ClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([(
                reconciliation_role,
                QualifiedBoundaryRef::Reconciliation(required_reconciliation),
            )]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    )
}

#[test]
fn declared_reconciliation_boundary_forms_valid_exact_cut() {
    assert!(qualify_reconciliation_cut(false).is_ok());
}

#[test]
fn unrelated_reconciliation_cannot_ride_along_in_successful_closure_cut() {
    assert!(
        qualify_reconciliation_cut(true).is_err(),
        "every reconciliation in an exact closure cut must occupy a declared reconciliation role"
    );
}
