// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, DerivationNodeRef,
    DomainObligationRef, DomainReconciliationRef, DomainRef, DomainScopedRef,
    ObligationDisposition, ObligationDispositionBinding, ObligationRef, OrganizationContextRef,
    QualificationCut, QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId,
    TimestampMs, ValidityEnd, ValidityWindow, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationError,
    ClosureQualificationProfile, QualifiedBoundaryRef,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid semantic profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("valid derivation node")
}

fn input(domain_name: &str, record_name: &str, semantic: &str) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: profile(semantic, 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(100)),
        )
        .expect("valid window"),
    }
}

fn obligation(domain_name: &str, local_id: &str) -> DomainObligationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ObligationRef::new(local_id).expect("valid obligation"),
    )
}

fn reconciliation(domain_name: &str, local_id: &str) -> DomainReconciliationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ReconciliationRef::new(local_id).expect("valid reconciliation"),
    )
}

fn service_sale_policy_source() -> QualifiedInputRef {
    input(
        "governance",
        "closure-policy:service-sale:v1",
        "governance.closure-policy-active",
    )
}

fn satisfied_profile() -> ClosureQualificationProfile {
    ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        service_sale_policy_source(),
        node("business:service-sale-closure"),
    )
}

fn service_sale_basis() -> ClosureQualificationBasis {
    let agreement = input("commerce", "service-agreement:1", "commerce.agreement");
    let work_disposition_source = input(
        "service-work",
        "work-disposition:1",
        "service-work.obligation-disposition",
    );
    let payment_disposition_source = input(
        "finance",
        "payment-disposition:1",
        "finance.obligation-disposition",
    );
    let accounting = input(
        "accounting",
        "economic-event:service-sale-1",
        "accounting.accepted-economic-event",
    );
    let policy_source = service_sale_policy_source();
    let finance_reconciliation = reconciliation("finance", "reconciliation:invoice-1");
    let payment_obligation = obligation("finance", "payment:invoice-1");
    let work_obligation = obligation("service-work", "work:agreement-1");

    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.service-sale.qualification", 1),
        TimestampMs::new(80),
    );
    for exact in [
        agreement.clone(),
        work_disposition_source.clone(),
        payment_disposition_source.clone(),
        accounting.clone(),
        policy_source,
    ] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_reconciliation(finance_reconciliation.clone()));

    let dispositions = vec![
        ObligationDispositionBinding::bind(
            payment_obligation.clone(),
            ObligationDisposition::Satisfied,
            payment_disposition_source.clone(),
            &cut,
        )
        .unwrap(),
        ObligationDispositionBinding::bind(
            work_obligation.clone(),
            ObligationDisposition::Satisfied,
            work_disposition_source.clone(),
            &cut,
        )
        .unwrap(),
    ];

    let root = node("business:service-sale-closure");
    let agreement_node = node("commerce:service-agreement");
    let work_node = node("service-work:work-disposition");
    let payment_node = node("finance:payment-disposition");
    let reconciliation_node = node("finance:reconciliation");
    let accounting_node = node("accounting:accepted-economic-event");

    let mut graph = ClosureDependencyGraph::default();
    for leaf in [
        agreement_node.clone(),
        work_node.clone(),
        payment_node.clone(),
        reconciliation_node.clone(),
        accounting_node.clone(),
    ] {
        graph.add_dependency(root.clone(), leaf);
    }

    let boundary_results = BTreeMap::from([
        (agreement_node, QualifiedBoundaryRef::Input(agreement)),
        (
            work_node,
            QualifiedBoundaryRef::Input(work_disposition_source),
        ),
        (
            payment_node,
            QualifiedBoundaryRef::Input(payment_disposition_source),
        ),
        (
            reconciliation_node,
            QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
        ),
        (accounting_node, QualifiedBoundaryRef::Input(accounting)),
    ]);

    ClosureQualificationBasis {
        qualification_cut: cut,
        dependency_graph: graph,
        boundary_results,
        closure_requirements: ClosureRequirements::new(BTreeSet::from([
            payment_obligation,
            work_obligation,
        ])),
        disposition_bindings: dispositions,
        exception_bindings: Vec::new(),
        compensating_intents: BTreeSet::new(),
    }
}

#[test]
fn reusable_profile_qualifies_exact_service_sale_basis() {
    let receipt = satisfied_profile()
        .qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            service_sale_basis(),
        )
        .expect("exact structural basis qualifies");

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.obligations().len(), 2);
}

#[test]
fn policy_profile_source_is_required_in_the_exact_cut() {
    let basis = service_sale_basis();
    let absent_policy_source = input(
        "governance",
        "closure-policy:service-sale:v2",
        "governance.closure-policy-active",
    );
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale").unwrap(),
        profile("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        absent_policy_source,
        node("business:service-sale-closure"),
    );

    assert!(matches!(
        profile.qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::PolicySourceMissingFromCut { .. })
    ));
}

#[test]
fn policy_source_is_semantic_authority_input_not_a_factual_dag_leaf() {
    let profile = satisfied_profile();
    let basis = service_sale_basis();

    assert!(basis
        .qualification_cut
        .inputs()
        .contains(profile.policy_source()));
    assert!(basis
        .boundary_results
        .values()
        .all(|boundary| boundary != &QualifiedBoundaryRef::Input(profile.policy_source().clone())));

    profile
        .qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        )
        .expect("policy source may govern the DAG without masquerading as a factual leaf");
}

#[test]
fn profile_prevents_cross_organization_replay() {
    let mut basis = service_sale_basis();
    basis.qualification_cut.organization_context = OrganizationContextRef::new("org:other").unwrap();

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::WrongOrganization { .. })
    ));
}

#[test]
fn profile_prevents_semantic_profile_substitution() {
    let mut basis = service_sale_basis();
    basis.qualification_cut.semantic_profile = profile("business.service-sale.qualification", 2);

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::WrongQualificationProfile { .. })
    ));
}

#[test]
fn cycle_is_rejected_before_receipt_construction() {
    let mut basis = service_sale_basis();
    let root = node("business:service-sale-closure");
    let agreement = node("commerce:service-agreement");
    basis
        .dependency_graph
        .add_dependency(agreement, root.clone());

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::Dependency(_))
    ));
}

#[test]
fn missing_root_is_rejected() {
    let mut basis = service_sale_basis();
    basis.dependency_graph = ClosureDependencyGraph::default();

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::MissingRootNode { .. })
    ));
}

#[test]
fn disconnected_decorative_graph_is_rejected() {
    let mut basis = service_sale_basis();
    basis
        .dependency_graph
        .add_node(node("unrelated:decorative-node"));

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::DisconnectedDependencyGraph { .. })
    ));
}

#[test]
fn every_reachable_leaf_must_have_exact_boundary_provenance() {
    let mut basis = service_sale_basis();
    basis.dependency_graph.add_dependency(
        node("business:service-sale-closure"),
        node("commerce:unbound-document"),
    );

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::BoundaryLeafMismatch { .. })
    ));
}

#[test]
fn boundary_mapping_cannot_point_to_non_leaf_internal_node() {
    let mut basis = service_sale_basis();
    basis.boundary_results.insert(
        node("business:service-sale-closure"),
        QualifiedBoundaryRef::Input(input("commerce", "extra:1", "commerce.extra")),
    );

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::BoundaryLeafMismatch { .. })
    ));
}

#[test]
fn boundary_input_must_exist_in_exact_cut() {
    let mut basis = service_sale_basis();
    let agreement_node = node("commerce:service-agreement");
    basis.boundary_results.insert(
        agreement_node,
        QualifiedBoundaryRef::Input(input(
            "commerce",
            "service-agreement:other",
            "commerce.agreement",
        )),
    );

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::BoundaryInputMissing { .. })
    ));
}

#[test]
fn reconciliation_leaf_must_exist_in_exact_cut() {
    let mut basis = service_sale_basis();
    let reconciliation_node = node("finance:reconciliation");
    basis.boundary_results.insert(
        reconciliation_node,
        QualifiedBoundaryRef::Reconciliation(reconciliation("finance", "reconciliation:other")),
    );

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::BoundaryReconciliationMissing { .. })
    ));
}

#[test]
fn closure_class_is_part_of_reusable_profile_not_caller_selected_basis() {
    let mut basis = service_sale_basis();
    let payment_source = basis
        .qualification_cut
        .inputs()
        .iter()
        .find(|item| item.record.as_str() == "payment-disposition:1")
        .expect("payment disposition input")
        .clone();
    let payment_obligation = obligation("finance", "payment:invoice-1");
    let work_source = basis
        .qualification_cut
        .inputs()
        .iter()
        .find(|item| item.record.as_str() == "work-disposition:1")
        .expect("work disposition input")
        .clone();
    let work_obligation = obligation("service-work", "work:agreement-1");

    basis.disposition_bindings = vec![
        ObligationDispositionBinding::bind(
            payment_obligation,
            ObligationDisposition::Terminated,
            payment_source,
            &basis.qualification_cut,
        )
        .unwrap(),
        ObligationDispositionBinding::bind(
            work_obligation,
            ObligationDisposition::Terminated,
            work_source,
            &basis.qualification_cut,
        )
        .unwrap(),
    ];

    assert!(matches!(
        satisfied_profile().qualify(
            WorkflowRef::new("workflow:service-sale:1").unwrap(),
            basis,
        ),
        Err(ClosureQualificationError::Closure(
            ClosureError::SatisfactionStrengthening { .. }
        ))
    ));
}

#[test]
fn one_accounting_boundary_can_compress_a_deeper_domain_proof_graph() {
    let accounting_close = input(
        "accounting",
        "period-close:2026-08",
        "accounting.qualified-period-close",
    );
    let policy_source = input(
        "governance",
        "closure-policy:month-end:v1",
        "governance.closure-policy-active",
    );
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.month-end.qualification", 1),
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(accounting_close.clone()).unwrap());
    assert!(cut.insert_input(policy_source.clone()).unwrap());

    let root = node("business:month-end-orchestration-close");
    let accounting_node = node("accounting:qualified-period-close");
    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), accounting_node.clone());

    let basis = ClosureQualificationBasis {
        qualification_cut: cut,
        dependency_graph: graph,
        boundary_results: BTreeMap::from([(
            accounting_node,
            QualifiedBoundaryRef::Input(accounting_close),
        )]),
        closure_requirements: ClosureRequirements::new(BTreeSet::new()),
        disposition_bindings: Vec::new(),
        exception_bindings: Vec::new(),
        compensating_intents: BTreeSet::new(),
    };

    assert_eq!(basis.boundary_results.len(), 1);
    assert_eq!(basis.qualification_cut.inputs().len(), 2);

    let month_end = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.month-end.qualification", 1),
        ClosurePolicyRef::new("closure:month-end").unwrap(),
        profile("business.month-end.close", 1),
        ClosureClass::Satisfied,
        policy_source,
        root,
    );

    let receipt = month_end
        .qualify(WorkflowRef::new("workflow:month-end:2026-08").unwrap(), basis)
        .expect("one qualified Accounting boundary is sufficient structurally");

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.qualification_cut().inputs().len(), 2);
}

#[test]
fn same_profile_and_exact_basis_are_deterministic() {
    let profile = satisfied_profile();
    let workflow = WorkflowRef::new("workflow:service-sale:1").unwrap();
    let basis = service_sale_basis();

    let first = profile.qualify(workflow.clone(), basis.clone()).unwrap();
    let second = profile.qualify(workflow, basis).unwrap();
    assert_eq!(first, second);
}
