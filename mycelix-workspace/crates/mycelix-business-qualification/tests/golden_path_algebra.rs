// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Compact cross-Golden-Path qualification corpus.
//!
//! The larger GP-002/003/006 fixtures remain as adversarial detail. This file
//! asks one narrower architectural question: can materially different Business
//! closures use the same reusable profile/basis algebra without losing their
//! distinct policy, evidence, and closure-class shapes?

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, ClosureRequirements, DerivationNodeRef, DomainObligationRef,
    DomainReconciliationRef, DomainRef, DomainScopedRef, ObligationDisposition,
    ObligationDispositionBinding, ObligationRef, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd,
    ValidityWindow, WorkflowClosureReceipt, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationError,
    ClosureQualificationProfile, QualifiedBoundaryRef,
};

fn semantic(name: &str) -> SemanticProfileId {
    SemanticProfileId::new(name, 1).expect("static semantic profile is valid")
}

fn org() -> OrganizationContextRef {
    OrganizationContextRef::new("org:acme").expect("static organization is valid")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("static domain is valid")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("static derivation node is valid")
}

fn input(domain_name: &str, record_name: &str, profile_name: &str) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("static record is valid"),
        version: 1,
        semantic_profile: semantic(profile_name),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(100)),
        )
        .expect("static validity window is valid"),
    }
}

fn obligation(domain_name: &str, local_id: &str) -> DomainObligationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ObligationRef::new(local_id).expect("static obligation is valid"),
    )
}

fn reconciliation(domain_name: &str, local_id: &str) -> DomainReconciliationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ReconciliationRef::new(local_id).expect("static reconciliation is valid"),
    )
}

fn policy_source(record_name: &str, profile_name: &str) -> QualifiedInputRef {
    input("governance", record_name, profile_name)
}

fn cut(profile_name: &str, policy: &QualifiedInputRef) -> QualificationCut {
    let mut cut = QualificationCut::new(org(), semantic(profile_name), TimestampMs::new(80));
    assert!(cut.insert_input(policy.clone()).unwrap());
    cut
}

fn insert_inputs(cut: &mut QualificationCut, inputs: &[QualifiedInputRef]) {
    for exact in inputs {
        assert!(cut.insert_input(exact.clone()).unwrap());
    }
}

fn bind_disposition(
    obligation: DomainObligationRef,
    disposition: ObligationDisposition,
    source: &QualifiedInputRef,
    cut: &QualificationCut,
) -> ObligationDispositionBinding {
    ObligationDispositionBinding::bind(obligation, disposition, source.clone(), cut)
        .expect("fixture disposition source is exact")
}

fn profile(
    qualification_profile: &str,
    closure_policy: &str,
    closure_profile: &str,
    class: ClosureClass,
    policy: QualifiedInputRef,
    root: &str,
) -> ClosureQualificationProfile {
    ClosureQualificationProfile::new(
        org(),
        semantic(qualification_profile),
        ClosurePolicyRef::new(closure_policy).expect("static closure policy is valid"),
        semantic(closure_profile),
        class,
        policy,
        node(root),
    )
}

struct GoldenPathCase {
    workflow: WorkflowRef,
    profile: ClosureQualificationProfile,
    basis: ClosureQualificationBasis,
}

impl GoldenPathCase {
    fn qualify(self) -> Result<WorkflowClosureReceipt, ClosureQualificationError> {
        self.profile.qualify(self.workflow, self.basis)
    }
}

fn service_sale_case() -> GoldenPathCase {
    let policy = policy_source(
        "closure-policy:service-sale:satisfied:v1",
        "governance.service-sale-closure-policy-active",
    );
    let agreement = input("commerce", "service-agreement:1", "commerce.agreement-accepted");
    let invoice = input("commerce", "invoice:1", "commerce.invoice-projection-accepted");
    let work_disposition = input(
        "service-work",
        "work-disposition:agreement-1",
        "service-work.obligation-disposition",
    );
    let payment_disposition = input(
        "finance",
        "payment-disposition:invoice-1",
        "finance.obligation-disposition",
    );
    let accounting = input(
        "accounting",
        "economic-event:service-sale-1",
        "accounting.accepted-economic-event",
    );
    let finance_reconciliation = reconciliation("finance", "reconciliation:invoice-1");
    let payment_obligation = obligation("finance", "payment:invoice-1");
    let work_obligation = obligation("service-work", "work:agreement-1");

    let mut exact_cut = cut("business.service-sale.qualification", &policy);
    insert_inputs(
        &mut exact_cut,
        &[
            agreement.clone(),
            invoice.clone(),
            work_disposition.clone(),
            payment_disposition.clone(),
            accounting.clone(),
        ],
    );
    assert!(exact_cut.insert_reconciliation(finance_reconciliation.clone()));

    let dispositions = vec![
        bind_disposition(
            payment_obligation.clone(),
            ObligationDisposition::Satisfied,
            &payment_disposition,
            &exact_cut,
        ),
        bind_disposition(
            work_obligation.clone(),
            ObligationDisposition::Satisfied,
            &work_disposition,
            &exact_cut,
        ),
    ];

    let root = node("business:service-sale-closure");
    let agreement_node = node("commerce:accepted-agreement");
    let invoice_node = node("commerce:accepted-invoice");
    let work_node = node("service-work:work-disposition");
    let payment_node = node("finance:payment-disposition");
    let reconciliation_node = node("finance:reconciliation");
    let accounting_node = node("accounting:accepted-economic-event");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), invoice_node.clone());
    graph.add_dependency(invoice_node.clone(), agreement_node.clone());
    graph.add_dependency(root.clone(), work_node.clone());
    graph.add_dependency(root.clone(), payment_node.clone());
    graph.add_dependency(root, accounting_node.clone());
    graph.add_dependency(accounting_node.clone(), reconciliation_node.clone());

    let boundaries = BTreeMap::from([
        (agreement_node, QualifiedBoundaryRef::Input(agreement)),
        (invoice_node, QualifiedBoundaryRef::Input(invoice)),
        (work_node, QualifiedBoundaryRef::Input(work_disposition)),
        (
            payment_node,
            QualifiedBoundaryRef::Input(payment_disposition),
        ),
        (
            reconciliation_node,
            QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
        ),
        (accounting_node, QualifiedBoundaryRef::Input(accounting)),
    ]);

    GoldenPathCase {
        workflow: WorkflowRef::new("workflow:service-sale:1").unwrap(),
        profile: profile(
            "business.service-sale.qualification",
            "closure:service-sale:satisfied",
            "business.service-sale.close",
            ClosureClass::Satisfied,
            policy,
            "business:service-sale-closure",
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            dependency_graph: graph,
            boundary_results: boundaries,
            closure_requirements: ClosureRequirements::new(BTreeSet::from([
                payment_obligation,
                work_obligation,
            ])),
            disposition_bindings: dispositions,
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    }
}

fn procurement_satisfied_case() -> GoldenPathCase {
    let policy = policy_source(
        "closure-policy:procurement:satisfied:v1",
        "governance.procurement-satisfied-policy-active",
    );
    let purchase_order = input("commerce", "purchase-order:1", "commerce.purchase-order-accepted");
    let receiving = input(
        "supply-chain",
        "receiving:purchase-order-1",
        "supply-chain.receiving-accepted",
    );
    let bill_match = input(
        "commerce",
        "bill-match:purchase-order-1",
        "commerce.bill-match-accepted",
    );
    let settlement = input(
        "finance",
        "settlement:supplier-invoice-1",
        "finance.settlement-accepted",
    );
    let payment_disposition = input(
        "finance",
        "payment-disposition:supplier-invoice-1",
        "finance.obligation-disposition",
    );
    let accounting = input(
        "accounting",
        "economic-event:procurement-1",
        "accounting.accepted-economic-event",
    );
    let finance_reconciliation = reconciliation("finance", "reconciliation:supplier-invoice-1");
    let payment_obligation = obligation("finance", "payment:supplier-invoice-1");
    let receiving_obligation = obligation("supply-chain", "receive:purchase-order-1");

    let mut exact_cut = cut("business.procurement.qualification", &policy);
    insert_inputs(
        &mut exact_cut,
        &[
            purchase_order.clone(),
            receiving.clone(),
            bill_match.clone(),
            settlement.clone(),
            payment_disposition.clone(),
            accounting.clone(),
        ],
    );
    assert!(exact_cut.insert_reconciliation(finance_reconciliation.clone()));

    let dispositions = vec![
        bind_disposition(
            payment_obligation.clone(),
            ObligationDisposition::Satisfied,
            &payment_disposition,
            &exact_cut,
        ),
        bind_disposition(
            receiving_obligation.clone(),
            ObligationDisposition::Satisfied,
            &receiving,
            &exact_cut,
        ),
    ];

    let root = node("business:procurement-satisfied-closure");
    let po_node = node("commerce:accepted-purchase-order");
    let receiving_node = node("supply-chain:accepted-receiving");
    let bill_match_node = node("commerce:accepted-bill-match");
    let settlement_node = node("finance:accepted-settlement");
    let reconciliation_node = node("finance:reconciliation");
    let payment_node = node("finance:payment-disposition");
    let accounting_node = node("accounting:accepted-procurement-event");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), bill_match_node.clone());
    graph.add_dependency(bill_match_node.clone(), receiving_node.clone());
    graph.add_dependency(receiving_node.clone(), po_node.clone());
    graph.add_dependency(root.clone(), payment_node.clone());
    graph.add_dependency(root, accounting_node.clone());
    graph.add_dependency(accounting_node.clone(), reconciliation_node.clone());
    graph.add_dependency(reconciliation_node.clone(), settlement_node.clone());

    let boundaries = BTreeMap::from([
        (po_node, QualifiedBoundaryRef::Input(purchase_order)),
        (receiving_node, QualifiedBoundaryRef::Input(receiving)),
        (bill_match_node, QualifiedBoundaryRef::Input(bill_match)),
        (settlement_node, QualifiedBoundaryRef::Input(settlement)),
        (
            reconciliation_node,
            QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
        ),
        (
            payment_node,
            QualifiedBoundaryRef::Input(payment_disposition),
        ),
        (accounting_node, QualifiedBoundaryRef::Input(accounting)),
    ]);

    GoldenPathCase {
        workflow: WorkflowRef::new("workflow:procurement:1").unwrap(),
        profile: profile(
            "business.procurement.qualification",
            "closure:procurement:satisfied",
            "business.procurement.close.satisfied",
            ClosureClass::Satisfied,
            policy,
            "business:procurement-satisfied-closure",
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            dependency_graph: graph,
            boundary_results: boundaries,
            closure_requirements: ClosureRequirements::new(BTreeSet::from([
                payment_obligation,
                receiving_obligation,
            ])),
            disposition_bindings: dispositions,
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    }
}

fn procurement_terminated_case() -> GoldenPathCase {
    let policy = policy_source(
        "closure-policy:procurement:terminated:v1",
        "governance.procurement-terminated-policy-active",
    );
    let cancellation = input(
        "finance",
        "payment-cancellation:supplier-invoice-1",
        "finance.payment-cancellation-accepted",
    );
    let payment_disposition = input(
        "finance",
        "payment-disposition:supplier-invoice-1:terminated",
        "finance.obligation-disposition",
    );
    let receiving_disposition = input(
        "supply-chain",
        "receiving-disposition:purchase-order-1:terminated",
        "supply-chain.obligation-disposition",
    );
    let accounting = input(
        "accounting",
        "economic-event:procurement-cancellation-1",
        "accounting.accepted-cancellation-event",
    );
    let payment_obligation = obligation("finance", "payment:supplier-invoice-1");
    let receiving_obligation = obligation("supply-chain", "receive:purchase-order-1");

    let mut exact_cut = cut("business.procurement.qualification", &policy);
    insert_inputs(
        &mut exact_cut,
        &[
            cancellation.clone(),
            payment_disposition.clone(),
            receiving_disposition.clone(),
            accounting.clone(),
        ],
    );

    let dispositions = vec![
        bind_disposition(
            payment_obligation.clone(),
            ObligationDisposition::Terminated,
            &payment_disposition,
            &exact_cut,
        ),
        bind_disposition(
            receiving_obligation.clone(),
            ObligationDisposition::Terminated,
            &receiving_disposition,
            &exact_cut,
        ),
    ];

    let root = node("business:procurement-terminated-closure");
    let cancellation_node = node("finance:accepted-payment-cancellation");
    let payment_node = node("finance:terminated-payment-obligation");
    let receiving_node = node("supply-chain:terminated-receiving-obligation");
    let accounting_node = node("accounting:accepted-procurement-cancellation");

    let mut graph = ClosureDependencyGraph::default();
    for boundary in [
        cancellation_node.clone(),
        payment_node.clone(),
        receiving_node.clone(),
        accounting_node.clone(),
    ] {
        graph.add_dependency(root.clone(), boundary);
    }

    let boundaries = BTreeMap::from([
        (cancellation_node, QualifiedBoundaryRef::Input(cancellation)),
        (
            payment_node,
            QualifiedBoundaryRef::Input(payment_disposition),
        ),
        (
            receiving_node,
            QualifiedBoundaryRef::Input(receiving_disposition),
        ),
        (accounting_node, QualifiedBoundaryRef::Input(accounting)),
    ]);

    GoldenPathCase {
        workflow: WorkflowRef::new("workflow:procurement:1").unwrap(),
        profile: profile(
            "business.procurement.qualification",
            "closure:procurement:terminated",
            "business.procurement.close.terminated",
            ClosureClass::Terminated,
            policy,
            "business:procurement-terminated-closure",
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            dependency_graph: graph,
            boundary_results: boundaries,
            closure_requirements: ClosureRequirements::new(BTreeSet::from([
                payment_obligation,
                receiving_obligation,
            ])),
            disposition_bindings: dispositions,
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    }
}

fn month_end_case() -> GoldenPathCase {
    let policy = policy_source(
        "closure-policy:month-end:satisfied:v1",
        "governance.month-end-closure-policy-active",
    );
    let accounting_close = input(
        "accounting",
        "period-close:2026-08",
        "accounting.qualified-period-close",
    );

    let mut exact_cut = cut("business.month-end.qualification", &policy);
    assert!(exact_cut.insert_input(accounting_close.clone()).unwrap());

    let root = node("business:month-end-orchestration-close");
    let accounting_node = node("accounting:qualified-period-close");
    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root, accounting_node.clone());

    GoldenPathCase {
        workflow: WorkflowRef::new("workflow:month-end:2026-08").unwrap(),
        profile: profile(
            "business.month-end.qualification",
            "closure:month-end:satisfied",
            "business.month-end.close",
            ClosureClass::Satisfied,
            policy,
            "business:month-end-orchestration-close",
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            dependency_graph: graph,
            boundary_results: BTreeMap::from([(
                accounting_node,
                QualifiedBoundaryRef::Input(accounting_close),
            )]),
            closure_requirements: ClosureRequirements::new(BTreeSet::new()),
            disposition_bindings: Vec::new(),
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        },
    }
}

#[test]
fn one_profile_basis_algebra_expresses_three_materially_different_golden_paths() {
    let service_sale = service_sale_case().qualify().unwrap();
    let procurement_satisfied = procurement_satisfied_case().qualify().unwrap();
    let procurement_terminated = procurement_terminated_case().qualify().unwrap();
    let month_end = month_end_case().qualify().unwrap();

    assert_eq!(service_sale.class(), ClosureClass::Satisfied);
    assert_eq!(service_sale.obligations().len(), 2);

    assert_eq!(procurement_satisfied.class(), ClosureClass::Satisfied);
    assert_eq!(procurement_satisfied.obligations().len(), 2);
    assert_eq!(procurement_satisfied.qualification_cut().reconciliations().len(), 1);

    assert_eq!(procurement_terminated.class(), ClosureClass::Terminated);
    assert_eq!(procurement_terminated.obligations().len(), 2);
    assert!(procurement_terminated
        .qualification_cut()
        .reconciliations()
        .is_empty());

    assert_eq!(month_end.class(), ClosureClass::Satisfied);
    assert!(month_end.obligations().is_empty());
    assert_eq!(month_end.qualification_cut().inputs().len(), 2);
}

#[test]
fn procurement_terminal_classes_keep_distinct_proof_shapes() {
    let satisfied = procurement_satisfied_case().qualify().unwrap();
    let terminated = procurement_terminated_case().qualify().unwrap();

    assert_eq!(satisfied.class(), ClosureClass::Satisfied);
    assert_eq!(terminated.class(), ClosureClass::Terminated);
    assert_eq!(satisfied.qualification_cut().reconciliations().len(), 1);
    assert!(terminated.qualification_cut().reconciliations().is_empty());
    assert_ne!(satisfied.closure_profile(), terminated.closure_profile());
    assert_ne!(satisfied.closure_policy(), terminated.closure_policy());
}

#[test]
fn service_sale_uses_reachable_internal_boundaries_without_flattening_them_to_leaves() {
    let receipt = service_sale_case().qualify().unwrap();

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.qualification_cut().reconciliations().len(), 1);
    assert!(receipt
        .qualification_cut()
        .inputs()
        .iter()
        .any(|exact| exact.record.as_str() == "invoice:1"));
    assert!(receipt
        .qualification_cut()
        .inputs()
        .iter()
        .any(|exact| exact.record.as_str() == "economic-event:service-sale-1"));
}

#[test]
fn month_end_preserves_proof_compression_at_the_business_boundary() {
    let receipt = month_end_case().qualify().unwrap();

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.qualification_cut().inputs().len(), 2);
    assert!(receipt
        .qualification_cut()
        .inputs()
        .iter()
        .any(|exact| exact.record.as_str() == "period-close:2026-08"));
    assert!(receipt.qualification_cut().reconciliations().is_empty());
}
