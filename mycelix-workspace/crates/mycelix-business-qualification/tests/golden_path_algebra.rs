// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Compact cross-Golden-Path qualification corpus.
//!
//! The larger GP-002/003/006 fixtures remain as adversarial detail. This file
//! asks one narrower architectural question: can materially different Business
//! closures use the same reusable profile/basis algebra while the reusable
//! profile—not each transaction basis—owns proof topology, boundary roles, and
//! which obligation roles count?

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosurePolicyRef, DerivationNodeRef, DomainObligationRef,
    DomainReconciliationRef, DomainRef, DomainScopedRef, ObligationDisposition,
    ObligationDispositionBinding, ObligationRef, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd,
    ValidityWindow, WorkflowClosureReceipt, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationError,
    ClosureQualificationProfile, QualifiedBoundaryRef, QualifiedBoundaryRequirement,
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

fn input_requirement(domain_name: &str, profile_name: &str) -> QualifiedBoundaryRequirement {
    QualifiedBoundaryRequirement::input(domain(domain_name), semantic(profile_name))
}

fn reconciliation_requirement(domain_name: &str) -> QualifiedBoundaryRequirement {
    QualifiedBoundaryRequirement::reconciliation(domain(domain_name))
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

#[allow(clippy::too_many_arguments)]
fn profile(
    qualification_profile: &str,
    closure_policy: &str,
    closure_profile: &str,
    class: ClosureClass,
    policy: QualifiedInputRef,
    root: &str,
    graph: ClosureDependencyGraph,
    requirements: BTreeMap<DerivationNodeRef, QualifiedBoundaryRequirement>,
    obligation_roles: BTreeMap<DerivationNodeRef, DomainRef>,
) -> ClosureQualificationProfile {
    ClosureQualificationProfile::new(
        org(),
        semantic(qualification_profile),
        ClosurePolicyRef::new(closure_policy).expect("static closure policy is valid"),
        semantic(closure_profile),
        class,
        policy,
        node(root),
        graph,
        requirements,
        obligation_roles,
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

    let requirements = BTreeMap::from([
        (
            agreement_node.clone(),
            input_requirement("commerce", "commerce.agreement-accepted"),
        ),
        (
            invoice_node.clone(),
            input_requirement("commerce", "commerce.invoice-projection-accepted"),
        ),
        (
            work_node.clone(),
            input_requirement("service-work", "service-work.obligation-disposition"),
        ),
        (
            payment_node.clone(),
            input_requirement("finance", "finance.obligation-disposition"),
        ),
        (
            reconciliation_node.clone(),
            reconciliation_requirement("finance"),
        ),
        (
            accounting_node.clone(),
            input_requirement("accounting", "accounting.accepted-economic-event"),
        ),
    ]);
    let obligation_roles = BTreeMap::from([
        (payment_node.clone(), domain("finance")),
        (work_node.clone(), domain("service-work")),
    ]);

    let boundaries = BTreeMap::from([
        (agreement_node, QualifiedBoundaryRef::Input(agreement)),
        (invoice_node, QualifiedBoundaryRef::Input(invoice)),
        (work_node.clone(), QualifiedBoundaryRef::Input(work_disposition)),
        (
            payment_node.clone(),
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
            graph,
            requirements,
            obligation_roles,
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            boundary_results: boundaries,
            obligations: BTreeMap::from([
                (payment_node, payment_obligation),
                (work_node, work_obligation),
            ]),
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

    let requirements = BTreeMap::from([
        (
            po_node.clone(),
            input_requirement("commerce", "commerce.purchase-order-accepted"),
        ),
        (
            receiving_node.clone(),
            input_requirement("supply-chain", "supply-chain.receiving-accepted"),
        ),
        (
            bill_match_node.clone(),
            input_requirement("commerce", "commerce.bill-match-accepted"),
        ),
        (
            settlement_node.clone(),
            input_requirement("finance", "finance.settlement-accepted"),
        ),
        (
            reconciliation_node.clone(),
            reconciliation_requirement("finance"),
        ),
        (
            payment_node.clone(),
            input_requirement("finance", "finance.obligation-disposition"),
        ),
        (
            accounting_node.clone(),
            input_requirement("accounting", "accounting.accepted-economic-event"),
        ),
    ]);
    let obligation_roles = BTreeMap::from([
        (payment_node.clone(), domain("finance")),
        (receiving_node.clone(), domain("supply-chain")),
    ]);

    let boundaries = BTreeMap::from([
        (po_node, QualifiedBoundaryRef::Input(purchase_order)),
        (receiving_node.clone(), QualifiedBoundaryRef::Input(receiving)),
        (bill_match_node, QualifiedBoundaryRef::Input(bill_match)),
        (settlement_node, QualifiedBoundaryRef::Input(settlement)),
        (
            reconciliation_node,
            QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
        ),
        (
            payment_node.clone(),
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
            graph,
            requirements,
            obligation_roles,
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            boundary_results: boundaries,
            obligations: BTreeMap::from([
                (payment_node, payment_obligation),
                (receiving_node, receiving_obligation),
            ]),
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

    let requirements = BTreeMap::from([
        (
            cancellation_node.clone(),
            input_requirement("finance", "finance.payment-cancellation-accepted"),
        ),
        (
            payment_node.clone(),
            input_requirement("finance", "finance.obligation-disposition"),
        ),
        (
            receiving_node.clone(),
            input_requirement("supply-chain", "supply-chain.obligation-disposition"),
        ),
        (
            accounting_node.clone(),
            input_requirement("accounting", "accounting.accepted-cancellation-event"),
        ),
    ]);
    let obligation_roles = BTreeMap::from([
        (payment_node.clone(), domain("finance")),
        (receiving_node.clone(), domain("supply-chain")),
    ]);

    let boundaries = BTreeMap::from([
        (cancellation_node, QualifiedBoundaryRef::Input(cancellation)),
        (
            payment_node.clone(),
            QualifiedBoundaryRef::Input(payment_disposition),
        ),
        (
            receiving_node.clone(),
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
            graph,
            requirements,
            obligation_roles,
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            boundary_results: boundaries,
            obligations: BTreeMap::from([
                (payment_node, payment_obligation),
                (receiving_node, receiving_obligation),
            ]),
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

    let requirements = BTreeMap::from([(
        accounting_node.clone(),
        input_requirement("accounting", "accounting.qualified-period-close"),
    )]);

    GoldenPathCase {
        workflow: WorkflowRef::new("workflow:month-end:2026-08").unwrap(),
        profile: profile(
            "business.month-end.qualification",
            "closure:month-end:satisfied",
            "business.month-end.close",
            ClosureClass::Satisfied,
            policy,
            "business:month-end-orchestration-close",
            graph,
            requirements,
            BTreeMap::new(),
        ),
        basis: ClosureQualificationBasis {
            qualification_cut: exact_cut,
            boundary_results: BTreeMap::from([(
                accounting_node,
                QualifiedBoundaryRef::Input(accounting_close),
            )]),
            obligations: BTreeMap::new(),
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
fn procurement_terminal_classes_keep_distinct_reusable_proof_schemas() {
    let satisfied_case = procurement_satisfied_case();
    let terminated_case = procurement_terminated_case();

    assert_ne!(
        satisfied_case.profile.dependency_graph(),
        terminated_case.profile.dependency_graph()
    );
    assert_ne!(
        satisfied_case.profile.boundary_requirements(),
        terminated_case.profile.boundary_requirements()
    );
    assert_eq!(satisfied_case.profile.required_obligation_roles().len(), 2);
    assert_eq!(terminated_case.profile.required_obligation_roles().len(), 2);

    let satisfied = satisfied_case.qualify().unwrap();
    let terminated = terminated_case.qualify().unwrap();
    assert_eq!(satisfied.class(), ClosureClass::Satisfied);
    assert_eq!(terminated.class(), ClosureClass::Terminated);
    assert_eq!(satisfied.qualification_cut().reconciliations().len(), 1);
    assert!(terminated.qualification_cut().reconciliations().is_empty());
}

#[test]
fn service_sale_profile_requires_internal_boundaries_and_both_obligation_roles() {
    let mut case = service_sale_case();
    assert!(case
        .profile
        .boundary_requirements()
        .contains_key(&node("commerce:accepted-invoice")));
    assert!(case
        .profile
        .boundary_requirements()
        .contains_key(&node("accounting:accepted-economic-event")));
    assert_eq!(case.profile.required_obligation_roles().len(), 2);

    case.basis
        .obligations
        .remove(&node("finance:payment-disposition"));
    assert!(matches!(
        case.profile.qualify(case.workflow, case.basis),
        Err(ClosureQualificationError::ObligationRoleSetMismatch { .. })
    ));
}

#[test]
fn month_end_preserves_proof_compression_at_the_business_boundary() {
    let case = month_end_case();
    assert_eq!(case.profile.boundary_requirements().len(), 1);
    assert!(case.profile.required_obligation_roles().is_empty());
    assert_eq!(case.basis.qualification_cut.inputs().len(), 2);

    let receipt = case.qualify().unwrap();
    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert!(receipt
        .qualification_cut()
        .inputs()
        .iter()
        .any(|exact| exact.record.as_str() == "period-close:2026-08"));
    assert!(receipt.qualification_cut().reconciliations().is_empty());
}
