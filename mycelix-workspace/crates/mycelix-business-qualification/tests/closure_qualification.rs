// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, DerivationNodeRef, DomainObligationRef,
    DomainReconciliationRef, DomainRef, DomainScopedRef, ObligationDisposition,
    ObligationDispositionBinding, ObligationRef, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId, TimestampMs, ValidityEnd,
    ValidityWindow, WorkflowRef,
};
use mycelix_business_qualification::{
    ClosureDependencyGraph, ClosureQualificationBasis, ClosureQualificationError,
    ClosureQualificationProfile, QualifiedBoundaryRef, QualifiedBoundaryRequirement,
};

fn semantic(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid semantic profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn node(name: &str) -> DerivationNodeRef {
    DerivationNodeRef::new(name).expect("valid derivation node")
}

fn input(
    domain_name: &str,
    record_name: &str,
    semantic_name: &str,
    semantic_version: u32,
) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: semantic(semantic_name, semantic_version),
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

#[derive(Clone)]
struct ServiceFixture {
    policy_source: QualifiedInputRef,
    agreement: QualifiedInputRef,
    invoice: QualifiedInputRef,
    work_disposition: QualifiedInputRef,
    payment_disposition: QualifiedInputRef,
    accounting: QualifiedInputRef,
    reconciliation: DomainReconciliationRef,
    payment_obligation: DomainObligationRef,
    work_obligation: DomainObligationRef,
}

impl ServiceFixture {
    fn new() -> Self {
        Self {
            policy_source: input(
                "governance",
                "closure-policy:service-sale:v1",
                "governance.closure-policy-active",
                1,
            ),
            agreement: input(
                "commerce",
                "service-agreement:1",
                "commerce.agreement",
                1,
            ),
            invoice: input(
                "commerce",
                "invoice:1",
                "commerce.invoice-projection",
                1,
            ),
            work_disposition: input(
                "service-work",
                "work-disposition:1",
                "service-work.obligation-disposition",
                1,
            ),
            payment_disposition: input(
                "finance",
                "payment-disposition:1",
                "finance.obligation-disposition",
                1,
            ),
            accounting: input(
                "accounting",
                "economic-event:service-sale-1",
                "accounting.accepted-economic-event",
                1,
            ),
            reconciliation: reconciliation("finance", "reconciliation:invoice-1"),
            payment_obligation: obligation("finance", "payment:invoice-1"),
            work_obligation: obligation("service-work", "work:agreement-1"),
        }
    }

    fn graph(&self) -> ClosureDependencyGraph {
        let root = node("business:service-sale-closure");
        let agreement = node("commerce:accepted-agreement");
        let invoice = node("commerce:accepted-invoice");
        let work = node("service-work:work-disposition");
        let payment = node("finance:payment-disposition");
        let accounting = node("accounting:accepted-economic-event");
        let reconciliation = node("finance:reconciliation");

        let mut graph = ClosureDependencyGraph::default();
        graph.add_dependency(root.clone(), invoice.clone());
        graph.add_dependency(invoice, agreement);
        graph.add_dependency(root.clone(), work);
        graph.add_dependency(root.clone(), payment);
        graph.add_dependency(root, accounting.clone());
        graph.add_dependency(accounting, reconciliation);
        graph
    }

    fn requirements(&self) -> BTreeMap<DerivationNodeRef, QualifiedBoundaryRequirement> {
        BTreeMap::from([
            (
                node("commerce:accepted-agreement"),
                QualifiedBoundaryRequirement::input(
                    domain("commerce"),
                    semantic("commerce.agreement", 1),
                ),
            ),
            (
                node("commerce:accepted-invoice"),
                QualifiedBoundaryRequirement::input(
                    domain("commerce"),
                    semantic("commerce.invoice-projection", 1),
                ),
            ),
            (
                node("service-work:work-disposition"),
                QualifiedBoundaryRequirement::input(
                    domain("service-work"),
                    semantic("service-work.obligation-disposition", 1),
                ),
            ),
            (
                node("finance:payment-disposition"),
                QualifiedBoundaryRequirement::input(
                    domain("finance"),
                    semantic("finance.obligation-disposition", 1),
                ),
            ),
            (
                node("accounting:accepted-economic-event"),
                QualifiedBoundaryRequirement::input(
                    domain("accounting"),
                    semantic("accounting.accepted-economic-event", 1),
                ),
            ),
            (
                node("finance:reconciliation"),
                QualifiedBoundaryRequirement::reconciliation(domain("finance")),
            ),
        ])
    }

    fn obligation_roles(&self) -> BTreeMap<DerivationNodeRef, DomainRef> {
        BTreeMap::from([
            (node("finance:payment-disposition"), domain("finance")),
            (
                node("service-work:work-disposition"),
                domain("service-work"),
            ),
        ])
    }

    fn profile(&self) -> ClosureQualificationProfile {
        ClosureQualificationProfile::new(
            OrganizationContextRef::new("org:acme").unwrap(),
            semantic("business.service-sale.qualification", 1),
            ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
            semantic("business.service-sale.close", 1),
            ClosureClass::Satisfied,
            self.policy_source.clone(),
            node("business:service-sale-closure"),
            self.graph(),
            self.requirements(),
            self.obligation_roles(),
        )
    }

    fn cut(&self) -> QualificationCut {
        let mut cut = QualificationCut::new(
            OrganizationContextRef::new("org:acme").unwrap(),
            semantic("business.service-sale.qualification", 1),
            TimestampMs::new(80),
        );

        for exact in [
            self.policy_source.clone(),
            self.agreement.clone(),
            self.invoice.clone(),
            self.work_disposition.clone(),
            self.payment_disposition.clone(),
            self.accounting.clone(),
        ] {
            assert!(cut.insert_input(exact).unwrap());
        }
        assert!(cut.insert_reconciliation(self.reconciliation.clone()));
        cut
    }

    fn basis(&self) -> ClosureQualificationBasis {
        let cut = self.cut();
        let dispositions = vec![
            ObligationDispositionBinding::bind(
                self.payment_obligation.clone(),
                ObligationDisposition::Satisfied,
                self.payment_disposition.clone(),
                &cut,
            )
            .unwrap(),
            ObligationDispositionBinding::bind(
                self.work_obligation.clone(),
                ObligationDisposition::Satisfied,
                self.work_disposition.clone(),
                &cut,
            )
            .unwrap(),
        ];

        ClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([
                (
                    node("commerce:accepted-agreement"),
                    QualifiedBoundaryRef::Input(self.agreement.clone()),
                ),
                (
                    node("commerce:accepted-invoice"),
                    QualifiedBoundaryRef::Input(self.invoice.clone()),
                ),
                (
                    node("service-work:work-disposition"),
                    QualifiedBoundaryRef::Input(self.work_disposition.clone()),
                ),
                (
                    node("finance:payment-disposition"),
                    QualifiedBoundaryRef::Input(self.payment_disposition.clone()),
                ),
                (
                    node("accounting:accepted-economic-event"),
                    QualifiedBoundaryRef::Input(self.accounting.clone()),
                ),
                (
                    node("finance:reconciliation"),
                    QualifiedBoundaryRef::Reconciliation(self.reconciliation.clone()),
                ),
            ]),
            obligations: BTreeMap::from([
                (
                    node("finance:payment-disposition"),
                    self.payment_obligation.clone(),
                ),
                (
                    node("service-work:work-disposition"),
                    self.work_obligation.clone(),
                ),
            ]),
            disposition_bindings: dispositions,
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::new(),
        }
    }
}

fn workflow() -> WorkflowRef {
    WorkflowRef::new("workflow:service-sale:1").unwrap()
}

#[test]
fn reusable_profile_qualifies_exact_service_sale_basis() {
    let fixture = ServiceFixture::new();
    let receipt = fixture.profile().qualify(workflow(), fixture.basis()).unwrap();
    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.obligations().len(), 2);
}

#[test]
fn profile_prevents_cross_organization_replay() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.qualification_cut.organization_context = OrganizationContextRef::new("org:other").unwrap();

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::WrongOrganization { .. })
    ));
}

#[test]
fn profile_prevents_qualification_profile_substitution() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.qualification_cut.semantic_profile = semantic("business.service-sale.qualification", 2);

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::WrongQualificationProfile { .. })
    ));
}

#[test]
fn policy_profile_source_is_required_in_exact_cut() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let mut replacement_cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        TimestampMs::new(80),
    );
    for exact in [
        fixture.agreement.clone(),
        fixture.invoice.clone(),
        fixture.work_disposition.clone(),
        fixture.payment_disposition.clone(),
        fixture.accounting.clone(),
    ] {
        assert!(replacement_cut.insert_input(exact).unwrap());
    }
    assert!(replacement_cut.insert_reconciliation(fixture.reconciliation.clone()));
    basis.qualification_cut = replacement_cut;

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::PolicySourceMissingFromCut { .. })
    ));
}

#[test]
fn profile_rejects_dependency_cycle() {
    let fixture = ServiceFixture::new();
    let mut graph = fixture.graph();
    graph.add_dependency(
        node("commerce:accepted-agreement"),
        node("business:service-sale-closure"),
    );
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        graph,
        fixture.requirements(),
        fixture.obligation_roles(),
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::Dependency(_))
    ));
}

#[test]
fn profile_rejects_missing_root() {
    let fixture = ServiceFixture::new();
    let mut graph = ClosureDependencyGraph::default();
    graph.add_node(node("commerce:accepted-agreement"));
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        graph,
        fixture.requirements(),
        fixture.obligation_roles(),
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::MissingRootNode { .. })
    ));
}

#[test]
fn profile_rejects_disconnected_decorative_nodes() {
    let fixture = ServiceFixture::new();
    let mut graph = fixture.graph();
    graph.add_node(node("unrelated:decorative-node"));
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        graph,
        fixture.requirements(),
        fixture.obligation_roles(),
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::DisconnectedDependencyGraph { .. })
    ));
}

#[test]
fn profile_cannot_leave_reachable_leaf_without_required_boundary_role() {
    let fixture = ServiceFixture::new();
    let mut graph = fixture.graph();
    graph.add_dependency(
        node("business:service-sale-closure"),
        node("commerce:required-but-ungrounded"),
    );
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        graph,
        fixture.requirements(),
        fixture.obligation_roles(),
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::ProfileUngroundedLeaves { .. })
    ));
}

#[test]
fn profile_boundary_role_must_be_reachable_from_closure_root() {
    let fixture = ServiceFixture::new();
    let mut requirements = fixture.requirements();
    requirements.insert(
        node("commerce:not-in-proof"),
        QualifiedBoundaryRequirement::input(
            domain("commerce"),
            semantic("commerce.agreement", 1),
        ),
    );
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        fixture.graph(),
        requirements,
        fixture.obligation_roles(),
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::ProfileUnreachableBoundaryNodes { .. })
    ));
}

#[test]
fn profile_obligation_role_must_also_be_boundary_role() {
    let fixture = ServiceFixture::new();
    let mut obligation_roles = fixture.obligation_roles();
    obligation_roles.insert(node("finance:not-a-boundary-role"), domain("finance"));
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        fixture.graph(),
        fixture.requirements(),
        obligation_roles,
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::ProfileObligationRoleMissingBoundary { .. })
    ));
}

#[test]
fn profile_obligation_domain_must_match_boundary_domain() {
    let fixture = ServiceFixture::new();
    let mut obligation_roles = fixture.obligation_roles();
    obligation_roles.insert(
        node("finance:payment-disposition"),
        domain("commerce"),
    );
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.service-sale.qualification", 1),
        ClosurePolicyRef::new("closure:service-sale:satisfied").unwrap(),
        semantic("business.service-sale.close", 1),
        ClosureClass::Satisfied,
        fixture.policy_source.clone(),
        node("business:service-sale-closure"),
        fixture.graph(),
        fixture.requirements(),
        obligation_roles,
    );

    assert!(matches!(
        profile.qualify(workflow(), fixture.basis()),
        Err(ClosureQualificationError::ProfileObligationRoleDomainMismatch { .. })
    ));
}

#[test]
fn basis_cannot_omit_internal_required_boundary_role() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis
        .boundary_results
        .remove(&node("commerce:accepted-invoice"));

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryRoleSetMismatch { .. })
    ));
}

#[test]
fn basis_cannot_inject_extra_boundary_role() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let extra = input("commerce", "extra:1", "commerce.extra", 1);
    assert!(basis.qualification_cut.insert_input(extra.clone()).unwrap());
    basis.boundary_results.insert(
        node("commerce:extra"),
        QualifiedBoundaryRef::Input(extra),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryRoleSetMismatch { .. })
    ));
}

#[test]
fn boundary_role_rejects_cross_domain_substitution() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let wrong = input(
        "commerce",
        "economic-event:not-accounting",
        "accounting.accepted-economic-event",
        1,
    );
    assert!(basis.qualification_cut.insert_input(wrong.clone()).unwrap());
    basis.boundary_results.insert(
        node("accounting:accepted-economic-event"),
        QualifiedBoundaryRef::Input(wrong),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryRequirementMismatch { .. })
    ));
}

#[test]
fn boundary_role_rejects_semantic_profile_substitution() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let wrong = input(
        "accounting",
        "economic-event:service-sale-1:v2-semantics",
        "accounting.accepted-economic-event",
        2,
    );
    assert!(basis.qualification_cut.insert_input(wrong.clone()).unwrap());
    basis.boundary_results.insert(
        node("accounting:accepted-economic-event"),
        QualifiedBoundaryRef::Input(wrong),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryRequirementMismatch { .. })
    ));
}

#[test]
fn exact_boundary_input_must_be_present_in_cut() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.boundary_results.insert(
        node("commerce:accepted-agreement"),
        QualifiedBoundaryRef::Input(input(
            "commerce",
            "service-agreement:other",
            "commerce.agreement",
            1,
        )),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryInputMissing { .. })
    ));
}

#[test]
fn exact_reconciliation_must_be_present_in_cut() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.boundary_results.insert(
        node("finance:reconciliation"),
        QualifiedBoundaryRef::Reconciliation(reconciliation(
            "finance",
            "reconciliation:other",
        )),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::BoundaryReconciliationMissing { .. })
    ));
}

#[test]
fn basis_cannot_omit_policy_required_obligation_role() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis
        .obligations
        .remove(&node("finance:payment-disposition"));

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::ObligationRoleSetMismatch { .. })
    ));
}

#[test]
fn basis_cannot_omit_policy_required_disposition_binding() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.disposition_bindings.retain(|binding| {
        binding.obligation() != &fixture.payment_obligation
    });

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::ObligationDispositionSetMismatch { .. })
    ));
}

#[test]
fn exact_obligation_domain_must_match_profile_role() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let wrong_obligation = obligation("supply-chain", "payment:not-finance");
    let wrong_source = input(
        "supply-chain",
        "payment-disposition:not-finance",
        "supply-chain.obligation-disposition",
        1,
    );
    assert!(basis
        .qualification_cut
        .insert_input(wrong_source.clone())
        .unwrap());
    basis.obligations.insert(
        node("finance:payment-disposition"),
        wrong_obligation.clone(),
    );
    basis.disposition_bindings.retain(|binding| {
        binding.obligation() != &fixture.payment_obligation
    });
    basis.disposition_bindings.push(
        ObligationDispositionBinding::bind(
            wrong_obligation,
            ObligationDisposition::Satisfied,
            wrong_source,
            &basis.qualification_cut,
        )
        .unwrap(),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::ObligationRoleDomainMismatch { .. })
    ));
}

#[test]
fn disposition_source_must_equal_exact_boundary_source_for_role() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    let alternate = input(
        "finance",
        "payment-disposition:alternate",
        "finance.obligation-disposition",
        1,
    );
    assert!(basis
        .qualification_cut
        .insert_input(alternate.clone())
        .unwrap());
    basis.disposition_bindings.retain(|binding| {
        binding.obligation() != &fixture.payment_obligation
    });
    basis.disposition_bindings.push(
        ObligationDispositionBinding::bind(
            fixture.payment_obligation.clone(),
            ObligationDisposition::Satisfied,
            alternate,
            &basis.qualification_cut,
        )
        .unwrap(),
    );

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::ObligationDispositionBoundaryMismatch { .. })
    ));
}

#[test]
fn satisfied_profile_cannot_strengthen_terminated_dispositions() {
    let fixture = ServiceFixture::new();
    let mut basis = fixture.basis();
    basis.disposition_bindings = vec![
        ObligationDispositionBinding::bind(
            fixture.payment_obligation.clone(),
            ObligationDisposition::Terminated,
            fixture.payment_disposition.clone(),
            &basis.qualification_cut,
        )
        .unwrap(),
        ObligationDispositionBinding::bind(
            fixture.work_obligation.clone(),
            ObligationDisposition::Terminated,
            fixture.work_disposition.clone(),
            &basis.qualification_cut,
        )
        .unwrap(),
    ];

    assert!(matches!(
        fixture.profile().qualify(workflow(), basis),
        Err(ClosureQualificationError::Closure(
            ClosureError::SatisfactionStrengthening { .. }
        ))
    ));
}

#[test]
fn one_accounting_boundary_can_compress_deeper_private_proof_graph() {
    let policy_source = input(
        "governance",
        "closure-policy:month-end:v1",
        "governance.closure-policy-active",
        1,
    );
    let accounting_close = input(
        "accounting",
        "period-close:2026-08",
        "accounting.qualified-period-close",
        1,
    );
    let root = node("business:month-end-orchestration-close");
    let accounting_node = node("accounting:qualified-period-close");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), accounting_node.clone());
    let requirements = BTreeMap::from([(
        accounting_node.clone(),
        QualifiedBoundaryRequirement::input(
            domain("accounting"),
            semantic("accounting.qualified-period-close", 1),
        ),
    )]);
    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.month-end.qualification", 1),
        ClosurePolicyRef::new("closure:month-end").unwrap(),
        semantic("business.month-end.close", 1),
        ClosureClass::Satisfied,
        policy_source.clone(),
        root,
        graph,
        requirements,
        BTreeMap::new(),
    );

    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.month-end.qualification", 1),
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(policy_source).unwrap());
    assert!(cut.insert_input(accounting_close.clone()).unwrap());

    let basis = ClosureQualificationBasis {
        qualification_cut: cut,
        boundary_results: BTreeMap::from([(
            accounting_node,
            QualifiedBoundaryRef::Input(accounting_close),
        )]),
        obligations: BTreeMap::new(),
        disposition_bindings: Vec::new(),
        exception_bindings: Vec::new(),
        compensating_intents: BTreeSet::new(),
    };

    let receipt = profile
        .qualify(WorkflowRef::new("workflow:month-end:2026-08").unwrap(), basis)
        .unwrap();
    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.qualification_cut().inputs().len(), 2);
}

#[test]
fn same_profile_and_exact_basis_are_deterministic() {
    let fixture = ServiceFixture::new();
    let profile = fixture.profile();
    let basis = fixture.basis();

    let first = profile.qualify(workflow(), basis.clone()).unwrap();
    let second = profile.qualify(workflow(), basis).unwrap();
    assert_eq!(first, second);
}
