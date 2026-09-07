// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::causal::CommittedIntent;
use mycelix_business_core::{
    AllocationError, AllocationLine, AllocationRef, AllocationStatement, AttemptOutcome,
    AttemptOutcomeBinding, AttemptRef, AuthorizationBinding, AuthorizationDecisionRef,
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, DependencyGraph,
    DerivationNodeRef, DomainAuthorizationDecisionRef, DomainObligationRef,
    DomainReconciliationRef, DomainRef, DomainScopedRef, GraphError, LogicalIntentRef,
    ObligationDisposition, ObligationDispositionBinding, ObligationRef, OperationCommitment,
    OrganizationContextRef, QualificationCut, QualifiedInputRef, Quantity, ReconciliationRef,
    RecordRef, SemanticProfileId, SubjectRef, TimestampMs, UnitId, ValidityEnd, ValidityWindow,
    WorkflowClosureReceipt, WorkflowRef,
};

fn profile(name: &str, version: u32) -> SemanticProfileId {
    SemanticProfileId::new(name, version).expect("valid profile")
}

fn domain(name: &str) -> DomainRef {
    DomainRef::new(name).expect("valid domain")
}

fn input(
    domain_name: &str,
    record_name: &str,
    version: u64,
    profile_name: &str,
    valid_until: i64,
) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version,
        semantic_profile: profile(profile_name, 1),
        generation: None,
        validity: ValidityWindow::new(
            TimestampMs::new(0),
            ValidityEnd::At(TimestampMs::new(valid_until)),
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

fn payment_intent(amount_cents: u64) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("pay:procurement:po-1").expect("valid intent"),
        profile("finance.pay-procurement", 1),
        OperationCommitment::new(format!(
            "purchase-order:1|supplier:vendor-a|USD-cent:{amount_cents}|destination:vendor-a-bank"
        ))
        .expect("valid operation commitment"),
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum ProcurementError {
    WrongOrganization {
        expected: OrganizationContextRef,
        actual: OrganizationContextRef,
    },
    WrongQualificationProfile {
        expected: SemanticProfileId,
        actual: SemanticProfileId,
    },
    UnsupportedClosureClass(ClosureClass),
    MissingInput(QualifiedInputRef),
    MissingReconciliation(DomainReconciliationRef),
    Dependency(GraphError),
    Kernel(ClosureError),
}

impl From<GraphError> for ProcurementError {
    fn from(value: GraphError) -> Self {
        Self::Dependency(value)
    }
}

impl From<ClosureError> for ProcurementError {
    fn from(value: ClosureError) -> Self {
        Self::Kernel(value)
    }
}

/// GP-003 policy-local closure adapter.
///
/// The policy consumes accepted domain results rather than recursively replaying
/// supplier identity, raw carrier observations, bank callbacks, or the historical
/// Governance approval used to authorize payment. Those remain behind their
/// authoritative domain boundaries.
///
/// Closure dependency sets are class-specific: a `Satisfied` procurement needs
/// exact settlement + reconciliation, whereas a `Terminated` procurement needs
/// an exact payment-cancellation result instead of pretending settlement occurred.
#[derive(Clone)]
struct ProcurementPolicy {
    organization_context: OrganizationContextRef,
    qualification_profile: SemanticProfileId,
    purchase_order: QualifiedInputRef,
    supplier_bill: QualifiedInputRef,
    bill_match_accepted: QualifiedInputRef,
    receipt_accepted: QualifiedInputRef,
    inventory_effect_accepted: QualifiedInputRef,
    finance_settlement: QualifiedInputRef,
    payment_cancelled: QualifiedInputRef,
    accounting_accepted: QualifiedInputRef,
    finance_reconciliation: DomainReconciliationRef,
    supplier_fulfillment_obligation: DomainObligationRef,
    payment_obligation: DomainObligationRef,
}

impl ProcurementPolicy {
    fn require_input(
        &self,
        cut: &QualificationCut,
        required: &QualifiedInputRef,
    ) -> Result<(), ProcurementError> {
        if cut.inputs().contains(required) {
            Ok(())
        } else {
            Err(ProcurementError::MissingInput(required.clone()))
        }
    }

    fn dependency_graph(&self, class: ClosureClass) -> Result<DependencyGraph, ProcurementError> {
        let mut graph = DependencyGraph::default();
        let closure = DerivationNodeRef::new("business:procurement-closure").unwrap();
        let purchase_order = DerivationNodeRef::new("commerce:purchase-order").unwrap();
        let supplier_bill = DerivationNodeRef::new("commerce:supplier-bill").unwrap();
        let bill_match = DerivationNodeRef::new("procurement:bill-match-accepted").unwrap();
        let receipt = DerivationNodeRef::new("supply-chain:receipt-accepted").unwrap();
        let inventory = DerivationNodeRef::new("supply-chain:inventory-effect-accepted").unwrap();
        let payment_disposition = DerivationNodeRef::new("finance:payment-disposition").unwrap();
        let fulfillment_disposition =
            DerivationNodeRef::new("supply-chain:fulfillment-disposition").unwrap();
        let accounting = DerivationNodeRef::new("accounting:accepted-economic-event").unwrap();

        graph.add_dependency(closure.clone(), purchase_order.clone());
        graph.add_dependency(closure.clone(), bill_match.clone());
        graph.add_dependency(closure.clone(), inventory.clone());
        graph.add_dependency(closure.clone(), payment_disposition.clone());
        graph.add_dependency(closure.clone(), fulfillment_disposition.clone());
        graph.add_dependency(closure, accounting.clone());

        graph.add_dependency(bill_match.clone(), purchase_order.clone());
        graph.add_dependency(bill_match.clone(), supplier_bill);
        graph.add_dependency(bill_match.clone(), receipt.clone());
        graph.add_dependency(inventory, receipt.clone());
        graph.add_dependency(fulfillment_disposition, receipt);
        graph.add_dependency(accounting.clone(), purchase_order);
        graph.add_dependency(accounting.clone(), bill_match);

        match class {
            ClosureClass::Satisfied => {
                let reconciliation = DerivationNodeRef::new("finance:reconciliation").unwrap();
                let settlement = DerivationNodeRef::new("finance:accepted-settlement").unwrap();
                graph.add_dependency(payment_disposition, reconciliation.clone());
                graph.add_dependency(reconciliation.clone(), settlement);
                graph.add_dependency(accounting, reconciliation);
            }
            ClosureClass::Terminated => {
                let cancellation =
                    DerivationNodeRef::new("finance:payment-cancellation-accepted").unwrap();
                graph.add_dependency(payment_disposition, cancellation.clone());
                graph.add_dependency(accounting, cancellation);
            }
            unsupported => return Err(ProcurementError::UnsupportedClosureClass(unsupported)),
        }

        Ok(graph)
    }

    fn qualify_with_graph(
        &self,
        graph: &DependencyGraph,
        cut: QualificationCut,
        dispositions: Vec<ObligationDispositionBinding>,
        class: ClosureClass,
    ) -> Result<WorkflowClosureReceipt, ProcurementError> {
        if cut.organization_context != self.organization_context {
            return Err(ProcurementError::WrongOrganization {
                expected: self.organization_context.clone(),
                actual: cut.organization_context.clone(),
            });
        }
        if cut.semantic_profile != self.qualification_profile {
            return Err(ProcurementError::WrongQualificationProfile {
                expected: self.qualification_profile.clone(),
                actual: cut.semantic_profile.clone(),
            });
        }

        graph.validate_acyclic()?;

        for required in [
            &self.purchase_order,
            &self.supplier_bill,
            &self.bill_match_accepted,
            &self.receipt_accepted,
            &self.inventory_effect_accepted,
            &self.accounting_accepted,
        ] {
            self.require_input(&cut, required)?;
        }

        match class {
            ClosureClass::Satisfied => {
                self.require_input(&cut, &self.finance_settlement)?;
                if !cut.reconciliations().contains(&self.finance_reconciliation) {
                    return Err(ProcurementError::MissingReconciliation(
                        self.finance_reconciliation.clone(),
                    ));
                }
            }
            ClosureClass::Terminated => {
                self.require_input(&cut, &self.payment_cancelled)?;
            }
            unsupported => return Err(ProcurementError::UnsupportedClosureClass(unsupported)),
        }

        WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:procurement:po-1").expect("valid workflow"),
            ClosurePolicyRef::new("closure:procurement:gp-003").expect("valid policy"),
            profile("business.procurement.close", 1),
            cut,
            ClosureRequirements::new(BTreeSet::from([
                self.supplier_fulfillment_obligation.clone(),
                self.payment_obligation.clone(),
            ])),
            class,
            dispositions,
            Vec::new(),
            BTreeSet::new(),
        )
        .map_err(Into::into)
    }

    fn qualify(
        &self,
        cut: QualificationCut,
        dispositions: Vec<ObligationDispositionBinding>,
        class: ClosureClass,
    ) -> Result<WorkflowClosureReceipt, ProcurementError> {
        let graph = self.dependency_graph(class)?;
        self.qualify_with_graph(&graph, cut, dispositions, class)
    }
}

struct Fixture {
    policy: ProcurementPolicy,
    payment_disposition: QualifiedInputRef,
    fulfillment_disposition: QualifiedInputRef,
}

impl Fixture {
    fn new() -> Self {
        Self {
            policy: ProcurementPolicy {
                organization_context: OrganizationContextRef::new("org:acme")
                    .expect("valid context"),
                qualification_profile: profile("business.procurement.qualification", 1),
                purchase_order: input(
                    "commerce",
                    "purchase-order:1",
                    1,
                    "commerce.purchase-order-accepted",
                    100,
                ),
                supplier_bill: input(
                    "commerce",
                    "supplier-bill:1",
                    1,
                    "commerce.supplier-bill-accepted",
                    100,
                ),
                bill_match_accepted: input(
                    "procurement",
                    "bill-match:po-1:bill-1",
                    1,
                    "procurement.bill-match-accepted",
                    100,
                ),
                receipt_accepted: input(
                    "supply-chain",
                    "receipt:po-1",
                    1,
                    "supply-chain.receipt-accepted",
                    100,
                ),
                inventory_effect_accepted: input(
                    "supply-chain",
                    "inventory-effect:po-1",
                    1,
                    "supply-chain.inventory-effect-accepted",
                    100,
                ),
                finance_settlement: input(
                    "finance",
                    "settlement:po-1",
                    1,
                    "finance.reconciled-settlement",
                    100,
                ),
                payment_cancelled: input(
                    "finance",
                    "payment-cancellation:po-1",
                    1,
                    "finance.payment-cancellation-accepted",
                    100,
                ),
                accounting_accepted: input(
                    "accounting",
                    "economic-event:procurement-po-1",
                    1,
                    "accounting.accepted-economic-event",
                    100,
                ),
                finance_reconciliation: reconciliation("finance", "reconciliation:po-1"),
                supplier_fulfillment_obligation: obligation(
                    "supply-chain",
                    "supplier-fulfillment:po-1",
                ),
                payment_obligation: obligation("finance", "supplier-payment:po-1"),
            },
            payment_disposition: input(
                "finance",
                "payment-disposition:po-1",
                1,
                "finance.obligation-disposition",
                100,
            ),
            fulfillment_disposition: input(
                "supply-chain",
                "fulfillment-disposition:po-1",
                1,
                "supply-chain.obligation-disposition",
                100,
            ),
        }
    }

    fn common_cut(&self) -> QualificationCut {
        let mut cut = QualificationCut::new(
            self.policy.organization_context.clone(),
            self.policy.qualification_profile.clone(),
            TimestampMs::new(80),
        );

        for required in [
            self.policy.purchase_order.clone(),
            self.policy.supplier_bill.clone(),
            self.policy.bill_match_accepted.clone(),
            self.policy.receipt_accepted.clone(),
            self.policy.inventory_effect_accepted.clone(),
            self.policy.accounting_accepted.clone(),
            self.payment_disposition.clone(),
            self.fulfillment_disposition.clone(),
        ] {
            assert!(cut.insert_input(required).expect("compatible exact input"));
        }
        cut
    }

    fn satisfied_cut(&self) -> QualificationCut {
        let mut cut = self.common_cut();
        assert!(cut.insert_input(self.policy.finance_settlement.clone()).unwrap());
        assert!(cut.insert_reconciliation(self.policy.finance_reconciliation.clone()));
        cut
    }

    fn terminated_cut(&self) -> QualificationCut {
        let mut cut = self.common_cut();
        assert!(cut.insert_input(self.policy.payment_cancelled.clone()).unwrap());
        cut
    }

    fn dispositions(
        &self,
        cut: &QualificationCut,
        payment: ObligationDisposition,
        fulfillment: ObligationDisposition,
    ) -> Vec<ObligationDispositionBinding> {
        vec![
            ObligationDispositionBinding::bind(
                self.policy.payment_obligation.clone(),
                payment,
                self.payment_disposition.clone(),
                cut,
            )
            .expect("exact Finance payment disposition"),
            ObligationDispositionBinding::bind(
                self.policy.supplier_fulfillment_obligation.clone(),
                fulfillment,
                self.fulfillment_disposition.clone(),
                cut,
            )
            .expect("exact Supply Chain fulfillment disposition"),
        ]
    }

    fn satisfied_dispositions(&self, cut: &QualificationCut) -> Vec<ObligationDispositionBinding> {
        self.dispositions(
            cut,
            ObligationDisposition::Satisfied,
            ObligationDisposition::Satisfied,
        )
    }
}

#[test]
fn gp003_procurement_can_close_from_exact_accepted_domain_results() {
    let fixture = Fixture::new();
    let cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&cut);

    let receipt = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied)
        .expect("GP-003 exact procurement path qualifies");

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.obligations().len(), 2);
    assert!(receipt.exceptions().is_empty());
}

#[test]
fn procurement_closure_does_not_require_raw_supplier_identity_or_provider_events() {
    let fixture = Fixture::new();
    let cut = fixture.satisfied_cut();

    assert!(cut.inputs().iter().all(|item| item.domain != domain("identity")));
    assert!(cut.source_events().is_empty());

    let dispositions = fixture.satisfied_dispositions(&cut);
    fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied)
        .expect("accepted domain results compress upstream proof for Business closure");
}

#[test]
fn bill_without_exact_purchase_order_cannot_close() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for item in [
        fixture.policy.supplier_bill.clone(),
        fixture.policy.bill_match_accepted.clone(),
        fixture.policy.receipt_accepted.clone(),
        fixture.policy.inventory_effect_accepted.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.payment_disposition.clone(),
        fixture.fulfillment_disposition.clone(),
    ] {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingInput(
            fixture.policy.purchase_order.clone()
        ))
    );
}

#[test]
fn quantity_mismatch_without_accepted_bill_match_cannot_close() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let rejected_match = input(
        "procurement",
        "bill-match:po-1:bill-1:mismatch",
        1,
        "procurement.bill-match-rejected",
        100,
    );
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for item in [
        fixture.policy.purchase_order.clone(),
        fixture.policy.supplier_bill.clone(),
        rejected_match,
        fixture.policy.receipt_accepted.clone(),
        fixture.policy.inventory_effect_accepted.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.payment_disposition.clone(),
        fixture.fulfillment_disposition.clone(),
    ] {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingInput(
            fixture.policy.bill_match_accepted.clone()
        ))
    );
}

#[test]
fn duplicate_supplier_bill_does_not_multiply_exact_inputs() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );

    assert!(cut.insert_input(fixture.policy.supplier_bill.clone()).unwrap());
    assert!(!cut.insert_input(fixture.policy.supplier_bill.clone()).unwrap());
    assert_eq!(cut.inputs().len(), 1);
}

#[test]
fn governance_approval_expiry_attenuates_payment_authorization() {
    let fixture = Fixture::new();
    let intent = payment_intent(50_000);
    let decision: DomainAuthorizationDecisionRef = DomainScopedRef::new(
        domain("finance"),
        AuthorizationDecisionRef::new("authorization:po-1").unwrap(),
    );
    let decision_source = input(
        "finance",
        "authorization-result:po-1",
        1,
        "finance.authorization-decision",
        60,
    );
    let governance_approval = input(
        "governance",
        "procurement-approval:po-1",
        1,
        "governance.procurement-approval",
        42,
    );

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.procurement-payment-authorization", 1),
        TimestampMs::new(30),
    );
    for item in [
        fixture.policy.purchase_order.clone(),
        governance_approval,
        decision_source.clone(),
    ] {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_authorization_decision(decision.clone()));

    let binding = AuthorizationBinding::bind(decision, intent.clone(), decision_source, cut)
        .expect("Finance authorization qualifies while Governance approval is current");

    assert_eq!(
        binding.effective_valid_until(),
        ValidityEnd::At(TimestampMs::new(42))
    );
    assert!(binding.is_applicable_to_committed(&intent, TimestampMs::new(42)));
    assert!(!binding.is_applicable_to_committed(&intent, TimestampMs::new(43)));
}

#[test]
fn historical_governance_approval_is_not_required_in_later_closure_cut() {
    let fixture = Fixture::new();
    let cut = fixture.satisfied_cut();

    assert!(cut.inputs().iter().all(|item| item.domain != domain("governance")));

    let dispositions = fixture.satisfied_dispositions(&cut);
    fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied)
        .expect("closure uses accepted post-execution facts, not stale approval as settlement truth");
}

#[test]
fn cancelled_payment_path_cannot_borrow_satisfied_dependencies() {
    let fixture = Fixture::new();
    let cut = fixture.terminated_cut();
    let dispositions = fixture.dispositions(
        &cut,
        ObligationDisposition::Terminated,
        ObligationDisposition::Satisfied,
    );

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingInput(
            fixture.policy.finance_settlement.clone()
        ))
    );
}

#[test]
fn payment_cancellation_can_remain_explicit_as_terminated_closure() {
    let fixture = Fixture::new();
    let cut = fixture.terminated_cut();
    let dispositions = fixture.dispositions(
        &cut,
        ObligationDisposition::Terminated,
        ObligationDisposition::Satisfied,
    );

    let receipt = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Terminated)
        .expect("exact cancellation result supports Terminated, not fictitious settlement");

    assert_eq!(receipt.class(), ClosureClass::Terminated);
}

#[test]
fn unsupported_closure_class_does_not_inherit_an_unrelated_dependency_set() {
    let fixture = Fixture::new();
    let cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&cut);

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Waived),
        Err(ProcurementError::UnsupportedClosureClass(
            ClosureClass::Waived
        ))
    );
}

#[test]
fn missing_finance_reconciliation_cannot_close_even_with_settlement_record() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let mut cut = fixture.common_cut();
    assert!(cut.insert_input(fixture.policy.finance_settlement.clone()).unwrap());

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingReconciliation(
            fixture.policy.finance_reconciliation.clone()
        ))
    );
}

#[test]
fn accounting_acceptance_is_independent_required_closure_fact() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for item in [
        fixture.policy.purchase_order.clone(),
        fixture.policy.supplier_bill.clone(),
        fixture.policy.bill_match_accepted.clone(),
        fixture.policy.receipt_accepted.clone(),
        fixture.policy.inventory_effect_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.payment_disposition.clone(),
        fixture.fulfillment_disposition.clone(),
    ] {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingInput(
            fixture.policy.accounting_accepted.clone()
        ))
    );
}

#[test]
fn corrected_supplier_purchase_order_requires_requalification() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let corrected_po = QualifiedInputRef {
        version: 2,
        ..fixture.policy.purchase_order.clone()
    };
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for item in [
        corrected_po,
        fixture.policy.supplier_bill.clone(),
        fixture.policy.bill_match_accepted.clone(),
        fixture.policy.receipt_accepted.clone(),
        fixture.policy.inventory_effect_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.fulfillment_disposition.clone(),
    ] {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert_eq!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::MissingInput(
            fixture.policy.purchase_order.clone()
        ))
    );
}

#[test]
fn payment_outcome_unknown_is_not_blind_retry_permission() {
    let fixture = Fixture::new();
    let intent = payment_intent(50_000);
    let attempt = AttemptRef::new("attempt:pay-po-1:1").unwrap();
    let outcome_source = input(
        "finance",
        "payment-attempt-outcome:po-1:1",
        1,
        "finance.payment-attempt-outcome",
        100,
    );
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.procurement-payment-outcome", 1),
        TimestampMs::new(50),
    );
    assert!(cut.insert_input(outcome_source.clone()).unwrap());

    let binding = AttemptOutcomeBinding::bind(
        attempt,
        intent,
        AttemptOutcome::OutcomeUnknown,
        outcome_source,
        &cut,
    )
    .expect("exact unknown outcome binding");

    assert!(!binding.retry_permitted_without_additional_safety(&cut));
}

#[test]
fn duplicate_payment_allocation_cannot_double_spend_one_settlement_capacity() {
    let usd = UnitId::new("USD-cent").unwrap();
    let statement = AllocationStatement {
        allocation_ref: DomainScopedRef::new(
            domain("finance"),
            AllocationRef::new("allocation:po-1").unwrap(),
        ),
        source_event: mycelix_business_core::SourceEventRef::new("bank", "settlement:po-1")
            .unwrap(),
        semantic_profile: profile("finance.payment-allocation", 1),
        total_available: Quantity::new(usd.clone(), 50_000),
        lines: vec![
            AllocationLine {
                target: SubjectRef::new("supplier-payment:po-1:first").unwrap(),
                quantity: Quantity::new(usd.clone(), 50_000),
            },
            AllocationLine {
                target: SubjectRef::new("supplier-payment:po-1:duplicate").unwrap(),
                quantity: Quantity::new(usd, 50_000),
            },
        ],
    };

    assert!(matches!(
        statement.validate_conservation(),
        Err(AllocationError::OverAllocation {
            available: 50_000,
            allocated: 100_000
        })
    ));
}

#[test]
fn cross_organization_procurement_cut_cannot_be_replayed() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:other").unwrap(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for item in full_cut.inputs().iter().cloned() {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert!(matches!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::WrongOrganization { .. })
    ));
}

#[test]
fn procurement_qualification_profile_substitution_is_denied() {
    let fixture = Fixture::new();
    let full_cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&full_cut);

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.procurement.qualification", 2),
        TimestampMs::new(80),
    );
    for item in full_cut.inputs().iter().cloned() {
        assert!(cut.insert_input(item).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    assert!(matches!(
        fixture
            .policy
            .qualify(cut, dispositions, ClosureClass::Satisfied),
        Err(ProcurementError::WrongQualificationProfile { .. })
    ));
}

#[test]
fn accounting_cannot_depend_on_same_business_procurement_closure() {
    let fixture = Fixture::new();
    let cut = fixture.satisfied_cut();
    let dispositions = fixture.satisfied_dispositions(&cut);
    let mut graph = fixture
        .policy
        .dependency_graph(ClosureClass::Satisfied)
        .expect("Satisfied dependency profile is supported");

    graph.add_dependency(
        DerivationNodeRef::new("accounting:accepted-economic-event").unwrap(),
        DerivationNodeRef::new("business:procurement-closure").unwrap(),
    );

    assert!(matches!(
        fixture.policy.qualify_with_graph(
            &graph,
            cut,
            dispositions,
            ClosureClass::Satisfied
        ),
        Err(ProcurementError::Dependency(GraphError::CycleDetected(_)))
    ));
}
