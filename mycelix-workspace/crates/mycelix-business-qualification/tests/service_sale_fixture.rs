// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::causal::CommittedIntent;
use mycelix_business_core::{
    AllocationError, AllocationLine, AllocationRef, AllocationStatement, AttemptOutcome,
    AttemptOutcomeBinding, AttemptRef, AuthorizationBinding, AuthorizationDecisionRef,
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, DependencyGraph,
    DerivationNodeRef, DomainAuthorizationDecisionRef, DomainExceptionRef, DomainObligationRef,
    DomainReconciliationRef, DomainRef, DomainScopedRef, ExceptionBinding, ExceptionRef,
    GraphError, LogicalIntentRef, ObligationDisposition, ObligationDispositionBinding,
    ObligationRef, OperationCommitment, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, Quantity, ReconciliationRef, RecordRef, SemanticProfileId,
    SourceEventRef, SubjectRef, TimestampMs, UnitId, ValidityEnd, ValidityWindow,
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

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

fn payment_intent(amount_cents: u64) -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("pay:service-sale:invoice-1").expect("valid intent"),
        profile("finance.pay-service-invoice", 1),
        OperationCommitment::new(format!(
            "service-agreement:1|invoice:1|customer:alice|business:acme|USD-cent:{amount_cents}"
        ))
        .expect("valid semantic commitment"),
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum ServiceSaleError {
    OrganizationContextMismatch {
        expected: OrganizationContextRef,
        actual: OrganizationContextRef,
    },
    QualificationProfileMismatch {
        expected: SemanticProfileId,
        actual: SemanticProfileId,
    },
    MissingInput(QualifiedInputRef),
    MissingReconciliation(DomainReconciliationRef),
    Dependency(GraphError),
    Kernel(ClosureError),
}

impl From<GraphError> for ServiceSaleError {
    fn from(value: GraphError) -> Self {
        Self::Dependency(value)
    }
}

impl From<ClosureError> for ServiceSaleError {
    fn from(value: ClosureError) -> Self {
        Self::Kernel(value)
    }
}

/// GP-002 policy adapter for this executable fixture.
///
/// This is intentionally policy-local rather than a new generic kernel primitive.
/// It names the exact non-obligation prerequisites GP-002 requires before the
/// generic closure constructor may be invoked. The owning adapters remain
/// responsible for the substantive meaning of each exact result.
#[derive(Clone)]
struct ServiceSalePolicy {
    organization_context: OrganizationContextRef,
    qualification_profile: SemanticProfileId,
    agreement: QualifiedInputRef,
    invoice_document: QualifiedInputRef,
    work_accepted: QualifiedInputRef,
    finance_settlement: QualifiedInputRef,
    accounting_accepted: QualifiedInputRef,
    finance_reconciliation: DomainReconciliationRef,
    payment_obligation: DomainObligationRef,
    work_obligation: DomainObligationRef,
    dependencies: DependencyGraph,
}

impl ServiceSalePolicy {
    fn require_input(
        &self,
        cut: &QualificationCut,
        required: &QualifiedInputRef,
    ) -> Result<(), ServiceSaleError> {
        if cut.inputs().contains(required) {
            Ok(())
        } else {
            Err(ServiceSaleError::MissingInput(required.clone()))
        }
    }

    fn qualify(
        &self,
        cut: QualificationCut,
        dispositions: Vec<ObligationDispositionBinding>,
        class: ClosureClass,
        exceptions: Vec<ExceptionBinding>,
    ) -> Result<WorkflowClosureReceipt, ServiceSaleError> {
        if cut.organization_context != self.organization_context {
            return Err(ServiceSaleError::OrganizationContextMismatch {
                expected: self.organization_context.clone(),
                actual: cut.organization_context.clone(),
            });
        }

        if cut.semantic_profile != self.qualification_profile {
            return Err(ServiceSaleError::QualificationProfileMismatch {
                expected: self.qualification_profile.clone(),
                actual: cut.semantic_profile.clone(),
            });
        }

        self.dependencies.validate_acyclic()?;
        self.require_input(&cut, &self.agreement)?;
        self.require_input(&cut, &self.invoice_document)?;
        self.require_input(&cut, &self.work_accepted)?;
        self.require_input(&cut, &self.finance_settlement)?;
        self.require_input(&cut, &self.accounting_accepted)?;

        if !cut.reconciliations().contains(&self.finance_reconciliation) {
            return Err(ServiceSaleError::MissingReconciliation(
                self.finance_reconciliation.clone(),
            ));
        }

        WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:service-sale:1").expect("valid workflow"),
            ClosurePolicyRef::new("closure:service-sale:gp-002").expect("valid policy"),
            profile("business.service-sale.close", 1),
            cut,
            ClosureRequirements::new(BTreeSet::from([
                self.payment_obligation.clone(),
                self.work_obligation.clone(),
            ])),
            class,
            dispositions,
            exceptions,
            BTreeSet::new(),
        )
        .map_err(ServiceSaleError::from)
    }
}

struct Fixture {
    policy: ServiceSalePolicy,
    payment_disposition: QualifiedInputRef,
    work_disposition: QualifiedInputRef,
}

impl Fixture {
    fn new() -> Self {
        let organization_context =
            OrganizationContextRef::new("org:acme").expect("valid context");
        let qualification_profile = profile("business.service-sale.qualification", 1);
        let agreement = input(
            "commerce",
            "service-agreement:1",
            1,
            "commerce.service-agreement",
            100,
        );
        let invoice_document = input(
            "commerce",
            "invoice:service-sale-1",
            1,
            "commerce.invoice-projection",
            100,
        );
        let work_accepted = input(
            "service-work",
            "work-outcome:1",
            1,
            "service-work.accepted-outcome",
            100,
        );
        let finance_settlement = input(
            "finance",
            "settlement:bank-txn-1",
            1,
            "finance.reconciled-settlement",
            100,
        );
        let accounting_accepted = input(
            "accounting",
            "economic-event:service-sale-1",
            1,
            "accounting.accepted-economic-event",
            100,
        );

        Self {
            policy: ServiceSalePolicy {
                organization_context,
                qualification_profile,
                agreement,
                invoice_document,
                work_accepted,
                finance_settlement,
                accounting_accepted,
                finance_reconciliation: reconciliation("finance", "reconciliation:invoice-1"),
                payment_obligation: obligation("finance", "payment:invoice-1"),
                work_obligation: obligation("service-work", "work:agreement-1"),
                dependencies: dependency_graph(),
            },
            payment_disposition: input(
                "finance",
                "payment-disposition:invoice-1",
                1,
                "finance.obligation-disposition",
                100,
            ),
            work_disposition: input(
                "service-work",
                "work-disposition:agreement-1",
                1,
                "service-work.obligation-disposition",
                100,
            ),
        }
    }

    fn closure_cut(&self) -> QualificationCut {
        let mut cut = QualificationCut::new(
            self.policy.organization_context.clone(),
            self.policy.qualification_profile.clone(),
            TimestampMs::new(80),
        );

        for required in [
            self.policy.agreement.clone(),
            self.policy.invoice_document.clone(),
            self.policy.work_accepted.clone(),
            self.policy.finance_settlement.clone(),
            self.policy.accounting_accepted.clone(),
            self.payment_disposition.clone(),
            self.work_disposition.clone(),
        ] {
            assert!(cut.insert_input(required).expect("compatible exact input"));
        }

        assert!(cut.insert_reconciliation(self.policy.finance_reconciliation.clone()));
        cut
    }

    fn satisfied_dispositions(
        &self,
        cut: &QualificationCut,
    ) -> Vec<ObligationDispositionBinding> {
        vec![
            ObligationDispositionBinding::bind(
                self.policy.payment_obligation.clone(),
                ObligationDisposition::Satisfied,
                self.payment_disposition.clone(),
                cut,
            )
            .expect("finance payment disposition is exact"),
            ObligationDispositionBinding::bind(
                self.policy.work_obligation.clone(),
                ObligationDisposition::Satisfied,
                self.work_disposition.clone(),
                cut,
            )
            .expect("work disposition is exact"),
        ]
    }
}

fn dependency_graph() -> DependencyGraph {
    let mut graph = DependencyGraph::default();
    let closure = DerivationNodeRef::new("business:service-sale-closure").unwrap();
    let agreement = DerivationNodeRef::new("commerce:service-agreement").unwrap();
    let invoice = DerivationNodeRef::new("commerce:invoice-projection").unwrap();
    let work_disposition = DerivationNodeRef::new("service-work:work-disposition").unwrap();
    let work_accepted = DerivationNodeRef::new("service-work:accepted-outcome").unwrap();
    let payment_disposition = DerivationNodeRef::new("finance:payment-disposition").unwrap();
    let reconciliation = DerivationNodeRef::new("finance:reconciliation").unwrap();
    let bank_settlement = DerivationNodeRef::new("bank:settlement-event").unwrap();
    let accounting = DerivationNodeRef::new("accounting:accepted-economic-event").unwrap();

    graph.add_dependency(closure.clone(), agreement.clone());
    graph.add_dependency(closure.clone(), invoice.clone());
    graph.add_dependency(closure.clone(), work_disposition.clone());
    graph.add_dependency(closure.clone(), payment_disposition.clone());
    graph.add_dependency(closure, accounting.clone());
    graph.add_dependency(invoice, agreement);
    graph.add_dependency(work_disposition, work_accepted.clone());
    graph.add_dependency(payment_disposition, reconciliation.clone());
    graph.add_dependency(accounting.clone(), reconciliation.clone());
    graph.add_dependency(accounting, work_accepted);
    graph.add_dependency(reconciliation, bank_settlement);
    graph
}

#[test]
fn gp002_service_sale_can_close_only_from_exact_domain_results() {
    let fixture = Fixture::new();
    let cut = fixture.closure_cut();
    let dispositions = fixture.satisfied_dispositions(&cut);

    let receipt = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new())
        .expect("GP-002 exact service sale qualifies");

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert_eq!(receipt.obligations().len(), 2);
    assert!(receipt.exceptions().is_empty());
}

#[test]
fn business_closure_does_not_require_upstream_identity_or_provider_event_replay() {
    let fixture = Fixture::new();
    let cut = fixture.closure_cut();
    let identity_record = input(
        "identity",
        "party:customer-alice",
        1,
        "identity.party-reference",
        100,
    );

    assert!(!cut.inputs().contains(&identity_record));
    assert!(cut.source_events().is_empty());

    let dispositions = fixture.satisfied_dispositions(&cut);
    fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new())
        .expect("accepted domain results are sufficient closure-facing boundaries");
}

#[test]
fn cross_organization_cut_cannot_replay_service_sale_policy() {
    let fixture = Fixture::new();
    let mut cut = fixture.closure_cut();
    cut.organization_context = OrganizationContextRef::new("org:other").unwrap();
    let dispositions = fixture.satisfied_dispositions(&cut);

    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::OrganizationContextMismatch {
            expected: fixture.policy.organization_context.clone(),
            actual: OrganizationContextRef::new("org:other").unwrap(),
        })
    );
}

#[test]
fn qualification_profile_substitution_requires_requalification() {
    let fixture = Fixture::new();
    let mut cut = fixture.closure_cut();
    cut.semantic_profile = profile("business.service-sale.qualification", 2);
    let dispositions = fixture.satisfied_dispositions(&cut);

    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::QualificationProfileMismatch {
            expected: fixture.policy.qualification_profile.clone(),
            actual: profile("business.service-sale.qualification", 2),
        })
    );
}

#[test]
fn payment_authorization_is_separate_from_later_closure_truth() {
    let fixture = Fixture::new();
    let intent = payment_intent(50_000);
    let decision: DomainAuthorizationDecisionRef = DomainScopedRef::new(
        domain("finance"),
        AuthorizationDecisionRef::new("authorization:invoice-1").unwrap(),
    );
    let source = input(
        "finance",
        "authorization-result:invoice-1",
        1,
        "finance.authorization-decision",
        45,
    );

    let mut auth_cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.payment-authorization", 1),
        TimestampMs::new(30),
    );
    assert!(auth_cut.insert_input(fixture.policy.agreement.clone()).unwrap());
    assert!(auth_cut.insert_input(source.clone()).unwrap());
    assert!(auth_cut.insert_authorization_decision(decision.clone()));

    let authorization = AuthorizationBinding::bind(decision, intent.clone(), source, auth_cut)
        .expect("payment authorization qualifies at execution boundary");

    assert!(authorization.is_applicable_to_committed(&intent, TimestampMs::new(40)));
    assert!(!authorization.is_applicable_to_committed(&intent, TimestampMs::new(46)));
}

#[test]
fn customer_payment_before_work_acceptance_does_not_close_engagement() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for input in [
        fixture.policy.agreement.clone(),
        fixture.policy.invoice_document.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingInput(
            fixture.policy.work_accepted.clone()
        ))
    );
}

#[test]
fn work_accepted_without_finance_reconciliation_does_not_close_engagement() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for input in [
        fixture.policy.agreement.clone(),
        fixture.policy.invoice_document.clone(),
        fixture.policy.work_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingReconciliation(
            fixture.policy.finance_reconciliation.clone()
        ))
    );
}

#[test]
fn invoice_projection_cannot_substitute_for_finance_settlement() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for input in [
        fixture.policy.agreement.clone(),
        fixture.policy.invoice_document.clone(),
        fixture.policy.work_accepted.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingInput(
            fixture.policy.finance_settlement.clone()
        ))
    );
}

#[test]
fn non_satisfied_payment_disposition_cannot_project_as_satisfied() {
    let fixture = Fixture::new();
    let cut = fixture.closure_cut();
    let dispositions = vec![
        ObligationDispositionBinding::bind(
            fixture.policy.payment_obligation.clone(),
            ObligationDisposition::Breached,
            fixture.payment_disposition.clone(),
            &cut,
        )
        .unwrap(),
        ObligationDispositionBinding::bind(
            fixture.policy.work_obligation.clone(),
            ObligationDisposition::Satisfied,
            fixture.work_disposition.clone(),
            &cut,
        )
        .unwrap(),
    ];

    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert!(matches!(
        result,
        Err(ServiceSaleError::Kernel(
            ClosureError::SatisfactionStrengthening { .. }
        ))
    ));
}

#[test]
fn accounting_acceptance_is_a_real_closure_prerequisite() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );

    for input in [
        fixture.policy.agreement.clone(),
        fixture.policy.invoice_document.clone(),
        fixture.policy.work_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingInput(
            fixture.policy.accounting_accepted.clone()
        ))
    );
}

#[test]
fn corrected_agreement_version_requires_explicit_requalification() {
    let fixture = Fixture::new();

    let corrected = input(
        "commerce",
        "service-agreement:1",
        2,
        "commerce.service-agreement",
        100,
    );

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for input in [
        corrected,
        fixture.policy.invoice_document.clone(),
        fixture.policy.work_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingInput(fixture.policy.agreement.clone()))
    );
}

#[test]
fn corrected_invoice_version_requires_explicit_requalification() {
    let fixture = Fixture::new();
    let corrected = input(
        "commerce",
        "invoice:service-sale-1",
        2,
        "commerce.invoice-projection",
        100,
    );

    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for input in [
        fixture.policy.agreement.clone(),
        corrected,
        fixture.policy.work_accepted.clone(),
        fixture.policy.finance_settlement.clone(),
        fixture.policy.accounting_accepted.clone(),
        fixture.payment_disposition.clone(),
        fixture.work_disposition.clone(),
    ] {
        assert!(cut.insert_input(input).unwrap());
    }
    assert!(cut.insert_reconciliation(fixture.policy.finance_reconciliation.clone()));

    let dispositions = fixture.satisfied_dispositions(&cut);
    let result = fixture
        .policy
        .qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new());

    assert_eq!(
        result,
        Err(ServiceSaleError::MissingInput(
            fixture.policy.invoice_document.clone()
        ))
    );
}

#[test]
fn stale_approver_authority_cannot_be_used_at_later_payment_time() {
    let fixture = Fixture::new();
    let intent = payment_intent(50_000);
    let decision: DomainAuthorizationDecisionRef = DomainScopedRef::new(
        domain("finance"),
        AuthorizationDecisionRef::new("authorization:invoice-1").unwrap(),
    );
    let source = input(
        "finance",
        "authorization-result:invoice-1",
        1,
        "finance.authorization-decision",
        25,
    );
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.payment-authorization", 1),
        TimestampMs::new(20),
    );
    assert!(cut.insert_input(fixture.policy.agreement.clone()).unwrap());
    assert!(cut.insert_input(source.clone()).unwrap());
    assert!(cut.insert_authorization_decision(decision.clone()));

    let authorization = AuthorizationBinding::bind(decision, intent.clone(), source, cut)
        .expect("authority was valid at qualification time");

    assert!(!authorization.is_applicable_to_committed(&intent, TimestampMs::new(30)));
}

#[test]
fn unknown_payment_outcome_is_not_blind_retry_permission() {
    let intent = payment_intent(50_000);
    let source = input(
        "finance",
        "attempt-result:invoice-1",
        1,
        "finance.payment-attempt-result",
        100,
    );
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.payment-attempt", 1),
        TimestampMs::new(40),
    );
    assert!(cut.insert_input(source.clone()).unwrap());

    let outcome = AttemptOutcomeBinding::bind(
        AttemptRef::new("attempt:invoice-1:1").unwrap(),
        intent,
        AttemptOutcome::OutcomeUnknown,
        source,
        &cut,
    )
    .expect("exact unknown outcome is retained");

    assert!(!outcome.retry_permitted_without_additional_safety(&cut));
}

#[test]
fn duplicate_payment_delivery_does_not_double_value() {
    let event = SourceEventRef::new("bank", "settlement:bank-txn-1").unwrap();
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        profile("business.service-sale.qualification", 1),
        TimestampMs::new(80),
    );
    assert!(cut.insert_source_event(event.clone()));
    assert!(!cut.insert_source_event(event.clone()));

    let usd = UnitId::new("USD-cent").unwrap();
    let statement = AllocationStatement {
        allocation_ref: DomainScopedRef::new(
            domain("finance"),
            AllocationRef::new("allocation:bank-txn-1").unwrap(),
        ),
        source_event: event,
        semantic_profile: profile("finance.settlement-allocation", 1),
        total_available: Quantity::new(usd.clone(), 50_000),
        lines: vec![
            AllocationLine {
                target: SubjectRef::new("payment-obligation:invoice-1").unwrap(),
                quantity: Quantity::new(usd.clone(), 50_000),
            },
            AllocationLine {
                target: SubjectRef::new("payment-obligation:invoice-1").unwrap(),
                quantity: Quantity::new(usd, 50_000),
            },
        ],
    };

    assert!(matches!(
        statement.validate_conservation(),
        Err(AllocationError::OverAllocation {
            available: 50_000,
            allocated: 100_000,
        })
    ));
}

#[test]
fn unresolved_dispute_cannot_strengthen_to_satisfied_closure() {
    let fixture = Fixture::new();
    let mut cut = fixture.closure_cut();
    let dispute = exception("commerce", "exception:customer-disputes-work");
    let dispute_source = input(
        "commerce",
        "dispute:customer-work:1",
        1,
        "commerce.dispute-exception",
        100,
    );
    assert!(cut.insert_input(dispute_source.clone()).unwrap());
    assert!(cut.insert_exception(dispute.clone()));

    let binding = ExceptionBinding::bind(dispute, dispute_source, &cut)
        .expect("exact dispute provenance");
    let dispositions = fixture.satisfied_dispositions(&cut);

    let result = fixture.policy.qualify(
        cut,
        dispositions,
        ClosureClass::Satisfied,
        vec![binding],
    );

    assert_eq!(
        result,
        Err(ServiceSaleError::Kernel(ClosureError::SatisfiedWithExceptions))
    );
}

#[test]
fn accounting_cannot_depend_on_the_same_closure_it_justifies() {
    let fixture = Fixture::new();
    let cut = fixture.closure_cut();
    let dispositions = fixture.satisfied_dispositions(&cut);
    let mut policy = fixture.policy.clone();
    let accounting = DerivationNodeRef::new("accounting:accepted-economic-event").unwrap();
    let closure = DerivationNodeRef::new("business:service-sale-closure").unwrap();
    policy.dependencies.add_dependency(accounting, closure);

    assert!(matches!(
        policy.qualify(cut, dispositions, ClosureClass::Satisfied, Vec::new()),
        Err(ServiceSaleError::Dependency(GraphError::CycleDetected(_)))
    ));
}
