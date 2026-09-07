// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! GP-005 refund/dispute diagnostic.
//!
//! This fixture intentionally keeps compensation/exception role semantics in a
//! policy-local wrapper. Its purpose is to determine whether those requirements
//! should later become reusable `ClosureQualificationProfile` schema roles, just
//! as boundary topology and required obligations already have.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{
    AttemptOutcome, AttemptOutcomeBinding, AttemptRef, ClosureClass, ClosurePolicyRef,
    CommittedIntent, CompensationBinding, DerivationNodeRef, DomainExceptionRef,
    DomainObligationRef, DomainReconciliationRef, DomainRef, DomainScopedRef, ExceptionBinding,
    ExceptionRef, LogicalIntentRef, ObligationDisposition, ObligationDispositionBinding,
    ObligationRef, OperationCommitment, OrganizationContextRef, QualificationCut,
    QualifiedInputRef, ReconciliationRef, RecordRef, SemanticProfileId, SourceEventRef,
    TimestampMs, ValidityEnd, ValidityWindow, WorkflowClosureReceipt, WorkflowRef,
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
    valid_until: i64,
) -> QualifiedInputRef {
    QualifiedInputRef {
        domain: domain(domain_name),
        record: RecordRef::new(record_name).expect("valid record"),
        version: 1,
        semantic_profile: semantic(semantic_name, 1),
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

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

fn reconciliation(domain_name: &str, local_id: &str) -> DomainReconciliationRef {
    DomainScopedRef::new(
        domain(domain_name),
        ReconciliationRef::new(local_id).expect("valid reconciliation"),
    )
}

fn refund_intent() -> CommittedIntent {
    CommittedIntent::new(
        LogicalIntentRef::new("refund:order-1:5000").unwrap(),
        semantic("finance.refund", 1),
        OperationCommitment::new("USD-cent:5000:customer-a:original-order-1").unwrap(),
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum TerminalRequirement {
    Compensation {
        role: DerivationNodeRef,
        intent: CommittedIntent,
    },
    Exception {
        role: DerivationNodeRef,
        exception: DomainExceptionRef,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum RefundPolicyError {
    Structural(ClosureQualificationError),
    RequiredRoleIsNotExactInput { role: DerivationNodeRef },
    CompensationSetMismatch,
    CompensationIntentMismatch,
    CompensationSourceMismatch,
    UnexpectedCompensation,
    ExceptionSetMismatch,
    ExceptionIdentityMismatch,
    ExceptionSourceMismatch,
    UnexpectedException,
}

impl From<ClosureQualificationError> for RefundPolicyError {
    fn from(value: ClosureQualificationError) -> Self {
        Self::Structural(value)
    }
}

#[derive(Clone)]
struct RefundDisputePolicy {
    profile: ClosureQualificationProfile,
    terminal_requirement: TerminalRequirement,
}

impl RefundDisputePolicy {
    fn qualify(
        &self,
        workflow: WorkflowRef,
        basis: ClosureQualificationBasis,
    ) -> Result<WorkflowClosureReceipt, RefundPolicyError> {
        match &self.terminal_requirement {
            TerminalRequirement::Compensation { role, intent } => {
                if !basis.exception_bindings.is_empty() {
                    return Err(RefundPolicyError::UnexpectedException);
                }
                if basis.compensating_intents.len() != 1 {
                    return Err(RefundPolicyError::CompensationSetMismatch);
                }

                let Some(QualifiedBoundaryRef::Input(expected_source)) =
                    basis.boundary_results.get(role)
                else {
                    return Err(RefundPolicyError::RequiredRoleIsNotExactInput {
                        role: role.clone(),
                    });
                };
                let binding = basis
                    .compensating_intents
                    .iter()
                    .next()
                    .expect("exact compensation cardinality checked above");
                if binding.intent() != intent {
                    return Err(RefundPolicyError::CompensationIntentMismatch);
                }
                if binding.source() != expected_source {
                    return Err(RefundPolicyError::CompensationSourceMismatch);
                }
            }
            TerminalRequirement::Exception { role, exception } => {
                if !basis.compensating_intents.is_empty() {
                    return Err(RefundPolicyError::UnexpectedCompensation);
                }
                if basis.exception_bindings.len() != 1 {
                    return Err(RefundPolicyError::ExceptionSetMismatch);
                }

                let Some(QualifiedBoundaryRef::Input(expected_source)) =
                    basis.boundary_results.get(role)
                else {
                    return Err(RefundPolicyError::RequiredRoleIsNotExactInput {
                        role: role.clone(),
                    });
                };
                let binding = &basis.exception_bindings[0];
                if binding.exception() != exception {
                    return Err(RefundPolicyError::ExceptionIdentityMismatch);
                }
                if binding.source() != expected_source {
                    return Err(RefundPolicyError::ExceptionSourceMismatch);
                }
            }
        }

        self.profile.qualify(workflow, basis).map_err(Into::into)
    }
}

#[derive(Clone)]
struct RefundCase {
    policy: RefundDisputePolicy,
    workflow: WorkflowRef,
    basis: ClosureQualificationBasis,
    original_transaction: QualifiedInputRef,
    resolution: QualifiedInputRef,
    refund_effect: Option<QualifiedInputRef>,
    required_exception: Option<DomainExceptionRef>,
}

impl RefundCase {
    fn qualify(self) -> Result<WorkflowClosureReceipt, RefundPolicyError> {
        self.policy.qualify(self.workflow, self.basis)
    }
}

fn compensated_case(resolution_valid_until: i64) -> RefundCase {
    let qualification_profile = semantic("business.refund-dispute.compensated.qualification", 1);
    let policy_source = input(
        "governance",
        "closure-policy:refund-dispute:compensated:v1",
        "governance.closure-policy-active",
        100,
    );
    let original_transaction = input(
        "commerce",
        "order:original-1",
        "commerce.accepted-order-history",
        100,
    );
    let resolution = input(
        "governance",
        "resolution:dispute-1",
        "governance.authorized-dispute-resolution",
        resolution_valid_until,
    );
    let refund_disposition = input(
        "finance",
        "refund-obligation-disposition:1",
        "finance.obligation-disposition",
        100,
    );
    let refund_effect = input(
        "finance",
        "refund-effect:1",
        "finance.accepted-refund-effect",
        100,
    );
    let accounting = input(
        "accounting",
        "compensating-event:refund-1",
        "accounting.accepted-compensating-event",
        100,
    );
    let appeal = input(
        "governance",
        "appeal-window:dispute-1:final",
        "governance.appeal-window-final",
        100,
    );
    let finance_reconciliation = reconciliation("finance", "refund-reconciliation:1");
    let refund_obligation = obligation("finance", "refund:order-1");
    let exact_intent = refund_intent();

    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for exact in [
        policy_source.clone(),
        original_transaction.clone(),
        resolution.clone(),
        refund_disposition.clone(),
        refund_effect.clone(),
        accounting.clone(),
        appeal.clone(),
    ] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_reconciliation(finance_reconciliation.clone()));

    let refund_disposition_binding = ObligationDispositionBinding::bind(
        refund_obligation.clone(),
        ObligationDisposition::Satisfied,
        refund_disposition.clone(),
        &cut,
    )
    .unwrap();
    let compensation = CompensationBinding::bind(
        exact_intent.clone(),
        refund_effect.clone(),
        &cut,
    )
    .unwrap();

    let root = node("business:refund-dispute-compensated-close");
    let original_node = node("commerce:original-transaction");
    let resolution_node = node("governance:authorized-resolution");
    let refund_disposition_node = node("finance:refund-obligation-disposition");
    let refund_effect_node = node("finance:accepted-refund-effect");
    let reconciliation_node = node("finance:refund-reconciliation");
    let accounting_node = node("accounting:accepted-compensating-event");
    let appeal_node = node("governance:appeal-window-final");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_node.clone());
    graph.add_dependency(root.clone(), original_node.clone());
    graph.add_dependency(root.clone(), refund_disposition_node.clone());
    graph.add_dependency(root.clone(), accounting_node.clone());
    graph.add_dependency(root.clone(), appeal_node.clone());
    graph.add_dependency(accounting_node.clone(), refund_effect_node.clone());
    graph.add_dependency(refund_effect_node.clone(), reconciliation_node.clone());

    let boundary_requirements = BTreeMap::from([
        (
            original_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("commerce"),
                semantic("commerce.accepted-order-history", 1),
            ),
        ),
        (
            resolution_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("governance"),
                semantic("governance.authorized-dispute-resolution", 1),
            ),
        ),
        (
            refund_disposition_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("finance"),
                semantic("finance.obligation-disposition", 1),
            ),
        ),
        (
            refund_effect_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("finance"),
                semantic("finance.accepted-refund-effect", 1),
            ),
        ),
        (
            reconciliation_node.clone(),
            QualifiedBoundaryRequirement::reconciliation(domain("finance")),
        ),
        (
            accounting_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("accounting"),
                semantic("accounting.accepted-compensating-event", 1),
            ),
        ),
        (
            appeal_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("governance"),
                semantic("governance.appeal-window-final", 1),
            ),
        ),
    ]);

    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        qualification_profile,
        ClosurePolicyRef::new("closure:refund-dispute:compensated").unwrap(),
        semantic("business.refund-dispute.compensated.close", 1),
        ClosureClass::Compensated,
        policy_source,
        root,
        graph,
        boundary_requirements,
        BTreeMap::from([(refund_disposition_node.clone(), domain("finance"))]),
    );

    let boundary_results = BTreeMap::from([
        (
            original_node,
            QualifiedBoundaryRef::Input(original_transaction.clone()),
        ),
        (
            resolution_node,
            QualifiedBoundaryRef::Input(resolution.clone()),
        ),
        (
            refund_disposition_node.clone(),
            QualifiedBoundaryRef::Input(refund_disposition),
        ),
        (
            refund_effect_node.clone(),
            QualifiedBoundaryRef::Input(refund_effect.clone()),
        ),
        (
            reconciliation_node,
            QualifiedBoundaryRef::Reconciliation(finance_reconciliation),
        ),
        (accounting_node, QualifiedBoundaryRef::Input(accounting)),
        (appeal_node, QualifiedBoundaryRef::Input(appeal)),
    ]);

    RefundCase {
        policy: RefundDisputePolicy {
            profile,
            terminal_requirement: TerminalRequirement::Compensation {
                role: refund_effect_node,
                intent: exact_intent,
            },
        },
        workflow: WorkflowRef::new("workflow:refund-dispute:1").unwrap(),
        basis: ClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results,
            obligations: BTreeMap::from([(refund_disposition_node, refund_obligation)]),
            disposition_bindings: vec![refund_disposition_binding],
            exception_bindings: Vec::new(),
            compensating_intents: BTreeSet::from([compensation]),
        },
        original_transaction,
        resolution,
        refund_effect: Some(refund_effect),
        required_exception: None,
    }
}

fn resolved_with_exception_case() -> RefundCase {
    let qualification_profile = semantic("business.refund-dispute.exception.qualification", 1);
    let policy_source = input(
        "governance",
        "closure-policy:refund-dispute:exceptions:v1",
        "governance.closure-policy-active",
        100,
    );
    let original_transaction = input(
        "commerce",
        "order:original-1",
        "commerce.accepted-order-history",
        100,
    );
    let resolution = input(
        "governance",
        "resolution:dispute-1:exception",
        "governance.authorized-dispute-resolution",
        100,
    );
    let exception_source = input(
        "finance",
        "chargeback:pending:1",
        "finance.chargeback-pending",
        100,
    );
    let accounting = input(
        "accounting",
        "dispute-event:1",
        "accounting.accepted-dispute-resolution-event",
        100,
    );
    let appeal = input(
        "governance",
        "appeal-window:dispute-1:active",
        "governance.appeal-window-active",
        100,
    );
    let retained_exception = exception("finance", "chargeback:pending:1");

    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        qualification_profile.clone(),
        TimestampMs::new(80),
    );
    for exact in [
        policy_source.clone(),
        original_transaction.clone(),
        resolution.clone(),
        exception_source.clone(),
        accounting.clone(),
        appeal.clone(),
    ] {
        assert!(cut.insert_input(exact).unwrap());
    }
    assert!(cut.insert_exception(retained_exception.clone()));
    let exception_binding = ExceptionBinding::bind(
        retained_exception.clone(),
        exception_source.clone(),
        &cut,
    )
    .unwrap();

    let root = node("business:refund-dispute-exception-close");
    let original_node = node("commerce:original-transaction");
    let resolution_node = node("governance:authorized-resolution");
    let exception_node = node("finance:retained-chargeback-exception");
    let accounting_node = node("accounting:accepted-dispute-resolution-event");
    let appeal_node = node("governance:appeal-window-active");

    let mut graph = ClosureDependencyGraph::default();
    graph.add_dependency(root.clone(), resolution_node.clone());
    graph.add_dependency(root.clone(), original_node.clone());
    graph.add_dependency(root.clone(), accounting_node.clone());
    graph.add_dependency(root.clone(), appeal_node.clone());
    graph.add_dependency(accounting_node.clone(), exception_node.clone());

    let boundary_requirements = BTreeMap::from([
        (
            original_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("commerce"),
                semantic("commerce.accepted-order-history", 1),
            ),
        ),
        (
            resolution_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("governance"),
                semantic("governance.authorized-dispute-resolution", 1),
            ),
        ),
        (
            exception_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("finance"),
                semantic("finance.chargeback-pending", 1),
            ),
        ),
        (
            accounting_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("accounting"),
                semantic("accounting.accepted-dispute-resolution-event", 1),
            ),
        ),
        (
            appeal_node.clone(),
            QualifiedBoundaryRequirement::input(
                domain("governance"),
                semantic("governance.appeal-window-active", 1),
            ),
        ),
    ]);

    let profile = ClosureQualificationProfile::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        qualification_profile,
        ClosurePolicyRef::new("closure:refund-dispute:resolved-with-exceptions").unwrap(),
        semantic("business.refund-dispute.exception.close", 1),
        ClosureClass::ResolvedWithExceptions,
        policy_source,
        root,
        graph,
        boundary_requirements,
        BTreeMap::new(),
    );

    RefundCase {
        policy: RefundDisputePolicy {
            profile,
            terminal_requirement: TerminalRequirement::Exception {
                role: exception_node.clone(),
                exception: retained_exception.clone(),
            },
        },
        workflow: WorkflowRef::new("workflow:refund-dispute:exception:1").unwrap(),
        basis: ClosureQualificationBasis {
            qualification_cut: cut,
            boundary_results: BTreeMap::from([
                (
                    original_node,
                    QualifiedBoundaryRef::Input(original_transaction.clone()),
                ),
                (
                    resolution_node,
                    QualifiedBoundaryRef::Input(resolution.clone()),
                ),
                (
                    exception_node,
                    QualifiedBoundaryRef::Input(exception_source),
                ),
                (accounting_node, QualifiedBoundaryRef::Input(accounting)),
                (appeal_node, QualifiedBoundaryRef::Input(appeal)),
            ]),
            obligations: BTreeMap::new(),
            disposition_bindings: Vec::new(),
            exception_bindings: vec![exception_binding],
            compensating_intents: BTreeSet::new(),
        },
        original_transaction,
        resolution,
        refund_effect: None,
        required_exception: Some(retained_exception),
    }
}

#[test]
fn compensated_refund_preserves_original_history_and_exact_compensation() {
    let case = compensated_case(100);
    let original = case.original_transaction.clone();
    let refund_effect = case.refund_effect.clone().unwrap();
    let receipt = case.qualify().expect("exact compensated dispute qualifies");

    assert_eq!(receipt.class(), ClosureClass::Compensated);
    assert!(receipt.qualification_cut().inputs().contains(&original));
    assert_eq!(receipt.compensating_intents().len(), 1);
    assert_eq!(
        receipt.compensating_intents().iter().next().unwrap().source(),
        &refund_effect
    );
}

#[test]
fn compensated_refund_requires_authorized_resolution_to_be_current() {
    let case = compensated_case(79);
    assert!(matches!(
        case.qualify(),
        Err(RefundPolicyError::Structural(
            ClosureQualificationError::InvalidCut(_)
        ))
    ));
}

#[test]
fn unrelated_exact_finance_result_cannot_substitute_for_refund_effect_role() {
    let mut case = compensated_case(100);
    let unrelated = input(
        "finance",
        "unrelated-credit:1",
        "finance.accepted-refund-effect",
        100,
    );
    assert!(case
        .basis
        .qualification_cut
        .insert_input(unrelated.clone())
        .unwrap());
    let structurally_valid_but_wrong = CompensationBinding::bind(
        refund_intent(),
        unrelated,
        &case.basis.qualification_cut,
    )
    .expect("core binding proves cut membership, not policy role correspondence");
    case.basis.compensating_intents = BTreeSet::from([structurally_valid_but_wrong]);

    assert_eq!(
        case.qualify(),
        Err(RefundPolicyError::CompensationSourceMismatch)
    );
}

#[test]
fn changed_refund_semantics_cannot_reuse_compensation_role() {
    let mut case = compensated_case(100);
    let changed_intent = CommittedIntent::new(
        LogicalIntentRef::new("refund:order-1:6000").unwrap(),
        semantic("finance.refund", 1),
        OperationCommitment::new("USD-cent:6000:customer-a:original-order-1").unwrap(),
    );
    let source = case.refund_effect.clone().unwrap();
    let binding = CompensationBinding::bind(
        changed_intent,
        source,
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis.compensating_intents = BTreeSet::from([binding]);

    assert_eq!(
        case.qualify(),
        Err(RefundPolicyError::CompensationIntentMismatch)
    );
}

#[test]
fn resolved_with_exceptions_retains_exact_authorized_exception() {
    let case = resolved_with_exception_case();
    let expected_exception = case.required_exception.clone().unwrap();
    let original = case.original_transaction.clone();
    let receipt = case
        .qualify()
        .expect("authorized resolution with exact retained exception qualifies");

    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
    assert!(receipt.qualification_cut().inputs().contains(&original));
    assert!(receipt.exceptions().contains_key(&expected_exception));
}

#[test]
fn unrelated_same_domain_result_cannot_substitute_for_exception_role_source() {
    let mut case = resolved_with_exception_case();
    let unrelated = input(
        "finance",
        "chargeback:other:1",
        "finance.chargeback-pending",
        100,
    );
    assert!(case
        .basis
        .qualification_cut
        .insert_input(unrelated.clone())
        .unwrap());
    let expected_exception = case.required_exception.clone().unwrap();
    let binding = ExceptionBinding::bind(
        expected_exception,
        unrelated,
        &case.basis.qualification_cut,
    )
    .expect("core exception binding proves same-domain cut membership only");
    case.basis.exception_bindings = vec![binding];

    assert_eq!(
        case.qualify(),
        Err(RefundPolicyError::ExceptionSourceMismatch)
    );
}

#[test]
fn exception_profile_does_not_accept_hidden_compensation() {
    let mut case = resolved_with_exception_case();
    let compensation_source = input(
        "finance",
        "refund-effect:hidden:1",
        "finance.accepted-refund-effect",
        100,
    );
    assert!(case
        .basis
        .qualification_cut
        .insert_input(compensation_source.clone())
        .unwrap());
    let compensation = CompensationBinding::bind(
        refund_intent(),
        compensation_source,
        &case.basis.qualification_cut,
    )
    .unwrap();
    case.basis.compensating_intents = BTreeSet::from([compensation]);

    assert_eq!(case.qualify(), Err(RefundPolicyError::UnexpectedCompensation));
}

#[test]
fn unknown_refund_outcome_does_not_become_blind_retry_permission() {
    let source = input(
        "finance",
        "refund-attempt:1:unknown",
        "finance.refund-attempt-outcome",
        100,
    );
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.refund-retry.qualification", 1),
        TimestampMs::new(80),
    );
    assert!(cut.insert_input(source.clone()).unwrap());

    let outcome = AttemptOutcomeBinding::bind(
        AttemptRef::new("attempt:refund:1").unwrap(),
        refund_intent(),
        AttemptOutcome::OutcomeUnknown,
        source,
        &cut,
    )
    .unwrap();

    assert!(!outcome.retry_permitted_without_additional_safety(&cut));
}

#[test]
fn duplicate_refund_request_source_event_does_not_amplify_history() {
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:acme").unwrap(),
        semantic("business.refund-request.qualification", 1),
        TimestampMs::new(80),
    );
    let request = SourceEventRef::new("commerce", "refund-request:1").unwrap();
    assert!(cut.insert_source_event(request.clone()));
    assert!(!cut.insert_source_event(request));
    assert_eq!(cut.source_events().len(), 1);
}
