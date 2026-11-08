// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BTreeSet;

use mycelix_business_core::{
    ClosureClass, ClosureError, ClosurePolicyRef, ClosureRequirements, CutError, DependencyGraph,
    DerivationNodeRef, DomainExceptionRef, DomainRef, DomainScopedRef, ExceptionBinding,
    ExceptionRef, GraphError, OrganizationContextRef, QualificationCut, QualifiedInputRef,
    RecordRef, SemanticProfileId, TimestampMs, ValidityEnd, ValidityWindow,
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

fn exception(domain_name: &str, local_id: &str) -> DomainExceptionRef {
    DomainScopedRef::new(
        domain(domain_name),
        ExceptionRef::new(local_id).expect("valid exception"),
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum MonthEndError {
    WrongOrganization {
        expected: OrganizationContextRef,
        actual: OrganizationContextRef,
    },
    WrongQualificationProfile {
        expected: SemanticProfileId,
        actual: SemanticProfileId,
    },
    UnsupportedClosureClass(ClosureClass),
    MissingAccountingCloseResult(QualifiedInputRef),
    Dependency(GraphError),
    Kernel(ClosureError),
}

impl From<GraphError> for MonthEndError {
    fn from(value: GraphError) -> Self {
        Self::Dependency(value)
    }
}

impl From<ClosureError> for MonthEndError {
    fn from(value: ClosureError) -> Self {
        Self::Kernel(value)
    }
}

/// GP-006 Business-side orchestration policy.
///
/// Accounting owns period recognition, journal/ledger semantics, adjustment
/// authority, reconciliation policy, and the actual accounting close decision.
/// Business therefore consumes one exact Accounting-qualified close result rather
/// than flattening the entire ledger/evidence graph into a Business-owned cut.
#[derive(Clone)]
struct MonthEndPolicy {
    organization_context: OrganizationContextRef,
    qualification_profile: SemanticProfileId,
    accounting_close_result: QualifiedInputRef,
}

impl MonthEndPolicy {
    fn dependency_graph(&self, class: ClosureClass) -> Result<DependencyGraph, MonthEndError> {
        let mut graph = DependencyGraph::default();
        let business_closure = DerivationNodeRef::new("business:month-end-workflow-closure").unwrap();
        let accounting_close = DerivationNodeRef::new("accounting:period-close-result").unwrap();
        let event_set = DerivationNodeRef::new("accounting:accepted-event-set-commitment").unwrap();
        let accounting_policy = DerivationNodeRef::new("accounting:policy-commitment").unwrap();
        let adjustment_set = DerivationNodeRef::new("accounting:adjustment-set-commitment").unwrap();
        let reconciliation_set =
            DerivationNodeRef::new("accounting:reconciliation-set-commitment").unwrap();
        let close_authority = DerivationNodeRef::new("accounting:close-authority-result").unwrap();

        graph.add_dependency(business_closure.clone(), accounting_close.clone());
        graph.add_dependency(accounting_close.clone(), event_set);
        graph.add_dependency(accounting_close.clone(), accounting_policy);
        graph.add_dependency(accounting_close.clone(), adjustment_set);
        graph.add_dependency(accounting_close.clone(), reconciliation_set);
        graph.add_dependency(accounting_close.clone(), close_authority);

        match class {
            ClosureClass::Satisfied => {}
            ClosureClass::ResolvedWithExceptions => {
                graph.add_dependency(
                    business_closure,
                    DerivationNodeRef::new("accounting:retained-exception-set").unwrap(),
                );
            }
            unsupported => return Err(MonthEndError::UnsupportedClosureClass(unsupported)),
        }

        Ok(graph)
    }

    fn qualify_with_graph(
        &self,
        graph: &DependencyGraph,
        cut: QualificationCut,
        class: ClosureClass,
        exceptions: Vec<ExceptionBinding>,
    ) -> Result<WorkflowClosureReceipt, MonthEndError> {
        if cut.organization_context != self.organization_context {
            return Err(MonthEndError::WrongOrganization {
                expected: self.organization_context.clone(),
                actual: cut.organization_context.clone(),
            });
        }
        if cut.semantic_profile != self.qualification_profile {
            return Err(MonthEndError::WrongQualificationProfile {
                expected: self.qualification_profile.clone(),
                actual: cut.semantic_profile.clone(),
            });
        }

        graph.validate_acyclic()?;

        if !cut.inputs().contains(&self.accounting_close_result) {
            return Err(MonthEndError::MissingAccountingCloseResult(
                self.accounting_close_result.clone(),
            ));
        }

        WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:month-end:2026-08").expect("valid workflow"),
            ClosurePolicyRef::new("closure:month-end:gp-006").expect("valid policy"),
            profile("business.month-end.close", 1),
            cut,
            ClosureRequirements::default(),
            class,
            Vec::new(),
            exceptions,
            BTreeSet::new(),
        )
        .map_err(Into::into)
    }

    fn qualify(
        &self,
        cut: QualificationCut,
        class: ClosureClass,
        exceptions: Vec<ExceptionBinding>,
    ) -> Result<WorkflowClosureReceipt, MonthEndError> {
        let graph = self.dependency_graph(class)?;
        self.qualify_with_graph(&graph, cut, class, exceptions)
    }
}

struct Fixture {
    policy: MonthEndPolicy,
}

impl Fixture {
    fn new() -> Self {
        Self::with_close_version(1, 1, 100)
    }

    fn with_close_version(close_version: u64, qualification_version: u32, valid_until: i64) -> Self {
        Self {
            policy: MonthEndPolicy {
                organization_context: OrganizationContextRef::new("org:acme")
                    .expect("valid context"),
                qualification_profile: profile(
                    "business.month-end-close.qualification",
                    qualification_version,
                ),
                accounting_close_result: input(
                    "accounting",
                    "period-close:2026-08",
                    close_version,
                    "accounting.period-close-qualified",
                    valid_until,
                ),
            },
        }
    }

    fn cut(&self) -> QualificationCut {
        let mut cut = QualificationCut::new(
            self.policy.organization_context.clone(),
            self.policy.qualification_profile.clone(),
            TimestampMs::new(80),
        );
        assert!(cut
            .insert_input(self.policy.accounting_close_result.clone())
            .unwrap());
        cut
    }

    fn cut_with_exception(
        &self,
        exception_ref: DomainExceptionRef,
        source: QualifiedInputRef,
    ) -> (QualificationCut, ExceptionBinding) {
        let mut cut = self.cut();
        assert!(cut.insert_input(source.clone()).unwrap());
        assert!(cut.insert_exception(exception_ref.clone()));
        let binding = ExceptionBinding::bind(exception_ref, source, &cut)
            .expect("exact Accounting exception provenance");
        (cut, binding)
    }
}

#[test]
fn gp006_business_orchestration_can_close_from_one_accounting_qualified_result() {
    let fixture = Fixture::new();
    let cut = fixture.cut();

    let receipt = fixture
        .policy
        .qualify(cut, ClosureClass::Satisfied, Vec::new())
        .expect("Accounting-qualified period close supports Business orchestration closure");

    assert_eq!(receipt.class(), ClosureClass::Satisfied);
    assert!(receipt.obligations().is_empty());
    assert!(receipt.exceptions().is_empty());
}

#[test]
fn business_does_not_flatten_accounting_internal_evidence_into_its_cut() {
    let fixture = Fixture::new();
    let cut = fixture.cut();

    assert_eq!(cut.inputs().len(), 1);
    assert!(cut
        .inputs()
        .iter()
        .all(|item| item.domain == domain("accounting")));
    assert!(cut.reconciliations().is_empty());
    assert!(cut.source_events().is_empty());
    assert!(cut.authorization_decisions().is_empty());

    fixture
        .policy
        .qualify(cut, ClosureClass::Satisfied, Vec::new())
        .expect("nested Accounting proof remains authoritative and compressed");
}

#[test]
fn business_cannot_close_month_end_without_exact_accounting_close_result() {
    let fixture = Fixture::new();
    let cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );

    assert_eq!(
        fixture
            .policy
            .qualify(cut, ClosureClass::Satisfied, Vec::new()),
        Err(MonthEndError::MissingAccountingCloseResult(
            fixture.policy.accounting_close_result.clone()
        ))
    );
}

#[test]
fn same_exact_month_end_cut_is_deterministic() {
    let fixture = Fixture::new();
    let first = fixture
        .policy
        .qualify(fixture.cut(), ClosureClass::Satisfied, Vec::new())
        .unwrap();
    let second = fixture
        .policy
        .qualify(fixture.cut(), ClosureClass::Satisfied, Vec::new())
        .unwrap();

    assert_eq!(first, second);
}

#[test]
fn cross_organization_month_end_result_cannot_be_replayed() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        OrganizationContextRef::new("org:other").unwrap(),
        fixture.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    assert!(cut
        .insert_input(fixture.policy.accounting_close_result.clone())
        .unwrap());

    assert!(matches!(
        fixture
            .policy
            .qualify(cut, ClosureClass::Satisfied, Vec::new()),
        Err(MonthEndError::WrongOrganization { .. })
    ));
}

#[test]
fn month_end_qualification_profile_substitution_is_denied() {
    let fixture = Fixture::new();
    let mut cut = QualificationCut::new(
        fixture.policy.organization_context.clone(),
        profile("business.month-end-close.qualification", 2),
        TimestampMs::new(80),
    );
    assert!(cut
        .insert_input(fixture.policy.accounting_close_result.clone())
        .unwrap());

    assert!(matches!(
        fixture
            .policy
            .qualify(cut, ClosureClass::Satisfied, Vec::new()),
        Err(MonthEndError::WrongQualificationProfile { .. })
    ));
}

#[test]
fn stale_accounting_close_result_fails_current_business_qualification() {
    let fixture = Fixture::with_close_version(1, 1, 79);
    let cut = fixture.cut();

    assert!(matches!(
        fixture
            .policy
            .qualify(cut, ClosureClass::Satisfied, Vec::new()),
        Err(MonthEndError::Kernel(ClosureError::InvalidQualification(
            CutError::InputNotValidAtQualification { .. }
        )))
    ));
}

#[test]
fn corrected_accounting_close_version_requires_explicit_requalification() {
    let historical = Fixture::with_close_version(1, 1, 100);
    let historical_receipt = historical
        .policy
        .qualify(historical.cut(), ClosureClass::Satisfied, Vec::new())
        .expect("historical close qualifies under v1");

    let current = Fixture::with_close_version(2, 2, 120);
    let mut current_cut_with_old_result = QualificationCut::new(
        current.policy.organization_context.clone(),
        current.policy.qualification_profile.clone(),
        TimestampMs::new(80),
    );
    assert!(current_cut_with_old_result
        .insert_input(historical.policy.accounting_close_result.clone())
        .unwrap());

    assert_eq!(
        current.policy.qualify(
            current_cut_with_old_result,
            ClosureClass::Satisfied,
            Vec::new()
        ),
        Err(MonthEndError::MissingAccountingCloseResult(
            current.policy.accounting_close_result.clone()
        ))
    );

    let current_receipt = current
        .policy
        .qualify(current.cut(), ClosureClass::Satisfied, Vec::new())
        .expect("current corrected close qualifies under explicit v2 lineage");

    assert!(historical_receipt
        .qualification_cut()
        .inputs()
        .contains(&historical.policy.accounting_close_result));
    assert!(!historical_receipt
        .qualification_cut()
        .inputs()
        .contains(&current.policy.accounting_close_result));
    assert!(current_receipt
        .qualification_cut()
        .inputs()
        .contains(&current.policy.accounting_close_result));
}

#[test]
fn close_with_exception_retains_exact_accounting_exception_provenance() {
    let fixture = Fixture::new();
    let exception_ref = exception("accounting", "exception:late-bank-item:1");
    let source = input(
        "accounting",
        "close-exception:late-bank-item:1",
        1,
        "accounting.close-exception-accepted",
        100,
    );
    let (cut, binding) = fixture.cut_with_exception(exception_ref.clone(), source.clone());

    let receipt = fixture
        .policy
        .qualify(
            cut,
            ClosureClass::ResolvedWithExceptions,
            vec![binding],
        )
        .expect("policy-approved close with exact retained exception qualifies");

    assert_eq!(receipt.class(), ClosureClass::ResolvedWithExceptions);
    assert_eq!(receipt.exceptions().len(), 1);
    let retained = receipt.exceptions().get(&exception_ref).expect("retained exception");
    assert_eq!(retained.source(), &source);
}

#[test]
fn receipt_cannot_hide_accounting_exception_retained_by_cut() {
    let fixture = Fixture::new();
    let exception_ref = exception("accounting", "exception:late-bank-item:1");
    let source = input(
        "accounting",
        "close-exception:late-bank-item:1",
        1,
        "accounting.close-exception-accepted",
        100,
    );
    let (cut, _binding) = fixture.cut_with_exception(exception_ref, source);

    assert!(matches!(
        fixture.policy.qualify(
            cut,
            ClosureClass::ResolvedWithExceptions,
            Vec::new()
        ),
        Err(MonthEndError::Kernel(ClosureError::ExceptionSetMismatch { .. }))
    ));
}

#[test]
fn receipt_cannot_invent_exception_not_in_accounting_close_cut() {
    let fixture = Fixture::new();
    let exception_ref = exception("accounting", "exception:invented");
    let source = input(
        "accounting",
        "close-exception:invented",
        1,
        "accounting.close-exception-accepted",
        100,
    );
    let mut cut = fixture.cut();
    assert!(cut.insert_input(source.clone()).unwrap());
    let binding = ExceptionBinding::bind(exception_ref, source, &cut)
        .expect("binding proves source provenance, not cut exception membership");

    assert!(matches!(
        fixture.policy.qualify(
            cut,
            ClosureClass::ResolvedWithExceptions,
            vec![binding]
        ),
        Err(MonthEndError::Kernel(ClosureError::ExceptionSetMismatch { .. }))
    ));
}

#[test]
fn stale_exception_source_cannot_support_current_close_with_exceptions() {
    let fixture = Fixture::new();
    let exception_ref = exception("accounting", "exception:stale-item:1");
    let source = input(
        "accounting",
        "close-exception:stale-item:1",
        1,
        "accounting.close-exception-accepted",
        79,
    );
    let (cut, binding) = fixture.cut_with_exception(exception_ref, source);

    assert!(matches!(
        fixture.policy.qualify(
            cut,
            ClosureClass::ResolvedWithExceptions,
            vec![binding]
        ),
        Err(MonthEndError::Kernel(ClosureError::InvalidQualification(
            CutError::InputNotValidAtQualification { .. }
        )))
    ));
}

#[test]
fn unsupported_month_end_closure_class_does_not_inherit_an_unrelated_proof_shape() {
    let fixture = Fixture::new();

    assert_eq!(
        fixture
            .policy
            .qualify(fixture.cut(), ClosureClass::Terminated, Vec::new()),
        Err(MonthEndError::UnsupportedClosureClass(
            ClosureClass::Terminated
        ))
    );
}

#[test]
fn accounting_close_result_cannot_depend_on_same_business_closure() {
    let fixture = Fixture::new();
    let cut = fixture.cut();
    let mut graph = fixture
        .policy
        .dependency_graph(ClosureClass::Satisfied)
        .expect("Satisfied month-end dependency profile is supported");

    graph.add_dependency(
        DerivationNodeRef::new("accounting:period-close-result").unwrap(),
        DerivationNodeRef::new("business:month-end-workflow-closure").unwrap(),
    );

    assert!(matches!(
        fixture.policy.qualify_with_graph(
            &graph,
            cut,
            ClosureClass::Satisfied,
            Vec::new()
        ),
        Err(MonthEndError::Dependency(GraphError::CycleDetected(_)))
    ));
}
