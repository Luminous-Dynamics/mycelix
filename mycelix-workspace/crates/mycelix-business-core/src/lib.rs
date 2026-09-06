// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Transport-neutral institutional composition contracts for Mycelix Business.
//!
//! This crate deliberately does **not** own identity, governance, finance,
//! inventory, work, accounting, provider, or external-world truth. It encodes a
//! small set of structural invariants needed to compose those independently
//! authoritative domains without manufacturing stronger conclusions.

pub mod allocation;
pub mod causal;
pub mod closure;
pub mod ids;
pub mod qualification;
pub mod time;

pub use allocation::{
    AllocationError, AllocationLine, AllocationStatement, AllocationSummary, Quantity,
};
pub use causal::{AttemptOutcome, CausalIdentity, RetrySafety};
pub use closure::{
    ClosureClass, ClosureError, ClosureRequirements, ObligationDisposition,
    ObligationDispositionBinding, WorkflowClosureReceipt,
};
pub use ids::*;
pub use qualification::{
    AuthorizationBinding, AuthorizationError, CutError, DependencyGraph, GenerationBinding,
    GraphError, QualificationCut, QualifiedInputRef,
};
pub use time::{
    ensure_not_wider, required_horizon, TimestampMs, ValidityEnd, ValidityError, ValidityWindow,
};

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet};

    use super::*;

    fn profile(name: &str, version: u32) -> SemanticProfileId {
        SemanticProfileId::new(name, version).expect("valid profile")
    }

    fn context() -> OrganizationContextRef {
        OrganizationContextRef::new("org:acme").expect("valid context")
    }

    fn domain(name: &str) -> DomainRef {
        DomainRef::new(name).expect("valid domain")
    }

    fn record(name: &str) -> RecordRef {
        RecordRef::new(name).expect("valid record")
    }

    fn intent(name: &str) -> LogicalIntentRef {
        LogicalIntentRef::new(name).expect("valid intent")
    }

    fn authz(domain_name: &str, local_id: &str) -> DomainAuthorizationDecisionRef {
        DomainScopedRef::new(
            domain(domain_name),
            AuthorizationDecisionRef::new(local_id).expect("valid authorization id"),
        )
    }

    fn allocation_ref(domain_name: &str, local_id: &str) -> DomainAllocationRef {
        DomainScopedRef::new(
            domain(domain_name),
            AllocationRef::new(local_id).expect("valid allocation id"),
        )
    }

    fn requirements(names: &[&str]) -> ClosureRequirements {
        ClosureRequirements::new(
            names
                .iter()
                .map(|name| ObligationRef::new(*name).expect("valid obligation id"))
                .collect(),
        )
    }

    fn input(
        domain_name: &str,
        record_name: &str,
        version: u64,
        generation: Option<(&str, u64)>,
        valid_from: i64,
        valid_until: ValidityEnd,
    ) -> QualifiedInputRef {
        QualifiedInputRef {
            domain: domain(domain_name),
            record: record(record_name),
            version,
            semantic_profile: profile("fixture", 1),
            generation: generation.map(|(namespace, generation)| GenerationBinding {
                namespace: GenerationNamespace::new(namespace).expect("valid generation namespace"),
                generation,
            }),
            validity: ValidityWindow::new(TimestampMs::new(valid_from), valid_until)
                .expect("valid window"),
        }
    }

    fn base_cut(at: i64) -> QualificationCut {
        QualificationCut::new(
            context(),
            profile("business-qualification", 1),
            TimestampMs::new(at),
        )
    }

    #[test]
    fn opaque_ids_reject_empty_values() {
        assert_eq!(DomainRef::new("   "), Err(IdError));
        assert!(DomainRef::new("finance").is_ok());
    }

    #[test]
    fn source_event_identity_includes_source_system() {
        let stripe = SourceEventRef::new("stripe", "event:1").unwrap();
        let bank = SourceEventRef::new("bank", "event:1").unwrap();
        assert_ne!(stripe, bank);
    }

    #[test]
    fn duplicate_source_event_does_not_amplify_cut() {
        let mut cut = base_cut(50);
        let event = SourceEventRef::new("stripe", "event:1").unwrap();
        assert!(cut.insert_source_event(event.clone()));
        assert!(!cut.insert_source_event(event));
        assert_eq!(cut.source_events().len(), 1);
    }

    #[test]
    fn domain_scoped_refs_include_domain_namespace() {
        let finance = authz("finance", "authz:1");
        let governance = authz("governance", "authz:1");
        assert_ne!(finance, governance);
    }

    #[test]
    fn equal_local_authorization_ids_from_different_domains_do_not_collapse() {
        let mut cut = base_cut(50);
        assert!(cut.insert_authorization_decision(authz("finance", "authz:1")));
        assert!(cut.insert_authorization_decision(authz("governance", "authz:1")));
        assert_eq!(cut.authorization_decisions().len(), 2);
    }

    #[test]
    fn outcome_unknown_does_not_become_retry_permission() {
        assert!(!AttemptOutcome::OutcomeUnknown.retry_permitted(RetrySafety::Default));
        assert!(AttemptOutcome::ProvenNotApplied.retry_permitted(RetrySafety::Default));
    }

    #[test]
    fn duplicate_safe_profile_can_explicitly_allow_unknown_retry() {
        assert!(AttemptOutcome::OutcomeUnknown
            .retry_permitted(RetrySafety::DuplicateApplicationImpossibleOrAcceptable));
    }

    #[test]
    fn applied_attempt_does_not_become_retry_permission() {
        assert!(!AttemptOutcome::Applied.retry_permitted(RetrySafety::Default));
    }

    #[test]
    fn qualification_cut_is_insertion_order_independent() {
        let first = input(
            "finance",
            "settlement:1",
            1,
            None,
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        );
        let second = input(
            "commerce",
            "order:1",
            3,
            None,
            0,
            ValidityEnd::At(TimestampMs::new(120)),
        );

        let mut a = base_cut(50);
        a.insert_input(first.clone()).unwrap();
        a.insert_input(second.clone()).unwrap();

        let mut b = base_cut(50);
        b.insert_input(second).unwrap();
        b.insert_input(first).unwrap();

        assert_eq!(a, b);
    }

    #[test]
    fn mixed_generation_in_same_domain_namespace_is_rejected() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "finance",
            "policy:a",
            1,
            Some(("policy", 3)),
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        ))
        .unwrap();

        let result = cut.insert_input(input(
            "finance",
            "policy:b",
            1,
            Some(("policy", 4)),
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        ));

        assert!(matches!(result, Err(CutError::GenerationConflict { .. })));
    }

    #[test]
    fn equal_generation_namespace_names_in_different_domains_are_independent() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "finance",
            "policy:finance",
            1,
            Some(("policy", 3)),
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        ))
        .unwrap();
        cut.insert_input(input(
            "governance",
            "policy:governance",
            1,
            Some(("policy", 9)),
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        ))
        .unwrap();
        assert_eq!(cut.inputs().len(), 2);
    }

    #[test]
    fn stale_prerequisite_rejects_qualification() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "governance",
            "role:alice",
            1,
            None,
            0,
            ValidityEnd::At(TimestampMs::new(49)),
        ))
        .unwrap();
        assert!(matches!(
            cut.validate_at_qualification_time(),
            Err(CutError::InputNotValidAtQualification { .. })
        ));
    }

    #[test]
    fn unknown_prerequisite_freshness_fails_closed() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "governance",
            "role:alice",
            1,
            None,
            0,
            ValidityEnd::Unknown,
        ))
        .unwrap();
        assert!(matches!(
            cut.validate_at_qualification_time(),
            Err(CutError::UnknownInputFreshness { .. })
        ));
    }

    #[test]
    fn unknown_domain_authorization_validity_fails_closed() {
        let cut = base_cut(50);
        let result = AuthorizationBinding::new(
            authz("finance", "authz:1"),
            intent("pay:supplier-a:500"),
            profile("finance.pay", 1),
            ValidityWindow::new(TimestampMs::new(0), ValidityEnd::Unknown).unwrap(),
            cut,
        );
        assert!(matches!(
            result,
            Err(AuthorizationError::InvalidValidity(
                ValidityError::UnknownRequiredEnd
            ))
        ));
    }

    #[test]
    fn authorization_cannot_outlive_prerequisite_cut() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "governance",
            "role:alice",
            1,
            None,
            0,
            ValidityEnd::At(TimestampMs::new(75)),
        ))
        .unwrap();
        let binding = AuthorizationBinding::new(
            authz("finance", "authz:1"),
            intent("pay:supplier-a:500"),
            profile("finance.pay", 1),
            ValidityWindow::new(
                TimestampMs::new(0),
                ValidityEnd::At(TimestampMs::new(100)),
            )
            .unwrap(),
            cut,
        )
        .unwrap();
        assert_eq!(
            binding.effective_valid_until,
            ValidityEnd::At(TimestampMs::new(75))
        );
    }

    #[test]
    fn authorization_is_bound_to_exact_logical_intent() {
        let mut cut = base_cut(50);
        cut.insert_input(input(
            "governance",
            "role:alice",
            1,
            None,
            0,
            ValidityEnd::At(TimestampMs::new(100)),
        ))
        .unwrap();

        let authorized_intent = intent("pay:supplier-a:500");
        let binding = AuthorizationBinding::new(
            authz("finance", "authz:1"),
            authorized_intent.clone(),
            profile("finance.pay", 1),
            ValidityWindow::new(
                TimestampMs::new(0),
                ValidityEnd::At(TimestampMs::new(90)),
            )
            .unwrap(),
            cut,
        )
        .unwrap();

        assert!(binding.is_applicable_to(&authorized_intent, TimestampMs::new(80)));
        assert!(!binding.is_applicable_to(
            &intent("pay:supplier-a:600"),
            TimestampMs::new(80)
        ));
        assert!(!binding.is_applicable_to(&authorized_intent, TimestampMs::new(91)));
    }

    #[test]
    fn dependency_graph_rejects_direct_cycle() {
        let mut graph = DependencyGraph::default();
        let closure = DerivationNodeRef::new("business-closure").unwrap();
        let accounting = DerivationNodeRef::new("accounting-acceptance").unwrap();
        graph.add_dependency(closure.clone(), accounting.clone());
        graph.add_dependency(accounting, closure);
        assert!(matches!(
            graph.validate_acyclic(),
            Err(GraphError::CycleDetected(_))
        ));
    }

    #[test]
    fn dependency_graph_accepts_acyclic_derivation() {
        let mut graph = DependencyGraph::default();
        let closure = DerivationNodeRef::new("business-closure").unwrap();
        let accounting = DerivationNodeRef::new("accounting-acceptance").unwrap();
        let finance = DerivationNodeRef::new("finance-settlement").unwrap();
        graph.add_dependency(closure, accounting.clone());
        graph.add_dependency(accounting, finance);
        assert_eq!(graph.validate_acyclic(), Ok(()));
    }

    #[test]
    fn allocation_rejects_over_allocation() {
        let usd = UnitId::new("USD-cent").unwrap();
        let statement = AllocationStatement {
            allocation_ref: allocation_ref("finance", "allocation:1"),
            source_event: SourceEventRef::new("bank", "txn:1").unwrap(),
            semantic_profile: profile("finance.allocation", 1),
            total_available: Quantity::new(usd.clone(), 50_000),
            lines: vec![
                AllocationLine {
                    target: SubjectRef::new("invoice:a").unwrap(),
                    quantity: Quantity::new(usd.clone(), 30_000),
                },
                AllocationLine {
                    target: SubjectRef::new("invoice:b").unwrap(),
                    quantity: Quantity::new(usd, 30_000),
                },
            ],
        };

        assert!(matches!(
            statement.validate_conservation(),
            Err(AllocationError::OverAllocation {
                available: 50_000,
                allocated: 60_000
            })
        ));
    }

    #[test]
    fn divisible_allocation_preserves_remainder() {
        let usd = UnitId::new("USD-cent").unwrap();
        let statement = AllocationStatement {
            allocation_ref: allocation_ref("finance", "allocation:1"),
            source_event: SourceEventRef::new("bank", "txn:1").unwrap(),
            semantic_profile: profile("finance.allocation", 1),
            total_available: Quantity::new(usd.clone(), 100_000),
            lines: vec![
                AllocationLine {
                    target: SubjectRef::new("invoice:a").unwrap(),
                    quantity: Quantity::new(usd.clone(), 60_000),
                },
                AllocationLine {
                    target: SubjectRef::new("invoice:b").unwrap(),
                    quantity: Quantity::new(usd.clone(), 40_000),
                },
            ],
        };

        let summary = statement.validate_conservation().unwrap();
        assert_eq!(summary.allocated, Quantity::new(usd.clone(), 100_000));
        assert_eq!(summary.remaining, Quantity::new(usd, 0));
    }

    #[test]
    fn allocation_rejects_unit_mismatch() {
        let usd = UnitId::new("USD-cent").unwrap();
        let eur = UnitId::new("EUR-cent").unwrap();
        let statement = AllocationStatement {
            allocation_ref: allocation_ref("finance", "allocation:1"),
            source_event: SourceEventRef::new("bank", "txn:1").unwrap(),
            semantic_profile: profile("finance.allocation", 1),
            total_available: Quantity::new(usd, 100),
            lines: vec![AllocationLine {
                target: SubjectRef::new("invoice:a").unwrap(),
                quantity: Quantity::new(eur, 100),
            }],
        };

        assert!(matches!(
            statement.validate_conservation(),
            Err(AllocationError::UnitMismatch { .. })
        ));
    }

    #[test]
    fn waiver_is_never_performance_satisfaction() {
        assert!(!ObligationDisposition::Waived.performance_satisfied());
        assert!(ObligationDisposition::Satisfied.performance_satisfied());
    }

    #[test]
    fn breach_does_not_silently_discharge_obligation() {
        assert!(!ObligationDisposition::Breached.is_terminal_disposition());
        assert!(!ObligationDisposition::Disputed.is_terminal_disposition());
    }

    #[test]
    fn satisfied_closure_rejects_omitted_required_obligation() {
        let result = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:1").unwrap(),
            ClosurePolicyRef::new("closure:service-sale").unwrap(),
            profile("service-sale-close", 1),
            base_cut(50),
            requirements(&["obligation:payment"]),
            ClosureClass::Satisfied,
            BTreeMap::new(),
            BTreeSet::new(),
            BTreeSet::new(),
        );
        assert!(matches!(
            result,
            Err(ClosureError::MissingRequiredObligation { .. })
        ));
    }

    #[test]
    fn satisfied_closure_rejects_waived_required_obligation() {
        let mut obligations = BTreeMap::new();
        obligations.insert(
            ObligationRef::new("obligation:payment").unwrap(),
            ObligationDisposition::Waived,
        );
        let result = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:1").unwrap(),
            ClosurePolicyRef::new("closure:service-sale").unwrap(),
            profile("service-sale-close", 1),
            base_cut(50),
            requirements(&["obligation:payment"]),
            ClosureClass::Satisfied,
            obligations,
            BTreeSet::new(),
            BTreeSet::new(),
        );
        assert!(matches!(
            result,
            Err(ClosureError::SatisfactionStrengthening { .. })
        ));
    }

    #[test]
    fn waived_closure_rejects_active_required_obligation() {
        let payment = ObligationRef::new("obligation:payment").unwrap();
        let fulfillment = ObligationRef::new("obligation:fulfillment").unwrap();
        let mut obligations = BTreeMap::new();
        obligations.insert(payment, ObligationDisposition::Waived);
        obligations.insert(fulfillment, ObligationDisposition::Active);

        let result = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:waiver").unwrap(),
            ClosurePolicyRef::new("closure:waiver").unwrap(),
            profile("waiver-close", 1),
            base_cut(50),
            requirements(&["obligation:payment", "obligation:fulfillment"]),
            ClosureClass::Waived,
            obligations,
            BTreeSet::new(),
            BTreeSet::new(),
        );
        assert!(matches!(
            result,
            Err(ClosureError::NonTerminalRequiredObligation { .. })
        ));
    }

    #[test]
    fn resolved_with_exceptions_requires_exact_exception_set() {
        let result = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:month-close").unwrap(),
            ClosurePolicyRef::new("closure:month-end").unwrap(),
            profile("month-end-close", 1),
            base_cut(50),
            ClosureRequirements::default(),
            ClosureClass::ResolvedWithExceptions,
            BTreeMap::new(),
            BTreeSet::new(),
            BTreeSet::new(),
        );
        assert_eq!(result, Err(ClosureError::ResolvedWithoutExceptions));
    }

    #[test]
    fn compensated_closure_requires_independent_compensating_intent() {
        let result = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:refund").unwrap(),
            ClosurePolicyRef::new("closure:refund").unwrap(),
            profile("refund-close", 1),
            base_cut(50),
            ClosureRequirements::default(),
            ClosureClass::Compensated,
            BTreeMap::new(),
            BTreeSet::new(),
            BTreeSet::new(),
        );
        assert_eq!(result, Err(ClosureError::CompensatedWithoutCompensation));
    }

    #[test]
    fn compensated_closure_retains_distinct_compensating_identity() {
        let mut compensations = BTreeSet::new();
        compensations.insert(intent("refund:payment:1"));
        let receipt = WorkflowClosureReceipt::new(
            WorkflowRef::new("workflow:refund").unwrap(),
            ClosurePolicyRef::new("closure:refund").unwrap(),
            profile("refund-close", 1),
            base_cut(50),
            ClosureRequirements::default(),
            ClosureClass::Compensated,
            BTreeMap::new(),
            BTreeSet::new(),
            compensations,
        )
        .unwrap();
        assert_eq!(receipt.class, ClosureClass::Compensated);
        assert!(!receipt.compensating_intents.is_empty());
    }
}
