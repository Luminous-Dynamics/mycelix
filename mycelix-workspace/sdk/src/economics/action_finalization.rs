// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Action Finalization Gate
//!
//! AC-045 defines a deterministic close-out boundary after lifecycle,
//! execution, and impact accounting have all been exercised.
//!
//! Finalization is deliberately separate from lifecycle mutation. A lifecycle
//! may reach Completed, but the action is not eligible for clean finalization
//! until:
//! - a Completion execution constraint has been satisfied;
//! - every explicitly supplied required execution constraint has at least one
//!   conformant reconciliation and no non-conformant reconciliation;
//! - the current economic integrity assessment is clean;
//! - no unresolved impact exposure remains.
//!
//! Historical execution constraints are permitted: a delivery/payment/milestone
//! can have been reconciled under an earlier lifecycle revision and later be
//! finalized after the action reaches Completed. The constraint must still
//! exactly match a revision in the immutable lifecycle history.
//!
//! This is a reference model. It does not prescribe settlement law, local
//! accounting policy, or tolerance semantics.

use super::{
    action_lifecycle::{EconomicActionLifecycle, EconomicActionStage},
    execution_reconciliation::{
        EconomicExecutionConstraint, EconomicExecutionReconciliationLedger, ExecutionConformance,
    },
    execution_receipt::EconomicExecutionKind,
    impact::ImpactLedger,
    integrity_gate::{
        EconomicActionScope, EconomicIntegrityDecision, EconomicIntegrityGate,
        ScopedEconomicIntegrityAssessment,
    },
    substrate::SubstrateLedger,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Outcome of the finalization eligibility assessment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EconomicFinalizationDecision {
    /// The action has a complete, clean close-out certificate.
    Ready,
    /// The lifecycle is not at normal completion.
    BlockedByLifecycle,
    /// One or more required execution controls were not supplied.
    BlockedByMissingConstraint,
    /// A required execution constraint has no reconciliation.
    BlockedByMissingReconciliation,
    /// At least one required reconciliation is non-conformant.
    BlockedByNonConformance,
    /// Impact evidence or restoration state is unresolved.
    BlockedByUnresolvedImpact,
    /// Substrate/impact integrity is not clean enough for finalization.
    BlockedByIntegrity,
    /// Explicit emergency escalation takes precedence over ordinary closure.
    EmergencyEscalationRequired,
}

/// Auditable finalization assessment.
///
/// The IDs retained here make the reason for a non-ready result inspectable
/// without rewriting or deleting the underlying append-only ledgers.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicFinalizationAssessment {
    /// Stable action identifier.
    pub action_ref: String,
    /// Lifecycle revision representing the current state.
    pub lifecycle_revision_id: String,
    /// Current scope identifier.
    pub scope_id: String,
    /// Current scope fingerprint.
    pub scope_fingerprint: String,
    /// Whether the explicit completion control is present.
    pub completion_constraint_present: bool,
    /// Finalization outcome.
    pub decision: EconomicFinalizationDecision,
    /// Required constraint IDs with no reconciliation.
    pub missing_constraint_ids: Vec<String>,
    /// Required reconciliation IDs whose recorded result was non-conformant.
    pub nonconformant_reconciliation_ids: Vec<String>,
    /// Open depletion impacts.
    pub open_impact_ids: Vec<String>,
    /// Attributed/remediating depletion impacts still unresolved.
    pub remediation_impact_ids: Vec<String>,
    /// Restoration obligations still blocking discretionary extraction.
    pub blocking_obligation_ids: Vec<String>,
    /// The current scoped AC-017/AC-018 assessment.
    pub integrity: ScopedEconomicIntegrityAssessment,
}

impl EconomicFinalizationAssessment {
    /// Whether the action has a clean finalization certificate.
    pub fn is_ready(&self) -> bool {
        self.decision == EconomicFinalizationDecision::Ready
    }

    /// Whether explicit escalation is required.
    pub fn requires_escalation(&self) -> bool {
        self.decision == EconomicFinalizationDecision::EmergencyEscalationRequired
    }
}

/// Stateless finalization gate.
#[derive(Debug, Default, Clone, Copy)]
pub struct EconomicActionFinalizationGate;

impl EconomicActionFinalizationGate {
    /// Assess whether a completed action can receive a clean finalization
    /// certificate.
    ///
    /// The order is fail-closed and deterministic:
    /// emergency escalation > lifecycle > missing controls > non-conformance
    /// > unresolved impact > non-clean integrity > ready.
    ///
    /// Required constraints are validated against immutable lifecycle history
    /// rather than only the current revision. This preserves valid evidence
    /// from earlier execution stages while preventing forged revision, scope,
    /// or fingerprint bindings.
    pub fn assess(
        lifecycle: &EconomicActionLifecycle,
        scope: &EconomicActionScope,
        substrate: &SubstrateLedger,
        impacts: &ImpactLedger,
        reconciliations: &EconomicExecutionReconciliationLedger,
        constraints: &[EconomicExecutionConstraint],
    ) -> Result<EconomicFinalizationAssessment, String> {
        lifecycle.validate()?;
        reconciliations.validate()?;
        scope.validate()?;

        let current_revision = lifecycle.current_revision()?;
        let scope_fingerprint = scope.fingerprint()?;

        if scope.action_ref != lifecycle.action_ref() {
            return Err("Finalization scope action reference does not match lifecycle".into());
        }
        if scope.scope_id != lifecycle.active_scope_id() {
            return Err("Finalization scope ID does not match active lifecycle scope".into());
        }
        if scope_fingerprint != lifecycle.active_scope_fingerprint() {
            return Err(
                "Finalization scope fingerprint does not match active lifecycle scope".into(),
            );
        }

        let integrity = EconomicIntegrityGate::assess_scoped(substrate, impacts, scope)?;

        let mut constraints_by_id: BTreeMap<String, &EconomicExecutionConstraint> = BTreeMap::new();
        let mut completion_constraint_present = false;

        for constraint in constraints {
            constraint.validate()?;

            if constraint.action_ref != lifecycle.action_ref()
                || constraint.action_ref != scope.action_ref
            {
                return Err(format!(
                    "Finalization constraint {} has an action identity mismatch",
                    constraint.constraint_id
                ));
            }

            let revision = lifecycle
                .revisions()
                .iter()
                .find(|revision| revision.revision_id == constraint.lifecycle_revision_id)
                .ok_or_else(|| {
                    format!(
                        "Finalization constraint {} references an unknown lifecycle revision",
                        constraint.constraint_id
                    )
                })?;

            if revision.action_ref != constraint.action_ref
                || revision.scope_id != constraint.scope_id
                || revision.scope_fingerprint != constraint.scope_fingerprint
            {
                return Err(format!(
                    "Finalization constraint {} is not bound to its referenced lifecycle revision",
                    constraint.constraint_id
                ));
            }

            match (constraint.kind, revision.stage) {
                (
                    EconomicExecutionKind::Payment
                    | EconomicExecutionKind::Delivery
                    | EconomicExecutionKind::Milestone,
                    EconomicActionStage::Contracted | EconomicActionStage::Implementation,
                ) => {}
                (EconomicExecutionKind::Completion, EconomicActionStage::Completed) => {
                    completion_constraint_present = true;
                    if constraint.lifecycle_revision_id != current_revision.revision_id {
                        return Err(
                            "Completion finalization constraint must reference the current Completed revision"
                                .into(),
                        );
                    }
                }
                _ => {
                    return Err(format!(
                        "Finalization constraint {} uses an execution kind incompatible with its lifecycle revision",
                        constraint.constraint_id
                    ));
                }
            }

            if constraints_by_id
                .insert(constraint.constraint_id.clone(), constraint)
                .is_some()
            {
                return Err(format!(
                    "Duplicate finalization constraint ID: {}",
                    constraint.constraint_id
                ));
            }
        }

        let mut matched = BTreeSet::new();
        let mut nonconformant_reconciliation_ids = Vec::new();

        for reconciliation in reconciliations.reconciliations() {
            if !constraints_by_id.contains_key(&reconciliation.constraint_id) {
                continue;
            }

            matched.insert(reconciliation.constraint_id.clone());
            if reconciliation.result != ExecutionConformance::Conformant {
                nonconformant_reconciliation_ids.push(reconciliation.reconciliation_id.clone());
            }
        }

        let mut missing_constraint_ids = constraints_by_id
            .keys()
            .filter(|constraint_id| !matched.contains(constraint_id.as_str()))
            .cloned()
            .collect::<Vec<_>>();
        missing_constraint_ids.sort();
        nonconformant_reconciliation_ids.sort();

        let exposure = impacts.exposure();

        let decision = if integrity.assessment.decision
            == EconomicIntegrityDecision::EmergencyEscalationRequired
        {
            EconomicFinalizationDecision::EmergencyEscalationRequired
        } else if current_revision.stage != EconomicActionStage::Completed {
            EconomicFinalizationDecision::BlockedByLifecycle
        } else if !completion_constraint_present {
            EconomicFinalizationDecision::BlockedByMissingConstraint
        } else if !missing_constraint_ids.is_empty() {
            EconomicFinalizationDecision::BlockedByMissingReconciliation
        } else if !nonconformant_reconciliation_ids.is_empty() {
            EconomicFinalizationDecision::BlockedByNonConformance
        } else if !exposure.open_impact_ids.is_empty()
            || !exposure.remediation_impact_ids.is_empty()
            || !exposure.blocking_obligation_ids.is_empty()
        {
            EconomicFinalizationDecision::BlockedByUnresolvedImpact
        } else if integrity.assessment.decision != EconomicIntegrityDecision::Allowed {
            EconomicFinalizationDecision::BlockedByIntegrity
        } else {
            EconomicFinalizationDecision::Ready
        };

        Ok(EconomicFinalizationAssessment {
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: current_revision.revision_id.clone(),
            scope_id: scope.scope_id.clone(),
            scope_fingerprint,
            completion_constraint_present,
            decision,
            missing_constraint_ids,
            nonconformant_reconciliation_ids,
            open_impact_ids: exposure.open_impact_ids,
            remediation_impact_ids: exposure.remediation_impact_ids,
            blocking_obligation_ids: exposure.blocking_obligation_ids,
            integrity,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::{
        action_lifecycle::{EconomicActionChangeKind, EconomicActionScope},
        execution_reconciliation::EconomicExecutionReconciliation,
        execution_receipt::EconomicExecutionReceipt,
        impact::{ImpactDirection, ImpactStatus, SubstrateImpact},
        substrate::{
            DistributionPurpose, SubstrateAccount, SubstrateBoundary, SubstrateDimension,
        },
    };

    fn scope() -> EconomicActionScope {
        EconomicActionScope {
            scope_id: "scope:1".into(),
            action_ref: "action:1".into(),
            purpose: DistributionPurpose::Discretionary,
            required_dimensions: vec![SubstrateDimension::Financial],
            policy_ref: "policy:finalization:v1".into(),
            authority_ref: "authority:dao-1".into(),
            attestation_ref: "attestation:scope:1".into(),
            evidence_refs: vec!["evidence:scope".into()],
            declared_at: 1_000,
        }
    }

    fn implementation_lifecycle() -> EconomicActionLifecycle {
        let mut lifecycle = EconomicActionLifecycle::start(
            &scope(),
            "revision:1",
            "authority:dao-1",
            vec!["evidence:planning".into()],
            1_000,
        )
        .unwrap();

        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope(),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();
        lifecycle
            .record(
                "revision:3",
                EconomicActionStage::Implementation,
                EconomicActionChangeKind::Update,
                &scope(),
                "authority:dao-1",
                vec!["evidence:implementation".into()],
                1_200,
            )
            .unwrap();

        lifecycle
    }

    fn completed_lifecycle() -> EconomicActionLifecycle {
        let mut lifecycle = implementation_lifecycle();
        lifecycle
            .record(
                "revision:4",
                EconomicActionStage::Completed,
                EconomicActionChangeKind::Completion,
                &scope(),
                "authority:dao-1",
                vec!["evidence:completion".into()],
                1_400,
            )
            .unwrap();
        lifecycle
    }

    fn delivery_constraint(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionConstraint {
        let revision = &lifecycle.revisions()[2];
        EconomicExecutionConstraint {
            constraint_id: "constraint:delivery".into(),
            action_ref: "action:1".into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Delivery,
            quantity: 100,
            quantity_unit: "unit".into(),
            evidence_refs: vec!["evidence:delivery-constraint".into()],
        }
    }

    fn completion_constraint(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionConstraint {
        let revision = lifecycle.current_revision().unwrap();
        EconomicExecutionConstraint {
            constraint_id: "constraint:completion".into(),
            action_ref: "action:1".into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Completion,
            quantity: 1,
            quantity_unit: "event".into(),
            evidence_refs: vec!["evidence:completion-constraint".into()],
        }
    }

    fn delivery_receipt(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionReceipt {
        let revision = lifecycle.current_revision().unwrap();
        EconomicExecutionReceipt {
            execution_id: "execution:delivery".into(),
            action_ref: "action:1".into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Delivery,
            quantity: Some(100),
            quantity_unit: Some("unit".into()),
            external_ref: "external:delivery".into(),
            evidence_refs: vec!["evidence:delivery".into()],
            recorded_at: 1_250,
        }
    }

    fn completion_receipt(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionReceipt {
        let revision = lifecycle.current_revision().unwrap();
        EconomicExecutionReceipt {
            execution_id: "execution:completion".into(),
            action_ref: "action:1".into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Completion,
            quantity: Some(1),
            quantity_unit: Some("event".into()),
            external_ref: "external:completion".into(),
            evidence_refs: vec!["evidence:completion".into()],
            recorded_at: 1_500,
        }
    }

    fn reconciliation_ledger(
        entries: Vec<EconomicExecutionReconciliation>,
    ) -> EconomicExecutionReconciliationLedger {
        let json = serde_json::to_string(&serde_json::json!({
            "reconciliations": entries
        }))
        .unwrap();
        serde_json::from_str(&json).unwrap()
    }

    fn healthy_substrate() -> SubstrateLedger {
        let mut ledger = SubstrateLedger::new();
        ledger
            .register_account(SubstrateAccount::new(
                SubstrateDimension::Financial,
                "sap",
                100,
                100,
                SubstrateBoundary::minimum(50, 10, true),
                1_000,
            ))
            .unwrap();
        ledger
    }

    fn warning_substrate() -> SubstrateLedger {
        let mut ledger = SubstrateLedger::new();
        ledger
            .register_account(SubstrateAccount::new(
                SubstrateDimension::Financial,
                "sap",
                100,
                55,
                SubstrateBoundary::minimum(50, 10, true),
                1_000,
            ))
            .unwrap();
        ledger
    }

    #[test]
    fn clean_completed_action_is_ready_and_accepts_historical_execution_evidence() {
        let implementation = implementation_lifecycle();
        let completed = completed_lifecycle();
        let delivery_constraint = delivery_constraint(&completed);
        let completion_constraint = completion_constraint(&completed);

        let mut delivery_reconciliations = EconomicExecutionReconciliationLedger::new();
        let delivery_receipt = delivery_receipt(&implementation);
        delivery_reconciliations
            .reconcile(
                &implementation,
                &delivery_receipt,
                &delivery_constraint,
                "reconciliation:delivery",
                vec!["evidence:reconciled-delivery".into()],
                1_300,
            )
            .unwrap();
        let delivery_reconciliation = delivery_reconciliations.reconciliations()[0].clone();

        let mut completion_reconciliations = EconomicExecutionReconciliationLedger::new();
        let completion_receipt = completion_receipt(&completed);
        completion_reconciliations
            .reconcile(
                &completed,
                &completion_receipt,
                &completion_constraint,
                "reconciliation:completion",
                vec!["evidence:reconciled-completion".into()],
                1_600,
            )
            .unwrap();
        let completion_reconciliation = completion_reconciliations.reconciliations()[0].clone();

        let reconciliations =
            reconciliation_ledger(vec![delivery_reconciliation, completion_reconciliation]);

        let assessment = EconomicActionFinalizationGate::assess(
            &completed,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &reconciliations,
            &[delivery_constraint, completion_constraint],
        )
        .unwrap();

        assert_eq!(assessment.decision, EconomicFinalizationDecision::Ready);
        assert!(assessment.is_ready());
        assert!(assessment.completion_constraint_present);
        assert!(assessment.missing_constraint_ids.is_empty());
        assert!(assessment.nonconformant_reconciliation_ids.is_empty());
    }

    #[test]
    fn missing_completion_constraint_blocks_finalization() {
        let lifecycle = completed_lifecycle();
        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &EconomicExecutionReconciliationLedger::new(),
            &[],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByMissingConstraint
        );
        assert!(!assessment.completion_constraint_present);
        assert!(assessment.missing_constraint_ids.is_empty());
    }

    #[test]
    fn missing_reconciliation_blocks_finalization() {
        let lifecycle = completed_lifecycle();
        let constraint = completion_constraint(&lifecycle);
        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &EconomicExecutionReconciliationLedger::new(),
            &[constraint],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByMissingReconciliation
        );
        assert_eq!(
            assessment.missing_constraint_ids,
            vec!["constraint:completion"]
        );
    }

    #[test]
    fn nonconformance_cannot_be_hidden_by_a_later_conformant_reconciliation() {
        let lifecycle = completed_lifecycle();
        let constraint = completion_constraint(&lifecycle);
        let reconciliations = reconciliation_ledger(vec![
            EconomicExecutionReconciliation {
                reconciliation_id: "reconciliation:bad".into(),
                execution_id: "execution:bad".into(),
                constraint_id: constraint.constraint_id.clone(),
                result: ExecutionConformance::OverQuantity,
                evidence_refs: vec!["evidence:bad".into()],
                recorded_at: 1_500,
            },
            EconomicExecutionReconciliation {
                reconciliation_id: "reconciliation:good".into(),
                execution_id: "execution:good".into(),
                constraint_id: constraint.constraint_id.clone(),
                result: ExecutionConformance::Conformant,
                evidence_refs: vec!["evidence:good".into()],
                recorded_at: 1_600,
            },
        ]);

        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &reconciliations,
            &[constraint],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByNonConformance
        );
        assert_eq!(
            assessment.nonconformant_reconciliation_ids,
            vec!["reconciliation:bad"]
        );
    }

    #[test]
    fn unresolved_impacts_block_even_when_execution_controls_are_clean() {
        let lifecycle = completed_lifecycle();
        let constraint = completion_constraint(&lifecycle);
        let reconciliations = reconciliation_ledger(vec![EconomicExecutionReconciliation {
            reconciliation_id: "reconciliation:completion".into(),
            execution_id: "execution:completion".into(),
            constraint_id: constraint.constraint_id.clone(),
            result: ExecutionConformance::Conformant,
            evidence_refs: vec!["evidence:completion".into()],
            recorded_at: 1_600,
        }]);

        let mut impacts = ImpactLedger::new();
        impacts
            .record_impact(SubstrateImpact {
                id: "impact:open".into(),
                action_actor: "actor:dao".into(),
                action_ref: "action:1".into(),
                dimension: SubstrateDimension::Financial,
                unit: "sap".into(),
                magnitude: 10,
                direction: ImpactDirection::Depletion,
                affected_ref: "commons:1".into(),
                attributions: Vec::new(),
                evidence_refs: vec!["evidence:impact".into()],
                status: ImpactStatus::Open,
                obligation_id: None,
                recorded_at: 1_300,
            })
            .unwrap();

        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &impacts,
            &reconciliations,
            &[constraint],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByUnresolvedImpact
        );
        assert_eq!(assessment.open_impact_ids, vec!["impact:open"]);
    }

    #[test]
    fn integrity_warning_blocks_clean_finalization() {
        let lifecycle = completed_lifecycle();
        let constraint = completion_constraint(&lifecycle);
        let reconciliations = reconciliation_ledger(vec![EconomicExecutionReconciliation {
            reconciliation_id: "reconciliation:completion".into(),
            execution_id: "execution:completion".into(),
            constraint_id: constraint.constraint_id.clone(),
            result: ExecutionConformance::Conformant,
            evidence_refs: vec!["evidence:completion".into()],
            recorded_at: 1_600,
        }]);

        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &warning_substrate(),
            &ImpactLedger::new(),
            &reconciliations,
            &[constraint],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByIntegrity
        );
    }

    #[test]
    fn non_completed_lifecycle_cannot_finalize() {
        let lifecycle = implementation_lifecycle();
        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &EconomicExecutionReconciliationLedger::new(),
            &[],
        )
        .unwrap();

        assert_eq!(
            assessment.decision,
            EconomicFinalizationDecision::BlockedByLifecycle
        );
    }

    #[test]
    fn completion_constraint_must_bind_to_current_completed_revision() {
        let lifecycle = completed_lifecycle();
        let mut constraint = completion_constraint(&lifecycle);
        constraint.lifecycle_revision_id = lifecycle.revisions()[2].revision_id.clone();

        assert!(EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope(),
            &healthy_substrate(),
            &ImpactLedger::new(),
            &EconomicExecutionReconciliationLedger::new(),
            &[constraint],
        )
        .is_err());
    }
}
