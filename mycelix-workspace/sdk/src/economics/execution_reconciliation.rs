// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Execution Conformance Reconciliation
//!
//! AC-041 compares an immutable execution receipt against an explicit,
//! lifecycle-bound execution constraint.
//!
//! The module deliberately does not invent tolerances, price semantics, or
//! domain-specific notions of acceptable variance. An exact-match constraint
//! is the reference primitive; any richer tolerance policy must be explicit
//! and governed by the integrating system.

use super::{
    action_lifecycle::EconomicActionLifecycle,
    execution_receipt::{EconomicExecutionKind, EconomicExecutionReceipt},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Explicit expected execution constraint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicExecutionConstraint {
    /// Stable constraint identifier.
    pub constraint_id: String,
    /// Stable action identity.
    pub action_ref: String,
    /// Exact lifecycle revision under which this execution is expected.
    pub lifecycle_revision_id: String,
    /// Scope identity bound to the constraint.
    pub scope_id: String,
    /// Scope content fingerprint.
    pub scope_fingerprint: String,
    /// Expected execution kind.
    pub kind: EconomicExecutionKind,
    /// Expected exact quantity.
    pub quantity: i128,
    /// Required native unit for the quantity.
    pub quantity_unit: String,
    /// Evidence supporting the expected execution.
    pub evidence_refs: Vec<String>,
}

impl EconomicExecutionConstraint {
    /// Validate the constraint envelope.
    pub fn validate(&self) -> Result<(), String> {
        if self.constraint_id.trim().is_empty() {
            return Err("Execution constraint ID cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Execution constraint action reference cannot be empty".into());
        }
        if self.lifecycle_revision_id.trim().is_empty() {
            return Err("Execution constraint lifecycle revision ID cannot be empty".into());
        }
        if self.scope_id.trim().is_empty() {
            return Err("Execution constraint scope ID cannot be empty".into());
        }
        if self.scope_fingerprint.len() != 64
            || !self
                .scope_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Execution constraint scope fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self.quantity_unit.trim().is_empty() {
            return Err("Execution constraint quantity unit cannot be empty".into());
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Execution constraint evidence references cannot be empty".into());
        }
        Ok(())
    }

    /// Return a deterministic SHA-256 fingerprint of the complete constraint.
    ///
    /// The digest binds the exact execution expectation observed during
    /// reconciliation, not merely its stable identifier.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;
        let canonical = serde_json::to_vec(self)
            .map_err(|error| format!("Execution constraint canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-EXECUTION-CONSTRAINT-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

/// Exact conformance result between an execution receipt and a constraint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionConformance {
    /// Receipt matches the authorized constraint exactly.
    Conformant,
    /// Receipt was smaller than the exact expected quantity.
    UnderQuantity,
    /// Receipt exceeded the exact expected quantity.
    OverQuantity,
    /// Receipt and constraint use different quantity units.
    UnitMismatch,
    /// Receipt kind differs from the constraint.
    KindMismatch,
    /// Receipt does not carry an observed quantity.
    MissingObservedQuantity,
    /// Receipt authorization context does not match the constraint.
    AuthorizationMismatch,
}

/// Immutable reconciliation record.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicExecutionReconciliation {
    /// Stable reconciliation identifier.
    pub reconciliation_id: String,
    /// Receipt being reconciled.
    pub execution_id: String,
    /// SHA-256 fingerprint of the exact receipt observed.
    pub execution_fingerprint: String,
    /// Constraint being applied.
    pub constraint_id: String,
    /// SHA-256 fingerprint of the exact constraint observed.
    pub constraint_fingerprint: String,
    /// Deterministic conformance result.
    pub result: ExecutionConformance,
    /// Reconciliation evidence references.
    pub evidence_refs: Vec<String>,
    /// Reconciliation timestamp.
    pub recorded_at: u64,
}

impl EconomicExecutionReconciliation {
    /// Validate reconciliation record structure.
    pub fn validate(&self) -> Result<(), String> {
        if self.reconciliation_id.trim().is_empty() {
            return Err("Reconciliation ID cannot be empty".into());
        }
        if self.execution_id.trim().is_empty() {
            return Err("Reconciliation execution ID cannot be empty".into());
        }
        if self.constraint_id.trim().is_empty() {
            return Err("Reconciliation constraint ID cannot be empty".into());
        }
        if self.execution_fingerprint.len() != 64
            || !self
                .execution_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Reconciliation execution fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self.constraint_fingerprint.len() != 64
            || !self
                .constraint_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Reconciliation constraint fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Reconciliation evidence references cannot be empty".into());
        }
        Ok(())
    }
}

/// Append-only reconciliation ledger.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicExecutionReconciliationLedger {
    reconciliations: Vec<EconomicExecutionReconciliation>,
}

impl EconomicExecutionReconciliationLedger {
    /// Create an empty reconciliation ledger.
    pub fn new() -> Self {
        Self::default()
    }

    /// Read immutable reconciliation history.
    pub fn reconciliations(&self) -> &[EconomicExecutionReconciliation] {
        &self.reconciliations
    }

    /// Validate persisted reconciliation history without mutating it.
    pub fn validate(&self) -> Result<(), String> {
        let mut seen = std::collections::BTreeSet::new();
        let mut previous_timestamp = None;

        for reconciliation in &self.reconciliations {
            reconciliation.validate()?;

            if !seen.insert(reconciliation.reconciliation_id.clone()) {
                return Err(format!(
                    "Duplicate reconciliation ID in history: {}",
                    reconciliation.reconciliation_id
                ));
            }

            if let Some(previous) = previous_timestamp {
                if reconciliation.recorded_at < previous {
                    return Err("Reconciliation history timestamps cannot move backwards".into());
                }
            }
            previous_timestamp = Some(reconciliation.recorded_at);
        }

        Ok(())
    }



    /// Compare a receipt against an explicit constraint and persist the result.
    ///
    /// A non-conformant result is retained as evidence rather than rewritten or
    /// silently converted into approval.
    pub fn reconcile(
        &mut self,
        lifecycle: &EconomicActionLifecycle,
        receipt: &EconomicExecutionReceipt,
        constraint: &EconomicExecutionConstraint,
        reconciliation_id: impl Into<String>,
        evidence_refs: Vec<String>,
        recorded_at: u64,
    ) -> Result<ExecutionConformance, String> {
        lifecycle.validate()?;
        receipt.validate()?;
        constraint.validate()?;
        self.validate()?;

        let reconciliation_id = reconciliation_id.into();
        if self
            .reconciliations
            .iter()
            .any(|record| record.reconciliation_id == reconciliation_id)
        {
            return Err("Duplicate reconciliation ID".into());
        }

        if recorded_at < receipt.recorded_at {
            return Err("Reconciliation timestamp cannot precede execution".into());
        }
        if let Some(previous) = self.reconciliations.last() {
            if recorded_at < previous.recorded_at {
                return Err("Reconciliation timestamps cannot move backwards".into());
            }
        }

        if receipt.action_ref != constraint.action_ref
            || receipt.action_ref != lifecycle.action_ref()
        {
            return Err("Execution reconciliation action identity mismatch".into());
        }

        if constraint.lifecycle_revision_id != lifecycle.current_revision()?.revision_id
            || constraint.scope_id != lifecycle.active_scope_id()
            || constraint.scope_fingerprint != lifecycle.active_scope_fingerprint()
        {
            return Err("Execution constraint is not bound to the current lifecycle authorization".into());
        }

        if receipt.lifecycle_revision_id != constraint.lifecycle_revision_id
            || receipt.scope_id != constraint.scope_id
            || receipt.scope_fingerprint != constraint.scope_fingerprint
        {
            return Err("Execution receipt authorization does not match constraint".into());
        }

        let execution_fingerprint = receipt.fingerprint()?;
        let constraint_fingerprint = constraint.fingerprint()?;

        let result = if receipt.kind != constraint.kind {
            ExecutionConformance::KindMismatch
        } else {
            match (receipt.quantity, receipt.quantity_unit.as_deref()) {
                (None, _) => ExecutionConformance::MissingObservedQuantity,
                (Some(_), None) => ExecutionConformance::UnitMismatch,
                (Some(actual), Some(unit)) if unit != constraint.quantity_unit => {
                    ExecutionConformance::UnitMismatch
                }
                (Some(actual), Some(_)) if actual < constraint.quantity => {
                    ExecutionConformance::UnderQuantity
                }
                (Some(actual), Some(_)) if actual > constraint.quantity => {
                    ExecutionConformance::OverQuantity
                }
                (Some(_), Some(_)) => ExecutionConformance::Conformant,
            }
        };

        let reconciliation = EconomicExecutionReconciliation {
            reconciliation_id,
            execution_id: receipt.execution_id.clone(),
            execution_fingerprint,
            constraint_id: constraint.constraint_id.clone(),
            constraint_fingerprint,
            result,
            evidence_refs,
            recorded_at,
        };
        reconciliation.validate()?;
        self.reconciliations.push(reconciliation);

        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::{
        action_lifecycle::{EconomicActionChangeKind, EconomicActionScope, EconomicActionStage},
        substrate::{DistributionPurpose, SubstrateDimension},
    };

    fn scope() -> EconomicActionScope {
        EconomicActionScope {
            scope_id: "scope:1".into(),
            action_ref: "action:1".into(),
            purpose: DistributionPurpose::Discretionary,
            required_dimensions: vec![SubstrateDimension::Financial],
            policy_ref: "policy:execution:v1".into(),
            authority_ref: "authority:dao-1".into(),
            attestation_ref: "attestation:scope:1".into(),
            evidence_refs: vec!["evidence:scope".into()],
            declared_at: 1_000,
        }
    }

    fn lifecycle() -> EconomicActionLifecycle {
        EconomicActionLifecycle::start(
            &scope(),
            "revision:1",
            "authority:dao-1",
            vec!["evidence:planning".into()],
            1_000,
        )
        .unwrap()
    }

    fn contracted_lifecycle() -> EconomicActionLifecycle {
        let mut lifecycle = lifecycle();
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
    }

    fn receipt(lifecycle: &EconomicActionLifecycle, quantity: Option<i128>) -> EconomicExecutionReceipt {
        EconomicExecutionReceipt {
            execution_id: "execution:1".into(),
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: lifecycle.current_revision().unwrap().revision_id.clone(),
            scope_id: lifecycle.active_scope_id().into(),
            scope_fingerprint: lifecycle.active_scope_fingerprint().into(),
            kind: EconomicExecutionKind::Delivery,
            quantity,
            quantity_unit: quantity.map(|_| "unit".into()),
            external_ref: "external:1".into(),
            evidence_refs: vec!["evidence:execution".into()],
            recorded_at: 1_200,
        }
    }

    fn constraint(lifecycle: &EconomicActionLifecycle, quantity: i128) -> EconomicExecutionConstraint {
        EconomicExecutionConstraint {
            constraint_id: "constraint:1".into(),
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: lifecycle.current_revision().unwrap().revision_id.clone(),
            scope_id: lifecycle.active_scope_id().into(),
            scope_fingerprint: lifecycle.active_scope_fingerprint().into(),
            kind: EconomicExecutionKind::Delivery,
            quantity,
            quantity_unit: "unit".into(),
            evidence_refs: vec!["evidence:contract-quantity".into()],
        }
    }

    #[test]
    fn exact_quantity_is_conformant() {
        let lifecycle = contracted_lifecycle();
        let receipt = receipt(&lifecycle, Some(100));
        let constraint = constraint(&lifecycle, 100);
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        assert_eq!(
            ledger.reconcile(
                &lifecycle,
                &receipt,
                &constraint,
                "reconciliation:1",
                vec!["evidence:reconciliation".into()],
                1_300,
            ),
            Ok(ExecutionConformance::Conformant)
        );
    }

    #[test]
    fn under_and_over_execution_are_visible() {
        let lifecycle = contracted_lifecycle();
        let constraint = constraint(&lifecycle, 100);
        let mut under = EconomicExecutionReconciliationLedger::new();
        let mut over = EconomicExecutionReconciliationLedger::new();

        assert_eq!(
            under.reconcile(
                &lifecycle,
                &receipt(&lifecycle, Some(90)),
                &constraint,
                "reconciliation:under",
                vec!["evidence:under".into()],
                1_300,
            ),
            Ok(ExecutionConformance::UnderQuantity)
        );

        assert_eq!(
            over.reconcile(
                &lifecycle,
                &receipt(&lifecycle, Some(110)),
                &constraint,
                "reconciliation:over",
                vec!["evidence:over".into()],
                1_300,
            ),
            Ok(ExecutionConformance::OverQuantity)
        );
    }

    #[test]
    fn missing_quantity_is_not_treated_as_conformant() {
        let lifecycle = contracted_lifecycle();
        let constraint = constraint(&lifecycle, 100);
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        assert_eq!(
            ledger.reconcile(
                &lifecycle,
                &receipt(&lifecycle, None),
                &constraint,
                "reconciliation:missing",
                vec!["evidence:missing".into()],
                1_300,
            ),
            Ok(ExecutionConformance::MissingObservedQuantity)
        );
    }

    #[test]
    fn wrong_unit_is_not_treated_as_conformant() {
        let lifecycle = contracted_lifecycle();
        let constraint = constraint(&lifecycle, 100);
        let mut receipt = receipt(&lifecycle, Some(100));
        receipt.quantity_unit = Some("wrong-unit".into());
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        assert_eq!(
            ledger.reconcile(
                &lifecycle,
                &receipt,
                &constraint,
                "reconciliation:unit",
                vec!["evidence:unit".into()],
                1_300,
            ),
            Ok(ExecutionConformance::UnitMismatch)
        );
    }

    #[test]
    fn stale_lifecycle_constraint_is_rejected() {
        let mut lifecycle = contracted_lifecycle();
        let old_constraint = constraint(&lifecycle, 100);
        lifecycle
            .record(
                "revision:3",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope(),
                "authority:dao-1",
                vec!["evidence:new".into()],
                1_400,
            )
            .unwrap();

        let receipt = receipt(&lifecycle, Some(100));
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        assert!(ledger
            .reconcile(
                &lifecycle,
                &receipt,
                &old_constraint,
                "reconciliation:stale",
                vec!["evidence:stale".into()],
                1_500,
            )
            .is_err());
    }

    #[test]
    fn malformed_persisted_reconciliation_history_is_rejected() {
        let mut ledger = EconomicExecutionReconciliationLedger::new();
        ledger.reconciliations.push(EconomicExecutionReconciliation {
            reconciliation_id: "reconciliation:one".into(),
            execution_id: "execution:one".into(),
            constraint_id: "constraint:one".into(),
            result: ExecutionConformance::Conformant,
            evidence_refs: vec!["evidence:one".into()],
            recorded_at: 1_300,
        });
        ledger.reconciliations.push(EconomicExecutionReconciliation {
            reconciliation_id: "reconciliation:one".into(),
            execution_id: "execution:two".into(),
            constraint_id: "constraint:two".into(),
            result: ExecutionConformance::Conformant,
            evidence_refs: vec!["evidence:two".into()],
            recorded_at: 1_400,
        });

        assert!(ledger.validate().is_err());
    }

    #[test]
    fn reconciliation_cannot_precede_execution() {
        let lifecycle = contracted_lifecycle();
        let receipt = receipt(&lifecycle, Some(100));
        let constraint = constraint(&lifecycle, 100);
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        let result = ledger.reconcile(
            &lifecycle,
            &receipt,
            &constraint,
            "reconciliation:temporal",
            vec!["evidence:temporal".into()],
            1_199,
        );

        assert!(result.is_err());
    }

    #[test]
    fn reconciliation_history_is_monotonic() {
        let lifecycle = contracted_lifecycle();
        let receipt = receipt(&lifecycle, Some(100));
        let constraint = constraint(&lifecycle, 100);
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        ledger
            .reconcile(
                &lifecycle,
                &receipt,
                &constraint,
                "reconciliation:first",
                vec!["evidence:first".into()],
                1_300,
            )
            .unwrap();

        let result = ledger.reconcile(
            &lifecycle,
            &receipt,
            &constraint,
            "reconciliation:second",
            vec!["evidence:second".into()],
            1_299,
        );

        assert!(result.is_err());
        assert_eq!(ledger.reconciliations().len(), 1);
    }

    #[test]
    fn duplicate_reconciliation_ids_are_rejected() {
        let lifecycle = contracted_lifecycle();
        let receipt = receipt(&lifecycle, Some(100));
        let constraint = constraint(&lifecycle, 100);
        let mut ledger = EconomicExecutionReconciliationLedger::new();

        ledger
            .reconcile(
                &lifecycle,
                &receipt,
                &constraint,
                "reconciliation:1",
                vec!["evidence:one".into()],
                1_300,
            )
            .unwrap();

        assert!(ledger
            .reconcile(
                &lifecycle,
                &receipt,
                &constraint,
                "reconciliation:1",
                vec!["evidence:two".into()],
                1_301,
            )
            .is_err());
    }
}
