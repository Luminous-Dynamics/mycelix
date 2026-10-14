// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Execution Receipt Binding
//!
//! Immutable execution receipts bound to the current economic action lifecycle.
//!
//! AC-039 closes the authorization-to-execution gap: a payment, delivery,
//! milestone, or completion record must identify the lifecycle revision and
//! exact scope fingerprint under which it was authorized.
//!
//! This is a reference model. It does not prescribe financial settlement,
//! procurement law, or a universal transaction schema.

use super::action_lifecycle::{
    EconomicActionLifecycle, EconomicActionStage,
};
use serde::{Deserialize, Serialize};

/// Type of execution evidence attached to an economic action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EconomicExecutionKind {
    /// Monetary payment or settlement event.
    Payment,
    /// Physical/service delivery event.
    Delivery,
    /// Contractual or operational milestone.
    Milestone,
    /// Evidence that the action reached normal completion.
    Completion,
}

/// Immutable execution receipt.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicExecutionReceipt {
    /// Stable receipt identifier.
    pub execution_id: String,
    /// Stable action identity.
    pub action_ref: String,
    /// Exact lifecycle revision authorizing this execution.
    pub lifecycle_revision_id: String,
    /// Scope identifier active at authorization time.
    pub scope_id: String,
    /// SHA-256 fingerprint of the scope contents.
    pub scope_fingerprint: String,
    /// Execution semantic.
    pub kind: EconomicExecutionKind,
    /// Optional quantity in the domain-native unit.
    pub quantity: Option<i128>,
    /// Native quantity unit when quantity is present.
    pub quantity_unit: Option<String>,
    /// External transaction/document reference.
    pub external_ref: String,
    /// Evidence supporting the execution record.
    pub evidence_refs: Vec<String>,
    /// Execution timestamp.
    pub recorded_at: u64,
}

impl EconomicExecutionReceipt {
    /// Validate receipt structure.
    pub fn validate(&self) -> Result<(), String> {
        if self.execution_id.trim().is_empty() {
            return Err("Execution ID cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Execution action reference cannot be empty".into());
        }
        if self.lifecycle_revision_id.trim().is_empty() {
            return Err("Execution lifecycle revision ID cannot be empty".into());
        }
        if self.scope_id.trim().is_empty() {
            return Err("Execution scope ID cannot be empty".into());
        }
        if self.scope_fingerprint.len() != 64
            || !self.scope_fingerprint
                .as_bytes()
                .iter()
                .all(u8::is_ascii_hexdigit)
        {
            return Err(
                "Execution scope fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self.external_ref.trim().is_empty() {
            return Err("Execution external reference cannot be empty".into());
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Execution evidence references cannot be empty".into());
        }
        match (self.quantity, &self.quantity_unit) {
            (Some(_), Some(unit)) if !unit.trim().is_empty() => {}
            (Some(_), _) => {
                return Err("Execution quantity requires a non-empty quantity unit".into());
            }
            (None, Some(_)) => {
                return Err("Execution quantity unit requires a quantity".into());
            }
            (None, None) => {}
        }
        Ok(())
    }
}

/// Append-only execution ledger for one economic action.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicExecutionLedger {
    action_ref: String,
    receipts: Vec<EconomicExecutionReceipt>,
}

impl EconomicExecutionLedger {
    /// Create an execution ledger for an action.
    pub fn new(action_ref: impl Into<String>) -> Result<Self, String> {
        let action_ref = action_ref.into();
        if action_ref.trim().is_empty() {
            return Err("Execution ledger action reference cannot be empty".into());
        }
        Ok(Self {
            action_ref,
            receipts: Vec::new(),
        })
    }

    /// Read stable action identity.
    pub fn action_ref(&self) -> &str {
        &self.action_ref
    }

    /// Read immutable execution history.
    pub fn receipts(&self) -> &[EconomicExecutionReceipt] {
        &self.receipts
    }

    /// Attach a receipt to the exact current lifecycle authorization.
    ///
    /// The receipt must reference the current revision and current scope
    /// fingerprint. This prevents execution from silently using stale policy.
    pub fn record(
        &mut self,
        lifecycle: &EconomicActionLifecycle,
        receipt: EconomicExecutionReceipt,
    ) -> Result<(), String> {
        lifecycle.validate()?;
        receipt.validate()?;

        if receipt.action_ref != self.action_ref || receipt.action_ref != lifecycle.action_ref() {
            return Err("Execution action reference does not match lifecycle".into());
        }
        if self
            .receipts
            .iter()
            .any(|existing| existing.execution_id == receipt.execution_id)
        {
            return Err(format!(
                "Duplicate execution ID: {}",
                receipt.execution_id
            ));
        }
        if receipt.lifecycle_revision_id != lifecycle.current_revision()?.revision_id {
            return Err("Execution receipt does not reference the current lifecycle revision".into());
        }
        if receipt.scope_id != lifecycle.active_scope_id() {
            return Err("Execution receipt does not reference the active lifecycle scope".into());
        }
        if receipt.scope_fingerprint != lifecycle.active_scope_fingerprint() {
            return Err("Execution receipt scope fingerprint does not match active scope".into());
        }
        if receipt.recorded_at < lifecycle.current_revision()?.recorded_at {
            return Err("Execution timestamp cannot precede lifecycle authorization".into());
        }

        match receipt.kind {
            EconomicExecutionKind::Payment
            | EconomicExecutionKind::Delivery
            | EconomicExecutionKind::Milestone => {
                if !matches!(
                    lifecycle.current_stage(),
                    EconomicActionStage::Contracted | EconomicActionStage::Implementation
                ) {
                    return Err(
                        "Execution evidence requires Contracted or Implementation lifecycle stage"
                            .into(),
                    );
                }
            }
            EconomicExecutionKind::Completion => {
                if lifecycle.current_stage() != EconomicActionStage::Completed {
                    return Err("Completion evidence requires Completed lifecycle stage".into());
                }
            }
        }

        self.receipts.push(receipt);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::action_lifecycle::{
        EconomicActionChangeKind, EconomicActionLifecycle, EconomicActionScope,
    };
    use crate::economics::substrate::{DistributionPurpose, SubstrateDimension};

    fn scope(action_ref: &str, scope_id: &str) -> EconomicActionScope {
        EconomicActionScope {
            scope_id: scope_id.into(),
            action_ref: action_ref.into(),
            purpose: DistributionPurpose::Discretionary,
            required_dimensions: vec![SubstrateDimension::Financial],
            policy_ref: "policy:execution:v1".into(),
            authority_ref: "authority:dao-1".into(),
            attestation_ref: format!("attestation:{scope_id}"),
            evidence_refs: vec![format!("evidence:{scope_id}")],
            declared_at: 1_000,
        }
    }

    fn lifecycle() -> EconomicActionLifecycle {
        EconomicActionLifecycle::start(
            &scope("action:1", "scope:1"),
            "revision:1",
            "authority:dao-1",
            vec!["evidence:planning".into()],
            1_000,
        )
        .unwrap()
    }

    fn receipt(
        lifecycle: &EconomicActionLifecycle,
        kind: EconomicExecutionKind,
    ) -> EconomicExecutionReceipt {
        EconomicExecutionReceipt {
            execution_id: format!("execution:{kind:?}"),
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: lifecycle.current_revision().unwrap().revision_id.clone(),
            scope_id: lifecycle.active_scope_id().into(),
            scope_fingerprint: lifecycle.active_scope_fingerprint().into(),
            kind,
            quantity: Some(1),
            quantity_unit: Some("unit".into()),
            external_ref: "external:1".into(),
            evidence_refs: vec!["evidence:execution".into()],
            recorded_at: 1_200,
        }
    }

    #[test]
    fn execution_requires_contract_or_implementation_stage() {
        let lifecycle = lifecycle();
        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();

        assert!(ledger.record(&lifecycle, receipt(&lifecycle, EconomicExecutionKind::Payment)).is_err());
    }

    #[test]
    fn execution_binds_to_current_revision_and_scope() {
        let mut lifecycle = lifecycle();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();

        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();
        ledger
            .record(&lifecycle, receipt(&lifecycle, EconomicExecutionKind::Payment))
            .unwrap();

        assert_eq!(ledger.receipts().len(), 1);
    }

    #[test]
    fn stale_revision_cannot_authorize_execution() {
        let mut lifecycle = lifecycle();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();

        let mut receipt = receipt(&lifecycle, EconomicExecutionKind::Payment);
        receipt.lifecycle_revision_id = "revision:1".into();

        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();
        assert!(ledger.record(&lifecycle, receipt).is_err());
    }

    #[test]
    fn same_scope_id_with_changed_contents_cannot_authorize_execution() {
        let mut lifecycle = lifecycle();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();

        let mut altered = scope("action:1", "scope:1");
        altered.authority_ref = "authority:changed".into();

        let mut receipt = receipt(&lifecycle, EconomicExecutionKind::Payment);
        receipt.scope_fingerprint = altered.fingerprint().unwrap();

        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();
        assert!(ledger.record(&lifecycle, receipt).is_err());
    }

    #[test]
    fn completion_receipt_requires_completed_lifecycle() {
        let lifecycle = lifecycle();
        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();

        assert!(ledger.record(&lifecycle, receipt(&lifecycle, EconomicExecutionKind::Completion)).is_err());
    }

    #[test]
    fn duplicate_execution_ids_are_rejected() {
        let mut lifecycle = lifecycle();
        lifecycle
            .record(
                "revision:2",
                EconomicActionStage::Contracted,
                EconomicActionChangeKind::Update,
                &scope("action:1", "scope:1"),
                "authority:dao-1",
                vec!["evidence:contract".into()],
                1_100,
            )
            .unwrap();

        let mut ledger = EconomicExecutionLedger::new("action:1").unwrap();
        let first = receipt(&lifecycle, EconomicExecutionKind::Payment);
        let second = first.clone();

        ledger.record(&lifecycle, first).unwrap();
        assert!(ledger.record(&lifecycle, second).is_err());
    }
}
