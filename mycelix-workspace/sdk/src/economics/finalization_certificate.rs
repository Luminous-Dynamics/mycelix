// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Economic Finalization Certificate Ledger
//!
//! AC-049 turns a fresh Ready assessment into a durable, terminal,
//! append-only finalization certificate.
//!
//! The certificate is separate from lifecycle mutation. It does not rewrite
//! or erase lifecycle, execution, reconciliation, substrate, or impact evidence.
//! It records the exact assessment snapshot that justified finalization and
//! can later be re-verified against current evidence.

use super::{
    action_finalization::{EconomicActionFinalizationGate, EconomicFinalizationAssessment},
    action_lifecycle::EconomicActionLifecycle,
    execution_reconciliation::{
        EconomicExecutionConstraint, EconomicExecutionReconciliationLedger,
    },
    execution_receipt::EconomicExecutionLedger,
    impact::ImpactLedger,
    integrity_gate::EconomicActionScope,
    substrate::SubstrateLedger,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Durable proof that one economic action was finalized from a fresh Ready
/// assessment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicFinalizationCertificate {
    /// Unique certificate identifier.
    pub certificate_id: String,
    /// Stable economic action identity.
    pub action_ref: String,
    /// Completed lifecycle revision used for finalization.
    pub lifecycle_revision_id: String,
    /// Active scope identity.
    pub scope_id: String,
    /// Active scope content fingerprint.
    pub scope_fingerprint: String,
    /// Exact evidence snapshot fingerprint from AC-048.
    pub evidence_snapshot_fingerprint: String,
    /// Content fingerprint of the certificate payload itself.
    pub certificate_fingerprint: String,
    /// Authority reference that issued the certificate.
    pub authority_ref: String,
    /// Supporting issuance evidence.
    pub evidence_refs: Vec<String>,
    /// Certificate issuance timestamp.
    pub finalized_at: u64,
}

impl EconomicFinalizationCertificate {
    fn validate_fields(&self, require_fingerprint: bool) -> Result<(), String> {
        if self.certificate_id.trim().is_empty() {
            return Err("Finalization certificate ID cannot be empty".into());
        }
        if self.action_ref.trim().is_empty() {
            return Err("Finalization certificate action reference cannot be empty".into());
        }
        if self.lifecycle_revision_id.trim().is_empty() {
            return Err("Finalization certificate lifecycle revision ID cannot be empty".into());
        }
        if self.scope_id.trim().is_empty() {
            return Err("Finalization certificate scope ID cannot be empty".into());
        }
        for (name, value) in [
            ("scope fingerprint", self.scope_fingerprint.as_str()),
            (
                "evidence snapshot fingerprint",
                self.evidence_snapshot_fingerprint.as_str(),
            ),
        ] {
            if value.len() != 64 || !value.as_bytes().iter().all(u8::is_ascii_hexdigit) {
                return Err(format!(
                    "Finalization certificate {name} must be a 64-character hexadecimal SHA-256"
                ));
            }
        }
        if require_fingerprint
            && (self.certificate_fingerprint.len() != 64
                || !self
                    .certificate_fingerprint
                    .as_bytes()
                    .iter()
                    .all(u8::is_ascii_hexdigit))
        {
            return Err(
                "Finalization certificate fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        if self.authority_ref.trim().is_empty() {
            return Err("Finalization certificate authority reference cannot be empty".into());
        }
        if self
            .evidence_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
        {
            return Err("Finalization certificate evidence references cannot be empty".into());
        }
        Ok(())
    }

    /// Return a deterministic SHA-256 fingerprint of the certificate payload.
    ///
    /// The fingerprint intentionally excludes itself. It is a tamper-evident
    /// content identifier, not a proof of authorship.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate_fields(false)?;
        let payload = serde_json::json!({
            "version": 1,
            "certificate_id": self.certificate_id,
            "action_ref": self.action_ref,
            "lifecycle_revision_id": self.lifecycle_revision_id,
            "scope_id": self.scope_id,
            "scope_fingerprint": self.scope_fingerprint,
            "evidence_snapshot_fingerprint": self.evidence_snapshot_fingerprint,
            "authority_ref": self.authority_ref,
            "evidence_refs": self.evidence_refs,
            "finalized_at": self.finalized_at,
        });
        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Finalization certificate canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-FINALIZATION-CERTIFICATE-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }

    /// Validate certificate structure and its self-integrity fingerprint.
    pub fn validate(&self) -> Result<(), String> {
        self.validate_fields(true)?;
        let expected = self.fingerprint()?;
        if expected != self.certificate_fingerprint {
            return Err("Finalization certificate fingerprint does not match its content".into());
        }
        Ok(())
    }
}

/// Append-only terminal finalization ledger for one economic action.
///
/// A clean finalization is intentionally single-shot. Once a certificate is
/// recorded, a second successful finalization for the same action is rejected
/// rather than creating competing terminal truths.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicFinalizationLedger {
    action_ref: String,
    certificates: Vec<EconomicFinalizationCertificate>,
}

impl EconomicFinalizationLedger {
    /// Create an empty terminal finalization ledger for an action.
    pub fn new(action_ref: impl Into<String>) -> Result<Self, String> {
        let action_ref = action_ref.into();
        if action_ref.trim().is_empty() {
            return Err("Finalization ledger action reference cannot be empty".into());
        }

        Ok(Self {
            action_ref,
            certificates: Vec::new(),
        })
    }

    /// Return stable action identity.
    pub fn action_ref(&self) -> &str {
        &self.action_ref
    }

    /// Return immutable finalization history.
    pub fn certificates(&self) -> &[EconomicFinalizationCertificate] {
        &self.certificates
    }

    /// Whether this action has already received a terminal finalization
    /// certificate.
    pub fn is_finalized(&self) -> bool {
        !self.certificates.is_empty()
    }

    /// Validate persisted certificate history without mutating it.
    pub fn validate(&self) -> Result<(), String> {
        if self.action_ref.trim().is_empty() {
            return Err("Finalization ledger action reference cannot be empty".into());
        }

        let mut seen = std::collections::BTreeSet::new();
        let mut previous_timestamp = None;

        for certificate in &self.certificates {
            certificate.validate()?;

            if certificate.action_ref != self.action_ref {
                return Err(format!(
                    "Finalization certificate {} action reference does not match ledger",
                    certificate.certificate_id
                ));
            }

            if !seen.insert(certificate.certificate_id.clone()) {
                return Err(format!(
                    "Duplicate finalization certificate ID in history: {}",
                    certificate.certificate_id
                ));
            }

            if let Some(previous) = previous_timestamp {
                if certificate.finalized_at < previous {
                    return Err("Finalization certificate timestamps cannot move backwards".into());
                }
            }
            previous_timestamp = Some(certificate.finalized_at);
        }

        if self.certificates.len() > 1 {
            return Err("Finalization ledger cannot contain multiple terminal certificates".into());
        }

        Ok(())
    }

    /// Issue exactly one terminal finalization certificate from a fresh Ready
    /// assessment.
    pub fn issue(
        &mut self,
        assessment: &EconomicFinalizationAssessment,
        lifecycle: &EconomicActionLifecycle,
        scope: &EconomicActionScope,
        substrate: &SubstrateLedger,
        impacts: &ImpactLedger,
        reconciliations: &EconomicExecutionReconciliationLedger,
        execution: &EconomicExecutionLedger,
        constraints: &[EconomicExecutionConstraint],
        certificate_id: impl Into<String>,
        authority_ref: impl Into<String>,
        evidence_refs: Vec<String>,
        finalized_at: u64,
    ) -> Result<EconomicFinalizationCertificate, String> {
        self.validate()?;

        if self.is_finalized() {
            return Err("Economic action is already finalized".into());
        }
        if assessment.action_ref != self.action_ref {
            return Err("Finalization assessment action reference does not match ledger".into());
        }
        if lifecycle.action_ref() != self.action_ref {
            return Err("Finalization lifecycle action reference does not match ledger".into());
        }

        if !assessment.verify_freshness(
            lifecycle,
            scope,
            substrate,
            impacts,
            reconciliations,
            execution,
            constraints,
        )? {
            return Err("Finalization assessment is stale or no longer equivalent to current evidence".into());
        }

        if !assessment.is_ready() {
            return Err(format!(
                "Finalization requires Ready assessment; got {:?}",
                assessment.decision
            ));
        }

        let mut certificate = EconomicFinalizationCertificate {
            certificate_id: certificate_id.into(),
            action_ref: self.action_ref.clone(),
            lifecycle_revision_id: lifecycle.current_revision()?.revision_id.clone(),
            scope_id: scope.scope_id.clone(),
            scope_fingerprint: scope.fingerprint()?,
            evidence_snapshot_fingerprint: assessment.evidence_snapshot_fingerprint.clone(),
            certificate_fingerprint: "0".repeat(64),
            authority_ref: authority_ref.into(),
            evidence_refs,
            finalized_at,
        };

        certificate.certificate_fingerprint = certificate.fingerprint()?;
        certificate.validate()?;

        if finalized_at < lifecycle.current_revision()?.recorded_at {
            return Err("Finalization timestamp cannot precede lifecycle completion".into());
        }

        self.certificates.push(certificate.clone());
        Ok(certificate)
    }

    /// Verify that a stored certificate still corresponds to fresh current
    /// evidence.
    pub fn verify_certificate(
        &self,
        certificate: &EconomicFinalizationCertificate,
        lifecycle: &EconomicActionLifecycle,
        scope: &EconomicActionScope,
        substrate: &SubstrateLedger,
        impacts: &ImpactLedger,
        reconciliations: &EconomicExecutionReconciliationLedger,
        execution: &EconomicExecutionLedger,
        constraints: &[EconomicExecutionConstraint],
    ) -> Result<bool, String> {
        self.validate()?;
        certificate.validate()?;

        let stored = self
            .certificates
            .iter()
            .find(|stored| stored.certificate_id == certificate.certificate_id)
            .ok_or_else(|| {
                format!(
                    "Finalization certificate is not present in this ledger: {}",
                    certificate.certificate_id
                )
            })?;

        if stored != certificate {
            return Ok(false);
        }

        if certificate.finalized_at < lifecycle.current_revision()?.recorded_at {
            return Ok(false);
        }

        let assessment = EconomicActionFinalizationGate::assess(
            lifecycle,
            scope,
            substrate,
            impacts,
            reconciliations,
            execution,
            constraints,
        )?;

        Ok(
            self.action_ref == certificate.action_ref
                && certificate.action_ref == lifecycle.action_ref()
                && certificate.lifecycle_revision_id == assessment.lifecycle_revision_id
                && certificate.scope_id == assessment.scope_id
                && certificate.scope_fingerprint == assessment.scope_fingerprint
                && certificate.evidence_snapshot_fingerprint
                    == assessment.evidence_snapshot_fingerprint
                && assessment.is_ready(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::{
        action_finalization::EconomicFinalizationDecision,
        action_lifecycle::{EconomicActionChangeKind, EconomicActionScope, EconomicActionStage},
        execution_reconciliation::{
            EconomicExecutionReconciliation, ExecutionConformance,
        },
        execution_receipt::{EconomicExecutionKind, EconomicExecutionReceipt},
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

    fn completed_lifecycle() -> EconomicActionLifecycle {
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

    fn constraint(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionConstraint {
        let revision = lifecycle.current_revision().unwrap();
        EconomicExecutionConstraint {
            constraint_id: "constraint:completion".into(),
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Completion,
            quantity: 1,
            quantity_unit: "event".into(),
            evidence_refs: vec!["evidence:completion-constraint".into()],
        }
    }

    fn receipt(lifecycle: &EconomicActionLifecycle) -> EconomicExecutionReceipt {
        let revision = lifecycle.current_revision().unwrap();
        EconomicExecutionReceipt {
            execution_id: "execution:completion".into(),
            action_ref: lifecycle.action_ref().into(),
            lifecycle_revision_id: revision.revision_id.clone(),
            scope_id: revision.scope_id.clone(),
            scope_fingerprint: revision.scope_fingerprint.clone(),
            kind: EconomicExecutionKind::Completion,
            quantity: Some(1),
            quantity_unit: Some("event".into()),
            external_ref: "external:completion".into(),
            evidence_refs: vec!["evidence:execution".into()],
            recorded_at: 1_500,
        }
    }

    fn execution_ledger(receipt: EconomicExecutionReceipt) -> EconomicExecutionLedger {
        serde_json::from_value(serde_json::json!({
            "action_ref": "action:1",
            "receipts": [receipt]
        }))
        .unwrap()
    }

    fn reconciliation_ledger(
        receipt: &EconomicExecutionReceipt,
        constraint: &EconomicExecutionConstraint,
    ) -> EconomicExecutionReconciliationLedger {
        let record = EconomicExecutionReconciliation {
            reconciliation_id: "reconciliation:completion".into(),
            execution_id: receipt.execution_id.clone(),
            execution_fingerprint: receipt.fingerprint().unwrap(),
            constraint_id: constraint.constraint_id.clone(),
            constraint_fingerprint: constraint.fingerprint().unwrap(),
            result: ExecutionConformance::Conformant,
            evidence_refs: vec!["evidence:reconciliation".into()],
            recorded_at: 1_600,
        };

        serde_json::from_value(serde_json::json!({
            "reconciliations": [record]
        }))
        .unwrap()
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

    fn ready_assessment() -> (
        EconomicFinalizationAssessment,
        EconomicActionLifecycle,
        EconomicActionScope,
        SubstrateLedger,
        ImpactLedger,
        EconomicExecutionReconciliationLedger,
        EconomicExecutionLedger,
        Vec<EconomicExecutionConstraint>,
    ) {
        let lifecycle = completed_lifecycle();
        let scope = scope();
        let constraint = constraint(&lifecycle);
        let receipt = receipt(&lifecycle);
        let reconciliations = reconciliation_ledger(&receipt, &constraint);
        let execution = execution_ledger(receipt);

        let assessment = EconomicActionFinalizationGate::assess(
            &lifecycle,
            &scope,
            &healthy_substrate(),
            &ImpactLedger::new(),
            &reconciliations,
            &execution,
            std::slice::from_ref(&constraint),
        )
        .unwrap();

        assert_eq!(assessment.decision, EconomicFinalizationDecision::Ready);

        (
            assessment,
            lifecycle,
            scope,
            healthy_substrate(),
            ImpactLedger::new(),
            reconciliations,
            execution,
            vec![constraint],
        )
    }

    #[test]
    fn fresh_ready_assessment_issues_one_terminal_certificate() {
        let (assessment, lifecycle, scope, substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();

        let certificate = ledger
            .issue(
                &assessment,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
                "finalization:1",
                "authority:dao-1",
                vec!["evidence:finalization".into()],
                1_700,
            )
            .unwrap();

        assert!(ledger.is_finalized());
        assert_eq!(ledger.certificates(), &[certificate.clone()]);
        assert_eq!(
            certificate.evidence_snapshot_fingerprint,
            assessment.evidence_snapshot_fingerprint
        );
    }

    #[test]
    fn certificate_fingerprint_changes_when_content_changes() {
        let (assessment, ..) = ready_assessment();
        let mut certificate = EconomicFinalizationCertificate {
            certificate_id: "finalization:1".into(),
            action_ref: assessment.action_ref.clone(),
            lifecycle_revision_id: assessment.lifecycle_revision_id.clone(),
            scope_id: assessment.scope_id.clone(),
            scope_fingerprint: assessment.scope_fingerprint.clone(),
            evidence_snapshot_fingerprint: assessment.evidence_snapshot_fingerprint.clone(),
            certificate_fingerprint: "0".repeat(64),
            authority_ref: "authority:dao-1".into(),
            evidence_refs: vec!["evidence:finalization".into()],
            finalized_at: 1_700,
        };
        let original = certificate.fingerprint().unwrap();
        certificate.authority_ref = "authority:dao-2".into();
        assert_ne!(original, certificate.fingerprint().unwrap());
    }

    #[test]
    fn stale_assessment_cannot_issue_certificate() {
        let (assessment, lifecycle, scope, mut substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();

        substrate
            .record_event(crate::economics::substrate::SubstrateEvent {
                id: "substrate:change".into(),
                dimension: SubstrateDimension::Financial,
                delta: -1,
                kind: crate::economics::substrate::SubstrateEventKind::Maintenance,
                actor: "actor:dao".into(),
                timestamp: 1_700,
                evidence_ref: Some("evidence:change".into()),
            })
            .unwrap();

        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();
        let result = ledger.issue(
            &assessment,
            &lifecycle,
            &scope,
            &substrate,
            &impacts,
            &reconciliations,
            &execution,
            &constraints,
            "finalization:stale",
            "authority:dao-1",
            vec!["evidence:finalization".into()],
            1_800,
        );

        assert!(result.is_err());
        assert!(!ledger.is_finalized());
    }

    #[test]
    fn persisted_certificate_mutation_is_rejected_by_validation() {
        let (assessment, lifecycle, scope, substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();
        let certificate = ledger
            .issue(
                &assessment,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
                "finalization:1",
                "authority:dao-1",
                vec!["evidence:finalization".into()],
                1_700,
            )
            .unwrap();

        let mut mutated = certificate;
        mutated.authority_ref = "authority:tampered".into();

        assert!(mutated.validate().is_err());
    }

    #[test]
    fn second_certificate_for_same_action_is_rejected() {
        let (assessment, lifecycle, scope, substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();
        ledger
            .issue(
                &assessment,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
                "finalization:first",
                "authority:dao-1",
                vec!["evidence:first".into()],
                1_700,
            )
            .unwrap();

        let result = ledger.issue(
            &assessment,
            &lifecycle,
            &scope,
            &substrate,
            &impacts,
            &reconciliations,
            &execution,
            &constraints,
            "finalization:second",
            "authority:dao-1",
            vec!["evidence:second".into()],
            1_800,
        );

        assert!(result.is_err());
        assert_eq!(ledger.certificates().len(), 1);
    }

    #[test]
    fn unrecorded_certificate_cannot_verify_as_current() {
        let (assessment, lifecycle, scope, substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();

        let certificate = EconomicFinalizationCertificate {
            certificate_id: "finalization:forged".into(),
            action_ref: "action:1".into(),
            lifecycle_revision_id: assessment.lifecycle_revision_id.clone(),
            scope_id: assessment.scope_id.clone(),
            scope_fingerprint: assessment.scope_fingerprint.clone(),
            evidence_snapshot_fingerprint: assessment.evidence_snapshot_fingerprint.clone(),
            certificate_fingerprint: "0".repeat(64),
            authority_ref: "authority:dao-1".into(),
            evidence_refs: vec!["evidence:forged".into()],
            finalized_at: 1_700,
        };

        assert!(!ledger
            .verify_certificate(
                &certificate,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
            )
            .is_ok());
    }

    #[test]
    fn certificate_verification_rejects_pre_completion_timestamp() {
        let (assessment, lifecycle, scope, substrate, impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();

        let certificate = ledger
            .issue(
                &assessment,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
                "finalization:1",
                "authority:dao-1",
                vec!["evidence:finalization".into()],
                1_700,
            )
            .unwrap();

        let mut persisted = certificate.clone();
        persisted.finalized_at = 1_399;
        ledger.certificates[0] = persisted.clone();

        assert!(!ledger
            .verify_certificate(
                &persisted,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
            )
            .unwrap());
    }

    #[test]
    fn certificate_verification_detects_current_evidence_change() {
        let (assessment, lifecycle, scope, substrate, mut impacts, reconciliations, execution, constraints) =
            ready_assessment();
        let mut ledger = EconomicFinalizationLedger::new("action:1").unwrap();
        let certificate = ledger
            .issue(
                &assessment,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
                "finalization:1",
                "authority:dao-1",
                vec!["evidence:finalization".into()],
                1_700,
            )
            .unwrap();

        assert!(ledger
            .verify_certificate(
                &certificate,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
            )
            .unwrap());

        impacts
            .record_impact(crate::economics::impact::SubstrateImpact {
                id: "impact:local".into(),
                action_actor: "actor:dao".into(),
                action_ref: "action:1".into(),
                dimension: SubstrateDimension::Financial,
                unit: "sap".into(),
                magnitude: 1,
                direction: crate::economics::impact::ImpactDirection::Depletion,
                affected_ref: "commons:1".into(),
                attributions: Vec::new(),
                evidence_refs: vec!["evidence:impact".into()],
                status: crate::economics::impact::ImpactStatus::Open,
                obligation_id: None,
                recorded_at: 1_750,
            })
            .unwrap();

        assert!(!ledger
            .verify_certificate(
                &certificate,
                &lifecycle,
                &scope,
                &substrate,
                &impacts,
                &reconciliations,
                &execution,
                &constraints,
            )
            .unwrap());
    }
}
