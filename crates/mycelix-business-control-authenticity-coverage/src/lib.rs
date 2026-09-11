// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Full-interval authenticity coverage for control statements used in Business qualification.
//!
//! This crate does not verify cryptography. It proves that every control statement already admitted
//! by `mycelix-business-control-coverage` is paired with an externally verified authenticity receipt
//! under one preregistered verifier policy, and that every receipt remains current at assembly time.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_control_coverage::{
    ControlCoverageEntry, ControlCoverageEvidence, ControlCoveragePlan, CoverageError,
};
use mycelix_business_control_reconciliation::control_source_authenticity_unverified_ref;
use mycelix_business_core::{Digest32, ReferenceId, ScopeRef};
use mycelix_business_evidence_authenticity::{AuthenticityError, VerifiedEvidenceAuthenticityRef};
use sha2::{Digest, Sha256};

pub const CONTROL_AUTHENTICITY_COVERAGE_IS_READ_ONLY: bool = true;
pub const CRYPTOGRAPHY_REMAINS_EXTERNAL: bool = true;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

pub fn control_source_issuer_authority_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:control-source-issuer-authority-unverified:v1")
        .expect("static limitation id is canonical")
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlAuthenticityCoverageConfig {
    pub plan_id: ReferenceId,
    pub verifier_domain: ReferenceId,
    pub verification_method: ReferenceId,
    pub verification_policy_digest: Digest32,
    pub registered_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlAuthenticityCoveragePlan {
    pub plan_id: ReferenceId,
    pub control_coverage_plan_digest: Digest32,
    pub claimed_issuer: ReferenceId,
    pub scope: ScopeRef,
    pub qualification_start_unix_ms: u64,
    pub qualification_end_unix_ms: u64,
    pub verifier_domain: ReferenceId,
    pub verification_method: ReferenceId,
    pub verification_policy_digest: Digest32,
    pub registered_at_unix_ms: u64,
    pub plan_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthenticityCoveragePlanError {
    InvalidControlPlan,
    ZeroControlPlanDigest,
    ZeroVerificationPolicyDigest,
    NotPreregistered,
    RegisteredAfterControlPlan,
    InvalidQualificationWindow,
    ZeroDigest,
    BindingMismatch,
    DigestMismatch,
}

impl ControlAuthenticityCoveragePlan {
    pub fn build(
        control_plan: &ControlCoveragePlan,
        config: ControlAuthenticityCoverageConfig,
    ) -> Result<Self, AuthenticityCoveragePlanError> {
        control_plan
            .validate()
            .map_err(|_| AuthenticityCoveragePlanError::InvalidControlPlan)?;
        let mut value = Self {
            plan_id: config.plan_id,
            control_coverage_plan_digest: control_plan.plan_digest,
            claimed_issuer: control_plan.control_source.clone(),
            scope: control_plan.scope.clone(),
            qualification_start_unix_ms: control_plan.qualification_start_unix_ms,
            qualification_end_unix_ms: control_plan.qualification_end_unix_ms,
            verifier_domain: config.verifier_domain,
            verification_method: config.verification_method,
            verification_policy_digest: config.verification_policy_digest,
            registered_at_unix_ms: config.registered_at_unix_ms,
            plan_digest: Digest32([0; 32]),
        };
        value.plan_digest = coverage_plan_digest(&value);
        value.validate_against(control_plan)?;
        Ok(value)
    }

    pub fn validate_against(
        &self,
        control_plan: &ControlCoveragePlan,
    ) -> Result<(), AuthenticityCoveragePlanError> {
        control_plan
            .validate()
            .map_err(|_| AuthenticityCoveragePlanError::InvalidControlPlan)?;
        if zero_digest(&self.control_coverage_plan_digest) {
            return Err(AuthenticityCoveragePlanError::ZeroControlPlanDigest);
        }
        if zero_digest(&self.verification_policy_digest) {
            return Err(AuthenticityCoveragePlanError::ZeroVerificationPolicyDigest);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.qualification_start_unix_ms
        {
            return Err(AuthenticityCoveragePlanError::NotPreregistered);
        }
        if self.registered_at_unix_ms > control_plan.registered_at_unix_ms {
            return Err(AuthenticityCoveragePlanError::RegisteredAfterControlPlan);
        }
        if self.qualification_start_unix_ms == 0
            || self.qualification_start_unix_ms >= self.qualification_end_unix_ms
        {
            return Err(AuthenticityCoveragePlanError::InvalidQualificationWindow);
        }
        if self.control_coverage_plan_digest != control_plan.plan_digest
            || self.claimed_issuer != control_plan.control_source
            || self.scope != control_plan.scope
            || self.qualification_start_unix_ms != control_plan.qualification_start_unix_ms
            || self.qualification_end_unix_ms != control_plan.qualification_end_unix_ms
        {
            return Err(AuthenticityCoveragePlanError::BindingMismatch);
        }
        if zero_digest(&self.plan_digest) {
            return Err(AuthenticityCoveragePlanError::ZeroDigest);
        }
        if self.plan_digest != coverage_plan_digest(self) {
            return Err(AuthenticityCoveragePlanError::DigestMismatch);
        }
        Ok(())
    }
}

fn coverage_plan_digest(value: &ControlAuthenticityCoveragePlan) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-authenticity-coverage-plan:v1");
    hash_str(&mut hasher, value.plan_id.as_str());
    hasher.update(value.control_coverage_plan_digest.0);
    hash_str(&mut hasher, value.claimed_issuer.as_str());
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hasher.update(value.qualification_start_unix_ms.to_be_bytes());
    hasher.update(value.qualification_end_unix_ms.to_be_bytes());
    hash_str(&mut hasher, value.verifier_domain.as_str());
    hash_str(&mut hasher, value.verification_method.as_str());
    hasher.update(value.verification_policy_digest.0);
    hasher.update(value.registered_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthenticityRevalidationContext {
    pub verifier_domain: ReferenceId,
    pub credential: ReferenceId,
    pub current_verifier_epoch: u64,
    pub current_credential_epoch: u64,
    pub current_revocation_frontier_digest: Digest32,
    pub revalidated_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlAuthenticityEntry {
    pub window_id: ReferenceId,
    pub authenticity: VerifiedEvidenceAuthenticityRef,
    pub revalidation: AuthenticityRevalidationContext,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WindowAuthenticityEvidence {
    pub window_id: ReferenceId,
    pub statement_digest: Digest32,
    pub source_document_digest: Digest32,
    pub authenticity_binding_digest: Digest32,
    pub verifier_receipt_digest: Digest32,
    pub credential: ReferenceId,
    pub credential_epoch: u64,
    pub revocation_frontier_digest: Digest32,
    pub revalidated_at_unix_ms: u64,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlAuthenticityCoverageEvidence {
    pub plan_digest: Digest32,
    pub control_coverage_digest: Digest32,
    pub windows: Vec<WindowAuthenticityEvidence>,
    pub coverage_digest: Digest32,
}

#[derive(Debug)]
pub enum AuthenticityCoverageError {
    Plan(AuthenticityCoveragePlanError),
    ControlCoverage(CoverageError),
    EntryCountMismatch { expected: usize, actual: usize },
    ControlEntryCountMismatch { expected: usize, actual: usize },
    DuplicateAuthenticityEntry { window_id: ReferenceId },
    DuplicateControlEntry { window_id: ReferenceId },
    UnknownWindow { window_id: ReferenceId },
    MissingWindow { window_id: ReferenceId },
    ControlReceiptMismatch { window_id: ReferenceId },
    StatementInvalid { window_id: ReferenceId },
    IssuerMismatch { window_id: ReferenceId },
    VerifierPolicyMismatch { window_id: ReferenceId },
    ReceiptVerifiedBeforeStatementIssued { window_id: ReferenceId },
    RevalidationContextMismatch { window_id: ReferenceId },
    Authenticity { window_id: ReferenceId, error: AuthenticityError },
    DuplicateVerifierReceipt,
    DuplicateAuthenticityBinding,
    WindowEvidenceInvalid { window_id: ReferenceId },
    ZeroDigest,
    DigestMismatch,
}

pub fn verify_control_authenticity_coverage(
    plan: &ControlAuthenticityCoveragePlan,
    control_plan: &ControlCoveragePlan,
    control_coverage: &ControlCoverageEvidence,
    control_entries: &[ControlCoverageEntry],
    authenticity_entries: &[ControlAuthenticityEntry],
) -> Result<ControlAuthenticityCoverageEvidence, AuthenticityCoverageError> {
    plan.validate_against(control_plan)
        .map_err(AuthenticityCoverageError::Plan)?;
    control_coverage
        .validate_against(control_plan)
        .map_err(AuthenticityCoverageError::ControlCoverage)?;
    let expected = control_plan.windows.len();
    if authenticity_entries.len() != expected {
        return Err(AuthenticityCoverageError::EntryCountMismatch {
            expected,
            actual: authenticity_entries.len(),
        });
    }
    if control_entries.len() != expected {
        return Err(AuthenticityCoverageError::ControlEntryCountMismatch {
            expected,
            actual: control_entries.len(),
        });
    }

    let control_by_window = index_control_entries(control_entries)?;
    let auth_by_window = index_authenticity_entries(authenticity_entries)?;
    let receipt_by_window = control_coverage
        .windows
        .iter()
        .map(|receipt| (receipt.window_id.clone(), receipt))
        .collect::<BTreeMap<_, _>>();

    let mut verifier_receipts = BTreeSet::new();
    let mut authenticity_bindings = BTreeSet::new();
    let mut windows = Vec::with_capacity(expected);

    for planned_window in &control_plan.windows {
        let window_id = &planned_window.window_id;
        let Some(control) = control_by_window.get(window_id) else {
            return Err(AuthenticityCoverageError::MissingWindow {
                window_id: window_id.clone(),
            });
        };
        let Some(auth_entry) = auth_by_window.get(window_id) else {
            return Err(AuthenticityCoverageError::MissingWindow {
                window_id: window_id.clone(),
            });
        };
        let Some(control_receipt) = receipt_by_window.get(window_id) else {
            return Err(AuthenticityCoverageError::MissingWindow {
                window_id: window_id.clone(),
            });
        };

        control
            .contract
            .validate()
            .map_err(|_| AuthenticityCoverageError::StatementInvalid {
                window_id: window_id.clone(),
            })?;
        control
            .statement
            .validate_against(&control.contract)
            .map_err(|_| AuthenticityCoverageError::StatementInvalid {
                window_id: window_id.clone(),
            })?;
        if control.contract.contract_digest != control_receipt.contract_digest
            || control.statement.statement_digest != control_receipt.statement_digest
            || control.reconciliation.evidence_digest != control_receipt.reconciliation_digest
            || control.contract.window_start_unix_ms != planned_window.start_unix_ms
            || control.contract.window_end_unix_ms != planned_window.end_unix_ms
        {
            return Err(AuthenticityCoverageError::ControlReceiptMismatch {
                window_id: window_id.clone(),
            });
        }
        if control.contract.control_source != plan.claimed_issuer {
            return Err(AuthenticityCoverageError::IssuerMismatch {
                window_id: window_id.clone(),
            });
        }

        let authenticity = &auth_entry.authenticity;
        authenticity
            .binds_subject_and_issuer(
                control.statement.source_document_digest,
                &plan.claimed_issuer,
            )
            .map_err(|error| AuthenticityCoverageError::Authenticity {
                window_id: window_id.clone(),
                error,
            })?;
        if authenticity.verifier_domain != plan.verifier_domain
            || authenticity.verification_method != plan.verification_method
            || authenticity.verification_policy_digest != plan.verification_policy_digest
        {
            return Err(AuthenticityCoverageError::VerifierPolicyMismatch {
                window_id: window_id.clone(),
            });
        }
        if authenticity.verified_at_unix_ms < control.statement.issued_at_unix_ms {
            return Err(AuthenticityCoverageError::ReceiptVerifiedBeforeStatementIssued {
                window_id: window_id.clone(),
            });
        }
        let context = &auth_entry.revalidation;
        if context.verifier_domain != authenticity.verifier_domain
            || context.credential != authenticity.credential
        {
            return Err(AuthenticityCoverageError::RevalidationContextMismatch {
                window_id: window_id.clone(),
            });
        }
        authenticity
            .validate_at(
                context.revalidated_at_unix_ms,
                context.current_verifier_epoch,
                context.current_credential_epoch,
                context.current_revocation_frontier_digest,
            )
            .map_err(|error| AuthenticityCoverageError::Authenticity {
                window_id: window_id.clone(),
                error,
            })?;
        if !verifier_receipts.insert(authenticity.verifier_receipt_digest) {
            return Err(AuthenticityCoverageError::DuplicateVerifierReceipt);
        }
        if !authenticity_bindings.insert(authenticity.binding_digest) {
            return Err(AuthenticityCoverageError::DuplicateAuthenticityBinding);
        }

        let mut receipt = WindowAuthenticityEvidence {
            window_id: window_id.clone(),
            statement_digest: control.statement.statement_digest,
            source_document_digest: control.statement.source_document_digest,
            authenticity_binding_digest: authenticity.binding_digest,
            verifier_receipt_digest: authenticity.verifier_receipt_digest,
            credential: authenticity.credential.clone(),
            credential_epoch: authenticity.credential_epoch,
            revocation_frontier_digest: authenticity.revocation_frontier_digest,
            revalidated_at_unix_ms: context.revalidated_at_unix_ms,
            evidence_digest: Digest32([0; 32]),
        };
        receipt.evidence_digest = window_authenticity_digest(&receipt);
        receipt.validate_against(control_receipt)?;
        windows.push(receipt);
    }

    let mut evidence = ControlAuthenticityCoverageEvidence {
        plan_digest: plan.plan_digest,
        control_coverage_digest: control_coverage.coverage_digest,
        windows,
        coverage_digest: Digest32([0; 32]),
    };
    evidence.coverage_digest = authenticity_coverage_digest(&evidence);
    evidence.validate_against(plan, control_plan, control_coverage)?;
    Ok(evidence)
}

fn index_control_entries<'a>(
    entries: &'a [ControlCoverageEntry],
) -> Result<BTreeMap<ReferenceId, &'a ControlCoverageEntry>, AuthenticityCoverageError> {
    let mut indexed = BTreeMap::new();
    for entry in entries {
        if indexed.insert(entry.window_id.clone(), entry).is_some() {
            return Err(AuthenticityCoverageError::DuplicateControlEntry {
                window_id: entry.window_id.clone(),
            });
        }
    }
    Ok(indexed)
}

fn index_authenticity_entries<'a>(
    entries: &'a [ControlAuthenticityEntry],
) -> Result<BTreeMap<ReferenceId, &'a ControlAuthenticityEntry>, AuthenticityCoverageError> {
    let mut indexed = BTreeMap::new();
    for entry in entries {
        if indexed.insert(entry.window_id.clone(), entry).is_some() {
            return Err(AuthenticityCoverageError::DuplicateAuthenticityEntry {
                window_id: entry.window_id.clone(),
            });
        }
    }
    Ok(indexed)
}

impl WindowAuthenticityEvidence {
    fn validate_against(
        &self,
        control_receipt: &mycelix_business_control_coverage::WindowCoverageEvidence,
    ) -> Result<(), AuthenticityCoverageError> {
        if self.window_id != control_receipt.window_id
            || self.statement_digest != control_receipt.statement_digest
            || zero_digest(&self.source_document_digest)
            || zero_digest(&self.authenticity_binding_digest)
            || zero_digest(&self.verifier_receipt_digest)
            || self.credential_epoch == 0
            || zero_digest(&self.revocation_frontier_digest)
            || self.revalidated_at_unix_ms == 0
            || zero_digest(&self.evidence_digest)
            || self.evidence_digest != window_authenticity_digest(self)
        {
            return Err(AuthenticityCoverageError::WindowEvidenceInvalid {
                window_id: self.window_id.clone(),
            });
        }
        Ok(())
    }
}

impl ControlAuthenticityCoverageEvidence {
    pub fn validate_against(
        &self,
        plan: &ControlAuthenticityCoveragePlan,
        control_plan: &ControlCoveragePlan,
        control_coverage: &ControlCoverageEvidence,
    ) -> Result<(), AuthenticityCoverageError> {
        plan.validate_against(control_plan)
            .map_err(AuthenticityCoverageError::Plan)?;
        control_coverage
            .validate_against(control_plan)
            .map_err(AuthenticityCoverageError::ControlCoverage)?;
        if self.plan_digest != plan.plan_digest
            || self.control_coverage_digest != control_coverage.coverage_digest
            || zero_digest(&self.coverage_digest)
        {
            return Err(AuthenticityCoverageError::DigestMismatch);
        }
        if self.windows.len() != control_plan.windows.len() {
            return Err(AuthenticityCoverageError::EntryCountMismatch {
                expected: control_plan.windows.len(),
                actual: self.windows.len(),
            });
        }
        let mut verifier_receipts = BTreeSet::new();
        let mut authenticity_bindings = BTreeSet::new();
        for ((planned, control_receipt), authenticity_receipt) in control_plan
            .windows
            .iter()
            .zip(&control_coverage.windows)
            .zip(&self.windows)
        {
            if planned.window_id != control_receipt.window_id {
                return Err(AuthenticityCoverageError::ControlReceiptMismatch {
                    window_id: planned.window_id.clone(),
                });
            }
            authenticity_receipt.validate_against(control_receipt)?;
            if !verifier_receipts.insert(authenticity_receipt.verifier_receipt_digest) {
                return Err(AuthenticityCoverageError::DuplicateVerifierReceipt);
            }
            if !authenticity_bindings.insert(authenticity_receipt.authenticity_binding_digest) {
                return Err(AuthenticityCoverageError::DuplicateAuthenticityBinding);
            }
        }
        if self.coverage_digest != authenticity_coverage_digest(self) {
            return Err(AuthenticityCoverageError::DigestMismatch);
        }
        Ok(())
    }

    /// A limitation transition requires the original external authenticity receipts and current
    /// verifier/credential/revocation contexts. A digest-valid summary alone is insufficient.
    pub fn qualification_limitation_transition(
        &self,
        plan: &ControlAuthenticityCoveragePlan,
        control_plan: &ControlCoveragePlan,
        control_coverage: &ControlCoverageEvidence,
        control_entries: &[ControlCoverageEntry],
        authenticity_entries: &[ControlAuthenticityEntry],
    ) -> Result<AuthenticityQualificationTransition, AuthenticityCoverageError> {
        let rebuilt = verify_control_authenticity_coverage(
            plan,
            control_plan,
            control_coverage,
            control_entries,
            authenticity_entries,
        )?;
        if &rebuilt != self {
            return Err(AuthenticityCoverageError::DigestMismatch);
        }
        Ok(AuthenticityQualificationTransition::new(
            plan,
            self.coverage_digest,
        ))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthenticityQualificationTransition {
    pub from: ReferenceId,
    pub to: ReferenceId,
    pub authenticity_plan_digest: Digest32,
    pub scope: ScopeRef,
    pub qualification_start_unix_ms: u64,
    pub qualification_end_unix_ms: u64,
    pub supporting_authenticity_coverage_digest: Digest32,
    pub transition_digest: Digest32,
}

impl AuthenticityQualificationTransition {
    fn new(
        plan: &ControlAuthenticityCoveragePlan,
        supporting_authenticity_coverage_digest: Digest32,
    ) -> Self {
        let mut value = Self {
            from: control_source_authenticity_unverified_ref(),
            to: control_source_issuer_authority_unverified_ref(),
            authenticity_plan_digest: plan.plan_digest,
            scope: plan.scope.clone(),
            qualification_start_unix_ms: plan.qualification_start_unix_ms,
            qualification_end_unix_ms: plan.qualification_end_unix_ms,
            supporting_authenticity_coverage_digest,
            transition_digest: Digest32([0; 32]),
        };
        value.transition_digest = authenticity_transition_digest(&value);
        value
    }

    pub fn validate_against(
        &self,
        plan: &ControlAuthenticityCoveragePlan,
        evidence: &ControlAuthenticityCoverageEvidence,
    ) -> bool {
        self.from == control_source_authenticity_unverified_ref()
            && self.to == control_source_issuer_authority_unverified_ref()
            && self.from != self.to
            && self.authenticity_plan_digest == plan.plan_digest
            && self.scope == plan.scope
            && self.qualification_start_unix_ms == plan.qualification_start_unix_ms
            && self.qualification_end_unix_ms == plan.qualification_end_unix_ms
            && self.supporting_authenticity_coverage_digest == evidence.coverage_digest
            && !zero_digest(&self.supporting_authenticity_coverage_digest)
            && !zero_digest(&self.transition_digest)
            && self.transition_digest == authenticity_transition_digest(self)
    }
}

fn window_authenticity_digest(value: &WindowAuthenticityEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-window-authenticity-evidence:v1");
    hash_str(&mut hasher, value.window_id.as_str());
    hasher.update(value.statement_digest.0);
    hasher.update(value.source_document_digest.0);
    hasher.update(value.authenticity_binding_digest.0);
    hasher.update(value.verifier_receipt_digest.0);
    hash_str(&mut hasher, value.credential.as_str());
    hasher.update(value.credential_epoch.to_be_bytes());
    hasher.update(value.revocation_frontier_digest.0);
    hasher.update(value.revalidated_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

fn authenticity_coverage_digest(value: &ControlAuthenticityCoverageEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:control-authenticity-coverage-evidence:v1");
    hasher.update(value.plan_digest.0);
    hasher.update(value.control_coverage_digest.0);
    hasher.update((value.windows.len() as u64).to_be_bytes());
    for window in &value.windows {
        hasher.update(window.evidence_digest.0);
    }
    finish_digest(hasher)
}

fn authenticity_transition_digest(value: &AuthenticityQualificationTransition) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:authenticity-qualification-transition:v1");
    hash_str(&mut hasher, value.from.as_str());
    hash_str(&mut hasher, value.to.as_str());
    hasher.update(value.authenticity_plan_digest.0);
    hash_str(&mut hasher, value.scope.as_ref_id().as_str());
    hasher.update(value.qualification_start_unix_ms.to_be_bytes());
    hasher.update(value.qualification_end_unix_ms.to_be_bytes());
    hasher.update(value.supporting_authenticity_coverage_digest.0);
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use mycelix_business_control_coverage::{ControlCoveragePlan, ControlCoverageWindow};
    use mycelix_business_control_reconciliation::{ControlAggregationKind, ControlSourceClass};
    use mycelix_business_ingress::IngressQualificationBinding;

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn control_plan() -> ControlCoveragePlan {
        ControlCoveragePlan::build(
            id("control-plan:test"),
            IngressQualificationBinding {
                source_system: id("source:pos"),
                adapter_semantic_id: id("adapter:test:v1"),
                adapter_digest: Digest32::repeat(1),
                mapping_digest: Digest32::repeat(2),
                source_schema_digest: Digest32::repeat(3),
            },
            id("input:hospitality:sales-transactions:v1"),
            id("metric:hospitality:item-demand"),
            ScopeRef(id("scope:location:a")),
            id("unit:count"),
            0,
            ControlAggregationKind::Sum,
            id("issuer:provider"),
            ControlSourceClass::ProviderAuthoritativeReport,
            200,
            1_000,
            3_000,
            vec![
                ControlCoverageWindow {
                    window_id: id("window:1"),
                    start_unix_ms: 1_000,
                    end_unix_ms: 2_000,
                },
                ControlCoverageWindow {
                    window_id: id("window:2"),
                    start_unix_ms: 2_000,
                    end_unix_ms: 3_000,
                },
            ],
        )
        .unwrap()
    }

    #[test]
    fn authenticity_plan_must_be_frozen_no_later_than_control_plan() {
        let control = control_plan();
        let ok = ControlAuthenticityCoveragePlan::build(
            &control,
            ControlAuthenticityCoverageConfig {
                plan_id: id("auth-plan:ok"),
                verifier_domain: id("verifier:xenia"),
                verification_method: id("method:signature:v1"),
                verification_policy_digest: Digest32::repeat(4),
                registered_at_unix_ms: 200,
            },
        );
        assert!(ok.is_ok());
        let late = ControlAuthenticityCoveragePlan::build(
            &control,
            ControlAuthenticityCoverageConfig {
                plan_id: id("auth-plan:late"),
                verifier_domain: id("verifier:xenia"),
                verification_method: id("method:signature:v1"),
                verification_policy_digest: Digest32::repeat(4),
                registered_at_unix_ms: 201,
            },
        );
        assert!(matches!(
            late,
            Err(AuthenticityCoveragePlanError::RegisteredAfterControlPlan)
        ));
    }

    #[test]
    fn qualification_transition_narrows_only_authenticity() {
        let control = control_plan();
        let plan = ControlAuthenticityCoveragePlan::build(
            &control,
            ControlAuthenticityCoverageConfig {
                plan_id: id("auth-plan:test"),
                verifier_domain: id("verifier:xenia"),
                verification_method: id("method:signature:v1"),
                verification_policy_digest: Digest32::repeat(4),
                registered_at_unix_ms: 200,
            },
        )
        .unwrap();
        let evidence = ControlAuthenticityCoverageEvidence {
            plan_digest: plan.plan_digest,
            control_coverage_digest: Digest32::repeat(5),
            windows: vec![],
            coverage_digest: Digest32::repeat(6),
        };
        let transition = AuthenticityQualificationTransition::new(&plan, evidence.coverage_digest);
        assert_eq!(transition.from, control_source_authenticity_unverified_ref());
        assert_eq!(transition.to, control_source_issuer_authority_unverified_ref());
    }
}
