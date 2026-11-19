// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Fail-closed reference lifecycle for land/resource operations that depend on
//! an external authority.
//!
//! This is a deterministic policy model, not a registry client, legal rules
//! engine, cryptographic verifier, or Property transaction-coordinator gate.
//! It separates internal workflow state from external receipts and records
//! only a scoped projection of an externally reported status. No phase
//! authorizes authoritative land-title or registry mutation.

use chrono::DateTime;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const LAND_OPERATION_LIFECYCLE_SCHEMA: &str = "mycelix.land-operation-lifecycle.v0";
pub const LAND_OPERATION_LIFECYCLE_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/mycelix-land-operation-lifecycle-v0.schema.json";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum LandOperationPhase {
    Prepared,
    Submitted {
        attempt_id: String,
        idempotency_key: String,
    },
    OutcomeUnknown {
        attempt_id: String,
        idempotency_key: String,
    },
    RetryEligible {
        previous_attempt_id: String,
        idempotency_key: String,
        reconciliation_reference: String,
        retry_semantics_reference: String,
    },
    ReceiptObserved {
        attempt_id: String,
        receipt_reference: String,
        receipt_sha256: String,
        captured_at: String,
    },
    ExternalStatusQualified {
        attempt_id: String,
        receipt_reference: String,
        receipt_sha256: String,
        captured_at: String,
        authority_profile_reference: String,
        qualification_reference: String,
        verifier_reference: String,
        qualification_sha256: String,
    },
    ExternalStatusProjected {
        attempt_id: String,
        receipt_reference: String,
        receipt_sha256: String,
        projection_reference: String,
        projection_scope_reference: String,
        authority_profile_reference: String,
        qualification_reference: String,
        verifier_reference: String,
        qualification_sha256: String,
    },
    ExternalStatusInvalidated {
        attempt_id: String,
        receipt_reference: String,
        receipt_sha256: String,
        projection_reference: String,
        invalidation_evidence_reference: String,
        reason: String,
    },
    Held {
        reason: String,
        decision_packet_reference: Option<String>,
    },
    Rejected {
        reason: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubmissionAttemptStatus {
    Submitted,
    OutcomeUnknown,
    ConfirmedNotCommitted,
    ReceiptObserved,
    ReconciliationConflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SubmissionAttempt {
    pub attempt_id: String,
    pub idempotency_key: String,
    pub status: SubmissionAttemptStatus,
    #[serde(default)]
    pub reconciliation_reference: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandOperationLifecycleV0 {
    pub schema: String,
    pub operation_id: String,
    pub operation_sha256: String,
    #[serde(default)]
    pub idempotency_key: Option<String>,
    pub phase: LandOperationPhase,
    #[serde(default)]
    pub attempt_history: Vec<SubmissionAttempt>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "result", rename_all = "snake_case")]
pub enum ReconciliationResult {
    StillUnknown,
    ConfirmedCommitted {
        reconciliation_reference: String,
        receipt_reference: String,
        receipt_sha256: String,
        captured_at: String,
    },
    ConfirmedNotCommitted {
        reconciliation_reference: String,
        same_idempotency_key_retry_supported: bool,
        retry_semantics_reference: Option<String>,
    },
    Conflict {
        reconciliation_reference: String,
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum LandOperationCommand {
    Submit {
        attempt_id: String,
        idempotency_key: String,
    },
    MarkOutcomeUnknown,
    ReconcileUnknown {
        result: ReconciliationResult,
    },
    ObserveReceipt {
        receipt_reference: String,
        receipt_sha256: String,
        captured_at: String,
    },
    QualifyExternalStatus {
        authority_profile_reference: String,
        qualification_reference: String,
        verifier_reference: String,
        qualification_sha256: String,
    },
    ProjectExternalStatus {
        projection_reference: String,
        projection_scope_reference: String,
    },
    InvalidateExternalStatusProjection {
        invalidation_evidence_reference: String,
        reason: String,
    },
    Hold {
        reason: String,
        decision_packet_reference: Option<String>,
    },
    Reject {
        reason: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LandOperationTransitionError {
    InvalidTransition,
    InvalidInput,
    IdempotencyKeyChanged,
    AttemptIdReused,
    ExternalOutcomeStillUnknown,
}

impl LandOperationLifecycleV0 {
    pub fn new(
        operation_id: impl Into<String>,
        operation_sha256: impl Into<String>,
    ) -> Result<Self, LandOperationTransitionError> {
        let operation_id = operation_id.into();
        let operation_sha256 = operation_sha256.into();
        if operation_id.trim().is_empty() || !is_sha256(&operation_sha256) {
            return Err(LandOperationTransitionError::InvalidInput);
        }
        Ok(Self {
            schema: LAND_OPERATION_LIFECYCLE_SCHEMA.into(),
            operation_id,
            operation_sha256,
            idempotency_key: None,
            phase: LandOperationPhase::Prepared,
            attempt_history: Vec::new(),
        })
    }

    /// Apply one deterministic state transition without performing I/O.
    ///
    /// A timed-out request cannot be resubmitted while its outcome is unknown.
    /// A retry becomes eligible only after an explicit reconciliation result
    /// confirms non-commit and affirms that reuse of the same idempotency key
    /// is supported by the external endpoint.
    pub fn apply(
        &mut self,
        command: LandOperationCommand,
    ) -> Result<(), LandOperationTransitionError> {
        match command {
            LandOperationCommand::Submit {
                attempt_id,
                idempotency_key,
            } => {
                if attempt_id.trim().is_empty() || idempotency_key.trim().is_empty() {
                    return Err(LandOperationTransitionError::InvalidInput);
                }
                match &self.phase {
                    LandOperationPhase::Prepared => {
                        if self.idempotency_key.is_some() || !self.attempt_history.is_empty() {
                            return Err(LandOperationTransitionError::InvalidTransition);
                        }
                    }
                    LandOperationPhase::RetryEligible {
                        previous_attempt_id,
                        idempotency_key: previous_key,
                        ..
                    } => {
                        if attempt_id == *previous_attempt_id
                            || self.attempt_history.iter().any(|a| a.attempt_id == attempt_id)
                        {
                            return Err(LandOperationTransitionError::AttemptIdReused);
                        }
                        if idempotency_key != *previous_key
                            || self.idempotency_key.as_deref() != Some(previous_key.as_str())
                        {
                            return Err(LandOperationTransitionError::IdempotencyKeyChanged);
                        }
                    }
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                }

                if self.attempt_history.iter().any(|a| a.attempt_id == attempt_id) {
                    return Err(LandOperationTransitionError::AttemptIdReused);
                }
                if self.idempotency_key.is_none() {
                    self.idempotency_key = Some(idempotency_key.clone());
                }
                self.attempt_history.push(SubmissionAttempt {
                    attempt_id: attempt_id.clone(),
                    idempotency_key: idempotency_key.clone(),
                    status: SubmissionAttemptStatus::Submitted,
                    reconciliation_reference: None,
                });
                self.phase = LandOperationPhase::Submitted {
                    attempt_id,
                    idempotency_key,
                };
                Ok(())
            }
            LandOperationCommand::MarkOutcomeUnknown => {
                let (attempt_id, idempotency_key) = match &self.phase {
                    LandOperationPhase::Submitted {
                        attempt_id,
                        idempotency_key,
                    } => (attempt_id.clone(), idempotency_key.clone()),
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                };
                let attempt = self
                    .attempt_history
                    .last_mut()
                    .ok_or(LandOperationTransitionError::InvalidTransition)?;
                if attempt.attempt_id != attempt_id
                    || attempt.idempotency_key != idempotency_key
                    || attempt.status != SubmissionAttemptStatus::Submitted
                {
                    return Err(LandOperationTransitionError::InvalidTransition);
                }
                attempt.status = SubmissionAttemptStatus::OutcomeUnknown;
                self.phase = LandOperationPhase::OutcomeUnknown {
                    attempt_id,
                    idempotency_key,
                };
                Ok(())
            }
            LandOperationCommand::ReconcileUnknown { result } => {
                let (attempt_id, idempotency_key) = match &self.phase {
                    LandOperationPhase::OutcomeUnknown {
                        attempt_id,
                        idempotency_key,
                    } => (attempt_id.clone(), idempotency_key.clone()),
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                };
                match result {
                    ReconciliationResult::StillUnknown => {
                        Err(LandOperationTransitionError::ExternalOutcomeStillUnknown)
                    }
                    ReconciliationResult::ConfirmedCommitted {
                        reconciliation_reference,
                        receipt_reference,
                        receipt_sha256,
                        captured_at,
                    } => {
                        require_non_empty(&reconciliation_reference)?;
                        validate_receipt(&receipt_reference, &receipt_sha256, &captured_at)?;
                        let attempt = self
                            .attempt_history
                            .last_mut()
                            .ok_or(LandOperationTransitionError::InvalidTransition)?;
                        if attempt.attempt_id != attempt_id {
                            return Err(LandOperationTransitionError::InvalidTransition);
                        }
                        attempt.status = SubmissionAttemptStatus::ReceiptObserved;
                        attempt.reconciliation_reference = Some(reconciliation_reference);
                        self.phase = LandOperationPhase::ReceiptObserved {
                            attempt_id,
                            receipt_reference,
                            receipt_sha256,
                            captured_at,
                        };
                        Ok(())
                    }
                    ReconciliationResult::ConfirmedNotCommitted {
                        reconciliation_reference,
                        same_idempotency_key_retry_supported,
                        retry_semantics_reference,
                    } => {
                        require_non_empty(&reconciliation_reference)?;
                        // Validate every retry prerequisite before changing
                        // attempt history. A rejected command must be atomic:
                        // it cannot leave the phase and attempt status divergent.
                        let retry_semantics_reference = if same_idempotency_key_retry_supported {
                            Some(
                                retry_semantics_reference
                                    .filter(|reference| !reference.trim().is_empty())
                                    .ok_or(LandOperationTransitionError::InvalidInput)?,
                            )
                        } else {
                            None
                        };
                        let attempt = self
                            .attempt_history
                            .last_mut()
                            .ok_or(LandOperationTransitionError::InvalidTransition)?;
                        if attempt.attempt_id != attempt_id {
                            return Err(LandOperationTransitionError::InvalidTransition);
                        }
                        attempt.status = SubmissionAttemptStatus::ConfirmedNotCommitted;
                        attempt.reconciliation_reference =
                            Some(reconciliation_reference.clone());
                        if let Some(retry_semantics_reference) = retry_semantics_reference {
                            self.phase = LandOperationPhase::RetryEligible {
                                previous_attempt_id: attempt_id,
                                idempotency_key,
                                reconciliation_reference,
                                retry_semantics_reference,
                            };
                        } else {
                            self.phase = LandOperationPhase::Held {
                                reason:
                                    "external endpoint does not establish safe same-key retry semantics"
                                        .into(),
                                decision_packet_reference: None,
                            };
                        }
                        Ok(())
                    }
                    ReconciliationResult::Conflict {
                        reconciliation_reference,
                        reason,
                    } => {
                        require_non_empty(&reconciliation_reference)?;
                        require_non_empty(&reason)?;
                        let attempt = self
                            .attempt_history
                            .last_mut()
                            .ok_or(LandOperationTransitionError::InvalidTransition)?;
                        if attempt.attempt_id != attempt_id {
                            return Err(LandOperationTransitionError::InvalidTransition);
                        }
                        attempt.status = SubmissionAttemptStatus::ReconciliationConflict;
                        attempt.reconciliation_reference =
                            Some(reconciliation_reference.clone());
                        self.phase = LandOperationPhase::Held {
                            reason,
                            decision_packet_reference: None,
                        };
                        Ok(())
                    }
                }
            }
            LandOperationCommand::ObserveReceipt {
                receipt_reference,
                receipt_sha256,
                captured_at,
            } => {
                let attempt_id = match &self.phase {
                    LandOperationPhase::Submitted { attempt_id, .. } => attempt_id.clone(),
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                };
                validate_receipt(&receipt_reference, &receipt_sha256, &captured_at)?;
                let attempt = self
                    .attempt_history
                    .last_mut()
                    .ok_or(LandOperationTransitionError::InvalidTransition)?;
                if attempt.attempt_id != attempt_id
                    || attempt.status != SubmissionAttemptStatus::Submitted
                {
                    return Err(LandOperationTransitionError::InvalidTransition);
                }
                attempt.status = SubmissionAttemptStatus::ReceiptObserved;
                self.phase = LandOperationPhase::ReceiptObserved {
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    captured_at,
                };
                Ok(())
            }
            LandOperationCommand::QualifyExternalStatus {
                authority_profile_reference,
                qualification_reference,
                verifier_reference,
                qualification_sha256,
            } => {
                let (attempt_id, receipt_reference, receipt_sha256, captured_at) =
                    match &self.phase {
                        LandOperationPhase::ReceiptObserved {
                            attempt_id,
                            receipt_reference,
                            receipt_sha256,
                            captured_at,
                        } => (
                            attempt_id.clone(),
                            receipt_reference.clone(),
                            receipt_sha256.clone(),
                            captured_at.clone(),
                        ),
                        _ => return Err(LandOperationTransitionError::InvalidTransition),
                    };
                for value in [
                    authority_profile_reference.as_str(),
                    qualification_reference.as_str(),
                    verifier_reference.as_str(),
                ] {
                    require_non_empty(value)?;
                }
                if !is_sha256(&qualification_sha256) {
                    return Err(LandOperationTransitionError::InvalidInput);
                }
                self.phase = LandOperationPhase::ExternalStatusQualified {
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    captured_at,
                    authority_profile_reference,
                    qualification_reference,
                    verifier_reference,
                    qualification_sha256,
                };
                Ok(())
            }
            LandOperationCommand::ProjectExternalStatus {
                projection_reference,
                projection_scope_reference,
            } => {
                let (
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    authority_profile_reference,
                    qualification_reference,
                    verifier_reference,
                    qualification_sha256,
                ) = match &self.phase {
                    LandOperationPhase::ExternalStatusQualified {
                        attempt_id,
                        receipt_reference,
                        receipt_sha256,
                        authority_profile_reference,
                        qualification_reference,
                        verifier_reference,
                        qualification_sha256,
                        ..
                    } => (
                        attempt_id.clone(),
                        receipt_reference.clone(),
                        receipt_sha256.clone(),
                        authority_profile_reference.clone(),
                        qualification_reference.clone(),
                        verifier_reference.clone(),
                        qualification_sha256.clone(),
                    ),
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                };
                require_non_empty(&projection_reference)?;
                require_non_empty(&projection_scope_reference)?;
                self.phase = LandOperationPhase::ExternalStatusProjected {
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    projection_reference,
                    projection_scope_reference,
                    authority_profile_reference,
                    qualification_reference,
                    verifier_reference,
                    qualification_sha256,
                };
                Ok(())
            }
            LandOperationCommand::InvalidateExternalStatusProjection {
                invalidation_evidence_reference,
                reason,
            } => {
                let (
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    projection_reference,
                ) = match &self.phase {
                    LandOperationPhase::ExternalStatusProjected {
                        attempt_id,
                        receipt_reference,
                        receipt_sha256,
                        projection_reference,
                        ..
                    } => (
                        attempt_id.clone(),
                        receipt_reference.clone(),
                        receipt_sha256.clone(),
                        projection_reference.clone(),
                    ),
                    _ => return Err(LandOperationTransitionError::InvalidTransition),
                };
                require_non_empty(&invalidation_evidence_reference)?;
                require_non_empty(&reason)?;
                self.phase = LandOperationPhase::ExternalStatusInvalidated {
                    attempt_id,
                    receipt_reference,
                    receipt_sha256,
                    projection_reference,
                    invalidation_evidence_reference,
                    reason,
                };
                Ok(())
            }
            LandOperationCommand::Hold {
                reason,
                decision_packet_reference,
            } => {
                if is_closed_phase(&self.phase) {
                    return Err(LandOperationTransitionError::InvalidTransition);
                }
                require_non_empty(&reason)?;
                if let Some(reference) = &decision_packet_reference {
                    require_non_empty(reference)?;
                }
                self.phase = LandOperationPhase::Held {
                    reason,
                    decision_packet_reference,
                };
                Ok(())
            }
            LandOperationCommand::Reject { reason } => {
                if is_closed_phase(&self.phase) {
                    return Err(LandOperationTransitionError::InvalidTransition);
                }
                require_non_empty(&reason)?;
                self.phase = LandOperationPhase::Rejected { reason };
                Ok(())
            }
        }
    }

    /// This reference model never grants permission to mutate authoritative state.
    pub const fn allows_authoritative_mutation(&self) -> bool {
        false
    }

    /// Only a qualified receipt may be projected into an internal read model.
    /// This is not a title decision or a registry mutation.
    pub const fn may_record_external_status_projection(&self) -> bool {
        matches!(self.phase, LandOperationPhase::ExternalStatusQualified { .. })
    }

    pub const fn requires_reconciliation(&self) -> bool {
        matches!(
            &self.phase,
            LandOperationPhase::OutcomeUnknown { .. }
                | LandOperationPhase::Held { .. }
                | LandOperationPhase::ExternalStatusInvalidated { .. }
        )
    }

    pub const fn external_status_projection_is_current(&self) -> bool {
        matches!(
            &self.phase,
            LandOperationPhase::ExternalStatusProjected { .. }
        )
    }

    /// Validate serialized state for internal consistency; this does not
    /// authenticate any source, receipt, verifier or authority profile.
    pub fn validate_structural(&self) -> Result<(), Vec<String>> {
        let mut issues = Vec::new();
        if self.schema != LAND_OPERATION_LIFECYCLE_SCHEMA {
            issues.push("unsupported land operation lifecycle schema".into());
        }
        if self.operation_id.trim().is_empty() || !is_sha256(&self.operation_sha256) {
            issues.push("operation id and lowercase SHA-256 digest are required".into());
        }

        let mut attempt_ids = HashSet::new();
        let mut stable_key: Option<&str> = None;
        for attempt in &self.attempt_history {
            if attempt.attempt_id.trim().is_empty() || attempt.idempotency_key.trim().is_empty() {
                issues.push("attempt id and idempotency key must not be empty".into());
            }
            if !attempt_ids.insert(attempt.attempt_id.as_str()) {
                issues.push(format!("duplicate submission attempt id {}", attempt.attempt_id));
            }
            if let Some(existing) = stable_key {
                if existing != attempt.idempotency_key {
                    issues.push("submission attempts changed the stable idempotency key".into());
                }
            } else {
                stable_key = Some(attempt.idempotency_key.as_str());
            }
            if attempt
                .reconciliation_reference
                .as_deref()
                .is_some_and(|reference| reference.trim().is_empty())
            {
                issues.push("reconciliation reference must not be empty".into());
            }
        }
        if stable_key != self.idempotency_key.as_deref() {
            issues.push("lifecycle idempotency key does not match attempt history".into());
        }

        match &self.phase {
            LandOperationPhase::Prepared => {
                if !self.attempt_history.is_empty() || self.idempotency_key.is_some() {
                    issues.push("prepared phase cannot contain submission attempts".into());
                }
            }
            LandOperationPhase::Submitted {
                attempt_id,
                idempotency_key,
            } => self.check_last_attempt(
                attempt_id,
                idempotency_key,
                SubmissionAttemptStatus::Submitted,
                &mut issues,
            ),
            LandOperationPhase::OutcomeUnknown {
                attempt_id,
                idempotency_key,
            } => self.check_last_attempt(
                attempt_id,
                idempotency_key,
                SubmissionAttemptStatus::OutcomeUnknown,
                &mut issues,
            ),
            LandOperationPhase::RetryEligible {
                previous_attempt_id,
                idempotency_key,
                reconciliation_reference,
                retry_semantics_reference,
            } => {
                self.check_last_attempt(
                    previous_attempt_id,
                    idempotency_key,
                    SubmissionAttemptStatus::ConfirmedNotCommitted,
                    &mut issues,
                );
                if reconciliation_reference.trim().is_empty()
                    || retry_semantics_reference.trim().is_empty()
                {
                    issues.push(
                        "retry-eligible phase requires reconciliation and retry-semantics evidence"
                            .into(),
                    );
                }
            }
            LandOperationPhase::ReceiptObserved {
                attempt_id,
                receipt_reference,
                receipt_sha256,
                captured_at,
            } => {
                self.check_last_attempt(
                    attempt_id,
                    self.idempotency_key.as_deref().unwrap_or(""),
                    SubmissionAttemptStatus::ReceiptObserved,
                    &mut issues,
                );
                if validate_receipt(receipt_reference, receipt_sha256, captured_at).is_err() {
                    issues.push("receipt reference, digest and capture time are invalid".into());
                }
            }
            LandOperationPhase::ExternalStatusQualified {
                attempt_id,
                receipt_reference,
                receipt_sha256,
                captured_at,
                authority_profile_reference,
                qualification_reference,
                verifier_reference,
                qualification_sha256,
            } => {
                self.check_last_attempt(
                    attempt_id,
                    self.idempotency_key.as_deref().unwrap_or(""),
                    SubmissionAttemptStatus::ReceiptObserved,
                    &mut issues,
                );
                if validate_receipt(receipt_reference, receipt_sha256, captured_at).is_err()
                    || authority_profile_reference.trim().is_empty()
                    || qualification_reference.trim().is_empty()
                    || verifier_reference.trim().is_empty()
                    || !is_sha256(qualification_sha256)
                {
                    issues.push(
                        "qualified status lacks valid receipt or qualification references".into(),
                    );
                }
            }
            LandOperationPhase::ExternalStatusProjected {
                attempt_id,
                receipt_reference,
                receipt_sha256,
                projection_reference,
                projection_scope_reference,
                authority_profile_reference,
                qualification_reference,
                verifier_reference,
                qualification_sha256,
            } => {
                self.check_last_attempt(
                    attempt_id,
                    self.idempotency_key.as_deref().unwrap_or(""),
                    SubmissionAttemptStatus::ReceiptObserved,
                    &mut issues,
                );
                if receipt_reference.trim().is_empty()
                    || !is_sha256(receipt_sha256)
                    || projection_reference.trim().is_empty()
                    || projection_scope_reference.trim().is_empty()
                    || authority_profile_reference.trim().is_empty()
                    || qualification_reference.trim().is_empty()
                    || verifier_reference.trim().is_empty()
                    || !is_sha256(qualification_sha256)
                {
                    issues.push(
                        "external status projection lacks a scoped receipt and qualification lineage"
                            .into(),
                    );
                }
            }
            LandOperationPhase::ExternalStatusInvalidated {
                attempt_id,
                receipt_reference,
                receipt_sha256,
                projection_reference,
                invalidation_evidence_reference,
                reason,
            } => {
                self.check_last_attempt(
                    attempt_id,
                    self.idempotency_key.as_deref().unwrap_or(""),
                    SubmissionAttemptStatus::ReceiptObserved,
                    &mut issues,
                );
                if receipt_reference.trim().is_empty()
                    || !is_sha256(receipt_sha256)
                    || projection_reference.trim().is_empty()
                    || invalidation_evidence_reference.trim().is_empty()
                    || reason.trim().is_empty()
                {
                    issues.push(
                        "invalidated external status must retain its source and invalidation evidence"
                            .into(),
                    );
                }
            }
            LandOperationPhase::Held {
                reason,
                decision_packet_reference,
            } => {
                if reason.trim().is_empty()
                    || decision_packet_reference
                        .as_deref()
                        .is_some_and(|reference| reference.trim().is_empty())
                {
                    issues.push(
                        "held phase requires a reason and valid optional packet reference".into(),
                    );
                }
            }
            LandOperationPhase::Rejected { reason } => {
                if reason.trim().is_empty() {
                    issues.push("rejected phase requires a reason".into());
                }
            }
        }

        if issues.is_empty() {
            Ok(())
        } else {
            Err(issues)
        }
    }

    fn check_last_attempt(
        &self,
        attempt_id: &str,
        idempotency_key: &str,
        expected_status: SubmissionAttemptStatus,
        issues: &mut Vec<String>,
    ) {
        match self.attempt_history.last() {
            Some(attempt)
                if attempt.attempt_id == attempt_id
                    && attempt.idempotency_key == idempotency_key
                    && attempt.status == expected_status => {}
            _ => issues.push("current phase does not match the latest submission attempt".into()),
        }
    }
}

fn is_closed_phase(phase: &LandOperationPhase) -> bool {
    matches!(
        phase,
        LandOperationPhase::Held { .. }
            | LandOperationPhase::Rejected { .. }
            | LandOperationPhase::ExternalStatusProjected { .. }
            | LandOperationPhase::ExternalStatusInvalidated { .. }
    )
}

fn require_non_empty(value: &str) -> Result<(), LandOperationTransitionError> {
    if value.trim().is_empty() {
        Err(LandOperationTransitionError::InvalidInput)
    } else {
        Ok(())
    }
}

fn validate_receipt(
    receipt_reference: &str,
    receipt_sha256: &str,
    captured_at: &str,
) -> Result<(), LandOperationTransitionError> {
    require_non_empty(receipt_reference)?;
    if !is_sha256(receipt_sha256) || DateTime::parse_from_rfc3339(captured_at).is_err() {
        return Err(LandOperationTransitionError::InvalidInput);
    }
    Ok(())
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Deserialize)]
    struct LifecycleScenarioCorpus {
        suite_id: String,
        suite_version: String,
        scenarios: Vec<LifecycleScenario>,
    }

    #[derive(Deserialize)]
    struct LifecycleScenario {
        scenario_id: String,
        source_scenario_id: String,
        operation_id: String,
        operation_sha256: String,
        commands: Vec<LandOperationCommand>,
        expected_phase: String,
        expected_error: Option<String>,
        expected_authoritative_mutation: bool,
    }

    fn lifecycle() -> LandOperationLifecycleV0 {
        LandOperationLifecycleV0::new("urn:synthetic:operation:001", "a".repeat(64))
            .expect("test operation must be valid")
    }

    fn submit(value: &mut LandOperationLifecycleV0) {
        value
            .apply(LandOperationCommand::Submit {
                attempt_id: "attempt-1".into(),
                idempotency_key: "stable-operation-key-001".into(),
            })
            .expect("first submission must be allowed");
    }

    fn receipt(value: &mut LandOperationLifecycleV0) {
        submit(value);
        value
            .apply(LandOperationCommand::ObserveReceipt {
                receipt_reference: "urn:synthetic:receipt:001".into(),
                receipt_sha256: "b".repeat(64),
                captured_at: "2026-11-19T04:00:00Z".into(),
            })
            .expect("receipt must be recorded");
    }

    fn qualify(value: &mut LandOperationLifecycleV0) {
        receipt(value);
        value
            .apply(LandOperationCommand::QualifyExternalStatus {
                authority_profile_reference: "urn:synthetic:authority-profile:001".into(),
                qualification_reference: "urn:synthetic:qualification:001".into(),
                verifier_reference: "urn:synthetic:verifier:001".into(),
                qualification_sha256: "c".repeat(64),
            })
            .expect("qualified status reference must be recorded");
    }

    #[test]
    fn tib_transition_scenarios_match_the_executable_lifecycle_policy() {
        let corpus: LifecycleScenarioCorpus = serde_json::from_str(include_str!(
            "../../../examples/land-operation-lifecycle-sequences-v0.json"
        ))
        .expect("TIB lifecycle sequence corpus must deserialize");
        assert_eq!(corpus.suite_id, "TIB-PROPERTY-STATE-TRANSITIONS-001");
        assert_eq!(corpus.suite_version, "0.1.0");
        assert_eq!(corpus.scenarios.len(), 9);

        for scenario in corpus.scenarios {
            let mut value = LandOperationLifecycleV0::new(
                scenario.operation_id,
                scenario.operation_sha256,
            )
            .expect("synthetic operation identity must be valid");
            let mut observed_error = None;
            for command in scenario.commands {
                if let Err(error) = value.apply(command) {
                    observed_error = Some(match error {
                        LandOperationTransitionError::InvalidTransition => "invalid_transition",
                        LandOperationTransitionError::InvalidInput => "invalid_input",
                        LandOperationTransitionError::IdempotencyKeyChanged => {
                            "idempotency_key_changed"
                        }
                        LandOperationTransitionError::AttemptIdReused => "attempt_id_reused",
                        LandOperationTransitionError::ExternalOutcomeStillUnknown => {
                            "external_outcome_still_unknown"
                        }
                    });
                    break;
                }
            }

            let phase = serde_json::to_value(&value.phase)
                .expect("lifecycle phase must serialize")["state"]
                .as_str()
                .expect("lifecycle phase must expose a state")
                .to_owned();

            assert_eq!(
                phase,
                scenario.expected_phase,
                "unexpected phase for {} (source {})",
                scenario.scenario_id,
                scenario.source_scenario_id
            );
            assert_eq!(
                observed_error,
                scenario.expected_error.as_deref(),
                "unexpected error for {}",
                scenario.scenario_id
            );
            assert_eq!(
                value.allows_authoritative_mutation(),
                scenario.expected_authoritative_mutation,
                "mutation boundary changed for {}",
                scenario.scenario_id
            );
            assert!(!value.allows_authoritative_mutation());
            assert_eq!(
                value.validate_structural(),
                Ok(()),
                "invalid final lifecycle structure for {}",
                scenario.scenario_id
            );
        }
    }

    #[test]
    fn prepared_operation_is_structurally_valid_and_non_authoritative() {
        let value = lifecycle();
        assert_eq!(value.phase, LandOperationPhase::Prepared);
        assert_eq!(value.validate_structural(), Ok(()));
        assert!(!value.allows_authoritative_mutation());
        assert!(!value.may_record_external_status_projection());
    }

    #[test]
    fn operation_requires_a_content_digest() {
        assert_eq!(
            LandOperationLifecycleV0::new("urn:op:1", "not-a-digest"),
            Err(LandOperationTransitionError::InvalidInput)
        );
    }

    #[test]
    fn submission_records_a_stable_idempotency_key() {
        let mut value = lifecycle();
        submit(&mut value);
        assert_eq!(
            value.idempotency_key.as_deref(),
            Some("stable-operation-key-001")
        );
        assert_eq!(value.attempt_history.len(), 1);
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn timeout_moves_submission_to_outcome_unknown() {
        let mut value = lifecycle();
        submit(&mut value);
        value
            .apply(LandOperationCommand::MarkOutcomeUnknown)
            .expect("timeout state must be recordable");
        assert!(matches!(&value.phase, LandOperationPhase::OutcomeUnknown { .. }));
        assert!(value.requires_reconciliation());
        assert_eq!(value.attempt_history[0].status, SubmissionAttemptStatus::OutcomeUnknown);
    }

    #[test]
    fn unknown_outcome_blocks_blind_retry() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        assert_eq!(
            value.apply(LandOperationCommand::Submit {
                attempt_id: "attempt-2".into(),
                idempotency_key: "stable-operation-key-001".into(),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );
        assert!(matches!(&value.phase, LandOperationPhase::OutcomeUnknown { .. }));
    }

    #[test]
    fn still_unknown_reconciliation_does_not_unlock_retry() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        assert_eq!(
            value.apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::StillUnknown,
            }),
            Err(LandOperationTransitionError::ExternalOutcomeStillUnknown)
        );
        assert!(matches!(&value.phase, LandOperationPhase::OutcomeUnknown { .. }));
    }

    #[test]
    fn confirmed_non_commit_with_safe_same_key_semantics_unlocks_retry() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:001".into(),
                    same_idempotency_key_retry_supported: true,
                    retry_semantics_reference: Some(
                        "urn:synthetic:endpoint-contract:idempotency-v1".into(),
                    ),
                },
            })
            .unwrap();
        assert!(matches!(&value.phase, LandOperationPhase::RetryEligible { .. }));
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn retry_requires_an_explicit_idempotency_semantics_reference() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        assert_eq!(
            value.apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:semantics-001".into(),
                    same_idempotency_key_retry_supported: true,
                    retry_semantics_reference: None,
                },
            }),
            Err(LandOperationTransitionError::InvalidInput)
        );
        assert!(matches!(
            &value.phase,
            LandOperationPhase::OutcomeUnknown { .. }
        ));
        assert_eq!(
            value.validate_structural(),
            Ok(()),
            "a rejected reconciliation command must not partially mutate lifecycle state"
        );
    }

    #[test]
    fn confirmed_non_commit_without_safe_retry_semantics_holds() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:002".into(),
                    same_idempotency_key_retry_supported: false,
                    retry_semantics_reference: None,
                },
            })
            .unwrap();
        assert!(matches!(&value.phase, LandOperationPhase::Held { .. }));
        assert!(value.requires_reconciliation());
    }

    #[test]
    fn retry_must_reuse_the_original_idempotency_key() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:003".into(),
                    same_idempotency_key_retry_supported: true,
                    retry_semantics_reference: Some(
                        "urn:synthetic:endpoint-contract:idempotency-v1".into(),
                    ),
                },
            })
            .unwrap();
        assert_eq!(
            value.apply(LandOperationCommand::Submit {
                attempt_id: "attempt-2".into(),
                idempotency_key: "different-key".into(),
            }),
            Err(LandOperationTransitionError::IdempotencyKeyChanged)
        );
    }

    #[test]
    fn retry_cannot_reuse_a_prior_attempt_id() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:004".into(),
                    same_idempotency_key_retry_supported: true,
                    retry_semantics_reference: Some(
                        "urn:synthetic:endpoint-contract:idempotency-v1".into(),
                    ),
                },
            })
            .unwrap();
        assert_eq!(
            value.apply(LandOperationCommand::Submit {
                attempt_id: "attempt-1".into(),
                idempotency_key: "stable-operation-key-001".into(),
            }),
            Err(LandOperationTransitionError::AttemptIdReused)
        );
    }

    #[test]
    fn reconciled_retry_adds_a_second_attempt_without_erasing_history() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedNotCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:005".into(),
                    same_idempotency_key_retry_supported: true,
                    retry_semantics_reference: Some(
                        "urn:synthetic:endpoint-contract:idempotency-v1".into(),
                    ),
                },
            })
            .unwrap();
        value
            .apply(LandOperationCommand::Submit {
                attempt_id: "attempt-2".into(),
                idempotency_key: "stable-operation-key-001".into(),
            })
            .unwrap();
        assert_eq!(value.attempt_history.len(), 2);
        assert_eq!(value.attempt_history[0].status, SubmissionAttemptStatus::ConfirmedNotCommitted);
        assert_eq!(value.attempt_history[1].status, SubmissionAttemptStatus::Submitted);
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn reconciliation_that_finds_a_commit_records_the_receipt_without_retry() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::ConfirmedCommitted {
                    reconciliation_reference: "urn:synthetic:reconciliation:006".into(),
                    receipt_reference: "urn:synthetic:receipt:006".into(),
                    receipt_sha256: "d".repeat(64),
                    captured_at: "2026-11-19T04:05:00Z".into(),
                },
            })
            .unwrap();
        assert!(matches!(&value.phase, LandOperationPhase::ReceiptObserved { .. }));
        assert_eq!(value.attempt_history.len(), 1);
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn conflict_during_reconciliation_holds_the_operation() {
        let mut value = lifecycle();
        submit(&mut value);
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        value
            .apply(LandOperationCommand::ReconcileUnknown {
                result: ReconciliationResult::Conflict {
                    reconciliation_reference: "urn:synthetic:reconciliation:007".into(),
                    reason: "external operation status conflicts with local receipt lookup".into(),
                },
            })
            .unwrap();
        assert!(matches!(&value.phase, LandOperationPhase::Held { .. }));
        assert_eq!(
            value.attempt_history[0].status,
            SubmissionAttemptStatus::ReconciliationConflict
        );
    }

    #[test]
    fn malformed_receipt_is_rejected_without_advancing_state() {
        let mut value = lifecycle();
        submit(&mut value);
        assert_eq!(
            value.apply(LandOperationCommand::ObserveReceipt {
                receipt_reference: "urn:synthetic:receipt:bad".into(),
                receipt_sha256: "bad-digest".into(),
                captured_at: "not-a-time".into(),
            }),
            Err(LandOperationTransitionError::InvalidInput)
        );
        assert!(matches!(&value.phase, LandOperationPhase::Submitted { .. }));
    }

    #[test]
    fn receipt_must_exist_before_external_status_can_be_qualified() {
        let mut value = lifecycle();
        submit(&mut value);
        assert_eq!(
            value.apply(LandOperationCommand::QualifyExternalStatus {
                authority_profile_reference: "urn:authority".into(),
                qualification_reference: "urn:qualification".into(),
                verifier_reference: "urn:verifier".into(),
                qualification_sha256: "e".repeat(64),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );
    }

    #[test]
    fn external_status_qualification_requires_verifier_and_digest_references() {
        let mut value = lifecycle();
        receipt(&mut value);
        assert_eq!(
            value.apply(LandOperationCommand::QualifyExternalStatus {
                authority_profile_reference: "urn:authority".into(),
                qualification_reference: "urn:qualification".into(),
                verifier_reference: String::new(),
                qualification_sha256: "e".repeat(64),
            }),
            Err(LandOperationTransitionError::InvalidInput)
        );
    }

    #[test]
    fn only_qualified_external_status_can_be_projected() {
        let mut value = lifecycle();
        receipt(&mut value);
        assert_eq!(
            value.apply(LandOperationCommand::ProjectExternalStatus {
                projection_reference: "urn:projection".into(),
                projection_scope_reference: "urn:scope".into(),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );
        qualify(&mut value);
        assert!(value.may_record_external_status_projection());
    }

    #[test]
    fn external_status_projection_remains_non_authoritative_and_scoped() {
        let mut value = lifecycle();
        qualify(&mut value);
        value
            .apply(LandOperationCommand::ProjectExternalStatus {
                projection_reference: "urn:synthetic:projection:001".into(),
                projection_scope_reference: "urn:synthetic:scope:mineral-right-status".into(),
            })
            .unwrap();
        assert!(matches!(&value.phase, LandOperationPhase::ExternalStatusProjected { .. }));
        assert!(!value.allows_authoritative_mutation());
        assert!(!value.may_record_external_status_projection());
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn projection_retains_receipt_authority_and_verifier_lineage() {
        let mut value = lifecycle();
        qualify(&mut value);
        value
            .apply(LandOperationCommand::ProjectExternalStatus {
                projection_reference: "urn:synthetic:projection:002".into(),
                projection_scope_reference: "urn:synthetic:scope:permit-status".into(),
            })
            .unwrap();
        match &value.phase {
            LandOperationPhase::ExternalStatusProjected {
                authority_profile_reference,
                qualification_reference,
                verifier_reference,
                qualification_sha256,
                ..
            } => {
                assert_eq!(
                    authority_profile_reference,
                    "urn:synthetic:authority-profile:001"
                );
                assert_eq!(qualification_reference, "urn:synthetic:qualification:001");
                assert_eq!(verifier_reference, "urn:synthetic:verifier:001");
                assert_eq!(qualification_sha256, &"c".repeat(64));
            }
            other => panic!("unexpected lifecycle phase: {other:?}"),
        }
        assert!(value.external_status_projection_is_current());
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn later_correction_or_challenge_invalidates_projected_status() {
        let mut value = lifecycle();
        qualify(&mut value);
        value
            .apply(LandOperationCommand::ProjectExternalStatus {
                projection_reference: "urn:synthetic:projection:003".into(),
                projection_scope_reference: "urn:synthetic:scope:water-permit-status".into(),
            })
            .unwrap();
        value
            .apply(LandOperationCommand::InvalidateExternalStatusProjection {
                invalidation_evidence_reference: "urn:synthetic:late-challenge:003".into(),
                reason: "a later challenge changes the current evidence frontier".into(),
            })
            .unwrap();
        assert!(matches!(
            &value.phase,
            LandOperationPhase::ExternalStatusInvalidated { .. }
        ));
        assert!(!value.external_status_projection_is_current());
        assert!(value.requires_reconciliation());
        assert!(!value.allows_authoritative_mutation());
        assert_eq!(value.validate_structural(), Ok(()));
    }

    #[test]
    fn invalidation_cannot_be_applied_before_a_projection_exists() {
        let mut value = lifecycle();
        assert_eq!(
            value.apply(LandOperationCommand::InvalidateExternalStatusProjection {
                invalidation_evidence_reference: "urn:synthetic:challenge:early".into(),
                reason: "challenge before projection".into(),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );
    }

    #[test]
    fn hold_and_reject_are_closed_fail_safe_phases() {
        let mut held = lifecycle();
        submit(&mut held);
        held.apply(LandOperationCommand::Hold {
            reason: "authority currentness is unknown".into(),
            decision_packet_reference: Some("urn:packet:001".into()),
        })
        .unwrap();
        assert_eq!(
            held.apply(LandOperationCommand::Submit {
                attempt_id: "attempt-2".into(),
                idempotency_key: "stable-operation-key-001".into(),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );

        let mut rejected = lifecycle();
        rejected
            .apply(LandOperationCommand::Reject {
                reason: "wrong-jurisdiction authority reference".into(),
            })
            .unwrap();
        assert_eq!(
            rejected.apply(LandOperationCommand::Submit {
                attempt_id: "attempt-1".into(),
                idempotency_key: "stable-operation-key-001".into(),
            }),
            Err(LandOperationTransitionError::InvalidTransition)
        );
    }

    #[test]
    fn lifecycle_schema_document_is_valid_json_and_has_the_expected_id() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../schemas/land-operation-lifecycle-v0.schema.json"
        ))
        .expect("lifecycle schema must parse");
        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(schema["$id"], LAND_OPERATION_LIFECYCLE_SCHEMA_URI);
    }

    #[test]
    fn synthetic_timeout_unknown_example_is_valid_and_blocks_reconciliation() {
        let value: LandOperationLifecycleV0 = serde_json::from_str(include_str!(
            "../../../examples/land-operation-lifecycle-timeout-unknown-v0.json"
        ))
        .expect("timeout-unknown example must deserialize");
        assert_eq!(value.validate_structural(), Ok(()));
        assert!(value.requires_reconciliation());
        assert!(!value.allows_authoritative_mutation());
        assert_eq!(value.attempt_history.len(), 1);
    }

    #[test]
    fn no_phase_in_this_reference_model_authorizes_authoritative_mutation() {
        let mut value = lifecycle();
        assert!(!value.allows_authoritative_mutation());
        submit(&mut value);
        assert!(!value.allows_authoritative_mutation());
        value.apply(LandOperationCommand::MarkOutcomeUnknown).unwrap();
        assert!(!value.allows_authoritative_mutation());
        value
            .apply(LandOperationCommand::Hold {
                reason: "test hold".into(),
                decision_packet_reference: None,
            })
            .unwrap();
        assert!(!value.allows_authoritative_mutation());
    }
}
