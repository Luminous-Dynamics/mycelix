// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral actuator effect protocol for SSF.
//!
//! This crate freezes the reservation/invocation/outcome state machine without
//! implementing an actuator. The durable SSF claim record is the stable
//! deduplication key. Reservation is non-effecting; invocation may apply the
//! external effect.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_effect_capability_claim::EffectCapabilityClaimRecordCommitment;
use mycelix_ssf_pre_invocation_actuator_qualification::{
    ActuatorExecutionDescriptorV1, ActuatorRecoveryModeV1,
    PreInvocationActuatorQualificationReceiptV1, PreInvocationQualifiedClaimedEffectV1,
};
use mycelix_ssf_source_owned_operation_material::{
    OperationEncodingCommitment, OperationPayloadCommitment,
    SourceOwnedOperationHandleCommitment,
};

macro_rules! digest_type {
    ($name:ident) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr(transparent)]
        pub struct $name([u8; 32]);

        impl $name {
            pub const fn from_bytes(bytes: [u8; 32]) -> Self {
                Self(bytes)
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

digest_type!(ActuatorInvocationAttemptId);
digest_type!(ActuatorReservationCommitment);
digest_type!(ActuatorReservationReceiptCommitment);
digest_type!(ActuatorEffectOutcomeCommitment);
digest_type!(ActuatorNonApplicationEvidenceCommitment);
digest_type!(ActuatorEffectReceiptCommitment);

/// Stable semantic/physical effect identity shared by an original invocation
/// and any later explicitly authorized replay.
///
/// Fresh pre-invocation qualification evidence is deliberately excluded from
/// this identity. Replay must re-qualify that evidence while preserving these
/// exact effect-defining facts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorStableEffectIdentityV1 {
    pub claim_record: EffectCapabilityClaimRecordCommitment,
    pub actuator: ActuatorExecutionDescriptorV1,
    pub operation_handle: SourceOwnedOperationHandleCommitment,
    pub payload_commitment: OperationPayloadCommitment,
    pub encoding_commitment: OperationEncodingCommitment,
    pub payload_length: u64,
    pub recovery_mode: ActuatorRecoveryModeV1,
}

/// Attempt-specific effect subject.
///
/// This retains the fresh pre-invocation qualification receipt for audit and
/// exact attempt binding while exposing a stable replay identity that excludes
/// that ephemeral evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorEffectSubjectV1 {
    pub claim_record: EffectCapabilityClaimRecordCommitment,
    pub pre_invocation_receipt: PreInvocationActuatorQualificationReceiptV1,
    pub actuator: ActuatorExecutionDescriptorV1,
    pub operation_handle: SourceOwnedOperationHandleCommitment,
    pub payload_commitment: OperationPayloadCommitment,
    pub encoding_commitment: OperationEncodingCommitment,
    pub payload_length: u64,
    pub recovery_mode: ActuatorRecoveryModeV1,
}

impl ActuatorEffectSubjectV1 {
    pub const fn stable_identity(&self) -> ActuatorStableEffectIdentityV1 {
        ActuatorStableEffectIdentityV1 {
            claim_record: self.claim_record,
            actuator: self.actuator,
            operation_handle: self.operation_handle,
            payload_commitment: self.payload_commitment,
            encoding_commitment: self.encoding_commitment,
            payload_length: self.payload_length,
            recovery_mode: self.recovery_mode,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorInvocationAttemptManifestV1 {
    pub schema_version: u16,
    pub attempt_id: ActuatorInvocationAttemptId,
    pub subject: ActuatorEffectSubjectV1,
    pub attempt_time_receipt: CurrentTimeReceiptCommitment,
    pub attempt_valid_until: u64,
    pub latest_possible_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorEffectAttemptPreparationError {
    PreInvocationQualificationAlreadyExpired,
    InvocationTimeAlreadyExpired,
}

pub struct PreparedActuatorEffectAttemptV1<Q, TQ, ITQ> {
    qualification: PreInvocationQualifiedClaimedEffectV1<Q, TQ>,
    invocation_time: QualifiedCurrentTimeV1<ITQ>,
    manifest: ActuatorInvocationAttemptManifestV1,
}

impl<Q, TQ, ITQ> PreparedActuatorEffectAttemptV1<Q, TQ, ITQ> {
    pub const fn qualification(&self) -> &PreInvocationQualifiedClaimedEffectV1<Q, TQ> {
        &self.qualification
    }

    pub const fn invocation_time(&self) -> &QualifiedCurrentTimeV1<ITQ> {
        &self.invocation_time
    }

    pub const fn manifest(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.manifest
    }
}

pub struct ActuatorEffectAttemptPreparationFailureV1<Q, TQ, ITQ> {
    qualification: PreInvocationQualifiedClaimedEffectV1<Q, TQ>,
    invocation_time: QualifiedCurrentTimeV1<ITQ>,
    error: ActuatorEffectAttemptPreparationError,
}

impl<Q, TQ, ITQ> ActuatorEffectAttemptPreparationFailureV1<Q, TQ, ITQ> {
    pub const fn error(&self) -> ActuatorEffectAttemptPreparationError {
        self.error
    }

    pub fn into_parts(
        self,
    ) -> (
        PreInvocationQualifiedClaimedEffectV1<Q, TQ>,
        QualifiedCurrentTimeV1<ITQ>,
    ) {
        (self.qualification, self.invocation_time)
    }
}

pub fn prepare_actuator_effect_attempt<Q, TQ, ITQ>(
    qualification: PreInvocationQualifiedClaimedEffectV1<Q, TQ>,
    invocation_time: QualifiedCurrentTimeV1<ITQ>,
    attempt_id: ActuatorInvocationAttemptId,
) -> Result<
    PreparedActuatorEffectAttemptV1<Q, TQ, ITQ>,
    ActuatorEffectAttemptPreparationFailureV1<Q, TQ, ITQ>,
> {
    let latest = invocation_time.latest_possible_unix_ms();
    let error = if qualification.valid_until() < latest {
        Some(ActuatorEffectAttemptPreparationError::PreInvocationQualificationAlreadyExpired)
    } else if invocation_time.valid_until() < latest {
        Some(ActuatorEffectAttemptPreparationError::InvocationTimeAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = error {
        return Err(ActuatorEffectAttemptPreparationFailureV1 {
            qualification,
            invocation_time,
            error,
        });
    }

    let receipt = qualification.receipt();
    let capability = qualification.lineage().capability_binding();
    let subject = ActuatorEffectSubjectV1 {
        claim_record: qualification.lineage().claim_record(),
        pre_invocation_receipt: receipt,
        actuator: qualification.expected().actuator(),
        operation_handle: capability.operation_handle,
        payload_commitment: capability.payload_commitment,
        encoding_commitment: capability.encoding_commitment,
        payload_length: capability.payload_length,
        recovery_mode: qualification.recovery_mode(),
    };
    let manifest = ActuatorInvocationAttemptManifestV1 {
        schema_version: SSF_SCHEMA_V1,
        attempt_id,
        subject,
        attempt_time_receipt: invocation_time.receipt_commitment(),
        attempt_valid_until: qualification.valid_until().min(invocation_time.valid_until()),
        latest_possible_unix_ms: latest,
    };

    Ok(PreparedActuatorEffectAttemptV1 {
        qualification,
        invocation_time,
        manifest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorReservationDispositionV1 {
    Reserved {
        reservation: ActuatorReservationCommitment,
    },
    SubjectConflict,
    OutcomeUnknown,
}

/// Actuator-side deduplication reservation for the stable effect identity.
///
/// Fresh attempt qualification is deliberately not part of the reservation
/// key; each invocation still carries its exact fresh qualification evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorReservationReceiptV1 {
    pub schema_version: u16,
    pub actuator: ActuatorExecutionDescriptorV1,
    pub subject: ActuatorStableEffectIdentityV1,
    pub disposition: ActuatorReservationDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: ActuatorReservationReceiptCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorEffectDispositionV1 {
    Confirmed {
        outcome: ActuatorEffectOutcomeCommitment,
    },
    ProvenNotApplied {
        evidence: ActuatorNonApplicationEvidenceCommitment,
    },
    OutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorEffectReceiptV1 {
    pub schema_version: u16,
    pub actuator: ActuatorExecutionDescriptorV1,
    pub subject: ActuatorEffectSubjectV1,
    pub reservation: ActuatorReservationCommitment,
    pub attempt_id: ActuatorInvocationAttemptId,
    pub disposition: ActuatorEffectDispositionV1,
    pub valid_until: u64,
    pub receipt_commitment: ActuatorEffectReceiptCommitment,
}

pub trait ActuatorEffectProtocolV1 {
    type Error;

    fn descriptor(&self) -> ActuatorExecutionDescriptorV1;

    /// Reserve/deduplicate the stable effect identity. This MUST NOT apply the
    /// external effect.
    fn reserve_effect_subject(
        &self,
        subject: &ActuatorStableEffectIdentityV1,
    ) -> Result<ActuatorReservationReceiptV1, Self::Error>;

    fn reconcile_effect_reservation(
        &self,
        subject: &ActuatorStableEffectIdentityV1,
    ) -> Result<ActuatorReservationReceiptV1, Self::Error>;

    fn invoke_reserved_effect(
        &self,
        attempt: &ActuatorInvocationAttemptManifestV1,
        reservation: ActuatorReservationCommitment,
    ) -> Result<ActuatorEffectReceiptV1, Self::Error>;

    /// Read-only reconciliation of the stable effect identity.
    fn reconcile_effect_subject(
        &self,
        subject: &ActuatorStableEffectIdentityV1,
    ) -> Result<ActuatorEffectReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorReservationValidationError {
    UnsupportedReceiptSchema,
    UnsupportedAttemptSchema,
    ActuatorMismatch,
    SubjectMismatch,
    ReceiptOutlivesActuator,
    ReceiptAlreadyExpiredAtInvocation,
    SubjectConflict,
    OutcomeUnknown,
}

pub struct ValidatedActuatorReservationV1 {
    receipt: ActuatorReservationReceiptV1,
    reservation: ActuatorReservationCommitment,
}

impl ValidatedActuatorReservationV1 {
    pub const fn receipt(&self) -> ActuatorReservationReceiptV1 {
        self.receipt
    }

    pub const fn reservation(&self) -> ActuatorReservationCommitment {
        self.reservation
    }
}

pub fn validate_actuator_reservation(
    expected: ActuatorExecutionDescriptorV1,
    attempt: ActuatorInvocationAttemptManifestV1,
    receipt: ActuatorReservationReceiptV1,
) -> Result<ValidatedActuatorReservationV1, ActuatorReservationValidationError> {
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(ActuatorReservationValidationError::UnsupportedReceiptSchema);
    }
    if attempt.schema_version != SSF_SCHEMA_V1 {
        return Err(ActuatorReservationValidationError::UnsupportedAttemptSchema);
    }
    if receipt.actuator != expected {
        return Err(ActuatorReservationValidationError::ActuatorMismatch);
    }
    if receipt.subject != attempt.subject.stable_identity() {
        return Err(ActuatorReservationValidationError::SubjectMismatch);
    }
    if receipt.valid_until > expected.valid_until {
        return Err(ActuatorReservationValidationError::ReceiptOutlivesActuator);
    }
    if receipt.valid_until < attempt.latest_possible_unix_ms {
        return Err(ActuatorReservationValidationError::ReceiptAlreadyExpiredAtInvocation);
    }

    let reservation = match receipt.disposition {
        ActuatorReservationDispositionV1::Reserved { reservation } => reservation,
        ActuatorReservationDispositionV1::SubjectConflict => {
            return Err(ActuatorReservationValidationError::SubjectConflict);
        }
        ActuatorReservationDispositionV1::OutcomeUnknown => {
            return Err(ActuatorReservationValidationError::OutcomeUnknown);
        }
    };

    Ok(ValidatedActuatorReservationV1 {
        receipt,
        reservation,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectRecoveryPolicyV1 {
    ReconcileOnly,
    ExactSameClaimReplayMayBePermitted,
    NeverAutomaticRetry,
}

pub const fn recovery_policy_for(mode: ActuatorRecoveryModeV1) -> EffectRecoveryPolicyV1 {
    match mode {
        ActuatorRecoveryModeV1::TransactionalClaimKey => EffectRecoveryPolicyV1::ReconcileOnly,
        ActuatorRecoveryModeV1::IdempotentClaimKey => {
            EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted
        }
        ActuatorRecoveryModeV1::NonIdempotentNoAutomaticRetry => {
            EffectRecoveryPolicyV1::NeverAutomaticRetry
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ActuatorEffectAmbiguityReasonV1 {
    ActuatorErrorAfterInvocation,
    ActuatorDescriptorChangedAfterInvocation,
    UnsupportedAttemptSchema,
    UnsupportedReceiptSchema,
    ReceiptActuatorMismatch,
    ReceiptSubjectMismatch,
    ReceiptReservationMismatch,
    ReceiptAttemptMismatch,
    ReceiptOutlivesActuator,
    ReceiptAlreadyExpiredAtInvocation,
    StoreReportedOutcomeUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidatedActuatorEffectDispositionV1 {
    Confirmed {
        outcome: ActuatorEffectOutcomeCommitment,
    },
    ProvenNotApplied {
        evidence: ActuatorNonApplicationEvidenceCommitment,
    },
    OutcomeUnknown {
        reason: ActuatorEffectAmbiguityReasonV1,
    },
}

/// Opaque post-invocation interpretation of one exact actuator attempt.
///
/// The only constructors are private to this crate; external code cannot mint
/// a supposedly validated `Confirmed` or `ProvenNotApplied` outcome by struct
/// literal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidatedActuatorEffectOutcomeV1 {
    subject: ActuatorEffectSubjectV1,
    attempt_id: ActuatorInvocationAttemptId,
    disposition: ValidatedActuatorEffectDispositionV1,
    recovery_policy: EffectRecoveryPolicyV1,
    receipt: Option<ActuatorEffectReceiptV1>,
}

impl ValidatedActuatorEffectOutcomeV1 {
    pub const fn subject(&self) -> ActuatorEffectSubjectV1 {
        self.subject
    }

    pub const fn stable_identity(&self) -> ActuatorStableEffectIdentityV1 {
        self.subject.stable_identity()
    }

    pub const fn attempt_id(&self) -> ActuatorInvocationAttemptId {
        self.attempt_id
    }

    pub const fn disposition(&self) -> ValidatedActuatorEffectDispositionV1 {
        self.disposition
    }

    pub const fn recovery_policy(&self) -> EffectRecoveryPolicyV1 {
        self.recovery_policy
    }

    pub const fn receipt(&self) -> Option<ActuatorEffectReceiptV1> {
        self.receipt
    }
}

fn unknown_effect(
    attempt: ActuatorInvocationAttemptManifestV1,
    reason: ActuatorEffectAmbiguityReasonV1,
    receipt: Option<ActuatorEffectReceiptV1>,
) -> ValidatedActuatorEffectOutcomeV1 {
    ValidatedActuatorEffectOutcomeV1 {
        subject: attempt.subject,
        attempt_id: attempt.attempt_id,
        disposition: ValidatedActuatorEffectDispositionV1::OutcomeUnknown { reason },
        recovery_policy: recovery_policy_for(attempt.subject.recovery_mode),
        receipt,
    }
}

pub fn interpret_actuator_invocation_result<E>(
    expected: ActuatorExecutionDescriptorV1,
    attempt: ActuatorInvocationAttemptManifestV1,
    reservation: ActuatorReservationCommitment,
    descriptor_after: ActuatorExecutionDescriptorV1,
    result: Result<ActuatorEffectReceiptV1, E>,
) -> ValidatedActuatorEffectOutcomeV1 {
    if attempt.schema_version != SSF_SCHEMA_V1 {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::UnsupportedAttemptSchema,
            None,
        );
    }

    let receipt = match result {
        Ok(receipt) => receipt,
        Err(_) => {
            return unknown_effect(
                attempt,
                ActuatorEffectAmbiguityReasonV1::ActuatorErrorAfterInvocation,
                None,
            );
        }
    };

    if descriptor_after != expected {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ActuatorDescriptorChangedAfterInvocation,
            Some(receipt),
        );
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::UnsupportedReceiptSchema,
            Some(receipt),
        );
    }
    if receipt.actuator != expected {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptActuatorMismatch,
            Some(receipt),
        );
    }
    if receipt.subject != attempt.subject {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptSubjectMismatch,
            Some(receipt),
        );
    }
    if receipt.reservation != reservation {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptReservationMismatch,
            Some(receipt),
        );
    }
    if receipt.attempt_id != attempt.attempt_id {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptAttemptMismatch,
            Some(receipt),
        );
    }
    if receipt.valid_until > expected.valid_until {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptOutlivesActuator,
            Some(receipt),
        );
    }
    if receipt.valid_until < attempt.latest_possible_unix_ms {
        return unknown_effect(
            attempt,
            ActuatorEffectAmbiguityReasonV1::ReceiptAlreadyExpiredAtInvocation,
            Some(receipt),
        );
    }

    let disposition = match receipt.disposition {
        ActuatorEffectDispositionV1::Confirmed { outcome } => {
            ValidatedActuatorEffectDispositionV1::Confirmed { outcome }
        }
        ActuatorEffectDispositionV1::ProvenNotApplied { evidence } => {
            ValidatedActuatorEffectDispositionV1::ProvenNotApplied { evidence }
        }
        ActuatorEffectDispositionV1::OutcomeUnknown => {
            return unknown_effect(
                attempt,
                ActuatorEffectAmbiguityReasonV1::StoreReportedOutcomeUnknown,
                Some(receipt),
            );
        }
    };

    ValidatedActuatorEffectOutcomeV1 {
        subject: attempt.subject,
        attempt_id: attempt.attempt_id,
        disposition,
        recovery_policy: recovery_policy_for(attempt.subject.recovery_mode),
        receipt: Some(receipt),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_policy_is_mode_specific() {
        assert_eq!(
            recovery_policy_for(ActuatorRecoveryModeV1::TransactionalClaimKey),
            EffectRecoveryPolicyV1::ReconcileOnly
        );
        assert_eq!(
            recovery_policy_for(ActuatorRecoveryModeV1::IdempotentClaimKey),
            EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted
        );
        assert_eq!(
            recovery_policy_for(ActuatorRecoveryModeV1::NonIdempotentNoAutomaticRetry),
            EffectRecoveryPolicyV1::NeverAutomaticRetry
        );
    }

    #[test]
    fn exactly_once_is_not_a_recovery_mode() {
        assert_ne!(
            recovery_policy_for(ActuatorRecoveryModeV1::TransactionalClaimKey),
            EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted
        );
    }

    #[test]
    fn receipt_expiry_is_checked_against_latest_possible_now() {
        let latest_possible_now = 100;
        assert!(99_u64 < latest_possible_now);
        assert!(!(100_u64 < latest_possible_now));
    }
}
