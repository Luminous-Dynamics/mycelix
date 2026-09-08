// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure canonical actuator-execution semantics shared by descendant execution
//! adapters.
//!
//! This crate contains no authority typestate constructor, trusted-time
//! qualifier call, journal/store write, actuator reservation call, or actuator
//! invocation. It evaluates already supplied evidence using one provider-neutral
//! semantic vocabulary. Execution adapters map these results into their public
//! evidence types.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::{
    validate_actuator_reservation, ActuatorEffectOutcomeCommitment,
    ActuatorEffectReceiptCommitment, ActuatorEffectReceiptV1,
    ActuatorInvocationAttemptManifestV1, ActuatorNonApplicationEvidenceCommitment,
    ActuatorReservationReceiptV1, ActuatorReservationValidationError,
    EffectRecoveryPolicyV1, ValidatedActuatorEffectDispositionV1,
    ValidatedActuatorEffectOutcomeV1, ValidatedActuatorReservationV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReservationSemanticErrorV1 {
    ReceiptAlreadyExpiredAtFinalActivation,
    Validation(ActuatorReservationValidationError),
}

pub struct CanonicalValidatedReservationSemanticsV1 {
    reservation: ValidatedActuatorReservationV1,
    valid_until: u64,
}

impl CanonicalValidatedReservationSemanticsV1 {
    pub const fn reservation(&self) -> &ValidatedActuatorReservationV1 {
        &self.reservation
    }

    pub fn into_reservation(self) -> ValidatedActuatorReservationV1 {
        self.reservation
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

/// Validate an already returned reservation receipt at the final activation
/// time. Descriptor-before/after and external-call errors remain adapter-owned
/// facts because a pure helper cannot establish them.
pub fn validate_reservation_semantics(
    final_activation_latest_possible_unix_ms: u64,
    final_ready_valid_until: u64,
    attempt: ActuatorInvocationAttemptManifestV1,
    receipt: ActuatorReservationReceiptV1,
) -> Result<CanonicalValidatedReservationSemanticsV1, CanonicalReservationSemanticErrorV1> {
    if receipt.valid_until < final_activation_latest_possible_unix_ms {
        return Err(CanonicalReservationSemanticErrorV1::ReceiptAlreadyExpiredAtFinalActivation);
    }

    let reservation = validate_actuator_reservation(attempt.subject.actuator, attempt, receipt)
        .map_err(CanonicalReservationSemanticErrorV1::Validation)?;
    let valid_until = final_ready_valid_until.min(receipt.valid_until);

    Ok(CanonicalValidatedReservationSemanticsV1 {
        reservation,
        valid_until,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalPreInvocationSemanticErrorV1 {
    TimeRegressedBeforeFinalActivation,
    ReservedLineageAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    ReservationAlreadyExpired,
}

pub const fn pre_invocation_error_from_ceilings(
    final_activation_latest_possible_unix_ms: u64,
    reserved_valid_until: u64,
    attempt_valid_until: u64,
    actuator_valid_until: u64,
    reservation_valid_until: u64,
    pre_invocation_latest_possible_unix_ms: u64,
) -> Option<CanonicalPreInvocationSemanticErrorV1> {
    let latest = pre_invocation_latest_possible_unix_ms;
    if latest < final_activation_latest_possible_unix_ms {
        Some(CanonicalPreInvocationSemanticErrorV1::TimeRegressedBeforeFinalActivation)
    } else if reserved_valid_until < latest {
        Some(CanonicalPreInvocationSemanticErrorV1::ReservedLineageAlreadyExpired)
    } else if attempt_valid_until < latest {
        Some(CanonicalPreInvocationSemanticErrorV1::AttemptAlreadyExpired)
    } else if actuator_valid_until < latest {
        Some(CanonicalPreInvocationSemanticErrorV1::ActuatorGenerationAlreadyExpired)
    } else if reservation_valid_until < latest {
        Some(CanonicalPreInvocationSemanticErrorV1::ReservationAlreadyExpired)
    } else {
        None
    }
}

pub const fn pre_invocation_error(
    final_activation_latest_possible_unix_ms: u64,
    reserved_valid_until: u64,
    attempt: ActuatorInvocationAttemptManifestV1,
    reservation_receipt: ActuatorReservationReceiptV1,
    pre_invocation_latest_possible_unix_ms: u64,
) -> Option<CanonicalPreInvocationSemanticErrorV1> {
    pre_invocation_error_from_ceilings(
        final_activation_latest_possible_unix_ms,
        reserved_valid_until,
        attempt.attempt_valid_until,
        attempt.subject.actuator.valid_until,
        reservation_receipt.valid_until,
        pre_invocation_latest_possible_unix_ms,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalPostInvocationAmbiguityV1 {
    PostInvocationTimeQualificationFailed,
    PostInvocationTimeRegressed,
    ReservedLineageExpiredDuringInvocation,
    AttemptExpiredDuringInvocation,
    ActuatorGenerationExpiredDuringInvocation,
    ReservationExpiredDuringInvocation,
    EffectReceiptExpiredAtPostInvocationTime,
}

pub const fn post_invocation_ambiguity_from_ceilings(
    reserved_valid_until: u64,
    attempt_valid_until: u64,
    actuator_valid_until: u64,
    reservation_valid_until: u64,
    pre_invocation_latest_possible_unix_ms: u64,
    post_invocation_latest_possible_unix_ms: u64,
    effect_receipt_valid_until: Option<u64>,
) -> Option<CanonicalPostInvocationAmbiguityV1> {
    let post = post_invocation_latest_possible_unix_ms;
    if post < pre_invocation_latest_possible_unix_ms {
        return Some(CanonicalPostInvocationAmbiguityV1::PostInvocationTimeRegressed);
    }
    if reserved_valid_until < post {
        return Some(CanonicalPostInvocationAmbiguityV1::ReservedLineageExpiredDuringInvocation);
    }
    if attempt_valid_until < post {
        return Some(CanonicalPostInvocationAmbiguityV1::AttemptExpiredDuringInvocation);
    }
    if actuator_valid_until < post {
        return Some(CanonicalPostInvocationAmbiguityV1::ActuatorGenerationExpiredDuringInvocation);
    }
    if reservation_valid_until < post {
        return Some(CanonicalPostInvocationAmbiguityV1::ReservationExpiredDuringInvocation);
    }
    if let Some(valid_until) = effect_receipt_valid_until {
        if valid_until < post {
            return Some(CanonicalPostInvocationAmbiguityV1::EffectReceiptExpiredAtPostInvocationTime);
        }
    }
    None
}

pub fn post_invocation_ambiguity(
    reserved_valid_until: u64,
    attempt: ActuatorInvocationAttemptManifestV1,
    reservation_receipt: ActuatorReservationReceiptV1,
    pre_invocation_latest_possible_unix_ms: u64,
    post_invocation_latest_possible_unix_ms: u64,
    effect_receipt: Option<&ActuatorEffectReceiptV1>,
) -> Option<CanonicalPostInvocationAmbiguityV1> {
    post_invocation_ambiguity_from_ceilings(
        reserved_valid_until,
        attempt.attempt_valid_until,
        attempt.subject.actuator.valid_until,
        reservation_receipt.valid_until,
        pre_invocation_latest_possible_unix_ms,
        post_invocation_latest_possible_unix_ms,
        effect_receipt.map(|receipt| receipt.valid_until),
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalExecutionQualifiedDispositionV1 {
    Confirmed {
        outcome: ActuatorEffectOutcomeCommitment,
        actuator_receipt: ActuatorEffectReceiptCommitment,
    },
    ProvenNotApplied {
        evidence: ActuatorNonApplicationEvidenceCommitment,
        actuator_receipt: ActuatorEffectReceiptCommitment,
    },
    OutcomeUnknown {
        low_level_disposition: ValidatedActuatorEffectDispositionV1,
        qualified_actuator_receipt: Option<ActuatorEffectReceiptCommitment>,
        post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityV1>,
        recovery_policy: EffectRecoveryPolicyV1,
    },
}

impl CanonicalExecutionQualifiedDispositionV1 {
    pub const fn is_terminal(&self) -> bool {
        matches!(Self::Confirmed { .. } | Self::ProvenNotApplied { .. }, self)
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

/// Derive stricter canonical classification without rewriting raw actuator
/// evidence or the low-level validated outcome.
pub fn qualified_effect_disposition(
    low_level: ValidatedActuatorEffectOutcomeV1,
    post_ambiguity: Option<CanonicalPostInvocationAmbiguityV1>,
) -> CanonicalExecutionQualifiedDispositionV1 {
    let qualified_receipt = low_level.receipt().map(|receipt| receipt.receipt_commitment);

    if post_ambiguity.is_some() {
        return CanonicalExecutionQualifiedDispositionV1::OutcomeUnknown {
            low_level_disposition: low_level.disposition(),
            qualified_actuator_receipt: qualified_receipt,
            post_invocation_ambiguity: post_ambiguity,
            recovery_policy: low_level.recovery_policy(),
        };
    }

    match low_level.disposition() {
        ValidatedActuatorEffectDispositionV1::Confirmed { outcome } => match qualified_receipt {
            Some(actuator_receipt) => CanonicalExecutionQualifiedDispositionV1::Confirmed {
                outcome,
                actuator_receipt,
            },
            None => CanonicalExecutionQualifiedDispositionV1::OutcomeUnknown {
                low_level_disposition: low_level.disposition(),
                qualified_actuator_receipt: None,
                post_invocation_ambiguity: None,
                recovery_policy: low_level.recovery_policy(),
            },
        },
        ValidatedActuatorEffectDispositionV1::ProvenNotApplied { evidence } => {
            match qualified_receipt {
                Some(actuator_receipt) => CanonicalExecutionQualifiedDispositionV1::ProvenNotApplied {
                    evidence,
                    actuator_receipt,
                },
                None => CanonicalExecutionQualifiedDispositionV1::OutcomeUnknown {
                    low_level_disposition: low_level.disposition(),
                    qualified_actuator_receipt: None,
                    post_invocation_ambiguity: None,
                    recovery_policy: low_level.recovery_policy(),
                },
            }
        }
        ValidatedActuatorEffectDispositionV1::OutcomeUnknown { .. } => {
            CanonicalExecutionQualifiedDispositionV1::OutcomeUnknown {
                low_level_disposition: low_level.disposition(),
                qualified_actuator_receipt: qualified_receipt,
                post_invocation_ambiguity: None,
                recovery_policy: low_level.recovery_policy(),
            }
        }
    }
}

pub const fn kernel_creates_effect_authority() -> bool {
    false
}

pub const fn kernel_creates_replay_authority() -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pre_invocation_rollback_fails_closed() {
        assert_eq!(
            pre_invocation_error_from_ceilings(100, 130, 130, 130, 130, 99),
            Some(CanonicalPreInvocationSemanticErrorV1::TimeRegressedBeforeFinalActivation)
        );
    }

    #[test]
    fn pre_invocation_ceiling_expiry_is_distinguished() {
        assert_eq!(
            pre_invocation_error_from_ceilings(100, 105, 120, 120, 120, 106),
            Some(CanonicalPreInvocationSemanticErrorV1::ReservedLineageAlreadyExpired)
        );
    }

    #[test]
    fn post_invocation_expiry_is_ambiguity() {
        assert_eq!(
            post_invocation_ambiguity_from_ceilings(105, 120, 120, 120, 100, 106, None),
            Some(CanonicalPostInvocationAmbiguityV1::ReservedLineageExpiredDuringInvocation)
        );
    }

    #[test]
    fn post_invocation_receipt_expiry_is_distinct() {
        assert_eq!(
            post_invocation_ambiguity_from_ceilings(130, 130, 130, 130, 100, 110, Some(109)),
            Some(CanonicalPostInvocationAmbiguityV1::EffectReceiptExpiredAtPostInvocationTime)
        );
    }

    #[test]
    fn kernel_is_non_authorizing() {
        assert!(!kernel_creates_effect_authority());
        assert!(!kernel_creates_replay_authority());
    }
}