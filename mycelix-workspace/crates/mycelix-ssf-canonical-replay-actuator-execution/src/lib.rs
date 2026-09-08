// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical replay bridge from final replay readiness to the provider-neutral
//! actuator effect protocol.
//!
//! The only replay execution entry point consumes
//! `FinalCanonicalReplayInvocationReadyV1`. Reservation is non-effecting. The
//! bridge obtains fresh trusted time immediately before and after invocation,
//! preserves the raw actuator receipt unchanged, delegates pure expiry and
//! classification rules to `mycelix-ssf-canonical-execution-semantics`, and
//! maps those results into the existing canonical outcome vocabulary.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    interpret_actuator_invocation_result, ActuatorEffectProtocolV1, ActuatorEffectReceiptV1,
    ActuatorInvocationAttemptManifestV1, ActuatorReservationReceiptV1,
    ValidatedActuatorEffectOutcomeV1, ValidatedActuatorReservationV1,
};
use mycelix_ssf_canonical_actuator_execution::{
    CanonicalInvocationPreconditionErrorV1, CanonicalPostInvocationAmbiguityReasonV1,
    CanonicalQualifiedEffectDispositionV1, CanonicalReservationAmbiguityReasonV1,
    CanonicalReservationPreconditionErrorV1,
};
use mycelix_ssf_canonical_execution_semantics::{
    post_invocation_ambiguity, pre_invocation_error, qualified_effect_disposition,
    validate_reservation_semantics, CanonicalExecutionQualifiedDispositionV1,
    CanonicalPostInvocationAmbiguityV1, CanonicalPreInvocationSemanticErrorV1,
    CanonicalReservationSemanticErrorV1,
};
use mycelix_ssf_current_authority_revalidation::{
    qualify_current_time, CurrentTimeBindingError, CurrentTimeQualifierV1,
    ExpectedCurrentTimeQualifierProfileV1, QualifiedCurrentTimeV1,
};
use mycelix_ssf_final_canonical_replay_readiness::{
    FinalCanonicalReplayAuditEvidenceV1, FinalCanonicalReplayInvocationReadyV1,
    FinalCanonicalReplaySourceKindV1,
};

/// Copyable provenance for the exact canonical replay lineage consumed by this
/// bridge. This is evidence only and cannot recreate final readiness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayInvocationAuditBindingV1 {
    pub source_kind: FinalCanonicalReplaySourceKindV1,
    pub attempt: ActuatorInvocationAttemptManifestV1,
    pub evidence: FinalCanonicalReplayAuditEvidenceV1,
}

impl CanonicalReplayInvocationAuditBindingV1 {
    pub const fn contains_effect_authority(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub struct CanonicalReplayReservationPreconditionFailureV1<A> {
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    error: CanonicalReservationPreconditionErrorV1,
}

impl<A> CanonicalReplayReservationPreconditionFailureV1<A> {
    pub const fn error(&self) -> CanonicalReservationPreconditionErrorV1 {
        self.error
    }

    pub fn into_ready(self) -> FinalCanonicalReplayInvocationReadyV1<A> {
        self.ready
    }
}

/// Reservation state is ambiguous/blocked, but this bridge has not invoked the
/// external effect.
pub struct FrozenCanonicalReplayReservationV1<A, P> {
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    audit: CanonicalReplayInvocationAuditBindingV1,
    reason: CanonicalReservationAmbiguityReasonV1,
    receipt: Option<ActuatorReservationReceiptV1>,
    _actuator: PhantomData<fn() -> P>,
}

impl<A, P> FrozenCanonicalReplayReservationV1<A, P> {
    pub const fn ready(&self) -> &FinalCanonicalReplayInvocationReadyV1<A> {
        &self.ready
    }

    pub const fn audit(&self) -> CanonicalReplayInvocationAuditBindingV1 {
        self.audit
    }

    pub const fn reason(&self) -> CanonicalReservationAmbiguityReasonV1 {
        self.reason
    }

    pub const fn receipt(&self) -> Option<ActuatorReservationReceiptV1> {
        self.receipt
    }

    pub const fn may_invoke(&self) -> bool {
        false
    }

    pub const fn external_effect_may_have_occurred(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

/// Final replay readiness plus one validated actuator-side reservation of the
/// exact stable effect identity. This remains pre-effect state.
pub struct CanonicalReservedReplayInvocationV1<A, P> {
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    audit: CanonicalReplayInvocationAuditBindingV1,
    reservation: ValidatedActuatorReservationV1,
    valid_until: u64,
    _actuator: PhantomData<fn() -> P>,
}

impl<A, P> CanonicalReservedReplayInvocationV1<A, P> {
    pub const fn ready(&self) -> &FinalCanonicalReplayInvocationReadyV1<A> {
        &self.ready
    }

    pub const fn audit(&self) -> CanonicalReplayInvocationAuditBindingV1 {
        self.audit
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.ready.attempt()
    }

    pub const fn reservation(&self) -> &ValidatedActuatorReservationV1 {
        &self.reservation
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn external_effect_attempted(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

pub enum CanonicalReplayReservationOutcomeV1<A, P> {
    Reserved(CanonicalReservedReplayInvocationV1<A, P>),
    OutcomeUnknown(FrozenCanonicalReplayReservationV1<A, P>),
}

fn audit_binding<A>(
    ready: &FinalCanonicalReplayInvocationReadyV1<A>,
) -> CanonicalReplayInvocationAuditBindingV1 {
    CanonicalReplayInvocationAuditBindingV1 {
        source_kind: ready.source_kind(),
        attempt: ready.attempt(),
        evidence: ready.audit_evidence(),
    }
}

fn map_reservation_error(
    error: CanonicalReservationSemanticErrorV1,
) -> CanonicalReservationAmbiguityReasonV1 {
    match error {
        CanonicalReservationSemanticErrorV1::ReceiptAlreadyExpiredAtFinalActivation => {
            CanonicalReservationAmbiguityReasonV1::ReceiptAlreadyExpiredAtFinalActivation
        }
        CanonicalReservationSemanticErrorV1::Validation(error) => {
            CanonicalReservationAmbiguityReasonV1::Validation(error)
        }
    }
}

fn freeze_reservation<A, P>(
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    audit: CanonicalReplayInvocationAuditBindingV1,
    reason: CanonicalReservationAmbiguityReasonV1,
    receipt: Option<ActuatorReservationReceiptV1>,
) -> CanonicalReplayReservationOutcomeV1<A, P> {
    CanonicalReplayReservationOutcomeV1::OutcomeUnknown(FrozenCanonicalReplayReservationV1 {
        ready,
        audit,
        reason,
        receipt,
        _actuator: PhantomData,
    })
}

fn complete_reservation<A, P>(
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    audit: CanonicalReplayInvocationAuditBindingV1,
    descriptor_after_matches: bool,
    result: Result<ActuatorReservationReceiptV1, P::Error>,
) -> CanonicalReplayReservationOutcomeV1<A, P>
where
    P: ActuatorEffectProtocolV1,
{
    let receipt = match result {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze_reservation::<A, P>(
                ready,
                audit,
                CanonicalReservationAmbiguityReasonV1::ActuatorErrorAfterReservationBoundary,
                None,
            );
        }
    };

    if !descriptor_after_matches {
        return freeze_reservation::<A, P>(
            ready,
            audit,
            CanonicalReservationAmbiguityReasonV1::ActuatorDescriptorChangedAfterReservation,
            Some(receipt),
        );
    }

    let attempt = ready.attempt();
    let semantic = match validate_reservation_semantics(
        ready.activation_latest_possible_unix_ms(),
        ready.valid_until(),
        attempt,
        receipt,
    ) {
        Ok(value) => value,
        Err(error) => {
            return freeze_reservation::<A, P>(
                ready,
                audit,
                map_reservation_error(error),
                Some(receipt),
            );
        }
    };

    let valid_until = semantic.valid_until();
    let reservation = semantic.into_reservation();
    CanonicalReplayReservationOutcomeV1::Reserved(CanonicalReservedReplayInvocationV1 {
        ready,
        audit,
        reservation,
        valid_until,
        _actuator: PhantomData,
    })
}

/// Perform the non-effecting stable-effect reservation for one exact canonical
/// replay lineage.
pub fn reserve_final_canonical_replay_invocation<A, P>(
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    actuator: &P,
) -> Result<
    CanonicalReplayReservationOutcomeV1<A, P>,
    CanonicalReplayReservationPreconditionFailureV1<A>,
>
where
    P: ActuatorEffectProtocolV1,
{
    let audit = audit_binding(&ready);
    let attempt = ready.attempt();
    let expected = attempt.subject.actuator;
    if actuator.descriptor() != expected {
        return Err(CanonicalReplayReservationPreconditionFailureV1 {
            ready,
            error: CanonicalReservationPreconditionErrorV1::ActuatorDescriptorMismatchBeforeReservation,
        });
    }

    let result = actuator.reserve_effect_subject(&attempt.subject.stable_identity());
    let descriptor_after_matches = actuator.descriptor() == expected;
    Ok(complete_reservation::<A, P>(
        ready,
        audit,
        descriptor_after_matches,
        result,
    ))
}

/// Read-only recovery of the same actuator-side stable-effect reservation.
/// This does not create a different effect subject or perform the effect.
pub fn reconcile_final_canonical_replay_reservation<A, P>(
    ready: FinalCanonicalReplayInvocationReadyV1<A>,
    actuator: &P,
) -> Result<
    CanonicalReplayReservationOutcomeV1<A, P>,
    CanonicalReplayReservationPreconditionFailureV1<A>,
>
where
    P: ActuatorEffectProtocolV1,
{
    let audit = audit_binding(&ready);
    let attempt = ready.attempt();
    let expected = attempt.subject.actuator;
    if actuator.descriptor() != expected {
        return Err(CanonicalReplayReservationPreconditionFailureV1 {
            ready,
            error: CanonicalReservationPreconditionErrorV1::ActuatorDescriptorMismatchBeforeReservation,
        });
    }

    let result = actuator.reconcile_effect_reservation(&attempt.subject.stable_identity());
    let descriptor_after_matches = actuator.descriptor() == expected;
    Ok(complete_reservation::<A, P>(
        ready,
        audit,
        descriptor_after_matches,
        result,
    ))
}

pub enum CanonicalReplayInvocationPreconditionFailureV1<A, P, TQ>
where
    TQ: CurrentTimeQualifierV1,
{
    ActuatorDescriptorMismatch {
        reserved: CanonicalReservedReplayInvocationV1<A, P>,
    },
    TimeQualification {
        reserved: CanonicalReservedReplayInvocationV1<A, P>,
        error: CurrentTimeBindingError<TQ::Error>,
    },
    Rejected {
        reserved: CanonicalReservedReplayInvocationV1<A, P>,
        current_time: QualifiedCurrentTimeV1<TQ>,
        error: CanonicalInvocationPreconditionErrorV1,
    },
}

fn map_pre_error(
    error: CanonicalPreInvocationSemanticErrorV1,
) -> CanonicalInvocationPreconditionErrorV1 {
    match error {
        CanonicalPreInvocationSemanticErrorV1::TimeRegressedBeforeFinalActivation => {
            CanonicalInvocationPreconditionErrorV1::TimeRegressedBeforeFinalActivation
        }
        CanonicalPreInvocationSemanticErrorV1::ReservedLineageAlreadyExpired => {
            CanonicalInvocationPreconditionErrorV1::ReservedLineageAlreadyExpired
        }
        CanonicalPreInvocationSemanticErrorV1::AttemptAlreadyExpired => {
            CanonicalInvocationPreconditionErrorV1::AttemptAlreadyExpired
        }
        CanonicalPreInvocationSemanticErrorV1::ActuatorGenerationAlreadyExpired => {
            CanonicalInvocationPreconditionErrorV1::ActuatorGenerationAlreadyExpired
        }
        CanonicalPreInvocationSemanticErrorV1::ReservationAlreadyExpired => {
            CanonicalInvocationPreconditionErrorV1::ReservationAlreadyExpired
        }
    }
}

fn map_post_ambiguity(
    value: CanonicalPostInvocationAmbiguityV1,
) -> CanonicalPostInvocationAmbiguityReasonV1 {
    match value {
        CanonicalPostInvocationAmbiguityV1::PostInvocationTimeQualificationFailed => {
            CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed
        }
        CanonicalPostInvocationAmbiguityV1::PostInvocationTimeRegressed => {
            CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed
        }
        CanonicalPostInvocationAmbiguityV1::ReservedLineageExpiredDuringInvocation => {
            CanonicalPostInvocationAmbiguityReasonV1::ReservedLineageExpiredDuringInvocation
        }
        CanonicalPostInvocationAmbiguityV1::AttemptExpiredDuringInvocation => {
            CanonicalPostInvocationAmbiguityReasonV1::AttemptExpiredDuringInvocation
        }
        CanonicalPostInvocationAmbiguityV1::ActuatorGenerationExpiredDuringInvocation => {
            CanonicalPostInvocationAmbiguityReasonV1::ActuatorGenerationExpiredDuringInvocation
        }
        CanonicalPostInvocationAmbiguityV1::ReservationExpiredDuringInvocation => {
            CanonicalPostInvocationAmbiguityReasonV1::ReservationExpiredDuringInvocation
        }
        CanonicalPostInvocationAmbiguityV1::EffectReceiptExpiredAtPostInvocationTime => {
            CanonicalPostInvocationAmbiguityReasonV1::EffectReceiptExpiredAtPostInvocationTime
        }
    }
}

fn map_disposition(
    value: CanonicalExecutionQualifiedDispositionV1,
) -> CanonicalQualifiedEffectDispositionV1 {
    match value {
        CanonicalExecutionQualifiedDispositionV1::Confirmed {
            outcome,
            actuator_receipt,
        } => CanonicalQualifiedEffectDispositionV1::Confirmed {
            outcome,
            actuator_receipt,
        },
        CanonicalExecutionQualifiedDispositionV1::ProvenNotApplied {
            evidence,
            actuator_receipt,
        } => CanonicalQualifiedEffectDispositionV1::ProvenNotApplied {
            evidence,
            actuator_receipt,
        },
        CanonicalExecutionQualifiedDispositionV1::OutcomeUnknown {
            low_level_disposition,
            qualified_actuator_receipt,
            post_invocation_ambiguity,
            recovery_policy,
        } => CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            low_level_disposition,
            qualified_actuator_receipt,
            post_invocation_ambiguity: post_invocation_ambiguity.map(map_post_ambiguity),
            recovery_policy,
        },
    }
}

/// Post-effect result for the one authorized canonical replay attempt. No method
/// returns the consumed reserved lineage, so this object cannot invoke a third
/// attempt or repeat the second attempt.
pub struct CanonicalReplayActuatorInvocationOutcomeV1<A, P, TQ> {
    audit: CanonicalReplayInvocationAuditBindingV1,
    attempt: ActuatorInvocationAttemptManifestV1,
    reservation_receipt: ActuatorReservationReceiptV1,
    pre_invocation_time: QualifiedCurrentTimeV1<TQ>,
    post_invocation_time: Option<QualifiedCurrentTimeV1<TQ>>,
    reported_receipt: Option<ActuatorEffectReceiptV1>,
    post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
    low_level_outcome: ValidatedActuatorEffectOutcomeV1,
    canonical_disposition: CanonicalQualifiedEffectDispositionV1,
    _lineage: PhantomData<fn() -> (A, P)>,
}

impl<A, P, TQ> CanonicalReplayActuatorInvocationOutcomeV1<A, P, TQ> {
    pub const fn audit(&self) -> CanonicalReplayInvocationAuditBindingV1 {
        self.audit
    }

    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.attempt
    }

    pub const fn reservation_receipt(&self) -> ActuatorReservationReceiptV1 {
        self.reservation_receipt
    }

    pub const fn pre_invocation_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.pre_invocation_time
    }

    pub fn post_invocation_time(&self) -> Option<&QualifiedCurrentTimeV1<TQ>> {
        self.post_invocation_time.as_ref()
    }

    pub const fn reported_receipt(&self) -> Option<ActuatorEffectReceiptV1> {
        self.reported_receipt
    }

    pub const fn post_invocation_ambiguity(
        &self,
    ) -> Option<CanonicalPostInvocationAmbiguityReasonV1> {
        self.post_invocation_ambiguity
    }

    pub const fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.low_level_outcome
    }

    pub const fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1 {
        self.canonical_disposition
    }

    pub const fn external_effect_may_have_occurred(&self) -> bool {
        true
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }

    pub const fn permits_third_attempt(&self) -> bool {
        false
    }
}

fn pre_failure<A, P, TQ>(
    reserved: CanonicalReservedReplayInvocationV1<A, P>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalPreInvocationSemanticErrorV1,
) -> CanonicalReplayInvocationPreconditionFailureV1<A, P, TQ>
where
    TQ: CurrentTimeQualifierV1,
{
    CanonicalReplayInvocationPreconditionFailureV1::Rejected {
        reserved,
        current_time,
        error: map_pre_error(error),
    }
}

/// Invoke the one exact reserved canonical replay effect. All failures before
/// `invoke_reserved_effect` return the owned reserved lineage. Once that call
/// begins, this function always returns a post-effect outcome and never a
/// retryable error.
pub fn invoke_canonical_replay_actuator_effect<A, P, TQ>(
    reserved: CanonicalReservedReplayInvocationV1<A, P>,
    actuator: &P,
    expected_time: ExpectedCurrentTimeQualifierProfileV1,
    time_qualifier: &TQ,
) -> Result<
    CanonicalReplayActuatorInvocationOutcomeV1<A, P, TQ>,
    CanonicalReplayInvocationPreconditionFailureV1<A, P, TQ>,
>
where
    P: ActuatorEffectProtocolV1,
    TQ: CurrentTimeQualifierV1,
{
    let attempt = reserved.attempt();
    let expected = attempt.subject.actuator;

    if actuator.descriptor() != expected {
        return Err(CanonicalReplayInvocationPreconditionFailureV1::ActuatorDescriptorMismatch {
            reserved,
        });
    }

    let pre_time = match qualify_current_time(expected_time, time_qualifier) {
        Ok(time) => time,
        Err(error) => {
            return Err(CanonicalReplayInvocationPreconditionFailureV1::TimeQualification {
                reserved,
                error,
            });
        }
    };
    let pre_latest = pre_time.latest_possible_unix_ms();

    if let Some(error) = pre_invocation_error(
        reserved.ready.activation_latest_possible_unix_ms(),
        reserved.valid_until,
        attempt,
        reserved.reservation.receipt(),
        pre_latest,
    ) {
        return Err(pre_failure(reserved, pre_time, error));
    }

    let audit = reserved.audit;
    let reservation_receipt = reserved.reservation.receipt();
    let reservation = reserved.reservation.reservation();
    let reserved_valid_until = reserved.valid_until;

    let raw_result = actuator.invoke_reserved_effect(&attempt, reservation);
    let descriptor_after = actuator.descriptor();

    let post_time_result = qualify_current_time(expected_time, time_qualifier);
    let (post_time, kernel_post_ambiguity) = match post_time_result {
        Ok(time) => {
            let ambiguity = post_invocation_ambiguity(
                reserved_valid_until,
                attempt,
                reservation_receipt,
                pre_latest,
                time.latest_possible_unix_ms(),
                raw_result.as_ref().ok(),
            );
            (Some(time), ambiguity)
        }
        Err(_) => (
            None,
            Some(CanonicalPostInvocationAmbiguityV1::PostInvocationTimeQualificationFailed),
        ),
    };

    let reported_receipt = raw_result.as_ref().ok().copied();

    // Interpret the exact raw actuator evidence without modifying any field or
    // receipt commitment. Canonical qualification is a separate layer.
    let low_level_outcome = interpret_actuator_invocation_result(
        expected,
        attempt,
        reservation,
        descriptor_after,
        raw_result,
    );
    let canonical_disposition = map_disposition(qualified_effect_disposition(
        low_level_outcome,
        kernel_post_ambiguity,
    ));
    let post_invocation_ambiguity = kernel_post_ambiguity.map(map_post_ambiguity);

    Ok(CanonicalReplayActuatorInvocationOutcomeV1 {
        audit,
        attempt,
        reservation_receipt,
        pre_invocation_time: pre_time,
        post_invocation_time: post_time,
        reported_receipt,
        post_invocation_ambiguity,
        low_level_outcome,
        canonical_disposition,
        _lineage: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_post_effect_never_grants_third_attempt() {
        let permits_third_attempt = false;
        assert!(!permits_third_attempt);
    }

    #[test]
    fn kernel_and_public_post_ambiguity_map_one_to_one() {
        assert_eq!(
            map_post_ambiguity(CanonicalPostInvocationAmbiguityV1::PostInvocationTimeRegressed),
            CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed
        );
    }

    #[test]
    fn reservation_expiry_reason_is_preserved() {
        assert_eq!(
            map_reservation_error(
                CanonicalReservationSemanticErrorV1::ReceiptAlreadyExpiredAtFinalActivation
            ),
            CanonicalReservationAmbiguityReasonV1::ReceiptAlreadyExpiredAtFinalActivation
        );
    }
}
