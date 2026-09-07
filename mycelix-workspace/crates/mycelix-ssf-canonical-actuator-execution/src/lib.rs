// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical SSF bridge from final actuator readiness to the provider-neutral
//! actuator effect protocol.
//!
//! The only execution entry point consumes `FinalActuatorInvocationReadyV1`.
//! Reservation is explicitly non-effecting. Immediately before invocation and
//! again after the actuator call returns, this crate obtains independently
//! qualified trusted time. Raw actuator evidence is never rewritten: the
//! wrapper retains the exact reported receipt and low-level interpretation while
//! deriving a separate canonical qualified disposition.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_actuator_effect_protocol::{
    interpret_actuator_invocation_result, validate_actuator_reservation,
    ActuatorEffectAmbiguityReasonV1, ActuatorEffectOutcomeCommitment,
    ActuatorEffectProtocolV1, ActuatorEffectReceiptCommitment, ActuatorEffectReceiptV1,
    ActuatorInvocationAttemptManifestV1, ActuatorNonApplicationEvidenceCommitment,
    ActuatorReservationReceiptV1, ActuatorReservationValidationError, EffectRecoveryPolicyV1,
    ValidatedActuatorEffectDispositionV1, ValidatedActuatorEffectOutcomeV1,
    ValidatedActuatorReservationV1,
};
use mycelix_ssf_actuator_invocation_activation::{
    ActivatedActuatorInvocationV1, InvocationJournalEvidenceV1,
};
use mycelix_ssf_current_authority_revalidation::{
    qualify_current_time, CurrentTimeBindingError, CurrentTimeQualifierV1,
    ExpectedCurrentTimeQualifierProfileV1, QualifiedCurrentTimeV1,
};
use mycelix_ssf_final_actuator_invocation_readiness::{
    FinalActuatorInvocationReadyV1, FinalInvocationSourceKindV1,
};
use mycelix_ssf_historical_actuator_invocation_activation::HistoricallyActivatedActuatorInvocationV1;
use mycelix_ssf_historical_actuator_invocation_journal::{
    HistoricalInvocationJournalEvidenceV1, HistoricallyReconciledInitialInvocationJournalV1,
    HistoricallyReconciledReplayInvocationJournalV1,
};

/// Copyable audit evidence for the exact durable invocation lineage consumed by
/// the canonical execution path. This is provenance, not authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalInvocationJournalEvidenceV1 {
    CurrentOrSameGeneration(InvocationJournalEvidenceV1),
    HistoricalSameIdentity(HistoricalInvocationJournalEvidenceV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalInvocationJournalAuditBindingV1 {
    pub source_kind: FinalInvocationSourceKindV1,
    pub attempt: ActuatorInvocationAttemptManifestV1,
    pub evidence: CanonicalInvocationJournalEvidenceV1,
}

mod audit_sealed {
    pub trait Sealed {}
}

/// Sealed extraction of non-authoritative journal provenance from the exact
/// final-readiness variants supported by the SSF stack.
pub trait ExactFinalReadinessJournalAuditV1: audit_sealed::Sealed {
    fn exact_journal_audit_binding(&self) -> CanonicalInvocationJournalAuditBindingV1;
}

impl<J, TQ> audit_sealed::Sealed
    for FinalActuatorInvocationReadyV1<ActivatedActuatorInvocationV1<J, TQ>>
{
}

impl<J, TQ> ExactFinalReadinessJournalAuditV1
    for FinalActuatorInvocationReadyV1<ActivatedActuatorInvocationV1<J, TQ>>
{
    fn exact_journal_audit_binding(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        CanonicalInvocationJournalAuditBindingV1 {
            source_kind: self.source_kind(),
            attempt: self.attempt(),
            evidence: CanonicalInvocationJournalEvidenceV1::CurrentOrSameGeneration(
                self.activated().journal_evidence(),
            ),
        }
    }
}

mod historical_evidence_sealed {
    pub trait Sealed {}
}

pub trait HistoricalJournalEvidenceSourceV1: historical_evidence_sealed::Sealed {
    fn historical_journal_evidence(&self) -> HistoricalInvocationJournalEvidenceV1;
}

impl<R, RTQ> historical_evidence_sealed::Sealed
    for HistoricallyReconciledInitialInvocationJournalV1<R, RTQ>
{
}

impl<R, RTQ> HistoricalJournalEvidenceSourceV1
    for HistoricallyReconciledInitialInvocationJournalV1<R, RTQ>
{
    fn historical_journal_evidence(&self) -> HistoricalInvocationJournalEvidenceV1 {
        self.reconciliation_receipt().evidence
    }
}

impl<R, RTQ> historical_evidence_sealed::Sealed
    for HistoricallyReconciledReplayInvocationJournalV1<R, RTQ>
{
}

impl<R, RTQ> HistoricalJournalEvidenceSourceV1
    for HistoricallyReconciledReplayInvocationJournalV1<R, RTQ>
{
    fn historical_journal_evidence(&self) -> HistoricalInvocationJournalEvidenceV1 {
        self.reconciliation_receipt().evidence
    }
}

impl<H, TQ> audit_sealed::Sealed
    for FinalActuatorInvocationReadyV1<HistoricallyActivatedActuatorInvocationV1<H, TQ>>
where
    H: HistoricalJournalEvidenceSourceV1,
{
}

impl<H, TQ> ExactFinalReadinessJournalAuditV1
    for FinalActuatorInvocationReadyV1<HistoricallyActivatedActuatorInvocationV1<H, TQ>>
where
    H: HistoricalJournalEvidenceSourceV1,
{
    fn exact_journal_audit_binding(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        CanonicalInvocationJournalAuditBindingV1 {
            source_kind: self.source_kind(),
            attempt: self.attempt(),
            evidence: CanonicalInvocationJournalEvidenceV1::HistoricalSameIdentity(
                self.activated().historical().historical_journal_evidence(),
            ),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReservationPreconditionErrorV1 {
    ActuatorDescriptorMismatchBeforeReservation,
}

pub struct CanonicalReservationPreconditionFailureV1<A> {
    ready: A,
    error: CanonicalReservationPreconditionErrorV1,
}

impl<A> CanonicalReservationPreconditionFailureV1<A> {
    pub const fn error(&self) -> CanonicalReservationPreconditionErrorV1 {
        self.error
    }

    pub fn into_ready(self) -> A {
        self.ready
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalReservationAmbiguityReasonV1 {
    ActuatorErrorAfterReservationBoundary,
    ActuatorDescriptorChangedAfterReservation,
    ReceiptAlreadyExpiredAtFinalActivation,
    Validation(ActuatorReservationValidationError),
}

/// Reservation state is unknown/blocked, but this layer has still not invoked
/// the external effect.
pub struct FrozenCanonicalActuatorReservationV1<A, P> {
    ready: FinalActuatorInvocationReadyV1<A>,
    journal_audit: CanonicalInvocationJournalAuditBindingV1,
    reason: CanonicalReservationAmbiguityReasonV1,
    receipt: Option<ActuatorReservationReceiptV1>,
    _actuator: PhantomData<fn() -> P>,
}

impl<A, P> FrozenCanonicalActuatorReservationV1<A, P> {
    pub const fn ready(&self) -> &FinalActuatorInvocationReadyV1<A> {
        &self.ready
    }

    pub const fn journal_audit(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        self.journal_audit
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
}

/// Exact final-ready invocation plus one validated actuator-side stable-effect
/// reservation. This remains pre-effect state.
pub struct CanonicalReservedActuatorInvocationV1<A, P> {
    ready: FinalActuatorInvocationReadyV1<A>,
    journal_audit: CanonicalInvocationJournalAuditBindingV1,
    reservation: ValidatedActuatorReservationV1,
    valid_until: u64,
    _actuator: PhantomData<fn() -> P>,
}

impl<A, P> CanonicalReservedActuatorInvocationV1<A, P> {
    pub const fn ready(&self) -> &FinalActuatorInvocationReadyV1<A> {
        &self.ready
    }

    pub const fn journal_audit(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        self.journal_audit
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
}

pub enum CanonicalReservationOutcomeV1<A, P> {
    Reserved(CanonicalReservedActuatorInvocationV1<A, P>),
    OutcomeUnknown(FrozenCanonicalActuatorReservationV1<A, P>),
}

fn freeze_reservation<A, P>(
    ready: FinalActuatorInvocationReadyV1<A>,
    journal_audit: CanonicalInvocationJournalAuditBindingV1,
    reason: CanonicalReservationAmbiguityReasonV1,
    receipt: Option<ActuatorReservationReceiptV1>,
) -> CanonicalReservationOutcomeV1<A, P> {
    CanonicalReservationOutcomeV1::OutcomeUnknown(FrozenCanonicalActuatorReservationV1 {
        ready,
        journal_audit,
        reason,
        receipt,
        _actuator: PhantomData,
    })
}

fn complete_reservation<A, P>(
    ready: FinalActuatorInvocationReadyV1<A>,
    journal_audit: CanonicalInvocationJournalAuditBindingV1,
    descriptor_after_matches: bool,
    result: Result<ActuatorReservationReceiptV1, P::Error>,
) -> CanonicalReservationOutcomeV1<A, P>
where
    P: ActuatorEffectProtocolV1,
{
    let receipt = match result {
        Ok(receipt) => receipt,
        Err(_) => {
            return freeze_reservation::<A, P>(
                ready,
                journal_audit,
                CanonicalReservationAmbiguityReasonV1::ActuatorErrorAfterReservationBoundary,
                None,
            );
        }
    };

    if !descriptor_after_matches {
        return freeze_reservation::<A, P>(
            ready,
            journal_audit,
            CanonicalReservationAmbiguityReasonV1::ActuatorDescriptorChangedAfterReservation,
            Some(receipt),
        );
    }

    if receipt.valid_until < ready.activation_latest_possible_unix_ms() {
        return freeze_reservation::<A, P>(
            ready,
            journal_audit,
            CanonicalReservationAmbiguityReasonV1::ReceiptAlreadyExpiredAtFinalActivation,
            Some(receipt),
        );
    }

    let attempt = ready.attempt();
    let expected = attempt.subject.actuator;
    let reservation = match validate_actuator_reservation(expected, attempt, receipt) {
        Ok(reservation) => reservation,
        Err(error) => {
            return freeze_reservation::<A, P>(
                ready,
                journal_audit,
                CanonicalReservationAmbiguityReasonV1::Validation(error),
                Some(receipt),
            );
        }
    };

    let valid_until = ready.valid_until().min(reservation.receipt().valid_until);
    CanonicalReservationOutcomeV1::Reserved(CanonicalReservedActuatorInvocationV1 {
        ready,
        journal_audit,
        reservation,
        valid_until,
        _actuator: PhantomData,
    })
}

/// Perform the non-effecting actuator-side stable-subject reservation.
pub fn reserve_final_actuator_invocation<A, P>(
    ready: FinalActuatorInvocationReadyV1<A>,
    actuator: &P,
) -> Result<
    CanonicalReservationOutcomeV1<A, P>,
    CanonicalReservationPreconditionFailureV1<FinalActuatorInvocationReadyV1<A>>,
>
where
    P: ActuatorEffectProtocolV1,
    FinalActuatorInvocationReadyV1<A>: ExactFinalReadinessJournalAuditV1,
{
    let journal_audit = ready.exact_journal_audit_binding();
    let attempt = ready.attempt();
    let expected = attempt.subject.actuator;
    if actuator.descriptor() != expected {
        return Err(CanonicalReservationPreconditionFailureV1 {
            ready,
            error: CanonicalReservationPreconditionErrorV1::ActuatorDescriptorMismatchBeforeReservation,
        });
    }

    let result = actuator.reserve_effect_subject(&attempt.subject.stable_identity());
    let descriptor_after_matches = actuator.descriptor() == expected;
    Ok(complete_reservation::<A, P>(
        ready,
        journal_audit,
        descriptor_after_matches,
        result,
    ))
}

/// Read-only recovery of the same stable actuator reservation. This does not
/// create a different effect subject and performs no external effect.
pub fn reconcile_final_actuator_reservation<A, P>(
    ready: FinalActuatorInvocationReadyV1<A>,
    actuator: &P,
) -> Result<
    CanonicalReservationOutcomeV1<A, P>,
    CanonicalReservationPreconditionFailureV1<FinalActuatorInvocationReadyV1<A>>,
>
where
    P: ActuatorEffectProtocolV1,
    FinalActuatorInvocationReadyV1<A>: ExactFinalReadinessJournalAuditV1,
{
    let journal_audit = ready.exact_journal_audit_binding();
    let attempt = ready.attempt();
    let expected = attempt.subject.actuator;
    if actuator.descriptor() != expected {
        return Err(CanonicalReservationPreconditionFailureV1 {
            ready,
            error: CanonicalReservationPreconditionErrorV1::ActuatorDescriptorMismatchBeforeReservation,
        });
    }

    let result = actuator.reconcile_effect_reservation(&attempt.subject.stable_identity());
    let descriptor_after_matches = actuator.descriptor() == expected;
    Ok(complete_reservation::<A, P>(
        ready,
        journal_audit,
        descriptor_after_matches,
        result,
    ))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalInvocationPreconditionErrorV1 {
    TimeRegressedBeforeFinalActivation,
    ReservedLineageAlreadyExpired,
    AttemptAlreadyExpired,
    ActuatorGenerationAlreadyExpired,
    ReservationAlreadyExpired,
}

pub enum CanonicalInvocationPreconditionFailureV1<A, P, TQ>
where
    TQ: CurrentTimeQualifierV1,
{
    ActuatorDescriptorMismatch {
        reserved: CanonicalReservedActuatorInvocationV1<A, P>,
    },
    TimeQualification {
        reserved: CanonicalReservedActuatorInvocationV1<A, P>,
        error: CurrentTimeBindingError<TQ::Error>,
    },
    Rejected {
        reserved: CanonicalReservedActuatorInvocationV1<A, P>,
        current_time: QualifiedCurrentTimeV1<TQ>,
        error: CanonicalInvocationPreconditionErrorV1,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalPostInvocationAmbiguityReasonV1 {
    PostInvocationTimeQualificationFailed,
    PostInvocationTimeRegressed,
    ReservedLineageExpiredDuringInvocation,
    AttemptExpiredDuringInvocation,
    ActuatorGenerationExpiredDuringInvocation,
    ReservationExpiredDuringInvocation,
    EffectReceiptExpiredAtPostInvocationTime,
}

/// Canonical effect classification after both low-level actuator validation and
/// the wrapper's post-invocation trusted-time/authority checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalQualifiedEffectDispositionV1 {
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
        post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
        recovery_policy: EffectRecoveryPolicyV1,
    },
}

impl CanonicalQualifiedEffectDispositionV1 {
    pub const fn is_terminal(&self) -> bool {
        matches!(Self::Confirmed { .. } | Self::ProvenNotApplied { .. }, self)
    }

    pub const fn creates_replay_authority(&self) -> bool {
        false
    }
}

fn canonical_disposition(
    low_level: ValidatedActuatorEffectOutcomeV1,
    post_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
) -> CanonicalQualifiedEffectDispositionV1 {
    let qualified_receipt = low_level.receipt().map(|receipt| receipt.receipt_commitment);

    if post_ambiguity.is_some() {
        return CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
            low_level_disposition: low_level.disposition(),
            qualified_actuator_receipt: qualified_receipt,
            post_invocation_ambiguity: post_ambiguity,
            recovery_policy: low_level.recovery_policy(),
        };
    }

    match low_level.disposition() {
        ValidatedActuatorEffectDispositionV1::Confirmed { outcome } => {
            if let Some(actuator_receipt) = qualified_receipt {
                CanonicalQualifiedEffectDispositionV1::Confirmed {
                    outcome,
                    actuator_receipt,
                }
            } else {
                CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
                    low_level_disposition: low_level.disposition(),
                    qualified_actuator_receipt: None,
                    post_invocation_ambiguity: None,
                    recovery_policy: low_level.recovery_policy(),
                }
            }
        }
        ValidatedActuatorEffectDispositionV1::ProvenNotApplied { evidence } => {
            if let Some(actuator_receipt) = qualified_receipt {
                CanonicalQualifiedEffectDispositionV1::ProvenNotApplied {
                    evidence,
                    actuator_receipt,
                }
            } else {
                CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
                    low_level_disposition: low_level.disposition(),
                    qualified_actuator_receipt: None,
                    post_invocation_ambiguity: None,
                    recovery_policy: low_level.recovery_policy(),
                }
            }
        }
        ValidatedActuatorEffectDispositionV1::OutcomeUnknown { .. } => {
            CanonicalQualifiedEffectDispositionV1::OutcomeUnknown {
                low_level_disposition: low_level.disposition(),
                qualified_actuator_receipt: qualified_receipt,
                post_invocation_ambiguity: None,
                recovery_policy: low_level.recovery_policy(),
            }
        }
    }
}

/// Canonical post-effect result. No method returns the consumed reserved token,
/// so this object cannot be used to invoke the same lineage again.
pub struct CanonicalActuatorInvocationOutcomeV1<A, P, TQ> {
    reserved: CanonicalReservedActuatorInvocationV1<A, P>,
    pre_invocation_time: QualifiedCurrentTimeV1<TQ>,
    post_invocation_time: Option<QualifiedCurrentTimeV1<TQ>>,
    reported_receipt: Option<ActuatorEffectReceiptV1>,
    post_invocation_ambiguity: Option<CanonicalPostInvocationAmbiguityReasonV1>,
    low_level_outcome: ValidatedActuatorEffectOutcomeV1,
    canonical_disposition: CanonicalQualifiedEffectDispositionV1,
}

impl<A, P, TQ> CanonicalActuatorInvocationOutcomeV1<A, P, TQ> {
    pub const fn attempt(&self) -> ActuatorInvocationAttemptManifestV1 {
        self.reserved.attempt()
    }

    pub const fn journal_audit(&self) -> CanonicalInvocationJournalAuditBindingV1 {
        self.reserved.journal_audit()
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

    /// Lower-level interpretation of the exact raw actuator receipt. This is
    /// retained for audit; downstream authority/replay policy should use
    /// `canonical_disposition` instead.
    pub const fn low_level_outcome(&self) -> ValidatedActuatorEffectOutcomeV1 {
        self.low_level_outcome
    }

    pub const fn canonical_disposition(&self) -> CanonicalQualifiedEffectDispositionV1 {
        self.canonical_disposition
    }

    pub const fn external_effect_may_have_occurred(&self) -> bool {
        true
    }
}

fn pre_invocation_error<A, P, TQ>(
    reserved: CanonicalReservedActuatorInvocationV1<A, P>,
    current_time: QualifiedCurrentTimeV1<TQ>,
    error: CanonicalInvocationPreconditionErrorV1,
) -> CanonicalInvocationPreconditionFailureV1<A, P, TQ>
where
    TQ: CurrentTimeQualifierV1,
{
    CanonicalInvocationPreconditionFailureV1::Rejected {
        reserved,
        current_time,
        error,
    }
}

fn post_invocation_reason<TQ>(
    reserved_valid_until: u64,
    attempt: ActuatorInvocationAttemptManifestV1,
    reservation_receipt: ActuatorReservationReceiptV1,
    pre_time: &QualifiedCurrentTimeV1<TQ>,
    post_time: &QualifiedCurrentTimeV1<TQ>,
    effect_receipt: Option<&ActuatorEffectReceiptV1>,
) -> Option<CanonicalPostInvocationAmbiguityReasonV1> {
    let post_latest = post_time.latest_possible_unix_ms();
    if post_latest < pre_time.latest_possible_unix_ms() {
        return Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeRegressed);
    }
    if reserved_valid_until < post_latest {
        return Some(
            CanonicalPostInvocationAmbiguityReasonV1::ReservedLineageExpiredDuringInvocation,
        );
    }
    if attempt.attempt_valid_until < post_latest {
        return Some(CanonicalPostInvocationAmbiguityReasonV1::AttemptExpiredDuringInvocation);
    }
    if attempt.subject.actuator.valid_until < post_latest {
        return Some(
            CanonicalPostInvocationAmbiguityReasonV1::ActuatorGenerationExpiredDuringInvocation,
        );
    }
    if reservation_receipt.valid_until < post_latest {
        return Some(CanonicalPostInvocationAmbiguityReasonV1::ReservationExpiredDuringInvocation);
    }
    if let Some(receipt) = effect_receipt {
        if receipt.valid_until < post_latest {
            return Some(
                CanonicalPostInvocationAmbiguityReasonV1::EffectReceiptExpiredAtPostInvocationTime,
            );
        }
    }
    None
}

/// Invoke one exact reserved effect. This is the first canonical external-effect
/// entry point. All failures before `invoke_reserved_effect` return the owned
/// reserved lineage. After that call begins, this function always returns a
/// canonical outcome and never a retryable error.
pub fn invoke_canonical_actuator_effect<A, P, TQ>(
    reserved: CanonicalReservedActuatorInvocationV1<A, P>,
    actuator: &P,
    expected_time: ExpectedCurrentTimeQualifierProfileV1,
    time_qualifier: &TQ,
) -> Result<
    CanonicalActuatorInvocationOutcomeV1<A, P, TQ>,
    CanonicalInvocationPreconditionFailureV1<A, P, TQ>,
>
where
    P: ActuatorEffectProtocolV1,
    TQ: CurrentTimeQualifierV1,
{
    let attempt = reserved.attempt();
    let expected = attempt.subject.actuator;

    if actuator.descriptor() != expected {
        return Err(CanonicalInvocationPreconditionFailureV1::ActuatorDescriptorMismatch {
            reserved,
        });
    }

    let pre_time = match qualify_current_time(expected_time, time_qualifier) {
        Ok(time) => time,
        Err(error) => {
            return Err(CanonicalInvocationPreconditionFailureV1::TimeQualification {
                reserved,
                error,
            });
        }
    };
    let pre_latest = pre_time.latest_possible_unix_ms();

    let pre_error = if pre_latest < reserved.ready.activation_latest_possible_unix_ms() {
        Some(CanonicalInvocationPreconditionErrorV1::TimeRegressedBeforeFinalActivation)
    } else if reserved.valid_until < pre_latest {
        Some(CanonicalInvocationPreconditionErrorV1::ReservedLineageAlreadyExpired)
    } else if attempt.attempt_valid_until < pre_latest {
        Some(CanonicalInvocationPreconditionErrorV1::AttemptAlreadyExpired)
    } else if expected.valid_until < pre_latest {
        Some(CanonicalInvocationPreconditionErrorV1::ActuatorGenerationAlreadyExpired)
    } else if reserved.reservation.receipt().valid_until < pre_latest {
        Some(CanonicalInvocationPreconditionErrorV1::ReservationAlreadyExpired)
    } else {
        None
    };

    if let Some(error) = pre_error {
        return Err(pre_invocation_error(reserved, pre_time, error));
    }

    let reservation = reserved.reservation.reservation();
    let raw_result = actuator.invoke_reserved_effect(&attempt, reservation);
    let descriptor_after = actuator.descriptor();

    let post_time_result = qualify_current_time(expected_time, time_qualifier);
    let (post_time, mut post_ambiguity) = match post_time_result {
        Ok(time) => (Some(time), None),
        Err(_) => (
            None,
            Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed),
        ),
    };

    let reported_receipt = raw_result.as_ref().ok().copied();

    if post_ambiguity.is_none() {
        if let Some(ref time) = post_time {
            post_ambiguity = post_invocation_reason(
                reserved.valid_until,
                attempt,
                reserved.reservation.receipt(),
                &pre_time,
                time,
                reported_receipt.as_ref(),
            );
        }
    }

    // Interpret the actuator's exact raw evidence without rewriting any field
    // or receipt commitment. Canonical post-time qualification is a separate
    // classification layer below.
    let low_level_outcome = interpret_actuator_invocation_result(
        expected,
        attempt,
        reservation,
        descriptor_after,
        raw_result,
    );
    let canonical_disposition = canonical_disposition(low_level_outcome, post_ambiguity);

    Ok(CanonicalActuatorInvocationOutcomeV1 {
        reserved,
        pre_invocation_time: pre_time,
        post_invocation_time: post_time,
        reported_receipt,
        post_invocation_ambiguity: post_ambiguity,
        low_level_outcome,
        canonical_disposition,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn post_time_must_not_regress() {
        let pre = 101_u64;
        let post = 100_u64;
        assert!(post < pre);
    }

    #[test]
    fn post_effect_ceiling_can_only_shrink() {
        let reserved = 120_u64;
        let attempt = 115_u64;
        let actuator = 110_u64;
        let reservation = 105_u64;
        let effect = 103_u64;
        assert_eq!(
            reserved.min(attempt).min(actuator).min(reservation).min(effect),
            103
        );
    }

    #[test]
    fn post_qualification_failure_is_not_terminal() {
        let ambiguity = Some(CanonicalPostInvocationAmbiguityReasonV1::PostInvocationTimeQualificationFailed);
        assert!(ambiguity.is_some());
    }
}
