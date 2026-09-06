// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Capability-time revalidation for one exact rebound durable possible-effect lineage.
//!
//! Durable effect admission proves that one exact effect lineage may have a
//! later execution capability minted. It does not prove that the authenticated
//! carrier, transport, actuator policy/health, or time are still current now.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_durable_effect_admission::ExactExecutionAdmissionCandidateV1;
use mycelix_ssf_effect_admission_rebind::{
    ReboundDurablyAdmittedEffectV1, ReboundEffectAdmissionBindingV1,
};
use mycelix_ssf_execution_surface_admission::{
    ExecutionSurfaceAdmissionReceiptV1, ExecutionSurfaceGenerationV1,
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

digest_type!(CapabilityTimeRevalidationReceiptCommitment);

/// Exact live execution generation independently supplied by the authenticated
/// execution adapter, plus the verifier generation trusted to rebind it to the
/// durable possible-effect lineage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedCapabilityExecutionProfileV1 {
    verifier: VerifierDescriptorV1,
    generation: ExecutionSurfaceGenerationV1,
}

impl ExpectedCapabilityExecutionProfileV1 {
    pub const fn from_authenticated_execution_surface(
        verifier: VerifierDescriptorV1,
        generation: ExecutionSurfaceGenerationV1,
    ) -> Self {
        Self {
            verifier,
            generation,
        }
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn generation(&self) -> ExecutionSurfaceGenerationV1 {
        self.generation
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapabilityTimeRevalidationSubjectV1 {
    pub admitted_effect: ReboundEffectAdmissionBindingV1,
    pub original_execution_admission: ExecutionSurfaceAdmissionReceiptV1,
    pub effect_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapabilityTimeRevalidationReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub subject: CapabilityTimeRevalidationSubjectV1,
    pub generation: ExecutionSurfaceGenerationV1,
    pub valid_until: u64,
    pub receipt_commitment: CapabilityTimeRevalidationReceiptCommitment,
}

impl CapabilityTimeRevalidationReceiptV1 {
    pub const fn new(
        verifier: VerifierDescriptorV1,
        subject: CapabilityTimeRevalidationSubjectV1,
        generation: ExecutionSurfaceGenerationV1,
        valid_until: u64,
        receipt_commitment: CapabilityTimeRevalidationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            subject,
            generation,
            valid_until,
            receipt_commitment,
        }
    }
}

pub trait CapabilityTimeExecutionQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn revalidate_execution_generation(
        &self,
        subject: &CapabilityTimeRevalidationSubjectV1,
        expected_generation: &ExecutionSurfaceGenerationV1,
    ) -> Result<CapabilityTimeRevalidationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapabilityTimeEffectRevalidationError<E> {
    LiveExecutionGenerationChanged,
    VerifierDescriptorMismatchBefore,
    Verifier(E),
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    SubjectMismatch,
    ExecutionGenerationMismatch,
    ReceiptOutlivesVerifier,
    ReceiptOutlivesExecutionGeneration,
    ReceiptOutlivesEffectEligibility,
    ExecutionGenerationAlreadyExpired,
    EffectEligibilityAlreadyExpired,
    ReceiptAlreadyExpired,
    TrustedTimeAlreadyExpired,
}

fn derive_capability_ready_until<E>(
    effect_eligibility_valid_until: u64,
    verifier_valid_until: u64,
    generation_valid_until: u64,
    receipt_valid_until: u64,
    trusted_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, CapabilityTimeEffectRevalidationError<E>> {
    if generation_valid_until < latest_possible_now {
        return Err(CapabilityTimeEffectRevalidationError::ExecutionGenerationAlreadyExpired);
    }
    if effect_eligibility_valid_until < latest_possible_now {
        return Err(CapabilityTimeEffectRevalidationError::EffectEligibilityAlreadyExpired);
    }
    if receipt_valid_until < latest_possible_now {
        return Err(CapabilityTimeEffectRevalidationError::ReceiptAlreadyExpired);
    }
    if trusted_time_valid_until < latest_possible_now {
        return Err(CapabilityTimeEffectRevalidationError::TrustedTimeAlreadyExpired);
    }

    Ok(effect_eligibility_valid_until
        .min(verifier_valid_until)
        .min(generation_valid_until)
        .min(receipt_valid_until)
        .min(trusted_time_valid_until))
}

/// Exact possible-effect lineage that is current enough to proceed to
/// source-owned operation materialization.
///
/// This is still not a move-only effect capability.
pub struct CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ> {
    rebound: ReboundDurablyAdmittedEffectV1<S, C>,
    expected: ExpectedCapabilityExecutionProfileV1,
    receipt: CapabilityTimeRevalidationReceiptV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    valid_until: u64,
    _qualifier: PhantomData<Q>,
}

impl<S, C, Q, TQ> CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ> {
    pub const fn rebound(&self) -> &ReboundDurablyAdmittedEffectV1<S, C> {
        &self.rebound
    }

    pub const fn expected(&self) -> ExpectedCapabilityExecutionProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> CapabilityTimeRevalidationReceiptV1 {
        self.receipt
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn exact_live_execution_generation_revalidated(&self) -> bool {
        true
    }

    pub const fn source_owned_operation_material_required(&self) -> bool {
        true
    }

    pub const fn eligible_for_move_only_capability_without_payload_binding(&self) -> bool {
        false
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn revalidate_effect_at_capability_time<S, C, Q, TQ>(
    rebound: ReboundDurablyAdmittedEffectV1<S, C>,
    expected: ExpectedCapabilityExecutionProfileV1,
    qualifier: &Q,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ>,
    CapabilityTimeEffectRevalidationError<Q::Error>,
>
where
    C: ExactExecutionAdmissionCandidateV1,
    Q: CapabilityTimeExecutionQualifierV1,
{
    let original_execution_admission = rebound.candidate().exact_admission_receipt();
    if original_execution_admission.generation != expected.generation {
        return Err(CapabilityTimeEffectRevalidationError::LiveExecutionGenerationChanged);
    }

    if qualifier.descriptor() != expected.verifier {
        return Err(CapabilityTimeEffectRevalidationError::VerifierDescriptorMismatchBefore);
    }

    let subject = CapabilityTimeRevalidationSubjectV1 {
        admitted_effect: rebound.binding(),
        original_execution_admission,
        effect_eligibility_valid_until: rebound.binding().effect_eligibility_valid_until,
    };

    let receipt = qualifier
        .revalidate_execution_generation(&subject, &expected.generation)
        .map_err(CapabilityTimeEffectRevalidationError::Verifier)?;

    if qualifier.descriptor() != expected.verifier {
        return Err(CapabilityTimeEffectRevalidationError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(CapabilityTimeEffectRevalidationError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.verifier {
        return Err(CapabilityTimeEffectRevalidationError::ReceiptVerifierMismatch);
    }
    if receipt.subject != subject {
        return Err(CapabilityTimeEffectRevalidationError::SubjectMismatch);
    }
    if receipt.generation != expected.generation {
        return Err(CapabilityTimeEffectRevalidationError::ExecutionGenerationMismatch);
    }
    if receipt.valid_until > expected.verifier.valid_until {
        return Err(CapabilityTimeEffectRevalidationError::ReceiptOutlivesVerifier);
    }
    if receipt.valid_until > expected.generation.valid_until {
        return Err(CapabilityTimeEffectRevalidationError::ReceiptOutlivesExecutionGeneration);
    }
    if receipt.valid_until > subject.effect_eligibility_valid_until {
        return Err(CapabilityTimeEffectRevalidationError::ReceiptOutlivesEffectEligibility);
    }

    let valid_until = derive_capability_ready_until::<Q::Error>(
        subject.effect_eligibility_valid_until,
        expected.verifier.valid_until,
        expected.generation.valid_until,
        receipt.valid_until,
        current_time.valid_until(),
        current_time.latest_possible_unix_ms(),
    )?;

    Ok(CapabilityTimeRevalidatedEffectV1 {
        rebound,
        expected,
        receipt,
        current_time,
        valid_until,
        _qualifier: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_execution_surface_admission::{
        ActuatorGenerationV1, ActuatorHealthGenerationCommitment,
        ActuatorPolicyGenerationCommitment, AuthenticatedCarrierGenerationV1,
        AuthenticatedSessionGenerationV1, AuthenticatedSessionTranscriptCommitment,
        ExecutionGenerationTimeBasisV1, ExecutionSurfaceIdentityCommitment,
        LogicalSessionIdCommitment, OperationCarrierCommitment,
        TransportAuthorityGenerationCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn generation(transcript: u8, valid_until: u64) -> ExecutionSurfaceGenerationV1 {
        ExecutionSurfaceGenerationV1 {
            surface_identity: ExecutionSurfaceIdentityCommitment::from_bytes(bytes(1)),
            carrier: AuthenticatedCarrierGenerationV1 {
                session: AuthenticatedSessionGenerationV1 {
                    logical_session: LogicalSessionIdCommitment::from_bytes(bytes(2)),
                    transcript: AuthenticatedSessionTranscriptCommitment::from_bytes(bytes(transcript)),
                },
                transport_authority: TransportAuthorityGenerationCommitment::from_bytes(bytes(4)),
            },
            actuator: ActuatorGenerationV1 {
                policy: ActuatorPolicyGenerationCommitment::from_bytes(bytes(5)),
                health: ActuatorHealthGenerationCommitment::from_bytes(bytes(6)),
            },
            operation_carrier: OperationCarrierCommitment::from_bytes(bytes(7)),
            time_basis: ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc,
            valid_until,
        }
    }

    #[test]
    fn same_logical_session_with_different_transcript_is_different_live_generation() {
        assert_ne!(generation(3, 100), generation(9, 100));
    }

    #[test]
    fn generation_fixture_uses_explicit_unix_millisecond_basis() {
        assert_eq!(
            generation(3, 100).time_basis,
            ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc
        );
    }

    #[test]
    fn natural_expiry_selects_oldest_capability_boundary() {
        assert_eq!(
            derive_capability_ready_until::<core::convert::Infallible>(90, 150, 120, 130, 140, 80),
            Ok(90)
        );
    }

    #[test]
    fn stale_effect_eligibility_fails_closed() {
        assert_eq!(
            derive_capability_ready_until::<core::convert::Infallible>(79, 150, 120, 130, 140, 80),
            Err(CapabilityTimeEffectRevalidationError::EffectEligibilityAlreadyExpired)
        );
    }
}
