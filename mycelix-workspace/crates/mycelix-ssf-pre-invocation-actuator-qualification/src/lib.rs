// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fresh pre-invocation qualification of one exact durable SSF claimed-effect
//! lineage against the exact live execution/provider/actuator environment.
//!
//! This crate performs no provider resolution and no actuator invocation. It
//! makes the actuator's recovery semantics explicit before any external call.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_effect_capability_claim_rebind::{
    ReboundClaimedEffectLineageBindingV1, ReboundClaimedEffectLineageV1,
};
use mycelix_ssf_execution_surface_admission::{
    ExecutionGenerationTimeBasisV1, ExecutionSurfaceGenerationV1,
};
use mycelix_ssf_source_owned_operation_material::{
    SourceOwnedOperationHandleCommitment, SourceOwnedOperationProviderDescriptorV1,
    SourceOwnedOperationProviderTimeBasisV1,
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

digest_type!(PreInvocationVerifierIdentityCommitment);
digest_type!(PreInvocationVerifierPolicyCommitment);
digest_type!(ActuatorStableIdentityCommitment);
digest_type!(ActuatorExecutionPolicyCommitment);
digest_type!(PreInvocationHandleResolutionCommitment);
digest_type!(ActuatorDeduplicationEvidenceCommitment);
digest_type!(PreInvocationActuatorQualificationReceiptCommitment);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PreInvocationTimeBasisV1 {
    UnixMillisecondsUtc,
}

/// Recovery semantics actually provided by the external actuator adapter.
///
/// None of these variants is named "exactly once". Stronger claims require an
/// actuator-specific proof beyond this provider-neutral contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ActuatorRecoveryModeV1 {
    /// The actuator atomically uses the durable claim record as its execution
    /// deduplication key.
    TransactionalClaimKey,
    /// Repeating the exact same claim-keyed operation is semantically
    /// idempotent.
    IdempotentClaimKey,
    /// No safe replay theorem is established. Any ambiguous invocation result
    /// must prohibit automatic retry.
    NonIdempotentNoAutomaticRetry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PreInvocationVerifierDescriptorV1 {
    pub stable_identity: PreInvocationVerifierIdentityCommitment,
    pub policy: PreInvocationVerifierPolicyCommitment,
    pub generation: u64,
    pub time_basis: PreInvocationTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ActuatorExecutionDescriptorV1 {
    pub stable_identity: ActuatorStableIdentityCommitment,
    pub policy: ActuatorExecutionPolicyCommitment,
    pub generation: u64,
    pub execution_generation: ExecutionSurfaceGenerationV1,
    pub provider: SourceOwnedOperationProviderDescriptorV1,
    pub recovery_mode: ActuatorRecoveryModeV1,
    pub time_basis: PreInvocationTimeBasisV1,
    pub valid_until: u64,
}

/// Independent local expectation. The qualifier does not choose the verifier
/// or actuator generation it is asked to attest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedPreInvocationActuatorProfileV1 {
    verifier: PreInvocationVerifierDescriptorV1,
    actuator: ActuatorExecutionDescriptorV1,
}

impl ExpectedPreInvocationActuatorProfileV1 {
    pub const fn from_trusted_configuration(
        verifier: PreInvocationVerifierDescriptorV1,
        actuator: ActuatorExecutionDescriptorV1,
    ) -> Self {
        Self { verifier, actuator }
    }

    pub const fn verifier(&self) -> PreInvocationVerifierDescriptorV1 {
        self.verifier
    }

    pub const fn actuator(&self) -> ActuatorExecutionDescriptorV1 {
        self.actuator
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreInvocationActuatorSubjectV1 {
    pub claimed_lineage: ReboundClaimedEffectLineageBindingV1,
    pub claim_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreInvocationActuatorQualificationReceiptV1 {
    pub schema_version: u16,
    pub verifier: PreInvocationVerifierDescriptorV1,
    pub subject: PreInvocationActuatorSubjectV1,
    pub actuator: ActuatorExecutionDescriptorV1,
    pub execution_generation: ExecutionSurfaceGenerationV1,
    pub provider: SourceOwnedOperationProviderDescriptorV1,
    pub operation_handle: SourceOwnedOperationHandleCommitment,
    pub handle_resolution: PreInvocationHandleResolutionCommitment,
    pub recovery_mode: ActuatorRecoveryModeV1,
    pub deduplication_evidence: ActuatorDeduplicationEvidenceCommitment,
    pub valid_until: u64,
    pub receipt_commitment: PreInvocationActuatorQualificationReceiptCommitment,
}

pub trait PreInvocationActuatorQualifierV1 {
    type Error;

    fn descriptor(&self) -> PreInvocationVerifierDescriptorV1;

    fn qualify_pre_invocation(
        &self,
        subject: &PreInvocationActuatorSubjectV1,
        actuator: &ActuatorExecutionDescriptorV1,
    ) -> Result<PreInvocationActuatorQualificationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreInvocationActuatorQualificationError<E> {
    UnsupportedVerifierTimeBasis,
    UnsupportedActuatorTimeBasis,
    UnsupportedExecutionTimeBasis,
    UnsupportedProviderTimeBasis,
    VerifierDescriptorMismatchBefore,
    ExecutionGenerationMismatch,
    ProviderGenerationMismatch,
    Qualifier(E),
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    ReceiptSubjectMismatch,
    ReceiptActuatorMismatch,
    ReceiptExecutionGenerationMismatch,
    ReceiptProviderMismatch,
    ReceiptOperationHandleMismatch,
    ReceiptRecoveryModeMismatch,
    ReceiptOutlivesVerifier,
    ReceiptOutlivesActuator,
    ReceiptOutlivesExecutionGeneration,
    ReceiptOutlivesProvider,
    ReceiptOutlivesClaimEligibility,
    ClaimEligibilityAlreadyExpired,
    VerifierAlreadyExpired,
    ActuatorAlreadyExpired,
    ExecutionGenerationAlreadyExpired,
    ProviderAlreadyExpired,
    ReceiptAlreadyExpired,
    TrustedTimeAlreadyExpired,
}

fn derive_pre_invocation_valid_until<E>(
    claim_eligibility_valid_until: u64,
    capability_valid_until: u64,
    verifier_valid_until: u64,
    actuator_valid_until: u64,
    execution_valid_until: u64,
    provider_valid_until: u64,
    receipt_valid_until: u64,
    trusted_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, PreInvocationActuatorQualificationError<E>> {
    if claim_eligibility_valid_until < latest_possible_now || capability_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::ClaimEligibilityAlreadyExpired);
    }
    if verifier_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::VerifierAlreadyExpired);
    }
    if actuator_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::ActuatorAlreadyExpired);
    }
    if execution_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::ExecutionGenerationAlreadyExpired);
    }
    if provider_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::ProviderAlreadyExpired);
    }
    if receipt_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::ReceiptAlreadyExpired);
    }
    if trusted_time_valid_until < latest_possible_now {
        return Err(PreInvocationActuatorQualificationError::TrustedTimeAlreadyExpired);
    }

    Ok(claim_eligibility_valid_until
        .min(capability_valid_until)
        .min(verifier_valid_until)
        .min(actuator_valid_until)
        .min(execution_valid_until)
        .min(provider_valid_until)
        .min(receipt_valid_until)
        .min(trusted_time_valid_until))
}

pub struct PreInvocationQualifiedClaimedEffectV1<Q, TQ> {
    lineage: ReboundClaimedEffectLineageV1,
    expected: ExpectedPreInvocationActuatorProfileV1,
    receipt: PreInvocationActuatorQualificationReceiptV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    valid_until: u64,
    _qualifier: PhantomData<fn() -> Q>,
}

impl<Q, TQ> PreInvocationQualifiedClaimedEffectV1<Q, TQ> {
    pub const fn lineage(&self) -> &ReboundClaimedEffectLineageV1 {
        &self.lineage
    }

    pub const fn expected(&self) -> ExpectedPreInvocationActuatorProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> PreInvocationActuatorQualificationReceiptV1 {
        self.receipt
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn recovery_mode(&self) -> ActuatorRecoveryModeV1 {
        self.receipt.recovery_mode
    }

    pub const fn durable_claim_record_is_execution_key(&self) -> bool {
        true
    }

    pub const fn exact_environment_revalidated(&self) -> bool {
        true
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }

    pub const fn external_exactly_once_claimed(&self) -> bool {
        false
    }

    pub const fn ambiguous_outcome_requires_reconciliation(&self) -> bool {
        true
    }

    pub const fn automatic_retry_for_non_idempotent_mode(&self) -> bool {
        false
    }
}

pub fn qualify_claimed_effect_pre_invocation<Q, TQ>(
    lineage: ReboundClaimedEffectLineageV1,
    expected: ExpectedPreInvocationActuatorProfileV1,
    qualifier: &Q,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<
    PreInvocationQualifiedClaimedEffectV1<Q, TQ>,
    PreInvocationActuatorQualificationError<Q::Error>,
>
where
    Q: PreInvocationActuatorQualifierV1,
{
    if expected.verifier.time_basis != PreInvocationTimeBasisV1::UnixMillisecondsUtc {
        return Err(PreInvocationActuatorQualificationError::UnsupportedVerifierTimeBasis);
    }
    if expected.actuator.time_basis != PreInvocationTimeBasisV1::UnixMillisecondsUtc {
        return Err(PreInvocationActuatorQualificationError::UnsupportedActuatorTimeBasis);
    }
    if expected.actuator.execution_generation.time_basis
        != ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc
    {
        return Err(PreInvocationActuatorQualificationError::UnsupportedExecutionTimeBasis);
    }
    if expected.actuator.provider.time_basis
        != SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc
    {
        return Err(PreInvocationActuatorQualificationError::UnsupportedProviderTimeBasis);
    }
    if qualifier.descriptor() != expected.verifier {
        return Err(PreInvocationActuatorQualificationError::VerifierDescriptorMismatchBefore);
    }

    let binding = lineage.binding();
    let capability = binding.capability;
    if expected.actuator.execution_generation != capability.execution_generation {
        return Err(PreInvocationActuatorQualificationError::ExecutionGenerationMismatch);
    }
    if expected.actuator.provider != capability.provider {
        return Err(PreInvocationActuatorQualificationError::ProviderGenerationMismatch);
    }

    let subject = PreInvocationActuatorSubjectV1 {
        claimed_lineage: binding,
        claim_eligibility_valid_until: lineage.claim_eligibility_valid_until(),
    };
    let receipt = qualifier
        .qualify_pre_invocation(&subject, &expected.actuator)
        .map_err(PreInvocationActuatorQualificationError::Qualifier)?;

    if qualifier.descriptor() != expected.verifier {
        return Err(PreInvocationActuatorQualificationError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(PreInvocationActuatorQualificationError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.verifier {
        return Err(PreInvocationActuatorQualificationError::ReceiptVerifierMismatch);
    }
    if receipt.subject != subject {
        return Err(PreInvocationActuatorQualificationError::ReceiptSubjectMismatch);
    }
    if receipt.actuator != expected.actuator {
        return Err(PreInvocationActuatorQualificationError::ReceiptActuatorMismatch);
    }
    if receipt.execution_generation != expected.actuator.execution_generation {
        return Err(PreInvocationActuatorQualificationError::ReceiptExecutionGenerationMismatch);
    }
    if receipt.provider != expected.actuator.provider {
        return Err(PreInvocationActuatorQualificationError::ReceiptProviderMismatch);
    }
    if receipt.operation_handle != capability.operation_handle {
        return Err(PreInvocationActuatorQualificationError::ReceiptOperationHandleMismatch);
    }
    if receipt.recovery_mode != expected.actuator.recovery_mode {
        return Err(PreInvocationActuatorQualificationError::ReceiptRecoveryModeMismatch);
    }
    if receipt.valid_until > expected.verifier.valid_until {
        return Err(PreInvocationActuatorQualificationError::ReceiptOutlivesVerifier);
    }
    if receipt.valid_until > expected.actuator.valid_until {
        return Err(PreInvocationActuatorQualificationError::ReceiptOutlivesActuator);
    }
    if receipt.valid_until > expected.actuator.execution_generation.valid_until {
        return Err(PreInvocationActuatorQualificationError::ReceiptOutlivesExecutionGeneration);
    }
    if receipt.valid_until > expected.actuator.provider.valid_until {
        return Err(PreInvocationActuatorQualificationError::ReceiptOutlivesProvider);
    }
    if receipt.valid_until > lineage.claim_eligibility_valid_until() {
        return Err(PreInvocationActuatorQualificationError::ReceiptOutlivesClaimEligibility);
    }

    let valid_until = derive_pre_invocation_valid_until::<Q::Error>(
        lineage.claim_eligibility_valid_until(),
        capability.valid_until,
        expected.verifier.valid_until,
        expected.actuator.valid_until,
        expected.actuator.execution_generation.valid_until,
        expected.actuator.provider.valid_until,
        receipt.valid_until,
        current_time.valid_until(),
        current_time.latest_possible_unix_ms(),
    )?;

    Ok(PreInvocationQualifiedClaimedEffectV1 {
        lineage,
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

    #[test]
    fn recovery_modes_are_not_collapsed() {
        assert_ne!(
            ActuatorRecoveryModeV1::TransactionalClaimKey,
            ActuatorRecoveryModeV1::NonIdempotentNoAutomaticRetry
        );
        assert_ne!(
            ActuatorRecoveryModeV1::IdempotentClaimKey,
            ActuatorRecoveryModeV1::TransactionalClaimKey
        );
    }

    #[test]
    fn natural_expiry_uses_oldest_boundary() {
        assert_eq!(
            derive_pre_invocation_valid_until::<core::convert::Infallible>(
                150, 140, 130, 120, 110, 100, 95, 90, 80
            ),
            Ok(90)
        );
    }

    #[test]
    fn stale_claim_fails_closed() {
        assert_eq!(
            derive_pre_invocation_valid_until::<core::convert::Infallible>(
                79, 140, 130, 120, 110, 100, 95, 90, 80
            ),
            Err(PreInvocationActuatorQualificationError::ClaimEligibilityAlreadyExpired)
        );
    }
}
