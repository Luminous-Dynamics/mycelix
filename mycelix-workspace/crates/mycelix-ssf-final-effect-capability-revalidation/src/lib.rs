// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Final fresh security proof before transient SSF effect-capability construction.
//!
//! Durable capability issuance is historical single-use evidence. It does not
//! prove that the exact execution generation, provider generation, opaque handle,
//! or time are still current now. This crate performs that final proof and still
//! does not mint or execute the transient capability.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_authorization::VerifierDescriptorV1;
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_effect_capability_issuance_rebind::{
    ReboundEffectCapabilityIssuanceBindingV1, ReboundIssuedEffectCapabilityLineageV1,
};
use mycelix_ssf_execution_surface_admission::{
    ExecutionGenerationTimeBasisV1, ExecutionSurfaceGenerationV1,
};
use mycelix_ssf_source_owned_operation_material::{
    OperationEncodingCommitment, OperationPayloadCommitment,
    SourceOwnedOperationHandleCommitment, SourceOwnedOperationMaterialReceiptV1,
    SourceOwnedOperationProviderDescriptorV1, SourceOwnedOperationProviderTimeBasisV1,
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

digest_type!(OperationHandleResolutionCommitment);
digest_type!(FinalEffectCapabilityRevalidationReceiptCommitment);

/// Independently expected live environment. The execution/provider values are
/// supplied separately from the final qualifier so the verifier cannot choose a
/// replacement generation to authorize.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedFinalEffectCapabilityProfileV1 {
    verifier: VerifierDescriptorV1,
    execution_generation: ExecutionSurfaceGenerationV1,
    provider: SourceOwnedOperationProviderDescriptorV1,
}

impl ExpectedFinalEffectCapabilityProfileV1 {
    pub const fn from_authenticated_environment(
        verifier: VerifierDescriptorV1,
        execution_generation: ExecutionSurfaceGenerationV1,
        provider: SourceOwnedOperationProviderDescriptorV1,
    ) -> Self {
        Self {
            verifier,
            execution_generation,
            provider,
        }
    }

    pub const fn verifier(&self) -> VerifierDescriptorV1 {
        self.verifier
    }

    pub const fn execution_generation(&self) -> ExecutionSurfaceGenerationV1 {
        self.execution_generation
    }

    pub const fn provider(&self) -> SourceOwnedOperationProviderDescriptorV1 {
        self.provider
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FinalEffectCapabilityRevalidationSubjectV1 {
    pub issued_capability: ReboundEffectCapabilityIssuanceBindingV1,
    pub material_receipt: SourceOwnedOperationMaterialReceiptV1,
    pub capability_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FinalEffectCapabilityRevalidationReceiptV1 {
    pub schema_version: u16,
    pub verifier: VerifierDescriptorV1,
    pub subject: FinalEffectCapabilityRevalidationSubjectV1,
    pub execution_generation: ExecutionSurfaceGenerationV1,
    pub provider: SourceOwnedOperationProviderDescriptorV1,
    pub operation_handle: SourceOwnedOperationHandleCommitment,
    pub handle_resolution: OperationHandleResolutionCommitment,
    pub valid_until: u64,
    pub receipt_commitment: FinalEffectCapabilityRevalidationReceiptCommitment,
}

impl FinalEffectCapabilityRevalidationReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        verifier: VerifierDescriptorV1,
        subject: FinalEffectCapabilityRevalidationSubjectV1,
        execution_generation: ExecutionSurfaceGenerationV1,
        provider: SourceOwnedOperationProviderDescriptorV1,
        operation_handle: SourceOwnedOperationHandleCommitment,
        handle_resolution: OperationHandleResolutionCommitment,
        valid_until: u64,
        receipt_commitment: FinalEffectCapabilityRevalidationReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            verifier,
            subject,
            execution_generation,
            provider,
            operation_handle,
            handle_resolution,
            valid_until,
            receipt_commitment,
        }
    }
}

pub trait FinalEffectCapabilityQualifierV1 {
    type Error;

    fn descriptor(&self) -> VerifierDescriptorV1;

    fn qualify_final_effect_capability(
        &self,
        subject: &FinalEffectCapabilityRevalidationSubjectV1,
        expected_execution_generation: &ExecutionSurfaceGenerationV1,
        expected_provider: &SourceOwnedOperationProviderDescriptorV1,
        expected_handle: SourceOwnedOperationHandleCommitment,
    ) -> Result<FinalEffectCapabilityRevalidationReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinalEffectCapabilityRevalidationError<E> {
    UnsupportedExecutionTimeBasis,
    UnsupportedProviderTimeBasis,
    LiveExecutionGenerationChanged,
    LiveProviderGenerationChanged,
    VerifierDescriptorMismatchBefore,
    Verifier(E),
    VerifierDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptVerifierMismatch,
    SubjectMismatch,
    ExecutionGenerationMismatch,
    ProviderMismatch,
    OperationHandleMismatch,
    ReceiptOutlivesVerifier,
    ReceiptOutlivesExecutionGeneration,
    ReceiptOutlivesProvider,
    ReceiptOutlivesMaterial,
    ReceiptOutlivesCapabilityEligibility,
    CapabilityEligibilityAlreadyExpired,
    MaterialAlreadyExpired,
    ExecutionGenerationAlreadyExpired,
    ProviderAlreadyExpired,
    ReceiptAlreadyExpired,
    TrustedTimeAlreadyExpired,
}

#[allow(clippy::too_many_arguments)]
fn derive_final_ready_until<E>(
    capability_eligibility_valid_until: u64,
    material_valid_until: u64,
    verifier_valid_until: u64,
    execution_generation_valid_until: u64,
    provider_valid_until: u64,
    receipt_valid_until: u64,
    trusted_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, FinalEffectCapabilityRevalidationError<E>> {
    if capability_eligibility_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::CapabilityEligibilityAlreadyExpired);
    }
    if material_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::MaterialAlreadyExpired);
    }
    if execution_generation_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::ExecutionGenerationAlreadyExpired);
    }
    if provider_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::ProviderAlreadyExpired);
    }
    if receipt_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptAlreadyExpired);
    }
    if trusted_time_valid_until < latest_possible_now {
        return Err(FinalEffectCapabilityRevalidationError::TrustedTimeAlreadyExpired);
    }

    Ok(capability_eligibility_valid_until
        .min(material_valid_until)
        .min(verifier_valid_until)
        .min(execution_generation_valid_until)
        .min(provider_valid_until)
        .min(receipt_valid_until)
        .min(trusted_time_valid_until))
}

/// Final pre-capability token. It establishes current readiness but still
/// contains no transient effect capability and cannot invoke an actuator.
pub struct FinalEffectCapabilityReadyV1<S, M, Q, TQ> {
    lineage: ReboundIssuedEffectCapabilityLineageV1<S, M>,
    expected: ExpectedFinalEffectCapabilityProfileV1,
    receipt: FinalEffectCapabilityRevalidationReceiptV1,
    current_time: QualifiedCurrentTimeV1<TQ>,
    valid_until: u64,
    _qualifier: PhantomData<Q>,
}

impl<S, M, Q, TQ> FinalEffectCapabilityReadyV1<S, M, Q, TQ> {
    pub const fn lineage(&self) -> &ReboundIssuedEffectCapabilityLineageV1<S, M> {
        &self.lineage
    }

    pub const fn expected(&self) -> ExpectedFinalEffectCapabilityProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> FinalEffectCapabilityRevalidationReceiptV1 {
        self.receipt
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<TQ> {
        &self.current_time
    }

    pub const fn operation_handle(&self) -> SourceOwnedOperationHandleCommitment {
        self.receipt.operation_handle
    }

    pub const fn payload_commitment(&self) -> OperationPayloadCommitment {
        self.receipt.subject.material_receipt.payload_commitment
    }

    pub const fn encoding_commitment(&self) -> OperationEncodingCommitment {
        self.receipt.subject.material_receipt.encoding_commitment
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn exact_live_environment_revalidated(&self) -> bool {
        true
    }

    pub const fn exact_handle_revalidated(&self) -> bool {
        true
    }

    pub const fn eligible_for_transient_move_only_capability(&self) -> bool {
        true
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn revalidate_final_effect_capability<S, M, Q, TQ>(
    lineage: ReboundIssuedEffectCapabilityLineageV1<S, M>,
    expected: ExpectedFinalEffectCapabilityProfileV1,
    qualifier: &Q,
    current_time: QualifiedCurrentTimeV1<TQ>,
) -> Result<FinalEffectCapabilityReadyV1<S, M, Q, TQ>, FinalEffectCapabilityRevalidationError<Q::Error>>
where
    Q: FinalEffectCapabilityQualifierV1,
{
    if expected.execution_generation.time_basis != ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc {
        return Err(FinalEffectCapabilityRevalidationError::UnsupportedExecutionTimeBasis);
    }
    if expected.provider.time_basis != SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc {
        return Err(FinalEffectCapabilityRevalidationError::UnsupportedProviderTimeBasis);
    }

    let issued_binding = lineage.binding();
    let issuance_subject = issued_binding.manifest.subject();
    let material_receipt = issuance_subject.material_receipt;
    let originally_admitted_generation =
        material_receipt.subject.capability_time_revalidation.generation;

    if expected.execution_generation != originally_admitted_generation {
        return Err(FinalEffectCapabilityRevalidationError::LiveExecutionGenerationChanged);
    }
    if expected.provider != material_receipt.provider {
        return Err(FinalEffectCapabilityRevalidationError::LiveProviderGenerationChanged);
    }

    if qualifier.descriptor() != expected.verifier {
        return Err(FinalEffectCapabilityRevalidationError::VerifierDescriptorMismatchBefore);
    }

    let subject = FinalEffectCapabilityRevalidationSubjectV1 {
        issued_capability: issued_binding,
        material_receipt,
        capability_eligibility_valid_until: issued_binding.capability_eligibility_valid_until,
    };
    let expected_handle = material_receipt.handle;

    let receipt = qualifier
        .qualify_final_effect_capability(
            &subject,
            &expected.execution_generation,
            &expected.provider,
            expected_handle,
        )
        .map_err(FinalEffectCapabilityRevalidationError::Verifier)?;

    if qualifier.descriptor() != expected.verifier {
        return Err(FinalEffectCapabilityRevalidationError::VerifierDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(FinalEffectCapabilityRevalidationError::UnsupportedReceiptSchema);
    }
    if receipt.verifier != expected.verifier {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptVerifierMismatch);
    }
    if receipt.subject != subject {
        return Err(FinalEffectCapabilityRevalidationError::SubjectMismatch);
    }
    if receipt.execution_generation != expected.execution_generation {
        return Err(FinalEffectCapabilityRevalidationError::ExecutionGenerationMismatch);
    }
    if receipt.provider != expected.provider {
        return Err(FinalEffectCapabilityRevalidationError::ProviderMismatch);
    }
    if receipt.operation_handle != expected_handle {
        return Err(FinalEffectCapabilityRevalidationError::OperationHandleMismatch);
    }
    if receipt.valid_until > expected.verifier.valid_until {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptOutlivesVerifier);
    }
    if receipt.valid_until > expected.execution_generation.valid_until {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptOutlivesExecutionGeneration);
    }
    if receipt.valid_until > expected.provider.valid_until {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptOutlivesProvider);
    }
    if receipt.valid_until > issuance_subject.material_valid_until {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptOutlivesMaterial);
    }
    if receipt.valid_until > issued_binding.capability_eligibility_valid_until {
        return Err(FinalEffectCapabilityRevalidationError::ReceiptOutlivesCapabilityEligibility);
    }

    let valid_until = derive_final_ready_until::<Q::Error>(
        issued_binding.capability_eligibility_valid_until,
        issuance_subject.material_valid_until,
        expected.verifier.valid_until,
        expected.execution_generation.valid_until,
        expected.provider.valid_until,
        receipt.valid_until,
        current_time.valid_until(),
        current_time.latest_possible_unix_ms(),
    )?;

    Ok(FinalEffectCapabilityReadyV1 {
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
    use mycelix_ssf_source_owned_operation_material::{
        SourceOwnedOperationProviderIdentityCommitment,
        SourceOwnedOperationProviderPolicyCommitment,
    };

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[test]
    fn natural_expiry_selects_oldest_final_boundary() {
        assert_eq!(
            derive_final_ready_until::<core::convert::Infallible>(
                90, 120, 150, 130, 140, 110, 125, 80,
            ),
            Ok(90)
        );
    }

    #[test]
    fn stale_inherited_capability_fails_closed() {
        assert_eq!(
            derive_final_ready_until::<core::convert::Infallible>(
                79, 120, 150, 130, 140, 110, 125, 80,
            ),
            Err(FinalEffectCapabilityRevalidationError::CapabilityEligibilityAlreadyExpired)
        );
    }

    #[test]
    fn handle_resolution_commitment_is_distinct_from_operation_handle() {
        assert_ne!(
            OperationHandleResolutionCommitment::from_bytes(bytes(1)).as_bytes(),
            SourceOwnedOperationHandleCommitment::from_bytes(bytes(2)).as_bytes()
        );
    }

    #[test]
    fn provider_generation_change_changes_exact_provider_identity() {
        let base = SourceOwnedOperationProviderDescriptorV1 {
            stable_identity: SourceOwnedOperationProviderIdentityCommitment::from_bytes(bytes(1)),
            policy: SourceOwnedOperationProviderPolicyCommitment::from_bytes(bytes(2)),
            generation: 3,
            time_basis: SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 100,
        };
        let mut changed = base;
        changed.generation = 4;
        assert_ne!(base, changed);
    }
}
