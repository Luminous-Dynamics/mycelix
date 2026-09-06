// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-owned immutable operation material for an SSF possible-effect lineage.
//!
//! The public materialization API deliberately accepts no payload bytes. An
//! independently expected concrete provider materializes the operation inside
//! its own trust domain and returns an opaque handle plus commitments bound to
//! the exact operation carrier already admitted by the execution generation.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use mycelix_ssf_capability_time_effect_revalidation::{
    CapabilityTimeRevalidatedEffectV1, CapabilityTimeRevalidationReceiptV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::QualifiedCurrentTimeV1;
use mycelix_ssf_effect_admission_rebind::ReboundEffectAdmissionBindingV1;
use mycelix_ssf_execution_surface_admission::OperationCarrierCommitment;

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

digest_type!(SourceOwnedOperationProviderIdentityCommitment);
digest_type!(SourceOwnedOperationProviderPolicyCommitment);
digest_type!(SourceOwnedOperationHandleCommitment);
digest_type!(OperationPayloadCommitment);
digest_type!(OperationEncodingCommitment);
digest_type!(SourceOwnedOperationMaterialReceiptCommitment);

/// Closed v0.1 basis for provider/material freshness comparisons.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SourceOwnedOperationProviderTimeBasisV1 {
    UnixMillisecondsUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SourceOwnedOperationProviderDescriptorV1 {
    pub stable_identity: SourceOwnedOperationProviderIdentityCommitment,
    pub policy: SourceOwnedOperationProviderPolicyCommitment,
    pub generation: u64,
    pub time_basis: SourceOwnedOperationProviderTimeBasisV1,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpectedSourceOwnedOperationProviderProfileV1 {
    descriptor: SourceOwnedOperationProviderDescriptorV1,
}

impl ExpectedSourceOwnedOperationProviderProfileV1 {
    pub const fn from_trusted_configuration(
        descriptor: SourceOwnedOperationProviderDescriptorV1,
    ) -> Self {
        Self { descriptor }
    }

    pub const fn descriptor(&self) -> SourceOwnedOperationProviderDescriptorV1 {
        self.descriptor
    }
}

/// Exact immutable subject the provider must materialize.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceOwnedOperationMaterializationSubjectV1 {
    pub admitted_effect: ReboundEffectAdmissionBindingV1,
    pub capability_time_revalidation: CapabilityTimeRevalidationReceiptV1,
    pub operation_carrier: OperationCarrierCommitment,
    pub capability_ready_valid_until: u64,
}

/// Provider receipt for immutable operation material.
///
/// The handle is opaque to SSF. The provider/actuator adapter owns resolution
/// from this handle to immutable source-owned bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceOwnedOperationMaterialReceiptV1 {
    pub schema_version: u16,
    pub provider: SourceOwnedOperationProviderDescriptorV1,
    pub subject: SourceOwnedOperationMaterializationSubjectV1,
    pub operation_carrier: OperationCarrierCommitment,
    pub handle: SourceOwnedOperationHandleCommitment,
    pub payload_commitment: OperationPayloadCommitment,
    pub encoding_commitment: OperationEncodingCommitment,
    pub payload_length: u64,
    pub valid_until: u64,
    pub receipt_commitment: SourceOwnedOperationMaterialReceiptCommitment,
}

impl SourceOwnedOperationMaterialReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        provider: SourceOwnedOperationProviderDescriptorV1,
        subject: SourceOwnedOperationMaterializationSubjectV1,
        operation_carrier: OperationCarrierCommitment,
        handle: SourceOwnedOperationHandleCommitment,
        payload_commitment: OperationPayloadCommitment,
        encoding_commitment: OperationEncodingCommitment,
        payload_length: u64,
        valid_until: u64,
        receipt_commitment: SourceOwnedOperationMaterialReceiptCommitment,
    ) -> Self {
        Self {
            schema_version: SSF_SCHEMA_V1,
            provider,
            subject,
            operation_carrier,
            handle,
            payload_commitment,
            encoding_commitment,
            payload_length,
            valid_until,
            receipt_commitment,
        }
    }
}

/// Concrete provider that owns and materializes operation bytes internally.
///
/// There is intentionally no payload argument. Implementations must derive the
/// immutable material from the exact subject/provider-owned state and return an
/// opaque handle plus commitments.
pub trait SourceOwnedOperationProviderV1 {
    type Error;

    fn descriptor(&self) -> SourceOwnedOperationProviderDescriptorV1;

    fn materialize_source_owned_operation(
        &self,
        subject: &SourceOwnedOperationMaterializationSubjectV1,
    ) -> Result<SourceOwnedOperationMaterialReceiptV1, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceOwnedOperationMaterializationError<E> {
    UnsupportedProviderTimeBasis,
    ProviderDescriptorMismatchBefore,
    Provider(E),
    ProviderDescriptorMismatchAfter,
    UnsupportedReceiptSchema,
    ReceiptProviderMismatch,
    SubjectMismatch,
    OperationCarrierMismatch,
    ReceiptOutlivesProvider,
    ReceiptOutlivesCapabilityReadiness,
    CapabilityReadinessAlreadyExpired,
    ProviderAlreadyExpired,
    ReceiptAlreadyExpired,
    TrustedTimeAlreadyExpired,
}

fn derive_material_valid_until<E>(
    capability_ready_valid_until: u64,
    provider_valid_until: u64,
    receipt_valid_until: u64,
    trusted_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, SourceOwnedOperationMaterializationError<E>> {
    if capability_ready_valid_until < latest_possible_now {
        return Err(SourceOwnedOperationMaterializationError::CapabilityReadinessAlreadyExpired);
    }
    if provider_valid_until < latest_possible_now {
        return Err(SourceOwnedOperationMaterializationError::ProviderAlreadyExpired);
    }
    if receipt_valid_until < latest_possible_now {
        return Err(SourceOwnedOperationMaterializationError::ReceiptAlreadyExpired);
    }
    if trusted_time_valid_until < latest_possible_now {
        return Err(SourceOwnedOperationMaterializationError::TrustedTimeAlreadyExpired);
    }

    Ok(capability_ready_valid_until
        .min(provider_valid_until)
        .min(receipt_valid_until)
        .min(trusted_time_valid_until))
}

/// Exact provider-owned immutable operation material bound to one qualified
/// possible-effect lineage.
///
/// This token is still not a move-only effect capability.
pub struct SourceOwnedOperationMaterialV1<P, S, C, Q, TQ, MTQ> {
    effect: CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ>,
    expected: ExpectedSourceOwnedOperationProviderProfileV1,
    receipt: SourceOwnedOperationMaterialReceiptV1,
    current_time: QualifiedCurrentTimeV1<MTQ>,
    valid_until: u64,
    _provider: PhantomData<P>,
}

impl<P, S, C, Q, TQ, MTQ> SourceOwnedOperationMaterialV1<P, S, C, Q, TQ, MTQ> {
    pub const fn effect(&self) -> &CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ> {
        &self.effect
    }

    pub const fn expected(&self) -> ExpectedSourceOwnedOperationProviderProfileV1 {
        self.expected
    }

    pub const fn receipt(&self) -> SourceOwnedOperationMaterialReceiptV1 {
        self.receipt
    }

    pub const fn current_time(&self) -> &QualifiedCurrentTimeV1<MTQ> {
        &self.current_time
    }

    pub const fn operation_handle(&self) -> SourceOwnedOperationHandleCommitment {
        self.receipt.handle
    }

    pub const fn payload_commitment(&self) -> OperationPayloadCommitment {
        self.receipt.payload_commitment
    }

    pub const fn encoding_commitment(&self) -> OperationEncodingCommitment {
        self.receipt.encoding_commitment
    }

    pub const fn valid_until(&self) -> u64 {
        self.valid_until
    }

    pub const fn materialized_without_caller_payload(&self) -> bool {
        true
    }

    pub const fn eligible_for_move_only_capability_issuance(&self) -> bool {
        true
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn materialize_source_owned_operation<P, S, C, Q, TQ, MTQ>(
    effect: CapabilityTimeRevalidatedEffectV1<S, C, Q, TQ>,
    expected: ExpectedSourceOwnedOperationProviderProfileV1,
    provider: &P,
    current_time: QualifiedCurrentTimeV1<MTQ>,
) -> Result<
    SourceOwnedOperationMaterialV1<P, S, C, Q, TQ, MTQ>,
    SourceOwnedOperationMaterializationError<P::Error>,
>
where
    P: SourceOwnedOperationProviderV1,
{
    if expected.descriptor.time_basis != SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc {
        return Err(SourceOwnedOperationMaterializationError::UnsupportedProviderTimeBasis);
    }
    if provider.descriptor() != expected.descriptor {
        return Err(SourceOwnedOperationMaterializationError::ProviderDescriptorMismatchBefore);
    }

    let operation_carrier = effect.expected().generation().operation_carrier;
    let subject = SourceOwnedOperationMaterializationSubjectV1 {
        admitted_effect: effect.rebound().binding(),
        capability_time_revalidation: effect.receipt(),
        operation_carrier,
        capability_ready_valid_until: effect.valid_until(),
    };

    let receipt = provider
        .materialize_source_owned_operation(&subject)
        .map_err(SourceOwnedOperationMaterializationError::Provider)?;

    if provider.descriptor() != expected.descriptor {
        return Err(SourceOwnedOperationMaterializationError::ProviderDescriptorMismatchAfter);
    }
    if receipt.schema_version != SSF_SCHEMA_V1 {
        return Err(SourceOwnedOperationMaterializationError::UnsupportedReceiptSchema);
    }
    if receipt.provider != expected.descriptor {
        return Err(SourceOwnedOperationMaterializationError::ReceiptProviderMismatch);
    }
    if receipt.subject != subject {
        return Err(SourceOwnedOperationMaterializationError::SubjectMismatch);
    }
    if receipt.operation_carrier != operation_carrier {
        return Err(SourceOwnedOperationMaterializationError::OperationCarrierMismatch);
    }
    if receipt.valid_until > expected.descriptor.valid_until {
        return Err(SourceOwnedOperationMaterializationError::ReceiptOutlivesProvider);
    }
    if receipt.valid_until > effect.valid_until() {
        return Err(SourceOwnedOperationMaterializationError::ReceiptOutlivesCapabilityReadiness);
    }

    let valid_until = derive_material_valid_until::<P::Error>(
        effect.valid_until(),
        expected.descriptor.valid_until,
        receipt.valid_until,
        current_time.valid_until(),
        current_time.latest_possible_unix_ms(),
    )?;

    Ok(SourceOwnedOperationMaterialV1 {
        effect,
        expected,
        receipt,
        current_time,
        valid_until,
        _provider: PhantomData,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn bytes(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    #[test]
    fn handle_and_payload_commitment_are_distinct_domains() {
        assert_ne!(
            SourceOwnedOperationHandleCommitment::from_bytes(bytes(1)).as_bytes(),
            OperationPayloadCommitment::from_bytes(bytes(2)).as_bytes()
        );
    }

    #[test]
    fn natural_expiry_selects_oldest_material_boundary() {
        assert_eq!(
            derive_material_valid_until::<core::convert::Infallible>(90, 120, 110, 130, 80),
            Ok(90)
        );
    }

    #[test]
    fn stale_capability_readiness_fails_closed() {
        assert_eq!(
            derive_material_valid_until::<core::convert::Infallible>(79, 120, 110, 130, 80),
            Err(SourceOwnedOperationMaterializationError::CapabilityReadinessAlreadyExpired)
        );
    }

    #[test]
    fn provider_descriptor_generation_participates_in_identity() {
        let a = SourceOwnedOperationProviderDescriptorV1 {
            stable_identity: SourceOwnedOperationProviderIdentityCommitment::from_bytes(bytes(1)),
            policy: SourceOwnedOperationProviderPolicyCommitment::from_bytes(bytes(2)),
            generation: 3,
            time_basis: SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 100,
        };
        let mut b = a;
        b.generation = 4;
        assert_ne!(a, b);
    }

    #[test]
    fn provider_descriptor_has_explicit_time_basis() {
        let descriptor = SourceOwnedOperationProviderDescriptorV1 {
            stable_identity: SourceOwnedOperationProviderIdentityCommitment::from_bytes(bytes(1)),
            policy: SourceOwnedOperationProviderPolicyCommitment::from_bytes(bytes(2)),
            generation: 3,
            time_basis: SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 100,
        };
        assert_eq!(
            descriptor.time_basis,
            SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc
        );
    }
}
