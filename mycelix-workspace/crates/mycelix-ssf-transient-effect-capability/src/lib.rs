// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Transient move-only-by-safe-API SSF effect capability.
//!
//! This is the first SSF typestate object intentionally carrying effect
//! authority. It still performs no actuator call. Cross-restart single use comes
//! from the durable capability record; non-Clone/non-Copy ownership provides the
//! in-process carrier discipline.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_contracts::SSF_SCHEMA_V1;
use mycelix_ssf_current_authority_revalidation::{
    CurrentTimeReceiptCommitment, QualifiedCurrentTimeV1,
};
use mycelix_ssf_durable_effect_capability_issuance::EffectCapabilityRecordCommitment;
use mycelix_ssf_execution_surface_admission::{
    ExecutionSurfaceGenerationV1, OperationCarrierCommitment,
};
use mycelix_ssf_final_effect_capability_revalidation::{
    FinalEffectCapabilityReadyV1, FinalEffectCapabilityRevalidationReceiptV1,
    OperationHandleResolutionCommitment,
};
use mycelix_ssf_source_owned_operation_material::{
    OperationEncodingCommitment, OperationPayloadCommitment,
    SourceOwnedOperationHandleCommitment, SourceOwnedOperationProviderDescriptorV1,
};

/// Copyable audit description of a transient capability.
///
/// This binding is deliberately **not** sufficient to reconstruct a capability.
/// The only public constructor consumes `FinalEffectCapabilityReadyV1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransientEffectCapabilityBindingV1 {
    pub schema_version: u16,
    pub durable_capability_record: EffectCapabilityRecordCommitment,
    pub final_revalidation_receipt: FinalEffectCapabilityRevalidationReceiptV1,
    pub execution_generation: ExecutionSurfaceGenerationV1,
    pub provider: SourceOwnedOperationProviderDescriptorV1,
    pub operation_handle: SourceOwnedOperationHandleCommitment,
    pub handle_resolution: OperationHandleResolutionCommitment,
    pub operation_carrier: OperationCarrierCommitment,
    pub payload_commitment: OperationPayloadCommitment,
    pub encoding_commitment: OperationEncodingCommitment,
    pub payload_length: u64,
    pub mint_time_receipt: CurrentTimeReceiptCommitment,
    pub valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransientEffectCapabilityMintError {
    FinalReadinessAlreadyExpired,
    MintTimeAlreadyExpired,
}

fn derive_transient_capability_valid_until(
    final_ready_valid_until: u64,
    mint_time_valid_until: u64,
    latest_possible_now: u64,
) -> Result<u64, TransientEffectCapabilityMintError> {
    if final_ready_valid_until < latest_possible_now {
        return Err(TransientEffectCapabilityMintError::FinalReadinessAlreadyExpired);
    }
    if mint_time_valid_until < latest_possible_now {
        return Err(TransientEffectCapabilityMintError::MintTimeAlreadyExpired);
    }
    Ok(final_ready_valid_until.min(mint_time_valid_until))
}

/// Owned transient effect capability.
///
/// This type intentionally does not implement `Clone` or `Copy`, has private
/// fields, and has no serialization dependency. Safe Rust callers can only
/// obtain it by consuming an exact final-readiness token.
///
/// A future actuator must accept this object by value and atomically claim
/// `durable_capability_record` before resolving/invoking the provider-owned
/// operation handle.
#[must_use = "effect capability must be explicitly consumed by an authorized actuator or dropped"]
pub struct TransientEffectCapabilityV1<S, M, Q, TQ, MTQ> {
    ready: FinalEffectCapabilityReadyV1<S, M, Q, TQ>,
    mint_time: QualifiedCurrentTimeV1<MTQ>,
    binding: TransientEffectCapabilityBindingV1,
}

impl<S, M, Q, TQ, MTQ> TransientEffectCapabilityV1<S, M, Q, TQ, MTQ> {
    pub const fn binding(&self) -> TransientEffectCapabilityBindingV1 {
        self.binding
    }

    pub const fn final_ready(&self) -> &FinalEffectCapabilityReadyV1<S, M, Q, TQ> {
        &self.ready
    }

    pub const fn mint_time(&self) -> &QualifiedCurrentTimeV1<MTQ> {
        &self.mint_time
    }

    pub const fn durable_capability_record(&self) -> EffectCapabilityRecordCommitment {
        self.binding.durable_capability_record
    }

    pub const fn operation_handle(&self) -> SourceOwnedOperationHandleCommitment {
        self.binding.operation_handle
    }

    pub const fn payload_commitment(&self) -> OperationPayloadCommitment {
        self.binding.payload_commitment
    }

    pub const fn encoding_commitment(&self) -> OperationEncodingCommitment {
        self.binding.encoding_commitment
    }

    pub const fn valid_until(&self) -> u64 {
        self.binding.valid_until
    }

    /// True for the safe public API: this type has no `Clone`/`Copy` impl and no
    /// constructor from its copyable binding.
    pub const fn safe_api_is_move_only(&self) -> bool {
        true
    }

    pub const fn cross_restart_single_use_identity_is_durable_record(&self) -> bool {
        true
    }

    pub const fn requires_atomic_actuator_consumption(&self) -> bool {
        true
    }

    pub const fn contains_effect_authority(&self) -> bool {
        true
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }
}

/// Mint the transient capability from exact final readiness plus fresh trusted
/// time. There is intentionally no constructor from
/// `TransientEffectCapabilityBindingV1`.
pub fn mint_transient_effect_capability<S, M, Q, TQ, MTQ>(
    ready: FinalEffectCapabilityReadyV1<S, M, Q, TQ>,
    mint_time: QualifiedCurrentTimeV1<MTQ>,
) -> Result<TransientEffectCapabilityV1<S, M, Q, TQ, MTQ>, TransientEffectCapabilityMintError> {
    let valid_until = derive_transient_capability_valid_until(
        ready.valid_until(),
        mint_time.valid_until(),
        mint_time.latest_possible_unix_ms(),
    )?;

    let final_receipt = ready.receipt();
    let material_receipt = final_receipt.subject.material_receipt;
    let binding = TransientEffectCapabilityBindingV1 {
        schema_version: SSF_SCHEMA_V1,
        durable_capability_record: ready.lineage().binding().capability_record,
        final_revalidation_receipt: final_receipt,
        execution_generation: final_receipt.execution_generation,
        provider: final_receipt.provider,
        operation_handle: final_receipt.operation_handle,
        handle_resolution: final_receipt.handle_resolution,
        operation_carrier: material_receipt.operation_carrier,
        payload_commitment: material_receipt.payload_commitment,
        encoding_commitment: material_receipt.encoding_commitment,
        payload_length: material_receipt.payload_length,
        mint_time_receipt: mint_time.receipt_commitment(),
        valid_until,
    };

    Ok(TransientEffectCapabilityV1 {
        ready,
        mint_time,
        binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn natural_expiry_uses_older_mint_boundary() {
        assert_eq!(derive_transient_capability_valid_until(120, 90, 80), Ok(90));
    }

    #[test]
    fn stale_final_readiness_fails_closed() {
        assert_eq!(
            derive_transient_capability_valid_until(79, 120, 80),
            Err(TransientEffectCapabilityMintError::FinalReadinessAlreadyExpired)
        );
    }

    #[test]
    fn stale_mint_time_fails_closed() {
        assert_eq!(
            derive_transient_capability_valid_until(120, 79, 80),
            Err(TransientEffectCapabilityMintError::MintTimeAlreadyExpired)
        );
    }
}
