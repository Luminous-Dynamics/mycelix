// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical wire encoding for the replay-stable actuator effect identity.
//!
//! This is the first complete composite encoder in the replay-evidence wire
//! series. It covers every semantic field of `ActuatorStableEffectIdentityV1`
//! and its nested execution/provider descriptors while deliberately excluding
//! fresh attempt-specific qualification evidence.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::ActuatorStableEffectIdentityV1;
use mycelix_ssf_canonical_replay_wire_tags::{
    write_actuator_recovery_mode_v1, write_execution_generation_time_basis_v1,
    write_pre_invocation_time_basis_v1, write_source_owned_provider_time_basis_v1,
};
use mycelix_ssf_canonical_wire::{
    write_fixed_bytes_v1, write_u16_be_v1, write_u64_be_v1, CanonicalWireSinkV1,
};
use mycelix_ssf_execution_surface_admission::{
    ActuatorGenerationV1, AuthenticatedCarrierGenerationV1, AuthenticatedSessionGenerationV1,
    ExecutionSurfaceGenerationV1,
};
use mycelix_ssf_pre_invocation_actuator_qualification::ActuatorExecutionDescriptorV1;
use mycelix_ssf_source_owned_operation_material::SourceOwnedOperationProviderDescriptorV1;

/// Nested composite encoding version for `ActuatorStableEffectIdentityV1`.
pub const STABLE_EFFECT_IDENTITY_ENCODING_VERSION_V1: u16 = 1;

pub fn write_authenticated_session_generation_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: AuthenticatedSessionGenerationV1,
) -> Result<(), S::Error> {
    write_fixed_bytes_v1(sink, value.logical_session.as_bytes())?;
    write_fixed_bytes_v1(sink, value.transcript.as_bytes())
}

pub fn write_authenticated_carrier_generation_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: AuthenticatedCarrierGenerationV1,
) -> Result<(), S::Error> {
    write_authenticated_session_generation_v1(sink, value.session)?;
    write_fixed_bytes_v1(sink, value.transport_authority.as_bytes())
}

pub fn write_actuator_generation_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ActuatorGenerationV1,
) -> Result<(), S::Error> {
    write_fixed_bytes_v1(sink, value.policy.as_bytes())?;
    write_fixed_bytes_v1(sink, value.health.as_bytes())
}

pub fn write_execution_surface_generation_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ExecutionSurfaceGenerationV1,
) -> Result<(), S::Error> {
    write_fixed_bytes_v1(sink, value.surface_identity.as_bytes())?;
    write_authenticated_carrier_generation_v1(sink, value.carrier)?;
    write_actuator_generation_v1(sink, value.actuator)?;
    write_fixed_bytes_v1(sink, value.operation_carrier.as_bytes())?;
    write_execution_generation_time_basis_v1(sink, value.time_basis)?;
    write_u64_be_v1(sink, value.valid_until)
}

pub fn write_source_owned_provider_descriptor_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: SourceOwnedOperationProviderDescriptorV1,
) -> Result<(), S::Error> {
    write_fixed_bytes_v1(sink, value.stable_identity.as_bytes())?;
    write_fixed_bytes_v1(sink, value.policy.as_bytes())?;
    write_u64_be_v1(sink, value.generation)?;
    write_source_owned_provider_time_basis_v1(sink, value.time_basis)?;
    write_u64_be_v1(sink, value.valid_until)
}

pub fn write_actuator_execution_descriptor_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ActuatorExecutionDescriptorV1,
) -> Result<(), S::Error> {
    write_fixed_bytes_v1(sink, value.stable_identity.as_bytes())?;
    write_fixed_bytes_v1(sink, value.policy.as_bytes())?;
    write_u64_be_v1(sink, value.generation)?;
    write_execution_surface_generation_v1(sink, value.execution_generation)?;
    write_source_owned_provider_descriptor_v1(sink, value.provider)?;
    write_actuator_recovery_mode_v1(sink, value.recovery_mode)?;
    write_pre_invocation_time_basis_v1(sink, value.time_basis)?;
    write_u64_be_v1(sink, value.valid_until)
}

/// Encode every field of the replay-stable effect identity in fixed v0.1 order.
///
/// The recovery mode appears both inside the actuator descriptor and again as
/// an explicit field of `ActuatorStableEffectIdentityV1`; both occurrences are
/// written because both are part of semantic equality of the source type.
pub fn write_actuator_stable_effect_identity_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ActuatorStableEffectIdentityV1,
) -> Result<(), S::Error> {
    write_u16_be_v1(sink, STABLE_EFFECT_IDENTITY_ENCODING_VERSION_V1)?;
    write_fixed_bytes_v1(sink, value.claim_record.as_bytes())?;
    write_actuator_execution_descriptor_v1(sink, value.actuator)?;
    write_fixed_bytes_v1(sink, value.operation_handle.as_bytes())?;
    write_fixed_bytes_v1(sink, value.payload_commitment.as_bytes())?;
    write_fixed_bytes_v1(sink, value.encoding_commitment.as_bytes())?;
    write_u64_be_v1(sink, value.payload_length)?;
    write_actuator_recovery_mode_v1(sink, value.recovery_mode)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_effect_capability_claim::EffectCapabilityClaimRecordCommitment;
    use mycelix_ssf_execution_surface_admission::{
        ActuatorHealthGenerationCommitment, ActuatorPolicyGenerationCommitment,
        AuthenticatedSessionTranscriptCommitment, ExecutionGenerationTimeBasisV1,
        ExecutionSurfaceIdentityCommitment, LogicalSessionIdCommitment, OperationCarrierCommitment,
        TransportAuthorityGenerationCommitment,
    };
    use mycelix_ssf_pre_invocation_actuator_qualification::{
        ActuatorExecutionPolicyCommitment, ActuatorRecoveryModeV1,
        ActuatorStableIdentityCommitment, PreInvocationTimeBasisV1,
    };
    use mycelix_ssf_source_owned_operation_material::{
        OperationEncodingCommitment, OperationPayloadCommitment,
        SourceOwnedOperationHandleCommitment, SourceOwnedOperationProviderIdentityCommitment,
        SourceOwnedOperationProviderPolicyCommitment, SourceOwnedOperationProviderTimeBasisV1,
    };
    use std::vec::Vec;

    #[derive(Default)]
    struct VecSink(Vec<u8>);

    impl CanonicalWireSinkV1 for VecSink {
        type Error = ();

        fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.extend_from_slice(bytes);
            Ok(())
        }
    }

    fn identity(
        transcript_byte: u8,
        provider_generation: u64,
        recovery_mode: ActuatorRecoveryModeV1,
    ) -> ActuatorStableEffectIdentityV1 {
        let provider = SourceOwnedOperationProviderDescriptorV1 {
            stable_identity: SourceOwnedOperationProviderIdentityCommitment::from_bytes([11; 32]),
            policy: SourceOwnedOperationProviderPolicyCommitment::from_bytes([12; 32]),
            generation: provider_generation,
            time_basis: SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 9_000,
        };
        let execution_generation = ExecutionSurfaceGenerationV1 {
            surface_identity: ExecutionSurfaceIdentityCommitment::from_bytes([21; 32]),
            carrier: AuthenticatedCarrierGenerationV1 {
                session: AuthenticatedSessionGenerationV1 {
                    logical_session: LogicalSessionIdCommitment::from_bytes([22; 32]),
                    transcript: AuthenticatedSessionTranscriptCommitment::from_bytes([
                        transcript_byte;
                        32
                    ]),
                },
                transport_authority: TransportAuthorityGenerationCommitment::from_bytes([23; 32]),
            },
            actuator: ActuatorGenerationV1 {
                policy: ActuatorPolicyGenerationCommitment::from_bytes([24; 32]),
                health: ActuatorHealthGenerationCommitment::from_bytes([25; 32]),
            },
            operation_carrier: OperationCarrierCommitment::from_bytes([26; 32]),
            time_basis: ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 8_000,
        };
        let actuator = ActuatorExecutionDescriptorV1 {
            stable_identity: ActuatorStableIdentityCommitment::from_bytes([31; 32]),
            policy: ActuatorExecutionPolicyCommitment::from_bytes([32; 32]),
            generation: 4,
            execution_generation,
            provider,
            recovery_mode,
            time_basis: PreInvocationTimeBasisV1::UnixMillisecondsUtc,
            valid_until: 7_000,
        };

        ActuatorStableEffectIdentityV1 {
            claim_record: EffectCapabilityClaimRecordCommitment::from_bytes([41; 32]),
            actuator,
            operation_handle: SourceOwnedOperationHandleCommitment::from_bytes([42; 32]),
            payload_commitment: OperationPayloadCommitment::from_bytes([43; 32]),
            encoding_commitment: OperationEncodingCommitment::from_bytes([44; 32]),
            payload_length: 4_096,
            recovery_mode,
        }
    }

    fn encode(value: ActuatorStableEffectIdentityV1) -> Vec<u8> {
        let mut sink = VecSink::default();
        write_actuator_stable_effect_identity_v1(&mut sink, value).unwrap();
        sink.0
    }

    #[test]
    fn encoding_is_deterministic() {
        let value = identity(51, 6, ActuatorRecoveryModeV1::IdempotentClaimKey);
        let first = encode(value);
        let second = encode(value);
        assert_eq!(first, second);
    }

    #[test]
    fn authenticated_transcript_changes_wire_identity() {
        let a = identity(51, 6, ActuatorRecoveryModeV1::IdempotentClaimKey);
        let b = identity(52, 6, ActuatorRecoveryModeV1::IdempotentClaimKey);
        assert_ne!(encode(a), encode(b));
    }

    #[test]
    fn provider_generation_changes_wire_identity() {
        let a = identity(51, 6, ActuatorRecoveryModeV1::IdempotentClaimKey);
        let b = identity(51, 7, ActuatorRecoveryModeV1::IdempotentClaimKey);
        assert_ne!(encode(a), encode(b));
    }

    #[test]
    fn recovery_mode_changes_wire_identity() {
        let a = identity(51, 6, ActuatorRecoveryModeV1::IdempotentClaimKey);
        let b = identity(51, 6, ActuatorRecoveryModeV1::TransactionalClaimKey);
        assert_ne!(encode(a), encode(b));
    }

    #[test]
    fn composite_version_is_first() {
        let encoded = encode(identity(
            51,
            6,
            ActuatorRecoveryModeV1::IdempotentClaimKey,
        ));
        assert_eq!(
            &encoded[..2],
            &STABLE_EFFECT_IDENTITY_ENCODING_VERSION_V1.to_be_bytes()
        );
    }
}
