// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit stable wire tags for leaf replay-evidence semantics.
//!
//! This crate maps semantic enums onto deliberately assigned canonical tags.
//! It never serializes Rust enum discriminants implicitly and does not encode
//! the complete replay-evidence decision subject.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_actuator_effect_protocol::EffectRecoveryPolicyV1;
use mycelix_ssf_canonical_completed_effect_evidence::CanonicalCompletedInvocationRecordV1;
use mycelix_ssf_canonical_completed_outcome_history::CanonicalCompletedOutcomeTerminalV1;
use mycelix_ssf_canonical_replay_evidence_decision_subject::CanonicalReplayEvidenceDecisionDomainV1;
use mycelix_ssf_canonical_replay_evidence_subject::CanonicalReplayEvidenceSubjectTimeBasisV1;
use mycelix_ssf_canonical_wire::{write_fixed_bytes_v1, write_u8_v1, CanonicalWireSinkV1};
use mycelix_ssf_execution_surface_admission::ExecutionGenerationTimeBasisV1;
use mycelix_ssf_no_third_attempt_provenance_gate::InitialReplayEvidenceBasisV1;
use mycelix_ssf_pre_invocation_actuator_qualification::{
    ActuatorRecoveryModeV1, PreInvocationTimeBasisV1,
};
use mycelix_ssf_source_owned_operation_material::SourceOwnedOperationProviderTimeBasisV1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ReplayDecisionDomainWireTagV1 {
    EvidenceFitnessForAtMostOneReplay = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ReplayEvidenceTimeBasisWireTagV1 {
    UnixMillisecondsUtc = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum PreInvocationTimeBasisWireTagV1 {
    UnixMillisecondsUtc = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ExecutionGenerationTimeBasisWireTagV1 {
    UnixMillisecondsUtc = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SourceOwnedProviderTimeBasisWireTagV1 {
    UnixMillisecondsUtc = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum InitialReplayEvidenceBasisWireTagV1 {
    ProvenNotApplied = 1,
    IdempotentOutcomeUnknown = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CompletedInvocationRecordWireTagV1 {
    Initial = 1,
    LegacyReplay = 2,
    CanonicalReplay = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CompletedOutcomeTerminalWireTagV1 {
    Confirmed = 1,
    ProvenNotApplied = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ActuatorRecoveryModeWireTagV1 {
    TransactionalClaimKey = 1,
    IdempotentClaimKey = 2,
    NonIdempotentNoAutomaticRetry = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum EffectRecoveryPolicyWireTagV1 {
    ReconcileOnly = 1,
    ExactSameClaimReplayMayBePermitted = 2,
    NeverAutomaticRetry = 3,
}

pub fn write_replay_decision_domain_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: CanonicalReplayEvidenceDecisionDomainV1,
) -> Result<(), S::Error> {
    let tag = match value {
        CanonicalReplayEvidenceDecisionDomainV1::EvidenceFitnessForAtMostOneReplay => {
            ReplayDecisionDomainWireTagV1::EvidenceFitnessForAtMostOneReplay
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_replay_evidence_time_basis_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: CanonicalReplayEvidenceSubjectTimeBasisV1,
) -> Result<(), S::Error> {
    let tag = match value {
        CanonicalReplayEvidenceSubjectTimeBasisV1::UnixMillisecondsUtc => {
            ReplayEvidenceTimeBasisWireTagV1::UnixMillisecondsUtc
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_pre_invocation_time_basis_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: PreInvocationTimeBasisV1,
) -> Result<(), S::Error> {
    let tag = match value {
        PreInvocationTimeBasisV1::UnixMillisecondsUtc => {
            PreInvocationTimeBasisWireTagV1::UnixMillisecondsUtc
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_execution_generation_time_basis_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ExecutionGenerationTimeBasisV1,
) -> Result<(), S::Error> {
    let tag = match value {
        ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc => {
            ExecutionGenerationTimeBasisWireTagV1::UnixMillisecondsUtc
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_source_owned_provider_time_basis_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: SourceOwnedOperationProviderTimeBasisV1,
) -> Result<(), S::Error> {
    let tag = match value {
        SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc => {
            SourceOwnedProviderTimeBasisWireTagV1::UnixMillisecondsUtc
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_initial_replay_evidence_basis_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: InitialReplayEvidenceBasisV1,
) -> Result<(), S::Error> {
    let tag = match value {
        InitialReplayEvidenceBasisV1::ProvenNotApplied => {
            InitialReplayEvidenceBasisWireTagV1::ProvenNotApplied
        }
        InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown => {
            InitialReplayEvidenceBasisWireTagV1::IdempotentOutcomeUnknown
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_completed_invocation_record_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: CanonicalCompletedInvocationRecordV1,
) -> Result<(), S::Error> {
    match value {
        CanonicalCompletedInvocationRecordV1::Initial(record) => {
            write_u8_v1(sink, CompletedInvocationRecordWireTagV1::Initial as u8)?;
            write_fixed_bytes_v1(sink, record.as_bytes())
        }
        CanonicalCompletedInvocationRecordV1::LegacyReplay(record) => {
            write_u8_v1(sink, CompletedInvocationRecordWireTagV1::LegacyReplay as u8)?;
            write_fixed_bytes_v1(sink, record.as_bytes())
        }
        CanonicalCompletedInvocationRecordV1::CanonicalReplay(record) => {
            write_u8_v1(sink, CompletedInvocationRecordWireTagV1::CanonicalReplay as u8)?;
            write_fixed_bytes_v1(sink, record.as_bytes())
        }
    }
}

pub fn write_completed_outcome_terminal_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: CanonicalCompletedOutcomeTerminalV1,
) -> Result<(), S::Error> {
    let tag = match value {
        CanonicalCompletedOutcomeTerminalV1::Confirmed => {
            CompletedOutcomeTerminalWireTagV1::Confirmed
        }
        CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied => {
            CompletedOutcomeTerminalWireTagV1::ProvenNotApplied
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_actuator_recovery_mode_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: ActuatorRecoveryModeV1,
) -> Result<(), S::Error> {
    let tag = match value {
        ActuatorRecoveryModeV1::TransactionalClaimKey => {
            ActuatorRecoveryModeWireTagV1::TransactionalClaimKey
        }
        ActuatorRecoveryModeV1::IdempotentClaimKey => {
            ActuatorRecoveryModeWireTagV1::IdempotentClaimKey
        }
        ActuatorRecoveryModeV1::NonIdempotentNoAutomaticRetry => {
            ActuatorRecoveryModeWireTagV1::NonIdempotentNoAutomaticRetry
        }
    };
    write_u8_v1(sink, tag as u8)
}

pub fn write_effect_recovery_policy_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: EffectRecoveryPolicyV1,
) -> Result<(), S::Error> {
    let tag = match value {
        EffectRecoveryPolicyV1::ReconcileOnly => EffectRecoveryPolicyWireTagV1::ReconcileOnly,
        EffectRecoveryPolicyV1::ExactSameClaimReplayMayBePermitted => {
            EffectRecoveryPolicyWireTagV1::ExactSameClaimReplayMayBePermitted
        }
        EffectRecoveryPolicyV1::NeverAutomaticRetry => {
            EffectRecoveryPolicyWireTagV1::NeverAutomaticRetry
        }
    };
    write_u8_v1(sink, tag as u8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment;
    use mycelix_ssf_replay_aware_invocation_journal::ReplayInvocationJournalRecordCommitment;
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

    #[test]
    fn replay_basis_tags_are_stable_and_distinct() {
        let mut sink = VecSink::default();
        write_initial_replay_evidence_basis_v1(
            &mut sink,
            InitialReplayEvidenceBasisV1::ProvenNotApplied,
        )
        .unwrap();
        write_initial_replay_evidence_basis_v1(
            &mut sink,
            InitialReplayEvidenceBasisV1::IdempotentOutcomeUnknown,
        )
        .unwrap();
        assert_eq!(sink.0, [1, 2]);
    }

    #[test]
    fn invocation_provenance_is_part_of_the_wire_identity() {
        let legacy = ReplayInvocationJournalRecordCommitment::from_bytes([7; 32]);
        let canonical = ReplayInvocationJournalRecordCommitment::from_bytes([7; 32]);

        let mut legacy_sink = VecSink::default();
        write_completed_invocation_record_v1(
            &mut legacy_sink,
            CanonicalCompletedInvocationRecordV1::LegacyReplay(legacy),
        )
        .unwrap();

        let mut canonical_sink = VecSink::default();
        write_completed_invocation_record_v1(
            &mut canonical_sink,
            CanonicalCompletedInvocationRecordV1::CanonicalReplay(canonical),
        )
        .unwrap();

        assert_ne!(legacy_sink.0, canonical_sink.0);
        assert_eq!(legacy_sink.0.len(), 33);
        assert_eq!(canonical_sink.0.len(), 33);
    }

    #[test]
    fn initial_record_tag_precedes_exact_commitment_bytes() {
        let record = InvocationAttemptJournalRecordCommitment::from_bytes([9; 32]);
        let mut sink = VecSink::default();
        write_completed_invocation_record_v1(
            &mut sink,
            CanonicalCompletedInvocationRecordV1::Initial(record),
        )
        .unwrap();
        assert_eq!(sink.0[0], CompletedInvocationRecordWireTagV1::Initial as u8);
        assert_eq!(&sink.0[1..], &[9; 32]);
    }

    #[test]
    fn all_current_unix_time_basis_domains_have_explicit_tags() {
        let mut sink = VecSink::default();
        write_pre_invocation_time_basis_v1(&mut sink, PreInvocationTimeBasisV1::UnixMillisecondsUtc)
            .unwrap();
        write_execution_generation_time_basis_v1(
            &mut sink,
            ExecutionGenerationTimeBasisV1::UnixMillisecondsUtc,
        )
        .unwrap();
        write_source_owned_provider_time_basis_v1(
            &mut sink,
            SourceOwnedOperationProviderTimeBasisV1::UnixMillisecondsUtc,
        )
        .unwrap();
        assert_eq!(sink.0, [1, 1, 1]);
    }

    #[test]
    fn actuator_modes_and_recovery_policies_are_distinct_semantic_domains() {
        assert_eq!(ActuatorRecoveryModeWireTagV1::TransactionalClaimKey as u8, 1);
        assert_eq!(EffectRecoveryPolicyWireTagV1::ReconcileOnly as u8, 1);
        assert_ne!(
            core::any::type_name::<ActuatorRecoveryModeWireTagV1>(),
            core::any::type_name::<EffectRecoveryPolicyWireTagV1>()
        );
    }
}
