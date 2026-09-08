// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical encoding of `CanonicalCompletedOutcomeHistoryHeadV1`.
//!
//! This crate completely covers the compact durable history-head composite:
//! invocation provenance, exact generation, optional durable head commitment,
//! and optional terminal state. It does not encode the latest observation
//! manifest/evidence subtree.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_canonical_completed_outcome_history::CanonicalCompletedOutcomeHistoryHeadV1;
use mycelix_ssf_canonical_replay_wire_tags::{
    write_completed_invocation_record_v1, write_completed_outcome_terminal_v1,
};
use mycelix_ssf_canonical_wire::{
    write_fixed_bytes_v1, write_option_tag_v1, write_u16_be_v1, write_u64_be_v1,
    CanonicalOptionTagV1, CanonicalWireSinkV1,
};

pub const COMPLETED_OUTCOME_HISTORY_HEAD_ENCODING_VERSION_V1: u16 = 1;

/// Encode every semantic field of one completed-outcome history head.
pub fn write_completed_outcome_history_head_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: CanonicalCompletedOutcomeHistoryHeadV1,
) -> Result<(), S::Error> {
    write_u16_be_v1(sink, COMPLETED_OUTCOME_HISTORY_HEAD_ENCODING_VERSION_V1)?;
    write_completed_invocation_record_v1(sink, value.invocation_record())?;
    write_u64_be_v1(sink, value.generation().get())?;

    match value.head() {
        None => write_option_tag_v1(sink, CanonicalOptionTagV1::None)?,
        Some(record) => {
            write_option_tag_v1(sink, CanonicalOptionTagV1::Some)?;
            write_fixed_bytes_v1(sink, record.as_bytes())?;
        }
    }

    match value.terminal() {
        None => write_option_tag_v1(sink, CanonicalOptionTagV1::None),
        Some(terminal) => {
            write_option_tag_v1(sink, CanonicalOptionTagV1::Some)?;
            write_completed_outcome_terminal_v1(sink, terminal)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_ssf_canonical_completed_effect_evidence::CanonicalCompletedInvocationRecordV1;
    use mycelix_ssf_canonical_completed_outcome_history::{
        CanonicalCompletedOutcomeHistoryGeneration, CanonicalCompletedOutcomeRecordCommitment,
        CanonicalCompletedOutcomeTerminalV1,
    };
    use mycelix_ssf_durable_actuator_invocation_attempt::InvocationAttemptJournalRecordCommitment;
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

    fn invocation() -> CanonicalCompletedInvocationRecordV1 {
        CanonicalCompletedInvocationRecordV1::Initial(
            InvocationAttemptJournalRecordCommitment::from_bytes([7; 32]),
        )
    }

    fn head(
        generation: u64,
        record: Option<[u8; 32]>,
        terminal: Option<CanonicalCompletedOutcomeTerminalV1>,
    ) -> CanonicalCompletedOutcomeHistoryHeadV1 {
        CanonicalCompletedOutcomeHistoryHeadV1::from_trusted_state(
            invocation(),
            CanonicalCompletedOutcomeHistoryGeneration::new(generation),
            record.map(CanonicalCompletedOutcomeRecordCommitment::from_bytes),
            terminal,
        )
        .unwrap()
    }

    fn encode(value: CanonicalCompletedOutcomeHistoryHeadV1) -> Vec<u8> {
        let mut sink = VecSink::default();
        write_completed_outcome_history_head_v1(&mut sink, value).unwrap();
        sink.0
    }

    #[test]
    fn genesis_and_non_genesis_have_distinct_wire_identity() {
        let genesis = head(0, None, None);
        let active = head(1, Some([9; 32]), None);
        assert_ne!(encode(genesis), encode(active));
    }

    #[test]
    fn terminal_state_changes_wire_identity() {
        let confirmed = head(
            1,
            Some([9; 32]),
            Some(CanonicalCompletedOutcomeTerminalV1::Confirmed),
        );
        let not_applied = head(
            1,
            Some([9; 32]),
            Some(CanonicalCompletedOutcomeTerminalV1::ProvenNotApplied),
        );
        assert_ne!(encode(confirmed), encode(not_applied));
    }

    #[test]
    fn generation_changes_wire_identity() {
        let first = head(1, Some([9; 32]), None);
        let second = head(2, Some([9; 32]), None);
        assert_ne!(encode(first), encode(second));
    }

    #[test]
    fn version_is_first() {
        let encoded = encode(head(0, None, None));
        assert_eq!(
            &encoded[..2],
            &COMPLETED_OUTCOME_HISTORY_HEAD_ENCODING_VERSION_V1.to_be_bytes()
        );
    }
}
