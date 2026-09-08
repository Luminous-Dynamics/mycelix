// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical byte-encoding profile for SSF replay-evidence decision subjects.
//!
//! This crate freezes the wire rules that a later exhaustive semantic encoder
//! must follow. It intentionally does **not** claim full subject coverage and
//! performs no hashing, signing, evidence qualification, replay authorization,
//! or effect authorization.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_canonical_replay_evidence_decision_subject::{
    CanonicalReplayEvidenceDecisionDomainV1, CanonicalReplayEvidenceDecisionSubjectV1,
};
use mycelix_ssf_contracts::SSF_SCHEMA_V1;

/// Fixed domain prefix for the canonical replay-evidence decision-subject wire
/// contract. This is written verbatim before all versioned fields.
pub const CANONICAL_REPLAY_EVIDENCE_ENCODING_DOMAIN_V1: &[u8; 31] =
    b"MYCELIX-SSF/REPLAY-EVIDENCE/V1\0";

pub const CANONICAL_REPLAY_EVIDENCE_ENCODING_VERSION_V1: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceIntegerEncodingV1 {
    BigEndian,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceEnumEncodingV1 {
    UnsignedByteTag,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceLengthEncodingV1 {
    UnsignedU32BigEndian,
}

/// Every encoded field uses a deterministic TLV frame:
///
/// `field_id:u16_be || length:u32_be || value_bytes`
///
/// Fields in each structure must appear once, in strictly increasing field-id
/// order. Unknown/duplicate/reordered fields are not canonical v1 encodings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanonicalReplayEvidenceEncodingProfileV1 {
    pub version: u16,
    pub integer_encoding: CanonicalReplayEvidenceIntegerEncodingV1,
    pub enum_encoding: CanonicalReplayEvidenceEnumEncodingV1,
    pub length_encoding: CanonicalReplayEvidenceLengthEncodingV1,
}

impl CanonicalReplayEvidenceEncodingProfileV1 {
    pub const fn v1() -> Self {
        Self {
            version: CANONICAL_REPLAY_EVIDENCE_ENCODING_VERSION_V1,
            integer_encoding: CanonicalReplayEvidenceIntegerEncodingV1::BigEndian,
            enum_encoding: CanonicalReplayEvidenceEnumEncodingV1::UnsignedByteTag,
            length_encoding: CanonicalReplayEvidenceLengthEncodingV1::UnsignedU32BigEndian,
        }
    }
}

/// Canonical top-level field IDs for `CanonicalReplayEvidenceDecisionSubjectV1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u16)]
pub enum CanonicalReplayEvidenceDecisionFieldV1 {
    SchemaVersion = 1,
    DecisionDomain = 2,
    QualificationRequest = 3,
    ValidUntil = 4,
}

impl CanonicalReplayEvidenceDecisionFieldV1 {
    pub const fn id(self) -> u16 {
        self as u16
    }
}

/// Canonical field IDs for `CanonicalReplayEvidenceQualificationRequestSubjectV1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u16)]
pub enum CanonicalReplayEvidenceRequestFieldV1 {
    FrozenEvidenceSubject = 1,
    QualificationTimeReceipt = 2,
    QualificationLatestPossibleUnixMs = 3,
    ValidUntil = 4,
}

impl CanonicalReplayEvidenceRequestFieldV1 {
    pub const fn id(self) -> u16 {
        self as u16
    }
}

/// Canonical field IDs for `CanonicalReplayEvidenceSubjectV1`.
///
/// Deeper nested values referenced by these fields remain explicitly uncovered
/// until later encoder tranches define their own canonical field tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u16)]
pub enum CanonicalReplayEvidenceSubjectFieldV1 {
    SchemaVersion = 1,
    TimeBasis = 2,
    InvocationRecord = 3,
    HistoryHead = 4,
    HistoryHeadReceipt = 5,
    PairedLatestEntry = 6,
    HistoryReadTimeReceipt = 7,
    HistoryReadLatestPossibleUnixMs = 8,
    PriorAttemptId = 9,
    PriorEffectSubject = 10,
    StableEffectIdentity = 11,
    ReplayBasis = 12,
    ValidUntil = 13,
}

impl CanonicalReplayEvidenceSubjectFieldV1 {
    pub const fn id(self) -> u16 {
        self as u16
    }
}

/// High-level coverage groups required before a subject may be presented to a
/// cryptographic adapter as canonically encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceRequiredCoverageV1 {
    DecisionHeader,
    DecisionPurpose,
    QualificationRequest,
    FrozenEvidenceSubject,
    OutcomeHistoryHead,
    PairedLatestOutcomeEntry,
    CompletedEffectEvidence,
    PriorAttempt,
    StableEffectIdentity,
    QualificationTimeBinding,
    ReplayBasis,
    NaturalExpiry,
}

pub const REQUIRED_COVERAGE_V1: [CanonicalReplayEvidenceRequiredCoverageV1; 12] = [
    CanonicalReplayEvidenceRequiredCoverageV1::DecisionHeader,
    CanonicalReplayEvidenceRequiredCoverageV1::DecisionPurpose,
    CanonicalReplayEvidenceRequiredCoverageV1::QualificationRequest,
    CanonicalReplayEvidenceRequiredCoverageV1::FrozenEvidenceSubject,
    CanonicalReplayEvidenceRequiredCoverageV1::OutcomeHistoryHead,
    CanonicalReplayEvidenceRequiredCoverageV1::PairedLatestOutcomeEntry,
    CanonicalReplayEvidenceRequiredCoverageV1::CompletedEffectEvidence,
    CanonicalReplayEvidenceRequiredCoverageV1::PriorAttempt,
    CanonicalReplayEvidenceRequiredCoverageV1::StableEffectIdentity,
    CanonicalReplayEvidenceRequiredCoverageV1::QualificationTimeBinding,
    CanonicalReplayEvidenceRequiredCoverageV1::ReplayBasis,
    CanonicalReplayEvidenceRequiredCoverageV1::NaturalExpiry,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceEncodingCoverageV1 {
    /// Only the framing/profile contract and reviewed field tables are frozen.
    /// Full semantic traversal is intentionally not yet claimed.
    ProfileOnly,
    /// Reserved for the later exhaustive encoder. No constructor in this crate
    /// can produce this state.
    ExhaustiveSemanticTraversal,
}

/// Non-authoritative plan tying the frozen profile to one exact r2 decision
/// subject. It is not encoded bytes and is not a cryptographic commitment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalReplayEvidenceEncodingPlanV1 {
    subject: CanonicalReplayEvidenceDecisionSubjectV1,
    profile: CanonicalReplayEvidenceEncodingProfileV1,
    coverage: CanonicalReplayEvidenceEncodingCoverageV1,
}

impl CanonicalReplayEvidenceEncodingPlanV1 {
    pub const fn subject(&self) -> CanonicalReplayEvidenceDecisionSubjectV1 {
        self.subject
    }

    pub const fn profile(&self) -> CanonicalReplayEvidenceEncodingProfileV1 {
        self.profile
    }

    pub const fn coverage(&self) -> CanonicalReplayEvidenceEncodingCoverageV1 {
        self.coverage
    }

    pub const fn is_exhaustively_encoded(&self) -> bool {
        matches!(
            self.coverage,
            CanonicalReplayEvidenceEncodingCoverageV1::ExhaustiveSemanticTraversal
        )
    }

    pub const fn may_enter_crypto_binding(&self) -> bool {
        self.is_exhaustively_encoded()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanonicalReplayEvidenceEncodingProfileErrorV1 {
    UnsupportedSubjectSchema,
    UnsupportedDecisionDomain,
}

pub fn prepare_canonical_replay_evidence_encoding_profile(
    subject: CanonicalReplayEvidenceDecisionSubjectV1,
) -> Result<CanonicalReplayEvidenceEncodingPlanV1, CanonicalReplayEvidenceEncodingProfileErrorV1>
{
    if subject.schema_version() != SSF_SCHEMA_V1 {
        return Err(CanonicalReplayEvidenceEncodingProfileErrorV1::UnsupportedSubjectSchema);
    }
    if subject.domain()
        != CanonicalReplayEvidenceDecisionDomainV1::EvidenceFitnessForAtMostOneReplay
    {
        return Err(CanonicalReplayEvidenceEncodingProfileErrorV1::UnsupportedDecisionDomain);
    }

    Ok(CanonicalReplayEvidenceEncodingPlanV1 {
        subject,
        profile: CanonicalReplayEvidenceEncodingProfileV1::v1(),
        coverage: CanonicalReplayEvidenceEncodingCoverageV1::ProfileOnly,
    })
}

/// Minimal sink used by future exhaustive encoders and crypto adapters. The
/// profile crate itself writes only profile/framing bytes, never subject bytes.
pub trait CanonicalReplayEvidenceEncodingSinkV1 {
    type Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
}

pub fn write_profile_preamble_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
) -> Result<(), S::Error> {
    sink.write(CANONICAL_REPLAY_EVIDENCE_ENCODING_DOMAIN_V1)?;
    sink.write(&CANONICAL_REPLAY_EVIDENCE_ENCODING_VERSION_V1.to_be_bytes())
}

pub fn write_field_header_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    field_id: u16,
    value_len: u32,
) -> Result<(), S::Error> {
    sink.write(&field_id.to_be_bytes())?;
    sink.write(&value_len.to_be_bytes())
}

pub fn write_u16_be_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    value: u16,
) -> Result<(), S::Error> {
    sink.write(&value.to_be_bytes())
}

pub fn write_u64_be_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    value: u64,
) -> Result<(), S::Error> {
    sink.write(&value.to_be_bytes())
}

pub fn write_enum_tag_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    tag: u8,
) -> Result<(), S::Error> {
    sink.write(&[tag])
}

pub fn write_option_tag_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    present: bool,
) -> Result<(), S::Error> {
    sink.write(&[u8::from(present)])
}

pub fn write_commitment32_v1<S: CanonicalReplayEvidenceEncodingSinkV1>(
    sink: &mut S,
    bytes: &[u8; 32],
) -> Result<(), S::Error> {
    sink.write(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct VecSink(Vec<u8>);

    impl CanonicalReplayEvidenceEncodingSinkV1 for VecSink {
        type Error = core::convert::Infallible;

        fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.extend_from_slice(bytes);
            Ok(())
        }
    }

    #[test]
    fn preamble_is_domain_then_big_endian_version() {
        let mut sink = VecSink::default();
        write_profile_preamble_v1(&mut sink).unwrap();

        let mut expected = CANONICAL_REPLAY_EVIDENCE_ENCODING_DOMAIN_V1.to_vec();
        expected.extend_from_slice(&1_u16.to_be_bytes());
        assert_eq!(sink.0, expected);
    }

    #[test]
    fn field_header_is_u16_id_then_u32_length_big_endian() {
        let mut sink = VecSink::default();
        write_field_header_v1(&mut sink, 0x0102, 0x03040506).unwrap();
        assert_eq!(sink.0, [1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn reviewed_field_tables_are_strictly_ordered() {
        assert!(
            CanonicalReplayEvidenceDecisionFieldV1::SchemaVersion.id()
                < CanonicalReplayEvidenceDecisionFieldV1::DecisionDomain.id()
        );
        assert!(
            CanonicalReplayEvidenceDecisionFieldV1::DecisionDomain.id()
                < CanonicalReplayEvidenceDecisionFieldV1::QualificationRequest.id()
        );
        assert!(
            CanonicalReplayEvidenceRequestFieldV1::FrozenEvidenceSubject.id()
                < CanonicalReplayEvidenceRequestFieldV1::QualificationTimeReceipt.id()
        );
        assert_eq!(CanonicalReplayEvidenceSubjectFieldV1::ValidUntil.id(), 13);
    }

    #[test]
    fn profile_does_not_claim_exhaustive_subject_coverage() {
        assert_ne!(
            CanonicalReplayEvidenceEncodingCoverageV1::ProfileOnly,
            CanonicalReplayEvidenceEncodingCoverageV1::ExhaustiveSemanticTraversal
        );
    }

    #[test]
    fn required_coverage_is_explicit_and_non_empty() {
        assert_eq!(REQUIRED_COVERAGE_V1.len(), 12);
    }
}
