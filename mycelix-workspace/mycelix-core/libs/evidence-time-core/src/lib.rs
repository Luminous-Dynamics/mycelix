// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! EVID-TIME-001A — structural time-evidence semantics.
//!
//! Governing boundary:
//!
//! ```text
//! timestamp present
//! != timestamp authentic
//! != source trusted
//! != synchronized clock
//! != policy-trusted time
//! != freshness
//! != currentness
//! ```
//!
//! This crate intentionally has no clock/network/crypto API. It validates exact
//! identities and interval structure only. Provider verification, trust-policy
//! admission, multi-source composition, and freshness are later theorems.

#![forbid(unsafe_code)]

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const TIME_OBSERVATION_DOMAIN_V1: &str = "mycelix-evidence-time-observation-v1";
pub const TIME_SOURCE_POLICY_DOMAIN_V1: &str = "mycelix-evidence-time-source-policy-v1";
pub const MAX_ID_BYTES: usize = 256;
pub const NANOS_PER_SECOND: u32 = 1_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct UnixTimePointV1 {
    pub seconds: i64,
    pub nanos: u32,
}

impl UnixTimePointV1 {
    pub fn new(seconds: i64, nanos: u32) -> Result<Self, TimeStructuralFailure> {
        if nanos >= NANOS_PER_SECOND {
            return Err(TimeStructuralFailure::InvalidNanoseconds);
        }
        Ok(Self { seconds, nanos })
    }

    pub fn validate(&self) -> Result<(), TimeStructuralFailure> {
        if self.nanos >= NANOS_PER_SECOND {
            return Err(TimeStructuralFailure::InvalidNanoseconds);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TimeIntervalV1 {
    pub earliest: UnixTimePointV1,
    pub latest: UnixTimePointV1,
}

impl TimeIntervalV1 {
    pub fn new(
        earliest: UnixTimePointV1,
        latest: UnixTimePointV1,
    ) -> Result<Self, TimeStructuralFailure> {
        let value = Self { earliest, latest };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), TimeStructuralFailure> {
        self.earliest.validate()?;
        self.latest.validate()?;
        if self.latest < self.earliest {
            return Err(TimeStructuralFailure::ReversedInterval);
        }
        Ok(())
    }

    pub fn width_nanos(&self) -> Result<u128, TimeStructuralFailure> {
        self.validate()?;
        let seconds = i128::from(self.latest.seconds) - i128::from(self.earliest.seconds);
        let nanos = i128::from(self.latest.nanos) - i128::from(self.earliest.nanos);
        let total = seconds
            .checked_mul(i128::from(NANOS_PER_SECOND))
            .and_then(|value| value.checked_add(nanos))
            .ok_or(TimeStructuralFailure::IntervalArithmeticOverflow)?;
        u128::try_from(total).map_err(|_| TimeStructuralFailure::IntervalArithmeticOverflow)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SequenceRequirementV1 {
    Optional,
    Required,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RequestBindingRequirementV1 {
    Optional,
    Required,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawTimeObservationV1 {
    pub source_namespace: String,
    pub source_instance: String,
    pub source_trust_domain: String,
    pub source_profile_id: String,
    pub source_profile_version: String,
    pub time_scale_profile: String,
    pub interval: TimeIntervalV1,
    pub source_sequence: Option<u64>,
    pub transcript_sha256: String,
    pub source_receipt_sha256: String,
    pub request_binding_sha256: Option<String>,
}

impl RawTimeObservationV1 {
    pub fn validate(&self) -> Result<(), TimeStructuralFailure> {
        for value in [
            &self.source_namespace,
            &self.source_instance,
            &self.source_trust_domain,
            &self.source_profile_id,
            &self.source_profile_version,
            &self.time_scale_profile,
        ] {
            if !valid_id(value) {
                return Err(TimeStructuralFailure::InvalidObservationIdentity);
            }
        }
        self.interval.validate()?;
        if !is_canonical_sha256_hex(&self.transcript_sha256)
            || !is_canonical_sha256_hex(&self.source_receipt_sha256)
        {
            return Err(TimeStructuralFailure::InvalidObservationDigest);
        }
        if let Some(binding) = &self.request_binding_sha256 {
            if !is_canonical_sha256_hex(binding) {
                return Err(TimeStructuralFailure::InvalidRequestBinding);
            }
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, TimeStructuralFailure> {
        self.validate()?;
        let mut out = Vec::new();
        append_field(&mut out, TIME_OBSERVATION_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.source_namespace.as_bytes());
        append_field(&mut out, self.source_instance.as_bytes());
        append_field(&mut out, self.source_trust_domain.as_bytes());
        append_field(&mut out, self.source_profile_id.as_bytes());
        append_field(&mut out, self.source_profile_version.as_bytes());
        append_field(&mut out, self.time_scale_profile.as_bytes());
        append_time_point(&mut out, &self.interval.earliest);
        append_time_point(&mut out, &self.interval.latest);
        match self.source_sequence {
            Some(sequence) => {
                append_field(&mut out, b"source-sequence-present-v1");
                append_field(&mut out, &sequence.to_be_bytes());
            }
            None => append_field(&mut out, b"source-sequence-absent-v1"),
        }
        append_field(&mut out, self.transcript_sha256.as_bytes());
        append_field(&mut out, self.source_receipt_sha256.as_bytes());
        match &self.request_binding_sha256 {
            Some(binding) => {
                append_field(&mut out, b"request-binding-present-v1");
                append_field(&mut out, binding.as_bytes());
            }
            None => append_field(&mut out, b"request-binding-absent-v1"),
        }
        Ok(sha256_hex(&out))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct TimeSourceAdmissionPolicyV1 {
    source_namespace: String,
    source_instance: String,
    source_trust_domain: String,
    source_profile_id: String,
    source_profile_version: String,
    time_scale_profile: String,
    verifier_profile_id: String,
    verifier_profile_version: String,
    max_interval_width_nanos: u64,
    sequence_requirement: SequenceRequirementV1,
    request_binding_requirement: RequestBindingRequirementV1,
}

impl TimeSourceAdmissionPolicyV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        source_namespace: impl Into<String>,
        source_instance: impl Into<String>,
        source_trust_domain: impl Into<String>,
        source_profile_id: impl Into<String>,
        source_profile_version: impl Into<String>,
        time_scale_profile: impl Into<String>,
        verifier_profile_id: impl Into<String>,
        verifier_profile_version: impl Into<String>,
        max_interval_width_nanos: u64,
        sequence_requirement: SequenceRequirementV1,
        request_binding_requirement: RequestBindingRequirementV1,
    ) -> Result<Self, TimeStructuralFailure> {
        let value = Self {
            source_namespace: source_namespace.into(),
            source_instance: source_instance.into(),
            source_trust_domain: source_trust_domain.into(),
            source_profile_id: source_profile_id.into(),
            source_profile_version: source_profile_version.into(),
            time_scale_profile: time_scale_profile.into(),
            verifier_profile_id: verifier_profile_id.into(),
            verifier_profile_version: verifier_profile_version.into(),
            max_interval_width_nanos,
            sequence_requirement,
            request_binding_requirement,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), TimeStructuralFailure> {
        for value in [
            &self.source_namespace,
            &self.source_instance,
            &self.source_trust_domain,
            &self.source_profile_id,
            &self.source_profile_version,
            &self.time_scale_profile,
            &self.verifier_profile_id,
            &self.verifier_profile_version,
        ] {
            if !valid_id(value) {
                return Err(TimeStructuralFailure::InvalidPolicyIdentity);
            }
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, TimeStructuralFailure> {
        self.validate()?;
        let mut out = Vec::new();
        append_field(&mut out, TIME_SOURCE_POLICY_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.source_namespace.as_bytes());
        append_field(&mut out, self.source_instance.as_bytes());
        append_field(&mut out, self.source_trust_domain.as_bytes());
        append_field(&mut out, self.source_profile_id.as_bytes());
        append_field(&mut out, self.source_profile_version.as_bytes());
        append_field(&mut out, self.time_scale_profile.as_bytes());
        append_field(&mut out, self.verifier_profile_id.as_bytes());
        append_field(&mut out, self.verifier_profile_version.as_bytes());
        append_field(&mut out, &self.max_interval_width_nanos.to_be_bytes());
        append_field(
            &mut out,
            match self.sequence_requirement {
                SequenceRequirementV1::Optional => b"sequence-optional-v1",
                SequenceRequirementV1::Required => b"sequence-required-v1",
            },
        );
        append_field(
            &mut out,
            match self.request_binding_requirement {
                RequestBindingRequirementV1::Optional => b"request-binding-optional-v1",
                RequestBindingRequirementV1::Required => b"request-binding-required-v1",
            },
        );
        Ok(sha256_hex(&out))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimeStructuralFailure {
    InvalidNanoseconds,
    ReversedInterval,
    IntervalArithmeticOverflow,
    InvalidObservationIdentity,
    InvalidObservationDigest,
    InvalidRequestBinding,
    InvalidPolicyIdentity,
    SourceNamespaceMismatch,
    SourceInstanceMismatch,
    SourceTrustDomainMismatch,
    SourceProfileMismatch,
    TimeScaleProfileMismatch,
    IntervalTooWide,
    RequiredSequenceMissing,
    RequiredRequestBindingMissing,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyCompatibleTimeObservationV1 {
    policy_sha256: String,
    observation_sha256: String,
    interval: TimeIntervalV1,
    source_namespace: String,
    source_instance: String,
    source_trust_domain: String,
    source_profile_id: String,
    source_profile_version: String,
    time_scale_profile: String,
    verifier_profile_id: String,
    verifier_profile_version: String,
    source_sequence: Option<u64>,
    transcript_sha256: String,
    source_receipt_sha256: String,
    request_binding_sha256: Option<String>,
}

impl StructurallyCompatibleTimeObservationV1 {
    pub fn policy_sha256(&self) -> &str { &self.policy_sha256 }
    pub fn observation_sha256(&self) -> &str { &self.observation_sha256 }
    pub const fn interval(&self) -> TimeIntervalV1 { self.interval }
    pub fn source_trust_domain(&self) -> &str { &self.source_trust_domain }
    pub fn source_sequence(&self) -> Option<u64> { self.source_sequence }
    pub const fn structural_compatibility_established(&self) -> bool { true }
    pub const fn source_cryptographically_verified(&self) -> bool { false }
    pub const fn verifier_execution_established(&self) -> bool { false }
    pub const fn source_trusted_under_policy(&self) -> bool { false }
    pub const fn replay_resistance_established(&self) -> bool { false }
    pub const fn synchronized_clock_established(&self) -> bool { false }
    pub const fn policy_trusted_time_established(&self) -> bool { false }
    pub const fn freshness_established(&self) -> bool { false }
    pub const fn currentness_established(&self) -> bool { false }
    pub const fn legal_timestamp_established(&self) -> bool { false }
    pub const fn application_authority_granted(&self) -> bool { false }
}

pub fn evaluate_time_observation_structure_v1(
    policy: &TimeSourceAdmissionPolicyV1,
    observation: &RawTimeObservationV1,
) -> Result<StructurallyCompatibleTimeObservationV1, TimeStructuralFailure> {
    policy.validate()?;
    observation.validate()?;

    if observation.source_namespace != policy.source_namespace {
        return Err(TimeStructuralFailure::SourceNamespaceMismatch);
    }
    if observation.source_instance != policy.source_instance {
        return Err(TimeStructuralFailure::SourceInstanceMismatch);
    }
    if observation.source_trust_domain != policy.source_trust_domain {
        return Err(TimeStructuralFailure::SourceTrustDomainMismatch);
    }
    if observation.source_profile_id != policy.source_profile_id
        || observation.source_profile_version != policy.source_profile_version
    {
        return Err(TimeStructuralFailure::SourceProfileMismatch);
    }
    if observation.time_scale_profile != policy.time_scale_profile {
        return Err(TimeStructuralFailure::TimeScaleProfileMismatch);
    }
    if observation.interval.width_nanos()? > u128::from(policy.max_interval_width_nanos) {
        return Err(TimeStructuralFailure::IntervalTooWide);
    }
    if policy.sequence_requirement == SequenceRequirementV1::Required
        && observation.source_sequence.is_none()
    {
        return Err(TimeStructuralFailure::RequiredSequenceMissing);
    }
    if policy.request_binding_requirement == RequestBindingRequirementV1::Required
        && observation.request_binding_sha256.is_none()
    {
        return Err(TimeStructuralFailure::RequiredRequestBindingMissing);
    }

    Ok(StructurallyCompatibleTimeObservationV1 {
        policy_sha256: policy.commitment_sha256()?,
        observation_sha256: observation.commitment_sha256()?,
        interval: observation.interval,
        source_namespace: observation.source_namespace.clone(),
        source_instance: observation.source_instance.clone(),
        source_trust_domain: observation.source_trust_domain.clone(),
        source_profile_id: observation.source_profile_id.clone(),
        source_profile_version: observation.source_profile_version.clone(),
        time_scale_profile: observation.time_scale_profile.clone(),
        verifier_profile_id: policy.verifier_profile_id.clone(),
        verifier_profile_version: policy.verifier_profile_version.clone(),
        source_sequence: observation.source_sequence,
        transcript_sha256: observation.transcript_sha256.clone(),
        source_receipt_sha256: observation.source_receipt_sha256.clone(),
        request_binding_sha256: observation.request_binding_sha256.clone(),
    })
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_ID_BYTES
        && value.is_ascii()
        && !value.bytes().any(|byte| byte.is_ascii_control() || byte.is_ascii_whitespace())
}

fn is_canonical_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_digit() || matches!(byte, b'a'..=b'f'))
}

fn append_time_point(out: &mut Vec<u8>, point: &UnixTimePointV1) {
    append_field(out, &point.seconds.to_be_bytes());
    append_field(out, &point.nanos.to_be_bytes());
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("time-evidence semantic fields fit u32");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(field);
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in digest {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(seconds: i64, nanos: u32) -> UnixTimePointV1 {
        UnixTimePointV1::new(seconds, nanos).unwrap()
    }

    fn interval(start: i64, end: i64) -> TimeIntervalV1 {
        TimeIntervalV1::new(point(start, 0), point(end, 0)).unwrap()
    }

    fn policy() -> TimeSourceAdmissionPolicyV1 {
        TimeSourceAdmissionPolicyV1::new(
            "example-time-provider",
            "server-a",
            "operator-a",
            "synthetic-authenticated-time",
            "1",
            "unix-utc-v1",
            "synthetic-time-verifier",
            "1",
            5_000_000_000,
            SequenceRequirementV1::Required,
            RequestBindingRequirementV1::Required,
        )
        .unwrap()
    }

    fn observation() -> RawTimeObservationV1 {
        RawTimeObservationV1 {
            source_namespace: "example-time-provider".into(),
            source_instance: "server-a".into(),
            source_trust_domain: "operator-a".into(),
            source_profile_id: "synthetic-authenticated-time".into(),
            source_profile_version: "1".into(),
            time_scale_profile: "unix-utc-v1".into(),
            interval: interval(1_800_000_000, 1_800_000_002),
            source_sequence: Some(42),
            transcript_sha256: "11".repeat(32),
            source_receipt_sha256: "22".repeat(32),
            request_binding_sha256: Some("33".repeat(32)),
        }
    }

    #[test]
    fn nanoseconds_must_be_canonical() {
        assert_eq!(
            UnixTimePointV1::new(0, NANOS_PER_SECOND),
            Err(TimeStructuralFailure::InvalidNanoseconds)
        );
    }

    #[test]
    fn reversed_interval_is_rejected() {
        assert_eq!(
            TimeIntervalV1::new(point(10, 0), point(9, 999_999_999)),
            Err(TimeStructuralFailure::ReversedInterval)
        );
    }

    #[test]
    fn zero_width_interval_is_structurally_allowed_but_not_trusted_time() {
        let instant = point(10, 123);
        let interval = TimeIntervalV1::new(instant, instant).unwrap();
        assert_eq!(interval.width_nanos().unwrap(), 0);
    }

    #[test]
    fn interval_width_handles_cross_second_nanoseconds_exactly() {
        let value = TimeIntervalV1::new(point(10, 900_000_000), point(11, 100_000_000)).unwrap();
        assert_eq!(value.width_nanos().unwrap(), 200_000_000);
    }

    #[test]
    fn exact_structural_match_produces_non_authoritative_positive() {
        let positive = evaluate_time_observation_structure_v1(&policy(), &observation()).unwrap();
        assert!(positive.structural_compatibility_established());
        assert_eq!(positive.source_trust_domain(), "operator-a");
        assert_eq!(positive.source_sequence(), Some(42));
        assert!(!positive.source_cryptographically_verified());
        assert!(!positive.verifier_execution_established());
        assert!(!positive.source_trusted_under_policy());
        assert!(!positive.replay_resistance_established());
        assert!(!positive.synchronized_clock_established());
        assert!(!positive.policy_trusted_time_established());
        assert!(!positive.freshness_established());
        assert!(!positive.currentness_established());
        assert!(!positive.legal_timestamp_established());
        assert!(!positive.application_authority_granted());
    }

    #[test]
    fn interval_too_wide_for_policy_fails_closed() {
        let mut observed = observation();
        observed.interval = interval(1_800_000_000, 1_800_000_006);
        assert_eq!(
            evaluate_time_observation_structure_v1(&policy(), &observed),
            Err(TimeStructuralFailure::IntervalTooWide)
        );
    }

    #[test]
    fn source_trust_domain_substitution_is_rejected() {
        let mut observed = observation();
        observed.source_trust_domain = "same-operator-second-key".into();
        assert_eq!(
            evaluate_time_observation_structure_v1(&policy(), &observed),
            Err(TimeStructuralFailure::SourceTrustDomainMismatch)
        );
    }

    #[test]
    fn source_profile_substitution_is_rejected() {
        let mut observed = observation();
        observed.source_profile_version = "2".into();
        assert_eq!(
            evaluate_time_observation_structure_v1(&policy(), &observed),
            Err(TimeStructuralFailure::SourceProfileMismatch)
        );
    }

    #[test]
    fn required_sequence_cannot_be_invented_by_policy() {
        let mut observed = observation();
        observed.source_sequence = None;
        assert_eq!(
            evaluate_time_observation_structure_v1(&policy(), &observed),
            Err(TimeStructuralFailure::RequiredSequenceMissing)
        );
    }

    #[test]
    fn required_request_binding_cannot_be_invented_by_policy() {
        let mut observed = observation();
        observed.request_binding_sha256 = None;
        assert_eq!(
            evaluate_time_observation_structure_v1(&policy(), &observed),
            Err(TimeStructuralFailure::RequiredRequestBindingMissing)
        );
    }

    #[test]
    fn changing_uncertainty_interval_changes_observation_identity() {
        let a = observation();
        let mut b = a.clone();
        b.interval.latest = point(1_800_000_003, 0);
        assert_ne!(a.commitment_sha256().unwrap(), b.commitment_sha256().unwrap());
    }

    #[test]
    fn raw_observation_serde_round_trip_remains_non_authoritative() {
        let raw = observation();
        let bytes = serde_json::to_vec(&raw).unwrap();
        let decoded: RawTimeObservationV1 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, raw);
    }
}
