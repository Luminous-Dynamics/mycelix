// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Prepared signed time-receipt body for Identity V2 historical activation.
//!
//! This pure theorem freezes the exact bytes a future independent time authority must
//! authenticate for one #393 activation subject. It validates shape, arithmetic, and the
//! conservative uncertainty rule only. It does not verify a signature, accept/current a
//! time-authority policy, establish clock trust, or make a key historically eligible.
//!
//! The conservative activation bound is the latest plausible observation time:
//!
//! `observed_at_micros + uncertainty_after_micros`.
//!
//! Using the nominal timestamp, or the earliest plausible time, would allow uncertainty
//! to create unjustified backdated authority.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const TIME_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const TIME_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const HISTORICAL_ACTIVATION_TIME_RECEIPT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-receipt:v2\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimeBasisV2 {
    UnixMicrosecondsUtc = 1,
}

impl TimeBasisV2 {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Exact wire/body fields that a later time-authority signature must authenticate.
///
/// Raw fields are public because this is a wire statement, not a success capability.
/// Callers cannot promote it to authenticated/trusted evidence without later layers.
#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimeReceiptBodyV2<'a> {
    pub activation_subject_sha256: &'a [u8],
    pub time_authority_policy_sha256: &'a [u8],
    pub time_authority_id: &'a str,
    pub time_authority_key_id: &'a str,
    pub time_authority_key_generation_sha256: &'a [u8],
    pub algorithm: AlgorithmId,
    pub time_basis: TimeBasisV2,
    pub observed_at_micros: i64,
    pub uncertainty_before_micros: u64,
    pub uncertainty_after_micros: u64,
    /// Latest time at which this receipt may be reused as time evidence. This is a
    /// signed ceiling only; enforcing current reuse eligibility requires later trusted
    /// time/currentness evidence.
    pub receipt_valid_until_micros: i64,
}

/// Opaque structurally prepared receipt. This means only that the body is canonical,
/// arithmetic is safe, and the exact signing digest/conservative interval are frozen.
#[derive(Debug)]
pub struct PreparedHistoricalActivationTimeReceiptV2 {
    activation_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_policy_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    time_authority_id: String,
    time_authority_key_id: String,
    algorithm: AlgorithmId,
    time_basis: TimeBasisV2,
    observed_at_micros: i64,
    uncertainty_before_micros: u64,
    uncertainty_after_micros: u64,
    earliest_plausible_observation_micros: i64,
    latest_plausible_observation_micros: i64,
    receipt_valid_until_micros: i64,
}

impl PreparedHistoricalActivationTimeReceiptV2 {
    pub fn activation_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.activation_subject_sha256
    }

    pub fn time_authority_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.time_authority_policy_sha256
    }

    pub fn time_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.time_authority_key_generation_sha256
    }

    pub fn signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.signing_digest_sha256
    }

    pub fn time_authority_id(&self) -> &str {
        &self.time_authority_id
    }

    pub fn time_authority_key_id(&self) -> &str {
        &self.time_authority_key_id
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn time_basis(&self) -> TimeBasisV2 {
        self.time_basis
    }

    pub fn observed_at_micros(&self) -> i64 {
        self.observed_at_micros
    }

    pub fn uncertainty_before_micros(&self) -> u64 {
        self.uncertainty_before_micros
    }

    pub fn uncertainty_after_micros(&self) -> u64 {
        self.uncertainty_after_micros
    }

    pub fn earliest_plausible_observation_micros(&self) -> i64 {
        self.earliest_plausible_observation_micros
    }

    pub fn latest_plausible_observation_micros(&self) -> i64 {
        self.latest_plausible_observation_micros
    }

    /// The only activation-time bound a historical eligibility theorem may use.
    pub fn conservative_activation_bound_micros(&self) -> i64 {
        self.latest_plausible_observation_micros
    }

    pub fn receipt_valid_until_micros(&self) -> i64 {
        self.receipt_valid_until_micros
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimeReceiptErrorV2 {
    ActivationSubjectDigestLengthInvalid,
    ActivationSubjectDigestAllZero,
    TimeAuthorityPolicyDigestLengthInvalid,
    TimeAuthorityPolicyDigestAllZero,
    TimeAuthorityIdInvalid,
    TimeAuthorityKeyIdInvalid,
    TimeAuthorityKeyGenerationDigestLengthInvalid,
    TimeAuthorityKeyGenerationDigestAllZero,
    NonSignatureAlgorithm,
    ObservedTimeInvalid,
    UncertaintyTooLarge,
    ObservationIntervalUnderflow,
    ObservationIntervalOverflow,
    ReceiptValidityInvalid,
}

fn valid_visible_ascii_identifier(value: &str, max_len: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_len
        && value.is_ascii()
        && value.as_bytes().iter().all(|byte| (0x21..=0x7e).contains(byte))
}

fn require_digest_v2(
    value: &[u8],
    length_error: HistoricalActivationTimeReceiptErrorV2,
    zero_error: HistoricalActivationTimeReceiptErrorV2,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeReceiptErrorV2> {
    let digest: [u8; SHA256_DIGEST_LEN_V2] = value.try_into().map_err(|_| length_error)?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(zero_error);
    }
    Ok(digest)
}

fn observation_interval_v2(
    observed_at_micros: i64,
    uncertainty_before_micros: u64,
    uncertainty_after_micros: u64,
) -> Result<(i64, i64), HistoricalActivationTimeReceiptErrorV2> {
    if observed_at_micros <= 0 {
        return Err(HistoricalActivationTimeReceiptErrorV2::ObservedTimeInvalid);
    }
    let before = i64::try_from(uncertainty_before_micros)
        .map_err(|_| HistoricalActivationTimeReceiptErrorV2::UncertaintyTooLarge)?;
    let after = i64::try_from(uncertainty_after_micros)
        .map_err(|_| HistoricalActivationTimeReceiptErrorV2::UncertaintyTooLarge)?;
    let earliest = observed_at_micros
        .checked_sub(before)
        .ok_or(HistoricalActivationTimeReceiptErrorV2::ObservationIntervalUnderflow)?;
    let latest = observed_at_micros
        .checked_add(after)
        .ok_or(HistoricalActivationTimeReceiptErrorV2::ObservationIntervalOverflow)?;
    if earliest <= 0 {
        return Err(HistoricalActivationTimeReceiptErrorV2::ObservationIntervalUnderflow);
    }
    Ok((earliest, latest))
}

fn update_len_prefixed_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

/// Deterministic digest that a future time-authority signature must authenticate.
pub fn derive_historical_activation_time_receipt_signing_digest_v2(
    body: HistoricalActivationTimeReceiptBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeReceiptErrorV2> {
    let activation_subject_sha256 = require_digest_v2(
        body.activation_subject_sha256,
        HistoricalActivationTimeReceiptErrorV2::ActivationSubjectDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::ActivationSubjectDigestAllZero,
    )?;
    let time_authority_policy_sha256 = require_digest_v2(
        body.time_authority_policy_sha256,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityPolicyDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityPolicyDigestAllZero,
    )?;
    let time_authority_key_generation_sha256 = require_digest_v2(
        body.time_authority_key_generation_sha256,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityKeyGenerationDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityKeyGenerationDigestAllZero,
    )?;
    if !valid_visible_ascii_identifier(body.time_authority_id, TIME_AUTHORITY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimeReceiptErrorV2::TimeAuthorityIdInvalid);
    }
    if !valid_visible_ascii_identifier(
        body.time_authority_key_id,
        TIME_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(HistoricalActivationTimeReceiptErrorV2::TimeAuthorityKeyIdInvalid);
    }
    if !body.algorithm.is_signature_algorithm() {
        return Err(HistoricalActivationTimeReceiptErrorV2::NonSignatureAlgorithm);
    }
    let (_, latest) = observation_interval_v2(
        body.observed_at_micros,
        body.uncertainty_before_micros,
        body.uncertainty_after_micros,
    )?;
    if body.receipt_valid_until_micros <= latest {
        return Err(HistoricalActivationTimeReceiptErrorV2::ReceiptValidityInvalid);
    }

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_RECEIPT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(activation_subject_sha256);
    hasher.update([0x02]);
    hasher.update(time_authority_policy_sha256);
    update_len_prefixed_v2(&mut hasher, 0x03, body.time_authority_id.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x04, body.time_authority_key_id.as_bytes());
    hasher.update([0x05]);
    hasher.update(time_authority_key_generation_sha256);
    hasher.update([0x06]);
    hasher.update(body.algorithm.as_u16().to_be_bytes());
    hasher.update([0x07]);
    hasher.update([body.time_basis.as_u8()]);
    hasher.update([0x08]);
    hasher.update(body.observed_at_micros.to_be_bytes());
    hasher.update([0x09]);
    hasher.update(body.uncertainty_before_micros.to_be_bytes());
    hasher.update([0x0a]);
    hasher.update(body.uncertainty_after_micros.to_be_bytes());
    hasher.update([0x0b]);
    hasher.update(body.receipt_valid_until_micros.to_be_bytes());
    Ok(hasher.finalize().into())
}

/// Prepare one receipt for later policy binding and cryptographic authentication.
///
/// Success is structural only. In particular, this result is not trusted time evidence.
pub fn prepare_historical_activation_time_receipt_v2(
    body: HistoricalActivationTimeReceiptBodyV2<'_>,
) -> Result<PreparedHistoricalActivationTimeReceiptV2, HistoricalActivationTimeReceiptErrorV2> {
    let activation_subject_sha256 = require_digest_v2(
        body.activation_subject_sha256,
        HistoricalActivationTimeReceiptErrorV2::ActivationSubjectDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::ActivationSubjectDigestAllZero,
    )?;
    let time_authority_policy_sha256 = require_digest_v2(
        body.time_authority_policy_sha256,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityPolicyDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityPolicyDigestAllZero,
    )?;
    let time_authority_key_generation_sha256 = require_digest_v2(
        body.time_authority_key_generation_sha256,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityKeyGenerationDigestLengthInvalid,
        HistoricalActivationTimeReceiptErrorV2::TimeAuthorityKeyGenerationDigestAllZero,
    )?;
    let signing_digest_sha256 = derive_historical_activation_time_receipt_signing_digest_v2(body)?;
    let (earliest_plausible_observation_micros, latest_plausible_observation_micros) =
        observation_interval_v2(
            body.observed_at_micros,
            body.uncertainty_before_micros,
            body.uncertainty_after_micros,
        )?;

    Ok(PreparedHistoricalActivationTimeReceiptV2 {
        activation_subject_sha256,
        time_authority_policy_sha256,
        time_authority_key_generation_sha256,
        signing_digest_sha256,
        time_authority_id: body.time_authority_id.to_string(),
        time_authority_key_id: body.time_authority_key_id.to_string(),
        algorithm: body.algorithm,
        time_basis: body.time_basis,
        observed_at_micros: body.observed_at_micros,
        uncertainty_before_micros: body.uncertainty_before_micros,
        uncertainty_after_micros: body.uncertainty_after_micros,
        earliest_plausible_observation_micros,
        latest_plausible_observation_micros,
        receipt_valid_until_micros: body.receipt_valid_until_micros,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    static ACTIVATION_SUBJECT: [u8; SHA256_DIGEST_LEN_V2] = [0x11; SHA256_DIGEST_LEN_V2];
    static POLICY: [u8; SHA256_DIGEST_LEN_V2] = [0x22; SHA256_DIGEST_LEN_V2];
    static KEY_GENERATION: [u8; SHA256_DIGEST_LEN_V2] = [0x33; SHA256_DIGEST_LEN_V2];

    fn receipt() -> HistoricalActivationTimeReceiptBodyV2<'static> {
        HistoricalActivationTimeReceiptBodyV2 {
            activation_subject_sha256: &ACTIVATION_SUBJECT,
            time_authority_policy_sha256: &POLICY,
            time_authority_id: "time:authority:primary-v2",
            time_authority_key_id: "time:authority:primary-v2#hybrid-1",
            time_authority_key_generation_sha256: &KEY_GENERATION,
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            time_basis: TimeBasisV2::UnixMicrosecondsUtc,
            observed_at_micros: 2_000_000,
            uncertainty_before_micros: 5_000,
            uncertainty_after_micros: 7_000,
            receipt_valid_until_micros: 3_000_000,
        }
    }

    #[test]
    fn frozen_time_receipt_signing_digest_is_stable() {
        assert_eq!(
            derive_historical_activation_time_receipt_signing_digest_v2(receipt()).unwrap(),
            [
                0xde, 0xc2, 0x08, 0xa7, 0x8f, 0x1e, 0x1b, 0x06, 0xa0, 0xf6, 0x6a, 0x99,
                0x99, 0x21, 0x84, 0x53, 0x7f, 0x8e, 0xc1, 0x63, 0x91, 0xf4, 0xcf, 0xb0,
                0x39, 0x7b, 0xab, 0x4b, 0x08, 0xa5, 0xf8, 0xd6,
            ]
        );
    }

    #[test]
    fn conservative_activation_uses_latest_plausible_observation() {
        let prepared = prepare_historical_activation_time_receipt_v2(receipt()).unwrap();
        assert_eq!(prepared.earliest_plausible_observation_micros(), 1_995_000);
        assert_eq!(prepared.latest_plausible_observation_micros(), 2_007_000);
        assert_eq!(prepared.conservative_activation_bound_micros(), 2_007_000);
        assert_ne!(prepared.conservative_activation_bound_micros(), 1_995_000);
        assert_ne!(prepared.conservative_activation_bound_micros(), 2_000_000);
    }

    #[test]
    fn uncertainty_cannot_overflow_or_backdate_interval() {
        let mut too_early = receipt();
        too_early.observed_at_micros = 1;
        too_early.uncertainty_before_micros = 2;
        assert_eq!(
            prepare_historical_activation_time_receipt_v2(too_early).unwrap_err(),
            HistoricalActivationTimeReceiptErrorV2::ObservationIntervalUnderflow
        );

        let mut overflow = receipt();
        overflow.observed_at_micros = i64::MAX - 1;
        overflow.uncertainty_after_micros = 2;
        assert_eq!(
            prepare_historical_activation_time_receipt_v2(overflow).unwrap_err(),
            HistoricalActivationTimeReceiptErrorV2::ObservationIntervalOverflow
        );
    }

    #[test]
    fn receipt_reuse_ceiling_must_follow_latest_plausible_observation() {
        let mut invalid = receipt();
        invalid.receipt_valid_until_micros = 2_007_000;
        assert_eq!(
            prepare_historical_activation_time_receipt_v2(invalid).unwrap_err(),
            HistoricalActivationTimeReceiptErrorV2::ReceiptValidityInvalid
        );
    }

    #[test]
    fn non_signature_time_authority_algorithm_fails_closed() {
        let mut invalid = receipt();
        invalid.algorithm = AlgorithmId::MlKem768;
        assert_eq!(
            prepare_historical_activation_time_receipt_v2(invalid).unwrap_err(),
            HistoricalActivationTimeReceiptErrorV2::NonSignatureAlgorithm
        );
    }

    #[test]
    fn prepared_receipt_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimeReceiptV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimeReceiptV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub activation_subject_sha256:",
            "pub signing_digest_sha256:",
            "pub latest_plausible_observation_micros:",
            "pub receipt_valid_until_micros:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
