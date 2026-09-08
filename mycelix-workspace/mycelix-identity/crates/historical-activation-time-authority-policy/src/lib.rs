// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Static time-authority policy for Identity V2 historical activation.
//!
//! This pure theorem defines what one prepared #394 time receipt is structurally allowed
//! to claim. It binds one exact authority/key generation/algorithm, UTC realization
//! profile, uncertainty ceilings, reuse lifetime, and static validity interval.
//!
//! Success here does not establish that this policy is accepted/current, that the time
//! receipt signature is authentic, or that the referenced activation subject is
//! historically eligible.

#![forbid(unsafe_code)]

use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_receipt_policy::{
    PreparedHistoricalActivationTimeReceiptV2, TimeBasisV2,
};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const POLICY_ID_MAX_LEN_V2: usize = 256;
pub const TIME_AUTHORITY_ID_MAX_LEN_V2: usize = 256;
pub const TIME_AUTHORITY_KEY_ID_MAX_LEN_V2: usize = 512;
pub const UTC_REALIZATION_ID_MAX_LEN_V2: usize = 256;
pub const HISTORICAL_ACTIVATION_TIME_AUTHORITY_POLICY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-authority-policy:v2\0";

#[derive(Debug, Clone, Copy)]
pub struct HistoricalActivationTimeAuthorityPolicyBodyV2<'a> {
    pub policy_id: &'a str,
    pub policy_version: u32,
    pub time_authority_id: &'a str,
    pub time_authority_key_id: &'a str,
    pub time_authority_key_generation_sha256: &'a [u8],
    pub algorithm: AlgorithmId,
    pub time_basis: TimeBasisV2,
    /// Canonical profile describing how a provider's source time is normalized into the
    /// signed Unix-microseconds-UTC representation. Any normalization uncertainty must be
    /// included in the receipt uncertainty bounds.
    pub utc_realization_id: &'a str,
    pub max_uncertainty_before_micros: u64,
    pub max_uncertainty_after_micros: u64,
    /// Maximum signed reuse lifetime measured from the latest plausible observation time.
    pub max_receipt_lifetime_micros: u64,
    /// Static policy applicability interval, interpreted as `[valid_from, valid_until)`.
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
}

#[derive(Debug)]
pub struct QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
    policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_id: String,
    policy_version: u32,
    time_authority_id: String,
    time_authority_key_id: String,
    time_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    time_basis: TimeBasisV2,
    utc_realization_id: String,
    max_uncertainty_before_micros: u64,
    max_uncertainty_after_micros: u64,
    max_receipt_lifetime_micros: u64,
    valid_from_micros: i64,
    valid_until_micros: i64,
}

impl QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
    pub fn policy_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_digest_sha256
    }

    pub fn policy_id(&self) -> &str {
        &self.policy_id
    }

    pub fn policy_version(&self) -> u32 {
        self.policy_version
    }

    pub fn time_authority_id(&self) -> &str {
        &self.time_authority_id
    }

    pub fn time_authority_key_id(&self) -> &str {
        &self.time_authority_key_id
    }

    pub fn time_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.time_authority_key_generation_sha256
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn time_basis(&self) -> TimeBasisV2 {
        self.time_basis
    }

    pub fn utc_realization_id(&self) -> &str {
        &self.utc_realization_id
    }

    pub fn max_uncertainty_before_micros(&self) -> u64 {
        self.max_uncertainty_before_micros
    }

    pub fn max_uncertainty_after_micros(&self) -> u64 {
        self.max_uncertainty_after_micros
    }

    pub fn max_receipt_lifetime_micros(&self) -> u64 {
        self.max_receipt_lifetime_micros
    }

    pub fn valid_from_micros(&self) -> i64 {
        self.valid_from_micros
    }

    pub fn valid_until_micros(&self) -> i64 {
        self.valid_until_micros
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimeAuthorityPolicyErrorV2 {
    PolicyIdInvalid,
    PolicyVersionInvalid,
    TimeAuthorityIdInvalid,
    TimeAuthorityKeyIdInvalid,
    TimeAuthorityKeyGenerationDigestLengthInvalid,
    TimeAuthorityKeyGenerationDigestAllZero,
    NonSignatureAlgorithm,
    UtcRealizationIdInvalid,
    MaxUncertaintyTooLarge,
    MaxReceiptLifetimeInvalid,
    PolicyValidityInvalid,
    ReceiptPolicyDigestMismatch,
    ReceiptAuthorityIdMismatch,
    ReceiptKeyIdMismatch,
    ReceiptKeyGenerationMismatch,
    ReceiptAlgorithmMismatch,
    ReceiptTimeBasisMismatch,
    ReceiptUncertaintyExceedsPolicy,
    ReceiptObservationOutsidePolicyValidity,
    ReceiptValidityExceedsPolicy,
    ReceiptLifetimeExceedsPolicy,
}

fn valid_visible_ascii_identifier(value: &str, max_len: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_len
        && value.is_ascii()
        && value
            .as_bytes()
            .iter()
            .all(|byte| (0x21..=0x7e).contains(byte))
}

fn require_generation_digest_v2(
    value: &[u8],
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeAuthorityPolicyErrorV2> {
    let digest: [u8; SHA256_DIGEST_LEN_V2] = value.try_into().map_err(|_| {
        HistoricalActivationTimeAuthorityPolicyErrorV2::TimeAuthorityKeyGenerationDigestLengthInvalid
    })?;
    if digest.iter().all(|byte| *byte == 0) {
        return Err(
            HistoricalActivationTimeAuthorityPolicyErrorV2::TimeAuthorityKeyGenerationDigestAllZero,
        );
    }
    Ok(digest)
}

fn validate_policy_body_v2(
    body: HistoricalActivationTimeAuthorityPolicyBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeAuthorityPolicyErrorV2> {
    if !valid_visible_ascii_identifier(body.policy_id, POLICY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::PolicyIdInvalid);
    }
    if body.policy_version == 0 {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::PolicyVersionInvalid);
    }
    if !valid_visible_ascii_identifier(body.time_authority_id, TIME_AUTHORITY_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::TimeAuthorityIdInvalid);
    }
    if !valid_visible_ascii_identifier(
        body.time_authority_key_id,
        TIME_AUTHORITY_KEY_ID_MAX_LEN_V2,
    ) {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::TimeAuthorityKeyIdInvalid);
    }
    let key_generation = require_generation_digest_v2(body.time_authority_key_generation_sha256)?;
    if !body.algorithm.is_signature_algorithm() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::NonSignatureAlgorithm);
    }
    if !valid_visible_ascii_identifier(body.utc_realization_id, UTC_REALIZATION_ID_MAX_LEN_V2) {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::UtcRealizationIdInvalid);
    }
    if body.max_uncertainty_before_micros > i64::MAX as u64
        || body.max_uncertainty_after_micros > i64::MAX as u64
    {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::MaxUncertaintyTooLarge);
    }
    if body.max_receipt_lifetime_micros == 0
        || body.max_receipt_lifetime_micros > i64::MAX as u64
    {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::MaxReceiptLifetimeInvalid);
    }
    if body.valid_from_micros <= 0 || body.valid_until_micros <= body.valid_from_micros {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::PolicyValidityInvalid);
    }
    Ok(key_generation)
}

fn update_len_prefixed_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn derive_historical_activation_time_authority_policy_digest_v2(
    body: HistoricalActivationTimeAuthorityPolicyBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], HistoricalActivationTimeAuthorityPolicyErrorV2> {
    let key_generation = validate_policy_body_v2(body)?;
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_AUTHORITY_POLICY_DOMAIN_V2);
    update_len_prefixed_v2(&mut hasher, 0x01, body.policy_id.as_bytes());
    hasher.update([0x02]);
    hasher.update(body.policy_version.to_be_bytes());
    update_len_prefixed_v2(&mut hasher, 0x03, body.time_authority_id.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x04, body.time_authority_key_id.as_bytes());
    hasher.update([0x05]);
    hasher.update(key_generation);
    hasher.update([0x06]);
    hasher.update(body.algorithm.as_u16().to_be_bytes());
    hasher.update([0x07]);
    hasher.update([body.time_basis.as_u8()]);
    update_len_prefixed_v2(&mut hasher, 0x08, body.utc_realization_id.as_bytes());
    hasher.update([0x09]);
    hasher.update(body.max_uncertainty_before_micros.to_be_bytes());
    hasher.update([0x0a]);
    hasher.update(body.max_uncertainty_after_micros.to_be_bytes());
    hasher.update([0x0b]);
    hasher.update(body.max_receipt_lifetime_micros.to_be_bytes());
    hasher.update([0x0c]);
    hasher.update(body.valid_from_micros.to_be_bytes());
    hasher.update([0x0d]);
    hasher.update(body.valid_until_micros.to_be_bytes());
    Ok(hasher.finalize().into())
}

pub fn qualify_static_historical_activation_time_authority_policy_v2(
    body: HistoricalActivationTimeAuthorityPolicyBodyV2<'_>,
) -> Result<
    QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
    HistoricalActivationTimeAuthorityPolicyErrorV2,
> {
    let key_generation = validate_policy_body_v2(body)?;
    let policy_digest_sha256 = derive_historical_activation_time_authority_policy_digest_v2(body)?;
    Ok(QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2 {
        policy_digest_sha256,
        policy_id: body.policy_id.to_string(),
        policy_version: body.policy_version,
        time_authority_id: body.time_authority_id.to_string(),
        time_authority_key_id: body.time_authority_key_id.to_string(),
        time_authority_key_generation_sha256: key_generation,
        algorithm: body.algorithm,
        time_basis: body.time_basis,
        utc_realization_id: body.utc_realization_id.to_string(),
        max_uncertainty_before_micros: body.max_uncertainty_before_micros,
        max_uncertainty_after_micros: body.max_uncertainty_after_micros,
        max_receipt_lifetime_micros: body.max_receipt_lifetime_micros,
        valid_from_micros: body.valid_from_micros,
        valid_until_micros: body.valid_until_micros,
    })
}

/// Bind a prepared #394 receipt to one exact static time-authority policy.
///
/// Success means only structural policy compatibility. It does not authenticate the
/// receipt signature or establish that the policy is accepted/current.
pub fn validate_prepared_time_receipt_against_static_policy_v2(
    receipt: &PreparedHistoricalActivationTimeReceiptV2,
    policy: &QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2,
) -> Result<(), HistoricalActivationTimeAuthorityPolicyErrorV2> {
    if receipt.time_authority_policy_sha256() != policy.policy_digest_sha256() {
        return Err(
            HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptPolicyDigestMismatch,
        );
    }
    if receipt.time_authority_id() != policy.time_authority_id() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptAuthorityIdMismatch);
    }
    if receipt.time_authority_key_id() != policy.time_authority_key_id() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptKeyIdMismatch);
    }
    if receipt.time_authority_key_generation_sha256()
        != policy.time_authority_key_generation_sha256()
    {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptKeyGenerationMismatch);
    }
    if receipt.algorithm() != policy.algorithm() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptAlgorithmMismatch);
    }
    if receipt.time_basis() != policy.time_basis() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptTimeBasisMismatch);
    }
    if receipt.uncertainty_before_micros() > policy.max_uncertainty_before_micros()
        || receipt.uncertainty_after_micros() > policy.max_uncertainty_after_micros()
    {
        return Err(
            HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptUncertaintyExceedsPolicy,
        );
    }

    let earliest = receipt.earliest_plausible_observation_micros();
    let latest = receipt.latest_plausible_observation_micros();
    if earliest < policy.valid_from_micros() || latest >= policy.valid_until_micros() {
        return Err(
            HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptObservationOutsidePolicyValidity,
        );
    }
    if receipt.receipt_valid_until_micros() > policy.valid_until_micros() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptValidityExceedsPolicy);
    }

    let lifetime = receipt
        .receipt_valid_until_micros()
        .checked_sub(latest)
        .and_then(|value| u64::try_from(value).ok())
        .ok_or(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptLifetimeExceedsPolicy)?;
    if lifetime > policy.max_receipt_lifetime_micros() {
        return Err(HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptLifetimeExceedsPolicy);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_receipt_policy::{
        prepare_historical_activation_time_receipt_v2, HistoricalActivationTimeReceiptBodyV2,
    };

    static KEY_GENERATION: [u8; SHA256_DIGEST_LEN_V2] = [0x33; SHA256_DIGEST_LEN_V2];
    static ACTIVATION_SUBJECT: [u8; SHA256_DIGEST_LEN_V2] = [0x11; SHA256_DIGEST_LEN_V2];

    fn policy_body() -> HistoricalActivationTimeAuthorityPolicyBodyV2<'static> {
        HistoricalActivationTimeAuthorityPolicyBodyV2 {
            policy_id: "time-policy:primary-v2",
            policy_version: 1,
            time_authority_id: "time:authority:primary-v2",
            time_authority_key_id: "time:authority:primary-v2#hybrid-1",
            time_authority_key_generation_sha256: &KEY_GENERATION,
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            time_basis: TimeBasisV2::UnixMicrosecondsUtc,
            utc_realization_id: "unix-utc-normalized-v1",
            max_uncertainty_before_micros: 10_000,
            max_uncertainty_after_micros: 10_000,
            max_receipt_lifetime_micros: 1_000_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 10_000_000,
        }
    }

    fn prepared_receipt(
        policy_digest: &[u8; SHA256_DIGEST_LEN_V2],
    ) -> PreparedHistoricalActivationTimeReceiptV2 {
        prepare_historical_activation_time_receipt_v2(HistoricalActivationTimeReceiptBodyV2 {
            activation_subject_sha256: &ACTIVATION_SUBJECT,
            time_authority_policy_sha256: policy_digest,
            time_authority_id: "time:authority:primary-v2",
            time_authority_key_id: "time:authority:primary-v2#hybrid-1",
            time_authority_key_generation_sha256: &KEY_GENERATION,
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            time_basis: TimeBasisV2::UnixMicrosecondsUtc,
            observed_at_micros: 2_000_000,
            uncertainty_before_micros: 5_000,
            uncertainty_after_micros: 7_000,
            receipt_valid_until_micros: 3_000_000,
        })
        .unwrap()
    }

    #[test]
    fn frozen_time_authority_policy_digest_is_stable() {
        assert_eq!(
            derive_historical_activation_time_authority_policy_digest_v2(policy_body()).unwrap(),
            [
                0xaf, 0xff, 0xb7, 0xc1, 0x90, 0x99, 0x1f, 0x2b, 0xcd, 0x72, 0x26, 0x62,
                0x9a, 0x3a, 0x1d, 0x8a, 0x72, 0x59, 0x5a, 0x0f, 0xa3, 0x11, 0x8d, 0xa8,
                0x81, 0x08, 0x1f, 0xdb, 0xb7, 0xcb, 0x1b, 0x82,
            ]
        );
    }

    #[test]
    fn matching_prepared_receipt_satisfies_static_policy() {
        let policy = qualify_static_historical_activation_time_authority_policy_v2(policy_body())
            .unwrap();
        let receipt = prepared_receipt(policy.policy_digest_sha256());
        assert_eq!(
            validate_prepared_time_receipt_against_static_policy_v2(&receipt, &policy),
            Ok(())
        );
    }

    #[test]
    fn uncertainty_exceeding_static_policy_fails_closed() {
        let policy = qualify_static_historical_activation_time_authority_policy_v2(policy_body())
            .unwrap();
        let receipt = prepare_historical_activation_time_receipt_v2(
            HistoricalActivationTimeReceiptBodyV2 {
                activation_subject_sha256: &ACTIVATION_SUBJECT,
                time_authority_policy_sha256: policy.policy_digest_sha256(),
                time_authority_id: policy.time_authority_id(),
                time_authority_key_id: policy.time_authority_key_id(),
                time_authority_key_generation_sha256: policy.time_authority_key_generation_sha256(),
                algorithm: policy.algorithm(),
                time_basis: policy.time_basis(),
                observed_at_micros: 2_000_000,
                uncertainty_before_micros: 10_001,
                uncertainty_after_micros: 7_000,
                receipt_valid_until_micros: 3_000_000,
            },
        )
        .unwrap();
        assert_eq!(
            validate_prepared_time_receipt_against_static_policy_v2(&receipt, &policy),
            Err(
                HistoricalActivationTimeAuthorityPolicyErrorV2::ReceiptUncertaintyExceedsPolicy
            )
        );
    }

    #[test]
    fn realization_profile_changes_policy_identity() {
        let a = derive_historical_activation_time_authority_policy_digest_v2(policy_body()).unwrap();
        let mut changed = policy_body();
        changed.utc_realization_id = "unix-utc-smear-normalized-v1";
        let b = derive_historical_activation_time_authority_policy_digest_v2(changed).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn qualified_policy_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedStaticHistoricalActivationTimeAuthorityPolicyV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub policy_digest_sha256:",
            "pub time_authority_key_generation_sha256:",
            "pub max_uncertainty_after_micros:",
            "pub valid_until_micros:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
