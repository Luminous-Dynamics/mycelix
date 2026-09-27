// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3A3A — structural RFC 9578 issuer-directory/key semantics.
//!
//! This crate parses no HTTP and verifies no TLS/signature/time authority. It
//! only establishes consistency between one exact B3A query-credit policy and
//! one normalized issuer-directory observation.

#![forbid(unsafe_code)]

use psi_privacy_pass_credit_core::{QueryCreditPolicyV1, Rfc9578TokenType};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const ISSUER_DIRECTORY_MEDIA_TYPE: &str = "application/private-token-issuer-directory";
pub const DIRECTORY_OBSERVATION_DOMAIN_V1: &str = "mycelix-psi-issuer-directory-observation-v1";
pub const DIRECTORY_ADMISSION_POLICY_DOMAIN_V1: &str = "mycelix-psi-issuer-directory-admission-policy-v1";
pub const MAX_DIRECTORY_KEYS: usize = 64;
pub const MAX_PUBLIC_KEY_BYTES: usize = 4096;
pub const MAX_TEXT_BYTES: usize = 2048;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum IssuerDirectoryStructuralFailure {
    InvalidQueryCreditPolicy,
    InvalidAdmissionPolicy,
    InvalidIssuerName,
    InvalidDirectoryOrigin,
    InvalidIssuerRequestUri,
    InvalidMediaType,
    InvalidResponseBodyDigest,
    EmptyKeySet,
    TooManyKeys,
    InvalidKeyEntry,
    PublicKeyTooLarge,
    NonCanonicalKeyId,
    PublicKeyDigestMismatch,
    DuplicateFullKeyId,
    InvalidCacheObservation,
    InvalidRetrievalEvidence,
    QueryPolicyCommitmentMismatch,
    IssuerMismatch,
    DirectoryOriginMismatch,
    RetrievalProfileMismatch,
    ConfigurationDigestMismatch,
    TokenTypeMismatch,
    ExactFullKeyAbsent,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IssuerDirectoryKeyObservationV1 {
    pub token_type: Rfc9578TokenType,
    pub public_key_spki_der: Vec<u8>,
    pub token_key_id_sha256: String,
    pub not_before_unix_seconds: Option<u64>,
}

impl IssuerDirectoryKeyObservationV1 {
    pub fn validate(&self) -> Result<(), IssuerDirectoryStructuralFailure> {
        if self.public_key_spki_der.is_empty() {
            return Err(IssuerDirectoryStructuralFailure::InvalidKeyEntry);
        }
        if self.public_key_spki_der.len() > MAX_PUBLIC_KEY_BYTES {
            return Err(IssuerDirectoryStructuralFailure::PublicKeyTooLarge);
        }
        if !is_canonical_sha256_hex(&self.token_key_id_sha256) {
            return Err(IssuerDirectoryStructuralFailure::NonCanonicalKeyId);
        }
        if sha256_hex(&self.public_key_spki_der) != self.token_key_id_sha256 {
            return Err(IssuerDirectoryStructuralFailure::PublicKeyDigestMismatch);
        }
        Ok(())
    }

    pub fn truncated_token_key_id(&self) -> Result<u8, IssuerDirectoryStructuralFailure> {
        self.validate()?;
        decode_hex_byte(&self.token_key_id_sha256[62..64])
            .ok_or(IssuerDirectoryStructuralFailure::NonCanonicalKeyId)
    }

    fn append_commitment_fields(&self, out: &mut Vec<u8>) -> Result<(), IssuerDirectoryStructuralFailure> {
        self.validate()?;
        append_field(out, &self.token_type.code().to_be_bytes());
        append_field(out, self.token_type.wire_id().as_bytes());
        append_field(out, &self.public_key_spki_der);
        append_field(out, self.token_key_id_sha256.as_bytes());
        match self.not_before_unix_seconds {
            Some(value) => {
                append_field(out, b"not-before-present-v1");
                append_field(out, &value.to_be_bytes());
            }
            None => append_field(out, b"not-before-absent-v1"),
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct HttpDirectoryCacheObservationV1 {
    pub response_date_unix_seconds: Option<u64>,
    pub age_seconds: Option<u64>,
    pub cache_control_max_age_seconds: Option<u64>,
    pub etag_sha256: Option<String>,
    pub last_modified_unix_seconds: Option<u64>,
    pub provider_observed_at_unix_seconds: Option<u64>,
}

impl HttpDirectoryCacheObservationV1 {
    pub fn validate(&self) -> Result<(), IssuerDirectoryStructuralFailure> {
        if let Some(etag) = &self.etag_sha256 {
            if !is_canonical_sha256_hex(etag) {
                return Err(IssuerDirectoryStructuralFailure::InvalidCacheObservation);
            }
        }
        Ok(())
    }

    fn append_commitment_fields(&self, out: &mut Vec<u8>) -> Result<(), IssuerDirectoryStructuralFailure> {
        self.validate()?;
        append_optional_u64(out, "response-date", self.response_date_unix_seconds);
        append_optional_u64(out, "age", self.age_seconds);
        append_optional_u64(out, "max-age", self.cache_control_max_age_seconds);
        match &self.etag_sha256 {
            Some(value) => {
                append_field(out, b"etag-present-v1");
                append_field(out, value.as_bytes());
            }
            None => append_field(out, b"etag-absent-v1"),
        }
        append_optional_u64(out, "last-modified", self.last_modified_unix_seconds);
        append_optional_u64(out, "provider-observed-at", self.provider_observed_at_unix_seconds);
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IssuerDirectoryObservationV1 {
    pub issuer_name: String,
    pub directory_origin: String,
    pub issuer_request_uri: String,
    pub media_type: String,
    pub response_body_sha256: String,
    pub token_keys: Vec<IssuerDirectoryKeyObservationV1>,
    pub cache: HttpDirectoryCacheObservationV1,
    pub retrieval_provider: String,
    pub retrieval_profile: String,
    pub retrieval_receipt_sha256: String,
}

impl IssuerDirectoryObservationV1 {
    pub fn validate(&self) -> Result<(), IssuerDirectoryStructuralFailure> {
        if !valid_server_name(&self.issuer_name) {
            return Err(IssuerDirectoryStructuralFailure::InvalidIssuerName);
        }
        if !valid_https_origin(&self.directory_origin) {
            return Err(IssuerDirectoryStructuralFailure::InvalidDirectoryOrigin);
        }
        if !valid_request_uri(&self.issuer_request_uri) {
            return Err(IssuerDirectoryStructuralFailure::InvalidIssuerRequestUri);
        }
        if self.media_type != ISSUER_DIRECTORY_MEDIA_TYPE {
            return Err(IssuerDirectoryStructuralFailure::InvalidMediaType);
        }
        if !is_canonical_sha256_hex(&self.response_body_sha256) {
            return Err(IssuerDirectoryStructuralFailure::InvalidResponseBodyDigest);
        }
        if self.token_keys.is_empty() {
            return Err(IssuerDirectoryStructuralFailure::EmptyKeySet);
        }
        if self.token_keys.len() > MAX_DIRECTORY_KEYS {
            return Err(IssuerDirectoryStructuralFailure::TooManyKeys);
        }
        let mut full_ids = BTreeSet::new();
        for key in &self.token_keys {
            key.validate()?;
            if !full_ids.insert(key.token_key_id_sha256.clone()) {
                return Err(IssuerDirectoryStructuralFailure::DuplicateFullKeyId);
            }
        }
        self.cache.validate()?;
        for value in [&self.retrieval_provider, &self.retrieval_profile] {
            if value.trim().is_empty() || !value.is_ascii() || value.len() > MAX_TEXT_BYTES {
                return Err(IssuerDirectoryStructuralFailure::InvalidRetrievalEvidence);
            }
        }
        if !is_canonical_sha256_hex(&self.retrieval_receipt_sha256) {
            return Err(IssuerDirectoryStructuralFailure::InvalidRetrievalEvidence);
        }
        Ok(())
    }

    pub fn commitment_sha256(&self) -> Result<String, IssuerDirectoryStructuralFailure> {
        self.validate()?;
        let mut out = Vec::new();
        append_field(&mut out, DIRECTORY_OBSERVATION_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.issuer_name.as_bytes());
        append_field(&mut out, self.directory_origin.as_bytes());
        append_field(&mut out, self.issuer_request_uri.as_bytes());
        append_field(&mut out, self.media_type.as_bytes());
        append_field(&mut out, self.response_body_sha256.as_bytes());
        append_field(&mut out, &(self.token_keys.len() as u32).to_be_bytes());
        for key in &self.token_keys {
            key.append_commitment_fields(&mut out)?;
        }
        self.cache.append_commitment_fields(&mut out)?;
        append_field(&mut out, self.retrieval_provider.as_bytes());
        append_field(&mut out, self.retrieval_profile.as_bytes());
        append_field(&mut out, self.retrieval_receipt_sha256.as_bytes());
        Ok(sha256_hex(&out))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct IssuerDirectoryAdmissionPolicyV1 {
    query_credit_policy_sha256: String,
    service_domain: String,
    issuer_name: String,
    directory_origin: String,
    token_type: Rfc9578TokenType,
    required_full_token_key_id_sha256: String,
    retrieval_profile: String,
    freshness_profile: String,
    max_directory_age_seconds: u64,
}

impl IssuerDirectoryAdmissionPolicyV1 {
    pub fn new(
        query_policy: &QueryCreditPolicyV1,
        directory_origin: impl Into<String>,
        retrieval_profile: impl Into<String>,
        freshness_profile: impl Into<String>,
        max_directory_age_seconds: u64,
    ) -> Result<Self, IssuerDirectoryStructuralFailure> {
        query_policy
            .validate()
            .map_err(|_| IssuerDirectoryStructuralFailure::InvalidQueryCreditPolicy)?;
        if !is_canonical_sha256_hex(&query_policy.issuer_configuration_sha256)
            || !is_canonical_sha256_hex(&query_policy.token_key_id_sha256)
        {
            return Err(IssuerDirectoryStructuralFailure::InvalidQueryCreditPolicy);
        }
        let directory_origin = directory_origin.into();
        let retrieval_profile = retrieval_profile.into();
        let freshness_profile = freshness_profile.into();
        if !valid_https_origin(&directory_origin)
            || retrieval_profile.trim().is_empty()
            || freshness_profile.trim().is_empty()
            || !retrieval_profile.is_ascii()
            || !freshness_profile.is_ascii()
            || max_directory_age_seconds == 0
        {
            return Err(IssuerDirectoryStructuralFailure::InvalidAdmissionPolicy);
        }
        Ok(Self {
            query_credit_policy_sha256: query_policy
                .commitment_sha256()
                .map_err(|_| IssuerDirectoryStructuralFailure::InvalidQueryCreditPolicy)?,
            service_domain: query_policy.service_domain.clone(),
            issuer_name: query_policy.issuer_name.clone(),
            directory_origin,
            token_type: query_policy.token_type,
            required_full_token_key_id_sha256: query_policy.token_key_id_sha256.clone(),
            retrieval_profile,
            freshness_profile,
            max_directory_age_seconds,
        })
    }

    pub fn commitment_sha256(&self) -> String {
        let mut out = Vec::new();
        append_field(&mut out, DIRECTORY_ADMISSION_POLICY_DOMAIN_V1.as_bytes());
        append_field(&mut out, self.query_credit_policy_sha256.as_bytes());
        append_field(&mut out, self.service_domain.as_bytes());
        append_field(&mut out, self.issuer_name.as_bytes());
        append_field(&mut out, self.directory_origin.as_bytes());
        append_field(&mut out, &self.token_type.code().to_be_bytes());
        append_field(&mut out, self.token_type.wire_id().as_bytes());
        append_field(&mut out, self.required_full_token_key_id_sha256.as_bytes());
        append_field(&mut out, self.retrieval_profile.as_bytes());
        append_field(&mut out, self.freshness_profile.as_bytes());
        append_field(&mut out, &self.max_directory_age_seconds.to_be_bytes());
        sha256_hex(&out)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructurallyConsistentIssuerDirectoryKeyObservationV1 {
    admission_policy_sha256: String,
    query_credit_policy_sha256: String,
    directory_observation_sha256: String,
    response_body_sha256: String,
    exact_full_token_key_id_sha256: String,
    exact_spki_sha256: String,
    source_order_index: u16,
    truncated_token_key_id: u8,
    colliding_full_key_ids_at_truncated_id: Vec<String>,
    not_before_unix_seconds: Option<u64>,
    cache_observation: HttpDirectoryCacheObservationV1,
    retrieval_provider: String,
    retrieval_profile: String,
}

impl StructurallyConsistentIssuerDirectoryKeyObservationV1 {
    pub fn exact_full_token_key_id_sha256(&self) -> &str { &self.exact_full_token_key_id_sha256 }
    pub fn source_order_index(&self) -> u16 { self.source_order_index }
    pub fn truncated_token_key_id(&self) -> u8 { self.truncated_token_key_id }
    pub fn truncated_collision_count(&self) -> usize { self.colliding_full_key_ids_at_truncated_id.len() }
    pub fn not_before_unix_seconds(&self) -> Option<u64> { self.not_before_unix_seconds }
    pub const fn exact_key_present_in_normalized_observation(&self) -> bool { true }
    pub const fn exact_spki_digest_matches_full_key_id(&self) -> bool { true }
    pub const fn spki_rfc9578_profile_verified(&self) -> bool { false }
    pub const fn directory_payload_authenticated(&self) -> bool { false }
    pub const fn retrieval_provider_trusted(&self) -> bool { false }
    pub const fn directory_freshness_established(&self) -> bool { false }
    pub const fn not_before_satisfied_under_trusted_clock(&self) -> bool { false }
    pub const fn issuer_key_admitted_under_service_policy(&self) -> bool { false }
    pub const fn issuer_key_current_under_service_policy(&self) -> bool { false }
    pub const fn token_cryptographically_verified(&self) -> bool { false }
    pub const fn query_credit_granted(&self) -> bool { false }
    pub const fn application_authority_granted(&self) -> bool { false }
}

pub fn evaluate_issuer_directory_structure_v1(
    query_policy: &QueryCreditPolicyV1,
    admission_policy: &IssuerDirectoryAdmissionPolicyV1,
    observation: &IssuerDirectoryObservationV1,
) -> Result<StructurallyConsistentIssuerDirectoryKeyObservationV1, IssuerDirectoryStructuralFailure> {
    query_policy
        .validate()
        .map_err(|_| IssuerDirectoryStructuralFailure::InvalidQueryCreditPolicy)?;
    observation.validate()?;

    let query_policy_sha256 = query_policy
        .commitment_sha256()
        .map_err(|_| IssuerDirectoryStructuralFailure::InvalidQueryCreditPolicy)?;
    if admission_policy.query_credit_policy_sha256 != query_policy_sha256
        || admission_policy.service_domain != query_policy.service_domain
        || admission_policy.issuer_name != query_policy.issuer_name
        || admission_policy.token_type != query_policy.token_type
        || admission_policy.required_full_token_key_id_sha256 != query_policy.token_key_id_sha256
    {
        return Err(IssuerDirectoryStructuralFailure::QueryPolicyCommitmentMismatch);
    }
    if observation.issuer_name != admission_policy.issuer_name {
        return Err(IssuerDirectoryStructuralFailure::IssuerMismatch);
    }
    if observation.directory_origin != admission_policy.directory_origin {
        return Err(IssuerDirectoryStructuralFailure::DirectoryOriginMismatch);
    }
    if observation.retrieval_profile != admission_policy.retrieval_profile {
        return Err(IssuerDirectoryStructuralFailure::RetrievalProfileMismatch);
    }
    if observation.response_body_sha256 != query_policy.issuer_configuration_sha256 {
        return Err(IssuerDirectoryStructuralFailure::ConfigurationDigestMismatch);
    }

    let (source_index, exact_key) = observation
        .token_keys
        .iter()
        .enumerate()
        .find(|(_, key)| {
            key.token_type == admission_policy.token_type
                && key.token_key_id_sha256 == admission_policy.required_full_token_key_id_sha256
        })
        .ok_or(IssuerDirectoryStructuralFailure::ExactFullKeyAbsent)?;
    if exact_key.token_type != query_policy.token_type {
        return Err(IssuerDirectoryStructuralFailure::TokenTypeMismatch);
    }
    let source_order_index = u16::try_from(source_index)
        .map_err(|_| IssuerDirectoryStructuralFailure::TooManyKeys)?;
    let truncated = exact_key.truncated_token_key_id()?;
    let colliding_full_key_ids_at_truncated_id = observation
        .token_keys
        .iter()
        .filter_map(|key| {
            let candidate = key.truncated_token_key_id().ok()?;
            if key.token_type == exact_key.token_type
                && candidate == truncated
                && key.token_key_id_sha256 != exact_key.token_key_id_sha256
            {
                Some(key.token_key_id_sha256.clone())
            } else {
                None
            }
        })
        .collect();

    Ok(StructurallyConsistentIssuerDirectoryKeyObservationV1 {
        admission_policy_sha256: admission_policy.commitment_sha256(),
        query_credit_policy_sha256,
        directory_observation_sha256: observation.commitment_sha256()?,
        response_body_sha256: observation.response_body_sha256.clone(),
        exact_full_token_key_id_sha256: exact_key.token_key_id_sha256.clone(),
        exact_spki_sha256: sha256_hex(&exact_key.public_key_spki_der),
        source_order_index,
        truncated_token_key_id: truncated,
        colliding_full_key_ids_at_truncated_id,
        not_before_unix_seconds: exact_key.not_before_unix_seconds,
        cache_observation: observation.cache.clone(),
        retrieval_provider: observation.retrieval_provider.clone(),
        retrieval_profile: observation.retrieval_profile.clone(),
    })
}

fn valid_server_name(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 255
        && value.is_ascii()
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'-' | b':' | b'[' | b']')
        })
}

fn valid_https_origin(value: &str) -> bool {
    let Some(authority) = value.strip_prefix("https://") else { return false; };
    !authority.is_empty()
        && authority.len() <= MAX_TEXT_BYTES
        && authority.is_ascii()
        && !authority.bytes().any(|byte| matches!(byte, b'/' | b'?' | b'#' | b'@' | b',' | b' ' | b'\t' | b'\r' | b'\n'))
}

fn valid_request_uri(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_TEXT_BYTES
        && value.is_ascii()
        && !value.bytes().any(|byte| matches!(byte, b' ' | b'\t' | b'\r' | b'\n'))
        && (value.starts_with('/') || value.starts_with("https://"))
}

fn is_canonical_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_digit() || matches!(byte, b'a'..=b'f'))
}

fn decode_hex_byte(value: &str) -> Option<u8> {
    if value.len() != 2 { return None; }
    let bytes = value.as_bytes();
    Some((hex_nibble(bytes[0])? << 4) | hex_nibble(bytes[1])?)
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        _ => None,
    }
}

fn append_optional_u64(out: &mut Vec<u8>, name: &str, value: Option<u64>) {
    append_field(out, name.as_bytes());
    match value {
        Some(value) => {
            append_field(out, b"present-v1");
            append_field(out, &value.to_be_bytes());
        }
        None => append_field(out, b"absent-v1"),
    }
}

fn append_field(out: &mut Vec<u8>, field: &[u8]) {
    let len = u32::try_from(field.len()).expect("issuer-directory semantic fields fit u32");
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
    use psi_privacy_pass_credit_core::ReplayPolicyV1;

    fn key(bytes: &[u8], not_before: Option<u64>) -> IssuerDirectoryKeyObservationV1 {
        IssuerDirectoryKeyObservationV1 {
            token_type: Rfc9578TokenType::PublicBlindRsa2048Sha384,
            public_key_spki_der: bytes.to_vec(),
            token_key_id_sha256: sha256_hex(bytes),
            not_before_unix_seconds: not_before,
        }
    }

    fn query_policy(key: &IssuerDirectoryKeyObservationV1) -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(),
            issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32),
            token_type: key.token_type,
            token_key_id_sha256: key.token_key_id_sha256.clone(),
            budget_epoch: "epoch-a".into(),
            max_identifiers_per_credit: 256,
            replay_policy: ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    fn admission(query: &QueryCreditPolicyV1) -> IssuerDirectoryAdmissionPolicyV1 {
        IssuerDirectoryAdmissionPolicyV1::new(
            query,
            "https://issuer.mycelix.test",
            "https-directory-fetch-v1",
            "http-cache-plus-trusted-clock-v1",
            86_400,
        )
        .unwrap()
    }

    fn observation(keys: Vec<IssuerDirectoryKeyObservationV1>) -> IssuerDirectoryObservationV1 {
        IssuerDirectoryObservationV1 {
            issuer_name: "issuer.mycelix.test".into(),
            directory_origin: "https://issuer.mycelix.test".into(),
            issuer_request_uri: "/request".into(),
            media_type: ISSUER_DIRECTORY_MEDIA_TYPE.into(),
            response_body_sha256: "11".repeat(32),
            token_keys: keys,
            cache: HttpDirectoryCacheObservationV1 {
                response_date_unix_seconds: Some(1_800_000_000),
                age_seconds: Some(10),
                cache_control_max_age_seconds: Some(86_400),
                etag_sha256: Some("22".repeat(32)),
                last_modified_unix_seconds: Some(1_799_999_000),
                provider_observed_at_unix_seconds: Some(1_800_000_010),
            },
            retrieval_provider: "synthetic-http-observer".into(),
            retrieval_profile: "https-directory-fetch-v1".into(),
            retrieval_receipt_sha256: "33".repeat(32),
        }
    }

    #[test]
    fn exact_policy_key_becomes_structural_positive_only() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let query = query_policy(&exact);
        let positive = evaluate_issuer_directory_structure_v1(
            &query,
            &admission(&query),
            &observation(vec![exact.clone()]),
        )
        .unwrap();
        assert_eq!(positive.exact_full_token_key_id_sha256(), exact.token_key_id_sha256);
        assert!(positive.exact_key_present_in_normalized_observation());
        assert!(positive.exact_spki_digest_matches_full_key_id());
        assert!(!positive.spki_rfc9578_profile_verified());
        assert!(!positive.directory_payload_authenticated());
        assert!(!positive.directory_freshness_established());
        assert!(!positive.issuer_key_admitted_under_service_policy());
        assert!(!positive.issuer_key_current_under_service_policy());
        assert!(!positive.query_credit_granted());
    }

    #[test]
    fn spki_digest_mismatch_fails_closed() {
        let mut exact = key(&[0x30, 1, 2, 3], None);
        exact.token_key_id_sha256 = "00".repeat(32);
        assert_eq!(exact.validate(), Err(IssuerDirectoryStructuralFailure::PublicKeyDigestMismatch));
    }

    #[test]
    fn exact_full_key_absence_is_not_resolved_by_same_token_type() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let other = key(&[0x30, 9, 9, 9], None);
        let query = query_policy(&exact);
        assert_eq!(
            evaluate_issuer_directory_structure_v1(&query, &admission(&query), &observation(vec![other])),
            Err(IssuerDirectoryStructuralFailure::ExactFullKeyAbsent)
        );
    }

    #[test]
    fn duplicate_full_key_id_is_rejected() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let duplicate = exact.clone();
        assert_eq!(
            observation(vec![exact, duplicate]).validate(),
            Err(IssuerDirectoryStructuralFailure::DuplicateFullKeyId)
        );
    }

    #[test]
    fn source_order_is_preserved_and_changes_observation_identity() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let other = key(&[0x30, 9, 9, 9], None);
        let query = query_policy(&exact);
        let first = observation(vec![exact.clone(), other.clone()]);
        let second = observation(vec![other, exact]);
        assert_ne!(first.commitment_sha256().unwrap(), second.commitment_sha256().unwrap());
        assert_eq!(
            evaluate_issuer_directory_structure_v1(&query, &admission(&query), &first)
                .unwrap()
                .source_order_index(),
            0
        );
        assert_eq!(
            evaluate_issuer_directory_structure_v1(&query, &admission(&query), &second)
                .unwrap()
                .source_order_index(),
            1
        );
    }

    #[test]
    fn truncated_id_collision_is_surfaced_without_equating_full_keys() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let target = exact.truncated_token_key_id().unwrap();
        let mut collision = None;
        for counter in 0u32..=65_535 {
            let mut bytes = vec![0x30, 7, 7, 7];
            bytes.extend_from_slice(&counter.to_be_bytes());
            let candidate = key(&bytes, None);
            if candidate.token_key_id_sha256 != exact.token_key_id_sha256
                && candidate.truncated_token_key_id().unwrap() == target
            {
                collision = Some(candidate);
                break;
            }
        }
        let collision = collision.expect("deterministic search finds an 8-bit suffix collision");
        let query = query_policy(&exact);
        let positive = evaluate_issuer_directory_structure_v1(
            &query,
            &admission(&query),
            &observation(vec![collision, exact]),
        )
        .unwrap();
        assert_eq!(positive.truncated_collision_count(), 1);
        assert_eq!(positive.truncated_token_key_id(), target);
    }

    #[test]
    fn not_before_is_retained_but_never_mints_currentness() {
        let exact = key(&[0x30, 1, 2, 3], Some(1_900_000_000));
        let query = query_policy(&exact);
        let positive = evaluate_issuer_directory_structure_v1(
            &query,
            &admission(&query),
            &observation(vec![exact]),
        )
        .unwrap();
        assert_eq!(positive.not_before_unix_seconds(), Some(1_900_000_000));
        assert!(!positive.not_before_satisfied_under_trusted_clock());
        assert!(!positive.issuer_key_current_under_service_policy());
    }

    #[test]
    fn friendly_http_cache_claims_do_not_establish_freshness() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let query = query_policy(&exact);
        let positive = evaluate_issuer_directory_structure_v1(
            &query,
            &admission(&query),
            &observation(vec![exact]),
        )
        .unwrap();
        assert!(!positive.retrieval_provider_trusted());
        assert!(!positive.directory_freshness_established());
    }

    #[test]
    fn wrong_configuration_digest_is_rejected() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let query = query_policy(&exact);
        let mut observed = observation(vec![exact]);
        observed.response_body_sha256 = "44".repeat(32);
        assert_eq!(
            evaluate_issuer_directory_structure_v1(&query, &admission(&query), &observed),
            Err(IssuerDirectoryStructuralFailure::ConfigurationDigestMismatch)
        );
    }

    #[test]
    fn retrieval_profile_substitution_is_rejected() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let query = query_policy(&exact);
        let mut observed = observation(vec![exact]);
        observed.retrieval_profile = "other-fetcher-v1".into();
        assert_eq!(
            evaluate_issuer_directory_structure_v1(&query, &admission(&query), &observed),
            Err(IssuerDirectoryStructuralFailure::RetrievalProfileMismatch)
        );
    }

    #[test]
    fn raw_observation_round_trip_remains_non_authoritative() {
        let exact = key(&[0x30, 1, 2, 3], None);
        let raw = observation(vec![exact]);
        let bytes = serde_json::to_vec(&raw).unwrap();
        let decoded: IssuerDirectoryObservationV1 = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, raw);
    }
}
