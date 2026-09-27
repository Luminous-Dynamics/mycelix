// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3A2P — pinned RFC 9578 public-token verifier backend preflight.
//!
//! This crate is deliberately a compatibility/evidence probe, not the final
//! Mycelix query-credit verifier. It exercises one exact external backend commit
//! with externally implemented key/nonce stores and preserves the authority split:
//!
//! ```text
//! pinned backend verifies token authenticator
//!     != issuer key trusted/current
//!     != service-wide token unspent
//!     != query credit granted
//! ```

#![forbid(unsafe_code)]

use async_trait::async_trait;
use privacypass::{Deserialize as _, Nonce, NonceStore, Serialize as _, TokenType};
use privacypass::public_tokens::{
    PublicKey, PublicToken, public_key_to_truncated_token_key_id,
    server::{OriginKeyStore, OriginServer, serialize_public_key},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    sync::{
        Mutex,
        atomic::{AtomicUsize, Ordering},
    },
};

pub const BACKEND_REPOSITORY: &str = "https://github.com/raphaelrobert/privacypass";
pub const BACKEND_COMMIT: &str = "5ff5f57a62877f42313d6600b53e0d4ee4e4e452";
pub const BACKEND_PACKAGE: &str = "privacypass";
pub const BACKEND_VERSION: &str = "0.2.0-pre.3";
pub const BACKEND_PROFILE: &str =
    "rfc9578-public-token-type-0002-blind-rsa-pinned-preflight-v1";
pub const BACKEND_AUDIT_STATUS: &str = "independent-audit-not-established";
pub const GO_KAT_SOURCE_PATH: &str = "tests/kat_vectors/public_go.json";
pub const GO_KAT_VECTOR_INDEX: usize = 0;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreflightError {
    InvalidPublicKey,
    PublicKeySpkiRoundTripMismatch,
    InvalidTokenEncoding,
    TrailingTokenBytes,
    WrongTokenType,
    TokenKeyIdMismatch,
    ChallengeDigestMismatch,
    BackendVerificationFailed(String),
    EphemeralNonceStateMismatch,
    InvalidHex,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PublicTokenBackendPreflightReceiptV1 {
    backend_repository: String,
    backend_commit: String,
    backend_package: String,
    backend_version: String,
    backend_profile: String,
    backend_audit_status: String,
    token_type: u16,
    token_sha256: String,
    token_nonce_sha256: String,
    token_challenge_digest_hex: String,
    token_key_id_sha256: String,
    public_key_spki_sha256: String,
    ephemeral_reserve_calls: usize,
    ephemeral_commit_calls: usize,
    ephemeral_release_calls: usize,
}

impl PublicTokenBackendPreflightReceiptV1 {
    pub fn backend_commit(&self) -> &str { &self.backend_commit }
    pub fn token_sha256(&self) -> &str { &self.token_sha256 }
    pub fn token_nonce_sha256(&self) -> &str { &self.token_nonce_sha256 }
    pub fn token_challenge_digest_hex(&self) -> &str { &self.token_challenge_digest_hex }
    pub fn token_key_id_sha256(&self) -> &str { &self.token_key_id_sha256 }
    pub const fn external_origin_key_store_usable(&self) -> bool { true }
    pub const fn external_ephemeral_nonce_store_usable(&self) -> bool { true }
    pub const fn exact_spki_round_trip_established(&self) -> bool { true }
    pub const fn pinned_backend_crypto_path_executed(&self) -> bool { true }
    pub const fn token_fields_bound_to_backend_verified_artifact(&self) -> bool { true }
    pub const fn backend_independently_audited(&self) -> bool { false }
    pub const fn issuer_key_trusted_under_mycelix_policy(&self) -> bool { false }
    pub const fn issuer_key_current_under_mycelix_policy(&self) -> bool { false }
    pub const fn service_wide_unspent_established(&self) -> bool { false }
    pub const fn durable_replay_protection_established(&self) -> bool { false }
    pub const fn query_credit_granted(&self) -> bool { false }
    pub const fn anonymous_rate_limit_established(&self) -> bool { false }
    pub const fn enumeration_resistance_established(&self) -> bool { false }
    pub const fn application_authority_granted(&self) -> bool { false }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EphemeralNonceState { Reserved, Committed }

#[derive(Debug, Default)]
pub struct ExternalEphemeralNonceStoreV1 {
    states: Mutex<HashMap<Nonce, EphemeralNonceState>>,
    reserve_calls: AtomicUsize,
    commit_calls: AtomicUsize,
    release_calls: AtomicUsize,
}

impl ExternalEphemeralNonceStoreV1 {
    fn state(&self, nonce: &Nonce) -> Option<EphemeralNonceState> {
        self.states.lock().expect("ephemeral nonce mutex poisoned").get(nonce).copied()
    }
    fn counts(&self) -> (usize, usize, usize) {
        (
            self.reserve_calls.load(Ordering::SeqCst),
            self.commit_calls.load(Ordering::SeqCst),
            self.release_calls.load(Ordering::SeqCst),
        )
    }
}

#[async_trait]
impl NonceStore for ExternalEphemeralNonceStoreV1 {
    async fn reserve(&self, nonce: &Nonce) -> bool {
        self.reserve_calls.fetch_add(1, Ordering::SeqCst);
        let mut states = self.states.lock().expect("ephemeral nonce mutex poisoned");
        if states.contains_key(nonce) { return false; }
        states.insert(*nonce, EphemeralNonceState::Reserved);
        true
    }
    async fn commit(&self, nonce: &Nonce) {
        self.commit_calls.fetch_add(1, Ordering::SeqCst);
        let mut states = self.states.lock().expect("ephemeral nonce mutex poisoned");
        if states.get(nonce) == Some(&EphemeralNonceState::Reserved) {
            states.insert(*nonce, EphemeralNonceState::Committed);
        }
    }
    async fn release(&self, nonce: &Nonce) {
        self.release_calls.fetch_add(1, Ordering::SeqCst);
        let mut states = self.states.lock().expect("ephemeral nonce mutex poisoned");
        if states.get(nonce) == Some(&EphemeralNonceState::Reserved) { states.remove(nonce); }
    }
}

#[derive(Debug, Default)]
pub struct ExternalOriginKeyStoreV1 { keys: Mutex<HashMap<u8, Vec<PublicKey>>> }

impl ExternalOriginKeyStoreV1 {
    pub fn insert_under_truncated_id(&self, truncated_id: u8, public_key: PublicKey) {
        self.keys.lock().expect("origin key mutex poisoned").entry(truncated_id).or_default().push(public_key);
    }
}

#[async_trait]
impl OriginKeyStore for ExternalOriginKeyStoreV1 {
    async fn insert(&self, truncated_token_key_id: u8, public_key: PublicKey) {
        self.insert_under_truncated_id(truncated_token_key_id, public_key);
    }
    async fn get(&self, truncated_token_key_id: &u8) -> Vec<PublicKey> {
        self.keys.lock().expect("origin key mutex poisoned").get(truncated_token_key_id).cloned().unwrap_or_default()
    }
    async fn remove(&self, truncated_token_key_id: &u8) -> bool {
        self.keys.lock().expect("origin key mutex poisoned").remove(truncated_token_key_id).is_some()
    }
}

pub async fn verify_public_token_via_pinned_backend_v1(
    public_key_spki_der: &[u8],
    exact_token_bytes: &[u8],
    expected_challenge_digest_hex: Option<&str>,
) -> Result<PublicTokenBackendPreflightReceiptV1, PreflightError> {
    let public_key = PublicKey::from_spki(public_key_spki_der).map_err(|_| PreflightError::InvalidPublicKey)?;
    let canonical_spki = serialize_public_key(&public_key).map_err(|_| PreflightError::InvalidPublicKey)?;
    if canonical_spki.as_slice() != public_key_spki_der {
        return Err(PreflightError::PublicKeySpkiRoundTripMismatch);
    }

    let mut reader = exact_token_bytes;
    let token = PublicToken::tls_deserialize(&mut reader).map_err(|_| PreflightError::InvalidTokenEncoding)?;
    if !reader.is_empty() { return Err(PreflightError::TrailingTokenBytes); }
    if token.token_type() != TokenType::Public { return Err(PreflightError::WrongTokenType); }

    let public_key_spki_sha256 = sha256_hex(&canonical_spki);
    let token_key_id_sha256 = hex_bytes(token.token_key_id());
    if token_key_id_sha256 != public_key_spki_sha256 { return Err(PreflightError::TokenKeyIdMismatch); }

    let token_challenge_digest_hex = hex_bytes(token.challenge_digest());
    if let Some(expected) = expected_challenge_digest_hex {
        if !is_sha256_hex(expected) || token_challenge_digest_hex != expected.to_ascii_lowercase() {
            return Err(PreflightError::ChallengeDigestMismatch);
        }
    }

    let nonce = token.nonce();
    let token_sha256 = sha256_hex(exact_token_bytes);
    let token_nonce_sha256 = sha256_hex(&nonce);

    let origin_keys = ExternalOriginKeyStoreV1::default();
    origin_keys.insert(
        public_key_to_truncated_token_key_id(&public_key).map_err(|_| PreflightError::InvalidPublicKey)?,
        public_key,
    ).await;

    let ephemeral_nonce_store = ExternalEphemeralNonceStoreV1::default();
    OriginServer::new().redeem_token(&origin_keys, &ephemeral_nonce_store, token).await
        .map_err(|error| PreflightError::BackendVerificationFailed(error.to_string()))?;

    if ephemeral_nonce_store.state(&nonce) != Some(EphemeralNonceState::Committed) {
        return Err(PreflightError::EphemeralNonceStateMismatch);
    }
    let (reserve_calls, commit_calls, release_calls) = ephemeral_nonce_store.counts();
    if (reserve_calls, commit_calls, release_calls) != (1, 1, 0) {
        return Err(PreflightError::EphemeralNonceStateMismatch);
    }

    Ok(PublicTokenBackendPreflightReceiptV1 {
        backend_repository: BACKEND_REPOSITORY.to_owned(),
        backend_commit: BACKEND_COMMIT.to_owned(),
        backend_package: BACKEND_PACKAGE.to_owned(),
        backend_version: BACKEND_VERSION.to_owned(),
        backend_profile: BACKEND_PROFILE.to_owned(),
        backend_audit_status: BACKEND_AUDIT_STATUS.to_owned(),
        token_type: 0x0002,
        token_sha256,
        token_nonce_sha256,
        token_challenge_digest_hex,
        token_key_id_sha256,
        public_key_spki_sha256,
        ephemeral_reserve_calls: reserve_calls,
        ephemeral_commit_calls: commit_calls,
        ephemeral_release_calls: release_calls,
    })
}

fn decode_hex(value: &str) -> Result<Vec<u8>, PreflightError> {
    if value.len() % 2 != 0 { return Err(PreflightError::InvalidHex); }
    let bytes = value.as_bytes();
    let mut out = Vec::with_capacity(value.len() / 2);
    for index in (0..bytes.len()).step_by(2) {
        let high = hex_nibble(bytes[index])?;
        let low = hex_nibble(bytes[index + 1])?;
        out.push((high << 4) | low);
    }
    Ok(out)
}

fn decode_hex_32(value: &str) -> Result<[u8; 32], PreflightError> {
    decode_hex(value)?.try_into().map_err(|_| PreflightError::InvalidHex)
}

fn hex_nibble(value: u8) -> Result<u8, PreflightError> {
    match value {
        b'0'..=b'9' => Ok(value - b'0'),
        b'a'..=b'f' => Ok(value - b'a' + 10),
        b'A'..=b'F' => Ok(value - b'A' + 10),
        _ => Err(PreflightError::InvalidHex),
    }
}

fn sha256_hex(bytes: &[u8]) -> String { hex_bytes(&Sha256::digest(bytes)) }

fn hex_bytes(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn is_sha256_hex(value: &str) -> bool { value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit()) }

#[cfg(test)]
mod tests {
    use super::*;
    use futures::executor::block_on;
    use psi_privacy_pass_credit_core::{QueryCreditPolicyV1, ReplayPolicyV1, Rfc9578TokenType};
    use psi_privacy_pass_rfc9577_challenge::encode_b3a_token_challenge_v1;
    use privacypass::auth::authenticate::TokenChallenge;

    #[derive(Debug, serde::Deserialize)]
    struct Fixture {
        source_repository: String,
        source_commit: String,
        source_path: String,
        source_vector_index: usize,
        public_key_spki_der_hex: String,
        token_hex: String,
        expected_spki_sha256: String,
        expected_token_sha256: String,
        expected_nonce_sha256: String,
        expected_challenge_digest_hex: String,
        expected_token_key_id_hex: String,
    }

    fn fixture() -> Fixture { serde_json::from_str(include_str!("../fixtures/public_go_vector_0.json")).unwrap() }

    fn policy() -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(), issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32), token_type: Rfc9578TokenType::PublicBlindRsa2048Sha384,
            token_key_id_sha256: "22".repeat(32), budget_epoch: "epoch-a".into(), max_identifiers_per_credit: 256,
            replay_policy: ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    fn verify_fixture() -> Result<PublicTokenBackendPreflightReceiptV1, PreflightError> {
        let fixture = fixture();
        block_on(verify_public_token_via_pinned_backend_v1(
            &decode_hex(&fixture.public_key_spki_der_hex)?, &decode_hex(&fixture.token_hex)?,
            Some(&fixture.expected_challenge_digest_hex),
        ))
    }

    #[test]
    fn cross_implementation_go_kat_verifies_with_external_stores() {
        let fixture = fixture();
        assert_eq!(fixture.source_repository, BACKEND_REPOSITORY);
        assert_eq!(fixture.source_commit, BACKEND_COMMIT);
        assert_eq!(fixture.source_path, GO_KAT_SOURCE_PATH);
        assert_eq!(fixture.source_vector_index, GO_KAT_VECTOR_INDEX);
        let receipt = verify_fixture().unwrap();
        assert!(receipt.external_origin_key_store_usable());
        assert!(receipt.external_ephemeral_nonce_store_usable());
        assert!(receipt.exact_spki_round_trip_established());
        assert!(receipt.pinned_backend_crypto_path_executed());
        assert!(receipt.token_fields_bound_to_backend_verified_artifact());
    }

    #[test]
    fn cross_implementation_vector_has_frozen_field_identities() {
        let fixture = fixture();
        let receipt = verify_fixture().unwrap();
        assert_eq!(receipt.backend_commit(), BACKEND_COMMIT);
        assert_eq!(receipt.token_sha256(), fixture.expected_token_sha256);
        assert_eq!(receipt.token_nonce_sha256(), fixture.expected_nonce_sha256);
        assert_eq!(receipt.token_challenge_digest_hex(), fixture.expected_challenge_digest_hex);
        assert_eq!(receipt.token_key_id_sha256(), fixture.expected_token_key_id_hex);
        assert_eq!(fixture.expected_spki_sha256, fixture.expected_token_key_id_hex);
    }

    #[test]
    fn same_valid_token_verifies_again_with_fresh_ephemeral_store() {
        let first = verify_fixture().unwrap();
        let second = verify_fixture().unwrap();
        assert_eq!(first.token_sha256(), second.token_sha256());
        assert!(!first.service_wide_unspent_established());
        assert!(!second.service_wide_unspent_established());
    }

    #[test]
    fn wrong_signature_fails_closed() {
        let fixture = fixture();
        let spki = decode_hex(&fixture.public_key_spki_der_hex).unwrap();
        let mut token = decode_hex(&fixture.token_hex).unwrap();
        *token.last_mut().unwrap() ^= 0x01;
        let result = block_on(verify_public_token_via_pinned_backend_v1(&spki, &token, Some(&fixture.expected_challenge_digest_hex)));
        assert!(matches!(result, Err(PreflightError::BackendVerificationFailed(_))));
    }

    #[test]
    fn wrong_token_type_fails_before_backend_redemption() {
        let fixture = fixture();
        let spki = decode_hex(&fixture.public_key_spki_der_hex).unwrap();
        let mut token = decode_hex(&fixture.token_hex).unwrap();
        token[0] = 0x00; token[1] = 0x01;
        let result = block_on(verify_public_token_via_pinned_backend_v1(&spki, &token, None));
        assert_eq!(result, Err(PreflightError::WrongTokenType));
    }

    #[test]
    fn wrong_expected_challenge_digest_fails_before_backend_redemption() {
        let fixture = fixture();
        let result = block_on(verify_public_token_via_pinned_backend_v1(
            &decode_hex(&fixture.public_key_spki_der_hex).unwrap(), &decode_hex(&fixture.token_hex).unwrap(), Some(&"00".repeat(32)),
        ));
        assert_eq!(result, Err(PreflightError::ChallengeDigestMismatch));
    }

    #[test]
    fn trailing_token_bytes_are_rejected() {
        let fixture = fixture();
        let mut token = decode_hex(&fixture.token_hex).unwrap(); token.push(0);
        let result = block_on(verify_public_token_via_pinned_backend_v1(&decode_hex(&fixture.public_key_spki_der_hex).unwrap(), &token, None));
        assert_eq!(result, Err(PreflightError::TrailingTokenBytes));
    }

    #[test]
    fn token_key_id_is_full_sha256_of_exact_spki() {
        let fixture = fixture();
        let spki = decode_hex(&fixture.public_key_spki_der_hex).unwrap();
        assert_eq!(sha256_hex(&spki), fixture.expected_spki_sha256);
        assert_eq!(fixture.expected_spki_sha256, fixture.expected_token_key_id_hex);
    }

    #[test]
    fn pinned_backend_token_challenge_matches_independent_b3a1_encoder() {
        let policy = policy();
        let semantic = policy.challenge_binding().unwrap();
        let independent = encode_b3a_token_challenge_v1(&policy, &semantic).unwrap();
        let context = decode_hex_32(&semantic.redemption_context_sha256).unwrap();
        let backend = TokenChallenge::new(TokenType::Public, &policy.issuer_name, Some(context), &[policy.service_domain.clone()]);
        assert_eq!(backend.serialize().unwrap(), independent.wire_bytes());
        assert_eq!(hex_bytes(&backend.digest().unwrap()), independent.challenge_sha256());
    }

    #[test]
    fn receipt_preserves_strict_preflight_authority_ceiling() {
        let receipt = verify_fixture().unwrap();
        assert!(!receipt.backend_independently_audited());
        assert!(!receipt.issuer_key_trusted_under_mycelix_policy());
        assert!(!receipt.issuer_key_current_under_mycelix_policy());
        assert!(!receipt.service_wide_unspent_established());
        assert!(!receipt.durable_replay_protection_established());
        assert!(!receipt.query_credit_granted());
        assert!(!receipt.anonymous_rate_limit_established());
        assert!(!receipt.enumeration_resistance_established());
        assert!(!receipt.application_authority_granted());
    }
}
