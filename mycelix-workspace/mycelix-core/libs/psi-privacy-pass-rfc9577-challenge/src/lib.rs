// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B3A1 — exact RFC 9577 TokenChallenge wire/digest binding.

#![forbid(unsafe_code)]

use psi_privacy_pass_credit_core::{
    PrivacyPassChallengeBindingV1, QueryCreditPolicyV1, QueryCreditStructuralFailure,
    Rfc9578TokenType,
};
use serde::Serialize;
use sha2::{Digest, Sha256};

pub const RFC9577_DEFAULT_CHALLENGE_PROFILE_V1: &str =
    "rfc9577-default-token-challenge-single-origin-context32-v1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Rfc9577ChallengeError {
    Structural(QueryCreditStructuralFailure),
    SemanticChallengeMismatch,
    InvalidIssuerName,
    InvalidOriginName,
    InvalidRedemptionContextLength,
    InvalidRedemptionContextHex,
    LengthOverflow,
    WrongOriginCount,
}

impl From<QueryCreditStructuralFailure> for Rfc9577ChallengeError {
    fn from(value: QueryCreditStructuralFailure) -> Self {
        Self::Structural(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Rfc9577EncodedTokenChallengeV1 {
    profile: String,
    policy_sha256: String,
    semantic_challenge_sha256: String,
    token_type: Rfc9578TokenType,
    issuer_name: String,
    redemption_context_hex: String,
    origin_info: String,
    wire_bytes: Vec<u8>,
    challenge_sha256: String,
}

impl Rfc9577EncodedTokenChallengeV1 {
    pub fn profile(&self) -> &str {
        &self.profile
    }

    pub fn policy_sha256(&self) -> &str {
        &self.policy_sha256
    }

    pub fn semantic_challenge_sha256(&self) -> &str {
        &self.semantic_challenge_sha256
    }

    pub fn wire_bytes(&self) -> &[u8] {
        &self.wire_bytes
    }

    pub fn challenge_sha256(&self) -> &str {
        &self.challenge_sha256
    }

    pub const fn exact_rfc9577_default_challenge_encoding_established(&self) -> bool {
        true
    }

    pub const fn token_challenge_digest_cryptographically_bound(&self) -> bool {
        false
    }

    pub const fn token_cryptographically_verified(&self) -> bool {
        false
    }

    pub const fn token_nonce_cryptographically_bound(&self) -> bool {
        false
    }

    pub const fn atomic_single_use_established(&self) -> bool {
        false
    }

    pub const fn query_credit_granted(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

/// Encode the RFC 9577 default `TokenChallenge` wire structure.
///
/// This function implements only the default challenge syntax. The caller is
/// responsible for any token-type-specific semantic requirements beyond the
/// conservative name/length validation performed here.
pub fn encode_default_token_challenge_v1(
    token_type: Rfc9578TokenType,
    issuer_name: &str,
    redemption_context: &[u8],
    origin_info: &str,
) -> Result<Vec<u8>, Rfc9577ChallengeError> {
    validate_server_name(issuer_name).map_err(|_| Rfc9577ChallengeError::InvalidIssuerName)?;
    if !origin_info.is_empty() {
        validate_server_name(origin_info).map_err(|_| Rfc9577ChallengeError::InvalidOriginName)?;
    }
    if !matches!(redemption_context.len(), 0 | 32) {
        return Err(Rfc9577ChallengeError::InvalidRedemptionContextLength);
    }

    let issuer_len = u16::try_from(issuer_name.len())
        .map_err(|_| Rfc9577ChallengeError::LengthOverflow)?;
    if issuer_len == 0 {
        return Err(Rfc9577ChallengeError::InvalidIssuerName);
    }
    let origin_len = u16::try_from(origin_info.len())
        .map_err(|_| Rfc9577ChallengeError::LengthOverflow)?;
    let context_len = u8::try_from(redemption_context.len())
        .map_err(|_| Rfc9577ChallengeError::InvalidRedemptionContextLength)?;

    let mut out = Vec::with_capacity(
        2 + 2 + issuer_name.len() + 1 + redemption_context.len() + 2 + origin_info.len(),
    );
    out.extend_from_slice(&token_type.code().to_be_bytes());
    out.extend_from_slice(&issuer_len.to_be_bytes());
    out.extend_from_slice(issuer_name.as_bytes());
    out.push(context_len);
    out.extend_from_slice(redemption_context);
    out.extend_from_slice(&origin_len.to_be_bytes());
    out.extend_from_slice(origin_info.as_bytes());
    Ok(out)
}

/// Re-derive the exact B3A semantic challenge and encode it as RFC 9577 bytes.
pub fn encode_b3a_token_challenge_v1(
    policy: &QueryCreditPolicyV1,
    challenge: &PrivacyPassChallengeBindingV1,
) -> Result<Rfc9577EncodedTokenChallengeV1, Rfc9577ChallengeError> {
    policy.validate()?;
    challenge.validate()?;

    let expected = policy.challenge_binding()?;
    if challenge != &expected {
        return Err(Rfc9577ChallengeError::SemanticChallengeMismatch);
    }
    if challenge.origin_info.len() != 1 {
        return Err(Rfc9577ChallengeError::WrongOriginCount);
    }

    let redemption_context = decode_hex_32(&challenge.redemption_context_sha256)?;
    let origin = &challenge.origin_info[0];
    let wire = encode_default_token_challenge_v1(
        challenge.token_type,
        &challenge.issuer_name,
        &redemption_context,
        origin,
    )?;

    Ok(Rfc9577EncodedTokenChallengeV1 {
        profile: RFC9577_DEFAULT_CHALLENGE_PROFILE_V1.to_owned(),
        policy_sha256: policy.commitment_sha256()?,
        semantic_challenge_sha256: challenge.commitment_sha256()?,
        token_type: challenge.token_type,
        issuer_name: challenge.issuer_name.clone(),
        redemption_context_hex: challenge.redemption_context_sha256.clone(),
        origin_info: origin.clone(),
        challenge_sha256: sha256_hex(&wire),
        wire_bytes: wire,
    })
}

fn validate_server_name(value: &str) -> Result<(), ()> {
    if value.is_empty() || !value.is_ascii() || value.len() > u16::MAX as usize {
        return Err(());
    }
    if !value.bytes().all(|byte| {
        byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'-' | b':' | b'[' | b']')
    }) {
        return Err(());
    }
    Ok(())
}

fn decode_hex_32(value: &str) -> Result<[u8; 32], Rfc9577ChallengeError> {
    if value.len() != 64 {
        return Err(Rfc9577ChallengeError::InvalidRedemptionContextHex);
    }
    let mut output = [0u8; 32];
    let bytes = value.as_bytes();
    for index in 0..32 {
        let high = hex_nibble(bytes[index * 2])?;
        let low = hex_nibble(bytes[index * 2 + 1])?;
        output[index] = (high << 4) | low;
    }
    Ok(output)
}

fn hex_nibble(value: u8) -> Result<u8, Rfc9577ChallengeError> {
    match value {
        b'0'..=b'9' => Ok(value - b'0'),
        b'a'..=b'f' => Ok(value - b'a' + 10),
        b'A'..=b'F' => Ok(value - b'A' + 10),
        _ => Err(Rfc9577ChallengeError::InvalidRedemptionContextHex),
    }
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

    fn policy() -> QueryCreditPolicyV1 {
        QueryCreditPolicyV1 {
            service_domain: "contacts.mycelix.test".into(),
            issuer_name: "issuer.mycelix.test".into(),
            issuer_configuration_sha256: "11".repeat(32),
            token_type: Rfc9578TokenType::PublicBlindRsa2048Sha384,
            token_key_id_sha256: "22".repeat(32),
            budget_epoch: "epoch-a".into(),
            max_identifiers_per_credit: 256,
            replay_policy: ReplayPolicyV1::AtomicSingleUseRequired,
        }
    }

    #[test]
    fn rfc9577_appendix_a_vector_1_matches_exact_bytes_and_digest() {
        let context = decode_hex_32(
            "476ac2c935f458e9b2d7af32dacfbd22dd6023ef5887a789f1abe004e79bb5bb",
        )
        .unwrap();
        let wire = encode_default_token_challenge_v1(
            Rfc9578TokenType::PublicBlindRsa2048Sha384,
            "issuer.example",
            &context,
            "origin.example",
        )
        .unwrap();
        assert_eq!(wire.len(), 67);
        assert_eq!(
            hex_bytes(&wire),
            "0002000e6973737565722e6578616d706c6520476ac2c935f458e9b2d7af32dacfbd22dd6023ef5887a789f1abe004e79bb5bb000e6f726967696e2e6578616d706c65"
        );
        assert_eq!(
            sha256_hex(&wire),
            "8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f"
        );
    }

    #[test]
    fn exact_b3a_policy_encodes_deterministically() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let a = encode_b3a_token_challenge_v1(&policy, &challenge).unwrap();
        let b = encode_b3a_token_challenge_v1(&policy, &challenge).unwrap();
        assert_eq!(a.wire_bytes(), b.wire_bytes());
        assert_eq!(a.challenge_sha256(), b.challenge_sha256());
    }

    #[test]
    fn service_epoch_and_budget_change_challenge_digest() {
        let base = policy();
        let base_encoded = encode_b3a_token_challenge_v1(&base, &base.challenge_binding().unwrap())
            .unwrap();

        let mut service = base.clone();
        service.service_domain = "other.mycelix.test".into();
        let service_encoded =
            encode_b3a_token_challenge_v1(&service, &service.challenge_binding().unwrap()).unwrap();
        assert_ne!(base_encoded.challenge_sha256(), service_encoded.challenge_sha256());

        let mut epoch = base.clone();
        epoch.budget_epoch = "epoch-b".into();
        let epoch_encoded =
            encode_b3a_token_challenge_v1(&epoch, &epoch.challenge_binding().unwrap()).unwrap();
        assert_ne!(base_encoded.challenge_sha256(), epoch_encoded.challenge_sha256());

        let mut budget = base.clone();
        budget.max_identifiers_per_credit = 64;
        let budget_encoded =
            encode_b3a_token_challenge_v1(&budget, &budget.challenge_binding().unwrap()).unwrap();
        assert_ne!(base_encoded.challenge_sha256(), budget_encoded.challenge_sha256());
    }

    #[test]
    fn borrowed_semantic_challenge_is_rejected() {
        let original = policy();
        let challenge = original.challenge_binding().unwrap();
        let mut changed = original;
        changed.max_identifiers_per_credit = 64;
        assert_eq!(
            encode_b3a_token_challenge_v1(&changed, &challenge),
            Err(Rfc9577ChallengeError::SemanticChallengeMismatch)
        );
    }

    #[test]
    fn malformed_redemption_context_hex_is_rejected() {
        assert_eq!(
            decode_hex_32(&"zz".repeat(32)),
            Err(Rfc9577ChallengeError::InvalidRedemptionContextHex)
        );
        assert_eq!(
            decode_hex_32("00"),
            Err(Rfc9577ChallengeError::InvalidRedemptionContextHex)
        );
    }

    #[test]
    fn ambiguous_or_unsafe_server_names_are_rejected() {
        let context = [0u8; 32];
        for bad in [
            "issuer@example.com",
            "issuer.example/path",
            "issuer.example?q=1",
            "issuer.example#frag",
            "issuer.example,other.example",
            "issuer example",
        ] {
            assert!(matches!(
                encode_default_token_challenge_v1(
                    Rfc9578TokenType::PublicBlindRsa2048Sha384,
                    bad,
                    &context,
                    "origin.example"
                ),
                Err(Rfc9577ChallengeError::InvalidIssuerName)
            ));
            assert!(matches!(
                encode_default_token_challenge_v1(
                    Rfc9578TokenType::PublicBlindRsa2048Sha384,
                    "issuer.example",
                    &context,
                    bad
                ),
                Err(Rfc9577ChallengeError::InvalidOriginName)
            ));
        }
    }

    #[test]
    fn overlong_server_name_is_rejected() {
        let overlong = "a".repeat(u16::MAX as usize + 1);
        assert_eq!(
            encode_default_token_challenge_v1(
                Rfc9578TokenType::PublicBlindRsa2048Sha384,
                &overlong,
                &[0u8; 32],
                "origin.example"
            ),
            Err(Rfc9577ChallengeError::InvalidIssuerName)
        );
    }

    #[test]
    fn invalid_redemption_context_lengths_are_rejected() {
        assert_eq!(
            encode_default_token_challenge_v1(
                Rfc9578TokenType::PublicBlindRsa2048Sha384,
                "issuer.example",
                &[0u8; 31],
                "origin.example"
            ),
            Err(Rfc9577ChallengeError::InvalidRedemptionContextLength)
        );
    }

    #[test]
    fn encoded_positive_preserves_authority_ceiling() {
        let policy = policy();
        let challenge = policy.challenge_binding().unwrap();
        let encoded = encode_b3a_token_challenge_v1(&policy, &challenge).unwrap();
        assert!(encoded.exact_rfc9577_default_challenge_encoding_established());
        assert!(!encoded.token_challenge_digest_cryptographically_bound());
        assert!(!encoded.token_cryptographically_verified());
        assert!(!encoded.token_nonce_cryptographically_bound());
        assert!(!encoded.atomic_single_use_established());
        assert!(!encoded.query_credit_granted());
        assert!(!encoded.application_authority_granted());
    }

    fn hex_bytes(bytes: &[u8]) -> String {
        let mut out = String::with_capacity(bytes.len() * 2);
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for byte in bytes {
            out.push(HEX[(byte >> 4) as usize] as char);
            out.push(HEX[(byte & 0x0f) as usize] as char);
        }
        out
    }
}
