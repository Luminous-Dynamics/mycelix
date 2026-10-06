//! Cryptographic verification of the FPM EAT-over-COSE profile.
//!
//! This module verifies the exact received COSE_Sign1 bytes before interpreting
//! the payload as an EAT/CWT claim set. It intentionally separates:
//! 1. cryptographic signature validity;
//! 2. EAT claim decoding;
//! 3. FPM challenge/subject/audience/profile binding.
//!
//! It does not establish that a key belongs to secure hardware, nor does it
//! establish physical sensor truth. Trust-root/key provisioning remains an
//! external verifier-policy responsibility.

use coset::{iana, CborSerializable, CoseSign1, ContentType, Header};
use p256::ecdsa::{signature::Verifier, Signature, VerifyingKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

const SHA256_HEX_LEN: usize = 64;
const MAX_EAT_TOKEN_BYTES: usize = 64 * 1024;
const ES256_SIGNATURE_BYTES: usize = 64;
const P256_SEC1_UNCOMPRESSED_BYTES: usize = 65;
const CLAIM_SUB: i64 = 2;
const CLAIM_AUD: i64 = 3;
const CLAIM_NONCE: i64 = 10;
const CLAIM_EAT_PROFILE: i64 = 265;

pub const FPM_EAT_MEDIA_TYPE: &str = "application/eat+cwt";
pub const FPM_EAT_PROFILE_URI: &str =
    "tag:luminousdynamics.org,2026:fpm-source-attestation-v1";
pub const FPM_EAT_COSE_ALGORITHM: &str = "ES256";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmEatCoseVerificationInput {
    pub expected_subject_id: String,
    pub expected_audience: String,
    pub expected_nonce: Vec<u8>,
    pub expected_key_id: Vec<u8>,
    pub trusted_public_key_sec1: Vec<u8>,
    pub expected_evidence_digest: Option<String>,
    pub token_bytes: Vec<u8>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum FpmEatCoseVerificationStatus {
    QualifiedForProfile,
    InvalidEvidence,
    ConflictingEvidence,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum FpmEatCoseVerificationReason {
    TokenTooLarge,
    MalformedCoseSign1,
    DetachedPayload,
    UnprotectedHeadersPresent,
    UnsupportedProtectedHeader,
    MissingProtectedAlgorithm,
    UnsupportedAlgorithm,
    MissingProtectedKeyId,
    KeyIdMismatch,
    InvalidVerificationKey,
    InvalidSignatureEncoding,
    SignatureInvalid,
    MalformedEatPayload,
    DuplicateClaim,
    UnsupportedClaimType,
    MissingSubject,
    SubjectMismatch,
    MissingAudience,
    AudienceMismatch,
    MissingNonce,
    NonceMismatch,
    MissingEatProfile,
    EatProfileMismatch,
    ContentTypeMismatch,
    EvidenceDigestMismatch,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmVerifiedEatCoseEvidence {
    pub status: FpmEatCoseVerificationStatus,
    pub reasons: Vec<FpmEatCoseVerificationReason>,
    pub evidence_digest: String,
    pub payload_digest: Option<String>,
    pub subject_id: Option<String>,
    pub audience: Option<String>,
    pub nonce: Option<Vec<u8>>,
    pub eat_profile_uri: Option<String>,
    pub key_id: Option<Vec<u8>>,
    pub verification_key_digest: String,
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn canonical_digest(value: &str) -> bool {
    value.len() == SHA256_HEX_LEN
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn valid_label(value: &str, max_len: usize) -> bool {
    !value.trim().is_empty()
        && value == value.trim()
        && value.len() <= max_len
        && !value.chars().any(char::is_control)
}

fn value_for_claim<'a>(
    map: &'a [(coset::cbor::Value, coset::cbor::Value)],
    key: i64,
    seen: &mut BTreeSet<String>,
) -> Result<Option<&'a coset::cbor::Value>, FpmEatCoseVerificationReason> {
    let mut found = None;
    for (claim_key, value) in map {
        let matches = *claim_key == coset::cbor::Value::Integer(key.into());
        if matches {
            let marker = key.to_string();
            if !seen.insert(marker) {
                return Err(FpmEatCoseVerificationReason::DuplicateClaim);
            }
            found = Some(value);
        }
    }
    Ok(found)
}

fn required_text_claim(
    map: &[(coset::cbor::Value, coset::cbor::Value)],
    key: i64,
    missing: FpmEatCoseVerificationReason,
    seen: &mut BTreeSet<String>,
) -> Result<String, FpmEatCoseVerificationReason> {
    let Some(value) = value_for_claim(map, key, seen)? else {
        return Err(missing);
    };
    match value {
        coset::cbor::Value::Text(text) => Ok(text.clone()),
        _ => Err(FpmEatCoseVerificationReason::UnsupportedClaimType),
    }
}

pub fn verify_fpm_eat_cose_sign1(
    input: &FpmEatCoseVerificationInput,
) -> FpmVerifiedEatCoseEvidence {
    let evidence_digest = hex_digest(&input.token_bytes);
    let verification_key_digest = hex_digest(&input.trusted_public_key_sec1);
    let mut reasons = BTreeSet::new();

    if input.token_bytes.len() > MAX_EAT_TOKEN_BYTES {
        reasons.insert(FpmEatCoseVerificationReason::TokenTooLarge);
    }
    if !valid_label(&input.expected_subject_id, 128)
        || !valid_label(&input.expected_audience, 256)
        || input.expected_nonce.len() < 8
        || input.expected_nonce.len() > 64
    {
        reasons.insert(FpmEatCoseVerificationReason::UnsupportedClaimType);
    }
    if !canonical_digest(&evidence_digest) {
        reasons.insert(FpmEatCoseVerificationReason::MalformedCoseSign1);
    }
    if let Some(expected) = &input.expected_evidence_digest {
        if !canonical_digest(expected) || expected != &evidence_digest {
            reasons.insert(FpmEatCoseVerificationReason::EvidenceDigestMismatch);
        }
    }
    if input.expected_key_id.is_empty() || input.expected_key_id.len() > 128 {
        reasons.insert(FpmEatCoseVerificationReason::KeyIdMismatch);
    }
    if input.trusted_public_key_sec1.len() != P256_SEC1_UNCOMPRESSED_BYTES
        || input.trusted_public_key_sec1.first() != Some(&0x04)
    {
        reasons.insert(FpmEatCoseVerificationReason::InvalidVerificationKey);
    }

    if !reasons.is_empty() {
        return FpmVerifiedEatCoseEvidence {
            status: FpmEatCoseVerificationStatus::InvalidEvidence,
            reasons: reasons.into_iter().collect(),
            evidence_digest,
            payload_digest: None,
            subject_id: None,
            audience: None,
            nonce: None,
            eat_profile_uri: None,
            key_id: None,
            verification_key_digest,
        };
    }

    let mut sign1 = match CoseSign1::from_tagged_slice(&input.token_bytes) {
        Ok(value) => value,
        Err(_) => {
            return FpmVerifiedEatCoseEvidence {
                status: FpmEatCoseVerificationStatus::InvalidEvidence,
                reasons: vec![FpmEatCoseVerificationReason::MalformedCoseSign1],
                evidence_digest,
                payload_digest: None,
                subject_id: None,
                audience: None,
                nonce: None,
                eat_profile_uri: None,
                key_id: None,
                verification_key_digest,
            };
        }
    };

    if !sign1.unprotected.is_empty() {
        reasons.insert(FpmEatCoseVerificationReason::UnprotectedHeadersPresent);
    }

    let Some(algorithm) = sign1.protected.header.alg.as_ref() else {
        reasons.insert(FpmEatCoseVerificationReason::MissingProtectedAlgorithm);
        return invalid_result(
            evidence_digest,
            verification_key_digest,
            reasons,
            sign1.protected.header.key_id.clone(),
        );
    };
    if *algorithm != coset::Algorithm::Assigned(iana::Algorithm::ES256) {
        reasons.insert(FpmEatCoseVerificationReason::UnsupportedAlgorithm);
    }

    let key_id = if sign1.protected.header.key_id.is_empty() {
        reasons.insert(FpmEatCoseVerificationReason::MissingProtectedKeyId);
        None
    } else {
        Some(sign1.protected.header.key_id.clone())
    };
    if key_id.as_deref() != Some(input.expected_key_id.as_slice()) {
        reasons.insert(FpmEatCoseVerificationReason::KeyIdMismatch);
    }

    if sign1.protected.header.content_type
        != Some(ContentType::Text(FPM_EAT_MEDIA_TYPE.to_string()))
    {
        reasons.insert(FpmEatCoseVerificationReason::ContentTypeMismatch);
    }

    if sign1.protected.header.crit.len() != 0
        || !sign1.protected.header.rest.is_empty()
        || !sign1.protected.header.iv.is_empty()
        || !sign1.protected.header.partial_iv.is_empty()
        || !sign1.protected.header.counter_signatures.is_empty()
    {
        reasons.insert(FpmEatCoseVerificationReason::UnsupportedProtectedHeader);
    }

    if !reasons.is_empty() {
        return invalid_result(evidence_digest, verification_key_digest, reasons, key_id);
    }

    let Some(payload) = sign1.payload.clone() else {
        reasons.insert(FpmEatCoseVerificationReason::DetachedPayload);
        return invalid_result(evidence_digest, verification_key_digest, reasons, key_id);
    };

    let verifying_key = match VerifyingKey::from_sec1_bytes(&input.trusted_public_key_sec1) {
        Ok(key) => key,
        Err(_) => {
            reasons.insert(FpmEatCoseVerificationReason::InvalidVerificationKey);
            return invalid_result(evidence_digest, verification_key_digest, reasons, key_id);
        }
    };

    if sign1
        .verify_signature(&[], |signature, data| {
            let signature = Signature::from_slice(signature)
                .map_err(|_| FpmEatCoseVerificationReason::InvalidSignatureEncoding)?;
            verifying_key
                .verify(data, &signature)
                .map_err(|_| FpmEatCoseVerificationReason::SignatureInvalid)
        })
        .is_err()
    {
        reasons.insert(FpmEatCoseVerificationReason::SignatureInvalid);
        return invalid_result(evidence_digest, verification_key_digest, reasons, key_id);
    }

    let payload_digest = hex_digest(&payload);
    let claims = match coset::cbor::Value::from_slice(&payload) {
        Ok(coset::cbor::Value::Map(map)) => map,
        Ok(_) | Err(_) => {
            reasons.insert(FpmEatCoseVerificationReason::MalformedEatPayload);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
    };

    let mut seen = BTreeSet::new();
    let subject_id = match required_text_claim(
        &claims,
        CLAIM_SUB,
        FpmEatCoseVerificationReason::MissingSubject,
        &mut seen,
    ) {
        Ok(value) => value,
        Err(reason) => {
            reasons.insert(reason);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
    };

    let audience = match required_text_claim(
        &claims,
        CLAIM_AUD,
        FpmEatCoseVerificationReason::MissingAudience,
        &mut seen,
    ) {
        Ok(value) => value,
        Err(reason) => {
            reasons.insert(reason);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
    };

    let nonce = match value_for_claim(&claims, CLAIM_NONCE, &mut seen) {
        Ok(Some(coset::cbor::Value::Bytes(value))) => value.clone(),
        Ok(Some(_)) => {
            reasons.insert(FpmEatCoseVerificationReason::UnsupportedClaimType);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
        Ok(None) => {
            reasons.insert(FpmEatCoseVerificationReason::MissingNonce);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
        Err(reason) => {
            reasons.insert(reason);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
    };

    let eat_profile_uri = match required_text_claim(
        &claims,
        CLAIM_EAT_PROFILE,
        FpmEatCoseVerificationReason::MissingEatProfile,
        &mut seen,
    ) {
        Ok(value) => value,
        Err(reason) => {
            reasons.insert(reason);
            return invalid_result_with_payload(
                evidence_digest,
                verification_key_digest,
                reasons,
                key_id,
                payload_digest,
            );
        }
    };

    if subject_id != input.expected_subject_id {
        reasons.insert(FpmEatCoseVerificationReason::SubjectMismatch);
    }
    if audience != input.expected_audience {
        reasons.insert(FpmEatCoseVerificationReason::AudienceMismatch);
    }
    if nonce != input.expected_nonce {
        reasons.insert(FpmEatCoseVerificationReason::NonceMismatch);
    }
    if eat_profile_uri != FPM_EAT_PROFILE_URI {
        reasons.insert(FpmEatCoseVerificationReason::EatProfileMismatch);
    }

    let status = if reasons.is_empty() {
        FpmEatCoseVerificationStatus::QualifiedForProfile
    } else if reasons.iter().any(|reason| {
        matches!(
            reason,
            FpmEatCoseVerificationReason::SubjectMismatch
                | FpmEatCoseVerificationReason::AudienceMismatch
                | FpmEatCoseVerificationReason::NonceMismatch
                | FpmEatCoseVerificationReason::EatProfileMismatch
                | FpmEatCoseVerificationReason::ContentTypeMismatch
                | FpmEatCoseVerificationReason::KeyIdMismatch
        )
    }) {
        FpmEatCoseVerificationStatus::ConflictingEvidence
    } else {
        FpmEatCoseVerificationStatus::InvalidEvidence
    };

    FpmVerifiedEatCoseEvidence {
        status,
        reasons: reasons.into_iter().collect(),
        evidence_digest,
        payload_digest: Some(payload_digest),
        subject_id: Some(subject_id),
        audience: Some(audience),
        nonce: Some(nonce),
        eat_profile_uri: Some(eat_profile_uri),
        key_id,
        verification_key_digest,
    }
}

fn invalid_result(
    evidence_digest: String,
    verification_key_digest: String,
    reasons: BTreeSet<FpmEatCoseVerificationReason>,
    key_id: Option<Vec<u8>>,
) -> FpmVerifiedEatCoseEvidence {
    invalid_result_with_payload(
        evidence_digest,
        verification_key_digest,
        reasons,
        key_id,
        String::new(),
    )
}

fn invalid_result_with_payload(
    evidence_digest: String,
    verification_key_digest: String,
    reasons: BTreeSet<FpmEatCoseVerificationReason>,
    key_id: Option<Vec<u8>>,
    payload_digest: String,
) -> FpmVerifiedEatCoseEvidence {
    FpmVerifiedEatCoseEvidence {
        status: FpmEatCoseVerificationStatus::InvalidEvidence,
        reasons: reasons.into_iter().collect(),
        evidence_digest,
        payload_digest: if payload_digest.is_empty() {
            None
        } else {
            Some(payload_digest)
        },
        subject_id: None,
        audience: None,
        nonce: None,
        eat_profile_uri: None,
        key_id,
        verification_key_digest,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coset::{CborSerializable, CoseSign1Builder, HeaderBuilder, TaggedCborSerializable};
    use p256::ecdsa::{signature::Signer, SigningKey};

    fn signing_key() -> SigningKey {
        SigningKey::from_bytes((&[7u8; 32]).into()).expect("test signing key")
    }

    fn verification_key_sec1() -> Vec<u8> {
        VerifyingKey::from(&signing_key())
            .to_encoded_point(false)
            .as_bytes()
            .to_vec()
    }

    fn alternate_verification_key_sec1() -> Vec<u8> {
        let alternate = SigningKey::from_bytes((&[8u8; 32]).into())
            .expect("alternate signing key");
        VerifyingKey::from(&alternate)
            .to_encoded_point(false)
            .as_bytes()
            .to_vec()
    }

    fn payload(
        subject: &str,
        audience: &str,
        nonce: &[u8],
    ) -> Vec<u8> {
        let claims = coset::cbor::Value::Map(vec![
            (
                coset::cbor::Value::Integer(CLAIM_SUB.into()),
                coset::cbor::Value::Text(subject.into()),
            ),
            (
                coset::cbor::Value::Integer(CLAIM_AUD.into()),
                coset::cbor::Value::Text(audience.into()),
            ),
            (
                coset::cbor::Value::Integer(CLAIM_NONCE.into()),
                coset::cbor::Value::Bytes(nonce.into()),
            ),
            (
                coset::cbor::Value::Integer(CLAIM_EAT_PROFILE.into()),
                coset::cbor::Value::Text(FPM_EAT_PROFILE_URI.into()),
            ),
        ]);
        claims.to_vec().expect("CBOR payload")
    }

    fn token(nonce: &[u8]) -> Vec<u8> {
        let protected = HeaderBuilder::new()
            .algorithm(iana::Algorithm::ES256)
            .key_id(b"fpm-key-1".to_vec())
            .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
            .build();

        CoseSign1Builder::new()
            .protected(protected)
            .payload(payload("source-1", "fpm-verifier", nonce))
            .create_signature(&[], |data| signing_key().sign(data))
            .build()
            .to_tagged_vec()
            .expect("COSE_Sign1")
    }

    fn input(token_bytes: Vec<u8>) -> FpmEatCoseVerificationInput {
        let nonce = b"fresh-nonce-32-bytes-123456789012";
        FpmEatCoseVerificationInput {
            expected_subject_id: "source-1".into(),
            expected_audience: "fpm-verifier".into(),
            expected_nonce: nonce.to_vec(),
            expected_key_id: b"fpm-key-1".to_vec(),
            trusted_public_key_sec1: verification_key_sec1(),
            expected_evidence_digest: Some(hex_digest(&token_bytes)),
            token_bytes,
        }
    }

    #[test]
    fn exact_signed_eat_qualifies() {
        let nonce = b"fresh-nonce-32-bytes-123456789012";
        let result = verify_fpm_eat_cose_sign1(&input(token(nonce)));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::QualifiedForProfile
        );
        assert!(result.reasons.is_empty());
        assert_eq!(result.subject_id.as_deref(), Some("source-1"));
        assert_eq!(result.audience.as_deref(), Some("fpm-verifier"));
    }

    #[test]
    fn signature_tamper_is_invalid() {
        let mut token = token(b"fresh-nonce-32-bytes-123456789012");
        let last = token.len() - 1;
        token[last] ^= 0x01;
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::InvalidEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::SignatureInvalid));
    }

    #[test]
    fn wrong_nonce_conflicts_after_valid_signature() {
        let token = token(b"different-nonce-32-bytes-123456789");
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::ConflictingEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::NonceMismatch));
        assert!(!result
            .reasons
            .contains(&FpmEatCoseVerificationReason::SignatureInvalid));
    }

    #[test]
    fn profile_substitution_conflicts_after_valid_signature() {
        let nonce = b"fresh-nonce-32-bytes-123456789012";
        let claims = coset::cbor::Value::Map(vec![
            (coset::cbor::Value::Integer(CLAIM_SUB.into()), coset::cbor::Value::Text("source-1".into())),
            (coset::cbor::Value::Integer(CLAIM_AUD.into()), coset::cbor::Value::Text("fpm-verifier".into())),
            (coset::cbor::Value::Integer(CLAIM_NONCE.into()), coset::cbor::Value::Bytes(nonce.to_vec())),
            (coset::cbor::Value::Integer(CLAIM_EAT_PROFILE.into()), coset::cbor::Value::Text("tag:luminousdynamics.org,2026:other-profile".into())),
        ]);
        let payload = claims.to_vec().expect("CBOR payload");
        let protected = HeaderBuilder::new()
            .algorithm(iana::Algorithm::ES256)
            .key_id(b"fpm-key-1".to_vec())
            .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
            .build();
        let token = CoseSign1Builder::new()
            .protected(protected)
            .payload(payload)
            .create_signature(&[], |data| signing_key().sign(data))
            .build()
            .to_tagged_vec()
            .expect("COSE_Sign1");
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::ConflictingEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::EatProfileMismatch));
    }

    #[test]
    fn wrong_key_id_conflicts_after_valid_signature() {
        let protected = HeaderBuilder::new()
            .algorithm(iana::Algorithm::ES256)
            .key_id(b"other-key".to_vec())
            .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
            .build();
        let token = CoseSign1Builder::new()
            .protected(protected)
            .payload(payload("source-1", "fpm-verifier", b"fresh-nonce-32-bytes-123456789012"))
            .create_signature(&[], |data| signing_key().sign(data))
            .build()
            .to_tagged_vec()
            .expect("COSE_Sign1");

        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::ConflictingEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::KeyIdMismatch));
        assert!(!result
            .reasons
            .contains(&FpmEatCoseVerificationReason::SignatureInvalid));
    }

    #[test]
    fn wrong_audience_conflicts_after_valid_signature() {
        let token = {
            let protected = HeaderBuilder::new()
                .algorithm(iana::Algorithm::ES256)
                .key_id(b"fpm-key-1".to_vec())
                .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
                .build();
            CoseSign1Builder::new()
                .protected(protected)
                .payload(payload("source-1", "other-audience", b"fresh-nonce-32-bytes-123456789012"))
                .create_signature(&[], |data| signing_key().sign(data))
                .build()
                .to_tagged_vec()
                .expect("COSE_Sign1")
        };
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::ConflictingEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::AudienceMismatch));
    }

    #[test]
    fn protected_header_substitution_breaks_signature() {
        let protected = HeaderBuilder::new()
            .algorithm(iana::Algorithm::ES256)
            .key_id(b"fpm-key-1".to_vec())
            .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
            .build();
        let mut token = CoseSign1Builder::new()
            .protected(protected)
            .payload(payload("source-1", "fpm-verifier", b"fresh-nonce-32-bytes-123456789012"))
            .create_signature(&[], |data| signing_key().sign(data))
            .build();
        token.protected.original_data = None;
        token.protected.header.key_id = b"other-key".to_vec();
        let token = token.to_tagged_vec().expect("tampered COSE");
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::InvalidEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::KeyIdMismatch));
    }

    #[test]
    fn unprotected_header_is_rejected() {
        let protected = HeaderBuilder::new()
            .algorithm(iana::Algorithm::ES256)
            .key_id(b"fpm-key-1".to_vec())
            .content_type(ContentType::Text(FPM_EAT_MEDIA_TYPE.into()))
            .build();
        let mut token = CoseSign1Builder::new()
            .protected(protected)
            .payload(payload("source-1", "fpm-verifier", b"fresh-nonce-32-bytes-123456789012"))
            .create_signature(&[], |data| signing_key().sign(data))
            .build();
        token.unprotected = HeaderBuilder::new()
            .key_id(b"fpm-key-1".to_vec())
            .build();
        let token = token.to_tagged_vec().expect("unprotected COSE");
        let result = verify_fpm_eat_cose_sign1(&input(token));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::InvalidEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::UnprotectedHeadersPresent));
    }

    #[test]
    fn trailing_cbor_is_rejected() {
        let valid = token(b"fresh-nonce-32-bytes-123456789012");
        let mut trailing = valid.clone();
        trailing.extend_from_slice(&[0x00]);
        let result = verify_fpm_eat_cose_sign1(&input(trailing));
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::InvalidEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::MalformedCoseSign1));
    }

    #[test]
    fn wrong_key_is_invalid() {
        let mut input = input(token(b"fresh-nonce-32-bytes-123456789012"));
        input.trusted_public_key_sec1 = alternate_verification_key_sec1();
        let result = verify_fpm_eat_cose_sign1(&input);
        assert_eq!(
            result.status,
            FpmEatCoseVerificationStatus::InvalidEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmEatCoseVerificationReason::SignatureInvalid));
    }
}
