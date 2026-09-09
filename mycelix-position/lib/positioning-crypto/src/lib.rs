// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Concrete cryptographic digest providers for Mycelix Position evidence.
//!
//! This crate is intentionally separate from `positioning`. The core Position
//! crate owns the provider-neutral typed digest contract; this crate qualifies
//! concrete implementations against that contract without making cryptographic
//! dependencies part of every positioning consumer.

use positioning::{
    QualificationDigestError, QualificationDigestProfileV1, QualificationDigestProvider,
    QualificationDigestProviderError,
};
use sha2::{Digest, Sha256};

/// Durable implementation-neutral SHA-256 algorithm identifier.
pub const SHA256_ALGORITHM_ID: &str = "sha-256";

/// Durable Position digest-profile identifier for SHA-256 / FIPS 180-4 semantics.
pub const SHA256_PROFILE_ID: &str = "sha-256-fips180-4-v1";

/// SHA-256 output size in bytes.
pub const SHA256_OUTPUT_BYTES: u16 = 32;

/// Construct the exact Position SHA-256 digest profile.
///
/// This profile identifies algorithm semantics, not the RustCrypto provider,
/// crate version, build target, or qualification environment.
pub fn sha256_qualification_digest_profile_v1()
-> Result<QualificationDigestProfileV1, QualificationDigestError> {
    QualificationDigestProfileV1::new(SHA256_PROFILE_ID, SHA256_ALGORITHM_ID, SHA256_OUTPUT_BYTES)
}

/// RustCrypto `sha2` implementation of the v0.6 SHA-256 provider theorem.
///
/// The provider hashes exactly the byte slice supplied by the v0.5 contract.
/// It adds no framing, domain prefix, profile bytes, serialization, signature,
/// authentication state, or authority semantics.
#[derive(Debug, Default, Clone, Copy)]
pub struct RustCryptoSha256ProviderV1;

impl QualificationDigestProvider for RustCryptoSha256ProviderV1 {
    fn algorithm_id(&self) -> &str {
        SHA256_ALGORITHM_ID
    }

    fn digest(&self, bytes: &[u8]) -> Result<Vec<u8>, QualificationDigestProviderError> {
        Ok(Sha256::digest(bytes).to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use positioning::{
        FacetStatus, QualificationAdmissionProfileV1, QualificationEnvelopeCodecV1,
        QualificationEnvelopePayload, QualificationFacet, QualificationManifest, TheoremId,
        digest_admitted_qualification_envelope_v1,
    };
    use std::fmt::Write;

    fn lowercase_hex(bytes: &[u8]) -> String {
        let mut out = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            write!(&mut out, "{byte:02x}")
                .expect("writing hexadecimal bytes to String cannot fail");
        }
        out
    }

    fn assert_known_answer(message: &[u8], expected_hex: &str) {
        let provider = RustCryptoSha256ProviderV1;
        let digest = provider.digest(message).unwrap();
        assert_eq!(digest.len(), usize::from(SHA256_OUTPUT_BYTES));
        assert_eq!(lowercase_hex(&digest), expected_hex);
    }

    fn codec() -> QualificationEnvelopeCodecV1 {
        QualificationEnvelopeCodecV1::new(QualificationAdmissionProfileV1::default()).unwrap()
    }

    fn manifest_payload(subject: &str) -> QualificationEnvelopePayload {
        QualificationEnvelopePayload::Manifest(QualificationManifest {
            subject_commitment: subject.into(),
            facets: vec![QualificationFacet {
                theorem_id: TheoremId::from("q.crypto.sha256.integration.v1"),
                subject_commitment: subject.into(),
                status: FacetStatus::Indeterminate,
                verifier_or_profile: None,
                evidence_refs: Vec::new(),
                dependency_commitments: Vec::new(),
                diagnostics_commitment: None,
            }],
        })
    }

    #[test]
    fn sha256_profile_is_exact_and_implementation_neutral() {
        let profile = sha256_qualification_digest_profile_v1().unwrap();
        assert_eq!(profile.profile_id(), SHA256_PROFILE_ID);
        assert_eq!(profile.algorithm_id(), SHA256_ALGORITHM_ID);
        assert_eq!(profile.output_bytes(), SHA256_OUTPUT_BYTES);
        assert!(!profile.profile_id().contains("rustcrypto"));
        assert!(!profile.profile_id().contains("0.10.9"));
    }

    #[test]
    fn rustcrypto_provider_declares_exact_sha256_algorithm() {
        assert_eq!(
            RustCryptoSha256ProviderV1.algorithm_id(),
            SHA256_ALGORITHM_ID
        );
    }

    #[test]
    fn nist_sha256_known_answer_empty() {
        assert_known_answer(
            b"",
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        );
    }

    #[test]
    fn nist_sha256_known_answer_abc() {
        assert_known_answer(
            b"abc",
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        );
    }

    #[test]
    fn nist_sha256_known_answer_multiblock() {
        assert_known_answer(
            b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
        );
    }

    #[test]
    fn nist_sha256_known_answer_million_a() {
        let message = vec![b'a'; 1_000_000];
        assert_known_answer(
            &message,
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
        );
    }

    #[test]
    fn typed_envelope_path_hashes_exact_admitted_bytes() {
        let codec = codec();
        let encoded = codec.encode(&manifest_payload("subject:sha256:a")).unwrap();
        let admitted = codec.admit_bytes(&encoded).unwrap();
        let provider = RustCryptoSha256ProviderV1;

        let typed = digest_admitted_qualification_envelope_v1(
            &codec,
            &admitted,
            sha256_qualification_digest_profile_v1().unwrap(),
            &provider,
        )
        .unwrap();
        let direct = provider.digest(admitted.as_bytes()).unwrap();

        assert_eq!(typed.digest().bytes(), direct.as_slice());
        assert_eq!(typed.admission_profile(), codec.profile());
        assert_eq!(
            typed.digest().to_algorithm_qualified_text(),
            format!("{SHA256_ALGORITHM_ID}:{}", lowercase_hex(&direct))
        );
    }

    #[test]
    fn changed_valid_envelope_changes_concrete_digest_fixture() {
        let codec = codec();
        let left_bytes = codec.encode(&manifest_payload("subject:sha256:a")).unwrap();
        let right_bytes = codec.encode(&manifest_payload("subject:sha256:b")).unwrap();
        assert_ne!(left_bytes, right_bytes);

        let left_admitted = codec.admit_bytes(&left_bytes).unwrap();
        let right_admitted = codec.admit_bytes(&right_bytes).unwrap();
        let provider = RustCryptoSha256ProviderV1;

        let left = digest_admitted_qualification_envelope_v1(
            &codec,
            &left_admitted,
            sha256_qualification_digest_profile_v1().unwrap(),
            &provider,
        )
        .unwrap();
        let right = digest_admitted_qualification_envelope_v1(
            &codec,
            &right_admitted,
            sha256_qualification_digest_profile_v1().unwrap(),
            &provider,
        )
        .unwrap();

        assert_ne!(left.digest().bytes(), right.digest().bytes());
    }

    #[test]
    fn sha256_provider_cannot_satisfy_a_different_digest_profile() {
        let codec = codec();
        let encoded = codec.encode(&manifest_payload("subject:sha256:a")).unwrap();
        let admitted = codec.admit_bytes(&encoded).unwrap();
        let other_profile =
            QualificationDigestProfileV1::new("other-digest-v1", "other-digest", 32).unwrap();

        assert_eq!(
            digest_admitted_qualification_envelope_v1(
                &codec,
                &admitted,
                other_profile,
                &RustCryptoSha256ProviderV1,
            ),
            Err(QualificationDigestError::ProviderAlgorithmMismatch)
        );
    }

    #[test]
    fn malformed_envelope_fails_before_typed_sha256_identity() {
        let codec = codec();
        let malformed = vec![0u8; 64];
        let admitted = codec.admit_bytes(&malformed).unwrap();

        assert!(matches!(
            digest_admitted_qualification_envelope_v1(
                &codec,
                &admitted,
                sha256_qualification_digest_profile_v1().unwrap(),
                &RustCryptoSha256ProviderV1,
            ),
            Err(QualificationDigestError::Envelope(_))
        ));
    }
}
