// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Xenia receipt adapter for Mycelix Forge principal authentication.
//!
//! This crate is deliberately **not** a Xenia daemon client and does not
//! verify Ed25519 or ML-DSA signatures itself. It defines the exact portable
//! receipt shape Forge expects from a Xenia verifier and checks that the
//! receipt is consistent with one exact [`PrincipalAuthenticationRequest`]
//! and [`PrincipalBinding`].
//!
//! A receipt only becomes trustworthy when produced by a verifier that has
//! independently established Xenia's hybrid proof-of-possession, consumed the
//! one-time challenge, and resolved the exact enrolled key pair to the stable
//! operator identity. FORGE-005B2 owns that producer-side theorem.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_forge_authentication::{
    AuthenticationObservation, PrincipalAuthenticationRequest, PrincipalBinding,
};
use mycelix_forge_core::{Digest, DigestAlgorithm, ProtocolVersion};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const PROVIDER_NAMESPACE_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/provider-namespace/v1\0";
const OPERATOR_ID_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/operator-id/v1\0";
const KEY_LINEAGE_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/key-lineage/v1\0";
const CHALLENGE_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/challenge/v1\0";
const RECEIPT_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/verification-receipt/v1\0";
const FRESHNESS_DOMAIN_V1: &[u8] = b"mycelix-forge/xenia/freshness-evidence/v1\0";

/// Xenia receipt v1 fixes SHA-256 for cross-repository interoperability.
pub const XENIA_RECEIPT_DIGEST_ALGORITHM: DigestAlgorithm = DigestAlgorithm::Sha256;

/// Hybrid signature suite whose successful verification a v1 receipt asserts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum XeniaHybridSuite {
    /// Both Ed25519 and ML-DSA-65 verified over the same typed transcript.
    Ed25519MlDsa65V1,
}

impl XeniaHybridSuite {
    const fn code(self) -> u16 {
        match self {
            Self::Ed25519MlDsa65V1 => 1,
        }
    }
}

/// Portable receipt emitted by a Xenia verifier after successful operator
/// proof-of-possession and one-time challenge consumption.
///
/// Deserializing this type is **not** evidence that those checks happened.
/// This crate only verifies exact structural/canonical cross-links. The Xenia
/// producer must establish the stronger claim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct XeniaVerificationReceiptV1 {
    version: ProtocolVersion,
    request_commitment: Digest,
    operator_id_commitment: Digest,
    key_lineage_commitment: Digest,
    challenge_commitment: Digest,
    suite: XeniaHybridSuite,
    cryptographic_evidence: Digest,
    challenge_consumption_evidence: Digest,
    verifier_state_commitment: Digest,
    verified_at_unix_secs: u64,
}

impl XeniaVerificationReceiptV1 {
    /// Construct a v1 receipt. This constructor does not itself prove that the
    /// verifier actually performed the claimed cryptographic/freshness checks.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        request_commitment: Digest,
        operator_id_commitment: Digest,
        key_lineage_commitment: Digest,
        challenge_commitment: Digest,
        suite: XeniaHybridSuite,
        cryptographic_evidence: Digest,
        challenge_consumption_evidence: Digest,
        verifier_state_commitment: Digest,
        verified_at_unix_secs: u64,
    ) -> Self {
        Self {
            version: ProtocolVersion::CURRENT,
            request_commitment,
            operator_id_commitment,
            key_lineage_commitment,
            challenge_commitment,
            suite,
            cryptographic_evidence,
            challenge_consumption_evidence,
            verifier_state_commitment,
            verified_at_unix_secs,
        }
    }

    /// Exact Forge authentication request named by this receipt.
    pub fn request_commitment(&self) -> &Digest {
        &self.request_commitment
    }

    /// Stable Xenia logical operator identity commitment.
    pub fn operator_id_commitment(&self) -> &Digest {
        &self.operator_id_commitment
    }

    /// Current enrolled Xenia hybrid key-lineage commitment.
    pub fn key_lineage_commitment(&self) -> &Digest {
        &self.key_lineage_commitment
    }

    /// Commitment to the verifier-issued one-time challenge.
    pub fn challenge_commitment(&self) -> &Digest {
        &self.challenge_commitment
    }

    /// Hybrid suite the producer claims to have verified.
    pub const fn suite(&self) -> XeniaHybridSuite {
        self.suite
    }

    /// Opaque commitment to the producer's cryptographic verification record.
    pub fn cryptographic_evidence(&self) -> &Digest {
        &self.cryptographic_evidence
    }

    /// Opaque commitment proving the one-time challenge was consumed.
    pub fn challenge_consumption_evidence(&self) -> &Digest {
        &self.challenge_consumption_evidence
    }

    /// Commitment to the authoritative Xenia verifier/enrollment state used.
    pub fn verifier_state_commitment(&self) -> &Digest {
        &self.verifier_state_commitment
    }

    /// Verification time recorded by the producer, in Unix seconds.
    pub const fn verified_at_unix_secs(&self) -> u64 {
        self.verified_at_unix_secs
    }

    /// Canonical receipt bytes.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, XeniaAdapterError> {
        ensure_sha256_receipt(self)?;
        let mut out = Vec::new();
        out.extend_from_slice(RECEIPT_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());
        push_digest(&mut out, &self.request_commitment)?;
        push_digest(&mut out, &self.operator_id_commitment)?;
        push_digest(&mut out, &self.key_lineage_commitment)?;
        push_digest(&mut out, &self.challenge_commitment)?;
        out.extend_from_slice(&self.suite.code().to_be_bytes());
        push_digest(&mut out, &self.cryptographic_evidence)?;
        push_digest(&mut out, &self.challenge_consumption_evidence)?;
        push_digest(&mut out, &self.verifier_state_commitment)?;
        out.extend_from_slice(&self.verified_at_unix_secs.to_be_bytes());
        Ok(out)
    }

    /// Digest identifying this exact receipt.
    pub fn digest(&self) -> Result<Digest, XeniaAdapterError> {
        Ok(Digest::of_bytes(
            XENIA_RECEIPT_DIGEST_ALGORITHM,
            &self.canonical_bytes()?,
        ))
    }
}

/// Stable provider namespace used by the Forge/Xenia v1 adapter.
pub fn xenia_provider_namespace() -> Digest {
    Digest::of_bytes(XENIA_RECEIPT_DIGEST_ALGORITHM, PROVIDER_NAMESPACE_DOMAIN_V1)
}

/// Stable commitment for Xenia's logical `operator_id`.
pub fn xenia_operator_id_commitment(operator_id: &str) -> Result<Digest, XeniaAdapterError> {
    let mut out = Vec::new();
    out.extend_from_slice(OPERATOR_ID_DOMAIN_V1);
    push_bytes(&mut out, "operator_id", operator_id.as_bytes())?;
    Ok(Digest::of_bytes(XENIA_RECEIPT_DIGEST_ALGORITHM, &out))
}

/// Commitment to the exact enrolled hybrid key lineage.
///
/// The logical operator identity is deliberately not included. A key
/// replacement therefore changes the lineage while preserving the same
/// operator/principal identity.
pub fn xenia_key_lineage_commitment(
    ed25519_pubkey: &[u8],
    ml_dsa_65_pubkey: &[u8],
    ml_dsa_87_pubkey: Option<&[u8]>,
) -> Result<Digest, XeniaAdapterError> {
    let mut out = Vec::new();
    out.extend_from_slice(KEY_LINEAGE_DOMAIN_V1);
    push_bytes(&mut out, "ed25519_pubkey", ed25519_pubkey)?;
    push_bytes(&mut out, "ml_dsa_65_pubkey", ml_dsa_65_pubkey)?;
    match ml_dsa_87_pubkey {
        Some(key) => {
            out.push(1);
            push_bytes(&mut out, "ml_dsa_87_pubkey", key)?;
        }
        None => out.push(0),
    }
    Ok(Digest::of_bytes(XENIA_RECEIPT_DIGEST_ALGORITHM, &out))
}

/// Commitment to the exact Forge verifier-issued challenge.
pub fn xenia_challenge_commitment(challenge: &[u8; 32]) -> Digest {
    let mut out = Vec::with_capacity(CHALLENGE_DOMAIN_V1.len() + challenge.len());
    out.extend_from_slice(CHALLENGE_DOMAIN_V1);
    out.extend_from_slice(challenge);
    Digest::of_bytes(XENIA_RECEIPT_DIGEST_ALGORITHM, &out)
}

/// Validate all Xenia/Forge structural cross-links and convert a receipt into
/// the provider-neutral 005A observation type.
///
/// This does not independently verify the producer's Ed25519/ML-DSA evidence.
/// It is safe only when the receipt came from a producer whose issuance path
/// established those facts (FORGE-005B2).
pub fn observe_xenia_receipt(
    request: &PrincipalAuthenticationRequest,
    binding: &PrincipalBinding,
    receipt: &XeniaVerificationReceiptV1,
) -> Result<AuthenticationObservation, XeniaAdapterError> {
    ensure_sha256_receipt(receipt)?;

    if binding.provider_namespace() != &xenia_provider_namespace() {
        return Err(XeniaAdapterError::ProviderNamespaceMismatch);
    }

    if binding.provider_principal() != receipt.operator_id_commitment() {
        return Err(XeniaAdapterError::OperatorIdentityMismatch);
    }

    if binding.key_lineage() != receipt.key_lineage_commitment() {
        return Err(XeniaAdapterError::KeyLineageMismatch);
    }

    let expected_request = request.digest(XENIA_RECEIPT_DIGEST_ALGORITHM)?;
    if receipt.request_commitment() != &expected_request {
        return Err(XeniaAdapterError::RequestMismatch);
    }

    let expected_challenge = xenia_challenge_commitment(request.challenge());
    if receipt.challenge_commitment() != &expected_challenge {
        return Err(XeniaAdapterError::ChallengeMismatch);
    }

    if receipt.suite() != XeniaHybridSuite::Ed25519MlDsa65V1 {
        return Err(XeniaAdapterError::UnsupportedHybridSuite);
    }

    let provider_evidence = receipt.digest()?;
    let freshness_evidence = freshness_evidence(receipt)?;

    Ok(AuthenticationObservation::new(
        expected_request,
        provider_evidence,
        freshness_evidence,
    ))
}

fn freshness_evidence(receipt: &XeniaVerificationReceiptV1) -> Result<Digest, XeniaAdapterError> {
    let mut out = Vec::new();
    out.extend_from_slice(FRESHNESS_DOMAIN_V1);
    push_digest(&mut out, receipt.challenge_commitment())?;
    push_digest(&mut out, receipt.challenge_consumption_evidence())?;
    out.extend_from_slice(&receipt.verified_at_unix_secs().to_be_bytes());
    Ok(Digest::of_bytes(XENIA_RECEIPT_DIGEST_ALGORITHM, &out))
}

fn ensure_sha256_receipt(receipt: &XeniaVerificationReceiptV1) -> Result<(), XeniaAdapterError> {
    if receipt.version.get() != ProtocolVersion::CURRENT.get() {
        return Err(XeniaAdapterError::UnsupportedProtocolVersion(receipt.version.get()));
    }

    for digest in [
        &receipt.request_commitment,
        &receipt.operator_id_commitment,
        &receipt.key_lineage_commitment,
        &receipt.challenge_commitment,
        &receipt.cryptographic_evidence,
        &receipt.challenge_consumption_evidence,
        &receipt.verifier_state_commitment,
    ] {
        if digest.algorithm() != XENIA_RECEIPT_DIGEST_ALGORITHM {
            return Err(XeniaAdapterError::DigestAlgorithmMismatch);
        }
    }
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), XeniaAdapterError> {
    let algorithm = digest.algorithm().id().as_bytes();
    push_bytes(out, "digest_algorithm", algorithm)?;
    push_bytes(out, "digest", digest.as_bytes())
}

fn push_bytes(out: &mut Vec<u8>, field: &'static str, bytes: &[u8]) -> Result<(), XeniaAdapterError> {
    let len = u32::try_from(bytes.len()).map_err(|_| XeniaAdapterError::CanonicalFieldTooLarge {
        field,
        len: bytes.len(),
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Xenia adapter validation failures.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum XeniaAdapterError {
    /// Unsupported Forge/Xenia receipt protocol version.
    #[error("unsupported Xenia Forge receipt protocol version: {0}")]
    UnsupportedProtocolVersion(u16),
    /// Xenia receipt v1 requires SHA-256 commitments throughout.
    #[error("Xenia Forge receipt v1 requires SHA-256 commitments")]
    DigestAlgorithmMismatch,
    /// Principal binding is not in the Xenia operator provider namespace.
    #[error("principal binding does not name the Xenia operator provider namespace")]
    ProviderNamespaceMismatch,
    /// Receipt stable operator identity differs from the Forge binding.
    #[error("Xenia receipt operator identity does not match principal binding")]
    OperatorIdentityMismatch,
    /// Receipt current key lineage differs from the Forge binding.
    #[error("Xenia receipt key lineage does not match principal binding")]
    KeyLineageMismatch,
    /// Receipt names a different Forge authentication request.
    #[error("Xenia receipt request commitment does not match")]
    RequestMismatch,
    /// Receipt names a different verifier challenge.
    #[error("Xenia receipt challenge commitment does not match")]
    ChallengeMismatch,
    /// Receipt uses a hybrid suite not supported by this bridge version.
    #[error("unsupported Xenia hybrid authentication suite")]
    UnsupportedHybridSuite,
    /// Canonical field exceeded the v1 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed byte length.
        len: usize,
        /// Maximum length.
        max: usize,
    },
    /// Provider-neutral authentication contract error.
    #[error(transparent)]
    Authentication(#[from] mycelix_forge_authentication::AuthenticationError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_authentication::{PrincipalAuthenticationRequest, PrincipalBinding};
    use mycelix_forge_authority::{Capability, PrincipalId};
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x12)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn fixture() -> (
        PrincipalAuthenticationRequest,
        PrincipalBinding,
        XeniaVerificationReceiptV1,
    ) {
        let operator = xenia_operator_id_commitment("operator:alice").unwrap();
        let lineage = xenia_key_lineage_commitment(&[0x21; 32], &[0x22; 64], None).unwrap();
        let binding = PrincipalBinding::new(
            xenia_provider_namespace(),
            operator.clone(),
            principal(0x31),
            lineage.clone(),
        );
        let binding_commitment = binding.digest(DigestAlgorithm::Sha256).unwrap();
        let request = PrincipalAuthenticationRequest::new(
            project(),
            digest(0x41),
            principal(0x31),
            binding_commitment,
            Capability::ReviewSource,
            digest(0x51),
            [0x61; 32],
        );
        let request_commitment = request.digest(DigestAlgorithm::Sha256).unwrap();
        let receipt = XeniaVerificationReceiptV1::new(
            request_commitment,
            operator,
            lineage,
            xenia_challenge_commitment(request.challenge()),
            XeniaHybridSuite::Ed25519MlDsa65V1,
            digest(0x71),
            digest(0x72),
            digest(0x73),
            1_797_000_000,
        );
        (request, binding, receipt)
    }

    #[test]
    fn exact_receipt_maps_to_authentication_observation() {
        let (request, binding, receipt) = fixture();
        let observation = observe_xenia_receipt(&request, &binding, &receipt).unwrap();
        assert_eq!(
            observation.request_commitment(),
            &request.digest(DigestAlgorithm::Sha256).unwrap()
        );
        assert_eq!(observation.provider_evidence(), &receipt.digest().unwrap());
    }

    #[test]
    fn operator_identity_mutation_fails() {
        let (request, binding, mut receipt) = fixture();
        receipt.operator_id_commitment = xenia_operator_id_commitment("operator:bob").unwrap();
        assert_eq!(
            observe_xenia_receipt(&request, &binding, &receipt).unwrap_err(),
            XeniaAdapterError::OperatorIdentityMismatch
        );
    }

    #[test]
    fn key_rotation_invalidates_old_receipt() {
        let (request, binding, mut receipt) = fixture();
        receipt.key_lineage_commitment =
            xenia_key_lineage_commitment(&[0x23; 32], &[0x24; 64], None).unwrap();
        assert_eq!(
            observe_xenia_receipt(&request, &binding, &receipt).unwrap_err(),
            XeniaAdapterError::KeyLineageMismatch
        );
    }

    #[test]
    fn challenge_mutation_fails() {
        let (request, binding, mut receipt) = fixture();
        receipt.challenge_commitment = xenia_challenge_commitment(&[0x62; 32]);
        assert_eq!(
            observe_xenia_receipt(&request, &binding, &receipt).unwrap_err(),
            XeniaAdapterError::ChallengeMismatch
        );
    }

    #[test]
    fn request_mutation_fails() {
        let (request, binding, mut receipt) = fixture();
        receipt.request_commitment = digest(0x99);
        assert_eq!(
            observe_xenia_receipt(&request, &binding, &receipt).unwrap_err(),
            XeniaAdapterError::RequestMismatch
        );
    }

    #[test]
    fn receipt_serde_round_trip_preserves_identity() {
        let (_, _, receipt) = fixture();
        let json = serde_json::to_vec(&receipt).unwrap();
        let decoded: XeniaVerificationReceiptV1 = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, receipt);
        assert_eq!(decoded.digest().unwrap(), receipt.digest().unwrap());
    }

    #[test]
    fn key_lineage_changes_when_any_key_changes() {
        let a = xenia_key_lineage_commitment(&[1; 32], &[2; 64], None).unwrap();
        let b = xenia_key_lineage_commitment(&[1; 32], &[3; 64], None).unwrap();
        let c = xenia_key_lineage_commitment(&[1; 32], &[2; 64], Some(&[4; 96])).unwrap();
        assert_ne!(a, b);
        assert_ne!(a, c);
    }
}
