// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cryptographic provider verification for Xenia-backed Forge authentication.
//!
//! `mycelix-forge-xenia` deliberately treats a serialized Xenia receipt as
//! structurally meaningful but not cryptographically trustworthy. This crate
//! adds the missing provider-provenance theorem:
//!
//! ```text
//! XeniaProviderAttestedReceiptV1
//! + explicitly trusted Xenia verifier Ed25519 key
//! + explicitly trusted Xenia verifier ML-DSA-65 key
//! + both provider signatures valid over the exact receipt
//! + FORGE-005B1 structural cross-links valid
//!     ↓
//! ProviderVerifiedXeniaAuthenticationV1
//! ```
//!
//! This does not independently replay Xenia's one-time challenge store. It
//! proves that the exact receipt came from the trusted verifier identity whose
//! producer contract owns that freshness theorem.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use ed25519_dalek::{Signature as Ed25519Signature, Verifier as _, VerifyingKey};
use ml_dsa::{
    EncodedSignature as MlDsaEncodedSignature, EncodedVerifyingKey as MlDsaEncodedVerifyingKey,
    MlDsa65, Signature as MlDsaSignature, VerifyingKey as MlDsaVerifyingKey,
    signature::Verifier as _,
};
use mycelix_forge_authentication::{
    AuthenticationObservation, PrincipalAuthenticationRequest, PrincipalBinding,
};
use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_xenia::{
    XeniaAdapterError, XeniaVerificationReceiptV1, observe_xenia_receipt,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// FIPS 204 ML-DSA-65 verifying-key size.
pub const ML_DSA_65_PK_LEN: usize = 1952;
/// FIPS 204 ML-DSA-65 signature size.
pub const ML_DSA_65_SIG_LEN: usize = 3309;

const PROVIDER_ATTESTATION_DOMAIN_V1: &[u8] = b"xenia-forge-provider-attestation-v1\0";
const VERIFIER_IDENTITY_DOMAIN_V1: &[u8] = b"xenia-forge/verifier-identity/v1\0";

/// Portable provider envelope byte-compatible with Xenia's
/// `XeniaProviderAttestedReceiptV1` v1 contract.
///
/// Deserializing this type is not a trust decision. The signatures become
/// authoritative only after [`verify_xenia_provider_authentication_v1`] checks
/// them against an independently configured [`TrustedXeniaVerifierV1`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct XeniaProviderAttestedReceiptV1 {
    receipt: XeniaVerificationReceiptV1,
    verifier_identity_commitment: Digest,
    ed25519_signature: Vec<u8>,
    ml_dsa_65_signature: Vec<u8>,
}

impl XeniaProviderAttestedReceiptV1 {
    /// Exact Xenia receipt covered by both provider signatures.
    pub fn receipt(&self) -> &XeniaVerificationReceiptV1 {
        &self.receipt
    }

    /// Commitment to the Xenia verifier hybrid identity named by the envelope.
    pub fn verifier_identity_commitment(&self) -> &Digest {
        &self.verifier_identity_commitment
    }

    /// Ed25519 provider signature bytes.
    pub fn ed25519_signature(&self) -> &[u8] {
        &self.ed25519_signature
    }

    /// ML-DSA-65 provider signature bytes.
    pub fn ml_dsa_65_signature(&self) -> &[u8] {
        &self.ml_dsa_65_signature
    }
}

/// One explicitly trusted Xenia verifier identity.
///
/// The trust decision is external to this crate: configuration, a pinned host
/// identity, governance, or another higher-level policy decides which keys are
/// acceptable. This type merely validates their shape and freezes the exact
/// pair used by one verification call.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustedXeniaVerifierV1 {
    ed25519_pubkey: [u8; 32],
    ml_dsa_65_pubkey: Vec<u8>,
    identity_commitment: Digest,
}

impl TrustedXeniaVerifierV1 {
    /// Construct one trusted verifier from the exact hybrid public-key pair.
    pub fn new(
        ed25519_pubkey: [u8; 32],
        ml_dsa_65_pubkey: Vec<u8>,
    ) -> Result<Self, XeniaProviderVerifierError> {
        VerifyingKey::from_bytes(&ed25519_pubkey)
            .map_err(|_| XeniaProviderVerifierError::MalformedTrustedEd25519Key)?;
        if ml_dsa_65_pubkey.len() != ML_DSA_65_PK_LEN {
            return Err(XeniaProviderVerifierError::MalformedTrustedMlDsaKey);
        }
        let identity_commitment =
            verifier_identity_commitment_v1(&ed25519_pubkey, &ml_dsa_65_pubkey)?;
        Ok(Self {
            ed25519_pubkey,
            ml_dsa_65_pubkey,
            identity_commitment,
        })
    }

    /// Trusted Ed25519 verifier key.
    pub fn ed25519_pubkey(&self) -> &[u8; 32] {
        &self.ed25519_pubkey
    }

    /// Trusted ML-DSA-65 verifier key.
    pub fn ml_dsa_65_pubkey(&self) -> &[u8] {
        &self.ml_dsa_65_pubkey
    }

    /// Commitment to this exact hybrid verifier identity.
    pub fn identity_commitment(&self) -> &Digest {
        &self.identity_commitment
    }
}

/// Positive result proving provider provenance plus FORGE-005B1 structural
/// binding for one exact authentication request.
///
/// Fields are private so callers cannot manufacture this positive type from a
/// raw `AuthenticationObservation` or receipt-shaped object.
pub struct ProviderVerifiedXeniaAuthenticationV1 {
    observation: AuthenticationObservation,
    receipt_digest: Digest,
    verifier_identity_commitment: Digest,
}

impl ProviderVerifiedXeniaAuthenticationV1 {
    /// Provider-neutral FORGE-005A observation produced only after provider
    /// signatures and Xenia receipt cross-links verified.
    pub fn observation(&self) -> &AuthenticationObservation {
        &self.observation
    }

    /// Digest of the exact Xenia verification receipt whose provenance passed.
    pub fn receipt_digest(&self) -> &Digest {
        &self.receipt_digest
    }

    /// Exact trusted Xenia verifier identity that authenticated the receipt.
    pub fn verifier_identity_commitment(&self) -> &Digest {
        &self.verifier_identity_commitment
    }

    /// Consume the positive wrapper and recover the provider-neutral
    /// authentication observation.
    pub fn into_observation(self) -> AuthenticationObservation {
        self.observation
    }
}

/// SHA-256 commitment byte-compatible with Xenia's v1 verifier-identity
/// commitment.
pub fn verifier_identity_commitment_v1(
    ed25519_pubkey: &[u8; 32],
    ml_dsa_65_pubkey: &[u8],
) -> Result<Digest, XeniaProviderVerifierError> {
    if ml_dsa_65_pubkey.len() != ML_DSA_65_PK_LEN {
        return Err(XeniaProviderVerifierError::MalformedTrustedMlDsaKey);
    }
    let mut out = Vec::new();
    out.extend_from_slice(VERIFIER_IDENTITY_DOMAIN_V1);
    push_bytes(&mut out, "verifier_ed25519_pubkey", ed25519_pubkey)?;
    push_bytes(&mut out, "verifier_ml_dsa_65_pubkey", ml_dsa_65_pubkey)?;
    Ok(Digest::of_bytes(DigestAlgorithm::Sha256, &out))
}

/// Exact transcript byte-compatible with Xenia's provider-attestation v1.
pub fn provider_attestation_transcript_v1(
    receipt: &XeniaVerificationReceiptV1,
    verifier_identity_commitment: &Digest,
) -> Result<Vec<u8>, XeniaProviderVerifierError> {
    let receipt_bytes = receipt.canonical_bytes()?;
    let mut out = Vec::new();
    out.extend_from_slice(PROVIDER_ATTESTATION_DOMAIN_V1);
    push_bytes(&mut out, "receipt", &receipt_bytes)?;
    push_bytes(
        &mut out,
        "verifier_identity_commitment",
        verifier_identity_commitment.as_bytes(),
    )?;
    Ok(out)
}

/// Verify Xenia provider provenance and then apply the existing FORGE-005B1
/// request/binding cross-link checks.
///
/// The ordering is deliberate: raw structurally correct receipt data does not
/// become a positive provider-verified value before both provider signatures
/// pass against the independently trusted verifier identity.
pub fn verify_xenia_provider_authentication_v1(
    trusted_verifier: &TrustedXeniaVerifierV1,
    request: &PrincipalAuthenticationRequest,
    binding: &PrincipalBinding,
    envelope: &XeniaProviderAttestedReceiptV1,
) -> Result<ProviderVerifiedXeniaAuthenticationV1, XeniaProviderVerifierError> {
    if envelope.verifier_identity_commitment() != trusted_verifier.identity_commitment() {
        return Err(XeniaProviderVerifierError::VerifierIdentityMismatch);
    }

    let transcript = provider_attestation_transcript_v1(
        envelope.receipt(),
        envelope.verifier_identity_commitment(),
    )?;

    let ed_signature: [u8; 64] = envelope
        .ed25519_signature()
        .try_into()
        .map_err(|_| XeniaProviderVerifierError::MalformedEd25519Signature)?;
    let ed_vk = VerifyingKey::from_bytes(trusted_verifier.ed25519_pubkey())
        .map_err(|_| XeniaProviderVerifierError::MalformedTrustedEd25519Key)?;
    ed_vk
        .verify(&transcript, &Ed25519Signature::from_bytes(&ed_signature))
        .map_err(|_| XeniaProviderVerifierError::Ed25519VerificationFailed)?;

    let ml_key_bytes: [u8; ML_DSA_65_PK_LEN] = trusted_verifier
        .ml_dsa_65_pubkey()
        .try_into()
        .map_err(|_| XeniaProviderVerifierError::MalformedTrustedMlDsaKey)?;
    let encoded_key = MlDsaEncodedVerifyingKey::<MlDsa65>::try_from(ml_key_bytes.as_slice())
        .map_err(|_| XeniaProviderVerifierError::MalformedTrustedMlDsaKey)?;
    let ml_vk = MlDsaVerifyingKey::<MlDsa65>::decode(&encoded_key);

    let ml_signature_bytes: [u8; ML_DSA_65_SIG_LEN] = envelope
        .ml_dsa_65_signature()
        .try_into()
        .map_err(|_| XeniaProviderVerifierError::MalformedMlDsaSignature)?;
    let encoded_signature =
        MlDsaEncodedSignature::<MlDsa65>::try_from(ml_signature_bytes.as_slice())
            .map_err(|_| XeniaProviderVerifierError::MalformedMlDsaSignature)?;
    let ml_signature = MlDsaSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(XeniaProviderVerifierError::MalformedMlDsaSignature)?;
    ml_vk
        .verify(&transcript, &ml_signature)
        .map_err(|_| XeniaProviderVerifierError::MlDsaVerificationFailed)?;

    let observation = observe_xenia_receipt(request, binding, envelope.receipt())?;
    let receipt_digest = envelope.receipt().digest()?;

    Ok(ProviderVerifiedXeniaAuthenticationV1 {
        observation,
        receipt_digest,
        verifier_identity_commitment: trusted_verifier.identity_commitment().clone(),
    })
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), XeniaProviderVerifierError> {
    let len = u32::try_from(bytes.len()).map_err(|_| {
        XeniaProviderVerifierError::CanonicalFieldTooLarge {
            field,
            len: bytes.len(),
            max: u32::MAX as usize,
        }
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

/// Provider-verification failure.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum XeniaProviderVerifierError {
    /// Trusted Ed25519 key is not a valid verifying key.
    #[error("trusted Xenia verifier Ed25519 key is malformed")]
    MalformedTrustedEd25519Key,
    /// Trusted ML-DSA-65 key is malformed or has the wrong size.
    #[error("trusted Xenia verifier ML-DSA-65 key is malformed")]
    MalformedTrustedMlDsaKey,
    /// Envelope names a different verifier identity than the trusted one.
    #[error("Xenia provider envelope verifier identity does not match trusted verifier")]
    VerifierIdentityMismatch,
    /// Ed25519 provider signature has the wrong size.
    #[error("Xenia provider Ed25519 signature is malformed")]
    MalformedEd25519Signature,
    /// ML-DSA-65 provider signature is malformed.
    #[error("Xenia provider ML-DSA-65 signature is malformed")]
    MalformedMlDsaSignature,
    /// Ed25519 provider signature failed verification.
    #[error("Xenia provider Ed25519 signature verification failed")]
    Ed25519VerificationFailed,
    /// ML-DSA-65 provider signature failed verification.
    #[error("Xenia provider ML-DSA-65 signature verification failed")]
    MlDsaVerificationFailed,
    /// Canonical v1 field exceeded the u32 length bound.
    #[error("canonical field {field} is too large: {len} > {max}")]
    CanonicalFieldTooLarge {
        /// Field name.
        field: &'static str,
        /// Observed length.
        len: usize,
        /// Maximum encodable length.
        max: usize,
    },
    /// Existing Xenia receipt adapter rejected the receipt/request/binding
    /// cross-links.
    #[error(transparent)]
    XeniaAdapter(#[from] XeniaAdapterError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        B32, Signature as MlDsaSignatureT, SigningKey as MlDsaSigningKey,
        signature::{Keypair as _, Signer as _},
    };
    use mycelix_forge_authentication::{PrincipalAuthenticationRequest, PrincipalBinding};
    use mycelix_forge_authority::{Capability, PrincipalId};
    use mycelix_forge_core::{ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN};
    use mycelix_forge_xenia::{
        XeniaHybridSuite, xenia_challenge_commitment, xenia_key_lineage_commitment,
        xenia_operator_id_commitment, xenia_provider_namespace,
    };

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

    fn receipt_fixture() -> (
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

    struct ProviderIdentity {
        ed: SigningKey,
        ml: MlDsaSigningKey<MlDsa65>,
    }

    impl ProviderIdentity {
        fn from_seeds(ed_seed: [u8; 32], ml_seed: [u8; 32]) -> Self {
            Self {
                ed: SigningKey::from_bytes(&ed_seed),
                ml: MlDsaSigningKey::<MlDsa65>::from_seed(&B32::from(ml_seed)),
            }
        }

        fn trusted(&self) -> TrustedXeniaVerifierV1 {
            TrustedXeniaVerifierV1::new(
                self.ed.verifying_key().to_bytes(),
                self.ml.verifying_key().encode().as_slice().to_vec(),
            )
            .unwrap()
        }

        fn envelope(&self, receipt: XeniaVerificationReceiptV1) -> XeniaProviderAttestedReceiptV1 {
            let trusted = self.trusted();
            let transcript = provider_attestation_transcript_v1(
                &receipt,
                trusted.identity_commitment(),
            )
            .unwrap();
            let ml_signature: MlDsaSignatureT<MlDsa65> = self.ml.sign(&transcript);
            XeniaProviderAttestedReceiptV1 {
                receipt,
                verifier_identity_commitment: trusted.identity_commitment().clone(),
                ed25519_signature: self.ed.sign(&transcript).to_bytes().to_vec(),
                ml_dsa_65_signature: ml_signature.encode().as_slice().to_vec(),
            }
        }
    }

    #[test]
    fn trusted_hybrid_provider_envelope_becomes_positive_authentication() {
        let (request, binding, receipt) = receipt_fixture();
        let provider = ProviderIdentity::from_seeds([0x81; 32], [0x82; 32]);
        let trusted = provider.trusted();
        let envelope = provider.envelope(receipt.clone());

        let positive = verify_xenia_provider_authentication_v1(
            &trusted,
            &request,
            &binding,
            &envelope,
        )
        .unwrap();

        assert_eq!(positive.receipt_digest(), &receipt.digest().unwrap());
        assert_eq!(
            positive.observation().provider_evidence(),
            &receipt.digest().unwrap()
        );
        assert_eq!(
            positive.verifier_identity_commitment(),
            trusted.identity_commitment()
        );
    }

    #[test]
    fn self_signed_attacker_identity_cannot_replace_trusted_provider() {
        let (request, binding, receipt) = receipt_fixture();
        let trusted_provider = ProviderIdentity::from_seeds([0x83; 32], [0x84; 32]);
        let attacker = ProviderIdentity::from_seeds([0x85; 32], [0x86; 32]);
        let trusted = trusted_provider.trusted();
        let attacker_envelope = attacker.envelope(receipt);

        assert_eq!(
            verify_xenia_provider_authentication_v1(
                &trusted,
                &request,
                &binding,
                &attacker_envelope,
            )
            .unwrap_err(),
            XeniaProviderVerifierError::VerifierIdentityMismatch
        );
    }

    #[test]
    fn tampered_provider_signature_fails_before_structural_promotion() {
        let (request, binding, receipt) = receipt_fixture();
        let provider = ProviderIdentity::from_seeds([0x87; 32], [0x88; 32]);
        let trusted = provider.trusted();
        let mut envelope = provider.envelope(receipt);
        envelope.ed25519_signature[0] ^= 1;

        assert_eq!(
            verify_xenia_provider_authentication_v1(
                &trusted,
                &request,
                &binding,
                &envelope,
            )
            .unwrap_err(),
            XeniaProviderVerifierError::Ed25519VerificationFailed
        );
    }

    #[test]
    fn provider_valid_but_structurally_wrong_receipt_still_fails() {
        let (request, binding, mut receipt) = receipt_fixture();
        receipt = XeniaVerificationReceiptV1::new(
            digest(0x99),
            receipt.operator_id_commitment().clone(),
            receipt.key_lineage_commitment().clone(),
            receipt.challenge_commitment().clone(),
            receipt.suite(),
            receipt.cryptographic_evidence().clone(),
            receipt.challenge_consumption_evidence().clone(),
            receipt.verifier_state_commitment().clone(),
            receipt.verified_at_unix_secs(),
        );
        let provider = ProviderIdentity::from_seeds([0x89; 32], [0x8a; 32]);
        let trusted = provider.trusted();
        let envelope = provider.envelope(receipt);

        assert!(matches!(
            verify_xenia_provider_authentication_v1(
                &trusted,
                &request,
                &binding,
                &envelope,
            ),
            Err(XeniaProviderVerifierError::XeniaAdapter(
                XeniaAdapterError::RequestMismatch
            ))
        ));
    }

    #[test]
    fn provider_envelope_serde_round_trip_preserves_bytes_and_identity() {
        let (_, _, receipt) = receipt_fixture();
        let provider = ProviderIdentity::from_seeds([0x8b; 32], [0x8c; 32]);
        let envelope = provider.envelope(receipt);
        let json = serde_json::to_vec(&envelope).unwrap();
        let decoded: XeniaProviderAttestedReceiptV1 = serde_json::from_slice(&json).unwrap();
        assert_eq!(decoded, envelope);
    }
}
