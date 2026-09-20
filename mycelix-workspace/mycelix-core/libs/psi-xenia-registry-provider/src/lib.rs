// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002B2B — Xenia registry-provider verification and exact trust-policy join.
//!
//! This crate mirrors the existing Forge/Xenia provider-verification pattern for
//! registry authenticity. It deliberately does not establish registry currentness.
//!
//! Governing theorem:
//!
//! ```text
//! exact B2A structural observation
//! + portable Xenia registry provider envelope
//! + independently trusted Ed25519 key
//! + independently trusted ML-DSA-65 key
//! + both signatures verify over one exact authenticity-only transcript
//! + exact B2A provider/verifier policy join
//!     -> ProjectPolicyTrustedXeniaRegistrySnapshotV1
//!
//! ProjectPolicyTrustedXeniaRegistrySnapshotV1
//!     != registry current
//!     != PSI security established
//!     != contact-discovery composition qualified
//!     != application authority
//! ```

#![forbid(unsafe_code)]

use ed25519_dalek::{Signature as Ed25519Signature, Verifier as _, VerifyingKey};
use ml_dsa::{
    EncodedSignature as MlDsaEncodedSignature, EncodedVerifyingKey as MlDsaEncodedVerifyingKey,
    MlDsa65, Signature as MlDsaSignature, VerifyingKey as MlDsaVerifyingKey,
    signature::Verifier as _,
};
use psi_registry_evidence_core::{
    ProviderClaimDisposition, RegistryEvidenceError, RegistryProviderObservationV1,
    RegistrySnapshotSubjectV1, RegistryTrustPolicyV1,
    StructurallyConsistentRegistryObservationV1, bind_structurally_consistent_observation_v1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest as _, Sha256};
use thiserror::Error;

pub const ML_DSA_65_PK_LEN: usize = 1952;
pub const ML_DSA_65_SIG_LEN: usize = 3309;
pub const XENIA_REGISTRY_PROVIDER_NAMESPACE_V1: &str = "xenia-registry-provider-v1";
pub const XENIA_REGISTRY_VERIFICATION_PROFILE_V1: &str =
    "xenia-mycelix-registry-provider-attestation-v1";

const PROVIDER_ATTESTATION_DOMAIN_V1: &[u8] =
    b"xenia-mycelix-registry-provider-attestation-v1\0";
const VERIFIER_IDENTITY_DOMAIN_V1: &[u8] = b"xenia-mycelix-registry/verifier-identity/v1\0";
const AUTHENTICITY_CLAIM_V1: &[u8] = b"registry-authenticity-established-v1";

/// Portable provider envelope. Deserializing it proves nothing.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct XeniaRegistryProviderAttestedReceiptV1 {
    subject_commitment_sha256: String,
    provider_namespace: String,
    verification_profile: String,
    provider_receipt_commitment_sha256: String,
    verifier_identity_commitment_sha256: String,
    ed25519_signature: Vec<u8>,
    ml_dsa_65_signature: Vec<u8>,
}

impl XeniaRegistryProviderAttestedReceiptV1 {
    pub fn subject_commitment_sha256(&self) -> &str {
        &self.subject_commitment_sha256
    }

    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    pub fn verification_profile(&self) -> &str {
        &self.verification_profile
    }

    pub fn provider_receipt_commitment_sha256(&self) -> &str {
        &self.provider_receipt_commitment_sha256
    }

    pub fn verifier_identity_commitment_sha256(&self) -> &str {
        &self.verifier_identity_commitment_sha256
    }

    pub fn ed25519_signature(&self) -> &[u8] {
        &self.ed25519_signature
    }

    pub fn ml_dsa_65_signature(&self) -> &[u8] {
        &self.ml_dsa_65_signature
    }
}

/// Independently configured Xenia verifier identity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrustedXeniaRegistryVerifierV1 {
    ed25519_pubkey: [u8; 32],
    ml_dsa_65_pubkey: Vec<u8>,
    identity_commitment_sha256: String,
}

impl TrustedXeniaRegistryVerifierV1 {
    pub fn new(
        ed25519_pubkey: [u8; 32],
        ml_dsa_65_pubkey: Vec<u8>,
    ) -> Result<Self, XeniaRegistryProviderError> {
        VerifyingKey::from_bytes(&ed25519_pubkey)
            .map_err(|_| XeniaRegistryProviderError::MalformedTrustedEd25519Key)?;
        if ml_dsa_65_pubkey.len() != ML_DSA_65_PK_LEN {
            return Err(XeniaRegistryProviderError::MalformedTrustedMlDsaKey);
        }
        let identity_commitment_sha256 =
            verifier_identity_commitment_sha256_v1(&ed25519_pubkey, &ml_dsa_65_pubkey)?;
        Ok(Self {
            ed25519_pubkey,
            ml_dsa_65_pubkey,
            identity_commitment_sha256,
        })
    }

    pub fn ed25519_pubkey(&self) -> &[u8; 32] {
        &self.ed25519_pubkey
    }

    pub fn ml_dsa_65_pubkey(&self) -> &[u8] {
        &self.ml_dsa_65_pubkey
    }

    pub fn identity_commitment_sha256(&self) -> &str {
        &self.identity_commitment_sha256
    }
}

/// Positive provider-provenance + project-policy result for one exact registry subject.
///
/// Construction is private and the type intentionally does not implement
/// `Deserialize` or `Default`.
#[derive(Debug, PartialEq, Eq, Serialize)]
pub struct ProjectPolicyTrustedXeniaRegistrySnapshotV1 {
    subject_commitment_sha256: String,
    trust_policy_commitment_sha256: String,
    provider_namespace: String,
    verifier_identity_commitment_sha256: String,
    provider_receipt_commitment_sha256: String,
}

impl ProjectPolicyTrustedXeniaRegistrySnapshotV1 {
    pub fn subject_commitment_sha256(&self) -> &str {
        &self.subject_commitment_sha256
    }

    pub fn trust_policy_commitment_sha256(&self) -> &str {
        &self.trust_policy_commitment_sha256
    }

    pub fn provider_namespace(&self) -> &str {
        &self.provider_namespace
    }

    pub fn verifier_identity_commitment_sha256(&self) -> &str {
        &self.verifier_identity_commitment_sha256
    }

    pub fn provider_receipt_commitment_sha256(&self) -> &str {
        &self.provider_receipt_commitment_sha256
    }

    pub const fn provider_provenance_verified(&self) -> bool {
        true
    }

    pub const fn registry_authenticated_under_policy(&self) -> bool {
        true
    }

    pub const fn registry_current(&self) -> bool {
        false
    }

    pub const fn psi_security_established(&self) -> bool {
        false
    }

    pub const fn contact_discovery_composition_qualified(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

pub fn verifier_identity_commitment_sha256_v1(
    ed25519_pubkey: &[u8; 32],
    ml_dsa_65_pubkey: &[u8],
) -> Result<String, XeniaRegistryProviderError> {
    if ml_dsa_65_pubkey.len() != ML_DSA_65_PK_LEN {
        return Err(XeniaRegistryProviderError::MalformedTrustedMlDsaKey);
    }
    let mut out = Vec::new();
    out.extend_from_slice(VERIFIER_IDENTITY_DOMAIN_V1);
    push_bytes(&mut out, "ed25519_pubkey", ed25519_pubkey)?;
    push_bytes(&mut out, "ml_dsa_65_pubkey", ml_dsa_65_pubkey)?;
    Ok(sha256_hex(&out))
}

/// Exact authenticity-only transcript covered by both Xenia provider signatures.
pub fn provider_attestation_transcript_v1(
    envelope: &XeniaRegistryProviderAttestedReceiptV1,
) -> Result<Vec<u8>, XeniaRegistryProviderError> {
    for (name, value) in [
        ("subject_commitment", envelope.subject_commitment_sha256()),
        ("provider_namespace", envelope.provider_namespace()),
        ("verification_profile", envelope.verification_profile()),
        (
            "provider_receipt_commitment",
            envelope.provider_receipt_commitment_sha256(),
        ),
        (
            "verifier_identity_commitment",
            envelope.verifier_identity_commitment_sha256(),
        ),
    ] {
        if value.trim().is_empty() {
            return Err(XeniaRegistryProviderError::EmptyTranscriptField(name));
        }
    }
    if !is_sha256_hex(envelope.subject_commitment_sha256()) {
        return Err(XeniaRegistryProviderError::MalformedSubjectCommitment);
    }
    if !is_sha256_hex(envelope.provider_receipt_commitment_sha256()) {
        return Err(XeniaRegistryProviderError::MalformedProviderReceiptCommitment);
    }
    if !is_sha256_hex(envelope.verifier_identity_commitment_sha256()) {
        return Err(XeniaRegistryProviderError::MalformedVerifierIdentityCommitment);
    }

    let mut out = Vec::new();
    out.extend_from_slice(PROVIDER_ATTESTATION_DOMAIN_V1);
    push_bytes(
        &mut out,
        "subject_commitment",
        envelope.subject_commitment_sha256().as_bytes(),
    )?;
    push_bytes(
        &mut out,
        "provider_namespace",
        envelope.provider_namespace().as_bytes(),
    )?;
    push_bytes(
        &mut out,
        "verification_profile",
        envelope.verification_profile().as_bytes(),
    )?;
    push_bytes(
        &mut out,
        "provider_receipt_commitment",
        envelope.provider_receipt_commitment_sha256().as_bytes(),
    )?;
    push_bytes(
        &mut out,
        "verifier_identity_commitment",
        envelope.verifier_identity_commitment_sha256().as_bytes(),
    )?;
    push_bytes(&mut out, "claim", AUTHENTICITY_CLAIM_V1)?;
    Ok(out)
}

/// Verify Xenia provider provenance and join it to the exact B2A structural policy result.
pub fn verify_xenia_registry_provider_v1(
    subject: &RegistrySnapshotSubjectV1,
    policy: &RegistryTrustPolicyV1,
    observation: &RegistryProviderObservationV1,
    structural_positive: &StructurallyConsistentRegistryObservationV1,
    trusted_verifier: &TrustedXeniaRegistryVerifierV1,
    envelope: &XeniaRegistryProviderAttestedReceiptV1,
) -> Result<ProjectPolicyTrustedXeniaRegistrySnapshotV1, XeniaRegistryProviderError> {
    let reconstructed = bind_structurally_consistent_observation_v1(subject, policy, observation)?;
    if &reconstructed != structural_positive {
        return Err(XeniaRegistryProviderError::StructuralPositiveMismatch);
    }
    if observation.authenticity != ProviderClaimDisposition::ClaimedPositive {
        return Err(XeniaRegistryProviderError::AuthenticityClaimNotPositive);
    }

    let subject_commitment = subject.commitment_sha256()?;
    if envelope.subject_commitment_sha256() != subject_commitment {
        return Err(XeniaRegistryProviderError::EnvelopeSubjectMismatch);
    }
    if envelope.provider_namespace() != observation.provider_namespace {
        return Err(XeniaRegistryProviderError::ProviderNamespaceMismatch);
    }
    if envelope.verification_profile() != observation.verification_profile {
        return Err(XeniaRegistryProviderError::VerificationProfileMismatch);
    }
    if envelope.provider_receipt_commitment_sha256()
        != observation.provider_receipt_commitment_sha256
    {
        return Err(XeniaRegistryProviderError::ProviderReceiptMismatch);
    }
    if envelope.verifier_identity_commitment_sha256() != observation.verifier_identity_sha256 {
        return Err(XeniaRegistryProviderError::ObservationVerifierIdentityMismatch);
    }
    if envelope.verifier_identity_commitment_sha256()
        != trusted_verifier.identity_commitment_sha256()
    {
        return Err(XeniaRegistryProviderError::TrustedVerifierIdentityMismatch);
    }

    let transcript = provider_attestation_transcript_v1(envelope)?;
    verify_ed25519(trusted_verifier, envelope, &transcript)?;
    verify_ml_dsa_65(trusted_verifier, envelope, &transcript)?;

    Ok(ProjectPolicyTrustedXeniaRegistrySnapshotV1 {
        subject_commitment_sha256: subject_commitment,
        trust_policy_commitment_sha256: policy.commitment_sha256()?,
        provider_namespace: envelope.provider_namespace().to_owned(),
        verifier_identity_commitment_sha256: trusted_verifier
            .identity_commitment_sha256()
            .to_owned(),
        provider_receipt_commitment_sha256: envelope
            .provider_receipt_commitment_sha256()
            .to_owned(),
    })
}

fn verify_ed25519(
    trusted: &TrustedXeniaRegistryVerifierV1,
    envelope: &XeniaRegistryProviderAttestedReceiptV1,
    transcript: &[u8],
) -> Result<(), XeniaRegistryProviderError> {
    let bytes: [u8; 64] = envelope
        .ed25519_signature()
        .try_into()
        .map_err(|_| XeniaRegistryProviderError::MalformedEd25519Signature)?;
    let key = VerifyingKey::from_bytes(trusted.ed25519_pubkey())
        .map_err(|_| XeniaRegistryProviderError::MalformedTrustedEd25519Key)?;
    key.verify(transcript, &Ed25519Signature::from_bytes(&bytes))
        .map_err(|_| XeniaRegistryProviderError::Ed25519VerificationFailed)
}

fn verify_ml_dsa_65(
    trusted: &TrustedXeniaRegistryVerifierV1,
    envelope: &XeniaRegistryProviderAttestedReceiptV1,
    transcript: &[u8],
) -> Result<(), XeniaRegistryProviderError> {
    let key_bytes: [u8; ML_DSA_65_PK_LEN] = trusted
        .ml_dsa_65_pubkey()
        .try_into()
        .map_err(|_| XeniaRegistryProviderError::MalformedTrustedMlDsaKey)?;
    let encoded_key = MlDsaEncodedVerifyingKey::<MlDsa65>::try_from(key_bytes.as_slice())
        .map_err(|_| XeniaRegistryProviderError::MalformedTrustedMlDsaKey)?;
    let key = MlDsaVerifyingKey::<MlDsa65>::decode(&encoded_key);

    let signature_bytes: [u8; ML_DSA_65_SIG_LEN] = envelope
        .ml_dsa_65_signature()
        .try_into()
        .map_err(|_| XeniaRegistryProviderError::MalformedMlDsaSignature)?;
    let encoded_signature =
        MlDsaEncodedSignature::<MlDsa65>::try_from(signature_bytes.as_slice())
            .map_err(|_| XeniaRegistryProviderError::MalformedMlDsaSignature)?;
    let signature = MlDsaSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(XeniaRegistryProviderError::MalformedMlDsaSignature)?;
    key.verify(transcript, &signature)
        .map_err(|_| XeniaRegistryProviderError::MlDsaVerificationFailed)
}

fn push_bytes(
    out: &mut Vec<u8>,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), XeniaRegistryProviderError> {
    let len = u32::try_from(bytes.len()).map_err(|_| XeniaRegistryProviderError::FieldTooLarge {
        field,
        len: bytes.len(),
    })?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
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

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum XeniaRegistryProviderError {
    #[error("malformed trusted Ed25519 key")]
    MalformedTrustedEd25519Key,
    #[error("malformed trusted ML-DSA-65 key")]
    MalformedTrustedMlDsaKey,
    #[error("malformed Ed25519 signature")]
    MalformedEd25519Signature,
    #[error("malformed ML-DSA-65 signature")]
    MalformedMlDsaSignature,
    #[error("Ed25519 provider signature verification failed")]
    Ed25519VerificationFailed,
    #[error("ML-DSA-65 provider signature verification failed")]
    MlDsaVerificationFailed,
    #[error("structural positive does not match exact subject/policy/observation join")]
    StructuralPositiveMismatch,
    #[error("raw provider observation does not claim authenticity")]
    AuthenticityClaimNotPositive,
    #[error("provider envelope subject mismatch")]
    EnvelopeSubjectMismatch,
    #[error("provider namespace mismatch")]
    ProviderNamespaceMismatch,
    #[error("verification profile mismatch")]
    VerificationProfileMismatch,
    #[error("provider receipt commitment mismatch")]
    ProviderReceiptMismatch,
    #[error("provider envelope verifier identity differs from structural observation")]
    ObservationVerifierIdentityMismatch,
    #[error("provider envelope verifier identity differs from independently trusted verifier")]
    TrustedVerifierIdentityMismatch,
    #[error("malformed registry subject commitment")]
    MalformedSubjectCommitment,
    #[error("malformed provider receipt commitment")]
    MalformedProviderReceiptCommitment,
    #[error("malformed verifier identity commitment")]
    MalformedVerifierIdentityCommitment,
    #[error("empty transcript field: {0}")]
    EmptyTranscriptField(&'static str),
    #[error("canonical field {field} too large: {len}")]
    FieldTooLarge { field: &'static str, len: usize },
    #[error("registry structural evidence rejected: {0:?}")]
    Structural(RegistryEvidenceError),
}

impl From<RegistryEvidenceError> for XeniaRegistryProviderError {
    fn from(value: RegistryEvidenceError) -> Self {
        Self::Structural(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey};
    use ml_dsa::{
        B32, Signature as MlDsaSignatureT, SigningKey as MlDsaSigningKey,
        signature::{Keypair as _, Signer as _},
    };
    use psi_registry_evidence_core::TrustedProviderVerifierV1;

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

        fn trusted(&self) -> TrustedXeniaRegistryVerifierV1 {
            TrustedXeniaRegistryVerifierV1::new(
                self.ed.verifying_key().to_bytes(),
                self.ml.verifying_key().encode().as_slice().to_vec(),
            )
            .unwrap()
        }

        fn envelope(
            &self,
            subject: &RegistrySnapshotSubjectV1,
            receipt_sha256: String,
        ) -> XeniaRegistryProviderAttestedReceiptV1 {
            let trusted = self.trusted();
            let mut envelope = XeniaRegistryProviderAttestedReceiptV1 {
                subject_commitment_sha256: subject.commitment_sha256().unwrap(),
                provider_namespace: XENIA_REGISTRY_PROVIDER_NAMESPACE_V1.into(),
                verification_profile: XENIA_REGISTRY_VERIFICATION_PROFILE_V1.into(),
                provider_receipt_commitment_sha256: receipt_sha256,
                verifier_identity_commitment_sha256: trusted
                    .identity_commitment_sha256()
                    .into(),
                ed25519_signature: vec![],
                ml_dsa_65_signature: vec![],
            };
            let transcript = provider_attestation_transcript_v1(&envelope).unwrap();
            envelope.ed25519_signature = self.ed.sign(&transcript).to_bytes().to_vec();
            let signature: MlDsaSignatureT<MlDsa65> = self.ml.sign(&transcript);
            envelope.ml_dsa_65_signature = signature.encode().as_slice().to_vec();
            envelope
        }
    }

    fn subject(snapshot_byte: &str) -> RegistrySnapshotSubjectV1 {
        RegistrySnapshotSubjectV1 {
            service_domain: "contacts.mycelix.test".into(),
            snapshot_sha256: snapshot_byte.repeat(32),
            registry_epoch: "registry-19".into(),
            sequence: 0,
            previous_snapshot_commitment_sha256: None,
            voprf_key_epoch: "key-7".into(),
            voprf_backend_profile: "rfc9497-ristretto255-sha512-voprf-tagged-set-v1".into(),
            canonicalization_profile: "ascii-trim-lower-synthetic-contact-v1".into(),
            equality_domain: "synthetic-contact-id-ascii-lower-v1".into(),
            construction_profile: "voprf-tagged-set-v1".into(),
        }
    }

    fn policy(
        trusted: &TrustedXeniaRegistryVerifierV1,
        sequence: u64,
        previous: Option<String>,
    ) -> RegistryTrustPolicyV1 {
        RegistryTrustPolicyV1::new(
            "contacts.mycelix.test",
            sequence,
            previous,
            vec![TrustedProviderVerifierV1 {
                provider_namespace: XENIA_REGISTRY_PROVIDER_NAMESPACE_V1.into(),
                verifier_identity_sha256: trusted.identity_commitment_sha256().into(),
            }],
        )
        .unwrap()
    }

    fn observation(
        subject: &RegistrySnapshotSubjectV1,
        trusted: &TrustedXeniaRegistryVerifierV1,
        receipt_sha256: &str,
    ) -> RegistryProviderObservationV1 {
        RegistryProviderObservationV1 {
            subject_commitment_sha256: subject.commitment_sha256().unwrap(),
            provider_namespace: XENIA_REGISTRY_PROVIDER_NAMESPACE_V1.into(),
            verifier_identity_sha256: trusted.identity_commitment_sha256().into(),
            verification_profile: XENIA_REGISTRY_VERIFICATION_PROFILE_V1.into(),
            provider_receipt_commitment_sha256: receipt_sha256.into(),
            observed_sequence: subject.sequence,
            authenticity: ProviderClaimDisposition::ClaimedPositive,
            currentness: ProviderClaimDisposition::ClaimedPositive,
        }
    }

    #[test]
    fn trusted_hybrid_provider_envelope_yields_policy_trusted_authenticity_only() {
        let provider = ProviderIdentity::from_seeds([0x41; 32], [0x42; 32]);
        let trusted = provider.trusted();
        let subject = subject("11");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let observation = observation(&subject, &trusted, &receipt);
        let structural =
            bind_structurally_consistent_observation_v1(&subject, &policy, &observation).unwrap();
        let envelope = provider.envelope(&subject, receipt);

        let positive = verify_xenia_registry_provider_v1(
            &subject,
            &policy,
            &observation,
            &structural,
            &trusted,
            &envelope,
        )
        .unwrap();

        assert!(positive.provider_provenance_verified());
        assert!(positive.registry_authenticated_under_policy());
        assert!(!positive.registry_current());
        assert!(!positive.psi_security_established());
        assert!(!positive.contact_discovery_composition_qualified());
        assert!(!positive.application_authority_granted());
    }

    #[test]
    fn self_signed_attacker_cannot_replace_independently_trusted_verifier() {
        let provider = ProviderIdentity::from_seeds([0x43; 32], [0x44; 32]);
        let attacker = ProviderIdentity::from_seeds([0x45; 32], [0x46; 32]);
        let trusted = provider.trusted();
        let subject = subject("11");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let observation = observation(&subject, &trusted, &receipt);
        let structural =
            bind_structurally_consistent_observation_v1(&subject, &policy, &observation).unwrap();
        let envelope = attacker.envelope(&subject, receipt);

        assert_eq!(
            verify_xenia_registry_provider_v1(
                &subject,
                &policy,
                &observation,
                &structural,
                &trusted,
                &envelope,
            )
            .unwrap_err(),
            XeniaRegistryProviderError::ObservationVerifierIdentityMismatch
        );
    }

    #[test]
    fn tampered_ed25519_signature_is_rejected() {
        let provider = ProviderIdentity::from_seeds([0x47; 32], [0x48; 32]);
        let trusted = provider.trusted();
        let subject = subject("11");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let observation = observation(&subject, &trusted, &receipt);
        let structural =
            bind_structurally_consistent_observation_v1(&subject, &policy, &observation).unwrap();
        let mut envelope = provider.envelope(&subject, receipt);
        envelope.ed25519_signature[0] ^= 1;

        assert_eq!(
            verify_xenia_registry_provider_v1(
                &subject,
                &policy,
                &observation,
                &structural,
                &trusted,
                &envelope,
            )
            .unwrap_err(),
            XeniaRegistryProviderError::Ed25519VerificationFailed
        );
    }

    #[test]
    fn provider_evidence_cannot_be_borrowed_across_registry_subjects() {
        let provider = ProviderIdentity::from_seeds([0x49; 32], [0x4a; 32]);
        let trusted = provider.trusted();
        let subject_a = subject("11");
        let subject_b = subject("22");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let observation_b = observation(&subject_b, &trusted, &receipt);
        let structural_b = bind_structurally_consistent_observation_v1(
            &subject_b,
            &policy,
            &observation_b,
        )
        .unwrap();
        let envelope_a = provider.envelope(&subject_a, receipt);

        assert_eq!(
            verify_xenia_registry_provider_v1(
                &subject_b,
                &policy,
                &observation_b,
                &structural_b,
                &trusted,
                &envelope_a,
            )
            .unwrap_err(),
            XeniaRegistryProviderError::EnvelopeSubjectMismatch
        );
    }

    #[test]
    fn old_policy_trust_cannot_be_borrowed_after_verifier_rotation() {
        let old_provider = ProviderIdentity::from_seeds([0x4b; 32], [0x4c; 32]);
        let new_provider = ProviderIdentity::from_seeds([0x4d; 32], [0x4e; 32]);
        let old_trusted = old_provider.trusted();
        let new_trusted = new_provider.trusted();
        let subject = subject("11");
        let old_policy = policy(&old_trusted, 0, None);
        let old_commitment = old_policy.commitment_sha256().unwrap();
        let new_policy = policy(&new_trusted, 1, Some(old_commitment));
        let receipt = "55".repeat(32);
        let old_observation = observation(&subject, &old_trusted, &receipt);

        assert!(matches!(
            bind_structurally_consistent_observation_v1(
                &subject,
                &new_policy,
                &old_observation,
            ),
            Err(RegistryEvidenceError::VerifierNotTrustedForProvider)
        ));
    }

    #[test]
    fn claimed_currentness_is_never_promoted_by_provider_authenticity() {
        let provider = ProviderIdentity::from_seeds([0x4f; 32], [0x50; 32]);
        let trusted = provider.trusted();
        let subject = subject("11");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let observation = observation(&subject, &trusted, &receipt);
        let structural =
            bind_structurally_consistent_observation_v1(&subject, &policy, &observation).unwrap();
        let envelope = provider.envelope(&subject, receipt);
        let positive = verify_xenia_registry_provider_v1(
            &subject,
            &policy,
            &observation,
            &structural,
            &trusted,
            &envelope,
        )
        .unwrap();
        assert!(!positive.registry_current());
    }

    #[test]
    fn non_positive_authenticity_claim_is_rejected_before_crypto_promotion() {
        let provider = ProviderIdentity::from_seeds([0x51; 32], [0x52; 32]);
        let trusted = provider.trusted();
        let subject = subject("11");
        let policy = policy(&trusted, 0, None);
        let receipt = "55".repeat(32);
        let mut observation = observation(&subject, &trusted, &receipt);
        observation.authenticity = ProviderClaimDisposition::Unspecified;
        let structural =
            bind_structurally_consistent_observation_v1(&subject, &policy, &observation).unwrap();
        let envelope = provider.envelope(&subject, receipt);

        assert_eq!(
            verify_xenia_registry_provider_v1(
                &subject,
                &policy,
                &observation,
                &structural,
                &trusted,
                &envelope,
            )
            .unwrap_err(),
            XeniaRegistryProviderError::AuthenticityClaimNotPositive
        );
    }

    #[test]
    fn portable_envelope_round_trip_does_not_construct_positive_type() {
        let provider = ProviderIdentity::from_seeds([0x53; 32], [0x54; 32]);
        let subject = subject("11");
        let envelope = provider.envelope(&subject, "55".repeat(32));
        let bytes = serde_json::to_vec(&envelope).unwrap();
        let decoded: XeniaRegistryProviderAttestedReceiptV1 =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, envelope);
    }
}
