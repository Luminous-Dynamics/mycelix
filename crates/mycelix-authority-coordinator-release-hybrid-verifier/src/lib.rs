// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Native hybrid signature verification for Mycelix coordinator release manifests.
//!
//! This crate sits after manifest-bound key authorization and immediately before
//! #269 release authentication. It accepts no caller-selected trusted public key.
//! The only key material it can use comes from non-deserializable
//! `QualifiedCoordinatorReleaseSigningKey`, which is already bound to one exact
//! candidate release manifest and one independently current key policy.
//!
//! Both Ed25519 and ML-DSA-65 signatures must verify over the identical fixed
//! message. Verification time is sampled only after both cryptographic checks pass.
//! The evidence-shaped #269 signature proof is constructed privately and consumed
//! locally; the public live API returns only #269's non-deserializable authenticated
//! release qualification.

use ed25519_dalek::{Signature as EdSignature, VerifyingKey as EdVerifyingKey};
use ml_dsa::signature::Verifier as MlVerifier;
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, KeyInit as _, MlDsa65,
    Signature as MlSignature, VerifyingKey as MlVerifyingKey,
};
use mycelix_authority_coordinator_release::{
    CoordinatorReleaseManifest, MANIFEST_PROFILE, QualifiedCoordinatorReleaseRequirement,
    SIGNATURE_PROOF_PROTOCOL, VerifiedCoordinatorReleaseSignatureProof,
    qualify_coordinator_release,
};
use mycelix_authority_coordinator_release_key_policy::{
    POLICY_PROFILE, QualifiedCoordinatorReleaseSigningKey,
};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-release-hybrid-verifier-v0.1";
pub const SIGNATURE_EVIDENCE_PROFILE: &str =
    "mycelix-authority-coordinator-release-hybrid-signature-v1-ed25519-64-ml-dsa-65-3309";
pub const SIGNATURE_MESSAGE_PROFILE: &str =
    "mycelix-authority-coordinator-release-signature-message-v1-blake3-subject";
pub const SIGNATURE_REF_PROFILE: &str =
    "mycelix-authority-coordinator-release-signature-ref-v1-blake3";
pub const VERIFIER_REF: &str =
    "mycelix-authority-coordinator-release-hybrid-verifier:v0.1:rustcrypto";
pub const ED25519_SIGNATURE_LEN: usize = 64;
pub const ML_DSA_65_SIGNATURE_LEN: usize = 3309;
pub const MAX_SIGNATURE_PROOF_REUSE_MS: u64 = 5_000;

const MESSAGE_DOMAIN: &[u8] = b"mycelix/authority/coordinator-release-signature/v1\0";
const SIGNATURE_REF_DOMAIN: &[u8] = b"mycelix/authority/coordinator-release-signature-ref/v1";
const MAX_TEXT_BYTES: usize = 2048;

/// Detached signature material for one exact release/key-id message.
///
/// This object is intentionally deserializable input. It contains no public key,
/// caller-selected proof time, proof horizon or signature reference. Those positive
/// evidence fields remain verifier-owned.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorReleaseHybridSignature {
    pub protocol_version: String,
    pub signature_profile: String,
    pub signing_key_id: String,
    pub ed25519_signature: Vec<u8>,
    pub ml_dsa_65_signature: Vec<u8>,
}

impl CoordinatorReleaseHybridSignature {
    pub fn validate(&self) -> Result<(), CoordinatorReleaseHybridVerifierError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoordinatorReleaseHybridVerifierError::WrongProtocol);
        }
        if self.signature_profile != SIGNATURE_EVIDENCE_PROFILE {
            return Err(CoordinatorReleaseHybridVerifierError::WrongSignatureProfile);
        }
        validate_text(&self.signing_key_id, "signing key id")?;
        if self.ed25519_signature.len() != ED25519_SIGNATURE_LEN {
            return Err(CoordinatorReleaseHybridVerifierError::InvalidEd25519SignatureLength);
        }
        if self.ml_dsa_65_signature.len() != ML_DSA_65_SIGNATURE_LEN {
            return Err(CoordinatorReleaseHybridVerifierError::InvalidMlDsa65SignatureLength);
        }
        Ok(())
    }

    pub fn evidence_digest(&self) -> Result<[u8; 32], CoordinatorReleaseHybridVerifierError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(SIGNATURE_REF_DOMAIN);
        hash_frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        hash_frame(&mut hasher, SIGNATURE_REF_PROFILE.as_bytes());
        hash_frame(&mut hasher, SIGNATURE_EVIDENCE_PROFILE.as_bytes());
        hash_frame(&mut hasher, self.signing_key_id.as_bytes());
        hash_frame(&mut hasher, &self.ed25519_signature);
        hash_frame(&mut hasher, &self.ml_dsa_65_signature);
        Ok(*hasher.finalize().as_bytes())
    }

    fn canonical_signature_ref(&self) -> Result<String, CoordinatorReleaseHybridVerifierError> {
        Ok(format!(
            "hybrid-signature-blake3:{}",
            encode_hex(&self.evidence_digest()?)
        ))
    }
}

/// Deterministic bytes signed by both algorithms for one candidate release and one
/// authorized key id.
///
/// The #269 manifest digest already commits the complete release semantics. The
/// explicit protocol, signature profile, authority and policy echoes below prevent
/// this domain from being repurposed as a generic signature over an opaque digest.
pub fn release_signature_message(
    manifest: &CoordinatorReleaseManifest,
    signing_key_id: &str,
) -> Result<Vec<u8>, CoordinatorReleaseHybridVerifierError> {
    manifest
        .validate()
        .map_err(|error| CoordinatorReleaseHybridVerifierError::CandidateManifest(error.to_string()))?;
    validate_text(signing_key_id, "signing key id")?;
    let manifest_digest = manifest
        .manifest_digest()
        .map_err(|error| CoordinatorReleaseHybridVerifierError::CandidateManifest(error.to_string()))?;

    let mut message = Vec::with_capacity(512);
    message.extend_from_slice(MESSAGE_DOMAIN);
    append_frame(&mut message, PROTOCOL_VERSION.as_bytes());
    append_frame(&mut message, SIGNATURE_MESSAGE_PROFILE.as_bytes());
    append_frame(&mut message, SIGNATURE_EVIDENCE_PROFILE.as_bytes());
    append_frame(&mut message, &manifest_digest);
    append_frame(&mut message, MANIFEST_PROFILE.as_bytes());
    append_frame(&mut message, manifest.release_authority_ref.as_bytes());
    append_frame(&mut message, &manifest.release_policy_digest);
    append_frame(&mut message, manifest.release_policy_profile.as_bytes());
    append_frame(&mut message, signing_key_id.as_bytes());
    Ok(message)
}

/// Authenticate one candidate coordinator release with an exact manifest-bound
/// hybrid signing-key capability.
///
/// The public live API never returns the deserializable signature proof. It builds
/// that proof privately only after both signature components succeed and immediately
/// consumes it through #269, returning the non-deserializable authenticated release.
pub fn authenticate_coordinator_release_hybrid(
    manifest: CoordinatorReleaseManifest,
    qualified_key: &QualifiedCoordinatorReleaseSigningKey,
    signature: &CoordinatorReleaseHybridSignature,
) -> Result<QualifiedCoordinatorReleaseRequirement, CoordinatorReleaseHybridVerifierError> {
    let proof = verify_and_build_signature_proof(&manifest, qualified_key, signature)?;
    qualify_coordinator_release(manifest, &proof, proof.verified_at_ms)
        .map_err(|error| CoordinatorReleaseHybridVerifierError::ReleaseQualification(error.to_string()))
}

fn verify_and_build_signature_proof(
    manifest: &CoordinatorReleaseManifest,
    qualified_key: &QualifiedCoordinatorReleaseSigningKey,
    signature: &CoordinatorReleaseHybridSignature,
) -> Result<VerifiedCoordinatorReleaseSignatureProof, CoordinatorReleaseHybridVerifierError> {
    signature.validate()?;
    manifest
        .validate()
        .map_err(|error| CoordinatorReleaseHybridVerifierError::CandidateManifest(error.to_string()))?;

    let manifest_digest = manifest
        .manifest_digest()
        .map_err(|error| CoordinatorReleaseHybridVerifierError::CandidateManifest(error.to_string()))?;
    if qualified_key.manifest_digest() != manifest_digest
        || qualified_key.manifest_profile() != MANIFEST_PROFILE
    {
        return Err(CoordinatorReleaseHybridVerifierError::ManifestBindingMismatch);
    }
    if manifest.release_authority_ref != qualified_key.release_authority_ref()
        || manifest.release_policy_digest != qualified_key.release_policy_digest()
        || manifest.release_policy_profile != qualified_key.release_policy_profile()
        || manifest.release_policy_profile != POLICY_PROFILE
    {
        return Err(CoordinatorReleaseHybridVerifierError::PolicyBindingMismatch);
    }
    if signature.signing_key_id != qualified_key.key_id() {
        return Err(CoordinatorReleaseHybridVerifierError::SigningKeyIdMismatch);
    }

    let message = release_signature_message(manifest, qualified_key.key_id())?;
    verify_ed25519(
        qualified_key.ed25519_public_key(),
        &message,
        &signature.ed25519_signature,
    )?;
    verify_ml_dsa_65(
        qualified_key.ml_dsa_65_public_key(),
        &message,
        &signature.ml_dsa_65_signature,
    )?;

    // Sample time only after both cryptographic checks have succeeded. A key that
    // expires while crypto is executing therefore fails closed below.
    let verified_at_ms = system_time_ms()?;
    if qualified_key.verified_at_ms() > verified_at_ms
        || qualified_key.valid_until_ms() <= verified_at_ms
        || manifest.valid_from_ms > verified_at_ms
        || manifest.valid_until_ms <= verified_at_ms
    {
        return Err(CoordinatorReleaseHybridVerifierError::QualifiedKeyNotLiveAfterVerification);
    }

    let local_cap = verified_at_ms
        .checked_add(MAX_SIGNATURE_PROOF_REUSE_MS)
        .ok_or(CoordinatorReleaseHybridVerifierError::ClockOverflow)?;
    let valid_until_ms = qualified_key
        .valid_until_ms()
        .min(manifest.valid_until_ms)
        .min(local_cap);
    if valid_until_ms <= verified_at_ms {
        return Err(CoordinatorReleaseHybridVerifierError::EmptyProofWindow);
    }

    let proof = VerifiedCoordinatorReleaseSignatureProof {
        protocol_version: SIGNATURE_PROOF_PROTOCOL.into(),
        manifest_digest,
        manifest_profile: MANIFEST_PROFILE.into(),
        release_authority_ref: manifest.release_authority_ref.clone(),
        release_policy_digest: manifest.release_policy_digest,
        release_policy_profile: manifest.release_policy_profile.clone(),
        signing_key_id: qualified_key.key_id().into(),
        signature_ref: signature.canonical_signature_ref()?,
        verifier_ref: VERIFIER_REF.into(),
        verified_at_ms,
        valid_until_ms,
    };
    proof
        .validate_at(verified_at_ms)
        .map_err(|error| CoordinatorReleaseHybridVerifierError::ProofShape(error.to_string()))?;
    Ok(proof)
}

fn verify_ed25519(
    key: &[u8],
    message: &[u8],
    signature: &[u8],
) -> Result<(), CoordinatorReleaseHybridVerifierError> {
    let key_bytes: [u8; 32] = key
        .try_into()
        .map_err(|_| CoordinatorReleaseHybridVerifierError::InvalidEd25519PublicKeyLength)?;
    let signature_bytes: [u8; ED25519_SIGNATURE_LEN] = signature
        .try_into()
        .map_err(|_| CoordinatorReleaseHybridVerifierError::InvalidEd25519SignatureLength)?;
    let verifying_key = EdVerifyingKey::from_bytes(&key_bytes)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::InvalidEd25519PublicKey)?;
    let signature = EdSignature::from_bytes(&signature_bytes);
    verifying_key
        .verify_strict(message, &signature)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::Ed25519VerificationFailed)
}

fn verify_ml_dsa_65(
    key: &[u8],
    message: &[u8],
    signature: &[u8],
) -> Result<(), CoordinatorReleaseHybridVerifierError> {
    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(key)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::InvalidMlDsa65PublicKeyLength)?;
    let verifying_key = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);
    let encoded_signature = EncodedSignature::<MlDsa65>::try_from(signature)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::InvalidMlDsa65SignatureLength)?;
    let signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or(CoordinatorReleaseHybridVerifierError::InvalidMlDsa65Signature)?;
    verifying_key
        .verify(message, &signature)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::MlDsa65VerificationFailed)
}

fn system_time_ms() -> Result<u64, CoordinatorReleaseHybridVerifierError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| CoordinatorReleaseHybridVerifierError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| CoordinatorReleaseHybridVerifierError::ClockOverflow)
}

fn append_frame(target: &mut Vec<u8>, bytes: &[u8]) {
    target.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    target.extend_from_slice(bytes);
}

fn hash_frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn validate_text(
    value: &str,
    field: &'static str,
) -> Result<(), CoordinatorReleaseHybridVerifierError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(CoordinatorReleaseHybridVerifierError::InvalidText(field))
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorReleaseHybridVerifierError {
    WrongProtocol,
    WrongSignatureProfile,
    InvalidText(&'static str),
    CandidateManifest(String),
    ManifestBindingMismatch,
    PolicyBindingMismatch,
    SigningKeyIdMismatch,
    InvalidEd25519PublicKeyLength,
    InvalidEd25519PublicKey,
    InvalidEd25519SignatureLength,
    Ed25519VerificationFailed,
    InvalidMlDsa65PublicKeyLength,
    InvalidMlDsa65SignatureLength,
    InvalidMlDsa65Signature,
    MlDsa65VerificationFailed,
    QualifiedKeyNotLiveAfterVerification,
    ClockBeforeUnixEpoch,
    ClockOverflow,
    EmptyProofWindow,
    ProofShape(String),
    ReleaseQualification(String),
}

impl fmt::Display for CoordinatorReleaseHybridVerifierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator release hybrid-verifier protocol"),
            Self::WrongSignatureProfile => {
                write!(f, "wrong coordinator release hybrid-signature profile")
            }
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::CandidateManifest(error) => {
                write!(f, "invalid candidate release manifest: {error}")
            }
            Self::ManifestBindingMismatch => {
                write!(f, "qualified signing key is bound to another release manifest")
            }
            Self::PolicyBindingMismatch => write!(
                f,
                "qualified signing key and candidate manifest disagree on release policy or authority"
            ),
            Self::SigningKeyIdMismatch => {
                write!(f, "detached signature names another signing key id")
            }
            Self::InvalidEd25519PublicKeyLength => {
                write!(f, "qualified Ed25519 public key has the wrong length")
            }
            Self::InvalidEd25519PublicKey => write!(f, "qualified Ed25519 public key is invalid"),
            Self::InvalidEd25519SignatureLength => {
                write!(f, "Ed25519 signature must be exactly 64 bytes")
            }
            Self::Ed25519VerificationFailed => {
                write!(f, "Ed25519 release signature verification failed")
            }
            Self::InvalidMlDsa65PublicKeyLength => {
                write!(f, "qualified ML-DSA-65 public key has the wrong length")
            }
            Self::InvalidMlDsa65SignatureLength => {
                write!(f, "ML-DSA-65 signature must be exactly 3309 bytes")
            }
            Self::InvalidMlDsa65Signature => {
                write!(f, "ML-DSA-65 release signature could not be decoded")
            }
            Self::MlDsa65VerificationFailed => {
                write!(f, "ML-DSA-65 release signature verification failed")
            }
            Self::QualifiedKeyNotLiveAfterVerification => write!(
                f,
                "manifest-bound signing key is not live after cryptographic verification"
            ),
            Self::ClockBeforeUnixEpoch => write!(f, "system clock is before the Unix epoch"),
            Self::ClockOverflow => write!(f, "hybrid verifier clock overflow"),
            Self::EmptyProofWindow => {
                write!(f, "hybrid signature proof has no reusable live window")
            }
            Self::ProofShape(error) => {
                write!(f, "constructed #269 signature proof failed validation: {error}")
            }
            Self::ReleaseQualification(error) => {
                write!(f, "#269 release qualification denied verified signature: {error}")
            }
        }
    }
}

impl std::error::Error for CoordinatorReleaseHybridVerifierError {}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer as _, SigningKey as EdSigningKey};
    use ml_dsa::signature::{Keypair as _, Signer as MlSigner};
    use ml_dsa::{Generate as _, KeyExport as _, SigningKey as MlSigningKey};
    use mycelix_authority_coordinator_deployment::CoordinatorCodeIdentity;
    use mycelix_authority_coordinator_release::PROTOCOL_VERSION as RELEASE_PROTOCOL_VERSION;
    use mycelix_authority_coordinator_release_key_policy::{
        AuthorizedHybridReleaseKey, CURRENT_POLICY_PROOF_PROTOCOL, CoordinatorReleaseKeyPolicy,
        ED25519_PUBLIC_KEY_LEN, ML_DSA_65_PUBLIC_KEY_LEN,
        PROTOCOL_VERSION as KEY_POLICY_PROTOCOL, VerifiedCurrentCoordinatorReleaseKeyPolicyProof,
        qualify_manifest_key_policy,
    };

    struct TestSigner {
        ed: EdSigningKey,
        ml: MlSigningKey<MlDsa65>,
    }

    impl TestSigner {
        fn new() -> Self {
            Self {
                ed: EdSigningKey::from_bytes(&[42; 32]),
                ml: MlSigningKey::<MlDsa65>::generate(),
            }
        }

        fn authorized_key(&self) -> AuthorizedHybridReleaseKey {
            let now = system_time_ms().unwrap();
            AuthorizedHybridReleaseKey {
                key_id: "release-key-1".into(),
                ed25519_public_key: self.ed.verifying_key().to_bytes().to_vec(),
                ml_dsa_65_public_key: self.ml.verifying_key().encode().as_slice().to_vec(),
                valid_from_ms: now.saturating_sub(1_000).max(1),
                valid_until_ms: now + 60_000,
            }
        }

        fn sign(&self, message: &[u8]) -> (Vec<u8>, Vec<u8>) {
            let ed = self.ed.sign(message).to_bytes().to_vec();
            let ml = self.ml.sign(message).encode().as_slice().to_vec();
            (ed, ml)
        }
    }

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn fixture(
        signer: &TestSigner,
    ) -> (
        CoordinatorReleaseManifest,
        QualifiedCoordinatorReleaseSigningKey,
    ) {
        let now = system_time_ms().unwrap();
        let policy = CoordinatorReleaseKeyPolicy {
            protocol_version: KEY_POLICY_PROTOCOL.into(),
            policy_id: "coordinator-release-keys".into(),
            policy_generation: 3,
            release_authority_ref: "release-authority:1".into(),
            authorized_keys: vec![signer.authorized_key()],
            valid_from_ms: now.saturating_sub(1_000).max(1),
            valid_until_ms: now + 60_000,
        };
        let manifest = CoordinatorReleaseManifest {
            protocol_version: RELEASE_PROTOCOL_VERSION.into(),
            release_id: "release-1".into(),
            release_version: 1,
            dna_hash_raw_39: h(9),
            coordinators: vec![CoordinatorCodeIdentity {
                zome_name: "authority_current_freshness_verifier".into(),
                wasm_hash_raw_39: h(1),
            }],
            dna_bundle_digest: d(3),
            source_tree_digest: d(4),
            lockfile_digest: d(5),
            toolchain_digest: d(6),
            build_recipe_digest: d(7),
            sbom_digest: d(8),
            source_ref: "git:tree:1".into(),
            build_ref: "nix:build:1".into(),
            release_authority_ref: policy.release_authority_ref.clone(),
            release_policy_digest: policy.policy_digest().unwrap(),
            release_policy_profile: POLICY_PROFILE.into(),
            valid_from_ms: now.saturating_sub(1_000).max(1),
            valid_until_ms: now + 60_000,
        };
        let currentness = VerifiedCurrentCoordinatorReleaseKeyPolicyProof {
            protocol_version: CURRENT_POLICY_PROOF_PROTOCOL.into(),
            policy_digest: policy.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
            policy_generation: policy.policy_generation,
            source_ref: "release-key-policy:head:3".into(),
            verifier_ref: "release-key-policy-verifier:test".into(),
            verified_at_ms: now.saturating_sub(100).max(1),
            valid_until_ms: now + 20_000,
        };
        let key = qualify_manifest_key_policy(&manifest, policy, &currentness, now)
            .unwrap()
            .qualify_key("release-key-1", now)
            .unwrap();
        assert_eq!(key.ed25519_public_key().len(), ED25519_PUBLIC_KEY_LEN);
        assert_eq!(key.ml_dsa_65_public_key().len(), ML_DSA_65_PUBLIC_KEY_LEN);
        (manifest, key)
    }

    fn signed_fixture(
        signer: &TestSigner,
        manifest: &CoordinatorReleaseManifest,
        key: &QualifiedCoordinatorReleaseSigningKey,
    ) -> CoordinatorReleaseHybridSignature {
        let message = release_signature_message(manifest, key.key_id()).unwrap();
        let (ed25519_signature, ml_dsa_65_signature) = signer.sign(&message);
        CoordinatorReleaseHybridSignature {
            protocol_version: PROTOCOL_VERSION.into(),
            signature_profile: SIGNATURE_EVIDENCE_PROFILE.into(),
            signing_key_id: key.key_id().into(),
            ed25519_signature,
            ml_dsa_65_signature,
        }
    }

    #[test]
    fn valid_hybrid_signature_returns_only_authenticated_release() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let signature = signed_fixture(&signer, &manifest, &key);
        let expected_manifest_digest = manifest.manifest_digest().unwrap();
        let qualified = authenticate_coordinator_release_hybrid(manifest, &key, &signature).unwrap();

        assert_eq!(qualified.manifest_digest(), expected_manifest_digest);
        assert_eq!(qualified.verifier_ref(), VERIFIER_REF);
        assert!(qualified.valid_until_ms() > qualified.verified_at_ms());
        assert!(qualified.valid_until_ms() - qualified.verified_at_ms() <= MAX_SIGNATURE_PROOF_REUSE_MS);
    }

    #[test]
    fn either_forged_signature_half_denies() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);

        let mut ed_forged = signed_fixture(&signer, &manifest, &key);
        ed_forged.ed25519_signature[0] ^= 1;
        assert_eq!(
            authenticate_coordinator_release_hybrid(manifest.clone(), &key, &ed_forged)
                .unwrap_err(),
            CoordinatorReleaseHybridVerifierError::Ed25519VerificationFailed
        );

        let mut ml_forged = signed_fixture(&signer, &manifest, &key);
        ml_forged.ml_dsa_65_signature[0] ^= 1;
        assert!(matches!(
            authenticate_coordinator_release_hybrid(manifest, &key, &ml_forged),
            Err(CoordinatorReleaseHybridVerifierError::InvalidMlDsa65Signature)
                | Err(CoordinatorReleaseHybridVerifierError::MlDsa65VerificationFailed)
        ));
    }

    #[test]
    fn manifest_substitution_denies_before_signature_use() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let signature = signed_fixture(&signer, &manifest, &key);
        let mut substituted = manifest;
        substituted.release_version += 1;

        assert_eq!(
            authenticate_coordinator_release_hybrid(substituted, &key, &signature).unwrap_err(),
            CoordinatorReleaseHybridVerifierError::ManifestBindingMismatch
        );
    }

    #[test]
    fn key_id_substitution_denies() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let mut signature = signed_fixture(&signer, &manifest, &key);
        signature.signing_key_id = "other-key".into();
        assert_eq!(
            authenticate_coordinator_release_hybrid(manifest, &key, &signature).unwrap_err(),
            CoordinatorReleaseHybridVerifierError::SigningKeyIdMismatch
        );
    }

    #[test]
    fn exact_signature_wire_lengths_are_required() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let mut signature = signed_fixture(&signer, &manifest, &key);
        signature.ml_dsa_65_signature.pop();
        assert_eq!(
            authenticate_coordinator_release_hybrid(manifest, &key, &signature).unwrap_err(),
            CoordinatorReleaseHybridVerifierError::InvalidMlDsa65SignatureLength
        );
    }

    #[test]
    fn signature_reference_is_derived_from_exact_signature_bytes() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let signature = signed_fixture(&signer, &manifest, &key);
        let first = signature.canonical_signature_ref().unwrap();
        let mut changed = signature.clone();
        changed.ed25519_signature[0] ^= 1;
        let second = changed.canonical_signature_ref().unwrap();
        assert_ne!(first, second);
        assert!(first.starts_with("hybrid-signature-blake3:"));
    }

    #[test]
    fn signed_message_is_manifest_key_id_and_profile_sensitive() {
        let signer = TestSigner::new();
        let (manifest, key) = fixture(&signer);
        let first = release_signature_message(&manifest, key.key_id()).unwrap();

        let mut changed_manifest = manifest.clone();
        changed_manifest.release_version += 1;
        let second = release_signature_message(&changed_manifest, key.key_id()).unwrap();
        let third = release_signature_message(&manifest, "another-key").unwrap();

        assert_ne!(first, second);
        assert_ne!(first, third);
        assert!(first
            .windows(SIGNATURE_MESSAGE_PROFILE.len())
            .any(|window| window == SIGNATURE_MESSAGE_PROFILE.as_bytes()));
        assert!(first
            .windows(SIGNATURE_EVIDENCE_PROFILE.len())
            .any(|window| window == SIGNATURE_EVIDENCE_PROFILE.as_bytes()));
    }
}
