// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

extern crate alloc;

use alloc::{string::String, vec::Vec};
use curve25519_dalek::{edwards::CompressedEdwardsY, scalar::Scalar};
use ed25519_dalek::{Signature, VerifyingKey};
use mycelix_constitutional_root::{
    IDENTITY_PROFILE as ROOT_IDENTITY_PROFILE, QualifiedConstitutionalRootA,
    ROTATION_AUTHORITY_PROFILE, SOURCE_DESCRIPTOR_PROFILE,
};
use sha2::{Digest, Sha256};

pub const TRANSITION_PROFILE: &str =
    "mycelix-constitutional-root-transition-v1-sha256-framed-semantic";
pub const MATERIAL_PROFILE: &str =
    "mycelix-constitutional-root-rotation-authority-ed25519-single-v1";
pub const MATERIAL_SUITE: &str = "ed25519-spki-v1";
pub const PROOF_SUITE: &str = "ed25519-signature-v1";
pub const NORMAL_ROTATION_PROFILE: &str = "constitutional-root-rotation-v1";

const TRANSITION_DOMAIN: &[u8] = b"mycelix/public-institution/constitutional-root-transition/v1";
const MATERIAL_DOMAIN: &[u8] =
    b"mycelix/public-institution/constitutional-root-rotation-authority-material/ed25519-single/v1";
const ED25519_SPKI_PREFIX: [u8; 12] = [
    0x30, 0x2a, 0x30, 0x05, 0x06, 0x03, 0x2b, 0x65, 0x70, 0x03, 0x21, 0x00,
];
const MAX_NONCE_BYTES: usize = 64;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TransitionCandidate {
    pub profile: String,
    pub predecessor_root_identity_profile: String,
    pub predecessor_root_digest: [u8; 32],
    pub predecessor_generation: u64,
    pub predecessor_source_descriptor_profile: String,
    pub predecessor_source_descriptor_digest: [u8; 32],
    pub predecessor_rotation_authority_profile: String,
    pub predecessor_rotation_authority_digest: [u8; 32],
    pub successor_root_identity_profile: String,
    pub successor_root_digest: [u8; 32],
    pub successor_generation: u64,
    pub successor_source_descriptor_profile: String,
    pub successor_source_descriptor_digest: [u8; 32],
    pub successor_rotation_authority_profile: Option<String>,
    pub successor_rotation_authority_digest: Option<[u8; 32]>,
    pub authorized_at_ms: u64,
    pub effective_at_ms: u64,
    pub replay_nonce: Vec<u8>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizationMaterial {
    pub profile: String,
    pub suite: String,
    pub public_key_spki_der: Vec<u8>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizationProof {
    pub suite: String,
    pub signature: [u8; 64],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StableIdentity {
    pub profile: &'static str,
    pub digest: [u8; 32],
}

#[derive(Debug, Eq, PartialEq)]
pub struct VerifiedRootTransition {
    transition_identity: StableIdentity,
    predecessor_root_digest: [u8; 32],
    predecessor_generation: u64,
    predecessor_source_descriptor_digest: [u8; 32],
    predecessor_rotation_authority_digest: [u8; 32],
    successor_root_digest: [u8; 32],
    successor_generation: u64,
    successor_source_descriptor_digest: [u8; 32],
    successor_rotation_authority_digest: Option<[u8; 32]>,
    authorization_material_commitment: [u8; 32],
    authorization_proof_digest: [u8; 32],
    authorized_at_ms: u64,
    effective_at_ms: u64,
    replay_nonce: Vec<u8>,
}

impl VerifiedRootTransition {
    pub const fn transition_identity(&self) -> &StableIdentity {
        &self.transition_identity
    }

    pub const fn predecessor_root_digest(&self) -> &[u8; 32] {
        &self.predecessor_root_digest
    }

    pub const fn predecessor_generation(&self) -> u64 {
        self.predecessor_generation
    }

    pub const fn predecessor_source_descriptor_digest(&self) -> &[u8; 32] {
        &self.predecessor_source_descriptor_digest
    }

    pub const fn predecessor_rotation_authority_digest(&self) -> &[u8; 32] {
        &self.predecessor_rotation_authority_digest
    }

    pub const fn successor_root_digest(&self) -> &[u8; 32] {
        &self.successor_root_digest
    }

    pub const fn successor_generation(&self) -> u64 {
        self.successor_generation
    }

    pub const fn successor_source_descriptor_digest(&self) -> &[u8; 32] {
        &self.successor_source_descriptor_digest
    }

    pub const fn successor_rotation_authority_digest(&self) -> Option<&[u8; 32]> {
        self.successor_rotation_authority_digest.as_ref()
    }

    pub const fn authorization_material_commitment(&self) -> &[u8; 32] {
        &self.authorization_material_commitment
    }

    pub const fn authorization_proof_digest(&self) -> &[u8; 32] {
        &self.authorization_proof_digest
    }

    pub const fn authorized_at_ms(&self) -> u64 {
        self.authorized_at_ms
    }

    pub const fn effective_at_ms(&self) -> u64 {
        self.effective_at_ms
    }

    pub fn replay_nonce(&self) -> &[u8] {
        &self.replay_nonce
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransitionError {
    PredecessorDoesNotAuthorizeRotation,
    GenerationOverflow,
    DiscontinuousGeneration,
    SuccessorPredecessorMismatch,
    PreservedFieldChanged,
    AuthorizationBeforePredecessorValidity,
    EffectiveBeforeAuthorization,
    TransitionOutsidePredecessorValidity,
    SuccessorEffectiveTimeMismatch,
    InvalidReplayNonce,
    CandidateRebindingMismatch,
    WrongMaterialProfile,
    WrongMaterialSuite,
    NonCanonicalSpki,
    MaterialCommitmentMismatch,
    WrongProofSuite,
    NonCanonicalPublicKey,
    NonPrimeOrderPublicKey,
    NonCanonicalSignaturePoint,
    NonPrimeOrderSignaturePoint,
    NonCanonicalSignatureScalar,
    InvalidVerifyingKey,
    InvalidSignature,
}

pub fn verify_transition(
    predecessor: &QualifiedConstitutionalRootA,
    successor: &QualifiedConstitutionalRootA,
    candidate: &TransitionCandidate,
    material: &AuthorizationMaterial,
    proof: &AuthorizationProof,
) -> Result<VerifiedRootTransition, TransitionError> {
    let pred = predecessor.root();
    let succ = successor.root();

    if pred.rotation_mode != "predecessor-authorized"
        || pred.rotation_profile.as_deref() != Some(NORMAL_ROTATION_PROFILE)
    {
        return Err(TransitionError::PredecessorDoesNotAuthorizeRotation);
    }

    let expected_generation = pred
        .generation
        .checked_add(1)
        .ok_or(TransitionError::GenerationOverflow)?;
    if succ.generation != expected_generation {
        return Err(TransitionError::DiscontinuousGeneration);
    }
    if succ.predecessor_root_digest != Some(predecessor.root_identity().digest) {
        return Err(TransitionError::SuccessorPredecessorMismatch);
    }

    if succ.protocol_version != pred.protocol_version
        || succ.institution_id != pred.institution_id
        || succ.jurisdiction_id != pred.jurisdiction_id
        || succ.bootstrap_mode != pred.bootstrap_mode
        || succ.bootstrap_profile != pred.bootstrap_profile
        || succ.authoritative_root_source_ref != pred.authoritative_root_source_ref
        || succ.root_coverage_profile != pred.root_coverage_profile
        || succ.root_source_verification_profile != pred.root_source_verification_profile
    {
        return Err(TransitionError::PreservedFieldChanged);
    }

    if candidate.authorized_at_ms < pred.valid_from_ms {
        return Err(TransitionError::AuthorizationBeforePredecessorValidity);
    }
    if candidate.effective_at_ms < candidate.authorized_at_ms {
        return Err(TransitionError::EffectiveBeforeAuthorization);
    }
    if let Some(expiry) = pred.expires_at_ms
        && (candidate.authorized_at_ms >= expiry || candidate.effective_at_ms >= expiry)
    {
        return Err(TransitionError::TransitionOutsidePredecessorValidity);
    }
    if succ.valid_from_ms != candidate.effective_at_ms {
        return Err(TransitionError::SuccessorEffectiveTimeMismatch);
    }
    validate_nonce(&candidate.replay_nonce)?;

    let predecessor_rotation = predecessor
        .rotation_authority_identity()
        .ok_or(TransitionError::PredecessorDoesNotAuthorizeRotation)?;
    let successor_rotation = successor.rotation_authority_identity();
    let expected_candidate = TransitionCandidate {
        profile: TRANSITION_PROFILE.into(),
        predecessor_root_identity_profile: ROOT_IDENTITY_PROFILE.into(),
        predecessor_root_digest: predecessor.root_identity().digest,
        predecessor_generation: pred.generation,
        predecessor_source_descriptor_profile: SOURCE_DESCRIPTOR_PROFILE.into(),
        predecessor_source_descriptor_digest: predecessor.source_descriptor_identity().digest,
        predecessor_rotation_authority_profile: ROTATION_AUTHORITY_PROFILE.into(),
        predecessor_rotation_authority_digest: predecessor_rotation.digest,
        successor_root_identity_profile: ROOT_IDENTITY_PROFILE.into(),
        successor_root_digest: successor.root_identity().digest,
        successor_generation: succ.generation,
        successor_source_descriptor_profile: SOURCE_DESCRIPTOR_PROFILE.into(),
        successor_source_descriptor_digest: successor.source_descriptor_identity().digest,
        successor_rotation_authority_profile: successor_rotation
            .map(|_| ROTATION_AUTHORITY_PROFILE.into()),
        successor_rotation_authority_digest: successor_rotation.map(|identity| identity.digest),
        authorized_at_ms: candidate.authorized_at_ms,
        effective_at_ms: candidate.effective_at_ms,
        replay_nonce: candidate.replay_nonce.clone(),
    };
    if candidate != &expected_candidate {
        return Err(TransitionError::CandidateRebindingMismatch);
    }

    let material_commitment = authorization_material_commitment(material)?;
    let predecessor_anchor = pred
        .rotation_authority_anchor_digest
        .ok_or(TransitionError::PredecessorDoesNotAuthorizeRotation)?;
    if material_commitment != predecessor_anchor {
        return Err(TransitionError::MaterialCommitmentMismatch);
    }

    if proof.suite != PROOF_SUITE {
        return Err(TransitionError::WrongProofSuite);
    }
    let message = canonical_candidate_bytes(candidate)?;
    verify_ed25519(material, &message, &proof.signature)?;

    Ok(VerifiedRootTransition {
        transition_identity: StableIdentity {
            profile: TRANSITION_PROFILE,
            digest: sha256(&message),
        },
        predecessor_root_digest: predecessor.root_identity().digest,
        predecessor_generation: pred.generation,
        predecessor_source_descriptor_digest: predecessor.source_descriptor_identity().digest,
        predecessor_rotation_authority_digest: predecessor_rotation.digest,
        successor_root_digest: successor.root_identity().digest,
        successor_generation: succ.generation,
        successor_source_descriptor_digest: successor.source_descriptor_identity().digest,
        successor_rotation_authority_digest: successor_rotation.map(|identity| identity.digest),
        authorization_material_commitment: material_commitment,
        authorization_proof_digest: sha256(&proof.signature),
        authorized_at_ms: candidate.authorized_at_ms,
        effective_at_ms: candidate.effective_at_ms,
        replay_nonce: candidate.replay_nonce.clone(),
    })
}

pub fn authorization_material_commitment(
    material: &AuthorizationMaterial,
) -> Result<[u8; 32], TransitionError> {
    if material.profile != MATERIAL_PROFILE {
        return Err(TransitionError::WrongMaterialProfile);
    }
    if material.suite != MATERIAL_SUITE {
        return Err(TransitionError::WrongMaterialSuite);
    }
    if material.public_key_spki_der.len() != ED25519_SPKI_PREFIX.len() + 32
        || !material
            .public_key_spki_der
            .starts_with(&ED25519_SPKI_PREFIX)
    {
        return Err(TransitionError::NonCanonicalSpki);
    }

    let mut bytes = Vec::new();
    bytes.extend_from_slice(MATERIAL_DOMAIN);
    frame_text(&mut bytes, MATERIAL_PROFILE);
    frame_text(&mut bytes, MATERIAL_SUITE);
    frame(&mut bytes, &material.public_key_spki_der);
    Ok(sha256(&bytes))
}

pub fn canonical_candidate_bytes(
    candidate: &TransitionCandidate,
) -> Result<Vec<u8>, TransitionError> {
    if candidate.profile != TRANSITION_PROFILE
        || candidate.predecessor_root_identity_profile != ROOT_IDENTITY_PROFILE
        || candidate.predecessor_source_descriptor_profile != SOURCE_DESCRIPTOR_PROFILE
        || candidate.predecessor_rotation_authority_profile != ROTATION_AUTHORITY_PROFILE
        || candidate.successor_root_identity_profile != ROOT_IDENTITY_PROFILE
        || candidate.successor_source_descriptor_profile != SOURCE_DESCRIPTOR_PROFILE
    {
        return Err(TransitionError::CandidateRebindingMismatch);
    }
    match (
        candidate.successor_rotation_authority_profile.as_deref(),
        candidate.successor_rotation_authority_digest,
    ) {
        (None, None) => {}
        (Some(profile), Some(_)) if profile == ROTATION_AUTHORITY_PROFILE => {}
        _ => return Err(TransitionError::CandidateRebindingMismatch),
    }
    validate_nonce(&candidate.replay_nonce)?;

    let mut out = Vec::new();
    out.extend_from_slice(TRANSITION_DOMAIN);
    frame_text(&mut out, TRANSITION_PROFILE);
    frame_profiled(
        &mut out,
        ROOT_IDENTITY_PROFILE,
        &candidate.predecessor_root_digest,
    );
    frame_u64(&mut out, candidate.predecessor_generation);
    frame_profiled(
        &mut out,
        SOURCE_DESCRIPTOR_PROFILE,
        &candidate.predecessor_source_descriptor_digest,
    );
    frame_profiled(
        &mut out,
        ROTATION_AUTHORITY_PROFILE,
        &candidate.predecessor_rotation_authority_digest,
    );
    frame_profiled(
        &mut out,
        ROOT_IDENTITY_PROFILE,
        &candidate.successor_root_digest,
    );
    frame_u64(&mut out, candidate.successor_generation);
    frame_profiled(
        &mut out,
        SOURCE_DESCRIPTOR_PROFILE,
        &candidate.successor_source_descriptor_digest,
    );
    match candidate.successor_rotation_authority_digest {
        Some(digest) => {
            frame(&mut out, &[1]);
            frame_profiled(&mut out, ROTATION_AUTHORITY_PROFILE, &digest);
        }
        None => frame(&mut out, &[0]),
    }
    frame_u64(&mut out, candidate.authorized_at_ms);
    frame_u64(&mut out, candidate.effective_at_ms);
    frame(&mut out, &candidate.replay_nonce);
    Ok(out)
}

fn verify_ed25519(
    material: &AuthorizationMaterial,
    message: &[u8],
    signature_bytes: &[u8; 64],
) -> Result<(), TransitionError> {
    if material.public_key_spki_der.len() != ED25519_SPKI_PREFIX.len() + 32
        || !material
            .public_key_spki_der
            .starts_with(&ED25519_SPKI_PREFIX)
    {
        return Err(TransitionError::NonCanonicalSpki);
    }
    let mut public_key = [0u8; 32];
    public_key.copy_from_slice(&material.public_key_spki_der[ED25519_SPKI_PREFIX.len()..]);
    validate_prime_order_point(
        public_key,
        TransitionError::NonCanonicalPublicKey,
        TransitionError::NonPrimeOrderPublicKey,
    )?;

    let mut r_bytes = [0u8; 32];
    r_bytes.copy_from_slice(&signature_bytes[..32]);
    validate_prime_order_point(
        r_bytes,
        TransitionError::NonCanonicalSignaturePoint,
        TransitionError::NonPrimeOrderSignaturePoint,
    )?;

    let mut s_bytes = [0u8; 32];
    s_bytes.copy_from_slice(&signature_bytes[32..]);
    let scalar: Option<Scalar> = Scalar::from_canonical_bytes(s_bytes).into();
    if scalar.is_none() {
        return Err(TransitionError::NonCanonicalSignatureScalar);
    }

    let verifying_key =
        VerifyingKey::from_bytes(&public_key).map_err(|_| TransitionError::InvalidVerifyingKey)?;
    let signature = Signature::from_bytes(signature_bytes);
    verifying_key
        .verify_strict(message, &signature)
        .map_err(|_| TransitionError::InvalidSignature)
}

fn validate_prime_order_point(
    encoded: [u8; 32],
    noncanonical: TransitionError,
    nontorsionfree: TransitionError,
) -> Result<(), TransitionError> {
    let point = CompressedEdwardsY(encoded)
        .decompress()
        .ok_or(noncanonical)?;
    if point.compress().to_bytes() != encoded {
        return Err(noncanonical);
    }
    if !point.is_torsion_free() {
        return Err(nontorsionfree);
    }
    Ok(())
}

fn validate_nonce(nonce: &[u8]) -> Result<(), TransitionError> {
    if nonce.is_empty() || nonce.len() > MAX_NONCE_BYTES || nonce.iter().all(|byte| *byte == 0) {
        return Err(TransitionError::InvalidReplayNonce);
    }
    Ok(())
}

fn sha256(bytes: &[u8]) -> [u8; 32] {
    let digest = Sha256::digest(bytes);
    let mut out = [0u8; 32];
    out.copy_from_slice(&digest);
    out
}

fn frame(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
}

fn frame_text(out: &mut Vec<u8>, value: &str) {
    frame(out, value.as_bytes());
}

fn frame_u64(out: &mut Vec<u8>, value: u64) {
    frame(out, &value.to_le_bytes());
}

fn frame_profiled(out: &mut Vec<u8>, profile: &str, digest: &[u8; 32]) {
    frame_text(out, profile);
    frame(out, digest);
}
