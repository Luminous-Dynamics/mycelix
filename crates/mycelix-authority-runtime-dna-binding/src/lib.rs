// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact runtime-DNA binding for live current authority.
//!
//! Constitutional statements remain substrate-neutral. This layer adds the
//! deployment-specific invariant needed before a Holochain runtime may treat a
//! semantically qualified authority as live in one concrete DNA:
//!
//! `current_constitution.dna_hash == independently verified local DNA hash`.
//!
//! The expected local DNA hash is an adapter boundary. A future HDK provider MUST
//! construct `VerifiedLocalDnaContext` from the local host/cell context, never from
//! caller input or from the constitution receipt being checked.

use mycelix_authority_current_operational::{
    qualify_current_operational_authority, CurrentOperationalAuthorityEvidence,
    QualifiedCurrentOperationalAuthority,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-runtime-dna-binding-v0.1";
pub const LOCAL_DNA_CONTEXT_PROTOCOL: &str = "mycelix-local-dna-context-v0.1";
pub const DEPLOYMENT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-runtime-dna-binding-v1-blake3-framed";
pub const DEPLOYMENT_EVIDENCE_PROFILE: &str =
    "mycelix-authority-runtime-dna-binding-evidence-v1-blake3-framed";

const DOMAIN_DEPLOYMENT: &[u8] = b"mycelix/authority/runtime-dna-binding/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority/runtime-dna-binding-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_DNA_HASH_BYTES: usize = 1024;

/// Evidence-shaped adapter receipt for the exact DNA in which qualification is
/// being performed.
///
/// This type is intentionally deserializable because it crosses a runtime adapter
/// boundary. Deserialization is not authority. A production HDK adapter must
/// construct it from the local host/cell context and must not accept these fields
/// from the external caller whose request is being authorized.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedLocalDnaContext {
    pub protocol_version: String,
    pub dna_hash: String,
    /// Host/local-cell evidence source, for example a local DNA-info provider ref.
    pub source_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedLocalDnaContext {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), RuntimeDnaBindingError> {
        if self.protocol_version != LOCAL_DNA_CONTEXT_PROTOCOL {
            return Err(RuntimeDnaBindingError::WrongLocalContextProtocol);
        }
        validate_dna_hash(&self.dna_hash)?;
        require_ref(&self.source_ref)?;
        require_ref(&self.verification_ref)?;
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms < self.verified_at_ms
        {
            return Err(RuntimeDnaBindingError::InvalidLocalContextWindow);
        }
        Ok(())
    }
}

/// Non-deserializable proof that one semantically current operational authority
/// is also live under one exact local DNA context.
///
/// `semantic_qualification_digest` remains suitable for substrate-neutral audit.
/// `deployment_qualification_digest` additionally commits the exact DNA hash and
/// is the identity that a production Holochain execution path should consume.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedRuntimeDnaAuthority {
    current_authority: QualifiedCurrentOperationalAuthority,
    dna_hash: String,
    semantic_qualification_digest: Digest32,
    semantic_qualification_profile: String,
    deployment_qualification_digest: Digest32,
    deployment_qualification_profile: String,
    deployment_evidence_digest: Digest32,
    deployment_evidence_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedRuntimeDnaAuthority {
    pub fn target_subject(&self) -> &mycelix_authority_freshness::AuthoritySubjectRef {
        self.current_authority.target_subject()
    }

    pub fn dna_hash(&self) -> &str {
        &self.dna_hash
    }

    pub fn semantic_qualification_digest(&self) -> Digest32 {
        self.semantic_qualification_digest
    }

    pub fn semantic_qualification_profile(&self) -> &str {
        &self.semantic_qualification_profile
    }

    pub fn deployment_qualification_digest(&self) -> Digest32 {
        self.deployment_qualification_digest
    }

    pub fn deployment_qualification_profile(&self) -> &str {
        &self.deployment_qualification_profile
    }

    pub fn deployment_evidence_digest(&self) -> Digest32 {
        self.deployment_evidence_digest
    }

    pub fn deployment_evidence_profile(&self) -> &str {
        &self.deployment_evidence_profile
    }

    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    /// Projection for existing downstream freshness consumers after exact local
    /// DNA qualification. The snapshot/generation identity remains unchanged;
    /// dynamic verification metadata is narrowed to this deployment proof.
    pub fn to_verified_freshness(
        &self,
    ) -> mycelix_authority_freshness::VerifiedAuthorityFreshness {
        let mut receipt = self.current_authority.to_verified_freshness();
        receipt.verification_ref = self.verification_ref.clone();
        receipt.verified_at_ms = receipt.verified_at_ms.max(self.verified_at_ms);
        receipt.lease_until_ms = receipt.lease_until_ms.min(self.valid_until_ms);
        receipt
    }
}

/// Re-run the complete current-authority chain and bind the positive result to the
/// exact independently supplied local DNA context.
pub fn qualify_current_authority_for_local_dna(
    evidence: &CurrentOperationalAuthorityEvidence,
    local_dna: &VerifiedLocalDnaContext,
    now_ms: u64,
) -> Result<QualifiedRuntimeDnaAuthority, RuntimeDnaBindingError> {
    local_dna.validate_at(now_ms)?;
    verify_exact_dna_match(&evidence.current_constitution.dna_hash, &local_dna.dna_hash)?;

    // Do not accept a serialized positive authority from an adapter. Re-run #120
    // from the complete evidence-shaped input after the local DNA check.
    let current_authority = qualify_current_operational_authority(evidence, now_ms)
        .map_err(|_| RuntimeDnaBindingError::CurrentAuthorityQualificationDenied)?;

    let semantic_digest = current_authority.qualification_digest();
    let semantic_profile = current_authority.qualification_profile().to_string();
    let deployment_digest = deployment_qualification_digest(
        semantic_digest,
        &semantic_profile,
        &local_dna.dna_hash,
    );
    let deployment_evidence_digest = deployment_evidence_digest(
        deployment_digest,
        current_authority.evidence_digest(),
        local_dna,
    );
    let verification_ref = format!(
        "runtime-dna-authority:{DEPLOYMENT_EVIDENCE_PROFILE}:{}",
        hex_digest(deployment_evidence_digest)
    );

    let verified_at_ms = current_authority
        .verified_at_ms()
        .max(local_dna.verified_at_ms);
    let valid_until_ms = current_authority
        .valid_until_ms()
        .min(local_dna.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(RuntimeDnaBindingError::DeploymentAuthorityNotCurrent);
    }

    let mut projected = current_authority.to_verified_freshness();
    projected.verification_ref = verification_ref.clone();
    projected.verified_at_ms = projected.verified_at_ms.max(verified_at_ms);
    projected.lease_until_ms = projected.lease_until_ms.min(valid_until_ms);
    projected
        .validate_at(now_ms)
        .map_err(|_| RuntimeDnaBindingError::DeploymentAuthorityNotCurrent)?;

    Ok(QualifiedRuntimeDnaAuthority {
        current_authority,
        dna_hash: local_dna.dna_hash.clone(),
        semantic_qualification_digest: semantic_digest,
        semantic_qualification_profile: semantic_profile,
        deployment_qualification_digest: deployment_digest,
        deployment_qualification_profile: DEPLOYMENT_QUALIFICATION_PROFILE.into(),
        deployment_evidence_digest,
        deployment_evidence_profile: DEPLOYMENT_EVIDENCE_PROFILE.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_exact_dna_match(
    constitution_dna_hash: &str,
    local_dna_hash: &str,
) -> Result<(), RuntimeDnaBindingError> {
    validate_dna_hash(constitution_dna_hash)?;
    validate_dna_hash(local_dna_hash)?;
    if constitution_dna_hash != local_dna_hash {
        return Err(RuntimeDnaBindingError::DnaHashMismatch);
    }
    Ok(())
}

fn deployment_qualification_digest(
    semantic_digest: Digest32,
    semantic_profile: &str,
    dna_hash: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_DEPLOYMENT);
    frame(&mut hasher, DEPLOYMENT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, semantic_profile.as_bytes());
    frame(&mut hasher, &semantic_digest.0);
    frame(&mut hasher, dna_hash.as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn deployment_evidence_digest(
    deployment_digest: Digest32,
    current_authority_evidence_digest: Digest32,
    local_dna: &VerifiedLocalDnaContext,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, DEPLOYMENT_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &deployment_digest.0);
    frame(&mut hasher, &current_authority_evidence_digest.0);
    frame(&mut hasher, local_dna.source_ref.as_bytes());
    frame(&mut hasher, local_dna.verification_ref.as_bytes());
    frame(&mut hasher, &local_dna.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &local_dna.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn validate_dna_hash(value: &str) -> Result<(), RuntimeDnaBindingError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_DNA_HASH_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        return Err(RuntimeDnaBindingError::InvalidDnaHash);
    }
    Ok(())
}

fn require_ref(value: &str) -> Result<(), RuntimeDnaBindingError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(RuntimeDnaBindingError::InvalidReference)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RuntimeDnaBindingError {
    WrongLocalContextProtocol,
    InvalidDnaHash,
    InvalidReference,
    InvalidLocalContextWindow,
    DnaHashMismatch,
    CurrentAuthorityQualificationDenied,
    DeploymentAuthorityNotCurrent,
}

impl fmt::Display for RuntimeDnaBindingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongLocalContextProtocol => "wrong local DNA context protocol",
            Self::InvalidDnaHash => "invalid local or constitutional DNA hash",
            Self::InvalidReference => "invalid local DNA verification reference",
            Self::InvalidLocalContextWindow => "local DNA context verification is stale or invalid",
            Self::DnaHashMismatch => "verified constitution belongs to a different DNA",
            Self::CurrentAuthorityQualificationDenied => "current operational authority qualification denied",
            Self::DeploymentAuthorityNotCurrent => "deployment-bound authority is stale or invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for RuntimeDnaBindingError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn local(dna: &str) -> VerifiedLocalDnaContext {
        VerifiedLocalDnaContext {
            protocol_version: LOCAL_DNA_CONTEXT_PROTOCOL.into(),
            dna_hash: dna.into(),
            source_ref: "host:local-dna".into(),
            verification_ref: "verify:local-dna:1".into(),
            verified_at_ms: 100,
            valid_until_ms: 200,
        }
    }

    #[test]
    fn exact_dna_match_is_mandatory() {
        assert!(verify_exact_dna_match("uhC0kA", "uhC0kA").is_ok());
        assert_eq!(
            verify_exact_dna_match("uhC0kA", "uhC0kB"),
            Err(RuntimeDnaBindingError::DnaHashMismatch)
        );
    }

    #[test]
    fn deployment_identity_changes_across_dna_hashes() {
        let a = deployment_qualification_digest(d(1), "semantic-profile-v1", "uhC0kA");
        let b = deployment_qualification_digest(d(1), "semantic-profile-v1", "uhC0kB");
        assert_ne!(a, b);
    }

    #[test]
    fn semantic_identity_can_remain_constant_while_deployment_identity_changes() {
        let semantic = d(7);
        let a = deployment_qualification_digest(semantic, "semantic-profile-v1", "uhC0kA");
        let b = deployment_qualification_digest(semantic, "semantic-profile-v1", "uhC0kB");
        assert_eq!(semantic, d(7));
        assert_ne!(a, b);
    }

    #[test]
    fn local_context_must_be_current_and_bounded() {
        let good = local("uhC0kA");
        assert!(good.validate_at(150).is_ok());

        let mut stale = good.clone();
        stale.valid_until_ms = 150;
        assert_eq!(
            stale.validate_at(150),
            Err(RuntimeDnaBindingError::InvalidLocalContextWindow)
        );
    }

    #[test]
    fn deployment_evidence_identity_changes_with_local_verification_instance() {
        let mut first = local("uhC0kA");
        let base = deployment_qualification_digest(d(1), "semantic-profile-v1", "uhC0kA");
        let a = deployment_evidence_digest(base, d(2), &first);
        first.verification_ref = "verify:local-dna:2".into();
        let b = deployment_evidence_digest(base, d(2), &first);
        assert_ne!(a, b);
    }
}
