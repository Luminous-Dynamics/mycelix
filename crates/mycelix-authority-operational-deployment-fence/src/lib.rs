// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact deployment binding for locally qualified operational freshness.
//!
//! This layer deliberately consumes #117's non-deserializable positive value
//! directly. It does not reopen the older evidence-shaped omnibus adapter path.
//! A runtime first reconstructs semantic currentness locally, then binds that
//! authority to the exact host DNA and exact binding constitutional epoch.

use mycelix_authority_freshness::{AuthoritySubjectRef, VerifiedAuthorityFreshness};
use mycelix_authority_operational_freshness::QualifiedOperationalSubjectFreshness;
use mycelix_governance_constitution::STATEMENT_PROFILE;
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-operational-deployment-fence-v0.1";
pub const HOST_DNA_CONTEXT_PROTOCOL: &str = "mycelix-host-local-dna-context-v0.1";
pub const HOST_DNA_SOURCE_REF: &str = "hdk:dna_info:local-cell";
pub const DEPLOYMENT_AUTHORITY_PROFILE: &str =
    "mycelix-authority-operational-deployment-v1-blake3-framed";
pub const DEPLOYMENT_EVIDENCE_PROFILE: &str =
    "mycelix-authority-operational-deployment-evidence-v1-blake3-framed";

const DOMAIN_DEPLOYMENT_AUTHORITY: &[u8] =
    b"mycelix/authority/operational-deployment/v1";
const DOMAIN_DEPLOYMENT_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-deployment-evidence/v1";
const MAX_DNA_HASH_BYTES: usize = 1024;

/// Non-deserializable in-process observation of the DNA hosting qualification.
///
/// The pure crate cannot invoke Holochain. Runtime qualification must construct
/// this only from `dna_info()?.hash`; CI on the HDK adapter freezes that rule.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct HostLocalDnaContext {
    protocol_version: String,
    dna_hash: String,
    source_ref: String,
    observed_at_ms: u64,
    valid_until_ms: u64,
}

impl HostLocalDnaContext {
    pub fn from_host_observation(
        dna_hash: String,
        observed_at_ms: u64,
        valid_until_ms: u64,
    ) -> Result<Self, DeploymentFenceError> {
        validate_dna_hash(&dna_hash)?;
        if observed_at_ms == 0 || valid_until_ms <= observed_at_ms {
            return Err(DeploymentFenceError::InvalidHostContextWindow);
        }
        Ok(Self {
            protocol_version: HOST_DNA_CONTEXT_PROTOCOL.into(),
            dna_hash,
            source_ref: HOST_DNA_SOURCE_REF.into(),
            observed_at_ms,
            valid_until_ms,
        })
    }

    pub fn dna_hash(&self) -> &str {
        &self.dna_hash
    }

    pub fn source_ref(&self) -> &str {
        &self.source_ref
    }

    pub fn observed_at_ms(&self) -> u64 {
        self.observed_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn validate_at(&self, now_ms: u64) -> Result<(), DeploymentFenceError> {
        if self.protocol_version != HOST_DNA_CONTEXT_PROTOCOL
            || self.source_ref != HOST_DNA_SOURCE_REF
        {
            return Err(DeploymentFenceError::WrongHostContextProtocol);
        }
        validate_dna_hash(&self.dna_hash)?;
        if now_ms == 0
            || self.observed_at_ms == 0
            || self.observed_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms <= self.observed_at_ms
        {
            return Err(DeploymentFenceError::InvalidHostContextWindow);
        }
        Ok(())
    }
}

/// Non-deserializable proof that one exact #117 current operational authority is
/// live in one exact host DNA under one exact binding constitutional statement.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedDeploymentOperationalFreshness {
    subject: AuthoritySubjectRef,
    dna_hash: String,
    constitution_statement_digest: Digest32,
    constitution_statement_profile: String,
    semantic_authority_digest: Digest32,
    semantic_authority_profile: String,
    semantic_evidence_digest: Digest32,
    semantic_evidence_profile: String,
    deployment_authority_digest: Digest32,
    deployment_authority_profile: String,
    deployment_evidence_digest: Digest32,
    deployment_evidence_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedDeploymentOperationalFreshness {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn dna_hash(&self) -> &str {
        &self.dna_hash
    }

    pub fn constitution_statement_digest(&self) -> Digest32 {
        self.constitution_statement_digest
    }

    pub fn constitution_statement_profile(&self) -> &str {
        &self.constitution_statement_profile
    }

    pub fn semantic_authority_digest(&self) -> Digest32 {
        self.semantic_authority_digest
    }

    pub fn semantic_authority_profile(&self) -> &str {
        &self.semantic_authority_profile
    }

    pub fn semantic_evidence_digest(&self) -> Digest32 {
        self.semantic_evidence_digest
    }

    pub fn semantic_evidence_profile(&self) -> &str {
        &self.semantic_evidence_profile
    }

    pub fn deployment_authority_digest(&self) -> Digest32 {
        self.deployment_authority_digest
    }

    pub fn deployment_authority_profile(&self) -> &str {
        &self.deployment_authority_profile
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

    pub fn to_verified_freshness(&self) -> VerifiedAuthorityFreshness {
        self.current_freshness.clone()
    }
}

/// Bind one locally-created #117 operational-currentness proof to the exact host
/// DNA and exact final constitutional statement.
pub fn qualify_operational_freshness_for_deployment(
    semantic: &QualifiedOperationalSubjectFreshness,
    constitutional_dna_hash: &str,
    constitution_statement_digest: Digest32,
    local_dna: &HostLocalDnaContext,
    now_ms: u64,
) -> Result<QualifiedDeploymentOperationalFreshness, DeploymentFenceError> {
    if now_ms == 0 {
        return Err(DeploymentFenceError::InvalidVerificationTime);
    }
    local_dna.validate_at(now_ms)?;
    validate_dna_hash(constitutional_dna_hash)?;
    if constitution_statement_digest.is_zero() {
        return Err(DeploymentFenceError::InvalidConstitutionDigest);
    }
    if constitutional_dna_hash != local_dna.dna_hash {
        return Err(DeploymentFenceError::DnaHashMismatch);
    }

    let semantic_freshness = semantic.to_verified_freshness();
    semantic_freshness
        .validate_at(now_ms)
        .map_err(|_| DeploymentFenceError::SemanticFreshnessNotCurrent)?;
    if &semantic_freshness.snapshot.subject != semantic.subject() {
        return Err(DeploymentFenceError::SemanticSubjectMismatch);
    }

    let deployment_authority_digest = deployment_authority_digest(
        semantic.authority_digest(),
        semantic.authority_profile(),
        semantic.root_qualification_digest(),
        semantic.root_qualification_profile(),
        semantic.operational_context_digest(),
        semantic.operational_context_profile(),
        constitution_statement_digest,
        local_dna.dna_hash(),
    );
    let deployment_evidence_digest = deployment_evidence_digest(
        deployment_authority_digest,
        semantic.evidence_digest(),
        semantic.evidence_profile(),
        local_dna,
    );
    let verification_ref = format!(
        "operational-deployment-evidence:{DEPLOYMENT_EVIDENCE_PROFILE}:{}",
        digest_hex(deployment_evidence_digest)
    );

    let verified_at_ms = semantic.verified_at_ms().max(local_dna.observed_at_ms);
    let valid_until_ms = semantic.lease_until_ms().min(local_dna.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(DeploymentFenceError::DeploymentFreshnessNotCurrent);
    }

    let mut current_freshness = semantic_freshness;
    current_freshness.verification_ref = verification_ref.clone();
    current_freshness.verified_at_ms = current_freshness.verified_at_ms.max(verified_at_ms);
    current_freshness.lease_until_ms = current_freshness.lease_until_ms.min(valid_until_ms);
    current_freshness
        .validate_at(now_ms)
        .map_err(|_| DeploymentFenceError::DeploymentFreshnessNotCurrent)?;

    Ok(QualifiedDeploymentOperationalFreshness {
        subject: semantic.subject().clone(),
        dna_hash: local_dna.dna_hash.clone(),
        constitution_statement_digest,
        constitution_statement_profile: STATEMENT_PROFILE.into(),
        semantic_authority_digest: semantic.authority_digest(),
        semantic_authority_profile: semantic.authority_profile().into(),
        semantic_evidence_digest: semantic.evidence_digest(),
        semantic_evidence_profile: semantic.evidence_profile().into(),
        deployment_authority_digest,
        deployment_authority_profile: DEPLOYMENT_AUTHORITY_PROFILE.into(),
        deployment_evidence_digest,
        deployment_evidence_profile: DEPLOYMENT_EVIDENCE_PROFILE.into(),
        current_freshness,
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

#[allow(clippy::too_many_arguments)]
fn deployment_authority_digest(
    semantic_authority_digest: Digest32,
    semantic_authority_profile: &str,
    root_qualification_digest: Digest32,
    root_qualification_profile: &str,
    operational_context_digest: Digest32,
    operational_context_profile: &str,
    constitution_statement_digest: Digest32,
    dna_hash: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_DEPLOYMENT_AUTHORITY);
    frame(&mut hasher, DEPLOYMENT_AUTHORITY_PROFILE.as_bytes());
    frame(&mut hasher, semantic_authority_profile.as_bytes());
    frame(&mut hasher, &semantic_authority_digest.0);
    frame(&mut hasher, root_qualification_profile.as_bytes());
    frame(&mut hasher, &root_qualification_digest.0);
    frame(&mut hasher, operational_context_profile.as_bytes());
    frame(&mut hasher, &operational_context_digest.0);
    frame(&mut hasher, STATEMENT_PROFILE.as_bytes());
    frame(&mut hasher, &constitution_statement_digest.0);
    frame(&mut hasher, dna_hash.as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn deployment_evidence_digest(
    deployment_authority_digest: Digest32,
    semantic_evidence_digest: Digest32,
    semantic_evidence_profile: &str,
    local_dna: &HostLocalDnaContext,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_DEPLOYMENT_EVIDENCE);
    frame(&mut hasher, DEPLOYMENT_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &deployment_authority_digest.0);
    frame(&mut hasher, semantic_evidence_profile.as_bytes());
    frame(&mut hasher, &semantic_evidence_digest.0);
    frame(&mut hasher, local_dna.source_ref.as_bytes());
    frame(&mut hasher, &local_dna.observed_at_ms.to_le_bytes());
    frame(&mut hasher, &local_dna.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn validate_dna_hash(value: &str) -> Result<(), DeploymentFenceError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_DNA_HASH_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        Err(DeploymentFenceError::InvalidDnaHash)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeploymentFenceError {
    WrongHostContextProtocol,
    InvalidDnaHash,
    InvalidHostContextWindow,
    InvalidVerificationTime,
    InvalidConstitutionDigest,
    DnaHashMismatch,
    SemanticFreshnessNotCurrent,
    SemanticSubjectMismatch,
    DeploymentFreshnessNotCurrent,
}

impl fmt::Display for DeploymentFenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongHostContextProtocol => "wrong host DNA context protocol/source",
            Self::InvalidDnaHash => "invalid host or constitutional DNA hash",
            Self::InvalidHostContextWindow => "host DNA observation window is invalid or stale",
            Self::InvalidVerificationTime => "deployment qualification time must be positive",
            Self::InvalidConstitutionDigest => "current constitutional statement digest is invalid",
            Self::DnaHashMismatch => "binding constitution belongs to a different DNA than the local host cell",
            Self::SemanticFreshnessNotCurrent => "locally qualified operational freshness is not current",
            Self::SemanticSubjectMismatch => "qualified operational subject does not match its freshness snapshot",
            Self::DeploymentFreshnessNotCurrent => "deployment-bound operational freshness is not current",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for DeploymentFenceError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn host_context_rejects_invalid_window() {
        assert!(HostLocalDnaContext::from_host_observation("uhC0kExample".into(), 100, 100).is_err());
        assert!(HostLocalDnaContext::from_host_observation("uhC0kExample".into(), 100, 101).is_ok());
    }

    #[test]
    fn deployment_identity_changes_with_dna() {
        let a = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(4), "uhC0kA");
        let b = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(4), "uhC0kB");
        assert_ne!(a, b);
    }

    #[test]
    fn deployment_identity_changes_with_constitution() {
        let a = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(4), "uhC0kA");
        let b = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(5), "uhC0kA");
        assert_ne!(a, b);
    }

    #[test]
    fn host_reobservation_changes_evidence_not_stable_authority() {
        let authority = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(4), "uhC0kA");
        let authority_again = deployment_authority_digest(d(1), "semantic", d(2), "root", d(3), "context", d(4), "uhC0kA");
        let first = HostLocalDnaContext::from_host_observation("uhC0kA".into(), 100, 200).unwrap();
        let second = HostLocalDnaContext::from_host_observation("uhC0kA".into(), 110, 210).unwrap();
        let evidence_a = deployment_evidence_digest(authority, d(9), "evidence", &first);
        let evidence_b = deployment_evidence_digest(authority, d(9), "evidence", &second);
        assert_ne!(evidence_a, evidence_b);
        assert_eq!(authority, authority_again);
    }
}
