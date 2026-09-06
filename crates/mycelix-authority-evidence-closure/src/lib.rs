// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Complete dynamic proof provenance for deployment-bound current authority.
//!
//! #120/#123 already provide the correct stable semantic/deployment authority
//! identities. This layer closes the remaining dynamic provenance gap by also
//! committing the exact fresh #115 control-plane proof instances used to establish
//! current operational policy authority.
//!
//! Stable deployment authority identity is deliberately unchanged. A re-probe of
//! the same root/policies/generations may change this layer's evidence identity
//! without minting a new authority domain.

use mycelix_authority_control_plane_freshness::{
    qualify_control_plane_subject_freshness, QualifiedControlPlaneSubjectFreshness,
};
use mycelix_authority_current_operational::CurrentOperationalAuthorityEvidence;
use mycelix_authority_runtime_dna_binding::{
    qualify_current_authority_for_local_dna, QualifiedRuntimeDnaAuthority,
    VerifiedLocalDnaContext,
};
use mycelix_authority_state_bootstrap_root::qualify_bootstrap_root;
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-evidence-closure-v0.1";
pub const CONTROL_PLANE_EVIDENCE_PROFILE: &str =
    "mycelix-authority-control-plane-evidence-set-v1-blake3-framed";
pub const COMPLETE_EVIDENCE_PROFILE: &str =
    "mycelix-authority-complete-deployment-evidence-v1-blake3-framed";

const DOMAIN_CONTROL_PLANE: &[u8] = b"mycelix/authority/control-plane-evidence-set/v1";
const DOMAIN_COMPLETE: &[u8] = b"mycelix/authority/complete-deployment-evidence/v1";
const MAX_CONTROL_PLANE_SUBJECTS: usize = 3;

/// Final non-deserializable live authority proof with complete dynamic provenance.
///
/// Stable deployment authority remains owned by #123. This wrapper adds a
/// canonical commitment to every fresh control-plane proof instance used by #120.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCompleteDeploymentAuthority {
    runtime_authority: QualifiedRuntimeDnaAuthority,
    control_plane_evidence_digest: Digest32,
    control_plane_evidence_profile: String,
    complete_evidence_digest: Digest32,
    complete_evidence_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCompleteDeploymentAuthority {
    pub fn target_subject(&self) -> &mycelix_authority_freshness::AuthoritySubjectRef {
        self.runtime_authority.target_subject()
    }

    pub fn dna_hash(&self) -> &str {
        self.runtime_authority.dna_hash()
    }

    pub fn semantic_qualification_digest(&self) -> Digest32 {
        self.runtime_authority.semantic_qualification_digest()
    }

    pub fn semantic_qualification_profile(&self) -> &str {
        self.runtime_authority.semantic_qualification_profile()
    }

    pub fn deployment_qualification_digest(&self) -> Digest32 {
        self.runtime_authority.deployment_qualification_digest()
    }

    pub fn deployment_qualification_profile(&self) -> &str {
        self.runtime_authority.deployment_qualification_profile()
    }

    pub fn control_plane_evidence_digest(&self) -> Digest32 {
        self.control_plane_evidence_digest
    }

    pub fn control_plane_evidence_profile(&self) -> &str {
        &self.control_plane_evidence_profile
    }

    pub fn complete_evidence_digest(&self) -> Digest32 {
        self.complete_evidence_digest
    }

    pub fn complete_evidence_profile(&self) -> &str {
        &self.complete_evidence_profile
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

    /// Projection for downstream live-freshness consumers after complete evidence
    /// closure. Snapshot/generation semantics remain unchanged; only dynamic proof
    /// provenance and the conservative evidence window are replaced/narrowed.
    pub fn to_verified_freshness(
        &self,
    ) -> mycelix_authority_freshness::VerifiedAuthorityFreshness {
        let mut receipt = self.runtime_authority.to_verified_freshness();
        receipt.verification_ref = self.verification_ref.clone();
        receipt.verified_at_ms = receipt.verified_at_ms.max(self.verified_at_ms);
        receipt.lease_until_ms = receipt.lease_until_ms.min(self.valid_until_ms);
        receipt
    }
}

/// Re-run deployment qualification and close its dynamic evidence identity over
/// all exact current control-plane proof instances.
pub fn qualify_complete_deployment_authority(
    evidence: &CurrentOperationalAuthorityEvidence,
    local_dna: &VerifiedLocalDnaContext,
    now_ms: u64,
) -> Result<QualifiedCompleteDeploymentAuthority, EvidenceClosureError> {
    if evidence.control_plane_subjects.is_empty()
        || evidence.control_plane_subjects.len() > MAX_CONTROL_PLANE_SUBJECTS
    {
        return Err(EvidenceClosureError::InvalidControlPlaneSubjectCount);
    }

    // #123 re-runs the complete #120 chain and exact local-DNA binding first.
    // Success proves the candidate control-plane list was exactly sufficient for
    // #116's deterministic 2/3-subject current-policy requirement.
    let runtime_authority = qualify_current_authority_for_local_dna(evidence, local_dna, now_ms)
        .map_err(|_| EvidenceClosureError::RuntimeAuthorityQualificationDenied)?;

    // Reconstruct the exact root and #115 proof instances independently so final
    // dynamic provenance commits the control-plane proofs that #120 consumed.
    let root = qualify_bootstrap_root(
        &evidence.root_manifest,
        &evidence.current_constitution,
        &evidence.root_adoption,
        now_ms,
    )
    .map_err(|_| EvidenceClosureError::BootstrapRootQualificationDenied)?;

    let mut qualified = Vec::with_capacity(evidence.control_plane_subjects.len());
    for item in &evidence.control_plane_subjects {
        let value = qualify_control_plane_subject_freshness(
            &root,
            &item.challenge_receipt,
            &item.source_receipt,
            &item.witness_receipts,
            &item.trust_bindings,
            &item.transition_receipts,
            now_ms,
        )
        .map_err(|_| EvidenceClosureError::ControlPlaneEvidenceQualificationDenied)?;
        qualified.push(value);
    }

    let (control_plane_digest, control_verified_at_ms, control_valid_until_ms) =
        control_plane_evidence_set_digest(&qualified, now_ms)?;

    let complete_digest = complete_evidence_digest(
        runtime_authority.deployment_qualification_digest(),
        runtime_authority.deployment_evidence_digest(),
        control_plane_digest,
    );
    let verification_ref = format!(
        "complete-deployment-authority:{COMPLETE_EVIDENCE_PROFILE}:{}",
        hex_digest(complete_digest)
    );
    let verified_at_ms = runtime_authority
        .verified_at_ms()
        .max(control_verified_at_ms);
    let valid_until_ms = runtime_authority
        .valid_until_ms()
        .min(control_valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(EvidenceClosureError::CompleteEvidenceNotCurrent);
    }

    let mut projected = runtime_authority.to_verified_freshness();
    projected.verification_ref = verification_ref.clone();
    projected.verified_at_ms = projected.verified_at_ms.max(verified_at_ms);
    projected.lease_until_ms = projected.lease_until_ms.min(valid_until_ms);
    projected
        .validate_at(now_ms)
        .map_err(|_| EvidenceClosureError::CompleteEvidenceNotCurrent)?;

    Ok(QualifiedCompleteDeploymentAuthority {
        runtime_authority,
        control_plane_evidence_digest: control_plane_digest,
        control_plane_evidence_profile: CONTROL_PLANE_EVIDENCE_PROFILE.into(),
        complete_evidence_digest: complete_digest,
        complete_evidence_profile: COMPLETE_EVIDENCE_PROFILE.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn control_plane_evidence_set_digest(
    qualified: &[QualifiedControlPlaneSubjectFreshness],
    now_ms: u64,
) -> Result<(Digest32, u64, u64), EvidenceClosureError> {
    if qualified.is_empty() || qualified.len() > MAX_CONTROL_PLANE_SUBJECTS {
        return Err(EvidenceClosureError::InvalidControlPlaneSubjectCount);
    }

    let mut items = Vec::<(Digest32, Digest32)>::with_capacity(qualified.len());
    let mut verified_at_ms = 0u64;
    let mut valid_until_ms = u64::MAX;

    for proof in qualified {
        let subject_digest = proof
            .subject()
            .identity_digest()
            .map_err(|_| EvidenceClosureError::InvalidControlPlaneSubject)?;
        let receipt = proof.to_verified_freshness();
        receipt
            .validate_at(now_ms)
            .map_err(|_| EvidenceClosureError::ControlPlaneEvidenceNotCurrent)?;
        verified_at_ms = verified_at_ms.max(receipt.verified_at_ms);
        valid_until_ms = valid_until_ms.min(receipt.lease_until_ms);
        items.push((subject_digest, proof.qualification_digest()));
    }

    items.sort_by(|left, right| left.0.0.cmp(&right.0.0));
    for pair in items.windows(2) {
        if pair[0].0 == pair[1].0 {
            return Err(EvidenceClosureError::DuplicateControlPlaneSubject);
        }
    }
    if valid_until_ms <= now_ms {
        return Err(EvidenceClosureError::ControlPlaneEvidenceNotCurrent);
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CONTROL_PLANE);
    frame(&mut hasher, CONTROL_PLANE_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &(items.len() as u64).to_le_bytes());
    for (subject_digest, proof_digest) in items {
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, &proof_digest.0);
    }
    Ok((
        Digest32(*hasher.finalize().as_bytes()),
        verified_at_ms,
        valid_until_ms,
    ))
}

fn complete_evidence_digest(
    deployment_qualification_digest: Digest32,
    deployment_evidence_digest: Digest32,
    control_plane_evidence_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_COMPLETE);
    frame(&mut hasher, COMPLETE_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &deployment_qualification_digest.0);
    frame(&mut hasher, &deployment_evidence_digest.0);
    frame(&mut hasher, CONTROL_PLANE_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &control_plane_evidence_digest.0);
    Digest32(*hasher.finalize().as_bytes())
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
pub enum EvidenceClosureError {
    InvalidControlPlaneSubjectCount,
    RuntimeAuthorityQualificationDenied,
    BootstrapRootQualificationDenied,
    ControlPlaneEvidenceQualificationDenied,
    InvalidControlPlaneSubject,
    ControlPlaneEvidenceNotCurrent,
    DuplicateControlPlaneSubject,
    CompleteEvidenceNotCurrent,
}

impl fmt::Display for EvidenceClosureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidControlPlaneSubjectCount => "invalid control-plane evidence subject count",
            Self::RuntimeAuthorityQualificationDenied => "deployment-bound current authority qualification denied",
            Self::BootstrapRootQualificationDenied => "bootstrap root requalification denied",
            Self::ControlPlaneEvidenceQualificationDenied => "control-plane proof requalification denied",
            Self::InvalidControlPlaneSubject => "invalid control-plane evidence subject identity",
            Self::ControlPlaneEvidenceNotCurrent => "control-plane proof evidence is stale or invalid",
            Self::DuplicateControlPlaneSubject => "duplicate control-plane evidence subject",
            Self::CompleteEvidenceNotCurrent => "complete deployment evidence is stale or invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for EvidenceClosureError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn digest_items(mut items: Vec<(Digest32, Digest32)>) -> Digest32 {
        items.sort_by(|left, right| left.0.0.cmp(&right.0.0));
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CONTROL_PLANE);
        frame(&mut hasher, CONTROL_PLANE_EVIDENCE_PROFILE.as_bytes());
        frame(&mut hasher, &(items.len() as u64).to_le_bytes());
        for (subject, proof) in items {
            frame(&mut hasher, &subject.0);
            frame(&mut hasher, &proof.0);
        }
        Digest32(*hasher.finalize().as_bytes())
    }

    #[test]
    fn control_plane_evidence_identity_is_order_independent() {
        let a = digest_items(vec![(d(1), d(11)), (d(2), d(22)), (d(3), d(33))]);
        let b = digest_items(vec![(d(3), d(33)), (d(1), d(11)), (d(2), d(22))]);
        assert_eq!(a, b);
    }

    #[test]
    fn control_plane_proof_refresh_changes_dynamic_evidence() {
        let baseline = digest_items(vec![(d(1), d(11)), (d(2), d(22))]);
        let refreshed = digest_items(vec![(d(1), d(12)), (d(2), d(22))]);
        assert_ne!(baseline, refreshed);
    }

    #[test]
    fn complete_evidence_changes_without_changing_deployment_authority() {
        let deployment = d(7);
        let target_evidence = d(8);
        let first = complete_evidence_digest(deployment, target_evidence, d(9));
        let second = complete_evidence_digest(deployment, target_evidence, d(10));
        assert_eq!(deployment, d(7));
        assert_ne!(first, second);
    }
}
