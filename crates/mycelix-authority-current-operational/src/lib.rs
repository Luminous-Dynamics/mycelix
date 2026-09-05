// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! End-to-end non-circular qualification of one current operational authority.
//!
//! Runtime/provider boundaries may transport evidence-shaped data, but positive
//! authority objects from the pure layers are intentionally non-deserializable.
//! This crate solves that adapter problem by accepting only candidate evidence and
//! re-running the complete pure chain in one process:
//!
//! constitution/root -> control-plane freshness -> operational policy context
//! -> operational subject freshness.
//!
//! It performs no Holochain calls, DHT selection, persistence, signature checking,
//! lifecycle mutation, or external effects.

use mycelix_authority_control_plane_freshness::{
    qualify_control_plane_subject_freshness, QualifiedControlPlaneSubjectFreshness,
};
use mycelix_authority_freshness::{AuthoritySubjectRef, VerifiedAuthorityFreshness};
use mycelix_authority_operational_context::{
    qualify_operational_policy_context, QualifiedOperationalPolicyContext,
};
use mycelix_authority_operational_freshness::{
    qualify_operational_subject_freshness, QualifiedOperationalSubjectFreshness,
};
use mycelix_authority_state_bootstrap_root::{
    qualify_bootstrap_root, AuthorityStateBootstrapRootManifest,
    QualifiedAuthorityStateBootstrapRoot, VerifiedBootstrapRootAdoption,
    VerifiedCurrentConstitutionReceipt,
};
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    VerifiedCoverageChallenge, VerifiedCoverageTrustContextPolicy, VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::VerifiedAuthorityStateTransition;
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-current-operational-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-current-operational-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-authority-current-operational-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority/current-operational/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority/current-operational-evidence/v1";
const MAX_CONTROL_PLANE_SUBJECTS: usize = 3;
const MAX_WITNESSES: usize = 64;
const MAX_TRANSITIONS: usize = 256;

/// Candidate evidence for one exact authority-state subject.
///
/// Every field remains evidence-shaped data. This struct is intentionally
/// deserializable for adapters; deserialization never creates positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoveredSubjectEvidence {
    pub challenge_receipt: VerifiedCoverageChallenge,
    pub source_receipt: VerifiedAuthoritySourceHead,
    pub witness_receipts: Vec<VerifiedAuthorityHeadWitness>,
    pub trust_bindings: Vec<VerifiedWitnessTrustBinding>,
    pub transition_receipts: Vec<VerifiedAuthorityStateTransition>,
}

impl CoveredSubjectEvidence {
    fn validate_bounds(&self) -> Result<(), CurrentOperationalAuthorityError> {
        if self.witness_receipts.len() > MAX_WITNESSES
            || self.trust_bindings.len() > MAX_WITNESSES
        {
            return Err(CurrentOperationalAuthorityError::EvidenceFanInExceeded);
        }
        if self.transition_receipts.is_empty()
            || self.transition_receipts.len() > MAX_TRANSITIONS
        {
            return Err(CurrentOperationalAuthorityError::InvalidTransitionCount);
        }
        Ok(())
    }
}

/// Candidate semantic policy evidence for the target operational subject.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OperationalPolicyEvidence {
    pub context_receipt: VerifiedCoverageTrustContextPolicy,
    pub coverage_receipt: VerifiedAuthorityCoveragePolicy,
}

/// Complete evidence-shaped input needed to reconstruct one current operational
/// authority from the constitutional root down to the target state head.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentOperationalAuthorityEvidence {
    pub root_manifest: AuthorityStateBootstrapRootManifest,
    pub current_constitution: VerifiedCurrentConstitutionReceipt,
    pub root_adoption: VerifiedBootstrapRootAdoption,
    pub target_subject: AuthoritySubjectRef,
    /// Evidence proving currentness of the exact operational policy objects.
    /// The eventual exact 2/3-subject requirement is reconstructed by #116.
    pub control_plane_subjects: Vec<CoveredSubjectEvidence>,
    pub operational_policy: OperationalPolicyEvidence,
    /// Fresh coverage + complete state history for the target operational subject.
    pub target_currentness: CoveredSubjectEvidence,
}

/// Non-deserializable end-to-end result. A runtime can only obtain this object by
/// re-running every pure qualification layer from candidate evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentOperationalAuthority {
    target_subject: AuthoritySubjectRef,
    bootstrap_root_digest: Digest32,
    bootstrap_root_profile: String,
    operational_context_digest: Digest32,
    operational_context_profile: String,
    operational_authority_digest: Digest32,
    operational_authority_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    qualification_digest: Digest32,
    qualification_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCurrentOperationalAuthority {
    pub fn target_subject(&self) -> &AuthoritySubjectRef {
        &self.target_subject
    }

    pub fn bootstrap_root_digest(&self) -> Digest32 {
        self.bootstrap_root_digest
    }

    pub fn bootstrap_root_profile(&self) -> &str {
        &self.bootstrap_root_profile
    }

    pub fn operational_context_digest(&self) -> Digest32 {
        self.operational_context_digest
    }

    pub fn operational_context_profile(&self) -> &str {
        &self.operational_context_profile
    }

    pub fn operational_authority_digest(&self) -> Digest32 {
        self.operational_authority_digest
    }

    pub fn operational_authority_profile(&self) -> &str {
        &self.operational_authority_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    /// ABI projection for existing downstream freshness consumers. The stable
    /// snapshot remains generation-bound while verification_ref carries exact
    /// current evidence provenance from #117.
    pub fn to_verified_freshness(&self) -> VerifiedAuthorityFreshness {
        self.current_freshness.clone()
    }
}

/// Reconstruct one current operational authority from evidence-shaped input.
pub fn qualify_current_operational_authority(
    evidence: &CurrentOperationalAuthorityEvidence,
    now_ms: u64,
) -> Result<QualifiedCurrentOperationalAuthority, CurrentOperationalAuthorityError> {
    if now_ms == 0 {
        return Err(CurrentOperationalAuthorityError::InvalidVerificationTime);
    }
    evidence
        .target_subject
        .validate()
        .map_err(|_| CurrentOperationalAuthorityError::InvalidTargetSubject)?;
    if evidence.control_plane_subjects.is_empty()
        || evidence.control_plane_subjects.len() > MAX_CONTROL_PLANE_SUBJECTS
    {
        return Err(CurrentOperationalAuthorityError::InvalidControlPlaneSubjectCount);
    }
    for item in &evidence.control_plane_subjects {
        item.validate_bounds()?;
    }
    evidence.target_currentness.validate_bounds()?;

    let root = qualify_bootstrap_root(
        &evidence.root_manifest,
        &evidence.current_constitution,
        &evidence.root_adoption,
        now_ms,
    )
    .map_err(|_| CurrentOperationalAuthorityError::BootstrapRootQualificationDenied)?;

    let control_plane = qualify_control_plane_subjects(
        &root,
        &evidence.control_plane_subjects,
        now_ms,
    )?;

    let operational_context = qualify_operational_policy_context(
        &root,
        &evidence.target_subject,
        &evidence.operational_policy.context_receipt,
        &evidence.operational_policy.coverage_receipt,
        &control_plane,
        now_ms,
    )
    .map_err(|_| CurrentOperationalAuthorityError::OperationalContextQualificationDenied)?;

    let operational = qualify_target_currentness(
        &operational_context,
        &evidence.target_currentness,
        now_ms,
    )?;

    if operational.subject() != &evidence.target_subject
        || operational.root_qualification_digest() != root.qualification_digest()
        || operational.root_qualification_profile() != root.qualification_profile()
    {
        return Err(CurrentOperationalAuthorityError::QualifiedTargetMismatch);
    }

    let current_freshness = operational.to_verified_freshness();
    current_freshness
        .validate_at(now_ms)
        .map_err(|_| CurrentOperationalAuthorityError::QualifiedTargetNotCurrent)?;

    let qualification_digest = stable_qualification_digest(
        root.qualification_digest(),
        operational_context.qualification_digest(),
        operational.authority_digest(),
    );
    let evidence_digest = evidence_qualification_digest(
        qualification_digest,
        operational.evidence_digest(),
    );
    let verified_at_ms = current_freshness
        .verified_at_ms
        .max(root.verified_at_ms())
        .max(operational_context.verified_at_ms());
    let valid_until_ms = current_freshness
        .lease_until_ms
        .min(root.valid_until_ms())
        .min(operational_context.valid_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(CurrentOperationalAuthorityError::QualifiedTargetNotCurrent);
    }

    Ok(QualifiedCurrentOperationalAuthority {
        target_subject: evidence.target_subject.clone(),
        bootstrap_root_digest: root.qualification_digest(),
        bootstrap_root_profile: root.qualification_profile().into(),
        operational_context_digest: operational_context.qualification_digest(),
        operational_context_profile: operational_context.qualification_profile().into(),
        operational_authority_digest: operational.authority_digest(),
        operational_authority_profile: operational.authority_profile().into(),
        current_freshness,
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn qualify_control_plane_subjects(
    root: &QualifiedAuthorityStateBootstrapRoot,
    evidence: &[CoveredSubjectEvidence],
    now_ms: u64,
) -> Result<Vec<QualifiedControlPlaneSubjectFreshness>, CurrentOperationalAuthorityError> {
    let mut qualified = Vec::with_capacity(evidence.len());
    for item in evidence {
        let value = qualify_control_plane_subject_freshness(
            root,
            &item.challenge_receipt,
            &item.source_receipt,
            &item.witness_receipts,
            &item.trust_bindings,
            &item.transition_receipts,
            now_ms,
        )
        .map_err(|_| CurrentOperationalAuthorityError::ControlPlaneFreshnessDenied)?;
        qualified.push(value);
    }
    Ok(qualified)
}

fn qualify_target_currentness(
    context: &QualifiedOperationalPolicyContext,
    evidence: &CoveredSubjectEvidence,
    now_ms: u64,
) -> Result<QualifiedOperationalSubjectFreshness, CurrentOperationalAuthorityError> {
    qualify_operational_subject_freshness(
        context,
        &evidence.challenge_receipt,
        &evidence.source_receipt,
        &evidence.witness_receipts,
        &evidence.trust_bindings,
        &evidence.transition_receipts,
        now_ms,
    )
    .map_err(|_| CurrentOperationalAuthorityError::OperationalFreshnessDenied)
}

fn stable_qualification_digest(
    root_digest: Digest32,
    operational_context_digest: Digest32,
    operational_authority_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &operational_context_digest.0);
    frame(&mut hasher, &operational_authority_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_qualification_digest(
    qualification_digest: Digest32,
    operational_evidence_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, &operational_evidence_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CurrentOperationalAuthorityError {
    InvalidVerificationTime,
    InvalidTargetSubject,
    InvalidControlPlaneSubjectCount,
    EvidenceFanInExceeded,
    InvalidTransitionCount,
    BootstrapRootQualificationDenied,
    ControlPlaneFreshnessDenied,
    OperationalContextQualificationDenied,
    OperationalFreshnessDenied,
    QualifiedTargetMismatch,
    QualifiedTargetNotCurrent,
}

impl fmt::Display for CurrentOperationalAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidVerificationTime => "invalid current operational authority verification time",
            Self::InvalidTargetSubject => "invalid current operational authority target subject",
            Self::InvalidControlPlaneSubjectCount => "invalid control-plane subject evidence count",
            Self::EvidenceFanInExceeded => "authority evidence fan-in exceeds v0.1 bound",
            Self::InvalidTransitionCount => "authority transition evidence count is invalid",
            Self::BootstrapRootQualificationDenied => "constitution-rooted bootstrap qualification denied",
            Self::ControlPlaneFreshnessDenied => "control-plane policy freshness qualification denied",
            Self::OperationalContextQualificationDenied => "current operational policy-context qualification denied",
            Self::OperationalFreshnessDenied => "covered operational subject freshness qualification denied",
            Self::QualifiedTargetMismatch => "qualified current authority does not match requested target/root",
            Self::QualifiedTargetNotCurrent => "qualified operational authority is stale or invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for CurrentOperationalAuthorityError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn stable_end_to_end_identity_excludes_fresh_evidence_instance() {
        let stable_a = stable_qualification_digest(d(1), d(2), d(3));
        let stable_b = stable_qualification_digest(d(1), d(2), d(3));
        assert_eq!(stable_a, stable_b);

        let evidence_a = evidence_qualification_digest(stable_a, d(4));
        let evidence_b = evidence_qualification_digest(stable_b, d(5));
        assert_ne!(evidence_a, evidence_b);
    }

    #[test]
    fn root_policy_or_subject_authority_change_changes_stable_identity() {
        let baseline = stable_qualification_digest(d(1), d(2), d(3));
        assert_ne!(baseline, stable_qualification_digest(d(9), d(2), d(3)));
        assert_ne!(baseline, stable_qualification_digest(d(1), d(8), d(3)));
        assert_ne!(baseline, stable_qualification_digest(d(1), d(2), d(7)));
    }
}
