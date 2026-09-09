// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Closed-set qualification for exact historical operational authority requirements.
//!
//! This layer composes opaque #481 `QualifiedHistoricalOperationalAuthority` capabilities.
//! It does not discover or create authority. Callers provide an exact closed requirement set
//! `(AuthoritySubjectRef, generation, transition digest)` and this theorem requires a one-to-one
//! match to already-qualified historical operational authority proofs.
//!
//! Input order is never authority. Requirements and proofs are canonicalized by exact subject
//! identity digest, generation, and transition digest. Missing, extra, duplicate, cross-root,
//! stale, or mismatched proofs fail closed.

#![forbid(unsafe_code)]

use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_authority_historical_operational::QualifiedHistoricalOperationalAuthority;
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-historical-operational-set-v0.1";
pub const SET_AUTHORITY_IDENTITY_PROFILE: &str =
    "mycelix-authority-historical-operational-set-v1-blake3-framed";
pub const SET_EVIDENCE_IDENTITY_PROFILE: &str =
    "mycelix-authority-historical-operational-set-evidence-v1-blake3-framed";
pub const MAX_HISTORICAL_OPERATIONAL_REQUIREMENTS: usize = 4096;

const DOMAIN_SET_AUTHORITY: &[u8] = b"mycelix/authority/historical-operational-set/v1";
const DOMAIN_SET_EVIDENCE: &[u8] =
    b"mycelix/authority/historical-operational-set-evidence/v1";

type RequirementKey = ([u8; 32], u64, [u8; 32]);

/// Transportable requirement only. This is not positive authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoricalOperationalAuthorityRequirement {
    pub subject: AuthoritySubjectRef,
    pub target_generation: u64,
    pub target_transition_digest: Digest32,
}

impl HistoricalOperationalAuthorityRequirement {
    fn key(&self) -> Result<RequirementKey, HistoricalOperationalAuthoritySetError> {
        if self.target_generation == 0 {
            return Err(HistoricalOperationalAuthoritySetError::ZeroGeneration);
        }
        if self.target_transition_digest.is_zero() {
            return Err(HistoricalOperationalAuthoritySetError::ZeroTransitionDigest);
        }
        let subject_digest = self
            .subject
            .identity_digest()
            .map_err(|_| HistoricalOperationalAuthoritySetError::InvalidSubject)?;
        Ok((
            subject_digest.0,
            self.target_generation,
            self.target_transition_digest.0,
        ))
    }
}

/// One exact requirement/proof pair retained inside the qualified closed set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedHistoricalOperationalAuthorityItem {
    subject: AuthoritySubjectRef,
    subject_identity_digest: Digest32,
    target_generation: u64,
    target_transition_digest: Digest32,
    authority_digest: Digest32,
    authority_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
}

impl QualifiedHistoricalOperationalAuthorityItem {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn subject_identity_digest(&self) -> Digest32 {
        self.subject_identity_digest
    }

    pub fn target_generation(&self) -> u64 {
        self.target_generation
    }

    pub fn target_transition_digest(&self) -> Digest32 {
        self.target_transition_digest
    }

    pub fn authority_digest(&self) -> Digest32 {
        self.authority_digest
    }

    pub fn authority_profile(&self) -> &str {
        &self.authority_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }
}

/// Non-deserializable exact closed-set historical operational authority result.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedHistoricalOperationalAuthoritySet {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    items: Vec<QualifiedHistoricalOperationalAuthorityItem>,
    qualification_digest: Digest32,
    qualification_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedHistoricalOperationalAuthoritySet {
    pub fn root_qualification_digest(&self) -> Digest32 {
        self.root_qualification_digest
    }

    pub fn root_qualification_profile(&self) -> &str {
        &self.root_qualification_profile
    }

    pub fn items(&self) -> &[QualifiedHistoricalOperationalAuthorityItem] {
        &self.items
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

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

/// Require an exact closed set of historical operational authority proofs.
pub fn qualify_historical_operational_authority_set(
    requirements: &[HistoricalOperationalAuthorityRequirement],
    qualified: &[&QualifiedHistoricalOperationalAuthority],
    now_ms: u64,
) -> Result<QualifiedHistoricalOperationalAuthoritySet, HistoricalOperationalAuthoritySetError> {
    if now_ms == 0 {
        return Err(HistoricalOperationalAuthoritySetError::InvalidVerificationTime);
    }
    if requirements.is_empty()
        || requirements.len() > MAX_HISTORICAL_OPERATIONAL_REQUIREMENTS
    {
        return Err(HistoricalOperationalAuthoritySetError::InvalidRequirementCount);
    }
    if qualified.len() != requirements.len() {
        return Err(HistoricalOperationalAuthoritySetError::QualifiedSetSizeMismatch);
    }

    let mut required_by_key = BTreeMap::<RequirementKey, &HistoricalOperationalAuthorityRequirement>::new();
    for requirement in requirements {
        let key = requirement.key()?;
        if required_by_key.insert(key, requirement).is_some() {
            return Err(HistoricalOperationalAuthoritySetError::DuplicateRequirement);
        }
    }

    let first = qualified
        .first()
        .ok_or(HistoricalOperationalAuthoritySetError::QualifiedSetSizeMismatch)?;
    let root_digest = first.root_qualification_digest();
    let root_profile = first.root_qualification_profile();

    let mut qualified_by_key = BTreeMap::<RequirementKey, &QualifiedHistoricalOperationalAuthority>::new();
    for proof in qualified {
        if proof.verified_at_ms() > now_ms || proof.lease_until_ms() <= now_ms {
            return Err(HistoricalOperationalAuthoritySetError::QualifiedProofStale);
        }
        if proof.root_qualification_digest() != root_digest
            || proof.root_qualification_profile() != root_profile
        {
            return Err(HistoricalOperationalAuthoritySetError::BootstrapRootMismatch);
        }

        let subject_digest = proof
            .subject()
            .identity_digest()
            .map_err(|_| HistoricalOperationalAuthoritySetError::InvalidQualifiedSubject)?;
        let key = (
            subject_digest.0,
            proof.target_generation(),
            proof.selected_transition_digest().0,
        );
        let Some(requirement) = required_by_key.get(&key) else {
            return Err(HistoricalOperationalAuthoritySetError::UnexpectedQualifiedProof);
        };
        if proof.subject() != &requirement.subject
            || proof.target_generation() != requirement.target_generation
            || proof.selected_transition_digest() != requirement.target_transition_digest
        {
            return Err(HistoricalOperationalAuthoritySetError::QualifiedBindingMismatch);
        }
        if qualified_by_key.insert(key, *proof).is_some() {
            return Err(HistoricalOperationalAuthoritySetError::DuplicateQualifiedProof);
        }
    }

    if qualified_by_key.len() != required_by_key.len() {
        return Err(HistoricalOperationalAuthoritySetError::MissingQualifiedProof);
    }

    let mut items = Vec::with_capacity(required_by_key.len());
    let mut verified_at_ms = 0u64;
    let mut lease_until_ms = u64::MAX;

    for (key, requirement) in &required_by_key {
        let proof = qualified_by_key
            .get(key)
            .ok_or(HistoricalOperationalAuthoritySetError::MissingQualifiedProof)?;
        let subject_identity_digest = requirement
            .subject
            .identity_digest()
            .map_err(|_| HistoricalOperationalAuthoritySetError::InvalidSubject)?;
        verified_at_ms = verified_at_ms.max(proof.verified_at_ms());
        lease_until_ms = lease_until_ms.min(proof.lease_until_ms());
        items.push(QualifiedHistoricalOperationalAuthorityItem {
            subject: requirement.subject.clone(),
            subject_identity_digest,
            target_generation: requirement.target_generation,
            target_transition_digest: requirement.target_transition_digest,
            authority_digest: proof.authority_digest(),
            authority_profile: proof.authority_profile().into(),
            evidence_digest: proof.evidence_digest(),
            evidence_profile: proof.evidence_profile().into(),
        });
    }

    if verified_at_ms > now_ms || lease_until_ms <= now_ms {
        return Err(HistoricalOperationalAuthoritySetError::QualifiedSetStale);
    }

    let qualification_digest = derive_set_authority_digest(root_digest, root_profile, &items);
    let evidence_digest = derive_set_evidence_digest(qualification_digest, &items);

    Ok(QualifiedHistoricalOperationalAuthoritySet {
        root_qualification_digest: root_digest,
        root_qualification_profile: root_profile.into(),
        items,
        qualification_digest,
        qualification_profile: SET_AUTHORITY_IDENTITY_PROFILE.into(),
        evidence_digest,
        evidence_profile: SET_EVIDENCE_IDENTITY_PROFILE.into(),
        verified_at_ms,
        lease_until_ms,
    })
}

fn derive_set_authority_digest(
    root_digest: Digest32,
    root_profile: &str,
    items: &[QualifiedHistoricalOperationalAuthorityItem],
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SET_AUTHORITY);
    frame(&mut hasher, SET_AUTHORITY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, root_profile.as_bytes());
    frame(&mut hasher, &(items.len() as u64).to_le_bytes());
    for item in items {
        frame(&mut hasher, &item.subject_identity_digest.0);
        frame(&mut hasher, &item.target_generation.to_le_bytes());
        frame(&mut hasher, &item.target_transition_digest.0);
        frame(&mut hasher, item.authority_profile.as_bytes());
        frame(&mut hasher, &item.authority_digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn derive_set_evidence_digest(
    qualification_digest: Digest32,
    items: &[QualifiedHistoricalOperationalAuthorityItem],
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SET_EVIDENCE);
    frame(&mut hasher, SET_EVIDENCE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    for item in items {
        frame(&mut hasher, item.evidence_profile.as_bytes());
        frame(&mut hasher, &item.evidence_digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HistoricalOperationalAuthoritySetError {
    InvalidVerificationTime,
    InvalidRequirementCount,
    QualifiedSetSizeMismatch,
    InvalidSubject,
    ZeroGeneration,
    ZeroTransitionDigest,
    DuplicateRequirement,
    QualifiedProofStale,
    BootstrapRootMismatch,
    InvalidQualifiedSubject,
    UnexpectedQualifiedProof,
    QualifiedBindingMismatch,
    DuplicateQualifiedProof,
    MissingQualifiedProof,
    QualifiedSetStale,
}

impl fmt::Display for HistoricalOperationalAuthoritySetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidVerificationTime => "invalid historical operational set verification time",
            Self::InvalidRequirementCount => "historical operational requirement count is invalid",
            Self::QualifiedSetSizeMismatch => "qualified historical authority count does not match requirements",
            Self::InvalidSubject => "historical operational requirement subject identity is invalid",
            Self::ZeroGeneration => "historical operational requirement generation must be non-zero",
            Self::ZeroTransitionDigest => "historical operational requirement transition digest must be non-zero",
            Self::DuplicateRequirement => "duplicate historical operational authority requirement",
            Self::QualifiedProofStale => "qualified historical operational proof is stale or not yet valid",
            Self::BootstrapRootMismatch => "qualified historical operational proofs use different current verification roots",
            Self::InvalidQualifiedSubject => "qualified historical operational subject identity is invalid",
            Self::UnexpectedQualifiedProof => "qualified historical operational proof was not requested",
            Self::QualifiedBindingMismatch => "qualified historical operational proof does not exactly match its requirement",
            Self::DuplicateQualifiedProof => "duplicate qualified historical operational proof",
            Self::MissingQualifiedProof => "required historical operational proof is missing",
            Self::QualifiedSetStale => "combined historical operational proof set is stale",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for HistoricalOperationalAuthoritySetError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject(byte: u8) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::SigningPolicy,
            namespace: "identity:time-policy:transition-authority".into(),
            subject_id: format!("signer:{byte}@generation:1"),
            identity: ProfiledDigest {
                digest: d(byte),
                profile: "mycelix-identity-time-policy-transition-signer-authority-v2-sha256-framed"
                    .into(),
            },
        }
    }

    fn item(byte: u8) -> QualifiedHistoricalOperationalAuthorityItem {
        QualifiedHistoricalOperationalAuthorityItem {
            subject: subject(byte),
            subject_identity_digest: subject(byte).identity_digest().unwrap(),
            target_generation: byte as u64,
            target_transition_digest: d(20 + byte),
            authority_digest: d(40 + byte),
            authority_profile: "historical-authority-v1".into(),
            evidence_digest: d(60 + byte),
            evidence_profile: "historical-evidence-v1".into(),
        }
    }

    #[test]
    fn canonical_set_identity_is_order_independent_once_items_are_sorted() {
        let mut left = vec![item(1), item(2)];
        let mut right = vec![item(2), item(1)];
        let key = |item: &QualifiedHistoricalOperationalAuthorityItem| {
            (
                item.subject_identity_digest.0,
                item.target_generation,
                item.target_transition_digest.0,
            )
        };
        left.sort_by_key(key);
        right.sort_by_key(key);
        assert_eq!(
            derive_set_authority_digest(d(9), "root-v1", &left),
            derive_set_authority_digest(d(9), "root-v1", &right)
        );
        assert_eq!(
            derive_set_evidence_digest(d(8), &left),
            derive_set_evidence_digest(d(8), &right)
        );
    }

    #[test]
    fn root_or_any_exact_requirement_authority_changes_set_identity() {
        let baseline = vec![item(1), item(2)];
        let baseline_digest = derive_set_authority_digest(d(9), "root-v1", &baseline);
        assert_ne!(
            baseline_digest,
            derive_set_authority_digest(d(8), "root-v1", &baseline)
        );
        let mut changed = baseline.clone();
        changed[1].authority_digest = d(99);
        assert_ne!(
            baseline_digest,
            derive_set_authority_digest(d(9), "root-v1", &changed)
        );
    }

    #[test]
    fn positive_set_type_is_not_deserializable_by_derivation() {
        let source = include_str!("lib.rs");
        let before = source
            .split("pub struct QualifiedHistoricalOperationalAuthoritySet")
            .next()
            .unwrap();
        let derive = before.rsplit("#[derive(").next().unwrap_or_default();
        assert!(!derive.contains("Deserialize"));
    }
}
