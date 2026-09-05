// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Generation-bound current-authority freshness.
//!
//! Immutable authority identity, historical validity, and current freshness are
//! different facts. This crate models the last one without mutating the first.
//! A later verifier may refresh proof/lease evidence for the same generation;
//! that must not change authority identity. A generation/state change must.

use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-freshness-v0.1";
pub const SUBJECT_IDENTITY_PROFILE: &str =
    "mycelix-authority-freshness-subject-v1-blake3-framed";
pub const SNAPSHOT_IDENTITY_PROFILE: &str =
    "mycelix-authority-freshness-snapshot-v1-blake3-framed";
pub const BUNDLE_IDENTITY_PROFILE: &str =
    "mycelix-authority-freshness-bundle-v1-blake3-framed";

const DOMAIN_SUBJECT: &[u8] = b"mycelix/authority-freshness/subject/v1";
const DOMAIN_SNAPSHOT: &[u8] = b"mycelix/authority-freshness/snapshot/v1";
const DOMAIN_BUNDLE: &[u8] = b"mycelix/authority-freshness/bundle/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_SUBJECTS: usize = 64;
const MAX_RECEIPTS: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProfiledDigest {
    pub digest: Digest32,
    pub profile: String,
}

impl ProfiledDigest {
    pub fn validate(&self) -> Result<(), FreshnessError> {
        if self.digest.is_zero() {
            return Err(FreshnessError::ZeroDigest);
        }
        validate_profile(&self.profile)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthoritySubjectKind {
    AuthorityGrant,
    SigningPolicy,
    ThresholdAuthorization,
    ExecutorDesignation,
    EffectSafetyPolicy,
    Delegation,
    AuthorityCoveragePolicy,
    CoverageTrustContextPolicy,
    WitnessTrustPolicy,
}

impl AuthoritySubjectKind {
    fn code(self) -> u8 {
        match self {
            Self::AuthorityGrant => 1,
            Self::SigningPolicy => 2,
            Self::ThresholdAuthorization => 3,
            Self::ExecutorDesignation => 4,
            Self::EffectSafetyPolicy => 5,
            Self::Delegation => 6,
            Self::AuthorityCoveragePolicy => 7,
            Self::CoverageTrustContextPolicy => 8,
            Self::WitnessTrustPolicy => 9,
        }
    }
}

/// Immutable identity of one revocable authority subject.
///
/// `subject_id` alone is intentionally insufficient: the exact canonical
/// identity digest/profile prevents same-ID rebinding from preserving authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthoritySubjectRef {
    pub kind: AuthoritySubjectKind,
    pub namespace: String,
    pub subject_id: String,
    pub identity: ProfiledDigest,
}

impl AuthoritySubjectRef {
    pub fn validate(&self) -> Result<(), FreshnessError> {
        validate_text(&self.namespace)?;
        validate_text(&self.subject_id)?;
        self.identity.validate()
    }

    pub fn identity_digest(&self) -> Result<Digest32, FreshnessError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_SUBJECT);
        frame(&mut hasher, SUBJECT_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, &[self.kind.code()]);
        frame(&mut hasher, self.namespace.as_bytes());
        frame(&mut hasher, self.subject_id.as_bytes());
        frame(&mut hasher, self.identity.profile.as_bytes());
        frame(&mut hasher, &self.identity.digest.0);
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityFreshnessState {
    Active,
    Revoked,
    Superseded,
}

impl AuthorityFreshnessState {
    fn code(self) -> u8 {
        match self {
            Self::Active => 1,
            Self::Revoked => 2,
            Self::Superseded => 3,
        }
    }
}

/// Stable current-state identity for one authority subject.
///
/// Verification timestamps and lease horizons are deliberately outside this
/// structure so refreshing current proof does not create a new authority domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityFreshnessSnapshot {
    pub protocol_version: String,
    pub subject: AuthoritySubjectRef,
    pub generation: u64,
    pub state: AuthorityFreshnessState,
    pub effective_at_ms: u64,
    /// Immutable record/content identity of this exact generation/state fact.
    pub status_record_ref: String,
}

impl AuthorityFreshnessSnapshot {
    pub fn validate(&self) -> Result<(), FreshnessError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(FreshnessError::WrongProtocolVersion);
        }
        self.subject.validate()?;
        if self.generation == 0 {
            return Err(FreshnessError::ZeroGeneration);
        }
        if self.effective_at_ms == 0 {
            return Err(FreshnessError::InvalidEffectiveTime);
        }
        validate_text(&self.status_record_ref)
    }

    pub fn identity_digest(&self) -> Result<Digest32, FreshnessError> {
        self.validate()?;
        let subject_digest = self.subject.identity_digest()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_SNAPSHOT);
        frame(&mut hasher, SNAPSHOT_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, &self.generation.to_le_bytes());
        frame(&mut hasher, &[self.state.code()]);
        frame(&mut hasher, &self.effective_at_ms.to_le_bytes());
        frame(&mut hasher, self.status_record_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Host-attested proof that one stable snapshot is the current authority state.
///
/// These proof/lease fields are freshness evidence, not authority identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityFreshness {
    pub snapshot: AuthorityFreshnessSnapshot,
    pub authoritative_source_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    /// Upper bound on how long this current-state verification may be reused.
    pub lease_until_ms: u64,
}

impl VerifiedAuthorityFreshness {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), FreshnessError> {
        self.snapshot.validate()?;
        validate_text(&self.authoritative_source_ref)?;
        validate_text(&self.verification_ref)?;
        if now_ms == 0
            || self.snapshot.effective_at_ms > now_ms
            || self.verified_at_ms < self.snapshot.effective_at_ms
            || self.verified_at_ms > now_ms
        {
            return Err(FreshnessError::InvalidVerificationTime);
        }
        if self.lease_until_ms <= now_ms || self.lease_until_ms < self.verified_at_ms {
            return Err(FreshnessError::FreshnessLeaseExpired);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedFreshnessSubject {
    pub subject: AuthoritySubjectRef,
    pub generation: u64,
    pub snapshot_identity_digest: Digest32,
    pub status_record_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentAuthorityFreshness {
    pub protocol_version: String,
    pub freshness_digest: Digest32,
    pub freshness_profile: String,
    /// Canonical subject order, independent of input/DHT order.
    pub subjects: Vec<QualifiedFreshnessSubject>,
    /// Dynamic proof metadata. Excluded from `freshness_digest`.
    pub verified_at_ms: u64,
    pub lease_until_ms: u64,
}

#[derive(Clone)]
struct NormalizedReceipt {
    snapshot: AuthorityFreshnessSnapshot,
    snapshot_digest: Digest32,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

/// Qualify the complete exact set of current authority-freshness facts.
///
/// This function deliberately does not choose a "latest" snapshot from history.
/// The runtime authority source must resolve current state and provide it here.
/// Different current snapshots for one required subject fail closed.
pub fn qualify_current_freshness(
    required_subjects: &[AuthoritySubjectRef],
    receipts: &[VerifiedAuthorityFreshness],
    now_ms: u64,
) -> Result<CurrentAuthorityFreshness, FreshnessError> {
    if now_ms == 0 {
        return Err(FreshnessError::InvalidVerificationTime);
    }
    if required_subjects.is_empty() || required_subjects.len() > MAX_SUBJECTS {
        return Err(FreshnessError::InvalidSubjectCount);
    }
    if receipts.len() > MAX_RECEIPTS {
        return Err(FreshnessError::InvalidReceiptCount);
    }

    let mut required = BTreeMap::<Vec<u8>, AuthoritySubjectRef>::new();
    for subject in required_subjects {
        subject.validate()?;
        let key = subject_key(subject)?;
        if required.insert(key, subject.clone()).is_some() {
            return Err(FreshnessError::DuplicateRequirement);
        }
    }

    let mut normalized = BTreeMap::<Vec<u8>, NormalizedReceipt>::new();
    for receipt in receipts {
        receipt.validate_at(now_ms)?;
        let key = subject_key(&receipt.snapshot.subject)?;
        if !required.contains_key(&key) {
            return Err(FreshnessError::UnexpectedSubject);
        }
        let snapshot_digest = receipt.snapshot.identity_digest()?;
        match normalized.get_mut(&key) {
            Some(existing) => {
                if existing.snapshot_digest != snapshot_digest
                    || existing.snapshot != receipt.snapshot
                {
                    return Err(FreshnessError::AmbiguousSubject);
                }
                existing.verified_at_ms = existing.verified_at_ms.max(receipt.verified_at_ms);
                existing.lease_until_ms = existing.lease_until_ms.min(receipt.lease_until_ms);
            }
            None => {
                normalized.insert(
                    key,
                    NormalizedReceipt {
                        snapshot: receipt.snapshot.clone(),
                        snapshot_digest,
                        verified_at_ms: receipt.verified_at_ms,
                        lease_until_ms: receipt.lease_until_ms,
                    },
                );
            }
        }
    }

    if normalized.len() != required.len() {
        return Err(FreshnessError::MissingSubject);
    }

    let mut subjects = Vec::with_capacity(required.len());
    let mut verified_at_ms = 0u64;
    let mut lease_until_ms = u64::MAX;

    for (key, required_subject) in &required {
        let current = normalized.get(key).ok_or(FreshnessError::MissingSubject)?;
        if &current.snapshot.subject != required_subject {
            return Err(FreshnessError::SubjectIdentityMismatch);
        }
        if current.snapshot.state != AuthorityFreshnessState::Active {
            return Err(FreshnessError::SubjectNotActive);
        }
        verified_at_ms = verified_at_ms.max(current.verified_at_ms);
        lease_until_ms = lease_until_ms.min(current.lease_until_ms);
        subjects.push(QualifiedFreshnessSubject {
            subject: current.snapshot.subject.clone(),
            generation: current.snapshot.generation,
            snapshot_identity_digest: current.snapshot_digest,
            status_record_ref: current.snapshot.status_record_ref.clone(),
        });
    }

    if lease_until_ms <= now_ms {
        return Err(FreshnessError::FreshnessLeaseExpired);
    }

    let freshness_digest = bundle_digest(&subjects)?;
    Ok(CurrentAuthorityFreshness {
        protocol_version: PROTOCOL_VERSION.into(),
        freshness_digest,
        freshness_profile: BUNDLE_IDENTITY_PROFILE.into(),
        subjects,
        verified_at_ms,
        lease_until_ms,
    })
}

fn bundle_digest(subjects: &[QualifiedFreshnessSubject]) -> Result<Digest32, FreshnessError> {
    if subjects.is_empty() || subjects.len() > MAX_SUBJECTS {
        return Err(FreshnessError::InvalidSubjectCount);
    }
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_BUNDLE);
    frame(&mut hasher, BUNDLE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &(subjects.len() as u64).to_le_bytes());
    for subject in subjects {
        frame(&mut hasher, &subject.snapshot_identity_digest.0);
    }
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

fn subject_key(subject: &AuthoritySubjectRef) -> Result<Vec<u8>, FreshnessError> {
    let digest = subject.identity_digest()?;
    let mut key = Vec::with_capacity(33);
    key.push(subject.kind.code());
    key.extend_from_slice(&digest.0);
    Ok(key)
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn validate_text(value: &str) -> Result<(), FreshnessError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(FreshnessError::InvalidText)
    } else {
        Ok(())
    }
}

fn validate_profile(value: &str) -> Result<(), FreshnessError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(FreshnessError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FreshnessError {
    WrongProtocolVersion,
    InvalidText,
    InvalidProfile,
    ZeroDigest,
    ZeroGeneration,
    InvalidEffectiveTime,
    InvalidVerificationTime,
    FreshnessLeaseExpired,
    InvalidSubjectCount,
    InvalidReceiptCount,
    DuplicateRequirement,
    UnexpectedSubject,
    MissingSubject,
    AmbiguousSubject,
    SubjectIdentityMismatch,
    SubjectNotActive,
}

impl fmt::Display for FreshnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong authority-freshness protocol version",
            Self::InvalidText => "invalid authority-freshness text field",
            Self::InvalidProfile => "invalid authority-freshness digest profile",
            Self::ZeroDigest => "authority subject identity digest must not be zero",
            Self::ZeroGeneration => "authority freshness generation must be non-zero",
            Self::InvalidEffectiveTime => "invalid authority freshness effective time",
            Self::InvalidVerificationTime => "invalid authority freshness verification time",
            Self::FreshnessLeaseExpired => "authority freshness verification lease is expired",
            Self::InvalidSubjectCount => "authority freshness subject count is invalid",
            Self::InvalidReceiptCount => "authority freshness receipt count exceeds the v0.1 bound",
            Self::DuplicateRequirement => "duplicate authority freshness requirement",
            Self::UnexpectedSubject => "unexpected authority freshness subject",
            Self::MissingSubject => "required authority freshness subject is missing",
            Self::AmbiguousSubject => "multiple different current snapshots exist for one subject",
            Self::SubjectIdentityMismatch => "authority freshness subject identity mismatch",
            Self::SubjectNotActive => "authority freshness subject is not active",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for FreshnessError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject(kind: AuthoritySubjectKind, id: &str, byte: u8) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind,
            namespace: "institution:test".into(),
            subject_id: id.into(),
            identity: ProfiledDigest {
                digest: d(byte),
                profile: "test-identity-v1-blake3".into(),
            },
        }
    }

    fn receipt(subject: AuthoritySubjectRef, generation: u64) -> VerifiedAuthorityFreshness {
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: PROTOCOL_VERSION.into(),
                subject,
                generation,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: 10,
                status_record_ref: format!("status:g{generation}"),
            },
            authoritative_source_ref: "authority-status-source:1".into(),
            verification_ref: "verification:1".into(),
            verified_at_ms: 20,
            lease_until_ms: 40,
        }
    }

    #[test]
    fn exact_current_set_qualifies() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let designation = subject(
            AuthoritySubjectKind::ExecutorDesignation,
            "designation:1",
            2,
        );
        let current = qualify_current_freshness(
            &[grant.clone(), designation.clone()],
            &[receipt(grant, 1), receipt(designation, 1)],
            30,
        )
        .unwrap();
        assert_eq!(current.subjects.len(), 2);
        assert_eq!(current.lease_until_ms, 40);
        assert!(!current.freshness_digest.is_zero());
    }

    #[test]
    fn subject_profile_is_part_of_subject_identity() {
        let a = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let mut b = a.clone();
        b.identity.profile = "test-identity-v2-blake3".into();
        assert_ne!(a.identity_digest().unwrap(), b.identity_digest().unwrap());
    }

    #[test]
    fn subject_kind_codes_are_stable_and_coverage_kinds_are_additive() {
        assert_eq!(AuthoritySubjectKind::AuthorityGrant.code(), 1);
        assert_eq!(AuthoritySubjectKind::SigningPolicy.code(), 2);
        assert_eq!(AuthoritySubjectKind::ThresholdAuthorization.code(), 3);
        assert_eq!(AuthoritySubjectKind::ExecutorDesignation.code(), 4);
        assert_eq!(AuthoritySubjectKind::EffectSafetyPolicy.code(), 5);
        assert_eq!(AuthoritySubjectKind::Delegation.code(), 6);
        assert_eq!(AuthoritySubjectKind::AuthorityCoveragePolicy.code(), 7);
        assert_eq!(AuthoritySubjectKind::CoverageTrustContextPolicy.code(), 8);
        assert_eq!(AuthoritySubjectKind::WitnessTrustPolicy.code(), 9);
    }

    #[test]
    fn input_order_does_not_change_bundle_identity() {
        let a = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let b = subject(AuthoritySubjectKind::ExecutorDesignation, "designation:1", 2);
        let first = qualify_current_freshness(
            &[a.clone(), b.clone()],
            &[receipt(a.clone(), 1), receipt(b.clone(), 1)],
            30,
        )
        .unwrap();
        let second = qualify_current_freshness(
            &[b.clone(), a.clone()],
            &[receipt(b, 1), receipt(a, 1)],
            30,
        )
        .unwrap();
        assert_eq!(first.freshness_digest, second.freshness_digest);
        assert_eq!(first.subjects, second.subjects);
    }

    #[test]
    fn reverification_same_generation_preserves_identity() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let first = qualify_current_freshness(
            std::slice::from_ref(&grant),
            &[receipt(grant.clone(), 1)],
            30,
        )
        .unwrap();
        let mut reverified = receipt(grant.clone(), 1);
        reverified.verification_ref = "verification:later".into();
        reverified.verified_at_ms = 29;
        reverified.lease_until_ms = 50;
        let second = qualify_current_freshness(&[grant], &[reverified], 30).unwrap();
        assert_eq!(first.freshness_digest, second.freshness_digest);
        assert_ne!(first.verified_at_ms, second.verified_at_ms);
        assert_ne!(first.lease_until_ms, second.lease_until_ms);
    }

    #[test]
    fn generation_advance_changes_identity() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let first = qualify_current_freshness(
            std::slice::from_ref(&grant),
            &[receipt(grant.clone(), 1)],
            30,
        )
        .unwrap();
        let second = qualify_current_freshness(
            std::slice::from_ref(&grant),
            &[receipt(grant, 2)],
            30,
        )
        .unwrap();
        assert_ne!(first.freshness_digest, second.freshness_digest);
    }

    #[test]
    fn revoked_subject_denies_current_authority() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let mut revoked = receipt(grant.clone(), 2);
        revoked.snapshot.state = AuthorityFreshnessState::Revoked;
        assert_eq!(
            qualify_current_freshness(&[grant], &[revoked], 30).unwrap_err(),
            FreshnessError::SubjectNotActive
        );
    }

    #[test]
    fn exact_duplicate_receipts_are_harmless() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let a = receipt(grant.clone(), 1);
        let mut b = a.clone();
        b.verification_ref = "verification:second".into();
        b.verified_at_ms = 25;
        b.lease_until_ms = 35;
        let current = qualify_current_freshness(&[grant], &[a, b], 30).unwrap();
        assert_eq!(current.verified_at_ms, 25);
        assert_eq!(current.lease_until_ms, 35);
    }

    #[test]
    fn conflicting_snapshots_for_same_subject_fail_closed() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let a = receipt(grant.clone(), 1);
        let b = receipt(grant.clone(), 2);
        assert_eq!(
            qualify_current_freshness(&[grant], &[a, b], 30).unwrap_err(),
            FreshnessError::AmbiguousSubject
        );
    }

    #[test]
    fn different_status_record_same_generation_is_ambiguous() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let a = receipt(grant.clone(), 1);
        let mut b = a.clone();
        b.snapshot.status_record_ref = "status:conflict".into();
        assert_eq!(
            qualify_current_freshness(&[grant], &[a, b], 30).unwrap_err(),
            FreshnessError::AmbiguousSubject
        );
    }

    #[test]
    fn missing_and_unexpected_subjects_deny() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let designation = subject(
            AuthoritySubjectKind::ExecutorDesignation,
            "designation:1",
            2,
        );
        assert_eq!(
            qualify_current_freshness(std::slice::from_ref(&grant), &[], 30).unwrap_err(),
            FreshnessError::MissingSubject
        );
        assert_eq!(
            qualify_current_freshness(&[grant], &[receipt(designation, 1)], 30).unwrap_err(),
            FreshnessError::UnexpectedSubject
        );
    }

    #[test]
    fn excessive_receipt_fan_in_is_rejected() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let receipts = vec![receipt(grant.clone(), 1); MAX_RECEIPTS + 1];
        assert_eq!(
            qualify_current_freshness(&[grant], &receipts, 30).unwrap_err(),
            FreshnessError::InvalidReceiptCount
        );
    }

    #[test]
    fn expired_freshness_lease_denies() {
        let grant = subject(AuthoritySubjectKind::AuthorityGrant, "grant:1", 1);
        let mut stale = receipt(grant.clone(), 1);
        stale.lease_until_ms = 30;
        assert_eq!(
            qualify_current_freshness(&[grant], &[stale], 30).unwrap_err(),
            FreshnessError::FreshnessLeaseExpired
        );
    }
}
