// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Monotone dynamic-evidence lease composition and provenance commitments.
//!
//! A verifier lease is dynamic evidence, not semantic authority identity. This
//! crate carries verifier horizons across compatibility ABIs and can additionally
//! commit the exact typed evidence contributors whose live windows produced one
//! aggregate lease. Neither a lease nor its provenance manifest grants authority.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-evidence-lease-v0.2";
pub const MANIFEST_PROFILE: &str =
    "mycelix-authority-evidence-lease-manifest-v2-blake3-framed";

const DOMAIN_CONTRIBUTION: &[u8] = b"mycelix/authority/evidence-lease/contribution/v2";
const DOMAIN_MANIFEST: &[u8] = b"mycelix/authority/evidence-lease/manifest/v2";
const MAX_PROFILE_BYTES: usize = 128;
const MAX_REF_BYTES: usize = 2048;
const MAX_CONTRIBUTIONS: usize = 1024;

/// Dynamic proof-usability interval.
///
/// `verified_at_ms` is the latest verification event on which the enclosed
/// evidence depends. `valid_until_ms` is the first instant at which reuse must
/// fail closed. The interval is therefore `[verified_at_ms, valid_until_ms)`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceLease {
    pub protocol_version: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl EvidenceLease {
    pub fn new(
        verified_at_ms: u64,
        valid_until_ms: u64,
        now_ms: u64,
    ) -> Result<Self, EvidenceLeaseError> {
        let lease = Self {
            protocol_version: PROTOCOL_VERSION.into(),
            verified_at_ms,
            valid_until_ms,
        };
        lease.validate_at(now_ms)?;
        Ok(lease)
    }

    pub fn validate_at(&self, now_ms: u64) -> Result<(), EvidenceLeaseError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(EvidenceLeaseError::WrongProtocolVersion);
        }
        if now_ms == 0 || self.verified_at_ms == 0 {
            return Err(EvidenceLeaseError::InvalidVerificationTime);
        }
        if self.verified_at_ms > now_ms {
            return Err(EvidenceLeaseError::VerificationFromFuture);
        }
        if self.valid_until_ms <= self.verified_at_ms {
            return Err(EvidenceLeaseError::InvertedLease);
        }
        if self.valid_until_ms <= now_ms {
            return Err(EvidenceLeaseError::ExpiredLease);
        }
        Ok(())
    }

    /// Intersect two evidence leases. This operation is monotone:
    /// verification time can only move later and validity can only move earlier.
    pub fn intersect(
        &self,
        other: &Self,
        now_ms: u64,
    ) -> Result<Self, EvidenceLeaseError> {
        self.validate_at(now_ms)?;
        other.validate_at(now_ms)?;
        Self::new(
            self.verified_at_ms.max(other.verified_at_ms),
            self.valid_until_ms.min(other.valid_until_ms),
            now_ms,
        )
    }

    /// Intersect a non-empty collection of evidence leases.
    pub fn intersect_all<'a, I>(leases: I, now_ms: u64) -> Result<Self, EvidenceLeaseError>
    where
        I: IntoIterator<Item = &'a EvidenceLease>,
    {
        let mut iter = leases.into_iter();
        let first = iter.next().ok_or(EvidenceLeaseError::EmptyLeaseSet)?;
        first.validate_at(now_ms)?;
        let mut result = first.clone();
        for lease in iter {
            result = result.intersect(lease, now_ms)?;
        }
        Ok(result)
    }

    /// Add an upper bound without ever extending the existing lease.
    pub fn cap_valid_until(
        &self,
        cap_valid_until_ms: u64,
        now_ms: u64,
    ) -> Result<Self, EvidenceLeaseError> {
        self.validate_at(now_ms)?;
        Self::new(
            self.verified_at_ms,
            self.valid_until_ms.min(cap_valid_until_ms),
            now_ms,
        )
    }
}

/// Transport-only pairing of evidence with the dynamic lease under which that
/// evidence may be reused. The envelope does not itself create authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeasedEvidence<T> {
    pub evidence: T,
    pub lease: EvidenceLease,
}

impl<T> LeasedEvidence<T> {
    pub fn new(
        evidence: T,
        lease: EvidenceLease,
        now_ms: u64,
    ) -> Result<Self, EvidenceLeaseError> {
        lease.validate_at(now_ms)?;
        Ok(Self { evidence, lease })
    }
}

/// Closed role vocabulary for authority-runtime lease provenance. Roles identify
/// why a dependency constrains reuse; they do not decide whether that dependency
/// is semantically or institutionally authoritative.
///
/// Codes 1-9 are retained from v0.1. New roles append codes rather than
/// renumbering historical meanings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EvidenceLeaseRole {
    BootstrapRoot,
    OperationalCoveragePolicy,
    OperationalContextPolicy,
    SourceHead,
    WitnessObservation,
    WitnessTrustBinding,
    AuthorityStateTransition,
    ControlPlaneFreshness,
    OperationalFreshness,
    CurrentConstitution,
    BootstrapRootAdoption,
}

impl EvidenceLeaseRole {
    fn code(self) -> u8 {
        match self {
            Self::BootstrapRoot => 1,
            Self::OperationalCoveragePolicy => 2,
            Self::OperationalContextPolicy => 3,
            Self::SourceHead => 4,
            Self::WitnessObservation => 5,
            Self::WitnessTrustBinding => 6,
            Self::AuthorityStateTransition => 7,
            Self::ControlPlaneFreshness => 8,
            Self::OperationalFreshness => 9,
            Self::CurrentConstitution => 10,
            Self::BootstrapRootAdoption => 11,
        }
    }
}

/// One exact dynamic-evidence dependency of a larger authority composition.
///
/// `evidence_digest/profile/ref` identify the already-qualified evidence surface;
/// `lease` states only its current reuse horizon. This object is audit evidence,
/// not a permission, currentness, quorum, or execution grant.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceLeaseContribution {
    pub role: EvidenceLeaseRole,
    pub evidence_digest: [u8; 32],
    pub evidence_profile: String,
    pub evidence_ref: String,
    pub lease: EvidenceLease,
}

impl EvidenceLeaseContribution {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), EvidenceLeaseError> {
        if self.evidence_digest.iter().all(|byte| *byte == 0) {
            return Err(EvidenceLeaseError::ZeroEvidenceDigest);
        }
        require_profile(&self.evidence_profile)?;
        require_ref(&self.evidence_ref)?;
        self.lease.validate_at(now_ms)
    }

    pub fn identity_digest(&self) -> Result<[u8; 32], EvidenceLeaseError> {
        if self.evidence_digest.iter().all(|byte| *byte == 0) {
            return Err(EvidenceLeaseError::ZeroEvidenceDigest);
        }
        require_profile(&self.evidence_profile)?;
        require_ref(&self.evidence_ref)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CONTRIBUTION);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, &[self.role.code()]);
        frame(&mut hasher, self.evidence_profile.as_bytes());
        frame(&mut hasher, &self.evidence_digest);
        frame(&mut hasher, self.evidence_ref.as_bytes());
        frame(&mut hasher, self.lease.protocol_version.as_bytes());
        frame(&mut hasher, &self.lease.verified_at_ms.to_le_bytes());
        frame(&mut hasher, &self.lease.valid_until_ms.to_le_bytes());
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Non-deserializable proof that one exact canonical contributor set was live at
/// qualification time and that its aggregate lease is the intersection of every
/// member. The manifest digest is audit provenance, never authority identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedEvidenceLeaseManifest {
    manifest_digest: [u8; 32],
    manifest_profile: String,
    contributor_count: u32,
    aggregate_lease: EvidenceLease,
}

impl QualifiedEvidenceLeaseManifest {
    pub fn manifest_digest(&self) -> [u8; 32] {
        self.manifest_digest
    }

    pub fn manifest_profile(&self) -> &str {
        &self.manifest_profile
    }

    pub fn contributor_count(&self) -> u32 {
        self.contributor_count
    }

    pub fn aggregate_lease(&self) -> &EvidenceLease {
        &self.aggregate_lease
    }
}

/// Qualify a canonical provenance manifest over a non-empty exact contributor set.
/// Input order has no meaning. Duplicate exact contribution identities are rejected.
pub fn qualify_evidence_lease_manifest(
    contributions: &[EvidenceLeaseContribution],
    now_ms: u64,
) -> Result<QualifiedEvidenceLeaseManifest, EvidenceLeaseError> {
    if contributions.is_empty() {
        return Err(EvidenceLeaseError::EmptyLeaseSet);
    }
    if contributions.len() > MAX_CONTRIBUTIONS {
        return Err(EvidenceLeaseError::TooManyContributions);
    }

    let mut identities = Vec::with_capacity(contributions.len());
    let mut seen = BTreeSet::new();
    for contribution in contributions {
        contribution.validate_at(now_ms)?;
        let identity = contribution.identity_digest()?;
        if !seen.insert(identity) {
            return Err(EvidenceLeaseError::DuplicateContribution);
        }
        identities.push(identity);
    }
    identities.sort_unstable();

    let aggregate_lease = EvidenceLease::intersect_all(
        contributions.iter().map(|contribution| &contribution.lease),
        now_ms,
    )?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_MANIFEST);
    frame(&mut hasher, MANIFEST_PROFILE.as_bytes());
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, &(identities.len() as u64).to_le_bytes());
    for identity in identities {
        frame(&mut hasher, &identity);
    }
    frame(&mut hasher, &aggregate_lease.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &aggregate_lease.valid_until_ms.to_le_bytes());

    Ok(QualifiedEvidenceLeaseManifest {
        manifest_digest: *hasher.finalize().as_bytes(),
        manifest_profile: MANIFEST_PROFILE.into(),
        contributor_count: contributions.len() as u32,
        aggregate_lease,
    })
}

fn require_ref(value: &str) -> Result<(), EvidenceLeaseError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(EvidenceLeaseError::InvalidEvidenceReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), EvidenceLeaseError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(EvidenceLeaseError::InvalidEvidenceProfile)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvidenceLeaseError {
    WrongProtocolVersion,
    InvalidVerificationTime,
    VerificationFromFuture,
    InvertedLease,
    ExpiredLease,
    EmptyLeaseSet,
    TooManyContributions,
    ZeroEvidenceDigest,
    InvalidEvidenceProfile,
    InvalidEvidenceReference,
    DuplicateContribution,
}

impl fmt::Display for EvidenceLeaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong evidence-lease protocol version",
            Self::InvalidVerificationTime => "evidence lease verification time must be positive",
            Self::VerificationFromFuture => "evidence lease verification time is in the future",
            Self::InvertedLease => "evidence lease validity must follow verification time",
            Self::ExpiredLease => "evidence lease is expired",
            Self::EmptyLeaseSet => "cannot intersect or manifest an empty evidence-lease set",
            Self::TooManyContributions => "evidence-lease manifest exceeds contribution bound",
            Self::ZeroEvidenceDigest => "evidence-lease contribution digest must be non-zero",
            Self::InvalidEvidenceProfile => "invalid evidence-lease contribution profile",
            Self::InvalidEvidenceReference => "invalid evidence-lease contribution reference",
            Self::DuplicateContribution => "duplicate evidence-lease contribution is forbidden",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for EvidenceLeaseError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn contribution(
        role: EvidenceLeaseRole,
        byte: u8,
        reference: &str,
        verified_at_ms: u64,
        valid_until_ms: u64,
    ) -> EvidenceLeaseContribution {
        EvidenceLeaseContribution {
            role,
            evidence_digest: d(byte),
            evidence_profile: "evidence-v1-blake3".into(),
            evidence_ref: reference.into(),
            lease: EvidenceLease::new(verified_at_ms, valid_until_ms, 200).unwrap(),
        }
    }

    #[test]
    fn intersection_moves_only_in_the_conservative_direction() {
        let left = EvidenceLease::new(100, 1_000, 200).unwrap();
        let right = EvidenceLease::new(150, 800, 200).unwrap();
        let joined = left.intersect(&right, 200).unwrap();
        assert_eq!(joined.verified_at_ms, 150);
        assert_eq!(joined.valid_until_ms, 800);
    }

    #[test]
    fn cap_cannot_widen_a_lease() {
        let lease = EvidenceLease::new(100, 1_000, 200).unwrap();
        assert_eq!(
            lease.cap_valid_until(2_000, 200).unwrap().valid_until_ms,
            1_000
        );
        assert_eq!(
            lease.cap_valid_until(700, 200).unwrap().valid_until_ms,
            700
        );
    }

    #[test]
    fn expired_member_cannot_be_revived_by_intersection() {
        let live = EvidenceLease::new(100, 1_000, 200).unwrap();
        let expired = EvidenceLease {
            protocol_version: PROTOCOL_VERSION.into(),
            verified_at_ms: 100,
            valid_until_ms: 150,
        };
        assert_eq!(
            live.intersect(&expired, 200).unwrap_err(),
            EvidenceLeaseError::ExpiredLease
        );
    }

    #[test]
    fn manifest_is_order_independent_and_uses_minimum_horizon() {
        let a = contribution(EvidenceLeaseRole::SourceHead, 1, "source:1", 120, 900);
        let b = contribution(
            EvidenceLeaseRole::AuthorityStateTransition,
            2,
            "transition:1",
            150,
            700,
        );
        let left = qualify_evidence_lease_manifest(&[a.clone(), b.clone()], 200).unwrap();
        let right = qualify_evidence_lease_manifest(&[b, a], 200).unwrap();
        assert_eq!(left.manifest_digest(), right.manifest_digest());
        assert_eq!(left.aggregate_lease().verified_at_ms, 150);
        assert_eq!(left.aggregate_lease().valid_until_ms, 700);
        assert_eq!(left.contributor_count(), 2);
    }

    #[test]
    fn root_subroles_are_distinct_provenance() {
        let constitution = contribution(
            EvidenceLeaseRole::CurrentConstitution,
            1,
            "constitution:1",
            120,
            900,
        );
        let adoption = contribution(
            EvidenceLeaseRole::BootstrapRootAdoption,
            1,
            "constitution:1",
            120,
            900,
        );
        assert_ne!(
            constitution.identity_digest().unwrap(),
            adoption.identity_digest().unwrap()
        );
    }

    #[test]
    fn contributor_horizon_change_changes_manifest() {
        let a = contribution(EvidenceLeaseRole::SourceHead, 1, "source:1", 120, 900);
        let mut b = contribution(
            EvidenceLeaseRole::AuthorityStateTransition,
            2,
            "transition:1",
            150,
            700,
        );
        let first = qualify_evidence_lease_manifest(&[a.clone(), b.clone()], 200).unwrap();
        b.lease.valid_until_ms = 650;
        let second = qualify_evidence_lease_manifest(&[a, b], 200).unwrap();
        assert_ne!(first.manifest_digest(), second.manifest_digest());
        assert_eq!(second.aggregate_lease().valid_until_ms, 650);
    }

    #[test]
    fn contributor_reference_or_role_change_changes_manifest() {
        let a = contribution(EvidenceLeaseRole::SourceHead, 1, "source:1", 120, 900);
        let mut changed_ref = a.clone();
        changed_ref.evidence_ref = "source:2".into();
        let mut changed_role = a.clone();
        changed_role.role = EvidenceLeaseRole::OperationalFreshness;
        let baseline = qualify_evidence_lease_manifest(&[a], 200).unwrap();
        assert_ne!(
            baseline.manifest_digest(),
            qualify_evidence_lease_manifest(&[changed_ref], 200)
                .unwrap()
                .manifest_digest()
        );
        assert_ne!(
            baseline.manifest_digest(),
            qualify_evidence_lease_manifest(&[changed_role], 200)
                .unwrap()
                .manifest_digest()
        );
    }

    #[test]
    fn duplicate_contribution_denies() {
        let a = contribution(EvidenceLeaseRole::SourceHead, 1, "source:1", 120, 900);
        assert_eq!(
            qualify_evidence_lease_manifest(&[a.clone(), a], 200).unwrap_err(),
            EvidenceLeaseError::DuplicateContribution
        );
    }

    #[test]
    fn future_verification_is_rejected() {
        assert_eq!(
            EvidenceLease::new(300, 400, 200).unwrap_err(),
            EvidenceLeaseError::VerificationFromFuture
        );
    }

    #[test]
    fn empty_intersection_is_not_unbounded_authority() {
        let leases: Vec<EvidenceLease> = Vec::new();
        assert_eq!(
            EvidenceLease::intersect_all(leases.iter(), 200).unwrap_err(),
            EvidenceLeaseError::EmptyLeaseSet
        );
        assert_eq!(
            qualify_evidence_lease_manifest(&[], 200).unwrap_err(),
            EvidenceLeaseError::EmptyLeaseSet
        );
    }
}
