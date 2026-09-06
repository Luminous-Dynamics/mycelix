// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Monotone dynamic-evidence lease composition.
//!
//! A verifier lease is dynamic evidence, not semantic authority identity.  This
//! crate gives runtime composers one small typed primitive for carrying verifier
//! horizons across compatibility ABIs that do not encode those horizons directly.
//! Intersections may only move verification time forward and validity earlier.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-evidence-lease-v0.1";

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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvidenceLeaseError {
    WrongProtocolVersion,
    InvalidVerificationTime,
    VerificationFromFuture,
    InvertedLease,
    ExpiredLease,
    EmptyLeaseSet,
}

impl fmt::Display for EvidenceLeaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong evidence-lease protocol version",
            Self::InvalidVerificationTime => "evidence lease verification time must be positive",
            Self::VerificationFromFuture => "evidence lease verification time is in the future",
            Self::InvertedLease => "evidence lease validity must follow verification time",
            Self::ExpiredLease => "evidence lease is expired",
            Self::EmptyLeaseSet => "cannot intersect an empty evidence-lease set",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for EvidenceLeaseError {}

#[cfg(test)]
mod tests {
    use super::*;

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
    }
}
