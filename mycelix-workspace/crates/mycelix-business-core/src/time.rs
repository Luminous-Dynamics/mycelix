// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Explicit time and validity semantics.
//!
//! This crate never reads the ambient clock. Consequential qualification must
//! receive time explicitly as data.

use core::fmt;

/// Explicit millisecond timestamp.
///
/// The interpretation is intentionally transport-neutral; callers decide which
/// clock/time standard a semantic profile requires. The key invariant is that
/// qualification time is supplied explicitly rather than read implicitly.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TimestampMs(i64);

impl TimestampMs {
    /// Construct an explicit timestamp.
    #[must_use]
    pub const fn new(value: i64) -> Self {
        Self(value)
    }

    /// Raw millisecond value.
    #[must_use]
    pub const fn value(self) -> i64 {
        self.0
    }
}

/// End of a validity interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ValidityEnd {
    /// Valid through this explicit timestamp.
    At(TimestampMs),
    /// Semantically unbounded under the owning profile.
    Unbounded,
    /// Required currentness/expiry is not known.
    Unknown,
}

/// Validity interval used as an explicit qualification prerequisite.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ValidityWindow {
    valid_from: TimestampMs,
    valid_until: ValidityEnd,
}

/// Structural validity errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidityError {
    EndBeforeStart,
    UnknownRequiredEnd,
    CandidateWidensPrerequisite,
}

impl fmt::Display for ValidityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EndBeforeStart => f.write_str("validity end precedes validity start"),
            Self::UnknownRequiredEnd => {
                f.write_str("required validity boundary is unknown; positive qualification fails closed")
            }
            Self::CandidateWidensPrerequisite => {
                f.write_str("candidate validity outlives a required prerequisite")
            }
        }
    }
}

impl std::error::Error for ValidityError {}

impl ValidityWindow {
    /// Construct a validity window.
    pub fn new(valid_from: TimestampMs, valid_until: ValidityEnd) -> Result<Self, ValidityError> {
        if let ValidityEnd::At(end) = valid_until {
            if end < valid_from {
                return Err(ValidityError::EndBeforeStart);
            }
        }

        Ok(Self {
            valid_from,
            valid_until,
        })
    }

    /// Start of validity.
    #[must_use]
    pub const fn valid_from(self) -> TimestampMs {
        self.valid_from
    }

    /// End of validity.
    #[must_use]
    pub const fn valid_until(self) -> ValidityEnd {
        self.valid_until
    }

    /// Whether the window positively establishes validity at `at`.
    ///
    /// `Unknown` fails closed.
    #[must_use]
    pub fn contains(self, at: TimestampMs) -> bool {
        if at < self.valid_from {
            return false;
        }

        match self.valid_until {
            ValidityEnd::At(end) => at <= end,
            ValidityEnd::Unbounded => true,
            ValidityEnd::Unknown => false,
        }
    }
}

/// Compute the earliest required validity horizon.
///
/// Unknown required expiry/currentness fails closed instead of inventing a
/// permissive duration.
pub fn required_horizon(
    bounds: impl IntoIterator<Item = ValidityEnd>,
) -> Result<ValidityEnd, ValidityError> {
    let mut earliest: Option<TimestampMs> = None;

    for bound in bounds {
        match bound {
            ValidityEnd::Unknown => return Err(ValidityError::UnknownRequiredEnd),
            ValidityEnd::Unbounded => {}
            ValidityEnd::At(end) => {
                earliest = Some(match earliest {
                    Some(current) => current.min(end),
                    None => end,
                });
            }
        }
    }

    Ok(match earliest {
        Some(end) => ValidityEnd::At(end),
        None => ValidityEnd::Unbounded,
    })
}

/// Verify that a downstream validity horizon does not widen an upstream one.
pub fn ensure_not_wider(
    candidate: ValidityEnd,
    prerequisite: ValidityEnd,
) -> Result<(), ValidityError> {
    use ValidityEnd::{At, Unbounded, Unknown};

    match (candidate, prerequisite) {
        (Unknown, _) | (_, Unknown) => Err(ValidityError::UnknownRequiredEnd),
        (Unbounded, At(_)) => Err(ValidityError::CandidateWidensPrerequisite),
        (At(candidate), At(required)) if candidate > required => {
            Err(ValidityError::CandidateWidensPrerequisite)
        }
        _ => Ok(()),
    }
}
