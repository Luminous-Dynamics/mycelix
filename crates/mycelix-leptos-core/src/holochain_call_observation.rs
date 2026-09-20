// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-local correlation for typed Holochain call failure observations.
//!
//! A reactive diagnostic surface must not let an older call that completes late
//! silently overwrite evidence from a newer call. This module provides a
//! provider-local monotonic attempt identity that can be attached to typed call
//! failures without strengthening that identity into transport, mutation, or
//! commit authority.
//!
//! In particular:
//!
//! `HolochainCallAttemptId != BrowserWsTransport request id`
//!
//! `HolochainCallAttemptId != mutation receipt id`
//!
//! `HolochainCallAttemptId != Holochain action hash`
//!
//! `HolochainCallAttemptId != commit evidence`

use crate::{HolochainCallError, HolochainCallFailureKind};

/// Provider-instance-local identity assigned when a shared-provider zome-call
/// attempt is admitted.
///
/// Values are monotonic only within one allocator instance. They are not
/// globally unique and must never be persisted or compared across provider
/// lifetimes as protocol authority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct HolochainCallAttemptId(u64);

impl HolochainCallAttemptId {
    pub const FIRST: Self = Self(1);

    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Non-cloneable allocator for provider-local call attempt identities.
///
/// The allocator exhausts instead of wrapping, so an ancient observation can
/// never become numerically indistinguishable from a future attempt after
/// `u64::MAX`.
#[derive(Debug, PartialEq, Eq)]
pub struct HolochainCallAttemptSequence {
    next: Option<u64>,
}

impl Default for HolochainCallAttemptSequence {
    fn default() -> Self {
        Self { next: Some(1) }
    }
}

impl HolochainCallAttemptSequence {
    /// Allocate the next provider-local attempt id.
    ///
    /// Returns `None` after the sequence is exhausted. Callers must fail closed
    /// rather than restarting or wrapping the sequence.
    pub fn allocate(&mut self) -> Option<HolochainCallAttemptId> {
        let current = self.next?;
        self.next = current.checked_add(1);
        Some(HolochainCallAttemptId(current))
    }

    pub const fn is_exhausted(&self) -> bool {
        self.next.is_none()
    }
}

/// Typed failure evidence correlated to the provider call attempt that
/// produced it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HolochainCallFailureObservation {
    attempt_id: HolochainCallAttemptId,
    error: HolochainCallError,
}

impl HolochainCallFailureObservation {
    pub const fn new(attempt_id: HolochainCallAttemptId, error: HolochainCallError) -> Self {
        Self { attempt_id, error }
    }

    pub const fn attempt_id(&self) -> HolochainCallAttemptId {
        self.attempt_id
    }

    pub const fn error(&self) -> &HolochainCallError {
        &self.error
    }

    pub fn failure_kind(&self) -> HolochainCallFailureKind {
        self.error.failure_kind()
    }

    /// Whether this observation belongs to a newer provider-admitted attempt.
    ///
    /// This is only an ordering relation for diagnostic correlation. It does
    /// not imply that either call committed, rolled back, or is safe to retry.
    pub const fn is_newer_than(&self, other: &Self) -> bool {
        self.attempt_id.0 > other.attempt_id.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HolochainCallPhase, HolochainCallFailureKind};
    use mycelix_leptos_client::{ClientError, ConductorError, ConductorErrorKind};

    fn failure(id: HolochainCallAttemptId, message: &str) -> HolochainCallFailureObservation {
        HolochainCallFailureObservation::new(
            id,
            HolochainCallError::new(
                HolochainCallPhase::Transport,
                "personal",
                "identity_vault",
                "set_profile_view_if_current",
                ClientError::Conductor(ConductorError {
                    kind: ConductorErrorKind::Ribosome,
                    message: message.into(),
                }),
            ),
        )
    }

    #[test]
    fn allocation_is_monotonic_within_one_provider_sequence() {
        let mut sequence = HolochainCallAttemptSequence::default();

        let first = sequence.allocate().expect("first id");
        let second = sequence.allocate().expect("second id");
        let third = sequence.allocate().expect("third id");

        assert_eq!(first, HolochainCallAttemptId::FIRST);
        assert_eq!(first.get(), 1);
        assert_eq!(second.get(), 2);
        assert_eq!(third.get(), 3);
        assert!(first < second && second < third);
        assert!(!sequence.is_exhausted());
    }

    #[test]
    fn sequence_exhaustion_fails_closed_instead_of_wrapping() {
        let mut sequence = HolochainCallAttemptSequence {
            next: Some(u64::MAX),
        };

        let last = sequence.allocate().expect("final unique id");
        assert_eq!(last.get(), u64::MAX);
        assert!(sequence.is_exhausted());
        assert_eq!(sequence.allocate(), None);
    }

    #[test]
    fn late_older_failure_does_not_supersede_newer_attempt_evidence() {
        let newer = failure(HolochainCallAttemptId(8), "newer failure");
        let older_finishing_late = failure(HolochainCallAttemptId(7), "older late failure");

        assert!(newer.is_newer_than(&older_finishing_late));
        assert!(!older_finishing_late.is_newer_than(&newer));
    }

    #[test]
    fn correlation_preserves_typed_failure_without_strengthening_it() {
        let observation = failure(
            HolochainCallAttemptId(11),
            "source chain head moved",
        );

        assert_eq!(observation.attempt_id().get(), 11);
        assert_eq!(
            observation.failure_kind(),
            HolochainCallFailureKind::Conductor(ConductorErrorKind::Ribosome)
        );
        assert_eq!(
            observation
                .error()
                .conductor_error()
                .map(|error| error.message.as_str()),
            Some("source chain head moved")
        );
    }

    #[test]
    fn observation_is_cloneable_value_evidence_but_allocator_is_not_duplicated() {
        let original = failure(HolochainCallAttemptId(3), "validation failed");
        let cloned = original.clone();

        assert_eq!(cloned, original);
        assert_eq!(cloned.attempt_id(), HolochainCallAttemptId(3));
    }
}
