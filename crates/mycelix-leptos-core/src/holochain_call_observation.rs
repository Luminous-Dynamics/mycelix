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

use crate::{
    HolochainCallError, HolochainCallFailureKind, HolochainCallPhase,
};
use mycelix_leptos_client::ClientError;

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

/// One provider-admitted zome-call attempt with call identity bound exactly
/// once before any phase-specific work begins.
///
/// This value is intentionally not `Clone`: consuming it into a terminal
/// failure observation prevents one admitted attempt from manufacturing
/// multiple independent terminal failure snapshots through ordinary API use.
#[derive(Debug, PartialEq, Eq)]
pub struct HolochainCallAttempt {
    id: HolochainCallAttemptId,
    role: String,
    zome: String,
    function: String,
}

impl HolochainCallAttempt {
    fn new(
        id: HolochainCallAttemptId,
        role: impl Into<String>,
        zome: impl Into<String>,
        function: impl Into<String>,
    ) -> Self {
        Self {
            id,
            role: role.into(),
            zome: zome.into(),
            function: function.into(),
        }
    }

    pub const fn id(&self) -> HolochainCallAttemptId {
        self.id
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn zome(&self) -> &str {
        &self.zome
    }

    pub fn function(&self) -> &str {
        &self.function
    }

    /// Consume this admitted attempt into one terminal typed failure
    /// observation while retaining the exact bound call target.
    pub fn fail(
        self,
        phase: HolochainCallPhase,
        source: ClientError,
    ) -> HolochainCallFailureObservation {
        HolochainCallFailureObservation::new(
            self.id,
            HolochainCallError::new(
                phase,
                self.role,
                self.zome,
                self.function,
                source,
            ),
        )
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

    /// Admit one call attempt and bind its target identity exactly once.
    ///
    /// Sequence exhaustion returns `None`; callers must not reset the sequence
    /// or invent an uncorrelated attempt id.
    pub fn admit(
        &mut self,
        role: impl Into<String>,
        zome: impl Into<String>,
        function: impl Into<String>,
    ) -> Option<HolochainCallAttempt> {
        let id = self.allocate()?;
        Some(HolochainCallAttempt::new(id, role, zome, function))
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
    use mycelix_leptos_client::{ConductorError, ConductorErrorKind};

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
        assert_eq!(
            sequence.admit("personal", "identity_vault", "get_my_profile_view"),
            None
        );
    }

    #[test]
    fn admitted_attempt_binds_call_identity_once() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let attempt = sequence
            .admit(
                "personal",
                "identity_vault",
                "set_profile_view_if_current",
            )
            .expect("attempt");

        assert_eq!(attempt.id(), HolochainCallAttemptId::FIRST);
        assert_eq!(attempt.role(), "personal");
        assert_eq!(attempt.zome(), "identity_vault");
        assert_eq!(attempt.function(), "set_profile_view_if_current");
    }

    #[test]
    fn terminal_failure_consumes_bound_identity_and_phase() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let attempt = sequence
            .admit(
                "personal",
                "data_preferences",
                "set_preference_view_if_current",
            )
            .expect("attempt");

        let observation = attempt.fail(
            HolochainCallPhase::Encode,
            ClientError::SerializationError("invalid input".into()),
        );

        assert_eq!(observation.attempt_id(), HolochainCallAttemptId::FIRST);
        assert_eq!(observation.error().phase(), HolochainCallPhase::Encode);
        assert_eq!(observation.error().role(), "personal");
        assert_eq!(observation.error().zome(), "data_preferences");
        assert_eq!(
            observation.error().function(),
            "set_preference_view_if_current"
        );
        assert_eq!(observation.failure_kind(), HolochainCallFailureKind::Serialization);
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
