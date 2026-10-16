// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure Hearth action-lifecycle contract.
//!
//! This module deliberately does not dispatch network calls or claim persistence.
//! A caller must durably write the Prepared record, durably transition it to
//! Submitted, and only then send the request. On restart, any persisted
//! Submitted attempt is ambiguous and must be recovered as OutcomeUnknown.

/// Action families currently exposed by the Hearth UI.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ActionKind {
    BondUpdate,
    Gratitude,
    Vote,
    Presence,
    CareCompletion,
    Invitation,
}

/// Knowledge about one operation, not a description of the UI animation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttemptStatus {
    /// Local operation metadata exists; no request has been authorized for send.
    Prepared,
    /// Dispatch was durably recorded before the remote call began.
    Submitted,
    /// A successful, decodable domain response was received.
    Confirmed,
    /// A typed domain-level rejection proved the mutation was not applied.
    ExplicitlyRejected,
    /// The remote effect may or may not have happened.
    OutcomeUnknown,
    /// An authoritative read/evidence source proved the requested effect exists.
    ReconciledApplied,
    /// An authoritative read/evidence source proved the requested effect does not exist.
    ReconciledNotApplied,
    /// A local preflight gate blocked dispatch before any remote request was sent.
    NotSent,
}

impl AttemptStatus {
    pub const ALL: [Self; 8] = [
        Self::Prepared,
        Self::Submitted,
        Self::Confirmed,
        Self::ExplicitlyRejected,
        Self::OutcomeUnknown,
        Self::ReconciledApplied,
        Self::ReconciledNotApplied,
        Self::NotSent,
    ];

    /// A request may be initiated only from a newly prepared attempt.
    ///
    /// In particular, Submitted is not dispatch-eligible: its outcome may be
    /// ambiguous after timeout, restart, or response-decode failure.
    pub const fn dispatch_eligible(self) -> bool {
        matches!(self, Self::Prepared)
    }

    /// Whether this state is closed to further state-changing lifecycle events.
    pub const fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::Confirmed
                | Self::ExplicitlyRejected
                | Self::ReconciledApplied
                | Self::ReconciledNotApplied
                | Self::NotSent
        )
    }

    pub const fn needs_reconciliation(self) -> bool {
        matches!(self, Self::OutcomeUnknown)
    }
}

/// Returns whether a state transition is legal.
///
/// Repeating the current state is accepted as an idempotent no-op. There is no
/// direct transition from OutcomeUnknown back to Submitted or to a claimed
/// result; only reconciliation may resolve that state.
pub const fn transition_allowed(from: AttemptStatus, to: AttemptStatus) -> bool {
    if matches!(
        (from, to),
        (AttemptStatus::Prepared, AttemptStatus::Prepared)
            | (AttemptStatus::Submitted, AttemptStatus::Submitted)
            | (AttemptStatus::Confirmed, AttemptStatus::Confirmed)
            | (AttemptStatus::ExplicitlyRejected, AttemptStatus::ExplicitlyRejected)
            | (AttemptStatus::OutcomeUnknown, AttemptStatus::OutcomeUnknown)
            | (AttemptStatus::ReconciledApplied, AttemptStatus::ReconciledApplied)
            | (AttemptStatus::ReconciledNotApplied, AttemptStatus::ReconciledNotApplied)
            | (AttemptStatus::NotSent, AttemptStatus::NotSent)
    ) {
        return true;
    }

    matches!(
        (from, to),
        (AttemptStatus::Prepared, AttemptStatus::Submitted)
            | (AttemptStatus::Prepared, AttemptStatus::NotSent)
            | (AttemptStatus::Submitted, AttemptStatus::Confirmed)
            | (AttemptStatus::Submitted, AttemptStatus::ExplicitlyRejected)
            | (AttemptStatus::Submitted, AttemptStatus::OutcomeUnknown)
            | (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledApplied)
            | (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledNotApplied)
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleError {
    MissingOperationId,
    MissingTargetFingerprint,
    InvalidTransition {
        from: AttemptStatus,
        to: AttemptStatus,
    },
}

/// Minimal operation metadata for a future durable client journal.
///
/// Do not put household names, action message bodies, or raw member identities
/// in this record. target_fingerprint is an opaque, stable caller-provided
/// fingerprint. Its cryptographic construction and persistence are the caller's
/// responsibility and are not implemented here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionAttempt {
    operation_id: String,
    action: ActionKind,
    target_fingerprint: String,
    status: AttemptStatus,
}

impl ActionAttempt {
    pub fn new(
        operation_id: impl Into<String>,
        action: ActionKind,
        target_fingerprint: impl Into<String>,
    ) -> Result<Self, LifecycleError> {
        let operation_id = operation_id.into();
        if operation_id.trim().is_empty() {
            return Err(LifecycleError::MissingOperationId);
        }

        let target_fingerprint = target_fingerprint.into();
        if target_fingerprint.trim().is_empty() {
            return Err(LifecycleError::MissingTargetFingerprint);
        }

        Ok(Self {
            operation_id,
            action,
            target_fingerprint,
            status: AttemptStatus::Prepared,
        })
    }

    pub fn operation_id(&self) -> &str {
        &self.operation_id
    }

    pub const fn action(&self) -> ActionKind {
        self.action
    }

    pub fn target_fingerprint(&self) -> &str {
        &self.target_fingerprint
    }

    pub const fn status(&self) -> AttemptStatus {
        self.status
    }

    pub const fn dispatch_eligible(&self) -> bool {
        self.status.dispatch_eligible()
    }

    pub fn transition(&mut self, next: AttemptStatus) -> Result<(), LifecycleError> {
        if !transition_allowed(self.status, next) {
            return Err(LifecycleError::InvalidTransition {
                from: self.status,
                to: next,
            });
        }
        self.status = next;
        Ok(())
    }

    /// Conservatively recover a process-local view after restart.
    ///
    /// A persisted Submitted state cannot tell whether bytes reached the
    /// conductor or whether a remote effect committed. The durable adapter must
    /// persist OutcomeUnknown before exposing the recovered attempt to the UI.
    /// Returns true iff the state was changed.
    pub fn recover_after_restart(&mut self) -> bool {
        if self.status == AttemptStatus::Submitted {
            self.status = AttemptStatus::OutcomeUnknown;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        transition_allowed, ActionAttempt, ActionKind, AttemptStatus, LifecycleError,
    };

    const EXPECTED_EDGES: [(AttemptStatus, AttemptStatus); 15] = [
        (AttemptStatus::Prepared, AttemptStatus::Prepared),
        (AttemptStatus::Prepared, AttemptStatus::Submitted),
        (AttemptStatus::Prepared, AttemptStatus::NotSent),
        (AttemptStatus::Submitted, AttemptStatus::Submitted),
        (AttemptStatus::Submitted, AttemptStatus::Confirmed),
        (AttemptStatus::Submitted, AttemptStatus::ExplicitlyRejected),
        (AttemptStatus::Submitted, AttemptStatus::OutcomeUnknown),
        (AttemptStatus::Confirmed, AttemptStatus::Confirmed),
        (AttemptStatus::ExplicitlyRejected, AttemptStatus::ExplicitlyRejected),
        (AttemptStatus::OutcomeUnknown, AttemptStatus::OutcomeUnknown),
        (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledApplied),
        (AttemptStatus::OutcomeUnknown, AttemptStatus::ReconciledNotApplied),
        (AttemptStatus::ReconciledApplied, AttemptStatus::ReconciledApplied),
        (AttemptStatus::ReconciledNotApplied, AttemptStatus::ReconciledNotApplied),
        (AttemptStatus::NotSent, AttemptStatus::NotSent),
    ];

    fn attempt(action: ActionKind) -> ActionAttempt {
        ActionAttempt::new("op-test-1", action, "target-fingerprint-test")
            .expect("test metadata is valid")
    }

    #[test]
    fn exhaustive_transition_matrix_accepts_only_the_declared_edges() {
        for from in AttemptStatus::ALL {
            for to in AttemptStatus::ALL {
                let expected = EXPECTED_EDGES.contains(&(from, to));
                assert_eq!(
                    transition_allowed(from, to),
                    expected,
                    "unexpected transition policy: {from:?} -> {to:?}"
                );
            }
        }
        assert_eq!(AttemptStatus::ALL.len() * AttemptStatus::ALL.len(), 64);
    }

    #[test]
    fn happy_path_must_be_prepared_then_submitted_then_confirmed() {
        let mut op = attempt(ActionKind::CareCompletion);
        assert!(op.dispatch_eligible());
        op.transition(AttemptStatus::Submitted).unwrap();
        assert!(!op.dispatch_eligible());
        op.transition(AttemptStatus::Confirmed).unwrap();
        assert!(op.status().is_terminal());
        assert!(matches!(
            op.transition(AttemptStatus::Submitted),
            Err(LifecycleError::InvalidTransition {
                from: AttemptStatus::Confirmed,
                to: AttemptStatus::Submitted,
            })
        ));
    }

    #[test]
    fn unknown_outcome_cannot_be_retried_or_promoted_without_reconciliation() {
        let mut op = attempt(ActionKind::Vote);
        op.transition(AttemptStatus::Submitted).unwrap();
        op.transition(AttemptStatus::OutcomeUnknown).unwrap();
        assert!(!op.dispatch_eligible());
        assert!(op.status().needs_reconciliation());

        for forbidden in [
            AttemptStatus::Submitted,
            AttemptStatus::Confirmed,
            AttemptStatus::ExplicitlyRejected,
            AttemptStatus::Prepared,
        ] {
            assert!(matches!(
                op.transition(forbidden),
                Err(LifecycleError::InvalidTransition { .. })
            ));
        }

        op.transition(AttemptStatus::ReconciledApplied).unwrap();
        assert!(op.status().is_terminal());
    }

    #[test]
    fn restart_turns_submitted_into_unknown_exactly_once() {
        let mut op = attempt(ActionKind::Presence);
        op.transition(AttemptStatus::Submitted).unwrap();
        assert!(op.recover_after_restart());
        assert_eq!(op.status(), AttemptStatus::OutcomeUnknown);
        assert!(!op.recover_after_restart());
        assert!(!op.dispatch_eligible());
    }

    #[test]
    fn only_prepared_attempts_are_dispatch_eligible_for_every_action_kind() {
        let kinds = [
            ActionKind::BondUpdate,
            ActionKind::Gratitude,
            ActionKind::Vote,
            ActionKind::Presence,
            ActionKind::CareCompletion,
            ActionKind::Invitation,
        ];

        for kind in kinds {
            let mut op = attempt(kind);
            for state in AttemptStatus::ALL {
                assert_eq!(
                    state.dispatch_eligible(),
                    state == AttemptStatus::Prepared,
                    "bad dispatch policy for {kind:?} in {state:?}"
                );
            }
            assert_eq!(op.action(), kind);
            assert!(op.dispatch_eligible());
            op.transition(AttemptStatus::NotSent).unwrap();
            assert!(!op.dispatch_eligible());
        }
    }

    #[test]
    fn inconclusive_reconciliation_is_a_noop_not_a_new_submission() {
        let mut op = attempt(ActionKind::BondUpdate);
        op.transition(AttemptStatus::Submitted).unwrap();
        op.transition(AttemptStatus::OutcomeUnknown).unwrap();
        // Repeating the unknown observation is allowed as an idempotent no-op.
        op.transition(AttemptStatus::OutcomeUnknown).unwrap();
        assert_eq!(op.status(), AttemptStatus::OutcomeUnknown);
        assert!(!op.dispatch_eligible());
    }

    #[test]
    fn typed_rejection_is_terminal_and_only_follows_submission() {
        let mut before_send = attempt(ActionKind::Invitation);
        assert!(matches!(
            before_send.transition(AttemptStatus::ExplicitlyRejected),
            Err(LifecycleError::InvalidTransition { .. })
        ));

        let mut sent = attempt(ActionKind::Invitation);
        sent.transition(AttemptStatus::Submitted).unwrap();
        sent.transition(AttemptStatus::ExplicitlyRejected).unwrap();
        assert!(sent.status().is_terminal());
        assert!(!sent.dispatch_eligible());
    }

    #[test]
    fn not_sent_is_terminal_and_only_available_before_dispatch() {
        let mut blocked = attempt(ActionKind::Gratitude);
        blocked.transition(AttemptStatus::NotSent).unwrap();
        assert!(blocked.status().is_terminal());

        let mut submitted = attempt(ActionKind::Gratitude);
        submitted.transition(AttemptStatus::Submitted).unwrap();
        assert!(matches!(
            submitted.transition(AttemptStatus::NotSent),
            Err(LifecycleError::InvalidTransition { .. })
        ));
    }

    #[test]
    fn operation_and_target_metadata_are_nonempty_and_stable() {
        assert_eq!(
            ActionAttempt::new("", ActionKind::Vote, "target"),
            Err(LifecycleError::MissingOperationId)
        );
        assert_eq!(
            ActionAttempt::new("operation", ActionKind::Vote, " "),
            Err(LifecycleError::MissingTargetFingerprint)
        );

        let mut op = attempt(ActionKind::Gratitude);
        let operation_id = op.operation_id().to_owned();
        let target = op.target_fingerprint().to_owned();
        op.transition(AttemptStatus::Submitted).unwrap();
        op.transition(AttemptStatus::OutcomeUnknown).unwrap();
        assert_eq!(op.operation_id(), operation_id);
        assert_eq!(op.target_fingerprint(), target);
    }

    #[test]
    fn restart_recovery_leaves_proven_terminal_states_unchanged() {
        for terminal in [
            AttemptStatus::Confirmed,
            AttemptStatus::ExplicitlyRejected,
            AttemptStatus::ReconciledApplied,
            AttemptStatus::ReconciledNotApplied,
            AttemptStatus::NotSent,
        ] {
            let mut op = attempt(ActionKind::Presence);
            op.status = terminal;
            assert!(!op.recover_after_restart());
            assert_eq!(op.status(), terminal);
        }
    }
}
