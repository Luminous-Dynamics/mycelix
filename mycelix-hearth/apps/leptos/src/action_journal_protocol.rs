// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Testable ordering contract for a future durable Hearth action journal.
//!
//! This module is deliberately storage- and transport-agnostic. It does not
//! implement IndexedDB or call Holochain itself. The caller supplies a journal
//! writer whose successful return means its transaction committed according to
//! the adapter's documented durability level.
//!
//! Critical ordering:
//!   persist(Prepared) -> persist(Submitted) -> dispatch once
//!
//! If the Submitted commit is not confirmed, dispatch is never called. After a
//! dispatch may have happened, any ambiguous result remains OutcomeUnknown.
//! This protects against blind client replay; it does not guarantee that the
//! remote side effect happened exactly once.

use crate::action_lifecycle::{ActionAttempt, ActionKind, AttemptStatus};

/// A prepared operation can only be obtained after the supplied journal writer
/// accepted its Prepared state. Callers cannot instantiate the token directly;
/// the persistence guarantee still depends on the writer's actual contract.
#[derive(Debug, PartialEq, Eq)]
pub struct PreparedAction {
    attempt: ActionAttempt,
}

impl PreparedAction {
    pub fn operation_id(&self) -> &str {
        self.attempt.operation_id()
    }

    pub const fn action(&self) -> ActionKind {
        self.attempt.action()
    }

    pub fn target_fingerprint(&self) -> &str {
        self.attempt.target_fingerprint()
    }

    pub const fn status(&self) -> AttemptStatus {
        self.attempt.status()
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum PrepareError<E> {
    InvalidState,
    Journal(E),
}

/// Result of the dispatch gate. A failed confirmation journal write does not
/// erase a received remote success: it means the current process has that
/// response, while a later recovery may need to reconcile an older state.
#[derive(Debug, PartialEq, Eq)]
pub enum DispatchGateResult<R, E> {
    /// No transport call was made because the Submitted write was not confirmed.
    NotSent { journal_error: E },
    /// A successful domain response arrived. If confirmation_persisted is
    /// false, the next process must conservatively reconcile from Submitted.
    SourceChainCommitted {
        response: R,
        confirmation_persisted: bool,
    },
    /// The remote call could have had an effect, but no conclusive response was
    /// received. The UI must not automatically resend this operation.
    OutcomeUnknown { outcome_persisted: bool },
    /// The prepared permit was already consumed or otherwise out of state.
    InvalidState,
}

/// Persist a newly created Prepared attempt before it can be dispatched.
///
/// The caller must retain the returned permit and must not dispatch the raw
/// ActionAttempt by another path. Persist should resolve only after its
/// transaction-complete/commit signal, not merely after scheduling a write.
pub fn persist_prepared<E>(
    attempt: ActionAttempt,
    mut persist: impl FnMut(&ActionAttempt) -> Result<(), E>,
) -> Result<PreparedAction, PrepareError<E>> {
    if attempt.status() != AttemptStatus::Prepared {
        return Err(PrepareError::InvalidState);
    }

    persist(&attempt).map_err(PrepareError::Journal)?;
    Ok(PreparedAction { attempt })
}

/// Persist Submitted and only then invoke the one-shot transport closure.
///
/// A generic transport error is treated as ambiguous. This function never
/// claims rejection from Err(()); a future typed backend result may add a
/// separately evidenced ExplicitlyRejected branch.
pub fn dispatch_after_journal<E, R>(
    prepared: &mut PreparedAction,
    mut persist: impl FnMut(&ActionAttempt) -> Result<(), E>,
    dispatch: impl FnOnce(&ActionAttempt) -> Result<R, ()>,
) -> DispatchGateResult<R, E> {
    if prepared.attempt.status() != AttemptStatus::Prepared {
        return DispatchGateResult::InvalidState;
    }

    if prepared.attempt.transition(AttemptStatus::Submitted).is_err() {
        return DispatchGateResult::InvalidState;
    }

    // Fail closed: never call dispatch unless the journal confirms Submitted.
    if let Err(journal_error) = persist(&prepared.attempt) {
        return DispatchGateResult::NotSent { journal_error };
    }

    match dispatch(&prepared.attempt) {
        Ok(response) => {
            if prepared
                .attempt
                .transition(AttemptStatus::SourceChainCommitted)
                .is_err()
            {
                return DispatchGateResult::InvalidState;
            }
            let confirmation_persisted = persist(&prepared.attempt).is_ok();
            DispatchGateResult::SourceChainCommitted {
                response,
                confirmation_persisted,
            }
        }
        Err(()) => {
            // The request may have committed remotely before the response was
            // lost. Never classify this as definite rejection or auto-retry.
            if prepared
                .attempt
                .transition(AttemptStatus::OutcomeUnknown)
                .is_err()
            {
                return DispatchGateResult::InvalidState;
            }
            let outcome_persisted = persist(&prepared.attempt).is_ok();
            DispatchGateResult::OutcomeUnknown { outcome_persisted }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        dispatch_after_journal, persist_prepared, DispatchGateResult, PrepareError,
    };
    use crate::action_lifecycle::{ActionAttempt, ActionKind, AttemptStatus};

    #[derive(Default)]
    struct FakeJournal {
        committed: Vec<AttemptStatus>,
        calls: usize,
        fail_on_call: Option<usize>,
    }

    impl FakeJournal {
        fn persist(&mut self, attempt: &ActionAttempt) -> Result<(), &'static str> {
            self.calls += 1;
            if self.fail_on_call == Some(self.calls) {
                return Err("injected commit failure");
            }
            self.committed.push(attempt.status());
            Ok(())
        }
    }

    fn attempt() -> ActionAttempt {
        ActionAttempt::new("op-001", ActionKind::CareCompletion, "target-sha256-test")
            .expect("test operation is well formed")
    }

    #[test]
    fn failed_prepared_commit_never_yields_a_dispatch_permit() {
        let mut journal = FakeJournal {
            fail_on_call: Some(1),
            ..FakeJournal::default()
        };
        let result = persist_prepared(attempt(), |a| journal.persist(a));
        assert_eq!(result, Err(PrepareError::Journal("injected commit failure")));
        assert!(journal.committed.is_empty());
        assert_eq!(journal.calls, 1);
    }

    #[test]
    fn submitted_commit_failure_never_calls_transport() {
        let mut journal = FakeJournal::default();
        let mut permit = persist_prepared(attempt(), |a| journal.persist(a)).unwrap();
        journal.fail_on_call = Some(2);
        let mut dispatch_calls = 0;

        let result = dispatch_after_journal(
            &mut permit,
            |a| journal.persist(a),
            |_| {
                dispatch_calls += 1;
                Ok::<_, ()>(())
            },
        );

        assert_eq!(
            result,
            DispatchGateResult::NotSent {
                journal_error: "injected commit failure"
            }
        );
        assert_eq!(dispatch_calls, 0);
        assert_eq!(journal.committed, vec![AttemptStatus::Prepared]);
        assert!(!permit.status().dispatch_eligible());
    }

    #[test]
    fn committed_submitted_state_precedes_transport_and_confirmation() {
        let mut journal = FakeJournal::default();
        let mut permit = persist_prepared(attempt(), |a| journal.persist(a)).unwrap();
        let submitted_committed = std::cell::Cell::new(false);

        let result = dispatch_after_journal(
            &mut permit,
            |a| {
                let status = a.status();
                let persisted = journal.persist(a);
                if persisted.is_ok() && status == AttemptStatus::Submitted {
                    submitted_committed.set(true);
                }
                persisted
            },
            |_| {
                assert!(submitted_committed.get(), "transport ran before Submitted commit");
                Ok::<_, ()>("remote-record")
            },
        );

        assert_eq!(
            result,
            DispatchGateResult::SourceChainCommitted {
                response: "remote-record",
                confirmation_persisted: true,
            }
        );
        assert_eq!(
            journal.committed,
            vec![
                AttemptStatus::Prepared,
                AttemptStatus::Submitted,
                AttemptStatus::SourceChainCommitted
            ]
        );
    }

    #[test]
    fn ambiguous_transport_result_is_journaled_unknown_and_not_retried() {
        let mut journal = FakeJournal::default();
        let mut permit = persist_prepared(attempt(), |a| journal.persist(a)).unwrap();
        let mut dispatch_calls = 0;

        let first = dispatch_after_journal(
            &mut permit,
            |a| journal.persist(a),
            |_| {
                dispatch_calls += 1;
                Err::<(), ()>(())
            },
        );
        assert_eq!(
            first,
            DispatchGateResult::OutcomeUnknown {
                outcome_persisted: true
            }
        );
        let second = dispatch_after_journal(&mut permit, |a| journal.persist(a), |_| {
            dispatch_calls += 1;
            Ok::<_, ()>(())
        });
        assert_eq!(second, DispatchGateResult::InvalidState);
        assert_eq!(dispatch_calls, 1);
        assert_eq!(
            journal.committed,
            vec![
                AttemptStatus::Prepared,
                AttemptStatus::Submitted,
                AttemptStatus::OutcomeUnknown
            ]
        );
    }

    #[test]
    fn lost_confirmation_commit_keeps_remote_receipt_but_requires_restart_reconciliation() {
        let mut journal = FakeJournal {
            fail_on_call: Some(3),
            ..FakeJournal::default()
        };
        let mut permit = persist_prepared(attempt(), |a| journal.persist(a)).unwrap();

        let result = dispatch_after_journal(
            &mut permit,
            |a| journal.persist(a),
            |_| Ok::<_, ()>("remote-committed-record"),
        );

        assert_eq!(
            result,
            DispatchGateResult::SourceChainCommitted {
                response: "remote-committed-record",
                confirmation_persisted: false
            }
        );
        assert_eq!(
            journal.committed,
            vec![AttemptStatus::Prepared, AttemptStatus::Submitted]
        );
        assert_eq!(permit.status(), AttemptStatus::SourceChainCommitted);
    }

    #[test]
    fn failure_to_persist_unknown_does_not_enable_another_dispatch() {
        let mut journal = FakeJournal {
            fail_on_call: Some(3),
            ..FakeJournal::default()
        };
        let mut permit = persist_prepared(attempt(), |a| journal.persist(a)).unwrap();

        let first = dispatch_after_journal(
            &mut permit,
            |a| journal.persist(a),
            |_| Err::<(), ()>(()),
        );
        assert_eq!(
            first,
            DispatchGateResult::OutcomeUnknown {
                outcome_persisted: false
            }
        );
        let second = dispatch_after_journal(&mut permit, |a| journal.persist(a), |_| {
            panic!("unknown operations must never be dispatched again")
        });
        assert_eq!(second, DispatchGateResult::InvalidState);
    }

    #[test]
    fn every_action_kind_obeys_commit_before_send_ordering() {
        for kind in [
            ActionKind::BondUpdate,
            ActionKind::Gratitude,
            ActionKind::Vote,
            ActionKind::Presence,
            ActionKind::CareCompletion,
            ActionKind::Invitation,
        ] {
            let op = ActionAttempt::new("op-matrix", kind, "target-matrix").unwrap();
            let mut journal = FakeJournal::default();
            let mut permit = persist_prepared(op, |a| journal.persist(a)).unwrap();
            let mut calls = 0;
            let result = dispatch_after_journal(
                &mut permit,
                |a| journal.persist(a),
                |_| {
                    calls += 1;
                    Ok::<_, ()>(())
                },
            );
            assert!(matches!(result, DispatchGateResult::SourceChainCommitted { confirmation_persisted: true, .. }));
            assert_eq!(calls, 1);
            assert_eq!(journal.committed[0], AttemptStatus::Prepared);
            assert_eq!(journal.committed[1], AttemptStatus::Submitted);
            assert_eq!(journal.committed[2], AttemptStatus::SourceChainCommitted);
        }
    }

    #[test]
    fn prepared_permit_cannot_be_created_from_another_state() {
        let mut op = attempt();
        op.transition(AttemptStatus::Submitted).unwrap();
        let result = persist_prepared(op, |_| Ok::<_, ()>(()));
        assert_eq!(result, Err(PrepareError::InvalidState));
    }
}
