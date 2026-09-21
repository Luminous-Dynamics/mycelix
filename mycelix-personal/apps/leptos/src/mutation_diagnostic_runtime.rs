// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! App-lifetime reactive runtime for Personal mutation diagnostics.
//!
//! This runtime owns only application-local correlation state. Reloading the
//! app resets Personal mutation-attempt ordering and diagnostic presentation;
//! it does not revoke, roll back, or erase any source-chain mutation receipt.

use leptos::prelude::*;

use crate::mutation_diagnostics::{
    PersonalMutationAttempt, PersonalMutationAttemptSequence, PersonalMutationDiagnosticLedger,
    PersonalMutationFailure, PersonalMutationSuccess,
};
use crate::mutation_state::PersonalMutationTarget;

/// Reactive app-local diagnostic runtime for semantic Personal mutation attempts.
#[derive(Clone, Copy)]
pub struct MutationDiagnosticRuntime {
    sequence: RwSignal<PersonalMutationAttemptSequence>,
    pub diagnostics: ReadSignal<PersonalMutationDiagnosticLedger>,
    set_diagnostics: WriteSignal<PersonalMutationDiagnosticLedger>,
}

impl MutationDiagnosticRuntime {
    /// Admit one exact semantic mutation attempt.
    ///
    /// `None` means the app-local monotonic sequence exhausted. Callers must
    /// fail closed rather than wrap/reset it inside the same app lifetime.
    pub fn admit(&self, target: PersonalMutationTarget) -> Option<PersonalMutationAttempt> {
        let binding = crate::mutation_diagnostics::PersonalMutationCallBinding::for_target(target);
        let mut admitted = None;
        self.sequence.update(|sequence| {
            admitted = sequence.admit(binding);
        });
        admitted
    }

    /// Publish one successfully decoded semantic attempt if it is newer for
    /// that exact Personal target.
    pub fn observe_success<T>(&self, success: &PersonalMutationSuccess<T>) -> bool {
        let mut snapshot = self.diagnostics.get_untracked();
        if !snapshot.observe_success(success) {
            return false;
        }
        self.set_diagnostics.set(snapshot);
        true
    }

    /// Publish one typed invocation failure if it is newer for that exact
    /// Personal target.
    pub fn observe_failure(&self, failure: PersonalMutationFailure) -> bool {
        let mut snapshot = self.diagnostics.get_untracked();
        if !snapshot.observe_failure(failure) {
            return false;
        }
        self.set_diagnostics.set(snapshot);
        true
    }

    /// Reactive clone of the latest accepted failure for one semantic target.
    pub fn latest_failure_for(
        &self,
        target: &PersonalMutationTarget,
    ) -> Option<PersonalMutationFailure> {
        self.diagnostics
            .get()
            .latest_failure_for(target)
            .cloned()
    }

    /// Non-reactive snapshot for event handlers and diagnostics.
    pub fn snapshot_untracked(&self) -> PersonalMutationDiagnosticLedger {
        self.diagnostics.get_untracked()
    }
}

pub fn provide_mutation_diagnostic_runtime() -> MutationDiagnosticRuntime {
    let (diagnostics, set_diagnostics) = signal(PersonalMutationDiagnosticLedger::default());
    let runtime = MutationDiagnosticRuntime {
        sequence: RwSignal::new(PersonalMutationAttemptSequence::default()),
        diagnostics,
        set_diagnostics,
    };
    provide_context(runtime);
    runtime
}

pub fn use_mutation_diagnostic_runtime() -> MutationDiagnosticRuntime {
    expect_context::<MutationDiagnosticRuntime>()
}
