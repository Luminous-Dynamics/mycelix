// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed dispatch bridge between Personal semantic mutation attempts and the
//! shared Holochain provider.
//!
//! A decoded provider response is only call-success evidence. Domain mutation
//! commit, conflict, receipt identity, and read-model reconciliation remain
//! owned by the decoded response contract and the existing mutation ledger.

use serde::{Serialize, de::DeserializeOwned};

use mycelix_leptos_core::HolochainCtx;

use crate::mutation_diagnostic_runtime::MutationDiagnosticRuntime;
use crate::mutation_diagnostics::{
    PersonalMutationDiagnosticTargetMismatch, PersonalMutationFailure, PersonalMutationSuccess,
};
use crate::mutation_state::PersonalMutationTarget;

/// Fail-closed dispatch errors at the Personal semantic/provider boundary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PersonalMutationDispatchError {
    /// No further app-local semantic attempt identity can be allocated in this
    /// Personal runtime lifetime. No provider call is made.
    PersonalAttemptSequenceExhausted { target: PersonalMutationTarget },
    /// The provider invocation failed on the exact provider target bound to the
    /// semantic Personal attempt. The failure has already been published to the
    /// semantic diagnostic runtime when this variant is returned.
    Invocation(PersonalMutationFailure),
    /// The provider returned typed failure evidence for a different role/zome/
    /// function than the semantic binding admitted. Nothing is published to the
    /// semantic diagnostic ledger in this case.
    ProviderTargetMismatch(PersonalMutationDiagnosticTargetMismatch),
}

impl PersonalMutationDispatchError {
    pub const fn invocation_failure(&self) -> Option<&PersonalMutationFailure> {
        match self {
            Self::Invocation(failure) => Some(failure),
            Self::PersonalAttemptSequenceExhausted { .. }
            | Self::ProviderTargetMismatch(_) => None,
        }
    }
}

/// Execute exactly one typed provider call under one admitted Personal semantic
/// mutation attempt.
///
/// The provider target is taken exclusively from the semantic binding created
/// by `MutationDiagnosticRuntime::admit`; callers do not pass role/zome/function
/// independently and therefore cannot accidentally publish a completion under a
/// different Personal target.
///
/// On provider success the returned [`PersonalMutationSuccess`] retains only the
/// app-local semantic attempt identity plus decoded value. That value may still
/// describe a domain conflict or another non-commit outcome.
pub async fn dispatch_personal_mutation_typed<I, O>(
    hc: &HolochainCtx,
    diagnostics: MutationDiagnosticRuntime,
    target: PersonalMutationTarget,
    input: &I,
) -> Result<PersonalMutationSuccess<O>, PersonalMutationDispatchError>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let Some(attempt) = diagnostics.admit(target.clone()) else {
        return Err(PersonalMutationDispatchError::PersonalAttemptSequenceExhausted {
            target,
        });
    };

    // Copy the exact provider target out of the admitted semantic binding before
    // awaiting transport. The attempt itself remains available to be consumed
    // exactly once into the terminal semantic observation afterward.
    let role = attempt.binding().role().to_string();
    let zome = attempt.binding().zome().to_string();
    let function = attempt.binding().function().to_string();

    match hc
        .call_zome_typed::<_, O>(&role, &zome, &function, input)
        .await
    {
        Ok(value) => {
            let success = attempt.succeed(value);
            diagnostics.observe_success(&success);
            Ok(success)
        }
        Err(error) => match attempt.fail(error) {
            Ok(failure) => {
                diagnostics.observe_failure(failure.clone());
                Err(PersonalMutationDispatchError::Invocation(failure))
            }
            Err(mismatch) => Err(PersonalMutationDispatchError::ProviderTargetMismatch(
                mismatch,
            )),
        },
    }
}
