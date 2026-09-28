// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Typed dispatch bridge between Personal semantic mutation attempts and the
//! shared Holochain provider.
//!
//! A decoded provider response is only call-success evidence. Domain mutation
//! commit, conflict, receipt identity, and read-model reconciliation remain
//! owned by the decoded response contract and the existing mutation ledger.

use serde::{Serialize, de::DeserializeOwned};

use mycelix_leptos_core::{HolochainCallPhase, HolochainCtx};

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

/// Whether a typed mutation dispatch was known not to reach provider transport
/// or whether its source-chain outcome remains unresolved.
///
/// This is an evidence disposition, not a commit/rollback result. In
/// particular, `OutcomeUnknown` does not assert that a mutation committed; it
/// only records that the typed invocation result is insufficient to establish
/// that it did not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersonalMutationDispatchDisposition {
    /// No provider zome request was dispatched.
    NotSubmitted,
    /// The provider call was attempted or a response was acquired, but no
    /// typed domain result established whether the source-chain mutation
    /// occurred.
    OutcomeUnknown,
}

impl PersonalMutationDispatchError {
    pub const fn invocation_failure(&self) -> Option<&PersonalMutationFailure> {
        match self {
            Self::Invocation(failure) => Some(failure),
            Self::PersonalAttemptSequenceExhausted { .. }
            | Self::ProviderTargetMismatch(_) => None,
        }
    }

    /// Classify how much the typed invocation failure establishes about whether
    /// a provider request reached the source chain.
    ///
    /// Admission and Encode fail before transport dispatch. Transport can fail
    /// after a request was sent, and Decode can fail after response bytes were
    /// acquired, so neither establishes source-chain mutation absence.
    pub fn disposition(&self) -> PersonalMutationDispatchDisposition {
        match self {
            Self::PersonalAttemptSequenceExhausted { .. } => {
                PersonalMutationDispatchDisposition::NotSubmitted
            }
            Self::ProviderTargetMismatch(_) => {
                PersonalMutationDispatchDisposition::OutcomeUnknown
            }
            Self::Invocation(failure) => match failure
                .provider_failure_observation()
                .map(|observation| observation.error().phase())
            {
                Some(HolochainCallPhase::Admission | HolochainCallPhase::Encode) => {
                    PersonalMutationDispatchDisposition::NotSubmitted
                }
                Some(HolochainCallPhase::Transport | HolochainCallPhase::Decode) | None => {
                    PersonalMutationDispatchDisposition::OutcomeUnknown
                }
            },
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

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_leptos_core::{
        HolochainCallAttemptId, HolochainCallError, HolochainCallFailureObservation,
    };
    use mycelix_leptos_client::ClientError;

    fn failure_for_phase(phase: HolochainCallPhase) -> PersonalMutationDispatchError {
        let binding =
            crate::mutation_diagnostics::PersonalMutationCallBinding::for_target(
                PersonalMutationTarget::Profile,
            );
        let mut sequence =
            crate::mutation_diagnostics::PersonalMutationAttemptSequence::default();
        let attempt = sequence.admit(binding.clone()).expect("personal attempt");
        let observation = HolochainCallFailureObservation::new(
            HolochainCallAttemptId::FIRST,
            HolochainCallError::new(
                phase,
                binding.role(),
                binding.zome(),
                binding.function(),
                ClientError::NotConnected,
            ),
        );
        let failure = attempt
            .fail(observation.into())
            .expect("provider target must match");
        PersonalMutationDispatchError::Invocation(failure)
    }

    #[test]
    fn admission_and_encode_are_not_submitted() {
        assert_eq!(
            failure_for_phase(HolochainCallPhase::Admission).disposition(),
            PersonalMutationDispatchDisposition::NotSubmitted
        );
        assert_eq!(
            failure_for_phase(HolochainCallPhase::Encode).disposition(),
            PersonalMutationDispatchDisposition::NotSubmitted
        );
    }

    #[test]
    fn transport_and_decode_are_outcome_unknown() {
        assert_eq!(
            failure_for_phase(HolochainCallPhase::Transport).disposition(),
            PersonalMutationDispatchDisposition::OutcomeUnknown
        );
        assert_eq!(
            failure_for_phase(HolochainCallPhase::Decode).disposition(),
            PersonalMutationDispatchDisposition::OutcomeUnknown
        );
    }

    #[test]
    fn personal_attempt_sequence_exhaustion_is_not_submitted() {
        let error = PersonalMutationDispatchError::PersonalAttemptSequenceExhausted {
            target: PersonalMutationTarget::Profile,
        };
        assert_eq!(
            error.disposition(),
            PersonalMutationDispatchDisposition::NotSubmitted
        );
    }

    #[test]
    fn provider_target_mismatch_is_outcome_unknown() {
        let profile =
            crate::mutation_diagnostics::PersonalMutationCallBinding::for_target(
                PersonalMutationTarget::Profile,
            );
        let mut sequence =
            crate::mutation_diagnostics::PersonalMutationAttemptSequence::default();
        let attempt = sequence.admit(profile.clone()).expect("personal attempt");
        let mismatch = attempt
            .fail(mycelix_leptos_core::HolochainCallInvocationError::attempt_sequence_exhausted(
                "personal",
                "health_vault",
                "grant_consent_view",
            ))
            .expect_err("mismatch must be refused");
        let error = PersonalMutationDispatchError::ProviderTargetMismatch(mismatch);
        assert_eq!(
            error.disposition(),
            PersonalMutationDispatchDisposition::OutcomeUnknown
        );
    }
}
