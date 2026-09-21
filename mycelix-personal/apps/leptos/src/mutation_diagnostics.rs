// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Personal mutation-attempt correlation for typed Holochain call diagnostics.
//!
//! Provider attempt ids order transport-facing calls. Personal mutation attempts
//! order application semantics such as one exact Profile write, Health consent,
//! or Preference pair. These identities are intentionally distinct.
//!
//! `PersonalMutationAttemptId != HolochainCallAttemptId`
//!
//! `PersonalMutationAttemptId != action hash`
//!
//! `PersonalMutationAttemptId != mutation receipt`
//!
//! `PersonalMutationAttemptId != commit evidence`

use mycelix_leptos_core::{
    HolochainCallFailureObservation, HolochainCallInvocationError, HolochainCallTarget,
};

use crate::mutation_state::PersonalMutationTarget;

/// App-lifetime monotonic identity assigned when one semantic Personal mutation
/// attempt is admitted for dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PersonalMutationAttemptId(u64);

impl PersonalMutationAttemptId {
    pub const FIRST: Self = Self(1);

    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Non-cloneable allocator for Personal semantic mutation attempts.
///
/// Exhaustion fails closed rather than wrapping, so an ancient semantic attempt
/// can never become numerically indistinguishable from a future one.
#[derive(Debug, PartialEq, Eq)]
pub struct PersonalMutationAttemptSequence {
    next: Option<u64>,
}

impl Default for PersonalMutationAttemptSequence {
    fn default() -> Self {
        Self { next: Some(1) }
    }
}

impl PersonalMutationAttemptSequence {
    pub fn allocate(&mut self) -> Option<PersonalMutationAttemptId> {
        let current = self.next?;
        self.next = current.checked_add(1);
        Some(PersonalMutationAttemptId(current))
    }

    pub fn admit(
        &mut self,
        binding: PersonalMutationCallBinding,
    ) -> Option<PersonalMutationAttempt> {
        let id = self.allocate()?;
        Some(PersonalMutationAttempt { id, binding })
    }

    pub const fn is_exhausted(&self) -> bool {
        self.next.is_none()
    }
}

/// Exact Personal semantic target paired with the provider call target that is
/// permitted to service it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalMutationCallBinding {
    target: PersonalMutationTarget,
    provider_target: HolochainCallTarget,
}

impl PersonalMutationCallBinding {
    pub fn for_target(target: PersonalMutationTarget) -> Self {
        let provider_target = match &target {
            PersonalMutationTarget::Profile => HolochainCallTarget::new(
                "personal",
                "identity_vault",
                "set_profile_view_if_current",
            ),
            PersonalMutationTarget::HealthConsent => HolochainCallTarget::new(
                "personal",
                "health_vault",
                "grant_consent_view",
            ),
            PersonalMutationTarget::Preference { .. } => HolochainCallTarget::new(
                "personal",
                "data_preferences",
                "set_preference_view_if_current",
            ),
        };

        Self {
            target,
            provider_target,
        }
    }

    pub const fn target(&self) -> &PersonalMutationTarget {
        &self.target
    }

    pub const fn provider_target(&self) -> &HolochainCallTarget {
        &self.provider_target
    }

    pub fn role(&self) -> &str {
        self.provider_target.role()
    }

    pub fn zome(&self) -> &str {
        self.provider_target.zome()
    }

    pub fn function(&self) -> &str {
        self.provider_target.function()
    }
}

/// One admitted semantic mutation attempt.
///
/// The value is intentionally non-cloneable. It is consumed into exactly one
/// terminal Personal success or failure observation.
#[derive(Debug, PartialEq, Eq)]
pub struct PersonalMutationAttempt {
    id: PersonalMutationAttemptId,
    binding: PersonalMutationCallBinding,
}

impl PersonalMutationAttempt {
    pub const fn id(&self) -> PersonalMutationAttemptId {
        self.id
    }

    pub const fn binding(&self) -> &PersonalMutationCallBinding {
        &self.binding
    }

    pub fn succeed<T>(self, value: T) -> PersonalMutationSuccess<T> {
        PersonalMutationSuccess {
            attempt_id: self.id,
            target: self.binding.target,
            value,
        }
    }

    /// Consume this semantic attempt into one typed failure only when the
    /// provider invocation error belongs to the provider target bound at
    /// admission.
    pub fn fail(
        self,
        error: HolochainCallInvocationError,
    ) -> Result<PersonalMutationFailure, PersonalMutationDiagnosticTargetMismatch> {
        let observed = HolochainCallTarget::new(error.role(), error.zome(), error.function());
        let expected = self.binding.provider_target.clone();
        if observed != expected {
            return Err(PersonalMutationDiagnosticTargetMismatch {
                expected,
                observed,
                error,
            });
        }

        Ok(PersonalMutationFailure {
            attempt_id: self.id,
            target: self.binding.target,
            error,
        })
    }
}

/// Successfully decoded provider result correlated to one exact Personal
/// semantic mutation attempt.
///
/// This is application correlation evidence only. The decoded value may itself
/// describe a domain conflict or another non-commit result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalMutationSuccess<T> {
    attempt_id: PersonalMutationAttemptId,
    target: PersonalMutationTarget,
    value: T,
}

impl<T> PersonalMutationSuccess<T> {
    pub const fn attempt_id(&self) -> PersonalMutationAttemptId {
        self.attempt_id
    }

    pub const fn target(&self) -> &PersonalMutationTarget {
        &self.target
    }

    pub const fn value(&self) -> &T {
        &self.value
    }

    pub fn into_value(self) -> T {
        self.value
    }
}

/// Typed provider invocation failure correlated to one exact Personal semantic
/// mutation attempt.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalMutationFailure {
    attempt_id: PersonalMutationAttemptId,
    target: PersonalMutationTarget,
    error: HolochainCallInvocationError,
}

impl PersonalMutationFailure {
    pub const fn attempt_id(&self) -> PersonalMutationAttemptId {
        self.attempt_id
    }

    pub const fn target(&self) -> &PersonalMutationTarget {
        &self.target
    }

    pub const fn invocation_error(&self) -> &HolochainCallInvocationError {
        &self.error
    }

    pub fn provider_failure_observation(&self) -> Option<&HolochainCallFailureObservation> {
        self.error.failure_observation()
    }
}

/// Fail-closed refusal to bind an invocation error from the wrong provider
/// target to a Personal semantic mutation attempt.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalMutationDiagnosticTargetMismatch {
    pub expected: HolochainCallTarget,
    pub observed: HolochainCallTarget,
    error: HolochainCallInvocationError,
}

impl PersonalMutationDiagnosticTargetMismatch {
    pub const fn invocation_error(&self) -> &HolochainCallInvocationError {
        &self.error
    }

    pub fn into_invocation_error(self) -> HolochainCallInvocationError {
        self.error
    }
}

/// Ordered diagnostic evidence for one exact Personal mutation target.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersonalMutationTargetDiagnostic {
    target: PersonalMutationTarget,
    latest_completed_attempt: Option<PersonalMutationAttemptId>,
    latest_failure: Option<PersonalMutationFailure>,
}

impl PersonalMutationTargetDiagnostic {
    fn new(target: PersonalMutationTarget) -> Self {
        Self {
            target,
            latest_completed_attempt: None,
            latest_failure: None,
        }
    }

    pub const fn target(&self) -> &PersonalMutationTarget {
        &self.target
    }

    pub const fn latest_completed_attempt(&self) -> Option<PersonalMutationAttemptId> {
        self.latest_completed_attempt
    }

    pub const fn latest_failure(&self) -> Option<&PersonalMutationFailure> {
        self.latest_failure.as_ref()
    }

    fn accepts_completion(&self, attempt_id: PersonalMutationAttemptId) -> bool {
        self.latest_completed_attempt
            .is_none_or(|current| attempt_id > current)
    }

    fn observe_success<T>(&mut self, success: &PersonalMutationSuccess<T>) -> bool {
        if !self.accepts_completion(success.attempt_id()) {
            return false;
        }
        self.latest_completed_attempt = Some(success.attempt_id());
        self.latest_failure = None;
        true
    }

    fn observe_failure(&mut self, failure: PersonalMutationFailure) -> bool {
        if !self.accepts_completion(failure.attempt_id()) {
            return false;
        }
        self.latest_completed_attempt = Some(failure.attempt_id());
        self.latest_failure = Some(failure);
        true
    }
}

/// App-local typed diagnostic ledger with independent ordering for each exact
/// Personal mutation target, including each individual Preference pair.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PersonalMutationDiagnosticLedger {
    entries: Vec<PersonalMutationTargetDiagnostic>,
}

impl PersonalMutationDiagnosticLedger {
    pub fn target(
        &self,
        target: &PersonalMutationTarget,
    ) -> Option<&PersonalMutationTargetDiagnostic> {
        self.entries.iter().find(|entry| entry.target() == target)
    }

    pub fn latest_failure_for(
        &self,
        target: &PersonalMutationTarget,
    ) -> Option<&PersonalMutationFailure> {
        self.target(target)
            .and_then(PersonalMutationTargetDiagnostic::latest_failure)
    }

    pub fn target_count(&self) -> usize {
        self.entries.len()
    }

    pub fn observe_success<T>(&mut self, success: &PersonalMutationSuccess<T>) -> bool {
        self.entry_mut(success.target().clone())
            .observe_success(success)
    }

    pub fn observe_failure(&mut self, failure: PersonalMutationFailure) -> bool {
        let target = failure.target().clone();
        self.entry_mut(target).observe_failure(failure)
    }

    fn entry_mut(
        &mut self,
        target: PersonalMutationTarget,
    ) -> &mut PersonalMutationTargetDiagnostic {
        if let Some(index) = self
            .entries
            .iter()
            .position(|entry| entry.target() == &target)
        {
            return &mut self.entries[index];
        }

        self.entries.push(PersonalMutationTargetDiagnostic::new(target));
        self.entries
            .last_mut()
            .expect("entry was pushed immediately above")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_leptos_client::{ClientError, ConductorError, ConductorErrorKind};
    use mycelix_leptos_core::{
        HolochainCallAttemptSequence, HolochainCallError, HolochainCallPhase,
    };

    fn provider_failure(
        sequence: &mut HolochainCallAttemptSequence,
        binding: &PersonalMutationCallBinding,
        message: &str,
    ) -> HolochainCallInvocationError {
        let provider_id = sequence.allocate().expect("provider attempt id");
        HolochainCallFailureObservation::new(
            provider_id,
            HolochainCallError::new(
                HolochainCallPhase::Transport,
                binding.role(),
                binding.zome(),
                binding.function(),
                ClientError::Conductor(ConductorError {
                    kind: ConductorErrorKind::Ribosome,
                    message: message.into(),
                }),
            ),
        )
        .into()
    }

    #[test]
    fn personal_attempt_allocation_is_monotonic_and_exhausts_without_wrap() {
        let mut sequence = PersonalMutationAttemptSequence::default();
        assert_eq!(sequence.allocate(), Some(PersonalMutationAttemptId::FIRST));
        assert_eq!(sequence.allocate().map(PersonalMutationAttemptId::get), Some(2));

        let mut exhausted = PersonalMutationAttemptSequence {
            next: Some(u64::MAX),
        };
        assert_eq!(
            exhausted.allocate().map(PersonalMutationAttemptId::get),
            Some(u64::MAX)
        );
        assert!(exhausted.is_exhausted());
        assert_eq!(exhausted.allocate(), None);
    }

    #[test]
    fn preference_pairs_share_provider_target_but_not_semantic_target() {
        let a = PersonalMutationCallBinding::for_target(PersonalMutationTarget::preference(
            "health", "finance",
        ));
        let b = PersonalMutationCallBinding::for_target(PersonalMutationTarget::preference(
            "health", "knowledge",
        ));

        assert_eq!(a.provider_target(), b.provider_target());
        assert_ne!(a.target(), b.target());
    }

    #[test]
    fn provider_sequence_exhaustion_can_be_scoped_to_the_admitted_personal_attempt() {
        let binding = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "finance"),
        );
        let mut personal = PersonalMutationAttemptSequence::default();
        let attempt = personal.admit(binding.clone()).expect("personal attempt");
        let failure = attempt
            .fail(HolochainCallInvocationError::attempt_sequence_exhausted(
                binding.role(),
                binding.zome(),
                binding.function(),
            ))
            .expect("matching provider target");

        assert_eq!(failure.attempt_id(), PersonalMutationAttemptId::FIRST);
        assert_eq!(failure.target(), binding.target());
        assert!(failure.provider_failure_observation().is_none());
    }

    #[test]
    fn unrelated_pair_success_does_not_clear_other_pairs_failure() {
        let failed_pair = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "finance"),
        );
        let other_pair = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "knowledge"),
        );
        let mut personal = PersonalMutationAttemptSequence::default();
        let mut provider = HolochainCallAttemptSequence::default();
        let failed_attempt = personal.admit(failed_pair.clone()).expect("failed attempt");
        let other_attempt = personal.admit(other_pair.clone()).expect("other attempt");
        let failure = failed_attempt
            .fail(provider_failure(&mut provider, &failed_pair, "pair A failed"))
            .expect("matching provider target");
        let success = other_attempt.succeed(());
        let mut ledger = PersonalMutationDiagnosticLedger::default();

        assert!(ledger.observe_failure(failure));
        assert!(ledger.observe_success(&success));
        assert!(ledger.latest_failure_for(failed_pair.target()).is_some());
        assert!(ledger.latest_failure_for(other_pair.target()).is_none());
        assert_eq!(ledger.target_count(), 2);
    }

    #[test]
    fn newer_success_clears_same_pair_failure() {
        let binding = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "finance"),
        );
        let mut personal = PersonalMutationAttemptSequence::default();
        let mut provider = HolochainCallAttemptSequence::default();
        let first = personal.admit(binding.clone()).expect("first attempt");
        let second = personal.admit(binding.clone()).expect("second attempt");
        let failure = first
            .fail(provider_failure(&mut provider, &binding, "failed"))
            .expect("matching provider target");
        let success = second.succeed(());
        let mut ledger = PersonalMutationDiagnosticLedger::default();

        assert!(ledger.observe_failure(failure));
        assert!(ledger.observe_success(&success));
        assert!(ledger.latest_failure_for(binding.target()).is_none());
        assert_eq!(
            ledger
                .target(binding.target())
                .and_then(PersonalMutationTargetDiagnostic::latest_completed_attempt),
            Some(success.attempt_id()),
        );
    }

    #[test]
    fn late_older_success_cannot_clear_newer_same_pair_failure() {
        let binding = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "finance"),
        );
        let mut personal = PersonalMutationAttemptSequence::default();
        let older = personal.admit(binding.clone()).expect("older attempt");
        let newer = personal.admit(binding.clone()).expect("newer attempt");
        let older_success = older.succeed(());

        // Deliberately allocate the provider failure after the older Personal
        // attempt to prove provider ordering is not the Personal ordering key.
        let mut provider = HolochainCallAttemptSequence::default();
        let newer_failure = newer
            .fail(provider_failure(&mut provider, &binding, "newer failure"))
            .expect("matching provider target");
        let newer_id = newer_failure.attempt_id();
        let mut ledger = PersonalMutationDiagnosticLedger::default();

        assert!(ledger.observe_failure(newer_failure));
        assert!(!ledger.observe_success(&older_success));
        assert_eq!(
            ledger
                .latest_failure_for(binding.target())
                .map(PersonalMutationFailure::attempt_id),
            Some(newer_id),
        );
    }

    #[test]
    fn higher_provider_attempt_id_cannot_override_newer_personal_attempt() {
        let binding = PersonalMutationCallBinding::for_target(PersonalMutationTarget::Profile);
        let mut personal = PersonalMutationAttemptSequence::default();
        let older = personal.admit(binding.clone()).expect("older personal attempt");
        let newer = personal.admit(binding.clone()).expect("newer personal attempt");
        let newer_success = newer.succeed(());

        let mut provider = HolochainCallAttemptSequence::default();
        let _provider_low = provider.allocate().expect("provider low");
        let older_failure = older
            .fail(provider_failure(
                &mut provider,
                &binding,
                "older personal attempt, higher provider id",
            ))
            .expect("matching provider target");
        let mut ledger = PersonalMutationDiagnosticLedger::default();

        assert!(ledger.observe_success(&newer_success));
        assert!(!ledger.observe_failure(older_failure));
        assert!(ledger.latest_failure_for(binding.target()).is_none());
    }

    #[test]
    fn provider_target_mismatch_fails_closed_and_preserves_error() {
        let profile = PersonalMutationCallBinding::for_target(PersonalMutationTarget::Profile);
        let mut personal = PersonalMutationAttemptSequence::default();
        let attempt = personal.admit(profile.clone()).expect("profile attempt");
        let mismatch_error = HolochainCallInvocationError::attempt_sequence_exhausted(
            "personal",
            "health_vault",
            "grant_consent_view",
        );

        let mismatch = attempt
            .fail(mismatch_error)
            .expect_err("mismatched provider target must be refused");
        assert_eq!(mismatch.expected, profile.provider_target().clone());
        assert_eq!(mismatch.observed.zome(), "health_vault");
        assert_eq!(mismatch.invocation_error().zome(), "health_vault");
    }

    #[test]
    fn late_older_failure_cannot_replace_newer_same_pair_success() {
        let binding = PersonalMutationCallBinding::for_target(
            PersonalMutationTarget::preference("health", "finance"),
        );
        let mut personal = PersonalMutationAttemptSequence::default();
        let older = personal.admit(binding.clone()).expect("older attempt");
        let newer = personal.admit(binding.clone()).expect("newer attempt");
        let newer_success = newer.succeed(());
        let mut provider = HolochainCallAttemptSequence::default();
        let older_failure = older
            .fail(provider_failure(&mut provider, &binding, "late old failure"))
            .expect("matching provider target");
        let mut ledger = PersonalMutationDiagnosticLedger::default();

        assert!(ledger.observe_success(&newer_success));
        assert!(!ledger.observe_failure(older_failure));
        assert!(ledger.latest_failure_for(binding.target()).is_none());
    }
}
