// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Target-scoped ordering for typed Holochain call diagnostics.
//!
//! A provider-wide "latest call" view is useful for coarse telemetry, but it is
//! too lossy for actionable diagnostics: a successful background read on one
//! zome must not erase a failure from an unrelated mutation target merely
//! because the read completed later. This module maintains one ordered
//! completion lane per `(role, zome, function)` target while keeping provider
//! attempt-sequence exhaustion as separate provider-wide evidence.
//!
//! Target-scoped diagnostic evidence remains observational only. It does not
//! establish authorization, mutation commit/rollback, retry safety, or a
//! stronger conductor error class than the transport actually supplied.

use std::collections::BTreeMap;

use crate::{
    HolochainCallAdmissionExhaustion, HolochainCallAttemptId,
    HolochainCallFailureObservation,
};

/// Stable provider-local identity of one logical zome-call target.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct HolochainCallTarget {
    role: String,
    zome: String,
    function: String,
}

impl HolochainCallTarget {
    pub fn new(
        role: impl Into<String>,
        zome: impl Into<String>,
        function: impl Into<String>,
    ) -> Self {
        Self {
            role: role.into(),
            zome: zome.into(),
            function: function.into(),
        }
    }

    pub fn from_failure(observation: &HolochainCallFailureObservation) -> Self {
        Self::new(
            observation.error().role(),
            observation.error().zome(),
            observation.error().function(),
        )
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
}

/// Ordered completion evidence for one exact call target.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HolochainCallTargetDiagnostic {
    latest_completed_attempt: Option<HolochainCallAttemptId>,
    latest_failure: Option<HolochainCallFailureObservation>,
}

impl HolochainCallTargetDiagnostic {
    pub const fn latest_completed_attempt(&self) -> Option<HolochainCallAttemptId> {
        self.latest_completed_attempt
    }

    pub const fn latest_failure(&self) -> Option<&HolochainCallFailureObservation> {
        self.latest_failure.as_ref()
    }

    fn accepts_completion(&self, attempt_id: HolochainCallAttemptId) -> bool {
        self.latest_completed_attempt
            .is_none_or(|current| attempt_id > current)
    }

    fn observe_success(&mut self, attempt_id: HolochainCallAttemptId) -> bool {
        if !self.accepts_completion(attempt_id) {
            return false;
        }
        self.latest_completed_attempt = Some(attempt_id);
        self.latest_failure = None;
        true
    }

    fn observe_failure(&mut self, observation: HolochainCallFailureObservation) -> bool {
        let attempt_id = observation.attempt_id();
        if !self.accepts_completion(attempt_id) {
            return false;
        }
        self.latest_completed_attempt = Some(attempt_id);
        self.latest_failure = Some(observation);
        true
    }
}

/// Provider-local typed diagnostic ledger with independent ordering per target.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HolochainCallDiagnosticLedger {
    targets: BTreeMap<HolochainCallTarget, HolochainCallTargetDiagnostic>,
    attempt_sequence_exhaustion: Option<HolochainCallAdmissionExhaustion>,
}

impl HolochainCallDiagnosticLedger {
    pub fn target(
        &self,
        target: &HolochainCallTarget,
    ) -> Option<&HolochainCallTargetDiagnostic> {
        self.targets.get(target)
    }

    pub fn latest_failure_for(
        &self,
        target: &HolochainCallTarget,
    ) -> Option<&HolochainCallFailureObservation> {
        self.target(target)
            .and_then(HolochainCallTargetDiagnostic::latest_failure)
    }

    pub fn attempt_sequence_exhaustion(&self) -> Option<&HolochainCallAdmissionExhaustion> {
        self.attempt_sequence_exhaustion.as_ref()
    }

    pub const fn is_attempt_sequence_exhausted(&self) -> bool {
        self.attempt_sequence_exhaustion.is_some()
    }

    pub fn target_count(&self) -> usize {
        self.targets.len()
    }

    /// Publish a successful completion only within the matching target lane.
    ///
    /// A success on target B never clears failure evidence retained for target A.
    pub fn observe_success(
        &mut self,
        target: HolochainCallTarget,
        attempt_id: HolochainCallAttemptId,
    ) -> bool {
        self.targets.entry(target).or_default().observe_success(attempt_id)
    }

    /// Publish a failed completion only within the target named by the bound
    /// failure observation.
    pub fn observe_failure(&mut self, observation: HolochainCallFailureObservation) -> bool {
        let target = HolochainCallTarget::from_failure(&observation);
        self.targets
            .entry(target)
            .or_default()
            .observe_failure(observation)
    }

    /// Record the first provider-wide attempt-sequence exhaustion observation.
    ///
    /// Later rejected calls have no attempt identity and therefore carry no
    /// stronger ordering evidence. The first exhaustion marker remains sticky.
    pub fn observe_attempt_sequence_exhaustion(
        &mut self,
        exhaustion: HolochainCallAdmissionExhaustion,
    ) -> bool {
        if self.attempt_sequence_exhaustion.is_some() {
            return false;
        }
        self.attempt_sequence_exhaustion = Some(exhaustion);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HolochainCallAttemptSequence, HolochainCallError, HolochainCallPhase};
    use mycelix_leptos_client::{ClientError, ConductorError, ConductorErrorKind};

    fn target(zome: &str, function: &str) -> HolochainCallTarget {
        HolochainCallTarget::new("personal", zome, function)
    }

    fn next_id(sequence: &mut HolochainCallAttemptSequence) -> HolochainCallAttemptId {
        sequence.allocate().expect("provider attempt id")
    }

    fn failure(
        id: HolochainCallAttemptId,
        zome: &str,
        function: &str,
        message: &str,
    ) -> HolochainCallFailureObservation {
        HolochainCallFailureObservation::new(
            id,
            HolochainCallError::new(
                HolochainCallPhase::Transport,
                "personal",
                zome,
                function,
                ClientError::Conductor(ConductorError {
                    kind: ConductorErrorKind::Ribosome,
                    message: message.into(),
                }),
            ),
        )
    }

    #[test]
    fn unrelated_success_does_not_clear_another_targets_failure() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let profile_id = next_id(&mut sequence);
        let wallet_id = next_id(&mut sequence);
        let profile = target("identity_vault", "set_profile_view_if_current");
        let wallet = target("credential_wallet", "get_my_credentials_view");
        let mut ledger = HolochainCallDiagnosticLedger::default();

        let profile_failure = failure(
            profile_id,
            "identity_vault",
            "set_profile_view_if_current",
            "profile rejected",
        );
        assert!(ledger.observe_failure(profile_failure.clone()));
        assert!(ledger.observe_success(wallet.clone(), wallet_id));

        assert_eq!(ledger.latest_failure_for(&profile), Some(&profile_failure));
        assert_eq!(ledger.latest_failure_for(&wallet), None);
    }

    #[test]
    fn newer_success_on_same_target_clears_that_targets_failure() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let failed_id = next_id(&mut sequence);
        let success_id = next_id(&mut sequence);
        let profile = target("identity_vault", "set_profile_view_if_current");
        let mut ledger = HolochainCallDiagnosticLedger::default();

        assert!(ledger.observe_failure(failure(
            failed_id,
            "identity_vault",
            "set_profile_view_if_current",
            "profile rejected",
        )));
        assert!(ledger.observe_success(profile.clone(), success_id));

        let state = ledger.target(&profile).expect("profile diagnostic lane");
        assert_eq!(state.latest_completed_attempt(), Some(success_id));
        assert_eq!(state.latest_failure(), None);
    }

    #[test]
    fn late_older_failure_cannot_replace_newer_same_target_success() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let older_id = next_id(&mut sequence);
        let newer_id = next_id(&mut sequence);
        let profile = target("identity_vault", "set_profile_view_if_current");
        let mut ledger = HolochainCallDiagnosticLedger::default();

        assert!(ledger.observe_success(profile.clone(), newer_id));
        assert!(!ledger.observe_failure(failure(
            older_id,
            "identity_vault",
            "set_profile_view_if_current",
            "late old failure",
        )));

        let state = ledger.target(&profile).expect("profile diagnostic lane");
        assert_eq!(state.latest_completed_attempt(), Some(newer_id));
        assert_eq!(state.latest_failure(), None);
    }

    #[test]
    fn independent_targets_retain_independent_failures() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let profile_id = next_id(&mut sequence);
        let preference_id = next_id(&mut sequence);
        let profile = target("identity_vault", "set_profile_view_if_current");
        let preference = target("data_preferences", "set_preference_view_if_current");
        let mut ledger = HolochainCallDiagnosticLedger::default();

        let profile_failure = failure(
            profile_id,
            "identity_vault",
            "set_profile_view_if_current",
            "profile failure",
        );
        let preference_failure = failure(
            preference_id,
            "data_preferences",
            "set_preference_view_if_current",
            "preference failure",
        );
        assert!(ledger.observe_failure(profile_failure.clone()));
        assert!(ledger.observe_failure(preference_failure.clone()));

        assert_eq!(ledger.target_count(), 2);
        assert_eq!(ledger.latest_failure_for(&profile), Some(&profile_failure));
        assert_eq!(
            ledger.latest_failure_for(&preference),
            Some(&preference_failure)
        );
    }

    #[test]
    fn target_is_derived_from_bound_failure_identity_not_message_text() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let id = next_id(&mut sequence);
        let observation = failure(
            id,
            "identity_vault",
            "set_profile_view_if_current",
            "mentions data_preferences.set_preference_view_if_current but is not that target",
        );

        let derived = HolochainCallTarget::from_failure(&observation);
        assert_eq!(derived.role(), "personal");
        assert_eq!(derived.zome(), "identity_vault");
        assert_eq!(derived.function(), "set_profile_view_if_current");
    }

    #[test]
    fn sequence_exhaustion_is_provider_wide_and_sticky() {
        let mut ledger = HolochainCallDiagnosticLedger::default();
        let first = HolochainCallAdmissionExhaustion::new(
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
        );

        assert!(ledger.observe_attempt_sequence_exhaustion(first.clone()));
        assert!(!ledger.observe_attempt_sequence_exhaustion(
            HolochainCallAdmissionExhaustion::new(
                "personal",
                "data_preferences",
                "set_preference_view_if_current",
            ),
        ));
        assert_eq!(ledger.attempt_sequence_exhaustion(), Some(&first));
    }

    #[test]
    fn message_cannot_strengthen_ribosome_failure_into_head_moved() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let id = next_id(&mut sequence);
        let profile = target("identity_vault", "set_profile_view_if_current");
        let mut ledger = HolochainCallDiagnosticLedger::default();

        assert!(ledger.observe_failure(failure(
            id,
            "identity_vault",
            "set_profile_view_if_current",
            "source chain head moved",
        )));

        let retained = ledger
            .latest_failure_for(&profile)
            .expect("retained typed failure");
        assert_eq!(
            retained.failure_kind(),
            crate::HolochainCallFailureKind::Conductor(ConductorErrorKind::Ribosome)
        );
    }
}
