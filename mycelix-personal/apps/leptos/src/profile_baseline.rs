// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Profile baseline evidence for conditional Personal mutations.
//!
//! `None` in the backend CAS contract means authoritative empty state. This
//! module keeps that meaning separate from an unestablished frontend baseline.

use crate::context::PersonalSourceState;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfileBaselineEvidence {
    /// No successful Identity publication has established the Profile baseline.
    Unestablished,
    /// A successful Identity publication established that no Profile exists.
    AuthoritativeEmpty,
    /// A successful Identity publication established this exact Profile action.
    Action(String),
}

impl ProfileBaselineEvidence {
    /// Derive Profile baseline authority only from atomically published Identity
    /// source disposition + action identity.
    ///
    /// `Live` may still carry an empty Profile when keys are present, so both
    /// `Live` and `Empty` are established source dispositions. Demo, pending,
    /// degraded and unavailable states never manufacture Live CAS authority.
    pub fn from_published_identity(
        state: PersonalSourceState,
        action_hash: Option<String>,
    ) -> Self {
        match state {
            PersonalSourceState::Live | PersonalSourceState::Empty => match action_hash {
                Some(hash) => Self::Action(hash),
                None => Self::AuthoritativeEmpty,
            },
            PersonalSourceState::Demo
            | PersonalSourceState::AwaitingLive
            | PersonalSourceState::LoadingLive
            | PersonalSourceState::Degraded
            | PersonalSourceState::Unavailable => Self::Unestablished,
        }
    }

    /// Convert an established baseline into the backend CAS expectation.
    ///
    /// Outer `None` means no conditional write may be constructed.
    /// `Some(None)` means authoritative empty and is the only path that may
    /// produce backend `expected_action_hash = None`.
    pub fn cas_expected_action_hash(&self) -> Option<Option<String>> {
        match self {
            Self::Unestablished => None,
            Self::AuthoritativeEmpty => Some(None),
            Self::Action(hash) => Some(Some(hash.clone())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_live_state_is_unestablished() {
        assert_eq!(
            ProfileBaselineEvidence::from_published_identity(
                PersonalSourceState::AwaitingLive,
                None,
            ),
            ProfileBaselineEvidence::Unestablished,
        );
    }

    #[test]
    fn authoritative_empty_identity_read_is_distinct_from_unknown() {
        assert_eq!(
            ProfileBaselineEvidence::from_published_identity(PersonalSourceState::Empty, None),
            ProfileBaselineEvidence::AuthoritativeEmpty,
        );
    }

    #[test]
    fn live_identity_with_keys_can_still_establish_empty_profile() {
        assert_eq!(
            ProfileBaselineEvidence::from_published_identity(PersonalSourceState::Live, None),
            ProfileBaselineEvidence::AuthoritativeEmpty,
        );
    }

    #[test]
    fn evidence_bearing_identity_read_preserves_exact_action() {
        assert_eq!(
            ProfileBaselineEvidence::from_published_identity(
                PersonalSourceState::Live,
                Some("uhCkk-profile-action".into()),
            ),
            ProfileBaselineEvidence::Action("uhCkk-profile-action".into()),
        );
    }

    #[test]
    fn failed_or_interrupted_source_never_upgrades_baseline() {
        for state in [
            PersonalSourceState::LoadingLive,
            PersonalSourceState::Degraded,
            PersonalSourceState::Unavailable,
        ] {
            assert_eq!(
                ProfileBaselineEvidence::from_published_identity(
                    state,
                    Some("uhCkk-ignored".into()),
                ),
                ProfileBaselineEvidence::Unestablished,
            );
        }
    }

    #[test]
    fn demo_never_manufactures_live_profile_authority() {
        assert_eq!(
            ProfileBaselineEvidence::from_published_identity(
                PersonalSourceState::Demo,
                Some("uhCkk-demo".into()),
            ),
            ProfileBaselineEvidence::Unestablished,
        );
    }

    #[test]
    fn only_authoritative_empty_maps_to_cas_none() {
        assert_eq!(
            ProfileBaselineEvidence::AuthoritativeEmpty.cas_expected_action_hash(),
            Some(None),
        );
        assert_eq!(
            ProfileBaselineEvidence::Unestablished.cas_expected_action_hash(),
            None,
        );
    }

    #[test]
    fn exact_action_maps_to_exact_cas_precondition() {
        let baseline = ProfileBaselineEvidence::Action("uhCkk-profile-action".into());
        assert_eq!(
            baseline.cas_expected_action_hash(),
            Some(Some("uhCkk-profile-action".into())),
        );
    }
}
