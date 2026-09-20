// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed admission for conditional Profile mutations.

use personal_leptos_types::{ConditionalProfileMutationInputView, ProfileView};

use crate::context::PersonalSourceState;
use crate::profile_baseline::ProfileBaselineEvidence;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileMutationAdmissionError {
    BaselineUnestablished,
}

/// Build a conditional Profile mutation only from an established, published
/// Identity baseline.
///
/// A successful authoritative-empty read is allowed to produce backend
/// `expected_action_hash = None`. Initial, demo, loading, degraded and
/// unavailable states cannot produce a wire payload at all.
pub fn admit_profile_mutation(
    identity_state: PersonalSourceState,
    profile_action_hash: Option<String>,
    profile: ProfileView,
) -> Result<ConditionalProfileMutationInputView, ProfileMutationAdmissionError> {
    let baseline =
        ProfileBaselineEvidence::from_published_identity(identity_state, profile_action_hash);
    let Some(expected_action_hash) = baseline.cas_expected_action_hash() else {
        return Err(ProfileMutationAdmissionError::BaselineUnestablished);
    };

    Ok(ConditionalProfileMutationInputView {
        expected_action_hash,
        profile,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> ProfileView {
        ProfileView {
            display_name: "Alice".into(),
            avatar: None,
            bio: None,
            metadata: Default::default(),
            updated_at: 1,
        }
    }

    #[test]
    fn initial_live_state_cannot_construct_conditional_write() {
        assert!(matches!(
            admit_profile_mutation(PersonalSourceState::AwaitingLive, None, profile()),
            Err(ProfileMutationAdmissionError::BaselineUnestablished),
        ));
    }

    #[test]
    fn degraded_identity_cannot_bless_retained_hash() {
        assert!(matches!(
            admit_profile_mutation(
                PersonalSourceState::Degraded,
                Some("uhCkk-retained".into()),
                profile(),
            ),
            Err(ProfileMutationAdmissionError::BaselineUnestablished),
        ));
    }

    #[test]
    fn demo_cannot_construct_live_conditional_write() {
        assert!(matches!(
            admit_profile_mutation(
                PersonalSourceState::Demo,
                Some("uhCkk-demo".into()),
                profile(),
            ),
            Err(ProfileMutationAdmissionError::BaselineUnestablished),
        ));
    }

    #[test]
    fn authoritative_empty_is_the_only_path_to_cas_none() {
        let input = admit_profile_mutation(PersonalSourceState::Empty, None, profile())
            .expect("authoritative empty baseline should admit a conditional create");
        assert_eq!(input.expected_action_hash, None);
    }

    #[test]
    fn live_empty_profile_with_other_identity_data_can_create() {
        let input = admit_profile_mutation(PersonalSourceState::Live, None, profile())
            .expect("successful Profile None may coexist with other live Identity data");
        assert_eq!(input.expected_action_hash, None);
    }

    #[test]
    fn exact_action_is_preserved_as_cas_precondition() {
        let input = admit_profile_mutation(
            PersonalSourceState::Live,
            Some("uhCkk-profile-action".into()),
            profile(),
        )
        .expect("evidence-bearing baseline should admit a conditional replacement");
        assert_eq!(
            input.expected_action_hash.as_deref(),
            Some("uhCkk-profile-action"),
        );
    }
}
