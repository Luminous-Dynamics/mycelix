// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed admission for conditional Preference mutations.
//!
//! The current Personal Preferences UI edits only source-backed existing rows.
//! Absence of an exact source-chain action therefore means the displayed pair
//! is not admitted for conditional replacement; it must not be reinterpreted as
//! authoritative empty/create semantics.

use personal_leptos_types::{
    ConditionalPreferenceMutationInputView, DataSharingPreferenceView,
};

use crate::mutation_state::PersonalMutationTarget;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreferenceMutationAdmissionError {
    /// The caller supplied a non-Preference semantic target to the Preference
    /// admission boundary. No wire payload is constructed.
    TargetKindMismatch,
    /// This existing-row edit has no exact action identity for its displayed
    /// source-backed baseline. No conditional wire payload may be constructed.
    BaselineActionUnestablished,
    /// The semantic mutation target and the payload describe different cluster
    /// pairs. An action identity from one pair must never authorize another.
    PairMismatch {
        target_source_cluster: String,
        target_target_cluster: String,
        payload_source_cluster: String,
        payload_target_cluster: String,
    },
}

/// Build a conditional replacement for one existing Preference row from its
/// exact displayed action identity.
///
/// This function intentionally does not support `expected_action_hash=None`.
/// The existing Preferences page is an edit surface for source-backed rows, not
/// a create-if-absent surface. A future explicit Preference creation flow needs
/// its own authoritative-empty admission theorem.
pub fn admit_existing_preference_mutation(
    target: PersonalMutationTarget,
    expected_action_hash: Option<String>,
    preference: DataSharingPreferenceView,
) -> Result<ConditionalPreferenceMutationInputView, PreferenceMutationAdmissionError> {
    let PersonalMutationTarget::Preference {
        source_cluster,
        target_cluster,
    } = target
    else {
        return Err(PreferenceMutationAdmissionError::TargetKindMismatch);
    };

    if source_cluster != preference.source_cluster || target_cluster != preference.target_cluster {
        return Err(PreferenceMutationAdmissionError::PairMismatch {
            target_source_cluster: source_cluster,
            target_target_cluster: target_cluster,
            payload_source_cluster: preference.source_cluster,
            payload_target_cluster: preference.target_cluster,
        });
    }

    let Some(expected_action_hash) = expected_action_hash else {
        return Err(PreferenceMutationAdmissionError::BaselineActionUnestablished);
    };

    Ok(ConditionalPreferenceMutationInputView {
        expected_action_hash: Some(expected_action_hash),
        preference,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn preference(source: &str, target: &str) -> DataSharingPreferenceView {
        DataSharingPreferenceView {
            source_cluster: source.into(),
            target_cluster: target.into(),
            allowed: true,
            blocked_zomes: Vec::new(),
            reason: String::new(),
            updated_at: 1,
        }
    }

    #[test]
    fn non_preference_target_fails_closed_without_panicking() {
        assert_eq!(
            admit_existing_preference_mutation(
                PersonalMutationTarget::Profile,
                Some("uhCkk-action".into()),
                preference("health", "finance"),
            ),
            Err(PreferenceMutationAdmissionError::TargetKindMismatch),
        );
    }

    #[test]
    fn missing_existing_row_action_cannot_be_promoted_to_authoritative_empty() {
        let target = PersonalMutationTarget::preference("health", "finance");
        assert_eq!(
            admit_existing_preference_mutation(
                target,
                None,
                preference("health", "finance"),
            ),
            Err(PreferenceMutationAdmissionError::BaselineActionUnestablished),
        );
    }

    #[test]
    fn exact_pair_and_action_admit_conditional_replacement() {
        let target = PersonalMutationTarget::preference("health", "finance");
        let input = admit_existing_preference_mutation(
            target,
            Some("uhCkk-pref-action".into()),
            preference("health", "finance"),
        )
        .expect("source-backed exact pair should admit");

        assert_eq!(input.expected_action_hash.as_deref(), Some("uhCkk-pref-action"));
        assert_eq!(input.preference.source_cluster, "health");
        assert_eq!(input.preference.target_cluster, "finance");
    }

    #[test]
    fn action_for_pair_a_cannot_authorize_payload_for_pair_b() {
        let target = PersonalMutationTarget::preference("health", "finance");
        let error = admit_existing_preference_mutation(
            target,
            Some("uhCkk-pair-a".into()),
            preference("health", "knowledge"),
        )
        .expect_err("pair substitution must fail closed");

        assert_eq!(
            error,
            PreferenceMutationAdmissionError::PairMismatch {
                target_source_cluster: "health".into(),
                target_target_cluster: "finance".into(),
                payload_source_cluster: "health".into(),
                payload_target_cluster: "knowledge".into(),
            },
        );
    }
}
