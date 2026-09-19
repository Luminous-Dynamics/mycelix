// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! App-lifetime evidence state for committed Personal mutations.
//!
//! Source-chain action receipts outlive page components and read-model epochs.
//! This module deliberately separates refresh progress from action observation:
//! a refresh can complete without publishing a replacement source snapshot, and
//! a published source snapshot can still fail to contain a particular action.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MutationRefreshOutcome {
    Published { epoch: u64 },
    SourceNotPublished { epoch: u64 },
    NoUsableEpoch,
    Busy { epoch: u64 },
    EpochChanged {
        started_epoch: u64,
        current_epoch: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PersonalMutationTarget {
    Profile,
    HealthConsent,
    Preference {
        source_cluster: String,
        target_cluster: String,
    },
}

impl PersonalMutationTarget {
    pub fn preference(source_cluster: impl Into<String>, target_cluster: impl Into<String>) -> Self {
        Self::Preference {
            source_cluster: source_cluster.into(),
            target_cluster: target_cluster.into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MutationObservationState {
    RefreshInProgress,
    RefreshDeferred(MutationRefreshOutcome),
    RefreshPublishedUncorrelated { epoch: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PendingMutationReceipt {
    pub target: PersonalMutationTarget,
    pub action_hash: String,
    pub observation: MutationObservationState,
}

impl PendingMutationReceipt {
    pub fn new(target: PersonalMutationTarget, action_hash: String) -> Self {
        Self {
            target,
            action_hash,
            observation: MutationObservationState::RefreshInProgress,
        }
    }

    pub fn with_refresh_outcome(mut self, outcome: MutationRefreshOutcome) -> Self {
        self.observation = match outcome {
            MutationRefreshOutcome::Published { epoch } => {
                MutationObservationState::RefreshPublishedUncorrelated { epoch }
            }
            other => MutationObservationState::RefreshDeferred(other),
        };
        self
    }

    pub fn title(&self) -> &'static str {
        match self.observation {
            MutationObservationState::RefreshInProgress => "Write committed; reconciling",
            MutationObservationState::RefreshDeferred(_) => {
                "Write committed; reconciliation pending"
            }
            MutationObservationState::RefreshPublishedUncorrelated { .. } => {
                "Write committed; observation unconfirmed"
            }
        }
    }

    pub fn description(&self) -> String {
        match self.observation {
            MutationObservationState::RefreshInProgress =>
                "The source-chain action receipt is established. Personal is attempting an epoch-safe read-model refresh before making any stronger presentation claim."
                    .into(),
            MutationObservationState::RefreshDeferred(MutationRefreshOutcome::SourceNotPublished { epoch }) =>
                format!(
                    "The write committed and its refresh completed in Personal epoch {epoch}, but the source query set did not produce an admissible replacement snapshot. The previous source values remain visible and this receipt remains pending."
                ),
            MutationObservationState::RefreshDeferred(MutationRefreshOutcome::NoUsableEpoch) =>
                "The write committed, but there is no usable conductor/signer epoch in which to establish a current Personal read-model refresh."
                    .into(),
            MutationObservationState::RefreshDeferred(MutationRefreshOutcome::Busy { epoch }) => {
                format!(
                    "The write committed, but Personal epoch {epoch} is already reconciling. This action has not been established in the displayed snapshot."
                )
            }
            MutationObservationState::RefreshDeferred(MutationRefreshOutcome::EpochChanged {
                started_epoch,
                current_epoch,
            }) => format!(
                "The write committed, but its refresh started in Personal epoch {started_epoch} and was rejected after the gate advanced to epoch {current_epoch}."
            ),
            MutationObservationState::RefreshDeferred(MutationRefreshOutcome::Published {
                epoch,
            })
            | MutationObservationState::RefreshPublishedUncorrelated { epoch } => format!(
                "A Personal source snapshot published in epoch {epoch}, but the atomically published read evidence did not contain this exact committed action hash. The receipt remains pending."
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn receipt_starts_as_committed_refresh_in_progress() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::Profile,
            "uhCkk-action".into(),
        );
        assert_eq!(receipt.action_hash, "uhCkk-action");
        assert_eq!(
            receipt.observation,
            MutationObservationState::RefreshInProgress
        );
    }

    #[test]
    fn published_refresh_remains_uncorrelated_until_action_is_observed() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::Profile,
            "uhCkk-action".into(),
        )
        .with_refresh_outcome(MutationRefreshOutcome::Published { epoch: 7 });

        assert_eq!(
            receipt.observation,
            MutationObservationState::RefreshPublishedUncorrelated { epoch: 7 }
        );
        assert!(receipt.description().contains("exact committed action hash"));
    }

    #[test]
    fn completed_refresh_without_source_publication_stays_deferred() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::Profile,
            "uhCkk-action".into(),
        )
        .with_refresh_outcome(MutationRefreshOutcome::SourceNotPublished { epoch: 7 });

        assert_eq!(
            receipt.observation,
            MutationObservationState::RefreshDeferred(
                MutationRefreshOutcome::SourceNotPublished { epoch: 7 }
            )
        );
        assert!(receipt.description().contains("did not produce"));
    }

    #[test]
    fn preference_target_is_pair_specific() {
        assert_ne!(
            PersonalMutationTarget::preference("health", "finance"),
            PersonalMutationTarget::preference("health", "knowledge")
        );
    }
}
