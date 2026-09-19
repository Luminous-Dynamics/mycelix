// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Presentation for committed Personal mutations whose read-model refresh has
//! not been established as published in the current reconciliation epoch.

use leptos::prelude::*;
use mycelix_leptos_core::{AvailabilityState, AvailabilityStateKind};

use crate::mutation_refresh::MutationRefreshOutcome;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PendingMutationReceipt {
    pub action_hash: String,
    pub refresh_outcome: MutationRefreshOutcome,
}

impl PendingMutationReceipt {
    pub fn new(action_hash: String, refresh_outcome: MutationRefreshOutcome) -> Option<Self> {
        if matches!(refresh_outcome, MutationRefreshOutcome::Published { .. }) {
            None
        } else {
            Some(Self {
                action_hash,
                refresh_outcome,
            })
        }
    }

    pub fn description(&self) -> String {
        match self.refresh_outcome {
            MutationRefreshOutcome::Published { epoch } => format!(
                "The write receipt exists and its source refresh published in Personal epoch {epoch}."
            ),
            MutationRefreshOutcome::NoUsableEpoch =>
                "The write committed, but there is no usable conductor/signer epoch in which to establish a current Personal read-model refresh."
                    .into(),
            MutationRefreshOutcome::Busy { epoch } => format!(
                "The write committed, but Personal epoch {epoch} is already reconciling. This action has not been established in the displayed snapshot."
            ),
            MutationRefreshOutcome::EpochChanged {
                started_epoch,
                current_epoch,
            } => format!(
                "The write committed, but its refresh started in Personal epoch {started_epoch} and was rejected after the gate advanced to epoch {current_epoch}."
            ),
        }
    }
}

#[component]
pub fn MutationPendingNotice(
    pending: RwSignal<Option<PendingMutationReceipt>>,
) -> impl IntoView {
    view! {
        {move || pending.get().map(|receipt| {
            let description = receipt.description();
            let action_hash = receipt.action_hash;
            view! {
                <AvailabilityState
                    kind=AvailabilityStateKind::Degraded
                    title="Write committed; reconciliation pending"
                    description=description
                    action={Some(view! {
                        <code class="hash-line">{action_hash}</code>
                    }.into_any())}
                />
            }.into_any()
        }).unwrap_or_else(|| view! { <></> }.into_any())}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn published_refresh_does_not_create_pending_receipt() {
        assert!(PendingMutationReceipt::new(
            "uhCkk-action".into(),
            MutationRefreshOutcome::Published { epoch: 7 },
        )
        .is_none());
    }

    #[test]
    fn epoch_change_keeps_committed_action_identity_visible() {
        let pending = PendingMutationReceipt::new(
            "uhCkk-action".into(),
            MutationRefreshOutcome::EpochChanged {
                started_epoch: 4,
                current_epoch: 5,
            },
        )
        .expect("non-published refresh must remain pending");

        assert_eq!(pending.action_hash, "uhCkk-action");
        assert!(pending.description().contains("epoch 4"));
        assert!(pending.description().contains("epoch 5"));
    }

    #[test]
    fn busy_refresh_does_not_upgrade_receipt_to_current_snapshot() {
        let pending = PendingMutationReceipt::new(
            "uhCkk-action".into(),
            MutationRefreshOutcome::Busy { epoch: 3 },
        )
        .expect("busy refresh must remain pending");

        assert!(pending.description().contains("already reconciling"));
        assert!(pending.description().contains("not been established"));
    }
}
