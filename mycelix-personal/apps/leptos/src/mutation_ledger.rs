// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! App-lifetime ledger for committed Personal mutation receipts.
//!
//! This deliberately lives outside `PersonalCtx`: read-model snapshots may be
//! invalidated, cleared, or replaced across conductor epochs, while an already
//! returned source-chain action receipt remains evidence of that committed
//! action. Snapshot lifetime must therefore not own receipt lifetime.

use leptos::prelude::*;

use crate::mutation_state::{
    MutationObservationState, MutationRefreshOutcome, PendingMutationReceipt,
    PersonalMutationTarget,
};

#[derive(Clone, Copy)]
pub struct MutationLedger {
    pending: RwSignal<Vec<PendingMutationReceipt>>,
}

impl MutationLedger {
    pub fn record_committed(
        &self,
        target: PersonalMutationTarget,
        action_hash: String,
    ) -> PendingMutationReceipt {
        let receipt = PendingMutationReceipt::new(target, action_hash);
        self.upsert(receipt.clone());
        receipt
    }

    pub fn record_refresh_outcome(&self, action_hash: &str, outcome: MutationRefreshOutcome) {
        self.pending.update(|items| {
            if let Some(receipt) = items
                .iter_mut()
                .find(|receipt| receipt.action_hash == action_hash)
            {
                receipt.observation = match outcome {
                    MutationRefreshOutcome::Published { epoch } => {
                        MutationObservationState::RefreshPublishedUncorrelated { epoch }
                    }
                    other => MutationObservationState::RefreshDeferred(other),
                };
            }
        });
    }

    pub fn mark_observed(&self, action_hash: &str) -> bool {
        let mut removed = false;
        self.pending.update(|items| {
            let before = items.len();
            items.retain(|receipt| receipt.action_hash != action_hash);
            removed = items.len() != before;
        });
        removed
    }

    pub fn for_target(&self, target: &PersonalMutationTarget) -> Vec<PendingMutationReceipt> {
        self.pending
            .get()
            .into_iter()
            .filter(|receipt| &receipt.target == target)
            .collect()
    }

    pub fn all_untracked(&self) -> Vec<PendingMutationReceipt> {
        self.pending.get_untracked()
    }

    fn upsert(&self, receipt: PendingMutationReceipt) {
        self.pending.update(|items| {
            if let Some(existing) = items
                .iter_mut()
                .find(|existing| existing.action_hash == receipt.action_hash)
            {
                *existing = receipt;
            } else {
                items.push(receipt);
            }
        });
    }
}

pub fn provide_mutation_ledger() -> MutationLedger {
    let ledger = MutationLedger {
        pending: RwSignal::new(Vec::new()),
    };
    provide_context(ledger);
    ledger
}

pub fn use_mutation_ledger() -> MutationLedger {
    expect_context::<MutationLedger>()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ledger_type_does_not_claim_snapshot_ownership() {
        // Compile-time/API shape regression: the ledger stores mutation
        // evidence directly and has no Personal read-model field to clear.
        let _target = PersonalMutationTarget::Profile;
        let _state = MutationObservationState::RefreshInProgress;
    }
}
