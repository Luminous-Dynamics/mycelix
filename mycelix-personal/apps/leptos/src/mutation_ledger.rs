// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! App-lifetime ledger for committed Personal mutation receipts.
//!
//! This deliberately lives outside `PersonalCtx`: read-model snapshots may be
//! invalidated, cleared, or replaced across conductor epochs, while an already
//! returned source-chain action receipt remains evidence of that committed
//! action. Snapshot lifetime must therefore not own receipt lifetime.

use std::collections::HashSet;

use leptos::prelude::*;

use crate::context::use_personal;
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

fn receipt_is_observed(
    receipt: &PendingMutationReceipt,
    profile_action_hash: Option<&str>,
    preference_action_hashes: &HashSet<String>,
    consent_action_hashes: &HashSet<String>,
) -> bool {
    match &receipt.target {
        PersonalMutationTarget::Profile => {
            profile_action_hash == Some(receipt.action_hash.as_str())
        }
        PersonalMutationTarget::HealthConsent => {
            consent_action_hashes.contains(&receipt.action_hash)
        }
        PersonalMutationTarget::Preference { .. } => {
            preference_action_hashes.contains(&receipt.action_hash)
        }
    }
}

pub fn provide_mutation_ledger() -> MutationLedger {
    let ledger = MutationLedger {
        pending: RwSignal::new(Vec::new()),
    };
    provide_context(ledger);

    // Read-model observation is derived from action identity, never payload
    // equality. These evidence signals are committed in the same source-level
    // batch as the values they describe, so an epoch-invalidated staging run
    // cannot accidentally retire a receipt.
    let ctx = use_personal();
    let ledger_for_effect = ledger;
    Effect::new(move |_| {
        let profile_action_hash = ctx.profile_action_hash.get();
        let preference_action_hashes = ctx.preference_action_hashes.get();
        let consent_action_hashes = ctx
            .consents
            .get()
            .into_iter()
            .map(|consent| consent.hash)
            .collect::<HashSet<_>>();

        ledger_for_effect.pending.update(|items| {
            items.retain(|receipt| {
                !receipt_is_observed(
                    receipt,
                    profile_action_hash.as_deref(),
                    &preference_action_hashes,
                    &consent_action_hashes,
                )
            });
        });
    });

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

    #[test]
    fn profile_observation_requires_exact_action_hash() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::Profile,
            "uhCkk-profile-new".into(),
        );
        assert!(!receipt_is_observed(
            &receipt,
            Some("uhCkk-profile-old"),
            &HashSet::new(),
            &HashSet::new(),
        ));
        assert!(receipt_is_observed(
            &receipt,
            Some("uhCkk-profile-new"),
            &HashSet::new(),
            &HashSet::new(),
        ));
    }

    #[test]
    fn preference_observation_requires_exact_evidence_membership() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::preference("personal", "health"),
            "uhCkk-preference-new".into(),
        );
        let mut evidence = HashSet::new();
        evidence.insert("uhCkk-preference-old".into());
        assert!(!receipt_is_observed(
            &receipt,
            None,
            &evidence,
            &HashSet::new(),
        ));
        evidence.insert("uhCkk-preference-new".into());
        assert!(receipt_is_observed(
            &receipt,
            None,
            &evidence,
            &HashSet::new(),
        ));
    }

    #[test]
    fn consent_observation_uses_the_same_exact_hash_rule() {
        let receipt = PendingMutationReceipt::new(
            PersonalMutationTarget::HealthConsent,
            "uhCkk-consent".into(),
        );
        let evidence = HashSet::from(["uhCkk-consent".to_string()]);
        assert!(receipt_is_observed(
            &receipt,
            None,
            &HashSet::new(),
            &evidence,
        ));
    }
}
