// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Mutation-triggered Personal refresh admission.
//!
//! A successful source-chain write, a completed refresh attempt, and a newly
//! published Personal source snapshot are separate facts. This module serializes
//! explicit post-mutation refreshes against the connection-epoch reconciliation
//! gate and reports which of those facts was actually established.

use leptos::prelude::*;
use mycelix_leptos_core::holochain_provider::HolochainCtx;

use crate::context::{
    refresh_health_state, refresh_identity_state, refresh_preferences_state, PersonalCtx,
    PersonalSourceState,
};
use crate::mutation_state::MutationRefreshOutcome;
use crate::reconciliation::ReconciliationEpoch;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MutationRefreshTarget {
    Identity,
    Health,
    Preferences,
}

fn begin_mutation_refresh(
    gate: &mut ReconciliationEpoch,
) -> Result<u64, MutationRefreshOutcome> {
    if !gate.is_usable() {
        return Err(MutationRefreshOutcome::NoUsableEpoch);
    }

    let epoch = gate.current_epoch();
    if gate.is_in_flight() {
        return Err(MutationRefreshOutcome::Busy { epoch });
    }

    let started_epoch = gate
        .begin_refresh()
        .expect("usable non-busy Personal epoch must admit one explicit refresh");
    debug_assert_eq!(started_epoch, epoch);
    Ok(started_epoch)
}

fn source_state_has_published_snapshot(state: PersonalSourceState) -> bool {
    matches!(state, PersonalSourceState::Live | PersonalSourceState::Empty)
}

fn finish_mutation_refresh(
    gate: &mut ReconciliationEpoch,
    started_epoch: u64,
    source_published: bool,
) -> MutationRefreshOutcome {
    if !gate.accepts(started_epoch) {
        return MutationRefreshOutcome::EpochChanged {
            started_epoch,
            current_epoch: gate.current_epoch(),
        };
    }

    if !gate.finish(started_epoch) {
        return MutationRefreshOutcome::EpochChanged {
            started_epoch,
            current_epoch: gate.current_epoch(),
        };
    }

    if source_published {
        MutationRefreshOutcome::Published {
            epoch: started_epoch,
        }
    } else {
        MutationRefreshOutcome::SourceNotPublished {
            epoch: started_epoch,
        }
    }
}

async fn run_mutation_refresh(
    ctx: PersonalCtx,
    hc: HolochainCtx,
    target: MutationRefreshTarget,
) -> MutationRefreshOutcome {
    let mut gate = ctx.reconciliation.get_untracked();
    let started_epoch = match begin_mutation_refresh(&mut gate) {
        Ok(epoch) => epoch,
        Err(outcome) => return outcome,
    };
    ctx.reconciliation.set(gate);

    let source_published = match target {
        MutationRefreshTarget::Identity => {
            refresh_identity_state(ctx.clone(), hc).await;
            source_state_has_published_snapshot(ctx.identity_state.get_untracked())
        }
        MutationRefreshTarget::Health => {
            refresh_health_state(ctx.clone(), hc).await;
            source_state_has_published_snapshot(ctx.health_state.get_untracked())
        }
        MutationRefreshTarget::Preferences => {
            refresh_preferences_state(ctx.clone(), hc).await;
            source_state_has_published_snapshot(ctx.preferences_state.get_untracked())
        }
    };

    let mut current_gate = ctx.reconciliation.get_untracked();
    let outcome = finish_mutation_refresh(&mut current_gate, started_epoch, source_published);
    if matches!(
        outcome,
        MutationRefreshOutcome::Published { .. }
            | MutationRefreshOutcome::SourceNotPublished { .. }
    ) {
        ctx.reconciliation.set(current_gate);
    }
    outcome
}

pub async fn refresh_identity_after_mutation(
    ctx: PersonalCtx,
    hc: HolochainCtx,
) -> MutationRefreshOutcome {
    run_mutation_refresh(ctx, hc, MutationRefreshTarget::Identity).await
}

pub async fn refresh_health_after_mutation(
    ctx: PersonalCtx,
    hc: HolochainCtx,
) -> MutationRefreshOutcome {
    run_mutation_refresh(ctx, hc, MutationRefreshTarget::Health).await
}

pub async fn refresh_preferences_after_mutation(
    ctx: PersonalCtx,
    hc: HolochainCtx,
) -> MutationRefreshOutcome {
    run_mutation_refresh(ctx, hc, MutationRefreshTarget::Preferences).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reconciliation::ReconciliationTransition;

    #[test]
    fn refresh_without_usable_epoch_is_rejected() {
        let mut gate = ReconciliationEpoch::default();
        assert_eq!(
            begin_mutation_refresh(&mut gate),
            Err(MutationRefreshOutcome::NoUsableEpoch)
        );
    }

    #[test]
    fn refresh_coalesces_while_initial_hydration_is_in_flight() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("usable session must start initial reconciliation");
        };

        assert_eq!(
            begin_mutation_refresh(&mut gate),
            Err(MutationRefreshOutcome::Busy { epoch })
        );
    }

    #[test]
    fn completed_epoch_reports_published_only_for_a_new_source_snapshot() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("usable session must start initial reconciliation");
        };
        assert!(gate.finish(epoch));

        let started = begin_mutation_refresh(&mut gate).expect("refresh should be admitted");
        assert_eq!(started, epoch);
        assert!(gate.is_in_flight());
        assert_eq!(
            finish_mutation_refresh(&mut gate, started, true),
            MutationRefreshOutcome::Published { epoch }
        );
        assert!(!gate.is_in_flight());
    }

    #[test]
    fn completed_refresh_without_source_commit_is_not_published() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("usable session must start initial reconciliation");
        };
        assert!(gate.finish(epoch));

        let started = begin_mutation_refresh(&mut gate).expect("refresh should be admitted");
        assert_eq!(
            finish_mutation_refresh(&mut gate, started, false),
            MutationRefreshOutcome::SourceNotPublished { epoch }
        );
        assert!(!gate.is_in_flight());
    }

    #[test]
    fn degraded_and_unavailable_states_are_not_publication_evidence() {
        assert!(!source_state_has_published_snapshot(
            PersonalSourceState::Degraded
        ));
        assert!(!source_state_has_published_snapshot(
            PersonalSourceState::Unavailable
        ));
        assert!(source_state_has_published_snapshot(PersonalSourceState::Live));
        assert!(source_state_has_published_snapshot(PersonalSourceState::Empty));
    }

    #[test]
    fn epoch_change_rejects_old_refresh_publication() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("usable session must start initial reconciliation");
        };
        assert!(gate.finish(epoch));
        let started = begin_mutation_refresh(&mut gate).expect("refresh should be admitted");

        let ReconciliationTransition::Invalidated {
            epoch: current_epoch,
            ..
        } = gate.observe_usable(false)
        else {
            panic!("disconnect must invalidate the admitted refresh");
        };

        assert_eq!(
            finish_mutation_refresh(&mut gate, started, true),
            MutationRefreshOutcome::EpochChanged {
                started_epoch: started,
                current_epoch,
            }
        );
    }
}
