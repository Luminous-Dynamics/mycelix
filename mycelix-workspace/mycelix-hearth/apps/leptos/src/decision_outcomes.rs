// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed DecisionOutcome loading and disclosure.
//!
//! Decision status and DecisionOutcome are separate facts. A `Finalized`
//! Decision never gets a fabricated winner from current vote bars: this module
//! queries the immutable outcome record explicitly and degrades semantic
//! inconsistencies instead of strengthening them into a successful result.

use crate::hearth_context::use_hearth;
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::WireRecord;
use hearth_leptos_types::{DecisionStatus, DecisionView};
use leptos::prelude::*;
use mycelix_leptos_client::{HoloHashBytes, HoloHashKind};
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use serde::Deserialize;
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionOutcomeView {
    pub record_hash: String,
    pub decision_hash: String,
    pub chosen_option: u32,
    pub participation_rate_bp: u32,
    pub resolved_at: i64,
    pub quorum_bp: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionOutcomeLookup {
    pub decision_hash: String,
    pub state: AvailabilityStateKind,
    pub outcome: Option<DecisionOutcomeView>,
}

#[derive(Clone)]
pub struct DecisionOutcomeState {
    pub availability: RwSignal<AvailabilityStateKind>,
    pub lookups: RwSignal<Vec<DecisionOutcomeLookup>>,
    pub loading: RwSignal<bool>,
}

#[derive(Debug, Clone, Deserialize)]
struct WireDecisionOutcome {
    decision_hash: Vec<u8>,
    chosen_option: u32,
    participation_rate_bp: u32,
    resolved_at: i64,
    #[serde(default)]
    quorum_bp: Option<u32>,
}

fn record_to_outcome(record: &WireRecord) -> Option<DecisionOutcomeView> {
    let outcome: WireDecisionOutcome = record.decode_entry()?;
    let decision_hash = HoloHashBytes::from_raw_39(outcome.decision_hash)
        .ok()?
        .require_kind(HoloHashKind::Action)
        .ok()?
        .to_raw_base64();

    Some(DecisionOutcomeView {
        record_hash: record.action_hash_b64(),
        decision_hash,
        chosen_option: outcome.chosen_option,
        participation_rate_bp: outcome.participation_rate_bp,
        resolved_at: outcome.resolved_at / 1_000_000,
        quorum_bp: outcome.quorum_bp,
    })
}

fn semantic_outcome_state(
    decision: &DecisionView,
    outcome: Option<&DecisionOutcomeView>,
) -> AvailabilityStateKind {
    match outcome {
        Some(outcome)
            if outcome.decision_hash != decision.hash
                || outcome.chosen_option as usize >= decision.options.len() =>
        {
            AvailabilityStateKind::Degraded
        }
        Some(_) if decision.status == DecisionStatus::Open => AvailabilityStateKind::Degraded,
        Some(_) => AvailabilityStateKind::Live,
        None if decision.status == DecisionStatus::Finalized => AvailabilityStateKind::Degraded,
        None => AvailabilityStateKind::Empty,
    }
}

fn summarize_availability(
    decision_state: AvailabilityStateKind,
    lookups: &[DecisionOutcomeLookup],
) -> AvailabilityStateKind {
    if decision_state == AvailabilityStateKind::Degraded
        || lookups
            .iter()
            .any(|lookup| lookup.state == AvailabilityStateKind::Degraded)
    {
        return AvailabilityStateKind::Degraded;
    }
    if lookups.is_empty() {
        return AvailabilityStateKind::Empty;
    }

    let unavailable = lookups
        .iter()
        .filter(|lookup| lookup.state == AvailabilityStateKind::Unavailable)
        .count();
    if unavailable == lookups.len() {
        return AvailabilityStateKind::Unavailable;
    }
    if unavailable > 0 {
        return AvailabilityStateKind::Degraded;
    }
    if lookups
        .iter()
        .any(|lookup| lookup.state == AvailabilityStateKind::Live)
    {
        AvailabilityStateKind::Live
    } else {
        AvailabilityStateKind::Empty
    }
}

fn decision_snapshot_key(decisions: &[DecisionView]) -> Vec<(String, u8)> {
    decisions
        .iter()
        .map(|decision| {
            let status = match &decision.status {
                DecisionStatus::Open => 0,
                DecisionStatus::Closed => 1,
                DecisionStatus::Finalized => 2,
            };
            (decision.hash.clone(), status)
        })
        .collect()
}

fn invalidate_load(generation: &Rc<Cell<u64>>) {
    generation.set(generation.get().wrapping_add(1));
}

pub fn provide_decision_outcomes() -> DecisionOutcomeState {
    let state = DecisionOutcomeState {
        availability: RwSignal::new(AvailabilityStateKind::Mock),
        lookups: RwSignal::new(Vec::new()),
        loading: RwSignal::new(false),
    };
    provide_context(state.clone());

    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let hc = use_holochain();
    let last_key = Rc::new(RefCell::new(None::<Vec<(String, u8)>>));
    let generation = Rc::new(Cell::new(0u64));

    let state_effect = state.clone();
    let hearth_effect = hearth.clone();
    let truth_effect = truth.clone();
    let hc_effect = hc.clone();
    let key_effect = last_key.clone();
    let generation_effect = generation.clone();

    Effect::new(move |_| {
        let status = hc_effect.status.get();
        let signer_ready = hc_effect.zome_call_signing_ready.get();
        let snapshot_loading = truth_effect.loading.get();
        let decision_state = truth_effect.availability.get().decisions;

        match status {
            ConnectionStatus::Mock => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect.availability.set(AvailabilityStateKind::Mock);
            }
            ConnectionStatus::Connected if snapshot_loading => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect.availability.set(AvailabilityStateKind::Unknown);
            }
            ConnectionStatus::Connected if !signer_ready => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect
                    .availability
                    .set(AvailabilityStateKind::Unavailable);
            }
            ConnectionStatus::Connected => {
                if decision_state == AvailabilityStateKind::Empty {
                    invalidate_load(&generation_effect);
                    *key_effect.borrow_mut() = Some(Vec::new());
                    state_effect.loading.set(false);
                    state_effect.lookups.set(Vec::new());
                    state_effect.availability.set(AvailabilityStateKind::Empty);
                    return;
                }
                if !matches!(
                    decision_state,
                    AvailabilityStateKind::Live | AvailabilityStateKind::Degraded
                ) {
                    invalidate_load(&generation_effect);
                    *key_effect.borrow_mut() = None;
                    state_effect.loading.set(false);
                    state_effect.lookups.set(Vec::new());
                    state_effect.availability.set(decision_state.clone());
                    return;
                }

                let decisions = hearth_effect.decisions.get();
                let key = decision_snapshot_key(&decisions);
                if key_effect.borrow().as_ref() == Some(&key) {
                    return;
                }
                *key_effect.borrow_mut() = Some(key);
                start_outcome_load(
                    state_effect.clone(),
                    hc_effect.clone(),
                    decisions,
                    decision_state,
                    generation_effect.clone(),
                );
            }
            ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.update(|lookups| {
                    for lookup in lookups {
                        if matches!(
                            lookup.state,
                            AvailabilityStateKind::Live | AvailabilityStateKind::Empty
                        ) {
                            lookup.state = AvailabilityStateKind::Degraded;
                        }
                    }
                });
                if matches!(
                    state_effect.availability.get_untracked(),
                    AvailabilityStateKind::Live | AvailabilityStateKind::Empty
                ) {
                    state_effect
                        .availability
                        .set(AvailabilityStateKind::Degraded);
                }
            }
            ConnectionStatus::Connecting => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.availability.set(AvailabilityStateKind::Unknown);
            }
        }
    });

    state
}

pub fn use_decision_outcomes() -> DecisionOutcomeState {
    expect_context::<DecisionOutcomeState>()
}

fn start_outcome_load(
    state: DecisionOutcomeState,
    hc: mycelix_leptos_core::HolochainCtx,
    decisions: Vec<DecisionView>,
    decision_state: AvailabilityStateKind,
    generation: Rc<Cell<u64>>,
) {
    invalidate_load(&generation);
    let token = generation.get();
    state.loading.set(true);
    state.lookups.set(Vec::new());
    state.availability.set(AvailabilityStateKind::Unknown);

    spawn_local(async move {
        let mut lookups = Vec::with_capacity(decisions.len());

        for decision in &decisions {
            let decision_hash = match HoloHashBytes::from_action_raw_base64(&decision.hash) {
                Ok(hash) => hash,
                Err(error) => {
                    web_sys::console::log_1(
                        &format!(
                            "[Hearth] outcome query blocked by invalid Decision ActionHash {}: {error}",
                            decision.hash
                        )
                        .into(),
                    );
                    lookups.push(DecisionOutcomeLookup {
                        decision_hash: decision.hash.clone(),
                        state: AvailabilityStateKind::Degraded,
                        outcome: None,
                    });
                    continue;
                }
            };

            match hc
                .call_zome_default::<HoloHashBytes, Option<WireRecord>>(
                    "hearth_decisions",
                    "get_decision_outcome",
                    &decision_hash,
                )
                .await
            {
                Ok(Some(record)) => {
                    let outcome = record_to_outcome(&record);
                    let state = semantic_outcome_state(decision, outcome.as_ref());
                    lookups.push(DecisionOutcomeLookup {
                        decision_hash: decision.hash.clone(),
                        state,
                        outcome,
                    });
                }
                Ok(None) => lookups.push(DecisionOutcomeLookup {
                    decision_hash: decision.hash.clone(),
                    state: semantic_outcome_state(decision, None),
                    outcome: None,
                }),
                Err(error) => {
                    web_sys::console::log_1(
                        &format!(
                            "[Hearth] get_decision_outcome failed for {}: {error}",
                            decision.hash
                        )
                        .into(),
                    );
                    lookups.push(DecisionOutcomeLookup {
                        decision_hash: decision.hash.clone(),
                        state: AvailabilityStateKind::Unavailable,
                        outcome: None,
                    });
                }
            }
        }

        if generation.get() != token {
            return;
        }

        let availability = summarize_availability(decision_state, &lookups);
        state.lookups.set(lookups);
        state.availability.set(availability);
        state.loading.set(false);
    });
}

fn quorum_snapshot_label(quorum_bp: Option<u32>) -> String {
    match quorum_bp {
        Some(bp) => format!("Recorded quorum: {:.2}%", bp as f64 / 100.0),
        None => "Recorded quorum: none".to_string(),
    }
}

#[component]
pub fn DecisionOutcomeSummary() -> impl IntoView {
    let hearth = use_hearth();
    let state = use_decision_outcomes();

    let hearth_view = hearth.clone();
    let state_view = state.clone();
    let state_status = state.clone();

    view! {
        <section class="decision-outcome-summary" aria-labelledby="decision-outcome-heading">
            <h2 id="decision-outcome-heading">"Recorded outcomes"</h2>
            <p class="decision-outcome-source" role="status">
                {move || format!(
                    "DecisionOutcome source: {}{}",
                    state_status.availability.get().label(),
                    if state_status.loading.get() { " (loading)" } else { "" },
                )}
            </p>
            {move || {
                let decisions = hearth_view.decisions.get();
                let lookups = state_view.lookups.get();
                let resolved = decisions
                    .iter()
                    .filter(|decision| decision.status != DecisionStatus::Open)
                    .cloned()
                    .collect::<Vec<_>>();

                if resolved.is_empty() {
                    return view! {
                        <p class="decision-outcome-empty">
                            "No closed or finalized decisions are present in this snapshot."
                        </p>
                    }
                    .into_any();
                }

                view! {
                    <div class="decision-outcome-list">
                        {resolved.into_iter().map(|decision| {
                            let lookup = lookups
                                .iter()
                                .find(|lookup| lookup.decision_hash == decision.hash)
                                .cloned();
                            let title = decision.title.clone();
                            let status = decision.status.clone();

                            match lookup {
                                Some(DecisionOutcomeLookup {
                                    state: AvailabilityStateKind::Live,
                                    outcome: Some(outcome),
                                    ..
                                }) => {
                                    let chosen = decision.options
                                        .get(outcome.chosen_option as usize)
                                        .cloned()
                                        .unwrap_or_else(|| "Invalid option index".to_string());
                                    let participation = outcome.participation_rate_bp as f64 / 100.0;
                                    let quorum = quorum_snapshot_label(outcome.quorum_bp);
                                    view! {
                                        <article class="decision-outcome-card outcome-live">
                                            <h3>{title}</h3>
                                            <p><strong>"Recorded outcome: "</strong>{chosen}</p>
                                            <p>{format!("Recorded participation: {participation:.2}%")}</p>
                                            <p>{quorum}</p>
                                            <p class="decision-outcome-provenance">
                                                "Source: immutable DecisionOutcome record returned by hearth_decisions."
                                            </p>
                                        </article>
                                    }.into_any()
                                }
                                Some(DecisionOutcomeLookup {
                                    state: AvailabilityStateKind::Empty,
                                    ..
                                }) => view! {
                                    <article class="decision-outcome-card outcome-empty">
                                        <h3>{title}</h3>
                                        <p>{if status == DecisionStatus::Closed {
                                            "No DecisionOutcome record was returned. A closed decision can legitimately have no outcome when it was closed without votes."
                                        } else {
                                            "No DecisionOutcome record was returned."
                                        }}</p>
                                    </article>
                                }.into_any(),
                                Some(DecisionOutcomeLookup {
                                    state: AvailabilityStateKind::Degraded,
                                    outcome,
                                    ..
                                }) => {
                                    let retained = outcome
                                        .and_then(|outcome| decision.options
                                            .get(outcome.chosen_option as usize)
                                            .cloned())
                                        .map(|chosen| format!(
                                            " A candidate record named ‘{chosen}’, but it is not promoted to authoritative display."
                                        ))
                                        .unwrap_or_default();
                                    view! {
                                        <article class="decision-outcome-card outcome-degraded" role="alert">
                                            <h3>{title}</h3>
                                            <p>
                                                "The DecisionOutcome source is inconsistent or only partially established. Refresh/reconcile before relying on a final result."
                                                {retained}
                                            </p>
                                        </article>
                                    }.into_any()
                                }
                                Some(DecisionOutcomeLookup {
                                    state: AvailabilityStateKind::Unavailable,
                                    ..
                                }) => view! {
                                    <article class="decision-outcome-card outcome-unavailable" role="alert">
                                        <h3>{title}</h3>
                                        <p>
                                            "The authoritative DecisionOutcome query is unavailable in this snapshot. No winner is inferred from current vote bars."
                                        </p>
                                    </article>
                                }.into_any(),
                                _ => view! {
                                    <article class="decision-outcome-card outcome-unknown">
                                        <h3>{title}</h3>
                                        <p>"The DecisionOutcome source has not been established yet."</p>
                                    </article>
                                }.into_any(),
                            }
                        }).collect_view()}
                    </div>
                }.into_any()
            }}
        </section>
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DecisionOutcomeLookup, DecisionOutcomeView, semantic_outcome_state,
        summarize_availability,
    };
    use hearth_leptos_types::{DecisionStatus, DecisionType, DecisionView, MemberRole};
    use mycelix_leptos_core::AvailabilityStateKind;

    fn decision(status: DecisionStatus) -> DecisionView {
        DecisionView {
            hash: "decision-hash".into(),
            hearth_hash: "hearth-hash".into(),
            title: "Test".into(),
            description: String::new(),
            decision_type: DecisionType::MajorityVote,
            eligible_roles: vec![MemberRole::Adult],
            options: vec!["A".into(), "B".into()],
            deadline: 100,
            quorum_bp: None,
            status,
            created_by: "agent".into(),
            created_at: 0,
        }
    }

    fn outcome(choice: u32) -> DecisionOutcomeView {
        DecisionOutcomeView {
            record_hash: "outcome-record".into(),
            decision_hash: "decision-hash".into(),
            chosen_option: choice,
            participation_rate_bp: 5_000,
            resolved_at: 200,
            quorum_bp: None,
        }
    }

    #[test]
    fn finalized_without_outcome_is_degraded_not_empty() {
        assert_eq!(
            semantic_outcome_state(&decision(DecisionStatus::Finalized), None),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn closed_without_outcome_can_be_empty() {
        assert_eq!(
            semantic_outcome_state(&decision(DecisionStatus::Closed), None),
            AvailabilityStateKind::Empty
        );
    }

    #[test]
    fn open_with_outcome_is_semantically_degraded() {
        let outcome = outcome(0);
        assert_eq!(
            semantic_outcome_state(&decision(DecisionStatus::Open), Some(&outcome)),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn invalid_chosen_option_is_degraded() {
        let outcome = outcome(2);
        assert_eq!(
            semantic_outcome_state(&decision(DecisionStatus::Finalized), Some(&outcome)),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn mixed_unavailable_and_live_is_globally_degraded() {
        let lookups = vec![
            DecisionOutcomeLookup {
                decision_hash: "a".into(),
                state: AvailabilityStateKind::Live,
                outcome: None,
            },
            DecisionOutcomeLookup {
                decision_hash: "b".into(),
                state: AvailabilityStateKind::Unavailable,
                outcome: None,
            },
        ];
        assert_eq!(
            summarize_availability(AvailabilityStateKind::Live, &lookups),
            AvailabilityStateKind::Degraded
        );
    }
}
