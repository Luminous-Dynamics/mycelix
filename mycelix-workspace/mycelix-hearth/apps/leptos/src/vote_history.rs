// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Immutable vote-history provenance for Hearth Decisions.
//!
//! Current vote links answer what contributes to the current tally. The
//! `DecisionToVoteHistory` chain answers which vote records were written over
//! time, including records superseded by amendments. These are separate facts:
//! this module preserves source order, validates each returned record, and
//! reports current-vote alignment independently from history availability.

use crate::hearth_context::{member_name, use_hearth};
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord, WireVote};
use hearth_leptos_types::{DecisionView, VoteView};
use leptos::prelude::*;
use mycelix_leptos_client::{HoloHashBytes, HoloHashKind};
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use std::cell::{Cell, RefCell};
use std::collections::BTreeSet;
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HistoricalVoteView {
    pub record_hash: String,
    pub decision_hash: String,
    pub voter: String,
    pub choice: u32,
    pub weight_bp: u32,
    pub reasoning: Option<String>,
    pub created_at: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoteHistoryLookup {
    pub decision_hash: String,
    pub state: AvailabilityStateKind,
    /// Source order returned by `get_vote_history`; the zome sorts by action
    /// timestamp. We intentionally do not reinterpret the sequence into motive.
    pub entries: Vec<HistoricalVoteView>,
}

#[derive(Clone)]
pub struct VoteHistoryState {
    pub availability: RwSignal<AvailabilityStateKind>,
    pub current_alignment: RwSignal<AvailabilityStateKind>,
    pub lookups: RwSignal<Vec<VoteHistoryLookup>>,
    pub loading: RwSignal<bool>,
}

fn decode_history_record(
    record: &WireRecord,
    decision: &DecisionView,
) -> Option<HistoricalVoteView> {
    let vote: WireVote = record.decode_entry()?;
    let decision_hash = HoloHashBytes::from_raw_39(vote.decision_hash)
        .ok()?
        .require_kind(HoloHashKind::Action)
        .ok()?
        .to_raw_base64();
    if decision_hash != decision.hash
        || vote.choice as usize >= decision.options.len()
        || vote.weight_bp > 10_000
    {
        return None;
    }

    Some(HistoricalVoteView {
        record_hash: record.action_hash_b64(),
        decision_hash,
        voter: record_bridge::agent_display(&vote.voter)?,
        choice: vote.choice,
        weight_bp: vote.weight_bp,
        reasoning: vote.reasoning,
        created_at: vote.created_at / 1_000_000,
    })
}

fn decode_history(records: &[WireRecord], decision: &DecisionView) -> VoteHistoryLookup {
    let entries = records
        .iter()
        .filter_map(|record| decode_history_record(record, decision))
        .collect::<Vec<_>>();
    let state = match (records.len(), entries.len()) {
        (0, 0) => AvailabilityStateKind::Empty,
        (source, decoded) if source == decoded => AvailabilityStateKind::Live,
        _ => AvailabilityStateKind::Degraded,
    };

    VoteHistoryLookup {
        decision_hash: decision.hash.clone(),
        state,
        entries,
    }
}

fn summarize_history(
    decision_state: AvailabilityStateKind,
    lookups: &[VoteHistoryLookup],
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

fn same_vote(current: &VoteView, historical: &HistoricalVoteView) -> bool {
    current.decision_hash == historical.decision_hash
        && current.voter == historical.voter
        && current.choice == historical.choice
        && current.weight_bp == historical.weight_bp
        && current.reasoning == historical.reasoning
        && current.created_at == historical.created_at
}

/// Establish whether current-vote links agree with each voter's latest
/// immutable history entry. This never changes history availability itself.
fn current_alignment(
    vote_state: AvailabilityStateKind,
    current_votes: &[VoteView],
    lookups: &[VoteHistoryLookup],
) -> AvailabilityStateKind {
    if !matches!(
        vote_state,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty
    ) {
        return vote_state;
    }
    if (vote_state == AvailabilityStateKind::Empty && !current_votes.is_empty())
        || (vote_state == AvailabilityStateKind::Live && current_votes.is_empty())
    {
        return AvailabilityStateKind::Degraded;
    }
    if lookups.iter().any(|lookup| {
        matches!(
            lookup.state,
            AvailabilityStateKind::Unavailable | AvailabilityStateKind::Degraded
        )
    }) {
        return AvailabilityStateKind::Unavailable;
    }

    let mut current_keys = BTreeSet::new();
    for vote in current_votes {
        if !current_keys.insert((vote.decision_hash.clone(), vote.voter.clone())) {
            return AvailabilityStateKind::Degraded;
        }

        let Some(history) = lookups
            .iter()
            .find(|lookup| lookup.decision_hash == vote.decision_hash)
        else {
            return AvailabilityStateKind::Degraded;
        };
        let Some(latest_for_voter) = history
            .entries
            .iter()
            .rev()
            .find(|entry| entry.voter == vote.voter)
        else {
            return AvailabilityStateKind::Degraded;
        };
        if !same_vote(vote, latest_for_voter) {
            return AvailabilityStateKind::Degraded;
        }
    }

    // There is no vote-retraction transition. Therefore a latest historical
    // record without a matching current-vote link is also inconsistent.
    for lookup in lookups {
        let voters = lookup
            .entries
            .iter()
            .map(|entry| entry.voter.clone())
            .collect::<BTreeSet<_>>();
        for voter in voters {
            let Some(latest) = lookup.entries.iter().rev().find(|entry| entry.voter == voter) else {
                continue;
            };
            if !current_votes.iter().any(|vote| same_vote(vote, latest)) {
                return AvailabilityStateKind::Degraded;
            }
        }
    }

    if current_votes.is_empty() {
        AvailabilityStateKind::Empty
    } else {
        AvailabilityStateKind::Live
    }
}

fn snapshot_key(decisions: &[DecisionView], votes: &[VoteView]) -> (Vec<String>, Vec<String>) {
    let decision_keys = decisions
        .iter()
        .map(|decision| decision.hash.clone())
        .collect::<Vec<_>>();
    let vote_keys = votes
        .iter()
        .map(|vote| {
            format!(
                "{}|{}|{}|{}|{}|{:?}",
                vote.decision_hash,
                vote.voter,
                vote.choice,
                vote.weight_bp,
                vote.created_at,
                vote.reasoning
            )
        })
        .collect::<Vec<_>>();
    (decision_keys, vote_keys)
}

fn invalidate_load(generation: &Rc<Cell<u64>>) {
    generation.set(generation.get().wrapping_add(1));
}

pub fn provide_vote_history() -> VoteHistoryState {
    let state = VoteHistoryState {
        availability: RwSignal::new(AvailabilityStateKind::Mock),
        current_alignment: RwSignal::new(AvailabilityStateKind::Mock),
        lookups: RwSignal::new(Vec::new()),
        loading: RwSignal::new(false),
    };
    provide_context(state.clone());

    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let hc = use_holochain();
    let last_key = Rc::new(RefCell::new(None::<(Vec<String>, Vec<String>)>));
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
        let availability = truth_effect.availability.get();
        let decision_state = availability.decisions;
        let vote_state = availability.votes;

        match status {
            ConnectionStatus::Mock => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect.availability.set(AvailabilityStateKind::Mock);
                state_effect
                    .current_alignment
                    .set(AvailabilityStateKind::Mock);
            }
            ConnectionStatus::Connected if snapshot_loading => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect.availability.set(AvailabilityStateKind::Unknown);
                state_effect
                    .current_alignment
                    .set(AvailabilityStateKind::Unknown);
            }
            ConnectionStatus::Connected if !signer_ready => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.lookups.set(Vec::new());
                state_effect
                    .availability
                    .set(AvailabilityStateKind::Unavailable);
                state_effect
                    .current_alignment
                    .set(AvailabilityStateKind::Unavailable);
            }
            ConnectionStatus::Connected => {
                if decision_state == AvailabilityStateKind::Empty {
                    invalidate_load(&generation_effect);
                    *key_effect.borrow_mut() = Some((Vec::new(), Vec::new()));
                    state_effect.loading.set(false);
                    state_effect.lookups.set(Vec::new());
                    state_effect.availability.set(AvailabilityStateKind::Empty);
                    state_effect.current_alignment.set(if vote_state == AvailabilityStateKind::Empty {
                        AvailabilityStateKind::Empty
                    } else {
                        vote_state
                    });
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
                    state_effect.current_alignment.set(vote_state);
                    return;
                }

                let decisions = hearth_effect.decisions.get();
                let votes = hearth_effect.votes.get();
                let key = snapshot_key(&decisions, &votes);
                if key_effect.borrow().as_ref() == Some(&key) {
                    return;
                }
                *key_effect.borrow_mut() = Some(key);
                start_history_load(
                    state_effect.clone(),
                    hc_effect.clone(),
                    decisions,
                    decision_state,
                    votes,
                    vote_state,
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
                if matches!(
                    state_effect.current_alignment.get_untracked(),
                    AvailabilityStateKind::Live | AvailabilityStateKind::Empty
                ) {
                    state_effect
                        .current_alignment
                        .set(AvailabilityStateKind::Degraded);
                }
            }
            ConnectionStatus::Connecting => {
                invalidate_load(&generation_effect);
                *key_effect.borrow_mut() = None;
                state_effect.loading.set(false);
                state_effect.availability.set(AvailabilityStateKind::Unknown);
                state_effect
                    .current_alignment
                    .set(AvailabilityStateKind::Unknown);
            }
        }
    });

    state
}

pub fn use_vote_history() -> VoteHistoryState {
    expect_context::<VoteHistoryState>()
}

#[allow(clippy::too_many_arguments)]
fn start_history_load(
    state: VoteHistoryState,
    hc: mycelix_leptos_core::HolochainCtx,
    decisions: Vec<DecisionView>,
    decision_state: AvailabilityStateKind,
    current_votes: Vec<VoteView>,
    vote_state: AvailabilityStateKind,
    generation: Rc<Cell<u64>>,
) {
    invalidate_load(&generation);
    let token = generation.get();
    state.loading.set(true);
    state.lookups.set(Vec::new());
    state.availability.set(AvailabilityStateKind::Unknown);
    state
        .current_alignment
        .set(AvailabilityStateKind::Unknown);

    spawn_local(async move {
        let mut lookups = Vec::with_capacity(decisions.len());
        for decision in &decisions {
            let decision_hash = match HoloHashBytes::from_action_raw_base64(&decision.hash) {
                Ok(hash) => hash,
                Err(error) => {
                    web_sys::console::log_1(
                        &format!(
                            "[Hearth] vote-history query blocked by invalid Decision ActionHash {}: {error}",
                            decision.hash
                        )
                        .into(),
                    );
                    lookups.push(VoteHistoryLookup {
                        decision_hash: decision.hash.clone(),
                        state: AvailabilityStateKind::Degraded,
                        entries: Vec::new(),
                    });
                    continue;
                }
            };

            match hc
                .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
                    "hearth_decisions",
                    "get_vote_history",
                    &decision_hash,
                )
                .await
            {
                Ok(records) => lookups.push(decode_history(&records, decision)),
                Err(error) => {
                    web_sys::console::log_1(
                        &format!(
                            "[Hearth] get_vote_history failed for {}: {error}",
                            decision.hash
                        )
                        .into(),
                    );
                    lookups.push(VoteHistoryLookup {
                        decision_hash: decision.hash.clone(),
                        state: AvailabilityStateKind::Unavailable,
                        entries: Vec::new(),
                    });
                }
            }
        }

        if generation.get() != token {
            return;
        }

        let history_state = summarize_history(decision_state, &lookups);
        let alignment = current_alignment(vote_state, &current_votes, &lookups);
        state.lookups.set(lookups);
        state.availability.set(history_state);
        state.current_alignment.set(alignment);
        state.loading.set(false);
    });
}

#[component]
pub fn VoteHistorySummary() -> impl IntoView {
    let hearth = use_hearth();
    let state = use_vote_history();
    let hearth_view = hearth.clone();
    let state_view = state.clone();
    let state_status = state.clone();

    view! {
        <details class="vote-history-audit">
            <summary>"Vote audit history"</summary>
            <p class="vote-history-source" role="status">
                {move || format!(
                    "History source: {}{} · current-vote alignment: {}",
                    state_status.availability.get().label(),
                    if state_status.loading.get() { " (loading)" } else { "" },
                    state_status.current_alignment.get().label(),
                )}
            </p>
            <p>
                "This is the immutable vote-record sequence returned by hearth_decisions, including current and superseded records. Sequence does not imply motive. Current tally state remains a separate source."
            </p>
            {move || {
                let members = hearth_view.members.get();
                let decisions = hearth_view.decisions.get();
                let lookups = state_view.lookups.get();
                if decisions.is_empty() {
                    return view! { <p>"No Decisions are present in this snapshot."</p> }.into_any();
                }

                view! {
                    <div class="vote-history-decisions">
                        {decisions.into_iter().map(|decision| {
                            let lookup = lookups
                                .iter()
                                .find(|lookup| lookup.decision_hash == decision.hash)
                                .cloned();
                            let title = decision.title.clone();
                            let options = decision.options.clone();

                            match lookup {
                                Some(VoteHistoryLookup {
                                    state: AvailabilityStateKind::Live,
                                    entries,
                                    ..
                                }) => view! {
                                    <section class="vote-history-decision">
                                        <h3>{title}</h3>
                                        <ol>
                                            {entries.into_iter().enumerate().map(|(index, entry)| {
                                                let voter = member_name(&members, &entry.voter);
                                                let choice = options
                                                    .get(entry.choice as usize)
                                                    .cloned()
                                                    .unwrap_or_else(|| "Invalid option index".to_string());
                                                let weight = entry.weight_bp as f64 / 10_000.0;
                                                view! {
                                                    <li class="vote-history-record">
                                                        <p>{format!(
                                                            "Record {} · {} · {} · {:.2} vote-weight units",
                                                            index + 1,
                                                            voter,
                                                            choice,
                                                            weight,
                                                        )}</p>
                                                        {entry.reasoning.map(|reasoning| view! {
                                                            <p class="vote-history-reasoning">"Recorded reasoning: " {reasoning}</p>
                                                        })}
                                                    </li>
                                                }
                                            }).collect_view()}
                                        </ol>
                                    </section>
                                }.into_any(),
                                Some(VoteHistoryLookup {
                                    state: AvailabilityStateKind::Empty,
                                    ..
                                }) => view! {
                                    <section class="vote-history-decision">
                                        <h3>{title}</h3>
                                        <p>"No vote-history records were returned for this Decision."</p>
                                    </section>
                                }.into_any(),
                                Some(VoteHistoryLookup {
                                    state: AvailabilityStateKind::Degraded,
                                    entries,
                                    ..
                                }) => view! {
                                    <section class="vote-history-decision" role="alert">
                                        <h3>{title}</h3>
                                        <p>{format!(
                                            "Vote history is only partially established. {} valid record(s) are retained but must not be treated as a complete audit sequence.",
                                            entries.len(),
                                        )}</p>
                                    </section>
                                }.into_any(),
                                Some(VoteHistoryLookup {
                                    state: AvailabilityStateKind::Unavailable,
                                    ..
                                }) => view! {
                                    <section class="vote-history-decision" role="alert">
                                        <h3>{title}</h3>
                                        <p>"Vote-history query unavailable for this Decision."</p>
                                    </section>
                                }.into_any(),
                                _ => view! {
                                    <section class="vote-history-decision">
                                        <h3>{title}</h3>
                                        <p>"Vote history has not been established yet."</p>
                                    </section>
                                }.into_any(),
                            }
                        }).collect_view()}
                    </div>
                }.into_any()
            }}
        </details>
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HistoricalVoteView, VoteHistoryLookup, current_alignment, same_vote,
        summarize_history,
    };
    use hearth_leptos_types::VoteView;
    use mycelix_leptos_core::AvailabilityStateKind;

    fn current(choice: u32, created_at: i64) -> VoteView {
        VoteView {
            decision_hash: "decision".into(),
            voter: "agent".into(),
            choice,
            weight_bp: 5_000,
            reasoning: Some("because".into()),
            created_at,
        }
    }

    fn historical(choice: u32, created_at: i64) -> HistoricalVoteView {
        HistoricalVoteView {
            record_hash: format!("record-{created_at}"),
            decision_hash: "decision".into(),
            voter: "agent".into(),
            choice,
            weight_bp: 5_000,
            reasoning: Some("because".into()),
            created_at,
        }
    }

    #[test]
    fn exact_current_vote_matches_latest_history_record() {
        let old = historical(0, 10);
        let latest = historical(1, 20);
        let vote = current(1, 20);
        assert!(same_vote(&vote, &latest));
        let lookups = vec![VoteHistoryLookup {
            decision_hash: "decision".into(),
            state: AvailabilityStateKind::Live,
            entries: vec![old, latest],
        }];
        assert_eq!(
            current_alignment(AvailabilityStateKind::Live, &[vote], &lookups),
            AvailabilityStateKind::Live
        );
    }

    #[test]
    fn stale_current_link_is_degraded() {
        let old = historical(0, 10);
        let latest = historical(1, 20);
        let stale = current(0, 10);
        let lookups = vec![VoteHistoryLookup {
            decision_hash: "decision".into(),
            state: AvailabilityStateKind::Live,
            entries: vec![old, latest],
        }];
        assert_eq!(
            current_alignment(AvailabilityStateKind::Live, &[stale], &lookups),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn contradictory_current_availability_is_degraded() {
        assert_eq!(
            current_alignment(AvailabilityStateKind::Empty, &[current(0, 10)], &[]),
            AvailabilityStateKind::Degraded
        );
        assert_eq!(
            current_alignment(AvailabilityStateKind::Live, &[], &[]),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn history_can_be_live_while_current_alignment_is_unavailable() {
        let lookups = vec![VoteHistoryLookup {
            decision_hash: "decision".into(),
            state: AvailabilityStateKind::Live,
            entries: vec![historical(0, 10)],
        }];
        assert_eq!(
            summarize_history(AvailabilityStateKind::Live, &lookups),
            AvailabilityStateKind::Live
        );
        assert_eq!(
            current_alignment(AvailabilityStateKind::Unavailable, &[], &lookups),
            AvailabilityStateKind::Unavailable
        );
    }

    #[test]
    fn partial_history_degrades_global_history() {
        let lookups = vec![VoteHistoryLookup {
            decision_hash: "decision".into(),
            state: AvailabilityStateKind::Degraded,
            entries: vec![],
        }];
        assert_eq!(
            summarize_history(AvailabilityStateKind::Live, &lookups),
            AvailabilityStateKind::Degraded
        );
    }
}
