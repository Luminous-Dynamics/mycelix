// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared source-backed projection for `get_my_pending_votes`.
//!
//! Despite the zome function's historical name, the demonstrated contract is
//! narrower: Open Decisions for which the caller has no current AgentToVotes
//! link. Deadline, role eligibility and civic authority are still established
//! only by the vote transition itself.

use crate::hearth_context::use_hearth;
use crate::hearth_truth::use_hearth_truth;
use crate::record_bridge::{self, WireRecord};
use hearth_leptos_types::{DecisionStatus, DecisionView, VoteView};
use leptos::prelude::*;
use leptos_router::components::A;
use mycelix_leptos_client::HoloHashBytes;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{AvailabilityStateKind, ConnectionStatus};
use std::cell::{Cell, RefCell};
use std::collections::BTreeSet;
use std::rc::Rc;
use wasm_bindgen_futures::spawn_local;

#[derive(Clone, Debug)]
pub struct UnvotedDecisionSnapshot {
    pub availability: AvailabilityStateKind,
    pub snapshot_alignment: AvailabilityStateKind,
    pub decisions: Vec<DecisionView>,
}

impl UnvotedDecisionSnapshot {
    fn unknown() -> Self {
        Self {
            availability: AvailabilityStateKind::Unknown,
            snapshot_alignment: AvailabilityStateKind::Unknown,
            decisions: Vec::new(),
        }
    }
}

#[derive(Clone)]
pub struct UnvotedDecisionState {
    pub snapshot: RwSignal<UnvotedDecisionSnapshot>,
    pub loading: RwSignal<bool>,
}

fn validate_source_records(
    records: &[WireRecord],
    current_hearth_hash: &str,
) -> (AvailabilityStateKind, Vec<DecisionView>) {
    let decoded = record_bridge::records_to_decisions(records);
    let mut valid = Vec::new();
    let mut seen = BTreeSet::new();

    for decision in decoded {
        let hash_ok = HoloHashBytes::from_action_raw_base64(&decision.hash).is_ok();
        let hearth_ok = HoloHashBytes::from_action_raw_base64(&decision.hearth_hash).is_ok()
            && decision.hearth_hash == current_hearth_hash;
        let unique = seen.insert(decision.hash.clone());
        if hash_ok && hearth_ok && unique && decision.status == DecisionStatus::Open {
            valid.push(decision);
        }
    }

    let state = match (records.len(), valid.len()) {
        (0, 0) => AvailabilityStateKind::Empty,
        (source, accepted) if source == accepted => AvailabilityStateKind::Live,
        _ => AvailabilityStateKind::Degraded,
    };
    (state, valid)
}

fn expected_unvoted_hashes(
    decisions: &[DecisionView],
    votes: &[VoteView],
    my_agent: &str,
) -> BTreeSet<String> {
    decisions
        .iter()
        .filter(|decision| decision.status == DecisionStatus::Open)
        .filter(|decision| {
            !votes
                .iter()
                .any(|vote| vote.decision_hash == decision.hash && vote.voter == my_agent)
        })
        .map(|decision| decision.hash.clone())
        .collect()
}

fn snapshot_alignment(
    decision_state: AvailabilityStateKind,
    vote_state: AvailabilityStateKind,
    decisions: &[DecisionView],
    votes: &[VoteView],
    my_agent: &str,
    returned: &[DecisionView],
) -> AvailabilityStateKind {
    if my_agent.is_empty() {
        return AvailabilityStateKind::Unknown;
    }
    if decision_state == AvailabilityStateKind::Degraded
        || vote_state == AvailabilityStateKind::Degraded
    {
        return AvailabilityStateKind::Degraded;
    }
    if decision_state == AvailabilityStateKind::Unavailable
        || vote_state == AvailabilityStateKind::Unavailable
    {
        return AvailabilityStateKind::Unavailable;
    }
    if !matches!(
        decision_state,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty
    ) || !matches!(
        vote_state,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty
    ) {
        return AvailabilityStateKind::Unknown;
    }

    let expected = expected_unvoted_hashes(decisions, votes, my_agent);
    let actual = returned
        .iter()
        .map(|decision| decision.hash.clone())
        .collect::<BTreeSet<_>>();
    if expected == actual {
        AvailabilityStateKind::Live
    } else {
        AvailabilityStateKind::Degraded
    }
}

/// Derive the personal-attention presentation state from both the direct
/// source read and its independent alignment with the loaded Decisions/Votes
/// snapshot. A successful empty query is not enough for a calm-state claim
/// unless the cross-source alignment is also established.
pub fn attention_state(snapshot: &UnvotedDecisionSnapshot) -> AvailabilityStateKind {
    match snapshot.availability {
        AvailabilityStateKind::Mock => AvailabilityStateKind::Mock,
        AvailabilityStateKind::Unknown => AvailabilityStateKind::Unknown,
        AvailabilityStateKind::Unavailable => AvailabilityStateKind::Unavailable,
        AvailabilityStateKind::Locked => AvailabilityStateKind::Locked,
        AvailabilityStateKind::Degraded => AvailabilityStateKind::Degraded,
        AvailabilityStateKind::Live | AvailabilityStateKind::Empty => {
            match snapshot.snapshot_alignment {
                AvailabilityStateKind::Live => {
                    if snapshot.decisions.is_empty() {
                        AvailabilityStateKind::Empty
                    } else {
                        AvailabilityStateKind::Live
                    }
                }
                AvailabilityStateKind::Unknown => AvailabilityStateKind::Unknown,
                AvailabilityStateKind::Unavailable => AvailabilityStateKind::Unavailable,
                AvailabilityStateKind::Locked => AvailabilityStateKind::Locked,
                AvailabilityStateKind::Degraded
                | AvailabilityStateKind::Empty
                | AvailabilityStateKind::Mock => AvailabilityStateKind::Degraded,
            }
        }
    }
}

pub fn attention_is_established_empty(snapshot: &UnvotedDecisionSnapshot) -> bool {
    attention_state(snapshot) == AvailabilityStateKind::Empty
}

fn source_key(
    hearth_hash: &str,
    decisions: &[DecisionView],
    votes: &[VoteView],
    my_agent: &str,
) -> String {
    let mut decision_keys = decisions
        .iter()
        .map(|decision| format!("{}:{:?}", decision.hash, decision.status))
        .collect::<Vec<_>>();
    decision_keys.sort();
    let mut vote_keys = votes
        .iter()
        .map(|vote| format!("{}:{}:{}", vote.decision_hash, vote.voter, vote.created_at))
        .collect::<Vec<_>>();
    vote_keys.sort();
    format!(
        "{hearth_hash}|{my_agent}|{}|{}",
        decision_keys.join(","),
        vote_keys.join(",")
    )
}

fn invalidate(generation: &Rc<Cell<u64>>) {
    generation.set(generation.get().wrapping_add(1));
}

pub fn provide_unvoted_decisions() -> UnvotedDecisionState {
    let state = UnvotedDecisionState {
        snapshot: RwSignal::new(UnvotedDecisionSnapshot::unknown()),
        loading: RwSignal::new(false),
    };
    provide_context(state.clone());

    let hearth = use_hearth();
    let truth = use_hearth_truth();
    let hc = use_holochain();
    let last_key = Rc::new(RefCell::new(None::<String>));
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
        let truth_loading = truth_effect.loading.get();
        let availability = truth_effect.availability.get();

        if status == ConnectionStatus::Mock {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            state_effect.snapshot.set(UnvotedDecisionSnapshot {
                availability: AvailabilityStateKind::Mock,
                snapshot_alignment: AvailabilityStateKind::Mock,
                decisions: Vec::new(),
            });
            return;
        }

        if status != ConnectionStatus::Connected || !signer_ready || truth_loading {
            invalidate(&generation_effect);
            *key_effect.borrow_mut() = None;
            state_effect.loading.set(false);
            let source_state = if status == ConnectionStatus::Connected && !signer_ready {
                AvailabilityStateKind::Unavailable
            } else if matches!(
                status,
                ConnectionStatus::Disconnected | ConnectionStatus::Reconnecting
            ) {
                AvailabilityStateKind::Degraded
            } else {
                AvailabilityStateKind::Unknown
            };
            state_effect.snapshot.update(|value| {
                value.availability = source_state.clone();
                value.snapshot_alignment = source_state;
            });
            return;
        }

        let Some(current_hearth) = hearth_effect.current_hearth.get() else {
            *key_effect.borrow_mut() = None;
            state_effect.snapshot.set(UnvotedDecisionSnapshot::unknown());
            return;
        };

        if !matches!(
            availability.current_hearth,
            AvailabilityStateKind::Live | AvailabilityStateKind::Degraded
        ) {
            *key_effect.borrow_mut() = None;
            state_effect.snapshot.set(UnvotedDecisionSnapshot::unknown());
            return;
        }

        let decisions = hearth_effect.decisions.get();
        let votes = hearth_effect.votes.get();
        let my_agent = hearth_effect.my_agent.get();
        let key = source_key(&current_hearth.hash, &decisions, &votes, &my_agent);
        if key_effect.borrow().as_deref() == Some(&key) {
            return;
        }
        *key_effect.borrow_mut() = Some(key);

        let hearth_hash = match HoloHashBytes::from_action_raw_base64(&current_hearth.hash) {
            Ok(hash) => hash,
            Err(error) => {
                web_sys::console::log_1(
                    &format!(
                        "[Hearth] unvoted-Decision query blocked by invalid Hearth ActionHash: {error}"
                    )
                    .into(),
                );
                state_effect.snapshot.set(UnvotedDecisionSnapshot {
                    availability: AvailabilityStateKind::Degraded,
                    snapshot_alignment: AvailabilityStateKind::Degraded,
                    decisions: Vec::new(),
                });
                return;
            }
        };

        invalidate(&generation_effect);
        let token = generation_effect.get();
        state_effect.loading.set(true);
        state_effect.snapshot.set(UnvotedDecisionSnapshot::unknown());

        let state_load = state_effect.clone();
        let hc_load = hc_effect.clone();
        let generation_load = generation_effect.clone();
        let hearth_hash_text = current_hearth.hash;
        let decision_state = availability.decisions;
        let vote_state = availability.votes;
        spawn_local(async move {
            let result = hc_load
                .call_zome_default::<HoloHashBytes, Vec<WireRecord>>(
                    "hearth_decisions",
                    "get_my_pending_votes",
                    &hearth_hash,
                )
                .await;

            if generation_load.get() != token {
                return;
            }

            match result {
                Ok(records) => {
                    let (source_state, returned) =
                        validate_source_records(&records, &hearth_hash_text);
                    let alignment = snapshot_alignment(
                        decision_state,
                        vote_state,
                        &decisions,
                        &votes,
                        &my_agent,
                        &returned,
                    );
                    state_load.snapshot.set(UnvotedDecisionSnapshot {
                        availability: source_state,
                        snapshot_alignment: alignment,
                        decisions: returned,
                    });
                }
                Err(error) => {
                    web_sys::console::log_1(
                        &format!("[Hearth] get_my_pending_votes failed: {error}").into(),
                    );
                    state_load.snapshot.set(UnvotedDecisionSnapshot {
                        availability: AvailabilityStateKind::Unavailable,
                        snapshot_alignment: AvailabilityStateKind::Unavailable,
                        decisions: Vec::new(),
                    });
                }
            }
            state_load.loading.set(false);
        });
    });

    state
}

pub fn use_unvoted_decisions() -> UnvotedDecisionState {
    expect_context::<UnvotedDecisionState>()
}

#[component]
pub fn UnvotedOpenDecisionsInbox() -> impl IntoView {
    let state = use_unvoted_decisions();
    let state_status = state.clone();
    let state_view = state.clone();

    view! {
        <section class="inbox-unvoted-decisions" aria-labelledby="inbox-unvoted-heading">
            <h2 id="inbox-unvoted-heading">"Open decisions without a current vote"</h2>
            <p class="task-surface-note">
                "This source proves only that the Decision is Open and no current vote link for you was found. It does not prove the deadline is still open, your Hearth role is eligible, or the civic gate will accept a vote. Those checks remain authoritative at vote time."
            </p>
            <p class="inbox-source-state" role="status">
                {move || {
                    let value = state_status.snapshot.get();
                    format!(
                        "Source: {}{} · alignment with the loaded Decisions/Votes snapshot: {}",
                        value.availability.label(),
                        if state_status.loading.get() { " (loading)" } else { "" },
                        value.snapshot_alignment.label(),
                    )
                }}
            </p>
            {move || {
                let value = state_view.snapshot.get();
                let presentation_state = attention_state(&value);
                match presentation_state {
                    AvailabilityStateKind::Live => view! {
                        <div class="task-directory" role="list">
                            {value.decisions.into_iter().map(|decision| view! {
                                <A href="/decisions" attr:class="task-directory-item" attr:role="listitem">
                                    <strong>{decision.title}</strong>
                                    <span>"Open in the source with no current vote record for you. Open Decisions to inspect eligibility, deadline, evidence, and vote controls."</span>
                                </A>
                            }).collect_view()}
                        </div>
                    }.into_any(),
                    AvailabilityStateKind::Empty => view! {
                        <p class="empty-state">"The source and loaded snapshot agree that there are no Open Decisions without a current vote for you."</p>
                    }.into_any(),
                    AvailabilityStateKind::Degraded => view! {
                        <div class="empty-state" role="alert">
                            <p>"This Inbox attention projection is only partially established or disagrees with the loaded Decisions/Votes snapshot. It is not being presented as a complete action list."</p>
                            {(value.decisions.len() > 0).then(|| view! {
                                <p>{format!("{} validated source record(s) were retained for reconciliation.", value.decisions.len())}</p>
                            })}
                        </div>
                    }.into_any(),
                    AvailabilityStateKind::Unavailable => view! {
                        <p class="empty-state" role="alert">"The source-backed unvoted-Decision projection is unavailable in this snapshot."</p>
                    }.into_any(),
                    AvailabilityStateKind::Locked => view! {
                        <p class="empty-state" role="status">"The source-backed unvoted-Decision projection is locked."</p>
                    }.into_any(),
                    AvailabilityStateKind::Mock => view! {
                        <p class="empty-state">"Demo mode does not fabricate a source-backed governance Inbox."</p>
                    }.into_any(),
                    AvailabilityStateKind::Unknown => view! {
                        <p class="empty-state">"This Inbox source has not been established yet."</p>
                    }.into_any(),
                }
            }}
        </section>
    }
}

#[component]
pub fn UnvotedDecisionHomeAttention() -> impl IntoView {
    let state = use_unvoted_decisions();
    view! {
        {move || {
            let value = state.snapshot.get();
            match attention_state(&value) {
                AvailabilityStateKind::Live => view! {
                    <section class="home-attention" aria-labelledby="home-decision-attention-heading">
                        <h2 id="home-decision-attention-heading">"needs attention"</h2>
                        <A href="/inbox" attr:class="nudge">
                            {format!(
                                "{} open decision{} without a current vote · inspect Inbox",
                                value.decisions.len(),
                                if value.decisions.len() == 1 { "" } else { "s" },
                            )}
                        </A>
                        <p class="task-surface-note">
                            "This is an attention cue, not proof that a vote is currently authorized or required."
                        </p>
                    </section>
                }.into_any(),
                AvailabilityStateKind::Degraded | AvailabilityStateKind::Unavailable => view! {
                    <section class="home-attention" role="status">
                        <h2>"attention source"</h2>
                        <p>"Personal Decision attention is not fully established, so Home is not claiming there is nothing to do."</p>
                    </section>
                }.into_any(),
                AvailabilityStateKind::Unknown | AvailabilityStateKind::Locked => view! {
                    <section class="home-attention" role="status">
                        <h2>"attention source"</h2>
                        <p>"Personal Decision attention is still being established or is not currently readable."</p>
                    </section>
                }.into_any(),
                AvailabilityStateKind::Empty | AvailabilityStateKind::Mock => {
                    view! { <></> }.into_any()
                }
            }
        }}
    }
}

#[cfg(test)]
mod tests {
    use super::{
        attention_is_established_empty, attention_state, expected_unvoted_hashes,
        snapshot_alignment, UnvotedDecisionSnapshot,
    };
    use hearth_leptos_types::{DecisionStatus, DecisionType, DecisionView, MemberRole, VoteView};
    use mycelix_leptos_core::AvailabilityStateKind;

    fn decision(hash: &str, status: DecisionStatus) -> DecisionView {
        DecisionView {
            hash: hash.into(),
            hearth_hash: "hearth".into(),
            title: hash.into(),
            description: String::new(),
            decision_type: DecisionType::MajorityVote,
            eligible_roles: vec![MemberRole::Adult],
            options: vec!["A".into(), "B".into()],
            deadline: 100,
            quorum_bp: None,
            status,
            created_by: "creator".into(),
            created_at: 0,
        }
    }

    fn vote(decision_hash: &str) -> VoteView {
        VoteView {
            decision_hash: decision_hash.into(),
            voter: "me".into(),
            choice: 0,
            weight_bp: 10_000,
            reasoning: None,
            created_at: 10,
        }
    }

    fn snapshot(
        availability: AvailabilityStateKind,
        alignment: AvailabilityStateKind,
        decisions: Vec<DecisionView>,
    ) -> UnvotedDecisionSnapshot {
        UnvotedDecisionSnapshot {
            availability,
            snapshot_alignment: alignment,
            decisions,
        }
    }

    #[test]
    fn local_alignment_contract_is_open_and_unvoted_not_eligibility() {
        let decisions = vec![
            decision("a", DecisionStatus::Open),
            decision("b", DecisionStatus::Open),
            decision("c", DecisionStatus::Closed),
        ];
        let votes = vec![vote("b")];
        assert_eq!(
            expected_unvoted_hashes(&decisions, &votes, "me"),
            ["a".to_string()].into_iter().collect()
        );
    }

    #[test]
    fn matching_source_and_snapshot_align() {
        let decisions = vec![decision("a", DecisionStatus::Open)];
        assert_eq!(
            snapshot_alignment(
                AvailabilityStateKind::Live,
                AvailabilityStateKind::Empty,
                &decisions,
                &[],
                "me",
                &decisions,
            ),
            AvailabilityStateKind::Live
        );
    }

    #[test]
    fn source_mismatch_is_degraded_not_silently_reconciled() {
        let decisions = vec![decision("a", DecisionStatus::Open)];
        assert_eq!(
            snapshot_alignment(
                AvailabilityStateKind::Live,
                AvailabilityStateKind::Empty,
                &decisions,
                &[],
                "me",
                &[],
            ),
            AvailabilityStateKind::Degraded
        );
    }

    #[test]
    fn empty_attention_requires_both_source_and_alignment() {
        let clean = snapshot(
            AvailabilityStateKind::Empty,
            AvailabilityStateKind::Live,
            Vec::new(),
        );
        assert!(attention_is_established_empty(&clean));
        assert_eq!(attention_state(&clean), AvailabilityStateKind::Empty);

        let misaligned = snapshot(
            AvailabilityStateKind::Empty,
            AvailabilityStateKind::Degraded,
            Vec::new(),
        );
        assert!(!attention_is_established_empty(&misaligned));
        assert_eq!(attention_state(&misaligned), AvailabilityStateKind::Degraded);
    }

    #[test]
    fn live_attention_requires_aligned_nonempty_source() {
        let aligned = snapshot(
            AvailabilityStateKind::Live,
            AvailabilityStateKind::Live,
            vec![decision("a", DecisionStatus::Open)],
        );
        assert_eq!(attention_state(&aligned), AvailabilityStateKind::Live);

        let unavailable_alignment = snapshot(
            AvailabilityStateKind::Live,
            AvailabilityStateKind::Unavailable,
            vec![decision("a", DecisionStatus::Open)],
        );
        assert_eq!(
            attention_state(&unavailable_alignment),
            AvailabilityStateKind::Unavailable
        );
    }
}
