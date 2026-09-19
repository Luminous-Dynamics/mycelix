// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::hearth_actions;
use crate::hearth_context::{member_name, use_hearth};
use hearth_leptos_types::*;
use leptos::prelude::*;
use mycelix_leptos_core::consciousness::DimensionWeights;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{ConnectionStatus, use_consciousness};

#[component]
pub fn DecisionsPage() -> impl IntoView {
    let hearth = use_hearth();
    let consciousness = use_consciousness();
    let hc = use_holochain();

    let (voting_for, set_voting_for) = signal::<Option<String>>(None);
    let (selected_choice, set_selected_choice) = signal::<Option<u32>>(None);
    let (vote_reasoning, set_vote_reasoning) = signal(String::new());

    let hearth_for_vote = hearth.clone();
    let consciousness_for_vote = consciousness.clone();
    let hc_for_vote = hc.clone();
    let do_cast_vote = move |decision_hash: String| {
        let choice = match selected_choice.get() {
            Some(choice) => choice,
            None => return,
        };

        // The browser-side weight exists only to keep demo-mode behavior useful.
        // Live voting ignores this value: the decisions zome derives the
        // authoritative weight from role + civic eligibility when accepting the vote.
        let demo_weight = if hc_for_vote.is_mock() {
            let role = hearth_for_vote
                .my_role
                .get()
                .unwrap_or(MemberRole::Guest);
            let tier_bp = ((consciousness_for_vote
                .profile
                .get()
                .combined_score(&DimensionWeights::governance())
                * 10000.0) as u32)
                .min(10000);
            effective_vote_weight(&role, tier_bp)
        } else {
            0
        };

        let reasoning = vote_reasoning.get();
        let reasoning = if reasoning.is_empty() {
            None
        } else {
            Some(reasoning)
        };

        hearth_actions::cast_vote(decision_hash, choice, reasoning, demo_weight);

        set_voting_for.set(None);
        set_selected_choice.set(None);
        set_vote_reasoning.set(String::new());
    };

    let hearth_for_meta = hearth.clone();
    let consciousness_for_meta = consciousness.clone();
    let hc_for_meta = hc.clone();
    let hearth_for_list = hearth.clone();
    let hc_for_list = hc.clone();

    view! {
        <div class="page decisions-page">
            <h1 class="page-title">"decision chamber"</h1>
            <p class="page-subtitle">"consciousness-weighted governance"</p>

            <div class="consciousness-info" role="status">
                {move || {
                    if hc_for_meta.status.get() == ConnectionStatus::Mock {
                        let role = hearth_for_meta
                            .my_role
                            .get()
                            .unwrap_or(MemberRole::Guest);
                        let role_bp = role.default_vote_weight_bp();
                        let tier = consciousness_for_meta.tier.get();
                        let tier_bp = ((consciousness_for_meta
                            .profile
                            .get()
                            .combined_score(&DimensionWeights::governance())
                            * 10000.0) as u32)
                            .min(10000);
                        let estimated = effective_vote_weight(&role, tier_bp);

                        view! {
                            <span>
                                "Demo role: " {role.label()}
                                " · Demo tier: " {tier.label()}
                                " · Estimated demo weight: "
                                {format!("{:.2}", estimated as f64 / 10000.0)}
                                " vote-weight units"
                            </span>
                        }
                            .into_any()
                    } else {
                        let role = hearth_for_meta
                            .my_role
                            .get()
                            .map(|role| role.label().to_string())
                            .unwrap_or_else(|| "No active Hearth role established".to_string());

                        view! {
                            <span>
                                "Hearth role: " {role}
                                ". Effective vote eligibility and weight are assigned by the authoritative decisions/civic zomes when a vote is accepted. Recorded vote weights below come from vote records; this page does not predict your live weight."
                            </span>
                        }
                            .into_any()
                    }
                }}
            </div>

            <button
                class="action-btn"
                type="button"
                disabled=true
                aria-disabled="true"
                title="Decision creation is supported by the zome, but the frontend composer is not wired yet"
            >
                "+ New Decision — composer not wired"
            </button>

            {move || {
                let members = hearth_for_list.members.get();
                let decisions = hearth_for_list.decisions.get();
                let all_votes = hearth_for_list.votes.get();
                let my_agent = hearth_for_list.my_agent.get();
                let current_voting = voting_for.get();
                let current_choice = selected_choice.get();
                let demo_mode = hc_for_list.status.get() == ConnectionStatus::Mock;

                if decisions.is_empty() {
                    view! {
                        <div class="empty-state">"no crossroads at the moment. the path is clear."</div>
                    }
                        .into_any()
                } else {
                    view! {
                        <div class="decision-list">
                            {decisions
                                .iter()
                                .map(|decision| {
                                    let title = decision.title.clone();
                                    let description = decision.description.clone();
                                    let proposer = member_name(&members, &decision.created_by);
                                    let decision_votes = all_votes
                                        .iter()
                                        .filter(|vote| vote.decision_hash == decision.hash)
                                        .cloned()
                                        .collect::<Vec<_>>();
                                    let status_label = format!("{:?}", decision.status);
                                    let status_class = match decision.status {
                                        DecisionStatus::Open => "decision-open",
                                        DecisionStatus::Closed => "decision-closed",
                                        DecisionStatus::Finalized => "decision-finalized",
                                    };
                                    let decision_type = format!("{:?}", decision.decision_type);
                                    let options = decision.options.clone();
                                    let hash = decision.hash.clone();
                                    let is_open = decision.status == DecisionStatus::Open;
                                    let existing_vote = decision_votes
                                        .iter()
                                        .find(|vote| vote.voter == my_agent)
                                        .cloned();
                                    let already_voted = existing_vote.is_some();
                                    let is_voting = current_voting.as_ref() == Some(&hash);

                                    let option_weights = (0..options.len())
                                        .map(|index| {
                                            decision_votes
                                                .iter()
                                                .filter(|vote| vote.choice == index as u32)
                                                .map(|vote| vote.weight_bp as u64)
                                                .sum::<u64>()
                                        })
                                        .collect::<Vec<_>>();
                                    let max_option_weight = option_weights.iter().copied().max().unwrap_or(0);

                                    view! {
                                        <div class=format!("decision-card {status_class}")>
                                            <div class="decision-header">
                                                <h3>{title}</h3>
                                                <div class="decision-badges">
                                                    <span class="decision-type-badge">{decision_type}</span>
                                                    <span class="decision-status-badge">{status_label}</span>
                                                </div>
                                            </div>
                                            <p class="decision-desc">{description}</p>
                                            <p class="decision-proposer">"Proposed by " {proposer}</p>

                                            <div
                                                class="decision-options"
                                                aria-label="Recorded weighted support by option"
                                            >
                                                {options
                                                    .iter()
                                                    .enumerate()
                                                    .map(|(index, option)| {
                                                        let weight_bp = option_weights[index];
                                                        let relative_width = if max_option_weight == 0 {
                                                            0.0
                                                        } else {
                                                            weight_bp as f64 * 100.0 / max_option_weight as f64
                                                        };
                                                        let weight_units = weight_bp as f64 / 10000.0;
                                                        let option_text = option.clone();
                                                        let voters = decision_votes
                                                            .iter()
                                                            .filter(|vote| vote.choice == index as u32)
                                                            .map(|vote| member_name(&members, &vote.voter))
                                                            .collect::<Vec<_>>();
                                                        let choice_index = index as u32;
                                                        let is_selected = current_choice == Some(choice_index) && is_voting;

                                                        let support = view! {
                                                            <>
                                                                <span class="option-text">{option_text}</span>
                                                                <div
                                                                    class="option-bar-container"
                                                                    aria-hidden="true"
                                                                    title="Bar length is relative to the strongest option, not an absolute percentage"
                                                                >
                                                                    <div
                                                                        class="option-bar"
                                                                        style=format!("width: {relative_width:.2}%")
                                                                    ></div>
                                                                </div>
                                                                <span
                                                                    class="option-pct"
                                                                    aria-label=format!("{weight_units:.2} vote-weight units")
                                                                >
                                                                    {format!("{weight_units:.2} weight")}
                                                                </span>
                                                                {(!voters.is_empty()).then(|| {
                                                                    view! {
                                                                        <span class="option-voters">{voters.join(", ")}</span>
                                                                    }
                                                                })}
                                                            </>
                                                        };

                                                        if is_voting {
                                                            view! {
                                                                <button
                                                                    type="button"
                                                                    class=format!(
                                                                        "option-row option-selectable {}",
                                                                        if is_selected { "option-selected" } else { "" }
                                                                    )
                                                                    aria-pressed=if is_selected { "true" } else { "false" }
                                                                    on:click=move |_| set_selected_choice.set(Some(choice_index))
                                                                >
                                                                    <span class="option-radio" aria-hidden="true">
                                                                        {if is_selected { "{25c9}" } else { "{25cb}" }}
                                                                    </span>
                                                                    {support}
                                                                </button>
                                                            }
                                                                .into_any()
                                                        } else {
                                                            view! {
                                                                <div class="option-row">
                                                                    {support}
                                                                </div>
                                                            }
                                                                .into_any()
                                                        }
                                                    })
                                                    .collect_view()}
                                            </div>

                                            {if is_open {
                                                let hash_for_button = hash.clone();
                                                let hash_for_cast = hash.clone();
                                                if is_voting {
                                                    let submit_label = if already_voted && demo_mode {
                                                        "Update Demo Vote"
                                                    } else {
                                                        "Cast Vote"
                                                    };
                                                    view! {
                                                        <div class="vote-form">
                                                            <div class="form-row">
                                                                <label for="vote-reasoning">"Reasoning (optional)"</label>
                                                                <input
                                                                    id="vote-reasoning"
                                                                    type="text"
                                                                    placeholder="Why this choice?"
                                                                    prop:value=move || vote_reasoning.get()
                                                                    on:input=move |event| {
                                                                        use wasm_bindgen::JsCast;
                                                                        if let Some(input) = event
                                                                            .target()
                                                                            .and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned())
                                                                        {
                                                                            set_vote_reasoning.set(input.value());
                                                                        }
                                                                    }
                                                                />
                                                            </div>
                                                            <div class="vote-actions">
                                                                <button
                                                                    class="action-btn"
                                                                    type="button"
                                                                    disabled=move || selected_choice.get().is_none()
                                                                    on:click=move |_| do_cast_vote(hash_for_cast.clone())
                                                                >
                                                                    {submit_label}
                                                                </button>
                                                                <button
                                                                    class="cancel-btn"
                                                                    type="button"
                                                                    on:click=move |_| {
                                                                        set_voting_for.set(None);
                                                                        set_selected_choice.set(None);
                                                                    }
                                                                >
                                                                    "Cancel"
                                                                </button>
                                                            </div>
                                                        </div>
                                                    }
                                                        .into_any()
                                                } else if already_voted && !demo_mode {
                                                    view! {
                                                        <div class="vote-recorded" role="status">
                                                            "Vote recorded. Amendment is a distinct governance transition and is not wired in this frontend yet."
                                                        </div>
                                                    }
                                                        .into_any()
                                                } else {
                                                    let initial_choice = existing_vote.as_ref().map(|vote| vote.choice);
                                                    view! {
                                                        <button
                                                            class=format!(
                                                                "vote-btn {}",
                                                                if already_voted { "voted" } else { "" }
                                                            )
                                                            type="button"
                                                            on:click=move |_| {
                                                                set_voting_for.set(Some(hash_for_button.clone()));
                                                                set_selected_choice.set(initial_choice);
                                                            }
                                                        >
                                                            {if already_voted {
                                                                "Change Demo Vote"
                                                            } else {
                                                                "Vote"
                                                            }}
                                                        </button>
                                                    }
                                                        .into_any()
                                                }
                                            } else {
                                                view! { <span></span> }.into_any()
                                            }}

                                            <div class="decision-meta">
                                                {format!(
                                                    "{} vote{} recorded",
                                                    decision_votes.len(),
                                                    if decision_votes.len() == 1 { "" } else { "s" }
                                                )}
                                            </div>
                                        </div>
                                    }
                                })
                                .collect_view()}
                        </div>
                    }
                        .into_any()
                }
            }}
        </div>
    }
}
