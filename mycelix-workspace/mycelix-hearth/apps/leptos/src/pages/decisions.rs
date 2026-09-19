// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::governance_actions;
use crate::hearth_actions;
use crate::hearth_context::{member_name, use_hearth};
use hearth_leptos_types::*;
use leptos::prelude::*;
use mycelix_leptos_core::consciousness::DimensionWeights;
use mycelix_leptos_core::holochain_provider::use_holochain;
use mycelix_leptos_core::{ConnectionStatus, use_consciousness};

fn decision_type_label(decision_type: &DecisionType) -> &'static str {
    match decision_type {
        DecisionType::Consensus => "Consensus",
        DecisionType::MajorityVote => "Majority vote",
        DecisionType::ElderDecision => "Elder decision",
        DecisionType::GuardianDecision => "Guardian decision",
    }
}

fn eligible_roles_label(roles: &[MemberRole]) -> String {
    if roles.is_empty() {
        return "No eligible roles recorded".to_string();
    }
    roles
        .iter()
        .map(MemberRole::label)
        .collect::<Vec<_>>()
        .join(", ")
}

fn quorum_label(quorum_bp: Option<u32>) -> String {
    match quorum_bp {
        None => "No quorum requirement".to_string(),
        Some(bp) => format!("Participation quorum: {:.2}%", bp as f64 / 100.0),
    }
}

fn deadline_orientation(deadline_seconds: i64, now_seconds: i64) -> String {
    if deadline_seconds <= now_seconds {
        return "Configured voting deadline has passed according to this device clock".to_string();
    }

    let remaining = deadline_seconds.saturating_sub(now_seconds);
    if remaining < 3_600 {
        let minutes = (remaining + 59) / 60;
        format!("Configured voting deadline is in about {minutes} minute(s) by this device clock")
    } else if remaining < 86_400 {
        let hours = (remaining + 3_599) / 3_600;
        format!("Configured voting deadline is in about {hours} hour(s) by this device clock")
    } else {
        let days = (remaining + 86_399) / 86_400;
        format!("Configured voting deadline is in about {days} day(s) by this device clock")
    }
}

fn browser_deadline_orientation(deadline_seconds: i64) -> String {
    let now_seconds = (js_sys::Date::now() / 1_000.0) as i64;
    deadline_orientation(deadline_seconds, now_seconds)
}

fn lifecycle_explanation(status: &DecisionStatus) -> &'static str {
    match status {
        DecisionStatus::Open => {
            "Open does not itself prove that a vote will be accepted. The decisions zome still checks the authoritative deadline, caller role, duplicate-vote state, and civic eligibility."
        }
        DecisionStatus::Closed => {
            "Closed blocks further voting. If votes existed when it was closed, the zome may also have recorded a closing outcome; this client does not load DecisionOutcome records yet."
        }
        DecisionStatus::Finalized => {
            "Finalized means the decisions zome recorded a DecisionOutcome after applying its deadline, role, quorum, and decision-type rules. This client does not load that outcome record yet."
        }
    }
}

#[component]
fn DecisionComposer() -> impl IntoView {
    let (open, set_open) = signal(false);
    let (title, set_title) = signal(String::new());
    let (description, set_description) = signal(String::new());
    let (decision_type, set_decision_type) = signal(String::new());
    let (options_text, set_options_text) = signal(String::new());
    let (voting_window_days, set_voting_window_days) = signal(0u32);
    let (quorum_mode, set_quorum_mode) = signal(String::new());
    let (quorum_percent, set_quorum_percent) = signal(String::new());
    let (founder, set_founder) = signal(false);
    let (elder, set_elder) = signal(false);
    let (adult, set_adult) = signal(false);
    let (youth, set_youth) = signal(false);
    let (child, set_child) = signal(false);
    let (guest, set_guest) = signal(false);
    let (ancestor, set_ancestor) = signal(false);
    let (form_error, set_form_error) = signal::<Option<String>>(None);

    let submit = move |_| {
        set_form_error.set(None);

        let decision_type = match decision_type.get().as_str() {
            "consensus" => DecisionType::Consensus,
            "majority" => DecisionType::MajorityVote,
            "elder" => DecisionType::ElderDecision,
            "guardian" => DecisionType::GuardianDecision,
            _ => {
                set_form_error.set(Some("Choose a decision type.".to_string()));
                return;
            }
        };

        let mut eligible_roles = Vec::new();
        if founder.get() {
            eligible_roles.push(MemberRole::Founder);
        }
        if elder.get() {
            eligible_roles.push(MemberRole::Elder);
        }
        if adult.get() {
            eligible_roles.push(MemberRole::Adult);
        }
        if youth.get() {
            eligible_roles.push(MemberRole::Youth);
        }
        if child.get() {
            eligible_roles.push(MemberRole::Child);
        }
        if guest.get() {
            eligible_roles.push(MemberRole::Guest);
        }
        if ancestor.get() {
            eligible_roles.push(MemberRole::Ancestor);
        }

        if eligible_roles.is_empty() {
            set_form_error.set(Some("Choose at least one eligible Hearth role.".to_string()));
            return;
        }

        let window_days = voting_window_days.get();
        if window_days == 0 {
            set_form_error.set(Some("Choose a voting window.".to_string()));
            return;
        }

        let quorum_bp = match quorum_mode.get().as_str() {
            "none" => None,
            "percent" => {
                let percent = match quorum_percent.get().trim().parse::<u32>() {
                    Ok(value) if value <= 100 => value,
                    _ => {
                        set_form_error.set(Some(
                            "Quorum must be a whole percentage from 0 to 100.".to_string(),
                        ));
                        return;
                    }
                };
                Some(percent * 100)
            }
            _ => {
                set_form_error.set(Some("Choose whether this decision requires quorum.".to_string()));
                return;
            }
        };

        let options = options_text
            .get()
            .lines()
            .map(str::to_string)
            .collect::<Vec<_>>();

        let scheduled = governance_actions::create_decision(
            governance_actions::CreateDecisionDraft {
                title: title.get(),
                description: description.get(),
                decision_type,
                eligible_roles,
                options,
                voting_window_days: window_days,
                quorum_bp,
            },
        );

        // Closing after a scheduled live attempt prevents accidental duplicate
        // dispatch. A later unknown outcome must be reconciled before retrying.
        if scheduled {
            set_open.set(false);
        }
    };

    view! {
        <div class="decision-create">
            <button
                class="action-btn"
                type="button"
                aria-expanded=move || if open.get() { "true" } else { "false" }
                aria-controls="decision-composer"
                on:click=move |_| set_open.update(|value| *value = !*value)
            >
                {move || if open.get() { "Close New Decision" } else { "+ New Decision" }}
            </button>

            <Show when=move || open.get()>
                <form
                    id="decision-composer"
                    class="decision-composer"
                    aria-describedby="decision-composer-authority"
                    on:submit=move |event| {
                        event.prevent_default();
                        submit(());
                    }
                >
                    <h2>"Create a household decision"</h2>
                    <p id="decision-composer-authority" class="task-surface-note">
                        "This form prepares a request. The Hearth decisions and civic zomes independently decide whether creation is authorized and whether the caller is an active member."
                    </p>

                    <div class="form-row">
                        <label for="decision-title">"Title"</label>
                        <input
                            id="decision-title"
                            type="text"
                            maxlength="256"
                            required=true
                            prop:value=move || title.get()
                            on:input=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned())
                                {
                                    set_title.set(input.value());
                                }
                            }
                        />
                    </div>

                    <div class="form-row">
                        <label for="decision-description">"Description"</label>
                        <textarea
                            id="decision-description"
                            maxlength="4096"
                            prop:value=move || description.get()
                            on:input=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlTextAreaElement>().cloned())
                                {
                                    set_description.set(input.value());
                                }
                            }
                        ></textarea>
                    </div>

                    <div class="form-row">
                        <label for="decision-type">"Decision type"</label>
                        <select
                            id="decision-type"
                            required=true
                            prop:value=move || decision_type.get()
                            on:change=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_decision_type.set(input.value());
                                }
                            }
                        >
                            <option value="">"Choose a decision type"</option>
                            <option value="consensus">"Consensus"</option>
                            <option value="majority">"Majority vote"</option>
                            <option value="elder">"Elder decision"</option>
                            <option value="guardian">"Guardian decision"</option>
                        </select>
                    </div>

                    <fieldset class="decision-role-fieldset">
                        <legend>"Eligible Hearth roles"</legend>
                        <p class="task-surface-note">
                            "Choose explicitly. Role eligibility is recorded on the Decision; effective live vote authority and weight are still determined by the zomes."
                        </p>
                        <label><input type="checkbox" prop:checked=move || founder.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_founder.set(input.checked()); } } /> "Founder"</label>
                        <label><input type="checkbox" prop:checked=move || elder.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_elder.set(input.checked()); } } /> "Elder"</label>
                        <label><input type="checkbox" prop:checked=move || adult.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_adult.set(input.checked()); } } /> "Adult"</label>
                        <label><input type="checkbox" prop:checked=move || youth.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_youth.set(input.checked()); } } /> "Youth"</label>
                        <label><input type="checkbox" prop:checked=move || child.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_child.set(input.checked()); } } /> "Child"</label>
                        <label><input type="checkbox" prop:checked=move || guest.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_guest.set(input.checked()); } } /> "Guest"</label>
                        <label><input type="checkbox" prop:checked=move || ancestor.get() on:change=move |event| { use wasm_bindgen::JsCast; if let Some(input) = event.target().and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned()) { set_ancestor.set(input.checked()); } } /> "Ancestor"</label>
                    </fieldset>

                    <div class="form-row">
                        <label for="decision-options">"Options — one per line"</label>
                        <textarea
                            id="decision-options"
                            rows="5"
                            required=true
                            aria-describedby="decision-options-help"
                            prop:value=move || options_text.get()
                            on:input=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlTextAreaElement>().cloned())
                                {
                                    set_options_text.set(input.value());
                                }
                            }
                        ></textarea>
                        <small id="decision-options-help">"Use 2–20 non-empty options. Each option may be up to 1024 UTF-8 bytes under the current integrity contract."</small>
                    </div>

                    <div class="form-row">
                        <label for="decision-window">"Voting window"</label>
                        <select
                            id="decision-window"
                            required=true
                            prop:value=move || voting_window_days.get().to_string()
                            on:change=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_voting_window_days.set(input.value().parse::<u32>().unwrap_or(0));
                                }
                            }
                        >
                            <option value="0">"Choose a voting window"</option>
                            <option value="1">"1 day"</option>
                            <option value="3">"3 days"</option>
                            <option value="7">"7 days"</option>
                            <option value="14">"14 days"</option>
                            <option value="30">"30 days"</option>
                        </select>
                        <small>"The browser computes the requested absolute deadline; the conductor independently requires it to still be in the future when creation is processed."</small>
                    </div>

                    <div class="form-row">
                        <label for="decision-quorum-mode">"Quorum rule"</label>
                        <select
                            id="decision-quorum-mode"
                            required=true
                            prop:value=move || quorum_mode.get()
                            on:change=move |event| {
                                use wasm_bindgen::JsCast;
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_quorum_mode.set(input.value());
                                }
                            }
                        >
                            <option value="">"Choose a quorum rule"</option>
                            <option value="none">"No quorum requirement"</option>
                            <option value="percent">"Require participation percentage"</option>
                        </select>
                    </div>

                    <Show when=move || quorum_mode.get() == "percent">
                        <div class="form-row">
                            <label for="decision-quorum-percent">"Required participation (%)"</label>
                            <input
                                id="decision-quorum-percent"
                                type="number"
                                min="0"
                                max="100"
                                step="1"
                                required=true
                                prop:value=move || quorum_percent.get()
                                on:input=move |event| {
                                    use wasm_bindgen::JsCast;
                                    if let Some(input) = event
                                        .target()
                                        .and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned())
                                    {
                                        set_quorum_percent.set(input.value());
                                    }
                                }
                            />
                            <small>"This first composer exposes whole percentages; the zome stores quorum in basis points."</small>
                        </div>
                    </Show>

                    <Show when=move || form_error.get().is_some()>
                        <p class="form-error" role="alert">
                            {move || form_error.get().unwrap_or_default()}
                        </p>
                    </Show>

                    <div class="vote-actions">
                        <button class="action-btn" type="submit">"Submit Decision Request"</button>
                        <button class="cancel-btn" type="button" on:click=move |_| set_open.set(false)>
                            "Cancel"
                        </button>
                    </div>
                </form>
            </Show>
        </div>
    }
}

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
    let do_submit_vote = move |decision_hash: String, amend: bool| {
        let choice = match selected_choice.get() {
            Some(choice) => choice,
            None => return,
        };

        // Browser-side weight exists only for demo-mode behavior. Live cast and
        // amendment calls derive authoritative weight in the decisions/civic zomes.
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

        if amend {
            governance_actions::amend_vote(decision_hash, choice, reasoning, demo_weight);
        } else {
            hearth_actions::cast_vote(decision_hash, choice, reasoning, demo_weight);
        }

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

            <DecisionComposer />

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
                                    let decision_type = decision_type_label(&decision.decision_type).to_string();
                                    let eligible_roles = eligible_roles_label(&decision.eligible_roles);
                                    let quorum = quorum_label(decision.quorum_bp);
                                    let deadline = browser_deadline_orientation(decision.deadline);
                                    let lifecycle = lifecycle_explanation(&decision.status).to_string();
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
                                            <div class="decision-contract" role="group" aria-label="Decision rules and lifecycle">
                                                <span>{format!("Eligible roles: {eligible_roles}")}</span>
                                                <span>{quorum}</span>
                                                <span>{deadline}</span>
                                            </div>
                                            <p class="decision-lifecycle-note">{lifecycle}</p>

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
                                                let hash_for_submit = hash.clone();
                                                if is_voting {
                                                    let submit_label = if already_voted {
                                                        if demo_mode { "Update Demo Vote" } else { "Amend Vote" }
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
                                                                    on:click=move |_| {
                                                                        do_submit_vote(hash_for_submit.clone(), already_voted)
                                                                    }
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
                                                } else {
                                                    let initial_choice = existing_vote.as_ref().map(|vote| vote.choice);
                                                    let button_label = if already_voted {
                                                        if demo_mode { "Change Demo Vote" } else { "Amend Vote" }
                                                    } else {
                                                        "Vote"
                                                    };
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
                                                            {button_label}
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

#[cfg(test)]
mod tests {
    use super::{deadline_orientation, decision_type_label, eligible_roles_label, lifecycle_explanation, quorum_label};
    use hearth_leptos_types::{DecisionStatus, DecisionType, MemberRole};

    #[test]
    fn governance_contract_labels_are_human_readable() {
        assert_eq!(decision_type_label(&DecisionType::MajorityVote), "Majority vote");
        assert_eq!(
            eligible_roles_label(&[MemberRole::Founder, MemberRole::Adult]),
            "Founder, Adult"
        );
        assert_eq!(quorum_label(None), "No quorum requirement");
        assert_eq!(quorum_label(Some(5_050)), "Participation quorum: 50.50%");
    }

    #[test]
    fn deadline_orientation_is_explicitly_device_relative() {
        assert!(deadline_orientation(1_000, 1_000).contains("device clock"));
        assert!(deadline_orientation(1_060, 1_000).contains("minute"));
        assert!(deadline_orientation(8_200, 1_000).contains("hour"));
        assert!(deadline_orientation(200_000, 1_000).contains("day"));
    }

    #[test]
    fn lifecycle_copy_does_not_strengthen_open_into_votable() {
        let open = lifecycle_explanation(&DecisionStatus::Open);
        assert!(open.contains("does not itself prove"));
        assert!(open.contains("zome"));

        let finalized = lifecycle_explanation(&DecisionStatus::Finalized);
        assert!(finalized.contains("DecisionOutcome"));
        assert!(finalized.contains("does not load"));
    }
}
