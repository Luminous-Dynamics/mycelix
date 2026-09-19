// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::components::MemberAvatar;
use crate::hearth_context::{member_name, use_hearth};
use crate::pending_votes::{
    UnvotedDecisionHomeAttention, attention_is_established_empty, use_unvoted_decisions,
};
use crate::visualization::KinshipCanvas;
use hearth_leptos_types::*;
use leptos::prelude::*;
use mycelix_leptos_core::AvailabilityStateKind;

#[component]
pub fn HomePage() -> impl IntoView {
    let hearth = use_hearth();
    let decision_attention = use_unvoted_decisions();

    let neglected_bonds = move || {
        hearth
            .bonds
            .get()
            .iter()
            .filter(|b| b.strength_bp < 4000)
            .count()
    };

    // A reassuring calm state is a positive presentation claim. In live mode
    // it is allowed only when the personal Decision-attention source is
    // established empty *and* independently aligned with the loaded
    // Decisions/Votes snapshot. Unknown, unavailable, locked or degraded
    // attention therefore cannot collapse into "all is well".
    //
    // Mock mode remains explicitly local/demo-only and derives its own
    // attention state from the mock Decisions/Votes already on screen.
    let decision_attention_for_homeostasis = decision_attention.clone();
    let in_homeostasis = move || {
        let active_care = hearth
            .care_schedules
            .get()
            .iter()
            .any(|care| care.status == CareScheduleStatus::Active);

        let attention = decision_attention_for_homeostasis.snapshot.get();
        let no_decision_attention = if attention.availability == AvailabilityStateKind::Mock {
            let my_agent = hearth.my_agent.get();
            let votes = hearth.votes.get();
            !hearth.decisions.get().iter().any(|decision| {
                decision.status == DecisionStatus::Open
                    && !votes.iter().any(|vote| {
                        vote.decision_hash == decision.hash && vote.voter == my_agent
                    })
            })
        } else {
            attention_is_established_empty(&attention)
        };

        !active_care && no_decision_attention
    };

    view! {
        <div class="page home-page">
            {move || {
                if in_homeostasis() {
                    view! {
                        <div class="homeostatic-void">
                            <p class="void-message">"all is well"</p>
                            <p class="void-sub">"the hearth glows steady. rest here."</p>
                        </div>
                    }
                        .into_any()
                } else {
                    let members = hearth.members.get();
                    let recent_gratitude = hearth.gratitude.get();
                    let presence = hearth.presence.get();

                    view! {
                        <div class="home-living">
                            <header class="home-header">
                                <h1 class="home-name">
                                    {move || {
                                        hearth
                                            .current_hearth
                                            .get()
                                            .map(|h| h.name.clone())
                                            .unwrap_or_else(|| "No Hearth".into())
                                    }}
                                </h1>
                                {move || {
                                    let prefs = crate::hearth_prefs::use_hearth_prefs();
                                    let motto = prefs.motto.get();
                                    if motto.is_empty() {
                                        None
                                    } else {
                                        Some(view! {
                                            <p class="home-motto" style="font-style: italic; color: var(--text-secondary); letter-spacing: 0.08em; font-size: 0.95rem; margin-top: -0.5rem; opacity: 0.8;">
                                                {motto}
                                            </p>
                                        })
                                    }
                                }}
                            </header>

                            <UnvotedDecisionHomeAttention />

                            <div class="home-web">
                                <KinshipCanvas />
                            </div>

                            {move || {
                                let n = neglected_bonds();
                                (n > 0).then(|| {
                                    view! {
                                        <div class="nudge">
                                            {format!(
                                                "{} bond{} could use some warmth",
                                                n,
                                                if n > 1 { "s" } else { "" },
                                            )}
                                        </div>
                                    }
                                })
                            }}

                            <section class="presence-strip">
                                <h2>"who’s here"</h2>
                                <div class="presence-row">
                                    {presence
                                        .iter()
                                        .map(|p| {
                                            let name = member_name(&members, &p.agent);
                                            let role = members
                                                .iter()
                                                .find(|m| m.agent == p.agent)
                                                .map(|m| m.role.clone())
                                                .unwrap_or(MemberRole::Guest);
                                            let status = p.status.label().to_string();
                                            let css = match p.status {
                                                PresenceStatusType::Home => "presence-home",
                                                PresenceStatusType::Sleeping => "presence-sleeping",
                                                _ => "presence-away",
                                            };
                                            let avatar_name = name.clone();
                                            view! {
                                                <div class=format!("presence-chip {css}")>
                                                    <MemberAvatar name=avatar_name role=role size=28 />
                                                    <span class="presence-name">{name}</span>
                                                    <span class="presence-status">{status}</span>
                                                </div>
                                            }
                                        })
                                        .collect_view()}
                                </div>
                            </section>

                            {(!recent_gratitude.is_empty()).then(|| {
                                view! {
                                    <section class="recent-gratitude">
                                        <h2>"recent warmth"</h2>
                                        {recent_gratitude
                                            .iter()
                                            .take(3)
                                            .map(|g| {
                                                let from = member_name(&members, &g.from_agent);
                                                let to = member_name(&members, &g.to_agent);
                                                let msg = g.message.clone();
                                                view! {
                                                    <div class="gratitude-card">
                                                        <div class="gratitude-header">
                                                            <span class="gratitude-from">{from}</span>
                                                            <span class="gratitude-arrow">" → "</span>
                                                            <span class="gratitude-to">{to}</span>
                                                        </div>
                                                        <p class="gratitude-message">"“" {msg} "”"</p>
                                                    </div>
                                                }
                                            })
                                            .collect_view()}
                                    </section>
                                }
                            })}
                        </div>
                    }
                        .into_any()
                }
            }}
        </div>
    }
}
