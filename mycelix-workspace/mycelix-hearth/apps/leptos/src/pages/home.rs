// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::care_attention::{
    CareHomeAttention, care_attention_is_established_empty, use_care_attention,
};
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
    let care_attention = use_care_attention();
    let decision_attention = use_unvoted_decisions();

    let neglected_bonds = move || {
        hearth
            .bonds
            .get()
            .iter()
            .filter(|b| b.strength_bp < 4000)
            .count()
    };

    // Home's calm surface is deliberately scoped to the personal attention
    // sources it can actually establish. In live mode, both Care and Decision
    // attention must be established Empty and independently aligned with their
    // corresponding loaded snapshots. Unknown/unavailable/degraded evidence can
    // never collapse into a reassuring empty claim.
    //
    // Mock mode remains explicitly local/demo-only and derives the same narrow
    // personal projection from the mock records already visible on screen.
    let care_attention_for_calm = care_attention.clone();
    let decision_attention_for_calm = decision_attention.clone();
    let in_attention_calm = move || {
        let care = care_attention_for_calm.snapshot.get();
        let no_care_attention = if care.availability == AvailabilityStateKind::Mock {
            let my_agent = hearth.my_agent.get();
            !hearth.care_schedules.get().iter().any(|schedule| {
                schedule.status == CareScheduleStatus::Active && schedule.assigned_to == my_agent
            })
        } else {
            care_attention_is_established_empty(&care)
        };

        let decisions = decision_attention_for_calm.snapshot.get();
        let no_decision_attention = if decisions.availability == AvailabilityStateKind::Mock {
            let my_agent = hearth.my_agent.get();
            let votes = hearth.votes.get();
            !hearth.decisions.get().iter().any(|decision| {
                decision.status == DecisionStatus::Open
                    && !votes.iter().any(|vote| {
                        vote.decision_hash == decision.hash && vote.voter == my_agent
                    })
            })
        } else {
            attention_is_established_empty(&decisions)
        };

        no_care_attention && no_decision_attention
    };

    view! {
        <div class="page home-page">
            {move || {
                if in_attention_calm() {
                    view! {
                        <div class="homeostatic-void">
                            <p class="void-message">"a quiet moment"</p>
                            <p class="void-sub">
                                "no Care or Decision attention is currently surfaced for you in this Hearth."
                            </p>
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

                            <CareHomeAttention />
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
