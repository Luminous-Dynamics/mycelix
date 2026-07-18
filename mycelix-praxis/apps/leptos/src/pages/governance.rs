// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Isibaya governance concept.
//!
//! The supplied DAO coordinator does not expose the merit-steering, hardware
//! grant, or prediction-market contract assumed by this page. Those ideas are
//! shown only as a disabled Demo-mode concept.

use leptos::prelude::*;

use crate::curriculum::curriculum_graph;
use crate::mode::{AppMode, use_app_mode};

#[component]
pub fn GovernancePage() -> impl IntoView {
    let mode = use_app_mode();

    view! {
        {move || {
            if mode.get() == AppMode::Demo {
                view! { <GovernanceConcept /> }.into_any()
            } else {
                view! {
                    <div class="governance-page">
                        <header class="gov-header">
                            <h2>"Isibaya Governance"</h2>
                        </header>
                        <div class="data-empty-state">
                            <strong>"Governance interface unavailable"</strong>
                            <p>
                                "No merit-steering, hardware-grant, or prediction-market UI contract is connected. No proposals, votes, stakes, or payouts are claimed."
                            </p>
                        </div>
                    </div>
                }.into_any()
            }
        }}
    }
}

#[component]
fn GovernanceConcept() -> impl IntoView {
    view! {
        <div class="governance-page">
            <div class="data-empty-state" style="margin-bottom: 1rem">
                <strong>"Demo-only governance concept"</strong>
                <p>
                    "All proposals, multipliers, percentages, markets, stakes, and outcomes below are fictional and inactive. Every control is disabled."
                </p>
            </div>

            <header class="gov-header">
                <h2>"Isibaya Governance Concept"</h2>
                <p class="subtitle">"Illustrative community-control layout."</p>
            </header>

            <div class="gov-grid">
                <section class="merit-steering">
                    <h3>"Illustrative Merit Weighting"</h3>
                    <p style="font-size: 0.8rem; color: var(--text-tertiary)">
                        "Example curriculum tracks derived from the local curriculum catalogue."
                    </p>

                    <div class="steering-list" style="margin-top: 1.5rem">
                        {curriculum_graph()
                            .subjects()
                            .into_iter()
                            .take(5)
                            .map(|subject| view! {
                                <div class="steering-row" style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 1rem; padding: 1rem; background: var(--surface-low); border-radius: 8px">
                                    <span style="font-weight: 600">{subject}</span>
                                    <div style="display: flex; align-items: center; gap: 1rem">
                                        <span style="font-family: monospace">"Example 1.0x"</span>
                                        <button class="btn-sm btn-outline" disabled>"Concept vote"</button>
                                    </div>
                                </div>
                            })
                            .collect_view()}
                    </div>
                </section>

                <section class="hardware-council">
                    <h3>"Hardware Council Concept"</h3>
                    <div class="council-card" style="padding: 1.5rem; background: var(--surface-high); border-radius: 12px; border: 1px solid var(--accent-low)">
                        <div style="font-size: 0.7rem; color: var(--accent); font-weight: 800; text-transform: uppercase">
                            "Illustrative proposal"
                        </div>
                        <h4 style="margin: 0.5rem 0">"Example: Warehouse CNC access"</h4>
                        <p style="font-size: 0.8rem; line-height: 1.5">
                            "Could demonstrated fabrication mastery participate in a future capability-grant policy?"
                        </p>
                        <div style="display: flex; gap: 0.5rem; margin-top: 1rem">
                            <button class="btn-sm btn-primary" style="flex: 1" disabled>"Example AYE 82%"</button>
                            <button class="btn-sm btn-outline" style="flex: 1" disabled>"Example NAY 18%"</button>
                        </div>
                    </div>
                </section>

                <section class="epistemic-markets" style="grid-column: span 2; margin-top: 2rem; padding: 2rem; background: var(--surface-high); border-radius: 12px; border: 1px solid var(--primary-low)">
                    <div style="display: flex; justify-content: space-between; align-items: center">
                        <h3>"Truth-Staking Concept"</h3>
                        <span class="badge">"Inactive illustration"</span>
                    </div>
                    <p class="subtitle">
                        "A hypothetical interface for reputation-weighted predictions; no staking or settlement exists here."
                    </p>

                    <div class="market-card" style="margin-top: 1.5rem; padding: 1.5rem; background: var(--surface); border-radius: 8px; border: 1px solid var(--border)">
                        <div style="display: flex; justify-content: space-between; font-size: 0.7rem; text-transform: uppercase; color: var(--text-tertiary)">
                            <span>"Illustrative metric: WATER-TURBIDITY"</span>
                            <span>"Example horizon: 24h"</span>
                        </div>
                        <h4 style="margin: 0.5rem 0">"Example: Biomimetic Water Filter Prototype"</h4>
                        <p style="font-size: 0.85rem">
                            "Could a measured prototype reduce turbidity by more than 15%?"
                        </p>

                        <div class="prediction-distribution" style="margin: 1.5rem 0; height: 40px; display: flex; align-items: flex-end; gap: 2px">
                            {(0..20)
                                .map(|index| {
                                    let height = if index < 10 {
                                        index * 4
                                    } else {
                                        (20 - index) * 4
                                    };
                                    view! {
                                        <div style=format!("flex: 1; height: {}%; background: var(--primary-low); border-radius: 2px", height)></div>
                                    }
                                })
                                .collect_view()}
                        </div>

                        <div style="display: flex; gap: 1rem; align-items: center">
                            <input type="number" placeholder="Concept stake" disabled style="flex: 1; padding: 0.5rem; border-radius: 4px; border: 1px solid var(--border); background: var(--surface-low)" />
                            <button class="btn-primary" disabled>"Prediction unavailable"</button>
                        </div>
                        <div style="font-size: 0.6rem; color: var(--text-tertiary); margin-top: 0.8rem; text-align: center">
                            "No influence, payout, or reputation change is calculated."
                        </div>
                    </div>
                </section>
            </div>
        </div>
    }
}
