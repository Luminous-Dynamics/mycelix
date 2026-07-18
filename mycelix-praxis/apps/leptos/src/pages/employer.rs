// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Employer talent-search concept.
//!
//! No employer-search wire contract exists in the supplied hApp. The concept
//! is therefore restricted to Demo mode and never presented as network data.

use leptos::prelude::*;

use crate::mode::{AppMode, use_app_mode};

#[component]
pub fn EmployerPortal() -> impl IntoView {
    let mode = use_app_mode();

    view! {
        {move || {
            if mode.get() == AppMode::Demo {
                view! { <EmployerConcept /> }.into_any()
            } else {
                view! {
                    <div class="employer-portal">
                        <header class="employer-header">
                            <div class="portal-brand">
                                <span class="portal-icon">"\u{1F4BC}"</span>
                                <h1>"Praxis Talent Mesh"</h1>
                            </div>
                        </header>
                        <div class="data-empty-state">
                            <strong>"Talent search unavailable"</strong>
                            <p>
                                "No employer-search coordinator contract is connected. Praxis is not claiming candidates, reputation scores, ZK verification, or mesh-search results."
                            </p>
                        </div>
                    </div>
                }.into_any()
            }
        }}
    }
}

#[component]
fn EmployerConcept() -> impl IntoView {
    view! {
        <div class="employer-portal">
            <div class="data-empty-state" style="margin-bottom: 1rem">
                <strong>"Demo-only talent concept"</strong>
                <p>
                    "Every candidate, identifier, score, skill, and proof badge below is fictional. No mesh search or credential verification is running."
                </p>
            </div>

            <header class="employer-header">
                <div class="portal-brand">
                    <span class="portal-icon">"\u{1F4BC}"</span>
                    <h1>"Praxis Talent Mesh Concept"</h1>
                </div>
                <div class="mesh-status">"Illustrative offline layout"</div>
            </header>

            <section class="search-hero">
                <div class="search-container">
                    <div class="search-input-wrapper">
                        <span class="search-icon">"\u{1F50D}"</span>
                        <input
                            type="text"
                            value="2512"
                            aria-label="Illustrative ESCO search"
                            disabled
                        />
                    </div>
                    <button class="btn-search" disabled>"Search unavailable"</button>
                </div>
            </section>

            <main class="results-layout">
                <aside class="search-filters">
                    <h4>"Illustrative Parameters"</h4>
                    <div class="filter-group">
                        <label>"Framework"</label>
                        <select disabled><option>"ESCO example"</option></select>
                    </div>
                    <div class="filter-group">
                        <label>"Illustrative trust threshold"</label>
                        <input type="range" min="0" max="1000" step="50" disabled />
                    </div>
                </aside>

                <div class="results-main">
                    <div class="results-header">
                        <h3>"Fictional Candidate Cards"</h3>
                    </div>
                    <div class="results-grid">
                        <TalentConceptCard
                            did="did:example:praxis-candidate-a"
                            trust=880
                            skills=vec!["Software Developer", "Systems Architect"]
                        />
                        <TalentConceptCard
                            did="did:example:praxis-candidate-b"
                            trust=740
                            skills=vec!["Software Developer"]
                        />
                    </div>
                </div>
            </main>
        </div>
    }
}

#[component]
fn TalentConceptCard(did: &'static str, trust: u16, skills: Vec<&'static str>) -> impl IntoView {
    view! {
        <div class="talent-card">
            <div class="talent-header">
                <div class="talent-meta">
                    <span class="talent-did">{did}</span>
                    <div class="talent-skills">
                        {skills
                            .into_iter()
                            .map(|skill| view! { <span class="skill-tag">{skill}</span> })
                            .collect_view()}
                    </div>
                </div>
                <div class="talent-scores">
                    <div class="score-pill">
                        <span class="score-label">"Illustrative score"</span>
                        <span class="score-value">{trust}</span>
                    </div>
                </div>
            </div>

            <div class="talent-verification">
                <div class="zk-badge">
                    <span class="zk-icon">"\u{1F512}"</span>
                    "Concept badge — no ZK proof checked"
                </div>
            </div>

            <div class="talent-actions">
                <button class="btn-sm btn-primary" disabled>"CLR request unavailable"</button>
                <button class="btn-sm btn-outline" disabled>"Capstone unavailable"</button>
            </div>
        </div>
    }
}
