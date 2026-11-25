// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Jobs page — search, browse, and post job opportunities.
//! Job matching runs client-side using the matching engine.
//!
//! Live job retrieval and the complete posting editor are not yet connected.
//! Sample postings are displayed only in explicit local mock/demo mode.

use leptos::prelude::*;
use std::collections::HashSet;
use wasm_bindgen::JsCast;

use mycelix_leptos_core::holochain_provider::use_holochain;

use crate::matching::{
    rank_jobs, EducationRequirementStatus, JobMatch, JobView, LegacyProfileSignalStatus,
};

/// Return a set of mock job postings spanning diverse skill domains.
fn mock_jobs() -> Vec<JobView> {
    vec![
        JobView {
            title: "Rust / Holochain Engineer".into(),
            organization: "Luminous Dynamics".into(),
            required_skills: vec!["rust".into(), "holochain".into(), "wasm".into()],
            preferred_skills: vec!["leptos".into(), "nix".into()],
            education_level: Some("Bachelor's".into()),
            remote_ok: true,
            vitality_minimum: Some(700),
        },
        JobView {
            title: "Data Science Lead".into(),
            organization: "Mycelix Foundation".into(),
            required_skills: vec!["python".into(), "data-science".into(), "machine-learning".into()],
            preferred_skills: vec!["rust".into(), "sql".into()],
            education_level: Some("Master's".into()),
            remote_ok: true,
            vitality_minimum: None,
        },
        JobView {
            title: "Cybersecurity Analyst".into(),
            organization: "NixForHumanity".into(),
            required_skills: vec!["cybersecurity".into(), "linux".into(), "networking".into()],
            preferred_skills: vec!["nix".into(), "rust".into(), "python".into()],
            education_level: None,
            remote_ok: false,
            vitality_minimum: None,
        },
        JobView {
            title: "Full-Stack Web Developer".into(),
            organization: "Resontia Labs".into(),
            required_skills: vec!["typescript".into(), "react".into(), "node".into()],
            preferred_skills: vec!["rust".into(), "wasm".into(), "holochain".into()],
            education_level: None,
            remote_ok: true,
            vitality_minimum: None,
        },
        JobView {
            title: "Distributed Systems Researcher".into(),
            organization: "Symthaea Institute".into(),
            required_skills: vec!["rust".into(), "distributed-systems".into()],
            preferred_skills: vec![
                "holochain".into(),
                "cryptography".into(),
                "python".into(),
                "data-science".into(),
            ],
            education_level: Some("PhD".into()),
            remote_ok: true,
            vitality_minimum: Some(700),
        },
    ]
}


#[component]
pub fn JobsPage() -> impl IntoView {
    let hc = use_holochain();
    let is_mock = hc.is_mock();

    let (search_query, set_search_query) = signal(String::new());
    let (skills_input, set_skills_input) = signal(String::from("rust, holochain"));

    // Parse comma-separated skills into a HashSet, updated reactively.
    let agent_skills = Memo::new(move |_| {
        skills_input
            .get()
            .split(',')
            .map(|s| s.trim().to_lowercase())
            .filter(|s| !s.is_empty())
            .collect::<HashSet<String>>()
    });

    // Sample postings are visible only in an explicitly marked local demo.
    // Connected mode stays empty until a real indexed read API is wired.
    let jobs = if is_mock { mock_jobs() } else { Vec::new() };
    let empty_jobs_message = if is_mock {
        "No demo jobs match these declared skills. Try adding or removing skills."
    } else {
        "Live job retrieval is not connected yet. No sample postings are shown in Live mode."
    };

    // Rank jobs whenever agent_skills changes.
    let matched_jobs = Memo::new(move |_| {
        let skills = agent_skills.get();
        let query = search_query.get().to_lowercase();
        let mut ranked = rank_jobs(&jobs, &skills, None, None, 20);
        // Additionally filter by free-text search if present.
        if !query.is_empty() {
            ranked.retain(|m| {
                m.job.title.to_lowercase().contains(&query)
                    || m.job.organization.to_lowercase().contains(&query)
                    || m.job
                        .required_skills
                        .iter()
                        .any(|s| s.contains(&query))
                    || m.job
                        .preferred_skills
                        .iter()
                        .any(|s| s.contains(&query))
            });
        }
        ranked
    });

    let hc_for_status = hc.clone();

    view! {
        <div class="page jobs-page">
            <h1>"Job Opportunities"</h1>

            // Conductor status
            <div class="conductor-status">
                {move || if hc_for_status.is_mock() {
                    view! { <span class="status-indicator mock">"Local demo -- matching engine active"</span> }.into_any()
                } else {
                    view! { <span class="status-indicator connected">"Connected to conductor"</span> }.into_any()
                }}
            </div>

            // Skills input for the matching engine
            <div class="skills-input-section">
                <label for="agent-skills"><strong>"Your declared skills"</strong>" (comma-separated)"</label>
                <input
                    id="agent-skills"
                    type="text"
                    placeholder="e.g. rust, holochain, python, data-science"
                    class="search-input"
                    aria-label="Your skills for job matching"
                    prop:value=move || skills_input.get()
                    on:input=move |ev| {
                        let input: web_sys::HtmlInputElement = ev.target().unwrap().dyn_into().unwrap();
                        set_skills_input.set(input.value());
                    }
                />
                <p class="text-secondary">
                    {move || {
                        let count = agent_skills.get().len();
                        format!("{count} self-declared skill{} active", if count == 1 { "" } else { "s" })
                    }}
                </p>
                <p class="text-secondary">
                    "Prototype ranking uses self-declared skill labels and declared education only. It is an advisory search aid, not verified competence, hiring eligibility or a hiring-success probability."
                </p>
            </div>

            // Free-text search bar
            <div class="search-bar">
                <input
                    type="text"
                    placeholder="Filter by keyword (title, org, skill)..."
                    class="search-input"
                    aria-label="Search jobs by keyword"
                    prop:value=move || search_query.get()
                    on:input=move |ev| {
                        let input: web_sys::HtmlInputElement = ev.target().unwrap().dyn_into().unwrap();
                        set_search_query.set(input.value());
                    }
                />
            </div>

            // Matched jobs grid
            <div class="jobs-grid">
                {move || {
                    let matches = matched_jobs.get();
                    if matches.is_empty() {
                        view! {
                            <div class="placeholder-card">
                                <p>{empty_jobs_message}</p>
                            </div>
                        }
                        .into_any()
                    } else {
                        view! {
                            <div class="matched-jobs-list">
                                {matches
                                    .into_iter()
                                    .map(job_card_view)
                                    .collect::<Vec<_>>()}
                            </div>
                        }
                        .into_any()
                    }
                }}
            </div>

            // Posting is intentionally disabled until the real editor and
            // coordinator contract are wired. Do not write placeholder jobs.
            <div class="post-job-section">
                <h3>"Post a Job"</h3>
                <p class="text-secondary">
                    "The complete posting form and live indexed retrieval are not connected yet. Craft will not submit a generic placeholder posting."
                </p>
                <button
                    class="btn-primary"
                    disabled
                    title="Job posting form is not implemented; no placeholder will be written"
                >
                    "Posting Form Not Available Yet"
                </button>
            </div>
        </div>
    }
}

/// Render a job card with explicit provenance and non-authoritative status.
fn job_card_view(m: JobMatch) -> impl IntoView {
    let pct = (m.score * 100.0).round() as u32;
    let pct_label = format!("{}% advisory profile heuristic", pct);
    let bar_width = format!("{}%", pct);
    let location = if m.job.remote_ok { "Remote" } else { "On-site" };
    let matched_required = m.matched_required.clone();
    let matched_preferred = m.matched_preferred.clone();

    let education_message = match m.education_status {
        EducationRequirementStatus::NotRequired => None,
        EducationRequirementStatus::NotProvided => {
            Some("Education not declared; no ranking bonus applied".to_owned())
        }
        EducationRequirementStatus::DeclaredMatch => {
            Some("Declared education label matches; issuer verification has not run".to_owned())
        }
        EducationRequirementStatus::DeclaredMismatch => {
            Some("Declared education label differs; no equivalency is inferred".to_owned())
        }
    };

    let legacy_signal_message = match m.legacy_profile_signal_status {
        LegacyProfileSignalStatus::NotRequired => None,
        LegacyProfileSignalStatus::NotEvaluated => {
            Some("Legacy profile signal not evaluated; this does not exclude the candidate".to_owned())
        }
        LegacyProfileSignalStatus::BelowLegacyThreshold => {
            Some("Legacy profile signal is below the posting value; this does not exclude the candidate".to_owned())
        }
        LegacyProfileSignalStatus::MeetsLegacyThreshold => {
            Some("Legacy profile signal meets the posting value; this is not proof of competence".to_owned())
        }
    };

    view! {
        <div class="job-card">
            <div class="job-header">
                <h3>{m.job.title.clone()}</h3>
                <span class="match-badge">{pct_label}</span>
            </div>
            <p class="job-org">{m.job.organization.clone()}</p>

            <div
                class="score-bar-bg"
                role="progressbar"
                aria-label="Advisory profile heuristic, not a hiring probability"
                aria-valuenow=pct
                aria-valuemin=0
                aria-valuemax=100
            >
                <div class="score-bar-fill" style:width=bar_width></div>
            </div>

            <div class="job-skills">
                <span class="skill-label">"Required: "</span>
                {m.job
                    .required_skills
                    .iter()
                    .map(|skill| {
                        let matched = matched_required.iter().any(|value| {
                            value == &skill.trim().to_lowercase()
                        });
                        let class = if matched { "skill-tag matched" } else { "skill-tag" };
                        view! { <span class=class>{skill.clone()}</span> }
                    })
                    .collect::<Vec<_>>()}
            </div>

            {if !m.job.preferred_skills.is_empty() {
                Some(
                    view! {
                        <div class="job-skills">
                            <span class="skill-label">"Preferred: "</span>
                            {m.job
                                .preferred_skills
                                .iter()
                                .map(|skill| {
                                    let matched = matched_preferred.iter().any(|value| {
                                        value == &skill.trim().to_lowercase()
                                    });
                                    let class = if matched {
                                        "skill-tag preferred matched"
                                    } else {
                                        "skill-tag preferred"
                                    };
                                    view! { <span class=class>{skill.clone()}</span> }
                                })
                                .collect::<Vec<_>>()}
                        </div>
                    },
                )
            } else {
                None
            }}

            <div class="job-meta">
                <span>{location}</span>
                {m.job
                    .education_level
                    .as_ref()
                    .map(|education| {
                        view! { <span>{format!("Education requirement: {education}")}</span> }
                    })}
                {education_message.map(|message| {
                    view! { <span class="education-status">{message}</span> }
                })}
                {legacy_signal_message.map(|message| {
                    view! { <span class="legacy-profile-signal">{message}</span> }
                })}
            </div>

            <button class="btn-primary" disabled title="Live application flow is not implemented yet">
                "Apply (not available yet)"
            </button>
        </div>
    }
}
