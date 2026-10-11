// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Source-backed model inventory for the Economy Atlas.
//!
//! This page only browses the versioned registry. It deliberately does not execute a
//! model or elevate a paper-reported result into a qualified simulation.

use leptos::prelude::*;
use serde::Deserialize;
use serde_json::Value;
use std::collections::BTreeMap;

const REGISTRY_JSON: &str =
    include_str!("../../../../../docs/economy-atlas/model-registry.v1.json");

#[derive(Clone, Debug, Deserialize)]
struct AtlasRegistry {
    registry_status: String,
    qualified_for_execution: bool,
    generated_from: AtlasGeneratedFrom,
    models: Vec<AtlasModel>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasGeneratedFrom {
    pinned_source_snapshots: BTreeMap<String, String>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasModel {
    model_id: String,
    title: String,
    domain: Vec<String>,
    epistemic_status: String,
    execution_status: String,
    #[serde(default)]
    source_refs: Vec<AtlasSource>,
    #[serde(default)]
    limitations: Vec<String>,
    #[serde(default)]
    validation_findings: Vec<AtlasValidationFinding>,
    #[serde(flatten)]
    details: BTreeMap<String, Value>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasValidationFinding {
    finding_id: String,
    severity: String,
    source_repository: String,
    source_file: String,
    source_commit: String,
    observation: String,
    required_follow_up: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasSource {
    repository: String,
    path: String,
    #[serde(default, rename = "ref")]
    ref_: String,
    url: String,
}

fn evidence_label(status: &str) -> &'static str {
    match status {
        "implemented_source" => "Implementation source exists",
        "paper_reported" => "Paper-reported",
        "design_only" => "Design-only",
        "conceptual" => "Conceptual",
        "discovery_required" => "Discovery required",
        _ => "Unknown status",
    }
}

fn evidence_class(status: &str) -> &'static str {
    match status {
        "implemented_source" => "atlas-evidence-implemented",
        "paper_reported" => "atlas-evidence-paper",
        "design_only" => "atlas-evidence-design",
        "conceptual" => "atlas-evidence-concept",
        "discovery_required" => "atlas-evidence-unknown",
        _ => "atlas-evidence-unknown",
    }
}

fn display_registry_value(value: &Value) -> String {
    value
        .as_str()
        .map(str::to_owned)
        .unwrap_or_else(|| {
            serde_json::to_string_pretty(value).unwrap_or_else(|_| value.to_string())
        })
}

fn render_model_card(model: AtlasModel) -> impl IntoView {
    let model_id = model.model_id.clone();
    let title = model.title.clone();
    let domain = model.domain.join(" · ");
    let status_label = evidence_label(&model.epistemic_status);
    let status_class = evidence_class(&model.epistemic_status);
    let execution_status = model.execution_status.replace('_', " ");
    let limitations = model.limitations;
    let source_refs = model.source_refs;
    let validation_findings = model.validation_findings;
    let details = model.details;

    view! {
        <article class="dash-card atlas-model-card">
            <div class="atlas-card-top">
                <span class=format!("badge {status_class}")>{status_label}</span>
                <span class="atlas-domain">{domain}</span>
            </div>
            <h2>{title}</h2>
            <p class="atlas-model-id">{model_id.clone()}</p>
            <p>
                <strong>"Execution: "</strong>
                {execution_status}
            </p>
            <details>
                <summary>"Assumptions, limitations and source files"</summary>
                <p class="atlas-model-id">{model_id}</p>
                {if details.is_empty() {
                    view! { <></> }.into_any()
                } else {
                    view! {
                        <section class="atlas-model-metadata">
                            <h3>"Model contract, parameters and reported outputs"</h3>
                            <p class="atlas-disclaimer">
                                "These are fields recorded in the pinned discovery registry. They are not a substitute for executing the source model."
                            </p>
                            {details.into_iter().map(|(key, value)| {
                                let label = key.replace('_', " ");
                                let value_text = display_registry_value(&value);
                                view! {
                                    <div class="atlas-metadata-row">
                                        <strong>{label}</strong>
                                        <pre>{value_text}</pre>
                                    </div>
                                }
                            }).collect_view()}
                        </section>
                    }.into_any()
                }}
                <h3>"Known limitations / open questions"</h3>
                <ul>
                    {limitations.into_iter().map(|limitation| view! {
                        <li>{limitation}</li>
                    }).collect_view()}
                </ul>
                {if validation_findings.is_empty() {
                    view! { <></> }.into_any()
                } else {
                    view! {
                        <section class="atlas-validation-findings">
                            <h3>"Source-level validation findings"</h3>
                            <p class="atlas-disclaimer">
                                "These findings identify code-level conditions to verify at call sites; presence in the registry does not prove an invalid path is reachable in production."
                            </p>
                            {validation_findings.into_iter().map(|finding| {
                                let finding_id = finding.finding_id;
                                let severity = finding.severity;
                                let source_repository = finding.source_repository;
                                let source_file = finding.source_file;
                                let source_commit = finding.source_commit;
                                let observation = finding.observation;
                                let follow_up = finding.required_follow_up;
                                view! {
                                    <div class="dash-card atlas-finding-card">
                                        <strong>{finding_id}</strong>
                                        <p>{severity}</p>
                                        <p>{observation}</p>
                                        <p>
                                            <strong>"Required follow-up: "</strong>
                                            {follow_up}
                                        </p>
                                        <p class="atlas-model-id">{format!("{source_repository}/{source_file} @ {source_commit}")}</p>
                                    </div>
                                }
                            }).collect_view()}
                        </section>
                    }.into_any()
                }}
                <h3>"Source references"</h3>
                {if source_refs.is_empty() {
                    view! {
                        <p>
                            "No canonical source resolved yet; this absence is not a claim the model does not exist."
                        </p>
                    }.into_any()
                } else {
                    source_refs.into_iter().map(|source| {
                        let short_ref = source.ref_.chars().take(12).collect::<String>();
                        let label = format!("{} · {} · {}", source.repository, source.path, short_ref);
                        let url = source.url;
                        view! {
                            <p class="atlas-source-link">
                                <a href=url target="_blank" rel="noopener noreferrer">{label}</a>
                            </p>
                        }
                    }).collect_view().into_any()
                }}
                <p class="atlas-disclaimer">
                    "A source link establishes traceability, not model correctness, runtime execution or empirical validity."
                </p>
            </details>
        </article>
    }
}

#[component]
pub fn EconomyAtlasPage() -> impl IntoView {
    let registry = match serde_json::from_str::<AtlasRegistry>(REGISTRY_JSON) {
        Ok(registry) => registry,
        Err(error) => {
            return view! {
                <section class="page-economy-atlas" role="alert">
                    <h1>"Economy Atlas unavailable"</h1>
                    <p class="subtitle">
                        "The versioned model registry could not be read. No fallback model data was substituted."
                    </p>
                    <pre>{error.to_string()}</pre>
                </section>
            }
            .into_any();
        }
    };

    let model_count = registry.models.len();
    let source_count: usize = registry.models.iter().map(|model| model.source_refs.len()).sum();
    let source_snapshots = registry
        .generated_from
        .pinned_source_snapshots
        .iter()
        .map(|(repository, commit)| format!("{repository} @ {commit}"))
        .collect::<Vec<_>>()
        .join(" · ");
    let registry_status = registry.registry_status.clone();
    let execution_qualified = registry.qualified_for_execution;
    let models_for_view = registry.models;
    let (query, set_query) = signal(String::new());
    let (status_filter, set_status_filter) = signal("all".to_string());
    let filtered_models = Memo::new(move |_| {
        let term = query.get().trim().to_lowercase();
        let selected_status = status_filter.get();
        models_for_view
            .iter()
            .filter(|model| {
                let metadata = serde_json::to_string(&model.details).unwrap_or_default();
                let source_text = model.source_refs.iter()
                    .map(|source| format!("{} {} {}", source.repository, source.path, source.url))
                    .collect::<Vec<_>>()
                    .join(" ");
                let finding_text = model.validation_findings.iter()
                    .map(|finding| format!("{} {} {}", finding.finding_id, finding.observation, finding.required_follow_up))
                    .collect::<Vec<_>>()
                    .join(" ");
                let haystack = format!(
                    "{} {} {} {} {} {} {} {}",
                    model.model_id,
                    model.title,
                    model.domain.join(" "),
                    model.epistemic_status,
                    model.execution_status,
                    model.limitations.join(" "),
                    metadata,
                    format!("{} {}", source_text, finding_text)
                )
                .to_lowercase();
                (term.is_empty() || haystack.contains(&term))
                    && (selected_status == "all" || model.epistemic_status == selected_status)
            })
            .cloned()
            .collect::<Vec<_>>()
    });
    let filtered_for_count = filtered_models.clone();
    let filtered_for_view = filtered_models.clone();

    view! {
        <section class="page-economy-atlas">
            <h1>"Economy Atlas"</h1>
            <p class="subtitle">
                "Explore the models, assumptions, source evidence and unresolved questions behind Mycelix and Symthaea economics."
            </p>

            <div class="availability-state" role="status">
                <strong>"Registry status: "</strong>
                <span>{registry_status}</span>
                <p>
                    "This is a source-backed discovery inventory—not a claim that every model is runnable, empirically validated, or ready for policy use."
                </p>
            </div>

            <div class="dashboard-grid">
                <div class="dash-card">
                    <span class="dash-label">"Registered model families"</span>
                    <span class="dash-value">{model_count.to_string()}</span>
                </div>
                <div class="dash-card">
                    <span class="dash-label">"Pinned source references"</span>
                    <span class="dash-value">{source_count.to_string()}</span>
                </div>
                <div class="dash-card">
                    <span class="dash-label">"Execution qualification"</span>
                    <span class="dash-value">{if execution_qualified { "Qualified" } else { "Not qualified" }}</span>
                </div>
            </div>

            <div class="dash-card" style="margin: 1rem 0;">
                <h2>"Source snapshot"</h2>
                <p class="subtitle">
                    "Registry source snapshots are pinned to immutable Git commits; use each model's source links to inspect its exact file revision."
                </p>
                <p class="atlas-snapshots">{source_snapshots}</p>
            </div>

            <div class="atlas-filter-row">
                <div class="form-field" style="margin: 1rem 0;">
                    <label for="economy-atlas-search">"Search model names, equations, source paths, results and limitations"</label>
                    <input
                        id="economy-atlas-search"
                        class="form-input"
                        type="search"
                        placeholder="e.g. demurrage, market, credit, energy…"
                        prop:value=move || query.get()
                        on:input=move |event| set_query.set(event_target_value(&event))
                    />
                </div>
                <div class="form-field" style="margin: 1rem 0;">
                    <label for="economy-atlas-evidence-filter">"Evidence class"</label>
                    <select
                        id="economy-atlas-evidence-filter"
                        class="form-input"
                        prop:value=move || status_filter.get()
                        on:change=move |event| set_status_filter.set(event_target_value(&event))
                    >
                        <option value="all">"All evidence classes"</option>
                        <option value="implemented_source">"Implementation source"</option>
                        <option value="paper_reported">"Paper-reported"</option>
                        <option value="design_only">"Design-only"</option>
                        <option value="conceptual">"Conceptual"</option>
                        <option value="discovery_required">"Discovery required"</option>
                    </select>
                </div>
            </div>
            <p class="subtitle" aria-live="polite">
                {move || format!("Showing {} of {} model families", filtered_for_count.get().len(), model_count)}
            </p>

            <div class="dashboard-grid atlas-model-grid">
                {move || {
                    let filtered = filtered_for_view.get();
                    if filtered.is_empty() {
                        view! {
                            <p class="subtitle">"No models match these filters."</p>
                        }
                        .into_any()
                    } else {
                        filtered.into_iter().map(render_model_card).collect_view().into_any()
                    }
                }}
            </div>

            <div class="dash-card" style="margin-top: 1rem;">
                <h2>"Comparison rules"</h2>
                <ul>
                    <li>"Compare outputs only when units, metric meaning, time horizon and population scope are compatible."</li>
                    <li>"Keep the three currency contracts distinct: Mycelix Finance TEND mutual credit, the Essay 16 conceptual TEND, and Symthaea's production/entropy ledger are not the same accounting model."</li>
                    <li>"Paper-reported results and conceptual proposals are visible for exploration but are not labeled as executed or qualified."</li>
                    <li>"Missing inputs stay unknown; this page does not mutate live finance state."</li>
                </ul>
            </div>
        </section>
    }
    .into_any()
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn registry() -> AtlasRegistry {
        serde_json::from_str(REGISTRY_JSON).expect("checked-in Economy Atlas registry must parse")
    }

    #[test]
    fn registry_is_visible_but_not_execution_qualified() {
        let registry = registry();
        assert_eq!(registry.registry_status, "initial_discovery_inventory");
        assert!(!registry.qualified_for_execution);
        assert_eq!(registry.models.len(), 9);
    }

    #[test]
    fn model_ids_are_unique_and_currency_contracts_are_separate() {
        let registry = registry();
        let ids = registry
            .models
            .iter()
            .map(|model| model.model_id.as_str())
            .collect::<HashSet<_>>();
        assert_eq!(ids.len(), registry.models.len());

        for required in [
            "mycelix.finance.tricurrency.v1",
            "symthaea.economy.thermodynamic-ledger.v1",
            "symthaea.concepts.tricurrency.essay16.v1",
        ] {
            assert!(ids.contains(required), "missing distinct currency variant: {required}");
        }
    }

    #[test]
    fn domain_contracts_and_paper_results_are_exposed_to_the_explorer() {
        let registry = registry();
        let find_model = |id: &str| {
            registry.models.iter().find(|model| model.model_id == id)
                .unwrap_or_else(|| panic!("missing registry model: {id}"))
        };

        assert!(find_model("mycelix.finance.tricurrency.v1").details.contains_key("currency_contract"));
        assert!(find_model("symthaea.economy.thermodynamic-ledger.v1").details.contains_key("documented_equations"));
        assert!(find_model("symthaea.energy.lcoe.v1").details.contains_key("documented_equation"));
        assert!(find_model("symthaea.governance.tricurrency.paper-results.v1").details.contains_key("paper_reported_results"));
    }

    #[test]
    fn every_registered_source_permalink_is_pinned_and_self_consistent() {
        let registry = registry();
        let mut sources = 0usize;
        for model in &registry.models {
            for source in &model.source_refs {
                sources += 1;
                assert_eq!(
                    source.ref_.len(),
                    40,
                    "source ref is not a Git SHA: {}",
                    model.model_id
                );
                assert!(source.ref_.bytes().all(|byte| byte.is_ascii_hexdigit()));
                assert_eq!(
                    source.ref_,
                    source.ref_.to_ascii_lowercase(),
                    "source ref must use lowercase hexadecimal: {}",
                    source.ref_
                );
                let expected_url = format!(
                    "https://github.com/{}/blob/{}/{}",
                    source.repository,
                    source.ref_,
                    source.path
                );
                assert_eq!(
                    source.url, expected_url,
                    "source URL must exactly match repository/path/commit"
                );
            }
        }
        assert_eq!(sources, 12);
    }

    #[test]
    fn every_validation_finding_binds_to_an_exact_registered_source() {
        let registry = registry();
        let mut count = 0usize;
        for model in &registry.models {
            for finding in &model.validation_findings {
                count += 1;
                assert!(
                    model.source_refs.iter().any(|source| {
                        source.repository == finding.source_repository
                            && source.path == finding.source_file
                            && source.ref_ == finding.source_commit
                    }),
                    "finding {} is not bound to a model source permalink",
                    finding.finding_id
                );
                assert!(!finding.observation.trim().is_empty());
                assert!(!finding.required_follow_up.trim().is_empty());
            }
        }
        assert_eq!(count, 4);
    }
}
