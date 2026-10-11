// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-first browser for the Economy Atlas discovery registry.
//!
//! This page intentionally does not execute economic models. Its registry
//! status is separate from execution qualification; a future runner adapter
//! must be added and qualified before a model can be run from the UI.

use leptos::prelude::*;
use serde::Deserialize;
use serde_json::Value;
use std::collections::BTreeMap;
#[cfg(test)]
use std::collections::HashSet;

const MODEL_REGISTRY_JSON: &str =
    include_str!("../../../../../docs/economy-atlas/model-registry.v1.json");

#[derive(Clone, Deserialize)]
struct AtlasRegistry {
    registry_id: String,
    schema_version: String,
    registry_status: String,
    qualified_for_execution: bool,
    models: Vec<AtlasModel>,
}

#[derive(Clone, Deserialize)]
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
    #[serde(flatten)]
    metadata: BTreeMap<String, Value>,
}

#[derive(Clone, Deserialize)]
struct AtlasSource {
    repository: String,
    path: String,
    #[serde(rename = "ref")]
    revision: String,
    url: String,
}

fn status_label(status: &str) -> &'static str {
    match status {
        "implemented_source" => "Implementation source",
        "paper_reported" => "Paper-reported",
        "design_only" => "Design-only",
        "conceptual" => "Conceptual",
        "discovery_required" => "Discovery required",
        _ => "Unclassified",
    }
}

fn status_css(status: &str) -> &'static str {
    match status {
        "implemented_source" => "atlas-status-implemented",
        "paper_reported" => "atlas-status-paper",
        "design_only" => "atlas-status-design",
        "conceptual" => "atlas-status-conceptual",
        "discovery_required" => "atlas-status-discovery",
        _ => "atlas-status-unknown",
    }
}

fn display_value(value: &Value) -> String {
    match value {
        Value::String(value) => value.clone(),
        Value::Array(values) if values.iter().all(Value::is_string) => values
            .iter()
            .filter_map(Value::as_str)
            .collect::<Vec<_>>()
            .join(", "),
        _ => serde_json::to_string_pretty(value).unwrap_or_else(|_| value.to_string()),
    }
}

fn searchable_text(model: &AtlasModel) -> String {
    let sources = model
        .source_refs
        .iter()
        .map(|source| format!("{} {} {}", source.repository, source.path, source.url))
        .collect::<Vec<_>>()
        .join(" ");
    let metadata = model
        .metadata
        .iter()
        .map(|(key, value)| format!("{key} {value}"))
        .collect::<Vec<_>>()
        .join(" ");

    format!(
        "{} {} {} {} {} {} {} {}",
        model.model_id,
        model.title,
        model.domain.join(" "),
        model.epistemic_status,
        model.execution_status,
        sources,
        model.limitations.join(" "),
        metadata
    )
    .to_lowercase()
}

#[component]
pub fn EconomyAtlasPage() -> impl IntoView {
    let registry_result = serde_json::from_str::<AtlasRegistry>(MODEL_REGISTRY_JSON);

    match registry_result {
        Err(error) => view! {
            <section class="economy-atlas-page" aria-labelledby="atlas-title">
                <h1 id="atlas-title">"Economy Atlas"</h1>
                <div class="atlas-alert atlas-alert-error" role="alert">
                    <strong>"The checked-in model registry could not be loaded."</strong>
                    <p>{error.to_string()}</p>
                    <p>"The catalogue has failed closed; no substitute or mock model data is shown."</p>
                </div>
            </section>
        }
        .into_any(),
        Ok(registry) => {
            let total = registry.models.len();
            let implemented = registry
                .models
                .iter()
                .filter(|model| model.epistemic_status == "implemented_source")
                .count();
            let paper_reported = registry
                .models
                .iter()
                .filter(|model| model.epistemic_status == "paper_reported")
                .count();
            let unresolved = registry
                .models
                .iter()
                .filter(|model| {
                    matches!(
                        model.epistemic_status.as_str(),
                        "design_only" | "conceptual" | "discovery_required"
                    )
                })
                .count();
            let registry_id = registry.registry_id.clone();
            let schema_version = registry.schema_version.clone();
            let registry_status = registry.registry_status.clone();
            let qualified = registry.qualified_for_execution;
            let models = registry.models;
            let (query, set_query) = signal(String::new());
            let (status_filter, set_status_filter) = signal("all".to_string());

            let filtered_models = Memo::new(move |_| {
                let needle = query.get().trim().to_lowercase();
                let selected_status = status_filter.get();
                models
                    .iter()
                    .filter(|model| {
                        (needle.is_empty() || searchable_text(model).contains(&needle))
                            && (selected_status == "all"
                                || model.epistemic_status == selected_status)
                    })
                    .cloned()
                    .collect::<Vec<_>>()
            });
            let filtered_for_count = filtered_models.clone();
            let filtered_for_list = filtered_models.clone();

            view! {
                <section class="economy-atlas-page" aria-labelledby="atlas-title">
                    <header class="atlas-header">
                        <div>
                            <p class="atlas-eyebrow">"MYCELIX · ECONOMIC MODEL REGISTRY"</p>
                            <h1 id="atlas-title">"Economy Atlas"</h1>
                            <p class="atlas-intro">
                                "Explore economic models, compare their assumptions, and inspect the evidence behind each one. A shared name does not imply a shared model."
                            </p>
                        </div>
                        <span class="atlas-registry-version">
                            {format!("Schema {}", schema_version)}
                        </span>
                    </header>

                    <div class="atlas-alert" role="note">
                        <strong>"Discovery registry — execution not qualified"</strong>
                        <p>
                            {format!(
                                "{} · {} · qualified_for_execution = {}",
                                registry_id, registry_status, qualified
                            )}
                        </p>
                        <p>
                            "This page displays registry metadata only. It does not run simulations, modify finance policy, or turn paper-reported results into reproduced evidence."
                        </p>
                    </div>

                    <div class="atlas-summary" aria-label="Registry summary">
                        <div class="atlas-summary-item">
                            <span class="atlas-summary-number">{total}</span>
                            <span class="atlas-summary-label">"Registered model families"</span>
                        </div>
                        <div class="atlas-summary-item">
                            <span class="atlas-summary-number">{implemented}</span>
                            <span class="atlas-summary-label">"Implementation sources found"</span>
                        </div>
                        <div class="atlas-summary-item">
                            <span class="atlas-summary-number">{paper_reported}</span>
                            <span class="atlas-summary-label">"Paper-reported families"</span>
                        </div>
                        <div class="atlas-summary-item">
                            <span class="atlas-summary-number">{unresolved}</span>
                            <span class="atlas-summary-label">"Design, conceptual or unresolved"</span>
                        </div>
                    </div>

                    <div class="atlas-controls">
                        <label class="atlas-search-label" for="atlas-search">"Search registry"</label>
                        <input
                            id="atlas-search"
                            class="atlas-search"
                            type="search"
                            placeholder="Search model names, equations, source paths, limitations…"
                            aria-label="Search model names, equations, source paths and limitations"
                            prop:value=move || query.get()
                            on:input=move |event| set_query.set(event_target_value(&event))
                        />
                        <label class="atlas-filter-label" for="atlas-status-filter">"Evidence class"</label>
                        <select
                            id="atlas-status-filter"
                            class="atlas-status-filter"
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
                        <span class="atlas-result-count" aria-live="polite">
                            {move || format!("Showing {} of {} models", filtered_for_count.get().len(), total)}
                        </span>
                    </div>

                    <div class="atlas-model-list" aria-label="Economic model catalogue">
                        <For
                            each=move || filtered_for_list.get()
                            key=|model| model.model_id.clone()
                            children=move |model: AtlasModel| {
                                let class = status_css(&model.epistemic_status);
                                let label = status_label(&model.epistemic_status);
                                let title = model.title.clone();
                                let model_id = model.model_id.clone();
                                let execution_status = model.execution_status.clone();
                                let domains = model.domain.join(" · ");
                                let source_refs = model.source_refs.clone();
                                let limitations = model.limitations.clone();
                                let metadata = model
                                    .metadata
                                    .iter()
                                    .map(|(key, value)| (key.clone(), value.clone()))
                                    .collect::<Vec<_>>();
                                let has_details =
                                    !source_refs.is_empty() || !limitations.is_empty() || !metadata.is_empty();

                                view! {
                                    <article class="atlas-model-card">
                                        <div class="atlas-model-heading">
                                            <div>
                                                <h2>{title}</h2>
                                                <code>{model_id}</code>
                                            </div>
                                            <span class=format!("atlas-status {class}")>{label}</span>
                                        </div>
                                        <p class="atlas-domain">{domains}</p>
                                        <p class="atlas-execution">
                                            <span>"Execution state"</span>
                                            <code>{execution_status}</code>
                                        </p>

                                        {if has_details {
                                            view! {
                                                <details class="atlas-model-details">
                                                    <summary>"Inspect sources, contracts and limitations"</summary>

                                                    {if !metadata.is_empty() {
                                                        view! {
                                                            <section class="atlas-detail-section">
                                                                <h3>"Registered model metadata"</h3>
                                                                <For
                                                                    each=move || metadata.clone()
                                                                    key=|(key, _)| key.clone()
                                                                    children=move |(key, value)| view! {
                                                                        <div class="atlas-metadata-row">
                                                                            <strong>{key.replace('_', " ")}</strong>
                                                                            <pre>{display_value(&value)}</pre>
                                                                        </div>
                                                                    }
                                                                />
                                                            </section>
                                                        }.into_any()
                                                    } else {
                                                        view! { <></> }.into_any()
                                                    }}

                                                    {if !source_refs.is_empty() {
                                                        view! {
                                                            <section class="atlas-detail-section">
                                                                <h3>"Source references"</h3>
                                                                <ul class="atlas-source-list">
                                                                    <For
                                                                        each=move || source_refs.clone()
                                                                        key=|source| format!("{}:{}", source.repository, source.path)
                                                                        children=move |source| view! {
                                                                            <li>
                                                                                <a href=source.url target="_blank" rel="noopener noreferrer">
                                                                                    {format!("{} · {}", source.repository, source.path)}
                                                                                </a>
                                                                                <small>{format!("Pinned ref: {}", source.revision)}</small>
                                                                            </li>
                                                                        }
                                                                    />
                                                                </ul>
                                                            </section>
                                                        }.into_any()
                                                    } else {
                                                        view! {
                                                            <p class="atlas-no-source">
                                                                "No canonical source reference is registered yet."
                                                            </p>
                                                        }.into_any()
                                                    }}

                                                    {if !limitations.is_empty() {
                                                        view! {
                                                            <section class="atlas-detail-section">
                                                                <h3>"Known limitations and open findings"</h3>
                                                                <ul class="atlas-limitations">
                                                                    <For
                                                                        each=move || limitations.clone().into_iter().enumerate().collect::<Vec<_>>()
                                                                        key=|(index, _)| *index
                                                                        children=move |(_index, limitation)| view! {
                                                                            <li>{limitation}</li>
                                                                        }
                                                                    />
                                                                </ul>
                                                            </section>
                                                        }.into_any()
                                                    } else {
                                                        view! { <></> }.into_any()
                                                    }}
                                                </details>
                                            }
                                            .into_any()
                                        } else {
                                            view! {
                                                <p class="atlas-no-details">
                                                    "No supporting metadata has been registered yet."
                                                </p>
                                            }
                                            .into_any()
                                        }}
                                    </article>
                                }
                            }
                        />
                    </div>

                    <footer class="atlas-footer">
                        <p>
                            "Next integration milestone: connect the Symthaea economics Rust kernel through a typed, fail-closed runner and attach reproducible run receipts. Until then, this catalogue is an evidence browser, not a simulator."
                        </p>
                    </footer>
                </section>
            }
            .into_any()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_registry_is_parseable_and_ids_are_unique() {
        let registry: AtlasRegistry =
            serde_json::from_str(MODEL_REGISTRY_JSON).expect("registry should parse");
        let mut ids = HashSet::new();

        assert!(!registry.registry_id.is_empty());
        assert!(!registry.schema_version.is_empty());
        assert!(
            !registry.qualified_for_execution,
            "a discovery inventory must not claim execution qualification"
        );
        assert!(!registry.models.is_empty());

        for model in &registry.models {
            assert!(!model.model_id.is_empty());
            assert!(!model.title.is_empty());
            assert!(ids.insert(model.model_id.clone()), "duplicate ID: {}", model.model_id);
            assert!(!model.epistemic_status.is_empty());
            assert!(!model.execution_status.is_empty());

            if model.epistemic_status != "discovery_required" {
                assert!(
                    !model.source_refs.is_empty(),
                    "registered model lacks source refs: {}",
                    model.model_id
                );
            }

            for source in &model.source_refs {
                assert!(
                    source.revision.len() == 40
                        && source.revision.chars().all(|ch| ch.is_ascii_hexdigit()),
                    "source ref must be an immutable 40-character commit SHA: {}:{}",
                    model.model_id,
                    source.path
                );
                assert!(
                    source.url.starts_with("https://github.com/"),
                    "source URL must be a GitHub URL: {}:{}",
                    model.model_id,
                    source.path
                );
            }
        }
    }

    #[test]
    fn unresolved_creditism_remains_explicitly_unresolved() {
        let registry: AtlasRegistry =
            serde_json::from_str(MODEL_REGISTRY_JSON).expect("registry should parse");
        let creditism = registry
            .models
            .iter()
            .find(|model| model.model_id == "creditism.fineco.discovery.v1")
            .expect("Creditism discovery record should remain visible");

        assert_eq!(creditism.epistemic_status, "discovery_required");
        assert!(creditism.source_refs.is_empty());
    }

    #[test]
    fn currency_variants_are_not_silently_normalized() {
        let registry: AtlasRegistry =
            serde_json::from_str(MODEL_REGISTRY_JSON).expect("registry should parse");

        let finance = registry
            .models
            .iter()
            .find(|model| model.model_id == "mycelix.finance.tricurrency.v1")
            .expect("Mycelix Finance contract should be registered");
        let essay = registry
            .models
            .iter()
            .find(|model| model.model_id == "symthaea.concepts.tricurrency.essay16.v1")
            .expect("Essay 16 should remain a separate variant");

        assert_ne!(finance.model_id, essay.model_id);
        assert_eq!(finance.epistemic_status, "implemented_source");
        assert_eq!(essay.epistemic_status, "conceptual");
    }
}
