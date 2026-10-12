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
const INDICATOR_CATALOG_JSON: &str =
    include_str!("../../../../../docs/economy-atlas/regenerative-indicators.v1.json");
const DATASET_CANDIDATES_JSON: &str =
    include_str!("../../../../../docs/economy-atlas/regenerative-dataset-candidates.v1.json");

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
    research_refs: Vec<AtlasResearchReference>,
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

#[derive(Clone, Debug, Deserialize)]
struct AtlasResearchReference {
    title: String,
    publisher: String,
    reference_type: String,
    #[serde(default)]
    published_year: Option<u16>,
    retrieved_on: String,
    url: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasIndicatorCatalog {
    catalog_status: String,
    qualification: AtlasIndicatorQualification,
    global_comparison_contract: Vec<String>,
    research_refs: Vec<AtlasIndicatorReference>,
    indicators: Vec<AtlasRegenerativeIndicator>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasIndicatorQualification {
    empirical_observations_loaded: bool,
    indicators_computed: bool,
    independently_validated: bool,
    composite_score_enabled: bool,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasIndicatorReference {
    ref_id: String,
    title: String,
    publisher: String,
    reference_type: String,
    #[serde(default)]
    published_year: Option<u16>,
    retrieved_on: String,
    url: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasRegenerativeIndicator {
    indicator_id: String,
    title: String,
    framework_ids: Vec<String>,
    domain: String,
    indicator_type: String,
    definition: String,
    native_unit: String,
    denominator: String,
    desired_direction: String,
    spatial_scope_required: String,
    temporal_resolution_required: String,
    baseline_and_target: String,
    uncertainty_required: String,
    input_requirements: Vec<String>,
    comparability_key: Vec<String>,
    missing_data_rule: String,
    caveats: Vec<String>,
    research_ref_ids: Vec<String>,
    status: String,
    observations_loaded: bool,
    computed: bool,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetCandidateCatalog {
    catalog_status: String,
    qualification: AtlasDatasetQualification,
    candidates: Vec<AtlasDatasetCandidate>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetQualification {
    source_artifacts_downloaded: bool,
    real_observations_imported: bool,
    candidate_ingestion_gates_passed: bool,
    dataset_rights_independently_verified: bool,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetCandidate {
    candidate_id: String,
    title: String,
    publisher: String,
    dataset_url: String,
    related_indicator_ids: Vec<String>,
    discovery_status: String,
    #[serde(default)]
    reported_availability: String,
    temporal_coverage: AtlasDatasetTemporalCoverage,
    spatial_coverage: AtlasDatasetSpatialCoverage,
    reported_measures: Vec<AtlasDatasetReportedMeasure>,
    reported_data_shape: BTreeMap<String, Value>,
    license_assessment: AtlasDatasetLicenseAssessment,
    ingestion_gate: AtlasDatasetIngestionGate,
    suitability: AtlasDatasetSuitability,
    #[serde(default)]
    known_findings: Vec<AtlasDatasetFinding>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetTemporalCoverage {
    start_date: Option<String>,
    end_date: Option<String>,
    basis: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetSpatialCoverage {
    geography: String,
    entities: Vec<Value>,
    caveat: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetReportedMeasure {
    name: String,
    native_unit: String,
    method_summary: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetLicenseAssessment {
    license_status: String,
    displayed_access_status: String,
    declared_file_license: Option<String>,
    commercial_reuse: String,
    redistribution: String,
    modification: String,
    evidence_urls: Vec<String>,
    assessment_note: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetIngestionGate {
    status: String,
    ingestion_authorized: bool,
    values_imported: bool,
    reason_codes: Vec<String>,
    required_evidence: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetSuitability {
    best_initial_use: String,
    not_suitable_for: String,
    next_action: String,
}

#[derive(Clone, Debug, Deserialize)]
struct AtlasDatasetFinding {
    finding_id: String,
    severity: String,
    observation: String,
    required_resolution: String,
}

fn evidence_label(status: &str) -> &'static str {
    match status {
        "implemented_source" => "Implementation source exists",
        "paper_reported" => "Paper-reported",
        "design_only" => "Design-only",
        "conceptual" => "Conceptual",
        "discovery_required" => "Discovery required",
        "external_framework" => "External framework",
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
        "external_framework" => "atlas-evidence-external",
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

fn display_dataset_spatial_entity(value: &Value) -> String {
    let Some(object) = value.as_object() else {
        return display_registry_value(value);
    };

    if let (Some(code), Some(name)) = (
        object.get("code").and_then(Value::as_str),
        object.get("name").and_then(Value::as_str),
    ) {
        return format!("{name} ({code})");
    }

    if let (Some(site), Some(context), Some(source_coordinates), Some(coordinates)) = (
        object.get("site").and_then(Value::as_str),
        object.get("context").and_then(Value::as_str),
        object.get("coordinates_as_published").and_then(Value::as_str),
        object.get("coordinates_decimal_degrees").and_then(Value::as_object),
    ) {
        if let (Some(latitude), Some(longitude)) = (
            coordinates.get("latitude").and_then(Value::as_f64),
            coordinates.get("longitude").and_then(Value::as_f64),
        ) {
            return format!(
                "Site {site} ({context}): {latitude:.5}, {longitude:.5}; source coordinates: {source_coordinates}"
            );
        }
    }

    display_registry_value(value)
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
    let research_refs = model.research_refs;
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
                            "No in-repository implementation source is registered; external research references may still be available below."
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
                {if research_refs.is_empty() {
                    view! { <></> }.into_any()
                } else {
                    view! {
                        <section class="atlas-research-references">
                            <h3>"External research and frameworks"</h3>
                            <p class="atlas-disclaimer">
                                "These references describe external research or methodologies; they do not establish that Mycelix implements or has validated them."
                            </p>
                            {research_refs.into_iter().map(|reference| {
                                let title = reference.title;
                                let publisher = reference.publisher;
                                let reference_type = reference.reference_type.replace('_', " ");
                                let year = reference.published_year
                                    .map(|value| format!(" · {value}"))
                                    .unwrap_or_default();
                                let retrieved_on = reference.retrieved_on;
                                let url = reference.url;
                                view! {
                                    <p class="atlas-source-link">
                                        <a href=url target="_blank" rel="noopener noreferrer">{title}</a>
                                        <small class="atlas-research-meta">
                                            {format!("{publisher} · {reference_type}{year} · Retrieved {retrieved_on}")}
                                        </small>
                                    </p>
                                }
                            }).collect_view()}
                        </section>
                    }.into_any()
                }}
                <p class="atlas-disclaimer">
                    "A source link establishes traceability, not model correctness, runtime execution or empirical validity."
                </p>
            </details>
        </article>
    }
}

fn render_regenerative_indicator_card(
    indicator: AtlasRegenerativeIndicator,
    references: Vec<AtlasIndicatorReference>,
) -> impl IntoView {
    let title = indicator.title;
    let indicator_id = indicator.indicator_id;
    let domain = indicator.domain.replace('_', " ");
    let definition = indicator.definition;
    let native_unit = indicator.native_unit;
    let direction = indicator.desired_direction.replace('_', " ");
    let denominator = indicator.denominator;
    let spatial_scope = indicator.spatial_scope_required;
    let temporal_resolution = indicator.temporal_resolution_required;
    let baseline_and_target = indicator.baseline_and_target;
    let uncertainty = indicator.uncertainty_required;
    let missing_data_rule = indicator.missing_data_rule;
    let status = indicator.status.replace('_', " ");
    let observations_loaded = indicator.observations_loaded;
    let computed = indicator.computed;
    let framework_ids = indicator.framework_ids.join(" · ");
    let indicator_type = indicator.indicator_type.replace('_', " ");
    let comparability_items = indicator.comparability_key.into_iter().map(|item| {
        view! { <li>{item}</li> }
    }).collect_view();
    let input_items = indicator.input_requirements.into_iter().map(|item| {
        view! { <li>{item}</li> }
    }).collect_view();
    let caveat_items = indicator.caveats.into_iter().map(|item| {
        view! { <li>{item}</li> }
    }).collect_view();
    let linked_references = indicator.research_ref_ids.iter()
        .filter_map(|ref_id| references.iter().find(|reference| &reference.ref_id == ref_id).cloned())
        .map(|reference| {
            let url = reference.url;
            let title = reference.title;
            let metadata = format!(
                "{} · {}{} · retrieved {}",
                reference.publisher,
                reference.reference_type.replace('_', " "),
                reference.published_year.map(|year| format!(" · {year}")).unwrap_or_default(),
                reference.retrieved_on
            );
            view! {
                <p class="atlas-source-link">
                    <a href={url} target="_blank" rel="noopener noreferrer">{title}</a>
                    <span class="atlas-research-meta">{metadata}</span>
                </p>
            }
        }).collect_view();

    view! {
        <article class="dash-card atlas-indicator-card">
            <div class="atlas-card-top">
                <span class="badge atlas-evidence-design">"Candidate definition"</span>
                <span class="atlas-domain">{domain}</span>
            </div>
            <h3>{title}</h3>
            <p class="atlas-model-id">{indicator_id}</p>
            <p>{definition}</p>
            <p class="atlas-indicator-unit"><strong>"Native unit: "</strong>{native_unit}</p>
            <details>
                <summary>"Measurement contract, comparability and evidence"</summary>
                <p><strong>"Type: "</strong>{indicator_type}</p>
                <p><strong>"Direction: "</strong>{direction}</p>
                <p><strong>"Denominator: "</strong>{denominator}</p>
                <p><strong>"Required spatial scope: "</strong>{spatial_scope}</p>
                <p><strong>"Required time resolution: "</strong>{temporal_resolution}</p>
                <p><strong>"Baseline and target: "</strong>{baseline_and_target}</p>
                <p><strong>"Uncertainty: "</strong>{uncertainty}</p>
                <p><strong>"Missing-data rule: "</strong>{missing_data_rule}</p>
                <h4>"Required input fields"</h4>
                <ul>{input_items}</ul>
                <h4>"Comparability key"</h4>
                <ul>{comparability_items}</ul>
                <h4>"Known caveats"</h4>
                <ul>{caveat_items}</ul>
                <h4>"Framework linkages"</h4>
                <p class="atlas-disclaimer">{framework_ids}</p>
                <h4>"Research provenance"</h4>
                {linked_references}
                <p class="atlas-disclaimer">
                    {format!("Status: {status}. Observations loaded: {observations_loaded}. Computed: {computed}.")}
                </p>
                <p class="atlas-disclaimer">
                    "A candidate definition is a measurement contract, not evidence that the quantity has been measured or improved."
                </p>
            </details>
        </article>
    }
}


fn render_dataset_candidate_card(candidate: AtlasDatasetCandidate) -> impl IntoView {
    let candidate_id = candidate.candidate_id;
    let title = candidate.title;
    let publisher = candidate.publisher;
    let url = candidate.dataset_url;
    let status = candidate.discovery_status.replace('_', " ");
    let reported_availability = if candidate.reported_availability.trim().is_empty() {
        "No separate source availability statement is recorded.".to_string()
    } else {
        candidate.reported_availability
    };
    let indicator_ids = candidate.related_indicator_ids.join(" · ");
    let start = candidate.temporal_coverage.start_date.unwrap_or_else(|| "unknown start".to_string());
    let end = candidate.temporal_coverage.end_date.unwrap_or_else(|| "unknown end".to_string());
    let temporal_basis = candidate.temporal_coverage.basis;
    let geography = candidate.spatial_coverage.geography;
    let spatial_caveat = candidate.spatial_coverage.caveat;
    let entities = candidate.spatial_coverage.entities.into_iter().map(|entity| {
        let label = display_dataset_spatial_entity(&entity);
        view! { <li>{label}</li> }
    }).collect_view();
    let measures = candidate.reported_measures.into_iter().map(|measure| {
        view! {
            <li>
                <strong>{measure.name}</strong>
                <span>{format!(" · Native unit: {}", measure.native_unit)}</span>
                <p>{measure.method_summary}</p>
            </li>
        }
    }).collect_view();
    let shape = serde_json::to_string_pretty(&Value::Object(
        candidate.reported_data_shape.into_iter().collect()
    )).unwrap_or_else(|error| format!("Reported metadata could not be formatted: {error}"));
    let license_status = candidate.license_assessment.license_status.replace('_', " ");
    let access_status = candidate.license_assessment.displayed_access_status;
    let declared_license = candidate.license_assessment.declared_file_license
        .unwrap_or_else(|| "Not explicitly displayed".to_string());
    let commercial_reuse = candidate.license_assessment.commercial_reuse.replace('_', " ");
    let redistribution = candidate.license_assessment.redistribution.replace('_', " ");
    let modification = candidate.license_assessment.modification.replace('_', " ");
    let license_note = candidate.license_assessment.assessment_note;
    let evidence_links = candidate.license_assessment.evidence_urls.into_iter().map(|evidence_url| {
        let label = evidence_url.clone();
        view! {
            <li><a href={evidence_url} target="_blank" rel="noopener noreferrer">{label}</a></li>
        }
    }).collect_view();
    let gate_status = candidate.ingestion_gate.status;
    let ingestion_authorized = candidate.ingestion_gate.ingestion_authorized;
    let values_imported = candidate.ingestion_gate.values_imported;
    let reason_codes = candidate.ingestion_gate.reason_codes.into_iter().map(|reason| {
        view! { <li>{reason.replace('_', " ")}</li> }
    }).collect_view();
    let required_evidence = candidate.ingestion_gate.required_evidence.into_iter().map(|item| {
        view! { <li>{item}</li> }
    }).collect_view();
    let best_use = candidate.suitability.best_initial_use;
    let excluded_use = candidate.suitability.not_suitable_for;
    let next_action = candidate.suitability.next_action;
    let metadata_findings = candidate.known_findings.into_iter().map(|finding| {
        view! {
            <li>
                <strong>{format!("{} · {}", finding.finding_id, finding.severity)}</strong>
                <p>{finding.observation}</p>
                <p><strong>"Required resolution: "</strong>{finding.required_resolution}</p>
            </li>
        }
    }).collect_view();

    view! {
        <article class="dash-card atlas-dataset-card">
            <div class="atlas-card-top">
                <span class="badge atlas-evidence-unknown">"Import blocked"</span>
                <span class="atlas-domain">{status}</span>
            </div>
            <h3>{title}</h3>
            <p class="atlas-model-id">{candidate_id}</p>
            <p class="atlas-research-meta">{publisher}</p>
            <p>{reported_availability}</p>
            <p><strong>"Rights assessment: "</strong>{access_status}</p>
            <p class="atlas-model-id">{format!("Related indicator: {indicator_ids}")}</p>
            <p><strong>"Source record: "</strong><a href={url} target="_blank" rel="noopener noreferrer">"Open source record"</a></p>
            <details>
                <summary>"Rights, coverage and ingestion gate"</summary>
                <p><strong>"Declared file license: "</strong>{declared_license}</p>
                <p><strong>"License review status: "</strong>{license_status}</p>
                <p><strong>"Commercial reuse: "</strong>{commercial_reuse}</p>
                <p><strong>"Redistribution: "</strong>{redistribution}</p>
                <p><strong>"Modification: "</strong>{modification}</p>
                <p>{license_note}</p>
                <h4>"Temporal coverage (reported)"</h4>
                <p>{format!("{start} to {end}")}</p>
                <p>{temporal_basis}</p>
                <h4>"Spatial coverage (reported)"</h4>
                <p>{geography}</p>
                <p>{spatial_caveat}</p>
                <ul>{entities}</ul>
                <h4>"Reported measures"</h4>
                <ul>{measures}</ul>
                <h4>"Reported data shape"</h4>
                <pre>{shape}</pre>
                <h4>"Recorded metadata findings (preview/source-level)"</h4>
                <ul>{metadata_findings}</ul>
                <p class="atlas-disclaimer">
                    "These are metadata discrepancies to reconcile, not a verdict that the underlying measurements are wrong."
                </p>
                <h4>"Rights evidence links"</h4>
                <ul>{evidence_links}</ul>
                <h4>"Ingestion gate"</h4>
                <p>{format!("Status: {gate_status}. Authorized: {ingestion_authorized}. Values imported: {values_imported}.")}</p>
                <h4>"Blocking reasons"</h4>
                <ul>{reason_codes}</ul>
                <h4>"Evidence needed before import"</h4>
                <ul>{required_evidence}</ul>
                <h4>"Scope and next step"</h4>
                <p><strong>"Potential use: "</strong>{best_use}</p>
                <p><strong>"Not suitable for: "</strong>{excluded_use}</p>
                <p><strong>"Next action: "</strong>{next_action}</p>
                <p class="atlas-disclaimer">
                    "This is a discovery record, not legal advice or evidence that any source file has been authenticated. No values are imported by this catalog."
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

    let indicator_catalog = match serde_json::from_str::<AtlasIndicatorCatalog>(INDICATOR_CATALOG_JSON) {
        Ok(catalog) => catalog,
        Err(error) => {
            return view! {
                <section class="page-economy-atlas" role="alert">
                    <h1>"Regenerative indicator catalog unavailable"</h1>
                    <p class="subtitle">
                        "The checked-in catalog could not be read. No indicator values or fallback definitions were substituted."
                    </p>
                    <pre>{error.to_string()}</pre>
                </section>
            }
            .into_any();
        }
    };
    let dataset_candidate_catalog = match serde_json::from_str::<AtlasDatasetCandidateCatalog>(DATASET_CANDIDATES_JSON) {
        Ok(catalog) => catalog,
        Err(error) => {
            return view! {
                <section class="page-economy-atlas" role="alert">
                    <h1>"Regenerative dataset catalog unavailable"</h1>
                    <p class="subtitle">
                        "The checked-in dataset catalog could not be read. No source data or fallback records were substituted."
                    </p>
                    <pre>{error.to_string()}</pre>
                </section>
            }
            .into_any();
        }
    };
    let dataset_candidate_count = dataset_candidate_catalog.candidates.len();
    let dataset_catalog_status = dataset_candidate_catalog.catalog_status;
    let dataset_qualification = dataset_candidate_catalog.qualification;
    let dataset_candidate_cards = dataset_candidate_catalog.candidates
        .into_iter()
        .map(render_dataset_candidate_card)
        .collect_view();
    let indicator_count = indicator_catalog.indicators.len();
    let indicator_references = indicator_catalog.research_refs.clone();
    let indicator_cards = indicator_catalog.indicators
        .into_iter()
        .map(|indicator| render_regenerative_indicator_card(indicator, indicator_references.clone()))
        .collect_view();
    let indicator_rules = indicator_catalog.global_comparison_contract;
    let indicator_catalog_status = indicator_catalog.catalog_status;
    let indicator_qualification = indicator_catalog.qualification;

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
                let research_text = model.research_refs.iter()
                    .map(|reference| format!(
                        "{} {} {} {}",
                        reference.title, reference.publisher, reference.reference_type, reference.url
                    ))
                    .collect::<Vec<_>>()
                    .join(" ");
                let finding_text = model.validation_findings.iter()
                    .map(|finding| format!("{} {} {}", finding.finding_id, finding.observation, finding.required_follow_up))
                    .collect::<Vec<_>>()
                    .join(" ");
                let haystack = format!(
                    "{} {} {} {} {} {} {} {} {}",
                    model.model_id,
                    model.title,
                    model.domain.join(" "),
                    model.epistemic_status,
                    model.execution_status,
                    model.limitations.join(" "),
                    metadata,
                    source_text,
                    format!("{} {}", research_text, finding_text)
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
                        <option value="external_framework">"External framework"</option>
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

            <section class="atlas-indicators-section" aria-labelledby="regenerative-indicators-heading">
                <div class="dash-card">
                    <h2 id="regenerative-indicators-heading">"Regenerative indicator catalog"</h2>
                    <p class="subtitle">
                        {format!("{} candidate definitions · catalog status: {}", indicator_count, indicator_catalog_status.replace('_', " "))}
                    </p>
                    <p>
                        "This is a measurement contract catalog only. No empirical observations have been loaded, indicators have not been computed, independent validation has not occurred, and no composite score is enabled."
                    </p>
                    <p class="atlas-indicator-qualification">
                        {format!(
                            "Observations loaded: {} · Computed: {} · Independently validated: {} · Composite score enabled: {}",
                            indicator_qualification.empirical_observations_loaded,
                            indicator_qualification.indicators_computed,
                            indicator_qualification.independently_validated,
                            indicator_qualification.composite_score_enabled
                        )}
                    </p>
                    <h3>"Shared comparison contract"</h3>
                    <ul>{indicator_rules.into_iter().map(|rule| view! { <li>{rule}</li> }).collect_view()}</ul>
                </div>
                <div class="dashboard-grid atlas-indicator-grid">
                    {indicator_cards}
                </div>
            </section>

            <section class="atlas-indicators-section" aria-labelledby="regenerative-dataset-candidates-heading">
                <div class="dash-card">
                    <h2 id="regenerative-dataset-candidates-heading">"Dataset candidates and rights gates"</h2>
                    <p class="subtitle">
                        {format!("{} data-source candidates · catalog status: {}", dataset_candidate_count, dataset_catalog_status.replace('_', " "))}
                    </p>
                    <p>
                        "Dataset access is not the same as reuse permission. Every candidate remains blocked until rights, coverage, data dictionary and source-file provenance have been reviewed."
                    </p>
                    <p class="atlas-indicator-qualification">
                        {format!(
                            "Source artifacts downloaded: {} · Real observations imported: {} · Ingestion gates passed: {} · Dataset rights independently verified: {}",
                            dataset_qualification.source_artifacts_downloaded,
                            dataset_qualification.real_observations_imported,
                            dataset_qualification.candidate_ingestion_gates_passed,
                            dataset_qualification.dataset_rights_independently_verified
                        )}
                    </p>
                </div>
                <div class="dashboard-grid atlas-dataset-grid">
                    {dataset_candidate_cards}
                </div>
            </section>

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

    fn indicator_catalog() -> AtlasIndicatorCatalog {
        serde_json::from_str(INDICATOR_CATALOG_JSON)
            .expect("checked-in regenerative indicator catalog must parse")
    }

    fn dataset_candidate_catalog() -> AtlasDatasetCandidateCatalog {
        serde_json::from_str(DATASET_CANDIDATES_JSON)
            .expect("checked-in dataset candidate catalog must parse")
    }

    #[test]
    fn asm_jukskei_site_coordinates_render_with_source_provenance() {
        let catalog = dataset_candidate_catalog();
        let asm = catalog.candidates.iter()
            .find(|candidate| candidate.candidate_id == "south-africa.jukskei-river-microbial-data.2025.v1")
            .expect("2025 ASM Jukskei candidate should exist");
        let site_a = asm.spatial_coverage.entities.iter()
            .find(|site| site.get("site").and_then(Value::as_str) == Some("A"))
            .expect("site A should exist");
        let rendered = display_dataset_spatial_entity(site_a);
        assert!(rendered.contains("-26.18147, 28.10237"), "{rendered}");
        assert!(rendered.contains("26,18147°S, 28,10237°E"), "{rendered}");
        assert!(rendered.contains("business district"), "{rendered}");
    }

    #[test]
    fn dataset_candidate_catalog_is_discovery_only_and_fail_closed() {
        let catalog = dataset_candidate_catalog();
        assert_eq!(catalog.catalog_status, "source_discovery_only");
        assert_eq!(catalog.candidates.len(), 6);
        assert!(!catalog.qualification.source_artifacts_downloaded);
        assert!(!catalog.qualification.real_observations_imported);
        assert!(!catalog.qualification.candidate_ingestion_gates_passed);
        assert!(!catalog.qualification.dataset_rights_independently_verified);
        let ids = catalog.candidates.iter().map(|candidate| candidate.candidate_id.as_str()).collect::<HashSet<_>>();
        assert_eq!(ids.len(), catalog.candidates.len());
        let indicator_catalog = indicator_catalog();
        let indicator_ids = indicator_catalog.indicators.iter().map(|indicator| indicator.indicator_id.as_str()).collect::<HashSet<_>>();
        let mut statuses = HashSet::new();
        for candidate in &catalog.candidates {
            assert!(candidate.dataset_url.starts_with("https://"));
            assert!(candidate.related_indicator_ids.iter().all(|id| indicator_ids.contains(id.as_str())));
            assert_eq!(candidate.ingestion_gate.status, "blocked");
            assert!(!candidate.ingestion_gate.ingestion_authorized);
            assert!(!candidate.ingestion_gate.values_imported);
            assert!(!candidate.ingestion_gate.required_evidence.is_empty());
            assert!(!candidate.license_assessment.evidence_urls.is_empty());
            if candidate.candidate_id == "south-africa.swam-satellite-water-quality.2026.v1" {
                assert!(candidate.known_findings.iter().any(|finding| finding.finding_id == "SWAM-PREVIEW-DATE-WINDOW-001"));
            }
            statuses.insert(candidate.discovery_status.as_str());
        }
        assert!(statuses.contains("license_not_verified"));
        assert!(statuses.contains("license_metadata_conflict"));
        assert!(statuses.contains("restricted_noncommercial_use"));
        assert!(statuses.contains("license_declared_access_private"));
        assert!(catalog.candidates.iter().any(|candidate| candidate.candidate_id == "south-africa.jukskei-water-quality-study.2021.v1"));
        assert!(catalog.candidates.iter().any(|candidate| candidate.candidate_id == "south-africa.up-hartbeespoort-roodeplaat-water-quality.2025.v1"));
        assert!(catalog.candidates.iter().any(|candidate| candidate.candidate_id == "south-africa.jukskei-river-microbial-data.2025.v1"));
        let swam = catalog.candidates.iter().find(|candidate| candidate.candidate_id == "south-africa.swam-satellite-water-quality.2026.v1").expect("SWAM candidate should exist");
        assert!(swam.reported_availability.contains("GPL-3.0"));
        let asm = catalog.candidates.iter().find(|candidate| candidate.candidate_id == "south-africa.jukskei-river-microbial-data.2025.v1").expect("ASM Jukskei candidate should exist");
        assert!(asm.reported_availability.contains("contact the corresponding author"));
    }

    #[test]
    fn regenerative_indicator_catalog_is_candidate_only() {
        let catalog = indicator_catalog();
        assert_eq!(catalog.catalog_status, "candidate_indicator_definitions");
        assert_eq!(catalog.indicators.len(), 21);
        assert!(!catalog.qualification.empirical_observations_loaded);
        assert!(!catalog.qualification.indicators_computed);
        assert!(!catalog.qualification.independently_validated);
        assert!(!catalog.qualification.composite_score_enabled);
        assert!(!catalog.global_comparison_contract.is_empty());
        assert_eq!(catalog.research_refs.len(), 14);
        let reference_ids = catalog.research_refs.iter().map(|reference| reference.ref_id.as_str()).collect::<HashSet<_>>();
        let model_registry = registry();
        let framework_ids = model_registry.models.iter().map(|model| model.model_id.as_str()).collect::<HashSet<_>>();
        let indicator_ids = catalog.indicators.iter().map(|indicator| indicator.indicator_id.as_str()).collect::<HashSet<_>>();
        assert_eq!(indicator_ids.len(), catalog.indicators.len());
        for indicator in &catalog.indicators {
            assert_eq!(indicator.status, "candidate_definition");
            assert!(!indicator.observations_loaded);
            assert!(!indicator.computed);
            assert!(!indicator.comparability_key.is_empty(), "{}", indicator.indicator_id);
            assert!(!indicator.research_ref_ids.is_empty(), "{}", indicator.indicator_id);
            for research_ref in &indicator.research_ref_ids {
                assert!(reference_ids.contains(research_ref.as_str()), "{} has missing research ref {}", indicator.indicator_id, research_ref);
            }
            for framework in &indicator.framework_ids {
                assert!(framework_ids.contains(framework.as_str()), "{} references an unknown framework {}", indicator.indicator_id, framework);
            }
        }
    }

    #[test]
    fn registry_is_visible_but_not_execution_qualified() {
        let registry = registry();
        assert_eq!(registry.registry_status, "initial_discovery_inventory");
        assert!(!registry.qualified_for_execution);
        assert_eq!(registry.models.len(), 13);
    }

    #[test]
    fn regenerative_frameworks_are_external_linked_and_not_execution_qualified() {
        let registry = registry();
        let ids = registry.models.iter().map(|model| model.model_id.as_str()).collect::<HashSet<_>>();
        for id in [
            "regenerative.economics.energy-network-science.v1",
            "regenerative.doughnut.place-portrait.v1",
            "regenerative.circular-economy.v1",
            "regenerative.land.agroecology-outcomes.v1",
        ] {
            assert!(ids.contains(id), "missing regenerative framework: {id}");
        }
        let external = registry.models.iter()
            .filter(|model| model.epistemic_status == "external_framework")
            .collect::<Vec<_>>();
        assert_eq!(external.len(), 4);
        for model in external {
            assert!(model.source_refs.is_empty(), "external refs must not masquerade as implementation source refs");
            assert!(!model.research_refs.is_empty(), "external framework needs source links");
            for source in &model.research_refs {
                assert!(source.url.starts_with("https://"));
                assert!(!source.title.trim().is_empty());
                assert!(!source.publisher.trim().is_empty());
            }
        }
    }

    #[test]
    fn regenerative_framework_contracts_survive_registry_deserialization() {
        let registry = registry();
        let find_model = |id: &str| {
            registry
                .models
                .iter()
                .find(|model| model.model_id == id)
                .unwrap_or_else(|| panic!("missing framework: {id}"))
        };
        assert!(find_model("regenerative.economics.energy-network-science.v1")
            .details.contains_key("measurable_dimensions"));
        assert!(find_model("regenerative.doughnut.place-portrait.v1")
            .details.contains_key("normalization_contract"));
        assert!(find_model("regenerative.circular-economy.v1")
            .details.contains_key("core_principles"));
        assert!(find_model("regenerative.land.agroecology-outcomes.v1")
            .details.contains_key("candidate_outcomes"));
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
    fn hearth_compost_delivery_finding_is_source_bound() {
        let registry = registry();
        let finance = registry
            .models
            .iter()
            .find(|model| model.model_id == "mycelix.finance.tricurrency.v1")
            .expect("Finance model should exist");
        let finding = finance
            .validation_findings
            .iter()
            .find(|finding| {
                finding.finding_id == "FIN-ATLAS-COMPOST-DELIVERY-STATUS-001"
            })
            .expect("hearth compost delivery finding should remain visible");
        assert_eq!(
            finding.source_file,
            "mycelix-finance/zomes/payments/coordinator/src/lib.rs"
        );
        assert!(finance.source_refs.iter().any(|source| {
            source.path == finding.source_file
                && source.ref_ == finding.source_commit
                && source.url.ends_with(&format!(
                    "/{}/{}",
                    finding.source_commit,
                    finding.source_file
                ))
        }));
        assert!(finding.observation.contains("redistributed: true"));
        assert!(!finding.required_follow_up.trim().is_empty());
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
        let mut finding_ids = HashSet::new();
        for model in &registry.models {
            for finding in &model.validation_findings {
                count += 1;
                assert!(
                    finding_ids.insert(finding.finding_id.as_str()),
                    "duplicate finding ID: {}",
                    finding.finding_id
                );
                assert!(!finding.severity.trim().is_empty());
                assert!(!finding.observation.trim().is_empty());
                assert!(!finding.required_follow_up.trim().is_empty());
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
        assert_eq!(count, 7);
    }
}
