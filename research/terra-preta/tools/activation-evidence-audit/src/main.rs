use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "activation-evidence-audit-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const CLAIM_CEILING: &str = "This evaluator tests evidence admissibility on manufactured fixtures. It does not infer real hydraulic activation, convert CT geometry or model predictions into observations, or validate a field flow model.";

#[derive(Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    specimen_id: String,
    event: Event,
    registered_edges: Vec<RegisteredEdge>,
    evidence_records: Vec<EvidenceRecord>,
    assertions: Vec<EdgeAssertion>,
}

#[derive(Debug, Deserialize)]
struct RegisteredEdge {
    edge_id: String,
    spatial_support_id: String,
}

#[derive(Debug, Deserialize)]
struct Event {
    event_id: String,
    start_epoch_seconds: i64,
    end_epoch_seconds: i64,
    minimum_relevant_tracer_mass_flux_mg_per_h: f64,
    minimum_relevant_water_volume_flux_ml_per_h: f64,
    minimum_bound_confidence_level: f64,
    maximum_sample_interval_seconds: i64,
    minimum_coverage_fraction: f64,
    maximum_registration_error_mm: f64,
    minimum_support_overlap_fraction: f64,
}

#[derive(Clone, Debug, Deserialize)]
struct EvidenceRecord {
    evidence_id: String,
    edge_id: String,
    specimen_id: String,
    event_id: String,
    evidence_class: String,
    source_artifact_uri: String,
    source_artifact_sha256: String,
    method_id: String,
    method_version: String,
    calibration_id: Option<String>,
    rate_measurand: String,
    rate_unit: String,
    qc_status: String,
    support_kind: String,
    registration_status: String,
    spatial_support_id: String,
    registration_transform_id: Option<String>,
    spatial_localization: String,
    registration_error_mm: Option<f64>,
    support_overlap_fraction: f64,
    observed_start_epoch_seconds: i64,
    observed_end_epoch_seconds: i64,
    sample_interval_seconds: i64,
    coverage_fraction: f64,
    measured_rate_value: Option<f64>,
    lower_rate_bound_value: Option<f64>,
    lower_rate_bound_confidence_level: Option<f64>,
    upper_rate_bound_value: Option<f64>,
    upper_rate_bound_confidence_level: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct EdgeAssertion {
    edge_id: String,
    asserted_state: String,
    evidence_ids: Vec<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    assertion_count: usize,
    confirmed_active_count: usize,
    confirmed_inactive_count: usize,
    unassessed_count: usize,
    results: Vec<EdgeResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct EdgeResult {
    edge_id: String,
    asserted_state: String,
    admission_status: String,
    qualifying_active_evidence_ids: Vec<String>,
    qualifying_inactive_evidence_ids: Vec<String>,
    reasons: Vec<String>,
}

#[derive(Debug, Serialize)]
struct Controls {
    direct_observation_required_for_confirmed_hydraulic_state: bool,
    structural_image_cannot_confirm_hydraulic_activation: bool,
    model_prediction_cannot_confirm_hydraulic_activation: bool,
    inactivity_requires_event_coverage_and_detection_bound: bool,
    conflicting_direct_evidence_abstains: bool,
    specimen_event_and_registered_3d_edge_support_required: bool,
    synthetic_only_claim_ceiling: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CandidateState {
    Active,
    Inactive,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn is_direct_observation(class: &str) -> bool {
    matches!(
        class,
        "direct_tracer_observation" | "direct_hydraulic_observation"
    )
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "activation-evidence-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.scenario_id.trim().is_empty()
        || s.specimen_id.trim().is_empty()
        || s.event.event_id.trim().is_empty()
    {
        return Err(
            "unsupported schema/fixture status or empty scenario, specimen or event identity"
                .into(),
        );
    }
    if s.event.start_epoch_seconds >= s.event.end_epoch_seconds
        || s.event.maximum_sample_interval_seconds <= 0
    {
        return Err("event time bounds or maximum sample interval are invalid".into());
    }
    finite(
        "minimum_relevant_tracer_mass_flux_mg_per_h",
        s.event.minimum_relevant_tracer_mass_flux_mg_per_h,
    )?;
    finite(
        "minimum_relevant_water_volume_flux_ml_per_h",
        s.event.minimum_relevant_water_volume_flux_ml_per_h,
    )?;
    finite(
        "minimum_coverage_fraction",
        s.event.minimum_coverage_fraction,
    )?;
    finite(
        "minimum_bound_confidence_level",
        s.event.minimum_bound_confidence_level,
    )?;
    if s.event.minimum_relevant_tracer_mass_flux_mg_per_h <= 0.0
        || s.event.minimum_relevant_water_volume_flux_ml_per_h <= 0.0
        || !(0.0..=1.0).contains(&s.event.minimum_coverage_fraction)
        || s.event.minimum_coverage_fraction <= 0.0
        || !(0.0..=1.0).contains(&s.event.minimum_bound_confidence_level)
        || s.event.minimum_bound_confidence_level <= 0.0
    {
        return Err(
            "event flux threshold and minimum coverage must be positive and admissible".into(),
        );
    }
    finite(
        "maximum_registration_error_mm",
        s.event.maximum_registration_error_mm,
    )?;
    finite(
        "minimum_support_overlap_fraction",
        s.event.minimum_support_overlap_fraction,
    )?;
    if s.event.maximum_registration_error_mm <= 0.0
        || !(0.0..=1.0).contains(&s.event.minimum_support_overlap_fraction)
        || s.event.minimum_support_overlap_fraction <= 0.0
    {
        return Err("registration error and overlap thresholds are invalid".into());
    }
    if s.registered_edges.is_empty() || s.assertions.is_empty() {
        return Err("scenario requires registered graph edges and edge assertions".into());
    }

    let mut edge_support_by_id = BTreeMap::new();
    for edge in &s.registered_edges {
        if edge.edge_id.trim().is_empty()
            || edge.spatial_support_id.trim().is_empty()
            || edge_support_by_id
                .insert(edge.edge_id.as_str(), edge.spatial_support_id.as_str())
                .is_some()
        {
            return Err("registered edges require unique non-empty edge and support IDs".into());
        }
    }
    let edge_ids: BTreeSet<&str> = edge_support_by_id.keys().copied().collect();

    let mut evidence_by_id = BTreeMap::new();
    for evidence in &s.evidence_records {
        if evidence.evidence_id.trim().is_empty()
            || evidence_by_id.contains_key(evidence.evidence_id.as_str())
            || !edge_ids.contains(evidence.edge_id.as_str())
        {
            return Err(
                "evidence IDs must be unique and every evidence edge must be registered".into(),
            );
        }
        if edge_support_by_id.get(evidence.edge_id.as_str()).copied()
            != Some(evidence.spatial_support_id.as_str())
        {
            return Err(format!(
                "evidence {} support identity does not match the registered graph edge support",
                evidence.evidence_id
            ));
        }
        if evidence.specimen_id.trim().is_empty()
            || evidence.event_id.trim().is_empty()
            || evidence.source_artifact_uri.trim().is_empty()
            || evidence.method_id.trim().is_empty()
            || evidence.method_version.trim().is_empty()
            || evidence.spatial_support_id.trim().is_empty()
            || evidence.rate_measurand.trim().is_empty()
            || evidence.rate_unit.trim().is_empty()
            || !valid_sha256(&evidence.source_artifact_sha256)
        {
            return Err(format!(
                "evidence {} requires specimen/event, immutable artifact URI/SHA-256 and method identity",
                evidence.evidence_id
            ));
        }
        if !matches!(
            evidence.evidence_class.as_str(),
            "direct_tracer_observation"
                | "direct_hydraulic_observation"
                | "boundary_tracer_response"
                | "structural_image_only"
                | "derived_sensor_estimate"
                | "model_prediction"
        ) {
            return Err(format!(
                "evidence {} has unsupported evidence class",
                evidence.evidence_id
            ));
        }
        let rate_matches_class = match evidence.evidence_class.as_str() {
            "direct_tracer_observation" | "boundary_tracer_response" => {
                evidence.rate_measurand == "tracer_mass_flux" && evidence.rate_unit == "mg/h"
            }
            "direct_hydraulic_observation" => {
                evidence.rate_measurand == "water_volume_flux" && evidence.rate_unit == "mL/h"
            }
            _ => matches!(
                (
                    evidence.rate_measurand.as_str(),
                    evidence.rate_unit.as_str()
                ),
                ("tracer_mass_flux", "mg/h") | ("water_volume_flux", "mL/h")
            ),
        };
        if !rate_matches_class {
            return Err(format!(
                "evidence {} measurand/unit does not match its evidence class",
                evidence.evidence_id
            ));
        }
        if !matches!(
            evidence.qc_status.as_str(),
            "passed" | "failed" | "not_assessed"
        ) {
            return Err("QC status must be passed, failed or not_assessed".into());
        }
        if !matches!(
            evidence.support_kind.as_str(),
            "registered_3d_edge" | "core_aggregate" | "profile_aggregate" | "section_2d"
        ) {
            return Err("unsupported evidence spatial support kind".into());
        }
        if !matches!(
            evidence.spatial_localization.as_str(),
            "edge_resolved" | "boundary_integrated" | "core_aggregate" | "profile_aggregate"
        ) {
            return Err("unsupported spatial localization class".into());
        }
        if !matches!(
            evidence.registration_status.as_str(),
            "registered" | "unregistered" | "not_applicable"
        ) {
            return Err(
                "registration status must be registered, unregistered or not_applicable".into(),
            );
        }
        if evidence.registration_status == "registered"
            && evidence
                .registration_transform_id
                .as_deref()
                .unwrap_or("")
                .trim()
                .is_empty()
        {
            return Err("registered evidence requires a registration transform identity".into());
        }
        finite(
            "support_overlap_fraction",
            evidence.support_overlap_fraction,
        )?;
        if !(0.0..=1.0).contains(&evidence.support_overlap_fraction) {
            return Err("support overlap fraction must be in [0,1]".into());
        }
        if let Some(error) = evidence.registration_error_mm {
            finite("registration_error_mm", error)?;
            if error < 0.0 {
                return Err("registration error cannot be negative".into());
            }
        }
        if evidence.spatial_localization == "edge_resolved"
            && (evidence.support_kind != "registered_3d_edge"
                || evidence.registration_status != "registered"
                || evidence
                    .registration_transform_id
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .is_empty()
                || evidence.registration_error_mm.is_none())
        {
            return Err("edge-resolved evidence requires registered 3D support, transform identity and registration error".into());
        }
        if evidence.observed_start_epoch_seconds >= evidence.observed_end_epoch_seconds
            || evidence.sample_interval_seconds <= 0
        {
            return Err(format!(
                "evidence {} has invalid observation time or sample interval",
                evidence.evidence_id
            ));
        }
        finite("evidence coverage fraction", evidence.coverage_fraction)?;
        if !(0.0..=1.0).contains(&evidence.coverage_fraction) {
            return Err("evidence coverage fraction must be in [0,1]".into());
        }
        if let Some(value) = evidence.measured_rate_value {
            finite("measured flux", value)?;
            if value < 0.0 {
                return Err("measured flux cannot be negative".into());
            }
        }
        if let Some(value) = evidence.lower_rate_bound_value {
            finite("lower flux bound", value)?;
            if value < 0.0 {
                return Err("lower flux bound cannot be negative".into());
            }
        }
        if let Some(value) = evidence.upper_rate_bound_value {
            finite("upper flux bound", value)?;
            if value < 0.0 {
                return Err("upper flux bound cannot be negative".into());
            }
        }
        for (name, confidence) in [
            (
                "lower_rate_bound_confidence_level",
                evidence.lower_rate_bound_confidence_level,
            ),
            (
                "upper_rate_bound_confidence_level",
                evidence.upper_rate_bound_confidence_level,
            ),
        ] {
            if let Some(value) = confidence {
                finite(name, value)?;
                if !(0.0..=1.0).contains(&value) || value <= 0.0 {
                    return Err(format!("{name} must be in (0,1]"));
                }
            }
        }
        if evidence.lower_rate_bound_value.is_some() && evidence.measured_rate_value.is_none() {
            return Err("lower rate bound requires an associated measured rate".into());
        }
        if let (Some(lower), Some(upper)) = (
            evidence.lower_rate_bound_value,
            evidence.upper_rate_bound_value,
        ) && lower > upper
        {
            return Err("lower rate bound cannot exceed upper rate bound".into());
        }
        if let Some(measured) = evidence.measured_rate_value
            && (evidence
                .lower_rate_bound_value
                .is_some_and(|lower| lower > measured)
                || evidence
                    .upper_rate_bound_value
                    .is_some_and(|upper| upper < measured))
        {
            return Err("measured rate must lie within supplied bounds".into());
        }
        if is_direct_observation(&evidence.evidence_class)
            && evidence
                .calibration_id
                .as_deref()
                .unwrap_or("")
                .trim()
                .is_empty()
        {
            return Err(format!(
                "direct observation {} requires a calibration/reference identity",
                evidence.evidence_id
            ));
        }
        evidence_by_id.insert(evidence.evidence_id.as_str(), evidence);
    }

    let mut assertion_edges = BTreeSet::new();
    let mut referenced_evidence = BTreeSet::new();
    for assertion in &s.assertions {
        if !edge_ids.contains(assertion.edge_id.as_str())
            || !assertion_edges.insert(assertion.edge_id.as_str())
        {
            return Err("assertion edges must be registered and appear exactly once".into());
        }
        if !matches!(
            assertion.asserted_state.as_str(),
            "active" | "inactive" | "unassessed"
        ) {
            return Err("asserted state must be active, inactive or unassessed".into());
        }
        if assertion.asserted_state != "unassessed" && assertion.evidence_ids.is_empty() {
            return Err(format!(
                "assertion {} requires evidence references",
                assertion.edge_id
            ));
        }
        let mut local_refs = BTreeSet::new();
        for evidence_id in &assertion.evidence_ids {
            if !local_refs.insert(evidence_id.as_str())
                || !referenced_evidence.insert(evidence_id.as_str())
            {
                return Err("evidence references must be unique and consume-once".into());
            }
            let evidence = evidence_by_id
                .get(evidence_id.as_str())
                .ok_or_else(|| format!("assertion references unknown evidence {evidence_id}"))?;
            if evidence.edge_id != assertion.edge_id {
                return Err(format!(
                    "evidence {} does not belong to asserted edge {}",
                    evidence.evidence_id, assertion.edge_id
                ));
            }
        }
    }
    if assertion_edges != edge_ids {
        return Err("every registered graph edge requires exactly one state assertion".into());
    }
    if referenced_evidence.len() != evidence_by_id.len() {
        return Err("unreferenced evidence records are not permitted".into());
    }
    Ok(())
}

fn evidence_candidate(
    s: &Scenario,
    evidence: &EvidenceRecord,
) -> (Option<CandidateState>, Vec<String>) {
    let mut reasons = Vec::new();
    if evidence.specimen_id != s.specimen_id {
        reasons.push("specimen_support_mismatch".into());
    }
    if evidence.event_id != s.event.event_id {
        reasons.push("event_identity_mismatch".into());
    }
    if evidence.observed_end_epoch_seconds < s.event.start_epoch_seconds
        || evidence.observed_start_epoch_seconds > s.event.end_epoch_seconds
    {
        reasons.push("outside_event_window".into());
    }
    if evidence.qc_status != "passed" {
        reasons.push("quality_control_not_passed".into());
    }
    if evidence.spatial_localization != "edge_resolved"
        || evidence.support_kind != "registered_3d_edge"
        || evidence.registration_status != "registered"
        || evidence
            .registration_transform_id
            .as_deref()
            .unwrap_or("")
            .trim()
            .is_empty()
    {
        reasons.push("evidence_is_not_spatially_resolved_to_one_3d_edge".into());
    }
    if evidence.support_overlap_fraction < s.event.minimum_support_overlap_fraction {
        reasons.push("edge_support_overlap_below_threshold".into());
    }
    if evidence
        .registration_error_mm
        .is_none_or(|error| error > s.event.maximum_registration_error_mm)
    {
        reasons.push("registration_error_exceeds_admission_limit".into());
    }
    if evidence.support_kind != "registered_3d_edge"
        || evidence.registration_status != "registered"
        || evidence
            .registration_transform_id
            .as_deref()
            .unwrap_or("")
            .trim()
            .is_empty()
    {
        reasons.push("spatial_support_not_registered_to_3d_edge".into());
    }
    if !is_direct_observation(&evidence.evidence_class) {
        reasons.push("non_direct_evidence_cannot_confirm_hydraulic_state".into());
    }
    if !reasons.is_empty() {
        return (None, reasons);
    }

    let threshold = match (
        evidence.rate_measurand.as_str(),
        evidence.rate_unit.as_str(),
    ) {
        ("tracer_mass_flux", "mg/h") => s.event.minimum_relevant_tracer_mass_flux_mg_per_h,
        ("water_volume_flux", "mL/h") => s.event.minimum_relevant_water_volume_flux_ml_per_h,
        _ => {
            return (None, vec!["unsupported_rate_measurand_or_unit".into()]);
        }
    };
    let measured_active = evidence
        .lower_rate_bound_value
        .is_some_and(|value| value > threshold)
        && evidence
            .lower_rate_bound_confidence_level
            .is_some_and(|value| value >= s.event.minimum_bound_confidence_level);
    if measured_active {
        return (Some(CandidateState::Active), vec![]);
    }

    let covers_entire_event = evidence.observed_start_epoch_seconds <= s.event.start_epoch_seconds
        && evidence.observed_end_epoch_seconds >= s.event.end_epoch_seconds;
    let cadence_adequate =
        evidence.sample_interval_seconds <= s.event.maximum_sample_interval_seconds;
    let coverage_adequate = evidence.coverage_fraction >= s.event.minimum_coverage_fraction;
    let non_detection_bound_adequate = evidence
        .upper_rate_bound_value
        .is_some_and(|value| value <= threshold)
        && evidence
            .upper_rate_bound_confidence_level
            .is_some_and(|value| value >= s.event.minimum_bound_confidence_level);

    if covers_entire_event
        && cadence_adequate
        && coverage_adequate
        && non_detection_bound_adequate
        && evidence.measured_rate_value.is_none()
    {
        return (Some(CandidateState::Inactive), vec![]);
    }

    if !covers_entire_event {
        reasons.push("negative_evidence_does_not_cover_entire_event".into());
    }
    if !cadence_adequate {
        reasons.push("sampling_cadence_too_sparse".into());
    }
    if !coverage_adequate {
        reasons.push("event_sampling_coverage_insufficient".into());
    }
    if !non_detection_bound_adequate {
        reasons.push("detection_upper_bound_does_not_exclude_relevant_flux".into());
    }
    if evidence
        .measured_rate_value
        .is_some_and(|value| value > threshold)
    {
        reasons.push("positive_point_estimate_without_qualified_lower_bound".into());
    } else if evidence.measured_rate_value.is_some() {
        reasons.push("measured_value_does_not_support_declared_state".into());
    }
    (None, reasons)
}

fn evaluate_assertion(
    s: &Scenario,
    assertion: &EdgeAssertion,
    evidence: &BTreeMap<&str, &EvidenceRecord>,
) -> EdgeResult {
    if assertion.asserted_state == "unassessed" {
        return EdgeResult {
            edge_id: assertion.edge_id.clone(),
            asserted_state: assertion.asserted_state.clone(),
            admission_status: "unassessed".into(),
            qualifying_active_evidence_ids: vec![],
            qualifying_inactive_evidence_ids: vec![],
            reasons: vec!["assertion_explicitly_unassessed".into()],
        };
    }

    let mut active_ids = Vec::new();
    let mut inactive_ids = Vec::new();
    let mut reasons = Vec::new();
    for evidence_id in &assertion.evidence_ids {
        let record = evidence
            .get(evidence_id.as_str())
            .expect("evidence references validated");
        let (candidate, evidence_reasons) = evidence_candidate(s, record);
        if evidence_reasons.is_empty() {
            match candidate {
                Some(CandidateState::Active) => active_ids.push(evidence_id.clone()),
                Some(CandidateState::Inactive) => inactive_ids.push(evidence_id.clone()),
                None => reasons.push("evidence_no_qualified_state".into()),
            }
        } else {
            reasons.extend(
                evidence_reasons
                    .into_iter()
                    .map(|reason| format!("{evidence_id}:{reason}")),
            );
        }
    }

    let has_active = !active_ids.is_empty();
    let has_inactive = !inactive_ids.is_empty();
    let admission_status = if has_active && has_inactive {
        reasons.push("conflicting_direct_evidence".into());
        "unassessed"
    } else if assertion.asserted_state == "active" && has_active {
        "confirmed_active"
    } else if assertion.asserted_state == "inactive" && has_inactive {
        "confirmed_inactive"
    } else if has_active || has_inactive {
        reasons.push("assertion_conflicts_with_qualified_direct_evidence".into());
        "unassessed"
    } else {
        reasons.push(if assertion.asserted_state == "active" {
            "no_qualifying_positive_hydraulic_evidence".into()
        } else {
            "no_qualifying_negative_evidence".into()
        });
        "unassessed"
    };

    EdgeResult {
        edge_id: assertion.edge_id.clone(),
        asserted_state: assertion.asserted_state.clone(),
        admission_status: admission_status.into(),
        qualifying_active_evidence_ids: active_ids,
        qualifying_inactive_evidence_ids: inactive_ids,
        reasons,
    }
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let evidence: BTreeMap<&str, &EvidenceRecord> = s
        .evidence_records
        .iter()
        .map(|record| (record.evidence_id.as_str(), record))
        .collect();
    let results: Vec<EdgeResult> = s
        .assertions
        .iter()
        .map(|assertion| evaluate_assertion(s, assertion, &evidence))
        .collect();
    let confirmed_active_count = results
        .iter()
        .filter(|result| result.admission_status == "confirmed_active")
        .count();
    let confirmed_inactive_count = results
        .iter()
        .filter(|result| result.admission_status == "confirmed_inactive")
        .count();
    let unassessed_count = results
        .iter()
        .filter(|result| result.admission_status == "unassessed")
        .count();

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        assertion_count: results.len(),
        confirmed_active_count,
        confirmed_inactive_count,
        unassessed_count,
        results,
        controls: Controls {
            direct_observation_required_for_confirmed_hydraulic_state: true,
            structural_image_cannot_confirm_hydraulic_activation: true,
            model_prediction_cannot_confirm_hydraulic_activation: true,
            inactivity_requires_event_coverage_and_detection_bound: true,
            conflicting_direct_evidence_abstains: true,
            specimen_event_and_registered_3d_edge_support_required: true,
            synthetic_only_claim_ceiling: true,
        },
        interpretation_boundary: CLAIM_CEILING,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: activation-evidence-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("activation evidence audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-activation-evidence.json"
        ))
        .expect("synthetic fixture parses")
    }

    fn results() -> Vec<EdgeResult> {
        let s = scenario();
        validate(&s).expect("fixture validates");
        build_report(&s, "0".repeat(64)).results
    }

    #[test]
    fn direct_event_matched_tracer_can_confirm_activity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-active-tracer")
            .unwrap();
        assert_eq!(edge.admission_status, "confirmed_active");
    }

    #[test]
    fn hydraulic_observation_uses_water_volume_rate_and_its_own_threshold() {
        let s = scenario();
        let mut evidence = s.evidence_records[0].clone();
        evidence.evidence_class = "direct_hydraulic_observation".into();
        evidence.rate_measurand = "water_volume_flux".into();
        evidence.rate_unit = "mL/h".into();
        let (state, reasons) = evidence_candidate(&s, &evidence);
        assert_eq!(state, Some(CandidateState::Active));
        assert!(reasons.is_empty());
    }

    #[test]
    fn point_estimate_without_lower_confidence_bound_cannot_confirm_activity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-active-point-only")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
        assert!(
            edge.reasons
                .iter()
                .any(|r| r.contains("positive_point_estimate_without_qualified_lower_bound"))
        );
    }

    #[test]
    fn qualified_full_window_non_detection_can_confirm_inactivity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-inactive-covered")
            .unwrap();
        assert_eq!(edge.admission_status, "confirmed_inactive");
    }

    #[test]
    fn structural_image_cannot_confirm_hydraulic_activity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-geometry-only")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
    }

    #[test]
    fn sparse_non_detection_cannot_confirm_inactivity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-inactive-sparse")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
        assert!(
            edge.reasons
                .iter()
                .any(|r| r.contains("sampling_cadence_too_sparse"))
        );
    }

    #[test]
    fn boundary_integrated_tracer_cannot_identify_one_active_edge() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-boundary-tracer")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
        assert!(
            edge.reasons
                .iter()
                .any(|r| r.contains("not_spatially_resolved"))
        );
    }

    #[test]
    fn registration_error_above_limit_abstains() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-high-registration-error")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
        assert!(
            edge.reasons
                .iter()
                .any(|r| r.contains("registration_error_exceeds"))
        );
    }

    #[test]
    fn model_prediction_cannot_confirm_activity() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-model-only")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
    }

    #[test]
    fn conflicting_direct_evidence_abstains() {
        let result = results();
        let edge = result
            .iter()
            .find(|r| r.edge_id == "edge-conflict")
            .unwrap();
        assert_eq!(edge.admission_status, "unassessed");
        assert!(
            edge.reasons
                .iter()
                .any(|r| r == "conflicting_direct_evidence")
        );
    }

    #[test]
    fn wrong_event_or_unregistered_spatial_support_abstains() {
        let result = results();
        let wrong_event = result
            .iter()
            .find(|r| r.edge_id == "edge-wrong-event")
            .unwrap();
        let unregistered = result
            .iter()
            .find(|r| r.edge_id == "edge-unregistered")
            .unwrap();
        assert_eq!(wrong_event.admission_status, "unassessed");
        assert_eq!(unregistered.admission_status, "unassessed");
    }

    #[test]
    fn evidence_support_identity_must_match_registered_edge_manifest() {
        let mut s = scenario();
        s.evidence_records[0].spatial_support_id = "wrong-support".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn malformed_artifact_digest_rejects_input() {
        let mut s = scenario();
        s.evidence_records[0].source_artifact_sha256 = "bad".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn missing_edge_assertion_rejects_input() {
        let mut s = scenario();
        s.assertions.pop();
        assert!(validate(&s).is_err());
    }
}
