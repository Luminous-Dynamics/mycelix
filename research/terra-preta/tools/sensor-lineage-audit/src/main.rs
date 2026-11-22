//! Synthetic-only provenance validator for sensor observations and derived soil indicators.
//!
//! A raw sensor signal and a model-derived soil property are different evidence
//! records. This validator checks their graph edges and split lineage; it does
//! not fit/validate calibration accuracy or qualify a real agronomic use.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic provenance and semantic audit only. Raw instrument signals are not soil-property assays. Derived estimates are not direct measurements. No calibration accuracy, predictive calibration, field applicability or agronomic decision is qualified.";

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    instruments: Vec<Instrument>,
    sensor_observations: Vec<SensorObservation>,
    reference_assays: Vec<ReferenceAssay>,
    derived_estimates: Vec<DerivedEstimate>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Instrument {
    instrument_id: String,
    manufacturer_model: String,
    serial_id: String,
    firmware_version: String,
    calibration_id: String,
    calibration_sha256: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct SensorObservation {
    observation_id: String,
    physical_sample_id: String,
    independence_group_id: String,
    instrument_id: String,
    modality: String,
    signal_type: String,
    signal_unit: String,
    measurement_semantics: String,
    calibration_id: String,
    acquired_at: String,
    sample_preparation: String,
    sample_moisture_status: String,
    measurement_geometry: String,
    raw_artifact: RawArtifact,
    qa_status: String,
    replicate_group_id: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct RawArtifact {
    uri: String,
    sha256: String,
    byte_size: u64,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct ReferenceAssay {
    assay_id: String,
    physical_sample_id: String,
    independence_group_id: String,
    target_property: String,
    value: f64,
    unit: String,
    method: String,
    extraction_basis: String,
    depth_min_m: f64,
    depth_max_m: f64,
    standard_error: f64,
    role: String,
    laboratory_id: String,
    batch_id: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct DerivedEstimate {
    estimate_id: String,
    physical_sample_id: String,
    independence_group_id: String,
    target_property: String,
    unit: String,
    value: f64,
    standard_error: f64,
    measurement_semantics: String,
    input_observation_ids: Vec<String>,
    model: ModelLineage,
    assessment_status: String,
    reference_assay_id: Option<String>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct ModelLineage {
    model_id: String,
    model_sha256: String,
    preprocessing_sha256: String,
    training_sample_ids: Vec<String>,
    training_independence_group_ids: Vec<String>,
    calibration_reference_assay_ids: Vec<String>,
    domain_status: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    instrument_count: usize,
    sensor_observation_count: usize,
    reference_assay_count: usize,
    derived_estimate_count: usize,
    estimate_results: Vec<EstimateResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct EstimateResult {
    estimate_id: String,
    physical_sample_id: String,
    target_property: String,
    measurement_semantics: &'static str,
    input_observation_count: usize,
    model_id: String,
    model_sha256: String,
    preprocessing_sha256: String,
    domain_status: String,
    assessment_status: String,
    reference_assay_status: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    sensor_signal_not_promoted_to_soil_property: bool,
    estimates_not_promoted_to_reference_assays: bool,
    observation_edges_resolve: bool,
    training_sample_and_group_leakage_rejected: bool,
    ood_estimates_abstain: bool,
    model_and_preprocessing_digests_present: bool,
}

fn nonempty(name: &str, value: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{name} must be non-empty"));
    }
    Ok(())
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soil-sensor-lineage-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic scenario".into());
    }
    nonempty("scenario_id", &s.scenario_id)?;
    if s.instruments.is_empty()
        || s.sensor_observations.is_empty()
        || s.derived_estimates.is_empty()
    {
        return Err(
            "at least one instrument, sensor observation and derived estimate are required".into(),
        );
    }

    let mut instruments = BTreeSet::new();
    let mut calibration_ids = BTreeSet::new();
    for instrument in &s.instruments {
        for (name, value) in [
            ("instrument_id", instrument.instrument_id.as_str()),
            ("manufacturer_model", instrument.manufacturer_model.as_str()),
            ("serial_id", instrument.serial_id.as_str()),
            ("firmware_version", instrument.firmware_version.as_str()),
            ("calibration_id", instrument.calibration_id.as_str()),
        ] {
            nonempty(name, value)?;
        }
        if !instruments.insert(instrument.instrument_id.clone())
            || !calibration_ids.insert(instrument.calibration_id.clone())
        {
            return Err("instrument and calibration IDs must be unique".into());
        }
        if !valid_sha256(&instrument.calibration_sha256) {
            return Err(format!(
                "instrument {} has invalid calibration SHA-256",
                instrument.instrument_id
            ));
        }
    }

    let mut observations = BTreeSet::new();
    for o in &s.sensor_observations {
        for (name, value) in [
            ("observation_id", o.observation_id.as_str()),
            ("physical_sample_id", o.physical_sample_id.as_str()),
            ("independence_group_id", o.independence_group_id.as_str()),
            ("modality", o.modality.as_str()),
            ("signal_type", o.signal_type.as_str()),
            ("signal_unit", o.signal_unit.as_str()),
            ("calibration_id", o.calibration_id.as_str()),
            ("acquired_at", o.acquired_at.as_str()),
            ("sample_preparation", o.sample_preparation.as_str()),
            ("sample_moisture_status", o.sample_moisture_status.as_str()),
            ("measurement_geometry", o.measurement_geometry.as_str()),
            ("replicate_group_id", o.replicate_group_id.as_str()),
        ] {
            nonempty(name, value)?;
        }
        if !observations.insert(o.observation_id.clone()) {
            return Err("sensor observation IDs must be unique".into());
        }
        if !instruments.contains(&o.instrument_id) {
            return Err(format!(
                "observation {} references unknown instrument",
                o.observation_id
            ));
        }
        if !calibration_ids.contains(&o.calibration_id) {
            return Err(format!(
                "observation {} references unknown calibration",
                o.observation_id
            ));
        }
        if o.measurement_semantics != "direct_sensor_signal" {
            return Err(format!(
                "observation {} must be typed as direct_sensor_signal, not a soil-property measurement",
                o.observation_id
            ));
        }
        if !matches!(
            o.qa_status.as_str(),
            "pass" | "flagged" | "rejected" | "unknown"
        ) {
            return Err("unsupported sensor observation QA status".into());
        }
        if o.qa_status != "pass" {
            // V0.1 does not let a non-passing signal silently support an accepted candidate.
            // It may remain in the observation archive, but not be linked to a derived estimate.
        }
        if !o.raw_artifact.uri.starts_with("https://")
            || !valid_sha256(&o.raw_artifact.sha256)
            || o.raw_artifact.byte_size == 0
        {
            return Err(format!(
                "observation {} has incomplete raw artifact identity",
                o.observation_id
            ));
        }
    }

    let mut assays = BTreeSet::new();
    for assay in &s.reference_assays {
        for (name, value) in [
            ("assay_id", assay.assay_id.as_str()),
            ("physical_sample_id", assay.physical_sample_id.as_str()),
            (
                "independence_group_id",
                assay.independence_group_id.as_str(),
            ),
            ("target_property", assay.target_property.as_str()),
            ("unit", assay.unit.as_str()),
            ("method", assay.method.as_str()),
            ("extraction_basis", assay.extraction_basis.as_str()),
            ("laboratory_id", assay.laboratory_id.as_str()),
            ("batch_id", assay.batch_id.as_str()),
        ] {
            nonempty(name, value)?;
        }
        if !assays.insert(assay.assay_id.clone()) {
            return Err("reference assay IDs must be unique".into());
        }
        if !matches!(
            assay.role.as_str(),
            "calibration_reference" | "independent_validation_reference"
        ) {
            return Err(format!(
                "assay {} has unsupported role",
                assay.assay_id
            ));
        }
        for (name, value) in [
            ("reference assay value", assay.value),
            ("reference assay standard error", assay.standard_error),
            ("reference depth minimum", assay.depth_min_m),
            ("reference depth maximum", assay.depth_max_m),
        ] {
            finite(name, value)?;
        }
        if assay.standard_error < 0.0
            || assay.depth_min_m < 0.0
            || assay.depth_max_m <= assay.depth_min_m
        {
            return Err(format!(
                "assay {} has invalid uncertainty or depth range",
                assay.assay_id
            ));
        }
    }

    let assay_map: std::collections::BTreeMap<&str, &ReferenceAssay> = s
        .reference_assays
        .iter()
        .map(|a| (a.assay_id.as_str(), a))
        .collect();

    let mut estimates = BTreeSet::new();
    for estimate in &s.derived_estimates {
        for (name, value) in [
            ("estimate_id", estimate.estimate_id.as_str()),
            ("physical_sample_id", estimate.physical_sample_id.as_str()),
            (
                "independence_group_id",
                estimate.independence_group_id.as_str(),
            ),
            ("target_property", estimate.target_property.as_str()),
            ("unit", estimate.unit.as_str()),
            ("model_id", estimate.model.model_id.as_str()),
        ] {
            nonempty(name, value)?;
        }
        if !estimates.insert(estimate.estimate_id.clone()) {
            return Err("derived estimate IDs must be unique".into());
        }
        if estimate.measurement_semantics != "derived_prediction" {
            return Err(format!(
                "estimate {} must be typed as derived_prediction, not a direct assay",
                estimate.estimate_id
            ));
        }
        finite("derived estimate value", estimate.value)?;
        finite("derived estimate standard error", estimate.standard_error)?;
        if estimate.standard_error < 0.0 || estimate.input_observation_ids.is_empty() {
            return Err(format!(
                "estimate {} has invalid uncertainty or no input observations",
                estimate.estimate_id
            ));
        }
        if !valid_sha256(&estimate.model.model_sha256)
            || !valid_sha256(&estimate.model.preprocessing_sha256)
        {
            return Err(format!(
                "estimate {} lacks model/preprocessing SHA-256",
                estimate.estimate_id
            ));
        }
        if estimate.model.training_sample_ids.is_empty()
            || estimate.model.training_independence_group_ids.is_empty()
        {
            return Err(format!(
                "estimate {} must declare training sample and independence groups",
                estimate.estimate_id
            ));
        }
        if estimate
            .model
            .training_sample_ids
            .contains(&estimate.physical_sample_id)
            || estimate
                .model
                .training_independence_group_ids
                .contains(&estimate.independence_group_id)
        {
            return Err(format!(
                "estimate {} leaks its evaluation sample/group into model training",
                estimate.estimate_id
            ));
        }
        if !matches!(
            estimate.model.domain_status.as_str(),
            "in_domain" | "out_of_domain" | "unassessed"
        ) {
            return Err(format!(
                "estimate {} has unsupported domain status",
                estimate.estimate_id
            ));
        }
        if !matches!(
            estimate.assessment_status.as_str(),
            "candidate_unqualified" | "abstain"
        ) {
            return Err(format!(
                "estimate {} has unsupported assessment status",
                estimate.estimate_id
            ));
        }
        if estimate.model.domain_status != "in_domain" && estimate.assessment_status != "abstain" {
            return Err(format!(
                "estimate {} must abstain when out-of-domain or unassessed",
                estimate.estimate_id
            ));
        }

        for observation_id in &estimate.input_observation_ids {
            let observation = s
                .sensor_observations
                .iter()
                .find(|o| o.observation_id == *observation_id)
                .ok_or_else(|| {
                    format!(
                        "estimate {} references missing sensor observation",
                        estimate.estimate_id
                    )
                })?;
            if observation.physical_sample_id != estimate.physical_sample_id
                || observation.independence_group_id != estimate.independence_group_id
            {
                return Err(format!(
                    "estimate {} input observation belongs to a different sample/group",
                    estimate.estimate_id
                ));
            }
            if observation.qa_status != "pass" {
                return Err(format!(
                    "estimate {} uses a sensor observation that did not pass QA",
                    estimate.estimate_id
                ));
            }
            if observation.signal_type == estimate.target_property {
                return Err(format!(
                    "estimate {} cannot treat the raw sensor signal as the target soil property",
                    estimate.estimate_id
                ));
            }
        }

        for assay_id in &estimate.model.calibration_reference_assay_ids {
            let assay = assay_map.get(assay_id.as_str()).ok_or_else(|| {
                format!(
                    "estimate {} references missing calibration assay",
                    estimate.estimate_id
                )
            })?;
            if assay.role != "calibration_reference"
                || !estimate
                    .model
                    .training_sample_ids
                    .contains(&assay.physical_sample_id)
                || !estimate
                    .model
                    .training_independence_group_ids
                    .contains(&assay.independence_group_id)
            {
                return Err(format!(
                    "estimate {} calibration assay is inconsistent with model training lineage",
                    estimate.estimate_id
                ));
            }
        }

        if let Some(assay_id) = &estimate.reference_assay_id {
            let assay = assay_map.get(assay_id.as_str()).ok_or_else(|| {
                format!(
                    "estimate {} references missing reference assay",
                    estimate.estimate_id
                )
            })?;
            if assay.role != "independent_validation_reference"
                || assay.physical_sample_id != estimate.physical_sample_id
                || assay.independence_group_id != estimate.independence_group_id
                || assay.target_property != estimate.target_property
                || assay.unit != estimate.unit
            {
                return Err(format!(
                    "estimate {} reference assay is not an independent, support-matched validation reference",
                    estimate.estimate_id
                ));
            }
            if estimate
                .model
                .training_sample_ids
                .contains(&assay.physical_sample_id)
                || estimate
                    .model
                    .training_independence_group_ids
                    .contains(&assay.independence_group_id)
            {
                return Err(format!(
                    "estimate {} validation assay leaks into model training",
                    estimate.estimate_id
                ));
            }
        }
    }
    Ok(())
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let estimate_results = s
        .derived_estimates
        .iter()
        .map(|e| EstimateResult {
            estimate_id: e.estimate_id.clone(),
            physical_sample_id: e.physical_sample_id.clone(),
            target_property: e.target_property.clone(),
            measurement_semantics: "derived_prediction",
            input_observation_count: e.input_observation_ids.len(),
            model_id: e.model.model_id.clone(),
            model_sha256: e.model.model_sha256.clone(),
            preprocessing_sha256: e.model.preprocessing_sha256.clone(),
            domain_status: e.model.domain_status.clone(),
            assessment_status: e.assessment_status.clone(),
            reference_assay_status: if e.reference_assay_id.is_some() {
                "independent_validation_linked"
            } else if e.model.calibration_reference_assay_ids.is_empty() {
                "no_reference_assay_linked"
            } else {
                "calibration_reference_only"
            },
        })
        .collect();

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        instrument_count: s.instruments.len(),
        sensor_observation_count: s.sensor_observations.len(),
        reference_assay_count: s.reference_assays.len(),
        derived_estimate_count: s.derived_estimates.len(),
        estimate_results,
        controls: Controls {
            sensor_signal_not_promoted_to_soil_property: true,
            estimates_not_promoted_to_reference_assays: true,
            observation_edges_resolve: true,
            training_sample_and_group_leakage_rejected: true,
            ood_estimates_abstain: true,
            model_and_preprocessing_digests_present: true,
        },
        interpretation_boundary: BOUNDARY,
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
        eprintln!("usage: sensor-lineage-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("sensor lineage audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-nir-soc-lineage.json"))
            .expect("fixture parses")
    }

    #[test]
    fn synthetic_nir_signal_soc_estimate_and_reference_are_separate_records() {
        let s = scenario();
        validate(&s).expect("valid synthetic lineage");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.instrument_count, 1);
        assert_eq!(report.sensor_observation_count, 1);
        assert_eq!(report.reference_assay_count, 1);
        assert_eq!(report.derived_estimate_count, 1);
        assert_eq!(
            report.estimate_results[0].measurement_semantics,
            "derived_prediction"
        );
        assert_eq!(
            report.estimate_results[0].reference_assay_status,
            "independent_validation_linked"
        );
    }

    #[test]
    fn sensor_observation_cannot_be_labelled_as_direct_soil_property() {
        let mut s = scenario();
        s.sensor_observations[0].measurement_semantics =
            "directly_measured_soil_organic_carbon".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn derived_estimate_cannot_be_promoted_to_direct_assay() {
        let mut s = scenario();
        s.derived_estimates[0].measurement_semantics = "direct_measurement".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn model_and_preprocessing_digests_are_required() {
        let mut s = scenario();
        s.derived_estimates[0].model.preprocessing_sha256 = "missing".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn evaluation_sample_or_group_cannot_leak_into_training() {
        let mut s = scenario();
        s.derived_estimates[0]
            .model
            .training_sample_ids
            .push("sample-eval-001".into());
        assert!(validate(&s).is_err());
        let mut t = scenario();
        t.derived_estimates[0]
            .model
            .training_independence_group_ids
            .push("evaluation-site-A-field-01".into());
        assert!(validate(&t).is_err());
    }

    #[test]
    fn estimate_input_must_resolve_to_same_physical_sample() {
        let mut s = scenario();
        s.sensor_observations[0].physical_sample_id = "some-other-sample".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn out_of_domain_estimate_must_abstain() {
        let mut s = scenario();
        s.derived_estimates[0].model.domain_status = "out_of_domain".into();
        assert!(validate(&s).is_err());
        s.derived_estimates[0].assessment_status = "abstain".into();
        validate(&s).expect("OOD estimate remains explicitly abstained");
    }

    #[test]
    fn independent_reference_assay_must_match_sample_target_and_unit() {
        let mut s = scenario();
        s.reference_assays[0].target_property = "available_phosphorus".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn calibration_reference_assay_must_belong_to_training_lineage() {
        let mut s = scenario();
        let mut calibration = s.reference_assays[0].clone();
        calibration.assay_id = "calibration-reference-001".into();
        calibration.physical_sample_id = "training-sample-01".into();
        calibration.independence_group_id = "training-field-A".into();
        calibration.role = "calibration_reference".into();
        s.reference_assays.push(calibration);
        s.derived_estimates[0].model.calibration_reference_assay_ids =
            vec!["calibration-reference-001".into()];
        validate(&s).expect("calibration assay is part of declared training group");
    }
}
