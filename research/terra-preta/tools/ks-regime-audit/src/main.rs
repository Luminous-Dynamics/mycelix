use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::env;
use std::error::Error;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const RAC_BOUNDARY: f64 = 0.05;
const POROSITY_BOUNDARY: f64 = 0.40;
const MINIMUM_GROUPS_FOR_DESCRIPTIVE_STABILITY: usize = 5;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum EvaluationTrack {
    UniformMethod,
    MultiMethod,
}

impl EvaluationTrack {
    fn as_str(self) -> &'static str {
        match self {
            Self::UniformMethod => "uniform_method",
            Self::MultiMethod => "multi_method",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum SplitRole {
    HeldOutTest,
    ExternalValidation,
}

impl SplitRole {
    fn as_str(self) -> &'static str {
        match self {
            Self::HeldOutTest => "held_out_test",
            Self::ExternalValidation => "external_validation",
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModelDeclaration {
    model_id: String,
    model_family: String,
    model_version: String,
    configuration_sha256: String,
    implementation_evidence_ref: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    sample_id: String,
    group_id: String,
    ks_measurement_method: String,
    ks_measurement_protocol_id: String,
    water_retention_method: String,
    water_retention_protocol_id: String,
    rac_fraction: f64,
    rac_method: String,
    rac_evidence_ref: String,
    total_porosity_fraction: f64,
    actual_ks_m_per_s: f64,
    predictions_m_per_s: BTreeMap<String, f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_label: String,
    evaluation_track: EvaluationTrack,
    split_role: SplitRole,
    source_sha256: String,
    split_manifest_sha256: String,
    source_evidence_ref: String,
    split_evidence_ref: String,
    models: Vec<ModelDeclaration>,
    observations: Vec<Observation>,
}

#[derive(Clone, Debug, Serialize)]
struct ModelMetrics {
    model_id: String,
    model_family: String,
    model_version: String,
    configuration_sha256: String,
    observations: usize,
    independent_groups: usize,
    mae_log10_m_per_s: f64,
    rmse_log10_m_per_s: f64,
    bias_prediction_minus_observed_log10_m_per_s: f64,
    underprediction_fraction: f64,
    overprediction_fraction: f64,
    unbiased_fraction_within_half_sigma_y: f64,
    material_underprediction_fraction: f64,
    material_overprediction_fraction: f64,
}

#[derive(Clone, Debug, Serialize)]
struct StratumResult {
    stratum_id: String,
    description: String,
    observations: usize,
    independent_groups: usize,
    minimum_group_gate: &'static str,
    model_metrics: Vec<ModelMetrics>,
    interpretation_boundary: &'static str,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    split_manifest_sha256: String,
    study_label: String,
    evaluation_track: &'static str,
    split_role: &'static str,
    unit_basis: &'static str,
    residual_definition: &'static str,
    rac_definition: &'static str,
    rac_boundary_fraction: f64,
    total_porosity_boundary_fraction: f64,
    observed_log10_ks_sample_standard_deviation: f64,
    half_sigma_y_log10_threshold: f64,
    observation_count: usize,
    independent_group_count: usize,
    ks_measurement_methods: Vec<String>,
    water_retention_methods: Vec<String>,
    model_declarations: Vec<ModelDeclaration>,
    strata: Vec<StratumResult>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    if value.trim() != value {
        return Err(format!("{label} must not contain surrounding whitespace"));
    }
    Ok(())
}

fn valid_sha256(value: &str, label: &str) -> Result<(), String> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("{label} must contain exactly 64 hexadecimal characters"));
    }
    Ok(())
}

fn validate(input: &Input) -> Result<(Vec<String>, Vec<String>, Vec<f64>), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    nonblank(&input.study_label, "study_label")?;
    nonblank(&input.source_evidence_ref, "source_evidence_ref")?;
    nonblank(&input.split_evidence_ref, "split_evidence_ref")?;
    valid_sha256(&input.source_sha256, "source_sha256")?;
    valid_sha256(&input.split_manifest_sha256, "split_manifest_sha256")?;
    if input.split_role != SplitRole::HeldOutTest
        && input.split_role != SplitRole::ExternalValidation
    {
        return Err("evaluation must use a held-out test or external-validation split".to_owned());
    }
    if input.models.len() < 2 {
        return Err("at least two model declarations are required for comparison".to_owned());
    }
    if input.observations.is_empty() {
        return Err("observations must not be empty".to_owned());
    }

    let mut model_ids = BTreeSet::new();
    for (index, model) in input.models.iter().enumerate() {
        nonblank(&model.model_id, &format!("models[{index}].model_id"))?;
        nonblank(&model.model_family, &format!("models[{index}].model_family"))?;
        nonblank(&model.model_version, &format!("models[{index}].model_version"))?;
        nonblank(
            &model.implementation_evidence_ref,
            &format!("models[{index}].implementation_evidence_ref"),
        )?;
        valid_sha256(
            &model.configuration_sha256,
            &format!("models[{index}].configuration_sha256"),
        )?;
        if !model_ids.insert(model.model_id.as_str()) {
            return Err(format!("duplicate model_id {:?}", model.model_id));
        }
    }

    let expected_models: BTreeSet<String> = model_ids.iter().map(|value| (*value).to_owned()).collect();
    let mut sample_ids = HashSet::new();
    let mut groups = BTreeSet::new();
    let mut ks_methods = BTreeSet::new();
    let mut retention_methods = BTreeSet::new();
    let mut observed_log_ks = Vec::with_capacity(input.observations.len());

    for (index, row) in input.observations.iter().enumerate() {
        for (value, label) in [
            (&row.sample_id, "sample_id"),
            (&row.group_id, "group_id"),
            (&row.ks_measurement_method, "ks_measurement_method"),
            (&row.ks_measurement_protocol_id, "ks_measurement_protocol_id"),
            (&row.water_retention_method, "water_retention_method"),
            (&row.water_retention_protocol_id, "water_retention_protocol_id"),
            (&row.rac_method, "rac_method"),
            (&row.rac_evidence_ref, "rac_evidence_ref"),
        ] {
            nonblank(value, &format!("observations[{index}].{label}"))?;
        }
        if !sample_ids.insert(row.sample_id.as_str()) {
            return Err(format!("duplicate sample_id {:?}", row.sample_id));
        }
        groups.insert(row.group_id.as_str());
        ks_methods.insert(row.ks_measurement_method.clone());
        retention_methods.insert(row.water_retention_method.clone());

        if !row.rac_fraction.is_finite() || !(0.0..=1.0).contains(&row.rac_fraction) {
            return Err(format!("observations[{index}].rac_fraction must be finite and within [0, 1]"));
        }
        if !row.total_porosity_fraction.is_finite()
            || !(0.0..=1.0).contains(&row.total_porosity_fraction)
        {
            return Err(format!("observations[{index}].total_porosity_fraction must be finite and within [0, 1]"));
        }
        if !row.actual_ks_m_per_s.is_finite() || row.actual_ks_m_per_s <= 0.0 {
            return Err(format!("observations[{index}].actual_ks_m_per_s must be finite and strictly positive"));
        }

        let prediction_ids: BTreeSet<String> = row.predictions_m_per_s.keys().cloned().collect();
        if prediction_ids != expected_models {
            return Err(format!("observations[{index}] model prediction roster differs from declared model roster"));
        }
        for (model_id, prediction) in &row.predictions_m_per_s {
            if !prediction.is_finite() || *prediction <= 0.0 {
                return Err(format!("observations[{index}].predictions_m_per_s[{model_id:?}] must be finite and strictly positive"));
            }
        }
        observed_log_ks.push(row.actual_ks_m_per_s.log10());
    }

    if groups.len() < 2 {
        return Err("at least two independently reviewed group_id values are required".to_owned());
    }
    if input.evaluation_track == EvaluationTrack::UniformMethod {
        if ks_methods.len() != 1 || retention_methods.len() != 1 {
            return Err("uniform_method track requires exactly one Ks measurement method and one water-retention method".to_owned());
        }
    }
    if input.evaluation_track == EvaluationTrack::MultiMethod
        && ks_methods.len() < 2
        && retention_methods.len() < 2
    {
        return Err("multi_method track requires at least two distinct Ks or water-retention methods".to_owned());
    }

    Ok((
        ks_methods.into_iter().collect(),
        retention_methods.into_iter().collect(),
        observed_log_ks,
    ))
}

fn sample_std(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter().map(|value| (value - mean).powi(2)).sum::<f64>()
        / (values.len() - 1) as f64;
    variance.sqrt()
}

fn strata(input: &Input) -> Vec<(String, String, Vec<usize>)> {
    let mut output = Vec::new();
    let all: Vec<usize> = (0..input.observations.len()).collect();
    output.push(("all".to_owned(), "All evaluation samples".to_owned(), all));

    for (id, description, predicate) in [
        ("rac_lt_5pct", "Relative air capacity < 5%", 0usize),
        ("rac_ge_5pct", "Relative air capacity >= 5%", 1usize),
        ("porosity_le_40pct", "Total porosity <= 40%", 2usize),
        ("porosity_gt_40pct", "Total porosity > 40%", 3usize),
    ] {
        let indices = input.observations.iter().enumerate().filter_map(|(index, row)| {
            let selected = match predicate {
                0 => row.rac_fraction < RAC_BOUNDARY,
                1 => row.rac_fraction >= RAC_BOUNDARY,
                2 => row.total_porosity_fraction <= POROSITY_BOUNDARY,
                _ => row.total_porosity_fraction > POROSITY_BOUNDARY,
            };
            selected.then_some(index)
        }).collect();
        output.push((id.to_owned(), description.to_owned(), indices));
    }

    let mut ks_methods = BTreeSet::new();
    let mut retention_methods = BTreeSet::new();
    for row in &input.observations {
        ks_methods.insert(row.ks_measurement_method.clone());
        retention_methods.insert(row.water_retention_method.clone());
    }
    for method in ks_methods {
        let indices = input.observations.iter().enumerate().filter_map(|(index, row)| {
            (row.ks_measurement_method == method).then_some(index)
        }).collect();
        output.push((
            format!("ks_method::{method}"),
            format!("Ks measurement method: {method}"),
            indices,
        ));
    }
    for method in retention_methods {
        let indices = input.observations.iter().enumerate().filter_map(|(index, row)| {
            (row.water_retention_method == method).then_some(index)
        }).collect();
        output.push((
            format!("water_retention_method::{method}"),
            format!("Water-retention method: {method}"),
            indices,
        ));
    }

    let mut cross_methods = BTreeSet::new();
    for row in &input.observations {
        cross_methods.insert((
            row.rac_fraction < RAC_BOUNDARY,
            row.ks_measurement_method.clone(),
        ));
        cross_methods.insert((
            row.rac_fraction >= RAC_BOUNDARY,
            row.ks_measurement_method.clone(),
        ));
    }
    for (rac_low, method) in cross_methods {
        let indices = input.observations.iter().enumerate().filter_map(|(index, row)| {
            ((row.rac_fraction < RAC_BOUNDARY) == rac_low && row.ks_measurement_method == method)
                .then_some(index)
        }).collect();
        let rac = if rac_low { "RAC < 5%" } else { "RAC >= 5%" };
        output.push((
            format!("{}::ks_method::{method}", if rac_low { "rac_lt_5pct" } else { "rac_ge_5pct" }),
            format!("{rac}; Ks measurement method: {method}"),
            indices,
        ));
    }
    output
}

fn model_metrics(
    input: &Input,
    model: &ModelDeclaration,
    indices: &[usize],
    half_sigma_y: f64,
) -> ModelMetrics {
    let count = indices.len() as f64;
    let mut absolute_error = 0.0;
    let mut squared_error = 0.0;
    let mut signed_prediction_bias = 0.0;
    let mut under = 0usize;
    let mut over = 0usize;
    let mut unbiased = 0usize;
    let mut material_under = 0usize;
    let mut material_over = 0usize;
    let mut group_ids = BTreeSet::new();

    for index in indices {
        let row = &input.observations[*index];
        group_ids.insert(row.group_id.as_str());
        let predicted_log = row.predictions_m_per_s[&model.model_id].log10();
        let observed_log = row.actual_ks_m_per_s.log10();
        let prediction_minus_observed = predicted_log - observed_log;
        let observed_minus_prediction = -prediction_minus_observed;
        absolute_error += prediction_minus_observed.abs();
        squared_error += prediction_minus_observed.powi(2);
        signed_prediction_bias += prediction_minus_observed;
        if prediction_minus_observed < 0.0 {
            under += 1;
        } else if prediction_minus_observed > 0.0 {
            over += 1;
        }
        if observed_minus_prediction.abs() <= half_sigma_y {
            unbiased += 1;
        }
        if observed_minus_prediction > half_sigma_y {
            material_under += 1;
        }
        if observed_minus_prediction < -half_sigma_y {
            material_over += 1;
        }
    }

    ModelMetrics {
        model_id: model.model_id.clone(),
        model_family: model.model_family.clone(),
        model_version: model.model_version.clone(),
        configuration_sha256: model.configuration_sha256.clone(),
        observations: indices.len(),
        independent_groups: group_ids.len(),
        mae_log10_m_per_s: absolute_error / count,
        rmse_log10_m_per_s: (squared_error / count).sqrt(),
        bias_prediction_minus_observed_log10_m_per_s: signed_prediction_bias / count,
        underprediction_fraction: under as f64 / count,
        overprediction_fraction: over as f64 / count,
        unbiased_fraction_within_half_sigma_y: unbiased as f64 / count,
        material_underprediction_fraction: material_under as f64 / count,
        material_overprediction_fraction: material_over as f64 / count,
    }
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (ks_methods, retention_methods, observed_log_ks) = validate(input)?;
    let sigma_y = sample_std(&observed_log_ks);
    let half_sigma_y = sigma_y * 0.5;
    let mut results = Vec::new();

    for (stratum_id, description, indices) in strata(input) {
        if indices.is_empty() {
            continue;
        }
        let group_count = indices.iter().map(|index| input.observations[*index].group_id.as_str())
            .collect::<BTreeSet<_>>().len();
        let metrics = input.models.iter().map(|model| {
            model_metrics(input, model, &indices, half_sigma_y)
        }).collect();
        results.push(StratumResult {
            stratum_id,
            description,
            observations: indices.len(),
            independent_groups: group_count,
            minimum_group_gate: if group_count >= MINIMUM_GROUPS_FOR_DESCRIPTIVE_STABILITY {
                "at_least_5_groups_descriptive"
            } else {
                "fewer_than_5_groups_exploratory_only"
            },
            model_metrics: metrics,
            interpretation_boundary: "Descriptive held-out stratum metrics only; not a model-selection or inferential result.",
        });
    }

    let mut warnings = vec![
        "This tool evaluates supplied held-out predictions; it does not fit Ks models or validate their governing equations.".to_owned(),
        "RAC is supplied as a fraction of total porosity air-filled at -10 kPa and must be derived by a separately reviewed source method; it is not calculated here.".to_owned(),
        "RAC 5% and porosity 40% boundaries are literature-informed diagnostic strata from Qin et al. (2026), not universal physical transition laws.".to_owned(),
        "Measurement methods are treated as declared categories. Different protocols, sample supports, institutions and water-retention point densities can still differ within a category.".to_owned(),
        "All Ks values must be converted to SI m/s upstream using a recorded, independently checked conversion.".to_owned(),
        "Half-sigma_y residual classes use one pooled sample standard deviation of observed log10(Ks) for this evaluation track; they are descriptive and can be sensitive to the evaluation population.".to_owned(),
        "No confidence interval or significance test is computed. Strata with fewer than five independent groups are explicitly exploratory.".to_owned(),
        "A lower pooled error cannot compensate for material underprediction or poor calibration in a scientifically important regime.".to_owned(),
    ];

    if input.evaluation_track == EvaluationTrack::MultiMethod {
        warnings.push("Multi-method evaluation is reported separately from uniform-method evaluation; do not pool the tracks into a single headline result.".to_owned());
    }

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        split_manifest_sha256: input.split_manifest_sha256.clone(),
        study_label: input.study_label.clone(),
        evaluation_track: input.evaluation_track.as_str(),
        split_role: input.split_role.as_str(),
        unit_basis: "m_per_s; all scoring uses log10(m/s)",
        residual_definition: "observed_minus_predicted = log10(Ks_observed) - log10(Ks_predicted); material residual thresholds are +/- 0.5 sample standard deviations of observed log10(Ks)",
        rac_definition: "fraction of total porosity that is air-filled at -10 kPa; supplied and provenance-bound, not derived by this tool",
        rac_boundary_fraction: RAC_BOUNDARY,
        total_porosity_boundary_fraction: POROSITY_BOUNDARY,
        observed_log10_ks_sample_standard_deviation: sigma_y,
        half_sigma_y_log10_threshold: half_sigma_y,
        observation_count: input.observations.len(),
        independent_group_count: input.observations.iter().map(|row| row.group_id.as_str()).collect::<BTreeSet<_>>().len(),
        ks_measurement_methods: ks_methods,
        water_retention_methods: retention_methods,
        model_declarations: input.models.clone(),
        strata: results,
        warnings,
        interpretation_boundary: "Method- and RAC-stratified evaluation of supplied held-out Ks predictions. It is not a field calibration, South African transfer result, amendment effect or regulatory decision.",
    })
}

fn ensure_distinct_paths(input: &Path, output: &Path) -> Result<(), Box<dyn Error>> {
    if output.exists() && fs::canonicalize(input)? == fs::canonicalize(output)? {
        return Err("input and report resolve to the same file; refusing to overwrite source".into());
    }
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: ks-regime-audit <held-out-ks-predictions.json> <new-report.json>",
        ).into());
    }
    let input_path = Path::new(&args[1]);
    let output_path = Path::new(&args[2]);
    ensure_distinct_paths(input_path, output_path)?;
    let bytes = fs::read(input_path)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let mut encoded = serde_json::to_vec_pretty(&report)?;
    encoded.push(b'\n');
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.sync_all()?;
    println!(
        "Ks regime audit complete: {} samples; {} groups; {} models; {} strata; input SHA-256 {}",
        report.observation_count, report.independent_group_count,
        report.model_declarations.len(), report.strata.len(), report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("Ks regime audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model(id: &str, family: &str, hash: char) -> ModelDeclaration {
        ModelDeclaration {
            model_id: id.to_owned(),
            model_family: family.to_owned(),
            model_version: "fixture-0.1".to_owned(),
            configuration_sha256: hash.to_string().repeat(64),
            implementation_evidence_ref: "synthetic fixture only".to_owned(),
        }
    }

    fn row(
        sample: &str,
        group: &str,
        ks_method: &str,
        wr_method: &str,
        rac: f64,
        porosity: f64,
        actual: f64,
    ) -> Observation {
        Observation {
            sample_id: sample.to_owned(),
            group_id: group.to_owned(),
            ks_measurement_method: ks_method.to_owned(),
            ks_measurement_protocol_id: format!("KS-{ks_method}"),
            water_retention_method: wr_method.to_owned(),
            water_retention_protocol_id: format!("WR-{wr_method}"),
            rac_fraction: rac,
            rac_method: "fixture-rac-method".to_owned(),
            rac_evidence_ref: "synthetic fixture only".to_owned(),
            total_porosity_fraction: porosity,
            actual_ks_m_per_s: actual,
            predictions_m_per_s: BTreeMap::from([
                ("brooks_corey_integral".to_owned(), actual * 0.8),
                ("van_genuchten_integral".to_owned(), actual * 1.1),
                ("rosetta3".to_owned(), actual * 1.0),
            ]),
        }
    }

    fn fixture() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_label: "synthetic fixture".to_owned(),
            evaluation_track: EvaluationTrack::MultiMethod,
            split_role: SplitRole::HeldOutTest,
            source_sha256: "a".repeat(64),
            split_manifest_sha256: "b".repeat(64),
            source_evidence_ref: "synthetic only".to_owned(),
            split_evidence_ref: "synthetic split only".to_owned(),
            models: vec![
                model("brooks_corey_integral", "pore_spectrum_integral", 'c'),
                model("van_genuchten_integral", "pore_spectrum_integral", 'd'),
                model("rosetta3", "pedotransfer_function", 'e'),
            ],
            observations: vec![
                row("S1", "SITE-A", "falling_head", "pressure_plate", 0.08, 0.50, 1.0e-5),
                row("S2", "SITE-A", "falling_head", "pressure_plate", 0.03, 0.40, 1.0e-6),
                row("S3", "SITE-B", "steady_state", "hanging_column", 0.10, 0.35, 2.0e-5),
                row("S4", "SITE-B", "steady_state", "hanging_column", 0.01, 0.30, 8.0e-7),
                row("S5", "SITE-C", "falling_head", "pressure_plate", 0.07, 0.45, 4.0e-6),
                row("S6", "SITE-D", "steady_state", "hanging_column", 0.02, 0.42, 3.0e-6),
            ],
        }
    }

    #[test]
    fn computes_rac_porosity_and_measurement_method_strata() {
        let input = fixture();
        let report = evaluate(&input, "f".repeat(64)).unwrap();
        assert!(report.strata.iter().any(|stratum| stratum.stratum_id == "rac_lt_5pct" && stratum.observations == 3));
        assert!(report.strata.iter().any(|stratum| stratum.stratum_id == "rac_ge_5pct" && stratum.observations == 3));
        assert!(report.strata.iter().any(|stratum| stratum.stratum_id == "porosity_le_40pct" && stratum.observations == 3));
        assert!(report.strata.iter().any(|stratum| stratum.stratum_id == "ks_method::falling_head" && stratum.observations == 3));
        assert!(report.strata.iter().any(|stratum| stratum.stratum_id == "rac_lt_5pct::ks_method::steady_state" && stratum.observations == 2));
    }

    #[test]
    fn threshold_boundaries_are_explicit_and_nonoverlapping() {
        let mut input = fixture();
        input.observations[0].rac_fraction = 0.05;
        input.observations[0].total_porosity_fraction = 0.40;
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        let rac_low = report.strata.iter().find(|item| item.stratum_id == "rac_lt_5pct").unwrap();
        let rac_high = report.strata.iter().find(|item| item.stratum_id == "rac_ge_5pct").unwrap();
        let phi_low = report.strata.iter().find(|item| item.stratum_id == "porosity_le_40pct").unwrap();
        assert_eq!(rac_low.observations, 3);
        assert_eq!(rac_high.observations, 3);
        assert_eq!(phi_low.observations, 4);
    }

    #[test]
    fn rejects_nonpositive_conductivity_and_nonfinite_values() {
        let mut input = fixture();
        input.observations[0].actual_ks_m_per_s = 0.0;
        assert!(validate(&input).unwrap_err().contains("strictly positive"));
        let mut input = fixture();
        input.observations[0].rac_fraction = f64::NAN;
        assert!(validate(&input).unwrap_err().contains("rac_fraction"));
    }

    #[test]
    fn rejects_missing_or_duplicate_group_sample_and_method_identity() {
        let mut input = fixture();
        input.observations[0].group_id = " ".to_owned();
        assert!(validate(&input).unwrap_err().contains("group_id"));
        let mut input = fixture();
        input.observations[1].sample_id = "S1".to_owned();
        assert!(validate(&input).unwrap_err().contains("duplicate sample_id"));
        let mut input = fixture();
        input.observations[0].water_retention_method = "".to_owned();
        assert!(validate(&input).unwrap_err().contains("water_retention_method"));
    }

    #[test]
    fn rejects_mismatched_model_rosters() {
        let mut input = fixture();
        input.observations[0].predictions_m_per_s.remove("rosetta3");
        assert!(validate(&input).unwrap_err().contains("model prediction roster"));
    }

    #[test]
    fn uniform_method_track_requires_one_protocol_family_per_method() {
        let mut input = fixture();
        input.evaluation_track = EvaluationTrack::UniformMethod;
        assert!(validate(&input).unwrap_err().contains("exactly one Ks measurement method"));
    }

    #[test]
    fn multi_method_track_requires_declared_method_heterogeneity() {
        let mut input = fixture();
        for row in &mut input.observations {
            row.ks_measurement_method = "falling_head".to_owned();
            row.water_retention_method = "pressure_plate".to_owned();
        }
        assert!(validate(&input).unwrap_err().contains("at least two distinct"));
    }

    #[test]
    fn output_records_low_group_strata_as_exploratory() {
        let report = evaluate(&fixture(), "f".repeat(64)).unwrap();
        let low = report.strata.iter().find(|item| item.stratum_id == "rac_lt_5pct").unwrap();
        assert_eq!(low.minimum_group_gate, "fewer_than_5_groups_exploratory_only");
    }

    #[test]
    fn computes_paper_style_material_underprediction_on_log_residuals() {
        let mut input = fixture();
        for row in &mut input.observations {
            row.predictions_m_per_s.insert("brooks_corey_integral".to_owned(), row.actual_ks_m_per_s * 0.1);
        }
        let report = evaluate(&input, "a".repeat(64)).unwrap();
        let all = report.strata.iter().find(|item| item.stratum_id == "all").unwrap();
        let model = all.model_metrics.iter().find(|item| item.model_id == "brooks_corey_integral").unwrap();
        assert!(model.material_underprediction_fraction > 0.0);
        assert_eq!(model.material_overprediction_fraction, 0.0);
    }
}
