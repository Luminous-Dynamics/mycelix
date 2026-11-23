use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "carbon-fate-audit-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "This report evaluates manufactured observations and predictions only. It is not empirical soil evidence, a field model qualification, a permanence or sequestration claim, an MRV method, a carbon-credit quantity, or an intervention recommendation.";

#[derive(Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    observations: Vec<Observation>,
    models: Vec<Model>,
    predictions: Vec<Prediction>,
    thresholds: Vec<StreamThreshold>,
    carbon_balance: CarbonBalance,
}

#[derive(Debug, Deserialize)]
struct Observation {
    observation_id: String,
    stream: String,
    time_days: f64,
    value: f64,
    unit: String,
    measurement_method: String,
    evaluation_group: String,
    standard_uncertainty: f64,
}

#[derive(Debug, Deserialize)]
struct Model {
    model_id: String,
    family: String,
    training_groups: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct Prediction {
    model_id: String,
    observation_id: String,
    status: String,
    mean: Option<f64>,
    lower_90: Option<f64>,
    upper_90: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct StreamThreshold {
    stream: String,
    max_mae: f64,
    min_coverage_90: f64,
}

#[derive(Debug, Deserialize)]
struct CarbonBalance {
    unit: String,
    initial_stock: f64,
    final_stock: f64,
    input_sources: Vec<InputSource>,
    respiration_export: f64,
    dissolved_export: f64,
    erosion_export: f64,
    harvest_export: f64,
    max_absolute_residual: f64,
}

#[derive(Debug, Deserialize)]
struct InputSource {
    source_class: String,
    carbon_input: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    observation_count: usize,
    model_count: usize,
    carbon_balance: CarbonBalanceResult,
    evaluations: Vec<ModelEvaluation>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct CarbonBalanceResult {
    unit: String,
    initial_stock: f64,
    total_input: f64,
    input_by_source: BTreeMap<String, f64>,
    final_stock: f64,
    respiration_export: f64,
    dissolved_export: f64,
    erosion_export: f64,
    harvest_export: f64,
    residual: f64,
    max_absolute_residual: f64,
    qualified: bool,
}

#[derive(Debug, Serialize)]
struct ModelEvaluation {
    model_id: String,
    family: String,
    prediction_count: usize,
    scored_count: usize,
    abstained_count: usize,
    stream_metrics: Vec<StreamMetric>,
    joint_adequacy_pass: bool,
    failed_streams: Vec<String>,
    mechanism_claims_qualified: bool,
}

#[derive(Debug, Serialize)]
struct StreamMetric {
    stream: String,
    scored_count: usize,
    abstained_count: usize,
    mae: Option<f64>,
    rmse: Option<f64>,
    bias: Option<f64>,
    empirical_coverage_90: Option<f64>,
    mean_interval_width: Option<f64>,
    max_mae: f64,
    min_coverage_90: f64,
    qualifies: bool,
}

#[derive(Debug, Serialize)]
struct Controls {
    observations_have_native_method_and_unit: bool,
    training_evaluation_groups_disjoint: bool,
    exactly_one_prediction_per_model_target: bool,
    stock_and_flux_scored_separately: bool,
    system_age_and_respired_transit_time_scored_separately: bool,
    carbon_balance_residual_explicit: bool,
    no_single_compensatory_score_emitted: bool,
    no_empirical_or_mrv_claim: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn allowed_stream(stream: &str) -> bool {
    matches!(
        stream,
        "total_soc_stock"
            | "poc_stock"
            | "maoc_stock"
            | "respiration_flux"
            | "system_age"
            | "respired_co2_transit_time"
    )
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "carbon-fate-v0.1" {
        return Err("unsupported schema_version".into());
    }
    if s.fixture_status != FIXTURE_STATUS {
        return Err("v0.1 accepts synthetic fixtures only".into());
    }
    if s.scenario_id.trim().is_empty() || s.observations.is_empty() || s.models.is_empty() {
        return Err("scenario_id, observations and models must be non-empty".into());
    }
    if s.thresholds.is_empty() {
        return Err("at least one stream threshold is required".into());
    }

    let mut observation_ids = BTreeSet::new();
    let mut evaluation_groups = BTreeSet::new();
    let mut streams = BTreeSet::new();
    for observation in &s.observations {
        if observation.observation_id.trim().is_empty()
            || !observation_ids.insert(observation.observation_id.clone())
            || !allowed_stream(&observation.stream)
            || observation.unit.trim().is_empty()
            || observation.measurement_method.trim().is_empty()
            || observation.evaluation_group.trim().is_empty()
        {
            return Err("observation identity, stream, method, unit and evaluation group must be valid and unique".into());
        }
        for (name, value) in [
            ("time_days", observation.time_days),
            ("observation value", observation.value),
            ("standard_uncertainty", observation.standard_uncertainty),
        ] {
            finite(name, value)?;
        }
        if observation.time_days < 0.0 || observation.standard_uncertainty < 0.0 {
            return Err("observation time and uncertainty cannot be negative".into());
        }
        streams.insert(observation.stream.clone());
        evaluation_groups.insert(observation.evaluation_group.clone());
    }

    let mut model_ids = BTreeSet::new();
    for model in &s.models {
        if model.model_id.trim().is_empty()
            || !model_ids.insert(model.model_id.clone())
            || !matches!(model.family.as_str(), "single_pool" | "two_pool" | "three_pool" | "distributed_or_other")
        {
            return Err("model IDs must be unique and model family must be recognized".into());
        }
        if model.training_groups.iter().any(|group| evaluation_groups.contains(group)) {
            return Err(format!("model {} has evaluation-group leakage in training_groups", model.model_id));
        }
    }

    let mut threshold_streams = BTreeSet::new();
    for threshold in &s.thresholds {
        finite("max_mae", threshold.max_mae)?;
        finite("min_coverage_90", threshold.min_coverage_90)?;
        if !allowed_stream(&threshold.stream)
            || !streams.contains(&threshold.stream)
            || !threshold_streams.insert(threshold.stream.clone())
            || threshold.max_mae <= 0.0
            || !(0.0..=1.0).contains(&threshold.min_coverage_90)
        {
            return Err("stream thresholds must be unique, supported and numerically valid".into());
        }
    }
    if threshold_streams != streams {
        return Err("every observed stream must have exactly one predeclared qualification threshold".into());
    }

    let mut prediction_keys = BTreeSet::new();
    for prediction in &s.predictions {
        if !model_ids.contains(&prediction.model_id)
            || !observation_ids.contains(&prediction.observation_id)
            || !prediction_keys.insert((prediction.model_id.clone(), prediction.observation_id.clone()))
        {
            return Err("predictions must reference known model/observation IDs with unique pairs".into());
        }
        match prediction.status.as_str() {
            "scored" => {
                let mean = prediction.mean.ok_or("scored prediction is missing mean")?;
                let lower = prediction.lower_90.ok_or("scored prediction is missing lower_90")?;
                let upper = prediction.upper_90.ok_or("scored prediction is missing upper_90")?;
                finite("prediction mean", mean)?;
                finite("prediction lower_90", lower)?;
                finite("prediction upper_90", upper)?;
                if lower > upper {
                    return Err("prediction lower_90 cannot exceed upper_90".into());
                }
            }
            "abstained_out_of_domain" => {
                if prediction.mean.is_some() || prediction.lower_90.is_some() || prediction.upper_90.is_some() {
                    return Err("abstained predictions cannot contain numerical forecasts".into());
                }
            }
            _ => return Err("prediction status must be scored or abstained_out_of_domain".into()),
        }
    }

    let expected_prediction_count = model_ids.len() * observation_ids.len();
    if prediction_keys.len() != expected_prediction_count {
        return Err("exactly one prediction or explicit abstention is required for every model/observation pair".into());
    }

    let b = &s.carbon_balance;
    if b.unit.trim().is_empty() || b.input_sources.is_empty() {
        return Err("carbon-balance unit and input source ledger must be non-empty".into());
    }
    for (name, value) in [
        ("initial_stock", b.initial_stock),
        ("final_stock", b.final_stock),
        ("respiration_export", b.respiration_export),
        ("dissolved_export", b.dissolved_export),
        ("erosion_export", b.erosion_export),
        ("harvest_export", b.harvest_export),
        ("max_absolute_residual", b.max_absolute_residual),
    ] {
        finite(name, value)?;
        if value < 0.0 {
            return Err(format!("{name} cannot be negative"));
        }
    }
    if b.initial_stock == 0.0 || b.final_stock == 0.0 || b.max_absolute_residual == 0.0 {
        return Err("stocks and residual tolerance must be positive".into());
    }
    let mut input_classes = BTreeSet::new();
    for source in &b.input_sources {
        if source.source_class.trim().is_empty() || !input_classes.insert(source.source_class.clone()) {
            return Err("carbon input source classes must be non-empty and unique".into());
        }
        finite("carbon_input", source.carbon_input)?;
        if source.carbon_input < 0.0 {
            return Err("carbon inputs cannot be negative".into());
        }
    }
    Ok(())
}

fn carbon_balance_result(balance: &CarbonBalance) -> CarbonBalanceResult {
    let mut input_by_source = BTreeMap::new();
    let mut total_input = 0.0;
    for source in &balance.input_sources {
        input_by_source.insert(source.source_class.clone(), source.carbon_input);
        total_input += source.carbon_input;
    }
    let total_exports = balance.respiration_export
        + balance.dissolved_export
        + balance.erosion_export
        + balance.harvest_export;
    let residual = balance.initial_stock + total_input - balance.final_stock - total_exports;
    CarbonBalanceResult {
        unit: balance.unit.clone(),
        initial_stock: balance.initial_stock,
        total_input,
        input_by_source,
        final_stock: balance.final_stock,
        respiration_export: balance.respiration_export,
        dissolved_export: balance.dissolved_export,
        erosion_export: balance.erosion_export,
        harvest_export: balance.harvest_export,
        residual,
        max_absolute_residual: balance.max_absolute_residual,
        qualified: residual.abs() <= balance.max_absolute_residual,
    }
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let balance = carbon_balance_result(&s.carbon_balance);
    let mut evaluations = Vec::with_capacity(s.models.len());

    for model in &s.models {
        let mut stream_metrics = Vec::with_capacity(s.thresholds.len());
        let mut failed_streams = Vec::new();
        let mut scored_count = 0;
        let mut abstained_count = 0;

        for threshold in &s.thresholds {
            let mut errors = Vec::new();
            let mut biases = Vec::new();
            let mut covered = 0usize;
            let mut width_sum = 0.0;
            let mut stream_abstentions = 0usize;

            for observation in s.observations.iter().filter(|o| o.stream == threshold.stream) {
                let prediction = s.predictions.iter().find(|p| {
                    p.model_id == model.model_id && p.observation_id == observation.observation_id
                }).expect("validation requires one prediction per model and observation");
                if prediction.status == "abstained_out_of_domain" {
                    stream_abstentions += 1;
                    continue;
                }
                let mean = prediction.mean.expect("scored prediction validated");
                let lower = prediction.lower_90.expect("scored prediction validated");
                let upper = prediction.upper_90.expect("scored prediction validated");
                let error = mean - observation.value;
                errors.push(error.abs());
                biases.push(error);
                if lower <= observation.value && observation.value <= upper {
                    covered += 1;
                }
                width_sum += upper - lower;
            }

            scored_count += errors.len();
            abstained_count += stream_abstentions;
            let count = errors.len();
            let mae = (count > 0).then(|| errors.iter().sum::<f64>() / count as f64);
            let rmse = (count > 0).then(|| {
                (errors.iter().map(|e| e * e).sum::<f64>() / count as f64).sqrt()
            });
            let bias = (count > 0).then(|| biases.iter().sum::<f64>() / count as f64);
            let coverage = (count > 0).then(|| covered as f64 / count as f64);
            let interval_width = (count > 0).then(|| width_sum / count as f64);
            let qualifies = mae.is_some_and(|value| value <= threshold.max_mae)
                && coverage.is_some_and(|value| value >= threshold.min_coverage_90);

            if !qualifies {
                failed_streams.push(threshold.stream.clone());
            }
            stream_metrics.push(StreamMetric {
                stream: threshold.stream.clone(),
                scored_count: count,
                abstained_count: stream_abstentions,
                mae,
                rmse,
                bias,
                empirical_coverage_90: coverage,
                mean_interval_width: interval_width,
                max_mae: threshold.max_mae,
                min_coverage_90: threshold.min_coverage_90,
                qualifies,
            });
        }

        if abstained_count > 0 {
            failed_streams.push("unscored_prediction_coverage".into());
        }
        let joint_adequacy_pass = failed_streams.is_empty() && abstained_count == 0;
        evaluations.push(ModelEvaluation {
            model_id: model.model_id.clone(),
            family: model.family.clone(),
            prediction_count: s.observations.len(),
            scored_count,
            abstained_count,
            stream_metrics,
            joint_adequacy_pass,
            failed_streams,
            mechanism_claims_qualified: false,
        });
    }

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        observation_count: s.observations.len(),
        model_count: s.models.len(),
        carbon_balance: balance,
        evaluations,
        controls: Controls {
            observations_have_native_method_and_unit: true,
            training_evaluation_groups_disjoint: true,
            exactly_one_prediction_per_model_target: true,
            stock_and_flux_scored_separately: true,
            system_age_and_respired_transit_time_scored_separately: true,
            carbon_balance_residual_explicit: true,
            no_single_compensatory_score_emitted: true,
            no_empirical_or_mrv_claim: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, input_sha256);
    let report_bytes = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)?;
    output.write_all(&report_bytes)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: carbon-fate-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("carbon fate audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-three-timescale.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn three_pool_jointly_explains_stocks_flux_and_two_carbon_clocks() {
        let s = scenario();
        validate(&s).expect("fixture valid");
        let report = build_report(&s, "0".repeat(64));
        let model = report.evaluations.iter().find(|e| e.model_id == "three-pool").unwrap();
        assert!(model.joint_adequacy_pass, "{:?}", model.failed_streams);
        assert!(!model.mechanism_claims_qualified);
    }

    #[test]
    fn two_pool_stock_fit_cannot_hide_flux_and_transit_time_failure() {
        let report = build_report(&scenario(), "0".repeat(64));
        let model = report.evaluations.iter().find(|e| e.model_id == "two-pool").unwrap();
        let stock = model.stream_metrics.iter().find(|m| m.stream == "total_soc_stock").unwrap();
        let respiration = model.stream_metrics.iter().find(|m| m.stream == "respiration_flux").unwrap();
        let transit = model.stream_metrics.iter().find(|m| m.stream == "respired_co2_transit_time").unwrap();
        assert!(stock.qualifies);
        assert!(!respiration.qualifies);
        assert!(!transit.qualifies);
        assert!(!model.joint_adequacy_pass);
        assert!(!model.mechanism_claims_qualified);
    }

    #[test]
    fn system_age_and_respired_transit_time_are_distinct_streams() {
        let report = build_report(&scenario(), "0".repeat(64));
        let model = report.evaluations.iter().find(|e| e.model_id == "three-pool").unwrap();
        assert!(model.stream_metrics.iter().any(|m| m.stream == "system_age"));
        assert!(model.stream_metrics.iter().any(|m| m.stream == "respired_co2_transit_time"));
    }

    #[test]
    fn carbon_balance_keeps_pyrogenic_and_plant_inputs_separate() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!(report.carbon_balance.qualified);
        assert_eq!(report.carbon_balance.input_by_source.get("plant"), Some(&3.0));
        assert_eq!(report.carbon_balance.input_by_source.get("pyrogenic"), Some(&1.0));
    }

    #[test]
    fn carbon_balance_residual_blocks_mechanism_claims() {
        let mut s = scenario();
        s.carbon_balance.final_stock += 1.0;
        let report = build_report(&s, "0".repeat(64));
        assert!(!report.carbon_balance.qualified);
        assert!(report.evaluations.iter().all(|m| !m.mechanism_claims_qualified));
    }

    #[test]
    fn evaluation_group_in_training_is_rejected() {
        let mut s = scenario();
        s.models[0].training_groups.push("synthetic-heldout-site".into());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn missing_model_target_prediction_is_rejected() {
        let mut s = scenario();
        s.predictions.pop();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn an_abstention_cannot_smuggle_a_numerical_forecast() {
        let mut s = scenario();
        s.predictions[0].status = "abstained_out_of_domain".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn interval_with_inverted_bounds_is_rejected() {
        let mut s = scenario();
        s.predictions[0].lower_90 = Some(9.0);
        s.predictions[0].upper_90 = Some(8.0);
        assert!(validate(&s).is_err());
    }
}
