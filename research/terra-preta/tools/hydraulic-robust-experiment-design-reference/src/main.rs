use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    cmp::Ordering,
    collections::BTreeSet,
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "hydraulic-robust-experiment-design-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const N: usize = 4;
const MAX_SCENARIOS: usize = 8;
const MAX_CANDIDATES: usize = 16;
const EIGEN_TOLERANCE: f64 = 1.0e-12;

type Vector = [f64; N];
type Matrix = [[f64; N]; N];

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_set_id: String,
    common_estimand_id: String,
    parameter_ids: [String; N],
    parameter_scales: Vector,
    acquisition_budget: u32,
    determinant_regularizer: f64,
    scenarios: Vec<Scenario>,
    candidates: Vec<Candidate>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Scenario {
    scenario_id: String,
    segmentation_variant_id: String,
    registration_variant_id: String,
    estimand_id: String,
    nominal: bool,
    prior_information: Matrix,
    baseline_measurements: Vec<Measurement>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Candidate {
    candidate_id: String,
    sensor_kind: String,
    cost: u32,
    measurements: Vec<ScenarioMeasurement>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ScenarioMeasurement {
    scenario_id: String,
    sensitivity: Vector,
    noise_std: f64,
    support_coverage_fraction: f64,
    support_description: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Measurement {
    measurement_id: String,
    sensitivity: Vector,
    noise_std: f64,
    support_description: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_set_id: String,
    common_estimand_id: String,
    input_sha256: String,
    parameter_ids: [String; N],
    parameter_scales: Vector,
    scenario_count: usize,
    scenario_probability_policy: String,
    acquisition_budget: u32,
    nominal_design: DesignSummary,
    robust_d_optimal_design: DesignSummary,
    robust_e_optimal_design: DesignSummary,
    minimax_regret_design: DesignSummary,
    per_scenario_oracles: Vec<ScenarioOracle>,
    candidate_support_receipts: Vec<CandidateSupportReceipt>,
    repeated_measurement_negative_controls: Vec<RepeatedMeasurementControl>,
    interpretation_boundary: String,
}

#[derive(Clone, Debug, Serialize)]
struct ScenarioScore {
    scenario_id: String,
    segmentation_variant_id: String,
    registration_variant_id: String,
    data_sensitivity_rank: usize,
    local_nullity: usize,
    minimum_eigenvalue_data_information: f64,
    minimum_eigenvalue_total_information: f64,
    log_determinant_total_information: f64,
    total_data_fisher: Matrix,
}

#[derive(Clone, Debug, Serialize)]
struct DesignSummary {
    candidate_ids: Vec<String>,
    total_cost: u32,
    acquisition_budget: u32,
    objective_name: String,
    objective_value: f64,
    worst_case_log_determinant: f64,
    worst_case_minimum_eigenvalue_data_information: f64,
    worst_case_minimum_eigenvalue_total_information: f64,
    worst_case_rank: usize,
    maximum_normalized_d_optimal_regret: f64,
    maximum_absolute_d_optimal_regret: f64,
    per_scenario_scores: Vec<ScenarioScore>,
}

#[derive(Clone, Debug, Serialize)]
struct ScenarioOracle {
    scenario_id: String,
    oracle_candidate_ids: Vec<String>,
    oracle_cost: u32,
    oracle_log_determinant: f64,
    oracle_minimum_eigenvalue: f64,
    oracle_rank: usize,
}

#[derive(Clone, Debug, Serialize)]
struct CandidateSupportReceipt {
    candidate_id: String,
    sensor_kind: String,
    cost: u32,
    per_scenario_support: Vec<ScenarioSupportReceipt>,
}

#[derive(Clone, Debug, Serialize)]
struct ScenarioSupportReceipt {
    scenario_id: String,
    support_coverage_fraction: f64,
    noise_std: f64,
    support_description: String,
    sensitivity: Vector,
}

#[derive(Clone, Debug, Serialize)]
struct RepeatedMeasurementControl {
    scenario_id: String,
    baseline_rank: usize,
    rank_after_repeating_first_baseline_measurement: usize,
    rank_increase: usize,
    explanation: String,
}

#[derive(Clone, Debug)]
struct EvaluatedDesign {
    candidate_ids: Vec<String>,
    total_cost: u32,
    scenario_scores: Vec<ScenarioScore>,
    worst_case_log_determinant: f64,
    worst_case_minimum_eigenvalue_data: f64,
    worst_case_minimum_eigenvalue_total: f64,
    worst_case_rank: usize,
    max_normalized_regret: f64,
    max_absolute_regret: f64,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_set_id.trim().is_empty()
        || fixture.common_estimand_id.trim().is_empty()
    {
        return Err("fixture identity, common-estimand identity or synthetic-only status is invalid".into());
    }
    if fixture.parameter_ids.iter().any(|id| id.trim().is_empty())
        || fixture.parameter_ids.iter().collect::<BTreeSet<_>>().len() != N
    {
        return Err("common parameter IDs must be nonempty and unique".into());
    }
    if fixture.parameter_scales.iter().any(|value| !value.is_finite() || *value <= 0.0) {
        return Err("dimensionless parameter scales must be finite and positive".into());
    }
    if fixture.acquisition_budget == 0
        || !fixture.determinant_regularizer.is_finite()
        || fixture.determinant_regularizer <= 0.0
    {
        return Err("budget and determinant regularizer must be positive".into());
    }
    if fixture.scenarios.len() < 2 || fixture.scenarios.len() > MAX_SCENARIOS {
        return Err(format!("between two and {MAX_SCENARIOS} scenarios are required"));
    }
    if fixture.candidates.is_empty() || fixture.candidates.len() > MAX_CANDIDATES {
        return Err(format!("between one and {MAX_CANDIDATES} candidates are required"));
    }

    let scenario_ids: BTreeSet<&str> = fixture.scenarios.iter().map(|scenario| scenario.scenario_id.as_str()).collect();
    if scenario_ids.len() != fixture.scenarios.len() {
        return Err("scenario IDs must be unique".into());
    }
    if fixture.scenarios.iter().filter(|scenario| scenario.nominal).count() != 1 {
        return Err("exactly one scenario must be designated nominal".into());
    }

    for scenario in &fixture.scenarios {
        if scenario.scenario_id.trim().is_empty()
            || scenario.segmentation_variant_id.trim().is_empty()
            || scenario.registration_variant_id.trim().is_empty()
            || scenario.estimand_id != fixture.common_estimand_id
        {
            return Err("every scenario must use nonempty identifiers and the declared common estimand".into());
        }
        validate_symmetric_information(&scenario.prior_information, "prior_information")?;
        if scenario.baseline_measurements.is_empty() {
            return Err(format!("scenario {} needs a baseline observation", scenario.scenario_id));
        }
        let mut ids = BTreeSet::new();
        for measurement in &scenario.baseline_measurements {
            validate_measurement(measurement, &fixture.parameter_scales)?;
            if !ids.insert(measurement.measurement_id.as_str()) {
                return Err(format!("duplicate baseline measurement ID in {}", scenario.scenario_id));
            }
        }
    }

    let mut candidate_ids = BTreeSet::new();
    for candidate in &fixture.candidates {
        if candidate.candidate_id.trim().is_empty()
            || candidate.sensor_kind.trim().is_empty()
            || candidate.cost == 0
            || !candidate_ids.insert(candidate.candidate_id.as_str())
        {
            return Err("candidate IDs/sensor kinds must be nonempty and costs positive".into());
        }
        if candidate.measurements.len() != fixture.scenarios.len() {
            return Err(format!("candidate {} needs exactly one operator per scenario", candidate.candidate_id));
        }
        let mut seen_scenarios = BTreeSet::new();
        for measurement in &candidate.measurements {
            if !scenario_ids.contains(measurement.scenario_id.as_str())
                || !seen_scenarios.insert(measurement.scenario_id.as_str())
            {
                return Err(format!("candidate {} has a duplicate or unknown scenario operator", candidate.candidate_id));
            }
            if measurement.sensitivity.iter().any(|value| !value.is_finite())
                || !measurement.noise_std.is_finite()
                || measurement.noise_std <= 0.0
                || !measurement.support_coverage_fraction.is_finite()
                || !(0.0..=1.0).contains(&measurement.support_coverage_fraction)
                || measurement.support_coverage_fraction == 0.0
                || measurement.support_description.trim().is_empty()
            {
                return Err(format!("candidate {} has invalid sensitivity, noise or support", candidate.candidate_id));
            }
        }
    }
    Ok(())
}

fn validate_measurement(measurement: &Measurement, scales: &Vector) -> Result<(), String> {
    if measurement.measurement_id.trim().is_empty()
        || measurement.support_description.trim().is_empty()
        || !measurement.noise_std.is_finite()
        || measurement.noise_std <= 0.0
        || measurement.sensitivity.iter().any(|value| !value.is_finite())
    {
        return Err("baseline measurement ID, support, sensitivity or noise is invalid".into());
    }
    let row = scale_row(&measurement.sensitivity, scales);
    if row.iter().any(|value| !value.is_finite()) {
        return Err("scaled baseline sensitivity is non-finite".into());
    }
    Ok(())
}

fn validate_symmetric_information(matrix: &Matrix, name: &str) -> Result<(), String> {
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return Err(format!("{name} contains a non-finite value"));
    }
    for row in 0..N {
        for column in 0..N {
            if (matrix[row][column] - matrix[column][row]).abs() > 1.0e-10 {
                return Err(format!("{name} is not symmetric"));
            }
        }
    }
    if minimum_symmetric_eigenvalue(*matrix) < -1.0e-10 {
        return Err(format!("{name} is not positive semidefinite"));
    }
    Ok(())
}

fn scale_row(row: &Vector, scales: &Vector) -> Vector {
    std::array::from_fn(|index| row[index] * scales[index])
}

fn add_outer_product(target: &mut Matrix, row: &Vector, precision: f64) {
    for left in 0..N {
        for right in 0..N {
            target[left][right] += row[left] * row[right] * precision;
        }
    }
}

fn rank_rows(rows: &[Vector]) -> usize {
    if rows.is_empty() {
        return 0;
    }
    let mut matrix: Vec<Vec<f64>> = rows.iter().map(|row| row.to_vec()).collect();
    let scale = matrix.iter().flatten().fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if scale == 0.0 {
        return 0;
    }
    let tolerance = scale * 1.0e-10;
    let mut pivot_row = 0usize;

    for column in 0..N {
        if pivot_row >= matrix.len() {
            break;
        }
        let Some(best_row) = (pivot_row..matrix.len())
            .max_by(|left, right| matrix[*left][column].abs().total_cmp(&matrix[*right][column].abs()))
        else {
            break;
        };
        if matrix[best_row][column].abs() <= tolerance {
            continue;
        }
        matrix.swap(pivot_row, best_row);
        let pivot = matrix[pivot_row][column];
        for row in (pivot_row + 1)..matrix.len() {
            let factor = matrix[row][column] / pivot;
            for index in column..N {
                matrix[row][index] -= factor * matrix[pivot_row][index];
            }
        }
        pivot_row += 1;
    }
    pivot_row
}

fn log_determinant_spd(mut matrix: Matrix, regularizer: f64) -> Result<f64, String> {
    for index in 0..N {
        matrix[index][index] += regularizer;
    }
    let mut lower = [[0.0; N]; N];
    for row in 0..N {
        for column in 0..=row {
            let correction: f64 = (0..column).map(|index| lower[row][index] * lower[column][index]).sum();
            if row == column {
                let diagonal = matrix[row][row] - correction;
                if !diagonal.is_finite() || diagonal <= 0.0 {
                    return Err("regularized information matrix is not positive definite".into());
                }
                lower[row][column] = diagonal.sqrt();
            } else {
                lower[row][column] = (matrix[row][column] - correction) / lower[column][column];
            }
        }
    }
    Ok(2.0 * (0..N).map(|index| lower[index][index].ln()).sum::<f64>())
}

fn minimum_symmetric_eigenvalue(mut matrix: Matrix) -> f64 {
    for _ in 0..64 {
        let mut p = 0usize;
        let mut q = 1usize;
        let mut largest = 0.0_f64;
        for row in 0..N {
            for column in (row + 1)..N {
                if matrix[row][column].abs() > largest {
                    largest = matrix[row][column].abs();
                    p = row;
                    q = column;
                }
            }
        }
        if largest <= EIGEN_TOLERANCE {
            break;
        }

        let app = matrix[p][p];
        let aqq = matrix[q][q];
        let apq = matrix[p][q];
        let tau = (aqq - app) / (2.0 * apq);
        let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
        let t = sign / (tau.abs() + (1.0 + tau * tau).sqrt());
        let cosine = 1.0 / (1.0 + t * t).sqrt();
        let sine = t * cosine;

        matrix[p][p] = app - t * apq;
        matrix[q][q] = aqq + t * apq;
        matrix[p][q] = 0.0;
        matrix[q][p] = 0.0;
        for index in 0..N {
            if index == p || index == q {
                continue;
            }
            let aip = matrix[index][p];
            let aiq = matrix[index][q];
            matrix[index][p] = cosine * aip - sine * aiq;
            matrix[p][index] = matrix[index][p];
            matrix[index][q] = sine * aip + cosine * aiq;
            matrix[q][index] = matrix[index][q];
        }
    }
    (0..N).map(|index| matrix[index][index]).fold(f64::INFINITY, f64::min)
}

fn find_scenario_measurement<'a>(candidate: &'a Candidate, scenario_id: &str) -> Result<&'a ScenarioMeasurement, String> {
    candidate.measurements.iter().find(|measurement| measurement.scenario_id == scenario_id)
        .ok_or_else(|| format!("candidate {} has no operator for scenario {scenario_id}", candidate.candidate_id))
}

fn evaluate_scenario(fixture: &Fixture, scenario: &Scenario, selected: &[usize]) -> Result<ScenarioScore, String> {
    let mut information = scenario.prior_information;
    let mut data_fisher = [[0.0; N]; N];
    let mut rows = Vec::new();

    for measurement in &scenario.baseline_measurements {
        let row = scale_row(&measurement.sensitivity, &fixture.parameter_scales);
        rows.push(row);
        add_outer_product(&mut data_fisher, &row, 1.0 / measurement.noise_std.powi(2));
    }
    for index in selected {
        let measurement = find_scenario_measurement(&fixture.candidates[*index], &scenario.scenario_id)?;
        let row = scale_row(&measurement.sensitivity, &fixture.parameter_scales);
        rows.push(row);
        add_outer_product(&mut data_fisher, &row, 1.0 / measurement.noise_std.powi(2));
    }
    for row in 0..N {
        for column in 0..N {
            information[row][column] += data_fisher[row][column];
        }
    }

    Ok(ScenarioScore {
        scenario_id: scenario.scenario_id.clone(),
        segmentation_variant_id: scenario.segmentation_variant_id.clone(),
        registration_variant_id: scenario.registration_variant_id.clone(),
        data_sensitivity_rank: rank_rows(&rows),
        local_nullity: N.saturating_sub(rank_rows(&rows)),
        minimum_eigenvalue_data_information: minimum_symmetric_eigenvalue(data_fisher),
        minimum_eigenvalue_total_information: minimum_symmetric_eigenvalue(information),
        log_determinant_total_information: log_determinant_spd(information, fixture.determinant_regularizer)?,
        total_data_fisher: data_fisher,
    })
}

fn score_design(fixture: &Fixture, selected: Vec<usize>, cost: u32) -> Result<EvaluatedDesign, String> {
    let mut scenario_scores = Vec::with_capacity(fixture.scenarios.len());
    for scenario in &fixture.scenarios {
        scenario_scores.push(evaluate_scenario(fixture, scenario, &selected)?);
    }
    let worst_case_log_determinant = scenario_scores.iter()
        .map(|score| score.log_determinant_total_information)
        .fold(f64::INFINITY, f64::min);
    let worst_case_minimum_eigenvalue_data = scenario_scores.iter()
        .map(|score| score.minimum_eigenvalue_data_information)
        .fold(f64::INFINITY, f64::min);
    let worst_case_minimum_eigenvalue_total = scenario_scores.iter()
        .map(|score| score.minimum_eigenvalue_total_information)
        .fold(f64::INFINITY, f64::min);
    let worst_case_rank = scenario_scores.iter().map(|score| score.data_sensitivity_rank).min().unwrap_or(0);
    Ok(EvaluatedDesign {
        candidate_ids: selected.iter().map(|index| fixture.candidates[*index].candidate_id.clone()).collect(),
        total_cost: cost,
        scenario_scores,
        worst_case_log_determinant,
        worst_case_minimum_eigenvalue_data,
        worst_case_minimum_eigenvalue_total,
        worst_case_rank,
        max_normalized_regret: 0.0,
        max_absolute_regret: 0.0,
    })
}

fn enumerate_designs(fixture: &Fixture) -> Result<Vec<EvaluatedDesign>, String> {
    let mut designs = Vec::new();
    let combinations = 1u32 << fixture.candidates.len();
    for mask in 0..combinations {
        let mut selected = Vec::new();
        let mut cost = 0u32;
        for (index, candidate) in fixture.candidates.iter().enumerate() {
            if mask & (1u32 << index) != 0 {
                cost = cost.saturating_add(candidate.cost);
                selected.push(index);
            }
        }
        if cost <= fixture.acquisition_budget {
            designs.push(score_design(fixture, selected, cost)?);
        }
    }
    if designs.is_empty() {
        return Err("no feasible design under the acquisition budget".into());
    }
    Ok(designs)
}

fn scenario_oracles(fixture: &Fixture, designs: &[EvaluatedDesign]) -> Vec<ScenarioOracle> {
    fixture.scenarios.iter().enumerate().map(|(scenario_index, scenario)| {
        let best = designs.iter().max_by(|left, right| {
            left.scenario_scores[scenario_index].log_determinant_total_information
                .total_cmp(&right.scenario_scores[scenario_index].log_determinant_total_information)
                .then_with(|| right.total_cost.cmp(&left.total_cost))
                .then_with(|| right.candidate_ids.cmp(&left.candidate_ids))
        }).expect("feasible design set is nonempty");
        let score = &best.scenario_scores[scenario_index];
        ScenarioOracle {
            scenario_id: scenario.scenario_id.clone(),
            oracle_candidate_ids: best.candidate_ids.clone(),
            oracle_cost: best.total_cost,
            oracle_log_determinant: score.log_determinant_total_information,
            oracle_minimum_eigenvalue: score.minimum_eigenvalue_total_information,
            oracle_rank: score.data_sensitivity_rank,
        }
    }).collect()
}

fn assign_regrets(designs: &mut [EvaluatedDesign], oracles: &[ScenarioOracle]) {
    for design in designs {
        let mut max_absolute = 0.0_f64;
        let mut max_normalized = 0.0_f64;
        for (index, score) in design.scenario_scores.iter().enumerate() {
            let regret = (oracles[index].oracle_log_determinant - score.log_determinant_total_information).max(0.0);
            let normalized = regret / oracles[index].oracle_log_determinant.abs().max(1.0);
            max_absolute = max_absolute.max(regret);
            max_normalized = max_normalized.max(normalized);
        }
        design.max_absolute_regret = max_absolute;
        design.max_normalized_regret = max_normalized;
    }
}

fn scalar_order(left: f64, right: f64, maximize: bool) -> Ordering {
    let order = left.total_cmp(&right);
    if maximize { order } else { order.reverse() }
}

fn deterministic_tie_break(left: &EvaluatedDesign, right: &EvaluatedDesign) -> Ordering {
    right.total_cost.cmp(&left.total_cost).then_with(|| right.candidate_ids.cmp(&left.candidate_ids))
}

fn select_design(designs: &[EvaluatedDesign], objective: &str, nominal_index: usize) -> Result<EvaluatedDesign, String> {
    designs.iter().cloned().max_by(|left, right| {
        let (left_score, right_score, maximize) = match objective {
            "nominal_d_optimal" => (
                left.scenario_scores[nominal_index].log_determinant_total_information,
                right.scenario_scores[nominal_index].log_determinant_total_information,
                true,
            ),
            "robust_d_optimal" => (left.worst_case_log_determinant, right.worst_case_log_determinant, true),
            "robust_e_optimal" => (left.worst_case_minimum_eigenvalue_data, right.worst_case_minimum_eigenvalue_data, true),
            "minimax_regret" => (left.max_normalized_regret, right.max_normalized_regret, false),
            _ => (f64::NAN, f64::NAN, true),
        };
        scalar_order(left_score, right_score, maximize).then_with(|| deterministic_tie_break(left, right))
    }).ok_or_else(|| format!("no feasible selection for {objective}"))
}

fn summarize(design: &EvaluatedDesign, budget: u32, name: &str, objective_value: f64) -> DesignSummary {
    DesignSummary {
        candidate_ids: design.candidate_ids.clone(),
        total_cost: design.total_cost,
        acquisition_budget: budget,
        objective_name: name.into(),
        objective_value,
        worst_case_log_determinant: design.worst_case_log_determinant,
        worst_case_minimum_eigenvalue_data_information: design.worst_case_minimum_eigenvalue_data,
        worst_case_minimum_eigenvalue_total_information: design.worst_case_minimum_eigenvalue_total,
        worst_case_rank: design.worst_case_rank,
        maximum_normalized_d_optimal_regret: design.max_normalized_regret,
        maximum_absolute_d_optimal_regret: design.max_absolute_regret,
        per_scenario_scores: design.scenario_scores.clone(),
    }
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let nominal_index = fixture.scenarios.iter().position(|scenario| scenario.nominal)
        .ok_or_else(|| "nominal scenario missing".to_string())?;
    let mut designs = enumerate_designs(fixture)?;
    let oracles = scenario_oracles(fixture, &designs);
    assign_regrets(&mut designs, &oracles);

    let nominal = select_design(&designs, "nominal_d_optimal", nominal_index)?;
    let robust_d = select_design(&designs, "robust_d_optimal", nominal_index)?;
    let robust_e = select_design(&designs, "robust_e_optimal", nominal_index)?;
    let minimax = select_design(&designs, "minimax_regret", nominal_index)?;
    let nominal_score = nominal.scenario_scores[nominal_index].log_determinant_total_information;

    let candidate_support_receipts = fixture.candidates.iter().map(|candidate| CandidateSupportReceipt {
        candidate_id: candidate.candidate_id.clone(),
        sensor_kind: candidate.sensor_kind.clone(),
        cost: candidate.cost,
        per_scenario_support: candidate.measurements.iter().map(|measurement| ScenarioSupportReceipt {
            scenario_id: measurement.scenario_id.clone(),
            support_coverage_fraction: measurement.support_coverage_fraction,
            noise_std: measurement.noise_std,
            support_description: measurement.support_description.clone(),
            sensitivity: measurement.sensitivity,
        }).collect(),
    }).collect();

    let repeated_measurement_negative_controls = fixture.scenarios.iter().map(|scenario| {
        let rows: Vec<Vector> = scenario.baseline_measurements.iter()
            .map(|measurement| scale_row(&measurement.sensitivity, &fixture.parameter_scales))
            .collect();
        let baseline_rank = rank_rows(&rows);
        let mut repeated_rows = rows.clone();
        repeated_rows.push(rows[0]);
        let repeated_rank = rank_rows(&repeated_rows);
        RepeatedMeasurementControl {
            scenario_id: scenario.scenario_id.clone(),
            baseline_rank,
            rank_after_repeating_first_baseline_measurement: repeated_rank,
            rank_increase: repeated_rank.saturating_sub(baseline_rank),
            explanation: "Repeating the same support can reduce uncertainty along an existing sensitivity direction but cannot add a structural direction to the local sensitivity row space.".into(),
        }
    }).collect();

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_set_id: fixture.scenario_set_id.clone(),
        common_estimand_id: fixture.common_estimand_id.clone(),
        input_sha256,
        parameter_ids: fixture.parameter_ids.clone(),
        parameter_scales: fixture.parameter_scales,
        scenario_count: fixture.scenarios.len(),
        scenario_probability_policy: "unweighted worst-case over declared segmentation/registration scenarios; scenario count is not interpreted as a probability".into(),
        acquisition_budget: fixture.acquisition_budget,
        nominal_design: summarize(&nominal, fixture.acquisition_budget, "nominal_d_optimal", nominal_score),
        robust_d_optimal_design: summarize(&robust_d, fixture.acquisition_budget, "maximin_logdet", robust_d.worst_case_log_determinant),
        robust_e_optimal_design: summarize(&robust_e, fixture.acquisition_budget, "maximin_data_minimum_eigenvalue", robust_e.worst_case_minimum_eigenvalue_data),
        minimax_regret_design: summarize(&minimax, fixture.acquisition_budget, "minimax_normalized_regret", minimax.max_normalized_regret),
        per_scenario_oracles: oracles,
        candidate_support_receipts,
        repeated_measurement_negative_controls,
        interpretation_boundary: "Synthetic sensitivity-row design reference. Candidate Jacobians encode declared scenario-specific support, registration, mixing and noise. The engine rejects common-estimand mismatch but does not derive Jacobians from images or calibrate physical sensors. Numerical rank is local and data-only; the regularized logdet includes prior information. No scenario probability is inferred from variant counts. No real soil or field experiment has been validated.".into(),
    })
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!("usage: {} INPUT.json OUTPUT.json", arguments.first().map(String::as_str).unwrap_or("hydraulic-robust-experiment-design-reference")));
    }
    let bytes = fs::read(&arguments[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let hash = format!("{:x}", Sha256::digest(&bytes));
    let fixture: Fixture = serde_json::from_slice(&bytes).map_err(|error| format!("invalid fixture: {error}"))?;
    let report = analyze(&fixture, hash)?;
    let output = serde_json::to_vec_pretty(&report).map_err(|error| format!("cannot serialize report: {error}"))?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(&arguments[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    file.write_all(&output).map_err(|error| format!("cannot write output: {error}"))?;
    file.write_all(b"\n").map_err(|error| format!("cannot finalize output: {error}"))?;
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Fixture {
        serde_json::from_str(include_str!("../fixtures/segmentation-support-ensemble.json")).unwrap()
    }

    #[test]
    fn robust_design_is_feasible_and_reports_every_scenario() {
        let report = analyze(&fixture(), "0".repeat(64)).unwrap();
        assert_eq!(report.scenario_count, 3);
        assert!(report.scenario_probability_policy.contains("unweighted"));
        for design in [
            &report.nominal_design,
            &report.robust_d_optimal_design,
            &report.robust_e_optimal_design,
            &report.minimax_regret_design,
        ] {
            assert!(design.total_cost <= design.acquisition_budget);
            assert_eq!(design.per_scenario_scores.len(), 3);
        }
        assert!(report.robust_d_optimal_design.worst_case_rank >= 3);
    }

    #[test]
    fn repeat_bulk_measurement_never_increases_structural_rank() {
        let report = analyze(&fixture(), "1".repeat(64)).unwrap();
        for control in &report.repeated_measurement_negative_controls {
            assert_eq!(control.rank_increase, 0);
            assert_eq!(control.baseline_rank, control.rank_after_repeating_first_baseline_measurement);
        }
    }

    #[test]
    fn scenario_order_permutation_does_not_change_robust_selection() {
        let base = fixture();
        let baseline = analyze(&base, "2".repeat(64)).unwrap();
        let mut reordered = base;
        reordered.scenarios.reverse();
        for candidate in &mut reordered.candidates {
            candidate.measurements.reverse();
        }
        let permuted = analyze(&reordered, "2".repeat(64)).unwrap();
        assert_eq!(baseline.robust_d_optimal_design.candidate_ids, permuted.robust_d_optimal_design.candidate_ids);
        assert!((baseline.robust_d_optimal_design.worst_case_log_determinant - permuted.robust_d_optimal_design.worst_case_log_determinant).abs() < 1.0e-10);
        assert_eq!(baseline.minimax_regret_design.candidate_ids, permuted.minimax_regret_design.candidate_ids);
    }

    #[test]
    fn scenario_estimand_mismatch_is_rejected() {
        let mut invalid = fixture();
        invalid.scenarios[1].estimand_id = "variant-local-pore-id".into();
        assert!(validate(&invalid).is_err());
    }

    #[test]
    fn candidate_missing_scenario_operator_is_rejected() {
        let mut invalid = fixture();
        invalid.candidates[0].measurements.pop();
        assert!(validate(&invalid).is_err());
    }

    #[test]
    fn support_registration_variants_change_operator_and_noise() {
        let f = fixture();
        let nominal = &f.candidates[2].measurements[0];
        let perturbed = &f.candidates[2].measurements[2];
        assert_ne!(nominal.sensitivity, perturbed.sensitivity);
        assert!(perturbed.noise_std > nominal.noise_std);
        assert!(perturbed.support_coverage_fraction < nominal.support_coverage_fraction);
    }

    #[test]
    fn prior_information_does_not_inflate_data_sensitivity_rank() {
        let report = analyze(&fixture(), "3".repeat(64)).unwrap();
        assert!(report.robust_d_optimal_design.per_scenario_scores.iter()
            .all(|score| score.data_sensitivity_rank <= N));
        assert!(report.robust_d_optimal_design.per_scenario_scores.iter()
            .all(|score| score.minimum_eigenvalue_total_information > score.minimum_eigenvalue_data_information));
    }
}
