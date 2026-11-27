use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeSet,
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "hydraulic-experiment-design-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const LOG_PARAMETER_STEP: f64 = 1.0e-5;
const RANK_RELATIVE_TOLERANCE: f64 = 1.0e-8;
const DETERMINANT_REGULARIZER: f64 = 1.0e-8;

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum ObservationKind {
    BulkConductance,
    UpperPressure,
    LowerPressure,
    UpperPathFlow,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Candidate {
    candidate_id: String,
    observation: ObservationKind,
    noise_std: f64,
    cost: u32,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    conductances: [f64; 4],
    baseline_bulk_noise_std: f64,
    acquisition_budget: u32,
    candidates: Vec<Candidate>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    parameterization: String,
    nominal_observations: NominalObservations,
    baseline: DesignSummary,
    cost_matched_repeat_bulk_design: DesignSummary,
    candidate_sensitivities: Vec<CandidateSensitivity>,
    optimal_design: DesignSummary,
    repeated_bulk_negative_control: RepeatedMeasurementControl,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct NominalObservations {
    bulk_effective_conductance: f64,
    upper_intermediate_pressure: f64,
    lower_intermediate_pressure: f64,
    upper_path_flow: f64,
}

#[derive(Debug, Serialize)]
struct CandidateSensitivity {
    candidate_id: String,
    observation: ObservationKind,
    cost: u32,
    noise_std: f64,
    derivative_with_respect_to_log_conductances: [f64; 4],
    rank_when_added_to_baseline: usize,
}

#[derive(Debug, Serialize)]
struct DesignSummary {
    candidate_ids: Vec<String>,
    total_cost: u32,
    acquisition_budget: u32,
    sensitivity_rank: usize,
    parameter_count: usize,
    local_nullity: usize,
    regularized_log_determinant_fisher: f64,
    fisher_information: [[f64; 4]; 4],
}

#[derive(Debug, Serialize)]
struct RepeatedMeasurementControl {
    baseline_rank: usize,
    rank_after_repeating_bulk_measurement: usize,
    rank_increase: usize,
    explanation: String,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture identity, schema version or synthetic-only status is invalid".into());
    }
    if fixture
        .conductances
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err("all four throat conductances must be finite and positive".into());
    }
    if !fixture.baseline_bulk_noise_std.is_finite() || fixture.baseline_bulk_noise_std <= 0.0 {
        return Err("baseline noise standard deviation must be finite and positive".into());
    }
    if fixture.candidates.is_empty() || fixture.candidates.len() > 16 {
        return Err("between one and sixteen candidate experiments are required".into());
    }

    let mut ids = BTreeSet::new();
    for candidate in &fixture.candidates {
        if candidate.candidate_id.trim().is_empty()
            || !ids.insert(candidate.candidate_id.as_str())
            || !candidate.noise_std.is_finite()
            || candidate.noise_std <= 0.0
            || candidate.cost == 0
        {
            return Err(
                "candidate IDs must be unique and each candidate needs positive cost and noise"
                    .into(),
            );
        }
    }
    Ok(())
}

fn path_flow(g_in: f64, g_out: f64) -> f64 {
    g_in * g_out / (g_in + g_out)
}

fn predict(kind: ObservationKind, g: &[f64; 4]) -> f64 {
    let upper_flow = path_flow(g[0], g[1]);
    let lower_flow = path_flow(g[2], g[3]);
    match kind {
        ObservationKind::BulkConductance => upper_flow + lower_flow,
        ObservationKind::UpperPressure => g[0] / (g[0] + g[1]),
        ObservationKind::LowerPressure => g[2] / (g[2] + g[3]),
        ObservationKind::UpperPathFlow => upper_flow,
    }
}

fn log_conductance_sensitivity(kind: ObservationKind, conductances: &[f64; 4]) -> [f64; 4] {
    let mut derivative = [0.0; 4];
    for (parameter, value) in derivative.iter_mut().enumerate() {
        let mut plus = *conductances;
        let mut minus = *conductances;
        plus[parameter] *= LOG_PARAMETER_STEP.exp();
        minus[parameter] *= (-LOG_PARAMETER_STEP).exp();
        *value = (predict(kind, &plus) - predict(kind, &minus)) / (2.0 * LOG_PARAMETER_STEP);
    }
    derivative
}

#[allow(clippy::needless_range_loop)]
fn rank_rows(rows: &[[f64; 4]]) -> usize {
    if rows.is_empty() {
        return 0;
    }
    let mut matrix: Vec<Vec<f64>> = rows.iter().map(|row| row.to_vec()).collect();
    let row_count = matrix.len();
    let column_count = 4usize;
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if scale == 0.0 {
        return 0;
    }
    let tolerance = scale * RANK_RELATIVE_TOLERANCE;
    let mut pivot_row = 0usize;

    for column in 0..column_count {
        if pivot_row >= row_count {
            break;
        }
        let candidate = (pivot_row..row_count).max_by(|&left, &right| {
            matrix[left][column]
                .abs()
                .total_cmp(&matrix[right][column].abs())
        });
        let Some(best_row) = candidate else {
            break;
        };
        if matrix[best_row][column].abs() <= tolerance {
            continue;
        }
        matrix.swap(pivot_row, best_row);
        let pivot = matrix[pivot_row][column];
        for row in (pivot_row + 1)..row_count {
            let factor = matrix[row][column] / pivot;
            for col in column..column_count {
                matrix[row][col] -= factor * matrix[pivot_row][col];
            }
        }
        pivot_row += 1;
    }
    pivot_row
}

fn fisher_information(rows: &[[f64; 4]], noise: &[f64]) -> [[f64; 4]; 4] {
    let mut fisher = [[0.0; 4]; 4];
    for (row, sigma) in rows.iter().zip(noise.iter()) {
        let precision = 1.0 / (sigma * sigma);
        for (left, fisher_row) in fisher.iter_mut().enumerate() {
            for (right, value) in fisher_row.iter_mut().enumerate() {
                *value += row[left] * row[right] * precision;
            }
        }
    }
    fisher
}

#[allow(clippy::needless_range_loop)]
fn regularized_log_determinant(mut matrix: [[f64; 4]; 4]) -> Result<f64, String> {
    for (index, row) in matrix.iter_mut().enumerate() {
        row[index] += DETERMINANT_REGULARIZER;
    }
    let mut lower = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..=row {
            let correction: f64 = (0..column).map(|k| lower[row][k] * lower[column][k]).sum();
            if row == column {
                let diagonal = matrix[row][row] - correction;
                if !diagonal.is_finite() || diagonal <= 0.0 {
                    return Err("regularized Fisher matrix is not positive definite".into());
                }
                lower[row][column] = diagonal.sqrt();
            } else {
                lower[row][column] = (matrix[row][column] - correction) / lower[column][column];
            }
        }
    }
    Ok(2.0 * (0..4).map(|index| lower[index][index].ln()).sum::<f64>())
}

fn make_summary(
    candidate_ids: Vec<String>,
    total_cost: u32,
    budget: u32,
    rows: &[[f64; 4]],
    noise: &[f64],
) -> Result<DesignSummary, String> {
    let rank = rank_rows(rows);
    let fisher = fisher_information(rows, noise);
    Ok(DesignSummary {
        candidate_ids,
        total_cost,
        acquisition_budget: budget,
        sensitivity_rank: rank,
        parameter_count: 4,
        local_nullity: 4usize.saturating_sub(rank),
        regularized_log_determinant_fisher: regularized_log_determinant(fisher)?,
        fisher_information: fisher,
    })
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let bulk_row =
        log_conductance_sensitivity(ObservationKind::BulkConductance, &fixture.conductances);
    let baseline_rows = vec![bulk_row];
    let baseline_noise = vec![fixture.baseline_bulk_noise_std];
    let baseline = make_summary(
        Vec::new(),
        0,
        fixture.acquisition_budget,
        &baseline_rows,
        &baseline_noise,
    )?;

    // Cost-matched conventional baseline: spend the entire budget repeating
    // the same bulk experiment. It gains precision along the same direction,
    // but cannot remove the structural nullspace.
    let repeat_count = fixture.acquisition_budget as usize;
    let mut repeated_bulk_rows = baseline_rows.clone();
    let mut repeated_bulk_noise = baseline_noise.clone();
    let mut repeated_bulk_ids = Vec::with_capacity(repeat_count);
    let repeat_candidate = fixture
        .candidates
        .iter()
        .find(|candidate| matches!(candidate.observation, ObservationKind::BulkConductance))
        .ok_or_else(|| {
            "a repeat-bulk candidate is required for the cost-matched baseline".to_string()
        })?;
    if repeat_candidate.cost != 1 {
        return Err(
            "repeat-bulk candidate cost must be one unit for the cost-matched baseline".into(),
        );
    }
    for repeat in 0..repeat_count {
        repeated_bulk_rows.push(bulk_row);
        repeated_bulk_noise.push(repeat_candidate.noise_std);
        repeated_bulk_ids.push(format!("repeat-bulk-{}", repeat + 1));
    }
    let cost_matched_repeat_bulk_design = make_summary(
        repeated_bulk_ids,
        fixture.acquisition_budget,
        fixture.acquisition_budget,
        &repeated_bulk_rows,
        &repeated_bulk_noise,
    )?;

    let mut candidate_rows = Vec::with_capacity(fixture.candidates.len());
    for candidate in &fixture.candidates {
        candidate_rows.push(log_conductance_sensitivity(
            candidate.observation,
            &fixture.conductances,
        ));
    }

    let mut candidate_sensitivities = Vec::with_capacity(fixture.candidates.len());
    for (index, candidate) in fixture.candidates.iter().enumerate() {
        let mut rows = baseline_rows.clone();
        rows.push(candidate_rows[index]);
        candidate_sensitivities.push(CandidateSensitivity {
            candidate_id: candidate.candidate_id.clone(),
            observation: candidate.observation,
            cost: candidate.cost,
            noise_std: candidate.noise_std,
            derivative_with_respect_to_log_conductances: candidate_rows[index],
            rank_when_added_to_baseline: rank_rows(&rows),
        });
    }

    // Exhaustive, deterministic search is practical for <=16 candidates and
    // provides an oracle against which future greedy design can be measured.
    let combinations = 1u32 << fixture.candidates.len();
    let mut best: Option<DesignSummary> = None;
    for mask in 0..combinations {
        let mut ids = Vec::new();
        let mut cost = 0u32;
        let mut rows = baseline_rows.clone();
        let mut noise = baseline_noise.clone();
        for (index, candidate) in fixture.candidates.iter().enumerate() {
            if mask & (1u32 << index) != 0 {
                cost = cost.saturating_add(candidate.cost);
                ids.push(candidate.candidate_id.clone());
                rows.push(candidate_rows[index]);
                noise.push(candidate.noise_std);
            }
        }
        if cost > fixture.acquisition_budget {
            continue;
        }
        let summary = make_summary(ids, cost, fixture.acquisition_budget, &rows, &noise)?;
        let replace = match &best {
            None => true,
            Some(current) => {
                summary.regularized_log_determinant_fisher
                    > current.regularized_log_determinant_fisher + 1.0e-12
            }
        };
        if replace {
            best = Some(summary);
        }
    }
    let optimal_design = best.ok_or_else(|| "no feasible design found".to_string())?;

    let repeat_bulk_row = bulk_row;
    let repeated_rows = vec![bulk_row, repeat_bulk_row];
    let rank_after_repeat = rank_rows(&repeated_rows);
    let repeated_bulk_negative_control = RepeatedMeasurementControl {
        baseline_rank: rank_rows(&baseline_rows),
        rank_after_repeating_bulk_measurement: rank_after_repeat,
        rank_increase: rank_after_repeat.saturating_sub(rank_rows(&baseline_rows)),
        explanation: "Repeating the same noiseless linear bulk observation adds precision along the existing sensitivity direction but cannot enlarge its structural row space. Independent measurement noise may shrink uncertainty along that direction; the structural nullspace remains.".into(),
    };

    let g = fixture.conductances;
    let nominal_observations = NominalObservations {
        bulk_effective_conductance: predict(ObservationKind::BulkConductance, &g),
        upper_intermediate_pressure: predict(ObservationKind::UpperPressure, &g),
        lower_intermediate_pressure: predict(ObservationKind::LowerPressure, &g),
        upper_path_flow: predict(ObservationKind::UpperPathFlow, &g),
    };

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        parameterization: "four positive throat conductances represented by eta_i = log(g_i); local finite-difference sensitivities are evaluated at the declared nominal conductances".into(),
        nominal_observations,
        baseline,
        cost_matched_repeat_bulk_design,
        candidate_sensitivities,
        optimal_design,
        repeated_bulk_negative_control,
        interpretation_boundary: "This is a local synthetic experiment-design reference for a manufactured two-parallel-path network. Full local rank in this example depends on the idealized availability and support of both interior pressure probes and one localized upper-path flow observation. A whole-core boundary flux or integrated tracer curve is not that localized observation. Regularized log-determinant is a D-optimal ranking criterion, not a proof of global identifiability or a posterior confidence interval. No real soil, field sensor, or management decision has been validated.".into(),
    })
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!(
            "usage: {} INPUT.json OUTPUT.json",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("hydraulic-experiment-design-reference")
        ));
    }
    let input_path = &arguments[1];
    let output_path = &arguments[2];
    let input_bytes =
        fs::read(input_path).map_err(|error| format!("cannot read input: {error}"))?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let fixture: Fixture = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("invalid fixture JSON: {error}"))?;
    let report = analyze(&fixture, input_sha256)?;
    let output_bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report: {error}"))?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output
        .write_all(&output_bytes)
        .map_err(|error| format!("cannot write report: {error}"))?;
    output
        .write_all(b"\n")
        .map_err(|error| format!("cannot finalize report: {error}"))?;
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
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "two-parallel-path-active-design".into(),
            conductances: [2.0, 2.0, 2.0, 2.0],
            baseline_bulk_noise_std: 0.02,
            acquisition_budget: 4,
            candidates: vec![
                Candidate {
                    candidate_id: "upper-pressure-probe".into(),
                    observation: ObservationKind::UpperPressure,
                    noise_std: 0.01,
                    cost: 1,
                },
                Candidate {
                    candidate_id: "lower-pressure-probe".into(),
                    observation: ObservationKind::LowerPressure,
                    noise_std: 0.01,
                    cost: 1,
                },
                Candidate {
                    candidate_id: "localized-upper-path-flow".into(),
                    observation: ObservationKind::UpperPathFlow,
                    noise_std: 0.02,
                    cost: 2,
                },
                Candidate {
                    candidate_id: "repeat-bulk-measurement".into(),
                    observation: ObservationKind::BulkConductance,
                    noise_std: 0.02,
                    cost: 1,
                },
            ],
        }
    }

    #[test]
    fn one_bulk_observation_leaves_three_local_directions_unresolved() {
        let f = fixture();
        let bulk = [log_conductance_sensitivity(
            ObservationKind::BulkConductance,
            &f.conductances,
        )];
        assert_eq!(rank_rows(&bulk), 1);
    }

    #[test]
    fn two_internal_pressure_probes_increase_but_do_not_complete_rank() {
        let f = fixture();
        let rows = [
            log_conductance_sensitivity(ObservationKind::BulkConductance, &f.conductances),
            log_conductance_sensitivity(ObservationKind::UpperPressure, &f.conductances),
            log_conductance_sensitivity(ObservationKind::LowerPressure, &f.conductances),
        ];
        assert_eq!(rank_rows(&rows), 3);
    }

    #[test]
    fn localized_path_flow_plus_two_pressure_probes_identifies_all_four_locally() {
        let f = fixture();
        let rows = [
            log_conductance_sensitivity(ObservationKind::BulkConductance, &f.conductances),
            log_conductance_sensitivity(ObservationKind::UpperPressure, &f.conductances),
            log_conductance_sensitivity(ObservationKind::LowerPressure, &f.conductances),
            log_conductance_sensitivity(ObservationKind::UpperPathFlow, &f.conductances),
        ];
        assert_eq!(rank_rows(&rows), 4);
    }

    #[test]
    fn repeated_bulk_observation_does_not_increase_structural_rank() {
        let f = fixture();
        let row = log_conductance_sensitivity(ObservationKind::BulkConductance, &f.conductances);
        assert_eq!(rank_rows(&[row]), 1);
        assert_eq!(rank_rows(&[row, row]), 1);
    }

    #[test]
    fn cost_constrained_d_optimal_design_prefers_new_information() {
        let f = fixture();
        let report = analyze(&f, "0".repeat(64)).expect("analysis");
        assert_eq!(report.baseline.sensitivity_rank, 1);
        assert_eq!(report.cost_matched_repeat_bulk_design.sensitivity_rank, 1);
        assert_eq!(report.cost_matched_repeat_bulk_design.total_cost, 4);
        assert_eq!(report.cost_matched_repeat_bulk_design.local_nullity, 3);
        assert_eq!(report.optimal_design.sensitivity_rank, 4);
        assert_eq!(report.optimal_design.total_cost, 4);
        assert!(
            report.optimal_design.regularized_log_determinant_fisher
                > report
                    .cost_matched_repeat_bulk_design
                    .regularized_log_determinant_fisher
        );
        assert_eq!(
            report.optimal_design.candidate_ids,
            vec![
                "upper-pressure-probe",
                "lower-pressure-probe",
                "localized-upper-path-flow"
            ]
        );
        assert_eq!(report.repeated_bulk_negative_control.rank_increase, 0);
    }

    #[test]
    fn rejects_invalid_noise_and_duplicate_candidate_ids() {
        let mut invalid_noise = fixture();
        invalid_noise.candidates[0].noise_std = 0.0;
        assert!(validate(&invalid_noise).is_err());

        let mut duplicate = fixture();
        duplicate.candidates[1].candidate_id = duplicate.candidates[0].candidate_id.clone();
        assert!(validate(&duplicate).is_err());
    }
}
