use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const SCHEMA: &str = "terra-correlated-metrology-boed-v0.1";
const STATUS: &str = "synthetic_only_not_empirical_evidence";
const GH_NODES: [f64; 7] = [
    -3.750_439_717_725_742,
    -2.366_759_410_734_541,
    -1.154_405_394_739_968,
    0.0,
    1.154_405_394_739_968,
    2.366_759_410_734_541,
    3.750_439_717_725_742,
];
const GH_WEIGHTS: [f64; 7] = [
    0.000_548_268_855_972_218,
    0.030_757_123_967_586_5,
    0.240_123_178_605_012,
    0.457_142_857_142_857,
    0.240_123_178_605_012,
    0.030_757_123_967_586_5,
    0.000_548_268_855_972_218,
];

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Channel {
    channel_id: String,
    support_id: String,
    method_id: String,
    calibration_group: String,
    calibration_record_id: String,
    units: String,
    noise_std: f64,
    mechanism_means: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    fixture_id: String,
    description: String,
    mechanism_ids: Vec<String>,
    prior_probabilities: Vec<f64>,
    channels: Vec<Channel>,
    observation_correlation: Vec<Vec<f64>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LikelihoodMode {
    JointCovariance,
    IndependentNegativeControl,
}

#[derive(Clone, Debug, Serialize)]
struct PairScore {
    first_channel_id: String,
    second_channel_id: String,
    expected_posterior_entropy_nats: f64,
    expected_model_information_gain_nats: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    fixture_id: String,
    fixture_status: String,
    input_sha256: String,
    mechanism_count: usize,
    channel_count: usize,
    quadrature_nodes_per_channel: usize,
    correlation_matrix: Vec<Vec<f64>>,
    maximum_absolute_off_diagonal_correlation: f64,
    selected_joint_covariance_pair: PairScore,
    selected_independent_negative_control_pair: PairScore,
    independent_selected_pair_evaluated_with_joint_likelihood: PairScore,
    covariance_aware_design_advantage_nats: f64,
    independent_model_selection_penalty_nats: f64,
    duplicate_support_pair_rejected: bool,
    minimum_cholesky_diagonal: f64,
    covariance_semantics: String,
    claim_boundary: String,
}

#[derive(Clone, Debug)]
struct CholeskyFactor {
    lower: Vec<Vec<f64>>,
    log_determinant: f64,
}

#[derive(Clone, Copy, Debug)]
struct PairObservation {
    first_index: usize,
    second_index: usize,
    first_value: f64,
    second_value: f64,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != SCHEMA || fixture.fixture_status != STATUS {
        return Err("unsupported schema or non-synthetic fixture status".into());
    }
    if fixture.fixture_id.trim().is_empty() || fixture.description.trim().is_empty() {
        return Err("fixture identity and description must be non-empty".into());
    }
    if fixture.mechanism_ids.len() < 3 || fixture.channels.len() < 2 {
        return Err("at least three mechanisms and two channels are required".into());
    }
    if fixture.prior_probabilities.len() != fixture.mechanism_ids.len()
        || fixture
            .prior_probabilities
            .iter()
            .any(|probability| !probability.is_finite() || *probability <= 0.0)
    {
        return Err("mechanism priors must be finite, positive and dimensionally complete".into());
    }
    let prior_sum: f64 = fixture.prior_probabilities.iter().sum();
    if (prior_sum - 1.0).abs() > 1.0e-12 {
        return Err("mechanism priors must sum to one".into());
    }
    if has_duplicates(
        &fixture
            .mechanism_ids
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    ) {
        return Err("mechanism IDs must be unique".into());
    }
    let channel_ids: Vec<&str> = fixture
        .channels
        .iter()
        .map(|channel| channel.channel_id.as_str())
        .collect();
    if has_duplicates(&channel_ids) {
        return Err("channel IDs must be unique".into());
    }
    for channel in &fixture.channels {
        if channel.channel_id.trim().is_empty()
            || channel.support_id.trim().is_empty()
            || channel.method_id.trim().is_empty()
            || channel.calibration_group.trim().is_empty()
            || channel.calibration_record_id.trim().is_empty()
            || channel.units.trim().is_empty()
            || !channel.noise_std.is_finite()
            || channel.noise_std <= 0.0
            || channel.mechanism_means.len() != fixture.mechanism_ids.len()
            || channel.mechanism_means.iter().any(|mean| !mean.is_finite())
        {
            return Err(
                "channel support, method, calibration, units, noise or means are incomplete".into(),
            );
        }
    }
    if fixture.observation_correlation.len() != fixture.channels.len()
        || fixture
            .observation_correlation
            .iter()
            .any(|row| row.len() != fixture.channels.len())
    {
        return Err("observation correlation matrix has incorrect dimensions".into());
    }
    for index in 0..fixture.channels.len() {
        if (fixture.observation_correlation[index][index] - 1.0).abs() > 1.0e-12 {
            return Err("observation correlation diagonal must equal one".into());
        }
        for column in 0..fixture.channels.len() {
            if fixture.observation_correlation[index][column].abs() > 1.0 + 1.0e-12 {
                return Err("observation correlation must lie within [-1, 1]".into());
            }
        }
    }
    let _ = cholesky(&fixture.observation_correlation)?;
    Ok(())
}

fn has_duplicates(values: &[&str]) -> bool {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted.windows(2).any(|pair| pair[0] == pair[1])
}

fn cholesky(matrix: &[Vec<f64>]) -> Result<CholeskyFactor, String> {
    let size = matrix.len();
    if size == 0 || matrix.iter().any(|row| row.len() != size) {
        return Err("covariance matrix must be non-empty and square".into());
    }
    for row in matrix {
        if row.iter().any(|value| !value.is_finite()) {
            return Err("covariance matrix contains non-finite values".into());
        }
    }
    for row in 0..size {
        if matrix[row][row] <= 0.0 {
            return Err("covariance matrix diagonal must be strictly positive".into());
        }
        for column in 0..row {
            if (matrix[row][column] - matrix[column][row]).abs() > 1.0e-12 {
                return Err("covariance matrix must be symmetric".into());
            }
        }
    }

    let mut lower = vec![vec![0.0; size]; size];
    let mut log_determinant = 0.0;
    for row in 0..size {
        for column in 0..=row {
            let cross = (0..column)
                .map(|index| lower[row][index] * lower[column][index])
                .sum::<f64>();
            if row == column {
                let pivot = matrix[row][row] - cross;
                if !pivot.is_finite() || pivot <= 1.0e-14 {
                    return Err(
                        "covariance matrix is singular or not positive definite; no jitter applied"
                            .into(),
                    );
                }
                lower[row][column] = pivot.sqrt();
                log_determinant += pivot.ln();
            } else {
                lower[row][column] = (matrix[row][column] - cross) / lower[column][column];
            }
        }
    }
    Ok(CholeskyFactor {
        lower,
        log_determinant,
    })
}

fn covariance_for_pair(fixture: &Fixture, first: usize, second: usize) -> Vec<Vec<f64>> {
    let left = &fixture.channels[first];
    let right = &fixture.channels[second];
    let correlation = fixture.observation_correlation[first][second];
    vec![
        vec![
            left.noise_std.powi(2),
            correlation * left.noise_std * right.noise_std,
        ],
        vec![
            correlation * left.noise_std * right.noise_std,
            right.noise_std.powi(2),
        ],
    ]
}

fn log_mvn_density(residual: &[f64], covariance: &[Vec<f64>]) -> Result<f64, String> {
    if residual.len() != covariance.len() {
        return Err("residual/covariance dimension mismatch".into());
    }
    let factor = cholesky(covariance)?;
    let mut whitened = vec![0.0; residual.len()];
    for row in 0..residual.len() {
        let prior_sum = (0..row)
            .map(|column| factor.lower[row][column] * whitened[column])
            .sum::<f64>();
        whitened[row] = (residual[row] - prior_sum) / factor.lower[row][row];
    }
    let quadratic_form: f64 = whitened.iter().map(|value| value * value).sum();
    Ok(-0.5
        * (residual.len() as f64 * (2.0 * std::f64::consts::PI).ln()
            + factor.log_determinant
            + quadratic_form))
}

fn log_normal_density(observation: f64, mean: f64, standard_deviation: f64) -> f64 {
    let standardized = (observation - mean) / standard_deviation;
    -standard_deviation.ln()
        - 0.5 * (2.0 * std::f64::consts::PI).ln()
        - 0.5 * standardized * standardized
}

fn log_sum_exp(values: &[f64]) -> f64 {
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    maximum
        + values
            .iter()
            .map(|value| (value - maximum).exp())
            .sum::<f64>()
            .ln()
}

fn posterior_update(
    fixture: &Fixture,
    pair: PairObservation,
    mode: LikelihoodMode,
) -> Result<Vec<f64>, String> {
    let first = &fixture.channels[pair.first_index];
    let second = &fixture.channels[pair.second_index];
    let covariance = covariance_for_pair(fixture, pair.first_index, pair.second_index);
    let residuals: Vec<Vec<f64>> = fixture
        .mechanism_ids
        .iter()
        .enumerate()
        .map(|(mechanism_index, _)| {
            vec![
                pair.first_value - first.mechanism_means[mechanism_index],
                pair.second_value - second.mechanism_means[mechanism_index],
            ]
        })
        .collect();
    let mut log_weights = Vec::with_capacity(fixture.mechanism_ids.len());
    for (mechanism_index, residual) in residuals.iter().enumerate() {
        let log_likelihood = match mode {
            LikelihoodMode::JointCovariance => log_mvn_density(residual, &covariance)?,
            LikelihoodMode::IndependentNegativeControl => {
                log_normal_density(
                    pair.first_value,
                    first.mechanism_means[mechanism_index],
                    first.noise_std,
                ) + log_normal_density(
                    pair.second_value,
                    second.mechanism_means[mechanism_index],
                    second.noise_std,
                )
            }
        };
        log_weights.push(fixture.prior_probabilities[mechanism_index].ln() + log_likelihood);
    }
    let normalizer = log_sum_exp(&log_weights);
    Ok(log_weights
        .iter()
        .map(|weight| (weight - normalizer).exp())
        .collect())
}

fn entropy(probabilities: &[f64]) -> f64 {
    -probabilities
        .iter()
        .filter(|probability| **probability > 0.0)
        .map(|probability| probability * probability.ln())
        .sum::<f64>()
}

fn supports_are_distinct(fixture: &Fixture, first: usize, second: usize) -> bool {
    fixture.channels[first].support_id != fixture.channels[second].support_id
}

fn pair_observations(
    fixture: &Fixture,
    first: usize,
    second: usize,
    mechanism_index: usize,
    node_first: f64,
    node_second: f64,
) -> PairObservation {
    let left = &fixture.channels[first];
    let right = &fixture.channels[second];
    let correlation = fixture.observation_correlation[first][second];
    let conditional_scale = (1.0 - correlation.powi(2)).max(0.0).sqrt();
    PairObservation {
        first_index: first,
        second_index: second,
        first_value: left.mechanism_means[mechanism_index] + left.noise_std * node_first,
        second_value: right.mechanism_means[mechanism_index]
            + right.noise_std * (correlation * node_first + conditional_scale * node_second),
    }
}

fn expected_posterior_entropy(
    fixture: &Fixture,
    first: usize,
    second: usize,
    mode: LikelihoodMode,
) -> Result<f64, String> {
    if !supports_are_distinct(fixture, first, second) {
        return Err(
            "duplicate measurement support cannot be treated as an independent channel".into(),
        );
    }
    let mut expected_entropy = 0.0;
    for (mechanism_index, prior_probability) in fixture.prior_probabilities.iter().enumerate() {
        for (first_node, first_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            for (second_node, second_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
                let pair = pair_observations(
                    fixture,
                    first,
                    second,
                    mechanism_index,
                    *first_node,
                    *second_node,
                );
                let posterior = posterior_update(fixture, pair, mode)?;
                expected_entropy +=
                    prior_probability * first_weight * second_weight * entropy(&posterior);
            }
        }
    }
    Ok(expected_entropy)
}

fn best_pair(fixture: &Fixture, mode: LikelihoodMode) -> Result<(usize, usize, f64), String> {
    let mut best: Option<(usize, usize, f64)> = None;
    for first in 0..fixture.channels.len() {
        for second in (first + 1)..fixture.channels.len() {
            if !supports_are_distinct(fixture, first, second) {
                continue;
            }
            let score = expected_posterior_entropy(fixture, first, second, mode)?;
            let replace = best
                .as_ref()
                .map(|(best_first, best_second, best_score)| {
                    score < *best_score - 1.0e-12
                        || ((score - *best_score).abs() <= 1.0e-12
                            && (
                                fixture.channels[first].channel_id.as_str(),
                                fixture.channels[second].channel_id.as_str(),
                            ) < (
                                fixture.channels[*best_first].channel_id.as_str(),
                                fixture.channels[*best_second].channel_id.as_str(),
                            ))
                })
                .unwrap_or(true);
            if replace {
                best = Some((first, second, score));
            }
        }
    }
    best.ok_or_else(|| "no feasible distinct-support channel pair".into())
}

fn pair_score(
    fixture: &Fixture,
    first: usize,
    second: usize,
    entropy_mode: LikelihoodMode,
) -> Result<PairScore, String> {
    let expected_entropy = expected_posterior_entropy(fixture, first, second, entropy_mode)?;
    let prior_entropy = entropy(&fixture.prior_probabilities);
    Ok(PairScore {
        first_channel_id: fixture.channels[first].channel_id.clone(),
        second_channel_id: fixture.channels[second].channel_id.clone(),
        expected_posterior_entropy_nats: expected_entropy,
        expected_model_information_gain_nats: (prior_entropy - expected_entropy).max(0.0),
    })
}

fn shared_offset_covariance(
    residual_variances: [f64; 2],
    shared_offset_variance: f64,
) -> Vec<Vec<f64>> {
    vec![
        vec![
            residual_variances[0] + shared_offset_variance,
            shared_offset_variance,
        ],
        vec![
            shared_offset_variance,
            residual_variances[1] + shared_offset_variance,
        ],
    ]
}

fn log_marginal_shared_offset(
    residuals: [f64; 2],
    residual_variances: [f64; 2],
    shared_offset_variance: f64,
) -> Result<f64, String> {
    let covariance = shared_offset_covariance(residual_variances, shared_offset_variance);
    log_mvn_density(&residuals, &covariance)
}

fn log_shared_offset_closed_form(
    residuals: [f64; 2],
    residual_variances: [f64; 2],
    shared_offset_variance: f64,
) -> f64 {
    let variance_one = residual_variances[0] + shared_offset_variance;
    let variance_two = residual_variances[1] + shared_offset_variance;
    let determinant = variance_one * variance_two - shared_offset_variance.powi(2);
    let quadratic = (variance_two * residuals[0].powi(2)
        - 2.0 * shared_offset_variance * residuals[0] * residuals[1]
        + variance_one * residuals[1].powi(2))
        / determinant;
    -0.5 * (2.0 * (2.0 * std::f64::consts::PI).ln() + determinant.ln() + quadratic)
}

fn evaluate(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let (joint_first, joint_second, _) = best_pair(fixture, LikelihoodMode::JointCovariance)?;
    let (independent_first, independent_second, _) =
        best_pair(fixture, LikelihoodMode::IndependentNegativeControl)?;

    let joint_selected = pair_score(
        fixture,
        joint_first,
        joint_second,
        LikelihoodMode::JointCovariance,
    )?;
    let independent_selected_correctly_scored = pair_score(
        fixture,
        independent_first,
        independent_second,
        LikelihoodMode::JointCovariance,
    )?;
    let independent_selected_misspecified_score = pair_score(
        fixture,
        independent_first,
        independent_second,
        LikelihoodMode::IndependentNegativeControl,
    )?;

    let covariance_aware_design_advantage_nats = independent_selected_correctly_scored
        .expected_posterior_entropy_nats
        - joint_selected.expected_posterior_entropy_nats;
    let independent_model_selection_penalty_nats = independent_selected_correctly_scored
        .expected_posterior_entropy_nats
        - joint_selected.expected_posterior_entropy_nats;
    let maximum_absolute_off_diagonal_correlation = fixture
        .observation_correlation
        .iter()
        .enumerate()
        .flat_map(|(row, values)| {
            values
                .iter()
                .enumerate()
                .filter(move |(column, _)| *column != row)
                .map(|(_, value)| value.abs())
        })
        .fold(0.0, f64::max);
    let minimum_cholesky_diagonal = cholesky(&fixture.observation_correlation)?
        .lower
        .iter()
        .enumerate()
        .map(|(index, row)| row[index])
        .fold(f64::INFINITY, f64::min);
    let mut duplicate_support_fixture = fixture.clone();
    duplicate_support_fixture.channels[1].support_id =
        duplicate_support_fixture.channels[0].support_id.clone();
    let duplicate_support_pair_rejected = expected_posterior_entropy(
        &duplicate_support_fixture,
        0,
        1,
        LikelihoodMode::JointCovariance,
    )
    .is_err();

    Ok(Report {
        schema_version: SCHEMA.into(),
        fixture_id: fixture.fixture_id.clone(),
        fixture_status: fixture.fixture_status.clone(),
        input_sha256,
        mechanism_count: fixture.mechanism_ids.len(),
        channel_count: fixture.channels.len(),
        quadrature_nodes_per_channel: GH_NODES.len(),
        correlation_matrix: fixture.observation_correlation.clone(),
        maximum_absolute_off_diagonal_correlation,
        selected_joint_covariance_pair: joint_selected,
        selected_independent_negative_control_pair: PairScore {
            first_channel_id: fixture.channels[independent_first].channel_id.clone(),
            second_channel_id: fixture.channels[independent_second].channel_id.clone(),
            ..independent_selected_misspecified_score
        },
        independent_selected_pair_evaluated_with_joint_likelihood: independent_selected_correctly_scored,
        covariance_aware_design_advantage_nats,
        independent_model_selection_penalty_nats,
        duplicate_support_pair_rejected,
        minimum_cholesky_diagonal,
        covariance_semantics: "Observation correlation is converted to a dimensional covariance by Sigma_ij = rho_ij * sigma_i * sigma_j. The full bivariate Gaussian likelihood is evaluated by Cholesky factorization. The independent-channel negative control uses the same generated correlated outcomes but multiplies scalar Gaussian likelihoods. No diagonal jitter or silent fallback is applied.".into(),
        claim_boundary: "Synthetic likelihood and design reference only. Channel means, noise levels, covariance and support metadata are analyst-authored. No real instrument covariance, soil property, physical measurement or SOTA superiority is established.".into(),
    })
}

fn hex_digest(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn run(input_path: &str, output_path: &str) -> Result<Report, String> {
    let input_bytes = fs::read(input_path)
        .map_err(|error| format!("failed to read correlated-metrology fixture: {error}"))?;
    let input_sha256 = hex_digest(&Sha256::digest(&input_bytes));
    let fixture: Fixture = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("invalid correlated-metrology fixture JSON: {error}"))?;
    let report = evaluate(&fixture, input_sha256)?;
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("failed to serialize correlated-metrology report: {error}"))?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| {
            format!("output must be a new file; refusing overwrite at {output_path}: {error}")
        })?;
    file.write_all(&output)
        .map_err(|error| format!("failed to write correlated-metrology report: {error}"))?;
    println!(
        "channels={} mechanisms={} rho_max={:.3} joint_pair={}+{} independent_pair={}+{} correct_pair_entropy={:.9} independent_pick_correct_entropy={:.9} mis_spec_entropy={:.9} design_penalty={:.9} duplicate_support_rejected={} min_cholesky_diag={:.6}",
        report.channel_count,
        report.mechanism_count,
        report.maximum_absolute_off_diagonal_correlation,
        report.selected_joint_covariance_pair.first_channel_id,
        report.selected_joint_covariance_pair.second_channel_id,
        report
            .selected_independent_negative_control_pair
            .first_channel_id,
        report
            .selected_independent_negative_control_pair
            .second_channel_id,
        report
            .selected_joint_covariance_pair
            .expected_posterior_entropy_nats,
        report
            .independent_selected_pair_evaluated_with_joint_likelihood
            .expected_posterior_entropy_nats,
        report
            .selected_independent_negative_control_pair
            .expected_posterior_entropy_nats,
        report.independent_model_selection_penalty_nats,
        report.duplicate_support_pair_rejected,
        report.minimum_cholesky_diagonal
    );
    Ok(report)
}

fn main() {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        eprintln!("usage: terra-correlated-metrology-boed INPUT.json OUTPUT.json");
        process::exit(2);
    }
    if let Err(error) = run(&arguments[1], &arguments[2]) {
        eprintln!("correlated-metrology BOED error: {error}");
        process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Fixture {
        Fixture {
            schema_version: SCHEMA.into(),
            fixture_status: STATUS.into(),
            fixture_id: "shared-calibration-four-mechanism-test".into(),
            description: "Synthetic pressure-flow-tracer observation covariance".into(),
            mechanism_ids: vec![
                "balanced".into(),
                "preferential".into(),
                "dual_domain".into(),
                "forchheimer".into(),
            ],
            prior_probabilities: vec![0.25; 4],
            channels: vec![
                Channel {
                    channel_id: "inlet_pressure".into(),
                    support_id: "pressure-port-a".into(),
                    method_id: "pressure-transducer".into(),
                    calibration_group: "pressure-cal-2026-01".into(),
                    calibration_record_id: "CAL-P-01".into(),
                    units: "kPa".into(),
                    noise_std: 0.08,
                    mechanism_means: vec![1.0, 1.0, 1.0, 1.0],
                },
                Channel {
                    channel_id: "total_flow".into(),
                    support_id: "outlet-flow-port".into(),
                    method_id: "gravimetric-flow".into(),
                    calibration_group: "flow-cal-2026-02".into(),
                    calibration_record_id: "CAL-F-02".into(),
                    units: "mL/min".into(),
                    noise_std: 0.08,
                    mechanism_means: vec![1.0, 1.4, 0.8, 0.65],
                },
                Channel {
                    channel_id: "early_tracer_moment".into(),
                    support_id: "tracer-window-early".into(),
                    method_id: "tracer-breakthrough".into(),
                    calibration_group: "tracer-lab-03".into(),
                    calibration_record_id: "CAL-T-03".into(),
                    units: "min".into(),
                    noise_std: 0.2,
                    mechanism_means: vec![1.0, 0.45, 1.8, 1.2],
                },
                Channel {
                    channel_id: "late_tracer_fraction".into(),
                    support_id: "tracer-window-late".into(),
                    method_id: "tracer-breakthrough".into(),
                    calibration_group: "tracer-lab-03".into(),
                    calibration_record_id: "CAL-T-03".into(),
                    units: "fraction".into(),
                    noise_std: 0.07,
                    mechanism_means: vec![0.1, 0.02, 0.55, 0.15],
                },
            ],
            observation_correlation: vec![
                vec![1.0, 0.65, 0.08, 0.06],
                vec![0.65, 1.0, 0.06, 0.04],
                vec![0.08, 0.06, 1.0, 0.55],
                vec![0.06, 0.04, 0.55, 1.0],
            ],
        }
    }

    #[test]
    fn marginalized_common_offset_matches_closed_form_shared_nuisance_likelihood() {
        let residuals = [0.17, -0.23];
        let residual_variances = [0.04, 0.09];
        let shared_offset_variance = 0.025;
        let marginalized =
            log_marginal_shared_offset(residuals, residual_variances, shared_offset_variance)
                .unwrap();
        let closed_form =
            log_shared_offset_closed_form(residuals, residual_variances, shared_offset_variance);
        assert!((marginalized - closed_form).abs() < 1.0e-12);
    }

    #[test]
    fn common_offset_nuisance_induces_the_expected_off_diagonal_covariance() {
        let residual_variances = [0.04, 0.09];
        let shared_offset_variance = 0.025;
        let covariance = shared_offset_covariance(residual_variances, shared_offset_variance);
        assert!((covariance[0][0] - 0.065).abs() < 1.0e-12);
        assert!((covariance[1][1] - 0.115).abs() < 1.0e-12);
        assert!((covariance[0][1] - 0.025).abs() < 1.0e-12);
        assert!((covariance[1][0] - 0.025).abs() < 1.0e-12);
        assert!(cholesky(&covariance).is_ok());
    }

    #[test]
    fn covariance_must_be_positive_definite_and_is_never_jittered() {
        let singular = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        let result = cholesky(&singular);
        assert!(result.unwrap_err().contains("no jitter applied"));
    }

    #[test]
    fn covariance_rejects_asymmetry() {
        let asymmetric = vec![vec![1.0, 0.2], vec![0.1, 1.0]];
        assert!(cholesky(&asymmetric).unwrap_err().contains("symmetric"));
    }

    #[test]
    fn joint_likelihood_differs_from_independent_negative_control() {
        let f = fixture();
        let pair = PairObservation {
            first_index: 0,
            second_index: 1,
            first_value: 1.05,
            second_value: 1.22,
        };
        let joint = posterior_update(&f, pair, LikelihoodMode::JointCovariance).unwrap();
        let independent =
            posterior_update(&f, pair, LikelihoodMode::IndependentNegativeControl).unwrap();
        assert!(
            joint
                .iter()
                .zip(independent.iter())
                .any(|(left, right)| { (left - right).abs() > 1.0e-5 })
        );
        assert!((joint.iter().sum::<f64>() - 1.0).abs() < 1.0e-12);
        assert!((independent.iter().sum::<f64>() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn duplicate_support_is_not_admitted_as_an_independent_pair() {
        let mut f = fixture();
        f.channels[1].support_id = f.channels[0].support_id.clone();
        assert!(
            expected_posterior_entropy(&f, 0, 1, LikelihoodMode::JointCovariance)
                .unwrap_err()
                .contains("duplicate measurement support")
        );
    }

    #[test]
    fn channel_permutation_preserves_joint_posterior() {
        let f = fixture();
        let pair = PairObservation {
            first_index: 0,
            second_index: 2,
            first_value: 1.05,
            second_value: 1.75,
        };
        let posterior = posterior_update(&f, pair, LikelihoodMode::JointCovariance).unwrap();
        let mut reversed = fixture();
        reversed.channels.reverse();
        reversed.observation_correlation = f
            .observation_correlation
            .iter()
            .rev()
            .map(|row| row.iter().rev().copied().collect())
            .collect();
        let reversed_pair = PairObservation {
            first_index: 3,
            second_index: 1,
            first_value: pair.first_value,
            second_value: pair.second_value,
        };
        let reversed_posterior =
            posterior_update(&reversed, reversed_pair, LikelihoodMode::JointCovariance).unwrap();
        assert!(
            posterior
                .iter()
                .zip(reversed_posterior.iter())
                .all(|(left, right)| (left - right).abs() < 1.0e-12)
        );
    }

    #[test]
    fn unit_rescaling_preserves_posterior_when_noise_scales_with_units() {
        let mut f = fixture();
        let pair = PairObservation {
            first_index: 0,
            second_index: 2,
            first_value: 1.05,
            second_value: 1.75,
        };
        let baseline = posterior_update(&f, pair, LikelihoodMode::JointCovariance).unwrap();
        let scale = 1000.0;
        f.channels[0].noise_std *= scale;
        for mean in &mut f.channels[0].mechanism_means {
            *mean *= scale;
        }
        let scaled = PairObservation {
            first_index: 0,
            second_index: 2,
            first_value: pair.first_value * scale,
            second_value: pair.second_value,
        };
        let posterior = posterior_update(&f, scaled, LikelihoodMode::JointCovariance).unwrap();
        assert!(
            baseline
                .iter()
                .zip(posterior.iter())
                .all(|(left, right)| (left - right).abs() < 1.0e-12)
        );
    }

    #[test]
    fn expected_design_search_is_deterministic_and_support_safe() {
        let f = fixture();
        let left = best_pair(&f, LikelihoodMode::JointCovariance).unwrap();
        let right = best_pair(&f, LikelihoodMode::JointCovariance).unwrap();
        assert_eq!((left.0, left.1), (right.0, right.1));
        assert_ne!(f.channels[left.0].support_id, f.channels[left.1].support_id);
    }
}
