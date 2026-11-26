use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const FIXTURE_SCHEMA: &str = "terra-hydraulic-paired-world-calibration-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
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
const FAMILY_BINS: usize = 10;
const RESCALE_FACTOR: f64 = 37.0;
const OOD_SHIFT_STANDARD_DEVIATIONS: f64 = 20.0;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Candidate {
    candidate_id: String,
    noise_std: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MechanismFamily {
    family_id: String,
    base_expected_observables: Vec<f64>,
    base_heldout_mean: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    fixture_id: String,
    description: String,
    seed: u64,
    worlds_per_family: usize,
    parameter_regimes: Vec<f64>,
    first_candidate_id: String,
    fixed_second_candidate_id: String,
    heldout_noise_std: f64,
    discrepancy_threshold_sd: f64,
    sbc_posterior_draws: usize,
    candidates: Vec<Candidate>,
    families: Vec<MechanismFamily>,
}

#[derive(Clone, Debug, Serialize)]
struct ReliabilityBin {
    family_id: String,
    probability_bin: usize,
    count: u64,
    mean_predicted_probability: Option<f64>,
    empirical_frequency: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct PolicyReport {
    policy_id: String,
    worlds: u64,
    family_brier_score: f64,
    family_log_loss_nats: f64,
    family_accuracy: f64,
    family_probability_ece: f64,
    regime_brier_score: f64,
    regime_log_loss_nats: f64,
    regime_accuracy: f64,
    discrete_regime_sbc_rank_histogram: Vec<u64>,
    heldout_predictive_log_score_nats: f64,
    heldout_predictive_crps: f64,
    heldout_50_percent_interval_coverage: f64,
    heldout_50_percent_interval_mean_width: f64,
    heldout_80_percent_interval_coverage: f64,
    heldout_80_percent_interval_mean_width: f64,
    heldout_95_percent_interval_coverage: f64,
    heldout_95_percent_interval_mean_width: f64,
    in_family_abstention_rate: f64,
    shifted_residual_detection_rate: f64,
    maximum_unit_rescaling_posterior_error: f64,
    maximum_observation_order_posterior_error: f64,
    second_candidate_selection_counts: Vec<(String, u64)>,
    family_reliability_bins: Vec<ReliabilityBin>,
}

#[derive(Clone, Debug, Serialize)]
struct PairedDifference {
    metric: String,
    adaptive_minus_fixed_mean: f64,
    standard_error: f64,
    lower_95_percent: f64,
    upper_95_percent: f64,
    interpretation: String,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    fixture_id: String,
    fixture_status: String,
    input_sha256: String,
    world_stream_sha256: String,
    root_seed: u64,
    family_count: usize,
    parameter_regime_count: usize,
    candidate_count: usize,
    worlds_per_family: usize,
    worlds_per_parameter_regime_per_family: usize,
    total_paired_worlds: u64,
    common_random_numbers: bool,
    fixed_first_candidate_id: String,
    fixed_second_candidate_id: String,
    adaptive_policy: String,
    discrete_parameter_sbc_definition: String,
    ood_control_definition: String,
    claim_boundary: String,
    fixed_policy: PolicyReport,
    adaptive_policy_report: PolicyReport,
    paired_differences: Vec<PairedDifference>,
}

#[derive(Clone, Copy, Debug, Default)]
struct BinAccumulator {
    count: u64,
    predicted_sum: f64,
    observed_sum: f64,
}

#[derive(Clone, Debug)]
struct Accumulator {
    worlds: u64,
    family_brier_sum: f64,
    family_log_loss_sum: f64,
    family_correct: u64,
    regime_brier_sum: f64,
    regime_log_loss_sum: f64,
    regime_correct: u64,
    sbc_rank_counts: Vec<u64>,
    predictive_log_score_sum: f64,
    predictive_crps_sum: f64,
    interval_coverage: [u64; 3],
    interval_width_sum: [f64; 3],
    in_family_abstentions: u64,
    shifted_residual_detections: u64,
    maximum_rescale_error: f64,
    maximum_order_error: f64,
    second_choice_counts: Vec<u64>,
    reliability_bins: Vec<Vec<BinAccumulator>>,
}

#[derive(Clone, Debug)]
struct WorldScore {
    family_probabilities: Vec<f64>,
    family_brier: f64,
    family_log_loss: f64,
    family_correct: bool,
    regime_brier: f64,
    regime_log_loss: f64,
    regime_correct: bool,
    sbc_rank: usize,
    predictive_log_score: f64,
    predictive_crps: f64,
    interval_coverage: [bool; 3],
    interval_width: [f64; 3],
    in_family_abstained: bool,
    shifted_residual_detected: bool,
    rescale_error: f64,
    order_error: f64,
    second_choice_index: usize,
}

#[derive(Clone, Debug, Default)]
struct DeltaAccumulator {
    count: u64,
    sums: [f64; 4],
    squared_sums: [f64; 4],
}

struct SplitMix64 {
    state: u64,
    spare_normal: Option<f64>,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self {
            state: seed,
            spare_normal: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        value ^ (value >> 31)
    }

    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / ((1_u64 << 53) as f64)
    }

    fn normal(&mut self) -> f64 {
        if let Some(value) = self.spare_normal.take() {
            return value;
        }
        let radius = (-2.0 * self.uniform().ln()).sqrt();
        let angle = 2.0 * std::f64::consts::PI * self.uniform();
        self.spare_normal = Some(radius * angle.sin());
        radius * angle.cos()
    }
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != FIXTURE_SCHEMA || fixture.fixture_status != FIXTURE_STATUS {
        return Err("unsupported schema or non-synthetic fixture status".into());
    }
    if fixture.fixture_id.trim().is_empty() || fixture.description.trim().is_empty() {
        return Err("fixture identity and description must be non-empty".into());
    }
    if fixture.families.len() < 10 {
        return Err("calibration fixture requires at least ten mechanism families".into());
    }
    if fixture.parameter_regimes.len() < 3
        || fixture
            .parameter_regimes
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err("at least three finite positive parameter regimes are required".into());
    }
    if fixture.worlds_per_family < 1_000
        || fixture.worlds_per_family % fixture.parameter_regimes.len() != 0
    {
        return Err("worlds_per_family must be >= 1000 and divisible by regime count".into());
    }
    if fixture.sbc_posterior_draws < 20 || fixture.sbc_posterior_draws > 10_000 {
        return Err("SBC posterior draw count must be in 20..=10000".into());
    }
    if fixture.candidates.len() < 3
        || fixture.candidates.iter().any(|candidate| {
            candidate.candidate_id.trim().is_empty()
                || !candidate.noise_std.is_finite()
                || candidate.noise_std <= 0.0
        })
    {
        return Err("at least three candidates with finite positive noise are required".into());
    }
    if fixture.heldout_noise_std <= 0.0 || !fixture.heldout_noise_std.is_finite() {
        return Err("heldout noise must be finite and positive".into());
    }
    if fixture.discrepancy_threshold_sd <= 0.0 || !fixture.discrepancy_threshold_sd.is_finite() {
        return Err("discrepancy threshold must be finite and positive".into());
    }
    let first = candidate_index(fixture, &fixture.first_candidate_id)?;
    let fixed_second = candidate_index(fixture, &fixture.fixed_second_candidate_id)?;
    if first == fixed_second {
        return Err("first and fixed second candidate must differ".into());
    }
    let candidate_ids: Vec<&str> = fixture
        .candidates
        .iter()
        .map(|candidate| candidate.candidate_id.as_str())
        .collect();
    let family_ids: Vec<&str> = fixture
        .families
        .iter()
        .map(|family| family.family_id.as_str())
        .collect();
    if has_duplicates(&candidate_ids) || has_duplicates(&family_ids) {
        return Err("candidate and family identifiers must be unique".into());
    }
    for family in &fixture.families {
        if family.family_id.trim().is_empty()
            || family.base_expected_observables.len() != fixture.candidates.len()
            || family
                .base_expected_observables
                .iter()
                .any(|value| !value.is_finite())
            || !family.base_heldout_mean.is_finite()
        {
            return Err(
                "family response vectors must match candidate dimensions and be finite".into(),
            );
        }
    }
    let mut regimes = fixture.parameter_regimes.clone();
    regimes.sort_by(f64::total_cmp);
    if regimes
        .windows(2)
        .any(|pair| (pair[1] - pair[0]).abs() <= 1.0e-12)
    {
        return Err("parameter regimes must be unique".into());
    }
    Ok(())
}

fn has_duplicates(values: &[&str]) -> bool {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted.windows(2).any(|pair| pair[0] == pair[1])
}

fn candidate_index(fixture: &Fixture, id: &str) -> Result<usize, String> {
    fixture
        .candidates
        .iter()
        .position(|candidate| candidate.candidate_id == id)
        .ok_or_else(|| format!("unknown candidate id: {id}"))
}

fn class_count(fixture: &Fixture) -> usize {
    fixture.families.len() * fixture.parameter_regimes.len()
}

fn class_mean(fixture: &Fixture, class_index: usize, candidate_index: usize) -> f64 {
    let regime_count = fixture.parameter_regimes.len();
    let family_index = class_index / regime_count;
    let regime_index = class_index % regime_count;
    fixture.families[family_index].base_expected_observables[candidate_index]
        * fixture.parameter_regimes[regime_index]
}

fn class_heldout_mean(fixture: &Fixture, class_index: usize) -> f64 {
    let regime_count = fixture.parameter_regimes.len();
    let family_index = class_index / regime_count;
    let regime_index = class_index % regime_count;
    fixture.families[family_index].base_heldout_mean * fixture.parameter_regimes[regime_index]
}

fn log_normal_density(observation: f64, mean: f64, standard_deviation: f64) -> f64 {
    let standardized = (observation - mean) / standard_deviation;
    -standard_deviation.ln()
        - 0.5 * (2.0 * std::f64::consts::PI).ln()
        - 0.5 * standardized * standardized
}

fn log_sum_exp(values: &[f64]) -> f64 {
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !maximum.is_finite() {
        return maximum;
    }
    maximum
        + values
            .iter()
            .map(|value| (value - maximum).exp())
            .sum::<f64>()
            .ln()
}

fn posterior_update(
    fixture: &Fixture,
    prior: &[f64],
    selected: &[usize],
    observations: &[f64],
    scale: f64,
) -> Vec<f64> {
    let log_weights: Vec<f64> = (0..class_count(fixture))
        .map(|class_index| {
            let log_likelihood = selected
                .iter()
                .zip(observations.iter())
                .map(|(candidate_index, observation)| {
                    let candidate = &fixture.candidates[*candidate_index];
                    log_normal_density(
                        observation * scale,
                        class_mean(fixture, class_index, *candidate_index) * scale,
                        candidate.noise_std * scale,
                    )
                })
                .sum::<f64>();
            prior[class_index].ln() + log_likelihood
        })
        .collect();
    let normalizer = log_sum_exp(&log_weights);
    log_weights
        .iter()
        .map(|value| (value - normalizer).exp())
        .collect()
}

fn uniform_prior(fixture: &Fixture) -> Vec<f64> {
    vec![1.0 / class_count(fixture) as f64; class_count(fixture)]
}

fn entropy(probabilities: &[f64]) -> f64 {
    -probabilities
        .iter()
        .filter(|probability| **probability > 0.0)
        .map(|probability| probability * probability.ln())
        .sum::<f64>()
}

fn expected_entropy_after_candidate(
    fixture: &Fixture,
    prior: &[f64],
    candidate_index: usize,
) -> f64 {
    let mut expected_entropy = 0.0;
    for (true_class, class_probability) in prior.iter().enumerate() {
        for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let observation = class_mean(fixture, true_class, candidate_index)
                + fixture.candidates[candidate_index].noise_std * node;
            let posterior =
                posterior_update(fixture, prior, &[candidate_index], &[observation], 1.0);
            expected_entropy += class_probability * quadrature_weight * entropy(&posterior);
        }
    }
    expected_entropy
}

fn choose_adaptive_second(
    fixture: &Fixture,
    posterior_after_first: &[f64],
    first_index: usize,
) -> usize {
    let mut best: Option<(usize, f64)> = None;
    for candidate_index in 0..fixture.candidates.len() {
        if candidate_index == first_index {
            continue;
        }
        let score =
            expected_entropy_after_candidate(fixture, posterior_after_first, candidate_index);
        let replace = best
            .as_ref()
            .map(|(best_index, best_score)| {
                score < *best_score - 1.0e-12
                    || ((score - *best_score).abs() <= 1.0e-12
                        && fixture.candidates[candidate_index].candidate_id
                            < fixture.candidates[*best_index].candidate_id)
            })
            .unwrap_or(true);
        if replace {
            best = Some((candidate_index, score));
        }
    }
    best.expect("validated fixture has at least two distinct candidates")
        .0
}

fn argmax(values: &[f64]) -> usize {
    values
        .iter()
        .enumerate()
        .max_by(|left, right| left.1.total_cmp(right.1))
        .map(|(index, _)| index)
        .unwrap_or(0)
}

fn family_probabilities(fixture: &Fixture, posterior: &[f64]) -> Vec<f64> {
    let regime_count = fixture.parameter_regimes.len();
    (0..fixture.families.len())
        .map(|family_index| {
            posterior[family_index * regime_count..(family_index + 1) * regime_count]
                .iter()
                .sum()
        })
        .collect()
}

fn regime_probabilities(fixture: &Fixture, posterior: &[f64]) -> Vec<f64> {
    let regime_count = fixture.parameter_regimes.len();
    (0..regime_count)
        .map(|regime_index| {
            posterior
                .iter()
                .skip(regime_index)
                .step_by(regime_count)
                .sum()
        })
        .collect()
}

fn normal_cdf(value: f64) -> f64 {
    let sign = if value < 0.0 { -1.0 } else { 1.0 };
    let absolute = value.abs() / std::f64::consts::SQRT_2;
    let t = 1.0 / (1.0 + 0.327_591_1 * absolute);
    let polynomial =
        (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t
            + 0.254_829_592)
            * t;
    let erf = sign * (1.0 - polynomial * (-absolute * absolute).exp());
    0.5 * (1.0 + erf)
}

fn expected_absolute_normal(mean: f64, standard_deviation: f64) -> f64 {
    let z = mean / standard_deviation;
    standard_deviation * (2.0 / std::f64::consts::PI).sqrt() * (-0.5 * z * z).exp()
        + mean * (2.0 * normal_cdf(z) - 1.0)
}

fn gaussian_mixture_crps(
    probabilities: &[f64],
    means: &[f64],
    standard_deviation: f64,
    observation: f64,
) -> f64 {
    let first_term: f64 = probabilities
        .iter()
        .zip(means.iter())
        .map(|(probability, mean)| {
            probability * expected_absolute_normal(mean - observation, standard_deviation)
        })
        .sum();
    let mut second_term = 0.0;
    let difference_standard_deviation = std::f64::consts::SQRT_2 * standard_deviation;
    for (left_probability, left_mean) in probabilities.iter().zip(means.iter()) {
        for (right_probability, right_mean) in probabilities.iter().zip(means.iter()) {
            second_term += left_probability
                * right_probability
                * expected_absolute_normal(left_mean - right_mean, difference_standard_deviation);
        }
    }
    (first_term - 0.5 * second_term).max(0.0)
}

fn mixture_cdf(value: f64, probabilities: &[f64], means: &[f64], standard_deviation: f64) -> f64 {
    probabilities
        .iter()
        .zip(means.iter())
        .map(|(probability, mean)| probability * normal_cdf((value - mean) / standard_deviation))
        .sum()
}

fn mixture_quantile(
    probability: f64,
    weights: &[f64],
    means: &[f64],
    standard_deviation: f64,
) -> f64 {
    let mut lower = means.iter().copied().fold(f64::INFINITY, f64::min) - 8.0 * standard_deviation;
    let mut upper =
        means.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 8.0 * standard_deviation;
    for _ in 0..28 {
        let midpoint = 0.5 * (lower + upper);
        if mixture_cdf(midpoint, weights, means, standard_deviation) < probability {
            lower = midpoint;
        } else {
            upper = midpoint;
        }
    }
    0.5 * (lower + upper)
}

fn discrepancy_score(fixture: &Fixture, selected: &[usize], observations: &[f64]) -> f64 {
    (0..class_count(fixture))
        .map(|class_index| {
            selected
                .iter()
                .zip(observations.iter())
                .map(|(candidate_index, observation)| {
                    ((observation - class_mean(fixture, class_index, *candidate_index))
                        / fixture.candidates[*candidate_index].noise_std)
                        .abs()
                })
                .fold(0.0, f64::max)
        })
        .fold(f64::INFINITY, f64::min)
}

fn randomized_discrete_sbc_rank(
    regime_probabilities: &[f64],
    true_regime: usize,
    posterior_draw_uniforms: &[f64],
    tie_uniform: f64,
) -> usize {
    let mut less = 0_usize;
    let mut equal = 0_usize;
    for uniform in posterior_draw_uniforms {
        let mut cumulative = 0.0;
        let mut sampled_regime = regime_probabilities.len() - 1;
        for (regime_index, probability) in regime_probabilities.iter().enumerate() {
            cumulative += probability;
            if *uniform <= cumulative {
                sampled_regime = regime_index;
                break;
            }
        }
        if sampled_regime < true_regime {
            less += 1;
        } else if sampled_regime == true_regime {
            equal += 1;
        }
    }
    let randomized_tie = ((tie_uniform * (equal + 1) as f64).floor() as usize).min(equal);
    less + randomized_tie
}

fn score_world(
    fixture: &Fixture,
    posterior: &[f64],
    selected: &[usize],
    observations: &[f64],
    heldout_observation: f64,
    true_family: usize,
    true_regime: usize,
    posterior_draw_uniforms: &[f64],
    tie_uniform: f64,
    second_choice_index: usize,
) -> WorldScore {
    let family_probabilities = family_probabilities(fixture, posterior);
    let regime_probabilities = regime_probabilities(fixture, posterior);

    let family_brier = family_probabilities
        .iter()
        .enumerate()
        .map(|(index, probability)| {
            (probability - if index == true_family { 1.0 } else { 0.0 }).powi(2)
        })
        .sum();
    let regime_brier = regime_probabilities
        .iter()
        .enumerate()
        .map(|(index, probability)| {
            (probability - if index == true_regime { 1.0 } else { 0.0 }).powi(2)
        })
        .sum();
    let family_log_loss = -family_probabilities[true_family].max(1.0e-15).ln();
    let regime_log_loss = -regime_probabilities[true_regime].max(1.0e-15).ln();
    let family_correct = argmax(&family_probabilities) == true_family;
    let regime_correct = argmax(&regime_probabilities) == true_regime;
    let sbc_rank = randomized_discrete_sbc_rank(
        &regime_probabilities,
        true_regime,
        posterior_draw_uniforms,
        tie_uniform,
    );

    let heldout_means: Vec<f64> = (0..class_count(fixture))
        .map(|class_index| class_heldout_mean(fixture, class_index))
        .collect();
    let predictive_log_terms: Vec<f64> = posterior
        .iter()
        .zip(heldout_means.iter())
        .map(|(probability, mean)| {
            probability.ln()
                + log_normal_density(heldout_observation, *mean, fixture.heldout_noise_std)
        })
        .collect();
    let predictive_log_score = log_sum_exp(&predictive_log_terms);
    let predictive_crps = gaussian_mixture_crps(
        posterior,
        &heldout_means,
        fixture.heldout_noise_std,
        heldout_observation,
    );

    let central_intervals = [(0.25, 0.75), (0.10, 0.90), (0.025, 0.975)];
    let mut interval_coverage = [false; 3];
    let mut interval_width = [0.0; 3];
    for (index, (lower_probability, upper_probability)) in central_intervals.iter().enumerate() {
        let lower = mixture_quantile(
            *lower_probability,
            posterior,
            &heldout_means,
            fixture.heldout_noise_std,
        );
        let upper = mixture_quantile(
            *upper_probability,
            posterior,
            &heldout_means,
            fixture.heldout_noise_std,
        );
        interval_coverage[index] = heldout_observation >= lower && heldout_observation <= upper;
        interval_width[index] = upper - lower;
    }

    let in_family_abstained =
        discrepancy_score(fixture, selected, observations) > fixture.discrepancy_threshold_sd;
    let shifted_observations: Vec<f64> = selected
        .iter()
        .zip(observations.iter())
        .map(|(candidate_index, observation)| {
            observation
                + OOD_SHIFT_STANDARD_DEVIATIONS * fixture.candidates[*candidate_index].noise_std
        })
        .collect();
    let shifted_residual_detected = discrepancy_score(fixture, selected, &shifted_observations)
        > fixture.discrepancy_threshold_sd;

    let scaled_posterior = posterior_update(
        fixture,
        &uniform_prior(fixture),
        selected,
        observations,
        RESCALE_FACTOR,
    );
    let rescale_error = posterior
        .iter()
        .zip(scaled_posterior.iter())
        .map(|(left, right)| (left - right).abs())
        .fold(0.0, f64::max);

    let reversed_selected: Vec<usize> = selected.iter().rev().copied().collect();
    let reversed_observations: Vec<f64> = observations.iter().rev().copied().collect();
    let reordered_posterior = posterior_update(
        fixture,
        &uniform_prior(fixture),
        &reversed_selected,
        &reversed_observations,
        1.0,
    );
    let order_error = posterior
        .iter()
        .zip(reordered_posterior.iter())
        .map(|(left, right)| (left - right).abs())
        .fold(0.0, f64::max);

    WorldScore {
        family_probabilities,
        family_brier,
        family_log_loss,
        family_correct,
        regime_brier,
        regime_log_loss,
        regime_correct,
        sbc_rank,
        predictive_log_score,
        predictive_crps,
        interval_coverage,
        interval_width,
        in_family_abstained,
        shifted_residual_detected,
        rescale_error,
        order_error,
        second_choice_index,
    }
}

impl Accumulator {
    fn new(fixture: &Fixture) -> Self {
        Self {
            worlds: 0,
            family_brier_sum: 0.0,
            family_log_loss_sum: 0.0,
            family_correct: 0,
            regime_brier_sum: 0.0,
            regime_log_loss_sum: 0.0,
            regime_correct: 0,
            sbc_rank_counts: vec![0; fixture.sbc_posterior_draws + 1],
            predictive_log_score_sum: 0.0,
            predictive_crps_sum: 0.0,
            interval_coverage: [0; 3],
            interval_width_sum: [0.0; 3],
            in_family_abstentions: 0,
            shifted_residual_detections: 0,
            maximum_rescale_error: 0.0,
            maximum_order_error: 0.0,
            second_choice_counts: vec![0; fixture.candidates.len()],
            reliability_bins: vec![
                vec![BinAccumulator::default(); FAMILY_BINS];
                fixture.families.len()
            ],
        }
    }

    fn record(&mut self, score: &WorldScore, true_family: usize) {
        self.worlds += 1;
        self.family_brier_sum += score.family_brier;
        self.family_log_loss_sum += score.family_log_loss;
        self.family_correct += score.family_correct as u64;
        self.regime_brier_sum += score.regime_brier;
        self.regime_log_loss_sum += score.regime_log_loss;
        self.regime_correct += score.regime_correct as u64;
        self.sbc_rank_counts[score.sbc_rank] += 1;
        self.predictive_log_score_sum += score.predictive_log_score;
        self.predictive_crps_sum += score.predictive_crps;
        for index in 0..3 {
            self.interval_coverage[index] += score.interval_coverage[index] as u64;
            self.interval_width_sum[index] += score.interval_width[index];
        }
        self.in_family_abstentions += score.in_family_abstained as u64;
        self.shifted_residual_detections += score.shifted_residual_detected as u64;
        self.maximum_rescale_error = self.maximum_rescale_error.max(score.rescale_error);
        self.maximum_order_error = self.maximum_order_error.max(score.order_error);
        self.second_choice_counts[score.second_choice_index] += 1;

        for (family_index, probability) in score.family_probabilities.iter().enumerate() {
            let bin_index =
                ((*probability * FAMILY_BINS as f64).floor() as usize).min(FAMILY_BINS - 1);
            let bin = &mut self.reliability_bins[family_index][bin_index];
            bin.count += 1;
            bin.predicted_sum += probability;
            bin.observed_sum += if family_index == true_family {
                1.0
            } else {
                0.0
            };
        }
    }

    fn report(&self, fixture: &Fixture, policy_id: &str) -> PolicyReport {
        let divisor = self.worlds.max(1) as f64;
        let mut family_reliability_bins = Vec::new();
        let mut calibration_error = 0.0;
        let reliability_divisor = (self.worlds * fixture.families.len() as u64).max(1) as f64;
        for (family_index, bins) in self.reliability_bins.iter().enumerate() {
            for (bin_index, bin) in bins.iter().enumerate() {
                let (mean_predicted_probability, empirical_frequency) = if bin.count == 0 {
                    (None, None)
                } else {
                    let predicted = bin.predicted_sum / bin.count as f64;
                    let empirical = bin.observed_sum / bin.count as f64;
                    calibration_error +=
                        bin.count as f64 * (predicted - empirical).abs() / reliability_divisor;
                    (Some(predicted), Some(empirical))
                };
                family_reliability_bins.push(ReliabilityBin {
                    family_id: fixture.families[family_index].family_id.clone(),
                    probability_bin: bin_index,
                    count: bin.count,
                    mean_predicted_probability,
                    empirical_frequency,
                });
            }
        }

        let mut second_candidate_selection_counts: Vec<(String, u64)> = fixture
            .candidates
            .iter()
            .enumerate()
            .map(|(index, candidate)| {
                (
                    candidate.candidate_id.clone(),
                    self.second_choice_counts[index],
                )
            })
            .collect();
        second_candidate_selection_counts.sort_by(|left, right| left.0.cmp(&right.0));

        PolicyReport {
            policy_id: policy_id.into(),
            worlds: self.worlds,
            family_brier_score: self.family_brier_sum / divisor,
            family_log_loss_nats: self.family_log_loss_sum / divisor,
            family_accuracy: self.family_correct as f64 / divisor,
            family_probability_ece: calibration_error,
            regime_brier_score: self.regime_brier_sum / divisor,
            regime_log_loss_nats: self.regime_log_loss_sum / divisor,
            regime_accuracy: self.regime_correct as f64 / divisor,
            discrete_regime_sbc_rank_histogram: self.sbc_rank_counts.clone(),
            heldout_predictive_log_score_nats: self.predictive_log_score_sum / divisor,
            heldout_predictive_crps: self.predictive_crps_sum / divisor,
            heldout_50_percent_interval_coverage: self.interval_coverage[0] as f64 / divisor,
            heldout_50_percent_interval_mean_width: self.interval_width_sum[0] / divisor,
            heldout_80_percent_interval_coverage: self.interval_coverage[1] as f64 / divisor,
            heldout_80_percent_interval_mean_width: self.interval_width_sum[1] / divisor,
            heldout_95_percent_interval_coverage: self.interval_coverage[2] as f64 / divisor,
            heldout_95_percent_interval_mean_width: self.interval_width_sum[2] / divisor,
            in_family_abstention_rate: self.in_family_abstentions as f64 / divisor,
            shifted_residual_detection_rate: self.shifted_residual_detections as f64 / divisor,
            maximum_unit_rescaling_posterior_error: self.maximum_rescale_error,
            maximum_observation_order_posterior_error: self.maximum_order_error,
            second_candidate_selection_counts,
            family_reliability_bins,
        }
    }
}

impl DeltaAccumulator {
    fn record(&mut self, fixed: &WorldScore, adaptive: &WorldScore) {
        let deltas = [
            adaptive.family_brier - fixed.family_brier,
            adaptive.family_log_loss - fixed.family_log_loss,
            adaptive.predictive_log_score - fixed.predictive_log_score,
            adaptive.predictive_crps - fixed.predictive_crps,
        ];
        self.count += 1;
        for (index, value) in deltas.iter().enumerate() {
            self.sums[index] += value;
            self.squared_sums[index] += value * value;
        }
    }

    fn report(&self) -> Vec<PairedDifference> {
        let labels = [
            ("family_brier_score", "negative favours adaptive policy"),
            ("family_log_loss_nats", "negative favours adaptive policy"),
            (
                "heldout_predictive_log_score_nats",
                "positive favours adaptive policy",
            ),
            (
                "heldout_predictive_crps",
                "negative favours adaptive policy",
            ),
        ];
        labels
            .iter()
            .enumerate()
            .map(|(index, (metric, interpretation))| {
                let count = self.count.max(1) as f64;
                let mean = self.sums[index] / count;
                let variance = (self.squared_sums[index] / count - mean * mean).max(0.0);
                let standard_error = (variance / count).sqrt();
                PairedDifference {
                    metric: (*metric).into(),
                    adaptive_minus_fixed_mean: mean,
                    standard_error,
                    lower_95_percent: mean - 1.96 * standard_error,
                    upper_95_percent: mean + 1.96 * standard_error,
                    interpretation: (*interpretation).into(),
                }
            })
            .collect()
    }
}

fn derived_seed(root_seed: u64, family_index: usize, world_index: usize) -> u64 {
    root_seed
        ^ (family_index as u64).wrapping_mul(0xD6E8_FEB8_6659_FD93)
        ^ (world_index as u64).wrapping_mul(0xA5A3_564E_27F8_862D)
}

fn hex_digest(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn evaluate(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let first_index = candidate_index(fixture, &fixture.first_candidate_id)?;
    let fixed_second_index = candidate_index(fixture, &fixture.fixed_second_candidate_id)?;
    let prior = uniform_prior(fixture);
    let mut fixed_accumulator = Accumulator::new(fixture);
    let mut adaptive_accumulator = Accumulator::new(fixture);
    let mut paired_deltas = DeltaAccumulator::default();
    let mut world_digest = Sha256::new();

    for (family_index, family) in fixture.families.iter().enumerate() {
        for world_index in 0..fixture.worlds_per_family {
            let regime_index = world_index % fixture.parameter_regimes.len();
            let regime = fixture.parameter_regimes[regime_index];
            let true_class = family_index * fixture.parameter_regimes.len() + regime_index;
            let mut rng = SplitMix64::new(derived_seed(fixture.seed, family_index, world_index));

            let candidate_observations: Vec<f64> = (0..fixture.candidates.len())
                .map(|candidate_index| {
                    class_mean(fixture, true_class, candidate_index)
                        + fixture.candidates[candidate_index].noise_std * rng.normal()
                })
                .collect();
            let heldout_observation =
                family.base_heldout_mean * regime + fixture.heldout_noise_std * rng.normal();
            let posterior_draw_uniforms: Vec<f64> = (0..fixture.sbc_posterior_draws)
                .map(|_| rng.uniform())
                .collect();
            let tie_uniform = rng.uniform();

            let first_observation = candidate_observations[first_index];
            let posterior_after_first =
                posterior_update(fixture, &prior, &[first_index], &[first_observation], 1.0);
            let adaptive_second_index =
                choose_adaptive_second(fixture, &posterior_after_first, first_index);

            let fixed_selected = [first_index, fixed_second_index];
            let fixed_observations = [
                candidate_observations[first_index],
                candidate_observations[fixed_second_index],
            ];
            let fixed_posterior =
                posterior_update(fixture, &prior, &fixed_selected, &fixed_observations, 1.0);

            let adaptive_selected = [first_index, adaptive_second_index];
            let adaptive_observations = [
                candidate_observations[first_index],
                candidate_observations[adaptive_second_index],
            ];
            let adaptive_posterior = posterior_update(
                fixture,
                &prior,
                &adaptive_selected,
                &adaptive_observations,
                1.0,
            );

            let fixed_score = score_world(
                fixture,
                &fixed_posterior,
                &fixed_selected,
                &fixed_observations,
                heldout_observation,
                family_index,
                regime_index,
                &posterior_draw_uniforms,
                tie_uniform,
                fixed_second_index,
            );
            let adaptive_score = score_world(
                fixture,
                &adaptive_posterior,
                &adaptive_selected,
                &adaptive_observations,
                heldout_observation,
                family_index,
                regime_index,
                &posterior_draw_uniforms,
                tie_uniform,
                adaptive_second_index,
            );
            fixed_accumulator.record(&fixed_score, family_index);
            adaptive_accumulator.record(&adaptive_score, family_index);
            paired_deltas.record(&fixed_score, &adaptive_score);

            world_digest.update(family.family_id.as_bytes());
            world_digest.update((world_index as u64).to_le_bytes());
            world_digest.update((regime_index as u64).to_le_bytes());
            for observation in &candidate_observations {
                world_digest.update(observation.to_le_bytes());
            }
            world_digest.update(heldout_observation.to_le_bytes());
        }
    }

    let digest = world_digest.finalize();
    let worlds_per_regime = fixture.worlds_per_family / fixture.parameter_regimes.len();
    let total_worlds = (fixture.families.len() * fixture.worlds_per_family) as u64;
    Ok(Report {
        schema_version: FIXTURE_SCHEMA.into(),
        fixture_id: fixture.fixture_id.clone(),
        fixture_status: fixture.fixture_status.clone(),
        input_sha256,
        world_stream_sha256: hex_digest(&digest),
        root_seed: fixture.seed,
        family_count: fixture.families.len(),
        parameter_regime_count: fixture.parameter_regimes.len(),
        candidate_count: fixture.candidates.len(),
        worlds_per_family: fixture.worlds_per_family,
        worlds_per_parameter_regime_per_family: worlds_per_regime,
        total_paired_worlds: total_worlds,
        common_random_numbers: true,
        fixed_first_candidate_id: fixture.first_candidate_id.clone(),
        fixed_second_candidate_id: fixture.fixed_second_candidate_id.clone(),
        adaptive_policy: "first fixed upper-path observation; choose the second candidate by minimum seven-node posterior expected model entropy".into(),
        discrete_parameter_sbc_definition: "Randomized rank histogram for the three-point discrete conductance-scale regime posterior, with randomized tie handling and common posterior-sample uniforms across paired policies. This is discrete-regime SBC, not continuous-parameter SBC.".into(),
        ood_control_definition: "For each in-family world and each policy, shift both selected observations by +8 sensor standard deviations; report the detection rate of the closed-set minimum-worst-standardized-residual alarm. This is a deterministic residual stress control, not a realistic held-out hydraulic mechanism.".into(),
        claim_boundary: "Synthetic known-truth calibration only. Family response signatures and parameter regimes are analyst-authored normalized patterns, not calibrated soil physics. The fixed-versus-adaptive comparison uses common random numbers, but finite-world confidence intervals are normal paired standard-error intervals. No physical or external SOTA superiority claim is established.".into(),
        fixed_policy: fixed_accumulator.report(fixture, "fixed_upper_plus_early_transient"),
        adaptive_policy_report: adaptive_accumulator.report(fixture, "greedy_adaptive_model_eig"),
        paired_differences: paired_deltas.report(),
    })
}

fn run(input_path: &str, output_path: &str) -> Result<Report, String> {
    let input_bytes = fs::read(input_path)
        .map_err(|error| format!("failed to read calibration fixture: {error}"))?;
    let input_sha256 = hex_digest(&Sha256::digest(&input_bytes));
    let fixture: Fixture = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("invalid calibration fixture JSON: {error}"))?;
    let report = evaluate(&fixture, input_sha256)?;
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("failed to serialize calibration report: {error}"))?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| {
            format!("output must be a new file; refusing overwrite at {output_path}: {error}")
        })?;
    file.write_all(&output)
        .map_err(|error| format!("failed to write calibration report: {error}"))?;
    println!(
        "families={} regimes={} worlds={} fixed_family_ece={:.6} adaptive_family_ece={:.6} fixed_regime_sbc_bins={} adaptive_regime_sbc_bins={} paired_world_sha256={}",
        report.family_count,
        report.parameter_regime_count,
        report.total_paired_worlds,
        report.fixed_policy.family_probability_ece,
        report.adaptive_policy_report.family_probability_ece,
        report.fixed_policy.discrete_regime_sbc_rank_histogram.len(),
        report
            .adaptive_policy_report
            .discrete_regime_sbc_rank_histogram
            .len(),
        report.world_stream_sha256
    );
    Ok(report)
}

fn main() {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        eprintln!("usage: terra-boed-calibration INPUT.json OUTPUT.json");
        process::exit(2);
    }
    if let Err(error) = run(&arguments[1], &arguments[2]) {
        eprintln!("calibration error: {error}");
        process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny_fixture() -> Fixture {
        let response = [
            [1.0, 2.0, 1.0],
            [2.0, 1.0, 2.0],
            [1.0, 1.0, 2.0],
            [2.0, 2.0, 1.0],
            [1.0, 2.0, 2.0],
            [2.0, 1.0, 1.0],
            [1.2, 1.9, 0.8],
            [1.8, 0.9, 2.1],
            [0.9, 1.1, 2.0],
            [2.1, 1.9, 0.9],
        ];
        Fixture {
            schema_version: FIXTURE_SCHEMA.into(),
            fixture_status: FIXTURE_STATUS.into(),
            fixture_id: "test-fixture".into(),
            description: "test".into(),
            seed: 7,
            worlds_per_family: 1_200,
            parameter_regimes: vec![0.9, 1.0, 1.1],
            first_candidate_id: "first".into(),
            fixed_second_candidate_id: "second".into(),
            heldout_noise_std: 0.2,
            discrepancy_threshold_sd: 4.0,
            sbc_posterior_draws: 20,
            candidates: vec![
                Candidate {
                    candidate_id: "first".into(),
                    noise_std: 0.1,
                },
                Candidate {
                    candidate_id: "second".into(),
                    noise_std: 0.1,
                },
                Candidate {
                    candidate_id: "third".into(),
                    noise_std: 0.15,
                },
            ],
            families: response
                .iter()
                .enumerate()
                .map(|(index, means)| MechanismFamily {
                    family_id: format!("family_{index}"),
                    base_expected_observables: means.to_vec(),
                    base_heldout_mean: 1.0 + index as f64 * 0.05,
                })
                .collect(),
        }
    }

    #[test]
    fn splitmix_rng_is_repeatable() {
        let mut left = SplitMix64::new(123);
        let mut right = SplitMix64::new(123);
        for _ in 0..32 {
            assert_eq!(left.next_u64(), right.next_u64());
        }
    }

    #[test]
    fn posterior_is_normalized_and_unit_rescaling_invariant() {
        let fixture = tiny_fixture();
        let prior = uniform_prior(&fixture);
        let selected = [0, 1];
        let observations = [1.05, 1.9];
        let posterior = posterior_update(&fixture, &prior, &selected, &observations, 1.0);
        let scaled = posterior_update(&fixture, &prior, &selected, &observations, RESCALE_FACTOR);
        assert!((posterior.iter().sum::<f64>() - 1.0).abs() < 1.0e-12);
        assert!(
            posterior
                .iter()
                .zip(scaled.iter())
                .all(|(left, right)| (left - right).abs() < 1.0e-9)
        );
    }

    #[test]
    fn observation_order_does_not_change_posterior() {
        let fixture = tiny_fixture();
        let prior = uniform_prior(&fixture);
        let selected = [0, 1];
        let observations = [1.05, 1.9];
        let forward = posterior_update(&fixture, &prior, &selected, &observations, 1.0);
        let reverse = posterior_update(
            &fixture,
            &prior,
            &[selected[1], selected[0]],
            &[observations[1], observations[0]],
            1.0,
        );
        assert!(
            forward
                .iter()
                .zip(reverse.iter())
                .all(|(left, right)| (left - right).abs() < 1.0e-12)
        );
    }

    #[test]
    fn shifted_residual_control_detects_large_out_of_family_perturbation() {
        let fixture = tiny_fixture();
        let selected = [0, 1];
        let observations = [1.0, 2.0];
        let shifted = [
            observations[0] + OOD_SHIFT_STANDARD_DEVIATIONS * fixture.candidates[0].noise_std,
            observations[1] + OOD_SHIFT_STANDARD_DEVIATIONS * fixture.candidates[1].noise_std,
        ];
        assert!(
            discrepancy_score(&fixture, &selected, &shifted) > fixture.discrepancy_threshold_sd
        );
    }

    #[test]
    fn mixture_quantiles_and_crps_are_finite() {
        let probabilities = [0.3, 0.7];
        let means = [1.0, 2.0];
        let lower = mixture_quantile(0.1, &probabilities, &means, 0.2);
        let median = mixture_quantile(0.5, &probabilities, &means, 0.2);
        let upper = mixture_quantile(0.9, &probabilities, &means, 0.2);
        let crps = gaussian_mixture_crps(&probabilities, &means, 0.2, 1.6);
        assert!(lower < median && median < upper);
        assert!(crps.is_finite() && crps >= 0.0);
    }

    #[test]
    fn adaptive_second_candidate_is_feasible_and_deterministic() {
        let fixture = tiny_fixture();
        let prior = uniform_prior(&fixture);
        let posterior = posterior_update(&fixture, &prior, &[0], &[1.1], 1.0);
        let first = choose_adaptive_second(&fixture, &posterior, 0);
        let second = choose_adaptive_second(&fixture, &posterior, 0);
        assert_ne!(first, 0);
        assert_eq!(first, second);
    }
}
