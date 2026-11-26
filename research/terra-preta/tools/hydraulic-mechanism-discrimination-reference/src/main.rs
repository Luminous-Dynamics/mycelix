use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeSet,
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "terra-hydraulic-mechanism-discrimination-v0.1";
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
const PROBABILITY_TOLERANCE: f64 = 1.0e-12;
const MAX_CANDIDATES: usize = 10;
const MAX_MECHANISMS: usize = 8;
const MAX_SELECTED_OBSERVATIONS: usize = 4;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Mechanism {
    mechanism_id: String,
    description: String,
    prior_probability: f64,
    heldout_late_transient_flow: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Candidate {
    candidate_id: String,
    description: String,
    cost: u32,
    noise_std: f64,
    expected_observable_by_mechanism: Vec<f64>,
    d_observable_d_log_total_conductance: Vec<f64>,
    observable_unit: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DecisionAction {
    action_id: String,
    description: String,
    loss_by_mechanism: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    fixture_id: String,
    description: String,
    acquisition_budget: u32,
    mechanisms: Vec<Mechanism>,
    heldout_observation_noise_std: f64,
    closed_set_discrepancy_threshold_sd: f64,
    decision_actions: Vec<DecisionAction>,
    candidates: Vec<Candidate>,
}

#[derive(Clone, Debug, Serialize)]
struct FisherByMechanism {
    mechanism_id: String,
    log_conductance_fisher_information: f64,
}

#[derive(Clone, Debug, Serialize)]
struct DesignScore {
    candidate_ids: Vec<String>,
    total_cost: u32,
    expected_model_information_gain_nats: f64,
    expected_posterior_model_entropy_nats: f64,
    expected_model_classification_accuracy: f64,
    expected_model_brier_score: f64,
    expected_model_log_loss_nats: f64,
    expected_predictive_information_gain_nats: f64,
    expected_value_of_sample_information: f64,
    worst_case_log_conductance_fisher_information: f64,
    fisher_information_by_mechanism: Vec<FisherByMechanism>,
}

#[derive(Clone, Debug, Serialize)]
struct UniformCostMatchedBaseline {
    policy_name: String,
    policy_definition: String,
    exact_cost: u32,
    portfolio_count: usize,
    probability_per_portfolio: f64,
    mean_model_information_gain_nats: f64,
    p10_model_information_gain_nats: f64,
    median_model_information_gain_nats: f64,
    p90_model_information_gain_nats: f64,
    minimum_model_information_gain_nats: f64,
    maximum_model_information_gain_nats: f64,
    selected_design_improvement_over_uniform_mean_percent: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct ClosedSetDiscrepancyNegativeControl {
    threshold_standard_deviations: f64,
    in_family_best_fit_residual_standard_deviations: f64,
    in_family_abstained: bool,
    out_of_family_best_fit_residual_standard_deviations: f64,
    out_of_family_abstained: bool,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    fixture_id: String,
    input_sha256: String,
    mechanism_count: usize,
    candidate_count: usize,
    acquisition_budget: u32,
    prior_model_entropy_nats: f64,
    closed_set_discrepancy_threshold_sd: f64,
    closed_set_discrepancy_policy: String,
    closed_set_discrepancy_negative_control: ClosedSetDiscrepancyNegativeControl,
    model_discrimination_design: DesignScore,
    parameter_precision_design: DesignScore,
    predictive_design: DesignScore,
    decision_design: DesignScore,
    prior_minimum_expected_decision_loss: f64,
    sequential_policy_comparison: SequentialPolicyComparison,
    uniform_cost_matched_baseline: UniformCostMatchedBaseline,
    all_feasible_designs: Vec<DesignScore>,
    interpretation_boundary: String,
}

#[derive(Clone, Debug)]
struct ExpectedMetrics {
    prior_entropy: f64,
    expected_posterior_entropy: f64,
    expected_classification_accuracy: f64,
    expected_brier_score: f64,
    expected_log_loss: f64,
    expected_predictive_information_gain: f64,
    expected_value_of_sample_information: f64,
}

#[derive(Clone, Debug)]
struct EvaluatedDesign {
    summary: DesignScore,
    worst_case_fisher: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SequentialBranchShare {
    next_candidate_id: String,
    predictive_probability_mass: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SequentialObservationBranch {
    observed_value: f64,
    posterior_model_probabilities: Vec<f64>,
    next_candidate_id: Option<String>,
    predictive_probability_mass: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SequentialPolicyScore {
    policy_name: String,
    first_candidate_id: String,
    expected_cost: f64,
    expected_terminal_decision_loss: f64,
    expected_value_of_sample_information: f64,
    second_stage_policy: Vec<SequentialBranchShare>,
    stop_probability_mass: f64,
    first_observation_policy: Vec<SequentialObservationBranch>,
}

#[derive(Clone, Debug, Serialize)]
struct SequentialPolicyComparison {
    horizon_measurements: usize,
    acquisition_budget: u32,
    fixed_best_policy: SequentialPolicyScore,
    greedy_myopic_evsi_policy: SequentialPolicyScore,
    finite_horizon_adaptive_policy: SequentialPolicyScore,
    adaptive_gain_over_fixed_loss_units: f64,
    adaptive_gain_over_greedy_loss_units: f64,
    oracle_definition: String,
    claim_boundary: String,
}

#[derive(Clone, Debug)]
struct SequentialEvaluation {
    expected_terminal_decision_loss: f64,
    expected_cost: f64,
    second_stage_policy: Vec<SequentialBranchShare>,
    stop_probability_mass: f64,
    first_observation_policy: Vec<SequentialObservationBranch>,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.fixture_id.trim().is_empty()
        || fixture.description.trim().is_empty()
    {
        return Err(
            "fixture identity, schema version, description or synthetic-only status is invalid"
                .into(),
        );
    }
    if fixture.mechanisms.len() < 2 || fixture.mechanisms.len() > MAX_MECHANISMS {
        return Err(format!(
            "between two and {MAX_MECHANISMS} mechanisms are required"
        ));
    }
    if fixture.candidates.is_empty() || fixture.candidates.len() > MAX_CANDIDATES {
        return Err(format!(
            "between one and {MAX_CANDIDATES} candidate observations are required"
        ));
    }
    if fixture.acquisition_budget == 0 {
        return Err("acquisition budget must be positive".into());
    }
    if !fixture.heldout_observation_noise_std.is_finite()
        || fixture.heldout_observation_noise_std <= 0.0
    {
        return Err("held-out observation noise must be finite and positive".into());
    }
    if !fixture.closed_set_discrepancy_threshold_sd.is_finite()
        || fixture.closed_set_discrepancy_threshold_sd <= 0.0
    {
        return Err("closed-set discrepancy threshold must be finite and positive".into());
    }

    let mut mechanism_ids = BTreeSet::new();
    let mut prior_sum = 0.0;
    for mechanism in &fixture.mechanisms {
        if mechanism.mechanism_id.trim().is_empty()
            || mechanism.description.trim().is_empty()
            || !mechanism_ids.insert(mechanism.mechanism_id.as_str())
            || !mechanism.prior_probability.is_finite()
            || mechanism.prior_probability <= 0.0
            || !mechanism.heldout_late_transient_flow.is_finite()
        {
            return Err(
                "mechanism IDs, descriptions, priors and held-out predictions must be valid".into(),
            );
        }
        prior_sum += mechanism.prior_probability;
    }
    if !prior_sum.is_finite() || (prior_sum - 1.0).abs() > PROBABILITY_TOLERANCE {
        return Err("mechanism prior probabilities must sum to one".into());
    }

    if fixture.decision_actions.len() < 2 {
        return Err("at least two downstream decision actions are required to score EVSI".into());
    }
    let mut action_ids = BTreeSet::new();
    for action in &fixture.decision_actions {
        if action.action_id.trim().is_empty()
            || action.description.trim().is_empty()
            || !action_ids.insert(action.action_id.as_str())
            || action.loss_by_mechanism.len() != fixture.mechanisms.len()
            || action
                .loss_by_mechanism
                .iter()
                .any(|loss| !loss.is_finite() || *loss < 0.0)
        {
            return Err(
                "decision actions require unique IDs and one finite nonnegative loss per mechanism"
                    .into(),
            );
        }
    }

    let mut candidate_ids = BTreeSet::new();
    for candidate in &fixture.candidates {
        if candidate.candidate_id.trim().is_empty()
            || candidate.description.trim().is_empty()
            || candidate.observable_unit.trim().is_empty()
            || !candidate_ids.insert(candidate.candidate_id.as_str())
            || candidate.cost == 0
            || !candidate.noise_std.is_finite()
            || candidate.noise_std <= 0.0
        {
            return Err("candidate identity, cost, observable unit or noise is invalid".into());
        }
        if candidate.expected_observable_by_mechanism.len() != fixture.mechanisms.len()
            || candidate.d_observable_d_log_total_conductance.len() != fixture.mechanisms.len()
        {
            return Err(format!(
                "candidate {} must provide one prediction and one parameter derivative per mechanism",
                candidate.candidate_id
            ));
        }
        if candidate
            .expected_observable_by_mechanism
            .iter()
            .chain(candidate.d_observable_d_log_total_conductance.iter())
            .any(|value| !value.is_finite())
        {
            return Err(format!(
                "candidate {} contains a non-finite model output or derivative",
                candidate.candidate_id
            ));
        }
    }

    let mut acquisition_costs: Vec<u32> = fixture
        .candidates
        .iter()
        .map(|candidate| candidate.cost)
        .collect();
    acquisition_costs.sort_unstable();
    let mut cumulative_cost = 0u32;
    let mut maximum_observations = 0usize;
    for cost in acquisition_costs {
        let Some(next_cost) = cumulative_cost.checked_add(cost) else {
            return Err("candidate acquisition costs overflow the finite budget range".into());
        };
        if next_cost > fixture.acquisition_budget {
            break;
        }
        cumulative_cost = next_cost;
        maximum_observations += 1;
    }
    if maximum_observations > MAX_SELECTED_OBSERVATIONS {
        return Err("a feasible portfolio exceeds deterministic quadrature capacity".into());
    }
    let quadrature_weight_sum: f64 = GH_WEIGHTS.iter().sum();
    if (quadrature_weight_sum - 1.0).abs() > 1.0e-12 {
        return Err("internal Gauss-Hermite weights do not sum to one".into());
    }
    Ok(())
}

fn log_normal_density(value: f64, mean: f64, standard_deviation: f64) -> f64 {
    let standardized = (value - mean) / standard_deviation;
    -0.5 * standardized * standardized
        - standard_deviation.ln()
        - 0.5 * (2.0 * std::f64::consts::PI).ln()
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

fn entropy(probabilities: &[f64]) -> f64 {
    -probabilities
        .iter()
        .filter(|probability| **probability > 0.0)
        .map(|probability| probability * probability.ln())
        .sum::<f64>()
}

fn posterior_from_prior(
    fixture: &Fixture,
    prior: &[f64],
    selected: &[usize],
    observations: &[f64],
) -> Vec<f64> {
    let log_weights: Vec<f64> = fixture
        .mechanisms
        .iter()
        .enumerate()
        .map(|(mechanism_index, _)| {
            let log_likelihood = selected
                .iter()
                .enumerate()
                .map(|(observation_index, candidate_index)| {
                    let candidate = &fixture.candidates[*candidate_index];
                    log_normal_density(
                        observations[observation_index],
                        candidate.expected_observable_by_mechanism[mechanism_index],
                        candidate.noise_std,
                    )
                })
                .sum::<f64>();
            if prior[mechanism_index] > 0.0 {
                prior[mechanism_index].ln() + log_likelihood
            } else {
                f64::NEG_INFINITY
            }
        })
        .collect();
    let normalizer = log_sum_exp(&log_weights);
    log_weights
        .iter()
        .map(|value| (value - normalizer).exp())
        .collect()
}

fn posterior_probabilities(
    fixture: &Fixture,
    selected: &[usize],
    observations: &[f64],
) -> Vec<f64> {
    let prior: Vec<f64> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect();
    posterior_from_prior(fixture, &prior, selected, observations)
}

fn minimum_best_fit_standardized_residual(
    fixture: &Fixture,
    selected: &[usize],
    observations: &[f64],
) -> Result<f64, String> {
    if selected.is_empty() || selected.len() != observations.len() {
        return Err(
            "closed-set discrepancy check requires one observation per selected candidate".into(),
        );
    }
    if observations.iter().any(|value| !value.is_finite()) {
        return Err("closed-set discrepancy observations must be finite".into());
    }

    // Find the candidate mechanism that best explains the full observed vector
    // under the declared independent Gaussian sensor-noise model. The score is
    // the smallest, over candidate mechanisms, of that mechanism's largest
    // absolute standardized residual.
    let minimum_score = fixture
        .mechanisms
        .iter()
        .enumerate()
        .map(|(mechanism_index, _)| {
            selected
                .iter()
                .zip(observations.iter())
                .map(|(candidate_index, observed)| {
                    let candidate = &fixture.candidates[*candidate_index];
                    ((observed - candidate.expected_observable_by_mechanism[mechanism_index])
                        / candidate.noise_std)
                        .abs()
                })
                .fold(0.0_f64, f64::max)
        })
        .fold(f64::INFINITY, f64::min);

    if !minimum_score.is_finite() {
        return Err("closed-set discrepancy score is non-finite".into());
    }
    Ok(minimum_score)
}

fn should_abstain_closed_set(
    fixture: &Fixture,
    selected: &[usize],
    observations: &[f64],
) -> Result<bool, String> {
    Ok(
        minimum_best_fit_standardized_residual(fixture, selected, observations)?
            > fixture.closed_set_discrepancy_threshold_sd,
    )
}

fn posterior_predictive_mutual_information(fixture: &Fixture, posterior: &[f64]) -> f64 {
    let mut information = 0.0;
    for (true_index, mechanism) in fixture.mechanisms.iter().enumerate() {
        for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let observation = mechanism.heldout_late_transient_flow
                + fixture.heldout_observation_noise_std * node;
            let conditional_log_density = log_normal_density(
                observation,
                mechanism.heldout_late_transient_flow,
                fixture.heldout_observation_noise_std,
            );
            let mixture_terms: Vec<f64> = fixture
                .mechanisms
                .iter()
                .enumerate()
                .map(|(index, other)| {
                    posterior[index].ln()
                        + log_normal_density(
                            observation,
                            other.heldout_late_transient_flow,
                            fixture.heldout_observation_noise_std,
                        )
                })
                .filter(|value| value.is_finite())
                .collect();
            let mixture_log_density = log_sum_exp(&mixture_terms);
            information += posterior[true_index]
                * quadrature_weight
                * (conditional_log_density - mixture_log_density);
        }
    }
    information.max(0.0)
}

fn integrate_observation_grid<F>(
    fixture: &Fixture,
    selected: &[usize],
    true_mechanism: usize,
    depth: usize,
    observations: &mut Vec<f64>,
    weight: f64,
    callback: &mut F,
) where
    F: FnMut(&[f64], f64),
{
    if depth == selected.len() {
        callback(observations, weight);
        return;
    }

    let candidate = &fixture.candidates[selected[depth]];
    let mean = candidate.expected_observable_by_mechanism[true_mechanism];
    for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
        observations.push(mean + candidate.noise_std * node);
        integrate_observation_grid(
            fixture,
            selected,
            true_mechanism,
            depth + 1,
            observations,
            weight * quadrature_weight,
            callback,
        );
        observations.pop();
    }
}

fn minimum_expected_decision_loss(fixture: &Fixture, probabilities: &[f64]) -> f64 {
    fixture
        .decision_actions
        .iter()
        .map(|action| {
            action
                .loss_by_mechanism
                .iter()
                .zip(probabilities.iter())
                .map(|(loss, probability)| loss * probability)
                .sum::<f64>()
        })
        .fold(f64::INFINITY, f64::min)
}

fn decision_actions_are_equivalent(fixture: &Fixture) -> bool {
    let Some(first) = fixture.decision_actions.first() else {
        return false;
    };
    fixture
        .decision_actions
        .iter()
        .skip(1)
        .all(|action| action.loss_by_mechanism == first.loss_by_mechanism)
}

fn expected_metrics(fixture: &Fixture, selected: &[usize]) -> ExpectedMetrics {
    let prior: Vec<f64> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect();
    let prior_entropy = entropy(&prior);
    let prior_minimum_expected_decision_loss = minimum_expected_decision_loss(fixture, &prior);
    let sequential_policy_comparison = sequential_policy_comparison(fixture)?;
    let mut expected_posterior_minimum_expected_loss = 0.0;
    let mut expected_posterior_entropy = 0.0;
    let mut expected_accuracy = 0.0;
    let mut expected_brier = 0.0;
    let mut expected_log_loss = 0.0;
    let mut expected_predictive_information = 0.0;

    for (true_index, mechanism) in fixture.mechanisms.iter().enumerate() {
        let prior_mass = mechanism.prior_probability;
        let mut observations = Vec::with_capacity(selected.len());
        let mut accumulate = |observed: &[f64], quadrature_mass: f64| {
            let posterior = posterior_probabilities(fixture, selected, observed);
            let mass = prior_mass * quadrature_mass;

            expected_posterior_entropy += mass * entropy(&posterior);
            expected_posterior_minimum_expected_loss +=
                mass * minimum_expected_decision_loss(fixture, &posterior);
            let predicted_class = posterior
                .iter()
                .enumerate()
                .max_by(|left, right| left.1.total_cmp(right.1))
                .map(|(index, _)| index)
                .unwrap_or(0);
            if predicted_class == true_index {
                expected_accuracy += mass;
            }

            let brier = posterior
                .iter()
                .enumerate()
                .map(|(index, probability)| {
                    let target = if index == true_index { 1.0 } else { 0.0 };
                    (probability - target).powi(2)
                })
                .sum::<f64>();
            expected_brier += mass * brier;
            expected_log_loss += mass * -posterior[true_index].max(f64::MIN_POSITIVE).ln();
            expected_predictive_information +=
                mass * posterior_predictive_mutual_information(fixture, &posterior);
        };

        integrate_observation_grid(
            fixture,
            selected,
            true_index,
            0,
            &mut observations,
            1.0,
            &mut accumulate,
        );
    }

    ExpectedMetrics {
        prior_entropy,
        expected_posterior_entropy,
        expected_classification_accuracy: expected_accuracy,
        expected_brier_score: expected_brier,
        expected_log_loss,
        // I(Y_design; Y*) = I(M; Y*) - E[I(M; Y* | Y_design)].
        expected_predictive_information_gain: (posterior_predictive_mutual_information(
            fixture, &prior,
        ) - expected_predictive_information)
            .max(0.0),
        expected_value_of_sample_information: if decision_actions_are_equivalent(fixture) {
            // When every action has the same state-contingent loss vector,
            // observing data cannot change the available decision. This exact
            // invariant avoids reporting quadrature integration error as EVSI.
            0.0
        } else {
            (prior_minimum_expected_decision_loss - expected_posterior_minimum_expected_loss)
                .max(0.0)
        },

    }
}

fn candidate_indices_by_id(fixture: &Fixture) -> Vec<usize> {
    let mut indices: Vec<usize> = (0..fixture.candidates.len()).collect();
    indices.sort_by(|left, right| {
        fixture.candidates[*left]
            .candidate_id
            .cmp(&fixture.candidates[*right].candidate_id)
    });
    indices
}

fn expected_decision_loss_after_candidate(
    fixture: &Fixture,
    posterior: &[f64],
    candidate_index: usize,
) -> f64 {
    let candidate = &fixture.candidates[candidate_index];
    let mut expected_loss = 0.0;
    for (mechanism_index, mechanism_probability) in posterior.iter().enumerate() {
        for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let observation = candidate.expected_observable_by_mechanism[mechanism_index]
                + candidate.noise_std * node;
            let updated = posterior_from_prior(
                fixture,
                posterior,
                &[candidate_index],
                &[observation],
            );
            expected_loss += mechanism_probability
                * quadrature_weight
                * minimum_expected_decision_loss(fixture, &updated);
        }
    }
    expected_loss
}

fn best_second_stage(
    fixture: &Fixture,
    posterior: &[f64],
    first_candidate_index: usize,
    remaining_budget: u32,
) -> (Option<usize>, f64) {
    let current_loss = minimum_expected_decision_loss(fixture, posterior);
    let mut best_candidate: Option<usize> = None;
    let mut best_loss = f64::INFINITY;

    for candidate_index in candidate_indices_by_id(fixture) {
        if candidate_index == first_candidate_index
            || fixture.candidates[candidate_index].cost > remaining_budget
        {
            continue;
        }
        let expected_loss =
            expected_decision_loss_after_candidate(fixture, posterior, candidate_index);
        if expected_loss < best_loss - 1.0e-12 {
            best_candidate = Some(candidate_index);
            best_loss = expected_loss;
        }
    }

    if best_loss < current_loss - 1.0e-12 {
        (best_candidate, best_loss)
    } else {
        (None, current_loss)
    }
}

fn evaluate_first_candidate(
    fixture: &Fixture,
    first_candidate_index: usize,
) -> SequentialEvaluation {
    let prior: Vec<f64> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect();
    let first_candidate = &fixture.candidates[first_candidate_index];
    let remaining_budget = fixture.acquisition_budget.saturating_sub(first_candidate.cost);
    let mut expected_terminal_decision_loss = 0.0;
    let mut expected_second_stage_cost = 0.0;
    let mut stop_probability_mass = 0.0;
    let mut second_stage_policy: Vec<SequentialBranchShare> = Vec::new();
    let mut first_observation_policy: Vec<SequentialObservationBranch> = Vec::new();

    for (true_mechanism_index, mechanism) in fixture.mechanisms.iter().enumerate() {
        for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let observation = first_candidate.expected_observable_by_mechanism
                [true_mechanism_index]
                + first_candidate.noise_std * node;
            let posterior =
                posterior_from_prior(fixture, &prior, &[first_candidate_index], &[observation]);
            let (next_candidate, conditional_terminal_loss) = best_second_stage(
                fixture,
                &posterior,
                first_candidate_index,
                remaining_budget,
            );
            let joint_probability_mass = mechanism.prior_probability * quadrature_weight;
            expected_terminal_decision_loss +=
                joint_probability_mass * conditional_terminal_loss;

            let next_candidate_id = next_candidate
                .map(|index| fixture.candidates[index].candidate_id.clone());
            if let Some(next_index) = next_candidate {
                expected_second_stage_cost +=
                    joint_probability_mass * fixture.candidates[next_index].cost as f64;
                if let Some(existing) = second_stage_policy
                    .iter_mut()
                    .find(|branch| branch.next_candidate_id == fixture.candidates[next_index].candidate_id)
                {
                    existing.predictive_probability_mass += joint_probability_mass;
                } else {
                    second_stage_policy.push(SequentialBranchShare {
                        next_candidate_id: fixture.candidates[next_index].candidate_id.clone(),
                        predictive_probability_mass: joint_probability_mass,
                    });
                }
            } else {
                stop_probability_mass += joint_probability_mass;
            }

            if let Some(existing) = first_observation_policy
                .iter_mut()
                .find(|branch| branch.observed_value.to_bits() == observation.to_bits())
            {
                existing.predictive_probability_mass += joint_probability_mass;
            } else {
                first_observation_policy.push(SequentialObservationBranch {
                    observed_value: observation,
                    posterior_model_probabilities: posterior,
                    next_candidate_id,
                    predictive_probability_mass: joint_probability_mass,
                });
            }
        }
    }

    second_stage_policy.sort_by(|left, right| {
        left.next_candidate_id.cmp(&right.next_candidate_id)
    });
    first_observation_policy.sort_by(|left, right| {
        left.observed_value.total_cmp(&right.observed_value)
    });

    SequentialEvaluation {
        expected_terminal_decision_loss,
        expected_cost: first_candidate.cost as f64 + expected_second_stage_cost,
        second_stage_policy,
        stop_probability_mass,
        first_observation_policy,
    }
}

fn stop_sequential_policy(
    policy_name: &str,
    prior_loss: f64,
) -> SequentialPolicyScore {
    SequentialPolicyScore {
        policy_name: policy_name.into(),
        first_candidate_id: "stop".into(),
        expected_cost: 0.0,
        expected_terminal_decision_loss: prior_loss,
        expected_value_of_sample_information: 0.0,
        second_stage_policy: Vec::new(),
        stop_probability_mass: 1.0,
        first_observation_policy: Vec::new(),
    }
}

fn sequential_policy_score(
    fixture: &Fixture,
    policy_name: &str,
    first_candidate_index: usize,
    evaluation: SequentialEvaluation,
    prior_loss: f64,
) -> SequentialPolicyScore {
    SequentialPolicyScore {
        policy_name: policy_name.into(),
        first_candidate_id: fixture.candidates[first_candidate_index].candidate_id.clone(),
        expected_cost: evaluation.expected_cost,
        expected_terminal_decision_loss: evaluation.expected_terminal_decision_loss,
        expected_value_of_sample_information: (prior_loss
            - evaluation.expected_terminal_decision_loss)
            .max(0.0),
        second_stage_policy: evaluation.second_stage_policy,
        stop_probability_mass: evaluation.stop_probability_mass,
        first_observation_policy: evaluation.first_observation_policy,
    }
}

fn sequential_policy_comparison(
    fixture: &Fixture,
) -> Result<SequentialPolicyComparison, String> {
    if fixture.acquisition_budget != 2
        || fixture.candidates.iter().any(|candidate| candidate.cost != 1)
        || fixture.candidates.len() < 3
    {
        return Err(
            "sequential v0.1 oracle requires a two-measurement budget, unit-cost candidates and at least three candidates"
                .into(),
        );
    }

    let prior: Vec<f64> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect();
    let prior_loss = minimum_expected_decision_loss(fixture, &prior);
    let ordered_candidates = candidate_indices_by_id(fixture);

    let mut fixed_best: Option<(Vec<usize>, f64)> = None;
    for left in 0..ordered_candidates.len() {
        for right in (left + 1)..ordered_candidates.len() {
            let selected = vec![ordered_candidates[left], ordered_candidates[right]];
            let expected_loss = (prior_loss
                - expected_metrics(fixture, &selected).expected_value_of_sample_information)
                .max(0.0);
            let replace = fixed_best
                .as_ref()
                .map(|(current_indices, current_loss)| {
                    expected_loss < *current_loss - 1.0e-12
                        || ((expected_loss - *current_loss).abs() <= 1.0e-12
                            && candidate_ids(fixture, &selected)
                                < candidate_ids(fixture, current_indices))
                })
                .unwrap_or(true);
            if replace {
                fixed_best = Some((selected, expected_loss));
            }
        }
    }
    let (fixed_indices, fixed_loss) =
        fixed_best.ok_or_else(|| "no fixed two-measurement design is feasible".to_string())?;
    let fixed_ids = candidate_ids(fixture, &fixed_indices);
    let fixed_policy = SequentialPolicyScore {
        policy_name: "best_fixed_set_exact_budget".into(),
        first_candidate_id: fixed_ids[0].clone(),
        expected_cost: fixture.acquisition_budget as f64,
        expected_terminal_decision_loss: fixed_loss,
        expected_value_of_sample_information: (prior_loss - fixed_loss).max(0.0),
        second_stage_policy: vec![SequentialBranchShare {
            next_candidate_id: fixed_ids[1].clone(),
            predictive_probability_mass: 1.0,
        }],
        stop_probability_mass: 0.0,
        first_observation_policy: Vec::new(),
    };

    let mut greedy_first: Option<(usize, f64)> = None;
    for candidate_index in &ordered_candidates {
        let one_step_loss =
            expected_decision_loss_after_candidate(fixture, &prior, *candidate_index);
        let replace = greedy_first
            .as_ref()
            .map(|(current_index, current_loss)| {
                one_step_loss < *current_loss - 1.0e-12
                    || ((one_step_loss - *current_loss).abs() <= 1.0e-12
                        && fixture.candidates[*candidate_index].candidate_id
                            < fixture.candidates[*current_index].candidate_id)
            })
            .unwrap_or(true);
        if replace {
            greedy_first = Some((*candidate_index, one_step_loss));
        }
    }
    let greedy_policy = match greedy_first {
        Some((candidate_index, one_step_loss)) if one_step_loss < prior_loss - 1.0e-12 => {
            sequential_policy_score(
                fixture,
                "greedy_myopic_evsi_then_optimal_final_step",
                candidate_index,
                evaluate_first_candidate(fixture, candidate_index),
                prior_loss,
            )
        }
        _ => stop_sequential_policy("greedy_myopic_evsi_then_optimal_final_step", prior_loss),
    };

    let mut adaptive_best: Option<(usize, SequentialEvaluation)> = None;
    for candidate_index in ordered_candidates {
        let evaluation = evaluate_first_candidate(fixture, candidate_index);
        if evaluation.expected_terminal_decision_loss >= prior_loss - 1.0e-12 {
            continue;
        }
        let replace = adaptive_best
            .as_ref()
            .map(|(current_index, current_evaluation)| {
                evaluation.expected_terminal_decision_loss
                    < current_evaluation.expected_terminal_decision_loss - 1.0e-12
                    || ((evaluation.expected_terminal_decision_loss
                        - current_evaluation.expected_terminal_decision_loss)
                        .abs()
                        <= 1.0e-12
                        && fixture.candidates[candidate_index].candidate_id
                            < fixture.candidates[*current_index].candidate_id)
            })
            .unwrap_or(true);
        if replace {
            adaptive_best = Some((candidate_index, evaluation));
        }
    }
    let adaptive_policy = match adaptive_best {
        Some((candidate_index, evaluation)) => sequential_policy_score(
            fixture,
            "finite_horizon_two_step_adaptive_evsi_oracle",
            candidate_index,
            evaluation,
            prior_loss,
        ),
        None => stop_sequential_policy(
            "finite_horizon_two_step_adaptive_evsi_oracle",
            prior_loss,
        ),
    };

    Ok(SequentialPolicyComparison {
        horizon_measurements: 2,
        acquisition_budget: fixture.acquisition_budget,
        adaptive_gain_over_fixed_loss_units: (fixed_policy.expected_terminal_decision_loss
            - adaptive_policy.expected_terminal_decision_loss)
            .max(0.0),
        adaptive_gain_over_greedy_loss_units: (greedy_policy.expected_terminal_decision_loss
            - adaptive_policy.expected_terminal_decision_loss)
            .max(0.0),
        fixed_best_policy: fixed_policy,
        greedy_myopic_evsi_policy: greedy_policy,
        finite_horizon_adaptive_policy: adaptive_policy,
        oracle_definition: "Two-stage Bellman optimization over every first candidate and every quadrature-supported first observation. At each observation, the final-stage action is the remaining affordable candidate with minimum posterior expected terminal decision loss, or stop. This is exact for the declared finite seven-node Gauss-Hermite quadrature approximation and the two-step policy class; it is not an exact integral over continuous Gaussian observations.".into(),
        claim_boundary: "Synthetic known-truth reference only. Adaptive gains are fixture-specific and quadrature-dependent. No real-soil calibration, physical performance or superiority over published continuous-space, SBI-BOED or robust-BED methods is established.".into(),
    })
}

fn fisher_information_by_mechanism(
    fixture: &Fixture,
    selected: &[usize],
) -> Vec<FisherByMechanism> {
    fixture
        .mechanisms
        .iter()
        .enumerate()
        .map(|(mechanism_index, mechanism)| {
            let information = selected
                .iter()
                .map(|candidate_index| {
                    let candidate = &fixture.candidates[*candidate_index];
                    let derivative =
                        candidate.d_observable_d_log_total_conductance[mechanism_index];
                    derivative.powi(2) / candidate.noise_std.powi(2)
                })
                .sum();
            FisherByMechanism {
                mechanism_id: mechanism.mechanism_id.clone(),
                log_conductance_fisher_information: information,
            }
        })
        .collect()
}

fn candidate_ids(fixture: &Fixture, selected: &[usize]) -> Vec<String> {
    let mut ids: Vec<String> = selected
        .iter()
        .map(|index| fixture.candidates[*index].candidate_id.clone())
        .collect();
    ids.sort();
    ids
}

fn evaluate_design(fixture: &Fixture, selected: Vec<usize>, cost: u32) -> EvaluatedDesign {
    let metrics = expected_metrics(fixture, &selected);
    let fisher = fisher_information_by_mechanism(fixture, &selected);
    let worst_case_fisher = fisher
        .iter()
        .map(|entry| entry.log_conductance_fisher_information)
        .fold(f64::INFINITY, f64::min);

    EvaluatedDesign {
        summary: DesignScore {
            candidate_ids: candidate_ids(fixture, &selected),
            total_cost: cost,
            expected_model_information_gain_nats: (metrics.prior_entropy
                - metrics.expected_posterior_entropy)
                .max(0.0),
            expected_posterior_model_entropy_nats: metrics.expected_posterior_entropy,
            expected_model_classification_accuracy: metrics.expected_classification_accuracy,
            expected_model_brier_score: metrics.expected_brier_score,
            expected_model_log_loss_nats: metrics.expected_log_loss,
            expected_predictive_information_gain_nats: metrics.expected_predictive_information_gain,
            expected_value_of_sample_information: metrics.expected_value_of_sample_information,
            worst_case_log_conductance_fisher_information: worst_case_fisher,
            fisher_information_by_mechanism: fisher,
        },
        worst_case_fisher,
    }
}

fn enumerate_designs(fixture: &Fixture) -> Vec<EvaluatedDesign> {
    let mut designs = Vec::new();
    let combination_count = 1u32 << fixture.candidates.len();

    for mask in 0..combination_count {
        let mut selected = Vec::new();
        let mut cost = 0u32;
        for (index, candidate) in fixture.candidates.iter().enumerate() {
            if mask & (1u32 << index) != 0 {
                cost = cost.saturating_add(candidate.cost);
                selected.push(index);
            }
        }
        if cost <= fixture.acquisition_budget {
            designs.push(evaluate_design(fixture, selected, cost));
        }
    }
    designs
}

fn choose_design<F>(designs: &[EvaluatedDesign], objective: F) -> Result<DesignScore, String>
where
    F: Fn(&EvaluatedDesign) -> f64,
{
    designs
        .iter()
        .max_by(|left, right| {
            objective(left)
                .total_cmp(&objective(right))
                .then_with(|| right.summary.total_cost.cmp(&left.summary.total_cost))
                .then_with(|| right.summary.candidate_ids.cmp(&left.summary.candidate_ids))
        })
        .map(|design| design.summary.clone())
        .ok_or_else(|| "no feasible experimental designs were enumerated".into())
}

fn interpolated_quantile(sorted_values: &[f64], probability: f64) -> Result<f64, String> {
    if sorted_values.is_empty()
        || !probability.is_finite()
        || !(0.0..=1.0).contains(&probability)
        || sorted_values.iter().any(|value| !value.is_finite())
    {
        return Err("quantile requires finite values and a probability in [0, 1]".into());
    }
    let position = probability * (sorted_values.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    Ok(sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction)
}

fn uniform_cost_matched_baseline(
    designs: &[EvaluatedDesign],
    selected: &DesignScore,
) -> Result<UniformCostMatchedBaseline, String> {
    let mut matched: Vec<f64> = designs
        .iter()
        .filter(|design| design.summary.total_cost == selected.total_cost)
        .map(|design| design.summary.expected_model_information_gain_nats)
        .collect();
    if matched.is_empty() {
        return Err("no portfolios match the selected design's exact acquisition cost".into());
    }
    matched.sort_by(f64::total_cmp);
    let mean = matched.iter().sum::<f64>() / matched.len() as f64;
    let selected_gain = selected.expected_model_information_gain_nats;
    let improvement = if mean.abs() > f64::EPSILON {
        Some((selected_gain - mean) / mean.abs() * 100.0)
    } else {
        None
    };

    Ok(UniformCostMatchedBaseline {
        policy_name: "uniform_probability_over_exact_cost_feasible_portfolios".into(),
        policy_definition: "Every distinct feasible candidate subset with exactly the model-discrimination design's total cost receives equal probability. This is a finite portfolio-selection baseline, not a spatial sensor-spacing baseline.".into(),
        exact_cost: selected.total_cost,
        portfolio_count: matched.len(),
        probability_per_portfolio: 1.0 / matched.len() as f64,
        mean_model_information_gain_nats: mean,
        p10_model_information_gain_nats: interpolated_quantile(&matched, 0.10)?,
        median_model_information_gain_nats: interpolated_quantile(&matched, 0.50)?,
        p90_model_information_gain_nats: interpolated_quantile(&matched, 0.90)?,
        minimum_model_information_gain_nats: matched[0],
        maximum_model_information_gain_nats: *matched.last().expect("nonempty exact-cost portfolios"),
        selected_design_improvement_over_uniform_mean_percent: improvement,
    })
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let designs = enumerate_designs(fixture);
    let model_design = choose_design(&designs, |design| {
        design.summary.expected_model_information_gain_nats
    })?;
    let precision_design = choose_design(&designs, |design| design.worst_case_fisher)?;
    let predictive_design = choose_design(&designs, |design| {
        design.summary.expected_predictive_information_gain_nats
    })?;
    let decision_design = choose_design(&designs, |design| {
        design.summary.expected_value_of_sample_information
    })?;
    let prior: Vec<f64> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect();
    let prior_minimum_expected_decision_loss = minimum_expected_decision_loss(fixture, &prior);
    // The CLI emits both sides of the discrepancy falsification control:
    // a mildly noisy observation generated by one declared mechanism and an
    // observation vector shifted ten sensor standard deviations away.
    let discrepancy_selected: Vec<usize> = model_design
        .candidate_ids
        .iter()
        .map(|candidate_id| {
            fixture
                .candidates
                .iter()
                .position(|candidate| &candidate.candidate_id == candidate_id)
                .ok_or_else(|| format!("selected candidate {candidate_id} disappeared"))
        })
        .collect::<Result<_, _>>()?;
    let in_family_observations: Vec<f64> = discrepancy_selected
        .iter()
        .map(|index| {
            let candidate = &fixture.candidates[*index];
            candidate.expected_observable_by_mechanism[0] + 0.5 * candidate.noise_std
        })
        .collect();
    let out_of_family_observations: Vec<f64> = discrepancy_selected
        .iter()
        .map(|index| {
            let candidate = &fixture.candidates[*index];
            candidate.expected_observable_by_mechanism[0] + 10.0 * candidate.noise_std
        })
        .collect();
    let discrepancy_negative_control = ClosedSetDiscrepancyNegativeControl {
        threshold_standard_deviations: fixture.closed_set_discrepancy_threshold_sd,
        in_family_best_fit_residual_standard_deviations: minimum_best_fit_standardized_residual(
            fixture,
            &discrepancy_selected,
            &in_family_observations,
        )?,
        in_family_abstained: should_abstain_closed_set(
            fixture,
            &discrepancy_selected,
            &in_family_observations,
        )?,
        out_of_family_best_fit_residual_standard_deviations:
            minimum_best_fit_standardized_residual(
                fixture,
                &discrepancy_selected,
                &out_of_family_observations,
            )?,
        out_of_family_abstained: should_abstain_closed_set(
            fixture,
            &discrepancy_selected,
            &out_of_family_observations,
        )?,
    };

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        fixture_id: fixture.fixture_id.clone(),
        input_sha256,
        mechanism_count: fixture.mechanisms.len(),
        candidate_count: fixture.candidates.len(),
        acquisition_budget: fixture.acquisition_budget,
        prior_model_entropy_nats: entropy(&prior),
        closed_set_discrepancy_threshold_sd: fixture.closed_set_discrepancy_threshold_sd,
        closed_set_discrepancy_policy: "Abstain rather than force a closed-set model selection when the smallest-over-models maximum absolute standardized residual exceeds the preregistered threshold. Assumes the declared independent Gaussian candidate-observation noise and is a discrepancy alarm, not a posterior probability that all models are false.".into(),
        closed_set_discrepancy_negative_control: discrepancy_negative_control,
        uniform_cost_matched_baseline: uniform_cost_matched_baseline(&designs, &model_design)?,
        model_discrimination_design: model_design,
        parameter_precision_design: precision_design,
        predictive_design,
        decision_design,
        prior_minimum_expected_decision_loss,
        sequential_policy_comparison,
        all_feasible_designs: designs
            .into_iter()
            .map(|design| design.summary)
            .collect(),
        interpretation_boundary: "Synthetic known-truth reference only. Expected model information gain and expected predictive information gain are computed by deterministic seven-point product Gauss-Hermite quadrature over independent Gaussian observation noise; the design oracle exhaustively enumerates the declared finite candidate subsets. The scalar Fisher comparator concerns log total conductance conditional on each mechanism. This does not establish simulator calibration, posterior calibration for real soil, completeness of the mechanism set, field validity or superiority over published BOED methods.".into(),
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
                .unwrap_or("hydraulic-mechanism-discrimination-reference")
        ));
    }
    let bytes = fs::read(&arguments[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let fixture: Fixture =
        serde_json::from_slice(&bytes).map_err(|error| format!("invalid fixture: {error}"))?;
    let report = analyze(&fixture, input_sha256)?;
    let encoded = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot encode report: {error}"))?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&arguments[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output
        .write_all(&encoded)
        .map_err(|error| format!("cannot write output: {error}"))?;
    output
        .write_all(b"\n")
        .map_err(|error| format!("cannot finish output: {error}"))?;
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
        serde_json::from_str(include_str!("../fixtures/matched-steady-transient.json")).unwrap()
    }

    #[test]
    fn all_mechanisms_share_the_same_steady_bulk_response() {
        let f = fixture();
        let bulk = f
            .candidates
            .iter()
            .find(|candidate| candidate.candidate_id == "bulk_steady_flow")
            .unwrap();
        assert!(
            bulk.expected_observable_by_mechanism
                .iter()
                .all(|value| (*value - 2.0).abs() < 1.0e-12)
        );
    }

    #[test]
    fn model_discrimination_selects_path_and_transient_observations() {
        let report = analyze(&fixture(), "a".repeat(64)).unwrap();
        assert_eq!(
            report.model_discrimination_design.candidate_ids,
            vec!["early_transient_flow", "upper_path_flow"]
        );
    }

    #[test]
    fn parameter_precision_can_prefer_a_non_discriminating_bulk_measurement() {
        let report = analyze(&fixture(), "b".repeat(64)).unwrap();
        assert_eq!(
            report.parameter_precision_design.candidate_ids,
            vec!["bulk_steady_flow", "upper_path_flow"]
        );
        assert!(
            report.model_discrimination_design.candidate_ids
                != report.parameter_precision_design.candidate_ids
        );
        assert_eq!(
            report.predictive_design.candidate_ids,
            vec!["early_transient_flow", "upper_path_flow"]
        );
        assert!(
            report
                .predictive_design
                .expected_predictive_information_gain_nats
                > 0.0
        );
    }

    #[test]
    fn observationally_equivalent_mechanisms_receive_zero_model_information() {
        let mut f = fixture();
        for candidate in &mut f.candidates {
            candidate.expected_observable_by_mechanism.fill(2.0);
        }
        for mechanism in &mut f.mechanisms {
            mechanism.heldout_late_transient_flow = 2.0;
        }
        let report = analyze(&f, "c".repeat(64)).unwrap();
        assert!(
            report
                .model_discrimination_design
                .expected_model_information_gain_nats
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn exact_cost_uniform_baseline_is_normalized_and_ordered() {
        let report = analyze(&fixture(), "d".repeat(64)).unwrap();
        let baseline = &report.uniform_cost_matched_baseline;
        assert_eq!(
            baseline.exact_cost,
            report.model_discrimination_design.total_cost
        );
        assert!(baseline.portfolio_count > 1);
        assert!(
            (baseline.probability_per_portfolio * baseline.portfolio_count as f64 - 1.0).abs()
                < 1.0e-12
        );
        assert!(
            baseline.p10_model_information_gain_nats <= baseline.median_model_information_gain_nats
        );
        assert!(
            baseline.median_model_information_gain_nats <= baseline.p90_model_information_gain_nats
        );
    }

    #[test]
    fn candidate_order_does_not_change_selected_physical_observations() {
        let mut f = fixture();
        let original = analyze(&f, "e".repeat(64)).unwrap();
        f.candidates.reverse();
        let permuted = analyze(&f, "e".repeat(64)).unwrap();
        assert_eq!(
            original.model_discrimination_design.candidate_ids,
            permuted.model_discrimination_design.candidate_ids
        );
        assert!(
            (original
                .model_discrimination_design
                .expected_model_information_gain_nats
                - permuted
                    .model_discrimination_design
                    .expected_model_information_gain_nats)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn gaussian_quadrature_rule_is_normalized_and_symmetric() {
        assert!((GH_WEIGHTS.iter().sum::<f64>() - 1.0).abs() < 1.0e-12);
        let first_moment = GH_NODES
            .iter()
            .zip(GH_WEIGHTS.iter())
            .map(|(node, weight)| node * weight)
            .sum::<f64>();
        let second_moment = GH_NODES
            .iter()
            .zip(GH_WEIGHTS.iter())
            .map(|(node, weight)| node * node * weight)
            .sum::<f64>();
        assert!(first_moment.abs() < 1.0e-12);
        assert!((second_moment - 1.0).abs() < 1.0e-12);
    }
    #[test]
    fn observation_unit_rescaling_preserves_information_and_design_rankings() {
        let mut scaled_fixture = fixture();
        let original = analyze(&scaled_fixture, "f".repeat(64)).unwrap();
        let unit_scale = 1.0e6;

        for candidate in &mut scaled_fixture.candidates {
            candidate.noise_std *= unit_scale;
            for value in &mut candidate.expected_observable_by_mechanism {
                *value *= unit_scale;
            }
            for derivative in &mut candidate.d_observable_d_log_total_conductance {
                *derivative *= unit_scale;
            }
        }
        scaled_fixture.heldout_observation_noise_std *= unit_scale;
        for mechanism in &mut scaled_fixture.mechanisms {
            mechanism.heldout_late_transient_flow *= unit_scale;
        }

        let scaled = analyze(&scaled_fixture, "f".repeat(64)).unwrap();
        assert_eq!(
            original.model_discrimination_design.candidate_ids,
            scaled.model_discrimination_design.candidate_ids
        );
        assert_eq!(
            original.parameter_precision_design.candidate_ids,
            scaled.parameter_precision_design.candidate_ids
        );
        assert_eq!(
            original.predictive_design.candidate_ids,
            scaled.predictive_design.candidate_ids
        );
        assert!(
            (original
                .model_discrimination_design
                .expected_model_information_gain_nats
                - scaled
                    .model_discrimination_design
                    .expected_model_information_gain_nats)
                .abs()
                < 1.0e-12
        );
        assert!(
            (original
                .parameter_precision_design
                .worst_case_log_conductance_fisher_information
                - scaled
                    .parameter_precision_design
                    .worst_case_log_conductance_fisher_information)
                .abs()
                < 1.0e-9
        );
    }

    #[test]
    fn overlarge_feasible_portfolio_is_rejected_before_quadrature() {
        let mut f = fixture();
        let mut fifth_candidate = f.candidates[0].clone();
        fifth_candidate.candidate_id = "extra_bulk_replicate".into();
        f.candidates.push(fifth_candidate);
        f.acquisition_budget = 5;
        assert!(validate(&f).is_err());
    }
    #[test]
    fn closed_set_discrepancy_detector_accepts_in_family_and_abstains_out_of_family() {
        let f = fixture();
        let selected = [
            f.candidates
                .iter()
                .position(|candidate| candidate.candidate_id == "early_transient_flow")
                .unwrap(),
            f.candidates
                .iter()
                .position(|candidate| candidate.candidate_id == "upper_path_flow")
                .unwrap(),
        ];
        let in_family = [
            f.candidates[selected[0]].expected_observable_by_mechanism[0]
                + 0.5 * f.candidates[selected[0]].noise_std,
            f.candidates[selected[1]].expected_observable_by_mechanism[0]
                - 0.5 * f.candidates[selected[1]].noise_std,
        ];
        assert!(!should_abstain_closed_set(&f, &selected, &in_family).unwrap());

        let out_of_family = [
            f.candidates[selected[0]].expected_observable_by_mechanism[0]
                + 10.0 * f.candidates[selected[0]].noise_std,
            f.candidates[selected[1]].expected_observable_by_mechanism[0]
                + 10.0 * f.candidates[selected[1]].noise_std,
        ];
        assert!(should_abstain_closed_set(&f, &selected, &out_of_family).unwrap());
        assert!(
            minimum_best_fit_standardized_residual(&f, &selected, &out_of_family).unwrap()
                > f.closed_set_discrepancy_threshold_sd
        );
    }

    #[test]
    fn closed_set_discrepancy_detector_rejects_malformed_observation_vectors() {
        let f = fixture();
        assert!(minimum_best_fit_standardized_residual(&f, &[0, 1], &[2.0]).is_err());
        assert!(minimum_best_fit_standardized_residual(&f, &[0], &[f64::NAN]).is_err());
        assert!(minimum_best_fit_standardized_residual(&f, &[], &[]).is_err());
    }
    #[test]
    fn decision_evsi_is_nonnegative_and_positive_for_informative_designs() {
        let f = fixture();
        let report = analyze(&f, "7".repeat(64)).unwrap();
        assert!(report.decision_design.expected_value_of_sample_information > 0.0);
        assert!(
            report.decision_design.expected_value_of_sample_information
                >= report
                    .all_feasible_designs
                    .iter()
                    .map(|design| design.expected_value_of_sample_information)
                    .fold(0.0_f64, f64::max)
                    - 1.0e-12
        );
        let no_measurements = report
            .all_feasible_designs
            .iter()
            .find(|design| design.candidate_ids.is_empty())
            .unwrap();
        assert!(no_measurements.expected_value_of_sample_information.abs() < 1.0e-12);
    }

    #[test]
    fn finite_horizon_adaptive_policy_beats_every_fixed_pair_at_equal_cost() {
        let comparison = sequential_policy_comparison(&fixture()).unwrap();
        let adaptive = &comparison.finite_horizon_adaptive_policy;
        let fixed = &comparison.fixed_best_policy;

        assert_eq!(adaptive.first_candidate_id, "upper_path_flow");
        assert!(
            adaptive.expected_terminal_decision_loss
                < fixed.expected_terminal_decision_loss - 1.0e-3
        );
        assert!(comparison.adaptive_gain_over_fixed_loss_units > 1.0e-3);
        assert!(adaptive.expected_cost <= comparison.acquisition_budget as f64 + 1.0e-12);
        assert!(
            adaptive
                .first_observation_policy
                .iter()
                .map(|branch| branch.predictive_probability_mass)
                .sum::<f64>()
                .sub(1.0)
                .abs()
                < 1.0e-12
        );
        assert!(
            adaptive
                .first_observation_policy
                .iter()
                .all(|branch| {
                    (branch.posterior_model_probabilities.iter().sum::<f64>() - 1.0).abs()
                        < 1.0e-12
                })
        );
        assert!(
            adaptive.second_stage_policy.iter().any(|branch| {
                branch.next_candidate_id == "early_transient_flow"
                    && branch.predictive_probability_mass > 0.9
            })
        );
        assert!(
            adaptive.second_stage_policy.iter().any(|branch| {
                branch.next_candidate_id == "lower_path_flow"
                    && branch.predictive_probability_mass > 0.0
            })
        );
    }

    #[test]
    fn adaptive_policy_stops_when_all_mechanisms_are_observationally_equivalent() {
        let mut f = fixture();
        for candidate in &mut f.candidates {
            candidate.expected_observable_by_mechanism.fill(2.0);
        }
        for mechanism in &mut f.mechanisms {
            mechanism.heldout_late_transient_flow = 2.0;
        }
        let comparison = sequential_policy_comparison(&f).unwrap();
        assert_eq!(
            comparison.finite_horizon_adaptive_policy.first_candidate_id,
            "stop"
        );
        assert!(
            comparison
                .finite_horizon_adaptive_policy
                .expected_value_of_sample_information
                .abs()
                < 1.0e-12
        );
        assert!(
            comparison.adaptive_gain_over_fixed_loss_units.abs() < 1.0e-12
        );
    }

    #[test]
    fn adaptive_policy_is_invariant_to_candidate_input_order() {
        let original = sequential_policy_comparison(&fixture()).unwrap();
        let mut reordered_fixture = fixture();
        reordered_fixture.candidates.reverse();
        let reordered = sequential_policy_comparison(&reordered_fixture).unwrap();

        assert_eq!(
            original.finite_horizon_adaptive_policy.first_candidate_id,
            reordered.finite_horizon_adaptive_policy.first_candidate_id
        );
        assert_eq!(
            original
                .finite_horizon_adaptive_policy
                .second_stage_policy
                .iter()
                .map(|branch| branch.next_candidate_id.clone())
                .collect::<Vec<_>>(),
            reordered
                .finite_horizon_adaptive_policy
                .second_stage_policy
                .iter()
                .map(|branch| branch.next_candidate_id.clone())
                .collect::<Vec<_>>()
        );
        assert!(
            (original.finite_horizon_adaptive_policy.expected_terminal_decision_loss
                - reordered
                    .finite_horizon_adaptive_policy
                    .expected_terminal_decision_loss)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn identical_action_losses_have_zero_value_of_sample_information() {
        let mut f = fixture();
        let common_losses = vec![1.0, 3.0, 2.0];
        for action in &mut f.decision_actions {
            action.loss_by_mechanism = common_losses.clone();
        }
        let report = analyze(&f, "8".repeat(64)).unwrap();
        for design in &report.all_feasible_designs {
            assert!(design.expected_value_of_sample_information.abs() < 1.0e-12);
        }
    }
}
