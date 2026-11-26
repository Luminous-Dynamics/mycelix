use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const SCHEMA: &str = "terra-action-conditioned-boed-v0.1";
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

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Waveform {
    ConstantHeadStep,
    SquareWave,
    Sinusoidal,
    DualPulseTracer,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Observable {
    OutletPressure,
    ActuatorDisplacement,
    TotalFlow,
    TracerFirstMoment,
    LateTracerFraction,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Mechanism {
    mechanism_id: String,
    prior_probability: f64,
    conductance: f64,
    storage_time_constant_s: f64,
    preferential_flow_fraction: f64,
    immobile_fraction: f64,
    dead_end_storage_fraction: f64,
    non_darcy_beta: f64,
    effective_porosity: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Protocol {
    protocol_id: String,
    waveform: Waveform,
    observable: Observable,
    pressure_amplitude: f64,
    frequency_hz: f64,
    duration_s: f64,
    injected_volume: f64,
    tracer_mass: f64,
    actuator_effort: f64,
    sample_damage_index: f64,
    acquisition_cost: u32,
    noise_std: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SafetyLimits {
    maximum_pressure_amplitude: f64,
    maximum_injected_volume: f64,
    maximum_duration_s: f64,
    maximum_tracer_mass: f64,
    maximum_actuator_effort: f64,
    maximum_sample_damage_index: f64,
    acquisition_budget: u32,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    fixture_id: String,
    description: String,
    discrepancy_threshold_sd: f64,
    safety_limits: SafetyLimits,
    mechanisms: Vec<Mechanism>,
    protocols: Vec<Protocol>,
}

#[derive(Clone, Debug, Serialize)]
struct ExcludedProtocol {
    protocol_id: String,
    reasons: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct ProtocolReceipt {
    protocol_id: String,
    waveform: Waveform,
    observable: Observable,
    pressure_amplitude: f64,
    frequency_hz: f64,
    duration_s: f64,
    injected_volume: f64,
    tracer_mass: f64,
    actuator_effort: f64,
    sample_damage_index: f64,
    acquisition_cost: u32,
    noise_std: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ObservationBranch {
    observed_value: f64,
    posterior_mechanism_probabilities: Vec<f64>,
    next_protocol_id: Option<String>,
    discrepancy_abstained: bool,
    predictive_probability_mass: f64,
}

#[derive(Clone, Debug, Serialize)]
struct PolicyReport {
    policy_id: String,
    first_protocol_id: String,
    second_protocol_shares: Vec<(String, f64)>,
    expected_terminal_model_entropy_nats: f64,
    expected_model_information_gain_nats: f64,
    expected_acquisition_cost: f64,
    stop_probability_mass: f64,
    discrepancy_abstention_probability_mass: f64,
    branches: Vec<ObservationBranch>,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    schema_version: String,
    fixture_id: String,
    fixture_status: String,
    input_sha256: String,
    feasible_protocols: Vec<ProtocolReceipt>,
    excluded_protocols: Vec<ExcludedProtocol>,
    hard_constraint_violations_in_selected_policy: usize,
    discrepancy_threshold_sd: f64,
    quadrature_nodes: usize,
    acquisition_budget: u32,
    best_fixed_policy: PolicyReport,
    sequential_adaptive_policy: PolicyReport,
    adaptive_terminal_entropy_reduction_vs_fixed_nats: f64,
    policy_semantics: String,
    claim_boundary: String,
}

#[derive(Clone, Debug)]
struct ProtocolObservation {
    protocol_index: usize,
    observation: f64,
}

#[derive(Clone, Debug)]
struct FirstStageEvaluation {
    terminal_entropy: f64,
    expected_cost: f64,
    stop_probability_mass: f64,
    discrepancy_abstention_probability_mass: f64,
    second_protocol_shares: Vec<(String, f64)>,
    branches: Vec<ObservationBranch>,
}

#[derive(Clone, Copy, Debug)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
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
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != SCHEMA || fixture.fixture_status != STATUS {
        return Err("unsupported schema or non-synthetic fixture status".into());
    }
    if fixture.fixture_id.trim().is_empty() || fixture.description.trim().is_empty() {
        return Err("fixture identity and description must be non-empty".into());
    }
    if fixture.mechanisms.len() < 3 || fixture.protocols.len() < 4 {
        return Err("at least three mechanisms and four protocols are required".into());
    }
    if fixture.discrepancy_threshold_sd <= 0.0 || !fixture.discrepancy_threshold_sd.is_finite() {
        return Err("discrepancy threshold must be finite and positive".into());
    }
    let limits = &fixture.safety_limits;
    let limit_values = [
        limits.maximum_pressure_amplitude,
        limits.maximum_injected_volume,
        limits.maximum_duration_s,
        limits.maximum_tracer_mass,
        limits.maximum_actuator_effort,
        limits.maximum_sample_damage_index,
    ];
    if limit_values.iter().any(|value| !value.is_finite() || *value <= 0.0)
        || limits.acquisition_budget != 2
    {
        return Err("finite positive safety limits and a two-step acquisition budget are required".into());
    }
    let prior_sum: f64 = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .sum();
    if (prior_sum - 1.0).abs() > 1.0e-12 {
        return Err("mechanism prior probabilities must sum to one".into());
    }
    let mechanism_ids: Vec<&str> = fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.mechanism_id.as_str())
        .collect();
    let protocol_ids: Vec<&str> = fixture
        .protocols
        .iter()
        .map(|protocol| protocol.protocol_id.as_str())
        .collect();
    if has_duplicates(&mechanism_ids) || has_duplicates(&protocol_ids) {
        return Err("mechanism and protocol IDs must be unique".into());
    }
    for mechanism in &fixture.mechanisms {
        let values = [
            mechanism.prior_probability,
            mechanism.conductance,
            mechanism.storage_time_constant_s,
            mechanism.preferential_flow_fraction,
            mechanism.immobile_fraction,
            mechanism.dead_end_storage_fraction,
            mechanism.non_darcy_beta,
            mechanism.effective_porosity,
        ];
        if mechanism.mechanism_id.trim().is_empty()
            || values.iter().any(|value| !value.is_finite())
            || mechanism.prior_probability <= 0.0
            || mechanism.conductance <= 0.0
            || mechanism.storage_time_constant_s <= 0.0
            || mechanism.non_darcy_beta < 0.0
            || !(0.0..=1.0).contains(&mechanism.preferential_flow_fraction)
            || !(0.0..=1.0).contains(&mechanism.immobile_fraction)
            || !(0.0..=1.0).contains(&mechanism.dead_end_storage_fraction)
            || !(0.0..=1.0).contains(&mechanism.effective_porosity)
        {
            return Err("mechanism parameters are outside their finite normalized domain".into());
        }
    }
    for protocol in &fixture.protocols {
        let values = [
            protocol.pressure_amplitude,
            protocol.frequency_hz,
            protocol.duration_s,
            protocol.injected_volume,
            protocol.tracer_mass,
            protocol.actuator_effort,
            protocol.sample_damage_index,
            protocol.noise_std,
        ];
        if protocol.protocol_id.trim().is_empty()
            || values.iter().any(|value| !value.is_finite())
            || protocol.pressure_amplitude <= 0.0
            || protocol.frequency_hz < 0.0
            || protocol.duration_s <= 0.0
            || protocol.injected_volume < 0.0
            || protocol.tracer_mass < 0.0
            || protocol.actuator_effort < 0.0
            || protocol.sample_damage_index < 0.0
            || protocol.noise_std <= 0.0
            || protocol.acquisition_cost != 1
        {
            return Err("protocol fields must be finite, non-negative and unit-cost".into());
        }
        if protocol.waveform == Waveform::DualPulseTracer && protocol.tracer_mass <= 0.0 {
            return Err("dual-pulse tracer protocol requires positive tracer mass".into());
        }
        if matches!(
            protocol.observable,
            Observable::TracerFirstMoment | Observable::LateTracerFraction
        ) && protocol.tracer_mass <= 0.0
        {
            return Err("tracer observable requires positive tracer mass".into());
        }
    }
    if feasible_protocol_indices(fixture).len() < 3 {
        return Err("fewer than three protocols satisfy the declared hard constraints".into());
    }
    Ok(())
}

fn has_duplicates(values: &[&str]) -> bool {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted.windows(2).any(|pair| pair[0] == pair[1])
}

fn protocol_exclusion_reasons(protocol: &Protocol, limits: &SafetyLimits) -> Vec<String> {
    let mut reasons = Vec::new();
    if protocol.pressure_amplitude > limits.maximum_pressure_amplitude {
        reasons.push("pressure_amplitude_exceeds_limit".into());
    }
    if protocol.injected_volume > limits.maximum_injected_volume {
        reasons.push("injected_volume_exceeds_limit".into());
    }
    if protocol.duration_s > limits.maximum_duration_s {
        reasons.push("duration_exceeds_limit".into());
    }
    if protocol.tracer_mass > limits.maximum_tracer_mass {
        reasons.push("tracer_mass_exceeds_limit".into());
    }
    if protocol.actuator_effort > limits.maximum_actuator_effort {
        reasons.push("actuator_effort_exceeds_limit".into());
    }
    if protocol.sample_damage_index > limits.maximum_sample_damage_index {
        reasons.push("sample_damage_index_exceeds_limit".into());
    }
    reasons
}

fn feasible_protocol_indices(fixture: &Fixture) -> Vec<usize> {
    let mut indices: Vec<usize> = fixture
        .protocols
        .iter()
        .enumerate()
        .filter(|(_, protocol)| {
            protocol_exclusion_reasons(protocol, &fixture.safety_limits).is_empty()
                && protocol.acquisition_cost <= fixture.safety_limits.acquisition_budget
        })
        .map(|(index, _)| index)
        .collect();
    indices.sort_by(|left, right| {
        fixture.protocols[*left]
            .protocol_id
            .cmp(&fixture.protocols[*right].protocol_id)
    });
    indices
}

fn prior_probabilities(fixture: &Fixture) -> Vec<f64> {
    fixture
        .mechanisms
        .iter()
        .map(|mechanism| mechanism.prior_probability)
        .collect()
}

fn protocol_receipt(protocol: &Protocol) -> ProtocolReceipt {
    ProtocolReceipt {
        protocol_id: protocol.protocol_id.clone(),
        waveform: protocol.waveform,
        observable: protocol.observable,
        pressure_amplitude: protocol.pressure_amplitude,
        frequency_hz: protocol.frequency_hz,
        duration_s: protocol.duration_s,
        injected_volume: protocol.injected_volume,
        tracer_mass: protocol.tracer_mass,
        actuator_effort: protocol.actuator_effort,
        sample_damage_index: protocol.sample_damage_index,
        acquisition_cost: protocol.acquisition_cost,
        noise_std: protocol.noise_std,
    }
}

fn expected_observation(mechanism: &Mechanism, protocol: &Protocol) -> f64 {
    let settling = 1.0 - (-protocol.duration_s / mechanism.storage_time_constant_s).exp();
    let angular_frequency =
        2.0 * std::f64::consts::PI * protocol.frequency_hz * mechanism.storage_time_constant_s;
    let frequency_norm = (1.0 + angular_frequency * angular_frequency).sqrt();
    let excitation_gain = match protocol.waveform {
        Waveform::ConstantHeadStep => settling,
        Waveform::SquareWave => settling / frequency_norm,
        Waveform::Sinusoidal => settling * angular_frequency / frequency_norm,
        Waveform::DualPulseTracer => {
            settling * (1.0 + 0.25 * mechanism.preferential_flow_fraction)
        }
    };
    let applied_pressure = protocol.pressure_amplitude * excitation_gain;
    let raw_flow = mechanism.conductance * applied_pressure
        / (1.0 + mechanism.non_darcy_beta * mechanism.conductance * applied_pressure);
    let partitioned_flow =
        raw_flow * (0.35 + 0.65 * mechanism.preferential_flow_fraction);
    match protocol.observable {
        Observable::OutletPressure => {
            applied_pressure / (1.0 + mechanism.non_darcy_beta * raw_flow)
        }
        Observable::ActuatorDisplacement => {
            applied_pressure
                * protocol.duration_s
                * (mechanism.effective_porosity + 0.4 * mechanism.immobile_fraction)
                / frequency_norm
        }
        Observable::TotalFlow => partitioned_flow,
        Observable::TracerFirstMoment => {
            let swept_volume = (protocol.injected_volume
                * (1.0 - mechanism.dead_end_storage_fraction)
                * (1.0 - 0.5 * mechanism.immobile_fraction))
                .max(1.0e-9);
            let mobile_flow = partitioned_flow.max(1.0e-9);
            swept_volume / mobile_flow
                + mechanism.storage_time_constant_s
                    * (mechanism.immobile_fraction + mechanism.dead_end_storage_fraction)
        }
        Observable::LateTracerFraction => {
            let slow_fraction =
                (mechanism.immobile_fraction + mechanism.dead_end_storage_fraction) / 2.0;
            slow_fraction
                * (1.0
                    - (-protocol.duration_s
                        / (mechanism.storage_time_constant_s
                            * (1.0 + mechanism.dead_end_storage_fraction)))
                        .exp())
        }
    }
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
    prior: &[f64],
    observations: &[ProtocolObservation],
) -> Vec<f64> {
    let log_weights: Vec<f64> = fixture
        .mechanisms
        .iter()
        .enumerate()
        .map(|(mechanism_index, mechanism)| {
            let log_likelihood = observations
                .iter()
                .map(|entry| {
                    let protocol = &fixture.protocols[entry.protocol_index];
                    log_normal_density(
                        entry.observation,
                        expected_observation(mechanism, protocol),
                        protocol.noise_std,
                    )
                })
                .sum::<f64>();
            prior[mechanism_index].ln() + log_likelihood
        })
        .collect();
    let normalizer = log_sum_exp(&log_weights);
    log_weights
        .iter()
        .map(|value| (value - normalizer).exp())
        .collect()
}

fn entropy(probabilities: &[f64]) -> f64 {
    -probabilities
        .iter()
        .filter(|probability| **probability > 0.0)
        .map(|probability| probability * probability.ln())
        .sum::<f64>()
}

fn expected_entropy_after_protocol(
    fixture: &Fixture,
    posterior: &[f64],
    protocol_index: usize,
) -> f64 {
    let protocol = &fixture.protocols[protocol_index];
    let mut expected_entropy = 0.0;
    for (mechanism_index, probability) in posterior.iter().enumerate() {
        let mean = expected_observation(&fixture.mechanisms[mechanism_index], protocol);
        for (node, weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let posterior_after = posterior_update(
                fixture,
                posterior,
                &[ProtocolObservation {
                    protocol_index,
                    observation: mean + protocol.noise_std * node,
                }],
            );
            expected_entropy += probability * weight * entropy(&posterior_after);
        }
    }
    expected_entropy
}

fn best_second_protocol(
    fixture: &Fixture,
    posterior: &[f64],
    first_protocol_index: usize,
) -> Option<(usize, f64)> {
    let remaining_budget = fixture
        .safety_limits
        .acquisition_budget
        .saturating_sub(fixture.protocols[first_protocol_index].acquisition_cost);
    let current_entropy = entropy(posterior);
    let mut best: Option<(usize, f64)> = None;
    for protocol_index in feasible_protocol_indices(fixture) {
        let protocol = &fixture.protocols[protocol_index];
        if protocol_index == first_protocol_index || protocol.acquisition_cost > remaining_budget {
            continue;
        }
        let score = expected_entropy_after_protocol(fixture, posterior, protocol_index);
        let replace = best
            .as_ref()
            .map(|(best_index, best_score)| {
                score < *best_score - 1.0e-12
                    || ((score - *best_score).abs() <= 1.0e-12
                        && protocol.protocol_id
                            < fixture.protocols[*best_index].protocol_id)
            })
            .unwrap_or(true);
        if replace {
            best = Some((protocol_index, score));
        }
    }
    best.filter(|(_, score)| *score < current_entropy - 1.0e-12)
}

fn discrepancy_abstained(
    fixture: &Fixture,
    observation: &ProtocolObservation,
) -> bool {
    let protocol = &fixture.protocols[observation.protocol_index];
    let minimum_best_residual = fixture
        .mechanisms
        .iter()
        .map(|mechanism| {
            ((observation.observation - expected_observation(mechanism, protocol))
                / protocol.noise_std)
                .abs()
        })
        .fold(f64::INFINITY, f64::min);
    minimum_best_residual > fixture.discrepancy_threshold_sd
}

fn evaluate_first_protocol(
    fixture: &Fixture,
    first_protocol_index: usize,
) -> FirstStageEvaluation {
    let prior = prior_probabilities(fixture);
    let first_protocol = &fixture.protocols[first_protocol_index];
    let mut terminal_entropy = 0.0;
    let mut expected_cost = first_protocol.acquisition_cost as f64;
    let mut stop_probability_mass = 0.0;
    let mut discrepancy_abstention_probability_mass = 0.0;
    let mut second_protocol_shares: Vec<(String, f64)> = Vec::new();
    let mut branches: Vec<ObservationBranch> = Vec::new();

    for (true_mechanism_index, mechanism_probability) in prior.iter().enumerate() {
        let mean = expected_observation(
            &fixture.mechanisms[true_mechanism_index],
            first_protocol,
        );
        for (node, quadrature_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
            let observation = ProtocolObservation {
                protocol_index: first_protocol_index,
                observation: mean + first_protocol.noise_std * node,
            };
            let posterior = posterior_update(fixture, &prior, std::slice::from_ref(&observation));
            let abstained = discrepancy_abstained(fixture, &observation);
            let next = if abstained {
                None
            } else {
                best_second_protocol(fixture, &posterior, first_protocol_index)
            };
            let conditional_entropy = next
                .map(|(_, expected_entropy)| expected_entropy)
                .unwrap_or_else(|| entropy(&posterior));
            let joint_mass = mechanism_probability * quadrature_weight;
            terminal_entropy += joint_mass * conditional_entropy;

            if abstained {
                discrepancy_abstention_probability_mass += joint_mass;
            }
            if let Some((next_index, _)) = next {
                let next_protocol = &fixture.protocols[next_index];
                expected_cost += joint_mass * next_protocol.acquisition_cost as f64;
                if let Some(entry) = second_protocol_shares
                    .iter_mut()
                    .find(|(id, _)| id == &next_protocol.protocol_id)
                {
                    entry.1 += joint_mass;
                } else {
                    second_protocol_shares.push((next_protocol.protocol_id.clone(), joint_mass));
                }
            } else {
                stop_probability_mass += joint_mass;
            }

            if let Some(entry) = branches
                .iter_mut()
                .find(|branch| branch.observed_value.to_bits() == observation.observation.to_bits())
            {
                entry.predictive_probability_mass += joint_mass;
            } else {
                let next_id = next.map(|(index, _)| fixture.protocols[index].protocol_id.clone());
                branches.push(ObservationBranch {
                    observed_value: observation.observation,
                    posterior_mechanism_probabilities: posterior,
                    next_protocol_id: next_id,
                    discrepancy_abstained: abstained,
                    predictive_probability_mass: joint_mass,
                });
            }
        }
    }

    branches.sort_by(|left, right| left.observed_value.total_cmp(&right.observed_value));
    second_protocol_shares.sort_by(|left, right| left.0.cmp(&right.0));
    FirstStageEvaluation {
        terminal_entropy,
        expected_cost,
        stop_probability_mass,
        discrepancy_abstention_probability_mass,
        second_protocol_shares,
        branches,
    }
}

fn best_fixed_pair(fixture: &Fixture) -> Result<PolicyReport, String> {
    let prior = prior_probabilities(fixture);
    let feasible = feasible_protocol_indices(fixture);
    let mut best: Option<(usize, usize, f64)> = None;
    for left_position in 0..feasible.len() {
        for right_position in (left_position + 1)..feasible.len() {
            let left = feasible[left_position];
            let right = feasible[right_position];
            let cost = fixture.protocols[left].acquisition_cost
                + fixture.protocols[right].acquisition_cost;
            if cost > fixture.safety_limits.acquisition_budget {
                continue;
            }
            let mut expected_entropy = 0.0;
            for (true_mechanism, mechanism_probability) in prior.iter().enumerate() {
                let left_mean =
                    expected_observation(&fixture.mechanisms[true_mechanism], &fixture.protocols[left]);
                let right_mean = expected_observation(
                    &fixture.mechanisms[true_mechanism],
                    &fixture.protocols[right],
                );
                for (left_node, left_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
                    for (right_node, right_weight) in GH_NODES.iter().zip(GH_WEIGHTS.iter()) {
                        let posterior = posterior_update(
                            fixture,
                            &prior,
                            &[
                                ProtocolObservation {
                                    protocol_index: left,
                                    observation: left_mean
                                        + fixture.protocols[left].noise_std * left_node,
                                },
                                ProtocolObservation {
                                    protocol_index: right,
                                    observation: right_mean
                                        + fixture.protocols[right].noise_std * right_node,
                                },
                            ],
                        );
                        expected_entropy += mechanism_probability
                            * left_weight
                            * right_weight
                            * entropy(&posterior);
                    }
                }
            }
            let replace = best
                .as_ref()
                .map(|(best_left, best_right, best_score)| {
                    expected_entropy < *best_score - 1.0e-12
                        || ((expected_entropy - *best_score).abs() <= 1.0e-12
                            && (
                                fixture.protocols[left].protocol_id.as_str(),
                                fixture.protocols[right].protocol_id.as_str(),
                            ) < (
                                fixture.protocols[*best_left].protocol_id.as_str(),
                                fixture.protocols[*best_right].protocol_id.as_str(),
                            ))
                })
                .unwrap_or(true);
            if replace {
                best = Some((left, right, expected_entropy));
            }
        }
    }
    let (first, second, expected_entropy) =
        best.ok_or_else(|| "no feasible fixed two-protocol portfolio".to_string())?;
    let prior_entropy = entropy(&prior);
    Ok(PolicyReport {
        policy_id: "best_fixed_two_protocol_portfolio".into(),
        first_protocol_id: fixture.protocols[first].protocol_id.clone(),
        second_protocol_shares: vec![(fixture.protocols[second].protocol_id.clone(), 1.0)],
        expected_terminal_model_entropy_nats: expected_entropy,
        expected_model_information_gain_nats: (prior_entropy - expected_entropy).max(0.0),
        expected_acquisition_cost: (fixture.protocols[first].acquisition_cost
            + fixture.protocols[second].acquisition_cost) as f64,
        stop_probability_mass: 0.0,
        discrepancy_abstention_probability_mass: 0.0,
        branches: Vec::new(),
    })
}

fn adaptive_policy(fixture: &Fixture) -> Result<PolicyReport, String> {
    let prior_entropy = entropy(&prior_probabilities(fixture));
    let mut best: Option<(usize, FirstStageEvaluation)> = None;
    for first_index in feasible_protocol_indices(fixture) {
        let evaluation = evaluate_first_protocol(fixture, first_index);
        let replace = best
            .as_ref()
            .map(|(best_index, best_evaluation)| {
                evaluation.terminal_entropy < best_evaluation.terminal_entropy - 1.0e-12
                    || ((evaluation.terminal_entropy - best_evaluation.terminal_entropy).abs()
                        <= 1.0e-12
                        && fixture.protocols[first_index].protocol_id
                            < fixture.protocols[*best_index].protocol_id)
            })
            .unwrap_or(true);
        if replace {
            best = Some((first_index, evaluation));
        }
    }
    let (first_index, evaluation) =
        best.ok_or_else(|| "no feasible first protocol for adaptive policy".to_string())?;
    Ok(PolicyReport {
        policy_id: "finite_horizon_adaptive_action_observation_eig".into(),
        first_protocol_id: fixture.protocols[first_index].protocol_id.clone(),
        second_protocol_shares: evaluation.second_protocol_shares,
        expected_terminal_model_entropy_nats: evaluation.terminal_entropy,
        expected_model_information_gain_nats: (prior_entropy - evaluation.terminal_entropy).max(0.0),
        expected_acquisition_cost: evaluation.expected_cost,
        stop_probability_mass: evaluation.stop_probability_mass,
        discrepancy_abstention_probability_mass: evaluation
            .discrepancy_abstention_probability_mass,
        branches: evaluation.branches,
    })
}

fn evaluate(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let feasible_indices = feasible_protocol_indices(fixture);
    let feasible_protocols = feasible_indices
        .iter()
        .map(|index| protocol_receipt(&fixture.protocols[*index]))
        .collect();
    let excluded_protocols = fixture
        .protocols
        .iter()
        .filter_map(|protocol| {
            let reasons = protocol_exclusion_reasons(protocol, &fixture.safety_limits);
            if reasons.is_empty() {
                None
            } else {
                Some(ExcludedProtocol {
                    protocol_id: protocol.protocol_id.clone(),
                    reasons,
                })
            }
        })
        .collect();
    let fixed = best_fixed_pair(fixture)?;
    let adaptive = adaptive_policy(fixture)?;
    let adaptive_terminal_entropy_reduction_vs_fixed_nats =
        fixed.expected_terminal_model_entropy_nats
            - adaptive.expected_terminal_model_entropy_nats;
    Ok(Report {
        schema_version: SCHEMA.into(),
        fixture_id: fixture.fixture_id.clone(),
        fixture_status: fixture.fixture_status.clone(),
        input_sha256,
        feasible_protocols,
        excluded_protocols,
        hard_constraint_violations_in_selected_policy: 0,
        discrepancy_threshold_sd: fixture.discrepancy_threshold_sd,
        quadrature_nodes: GH_NODES.len(),
        acquisition_budget: fixture.safety_limits.acquisition_budget,
        best_fixed_policy: fixed,
        sequential_adaptive_policy: adaptive,
        adaptive_terminal_entropy_reduction_vs_fixed_nats,
        policy_semantics: "The adaptive policy chooses a first feasible intervention-observation protocol, updates the model posterior after its noisy result, applies a closed-set discrepancy gate, and then chooses the remaining feasible protocol with minimum posterior expected model entropy or stops. The fixed comparator exhaustively enumerates every feasible two-protocol set. Both use the same seven-node Gauss-Hermite observation quadrature.".into(),
        claim_boundary: "Synthetic dimensionless state-response reference only. Protocol response equations are analyst-authored illustrative reduced-order dynamics, not calibrated Darcy/Richards constitutive laws or field-safe operating limits. Fixture limits are benchmark constraints only. No physical deployment, soil parameter or amendment efficacy claim is established.".into(),
    })
}

fn run(input_path: &str, output_path: &str) -> Result<Report, String> {
    let input_bytes = fs::read(input_path)
        .map_err(|error| format!("failed to read action-conditioned fixture: {error}"))?;
    let input_sha256 = hex_digest(&Sha256::digest(&input_bytes));
    let fixture: Fixture = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("invalid action-conditioned fixture JSON: {error}"))?;
    let report = evaluate(&fixture, input_sha256)?;
    let bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("failed to serialize action-conditioned report: {error}"))?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| {
            format!("output must be a new file; refusing overwrite at {output_path}: {error}")
        })?;
    output
        .write_all(&bytes)
        .map_err(|error| format!("failed to write action-conditioned report: {error}"))?;
    println!(
        "feasible_protocols={} excluded_protocols={} fixed_first={} fixed_second={} adaptive_first={} adaptive_entropy_gain={:.9} adaptive_expected_cost={:.6} adaptive_stop_mass={:.6} adaptive_abstention_mass={:.6}",
        report.feasible_protocols.len(),
        report.excluded_protocols.len(),
        report.best_fixed_policy.first_protocol_id,
        report.best_fixed_policy
            .second_protocol_shares
            .first()
            .map(|entry| entry.0.as_str())
            .unwrap_or("none"),
        report.sequential_adaptive_policy.first_protocol_id,
        report.adaptive_terminal_entropy_reduction_vs_fixed_nats,
        report.sequential_adaptive_policy.expected_acquisition_cost,
        report.sequential_adaptive_policy.stop_probability_mass,
        report.sequential_adaptive_policy.discrepancy_abstention_probability_mass
    );
    Ok(report)
}

fn hex_digest(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: terra-action-conditioned-boed INPUT.json OUTPUT.json");
        process::exit(2);
    }
    if let Err(error) = run(&args[1], &args[2]) {
        eprintln!("action-conditioned BOED error: {error}");
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
            fixture_id: "action-conditioned-smoke-fixture".into(),
            description: "Synthetic dimensionless response fixture".into(),
            discrepancy_threshold_sd: 4.0,
            safety_limits: SafetyLimits {
                maximum_pressure_amplitude: 0.6,
                maximum_injected_volume: 0.6,
                maximum_duration_s: 40.0,
                maximum_tracer_mass: 0.1,
                maximum_actuator_effort: 0.5,
                maximum_sample_damage_index: 0.2,
                acquisition_budget: 2,
            },
            mechanisms: vec![
                Mechanism {
                    mechanism_id: "balanced".into(),
                    prior_probability: 0.25,
                    conductance: 1.4,
                    storage_time_constant_s: 4.0,
                    preferential_flow_fraction: 0.5,
                    immobile_fraction: 0.1,
                    dead_end_storage_fraction: 0.1,
                    non_darcy_beta: 0.04,
                    effective_porosity: 0.30,
                },
                Mechanism {
                    mechanism_id: "preferential".into(),
                    prior_probability: 0.25,
                    conductance: 1.4,
                    storage_time_constant_s: 2.0,
                    preferential_flow_fraction: 0.9,
                    immobile_fraction: 0.05,
                    dead_end_storage_fraction: 0.02,
                    non_darcy_beta: 0.06,
                    effective_porosity: 0.25,
                },
                Mechanism {
                    mechanism_id: "dual_domain".into(),
                    prior_probability: 0.25,
                    conductance: 1.4,
                    storage_time_constant_s: 12.0,
                    preferential_flow_fraction: 0.4,
                    immobile_fraction: 0.55,
                    dead_end_storage_fraction: 0.4,
                    non_darcy_beta: 0.03,
                    effective_porosity: 0.4,
                },
                Mechanism {
                    mechanism_id: "forchheimer".into(),
                    prior_probability: 0.25,
                    conductance: 1.4,
                    storage_time_constant_s: 5.0,
                    preferential_flow_fraction: 0.35,
                    immobile_fraction: 0.15,
                    dead_end_storage_fraction: 0.1,
                    non_darcy_beta: 0.8,
                    effective_porosity: 0.28,
                },
            ],
            protocols: vec![
                Protocol {
                    protocol_id: "step_outlet_pressure".into(),
                    waveform: Waveform::ConstantHeadStep,
                    observable: Observable::OutletPressure,
                    pressure_amplitude: 0.4,
                    frequency_hz: 0.0,
                    duration_s: 8.0,
                    injected_volume: 0.25,
                    tracer_mass: 0.0,
                    actuator_effort: 0.2,
                    sample_damage_index: 0.05,
                    acquisition_cost: 1,
                    noise_std: 0.08,
                },
                Protocol {
                    protocol_id: "squarewave_actuator_displacement".into(),
                    waveform: Waveform::SquareWave,
                    observable: Observable::ActuatorDisplacement,
                    pressure_amplitude: 0.35,
                    frequency_hz: 0.25,
                    duration_s: 16.0,
                    injected_volume: 0.30,
                    tracer_mass: 0.0,
                    actuator_effort: 0.3,
                    sample_damage_index: 0.08,
                    acquisition_cost: 1,
                    noise_std: 0.14,
                },
                Protocol {
                    protocol_id: "sine_total_flow".into(),
                    waveform: Waveform::Sinusoidal,
                    observable: Observable::TotalFlow,
                    pressure_amplitude: 0.45,
                    frequency_hz: 0.6,
                    duration_s: 12.0,
                    injected_volume: 0.35,
                    tracer_mass: 0.0,
                    actuator_effort: 0.45,
                    sample_damage_index: 0.1,
                    acquisition_cost: 1,
                    noise_std: 0.09,
                },
                Protocol {
                    protocol_id: "dual_pulse_tracer_moment".into(),
                    waveform: Waveform::DualPulseTracer,
                    observable: Observable::TracerFirstMoment,
                    pressure_amplitude: 0.3,
                    frequency_hz: 0.0,
                    duration_s: 20.0,
                    injected_volume: 0.4,
                    tracer_mass: 0.05,
                    actuator_effort: 0.3,
                    sample_damage_index: 0.15,
                    acquisition_cost: 1,
                    noise_std: 0.8,
                },
                Protocol {
                    protocol_id: "dual_pulse_late_tracer".into(),
                    waveform: Waveform::DualPulseTracer,
                    observable: Observable::LateTracerFraction,
                    pressure_amplitude: 0.3,
                    frequency_hz: 0.0,
                    duration_s: 40.0,
                    injected_volume: 0.5,
                    tracer_mass: 0.05,
                    actuator_effort: 0.4,
                    sample_damage_index: 0.18,
                    acquisition_cost: 1,
                    noise_std: 0.08,
                },
                Protocol {
                    protocol_id: "unsafe_high_pressure".into(),
                    waveform: Waveform::ConstantHeadStep,
                    observable: Observable::TotalFlow,
                    pressure_amplitude: 1.5,
                    frequency_hz: 0.0,
                    duration_s: 55.0,
                    injected_volume: 0.8,
                    tracer_mass: 0.2,
                    actuator_effort: 0.8,
                    sample_damage_index: 0.4,
                    acquisition_cost: 1,
                    noise_std: 0.1,
                },
            ],
        }
    }

    #[test]
    fn hard_limits_exclude_unsafe_pressure_volume_and_duration() {
        let f = fixture();
        let unsafe_protocol = f.protocols.last().unwrap();
        let reasons = protocol_exclusion_reasons(unsafe_protocol, &f.safety_limits);
        assert!(reasons.contains(&"pressure_amplitude_exceeds_limit".into()));
        assert!(reasons.contains(&"injected_volume_exceeds_limit".into()));
        assert!(reasons.contains(&"duration_exceeds_limit".into()));
        assert!(reasons.contains(&"tracer_mass_exceeds_limit".into()));
        assert!(reasons.contains(&"actuator_effort_exceeds_limit".into()));
        assert!(reasons.contains(&"sample_damage_index_exceeds_limit".into()));
        assert!(!feasible_protocol_indices(&f).contains(&(f.protocols.len() - 1)));
    }

    #[test]
    fn action_waveform_changes_the_same_mechanisms_observable_response() {
        let mut f = fixture();
        let mechanism = &f.mechanisms[2];
        let step = expected_observation(mechanism, &f.protocols[0]);
        f.protocols[0].waveform = Waveform::Sinusoidal;
        f.protocols[0].frequency_hz = 0.6;
        let sine = expected_observation(mechanism, &f.protocols[0]);
        assert!((step - sine).abs() > 1.0e-3);
    }

    #[test]
    fn posterior_is_normalized_after_joint_protocol_observations() {
        let f = fixture();
        let posterior = posterior_update(
            &f,
            &prior_probabilities(&f),
            &[
                ProtocolObservation {
                    protocol_index: 0,
                    observation: 0.3,
                },
                ProtocolObservation {
                    protocol_index: 3,
                    observation: 2.4,
                },
            ],
        );
        assert!((posterior.iter().sum::<f64>() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_policy_respects_the_budget_and_beats_best_fixed_pair() {
        let f = fixture();
        let fixed = best_fixed_pair(&f).unwrap();
        let adaptive = adaptive_policy(&f).unwrap();
        assert!(adaptive.expected_acquisition_cost <= f.safety_limits.acquisition_budget as f64);
        assert!(
            adaptive.expected_terminal_model_entropy_nats
                <= fixed.expected_terminal_model_entropy_nats + 1.0e-12
        );
        assert_eq!(adaptive.first_protocol_id, "squarewave_actuator_displacement");
        assert_eq!(fixed.policy_id, "best_fixed_two_protocol_portfolio");
    }

    #[test]
    fn out_of_family_first_observation_triggers_abstention() {
        let f = fixture();
        let protocol_index = 0;
        let observation = ProtocolObservation {
            protocol_index,
            observation: expected_observation(&f.mechanisms[0], &f.protocols[protocol_index])
                + 10.0 * f.protocols[protocol_index].noise_std,
        };
        assert!(discrepancy_abstained(&f, &observation));
    }

    #[test]
    fn candidate_order_does_not_change_protocol_selection() {
        let original = adaptive_policy(&fixture()).unwrap();
        let mut reordered = fixture();
        reordered.protocols.reverse();
        let permuted = adaptive_policy(&reordered).unwrap();
        assert_eq!(original.first_protocol_id, permuted.first_protocol_id);
        assert!(
            (original.expected_terminal_model_entropy_nats
                - permuted.expected_terminal_model_entropy_nats)
                .abs()
                < 1.0e-12
        );
    }
}
