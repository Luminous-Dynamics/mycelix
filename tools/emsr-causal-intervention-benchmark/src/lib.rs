use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const INPUT_CONTRACT: &str = "emsr.causal-intervention-public-input/0.1";
pub const REPORT_CONTRACT: &str = "emsr.causal-intervention-candidate-report/0.1";
pub const TRUTH_CONTRACT: &str = "emsr.causal-intervention-hidden-truth/0.1";
pub const SCORE_CONTRACT: &str = "emsr.causal-intervention-evaluation-score/0.1";

const CLAIM_CEILING: &str = "Synthetic controlled-intervention evidence for the declared intervention, endpoints, observation window and public scenario. A non-detection is not proof of physical or statistical independence and does not generalize beyond the tested envelope.";

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TrialArm {
    Active,
    Sham,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TimeMapping {
    Qualified,
    Unknown,
    Invalid,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseState {
    Degraded,
    Nominal,
    Unobserved,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationRole {
    TargetEndpoint,
    NegativeControl,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CandidateDisposition {
    CommonCauseSupportedWithinInterventionScope,
    NoCommonCauseDetectedWithinTestedEnvelope,
    IndeterminateInterventionCoverage,
    ObservationOrClockMappingInvalid,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InterventionPlan {
    pub intervention_id: String,
    pub target_id: String,
    pub domain: String,
    pub intervention_kind: String,
    pub minimum_latency_ns: u64,
    pub maximum_latency_ns: u64,
    pub minimum_active_trials: usize,
    pub minimum_sham_trials: usize,
    pub response_threshold_milli: u16,
    pub maximum_sham_response_rate_milli: u16,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Trial {
    pub trial_id: String,
    pub arm: TrialArm,
    pub randomized_order: u64,
    pub injection_time_tai_ns: u64,
    pub observation_start_tai_ns: u64,
    pub observation_end_tai_ns: u64,
    pub randomization_receipt_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TrialObservation {
    pub trial_id: String,
    pub source_id: String,
    pub event_time_tai_ns: u64,
    pub received_time_tai_ns: u64,
    pub sequence: u64,
    pub time_mapping: TimeMapping,
    pub state: ResponseState,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PublicInput {
    pub contract: String,
    pub scenario_id: String,
    pub decision_scope: String,
    pub expected_source_ids: Vec<String>,
    pub negative_control_source_ids: Vec<String>,
    pub intervention: InterventionPlan,
    pub trials: Vec<Trial>,
    pub observations: Vec<TrialObservation>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SourceContrast {
    pub source_id: String,
    pub role: ObservationRole,
    pub active_responses: usize,
    pub active_trials: usize,
    pub sham_responses: usize,
    pub sham_trials: usize,
    pub active_rate_milli: u16,
    pub sham_rate_milli: u16,
    pub responsive: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CandidateReport {
    pub report_contract: String,
    pub scenario_id: String,
    pub disposition: CandidateDisposition,
    pub input_sha256: String,
    pub intervention_id: String,
    pub target_id: String,
    pub declared_scope: String,
    pub responsive_source_ids: Vec<String>,
    pub negative_control_response_source_ids: Vec<String>,
    pub contrasts: Vec<SourceContrast>,
    pub observations_used: usize,
    pub tested_active_trials: usize,
    pub tested_sham_trials: usize,
    pub reasons: Vec<String>,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HiddenCausalPath {
    pub intervention_target_id: String,
    pub source_id: String,
    pub domain: String,
    pub path_node_ids: Vec<String>,
    pub minimum_latency_ns: u64,
    pub maximum_latency_ns: u64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HiddenTruth {
    pub contract: String,
    pub scenario_id: String,
    pub causal_paths: Vec<HiddenCausalPath>,
    pub truth_commitment_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluationScore {
    pub score_contract: String,
    pub scenario_id: String,
    pub public_input_sha256: String,
    pub candidate_report_sha256: String,
    pub truth_commitment_sha256: String,
    pub ground_truth_common_cause: bool,
    pub candidate_disposition: CandidateDisposition,
    pub true_positive: bool,
    pub false_positive: bool,
    pub true_negative: bool,
    pub false_negative: bool,
    pub abstained: bool,
    pub false_independence_claim: bool,
    pub common_cause_source_ids: Vec<String>,
    pub limitations: Vec<String>,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_public_input(input: &PublicInput) -> Vec<String> {
    let mut errors = Vec::new();
    if input.contract != INPUT_CONTRACT {
        errors.push("unsupported_input_contract".to_owned());
    }
    if input.scenario_id.trim().is_empty() || input.decision_scope.trim().is_empty() {
        errors.push("scenario_identity_or_scope_empty".to_owned());
    }
    let plan = &input.intervention;
    if input.trials.len() > 4096
        || input.expected_source_ids.len() > 4096
        || input.observations.len() > 1_000_000
    {
        errors.push("public_input_exceeds_benchmark_resource_limits".to_owned());
    }
    if plan.intervention_id.trim().is_empty()
        || plan.target_id.trim().is_empty()
        || plan.domain.trim().is_empty()
        || plan.intervention_kind.trim().is_empty()
        || plan.minimum_latency_ns > plan.maximum_latency_ns
        || plan.minimum_active_trials == 0
        || plan.minimum_sham_trials == 0
        || plan.response_threshold_milli > 1000
        || plan.maximum_sham_response_rate_milli > 1000
    {
        errors.push("intervention_plan_invalid".to_owned());
    }
    let target_sources: BTreeSet<&str> = input
        .expected_source_ids
        .iter()
        .map(String::as_str)
        .collect();
    let control_sources: BTreeSet<&str> = input
        .negative_control_source_ids
        .iter()
        .map(String::as_str)
        .collect();
    let all_sources: BTreeSet<&str> = target_sources.union(&control_sources).copied().collect();
    if input.expected_source_ids.len() < 2
        || target_sources.len() != input.expected_source_ids.len()
        || input.expected_source_ids.iter().any(|id| id.trim().is_empty())
        || input.negative_control_source_ids.is_empty()
        || control_sources.len() != input.negative_control_source_ids.len()
        || input
            .negative_control_source_ids
            .iter()
            .any(|id| id.trim().is_empty())
        || !target_sources.is_disjoint(&control_sources)
    {
        errors.push("target_or_negative_control_inventory_invalid".to_owned());
    }

    let mut trial_ids = BTreeSet::new();
    let mut randomized_orders = BTreeSet::new();
    for trial in &input.trials {
        if trial.trial_id.trim().is_empty()
            || !trial_ids.insert(trial.trial_id.as_str())
            || !randomized_orders.insert(trial.randomized_order)
            || !valid_digest(&trial.randomization_receipt_sha256)
            || trial.observation_start_tai_ns > trial.observation_end_tai_ns
            || trial.observation_start_tai_ns
                < trial
                    .injection_time_tai_ns
                    .saturating_add(plan.minimum_latency_ns)
            || trial.observation_end_tai_ns
                > trial
                    .injection_time_tai_ns
                    .saturating_add(plan.maximum_latency_ns)
        {
            errors.push(format!("trial_{}_invalid_or_duplicate", trial.trial_id));
        }
    }

    let trial_by_id: BTreeMap<&str, &Trial> = input
        .trials
        .iter()
        .map(|trial| (trial.trial_id.as_str(), trial))
        .collect();
    let mut observation_keys = BTreeSet::new();
    let mut source_sequences: BTreeMap<&str, u64> = BTreeMap::new();
    for observation in &input.observations {
        let key = (
            observation.trial_id.as_str(),
            observation.source_id.as_str(),
        );
        if !observation_keys.insert(key) {
            errors.push(format!(
                "observation_{}_{}_duplicated",
                observation.trial_id, observation.source_id
            ));
        }
        if !all_sources.contains(observation.source_id.as_str()) {
            errors.push(format!(
                "observation_{}_source_outside_inventory",
                observation.source_id
            ));
        }
        let Some(trial) = trial_by_id.get(observation.trial_id.as_str()) else {
            errors.push(format!(
                "observation_{}_trial_missing",
                observation.trial_id
            ));
            continue;
        };
        if observation.event_time_tai_ns < trial.observation_start_tai_ns
            || observation.event_time_tai_ns > trial.observation_end_tai_ns
            || observation.received_time_tai_ns < observation.event_time_tai_ns
        {
            errors.push(format!(
                "observation_{}_event_or_receive_time_invalid",
                observation.source_id
            ));
        }
        if let Some(previous) = source_sequences.get(observation.source_id.as_str()) {
            if observation.sequence <= *previous {
                errors.push(format!(
                    "observation_{}_sequence_not_increasing",
                    observation.source_id
                ));
            }
        }
        source_sequences.insert(observation.source_id.as_str(), observation.sequence);
    }

    let active = input
        .trials
        .iter()
        .filter(|trial| trial.arm == TrialArm::Active)
        .count();
    let sham = input
        .trials
        .iter()
        .filter(|trial| trial.arm == TrialArm::Sham)
        .count();
    if active < plan.minimum_active_trials || sham < plan.minimum_sham_trials {
        errors.push("trial_repetition_count_below_plan_minimum".to_owned());
    }
    errors.sort();
    errors.dedup();
    errors
}

pub fn evaluate_public_bytes(public_bytes: &[u8]) -> CandidateReport {
    let input_sha256 = sha256_hex(public_bytes);
    let input = match serde_json::from_slice::<PublicInput>(public_bytes) {
        Ok(value) => value,
        Err(_) => {
            return invalid_report(
                input_sha256,
                String::new(),
                String::new(),
                "public_input_parse_failed",
            );
        }
    };
    evaluate_public(&input, input_sha256)
}

pub fn evaluate_public(input: &PublicInput, input_sha256: String) -> CandidateReport {
    let errors = valid_public_input(input);
    if !errors.is_empty() {
        return invalid_report(
            input_sha256,
            input.scenario_id.clone(),
            input.intervention.intervention_id.clone(),
            "public_input_invalid",
        );
    }

    let plan = &input.intervention;
    let trial_by_id: BTreeMap<&str, &Trial> = input
        .trials
        .iter()
        .map(|trial| (trial.trial_id.as_str(), trial))
        .collect();
    let mut by_source: BTreeMap<String, (usize, usize, usize, usize)> = input
        .expected_source_ids
        .iter()
        .chain(input.negative_control_source_ids.iter())
        .map(|source| (source.clone(), (0, 0, 0, 0)))
        .collect();
    let mut reasons = Vec::new();
    let mut time_invalid = false;
    let mut time_unknown = false;
    let mut unobserved = false;

    for observation in &input.observations {
        match observation.time_mapping {
            TimeMapping::Invalid => time_invalid = true,
            TimeMapping::Unknown => time_unknown = true,
            TimeMapping::Qualified => {}
        }
        if observation.state == ResponseState::Unobserved {
            unobserved = true;
        }
        let Some(trial) = trial_by_id.get(observation.trial_id.as_str()) else {
            continue;
        };
        let Some(counts) = by_source.get_mut(&observation.source_id) else {
            continue;
        };
        match (trial.arm, observation.state) {
            (TrialArm::Active, ResponseState::Degraded) => {
                counts.0 += 1;
                counts.1 += 1;
            }
            (TrialArm::Active, ResponseState::Nominal) => counts.1 += 1,
            (TrialArm::Sham, ResponseState::Degraded) => {
                counts.2 += 1;
                counts.3 += 1;
            }
            (TrialArm::Sham, ResponseState::Nominal) => counts.3 += 1,
            (_, ResponseState::Unobserved) => {}
        }
    }

    let expected_observations = input.trials.len()
        * (input.expected_source_ids.len() + input.negative_control_source_ids.len());
    let coverage_complete = input.observations.len() == expected_observations
        && !unobserved
        && input.observations.iter().all(|observation| {
            observation.time_mapping == TimeMapping::Qualified
                && observation.state != ResponseState::Unobserved
        });
    let sham_nuisance_detected = by_source.values().any(|(_, _, sham_response, sham_count)| {
        *sham_count > 0
            && *sham_response * 1000
                > usize::from(plan.maximum_sham_response_rate_milli) * *sham_count
    });
    let negative_control_response_detected = !negative_control_responses.is_empty();
    if !coverage_complete {
        reasons.push("observation_coverage_incomplete_or_unqualified".to_owned());
    }
    if sham_nuisance_detected {
        reasons.push("sham_trial_response_indicates_nuisance_or_noncausal_correlation".to_owned());
    }
    if negative_control_response_detected {
        reasons.push("negative_control_response_indicates_shared_observation_or_collection_artifact".to_owned());
    }
    if time_invalid {
        reasons.push("clock_mapping_or_observation_invalid".to_owned());
    } else if time_unknown {
        reasons.push("clock_mapping_unknown".to_owned());
    }

    let mut contrasts = Vec::with_capacity(
        input.expected_source_ids.len() + input.negative_control_source_ids.len(),
    );
    let mut responsive = Vec::new();
    let mut negative_control_responses = Vec::new();

    for source_id in &input.expected_source_ids {
        let (active_response, active_count, sham_response, sham_count) =
            by_source.get(source_id).copied().unwrap_or_default();
        let active_rate_milli = rate_milli(active_response, active_count);
        let sham_rate_milli = rate_milli(sham_response, sham_count);
        let is_responsive = endpoint_responsive(
            active_response,
            active_count,
            sham_response,
            sham_count,
            plan,
        );
        if is_responsive {
            responsive.push(source_id.clone());
        }
        contrasts.push(SourceContrast {
            source_id: source_id.clone(),
            role: ObservationRole::TargetEndpoint,
            active_responses: active_response,
            active_trials: active_count,
            sham_responses: sham_response,
            sham_trials: sham_count,
            active_rate_milli,
            sham_rate_milli,
            responsive: is_responsive,
        });
    }

    for source_id in &input.negative_control_source_ids {
        let (active_response, active_count, sham_response, sham_count) =
            by_source.get(source_id).copied().unwrap_or_default();
        let active_rate_milli = rate_milli(active_response, active_count);
        let sham_rate_milli = rate_milli(sham_response, sham_count);
        let is_responsive = endpoint_responsive(
            active_response,
            active_count,
            sham_response,
            sham_count,
            plan,
        );
        if is_responsive {
            negative_control_responses.push(source_id.clone());
        }
        contrasts.push(SourceContrast {
            source_id: source_id.clone(),
            role: ObservationRole::NegativeControl,
            active_responses: active_response,
            active_trials: active_count,
            sham_responses: sham_response,
            sham_trials: sham_count,
            active_rate_milli,
            sham_rate_milli,
            responsive: is_responsive,
        });
    }

    responsive.sort();
    negative_control_responses.sort();
    contrasts.sort_by(|a, b| a.source_id.cmp(&b.source_id));
    responsive.sort();

    let disposition = if time_invalid || !errors.is_empty() {
        CandidateDisposition::ObservationOrClockMappingInvalid
    } else if time_unknown
        || !coverage_complete
        || sham_nuisance_detected
        || negative_control_response_detected
    {
        CandidateDisposition::IndeterminateInterventionCoverage
    } else if responsive.len() >= 2 {
        CandidateDisposition::CommonCauseSupportedWithinInterventionScope
    } else {
        CandidateDisposition::NoCommonCauseDetectedWithinTestedEnvelope
    };

    CandidateReport {
        report_contract: REPORT_CONTRACT.to_owned(),
        scenario_id: input.scenario_id.clone(),
        disposition,
        input_sha256,
        intervention_id: plan.intervention_id.clone(),
        target_id: plan.target_id.clone(),
        declared_scope: input.decision_scope.clone(),
        responsive_source_ids: responsive,
        negative_control_response_source_ids: negative_control_responses,
        contrasts,
        observations_used: input.observations.len(),
        tested_active_trials: input
            .trials
            .iter()
            .filter(|trial| trial.arm == TrialArm::Active)
            .count(),
        tested_sham_trials: input
            .trials
            .iter()
            .filter(|trial| trial.arm == TrialArm::Sham)
            .count(),
        reasons,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    }
}

fn hidden_common_cause_sources(input: &PublicInput, truth: &HiddenTruth) -> Vec<String> {
    let expected: BTreeSet<&str> = input
        .expected_source_ids
        .iter()
        .map(String::as_str)
        .collect();
    let mut sources = BTreeSet::new();
    for path in &truth.causal_paths {
        if path.intervention_target_id == input.intervention.target_id
            && path.domain == input.intervention.domain
            && expected.contains(path.source_id.as_str())
            && !path.path_node_ids.is_empty()
            && path.minimum_latency_ns >= input.intervention.minimum_latency_ns
            && path.maximum_latency_ns <= input.intervention.maximum_latency_ns
            && path.minimum_latency_ns <= path.maximum_latency_ns
            && path.path_node_ids.len() >= 2
            && path.path_node_ids.first().map(String::as_str)
                == Some(path.intervention_target_id.as_str())
            && path.path_node_ids.last().map(String::as_str) == Some(path.source_id.as_str())
            && path.path_node_ids.iter().collect::<BTreeSet<_>>().len() == path.path_node_ids.len()
        {
            sources.insert(path.source_id.clone());
        }
    }
    sources.into_iter().collect()
}

pub fn hidden_truth_commitment_sha256(truth: &HiddenTruth) -> Result<String, String> {
    let mut uncommitted = truth.clone();
    uncommitted.truth_commitment_sha256.clear();
    let bytes = serde_json::to_vec(&uncommitted)
        .map_err(|error| format!("failed to serialize hidden truth: {error}"))?;
    Ok(sha256_hex(&bytes))
}

fn valid_candidate_report(input: &PublicInput, report: &CandidateReport) -> bool {
    if report.responsive_source_ids.len()
        != report.responsive_source_ids.iter().collect::<BTreeSet<_>>().len()
        || report.negative_control_response_source_ids.len()
            != report
                .negative_control_response_source_ids
                .iter()
                .collect::<BTreeSet<_>>()
                .len()
        || report.contrasts.len()
            != input.expected_source_ids.len() + input.negative_control_source_ids.len()
    {
        return false;
    }
    let expected_targets: BTreeSet<&str> =
        input.expected_source_ids.iter().map(String::as_str).collect();
    let expected_controls: BTreeSet<&str> = input
        .negative_control_source_ids
        .iter()
        .map(String::as_str)
        .collect();
    let mut contrasts = BTreeMap::new();
    for contrast in &report.contrasts {
        let expected_role = if expected_targets.contains(contrast.source_id.as_str()) {
            Some(ObservationRole::TargetEndpoint)
        } else if expected_controls.contains(contrast.source_id.as_str()) {
            Some(ObservationRole::NegativeControl)
        } else {
            None
        };
        if expected_role != Some(contrast.role)
            || contrasts
                .insert(contrast.source_id.as_str(), contrast)
                .is_some()
        {
            return false;
        }
        if contrast.active_responses > contrast.active_trials
            || contrast.sham_responses > contrast.sham_trials
            || contrast.active_rate_milli > 1000
            || contrast.sham_rate_milli > 1000
        {
            return false;
        }
        let expected_active_rate = rate_milli(contrast.active_responses, contrast.active_trials);
        let expected_sham_rate = rate_milli(contrast.sham_responses, contrast.sham_trials);
        let expected_responsive = endpoint_responsive(
            contrast.active_responses,
            contrast.active_trials,
            contrast.sham_responses,
            contrast.sham_trials,
            &input.intervention,
        );
        if contrast.active_rate_milli != expected_active_rate
            || contrast.sham_rate_milli != expected_sham_rate
            || contrast.responsive != expected_responsive
        {
            return false;
        }
    }
    if contrasts.len() != expected_targets.len() + expected_controls.len() {
        return false;
    }
    let expected_responders: Vec<String> = input
        .expected_source_ids
        .iter()
        .filter(|source| contrasts.get(source.as_str()).is_some_and(|item| item.responsive))
        .cloned()
        .collect();
    let expected_control_responses: Vec<String> = input
        .negative_control_source_ids
        .iter()
        .filter(|source| contrasts.get(source.as_str()).is_some_and(|item| item.responsive))
        .cloned()
        .collect();
    if report.responsive_source_ids != expected_responders
        || report.negative_control_response_source_ids != expected_control_responses
    {
        return false;
    }
    let active_count = input.trials.iter().filter(|trial| trial.arm == TrialArm::Active).count();
    let sham_count = input.trials.iter().filter(|trial| trial.arm == TrialArm::Sham).count();
    if report.tested_active_trials != active_count
        || report.tested_sham_trials != sham_count
        || report.observations_used != input.observations.len()
    {
        return false;
    }
    match report.disposition {
        CandidateDisposition::CommonCauseSupportedWithinInterventionScope => {
            report.responsive_source_ids.len() >= 2
                && report.negative_control_response_source_ids.is_empty()
        }
        CandidateDisposition::NoCommonCauseDetectedWithinTestedEnvelope => {
            report.responsive_source_ids.len() < 2
                && report.negative_control_response_source_ids.is_empty()
                && report.reasons.is_empty()
        }
        CandidateDisposition::IndeterminateInterventionCoverage
        | CandidateDisposition::ObservationOrClockMappingInvalid => true,
    }
}

fn rate_milli(responses: usize, trials: usize) -> u16 {
    if trials == 0 {
        0
    } else {
        ((responses * 1000 + trials / 2) / trials) as u16
    }
}

fn endpoint_responsive(
    active_responses: usize,
    active_trials: usize,
    sham_responses: usize,
    sham_trials: usize,
    plan: &InterventionPlan,
) -> bool {
    active_trials > 0
        && sham_trials > 0
        && active_responses * 1000
            >= usize::from(plan.response_threshold_milli) * active_trials
        && sham_responses * 1000
            <= usize::from(plan.maximum_sham_response_rate_milli) * sham_trials
}

pub fn score_candidate(
    public_bytes: &[u8],
    truth_bytes: &[u8],
    report_bytes: &[u8],
) -> EvaluationScore {
    let public_hash = sha256_hex(public_bytes);
    let report_hash = sha256_hex(report_bytes);
    let Ok(input) = serde_json::from_slice::<PublicInput>(public_bytes) else {
        return invalid_score(public_hash, report_hash, "public_input_parse_failed");
    };
    let Ok(truth) = serde_json::from_slice::<HiddenTruth>(truth_bytes) else {
        return invalid_score(public_hash, report_hash, "hidden_truth_parse_failed");
    };
    let Ok(report) = serde_json::from_slice::<CandidateReport>(report_bytes) else {
        return invalid_score(public_hash, report_hash, "candidate_report_parse_failed");
    };
    let truth_hash = truth.truth_commitment_sha256.clone();
    let truth_commitment_valid = hidden_truth_commitment_sha256(&truth)
        .is_ok_and(|expected| expected == truth.truth_commitment_sha256);
    if truth.contract != TRUTH_CONTRACT
        || !valid_digest(&truth.truth_commitment_sha256)
        || !truth_commitment_valid
        || !valid_hidden_truth(&input, &truth)
        || truth.scenario_id != input.scenario_id
        || report.report_contract != REPORT_CONTRACT
        || report.scenario_id != input.scenario_id
        || report.input_sha256 != public_hash
        || report.intervention_id != input.intervention.intervention_id
        || report.target_id != input.intervention.target_id
        || report.declared_scope != input.decision_scope
        || !valid_candidate_report(&input, &report)
    {
        return EvaluationScore {
            score_contract: SCORE_CONTRACT.to_owned(),
            scenario_id: input.scenario_id,
            public_input_sha256: public_hash,
            candidate_report_sha256: report_hash,
            truth_commitment_sha256: truth_hash,
            ground_truth_common_cause: false,
            candidate_disposition: report.disposition,
            true_positive: false,
            false_positive: false,
            true_negative: false,
            false_negative: false,
            abstained: false,
            false_independence_claim: false,
            common_cause_source_ids: Vec::new(),
            limitations: vec!["candidate_or_truth_binding_invalid".to_owned()],
        };
    }

    let truth_sources = hidden_common_cause_sources(&input, &truth);
    let ground_truth = truth_sources.len() >= 2;
    let candidate_detected =
        report.disposition == CandidateDisposition::CommonCauseSupportedWithinInterventionScope;
    let candidate_non_detection =
        report.disposition == CandidateDisposition::NoCommonCauseDetectedWithinTestedEnvelope;
    let abstained = matches!(
        report.disposition,
        CandidateDisposition::IndeterminateInterventionCoverage
            | CandidateDisposition::ObservationOrClockMappingInvalid
    );
    let false_independence = ground_truth && candidate_non_detection;

    EvaluationScore {
        score_contract: SCORE_CONTRACT.to_owned(),
        scenario_id: input.scenario_id,
        public_input_sha256: public_hash,
        candidate_report_sha256: report_hash,
        truth_commitment_sha256: truth_hash,
        ground_truth_common_cause: ground_truth,
        candidate_disposition: report.disposition,
        true_positive: ground_truth && candidate_detected,
        false_positive: !ground_truth && candidate_detected,
        true_negative: !ground_truth && candidate_non_detection,
        false_negative: ground_truth && candidate_non_detection,
        abstained,
        false_independence_claim: false_independence,
        common_cause_source_ids: truth_sources,
        limitations: vec![
            "synthetic_hidden_topology_and_controlled_intervention_only".to_owned(),
            "non_detection_is_not_physical_or_statistical_independence".to_owned(),
            "truth_commitment_shape_is_checked_but_signature_and_custody_are_not_authenticated"
                .to_owned(),
        ],
    }
}

fn invalid_report(
    input_sha256: String,
    scenario_id: String,
    intervention_id: String,
    reason: &str,
) -> CandidateReport {
    CandidateReport {
        report_contract: REPORT_CONTRACT.to_owned(),
        scenario_id,
        disposition: CandidateDisposition::ObservationOrClockMappingInvalid,
        input_sha256,
        intervention_id,
        target_id: String::new(),
        declared_scope: String::new(),
        responsive_source_ids: Vec::new(),
        negative_control_response_source_ids: Vec::new(),
        contrasts: Vec::new(),
        observations_used: 0,
        tested_active_trials: 0,
        tested_sham_trials: 0,
        reasons: vec![reason.to_owned()],
        claim_ceiling: CLAIM_CEILING.to_owned(),
    }
}

fn invalid_score(public_hash: String, report_hash: String, reason: &str) -> EvaluationScore {
    EvaluationScore {
        score_contract: SCORE_CONTRACT.to_owned(),
        scenario_id: String::new(),
        public_input_sha256: public_hash,
        candidate_report_sha256: report_hash,
        truth_commitment_sha256: String::new(),
        ground_truth_common_cause: false,
        candidate_disposition: CandidateDisposition::ObservationOrClockMappingInvalid,
        true_positive: false,
        false_positive: false,
        true_negative: false,
        false_negative: false,
        abstained: false,
        false_independence_claim: false,
        common_cause_source_ids: Vec::new(),
        limitations: vec![reason.to_owned()],
    }
}

/// A convenience helper used only to keep synthetic fixture construction deterministic.
pub fn parse_json_value(bytes: &[u8]) -> Result<Value, serde_json::Error> {
    serde_json::from_slice(bytes)
}
