//! Evidence-qualified comparison of single-agent and multi-agent coordination.
//!
//! The treatment is coordination structure and agent count. Task, model, tools,
//! benchmark, evaluator, seed and total resource budgets remain matched. This
//! module measures when coordination adds value; it does not prescribe a
//! universal agent-count or model-capability threshold.

use super::{BenchmarkRunManifest, QualificationReport, QualificationStatus};

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}

fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinationArchitecture {
    SingleAgent,
    IndependentParallel,
    Centralized,
    Decentralized,
    Hybrid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinationDisposition {
    CoordinationSupported,
    PositiveGainButEfficiencyGateExceeded,
    SingleAgentPreferred,
    Inconclusive,
    NotEvaluable,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CoordinationTaskFeatures {
    /// Preregistered degree to which the task can be decomposed into parallel subtasks.
    pub decomposability_score: f64,
    /// Preregistered degree to which later actions depend on earlier sequential state.
    pub sequential_interdependence_score: f64,
    pub partial_observability: bool,
    pub requires_environment_feedback: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CoordinationRunEvidence {
    pub manifest: BenchmarkRunManifest,
    pub architecture: CoordinationArchitecture,
    /// Same task-instance digest in the single-agent and coordinated runs.
    pub task_instance_digest: String,
    pub execution_trace_digest: Option<String>,
    pub outcome_evidence_digest: Option<String>,
    pub cost_evidence_digest: Option<String>,
    /// Normalized task quality [0, 1].
    pub task_quality: f64,
    /// Normalized unique-information preservation [0, 1].
    pub unique_information_preservation: f64,
    /// Actual measured resource use, not the declared budget.
    pub total_cost_micro_usd: u64,
    pub total_tokens: u64,
    pub wall_clock_ms: u64,
    pub inter_agent_messages: u64,
    /// Candidate error amplification relative to the single-agent reference.
    pub relative_error_amplification: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairedCoordinationGain {
    /// Multi-agent outcome minus single-agent outcome, paired by task/team unit.
    pub lower_bound: f64,
    pub point_estimate: f64,
    pub upper_bound: f64,
    pub confidence_level_bps: u16,
    pub simultaneous_intervals: bool,
    pub independent_units: u64,
    pub observation_count: u64,
    pub paired_unit_manifest_digest: Option<String>,
    pub inference_plan_digest: Option<String>,
    pub evidence_digest: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CoordinationStudyContract {
    pub expected_task_family_digest: String,
    pub confidence_level_bps: u16,
    pub minimum_independent_units: u64,
    pub maximum_cost_ratio: f64,
    pub maximum_token_ratio: f64,
    pub maximum_latency_ratio: f64,
    pub maximum_error_amplification: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CoordinationTrial {
    pub trial_id: String,
    pub task_features: CoordinationTaskFeatures,
    pub single_agent: CoordinationRunEvidence,
    pub coordinated: CoordinationRunEvidence,
    pub paired_gain: PairedCoordinationGain,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CoordinationQualificationReport {
    pub qualification: QualificationReport,
    pub disposition: CoordinationDisposition,
    pub task_quality_gain: Option<f64>,
    pub unique_information_delta: Option<f64>,
    pub cost_ratio: Option<f64>,
    pub token_ratio: Option<f64>,
    pub latency_ratio: Option<f64>,
    pub inter_agent_messages: Option<u64>,
    pub cost_limit_met: Option<bool>,
    pub token_limit_met: Option<bool>,
    pub latency_limit_met: Option<bool>,
    pub error_amplification_limit_met: Option<bool>,
    pub task_features: Option<CoordinationTaskFeatures>,
    pub rationale: Vec<String>,
}

fn validate_run(
    label: &str,
    run: &CoordinationRunEvidence,
    report: &mut QualificationReport,
) {
    let manifest = &run.manifest;
    if manifest.run_id.trim().is_empty() || manifest.system_variant.trim().is_empty() {
        fail(report, format!("{label}: run ID and system variant must be non-empty"));
    }

    for (name, digest) in [
        ("system configuration", manifest.system_config_digest.as_str()),
        ("dataset", manifest.dataset_digest.as_str()),
        ("split", manifest.split_digest.as_str()),
        ("task family", manifest.task_family_digest.as_str()),
        ("evaluation protocol", manifest.evaluation_protocol_digest.as_str()),
        ("scorer", manifest.scorer_digest.as_str()),
        ("model snapshot", manifest.model_snapshot_digest.as_str()),
        ("tokenizer", manifest.tokenizer_digest.as_str()),
        ("prompt", manifest.prompt_digest.as_str()),
        ("toolset", manifest.toolset_digest.as_str()),
        ("decoding configuration", manifest.decoding_config_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            fail(report, format!("{label}: {name} digest is malformed"));
        }
    }

    if manifest.benchmark_id.trim().is_empty()
        || manifest.benchmark_version.trim().is_empty()
        || manifest.seed_policy_id.trim().is_empty()
    {
        fail(
            report,
            format!("{label}: benchmark identity/version and seed policy must be non-empty"),
        );
    }
    if manifest.context_tokens == 0
        || manifest.max_output_tokens == 0
        || manifest.time_budget_ms == 0
        || manifest.token_budget == 0
        || manifest.compute_budget_millijoules == 0
        || manifest.memory_budget_bytes == 0
        || manifest.agent_count == 0
    {
        fail(report, format!("{label}: declared resource budgets must be non-zero"));
    }
    if !valid_digest(&run.task_instance_digest) {
        fail(report, format!("{label}: task-instance digest is malformed"));
    }
    for (name, digest) in [
        ("execution trace", run.execution_trace_digest.as_deref()),
        ("outcome evidence", run.outcome_evidence_digest.as_deref()),
        ("cost evidence", run.cost_evidence_digest.as_deref()),
    ] {
        match digest {
            Some(value) if valid_digest(value) => {}
            Some(_) => fail(report, format!("{label}: {name} digest is malformed")),
            None => indeterminate(report, format!("{label}: {name} evidence is missing")),
        }
    }

    if !run.task_quality.is_finite()
        || !(0.0..=1.0).contains(&run.task_quality)
        || !run.unique_information_preservation.is_finite()
        || !(0.0..=1.0).contains(&run.unique_information_preservation)
    {
        fail(report, format!("{label}: normalized outcome must be within [0, 1]"));
    }
    if run.total_cost_micro_usd == 0 || run.total_tokens == 0 || run.wall_clock_ms == 0 {
        fail(report, format!("{label}: measured cost, tokens and latency must be non-zero"));
    }
    if !run.relative_error_amplification.is_finite()
        || run.relative_error_amplification < 0.0
    {
        fail(report, format!("{label}: error amplification ratio is invalid"));
    }
}

fn same_budgeted_experiment(
    baseline: &BenchmarkRunManifest,
    candidate: &BenchmarkRunManifest,
) -> Vec<String> {
    let mut mismatches = Vec::new();
    macro_rules! same {
        ($field:ident, $label:literal) => {
            if baseline.$field != candidate.$field {
                mismatches.push(concat!($label, " differs").to_string());
            }
        };
    }
    same!(benchmark_id, "benchmark identity");
    same!(benchmark_version, "benchmark version");
    same!(dataset_digest, "dataset digest");
    same!(split_digest, "split digest");
    same!(task_family_digest, "task-family digest");
    same!(evaluation_protocol_digest, "evaluation protocol");
    same!(scorer_digest, "scorer");
    same!(model_snapshot_digest, "model snapshot");
    same!(tokenizer_digest, "tokenizer");
    same!(prompt_digest, "prompt");
    same!(toolset_digest, "toolset");
    same!(decoding_config_digest, "decoding configuration");
    same!(context_tokens, "context-token budget");
    same!(max_output_tokens, "maximum output-token budget");
    same!(time_budget_ms, "wall-clock budget");
    same!(token_budget, "total-token budget");
    same!(compute_budget_millijoules, "compute budget");
    same!(memory_budget_bytes, "memory budget");
    same!(seed_policy_id, "seed policy");
    same!(seed, "random seed");
    // Agent count and system configuration intentionally vary: they are the treatment.
    mismatches
}

fn empty_report(report: QualificationReport) -> CoordinationQualificationReport {
    CoordinationQualificationReport {
        qualification: report,
        disposition: CoordinationDisposition::NotEvaluable,
        task_quality_gain: None,
        unique_information_delta: None,
        cost_ratio: None,
        token_ratio: None,
        latency_ratio: None,
        inter_agent_messages: None,
        cost_limit_met: None,
        token_limit_met: None,
        latency_limit_met: None,
        error_amplification_limit_met: None,
        task_features: None,
        rationale: vec![
            "Coordination value is not evaluated when matched execution or evidence qualification fails.".into(),
        ],
    }
}

/// Qualifies one matched single-agent versus multi-agent trial.
///
/// The only declared treatment differences are coordination architecture,
/// agent count and their resulting runtime behavior. The model, task, scorer,
/// prompt, tools, decoding, seed and total declared budgets must remain fixed.
/// The capability-saturation boundary is learned from task/model data; no
/// universal fixed model-score threshold is assumed.
pub fn qualify_coordination_trial(
    contract: &CoordinationStudyContract,
    trial: &CoordinationTrial,
) -> CoordinationQualificationReport {
    let mut report = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };

    if trial.trial_id.trim().is_empty() {
        fail(&mut report, "trial ID must be non-empty");
    }
    if !valid_digest(&contract.expected_task_family_digest) {
        fail(&mut report, "expected task-family digest is malformed");
    }
    if contract.confidence_level_bps < 9_000 || contract.confidence_level_bps >= 10_000 {
        fail(&mut report, "confidence level must be between 90.00% and below 100.00%");
    }
    if contract.minimum_independent_units < 2 {
        fail(&mut report, "minimum independent units must be at least two");
    }
    for (name, value) in [
        ("maximum cost ratio", contract.maximum_cost_ratio),
        ("maximum token ratio", contract.maximum_token_ratio),
        ("maximum latency ratio", contract.maximum_latency_ratio),
        (
            "maximum error amplification",
            contract.maximum_error_amplification,
        ),
    ] {
        if !value.is_finite() || value <= 0.0 {
            fail(&mut report, format!("{name} must be finite and positive"));
        }
    }

    validate_run("single-agent", &trial.single_agent, &mut report);
    validate_run("coordinated", &trial.coordinated, &mut report);

    let baseline = &trial.single_agent;
    let coordinated = &trial.coordinated;
    if baseline.manifest.run_id == coordinated.manifest.run_id {
        fail(&mut report, "single-agent and coordinated run IDs must be distinct");
    }
    if baseline.manifest.system_variant == coordinated.manifest.system_variant {
        fail(&mut report, "single-agent and coordinated system variants must be distinct");
    }
    if baseline.manifest.task_family_digest != contract.expected_task_family_digest
        || coordinated.manifest.task_family_digest != contract.expected_task_family_digest
    {
        fail(&mut report, "run task-family digest differs from the study contract");
    }
    if baseline.architecture != CoordinationArchitecture::SingleAgent
        || baseline.manifest.agent_count != 1
    {
        fail(&mut report, "baseline must be a single-agent architecture with agent_count=1");
    }
    if coordinated.architecture == CoordinationArchitecture::SingleAgent
        || coordinated.manifest.agent_count < 2
    {
        fail(
            &mut report,
            "coordinated treatment must use a multi-agent architecture with agent_count>=2",
        );
    }
    if baseline.task_instance_digest != coordinated.task_instance_digest {
        fail(&mut report, "single-agent and coordinated runs use different task instances");
    }

    for mismatch in same_budgeted_experiment(&baseline.manifest, &coordinated.manifest) {
        fail(&mut report, format!("single-agent/multi-agent matched control: {mismatch}"));
    }

    let features = &trial.task_features;
    if !features.decomposability_score.is_finite()
        || !(0.0..=1.0).contains(&features.decomposability_score)
        || !features.sequential_interdependence_score.is_finite()
        || !(0.0..=1.0).contains(&features.sequential_interdependence_score)
    {
        fail(
            &mut report,
            "task decomposability and sequential-interdependence scores must be within [0, 1]",
        );
    }

    let gain = &trial.paired_gain;
    if !gain.lower_bound.is_finite()
        || !gain.point_estimate.is_finite()
        || !gain.upper_bound.is_finite()
        || gain.lower_bound < -1.0
        || gain.upper_bound > 1.0
        || gain.lower_bound > gain.point_estimate
        || gain.point_estimate > gain.upper_bound
    {
        fail(&mut report, "paired coordination-gain interval is invalid");
    }
    if gain.confidence_level_bps != contract.confidence_level_bps {
        fail(&mut report, "paired-gain confidence level differs from study contract");
    }
    if !gain.simultaneous_intervals {
        fail(&mut report, "paired-gain interval is not qualified for the declared comparison family");
    }
    if gain.independent_units < contract.minimum_independent_units {
        indeterminate(&mut report, "paired-gain sample has too few independent units");
    }
    if gain.observation_count < gain.independent_units {
        fail(&mut report, "paired-gain observation count is below independent-unit count");
    }
    for (name, digest) in [
        ("paired-unit manifest", gain.paired_unit_manifest_digest.as_deref()),
        ("inference plan", gain.inference_plan_digest.as_deref()),
        ("paired-gain evidence", gain.evidence_digest.as_deref()),
    ] {
        match digest {
            Some(value) if valid_digest(value) => {}
            Some(_) => fail(&mut report, format!("{name} digest is malformed")),
            None => indeterminate(&mut report, format!("{name} evidence is missing")),
        }
    }

    if (gain.point_estimate - (coordinated.task_quality - baseline.task_quality)).abs() > 1e-6 {
        fail(
            &mut report,
            "paired-gain point estimate does not equal coordinated minus single-agent task quality",
        );
    }

    if report.status != QualificationStatus::Pass {
        return empty_report(report);
    }

    let cost_ratio = coordinated.total_cost_micro_usd as f64
        / baseline.total_cost_micro_usd as f64;
    let token_ratio = coordinated.total_tokens as f64 / baseline.total_tokens as f64;
    let latency_ratio = coordinated.wall_clock_ms as f64 / baseline.wall_clock_ms as f64;
    let cost_limit_met = cost_ratio <= contract.maximum_cost_ratio;
    let token_limit_met = token_ratio <= contract.maximum_token_ratio;
    let latency_limit_met = latency_ratio <= contract.maximum_latency_ratio;
    let error_limit_met =
        coordinated.relative_error_amplification <= contract.maximum_error_amplification;

    let disposition = if gain.upper_bound <= 0.0 {
        CoordinationDisposition::SingleAgentPreferred
    } else if gain.lower_bound <= 0.0 {
        CoordinationDisposition::Inconclusive
    } else if !cost_limit_met || !token_limit_met || !latency_limit_met || !error_limit_met {
        CoordinationDisposition::PositiveGainButEfficiencyGateExceeded
    } else {
        CoordinationDisposition::CoordinationSupported
    };

    CoordinationQualificationReport {
        qualification: report,
        disposition,
        task_quality_gain: Some(gain.point_estimate),
        unique_information_delta: Some(
            coordinated.unique_information_preservation
                - baseline.unique_information_preservation,
        ),
        cost_ratio: Some(cost_ratio),
        token_ratio: Some(token_ratio),
        latency_ratio: Some(latency_ratio),
        inter_agent_messages: Some(coordinated.inter_agent_messages),
        cost_limit_met: Some(cost_limit_met),
        token_limit_met: Some(token_limit_met),
        latency_limit_met: Some(latency_limit_met),
        error_amplification_limit_met: Some(error_limit_met),
        task_features: Some(features.clone()),
        rationale: vec![
            "Architecture selection is task/model-specific; no universal agent-count or capability threshold is assumed.".into(),
            "The single-agent baseline is retained as the counterfactual reference under matched task, model, tool, evaluator, seed and total budgets.".into(),
            "Positive task-quality gain does not establish a coordination recommendation when cost, token, latency or error-amplification limits are exceeded.".into(),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn manifest(run_id: &str, agents: u32, variant: &str) -> BenchmarkRunManifest {
        BenchmarkRunManifest {
            run_id: run_id.into(),
            system_variant: variant.into(),
            system_config_digest: digest(if agents == 1 { 'a' } else { '9' }),
            benchmark_id: "coordination-value".into(),
            benchmark_version: "0.1".into(),
            dataset_digest: digest('b'),
            split_digest: digest('c'),
            task_family_digest: digest('d'),
            evaluation_protocol_digest: digest('e'),
            scorer_digest: digest('f'),
            model_snapshot_digest: digest('1'),
            tokenizer_digest: digest('2'),
            prompt_digest: digest('3'),
            toolset_digest: digest('4'),
            decoding_config_digest: digest('5'),
            context_tokens: 4096,
            max_output_tokens: 1024,
            time_budget_ms: 30000,
            token_budget: 8192,
            compute_budget_millijoules: 100000,
            agent_count: agents,
            memory_budget_bytes: 100000000,
            seed_policy_id: "paired-seed-v1".into(),
            seed: 17,
        }
    }

    fn contract() -> CoordinationStudyContract {
        CoordinationStudyContract {
            expected_task_family_digest: digest('d'),
            confidence_level_bps: 9500,
            minimum_independent_units: 20,
            maximum_cost_ratio: 1.5,
            maximum_token_ratio: 1.5,
            maximum_latency_ratio: 1.5,
            maximum_error_amplification: 1.2,
        }
    }

    fn run_evidence(single: bool) -> CoordinationRunEvidence {
        let baseline = single;
        CoordinationRunEvidence {
            manifest: if baseline {
                manifest("single-run", 1, "single")
            } else {
                manifest("multi-run", 3, "centralized")
            },
            architecture: if baseline {
                CoordinationArchitecture::SingleAgent
            } else {
                CoordinationArchitecture::Centralized
            },
            task_instance_digest: digest('6'),
            execution_trace_digest: Some(digest('7')),
            outcome_evidence_digest: Some(digest('8')),
            cost_evidence_digest: Some(digest('9')),
            task_quality: if baseline { 0.70 } else { 0.78 },
            unique_information_preservation: if baseline { 0.55 } else { 0.72 },
            total_cost_micro_usd: if baseline { 100 } else { 120 },
            total_tokens: if baseline { 1000 } else { 1200 },
            wall_clock_ms: if baseline { 10000 } else { 12000 },
            inter_agent_messages: if baseline { 0 } else { 45 },
            relative_error_amplification: if baseline { 1.0 } else { 0.9 },
        }
    }

    fn trial() -> CoordinationTrial {
        CoordinationTrial {
            trial_id: "coordination-trial-001".into(),
            task_features: CoordinationTaskFeatures {
                decomposability_score: 0.8,
                sequential_interdependence_score: 0.2,
                partial_observability: true,
                requires_environment_feedback: true,
            },
            single_agent: run_evidence(true),
            coordinated: run_evidence(false),
            paired_gain: PairedCoordinationGain {
                lower_bound: 0.03,
                point_estimate: 0.08,
                upper_bound: 0.13,
                confidence_level_bps: 9500,
                simultaneous_intervals: true,
                independent_units: 50,
                observation_count: 500,
                paired_unit_manifest_digest: Some(digest('a')),
                inference_plan_digest: Some(digest('b')),
                evidence_digest: Some(digest('c')),
            },
        }
    }

    #[test]
    fn matched_coordination_gain_within_efficiency_limits_is_supported() {
        let result = qualify_coordination_trial(&contract(), &trial());
        assert_eq!(result.qualification.status, QualificationStatus::Pass);
        assert_eq!(result.disposition, CoordinationDisposition::CoordinationSupported);
        assert_eq!(result.cost_ratio, Some(1.2));
        assert_eq!(result.inter_agent_messages, Some(45));
    }

    #[test]
    fn negative_gain_prefers_single_agent() {
        let mut case = trial();
        case.paired_gain.lower_bound = -0.12;
        case.paired_gain.point_estimate = -0.08;
        case.paired_gain.upper_bound = -0.03;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.disposition, CoordinationDisposition::SingleAgentPreferred);
    }

    #[test]
    fn interval_crossing_zero_is_inconclusive() {
        let mut case = trial();
        case.paired_gain.lower_bound = -0.02;
        case.paired_gain.point_estimate = 0.03;
        case.paired_gain.upper_bound = 0.07;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.disposition, CoordinationDisposition::Inconclusive);
    }

    #[test]
    fn positive_quality_gain_does_not_compensate_for_excess_cost() {
        let mut case = trial();
        case.coordinated.total_cost_micro_usd = 200;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(
            result.disposition,
            CoordinationDisposition::PositiveGainButEfficiencyGateExceeded
        );
        assert_eq!(result.cost_limit_met, Some(false));
    }

    #[test]
    fn positive_quality_gain_does_not_compensate_for_error_amplification() {
        let mut case = trial();
        case.coordinated.relative_error_amplification = 1.5;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(
            result.disposition,
            CoordinationDisposition::PositiveGainButEfficiencyGateExceeded
        );
        assert_eq!(result.error_amplification_limit_met, Some(false));
    }


    #[test]
    fn inconsistent_paired_gain_point_estimate_fails() {
        let mut case = trial();
        case.paired_gain.point_estimate = 0.20;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.disposition, CoordinationDisposition::NotEvaluable);
    }

    #[test]
    fn paired_gain_outside_normalized_difference_range_fails() {
        let mut case = trial();
        case.paired_gain.upper_bound = 1.1;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn identical_system_variant_names_fail() {
        let mut case = trial();
        case.coordinated.manifest.system_variant = "single".into();
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn model_mismatch_blocks_coordination_comparison() {
        let mut case = trial();
        case.coordinated.manifest.model_snapshot_digest = digest('8');
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
        assert_eq!(result.disposition, CoordinationDisposition::NotEvaluable);
    }

    #[test]
    fn compute_budget_mismatch_blocks_comparison() {
        let mut case = trial();
        case.coordinated.manifest.compute_budget_millijoules += 1;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn agent_count_is_a_treatment_variable_not_a_matched_budget() {
        let result = qualify_coordination_trial(&contract(), &trial());
        assert_eq!(result.qualification.status, QualificationStatus::Pass);
    }

    #[test]
    fn different_task_instance_blocks_comparison() {
        let mut case = trial();
        case.coordinated.task_instance_digest = digest('7');
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn invalid_baseline_architecture_fails() {
        let mut case = trial();
        case.single_agent.manifest.agent_count = 2;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn coordinated_treatment_requires_multiple_agents() {
        let mut case = trial();
        case.coordinated.manifest.agent_count = 1;
        case.coordinated.architecture = CoordinationArchitecture::SingleAgent;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_execution_evidence_is_indeterminate() {
        let mut case = trial();
        case.coordinated.execution_trace_digest = None;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Indeterminate);
        assert_eq!(result.disposition, CoordinationDisposition::NotEvaluable);
    }

    #[test]
    fn too_few_independent_units_is_indeterminate() {
        let mut case = trial();
        case.paired_gain.independent_units = 5;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn non_simultaneous_gain_interval_fails() {
        let mut case = trial();
        case.paired_gain.simultaneous_intervals = false;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_task_family_digest_is_not_evaluable() {
        let mut case = trial();
        case.coordinated.manifest.task_family_digest = digest('9');
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn invalid_task_structure_scores_fail() {
        let mut case = trial();
        case.task_features.sequential_interdependence_score = f64::NAN;
        let result = qualify_coordination_trial(&contract(), &case);
        assert_eq!(result.qualification.status, QualificationStatus::Fail);
    }
}
