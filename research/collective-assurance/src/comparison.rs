//! Matched-comparison qualification for benchmark and SOTA claims.
//!
//! This module does not score a benchmark. It determines whether two run
//! manifests support a matched leaderboard comparison or only a contextual
//! evidence comparison. Missing or mismatched controls never become a win.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ComparisonIntent {
    MatchedSystem,
    CrossModelContext,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ComparisonStatus {
    Comparable,
    ContextOnly,
    NotComparable,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BenchmarkRunManifest {
    pub run_id: String,
    pub system_variant: String,
    pub system_config_digest: String,
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub dataset_digest: String,
    pub split_digest: String,
    pub task_family_digest: String,
    pub evaluation_protocol_digest: String,
    pub scorer_digest: String,
    pub model_snapshot_digest: String,
    pub tokenizer_digest: String,
    pub prompt_digest: String,
    pub toolset_digest: String,
    pub decoding_config_digest: String,
    pub context_tokens: u64,
    pub max_output_tokens: u64,
    pub time_budget_ms: u64,
    pub token_budget: u64,
    pub compute_budget_millijoules: u64,
    pub agent_count: u32,
    pub memory_budget_bytes: u64,
    pub seed_policy_id: String,
    pub seed: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComparisonReport {
    pub status: ComparisonStatus,
    pub hard_mismatches: Vec<String>,
    pub contextual_differences: Vec<String>,
    pub rationale: Vec<String>,
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_manifest(label: &str, manifest: &BenchmarkRunManifest, errors: &mut Vec<String>) {
    if manifest.run_id.trim().is_empty() {
        errors.push(format!("{label}: run_id is empty"));
    }
    if manifest.system_variant.trim().is_empty() {
        errors.push(format!("{label}: system_variant is empty"));
    }
    if manifest.benchmark_id.trim().is_empty() || manifest.benchmark_version.trim().is_empty() {
        errors.push(format!("{label}: benchmark identity/version is empty"));
    }
    if manifest.seed_policy_id.trim().is_empty() {
        errors.push(format!("{label}: seed policy is empty"));
    }
    for (name, digest) in [
        ("system config", manifest.system_config_digest.as_str()),
        ("dataset", manifest.dataset_digest.as_str()),
        ("split", manifest.split_digest.as_str()),
        ("task family", manifest.task_family_digest.as_str()),
        ("evaluation protocol", manifest.evaluation_protocol_digest.as_str()),
        ("scorer", manifest.scorer_digest.as_str()),
        ("model snapshot", manifest.model_snapshot_digest.as_str()),
        ("tokenizer", manifest.tokenizer_digest.as_str()),
        ("prompt", manifest.prompt_digest.as_str()),
        ("toolset", manifest.toolset_digest.as_str()),
        ("decoding config", manifest.decoding_config_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            errors.push(format!("{label}: {name} digest is not lowercase SHA-256 hex"));
        }
    }
    if manifest.context_tokens == 0
        || manifest.max_output_tokens == 0
        || manifest.time_budget_ms == 0
        || manifest.token_budget == 0
        || manifest.compute_budget_millijoules == 0
        || manifest.agent_count == 0
        || manifest.memory_budget_bytes == 0
    {
        errors.push(format!("{label}: context, output, time, token, compute, agent and memory budgets must be non-zero"));
    }
}

pub fn qualify_benchmark_comparison(
    baseline: &BenchmarkRunManifest,
    candidate: &BenchmarkRunManifest,
    intent: ComparisonIntent,
) -> ComparisonReport {
    let mut hard_mismatches = Vec::new();
    let mut contextual_differences = Vec::new();
    let mut rationale = Vec::new();

    validate_manifest("baseline", baseline, &mut hard_mismatches);
    validate_manifest("candidate", candidate, &mut hard_mismatches);

    if baseline.run_id == candidate.run_id {
        hard_mismatches.push("baseline and candidate run IDs must be distinct".into());
    }

    macro_rules! require_equal {
        ($field:ident, $label:literal) => {
            if baseline.$field != candidate.$field {
                hard_mismatches.push(format!(concat!($label, " differs")));
            }
        };
    }

    require_equal!(benchmark_id, "benchmark identity");
    require_equal!(benchmark_version, "benchmark version");
    require_equal!(dataset_digest, "dataset digest");
    require_equal!(split_digest, "split digest");
    require_equal!(task_family_digest, "task family digest");
    require_equal!(evaluation_protocol_digest, "evaluation protocol digest");
    require_equal!(scorer_digest, "scorer digest");
    require_equal!(seed_policy_id, "seed policy");
    if baseline.seed != candidate.seed {
        hard_mismatches.push("random seed differs".into());
    }
    require_equal!(context_tokens, "context token budget");
    require_equal!(max_output_tokens, "maximum output token budget");
    require_equal!(time_budget_ms, "time budget");
    require_equal!(token_budget, "token budget");
    require_equal!(compute_budget_millijoules, "compute/energy budget");
    require_equal!(agent_count, "agent count");
    require_equal!(memory_budget_bytes, "memory budget");

    let contextual_fields = [
        ("model snapshot", baseline.model_snapshot_digest.as_str(), candidate.model_snapshot_digest.as_str()),
        ("tokenizer", baseline.tokenizer_digest.as_str(), candidate.tokenizer_digest.as_str()),
        ("prompt", baseline.prompt_digest.as_str(), candidate.prompt_digest.as_str()),
        ("toolset", baseline.toolset_digest.as_str(), candidate.toolset_digest.as_str()),
        ("decoding configuration", baseline.decoding_config_digest.as_str(), candidate.decoding_config_digest.as_str()),
    ];
    for (label, left, right) in contextual_fields {
        if left != right {
            match intent {
                ComparisonIntent::MatchedSystem => {
                    hard_mismatches.push(format!("{label} differs in a matched-system comparison"));
                }
                ComparisonIntent::CrossModelContext => {
                    contextual_differences.push(format!("{label} differs"));
                }
            }
        }
    }

    if intent == ComparisonIntent::MatchedSystem {
        rationale.push("Matched-system comparisons hold model, tokenizer, prompt, tools and decoding constant; only the declared system variant/configuration is allowed to differ.".into());
    } else {
        rationale.push("Cross-model/system results are contextual evidence, not a matched leaderboard head-to-head.".into());
    }
    rationale.push("Benchmark identity, version, dataset, split, task family, evaluator, scorer, budgets, agent count, memory budget, seed policy and seed must match.".into());

    let status = if !hard_mismatches.is_empty() {
        ComparisonStatus::NotComparable
    } else if !contextual_differences.is_empty() || intent == ComparisonIntent::CrossModelContext {
        ComparisonStatus::ContextOnly
    } else {
        ComparisonStatus::Comparable
    };

    ComparisonReport {
        status,
        hard_mismatches,
        contextual_differences,
        rationale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: char) -> String {
        std::iter::repeat_n(byte, 64).collect()
    }

    fn manifest(run: &str, variant: &str) -> BenchmarkRunManifest {
        BenchmarkRunManifest {
            run_id: run.into(),
            system_variant: variant.into(),
            system_config_digest: digest(if variant == "baseline" { 'a' } else { 'b' }),
            benchmark_id: "groupassistbench".into(),
            benchmark_version: "2026.11".into(),
            dataset_digest: digest('c'),
            split_digest: digest('d'),
            task_family_digest: digest('e'),
            evaluation_protocol_digest: digest('f'),
            scorer_digest: digest('1'),
            model_snapshot_digest: digest('2'),
            tokenizer_digest: digest('3'),
            prompt_digest: digest('4'),
            toolset_digest: digest('5'),
            decoding_config_digest: digest('6'),
            context_tokens: 128_000,
            max_output_tokens: 4_096,
            time_budget_ms: 60_000,
            token_budget: 32_000,
            compute_budget_millijoules: 250_000,
            agent_count: 1,
            memory_budget_bytes: 1_000_000_000,
            seed_policy_id: "paired-seeds-v1".into(),
            seed: 42,
        }
    }

    #[test]
    fn matched_system_with_identical_controls_is_comparable() {
        let report = qualify_benchmark_comparison(
            &manifest("baseline-1", "baseline"),
            &manifest("candidate-1", "candidate"),
            ComparisonIntent::MatchedSystem,
        );
        assert_eq!(report.status, ComparisonStatus::Comparable, "{:?}", report.hard_mismatches);
    }

    #[test]
    fn model_snapshot_difference_blocks_matched_system_claim() {
        let baseline = manifest("baseline-1", "baseline");
        let mut candidate = manifest("candidate-1", "candidate");
        candidate.model_snapshot_digest = digest('9');
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::MatchedSystem);
        assert_eq!(report.status, ComparisonStatus::NotComparable);
    }

    #[test]
    fn model_snapshot_difference_is_context_only_when_declared() {
        let baseline = manifest("baseline-1", "baseline");
        let mut candidate = manifest("candidate-1", "candidate");
        candidate.model_snapshot_digest = digest('9');
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::CrossModelContext);
        assert_eq!(report.status, ComparisonStatus::ContextOnly);
        assert!(!report.contextual_differences.is_empty());
    }

    #[test]
    fn dataset_or_evaluator_mismatch_is_never_comparable() {
        let baseline = manifest("baseline-1", "baseline");
        let mut candidate = manifest("candidate-1", "candidate");
        candidate.dataset_digest = digest('8');
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::CrossModelContext);
        assert_eq!(report.status, ComparisonStatus::NotComparable);
    }

    #[test]
    fn budget_mismatch_is_never_comparable() {
        let baseline = manifest("baseline-1", "baseline");
        let mut candidate = manifest("candidate-1", "candidate");
        candidate.time_budget_ms += 1;
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::MatchedSystem);
        assert_eq!(report.status, ComparisonStatus::NotComparable);
    }

    #[test]
    fn seed_mismatch_is_never_comparable() {
        let baseline = manifest("baseline-1", "baseline");
        let mut candidate = manifest("candidate-1", "candidate");
        candidate.seed += 1;
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::CrossModelContext);
        assert_eq!(report.status, ComparisonStatus::NotComparable);
    }

    #[test]
    fn malformed_manifest_fails_closed() {
        let mut baseline = manifest("baseline-1", "baseline");
        baseline.scorer_digest = "not-a-digest".into();
        let candidate = manifest("candidate-1", "candidate");
        let report = qualify_benchmark_comparison(&baseline, &candidate, ComparisonIntent::MatchedSystem);
        assert_eq!(report.status, ComparisonStatus::NotComparable);
    }

    #[test]
    fn cross_model_mode_is_never_a_matched_leaderboard_claim() {
        let report = qualify_benchmark_comparison(
            &manifest("baseline-1", "baseline"),
            &manifest("candidate-1", "candidate"),
            ComparisonIntent::CrossModelContext,
        );
        assert_eq!(report.status, ComparisonStatus::ContextOnly);
    }
}
