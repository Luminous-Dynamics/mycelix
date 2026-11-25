//! Benchmark qualification-of-qualification contract.
//!
//! This module evaluates the evidence package for a benchmark itself before
//! benchmark scores are compared. A structural PASS is bounded by the supplied
//! registry and evidence references; it is not a certification authority.

use super::{QualificationReport, QualificationStatus};

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BenchmarkClaimProfile {
    SyntheticContract,
    AgentSystem,
    HumanTeam,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScoringMethod {
    DeterministicOracle,
    IndependentHumanAdjudication,
    LlmJudge,
    Hybrid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvidenceGate {
    Pass,
    Fail,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BenchmarkQualificationManifest {
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub benchmark_owner_id: String,
    pub independent_evaluator_id: String,
    pub claim_profile: BenchmarkClaimProfile,
    pub scoring_method: ScoringMethod,
    pub stochastic_evaluation: bool,
    pub repeated_run_count: u32,
    pub distinct_seed_count: u32,

    pub task_catalog_digest: Option<String>,
    pub dataset_digest: Option<String>,
    pub split_manifest_digest: Option<String>,
    pub hidden_split_commitment_digest: Option<String>,
    pub license_and_data_rights_digest: Option<String>,
    pub oracle_specification_digest: Option<String>,
    pub scoring_specification_digest: Option<String>,
    pub scorer_implementation_digest: Option<String>,
    pub evaluator_independence_receipt_digest: Option<String>,
    pub leakage_audit_digest: Option<String>,
    pub holdout_separation_evidence_digest: Option<String>,
    pub reproducibility_report_digest: Option<String>,
    pub cost_telemetry_digest: Option<String>,

    pub leakage_audit: EvidenceGate,
    pub oracle_independence: EvidenceGate,
    pub scorer_validation: EvidenceGate,
    pub holdout_separation: EvidenceGate,
    pub replay_reproducibility: EvidenceGate,

    pub deterministic_replay_evidence_digest: Option<String>,
    pub judge_calibration_digest: Option<String>,
    pub human_adjudication_digest: Option<String>,
    pub inter_rater_reliability_digest: Option<String>,

    pub property_test_coverage_digest: Option<String>,

    pub agent_environment_image_digest: Option<String>,
    pub agent_attack_surface_manifest_digest: Option<String>,
    pub effect_sink_coverage_digest: Option<String>,

    pub human_preregistration_digest: Option<String>,
    pub human_ethics_approval_digest: Option<String>,
    pub accessible_consent_protocol_digest: Option<String>,
    pub withdrawal_and_data_minimization_digest: Option<String>,
    pub cluster_analysis_plan_digest: Option<String>,
}

fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}

fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

fn require_digest(report: &mut QualificationReport, label: &str, value: Option<&str>) {
    match value {
        Some(digest) if valid_digest(digest) => {}
        Some(_) => fail(report, format!("{label} digest is malformed")),
        None => indeterminate(report, format!("{label} evidence is missing")),
    }
}

fn require_gate(report: &mut QualificationReport, label: &str, gate: EvidenceGate) {
    match gate {
        EvidenceGate::Pass => {}
        EvidenceGate::Fail => fail(report, format!("{label} gate failed")),
        EvidenceGate::Unknown => indeterminate(report, format!("{label} gate is unknown")),
    }
}

fn has_valid_optional_digest(value: Option<&str>) -> bool {
    value.is_some_and(valid_digest)
}

/// Qualifies a benchmark's evidence package for a declared claim profile.
///
/// PASS means the declared validity, rights, evaluator, reproducibility and
/// claim-specific evidence gates are structurally satisfied. It does not mean
/// any candidate system performed well or that the benchmark is universally
/// valid for uses outside the declared profile.
pub fn qualify_benchmark_definition(
    manifest: &BenchmarkQualificationManifest,
) -> QualificationReport {
    let mut report = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };

    if manifest.benchmark_id.trim().is_empty()
        || manifest.benchmark_version.trim().is_empty()
        || manifest.benchmark_owner_id.trim().is_empty()
        || manifest.independent_evaluator_id.trim().is_empty()
    {
        fail(
            &mut report,
            "benchmark, version, owner and independent evaluator identifiers must be non-empty",
        );
    }
    if manifest.benchmark_owner_id == manifest.independent_evaluator_id {
        fail(
            &mut report,
            "benchmark owner cannot serve as the independent evaluator",
        );
    }
    if manifest.repeated_run_count < 3 {
        indeterminate(
            &mut report,
            "fewer than three repeated runs; reproducibility evidence is insufficient for this profile",
        );
    }
    if manifest.stochastic_evaluation && manifest.distinct_seed_count < 3 {
        indeterminate(
            &mut report,
            "stochastic evaluation has fewer than three distinct seeds",
        );
    }
    if !manifest.stochastic_evaluation && manifest.distinct_seed_count != 1 {
        fail(
            &mut report,
            "deterministic evaluation must declare exactly one canonical seed policy",
        );
    }

    for (label, value) in [
        ("task catalog", manifest.task_catalog_digest.as_deref()),
        ("dataset", manifest.dataset_digest.as_deref()),
        ("split manifest", manifest.split_manifest_digest.as_deref()),
        (
            "hidden split commitment",
            manifest.hidden_split_commitment_digest.as_deref(),
        ),
        (
            "license and data-rights",
            manifest.license_and_data_rights_digest.as_deref(),
        ),
        (
            "oracle specification",
            manifest.oracle_specification_digest.as_deref(),
        ),
        (
            "scoring specification",
            manifest.scoring_specification_digest.as_deref(),
        ),
        (
            "scorer implementation",
            manifest.scorer_implementation_digest.as_deref(),
        ),
        (
            "evaluator independence receipt",
            manifest.evaluator_independence_receipt_digest.as_deref(),
        ),
        ("leakage audit", manifest.leakage_audit_digest.as_deref()),
        (
            "holdout separation",
            manifest.holdout_separation_evidence_digest.as_deref(),
        ),
        (
            "reproducibility report",
            manifest.reproducibility_report_digest.as_deref(),
        ),
        ("cost telemetry", manifest.cost_telemetry_digest.as_deref()),
    ] {
        require_digest(&mut report, label, value);
    }

    for (label, gate) in [
        ("task/dataset leakage audit", manifest.leakage_audit),
        ("oracle independence", manifest.oracle_independence),
        ("scorer validation", manifest.scorer_validation),
        ("sealed holdout separation", manifest.holdout_separation),
        ("repeated-run replay", manifest.replay_reproducibility),
    ] {
        require_gate(&mut report, label, gate);
    }

    match manifest.scoring_method {
        ScoringMethod::DeterministicOracle => {
            require_digest(
                &mut report,
                "deterministic oracle replay",
                manifest.deterministic_replay_evidence_digest.as_deref(),
            );
        }
        ScoringMethod::IndependentHumanAdjudication => {
            require_digest(
                &mut report,
                "independent inter-rater reliability",
                manifest.inter_rater_reliability_digest.as_deref(),
            );
        }
        ScoringMethod::LlmJudge | ScoringMethod::Hybrid => {
            require_digest(
                &mut report,
                "LLM-judge calibration",
                manifest.judge_calibration_digest.as_deref(),
            );
            require_digest(
                &mut report,
                "independent human adjudication sample",
                manifest.human_adjudication_digest.as_deref(),
            );
        }
    }

    match manifest.claim_profile {
        BenchmarkClaimProfile::SyntheticContract => {
            require_digest(
                &mut report,
                "property-test coverage",
                manifest.property_test_coverage_digest.as_deref(),
            );
        }
        BenchmarkClaimProfile::AgentSystem => {
            for (label, value) in [
                (
                    "agent environment image",
                    manifest.agent_environment_image_digest.as_deref(),
                ),
                (
                    "agent attack-surface manifest",
                    manifest.agent_attack_surface_manifest_digest.as_deref(),
                ),
                (
                    "effect-sink coverage",
                    manifest.effect_sink_coverage_digest.as_deref(),
                ),
            ] {
                require_digest(&mut report, label, value);
            }
        }
        BenchmarkClaimProfile::HumanTeam => {
            for (label, value) in [
                (
                    "human-study preregistration",
                    manifest.human_preregistration_digest.as_deref(),
                ),
                (
                    "human ethics approval",
                    manifest.human_ethics_approval_digest.as_deref(),
                ),
                (
                    "accessible consent protocol",
                    manifest.accessible_consent_protocol_digest.as_deref(),
                ),
                (
                    "withdrawal and data-minimization plan",
                    manifest.withdrawal_and_data_minimization_digest.as_deref(),
                ),
                (
                    "cluster-level analysis plan",
                    manifest.cluster_analysis_plan_digest.as_deref(),
                ),
            ] {
                require_digest(&mut report, label, value);
            }
        }
    }

    for (label, value) in [
        ("deterministic replay", manifest.deterministic_replay_evidence_digest.as_deref()),
        ("judge calibration", manifest.judge_calibration_digest.as_deref()),
        ("human adjudication", manifest.human_adjudication_digest.as_deref()),
        ("inter-rater reliability", manifest.inter_rater_reliability_digest.as_deref()),
        ("property-test coverage", manifest.property_test_coverage_digest.as_deref()),
        ("agent environment image", manifest.agent_environment_image_digest.as_deref()),
        ("agent attack-surface manifest", manifest.agent_attack_surface_manifest_digest.as_deref()),
        ("effect-sink coverage", manifest.effect_sink_coverage_digest.as_deref()),
        ("human preregistration", manifest.human_preregistration_digest.as_deref()),
        ("human ethics approval", manifest.human_ethics_approval_digest.as_deref()),
        ("accessible consent protocol", manifest.accessible_consent_protocol_digest.as_deref()),
        ("withdrawal/data-minimization plan", manifest.withdrawal_and_data_minimization_digest.as_deref()),
        ("cluster analysis plan", manifest.cluster_analysis_plan_digest.as_deref()),
    ] {
        if value.is_some_and(|digest| !valid_digest(digest)) {
            fail(&mut report, format!("{label} digest is malformed"));
        }
    }

    // Evidence supplied for a different profile is not silently used to
    // upgrade a claim. The selected profile determines the qualification scope.
    if manifest.claim_profile != BenchmarkClaimProfile::AgentSystem
        && (has_valid_optional_digest(manifest.agent_environment_image_digest.as_deref())
            || has_valid_optional_digest(
                manifest.agent_attack_surface_manifest_digest.as_deref(),
            )
            || has_valid_optional_digest(manifest.effect_sink_coverage_digest.as_deref()))
    {
        indeterminate(
            &mut report,
            "agent-system evidence is present outside the declared agent-system profile",
        );
    }
    if manifest.claim_profile != BenchmarkClaimProfile::HumanTeam
        && (has_valid_optional_digest(manifest.human_preregistration_digest.as_deref())
            || has_valid_optional_digest(manifest.human_ethics_approval_digest.as_deref())
            || has_valid_optional_digest(manifest.accessible_consent_protocol_digest.as_deref())
            || has_valid_optional_digest(
                manifest.withdrawal_and_data_minimization_digest.as_deref(),
            )
            || has_valid_optional_digest(manifest.cluster_analysis_plan_digest.as_deref()))
    {
        indeterminate(
            &mut report,
            "human-study evidence is present outside the declared human-team profile",
        );
    }

    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn valid_manifest() -> BenchmarkQualificationManifest {
        BenchmarkQualificationManifest {
            benchmark_id: "synthetic-qualification-suite".into(),
            benchmark_version: "0.1".into(),
            benchmark_owner_id: "benchmark-owner".into(),
            independent_evaluator_id: "independent-evaluator".into(),
            claim_profile: BenchmarkClaimProfile::SyntheticContract,
            scoring_method: ScoringMethod::DeterministicOracle,
            stochastic_evaluation: false,
            repeated_run_count: 3,
            distinct_seed_count: 1,
            task_catalog_digest: Some(digest('a')),
            dataset_digest: Some(digest('b')),
            split_manifest_digest: Some(digest('c')),
            hidden_split_commitment_digest: Some(digest('d')),
            license_and_data_rights_digest: Some(digest('e')),
            oracle_specification_digest: Some(digest('f')),
            scoring_specification_digest: Some(digest('1')),
            scorer_implementation_digest: Some(digest('2')),
            evaluator_independence_receipt_digest: Some(digest('3')),
            leakage_audit_digest: Some(digest('4')),
            holdout_separation_evidence_digest: Some(digest('5')),
            reproducibility_report_digest: Some(digest('6')),
            cost_telemetry_digest: Some(digest('7')),
            leakage_audit: EvidenceGate::Pass,
            oracle_independence: EvidenceGate::Pass,
            scorer_validation: EvidenceGate::Pass,
            holdout_separation: EvidenceGate::Pass,
            replay_reproducibility: EvidenceGate::Pass,
            deterministic_replay_evidence_digest: Some(digest('8')),
            judge_calibration_digest: None,
            human_adjudication_digest: None,
            inter_rater_reliability_digest: None,
            property_test_coverage_digest: Some(digest('9')),
            agent_environment_image_digest: None,
            agent_attack_surface_manifest_digest: None,
            effect_sink_coverage_digest: None,
            human_preregistration_digest: None,
            human_ethics_approval_digest: None,
            accessible_consent_protocol_digest: None,
            withdrawal_and_data_minimization_digest: None,
            cluster_analysis_plan_digest: None,
        }
    }

    #[test]
    fn complete_synthetic_benchmark_evidence_passes() {
        let report = qualify_benchmark_definition(&valid_manifest());
        assert_eq!(report.status, QualificationStatus::Pass);
        assert!(report.reasons.is_empty());
    }

    #[test]
    fn benchmark_owner_cannot_self_evaluate() {
        let mut manifest = valid_manifest();
        manifest.independent_evaluator_id = manifest.benchmark_owner_id.clone();
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn missing_leakage_audit_is_indeterminate() {
        let mut manifest = valid_manifest();
        manifest.leakage_audit_digest = None;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn failed_leakage_audit_fails_qualification() {
        let mut manifest = valid_manifest();
        manifest.leakage_audit = EvidenceGate::Fail;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn llm_judge_requires_calibration_and_human_adjudication() {
        let mut manifest = valid_manifest();
        manifest.scoring_method = ScoringMethod::LlmJudge;
        manifest.deterministic_replay_evidence_digest = None;
        manifest.judge_calibration_digest = Some(digest('a'));
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn human_team_profile_requires_ethics_and_consent_evidence() {
        let mut manifest = valid_manifest();
        manifest.claim_profile = BenchmarkClaimProfile::HumanTeam;
        manifest.human_preregistration_digest = Some(digest('a'));
        manifest.human_ethics_approval_digest = None;
        manifest.accessible_consent_protocol_digest = Some(digest('b'));
        manifest.withdrawal_and_data_minimization_digest = Some(digest('c'));
        manifest.cluster_analysis_plan_digest = Some(digest('d'));
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn agent_system_profile_requires_effect_sink_coverage() {
        let mut manifest = valid_manifest();
        manifest.claim_profile = BenchmarkClaimProfile::AgentSystem;
        manifest.property_test_coverage_digest = None;
        manifest.agent_environment_image_digest = Some(digest('a'));
        manifest.agent_attack_surface_manifest_digest = Some(digest('b'));
        manifest.effect_sink_coverage_digest = None;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn stochastic_benchmark_requires_multiple_seeds() {
        let mut manifest = valid_manifest();
        manifest.stochastic_evaluation = true;
        manifest.distinct_seed_count = 1;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn insufficient_repetitions_are_indeterminate() {
        let mut manifest = valid_manifest();
        manifest.repeated_run_count = 1;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn invalid_digest_fails() {
        let mut manifest = valid_manifest();
        manifest.cost_telemetry_digest = Some("ABC".into());
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Fail);
    }

    #[test]
    fn unknown_holdout_separation_never_passes() {
        let mut manifest = valid_manifest();
        manifest.holdout_separation = EvidenceGate::Unknown;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn human_adjudication_requires_inter_rater_evidence() {
        let mut manifest = valid_manifest();
        manifest.scoring_method = ScoringMethod::IndependentHumanAdjudication;
        manifest.deterministic_replay_evidence_digest = None;
        let report = qualify_benchmark_definition(&manifest);
        assert_eq!(report.status, QualificationStatus::Indeterminate);
    }
}
