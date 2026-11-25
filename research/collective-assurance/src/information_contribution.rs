//! Paired leave-one-source-origin-out information-contribution qualification.
//!
//! This module validates whether a counterfactual replay changes only one
//! authenticated, authorized source-origin closure under an otherwise matched
//! execution manifest. It does not estimate a human causal effect or prove
//! that the declared source registry / replay runner is truthful.

use std::collections::BTreeSet;

use super::{
    is_sha256, qualify_benchmark_comparison, BenchmarkRunManifest, ComparisonIntent,
    ComparisonStatus, QualificationReport, QualificationStatus,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OriginDisclosureStatus {
    AuthorizedAndDisclosed,
    WithheldByRights,
    NoResponse,
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InformationOriginCounterfactual {
    pub experiment_id: String,
    pub origin_disclosure_status: OriginDisclosureStatus,
    pub omitted_source_origin_id: String,
    pub omitted_source_origin_digest: String,
    /// Digest of the independently registered source-origin receipt.
    pub source_origin_registry_receipt_digest: Option<String>,
    /// Digest of an independently checked origin/derivation removal closure.
    pub removal_closure_attestation_digest: Option<String>,
    pub baseline_manifest: BenchmarkRunManifest,
    pub counterfactual_manifest: BenchmarkRunManifest,
    pub baseline_input_digest: String,
    pub counterfactual_input_digest: String,
    pub baseline_shared_context_digest: String,
    pub counterfactual_shared_context_digest: String,
    pub baseline_ledger_root_digest: Option<String>,
    pub counterfactual_ledger_root_digest: Option<String>,
    pub baseline_source_origins: Vec<String>,
    pub counterfactual_source_origins: Vec<String>,
    /// The root origin plus all authenticated derived source origins removed with it.
    pub removed_origin_closure: Vec<String>,
    pub baseline_decision_digest: String,
    pub counterfactual_decision_digest: String,
    pub baseline_confidence: f64,
    pub counterfactual_confidence: f64,
    pub baseline_decision_quality: f64,
    pub counterfactual_decision_quality: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InformationContributionReport {
    pub qualification: QualificationReport,
    /// None unless the pair passed the structural and rights contract.
    pub decision_changed: Option<bool>,
    /// Counterfactual minus baseline confidence; negative means confidence fell.
    pub confidence_delta: Option<f64>,
    /// Counterfactual minus baseline decision quality; negative means quality fell.
    pub decision_quality_delta: Option<f64>,
    pub removed_origin_count: Option<usize>,
}

fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}

fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

fn unique_nonempty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .all(|value| !value.trim().is_empty() && seen.insert(value.as_str()))
}

fn normalized(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn no_metrics(report: QualificationReport) -> InformationContributionReport {
    InformationContributionReport {
        qualification: report,
        decision_changed: None,
        confidence_delta: None,
        decision_quality_delta: None,
        removed_origin_count: None,
    }
}

/// Qualifies one source-origin counterfactual replay.
///
/// The baseline and counterfactual must use the same model, task, prompt, tools,
/// evaluator, budgets, agent count, memory, seed and system configuration. The
/// only intended difference is the removal of one authorized source origin and
/// its authenticated derived-origin closure. Rights-based withholding, silence
/// and unknown disclosure are not scored as a missing contribution.
pub fn qualify_information_origin_counterfactual(
    case: &InformationOriginCounterfactual,
) -> InformationContributionReport {
    let mut report = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };

    if case.experiment_id.trim().is_empty() || case.omitted_source_origin_id.trim().is_empty() {
        fail(&mut report, "experiment and omitted source-origin identifiers must be non-empty");
    }

    match case.origin_disclosure_status {
        OriginDisclosureStatus::AuthorizedAndDisclosed => {}
        OriginDisclosureStatus::WithheldByRights => {
            indeterminate(
                &mut report,
                "source was withheld by rights; do not score it as a missing contribution",
            )
        }
        OriginDisclosureStatus::NoResponse => {
            indeterminate(
                &mut report,
                "source holder did not respond; no source contribution is inferable",
            )
        }
        OriginDisclosureStatus::Unknown => {
            indeterminate(
                &mut report,
                "source disclosure status is unknown; contribution is not evaluable",
            )
        }
    }

    for (label, digest) in [
        ("source origin", case.omitted_source_origin_digest.as_str()),
        ("baseline input", case.baseline_input_digest.as_str()),
        ("counterfactual input", case.counterfactual_input_digest.as_str()),
        ("baseline shared context", case.baseline_shared_context_digest.as_str()),
        (
            "counterfactual shared context",
            case.counterfactual_shared_context_digest.as_str(),
        ),
        ("baseline decision", case.baseline_decision_digest.as_str()),
        (
            "counterfactual decision",
            case.counterfactual_decision_digest.as_str(),
        ),
    ] {
        if !is_sha256(digest) {
            fail(&mut report, format!("{label} digest is malformed"));
        }
    }

    for (label, digest) in [
        (
            "source-origin registry receipt",
            case.source_origin_registry_receipt_digest.as_deref(),
        ),
        (
            "origin-removal closure attestation",
            case.removal_closure_attestation_digest.as_deref(),
        ),
        ("baseline ledger root", case.baseline_ledger_root_digest.as_deref()),
        (
            "counterfactual ledger root",
            case.counterfactual_ledger_root_digest.as_deref(),
        ),
    ] {
        match digest {
            Some(value) if is_sha256(value) => {}
            Some(_) => fail(&mut report, format!("{label} digest is malformed")),
            None => indeterminate(&mut report, format!("{label} is missing")),
        }
    }

    if case.baseline_input_digest == case.counterfactual_input_digest {
        fail(&mut report, "counterfactual input digest did not change");
    }
    if case.baseline_shared_context_digest != case.counterfactual_shared_context_digest {
        fail(&mut report, "non-target shared context changed between paired replays");
    }

    let comparison = qualify_benchmark_comparison(
        &case.baseline_manifest,
        &case.counterfactual_manifest,
        ComparisonIntent::MatchedSystem,
    );
    if comparison.status != ComparisonStatus::Comparable {
        for mismatch in comparison.hard_mismatches {
            fail(&mut report, format!("paired replay is not strictly matched: {mismatch}"));
        }
        for reason in comparison.rationale {
            report.reasons.push(reason);
        }
    }
    if case.baseline_manifest.system_variant != case.counterfactual_manifest.system_variant
        || case.baseline_manifest.system_config_digest
            != case.counterfactual_manifest.system_config_digest
    {
        fail(
            &mut report,
            "system variant or configuration changed; only the information origin may be removed",
        );
    }

    if !unique_nonempty(&case.baseline_source_origins)
        || !unique_nonempty(&case.counterfactual_source_origins)
        || !unique_nonempty(&case.removed_origin_closure)
    {
        fail(
            &mut report,
            "source-origin lists and removal closure must contain unique non-empty identifiers",
        );
    }

    let baseline_origins: BTreeSet<&str> = case
        .baseline_source_origins
        .iter()
        .map(String::as_str)
        .collect();
    let counterfactual_origins: BTreeSet<&str> = case
        .counterfactual_source_origins
        .iter()
        .map(String::as_str)
        .collect();
    let removal_closure: BTreeSet<&str> = case
        .removed_origin_closure
        .iter()
        .map(String::as_str)
        .collect();

    if !baseline_origins.contains(case.omitted_source_origin_id.as_str()) {
        fail(&mut report, "omitted source origin is absent from the baseline evidence set");
    }
    if !removal_closure.contains(case.omitted_source_origin_id.as_str()) {
        fail(&mut report, "removal closure does not include the requested source origin");
    }
    if !removal_closure.is_subset(&baseline_origins) {
        fail(&mut report, "removal closure includes origins absent from the baseline");
    }
    if !counterfactual_origins.is_disjoint(&removal_closure) {
        fail(
            &mut report,
            "counterfactual retains an origin or derived origin that should have been removed",
        );
    }
    let expected_counterfactual: BTreeSet<&str> = baseline_origins
        .difference(&removal_closure)
        .copied()
        .collect();
    if counterfactual_origins != expected_counterfactual {
        fail(
            &mut report,
            "counterfactual source-origin set differs beyond the declared removal closure",
        );
    }

    for (label, value) in [
        ("baseline confidence", case.baseline_confidence),
        ("counterfactual confidence", case.counterfactual_confidence),
        ("baseline decision quality", case.baseline_decision_quality),
        (
            "counterfactual decision quality",
            case.counterfactual_decision_quality,
        ),
    ] {
        if !normalized(value) {
            fail(
                &mut report,
                format!("{label} must be finite and within [0, 1]"),
            );
        }
    }

    if report.status != QualificationStatus::Pass {
        return no_metrics(report);
    }

    InformationContributionReport {
        qualification: report,
        decision_changed: Some(
            case.baseline_decision_digest != case.counterfactual_decision_digest,
        ),
        confidence_delta: Some(
            case.counterfactual_confidence - case.baseline_confidence,
        ),
        decision_quality_delta: Some(
            case.counterfactual_decision_quality - case.baseline_decision_quality,
        ),
        removed_origin_count: Some(removal_closure.len()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(character: char) -> String {
        std::iter::repeat_n(character, 64).collect()
    }

    fn manifest(run_id: &str) -> BenchmarkRunManifest {
        BenchmarkRunManifest {
            run_id: run_id.into(),
            system_variant: "same-system".into(),
            system_config_digest: digest('a'),
            benchmark_id: "information-origin-replay".into(),
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
            agent_count: 1,
            memory_budget_bytes: 100000000,
            seed_policy_id: "paired-seed-v1".into(),
            seed: 23,
        }
    }

    fn case() -> InformationOriginCounterfactual {
        InformationOriginCounterfactual {
            experiment_id: "info-replay-001".into(),
            origin_disclosure_status: OriginDisclosureStatus::AuthorizedAndDisclosed,
            omitted_source_origin_id: "origin-A".into(),
            omitted_source_origin_digest: digest('6'),
            source_origin_registry_receipt_digest: Some(digest('7')),
            removal_closure_attestation_digest: Some(digest('8')),
            baseline_manifest: manifest("baseline-run"),
            counterfactual_manifest: manifest("counterfactual-run"),
            baseline_input_digest: digest('9'),
            counterfactual_input_digest: digest('0'),
            baseline_shared_context_digest: digest('a'),
            counterfactual_shared_context_digest: digest('a'),
            baseline_ledger_root_digest: Some(digest('b')),
            counterfactual_ledger_root_digest: Some(digest('c')),
            baseline_source_origins: vec!["origin-A".into(), "origin-B".into()],
            counterfactual_source_origins: vec!["origin-B".into()],
            removed_origin_closure: vec!["origin-A".into()],
            baseline_decision_digest: digest('d'),
            counterfactual_decision_digest: digest('e'),
            baseline_confidence: 0.9,
            counterfactual_confidence: 0.6,
            baseline_decision_quality: 0.85,
            counterfactual_decision_quality: 0.55,
        }
    }

    #[test]
    fn matched_authorized_origin_removal_passes_and_reports_deltas() {
        let report = qualify_information_origin_counterfactual(&case());
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.decision_changed, Some(true));
        assert_eq!(report.confidence_delta, Some(-0.3));
        assert_eq!(report.decision_quality_delta, Some(-0.3));
        assert_eq!(report.removed_origin_count, Some(1));
    }

    #[test]
    fn rights_withholding_is_indeterminate_not_a_missing_contribution() {
        let mut test = case();
        test.origin_disclosure_status = OriginDisclosureStatus::WithheldByRights;
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
        assert!(report.decision_quality_delta.is_none());
        assert!(report
            .qualification
            .reasons
            .iter()
            .any(|reason| reason.contains("withheld by rights")));
    }

    #[test]
    fn missing_source_origin_registry_receipt_is_indeterminate() {
        let mut test = case();
        test.source_origin_registry_receipt_digest = None;
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn missing_removal_closure_attestation_is_indeterminate() {
        let mut test = case();
        test.removal_closure_attestation_digest = None;
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Indeterminate);
    }

    #[test]
    fn mismatched_budget_fails_paired_replay() {
        let mut test = case();
        test.counterfactual_manifest.token_budget += 1;
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn changed_shared_context_fails_isolation() {
        let mut test = case();
        test.counterfactual_shared_context_digest = digest('f');
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn unchanged_input_fails_to_define_a_counterfactual() {
        let mut test = case();
        test.counterfactual_input_digest = test.baseline_input_digest.clone();
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn duplicate_source_origins_fail() {
        let mut test = case();
        test.baseline_source_origins.push("origin-A".into());
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn omitted_origin_must_be_in_baseline() {
        let mut test = case();
        test.baseline_source_origins = vec!["origin-B".into()];
        test.removed_origin_closure = vec!["origin-A".into()];
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn derived_origin_must_be_removed_with_root() {
        let mut test = case();
        test.baseline_source_origins = vec![
            "origin-A".into(),
            "derived-A-summary".into(),
            "origin-B".into(),
        ];
        test.removed_origin_closure = vec!["origin-A".into(), "derived-A-summary".into()];
        test.counterfactual_source_origins = vec!["derived-A-summary".into(), "origin-B".into()];
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn added_origin_in_counterfactual_fails() {
        let mut test = case();
        test.counterfactual_source_origins.push("origin-C".into());
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }

    #[test]
    fn invalid_normalized_measurement_fails() {
        let mut test = case();
        test.counterfactual_confidence = f64::NAN;
        let report = qualify_information_origin_counterfactual(&test);
        assert_eq!(report.qualification.status, QualificationStatus::Fail);
    }
}
