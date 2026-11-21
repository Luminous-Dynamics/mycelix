//! Synthetic-only NIR screening and confirmatory referral bake-off.
//!
//! Hidden evaluator truth is stored separately from ScreenCandidate and is
//! never passed to deployable policy selection functions. The perfect-information
//! comparator is explicitly evaluator-only hindsight, not a deployable policy.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic routing benchmark only. The decision-risk score is a Gaussian threshold-risk heuristic, not full Bayesian EVSI. No real-data accuracy, agronomic threshold, laboratory replacement, diagnosis, or field authority is established.";

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    config: Config,
    screen_candidates: Vec<ScreenCandidate>,
    evaluator_only: EvaluatorOnly,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    referral_capacity: usize,
    nir_screening_cost: f64,
    confirmatory_test_cost: f64,
    seed: u64,
    fixed_uncertainty_threshold: f64,
}

/// This is the complete selector-visible type. It deliberately has no truth field.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScreenCandidate {
    sample_id: String,
    property_id: String,
    predicted_mean: f64,
    predictive_sd: f64,
    decision_threshold: f64,
    positive_if_above: bool,
    in_domain: bool,
    false_positive_loss: f64,
    false_negative_loss: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorOnly {
    truth_by_sample_id: Vec<TruthEntry>,
    reference_assumed_perfect_for_synthetic_fixture: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TruthEntry {
    sample_id: String,
    true_value: f64,
}

#[derive(Debug, Clone, Copy)]
enum Policy {
    NirOnly,
    AllReference,
    RandomFixedBudget,
    FixedUncertaintyThreshold,
    UncertaintyRank,
    ApplicabilityDomain,
    DecisionRiskRank,
    PerfectInformationHindsightOracle,
}

impl Policy {
    const ALL: [Policy; 8] = [
        Policy::NirOnly,
        Policy::AllReference,
        Policy::RandomFixedBudget,
        Policy::FixedUncertaintyThreshold,
        Policy::UncertaintyRank,
        Policy::ApplicabilityDomain,
        Policy::DecisionRiskRank,
        Policy::PerfectInformationHindsightOracle,
    ];

    fn id(self) -> &'static str {
        match self {
            Policy::NirOnly => "nir_only",
            Policy::AllReference => "all_reference",
            Policy::RandomFixedBudget => "random_fixed_budget",
            Policy::FixedUncertaintyThreshold => "fixed_uncertainty_threshold",
            Policy::UncertaintyRank => "uncertainty_rank",
            Policy::ApplicabilityDomain => "applicability_domain",
            Policy::DecisionRiskRank => "decision_risk_rank",
            Policy::PerfectInformationHindsightOracle => "perfect_information_hindsight_oracle",
        }
    }

    fn is_deployable(self) -> bool {
        !matches!(self, Policy::PerfectInformationHindsightOracle)
    }

    fn is_capacity_limited(self) -> bool {
        !matches!(self, Policy::AllReference)
    }
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    sample_count: usize,
    referral_capacity: usize,
    policies: Vec<PolicyResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct PolicyResult {
    policy_id: &'static str,
    referral_count: usize,
    referral_fraction: f64,
    nir_screening_count: usize,
    confirmatory_cost: f64,
    total_cost: f64,
    false_positive_count: usize,
    false_negative_count: usize,
    weighted_decision_loss: f64,
    errors_after_referral: usize,
    loss_regret_vs_all_reference: f64,
    loss_regret_vs_hindsight_oracle: f64,
    selection_sha256: String,
    deterministic_replay_pass: bool,
}

#[derive(Debug, Serialize)]
struct Controls {
    truth_mutation_selection_invariant: bool,
    hidden_truth_not_used_by_selectors: bool,
    capacity_pass: bool,
    deterministic_replay_pass: bool,
}

fn validate(scenario: &Scenario) -> Result<(), String> {
    if scenario.schema_version != "spectral-screening-referral-v0.1" {
        return Err("unsupported schema_version".into());
    }
    if scenario.fixture_status != FIXTURE_STATUS || scenario.evidence_class != "synthetic" {
        return Err("v0.1 accepts synthetic fixtures only".into());
    }
    if scenario.scenario_id.trim().is_empty() || scenario.screen_candidates.is_empty() {
        return Err("scenario_id and screen_candidates must be non-empty".into());
    }
    for (name, value) in [
        ("nir_screening_cost", scenario.config.nir_screening_cost),
        (
            "confirmatory_test_cost",
            scenario.config.confirmatory_test_cost,
        ),
        (
            "fixed_uncertainty_threshold",
            scenario.config.fixed_uncertainty_threshold,
        ),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(format!("{name} must be finite and non-negative"));
        }
    }
    if scenario.config.fixed_uncertainty_threshold == 0.0 {
        return Err("fixed_uncertainty_threshold must be greater than zero".into());
    }
    if !scenario
        .evaluator_only
        .reference_assumed_perfect_for_synthetic_fixture
    {
        return Err(
            "v0.1 requires explicit perfect-reference synthetic evaluator semantics".into(),
        );
    }

    let mut sample_ids = BTreeSet::new();
    for candidate in &scenario.screen_candidates {
        if candidate.sample_id.trim().is_empty() || candidate.property_id.trim().is_empty() {
            return Err("sample_id and property_id must be non-empty".into());
        }
        if !sample_ids.insert(candidate.sample_id.clone()) {
            return Err(format!("duplicate sample_id {}", candidate.sample_id));
        }
        for (name, value) in [
            ("predicted_mean", candidate.predicted_mean),
            ("predictive_sd", candidate.predictive_sd),
            ("decision_threshold", candidate.decision_threshold),
            ("false_positive_loss", candidate.false_positive_loss),
            ("false_negative_loss", candidate.false_negative_loss),
        ] {
            if !value.is_finite() {
                return Err(format!("{} {name} must be finite", candidate.sample_id));
            }
        }
        if candidate.predictive_sd <= 0.0
            || candidate.false_positive_loss < 0.0
            || candidate.false_negative_loss < 0.0
        {
            return Err(format!(
                "{} has invalid uncertainty or loss weight",
                candidate.sample_id
            ));
        }
    }

    let mut truth_ids = BTreeSet::new();
    for truth in &scenario.evaluator_only.truth_by_sample_id {
        if truth.sample_id.trim().is_empty() || !truth.true_value.is_finite() {
            return Err("evaluator truth identifiers and values must be valid".into());
        }
        if !truth_ids.insert(truth.sample_id.clone()) {
            return Err(format!("duplicate evaluator truth for {}", truth.sample_id));
        }
    }
    if sample_ids != truth_ids {
        return Err("selector candidate IDs and evaluator truth IDs must match exactly".into());
    }
    Ok(())
}

fn positive_class(value: f64, threshold: f64, positive_if_above: bool) -> bool {
    if positive_if_above {
        value >= threshold
    } else {
        value <= threshold
    }
}

fn normal_tail(z_abs: f64) -> f64 {
    // Abramowitz-Stegun 7.1.26 normal-tail approximation; deterministic and
    // adequate for a screening heuristic, not a calibrated inferential claim.
    let z = z_abs.max(0.0);
    let t = 1.0 / (1.0 + 0.231_641_9 * z);
    let density = (-0.5 * z * z).exp() * 0.398_942_280_401_432_7;
    let polynomial = t
        * (0.319_381_530
            + t * (-0.356_563_782
                + t * (1.781_477_937 + t * (-1.821_255_978 + t * 1.330_274_429))));
    (density * polynomial).clamp(0.0, 0.5)
}

fn expected_threshold_loss(candidate: &ScreenCandidate) -> f64 {
    let is_positive = positive_class(
        candidate.predicted_mean,
        candidate.decision_threshold,
        candidate.positive_if_above,
    );
    let z =
        (candidate.predicted_mean - candidate.decision_threshold).abs() / candidate.predictive_sd;
    let probability_wrong = normal_tail(z);
    let loss = if is_positive {
        candidate.false_positive_loss
    } else {
        candidate.false_negative_loss
    };
    probability_wrong * loss
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = value;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn random_score(seed: u64, sample_id: &str) -> f64 {
    let mut hash = seed;
    for byte in sample_id.as_bytes() {
        hash = splitmix64(hash ^ u64::from(*byte));
    }
    (hash as f64) / (u64::MAX as f64)
}

fn ranked_ids<F>(candidates: &[ScreenCandidate], capacity: usize, score: F) -> Vec<String>
where
    F: Fn(&ScreenCandidate) -> f64,
{
    let mut ranked: Vec<&ScreenCandidate> = candidates.iter().collect();
    ranked.sort_by(|a, b| {
        score(b)
            .total_cmp(&score(a))
            .then_with(|| a.sample_id.cmp(&b.sample_id))
    });
    ranked
        .into_iter()
        .take(capacity)
        .map(|candidate| candidate.sample_id.clone())
        .collect()
}

/// Selector accepts only selector-visible candidates; no evaluator truth is in this API.
fn select(policy: Policy, candidates: &[ScreenCandidate], config: &Config) -> Vec<String> {
    let capacity = config.referral_capacity.min(candidates.len());
    match policy {
        Policy::NirOnly => Vec::new(),
        Policy::AllReference => candidates.iter().map(|x| x.sample_id.clone()).collect(),
        Policy::RandomFixedBudget => ranked_ids(candidates, capacity, |x| {
            random_score(config.seed, &x.sample_id)
        }),
        Policy::FixedUncertaintyThreshold => {
            let eligible: Vec<ScreenCandidate> = candidates
                .iter()
                .filter(|x| x.predictive_sd >= config.fixed_uncertainty_threshold)
                .cloned()
                .collect();
            ranked_ids(&eligible, capacity, |x| x.predictive_sd)
        }
        Policy::UncertaintyRank => ranked_ids(candidates, capacity, |x| x.predictive_sd),
        Policy::ApplicabilityDomain => {
            let mut ranked: Vec<&ScreenCandidate> = candidates.iter().collect();
            ranked.sort_by(|a, b| {
                a.in_domain
                    .cmp(&b.in_domain)
                    .then_with(|| b.predictive_sd.total_cmp(&a.predictive_sd))
                    .then_with(|| a.sample_id.cmp(&b.sample_id))
            });
            ranked
                .into_iter()
                .take(capacity)
                .map(|candidate| candidate.sample_id.clone())
                .collect()
        }
        Policy::DecisionRiskRank => ranked_ids(candidates, capacity, expected_threshold_loss),
        Policy::PerfectInformationHindsightOracle => {
            // Not reachable through deployable selection. Evaluator truth is
            // joined only in build_report, in a separately named oracle path.
            Vec::new()
        }
    }
}

fn oracle_ids(
    candidates: &[ScreenCandidate],
    truths: &BTreeMap<String, f64>,
    capacity: usize,
) -> Vec<String> {
    let mut wrongly_classified: Vec<(&ScreenCandidate, f64)> = candidates
        .iter()
        .filter_map(|candidate| {
            let truth = truths.get(&candidate.sample_id)?;
            let predicted_positive = positive_class(
                candidate.predicted_mean,
                candidate.decision_threshold,
                candidate.positive_if_above,
            );
            let true_positive = positive_class(
                *truth,
                candidate.decision_threshold,
                candidate.positive_if_above,
            );
            if predicted_positive == true_positive {
                None
            } else {
                let loss = if predicted_positive {
                    candidate.false_positive_loss
                } else {
                    candidate.false_negative_loss
                };
                Some((candidate, loss))
            }
        })
        .collect();
    wrongly_classified.sort_by(|(a, loss_a), (b, loss_b)| {
        loss_b
            .total_cmp(loss_a)
            .then_with(|| a.sample_id.cmp(&b.sample_id))
    });
    wrongly_classified
        .into_iter()
        .take(capacity)
        .map(|(candidate, _)| candidate.sample_id.clone())
        .collect()
}

fn digest_ids(ids: &[String]) -> String {
    let mut sorted = ids.to_vec();
    sorted.sort();
    let joined = sorted.join("\n");
    format!("{:x}", Sha256::digest(joined.as_bytes()))
}

fn evaluate_policy(
    policy: Policy,
    candidates: &[ScreenCandidate],
    truths: &BTreeMap<String, f64>,
    config: &Config,
    oracle_loss: f64,
) -> PolicyResult {
    let selected = if matches!(policy, Policy::PerfectInformationHindsightOracle) {
        oracle_ids(
            candidates,
            truths,
            config.referral_capacity.min(candidates.len()),
        )
    } else {
        select(policy, candidates, config)
    };
    let selected_set: BTreeSet<&str> = selected.iter().map(String::as_str).collect();
    let mut false_positive_count = 0usize;
    let mut false_negative_count = 0usize;
    let mut weighted_decision_loss = 0.0;

    for candidate in candidates {
        let truth = truths[&candidate.sample_id];
        let resolved_value = if selected_set.contains(candidate.sample_id.as_str()) {
            truth
        } else {
            candidate.predicted_mean
        };
        let predicted_positive = positive_class(
            resolved_value,
            candidate.decision_threshold,
            candidate.positive_if_above,
        );
        let true_positive = positive_class(
            truth,
            candidate.decision_threshold,
            candidate.positive_if_above,
        );
        if predicted_positive && !true_positive {
            false_positive_count += 1;
            weighted_decision_loss += candidate.false_positive_loss;
        } else if !predicted_positive && true_positive {
            false_negative_count += 1;
            weighted_decision_loss += candidate.false_negative_loss;
        }
    }

    let referral_count = selected.len();
    let all_reference = matches!(policy, Policy::AllReference);
    let nir_screening_count = if all_reference { 0 } else { candidates.len() };
    let confirmatory_cost = referral_count as f64 * config.confirmatory_test_cost;
    let total_cost = if all_reference {
        confirmatory_cost
    } else {
        nir_screening_count as f64 * config.nir_screening_cost + confirmatory_cost
    };
    let repeated_selection = if matches!(policy, Policy::PerfectInformationHindsightOracle) {
        oracle_ids(
            candidates,
            truths,
            config.referral_capacity.min(candidates.len()),
        )
    } else {
        select(policy, candidates, config)
    };
    let deterministic_replay_pass = repeated_selection == selected;

    PolicyResult {
        policy_id: policy.id(),
        referral_count,
        referral_fraction: referral_count as f64 / candidates.len() as f64,
        nir_screening_count,
        confirmatory_cost,
        total_cost,
        false_positive_count,
        false_negative_count,
        weighted_decision_loss,
        errors_after_referral: false_positive_count + false_negative_count,
        loss_regret_vs_all_reference: weighted_decision_loss,
        loss_regret_vs_hindsight_oracle: weighted_decision_loss - oracle_loss,
        selection_sha256: digest_ids(&selected),
        deterministic_replay_pass,
    }
}

fn build_report(scenario: &Scenario, input_sha256: String) -> Result<Report, String> {
    let truths: BTreeMap<String, f64> = scenario
        .evaluator_only
        .truth_by_sample_id
        .iter()
        .map(|entry| (entry.sample_id.clone(), entry.true_value))
        .collect();

    let oracle_selection = oracle_ids(
        &scenario.screen_candidates,
        &truths,
        scenario
            .config
            .referral_capacity
            .min(scenario.screen_candidates.len()),
    );
    let oracle_set: BTreeSet<&str> = oracle_selection.iter().map(String::as_str).collect();
    let oracle_loss = scenario
        .screen_candidates
        .iter()
        .filter(|candidate| {
            let truth = truths[&candidate.sample_id];
            let resolved = if oracle_set.contains(candidate.sample_id.as_str()) {
                truth
            } else {
                candidate.predicted_mean
            };
            positive_class(
                resolved,
                candidate.decision_threshold,
                candidate.positive_if_above,
            ) != positive_class(
                truth,
                candidate.decision_threshold,
                candidate.positive_if_above,
            )
        })
        .map(|candidate| {
            let truth = truths[&candidate.sample_id];
            if positive_class(
                candidate.predicted_mean,
                candidate.decision_threshold,
                candidate.positive_if_above,
            ) {
                candidate.false_positive_loss
            } else if positive_class(
                truth,
                candidate.decision_threshold,
                candidate.positive_if_above,
            ) {
                candidate.false_negative_loss
            } else {
                0.0
            }
        })
        .sum::<f64>();

    let policies: Vec<PolicyResult> = Policy::ALL
        .iter()
        .copied()
        .map(|policy| {
            evaluate_policy(
                policy,
                &scenario.screen_candidates,
                &truths,
                &scenario.config,
                oracle_loss,
            )
        })
        .collect();

    let mut mutation_scenario = scenario.clone();
    for truth in &mut mutation_scenario.evaluator_only.truth_by_sample_id {
        truth.true_value = -truth.true_value - 10_000.0;
    }
    let truth_mutation_selection_invariant = Policy::ALL
        .iter()
        .copied()
        .filter(|policy| policy.is_deployable())
        .all(|policy| {
            select(policy, &scenario.screen_candidates, &scenario.config)
                == select(
                    policy,
                    &scenario.screen_candidates,
                    &mutation_scenario.config,
                )
        });

    let capacity_pass = Policy::ALL
        .iter()
        .zip(&policies)
        .filter(|(policy, _)| policy.is_capacity_limited())
        .all(|(_, result)| result.referral_count <= scenario.config.referral_capacity);
    let deterministic_replay_pass = policies.iter().all(|p| p.deterministic_replay_pass);

    Ok(Report {
        audit_version: AUDIT_VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: scenario.scenario_id.clone(),
        input_sha256,
        sample_count: scenario.screen_candidates.len(),
        referral_capacity: scenario.config.referral_capacity,
        policies,
        controls: Controls {
            truth_mutation_selection_invariant,
            hidden_truth_not_used_by_selectors: truth_mutation_selection_invariant,
            capacity_pass,
            deterministic_replay_pass,
        },
        interpretation_boundary: BOUNDARY,
    })
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let input = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&input)?;
    validate(&scenario).map_err(io::Error::other)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input));
    let report = build_report(&scenario, input_sha256)?;
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: spectral-screening-referral <input.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("spectral referral audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        let raw = include_str!("../fixtures/synthetic-referral.json");
        serde_json::from_str(raw).expect("valid fixture")
    }

    fn truths(s: &Scenario) -> BTreeMap<String, f64> {
        s.evaluator_only
            .truth_by_sample_id
            .iter()
            .map(|x| (x.sample_id.clone(), x.true_value))
            .collect()
    }

    #[test]
    fn synthetic_fixture_is_explicitly_qualified_as_synthetic_only() {
        let s = scenario();
        validate(&s).expect("valid synthetic scenario");
        assert_eq!(s.fixture_status, FIXTURE_STATUS);
        assert_eq!(s.evidence_class, "synthetic");
    }

    #[test]
    fn selectors_do_not_receive_or_change_with_hidden_truth() {
        let mut s = scenario();
        let original = s.clone();
        for truth in &mut s.evaluator_only.truth_by_sample_id {
            truth.true_value += 99_999.0;
        }
        for policy in Policy::ALL.into_iter().filter(|x| x.is_deployable()) {
            assert_eq!(
                select(policy, &original.screen_candidates, &original.config),
                select(policy, &s.screen_candidates, &s.config)
            );
        }
    }

    #[test]
    fn referral_capacity_is_respected_by_deployable_policies() {
        let s = scenario();
        for policy in Policy::ALL.into_iter().filter(|x| x.is_capacity_limited()) {
            assert!(
                select(policy, &s.screen_candidates, &s.config).len() <= s.config.referral_capacity
            );
        }
        assert_eq!(
            select(Policy::AllReference, &s.screen_candidates, &s.config).len(),
            s.screen_candidates.len(),
            "all-reference is an intentionally uncapped benchmark arm"
        );
    }

    #[test]
    fn out_of_domain_candidates_are_prioritized() {
        let s = scenario();
        let selected = select(Policy::ApplicabilityDomain, &s.screen_candidates, &s.config);
        assert_eq!(selected[0], "syn-008");
        assert_eq!(selected[1], "syn-003");
    }

    #[test]
    fn fixed_uncertainty_threshold_can_refer_fewer_than_capacity() {
        let mut s = scenario();
        s.config.fixed_uncertainty_threshold = 0.7;
        assert!(
            select(
                Policy::FixedUncertaintyThreshold,
                &s.screen_candidates,
                &s.config
            )
            .is_empty()
        );
    }

    #[test]
    fn random_and_ranked_policies_replay_deterministically() {
        let s = scenario();
        for policy in Policy::ALL.into_iter().filter(|x| x.is_deployable()) {
            assert_eq!(
                select(policy, &s.screen_candidates, &s.config),
                select(policy, &s.screen_candidates, &s.config)
            );
        }
    }

    #[test]
    fn hindsight_oracle_uses_budget_and_only_selects_errors() {
        let s = scenario();
        let selected = oracle_ids(
            &s.screen_candidates,
            &truths(&s),
            s.config.referral_capacity,
        );
        assert!(selected.len() <= s.config.referral_capacity);
        for sample_id in selected {
            let candidate = s
                .screen_candidates
                .iter()
                .find(|x| x.sample_id == sample_id)
                .unwrap();
            let truth = truths(&s)[&sample_id];
            assert_ne!(
                positive_class(
                    candidate.predicted_mean,
                    candidate.decision_threshold,
                    candidate.positive_if_above
                ),
                positive_class(
                    truth,
                    candidate.decision_threshold,
                    candidate.positive_if_above
                )
            );
        }
    }

    #[test]
    fn report_has_all_policy_comparators_and_controls() {
        let s = scenario();
        let report = build_report(&s, "0".repeat(64)).expect("report");
        assert_eq!(report.policies.len(), 8);
        assert!(report.controls.truth_mutation_selection_invariant);
        assert!(report.controls.hidden_truth_not_used_by_selectors);
        assert!(report.controls.capacity_pass);
        assert!(report.controls.deterministic_replay_pass);
        assert!(
            report
                .policies
                .iter()
                .any(|p| p.policy_id == "perfect_information_hindsight_oracle")
        );
    }

    #[test]
    fn duplicate_sample_id_is_rejected() {
        let mut s = scenario();
        s.screen_candidates.push(s.screen_candidates[0].clone());
        assert!(validate(&s).is_err());
    }
}
