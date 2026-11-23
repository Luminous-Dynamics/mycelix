//! Deterministic Monte Carlo operating-characteristics benchmark for paired-block
//! randomized soil trials. All data are manufactured and all results are synthetic.
//!
//! Uses assignment-based ITT and exact paired-block sign-flip randomization tests.
//! Incomplete trials and possible/unassessed interference abstain; units are never
//! dropped to form a complete-case estimator.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic Monte Carlo only. The DGP, seed and trial assumptions are manufactured. Conditional rejection rates among qualified trials must be interpreted with qualification rates and abstention reasons. No field effect, validated confidence interval, agronomic threshold or causal transportability claim is established.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Suite {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    suite_id: String,
    base_seed: u64,
    simulation_count_per_scenario: usize,
    alpha: f64,
    scenarios: Vec<Scenario>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    scenario_id: String,
    block_count: usize,
    true_mean_treatment_effect: f64,
    effect_heterogeneity_sd: f64,
    baseline_mean: f64,
    baseline_sd: f64,
    block_change_sd: f64,
    plot_change_sd: f64,
    measurement_sd: f64,
    treatment_compliance_probability: f64,
    control_crossover_probability: f64,
    missing_outcome_probability: f64,
    missingness_mode: String,
    missingness_outcome_sensitivity: f64,
    interference_status: String,
}

#[derive(Debug, Serialize)]
struct Report {
    report_version: &'static str,
    fixture_status: &'static str,
    suite_id: String,
    input_sha256: String,
    base_seed: u64,
    simulation_count_per_scenario: usize,
    alpha: f64,
    scenario_count: usize,
    scenarios: Vec<ScenarioResult>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct ScenarioResult {
    scenario_id: String,
    operating_characteristic: &'static str,
    block_count: usize,
    randomization_assignments_enumerated: usize,
    minimum_attainable_two_sided_p_value: f64,
    true_itt_effect: f64,
    qualified_trial_count: usize,
    abstained_missing_outcome_count: usize,
    abstained_interference_count: usize,
    qualification_rate: Option<f64>,
    mean_itt_estimate: Option<f64>,
    bias: Option<f64>,
    rmse: Option<f64>,
    empirical_rejection_rate: Option<f64>,
    rejection_rate_monte_carlo_se: Option<f64>,
    treatment_nonadherence_rate: f64,
    control_crossover_rate: f64,
    interpretation: String,
}

#[derive(Clone, Copy)]
struct Unit {
    baseline_observed: f64,
    followup_observed: f64,
    treatment_effect: f64,
    received_treatment_if_assigned: bool,
    received_treatment_if_control: bool,
    missing: bool,
}

struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    fn uniform_open(&mut self) -> f64 {
        let bits = self.next_u64() >> 11;
        (bits as f64 + 0.5) / ((1u64 << 53) as f64)
    }

    fn standard_normal(&mut self) -> f64 {
        let u1 = self.uniform_open();
        let u2 = self.uniform_open();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    fn bernoulli(&mut self, probability: f64) -> bool {
        self.uniform_open() < probability
    }
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn validate(s: &Suite) -> Result<(), String> {
    if s.schema_version != "soil-trial-operating-characteristics-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
        || s.suite_id.trim().is_empty()
    {
        return Err("unsupported schema or non-synthetic evidence; suite must be synthetic".into());
    }
    if !(100..=20_000).contains(&s.simulation_count_per_scenario)
        || !(0.0 < s.alpha && s.alpha <= 0.2)
        || s.scenarios.len() < 2
    {
        return Err("simulation count, alpha or scenario count is outside v0.1 limits".into());
    }
    let mut ids = std::collections::BTreeSet::new();
    for x in &s.scenarios {
        if x.scenario_id.trim().is_empty() || !ids.insert(x.scenario_id.as_str()) {
            return Err("scenario IDs must be non-empty and unique".into());
        }
        if !(2..=12).contains(&x.block_count) {
            return Err(format!(
                "{} block count must be between 2 and 12",
                x.scenario_id
            ));
        }
        for (name, value) in [
            ("true_mean_treatment_effect", x.true_mean_treatment_effect),
            ("effect_heterogeneity_sd", x.effect_heterogeneity_sd),
            ("baseline_mean", x.baseline_mean),
            ("baseline_sd", x.baseline_sd),
            ("block_change_sd", x.block_change_sd),
            ("plot_change_sd", x.plot_change_sd),
            ("measurement_sd", x.measurement_sd),
            (
                "treatment_compliance_probability",
                x.treatment_compliance_probability,
            ),
            (
                "control_crossover_probability",
                x.control_crossover_probability,
            ),
            ("missing_outcome_probability", x.missing_outcome_probability),
            (
                "missingness_outcome_sensitivity",
                x.missingness_outcome_sensitivity,
            ),
        ] {
            finite(name, value)?;
        }
        if x.effect_heterogeneity_sd < 0.0
            || x.baseline_sd < 0.0
            || x.block_change_sd < 0.0
            || x.plot_change_sd < 0.0
            || x.measurement_sd < 0.0
            || !(0.0..=1.0).contains(&x.treatment_compliance_probability)
            || !(0.0..=1.0).contains(&x.control_crossover_probability)
            || !(0.0..=0.95).contains(&x.missing_outcome_probability)
            || !(0.0..=1.0).contains(&x.missingness_outcome_sensitivity)
        {
            return Err(format!(
                "{} has an invalid scale/probability",
                x.scenario_id
            ));
        }
        if !matches!(
            x.missingness_mode.as_str(),
            "none" | "mcar" | "response_dependent"
        ) {
            return Err(format!(
                "{} has unsupported missingness mode",
                x.scenario_id
            ));
        }
        if x.missingness_mode == "none" && x.missing_outcome_probability != 0.0 {
            return Err(format!(
                "{} missingness_mode=none requires zero missing probability",
                x.scenario_id
            ));
        }
        if x.missingness_mode != "response_dependent" && x.missingness_outcome_sensitivity != 0.0 {
            return Err(format!(
                "{} outcome sensitivity applies only to response_dependent missingness",
                x.scenario_id
            ));
        }
        if !matches!(
            x.interference_status.as_str(),
            "none_reported" | "possible_spillover" | "unassessed"
        ) {
            return Err(format!(
                "{} has unsupported interference status",
                x.scenario_id
            ));
        }
    }
    Ok(())
}

fn min_two_sided_p(blocks: usize) -> f64 {
    2.0 / (1usize << blocks) as f64
}

fn exact_sign_flip_p(values: &[f64], observed_mean: f64) -> f64 {
    let n = values.len();
    let total = 1usize << n;
    let mut extreme = 0usize;
    for mask in 0..total {
        let sum: f64 = values
            .iter()
            .enumerate()
            .map(|(i, value)| {
                if ((mask >> i) & 1) == 0 {
                    *value
                } else {
                    -*value
                }
            })
            .sum();
        let statistic = sum / n as f64;
        if statistic.abs() + 1e-12 >= observed_mean.abs() {
            extreme += 1;
        }
    }
    extreme as f64 / total as f64
}

/// Independently structured recursive enumerator used only as a test oracle.
#[cfg(test)]
fn recursive_reference_p(values: &[f64], observed_mean: f64) -> f64 {
    fn recurse(
        values: &[f64],
        index: usize,
        sum: f64,
        n: usize,
        obs: f64,
        extreme: &mut usize,
        total: &mut usize,
    ) {
        if index == values.len() {
            *total += 1;
            if (sum / n as f64).abs() + 1e-12 >= obs.abs() {
                *extreme += 1;
            }
        } else {
            recurse(
                values,
                index + 1,
                sum + values[index],
                n,
                obs,
                extreme,
                total,
            );
            recurse(
                values,
                index + 1,
                sum - values[index],
                n,
                obs,
                extreme,
                total,
            );
        }
    }
    let mut extreme = 0;
    let mut total = 0;
    recurse(
        values,
        0,
        0.0,
        values.len(),
        observed_mean,
        &mut extreme,
        &mut total,
    );
    extreme as f64 / total as f64
}

fn simulate_unit_pair(rng: &mut SplitMix64, x: &Scenario, block_change: f64) -> (Unit, Unit) {
    let mut make_unit = || {
        let baseline_true = x.baseline_mean + x.baseline_sd * rng.standard_normal();
        let baseline_observed = baseline_true + x.measurement_sd * rng.standard_normal();
        let treatment_effect =
            x.true_mean_treatment_effect + x.effect_heterogeneity_sd * rng.standard_normal();
        let untreated_change = block_change + x.plot_change_sd * rng.standard_normal();
        let treated_change = untreated_change + treatment_effect;
        let followup_if_treated =
            baseline_true + treated_change + x.measurement_sd * rng.standard_normal();
        let followup_if_control =
            baseline_true + untreated_change + x.measurement_sd * rng.standard_normal();
        let received_treatment_if_assigned = rng.bernoulli(x.treatment_compliance_probability);
        let received_treatment_if_control = rng.bernoulli(x.control_crossover_probability);
        let missing = match x.missingness_mode.as_str() {
            "none" => false,
            "mcar" => rng.bernoulli(x.missing_outcome_probability),
            "response_dependent" => {
                let standardized = ((followup_if_treated - x.baseline_mean)
                    / (x.block_change_sd + x.plot_change_sd + x.measurement_sd + 1e-9))
                    .max(0.0);
                let p = (x.missing_outcome_probability
                    + x.missingness_outcome_sensitivity * standardized * 0.1)
                    .clamp(0.0, 0.95);
                rng.bernoulli(p)
            }
            _ => unreachable!(),
        };
        Unit {
            baseline_observed,
            followup_observed: followup_if_control,
            treatment_effect,
            received_treatment_if_assigned,
            received_treatment_if_control,
            missing,
        }
    };
    (make_unit(), make_unit())
}

fn run_trial(rng: &mut SplitMix64, scenario: &Scenario) -> TrialResult {
    let mut contrasts = Vec::with_capacity(scenario.block_count);
    let mut true_effect_sum = 0.0;
    let mut unit_count = 0usize;
    let mut missing = false;
    let mut nonadherent = 0usize;
    let mut crossover = 0usize;
    let mut treatment_assigned = 0usize;
    let mut control_assigned = 0usize;

    for _ in 0..scenario.block_count {
        let common_block_change = scenario.block_change_sd * rng.standard_normal();
        let (mut a, mut b) = simulate_unit_pair(rng, scenario, common_block_change);
        true_effect_sum += a.treatment_effect + b.treatment_effect;
        unit_count += 2;
        let a_treated = rng.bernoulli(0.5);
        let (treatment, control) = if a_treated {
            (&mut a, &mut b)
        } else {
            (&mut b, &mut a)
        };
        treatment_assigned += 1;
        control_assigned += 1;
        if !treatment.received_treatment_if_assigned {
            nonadherent += 1;
        }
        if control.received_treatment_if_control {
            crossover += 1;
        }
        if treatment.missing || control.missing {
            missing = true;
        }

        let treatment_receipt = treatment.received_treatment_if_assigned;
        let control_receipt = control.received_treatment_if_control;
        let treatment_followup = if treatment_receipt {
            treatment.followup_observed + treatment.treatment_effect
        } else {
            treatment.followup_observed
        };
        let control_followup = if control_receipt {
            control.followup_observed + control.treatment_effect
        } else {
            control.followup_observed
        };
        let treatment_change = treatment_followup - treatment.baseline_observed;
        let control_change = control_followup - control.baseline_observed;
        contrasts.push(treatment_change - control_change);
    }
    let true_itt = (true_effect_sum / unit_count as f64)
        * (scenario.treatment_compliance_probability - scenario.control_crossover_probability);
    if scenario.interference_status != "none_reported" {
        return TrialResult::AbstainedInterference {
            nonadherent,
            crossover,
            treatment_assigned,
            control_assigned,
        };
    }
    if missing {
        return TrialResult::AbstainedMissing {
            nonadherent,
            crossover,
            treatment_assigned,
            control_assigned,
            true_itt,
        };
    }
    let estimate = contrasts.iter().sum::<f64>() / contrasts.len() as f64;
    let p = exact_sign_flip_p(&contrasts, estimate);
    TrialResult::Qualified {
        estimate,
        p,
        true_itt,
        nonadherent,
        crossover,
        treatment_assigned,
        control_assigned,
    }
}

enum TrialResult {
    Qualified {
        estimate: f64,
        p: f64,
        true_itt: f64,
        nonadherent: usize,
        crossover: usize,
        treatment_assigned: usize,
        control_assigned: usize,
    },
    AbstainedMissing {
        nonadherent: usize,
        crossover: usize,
        treatment_assigned: usize,
        control_assigned: usize,
    },
    AbstainedInterference {
        nonadherent: usize,
        crossover: usize,
        treatment_assigned: usize,
        control_assigned: usize,
    },
}

fn simulate_scenario(suite: &Suite, scenario: &Scenario, scenario_index: usize) -> ScenarioResult {
    let seed = suite.base_seed ^ ((scenario_index as u64 + 1).wrapping_mul(0xD6E8FEB86659FD93));
    let mut rng = SplitMix64::new(seed);
    let mut qualified = 0usize;
    let mut missing_abstain = 0usize;
    let mut interference_abstain = 0usize;
    let mut reject = 0usize;
    let mut sum_estimate = 0.0;
    let mut sum_error = 0.0;
    let mut sum_squared_error = 0.0;
    let mut true_effect = 0.0;
    let mut nonadherence = 0usize;
    let mut crossover = 0usize;
    let mut treatment_assigned = 0usize;
    let mut control_assigned = 0usize;
    for _ in 0..suite.simulation_count_per_scenario {
        match run_trial(&mut rng, scenario) {
            TrialResult::Qualified {
                estimate,
                p,
                true_itt,
                nonadherent,
                crossover: cross,
                treatment_assigned: ta,
                control_assigned: ca,
            } => {
                qualified += 1;
                if p <= suite.alpha {
                    reject += 1;
                }
                sum_estimate += estimate;
                let error = estimate - true_itt;
                sum_error += error;
                sum_squared_error += error * error;
                true_effect += true_itt;
                nonadherence += nonadherent;
                crossover += cross;
                treatment_assigned += ta;
                control_assigned += ca;
            }
            TrialResult::AbstainedMissing {
                nonadherent,
                crossover: cross,
                treatment_assigned: ta,
                control_assigned: ca,
                ..
            } => {
                missing_abstain += 1;
                nonadherence += nonadherent;
                crossover += cross;
                treatment_assigned += ta;
                control_assigned += ca;
            }
            TrialResult::AbstainedInterference {
                nonadherent,
                crossover: cross,
                treatment_assigned: ta,
                control_assigned: ca,
                ..
            } => {
                interference_abstain += 1;
                nonadherence += nonadherent;
                crossover += cross;
                treatment_assigned += ta;
                control_assigned += ca;
            }
        }
    }
    let total = suite.simulation_count_per_scenario;
    let qrate = qualified as f64 / total as f64;
    let mean_est = if qualified > 0 {
        Some(sum_estimate / qualified as f64)
    } else {
        None
    };
    let bias = if qualified > 0 {
        Some(sum_error / qualified as f64)
    } else {
        None
    };
    let rmse = if qualified > 0 {
        Some((sum_squared_error / qualified as f64).sqrt())
    } else {
        None
    };
    let rejection = if qualified > 0 {
        Some(reject as f64 / qualified as f64)
    } else {
        None
    };
    let mcse = rejection.map(|r| (r * (1.0 - r) / qualified.max(1) as f64).sqrt());
    let mean_true = if qualified > 0 {
        true_effect / qualified as f64
    } else {
        scenario.true_mean_treatment_effect
            * (scenario.treatment_compliance_probability - scenario.control_crossover_probability)
    };
    let oc = if qualified == 0 {
        "not_estimable_due_to_abstention"
    } else if mean_true.abs() < 1e-12 {
        "type_i_error"
    } else {
        "power"
    };
    let nonadh_rate = if treatment_assigned > 0 {
        nonadherence as f64 / treatment_assigned as f64
    } else {
        0.0
    };
    let cross_rate = if control_assigned > 0 {
        crossover as f64 / control_assigned as f64
    } else {
        0.0
    };
    let resolution = min_two_sided_p(scenario.block_count);
    ScenarioResult {
        scenario_id:scenario.scenario_id.clone(),
        operating_characteristic:oc,
        block_count:scenario.block_count,
        randomization_assignments_enumerated:1usize<<scenario.block_count,
        minimum_attainable_two_sided_p_value:resolution,
        true_itt_effect:mean_true,
        qualified_trial_count:qualified,
        abstained_missing_outcome_count:missing_abstain,
        abstained_interference_count:interference_abstain,
        qualification_rate:if total>0 {Some(qrate)} else {None},
        mean_itt_estimate:mean_est,
        bias,
        rmse,
        empirical_rejection_rate:rejection,
        rejection_rate_monte_carlo_se:mcse,
        treatment_nonadherence_rate:nonadh_rate,
        control_crossover_rate:cross_rate,
        interpretation: if oc=="type_i_error" {
            "Empirical rejection probability under a synthetic zero-ITT DGP, conditional on qualification. Compare to nominal alpha and report the qualification rate."
        } else if oc=="power" {
            "Empirical rejection probability under a synthetic nonzero-ITT DGP, conditional on qualification. This is not a field-trial power guarantee."
        } else {
            "No qualified trials; no type-I error / power estimate is emitted. Abstentions remain in the requested-trial denominator."
        }.into(),
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output already exists").into());
    }
    let bytes = fs::read(input)?;
    let suite: Suite = serde_json::from_slice(&bytes)?;
    validate(&suite).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let scenarios = suite
        .scenarios
        .iter()
        .enumerate()
        .map(|(i, s)| simulate_scenario(&suite, s, i))
        .collect::<Vec<_>>();
    let report = Report {
        report_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        suite_id: suite.suite_id.clone(),
        input_sha256: digest,
        base_seed: suite.base_seed,
        simulation_count_per_scenario: suite.simulation_count_per_scenario,
        alpha: suite.alpha,
        scenario_count: scenarios.len(),
        scenarios,
        interpretation_boundary: BOUNDARY,
    };
    let bytes = serde_json::to_vec_pretty(&report)?;
    let mut f = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    f.write_all(&bytes)?;
    f.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: soil-trial-operating-characteristics <suite.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(e) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("soil trial operating-characteristics failed: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn suite() -> Suite {
        serde_json::from_str(include_str!("../fixtures/synthetic-soil-trial-oc.json"))
            .expect("suite fixture parses")
    }

    #[test]
    fn sign_flip_mask_enumerator_matches_independent_recursive_reference() {
        for values in [
            vec![1.0, 2.0, 3.0, 4.0],
            vec![-0.4, 0.2, 1.8, -2.1, 0.6],
            vec![0.0, 0.0, 1.0, -1.0, 2.0, 0.5],
        ] {
            let observed = values.iter().sum::<f64>() / values.len() as f64;
            assert!(
                (exact_sign_flip_p(&values, observed) - recursive_reference_p(&values, observed))
                    .abs()
                    < 1e-12
            );
        }
    }

    #[test]
    fn resolution_bound_matches_paired_sign_flip_discreteness() {
        assert!((min_two_sided_p(4) - 0.125).abs() < 1e-12);
        assert!((min_two_sided_p(6) - 0.03125).abs() < 1e-12);
        assert!((min_two_sided_p(8) - 0.0078125).abs() < 1e-12);
    }

    #[test]
    fn simulator_is_deterministic_for_fixed_seed_and_suite() {
        let s = suite();
        let a = simulate_scenario(&s, &s.scenarios[1], 1);
        let b = simulate_scenario(&s, &s.scenarios[1], 1);
        assert_eq!(
            serde_json::to_string(&a).unwrap(),
            serde_json::to_string(&b).unwrap()
        );
    }

    #[test]
    fn four_block_null_cannot_reject_at_five_percent() {
        let s = suite();
        let r = simulate_scenario(&s, &s.scenarios[0], 0);
        assert_eq!(r.minimum_attainable_two_sided_p_value, 0.125);
        assert_eq!(r.empirical_rejection_rate, Some(0.0));
    }

    #[test]
    fn missingness_and_interference_abstain_instead_of_dropping_units() {
        let s = suite();
        let missing = simulate_scenario(&s, &s.scenarios[5], 5);
        assert_eq!(
            missing.qualified_trial_count + missing.abstained_missing_outcome_count,
            1_200
        );
        assert!(missing.abstained_missing_outcome_count > 0);
        assert!(missing.qualified_trial_count < 1_200);

        let interference = simulate_scenario(&s, &s.scenarios[7], 7);
        assert_eq!(interference.qualified_trial_count, 0);
        assert_eq!(interference.abstained_interference_count, 1_200);
    }
}
