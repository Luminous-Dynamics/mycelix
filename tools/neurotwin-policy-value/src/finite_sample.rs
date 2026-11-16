//! Finite-sample, design-based inference for a paired two-arm AIPW policy contrast.
//!
//! Supported design: independent participant-level Bernoulli randomization, with
//! known row-specific assignment probabilities, fixed policies/predictions, bounded
//! fully observed outcomes, and no interference. Other designs are rejected.

use crate::{Arm, DEFAULT_MIN_REPORT_N, Observation};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RandomizationDesign {
    IndependentBernoulliParticipant,
    StratifiedOrBlocked,
    ClusterRandomized,
    RepeatedOrAdaptive,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FiniteSampleReport {
    pub n: usize,
    pub point_estimate: f64,
    pub confidence_delta: f64,
    pub confidence_level: f64,
    pub outcome_lower_bound: f64,
    pub outcome_upper_bound: f64,
    pub sum_squared_row_ranges: f64,
    pub hoeffding_radius: f64,
    pub finite_sample_lower: f64,
    pub finite_sample_upper: f64,
    pub normal_approximation_95_lower: f64,
    pub normal_approximation_95_upper: f64,
    pub minimum_assignment_probability: f64,
    pub maximum_inverse_assignment_probability: f64,
}

impl FiniteSampleReport {
    pub fn render_json(&self) -> String {
        format!(
            concat!(
                "{{\n",
                "  \"schema_version\": \"neurotwin-policy-finite-sample-report-v0.1\",\n",
                "  \"n\": {},\n",
                "  \"estimand\": \"finite_sample_average_candidate_minus_comparator_policy_value\",\n",
                "  \"estimator\": \"paired_augmented_inverse_probability_weighted_difference\",\n",
                "  \"randomization_unit\": \"participant\",\n",
                "  \"assignment_mechanism\": \"independent_bernoulli\",\n",
                "  \"outcome_bounds\": {{\"lower\": {}, \"upper\": {}}},\n",
                "  \"confidence_delta\": {},\n",
                "  \"confidence_level\": {},\n",
                "  \"point_estimate\": {},\n",
                "  \"finite_sample_interval\": {{\n",
                "    \"method\": \"two_sided_hoeffding_design_based\",\n",
                "    \"lower\": {},\n",
                "    \"upper\": {},\n",
                "    \"radius\": {},\n",
                "    \"sum_squared_row_ranges\": {}\n",
                "  }},\n",
                "  \"exploratory_normal_approximation_95\": {{\"lower\": {}, \"upper\": {}}},\n",
                "  \"randomization_diagnostics\": {{\n",
                "    \"minimum_assignment_probability\": {},\n",
                "    \"maximum_inverse_assignment_probability\": {}\n",
                "  }},\n",
                "  \"limitations\": [\n",
                "    \"Valid only for independent participant-level Bernoulli assignment with the supplied known row-specific probabilities; blocked, stratified, cluster, repeated-measure and adaptive designs are unsupported.\",\n",
                "    \"Requires one independent row per participant, no interference, fixed policies and frozen out-of-fold or independently trained predictions, and a fully observed outcome inside the declared bounds.\",\n",
                "    \"The finite-sample interval is a conservative Hoeffding design-based bound around the paired AIPW estimate; it may be wide, especially with weak positivity or small samples.\",\n",
                "    \"The tool cannot verify the source trial randomization protocol, policy lock, prediction provenance, outcome bounds or institutional authority.\",\n",
                "    \"Synthetic or retrospective output is not evidence of clinical utility, patient benefit or a treatment recommendation.\"\n",
                "  ]\n",
                "}}\n"
            ),
            self.n,
            json_number(self.outcome_lower_bound),
            json_number(self.outcome_upper_bound),
            json_number(self.confidence_delta),
            json_number(self.confidence_level),
            json_number(self.point_estimate),
            json_number(self.finite_sample_lower),
            json_number(self.finite_sample_upper),
            json_number(self.hoeffding_radius),
            json_number(self.sum_squared_row_ranges),
            json_number(self.normal_approximation_95_lower),
            json_number(self.normal_approximation_95_upper),
            json_number(self.minimum_assignment_probability),
            json_number(self.maximum_inverse_assignment_probability),
        )
    }
}

/// Computes a finite-sample interval for the average policy-value contrast over the
/// evaluated participants. The guarantee is over the independent random assignments,
/// conditional on fixed potential outcomes, policies and prediction functions.
pub fn evaluate_finite_sample(
    observations: &[Observation],
    design: RandomizationDesign,
    outcome_lower_bound: f64,
    outcome_upper_bound: f64,
    confidence_delta: f64,
) -> Result<FiniteSampleReport, String> {
    if design != RandomizationDesign::IndependentBernoulliParticipant {
        return Err(
            "finite-sample v0.1 supports only independent participant-level Bernoulli randomization"
                .to_string(),
        );
    }
    if observations.len() < DEFAULT_MIN_REPORT_N {
        return Err(format!(
            "finite-sample aggregate suppressed below the built-in reporting floor of {DEFAULT_MIN_REPORT_N}"
        ));
    }
    if !outcome_lower_bound.is_finite()
        || !outcome_upper_bound.is_finite()
        || outcome_lower_bound >= outcome_upper_bound
    {
        return Err("outcome bounds must be finite with lower strictly below upper".to_string());
    }
    if !confidence_delta.is_finite() || confidence_delta <= 0.0 || confidence_delta >= 1.0 {
        return Err("confidence delta must be strictly between zero and one".to_string());
    }

    // Reuse the narrow v0.1 parser/evaluator's duplicate-key, probability, support,
    // finite-value and paired-estimator validation. Its normal interval is reported
    // only as a comparison; it is not the finite-sample guarantee.
    let reference = crate::evaluate(observations)?;
    let n = observations.len();
    let mut sum_squared_row_ranges = 0.0_f64;
    let mut minimum_assignment_probability = 1.0_f64;
    let mut maximum_inverse_assignment_probability = 1.0_f64;

    for (index, row) in observations.iter().enumerate() {
        if row.outcome < outcome_lower_bound || row.outcome > outcome_upper_bound {
            return Err(format!(
                "outcome outside declared bounds at row {}",
                index + 1
            ));
        }
        let mu_a = prediction_for(row, Arm::A);
        let mu_b = prediction_for(row, Arm::B);
        if mu_a < outcome_lower_bound
            || mu_a > outcome_upper_bound
            || mu_b < outcome_lower_bound
            || mu_b > outcome_upper_bound
        {
            return Err(format!(
                "arm prediction outside declared outcome bounds at row {}",
                index + 1
            ));
        }

        minimum_assignment_probability =
            minimum_assignment_probability.min(row.p_arm_a.min(row.p_arm_b));
        maximum_inverse_assignment_probability =
            maximum_inverse_assignment_probability.max((1.0 / row.p_arm_a).max(1.0 / row.p_arm_b));

        let (lower, upper) = row_contrast_bounds(row, outcome_lower_bound, outcome_upper_bound);
        let range = upper - lower;
        if !lower.is_finite() || !upper.is_finite() || !range.is_finite() || range < 0.0 {
            return Err("non-finite row-wise contribution range".to_string());
        }
        sum_squared_row_ranges += range * range;
        if !sum_squared_row_ranges.is_finite() {
            return Err("sum of squared row-wise ranges overflowed".to_string());
        }
    }

    // Hoeffding's inequality for independent, not necessarily identically distributed,
    // bounded row contributions: P(|mean - E mean| >= r) <= delta.
    let n_float = n as f64;
    let radius =
        (sum_squared_row_ranges * (2.0 / confidence_delta).ln() / (2.0 * n_float * n_float)).sqrt();
    if !radius.is_finite() {
        return Err("finite-sample confidence radius is not finite".to_string());
    }

    let point_estimate = reference.candidate_minus_comparator.dr_value_difference;
    let report = FiniteSampleReport {
        n,
        point_estimate,
        confidence_delta,
        confidence_level: 1.0 - confidence_delta,
        outcome_lower_bound,
        outcome_upper_bound,
        sum_squared_row_ranges,
        hoeffding_radius: radius,
        finite_sample_lower: point_estimate - radius,
        finite_sample_upper: point_estimate + radius,
        normal_approximation_95_lower: reference.candidate_minus_comparator.ci95_lower,
        normal_approximation_95_upper: reference.candidate_minus_comparator.ci95_upper,
        minimum_assignment_probability,
        maximum_inverse_assignment_probability,
    };

    if [
        report.point_estimate,
        report.confidence_level,
        report.finite_sample_lower,
        report.finite_sample_upper,
        report.normal_approximation_95_lower,
        report.normal_approximation_95_upper,
        report.minimum_assignment_probability,
        report.maximum_inverse_assignment_probability,
    ]
    .iter()
    .any(|value| !value.is_finite())
    {
        return Err("finite-sample report contains a non-finite value".to_string());
    }
    Ok(report)
}

fn row_contrast_bounds(
    row: &Observation,
    outcome_lower_bound: f64,
    outcome_upper_bound: f64,
) -> (f64, f64) {
    if row.policy_candidate == row.policy_comparator {
        return (0.0, 0.0);
    }

    let candidate_prediction = prediction_for(row, row.policy_candidate);
    let comparator_prediction = prediction_for(row, row.policy_comparator);
    let candidate_probability = probability_for(row, row.policy_candidate);
    let comparator_probability = probability_for(row, row.policy_comparator);
    let baseline = candidate_prediction - comparator_prediction;

    // If the randomized assignment matches the candidate action, the residual
    // correction is (Y - mu_candidate) / e_candidate. If it matches the comparator,
    // the paired contrast contains -(Y - mu_comparator) / e_comparator.
    let candidate_min = (outcome_lower_bound - candidate_prediction) / candidate_probability;
    let candidate_max = (outcome_upper_bound - candidate_prediction) / candidate_probability;
    let comparator_min = (comparator_prediction - outcome_upper_bound) / comparator_probability;
    let comparator_max = (comparator_prediction - outcome_lower_bound) / comparator_probability;

    (
        baseline + candidate_min.min(comparator_min),
        baseline + candidate_max.max(comparator_max),
    )
}

fn prediction_for(row: &Observation, arm: Arm) -> f64 {
    match arm {
        Arm::A => row.mu_arm_a,
        Arm::B => row.mu_arm_b,
    }
}

fn probability_for(row: &Observation, arm: Arm) -> f64 {
    match arm {
        Arm::A => row.p_arm_a,
        Arm::B => row.p_arm_b,
    }
}

fn json_number(value: f64) -> String {
    if value.abs() < 0.000_000_000_000_5 {
        "0.000000000000".to_string()
    } else {
        format!("{value:.12}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(
        key: &str,
        assigned_arm: Arm,
        candidate: Arm,
        comparator: Arm,
        outcome: f64,
        p_a: f64,
        mu_a: f64,
        mu_b: f64,
    ) -> Observation {
        Observation {
            observation_key: key.to_string(),
            assigned_arm,
            p_arm_a: p_a,
            p_arm_b: 1.0 - p_a,
            outcome,
            policy_candidate: candidate,
            policy_comparator: comparator,
            mu_arm_a: mu_a,
            mu_arm_b: mu_b,
        }
    }

    fn fixture_rows() -> Vec<Observation> {
        (0..40)
            .map(|index| {
                let assigned = if index % 2 == 0 { Arm::A } else { Arm::B };
                observation(
                    &format!("synthetic-{index:03}"),
                    assigned,
                    Arm::A,
                    Arm::B,
                    if assigned == Arm::A { 0.7 } else { 0.4 },
                    0.5,
                    0.45,
                    0.55,
                )
            })
            .collect()
    }

    #[test]
    fn row_range_contains_both_possible_assignment_contributions() {
        let row = observation("row", Arm::A, Arm::A, Arm::B, 0.7, 0.5, 0.4, 0.6);
        let (lower, upper) = row_contrast_bounds(&row, 0.0, 1.0);
        assert!((lower + 1.0).abs() < 1.0e-12);
        assert!((upper - 1.0).abs() < 1.0e-12);
        let candidate_assignment_score = (0.4 - 0.6) + (0.7 - 0.4) / 0.5;
        let comparator_assignment_score = (0.4 - 0.6) - (0.4 - 0.6) / 0.5;
        assert!((candidate_assignment_score - 0.4).abs() < 1.0e-12);
        assert!((comparator_assignment_score - 0.2).abs() < 1.0e-12);
        assert!(candidate_assignment_score >= lower && candidate_assignment_score <= upper);
        assert!(comparator_assignment_score >= lower && comparator_assignment_score <= upper);
    }

    #[test]
    fn identical_policies_have_zero_contrast_range() {
        let row = observation("row", Arm::A, Arm::A, Arm::A, 0.7, 0.5, 0.4, 0.6);
        assert_eq!(row_contrast_bounds(&row, 0.0, 1.0), (0.0, 0.0));
    }

    #[test]
    fn finite_sample_report_uses_paired_design_based_interval() {
        let report = evaluate_finite_sample(
            &fixture_rows(),
            RandomizationDesign::IndependentBernoulliParticipant,
            0.0,
            1.0,
            0.05,
        )
        .unwrap();
        assert_eq!(report.n, 40);
        assert_eq!(report.confidence_level, 0.95);
        assert!(report.finite_sample_lower <= report.point_estimate);
        assert!(report.finite_sample_upper >= report.point_estimate);
        assert!(report.hoeffding_radius > 0.0);
        let json = report.render_json();
        assert!(json.contains("two_sided_hoeffding_design_based"));
        assert!(!json.contains("synthetic-"));
    }

    #[test]
    fn unsupported_randomization_designs_fail_closed() {
        for design in [
            RandomizationDesign::StratifiedOrBlocked,
            RandomizationDesign::ClusterRandomized,
            RandomizationDesign::RepeatedOrAdaptive,
        ] {
            assert!(evaluate_finite_sample(&fixture_rows(), design, 0.0, 1.0, 0.05).is_err());
        }
    }

    #[test]
    fn outcomes_and_predictions_must_obey_declared_bounds() {
        let mut rows = fixture_rows();
        rows[0].outcome = 1.1;
        assert!(
            evaluate_finite_sample(
                &rows,
                RandomizationDesign::IndependentBernoulliParticipant,
                0.0,
                1.0,
                0.05,
            )
            .is_err()
        );
        rows = fixture_rows();
        rows[0].mu_arm_a = -0.1;
        assert!(
            evaluate_finite_sample(
                &rows,
                RandomizationDesign::IndependentBernoulliParticipant,
                0.0,
                1.0,
                0.05,
            )
            .is_err()
        );
    }

    #[test]
    fn invalid_bounds_and_confidence_delta_are_rejected() {
        let rows = fixture_rows();
        assert!(
            evaluate_finite_sample(
                &rows,
                RandomizationDesign::IndependentBernoulliParticipant,
                1.0,
                0.0,
                0.05,
            )
            .is_err()
        );
        assert!(
            evaluate_finite_sample(
                &rows,
                RandomizationDesign::IndependentBernoulliParticipant,
                0.0,
                1.0,
                1.0,
            )
            .is_err()
        );
    }

    #[test]
    fn deterministic_randomization_simulation_covers_finite_sample_policy_effect() {
        let n = 240_usize;
        let replicates = 250_usize;
        let mut random_state = 0x4e54_5754_494e_2026_u64;
        let mut covered = 0_usize;

        for replicate in 0..replicates {
            let mut rows = Vec::with_capacity(n);
            let mut truth = 0.0_f64;
            for index in 0..n {
                let y_a = ((index * 17 + replicate * 3) % 101) as f64 / 100.0;
                let y_b = ((index * 29 + replicate * 7 + 13) % 101) as f64 / 100.0;
                truth += y_a - y_b;
                let p_a = if index % 2 == 0 { 0.30 } else { 0.70 };
                random_state = random_state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                let draw = (random_state >> 11) as f64 / ((1_u64 << 53) as f64);
                let assigned_arm = if draw < p_a { Arm::A } else { Arm::B };
                let outcome = if assigned_arm == Arm::A { y_a } else { y_b };
                rows.push(observation(
                    &format!("r{replicate}-p{index}"),
                    assigned_arm,
                    Arm::A,
                    Arm::B,
                    outcome,
                    p_a,
                    0.20, // deliberately misspecified, but fixed and outcome-bounded
                    0.80,
                ));
            }
            truth /= n as f64;

            let report = evaluate_finite_sample(
                &rows,
                RandomizationDesign::IndependentBernoulliParticipant,
                0.0,
                1.0,
                0.05,
            )
            .unwrap();
            if report.finite_sample_lower <= truth && truth <= report.finite_sample_upper {
                covered += 1;
            }
        }

        // A conservative finite-sample bound should cover at least 95% in repeated
        // independent-randomization experiments; 90% is the prespecified Monte Carlo
        // tolerance for this deterministic, finite-replicate regression test.
        assert!(
            covered >= 225,
            "coverage {covered}/{replicates} fell below the Monte Carlo regression floor"
        );
    }
}
