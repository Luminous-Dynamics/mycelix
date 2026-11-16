//! Aggregate-only evaluation of a fixed two-arm treatment policy in randomized data.
//!
//! This reference does not train policies or verify that supplied predictions are
//! out-of-fold. Those properties must be established by the frozen upstream protocol.

use std::collections::HashSet;

pub const DEFAULT_MIN_REPORT_N: usize = 20;
const PROBABILITY_TOLERANCE: f64 = 1.0e-9;
const NORMAL_95_Z: f64 = 1.959_963_984_540_054;

const EXPECTED_HEADER: &str = "observation_key\tassigned_arm\tp_arm_a\tp_arm_b\toutcome\tpolicy_candidate\tpolicy_comparator\tmu_arm_a\tmu_arm_b";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Arm {
    A,
    B,
}

impl Arm {
    fn parse(value: &str, field: &str, line: usize) -> Result<Self, String> {
        match value {
            "A" => Ok(Self::A),
            "B" => Ok(Self::B),
            _ => Err(format!("invalid {field} on line {line}; expected A or B")),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Observation {
    /// Opaque local-only key used for duplicate detection and deterministic ordering.
    /// It is never copied into the aggregate report.
    pub observation_key: String,
    pub assigned_arm: Arm,
    /// Randomization probabilities supplied by the trial protocol for this row.
    pub p_arm_a: f64,
    pub p_arm_b: f64,
    /// Outcome is oriented so that a larger value is better.
    pub outcome: f64,
    /// Actions recommended by two already-frozen policies.
    pub policy_candidate: Arm,
    pub policy_comparator: Arm,
    /// Arm-specific outcome predictions generated out-of-fold or on independent data.
    pub mu_arm_a: f64,
    pub mu_arm_b: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PolicyEstimate {
    pub dr_value: f64,
    pub dr_standard_error: f64,
    pub dr_ci95_lower: f64,
    pub dr_ci95_upper: f64,
    pub ipw_value: f64,
    pub matches_assigned: usize,
    pub coverage: f64,
    pub effective_sample_size: f64,
    pub max_inverse_probability_weight: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PolicyDifference {
    /// Candidate policy value minus comparator policy value.
    pub dr_value_difference: f64,
    pub paired_standard_error: f64,
    pub ci95_lower: f64,
    pub ci95_upper: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AggregateReport {
    pub n: usize,
    pub candidate: PolicyEstimate,
    pub comparator: PolicyEstimate,
    pub candidate_minus_comparator: PolicyDifference,
}

/// Parse the intentionally narrow TSV contract. Quoting, extra columns, blank rows,
/// duplicate keys and non-finite numbers are rejected rather than guessed.
pub fn parse_tsv(input: &str) -> Result<Vec<Observation>, String> {
    let mut lines = input.lines();
    let header = lines
        .next()
        .ok_or_else(|| "input is empty; expected a TSV header".to_string())?
        .trim_end_matches('\r');

    if header != EXPECTED_HEADER {
        return Err("input header does not match the exact v0.1 TSV contract".to_string());
    }

    let mut observations = Vec::new();
    let mut keys = HashSet::new();

    for (index, raw_line) in lines.enumerate() {
        let line_number = index + 2;
        let line = raw_line.trim_end_matches('\r');

        if line.is_empty() {
            return Err(format!("blank input row at line {line_number}"));
        }

        let fields: Vec<&str> = line.split('\t').collect();
        if fields.len() != 9 {
            return Err(format!("wrong column count on line {line_number}"));
        }

        let key = fields[0];
        if !valid_observation_key(key) {
            return Err(format!("invalid opaque observation key on line {line_number}"));
        }
        if !keys.insert(key.to_string()) {
            return Err(format!("duplicate observation key on line {line_number}"));
        }

        let assigned_arm = Arm::parse(fields[1], "assigned_arm", line_number)?;
        let p_arm_a = parse_finite(fields[2], "p_arm_a", line_number)?;
        let p_arm_b = parse_finite(fields[3], "p_arm_b", line_number)?;
        let outcome = parse_finite(fields[4], "outcome", line_number)?;
        let policy_candidate = Arm::parse(fields[5], "policy_candidate", line_number)?;
        let policy_comparator = Arm::parse(fields[6], "policy_comparator", line_number)?;
        let mu_arm_a = parse_finite(fields[7], "mu_arm_a", line_number)?;
        let mu_arm_b = parse_finite(fields[8], "mu_arm_b", line_number)?;

        let row = Observation {
            observation_key: key.to_string(),
            assigned_arm,
            p_arm_a,
            p_arm_b,
            outcome,
            policy_candidate,
            policy_comparator,
            mu_arm_a,
            mu_arm_b,
        };
        validate_observation(&row, line_number)?;
        observations.push(row);
    }

    if observations.is_empty() {
        return Err("input contains no observations".to_string());
    }

    Ok(observations)
}

/// Evaluate two fixed policies using augmented inverse-probability weighting.
///
/// For policy pi:
///   psi_i(pi) = mu_hat_pi(X_i)
///       + I(A_i = pi(X_i)) / e_Ai(X_i) * (Y_i - mu_hat_Ai(X_i))
///
/// Assignment probabilities must be the actual row-specific probabilities from
/// the randomized protocol. They are never estimated from the observed arm counts.
pub fn evaluate(observations: &[Observation]) -> Result<AggregateReport, String> {
    if observations.len() < 2 {
        return Err("at least two independent observations are required".to_string());
    }

    let mut rows = observations.to_vec();
    rows.sort_by(|left, right| left.observation_key.cmp(&right.observation_key));

    let mut keys = HashSet::new();
    for (index, row) in rows.iter().enumerate() {
        if !valid_observation_key(&row.observation_key) {
            return Err(format!("invalid opaque observation key at row {}", index + 1));
        }
        if !keys.insert(row.observation_key.clone()) {
            return Err(format!("duplicate observation key at row {}", index + 1));
        }
        validate_observation(row, index + 1)?;
    }

    let n = rows.len();
    let mut candidate_scores = Vec::with_capacity(n);
    let mut comparator_scores = Vec::with_capacity(n);
    let mut candidate_ipw_scores = Vec::with_capacity(n);
    let mut comparator_ipw_scores = Vec::with_capacity(n);
    let mut candidate_weights = Vec::new();
    let mut comparator_weights = Vec::new();
    let mut candidate_matches = 0usize;
    let mut comparator_matches = 0usize;

    for row in &rows {
        let candidate_score = dr_score(row, row.policy_candidate);
        let comparator_score = dr_score(row, row.policy_comparator);
        let candidate_ipw = ipw_score(row, row.policy_candidate);
        let comparator_ipw = ipw_score(row, row.policy_comparator);

        if !candidate_score.is_finite()
            || !comparator_score.is_finite()
            || !candidate_ipw.is_finite()
            || !comparator_ipw.is_finite()
        {
            return Err("non-finite estimator contribution; inspect propensity support".to_string());
        }

        if row.assigned_arm == row.policy_candidate {
            candidate_matches += 1;
            candidate_weights.push(inverse_assignment_probability(row)?);
        }
        if row.assigned_arm == row.policy_comparator {
            comparator_matches += 1;
            comparator_weights.push(inverse_assignment_probability(row)?);
        }

        candidate_scores.push(candidate_score);
        comparator_scores.push(comparator_score);
        candidate_ipw_scores.push(candidate_ipw);
        comparator_ipw_scores.push(comparator_ipw);
    }

    if candidate_matches == 0 || comparator_matches == 0 {
        return Err(
            "one or both policies have no observed randomized assignment support".to_string(),
        );
    }

    let candidate = summarize_policy(
        &candidate_scores,
        &candidate_ipw_scores,
        candidate_matches,
        &candidate_weights,
    )?;
    let comparator = summarize_policy(
        &comparator_scores,
        &comparator_ipw_scores,
        comparator_matches,
        &comparator_weights,
    )?;

    let paired_differences: Vec<f64> = candidate_scores
        .iter()
        .zip(comparator_scores.iter())
        .map(|(candidate, comparator)| candidate - comparator)
        .collect();

    if paired_differences.iter().any(|value| !value.is_finite()) {
        return Err("non-finite paired policy difference".to_string());
    }

    let difference_mean = stable_mean(&paired_differences)?;
    let difference_se = standard_error(&paired_differences, difference_mean)?;
    let difference = PolicyDifference {
        dr_value_difference: difference_mean,
        paired_standard_error: difference_se,
        ci95_lower: difference_mean - NORMAL_95_Z * difference_se,
        ci95_upper: difference_mean + NORMAL_95_Z * difference_se,
    };

    if !difference.ci95_lower.is_finite() || !difference.ci95_upper.is_finite() {
        return Err("non-finite confidence interval".to_string());
    }

    Ok(AggregateReport {
        n,
        candidate,
        comparator,
        candidate_minus_comparator: difference,
    })
}

/// Render only aggregate statistics. Observation keys and row-level scores are
/// intentionally absent from this output format.
pub fn render_json(report: &AggregateReport) -> String {
    let candidate = &report.candidate;
    let comparator = &report.comparator;
    let difference = &report.candidate_minus_comparator;

    format!(
        concat!(
            "{{\n",
            "  \"schema_version\": \"neurotwin-policy-value-report-v0.1\",\n",
            "  \"n\": {},\n",
            "  \"outcome_direction\": \"higher_is_better\",\n",
            "  \"estimator\": \"augmented_inverse_probability_weighted\",\n",
            "  \"uncertainty\": \"empirical_influence_function_normal_approximation_95pct\",\n",
            "  \"candidate_policy\": {},\n",
            "  \"comparator_policy\": {},\n",
            "  \"candidate_minus_comparator\": {{\n",
            "    \"dr_value_difference\": {},\n",
            "    \"paired_standard_error\": {},\n",
            "    \"ci95_lower\": {},\n",
            "    \"ci95_upper\": {}\n",
            "  }},\n",
            "  \"limitations\": [\n",
            "    \"Policies and arm-specific outcome predictions must be frozen and generated out-of-fold or on independent training data; this tool cannot verify that provenance.\",\n",
            "    \"Randomization probabilities must come from the trial protocol at the observation level; observational propensities are not supported.\",\n",
            "    \"Normal-approximation intervals are exploratory and may be unreliable in small samples; use a prespecified trial-appropriate inferential procedure for final analyses.\",\n",
            "    \"One independent observation per randomized participant and a fully observed prespecified outcome are required.\",\n",
            "    \"This report is not a treatment recommendation, clinical validation or proof of patient benefit.\"\n",
            "  ]\n",
            "}}\n"
        ),
        report.n,
        render_policy(candidate),
        render_policy(comparator),
        json_number(difference.dr_value_difference),
        json_number(difference.paired_standard_error),
        json_number(difference.ci95_lower),
        json_number(difference.ci95_upper)
    )
}

fn render_policy(estimate: &PolicyEstimate) -> String {
    format!(
        concat!(
            "{{\n",
            "    \"dr_value\": {},\n",
            "    \"dr_standard_error\": {},\n",
            "    \"dr_ci95_lower\": {},\n",
            "    \"dr_ci95_upper\": {},\n",
            "    \"ipw_value\": {},\n",
            "    \"matches_assigned\": {},\n",
            "    \"coverage\": {},\n",
            "    \"effective_sample_size\": {},\n",
            "    \"max_inverse_probability_weight\": {}\n",
            "  }}"
        ),
        json_number(estimate.dr_value),
        json_number(estimate.dr_standard_error),
        json_number(estimate.dr_ci95_lower),
        json_number(estimate.dr_ci95_upper),
        json_number(estimate.ipw_value),
        estimate.matches_assigned,
        json_number(estimate.coverage),
        json_number(estimate.effective_sample_size),
        json_number(estimate.max_inverse_probability_weight)
    )
}

fn json_number(value: f64) -> String {
    if value.abs() < 0.000_000_000_000_5 {
        "0.000000000000".to_string()
    } else {
        format!("{value:.12}")
    }
}

fn summarize_policy(
    dr_scores: &[f64],
    ipw_scores: &[f64],
    matches: usize,
    weights: &[f64],
) -> Result<PolicyEstimate, String> {
    let n = dr_scores.len();
    let dr_value = stable_mean(dr_scores)?;
    let dr_standard_error = standard_error(dr_scores, dr_value)?;
    let ipw_value = stable_mean(ipw_scores)?;
    let (effective_sample_size, max_inverse_probability_weight) = weight_diagnostics(weights);

    let estimate = PolicyEstimate {
        dr_value,
        dr_standard_error,
        dr_ci95_lower: dr_value - NORMAL_95_Z * dr_standard_error,
        dr_ci95_upper: dr_value + NORMAL_95_Z * dr_standard_error,
        ipw_value,
        matches_assigned: matches,
        coverage: matches as f64 / n as f64,
        effective_sample_size,
        max_inverse_probability_weight,
    };

    if [
        estimate.dr_ci95_lower,
        estimate.dr_ci95_upper,
        estimate.coverage,
        estimate.effective_sample_size,
        estimate.max_inverse_probability_weight,
    ]
    .iter()
    .any(|value| !value.is_finite())
    {
        return Err("non-finite policy summary".to_string());
    }

    Ok(estimate)
}

fn dr_score(row: &Observation, policy_arm: Arm) -> f64 {
    let mu_policy = prediction_for(row, policy_arm);
    if row.assigned_arm == policy_arm {
        let mu_observed = prediction_for(row, row.assigned_arm);
        mu_policy + (row.outcome - mu_observed) / propensity_for(row, row.assigned_arm)
    } else {
        mu_policy
    }
}

fn ipw_score(row: &Observation, policy_arm: Arm) -> f64 {
    if row.assigned_arm == policy_arm {
        row.outcome / propensity_for(row, row.assigned_arm)
    } else {
        0.0
    }
}

fn prediction_for(row: &Observation, arm: Arm) -> f64 {
    match arm {
        Arm::A => row.mu_arm_a,
        Arm::B => row.mu_arm_b,
    }
}

fn propensity_for(row: &Observation, arm: Arm) -> f64 {
    match arm {
        Arm::A => row.p_arm_a,
        Arm::B => row.p_arm_b,
    }
}

fn inverse_assignment_probability(row: &Observation) -> Result<f64, String> {
    let weight = 1.0 / propensity_for(row, row.assigned_arm);
    if weight.is_finite() {
        Ok(weight)
    } else {
        Err("inverse assignment probability overflow".to_string())
    }
}

fn weight_diagnostics(weights: &[f64]) -> (f64, f64) {
    if weights.is_empty() {
        return (0.0, 0.0);
    }
    let max_weight = weights.iter().copied().fold(0.0_f64, f64::max);
    if max_weight <= 0.0 || !max_weight.is_finite() {
        return (0.0, 0.0);
    }

    let scaled_sum: f64 = weights.iter().map(|weight| weight / max_weight).sum();
    let scaled_squares: f64 = weights
        .iter()
        .map(|weight| {
            let scaled = weight / max_weight;
            scaled * scaled
        })
        .sum();

    if scaled_squares <= 0.0 {
        (0.0, max_weight)
    } else {
        (scaled_sum * scaled_sum / scaled_squares, max_weight)
    }
}

fn stable_mean(values: &[f64]) -> Result<f64, String> {
    if values.is_empty() {
        return Err("cannot summarize an empty value set".to_string());
    }

    let mut sum = 0.0;
    let mut compensation = 0.0;
    for value in values {
        let adjusted = *value - compensation;
        let next = sum + adjusted;
        compensation = (next - sum) - adjusted;
        sum = next;
    }
    let mean = sum / values.len() as f64;
    if mean.is_finite() {
        Ok(mean)
    } else {
        Err("non-finite aggregate mean".to_string())
    }
}

fn standard_error(values: &[f64], mean: f64) -> Result<f64, String> {
    if values.len() < 2 {
        return Err("at least two values are required for uncertainty".to_string());
    }

    let mut sum_squared_deviations = 0.0;
    for value in values {
        let deviation = *value - mean;
        sum_squared_deviations += deviation * deviation;
    }
    let variance = sum_squared_deviations / (values.len() - 1) as f64;
    let standard_error = (variance / values.len() as f64).sqrt();

    if standard_error.is_finite() {
        Ok(standard_error)
    } else {
        Err("non-finite standard error".to_string())
    }
}

fn validate_observation(row: &Observation, line: usize) -> Result<(), String> {
    if !row.p_arm_a.is_finite()
        || !row.p_arm_b.is_finite()
        || row.p_arm_a <= 0.0
        || row.p_arm_b <= 0.0
        || row.p_arm_a >= 1.0
        || row.p_arm_b >= 1.0
    {
        return Err(format!("positivity violation in assignment probabilities at line {line}"));
    }
    if (row.p_arm_a + row.p_arm_b - 1.0).abs() > PROBABILITY_TOLERANCE {
        return Err(format!("assignment probabilities do not sum to one at line {line}"));
    }
    if !row.outcome.is_finite() || !row.mu_arm_a.is_finite() || !row.mu_arm_b.is_finite() {
        return Err(format!("non-finite outcome or prediction at line {line}"));
    }
    if !(1.0 / row.p_arm_a).is_finite() || !(1.0 / row.p_arm_b).is_finite() {
        return Err(format!("inverse assignment probability overflow at line {line}"));
    }
    Ok(())
}

fn parse_finite(value: &str, field: &str, line: usize) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| format!("invalid {field} on line {line}"))?;
    if !parsed.is_finite() {
        return Err(format!("non-finite {field} on line {line}"));
    }
    Ok(parsed)
}

fn valid_observation_key(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(
        key: &str,
        assigned_arm: Arm,
        outcome: f64,
        policy_candidate: Arm,
        policy_comparator: Arm,
    ) -> Observation {
        Observation {
            observation_key: key.to_string(),
            assigned_arm,
            p_arm_a: 0.5,
            p_arm_b: 0.5,
            outcome,
            policy_candidate,
            policy_comparator,
            mu_arm_a: 8.0,
            mu_arm_b: 4.0,
        }
    }

    fn hand_computable_rows() -> Vec<Observation> {
        vec![
            row("obs-1", Arm::A, 10.0, Arm::A, Arm::A),
            row("obs-2", Arm::B, 6.0, Arm::A, Arm::B),
            row("obs-3", Arm::A, 8.0, Arm::B, Arm::B),
            row("obs-4", Arm::B, 2.0, Arm::B, Arm::A),
        ]
    }

    #[test]
    fn hand_computable_aipw_values_and_paired_difference() {
        let report = evaluate(&hand_computable_rows()).unwrap();

        assert_eq!(report.n, 4);
        assert!((report.candidate.dr_value - 6.0).abs() < 1.0e-12);
        assert!((report.comparator.dr_value - 8.0).abs() < 1.0e-12);
        assert!((report.candidate_minus_comparator.dr_value_difference + 2.0).abs() < 1.0e-12);
        assert!((report.candidate_minus_comparator.paired_standard_error - 2.0).abs() < 1.0e-12);
        assert!((report.candidate.ipw_value - 6.0).abs() < 1.0e-12);
        assert!((report.comparator.ipw_value - 8.0).abs() < 1.0e-12);
        assert_eq!(report.candidate.matches_assigned, 2);
        assert_eq!(report.candidate.effective_sample_size, 2.0);
        assert_eq!(report.candidate.max_inverse_probability_weight, 2.0);
    }

    #[test]
    fn output_contains_aggregates_but_not_observation_keys() {
        let report = evaluate(&hand_computable_rows()).unwrap();
        let output = render_json(&report);
        assert!(output.contains("candidate_minus_comparator"));
        assert!(!output.contains("obs-1"));
        assert!(!output.contains("obs-2"));
    }

    #[test]
    fn result_is_invariant_to_input_row_order() {
        let rows = hand_computable_rows();
        let original = render_json(&evaluate(&rows).unwrap());
        let mut reversed = rows;
        reversed.reverse();
        let reordered = render_json(&evaluate(&reversed).unwrap());
        assert_eq!(original, reordered);
    }

    #[test]
    fn zero_probability_is_rejected() {
        let mut rows = hand_computable_rows();
        rows[0].p_arm_a = 0.0;
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn non_normalized_probabilities_are_rejected() {
        let mut rows = hand_computable_rows();
        rows[0].p_arm_a = 0.6;
        rows[0].p_arm_b = 0.6;
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn duplicate_observation_keys_are_rejected() {
        let mut rows = hand_computable_rows();
        rows[1].observation_key = rows[0].observation_key.clone();
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn non_finite_observations_are_rejected() {
        let mut rows = hand_computable_rows();
        rows[0].outcome = f64::NAN;
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn missing_or_single_observation_is_rejected() {
        assert!(evaluate(&[]).is_err());
        assert!(evaluate(&hand_computable_rows()[..1]).is_err());
    }

    #[test]
    fn a_policy_without_observed_randomized_support_is_rejected() {
        let mut rows = hand_computable_rows();
        for row in &mut rows {
            row.assigned_arm = Arm::A;
            row.policy_candidate = Arm::B;
            row.policy_comparator = Arm::A;
        }
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn committed_synthetic_fixture_parses_and_emits_aggregate_only() {
        let fixture = include_str!("../fixtures/synthetic-randomized-trial.tsv");
        let rows = parse_tsv(fixture).unwrap();
        assert!(rows.len() >= DEFAULT_MIN_REPORT_N);
        let output = render_json(&evaluate(&rows).unwrap());
        assert!(!output.contains("synthetic-observation-"));
        assert!(output.contains("candidate_minus_comparator"));
    }

    #[test]
    fn tsv_parser_requires_exact_header_and_rejects_duplicates() {
        let header = EXPECTED_HEADER;
        let good = format!(
            "{header}\nobs-a\tA\t0.5\t0.5\t10\tA\tA\t8\t4\nobs-b\tB\t0.5\t0.5\t6\tA\tB\t8\t4\n"
        );
        assert_eq!(parse_tsv(&good).unwrap().len(), 2);
        let bad_header = good.replacen("observation_key", "subject_id", 1);
        assert!(parse_tsv(&bad_header).is_err());
        let duplicate = format!(
            "{header}\nobs-a\tA\t0.5\t0.5\t10\tA\tA\t8\t4\nobs-a\tB\t0.5\t0.5\t6\tA\tB\t8\t4\n"
        );
        assert!(parse_tsv(&duplicate).is_err());
    }
}
