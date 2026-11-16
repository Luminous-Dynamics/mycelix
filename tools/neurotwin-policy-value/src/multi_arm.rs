//! Multi-arm AIPW evaluation for fixed policies in randomized trials.
//! This module supports 2–8 arms with known, row-specific randomization probabilities.
//! Policies and arm-specific predictions must be frozen and out-of-fold or independently trained.

use std::collections::HashSet;

pub const MIN_ARMS: usize = 2;
pub const MAX_ARMS: usize = 8;
pub const DEFAULT_MIN_REPORT_N: usize = 20;
pub const SMALL_CELL_SUPPRESSION_THRESHOLD: usize = 5;
const TOLERANCE: f64 = 1.0e-9;
const Z95: f64 = 1.959_963_984_540_054;
const PREFIX: [&str; 5] = [
    "observation_key",
    "assigned_arm",
    "outcome",
    "policy_candidate",
    "policy_comparator",
];

#[derive(Clone, Debug, PartialEq)]
pub struct Observation {
    pub observation_key: String,
    pub assigned_arm: usize,
    pub probabilities: Vec<f64>,
    pub outcome: f64,
    pub candidate_arm: usize,
    pub comparator_arm: usize,
    pub predictions: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Dataset {
    pub arms: Vec<String>,
    pub observations: Vec<Observation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PolicyEstimate {
    pub value: f64,
    pub standard_error: f64,
    pub ci95_lower: f64,
    pub ci95_upper: f64,
    pub ipw_value: f64,
    pub matched_count: usize,
    /// Fraction of rows where observed randomized arm matched the policy choice.
    pub observed_match_rate: f64,
    pub effective_sample_size: f64,
    pub maximum_inverse_probability_weight: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Difference {
    pub value_difference: f64,
    pub paired_standard_error: f64,
    pub ci95_lower: f64,
    pub ci95_upper: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArmSupport {
    pub arm: String,
    pub randomized_assignments: usize,
    pub candidate_policy_selections: usize,
    pub candidate_observed_matches: usize,
    pub comparator_policy_selections: usize,
    pub comparator_observed_matches: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub n: usize,
    pub arms: Vec<String>,
    pub candidate: PolicyEstimate,
    pub comparator: PolicyEstimate,
    pub candidate_minus_comparator: Difference,
    pub arm_support: Vec<ArmSupport>,
}

/// Exact TSV header:
/// observation_key, assigned_arm, outcome, policy_candidate, policy_comparator,
/// probability:<arm>..., prediction:<arm>...
pub fn parse_tsv(input: &str) -> Result<Dataset, String> {
    let mut lines = input.lines();
    let header = lines
        .next()
        .ok_or_else(|| "input is empty".to_string())?
        .trim_end_matches('\r');
    let columns: Vec<&str> = header.split('\t').collect();
    if columns.len() < PREFIX.len() + 2 * MIN_ARMS
        || !columns.iter().take(PREFIX.len()).copied().eq(PREFIX)
    {
        return Err("header does not match the multi-arm v0.1 contract".to_string());
    }
    let k = (columns.len() - PREFIX.len()) / 2;
    if !(MIN_ARMS..=MAX_ARMS).contains(&k) || columns.len() != PREFIX.len() + 2 * k {
        return Err(format!("expected between {MIN_ARMS} and {MAX_ARMS} arms"));
    }

    let mut arms = Vec::with_capacity(k);
    let mut seen_arms = HashSet::new();
    for column in &columns[PREFIX.len()..PREFIX.len() + k] {
        let arm = column
            .strip_prefix("probability:")
            .ok_or_else(|| "probability columns must precede prediction columns".to_string())?;
        if !valid_arm(arm) || !seen_arms.insert(arm.to_string()) {
            return Err("arm keys must be unique ASCII letters, digits, '_' or '-'".to_string());
        }
        arms.push(arm.to_string());
    }
    for (index, column) in columns[PREFIX.len() + k..].iter().enumerate() {
        let arm = column
            .strip_prefix("prediction:")
            .ok_or_else(|| "expected prediction:<arm> columns".to_string())?;
        if arm != arms[index] {
            return Err("prediction arm order must match probability arm order".to_string());
        }
    }

    let mut observations = Vec::new();
    let mut seen_keys = HashSet::new();
    for (row_index, raw) in lines.enumerate() {
        let line_number = row_index + 2;
        let raw = raw.trim_end_matches('\r');
        if raw.is_empty() {
            return Err(format!("blank row at line {line_number}"));
        }
        let fields: Vec<&str> = raw.split('\t').collect();
        if fields.len() != columns.len() {
            return Err(format!("wrong column count at line {line_number}"));
        }
        if !valid_key(fields[0]) || !seen_keys.insert(fields[0].to_string()) {
            return Err(format!(
                "invalid or duplicate observation key at line {line_number}"
            ));
        }

        let assigned_arm = find_arm(&arms, fields[1])
            .ok_or_else(|| format!("unknown assigned arm at line {line_number}"))?;
        let outcome = parse_finite(fields[2], "outcome", line_number)?;
        let candidate_arm = find_arm(&arms, fields[3])
            .ok_or_else(|| format!("unknown candidate arm at line {line_number}"))?;
        let comparator_arm = find_arm(&arms, fields[4])
            .ok_or_else(|| format!("unknown comparator arm at line {line_number}"))?;
        let mut probabilities = Vec::with_capacity(k);
        let mut predictions = Vec::with_capacity(k);
        for index in 0..k {
            probabilities.push(parse_finite(
                fields[PREFIX.len() + index],
                "assignment probability",
                line_number,
            )?);
        }
        for index in 0..k {
            predictions.push(parse_finite(
                fields[PREFIX.len() + k + index],
                "arm prediction",
                line_number,
            )?);
        }

        let observation = Observation {
            observation_key: fields[0].to_string(),
            assigned_arm,
            probabilities,
            outcome,
            candidate_arm,
            comparator_arm,
            predictions,
        };
        validate_observation(&observation, k, line_number)?;
        observations.push(observation);
    }
    if observations.is_empty() {
        return Err("input contains no observations".to_string());
    }
    Ok(Dataset { arms, observations })
}

/// For policy pi, psi_i = mu_hat_pi(X_i) + I(A_i=pi(X_i))/e_i(A_i)
/// * (Y_i - mu_hat_A_i(X_i)). Assignment probabilities must come from
/// the randomized trial protocol, not the observed arm frequencies.
pub fn evaluate(dataset: &Dataset) -> Result<Report, String> {
    let k = dataset.arms.len();
    if !(MIN_ARMS..=MAX_ARMS).contains(&k) {
        return Err(format!(
            "arm count must be between {MIN_ARMS} and {MAX_ARMS}"
        ));
    }
    if dataset.observations.len() < 2 {
        return Err("at least two independent observations are required".to_string());
    }
    let mut arm_set = HashSet::new();
    for arm in &dataset.arms {
        if !valid_arm(arm) || !arm_set.insert(arm.clone()) {
            return Err("arm registry contains an invalid or duplicate key".to_string());
        }
    }

    let mut rows = dataset.observations.clone();
    rows.sort_by(|a, b| a.observation_key.cmp(&b.observation_key));
    let mut key_set = HashSet::new();
    for (index, row) in rows.iter().enumerate() {
        if !valid_key(&row.observation_key) || !key_set.insert(row.observation_key.clone()) {
            return Err(format!(
                "invalid or duplicate observation key at row {}",
                index + 1
            ));
        }
        validate_observation(row, k, index + 1)?;
    }

    let mut candidate_scores = Vec::with_capacity(rows.len());
    let mut comparator_scores = Vec::with_capacity(rows.len());
    let mut candidate_ipw = Vec::with_capacity(rows.len());
    let mut comparator_ipw = Vec::with_capacity(rows.len());
    let mut candidate_weights = Vec::new();
    let mut comparator_weights = Vec::new();
    let mut support: Vec<ArmSupport> = dataset
        .arms
        .iter()
        .map(|arm| ArmSupport {
            arm: arm.clone(),
            randomized_assignments: 0,
            candidate_policy_selections: 0,
            candidate_observed_matches: 0,
            comparator_policy_selections: 0,
            comparator_observed_matches: 0,
        })
        .collect();

    for row in &rows {
        support[row.assigned_arm].randomized_assignments += 1;
        support[row.candidate_arm].candidate_policy_selections += 1;
        support[row.comparator_arm].comparator_policy_selections += 1;

        let candidate_score = dr_score(row, row.candidate_arm);
        let comparator_score = dr_score(row, row.comparator_arm);
        let candidate_ipw_score = ipw_score(row, row.candidate_arm);
        let comparator_ipw_score = ipw_score(row, row.comparator_arm);
        if [
            candidate_score,
            comparator_score,
            candidate_ipw_score,
            comparator_ipw_score,
        ]
        .iter()
        .any(|v| !v.is_finite())
        {
            return Err("non-finite estimator contribution".to_string());
        }

        if row.assigned_arm == row.candidate_arm {
            support[row.candidate_arm].candidate_observed_matches += 1;
            candidate_weights.push(1.0 / row.probabilities[row.assigned_arm]);
        }
        if row.assigned_arm == row.comparator_arm {
            support[row.comparator_arm].comparator_observed_matches += 1;
            comparator_weights.push(1.0 / row.probabilities[row.assigned_arm]);
        }
        candidate_scores.push(candidate_score);
        comparator_scores.push(comparator_score);
        candidate_ipw.push(candidate_ipw_score);
        comparator_ipw.push(comparator_ipw_score);
    }

    for item in &support {
        if item.candidate_policy_selections > 0 && item.candidate_observed_matches == 0 {
            return Err(format!(
                "candidate policy arm '{}' has no observed randomized assignment support",
                item.arm
            ));
        }
        if item.comparator_policy_selections > 0 && item.comparator_observed_matches == 0 {
            return Err(format!(
                "comparator policy arm '{}' has no observed randomized assignment support",
                item.arm
            ));
        }
    }

    let candidate = summarize_policy(&candidate_scores, &candidate_ipw, &candidate_weights)?;
    let comparator = summarize_policy(&comparator_scores, &comparator_ipw, &comparator_weights)?;
    let paired: Vec<f64> = candidate_scores
        .iter()
        .zip(&comparator_scores)
        .map(|(a, b)| a - b)
        .collect();
    let difference_mean = mean(&paired)?;
    let difference_se = se(&paired, difference_mean)?;
    let difference = Difference {
        value_difference: difference_mean,
        paired_standard_error: difference_se,
        ci95_lower: difference_mean - Z95 * difference_se,
        ci95_upper: difference_mean + Z95 * difference_se,
    };
    if [
        difference.value_difference,
        difference.paired_standard_error,
        difference.ci95_lower,
        difference.ci95_upper,
    ]
    .iter()
    .any(|v| !v.is_finite())
    {
        return Err("non-finite paired policy difference".to_string());
    }

    Ok(Report {
        n: rows.len(),
        arms: dataset.arms.clone(),
        candidate,
        comparator,
        candidate_minus_comparator: difference,
        arm_support: support,
    })
}

pub fn render_json(report: &Report) -> String {
    let suppress_arm_support = report.arm_support.iter().any(|item| {
        [
            item.randomized_assignments,
            item.candidate_policy_selections,
            item.candidate_observed_matches,
            item.comparator_policy_selections,
            item.comparator_observed_matches,
        ]
        .iter()
        .any(|count| *count > 0 && *count < SMALL_CELL_SUPPRESSION_THRESHOLD)
    });

    let mut out = String::new();
    out.push_str("{\n");
    out.push_str("  \"schema_version\": \"neurotwin-multiarm-policy-value-report-v0.1\",\n");
    out.push_str(&format!("  \"n\": {},\n", report.n));
    out.push_str("  \"estimator\": \"augmented_inverse_probability_weighted\",\n");
    out.push_str(
        "  \"uncertainty\": \"empirical_influence_function_normal_approximation_95pct\",\n",
    );
    out.push_str("  \"arms\": [");
    for (index, arm) in report.arms.iter().enumerate() {
        if index > 0 {
            out.push_str(", ");
        }
        out.push('"');
        out.push_str(arm);
        out.push('"');
    }
    out.push_str("],\n  \"candidate_policy\": ");
    out.push_str(&render_policy(&report.candidate));
    out.push_str(",\n  \"comparator_policy\": ");
    out.push_str(&render_policy(&report.comparator));
    out.push_str(",\n  \"candidate_minus_comparator\": {");
    out.push_str(&format!(
        "\n    \"value_difference\": {},\n    \"paired_standard_error\": {},\n    \"ci95_lower\": {},\n    \"ci95_upper\": {}\n  }},\n",
        number(report.candidate_minus_comparator.value_difference),
        number(report.candidate_minus_comparator.paired_standard_error),
        number(report.candidate_minus_comparator.ci95_lower),
        number(report.candidate_minus_comparator.ci95_upper)
    ));
    out.push_str(&format!(
        "  \"arm_support_suppressed\": {},\n  \"arm_support_suppression_threshold\": {},\n",
        suppress_arm_support, SMALL_CELL_SUPPRESSION_THRESHOLD
    ));
    if suppress_arm_support {
        out.push_str("  \"arm_support\": null,\n");
    } else {
        out.push_str("  \"arm_support\": [");
        for (index, item) in report.arm_support.iter().enumerate() {
            if index > 0 {
                out.push_str(",");
            }
            out.push_str(&format!(
                "{{\"arm\":\"{}\",\"randomized_assignments\":{},\"candidate_policy_selections\":{},\"candidate_observed_matches\":{},\"comparator_policy_selections\":{},\"comparator_observed_matches\":{}}}",
                item.arm,
                item.randomized_assignments,
                item.candidate_policy_selections,
                item.candidate_observed_matches,
                item.comparator_policy_selections,
                item.comparator_observed_matches
            ));
        }
        out.push_str("],\n");
    }
    out.push_str("  \"limitations\": [\n");
    out.push_str("    \"Policies and arm-specific outcome predictions must be frozen and generated out-of-fold or on independent training data.\",\n");
    out.push_str("    \"Randomization probabilities must come from the trial protocol; they are not inferred from observed arm frequencies.\",\n");
    out.push_str("    \"Per-arm support counts are withheld if any nonzero cell is below the suppression threshold.\",\n");
    out.push_str("    \"One independent observation per randomized participant and a fully observed prespecified outcome are required.\",\n");
    out.push_str("    \"Normal-approximation intervals are exploratory and may be unreliable in small samples.\",\n");
    out.push_str("    \"This report is not a treatment recommendation, clinical validation or proof of patient benefit.\"\n");
    out.push_str("  ]\n");
    out.push_str("}\n");
    out
}

fn render_policy(x: &PolicyEstimate) -> String {
    format!(
        "{{\"value\":{},\"standard_error\":{},\"ci95_lower\":{},\"ci95_upper\":{},\"ipw_value\":{},\"matched_count\":{},\"observed_match_rate\":{},\"effective_sample_size\":{},\"maximum_inverse_probability_weight\":{}}}",
        number(x.value),
        number(x.standard_error),
        number(x.ci95_lower),
        number(x.ci95_upper),
        number(x.ipw_value),
        x.matched_count,
        number(x.observed_match_rate),
        number(x.effective_sample_size),
        number(x.maximum_inverse_probability_weight)
    )
}

fn summarize_policy(
    scores: &[f64],
    ipw: &[f64],
    weights: &[f64],
) -> Result<PolicyEstimate, String> {
    if weights.is_empty() {
        return Err("policy has no observed randomized assignment support".to_string());
    }
    let value = mean(scores)?;
    let standard_error = se(scores, value)?;
    let ipw_value = mean(ipw)?;
    let (ess, max_weight) = weight_diagnostics(weights);
    let result = PolicyEstimate {
        value,
        standard_error,
        ci95_lower: value - Z95 * standard_error,
        ci95_upper: value + Z95 * standard_error,
        ipw_value,
        matched_count: weights.len(),
        observed_match_rate: weights.len() as f64 / scores.len() as f64,
        effective_sample_size: ess,
        maximum_inverse_probability_weight: max_weight,
    };
    if [
        result.value,
        result.standard_error,
        result.ci95_lower,
        result.ci95_upper,
        result.ipw_value,
        result.observed_match_rate,
        result.effective_sample_size,
        result.maximum_inverse_probability_weight,
    ]
    .iter()
    .any(|v| !v.is_finite())
    {
        return Err("non-finite policy summary".to_string());
    }
    Ok(result)
}

fn dr_score(row: &Observation, policy_arm: usize) -> f64 {
    let mu_policy = row.predictions[policy_arm];
    if row.assigned_arm == policy_arm {
        mu_policy
            + (row.outcome - row.predictions[row.assigned_arm])
                / row.probabilities[row.assigned_arm]
    } else {
        mu_policy
    }
}

fn ipw_score(row: &Observation, policy_arm: usize) -> f64 {
    if row.assigned_arm == policy_arm {
        row.outcome / row.probabilities[row.assigned_arm]
    } else {
        0.0
    }
}

fn validate_observation(row: &Observation, k: usize, line: usize) -> Result<(), String> {
    if row.assigned_arm >= k
        || row.candidate_arm >= k
        || row.comparator_arm >= k
        || row.probabilities.len() != k
        || row.predictions.len() != k
    {
        return Err(format!(
            "arm index or vector length mismatch at line {line}"
        ));
    }
    let mut sum = 0.0;
    for p in &row.probabilities {
        if !p.is_finite() || *p <= 0.0 || *p >= 1.0 {
            return Err(format!("positivity violation at line {line}"));
        }
        if !(1.0 / *p).is_finite() {
            return Err(format!("inverse probability overflow at line {line}"));
        }
        sum += *p;
    }
    if (sum - 1.0).abs() > TOLERANCE {
        return Err(format!(
            "assignment probabilities do not sum to one at line {line}"
        ));
    }
    if !row.outcome.is_finite() || row.predictions.iter().any(|v| !v.is_finite()) {
        return Err(format!("non-finite outcome or prediction at line {line}"));
    }
    Ok(())
}

fn parse_finite(value: &str, field: &str, line: usize) -> Result<f64, String> {
    let value = value
        .parse::<f64>()
        .map_err(|_| format!("invalid {field} at line {line}"))?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(format!("non-finite {field} at line {line}"))
    }
}

fn find_arm(arms: &[String], value: &str) -> Option<usize> {
    arms.iter().position(|arm| arm == value)
}

fn valid_arm(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 64
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-".contains(&b))
}

fn valid_key(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"-_.:".contains(&b))
}

fn mean(values: &[f64]) -> Result<f64, String> {
    if values.is_empty() {
        return Err("cannot summarize empty values".to_string());
    }
    let mut sum = 0.0;
    let mut compensation = 0.0;
    for value in values {
        let y = *value - compensation;
        let t = sum + y;
        compensation = (t - sum) - y;
        sum = t;
    }
    let result = sum / values.len() as f64;
    if result.is_finite() {
        Ok(result)
    } else {
        Err("non-finite mean".to_string())
    }
}

fn se(values: &[f64], avg: f64) -> Result<f64, String> {
    if values.len() < 2 {
        return Err("at least two values are required for uncertainty".to_string());
    }
    let ss: f64 = values.iter().map(|v| (*v - avg).powi(2)).sum();
    let result = (ss / (values.len() - 1) as f64 / values.len() as f64).sqrt();
    if result.is_finite() {
        Ok(result)
    } else {
        Err("non-finite standard error".to_string())
    }
}

fn weight_diagnostics(weights: &[f64]) -> (f64, f64) {
    if weights.is_empty() {
        return (0.0, 0.0);
    }
    let max = weights.iter().copied().fold(0.0_f64, f64::max);
    if max <= 0.0 || !max.is_finite() {
        return (0.0, 0.0);
    }
    let sum: f64 = weights.iter().map(|w| w / max).sum();
    let squares: f64 = weights.iter().map(|w| (w / max).powi(2)).sum();
    (
        if squares > 0.0 {
            sum * sum / squares
        } else {
            0.0
        },
        max,
    )
}

fn number(value: f64) -> String {
    if value.abs() < 0.000_000_000_000_5 {
        "0.000000000000".to_string()
    } else {
        format!("{value:.12}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEADER: &str = "observation_key\tassigned_arm\toutcome\tpolicy_candidate\tpolicy_comparator\tprobability:A\tprobability:B\tprobability:C\tprediction:A\tprediction:B\tprediction:C";

    fn hand_input() -> String {
        format!(
            "{HEADER}\nobs-1\tA\t12\tA\tC\t0.5\t0.25\t0.25\t10\t8\t6\nobs-2\tB\t7\tA\tB\t0.25\t0.5\t0.25\t10\t8\t6\nobs-3\tC\t9\tC\tC\t0.2\t0.3\t0.5\t10\t8\t6\n"
        )
    }

    fn generated_tsv(k: usize) -> String {
        let arms: Vec<String> = (0..k).map(|i| format!("arm{i}")).collect();
        let mut header: Vec<String> = PREFIX.iter().map(|v| v.to_string()).collect();
        header.extend(arms.iter().map(|a| format!("probability:{a}")));
        header.extend(arms.iter().map(|a| format!("prediction:{a}")));
        let mut lines = vec![header.join("\t")];
        for i in 0..k {
            let p = format!("{:.12}", 1.0 / k as f64);
            let mut row = vec![
                format!("obs-{i}"),
                arms[i].clone(),
                (i as f64 + 2.0).to_string(),
                arms[i].clone(),
                arms[i].clone(),
            ];
            row.extend((0..k).map(|_| p.clone()));
            row.extend((0..k).map(|j| (j as f64 + 1.0).to_string()));
            lines.push(row.join("\t"));
        }
        lines.join("\n") + "\n"
    }

    #[test]
    fn hand_computable_three_arm_aipw_values_and_paired_difference() {
        let report = evaluate(&parse_tsv(&hand_input()).unwrap()).unwrap();
        assert_eq!(report.n, 3);
        assert_eq!(report.arms, vec!["A", "B", "C"]);
        assert!((report.candidate.value - 12.0).abs() < 1e-12);
        assert!((report.comparator.value - 8.0).abs() < 1e-12);
        assert!((report.candidate_minus_comparator.value_difference - 4.0).abs() < 1e-12);
        assert_eq!(report.candidate.matched_count, 2);
        assert_eq!(report.comparator.matched_count, 2);
    }

    #[test]
    fn per_arm_support_counts_are_suppressed_when_any_cell_is_small() {
        let report = evaluate(&parse_tsv(&hand_input()).unwrap()).unwrap();
        let output = render_json(&report);
        assert!(output.contains("\"arm_support_suppressed\": true"));
        assert!(output.contains("\"arm_support\": null"));
        assert!(!output.contains("randomized_assignments"));
    }

    #[test]
    fn generic_parser_and_evaluator_support_two_and_four_arms() {
        for k in [2, 4] {
            let report = evaluate(&parse_tsv(&generated_tsv(k)).unwrap()).unwrap();
            assert_eq!(report.arms.len(), k);
            assert_eq!(report.n, k);
            assert!((report.candidate.value - report.comparator.value).abs() < 1e-12);
        }
    }

    #[test]
    fn eight_arm_protocol_is_supported() {
        let report = evaluate(&parse_tsv(&generated_tsv(8)).unwrap()).unwrap();
        assert_eq!(report.arms.len(), 8);
        assert_eq!(report.n, 8);
    }

    #[test]
    fn nine_arm_protocol_is_rejected() {
        assert!(parse_tsv(&generated_tsv(9)).is_err());
    }

    #[test]
    fn invalid_registry_unknown_arm_and_bad_probabilities_are_rejected() {
        assert!(parse_tsv(&hand_input().replace("probability:C", "probability:B")).is_err());
        assert!(parse_tsv(&hand_input().replace("\tA\tC\t0.5", "\tA\tD\t0.5")).is_err());
        assert!(
            parse_tsv(&hand_input().replace("\t0.5\t0.25\t0.25\t10", "\t0\t0.25\t0.75\t10"))
                .is_err()
        );
        assert!(
            parse_tsv(&hand_input().replace("\t0.5\t0.25\t0.25\t10", "\t0.5\t0.5\t0.25\t10"))
                .is_err()
        );
    }

    #[test]
    fn policy_arm_without_observed_support_is_rejected() {
        let input = format!(
            "{HEADER}\nobs-1\tA\t10\tC\tA\t0.5\t0.25\t0.25\t8\t7\t6\nobs-2\tB\t8\tC\tB\t0.25\t0.5\t0.25\t8\t7\t6\n"
        );
        assert!(evaluate(&parse_tsv(&input).unwrap()).is_err());
    }

    #[test]
    fn output_is_aggregate_only_and_row_order_invariant() {
        let dataset = parse_tsv(&hand_input()).unwrap();
        let expected = render_json(&evaluate(&dataset).unwrap());
        assert!(!expected.contains("obs-1"));
        let mut reversed = dataset;
        reversed.observations.reverse();
        assert_eq!(expected, render_json(&evaluate(&reversed).unwrap()));
    }

    #[test]
    fn unequal_allocation_simulation_recovers_policy_values_with_biased_predictions() {
        let n = 5_000usize;
        let mut state = 0x4d59_5df4_d0f3_3173_u64;
        let mut observations = Vec::with_capacity(n);
        let mut truth_candidate = 0.0;
        let mut truth_comparator = 0.0;
        let arms = vec![
            "escitalopram".into(),
            "sertraline".into(),
            "venlafaxine".into(),
        ];

        for i in 0..n {
            let x = ((i * 37 % 997) as f64 / 996.0) * 2.0 - 1.0;
            let phase = i as f64 * 0.017;
            let y = vec![
                1.2 + 0.7 * x + 0.25 * phase.sin(),
                0.7 - 0.35 * x + 0.20 * (phase * 0.73).cos(),
                0.45 + 0.15 * x - 0.10 * (phase * 0.51).sin(),
            ];
            let candidate = if x < -0.3 {
                0
            } else if x < 0.3 {
                1
            } else {
                2
            };
            let comparator = 1;
            truth_candidate += y[candidate];
            truth_comparator += y[comparator];

            let pa = 0.2 + 0.1 * (i % 3) as f64;
            let probs = vec![pa, 0.35, 1.0 - pa - 0.35];
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let draw = (state >> 11) as f64 / ((1_u64 << 53) as f64);
            let assigned = if draw < probs[0] {
                0
            } else if draw < probs[0] + probs[1] {
                1
            } else {
                2
            };
            let predictions = vec![y[0] + 0.25, y[1] - 0.15, y[2] + 0.20];
            observations.push(Observation {
                observation_key: format!("sim-{i:05}"),
                assigned_arm: assigned,
                probabilities: probs,
                outcome: y[assigned],
                candidate_arm: candidate,
                comparator_arm: comparator,
                predictions,
            });
        }

        truth_candidate /= n as f64;
        truth_comparator /= n as f64;
        let report = evaluate(&Dataset { arms, observations }).unwrap();
        assert!((report.candidate.value - truth_candidate).abs() < 0.06);
        assert!((report.comparator.value - truth_comparator).abs() < 0.06);
        assert!(
            (report.candidate_minus_comparator.value_difference
                - (truth_candidate - truth_comparator))
                .abs()
                < 0.06
        );
    }
}
