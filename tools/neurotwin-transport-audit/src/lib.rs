//! Aggregate-only evaluation of locked binary predictions across domains.

use std::collections::{BTreeMap, HashSet};

pub const DEFAULT_MIN_REPORT_N: usize = 20;
const LOG_EPSILON: f64 = 1.0e-15;
const CALIBRATION_TOLERANCE: f64 = 1.0e-8;
const TARGET_COVERAGES: [f64; 6] = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5];
const EXPECTED_HEADER: &str =
    "observation_key\tdomain\toutcome\tpredicted_probability\tsupport_score";

#[derive(Clone, Debug, PartialEq)]
pub struct Observation {
    /// Local-only key for duplicate detection; never serialized.
    pub observation_key: String,
    /// Local-only domain key; never serialized.
    pub domain: String,
    pub outcome: bool,
    pub predicted_probability: f64,
    /// Frozen model-support score; larger values are retained first.
    pub support_score: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Metrics {
    pub auroc: Option<f64>,
    pub auprc: Option<f64>,
    pub brier_score: f64,
    pub log_loss: f64,
    pub calibration_intercept: Option<f64>,
    pub calibration_slope: Option<f64>,
    pub observed_to_expected_ratio: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SelectivePoint {
    pub requested_coverage: f64,
    pub actual_coverage: Option<f64>,
    pub brier_score: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DomainReport {
    pub domain_index: usize,
    pub suppressed: bool,
    pub n: Option<usize>,
    pub metrics: Option<Metrics>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransportReport {
    pub n: usize,
    pub metrics: Metrics,
    pub selective_curve: Vec<SelectivePoint>,
    pub domains: Vec<DomainReport>,
}

/// Parse a strict TSV contract. Original keys and domain labels are local-only.
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
        if fields.len() != 5 {
            return Err(format!("wrong column count on line {line_number}"));
        }
        if !valid_key(fields[0], 128) || !valid_key(fields[1], 64) {
            return Err(format!("invalid local key or domain on line {line_number}"));
        }
        if !keys.insert(fields[0].to_string()) {
            return Err(format!(
                "duplicate local observation key on line {line_number}"
            ));
        }
        let outcome = match fields[2] {
            "0" => false,
            "1" => true,
            _ => return Err(format!("outcome must be 0 or 1 on line {line_number}")),
        };
        let row = Observation {
            observation_key: fields[0].to_string(),
            domain: fields[1].to_string(),
            outcome,
            predicted_probability: parse_finite(fields[3], "predicted_probability", line_number)?,
            support_score: parse_finite(fields[4], "support_score", line_number)?,
        };
        validate_observation(&row, line_number)?;
        observations.push(row);
    }
    if observations.is_empty() {
        return Err("input contains no observations".to_string());
    }
    Ok(observations)
}

/// Evaluate overall and domain-wise performance. Domain labels are replaced by
/// local ordinal indices and never included in the result.
pub fn evaluate(observations: &[Observation]) -> Result<TransportReport, String> {
    if observations.len() < DEFAULT_MIN_REPORT_N {
        return Err("overall cohort is below the built-in reporting minimum".to_string());
    }

    let mut rows = observations.to_vec();
    rows.sort_by(|left, right| {
        left.domain
            .cmp(&right.domain)
            .then_with(|| left.observation_key.cmp(&right.observation_key))
    });

    let mut keys = HashSet::new();
    for (index, row) in rows.iter().enumerate() {
        if !valid_key(&row.observation_key, 128) || !valid_key(&row.domain, 64) {
            return Err(format!("invalid local key or domain at row {}", index + 1));
        }
        if !keys.insert(row.observation_key.clone()) {
            return Err(format!(
                "duplicate local observation key at row {}",
                index + 1
            ));
        }
        validate_observation(row, index + 1)?;
    }

    let metrics = calculate_metrics(&rows)?;
    let selective_curve = calculate_selective_curve(&rows);
    let mut groups: BTreeMap<String, Vec<Observation>> = BTreeMap::new();
    for row in &rows {
        groups
            .entry(row.domain.clone())
            .or_default()
            .push(row.clone());
    }

    let mut domains = Vec::with_capacity(groups.len());
    for (index, (_domain_label, domain_rows)) in groups.into_iter().enumerate() {
        if domain_rows.len() < DEFAULT_MIN_REPORT_N {
            domains.push(DomainReport {
                domain_index: index + 1,
                suppressed: true,
                n: None,
                metrics: None,
            });
        } else {
            domains.push(DomainReport {
                domain_index: index + 1,
                suppressed: false,
                n: Some(domain_rows.len()),
                metrics: Some(calculate_metrics(&domain_rows)?),
            });
        }
    }

    Ok(TransportReport {
        n: rows.len(),
        metrics,
        selective_curve,
        domains,
    })
}

/// Render aggregate-only JSON. No row keys, original domain labels or predictions.
pub fn render_json(report: &TransportReport) -> String {
    let mut output = String::new();
    output.push_str("{\n");
    output.push_str("  \"schema_version\": \"neurotwin-transport-report-v0.1\",\n");
    output.push_str(&format!("  \"n\": {},\n", report.n));
    output.push_str(&format!(
        "  \"minimum_report_n\": {},\n",
        DEFAULT_MIN_REPORT_N
    ));
    output.push_str("  \"auprc_method\": \"non_interpolated_average_precision_tie_grouped\",\n");
    output.push_str("  \"overall\": ");
    render_metrics(&mut output, &report.metrics, 2);
    output.push_str(",\n  \"selective_curve\": [");
    for (index, point) in report.selective_curve.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push_str("\n    {");
        output.push_str(&format!(
            "\"requested_coverage\": {}, \"actual_coverage\": {}, \"brier_score\": {}",
            json_number(point.requested_coverage),
            option_number(point.actual_coverage),
            option_number(point.brier_score)
        ));
        output.push('}');
    }
    if !report.selective_curve.is_empty() {
        output.push('\n');
        output.push_str("  ");
    }
    output.push_str("],\n  \"domains\": [");
    for (index, domain) in report.domains.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push_str("\n    {");
        output.push_str(&format!(
            "\"domain_index\": {}, \"status\": \"{}\", \"n\": {}, \"metrics\": ",
            domain.domain_index,
            if domain.suppressed {
                "suppressed"
            } else {
                "reported"
            },
            domain
                .n
                .map(|value| value.to_string())
                .unwrap_or_else(|| "null".to_string())
        ));
        if let Some(metrics) = &domain.metrics {
            render_metrics(&mut output, metrics, 6);
        } else {
            output.push_str("null");
        }
        output.push('}');
    }
    if !report.domains.is_empty() {
        output.push('\n');
        output.push_str("  ");
    }
    output.push_str("],\n  \"limitations\": [");
    output.push_str("\n    \"Predictions and support scores must be locked before evaluation; this tool cannot verify provenance.\",");
    output.push_str("\n    \"Calibration is descriptive; no calibration fitting or threshold selection is performed.\",");
    output.push_str("\n    \"Support scores are not clinical certainty; abstention is not safe until independently validated.\",");
    output.push_str("\n    \"AUROC/AUPRC are undefined for single-class cohorts or domains.\",");
    output.push_str("\n    \"Domain indices are local ordinal labels and do not establish independent external validation.\",");
    output.push_str(
        "\n    \"This report is not a treatment recommendation or proof of clinical utility.\"",
    );
    output.push_str("\n  ]\n}\n");
    output
}

fn calculate_metrics(rows: &[Observation]) -> Result<Metrics, String> {
    if rows.is_empty() {
        return Err("cannot calculate metrics for an empty cohort".to_string());
    }
    let positives = rows.iter().filter(|row| row.outcome).count();
    let negatives = rows.len() - positives;
    let mut brier_sum = 0.0;
    let mut log_loss_sum = 0.0;
    let mut expected = 0.0;
    for row in rows {
        let y = if row.outcome { 1.0 } else { 0.0 };
        let p = row.predicted_probability;
        let clipped = p.clamp(LOG_EPSILON, 1.0 - LOG_EPSILON);
        let error = p - y;
        brier_sum += error * error;
        log_loss_sum += -(y * clipped.ln() + (1.0 - y) * (1.0 - clipped).ln());
        expected += p;
    }
    let n = rows.len() as f64;
    let brier_score = brier_sum / n;
    let log_loss = log_loss_sum / n;
    if !brier_score.is_finite() || !log_loss.is_finite() {
        return Err("non-finite aggregate metric".to_string());
    }
    let (calibration_intercept, calibration_slope) = fit_calibration(rows);
    let observed_to_expected_ratio = if expected > 0.0 && expected.is_finite() {
        Some(positives as f64 / expected)
    } else {
        None
    };
    Ok(Metrics {
        auroc: calculate_auroc(rows, positives, negatives),
        auprc: calculate_auprc(rows, positives),
        brier_score,
        log_loss,
        calibration_intercept,
        calibration_slope,
        observed_to_expected_ratio,
    })
}

fn calculate_auroc(rows: &[Observation], positives: usize, negatives: usize) -> Option<f64> {
    if positives == 0 || negatives == 0 {
        return None;
    }
    let mut sorted: Vec<&Observation> = rows.iter().collect();
    sorted.sort_by(|left, right| {
        left.predicted_probability
            .total_cmp(&right.predicted_probability)
    });
    let mut positive_rank_sum = 0.0;
    let mut start = 0usize;
    while start < sorted.len() {
        let mut end = start + 1;
        while end < sorted.len()
            && sorted[end].predicted_probability == sorted[start].predicted_probability
        {
            end += 1;
        }
        let average_rank = (start + 1 + end) as f64 / 2.0;
        let tied_positives = sorted[start..end].iter().filter(|row| row.outcome).count();
        positive_rank_sum += average_rank * tied_positives as f64;
        start = end;
    }
    let positive = positives as f64;
    let negative = negatives as f64;
    let auc = (positive_rank_sum - positive * (positive + 1.0) / 2.0) / (positive * negative);
    auc.is_finite().then_some(auc)
}

fn calculate_auprc(rows: &[Observation], positives: usize) -> Option<f64> {
    if positives == 0 {
        return None;
    }
    let mut sorted: Vec<&Observation> = rows.iter().collect();
    sorted.sort_by(|left, right| {
        right
            .predicted_probability
            .total_cmp(&left.predicted_probability)
    });
    let mut true_positives = 0usize;
    let mut false_positives = 0usize;
    let mut previous_recall = 0.0;
    let mut area = 0.0;
    let mut index = 0usize;
    while index < sorted.len() {
        let score = sorted[index].predicted_probability;
        let mut end = index;
        while end < sorted.len() && sorted[end].predicted_probability == score {
            if sorted[end].outcome {
                true_positives += 1;
            } else {
                false_positives += 1;
            }
            end += 1;
        }
        let recall = true_positives as f64 / positives as f64;
        let precision = true_positives as f64 / (true_positives + false_positives) as f64;
        area += (recall - previous_recall) * precision;
        previous_recall = recall;
        index = end;
    }
    area.is_finite().then_some(area)
}

/// Logistic calibration model: logit(Y) = intercept + slope * logit(prediction).
/// No recalibration is applied. Singular/non-convergent fits are returned as null.
fn fit_calibration(rows: &[Observation]) -> (Option<f64>, Option<f64>) {
    let positives = rows.iter().filter(|row| row.outcome).count();
    if positives == 0 || positives == rows.len() {
        return (None, None);
    }
    let prevalence = positives as f64 / rows.len() as f64;
    let mut intercept = (prevalence / (1.0 - prevalence)).ln();
    let mut slope = 1.0;

    for _ in 0..100 {
        let mut g0 = 0.0;
        let mut g1 = 0.0;
        let mut h00 = 0.0;
        let mut h01 = 0.0;
        let mut h11 = 0.0;
        for row in rows {
            let x = logit(row.predicted_probability);
            let fitted = sigmoid((intercept + slope * x).clamp(-35.0, 35.0));
            let y = if row.outcome { 1.0 } else { 0.0 };
            let residual = y - fitted;
            let weight = (fitted * (1.0 - fitted)).max(1.0e-12);
            g0 += residual;
            g1 += residual * x;
            h00 += weight;
            h01 += weight * x;
            h11 += weight * x * x;
        }
        let determinant = h00 * h11 - h01 * h01;
        if !determinant.is_finite() || determinant <= 1.0e-14 {
            return (None, None);
        }
        let delta0 = (h11 * g0 - h01 * g1) / determinant;
        let delta1 = (h00 * g1 - h01 * g0) / determinant;
        if !delta0.is_finite() || !delta1.is_finite() {
            return (None, None);
        }
        let scale = 1.0_f64
            .min(5.0 / delta0.abs().max(1.0))
            .min(5.0 / delta1.abs().max(1.0));
        intercept = (intercept + scale * delta0).clamp(-35.0, 35.0);
        slope = (slope + scale * delta1).clamp(-35.0, 35.0);
        if (scale * delta0).abs() < CALIBRATION_TOLERANCE
            && (scale * delta1).abs() < CALIBRATION_TOLERANCE
        {
            return (Some(intercept), Some(slope));
        }
    }
    (None, None)
}

fn calculate_selective_curve(rows: &[Observation]) -> Vec<SelectivePoint> {
    let mut sorted: Vec<&Observation> = rows.iter().collect();
    sorted.sort_by(|left, right| right.support_score.total_cmp(&left.support_score));
    let mut points = Vec::new();
    for requested_coverage in TARGET_COVERAGES {
        let target_count =
            ((requested_coverage * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len());
        let threshold = sorted[target_count - 1].support_score;
        // Retain all ties at the cut-off; no row-order/key tie-break is used.
        let retained: Vec<&Observation> = sorted
            .iter()
            .copied()
            .filter(|row| row.support_score >= threshold)
            .collect();
        if retained.len() < DEFAULT_MIN_REPORT_N {
            points.push(SelectivePoint {
                requested_coverage,
                actual_coverage: None,
                brier_score: None,
            });
        } else {
            let brier_sum: f64 = retained
                .iter()
                .map(|row| {
                    let error = row.predicted_probability - (if row.outcome { 1.0 } else { 0.0 });
                    error * error
                })
                .sum();
            points.push(SelectivePoint {
                requested_coverage,
                actual_coverage: Some(retained.len() as f64 / rows.len() as f64),
                brier_score: Some(brier_sum / retained.len() as f64),
            });
        }
    }
    points
}

fn render_metrics(output: &mut String, metrics: &Metrics, indent: usize) {
    let pad = " ".repeat(indent);
    output.push_str("{\n");
    output.push_str(&format!(
        "{pad}  \"auroc\": {},\n",
        option_number(metrics.auroc)
    ));
    output.push_str(&format!(
        "{pad}  \"auprc\": {},\n",
        option_number(metrics.auprc)
    ));
    output.push_str(&format!(
        "{pad}  \"brier_score\": {},\n",
        json_number(metrics.brier_score)
    ));
    output.push_str(&format!(
        "{pad}  \"log_loss\": {},\n",
        json_number(metrics.log_loss)
    ));
    output.push_str(&format!(
        "{pad}  \"calibration_intercept\": {},\n",
        option_number(metrics.calibration_intercept)
    ));
    output.push_str(&format!(
        "{pad}  \"calibration_slope\": {},\n",
        option_number(metrics.calibration_slope)
    ));
    output.push_str(&format!(
        "{pad}  \"observed_to_expected_ratio\": {}\n",
        option_number(metrics.observed_to_expected_ratio)
    ));
    output.push_str(&format!("{pad}}}"));
}

fn option_number(value: Option<f64>) -> String {
    value.map(json_number).unwrap_or_else(|| "null".to_string())
}

fn json_number(value: f64) -> String {
    if value.abs() < 0.000_000_000_000_5 {
        "0.000000000000".to_string()
    } else {
        format!("{value:.12}")
    }
}

fn validate_observation(row: &Observation, line: usize) -> Result<(), String> {
    if !row.predicted_probability.is_finite() || !(0.0..=1.0).contains(&row.predicted_probability) {
        return Err(format!("predicted probability outside [0,1] at row {line}"));
    }
    if !row.support_score.is_finite() || !(0.0..=1.0).contains(&row.support_score) {
        return Err(format!("support score outside [0,1] at row {line}"));
    }
    Ok(())
}

fn parse_finite(value: &str, field: &str, line: usize) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| format!("invalid {field} at line {line}"))?;
    if !parsed.is_finite() {
        return Err(format!("non-finite {field} at line {line}"));
    }
    Ok(parsed)
}

fn valid_key(value: &str, max_length: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_length
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

fn logit(probability: f64) -> f64 {
    let clipped = probability.clamp(LOG_EPSILON, 1.0 - LOG_EPSILON);
    (clipped / (1.0 - clipped)).ln()
}

fn sigmoid(value: f64) -> f64 {
    if value >= 0.0 {
        1.0 / (1.0 + (-value).exp())
    } else {
        let exponential = value.exp();
        exponential / (1.0 + exponential)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(key: &str, domain: &str, outcome: bool, p: f64, support: f64) -> Observation {
        Observation {
            observation_key: key.to_string(),
            domain: domain.to_string(),
            outcome,
            predicted_probability: p,
            support_score: support,
        }
    }

    fn perfect_rows() -> Vec<Observation> {
        (0..20)
            .map(|index| {
                let outcome = index >= 10;
                let p = if outcome {
                    0.70 + (index - 10) as f64 * 0.03
                } else {
                    0.10 + index as f64 * 0.03
                };
                row(
                    &format!("perfect-{index:02}"),
                    "trial-a",
                    outcome,
                    p,
                    1.0 - index as f64 * 0.04,
                )
            })
            .collect()
    }

    fn mixed_rows() -> Vec<Observation> {
        let values = [
            (false, 0.10),
            (false, 0.20),
            (false, 0.30),
            (false, 0.40),
            (false, 0.45),
            (false, 0.49),
            (false, 0.51),
            (false, 0.55),
            (false, 0.60),
            (false, 0.80),
            (true, 0.20),
            (true, 0.30),
            (true, 0.40),
            (true, 0.45),
            (true, 0.49),
            (true, 0.51),
            (true, 0.70),
            (true, 0.75),
            (true, 0.90),
            (true, 0.99),
        ];
        values
            .iter()
            .enumerate()
            .map(|(index, (outcome, p))| {
                row(
                    &format!("mixed-{index:02}"),
                    "trial-a",
                    *outcome,
                    *p,
                    1.0 - index as f64 * 0.04,
                )
            })
            .collect()
    }

    #[test]
    fn perfect_ranking_has_unit_auc_and_average_precision() {
        let metrics = calculate_metrics(&perfect_rows()).unwrap();
        assert!((metrics.auroc.unwrap() - 1.0).abs() < 1.0e-12);
        assert!((metrics.auprc.unwrap() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn tied_scores_receive_half_credit_for_auc() {
        let mut rows = perfect_rows();
        for row in &mut rows {
            row.predicted_probability = 0.5;
        }
        let metrics = calculate_metrics(&rows).unwrap();
        assert!((metrics.auroc.unwrap() - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn calibration_metrics_are_available_for_overlapping_predictions() {
        let metrics = calculate_metrics(&mixed_rows()).unwrap();
        assert!(metrics.calibration_intercept.unwrap().is_finite());
        assert!(metrics.calibration_slope.unwrap().is_finite());
        assert!(metrics.observed_to_expected_ratio.unwrap().is_finite());
    }

    #[test]
    fn calibration_fit_recovers_known_intercept_and_slope_under_synthetic_shift() {
        let mut rows = Vec::with_capacity(50_000);
        let mut state = 0x7a31_8d2f_6c45_b901_u64;
        for index in 0..50_000 {
            let p = 0.02 + 0.96 * index as f64 / 49_999.0;
            let true_probability = sigmoid(0.6 + 0.8 * logit(p));
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let draw = (state >> 11) as f64 / ((1_u64 << 53) as f64);
            rows.push(row(
                &format!("cal-{index:05}"),
                "calibration-sim",
                draw < true_probability,
                p,
                0.8,
            ));
        }

        let metrics = calculate_metrics(&rows).unwrap();
        let intercept = metrics.calibration_intercept.unwrap();
        let slope = metrics.calibration_slope.unwrap();
        assert!(
            (intercept - 0.6).abs() < 0.04,
            "intercept recovered as {intercept}"
        );
        assert!((slope - 0.8).abs() < 0.04, "slope recovered as {slope}");
    }

    #[test]
    fn prevalence_shift_preserves_auc_but_changes_precision_recall() {
        let negative_scores = [
            0.10, 0.15, 0.20, 0.25, 0.30, 0.12, 0.17, 0.22, 0.27, 0.32, 0.14, 0.19, 0.24, 0.29,
            0.34, 0.16, 0.21, 0.26, 0.31, 0.36,
        ];
        let positive_scores = [0.20, 0.30, 0.40, 0.50, 0.60];
        let mut common = Vec::new();
        let mut rare = Vec::new();

        for (index, score) in negative_scores.iter().enumerate() {
            common.push(row(
                &format!("common-neg-{index}"),
                "common",
                false,
                *score,
                0.8,
            ));
            rare.push(row(
                &format!("rare-neg-{index}"),
                "rare",
                false,
                *score,
                0.8,
            ));
        }
        for (index, score) in positive_scores.iter().enumerate() {
            for repeat in 0..4 {
                common.push(row(
                    &format!("common-pos-{index}-{repeat}"),
                    "common",
                    true,
                    *score,
                    0.8,
                ));
            }
            rare.push(row(&format!("rare-pos-{index}"), "rare", true, *score, 0.8));
        }

        let common_metrics = calculate_metrics(&common).unwrap();
        let rare_metrics = calculate_metrics(&rare).unwrap();
        assert!((common_metrics.auroc.unwrap() - rare_metrics.auroc.unwrap()).abs() < 1.0e-12);
        assert!(common_metrics.auprc.unwrap() > rare_metrics.auprc.unwrap());
    }

    #[test]
    fn single_class_domain_has_undefined_discrimination_and_calibration() {
        let rows: Vec<Observation> = (0..20)
            .map(|index| row(&format!("neg-{index}"), "all-negative", false, 0.2, 0.5))
            .collect();
        let metrics = calculate_metrics(&rows).unwrap();
        assert!(metrics.auroc.is_none());
        assert!(metrics.auprc.is_none());
        assert!(metrics.calibration_intercept.is_none());
        assert!(metrics.calibration_slope.is_none());
    }

    #[test]
    fn domain_labels_and_observation_keys_never_appear_in_report() {
        let report = evaluate(&perfect_rows()).unwrap();
        let output = render_json(&report);
        assert!(!output.contains("trial-a"));
        assert!(!output.contains("perfect-00"));
        assert!(output.contains("\"domain_index\""));
    }

    #[test]
    fn report_names_its_precision_recall_integration_convention() {
        let report = evaluate(&perfect_rows()).unwrap();
        let output = render_json(&report);
        assert!(output.contains("non_interpolated_average_precision_tie_grouped"));
    }

    #[test]
    fn small_domains_are_suppressed_without_their_denominators() {
        let mut rows = perfect_rows();
        rows.extend((0..3).map(|index| {
            row(
                &format!("small-{index}"),
                "sensitive-small-site",
                index == 0,
                0.4,
                0.5,
            )
        }));
        let report = evaluate(&rows).unwrap();
        let small = report
            .domains
            .iter()
            .find(|domain| domain.suppressed)
            .unwrap();
        assert!(small.n.is_none());
        assert!(small.metrics.is_none());
        assert!(!render_json(&report).contains("sensitive-small-site"));
    }

    #[test]
    fn selective_curve_suppresses_retained_sets_below_minimum() {
        let report = evaluate(&perfect_rows()).unwrap();
        assert_eq!(report.selective_curve[0].actual_coverage, Some(1.0));
        assert!(report.selective_curve[5].actual_coverage.is_none());
        assert!(report.selective_curve[5].brier_score.is_none());
    }

    #[test]
    fn support_ties_are_never_split_by_input_order() {
        let mut rows = perfect_rows();
        for row in rows.iter_mut().take(4) {
            row.support_score = 0.5;
        }
        let first = render_json(&evaluate(&rows).unwrap());
        rows.reverse();
        assert_eq!(first, render_json(&evaluate(&rows).unwrap()));
    }

    #[test]
    fn invalid_probability_and_support_are_rejected() {
        let mut rows = perfect_rows();
        rows[0].predicted_probability = 1.2;
        assert!(evaluate(&rows).is_err());
        rows[0].predicted_probability = 0.3;
        rows[0].support_score = f64::NAN;
        assert!(evaluate(&rows).is_err());
    }

    #[test]
    fn parser_rejects_duplicate_keys_and_extra_columns() {
        let first = "k01\ttrial-a\t0\t0.1\t0.9";
        let second = "k01\ttrial-a\t1\t0.9\t0.8";
        let duplicate = format!("{EXPECTED_HEADER}\n{first}\n{second}\n");
        assert!(parse_tsv(&duplicate).is_err());
        let extra = format!("{EXPECTED_HEADER}\nk01\ttrial-a\t0\t0.1\t0.9\textra\n");
        assert!(parse_tsv(&extra).is_err());
    }

    #[test]
    fn row_order_does_not_change_aggregate_report() {
        let mut rows = mixed_rows();
        let first = render_json(&evaluate(&rows).unwrap());
        rows.rotate_left(7);
        assert_eq!(first, render_json(&evaluate(&rows).unwrap()));
    }

    #[test]
    fn evaluator_rejects_small_overall_cohort() {
        assert!(evaluate(&perfect_rows()[..19]).is_err());
    }
}
