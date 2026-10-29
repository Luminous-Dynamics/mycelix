//! Evaluate an already-frozen held-out prediction CSV without fitting a model.
//!
//! Required CSV headers:
//! group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper
//!
//! Group IDs must be independently reviewed source-study/site/batch clusters. This
//! tool cannot validate the scientific truth of those identifiers or the model.

use csv::ReaderBuilder;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const BOOTSTRAP_REPLICATES: usize = 2_000;
const DEFAULT_SEED: u64 = 0x5450_5245_4441_5544; // "TPREDAUD"

#[derive(Debug, Deserialize)]
struct PredictionRecord {
    group_id: String,
    actual: f64,
    candidate_prediction: f64,
    baseline_prediction: f64,
    interval_lower: f64,
    interval_upper: f64,
}

#[derive(Debug, Serialize, Clone)]
struct Metrics {
    observations: usize,
    mae: f64,
    rmse: f64,
    bias_prediction_minus_actual: f64,
    empirical_interval_coverage: Option<f64>,
    mean_interval_width: Option<f64>,
}

#[derive(Debug, Serialize)]
struct BootstrapInterval {
    confidence_level: f64,
    replicates_requested: usize,
    replicates_valid: usize,
    relative_mae_improvement_lower: f64,
    relative_mae_improvement_upper: f64,
    excludes_zero: bool,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    input_label: String,
    input_sha256: String,
    observations: usize,
    independent_groups: usize,
    bootstrap_seed: u64,
    candidate: Metrics,
    baseline: Metrics,
    relative_mae_improvement: Option<f64>,
    improvement_cluster_bootstrap_95pct: Option<BootstrapInterval>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to a String is infallible");
    }
    output
}

fn parse_records(bytes: &[u8]) -> Result<Vec<PredictionRecord>, Box<dyn Error>> {
    if bytes.is_empty() {
        return Err("input file is empty".into());
    }
    let mut reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::All)
        .from_reader(bytes);
    let headers = reader.headers()?.clone();
    for required in [
        "group_id",
        "actual",
        "candidate_prediction",
        "baseline_prediction",
        "interval_lower",
        "interval_upper",
    ] {
        if !headers.iter().any(|header| header == required) {
            return Err(format!("required CSV column is missing: {required}").into());
        }
    }

    let mut records = Vec::new();
    for (row_index, row) in reader.deserialize::<PredictionRecord>().enumerate() {
        let record = row.map_err(|error| {
            format!("invalid CSV record at data row {}: {error}", row_index + 1)
        })?;
        if record.group_id.trim().is_empty() {
            return Err(format!("empty group_id at data row {}", row_index + 1).into());
        }
        if record.group_id.trim() != record.group_id {
            return Err(format!(
                "group_id has leading/trailing whitespace at data row {}; refusing ambiguous group aliases",
                row_index + 1
            )
            .into());
        }
        if ["na", "n/a", "none", "null", "unknown", "missing", "-"]
            .contains(&record.group_id.to_ascii_lowercase().as_str())
        {
            return Err(format!(
                "group_id is a placeholder at data row {}; a reviewed independent-group identifier is required",
                row_index + 1
            )
            .into());
        }
        for (name, value) in [
            ("actual", record.actual),
            ("candidate_prediction", record.candidate_prediction),
            ("baseline_prediction", record.baseline_prediction),
            ("interval_lower", record.interval_lower),
            ("interval_upper", record.interval_upper),
        ] {
            if !value.is_finite() {
                return Err(format!("{name} is non-finite at data row {}", row_index + 1).into());
            }
        }
        if record.interval_lower > record.interval_upper {
            return Err(format!(
                "interval_lower exceeds interval_upper at data row {}",
                row_index + 1
            )
            .into());
        }
        records.push(record);
    }
    if records.is_empty() {
        return Err("CSV contains a header but no prediction rows".into());
    }
    Ok(records)
}

fn metrics(records: &[PredictionRecord], indices: &[usize], use_candidate: bool) -> Metrics {
    let count = indices.len() as f64;
    let mut absolute_error_sum = 0.0;
    let mut squared_error_sum = 0.0;
    let mut signed_error_sum = 0.0;
    let mut covered = 0_usize;
    let mut interval_width_sum = 0.0;

    for &index in indices {
        let row = &records[index];
        let prediction = if use_candidate {
            row.candidate_prediction
        } else {
            row.baseline_prediction
        };
        let error = prediction - row.actual;
        absolute_error_sum += error.abs();
        squared_error_sum += error * error;
        signed_error_sum += error;
        if use_candidate && row.actual >= row.interval_lower && row.actual <= row.interval_upper {
            covered += 1;
        }
        if use_candidate {
            interval_width_sum += row.interval_upper - row.interval_lower;
        }
    }

    Metrics {
        observations: indices.len(),
        mae: absolute_error_sum / count,
        rmse: (squared_error_sum / count).sqrt(),
        bias_prediction_minus_actual: signed_error_sum / count,
        empirical_interval_coverage: if use_candidate {
            Some(covered as f64 / count)
        } else {
            None
        },
        mean_interval_width: if use_candidate {
            Some(interval_width_sum / count)
        } else {
            None
        },
    }
}

fn group_indices(records: &[PredictionRecord]) -> Vec<Vec<usize>> {
    let mut groups = BTreeMap::<String, Vec<usize>>::new();
    for (index, record) in records.iter().enumerate() {
        groups
            .entry(record.group_id.clone())
            .or_default()
            .push(index);
    }
    groups.into_values().collect()
}

#[derive(Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { DEFAULT_SEED } else { seed },
        }
    }

    fn next(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn index(&mut self, upper_exclusive: usize) -> usize {
        (self.next() % upper_exclusive as u64) as usize
    }
}

fn percentile(sorted_values: &[f64], probability: f64) -> f64 {
    let rank = (probability * sorted_values.len() as f64).ceil() as usize;
    sorted_values[rank.saturating_sub(1).min(sorted_values.len() - 1)]
}

fn cluster_bootstrap(
    records: &[PredictionRecord],
    groups: &[Vec<usize>],
    seed: u64,
) -> Result<BootstrapInterval, Box<dyn Error>> {
    if groups.len() < 2 {
        return Err("cluster-bootstrap requires at least two independent groups".into());
    }
    let mut rng = XorShift64::new(seed);
    let mut improvements = Vec::with_capacity(BOOTSTRAP_REPLICATES);

    for _ in 0..BOOTSTRAP_REPLICATES {
        let mut sampled_indices = Vec::new();
        for _ in 0..groups.len() {
            let chosen_group = &groups[rng.index(groups.len())];
            sampled_indices.extend_from_slice(chosen_group);
        }
        let candidate_mae = metrics(records, &sampled_indices, true).mae;
        let baseline_mae = metrics(records, &sampled_indices, false).mae;
        if baseline_mae > 0.0 {
            improvements.push((baseline_mae - candidate_mae) / baseline_mae);
        }
    }

    if improvements.len() < BOOTSTRAP_REPLICATES * 9 / 10 {
        return Err(
            "too many bootstrap replicates have zero baseline MAE; relative improvement is unstable"
                .into(),
        );
    }
    improvements.sort_by(f64::total_cmp);
    let lower = percentile(&improvements, 0.025);
    let upper = percentile(&improvements, 0.975);
    Ok(BootstrapInterval {
        confidence_level: 0.95,
        replicates_requested: BOOTSTRAP_REPLICATES,
        replicates_valid: improvements.len(),
        relative_mae_improvement_lower: lower,
        relative_mae_improvement_upper: upper,
        excludes_zero: lower > 0.0 || upper < 0.0,
    })
}

fn ensure_distinct_input_output(
    input_path: &Path,
    output_path: &Path,
) -> Result<(), Box<dyn Error>> {
    if output_path.exists() && fs::canonicalize(input_path)? == fs::canonicalize(output_path)? {
        return Err(
            "input and report paths resolve to the same file; refusing to overwrite source".into(),
        );
    }
    Ok(())
}

fn audit(bytes: &[u8], input_label: &str, seed: u64) -> Result<AuditReport, Box<dyn Error>> {
    let records = parse_records(bytes)?;
    let groups = group_indices(&records);
    if groups.len() < 2 {
        return Err(
            "at least two independently identified groups are required; rows are not independent units"
                .into(),
        );
    }

    let all_indices: Vec<usize> = (0..records.len()).collect();
    let candidate = metrics(&records, &all_indices, true);
    let baseline = metrics(&records, &all_indices, false);
    let relative_improvement = if baseline.mae > 0.0 {
        Some((baseline.mae - candidate.mae) / baseline.mae)
    } else {
        None
    };

    let mut warnings = vec![
        "This evaluator scores supplied predictions; it does not train or validate a model's scientific assumptions.".to_owned(),
        "Group IDs must be reviewed source-study/site/batch clusters; software cannot prove independence from labels alone.".to_owned(),
        "Empirical interval coverage is descriptive and does not establish calibration outside this held-out sample.".to_owned(),
        "A lower MAE or nonzero bootstrap interval is not proof of causality, field efficacy, safety, or regulatory approval.".to_owned(),
    ];
    if groups.len() < 10 {
        warnings.push(format!(
            "Only {} independent groups are present; treat the cluster-bootstrap interval as exploratory and do not use it as confirmatory evidence.",
            groups.len()
        ));
    }
    let bootstrap = if baseline.mae > 0.0 {
        Some(cluster_bootstrap(&records, &groups, seed)?)
    } else {
        warnings.push(
            "Baseline MAE is zero; relative MAE improvement and its confidence interval are undefined."
                .to_owned(),
        );
        None
    };

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        input_label: input_label.to_owned(),
        input_sha256: sha256_hex(bytes),
        observations: records.len(),
        independent_groups: groups.len(),
        bootstrap_seed: seed,
        candidate,
        baseline,
        relative_mae_improvement: relative_improvement,
        improvement_cluster_bootstrap_95pct: bootstrap,
        warnings,
        interpretation_boundary: "Exploratory scoring of supplied held-out predictions; not efficacy evidence or a release authorization.",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() < 3 || arguments.len() > 4 {
        return Err(format!(
            "usage: {} <held-out-predictions.csv> <report.json> [bootstrap-seed]",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-prediction-audit")
        )
        .into());
    }
    let seed = if let Some(value) = arguments.get(3) {
        value
            .parse::<u64>()
            .map_err(|_| "bootstrap seed must be an unsigned integer")?
    } else {
        DEFAULT_SEED
    };
    let input_path = Path::new(&arguments[1]);
    let output_path = Path::new(&arguments[2]);
    ensure_distinct_input_output(input_path, output_path)?;
    let bytes = fs::read(input_path)?;
    let input_label = input_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("predictions.csv")
        .to_owned();
    let report = audit(&bytes, &input_label, seed)?;
    let output = serde_json::to_vec_pretty(&report)?;
    fs::write(output_path, output)?;
    println!(
        "audited {} observations across {} groups; report written to {}",
        report.observations, report.independent_groups, arguments[2]
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("prediction audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CSV: &str =
        "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper
study-a,10,10,8,8,12
study-a,12,11,9,9,13
study-b,20,19,15,16,22
study-b,22,21,18,18,24
study-c,30,29,24,25,33
study-c,32,31,26,27,35
";

    #[test]
    fn computes_metrics_and_group_counts() {
        let report = audit(VALID_CSV.as_bytes(), "fixture.csv", DEFAULT_SEED).unwrap();
        assert_eq!(report.observations, 6);
        assert_eq!(report.independent_groups, 3);
        assert!(report.candidate.mae < report.baseline.mae);
        assert_eq!(report.candidate.empirical_interval_coverage, Some(1.0));
        assert_eq!(report.baseline.empirical_interval_coverage, None);
        assert!(report.improvement_cluster_bootstrap_95pct.is_some());
        assert!(
            report
                .warnings
                .iter()
                .any(|warning| warning.contains("Only 3 independent groups"))
        );
    }

    #[test]
    fn bootstrap_is_reproducible() {
        let records = parse_records(VALID_CSV.as_bytes()).unwrap();
        let groups = group_indices(&records);
        let a = cluster_bootstrap(&records, &groups, 12345).unwrap();
        let b = cluster_bootstrap(&records, &groups, 12345).unwrap();
        assert_eq!(
            a.relative_mae_improvement_lower,
            b.relative_mae_improvement_lower
        );
        assert_eq!(
            a.relative_mae_improvement_upper,
            b.relative_mae_improvement_upper
        );
    }

    #[test]
    fn refuses_to_overwrite_input_with_report() {
        let path = std::env::temp_dir().join(format!(
            "terra-preta-prediction-audit-alias-{}.csv",
            std::process::id()
        ));
        fs::write(&path, "fixture").unwrap();
        let result = ensure_distinct_input_output(&path, &path);
        fs::remove_file(&path).unwrap();
        assert!(result.is_err());
    }

    #[test]
    fn rejects_missing_required_column() {
        let csv =
            "group_id,actual,candidate_prediction,baseline_prediction,interval_lower\na,1,1,1,0\n";
        assert!(parse_records(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_empty_group_id() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\n,1,1,1,0,2\nb,2,2,1,1,3\n";
        assert!(audit(csv.as_bytes(), "bad.csv", 1).is_err());
    }

    #[test]
    fn rejects_group_id_whitespace_alias() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\nstudy-a,1,1,1,0,2\n study-a,2,2,1,1,3\nstudy-c,3,3,2,2,4\n";
        assert!(parse_records(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_placeholder_group_id() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\nunknown,1,1,1,0,2\nstudy-b,2,2,1,1,3\nstudy-c,3,3,2,2,4\n";
        assert!(parse_records(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_reversed_prediction_interval() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\na,1,1,1,2,0\nb,2,2,1,1,3\n";
        assert!(parse_records(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_non_finite_prediction() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\na,1,NaN,1,0,2\nb,2,2,1,1,3\n";
        assert!(parse_records(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_single_group_as_pseudoreplication_risk() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\na,1,1,1,0,2\na,2,2,1,1,3\n";
        assert!(audit(csv.as_bytes(), "one-group.csv", 1).is_err());
    }

    #[test]
    fn flags_zero_baseline_mae_as_undefined_relative_improvement() {
        let csv = "group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper\na,1,1,1,0,2\nb,2,2,2,1,3\n";
        let report = audit(csv.as_bytes(), "zero-baseline.csv", 1).unwrap();
        assert_eq!(report.relative_mae_improvement, None);
        assert!(report.improvement_cluster_bootstrap_95pct.is_none());
    }
}
