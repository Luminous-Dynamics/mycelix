use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;

const VERSION: &str = "0.3.0";
const DEFAULT_BOOTSTRAPS: usize = 1_000;
const DEFAULT_SEED: u64 = 9_901;
const MIN_PAIRS_FOR_REVIEW: usize = 10_000;
const MIN_SUBJECTS_FOR_REVIEW: usize = 2_000;

#[derive(Clone, Debug)]
struct TruthRow {
    pair_id: String,
    subject_a: String,
    subject_b: String,
    label: u8,
    stratum: String,
}

#[derive(Clone, Copy, Debug)]
struct WeightedObservation {
    score: f64,
    label: u8,
    weight: f64,
}

#[derive(Clone, Copy, Debug)]
struct Metrics {
    roc_auc: f64,
    average_precision: f64,
    tpr_at_fpr_1pct: f64,
    tpr_at_fpr_5pct: f64,
}

#[derive(Clone, Copy, Debug)]
struct Interval {
    low: f64,
    high: f64,
    valid_replicates: usize,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("life-privacy-eval: {err}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.get(1).is_some_and(|arg| arg == "compare") {
        return run_compare(&args);
    }
    if args.get(1).is_some_and(|arg| arg == "calibrated") {
        return run_calibrated(&args);
    }
    if args.len() < 5
        || args.len() > 7
        || args
            .get(1)
            .is_some_and(|arg| arg == "--help" || arg == "-h")
    {
        eprintln!(
            "Usage: {} <truth.tsv> <scores.tsv> <training-subjects.txt> <report.json> [bootstrap-replicates=1000] [seed=9901]\n       {} compare <truth.tsv> <baseline-scores.tsv> <treatment-scores.tsv> <training-subjects.txt> <report.json> [bootstrap-replicates=1000] [seed=9901] [max_auc_upper=0.55]\n\ntruth.tsv has pair_id, subject_a, subject_b, label, stratum.\nscores.tsv has only pair_id and score; never give ground-truth fields to the attack scorer. Paired comparison requires identical pair sets. The calibrated subcommand selects a threshold on validation labels and evaluates it on disjoint test subjects.",
            args.first().map_or("life-privacy-eval", String::as_str),
            args.first().map_or("life-privacy-eval", String::as_str)
        );
        if args
            .get(1)
            .is_some_and(|arg| arg == "--help" || arg == "-h")
        {
            return Ok(());
        }
        return Err("invalid argument count".into());
    }

    let truth_path = &args[1];
    let scores_path = &args[2];
    let train_path = &args[3];
    let report_path = &args[4];
    let bootstrap_replicates = args
        .get(5)
        .map_or(Ok(DEFAULT_BOOTSTRAPS), |s| s.parse::<usize>())?;
    let seed = args.get(6).map_or(Ok(DEFAULT_SEED), |s| s.parse::<u64>())?;
    if bootstrap_replicates < 100 {
        return Err("at least 100 subject-cluster bootstrap replicates are required for an interval".into());
    }

    let truth = read_truth(truth_path)?;
    let scores = read_scores(scores_path)?;
    let training_subjects = read_training_subjects(train_path)?;
    let rows = join_and_validate(truth, scores, &training_subjects)?;
    if !rows.iter().any(|row| row.label == 1) || !rows.iter().any(|row| row.label == 0) {
        return Err("truth set must contain at least one positive and one negative pair".into());
    }
    let report = build_report(&rows, &training_subjects, bootstrap_replicates, seed);
    let report = format!("{report}\n");
    if report_path == "-" {
        print!("{report}");
    } else {
        fs::write(report_path, report)?;
    }
    Ok(())
}


fn run_compare(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() < 7 || args.len() > 10 {
        return Err("compare usage: compare <truth.tsv> <baseline-scores.tsv> <treatment-scores.tsv> <training-subjects.txt> <report.json> [bootstrap-replicates=1000] [seed=9901] [max_auc_upper=0.55]".into());
    }
    let truth = read_truth(&args[2])?;
    let baseline_scores = read_scores(&args[3])?;
    let treatment_scores = read_scores(&args[4])?;
    let training_subjects = read_training_subjects(&args[5])?;
    let report_path = &args[6];
    let bootstrap_replicates = args.get(7).map_or(Ok(DEFAULT_BOOTSTRAPS), |s| s.parse::<usize>())?;
    let seed = args.get(8).map_or(Ok(DEFAULT_SEED), |s| s.parse::<u64>())?;
    let max_auc_upper = args.get(9).map_or(Ok(0.55_f64), |s| s.parse::<f64>())?;
    if bootstrap_replicates < 100 {
        return Err("at least 100 subject-cluster bootstrap replicates are required for an interval".into());
    }
    if !max_auc_upper.is_finite() || !(0.5..=1.0).contains(&max_auc_upper) {
        return Err("max_auc_upper must be finite and in the range [0.5, 1.0]".into());
    }
    let rows = join_paired_scores(truth, baseline_scores, treatment_scores, &training_subjects)?;
    if !rows.iter().any(|row| row.label == 1) || !rows.iter().any(|row| row.label == 0) {
        return Err("paired truth set must contain at least one positive and one negative pair".into());
    }
    let report = format!("{}\n", build_paired_report(&rows, &training_subjects, bootstrap_replicates, seed, max_auc_upper));
    if report_path == "-" {
        print!("{report}");
    } else {
        fs::write(report_path, report)?;
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct PairedScoreRow {
    pair_id: String,
    subject_a: String,
    subject_b: String,
    label: u8,
    stratum: String,
    baseline_score: f64,
    treatment_score: f64,
}

fn join_paired_scores(
    truth: Vec<TruthRow>,
    baseline_scores: HashMap<String, f64>,
    treatment_scores: HashMap<String, f64>,
    training_subjects: &HashSet<String>,
) -> Result<Vec<PairedScoreRow>, Box<dyn Error>> {
    let baseline = join_and_validate(truth.clone(), baseline_scores, training_subjects)?;
    let treatment = join_and_validate(truth, treatment_scores, training_subjects)?;
    if baseline.len() != treatment.len() {
        return Err("baseline and treatment must contain exactly the same pair set".into());
    }
    let mut output = Vec::with_capacity(baseline.len());
    for (left, right) in baseline.into_iter().zip(treatment) {
        if left.pair_id != right.pair_id
            || left.subject_a != right.subject_a
            || left.subject_b != right.subject_b
            || left.label != right.label
            || left.stratum != right.stratum
        {
            return Err("baseline and treatment rows are not aligned to identical pairs, subjects, labels and strata".into());
        }
        output.push(PairedScoreRow {
            pair_id: left.pair_id,
            subject_a: left.subject_a,
            subject_b: left.subject_b,
            label: left.label,
            stratum: left.stratum,
            baseline_score: left.score,
            treatment_score: right.score,
        });
    }
    Ok(output)
}


fn run_calibrated(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() < 8 || args.len() > 11 {
        return Err("calibrated usage: calibrated <validation-truth.tsv> <validation-scores.tsv> <test-truth.tsv> <test-scores.tsv> <training-subjects.txt> <report.json> [max_fpr=0.01] [bootstrap-replicates=1000] [seed=9901]".into());
    }
    let validation_truth = read_truth(&args[2])?;
    let validation_scores = read_scores(&args[3])?;
    let test_truth = read_truth(&args[4])?;
    let test_scores = read_scores(&args[5])?;
    let training_subjects = read_training_subjects(&args[6])?;
    let report_path = &args[7];
    let max_fpr = args.get(8).map_or(Ok(0.01_f64), |s| s.parse::<f64>())?;
    let bootstraps = args.get(9).map_or(Ok(DEFAULT_BOOTSTRAPS), |s| s.parse::<usize>())?;
    let seed = args.get(10).map_or(Ok(DEFAULT_SEED), |s| s.parse::<u64>())?;
    if !max_fpr.is_finite() || !(0.0..=1.0).contains(&max_fpr) {
        return Err("max_fpr must be finite and in [0, 1]".into());
    }
    if bootstraps < 100 {
        return Err("at least 100 subject-cluster bootstrap replicates are required for an interval".into());
    }
    let validation_rows = join_and_validate(validation_truth, validation_scores, &training_subjects)?;
    let test_rows = join_and_validate(test_truth, test_scores, &training_subjects)?;
    require_both_classes(&validation_rows, "validation")?;
    require_both_classes(&test_rows, "test")?;
    ensure_calibration_test_disjoint(&validation_rows, &test_rows)?;

    // Threshold selection reads validation labels only; test labels are used only after threshold lock.
    let selection = select_validation_threshold(&validation_rows, max_fpr);
    let rates = fixed_operating_rates(&test_rows, selection.threshold);
    let (tpr_interval, fpr_interval) =
        fixed_threshold_cluster_bootstrap(&test_rows, selection.threshold, bootstraps, seed);
    let report = format!(
        "{}\\n",
        build_calibrated_report(
            &validation_rows, &test_rows, &training_subjects, &selection,
            &rates, tpr_interval, fpr_interval, max_fpr, bootstraps, seed
        )
    );
    if report_path == "-" {
        print!("{report}");
    } else {
        fs::write(report_path, report)?;
    }
    Ok(())
}

fn require_both_classes(rows: &[TruthRowWithScore], partition: &str) -> Result<(), Box<dyn Error>> {
    if !rows.iter().any(|row| row.label == 1) || !rows.iter().any(|row| row.label == 0) {
        return Err(format!("{partition} partition must contain at least one positive and one negative pair").into());
    }
    Ok(())
}

fn ensure_calibration_test_disjoint(
    validation_rows: &[TruthRowWithScore],
    test_rows: &[TruthRowWithScore],
) -> Result<(), Box<dyn Error>> {
    let validation_subjects: BTreeSet<&str> = validation_rows.iter()
        .flat_map(|row| [row.subject_a.as_str(), row.subject_b.as_str()]).collect();
    let test_subjects: BTreeSet<&str> = test_rows.iter()
        .flat_map(|row| [row.subject_a.as_str(), row.subject_b.as_str()]).collect();
    if let Some(subject) = validation_subjects.intersection(&test_subjects).next() {
        return Err(format!("calibration/test subject overlap: {subject}").into());
    }
    let validation_pairs: BTreeSet<&str> = validation_rows.iter().map(|row| row.pair_id.as_str()).collect();
    let test_pairs: BTreeSet<&str> = test_rows.iter().map(|row| row.pair_id.as_str()).collect();
    if let Some(pair_id) = validation_pairs.intersection(&test_pairs).next() {
        return Err(format!("calibration/test pair_id overlap: {pair_id}").into());
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct ThresholdSelection {
    threshold: Option<f64>,
    true_positives: usize,
    false_positives: usize,
    positives: usize,
    negatives: usize,
}
impl ThresholdSelection {
    fn tpr(self) -> f64 { ratio(self.true_positives, self.positives) }
    fn fpr(self) -> f64 { ratio(self.false_positives, self.negatives) }
}

#[derive(Clone, Copy, Debug)]
struct OperatingRates {
    true_positives: usize,
    false_positives: usize,
    positives: usize,
    negatives: usize,
}
impl OperatingRates {
    fn tpr(self) -> f64 { ratio(self.true_positives, self.positives) }
    fn fpr(self) -> f64 { ratio(self.false_positives, self.negatives) }
}
fn ratio(numerator: usize, denominator: usize) -> f64 {
    if denominator == 0 { 0.0 } else { numerator as f64 / denominator as f64 }
}

fn select_validation_threshold(rows: &[TruthRowWithScore], max_fpr: f64) -> ThresholdSelection {
    let positives = rows.iter().filter(|row| row.label == 1).count();
    let negatives = rows.len() - positives;
    let mut sorted: Vec<&TruthRowWithScore> = rows.iter().collect();
    sorted.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut best = ThresholdSelection {
        threshold: None, true_positives: 0, false_positives: 0, positives, negatives,
    };
    let mut tp = 0usize;
    let mut fp = 0usize;
    let mut i = 0usize;
    while i < sorted.len() {
        let mut j = i + 1;
        while j < sorted.len() && sorted[j].score == sorted[i].score { j += 1; }
        tp += sorted[i..j].iter().filter(|row| row.label == 1).count();
        fp += sorted[i..j].iter().filter(|row| row.label == 0).count();
        let fpr = ratio(fp, negatives);
        let tpr = ratio(tp, positives);
        if fpr <= max_fpr + f64::EPSILON
            && (tpr > best.tpr() || (tpr == best.tpr() && fpr < best.fpr()))
        {
            best.threshold = Some(sorted[i].score);
            best.true_positives = tp;
            best.false_positives = fp;
        }
        i = j;
    }
    best
}

fn fixed_operating_rates(rows: &[TruthRowWithScore], threshold: Option<f64>) -> OperatingRates {
    let positives = rows.iter().filter(|row| row.label == 1).count();
    let negatives = rows.len() - positives;
    let mut tp = 0usize;
    let mut fp = 0usize;
    for row in rows {
        let predicted_link = threshold.is_some_and(|cutoff| row.score >= cutoff);
        if predicted_link && row.label == 1 { tp += 1; }
        if predicted_link && row.label == 0 { fp += 1; }
    }
    OperatingRates { true_positives: tp, false_positives: fp, positives, negatives }
}

fn fixed_threshold_cluster_bootstrap(
    rows: &[TruthRowWithScore],
    threshold: Option<f64>,
    replicates: usize,
    seed: u64,
) -> (Interval, Interval) {
    let subjects: Vec<String> = rows.iter()
        .flat_map(|row| [row.subject_a.clone(), row.subject_b.clone()])
        .collect::<BTreeSet<_>>().into_iter().collect();
    if subjects.is_empty() {
        let empty = Interval { low: f64::NAN, high: f64::NAN, valid_replicates: 0 };
        return (empty, empty);
    }
    let mut rng = XorShift64::new(seed);
    let mut tprs = Vec::with_capacity(replicates);
    let mut fprs = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut multiplicity: HashMap<&str, u64> = HashMap::new();
        for _ in 0..subjects.len() {
            let index = (rng.next() % subjects.len() as u64) as usize;
            *multiplicity.entry(subjects[index].as_str()).or_insert(0) += 1;
        }
        let mut tp = 0.0;
        let mut fp = 0.0;
        let mut positives = 0.0;
        let mut negatives = 0.0;
        for row in rows {
            let a = *multiplicity.get(row.subject_a.as_str()).unwrap_or(&0) as f64;
            let b = *multiplicity.get(row.subject_b.as_str()).unwrap_or(&0) as f64;
            let weight = if row.subject_a == row.subject_b { a } else { a * b };
            if weight == 0.0 { continue; }
            let predicted_link = threshold.is_some_and(|cutoff| row.score >= cutoff);
            if row.label == 1 {
                positives += weight;
                if predicted_link { tp += weight; }
            } else {
                negatives += weight;
                if predicted_link { fp += weight; }
            }
        }
        if positives > 0.0 && negatives > 0.0 {
            tprs.push(tp / positives);
            fprs.push(fp / negatives);
        }
    }
    (percentile_interval(tprs), percentile_interval(fprs))
}

fn operating_point_gate(
    sample_size_met: bool,
    observed_false_positives: usize,
    fpr_interval: Interval,
    max_fpr: f64,
) -> &'static str {
    if !sample_size_met {
        "NOT_EVALUATED_BELOW_SAMPLE_MINIMUM"
    } else if observed_false_positives == 0 {
        "INCONCLUSIVE_ZERO_FP_CANNOT_BOUND_RARE_EVENT_RISK"
    } else if fpr_interval.high.is_finite() && fpr_interval.high <= max_fpr {
        "PROVISIONAL_PASS_REQUIRES_REVIEW"
    } else if fpr_interval.low.is_finite() && fpr_interval.low > max_fpr {
        "FAIL"
    } else {
        "INCONCLUSIVE"
    }
}

fn build_calibrated_report(
    validation_rows: &[TruthRowWithScore],
    test_rows: &[TruthRowWithScore],
    training_subjects: &HashSet<String>,
    selection: &ThresholdSelection,
    rates: &OperatingRates,
    tpr_interval: Interval,
    fpr_interval: Interval,
    max_fpr: f64,
    bootstraps: usize,
    seed: u64,
) -> String {
    let calibration_subjects: BTreeSet<&str> = validation_rows.iter()
        .flat_map(|row| [row.subject_a.as_str(), row.subject_b.as_str()]).collect();
    let test_subjects: BTreeSet<&str> = test_rows.iter()
        .flat_map(|row| [row.subject_a.as_str(), row.subject_b.as_str()]).collect();
    let sample_size_met =
        validation_rows.len() >= MIN_PAIRS_FOR_REVIEW
        && calibration_subjects.len() >= MIN_SUBJECTS_FOR_REVIEW
        && test_rows.len() >= MIN_PAIRS_FOR_REVIEW
        && test_subjects.len() >= MIN_SUBJECTS_FOR_REVIEW;
    let sample_size_gate = if sample_size_met { "MINIMUM_COUNTS_MET_REQUIRES_REVIEW" } else { "SMOKE_ONLY_BELOW_MINIMUM" };
    let gate = operating_point_gate(sample_size_met, rates.false_positives, fpr_interval, max_fpr);
    let rare_event_status = if rates.false_positives == 0 {
        "ZERO_FALSE_POSITIVES_DO_NOT_CERTIFY_TAIL_RISK"
    } else {
        "CLUSTER_BOOTSTRAP_INTERVAL_REPORTED_NO_FORMAL_PRIVACY_GUARANTEE"
    };
    let valid_replicates = tpr_interval.valid_replicates.min(fpr_interval.valid_replicates);
    format!(
r#"{{ 
  "schema_version": "1.0",
  "evaluator_version": "{}",
  "evaluation_mode": "validation_locked_operating_point",
  "threshold_source": "validation_only_locked_before_test",
  "seed": {},
  "requested_bootstrap_replicates": {},
  "valid_bootstrap_replicates": {},
  "max_fpr_target": {},
  "calibration_pairs": {},
  "calibration_subjects": {},
  "test_pairs": {},
  "positive_pairs": {},
  "negative_pairs": {},
  "unique_test_subjects": {},
  "training_subject_count": {},
  "subject_overlap_count": 0,
  "calibration_threshold": {},
  "validation_tpr_at_locked_threshold": {},
  "validation_fpr_at_locked_threshold": {},
  "true_positives": {},
  "false_positives": {},
  "empirical_fpr_resolution": {},
  "test_tpr_at_locked_threshold": {},
  "test_fpr_at_locked_threshold": {},
  "test_tpr_cluster_bootstrap_ci95": {{"low": {}, "high": {}, "valid_replicates": {}}},
  "test_fpr_cluster_bootstrap_ci95": {{"low": {}, "high": {}, "valid_replicates": {}}},
  "observed_zero_false_positives": {},
  "rare_event_inference_status": "{}",
  "sample_size_gate": "{}",
  "operating_point_gate": "{}",
  "task_utility_evidence_present": false,
  "official_conformance_evidence_present": false,
  "overall_release_gate": "NOT_EVALUATED_REQUIRED_UTILITY_AND_CONFORMANCE_MISSING",
  "interpretation": "Threshold selection uses validation labels only and is frozen before test scoring. Test rates are evaluated at that fixed threshold. Subject-cluster bootstrap intervals describe variability under the declared dyad model; zero observed false positives are not proof of zero tail risk. No task utility or official conformance evidence is provided."
}}"#,
        VERSION, seed, bootstraps, valid_replicates, max_fpr,
        validation_rows.len(), calibration_subjects.len(), test_rows.len(),
        rates.positives, rates.negatives, test_subjects.len(), training_subjects.len(),
        metric_or_null(selection.threshold), selection.tpr(), selection.fpr(),
        rates.true_positives, rates.false_positives, ratio(1, rates.negatives),
        rates.tpr(), rates.fpr(),
        metric_or_null(tpr_interval.low.is_finite().then_some(tpr_interval.low)),
        metric_or_null(tpr_interval.high.is_finite().then_some(tpr_interval.high)),
        tpr_interval.valid_replicates,
        metric_or_null(fpr_interval.low.is_finite().then_some(fpr_interval.low)),
        metric_or_null(fpr_interval.high.is_finite().then_some(fpr_interval.high)),
        fpr_interval.valid_replicates,
        rates.false_positives == 0, rare_event_status, sample_size_gate, gate
    )
}

fn read_truth(path: &str) -> Result<Vec<TruthRow>, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    let header = lines.next().ok_or("truth TSV is empty")?;
    if header != "pair_id\tsubject_a\tsubject_b\tlabel\tstratum" {
        return Err("truth TSV header must be exactly: pair_id<TAB>subject_a<TAB>subject_b<TAB>label<TAB>stratum".into());
    }
    let mut out = Vec::new();
    let mut ids = HashSet::new();
    for (line_no, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split('\t').collect();
        if fields.len() != 5 {
            return Err(format!("truth TSV line {} must contain 5 tab-separated fields", line_no + 2).into());
        }
        let (pair_id, subject_a, subject_b, label_text, stratum) =
            (fields[0], fields[1], fields[2], fields[3], fields[4]);
        validate_token(pair_id, "pair_id", "pair_")?;
        validate_token(subject_a, "subject_a", "")?;
        validate_token(subject_b, "subject_b", "")?;
        validate_token(stratum, "stratum", "")?;
        if !ids.insert(pair_id.to_owned()) {
            return Err(format!("duplicate pair_id in truth TSV: {pair_id}").into());
        }
        let label = label_text.parse::<u8>()?;
        if label > 1 {
            return Err(format!("label must be 0 or 1 on line {}", line_no + 2).into());
        }
        let same_subject = subject_a == subject_b;
        if (label == 1) != same_subject {
            return Err(format!("label/subject oracle mismatch on line {}: label 1 requires subject_a == subject_b", line_no + 2).into());
        }
        out.push(TruthRow {
            pair_id: pair_id.to_owned(),
            subject_a: subject_a.to_owned(),
            subject_b: subject_b.to_owned(),
            label,
            stratum: stratum.to_owned(),
        });
    }
    if out.is_empty() {
        return Err("truth TSV has no data rows".into());
    }
    Ok(out)
}

fn read_scores(path: &str) -> Result<HashMap<String, f64>, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut lines = text.lines();
    let header = lines.next().ok_or("scores TSV is empty")?;
    if header != "pair_id\tscore" {
        return Err("scores TSV header must be exactly: pair_id<TAB>score; labels and subject identifiers are forbidden here".into());
    }
    let mut out = HashMap::new();
    for (line_no, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split('\t').collect();
        if fields.len() != 2 {
            return Err(format!("scores TSV line {} must contain exactly 2 fields", line_no + 2).into());
        }
        validate_token(fields[0], "pair_id", "pair_")?;
        let score = fields[1].parse::<f64>()?;
        if !score.is_finite() {
            return Err(format!("score must be finite on line {}", line_no + 2).into());
        }
        if out.insert(fields[0].to_owned(), score).is_some() {
            return Err(format!("duplicate pair_id in scores TSV: {}", fields[0]).into());
        }
    }
    if out.is_empty() {
        return Err("scores TSV has no data rows".into());
    }
    Ok(out)
}

fn read_training_subjects(path: &str) -> Result<HashSet<String>, Box<dyn Error>> {
    let text = fs::read_to_string(path)?;
    let mut out = HashSet::new();
    for (line_no, line) in text.lines().enumerate() {
        let value = line.trim();
        if value.is_empty() {
            continue;
        }
        validate_token(value, "training subject", "")?;
        if !out.insert(value.to_owned()) {
            return Err(format!("duplicate training subject on line {}", line_no + 1).into());
        }
    }
    Ok(out)
}

fn validate_token(value: &str, field: &str, required_prefix: &str) -> Result<(), Box<dyn Error>> {
    if value.is_empty()
        || (!required_prefix.is_empty() && !value.starts_with(required_prefix))
    {
        return Err(format!("invalid {field}: empty or missing required prefix").into());
    }
    if value.len() > 128
        || !value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-' || b == b'.')
    {
        return Err(format!("invalid {field}: use at most 128 ASCII letters, digits, '.', '_' or '-'; do not use real identifiers").into());
    }
    Ok(())
}

fn join_and_validate(
    truth: Vec<TruthRow>,
    mut scores: HashMap<String, f64>,
    training_subjects: &HashSet<String>,
) -> Result<Vec<TruthRowWithScore>, Box<dyn Error>> {
    let expected: HashSet<String> = truth.iter().map(|r| r.pair_id.clone()).collect();
    if expected.len() != scores.len() {
        return Err(format!("truth/score row-count mismatch: {} truth rows, {} scored rows", expected.len(), scores.len()).into());
    }
    if let Some(extra) = scores.keys().find(|id| !expected.contains(*id)) {
        return Err(format!("score file contains unknown pair_id {extra}").into());
    }
    let mut output = Vec::with_capacity(truth.len());
    let mut overlap = BTreeSet::new();
    for row in truth {
        if training_subjects.contains(&row.subject_a) {
            overlap.insert(row.subject_a.clone());
        }
        if training_subjects.contains(&row.subject_b) {
            overlap.insert(row.subject_b.clone());
        }
        let score = scores.remove(&row.pair_id).ok_or_else(|| format!("missing score for pair_id {}", row.pair_id))?;
        output.push(TruthRowWithScore {
            pair_id: row.pair_id,
            subject_a: row.subject_a,
            subject_b: row.subject_b,
            label: row.label,
            stratum: row.stratum,
            score,
        });
    }
    if !overlap.is_empty() {
        return Err(format!("subject-disjointness violated: {} test subject(s) also appear in training-subjects.txt", overlap.len()).into());
    }
    output.sort_by(|a, b| a.pair_id.cmp(&b.pair_id));
    Ok(output)
}

#[derive(Clone, Debug)]
struct TruthRowWithScore {
    pair_id: String,
    subject_a: String,
    subject_b: String,
    label: u8,
    stratum: String,
    score: f64,
}

fn observations<'a, I>(rows: I) -> Vec<WeightedObservation>
where
    I: IntoIterator<Item = &'a TruthRowWithScore>,
{
    rows.into_iter()
        .map(|row| WeightedObservation { score: row.score, label: row.label, weight: 1.0 })
        .collect()
}

fn compute_metrics(obs: &[WeightedObservation]) -> Option<Metrics> {
    let positives: f64 = obs.iter().filter(|o| o.label == 1).map(|o| o.weight).sum();
    let negatives: f64 = obs.iter().filter(|o| o.label == 0).map(|o| o.weight).sum();
    if positives <= 0.0 || negatives <= 0.0 {
        return None;
    }
    Some(Metrics {
        roc_auc: weighted_auc(obs, positives, negatives),
        average_precision: weighted_average_precision(obs, positives),
        tpr_at_fpr_1pct: weighted_tpr_at_fpr(obs, positives, negatives, 0.01),
        tpr_at_fpr_5pct: weighted_tpr_at_fpr(obs, positives, negatives, 0.05),
    })
}

fn weighted_auc(obs: &[WeightedObservation], positives: f64, negatives: f64) -> f64 {
    let mut sorted = obs.to_vec();
    sorted.sort_by(|a, b| a.score.total_cmp(&b.score));
    let mut cumulative_weight = 0.0;
    let mut positive_rank_sum = 0.0;
    let mut i = 0;
    while i < sorted.len() {
        let mut j = i + 1;
        while j < sorted.len() && sorted[j].score == sorted[i].score {
            j += 1;
        }
        let group_weight: f64 = sorted[i..j].iter().map(|o| o.weight).sum();
        let positive_weight: f64 = sorted[i..j].iter().filter(|o| o.label == 1).map(|o| o.weight).sum();
        let average_rank = cumulative_weight + (group_weight + 1.0) / 2.0;
        positive_rank_sum += average_rank * positive_weight;
        cumulative_weight += group_weight;
        i = j;
    }
    let rank_adjustment = positives * (positives + 1.0) / 2.0;
    ((positive_rank_sum - rank_adjustment) / (positives * negatives)).clamp(0.0, 1.0)
}

fn weighted_average_precision(obs: &[WeightedObservation], positives: f64) -> f64 {
    let mut sorted = obs.to_vec();
    sorted.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut cumulative_positive = 0.0;
    let mut cumulative_total = 0.0;
    let mut ap = 0.0;
    let mut i = 0;
    while i < sorted.len() {
        let mut j = i + 1;
        while j < sorted.len() && sorted[j].score == sorted[i].score {
            j += 1;
        }
        let group_positive: f64 = sorted[i..j].iter().filter(|o| o.label == 1).map(|o| o.weight).sum();
        let group_total: f64 = sorted[i..j].iter().map(|o| o.weight).sum();
        cumulative_positive += group_positive;
        cumulative_total += group_total;
        if group_positive > 0.0 {
            ap += (group_positive / positives) * (cumulative_positive / cumulative_total);
        }
        i = j;
    }
    ap.clamp(0.0, 1.0)
}

fn weighted_tpr_at_fpr(obs: &[WeightedObservation], positives: f64, negatives: f64, max_fpr: f64) -> f64 {
    let mut sorted = obs.to_vec();
    sorted.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut tp = 0.0;
    let mut fp = 0.0;
    let mut best: f64 = 0.0;
    let mut i = 0;
    while i < sorted.len() {
        let mut j = i + 1;
        while j < sorted.len() && sorted[j].score == sorted[i].score {
            j += 1;
        }
        tp += sorted[i..j].iter().filter(|o| o.label == 1).map(|o| o.weight).sum::<f64>();
        fp += sorted[i..j].iter().filter(|o| o.label == 0).map(|o| o.weight).sum::<f64>();
        let fpr = fp / negatives;
        let tpr = tp / positives;
        if fpr <= max_fpr + f64::EPSILON {
            best = best.max(tpr);
        }
        i = j;
    }
    best.clamp(0.0, 1.0)
}

fn subject_cluster_bootstrap(rows: &[TruthRowWithScore], replicates: usize, seed: u64) -> Interval {
    let subjects: Vec<String> = rows
        .iter()
        .flat_map(|row| [row.subject_a.clone(), row.subject_b.clone()])
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let mut rng = XorShift64::new(seed);
    let mut aucs = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut multiplicity: HashMap<&str, u64> = HashMap::new();
        for _ in 0..subjects.len() {
            let index = (rng.next() % subjects.len() as u64) as usize;
            *multiplicity.entry(subjects[index].as_str()).or_insert(0) += 1;
        }
        let weighted: Vec<WeightedObservation> = rows
            .iter()
            .filter_map(|row| {
                let a = *multiplicity.get(row.subject_a.as_str()).unwrap_or(&0) as f64;
                let b = *multiplicity.get(row.subject_b.as_str()).unwrap_or(&0) as f64;
                let weight = if row.subject_a == row.subject_b { a } else { a * b };
                (weight > 0.0).then_some(WeightedObservation { score: row.score, label: row.label, weight })
            })
            .collect();
        let positive: f64 = weighted.iter().filter(|o| o.label == 1).map(|o| o.weight).sum();
        let negative: f64 = weighted.iter().filter(|o| o.label == 0).map(|o| o.weight).sum();
        if positive > 0.0 && negative > 0.0 {
            aucs.push(weighted_auc(&weighted, positive, negative));
        }
    }
    percentile_interval(aucs)
}

fn percentile_interval(mut values: Vec<f64>) -> Interval {
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
        return Interval { low: f64::NAN, high: f64::NAN, valid_replicates: 0 };
    }
    let low_index = ((values.len() as f64 * 0.025).floor() as usize).min(values.len() - 1);
    let high_index = ((values.len() as f64 * 0.975).floor() as usize).min(values.len() - 1);
    Interval { low: values[low_index], high: values[high_index], valid_replicates: values.len() }
}

struct XorShift64 { state: u64 }
impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self { state: if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed } }
    }
    fn next(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }
}

fn build_report(rows: &[TruthRowWithScore], training_subjects: &HashSet<String>, bootstraps: usize, seed: u64) -> String {
    let all = observations(rows);
    let metrics = compute_metrics(&all).expect("validated data has positive and negative pairs");
    let interval = subject_cluster_bootstrap(rows, bootstraps, seed);
    let positives = rows.iter().filter(|r| r.label == 1).count();
    let negatives = rows.len() - positives;
    let subjects: BTreeSet<&str> = rows
        .iter()
        .flat_map(|r| [r.subject_a.as_str(), r.subject_b.as_str()])
        .collect();
    let sample_status = if rows.len() < MIN_PAIRS_FOR_REVIEW || subjects.len() < MIN_SUBJECTS_FOR_REVIEW {
        "SMOKE_ONLY_BELOW_MINIMUM"
    } else {
        "BENCHMARK_SIZED_REQUIRES_REVIEW_NOT_A_PASS"
    };
    let mut strata: BTreeMap<&str, Vec<&TruthRowWithScore>> = BTreeMap::new();
    for row in rows {
        strata.entry(&row.stratum).or_default().push(row);
    }
    let by_stratum = strata
        .iter()
        .map(|(name, group)| {
            let obs = observations(group.iter().copied());
            let m = compute_metrics(&obs);
            let p = group.iter().filter(|r| r.label == 1).count();
            let n = group.len() - p;
            format!(
                "{{\"stratum\":\"{}\",\"pairs\":{},\"positive_pairs\":{},\"negative_pairs\":{},\"roc_auc\":{},\"average_precision\":{},\"tpr_at_fpr_1pct\":{},\"tpr_at_fpr_5pct\":{}}}",
                json_escape(name), group.len(), p, n,
                metric_or_null(m.map(|x| x.roc_auc)), metric_or_null(m.map(|x| x.average_precision)),
                metric_or_null(m.map(|x| x.tpr_at_fpr_1pct)), metric_or_null(m.map(|x| x.tpr_at_fpr_5pct))
            )
        })
        .collect::<Vec<_>>()
        .join(",\n    ");
    format!(
        "{{\n  \"schema_version\":\"1.0\",\n  \"evaluator_version\":\"{}\",\n  \"evaluation_status\":\"{}\",\n  \"seed\":{},\n  \"requested_bootstrap_replicates\":{},\n  \"valid_bootstrap_replicates\":{},\n  \"pairs\":{},\n  \"positive_pairs\":{},\n  \"negative_pairs\":{},\n  \"unique_test_subjects\":{},\n  \"training_subject_count\":{},\n  \"subject_overlap_count\":0,\n  \"low_fpr_threshold_selection\":\"empirical_test_curve_descriptive_not_locked\",\n  \"low_fpr_metrics_are_certification_ready\":false,\n  \"metrics\":{{\n    \"roc_auc\":{},\n    \"average_precision\":{},\n    \"tpr_at_fpr_1pct\":{},\n    \"tpr_at_fpr_5pct\":{},\n    \"roc_auc_cluster_bootstrap_ci95\":{{\"low\":{},\"high\":{}}}\n  }},\n  \"by_stratum\":[\n    {}\n  ],\n  \"interpretation\":\"Metrics describe only the supplied attack scores and synthetic/test cohort. Low-FPR TPR values are empirical test-curve summaries with no validation-locked threshold and are not deployable operating points. Sample-size status is not a privacy pass; threat model, attack adequacy, dataset provenance, and held-out subject split require review.\"\n}}",
        VERSION, sample_status, seed, bootstraps, interval.valid_replicates, rows.len(), positives, negatives,
        subjects.len(), training_subjects.len(), metrics.roc_auc, metrics.average_precision, metrics.tpr_at_fpr_1pct, metrics.tpr_at_fpr_5pct,
        metric_or_null((!interval.low.is_nan()).then_some(interval.low)), metric_or_null((!interval.high.is_nan()).then_some(interval.high)), by_stratum
    )
}


fn paired_auc_delta_bootstrap(rows: &[PairedScoreRow], replicates: usize, seed: u64) -> Interval {
    let subjects: Vec<String> = rows
        .iter()
        .flat_map(|row| [row.subject_a.clone(), row.subject_b.clone()])
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    if subjects.is_empty() {
        return Interval { low: f64::NAN, high: f64::NAN, valid_replicates: 0 };
    }
    let mut rng = XorShift64::new(seed);
    let mut deltas = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut multiplicity: HashMap<&str, u64> = HashMap::new();
        for _ in 0..subjects.len() {
            let index = (rng.next() % subjects.len() as u64) as usize;
            *multiplicity.entry(subjects[index].as_str()).or_insert(0) += 1;
        }
        let mut baseline_obs = Vec::with_capacity(rows.len());
        let mut treatment_obs = Vec::with_capacity(rows.len());
        for row in rows {
            let a = *multiplicity.get(row.subject_a.as_str()).unwrap_or(&0) as f64;
            let b = *multiplicity.get(row.subject_b.as_str()).unwrap_or(&0) as f64;
            let weight = if row.subject_a == row.subject_b { a } else { a * b };
            if weight > 0.0 {
                baseline_obs.push(WeightedObservation { score: row.baseline_score, label: row.label, weight });
                treatment_obs.push(WeightedObservation { score: row.treatment_score, label: row.label, weight });
            }
        }
        if let (Some(baseline), Some(treatment)) = (compute_metrics(&baseline_obs), compute_metrics(&treatment_obs)) {
            deltas.push(treatment.roc_auc - baseline.roc_auc);
        }
    }
    percentile_interval(deltas)
}

fn paired_project(rows: &[PairedScoreRow], use_treatment: bool) -> Vec<TruthRowWithScore> {
    rows.iter()
        .map(|row| TruthRowWithScore {
            pair_id: row.pair_id.clone(),
            subject_a: row.subject_a.clone(),
            subject_b: row.subject_b.clone(),
            label: row.label,
            stratum: row.stratum.clone(),
            score: if use_treatment { row.treatment_score } else { row.baseline_score },
        })
        .collect()
}

fn build_paired_report(
    rows: &[PairedScoreRow],
    training_subjects: &HashSet<String>,
    bootstraps: usize,
    seed: u64,
    max_auc_upper: f64,
) -> String {
    let baseline_rows = paired_project(rows, false);
    let treatment_rows = paired_project(rows, true);
    let baseline_metrics = compute_metrics(&observations(&baseline_rows)).expect("validated data has positive and negative pairs");
    let treatment_metrics = compute_metrics(&observations(&treatment_rows)).expect("validated data has positive and negative pairs");
    let baseline_interval = subject_cluster_bootstrap(&baseline_rows, bootstraps, seed);
    let treatment_interval = subject_cluster_bootstrap(&treatment_rows, bootstraps, seed);
    let delta_interval = paired_auc_delta_bootstrap(rows, bootstraps, seed ^ 0xA5A5_9901_1234_5678);
    let positives = rows.iter().filter(|row| row.label == 1).count();
    let negatives = rows.len() - positives;
    let subjects: BTreeSet<&str> = rows
        .iter()
        .flat_map(|row| [row.subject_a.as_str(), row.subject_b.as_str()])
        .collect();
    let sample_size_met = rows.len() >= MIN_PAIRS_FOR_REVIEW && subjects.len() >= MIN_SUBJECTS_FOR_REVIEW;
    let sample_size_gate = if sample_size_met { "MINIMUM_COUNTS_MET_REQUIRES_REVIEW" } else { "SMOKE_ONLY_BELOW_MINIMUM" };
    let unlinkability_auc_gate = if !sample_size_met {
        "NOT_EVALUATED_BELOW_SAMPLE_MINIMUM"
    } else if treatment_interval.high.is_finite() && treatment_interval.high <= max_auc_upper {
        "PROVISIONAL_PASS_REQUIRES_REVIEW"
    } else if treatment_interval.low.is_finite() && treatment_interval.low > max_auc_upper {
        "FAIL"
    } else {
        "INCONCLUSIVE"
    };
    let linkability_change_status = if delta_interval.high.is_finite() && delta_interval.high < 0.0 {
        "REDUCTION_SUPPORTED"
    } else if delta_interval.low.is_finite() && delta_interval.low > 0.0 {
        "LINKABILITY_WORSENED"
    } else {
        "INCONCLUSIVE"
    };
    let valid_replicates = baseline_interval.valid_replicates
        .min(treatment_interval.valid_replicates)
        .min(delta_interval.valid_replicates);
    format!(
r#"{{ 
  "schema_version": "1.0",
  "evaluator_version": "{}",
  "evaluation_mode": "paired_mitigation_comparison",
  "seed": {},
  "requested_bootstrap_replicates": {},
  "valid_bootstrap_replicates": {},
  "pair_count": {},
  "positive_pairs": {},
  "negative_pairs": {},
  "unique_test_subjects": {},
  "training_subject_count": {},
  "subject_overlap_count": 0,
  "baseline_metrics": {},
  "treatment_metrics": {},
  "treatment_minus_baseline_auc": {},
  "paired_delta_auc_cluster_bootstrap_ci95": {{"low": {}, "high": {}, "valid_replicates": {}}},
  "proposed_max_auc_upper": {},
  "sample_size_gate": "{}",
  "linkability_change_status": "{}",
  "unlinkability_auc_gate": "{}",
  "task_utility_evidence_present": false,
  "official_conformance_evidence_present": false,
  "overall_release_gate": "NOT_EVALUATED_REQUIRED_UTILITY_AND_CONFORMANCE_MISSING",
  "interpretation": "A paired subject-cluster bootstrap compares baseline and treatment ranking scores on identical pairs. This interval supports only the supplied test cohort and declared score files. Smoke-size data cannot qualify a privacy profile; a supported AUC reduction is not an unlinkability pass. Task utility and official conformance are not inputs and are never inferred."
}}"#,
        VERSION, seed, bootstraps, valid_replicates, rows.len(), positives, negatives,
        subjects.len(), training_subjects.len(),
        metrics_json(baseline_metrics, baseline_interval),
        metrics_json(treatment_metrics, treatment_interval),
        metric_or_null(Some(treatment_metrics.roc_auc - baseline_metrics.roc_auc)),
        metric_or_null(delta_interval.low.is_finite().then_some(delta_interval.low)),
        metric_or_null(delta_interval.high.is_finite().then_some(delta_interval.high)),
        delta_interval.valid_replicates,
        max_auc_upper, sample_size_gate, linkability_change_status, unlinkability_auc_gate
    )
}

fn metrics_json(metrics: Metrics, interval: Interval) -> String {
    format!(
        r#"{{"roc_auc":{},"average_precision":{},"tpr_at_fpr_1pct":{},"tpr_at_fpr_5pct":{},"roc_auc_cluster_bootstrap_ci95":{{"low":{},"high":{},"valid_replicates":{}}}}}"#,
        metric_or_null(Some(metrics.roc_auc)),
        metric_or_null(Some(metrics.average_precision)),
        metric_or_null(Some(metrics.tpr_at_fpr_1pct)),
        metric_or_null(Some(metrics.tpr_at_fpr_5pct)),
        metric_or_null(interval.low.is_finite().then_some(interval.low)),
        metric_or_null(interval.high.is_finite().then_some(interval.high)),
        interval.valid_replicates
    )
}

fn metric_or_null(value: Option<f64>) -> String {
    value.filter(|x| x.is_finite()).map_or_else(|| "null".to_owned(), |x| format!("{x:.8}"))
}

fn json_escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(items: &[(f64, u8)]) -> Vec<WeightedObservation> {
        items.iter().map(|(score, label)| WeightedObservation { score: *score, label: *label, weight: 1.0 }).collect()
    }

    #[test]
    fn auc_perfect_separation_is_one() {
        assert!((compute_metrics(&obs(&[(0.1, 0), (0.8, 1), (0.2, 0), (0.9, 1)])).unwrap().roc_auc - 1.0).abs() < 1e-12);
    }

    #[test]
    fn auc_complete_tie_is_half() {
        assert!((compute_metrics(&obs(&[(0.5, 0), (0.5, 1)])).unwrap().roc_auc - 0.5).abs() < 1e-12);
    }

    #[test]
    fn low_fpr_recall_does_not_split_tied_scores() {
        let m = compute_metrics(&obs(&[(0.9, 1), (0.8, 1), (0.1, 0), (0.2, 0)])).unwrap();
        assert!((m.tpr_at_fpr_1pct - 1.0).abs() < 1e-12);
        assert!((m.tpr_at_fpr_5pct - 1.0).abs() < 1e-12);
    }

    #[test]
    fn average_precision_respects_ranking() {
        let good = compute_metrics(&obs(&[(0.9, 1), (0.8, 1), (0.1, 0), (0.2, 0)])).unwrap().average_precision;
        let bad = compute_metrics(&obs(&[(0.1, 1), (0.2, 1), (0.8, 0), (0.9, 0)])).unwrap().average_precision;
        assert!(good > bad);
    }

    #[test]
    fn bootstrap_is_deterministic_for_same_seed() {
        let rows: Vec<TruthRowWithScore> = (0..12).map(|i| {
            let subject = format!("test_subject_{i:03}");
            TruthRowWithScore { pair_id: format!("pair_{i:03}"), subject_a: subject.clone(), subject_b: subject,
                label: 1, stratum: "test".to_owned(), score: 0.5 + i as f64 / 20.0 }
        }).chain((0..12).map(|i| {
            TruthRowWithScore { pair_id: format!("pair_neg_{i:03}"), subject_a: format!("test_subject_{i:03}"), subject_b: format!("test_subject_{:03}", i + 12),
                label: 0, stratum: "test".to_owned(), score: i as f64 / 30.0 }
        })).collect();
        let a = subject_cluster_bootstrap(&rows, 200, 9901);
        let b = subject_cluster_bootstrap(&rows, 200, 9901);
        assert_eq!(a.valid_replicates, b.valid_replicates);
        assert_eq!(a.low.to_bits(), b.low.to_bits());
        assert_eq!(a.high.to_bits(), b.high.to_bits());
    }

    #[test]
    fn score_parser_rejects_subject_and_label_columns() {
        let path = temp_path("extra-columns");
        fs::write(&path, "pair_id\tsubject_a\tsubject_b\tlabel\tscore\npair_001\ts1\ts1\t1\t0.9\n").unwrap();
        assert!(read_scores(path.to_str().unwrap()).is_err());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn score_parser_rejects_non_finite_values() {
        let path = temp_path("non-finite");
        fs::write(&path, "pair_id\tscore\npair_001\tNaN\n").unwrap();
        assert!(read_scores(path.to_str().unwrap()).is_err());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn subject_overlap_fails_closed() {
        let truth = vec![TruthRow { pair_id: "pair_001".to_owned(), subject_a: "test_subject_001".to_owned(),
            subject_b: "test_subject_001".to_owned(), label: 1, stratum: "x".to_owned() }];
        let mut scores = HashMap::new();
        scores.insert("pair_001".to_owned(), 0.7);
        let training = HashSet::from(["test_subject_001".to_owned()]);
        assert!(join_and_validate(truth, scores, &training).is_err());
    }


    fn paired_positive_control_rows() -> Vec<PairedScoreRow> {
        let mut rows = Vec::new();
        for i in 0..8 {
            let subject = format!("test_subject_{i:03}");
            rows.push(PairedScoreRow {
                pair_id: format!("pair_positive_{i:03}"),
                subject_a: subject.clone(),
                subject_b: subject,
                label: 1,
                stratum: "known_answer".to_owned(),
                baseline_score: 0.9,
                treatment_score: 0.5,
            });
        }
        for i in 0..8 {
            rows.push(PairedScoreRow {
                pair_id: format!("pair_negative_{i:03}"),
                subject_a: format!("test_subject_{i:03}"),
                subject_b: format!("test_subject_{:03}", i + 8),
                label: 0,
                stratum: "known_answer".to_owned(),
                baseline_score: 0.1,
                treatment_score: 0.5,
            });
        }
        rows
    }

    #[test]
    fn paired_bootstrap_detects_known_answer_auc_reduction() {
        let delta = paired_auc_delta_bootstrap(&paired_positive_control_rows(), 300, 9901);
        assert!(delta.valid_replicates >= 100);
        assert!((delta.low + 0.5).abs() < 1e-12);
        assert!((delta.high + 0.5).abs() < 1e-12);
    }

    #[test]
    fn paired_bootstrap_is_deterministic() {
        let rows = paired_positive_control_rows();
        let a = paired_auc_delta_bootstrap(&rows, 200, 44);
        let b = paired_auc_delta_bootstrap(&rows, 200, 44);
        assert_eq!(a.valid_replicates, b.valid_replicates);
        assert_eq!(a.low.to_bits(), b.low.to_bits());
        assert_eq!(a.high.to_bits(), b.high.to_bits());
    }

    #[test]
    fn paired_join_rejects_different_pair_sets() {
        let truth = vec![TruthRow {
            pair_id: "pair_001".to_owned(),
            subject_a: "test_subject_001".to_owned(),
            subject_b: "test_subject_001".to_owned(),
            label: 1,
            stratum: "known_answer".to_owned(),
        }];
        let baseline = HashMap::from([("pair_001".to_owned(), 0.9)]);
        let treatment = HashMap::from([("pair_999".to_owned(), 0.5)]);
        assert!(join_paired_scores(truth, baseline, treatment, &HashSet::new()).is_err());
    }

    #[test]
    fn paired_report_never_claims_complete_release_without_utility_and_conformance() {
        let report = build_paired_report(&paired_positive_control_rows(), &HashSet::new(), 100, 9901, 0.55);
        assert!(report.contains("REDUCTION_SUPPORTED"));
        assert!(report.contains("SMOKE_ONLY_BELOW_MINIMUM"));
        assert!(report.contains("NOT_EVALUATED_REQUIRED_UTILITY_AND_CONFORMANCE_MISSING"));
        assert!(report.contains("task_utility_evidence_present"));
    }


    fn calibration_fixture_rows() -> Vec<TruthRowWithScore> {
        vec![
            TruthRowWithScore { pair_id: "pair_pos_a".into(), subject_a: "person_a".into(), subject_b: "person_a".into(), label: 1, stratum: "test".into(), score: 0.9 },
            TruthRowWithScore { pair_id: "pair_pos_b".into(), subject_a: "person_b".into(), subject_b: "person_b".into(), label: 1, stratum: "test".into(), score: 0.8 },
            TruthRowWithScore { pair_id: "pair_neg_ab".into(), subject_a: "person_a".into(), subject_b: "person_b".into(), label: 0, stratum: "test".into(), score: 0.7 },
            TruthRowWithScore { pair_id: "pair_neg_cd".into(), subject_a: "person_c".into(), subject_b: "person_d".into(), label: 0, stratum: "test".into(), score: 0.1 },
        ]
    }

    #[test]
    fn validation_threshold_meets_empirical_fpr_cap() {
        let selection = select_validation_threshold(&calibration_fixture_rows(), 0.25);
        assert_eq!(selection.threshold, Some(0.8));
        assert_eq!(selection.true_positives, 2);
        assert_eq!(selection.false_positives, 0);
    }

    #[test]
    fn test_operating_point_never_retunes_threshold() {
        let rows = calibration_fixture_rows();
        let rates = fixed_operating_rates(&rows, Some(0.65));
        assert_eq!(rates.true_positives, 2);
        assert_eq!(rates.false_positives, 1);
        assert_eq!(rates.fpr(), 0.5);
    }

    #[test]
    fn calibration_test_subject_overlap_fails_closed() {
        let validation = calibration_fixture_rows();
        let test = vec![TruthRowWithScore {
            pair_id: "pair_test".into(), subject_a: "person_a".into(), subject_b: "person_a".into(),
            label: 1, stratum: "test".into(), score: 0.5
        }];
        assert!(ensure_calibration_test_disjoint(&validation, &test).is_err());
    }

    #[test]
    fn zero_false_positives_do_not_pass_tail_risk_gate() {
        let interval = Interval { low: 0.0, high: 0.0, valid_replicates: 1000 };
        assert_eq!(operating_point_gate(true, 0, interval, 0.01), "INCONCLUSIVE_ZERO_FP_CANNOT_BOUND_RARE_EVENT_RISK");
    }

    #[test]
    fn calibrated_report_keeps_release_gate_closed_without_utility_and_conformance() {
        let validation = calibration_fixture_rows();
        let test: Vec<TruthRowWithScore> = validation.iter().map(|row| TruthRowWithScore {
            pair_id: format!("test_{}", row.pair_id),
            subject_a: format!("test_{}", row.subject_a),
            subject_b: format!("test_{}", row.subject_b),
            label: row.label, stratum: row.stratum.clone(), score: row.score
        }).collect();
        let selection = select_validation_threshold(&validation, 0.25);
        let rates = fixed_operating_rates(&test, selection.threshold);
        let (tpr_ci, fpr_ci) = fixed_threshold_cluster_bootstrap(&test, selection.threshold, 100, 99);
        let report = build_calibrated_report(&validation,&test,&HashSet::new(),&selection,&rates,tpr_ci,fpr_ci,0.25,100,99);
        assert!(report.contains("validation_locked_operating_point"));
        assert!(report.contains("NOT_EVALUATED_REQUIRED_UTILITY_AND_CONFORMANCE_MISSING"));
        assert!(report.contains("ZERO_FALSE_POSITIVES_DO_NOT_CERTIFY_TAIL_RISK"));
    }

    fn temp_path(label: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("life-privacy-eval-{}-{label}.tsv", std::process::id()))
    }
}
