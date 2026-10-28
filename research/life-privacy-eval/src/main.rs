use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;

const VERSION: &str = "0.1.0";
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
    if args.len() < 5
        || args.len() > 7
        || args
            .get(1)
            .is_some_and(|arg| arg == "--help" || arg == "-h")
    {
        eprintln!(
            "Usage: {} <truth.tsv> <scores.tsv> <training-subjects.txt> <report.json> [bootstrap-replicates=1000] [seed=9901]\n\ntruth.tsv has pair_id, subject_a, subject_b, label, stratum.\nscores.tsv has only pair_id and score; never give ground-truth fields to the attack scorer.",
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
    aucs.sort_by(f64::total_cmp);
    if aucs.is_empty() {
        return Interval { low: f64::NAN, high: f64::NAN, valid_replicates: 0 };
    }
    let low_index = ((aucs.len() as f64 * 0.025).floor() as usize).min(aucs.len() - 1);
    let high_index = ((aucs.len() as f64 * 0.975).floor() as usize).min(aucs.len() - 1);
    Interval { low: aucs[low_index], high: aucs[high_index], valid_replicates: aucs.len() }
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
        "{{\n  \"schema_version\":\"1.0\",\n  \"evaluator_version\":\"{}\",\n  \"evaluation_status\":\"{}\",\n  \"seed\":{},\n  \"requested_bootstrap_replicates\":{},\n  \"valid_bootstrap_replicates\":{},\n  \"pairs\":{},\n  \"positive_pairs\":{},\n  \"negative_pairs\":{},\n  \"unique_test_subjects\":{},\n  \"training_subject_count\":{},\n  \"subject_overlap_count\":0,\n  \"metrics\":{{\n    \"roc_auc\":{},\n    \"average_precision\":{},\n    \"tpr_at_fpr_1pct\":{},\n    \"tpr_at_fpr_5pct\":{},\n    \"roc_auc_cluster_bootstrap_ci95\":{{\"low\":{},\"high\":{}}}\n  }},\n  \"by_stratum\":[\n    {}\n  ],\n  \"interpretation\":\"Metrics describe only the supplied attack scores and synthetic/test cohort. Sample-size status is not a privacy pass; threat model, attack adequacy, dataset provenance, and held-out subject split require review.\"\n}}",
        VERSION, sample_status, seed, bootstraps, interval.valid_replicates, rows.len(), positives, negatives,
        subjects.len(), training_subjects.len(), metrics.roc_auc, metrics.average_precision, metrics.tpr_at_fpr_1pct, metrics.tpr_at_fpr_5pct,
        metric_or_null((!interval.low.is_nan()).then_some(interval.low)), metric_or_null((!interval.high.is_nan()).then_some(interval.high)), by_stratum
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

    fn temp_path(label: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("life-privacy-eval-{}-{label}.tsv", std::process::id()))
    }
}
