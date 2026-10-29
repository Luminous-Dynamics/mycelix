use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::error::Error;
use std::fs;

const VERSION: &str = "0.6.0";
const SUBJECT_RISK_ALPHA: f64 = 0.05;
const DEFAULT_BOOTSTRAPS: usize = 1_000;
const DEFAULT_SEED: u64 = 9_901;
const MIN_QUERIES_PER_CLASS: usize = 2_000;
const MIN_DISTINCT_SUBJECTS_PER_CLASS: usize = 2_000;
const MIN_CANDIDATES: usize = 10;

#[derive(Clone, Debug)]
struct TruthRow {
    query_id: String,
    query_subject: String,
    candidate_id: String,
    candidate_subject: String,
    true_match: bool,
    stratum: String,
}

#[derive(Clone, Debug)]
struct Candidate {
    candidate_id: String,
    candidate_subject: String,
    score: f64,
    true_match: bool,
}

#[derive(Clone, Debug)]
struct QueryResult {
    query_id: String,
    query_subject: String,
    stratum: String,
    matched: bool,
    max_score: f64,
    target_score: Option<f64>,
    target_rank_low: Option<usize>,
    target_rank_high: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
struct Rates {
    mated_queries: usize,
    nonmated_queries: usize,
    false_identifications: usize,
    false_negative_rank1: f64,
    false_negative_rank3: f64,
    false_negative_rank5: f64,
    false_negative_rank10: f64,
    false_positive_identification_rate: f64,
    correct_identification_precision: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
struct SubjectRisk {
    subjects: usize,
    subjects_with_any_false_identification: usize,
    empirical_rate: f64,
    upper_bound_95: f64,
}

#[derive(Clone, Copy, Debug)]
struct Interval {
    low: f64,
    high: f64,
    valid_replicates: usize,
}

struct XorShift64(u64);
impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

pub fn run(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.get(2).is_some_and(|arg| arg == "--help" || arg == "-h") {
        println!(
            "Usage: {} open-set <validation-truth.tsv> <validation-scores.tsv> <test-truth.tsv> <test-scores.tsv> <training-subjects.txt> <report.json> [max_fpir=0.01] [bootstrap-replicates=1000] [seed=9901]",
            args.first().map_or("life-privacy-eval", String::as_str)
        );
        return Ok(());
    }
    if args.len() < 8 || args.len() > 11 {
        return Err("open-set usage: open-set <validation-truth.tsv> <validation-scores.tsv> <test-truth.tsv> <test-scores.tsv> <training-subjects.txt> <report.json> [max_fpir=0.01] [bootstrap-replicates=1000] [seed=9901]".into());
    }

    let validation_truth = read_truth(&args[2])?;
    let validation_scores = read_scores(&args[3])?;
    let test_truth = read_truth(&args[4])?;
    let test_scores = read_scores(&args[5])?;
    let training_subjects = read_training_subjects(&args[6])?;
    let report_path = &args[7];
    let max_fpir = args.get(8).map_or(Ok(0.01_f64), |s| s.parse::<f64>())?;
    let bootstraps = args.get(9).map_or(Ok(DEFAULT_BOOTSTRAPS), |s| s.parse::<usize>())?;
    let seed = args.get(10).map_or(Ok(DEFAULT_SEED), |s| s.parse::<u64>())?;

    if !max_fpir.is_finite() || !(0.0..=1.0).contains(&max_fpir) {
        return Err("max_fpir must be finite and in [0, 1]".into());
    }
    if bootstraps < 100 {
        return Err("at least 100 query-subject bootstrap replicates are required".into());
    }

    let (validation, validation_pool) = validate_partition(validation_truth, validation_scores, &training_subjects)?;
    let (test, test_pool) = validate_partition(test_truth, test_scores, &training_subjects)?;
    validate_partition_disjoint(&validation, &test)?;
    if validation_pool != test_pool {
        return Err("validation and test must use exactly the same candidate handles and subject mappings".into());
    }
    let candidate_count = validation_pool.len();
    if candidate_count == 0 {
        return Err("open-set evaluation requires a non-empty common candidate pool".into());
    }

    let threshold = select_threshold(&validation, max_fpir);
    let validation_rates = compute_rates(&validation, threshold);
    let test_rates = compute_rates(&test, threshold);
    let intervals = bootstrap_rates(&test, threshold, bootstraps, seed);
    let validation_subject_risk = subject_risk(&validation, threshold);
    let test_subject_risk = subject_risk(&test, threshold);
    let sample_size_met = meets_sample_minimums(&validation, &test, candidate_count);
    let report = build_report(
        &validation, &test, threshold, max_fpir, validation_rates, test_rates,
        intervals, bootstraps, seed, sample_size_met, candidate_count,
        validation_subject_risk, test_subject_risk,
    );
    let report = format!("{report}\n");
    if report_path == "-" {
        print!("{report}");
    } else {
        fs::write(report_path, report)?;
    }
    Ok(())
}

fn read_truth(path: &str) -> Result<Vec<TruthRow>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    let header = lines.next().ok_or("open-set truth file is empty")?;
    if header != "query_id\tquery_subject\tcandidate_id\tcandidate_subject\ttrue_match\tstratum" {
        return Err("open-set truth header must be query_id, query_subject, candidate_id, candidate_subject, true_match, stratum".into());
    }
    let mut rows = Vec::new();
    let mut seen = BTreeSet::new();
    let mut query_meta: HashMap<String, (String, String)> = HashMap::new();
    let mut candidate_meta: HashMap<String, String> = HashMap::new();
    for (idx, line) in lines.enumerate() {
        if line.trim().is_empty() { continue; }
        let c: Vec<&str> = line.split('\t').collect();
        if c.len() != 6 || c.iter().any(|v| v.is_empty()) {
            return Err(format!("truth line {} must have exactly 6 non-empty columns", idx + 2).into());
        }
        let true_match = match c[4] {
            "0" => false,
            "1" => true,
            _ => return Err(format!("truth line {} true_match must be 0 or 1", idx + 2).into()),
        };
        let (query_id, query_subject, candidate_id, candidate_subject, stratum) =
            (c[0].to_owned(), c[1].to_owned(), c[2].to_owned(), c[3].to_owned(), c[5].to_owned());
        if !seen.insert((query_id.clone(), candidate_id.clone())) {
            return Err(format!("duplicate query/candidate in truth at line {}", idx + 2).into());
        }
        if true_match != (query_subject == candidate_subject) {
            return Err(format!("true_match disagrees with oracle identities at line {}", idx + 2).into());
        }
        let q = (query_subject.clone(), stratum.clone());
        if query_meta.get(&query_id).is_some_and(|old| old != &q) {
            return Err(format!("query {query_id} changes subject or stratum").into());
        }
        query_meta.insert(query_id.clone(), q);
        if candidate_meta.get(&candidate_id).is_some_and(|old| old != &candidate_subject) {
            return Err(format!("candidate handle {candidate_id} maps to multiple subjects").into());
        }
        candidate_meta.insert(candidate_id.clone(), candidate_subject.clone());
        rows.push(TruthRow { query_id, query_subject, candidate_id, candidate_subject, true_match, stratum });
    }
    if rows.is_empty() { return Err("open-set truth file has no rows".into()); }
    Ok(rows)
}

fn read_scores(path: &str) -> Result<HashMap<(String, String), f64>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    let header = lines.next().ok_or("open-set score file is empty")?;
    if header != "query_id\tcandidate_id\tscore" {
        return Err("open-set scores must contain only query_id, candidate_id, score".into());
    }
    let mut scores = HashMap::new();
    for (idx, line) in lines.enumerate() {
        if line.trim().is_empty() { continue; }
        let c: Vec<&str> = line.split('\t').collect();
        if c.len() != 3 || c.iter().any(|v| v.is_empty()) {
            return Err(format!("score line {} must have exactly 3 non-empty columns", idx + 2).into());
        }
        let score = c[2].parse::<f64>().map_err(|_| format!("invalid score at line {}", idx + 2))?;
        if !score.is_finite() { return Err(format!("non-finite score at line {}", idx + 2).into()); }
        if scores.insert((c[0].to_owned(), c[1].to_owned()), score).is_some() {
            return Err(format!("duplicate query/candidate score at line {}", idx + 2).into());
        }
    }
    if scores.is_empty() { return Err("open-set score file has no rows".into()); }
    Ok(scores)
}

fn read_training_subjects(path: &str) -> Result<HashSet<String>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut subjects = HashSet::new();
    for line in input.lines() {
        let s = line.trim();
        if !s.is_empty() && !subjects.insert(s.to_owned()) {
            return Err(format!("duplicate training subject identifier: {s}").into());
        }
    }
    Ok(subjects)
}

fn validate_partition(
    truth: Vec<TruthRow>,
    mut scores: HashMap<(String, String), f64>,
    training: &HashSet<String>,
) -> Result<(Vec<QueryResult>, BTreeSet<(String, String)>), Box<dyn Error>> {
    let expected = truth.len();
    let mut grouped: BTreeMap<String, Vec<Candidate>> = BTreeMap::new();
    let mut query_meta: BTreeMap<String, (String, String)> = BTreeMap::new();
    let mut common_candidate_pool: Option<BTreeSet<(String, String)>> = None;

    for row in truth {
        if training.contains(&row.query_subject) {
            return Err(format!("training/query subject overlap for {}", row.query_subject).into());
        }
        if training.contains(&row.candidate_subject) {
            return Err(format!("training/candidate subject overlap for {}", row.candidate_subject).into());
        }
        let key = (row.query_id.clone(), row.candidate_id.clone());
        let score = scores.remove(&key).ok_or_else(|| format!("missing score for {}/{}", row.query_id, row.candidate_id))?;
        let meta = (row.query_subject.clone(), row.stratum.clone());
        if query_meta.get(&row.query_id).is_some_and(|old| old != &meta) {
            return Err(format!("query {} changes subject or stratum", row.query_id).into());
        }
        query_meta.insert(row.query_id.clone(), meta);
        grouped.entry(row.query_id).or_default().push(Candidate {
            candidate_id: row.candidate_id,
            candidate_subject: row.candidate_subject,
            score,
            true_match: row.true_match,
        });
    }
    if !scores.is_empty() { return Err(format!("score file has {} extra rows", scores.len()).into()); }
    if expected == 0 || grouped.len() < 2 { return Err("open-set partition requires at least two queries".into()); }

    let mut output = Vec::with_capacity(grouped.len());
    for (query_id, candidates) in grouped {
        let (query_subject, stratum) = query_meta.remove(&query_id).ok_or("missing query metadata")?;
        let candidate_pool: BTreeSet<(String, String)> = candidates.iter()
            .map(|c| (c.candidate_id.clone(), c.candidate_subject.clone())).collect();
        if candidate_pool.len() != candidates.len() {
            return Err(format!("query {query_id} has duplicate candidate IDs or subjects").into());
        }
        if common_candidate_pool.as_ref().is_some_and(|pool| pool != &candidate_pool) {
            return Err("all queries must use exactly the same candidate handles and subject mappings".into());
        }
        common_candidate_pool.get_or_insert(candidate_pool);
        let mated: Vec<&Candidate> = candidates.iter().filter(|c| c.true_match).collect();
        if mated.len() > 1 { return Err(format!("query {query_id} has multiple true candidates").into()); }
        if mated.is_empty() && candidates.iter().any(|c| c.candidate_subject == query_subject) {
            return Err(format!("query {query_id} has no true_match label but its subject is in the candidate pool").into());
        }
        if mated.len() == 1 && !candidates.iter().any(|c| c.candidate_subject == query_subject) {
            return Err(format!("query {query_id} claims a match but query subject is absent from candidate pool").into());
        }
        let max_score = candidates.iter().map(|c| c.score).max_by(f64::total_cmp).ok_or("empty candidate set")?;
        let target = mated.first().copied();
        let (target_score, target_rank_low, target_rank_high) = if let Some(target) = target {
            let greater = candidates.iter().filter(|c| c.score > target.score).count();
            let tied = candidates.iter().filter(|c| c.score == target.score).count();
            (Some(target.score), Some(greater + 1), Some(greater + tied))
        } else { (None, None, None) };
        output.push(QueryResult {
            query_id, query_subject, stratum, matched: target.is_some(), max_score,
            target_score, target_rank_low, target_rank_high,
        });
    }
    Ok((output, common_candidate_pool.ok_or("missing candidate pool")?))
}

fn validate_partition_disjoint(validation: &[QueryResult], test: &[QueryResult]) -> Result<(), Box<dyn Error>> {
    let val_ids: BTreeSet<&str> = validation.iter().map(|q| q.query_id.as_str()).collect();
    let test_ids: BTreeSet<&str> = test.iter().map(|q| q.query_id.as_str()).collect();
    if let Some(id) = val_ids.intersection(&test_ids).next() {
        return Err(format!("validation/test query handle overlap: {id}").into());
    }
    let val_subjects: BTreeSet<&str> = validation.iter().map(|q| q.query_subject.as_str()).collect();
    let test_subjects: BTreeSet<&str> = test.iter().map(|q| q.query_subject.as_str()).collect();
    if let Some(subject) = val_subjects.intersection(&test_subjects).next() {
        return Err(format!("validation/test query subject overlap: {subject}").into());
    }
    let val_matched = validation.iter().any(|q| q.matched);
    let val_unknown = validation.iter().any(|q| !q.matched);
    let test_matched = test.iter().any(|q| q.matched);
    let test_unknown = test.iter().any(|q| !q.matched);
    if !(val_matched && val_unknown && test_matched && test_unknown) {
        return Err("validation and test must each include both mated and non-mated queries".into());
    }
    Ok(())
}

fn distinct_subjects_by_class(rows: &[QueryResult], matched: bool) -> usize {
    rows.iter().filter(|q| q.matched == matched).map(|q| q.query_subject.as_str())
        .collect::<BTreeSet<_>>().len()
}

fn meets_sample_minimums(validation: &[QueryResult], test: &[QueryResult], candidate_count: usize) -> bool {
    fn class_counts(rows: &[QueryResult]) -> (usize, usize) {
        (rows.iter().filter(|q| q.matched).count(), rows.iter().filter(|q| !q.matched).count())
    }
    let (val_mated, val_unknown) = class_counts(validation);
    let (test_mated, test_unknown) = class_counts(test);
    val_mated >= MIN_QUERIES_PER_CLASS
        && val_unknown >= MIN_QUERIES_PER_CLASS
        && test_mated >= MIN_QUERIES_PER_CLASS
        && test_unknown >= MIN_QUERIES_PER_CLASS
        && distinct_subjects_by_class(validation, true) >= MIN_DISTINCT_SUBJECTS_PER_CLASS
        && distinct_subjects_by_class(validation, false) >= MIN_DISTINCT_SUBJECTS_PER_CLASS
        && distinct_subjects_by_class(test, true) >= MIN_DISTINCT_SUBJECTS_PER_CLASS
        && distinct_subjects_by_class(test, false) >= MIN_DISTINCT_SUBJECTS_PER_CLASS
        && candidate_count >= MIN_CANDIDATES
}


fn select_threshold(rows: &[QueryResult], max_fpir: f64) -> Option<f64> {
    let mut candidates = Vec::new();
    for row in rows {
        if let Some(score) = row.target_score { candidates.push(score); }
        // Include each query's maximum; these determine false-positive identification for non-mated probes.
        candidates.push(row.max_score);
    }
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|left, right| left.total_cmp(right).is_eq());
    let mut best: Option<f64> = None;
    let mut best_hit_rate = 0.0;
    let mut best_fpir = f64::INFINITY;
    for threshold in candidates.into_iter().rev() {
        let rates = compute_rates(rows, Some(threshold));
        let hit_rate = 1.0 - rates.false_negative_rank1;
        if rates.false_positive_identification_rate <= max_fpir + f64::EPSILON
            && (hit_rate > best_hit_rate || (hit_rate == best_hit_rate && rates.false_positive_identification_rate < best_fpir)) {
            best = Some(threshold);
            best_hit_rate = hit_rate;
            best_fpir = rates.false_positive_identification_rate;
        }
    }
    best
}

fn compute_rates(rows: &[QueryResult], threshold: Option<f64>) -> Rates {
    let mated: Vec<&QueryResult> = rows.iter().filter(|q| q.matched).collect();
    let nonmated: Vec<&QueryResult> = rows.iter().filter(|q| !q.matched).collect();
    let success_at = |q: &QueryResult, k: usize| -> f64 {
        let Some(score) = q.target_score else { return 0.0; };
        if !threshold.is_some_and(|t| score >= t) { return 0.0; }
        let (Some(low), Some(high)) = (q.target_rank_low, q.target_rank_high) else { return 0.0; };
        if low > k { 0.0 } else { (k.min(high) - low + 1) as f64 / (high - low + 1) as f64 }
    };
    let denominator = mated.len().max(1) as f64;
    let false_negative_rank1 = 1.0 - mated.iter().map(|q| success_at(q, 1)).sum::<f64>() / denominator;
    let false_negative_rank3 = 1.0 - mated.iter().map(|q| success_at(q, 3)).sum::<f64>() / denominator;
    let false_negative_rank5 = 1.0 - mated.iter().map(|q| success_at(q, 5)).sum::<f64>() / denominator;
    let false_negative_rank10 = 1.0 - mated.iter().map(|q| success_at(q, 10)).sum::<f64>() / denominator;
    let false_identifications = nonmated.iter().filter(|q| threshold.is_some_and(|t| q.max_score >= t)).count();
    let false_positive_identification_rate = if nonmated.is_empty() { 0.0 } else {
        false_identifications as f64 / nonmated.len() as f64
    };
    let accepted = rows.iter().filter(|q| threshold.is_some_and(|t| q.max_score >= t)).count();
    let correct_rank1 = mated.iter().map(|q| success_at(q, 1)).sum::<f64>();
    let correct_identification_precision = if accepted == 0 { None } else { Some(correct_rank1 / accepted as f64) };
    Rates {
        mated_queries: mated.len(),
        nonmated_queries: nonmated.len(),
        false_identifications,
        false_negative_rank1, false_negative_rank3, false_negative_rank5, false_negative_rank10,
        false_positive_identification_rate, correct_identification_precision,
    }
}

fn bootstrap_rates(rows: &[QueryResult], threshold: Option<f64>, replicates: usize, seed: u64) -> (Interval, Interval, Interval) {
    let mated: Vec<&QueryResult> = rows.iter().filter(|q| q.matched).collect();
    let nonmated: Vec<&QueryResult> = rows.iter().filter(|q| !q.matched).collect();
    let by_mated_subject: BTreeMap<&str, Vec<&QueryResult>> = group_subjects(&mated);
    let by_nonmated_subject: BTreeMap<&str, Vec<&QueryResult>> = group_subjects(&nonmated);
    let mated_subjects: Vec<&str> = by_mated_subject.keys().copied().collect();
    let nonmated_subjects: Vec<&str> = by_nonmated_subject.keys().copied().collect();
    let mut rng = XorShift64::new(seed);
    let mut fnir1 = Vec::with_capacity(replicates);
    let mut fnir5 = Vec::with_capacity(replicates);
    let mut fpir = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut sampled_mated = Vec::new();
        let mut sampled_unknown = Vec::new();
        for _ in 0..mated_subjects.len() {
            let subject = mated_subjects[(rng.next() % mated_subjects.len() as u64) as usize];
            sampled_mated.extend(by_mated_subject[subject].iter().copied());
        }
        for _ in 0..nonmated_subjects.len() {
            let subject = nonmated_subjects[(rng.next() % nonmated_subjects.len() as u64) as usize];
            sampled_unknown.extend(by_nonmated_subject[subject].iter().copied());
        }
        let rates = compute_rates_from_samples(&sampled_mated, &sampled_unknown, threshold);
        fnir1.push(rates.0);
        fnir5.push(rates.1);
        fpir.push(rates.2);
    }
    (percentile_interval(fnir1), percentile_interval(fnir5), percentile_interval(fpir))
}

fn group_subjects<'a>(rows: &[&'a QueryResult]) -> BTreeMap<&'a str, Vec<&'a QueryResult>> {
    let mut groups = BTreeMap::new();
    for row in rows { groups.entry(row.query_subject.as_str()).or_insert_with(Vec::new).push(*row); }
    groups
}

fn compute_rates_from_samples(mated: &[&QueryResult], unknown: &[&QueryResult], threshold: Option<f64>) -> (f64, f64, f64) {
    let success = |q: &QueryResult, k: usize| -> f64 {
        let Some(score) = q.target_score else { return 0.0; };
        if !threshold.is_some_and(|t| score >= t) { return 0.0; }
        let (Some(low), Some(high)) = (q.target_rank_low, q.target_rank_high) else { return 0.0; };
        if low > k { 0.0 } else { (k.min(high) - low + 1) as f64 / (high - low + 1) as f64 }
    };
    let mated_den = mated.len().max(1) as f64;
    let unknown_den = unknown.len().max(1) as f64;
    let fn1 = 1.0 - mated.iter().map(|q| success(q, 1)).sum::<f64>() / mated_den;
    let fn5 = 1.0 - mated.iter().map(|q| success(q, 5)).sum::<f64>() / mated_den;
    let fp = unknown.iter().filter(|q| threshold.is_some_and(|t| q.max_score >= t)).count() as f64 / unknown_den;
    (fn1, fn5, fp)
}

fn percentile_interval(mut values: Vec<f64>) -> Interval {
    if values.is_empty() { return Interval { low: 0.0, high: 1.0, valid_replicates: 0 }; }
    values.sort_by(f64::total_cmp);
    let quantile = |p: f64| {
        let position = (values.len() - 1) as f64 * p;
        let low = position.floor() as usize;
        let high = position.ceil() as usize;
        values[low] + (values[high] - values[low]) * (position - low as f64)
    };
    Interval { low: quantile(0.025), high: quantile(0.975), valid_replicates: values.len() }
}

fn binomial_cdf_at_most(events: usize, trials: usize, probability: f64) -> f64 {
    if events >= trials || probability <= 0.0 {
        return 1.0;
    }
    if probability >= 1.0 {
        return 0.0;
    }

    // Log-PMF recurrence avoids underflow in the first probability term.
    // The log-sum-exp accumulator keeps the partial CDF numerically stable.
    let log_p = probability.ln();
    let log_q = (-probability).ln_1p();
    let mut log_mass = trials as f64 * log_q;
    let mut max_log_mass = f64::NEG_INFINITY;
    let mut scaled_sum = 0.0;
    for i in 0..=events {
        if log_mass > max_log_mass {
            scaled_sum = if max_log_mass.is_finite() {
                scaled_sum * (max_log_mass - log_mass).exp() + 1.0
            } else {
                1.0
            };
            max_log_mass = log_mass;
        } else {
            scaled_sum += (log_mass - max_log_mass).exp();
        }
        if i < events {
            log_mass += ((trials - i) as f64).ln()
                - ((i + 1) as f64).ln()
                + log_p
                - log_q;
        }
    }
    (max_log_mass.exp() * scaled_sum).clamp(0.0, 1.0)
}

fn exact_binomial_upper_bound_95(events: usize, independent_clusters: usize) -> f64 {
    if independent_clusters == 0 || events >= independent_clusters {
        return 1.0;
    }
    if events == 0 {
        // Stable exact one-sided Clopper-Pearson endpoint for zero observed events.
        return -((SUBJECT_RISK_ALPHA.ln() / independent_clusters as f64).exp_m1());
    }

    // Clopper-Pearson upper endpoint: choose the smallest p whose lower
    // binomial tail P[X <= events] is no greater than alpha.
    let mut low = events as f64 / independent_clusters as f64;
    let mut high = 1.0;
    for _ in 0..80 {
        let mid = low + (high - low) * 0.5;
        if binomial_cdf_at_most(events, independent_clusters, mid) > SUBJECT_RISK_ALPHA {
            low = mid;
        } else {
            high = mid;
        }
    }
    high
}

fn subject_risk(rows: &[QueryResult], threshold: Option<f64>) -> SubjectRisk {
    // One unknown subject is one Bernoulli unit. Repeated sessions count as a
    // single event if any session falsely identifies a candidate.
    let mut by_subject: BTreeMap<&str, bool> = BTreeMap::new();
    for query in rows.iter().filter(|query| !query.matched) {
        let crossed = threshold.is_some_and(|value| query.max_score >= value);
        let event = by_subject.entry(query.query_subject.as_str()).or_insert(false);
        *event |= crossed;
    }
    let subjects = by_subject.len();
    let subjects_with_any_false_identification =
        by_subject.values().filter(|&&event| event).count();
    let empirical_rate = if subjects == 0 {
        0.0
    } else {
        subjects_with_any_false_identification as f64 / subjects as f64
    };
    SubjectRisk {
        subjects,
        subjects_with_any_false_identification,
        empirical_rate,
        upper_bound_95: exact_binomial_upper_bound_95(
            subjects_with_any_false_identification,
            subjects,
        ),
    }
}

fn required_subject_clusters_for_zero_event_bound(target: f64) -> Option<usize> {
    if !target.is_finite() || target <= 0.0 || target > 1.0 {
        return None;
    }
    if target >= 1.0 {
        return Some(1);
    }
    let required = SUBJECT_RISK_ALPHA.ln() / (-target).ln_1p();
    if !required.is_finite() || required > usize::MAX as f64 {
        None
    } else {
        Some(required.ceil().max(1.0) as usize)
    }
}

fn operating_point_gate(
    sample_size_met: bool,
    fpir_interval: Interval,
    subject_fp_upper95: f64,
    max_fpir: f64,
) -> &'static str {
    if !sample_size_met {
        "NOT_EVALUATED_BELOW_SAMPLE_MINIMUM"
    } else if fpir_interval.low.is_finite() && fpir_interval.low > max_fpir {
        "FAIL"
    } else if !subject_fp_upper95.is_finite()
        || subject_fp_upper95 > max_fpir + f64::EPSILON
    {
        "INCONCLUSIVE_SUBJECT_LEVEL_UPPER_BOUND_EXCEEDS_TARGET"
    } else if fpir_interval.high.is_finite() && fpir_interval.high <= max_fpir {
        "PROVISIONAL_PASS_REQUIRES_REVIEW"
    } else {
        "INCONCLUSIVE"
    }
}

fn build_report(
    validation: &[QueryResult], test: &[QueryResult], threshold: Option<f64>, max_fpir: f64,
    validation_rates: Rates, test_rates: Rates, intervals: (Interval, Interval, Interval),
    bootstraps: usize, seed: u64, sample_size_met: bool, candidate_count: usize,
    validation_subject_risk: SubjectRisk, test_subject_risk: SubjectRisk,
) -> String {
    let sample_gate = if sample_size_met { "MINIMUM_COUNTS_MET_REQUIRES_REVIEW" } else { "SMOKE_ONLY_BELOW_MINIMUM" };
    let operating_gate = operating_point_gate(
        sample_size_met, intervals.2, test_subject_risk.upper_bound_95, max_fpir
    );
    let val_threshold = threshold.map_or("null".to_owned(), |x| format!("{x:.8}"));
    let target_res = if test_rates.nonmated_queries == 0 { 0.0 } else { 1.0 / test_rates.nonmated_queries as f64 };
    let rare_event = if test_subject_risk.upper_bound_95 <= max_fpir {
        "SUBJECT_CLUSTER_UPPER_BOUND_WITHIN_TARGET_NO_FORMAL_PRIVACY_GUARANTEE"
    } else {
        "SUBJECT_CLUSTER_UPPER_BOUND_EXCEEDS_TARGET"
    };
    let required_clusters = required_subject_clusters_for_zero_event_bound(max_fpir)
        .map_or("null".to_owned(), |value| value.to_string());
    let val_target_pool = candidate_count;

    let mut by_stratum: BTreeMap<&str, Vec<&QueryResult>> = BTreeMap::new();
    for q in test { by_stratum.entry(q.stratum.as_str()).or_default().push(q); }
    let strata_json = by_stratum.iter().map(|(stratum, rows)| {
        let threshold_metrics = compute_rates(rows, threshold);
        let fnir1 = if threshold_metrics.mated_queries == 0 { "null".to_owned() } else { format!("{:.8}", threshold_metrics.false_negative_rank1) };
        let fnir5 = if threshold_metrics.mated_queries == 0 { "null".to_owned() } else { format!("{:.8}", threshold_metrics.false_negative_rank5) };
        let fpir = if threshold_metrics.nonmated_queries == 0 { "null".to_owned() } else { format!("{:.8}", threshold_metrics.false_positive_identification_rate) };
        format!(
            r#"{{"stratum":"{}","query_count":{},"mated_queries":{},"nonmated_queries":{},"fnir_rank1":{},"fnir_rank5":{},"fpir":{}}}"#,
            json_escape(stratum), rows.len(), threshold_metrics.mated_queries, threshold_metrics.nonmated_queries,
            fnir1, fnir5, fpir
        )
    }).collect::<Vec<_>>().join(",");
    format!(
        r#"{{
  "schema_version":"1.1",
  "evaluator_version":"{}",
  "evaluation_mode":"validation_locked_open_set_identification",
  "threshold_selection":"validation_only_maximize_rank1_identification_subject_to_fpir_cap",
  "threshold_source":"validation_only_locked_before_test",
  "threshold":{},
  "seed":{},
  "requested_bootstrap_replicates":{},
  "valid_bootstrap_replicates":{},
  "max_fpir_target":{:.8},
  "validation_query_count":{},
  "validation_mated_queries":{},
  "validation_nonmated_queries":{},
  "validation_false_identifications":{},
  "validation_fnir_rank1":{:.8},
  "validation_fpir":{:.8},
  "test_query_count":{},
  "test_mated_queries":{},
  "test_nonmated_queries":{},
  "candidate_pool_size":{},
  "uniform_random_top1_baseline":{:.8},
  "false_negative_identification_rate_rank1":{:.8},
  "false_negative_identification_rate_rank3":{:.8},
  "false_negative_identification_rate_rank5":{:.8},
  "false_negative_identification_rate_rank10":{:.8},
  "false_identifications":{},
  "expected_false_negative_identifications_rank1":{:.8},
  "false_positive_identification_rate":{:.8},
  "fpir_resolution":{:.8},
  "correct_identification_precision":{},
  "fnir_rank1_cluster_bootstrap_ci95":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}},
  "fnir_rank5_cluster_bootstrap_ci95":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}},
  "fpir_cluster_bootstrap_ci95":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}},
  "subject_fp_rate_bound_method":"one_sided_exact_clopper_pearson_binomial_upper_bound_on_subject_level_any_session_false_identification",
  "subject_fp_rate_confidence_level":0.95000000,
  "subject_any_false_identification_target":{:.8},
  "validation_nonmated_subject_clusters":{},
  "validation_subjects_with_any_false_identification":{},
  "validation_subject_any_false_identification_rate":{:.8},
  "validation_subject_any_false_identification_upper95":{:.8},
  "test_nonmated_subject_clusters":{},
  "test_subjects_with_any_false_identification":{},
  "test_subject_any_false_identification_rate":{:.8},
  "test_subject_any_false_identification_upper95":{:.8},
  "nonmated_subject_clusters_required_for_zero_event_bound":{},
  "rare_event_inference_status":"{}",
  "sample_size_gate":"{}",
  "operating_point_gate":"{}",
  "task_utility_evidence_present":false,
  "official_conformance_evidence_present":false,
  "overall_release_gate":"NOT_EVALUATED_OPEN_SET_UTILITY_AND_CONFORMANCE_MISSING",
  "by_stratum":[{}],
  "interpretation":"Threshold is selected only on validation queries and then frozen on test. FPIR is the fraction of non-mated query subjects for which any candidate score crosses the threshold; FNIR at rank K is the fraction of mated queries whose true identity is not accepted within top K. Tie credit is fractional. Bootstrap resamples query-subject clusters separately by mated/non-mated class and is conditional on the fixed candidate pool. The one-sided 95% Hoeffding bound is computed on per-unknown-subject any-session false-identification events, conditional on a frozen scorer and fixed candidate pool. It assumes independent subject clusters, does not cover population shift or gallery-sampling uncertainty, and is not a formal privacy guarantee."
}}"#,
        VERSION, val_threshold, seed, bootstraps, intervals.0.valid_replicates.min(intervals.1.valid_replicates).min(intervals.2.valid_replicates),
        max_fpir, validation.len(), validation_rates.mated_queries, validation_rates.nonmated_queries,
        validation_rates.false_identifications, validation_rates.false_negative_rank1, validation_rates.false_positive_identification_rate,
        test.len(), test_rates.mated_queries, test_rates.nonmated_queries, val_target_pool,
        1.0 / candidate_count.max(1) as f64,
        test_rates.false_negative_rank1, test_rates.false_negative_rank3, test_rates.false_negative_rank5,
        test_rates.false_negative_rank10, test_rates.false_identifications,
        test_rates.false_negative_rank1 * test_rates.mated_queries as f64,
        test_rates.false_positive_identification_rate, target_res,
        metric_or_null(test_rates.correct_identification_precision),
        intervals.0.low, intervals.0.high, intervals.0.valid_replicates,
        intervals.1.low, intervals.1.high, intervals.1.valid_replicates,
        intervals.2.low, intervals.2.high, intervals.2.valid_replicates,
        max_fpir,
        validation_subject_risk.subjects,
        validation_subject_risk.subjects_with_any_false_identification,
        validation_subject_risk.empirical_rate,
        validation_subject_risk.upper_bound_95,
        test_subject_risk.subjects,
        test_subject_risk.subjects_with_any_false_identification,
        test_subject_risk.empirical_rate,
        test_subject_risk.upper_bound_95,
        required_clusters,
        rare_event, sample_gate, operating_gate, strata_json
    )
}

fn metric_or_null(value: Option<f64>) -> String {
    value.filter(|x| x.is_finite()).map_or_else(|| "null".to_owned(), |x| format!("{x:.8}"))
}

fn json_escape(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        match character {
            '\\' => { escaped.push('\\'); escaped.push('\\'); }
            '"' => { escaped.push('\\'); escaped.push('"'); }
            '\n' => { escaped.push('\\'); escaped.push('n'); }
            '\r' => { escaped.push('\\'); escaped.push('r'); }
            '\t' => { escaped.push('\\'); escaped.push('t'); }
            '\x08' => escaped.push_str("\\b"),
            '\x0c' => escaped.push_str("\\f"),
            ch if ch.is_control() => escaped.push_str(&format!("\\u{:04x}", ch as u32)),
            ch => escaped.push(ch),
        }
    }
    escaped
}

#[cfg(test)]
mod tests {
    use super::*;

    fn query(id: &str, subject: &str, matched: bool, max_score: f64, target_score: Option<f64>, rank: Option<(usize, usize)>, stratum: &str) -> QueryResult {
        QueryResult {
            query_id:id.into(), query_subject:subject.into(), stratum:stratum.into(), matched, max_score,
            target_score, target_rank_low:rank.map(|r|r.0), target_rank_high:rank.map(|r|r.1),
        }
    }

    #[test]
    fn sample_minimum_requires_both_mated_and_nonmated_cohorts() {
        let mated_only = (0..4_000).map(|i| query(
            &format!("m{i}"), &format!("mated{i}"), true, 0.9, Some(0.9), Some((1, 1)), "mated"
        )).collect::<Vec<_>>();
        let test_mated_only = mated_only.clone();
        assert!(!meets_sample_minimums(&mated_only, &test_mated_only, 20));
    }

    #[test]
    fn threshold_respects_validation_fpir_cap() {
        let rows=vec![
            query("q1","s1",true,0.9,Some(0.9),Some((1,1)),"a"),
            query("q2","s2",true,0.7,Some(0.6),Some((2,2)),"a"),
            query("q3","unknown1",false,0.8,None,None,"unknown"),
            query("q4","unknown2",false,0.2,None,None,"unknown"),
        ];
        let threshold=select_threshold(&rows,0.0);
        assert_eq!(threshold,Some(0.9));
        let rates=compute_rates(&rows,threshold);
        assert_eq!(rates.false_positive_identification_rate,0.0);
        assert_eq!(rates.false_negative_rank1,0.5);
    }

    #[test]
    fn subject_level_bound_controls_provisional_gate() {
        let interval=Interval{low:0.0,high:0.0,valid_replicates:1000};
        let rates=Rates{mated_queries:10,nonmated_queries:10,false_identifications:0,false_negative_rank1:0.5,false_negative_rank3:0.4,false_negative_rank5:0.3,false_negative_rank10:0.0,false_positive_identification_rate:0.0,correct_identification_precision:Some(0.5)};
        let empty_risk = subject_risk(&[], Some(0.9));
        let report = build_report(
            &[],
            &[],
            Some(0.9),
            0.01,
            rates,
            rates,
            (interval, interval, interval),
            1000,
            1,
            false,
            10,
            empty_risk,
            empty_risk,
        );
        assert!(report.contains("SUBJECT_CLUSTER_UPPER_BOUND_EXCEEDS_TARGET"));
        assert_eq!(operating_point_gate(false, interval, 1.0, 0.01), "NOT_EVALUATED_BELOW_SAMPLE_MINIMUM");
        assert_eq!(operating_point_gate(true, interval, 0.03, 0.01), "INCONCLUSIVE_SUBJECT_LEVEL_UPPER_BOUND_EXCEEDS_TARGET");
        assert_eq!(operating_point_gate(true, interval, 0.005, 0.01), "PROVISIONAL_PASS_REQUIRES_REVIEW");
    }

    #[test]
    fn exact_subject_bound_has_known_zero_and_nonzero_event_cases() {
        let at_2_000_for_one_percent = exact_binomial_upper_bound_95(0, 2_000);
        assert!(at_2_000_for_one_percent < 0.01);
        assert!(exact_binomial_upper_bound_95(0, 298) > 0.01);
        assert!(exact_binomial_upper_bound_95(0, 299) <= 0.01);
        assert_eq!(required_subject_clusters_for_zero_event_bound(0.01), Some(299));
        assert_eq!(required_subject_clusters_for_zero_event_bound(0.001), Some(2_995));
        assert_eq!(required_subject_clusters_for_zero_event_bound(0.0), None);
        assert_eq!(required_subject_clusters_for_zero_event_bound(1.0), Some(1));

        let one_event_in_ten = exact_binomial_upper_bound_95(1, 10);
        assert!(one_event_in_ten > 0.39 && one_event_in_ten < 0.40);
        assert!(binomial_cdf_at_most(1, 10, one_event_in_ten) <= SUBJECT_RISK_ALPHA + 1e-12);
    }

    #[test]
    fn repeated_unknown_sessions_count_as_one_subject_level_event() {
        let rows = vec![
            query("u1a", "unknown_a", false, 0.9, None, None, "unknown"),
            query("u1b", "unknown_a", false, 0.1, None, None, "unknown"),
            query("u2a", "unknown_b", false, 0.1, None, None, "unknown"),
        ];
        let risk = subject_risk(&rows, Some(0.5));
        assert_eq!(risk.subjects, 2);
        assert_eq!(risk.subjects_with_any_false_identification, 1);
        assert_eq!(risk.empirical_rate, 0.5);
    }

    #[test]
    fn tied_target_uses_fractional_credit() {
        let q=query("q","s1",true,0.9,Some(0.5),Some((1,2)),"a");
        assert_eq!(compute_rates(&[q],Some(0.5)).false_negative_rank1,0.5);
    }

    #[test]
    fn validation_and_test_query_subject_overlap_fails_closed() {
        let validation = vec![
            query("qv1", "matched_subject", true, 0.9, Some(0.9), Some((1, 1)), "matched"),
            query("qv2", "unknown_val", false, 0.2, None, None, "unknown"),
        ];
        let test = vec![
            query("qt1", "matched_subject", true, 0.8, Some(0.8), Some((1, 1)), "matched"),
            query("qt2", "unknown_test", false, 0.1, None, None, "unknown"),
        ];
        assert!(validate_partition_disjoint(&validation, &test).is_err());
    }

    #[test]
    fn unknown_query_crossing_threshold_counts_as_false_identification() {
        let rows = vec![
            query("q1", "matched", true, 0.9, Some(0.9), Some((1, 1)), "matched"),
            query("q2", "unknown", false, 0.8, None, None, "unknown"),
        ];
        let rates = compute_rates(&rows, Some(0.75));
        assert_eq!(rates.false_positive_identification_rate, 1.0);
        assert_eq!(rates.false_negative_rank1, 0.0);
    }

    #[test]
    fn bootstrap_is_deterministic() {
        let rows=vec![
            query("q1","s1",true,0.9,Some(0.9),Some((1,1)),"a"),
            query("q2","s2",true,0.7,Some(0.5),Some((2,2)),"a"),
            query("q3","u1",false,0.4,None,None,"b"),
            query("q4","u2",false,0.2,None,None,"b"),
        ];
        let a=bootstrap_rates(&rows,Some(0.5),300,9901);
        let b=bootstrap_rates(&rows,Some(0.5),300,9901);
        assert_eq!(a.0.low.to_bits(),b.0.low.to_bits());
        assert_eq!(a.2.high.to_bits(),b.2.high.to_bits());
        assert_eq!(a.0.valid_replicates,300);
    }
}
