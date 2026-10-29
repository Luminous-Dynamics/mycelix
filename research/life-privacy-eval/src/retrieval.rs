use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::error::Error;
use std::fs;

const VERSION: &str = "0.4.0";
const DEFAULT_BOOTSTRAPS: usize = 1_000;
const DEFAULT_SEED: u64 = 9_901;
const MIN_QUERIES_FOR_REVIEW: usize = 2_000;
const MIN_QUERY_SUBJECTS_FOR_REVIEW: usize = 2_000;

#[derive(Clone, Debug)]
struct TruthRow {
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
struct QueryRank {
    query_id: String,
    query_subject: String,
    stratum: String,
    candidate_count: usize,
    rank_low: usize,
    rank_high: usize,
    expected_rank: f64,
    top_1: f64,
    top_3: f64,
    top_5: f64,
    top_10: f64,
    reciprocal_rank: f64,
}

#[derive(Clone, Copy, Debug)]
struct Summary {
    query_count: usize,
    top_1: f64,
    top_3: f64,
    top_5: f64,
    top_10: f64,
    mean_reciprocal_rank: f64,
    mean_expected_rank: f64,
    median_expected_rank: f64,
    p95_expected_rank: f64,
}

#[derive(Clone, Copy, Debug)]
struct Interval {
    low: f64,
    high: f64,
    valid_replicates: usize,
}

pub fn run(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() < 6 || args.len() > 8 {
        eprintln!(
            "Usage: {} retrieval <truth.tsv> <scores.tsv> <training-subjects.txt> <report.json> [bootstrap-replicates=1000] [seed=9901]\n\
             truth.tsv is oracle-only: query_id, query_subject, candidate_id, candidate_subject, true_match, stratum.\n\
             scores.tsv is attacker output only: query_id, candidate_id, score.\n\
             This mode is closed-set retrieval: every query must have exactly one true candidate and the same candidate pool.",
            args.first().map_or("life-privacy-eval", String::as_str)
        );
        return Err("invalid retrieval argument count".into());
    }

    let truth = read_truth(&args[2])?;
    let scores = read_scores(&args[3])?;
    let training_subjects = read_training_subjects(&args[4])?;
    let report_path = &args[5];
    let bootstraps = args.get(6).map_or(Ok(DEFAULT_BOOTSTRAPS), |s| s.parse::<usize>())?;
    let seed = args.get(7).map_or(Ok(DEFAULT_SEED), |s| s.parse::<u64>())?;
    if bootstraps < 100 {
        return Err("at least 100 query-subject bootstrap replicates are required".into());
    }

    let queries = validate_and_rank(truth, scores, &training_subjects)?;
    let report = format!("{}\n", build_report(&queries, bootstraps, seed));
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
    let header = lines.next().ok_or("retrieval truth file is empty")?;
    if header != "query_id\tquery_subject\tcandidate_id\tcandidate_subject\ttrue_match\tstratum" {
        return Err("retrieval truth header must be query_id, query_subject, candidate_id, candidate_subject, true_match, stratum".into());
    }

    let mut rows = Vec::new();
    let mut seen_pairs = BTreeSet::new();
    let mut query_metadata: HashMap<String, (String, String)> = HashMap::new();
    let mut candidate_metadata: HashMap<String, String> = HashMap::new();

    for (index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let columns: Vec<&str> = line.split('\t').collect();
        if columns.len() != 6 {
            return Err(format!("truth line {} must contain exactly 6 TSV columns", index + 2).into());
        }
        if columns.iter().any(|column| column.is_empty()) {
            return Err(format!("truth line {} contains an empty field", index + 2).into());
        }
        let true_match = match columns[4] {
            "0" => false,
            "1" => true,
            _ => return Err(format!("truth line {} true_match must be 0 or 1", index + 2).into()),
        };
        let query_id = columns[0].to_owned();
        let query_subject = columns[1].to_owned();
        let candidate_id = columns[2].to_owned();
        let candidate_subject = columns[3].to_owned();
        let stratum = columns[5].to_owned();

        if !seen_pairs.insert((query_id.clone(), candidate_id.clone())) {
            return Err(format!("duplicate query/candidate handle at line {}", index + 2).into());
        }
        if true_match != (query_subject == candidate_subject) {
            return Err(format!("oracle true_match disagrees with query/candidate subject identity at line {}", index + 2).into());
        }

        let query_value = (query_subject.clone(), stratum.clone());
        if query_metadata.get(&query_id).is_some_and(|previous| previous != &query_value) {
            return Err(format!("query {query_id} changes subject or stratum across candidate rows").into());
        }
        query_metadata.insert(query_id, query_value);

        if candidate_metadata.get(&candidate_id).is_some_and(|previous| previous != &candidate_subject) {
            return Err(format!("candidate handle {candidate_id} maps to multiple candidate subjects").into());
        }
        candidate_metadata.insert(candidate_id.clone(), candidate_subject.clone());

        rows.push(TruthRow {
            query_id: columns[0].to_owned(),
            query_subject,
            candidate_id,
            candidate_subject,
            true_match,
            stratum,
        });
    }

    if rows.is_empty() {
        return Err("retrieval truth file contains no data rows".into());
    }
    Ok(rows)
}

fn read_scores(path: &str) -> Result<HashMap<(String, String), f64>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    let header = lines.next().ok_or("retrieval score file is empty")?;
    if header != "query_id\tcandidate_id\tscore" {
        return Err("retrieval scores must contain exactly query_id, candidate_id, score; do not include ground-truth fields".into());
    }

    let mut scores = HashMap::new();
    for (index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let columns: Vec<&str> = line.split('\t').collect();
        if columns.len() != 3 || columns.iter().any(|column| column.is_empty()) {
            return Err(format!("score line {} must contain exactly 3 non-empty TSV columns", index + 2).into());
        }
        let score = columns[2]
            .parse::<f64>()
            .map_err(|_| format!("invalid score on line {}", index + 2))?;
        if !score.is_finite() {
            return Err(format!("non-finite score on line {}", index + 2).into());
        }
        let key = (columns[0].to_owned(), columns[1].to_owned());
        if scores.insert(key, score).is_some() {
            return Err(format!("duplicate query/candidate score at line {}", index + 2).into());
        }
    }
    if scores.is_empty() {
        return Err("retrieval score file contains no data rows".into());
    }
    Ok(scores)
}

fn read_training_subjects(path: &str) -> Result<HashSet<String>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut subjects = HashSet::new();
    for line in input.lines() {
        let subject = line.trim();
        if !subject.is_empty() && !subjects.insert(subject.to_owned()) {
            return Err(format!("duplicate training subject identifier: {subject}").into());
        }
    }
    Ok(subjects)
}

fn validate_and_rank(
    truth: Vec<TruthRow>,
    mut scores: HashMap<(String, String), f64>,
    training_subjects: &HashSet<String>,
) -> Result<Vec<QueryRank>, Box<dyn Error>> {
    let expected_score_count = truth.len();
    let mut grouped: BTreeMap<String, Vec<Candidate>> = BTreeMap::new();
    let mut query_metadata: BTreeMap<String, (String, String)> = BTreeMap::new();
    let mut candidate_pool: Option<BTreeSet<String>> = None;

    for row in truth {
        if training_subjects.contains(&row.query_subject) {
            return Err(format!("training/test identity overlap for query subject {}", row.query_subject).into());
        }
        if training_subjects.contains(&row.candidate_subject) {
            return Err(format!("training/test identity overlap for candidate subject {}", row.candidate_subject).into());
        }
        let key = (row.query_id.clone(), row.candidate_id.clone());
        let score = scores.remove(&key).ok_or_else(|| format!("missing score for query/candidate {}/{}", row.query_id, row.candidate_id))?;

        let meta = (row.query_subject.clone(), row.stratum.clone());
        if query_metadata.get(&row.query_id).is_some_and(|previous| previous != &meta) {
            return Err(format!("query {} changes subject or stratum", row.query_id).into());
        }
        query_metadata.insert(row.query_id.clone(), meta);

        grouped.entry(row.query_id).or_default().push(Candidate {
            candidate_id: row.candidate_id,
            candidate_subject: row.candidate_subject,
            score,
            true_match: row.true_match,
        });
    }

    if !scores.is_empty() {
        return Err(format!("score file contains {} extra query/candidate rows", scores.len()).into());
    }
    if expected_score_count == 0 || grouped.len() < 2 {
        return Err("closed-set retrieval requires at least two non-empty queries".into());
    }

    let mut query_subjects = BTreeSet::new();
    let mut candidate_pool_ids: Option<BTreeSet<String>> = None;
    let mut results = Vec::with_capacity(grouped.len());

    for (query_id, candidates) in grouped {
        let (query_subject, stratum) = query_metadata.remove(&query_id)
            .ok_or_else(|| format!("missing query metadata for {query_id}"))?;
        query_subjects.insert(query_subject.clone());
        let candidate_ids: BTreeSet<String> = candidates.iter().map(|candidate| candidate.candidate_id.clone()).collect();
        if candidate_pool_ids.as_ref().is_some_and(|pool| pool != &candidate_ids) {
            return Err("every query must use the same closed-set candidate handles".into());
        }
        candidate_pool_ids.get_or_insert(candidate_ids);

        let candidate_subjects: BTreeSet<String> = candidates.iter().map(|candidate| candidate.candidate_subject.clone()).collect();
        if candidate_subjects.len() != candidates.len() {
            return Err(format!("query {query_id} contains duplicate candidate subjects").into());
        }
        if candidate_pool.as_ref().is_some_and(|pool| pool != &candidate_subjects) {
            return Err("candidate handles do not resolve to the same candidate subject pool for every query".into());
        }
        candidate_pool.get_or_insert(candidate_subjects);

        if !candidates.iter().any(|candidate| candidate.candidate_subject == query_subject) {
            return Err(format!("query {query_id} subject is absent from the closed candidate pool").into());
        }
        if candidates.iter().filter(|candidate| candidate.true_match).count() != 1 {
            return Err(format!("query {query_id} must have exactly one true candidate").into());
        }
        results.push(rank_query(query_id, query_subject, stratum, &candidates)?);
    }

    if results.is_empty() {
        return Err("no valid retrieval queries".into());
    }
    Ok(results)
}

fn rank_query(
    _query_id: String,
    query_subject: String,
    stratum: String,
    candidates: &[Candidate],
) -> Result<QueryRank, Box<dyn Error>> {
    let true_candidate = candidates.iter().find(|candidate| candidate.true_match)
        .ok_or("query has no true candidate")?;
    let score = true_candidate.score;
    let greater = candidates.iter().filter(|candidate| candidate.score > score).count();
    let tied = candidates.iter().filter(|candidate| candidate.score == score).count();
    if tied == 0 {
        return Err("true candidate is missing from the candidate score set".into());
    }
    let rank_low = greater + 1;
    let rank_high = greater + tied;
    let expected_rank = (rank_low + rank_high) as f64 / 2.0;
    let top_k = |k: usize| {
        let positions_in_top_k = k.saturating_sub(greater).min(tied);
        positions_in_top_k as f64 / tied as f64
    };
    let reciprocal_rank = (rank_low..=rank_high)
        .map(|rank| 1.0 / rank as f64)
        .sum::<f64>() / tied as f64;

    Ok(QueryRank {
        query_subject,
        stratum,
        candidate_count: candidates.len(),
        rank_low,
        rank_high,
        expected_rank,
        top_1: top_k(1),
        top_3: top_k(3),
        top_5: top_k(5),
        top_10: top_k(10),
        reciprocal_rank,
    })
}

fn summarize(rows: &[&QueryRank]) -> Summary {
    let count = rows.len();
    let denominator = count.max(1) as f64;
    let mut ranks: Vec<f64> = rows.iter().map(|row| row.expected_rank).collect();
    ranks.sort_by(f64::total_cmp);
    let percentile = |p: f64| {
        if ranks.is_empty() { 0.0 } else {
            let index = ((ranks.len() - 1) as f64 * p).floor() as usize;
            ranks[index]
        }
    };
    Summary {
        query_count: count,
        top_1: rows.iter().map(|row| row.top_1).sum::<f64>() / denominator,
        top_3: rows.iter().map(|row| row.top_3).sum::<f64>() / denominator,
        top_5: rows.iter().map(|row| row.top_5).sum::<f64>() / denominator,
        top_10: rows.iter().map(|row| row.top_10).sum::<f64>() / denominator,
        mean_reciprocal_rank: rows.iter().map(|row| row.reciprocal_rank).sum::<f64>() / denominator,
        mean_expected_rank: rows.iter().map(|row| row.expected_rank).sum::<f64>() / denominator,
        median_expected_rank: percentile(0.50),
        p95_expected_rank: percentile(0.95),
    }
}

fn summary_json(summary: Summary) -> String {
    format!(
        r#"{{"query_count":{},"top_1_reidentification_rate":{:.8},"top_3_reidentification_rate":{:.8},"top_5_reidentification_rate":{:.8},"top_10_reidentification_rate":{:.8},"mean_reciprocal_rank":{:.8},"mean_expected_rank":{:.8},"median_expected_rank":{:.8},"p95_expected_rank":{:.8}}}"#,
        summary.query_count, summary.top_1, summary.top_3, summary.top_5, summary.top_10,
        summary.mean_reciprocal_rank, summary.mean_expected_rank, summary.median_expected_rank, summary.p95_expected_rank
    )
}

fn build_report(rows: &[QueryRank], bootstraps: usize, seed: u64) -> String {
    let query_refs: Vec<&QueryRank> = rows.iter().collect();
    let overall = summarize(&query_refs);
    let candidate_count = rows.first().map_or(0, |row| row.candidate_count);
    let subjects: BTreeSet<&str> = rows.iter().map(|row| row.query_subject.as_str()).collect();
    let sample_size_met = overall.query_count >= MIN_QUERIES_FOR_REVIEW
        && subjects.len() >= MIN_QUERY_SUBJECTS_FOR_REVIEW
        && candidate_count >= 10;
    let sample_size_gate = if sample_size_met {
        "MINIMUM_COUNTS_MET_REQUIRES_REVIEW"
    } else {
        "SMOKE_ONLY_BELOW_MINIMUM"
    };
    let (top1_interval, top5_interval, mrr_interval) = query_subject_cluster_bootstrap(rows, bootstraps, seed);

    let mut strata: BTreeMap<&str, Vec<&QueryRank>> = BTreeMap::new();
    for row in rows {
        strata.entry(row.stratum.as_str()).or_default().push(row);
    }
    let by_stratum = strata.iter().map(|(stratum, stratum_rows)| {
        format!(
            r#"{{"stratum":"{}","metrics":{}}}"#,
            json_escape(stratum),
            summary_json(summarize(stratum_rows))
        )
    }).collect::<Vec<_>>().join(",");

    format!(
        r#"{{
  "schema_version":"1.0",
  "evaluator_version":"{}",
  "evaluation_mode":"closed_set_ranked_reidentification",
  "rank_tie_policy":"uniform_fractional_credit_across_true_candidate_tie_block",
  "uncertainty_scope":"query_subject_cluster_bootstrap_conditioned_on_fixed_candidate_pool",
  "seed":{},
  "requested_bootstrap_replicates":{},
  "valid_bootstrap_replicates":{},
  "query_count":{},
  "unique_query_subjects":{},
  "candidate_pool_size":{},
  "uniform_chance_baseline":{{"top_1":{:.8},"top_3":{:.8},"top_5":{:.8},"top_10":{:.8}}},
  "metrics":{},
  "bootstrap_ci95":{{"top_1":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}},"top_5":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}},"mean_reciprocal_rank":{{"low":{:.8},"high":{:.8},"valid_replicates":{}}}}},
  "by_stratum":[{}],
  "sample_size_gate":"{}",
  "task_utility_evidence_present":false,
  "official_conformance_evidence_present":false,
  "overall_release_gate":"NOT_EVALUATED_RETRIEVAL_ONLY_UTILITY_AND_CONFORMANCE_MISSING",
  "interpretation":"Closed-set retrieval ranking is measured against a fixed candidate pool. Ties receive expected fractional credit rather than deterministic ID-based tie-breaking. Bootstrap intervals resample query-subject clusters and are conditional on the fixed candidate pool; they do not quantify candidate-pool sampling uncertainty. Scores are not interpreted as calibrated posterior probabilities and this report does not estimate an effective anonymity set."
}}"#,
        VERSION, seed, bootstraps,
        top1_interval.valid_replicates.min(top5_interval.valid_replicates).min(mrr_interval.valid_replicates),
        overall.query_count, subjects.len(), candidate_count,
        1.0 / candidate_count.max(1) as f64,
        3.min(candidate_count) as f64 / candidate_count.max(1) as f64,
        5.min(candidate_count) as f64 / candidate_count.max(1) as f64,
        10.min(candidate_count) as f64 / candidate_count.max(1) as f64,
        summary_json(overall),
        top1_interval.low, top1_interval.high, top1_interval.valid_replicates,
        top5_interval.low, top5_interval.high, top5_interval.valid_replicates,
        mrr_interval.low, mrr_interval.high, mrr_interval.valid_replicates,
        by_stratum, sample_size_gate
    )
}

fn query_subject_cluster_bootstrap(
    rows: &[QueryRank],
    replicates: usize,
    seed: u64,
) -> (Interval, Interval, Interval) {
    let subjects: Vec<String> = rows.iter().map(|row| row.query_subject.clone())
        .collect::<BTreeSet<_>>().into_iter().collect();
    let mut by_subject: HashMap<&str, Vec<&QueryRank>> = HashMap::new();
    for row in rows {
        by_subject.entry(row.query_subject.as_str()).or_default().push(row);
    }
    let mut rng = XorShift64::new(seed);
    let mut top1 = Vec::with_capacity(replicates);
    let mut top5 = Vec::with_capacity(replicates);
    let mut mrr = Vec::with_capacity(replicates);
    for _ in 0..replicates {
        let mut multiplicity: HashMap<&str, usize> = HashMap::new();
        for _ in 0..subjects.len() {
            let index = (rng.next() % subjects.len() as u64) as usize;
            *multiplicity.entry(subjects[index].as_str()).or_insert(0) += 1;
        }
        let mut count = 0usize;
        let mut one_sum = 0.0;
        let mut five_sum = 0.0;
        let mut reciprocal_sum = 0.0;
        for (subject, weight) in multiplicity {
            if let Some(subject_rows) = by_subject.get(subject) {
                for row in subject_rows {
                    count += weight;
                    one_sum += row.top_1 * weight as f64;
                    five_sum += row.top_5 * weight as f64;
                    reciprocal_sum += row.reciprocal_rank * weight as f64;
                }
            }
        }
        if count > 0 {
            top1.push(one_sum / count as f64);
            top5.push(five_sum / count as f64);
            mrr.push(reciprocal_sum / count as f64);
        }
    }
    (percentile_interval(top1), percentile_interval(top5), percentile_interval(mrr))
}

fn percentile_interval(mut values: Vec<f64>) -> Interval {
    if values.is_empty() {
        return Interval { low: 0.0, high: 0.0, valid_replicates: 0 };
    }
    values.sort_by(f64::total_cmp);
    let low = ((values.len() - 1) as f64 * 0.025).floor() as usize;
    let high = ((values.len() - 1) as f64 * 0.975).floor() as usize;
    Interval { low: values[low], high: values[high], valid_replicates: values.len() }
}

struct XorShift64(u64);
impl XorShift64 {
    fn new(seed: u64) -> Self { Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed }) }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

fn json_escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(id: &str, subject: &str, score: f64, truth: bool) -> Candidate {
        Candidate {
            candidate_id: id.to_owned(),
            candidate_subject: subject.to_owned(),
            score,
            true_match: truth,
        }
    }

    #[test]
    fn rank_metrics_have_known_answer_for_perfect_ranking() {
        let row = rank_query(
            "q".into(), "s1".into(), "test".into(),
            &[candidate("c1", "s1", 0.9, true), candidate("c2", "s2", 0.3, false),
              candidate("c3", "s3", 0.2, false), candidate("c4", "s4", 0.1, false)],
        ).unwrap();
        assert_eq!(row.rank_low, 1);
        assert_eq!(row.rank_high, 1);
        assert!((row.top_1 - 1.0).abs() < 1e-12);
        assert!((row.top_3 - 1.0).abs() < 1e-12);
        assert!((row.reciprocal_rank - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tied_true_candidate_gets_fractional_top1_credit() {
        let row = rank_query(
            "q".into(), "s1".into(), "test".into(),
            &[candidate("c1", "s1", 0.5, true), candidate("c2", "s2", 0.5, false),
              candidate("c3", "s3", 0.2, false), candidate("c4", "s4", 0.1, false)],
        ).unwrap();
        assert_eq!(row.rank_low, 1);
        assert_eq!(row.rank_high, 2);
        assert!((row.top_1 - 0.5).abs() < 1e-12);
        assert!((row.top_3 - 1.0).abs() < 1e-12);
        assert!((row.reciprocal_rank - 0.75).abs() < 1e-12);
    }

    #[test]
    fn ranking_metrics_detect_a_weak_match() {
        let row = rank_query(
            "q".into(), "s3".into(), "test".into(),
            &[candidate("c1", "s1", 0.9, false), candidate("c2", "s2", 0.6, false),
              candidate("c3", "s3", 0.2, true), candidate("c4", "s4", 0.1, false)],
        ).unwrap();
        assert_eq!(row.rank_low, 3);
        assert_eq!(row.rank_high, 3);
        assert_eq!(row.top_1, 0.0);
        assert_eq!(row.top_3, 1.0);
        assert!((row.reciprocal_rank - 1.0 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn bootstrap_is_deterministic() {
        let rows = vec![
            QueryRank { query_id:"q1".into(), query_subject:"s1".into(), stratum:"a".into(), candidate_count:4, rank_low:1, rank_high:1, expected_rank:1.0, top_1:1.0, top_3:1.0, top_5:1.0, top_10:1.0, reciprocal_rank:1.0 },
            QueryRank { query_id:"q2".into(), query_subject:"s2".into(), stratum:"a".into(), candidate_count:4, rank_low:1, rank_high:2, expected_rank:1.5, top_1:0.5, top_3:1.0, top_5:1.0, top_10:1.0, reciprocal_rank:0.75 },
            QueryRank { query_id:"q3".into(), query_subject:"s3".into(), stratum:"b".into(), candidate_count:4, rank_low:3, rank_high:3, expected_rank:3.0, top_1:0.0, top_3:1.0, top_5:1.0, top_10:1.0, reciprocal_rank:1.0/3.0 },
        ];
        let a = query_subject_cluster_bootstrap(&rows, 300, 9901);
        let b = query_subject_cluster_bootstrap(&rows, 300, 9901);
        assert_eq!(a.0.low.to_bits(), b.0.low.to_bits());
        assert_eq!(a.0.high.to_bits(), b.0.high.to_bits());
        assert_eq!(a.2.low.to_bits(), b.2.low.to_bits());
        assert_eq!(a.0.valid_replicates, 300);
    }

    #[test]
    fn fractional_tie_policy_does_not_depend_on_candidate_identifier_order() {
        let a = rank_query(
            "q".into(), "s1".into(), "test".into(),
            &[candidate("c1", "s1", 0.4, true), candidate("c2", "s2", 0.4, false), candidate("c3", "s3", 0.4, false)],
        ).unwrap();
        let b = rank_query(
            "q".into(), "s1".into(), "test".into(),
            &[candidate("z", "s1", 0.4, true), candidate("a", "s2", 0.4, false), candidate("m", "s3", 0.4, false)],
        ).unwrap();
        assert!((a.top_1 - b.top_1).abs() < 1e-12);
        assert!((a.reciprocal_rank - b.reciprocal_rank).abs() < 1e-12);
    }
}
