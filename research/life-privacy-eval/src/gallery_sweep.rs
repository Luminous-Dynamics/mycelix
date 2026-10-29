use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::error::Error;
use std::fs;

const VERSION: &str = "0.9.0";
const DEFAULT_BOOTSTRAPS: usize = 1_000;
const DEFAULT_SEED: u64 = 9_901;

#[derive(Clone, Debug)]
struct QueryTruth {
    query_id: String,
    query_subject: String,
    stratum: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct CandidateIdentity {
    candidate_id: String,
    candidate_subject: String,
}

#[derive(Clone, Debug)]
struct QueryResult {
    query_subject: String,
    stratum: String,
    draw_id: String,
    gallery_size: usize,
    matched: bool,
    max_score: f64,
    target_score: Option<f64>,
    target_rank_low: Option<usize>,
    target_rank_high: Option<usize>,
}

#[derive(Clone, Copy, Debug, Default)]
struct RateMetrics {
    fnir1: Option<f64>,
    fnir3: Option<f64>,
    fnir5: Option<f64>,
    fnir10: Option<f64>,
    fpir: Option<f64>,
}

#[derive(Clone, Debug, Default)]
struct Interval {
    low: Option<f64>,
    high: Option<f64>,
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

    fn index(&mut self, upper_exclusive: usize) -> usize {
        assert!(upper_exclusive > 0);
        let bound = upper_exclusive as u64;
        let cutoff = u64::MAX - (u64::MAX % bound);
        loop {
            let value = self.next();
            if value < cutoff {
                return (value % bound) as usize;
            }
        }
    }
}

pub fn run(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.get(2).is_some_and(|arg| arg == "--help" || arg == "-h") {
        println!(
            "Usage: {} gallery-sweep <validation-query-truth.tsv> <validation-scores.tsv> <test-query-truth.tsv> <test-scores.tsv> <candidate-frame.tsv> <training-subjects.txt> <report.json> <max_fpir=0.01> <gallery-draws=100> <bootstrap-replicates=1000> <seed=9901> <gallery-sizes=100,1000,10000> [reference-size]",
            args.first().map_or("life-privacy-eval", String::as_str)
        );
        return Ok(());
    }
    if args.len() < 14 || args.len() > 15 {
        return Err("gallery-sweep usage: gallery-sweep <validation-query-truth.tsv> <validation-scores.tsv> <test-query-truth.tsv> <test-scores.tsv> <candidate-frame.tsv> <training-subjects.txt> <report.json> <max_fpir=0.01> <gallery-draws=100> <bootstrap-replicates=1000> <seed=9901> <gallery-sizes=100,1000,10000> [reference-size]".into());
    }

    let validation_truth = read_query_truth(&args[2], "validation")?;
    let validation_scores = read_scores(&args[3])?;
    let test_truth = read_query_truth(&args[4], "test")?;
    let test_scores = read_scores(&args[5])?;
    let mut candidates = read_candidate_frame(&args[6])?;
    let training_subjects = read_training_subjects(&args[7])?;
    let report_path = &args[8];
    let max_fpir = args[9].parse::<f64>()?;
    let draw_count = args[10].parse::<usize>()?;
    let bootstrap_replicates = args[11].parse::<usize>()?;
    let seed = args[12].parse::<u64>()?;
    let sizes = parse_gallery_sizes(&args[13])?;

    if !max_fpir.is_finite() || !(0.0..=1.0).contains(&max_fpir) {
        return Err("max_fpir must be finite and in [0, 1]".into());
    }
    if draw_count < 2 {
        return Err("at least two independent gallery draws are required to estimate between-draw variability".into());
    }
    if bootstrap_replicates < 100 {
        return Err("at least 100 crossed-bootstrap replicates are required".into());
    }
    if sizes.last().copied() != Some(candidates.len()) {
        return Err("the largest declared gallery size must equal the candidate-frame size; otherwise the candidate frame and operational size must be declared separately".into());
    }

    candidates.sort_by(|left, right| left.candidate_id.cmp(&right.candidate_id));
    let candidate_subjects: HashSet<String> =
        candidates.iter().map(|candidate| candidate.candidate_subject.clone()).collect();
    validate_training_disjointness(&validation_truth, &test_truth, &candidate_subjects, &training_subjects)?;
    validate_query_partitions_disjoint(&validation_truth, &test_truth)?;
    validate_score_grid(&validation_truth, &validation_scores, &candidates, "validation")?;
    validate_score_grid(&test_truth, &test_scores, &candidates, "test")?;

    let draws = generate_nested_gallery_draws(&candidates, &sizes, draw_count, seed);
    let mut validation_results = Vec::new();
    let mut test_results = Vec::new();
    for draw in &draws {
        validation_results.extend(build_query_results(
            &validation_truth, &validation_scores, &draw.members, &draw.draw_id, draw.gallery_size,
        )?);
        test_results.extend(build_query_results(
            &test_truth, &test_scores, &draw.members, &draw.draw_id, draw.gallery_size,
        )?);
    }

    let mut validation_by_size: BTreeMap<usize, Vec<QueryResult>> = BTreeMap::new();
    let mut test_by_size: BTreeMap<usize, Vec<QueryResult>> = BTreeMap::new();
    for row in validation_results {
        validation_by_size.entry(row.gallery_size).or_default().push(row);
    }
    for row in test_results {
        test_by_size.entry(row.gallery_size).or_default().push(row);
    }

    let mut thresholds: BTreeMap<usize, Option<f64>> = BTreeMap::new();
    for size in &sizes {
        let validation = validation_by_size.get(size).ok_or("missing validation gallery size")?;
        let test = test_by_size.get(size).ok_or("missing test gallery size")?;
        require_both_query_classes(validation, *size, "validation")?;
        require_both_query_classes(test, *size, "test")?;
        thresholds.insert(*size, select_validation_threshold(validation, max_fpir));
    }
    let reference_size = args.get(14)
        .map(|value| value.parse::<usize>())
        .transpose()?
        .unwrap_or_else(|| *sizes.last().expect("gallery sizes are non-empty"));
    if !sizes.contains(&reference_size) {
        return Err("reference-size must be one of the declared gallery sizes".into());
    }
    let frozen_threshold = thresholds[&reference_size];

    let report = build_report(ReportContext {
        sizes: &sizes,
        draws: &draws,
        validation_by_size: &validation_by_size,
        test_by_size: &test_by_size,
        thresholds: &thresholds,
        reference_size,
        frozen_threshold,
        candidate_count: candidates.len(),
        validation_truth: &validation_truth,
        test_truth: &test_truth,
        max_fpir,
        draw_count,
        bootstrap_replicates,
        seed,
    });
    let report = format!("{report}\n");
    if report_path == "-" {
        print!("{report}");
    } else {
        fs::write(report_path, report)?;
    }
    Ok(())
}

fn read_query_truth(path: &str, partition: &str) -> Result<Vec<QueryTruth>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    let expected = "query_id\tquery_subject\tstratum";
    if lines.next() != Some(expected) {
        return Err(format!("{partition} query truth header must be query_id, query_subject, stratum").into());
    }
    let mut rows = Vec::new();
    let mut seen = HashSet::new();
    let mut query_meta: HashMap<String, (String, String)> = HashMap::new();
    for (index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let columns: Vec<&str> = line.split('\t').collect();
        if columns.len() != 3 || columns.iter().any(|value| value.is_empty()) {
            return Err(format!("{partition} query truth line {} must have exactly three non-empty columns", index + 2).into());
        }
        let row = QueryTruth {
            query_id: columns[0].to_owned(),
            query_subject: columns[1].to_owned(),
            stratum: columns[2].to_owned(),
        };
        if !seen.insert(row.query_id.clone()) {
            return Err(format!("{partition} query truth contains duplicate query ID {}", row.query_id).into());
        }
        query_meta.insert(row.query_id.clone(), (row.query_subject.clone(), row.stratum.clone()));
        rows.push(row);
    }
    if rows.is_empty() {
        return Err(format!("{partition} query truth must not be empty").into());
    }
    if query_meta.len() != rows.len() {
        return Err(format!("{partition} query truth has inconsistent query metadata").into());
    }
    Ok(rows)
}

fn read_candidate_frame(path: &str) -> Result<Vec<CandidateIdentity>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    if lines.next() != Some("candidate_id\tcandidate_subject") {
        return Err("candidate frame header must be candidate_id, candidate_subject".into());
    }
    let mut rows = Vec::new();
    let mut ids = HashSet::new();
    let mut subjects = HashSet::new();
    for (index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let columns: Vec<&str> = line.split('\t').collect();
        if columns.len() != 2 || columns.iter().any(|value| value.is_empty()) {
            return Err(format!("candidate frame line {} must have exactly two non-empty columns", index + 2).into());
        }
        if !ids.insert(columns[0].to_owned()) {
            return Err(format!("duplicate candidate ID {}", columns[0]).into());
        }
        if !subjects.insert(columns[1].to_owned()) {
            return Err(format!("candidate frame contains multiple handles for subject {}; this v0.9 mode permits one identity-level candidate per subject", columns[1]).into());
        }
        rows.push(CandidateIdentity {
            candidate_id: columns[0].to_owned(),
            candidate_subject: columns[1].to_owned(),
        });
    }
    if rows.len() < 2 {
        return Err("candidate frame must contain at least two distinct identities".into());
    }
    Ok(rows)
}

fn read_scores(path: &str) -> Result<HashMap<(String, String), f64>, Box<dyn Error>> {
    let input = fs::read_to_string(path)?;
    let mut lines = input.lines();
    if lines.next() != Some("query_id\tcandidate_id\tscore") {
        return Err("score header must be query_id, candidate_id, score; identity and truth columns are forbidden".into());
    }
    let mut scores = HashMap::new();
    for (index, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let columns: Vec<&str> = line.split('\t').collect();
        if columns.len() != 3 || columns[..2].iter().any(|value| value.is_empty()) {
            return Err(format!("score line {} must have exactly three columns with non-empty handles", index + 2).into());
        }
        let score = columns[2].parse::<f64>()?;
        if !score.is_finite() {
            return Err(format!("score line {} must have a finite score", index + 2).into());
        }
        let key = (columns[0].to_owned(), columns[1].to_owned());
        if scores.insert(key, score).is_some() {
            return Err(format!("duplicate query/candidate score at line {}", index + 2).into());
        }
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

fn validate_training_disjointness(
    validation: &[QueryTruth],
    test: &[QueryTruth],
    candidates: &HashSet<String>,
    training: &HashSet<String>,
) -> Result<(), Box<dyn Error>> {
    for row in validation.iter().chain(test.iter()) {
        if training.contains(&row.query_subject) {
            return Err(format!("training/query subject overlap for {}", row.query_subject).into());
        }
    }
    if let Some(subject) = candidates.intersection(training).next() {
        return Err(format!("training/candidate subject overlap for {subject}").into());
    }
    Ok(())
}

fn validate_query_partitions_disjoint(
    validation: &[QueryTruth],
    test: &[QueryTruth],
) -> Result<(), Box<dyn Error>> {
    let val_ids: BTreeSet<&str> = validation.iter().map(|row| row.query_id.as_str()).collect();
    let test_ids: BTreeSet<&str> = test.iter().map(|row| row.query_id.as_str()).collect();
    if let Some(query_id) = val_ids.intersection(&test_ids).next() {
        return Err(format!("validation/test query handle overlap for {query_id}").into());
    }
    let val_subjects: BTreeSet<&str> = validation.iter().map(|row| row.query_subject.as_str()).collect();
    let test_subjects: BTreeSet<&str> = test.iter().map(|row| row.query_subject.as_str()).collect();
    if let Some(subject) = val_subjects.intersection(&test_subjects).next() {
        return Err(format!("validation/test query subject overlap for {subject}").into());
    }
    Ok(())
}

fn validate_score_grid(
    queries: &[QueryTruth],
    scores: &HashMap<(String, String), f64>,
    candidates: &[CandidateIdentity],
    partition: &str,
) -> Result<(), Box<dyn Error>> {
    let expected_count = queries.len().checked_mul(candidates.len())
        .ok_or("query/candidate score-grid size overflow")?;
    if scores.len() != expected_count {
        return Err(format!(
            "{partition} score grid has {} rows; expected exactly {expected_count} query/candidate pairs",
            scores.len()
        ).into());
    }
    for query in queries {
        for candidate in candidates {
            if !scores.contains_key(&(query.query_id.clone(), candidate.candidate_id.clone())) {
                return Err(format!(
                    "{partition} score grid is missing {}/{}",
                    query.query_id, candidate.candidate_id
                ).into());
            }
        }
    }
    Ok(())
}

fn parse_gallery_sizes(raw: &str) -> Result<Vec<usize>, Box<dyn Error>> {
    let sizes: Vec<usize> = raw.split(',').map(|part| part.trim().parse::<usize>())
        .collect::<Result<Vec<_>, _>>()?;
    if sizes.len() < 2 || sizes.iter().any(|size| *size < 2) {
        return Err("gallery-sizes must contain at least two integer sizes, each >= 2".into());
    }
    if sizes.windows(2).any(|window| window[0] >= window[1]) {
        return Err("gallery-sizes must be strictly increasing with no duplicates".into());
    }
    Ok(sizes)
}

#[derive(Clone, Debug)]
struct GalleryDraw {
    draw_id: String,
    gallery_size: usize,
    members: Vec<CandidateIdentity>,
}

fn generate_nested_gallery_draws(
    candidates: &[CandidateIdentity],
    sizes: &[usize],
    draw_count: usize,
    seed: u64,
) -> Vec<GalleryDraw> {
    let mut output = Vec::with_capacity(draw_count * sizes.len());
    for draw_index in 0..draw_count {
        let mut order = candidates.to_vec();
        let draw_seed = seed ^ (0x9E37_79B9_7F4A_7C15_u64.wrapping_mul(draw_index as u64 + 1));
        let mut rng = XorShift64::new(draw_seed);
        for index in (1..order.len()).rev() {
            let other = rng.index(index + 1);
            order.swap(index, other);
        }
        let draw_id = format!("gallery_draw_{:04}", draw_index + 1);
        for size in sizes {
            output.push(GalleryDraw {
                draw_id: draw_id.clone(),
                gallery_size: *size,
                members: order[..*size].to_vec(),
            });
        }
    }
    output
}

fn build_query_results(
    queries: &[QueryTruth],
    scores: &HashMap<(String, String), f64>,
    members: &[CandidateIdentity],
    draw_id: &str,
    gallery_size: usize,
) -> Result<Vec<QueryResult>, Box<dyn Error>> {
    let mut output = Vec::with_capacity(queries.len());
    for query in queries {
        let mut max_score = f64::NEG_INFINITY;
        let mut target_score = None;
        for candidate in members {
            let score = *scores.get(&(query.query_id.clone(), candidate.candidate_id.clone()))
                .ok_or_else(|| format!("missing score for {}/{} in {draw_id}", query.query_id, candidate.candidate_id))?;
            max_score = max_score.max(score);
            if query.query_subject == candidate.candidate_subject {
                if target_score.replace(score).is_some() {
                    return Err(format!("query {} has multiple target candidates in one gallery", query.query_id).into());
                }
            }
        }
        let matched = target_score.is_some();
        let (target_rank_low, target_rank_high) = if let Some(target) = target_score {
            let greater = members.iter().filter(|candidate| {
                scores[&(query.query_id.clone(), candidate.candidate_id.clone())] > target
            }).count();
            let tied = members.iter().filter(|candidate| {
                scores[&(query.query_id.clone(), candidate.candidate_id.clone())] == target
            }).count();
            (Some(greater + 1), Some(greater + tied))
        } else {
            (None, None)
        };
        output.push(QueryResult {
            query_subject: query.query_subject.clone(),
            stratum: query.stratum.clone(),
            draw_id: draw_id.to_owned(),
            gallery_size,
            matched,
            max_score,
            target_score,
            target_rank_low,
            target_rank_high,
        });
    }
    Ok(output)
}

fn require_both_query_classes(rows: &[QueryResult], size: usize, partition: &str) -> Result<(), Box<dyn Error>> {
    if !rows.iter().any(|row| row.matched) || !rows.iter().any(|row| !row.matched) {
        return Err(format!("{partition} gallery size {size} must contain at least one mated and one non-mated query-draw observation").into());
    }
    Ok(())
}

fn hit_credit(row: &QueryResult, rank_limit: usize, threshold: Option<f64>) -> f64 {
    let Some(target) = row.target_score else { return 0.0; };
    if !threshold.is_some_and(|value| target >= value) {
        return 0.0;
    }
    let (Some(low), Some(high)) = (row.target_rank_low, row.target_rank_high) else { return 0.0; };
    if low > rank_limit {
        0.0
    } else {
        (rank_limit.min(high) - low + 1) as f64 / (high - low + 1) as f64
    }
}

fn weighted_metrics(
    rows: &[QueryResult],
    threshold: Option<f64>,
    subject_multiplicity: Option<&HashMap<String, usize>>,
    draw_multiplicity: Option<&HashMap<String, usize>>,
) -> RateMetrics {
    let mut mated_weight = 0.0;
    let mut nonmated_weight = 0.0;
    let mut false_identification_weight = 0.0;
    let mut hits = [0.0_f64; 4];

    for row in rows {
        let subject_weight = subject_multiplicity
            .map_or(1, |counts| counts.get(&row.query_subject).copied().unwrap_or(0));
        let draw_weight = draw_multiplicity
            .map_or(1, |counts| counts.get(&row.draw_id).copied().unwrap_or(0));
        let weight = (subject_weight as f64) * (draw_weight as f64);
        if weight == 0.0 {
            continue;
        }
        if row.matched {
            mated_weight += weight;
            hits[0] += hit_credit(row, 1, threshold) * weight;
            hits[1] += hit_credit(row, 3, threshold) * weight;
            hits[2] += hit_credit(row, 5, threshold) * weight;
            hits[3] += hit_credit(row, 10, threshold) * weight;
        } else {
            nonmated_weight += weight;
            if threshold.is_some_and(|value| row.max_score >= value) {
                false_identification_weight += weight;
            }
        }
    }

    let fnir = |hit_weight: f64| {
        (mated_weight > 0.0).then_some((1.0 - hit_weight / mated_weight).clamp(0.0, 1.0))
    };
    RateMetrics {
        fnir1: fnir(hits[0]),
        fnir3: fnir(hits[1]),
        fnir5: fnir(hits[2]),
        fnir10: fnir(hits[3]),
        fpir: (nonmated_weight > 0.0).then_some((false_identification_weight / nonmated_weight).clamp(0.0, 1.0)),
    }
}

fn select_validation_threshold(rows: &[QueryResult], max_fpir: f64) -> Option<f64> {
    let mut candidates = Vec::new();
    for row in rows {
        if let Some(target) = row.target_score {
            candidates.push(target);
        }
        candidates.push(row.max_score);
    }
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|left, right| left.total_cmp(right).is_eq());

    let mut best_threshold = None;
    let mut best_hit_rate = -1.0_f64;
    let mut best_fpir = f64::INFINITY;
    for threshold in candidates.into_iter().rev() {
        let metrics = weighted_metrics(rows, Some(threshold), None, None);
        let hit_rate = metrics.fnir1.map_or(0.0, |fnir| 1.0 - fnir);
        let fpir = metrics.fpir.unwrap_or(1.0);
        if fpir <= max_fpir + f64::EPSILON
            && (hit_rate > best_hit_rate || (hit_rate == best_hit_rate && fpir < best_fpir))
        {
            best_threshold = Some(threshold);
            best_hit_rate = hit_rate;
            best_fpir = fpir;
        }
    }
    best_threshold
}

fn metric_json(value: Option<f64>) -> String {
    value.filter(|number| number.is_finite())
        .map_or_else(|| "null".to_owned(), |number| format!("{number:.8}"))
}

fn metrics_json(rows: &[QueryResult], threshold: Option<f64>) -> String {
    let rates = weighted_metrics(rows, threshold, None, None);
    let mated = rows.iter().filter(|row| row.matched).count();
    let nonmated = rows.len() - mated;
    let false_identifications = rows.iter().filter(|row| {
        !row.matched && threshold.is_some_and(|value| row.max_score >= value)
    }).count();
    format!(
        r#"{{"query_draw_observations":{},"mated_query_draw_observations":{},"nonmated_query_draw_observations":{},"false_identifications":{},"fnir_at_rank_1":{},"fnir_at_rank_3":{},"fnir_at_rank_5":{},"fnir_at_rank_10":{},"fpir":{}}}"#,
        rows.len(), mated, nonmated, false_identifications,
        metric_json(rates.fnir1), metric_json(rates.fnir3), metric_json(rates.fnir5),
        metric_json(rates.fnir10), metric_json(rates.fpir)
    )
}

fn stratum_metrics_json(rows: &[QueryResult], threshold: Option<f64>) -> String {
    let strata: BTreeSet<String> = rows.iter().map(|row| row.stratum.clone()).collect();
    let mut reports = Vec::new();
    for stratum in strata {
        let subset: Vec<QueryResult> = rows.iter().filter(|row| row.stratum == stratum).cloned().collect();
        reports.push(format!(
            r#"{{"stratum":"{}","metrics":{}}}"#,
            json_escape(&stratum),
            metrics_json(&subset, threshold)
        ));
    }
    format!("[{}]", reports.join(","))
}

fn percentile(values: &mut [f64], probability: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let position = probability * (values.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        Some(values[lower])
    } else {
        let fraction = position - lower as f64;
        Some(values[lower] * (1.0 - fraction) + values[upper] * fraction)
    }
}

fn interval_from_values(mut values: Vec<f64>) -> Interval {
    let valid_replicates = values.len();
    let low = percentile(&mut values, 0.025);
    let high = percentile(&mut values, 0.975);
    Interval { low, high, valid_replicates }
}

fn interval_json(interval: &Interval) -> String {
    format!(
        r#"{{"low":{},"high":{},"valid_replicates":{}}}"#,
        metric_json(interval.low), metric_json(interval.high), interval.valid_replicates
    )
}

fn crossed_bootstrap_json(rows: &[QueryResult], threshold: Option<f64>, replicates: usize, seed: u64) -> String {
    let subjects: Vec<String> = rows.iter().map(|row| row.query_subject.clone())
        .collect::<BTreeSet<_>>().into_iter().collect();
    let draws: Vec<String> = rows.iter().map(|row| row.draw_id.clone())
        .collect::<BTreeSet<_>>().into_iter().collect();
    if subjects.is_empty() || draws.len() < 2 {
        let empty = Interval::default();
        return format!(
            r#"{{"method":"crossed_query_subject_and_gallery_draw_cluster_percentile_bootstrap","coverage_validation_status":"KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION","fnir_at_rank_1":{},"fnir_at_rank_3":{},"fnir_at_rank_5":{},"fnir_at_rank_10":{},"fpir":{}}}"#,
            interval_json(&empty), interval_json(&empty), interval_json(&empty),
            interval_json(&empty), interval_json(&empty)
        );
    }

    let mut rng = XorShift64::new(seed);
    let mut fnir1_values = Vec::with_capacity(replicates);
    let mut fnir3_values = Vec::with_capacity(replicates);
    let mut fnir5_values = Vec::with_capacity(replicates);
    let mut fnir10_values = Vec::with_capacity(replicates);
    let mut fpir_values = Vec::with_capacity(replicates);

    for _ in 0..replicates {
        let mut subject_counts: HashMap<String, usize> = HashMap::new();
        let mut draw_counts: HashMap<String, usize> = HashMap::new();
        for _ in 0..subjects.len() {
            let index = rng.index(subjects.len());
            *subject_counts.entry(subjects[index].clone()).or_default() += 1;
        }
        for _ in 0..draws.len() {
            let index = rng.index(draws.len());
            *draw_counts.entry(draws[index].clone()).or_default() += 1;
        }
        let rates = weighted_metrics(rows, threshold, Some(&subject_counts), Some(&draw_counts));
        if let Some(value) = rates.fnir1 { fnir1_values.push(value); }
        if let Some(value) = rates.fnir3 { fnir3_values.push(value); }
        if let Some(value) = rates.fnir5 { fnir5_values.push(value); }
        if let Some(value) = rates.fnir10 { fnir10_values.push(value); }
        if let Some(value) = rates.fpir { fpir_values.push(value); }
    }

    format!(
        r#"{{"method":"crossed_query_subject_and_gallery_draw_cluster_percentile_bootstrap","coverage_validation_status":"KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION","fnir_at_rank_1":{},"fnir_at_rank_3":{},"fnir_at_rank_5":{},"fnir_at_rank_10":{},"fpir":{}}}"#,
        interval_json(&interval_from_values(fnir1_values)),
        interval_json(&interval_from_values(fnir3_values)),
        interval_json(&interval_from_values(fnir5_values)),
        interval_json(&interval_from_values(fnir10_values)),
        interval_json(&interval_from_values(fpir_values))
    )
}

fn unique_subject_count(rows: &[QueryTruth]) -> usize {
    rows.iter().map(|row| row.query_subject.as_str()).collect::<BTreeSet<_>>().len()
}

fn draw_results_json(
    rows: &[QueryResult],
    draw_ids: &[String],
    size: usize,
    threshold: Option<f64>,
) -> String {
    let mut draw_json = Vec::new();
    for draw_id in draw_ids {
        let draw_rows: Vec<QueryResult> = rows.iter()
            .filter(|row| row.draw_id == *draw_id && row.gallery_size == size)
            .cloned().collect();
        let mated = draw_rows.iter().filter(|row| row.matched).count();
        let nonmated = draw_rows.len() - mated;
        let rates = weighted_metrics(&draw_rows, threshold, None, None);
        draw_json.push(format!(
            r#"{{"draw_id":"{}","mated_queries":{},"nonmated_queries":{},"fnir_at_rank_1":{},"fnir_at_rank_3":{},"fnir_at_rank_5":{},"fnir_at_rank_10":{},"fpir":{}}}"#,
            json_escape(draw_id), mated, nonmated,
            metric_json(rates.fnir1), metric_json(rates.fnir3), metric_json(rates.fnir5),
            metric_json(rates.fnir10), metric_json(rates.fpir)
        ));
    }
    format!("[{}]", draw_json.join(","))
}

struct ReportContext<'a> {
    sizes: &'a [usize],
    draws: &'a [GalleryDraw],
    validation_by_size: &'a BTreeMap<usize, Vec<QueryResult>>,
    test_by_size: &'a BTreeMap<usize, Vec<QueryResult>>,
    thresholds: &'a BTreeMap<usize, Option<f64>>,
    reference_size: usize,
    frozen_threshold: Option<f64>,
    candidate_count: usize,
    validation_truth: &'a [QueryTruth],
    test_truth: &'a [QueryTruth],
    max_fpir: f64,
    draw_count: usize,
    bootstrap_replicates: usize,
    seed: u64,
}

fn build_report(context: ReportContext<'_>) -> String {
    let ReportContext {
        sizes,
        draws,
        validation_by_size,
        test_by_size,
        thresholds,
        reference_size,
        frozen_threshold,
        candidate_count,
        validation_truth,
        test_truth,
        max_fpir,
        draw_count,
        bootstrap_replicates,
        seed,
    } = context;
    let draw_ids: Vec<String> = draws.iter().map(|draw| draw.draw_id.clone())
        .collect::<BTreeSet<_>>().into_iter().collect();
    let mut size_json = Vec::new();

    for size in sizes {
        let validation = &validation_by_size[size];
        let test = &test_by_size[size];
        let threshold = thresholds[size];
        let validation_metrics = metrics_json(validation, threshold);
        let validation_strata = stratum_metrics_json(validation, threshold);
        let test_metrics = metrics_json(test, threshold);
        let test_strata_locked = stratum_metrics_json(test, threshold);
        let validation_frozen_strata = stratum_metrics_json(validation, frozen_threshold);
        let test_strata_frozen = stratum_metrics_json(test, frozen_threshold);
        let test_intervals = crossed_bootstrap_json(
            test, threshold, bootstrap_replicates,
            seed ^ (*size as u64).rotate_left(13) ^ 0x4753_5745_4550_3031,
        );
        let frozen_test_metrics = metrics_json(test, frozen_threshold);
        let frozen_test_intervals = crossed_bootstrap_json(
            test, frozen_threshold, bootstrap_replicates,
            seed ^ (*size as u64).rotate_left(13) ^ 0x4652_4F5A_454E_3031,
        );
        let val_operating_metrics = weighted_metrics(validation, threshold, None, None);
        let frozen_val_metrics = metrics_json(validation, frozen_threshold);
        let val_observed_fpir = metric_json(val_operating_metrics.fpir);
        let per_draw_locked = draw_results_json(test, &draw_ids, *size, threshold);
        let per_draw_frozen = draw_results_json(test, &draw_ids, *size, frozen_threshold);
        let threshold_text = metric_json(threshold);
        size_json.push(format!(
            r#"{{"gallery_size":{},"validation_query_subjects":{},"test_query_subjects":{},"validation_locked_operating_point":{{"threshold":{},"operating_point_found":{},"validation_metrics":{},"validation_metrics_by_stratum":{},"validation_observed_fpir":{},"test_metrics":{},"test_metrics_by_stratum":{},"test_crossed_bootstrap_ci95":{},"per_gallery_draw_test_metrics":{}}},"frozen_reference_threshold_scaling":{{"reference_gallery_size":{},"threshold":{},"test_metrics":{},"test_metrics_by_stratum":{},"test_crossed_bootstrap_ci95":{},"per_gallery_draw_test_metrics":{}}},"validation_metrics_at_frozen_threshold":{},"validation_metrics_at_frozen_threshold_by_stratum":{}}}"#,
            size, unique_subject_count(validation_truth), unique_subject_count(test_truth),
            threshold_text, threshold.is_some(), validation_metrics, validation_strata, val_observed_fpir,
            test_metrics, test_strata_locked, test_intervals, per_draw_locked,
            reference_size, metric_json(frozen_threshold), frozen_test_metrics, test_strata_frozen,
            frozen_test_intervals, per_draw_frozen, frozen_val_metrics, validation_frozen_strata
        ));
    }

    format!(
        r#"{{"schema_version":"1.0","evaluator_version":"{}","evaluation_mode":"gallery_scaling_open_set_identification","seed":{},"gallery_sampling":{{"method":"deterministic_seeded_uniform_without_replacement_nested_prefix_draws","rng":"xorshift64","draw_count":{},"candidate_frame_size":{},"candidate_score_semantics":"pairwise_scores_are_assumed_gallery_invariant","uncertainty_scope":"conditional_on_candidate_frame; crossed query-subject and gallery-draw resampling","crossed_bootstrap_coverage_validation_status":"KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION"}},"gallery_sizes":{:?},"max_fpir_target":{},"validation_query_count":{},"validation_unique_query_subjects":{},"test_query_count":{},"test_unique_query_subjects":{},"threshold_policy":"validation-locked threshold selected independently per gallery size by maximizing expected rank-1 hit rate subject to empirical validation FPIR cap","frozen_threshold_policy":{{"reference_gallery_size":{},"threshold":{}}},"sample_size_gate":"SMOKE_ONLY_OR_RESEARCH_ONLY_UNTIL_PREREGISTERED_RELEASE_FLOORS_ARE_MET","operating_point_gate":"NOT_A_PRIVACY_PASS; validation FPIR is empirical and test uncertainty coverage study is pending","size_results":[{}],"task_utility_evidence_present":false,"official_conformance_evidence_present":false,"overall_release_gate":"NOT_EVALUATED_CROSSED_GALLERY_EVIDENCE_IS_NOT_RELEASE_CERTIFICATION","limitations":["The candidate frame is fixed; inference does not generalize beyond its sampling frame without a separate transportability argument.","Scores are assumed invariant to candidate-set composition. Context-dependent scoring requires a separate score input for each draw and is not implemented in v0.9.","A finite-grid diagnostic reproduced substantial sparse-event undercoverage in the crossed percentile bootstrap; its intervals must not be used as certification evidence.","The score grid is complete over every query and every candidate in the supplied frame; very large benchmarks may need a streaming implementation.","A threshold selected on validation data is frozen for test evaluation. Test labels never select or retune thresholds.","A reduction in identification rates is not a formal unlinkability or differential-privacy guarantee."]}}"#,
        VERSION, seed, draw_count, candidate_count, sizes, max_fpir,
        validation_truth.len(), unique_subject_count(validation_truth),
        test_truth.len(), unique_subject_count(test_truth),
        reference_size, metric_json(frozen_threshold), size_json.join(",")
    )
}

fn json_escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate_frame(count: usize) -> Vec<CandidateIdentity> {
        (0..count).map(|index| CandidateIdentity {
            candidate_id: format!("candidate_{index:03}"),
            candidate_subject: format!("subject_{index:03}"),
        }).collect()
    }

    fn test_query(id: &str, subject: &str) -> QueryTruth {
        QueryTruth { query_id: id.to_owned(), query_subject: subject.to_owned(), stratum: "synthetic".to_owned() }
    }

    fn perfect_scores(queries: &[QueryTruth], candidates: &[CandidateIdentity]) -> HashMap<(String, String), f64> {
        let mut scores = HashMap::new();
        for query in queries {
            for candidate in candidates {
                let score = if query.query_subject == candidate.candidate_subject { 0.99 } else { 0.10 };
                scores.insert((query.query_id.clone(), candidate.candidate_id.clone()), score);
            }
        }
        scores
    }

    #[test]
    fn gallery_sizes_must_be_strictly_increasing() {
        assert_eq!(parse_gallery_sizes("2,4,8").unwrap(), vec![2, 4, 8]);
        assert!(parse_gallery_sizes("2,2,8").is_err());
        assert!(parse_gallery_sizes("4,2,8").is_err());
        assert!(parse_gallery_sizes("1,4").is_err());
    }

    #[test]
    fn gallery_draws_are_deterministic_and_nested() {
        let candidates = candidate_frame(12);
        let sizes = vec![2, 4, 8, 12];
        let a = generate_nested_gallery_draws(&candidates, &sizes, 5, 9901);
        let b = generate_nested_gallery_draws(&candidates, &sizes, 5, 9901);
        assert_eq!(a.len(), 20);
        assert_eq!(a.iter().map(|d| (&d.draw_id, d.gallery_size, &d.members)).collect::<Vec<_>>(),
                   b.iter().map(|d| (&d.draw_id, d.gallery_size, &d.members)).collect::<Vec<_>>());
        for draw_index in 0..5 {
            let group = &a[draw_index * sizes.len()..(draw_index + 1) * sizes.len()];
            for pair in group.windows(2) {
                let smaller: HashSet<&str> = pair[0].members.iter().map(|c| c.candidate_id.as_str()).collect();
                let larger: HashSet<&str> = pair[1].members.iter().map(|c| c.candidate_id.as_str()).collect();
                assert!(smaller.is_subset(&larger));
                assert_eq!(pair[0].gallery_size, pair[0].members.len());
                assert_eq!(pair[1].gallery_size, pair[1].members.len());
            }
        }
    }

    #[test]
    fn true_target_missing_from_one_gallery_is_scored_as_nonmated_for_that_draw() {
        let candidates = candidate_frame(4);
        let queries = vec![test_query("q1", "subject_003"), test_query("q2", "outside_subject")];
        let scores = perfect_scores(&queries, &candidates);
        let members = candidates[..2].to_vec();
        let rows = build_query_results(&queries, &scores, &members, "draw_1", 2).unwrap();
        assert!(!rows[0].matched);
        assert!(!rows[1].matched);
        assert!(rows[0].target_score.is_none());
    }

    #[test]
    fn threshold_is_selected_from_validation_observations_only_and_respects_cap() {
        let candidates = candidate_frame(4);
        let validation = vec![test_query("q1", "subject_000"), test_query("q2", "outside_1")];
        let scores = perfect_scores(&validation, &candidates);
        let rows = build_query_results(&validation, &scores, &candidates, "draw_1", 4).unwrap();
        let threshold = select_validation_threshold(&rows, 0.0);
        assert_eq!(threshold, Some(0.99));
        let rates = weighted_metrics(&rows, threshold, None, None);
        assert_eq!(rates.fpir, Some(0.0));
        assert_eq!(rates.fnir1, Some(0.0));
    }

    #[test]
    fn crossed_bootstrap_and_report_are_deterministic_and_never_claim_release() {
        let candidates = candidate_frame(6);
        let validation = vec![test_query("v1", "subject_000"), test_query("v2", "outside_v")];
        let test = vec![test_query("t1", "subject_001"), test_query("t2", "outside_t")];
        let val_scores = perfect_scores(&validation, &candidates);
        let test_scores = perfect_scores(&test, &candidates);
        let sizes = vec![2, 4, 6];
        let draws = generate_nested_gallery_draws(&candidates, &sizes, 4, 9901);
        let mut val_by_size = BTreeMap::new();
        let mut test_by_size = BTreeMap::new();
        for draw in &draws {
            val_by_size.entry(draw.gallery_size).or_insert_with(Vec::new)
                .extend(build_query_results(&validation, &val_scores, &draw.members, &draw.draw_id, draw.gallery_size).unwrap());
            test_by_size.entry(draw.gallery_size).or_insert_with(Vec::new)
                .extend(build_query_results(&test, &test_scores, &draw.members, &draw.draw_id, draw.gallery_size).unwrap());
        }
        let thresholds: BTreeMap<usize, Option<f64>> = sizes.iter()
            .map(|size| (*size, select_validation_threshold(&val_by_size[size], 0.01))).collect();
        let make_context = || ReportContext {
            sizes: &sizes,
            draws: &draws,
            validation_by_size: &val_by_size,
            test_by_size: &test_by_size,
            thresholds: &thresholds,
            reference_size: 6,
            frozen_threshold: thresholds[&6],
            candidate_count: candidates.len(),
            validation_truth: &validation,
            test_truth: &test,
            max_fpir: 0.01,
            draw_count: 4,
            bootstrap_replicates: 200,
            seed: 9901,
        };
        let report_a = build_report(make_context());
        let report_b = build_report(make_context());
        assert_eq!(report_a, report_b);
        assert!(report_a.contains("KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION"));
        assert!(report_a.contains("NOT_EVALUATED_CROSSED_GALLERY_EVIDENCE_IS_NOT_RELEASE_CERTIFICATION"));
    }

    #[test]
    fn score_grid_rejects_missing_pairs_and_training_overlap_is_fail_closed() {
        let candidates = candidate_frame(3);
        let queries = vec![test_query("q1", "subject_000")];
        let mut scores = perfect_scores(&queries, &candidates);
        scores.remove(&("q1".to_owned(), "candidate_002".to_owned()));
        assert!(validate_score_grid(&queries, &scores, &candidates, "test").is_err());
        let training = HashSet::from(["subject_001".to_owned()]);
        assert!(validate_training_disjointness(&queries, &[], &HashSet::from(["subject_000".to_owned(), "subject_001".to_owned()]), &training).is_err());
    }
}
