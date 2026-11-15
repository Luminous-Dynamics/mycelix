#![forbid(unsafe_code)]

// Evaluates downstream effects of candidate entity clusters against declared
// gold labels. It does not perform linkage or authenticate the gold standard.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const HEADER: &str = "record_id|namespace|truth_entity_id|predicted_cluster_id|truth_entity_weight|predicted_cluster_weight|effective_at|recorded_at";
const MAX_INPUT_BYTES: usize = 8 * 1024 * 1024;
const MAX_TOKEN_BYTES: usize = 200;
const MAX_RECORDS_HARD: u64 = 100_000;

type EntityKey = (String, String);
type ClusterKey = (String, String);
type ContingencyKey = (EntityKey, ClusterKey);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Record {
    pub record_id: String,
    pub namespace: String,
    pub truth_entity_id: String,
    pub predicted_cluster_id: String,
    pub truth_entity_weight: u64,
    pub predicted_cluster_weight: u64,
    pub effective_at: u64,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Problem {
    pub benchmark_id: String,
    pub scope_sha256: String,
    pub effective_cutoff: u64,
    pub recorded_cutoff: u64,
    pub max_records: u64,
    pub records: Vec<Record>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Report {
    pub problem: Problem,
    pub effective_record_count: u128,
    pub future_effective_excluded_count: u128,
    pub late_arrival_leakage_count: u128,
    pub candidate_assigned_record_count: u128,
    pub candidate_unassigned_record_count: u128,
    pub namespace_count: usize,
    pub truth_entity_count: usize,
    pub predicted_cluster_count: usize,
    pub true_pair_count: u128,
    pub predicted_pair_count: u128,
    pub true_positive_pair_count: u128,
    pub false_positive_pair_count: u128,
    pub false_negative_pair_count: u128,
    pub pair_precision: String,
    pub pair_recall: String,
    pub bridge_cluster_count: usize,
    pub bridge_entity_pair_count: u128,
    pub bridge_at_risk_population_weight: u128,
    pub fragmented_entity_count: usize,
    pub fragmentation_duplicate_weight_at_risk: u128,
    pub truth_population_weight: u128,
    pub predicted_population_weight: u128,
    pub denominator_weight_bias: String,
    pub truth_entity_count_bias: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LinkageImpactError(pub String);

impl std::fmt::Display for LinkageImpactError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for LinkageImpactError {}

fn fail(message: impl Into<String>) -> LinkageImpactError {
    LinkageImpactError(message.into())
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_TOKEN_BYTES
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn parse_u64(value: &str, field: &str) -> Result<u64, LinkageImpactError> {
    value
        .parse::<u64>()
        .map_err(|error| fail(format!("invalid {field}: {error}")))
}

fn choose_two(count: u128) -> Result<u128, LinkageImpactError> {
    if count < 2 {
        return Ok(0);
    }
    let (left, right) = if count % 2 == 0 {
        (count / 2, count - 1)
    } else {
        (count, (count - 1) / 2)
    };
    left.checked_mul(right)
        .ok_or_else(|| fail("pair count overflow"))
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn ratio(numerator: u128, denominator: u128) -> String {
    if denominator == 0 {
        return "NOT_ESTIMABLE".to_owned();
    }
    let divisor = gcd(numerator, denominator);
    format!("{}/{}", numerator / divisor, denominator / divisor)
}

fn signed_difference(left: u128, right: u128) -> String {
    if left >= right {
        format!("+{}", left - right)
    } else {
        format!("-{}", right - left)
    }
}

pub fn parse_input(source: &str) -> Result<Problem, LinkageImpactError> {
    if source.len() > MAX_INPUT_BYTES {
        return Err(fail("input exceeds 8 MiB"));
    }

    let mut metadata = BTreeMap::<String, String>::new();
    let mut lines = Vec::<(usize, &str)>::new();
    for (index, raw) in source.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let (key, value) = comment
                .trim()
                .split_once('=')
                .ok_or_else(|| fail(format!("line {}: malformed metadata", index + 1)))?;
            if !matches!(
                key,
                "benchmark_id"
                    | "scope_sha256"
                    | "effective_cutoff"
                    | "recorded_cutoff"
                    | "max_records"
            ) {
                return Err(fail(format!(
                    "line {}: unknown metadata key {key}",
                    index + 1
                )));
            }
            if value.is_empty() || metadata.insert(key.to_owned(), value.to_owned()).is_some() {
                return Err(fail(format!(
                    "line {}: empty or duplicate metadata key",
                    index + 1
                )));
            }
        } else {
            lines.push((index + 1, line));
        }
    }

    let required = |key: &str| -> Result<&str, LinkageImpactError> {
        metadata
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| fail(format!("missing metadata key {key}")))
    };

    let benchmark_id = required("benchmark_id")?.to_owned();
    let scope_sha256 = required("scope_sha256")?.to_owned();
    let effective_cutoff = parse_u64(required("effective_cutoff")?, "effective_cutoff")?;
    let recorded_cutoff = parse_u64(required("recorded_cutoff")?, "recorded_cutoff")?;
    let max_records = parse_u64(required("max_records")?, "max_records")?;

    if !valid_token(&benchmark_id) {
        return Err(fail("benchmark_id must be a valid token"));
    }
    if !valid_sha256(&scope_sha256) {
        return Err(fail(
            "scope_sha256 must be 64 lowercase hexadecimal characters",
        ));
    }
    if max_records == 0 || max_records > MAX_RECORDS_HARD {
        return Err(fail(format!(
            "max_records must be in 1..={MAX_RECORDS_HARD}"
        )));
    }
    if lines.first().map(|(_, line)| *line) != Some(HEADER) {
        return Err(fail("expected exact linkage-impact record header"));
    }

    let mut records = Vec::new();
    let mut record_ids = BTreeSet::<String>::new();
    for (line_no, line) in lines.iter().skip(1) {
        let fields = line.split('|').map(str::trim).collect::<Vec<_>>();
        if fields.len() != 8
            || !valid_token(fields[0])
            || !valid_token(fields[1])
            || !valid_token(fields[2])
            || (fields[3] != "-" && !valid_token(fields[3]))
        {
            return Err(fail(format!("line {line_no}: invalid record row")));
        }
        if !record_ids.insert(fields[0].to_owned()) {
            return Err(fail(format!("line {line_no}: duplicate record ID")));
        }
        let truth_entity_weight = parse_u64(fields[4], "truth_entity_weight")?;
        let predicted_cluster_weight = parse_u64(fields[5], "predicted_cluster_weight")?;
        if fields[3] == "-" && predicted_cluster_weight != 0 {
            return Err(fail(format!(
                "line {line_no}: unassigned record must have predicted_cluster_weight=0"
            )));
        }
        records.push(Record {
            record_id: fields[0].to_owned(),
            namespace: fields[1].to_owned(),
            truth_entity_id: fields[2].to_owned(),
            predicted_cluster_id: fields[3].to_owned(),
            truth_entity_weight,
            predicted_cluster_weight,
            effective_at: parse_u64(fields[6], "effective_at")?,
            recorded_at: parse_u64(fields[7], "recorded_at")?,
        });
        if records.len() > max_records as usize {
            return Err(fail("record count exceeds declared max_records"));
        }
    }
    if records.is_empty() {
        return Err(fail("at least one linkage-impact record is required"));
    }

    Ok(Problem {
        benchmark_id,
        scope_sha256,
        effective_cutoff,
        recorded_cutoff,
        max_records,
        records,
    })
}

pub fn analyze(problem: &Problem) -> Result<Report, LinkageImpactError> {
    let mut record_counts_by_truth = BTreeMap::<EntityKey, u128>::new();
    let mut record_counts_by_cluster = BTreeMap::<ClusterKey, u128>::new();
    let mut contingency = BTreeMap::<ContingencyKey, u128>::new();
    let mut truth_weights = BTreeMap::<EntityKey, u128>::new();
    let mut predicted_weights = BTreeMap::<ClusterKey, u128>::new();
    let mut cluster_truths = BTreeMap::<ClusterKey, BTreeSet<EntityKey>>::new();
    let mut truth_clusters = BTreeMap::<EntityKey, BTreeSet<ClusterKey>>::new();
    let mut namespaces = BTreeSet::<String>::new();

    let mut effective_record_count = 0_u128;
    let mut future_effective_excluded_count = 0_u128;
    let mut late_arrival_leakage_count = 0_u128;
    let mut candidate_assigned_record_count = 0_u128;
    let mut candidate_unassigned_record_count = 0_u128;

    for record in &problem.records {
        if record.effective_at > problem.effective_cutoff {
            future_effective_excluded_count = future_effective_excluded_count
                .checked_add(1)
                .ok_or_else(|| fail("future-effective record counter overflow"))?;
            continue;
        }
        effective_record_count = effective_record_count
            .checked_add(1)
            .ok_or_else(|| fail("effective record counter overflow"))?;

        let entity_key = (
            record.namespace.clone(),
            record.truth_entity_id.clone(),
        );
        namespaces.insert(record.namespace.clone());

        match truth_weights.get(&entity_key) {
            Some(existing) if *existing != u128::from(record.truth_entity_weight) => {
                return Err(fail(format!(
                    "inconsistent truth weight for {}/{}",
                    entity_key.0, entity_key.1
                )));
            }
            None => {
                truth_weights.insert(
                    entity_key.clone(),
                    u128::from(record.truth_entity_weight),
                );
            }
            _ => {}
        }
        let truth_count = record_counts_by_truth.entry(entity_key.clone()).or_default();
        *truth_count = truth_count
            .checked_add(1)
            .ok_or_else(|| fail("truth record count overflow"))?;

        // Late-recorded rows are visible to the retrospective gold population,
        // but not to a decision made at the recorded-time cutoff.
        if record.recorded_at > problem.recorded_cutoff {
            if record.predicted_cluster_id != "-" {
                late_arrival_leakage_count = late_arrival_leakage_count
                    .checked_add(1)
                    .ok_or_else(|| fail("late-arrival leakage counter overflow"))?;
            }
            candidate_unassigned_record_count = candidate_unassigned_record_count
                .checked_add(1)
                .ok_or_else(|| fail("unassigned record counter overflow"))?;
            continue;
        }

        if record.predicted_cluster_id == "-" {
            candidate_unassigned_record_count = candidate_unassigned_record_count
                .checked_add(1)
                .ok_or_else(|| fail("unassigned record counter overflow"))?;
            continue;
        }

        candidate_assigned_record_count = candidate_assigned_record_count
            .checked_add(1)
            .ok_or_else(|| fail("assigned record counter overflow"))?;
        let cluster_key = (
            record.namespace.clone(),
            record.predicted_cluster_id.clone(),
        );

        match predicted_weights.get(&cluster_key) {
            Some(existing) if *existing != u128::from(record.predicted_cluster_weight) => {
                return Err(fail(format!(
                    "inconsistent predicted cluster weight for {}/{}",
                    cluster_key.0, cluster_key.1
                )));
            }
            None => {
                predicted_weights.insert(
                    cluster_key.clone(),
                    u128::from(record.predicted_cluster_weight),
                );
            }
            _ => {}
        }

        let cluster_count = record_counts_by_cluster
            .entry(cluster_key.clone())
            .or_default();
        *cluster_count = cluster_count
            .checked_add(1)
            .ok_or_else(|| fail("predicted cluster record count overflow"))?;

        let cell_count = contingency
            .entry((entity_key.clone(), cluster_key.clone()))
            .or_default();
        *cell_count = cell_count
            .checked_add(1)
            .ok_or_else(|| fail("contingency count overflow"))?;

        cluster_truths
            .entry(cluster_key.clone())
            .or_default()
            .insert(entity_key.clone());
        truth_clusters
            .entry(entity_key)
            .or_default()
            .insert(cluster_key);
    }

    let true_pair_count = record_counts_by_truth.values().try_fold(
        0_u128,
        |sum, count| {
            sum.checked_add(choose_two(*count)?)
                .ok_or_else(|| fail("true pair total overflow"))
        },
    )?;
    let predicted_pair_count = record_counts_by_cluster.values().try_fold(
        0_u128,
        |sum, count| {
            sum.checked_add(choose_two(*count)?)
                .ok_or_else(|| fail("predicted pair total overflow"))
        },
    )?;
    let true_positive_pair_count = contingency.values().try_fold(
        0_u128,
        |sum, count| {
            sum.checked_add(choose_two(*count)?)
                .ok_or_else(|| fail("true-positive pair total overflow"))
        },
    )?;
    let false_positive_pair_count = predicted_pair_count
        .checked_sub(true_positive_pair_count)
        .ok_or_else(|| fail("true-positive count exceeds predicted pair count"))?;
    let false_negative_pair_count = true_pair_count
        .checked_sub(true_positive_pair_count)
        .ok_or_else(|| fail("true-positive count exceeds true pair count"))?;

    let mut bridge_cluster_count = 0_usize;
    let mut bridge_entity_pair_count = 0_u128;
    let mut bridge_at_risk_population_weight = 0_u128;
    for entities in cluster_truths.values() {
        if entities.len() <= 1 {
            continue;
        }
        bridge_cluster_count += 1;
        bridge_entity_pair_count = bridge_entity_pair_count
            .checked_add(choose_two(entities.len() as u128)?)
            .ok_or_else(|| fail("bridge entity-pair count overflow"))?;
        let mut total = 0_u128;
        let mut maximum = 0_u128;
        for entity in entities {
            let weight = truth_weights[entity];
            total = total
                .checked_add(weight)
                .ok_or_else(|| fail("bridge population weight overflow"))?;
            maximum = maximum.max(weight);
        }
        bridge_at_risk_population_weight = bridge_at_risk_population_weight
            .checked_add(total - maximum)
            .ok_or_else(|| fail("bridge at-risk population weight overflow"))?;
    }

    let mut fragmented_entity_count = 0_usize;
    let mut fragmentation_duplicate_weight_at_risk = 0_u128;
    for (entity, clusters) in &truth_clusters {
        if clusters.len() <= 1 {
            continue;
        }
        fragmented_entity_count += 1;
        let weight = truth_weights[entity];
        let extra_copies = (clusters.len() - 1) as u128;
        let duplicate_risk = weight
            .checked_mul(extra_copies)
            .ok_or_else(|| fail("fragmentation duplicate weight overflow"))?;
        fragmentation_duplicate_weight_at_risk = fragmentation_duplicate_weight_at_risk
            .checked_add(duplicate_risk)
            .ok_or_else(|| fail("fragmentation duplicate-weight total overflow"))?;
    }

    let truth_population_weight = truth_weights.values().try_fold(
        0_u128,
        |sum, weight| {
            sum.checked_add(*weight)
                .ok_or_else(|| fail("truth population weight overflow"))
        },
    )?;
    let predicted_population_weight = predicted_weights.values().try_fold(
        0_u128,
        |sum, weight| {
            sum.checked_add(*weight)
                .ok_or_else(|| fail("predicted population weight overflow"))
        },
    )?;

    let truth_entity_count = truth_weights.len();
    let predicted_cluster_count = predicted_weights.len();

    Ok(Report {
        problem: problem.clone(),
        effective_record_count,
        future_effective_excluded_count,
        late_arrival_leakage_count,
        candidate_assigned_record_count,
        candidate_unassigned_record_count,
        namespace_count: namespaces.len(),
        truth_entity_count,
        predicted_cluster_count,
        true_pair_count,
        predicted_pair_count,
        true_positive_pair_count,
        false_positive_pair_count,
        false_negative_pair_count,
        pair_precision: ratio(true_positive_pair_count, predicted_pair_count),
        pair_recall: ratio(true_positive_pair_count, true_pair_count),
        bridge_cluster_count,
        bridge_entity_pair_count,
        bridge_at_risk_population_weight,
        fragmented_entity_count,
        fragmentation_duplicate_weight_at_risk,
        truth_population_weight,
        predicted_population_weight,
        denominator_weight_bias: signed_difference(
            predicted_population_weight,
            truth_population_weight,
        ),
        truth_entity_count_bias: signed_difference(
            predicted_cluster_count as u128,
            truth_entity_count as u128,
        ),
    })
}

impl Report {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "schema=fsre.linkage-impact-e0").expect("String write");
        writeln!(out, "version=0.1.0").expect("String write");
        writeln!(out, "benchmark_id={}", self.problem.benchmark_id).expect("String write");
        writeln!(out, "scope_sha256={}", self.problem.scope_sha256).expect("String write");
        writeln!(out, "effective_cutoff={}", self.problem.effective_cutoff).expect("String write");
        writeln!(out, "recorded_cutoff={}", self.problem.recorded_cutoff).expect("String write");
        writeln!(out, "effective_record_count={}", self.effective_record_count)
            .expect("String write");
        writeln!(
            out,
            "future_effective_excluded_count={}",
            self.future_effective_excluded_count
        )
        .expect("String write");
        writeln!(out, "late_arrival_leakage_count={}", self.late_arrival_leakage_count)
            .expect("String write");
        writeln!(
            out,
            "candidate_assigned_record_count={}",
            self.candidate_assigned_record_count
        )
        .expect("String write");
        writeln!(
            out,
            "candidate_unassigned_record_count={}",
            self.candidate_unassigned_record_count
        )
        .expect("String write");
        writeln!(out, "namespace_count={}", self.namespace_count).expect("String write");
        writeln!(out, "truth_entity_count={}", self.truth_entity_count).expect("String write");
        writeln!(out, "predicted_cluster_count={}", self.predicted_cluster_count)
            .expect("String write");
        writeln!(out, "true_pair_count={}", self.true_pair_count).expect("String write");
        writeln!(out, "predicted_pair_count={}", self.predicted_pair_count)
            .expect("String write");
        writeln!(
            out,
            "true_positive_pair_count={}",
            self.true_positive_pair_count
        )
        .expect("String write");
        writeln!(
            out,
            "false_positive_pair_count={}",
            self.false_positive_pair_count
        )
        .expect("String write");
        writeln!(
            out,
            "false_negative_pair_count={}",
            self.false_negative_pair_count
        )
        .expect("String write");
        writeln!(out, "pair_precision={}", self.pair_precision).expect("String write");
        writeln!(out, "pair_recall={}", self.pair_recall).expect("String write");
        writeln!(out, "bridge_cluster_count={}", self.bridge_cluster_count)
            .expect("String write");
        writeln!(
            out,
            "bridge_entity_pair_count={}",
            self.bridge_entity_pair_count
        )
        .expect("String write");
        writeln!(
            out,
            "bridge_at_risk_population_weight={}",
            self.bridge_at_risk_population_weight
        )
        .expect("String write");
        writeln!(out, "fragmented_entity_count={}", self.fragmented_entity_count)
            .expect("String write");
        writeln!(
            out,
            "fragmentation_duplicate_weight_at_risk={}",
            self.fragmentation_duplicate_weight_at_risk
        )
        .expect("String write");
        writeln!(
            out,
            "truth_population_weight={}",
            self.truth_population_weight
        )
        .expect("String write");
        writeln!(
            out,
            "predicted_population_weight={}",
            self.predicted_population_weight
        )
        .expect("String write");
        writeln!(
            out,
            "denominator_weight_bias={}",
            self.denominator_weight_bias
        )
        .expect("String write");
        writeln!(
            out,
            "truth_entity_count_bias={}",
            self.truth_entity_count_bias
        )
        .expect("String write");
        out.push_str("gold_truth_authenticity=NOT_ESTABLISHED_BY_EVALUATOR\n");
        out.push_str("linkage_execution=NOT_PERFORMED\n");
        out.push_str("source_independence=NOT_ESTABLISHED\n");
        out.push_str("legal_identity=NOT_ESTABLISHED\n");
        out.push_str("decision_authorization=NOT_GRANTED\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/bridge-and-split.psv");

    fn fixture() -> Problem {
        parse_input(FIXTURE).expect("fixture parses")
    }

    #[test]
    fn bridge_and_split_failures_remain_visible_when_count_bias_cancels() {
        let report = analyze(&fixture()).unwrap();
        assert_eq!(report.truth_entity_count, 3);
        assert_eq!(report.predicted_cluster_count, 3);
        assert_eq!(report.truth_entity_count_bias, "+0");
        assert_eq!(report.bridge_cluster_count, 1);
        assert_eq!(report.bridge_entity_pair_count, 1);
        assert_eq!(report.bridge_at_risk_population_weight, 80);
        assert_eq!(report.fragmented_entity_count, 1);
        assert_eq!(report.fragmentation_duplicate_weight_at_risk, 50);
    }

    #[test]
    fn pair_metrics_are_exact_and_reported_separately_from_denominator_harm() {
        let report = analyze(&fixture()).unwrap();
        assert_eq!(report.true_pair_count, 4);
        assert_eq!(report.predicted_pair_count, 3);
        assert_eq!(report.true_positive_pair_count, 1);
        assert_eq!(report.false_positive_pair_count, 2);
        assert_eq!(report.false_negative_pair_count, 3);
        assert_eq!(report.pair_precision, "1/3");
        assert_eq!(report.pair_recall, "1/4");
        assert_eq!(report.denominator_weight_bias, "-30");
        assert_eq!(report.truth_population_weight, 230);
        assert_eq!(report.predicted_population_weight, 200);
    }

    #[test]
    fn late_arrival_assignment_is_counted_but_excluded_from_asof_clusters() {
        let report = analyze(&fixture()).unwrap();
        assert_eq!(report.late_arrival_leakage_count, 1);
        assert_eq!(report.candidate_unassigned_record_count, 1);
        assert_eq!(report.candidate_assigned_record_count, 5);
        assert_eq!(report.future_effective_excluded_count, 1);
        assert_eq!(report.effective_record_count, 6);
    }

    #[test]
    fn equal_identifiers_in_different_namespaces_are_not_merged() {
        let input = String::from(
            "#benchmark_id=namespace-separation\n\
             #scope_sha256=cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n\
             #effective_cutoff=10\n\
             #recorded_cutoff=10\n\
             #max_records=10\n\
             record_id|namespace|truth_entity_id|predicted_cluster_id|truth_entity_weight|predicted_cluster_weight|effective_at|recorded_at\n\
             r1|PERSON|same-id|same-cluster|10|10|1|1\n\
             r2|ASSET|same-id|same-cluster|20|20|1|1\n",
        );
        let report = analyze(&parse_input(&input).unwrap()).unwrap();
        assert_eq!(report.namespace_count, 2);
        assert_eq!(report.truth_entity_count, 2);
        assert_eq!(report.predicted_cluster_count, 2);
        assert_eq!(report.bridge_cluster_count, 0);
        assert_eq!(report.predicted_population_weight, 30);
    }

    #[test]
    fn inconsistent_gold_or_predicted_weights_fail_closed() {
        let gold = FIXTURE.replace("r2|PERSON|entity-a|cluster-ab|100", "r2|PERSON|entity-a|cluster-ab|101");
        assert!(analyze(&parse_input(&gold).unwrap()).is_err());
        let predicted = FIXTURE.replace("r2|PERSON|entity-a|cluster-ab|100|100", "r2|PERSON|entity-a|cluster-ab|100|101");
        assert!(analyze(&parse_input(&predicted).unwrap()).is_err());
    }

    #[test]
    fn duplicate_record_ids_and_unassigned_nonzero_weights_are_rejected() {
        let duplicate = FIXTURE.replace(
            "r2|PERSON|entity-a|cluster-ab|100|100|2|11",
            "r1|PERSON|entity-a|cluster-ab|100|100|2|11",
        );
        assert!(parse_input(&duplicate).is_err());
        let unassigned = FIXTURE.replace(
            "r1|PERSON|entity-a|cluster-ab|100|100|1|10",
            "r1|PERSON|entity-a|-|100|100|1|10",
        );
        assert!(parse_input(&unassigned).is_err());
    }

    #[test]
    fn row_permutation_does_not_change_the_receipt() {
        let original = analyze(&fixture()).unwrap().render();
        let mut lines = FIXTURE.lines().collect::<Vec<_>>();
        let header = lines.iter().position(|line| line.starts_with("record_id|")).unwrap();
        lines[header + 1..].reverse();
        let reordered = lines.join("\n");
        assert_eq!(original, analyze(&parse_input(&reordered).unwrap()).unwrap().render());
    }
}
