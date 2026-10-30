//! Detect leakage through transitive links across multiple reviewed dependency keys.
//!
//! Input CSV must contain row_id, split, and every dependency column named on
//! the command line. Example dependency keys: publication_id, study_id,
//! experiment_id, site_id, material_batch_id, curve_id.
//!
//! Rows sharing ANY selected key are unioned into connected components. A
//! component crossing split boundaries is a leakage failure. This tool cannot
//! discover missing crosswalk links or prove that supplied IDs are correct.

use csv::{ReaderBuilder, StringRecord};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const ALLOWED_SPLITS: [&str; 4] = ["train", "validation", "test", "excluded"];
const PLACEHOLDER_IDS: [&str; 7] = ["", "na", "n/a", "none", "null", "unknown", "missing"];

#[derive(Debug)]
struct ManifestRow {
    row_id: String,
    split: String,
    dependencies: Vec<Option<String>>,
}

#[derive(Debug, Serialize, Default)]
struct SplitCounts {
    rows: usize,
    components: usize,
}

#[derive(Debug, Serialize, Default)]
struct DependencyKeyStats {
    rows_with_value: usize,
    rows_missing_value: usize,
    distinct_values: usize,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    input_label: String,
    input_sha256: String,
    row_count: usize,
    component_count: usize,
    dependency_columns: Vec<String>,
    component_assignment_sha256: String,
    split_counts: BTreeMap<String, SplitCounts>,
    dependency_key_stats: BTreeMap<String, DependencyKeyStats>,
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

fn is_placeholder(value: &str) -> bool {
    let normalized = value.to_ascii_lowercase();
    PLACEHOLDER_IDS.contains(&normalized.as_str()) || value == "-"
}

fn validate_identifier(value: &str, kind: &str, row_number: usize) -> Result<(), Box<dyn Error>> {
    if value.trim() != value {
        return Err(format!(
            "{kind} has leading/trailing whitespace at data row {row_number}; refusing ambiguous aliases"
        )
        .into());
    }
    if is_placeholder(value) {
        return Err(format!("{kind} is blank or a placeholder at data row {row_number}").into());
    }
    Ok(())
}

fn required_header(headers: &StringRecord, name: &str) -> Result<usize, Box<dyn Error>> {
    let matches: Vec<usize> = headers
        .iter()
        .enumerate()
        .filter_map(|(index, header)| (header == name).then_some(index))
        .collect();
    match matches.as_slice() {
        [index] => Ok(*index),
        [] => Err(format!("required manifest column is missing: {name}").into()),
        _ => Err(format!("manifest has duplicate columns named {name:?}").into()),
    }
}

fn parse_dependency_columns(value: &str) -> Result<Vec<String>, Box<dyn Error>> {
    let columns: Vec<String> = value.split(',').map(str::to_owned).collect();
    if columns.is_empty() {
        return Err("at least one dependency column is required".into());
    }
    let mut seen = HashSet::new();
    for column in &columns {
        if column.is_empty() || column.trim() != column {
            return Err(
                "dependency column names must be non-empty and have no surrounding whitespace"
                    .into(),
            );
        }
        if column == "row_id" || column == "split" {
            return Err(format!("{column:?} is reserved and cannot be a dependency column").into());
        }
        if !seen.insert(column.clone()) {
            return Err(format!("dependency column {column:?} was selected more than once").into());
        }
    }
    Ok(columns)
}

fn parse_rows(
    bytes: &[u8],
    dependency_columns: &[String],
) -> Result<(Vec<ManifestRow>, BTreeMap<String, DependencyKeyStats>), Box<dyn Error>> {
    if bytes.is_empty() {
        return Err("input manifest is empty".into());
    }
    let mut reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(bytes);
    let headers = reader.headers()?.clone();
    let row_id_index = required_header(&headers, "row_id")?;
    let split_index = required_header(&headers, "split")?;
    let dependency_indices: Vec<usize> = dependency_columns
        .iter()
        .map(|column| required_header(&headers, column))
        .collect::<Result<_, _>>()?;

    let mut rows = Vec::new();
    let mut seen_row_ids = HashSet::<String>::new();
    let mut key_values = BTreeMap::<String, HashSet<String>>::new();
    let mut stats = BTreeMap::<String, DependencyKeyStats>::new();
    for column in dependency_columns {
        key_values.insert(column.clone(), HashSet::new());
        stats.insert(column.clone(), DependencyKeyStats::default());
    }

    for (index, record) in reader.records().enumerate() {
        let record = record.map_err(|error| format!("invalid CSV row {}: {error}", index + 1))?;
        let row_number = index + 1;
        let row_id = record
            .get(row_id_index)
            .ok_or_else(|| format!("row_id missing at data row {row_number}"))?
            .to_owned();
        validate_identifier(&row_id, "row_id", row_number)?;
        if !seen_row_ids.insert(row_id.clone()) {
            return Err(format!("duplicate row_id {row_id:?} at data row {row_number}").into());
        }

        let split = record
            .get(split_index)
            .ok_or_else(|| format!("split missing at data row {row_number}"))?
            .to_owned();
        if !ALLOWED_SPLITS.contains(&split.as_str()) {
            return Err(format!(
                "unknown split {split:?} at data row {row_number}; allowed: train, validation, test, excluded"
            )
            .into());
        }

        let mut dependencies = Vec::with_capacity(dependency_columns.len());
        for (column, &column_index) in dependency_columns.iter().zip(&dependency_indices) {
            let value = record
                .get(column_index)
                .ok_or_else(|| format!("{column} missing at data row {row_number}"))?;
            if value.is_empty() || is_placeholder(value) {
                stats
                    .get_mut(column)
                    .expect("stats initialized")
                    .rows_missing_value += 1;
                dependencies.push(None);
                continue;
            }
            validate_identifier(value, column, row_number)?;
            stats
                .get_mut(column)
                .expect("stats initialized")
                .rows_with_value += 1;
            key_values
                .get_mut(column)
                .expect("key set initialized")
                .insert(value.to_owned());
            dependencies.push(Some(value.to_owned()));
        }

        if dependencies.iter().all(Option::is_none) {
            return Err(format!(
                "row_id {row_id:?} has no usable value in any selected dependency column; confirmatory independence cannot be assessed"
            )
            .into());
        }
        rows.push(ManifestRow {
            row_id,
            split,
            dependencies,
        });
    }

    if rows.is_empty() {
        return Err("manifest contains a header but no rows".into());
    }
    for column in dependency_columns {
        stats
            .get_mut(column)
            .expect("stats initialized")
            .distinct_values = key_values.get(column).expect("key set initialized").len();
    }
    Ok((rows, stats))
}

struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new(size: usize) -> Self {
        Self {
            parent: (0..size).collect(),
            rank: vec![0; size],
        }
    }

    fn find(&mut self, index: usize) -> usize {
        if self.parent[index] != index {
            let root = self.find(self.parent[index]);
            self.parent[index] = root;
        }
        self.parent[index]
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root == right_root {
            return;
        }
        if self.rank[left_root] < self.rank[right_root] {
            self.parent[left_root] = right_root;
        } else if self.rank[left_root] > self.rank[right_root] {
            self.parent[right_root] = left_root;
        } else {
            self.parent[right_root] = left_root;
            self.rank[left_root] += 1;
        }
    }
}

fn assignment_digest(assignments: &BTreeMap<String, String>) -> String {
    let mut hasher = Sha256::new();
    for (row_id, assignment) in assignments {
        hasher.update((row_id.len() as u64).to_le_bytes());
        hasher.update(row_id.as_bytes());
        hasher.update((assignment.len() as u64).to_le_bytes());
        hasher.update(assignment.as_bytes());
    }
    let digest = hasher.finalize();
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to a String is infallible");
    }
    output
}

fn ensure_distinct_paths(input: &Path, output: &Path) -> Result<(), Box<dyn Error>> {
    if output.exists() && fs::canonicalize(input)? == fs::canonicalize(output)? {
        return Err(
            "input and output paths resolve to the same file; refusing to overwrite the manifest"
                .into(),
        );
    }
    Ok(())
}

fn audit(
    bytes: &[u8],
    input_label: &str,
    dependency_columns: &[String],
) -> Result<AuditReport, Box<dyn Error>> {
    let (rows, dependency_key_stats) = parse_rows(bytes, dependency_columns)?;
    let mut sets = UnionFind::new(rows.len());
    let mut first_row_by_key = HashMap::<(usize, String), usize>::new();

    for (row_index, row) in rows.iter().enumerate() {
        for (column_index, value) in row.dependencies.iter().enumerate() {
            let Some(value) = value else {
                continue;
            };
            let key = (column_index, value.clone());
            if let Some(&first_row) = first_row_by_key.get(&key) {
                sets.union(first_row, row_index);
            } else {
                first_row_by_key.insert(key, row_index);
            }
        }
    }

    let mut members_by_root = BTreeMap::<usize, Vec<usize>>::new();
    for index in 0..rows.len() {
        let root = sets.find(index);
        members_by_root.entry(root).or_default().push(index);
    }

    let mut split_counts = BTreeMap::<String, SplitCounts>::new();
    for split in ALLOWED_SPLITS {
        split_counts.insert(split.to_owned(), SplitCounts::default());
    }
    let mut assignments = BTreeMap::<String, String>::new();

    for members in members_by_root.values() {
        let first = &rows[members[0]];
        let component_split = first.split.as_str();
        let mut member_ids: Vec<&str> = members
            .iter()
            .map(|&index| rows[index].row_id.as_str())
            .collect();
        member_ids.sort_unstable();
        let component_label = member_ids[0].to_owned();
        for &index in members.iter().skip(1) {
            if rows[index].split != component_split {
                return Err(format!(
                    "dependency leakage: connected component containing row_id {:?} crosses {:?} and {:?} via selected keys {}; review the source crosswalk or use a disjoint split",
                    component_label,
                    component_split,
                    rows[index].split,
                    dependency_columns.join(",")
                )
                .into());
            }
        }
        let counts = split_counts
            .get_mut(component_split)
            .expect("split label validated");
        counts.components += 1;
        counts.rows += members.len();
        for &index in members {
            assignments.insert(
                rows[index].row_id.clone(),
                format!("{component_split}:{component_label}"),
            );
        }
    }

    for split in ["train", "validation", "test"] {
        let counts = split_counts
            .get(split)
            .expect("all split counts initialized");
        if counts.rows == 0 || counts.components == 0 {
            return Err(format!("required split {split:?} is empty").into());
        }
    }

    let mut warnings = vec![
        "This check detects only dependency links represented in the selected, reviewed columns; it cannot find missing crosswalk links, mislabeled IDs, or unrecorded source overlap.".to_owned(),
        "Connected-component separation is a structural leakage check, not evidence of model accuracy, causal identification, local efficacy, safety, or regulatory approval.".to_owned(),
    ];
    for (column, stats) in &dependency_key_stats {
        if stats.rows_missing_value > 0 {
            warnings.push(format!(
                "Dependency key {column:?} is missing or placeholder for {} of {} rows; coverage limitations must be reported.",
                stats.rows_missing_value,
                rows.len()
            ));
        }
        if stats.distinct_values == stats.rows_with_value && stats.rows_with_value > 1 {
            warnings.push(format!(
                "Dependency key {column:?} has a unique value for every non-missing row; it may not contribute links, so confirm the key semantics."
            ));
        }
    }

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status: "passed",
        input_label: input_label.to_owned(),
        input_sha256: sha256_hex(bytes),
        row_count: rows.len(),
        component_count: members_by_root.len(),
        dependency_columns: dependency_columns.to_vec(),
        component_assignment_sha256: assignment_digest(&assignments),
        split_counts,
        dependency_key_stats,
        warnings,
        interpretation_boundary: "Multi-key connected-component split audit; structural check only.",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 4 {
        return Err(format!(
            "usage: {} <row-manifest.csv> <report.json> <dependency-columns-comma-separated>",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-dependency-audit")
        )
        .into());
    }
    let dependency_columns = parse_dependency_columns(&arguments[3])?;
    let input = Path::new(&arguments[1]);
    let output = Path::new(&arguments[2]);
    ensure_distinct_paths(input, output)?;
    let bytes = fs::read(input)?;
    let input_label = input
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("row-manifest.csv")
        .to_owned();
    let report = audit(&bytes, &input_label, &dependency_columns)?;
    fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "dependency audit passed: {} rows, {} connected components, dependency keys {}; report: {}",
        report.row_count,
        report.component_count,
        report.dependency_columns.join(","),
        output.display()
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("dependency audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CSV: &str = "row_id,split,study_id,site_id,batch_id
r1,train,study-a,site-1,batch-1
r2,train,study-a,site-2,batch-2
r3,validation,study-b,site-3,batch-3
r4,test,study-c,site-4,batch-4
r5,excluded,study-d,site-5,batch-5
";

    fn selected_keys() -> Vec<String> {
        vec![
            "study_id".to_owned(),
            "site_id".to_owned(),
            "batch_id".to_owned(),
        ]
    }

    #[test]
    fn accepts_disjoint_components_and_reports_component_counts() {
        let report = audit(VALID_CSV.as_bytes(), "fixture.csv", &selected_keys()).unwrap();
        assert_eq!(report.row_count, 5);
        assert_eq!(report.component_count, 4);
        assert_eq!(report.split_counts["train"].rows, 2);
        assert_eq!(report.split_counts["train"].components, 1);
    }

    #[test]
    fn rejects_direct_cross_split_dependency() {
        let csv = "row_id,split,study_id,site_id\nr1,train,study-a,site-1\nr2,test,study-a,site-2\nr3,validation,study-b,site-3\n";
        let keys = vec!["study_id".to_owned(), "site_id".to_owned()];
        assert!(audit(csv.as_bytes(), "bad.csv", &keys).is_err());
    }

    #[test]
    fn rejects_transitive_leakage_across_different_keys() {
        let csv = "row_id,split,study_id,site_id\nr1,train,study-a,site-1\nr2,validation,study-a,site-2\nr3,test,study-b,site-2\n";
        let keys = vec!["study_id".to_owned(), "site_id".to_owned()];
        assert!(audit(csv.as_bytes(), "transitive.csv", &keys).is_err());
    }

    #[test]
    fn rejects_duplicate_row_ids() {
        let csv =
            "row_id,split,study_id\nr1,train,study-a\nr1,test,study-b\nr3,validation,study-c\n";
        let keys = vec!["study_id".to_owned()];
        assert!(audit(csv.as_bytes(), "duplicate.csv", &keys).is_err());
    }

    #[test]
    fn rejects_missing_dependency_column() {
        let csv =
            "row_id,split,study_id\nr1,train,study-a\nr2,validation,study-b\nr3,test,study-c\n";
        let keys = vec!["experiment_id".to_owned()];
        assert!(audit(csv.as_bytes(), "missing.csv", &keys).is_err());
    }

    #[test]
    fn rejects_row_with_no_usable_dependency_identifiers() {
        let csv = "row_id,split,study_id,site_id\nr1,train,unknown,NA\nr2,validation,s2,site-2\nr3,test,s3,site-3\n";
        let keys = vec!["study_id".to_owned(), "site_id".to_owned()];
        assert!(audit(csv.as_bytes(), "unlinked.csv", &keys).is_err());
    }

    #[test]
    fn rejects_identifier_whitespace_aliases() {
        let csv =
            "row_id,split,study_id\nr1,train,study-a\nr2,test, study-a\nr3,validation,study-b\n";
        let keys = vec!["study_id".to_owned()];
        assert!(audit(csv.as_bytes(), "whitespace.csv", &keys).is_err());
    }

    #[test]
    fn rejects_unknown_split_label() {
        let csv =
            "row_id,split,study_id\nr1,train,study-a\nr2,testing,study-b\nr3,validation,study-c\n";
        let keys = vec!["study_id".to_owned()];
        assert!(audit(csv.as_bytes(), "unknown-split.csv", &keys).is_err());
    }

    #[test]
    fn rejects_empty_required_split() {
        let csv = "row_id,split,study_id\nr1,train,study-a\nr2,train,study-b\nr3,test,study-c\n";
        let keys = vec!["study_id".to_owned()];
        assert!(audit(csv.as_bytes(), "empty-split.csv", &keys).is_err());
    }

    #[test]
    fn warns_when_some_dependency_values_are_missing() {
        let csv = "row_id,split,study_id,site_id\nr1,train,study-a,NA\nr2,validation,study-b,site-2\nr3,test,study-c,site-3\n";
        let keys = vec!["study_id".to_owned(), "site_id".to_owned()];
        let report = audit(csv.as_bytes(), "missing-values.csv", &keys).unwrap();
        assert!(
            report
                .warnings
                .iter()
                .any(|warning| warning.contains("1 of 3 rows"))
        );
    }

    #[test]
    fn assignment_digest_is_stable_when_row_order_changes() {
        let csv = "row_id,split,study_id,site_id\nr1,train,study-a,site-1\nr2,train,study-a,site-2\nr3,validation,study-b,site-3\nr4,test,study-c,site-4\n";
        let reordered = "row_id,split,study_id,site_id\nr4,test,study-c,site-4\nr2,train,study-a,site-2\nr1,train,study-a,site-1\nr3,validation,study-b,site-3\n";
        let keys = vec!["study_id".to_owned(), "site_id".to_owned()];
        let first = audit(csv.as_bytes(), "a.csv", &keys).unwrap();
        let second = audit(reordered.as_bytes(), "b.csv", &keys).unwrap();
        assert_eq!(
            first.component_assignment_sha256,
            second.component_assignment_sha256
        );
    }
}
