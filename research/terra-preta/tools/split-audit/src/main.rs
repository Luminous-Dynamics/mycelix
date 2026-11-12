//! Audit a frozen row-to-split manifest for group leakage before model fitting.
//!
//! Input CSV headers: row_id,group_id,split
//! Allowed split labels: train, validation, test, excluded.
//!
//! Run this tool separately for each reviewed independence key (e.g. source study,
//! site, material batch). It cannot infer whether supplied identifiers truly
//! represent independent experiments or discover unrecorded related publications.

use csv::ReaderBuilder;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const ALLOWED_SPLITS: [&str; 4] = ["train", "validation", "test", "excluded"];
const PLACEHOLDER_IDS: [&str; 7] = ["", "na", "n/a", "none", "null", "unknown", "missing"];

#[derive(Debug, Deserialize)]
struct ManifestRow {
    row_id: String,
    group_id: String,
    split: String,
}

#[derive(Debug, Serialize, Default)]
struct SplitCounts {
    rows: usize,
    groups: usize,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    input_label: String,
    input_sha256: String,
    grouping_key: String,
    row_count: usize,
    distinct_group_count: usize,
    group_assignment_sha256: String,
    split_counts: BTreeMap<String, SplitCounts>,
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

fn validate_identifier(value: &str, kind: &str, row_number: usize) -> Result<(), Box<dyn Error>> {
    if value.trim() != value {
        return Err(format!("{kind} has leading/trailing whitespace at data row {row_number}; clean and review the source identifier explicitly").into());
    }
    let normalized = value.to_ascii_lowercase();
    if PLACEHOLDER_IDS.contains(&normalized.as_str()) || value == "-" {
        return Err(format!("{kind} is blank or a placeholder at data row {row_number}").into());
    }
    Ok(())
}

fn parse_rows(bytes: &[u8]) -> Result<Vec<ManifestRow>, Box<dyn Error>> {
    if bytes.is_empty() {
        return Err("input file is empty".into());
    }
    let mut reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(bytes);
    let headers = reader.headers()?.clone();
    for required in ["row_id", "group_id", "split"] {
        if !headers.iter().any(|header| header == required) {
            return Err(format!("required CSV column is missing: {required}").into());
        }
    }

    let mut rows = Vec::new();
    for (index, row) in reader.deserialize::<ManifestRow>().enumerate() {
        let row = row.map_err(|error| format!("invalid manifest row {}: {error}", index + 1))?;
        rows.push(row);
    }
    if rows.is_empty() {
        return Err("manifest contains a header but no rows".into());
    }
    Ok(rows)
}

fn group_assignment_digest(assignments: &BTreeMap<String, String>) -> String {
    let mut hasher = Sha256::new();
    for (group_id, split) in assignments {
        hasher.update((group_id.len() as u64).to_le_bytes());
        hasher.update(group_id.as_bytes());
        hasher.update((split.len() as u64).to_le_bytes());
        hasher.update(split.as_bytes());
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
    grouping_key: &str,
) -> Result<AuditReport, Box<dyn Error>> {
    if grouping_key.trim().is_empty() || grouping_key.trim() != grouping_key {
        return Err(
            "grouping-key label must be non-empty and have no leading/trailing whitespace".into(),
        );
    }
    let rows = parse_rows(bytes)?;
    let mut seen_row_ids = HashSet::<String>::new();
    let mut assignments = BTreeMap::<String, String>::new();
    let mut counts = BTreeMap::<String, SplitCounts>::new();
    for split in ALLOWED_SPLITS {
        counts.insert(split.to_owned(), SplitCounts::default());
    }

    for (index, row) in rows.iter().enumerate() {
        let row_number = index + 1;
        validate_identifier(&row.row_id, "row_id", row_number)?;
        validate_identifier(&row.group_id, "group_id", row_number)?;
        if !ALLOWED_SPLITS.contains(&row.split.as_str()) {
            return Err(format!(
                "unknown split {:?} at data row {}; allowed: train, validation, test, excluded",
                row.split, row_number
            )
            .into());
        }
        if !seen_row_ids.insert(row.row_id.clone()) {
            return Err(format!(
                "duplicate row_id {:?}; each measurement row needs a unique identifier",
                row.row_id
            )
            .into());
        }

        match assignments.get(&row.group_id) {
            Some(previous) if previous != &row.split => {
                return Err(format!(
                    "group {:?} leaks across splits: first seen in {:?}, then in {:?}; grouping key is {}",
                    row.group_id, previous, row.split, grouping_key
                ).into());
            }
            Some(_) => {}
            None => {
                assignments.insert(row.group_id.clone(), row.split.clone());
                counts
                    .get_mut(&row.split)
                    .expect("split label was validated")
                    .groups += 1;
            }
        }
        counts
            .get_mut(&row.split)
            .expect("split label was validated")
            .rows += 1;
    }

    for split in ["train", "validation", "test"] {
        let observed = counts.get(split).expect("all splits initialized");
        if observed.rows == 0 || observed.groups == 0 {
            return Err(format!("required split {split:?} is empty").into());
        }
    }

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status: "passed",
        input_label: input_label.to_owned(),
        input_sha256: sha256_hex(bytes),
        grouping_key: grouping_key.to_owned(),
        row_count: rows.len(),
        distinct_group_count: assignments.len(),
        group_assignment_sha256: group_assignment_digest(&assignments),
        split_counts: counts,
        warnings: vec![
            "This audit proves only that the supplied group_id values do not cross split labels; it cannot prove that the grouping key correctly captures related studies, sites, batches, or repeated measurements.".to_owned(),
            "Run once per independently reviewed grouping dimension required by the deployment claim; a study-held-out pass does not imply a site-, batch-, or time-held-out pass.".to_owned(),
            "Do not use this manifest alone to claim predictive performance, causal identification, efficacy, safety, or regulatory approval.".to_owned(),
        ],
        interpretation_boundary: "Structural leakage check on a frozen row-level manifest; not a scientific-independence certificate.",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 4 {
        return Err(format!(
            "usage: {} <row-manifest.csv> <report.json> <reviewed-grouping-key-name>",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-split-audit")
        )
        .into());
    }
    let input = Path::new(&arguments[1]);
    let output = Path::new(&arguments[2]);
    ensure_distinct_paths(input, output)?;
    let bytes = fs::read(input)?;
    let input_label = input
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("row-manifest.csv")
        .to_owned();
    let report = audit(&bytes, &input_label, &arguments[3])?;
    fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "split audit passed: {} rows, {} groups, grouping key {}; report: {}",
        report.row_count,
        report.distinct_group_count,
        report.grouping_key,
        output.display()
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("split audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_CSV: &str = "row_id,group_id,split
row-1,study-a,train
row-2,study-a,train
row-3,study-b,validation
row-4,study-c,test
row-5,study-d,excluded
";

    #[test]
    fn accepts_group_disjoint_manifest_with_repeated_rows_in_one_group() {
        let report = audit(VALID_CSV.as_bytes(), "fixture.csv", "source_study_id").unwrap();
        assert_eq!(report.row_count, 5);
        assert_eq!(report.distinct_group_count, 4);
        assert_eq!(report.split_counts["train"].rows, 2);
        assert_eq!(report.split_counts["train"].groups, 1);
        assert_eq!(report.status, "passed");
    }

    #[test]
    fn rejects_group_leakage_across_train_and_test() {
        let csv =
            "row_id,group_id,split\nr1,study-a,train\nr2,study-a,test\nr3,study-b,validation\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn rejects_duplicate_row_ids() {
        let csv =
            "row_id,group_id,split\nr1,study-a,train\nr1,study-b,validation\nr3,study-c,test\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn rejects_missing_required_column() {
        let csv = "row_id,split\nr1,train\nr2,validation\nr3,test\n";
        assert!(parse_rows(csv.as_bytes()).is_err());
    }

    #[test]
    fn rejects_unknown_split_labels() {
        let csv =
            "row_id,group_id,split\nr1,study-a,training\nr2,study-b,validation\nr3,study-c,test\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn rejects_placeholder_group_ids() {
        let csv =
            "row_id,group_id,split\nr1,unknown,train\nr2,study-b,validation\nr3,study-c,test\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn rejects_identifier_whitespace_instead_of_silently_normalizing() {
        let csv =
            "row_id,group_id,split\nr1,study-a,train\nr2, study-a,validation\nr3,study-c,test\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn rejects_empty_required_split() {
        let csv = "row_id,group_id,split\nr1,study-a,train\nr2,study-b,train\nr3,study-c,test\n";
        assert!(audit(csv.as_bytes(), "bad.csv", "study_id").is_err());
    }

    #[test]
    fn assignment_digest_is_deterministic() {
        let first = audit(VALID_CSV.as_bytes(), "fixture.csv", "source_study_id").unwrap();
        let second = audit(VALID_CSV.as_bytes(), "another-label.csv", "source_study_id").unwrap();
        assert_eq!(first.input_sha256, second.input_sha256);
        assert_eq!(
            first.group_assignment_sha256,
            second.group_assignment_sha256
        );
    }

    #[test]
    fn refuses_to_overwrite_the_input_manifest() {
        let path = std::env::temp_dir().join(format!(
            "terra-preta-split-audit-alias-{}.csv",
            std::process::id()
        ));
        fs::write(&path, "fixture").unwrap();
        let result = ensure_distinct_paths(&path, &path);
        fs::remove_file(&path).unwrap();
        assert!(result.is_err());
    }
}
