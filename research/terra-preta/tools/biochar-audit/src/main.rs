//! Conservative structural audit of a published biochar CSV.
//!
//! This tool does not fit models, infer treatment effects, or validate agronomic
//! claims. It establishes a reproducible inventory of the exact input artifact,
//! table shape, missingness, potential study/treatment identifiers, exact
//! duplicate rows, and numeric-looking columns before any modelling begins.
//!
//! Usage:
//!   cargo run --release --manifest-path research/terra-preta/tools/biochar-audit/Cargo.toml -- \
//!     BiocharDS_V1.0.csv audit-report.json

use csv::{ReaderBuilder, StringRecord};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const PUBLISHED_REFERENCE_ROWS: usize = 2_439;

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    input_label: String,
    input_sha256: String,
    published_reference_row_count: usize,
    row_count_matches_published_description: bool,
    row_count: usize,
    column_count: usize,
    blank_header_columns: Vec<usize>,
    duplicate_normalized_headers: Vec<HeaderCollision>,
    ragged_rows: usize,
    rows_with_more_fields_than_header: usize,
    rows_with_fewer_fields_than_header: usize,
    exact_duplicate_rows_after_first_occurrence: usize,
    total_missing_cells: usize,
    missing_cell_rate: f64,
    study_group_key_candidate_found: bool,
    study_group_key_has_variation: bool,
    treatment_group_key_candidate_found: bool,
    treatment_group_key_has_variation: bool,
    study_group_candidates: Vec<CandidateColumn>,
    treatment_group_candidates: Vec<CandidateColumn>,
    columns: Vec<ColumnStats>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct CandidateColumn {
    index: usize,
    name: String,
    normalized_name: String,
    unique_non_empty_values: usize,
    missing_cells: usize,
}

#[derive(Debug, Serialize)]
struct HeaderCollision {
    normalized_name: String,
    column_indices: Vec<usize>,
}

#[derive(Debug, Serialize)]
struct ColumnStats {
    index: usize,
    name: String,
    normalized_name: String,
    missing_cells: usize,
    unique_non_empty_values: usize,
    numeric_looking_non_empty_values: usize,
    numeric_looking_rate: f64,
}

#[derive(Debug)]
struct MutableColumnStats {
    missing_cells: usize,
    unique_values: HashSet<String>,
    numeric_looking_non_empty_values: usize,
    non_empty_values: usize,
}

impl MutableColumnStats {
    fn new() -> Self {
        Self {
            missing_cells: 0,
            unique_values: HashSet::new(),
            numeric_looking_non_empty_values: 0,
            non_empty_values: 0,
        }
    }

    fn add(&mut self, value: &str) {
        if value.trim().is_empty() {
            self.missing_cells += 1;
            return;
        }
        self.non_empty_values += 1;
        self.unique_values.insert(value.to_owned());
        if value.trim().parse::<f64>().is_ok_and(f64::is_finite) {
            self.numeric_looking_non_empty_values += 1;
        }
    }
}

/// Normalize a header only for heuristic discovery; original header is retained.
fn normalize_header(value: &str) -> String {
    value
        .chars()
        .filter(|character| character.is_ascii_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

fn looks_like_study_key(normalized: &str) -> bool {
    [
        "study",
        "firstauthor",
        "author",
        "reference",
        "citation",
        "publication",
        "doi",
        "sourcepaper",
        "studyid",
    ]
    .iter()
    .any(|needle| normalized.contains(needle))
}

fn looks_like_treatment_key(normalized: &str) -> bool {
    [
        "treatment",
        "treatmentgroup",
        "treatmenttype",
        "biochartype",
        "applicationtype",
        "amendmenttype",
        "managementpractice",
    ]
    .iter()
    .any(|needle| normalized.contains(needle))
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

/// Hash a row with explicit field-length boundaries to avoid delimiter collisions.
fn row_digest(record: &StringRecord) -> String {
    let mut hasher = Sha256::new();
    hasher.update((record.len() as u64).to_le_bytes());
    for field in record.iter() {
        hasher.update((field.len() as u64).to_le_bytes());
        hasher.update(field.as_bytes());
    }
    let digest = hasher.finalize();
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to a String is infallible");
    }
    output
}

fn audit_bytes(bytes: &[u8], input_label: &str) -> Result<AuditReport, Box<dyn Error>> {
    if bytes.is_empty() {
        return Err("input file is empty".into());
    }
    let mut reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(true)
        .trim(csv::Trim::None)
        .from_reader(bytes);

    let headers = reader.headers()?.clone();
    if headers.is_empty() {
        return Err("CSV has no header columns".into());
    }

    let header_names: Vec<String> = headers.iter().map(str::to_owned).collect();
    let normalized: Vec<String> = header_names
        .iter()
        .map(|header| normalize_header(header))
        .collect();
    let blank_header_columns: Vec<usize> = normalized
        .iter()
        .enumerate()
        .filter(|(_, name)| name.is_empty())
        .map(|(index, _)| index)
        .collect();
    let mut normalized_header_indices = BTreeMap::<String, Vec<usize>>::new();
    for (index, name) in normalized.iter().enumerate() {
        if !name.is_empty() {
            normalized_header_indices
                .entry(name.clone())
                .or_default()
                .push(index);
        }
    }
    let duplicate_normalized_headers: Vec<HeaderCollision> = normalized_header_indices
        .into_iter()
        .filter(|(_, indices)| indices.len() > 1)
        .map(|(name, indices)| HeaderCollision {
            normalized_name: name,
            column_indices: indices,
        })
        .collect();
    let mut column_stats: Vec<MutableColumnStats> = (0..headers.len())
        .map(|_| MutableColumnStats::new())
        .collect();
    let mut duplicate_rows_seen = HashSet::<String>::new();
    let mut duplicate_rows = 0usize;
    let mut row_count = 0usize;
    let mut ragged_rows = 0usize;
    let mut more_fields = 0usize;
    let mut fewer_fields = 0usize;

    for result in reader.records() {
        let record = result?;
        row_count += 1;
        if record.len() != headers.len() {
            ragged_rows += 1;
            if record.len() > headers.len() {
                more_fields += 1;
            } else {
                fewer_fields += 1;
            }
        }

        let digest = row_digest(&record);
        if !duplicate_rows_seen.insert(digest) {
            duplicate_rows += 1;
        }

        for (index, stats) in column_stats.iter_mut().enumerate() {
            stats.add(record.get(index).unwrap_or(""));
        }
    }

    let mut total_missing_cells = 0usize;
    let mut columns = Vec::with_capacity(headers.len());
    for (index, stats) in column_stats.into_iter().enumerate() {
        total_missing_cells += stats.missing_cells;
        let numeric_rate = if stats.non_empty_values == 0 {
            0.0
        } else {
            stats.numeric_looking_non_empty_values as f64 / stats.non_empty_values as f64
        };
        columns.push(ColumnStats {
            index,
            name: header_names[index].clone(),
            normalized_name: normalized[index].clone(),
            missing_cells: stats.missing_cells,
            unique_non_empty_values: stats.unique_values.len(),
            numeric_looking_non_empty_values: stats.numeric_looking_non_empty_values,
            numeric_looking_rate: numeric_rate,
        });
    }

    let total_cells = row_count.saturating_mul(headers.len());
    let missing_rate = if total_cells == 0 {
        0.0
    } else {
        total_missing_cells as f64 / total_cells as f64
    };

    let mut study_candidates = Vec::new();
    let mut treatment_candidates = Vec::new();
    for column in &columns {
        if looks_like_study_key(&column.normalized_name) {
            study_candidates.push(CandidateColumn {
                index: column.index,
                name: column.name.clone(),
                normalized_name: column.normalized_name.clone(),
                unique_non_empty_values: column.unique_non_empty_values,
                missing_cells: column.missing_cells,
            });
        }
        if looks_like_treatment_key(&column.normalized_name) {
            treatment_candidates.push(CandidateColumn {
                index: column.index,
                name: column.name.clone(),
                normalized_name: column.normalized_name.clone(),
                unique_non_empty_values: column.unique_non_empty_values,
                missing_cells: column.missing_cells,
            });
        }
    }

    let mut warnings = Vec::new();
    if !blank_header_columns.is_empty() {
        warnings.push(format!(
            "Columns {:?} have blank/non-identifiable headers; map them manually before analysis.",
            blank_header_columns
        ));
    }
    if !duplicate_normalized_headers.is_empty() {
        warnings.push(format!(
            "{} header collision group(s) remain after punctuation/case normalization. Do not merge or rename columns automatically.",
            duplicate_normalized_headers.len()
        ));
    }
    if row_count != PUBLISHED_REFERENCE_ROWS {
        warnings.push(format!(
            "Observed {row_count} data rows; the paper describes 2,439 CSV rows. Verify the downloaded file/version and whether the published count refers to data rows or spreadsheet rows."
        ));
    }
    if ragged_rows > 0 {
        warnings.push(format!(
            "{ragged_rows} rows have a field count different from the header width."
        ));
    }
    if duplicate_rows > 0 {
        warnings.push(format!(
            "{duplicate_rows} exact duplicate rows occur after their first occurrence; investigate before deciding whether to remove them."
        ));
    }
    let study_group_key_has_variation = study_candidates
        .iter()
        .any(|candidate| candidate.unique_non_empty_values > 1);
    let treatment_group_key_has_variation = treatment_candidates
        .iter()
        .any(|candidate| candidate.unique_non_empty_values > 1);
    if study_candidates.is_empty() {
        warnings.push(
            "No obvious study/source identifier candidate was detected from header names. Do not run random-row validation or claim study-held-out generalization until a reliable grouping key is established."
                .to_owned(),
        );
    } else if !study_group_key_has_variation {
        warnings.push(
            "Study/source key header candidates were detected, but no candidate has more than one distinct non-empty value. This input cannot support study-held-out generalization until a usable group key is established."
                .to_owned(),
        );
    }
    if treatment_candidates.is_empty() {
        warnings.push(
            "No obvious treatment-group identifier candidate was detected from header names. Inspect the data dictionary and establish treatment/comparator semantics before estimating treatment effects."
                .to_owned(),
        );
    } else if !treatment_group_key_has_variation {
        warnings.push(
            "Treatment-group key header candidates were detected, but no candidate has more than one distinct non-empty value. Verify whether this input contains a meaningful treatment comparison."
                .to_owned(),
        );
    }
    if total_missing_cells > 0 {
        warnings.push(format!(
            "{total_missing_cells} cells are blank or whitespace-only. Missingness must be profiled by target, study, treatment, and candidate covariates; do not impute silently."
        ));
    }

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        input_label: input_label.to_owned(),
        input_sha256: sha256_hex(bytes),
        published_reference_row_count: PUBLISHED_REFERENCE_ROWS,
        row_count_matches_published_description: row_count == PUBLISHED_REFERENCE_ROWS,
        row_count,
        column_count: headers.len(),
        blank_header_columns,
        duplicate_normalized_headers,
        ragged_rows,
        rows_with_more_fields_than_header: more_fields,
        rows_with_fewer_fields_than_header: fewer_fields,
        exact_duplicate_rows_after_first_occurrence: duplicate_rows,
        total_missing_cells,
        missing_cell_rate: missing_rate,
        study_group_key_candidate_found: !study_candidates.is_empty(),
        study_group_key_has_variation,
        treatment_group_key_candidate_found: !treatment_candidates.is_empty(),
        treatment_group_key_has_variation,
        study_group_candidates: study_candidates,
        treatment_group_candidates: treatment_candidates,
        columns,
        warnings,
        interpretation_boundary: "Structural CSV audit only. It does not establish causal identifiability, data correctness, agricultural efficacy, material safety, or regulatory approval.",
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let input = args
        .next()
        .ok_or("usage: terra-preta-biochar-audit <input.csv> [report.json]")?;
    let output = args.next();
    if args.next().is_some() {
        return Err("too many arguments; expected input.csv and optional report.json".into());
    }

    let input_path = Path::new(&input);
    let input_bytes = fs::read(input_path)?;
    let input_label = input_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("input.csv")
        .to_owned();
    let report = audit_bytes(&input_bytes, &input_label)?;
    let json = serde_json::to_string_pretty(&report)?;

    if let Some(output_path) = output {
        fs::write(output_path, format!("{json}\n"))?;
    } else {
        println!("{json}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_header_is_deterministic() {
        assert_eq!(normalize_header("Study / Author ID"), "studyauthorid");
    }

    #[test]
    fn identifies_candidate_keys_and_missing_cells() {
        let csv = b"Study ID,Treatment group,Yield\nA,B,1.0\nA,B,\nB,BF,2.0\n";
        let report = audit_bytes(csv, "fixture.csv").unwrap();
        assert_eq!(report.row_count, 3);
        assert_eq!(report.column_count, 3);
        assert!(report.study_group_key_candidate_found);
        assert!(report.study_group_key_has_variation);
        assert!(report.treatment_group_key_candidate_found);
        assert!(report.treatment_group_key_has_variation);
        assert_eq!(report.total_missing_cells, 1);
        assert_eq!(report.columns[2].numeric_looking_non_empty_values, 2);
    }

    #[test]
    fn detects_ragged_rows_without_silently_dropping_them() {
        let csv = b"Study ID,Treatment,Yield\nA,B,1.0\nB,BF\nC,BF,2.0,unexpected\n";
        let report = audit_bytes(csv, "ragged.csv").unwrap();
        assert_eq!(report.row_count, 3);
        assert_eq!(report.ragged_rows, 2);
        assert_eq!(report.rows_with_fewer_fields_than_header, 1);
        assert_eq!(report.rows_with_more_fields_than_header, 1);
    }

    #[test]
    fn detects_exact_duplicate_rows() {
        let csv = b"Study ID,Treatment,Yield\nA,B,1.0\nA,B,1.0\nA,B,1.1\n";
        let report = audit_bytes(csv, "duplicates.csv").unwrap();
        assert_eq!(report.exact_duplicate_rows_after_first_occurrence, 1);
    }

    #[test]
    fn source_hash_is_stable_and_changes_with_input() {
        assert_eq!(sha256_hex(b"abc"), sha256_hex(b"abc"));
        assert_ne!(sha256_hex(b"abc"), sha256_hex(b"abcd"));
    }

    #[test]
    fn empty_input_is_rejected() {
        assert!(audit_bytes(b"", "empty.csv").is_err());
    }

    #[test]
    fn duplicate_headers_remain_visible() {
        let csv = b"Study,study,Yield\nA,A,1\n";
        let report = audit_bytes(csv, "duplicate-headers.csv").unwrap();
        assert_eq!(
            report.columns[0].normalized_name,
            report.columns[1].normalized_name
        );
        assert_ne!(report.columns[0].index, report.columns[1].index);
        assert_eq!(report.duplicate_normalized_headers.len(), 1);
        assert_eq!(
            report.duplicate_normalized_headers[0].column_indices,
            vec![0, 1]
        );
        assert!(
            report
                .warnings
                .iter()
                .any(|warning| warning.contains("header collision"))
        );
    }

    #[test]
    fn blank_study_key_header_is_not_a_usable_group_key() {
        let csv = b"Study ID,Treatment,Yield\n,B,1.0\n,BF,2.0\n";
        let report = audit_bytes(csv, "blank-study-key.csv").unwrap();
        assert!(report.study_group_key_candidate_found);
        assert!(!report.study_group_key_has_variation);
        assert!(
            report
                .warnings
                .iter()
                .any(|warning| warning.contains("cannot support study-held-out"))
        );
    }

    #[test]
    fn no_study_identifier_emits_a_warning() {
        let csv = b"Treatment,Yield\nB,1.0\nBF,2.0\n";
        let report = audit_bytes(csv, "no-study-key.csv").unwrap();
        assert!(!report.study_group_key_candidate_found);
        assert!(!report.study_group_key_has_variation);
        assert!(
            report
                .warnings
                .iter()
                .any(|warning| warning.contains("random-row validation"))
        );
    }
}
