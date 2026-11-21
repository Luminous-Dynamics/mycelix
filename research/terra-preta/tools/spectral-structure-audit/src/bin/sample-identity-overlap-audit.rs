//! Exact sample-identity overlap auditor for related soil spectral cohorts.
//!
//! This reports exact string matches only. It does not infer semantic identity
//! when source naming schemes differ, and no overlap found is not proof of
//! independent sampling. Raw identifiers are not emitted in the report.

use csv::ReaderBuilder;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::error::Error;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const MATCH_METHOD: &str = "exact_string_match_no_normalization";
const INTERPRETATION_BOUNDARY: &str =
    "Exact string identity overlap only. Does not establish semantic cohort independence when identifiers differ, nor experimental independence among samples; source crosswalk and field/experiment grouping still require review.";

#[derive(Debug, Serialize)]
struct SourceSummary {
    label: String,
    input_sha256: String,
    row_count: usize,
    unique_identifier_count: usize,
    duplicate_identifier_row_count: usize,
}

#[derive(Debug, Serialize)]
struct OverlapReport {
    audit_version: &'static str,
    status: &'static str,
    match_method: &'static str,
    source_a: SourceSummary,
    source_b: SourceSummary,
    overlap_sample_count: usize,
    source_a_only_sample_count: usize,
    source_b_only_sample_count: usize,
    overlap_fraction_of_source_a_unique: f64,
    overlap_fraction_of_source_b_unique: f64,
    overlap_fraction_of_smaller_unique_cohort: f64,
    overlap_identity_sha256: String,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Debug)]
struct Cohort {
    label: String,
    input_sha256: String,
    row_count: usize,
    identifiers: BTreeSet<String>,
    duplicate_identifier_row_count: usize,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn nonblank(value: &str, name: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.trim() != value {
        return Err(format!("{name} must be non-empty and have no surrounding whitespace"));
    }
    Ok(())
}

fn parse_cohort(bytes: &[u8], id_column: &str, label: &str) -> Result<Cohort, String> {
    nonblank(id_column, "sample identifier column")?;
    nonblank(label, "source label")?;

    let mut reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .from_reader(bytes);
    let headers = reader
        .headers()
        .map_err(|error| format!("{label}: unable to read CSV headers: {error}"))?
        .clone();
    let mut seen_headers = BTreeSet::new();
    for header in &headers {
        nonblank(header, &format!("{label}: CSV header"))?;
        if !seen_headers.insert(header.to_string()) {
            return Err(format!("{label}: duplicate CSV header {header}"));
        }
    }
    let id_index = headers
        .iter()
        .position(|header| header == id_column)
        .ok_or_else(|| format!("{label}: sample ID column {id_column:?} not found"))?;

    let mut identifiers = BTreeSet::new();
    let mut row_count = 0usize;
    let mut duplicate_identifier_row_count = 0usize;
    for (index, row) in reader.records().enumerate() {
        let row = row.map_err(|error| format!("{label}: CSV row {} is invalid: {error}", index + 1))?;
        row_count += 1;
        let identifier = row
            .get(id_index)
            .ok_or_else(|| format!("{label}: missing identifier cell at row {}", index + 1))?;
        nonblank(identifier, &format!("{label}: sample ID at row {}", index + 1))?;
        if !identifiers.insert(identifier.to_string()) {
            duplicate_identifier_row_count += 1;
        }
    }
    if row_count == 0 {
        return Err(format!("{label}: cohort has no data rows"));
    }
    Ok(Cohort {
        label: label.to_string(),
        input_sha256: sha256_hex(bytes),
        row_count,
        identifiers,
        duplicate_identifier_row_count,
    })
}

fn audit_bytes(
    source_a_bytes: &[u8],
    source_a_id_column: &str,
    source_a_label: &str,
    source_b_bytes: &[u8],
    source_b_id_column: &str,
    source_b_label: &str,
) -> Result<OverlapReport, String> {
    if source_a_label == source_b_label {
        return Err("source labels must be distinct".to_string());
    }
    let source_a = parse_cohort(source_a_bytes, source_a_id_column, source_a_label)?;
    let source_b = parse_cohort(source_b_bytes, source_b_id_column, source_b_label)?;

    let overlap: Vec<String> = source_a
        .identifiers
        .intersection(&source_b.identifiers)
        .cloned()
        .collect();
    let source_a_only = source_a.identifiers.difference(&source_b.identifiers).count();
    let source_b_only = source_b.identifiers.difference(&source_a.identifiers).count();

    let overlap_fraction_of_a = overlap.len() as f64 / source_a.identifiers.len() as f64;
    let overlap_fraction_of_b = overlap.len() as f64 / source_b.identifiers.len() as f64;
    let smaller = source_a.identifiers.len().min(source_b.identifiers.len());
    let overlap_fraction_of_smaller = overlap.len() as f64 / smaller as f64;

    // Include newline separators to make the canonical membership digest unambiguous.
    let mut canonical = Vec::new();
    for identifier in &overlap {
        canonical.extend_from_slice(identifier.as_bytes());
        canonical.push(b'\n');
    }

    let mut warnings = Vec::new();
    if source_a.duplicate_identifier_row_count > 0 {
        warnings.push(format!(
            "{} contains {} duplicate identifier rows; these rows require grouped treatment",
            source_a.label, source_a.duplicate_identifier_row_count
        ));
    }
    if source_b.duplicate_identifier_row_count > 0 {
        warnings.push(format!(
            "{} contains {} duplicate identifier rows; these rows require grouped treatment",
            source_b.label, source_b.duplicate_identifier_row_count
        ));
    }
    if overlap.is_empty() {
        warnings.push(
            "no exact string overlap was found; this does not prove independent sampling if identifiers were transformed or renamed".to_string(),
        );
    } else {
        warnings.push(
            "overlapping sample identities must remain in one evaluation group across every related release".to_string(),
        );
    }

    Ok(OverlapReport {
        audit_version: AUDIT_VERSION,
        status: if overlap.is_empty() {
            "no_exact_identifier_overlap_detected"
        } else {
            "exact_identifier_overlap_detected"
        },
        match_method: MATCH_METHOD,
        source_a: SourceSummary {
            label: source_a.label,
            input_sha256: source_a.input_sha256,
            row_count: source_a.row_count,
            unique_identifier_count: source_a.identifiers.len(),
            duplicate_identifier_row_count: source_a.duplicate_identifier_row_count,
        },
        source_b: SourceSummary {
            label: source_b.label,
            input_sha256: source_b.input_sha256,
            row_count: source_b.row_count,
            unique_identifier_count: source_b.identifiers.len(),
            duplicate_identifier_row_count: source_b.duplicate_identifier_row_count,
        },
        overlap_sample_count: overlap.len(),
        source_a_only_sample_count: source_a_only,
        source_b_only_sample_count: source_b_only,
        overlap_fraction_of_source_a_unique: overlap_fraction_of_a,
        overlap_fraction_of_source_b_unique: overlap_fraction_of_b,
        overlap_fraction_of_smaller_unique_cohort: overlap_fraction_of_smaller,
        overlap_identity_sha256: sha256_hex(&canonical),
        warnings,
        interpretation_boundary: INTERPRETATION_BOUNDARY,
    })
}

fn write_new_file(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(bytes)?;
    file.write_all(b"\n")?;
    file.sync_all()
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 8 {
        return Err(format!(
            "usage: {} <source-a.csv> <source-a-id-column> <source-a-label> <source-b.csv> <source-b-id-column> <source-b-label> <new-report.json>",
            args.first().map(String::as_str).unwrap_or("sample-identity-overlap-audit")
        ).into());
    }
    let source_a_bytes = fs::read(&args[1])?;
    let source_b_bytes = fs::read(&args[4])?;
    let output_path = Path::new(&args[7]);
    if output_path.exists() {
        return Err(format!("refusing to overwrite report: {}", output_path.display()).into());
    }
    let report = audit_bytes(
        &source_a_bytes,
        &args[2],
        &args[3],
        &source_b_bytes,
        &args[5],
        &args[6],
    )
    .map_err(io::Error::other)?;
    let report_bytes = serde_json::to_vec_pretty(&report)?;
    write_new_file(output_path, &report_bytes)?;
    println!(
        "compared {} unique identifiers with {} unique identifiers; overlap={}; status={}",
        report.source_a.unique_identifier_count,
        report.source_b.unique_identifier_count,
        report.overlap_sample_count,
        report.status
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("sample identity overlap audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &str = "Sample Name,pH\nA-01,5.1\nA-02,5.2\nA-03,5.3\n";
    const B: &str = "Sample ID,NIR\nA-03,0.1\nB-01,0.2\n";

    #[test]
    fn exact_overlap_is_detected_without_exposing_identifiers() {
        let report = audit_bytes(A.as_bytes(), "Sample Name", "2024-MIR", B.as_bytes(), "Sample ID", "2026-NIR-MIR").unwrap();
        assert_eq!(report.overlap_sample_count, 1);
        assert_eq!(report.source_a_only_sample_count, 2);
        assert_eq!(report.source_b_only_sample_count, 1);
        assert_eq!(report.status, "exact_identifier_overlap_detected");
        assert!(!serde_json::to_string(&report).unwrap().contains("A-03"));
        assert_eq!(report.overlap_identity_sha256.len(), 64);
    }

    #[test]
    fn different_column_names_are_supported() {
        let report = audit_bytes(A.as_bytes(), "Sample Name", "source-a", B.as_bytes(), "Sample ID", "source-b").unwrap();
        assert_eq!(report.overlap_sample_count, 1);
    }

    #[test]
    fn no_overlap_is_reported_without_claiming_independence() {
        let no_overlap = "ID,value\nC-01,1\nC-02,2\n";
        let report = audit_bytes(A.as_bytes(), "Sample Name", "source-a", no_overlap.as_bytes(), "ID", "source-c").unwrap();
        assert_eq!(report.overlap_sample_count, 0);
        assert_eq!(report.status, "no_exact_identifier_overlap_detected");
        assert!(report.warnings.iter().any(|warning| warning.contains("does not prove independent")));
    }

    #[test]
    fn duplicate_identifiers_are_reported_not_silently_counted_as_independent() {
        let duplicate = "ID,value\nA-01,1\nA-01,2\nA-02,3\n";
        let report = audit_bytes(duplicate.as_bytes(), "ID", "source-a", B.as_bytes(), "Sample ID", "source-b").unwrap();
        assert_eq!(report.source_a.row_count, 3);
        assert_eq!(report.source_a.unique_identifier_count, 2);
        assert_eq!(report.source_a.duplicate_identifier_row_count, 1);
        assert!(!report.warnings.is_empty());
    }

    #[test]
    fn blank_identifier_is_rejected() {
        let blank = "ID,value\n,1\n";
        assert!(audit_bytes(blank.as_bytes(), "ID", "source-a", B.as_bytes(), "Sample ID", "source-b").is_err());
    }

    #[test]
    fn duplicate_headers_are_rejected() {
        let duplicate = "ID,ID\nA-01,A-01\n";
        assert!(audit_bytes(duplicate.as_bytes(), "ID", "source-a", B.as_bytes(), "Sample ID", "source-b").is_err());
    }

    #[test]
    fn missing_id_column_is_rejected() {
        assert!(audit_bytes(A.as_bytes(), "unknown", "source-a", B.as_bytes(), "Sample ID", "source-b").is_err());
    }

    #[test]
    fn same_source_label_is_rejected() {
        assert!(audit_bytes(A.as_bytes(), "Sample Name", "same", B.as_bytes(), "Sample ID", "same").is_err());
    }
}
