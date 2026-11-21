//! Structural audit for paired NIR/MIR soil-property datasets.
//!
//! This tool validates CSV identity, reference-property columns, numeric values,
//! spectral axes and region coverage. It does not train a model, claim local
//! efficacy, or establish an independent holdout.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::error::Error;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    profile_version: String,
    dataset_source: DatasetSource,
    sample_id_column: String,
    properties: Vec<PropertySpec>,
    spectral_column_prefix: String,
    spectral_axis_unit: String,
    spectral_axis_descending: bool,
    spectral_regions: Vec<SpectralRegion>,
    expected_nir_to_mir_gap_cm1: Option<f64>,
    spectral_value_semantics: String,
    sample_support: String,
    georeferenced_sample_count: usize,
    georeferenced_sample_count_is_subset: bool,
    notes: Vec<String>,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct DatasetSource {
    article_doi: String,
    zenodo_version_doi: String,
    zenodo_record_id: String,
    version: String,
    filename: String,
    publisher_md5_reference: Option<String>,
    retrieval_sha256: Option<String>,
    expected_sample_rows: Option<usize>,
    license_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PropertySpec {
    column: String,
    unit: String,
    reference_method: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SpectralRegion {
    region_id: String,
    minimum_cm1: f64,
    maximum_cm1: f64,
    instrument: String,
}

#[derive(Clone, Copy, Debug)]
struct SpectralColumn {
    index: usize,
    axis_cm1: f64,
    region_index: Option<usize>,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    profile_version: String,
    source_article_doi: String,
    source_dataset_doi: String,
    source_zenodo_record_id: String,
    source_version: String,
    source_filename: String,
    publisher_md5_reference: Option<String>,
    retrieval_sha256_reference: Option<String>,
    expected_sample_rows: Option<usize>,
    actual_sample_rows: usize,
    input_sha256: String,
    profile_sha256: String,
    column_count: usize,
    sample_id_column: String,
    unique_sample_id_count: usize,
    duplicate_sample_id_count: usize,
    blank_sample_id_count: usize,
    spectral_column_count: usize,
    spectral_axis_unit: String,
    spectral_axis_descending: bool,
    spectral_axis_monotonic: bool,
    spectral_axis_duplicate_count: usize,
    spectral_axis_step_summary: Vec<AxisStepSummary>,
    spectral_regions: Vec<RegionSummary>,
    nir_to_mir_boundary_gap_cm1: Option<f64>,
    property_summaries: Vec<PropertySummary>,
    rows_with_any_spectral_missingness: usize,
    total_missing_spectral_cells: usize,
    spectral_value_semantics: String,
    georeferenced_sample_count: usize,
    georeferenced_sample_count_is_subset: bool,
    sample_support: String,
    profile_notes: Vec<String>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct AxisStepSummary {
    step_cm1: f64,
    count: usize,
}

#[derive(Debug, Serialize)]
struct RegionSummary {
    region_id: String,
    instrument: String,
    column_count: usize,
    first_axis_cm1: Option<f64>,
    last_axis_cm1: Option<f64>,
    minimum_axis_cm1: Option<f64>,
    maximum_axis_cm1: Option<f64>,
    missing_cells: usize,
}

#[derive(Debug, Serialize)]
struct PropertySummary {
    column: String,
    unit: String,
    reference_method: String,
    non_missing_count: usize,
    missing_count: usize,
    minimum: Option<f64>,
    maximum: Option<f64>,
}

struct MutablePropertySummary {
    non_missing_count: usize,
    missing_count: usize,
    minimum: Option<f64>,
    maximum: Option<f64>,
}

struct MutableRegionSummary {
    column_count: usize,
    first_axis_cm1: Option<f64>,
    last_axis_cm1: Option<f64>,
    minimum_axis_cm1: Option<f64>,
    maximum_axis_cm1: Option<f64>,
    missing_cells: usize,
}

const INTERPRETATION_BOUNDARY: &str = "Structural dataset audit only. Not model validation, laboratory qualification, treatment-effect evidence, field efficacy, agronomic advice, fertilizer recommendation, product certification, or regulatory approval.";

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn nonblank(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.trim() != value {
        return Err(format!(
            "{field} must be non-empty with no surrounding whitespace"
        ));
    }
    Ok(())
}

fn parse_finite(value: &str, field: &str) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| format!("{field} is not a numeric value: {value:?}"))?;
    if !parsed.is_finite() {
        return Err(format!("{field} must be finite"));
    }
    Ok(parsed)
}

fn validate_profile(profile: &Profile) -> Result<(), String> {
    for (value, field) in [
        (&profile.profile_version, "profile_version"),
        (
            &profile.dataset_source.article_doi,
            "dataset_source.article_doi",
        ),
        (
            &profile.dataset_source.zenodo_version_doi,
            "dataset_source.zenodo_version_doi",
        ),
        (
            &profile.dataset_source.zenodo_record_id,
            "dataset_source.zenodo_record_id",
        ),
        (&profile.dataset_source.version, "dataset_source.version"),
        (&profile.dataset_source.filename, "dataset_source.filename"),
        (
            &profile.dataset_source.license_status,
            "dataset_source.license_status",
        ),
        (&profile.sample_id_column, "sample_id_column"),
        (&profile.spectral_column_prefix, "spectral_column_prefix"),
        (&profile.spectral_axis_unit, "spectral_axis_unit"),
        (
            &profile.spectral_value_semantics,
            "spectral_value_semantics",
        ),
        (&profile.sample_support, "sample_support"),
    ] {
        nonblank(value, field)?;
    }
    if profile.properties.is_empty() {
        return Err("profile must declare at least one reference property".to_string());
    }
    if profile.spectral_regions.is_empty() {
        return Err("profile must declare at least one spectral region".to_string());
    }
    let mut property_names = HashSet::new();
    for property in &profile.properties {
        nonblank(&property.column, "property.column")?;
        nonblank(&property.unit, "property.unit")?;
        nonblank(&property.reference_method, "property.reference_method")?;
        if !property_names.insert(property.column.as_str()) {
            return Err(format!(
                "duplicate property column in profile: {}",
                property.column
            ));
        }
    }
    let mut region_names = HashSet::new();
    for region in &profile.spectral_regions {
        nonblank(&region.region_id, "spectral_region.region_id")?;
        nonblank(&region.instrument, "spectral_region.instrument")?;
        if !region.minimum_cm1.is_finite()
            || !region.maximum_cm1.is_finite()
            || region.minimum_cm1 >= region.maximum_cm1
        {
            return Err(format!(
                "invalid spectral bounds for region {}",
                region.region_id
            ));
        }
        if !region_names.insert(region.region_id.as_str()) {
            return Err(format!(
                "duplicate spectral region id: {}",
                region.region_id
            ));
        }
    }
    if let Some(expected) = profile.expected_nir_to_mir_gap_cm1 {
        if !expected.is_finite() || expected <= 0.0 {
            return Err("expected_nir_to_mir_gap_cm1 must be finite and positive".to_string());
        }
    }
    if let Some(rows) = profile.dataset_source.expected_sample_rows {
        if rows == 0 {
            return Err("expected_sample_rows must be positive when supplied".to_string());
        }
    }
    if profile.georeferenced_sample_count_is_subset
        && profile
            .dataset_source
            .expected_sample_rows
            .is_some_and(|n| profile.georeferenced_sample_count > n)
    {
        return Err("georeferenced sample count cannot exceed expected sample rows".to_string());
    }
    Ok(())
}

fn parse_spectral_axis(column: &str, prefix: &str) -> Result<Option<f64>, String> {
    let Some(suffix) = column.strip_prefix(prefix) else {
        return Ok(None);
    };
    let axis = suffix
        .parse::<f64>()
        .map_err(|_| format!("spectral column has invalid numeric axis: {column}"))?;
    if !axis.is_finite() || axis <= 0.0 {
        return Err(format!(
            "spectral axis must be finite and positive: {column}"
        ));
    }
    Ok(Some(axis))
}

fn audit_csv_bytes(csv_bytes: &[u8], profile_bytes: &[u8]) -> Result<AuditReport, String> {
    let profile: Profile = serde_json::from_slice(profile_bytes)
        .map_err(|error| format!("invalid profile JSON: {error}"))?;
    validate_profile(&profile)?;

    let mut reader = csv::ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .from_reader(csv_bytes);
    let headers = reader
        .headers()
        .map_err(|error| format!("unable to read CSV header: {error}"))?
        .clone();

    let mut header_indices = BTreeMap::new();
    let mut seen_headers = HashSet::new();
    for (index, header) in headers.iter().enumerate() {
        nonblank(header, "CSV header")?;
        if !seen_headers.insert(header) {
            return Err(format!("duplicate CSV header: {header}"));
        }
        header_indices.insert(header.to_string(), index);
    }

    let sample_id_index = *header_indices
        .get(&profile.sample_id_column)
        .ok_or_else(|| format!("missing sample ID column: {}", profile.sample_id_column))?;

    let mut property_indices = Vec::new();
    for property in &profile.properties {
        let index = *header_indices
            .get(&property.column)
            .ok_or_else(|| format!("missing reference property column: {}", property.column))?;
        property_indices.push(index);
    }

    let mut spectral_columns = Vec::new();
    for (index, header) in headers.iter().enumerate() {
        if let Some(axis_cm1) = parse_spectral_axis(header, &profile.spectral_column_prefix)? {
            let region_index = profile.spectral_regions.iter().position(|region| {
                axis_cm1 >= region.minimum_cm1 && axis_cm1 <= region.maximum_cm1
            });
            spectral_columns.push(SpectralColumn {
                index,
                axis_cm1,
                region_index,
            });
        }
    }
    if spectral_columns.is_empty() {
        return Err("no spectral columns found for configured prefix".to_string());
    }

    let mut axis_monotonic = true;
    let mut axis_duplicate_count = 0usize;
    let mut axis_steps: BTreeMap<String, usize> = BTreeMap::new();
    let mut axes_seen = HashSet::new();
    let mut previous_axis = None;
    for column in &spectral_columns {
        if !axes_seen.insert(column.axis_cm1.to_bits()) {
            axis_duplicate_count += 1;
        }
        if let Some(previous) = previous_axis {
            let difference: f64 = previous - column.axis_cm1;
            let monotonic = if profile.spectral_axis_descending {
                difference > 0.0
            } else {
                difference < 0.0
            };
            if !monotonic {
                axis_monotonic = false;
            }
            let step = difference.abs();
            *axis_steps.entry(format!("{step:.6}")).or_insert(0) += 1;
        }
        previous_axis = Some(column.axis_cm1);
    }
    if !axis_monotonic {
        return Err("spectral axis is not strictly monotonic in declared column order".to_string());
    }
    if axis_duplicate_count > 0 {
        return Err(format!(
            "spectral axis contains {axis_duplicate_count} duplicate values"
        ));
    }

    let mut property_stats: Vec<MutablePropertySummary> = profile
        .properties
        .iter()
        .map(|_| MutablePropertySummary {
            non_missing_count: 0,
            missing_count: 0,
            minimum: None,
            maximum: None,
        })
        .collect();

    let mut region_stats: Vec<MutableRegionSummary> = profile
        .spectral_regions
        .iter()
        .map(|_| MutableRegionSummary {
            column_count: 0,
            first_axis_cm1: None,
            last_axis_cm1: None,
            minimum_axis_cm1: None,
            maximum_axis_cm1: None,
            missing_cells: 0,
        })
        .collect();

    for column in &spectral_columns {
        if let Some(region_index) = column.region_index {
            let stat = &mut region_stats[region_index];
            stat.column_count += 1;
            stat.first_axis_cm1.get_or_insert(column.axis_cm1);
            stat.last_axis_cm1 = Some(column.axis_cm1);
            stat.minimum_axis_cm1 = Some(
                stat.minimum_axis_cm1
                    .map_or(column.axis_cm1, |value| value.min(column.axis_cm1)),
            );
            stat.maximum_axis_cm1 = Some(
                stat.maximum_axis_cm1
                    .map_or(column.axis_cm1, |value| value.max(column.axis_cm1)),
            );
        }
    }

    let mut sample_ids = HashSet::new();
    let mut duplicate_sample_id_count = 0usize;
    let mut blank_sample_id_count = 0usize;
    let mut actual_rows = 0usize;
    let mut rows_with_spectral_missingness = 0usize;
    let mut total_missing_spectral_cells = 0usize;
    let mut unassigned_spectral_axes = Vec::new();

    for column in &spectral_columns {
        if column.region_index.is_none() {
            unassigned_spectral_axes.push(column.axis_cm1);
        }
    }

    for (row_index, record_result) in reader.records().enumerate() {
        let record = record_result.map_err(|error| {
            format!(
                "CSV parse/row-width failure at data row {}: {error}",
                row_index + 1
            )
        })?;
        actual_rows += 1;

        let sample_id = record
            .get(sample_id_index)
            .ok_or_else(|| format!("missing sample ID cell at data row {}", row_index + 1))?;
        if sample_id.trim().is_empty() {
            blank_sample_id_count += 1;
            return Err(format!("blank sample ID at data row {}", row_index + 1));
        }
        if sample_id.trim() != sample_id {
            return Err(format!(
                "sample ID has surrounding whitespace at data row {}",
                row_index + 1
            ));
        }
        if !sample_ids.insert(sample_id.to_string()) {
            duplicate_sample_id_count += 1;
        }

        for (property_index, csv_index) in property_indices.iter().enumerate() {
            let raw = record
                .get(*csv_index)
                .ok_or_else(|| format!("missing property cell at data row {}", row_index + 1))?;
            let stat = &mut property_stats[property_index];
            if raw.trim().is_empty() {
                stat.missing_count += 1;
            } else {
                let value = parse_finite(
                    raw,
                    &format!(
                        "property {} at data row {}",
                        profile.properties[property_index].column,
                        row_index + 1
                    ),
                )?;
                stat.non_missing_count += 1;
                stat.minimum = Some(stat.minimum.map_or(value, |current| current.min(value)));
                stat.maximum = Some(stat.maximum.map_or(value, |current| current.max(value)));
            }
        }

        let mut row_has_missing_spectral = false;
        for column in &spectral_columns {
            let raw = record
                .get(column.index)
                .ok_or_else(|| format!("missing spectral cell at data row {}", row_index + 1))?;
            if raw.trim().is_empty() {
                total_missing_spectral_cells += 1;
                row_has_missing_spectral = true;
                if let Some(region_index) = column.region_index {
                    region_stats[region_index].missing_cells += 1;
                }
            } else {
                let _ = parse_finite(
                    raw,
                    &format!(
                        "spectral value spec.{} at data row {}",
                        column.axis_cm1,
                        row_index + 1
                    ),
                )?;
            }
        }
        if row_has_missing_spectral {
            rows_with_spectral_missingness += 1;
        }
    }

    if actual_rows == 0 {
        return Err("CSV contains no data rows".to_string());
    }
    if duplicate_sample_id_count > 0 {
        return Err(format!(
            "sample identifiers are not unique: {duplicate_sample_id_count} duplicate rows"
        ));
    }

    let mut warnings = Vec::new();
    if profile
        .dataset_source
        .expected_sample_rows
        .is_some_and(|expected| expected != actual_rows)
    {
        warnings.push(format!(
            "expected {} sample rows, observed {actual_rows}",
            profile.dataset_source.expected_sample_rows.unwrap()
        ));
    }
    if profile.spectral_value_semantics == "unresolved_source_description_conflict" {
        warnings.push(
            "source article contains conflicting descriptions of spectral value semantics; preserve raw values and seek source clarification before preprocessing".to_string()
        );
    }
    if profile.georeferenced_sample_count_is_subset && profile.georeferenced_sample_count > 0 {
        warnings.push(format!(
            "only {} of {} samples have released coordinates; do not infer missing locations",
            profile.georeferenced_sample_count, actual_rows
        ));
    }
    if !unassigned_spectral_axes.is_empty() {
        warnings.push(format!(
            "{} spectral axes fall outside the declared NIR/MIR regions",
            unassigned_spectral_axes.len()
        ));
    }
    if rows_with_spectral_missingness > 0 {
        warnings.push(format!(
            "{} rows contain blank spectral cells; missingness must be reviewed before modelling",
            rows_with_spectral_missingness
        ));
    }
    for (index, stat) in region_stats.iter().enumerate() {
        if stat.column_count == 0 {
            warnings.push(format!(
                "declared spectral region {} has no spectral columns",
                profile.spectral_regions[index].region_id
            ));
        }
    }

    let mut boundary_gap = None;
    let nir = profile
        .spectral_regions
        .iter()
        .find(|region| region.region_id == "NIR");
    let mir = profile
        .spectral_regions
        .iter()
        .find(|region| region.region_id == "MIR");
    if let (Some(nir), Some(mir)) = (nir, mir) {
        let nir_edge = spectral_columns
            .iter()
            .filter(|column| {
                column.axis_cm1 >= nir.minimum_cm1 && column.axis_cm1 <= nir.maximum_cm1
            })
            .map(|column| column.axis_cm1)
            .min_by(f64::total_cmp);
        let mir_edge = spectral_columns
            .iter()
            .filter(|column| {
                column.axis_cm1 >= mir.minimum_cm1 && column.axis_cm1 <= mir.maximum_cm1
            })
            .map(|column| column.axis_cm1)
            .max_by(f64::total_cmp);
        if let (Some(nir_edge), Some(mir_edge)) = (nir_edge, mir_edge) {
            boundary_gap = Some(nir_edge - mir_edge);
            if let Some(expected_gap) = profile.expected_nir_to_mir_gap_cm1 {
                if (nir_edge - mir_edge - expected_gap).abs() > 1e-6 {
                    warnings.push(format!(
                        "NIR/MIR boundary gap is {} cm-1; profile expects {} cm-1",
                        nir_edge - mir_edge,
                        expected_gap
                    ));
                }
            }
        }
    }

    let property_summaries = profile
        .properties
        .iter()
        .zip(property_stats)
        .map(|(property, stat)| PropertySummary {
            column: property.column.clone(),
            unit: property.unit.clone(),
            reference_method: property.reference_method.clone(),
            non_missing_count: stat.non_missing_count,
            missing_count: stat.missing_count,
            minimum: stat.minimum,
            maximum: stat.maximum,
        })
        .collect();

    let spectral_regions = profile
        .spectral_regions
        .iter()
        .zip(region_stats)
        .map(|(region, stat)| RegionSummary {
            region_id: region.region_id.clone(),
            instrument: region.instrument.clone(),
            column_count: stat.column_count,
            first_axis_cm1: stat.first_axis_cm1,
            last_axis_cm1: stat.last_axis_cm1,
            minimum_axis_cm1: stat.minimum_axis_cm1,
            maximum_axis_cm1: stat.maximum_axis_cm1,
            missing_cells: stat.missing_cells,
        })
        .collect();

    let axis_step_summary = axis_steps
        .into_iter()
        .map(|(step, count)| AxisStepSummary {
            step_cm1: step.parse::<f64>().expect("formatted axis step is numeric"),
            count,
        })
        .collect();

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status: if warnings.is_empty() {
            "structural_audit_complete_not_model_qualified"
        } else {
            "structural_audit_complete_with_warnings_not_model_qualified"
        },
        profile_version: profile.profile_version,
        source_article_doi: profile.dataset_source.article_doi,
        source_dataset_doi: profile.dataset_source.zenodo_version_doi,
        source_zenodo_record_id: profile.dataset_source.zenodo_record_id,
        source_version: profile.dataset_source.version,
        source_filename: profile.dataset_source.filename,
        publisher_md5_reference: profile.dataset_source.publisher_md5_reference,
        retrieval_sha256_reference: profile.dataset_source.retrieval_sha256,
        expected_sample_rows: profile.dataset_source.expected_sample_rows,
        actual_sample_rows: actual_rows,
        input_sha256: sha256_hex(csv_bytes),
        profile_sha256: sha256_hex(profile_bytes),
        column_count: headers.len(),
        sample_id_column: profile.sample_id_column,
        unique_sample_id_count: sample_ids.len(),
        duplicate_sample_id_count,
        blank_sample_id_count,
        spectral_column_count: spectral_columns.len(),
        spectral_axis_unit: profile.spectral_axis_unit,
        spectral_axis_descending: profile.spectral_axis_descending,
        spectral_axis_monotonic: axis_monotonic,
        spectral_axis_duplicate_count: axis_duplicate_count,
        spectral_axis_step_summary: axis_step_summary,
        spectral_regions,
        nir_to_mir_boundary_gap_cm1: boundary_gap,
        property_summaries,
        rows_with_any_spectral_missingness: rows_with_spectral_missingness,
        total_missing_spectral_cells,
        spectral_value_semantics: profile.spectral_value_semantics,
        georeferenced_sample_count: profile.georeferenced_sample_count,
        georeferenced_sample_count_is_subset: profile.georeferenced_sample_count_is_subset,
        sample_support: profile.sample_support,
        profile_notes: profile.notes,
        warnings,
        interpretation_boundary: INTERPRETATION_BOUNDARY,
    })
}

fn write_new_file(path: &Path, content: &[u8]) -> io::Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(content)?;
    file.write_all(b"\n")?;
    file.sync_all()
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        return Err(format!(
            "usage: {} <dataset.csv> <profile.json> <new-report.json>",
            args.first()
                .map(String::as_str)
                .unwrap_or("spectral-structure-audit")
        )
        .into());
    }
    let input_path = Path::new(&args[1]);
    let profile_path = Path::new(&args[2]);
    let output_path = Path::new(&args[3]);
    if output_path.exists() {
        return Err(format!("refusing to overwrite report: {}", output_path.display()).into());
    }
    let csv_bytes = fs::read(input_path)?;
    let profile_bytes = fs::read(profile_path)?;
    let report = audit_csv_bytes(&csv_bytes, &profile_bytes).map_err(io::Error::other)?;
    let output = serde_json::to_vec_pretty(&report)?;
    write_new_file(output_path, &output)?;
    println!(
        "audited {} samples and {} spectral columns; status={}; input_sha256={}",
        report.actual_sample_rows, report.spectral_column_count, report.status, report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("spectral structure audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CSV: &str = include_str!("../fixtures/synthetic-spectral-data.csv");
    const PROFILE: &str = include_str!("../profiles/western-highveld-v2.json");

    fn synthetic_profile_json() -> String {
        let mut profile: serde_json::Value = serde_json::from_str(PROFILE).unwrap();
        profile["dataset_source"]["expected_sample_rows"] = serde_json::json!(2);
        profile["georeferenced_sample_count"] = serde_json::json!(0);
        serde_json::to_string(&profile).unwrap()
    }

    fn run_fixture(csv: &str) -> Result<AuditReport, String> {
        let profile = synthetic_profile_json();
        audit_csv_bytes(csv.as_bytes(), profile.as_bytes())
    }

    #[test]
    fn synthetic_fixture_has_auditable_structure_and_visible_warnings() {
        let report = run_fixture(CSV).expect("fixture should be structurally auditable");
        assert_eq!(report.actual_sample_rows, 2);
        assert_eq!(report.expected_sample_rows, Some(2));
        assert_eq!(report.spectral_column_count, 6);
        assert_eq!(report.unique_sample_id_count, 2);
        assert_eq!(report.spectral_regions[0].column_count, 3);
        assert_eq!(report.spectral_regions[1].column_count, 3);
        assert_eq!(report.nir_to_mir_boundary_gap_cm1, Some(6.0));
        assert_eq!(report.property_summaries[1].missing_count, 1);
        assert_eq!(
            report.status,
            "structural_audit_complete_with_warnings_not_model_qualified"
        );
    }

    #[test]
    fn duplicate_sample_ids_are_rejected() {
        let duplicate = CSV.replace("synthetic-sample-02", "synthetic-sample-01");
        assert!(run_fixture(&duplicate).is_err());
    }

    #[test]
    fn descending_spectral_axis_is_required() {
        let reordered = CSV.replace(
            "spec.7407,spec.7405,spec.4003",
            "spec.7405,spec.7407,spec.4003",
        );
        assert!(run_fixture(&reordered).is_err());
    }

    #[test]
    fn duplicate_spectral_axis_is_rejected() {
        let duplicate = CSV.replace("spec.7405", "spec.7407");
        assert!(run_fixture(&duplicate).is_err());
    }

    #[test]
    fn nonfinite_spectral_value_is_rejected() {
        let nonfinite = CSV.replace("0.1,0.2,0.3", "0.1,NaN,0.3");
        assert!(run_fixture(&nonfinite).is_err());
    }

    #[test]
    fn missing_reference_property_is_reported_not_imputed() {
        let report = run_fixture(CSV).unwrap();
        let phosphorus = report
            .property_summaries
            .iter()
            .find(|property| property.column == "P (Bray 1)")
            .unwrap();
        assert_eq!(phosphorus.missing_count, 1);
        assert_eq!(phosphorus.non_missing_count, 1);
    }

    #[test]
    fn duplicate_csv_headers_are_rejected() {
        let duplicate = CSV.replace("Sample Name,pH (KCl)", "Sample Name,Sample Name");
        assert!(run_fixture(&duplicate).is_err());
    }

    #[test]
    fn reference_property_units_and_methods_are_pinned_in_profile() {
        let profile: Profile = serde_json::from_str(PROFILE).unwrap();
        validate_profile(&profile).unwrap();
        assert!(profile.properties.iter().any(|property| {
            property.column == "pH (KCl)" && property.reference_method.contains("1 M KCl")
        }));
    }
}
