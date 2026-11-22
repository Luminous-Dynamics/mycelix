//! Synthetic registry overlap and double-counting auditor.
//!
//! This module never estimates PyC source attribution. It checks whether a
//! candidate ESM SOC stock claim corrects for already credited biochar CDR
//! applied to the same reviewed spatial-overlap group in the same period.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic accounting-overlap audit only. No PyC source attribution, MRV certification, permanence, additionality, leakage, reversal or lifecycle assessment is performed. The supplied spatial overlap group is trusted as an external reviewed input, not independently intersected by GIS.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    registry_manifest_sha256: String,
    application_records: Vec<ApplicationRecord>,
    registry_entries: Vec<RegistryEntry>,
    soc_claim: SocClaim,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ApplicationRecord {
    application_record_id: String,
    batch_id: String,
    field_id: String,
    spatial_overlap_group_id: String,
    geometry_sha256: String,
    application_date: String,
    applied_biochar_carbon_mg_c: f64,
    batch_passport_sha256: String,
    application_record_sha256: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RegistryEntry {
    registry_entry_id: String,
    carbon_tranche_id: String,
    sink_type: String,
    batch_id: Option<String>,
    batch_passport_sha256: Option<String>,
    field_id: String,
    spatial_overlap_group_id: String,
    geometry_sha256: String,
    application_date: Option<String>,
    claim_period_start: String,
    claim_period_end: String,
    credited_mg_c: f64,
    status: String,
    methodology_id: String,
    methodology_version: String,
    evidence_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SocClaim {
    claim_id: String,
    project_id: String,
    field_id: String,
    spatial_overlap_group_id: String,
    geometry_sha256: String,
    spatial_relationship_status: String,
    period_start: String,
    period_end: String,
    gross_soc_stock_change_mg_c: f64,
    stock_basis: String,
    esm_method_id: String,
    esm_method_version: String,
    esm_reference_fine_earth_mass_mg_ha: f64,
    esm_report_sha256: String,
    declared_biochar_correction_mg_c: f64,
    pyc_evidence: PycEvidence,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PycEvidence {
    status: String,
    measured_total_pyc_mg_c: Option<f64>,
    measurement_method: Option<String>,
    measurement_sha256: Option<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    registry_manifest_sha256: String,
    claim_id: String,
    gross_soc_stock_change_mg_c: f64,
    overlapping_biochar_registry_entry_ids: Vec<String>,
    overlapping_application_record_ids: Vec<String>,
    matched_batch_passport_sha256s: Vec<String>,
    overlapping_biochar_credited_mg_c: f64,
    declared_biochar_correction_mg_c: f64,
    required_minimum_correction_mg_c: f64,
    eligible_residual_soc_cdr_mg_c: Option<f64>,
    eligibility_status: &'static str,
    pyc_evidence_status: String,
    pyc_source_interpretation: &'static str,
    spatial_relationship_status: String,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    batch_registry_match_pass: bool,
    no_duplicate_registry_or_tranche_ids_pass: bool,
    spatial_overlap_group_gate_pass: bool,
    temporal_overlap_gate_pass: bool,
    soc_stock_basis_pass: bool,
    pyc_not_promoted_to_batch_origin_pass: bool,
    correction_floor_pass: bool,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn valid_date(value: &str) -> bool {
    value.len() == 10
        && value.as_bytes()[4] == b'-'
        && value.as_bytes()[7] == b'-'
        && value
            .bytes()
            .enumerate()
            .all(|(i, b)| i == 4 || i == 7 || b.is_ascii_digit())
        && value[0..4]
            .parse::<u32>()
            .is_ok_and(|x| (1..=9999).contains(&x))
        && value[5..7]
            .parse::<u32>()
            .is_ok_and(|x| (1..=12).contains(&x))
        && value[8..10]
            .parse::<u32>()
            .is_ok_and(|x| (1..=31).contains(&x))
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn active_credit(status: &str) -> bool {
    matches!(status, "active" | "retired")
}

fn date_inclusive(date: &str, start: &str, end: &str) -> bool {
    date >= start && date <= end
}

fn intervals_overlap(a_start: &str, a_end: &str, b_start: &str, b_end: &str) -> bool {
    a_start <= b_end && b_start <= a_end
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "carbon-origin-registry-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic accounting scenario".into());
    }
    if s.scenario_id.trim().is_empty() || !valid_sha256(&s.registry_manifest_sha256) {
        return Err("scenario ID or registry manifest digest is invalid".into());
    }
    let claim = &s.soc_claim;
    if claim.claim_id.trim().is_empty()
        || claim.project_id.trim().is_empty()
        || claim.field_id.trim().is_empty()
        || claim.spatial_overlap_group_id.trim().is_empty()
        || claim.esm_method_id.trim().is_empty()
        || claim.esm_method_version.trim().is_empty()
    {
        return Err("SOC claim identity, footprint and ESM method metadata must be present".into());
    }
    if !valid_sha256(&claim.geometry_sha256) || !valid_sha256(&claim.esm_report_sha256) {
        return Err("SOC claim geometry and ESM report digests must be valid SHA-256".into());
    }
    if claim.stock_basis != "equivalent_soil_mass" {
        return Err(
            "only ESM-based stock change is eligible for this v0.1 accounting audit".into(),
        );
    }
    for (name, value) in [
        (
            "gross_soc_stock_change_mg_c",
            claim.gross_soc_stock_change_mg_c,
        ),
        (
            "esm_reference_fine_earth_mass_mg_ha",
            claim.esm_reference_fine_earth_mass_mg_ha,
        ),
        (
            "declared_biochar_correction_mg_c",
            claim.declared_biochar_correction_mg_c,
        ),
    ] {
        finite(name, value)?;
    }
    if claim.esm_reference_fine_earth_mass_mg_ha <= 0.0
        || claim.declared_biochar_correction_mg_c < 0.0
    {
        return Err("ESM reference mass must be positive and correction non-negative".into());
    }
    if !matches!(
        claim.spatial_relationship_status.as_str(),
        "exact_overlap_group_resolved" | "unknown"
    ) {
        return Err("unsupported spatial overlap status".into());
    }
    if !valid_date(&claim.period_start)
        || !valid_date(&claim.period_end)
        || claim.period_start >= claim.period_end
    {
        return Err("SOC claim period must be valid and ordered".into());
    }
    match claim.pyc_evidence.status.as_str() {
        "not_measured" => {
            if claim.pyc_evidence.measured_total_pyc_mg_c.is_some()
                || claim.pyc_evidence.measurement_method.is_some()
                || claim.pyc_evidence.measurement_sha256.is_some()
            {
                return Err("not_measured PyC status cannot carry measurement fields".into());
            }
        }
        "total_pyrogenic_only" | "source_unresolved" => {
            let value = claim
                .pyc_evidence
                .measured_total_pyc_mg_c
                .ok_or_else(|| "PyC measurement needs a total value".to_string())?;
            finite("measured_total_pyc_mg_c", value)?;
            if value < 0.0
                || claim
                    .pyc_evidence
                    .measurement_method
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .is_empty()
                || !claim
                    .pyc_evidence
                    .measurement_sha256
                    .as_deref()
                    .is_some_and(valid_sha256)
            {
                return Err(
                    "PyC evidence needs non-negative value, method and valid digest".into(),
                );
            }
        }
        _ => return Err(
            "v0.1 never accepts source-specific applied-biochar attribution from PyC method alone"
                .into(),
        ),
    }

    let mut app_ids = BTreeSet::new();
    let mut app_keys = BTreeSet::new();
    for app in &s.application_records {
        if app.application_record_id.trim().is_empty()
            || app.batch_id.trim().is_empty()
            || app.field_id.trim().is_empty()
            || app.spatial_overlap_group_id.trim().is_empty()
            || !app_ids.insert(app.application_record_id.as_str())
            || !app_keys.insert((
                app.batch_id.as_str(),
                app.field_id.as_str(),
                app.application_date.as_str(),
            ))
        {
            return Err("application IDs and batch/field/date tuples must be unique".into());
        }
        if !valid_date(&app.application_date)
            || !valid_sha256(&app.geometry_sha256)
            || !valid_sha256(&app.batch_passport_sha256)
            || !valid_sha256(&app.application_record_sha256)
        {
            return Err(format!(
                "application {} has invalid date or digest",
                app.application_record_id
            ));
        }
        finite(
            "applied_biochar_carbon_mg_c",
            app.applied_biochar_carbon_mg_c,
        )?;
        if app.applied_biochar_carbon_mg_c <= 0.0 {
            return Err("applied biochar carbon must be positive".into());
        }
    }

    let mut registry_ids = BTreeSet::new();
    let mut tranche_ids = BTreeSet::new();
    for entry in &s.registry_entries {
        if entry.registry_entry_id.trim().is_empty()
            || entry.carbon_tranche_id.trim().is_empty()
            || entry.field_id.trim().is_empty()
            || entry.spatial_overlap_group_id.trim().is_empty()
            || !registry_ids.insert(entry.registry_entry_id.as_str())
            || !tranche_ids.insert(entry.carbon_tranche_id.as_str())
        {
            return Err(
                "registry entry and carbon tranche IDs must be unique and non-empty".into(),
            );
        }
        if !matches!(
            entry.sink_type.as_str(),
            "biochar_batch_cdr" | "soil_soc_stock_cdr"
        ) || !matches!(
            entry.status.as_str(),
            "active" | "retired" | "withdrawn" | "rejected"
        ) {
            return Err("unsupported registry sink type or status".into());
        }
        if !valid_sha256(&entry.geometry_sha256) || !valid_sha256(&entry.evidence_sha256) {
            return Err(format!(
                "registry entry {} has invalid digest",
                entry.registry_entry_id
            ));
        }
        if !valid_date(&entry.claim_period_start)
            || !valid_date(&entry.claim_period_end)
            || entry.claim_period_start > entry.claim_period_end
        {
            return Err(format!(
                "registry entry {} has invalid period",
                entry.registry_entry_id
            ));
        }
        finite("credited_mg_c", entry.credited_mg_c)?;
        if entry.credited_mg_c <= 0.0
            || entry.methodology_id.trim().is_empty()
            || entry.methodology_version.trim().is_empty()
        {
            return Err(format!(
                "registry entry {} has invalid amount or method identity",
                entry.registry_entry_id
            ));
        }
        if entry.sink_type == "biochar_batch_cdr" {
            let batch = entry.batch_id.as_deref().unwrap_or("");
            let app_date = entry.application_date.as_deref().unwrap_or("");
            if batch.is_empty() || !valid_date(app_date) {
                return Err(format!(
                    "biochar registry entry {} needs batch and application date",
                    entry.registry_entry_id
                ));
            }
            let app = s
                .application_records
                .iter()
                .find(|a| {
                    a.batch_id == batch
                        && a.field_id == entry.field_id
                        && a.application_date == app_date
                })
                .ok_or_else(|| {
                    format!(
                        "biochar registry entry {} has no matching batch application record",
                        entry.registry_entry_id
                    )
                })?;
            if app.spatial_overlap_group_id != entry.spatial_overlap_group_id
                || app.geometry_sha256 != entry.geometry_sha256
                || entry.batch_passport_sha256.as_deref()
                    != Some(app.batch_passport_sha256.as_str())
            {
                return Err(format!(
                    "biochar registry entry {} footprint or batch passport differs from its application record",
                    entry.registry_entry_id
                ));
            }
            let total_for_batch: f64 = s
                .registry_entries
                .iter()
                .filter(|x| {
                    x.sink_type == "biochar_batch_cdr"
                        && x.batch_id.as_deref() == Some(batch)
                        && x.field_id == entry.field_id
                        && active_credit(&x.status)
                })
                .map(|x| x.credited_mg_c)
                .sum();
            if total_for_batch > app.applied_biochar_carbon_mg_c + 1e-9 {
                return Err(format!(
                    "credited biochar carbon for batch {} exceeds applied carbon in field {}",
                    batch, entry.field_id
                ));
            }
        } else if entry.batch_id.is_some()
            || entry.batch_passport_sha256.is_some()
            || entry.application_date.is_some()
        {
            return Err(
                "soil SOC registry entries must not carry batch application identity".into(),
            );
        }
    }
    Ok(())
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let claim = &s.soc_claim;
    let mut overlap_ids = Vec::new();
    let mut overlap_application_ids = Vec::new();
    let mut matched_batch_passports = Vec::new();
    let mut overlap_amount = 0.0;
    let mut duplicate_soc_overlap = false;

    for entry in &s.registry_entries {
        if !active_credit(&entry.status)
            || entry.spatial_overlap_group_id != claim.spatial_overlap_group_id
            || !intervals_overlap(
                &entry.claim_period_start,
                &entry.claim_period_end,
                &claim.period_start,
                &claim.period_end,
            )
        {
            continue;
        }
        match entry.sink_type.as_str() {
            "biochar_batch_cdr" => {
                let applied_on = entry.application_date.as_deref().unwrap_or("");
                if date_inclusive(applied_on, &claim.period_start, &claim.period_end) {
                    overlap_ids.push(entry.registry_entry_id.clone());
                    if let Some(app) = s.application_records.iter().find(|app| {
                        Some(app.batch_id.as_str()) == entry.batch_id.as_deref()
                            && app.field_id == entry.field_id
                            && Some(app.application_date.as_str())
                                == entry.application_date.as_deref()
                    }) {
                        overlap_application_ids.push(app.application_record_id.clone());
                        matched_batch_passports.push(app.batch_passport_sha256.clone());
                    }
                    overlap_amount += entry.credited_mg_c;
                }
            }
            "soil_soc_stock_cdr" => {
                duplicate_soc_overlap = true;
            }
            _ => {}
        }
    }
    overlap_ids.sort();
    overlap_application_ids.sort();
    matched_batch_passports.sort();
    matched_batch_passports.dedup();

    let spatial_resolved = claim.spatial_relationship_status == "exact_overlap_group_resolved";
    let correction_floor_pass = claim.declared_biochar_correction_mg_c + 1e-9 >= overlap_amount;
    let (eligibility_status, eligible) = if !spatial_resolved {
        ("blocked_unknown_spatial_relationship", None)
    } else if duplicate_soc_overlap {
        ("blocked_overlapping_soc_claim", None)
    } else if !correction_floor_pass {
        ("blocked_under_correction", None)
    } else {
        let residual =
            (claim.gross_soc_stock_change_mg_c - claim.declared_biochar_correction_mg_c).max(0.0);
        if residual <= 1e-12 {
            ("zero_eligible_after_correction", Some(0.0))
        } else {
            ("qualified_accounting_residual", Some(residual))
        }
    };

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        registry_manifest_sha256: s.registry_manifest_sha256.clone(),
        claim_id: claim.claim_id.clone(),
        gross_soc_stock_change_mg_c: claim.gross_soc_stock_change_mg_c,
        overlapping_biochar_registry_entry_ids: overlap_ids,
        overlapping_application_record_ids: overlap_application_ids,
        matched_batch_passport_sha256s: matched_batch_passports,
        overlapping_biochar_credited_mg_c: overlap_amount,
        declared_biochar_correction_mg_c: claim.declared_biochar_correction_mg_c,
        required_minimum_correction_mg_c: overlap_amount,
        eligible_residual_soc_cdr_mg_c: eligible,
        eligibility_status,
        pyc_evidence_status: claim.pyc_evidence.status.clone(),
        pyc_source_interpretation: "pyc_measurement_does_not_identify_applied_batch_origin",
        spatial_relationship_status: claim.spatial_relationship_status.clone(),
        controls: Controls {
            batch_registry_match_pass: true,
            no_duplicate_registry_or_tranche_ids_pass: true,
            spatial_overlap_group_gate_pass: spatial_resolved,
            temporal_overlap_gate_pass: true,
            soc_stock_basis_pass: true,
            pyc_not_promoted_to_batch_origin_pass: true,
            correction_floor_pass,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output already exists").into());
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let output_bytes = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&output_bytes)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: carbon-origin-registry-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("carbon-origin registry audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-carbon-origin-registry.json"
        ))
        .expect("synthetic fixture parses")
    }

    #[test]
    fn registered_biochar_is_subtracted_from_soc_cdr_residual() {
        let s = scenario();
        validate(&s).expect("valid synthetic registry");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.overlapping_biochar_credited_mg_c, 8.0);
        assert_eq!(report.eligible_residual_soc_cdr_mg_c, Some(4.0));
        assert_eq!(report.eligibility_status, "qualified_accounting_residual");
    }

    #[test]
    fn total_pyc_measurement_does_not_change_batch_source_attribution_or_correction() {
        let mut s = scenario();
        s.soc_claim.pyc_evidence.measured_total_pyc_mg_c = Some(999.0);
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.eligible_residual_soc_cdr_mg_c, Some(4.0));
        assert_eq!(
            r.pyc_source_interpretation,
            "pyc_measurement_does_not_identify_applied_batch_origin"
        );
    }

    #[test]
    fn under_correction_blocks_candidate_soc_claim() {
        let mut s = scenario();
        s.soc_claim.declared_biochar_correction_mg_c = 2.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.eligibility_status, "blocked_under_correction");
        assert!(r.eligible_residual_soc_cdr_mg_c.is_none());
    }

    #[test]
    fn correction_equal_to_or_above_gross_stock_change_leaves_zero_residual() {
        let mut s = scenario();
        s.soc_claim.declared_biochar_correction_mg_c = 12.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.eligibility_status, "zero_eligible_after_correction");
        assert_eq!(r.eligible_residual_soc_cdr_mg_c, Some(0.0));
    }

    #[test]
    fn unknown_spatial_relationship_abstains() {
        let mut s = scenario();
        s.soc_claim.spatial_relationship_status = "unknown".into();
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.eligibility_status, "blocked_unknown_spatial_relationship");
        assert!(r.eligible_residual_soc_cdr_mg_c.is_none());
    }

    #[test]
    fn retired_registry_credit_still_counts_for_double_counting() {
        let mut s = scenario();
        s.registry_entries[0].status = "retired".into();
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.overlapping_biochar_credited_mg_c, 8.0);
    }

    #[test]
    fn duplicate_registry_entry_or_tranche_is_rejected() {
        let mut s = scenario();
        s.registry_entries.push(s.registry_entries[0].clone());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn credited_carbon_cannot_exceed_applied_batch_carbon() {
        let mut s = scenario();
        s.registry_entries[0].credited_mg_c = 12.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn overlapping_existing_soc_claim_blocks_second_soc_claim() {
        let mut s = scenario();
        let mut old = s.registry_entries[0].clone();
        old.registry_entry_id = "registry-soc-001".into();
        old.carbon_tranche_id = "tranche-soc-001".into();
        old.sink_type = "soil_soc_stock_cdr".into();
        old.batch_id = None;
        old.application_date = None;
        old.credited_mg_c = 2.0;
        s.registry_entries.push(old);
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.eligibility_status, "blocked_overlapping_soc_claim");
    }

    #[test]
    fn batch_passport_digest_must_match_registry_and_application_records() {
        let mut s = scenario();
        s.registry_entries[0].batch_passport_sha256 =
            Some("9999999999999999999999999999999999999999999999999999999999999999".into());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn missing_application_record_for_registry_batch_is_rejected() {
        let mut s = scenario();
        s.application_records.clear();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn date_outside_claim_period_is_not_counted_as_incremental_overlap() {
        let mut s = scenario();
        s.application_records[0].application_date = "2025-06-01".into();
        s.registry_entries[0].application_date = Some("2025-06-01".into());
        s.soc_claim.declared_biochar_correction_mg_c = 0.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.overlapping_biochar_credited_mg_c, 0.0);
        assert_eq!(r.eligible_residual_soc_cdr_mg_c, Some(12.0));
    }
}
