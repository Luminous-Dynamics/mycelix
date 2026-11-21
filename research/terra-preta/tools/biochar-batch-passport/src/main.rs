//! Synthetic-only batch-level biochar quality passport auditor.
//!
//! The auditor evaluates one manufactured batch against one versioned
//! end-use-specific quality profile. It does not certify a real product,
//! calculate carbon removals, or establish site-specific agronomic suitability.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_product_evidence";
const BOUNDARY: &str = "Synthetic batch passport audit only. A synthetic profile pass is not a product certificate, regulatory determination, safety assessment, carbon-removal credit, or soil/crop/site suitability finding. Carbon removal and agronomic suitability are not assessed by v0.1.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    quality_profile: QualityProfile,
    batch: Batch,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct QualityProfile {
    profile_id: String,
    profile_version: String,
    profile_sha256: String,
    authority: String,
    jurisdiction: String,
    end_use: String,
    status: String,
    effective_from: String,
    effective_until: Option<String>,
    require_accredited_labs: bool,
    non_biogenic_co_processing_allowed: bool,
    minimum_temperature_c: f64,
    minimum_residence_time_min: f64,
    minimum_sampling_increments: usize,
    minimum_retention_sample_volume_l: f64,
    minimum_retention_duration_days: i64,
    analyte_criteria: Vec<AnalyteCriterion>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AnalyteCriterion {
    analyte_id: String,
    unit: String,
    basis: String,
    method_id: String,
    direction: String,
    limit: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Batch {
    batch_id: String,
    production_unit_id: String,
    production_start: String,
    production_end: String,
    intended_end_use: String,
    non_biogenic_co_processed: bool,
    feedstock_lots: Vec<FeedstockLot>,
    process: ProcessRecord,
    mass_accounting: MassAccounting,
    sampling: SamplingRecord,
    laboratory_results: Vec<LabResult>,
    carbon_removal_claim_requested: bool,
    agronomic_suitability_claim_requested: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FeedstockLot {
    lot_id: String,
    feedstock_type: String,
    dry_mass_kg: f64,
    origin_status: String,
    prior_use: String,
    contamination_status: String,
    ecological_retention_status: String,
    source_record_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProcessRecord {
    process_record_sha256: String,
    common_conditions_verified: bool,
    temperature_target_c: f64,
    temperature_min_c: f64,
    temperature_max_c: f64,
    residence_time_min: f64,
    oxygen_control_method: String,
    production_unit_id: String,
    run_count: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MassAccounting {
    dry_feedstock_input_kg: f64,
    biochar_output_kg: f64,
    other_co_product_output_kg: f64,
    mass_accounting_record_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SamplingRecord {
    sampling_plan_id: String,
    sampling_method: String,
    representative_sampling_confirmed: bool,
    homogenized_before_sampling: bool,
    increment_count: usize,
    sample_id: String,
    sample_seal_id: String,
    custody_record_sha256: String,
    laboratory_sample_id: String,
    sample_sha256: String,
    retention_sample_volume_l: f64,
    retention_until: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct LabResult {
    analyte_id: String,
    result_qualifier: String,
    measured_value: Option<f64>,
    unit: String,
    basis: String,
    method_id: String,
    laboratory_id: String,
    accreditation_status: String,
    accreditation_scope: Vec<String>,
    detection_limit: f64,
    quantification_limit: f64,
    expanded_uncertainty: f64,
    replicate_count: usize,
    lab_report_sha256: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    batch_id: String,
    quality_profile_id: String,
    quality_profile_sha256: String,
    feedstock_gate: &'static str,
    process_gate: &'static str,
    sampling_gate: &'static str,
    analytical_gate: &'static str,
    analyte_results: Vec<AnalyteResult>,
    quality_conformity_status: &'static str,
    carbon_removal_status: &'static str,
    agronomic_suitability_status: &'static str,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct AnalyteResult {
    analyte_id: String,
    status: &'static str,
    measured_value: Option<f64>,
    uncertainty_or_censor_bound: Option<f64>,
    limit: f64,
    unit: String,
    reason: String,
}

#[derive(Debug, Serialize)]
struct Controls {
    batch_identity_pass: bool,
    feedstock_traceability_pass: bool,
    process_lineage_pass: bool,
    representative_sampling_pass: bool,
    laboratory_result_binding_pass: bool,
    uncertainty_aware_limit_check_pass: bool,
    claim_separation_pass: bool,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_date(value: &str) -> bool {
    value.len() == 10
        && value.as_bytes()[4] == b'-'
        && value.as_bytes()[7] == b'-'
        && value
            .bytes()
            .enumerate()
            .all(|(i, byte)| i == 4 || i == 7 || byte.is_ascii_digit())
        && value[0..4]
            .parse::<u32>()
            .is_ok_and(|year| (1..=9999).contains(&year))
        && value[5..7]
            .parse::<u32>()
            .is_ok_and(|month| (1..=12).contains(&month))
        && value[8..10]
            .parse::<u32>()
            .is_ok_and(|day| (1..=31).contains(&day))
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn required_text(value: &str, name: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{name} must be non-empty"));
    }
    Ok(())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "biochar-batch-passport-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
        || s.quality_profile.status != "synthetic_fixture_not_certification"
    {
        return Err("v0.1 accepts only the declared synthetic schema/profile".into());
    }
    for (value, label) in [
        (s.scenario_id.as_str(), "scenario_id"),
        (s.quality_profile.profile_id.as_str(), "profile_id"),
        (s.quality_profile.profile_version.as_str(), "profile_version"),
        (s.quality_profile.authority.as_str(), "profile authority"),
        (s.quality_profile.jurisdiction.as_str(), "profile jurisdiction"),
        (s.batch.batch_id.as_str(), "batch_id"),
        (s.batch.production_unit_id.as_str(), "production_unit_id"),
    ] {
        required_text(value, label)?;
    }
    if !valid_sha256(&s.quality_profile.profile_sha256) {
        return Err("quality profile SHA-256 is invalid".into());
    }
    if !matches!(
        s.quality_profile.end_use.as_str(),
        "agricultural_soil" | "forest_soil" | "materials" | "other"
    ) || s.batch.intended_end_use != s.quality_profile.end_use
    {
        return Err("batch intended end use does not match the quality profile".into());
    }
    if !valid_date(&s.quality_profile.effective_from)
        || s
            .quality_profile
            .effective_until
            .as_deref()
            .is_some_and(|date| !valid_date(date))
    {
        return Err("quality profile effective date is invalid".into());
    }
    if let Some(until) = &s.quality_profile.effective_until {
        if until < &s.quality_profile.effective_from {
            return Err("quality profile effective interval is reversed".into());
        }
    }
    if s.quality_profile.analyte_criteria.is_empty() {
        return Err("quality profile must define at least one analyte criterion".into());
    }
    for (name, value) in [
        ("minimum_temperature_c", s.quality_profile.minimum_temperature_c),
        (
            "minimum_residence_time_min",
            s.quality_profile.minimum_residence_time_min,
        ),
        (
            "minimum_retention_sample_volume_l",
            s.quality_profile.minimum_retention_sample_volume_l,
        ),
    ] {
        finite(name, value)?;
        if value < 0.0 {
            return Err(format!("{name} cannot be negative"));
        }
    }

    let mut criteria_ids = BTreeSet::new();
    for criterion in &s.quality_profile.analyte_criteria {
        required_text(&criterion.analyte_id, "criterion analyte_id")?;
        required_text(&criterion.unit, "criterion unit")?;
        required_text(&criterion.method_id, "criterion method_id")?;
        if !matches!(criterion.basis.as_str(), "dry_matter") {
            return Err("v0.1 quality criteria must state dry_matter basis".into());
        }
        if !matches!(criterion.direction.as_str(), "maximum" | "minimum") {
            return Err("unsupported analyte comparison direction".into());
        }
        finite("analyte criterion limit", criterion.limit)?;
        if !criteria_ids.insert(criterion.analyte_id.clone()) {
            return Err("quality profile contains duplicate analyte criteria".into());
        }
    }

    if !valid_timestamp(&s.batch.production_start)
        || !valid_timestamp(&s.batch.production_end)
        || s.batch.production_start >= s.batch.production_end
    {
        return Err("batch production timestamps are invalid or reversed".into());
    }
    let batch_date = s.batch.production_start.get(0..10).unwrap_or("");
    if batch_date < s.quality_profile.effective_from.as_str()
        || s
            .quality_profile
            .effective_until
            .as_deref()
            .is_some_and(|until| batch_date > until)
    {
        return Err("batch production lies outside the profile effective interval".into());
    }

    if s.batch.feedstock_lots.is_empty() {
        return Err("at least one feedstock lot is required".into());
    }
    let mut lot_ids = BTreeSet::new();
    let mut lot_mass = 0.0;
    for lot in &s.batch.feedstock_lots {
        required_text(&lot.lot_id, "feedstock lot ID")?;
        required_text(&lot.feedstock_type, "feedstock type")?;
        required_text(&lot.prior_use, "feedstock prior use")?;
        if !lot_ids.insert(lot.lot_id.clone()) {
            return Err("feedstock lot IDs must be unique".into());
        }
        finite("feedstock dry mass", lot.dry_mass_kg)?;
        if lot.dry_mass_kg <= 0.0 || !valid_sha256(&lot.source_record_sha256) {
            return Err("feedstock lot mass or source digest is invalid".into());
        }
        if !matches!(
            lot.origin_status.as_str(),
            "documented" | "unknown" | "restricted"
        ) || !matches!(
            lot.contamination_status.as_str(),
            "screened" | "unknown" | "flagged"
        ) || !matches!(
            lot.ecological_retention_status.as_str(),
            "approved_residual" | "removal_approved" | "unknown" | "not_assessed"
        ) {
            return Err("unsupported feedstock evidence state".into());
        }
        lot_mass += lot.dry_mass_kg;
    }

    let p = &s.batch.process;
    for (name, value) in [
        ("temperature target", p.temperature_target_c),
        ("minimum process temperature", p.temperature_min_c),
        ("maximum process temperature", p.temperature_max_c),
        ("residence time", p.residence_time_min),
        ("dry feedstock input", s.batch.mass_accounting.dry_feedstock_input_kg),
        ("biochar output", s.batch.mass_accounting.biochar_output_kg),
        (
            "other co-product output",
            s.batch.mass_accounting.other_co_product_output_kg,
        ),
    ] {
        finite(name, value)?;
        if value < 0.0 {
            return Err(format!("{name} cannot be negative"));
        }
    }
    if !valid_sha256(&p.process_record_sha256)
        || !valid_sha256(&s.batch.mass_accounting.mass_accounting_record_sha256)
        || !required_text(&p.oxygen_control_method, "oxygen control method").is_ok()
        || p.run_count == 0
    {
        return Err("process record identity is incomplete".into());
    }
    if p.production_unit_id != s.batch.production_unit_id {
        return Err("process and batch production-unit identities differ".into());
    }
    if p.temperature_min_c > p.temperature_target_c
        || p.temperature_target_c > p.temperature_max_c
        || p.temperature_min_c > p.temperature_max_c
    {
        return Err("process temperature trace does not bracket target temperature".into());
    }
    if (lot_mass - s.batch.mass_accounting.dry_feedstock_input_kg).abs()
        > (lot_mass * 0.001).max(0.001)
    {
        return Err("mass input differs from the sum of feedstock lots".into());
    }
    if s.batch.mass_accounting.biochar_output_kg
        + s.batch.mass_accounting.other_co_product_output_kg
        > s.batch.mass_accounting.dry_feedstock_input_kg * 1.001
    {
        return Err("reported dry outputs exceed dry feedstock input beyond tolerance".into());
    }

    let sample = &s.batch.sampling;
    for (value, label) in [
        (sample.sampling_plan_id.as_str(), "sampling plan ID"),
        (sample.sampling_method.as_str(), "sampling method"),
        (sample.sample_id.as_str(), "sample ID"),
        (sample.sample_seal_id.as_str(), "sample seal ID"),
        (sample.laboratory_sample_id.as_str(), "laboratory sample ID"),
    ] {
        required_text(value, label)?;
    }
    if !valid_sha256(&sample.custody_record_sha256)
        || !valid_sha256(&sample.sample_sha256)
        || !valid_date(&sample.retention_until)
    {
        return Err("sampling custody, digest or retention date is invalid".into());
    }
    finite("retention sample volume", sample.retention_sample_volume_l)?;
    if sample.retention_sample_volume_l < 0.0 {
        return Err("retention sample volume cannot be negative".into());
    }

    let mut result_ids = BTreeSet::new();
    for result in &s.batch.laboratory_results {
        required_text(&result.analyte_id, "laboratory analyte ID")?;
        required_text(&result.laboratory_id, "laboratory ID")?;
        required_text(&result.method_id, "laboratory method ID")?;
        required_text(&result.unit, "laboratory result unit")?;
        if !result_ids.insert(result.analyte_id.clone()) {
            return Err("laboratory result analyte IDs must be unique".into());
        }
        if !criteria_ids.contains(&result.analyte_id) {
            return Err("laboratory result contains analyte not present in quality profile".into());
        }
        if !matches!(
            result.result_qualifier.as_str(),
            "quantified" | "below_lod" | "below_loq"
        ) || !matches!(
            result.basis.as_str(),
            "dry_matter" | "as_received" | "other"
        ) || !matches!(
            result.accreditation_status.as_str(),
            "accredited" | "not_accredited" | "unknown"
        ) {
            return Err("unsupported laboratory result state".into());
        }
        for (name, value) in [
            ("detection limit", result.detection_limit),
            ("quantification limit", result.quantification_limit),
            ("expanded uncertainty", result.expanded_uncertainty),
        ] {
            finite(name, value)?;
            if value < 0.0 {
                return Err(format!("{name} cannot be negative"));
            }
        }
        if result.detection_limit > result.quantification_limit {
            return Err("detection limit cannot exceed quantification limit".into());
        }
        if result.replicate_count == 0 || !valid_sha256(&result.lab_report_sha256) {
            return Err("laboratory result replicate count or report digest is invalid".into());
        }
        match result.result_qualifier.as_str() {
            "quantified" => {
                let value = result
                    .measured_value
                    .ok_or_else(|| "quantified result requires measured_value".to_string())?;
                finite("measured value", value)?;
            }
            "below_lod" | "below_loq" if result.measured_value.is_some() => {
                return Err("censored result must not carry a quantified measured_value".into());
            }
            _ => {}
        }
    }

    Ok(())
}

fn valid_timestamp(value: &str) -> bool {
    value.len() >= 20
        && valid_date(value.get(0..10).unwrap_or(""))
        && value.as_bytes().get(10) == Some(&b'T')
        && (value.ends_with('Z') || value.contains('+'))
}

fn gate_feedstock(batch: &Batch) -> bool {
    batch.feedstock_lots.iter().all(|lot| {
        lot.origin_status == "documented"
            && lot.contamination_status == "screened"
            && matches!(
                lot.ecological_retention_status.as_str(),
                "approved_residual" | "removal_approved"
            )
    })
}

fn gate_process(profile: &QualityProfile, batch: &Batch) -> bool {
    let p = &batch.process;
    p.common_conditions_verified
        && p.production_unit_id == batch.production_unit_id
        && p.temperature_min_c >= profile.minimum_temperature_c
        && p.residence_time_min >= profile.minimum_residence_time_min
        && (!batch.non_biogenic_co_processed || profile.non_biogenic_co_processing_allowed)
}

fn gate_sampling(profile: &QualityProfile, batch: &Batch) -> bool {
    let s = &batch.sampling;
    let production_end_date = batch.production_end.get(0..10).unwrap_or("");
    let retention_days = days_between(production_end_date, &s.retention_until);
    s.representative_sampling_confirmed
        && s.homogenized_before_sampling
        && s.increment_count >= profile.minimum_sampling_increments
        && s.retention_sample_volume_l >= profile.minimum_retention_sample_volume_l
        && retention_days.is_some_and(|days| days >= profile.minimum_retention_duration_days)
}

fn days_between(start: &str, end: &str) -> Option<i64> {
    fn ordinal(date: &str) -> Option<i64> {
        if !valid_date(date) {
            return None;
        }
        let year: i64 = date.get(0..4)?.parse().ok()?;
        let month: i64 = date.get(5..7)?.parse().ok()?;
        let day: i64 = date.get(8..10)?.parse().ok()?;
        let adjusted_year = year - i64::from(month <= 2);
        let era = adjusted_year.div_euclid(400);
        let year_of_era = adjusted_year - era * 400;
        let adjusted_month = month + if month > 2 { -3 } else { 9 };
        let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
        let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
        Some(era * 146_097 + day_of_era)
    }
    Some(ordinal(end)? - ordinal(start)?)
}

fn compare_result(
    criterion: &AnalyteCriterion,
    result: Option<&LabResult>,
    require_accredited_labs: bool,
) -> AnalyteResult {
    let Some(result) = result else {
        return AnalyteResult {
            analyte_id: criterion.analyte_id.clone(),
            status: "missing",
            measured_value: None,
            uncertainty_or_censor_bound: None,
            limit: criterion.limit,
            unit: criterion.unit.clone(),
            reason: "required analyte result is missing".into(),
        };
    };

    if result.unit != criterion.unit || result.basis != criterion.basis || result.method_id != criterion.method_id {
        return AnalyteResult {
            analyte_id: criterion.analyte_id.clone(),
            status: "indeterminate",
            measured_value: result.measured_value,
            uncertainty_or_censor_bound: None,
            limit: criterion.limit,
            unit: criterion.unit.clone(),
            reason: "laboratory unit, dry-matter basis or method does not match the profile".into(),
        };
    }
    if require_accredited_labs
        && (result.accreditation_status != "accredited"
            || !result.accreditation_scope.iter().any(|x| x == &criterion.analyte_id))
    {
        return AnalyteResult {
            analyte_id: criterion.analyte_id.clone(),
            status: "indeterminate",
            measured_value: result.measured_value,
            uncertainty_or_censor_bound: None,
            limit: criterion.limit,
            unit: criterion.unit.clone(),
            reason: "laboratory accreditation or analyte scope is insufficient".into(),
        };
    }

    if result.result_qualifier != "quantified" {
        let bound = if result.result_qualifier == "below_lod" {
            result.detection_limit
        } else {
            result.quantification_limit
        };
        let (status, reason) = if criterion.direction == "maximum" && bound <= criterion.limit {
            ("pass", "censoring bound is below the maximum profile limit")
        } else {
            (
                "indeterminate",
                "censored result cannot demonstrate this criterion within its limit",
            )
        };
        return AnalyteResult {
            analyte_id: criterion.analyte_id.clone(),
            status,
            measured_value: None,
            uncertainty_or_censor_bound: Some(bound),
            limit: criterion.limit,
            unit: criterion.unit.clone(),
            reason: reason.into(),
        };
    }

    let value = result.measured_value.unwrap_or(f64::NAN);
    let uncertainty = result.expanded_uncertainty;
    let lower = value - uncertainty;
    let upper = value + uncertainty;
    let (status, reason) = match criterion.direction.as_str() {
        "maximum" if upper <= criterion.limit => ("pass", "upper uncertainty bound is at or below maximum"),
        "maximum" if lower > criterion.limit => ("fail", "lower uncertainty bound exceeds maximum"),
        "maximum" => ("indeterminate", "expanded uncertainty overlaps the maximum limit"),
        "minimum" if lower >= criterion.limit => ("pass", "lower uncertainty bound is at or above minimum"),
        "minimum" if upper < criterion.limit => ("fail", "upper uncertainty bound is below minimum"),
        "minimum" => ("indeterminate", "expanded uncertainty overlaps the minimum limit"),
        _ => ("indeterminate", "unsupported comparison direction"),
    };
    AnalyteResult {
        analyte_id: criterion.analyte_id.clone(),
        status,
        measured_value: Some(value),
        uncertainty_or_censor_bound: Some(uncertainty),
        limit: criterion.limit,
        unit: criterion.unit.clone(),
        reason: reason.into(),
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let feedstock_ok = gate_feedstock(&s.batch);
    let process_ok = gate_process(&s.quality_profile, &s.batch);
    let sampling_ok = gate_sampling(&s.quality_profile, &s.batch);

    let results_by_id: BTreeMap<&str, &LabResult> = s
        .batch
        .laboratory_results
        .iter()
        .map(|result| (result.analyte_id.as_str(), result))
        .collect();

    let analyte_results: Vec<AnalyteResult> = s
        .quality_profile
        .analyte_criteria
        .iter()
        .map(|criterion| {
            compare_result(
                criterion,
                results_by_id.get(criterion.analyte_id.as_str()).copied(),
                s.quality_profile.require_accredited_labs,
            )
        })
        .collect();

    let has_missing = analyte_results.iter().any(|result| result.status == "missing");
    let has_fail = analyte_results.iter().any(|result| result.status == "fail");
    let has_indeterminate = analyte_results
        .iter()
        .any(|result| result.status == "indeterminate");
    let analytical_gate = if has_missing {
        "incomplete"
    } else if has_fail {
        "fail_synthetic"
    } else if has_indeterminate {
        "indeterminate_synthetic"
    } else {
        "pass_synthetic"
    };

    let blocked = !feedstock_ok || !process_ok || !sampling_ok || has_missing;
    let quality_status = if blocked {
        "incomplete_or_blocked"
    } else if has_fail {
        "synthetic_profile_fail"
    } else if has_indeterminate {
        "synthetic_profile_indeterminate"
    } else {
        "synthetic_profile_pass"
    };

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        batch_id: s.batch.batch_id.clone(),
        quality_profile_id: s.quality_profile.profile_id.clone(),
        quality_profile_sha256: s.quality_profile.profile_sha256.clone(),
        feedstock_gate: if feedstock_ok { "pass_synthetic" } else { "blocked" },
        process_gate: if process_ok { "pass_synthetic" } else { "blocked" },
        sampling_gate: if sampling_ok { "pass_synthetic" } else { "blocked" },
        analytical_gate,
        analyte_results,
        quality_conformity_status: quality_status,
        carbon_removal_status: "not_assessed_by_v0_1",
        agronomic_suitability_status: "not_assessed_by_v0_1",
        controls: Controls {
            batch_identity_pass: true,
            feedstock_traceability_pass: feedstock_ok,
            process_lineage_pass: process_ok,
            representative_sampling_pass: sampling_ok,
            laboratory_result_binding_pass: true,
            uncertainty_aware_limit_check_pass: true,
            claim_separation_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(output)?;
    file.write_all(&json)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: biochar-batch-passport <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("biochar batch passport failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-biochar-batch.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn synthetic_fixture_passes_only_its_synthetic_quality_profile() {
        let s = scenario();
        validate(&s).expect("fixture is structurally valid");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.quality_conformity_status, "synthetic_profile_pass");
        assert_eq!(report.carbon_removal_status, "not_assessed_by_v0_1");
        assert_eq!(report.agronomic_suitability_status, "not_assessed_by_v0_1");
        assert_eq!(report.analyte_results.len(), 5);
        assert!(report.analyte_results.iter().all(|x| x.status == "pass"));
    }

    #[test]
    fn unknown_feedstock_contamination_blocks_conformity() {
        let mut s = scenario();
        s.batch.feedstock_lots[0].contamination_status = "unknown".into();
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.feedstock_gate, "blocked");
        assert_eq!(r.quality_conformity_status, "incomplete_or_blocked");
    }

    #[test]
    fn nonrepresentative_sampling_blocks_conformity() {
        let mut s = scenario();
        s.batch.sampling.representative_sampling_confirmed = false;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.sampling_gate, "blocked");
        assert_eq!(r.quality_conformity_status, "incomplete_or_blocked");
    }

    #[test]
    fn uncertainty_overlapping_maximum_is_indeterminate_not_pass() {
        let mut s = scenario();
        let lead = s.batch.laboratory_results.iter_mut()
            .find(|x| x.analyte_id == "lead_mg_kg").unwrap();
        lead.measured_value = Some(99.0);
        lead.expanded_uncertainty = 3.0;
        let r = build_report(&s, "0".repeat(64));
        let lead = r.analyte_results.iter().find(|x| x.analyte_id == "lead_mg_kg").unwrap();
        assert_eq!(lead.status, "indeterminate");
        assert_eq!(r.quality_conformity_status, "synthetic_profile_indeterminate");
    }

    #[test]
    fn confirmed_exceedance_fails_synthetic_profile() {
        let mut s = scenario();
        let lead = s.batch.laboratory_results.iter_mut()
            .find(|x| x.analyte_id == "lead_mg_kg").unwrap();
        lead.measured_value = Some(110.0);
        lead.expanded_uncertainty = 2.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.quality_conformity_status, "synthetic_profile_fail");
    }

    #[test]
    fn missing_required_analyte_is_incomplete_not_pass() {
        let mut s = scenario();
        s.batch.laboratory_results.retain(|x| x.analyte_id != "cadmium_mg_kg");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.analytical_gate, "incomplete");
        assert_eq!(r.quality_conformity_status, "incomplete_or_blocked");
    }

    #[test]
    fn method_unit_or_basis_mismatch_is_indeterminate() {
        let mut s = scenario();
        let lead = s.batch.laboratory_results.iter_mut()
            .find(|x| x.analyte_id == "lead_mg_kg").unwrap();
        lead.basis = "as_received".into();
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(
            r.analyte_results.iter().find(|x| x.analyte_id == "lead_mg_kg").unwrap().status,
            "indeterminate"
        );
    }

    #[test]
    fn below_loq_supports_maximum_only_if_quantification_bound_is_low_enough() {
        let mut s = scenario();
        let pah = s.batch.laboratory_results.iter_mut()
            .find(|x| x.analyte_id == "pah16_mg_kg").unwrap();
        pah.quantification_limit = 8.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(
            r.analyte_results.iter().find(|x| x.analyte_id == "pah16_mg_kg").unwrap().status,
            "indeterminate"
        );
    }

    #[test]
    fn process_and_batch_unit_mismatch_blocks_process_gate() {
        let mut s = scenario();
        s.batch.process.production_unit_id = "OTHER-UNIT".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn profile_hash_and_feedstock_record_hash_are_validated() {
        let mut s = scenario();
        s.quality_profile.profile_sha256 = "unverified".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.batch.feedstock_lots[0].source_record_sha256 = "bad".into();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn output_claims_are_separate_even_when_requested() {
        let s = scenario();
        assert!(s.batch.carbon_removal_claim_requested);
        assert!(s.batch.agronomic_suitability_claim_requested);
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.carbon_removal_status, "not_assessed_by_v0_1");
        assert_eq!(r.agronomic_suitability_status, "not_assessed_by_v0_1");
        assert!(r.controls.claim_separation_pass);
    }

    #[test]
    fn profile_is_end_use_specific_and_production_date_scoped() {
        let mut s = scenario();
        s.batch.intended_end_use = "materials".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.batch.production_start = "2025-12-31T08:00:00Z".into();
        assert!(validate(&t).is_err());
    }
}
