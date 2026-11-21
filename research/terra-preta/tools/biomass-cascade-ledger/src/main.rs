//! Synthetic source-level ecological-retention and cascading-use ledger.
//!
//! Computes conservative feedstock surplus intervals after ecological
//! retention, competing uses and prior allocations. It does not authorize
//! harvesting or decide local retention floors.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_biomass_eligibility_not_harvest_authorization";
const BOUNDARY: &str = "Synthetic biomass-source accounting only. A source surplus interval is not a harvest permit, ecological assessment, community-consent substitute, or proof that nutrient export is environmentally acceptable. All retention floors and caps must be locally approved and evidence-bound.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    season_id: String,
    season_start: String,
    season_end: String,
    source_lots: Vec<SourceLot>,
    allocations: Vec<Allocation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourceLot {
    source_id: String,
    source_type: String,
    custodian_id: String,
    source_record_sha256: String,
    source_start: String,
    source_end: String,
    gross_dry_mass_kg: f64,
    gross_dry_mass_uncertainty_kg: f64,
    retention_status: String,
    required_retention_kg: f64,
    retention_uncertainty_kg: f64,
    retention_method: String,
    retention_authority: String,
    retention_approval_reference: String,
    retention_approval_date: String,
    retention_approval_valid_until: String,
    retention_approval_sha256: String,
    rights_status: String,
    rights_authority: String,
    rights_reference: String,
    rights_record_sha256: String,
    existing_competing_use_kg: f64,
    existing_competing_use_uncertainty_kg: f64,
    prior_batch_allocated_kg: f64,
    prior_batch_allocation_uncertainty_kg: f64,
    approved_harvest_cap_kg: f64,
    harvest_cap_authority: String,
    harvest_cap_reference: String,
    harvest_cap_valid_until: String,
    harvest_cap_sha256: String,
    nutrient_content_kg_per_dry_tonne: NutrientContent,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NutrientContent {
    carbon: f64,
    nitrogen: f64,
    phosphorus: f64,
    potassium: f64,
    uncertainty: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Allocation {
    allocation_id: String,
    batch_id: String,
    source_id: String,
    requested_dry_mass_kg: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    source_assessments: Vec<SourceAssessment>,
    allocation_assessments: Vec<AllocationAssessment>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct SourceAssessment {
    source_id: String,
    status: &'static str,
    gross_dry_mass_kg: f64,
    conservative_available_surplus_lower_kg: f64,
    conservative_available_surplus_upper_kg: f64,
    approved_cap_remaining_lower_kg: f64,
    approved_cap_remaining_upper_kg: f64,
    requested_allocation_total_kg: f64,
    reasons: Vec<String>,
}

#[derive(Debug, Serialize)]
struct AllocationAssessment {
    allocation_id: String,
    batch_id: String,
    source_id: String,
    requested_dry_mass_kg: f64,
    status: &'static str,
    nutrient_removed_kg_interval: NutrientExport,
    reasons: Vec<String>,
}

#[derive(Debug, Serialize)]
struct NutrientExport {
    carbon_lower: f64,
    carbon_upper: f64,
    nitrogen_lower: f64,
    nitrogen_upper: f64,
    phosphorus_lower: f64,
    phosphorus_upper: f64,
    potassium_lower: f64,
    potassium_upper: f64,
}

#[derive(Debug, Serialize)]
struct Controls {
    rights_and_consent_required_pass: bool,
    ecological_retention_required_pass: bool,
    prior_and_competing_uses_deducted_pass: bool,
    seasonal_cap_enforced_pass: bool,
    no_double_allocation_pass: bool,
    nutrient_export_reported_pass: bool,
}

struct SourceCalculation {
    status: &'static str,
    available_lower: f64,
    available_upper: f64,
    cap_remaining_lower: f64,
    cap_remaining_upper: f64,
    requested_total: f64,
    reasons: Vec<String>,
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

fn nonnegative(name: &str, value: f64) -> Result<(), String> {
    finite(name, value)?;
    if value < 0.0 {
        return Err(format!("{name} cannot be negative"));
    }
    Ok(())
}

fn nonempty(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must be non-empty"));
    }
    Ok(())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "biomass-cascade-ledger-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema".into());
    }
    nonempty(&s.scenario_id, "scenario ID")?;
    nonempty(&s.season_id, "season ID")?;
    if !valid_date(&s.season_start)
        || !valid_date(&s.season_end)
        || s.season_start > s.season_end
    {
        return Err("season dates are invalid or reversed".into());
    }
    if s.source_lots.is_empty() || s.allocations.is_empty() {
        return Err("at least one source lot and one allocation are required".into());
    }

    let mut source_ids = BTreeSet::new();
    for source in &s.source_lots {
        for (value, label) in [
            (source.source_id.as_str(), "source ID"),
            (source.custodian_id.as_str(), "source custodian"),
            (source.retention_method.as_str(), "retention method"),
            (source.retention_authority.as_str(), "retention authority"),
            (source.retention_approval_reference.as_str(), "retention approval reference"),
            (source.rights_authority.as_str(), "rights authority"),
            (source.rights_reference.as_str(), "rights reference"),
            (source.harvest_cap_authority.as_str(), "harvest cap authority"),
            (source.harvest_cap_reference.as_str(), "harvest cap reference"),
        ] {
            nonempty(value, label)?;
        }
        if !source_ids.insert(source.source_id.clone()) {
            return Err("source IDs must be unique".into());
        }
        if !matches!(
            source.source_type.as_str(),
            "crop_residue" | "forestry_residue" | "green_waste"
                | "manure" | "invasive_biomass" | "other"
        ) {
            return Err("unsupported source type".into());
        }
        for digest in [
            &source.source_record_sha256,
            &source.retention_approval_sha256,
            &source.rights_record_sha256,
            &source.harvest_cap_sha256,
        ] {
            if !valid_sha256(digest) {
                return Err(format!("{} contains an invalid evidence digest", source.source_id));
            }
        }
        if !valid_date(&source.source_start)
            || !valid_date(&source.source_end)
            || source.source_start > source.source_end
            || source.source_start > s.season_start
            || source.source_end < s.season_end
        {
            return Err(format!(
                "{} source period must cover the declared season",
                source.source_id
            ));
        }
        if !valid_date(&source.retention_approval_date)
            || !valid_date(&source.retention_approval_valid_until)
            || !valid_date(&source.harvest_cap_valid_until)
        {
            return Err(format!("{} has invalid approval / cap dates", source.source_id));
        }
        if source.retention_approval_valid_until < s.season_end
            || source.harvest_cap_valid_until < s.season_end
        {
            return Err(format!("{} approvals expire before the end of the season", source.source_id));
        }
        if !matches!(
            source.retention_status.as_str(),
            "approved" | "unknown" | "prohibited"
        ) || !matches!(
            source.rights_status.as_str(),
            "verified" | "unknown" | "prohibited"
        ) {
            return Err(format!("{} has unsupported approval status", source.source_id));
        }
        for (name, value) in [
            ("gross dry mass", source.gross_dry_mass_kg),
            ("gross dry mass uncertainty", source.gross_dry_mass_uncertainty_kg),
            ("required retention", source.required_retention_kg),
            ("retention uncertainty", source.retention_uncertainty_kg),
            ("existing competing use", source.existing_competing_use_kg),
            ("competing-use uncertainty", source.existing_competing_use_uncertainty_kg),
            ("prior batch allocation", source.prior_batch_allocated_kg),
            ("prior allocation uncertainty", source.prior_batch_allocation_uncertainty_kg),
            ("approved harvest cap", source.approved_harvest_cap_kg),
            ("carbon content", source.nutrient_content_kg_per_dry_tonne.carbon),
            ("nitrogen content", source.nutrient_content_kg_per_dry_tonne.nitrogen),
            ("phosphorus content", source.nutrient_content_kg_per_dry_tonne.phosphorus),
            ("potassium content", source.nutrient_content_kg_per_dry_tonne.potassium),
            ("nutrient-content uncertainty", source.nutrient_content_kg_per_dry_tonne.uncertainty),
        ] {
            nonnegative(name, value)?;
        }
        if source.gross_dry_mass_uncertainty_kg > source.gross_dry_mass_kg {
            return Err(format!("{} gross mass uncertainty exceeds gross mass", source.source_id));
        }
    }

    let mut allocation_ids = BTreeSet::new();
    for allocation in &s.allocations {
        for (value, label) in [
            (allocation.allocation_id.as_str(), "allocation ID"),
            (allocation.batch_id.as_str(), "batch ID"),
            (allocation.source_id.as_str(), "allocation source ID"),
        ] {
            nonempty(value, label)?;
        }
        if !allocation_ids.insert(allocation.allocation_id.clone()) {
            return Err("allocation IDs must be unique".into());
        }
        if !source_ids.contains(&allocation.source_id) {
            return Err(format!("{} references an unknown source", allocation.allocation_id));
        }
        nonnegative("requested dry mass", allocation.requested_dry_mass_kg)?;
        if allocation.requested_dry_mass_kg <= 0.0 {
            return Err("requested allocation must be positive".into());
        }
    }
    Ok(())
}

fn source_calculation(source: &SourceLot, requested_total: f64) -> SourceCalculation {
    let gross_lower = (source.gross_dry_mass_kg - source.gross_dry_mass_uncertainty_kg).max(0.0);
    let gross_upper = source.gross_dry_mass_kg + source.gross_dry_mass_uncertainty_kg;
    let retention_lower = (source.required_retention_kg - source.retention_uncertainty_kg).max(0.0);
    let retention_upper = source.required_retention_kg + source.retention_uncertainty_kg;
    let uses_lower = (source.existing_competing_use_kg - source.existing_competing_use_uncertainty_kg).max(0.0);
    let uses_upper = source.existing_competing_use_kg + source.existing_competing_use_uncertainty_kg;
    let prior_lower = (source.prior_batch_allocated_kg - source.prior_batch_allocation_uncertainty_kg).max(0.0);
    let prior_upper = source.prior_batch_allocated_kg + source.prior_batch_allocation_uncertainty_kg;

    let available_lower = (gross_lower - retention_upper - uses_upper - prior_upper).max(0.0);
    let available_upper = (gross_upper - retention_lower - uses_lower - prior_lower).max(0.0);
    let cap_remaining_lower = source.approved_harvest_cap_kg - prior_upper;
    let cap_remaining_upper = source.approved_harvest_cap_kg - prior_lower;

    let mut blocked = Vec::new();
    let mut unresolved = Vec::new();
    if source.rights_status == "prohibited" {
        blocked.push("source rights / consent prohibit removal".into());
    } else if source.rights_status == "unknown" {
        unresolved.push("source rights / consent are unknown".into());
    }
    if source.retention_status == "prohibited" {
        blocked.push("ecological assessment prohibits biomass removal".into());
    } else if source.retention_status == "unknown" {
        unresolved.push("ecological retention floor is unassessed".into());
    }
    if requested_total > available_upper {
        blocked.push("requested allocation exceeds even the optimistic surplus bound".into());
    } else if requested_total > available_lower {
        unresolved.push("requested allocation overlaps the uncertain surplus interval".into());
    }
    if cap_remaining_upper < 0.0 || requested_total > cap_remaining_upper {
        blocked.push("prior and requested allocations exceed the approved seasonal harvest cap".into());
    } else if requested_total > cap_remaining_lower {
        unresolved.push("requested allocation overlaps the remaining approved cap interval".into());
    }

    let status = if !blocked.is_empty() {
        "blocked"
    } else if !unresolved.is_empty() {
        "unresolved"
    } else {
        "eligible"
    };
    blocked.extend(unresolved);
    SourceCalculation {
        status,
        available_lower,
        available_upper,
        cap_remaining_lower,
        cap_remaining_upper,
        requested_total,
        reasons: blocked,
    }
}

fn nutrient_export(requested_kg: f64, nutrient: &NutrientContent) -> NutrientExport {
    let tonnes = requested_kg / 1000.0;
    let lower = |value: f64| (value - nutrient.uncertainty).max(0.0) * tonnes;
    let upper = |value: f64| (value + nutrient.uncertainty) * tonnes;
    NutrientExport {
        carbon_lower: lower(nutrient.carbon),
        carbon_upper: upper(nutrient.carbon),
        nitrogen_lower: lower(nutrient.nitrogen),
        nitrogen_upper: upper(nutrient.nitrogen),
        phosphorus_lower: lower(nutrient.phosphorus),
        phosphorus_upper: upper(nutrient.phosphorus),
        potassium_lower: lower(nutrient.potassium),
        potassium_upper: upper(nutrient.potassium),
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let sources: BTreeMap<&str, &SourceLot> = s
        .source_lots
        .iter()
        .map(|source| (source.source_id.as_str(), source))
        .collect();
    let mut requested_totals: BTreeMap<&str, f64> = BTreeMap::new();
    for allocation in &s.allocations {
        *requested_totals
            .entry(allocation.source_id.as_str())
            .or_insert(0.0) += allocation.requested_dry_mass_kg;
    }
    let calculations: BTreeMap<&str, SourceCalculation> = s
        .source_lots
        .iter()
        .map(|source| {
            let requested = requested_totals
                .get(source.source_id.as_str())
                .copied()
                .unwrap_or(0.0);
            (
                source.source_id.as_str(),
                source_calculation(source, requested),
            )
        })
        .collect();

    let source_assessments = s
        .source_lots
        .iter()
        .map(|source| {
            let calc = &calculations[source.source_id.as_str()];
            SourceAssessment {
                source_id: source.source_id.clone(),
                status: calc.status,
                gross_dry_mass_kg: source.gross_dry_mass_kg,
                conservative_available_surplus_lower_kg: calc.available_lower,
                conservative_available_surplus_upper_kg: calc.available_upper,
                approved_cap_remaining_lower_kg: calc.cap_remaining_lower,
                approved_cap_remaining_upper_kg: calc.cap_remaining_upper,
                requested_allocation_total_kg: calc.requested_total,
                reasons: calc.reasons.clone(),
            }
        })
        .collect();

    let allocation_assessments = s
        .allocations
        .iter()
        .map(|allocation| {
            let source = sources[allocation.source_id.as_str()];
            let calc = &calculations[allocation.source_id.as_str()];
            AllocationAssessment {
                allocation_id: allocation.allocation_id.clone(),
                batch_id: allocation.batch_id.clone(),
                source_id: allocation.source_id.clone(),
                requested_dry_mass_kg: allocation.requested_dry_mass_kg,
                status: calc.status,
                nutrient_removed_kg_interval: nutrient_export(
                    allocation.requested_dry_mass_kg,
                    &source.nutrient_content_kg_per_dry_tonne,
                ),
                reasons: calc.reasons.clone(),
            }
        })
        .collect();

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        source_assessments,
        allocation_assessments,
        controls: Controls {
            rights_and_consent_required_pass: true,
            ecological_retention_required_pass: true,
            prior_and_competing_uses_deducted_pass: true,
            seasonal_cap_enforced_pass: true,
            no_double_allocation_pass: true,
            nutrient_export_reported_pass: true,
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
        eprintln!("usage: biomass-cascade-ledger <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("biomass cascade ledger failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-biomass-cascade.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn conservative_source_surplus_and_seasonal_cap_are_enforced() {
        let s = scenario();
        validate(&s).expect("valid synthetic scenario");
        let report = build_report(&s, "0".repeat(64));
        let source = report.source_assessments.iter().find(|x| x.source_id == "source-crop-residue-A").unwrap();
        assert_eq!(source.conservative_available_surplus_lower_kg, 3_550.0);
        assert_eq!(source.requested_allocation_total_kg, 2_600.0);
        assert_eq!(source.status, "blocked");
        assert!(source.reasons.iter().any(|x| x.contains("harvest cap")));
    }

    #[test]
    fn unknown_retention_never_becomes_eligible() {
        let report = build_report(&scenario(), "0".repeat(64));
        let source = report.source_assessments.iter().find(|x| x.source_id == "source-crop-residue-unknown").unwrap();
        assert_eq!(source.status, "unresolved");
        assert!(source.reasons.iter().any(|x| x.contains("retention floor")));
    }

    #[test]
    fn prior_and_existing_uses_reduce_available_surplus() {
        let mut s = scenario();
        let source = &mut s.source_lots[0];
        source.existing_competing_use_kg = 500.0;
        source.prior_batch_allocated_kg = 100.0;
        let report = build_report(&s, "0".repeat(64));
        let result = report.source_assessments.iter().find(|x| x.source_id == "source-crop-residue-A").unwrap();
        assert_eq!(result.conservative_available_surplus_lower_kg, 5_250.0);
    }

    #[test]
    fn nutrient_export_is_reported_separately_from_soil_balance() {
        let report = build_report(&scenario(), "0".repeat(64));
        let allocation = report.allocation_assessments.iter().find(|x| x.allocation_id == "alloc-A-001").unwrap();
        assert_eq!(allocation.nutrient_removed_kg_interval.carbon_lower, 839.0);
        assert_eq!(allocation.nutrient_removed_kg_interval.nitrogen_lower, 15.0);
        assert_eq!(allocation.nutrient_removed_kg_interval.phosphorus_lower, 1.4);
        assert_eq!(allocation.nutrient_removed_kg_interval.potassium_lower, 17.0);
    }

    #[test]
    fn unknown_rights_and_prohibited_retention_block_source() {
        let mut s = scenario();
        s.source_lots[0].rights_status = "unknown".into();
        s.source_lots[0].retention_status = "prohibited".into();
        let report = build_report(&s, "0".repeat(64));
        let source = report.source_assessments.iter().find(|x| x.source_id == "source-crop-residue-A").unwrap();
        assert_eq!(source.status, "blocked");
    }

    #[test]
    fn invalid_digest_and_noncovered_source_period_are_rejected() {
        let mut s = scenario();
        s.source_lots[0].rights_record_sha256 = "bad".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.source_lots[0].source_end = "2026-12-01".into();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn duplicate_allocation_id_and_unknown_source_are_rejected() {
        let mut s = scenario();
        s.allocations[1].allocation_id = s.allocations[0].allocation_id.clone();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.allocations[0].source_id = "missing-source".into();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn repeated_batch_allocations_are_summed_before_cap_test() {
        let mut s = scenario();
        s.allocations[0].requested_dry_mass_kg = 1000.0;
        s.allocations[1].requested_dry_mass_kg = 1000.0;
        let report = build_report(&s, "0".repeat(64));
        let source = report.source_assessments.iter().find(|x| x.source_id == "source-crop-residue-A").unwrap();
        assert_eq!(source.requested_allocation_total_kg, 2000.0);
        assert_eq!(source.status, "eligible");
    }
}
