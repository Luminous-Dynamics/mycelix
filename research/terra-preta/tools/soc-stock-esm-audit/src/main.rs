//! Synthetic SOC stock accounting: fixed-depth and equivalent soil mass.
//!
//! Fine-earth bulk density is oven-dry fine-earth mass per fine-earth volume.
//! Coarse fragments are represented by their volume fraction of total core
//! volume and applied once. V0.1 retains but does not propagate input SEs.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const ATTRIBUTION: &str = "unresolved_bulk_soc_does_not_identify_native_vs_amendment_carbon";
const UNCERTAINTY: &str = "input_standard_errors_preserved_not_propagated_by_v0_1";
const BOUNDARY: &str = "Synthetic stock accounting only. Fixed-depth and equivalent-soil-mass results are reported separately. Applied amendment carbon is a separate input, not observed retained SOC. Source attribution between native and amendment-derived carbon is unresolved. No net CO2e, permanence, additionality, leakage, issuance or carbon credit is calculated.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    config: Config,
    unit_semantics: UnitSemantics,
    baseline_profile: Profile,
    followup_profile: Profile,
    amendment_inputs: Vec<AmendmentInput>,
    source_attribution_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    fixed_accounting_depth_m: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct UnitSemantics {
    bulk_density_basis: String,
    soc_concentration_unit: String,
    coarse_fragment_basis: String,
    stock_unit: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    profile_id: String,
    plot_id: String,
    measurement_date: String,
    measurement_method: String,
    reference_method_id: String,
    lab_batch_id: String,
    source_sha256: String,
    layers: Vec<Layer>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Layer {
    layer_id: String,
    top_depth_m: f64,
    bottom_depth_m: f64,
    fine_earth_bulk_density_kg_m3: f64,
    coarse_fragment_volume_fraction: f64,
    soc_concentration_g_c_per_kg: f64,
    bulk_density_standard_error_kg_m3: f64,
    coarse_fragment_fraction_standard_error: f64,
    soc_standard_error_g_c_per_kg: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AmendmentInput {
    input_id: String,
    material_name: String,
    applied_date: String,
    dry_matter_mg_per_ha: f64,
    carbon_concentration_g_per_kg_dry_matter: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    plot_id: String,
    baseline_profile_id: String,
    followup_profile_id: String,
    fixed_accounting_depth_m: f64,
    baseline_fixed_depth_stock_mg_c_ha: f64,
    followup_fixed_depth_stock_mg_c_ha: Option<f64>,
    fixed_depth_stock_change_mg_c_ha: Option<f64>,
    baseline_reference_fine_earth_mass_mg_ha: f64,
    baseline_esm_stock_mg_c_ha: f64,
    followup_esm_stock_mg_c_ha: Option<f64>,
    followup_esm_depth_m: Option<f64>,
    esm_stock_change_mg_c_ha: Option<f64>,
    esm_status: &'static str,
    amendment_carbon_input_mg_c_ha: f64,
    source_attribution_status: &'static str,
    uncertainty_status: &'static str,
    controls: Controls,
    claim_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    profile_identity_match_pass: bool,
    chronology_pass: bool,
    method_and_reference_match_pass: bool,
    layer_contiguity_pass: bool,
    unit_semantics_pass: bool,
    no_depth_extrapolation_pass: bool,
    amendment_not_added_to_observed_stock_pass: bool,
}

#[derive(Clone, Copy, Debug)]
struct StockAndMass {
    fine_earth_mass_mg_ha: f64,
    carbon_stock_mg_c_ha: f64,
}

fn finite(name: &str, x: f64) -> Result<(), String> {
    if x.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn valid_date(s: &str) -> bool {
    s.len() == 10
        && s.as_bytes()[4] == b'-'
        && s.as_bytes()[7] == b'-'
        && s.bytes()
            .enumerate()
            .all(|(i, b)| i == 4 || i == 7 || b.is_ascii_digit())
        && s[0..4]
            .parse::<u32>()
            .is_ok_and(|x| (1..=9999).contains(&x))
        && s[5..7].parse::<u32>().is_ok_and(|x| (1..=12).contains(&x))
        && s[8..10].parse::<u32>().is_ok_and(|x| (1..=31).contains(&x))
}

fn valid_sha256(s: &str) -> bool {
    s.len() == 64
        && s.bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn validate_profile(profile: &Profile, accounting_depth: f64) -> Result<(), String> {
    if profile.profile_id.trim().is_empty()
        || profile.plot_id.trim().is_empty()
        || profile.measurement_method.trim().is_empty()
        || profile.reference_method_id.trim().is_empty()
        || profile.lab_batch_id.trim().is_empty()
        || !valid_date(&profile.measurement_date)
        || !valid_sha256(&profile.source_sha256)
        || profile.layers.is_empty()
    {
        return Err(format!(
            "profile {} has invalid identity/provenance",
            profile.profile_id
        ));
    }
    let mut previous_bottom = 0.0_f64;
    let mut ids = BTreeSet::new();
    for layer in &profile.layers {
        if layer.layer_id.trim().is_empty() || !ids.insert(layer.layer_id.clone()) {
            return Err("layer IDs must be non-empty and unique within a profile".into());
        }
        for (name, value) in [
            ("top_depth_m", layer.top_depth_m),
            ("bottom_depth_m", layer.bottom_depth_m),
            (
                "fine_earth_bulk_density_kg_m3",
                layer.fine_earth_bulk_density_kg_m3,
            ),
            (
                "coarse_fragment_volume_fraction",
                layer.coarse_fragment_volume_fraction,
            ),
            (
                "soc_concentration_g_c_per_kg",
                layer.soc_concentration_g_c_per_kg,
            ),
            (
                "bulk_density_standard_error_kg_m3",
                layer.bulk_density_standard_error_kg_m3,
            ),
            (
                "coarse_fragment_fraction_standard_error",
                layer.coarse_fragment_fraction_standard_error,
            ),
            (
                "soc_standard_error_g_c_per_kg",
                layer.soc_standard_error_g_c_per_kg,
            ),
        ] {
            finite(name, value)?;
        }
        if (layer.top_depth_m - previous_bottom).abs() > 1e-9
            || layer.bottom_depth_m <= layer.top_depth_m
            || layer.fine_earth_bulk_density_kg_m3 <= 0.0
            || !(0.0..1.0).contains(&layer.coarse_fragment_volume_fraction)
            || layer.soc_concentration_g_c_per_kg < 0.0
            || layer.bulk_density_standard_error_kg_m3 < 0.0
            || layer.coarse_fragment_fraction_standard_error < 0.0
            || layer.soc_standard_error_g_c_per_kg < 0.0
        {
            return Err(format!(
                "profile {} layer {} has invalid depth or physical values",
                profile.profile_id, layer.layer_id
            ));
        }
        previous_bottom = layer.bottom_depth_m;
    }
    if previous_bottom + 1e-9 < accounting_depth {
        return Err(format!(
            "profile {} does not cover the fixed accounting depth",
            profile.profile_id
        ));
    }
    Ok(())
}

/// kg/m2 × 10 = Mg/ha. Fine-earth density is already per fine-earth
/// volume, so (1 - coarse-fragment volume fraction) is applied once.
fn layer_mass_and_carbon(layer: &Layer, used_thickness_m: f64) -> StockAndMass {
    let mass_mg_ha = layer.fine_earth_bulk_density_kg_m3
        * used_thickness_m
        * (1.0 - layer.coarse_fragment_volume_fraction)
        * 10.0;
    StockAndMass {
        fine_earth_mass_mg_ha: mass_mg_ha,
        carbon_stock_mg_c_ha: mass_mg_ha * layer.soc_concentration_g_c_per_kg / 1000.0,
    }
}

fn stock_to_fixed_depth(profile: &Profile, depth_m: f64) -> Option<StockAndMass> {
    if profile.layers.last()?.bottom_depth_m + 1e-9 < depth_m {
        return None;
    }
    let mut total = StockAndMass {
        fine_earth_mass_mg_ha: 0.0,
        carbon_stock_mg_c_ha: 0.0,
    };
    for layer in &profile.layers {
        if layer.top_depth_m >= depth_m {
            break;
        }
        let used_bottom = layer.bottom_depth_m.min(depth_m);
        let thickness = (used_bottom - layer.top_depth_m).max(0.0);
        let contribution = layer_mass_and_carbon(layer, thickness);
        total.fine_earth_mass_mg_ha += contribution.fine_earth_mass_mg_ha;
        total.carbon_stock_mg_c_ha += contribution.carbon_stock_mg_c_ha;
        if used_bottom >= depth_m {
            break;
        }
    }
    Some(total)
}

fn stock_at_equivalent_mass(profile: &Profile, target_mass_mg_ha: f64) -> Option<(f64, f64)> {
    let mut remaining = target_mass_mg_ha;
    let mut carbon = 0.0;
    for layer in &profile.layers {
        let thickness = layer.bottom_depth_m - layer.top_depth_m;
        let full = layer_mass_and_carbon(layer, thickness);
        if remaining <= full.fine_earth_mass_mg_ha {
            let fraction = remaining / full.fine_earth_mass_mg_ha;
            carbon += full.carbon_stock_mg_c_ha * fraction;
            let reached_depth = layer.top_depth_m + thickness * fraction;
            return Some((carbon, reached_depth));
        }
        carbon += full.carbon_stock_mg_c_ha;
        remaining -= full.fine_earth_mass_mg_ha;
    }
    None
}

fn amendment_carbon_input(inputs: &[AmendmentInput]) -> Result<f64, String> {
    let mut ids = BTreeSet::new();
    let mut total = 0.0;
    for input in inputs {
        if input.input_id.trim().is_empty()
            || !ids.insert(input.input_id.clone())
            || input.material_name.trim().is_empty()
            || !valid_date(&input.applied_date)
        {
            return Err("amendment input identity/date is invalid or duplicated".into());
        }
        finite("amendment dry matter rate", input.dry_matter_mg_per_ha)?;
        finite(
            "amendment C concentration",
            input.carbon_concentration_g_per_kg_dry_matter,
        )?;
        if input.dry_matter_mg_per_ha < 0.0
            || input.carbon_concentration_g_per_kg_dry_matter < 0.0
        {
            return Err("amendment inputs cannot be negative".into());
        }
        total +=
            input.dry_matter_mg_per_ha * input.carbon_concentration_g_per_kg_dry_matter / 1000.0;
    }
    Ok(total)
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soc-stock-esm-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
        || s.scenario_id.trim().is_empty()
    {
        return Err("v0.1 accepts only the declared synthetic scenario/version".into());
    }
    finite(
        "fixed_accounting_depth_m",
        s.config.fixed_accounting_depth_m,
    )?;
    if s.config.fixed_accounting_depth_m <= 0.0 {
        return Err("fixed accounting depth must be positive".into());
    }
    if s.unit_semantics.bulk_density_basis != "oven_dry_fine_earth_mass_per_fine_earth_volume"
        || s.unit_semantics.soc_concentration_unit != "g_C_per_kg_dry_fine_earth"
        || s.unit_semantics.coarse_fragment_basis
            != "coarse_fragment_volume_fraction_of_total_core_volume"
        || s.unit_semantics.stock_unit != "Mg_C_per_ha"
        || s.source_attribution_status != ATTRIBUTION
    {
        return Err("unit semantics or source attribution status do not match v0.1".into());
    }
    validate_profile(&s.baseline_profile, s.config.fixed_accounting_depth_m)?;
    validate_profile(&s.followup_profile, s.config.fixed_accounting_depth_m)?;
    if s.baseline_profile.profile_id == s.followup_profile.profile_id
        || s.baseline_profile.plot_id != s.followup_profile.plot_id
    {
        return Err("baseline and follow-up must be distinct profiles from the same plot".into());
    }
    if s.baseline_profile.measurement_date >= s.followup_profile.measurement_date {
        return Err("follow-up date must be after baseline date".into());
    }
    if s.baseline_profile.measurement_method != s.followup_profile.measurement_method
        || s.baseline_profile.reference_method_id != s.followup_profile.reference_method_id
    {
        return Err("v0.1 requires matched SOC and reference methods between profiles".into());
    }
    amendment_carbon_input(&s.amendment_inputs)?;
    Ok(())
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let baseline = stock_to_fixed_depth(&s.baseline_profile, s.config.fixed_accounting_depth_m)
        .expect("validation requires baseline depth coverage");
    let followup_fixed =
        stock_to_fixed_depth(&s.followup_profile, s.config.fixed_accounting_depth_m);
    let baseline_esm =
        stock_at_equivalent_mass(&s.baseline_profile, baseline.fine_earth_mass_mg_ha)
            .expect("baseline mass matches own soil profile");
    let followup_esm =
        stock_at_equivalent_mass(&s.followup_profile, baseline.fine_earth_mass_mg_ha);
    let amendment_input =
        amendment_carbon_input(&s.amendment_inputs).expect("validation checks amendment inputs");

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        plot_id: s.baseline_profile.plot_id.clone(),
        baseline_profile_id: s.baseline_profile.profile_id.clone(),
        followup_profile_id: s.followup_profile.profile_id.clone(),
        fixed_accounting_depth_m: s.config.fixed_accounting_depth_m,
        baseline_fixed_depth_stock_mg_c_ha: baseline.carbon_stock_mg_c_ha,
        followup_fixed_depth_stock_mg_c_ha: followup_fixed.map(|x| x.carbon_stock_mg_c_ha),
        fixed_depth_stock_change_mg_c_ha: followup_fixed
            .map(|x| x.carbon_stock_mg_c_ha - baseline.carbon_stock_mg_c_ha),
        baseline_reference_fine_earth_mass_mg_ha: baseline.fine_earth_mass_mg_ha,
        baseline_esm_stock_mg_c_ha: baseline_esm.0,
        followup_esm_stock_mg_c_ha: followup_esm.map(|x| x.0),
        followup_esm_depth_m: followup_esm.map(|x| x.1),
        esm_stock_change_mg_c_ha: followup_esm.map(|x| x.0 - baseline_esm.0),
        esm_status: if followup_esm.is_some() {
            "qualified_at_baseline_reference_mass"
        } else {
            "insufficient_followup_soil_mass"
        },
        amendment_carbon_input_mg_c_ha: amendment_input,
        source_attribution_status: ATTRIBUTION,
        uncertainty_status: UNCERTAINTY,
        controls: Controls {
            profile_identity_match_pass: true,
            chronology_pass: true,
            method_and_reference_match_pass: true,
            layer_contiguity_pass: true,
            unit_semantics_pass: true,
            no_depth_extrapolation_pass: true,
            amendment_not_added_to_observed_stock_pass: true,
        },
        claim_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: soc-stock-esm-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("SOC stock ESM audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-stock-esm.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn fixed_depth_and_esm_can_diverge_under_bulk_density_change() {
        let s = scenario();
        validate(&s).expect("valid scenario");
        let report = build_report(&s, "0".repeat(64));
        assert!((report.baseline_fixed_depth_stock_mg_c_ha - 52.65).abs() < 1e-8);
        assert!((report.followup_fixed_depth_stock_mg_c_ha.unwrap() - 51.84).abs() < 1e-8);
        assert!(report.fixed_depth_stock_change_mg_c_ha.unwrap() < 0.0);
        assert!((report.followup_esm_stock_mg_c_ha.unwrap() - 56.16).abs() < 1e-8);
        assert!(report.esm_stock_change_mg_c_ha.unwrap() > 0.0);
        assert!((report.followup_esm_depth_m.unwrap() - 0.325).abs() < 1e-9);
    }

    #[test]
    fn insufficient_followup_mass_abstains_without_extrapolation() {
        let mut s = scenario();
        s.followup_profile.layers.pop();
        validate(&s).expect("fixed depth still covered");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.esm_status, "insufficient_followup_soil_mass");
        assert_eq!(report.followup_esm_stock_mg_c_ha, None);
        assert_eq!(report.esm_stock_change_mg_c_ha, None);
    }

    #[test]
    fn layer_gap_or_overlap_is_rejected() {
        let mut s = scenario();
        s.followup_profile.layers[1].top_depth_m = 0.31;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn coarse_fragment_fraction_cannot_be_applied_outside_zero_one() {
        let mut s = scenario();
        s.baseline_profile.layers[0].coarse_fragment_volume_fraction = 1.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn amendment_carbon_input_is_separate_from_measured_stock() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!((report.amendment_carbon_input_mg_c_ha - 0.9).abs() < 1e-9);
        assert!((report.baseline_fixed_depth_stock_mg_c_ha - 52.65).abs() < 1e-8);
        assert_eq!(
            report.source_attribution_status,
            "unresolved_bulk_soc_does_not_identify_native_vs_amendment_carbon"
        );
    }

    #[test]
    fn duplicated_layer_identity_is_rejected() {
        let mut s = scenario();
        s.followup_profile.layers[1].layer_id = s.followup_profile.layers[0].layer_id.clone();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn mismatched_reference_method_is_rejected() {
        let mut s = scenario();
        s.followup_profile.reference_method_id = "different-method".into();
        assert!(validate(&s).is_err());
    }
}
