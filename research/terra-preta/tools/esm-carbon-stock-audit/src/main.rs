//! Synthetic-only layered-core fixed-depth and equivalent-soil-mass SOC stock audit.
//!
//! Fine-earth soil mass is calculated per hectare from dry fine-earth bulk
//! density, layer thickness and coarse-fragment volume fraction. ESM integrates
//! measured layer concentrations up to a predeclared reference fine-earth mass.
//! No profile extrapolation or causal claim is made.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const UNCERTAINTY_STATUS: &str = "not_propagated_core_spatial_and_method_covariance_unavailable";
const BOUNDARY: &str = "Synthetic-only SOC stock calculation. Cores are nested spatial subsamples of one experimental unit. No spatial, analytical, reference-mass or method covariance is propagated. FD and ESM changes are descriptive, not causal treatment effects or MRV certification.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    experimental_unit: ExperimentalUnit,
    method_basis: MethodBasis,
    reference_fine_earth_mass_mg_ha: f64,
    rounds: Vec<SamplingRound>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExperimentalUnit {
    experimental_unit_id: String,
    independence_group_id: String,
    core_role: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MethodBasis {
    soc_unit: String,
    bulk_density_basis: String,
    coarse_fragment_basis: String,
    soc_method: String,
    bulk_density_method: String,
    coarse_fragment_method: String,
    profile_area_basis: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SamplingRound {
    round_id: String,
    date: String,
    round_role: String,
    cores: Vec<Core>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Core {
    core_id: String,
    layers: Vec<Layer>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Layer {
    depth_top_cm: f64,
    depth_bottom_cm: f64,
    soc_g_per_kg_fine_earth: f64,
    fine_earth_bulk_density_g_cm3: f64,
    coarse_fragment_volume_fraction: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    experimental_unit_id: String,
    independence_group_id: String,
    core_role: &'static str,
    reference_fine_earth_mass_mg_ha: f64,
    rounds: Vec<RoundResult>,
    fixed_depth_stock_change_mg_c_ha: f64,
    equivalent_soil_mass_stock_change_mg_c_ha: f64,
    difference_between_method_changes_mg_c_ha: f64,
    uncertainty_status: &'static str,
    interpretation_boundary: &'static str,
    controls: Controls,
}

#[derive(Debug, Serialize)]
struct RoundResult {
    round_id: String,
    date: String,
    round_role: String,
    core_count: usize,
    fixed_depth_stock_mean_mg_c_ha: f64,
    equivalent_soil_mass_stock_mean_mg_c_ha: f64,
    cores: Vec<CoreResult>,
}

#[derive(Debug, Serialize)]
struct CoreResult {
    core_id: String,
    total_fine_earth_mass_mg_ha: f64,
    fixed_depth_stock_mg_c_ha: f64,
    equivalent_soil_mass_stock_mg_c_ha: f64,
}

#[derive(Debug, Serialize)]
struct Controls {
    matched_vertical_support_pass: bool,
    reference_mass_available_in_all_cores_pass: bool,
    method_basis_consistent_pass: bool,
    no_extrapolation_pass: bool,
    cores_not_counted_as_independent_treatment_replicates_pass: bool,
    no_causal_claim_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn valid_date(date: &str) -> bool {
    date.len() == 10
        && date.as_bytes()[4] == b'-'
        && date.as_bytes()[7] == b'-'
        && date
            .bytes()
            .enumerate()
            .all(|(i, byte)| i == 4 || i == 7 || byte.is_ascii_digit())
        && date[0..4].parse::<u32>().is_ok_and(|x| (1..=9999).contains(&x))
        && date[5..7].parse::<u32>().is_ok_and(|x| (1..=12).contains(&x))
        && date[8..10].parse::<u32>().is_ok_and(|x| (1..=31).contains(&x))
}

fn layer_support(layers: &[Layer]) -> Result<Vec<(f64, f64)>, String> {
    if layers.is_empty() {
        return Err("each core needs at least one measured layer".into());
    }
    let mut support = Vec::with_capacity(layers.len());
    let mut previous_bottom = 0.0;
    for (index, layer) in layers.iter().enumerate() {
        for (name, value) in [
            ("depth_top_cm", layer.depth_top_cm),
            ("depth_bottom_cm", layer.depth_bottom_cm),
            ("soc_g_per_kg_fine_earth", layer.soc_g_per_kg_fine_earth),
            ("fine_earth_bulk_density_g_cm3", layer.fine_earth_bulk_density_g_cm3),
            ("coarse_fragment_volume_fraction", layer.coarse_fragment_volume_fraction),
        ] {
            finite(name, value)?;
        }
        if (layer.depth_top_cm - previous_bottom).abs() > 1e-9
            || layer.depth_bottom_cm <= layer.depth_top_cm
            || layer.soc_g_per_kg_fine_earth < 0.0
            || layer.fine_earth_bulk_density_g_cm3 <= 0.0
            || !(0.0..1.0).contains(&layer.coarse_fragment_volume_fraction)
        {
            return Err(format!("layer {index} has a gap, overlap or inadmissible measurement"));
        }
        support.push((layer.depth_top_cm, layer.depth_bottom_cm));
        previous_bottom = layer.depth_bottom_cm;
    }
    if (support[0].0 - 0.0).abs() > 1e-9 {
        return Err("profile must begin at depth 0 cm in v0.1".into());
    }
    Ok(support)
}

/// Convert one layer to Mg fine earth / ha.
/// g/cm3 = Mg/m3; cm × 100 converts a layer to Mg/ha.
fn layer_fine_earth_mass_mg_ha(layer: &Layer) -> f64 {
    let thickness_cm = layer.depth_bottom_cm - layer.depth_top_cm;
    layer.fine_earth_bulk_density_g_cm3
        * thickness_cm
        * 100.0
        * (1.0 - layer.coarse_fragment_volume_fraction)
}

fn layer_carbon_stock_mg_c_ha(layer: &Layer) -> f64 {
    layer_fine_earth_mass_mg_ha(layer) * layer.soc_g_per_kg_fine_earth / 1000.0
}

fn compute_core(core: &Core, reference_mass: f64) -> Result<CoreResult, String> {
    let total_mass = core.layers.iter().map(layer_fine_earth_mass_mg_ha).sum::<f64>();
    let fixed_depth_stock = core.layers.iter().map(layer_carbon_stock_mg_ha).sum::<f64>();
    if reference_mass > total_mass + 1e-9 {
        return Err(format!(
            "ESM reference mass exceeds measured profile mass for core {}",
            core.core_id
        ));
    }
    let mut remaining = reference_mass;
    let mut esm_stock = 0.0;
    for layer in &core.layers {
        if remaining <= 1e-9 {
            break;
        }
        let available = layer_fine_earth_mass_mg_ha(layer);
        let take = remaining.min(available);
        esm_stock += take * layer.soc_g_per_kg_fine_earth / 1000.0;
        remaining -= take;
    }
    if remaining > 1e-7 {
        return Err(format!(
            "ESM reference mass cannot be closed from measured profile for core {}",
            core.core_id
        ));
    }
    Ok(CoreResult {
        core_id: core.core_id.clone(),
        total_fine_earth_mass_mg_ha: total_mass,
        fixed_depth_stock_mg_c_ha: fixed_depth_stock,
        equivalent_soil_mass_stock_mg_c_ha: esm_stock,
    })
}

fn validate(s: &Scenario) -> Result<Vec<Vec<CoreResult>>, String> {
    if s.schema_version != "esm-carbon-stock-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts the declared synthetic schema/version only".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.experimental_unit.experimental_unit_id.trim().is_empty()
        || s.experimental_unit.independence_group_id.trim().is_empty()
        || s.experimental_unit.core_role
            != "nested_spatial_subsample_not_independent_treatment_replicate"
    {
        return Err("scenario and nested experimental-unit metadata are required".into());
    }
    if s.method_basis.soc_unit != "g_C_per_kg_dry_fine_earth"
        || s.method_basis.bulk_density_basis
            != "g_cm3_dry_fine_earth_excluding_coarse_fragments"
        || s.method_basis.coarse_fragment_basis != "volume_fraction_of_total_core"
        || s.method_basis.profile_area_basis != "per_hectare"
        || s.method_basis.soc_method.trim().is_empty()
        || s.method_basis.bulk_density_method.trim().is_empty()
        || s.method_basis.coarse_fragment_method.trim().is_empty()
    {
        return Err("unsupported or incomplete SOC / bulk-density / coarse-fragment method basis".into());
    }
    finite("reference_fine_earth_mass_mg_ha", s.reference_fine_earth_mass_mg_ha)?;
    if s.reference_fine_earth_mass_mg_ha <= 0.0 {
        return Err("reference fine-earth soil mass must be positive".into());
    }
    if s.rounds.len() != 2 {
        return Err("v0.1 requires exactly baseline and follow-up rounds".into());
    }
    if s.rounds[0].round_role != "baseline" || s.rounds[1].round_role != "follow_up" {
        return Err("rounds must be ordered baseline then follow_up".into());
    }
    if !valid_date(&s.rounds[0].date)
        || !valid_date(&s.rounds[1].date)
        || s.rounds[0].date >= s.rounds[1].date
    {
        return Err("round dates must be valid and strictly chronological".into());
    }
    let mut round_ids = BTreeSet::new();
    let mut core_ids = BTreeSet::new();
    let mut expected_support: Option<Vec<(f64, f64)>> = None;
    let mut round_results = Vec::with_capacity(2);
    for round in &s.rounds {
        if round.round_id.trim().is_empty() || !round_ids.insert(round.round_id.clone()) {
            return Err("sampling-round IDs must be non-empty and unique".into());
        }
        if round.cores.len() < 2 {
            return Err("each round must contain at least two nested spatial cores".into());
        }
        let mut results = Vec::with_capacity(round.cores.len());
        for core in &round.cores {
            if core.core_id.trim().is_empty() || !core_ids.insert(core.core_id.clone()) {
                return Err("core IDs must be non-empty and unique across rounds".into());
            }
            let support = layer_support(&core.layers)?;
            if let Some(expected) = &expected_support {
                if support.len() != expected.len()
                    || support.iter().zip(expected).any(|(a, b)| {
                        (a.0 - b.0).abs() > 1e-9 || (a.1 - b.1).abs() > 1e-9
                    })
                {
                    return Err("all cores and rounds must have identical vertical layer boundaries".into());
                }
            } else {
                expected_support = Some(support);
            }
            results.push(compute_core(core, s.reference_fine_earth_mass_mg_ha)?);
        }
        round_results.push(results);
    }
    Ok(round_results)
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let mut total = 0.0;
    let mut count = 0usize;
    for value in values {
        total += value;
        count += 1;
    }
    total / count as f64
}

fn build_report(s: &Scenario, digest: String, computed: Vec<Vec<CoreResult>>) -> Report {
    let round_results: Vec<RoundResult> = s.rounds.iter().zip(computed).map(|(round, cores)| {
        let fd_mean = mean(cores.iter().map(|core| core.fixed_depth_stock_mg_c_ha));
        let esm_mean = mean(cores.iter().map(|core| core.equivalent_soil_mass_stock_mg_c_ha));
        RoundResult {
            round_id: round.round_id.clone(),
            date: round.date.clone(),
            round_role: round.round_role.clone(),
            core_count: cores.len(),
            fixed_depth_stock_mean_mg_c_ha: fd_mean,
            equivalent_soil_mass_stock_mean_mg_c_ha: esm_mean,
            cores,
        }
    }).collect();
    let fixed_change = round_results[1].fixed_depth_stock_mean_mg_c_ha
        - round_results[0].fixed_depth_stock_mean_mg_c_ha;
    let esm_change = round_results[1].equivalent_soil_mass_stock_mean_mg_c_ha
        - round_results[0].equivalent_soil_mass_stock_mean_mg_c_ha;
    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        experimental_unit_id: s.experimental_unit.experimental_unit_id.clone(),
        independence_group_id: s.experimental_unit.independence_group_id.clone(),
        core_role: "nested_spatial_subsample_not_independent_treatment_replicate",
        reference_fine_earth_mass_mg_ha: s.reference_fine_earth_mass_mg_ha,
        rounds: round_results,
        fixed_depth_stock_change_mg_c_ha: fixed_change,
        equivalent_soil_mass_stock_change_mg_c_ha: esm_change,
        difference_between_method_changes_mg_c_ha: fixed_change - esm_change,
        uncertainty_status: UNCERTAINTY_STATUS,
        interpretation_boundary: BOUNDARY,
        controls: Controls {
            matched_vertical_support_pass: true,
            reference_mass_available_in_all_cores_pass: true,
            method_basis_consistent_pass: true,
            no_extrapolation_pass: true,
            cores_not_counted_as_independent_treatment_replicates_pass: true,
            no_causal_claim_pass: true,
        },
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    let computed = validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest, computed);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: esm-carbon-stock-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("ESM carbon stock audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-compaction-only.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn compaction_only_control_separates_fd_from_esm() {
        let s = scenario();
        let computed = validate(&s).expect("valid synthetic profiles");
        let r = build_report(&s, "0".repeat(64), computed);
        assert!(r.rounds[1].fixed_depth_stock_mean_mg_c_ha
            > r.rounds[0].fixed_depth_stock_mean_mg_c_ha);
        assert!(
            (r.rounds[1].equivalent_soil_mass_stock_mean_mg_c_ha
                - r.rounds[0].equivalent_soil_mass_stock_mean_mg_c_ha)
                .abs()
                < 1e-9
        );
        assert!(r.fixed_depth_stock_change_mg_c_ha > 0.0);
        assert!(r.equivalent_soil_mass_stock_change_mg_c_ha.abs() < 1e-9);
    }

    #[test]
    fn layer_mass_conversion_matches_analytic_value() {
        let s = scenario();
        let layer = &s.rounds[0].cores[0].layers[0];
        assert!((layer_fine_earth_mass_mg_ha(layer) - 1235.0).abs() < 1e-9);
        assert!((layer_carbon_stock_mg_c_ha(layer) - 13.585).abs() < 1e-9);
    }

    #[test]
    fn reference_mass_above_observed_profile_is_rejected() {
        let mut s = scenario();
        s.reference_fine_earth_mass_mg_ha = 999_999.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn layer_gap_or_overlap_is_rejected() {
        let mut s = scenario();
        s.rounds[1].cores[0].layers[1].depth_top_cm = 9.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn vertical_support_must_match_across_cores_and_rounds() {
        let mut s = scenario();
        s.rounds[1].cores[0].layers[1].depth_bottom_cm = 21.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn invalid_coarse_fragment_fraction_is_rejected() {
        let mut s = scenario();
        s.rounds[0].cores[0].layers[0].coarse_fragment_volume_fraction = 1.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rounds_must_be_baseline_then_follow_up() {
        let mut s = scenario();
        s.rounds.swap(0, 1);
        assert!(validate(&s).is_err());
    }

    #[test]
    fn nested_cores_do_not_become_extra_experimental_units() {
        let s = scenario();
        let r = build_report(&s, "0".repeat(64), validate(&s).expect("valid fixture"));
        assert_eq!(r.rounds[0].core_count, 2);
        assert_eq!(r.rounds[1].core_count, 2);
        assert_eq!(
            r.core_role,
            "nested_spatial_subsample_not_independent_treatment_replicate"
        );
        assert_eq!(r.experimental_unit_id, "synthetic-plot-01");
    }

    #[test]
    fn equivalent_mass_integrates_piecewise_by_measured_layers() {
        let s = scenario();
        let c = compute_core(&s.rounds[1].cores[0], s.reference_fine_earth_mass_mg_ha)
            .expect("reference mass observed");
        assert!((c.equivalent_soil_mass_stock_mg_c_ha - 28.215).abs() < 1e-9);
    }
}
