//! Synthetic-only layer-resolved SOC stock / equivalent-soil-mass auditor.
//!
//! Fixed-depth (FD) and ESM are calculated separately from layer SOC
//! concentration and fine-earth mass. Coarse fragments are removed using an
//! explicitly declared mass fraction. No extrapolation or uncertainty
//! propagation is performed.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const UNCERTAINTY_STATUS: &str = "not_propagated_laboratory_and_core_covariance_unavailable";
const FINE_EARTH_BASIS: &str = "total_dry_soil_mass_times_one_minus_coarse_fragment_mass_fraction";
const COARSE_BASIS: &str = "mass_fraction_of_total_dry_soil";
const BOUNDARY: &str = "Synthetic SOC stock arithmetic audit only. Fixed-depth and equivalent-soil-mass estimates are separately reported. No uncertainty propagation, treatment effect, carbon-credit quantity, MRV certification or field sequestration claim is estimated. Core dispersion is not treatment replication.";

const EPS: f64 = 1e-8;

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    methods: Methods,
    config: Config,
    observation_rounds: Vec<ObservationRound>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Methods {
    dry_mass_method: String,
    coarse_fragment_method: String,
    coarse_fragment_basis: String,
    soc_method: String,
    carbon_basis: String,
    inorganic_carbon_correction_status: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Config {
    fixed_depth_cm: f64,
    reference_fine_earth_mass_Mg_per_ha: f64,
    minimum_cores_per_round: usize,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct ObservationRound {
    round_id: String,
    date: String,
    experimental_unit_id: String,
    cores: Vec<CoreProfile>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct CoreProfile {
    profile_id: String,
    core_id: String,
    layers: Vec<Layer>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Layer {
    layer_id: String,
    depth_top_cm: f64,
    depth_bottom_cm: f64,
    dry_soil_mass_Mg_per_ha: f64,
    coarse_fragment_mass_fraction: f64,
    soc_g_C_kg_fine_earth: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    fixed_depth_cm: f64,
    reference_fine_earth_mass_Mg_per_ha: f64,
    fine_earth_mass_basis: &'static str,
    coarse_fragment_basis: &'static str,
    uncertainty_status: &'static str,
    rounds: Vec<RoundResult>,
    temporal_change: TemporalChange,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct RoundResult {
    round_id: String,
    date: String,
    experimental_unit_id: String,
    core_count: usize,
    mean_fixed_depth_soc_stock_Mg_C_per_ha: f64,
    sd_fixed_depth_soc_stock_Mg_C_per_ha: Option<f64>,
    mean_esm_soc_stock_Mg_C_per_ha: f64,
    sd_esm_soc_stock_Mg_C_per_ha: Option<f64>,
    profiles: Vec<ProfileResult>,
}

#[derive(Debug, Serialize)]
struct ProfileResult {
    profile_id: String,
    core_id: String,
    observed_fine_earth_mass_Mg_per_ha: f64,
    fixed_depth_fine_earth_mass_Mg_per_ha: f64,
    fixed_depth_soc_stock_Mg_C_per_ha: f64,
    esm_soc_stock_Mg_C_per_ha: f64,
    esm_reference_mass_Mg_per_ha: f64,
    esm_partial_layer_used: bool,
}

#[derive(Debug, Serialize)]
struct TemporalChange {
    first_round_id: String,
    last_round_id: String,
    fixed_depth_stock_change_Mg_C_per_ha: f64,
    esm_stock_change_Mg_C_per_ha: f64,
    interpretation: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    concentration_not_stock_pass: bool,
    same_fixed_depth_support_pass: bool,
    same_reference_mass_pass: bool,
    coarse_fragment_basis_pass: bool,
    no_extrapolation_beyond_profile_pass: bool,
    same_experimental_unit_pass: bool,
    no_causal_claim_pass: bool,
    no_treatment_replication_claim_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn valid_date(date: &str) -> bool {
    date.len() == 10
        && date.as_bytes()[4] == b'-'
        && date.as_bytes()[7] == b'-'
        && date
            .bytes()
            .enumerate()
            .all(|(i, byte)| i == 4 || i == 7 || byte.is_ascii_digit())
        && date[0..4]
            .parse::<u32>()
            .is_ok_and(|year| (1..=9999).contains(&year))
        && date[5..7]
            .parse::<u32>()
            .is_ok_and(|month| (1..=12).contains(&month))
        && date[8..10]
            .parse::<u32>()
            .is_ok_and(|day| (1..=31).contains(&day))
}

fn fine_earth_mass(layer: &Layer) -> f64 {
    layer.dry_soil_mass_Mg_per_ha * (1.0 - layer.coarse_fragment_mass_fraction)
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soc-stock-esm-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema/version".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.methods.dry_mass_method.trim().is_empty()
        || s.methods.coarse_fragment_method.trim().is_empty()
        || s.methods.soc_method.trim().is_empty()
    {
        return Err("scenario and measurement-method metadata must be non-empty".into());
    }
    if s.methods.coarse_fragment_basis != COARSE_BASIS {
        return Err(
            "coarse fragments must be expressed as a mass fraction of total dry soil mass".into(),
        );
    }
    if !matches!(
        s.methods.carbon_basis.as_str(),
        "organic_carbon_direct" | "total_carbon_corrected_for_inorganic"
    ) {
        return Err("carbon basis must represent soil organic carbon".into());
    }
    if !matches!(
        s.methods.inorganic_carbon_correction_status.as_str(),
        "corrected_by_method" | "not_applicable_confirmed_noncalcareous"
    ) {
        return Err(
            "inorganic carbon interference is unassessed; SOC stock is not qualified".into(),
        );
    }
    if s.methods.carbon_basis == "total_carbon_corrected_for_inorganic"
        && s.methods.inorganic_carbon_correction_status != "corrected_by_method"
    {
        return Err("total carbon requires an explicit inorganic-carbon correction".into());
    }
    finite("fixed_depth_cm", s.config.fixed_depth_cm)?;
    finite(
        "reference_fine_earth_mass_Mg_per_ha",
        s.config.reference_fine_earth_mass_Mg_per_ha,
    )?;
    if s.config.fixed_depth_cm <= 0.0
        || s.config.reference_fine_earth_mass_Mg_per_ha <= 0.0
        || s.config.minimum_cores_per_round < 2
    {
        return Err("fixed depth, reference mass and minimum core count must be positive (at least two cores)".into());
    }
    if s.observation_rounds.len() < 2 {
        return Err("at least two observation rounds are required".into());
    }

    let mut round_ids = BTreeSet::new();
    let mut dates = BTreeSet::new();
    let mut profile_ids = BTreeSet::new();
    let mut core_ids = BTreeSet::new();
    let mut expected_bounds: Option<Vec<(f64, f64)>> = None;
    let mut reference_unit: Option<&str> = None;
    let mut prior_date: Option<&str> = None;

    for (round_index, round) in s.observation_rounds.iter().enumerate() {
        if round.round_id.trim().is_empty()
            || round.experimental_unit_id.trim().is_empty()
            || !round_ids.insert(round.round_id.clone())
        {
            return Err(
                "round IDs and experimental-unit identity must be non-empty and round IDs unique"
                    .into(),
            );
        }
        if !valid_date(&round.date) || !dates.insert(round.date.clone()) {
            return Err("round dates must be valid and unique".into());
        }
        if let Some(previous) = prior_date
            && round.date.as_str() <= previous
        {
            return Err("observation rounds must be strictly chronological".into());
        }
        prior_date = Some(&round.date);
        if let Some(unit) = reference_unit {
            if unit != round.experimental_unit_id {
                return Err(
                    "all v0.1 rounds must refer to the same declared experimental unit".into(),
                );
            }
        } else {
            reference_unit = Some(&round.experimental_unit_id);
        }
        if round.cores.len() < s.config.minimum_cores_per_round {
            return Err(format!(
                "round {} has fewer cores than the declared minimum",
                round.round_id
            ));
        }

        for core in &round.cores {
            if core.profile_id.trim().is_empty()
                || core.core_id.trim().is_empty()
                || !profile_ids.insert(core.profile_id.clone())
                || !core_ids.insert(core.core_id.clone())
            {
                return Err(
                    "profile and core IDs must be non-empty and unique across the scenario".into(),
                );
            }
            if core.layers.is_empty() {
                return Err(format!("profile {} has no layers", core.profile_id));
            }
            let mut prior_bottom: Option<f64> = None;
            let mut bounds = Vec::with_capacity(core.layers.len());
            let mut layer_ids = BTreeSet::new();
            let mut fixed_depth_boundary_found = false;
            let mut observed_mass = 0.0;

            for (layer_index, layer) in core.layers.iter().enumerate() {
                for (name, value) in [
                    ("depth_top_cm", layer.depth_top_cm),
                    ("depth_bottom_cm", layer.depth_bottom_cm),
                    ("dry_soil_mass_Mg_per_ha", layer.dry_soil_mass_Mg_per_ha),
                    (
                        "coarse_fragment_mass_fraction",
                        layer.coarse_fragment_mass_fraction,
                    ),
                    ("soc_g_C_kg_fine_earth", layer.soc_g_C_kg_fine_earth),
                ] {
                    finite(name, value)?;
                }
                if layer.layer_id.trim().is_empty() || !layer_ids.insert(layer.layer_id.clone()) {
                    return Err(format!(
                        "profile {} has empty or duplicate layer ID",
                        core.profile_id
                    ));
                }
                if layer.depth_top_cm < 0.0
                    || layer.depth_bottom_cm <= layer.depth_top_cm
                    || layer.dry_soil_mass_Mg_per_ha <= 0.0
                    || !(0.0..1.0).contains(&layer.coarse_fragment_mass_fraction)
                    || layer.soc_g_C_kg_fine_earth < 0.0
                {
                    return Err(format!(
                        "profile {} has physically inadmissible layer values",
                        core.profile_id
                    ));
                }
                if layer_index == 0 && layer.depth_top_cm.abs() > EPS {
                    return Err(format!(
                        "profile {} does not start at the soil surface",
                        core.profile_id
                    ));
                }
                if let Some(bottom) = prior_bottom
                    && (layer.depth_top_cm - bottom).abs() > EPS
                {
                    return Err(format!(
                        "profile {} has a layer gap or overlap",
                        core.profile_id
                    ));
                }
                if (layer.depth_bottom_cm - s.config.fixed_depth_cm).abs() <= EPS {
                    fixed_depth_boundary_found = true;
                }
                prior_bottom = Some(layer.depth_bottom_cm);
                observed_mass += fine_earth_mass(layer);
                bounds.push((layer.depth_top_cm, layer.depth_bottom_cm));
            }
            if !fixed_depth_boundary_found {
                return Err(format!(
                    "profile {} does not have an exact layer boundary at the declared fixed depth",
                    core.profile_id
                ));
            }
            if observed_mass + EPS < s.config.reference_fine_earth_mass_Mg_per_ha {
                return Err(format!(
                    "profile {} contains insufficient measured fine-earth mass for the declared ESM reference mass",
                    core.profile_id
                ));
            }
            if let Some(expected) = &expected_bounds {
                if expected.len() != bounds.len()
                    || expected
                        .iter()
                        .zip(&bounds)
                        .any(|(a, b)| (a.0 - b.0).abs() > EPS || (a.1 - b.1).abs() > EPS)
                {
                    return Err(
                        "layer depth boundaries must match across every core and round".into(),
                    );
                }
            } else {
                expected_bounds = Some(bounds);
            }
        }

        if round_index == 0 && round.cores.is_empty() {
            return Err("baseline round has no core profiles".into());
        }
    }
    Ok(())
}

fn calculate_profile(
    core: &CoreProfile,
    fixed_depth_cm: f64,
    reference_mass: f64,
) -> Result<ProfileResult, String> {
    let mut observed_fine_mass = 0.0;
    let mut fixed_depth_fine_mass = 0.0;
    let mut fixed_depth_carbon = 0.0;
    let mut remaining_reference_mass = reference_mass;
    let mut esm_carbon = 0.0;
    let mut partial_layer_used = false;

    for layer in &core.layers {
        let fine_mass = fine_earth_mass(layer);
        observed_fine_mass += fine_mass;

        if layer.depth_bottom_cm <= fixed_depth_cm + EPS {
            fixed_depth_fine_mass += fine_mass;
            fixed_depth_carbon += fine_mass * layer.soc_g_C_kg_fine_earth / 1000.0;
        }

        if remaining_reference_mass > EPS {
            let consumed = fine_mass.min(remaining_reference_mass);
            esm_carbon += consumed * layer.soc_g_C_kg_fine_earth / 1000.0;
            if consumed + EPS < fine_mass {
                partial_layer_used = true;
            }
            remaining_reference_mass -= consumed;
        }
    }

    if remaining_reference_mass > EPS {
        return Err(format!(
            "profile {} cannot supply the complete reference fine-earth mass; no extrapolation allowed",
            core.profile_id
        ));
    }

    Ok(ProfileResult {
        profile_id: core.profile_id.clone(),
        core_id: core.core_id.clone(),
        observed_fine_earth_mass_Mg_per_ha: observed_fine_mass,
        fixed_depth_fine_earth_mass_Mg_per_ha: fixed_depth_fine_mass,
        fixed_depth_soc_stock_Mg_C_per_ha: fixed_depth_carbon,
        esm_soc_stock_Mg_C_per_ha: esm_carbon,
        esm_reference_mass_Mg_per_ha: reference_mass,
        esm_partial_layer_used: partial_layer_used,
    })
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn sample_sd(values: &[f64]) -> Option<f64> {
    if values.len() < 2 {
        return None;
    }
    let average = mean(values);
    let sum_squares = values
        .iter()
        .map(|value| (value - average).powi(2))
        .sum::<f64>();
    Some((sum_squares / (values.len() - 1) as f64).sqrt())
}

fn build_report(s: &Scenario, input_sha256: String) -> Result<Report, String> {
    let mut round_results = Vec::with_capacity(s.observation_rounds.len());

    for round in &s.observation_rounds {
        let mut profiles = Vec::with_capacity(round.cores.len());
        for core in &round.cores {
            profiles.push(calculate_profile(
                core,
                s.config.fixed_depth_cm,
                s.config.reference_fine_earth_mass_Mg_per_ha,
            )?);
        }
        let fixed_values: Vec<f64> = profiles
            .iter()
            .map(|profile| profile.fixed_depth_soc_stock_Mg_C_per_ha)
            .collect();
        let esm_values: Vec<f64> = profiles
            .iter()
            .map(|profile| profile.esm_soc_stock_Mg_C_per_ha)
            .collect();

        round_results.push(RoundResult {
            round_id: round.round_id.clone(),
            date: round.date.clone(),
            experimental_unit_id: round.experimental_unit_id.clone(),
            core_count: profiles.len(),
            mean_fixed_depth_soc_stock_Mg_C_per_ha: mean(&fixed_values),
            sd_fixed_depth_soc_stock_Mg_C_per_ha: sample_sd(&fixed_values),
            mean_esm_soc_stock_Mg_C_per_ha: mean(&esm_values),
            sd_esm_soc_stock_Mg_C_per_ha: sample_sd(&esm_values),
            profiles,
        });
    }

    let first = round_results.first().expect("validation requires rounds");
    let last = round_results.last().expect("validation requires rounds");
    Ok(Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        fixed_depth_cm: s.config.fixed_depth_cm,
        reference_fine_earth_mass_Mg_per_ha: s.config.reference_fine_earth_mass_Mg_per_ha,
        fine_earth_mass_basis: FINE_EARTH_BASIS,
        coarse_fragment_basis: COARSE_BASIS,
        uncertainty_status: UNCERTAINTY_STATUS,
        rounds: round_results,
        temporal_change: TemporalChange {
            first_round_id: first.round_id.clone(),
            last_round_id: last.round_id.clone(),
            fixed_depth_stock_change_Mg_C_per_ha: last.mean_fixed_depth_soc_stock_Mg_C_per_ha
                - first.mean_fixed_depth_soc_stock_Mg_C_per_ha,
            esm_stock_change_Mg_C_per_ha: last.mean_esm_soc_stock_Mg_C_per_ha - first.mean_esm_soc_stock_Mg_C_per_ha,
            interpretation: "Descriptive difference between round means for one experimental unit; within-round core dispersion is subsampling variation, not independent treatment replication or causal effect.",
        },
        controls: Controls {
            concentration_not_stock_pass: true,
            same_fixed_depth_support_pass: true,
            same_reference_mass_pass: true,
            coarse_fragment_basis_pass: true,
            no_extrapolation_beyond_profile_pass: true,
            same_experimental_unit_pass: true,
            no_causal_claim_pass: true,
            no_treatment_replication_claim_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    })
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
    let report = build_report(&scenario, digest).map_err(io::Error::other)?;
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
        eprintln!("SOC stock / ESM audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-compaction-esm.json"))
            .expect("synthetic fixture parses")
    }

    fn approx(left: f64, right: f64) {
        assert!((left - right).abs() < 1e-8, "expected {right}, got {left}");
    }

    #[test]
    fn compaction_changes_fixed_depth_stock_but_not_esm_stock() {
        let s = scenario();
        validate(&s).expect("valid synthetic compaction fixture");
        let report = build_report(&s, "0".repeat(64)).expect("report builds");
        approx(report.rounds[0].mean_fixed_depth_soc_stock_Mg_C_per_ha, 39.204);
        approx(report.rounds[1].mean_fixed_depth_soc_stock_Mg_C_per_ha, 43.1244);
        approx(report.rounds[0].mean_esm_soc_stock_Mg_C_per_ha, 39.204);
        approx(report.rounds[1].mean_esm_soc_stock_Mg_C_per_ha, 39.204);
        approx(report.temporal_change.esm_stock_change_Mg_C_per_ha, 0.0);
        assert!(report.temporal_change.fixed_depth_stock_change_Mg_C_per_ha > 3.9);
    }

    #[test]
    fn coarse_fragments_are_removed_from_fine_earth_mass() {
        let report = build_report(&scenario(), "0".repeat(64)).expect("report builds");
        approx(
            report.rounds[0].profiles[0].fixed_depth_fine_earth_mass_Mg_per_ha,
            3960.0,
        );
        approx(
            report.rounds[0].profiles[0].observed_fine_earth_mass_Mg_per_ha,
            5400.0,
        );
    }

    #[test]
    fn partial_layer_consumption_uses_declared_reference_mass() {
        let mut s = scenario();
        s.config.reference_fine_earth_mass_Mg_per_ha = 2000.0;
        validate(&s).expect("reference mass is supported");
        let report = build_report(&s, "0".repeat(64)).expect("report builds");
        approx(report.rounds[0].profiles[0].esm_soc_stock_Mg_C_per_ha, 20.0);
        assert!(report.rounds[0].profiles[0].esm_partial_layer_used);
    }

    #[test]
    fn reference_mass_above_profile_support_is_rejected() {
        let mut s = scenario();
        s.config.reference_fine_earth_mass_Mg_per_ha = 9000.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn volume_based_coarse_fragment_fraction_is_rejected() {
        let mut s = scenario();
        s.methods.coarse_fragment_basis = "volume_fraction".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn unassessed_inorganic_carbon_interference_is_rejected() {
        let mut s = scenario();
        s.methods.inorganic_carbon_correction_status = "not_assessed".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn layer_gap_or_overlap_is_rejected() {
        let mut s = scenario();
        s.observation_rounds[0].cores[0].layers[1].depth_top_cm = 11.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn inconsistent_layer_boundaries_across_cores_are_rejected() {
        let mut s = scenario();
        s.observation_rounds[1].cores[1].layers[1].depth_bottom_cm = 29.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn fixed_depth_must_match_a_measured_layer_boundary() {
        let mut s = scenario();
        s.config.fixed_depth_cm = 25.0;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn temporal_rounds_must_refer_to_the_same_experimental_unit() {
        let mut s = scenario();
        s.observation_rounds[1].experimental_unit_id = "other-plot".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rounds_must_be_strictly_chronological() {
        let mut s = scenario();
        s.observation_rounds[1].date = "2020-01-01".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn core_ids_must_be_unique_across_rounds() {
        let mut s = scenario();
        s.observation_rounds[1].cores[0].core_id = "base-core-01".into();
        assert!(validate(&s).is_err());
    }
}
