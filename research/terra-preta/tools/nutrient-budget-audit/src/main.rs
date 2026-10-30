//! Audit an elemental nutrient boundary budget for one experimental unit and period.
//!
//! Internal transfers (mineralization, immobilization, sorption, root uptake when
//! plant biomass is inside the control volume) are not boundary inputs/outputs.
//! Availability indicators are reported separately and never added to stock pools.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const UNIT_BASIS: &str = "kg_element_per_ha";

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
enum Element {
    #[serde(rename = "N")]
    N,
    #[serde(rename = "P")]
    P,
    #[serde(rename = "K")]
    K,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum InputCategory {
    Fertilizer,
    OrganicAmendment,
    Irrigation,
    AtmosphericDeposition,
    BiologicalFixation,
    Weathering,
    SeedInput,
    OtherDeclared,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum OutputCategory {
    HarvestExport,
    Leaching,
    Runoff,
    Erosion,
    GaseousLoss,
    OtherDeclared,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Compartment {
    Soil,
    StandingBiomass,
    RetainedResidue,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Period {
    start_date: String,
    end_date: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ControlVolume {
    area_ha: f64,
    top_depth_cm: f64,
    bottom_depth_cm: f64,
    boundary_description: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Flow<C> {
    flow_id: String,
    category: C,
    amount_kg_element_per_ha: f64,
    standard_uncertainty_kg_element_per_ha: Option<f64>,
    method: String,
    evidence_ref: String,
}

type InputFlow = Flow<InputCategory>;
type OutputFlow = Flow<OutputCategory>;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoragePool {
    pool_id: String,
    compartment: Compartment,
    stock_kg_element_per_ha: f64,
    standard_uncertainty_kg_element_per_ha: Option<f64>,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct AvailabilityIndicator {
    indicator_id: String,
    name: String,
    value: f64,
    unit: String,
    depth_top_cm: f64,
    depth_bottom_cm: f64,
    method: String,
    evidence_ref: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BudgetInput {
    schema_version: String,
    record_id: String,
    study_id: String,
    site_id: String,
    experimental_unit_id: String,
    treatment_arm_id: String,
    comparator_arm_id: String,
    element: Element,
    unit_basis: String,
    source_sha256: String,
    period: Period,
    control_volume: ControlVolume,
    inputs: Vec<InputFlow>,
    outputs: Vec<OutputFlow>,
    storage_start: Vec<StoragePool>,
    storage_end: Vec<StoragePool>,
    availability_indicators: Vec<AvailabilityIndicator>,
}

#[derive(Debug, Serialize)]
struct PlotScaled {
    input_kg_element: f64,
    output_kg_element: f64,
    storage_change_kg_element: f64,
    residual_kg_element: f64,
    uncertainty_bound_kg_element: Option<f64>,
}

#[derive(Debug, Serialize)]
struct FlowCounts {
    inputs: usize,
    outputs: usize,
    storage_pools: usize,
    availability_indicators: usize,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    record_id: String,
    element: Element,
    input_sha256: String,
    study_id: String,
    site_id: String,
    experimental_unit_id: String,
    treatment_arm_id: String,
    comparator_arm_id: String,
    period_start_date: String,
    period_end_date: String,
    control_volume: ControlVolume,
    input_total_kg_element_per_ha: f64,
    output_total_kg_element_per_ha: f64,
    storage_start_total_kg_element_per_ha: f64,
    storage_end_total_kg_element_per_ha: f64,
    storage_change_kg_element_per_ha: f64,
    residual_kg_element_per_ha: f64,
    conservative_linear_standard_uncertainty_bound_kg_element_per_ha: Option<f64>,
    plot_scaled: PlotScaled,
    flow_counts: FlowCounts,
    availability_indicators: Vec<AvailabilityIndicator>,
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

fn nonblank(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.trim() != value {
        return Err(format!("{field} must be non-empty and have no surrounding whitespace"));
    }
    Ok(())
}

fn valid_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 10 || bytes[4] != b'-' || bytes[7] != b'-' {
        return false;
    }
    if !bytes
        .iter()
        .enumerate()
        .all(|(index, byte)| index == 4 || index == 7 || byte.is_ascii_digit())
    {
        return false;
    }
    let year = value[0..4].parse::<u32>().ok();
    let month = value[5..7].parse::<u32>().ok();
    let day = value[8..10].parse::<u32>().ok();
    let (Some(year), Some(month), Some(day)) = (year, month, day) else {
        return false;
    };
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    day >= 1 && day <= days_in_month
}

fn finite_nonnegative(value: f64, field: &str) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{field} must be finite and non-negative"));
    }
    Ok(())
}

fn validate_uncertainty(value: Option<f64>, field: &str) -> Result<(), String> {
    if let Some(value) = value {
        finite_nonnegative(value, field)?;
    }
    Ok(())
}

fn checked_sum(values: impl Iterator<Item = f64>, field: &str) -> Result<f64, String> {
    let mut total = 0.0;
    for value in values {
        total += value;
        if !total.is_finite() {
            return Err(format!("{field} overflowed or became non-finite"));
        }
    }
    Ok(total)
}

fn validate_flow<C>(flow: &Flow<C>, seen: &mut HashSet<String>) -> Result<(), String> {
    nonblank(&flow.flow_id, "flow_id")?;
    if !seen.insert(flow.flow_id.clone()) {
        return Err(format!("duplicate flow_id {:?}", flow.flow_id));
    }
    finite_nonnegative(
        flow.amount_kg_element_per_ha,
        "amount_kg_element_per_ha",
    )?;
    validate_uncertainty(
        flow.standard_uncertainty_kg_element_per_ha,
        "standard_uncertainty_kg_element_per_ha",
    )?;
    nonblank(&flow.method, "flow method")?;
    nonblank(&flow.evidence_ref, "flow evidence_ref")?;
    Ok(())
}

fn validate_pool(pool: &StoragePool) -> Result<(), String> {
    nonblank(&pool.pool_id, "pool_id")?;
    finite_nonnegative(pool.stock_kg_element_per_ha, "stock_kg_element_per_ha")?;
    validate_uncertainty(
        pool.standard_uncertainty_kg_element_per_ha,
        "pool standard_uncertainty_kg_element_per_ha",
    )?;
    nonblank(&pool.method, "pool method")?;
    nonblank(&pool.evidence_ref, "pool evidence_ref")?;
    Ok(())
}

fn validate_input(input: &BudgetInput) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err("unsupported schema_version; expected 0.1.0".to_owned());
    }
    for (value, field) in [
        (input.record_id.as_str(), "record_id"),
        (input.study_id.as_str(), "study_id"),
        (input.site_id.as_str(), "site_id"),
        (input.experimental_unit_id.as_str(), "experimental_unit_id"),
        (input.treatment_arm_id.as_str(), "treatment_arm_id"),
        (input.comparator_arm_id.as_str(), "comparator_arm_id"),
    ] {
        nonblank(value, field)?;
    }
    if input.unit_basis != UNIT_BASIS {
        return Err(format!("unit_basis must be {UNIT_BASIS:?}; convert elemental mass explicitly upstream"));
    }
    if input.source_sha256.len() != 64
        || !input.source_sha256.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }

    if !valid_date(&input.period.start_date) || !valid_date(&input.period.end_date) {
        return Err("period dates must be valid calendar dates in YYYY-MM-DD format".to_owned());
    }
    if input.period.end_date < input.period.start_date {
        return Err("period end_date precedes start_date".to_owned());
    }

    let volume = &input.control_volume;
    if !volume.area_ha.is_finite() || volume.area_ha <= 0.0 {
        return Err("control_volume.area_ha must be finite and greater than zero".to_owned());
    }
    finite_nonnegative(volume.top_depth_cm, "control_volume.top_depth_cm")?;
    if !volume.bottom_depth_cm.is_finite()
        || volume.bottom_depth_cm <= volume.top_depth_cm
    {
        return Err("control_volume.bottom_depth_cm must be finite and greater than top_depth_cm".to_owned());
    }
    nonblank(&volume.boundary_description, "control_volume.boundary_description")?;

    let mut flow_ids = HashSet::new();
    for flow in &input.inputs {
        validate_flow(flow, &mut flow_ids)?;
    }
    for flow in &input.outputs {
        validate_flow(flow, &mut flow_ids)?;
    }

    if input.storage_start.is_empty() || input.storage_end.is_empty() {
        return Err("storage_start and storage_end must each contain at least one disjoint stock pool".to_owned());
    }
    let mut start_pools = BTreeMap::new();
    for pool in &input.storage_start {
        validate_pool(pool)?;
        if start_pools.insert(pool.pool_id.as_str(), pool.compartment).is_some() {
            return Err(format!("duplicate start storage pool_id {:?}", pool.pool_id));
        }
    }
    let mut end_pools = BTreeMap::new();
    for pool in &input.storage_end {
        validate_pool(pool)?;
        if end_pools.insert(pool.pool_id.as_str(), pool.compartment).is_some() {
            return Err(format!("duplicate end storage pool_id {:?}", pool.pool_id));
        }
    }
    if start_pools != end_pools {
        return Err("storage_start and storage_end must have identical pool_id and compartment sets; represent absent pools with an explicit zero stock".to_owned());
    }

    let mut indicator_ids = HashSet::new();
    for indicator in &input.availability_indicators {
        nonblank(&indicator.indicator_id, "availability indicator_id")?;
        if !indicator_ids.insert(indicator.indicator_id.clone()) {
            return Err(format!(
                "duplicate availability indicator_id {:?}",
                indicator.indicator_id
            ));
        }
        nonblank(&indicator.name, "availability indicator name")?;
        finite_nonnegative(indicator.value, "availability indicator value")?;
        nonblank(&indicator.unit, "availability indicator unit")?;
        finite_nonnegative(indicator.depth_top_cm, "availability depth_top_cm")?;
        if !indicator.depth_bottom_cm.is_finite()
            || indicator.depth_bottom_cm <= indicator.depth_top_cm
        {
            return Err("availability depth_bottom_cm must be finite and greater than depth_top_cm".to_owned());
        }
        if indicator.depth_top_cm < volume.top_depth_cm
            || indicator.depth_bottom_cm > volume.bottom_depth_cm
        {
            return Err(format!(
                "availability indicator {:?} lies outside the declared control-volume soil depth",
                indicator.indicator_id
            ));
        }
        nonblank(&indicator.method, "availability indicator method")?;
        nonblank(&indicator.evidence_ref, "availability indicator evidence_ref")?;
    }
    Ok(())
}

fn audit(bytes: &[u8]) -> Result<AuditReport, String> {
    if bytes.is_empty() {
        return Err("input budget is empty".to_owned());
    }
    let input: BudgetInput = serde_json::from_slice(bytes)
        .map_err(|error| format!("invalid nutrient budget JSON: {error}"))?;
    validate_input(&input)?;

    let input_total = checked_sum(
        input.inputs.iter().map(|flow| flow.amount_kg_element_per_ha),
        "input total",
    )?;
    let output_total = checked_sum(
        input.outputs.iter().map(|flow| flow.amount_kg_element_per_ha),
        "output total",
    )?;
    let storage_start_total = checked_sum(
        input
            .storage_start
            .iter()
            .map(|pool| pool.stock_kg_element_per_ha),
        "start storage total",
    )?;
    let storage_end_total = checked_sum(
        input
            .storage_end
            .iter()
            .map(|pool| pool.stock_kg_element_per_ha),
        "end storage total",
    )?;
    let storage_change = storage_end_total - storage_start_total;
    let residual = input_total - output_total - storage_change;
    if !storage_change.is_finite() || !residual.is_finite() {
        return Err("computed storage change or residual is non-finite".to_owned());
    }

    let mut uncertainty_bound = 0.0;
    let mut all_uncertainties_present = true;
    for uncertainty in input
        .inputs
        .iter()
        .map(|flow| flow.standard_uncertainty_kg_element_per_ha)
        .chain(
            input
                .outputs
                .iter()
                .map(|flow| flow.standard_uncertainty_kg_element_per_ha),
        )
        .chain(
            input
                .storage_start
                .iter()
                .map(|pool| pool.standard_uncertainty_kg_element_per_ha),
        )
        .chain(
            input
                .storage_end
                .iter()
                .map(|pool| pool.standard_uncertainty_kg_element_per_ha),
        )
    {
        if let Some(value) = uncertainty {
            uncertainty_bound += value;
            if !uncertainty_bound.is_finite() {
                return Err("uncertainty bound overflowed or became non-finite".to_owned());
            }
        } else {
            all_uncertainties_present = false;
        }
    }

    let uncertainty_option = all_uncertainties_present.then_some(uncertainty_bound);
    let status = match uncertainty_option {
        None => "unqualified_missing_uncertainty",
        Some(bound) => {
            let scale = input_total + output_total + storage_start_total + storage_end_total;
            let floating_point_guard = 16.0 * f64::EPSILON * scale.max(1.0);
            if residual.abs() <= bound + floating_point_guard {
                "within_conservative_uncertainty_bound"
            } else {
                "outside_conservative_uncertainty_bound"
            }
        }
    };

    let area = input.control_volume.area_ha;
    let scaled = |value: f64, name: &str| -> Result<f64, String> {
        let result = value * area;
        if !result.is_finite() {
            Err(format!("plot-scaled {name} is non-finite"))
        } else {
            Ok(result)
        }
    };
    let plot_uncertainty = uncertainty_option
        .map(|bound| scaled(bound, "uncertainty bound"))
        .transpose()?;

    let mut warnings = vec![
        "Arithmetic audit only: a residual within the uncertainty bound does not prove that all real-world fluxes were measured or that the control volume is complete.".to_owned(),
        "Internal transfers are not boundary inputs/outputs when both pools are inside the declared control volume; do not double-count root uptake, mineralization, immobilization, sorption/desorption or residue return.".to_owned(),
        "Availability indicators are method-dependent measurements and are reported separately; they are never added to the disjoint stock pools.".to_owned(),
        "The linear sum of standard uncertainties is a conservative bound under unspecified covariance, not a confidence interval and not a model of systematic bias.".to_owned(),
    ];
    if uncertainty_option.is_none() {
        warnings.push("At least one input, output or stock pool lacks standard uncertainty; the computed residual is unqualified.".to_owned());
    }
    if input.availability_indicators.is_empty() {
        warnings.push("No availability indicators were supplied; the mass balance does not establish plant-available nutrient status.".to_owned());
    }

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status,
        record_id: input.record_id,
        element: input.element,
        input_sha256: sha256_hex(bytes),
        study_id: input.study_id,
        site_id: input.site_id,
        experimental_unit_id: input.experimental_unit_id,
        treatment_arm_id: input.treatment_arm_id,
        comparator_arm_id: input.comparator_arm_id,
        period_start_date: input.period.start_date,
        period_end_date: input.period.end_date,
        control_volume: input.control_volume,
        input_total_kg_element_per_ha: input_total,
        output_total_kg_element_per_ha: output_total,
        storage_start_total_kg_element_per_ha: storage_start_total,
        storage_end_total_kg_element_per_ha: storage_end_total,
        storage_change_kg_element_per_ha: storage_change,
        residual_kg_element_per_ha: residual,
        conservative_linear_standard_uncertainty_bound_kg_element_per_ha: uncertainty_option,
        plot_scaled: PlotScaled {
            input_kg_element: scaled(input_total, "input total")?,
            output_kg_element: scaled(output_total, "output total")?,
            storage_change_kg_element: scaled(storage_change, "storage change")?,
            residual_kg_element: scaled(residual, "residual")?,
            uncertainty_bound_kg_element: plot_uncertainty,
        },
        flow_counts: FlowCounts {
            inputs: input.inputs.len(),
            outputs: input.outputs.len(),
            storage_pools: input.storage_start.len(),
            availability_indicators: input.availability_indicators.len(),
        },
        availability_indicators: input.availability_indicators,
        warnings,
        interpretation_boundary: "Elemental boundary-mass arithmetic audit only; not proof of flux completeness, causal effect, agronomic benefit, safety or regulatory approval.",
    })
}

fn ensure_distinct_paths(input: &Path, output: &Path) -> Result<(), Box<dyn Error>> {
    if output.exists() {
        return Err("output report already exists; refusing to overwrite an existing artifact".into());
    }
    let canonical_input = fs::canonicalize(input)?;
    if output == canonical_input {
        return Err("input and output paths must differ; refusing to overwrite source".into());
    }
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!(
            "usage: {} <nutrient-budget-input.json> <audit-report.json>",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-nutrient-budget-audit")
        )
        .into());
    }
    let input_path = Path::new(&arguments[1]);
    let output_path = Path::new(&arguments[2]);
    ensure_distinct_paths(input_path, output_path)?;
    let bytes = fs::read(input_path)?;
    let report = audit(&bytes).map_err(|message| io::Error::new(io::ErrorKind::InvalidData, message))?;
    let serialized = serde_json::to_vec_pretty(&report)?;
    fs::write(output_path, serialized)?;
    println!(
        "nutrient budget audited: element={:?}, status={}, residual={} kg/ha; report={}",
        report.element,
        report.status,
        report.residual_kg_element_per_ha,
        output_path.display()
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("nutrient budget audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn fixture_bytes() -> Vec<u8> {
        include_bytes!("../fixtures/synthetic-n-budget.json").to_vec()
    }

    fn fixture_report(bytes: &[u8]) -> AuditReport {
        audit(bytes).expect("synthetic fixture should audit")
    }

    #[test]
    fn computes_synthetic_mass_balance_and_hash() {
        let bytes = fixture_bytes();
        let report = fixture_report(&bytes);
        assert_eq!(report.input_total_kg_element_per_ha, 50.0);
        assert_eq!(report.output_total_kg_element_per_ha, 28.0);
        assert_eq!(report.storage_start_total_kg_element_per_ha, 100.0);
        assert_eq!(report.storage_end_total_kg_element_per_ha, 122.0);
        assert_eq!(report.storage_change_kg_element_per_ha, 22.0);
        assert!(report.residual_kg_element_per_ha.abs() < 1e-12);
        assert_eq!(
            report.input_sha256,
            sha256_hex(&bytes)
        );
        assert_eq!(report.status, "within_conservative_uncertainty_bound");
    }

    #[test]
    fn scales_per_hectare_values_to_declared_plot_area() {
        let report = fixture_report(&fixture_bytes());
        assert!((report.plot_scaled.input_kg_element - 12.5).abs() < 1e-12);
        assert!((report.plot_scaled.output_kg_element - 7.0).abs() < 1e-12);
        assert!((report.plot_scaled.storage_change_kg_element - 5.5).abs() < 1e-12);
    }

    #[test]
    fn reports_conservative_uncertainty_bound_without_calling_it_a_confidence_interval() {
        let report = fixture_report(&fixture_bytes());
        let bound = report
            .conservative_linear_standard_uncertainty_bound_kg_element_per_ha
            .unwrap();
        assert!((bound - 5.1).abs() < 1e-12);
        assert!(report.warnings.iter().any(|warning| warning.contains("not a confidence interval")));
    }

    #[test]
    fn availability_indicators_do_not_change_stock_or_residual() {
        let original = fixture_bytes();
        let first = fixture_report(&original);
        let mut value: Value = serde_json::from_slice(&original).unwrap();
        value["availability_indicators"][0]["value"] = Value::from(999_999.0);
        let changed = serde_json::to_vec(&value).unwrap();
        let second = fixture_report(&changed);
        assert_eq!(first.storage_start_total_kg_element_per_ha, second.storage_start_total_kg_element_per_ha);
        assert_eq!(first.storage_end_total_kg_element_per_ha, second.storage_end_total_kg_element_per_ha);
        assert_eq!(first.residual_kg_element_per_ha, second.residual_kg_element_per_ha);
    }

    #[test]
    fn rejects_negative_flow() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["inputs"][0]["amount_kg_element_per_ha"] = Value::from(-1.0);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("finite and non-negative"));
    }

    #[test]
    fn rejects_negative_uncertainty() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["outputs"][0]["standard_uncertainty_kg_element_per_ha"] = Value::from(-0.1);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("finite and non-negative"));
    }

    #[test]
    fn missing_uncertainty_prevents_closure_qualification() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["outputs"][0]["standard_uncertainty_kg_element_per_ha"] = Value::Null;
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = fixture_report(&bytes);
        assert_eq!(report.status, "unqualified_missing_uncertainty");
        assert_eq!(
            report.conservative_linear_standard_uncertainty_bound_kg_element_per_ha,
            None
        );
    }

    #[test]
    fn outside_uncertainty_bound_is_reported_not_repaired() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["outputs"][1]["amount_kg_element_per_ha"] = Value::from(40.0);
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = fixture_report(&bytes);
        assert_eq!(report.status, "outside_conservative_uncertainty_bound");
        assert!((report.residual_kg_element_per_ha + 35.0).abs() < 1e-12);
    }

    #[test]
    fn rejects_mismatched_start_and_end_pool_sets() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["storage_end"][0]["pool_id"] = Value::from("renamed-pool");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("identical pool_id"));
    }

    #[test]
    fn rejects_duplicate_start_pool_ids() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["storage_start"][1]["pool_id"] = Value::from("soil-n");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("duplicate start storage pool_id"));
    }

    #[test]
    fn rejects_duplicate_ids_across_input_and_output_flows() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["outputs"][0]["flow_id"] = Value::from("fertilizer-n");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("duplicate flow_id"));
    }

    #[test]
    fn rejects_wrong_elemental_basis() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["unit_basis"] = Value::from("kg_P2O5_per_ha");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("elemental mass explicitly"));
    }

    #[test]
    fn rejects_invalid_calendar_date() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["period"]["start_date"] = Value::from("2026-02-30");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("valid calendar dates"));
    }

    #[test]
    fn rejects_period_end_before_start() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["period"]["start_date"] = Value::from("2026-12-31");
        value["period"]["end_date"] = Value::from("2026-01-01");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("end_date precedes"));
    }

    #[test]
    fn rejects_availability_indicator_outside_control_volume_depth() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["availability_indicators"][0]["depth_bottom_cm"] = Value::from(45.0);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("outside the declared control-volume"));
    }

    #[test]
    fn rejects_unknown_json_fields() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["assume_phosphorus_is_plant_available"] = Value::Bool(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("unknown field"));
    }

    #[test]
    fn rejects_duplicate_availability_indicators() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["availability_indicators"].push(value["availability_indicators"][0].clone());
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("duplicate availability indicator_id"));
    }
}
