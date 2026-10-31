//! Descriptive audit of factorial, non-monotonic soil-response observations.
//!
//! The treatment-assignment unit is the replication unit. Repeated readings,
//! cores, sensors, events, and time points are nested observations.
//! This program does not estimate causal effects or statistical significance.

use csv::{ReaderBuilder, StringRecord};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const VERSION: &str = "0.1.0";
const COLUMNS: [&str; 19] = [
    "observation_id", "study_id", "site_id", "assignment_unit_id", "block_id",
    "soil_source_id", "material_batch_id", "amendment_family_id", "treatment_id",
    "dose", "dose_unit", "root_regime", "wet_dry_cycles", "antecedent_state",
    "forcing_id", "endpoint_id", "timepoint_id", "allocation_method", "value",
];

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize)]
struct Context {
    study_id: String,
    site_id: String,
    soil_source_id: String,
    amendment_family_id: String,
    treatment_id: String,
    dose_unit: String,
    root_regime: String,
    wet_dry_cycles: u32,
    antecedent_state: String,
    forcing_id: String,
    endpoint_id: String,
    timepoint_id: String,
    allocation_method: String,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct UnitKey(String, String);

#[derive(Clone, Debug, Eq, PartialEq)]
struct Assignment {
    site_id: String,
    soil_source_id: String,
    material_batch_id: String,
    amendment_family_id: String,
    treatment_id: String,
    dose_bits: u64,
    dose_unit: String,
    root_regime: String,
    cycles: u32,
    block_id: String,
    allocation_method: String,
}

#[derive(Default)]
struct UnitCell {
    sum: f64,
    rows: usize,
}

struct Parsed {
    rows: usize,
    assignments: BTreeMap<UnitKey, Assignment>,
    cells: BTreeMap<(Context, u64, UnitKey), UnitCell>,
}

struct UnitMean {
    value: f64,
    rows: usize,
    block: String,
    batch: String,
}

struct Aggregate {
    context: Context,
    doses: BTreeMap<u64, Vec<UnitMean>>,
}

#[derive(Serialize)]
struct DosePoint {
    dose: f64,
    assignment_unit_count: usize,
    observation_row_count: usize,
    mean_value: f64,
    sd_across_assignment_unit_means: Option<f64>,
    block_ids: Vec<String>,
    material_batch_ids: Vec<String>,
}

#[derive(Serialize)]
struct SeriesReport {
    series_id: String,
    context: Context,
    dose_points: Vec<DosePoint>,
    common_block_ids_across_all_doses: Vec<String>,
    common_block_count: usize,
    common_material_batch_ids_across_all_doses: Vec<String>,
    common_material_batch_count: usize,
    minimum_independent_units_per_dose: usize,
    shape_status: &'static str,
    reversal_detected: bool,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize)]
struct InteractionContext {
    study_id: String,
    site_id: String,
    soil_source_id: String,
    amendment_family_id: String,
    treatment_id: String,
    dose_unit: String,
    antecedent_state: String,
    forcing_id: String,
    endpoint_id: String,
    timepoint_id: String,
    allocation_method: String,
}

impl From<&Context> for InteractionContext {
    fn from(c: &Context) -> Self {
        Self {
            study_id: c.study_id.clone(),
            site_id: c.site_id.clone(),
            soil_source_id: c.soil_source_id.clone(),
            amendment_family_id: c.amendment_family_id.clone(),
            treatment_id: c.treatment_id.clone(),
            dose_unit: c.dose_unit.clone(),
            antecedent_state: c.antecedent_state.clone(),
            forcing_id: c.forcing_id.clone(),
            endpoint_id: c.endpoint_id.clone(),
            timepoint_id: c.timepoint_id.clone(),
            allocation_method: c.allocation_method.clone(),
        }
    }
}

#[derive(Serialize)]
struct InteractionReport {
    context: InteractionContext,
    factor_compared: &'static str,
    factor_level_a: String,
    factor_level_b: String,
    root_regime_a: String,
    wet_dry_cycles_a: u32,
    root_regime_b: String,
    wet_dry_cycles_b: u32,
    common_dose_values: Vec<f64>,
    assignment_units_a: usize,
    assignment_units_b: usize,
    dose_slope_a_per_dose_unit: f64,
    dose_slope_b_per_dose_unit: f64,
    slope_difference_a_minus_b: f64,
    status: &'static str,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    status: &'static str,
    input_label: String,
    input_sha256: String,
    observation_row_count: usize,
    assignment_unit_count: usize,
    minimum_independent_units_per_dose: usize,
    absolute_flat_tolerance_in_endpoint_units: f64,
    series_count: usize,
    non_monotonic_series_count: usize,
    interaction_contrast_count: usize,
    series: Vec<SeriesReport>,
    interaction_contrasts: Vec<InteractionReport>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn digest(bytes: &[u8]) -> String {
    let hash = Sha256::digest(bytes);
    hash.iter().map(|b| format!("{b:02x}")).collect()
}

fn column_index(headers: &StringRecord, name: &str) -> Result<usize, Box<dyn Error>> {
    let positions: Vec<_> = headers.iter().enumerate()
        .filter_map(|(i, h)| (h == name).then_some(i)).collect();
    match positions.as_slice() {
        [i] => Ok(*i),
        [] => Err(format!("missing required column {name:?}").into()),
        _ => Err(format!("duplicate column {name:?}").into()),
    }
}

fn read_text<'a>(
    row: &'a StringRecord,
    columns: &BTreeMap<&'static str, usize>,
    name: &'static str,
    row_number: usize,
) -> Result<&'a str, Box<dyn Error>> {
    let index = columns.get(name).ok_or("internal column map error")?;
    row.get(*index).ok_or_else(|| format!("missing {name} at data row {row_number}").into())
}

fn identifier(
    row: &StringRecord,
    columns: &BTreeMap<&'static str, usize>,
    name: &'static str,
    row_number: usize,
) -> Result<String, Box<dyn Error>> {
    let value = read_text(row, columns, name, row_number)?;
    let lower = value.to_ascii_lowercase();
    if value.trim() != value || value.is_empty()
        || ["na", "n/a", "none", "null", "unknown", "missing", "-"].contains(&lower.as_str())
    {
        return Err(format!("blank, placeholder, or whitespace-ambiguous {name} at row {row_number}").into());
    }
    Ok(value.to_owned())
}

fn parse(bytes: &[u8]) -> Result<Parsed, Box<dyn Error>> {
    if bytes.is_empty() {
        return Err("input CSV is empty".into());
    }
    let mut reader = ReaderBuilder::new().has_headers(true).flexible(false)
        .trim(csv::Trim::None).from_reader(bytes);
    let headers = reader.headers()?.clone();
    let mut columns = BTreeMap::new();
    for name in COLUMNS {
        columns.insert(name, column_index(&headers, name)?);
    }

    let mut seen = HashSet::new();
    let mut assignments = BTreeMap::<UnitKey, Assignment>::new();
    let mut cells = BTreeMap::<(Context, u64, UnitKey), UnitCell>::new();
    let mut row_count = 0;

    for (i, result) in reader.records().enumerate() {
        let row = result.map_err(|e| format!("invalid CSV row {}: {e}", i + 1))?;
        let n = i + 1;
        let observation_id = identifier(&row, &columns, "observation_id", n)?;
        if !seen.insert(observation_id.clone()) {
            return Err(format!("duplicate observation_id {observation_id:?}").into());
        }

        let study = identifier(&row, &columns, "study_id", n)?;
        let site = identifier(&row, &columns, "site_id", n)?;
        let unit_id = identifier(&row, &columns, "assignment_unit_id", n)?;
        let block = identifier(&row, &columns, "block_id", n)?;
        let soil = identifier(&row, &columns, "soil_source_id", n)?;
        let batch = identifier(&row, &columns, "material_batch_id", n)?;
        let amendment = identifier(&row, &columns, "amendment_family_id", n)?;
        let treatment = identifier(&row, &columns, "treatment_id", n)?;
        let dose_unit = identifier(&row, &columns, "dose_unit", n)?;
        let root = identifier(&row, &columns, "root_regime", n)?;
        let antecedent = identifier(&row, &columns, "antecedent_state", n)?;
        let forcing = identifier(&row, &columns, "forcing_id", n)?;
        let endpoint = identifier(&row, &columns, "endpoint_id", n)?;
        let timepoint = identifier(&row, &columns, "timepoint_id", n)?;
        let allocation = identifier(&row, &columns, "allocation_method", n)?;

        let dose_value: f64 = read_text(&row, &columns, "dose", n)?.parse()?;
        if !dose_value.is_finite() || dose_value < 0.0 {
            return Err(format!("dose must be finite and non-negative at row {n}").into());
        }
        let dose = if dose_value == 0.0 { 0.0 } else { dose_value };
        let bits = dose.to_bits();
        let cycles: u32 = read_text(&row, &columns, "wet_dry_cycles", n)?.parse()?;
        let value: f64 = read_text(&row, &columns, "value", n)?.parse()?;
        if !value.is_finite() {
            return Err(format!("outcome must be finite at row {n}").into());
        }

        let unit_key = UnitKey(study.clone(), unit_id);
        let assignment = Assignment {
            site_id: site.clone(),
            soil_source_id: soil.clone(),
            material_batch_id: batch.clone(),
            amendment_family_id: amendment.clone(),
            treatment_id: treatment.clone(),
            dose_bits: bits,
            dose_unit: dose_unit.clone(),
            root_regime: root.clone(),
            cycles,
            block_id: block.clone(),
            allocation_method: allocation.clone(),
        };
        if let Some(previous) = assignments.get(&unit_key) {
            if previous != &assignment {
                return Err(format!(
                    "assignment unit {:?} changes assigned condition or lineage",
                    unit_key.1
                ).into());
            }
        } else {
            assignments.insert(unit_key.clone(), assignment);
        }

        let context = Context {
            study_id: study,
            site_id: site,
            soil_source_id: soil,
            amendment_family_id: amendment,
            treatment_id: treatment,
            dose_unit,
            root_regime: root,
            wet_dry_cycles: cycles,
            antecedent_state: antecedent,
            forcing_id: forcing,
            endpoint_id: endpoint,
            timepoint_id: timepoint,
            allocation_method: allocation,
        };
        let cell = cells.entry((context, bits, unit_key)).or_default();
        cell.sum += value;
        if !cell.sum.is_finite() {
            return Err(format!("unit outcome overflow at row {n}").into());
        }
        cell.rows += 1;
        row_count += 1;
    }

    if row_count == 0 {
        return Err("input CSV contains no observations".into());
    }
    Ok(Parsed { rows: row_count, assignments, cells })
}

fn mean_sd(values: &[f64]) -> (f64, Option<f64>) {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    if values.len() < 2 {
        return (mean, None);
    }
    let ss = values.iter().map(|x| (x - mean).powi(2)).sum::<f64>();
    (mean, Some((ss / (values.len() - 1) as f64).sqrt()))
}

fn common_blocks(doses: &BTreeMap<u64, Vec<UnitMean>>) -> Vec<String> {
    let mut common: Option<BTreeSet<String>> = None;
    for units in doses.values() {
        let blocks: BTreeSet<String> = units.iter().map(|u| u.block.clone()).collect();
        common = Some(match common {
            Some(previous) => previous.intersection(&blocks).cloned().collect(),
            None => blocks,
        });
    }
    common.unwrap_or_default().into_iter().collect()
}

fn common_batches(doses: &BTreeMap<u64, Vec<UnitMean>>) -> Vec<String> {
    let mut common: Option<BTreeSet<String>> = None;
    for units in doses.values() {
        let batches: BTreeSet<String> = units.iter().map(|u| u.batch.clone()).collect();
        common = Some(match common {
            Some(previous) => previous.intersection(&batches).cloned().collect(),
            None => batches,
        });
    }
    common.unwrap_or_default().into_iter().collect()
}

fn shape(doses: &BTreeMap<u64, Vec<UnitMean>>, floor: usize, tol: f64) -> &'static str {
    if doses.values().any(|units| units.len() < floor) {
        return "insufficient_independent_replication";
    }
    if doses.len() < 3 {
        return "not_evaluable_two_or_fewer_dose_levels";
    }
    let means: Vec<f64> = doses.values().map(|units| {
        units.iter().map(|u| u.value).sum::<f64>() / units.len() as f64
    }).collect();
    let mut up = false;
    let mut down = false;
    for pair in means.windows(2) {
        if pair[1] - pair[0] > tol { up = true; }
        if pair[1] - pair[0] < -tol { down = true; }
    }
    match (up, down) {
        (true, true) => "non_monotonic_reversal",
        (true, false) => "monotonic_increasing_or_plateau",
        (false, true) => "monotonic_decreasing_or_plateau",
        (false, false) => "flat_within_tolerance",
    }
}

fn slope(doses: &BTreeMap<u64, Vec<UnitMean>>, common: &BTreeSet<u64>) -> Option<f64> {
    // Fit to dose-level means so unequal unit counts do not silently change
    // the weight of a dose level in the descriptive contrast.
    let data: Vec<(f64, f64)> = common.iter().filter_map(|bits| {
        let units = doses.get(bits)?;
        let mean = units.iter().map(|unit| unit.value).sum::<f64>() / units.len() as f64;
        Some((f64::from_bits(*bits), mean))
    }).collect();
    if data.len() < 2 { return None; }
    let n = data.len() as f64;
    let sx = data.iter().map(|(x, _)| x).sum::<f64>();
    let sy = data.iter().map(|(_, y)| y).sum::<f64>();
    let sxx = data.iter().map(|(x, _)| x*x).sum::<f64>();
    let sxy = data.iter().map(|(x, y)| x*y).sum::<f64>();
    let den = n*sxx - sx*sx;
    if den.abs() <= f64::EPSILON { return None; }
    Some((n*sxy - sx*sy)/den)
}

fn interactions(aggregates: &[Aggregate], floor: usize) -> Vec<InteractionReport> {
    let mut groups = BTreeMap::<InteractionContext, Vec<usize>>::new();
    for (i, item) in aggregates.iter().enumerate() {
        groups.entry(InteractionContext::from(&item.context)).or_default().push(i);
    }
    let mut output = Vec::new();
    for (base, indices) in groups {
        for a in 0..indices.len() {
            for b in (a+1)..indices.len() {
                let left = &aggregates[indices[a]];
                let right = &aggregates[indices[b]];
                let root_diff = left.context.root_regime != right.context.root_regime;
                let cycle_diff = left.context.wet_dry_cycles != right.context.wet_dry_cycles;
                if root_diff == cycle_diff { continue; }

                let common: BTreeSet<u64> = left.doses.iter().filter_map(|(dose, lu)| {
                    let ru = right.doses.get(dose)?;
                    (lu.len() >= floor && ru.len() >= floor).then_some(*dose)
                }).collect();
                if common.len() < 2 { continue; }
                let (Some(sa), Some(sb)) = (slope(&left.doses, &common), slope(&right.doses, &common)) else {
                    continue;
                };
                let units_a: usize = common.iter().map(|d| left.doses[d].len()).sum();
                let units_b: usize = common.iter().map(|d| right.doses[d].len()).sum();
                let factor = if root_diff { "root_regime" } else { "wet_dry_cycles" };
                output.push(InteractionReport {
                    context: base.clone(),
                    factor_compared: factor,
                    factor_level_a: if root_diff { left.context.root_regime.clone() } else { left.context.wet_dry_cycles.to_string() },
                    factor_level_b: if root_diff { right.context.root_regime.clone() } else { right.context.wet_dry_cycles.to_string() },
                    root_regime_a: left.context.root_regime.clone(),
                    wet_dry_cycles_a: left.context.wet_dry_cycles,
                    root_regime_b: right.context.root_regime.clone(),
                    wet_dry_cycles_b: right.context.wet_dry_cycles,
                    common_dose_values: common.iter().map(|d| f64::from_bits(*d)).collect(),
                    assignment_units_a: units_a,
                    assignment_units_b: units_b,
                    dose_slope_a_per_dose_unit: sa,
                    dose_slope_b_per_dose_unit: sb,
                    slope_difference_a_minus_b: sa - sb,
                    status: "descriptive_only_no_inference",
                });
            }
        }
    }
    output
}

fn audit(bytes: &[u8], label: &str, floor: usize, tolerance: f64) -> Result<Report, Box<dyn Error>> {
    let parsed = parse(bytes)?;
    let mut grouped = BTreeMap::<Context, BTreeMap<u64, Vec<UnitMean>>>::new();

    for ((context, dose, key), cell) in parsed.cells {
        let assignment = parsed.assignments.get(&key).ok_or("missing assignment metadata")?;
        grouped.entry(context).or_default().entry(dose).or_default().push(UnitMean {
            value: cell.sum / cell.rows as f64,
            rows: cell.rows,
            block: assignment.block_id.clone(),
            batch: assignment.material_batch_id.clone(),
        });
    }

    let aggregates: Vec<Aggregate> = grouped.into_iter()
        .map(|(context, doses)| Aggregate { context, doses }).collect();
    let mut reports = Vec::new();
    for (index, item) in aggregates.iter().enumerate() {
        let status = shape(&item.doses, floor, tolerance);
        let blocks = common_blocks(&item.doses);
        let batches = common_batches(&item.doses);
        let mut points = Vec::new();
        for (dose, units) in &item.doses {
            let values: Vec<f64> = units.iter().map(|u| u.value).collect();
            let (mean, sd) = mean_sd(&values);
            let block_ids: BTreeSet<String> = units.iter().map(|u| u.block.clone()).collect();
            let batches: BTreeSet<String> = units.iter().map(|u| u.batch.clone()).collect();
            points.push(DosePoint {
                dose: f64::from_bits(*dose),
                assignment_unit_count: units.len(),
                observation_row_count: units.iter().map(|u| u.rows).sum(),
                mean_value: mean,
                sd_across_assignment_unit_means: sd,
                block_ids: block_ids.into_iter().collect(),
                material_batch_ids: batches.into_iter().collect(),
            });
        }
        reports.push(SeriesReport {
            series_id: format!("series-{:03}", index + 1),
            context: item.context.clone(),
            dose_points: points,
            common_block_count: blocks.len(),
            common_block_ids_across_all_doses: blocks,
            common_material_batch_count: batches.len(),
            common_material_batch_ids_across_all_doses: batches,
            minimum_independent_units_per_dose: floor,
            shape_status: status,
            reversal_detected: status == "non_monotonic_reversal",
        });
    }

    let reversals = reports.iter().filter(|s| s.reversal_detected).count();
    let contrasts = interactions(&aggregates, floor);
    let mut warnings = vec![
        "Repeated readings are averaged within assignment unit and response cell; observation rows are not independent treatment replicates.".to_owned(),
        "Allocation labels and experimental-unit IDs are supplied metadata, not independently verified randomization evidence.".to_owned(),
        "Dose shapes and root/cycle slope contrasts are descriptive only; no p-values, intervals, causal effects, or recommendations are produced.".to_owned(),
        "Slope contrasts are unadjusted for possible batch, site, or study confounding.".to_owned(),
    ];
    if reports.iter().any(|s| s.common_block_count == 0 && s.dose_points.len() > 1) {
        warnings.push("At least one response series has no common block represented at every dose.".to_owned());
    }
    if reports.iter().any(|s| s.common_material_batch_count == 0 && s.dose_points.len() > 1) {
        warnings.push("At least one response series has no material batch represented at every dose; dose and batch effects may be confounded.".to_owned());
    }
    if reports.iter().any(|s| s.shape_status == "insufficient_independent_replication") {
        warnings.push("At least one series abstains because a dose cell is below the independent-unit floor.".to_owned());
    }
    if reversals > 0 {
        warnings.push(format!("{reversals} series show a descriptive direction reversal; review uncertainty and design before interpreting."));
    }

    Ok(Report {
        audit_version: VERSION,
        status: "descriptive_only",
        input_label: label.to_owned(),
        input_sha256: digest(bytes),
        observation_row_count: parsed.rows,
        assignment_unit_count: parsed.assignments.len(),
        minimum_independent_units_per_dose: floor,
        absolute_flat_tolerance_in_endpoint_units: tolerance,
        series_count: reports.len(),
        non_monotonic_series_count: reversals,
        interaction_contrast_count: contrasts.len(),
        series: reports,
        interaction_contrasts: contrasts,
        warnings,
        interpretation_boundary: "Descriptive response-surface audit only; not causal inference, statistical significance, field efficacy, a dose recommendation, or application authorization.",
    })
}

fn main_run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 || args.len() > 5 {
        return Err(format!("usage: {} <observations.csv> <report.json> [minimum-independent-units-per-dose] [absolute-flat-tolerance]", args.first().map(String::as_str).unwrap_or("terra-preta-response-surface-audit")).into());
    }
    let floor: usize = args.get(3).map(|v| v.parse()).transpose()?.unwrap_or(3);
    if floor == 0 { return Err("minimum independent units per dose must be at least 1".into()); }
    let tolerance: f64 = args.get(4).map(|v| v.parse()).transpose()?.unwrap_or(0.0);
    if !tolerance.is_finite() || tolerance < 0.0 { return Err("flat tolerance must be finite and non-negative".into()); }
    let input = Path::new(&args[1]);
    let output = Path::new(&args[2]);
    if output.exists() && fs::canonicalize(input)? == fs::canonicalize(output)? {
        return Err("input and output paths resolve to the same file".into());
    }
    let bytes = fs::read(input)?;
    let label = input.file_name().and_then(|v| v.to_str()).unwrap_or("observations.csv");
    let report = audit(&bytes, label, floor, tolerance)?;
    fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!("descriptive response-surface audit: {} observations, {} assignment units, {} series, {} reversals, {} slope contrasts", report.observation_row_count, report.assignment_unit_count, report.series_count, report.non_monotonic_series_count, report.interaction_contrast_count);
    Ok(())
}

fn main() {
    if let Err(error) = main_run() {
        eprintln!("response-surface audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-factorial-response.csv");

    #[test]
    fn detects_synthetic_reversals_and_root_cycle_conditioning() {
        let report = audit(FIXTURE.as_bytes(), "synthetic.csv", 3, 0.0).unwrap();
        assert_eq!(report.observation_row_count, 108);
        assert_eq!(report.assignment_unit_count, 36);
        assert_eq!(report.series_count, 8);
        assert_eq!(report.non_monotonic_series_count, 2);
        assert_eq!(report.interaction_contrast_count, 8);
    }

    #[test]
    fn repeated_readings_do_not_inflate_independent_replication() {
        let report = audit(FIXTURE.as_bytes(), "synthetic.csv", 3, 0.0).unwrap();
        let series = report.series.iter().find(|s|
            s.context.root_regime == "bare" && s.context.wet_dry_cycles == 0
                && s.context.timepoint_id == "day_1").unwrap();
        let middle = series.dose_points.iter().find(|p| p.dose == 5.0).unwrap();
        assert_eq!(middle.assignment_unit_count, 3);
        assert_eq!(middle.observation_row_count, 6);
    }

    #[test]
    fn rejects_assignment_unit_with_changed_dose() {
        let mut lines: Vec<String> = FIXTURE.lines().map(str::to_owned).collect();
        let line = lines.iter_mut().find(|l| l.starts_with("obs-bare-c0-d5-r1-day_1,")).unwrap();
        let mut fields: Vec<String> = line.split(',').map(str::to_owned).collect();
        fields[9] = "6".to_owned();
        *line = fields.join(",");
        assert!(audit(lines.join("\n").as_bytes(), "bad.csv", 3, 0.0).is_err());
    }

    #[test]
    fn rejects_duplicate_observation_ids() {
        let mut lines: Vec<String> = FIXTURE.lines().map(str::to_owned).collect();
        let id = lines[1].split(',').next().unwrap().to_owned();
        let mut fields: Vec<String> = lines[2].split(',').map(str::to_owned).collect();
        fields[0] = id;
        lines[2] = fields.join(",");
        assert!(audit(lines.join("\n").as_bytes(), "duplicate.csv", 3, 0.0).is_err());
    }

    #[test]
    fn abstains_below_replication_floor() {
        let report = audit(FIXTURE.as_bytes(), "synthetic.csv", 4, 0.0).unwrap();
        assert_eq!(report.non_monotonic_series_count, 0);
        assert_eq!(report.interaction_contrast_count, 0);
        assert!(report.series.iter().all(|s| s.shape_status == "insufficient_independent_replication"));
    }

    #[test]
    fn rejects_non_finite_dose() {
        let mutated = FIXTURE.replacen(",0,percent_dry_mass,bare,0,", ",NaN,percent_dry_mass,bare,0,", 1);
        assert!(audit(mutated.as_bytes(), "nan.csv", 3, 0.0).is_err());
    }
}
