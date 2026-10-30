use chrono::{DateTime, Duration, FixedOffset};
#[cfg(test)]
use chrono::{TimeZone, Utc};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, fs::OpenOptions, io::{self, Write}};

const SUPPORTED_DT_SECONDS: i64 = 600;
const BALANCE_TOLERANCE: f64 = 1.0e-10;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ScenarioFamily {
    MatrixOnly,
    ConnectedMacropore,
    DisconnectedMacropore,
    DualDomainExchange,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    scenario_id: String,
    dt_seconds: i64,
    family: ScenarioFamily,
    parameters: Parameters,
    initial_state: InitialState,
    forcing: Vec<ForcingStep>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Parameters {
    matrix_capacity_mm: f64,
    macropore_capacity_mm: f64,
    macropore_inflow_fraction: f64,
    matrix_drainage_fraction_per_step: f64,
    macropore_drainage_fraction_per_step: f64,
    interdomain_exchange_fraction_per_step: f64,
    macropore_path_connected: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct InitialState {
    matrix_water_mm: f64,
    macropore_water_mm: f64,
    matrix_tracer_mg_m2: f64,
    macropore_tracer_mg_m2: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ForcingStep {
    timestamp_utc: String,
    rainfall_mm: f64,
    potential_evaporation_mm: f64,
    rainfall_tracer_concentration_mg_l: f64,
}

#[derive(Clone, Debug)]
struct ParsedForcing {
    timestamp: DateTime<FixedOffset>,
    rainfall_mm: f64,
    potential_evaporation_mm: f64,
    rainfall_tracer_concentration_mg_l: f64,
}

#[derive(Clone, Copy, Debug)]
struct Pool {
    water_mm: f64,
    tracer_mg_m2: f64,
    capacity_mm: f64,
}

impl Pool {
    fn concentration_mg_l(&self) -> f64 {
        if self.water_mm > 0.0 {
            self.tracer_mg_m2 / self.water_mm
        } else {
            0.0
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct StepResult {
    step_index: usize,
    timestamp_utc: String,
    rainfall_input_mm: f64,
    tracer_input_mg_m2: f64,
    evaporation_mm: f64,
    matrix_water_mm: f64,
    macropore_water_mm: f64,
    matrix_tracer_mg_m2: f64,
    macropore_tracer_mg_m2: f64,
    matrix_deep_drainage_mm: f64,
    macropore_deep_drainage_mm: f64,
    matrix_deep_tracer_mg_m2: f64,
    macropore_deep_tracer_mg_m2: f64,
    surface_runoff_mm: f64,
    lateral_macropore_loss_mm: f64,
    surface_runoff_tracer_mg_m2: f64,
    lateral_macropore_tracer_mg_m2: f64,
    matrix_to_macropore_exchange_mm: f64,
    macropore_to_matrix_exchange_mm: f64,
    water_balance_residual_mm: f64,
    tracer_balance_residual_mg_m2: f64,
}

#[derive(Clone, Debug, Serialize)]
struct Report {
    tool_version: &'static str,
    input_sha256: String,
    scenario_id: String,
    scenario_family: ScenarioFamily,
    dt_seconds: i64,
    true_macropore_path_connected: bool,
    parameters: ParameterRecord,
    step_count: usize,
    total_rainfall_mm: f64,
    total_tracer_input_mg_m2: f64,
    total_evaporation_mm: f64,
    total_surface_runoff_mm: f64,
    total_lateral_macropore_loss_mm: f64,
    total_matrix_deep_drainage_mm: f64,
    total_macropore_deep_drainage_mm: f64,
    total_matrix_deep_tracer_mg_m2: f64,
    total_macropore_deep_tracer_mg_m2: f64,
    max_abs_water_balance_residual_mm: f64,
    max_abs_tracer_balance_residual_mg_m2: f64,
    final_matrix_water_mm: f64,
    final_macropore_water_mm: f64,
    final_matrix_tracer_mg_m2: f64,
    final_macropore_tracer_mg_m2: f64,
    steps: Vec<StepResult>,
    model_boundary: &'static str,
    warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct ParameterRecord {
    matrix_capacity_mm: f64,
    macropore_capacity_mm: f64,
    macropore_inflow_fraction: f64,
    matrix_drainage_fraction_per_step: f64,
    macropore_drainage_fraction_per_step: f64,
    interdomain_exchange_fraction_per_step: f64,
}

fn finite_nonnegative(value: f64, label: &str) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{label} must be finite and non-negative"));
    }
    Ok(())
}

fn parse_utc(value: &str, label: &str) -> Result<DateTime<FixedOffset>, String> {
    let parsed = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if parsed.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp_utc must use a zero UTC offset"));
    }
    if parsed.timestamp() % SUPPORTED_DT_SECONDS != 0 || parsed.timestamp_subsec_nanos() != 0 {
        return Err(format!("{label}: timestamp is not aligned to a 10-minute UTC boundary"));
    }
    Ok(parsed)
}

fn validate(input: &Input) -> Result<(Vec<ParsedForcing>, Pool, Pool), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    if input.scenario_id.trim().is_empty() {
        return Err("scenario_id must not be blank".to_owned());
    }
    if input.dt_seconds != SUPPORTED_DT_SECONDS {
        return Err("v0.1 requires a 600-second (10-minute) time step".to_owned());
    }
    if input.forcing.is_empty() {
        return Err("forcing must contain at least one step".to_owned());
    }

    let p = &input.parameters;
    for (value, label) in [
        (p.matrix_capacity_mm, "matrix_capacity_mm"),
        (p.macropore_capacity_mm, "macropore_capacity_mm"),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(format!("{label} must be finite and greater than zero"));
        }
    }
    for (value, label) in [
        (p.macropore_inflow_fraction, "macropore_inflow_fraction"),
        (p.matrix_drainage_fraction_per_step, "matrix_drainage_fraction_per_step"),
        (p.macropore_drainage_fraction_per_step, "macropore_drainage_fraction_per_step"),
        (p.interdomain_exchange_fraction_per_step, "interdomain_exchange_fraction_per_step"),
    ] {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(format!("{label} must be finite and within [0, 1]"));
        }
    }

    match input.family {
        ScenarioFamily::MatrixOnly => {
            if p.macropore_inflow_fraction != 0.0 || p.macropore_path_connected {
                return Err("matrix_only requires zero macropore inflow fraction and disconnected macropore path".to_owned());
            }
        }
        ScenarioFamily::ConnectedMacropore => {
            if p.macropore_inflow_fraction <= 0.0 || !p.macropore_path_connected {
                return Err("connected_macropore requires positive macropore partition and connected path".to_owned());
            }
        }
        ScenarioFamily::DisconnectedMacropore => {
            if p.macropore_inflow_fraction <= 0.0 || p.macropore_path_connected {
                return Err("disconnected_macropore requires positive macropore partition and disconnected path".to_owned());
            }
        }
        ScenarioFamily::DualDomainExchange => {
            if p.macropore_inflow_fraction <= 0.0
                || !p.macropore_path_connected
                || p.interdomain_exchange_fraction_per_step <= 0.0
            {
                return Err("dual_domain_exchange requires macropore inflow, connected path and positive exchange".to_owned());
            }
        }
    }

    let initial = &input.initial_state;
    for (value, label) in [
        (initial.matrix_water_mm, "initial matrix water"),
        (initial.macropore_water_mm, "initial macropore water"),
        (initial.matrix_tracer_mg_m2, "initial matrix tracer"),
        (initial.macropore_tracer_mg_m2, "initial macropore tracer"),
    ] {
        finite_nonnegative(value, label)?;
    }
    if initial.matrix_water_mm > p.matrix_capacity_mm
        || initial.macropore_water_mm > p.macropore_capacity_mm
    {
        return Err("initial water storage must not exceed its domain capacity".to_owned());
    }
    if initial.matrix_water_mm == 0.0 && initial.matrix_tracer_mg_m2 > 0.0 {
        return Err("matrix tracer mass requires nonzero matrix water storage".to_owned());
    }
    if initial.macropore_water_mm == 0.0 && initial.macropore_tracer_mg_m2 > 0.0 {
        return Err("macropore tracer mass requires nonzero macropore water storage".to_owned());
    }

    let mut forcing = Vec::with_capacity(input.forcing.len());
    for (index, sample) in input.forcing.iter().enumerate() {
        let timestamp = parse_utc(&sample.timestamp_utc, &format!("forcing[{index}]"))?;
        finite_nonnegative(sample.rainfall_mm, &format!("forcing[{index}].rainfall_mm"))?;
        finite_nonnegative(sample.potential_evaporation_mm, &format!("forcing[{index}].potential_evaporation_mm"))?;
        finite_nonnegative(sample.rainfall_tracer_concentration_mg_l, &format!("forcing[{index}].rainfall_tracer_concentration_mg_l"))?;
        if index > 0 {
            let elapsed = (timestamp - forcing[index - 1].timestamp).num_seconds();
            if elapsed != SUPPORTED_DT_SECONDS {
                return Err(format!("forcing[{index}] must be contiguous at 10-minute cadence; found {elapsed} seconds"));
            }
        }
        forcing.push(ParsedForcing {
            timestamp,
            rainfall_mm: sample.rainfall_mm,
            potential_evaporation_mm: sample.potential_evaporation_mm,
            rainfall_tracer_concentration_mg_l: sample.rainfall_tracer_concentration_mg_l,
        });
    }

    let matrix = Pool {
        water_mm: initial.matrix_water_mm,
        tracer_mg_m2: initial.matrix_tracer_mg_m2,
        capacity_mm: p.matrix_capacity_mm,
    };
    let macropore = Pool {
        water_mm: initial.macropore_water_mm,
        tracer_mg_m2: initial.macropore_tracer_mg_m2,
        capacity_mm: p.macropore_capacity_mm,
    };
    Ok((forcing, matrix, macropore))
}

fn add_water_and_tracer(pool: &mut Pool, water_mm: f64, tracer_mg_m2: f64) -> (f64, f64) {
    pool.water_mm += water_mm;
    pool.tracer_mg_m2 += tracer_mg_m2;
    let overflow_mm = (pool.water_mm - pool.capacity_mm).max(0.0);
    if overflow_mm == 0.0 {
        return (0.0, 0.0);
    }
    let overflow_concentration = pool.concentration_mg_l();
    let overflow_tracer = overflow_mm * overflow_concentration;
    pool.water_mm -= overflow_mm;
    pool.tracer_mg_m2 -= overflow_tracer;
    (overflow_mm, overflow_tracer)
}

fn exchange(matrix: &mut Pool, macropore: &mut Pool, fraction: f64) -> (f64, f64) {
    if fraction == 0.0 {
        return (0.0, 0.0);
    }
    let matrix_saturation = matrix.water_mm / matrix.capacity_mm;
    let macropore_saturation = macropore.water_mm / macropore.capacity_mm;
    let capacity_scale = matrix.capacity_mm.min(macropore.capacity_mm);
    let requested = fraction * (macropore_saturation - matrix_saturation).abs() * capacity_scale;

    if macropore_saturation > matrix_saturation {
        let transferable = requested
            .min(macropore.water_mm)
            .min(matrix.capacity_mm - matrix.water_mm);
        if transferable == 0.0 {
            return (0.0, 0.0);
        }
        let mass = transferable * macropore.concentration_mg_l();
        macropore.water_mm -= transferable;
        macropore.tracer_mg_m2 -= mass;
        matrix.water_mm += transferable;
        matrix.tracer_mg_m2 += mass;
        (0.0, transferable)
    } else {
        let transferable = requested
            .min(matrix.water_mm)
            .min(macropore.capacity_mm - macropore.water_mm);
        if transferable == 0.0 {
            return (0.0, 0.0);
        }
        let mass = transferable * matrix.concentration_mg_l();
        matrix.water_mm -= transferable;
        matrix.tracer_mg_m2 -= mass;
        macropore.water_mm += transferable;
        macropore.tracer_mg_m2 += mass;
        (transferable, 0.0)
    }
}

fn drain(pool: &mut Pool, fraction: f64) -> (f64, f64) {
    let water = (pool.water_mm * fraction).min(pool.water_mm);
    let tracer = water * pool.concentration_mg_l();
    pool.water_mm -= water;
    pool.tracer_mg_m2 -= tracer;
    (water, tracer)
}

fn simulate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let (forcing, mut matrix, mut macropore) = validate(input)?;
    let p = &input.parameters;
    let mut steps = Vec::with_capacity(forcing.len());

    for (index, sample) in forcing.iter().enumerate() {
        let initial_water = matrix.water_mm + macropore.water_mm;
        let initial_tracer = matrix.tracer_mg_m2 + macropore.tracer_mg_m2;
        let tracer_input = sample.rainfall_mm * sample.rainfall_tracer_concentration_mg_l;
        let macropore_rain = sample.rainfall_mm * p.macropore_inflow_fraction;
        let matrix_rain = sample.rainfall_mm - macropore_rain;
        let macropore_tracer_input = tracer_input * p.macropore_inflow_fraction;
        let matrix_tracer_input = tracer_input - macropore_tracer_input;

        let (surface_runoff, surface_runoff_tracer) =
            add_water_and_tracer(&mut matrix, matrix_rain, matrix_tracer_input);
        let (lateral_macropore_loss, lateral_macropore_tracer) =
            add_water_and_tracer(&mut macropore, macropore_rain, macropore_tracer_input);

        let (matrix_to_macropore_exchange, macropore_to_matrix_exchange) =
            exchange(&mut matrix, &mut macropore, p.interdomain_exchange_fraction_per_step);

        let (matrix_drainage, matrix_drainage_tracer) =
            drain(&mut matrix, p.matrix_drainage_fraction_per_step);
        let (macropore_drainage, macropore_drainage_tracer) = if p.macropore_path_connected {
            drain(&mut macropore, p.macropore_drainage_fraction_per_step)
        } else {
            (0.0, 0.0)
        };

        let evaporation = sample.potential_evaporation_mm.min(matrix.water_mm);
        matrix.water_mm -= evaporation;

        let final_water = matrix.water_mm + macropore.water_mm;
        let final_tracer = matrix.tracer_mg_m2 + macropore.tracer_mg_m2;
        let water_residual = initial_water + sample.rainfall_mm
            - final_water
            - surface_runoff
            - lateral_macropore_loss
            - matrix_drainage
            - macropore_drainage
            - evaporation;
        let tracer_residual = initial_tracer + tracer_input
            - final_tracer
            - surface_runoff_tracer
            - lateral_macropore_tracer
            - matrix_drainage_tracer
            - macropore_drainage_tracer;

        if water_residual.abs() > BALANCE_TOLERANCE || tracer_residual.abs() > BALANCE_TOLERANCE {
            return Err(format!(
                "conservation failure at step {index}: water residual {water_residual:e} mm; tracer residual {tracer_residual:e} mg/m2"
            ));
        }

        steps.push(StepResult {
            step_index: index + 1,
            timestamp_utc: sample.timestamp.to_rfc3339(),
            rainfall_input_mm: sample.rainfall_mm,
            tracer_input_mg_m2: tracer_input,
            evaporation_mm: evaporation,
            matrix_water_mm: matrix.water_mm,
            macropore_water_mm: macropore.water_mm,
            matrix_tracer_mg_m2: matrix.tracer_mg_m2,
            macropore_tracer_mg_m2: macropore.tracer_mg_m2,
            matrix_deep_drainage_mm: matrix_drainage,
            macropore_deep_drainage_mm: macropore_drainage,
            matrix_deep_tracer_mg_m2: matrix_drainage_tracer,
            macropore_deep_tracer_mg_m2: macropore_drainage_tracer,
            surface_runoff_mm: surface_runoff,
            lateral_macropore_loss_mm: lateral_macropore_loss,
            surface_runoff_tracer_mg_m2: surface_runoff_tracer,
            lateral_macropore_tracer_mg_m2: lateral_macropore_tracer,
            matrix_to_macropore_exchange_mm: matrix_to_macropore_exchange,
            macropore_to_matrix_exchange_mm: macropore_to_matrix_exchange,
            water_balance_residual_mm: water_residual,
            tracer_balance_residual_mg_m2: tracer_residual,
        });
    }

    let sum = |f: fn(&StepResult) -> f64| -> f64 { steps.iter().map(f).sum() };
    let max_abs = |f: fn(&StepResult) -> f64| -> f64 {
        steps.iter().map(|step| f(step).abs()).fold(0.0, f64::max)
    };
    let total_rainfall = sum(|step| step.rainfall_input_mm);
    let total_tracer_input = sum(|step| step.tracer_input_mg_m2);
    let total_evaporation = sum(|step| step.evaporation_mm);
    let total_surface_runoff = sum(|step| step.surface_runoff_mm);
    let total_lateral_loss = sum(|step| step.lateral_macropore_loss_mm);
    let total_matrix_drainage = sum(|step| step.matrix_deep_drainage_mm);
    let total_macropore_drainage = sum(|step| step.macropore_deep_drainage_mm);

    Ok(Report {
        tool_version: "0.1.0",
        input_sha256,
        scenario_id: input.scenario_id.clone(),
        scenario_family: input.family,
        dt_seconds: input.dt_seconds,
        true_macropore_path_connected: p.macropore_path_connected,
        parameters: ParameterRecord {
            matrix_capacity_mm: p.matrix_capacity_mm,
            macropore_capacity_mm: p.macropore_capacity_mm,
            macropore_inflow_fraction: p.macropore_inflow_fraction,
            matrix_drainage_fraction_per_step: p.matrix_drainage_fraction_per_step,
            macropore_drainage_fraction_per_step: p.macropore_drainage_fraction_per_step,
            interdomain_exchange_fraction_per_step: p.interdomain_exchange_fraction_per_step,
        },
        step_count: steps.len(),
        total_rainfall_mm: total_rainfall,
        total_tracer_input_mg_m2: total_tracer_input,
        total_evaporation_mm: total_evaporation,
        total_surface_runoff_mm: total_surface_runoff,
        total_lateral_macropore_loss_mm: total_lateral_loss,
        total_matrix_deep_drainage_mm: total_matrix_drainage,
        total_macropore_deep_drainage_mm: total_macropore_drainage,
        total_matrix_deep_tracer_mg_m2: sum(|step| step.matrix_deep_tracer_mg_m2),
        total_macropore_deep_tracer_mg_m2: sum(|step| step.macropore_deep_tracer_mg_m2),
        max_abs_water_balance_residual_mm: max_abs(|step| step.water_balance_residual_mm),
        max_abs_tracer_balance_residual_mg_m2: max_abs(|step| step.tracer_balance_residual_mg_m2),
        final_matrix_water_mm: matrix.water_mm,
        final_macropore_water_mm: macropore.water_mm,
        final_matrix_tracer_mg_m2: matrix.tracer_mg_m2,
        final_macropore_tracer_mg_m2: macropore.tracer_mg_m2,
        steps,
        model_boundary: "deterministic two-reservoir bucket model with finite capacities, first-order drainage and advective inter-domain exchange; not Richards' equation, not a spatial soil profile solver, and not a field-calibrated hydraulic model",
        warnings: vec![
            "This generator creates controlled known-truth cases for software, conservation, identifiability and observation-operator tests; it is not a prediction of real soil behavior.".to_owned(),
            "The reservoirs are well-mixed conceptual domains. No pore geometry, vertical discretization, Richards pressure-gradient solution, dispersion, capillarity or hysteresis is represented.".to_owned(),
            "The disconnected macropore domain has no direct lower-boundary drainage; its water can exchange into the matrix or leave through capacity overflow classified as lateral loss.".to_owned(),
            "Tracer transfer is advective and perfectly mixed within each reservoir; molecular diffusion, adsorption, decay and kinetic reactions are omitted.".to_owned(),
            "All parameter fractions are per 10-minute step and must not be interpreted as calibrated hydraulic conductivity values.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: known-truth-generator <scenario-input.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = simulate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "known-truth scenario complete: {} steps; rainfall {:.3} mm; macro boundary drainage {:.3} mm; max balance residual {:.3e} mm",
        report.step_count,
        report.total_rainfall_mm,
        report.total_macropore_deep_drainage_mm,
        report.max_abs_water_balance_residual_mm
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("known-truth generator failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(minutes: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 1, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::minutes(minutes)).to_rfc3339()
    }

    fn base(family: ScenarioFamily, macro_fraction: f64, connected: bool, exchange_fraction: f64) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            scenario_id: "SYNTH-001".to_owned(),
            dt_seconds: SUPPORTED_DT_SECONDS,
            family,
            parameters: Parameters {
                matrix_capacity_mm: 100.0,
                macropore_capacity_mm: 20.0,
                macropore_inflow_fraction: macro_fraction,
                matrix_drainage_fraction_per_step: 0.05,
                macropore_drainage_fraction_per_step: 0.5,
                interdomain_exchange_fraction_per_step: exchange_fraction,
                macropore_path_connected: connected,
            },
            initial_state: InitialState {
                matrix_water_mm: 10.0,
                macropore_water_mm: 0.0,
                matrix_tracer_mg_m2: 0.0,
                macropore_tracer_mg_m2: 0.0,
            },
            forcing: (0..12).map(|index| ForcingStep {
                timestamp_utc: stamp(index * 10),
                rainfall_mm: if index == 0 { 8.0 } else if index == 1 { 4.0 } else { 0.0 },
                potential_evaporation_mm: if index > 2 { 0.1 } else { 0.0 },
                rainfall_tracer_concentration_mg_l: 1.0,
            }).collect(),
        }
    }

    #[test]
    fn matrix_only_has_no_true_macropore_drainage() {
        let input = base(ScenarioFamily::MatrixOnly, 0.0, false, 0.0);
        let report = simulate(&input, "a".repeat(64)).unwrap();
        assert_eq!(report.total_macropore_deep_drainage_mm, 0.0);
        assert_eq!(report.total_macropore_deep_tracer_mg_m2, 0.0);
        assert!(report.max_abs_water_balance_residual_mm < BALANCE_TOLERANCE);
        assert!(report.max_abs_tracer_balance_residual_mg_m2 < BALANCE_TOLERANCE);
    }

    #[test]
    fn connected_macropore_has_explicit_deep_boundary_flux() {
        let input = base(ScenarioFamily::ConnectedMacropore, 0.3, true, 0.0);
        let report = simulate(&input, "b".repeat(64)).unwrap();
        assert!(report.total_macropore_deep_drainage_mm > 0.0);
        assert!(report.total_macropore_deep_tracer_mg_m2 > 0.0);
    }

    #[test]
    fn disconnected_macropore_never_drains_directly_to_lower_boundary() {
        let input = base(ScenarioFamily::DisconnectedMacropore, 0.3, false, 0.0);
        let report = simulate(&input, "c".repeat(64)).unwrap();
        assert_eq!(report.total_macropore_deep_drainage_mm, 0.0);
        assert_eq!(report.total_macropore_deep_tracer_mg_m2, 0.0);
    }

    #[test]
    fn advective_exchange_conserves_combined_water_and_tracer() {
        let input = base(ScenarioFamily::DualDomainExchange, 0.3, true, 0.2);
        let report = simulate(&input, "d".repeat(64)).unwrap();
        assert!(report.steps.iter().any(|step| {
            step.matrix_to_macropore_exchange_mm > 0.0
                || step.macropore_to_matrix_exchange_mm > 0.0
        }));
        assert!(report.max_abs_water_balance_residual_mm < BALANCE_TOLERANCE);
        assert!(report.max_abs_tracer_balance_residual_mg_m2 < BALANCE_TOLERANCE);
    }

    #[test]
    fn repeated_storms_keep_stepwise_and_total_mass_balance() {
        let input = base(ScenarioFamily::DualDomainExchange, 0.3, true, 0.1);
        let report = simulate(&input, "e".repeat(64)).unwrap();
        assert!(report.steps.iter().all(|step| step.water_balance_residual_mm.abs() < BALANCE_TOLERANCE));
        assert!(report.steps.iter().all(|step| step.tracer_balance_residual_mg_m2.abs() < BALANCE_TOLERANCE));
    }

    #[test]
    fn rejects_noncontiguous_forcing() {
        let mut input = base(ScenarioFamily::ConnectedMacropore, 0.3, true, 0.0);
        input.forcing[2].timestamp_utc = stamp(25);
        assert!(validate(&input).unwrap_err().contains("contiguous at 10-minute cadence"));
    }

    #[test]
    fn rejects_matrix_only_with_macropore_inflow() {
        let input = base(ScenarioFamily::MatrixOnly, 0.1, false, 0.0);
        assert!(validate(&input).unwrap_err().contains("matrix_only requires"));
    }

    #[test]
    fn rejects_negative_rainfall() {
        let mut input = base(ScenarioFamily::ConnectedMacropore, 0.3, true, 0.0);
        input.forcing[0].rainfall_mm = -1.0;
        assert!(validate(&input).unwrap_err().contains("non-negative"));
    }

    #[test]
    fn preserves_scenario_family_as_separate_ground_truth() {
        let input = base(ScenarioFamily::DisconnectedMacropore, 0.3, false, 0.0);
        let report = simulate(&input, "f".repeat(64)).unwrap();
        assert_eq!(report.scenario_family, ScenarioFamily::DisconnectedMacropore);
        assert!(!report.true_macropore_path_connected);
        assert!(report.model_boundary.contains("not Richards"));
    }
}
