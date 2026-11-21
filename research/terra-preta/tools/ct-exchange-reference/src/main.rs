//! Synthetic CT-informed transient macropore–matrix exchange reference.
//!
//! a_wetted [m² interface / m³ bulk soil] * j_interface [m/s] = exchange rate density [1/s].
//! Positive transfer is macropore -> matrix; negative transfer reverses direction.
//! This is a manufactured reference kernel, not a reproduction of HYDRUS/DualPerm
//! or of the full model in Chaif et al. (2025).

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic CT-informed exchange-kernel reference only. Not a full physical model, field validation, parameter calibration, or agronomic advice.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    bulk_volume_m3: f64,
    maximum_gap_seconds: f64,
    interfacial_flux_density_source: String,
    matrix_water_initial_m3: f64,
    macropore_water_initial_m3: f64,
    matrix_water_capacity_m3: f64,
    macropore_water_capacity_m3: f64,
    points: Vec<Point>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Point {
    elapsed_seconds: f64,
    wetted_interface_area_m2_per_m3: f64,
    interfacial_exchange_flux_density_m_per_s: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    bulk_volume_m3: f64,
    point_count: usize,
    maximum_gap_seconds: f64,
    interfacial_flux_density_source: String,
    transient_interface_exchange: ModelResult,
    fixed_initial_interface_ablation: ModelResult,
    transient_minus_fixed_transfer_m3: f64,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct ModelResult {
    model_id: &'static str,
    exchange_operator: &'static str,
    signed_transfer_volume_m3: f64,
    macropore_to_matrix_volume_m3: f64,
    matrix_to_macropore_volume_m3: f64,
    matrix_water_final_m3: f64,
    macropore_water_final_m3: f64,
    water_balance_residual_m3: f64,
    maximum_absolute_rate_density_per_s: f64,
    intervals_integrated: usize,
}

#[derive(Debug, Serialize)]
struct Controls {
    pairwise_exchange_cancellation_pass: bool,
    water_balance_closure_pass: bool,
    deterministic_replay_pass: bool,
}

#[derive(Clone, Copy)]
struct WaterState {
    matrix_m3: f64,
    macropore_m3: f64,
}

#[derive(Clone, Copy)]
struct IntervalInputs {
    area0_m2_per_m3: f64,
    flux0_m_per_s: f64,
    area1_m2_per_m3: f64,
    flux1_m_per_s: f64,
    duration_seconds: f64,
}

#[derive(Clone, Copy)]
struct CompartmentCapacities {
    matrix_m3: f64,
    macropore_m3: f64,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn positive(value: f64, name: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{name} must be finite and positive"));
    }
    Ok(())
}

fn nonnegative(value: f64, name: &str) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{name} must be finite and non-negative"));
    }
    Ok(())
}

fn apply_transfer(
    state: &mut WaterState,
    signed_volume_m3: f64,
    matrix_capacity_m3: f64,
    macropore_capacity_m3: f64,
) -> Result<(), String> {
    if !signed_volume_m3.is_finite() {
        return Err("exchange transfer must be finite".to_string());
    }
    let tolerance = 1e-12;
    if signed_volume_m3 >= 0.0 {
        if signed_volume_m3 > state.macropore_m3 + tolerance {
            return Err("exchange would overdraw macropore water".to_string());
        }
        if state.matrix_m3 + signed_volume_m3 > matrix_capacity_m3 + tolerance {
            return Err("exchange would exceed matrix capacity".to_string());
        }
        state.macropore_m3 -= signed_volume_m3;
        state.matrix_m3 += signed_volume_m3;
    } else {
        let reverse = -signed_volume_m3;
        if reverse > state.matrix_m3 + tolerance {
            return Err("reverse exchange would overdraw matrix water".to_string());
        }
        if state.macropore_m3 + reverse > macropore_capacity_m3 + tolerance {
            return Err("reverse exchange would exceed macropore capacity".to_string());
        }
        state.matrix_m3 -= reverse;
        state.macropore_m3 += reverse;
    }
    if state.matrix_m3 < -tolerance
        || state.macropore_m3 < -tolerance
        || state.matrix_m3 > matrix_capacity_m3 + tolerance
        || state.macropore_m3 > macropore_capacity_m3 + tolerance
    {
        return Err("water state left declared physical bounds".to_string());
    }
    Ok(())
}

/// Integrate an interval with linearly interpolated endpoint exchange rates.
/// If the rate changes sign, split at the zero crossing and apply each
/// direction sequentially to prevent a later reversal hiding a transient
/// donor overdraw or receiver overfill.
fn integrate_interval(
    state: &mut WaterState,
    interval: IntervalInputs,
    bulk_volume_m3: f64,
    capacities: CompartmentCapacities,
) -> Result<(f64, f64, f64, f64), String> {
    nonnegative(interval.area0_m2_per_m3, "left wetted interface area")?;
    nonnegative(interval.area1_m2_per_m3, "right wetted interface area")?;
    if !interval.flux0_m_per_s.is_finite() || !interval.flux1_m_per_s.is_finite() {
        return Err("interfacial exchange flux density must be finite".to_string());
    }
    positive(interval.duration_seconds, "interval duration")?;
    positive(bulk_volume_m3, "bulk volume")?;

    let rate0 = interval.area0_m2_per_m3 * interval.flux0_m_per_s;
    let rate1 = interval.area1_m2_per_m3 * interval.flux1_m_per_s;
    if !rate0.is_finite() || !rate1.is_finite() {
        return Err("exchange rate density overflowed".to_string());
    }
    let mut macro_to_matrix = 0.0;
    let mut matrix_to_macro = 0.0;

    let mut apply_piece = |integrated_rate_seconds: f64| -> Result<(), String> {
        let transfer_m3 = integrated_rate_seconds * bulk_volume_m3;
        apply_transfer(
            state,
            transfer_m3,
            capacities.matrix_m3,
            capacities.macropore_m3,
        )?;
        if transfer_m3 >= 0.0 {
            macro_to_matrix += transfer_m3;
        } else {
            matrix_to_macro += -transfer_m3;
        }
        Ok(())
    };

    if rate0 * rate1 < 0.0 {
        let fraction = rate0.abs() / (rate0.abs() + rate1.abs());
        let first_seconds = interval.duration_seconds * fraction;
        let second_seconds = interval.duration_seconds - first_seconds;
        apply_piece(rate0 * 0.5 * first_seconds)?;
        apply_piece(rate1 * 0.5 * second_seconds)?;
    } else {
        apply_piece((rate0 + rate1) * 0.5 * interval.duration_seconds)?;
    }

    Ok((
        macro_to_matrix - matrix_to_macro,
        macro_to_matrix,
        matrix_to_macro,
        rate0.abs().max(rate1.abs()),
    ))
}

fn run_model(fixture: &Fixture, fixed_interface: bool) -> Result<ModelResult, String> {
    let mut state = WaterState {
        matrix_m3: fixture.matrix_water_initial_m3,
        macropore_m3: fixture.macropore_water_initial_m3,
    };
    let starting_total = state.matrix_m3 + state.macropore_m3;
    let fixed_area = fixture.points[0].wetted_interface_area_m2_per_m3;
    let mut signed_total = 0.0;
    let mut macro_to_matrix_total = 0.0;
    let mut matrix_to_macro_total = 0.0;
    let mut maximum_rate: f64 = 0.0;
    let mut intervals = 0;

    for pair in fixture.points.windows(2) {
        let left = &pair[0];
        let right = &pair[1];
        let duration = right.elapsed_seconds - left.elapsed_seconds;
        if duration <= 0.0 || !duration.is_finite() {
            return Err("time points must be strictly increasing".to_string());
        }
        if duration > fixture.maximum_gap_seconds {
            return Err(format!(
                "time gap {duration}s exceeds maximum {}s",
                fixture.maximum_gap_seconds
            ));
        }
        let (area0, area1) = if fixed_interface {
            (fixed_area, fixed_area)
        } else {
            (
                left.wetted_interface_area_m2_per_m3,
                right.wetted_interface_area_m2_per_m3,
            )
        };
        let (signed, forward, reverse, max_rate) = integrate_interval(
            &mut state,
            IntervalInputs {
                area0_m2_per_m3: area0,
                flux0_m_per_s: left.interfacial_exchange_flux_density_m_per_s,
                area1_m2_per_m3: area1,
                flux1_m_per_s: right.interfacial_exchange_flux_density_m_per_s,
                duration_seconds: duration,
            },
            fixture.bulk_volume_m3,
            CompartmentCapacities {
                matrix_m3: fixture.matrix_water_capacity_m3,
                macropore_m3: fixture.macropore_water_capacity_m3,
            },
        )?;
        signed_total += signed;
        macro_to_matrix_total += forward;
        matrix_to_macro_total += reverse;
        maximum_rate = maximum_rate.max(max_rate);
        intervals += 1;
    }

    let final_total = state.matrix_m3 + state.macropore_m3;
    let residual = final_total - starting_total;
    if residual.abs() > 1e-12 * starting_total.abs().max(1.0) {
        return Err(format!(
            "water-balance residual {residual} m3 exceeds tolerance"
        ));
    }
    Ok(ModelResult {
        model_id: if fixed_interface {
            "fixed_initial_interface_ablation"
        } else {
            "transient_ct_interface_reference"
        },
        exchange_operator: if fixed_interface {
            "a_wetted(t)=a_wetted(t0); r_exchange=a_wetted(t0)*j_macropore(t)"
        } else {
            "r_exchange=a_wetted(t)*j_macropore(t)"
        },
        signed_transfer_volume_m3: signed_total,
        macropore_to_matrix_volume_m3: macro_to_matrix_total,
        matrix_to_macropore_volume_m3: matrix_to_macro_total,
        matrix_water_final_m3: state.matrix_m3,
        macropore_water_final_m3: state.macropore_m3,
        water_balance_residual_m3: residual,
        maximum_absolute_rate_density_per_s: maximum_rate,
        intervals_integrated: intervals,
    })
}

fn validate_fixture(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != "ct-matrix-macropore-exchange-v0.1" {
        return Err("unsupported schema_version".to_string());
    }
    if fixture.fixture_status != FIXTURE_STATUS {
        return Err("v0.1 accepts explicitly synthetic-only inputs".to_string());
    }
    if fixture.scenario_id.trim().is_empty() {
        return Err("scenario_id must be non-empty".to_string());
    }
    if fixture.interfacial_flux_density_source != "synthetic_manufactured" {
        return Err("v0.1 fixture must identify its interfacial flux density as synthetic_manufactured".to_string());
    }
    positive(fixture.bulk_volume_m3, "bulk_volume_m3")?;
    positive(fixture.maximum_gap_seconds, "maximum_gap_seconds")?;
    nonnegative(fixture.matrix_water_initial_m3, "matrix_water_initial_m3")?;
    nonnegative(
        fixture.macropore_water_initial_m3,
        "macropore_water_initial_m3",
    )?;
    positive(fixture.matrix_water_capacity_m3, "matrix_water_capacity_m3")?;
    positive(
        fixture.macropore_water_capacity_m3,
        "macropore_water_capacity_m3",
    )?;
    if fixture.points.len() < 2 {
        return Err("at least two time points are required".to_string());
    }
    if fixture.matrix_water_initial_m3 > fixture.matrix_water_capacity_m3
        || fixture.macropore_water_initial_m3 > fixture.macropore_water_capacity_m3
    {
        return Err("initial water exceeds compartment capacity".to_string());
    }
    for (index, point) in fixture.points.iter().enumerate() {
        if !point.elapsed_seconds.is_finite() {
            return Err(format!("point {index} has a non-finite time"));
        }
        nonnegative(
            point.wetted_interface_area_m2_per_m3,
            &format!("point {index} wetted interface area"),
        )?;
        if !point.interfacial_exchange_flux_density_m_per_s.is_finite() {
            return Err(format!("point {index} interfacial exchange flux density is non-finite"));
        }
        if index > 0 && point.elapsed_seconds <= fixture.points[index - 1].elapsed_seconds {
            return Err("time points must be strictly increasing".to_string());
        }
    }
    Ok(())
}

fn audit_bytes(bytes: &[u8]) -> Result<Report, String> {
    let fixture: Fixture =
        serde_json::from_slice(bytes).map_err(|error| format!("invalid fixture JSON: {error}"))?;
    validate_fixture(&fixture)?;
    let transient = run_model(&fixture, false)?;
    let fixed = run_model(&fixture, true)?;
    let pairwise_exchange_cancellation_pass = (transient.macropore_to_matrix_volume_m3
        - transient.matrix_to_macropore_volume_m3
        - transient.signed_transfer_volume_m3)
        .abs()
        < 1e-12;
    let water_balance_closure_pass = transient.water_balance_residual_m3.abs() < 1e-12;
    let replay = run_model(&fixture, false)?;
    let deterministic_replay_pass = serde_json::to_vec(&transient)
        .map_err(|error| error.to_string())?
        == serde_json::to_vec(&replay).map_err(|error| error.to_string())?;
    if !pairwise_exchange_cancellation_pass
        || !water_balance_closure_pass
        || !deterministic_replay_pass
    {
        return Err("synthetic numerical reference control failed".to_string());
    }
    Ok(Report {
        audit_version: AUDIT_VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: fixture.scenario_id,
        input_sha256: sha256_hex(bytes),
        bulk_volume_m3: fixture.bulk_volume_m3,
        point_count: fixture.points.len(),
        maximum_gap_seconds: fixture.maximum_gap_seconds,
        interfacial_flux_density_source: fixture.interfacial_flux_density_source,
        transient_minus_fixed_transfer_m3: transient.signed_transfer_volume_m3
            - fixed.signed_transfer_volume_m3,
        transient_interface_exchange: transient,
        fixed_initial_interface_ablation: fixed,
        controls: Controls {
            pairwise_exchange_cancellation_pass,
            water_balance_closure_pass,
            deterministic_replay_pass,
        },
        interpretation_boundary: BOUNDARY,
    })
}

fn write_new_file(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(bytes)?;
    file.write_all(b"\n")?;
    file.sync_all()
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <synthetic-exchange-fixture.json> <new-report.json>",
            args.first()
                .map(String::as_str)
                .unwrap_or("ct-exchange-reference")
        )
        .into());
    }
    let input = fs::read(&args[1])?;
    let output_path = Path::new(&args[2]);
    if output_path.exists() {
        return Err(format!("refusing to overwrite report: {}", output_path.display()).into());
    }
    let report = audit_bytes(&input).map_err(io::Error::other)?;
    let output = serde_json::to_vec_pretty(&report)?;
    write_new_file(output_path, &output)?;
    println!(
        "audited synthetic scenario {}; transient transfer={:.9} m3; reference-only",
        report.scenario_id,
        report
            .transient_interface_exchange
            .signed_transfer_volume_m3
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("CT exchange reference audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn fixture() -> Value {
        serde_json::from_str(include_str!("../fixtures/synthetic-ct-exchange.json"))
            .expect("synthetic fixture must parse")
    }

    fn audit(value: &Value) -> Result<Report, String> {
        audit_bytes(value.to_string().as_bytes())
    }

    #[test]
    fn manufactured_transient_exchange_matches_analytic_integral() {
        let report = audit(&fixture()).unwrap();
        // [4, 8, 2]e-6 s-1 over two 1800s intervals:
        // 6e-6*1800 + 5e-6*1800 = 0.0198 m3 for 1m3 bulk volume.
        assert!(
            (report
                .transient_interface_exchange
                .signed_transfer_volume_m3
                - 0.0198)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn transient_interface_differs_from_fixed_initial_interface_ablation() {
        let report = audit(&fixture()).unwrap();
        assert!(
            (report
                .transient_interface_exchange
                .signed_transfer_volume_m3
                - report
                    .fixed_initial_interface_ablation
                    .signed_transfer_volume_m3)
                .abs()
                > 1e-6
        );
    }

    #[test]
    fn zero_wetted_interface_produces_zero_exchange() {
        let mut value = fixture();
        for point in value["points"].as_array_mut().unwrap() {
            point["wetted_interface_area_m2_per_m3"] = Value::from(0.0);
        }
        let report = audit(&value).unwrap();
        assert_eq!(
            report
                .transient_interface_exchange
                .signed_transfer_volume_m3,
            0.0
        );
    }

    #[test]
    fn reverse_flux_transfers_water_from_matrix_to_macropore() {
        let mut value = fixture();
        for point in value["points"].as_array_mut().unwrap() {
            point["interfacial_exchange_flux_density_m_per_s"] = Value::from(-1e-5);
        }
        let report = audit(&value).unwrap();
        assert!(
            report
                .transient_interface_exchange
                .matrix_to_macropore_volume_m3
                > 0.0
        );
        assert_eq!(
            report
                .transient_interface_exchange
                .macropore_to_matrix_volume_m3,
            0.0
        );
    }

    #[test]
    fn excessive_time_gap_is_rejected() {
        let mut value = fixture();
        value["points"][1]["elapsed_seconds"] = Value::from(9000.0);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn nonmonotonic_time_is_rejected() {
        let mut value = fixture();
        value["points"][2]["elapsed_seconds"] = Value::from(1000.0);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn matrix_overfill_is_rejected_without_clipping() {
        let mut value = fixture();
        value["matrix_water_initial_m3"] = Value::from(0.29);
        value["matrix_water_capacity_m3"] = Value::from(0.30);
        value["macropore_water_initial_m3"] = Value::from(0.30);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn sign_reversal_cannot_hide_transient_donor_overdraw() {
        let mut value = fixture();
        value["points"][0]["wetted_interface_area_m2_per_m3"] = Value::from(0.8);
        value["points"][1]["wetted_interface_area_m2_per_m3"] = Value::from(0.8);
        value["points"][0]["interfacial_exchange_flux_density_m_per_s"] =
            Value::from(1e-3);
        value["points"][1]["interfacial_exchange_flux_density_m_per_s"] =
            Value::from(-1e-3);
        value["macropore_water_initial_m3"] = Value::from(0.3);
        // The signed net trapezoid is zero, but the positive half-interval
        // would transfer 0.72 m3 and overdraw the 0.3 m3 donor pool.
        assert!(audit(&value).is_err());
    }

    #[test]
    fn macropore_donor_overdraw_is_rejected_without_clipping() {
        let mut value = fixture();
        value["macropore_water_initial_m3"] = Value::from(0.001);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn initial_state_beyond_capacity_is_rejected() {
        let mut value = fixture();
        value["matrix_water_initial_m3"] = Value::from(0.40);
        value["matrix_water_capacity_m3"] = Value::from(0.35);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn repeated_replay_is_identical() {
        let value = fixture();
        let first = audit(&value).unwrap();
        let second = audit(&value).unwrap();
        assert_eq!(
            first.transient_interface_exchange.signed_transfer_volume_m3,
            second
                .transient_interface_exchange
                .signed_transfer_volume_m3
        );
        assert_eq!(first.input_sha256, second.input_sha256);
    }

    #[test]
    fn signed_transfer_components_cancel_against_state_change() {
        let report = audit(&fixture()).unwrap();
        assert!(report.controls.pairwise_exchange_cancellation_pass);
        assert!(report.controls.water_balance_closure_pass);
    }
}
