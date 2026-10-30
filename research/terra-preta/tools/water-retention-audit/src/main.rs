//! Audits a single soil-water desorption curve without fitting or extrapolating.
//!
//! Targets (field capacity and permanent wilting point) are explicit study inputs.
//! Interpolation is piecewise linear in log10 suction and is reported as such.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum MeasurementBasis {
    VolumetricFraction,
    GravimetricKgPerKgDrySoil,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct InputCurve {
    schema_version: String,
    sample_id: String,
    curve_id: String,
    measurement_basis: MeasurementBasis,
    bulk_density_kg_m3: Option<f64>,
    water_density_kg_m3: Option<f64>,
    monotonicity_tolerance_m3_m3: f64,
    method: MethodMetadata,
    target: TargetPotentials,
    layer_depth_mm: Option<f64>,
    points: Vec<RawPoint>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MethodMetadata {
    name: String,
    standard_or_protocol: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TargetPotentials {
    field_capacity_matric_potential_kpa: f64,
    permanent_wilting_point_matric_potential_kpa: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPoint {
    matric_potential_kpa: f64,
    water_content: f64,
}

#[derive(Debug, Clone, Serialize)]
struct NormalizedPoint {
    matric_potential_kpa: f64,
    suction_kpa: f64,
    water_content_m3_m3: f64,
}

#[derive(Debug, Serialize)]
struct ThresholdEstimate {
    target_matric_potential_kpa: f64,
    water_content_m3_m3: f64,
    estimate_method: &'static str,
    less_suction_bracket_matric_potential_kpa: Option<f64>,
    greater_suction_bracket_matric_potential_kpa: Option<f64>,
    interpolation_fraction: Option<f64>,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    sample_id: String,
    curve_id: String,
    input_label: String,
    input_sha256: String,
    measurement_basis: MeasurementBasis,
    bulk_density_kg_m3: Option<f64>,
    water_density_kg_m3: Option<f64>,
    method: MethodMetadataReport,
    measurement_count: usize,
    normalized_points: Vec<NormalizedPoint>,
    monotonicity_tolerance_m3_m3: f64,
    field_capacity: ThresholdEstimate,
    permanent_wilting_point: ThresholdEstimate,
    available_water_fraction_m3_m3: f64,
    layer_depth_mm: Option<f64>,
    available_water_storage_mm: Option<f64>,
    warnings: Vec<String>,
    uncertainty_status: &'static str,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct MethodMetadataReport {
    name: String,
    standard_or_protocol: Option<String>,
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

fn finite_positive(name: &str, value: f64) -> Result<(), Box<dyn Error>> {
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{name} must be finite and greater than zero").into());
    }
    Ok(())
}

fn audit_curve(
    curve: &InputCurve,
    input_label: &str,
    input_bytes: &[u8],
) -> Result<AuditReport, Box<dyn Error>> {
    if curve.schema_version != "0.1.0" {
        return Err("unsupported schema_version; expected 0.1.0".into());
    }
    if curve.sample_id.trim().is_empty() || curve.curve_id.trim().is_empty() {
        return Err("sample_id and curve_id must be non-empty".into());
    }
    if curve.sample_id.trim() != curve.sample_id || curve.curve_id.trim() != curve.curve_id {
        return Err("sample_id and curve_id must not contain leading/trailing whitespace".into());
    }
    if curve.method.name.trim().is_empty() {
        return Err("method.name must be non-empty".into());
    }
    if !curve.monotonicity_tolerance_m3_m3.is_finite()
        || curve.monotonicity_tolerance_m3_m3 < 0.0
        || curve.monotonicity_tolerance_m3_m3 > 1.0
    {
        return Err("monotonicity_tolerance_m3_m3 must be finite and within [0, 1]".into());
    }
    if let Some(depth) = curve.layer_depth_mm {
        finite_positive("layer_depth_mm", depth)?;
    }

    let fc = curve.target.field_capacity_matric_potential_kpa;
    let pwp = curve.target.permanent_wilting_point_matric_potential_kpa;
    if !fc.is_finite() || !pwp.is_finite() || !(pwp < fc && fc < 0.0) {
        return Err(
            "targets must be negative kPa values with permanent wilting point more negative than field capacity"
                .into(),
        );
    }

    let (bulk_density, water_density) = match curve.measurement_basis {
        MeasurementBasis::VolumetricFraction => (curve.bulk_density_kg_m3, curve.water_density_kg_m3),
        MeasurementBasis::GravimetricKgPerKgDrySoil => {
            let bulk = curve.bulk_density_kg_m3.ok_or(
                "gravimetric input requires bulk_density_kg_m3 (dry soil mass per bulk volume)",
            )?;
            let water = curve.water_density_kg_m3.ok_or(
                "gravimetric input requires water_density_kg_m3 for explicit mass-to-volume conversion",
            )?;
            finite_positive("bulk_density_kg_m3", bulk)?;
            finite_positive("water_density_kg_m3", water)?;
            (Some(bulk), Some(water))
        }
    };

    if curve.points.len() < 2 {
        return Err("at least two water-retention measurements are required".into());
    }

    let mut normalized = Vec::with_capacity(curve.points.len());
    for (index, point) in curve.points.iter().enumerate() {
        if !point.matric_potential_kpa.is_finite() || point.matric_potential_kpa > 0.0 {
            return Err(format!(
                "matric_potential_kpa must be finite and <= 0 at point {}",
                index + 1
            )
            .into());
        }
        if !point.water_content.is_finite() || point.water_content < 0.0 {
            return Err(format!("water_content must be finite and >= 0 at point {}", index + 1).into());
        }

        let theta = match curve.measurement_basis {
            MeasurementBasis::VolumetricFraction => {
                if point.water_content > 1.0 {
                    return Err(format!(
                        "volumetric water_content must be within [0, 1] at point {}",
                        index + 1
                    )
                    .into());
                }
                point.water_content
            }
            MeasurementBasis::GravimetricKgPerKgDrySoil => {
                point.water_content * bulk_density.unwrap() / water_density.unwrap()
            }
        };
        if !theta.is_finite() || theta > 1.0 {
            return Err(format!(
                "converted volumetric water content is outside [0, 1] at point {}",
                index + 1
            )
            .into());
        }
        normalized.push(NormalizedPoint {
            matric_potential_kpa: point.matric_potential_kpa,
            suction_kpa: -point.matric_potential_kpa,
            water_content_m3_m3: theta,
        });
    }

    normalized.sort_by(|a, b| a.suction_kpa.total_cmp(&b.suction_kpa));
    for pair in normalized.windows(2) {
        if pair[0].suction_kpa == pair[1].suction_kpa {
            return Err("duplicate matric-potential/suction measurements are not allowed".into());
        }
    }

    let mut warnings = Vec::new();
    for pair in normalized.windows(2) {
        let increase = pair[1].water_content_m3_m3 - pair[0].water_content_m3_m3;
        if increase > curve.monotonicity_tolerance_m3_m3 {
            return Err(format!(
                "desorption curve is non-monotonic by {increase:.6} m3/m3 between {} and {} kPa suction; raw measurements are not auto-corrected",
                pair[0].suction_kpa, pair[1].suction_kpa
            )
            .into());
        }
        if increase > 0.0 {
            warnings.push(format!(
                "small monotonicity violation of {increase:.6} m3/m3 between {} and {} kPa suction; retained without smoothing",
                pair[0].suction_kpa, pair[1].suction_kpa
            ));
        }
    }

    let field_capacity = estimate_at_potential(fc, &normalized)?;
    let permanent_wilting_point = estimate_at_potential(pwp, &normalized)?;
    let available_water =
        field_capacity.water_content_m3_m3 - permanent_wilting_point.water_content_m3_m3;
    if !available_water.is_finite() || available_water < 0.0 {
        return Err("computed available-water fraction is negative or non-finite".into());
    }
    let available_water_storage = curve.layer_depth_mm.map(|depth| available_water * depth);

    if warnings.is_empty() {
        warnings.push(
            "desorption-curve monotonicity and interpolation checks passed; measurement uncertainty is not propagated"
                .to_string(),
        );
    }

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status: if warnings.len() == 1
            && warnings[0].starts_with("desorption-curve monotonicity")
        {
            "passed"
        } else {
            "passed_with_warnings"
        },
        sample_id: curve.sample_id.clone(),
        curve_id: curve.curve_id.clone(),
        input_label: input_label.to_string(),
        input_sha256: sha256_hex(input_bytes),
        measurement_basis: curve.measurement_basis,
        bulk_density_kg_m3: bulk_density,
        water_density_kg_m3: water_density,
        method: MethodMetadataReport {
            name: curve.method.name.clone(),
            standard_or_protocol: curve.method.standard_or_protocol.clone(),
        },
        measurement_count: normalized.len(),
        normalized_points: normalized,
        monotonicity_tolerance_m3_m3: curve.monotonicity_tolerance_m3_m3,
        field_capacity,
        permanent_wilting_point,
        available_water_fraction_m3_m3: available_water,
        layer_depth_mm: curve.layer_depth_mm,
        available_water_storage_mm: available_water_storage,
        warnings,
        uncertainty_status: "not_propagated",
        interpretation_boundary: "single-sample measurement audit only; not a crop-response prediction, treatment-effect estimate, certification, safety determination, or authorization to apply a material",
    })
}

fn estimate_at_potential(
    target_matric_potential_kpa: f64,
    points: &[NormalizedPoint],
) -> Result<ThresholdEstimate, Box<dyn Error>> {
    let target_suction = -target_matric_potential_kpa;
    if let Some(point) = points
        .iter()
        .find(|point| point.suction_kpa == target_suction)
    {
        return Ok(ThresholdEstimate {
            target_matric_potential_kpa,
            water_content_m3_m3: point.water_content_m3_m3,
            estimate_method: "observed",
            less_suction_bracket_matric_potential_kpa: None,
            greater_suction_bracket_matric_potential_kpa: None,
            interpolation_fraction: None,
        });
    }

    for pair in points.windows(2) {
        if pair[0].suction_kpa < target_suction && target_suction < pair[1].suction_kpa {
            if pair[0].suction_kpa <= 0.0 {
                return Err(
                    "cannot interpolate logarithmically from zero suction; add a positive-suction lower bracket"
                        .into(),
                );
            }
            let low_log = pair[0].suction_kpa.log10();
            let high_log = pair[1].suction_kpa.log10();
            let target_log = target_suction.log10();
            let fraction = (target_log - low_log) / (high_log - low_log);
            let value = pair[0].water_content_m3_m3
                + fraction * (pair[1].water_content_m3_m3 - pair[0].water_content_m3_m3);
            return Ok(ThresholdEstimate {
                target_matric_potential_kpa,
                water_content_m3_m3: value,
                estimate_method: "log10_suction_linear_interpolation",
                less_suction_bracket_matric_potential_kpa: Some(pair[0].matric_potential_kpa),
                greater_suction_bracket_matric_potential_kpa: Some(pair[1].matric_potential_kpa),
                interpolation_fraction: Some(fraction),
            });
        }
    }

    Err(format!(
        "target {} kPa lies outside measured curve support; extrapolation is prohibited",
        target_matric_potential_kpa
    )
    .into())
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <curve-input.json> <audit-report.json>",
            args.first().map(String::as_str).unwrap_or("terra-preta-water-retention-audit")
        )
        .into());
    }
    let input_path = Path::new(&args[1]);
    let output_path = Path::new(&args[2]);
    if input_path == output_path {
        return Err("input and output paths must differ; preserve the original input".into());
    }
    let bytes = fs::read(input_path)?;
    let curve: InputCurve = serde_json::from_slice(&bytes)
        .map_err(|error| format!("invalid curve input JSON: {error}"))?;
    let label = input_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("curve-input.json");
    let report = audit_curve(&curve, label, &bytes)?;
    fs::write(output_path, serde_json::to_vec_pretty(&report)?)?;
    println!("water-retention audit written to {}", output_path.display());
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("water-retention audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(potential: f64, water: f64) -> RawPoint {
        RawPoint {
            matric_potential_kpa: potential,
            water_content: water,
        }
    }

    fn fixture() -> InputCurve {
        InputCurve {
            schema_version: "0.1.0".to_string(),
            sample_id: "sample-01".to_string(),
            curve_id: "curve-01".to_string(),
            measurement_basis: MeasurementBasis::VolumetricFraction,
            bulk_density_kg_m3: None,
            water_density_kg_m3: None,
            monotonicity_tolerance_m3_m3: 0.0,
            method: MethodMetadata {
                name: "pressure-plate-fixture".to_string(),
                standard_or_protocol: Some("synthetic-only".to_string()),
            },
            target: TargetPotentials {
                field_capacity_matric_potential_kpa: -33.0,
                permanent_wilting_point_matric_potential_kpa: -1500.0,
            },
            layer_depth_mm: Some(200.0),
            points: vec![
                point(-10.0, 0.30),
                point(-100.0, 0.22),
                point(-1500.0, 0.12),
            ],
        }
    }

    fn run_fixture(curve: &InputCurve) -> Result<AuditReport, Box<dyn Error>> {
        audit_curve(curve, "fixture.json", br#"{"fixture":true}"#)
    }

    #[test]
    fn interpolates_on_log10_suction_and_reports_brackets() {
        let report = run_fixture(&fixture()).unwrap();
        let expected_fraction = 33.0_f64.log10() - 1.0;
        let expected_fc = 0.30 + expected_fraction * (0.22 - 0.30);
        assert!((report.field_capacity.water_content_m3_m3 - expected_fc).abs() < 1e-12);
        assert_eq!(report.field_capacity.estimate_method, "log10_suction_linear_interpolation");
        assert_eq!(report.field_capacity.less_suction_bracket_matric_potential_kpa, Some(-10.0));
        assert_eq!(report.field_capacity.greater_suction_bracket_matric_potential_kpa, Some(-100.0));
        assert_eq!(report.permanent_wilting_point.estimate_method, "observed");
    }

    #[test]
    fn computes_available_water_depth_without_root_zone_assumptions() {
        let report = run_fixture(&fixture()).unwrap();
        let expected_awc = report.field_capacity.water_content_m3_m3 - 0.12;
        assert!((report.available_water_fraction_m3_m3 - expected_awc).abs() < 1e-12);
        assert!((report.available_water_storage_mm.unwrap() - expected_awc * 200.0).abs() < 1e-10);
    }

    #[test]
    fn leaves_layer_storage_absent_when_depth_is_unknown() {
        let mut curve = fixture();
        curve.layer_depth_mm = None;
        let report = run_fixture(&curve).unwrap();
        assert_eq!(report.available_water_storage_mm, None);
    }

    #[test]
    fn converts_gravimetric_water_content_using_explicit_densities() {
        let mut curve = fixture();
        curve.measurement_basis = MeasurementBasis::GravimetricKgPerKgDrySoil;
        curve.bulk_density_kg_m3 = Some(1300.0);
        curve.water_density_kg_m3 = Some(1000.0);
        curve.points = vec![
            point(-10.0, 0.30),
            point(-100.0, 0.22),
            point(-1500.0, 0.12),
        ];
        let report = run_fixture(&curve).unwrap();
        assert!((report.normalized_points[0].water_content_m3_m3 - 0.39).abs() < 1e-12);
        assert_eq!(report.bulk_density_kg_m3, Some(1300.0));
        assert_eq!(report.water_density_kg_m3, Some(1000.0));
    }

    #[test]
    fn rejects_gravimetric_input_without_bulk_density() {
        let mut curve = fixture();
        curve.measurement_basis = MeasurementBasis::GravimetricKgPerKgDrySoil;
        curve.water_density_kg_m3 = Some(1000.0);
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("bulk_density"));
    }

    #[test]
    fn rejects_gravimetric_input_without_water_density() {
        let mut curve = fixture();
        curve.measurement_basis = MeasurementBasis::GravimetricKgPerKgDrySoil;
        curve.bulk_density_kg_m3 = Some(1300.0);
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("water_density"));
    }

    #[test]
    fn rejects_duplicate_suction_measurements() {
        let mut curve = fixture();
        curve.points.push(point(-100.0, 0.20));
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("duplicate"));
    }

    #[test]
    fn rejects_non_monotonic_curve_beyond_declared_tolerance() {
        let mut curve = fixture();
        curve.points[1].water_content = 0.31;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("non-monotonic"));
    }

    #[test]
    fn preserves_small_monotonicity_violation_as_warning_without_smoothing() {
        let mut curve = fixture();
        curve.monotonicity_tolerance_m3_m3 = 0.002;
        curve.points[1].water_content = 0.301;
        let report = run_fixture(&curve).unwrap();
        assert_eq!(report.status, "passed_with_warnings");
        assert_eq!(report.normalized_points[1].water_content_m3_m3, 0.301);
        assert!(report.warnings.iter().any(|warning| warning.contains("retained without smoothing")));
    }

    #[test]
    fn rejects_extrapolation_beyond_observed_curve_support() {
        let mut curve = fixture();
        curve.target.field_capacity_matric_potential_kpa = -5.0;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("extrapolation is prohibited"));
    }

    #[test]
    fn rejects_invalid_target_order() {
        let mut curve = fixture();
        curve.target.permanent_wilting_point_matric_potential_kpa = -5.0;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("targets must be negative"));
    }

    #[test]
    fn rejects_positive_matric_potential() {
        let mut curve = fixture();
        curve.points[0].matric_potential_kpa = 5.0;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("matric_potential_kpa"));
    }

    #[test]
    fn rejects_volumetric_water_content_above_one() {
        let mut curve = fixture();
        curve.points[0].water_content = 1.01;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("within [0, 1]"));
    }

    #[test]
    fn rejects_zero_suction_log_interpolation_without_positive_lower_bracket() {
        let mut curve = fixture();
        curve.points = vec![point(0.0, 0.40), point(-100.0, 0.22), point(-1500.0, 0.12)];
        curve.target.field_capacity_matric_potential_kpa = -10.0;
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("zero suction"));
    }

    #[test]
    fn rejects_non_finite_or_negative_layer_depth() {
        let mut curve = fixture();
        curve.layer_depth_mm = Some(0.0);
        assert!(run_fixture(&curve).unwrap_err().to_string().contains("layer_depth_mm"));
    }

    #[test]
    fn keeps_input_digest_bound_to_original_bytes() {
        let curve = fixture();
        let bytes = br#"{"schema_version":"0.1.0","fixed":"input"}"#;
        let report = audit_curve(&curve, "fixture.json", bytes).unwrap();
        assert_eq!(report.input_sha256, sha256_hex(bytes));
    }
}
