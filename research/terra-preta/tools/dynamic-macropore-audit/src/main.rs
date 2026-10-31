use chrono::{DateTime, FixedOffset};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, fs::OpenOptions, io::{self, Write}, path::Path};

const MODEL_VERSION: &str = "0.1.0";
const NUMERIC_TOLERANCE: f64 = 1.0e-10;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    profile_id: String,
    horizon_label: String,
    image: ImageGeometry,
    soil: SoilProperties,
    fluid: FluidProperties,
    structural_retention: Option<StructuralRetention>,
    matrix_water_content_observations: Vec<MatrixWaterObservation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ImageGeometry {
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: f64,
    projected_macropore_area_mm2: f64,
    total_macropore_perimeter_mm: f64,
    mean_dry_macropore_width_mm: f64,
    minimum_object_area_threshold_mm2: f64,
    included_macropore_object_count: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SoilProperties {
    particle_density_kg_m3: f64,
    oven_dry_bulk_density_kg_m3: f64,
    field_capacity_bulk_density_kg_m3: f64,
    field_capacity_water_content_m3_m3: f64,
    wilting_point_water_content_m3_m3: f64,
    matrix_saturated_water_content_m3_m3: f64,
    matrix_saturated_hydraulic_conductivity_m_s: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct FluidProperties {
    surface_tension_n_m: f64,
    water_density_kg_m3: f64,
    dynamic_viscosity_pa_s: f64,
    gravity_m_s2: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StructuralRetention {
    alpha_dry_per_m: f64,
    beta0: f64,
    beta1: f64,
    n_dry: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MatrixWaterObservation {
    timestamp_utc: String,
    matrix_water_content_m3_m3: f64,
}

#[derive(Clone, Debug, Serialize)]
struct TimePoint {
    timestamp_utc: String,
    matrix_water_content_m3_m3: f64,
    bulk_density_kg_m3: f64,
    dynamic_cole: f64,
    structural_unit_width_mm: f64,
    macropore_width_mm: f64,
    macropore_volume_fraction: f64,
    macropore_saturated_conductivity_m_s: f64,
    whole_soil_transient_saturated_conductivity_m_s: f64,
    structural_alpha_per_m: Option<f64>,
    structural_n: Option<f64>,
}

#[derive(Debug, Serialize)]
struct Report {
    model_version: &'static str,
    input_sha256: String,
    study_id: String,
    profile_id: String,
    horizon_label: String,
    equations: Vec<&'static str>,
    image_geometry: ImageGeometryReport,
    soil_properties: SoilProperties,
    fluid_properties: FluidProperties,
    derived_geometry: DerivedGeometry,
    maximum_macropore_volume_fraction_at_wilting_point: f64,
    time_point_count: usize,
    time_points: Vec<TimePoint>,
    unsaturated_conductivity_status: &'static str,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

#[derive(Debug, Serialize)]
struct ImageGeometryReport {
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: f64,
    projected_macropore_area_mm2: f64,
    non_macropore_area_mm2: f64,
    total_macropore_perimeter_mm: f64,
    mean_dry_macropore_width_mm: f64,
    minimum_object_area_threshold_mm2: f64,
    included_macropore_object_count: usize,
}

#[derive(Debug, Serialize)]
struct DerivedGeometry {
    dry_structural_unit_width_mm: f64,
    dry_mean_macropore_width_mm: f64,
    total_dry_repeat_width_mm: f64,
    saturated_matrix_bulk_density_kg_m3: f64,
    saturated_dynamic_cole: f64,
    structural_width_correction_delta: f64,
}

fn finite_positive(value: f64, label: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{label} must be finite and greater than zero"));
    }
    Ok(())
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
        return Err(format!("{label}: timestamp must use UTC offset +00:00"));
    }
    Ok(parsed)
}

fn rho_saturated(soil: &SoilProperties) -> f64 {
    soil.particle_density_kg_m3 * (1.0 - soil.matrix_saturated_water_content_m3_m3)
}

fn bulk_density_at_matrix_water_content(
    theta_m: f64,
    soil: &SoilProperties,
) -> Result<f64, String> {
    let theta_wp = soil.wilting_point_water_content_m3_m3;
    let theta_fc = soil.field_capacity_water_content_m3_m3;
    let theta_sm = soil.matrix_saturated_water_content_m3_m3;
    let rho_od = soil.oven_dry_bulk_density_kg_m3;
    let rho_fc = soil.field_capacity_bulk_density_kg_m3;
    let rho_s = rho_saturated(soil);

    if !theta_m.is_finite() || theta_m < 0.0 || theta_m > theta_sm {
        return Err(format!(
            "matrix water content {theta_m} must be finite and within [0, theta_sm={theta_sm}]"
        ));
    }

    let rho = if theta_m >= theta_fc {
        if rho_fc <= rho_s {
            // Hirmas et al. Equation 9, first high-water branch.
            rho_fc
        } else {
            // Linear interpolation from field capacity to matrix saturation.
            rho_fc + (theta_m - theta_fc) * (rho_s - rho_fc) / (theta_sm - theta_fc)
        }
    } else if theta_m > theta_wp {
        // Linear interpolation from oven-dry state to field capacity.
        rho_od + (theta_m - theta_wp) * (rho_fc - rho_od) / (theta_fc - theta_wp)
    } else {
        rho_od
    };

    if !rho.is_finite() || rho <= 0.0 || rho > rho_od + NUMERIC_TOLERANCE {
        return Err("bulk-density function produced an invalid or nonphysical value".to_owned());
    }
    Ok(rho)
}

fn dynamic_cole(theta_m: f64, soil: &SoilProperties) -> Result<f64, String> {
    let rho = bulk_density_at_matrix_water_content(theta_m, soil)?;
    let cole = (soil.oven_dry_bulk_density_kg_m3 / rho).cbrt() - 1.0;
    if !cole.is_finite() || cole < -NUMERIC_TOLERANCE {
        return Err("dynamic COLE became non-finite or negative".to_owned());
    }
    Ok(cole.max(0.0))
}

fn validate(input: &Input) -> Result<Vec<DateTime<FixedOffset>>, String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.profile_id, "profile_id"),
        (&input.horizon_label, "horizon_label"),
    ] {
        if value.trim().is_empty() {
            return Err(format!("{label} must not be blank"));
        }
    }

    let image = &input.image;
    finite_positive(image.image_resolution_um_per_pixel, "image resolution")?;
    finite_positive(image.cross_section_area_mm2, "cross-section area")?;
    finite_nonnegative(image.projected_macropore_area_mm2, "projected macropore area")?;
    finite_positive(image.total_macropore_perimeter_mm, "total macropore perimeter")?;
    finite_positive(image.mean_dry_macropore_width_mm, "mean dry macropore width")?;
    finite_nonnegative(image.minimum_object_area_threshold_mm2, "minimum object area threshold")?;
    if image.included_macropore_object_count == 0 {
        return Err("included_macropore_object_count must be greater than zero".to_owned());
    }
    if image.projected_macropore_area_mm2 >= image.cross_section_area_mm2 {
        return Err("projected macropore area must be smaller than cross-section area".to_owned());
    }

    let soil = &input.soil;
    for (value, label) in [
        (soil.particle_density_kg_m3, "particle density"),
        (soil.oven_dry_bulk_density_kg_m3, "oven-dry bulk density"),
        (soil.field_capacity_bulk_density_kg_m3, "field-capacity bulk density"),
        (soil.matrix_saturated_hydraulic_conductivity_m_s, "matrix saturated hydraulic conductivity"),
    ] {
        finite_positive(value, label)?;
    }
    for (value, label) in [
        (soil.wilting_point_water_content_m3_m3, "wilting point"),
        (soil.field_capacity_water_content_m3_m3, "field capacity"),
        (soil.matrix_saturated_water_content_m3_m3, "matrix saturation"),
    ] {
        finite_positive(value, label)?;
    }
    if !(soil.wilting_point_water_content_m3_m3
        < soil.field_capacity_water_content_m3_m3
        && soil.field_capacity_water_content_m3_m3
            < soil.matrix_saturated_water_content_m3_m3
        && soil.matrix_saturated_water_content_m3_m3 < 1.0)
    {
        return Err("water contents must satisfy 0 < theta_wp < theta_fc < theta_sm < 1".to_owned());
    }

    let rho_s = rho_saturated(soil);
    finite_positive(rho_s, "matrix-saturated bulk density")?;
    if soil.oven_dry_bulk_density_kg_m3 < soil.field_capacity_bulk_density_kg_m3 {
        return Err("oven-dry bulk density must be greater than or equal to field-capacity bulk density".to_owned());
    }

    let fluid = &input.fluid;
    finite_positive(fluid.surface_tension_n_m, "water surface tension")?;
    finite_positive(fluid.water_density_kg_m3, "water density")?;
    finite_positive(fluid.dynamic_viscosity_pa_s, "dynamic viscosity")?;
    finite_positive(fluid.gravity_m_s2, "gravity")?;

    if let Some(parameters) = &input.structural_retention {
        finite_positive(parameters.alpha_dry_per_m, "dry structural alpha")?;
        finite_positive(parameters.n_dry, "dry structural n")?;
        finite_nonnegative(parameters.beta0, "beta0")?;
        finite_nonnegative(parameters.beta1, "beta1")?;
    }

    if input.matrix_water_content_observations.is_empty() {
        return Err("matrix_water_content_observations must not be empty".to_owned());
    }
    let mut times = Vec::with_capacity(input.matrix_water_content_observations.len());
    for (index, observation) in input.matrix_water_content_observations.iter().enumerate() {
        let timestamp = parse_utc(&observation.timestamp_utc, &format!("matrix_water_content_observations[{index}]"))?;
        bulk_density_at_matrix_water_content(
            observation.matrix_water_content_m3_m3,
            soil,
        )?;
        if let Some(previous) = times.last() {
            if timestamp <= *previous {
                return Err("matrix-water observations must be strictly ordered with no duplicate timestamps".to_owned());
            }
        }
        times.push(timestamp);
    }
    Ok(times)
}

fn macropore_fraction(
    width_mm: f64,
    structural_width_mm: f64,
    projected_pore_area_mm2: f64,
    cross_section_area_mm2: f64,
) -> Result<(f64, f64), String> {
    if width_mm <= NUMERIC_TOLERANCE {
        return Ok((0.0, 0.0));
    }
    let repeat_width = structural_width_mm + width_mm;
    let numerator = repeat_width.powi(3) - structural_width_mm.powi(3);
    let denominator = repeat_width * width_mm * (2.0 * structural_width_mm + width_mm);
    let rp = numerator / denominator;
    let fp = rp * projected_pore_area_mm2 / cross_section_area_mm2;
    if !rp.is_finite() || !fp.is_finite() || fp < -NUMERIC_TOLERANCE || fp > 1.0 + 1.0e-8 {
        return Err("macropore conversion produced a nonphysical projected-to-volume fraction".to_owned());
    }
    Ok((rp, fp.max(0.0)))
}

fn macropore_saturated_conductivity_m_s(
    width_mm: f64,
    structural_width_mm: f64,
    fluid: &FluidProperties,
) -> Result<f64, String> {
    if width_mm <= NUMERIC_TOLERANCE {
        return Ok(0.0);
    }
    let width = width_mm / 1000.0;
    let structural_width = structural_width_mm / 1000.0;
    let repeat_width = width + structural_width;
    let numerator = width.powi(3)
        * structural_width
        * fluid.water_density_kg_m3
        * fluid.gravity_m_s2;
    let denominator = 9.0 * fluid.dynamic_viscosity_pa_s * repeat_width.powi(2);
    let conductivity = numerator / denominator;
    if !conductivity.is_finite() || conductivity < 0.0 {
        return Err("macropore saturated conductivity became non-finite or negative".to_owned());
    }
    Ok(conductivity)
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    validate(input)?;
    let image = &input.image;
    let soil = &input.soil;
    let fluid = &input.fluid;

    // Hirmas et al. Equation 8: structural-unit dry width from MLT image geometry.
    let non_macropore_area_mm2 = image.cross_section_area_mm2
        - image.projected_macropore_area_mm2;
    let dry_structural_unit_width_mm =
        4.0 * non_macropore_area_mm2 / image.total_macropore_perimeter_mm;
    let dry_mean_macropore_width_mm = image.mean_dry_macropore_width_mm;
    let total_dry_repeat_width_mm =
        dry_structural_unit_width_mm + dry_mean_macropore_width_mm;
    if !dry_structural_unit_width_mm.is_finite()
        || dry_structural_unit_width_mm <= 0.0
        || !total_dry_repeat_width_mm.is_finite()
    {
        return Err("derived dry structural geometry is invalid".to_owned());
    }

    let saturated_matrix_bulk_density_kg_m3 = rho_saturated(soil);
    let saturated_dynamic_cole = dynamic_cole(
        soil.matrix_saturated_water_content_m3_m3,
        soil,
    )?;
    let saturated_unconstrained_width =
        dry_structural_unit_width_mm * (1.0 + saturated_dynamic_cole);
    let structural_width_correction_delta =
        if saturated_unconstrained_width > total_dry_repeat_width_mm {
            dry_mean_macropore_width_mm
                / (saturated_unconstrained_width - dry_structural_unit_width_mm)
        } else {
            1.0
        };
    if !structural_width_correction_delta.is_finite()
        || structural_width_correction_delta <= 0.0
    {
        return Err("structural width correction delta is invalid".to_owned());
    }

    // The dry reference at theta_wp is used to enforce the published maximum fp constraint.
    let dry_rho = bulk_density_at_matrix_water_content(
        soil.wilting_point_water_content_m3_m3,
        soil,
    )?;
    let dry_cole = (soil.oven_dry_bulk_density_kg_m3 / dry_rho).cbrt() - 1.0;
    let dry_unit_width =
        dry_structural_unit_width_mm * (1.0 + structural_width_correction_delta * dry_cole);
    let dry_pore_width = total_dry_repeat_width_mm - dry_unit_width;
    let (_, maximum_macropore_volume_fraction_at_wilting_point) = macropore_fraction(
        dry_pore_width,
        dry_unit_width,
        image.projected_macropore_area_mm2,
        image.cross_section_area_mm2,
    )?;

    let mut time_points = Vec::with_capacity(input.matrix_water_content_observations.len());
    for observation in &input.matrix_water_content_observations {
        let theta_m = observation.matrix_water_content_m3_m3;
        let rho = bulk_density_at_matrix_water_content(theta_m, soil)?;
        let cole = (soil.oven_dry_bulk_density_kg_m3 / rho).cbrt() - 1.0;
        let structural_width =
            dry_structural_unit_width_mm * (1.0 + structural_width_correction_delta * cole);
        let macropore_width = total_dry_repeat_width_mm - structural_width;
        if !macropore_width.is_finite()
            || macropore_width < -NUMERIC_TOLERANCE
            || macropore_width > dry_mean_macropore_width_mm + 1.0e-8
        {
            return Err(format!(
                "dynamic macropore width {macropore_width} mm is outside [0, dry mean width {} mm]",
                dry_mean_macropore_width_mm
            ));
        }
        let macropore_width = macropore_width.max(0.0);

        let (_rp, fp) = macropore_fraction(
            macropore_width,
            structural_width,
            image.projected_macropore_area_mm2,
            image.cross_section_area_mm2,
        )?;
        if fp > maximum_macropore_volume_fraction_at_wilting_point + 1.0e-8 {
            return Err("dynamic macropore volume fraction exceeds its wilting-point maximum".to_owned());
        }

        let ksp = macropore_saturated_conductivity_m_s(
            macropore_width,
            structural_width,
            fluid,
        )?;
        let ks = (1.0 - fp) * soil.matrix_saturated_hydraulic_conductivity_m_s
            + fp * ksp;
        if !ks.is_finite() || ks < 0.0 {
            return Err("whole-soil transient saturated conductivity became invalid".to_owned());
        }

        let (alpha2, n2) = if let Some(parameters) = &input.structural_retention {
            let width_ratio = macropore_width / dry_mean_macropore_width_mm;
            let alpha = parameters.alpha_dry_per_m
                * (parameters.beta0 + parameters.beta1 * width_ratio);
            (Some(alpha), Some(parameters.n_dry))
        } else {
            (None, None)
        };

        time_points.push(TimePoint {
            timestamp_utc: observation.timestamp_utc.clone(),
            matrix_water_content_m3_m3: theta_m,
            bulk_density_kg_m3: rho,
            dynamic_cole: cole.max(0.0),
            structural_unit_width_mm: structural_width,
            macropore_width_mm: macropore_width,
            macropore_volume_fraction: fp,
            macropore_saturated_conductivity_m_s: ksp,
            whole_soil_transient_saturated_conductivity_m_s: ks,
            structural_alpha_per_m: alpha2,
            structural_n: n2,
        });
    }

    Ok(Report {
        model_version: MODEL_VERSION,
        input_sha256,
        study_id: input.study_id.clone(),
        profile_id: input.profile_id.clone(),
        horizon_label: input.horizon_label.clone(),
        equations: vec![
            "Hirmas et al. (2025) Eq. 8: w_ds = 4(A_xs - A_p)/P",
            "Hirmas et al. (2025) Eqs. 9-11: rho(theta_m), rho_s, and dynamic COLE",
            "Hirmas et al. (2025) Eqs. 13-16: capped structural swelling and d(theta_m)",
            "Hirmas et al. (2025) Eqs. 20-23: K_sp, K_s, f_p and R_p",
            "Hirmas et al. (2025) Eq. 37: alpha_2/alpha_ds = beta_0 + beta_1(d/d_ds), when supplied",
        ],
        image_geometry: ImageGeometryReport {
            image_resolution_um_per_pixel: image.image_resolution_um_per_pixel,
            cross_section_area_mm2: image.cross_section_area_mm2,
            projected_macropore_area_mm2: image.projected_macropore_area_mm2,
            non_macropore_area_mm2,
            total_macropore_perimeter_mm: image.total_macropore_perimeter_mm,
            mean_dry_macropore_width_mm: image.mean_dry_macropore_width_mm,
            minimum_object_area_threshold_mm2: image.minimum_object_area_threshold_mm2,
            included_macropore_object_count: image.included_macropore_object_count,
        },
        soil_properties: (*soil).clone(),
        fluid_properties: (*fluid).clone(),
        derived_geometry: DerivedGeometry {
            dry_structural_unit_width_mm,
            dry_mean_macropore_width_mm,
            total_dry_repeat_width_mm,
            saturated_matrix_bulk_density_kg_m3,
            saturated_dynamic_cole,
            structural_width_correction_delta,
        },
        maximum_macropore_volume_fraction_at_wilting_point,
        time_point_count: time_points.len(),
        time_points,
        unsaturated_conductivity_status: "not_implemented_v0.1; no unsaturated conductivity claim",
        interpretation_boundary: "equation-level dynamic interpedal macropore geometry and saturated-conductivity model; not Richards flow, not a 3D pore network, not tracer-confirmed preferential flow, and not a South African soil calibration",
        warnings: vec![
            "Input matrix water contents must be independently measured or produced by a separately audited partition model; total theta must not be relabelled as matrix theta.".to_owned(),
            "MLT input is a 2D cross-section. This model uses the paper's equivalent-area structural-unit and planar slit assumptions; 3D connectivity is not established.".to_owned(),
            "The model conditions one dry-state image on matrix water content. Dynamic pore widths are model predictions, not repeated time-lapse pore imaging.".to_owned(),
            "The model is not the full unsaturated conductivity function in Hirmas et al. Eq. 38 and does not solve water transport.".to_owned(),
            "The matrix saturated conductivity is supplied as an input, not predicted from these image data.".to_owned(),
            "The original paper's fitted coefficients belong to one Konza APT soil profile; no direct transfer to Terra Preta, African or South African soils is claimed.".to_owned(),
            "Image object-area threshold and measured scale are recorded as supplied. Verify them against the exact source image/export.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: dynamic-macropore-audit <input.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let mut encoded = serde_json::to_vec_pretty(&report)?;
    encoded.push(b'\n');

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(Path::new(&args[2]))
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.sync_all()?;
    println!(
        "dynamic macropore audit complete: {} matrix-water states; dry w_ds {:.6} mm; max fp {:.6}; input SHA-256 {}",
        report.time_point_count,
        report.derived_geometry.dry_structural_unit_width_mm,
        report.maximum_macropore_volume_fraction_at_wilting_point,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("dynamic macropore audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic-equation-test".to_owned(),
            profile_id: "synthetic-profile-01".to_owned(),
            horizon_label: "Ap1".to_owned(),
            image: ImageGeometry {
                image_resolution_um_per_pixel: 180.0,
                cross_section_area_mm2: 100.0,
                projected_macropore_area_mm2: 10.0,
                total_macropore_perimeter_mm: 20.0,
                mean_dry_macropore_width_mm: 2.0,
                minimum_object_area_threshold_mm2: std::f64::consts::PI,
                included_macropore_object_count: 3,
            },
            soil: SoilProperties {
                particle_density_kg_m3: 2650.0,
                oven_dry_bulk_density_kg_m3: 1600.0,
                field_capacity_bulk_density_kg_m3: 1450.0,
                field_capacity_water_content_m3_m3: 0.30,
                wilting_point_water_content_m3_m3: 0.12,
                matrix_saturated_water_content_m3_m3: 0.43,
                matrix_saturated_hydraulic_conductivity_m_s: 1.0e-6,
            },
            fluid: FluidProperties {
                surface_tension_n_m: 0.0728,
                water_density_kg_m3: 1000.0,
                dynamic_viscosity_pa_s: 0.001,
                gravity_m_s2: 9.80665,
            },
            structural_retention: Some(StructuralRetention {
                alpha_dry_per_m: 1528.0,
                beta0: 0.0124,
                beta1: 0.990,
                n_dry: 2.78,
            }),
            matrix_water_content_observations: vec![
                MatrixWaterObservation {
                    timestamp_utc: "2026-01-01T00:00:00Z".to_owned(),
                    matrix_water_content_m3_m3: 0.12,
                },
                MatrixWaterObservation {
                    timestamp_utc: "2026-01-01T01:00:00Z".to_owned(),
                    matrix_water_content_m3_m3: 0.30,
                },
                MatrixWaterObservation {
                    timestamp_utc: "2026-01-01T02:00:00Z".to_owned(),
                    matrix_water_content_m3_m3: 0.40,
                },
            ],
        }
    }

    #[test]
    fn equation_8_derives_dry_structural_unit_width() {
        let report = evaluate(&base(), "a".repeat(64)).unwrap();
        assert_eq!(report.derived_geometry.dry_structural_unit_width_mm, 18.0);
    }

    #[test]
    fn wilting_point_retains_dry_pore_width() {
        let report = evaluate(&base(), "b".repeat(64)).unwrap();
        assert!((report.time_points[0].macropore_width_mm - 2.0).abs() < 1.0e-12);
        assert_eq!(report.time_points[0].dynamic_cole, 0.0);
    }

    #[test]
    fn wetter_matrix_conditions_contract_macropore_width() {
        let report = evaluate(&base(), "c".repeat(64)).unwrap();
        assert!(report.time_points[1].macropore_width_mm < report.time_points[0].macropore_width_mm);
        assert!(report.time_points[2].macropore_width_mm <= report.time_points[1].macropore_width_mm);
        assert!(report.time_points[2].macropore_volume_fraction < report.time_points[0].macropore_volume_fraction);
    }

    #[test]
    fn transient_saturated_conductivity_is_positive_and_finite() {
        let report = evaluate(&base(), "d".repeat(64)).unwrap();
        assert!(report.time_points.iter().all(|point| {
            point.macropore_saturated_conductivity_m_s.is_finite()
                && point.macropore_saturated_conductivity_m_s >= 0.0
                && point.whole_soil_transient_saturated_conductivity_m_s.is_finite()
                && point.whole_soil_transient_saturated_conductivity_m_s > 0.0
        }));
    }

    #[test]
    fn alpha_two_uses_positive_empirical_width_ratio_relationship() {
        let report = evaluate(&base(), "e".repeat(64)).unwrap();
        assert!(report.time_points[0].structural_alpha_per_m.unwrap()
            > report.time_points[2].structural_alpha_per_m.unwrap());
        assert!(report.time_points.iter().all(|point| {
            point.structural_n == Some(2.78)
        }));
    }

    #[test]
    fn rejects_out_of_range_matrix_water_content() {
        let mut input = base();
        input.matrix_water_content_observations[1].matrix_water_content_m3_m3 = 0.50;
        assert!(evaluate(&input, "f".repeat(64)).unwrap_err().contains("within [0, theta_sm"));
    }

    #[test]
    fn rejects_duplicate_or_out_of_order_timestamps() {
        let mut input = base();
        input.matrix_water_content_observations[2].timestamp_utc =
            input.matrix_water_content_observations[1].timestamp_utc.clone();
        assert!(validate(&input).unwrap_err().contains("strictly ordered"));
    }

    #[test]
    fn rejects_impossible_image_geometry() {
        let mut input = base();
        input.image.projected_macropore_area_mm2 = 101.0;
        assert!(validate(&input).unwrap_err().contains("smaller than cross-section area"));
    }

    #[test]
    fn output_declares_unsaturated_conductivity_not_implemented() {
        let report = evaluate(&base(), "1".repeat(64)).unwrap();
        assert_eq!(
            report.unsaturated_conductivity_status,
            "not_implemented_v0.1; no unsaturated conductivity claim"
        );
    }
}
