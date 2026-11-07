use chrono::{DateTime, FixedOffset};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeSet,
    env,
    error::Error,
    fs::{self, OpenOptions},
    io::{self, Write},
    path::Path,
};

const AUDIT_VERSION: &str = "0.1.0";
const GEOMETRY_TOLERANCE: f64 = 1.0e-6;
const ZERO_WIDTH_TOLERANCE_MM: f64 = 1.0e-12;
const MIN_RETENTION_FIT_POINTS: usize = 5;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    schema_version: String,
    study_id: String,
    profile_id: String,
    horizon_label: String,
    scan_id: String,
    image_source_sha256: String,
    image_evidence_ref: String,
    morphology_evidence_ref: String,
    dynamic_model_evidence_ref: String,
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: f64,
    minimum_object_area_mm2: f64,
    columns: ColumnMapping,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ColumnMapping {
    area: String,
    perimeter: String,
    min_feret: String,
}

#[derive(Clone, Debug)]
struct Pore {
    object_id: String,
    area_mm2: f64,
    perimeter_mm: f64,
    dry_min_feret_mm: f64,
}

#[derive(Debug, Deserialize)]
struct DynamicReport {
    model_version: String,
    input_sha256: String,
    study_id: String,
    profile_id: String,
    horizon_label: String,
    image_geometry: DynamicImageGeometry,
    fluid_properties: FluidProperties,
    derived_geometry: DynamicDerivedGeometry,
    maximum_macropore_volume_fraction_at_wilting_point: f64,
    time_points: Vec<DynamicTimePoint>,
}

#[derive(Debug, Deserialize)]
struct DynamicImageGeometry {
    image_resolution_um_per_pixel: f64,
    cross_section_area_mm2: f64,
    projected_macropore_area_mm2: f64,
    total_macropore_perimeter_mm: f64,
    mean_dry_macropore_width_mm: f64,
    minimum_object_area_threshold_mm2: f64,
    included_macropore_object_count: usize,
}

#[derive(Debug, Deserialize)]
struct FluidProperties {
    surface_tension_n_m: f64,
    water_density_kg_m3: f64,
    gravity_m_s2: f64,
}

#[derive(Debug, Deserialize)]
struct DynamicDerivedGeometry {
    dry_structural_unit_width_mm: f64,
    dry_mean_macropore_width_mm: f64,
    total_dry_repeat_width_mm: f64,
}

#[derive(Debug, Deserialize)]
struct DynamicTimePoint {
    timestamp_utc: String,
    matrix_water_content_m3_m3: f64,
    structural_unit_width_mm: f64,
    macropore_width_mm: f64,
    macropore_volume_fraction: f64,
    structural_alpha_per_m: Option<f64>,
    structural_alpha_figure_table_plus_per_m: Option<f64>,
    structural_alpha_printed_eq37_minus_per_m: Option<f64>,
    structural_n: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct PoreRetentionPoint {
    object_id: String,
    area_mm2: f64,
    dry_min_feret_mm: f64,
    dynamic_min_feret_mm: f64,
    matric_head_m: Option<f64>,
    individual_conversion_factor_rp: Option<f64>,
    individual_volume_fraction: f64,
    cumulative_macropore_water_content_m3_m3: f64,
}

#[derive(Clone, Debug, Serialize)]
struct TimeStateAudit {
    timestamp_utc: String,
    matrix_water_content_m3_m3: f64,
    average_dynamic_macropore_width_mm: f64,
    structural_unit_width_mm: f64,
    dynamic_average_ratio_d_over_dry: f64,
    per_pore_volume_fraction_sum: f64,
    aggregate_eq22_volume_fraction: f64,
    per_pore_minus_aggregate_difference: f64,
    per_pore_retention_curve: Vec<PoreRetentionPoint>,
    retention_curve_fit: RetentionCurveFit,
    maximum_macropore_volume_fraction_at_wilting_point: f64,
}

#[derive(Clone, Debug, Serialize)]
struct RetentionCurveFit {
    status: &'static str,
    input_point_count: usize,
    alpha2_fit_per_m: Option<f64>,
    n2_fit: Option<f64>,
    rmse_m3_m3: Option<f64>,
    maximum_absolute_error_m3_m3: Option<f64>,
    near_optimal_alpha_min_per_m: Option<f64>,
    near_optimal_alpha_max_per_m: Option<f64>,
    near_optimal_n_min: Option<f64>,
    near_optimal_n_max: Option<f64>,
    near_optimality_rule: &'static str,
    figure_table_alpha_rmse_m3_m3: Option<f64>,
    printed_eq37_minus_alpha_rmse_m3_m3: Option<f64>,
    upstream_figure_table_alpha_per_m: Option<f64>,
    upstream_printed_eq37_minus_alpha_per_m: Option<f64>,
    upstream_n2: Option<f64>,
}

#[derive(Serialize)]
struct GeometryAudit {
    input_object_count: usize,
    retained_macropore_object_count: usize,
    excluded_object_count: usize,
    area_filter_operator: &'static str,
    minimum_object_area_mm2: f64,
    cross_section_area_mm2: f64,
    summed_projected_macropore_area_mm2: f64,
    summed_macropore_perimeter_mm: f64,
    geometric_mean_dry_min_feret_mm: f64,
    dry_structural_unit_width_from_image_mm: f64,
    dynamic_report_dry_structural_unit_width_mm: f64,
    structural_width_absolute_difference_mm: f64,
    dynamic_report_mean_dry_macropore_width_mm: f64,
    dry_mean_width_absolute_difference_mm: f64,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    config_sha256: String,
    morphology_csv_sha256: String,
    dynamic_report_sha256: String,
    dynamic_model_input_sha256: String,
    study_id: String,
    profile_id: String,
    horizon_label: String,
    scan_id: String,
    image_source_sha256: String,
    image_resolution_um_per_pixel: f64,
    equations: Vec<&'static str>,
    geometry_audit: GeometryAudit,
    maximum_macropore_volume_fraction_at_wilting_point: f64,
    time_state_count: usize,
    time_states: Vec<TimeStateAudit>,
    discrepancy_interpretation: &'static str,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
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

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() <= GEOMETRY_TOLERANCE * a.abs().max(b.abs()).max(1.0)
}

fn valid_sha256(value: &str, label: &str) -> Result<(), String> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!(
            "{label} must contain exactly 64 hexadecimal characters"
        ));
    }
    Ok(())
}

fn geometric_mean(values: &[f64]) -> Result<f64, String> {
    if values.is_empty()
        || values
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err("geometric mean requires at least one finite positive value".to_owned());
    }
    let mean_log = values.iter().map(|value| value.ln()).sum::<f64>() / values.len() as f64;
    let result = mean_log.exp();
    finite_positive(result, "geometric mean")?;
    Ok(result)
}

fn parse_config(bytes: &[u8]) -> Result<Config, String> {
    let config: Config = serde_json::from_slice(bytes)
        .map_err(|error| format!("config JSON parse failed: {error}"))?;
    if config.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported config schema_version {:?}",
            config.schema_version
        ));
    }
    for (value, label) in [
        (&config.study_id, "study_id"),
        (&config.profile_id, "profile_id"),
        (&config.horizon_label, "horizon_label"),
        (&config.scan_id, "scan_id"),
        (&config.image_evidence_ref, "image_evidence_ref"),
        (&config.morphology_evidence_ref, "morphology_evidence_ref"),
        (
            &config.dynamic_model_evidence_ref,
            "dynamic_model_evidence_ref",
        ),
        (&config.columns.area, "columns.area"),
        (&config.columns.perimeter, "columns.perimeter"),
        (&config.columns.min_feret, "columns.min_feret"),
    ] {
        if value.trim().is_empty() || value.trim() != value {
            return Err(format!(
                "{label} must be nonblank and have no surrounding whitespace"
            ));
        }
    }
    valid_sha256(&config.image_source_sha256, "image_source_sha256")?;
    finite_positive(
        config.image_resolution_um_per_pixel,
        "image_resolution_um_per_pixel",
    )?;
    finite_positive(config.cross_section_area_mm2, "cross_section_area_mm2")?;
    finite_nonnegative(config.minimum_object_area_mm2, "minimum_object_area_mm2")?;
    if config.columns.area == config.columns.perimeter
        || config.columns.area == config.columns.min_feret
        || config.columns.perimeter == config.columns.min_feret
    {
        return Err("area, perimeter and min_feret must map to distinct CSV columns".to_owned());
    }
    Ok(config)
}

fn csv_column(headers: &csv::StringRecord, name: &str) -> Result<usize, String> {
    headers
        .iter()
        .position(|header| header == name)
        .ok_or_else(|| format!("required ImageJ CSV column {name:?} was not found"))
}

fn csv_number(
    record: &csv::StringRecord,
    index: usize,
    name: &str,
    row: usize,
) -> Result<f64, String> {
    let raw = record
        .get(index)
        .ok_or_else(|| format!("row {row}: missing value for {name}"))?;
    let value: f64 = raw
        .trim()
        .parse()
        .map_err(|error| format!("row {row}: cannot parse {name} value {raw:?}: {error}"))?;
    finite_positive(value, &format!("row {row}: {name}"))?;
    Ok(value)
}

fn parse_pores(bytes: &[u8], config: &Config) -> Result<(usize, Vec<Pore>), String> {
    let mut reader = csv::ReaderBuilder::new()
        .flexible(false)
        .trim(csv::Trim::Headers)
        .from_reader(bytes);
    let headers = reader
        .headers()
        .map_err(|error| format!("ImageJ CSV header parse failed: {error}"))?
        .clone();
    let area_index = csv_column(&headers, &config.columns.area)?;
    let perimeter_index = csv_column(&headers, &config.columns.perimeter)?;
    let min_feret_index = csv_column(&headers, &config.columns.min_feret)?;
    let mut retained = Vec::new();
    let mut input_count = 0usize;

    for (offset, result) in reader.records().enumerate() {
        let line = offset + 2;
        let record = result.map_err(|error| format!("ImageJ CSV row {line} failed: {error}"))?;
        if record.iter().all(|value| value.trim().is_empty()) {
            continue;
        }
        input_count += 1;
        let area = csv_number(&record, area_index, &config.columns.area, line)?;
        let perimeter = csv_number(&record, perimeter_index, &config.columns.perimeter, line)?;
        let min_feret = csv_number(&record, min_feret_index, &config.columns.min_feret, line)?;
        // Hirmas et al. retain areas strictly greater than pi mm2.
        if area > config.minimum_object_area_mm2 {
            retained.push(Pore {
                object_id: format!("imagej-row-{line}"),
                area_mm2: area,
                perimeter_mm: perimeter,
                dry_min_feret_mm: min_feret,
            });
        }
    }
    if input_count == 0 {
        return Err("ImageJ CSV contains no pore rows".to_owned());
    }
    if retained.is_empty() {
        return Err("no pore objects remain after the strict minimum-area filter".to_owned());
    }
    Ok((input_count, retained))
}

fn parse_utc(value: &str, label: &str) -> Result<DateTime<FixedOffset>, String> {
    let timestamp = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if timestamp.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp must use a zero UTC offset"));
    }
    Ok(timestamp)
}

fn validate_dynamic(config: &Config, report: &DynamicReport) -> Result<(), String> {
    if report.study_id != config.study_id
        || report.profile_id != config.profile_id
        || report.horizon_label != config.horizon_label
    {
        return Err(
            "dynamic report study/profile/horizon identity does not match config".to_owned(),
        );
    }
    if report.model_version != "0.1.0" {
        return Err(format!(
            "unsupported dynamic model version {:?}",
            report.model_version
        ));
    }
    valid_sha256(&report.input_sha256, "dynamic report input_sha256")?;
    if !near(
        report.image_geometry.image_resolution_um_per_pixel,
        config.image_resolution_um_per_pixel,
    ) || !near(
        report.image_geometry.cross_section_area_mm2,
        config.cross_section_area_mm2,
    ) || !near(
        report.image_geometry.minimum_object_area_threshold_mm2,
        config.minimum_object_area_mm2,
    ) {
        return Err("dynamic report image resolution, cross-section area or object filter does not match config".to_owned());
    }
    if report.time_points.is_empty() {
        return Err("dynamic report contains no time points".to_owned());
    }
    if !report
        .image_geometry
        .projected_macropore_area_mm2
        .is_finite()
        || report.image_geometry.projected_macropore_area_mm2 <= 0.0
        || !report
            .image_geometry
            .total_macropore_perimeter_mm
            .is_finite()
        || report.image_geometry.total_macropore_perimeter_mm <= 0.0
        || !report
            .derived_geometry
            .dry_structural_unit_width_mm
            .is_finite()
        || report.derived_geometry.dry_structural_unit_width_mm <= 0.0
        || !report
            .derived_geometry
            .dry_mean_macropore_width_mm
            .is_finite()
        || report.derived_geometry.dry_mean_macropore_width_mm <= 0.0
        || !report
            .derived_geometry
            .total_dry_repeat_width_mm
            .is_finite()
        || report.derived_geometry.total_dry_repeat_width_mm <= 0.0
    {
        return Err("dynamic report contains non-positive or non-finite pore geometry".to_owned());
    }
    if report.image_geometry.included_macropore_object_count == 0 {
        return Err("dynamic report included_macropore_object_count must be positive".to_owned());
    }
    finite_positive(
        report.fluid_properties.surface_tension_n_m,
        "dynamic report surface tension",
    )?;
    finite_positive(
        report.fluid_properties.water_density_kg_m3,
        "dynamic report water density",
    )?;
    finite_positive(
        report.fluid_properties.gravity_m_s2,
        "dynamic report gravity",
    )?;
    finite_nonnegative(
        report.maximum_macropore_volume_fraction_at_wilting_point,
        "maximum macropore volume fraction",
    )?;

    let mut previous_time: Option<DateTime<FixedOffset>> = None;
    for (index, point) in report.time_points.iter().enumerate() {
        let timestamp = parse_utc(
            &point.timestamp_utc,
            &format!("time_points[{index}].timestamp_utc"),
        )?;
        if let Some(previous) = &previous_time {
            if timestamp <= previous.clone() {
                return Err("dynamic report time points must be strictly chronological with no duplicate timestamps".to_owned());
            }
        }
        previous_time = Some(timestamp);
        finite_nonnegative(
            point.matrix_water_content_m3_m3,
            &format!("time_points[{index}].matrix_water_content"),
        )?;
        finite_positive(
            point.structural_unit_width_mm,
            &format!("time_points[{index}].structural_unit_width_mm"),
        )?;
        finite_nonnegative(
            point.macropore_width_mm,
            &format!("time_points[{index}].macropore_width_mm"),
        )?;
        finite_nonnegative(
            point.macropore_volume_fraction,
            &format!("time_points[{index}].macropore_volume_fraction"),
        )?;
        if !near(
            point.structural_unit_width_mm + point.macropore_width_mm,
            report.derived_geometry.total_dry_repeat_width_mm,
        ) {
            return Err(format!(
                "time_points[{index}] violates the declared structural repeat width"
            ));
        }
        if point.macropore_volume_fraction
            > report.maximum_macropore_volume_fraction_at_wilting_point + 1.0e-8
        {
            return Err(format!(
                "time_points[{index}] aggregate macro fraction exceeds the wilting-point maximum"
            ));
        }
    }
    Ok(())
}

fn conversion_factor_rp(w_mm: f64, pore_width_mm: f64) -> Result<f64, String> {
    finite_positive(w_mm, "dynamic structural-unit width")?;
    finite_positive(pore_width_mm, "dynamic individual pore width")?;
    let repeat_width = w_mm + pore_width_mm;
    let numerator = repeat_width.powi(3) - w_mm.powi(3);
    let denominator = repeat_width * pore_width_mm * (2.0 * w_mm + pore_width_mm);
    let result = numerator / denominator;
    if !result.is_finite() || result <= 0.0 {
        return Err(
            "Equation 35 produced a non-positive or non-finite pore conversion factor".to_owned(),
        );
    }
    Ok(result)
}

fn van_genuchten_theta_p(
    maximum_fraction: f64,
    matric_head_m: f64,
    alpha_per_m: f64,
    n: f64,
) -> Option<f64> {
    if !maximum_fraction.is_finite()
        || maximum_fraction <= 0.0
        || !matric_head_m.is_finite()
        || matric_head_m >= 0.0
        || !alpha_per_m.is_finite()
        || alpha_per_m <= 0.0
        || !n.is_finite()
        || n <= 1.0
    {
        return None;
    }
    let log_term = n * (alpha_per_m.ln() + (-matric_head_m).ln());
    if !log_term.is_finite() {
        return None;
    }
    let powered_term = if log_term > 700.0 {
        f64::INFINITY
    } else if log_term < -745.0 {
        0.0
    } else {
        log_term.exp()
    };
    let value = if powered_term.is_infinite() {
        0.0
    } else {
        maximum_fraction / (1.0 + powered_term).powf(1.0 - 1.0 / n)
    };
    (value.is_finite() && value >= 0.0).then_some(value)
}

fn retention_curve_rmse(
    samples: &[(f64, f64)],
    maximum_fraction: f64,
    alpha_per_m: f64,
    n: f64,
) -> Option<f64> {
    if samples.is_empty() {
        return None;
    }
    let mut squared_error = 0.0;
    for (head, observed) in samples {
        let predicted = van_genuchten_theta_p(maximum_fraction, *head, alpha_per_m, n)?;
        squared_error += (predicted - observed).powi(2);
    }
    let rmse = (squared_error / samples.len() as f64).sqrt();
    rmse.is_finite().then_some(rmse)
}

fn fit_van_genuchten_retention_curve(
    points: &[PoreRetentionPoint],
    maximum_fraction: f64,
    figure_table_alpha: Option<f64>,
    printed_eq37_minus_alpha: Option<f64>,
    upstream_n: Option<f64>,
) -> RetentionCurveFit {
    let samples: Vec<(f64, f64)> = points
        .iter()
        .filter_map(|point| {
            point
                .matric_head_m
                .map(|head| (head, point.cumulative_macropore_water_content_m3_m3))
        })
        .filter(|(head, theta)| {
            head.is_finite() && *head < 0.0 && theta.is_finite() && *theta >= 0.0
        })
        .collect();

    let branch_rmse = |alpha: Option<f64>| match (alpha, upstream_n) {
        (Some(alpha), Some(n)) => retention_curve_rmse(&samples, maximum_fraction, alpha, n),
        _ => None,
    };

    let figure_table_rmse = branch_rmse(figure_table_alpha);
    let printed_minus_rmse = branch_rmse(printed_eq37_minus_alpha);

    let empty = |status: &'static str| RetentionCurveFit {
        status,
        input_point_count: samples.len(),
        alpha2_fit_per_m: None,
        n2_fit: None,
        rmse_m3_m3: None,
        maximum_absolute_error_m3_m3: None,
        near_optimal_alpha_min_per_m: None,
        near_optimal_alpha_max_per_m: None,
        near_optimal_n_min: None,
        near_optimal_n_max: None,
        near_optimality_rule: "grid points within max(5% of minimum SSE, 1e-12 absolute SSE); sensitivity region only, not a confidence interval",
        figure_table_alpha_rmse_m3_m3: figure_table_rmse,
        printed_eq37_minus_alpha_rmse_m3_m3: printed_minus_rmse,
        upstream_figure_table_alpha_per_m: figure_table_alpha,
        upstream_printed_eq37_minus_alpha_per_m: printed_eq37_minus_alpha,
        upstream_n2,
    };

    if !maximum_fraction.is_finite() || maximum_fraction <= 0.0 {
        return empty("not_fitted_no_positive_macropore_fraction");
    }
    if samples.len() < MIN_RETENTION_FIT_POINTS {
        return empty("not_fitted_insufficient_retention_points");
    }
    let unique_heads: BTreeSet<i64> = samples
        .iter()
        .map(|(head, _)| (head.abs().log10() * 1.0e9).round() as i64)
        .collect();
    if unique_heads.len() < MIN_RETENTION_FIT_POINTS {
        return empty("not_fitted_insufficient_distinct_matric_heads");
    }

    let original_log_alpha_min = -2.0_f64;
    let original_log_alpha_max = 8.0_f64;
    let original_n_min = 1.01_f64;
    let original_n_max = 10.0_f64;
    let mut log_alpha_min = original_log_alpha_min;
    let mut log_alpha_max = original_log_alpha_max;
    let mut n_min = original_n_min;
    let mut n_max = original_n_max;
    let mut best: Option<(f64, f64, f64)> = None;
    let mut final_candidates: Vec<(f64, f64, f64)> = Vec::new();

    for stage in 0..6 {
        let divisions = 48usize;
        let log_alpha_step = (log_alpha_max - log_alpha_min) / divisions as f64;
        let n_step = (n_max - n_min) / divisions as f64;
        let mut stage_candidates = Vec::with_capacity((divisions + 1) * (divisions + 1));

        for alpha_index in 0..=divisions {
            let log_alpha = log_alpha_min + alpha_index as f64 * log_alpha_step;
            let alpha = 10.0_f64.powf(log_alpha);
            for n_index in 0..=divisions {
                let n = n_min + n_index as f64 * n_step;
                if let Some(rmse) = retention_curve_rmse(&samples, maximum_fraction, alpha, n) {
                    let sse = rmse.powi(2);
                    stage_candidates.push((sse, log_alpha, n));
                    if best.map(|current| sse < current.0).unwrap_or(true) {
                        best = Some((sse, log_alpha, n));
                    }
                }
            }
        }

        if stage == 5 {
            final_candidates = stage_candidates;
            break;
        }
        let Some((_, best_log_alpha, best_n)) = best else {
            return empty("not_fitted_nonfinite_objective");
        };
        let alpha_span = log_alpha_step * 2.0;
        let n_span = n_step * 2.0;
        log_alpha_min = (best_log_alpha - alpha_span).max(original_log_alpha_min);
        log_alpha_max = (best_log_alpha + alpha_span).min(original_log_alpha_max);
        n_min = (best_n - n_span).max(original_n_min);
        n_max = (best_n + n_span).min(original_n_max);
    }

    let Some((best_sse, best_log_alpha, best_n)) = best else {
        return empty("not_fitted_no_valid_parameter_pair");
    };
    let threshold = best_sse + (best_sse * 0.05).max(1.0e-12);
    let near_optimal: Vec<_> = final_candidates
        .iter()
        .filter(|(sse, _, _)| *sse <= threshold)
        .collect();
    let alpha_min = near_optimal
        .iter()
        .map(|(_, log_alpha, _)| 10.0_f64.powf(*log_alpha))
        .fold(f64::INFINITY, f64::min);
    let alpha_max = near_optimal
        .iter()
        .map(|(_, log_alpha, _)| 10.0_f64.powf(*log_alpha))
        .fold(0.0_f64, f64::max);
    let n_min_range = near_optimal
        .iter()
        .map(|(_, _, n)| *n)
        .fold(f64::INFINITY, f64::min);
    let n_max_range = near_optimal
        .iter()
        .map(|(_, _, n)| *n)
        .fold(0.0_f64, f64::max);

    let alpha = 10.0_f64.powf(best_log_alpha);
    let fitted_rmse = best_sse.sqrt();
    let maximum_absolute_error = samples
        .iter()
        .filter_map(|(head, observed)| {
            van_genuchten_theta_p(maximum_fraction, *head, alpha, best_n)
                .map(|predicted| (predicted - observed).abs())
        })
        .fold(0.0_f64, f64::max);
    let boundary_limited = (best_log_alpha - original_log_alpha_min).abs() < 1.0e-8
        || (best_log_alpha - original_log_alpha_max).abs() < 1.0e-8
        || (best_n - original_n_min).abs() < 1.0e-8
        || (best_n - original_n_max).abs() < 1.0e-8;

    RetentionCurveFit {
        status: if boundary_limited {
            "fit_boundary_limited_exploratory"
        } else {
            "fitted_descriptive_no_uncertainty_interval"
        },
        input_point_count: samples.len(),
        alpha2_fit_per_m: Some(alpha),
        n2_fit: Some(best_n),
        rmse_m3_m3: Some(fitted_rmse),
        maximum_absolute_error_m3_m3: Some(maximum_absolute_error),
        near_optimal_alpha_min_per_m: Some(alpha_min),
        near_optimal_alpha_max_per_m: Some(alpha_max),
        near_optimal_n_min: Some(n_min_range),
        near_optimal_n_max: Some(n_max_range),
        near_optimality_rule: "grid points within max(5% of minimum SSE, 1e-12 absolute SSE); sensitivity region only, not a confidence interval",
        figure_table_alpha_rmse_m3_m3: figure_table_rmse,
        printed_eq37_minus_alpha_rmse_m3_m3: printed_minus_rmse,
        upstream_figure_table_alpha_per_m: figure_table_alpha,
        upstream_printed_eq37_minus_alpha_per_m: printed_eq37_minus_alpha,
        upstream_n2,
    }
}

fn per_pore_state(
    pores: &[Pore],
    config: &Config,
    report: &DynamicReport,
    point: &DynamicTimePoint,
) -> Result<TimeStateAudit, String> {
    let dry_mean_width = report.derived_geometry.dry_mean_macropore_width_mm;
    let average_dynamic_width = point.macropore_width_mm;
    let ratio = average_dynamic_width / dry_mean_width;
    if !ratio.is_finite() || !(0.0..=1.0 + 1.0e-8).contains(&ratio) {
        return Err(format!(
            "time {} average pore-width ratio must be within [0, 1]; received {ratio}",
            point.timestamp_utc
        ));
    }
    if !near(
        point.structural_unit_width_mm + average_dynamic_width,
        report.derived_geometry.total_dry_repeat_width_mm,
    ) {
        return Err(format!(
            "time {} violates dynamic repeat-width geometry",
            point.timestamp_utc
        ));
    }

    let mut states = Vec::with_capacity(pores.len());
    for pore in pores {
        let dynamic_width = pore.dry_min_feret_mm * ratio;
        if dynamic_width <= ZERO_WIDTH_TOLERANCE_MM {
            states.push((
                dynamic_width,
                pore.object_id.clone(),
                pore.area_mm2,
                pore.dry_min_feret_mm,
                None,
                None,
                0.0,
            ));
            continue;
        }
        let rp = conversion_factor_rp(point.structural_unit_width_mm, dynamic_width)?;
        // Hirmas et al. Eq. 36: f_pi = R_pi * A_pi*d(theta_m)/(A_xs*d_ds).
        let fp = rp * pore.area_mm2 * average_dynamic_width
            / (config.cross_section_area_mm2 * dry_mean_width);
        finite_nonnegative(fp, "individual pore volume fraction")?;
        let head = -report.fluid_properties.surface_tension_n_m
            / (report.fluid_properties.water_density_kg_m3
                * report.fluid_properties.gravity_m_s2
                * (dynamic_width / 1000.0));
        if !head.is_finite() || head >= 0.0 {
            return Err("individual pore matric head must be finite and negative".to_owned());
        }
        states.push((
            dynamic_width,
            pore.object_id.clone(),
            pore.area_mm2,
            pore.dry_min_feret_mm,
            Some(head),
            Some(rp),
            fp,
        ));
    }

    states.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    let mut cumulative = 0.0;
    let mut per_pore_retention_curve = Vec::with_capacity(states.len());
    for (width, object_id, area, dry_width, head, rp, fp) in states {
        cumulative += fp;
        if !cumulative.is_finite() || cumulative > 1.0 + 1.0e-8 {
            return Err("cumulative per-pore macropore water fraction exceeds one".to_owned());
        }
        per_pore_retention_curve.push(PoreRetentionPoint {
            object_id,
            area_mm2: area,
            dry_min_feret_mm: dry_width,
            dynamic_min_feret_mm: width,
            matric_head_m: head,
            individual_conversion_factor_rp: rp,
            individual_volume_fraction: fp,
            cumulative_macropore_water_content_m3_m3: cumulative,
        });
    }

    let per_pore_volume_fraction_sum = cumulative;
    if per_pore_volume_fraction_sum
        > report.maximum_macropore_volume_fraction_at_wilting_point + 1.0e-8
    {
        return Err(format!(
            "per-pore sum exceeds dynamic-model wilting-point maximum at {}",
            point.timestamp_utc
        ));
    }

    let retention_curve_fit = fit_van_genuchten_retention_curve(
        &per_pore_retention_curve,
        per_pore_volume_fraction_sum,
        point
            .structural_alpha_figure_table_plus_per_m
            .or(point.structural_alpha_per_m),
        point.structural_alpha_printed_eq37_minus_per_m,
        point.structural_n,
    );

    Ok(TimeStateAudit {
        timestamp_utc: point.timestamp_utc.clone(),
        matrix_water_content_m3_m3: point.matrix_water_content_m3_m3,
        average_dynamic_macropore_width_mm: average_dynamic_width,
        structural_unit_width_mm: point.structural_unit_width_mm,
        dynamic_average_ratio_d_over_dry: ratio,
        per_pore_volume_fraction_sum,
        aggregate_eq22_volume_fraction: point.macropore_volume_fraction,
        per_pore_minus_aggregate_difference: per_pore_volume_fraction_sum
            - point.macropore_volume_fraction,
        per_pore_retention_curve,
        retention_curve_fit,
        maximum_macropore_volume_fraction_at_wilting_point: report
            .maximum_macropore_volume_fraction_at_wilting_point,
    })
}

fn evaluate(
    config: &Config,
    dynamic: &DynamicReport,
    input_count: usize,
    pores: &[Pore],
    config_sha: String,
    morphology_sha: String,
    dynamic_sha: String,
) -> Result<Report, String> {
    validate_dynamic(config, dynamic)?;
    if pores.len() != dynamic.image_geometry.included_macropore_object_count {
        return Err("retained per-pore object count differs from dynamic report".to_owned());
    }
    let total_area: f64 = pores.iter().map(|pore| pore.area_mm2).sum();
    let total_perimeter: f64 = pores.iter().map(|pore| pore.perimeter_mm).sum();
    let dry_widths: Vec<f64> = pores.iter().map(|pore| pore.dry_min_feret_mm).collect();
    let d_ds = geometric_mean(&dry_widths)?;
    let w_ds = 4.0 * (config.cross_section_area_mm2 - total_area) / total_perimeter;
    finite_positive(
        w_ds,
        "dry structural-unit width derived from per-pore table",
    )?;

    if !near(
        total_area,
        dynamic.image_geometry.projected_macropore_area_mm2,
    ) || !near(
        total_perimeter,
        dynamic.image_geometry.total_macropore_perimeter_mm,
    ) {
        return Err("summed per-pore projected area/perimeter differs from dynamic report; check filter, columns and ROI".to_owned());
    }
    if !near(d_ds, dynamic.image_geometry.mean_dry_macropore_width_mm)
        || !near(d_ds, dynamic.derived_geometry.dry_mean_macropore_width_mm)
    {
        return Err(
            "geometric mean of retained MinFeret values differs from dynamic dry mean pore width"
                .to_owned(),
        );
    }
    if !near(w_ds, dynamic.derived_geometry.dry_structural_unit_width_mm) {
        return Err(
            "per-pore dry structural-unit width does not match dynamic report geometry".to_owned(),
        );
    }

    let mut states = Vec::with_capacity(dynamic.time_points.len());
    for point in &dynamic.time_points {
        states.push(per_pore_state(pores, config, dynamic, point)?);
    }

    let difference = states
        .iter()
        .map(|state| state.per_pore_minus_aggregate_difference.abs())
        .fold(0.0_f64, f64::max);
    let interpretation = if difference <= 0.01 {
        "per-pore sums are close to aggregate Eq. 22 within an absolute 0.01 fraction for this fixture/input"
    } else {
        "per-pore and aggregate Eq. 22 fractions differ by more than 0.01; retain both and investigate segmentation/shape assumptions"
    };

    Ok(Report {
        audit_version: AUDIT_VERSION,
        config_sha256: config_sha,
        morphology_csv_sha256: morphology_sha,
        dynamic_report_sha256: dynamic_sha,
        dynamic_model_input_sha256: dynamic.input_sha256.clone(),
        study_id: config.study_id.clone(),
        profile_id: config.profile_id.clone(),
        horizon_label: config.horizon_label.clone(),
        scan_id: config.scan_id.clone(),
        image_source_sha256: config.image_source_sha256.clone(),
        image_resolution_um_per_pixel: config.image_resolution_um_per_pixel,
        equations: vec![
            "Hirmas et al. (2025) Eq. 35: R_pi(theta_m) = ((w(theta_m)+d_i(theta_m))^3-w(theta_m)^3)/((w+d_i)d_i(2w+d_i))",
            "Hirmas et al. (2025) Eq. 36: f_pi(theta_m) = R_pi(theta_m)*A_pi*d(theta_m)/(A_xs*d_ds)",
            "Hirmas et al. (2025) Eq. 33: h_i = -sigma/(rho_w*g*d_i); head in metres with d_i converted from mm",
            "Hirmas et al. (2025) Eq. 46: cumulative macropore water content by ascending pore width",
        ],
        geometry_audit: GeometryAudit {
            input_object_count: input_count,
            retained_macropore_object_count: pores.len(),
            excluded_object_count: input_count - pores.len(),
            area_filter_operator: "strictly_greater_than",
            minimum_object_area_mm2: config.minimum_object_area_mm2,
            cross_section_area_mm2: config.cross_section_area_mm2,
            summed_projected_macropore_area_mm2: total_area,
            summed_macropore_perimeter_mm: total_perimeter,
            geometric_mean_dry_min_feret_mm: d_ds,
            dry_structural_unit_width_from_image_mm: w_ds,
            dynamic_report_dry_structural_unit_width_mm: dynamic.derived_geometry.dry_structural_unit_width_mm,
            structural_width_absolute_difference_mm: (w_ds - dynamic.derived_geometry.dry_structural_unit_width_mm).abs(),
            dynamic_report_mean_dry_macropore_width_mm: dynamic.derived_geometry.dry_mean_macropore_width_mm,
            dry_mean_width_absolute_difference_mm: (d_ds - dynamic.derived_geometry.dry_mean_macropore_width_mm).abs(),
        },
        maximum_macropore_volume_fraction_at_wilting_point:
            dynamic.maximum_macropore_volume_fraction_at_wilting_point,
        time_state_count: states.len(),
        time_states: states,
        discrepancy_interpretation: interpretation,
        warnings: vec![
            "This stage consumes object-level 2D MLT geometry and an upstream dynamic-macropore report; it does not segment raw images or infer 3D connectivity.".to_owned(),
            "The dynamic MinFeret width for each pore is scaled proportionally by the dynamic mean-width ratio d(theta_m)/d_ds, consistent with the paper's stated proportional width adjustment; verify numerical parity against the authors' R scripts before claiming exact reproduction.".to_owned(),
            "Per-pore Eq. 36 sums and aggregate Eq. 22 values are reported side by side; no forced normalization or silent reconciliation is performed.".to_owned(),
            "Matric head is derived from planar-pore capillarity and assumes the paper's slit-pore idealization; it is not direct field pressure-head measurement.".to_owned(),
            "The selected-object threshold is strict (> minimum area) because the Dryad protocol says objects with areas larger than pi mm2 were retained.".to_owned(),
            "Image calibration, segmentation, object linking, plane orientation and ROI selection dominate the validity of morphology inputs and must be reviewed independently.".to_owned(),
        ],
        interpretation_boundary: "per-pore 2D geometry-derived macropore water-retention estimates; not 3D connected flow, measured flux, tracer-confirmed preferential flow or field efficacy",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 5 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: hirmas-pore-resolved-audit <imagej-objects.csv> <dynamic-report.json> <config.json> <new-report.json>",
        ).into());
    }
    let csv_path = Path::new(&args[1]);
    let dynamic_path = Path::new(&args[2]);
    let config_path = Path::new(&args[3]);
    let output_path = Path::new(&args[4]);
    if output_path.exists() {
        for input_path in [csv_path, dynamic_path, config_path] {
            if fs::canonicalize(output_path)? == fs::canonicalize(input_path)? {
                return Err(format!(
                    "output path resolves to input {}; refusing overwrite",
                    input_path.display()
                )
                .into());
            }
        }
        return Err(format!("output path already exists: {}", output_path.display()).into());
    }

    let csv_bytes = fs::read(csv_path)?;
    let dynamic_bytes = fs::read(dynamic_path)?;
    let config_bytes = fs::read(config_path)?;
    let csv_sha = format!("{:x}", Sha256::digest(&csv_bytes));
    let dynamic_sha = format!("{:x}", Sha256::digest(&dynamic_bytes));
    let config_sha = format!("{:x}", Sha256::digest(&config_bytes));
    let config = parse_config(&config_bytes)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let dynamic: DynamicReport = serde_json::from_slice(&dynamic_bytes)?;
    let (input_count, pores) = parse_pores(&csv_bytes, &config)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let report = evaluate(
        &config,
        &dynamic,
        input_count,
        &pores,
        config_sha,
        csv_sha,
        dynamic_sha,
    )
    .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;

    let mut encoded = serde_json::to_vec_pretty(&report)?;
    encoded.push(b'\n');
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("refusing overwrite or cannot create report: {error}"),
            )
        })?;
    output.write_all(&encoded)?;
    output.sync_all()?;
    println!(
        "Hirmas per-pore audit complete: {} retained pores; {} dynamic states; max |per-pore - aggregate| {}; input CSV SHA-256 {}",
        report.geometry_audit.retained_macropore_object_count,
        report.time_state_count,
        report
            .time_states
            .iter()
            .map(|state| state.per_pore_minus_aggregate_difference.abs())
            .fold(0.0_f64, f64::max),
        report.morphology_csv_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("Hirmas per-pore audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pore(id: &str, area: f64, perimeter: f64, min_feret: f64) -> Pore {
        Pore {
            object_id: id.to_owned(),
            area_mm2: area,
            perimeter_mm: perimeter,
            dry_min_feret_mm: min_feret,
        }
    }

    #[test]
    fn parse_utc_requires_explicit_zero_offset() {
        assert!(
            parse_utc("2026-01-01T00:00:00+01:00", "timestamp")
                .unwrap_err()
                .contains("zero UTC offset")
        );
        assert!(parse_utc("2026-01-01T00:00:00Z", "timestamp").is_ok());
    }

    #[test]
    fn parse_utc_rejects_invalid_timestamp() {
        assert!(
            parse_utc("not-a-timestamp", "timestamp")
                .unwrap_err()
                .contains("invalid RFC3339")
        );
    }

    fn retention_point(head: f64, theta_p: f64) -> PoreRetentionPoint {
        PoreRetentionPoint {
            object_id: format!("synthetic-{}", head.abs()),
            area_mm2: 1.0,
            dry_min_feret_mm: 1.0,
            dynamic_min_feret_mm: 1.0,
            matric_head_m: Some(head),
            individual_conversion_factor_rp: Some(1.0),
            individual_volume_fraction: theta_p,
            cumulative_macropore_water_content_m3_m3: theta_p,
        }
    }

    #[test]
    fn fits_a_synthetic_van_genuchten_retention_curve() {
        let maximum_fraction = 0.20;
        let alpha = 1_500.0;
        let n = 2.8;
        let points: Vec<_> = (0..21)
            .map(|index| {
                let log_head = -4.0 + (index as f64 * 3.0 / 20.0);
                let head = -10.0_f64.powf(log_head);
                let theta = van_genuchten_theta_p(maximum_fraction, head, alpha, n).unwrap();
                retention_point(head, theta)
            })
            .collect();
        let fit = fit_van_genuchten_retention_curve(
            &points,
            maximum_fraction,
            Some(alpha),
            None,
            Some(n),
        );
        assert_eq!(fit.status, "fitted_descriptive_no_uncertainty_interval");
        let fitted_alpha = fit.alpha2_fit_per_m.unwrap();
        let fitted_n = fit.n2_fit.unwrap();
        assert!((fitted_alpha / alpha - 1.0).abs() < 0.03);
        assert!((fitted_n - n).abs() < 0.08);
        assert!(fit.rmse_m3_m3.unwrap() < 1.0e-4);
        assert!(fit.figure_table_alpha_rmse_m3_m3.unwrap() < 1.0e-10);
    }

    #[test]
    fn retention_fit_abstains_when_too_few_pore_points_exist() {
        let points = vec![retention_point(-0.01, 0.10), retention_point(-0.02, 0.15)];
        let fit =
            fit_van_genuchten_retention_curve(&points, 0.2, Some(1500.0), Some(-500.0), Some(2.8));
        assert_eq!(fit.status, "not_fitted_insufficient_retention_points");
        assert_eq!(fit.alpha2_fit_per_m, None);
        assert_eq!(fit.figure_table_alpha_rmse_m3_m3, Some(0.0));
        assert_eq!(fit.printed_eq37_minus_alpha_rmse_m3_m3, None);
    }

    #[test]
    fn equation_35_returns_positive_dimensionless_conversion() {
        let result = conversion_factor_rp(20.0, 2.0).unwrap();
        assert!(result.is_finite());
        assert!(result > 1.0);
    }

    #[test]
    fn equation_36_scales_with_pore_area_and_dynamic_width() {
        let rp = conversion_factor_rp(20.0, 2.0).unwrap();
        let fp = rp * 5.0 * 2.0 / (100.0 * 4.0);
        let doubled = rp * 10.0 * 2.0 / (100.0 * 4.0);
        assert!((doubled - 2.0 * fp).abs() < 1.0e-12);
    }

    #[test]
    fn zero_dynamic_width_produces_zero_volume_and_no_finite_head_claim() {
        let config = Config {
            schema_version: "0.1.0".to_owned(),
            study_id: "S".to_owned(),
            profile_id: "P".to_owned(),
            horizon_label: "Ap1".to_owned(),
            scan_id: "scan".to_owned(),
            image_source_sha256: "a".repeat(64),
            image_evidence_ref: "synthetic".to_owned(),
            morphology_evidence_ref: "synthetic".to_owned(),
            dynamic_model_evidence_ref: "synthetic".to_owned(),
            image_resolution_um_per_pixel: 180.0,
            cross_section_area_mm2: 100.0,
            minimum_object_area_mm2: 0.0,
            columns: ColumnMapping {
                area: "Area".to_owned(),
                perimeter: "Perim.".to_owned(),
                min_feret: "MinFeret".to_owned(),
            },
        };
        let dynamic = DynamicReport {
            model_version: "0.1.0".to_owned(),
            input_sha256: "b".repeat(64),
            study_id: "S".to_owned(),
            profile_id: "P".to_owned(),
            horizon_label: "Ap1".to_owned(),
            image_geometry: DynamicImageGeometry {
                image_resolution_um_per_pixel: 180.0,
                cross_section_area_mm2: 100.0,
                projected_macropore_area_mm2: 10.0,
                total_macropore_perimeter_mm: 20.0,
                mean_dry_macropore_width_mm: 2.0,
                minimum_object_area_threshold_mm2: 0.0,
                included_macropore_object_count: 1,
            },
            fluid_properties: FluidProperties {
                surface_tension_n_m: 0.0728,
                water_density_kg_m3: 1000.0,
                gravity_m_s2: 9.80665,
            },
            derived_geometry: DynamicDerivedGeometry {
                dry_structural_unit_width_mm: 18.0,
                dry_mean_macropore_width_mm: 2.0,
                total_dry_repeat_width_mm: 20.0,
            },
            maximum_macropore_volume_fraction_at_wilting_point: 0.1,
            time_points: vec![DynamicTimePoint {
                timestamp_utc: "2026-01-01T00:00:00Z".to_owned(),
                matrix_water_content_m3_m3: 0.3,
                structural_unit_width_mm: 20.0,
                macropore_width_mm: 0.0,
                macropore_volume_fraction: 0.0,
                structural_alpha_per_m: None,
                structural_alpha_figure_table_plus_per_m: None,
                structural_alpha_printed_eq37_minus_per_m: None,
                structural_n: None,
            }],
        };
        let state = per_pore_state(
            &[pore("A", 10.0, 20.0, 2.0)],
            &config,
            &dynamic,
            &dynamic.time_points[0],
        )
        .unwrap();
        assert_eq!(state.per_pore_volume_fraction_sum, 0.0);
        assert_eq!(state.per_pore_retention_curve[0].matric_head_m, None);
        assert_eq!(
            state.per_pore_retention_curve[0].individual_conversion_factor_rp,
            None
        );
    }

    #[test]
    fn rejects_nonmatching_geometry_and_horizon() {
        let config = Config {
            schema_version: "0.1.0".to_owned(),
            study_id: "S".to_owned(),
            profile_id: "P".to_owned(),
            horizon_label: "Ap1".to_owned(),
            scan_id: "scan".to_owned(),
            image_source_sha256: "a".repeat(64),
            image_evidence_ref: "s".to_owned(),
            morphology_evidence_ref: "s".to_owned(),
            dynamic_model_evidence_ref: "s".to_owned(),
            image_resolution_um_per_pixel: 180.0,
            cross_section_area_mm2: 100.0,
            minimum_object_area_mm2: 0.0,
            columns: ColumnMapping {
                area: "Area".to_owned(),
                perimeter: "Perim.".to_owned(),
                min_feret: "MinFeret".to_owned(),
            },
        };
        let mut report: DynamicReport=serde_json::from_str(r#"{
        "model_version":"0.1.0","input_sha256":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
        "study_id":"S","profile_id":"P","horizon_label":"Bt",
        "image_geometry":{"image_resolution_um_per_pixel":180,"cross_section_area_mm2":100,"projected_macropore_area_mm2":10,"total_macropore_perimeter_mm":20,"mean_dry_macropore_width_mm":2,"minimum_object_area_threshold_mm2":0,"included_macropore_object_count":1},
        "fluid_properties":{"surface_tension_n_m":0.0728,"water_density_kg_m3":1000,"gravity_m_s2":9.80665},
        "derived_geometry":{"dry_structural_unit_width_mm":18,"dry_mean_macropore_width_mm":2,"total_dry_repeat_width_mm":20},
        "maximum_macropore_volume_fraction_at_wilting_point":0.1,
        "time_points":[{"timestamp_utc":"2026-01-01T00:00:00Z","matrix_water_content_m3_m3":0.3,"structural_unit_width_mm":18,"macropore_width_mm":2,"macropore_volume_fraction":0.1}]
        }"#).unwrap();
        report.horizon_label = "Bt".to_owned();
        assert!(
            validate_dynamic(&config, &report)
                .unwrap_err()
                .contains("identity does not match")
        );
    }

    #[test]
    fn stable_sort_orders_retention_curve_by_dynamic_pore_width() {
        let mut values = vec![
            (4.0, "B".to_owned()),
            (1.0, "A".to_owned()),
            (2.0, "C".to_owned()),
        ];
        values.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
        assert_eq!(
            values
                .iter()
                .map(|item| item.1.as_str())
                .collect::<Vec<_>>(),
            vec!["A", "C", "B"]
        );
    }

    #[test]
    fn strict_size_filter_excludes_objects_equal_to_threshold() {
        let threshold = std::f64::consts::PI;
        let rows = [
            pore("A", threshold, 10.0, 1.0),
            pore("B", threshold + 0.001, 20.0, 1.5),
        ];
        let retained: Vec<_> = rows
            .iter()
            .filter(|item| item.area_mm2 > threshold)
            .collect();
        assert_eq!(retained.len(), 1);
        assert_eq!(retained[0].object_id, "B");
    }
}
