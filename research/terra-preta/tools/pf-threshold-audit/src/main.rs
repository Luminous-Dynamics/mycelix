use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, HashSet},
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const AUDIT_VERSION: &str = "0.1.0";

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ValueScale {
    LinearCmPerDay,
    StoredLogScaleCmPerDay,
}

impl ValueScale {
    fn as_str(self) -> &'static str {
        match self {
            Self::LinearCmPerDay => "linear_cm_per_day",
            Self::StoredLogScaleCmPerDay => "stored_log_scale_cm_per_day",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
enum ThresholdKind {
    RosettaKsatLow,
    RosettaKsatMax,
    RosettaKsatHigh,
    RosettaKsatStdMedian,
    RosettaKsatStdUpper,
    MatrixVelocityMedian,
    MatrixVelocity95th,
    MatrixVelocity99th,
    OtherDeclared,
}

impl ThresholdKind {
    fn as_str(self) -> &'static str {
        match self {
            Self::RosettaKsatLow => "rosetta_ksat_low",
            Self::RosettaKsatMax => "rosetta_ksat_max",
            Self::RosettaKsatHigh => "rosetta_ksat_high",
            Self::RosettaKsatStdMedian => "rosetta_ksat_std_median",
            Self::RosettaKsatStdUpper => "rosetta_ksat_std_upper",
            Self::MatrixVelocityMedian => "matrix_velocity_median",
            Self::MatrixVelocity95th => "matrix_velocity_95th",
            Self::MatrixVelocity99th => "matrix_velocity_99th",
            Self::OtherDeclared => "other_declared",
        }
    }

    fn is_matrix_model_velocity(self) -> bool {
        matches!(
            self,
            Self::MatrixVelocityMedian | Self::MatrixVelocity95th | Self::MatrixVelocity99th
        )
    }

    fn is_rosetta_ksat(self) -> bool {
        matches!(
            self,
            Self::RosettaKsatLow
                | Self::RosettaKsatMax
                | Self::RosettaKsatHigh
                | Self::RosettaKsatStdMedian
                | Self::RosettaKsatStdUpper
        )
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Velocity {
    value: f64,
    value_scale: ValueScale,
    source_column: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Threshold {
    threshold_id: String,
    kind: ThresholdKind,
    value: f64,
    value_scale: ValueScale,
    source_column: String,
    evidence_ref: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    profile_id: String,
    event_id: String,
    sensor_id: String,
    sensor_depth_cm: f64,
    velocity: Velocity,
    thresholds: Vec<Threshold>,
    source_sha256: String,
    source_evidence_ref: String,
}

#[derive(Serialize)]
struct ThresholdResult {
    threshold_id: String,
    kind: &'static str,
    value: f64,
    value_scale: &'static str,
    source_column: String,
    evidence_ref: String,
    preferential_flow_proxy: bool,
    margin_velocity_minus_threshold: f64,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    site_id: String,
    profile_id: String,
    event_id: String,
    sensor_id: String,
    sensor_depth_cm: f64,
    velocity_value: f64,
    value_scale: &'static str,
    velocity_source_column: String,
    comparison_operator: &'static str,
    threshold_count: usize,
    rosetta_ksat_threshold_count: usize,
    rosetta_ksat_positive_count: usize,
    rosetta_ksat_negative_count: usize,
    rosetta_ksat_proxy_stability: &'static str,
    matrix_model_velocity_threshold_count: usize,
    matrix_model_velocity_positive_count: usize,
    matrix_model_velocity_negative_count: usize,
    matrix_model_velocity_proxy_stability: &'static str,
    matrix_velocity_99th_threshold_count: usize,
    matrix_velocity_99th_positive_count: usize,
    matrix_velocity_99th_negative_count: usize,
    matrix_velocity_99th_proxy_stability: &'static str,
    threshold_results: Vec<ThresholdResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn finite_value(value: f64, scale: ValueScale, label: &str) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{label} must be finite"));
    }
    if matches!(scale, ValueScale::LinearCmPerDay) && value <= 0.0 {
        return Err(format!("{label} must be greater than zero on linear cm/day scale"));
    }
    Ok(())
}

fn stability(count: usize, positives: usize) -> &'static str {
    if count == 0 {
        "not_evaluated"
    } else if count == 1 {
        "single_reference_not_robustness_tested"
    } else if positives == count {
        "stable_positive_across_declared_thresholds"
    } else if positives == 0 {
        "stable_negative_across_declared_thresholds"
    } else {
        "threshold_sensitive"
    }
}

fn validate(input: &Input) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported schema_version {:?}",
            input.schema_version
        ));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
        (&input.sensor_id, "sensor_id"),
        (&input.velocity.source_column, "velocity.source_column"),
        (&input.source_evidence_ref, "source_evidence_ref"),
    ] {
        nonblank(value, label)?;
    }
    if input.source_sha256.len() != 64
        || !input.source_sha256.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if !input.sensor_depth_cm.is_finite() || input.sensor_depth_cm <= 0.0 {
        return Err("sensor_depth_cm must be finite and greater than zero".to_owned());
    }
    finite_value(
        input.velocity.value,
        input.velocity.value_scale,
        "velocity.value",
    )?;
    if input.thresholds.is_empty() {
        return Err("at least one comparison threshold is required".to_owned());
    }

    let mut ids = HashSet::new();
    let mut kinds = HashMap::new();
    for (index, threshold) in input.thresholds.iter().enumerate() {
        nonblank(
            &threshold.threshold_id,
            &format!("thresholds[{index}].threshold_id"),
        )?;
        nonblank(
            &threshold.source_column,
            &format!("thresholds[{index}].source_column"),
        )?;
        nonblank(
            &threshold.evidence_ref,
            &format!("thresholds[{index}].evidence_ref"),
        )?;
        if !ids.insert(threshold.threshold_id.as_str()) {
            return Err(format!(
                "duplicate threshold_id {:?}",
                threshold.threshold_id
            ));
        }
        if kinds.insert(threshold.kind, threshold.value).is_some() {
            return Err(format!(
                "duplicate threshold kind {:?}",
                threshold.kind.as_str()
            ));
        }
        if threshold.value_scale != input.velocity.value_scale {
            return Err(format!(
                "threshold {:?} uses scale {}, but velocity uses {}; implicit scale conversion is forbidden",
                threshold.threshold_id,
                threshold.value_scale.as_str(),
                input.velocity.value_scale.as_str()
            ));
        }
        finite_value(
            threshold.value,
            threshold.value_scale,
            &format!("thresholds[{index}].value"),
        )?;
    }

    if let (Some(low), Some(max), Some(high)) = (
        kinds.get(&ThresholdKind::RosettaKsatLow),
        kinds.get(&ThresholdKind::RosettaKsatMax),
        kinds.get(&ThresholdKind::RosettaKsatHigh),
    ) {
        if low > max || max > high {
            return Err("Rosetta Ksat thresholds must satisfy low <= max <= high".to_owned());
        }
    }
    if let (Some(median), Some(upper)) = (
        kinds.get(&ThresholdKind::RosettaKsatStdMedian),
        kinds.get(&ThresholdKind::RosettaKsatStdUpper),
    ) {
        if median > upper {
            return Err("Rosetta median Ksat threshold must not exceed its upper estimate".to_owned());
        }
    }
    Ok(())
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    validate(input)?;
    let mut threshold_results = Vec::with_capacity(input.thresholds.len());
    let mut rosetta_count = 0usize;
    let mut rosetta_positive = 0usize;
    let mut matrix_model_velocity_count = 0usize;
    let mut matrix_model_velocity_positive = 0usize;
    let mut matrix_velocity_99th_count = 0usize;
    let mut matrix_velocity_99th_positive = 0usize;

    for threshold in &input.thresholds {
        let margin = input.velocity.value - threshold.value;
        let positive = input.velocity.value > threshold.value;
        if threshold.kind.is_rosetta_ksat() {
            rosetta_count += 1;
            if positive {
                rosetta_positive += 1;
            }
        } else if threshold.kind.is_matrix_model_velocity() {
            matrix_model_velocity_count += 1;
            if positive {
                matrix_model_velocity_positive += 1;
            }
            if threshold.kind == ThresholdKind::MatrixVelocity99th {
                matrix_velocity_99th_count += 1;
                if positive {
                    matrix_velocity_99th_positive += 1;
                }
            }
        }

        threshold_results.push(ThresholdResult {
            threshold_id: threshold.threshold_id.clone(),
            kind: threshold.kind.as_str(),
            value: threshold.value,
            value_scale: threshold.value_scale.as_str(),
            source_column: threshold.source_column.clone(),
            evidence_ref: threshold.evidence_ref.clone(),
            preferential_flow_proxy: positive,
            margin_velocity_minus_threshold: margin,
        });
    }

    let rosetta_negative = rosetta_count - rosetta_positive;
    let matrix_model_velocity_negative =
        matrix_model_velocity_count - matrix_model_velocity_positive;
    let matrix_velocity_99th_negative =
        matrix_velocity_99th_count - matrix_velocity_99th_positive;

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        profile_id: input.profile_id.clone(),
        event_id: input.event_id.clone(),
        sensor_id: input.sensor_id.clone(),
        sensor_depth_cm: input.sensor_depth_cm,
        velocity_value: input.velocity.value,
        value_scale: input.velocity.value_scale.as_str(),
        velocity_source_column: input.velocity.source_column.clone(),
        comparison_operator: "strict_greater_than",
        threshold_count: input.thresholds.len(),
        rosetta_ksat_threshold_count: rosetta_count,
        rosetta_ksat_positive_count: rosetta_positive,
        rosetta_ksat_negative_count: rosetta_negative,
        rosetta_ksat_proxy_stability: stability(rosetta_count, rosetta_positive),
        matrix_model_velocity_threshold_count: matrix_model_velocity_count,
        matrix_model_velocity_positive_count: matrix_model_velocity_positive,
        matrix_model_velocity_negative_count: matrix_model_velocity_negative,
        matrix_model_velocity_proxy_stability: stability(
            matrix_model_velocity_count,
            matrix_model_velocity_positive,
        ),
        matrix_velocity_99th_threshold_count: matrix_velocity_99th_count,
        matrix_velocity_99th_positive_count: matrix_velocity_99th_positive,
        matrix_velocity_99th_negative_count: matrix_velocity_99th_negative,
        matrix_velocity_99th_proxy_stability: stability(
            matrix_velocity_99th_count,
            matrix_velocity_99th_positive,
        ),
        threshold_results,
        interpretation_boundary: "sensitivity of velocity-threshold proxy labels to declared hydraulic/model thresholds; not independent preferential-flow truth",
        warnings: vec![
            "All compared values must use the same declared numeric scale. Stored log-scale inputs are compared as stored; the log base is not assumed, and no exponentiation or unit conversion is inferred.".to_owned(),
            "Rosetta Ksat thresholds and the matrix-model 99th-percentile velocity threshold are separate reference families and are not pooled into one consensus label.".to_owned(),
            "A stable label across the supplied thresholds only means robust to these declared threshold choices; it does not prove a physical preferential pathway.".to_owned(),
            "A threshold-sensitive result is evidence that the PF proxy class depends on the selected hydraulic/model reference and should not be treated as a binary ground-truth label.".to_owned(),
            "Strict greater-than is applied exactly: velocity equal to a threshold is classified as not preferential under that threshold.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: pf-threshold-audit <threshold-input.json> <new-report.json>",
        )
        .into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("refusing overwrite or cannot create report: {error}"),
            )
        })?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;

    println!(
        "PF threshold audit complete: {} thresholds; Rosetta stability={}; matrix-velocity stability={}; input_sha256={}",
        report.threshold_count,
        report.rosetta_ksat_proxy_stability,
        report.matrix_model_velocity_proxy_stability,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("PF threshold audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn threshold(id: &str, kind: ThresholdKind, value: f64) -> Threshold {
        Threshold {
            threshold_id: id.to_owned(),
            kind,
            value,
            value_scale: ValueScale::StoredLogScaleCmPerDay,
            source_column: id.to_owned(),
            evidence_ref: "synthetic fixture only".to_owned(),
        }
    }

    fn input(velocity: f64, thresholds: Vec<Threshold>) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            profile_id: "P01".to_owned(),
            event_id: "E001".to_owned(),
            sensor_id: "S30".to_owned(),
            sensor_depth_cm: 30.0,
            velocity: Velocity {
                value: velocity,
                value_scale: ValueScale::StoredLogScaleCmPerDay,
                source_column: "velocity_computed_50X".to_owned(),
            },
            thresholds,
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic fixture only".to_owned(),
        }
    }

    fn rosetta_thresholds() -> Vec<Threshold> {
        vec![
            threshold("ksat_low", ThresholdKind::RosettaKsatLow, 0.70),
            threshold("ksat_max", ThresholdKind::RosettaKsatMax, 1.00),
            threshold("ksat_high", ThresholdKind::RosettaKsatHigh, 1.30),
            threshold(
                "ksat_std_median",
                ThresholdKind::RosettaKsatStdMedian,
                0.80,
            ),
            threshold(
                "ksat_std_upper",
                ThresholdKind::RosettaKsatStdUpper,
                1.10,
            ),
        ]
    }

    #[test]
    fn reports_threshold_sensitive_rosetta_proxy() {
        let report = evaluate(&input(0.90, rosetta_thresholds()), "b".repeat(64)).unwrap();
        assert_eq!(report.rosetta_ksat_positive_count, 2);
        assert_eq!(report.rosetta_ksat_negative_count, 3);
        assert_eq!(report.rosetta_ksat_proxy_stability, "threshold_sensitive");
    }

    #[test]
    fn reports_stable_positive_across_declared_thresholds() {
        let report = evaluate(&input(1.50, rosetta_thresholds()), "c".repeat(64)).unwrap();
        assert_eq!(
            report.rosetta_ksat_proxy_stability,
            "stable_positive_across_declared_thresholds"
        );
    }

    #[test]
    fn reports_stable_negative_across_declared_thresholds() {
        let report = evaluate(&input(0.50, rosetta_thresholds()), "d".repeat(64)).unwrap();
        assert_eq!(
            report.rosetta_ksat_proxy_stability,
            "stable_negative_across_declared_thresholds"
        );
    }

    #[test]
    fn keeps_matrix_velocity_threshold_separate_from_rosetta_family() {
        let mut thresholds = rosetta_thresholds();
        thresholds.push(threshold(
            "num_velocity_99th",
            ThresholdKind::MatrixVelocity99th,
            1.20,
        ));
        let report = evaluate(&input(0.90, thresholds), "e".repeat(64)).unwrap();
        assert_eq!(report.matrix_velocity_99th_threshold_count, 1);
        assert_eq!(report.matrix_velocity_99th_positive_count, 0);
        assert_eq!(
            report.matrix_velocity_99th_proxy_stability,
            "single_reference_not_robustness_tested"
        );
        assert_eq!(report.rosetta_ksat_proxy_stability, "threshold_sensitive");
    }

    #[test]
    fn strict_greater_than_means_equal_velocity_is_not_positive() {
        let thresholds = vec![
            threshold("ksat_max", ThresholdKind::RosettaKsatMax, 1.0),
            threshold("ksat_high", ThresholdKind::RosettaKsatHigh, 1.1),
        ];
        let report = evaluate(&input(1.0, thresholds), "f".repeat(64)).unwrap();
        assert_eq!(report.rosetta_ksat_positive_count, 0);
        assert_eq!(
            report.rosetta_ksat_proxy_stability,
            "stable_negative_across_declared_thresholds"
        );
    }

    #[test]
    fn rejects_mixed_log_and_linear_scales() {
        let mut thresholds = rosetta_thresholds();
        thresholds[0].value_scale = ValueScale::LinearCmPerDay;
        assert!(validate(&input(0.9, thresholds))
            .unwrap_err()
            .contains("implicit scale conversion is forbidden"));
    }

    #[test]
    fn permits_negative_log_values_but_rejects_nonpositive_linear_values() {
        let mut data = input(-0.3, vec![
            threshold("ksat_low", ThresholdKind::RosettaKsatLow, -0.8),
            threshold("ksat_max", ThresholdKind::RosettaKsatMax, -0.4),
        ]);
        assert!(validate(&data).is_ok());
        data.velocity.value_scale = ValueScale::LinearCmPerDay;
        data.velocity.value = 0.9;
        data.thresholds[0].value_scale = ValueScale::LinearCmPerDay;
        data.thresholds[0].value = 0.0;
        data.thresholds[1].value_scale = ValueScale::LinearCmPerDay;
        data.thresholds[1].value = 1.0;
        assert!(validate(&data).unwrap_err().contains("greater than zero"));
    }

    #[test]
    fn rejects_inverted_rosetta_low_max_high_bounds() {
        let mut thresholds = rosetta_thresholds();
        thresholds[0].value = 1.1;
        assert!(validate(&input(1.2, thresholds))
            .unwrap_err()
            .contains("low <= max <= high"));
    }

    #[test]
    fn rejects_inverted_median_upper_bounds() {
        let thresholds = vec![
            threshold("ksat_std_median", ThresholdKind::RosettaKsatStdMedian, 1.2),
            threshold("ksat_std_upper", ThresholdKind::RosettaKsatStdUpper, 1.1),
        ];
        assert!(validate(&input(1.3, thresholds))
            .unwrap_err()
            .contains("median Ksat threshold"));
    }

    #[test]
    fn rejects_duplicate_threshold_ids_and_kinds() {
        let thresholds = vec![
            threshold("same", ThresholdKind::RosettaKsatLow, 0.5),
            threshold("same", ThresholdKind::RosettaKsatMax, 0.7),
        ];
        assert!(validate(&input(1.0, thresholds))
            .unwrap_err()
            .contains("duplicate threshold_id"));
        let thresholds = vec![
            threshold("low_a", ThresholdKind::RosettaKsatLow, 0.5),
            threshold("low_b", ThresholdKind::RosettaKsatLow, 0.6),
        ];
        assert!(validate(&input(1.0, thresholds))
            .unwrap_err()
            .contains("duplicate threshold kind"));
    }

    #[test]
    fn rejects_invalid_source_digest() {
        let mut data = input(1.0, rosetta_thresholds());
        data.source_sha256 = "not-a-sha256".to_owned();
        assert!(validate(&data).unwrap_err().contains("source_sha256"));
    }
}
