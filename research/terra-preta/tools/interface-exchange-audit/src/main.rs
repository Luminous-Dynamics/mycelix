use chrono::{DateTime, FixedOffset};
#[cfg(test)]
use chrono::{Duration, TimeZone, Utc};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const AUDIT_VERSION: &str = "0.1.0";
const AREA_BASIS: &str = "m2_interface_per_m3_bulk_soil";
const FLUX_BASIS: &str = "m3_water_per_m2_interface_per_s";
const SIGN_CONVENTION: &str = "positive_macropore_to_matrix";

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum EvidenceStatus {
    CtMeasured,
    DerivedFromCt,
    Modelled,
    Estimated,
}

impl EvidenceStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::CtMeasured => "ct_measured",
            Self::DerivedFromCt => "derived_from_ct",
            Self::Modelled => "modelled",
            Self::Estimated => "estimated",
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    sample_id: String,
    horizon_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    maximum_allowed_interval_seconds: u64,
    profile_depth_mm: f64,
    bulk_sample_volume_m3: f64,
    area_normalization_basis: String,
    flux_density_basis: String,
    sign_convention: String,
    equation_reference: String,
    source_sha256: String,
    interface_area_source_sha256: String,
    flux_density_source_sha256: String,
    samples: Vec<ExchangeSample>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExchangeSample {
    timestamp_utc: String,
    specific_wetted_interface_area_m2_per_m3_bulk: f64,
    macropore_to_matrix_flux_density_m_s: f64,
    interface_area_status: EvidenceStatus,
    flux_density_status: EvidenceStatus,
    interface_area_evidence_ref: String,
    flux_density_evidence_ref: String,
}

#[derive(Clone, Debug)]
struct ParsedSample {
    timestamp: DateTime<FixedOffset>,
    area_m2_per_m3: f64,
    flux_m_s: f64,
    exchange_rate_s_inv: f64,
    interface_area_status: EvidenceStatus,
    flux_density_status: EvidenceStatus,
}

#[derive(Serialize)]
struct SampleResult {
    timestamp_utc: String,
    specific_wetted_interface_area_m2_per_m3_bulk: f64,
    macropore_to_matrix_flux_density_m_s: f64,
    exchange_rate_per_s: f64,
    exchange_rate_depth_equivalent_mm_per_s: f64,
    interface_area_status: &'static str,
    flux_density_status: &'static str,
    interface_area_evidence_ref: String,
    flux_density_evidence_ref: String,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    interface_area_source_sha256: String,
    flux_density_source_sha256: String,
    study_id: String,
    site_id: String,
    sample_id: String,
    horizon_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    maximum_allowed_interval_seconds: u64,
    profile_depth_mm: f64,
    bulk_sample_volume_m3: f64,
    area_normalization_basis: &'static str,
    flux_density_basis: &'static str,
    sign_convention: &'static str,
    equation_reference: String,
    integration_method: &'static str,
    sample_count: usize,
    measured_interface_area_sample_count: usize,
    derived_interface_area_sample_count: usize,
    modelled_interface_area_sample_count: usize,
    estimated_interface_area_sample_count: usize,
    modelled_flux_density_sample_count: usize,
    measured_or_ct_derived_flux_density_sample_count: usize,
    estimated_flux_density_sample_count: usize,
    peak_exchange_rate_per_s: f64,
    cumulative_gross_exchange_m3_per_m3_bulk: f64,
    cumulative_gross_exchange_volume_m3: f64,
    cumulative_gross_exchange_depth_mm: f64,
    mean_exchange_rate_per_s: f64,
    interval_count: usize,
    minimum_interval_seconds: f64,
    maximum_interval_seconds: f64,
    sample_results: Vec<SampleResult>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn valid_sha256(value: &str, label: &str) -> Result<(), String> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("{label} must contain exactly 64 hexadecimal characters"));
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
    let timestamp = DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{label}: invalid RFC3339 timestamp: {error}"))?;
    if timestamp.offset().local_minus_utc() != 0 {
        return Err(format!("{label}: timestamp must use a zero UTC offset"));
    }
    Ok(timestamp)
}

fn validate(input: &Input) -> Result<Vec<ParsedSample>, String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.sample_id, "sample_id"),
        (&input.horizon_id, "horizon_id"),
        (&input.event_id, "event_id"),
        (&input.equation_reference, "equation_reference"),
    ] {
        nonblank(value, label)?;
    }
    valid_sha256(&input.source_sha256, "source_sha256")?;
    valid_sha256(&input.interface_area_source_sha256, "interface_area_source_sha256")?;
    valid_sha256(&input.flux_density_source_sha256, "flux_density_source_sha256")?;

    if input.area_normalization_basis != AREA_BASIS {
        return Err(format!("area_normalization_basis must be {AREA_BASIS}"));
    }
    if input.flux_density_basis != FLUX_BASIS {
        return Err(format!("flux_density_basis must be {FLUX_BASIS}"));
    }
    if input.sign_convention != SIGN_CONVENTION {
        return Err(format!("sign_convention must be {SIGN_CONVENTION}"));
    }
    finite_nonnegative(input.profile_depth_mm, "profile_depth_mm")?;
    finite_nonnegative(input.bulk_sample_volume_m3, "bulk_sample_volume_m3")?;
    if input.profile_depth_mm <= 0.0 || input.bulk_sample_volume_m3 <= 0.0 {
        return Err("profile depth and bulk sample volume must be greater than zero".to_owned());
    }
    if input.samples.len() < 2 {
        return Err("at least two time samples are required for interval integration".to_owned());
    }
    if input.maximum_allowed_interval_seconds == 0 {
        return Err("maximum_allowed_interval_seconds must be greater than zero".to_owned());
    }

    let event_start = parse_utc(&input.event_start_utc, "event_start_utc")?;
    let event_end = parse_utc(&input.observation_window_end_utc, "observation_window_end_utc")?;
    if event_start >= event_end {
        return Err("observation_window_end_utc must be after event_start_utc".to_owned());
    }

    let mut parsed = Vec::with_capacity(input.samples.len());
    for (index, sample) in input.samples.iter().enumerate() {
        let timestamp = parse_utc(&sample.timestamp_utc, &format!("samples[{index}].timestamp_utc"))?;
        if timestamp < event_start || timestamp > event_end {
            return Err(format!("samples[{index}] timestamp lies outside the event window"));
        }
        finite_nonnegative(
            sample.specific_wetted_interface_area_m2_per_m3_bulk,
            &format!("samples[{index}].specific_wetted_interface_area_m2_per_m3_bulk"),
        )?;
        finite_nonnegative(
            sample.macropore_to_matrix_flux_density_m_s,
            &format!("samples[{index}].macropore_to_matrix_flux_density_m_s"),
        )?;
        nonblank(
            &sample.interface_area_evidence_ref,
            &format!("samples[{index}].interface_area_evidence_ref"),
        )?;
        nonblank(
            &sample.flux_density_evidence_ref,
            &format!("samples[{index}].flux_density_evidence_ref"),
        )?;
        let rate = sample.specific_wetted_interface_area_m2_per_m3_bulk
            * sample.macropore_to_matrix_flux_density_m_s;
        if !rate.is_finite() {
            return Err(format!("samples[{index}] exchange rate overflowed"));
        }
        parsed.push(ParsedSample {
            timestamp,
            area_m2_per_m3: sample.specific_wetted_interface_area_m2_per_m3_bulk,
            flux_m_s: sample.macropore_to_matrix_flux_density_m_s,
            exchange_rate_s_inv: rate,
            interface_area_status: sample.interface_area_status,
            flux_density_status: sample.flux_density_status,
        });
    }

    for pair in parsed.windows(2) {
        if pair[1].timestamp <= pair[0].timestamp {
            return Err("sample timestamps must be strictly increasing with no duplicates".to_owned());
        }
        let elapsed = (pair[1].timestamp.clone() - pair[0].timestamp.clone()).num_milliseconds() as f64 / 1000.0;
        if elapsed > input.maximum_allowed_interval_seconds as f64 {
            return Err(format!(
                "adjacent sample interval {elapsed} seconds exceeds maximum_allowed_interval_seconds {}",
                input.maximum_allowed_interval_seconds
            ));
        }
    }
    if parsed.first().map(|sample| sample.timestamp.clone()) != Some(event_start.clone())
        || parsed.last().map(|sample| sample.timestamp.clone()) != Some(event_end.clone())
    {
        return Err("first and last sample timestamps must equal the declared event window boundaries".to_owned());
    }
    Ok(parsed)
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    let parsed = validate(input)?;
    let mut integrated_exchange_fraction = 0.0f64;
    let mut minimum_interval_seconds = f64::INFINITY;
    let mut maximum_interval_seconds: f64 = 0.0;
    let mut peak_exchange_rate_per_s: f64 = 0.0;

    let mut measured_area_count = 0usize;
    let mut derived_area_count = 0usize;
    let mut modelled_area_count = 0usize;
    let mut estimated_area_count = 0usize;
    let mut modelled_flux_count = 0usize;
    let mut measured_or_ct_derived_flux_count = 0usize;
    let mut estimated_flux_count = 0usize;

    let mut sample_results = Vec::with_capacity(parsed.len());
    for (index, sample) in parsed.iter().enumerate() {
        peak_exchange_rate_per_s = peak_exchange_rate_per_s.max(sample.exchange_rate_s_inv);
        match sample.interface_area_status {
            EvidenceStatus::CtMeasured => measured_area_count += 1,
            EvidenceStatus::DerivedFromCt => derived_area_count += 1,
            EvidenceStatus::Modelled => modelled_area_count += 1,
            EvidenceStatus::Estimated => estimated_area_count += 1,
        }
        match sample.flux_density_status {
            EvidenceStatus::Modelled => modelled_flux_count += 1,
            EvidenceStatus::CtMeasured | EvidenceStatus::DerivedFromCt => {
                measured_or_ct_derived_flux_count += 1;
            }
            EvidenceStatus::Estimated => estimated_flux_count += 1,
        }
        sample_results.push(SampleResult {
            timestamp_utc: sample.timestamp.to_rfc3339(),
            specific_wetted_interface_area_m2_per_m3_bulk: sample.area_m2_per_m3,
            macropore_to_matrix_flux_density_m_s: sample.flux_m_s,
            exchange_rate_per_s: sample.exchange_rate_s_inv,
            exchange_rate_depth_equivalent_mm_per_s: sample.exchange_rate_s_inv * input.profile_depth_mm,
            interface_area_status: sample.interface_area_status.as_str(),
            flux_density_status: sample.flux_density_status.as_str(),
            interface_area_evidence_ref: input.samples[index].interface_area_evidence_ref.clone(),
            flux_density_evidence_ref: input.samples[index].flux_density_evidence_ref.clone(),
        });
    }

    for pair in parsed.windows(2) {
        let elapsed_seconds = (pair[1].timestamp - pair[0].timestamp).num_milliseconds() as f64 / 1000.0;
        if !elapsed_seconds.is_finite() || elapsed_seconds <= 0.0 {
            return Err("time interval must be finite and positive".to_owned());
        }
        minimum_interval_seconds = minimum_interval_seconds.min(elapsed_seconds);
        maximum_interval_seconds = maximum_interval_seconds.max(elapsed_seconds);
        integrated_exchange_fraction +=
            0.5 * (pair[0].exchange_rate_s_inv + pair[1].exchange_rate_s_inv) * elapsed_seconds;
        if !integrated_exchange_fraction.is_finite() {
            return Err("integrated gross exchange overflowed or became non-finite".to_owned());
        }
    }

    let cumulative_gross_exchange_volume_m3 =
        integrated_exchange_fraction * input.bulk_sample_volume_m3;
    let cumulative_gross_exchange_depth_mm =
        integrated_exchange_fraction * input.profile_depth_mm;
    let duration_seconds = (parsed.last().unwrap().timestamp.clone()
        - parsed.first().unwrap().timestamp.clone()).num_milliseconds() as f64 / 1000.0;
    let mean_exchange_rate_per_s = integrated_exchange_fraction / duration_seconds;

    if !cumulative_gross_exchange_volume_m3.is_finite()
        || !cumulative_gross_exchange_depth_mm.is_finite()
        || !mean_exchange_rate_per_s.is_finite()
    {
        return Err("derived integrated exchange output overflowed or became non-finite".to_owned());
    }

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        interface_area_source_sha256: input.interface_area_source_sha256.clone(),
        flux_density_source_sha256: input.flux_density_source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        sample_id: input.sample_id.clone(),
        horizon_id: input.horizon_id.clone(),
        event_id: input.event_id.clone(),
        event_start_utc: input.event_start_utc.clone(),
        observation_window_end_utc: input.observation_window_end_utc.clone(),
        maximum_allowed_interval_seconds: input.maximum_allowed_interval_seconds,
        profile_depth_mm: input.profile_depth_mm,
        bulk_sample_volume_m3: input.bulk_sample_volume_m3,
        area_normalization_basis: AREA_BASIS,
        flux_density_basis: FLUX_BASIS,
        sign_convention: SIGN_CONVENTION,
        equation_reference: input.equation_reference.clone(),
        integration_method: "trapezoidal_integration_of_a_wetted_specific_interface_area_times_macropore_flux_density",
        sample_count: parsed.len(),
        measured_interface_area_sample_count: measured_area_count,
        derived_interface_area_sample_count: derived_area_count,
        modelled_interface_area_sample_count: modelled_area_count,
        estimated_interface_area_sample_count: estimated_area_count,
        modelled_flux_density_sample_count: modelled_flux_count,
        measured_or_ct_derived_flux_density_sample_count: measured_or_ct_derived_flux_count,
        estimated_flux_density_sample_count: estimated_flux_count,
        peak_exchange_rate_per_s,
        cumulative_gross_exchange_m3_per_m3_bulk: integrated_exchange_fraction,
        cumulative_gross_exchange_volume_m3,
        cumulative_gross_exchange_depth_mm,
        mean_exchange_rate_per_s,
        interval_count: parsed.len() - 1,
        minimum_interval_seconds,
        maximum_interval_seconds,
        sample_results,
        interpretation_boundary: "dimensional audit and time integration of declared macropore-to-matrix wetted-interface exchange inputs; not CT segmentation, a Richards solver, a complete dual-permeability model, net water balance or measured treatment effect",
        warnings: vec![
            "The exact normalization and sign conventions must be checked against the full Chaif et al. (2025) equations/supplement before calling this a paper reproduction.".to_owned(),
            "Specific wetted interface area is normalized by bulk soil volume; flux density is per declared wetted interface area. Their product has units s^-1.".to_owned(),
            "The integral is gross one-way macropore-to-matrix exchange per bulk volume. It is not constrained to [0,1], is not net exchange, and may exceed one under repeated/cycling transfer.".to_owned(),
            "The calculator does not derive wetted interface area from CT, solve horizontal Richards flow, infer macropore flux density, simulate reverse matrix-to-macropore exchange or validate a hydraulic constitutive law.".to_owned(),
            "Measured, CT-derived, modelled and estimated evidence statuses are retained; source labels are not independently authenticated by this tool.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: interface-exchange-audit <exchange-series-input.json> <new-report.json>",
        ).into());
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
        "interface-exchange audit complete: {} samples; integrated exchange={:.8} m3/m3 bulk; peak={:.8} s^-1; input_sha256={}",
        report.sample_count,
        report.cumulative_gross_exchange_m3_per_m3_bulk,
        report.peak_exchange_rate_per_s,
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("interface-exchange audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(seconds: i64) -> String {
        let origin = Utc.with_ymd_and_hms(2026, 1, 1, 0, 0, 0).single().unwrap();
        (origin + Duration::seconds(seconds)).to_rfc3339()
    }

    fn sample(
        seconds: i64,
        area: f64,
        flux: f64,
        area_status: EvidenceStatus,
        flux_status: EvidenceStatus,
    ) -> ExchangeSample {
        ExchangeSample {
            timestamp_utc: stamp(seconds),
            specific_wetted_interface_area_m2_per_m3_bulk: area,
            macropore_to_matrix_flux_density_m_s: flux,
            interface_area_status: area_status,
            flux_density_status: flux_status,
            interface_area_evidence_ref: "synthetic fixture".to_owned(),
            flux_density_evidence_ref: "synthetic fixture".to_owned(),
        }
    }

    fn input(samples: Vec<ExchangeSample>) -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            sample_id: "CORE-01".to_owned(),
            horizon_id: "0-10cm".to_owned(),
            event_id: "RAIN-01".to_owned(),
            event_start_utc: stamp(0),
            observation_window_end_utc: stamp(3600),
            maximum_allowed_interval_seconds: 3600,
            profile_depth_mm: 100.0,
            bulk_sample_volume_m3: 0.0001,
            area_normalization_basis: AREA_BASIS.to_owned(),
            flux_density_basis: FLUX_BASIS.to_owned(),
            sign_convention: SIGN_CONVENTION.to_owned(),
            equation_reference: "Chaif et al. (2025) candidate formulation; synthetic test only".to_owned(),
            source_sha256: "a".repeat(64),
            interface_area_source_sha256: "b".repeat(64),
            flux_density_source_sha256: "c".repeat(64),
            samples,
        }
    }

    #[test]
    fn constant_area_and_flux_have_analytic_integral() {
        let data = input(vec![
            sample(0, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        let report = evaluate(&data, "d".repeat(64)).unwrap();
        assert!((report.peak_exchange_rate_per_s - 1.0e-5).abs() < 1.0e-15);
        assert!((report.cumulative_gross_exchange_m3_per_m3_bulk - 0.036).abs() < 1.0e-12);
        assert!((report.cumulative_gross_exchange_depth_mm - 3.6).abs() < 1.0e-10);
        assert!((report.cumulative_gross_exchange_volume_m3 - 3.6e-6).abs() < 1.0e-12);
    }

    #[test]
    fn time_varying_area_and_flux_use_trapezoidal_quadrature() {
        let data = input(vec![
            sample(0, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(1800, 20.0, 2.0e-6, EvidenceStatus::DerivedFromCt, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        let report = evaluate(&data, "e".repeat(64)).unwrap();
        assert!((report.cumulative_gross_exchange_m3_per_m3_bulk - 0.09).abs() < 1.0e-12);
        assert_eq!(report.derived_interface_area_sample_count, 1);
        assert_eq!(report.measured_interface_area_sample_count, 2);
    }

    #[test]
    fn irregular_time_intervals_use_actual_elapsed_seconds() {
        let mut data = input(vec![
            sample(0, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(900, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        data.samples[1].timestamp_utc = stamp(1200);
        let report = evaluate(&data, "f".repeat(64)).unwrap();
        assert!((report.cumulative_gross_exchange_m3_per_m3_bulk - 0.036).abs() < 1.0e-12);
        assert_eq!(report.minimum_interval_seconds, 1200.0);
        assert_eq!(report.maximum_interval_seconds, 2400.0);
    }

    #[test]
    fn rejects_duplicate_or_reversed_timestamps() {
        let mut data = input(vec![
            sample(0, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        data.samples[1].timestamp_utc = stamp(0);
        assert!(validate(&data).unwrap_err().contains("strictly increasing"));
    }

    #[test]
    fn rejects_time_gap_above_declared_maximum() {
        let mut data = input(vec![
            sample(0, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        data.maximum_allowed_interval_seconds = 1800;
        assert!(validate(&data).unwrap_err().contains("exceeds maximum_allowed_interval_seconds"));
    }

    #[test]
    fn rejects_negative_area_and_nonfinite_flux() {
        let data = input(vec![
            sample(0, -1.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        assert!(validate(&data).unwrap_err().contains("non-negative"));

        let data = input(vec![
            sample(0, 10.0, f64::INFINITY, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
            sample(3600, 10.0, 1.0e-6, EvidenceStatus::CtMeasured, EvidenceStatus::Modelled),
        ]);
        assert!(validate(&data).unwrap_err().contains("non-negative"));
    }
}
