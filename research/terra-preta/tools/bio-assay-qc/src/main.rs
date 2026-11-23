//! Synthetic-only, method-specific potential enzyme activity QC auditor.
//!
//! The auditor checks assay provenance, calibration, blanks, recovery and
//! quench handling. It never computes actual in-situ process rates or a soil
//! biological health score.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic potential enzyme-activity assay QC only. The reported value is not recalculated, and no in-situ process rate, causal treatment effect, microbial health or ecosystem benefit is inferred.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    acceptance_profile: AcceptanceProfile,
    assays: Vec<Assay>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AcceptanceProfile {
    profile_id: String,
    method_id: String,
    method_version: String,
    method_profile_sha256: String,
    status: String,
    minimum_calibration_r_squared: f64,
    spike_recovery_min_percent: f64,
    spike_recovery_max_percent: f64,
    matrix_recovery_min_percent: f64,
    matrix_recovery_max_percent: f64,
    maximum_uncorrected_quench_percent: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Assay {
    assay_id: String,
    sample_id: String,
    experimental_unit_id: String,
    independence_group_id: String,
    evidence_class: String,
    analyte: String,
    measurement_method: String,
    method_id: String,
    method_version: String,
    method_profile_sha256: String,
    unit: String,
    normalization_basis: String,
    reported_rate: f64,
    reported_rate_standard_error: f64,
    substrate: String,
    substrate_concentration: f64,
    assay_ph: f64,
    assay_temperature_c: f64,
    incubation_minutes: f64,
    calibration: Calibration,
    blanks: Blanks,
    matrix_controls: MatrixControls,
    saturation_detected: bool,
    sample_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Calibration {
    curve_id: String,
    slope: f64,
    intercept: f64,
    r_squared: f64,
    signal_min: f64,
    signal_max: f64,
    sample_signal: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Blanks {
    substrate_only_blank_signal: f64,
    matrix_blank_signal: f64,
    sample_blank_signal: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MatrixControls {
    spike_recovery_percent: f64,
    matrix_spike_recovery_percent: f64,
    optical_quench_percent: f64,
    quench_correction_applied: bool,
    correction_method_id: Option<String>,
    correction_method_sha256: Option<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    method_profile_id: String,
    method_profile_sha256: String,
    assay_count: usize,
    assays: Vec<AssayResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct AssayResult {
    assay_id: String,
    sample_id: String,
    experimental_unit_id: String,
    analyte: String,
    evidence_class: &'static str,
    reported_rate: f64,
    unit: String,
    normalization_basis: String,
    qualification_status: &'static str,
    failure_reasons: Vec<String>,
    process_rate_claim: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    no_composition_to_function_promotion: bool,
    no_potential_to_process_rate_promotion: bool,
    no_aggregate_biological_score: bool,
    method_profile_digest_binding: bool,
    no_output_overwrite: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "bio-assay-qc-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema/version".into());
    }
    let p = &s.acceptance_profile;
    if s.scenario_id.trim().is_empty()
        || p.profile_id.trim().is_empty()
        || p.method_id.trim().is_empty()
        || p.method_version.trim().is_empty()
        || p.status != "synthetic_fixture"
        || !valid_sha256(&p.method_profile_sha256)
    {
        return Err("scenario or synthetic method-profile identity is invalid".into());
    }
    for (name, value) in [
        (
            "minimum_calibration_r_squared",
            p.minimum_calibration_r_squared,
        ),
        ("spike_recovery_min_percent", p.spike_recovery_min_percent),
        ("spike_recovery_max_percent", p.spike_recovery_max_percent),
        ("matrix_recovery_min_percent", p.matrix_recovery_min_percent),
        ("matrix_recovery_max_percent", p.matrix_recovery_max_percent),
        (
            "maximum_uncorrected_quench_percent",
            p.maximum_uncorrected_quench_percent,
        ),
    ] {
        finite(name, value)?;
    }
    if !(0.0..=1.0).contains(&p.minimum_calibration_r_squared)
        || p.spike_recovery_min_percent < 0.0
        || p.spike_recovery_min_percent > p.spike_recovery_max_percent
        || p.matrix_recovery_min_percent < 0.0
        || p.matrix_recovery_min_percent > p.matrix_recovery_max_percent
        || !(0.0..=100.0).contains(&p.maximum_uncorrected_quench_percent)
    {
        return Err("method-specific acceptance limits are invalid".into());
    }
    if s.assays.is_empty() {
        return Err("at least one assay is required".into());
    }
    let mut ids = BTreeSet::new();
    let mut samples = BTreeSet::new();
    for a in &s.assays {
        if a.assay_id.trim().is_empty()
            || a.sample_id.trim().is_empty()
            || a.experimental_unit_id.trim().is_empty()
            || a.independence_group_id.trim().is_empty()
            || a.analyte.trim().is_empty()
            || a.measurement_method.trim().is_empty()
            || a.unit.trim().is_empty()
            || a.substrate.trim().is_empty()
            || a.calibration.curve_id.trim().is_empty()
            || a.evidence_class != "potential_activity"
            || a.normalization_basis != "dry_soil_mass"
            || a.method_id != p.method_id
            || a.method_version != p.method_version
            || a.method_profile_sha256 != p.method_profile_sha256
            || !ids.insert(a.assay_id.as_str())
            || !samples.insert(a.sample_id.as_str())
            || !valid_sha256(&a.sample_sha256)
        {
            return Err(format!(
                "assay {} has invalid/duplicate identity, evidence class, method binding or normalization",
                a.assay_id
            ));
        }
        for (name, value) in [
            ("reported_rate", a.reported_rate),
            (
                "reported_rate_standard_error",
                a.reported_rate_standard_error,
            ),
            ("substrate_concentration", a.substrate_concentration),
            ("assay_ph", a.assay_ph),
            ("assay_temperature_c", a.assay_temperature_c),
            ("incubation_minutes", a.incubation_minutes),
            ("calibration_slope", a.calibration.slope),
            ("calibration_intercept", a.calibration.intercept),
            ("calibration_r_squared", a.calibration.r_squared),
            ("signal_min", a.calibration.signal_min),
            ("signal_max", a.calibration.signal_max),
            ("sample_signal", a.calibration.sample_signal),
            (
                "substrate_only_blank_signal",
                a.blanks.substrate_only_blank_signal,
            ),
            ("matrix_blank_signal", a.blanks.matrix_blank_signal),
            ("sample_blank_signal", a.blanks.sample_blank_signal),
            (
                "spike_recovery_percent",
                a.matrix_controls.spike_recovery_percent,
            ),
            (
                "matrix_spike_recovery_percent",
                a.matrix_controls.matrix_spike_recovery_percent,
            ),
            (
                "optical_quench_percent",
                a.matrix_controls.optical_quench_percent,
            ),
        ] {
            finite(name, value)?;
        }
        if a.reported_rate_standard_error < 0.0
            || a.substrate_concentration <= 0.0
            || !(0.0..=14.0).contains(&a.assay_ph)
            || a.incubation_minutes <= 0.0
            || a.calibration.slope <= 0.0
            || !(0.0..=1.0).contains(&a.calibration.r_squared)
            || a.calibration.signal_min >= a.calibration.signal_max
            || a.matrix_controls.spike_recovery_percent < 0.0
            || a.matrix_controls.matrix_spike_recovery_percent < 0.0
            || !(0.0..=100.0).contains(&a.matrix_controls.optical_quench_percent)
        {
            return Err(format!(
                "assay {} has physically invalid numeric metadata",
                a.assay_id
            ));
        }
        if a.matrix_controls.quench_correction_applied {
            if a.matrix_controls
                .correction_method_id
                .as_deref()
                .unwrap_or("")
                .trim()
                .is_empty()
                || !a
                    .matrix_controls
                    .correction_method_sha256
                    .as_deref()
                    .is_some_and(valid_sha256)
            {
                return Err(format!(
                    "assay {} reports quench correction without method identity/digest",
                    a.assay_id
                ));
            }
        } else if a.matrix_controls.correction_method_id.is_some()
            || a.matrix_controls.correction_method_sha256.is_some()
        {
            return Err(format!(
                "assay {} carries correction identity but correction is not marked applied",
                a.assay_id
            ));
        }
    }
    Ok(())
}

fn qualify(a: &Assay, p: &AcceptanceProfile) -> AssayResult {
    let mut failures = Vec::new();
    if a.calibration.r_squared < p.minimum_calibration_r_squared {
        failures.push("calibration_r_squared_below_method_profile".into());
    }
    if a.calibration.sample_signal < a.calibration.signal_min
        || a.calibration.sample_signal > a.calibration.signal_max
    {
        failures.push("sample_signal_outside_calibration_range".into());
    }
    if a.matrix_controls.spike_recovery_percent < p.spike_recovery_min_percent
        || a.matrix_controls.spike_recovery_percent > p.spike_recovery_max_percent
    {
        failures.push("spike_recovery_outside_method_profile".into());
    }
    if a.matrix_controls.matrix_spike_recovery_percent < p.matrix_recovery_min_percent
        || a.matrix_controls.matrix_spike_recovery_percent > p.matrix_recovery_max_percent
    {
        failures.push("matrix_spike_recovery_outside_method_profile".into());
    }
    if a.matrix_controls.optical_quench_percent > p.maximum_uncorrected_quench_percent {
        failures.push("optical_quench_exceeds_v0_1_support_even_with_declared_correction".into());
    }
    if a.saturation_detected {
        failures.push("assay_signal_saturation_detected".into());
    }
    AssayResult {
        assay_id: a.assay_id.clone(),
        sample_id: a.sample_id.clone(),
        experimental_unit_id: a.experimental_unit_id.clone(),
        analyte: a.analyte.clone(),
        evidence_class: "potential_activity",
        reported_rate: a.reported_rate,
        unit: a.unit.clone(),
        normalization_basis: a.normalization_basis.clone(),
        qualification_status: if failures.is_empty() {
            "qualified_potential_activity"
        } else {
            "unqualified_potential_activity"
        },
        failure_reasons: failures,
        process_rate_claim: "not_estimated",
    }
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let results: Vec<AssayResult> = s
        .assays
        .iter()
        .map(|a| qualify(a, &s.acceptance_profile))
        .collect();
    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        method_profile_id: s.acceptance_profile.profile_id.clone(),
        method_profile_sha256: s.acceptance_profile.method_profile_sha256.clone(),
        assay_count: results.len(),
        assays: results,
        controls: Controls {
            no_composition_to_function_promotion: true,
            no_potential_to_process_rate_promotion: true,
            no_aggregate_biological_score: true,
            method_profile_digest_binding: true,
            no_output_overwrite: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output already exists").into());
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let bytes = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&bytes)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: bio-assay-qc <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("bio-assay QC failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-bio-assay-qc.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn method_profile_qualifies_one_and_rejects_failed_spike_recovery() {
        let s = scenario();
        validate(&s).expect("valid synthetic assay records");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.assay_count, 2);
        assert_eq!(
            r.assays[0].qualification_status,
            "qualified_potential_activity"
        );
        assert_eq!(
            r.assays[1].qualification_status,
            "unqualified_potential_activity"
        );
        assert!(
            r.assays[1]
                .failure_reasons
                .contains(&"spike_recovery_outside_method_profile".into())
        );
    }

    #[test]
    fn uncorrected_optical_quench_blocks_assay() {
        let mut s = scenario();
        s.assays[0].matrix_controls.optical_quench_percent = 40.0;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(
            r.assays[0].qualification_status,
            "unqualified_potential_activity"
        );
        assert!(
            r.assays[0].failure_reasons.contains(
                &"optical_quench_exceeds_v0_1_support_even_with_declared_correction".into()
            )
        );
    }

    #[test]
    fn declared_quench_correction_is_not_executed_or_qualified_in_v0_1() {
        let mut s = scenario();
        s.assays[0].matrix_controls.optical_quench_percent = 40.0;
        s.assays[0].matrix_controls.quench_correction_applied = true;
        assert!(validate(&s).is_err());

        s.assays[0].matrix_controls.correction_method_id =
            Some("synthetic-quench-correction".into());
        s.assays[0].matrix_controls.correction_method_sha256 = Some("c".repeat(64));
        validate(&s).expect("correction method identity is bound");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(
            r.assays[0].qualification_status,
            "unqualified_potential_activity"
        );
        assert!(
            r.assays[0].failure_reasons.contains(
                &"optical_quench_exceeds_v0_1_support_even_with_declared_correction".into()
            )
        );
    }

    #[test]
    fn calibration_range_and_r_squared_are_enforced() {
        let mut s = scenario();
        s.assays[0].calibration.sample_signal = 900.0;
        s.assays[0].calibration.r_squared = 0.8;
        let r = build_report(&s, "0".repeat(64));
        assert!(
            r.assays[0]
                .failure_reasons
                .contains(&"sample_signal_outside_calibration_range".into())
        );
        assert!(
            r.assays[0]
                .failure_reasons
                .contains(&"calibration_r_squared_below_method_profile".into())
        );
    }

    #[test]
    fn saturation_never_qualifies() {
        let mut s = scenario();
        s.assays[0].saturation_detected = true;
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(
            r.assays[0].qualification_status,
            "unqualified_potential_activity"
        );
    }

    #[test]
    fn method_and_sample_digests_are_bound() {
        let mut s = scenario();
        s.assays[0].method_profile_sha256 = "d".repeat(64);
        assert!(validate(&s).is_err());
        let mut t = scenario();
        t.assays[0].sample_sha256 = "invalid".into();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn potential_activity_never_becomes_a_process_rate_or_composite_score() {
        let r = build_report(&scenario(), "0".repeat(64));
        assert!(
            r.assays
                .iter()
                .all(|a| a.process_rate_claim == "not_estimated")
        );
        assert!(r.controls.no_aggregate_biological_score);
    }

    #[test]
    fn output_schema_unknown_fields_are_rejected_by_typed_input() {
        let mut s = scenario();
        s.assays[0].normalization_basis = "wet_soil_mass".into();
        assert!(validate(&s).is_err());
    }
}
