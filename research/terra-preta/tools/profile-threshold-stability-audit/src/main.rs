use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, fs::OpenOptions, io::{self, Write}};

const AUDIT_VERSION: &str = "0.1.0";
const MIN_THRESHOLD: f64 = 0.000_001;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    source_evidence_ref: String,
    profile_reports: Vec<ProfileReport>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProfileReport {
    audit_version: String,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_threshold_m3_m3: f64,
    response_onset_method: String,
    sensor_count: usize,
    indeterminate_sensor_count: usize,
    profile_coverage_status: String,
    profile_classification: String,
    pair_results: Vec<PairResult>,
}

#[derive(Clone, Debug, Deserialize)]
struct PairResult {
    shallow_sensor_id: String,
    shallow_depth_cm: f64,
    deep_sensor_id: String,
    deep_depth_cm: f64,
    ordering: String,
}

#[derive(Clone, Debug, Serialize)]
struct ThresholdSummary {
    response_threshold_m3_m3: f64,
    profile_classification: String,
    profile_coverage_status: String,
    indeterminate_sensor_count: usize,
    nsr_pair_count: usize,
    sequential_pair_count: usize,
    ambiguous_pair_count: usize,
    tied_pair_count: usize,
    comparable_pair_count: usize,
}

#[derive(Clone, Debug, Serialize)]
struct PairThresholdResult {
    response_threshold_m3_m3: f64,
    ordering: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
struct PairStability {
    shallow_sensor_id: String,
    shallow_depth_cm: f64,
    deep_sensor_id: String,
    deep_depth_cm: f64,
    threshold_count: usize,
    nsr_threshold_count: usize,
    sequential_threshold_count: usize,
    ambiguous_threshold_count: usize,
    tied_threshold_count: usize,
    absent_pair_threshold_count: usize,
    persistence_status: &'static str,
    threshold_results: Vec<PairThresholdResult>,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    common_source_sha256: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    event_start_utc: String,
    observation_window_end_utc: String,
    time_resolution_seconds: u32,
    response_onset_method: String,
    minimum_response_threshold_m3_m3: f64,
    maximum_response_threshold_m3_m3: f64,
    threshold_count: usize,
    nsr_detected_threshold_count: usize,
    nsr_absent_threshold_count: usize,
    incomplete_profile_threshold_count: usize,
    profile_nsr_persistence_status: &'static str,
    persistent_nsr_pair_count: usize,
    threshold_sensitive_pair_count: usize,
    non_evaluable_pair_count: usize,
    threshold_summaries: Vec<ThresholdSummary>,
    pair_stability: Vec<PairStability>,
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

fn is_nsr(ordering: &str) -> bool {
    ordering == "nonsequential_deeper_first_interval_separated"
}

fn is_sequential(ordering: &str) -> bool {
    ordering == "sequential_shallower_first_interval_separated"
}

fn is_ambiguous(ordering: &str) -> bool {
    ordering == "ambiguous_overlapping_response_intervals"
}

fn is_tied(ordering: &str) -> bool {
    ordering == "simultaneous_exact_onset"
}

fn validate(input: &Input) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    nonblank(&input.source_evidence_ref, "source_evidence_ref")?;
    if input.profile_reports.len() < 2 {
        return Err("at least two profile reports from distinct thresholds are required".to_owned());
    }

    let first = &input.profile_reports[0];
    let mut previous_threshold = 0.0_f64;
    let mut seen_report_digests = std::collections::HashSet::new();

    for (index, report) in input.profile_reports.iter().enumerate() {
        if report.audit_version != "0.3.0" {
            return Err(format!(
                "profile_reports[{index}] has audit_version {:?}; expected 0.3.0",
                report.audit_version
            ));
        }
        valid_sha256(&report.input_sha256, &format!("profile_reports[{index}].input_sha256"))?;
        valid_sha256(&report.source_sha256, &format!("profile_reports[{index}].source_sha256"))?;
        if !seen_report_digests.insert(report.input_sha256.as_str()) {
            return Err(format!("duplicate profile report input digest at index {index}"));
        }
        for (value, label) in [
            (&report.study_id, "study_id"),
            (&report.site_id, "site_id"),
            (&report.plot_id, "plot_id"),
            (&report.profile_id, "profile_id"),
            (&report.event_id, "event_id"),
            (&report.event_start_utc, "event_start_utc"),
            (&report.observation_window_end_utc, "observation_window_end_utc"),
            (&report.response_onset_method, "response_onset_method"),
            (&report.profile_coverage_status, "profile_coverage_status"),
            (&report.profile_classification, "profile_classification"),
        ] {
            nonblank(value, &format!("profile_reports[{index}].{label}"))?;
        }
        if report.time_resolution_seconds < 300 || report.time_resolution_seconds > 1800 {
            return Err(format!(
                "profile_reports[{index}].time_resolution_seconds must be between 300 and 1800"
            ));
        }
        let threshold = report.response_threshold_m3_m3;
        if !threshold.is_finite() || threshold < MIN_THRESHOLD || threshold > 1.0 {
            return Err(format!(
                "profile_reports[{index}].response_threshold_m3_m3 must be finite and in [{MIN_THRESHOLD}, 1]"
            ));
        }
        if threshold <= previous_threshold {
            return Err("profile reports must be supplied in strictly increasing response-threshold order".to_owned());
        }
        previous_threshold = threshold;

        if report.source_sha256 != first.source_sha256
            || report.study_id != first.study_id
            || report.site_id != first.site_id
            || report.plot_id != first.plot_id
            || report.profile_id != first.profile_id
            || report.event_id != first.event_id
            || report.event_start_utc != first.event_start_utc
            || report.observation_window_end_utc != first.observation_window_end_utc
            || report.time_resolution_seconds != first.time_resolution_seconds
            || report.response_onset_method != first.response_onset_method
            || report.sensor_count != first.sensor_count
        {
            return Err(format!(
                "profile_reports[{index}] does not match the shared source, study/site/plot/profile/event, time window, cadence, onset method and declared sensor roster"
            ));
        }
        if report.indeterminate_sensor_count > report.sensor_count {
            return Err(format!(
                "profile_reports[{index}] indeterminate_sensor_count exceeds sensor_count"
            ));
        }

        let mut pair_ids = std::collections::HashSet::new();
        for (pair_index, pair) in report.pair_results.iter().enumerate() {
            nonblank(
                &pair.shallow_sensor_id,
                &format!("profile_reports[{index}].pair_results[{pair_index}].shallow_sensor_id"),
            )?;
            nonblank(
                &pair.deep_sensor_id,
                &format!("profile_reports[{index}].pair_results[{pair_index}].deep_sensor_id"),
            )?;
            if pair.shallow_sensor_id == pair.deep_sensor_id {
                return Err(format!(
                    "profile_reports[{index}].pair_results[{pair_index}] uses the same sensor twice"
                ));
            }
            if !pair.shallow_depth_cm.is_finite()
                || !pair.deep_depth_cm.is_finite()
                || pair.shallow_depth_cm <= 0.0
                || pair.deep_depth_cm <= pair.shallow_depth_cm
            {
                return Err(format!(
                    "profile_reports[{index}].pair_results[{pair_index}] must identify shallow and deeper positive depths"
                ));
            }
            let key = (pair.shallow_sensor_id.as_str(), pair.deep_sensor_id.as_str());
            if !pair_ids.insert(key) {
                return Err(format!(
                    "profile_reports[{index}] contains duplicate sensor pair {:?}",
                    key
                ));
            }
            if !matches!(
                pair.ordering.as_str(),
                "nonsequential_deeper_first_interval_separated"
                    | "sequential_shallower_first_interval_separated"
                    | "ambiguous_overlapping_response_intervals"
                    | "simultaneous_exact_onset"
            ) {
                return Err(format!(
                    "profile_reports[{index}].pair_results[{pair_index}] has unsupported ordering {:?}",
                    pair.ordering
                ));
            }
        }
    }

    if first.source_sha256 != first.source_sha256.to_ascii_lowercase() {
        return Err("source_sha256 must use lowercase hexadecimal notation".to_owned());
    }
    Ok(())
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    validate(input)?;
    let first = &input.profile_reports[0];
    let threshold_count = input.profile_reports.len();

    let mut threshold_summaries = Vec::with_capacity(threshold_count);
    let mut nsr_detected_threshold_count = 0usize;
    let mut nsr_absent_threshold_count = 0usize;
    let mut incomplete_profile_threshold_count = 0usize;

    for report in &input.profile_reports {
        let nsr_count = report.pair_results.iter().filter(|pair| is_nsr(&pair.ordering)).count();
        let sequential_count = report.pair_results.iter().filter(|pair| is_sequential(&pair.ordering)).count();
        let ambiguous_count = report.pair_results.iter().filter(|pair| is_ambiguous(&pair.ordering)).count();
        let tied_count = report.pair_results.iter().filter(|pair| is_tied(&pair.ordering)).count();
        if nsr_count > 0 {
            nsr_detected_threshold_count += 1;
        } else {
            nsr_absent_threshold_count += 1;
        }
        if report.indeterminate_sensor_count > 0 {
            incomplete_profile_threshold_count += 1;
        }
        threshold_summaries.push(ThresholdSummary {
            response_threshold_m3_m3: report.response_threshold_m3_m3,
            profile_classification: report.profile_classification.clone(),
            profile_coverage_status: report.profile_coverage_status.clone(),
            indeterminate_sensor_count: report.indeterminate_sensor_count,
            nsr_pair_count: nsr_count,
            sequential_pair_count: sequential_count,
            ambiguous_pair_count: ambiguous_count,
            tied_pair_count: tied_count,
            comparable_pair_count: report.pair_results.len(),
        });
    }

    let mut pair_keys = std::collections::BTreeMap::<(String, String), (f64, f64)>::new();
    for report in &input.profile_reports {
        for pair in &report.pair_results {
            let key = (pair.shallow_sensor_id.clone(), pair.deep_sensor_id.clone());
            let value = (pair.shallow_depth_cm, pair.deep_depth_cm);
            if let Some(existing) = pair_keys.get(&key) {
                if existing.0.total_cmp(&value.0).is_ne()
                    || existing.1.total_cmp(&value.1).is_ne()
                {
                    return Err(format!(
                        "sensor pair {:?} has inconsistent depths across thresholds",
                        key
                    ));
                }
            } else {
                pair_keys.insert(key, value);
            }
        }
    }

    let mut pair_stability = Vec::with_capacity(pair_keys.len());
    for ((shallow_id, deep_id), (shallow_depth, deep_depth)) in pair_keys {
        let mut nsr_count = 0usize;
        let mut sequential_count = 0usize;
        let mut ambiguous_count = 0usize;
        let mut tied_count = 0usize;
        let mut absent_count = 0usize;
        let mut threshold_results = Vec::with_capacity(threshold_count);
        for report in &input.profile_reports {
            let found = report.pair_results.iter().find(|pair| {
                pair.shallow_sensor_id == shallow_id && pair.deep_sensor_id == deep_id
            });
            let ordering = found.map(|pair| pair.ordering.clone());
            match ordering.as_deref() {
                Some(value) if is_nsr(value) => nsr_count += 1,
                Some(value) if is_sequential(value) => sequential_count += 1,
                Some(value) if is_ambiguous(value) => ambiguous_count += 1,
                Some(value) if is_tied(value) => tied_count += 1,
                None => absent_count += 1,
                Some(_) => unreachable!("validation restricts ordering values"),
            }
            threshold_results.push(PairThresholdResult {
                response_threshold_m3_m3: report.response_threshold_m3_m3,
                ordering,
            });
        }

        let persistence_status = if nsr_count == threshold_count {
            "persistent_nsr_all_thresholds"
        } else if nsr_count > 0 && absent_count == 0 {
            "threshold_sensitive_ordering"
        } else if nsr_count > 0 {
            "partially_evaluable_nsr"
        } else if absent_count > 0 {
            "not_evaluable_across_all_thresholds"
        } else if sequential_count == threshold_count {
            "sequential_all_thresholds"
        } else if tied_count == threshold_count {
            "simultaneous_all_thresholds"
        } else if ambiguous_count == threshold_count {
            "ambiguous_all_thresholds"
        } else {
            "mixed_non_nsr_ordering"
        };

        pair_stability.push(PairStability {
            shallow_sensor_id: shallow_id,
            shallow_depth_cm: shallow_depth,
            deep_sensor_id: deep_id,
            deep_depth_cm: deep_depth,
            threshold_count,
            nsr_threshold_count: nsr_count,
            sequential_threshold_count: sequential_count,
            ambiguous_threshold_count: ambiguous_count,
            tied_threshold_count: tied_count,
            absent_pair_threshold_count: absent_count,
            persistence_status,
            threshold_results,
        });
    }

    let persistent_nsr_pair_count = pair_stability.iter()
        .filter(|pair| pair.persistence_status == "persistent_nsr_all_thresholds")
        .count();
    let threshold_sensitive_pair_count = pair_stability.iter()
        .filter(|pair| matches!(pair.persistence_status, "threshold_sensitive_ordering" | "mixed_non_nsr_ordering"))
        .count();
    let non_evaluable_pair_count = pair_stability.iter()
        .filter(|pair| pair.persistence_status == "not_evaluable_across_all_thresholds" || pair.persistence_status == "partially_evaluable_nsr")
        .count();

    let profile_nsr_persistence_status = if persistent_nsr_pair_count > 0 && incomplete_profile_threshold_count > 0 {
        "nsr_pair_persistent_with_incomplete_profile_coverage"
    } else if persistent_nsr_pair_count > 0 {
        "nsr_pair_persistent_across_all_thresholds"
    } else if nsr_detected_threshold_count > 0 && nsr_absent_threshold_count > 0 {
        "profile_nsr_threshold_sensitive"
    } else if nsr_detected_threshold_count == threshold_count {
        "nsr_present_at_all_thresholds_but_no_pair_persistent"
    } else if incomplete_profile_threshold_count > 0 {
        "no_nsr_detected_with_incomplete_profile_evidence"
    } else {
        "no_nsr_signature_detected_across_declared_thresholds"
    };

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        common_source_sha256: first.source_sha256.clone(),
        study_id: first.study_id.clone(),
        site_id: first.site_id.clone(),
        plot_id: first.plot_id.clone(),
        profile_id: first.profile_id.clone(),
        event_id: first.event_id.clone(),
        event_start_utc: first.event_start_utc.clone(),
        observation_window_end_utc: first.observation_window_end_utc.clone(),
        time_resolution_seconds: first.time_resolution_seconds,
        response_onset_method: first.response_onset_method.clone(),
        minimum_response_threshold_m3_m3: first.response_threshold_m3_m3,
        maximum_response_threshold_m3_m3: input.profile_reports.last().unwrap().response_threshold_m3_m3,
        threshold_count,
        nsr_detected_threshold_count,
        nsr_absent_threshold_count,
        incomplete_profile_threshold_count,
        profile_nsr_persistence_status,
        persistent_nsr_pair_count,
        threshold_sensitive_pair_count,
        non_evaluable_pair_count,
        threshold_summaries,
        pair_stability,
        interpretation_boundary: "NSR persistence describes only the supplied response-threshold family and same declared sensor profile. A persistent sensor-response ordering is not tracer-confirmed preferential flow; lack of NSR does not rule out preferential flow.",
        warnings: vec![
            "The input profile reports must be independently produced from the exact same normalized source series with only response_threshold_m3_m3 changed.".to_owned(),
            "Pairs absent at a threshold are not counted as negative or sequential; they are not evaluable at that threshold.".to_owned(),
            "Threshold persistence does not remove common sensor calibration error, baseline bias, spatial aliasing or other shared measurement failure.".to_owned(),
            "NSR is a response-order signature and does not uniquely identify vertical macropore flow; lateral routing and measurement artefacts remain possible.".to_owned(),
            "The tool compares profile-flow audit artifacts; it does not reopen or verify each artifact file. input_sha256 values establish declared identity only.".to_owned(),
        ],
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: profile-threshold-stability-audit <threshold-reports.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let mut encoded = serde_json::to_vec_pretty(&report)?;
    encoded.push(b'\n');
    let mut output = OpenOptions::new().write(true).create_new(true).open(&args[2])
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report {}: {error}", args[2])))?;
    output.write_all(&encoded)?;
    output.sync_all()?;
    println!(
        "profile threshold stability audit complete: {} thresholds; {} persistent NSR pairs; {} threshold-sensitive pairs; input SHA-256 {}",
        report.threshold_count, report.persistent_nsr_pair_count,
        report.threshold_sensitive_pair_count, report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("profile threshold stability audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(shallow: &str, deep: &str, ordering: &str) -> PairResult {
        PairResult {
            shallow_sensor_id: shallow.to_owned(),
            shallow_depth_cm: if shallow == "D10" { 10.0 } else { 30.0 },
            deep_sensor_id: deep.to_owned(),
            deep_depth_cm: if deep == "D90" { 90.0 } else if deep == "D60" { 60.0 } else { 30.0 },
            ordering: ordering.to_owned(),
        }
    }

    fn profile(threshold: f64, digest: &str, pairs: Vec<PairResult>) -> ProfileReport {
        ProfileReport {
            audit_version: "0.3.0".to_owned(),
            input_sha256: digest.to_owned(),
            source_sha256: "a".repeat(64),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            event_id: "E001".to_owned(),
            event_start_utc: "2026-01-01T00:00:00Z".to_owned(),
            observation_window_end_utc: "2026-01-01T03:00:00Z".to_owned(),
            time_resolution_seconds: 600,
            response_threshold_m3_m3: threshold,
            response_onset_method: "first_observed_crossing".to_owned(),
            sensor_count: 4,
            indeterminate_sensor_count: 0,
            profile_coverage_status: "complete_for_declared_sensor_roster".to_owned(),
            profile_classification: "mixed_response_ordering".to_owned(),
            pair_results: pairs,
        }
    }

    fn fixture_input() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            source_evidence_ref: "synthetic test only".to_owned(),
            profile_reports: vec![
                profile(0.01, &"b".repeat(64), vec![
                    pair("D10", "D30", "nonsequential_deeper_first_interval_separated"),
                    pair("D10", "D60", "sequential_shallower_first_interval_separated"),
                    pair("D30", "D90", "nonsequential_deeper_first_interval_separated"),
                ]),
                profile(0.02, &"c".repeat(64), vec![
                    pair("D10", "D30", "nonsequential_deeper_first_interval_separated"),
                    pair("D10", "D60", "ambiguous_overlapping_response_intervals"),
                    pair("D30", "D90", "nonsequential_deeper_first_interval_separated"),
                ]),
                profile(0.03, &"d".repeat(64), vec![
                    pair("D10", "D30", "ambiguous_overlapping_response_intervals"),
                    pair("D10", "D60", "sequential_shallower_first_interval_separated"),
                ]),
            ],
        }
    }

    #[test]
    fn recognizes_nsr_pair_that_persists_at_all_thresholds() {
        let report = evaluate(&fixture_input(), "e".repeat(64)).unwrap();
        assert_eq!(report.profile_nsr_persistence_status, "nsr_pair_persistent_across_all_thresholds");
        assert_eq!(report.persistent_nsr_pair_count, 1);
        let persistent = report.pair_stability.iter()
            .find(|pair| pair.shallow_sensor_id == "D30" && pair.deep_sensor_id == "D90")
            .unwrap();
        assert_eq!(persistent.persistence_status, "persistent_nsr_all_thresholds");
    }

    #[test]
    fn reports_threshold_sensitive_pair_without_forcing_majority_vote() {
        let report = evaluate(&fixture_input(), "f".repeat(64)).unwrap();
        let pair = report.pair_stability.iter()
            .find(|pair| pair.shallow_sensor_id == "D10" && pair.deep_sensor_id == "D30")
            .unwrap();
        assert_eq!(pair.persistence_status, "threshold_sensitive_ordering");
        assert_eq!(pair.nsr_threshold_count, 2);
        assert_eq!(pair.sequential_threshold_count, 0);
        assert_eq!(pair.ambiguous_threshold_count, 1);
    }

    #[test]
    fn absent_pair_is_not_treated_as_a_negative_result() {
        let report = evaluate(&fixture_input(), "1".repeat(64)).unwrap();
        let pair = report.pair_stability.iter()
            .find(|pair| pair.shallow_sensor_id == "D10" && pair.deep_sensor_id == "D60")
            .unwrap();
        assert_eq!(pair.persistence_status, "not_evaluable_across_all_thresholds");
        assert_eq!(pair.sequential_threshold_count, 1);
        assert_eq!(pair.ambiguous_threshold_count, 1);
        assert_eq!(pair.absent_pair_threshold_count, 1);
    }

    #[test]
    fn rejects_reports_from_different_raw_sources() {
        let mut input = fixture_input();
        input.profile_reports[1].source_sha256 = "f".repeat(64);
        assert!(validate(&input).unwrap_err().contains("shared source"));
    }

    #[test]
    fn rejects_reports_in_nonincreasing_threshold_order() {
        let mut input = fixture_input();
        input.profile_reports[1].response_threshold_m3_m3 = 0.01;
        assert!(validate(&input).unwrap_err().contains("strictly increasing"));
    }

    #[test]
    fn rejects_mismatched_sensor_roster() {
        let mut input = fixture_input();
        input.profile_reports[2].sensor_count = 3;
        assert!(validate(&input).unwrap_err().contains("declared sensor roster"));
    }

    #[test]
    fn rejects_invalid_or_duplicate_sensor_pairs() {
        let mut input = fixture_input();
        input.profile_reports[1].pair_results.push(input.profile_reports[1].pair_results[0].clone());
        assert!(validate(&input).unwrap_err().contains("duplicate sensor pair"));
    }

    #[test]
    fn requires_more_than_one_threshold() {
        let mut input = fixture_input();
        input.profile_reports.pop();
        assert!(validate(&input).unwrap_err().contains("at least two"));
    }
}
