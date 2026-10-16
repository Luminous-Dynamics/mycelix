//! Independent public-API regression tests driven by the frozen TSV corpus.
//!
//! The corpus encodes expected dispositions and signal-level outcomes separately
//! from the detector implementation. All inputs are synthetic and prove software
//! contract behavior only; they are not measured factory telemetry.

use fabrication_monitor_core::{
    BaselineError, ConfigError, DetectorConfig, InputError, MonitoringEngine,
    ObservationDisposition, ProcessProfile, ReferenceBaseline, SensorFrame, Signal,
};
use std::collections::BTreeSet;

const CORPUS: &str = include_str!("fixtures/cincinnati_v1.tsv");
const CORPUS_ID: &str = "CINCINNATI-MONITOR-SYNTHETIC-V1";

fn profile() -> ProcessProfile {
    ProcessProfile {
        design_revision: "design:bracket@sha256:test-a".into(),
        recipe_revision: "recipe:pla-0.2mm@sha256:test-b".into(),
        equipment_profile: "equipment:printer-7:config-r3".into(),
        material_profile: "material:PLA-grade-A:lot-2026-10-01".into(),
        environment_profile: "environment:enclosed-lab-profile-v1".into(),
        calibration_profile: "calibration:printer-7:2026-10-01".into(),
    }
}

fn config() -> DetectorConfig {
    DetectorConfig {
        detector_id: "cincinnati.robust-reference".into(),
        detector_version: "1.0.0".into(),
        robust_z_threshold: 3.5,
        minimum_scale_by_signal: [
            0.5, 0.5, 10.0, 10.0, 10.0, 10.0, 0.01, 2.0, 0.5, 2.0,
        ],
        minimum_baseline_samples: 5,
        maximum_sample_gap_ms: 1_000,
    }
}

fn frame(session_id: &str, sequence: u64, timestamp_ms: u64) -> SensorFrame {
    let i = sequence as f64;
    SensorFrame {
        session_id: session_id.into(),
        sequence,
        timestamp_ms,
        profile: profile(),
        hotend_temp_c: 200.0 + i * 0.1,
        bed_temp_c: 60.0 + i * 0.1,
        stepper_current_ma: [100.0 + i, 200.0 + i, 150.0 + i, 300.0 + 2.0 * i],
        vibration_rms_g: 0.10 + i * 0.001,
        filament_tension_g: Some(50.0 + i * 0.5),
        ambient_temp_c: Some(25.0 + i * 0.05),
        humidity_pct: Some(45.0 + i * 0.2),
    }
}

fn healthy_reference() -> Vec<SensorFrame> {
    (0..7).map(|i| frame("reference", i, i * 100)).collect()
}

fn parse_signal(value: &str) -> Signal {
    match value {
        "HotendTemperature" => Signal::HotendTemperature,
        "BedTemperature" => Signal::BedTemperature,
        "XCurrent" => Signal::XCurrent,
        "YCurrent" => Signal::YCurrent,
        "ZCurrent" => Signal::ZCurrent,
        "ExtruderCurrent" => Signal::ExtruderCurrent,
        "VibrationRms" => Signal::VibrationRms,
        "FilamentTension" => Signal::FilamentTension,
        "AmbientTemperature" => Signal::AmbientTemperature,
        "Humidity" => Signal::Humidity,
        other => panic!("unknown signal in frozen corpus: {other}"),
    }
}

fn parse_disposition(value: &str) -> ObservationDisposition {
    let parts: Vec<&str> = value.split(':').collect();
    match parts[0] {
        "Evaluated" => ObservationDisposition::Evaluated,
        "EvaluatedWithUncalibratedSignals" => {
            ObservationDisposition::EvaluatedWithUncalibratedSignals
        }
        "ProcessProfileMismatch" => ObservationDisposition::ProcessProfileMismatch,
        "SessionMismatch" => ObservationDisposition::SessionMismatch,
        "NonMonotonicSequence" => ObservationDisposition::NonMonotonicSequence,
        "NonMonotonicTimestamp" => ObservationDisposition::NonMonotonicTimestamp,
        "DataGap" => ObservationDisposition::DataGap {
            missing_sequences: parts[1].parse().expect("missing-sequence field"),
            elapsed_ms: parts[2].parse().expect("elapsed-ms field"),
        },
        "InvalidObservation" => {
            let reason = match (parts[1], parts[2]) {
                ("NonFinite", "HotendTemperature") => {
                    InputError::NonFinite(Signal::HotendTemperature)
                }
                ("Negative", "XCurrent") => InputError::Negative(Signal::XCurrent),
                ("OutOfRange", "Humidity") => InputError::OutOfRange(Signal::Humidity),
                other => panic!("unknown invalid-observation reason in corpus: {other:?}"),
            };
            ObservationDisposition::InvalidObservation(reason)
        }
        other => panic!("unknown disposition in frozen corpus: {other}"),
    }
}

fn run_scenario(scenario: &str) -> (ObservationDisposition, Vec<Signal>, Vec<Signal>) {
    let mut reference = healthy_reference();
    match scenario {
        "tension-unqualified" => {
            for sample in &mut reference {
                sample.filament_tension_g = None;
            }
        }
        "ambient-unqualified" => {
            for sample in &mut reference {
                sample.ambient_temp_c = None;
            }
        }
        "humidity-unqualified" => {
            for sample in &mut reference {
                sample.humidity_pct = None;
            }
        }
        _ => {}
    }

    let baseline = ReferenceBaseline::fit(profile(), config(), &reference)
        .expect("frozen healthy-reference fixture must fit");
    let mut engine = MonitoringEngine::new(baseline, "live")
        .expect("frozen session ID must be valid");

    let mut sample = frame("live", 1, 100);
    match scenario {
        "healthy" | "tension-unqualified" => {}
        "hotend-drift" => sample.hotend_temp_c = 230.0,
        "bed-drift" => sample.bed_temp_c = 90.0,
        "extruder-current-drift" => sample.stepper_current_ma[3] = 500.0,
        "vibration-drift" => sample.vibration_rms_g = 0.8,
        "ambient-drift" => sample.ambient_temp_c = Some(45.0),
        "humidity-drift" => sample.humidity_pct = Some(80.0),
        "recipe-mismatch" => sample.profile.recipe_revision = "recipe:changed".into(),
        "calibration-mismatch" => {
            sample.profile.calibration_profile = "calibration:new".into();
        }
        "equipment-mismatch" => {
            sample.profile.equipment_profile = "equipment:printer-8:config-r1".into();
        }
        "material-mismatch" => {
            sample.profile.material_profile = "material:PLA-grade-B:lot-2026-10-02".into();
        }
        "environment-mismatch" => {
            sample.profile.environment_profile = "environment:open-shop-profile-v1".into();
        }
        "nonfinite-hotend" => sample.hotend_temp_c = f64::NAN,
        "negative-current" => sample.stepper_current_ma[0] = -1.0,
        "humidity-out-of-range" => sample.humidity_pct = Some(101.0),
        "session-mismatch" => sample.session_id = "other-session".into(),
        "sequence-gap" => {
            let _ = engine.observe(&sample);
            sample.sequence = 3;
            sample.timestamp_ms = 300;
        }
        "duplicate-sequence" => {
            let _ = engine.observe(&sample);
            sample.timestamp_ms = 200;
        }
        "oversized-time-gap" => {
            let _ = engine.observe(&sample);
            sample.sequence = 2;
            sample.timestamp_ms = 2_600;
        }
        "timestamp-reordered" => {
            let _ = engine.observe(&sample);
            sample.sequence = 2;
            sample.timestamp_ms = 99;
        }
        other => panic!("unknown scenario in frozen corpus: {other}"),
    }

    let report = engine.observe(&sample);
    let uncalibrated_signals = report.uncalibrated_signals.clone();
    let deviation_signals = report.deviations.iter().map(|d| d.signal).collect();
    (report.disposition, deviation_signals, uncalibrated_signals)
}

#[test]
fn frozen_cincinnati_v1_corpus_matches_expected_dispositions_and_signals() {
    let first_line = CORPUS.lines().next().expect("corpus metadata line");
    assert!(first_line.contains(CORPUS_ID), "corpus identity must remain explicit");

    let mut ids = BTreeSet::new();
    for line in CORPUS.lines().filter(|line| !line.trim().is_empty() && !line.starts_with('#')) {
        let fields: Vec<&str> = line.split('|').collect();
        assert_eq!(fields.len(), 5, "invalid TSV row: {line}");
        let case_id = fields[0];
        let scenario = fields[1];
        let expected_disposition = parse_disposition(fields[2]);
        let expected_signal = if fields[3].is_empty() {
            None
        } else {
            Some(parse_signal(fields[3]))
        };
        let expected_uncalibrated = if fields[4].is_empty() {
            None
        } else {
            Some(parse_signal(fields[4]))
        };

        assert!(!case_id.trim().is_empty(), "case IDs cannot be empty");
        assert!(ids.insert(case_id), "duplicate case ID: {case_id}");
        let (actual_disposition, actual_signals, actual_uncalibrated) = run_scenario(scenario);

        assert_eq!(actual_disposition, expected_disposition, "case {case_id}");
        match expected_signal {
            Some(signal) => assert!(
                actual_signals.contains(&signal),
                "case {case_id} missing {signal:?}"
            ),
            None => assert!(
                actual_signals.is_empty(),
                "case {case_id} emitted unexpected deviations: {actual_signals:?}"
            ),
        }
        match expected_uncalibrated {
            Some(signal) => assert!(
                actual_uncalibrated.contains(&signal),
                "case {case_id} missing uncalibrated {signal:?}"
            ),
            None => assert!(
                actual_uncalibrated.is_empty(),
                "case {case_id} emitted unexpected uncalibrated signals: {actual_uncalibrated:?}"
            ),
        }
    }

    assert_eq!(ids.len(), 23, "corpus cardinality is versioned and must not drift silently");
}

#[test]
fn frozen_corpus_baseline_builder_rejects_mismatched_reference_identity() {
    let mut reference = healthy_reference();
    reference[0].profile.recipe_revision = "recipe:other".into();
    let error = ReferenceBaseline::fit(profile(), config(), &reference).unwrap_err();
    assert_eq!(
        error,
        BaselineError::ProcessProfileMismatch { sample_index: 0 }
    );
}

#[test]
fn frozen_corpus_config_guard_rejects_invalid_signal_noise_floor() {
    let mut detector_config = config();
    detector_config.minimum_scale_by_signal[Signal::VibrationRms.index()] = 0.0;
    assert_eq!(
        detector_config.validate(),
        Err(ConfigError::InvalidMinimumScale(Signal::VibrationRms))
    );
}
