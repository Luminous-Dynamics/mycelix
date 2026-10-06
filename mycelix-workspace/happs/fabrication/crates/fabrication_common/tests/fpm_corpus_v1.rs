use fabrication_common::fpm::{analyze, BaselineProfile, DetectorConfig, DetectionStatus, FpmError, ProcessAnomalyType, ProcessObservation};
use fabrication_common::SensorSnapshot;

fn sensor(hotend: f32, bed: f32, extruder: f32, vibration: f32, filament: Option<f32>) -> SensorSnapshot {
    SensorSnapshot {
        hotend_temp: hotend,
        bed_temp: bed,
        stepper_currents: [0.8, 0.8, 1.0, extruder],
        vibration_rms: vibration,
        filament_tension: filament,
        ambient_temp: Some(22.0),
        humidity: Some(40.0),
    }
}

fn obs(sequence: u64, sensor: SensorSnapshot) -> ProcessObservation {
    ProcessObservation { sequence, sensor }
}

fn baseline() -> BaselineProfile {
    let observations = (0..10)
        .map(|sequence| obs(sequence, sensor(210.0 + (sequence % 2) as f32 * 0.2, 60.0, 1.0, 0.05, Some(100.0))))
        .collect::<Vec<_>>();
    BaselineProfile::from_observations(&DetectorConfig::default(), &observations)
        .expect("frozen FPM corpus baseline must build")
}

#[test]
fn fpm_corpus_v1_expected_results() {
    let config = DetectorConfig::default();
    let baseline = baseline();

    let normal = analyze(&config, &baseline, &obs(100, sensor(210.1, 60.0, 1.0, 0.05, Some(100.0))))
        .expect("healthy observation should analyze");
    assert_eq!(normal.status, DetectionStatus::Normal);

    let hotend = analyze(&config, &baseline, &obs(101, sensor(250.0, 60.0, 1.0, 0.05, Some(100.0))))
        .expect("hotend case should analyze");
    assert_eq!(hotend.status, DetectionStatus::Anomalous);
    assert!(hotend.anomaly_types.contains(&ProcessProcessAnomalyType::TemperatureDeviation));

    let bed = analyze(&config, &baseline, &obs(102, sensor(210.0, 90.0, 1.0, 0.05, Some(100.0))))
        .expect("bed case should analyze");
    assert!(bed.anomaly_types.contains(&ProcessProcessAnomalyType::TemperatureDeviation));

    let extruder = analyze(&config, &baseline, &obs(103, sensor(210.0, 60.0, 2.0, 0.05, Some(100.0))))
        .expect("extruder case should analyze");
    assert!(extruder.anomaly_types.contains(&ProcessProcessAnomalyType::ExtrusionInconsistency));

    let vibration = analyze(&config, &baseline, &obs(104, sensor(210.0, 60.0, 1.0, 0.5, Some(100.0))))
        .expect("vibration case should analyze");
    assert!(vibration.anomaly_types.contains(&ProcessProcessAnomalyType::VibrationAnomaly));

    let filament = analyze(&config, &baseline, &obs(105, sensor(210.0, 60.0, 1.0, 0.05, Some(500.0))))
        .expect("filament case should analyze");
    assert!(filament.anomaly_types.contains(&ProcessProcessAnomalyType::FilamentSlip));

    let mut nonfinite = obs(106, sensor(210.0, 60.0, 1.0, 0.05, Some(100.0)));
    nonfinite.sensor.vibration_rms = f32::NAN;
    assert_eq!(
        analyze(&config, &baseline, &nonfinite),
        Err(FpmError::NonFiniteObservation { sequence: 106 })
    );

    let mut wrong_version = config.clone();
    wrong_version.detector_version = "2".to_string();
    assert!(matches!(
        analyze(&wrong_version, &baseline, &obs(107, sensor(210.0, 60.0, 1.0, 0.05, Some(100.0)))),
        Err(FpmError::InvalidConfiguration(_))
    ));

    let digest_a = analyze(&config, &baseline, &obs(108, sensor(210.0, 60.0, 1.0, 0.05, Some(100.0))))
        .expect("digest A");
    let digest_b = analyze(&config, &baseline, &obs(108, sensor(211.0, 60.0, 1.0, 0.05, Some(100.0))))
        .expect("digest B");
    assert_ne!(digest_a.provenance.input_digest, digest_b.provenance.input_digest);
}

#[test]
fn fpm_corpus_v1_baseline_reordering_is_rejected() {
    let observations = vec![
        obs(2, sensor(210.0, 60.0, 1.0, 0.05, None)),
        obs(1, sensor(210.0, 60.0, 1.0, 0.05, None)),
    ];
    assert_eq!(
        BaselineProfile::from_observations(&DetectorConfig::default(), &observations),
        Err(FpmError::NonMonotonicSequence)
    );
}