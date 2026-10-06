use fabrication_common::fpm::{DetectorConfig, ProcessObservation};
use fabrication_common::fpm_context::{
    analyze, ContextualBaselineProfile, ContextualFpmError, ContextualObservation, ProcessContext,
};
use fabrication_common::SensorSnapshot;

fn context() -> ProcessContext {
    ProcessContext {
        phase: Some("perimeter".into()),
        layer_number: Some(7),
        move_class: Some("extrusion".into()),
        commanded_speed_mm_s: Some(45),
        target_hotend_temp_c: Some(210),
        target_bed_temp_c: Some(60),
        acquisition_clock_domain: "printer-clock-1".into(),
        sensor_source_id: "sensor-array-1".into(),
        calibration_profile_digest:
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
    }
}

fn observation(ctx: ProcessContext, sequence: u64, hotend: f32) -> ContextualObservation {
    ContextualObservation {
        observation: ProcessObservation {
            sequence,
            sensor: SensorSnapshot {
                hotend_temp: hotend,
                bed_temp: 60.0,
                stepper_currents: [0.8, 0.8, 1.0, 1.0],
                vibration_rms: 0.05,
                filament_tension: Some(100.0),
                ambient_temp: Some(22.0),
                humidity: Some(40.0),
            },
        },
        context: ctx,
    }
}

fn baseline() -> ContextualBaselineProfile {
    let ctx = context();
    let observations = (0..10)
        .map(|sequence| observation(ctx.clone(), sequence, 210.0))
        .collect::<Vec<_>>();
    ContextualBaselineProfile::from_observations(&DetectorConfig::default(), &observations)
        .expect("frozen contextual baseline must build")
}

#[test]
fn exact_context_is_accepted() {
    let config = DetectorConfig::default();
    let result = analyze(&config, &baseline(), &observation(context(), 100, 210.0));
    assert!(result.is_ok());
}

#[test]
fn layer_mismatch_is_rejected_even_when_sensor_values_are_identical() {
    let config = DetectorConfig::default();
    let mut mismatched = context();
    mismatched.layer_number = Some(8);
    assert_eq!(
        analyze(&config, &baseline(), &observation(mismatched, 100, 210.0)),
        Err(ContextualFpmError::ContextMismatch)
    );
}

#[test]
fn calibration_mismatch_is_rejected_even_when_sensor_values_are_identical() {
    let config = DetectorConfig::default();
    let mut mismatched = context();
    mismatched.calibration_profile_digest =
        "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
    assert_eq!(
        analyze(&config, &baseline(), &observation(mismatched, 100, 210.0)),
        Err(ContextualFpmError::ContextMismatch)
    );
}

#[test]
fn missing_process_discriminator_cannot_select_a_baseline() {
    let mut ctx = context();
    ctx.phase = None;
    ctx.layer_number = None;
    ctx.move_class = None;
    ctx.commanded_speed_mm_s = None;
    ctx.target_hotend_temp_c = None;
    ctx.target_bed_temp_c = None;

    let result = ContextualBaselineProfile::from_observations(
        &DetectorConfig::default(),
        &(0..10)
            .map(|sequence| observation(ctx.clone(), sequence, 210.0))
            .collect::<Vec<_>>(),
    );
    assert!(matches!(
        result,
        Err(ContextualFpmError::IncompleteContext(_))
    ));
}

#[test]
fn mixed_context_source_window_cannot_build_one_baseline() {
    let ctx = context();
    let mut observations = (0..10)
        .map(|sequence| observation(ctx.clone(), sequence, 210.0))
        .collect::<Vec<_>>();
    let mut different = ctx;
    different.move_class = Some("travel".into());
    observations[9].context = different;

    assert_eq!(
        ContextualBaselineProfile::from_observations(&DetectorConfig::default(), &observations),
        Err(ContextualFpmError::ContextMismatch)
    );
}
