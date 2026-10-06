use fabrication_common::fpm_context::ProcessContext;
use fabrication_common::fpm_registration::{
    AlignmentMethod, ModalityObservationRef, RegistrationEnvelope, RegistrationState,
    FPM_REGISTRATION_SCHEMA_VERSION,
};

fn digest(ch: char) -> String {
    std::iter::repeat(ch).take(64).collect()
}

fn sample(source: &str, modality: &str, sequence: u64) -> ModalityObservationRef {
    ModalityObservationRef {
        source_id: source.into(),
        modality: modality.into(),
        clock_domain: "ptp-domain-1".into(),
        source_sequence: sequence,
        correlation_domain: "printer-frame-domain-1".into(),
        correlation_id: format!("frame-{sequence}"),
        source_timestamp_micros: Some(1_000_000),
        calibration_profile_digest: digest('a'),
        process_context_digest: digest('b'),
        source_data_digest: digest('c'),
    }
}

fn envelope(method: AlignmentMethod) -> RegistrationEnvelope {
    RegistrationEnvelope {
        schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
        reference: sample("thermal-1", "thermal", 10),
        related: vec![sample("vibration-1", "vibration", 10)],
        alignment_method: Some(method),
    }
}

#[test]
fn registered_requires_explicit_alignment_evidence() {
    let exact = envelope(AlignmentMethod::ExactCorrelationId);
    assert_eq!(exact.assess(), RegistrationState::Registered);

    let mut missing = exact;
    missing.alignment_method = None;
    assert_eq!(missing.assess(), RegistrationState::Unregistered);
}

#[test]
fn correlation_id_mismatch_is_conflicting_even_with_equal_sequences() {
    let mut skewed = envelope(AlignmentMethod::ExactCorrelationId);
    skewed.related[0].correlation_id = "frame-11".into();
    assert_eq!(skewed.assess(), RegistrationState::Conflicting);
}

#[test]
fn correlation_domain_mismatch_is_conflicting_even_with_equal_ids() {
    let mut conflicting = envelope(AlignmentMethod::ExactCorrelationId);
    conflicting.related[0].correlation_domain = "other-domain".into();
    assert_eq!(conflicting.assess(), RegistrationState::Conflicting);
}

#[test]
fn independent_source_sequences_may_differ_when_shared_correlation_is_explicit() {
    let mut registered = envelope(AlignmentMethod::ExactCorrelationId);
    registered.related[0].source_sequence = 11;
    assert_eq!(registered.assess(), RegistrationState::Registered);
}

#[test]
fn context_conflict_is_not_registered() {
    let mut conflicting = envelope(AlignmentMethod::ExactCorrelationId);
    conflicting.related[0].process_context_digest = digest('d');
    assert_eq!(conflicting.assess(), RegistrationState::Conflicting);
}

#[test]
fn calibration_conflict_is_not_registered() {
    let mut conflicting = envelope(AlignmentMethod::ExactCorrelationId);
    conflicting.related[0].calibration_profile_digest = digest('d');
    assert_eq!(conflicting.assess(), RegistrationState::Conflicting);
}

#[test]
fn timestamp_alignment_requires_source_timestamps() {
    let exact = envelope(AlignmentMethod::ExactSourceTimestampMicros);
    assert_eq!(exact.assess(), RegistrationState::Registered);

    let mut unknown = exact;
    unknown.reference.source_timestamp_micros = None;
    assert_eq!(unknown.assess(), RegistrationState::Unknown);
}

#[test]
fn malformed_digest_is_invalid() {
    let mut invalid = envelope(AlignmentMethod::ExactCorrelationId);
    invalid.related[0].source_data_digest = "not-a-digest".into();
    assert_eq!(invalid.assess(), RegistrationState::Invalid);
}

#[test]
fn registered_envelope_is_deterministically_committed() {
    let a = envelope(AlignmentMethod::ExactCorrelationId);
    let b = envelope(AlignmentMethod::ExactCorrelationId);
    assert_eq!(a.digest().expect("digest a"), b.digest().expect("digest b"));
}

#[test]
fn context_digest_can_bind_to_fpm_process_context() {
    let context = ProcessContext {
        phase: Some("perimeter".into()),
        layer_number: Some(7),
        move_class: Some("extrusion".into()),
        commanded_speed_mm_s: Some(45),
        target_hotend_temp_c: Some(210),
        target_bed_temp_c: Some(60),
        acquisition_clock_domain: "printer-clock-1".into(),
        sensor_source_id: "thermal-1".into(),
        calibration_profile_digest: digest('a'),
    };
    let context_digest = context.digest().expect("context digest");

    let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
    registration.reference.process_context_digest = context_digest.clone();
    registration.related[0].process_context_digest = context_digest;

    assert_eq!(registration.assess(), RegistrationState::Registered);
}
