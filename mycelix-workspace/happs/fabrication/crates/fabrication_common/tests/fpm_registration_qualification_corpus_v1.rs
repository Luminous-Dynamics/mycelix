use fabrication_common::fpm_registration::{
    AlignmentMethod, ModalityObservationRef, RegistrationEnvelope, RegistrationState,
    FPM_REGISTRATION_SCHEMA_VERSION,
};
use fabrication_common::fpm_registration_qualification::{
    qualify_registration_structure, StructuralQualificationOutcome,
    FPM_QUALIFICATION_DIGEST_PROFILE_ID, FPM_REGISTRATION_DIGEST_PROFILE_ID,
};
use hdi::prelude::AgentPubKey;

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

fn verifier(byte: u8) -> AgentPubKey {
    AgentPubKey::from_raw_36(vec![byte; 36])
}

#[test]
fn structural_profile_qualifies_consistent_metadata_only() {
    let result = qualify_registration_structure(
        &envelope(AlignmentMethod::ExactCorrelationId),
        &verifier(1),
    )
    .expect("qualification");
    assert_eq!(result.registration_state, RegistrationState::Consistent);
    assert_eq!(result.outcome, StructuralQualificationOutcome::Qualified);
}

#[test]
fn qualification_records_exact_digest_profiles() {
    let result =
        qualify_registration_structure(
            &envelope(AlignmentMethod::ExactCorrelationId),
            &verifier(1),
        )
        .expect("qualification");
    assert_eq!(
        result.registration_digest_profile,
        FPM_REGISTRATION_DIGEST_PROFILE_ID
    );
    assert_eq!(
        result.qualification_digest_profile,
        FPM_QUALIFICATION_DIGEST_PROFILE_ID
    );
}

#[test]
fn unregistered_metadata_is_insufficient_evidence() {
    let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
    registration.alignment_method = None;
    let result = qualify_registration_structure(&registration, &verifier(1))
        .expect("qualification");
    assert_eq!(
        result.outcome,
        StructuralQualificationOutcome::InsufficientEvidence
    );
}

#[test]
fn unknown_alignment_evidence_is_not_qualified() {
    let registration = envelope(AlignmentMethod::DeclaredClockTransform {
        transform_digest: digest('d'),
    });
    let result = qualify_registration_structure(&registration, &verifier(1))
        .expect("qualification");
    assert_eq!(result.registration_state, RegistrationState::Unknown);
    assert_eq!(
        result.outcome,
        StructuralQualificationOutcome::InsufficientEvidence
    );
}

#[test]
fn conflicting_participants_are_rejected() {
    let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
    registration.related[0].correlation_id = "different-frame".into();
    let result = qualify_registration_structure(&registration, &verifier(1))
        .expect("qualification");
    assert_eq!(result.registration_state, RegistrationState::Conflicting);
    assert_eq!(result.outcome, StructuralQualificationOutcome::Rejected);
}

#[test]
fn malformed_registration_is_rejected() {
    let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
    registration.related[0].source_data_digest = "bad".into();
    let result = qualify_registration_structure(&registration, &verifier(1))
        .expect("qualification");
    assert_eq!(result.registration_state, RegistrationState::Invalid);
    assert_eq!(result.outcome, StructuralQualificationOutcome::Rejected);
}

#[test]
fn qualification_receipt_is_self_verifiable() {
    let registration = envelope(AlignmentMethod::ExactCorrelationId);
    let qualification = qualify_registration_structure(&registration, &verifier(1))
        .expect("qualification");
    assert!(qualification.verify_digest().expect("verify"));

    let mut tampered = qualification;
    tampered.verifier = verifier(2);
    assert!(!tampered.verify_digest().expect("tampered verify"));
}

#[test]
fn different_verifiers_produce_distinct_qualification_digests() {
    let registration = envelope(AlignmentMethod::ExactCorrelationId);
    let a = qualify_registration_structure(&registration, &verifier(1)).expect("a");
    let b = qualify_registration_structure(&registration, &verifier(2)).expect("b");
    assert_ne!(a.qualification_digest, b.qualification_digest);
}
