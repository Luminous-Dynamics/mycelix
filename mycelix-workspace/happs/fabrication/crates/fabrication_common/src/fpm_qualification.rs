//! Deterministic qualification of FPM registration evidence.
//!
//! This layer is intentionally narrower than physical validation. It verifies
//! that a registration envelope and the exact evidence commitments it references
//! are available, intact, and sufficient for an explicitly named qualification
//! profile. It does not establish source authenticity, clock synchronization,
//! calibration correctness, or physical truth.

use crate::fpm_registration::{
    AlignmentMethod, RegistrationEnvelope, RegistrationState,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const FPM_REGISTRATION_QUALIFICATION_SCHEMA_VERSION: &str =
    "fpm.registration-qualification.v1";
pub const FPM_REGISTRATION_QUALIFICATION_PROFILE_ID: &str =
    "fpm.registration.structural";
pub const FPM_REGISTRATION_QUALIFICATION_PROFILE_VERSION: &str = "1";

const SHA256_HEX_LEN: usize = 64;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EvidenceKind {
    SourceData,
    /// Canonical record binding source metadata to the source-data commitment.
    SourceObservation,
    CalibrationProfile,
    ProcessContext,
    AlignmentEvidence,
}

impl EvidenceKind {
    fn tag(self) -> &'static str {
        match self {
            Self::SourceData => "source-data",
            Self::SourceObservation => "source-observation",
            Self::CalibrationProfile => "calibration-profile",
            Self::ProcessContext => "process-context",
            Self::AlignmentEvidence => "alignment-evidence",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedEvidenceArtifact {
    /// The commitment claimed by the registration envelope or derived exact
    /// source-observation binding for this artifact.
    pub declared_digest: String,
    /// Original immutable bytes resolved for the committed artifact.
    pub bytes: Vec<u8>,
    pub kind: EvidenceKind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationQualificationInput {
    /// Exact digest expected for the serialized registration envelope.
    pub registration_envelope_digest: String,
    pub envelope: RegistrationEnvelope,
    /// Exact bytes resolved for the commitments referenced by the envelope.
    pub artifacts: Vec<ResolvedEvidenceArtifact>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistrationQualificationVerifier {
    /// Stable verifier identity. This is an identifier, not an authentication claim.
    pub verifier_id: String,
    /// Stable implementation/profile version for the verifier.
    pub verifier_version: String,
    /// Immutable commitment to the verifier implementation artifact.
    pub verifier_implementation_digest: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RegistrationQualificationStatus {
    QualifiedForProfile,
    InsufficientEvidence,
    InvalidEvidence,
    ConflictingEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, PartialOrd, Ord)]
pub enum RegistrationQualificationReason {
    EmptyVerifierIdentity,
    EmptyVerifierVersion,
    InvalidVerifierImplementationDigest,
    EnvelopeDigestMismatch,
    RegistrationUnregistered,
    RegistrationConflicting,
    RegistrationInvalid,
    RegistrationUnknown,
    MissingCommittedArtifact,
    ArtifactDigestMismatch,
    ArtifactBindingMismatch,
    DuplicateArtifact,
    UnexpectedArtifact,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RegistrationQualification {
    pub schema_version: String,
    pub registration_envelope_digest: String,
    pub evidence_manifest_digest: String,
    pub qualification_basis_digest: String,
    pub verifier_id: String,
    pub verifier_version: String,
    pub verifier_implementation_digest: String,
    pub profile_id: String,
    pub profile_version: String,
    pub profile_digest: String,
    pub status: RegistrationQualificationStatus,
    pub reasons: Vec<RegistrationQualificationReason>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegistrationQualificationProfile {
    profile_id: &'static str,
    profile_version: &'static str,
    requires_consistent_registration: bool,
    requires_exact_committed_artifacts: bool,
}

impl RegistrationQualificationProfile {
    pub const STRUCTURAL_V1: Self = Self {
        profile_id: FPM_REGISTRATION_QUALIFICATION_PROFILE_ID,
        profile_version: FPM_REGISTRATION_QUALIFICATION_PROFILE_VERSION,
        requires_consistent_registration: true,
        requires_exact_committed_artifacts: true,
    };

    /// Deterministically commit to the declarative profile itself.
    pub fn digest(&self) -> String {
        let mut bytes = Vec::new();
        append_field(&mut bytes, b"fpm.registration-qualification-profile.v1");
        append_field(&mut bytes, self.profile_id.as_bytes());
        append_field(&mut bytes, self.profile_version.as_bytes());
        append_field(
            &mut bytes,
            if self.requires_consistent_registration {
                b"1"
            } else {
                b"0"
            },
        );
        append_field(
            &mut bytes,
            if self.requires_exact_committed_artifacts {
                b"1"
            } else {
                b"0"
            },
        );
        hex_digest(&bytes)
    }
}

pub fn qualify_registration(
    profile: RegistrationQualificationProfile,
    verifier: &RegistrationQualificationVerifier,
    input: &RegistrationQualificationInput,
) -> RegistrationQualification {
    let mut reasons = BTreeSet::new();

    if verifier.verifier_id.trim().is_empty() {
        reasons.insert(RegistrationQualificationReason::EmptyVerifierIdentity);
    }
    if verifier.verifier_version.trim().is_empty() {
        reasons.insert(RegistrationQualificationReason::EmptyVerifierVersion);
    }
    if !is_valid_digest(&verifier.verifier_implementation_digest) {
        reasons.insert(RegistrationQualificationReason::InvalidVerifierImplementationDigest);
    }

    let computed_envelope_digest = input.envelope.digest();
    match computed_envelope_digest {
        Ok(digest) if digest == input.registration_envelope_digest => {}
        _ => {
            reasons.insert(RegistrationQualificationReason::EnvelopeDigestMismatch);
        }
    }

    if profile.requires_consistent_registration {
        match input.envelope.assess() {
            RegistrationState::Consistent => {}
            RegistrationState::Unregistered => {
                reasons.insert(RegistrationQualificationReason::RegistrationUnregistered);
            }
            RegistrationState::Conflicting => {
                reasons.insert(RegistrationQualificationReason::RegistrationConflicting);
            }
            RegistrationState::Invalid => {
                reasons.insert(RegistrationQualificationReason::RegistrationInvalid);
            }
            RegistrationState::Unknown => {
                reasons.insert(RegistrationQualificationReason::RegistrationUnknown);
            }
        }
    }

    let expected = expected_artifacts(&input.envelope);
    let mut seen = BTreeSet::new();

    for artifact in &input.artifacts {
        let key = (artifact.kind, artifact.declared_digest.clone());
        if !seen.insert(key.clone()) {
            reasons.insert(RegistrationQualificationReason::DuplicateArtifact);
            continue;
        }

        if !expected.contains(&key) {
            reasons.insert(RegistrationQualificationReason::UnexpectedArtifact);
            continue;
        }

        if !is_valid_digest(&artifact.declared_digest)
            || hex_digest(&artifact.bytes) != artifact.declared_digest
        {
            reasons.insert(RegistrationQualificationReason::ArtifactDigestMismatch);
            continue;
        }

        if artifact.kind == EvidenceKind::SourceObservation {
            let matches_declared_record = expected.contains(&(
                EvidenceKind::SourceObservation,
                artifact.declared_digest.clone(),
            ));
            let canonical_records = canonical_source_observation_records(&input.envelope);
            let matches_canonical_record = canonical_records
                .iter()
                .any(|(digest, bytes)| digest == &artifact.declared_digest && bytes == &artifact.bytes);
            if !matches_declared_record || !matches_canonical_record {
                reasons.insert(RegistrationQualificationReason::ArtifactBindingMismatch);
            }
        }
    }

    if profile.requires_exact_committed_artifacts {
        for key in &expected {
            if !seen.contains(key) {
                reasons.insert(RegistrationQualificationReason::MissingCommittedArtifact);
            }
        }
    }

    let evidence_manifest_digest = evidence_manifest_digest(&input.artifacts);
    let qualification_basis_digest = qualification_basis_digest(
        &input.registration_envelope_digest,
        &evidence_manifest_digest,
        &profile.digest(),
    );

    let status = if reasons.is_empty() {
        RegistrationQualificationStatus::QualifiedForProfile
    } else if reasons.iter().any(|reason| {
        matches!(
            reason,
            RegistrationQualificationReason::EmptyVerifierIdentity
                | RegistrationQualificationReason::EmptyVerifierVersion
                | RegistrationQualificationReason::InvalidVerifierImplementationDigest
                | RegistrationQualificationReason::EnvelopeDigestMismatch
                | RegistrationQualificationReason::ArtifactDigestMismatch
                | RegistrationQualificationReason::ArtifactBindingMismatch
                | RegistrationQualificationReason::DuplicateArtifact
                | RegistrationQualificationReason::UnexpectedArtifact
                | RegistrationQualificationReason::RegistrationInvalid
        )
    }) {
        RegistrationQualificationStatus::InvalidEvidence
    } else if reasons.contains(&RegistrationQualificationReason::RegistrationConflicting) {
        RegistrationQualificationStatus::ConflictingEvidence
    } else {
        RegistrationQualificationStatus::InsufficientEvidence
    };

    RegistrationQualification {
        schema_version: FPM_REGISTRATION_QUALIFICATION_SCHEMA_VERSION.into(),
        registration_envelope_digest: input.registration_envelope_digest.clone(),
        evidence_manifest_digest,
        qualification_basis_digest,
        verifier_id: verifier.verifier_id.clone(),
        verifier_version: verifier.verifier_version.clone(),
        verifier_implementation_digest: verifier.verifier_implementation_digest.clone(),
        profile_id: profile.profile_id.into(),
        profile_version: profile.profile_version.into(),
        profile_digest: profile.digest(),
        status,
        reasons: reasons.into_iter().collect(),
    }
}

fn expected_artifacts(envelope: &RegistrationEnvelope) -> BTreeSet<(EvidenceKind, String)> {
    let participants = std::iter::once(&envelope.reference).chain(envelope.related.iter());
    let mut expected = BTreeSet::new();

    for participant in participants {
        expected.insert((
            EvidenceKind::SourceData,
            participant.source_data_digest.clone(),
        ));
        expected.insert((
            EvidenceKind::SourceObservation,
            source_observation_digest(participant),
        ));
        expected.insert((
            EvidenceKind::CalibrationProfile,
            participant.calibration_profile_digest.clone(),
        ));
        expected.insert((
            EvidenceKind::ProcessContext,
            participant.process_context_digest.clone(),
        ));
    }

    match &envelope.alignment_method {
        Some(AlignmentMethod::DeclaredClockTransform { transform_digest }) => {
            expected.insert((EvidenceKind::AlignmentEvidence, transform_digest.clone()));
        }
        Some(AlignmentMethod::ExternalRegistrationEvidence { evidence_digest }) => {
            expected.insert((EvidenceKind::AlignmentEvidence, evidence_digest.clone()));
        }
        Some(AlignmentMethod::ExactCorrelationId | AlignmentMethod::ExactSourceTimestampMicros)
        | None => {}
    }

    expected
}

fn is_valid_digest(value: &str) -> bool {
    value.len() == SHA256_HEX_LEN
        && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn qualification_basis_digest(
    registration_envelope_digest: &str,
    evidence_manifest_digest: &str,
    profile_digest: &str,
) -> String {
    let mut bytes = Vec::new();
    append_field(&mut bytes, b"fpm.registration-qualification-basis.v1");
    append_field(&mut bytes, registration_envelope_digest.as_bytes());
    append_field(&mut bytes, evidence_manifest_digest.as_bytes());
    append_field(&mut bytes, profile_digest.as_bytes());
    hex_digest(&bytes)
}

fn source_observation_digest(
    participant: &crate::fpm_registration::ModalityObservationRef,
) -> String {
    hex_digest(&canonical_source_observation_bytes(participant))
}

fn canonical_source_observation_bytes(
    participant: &crate::fpm_registration::ModalityObservationRef,
) -> Vec<u8> {
    let mut bytes = Vec::new();
    append_field(&mut bytes, b"fpm.source-observation.v1");
    append_field(&mut bytes, participant.source_id.as_bytes());
    append_field(&mut bytes, participant.modality.as_bytes());
    append_field(&mut bytes, participant.clock_domain.as_bytes());
    append_field(&mut bytes, &participant.source_sequence.to_be_bytes());
    append_field(&mut bytes, participant.correlation_domain.as_bytes());
    append_field(&mut bytes, participant.correlation_id.as_bytes());
    match participant.source_timestamp_micros {
        Some(value) => {
            append_field(&mut bytes, b"some");
            append_field(&mut bytes, &value.to_be_bytes());
        }
        None => append_field(&mut bytes, b"none"),
    }
    append_field(&mut bytes, participant.calibration_profile_digest.as_bytes());
    append_field(&mut bytes, participant.process_context_digest.as_bytes());
    append_field(&mut bytes, participant.source_data_digest.as_bytes());
    bytes
}

fn canonical_source_observation_records(
    envelope: &RegistrationEnvelope,
) -> Vec<(String, Vec<u8>)> {
    std::iter::once(&envelope.reference)
        .chain(envelope.related.iter())
        .map(|participant| {
            (
                source_observation_digest(participant),
                canonical_source_observation_bytes(participant),
            )
        })
        .collect()
}

fn evidence_manifest_digest(artifacts: &[ResolvedEvidenceArtifact]) -> String {
    let mut entries = artifacts
        .iter()
        .map(|artifact| (artifact.kind, artifact.declared_digest.as_str()))
        .collect::<Vec<_>>();
    entries.sort_unstable();

    let mut bytes = Vec::new();
    append_field(&mut bytes, b"fpm.evidence-manifest.v1");
    for (kind, digest) in entries {
        append_field(&mut bytes, kind.tag().as_bytes());
        append_field(&mut bytes, digest.as_bytes());
    }
    hex_digest(&bytes)
}

fn append_field(buffer: &mut Vec<u8>, field: &[u8]) {
    buffer.extend_from_slice(&(field.len() as u64).to_be_bytes());
    buffer.extend_from_slice(field);
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fpm_registration::{AlignmentMethod, ModalityObservationRef, FPM_REGISTRATION_SCHEMA_VERSION};

    fn digest(ch: char) -> String {
        std::iter::repeat(ch).take(SHA256_HEX_LEN).collect()
    }

    fn verifier() -> RegistrationQualificationVerifier {
        RegistrationQualificationVerifier {
            verifier_id: "fpm.registration.qualifier".into(),
            verifier_version: "1".into(),
            verifier_implementation_digest: digest('d'),
        }
    }

    fn sample(source: &str, modality: &str, sequence: u64, data: &[u8]) -> (ModalityObservationRef, ResolvedEvidenceArtifact) {
        let source_data_digest = hex_digest(data);
        (
            ModalityObservationRef {
                source_id: source.into(),
                modality: modality.into(),
                clock_domain: "ptp-domain-1".into(),
                source_sequence: sequence,
                correlation_domain: "printer-frame-domain-1".into(),
                correlation_id: format!("frame-{sequence}"),
                source_timestamp_micros: Some(1_000_000),
                calibration_profile_digest: hex_digest(b"calibration-profile-v1"),
                process_context_digest: hex_digest(b"process-context-v1"),
                source_data_digest: source_data_digest.clone(),
            },
            ResolvedEvidenceArtifact {
                declared_digest: source_data_digest,
                bytes: data.to_vec(),
                kind: EvidenceKind::SourceData,
            },
        )
    }

    fn input_with_exact_artifacts() -> RegistrationQualificationInput {
        let (reference, source_artifact) = sample("thermal-1", "thermal", 10, b"thermal-frame");
        let (related, related_source_artifact) = sample("vibration-1", "vibration", 10, b"vibration-frame");
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference,
            related: vec![related],
            alignment_method: Some(AlignmentMethod::ExactCorrelationId),
        };
        let registration_envelope_digest = envelope.digest().expect("envelope digest");

        let calibration_bytes = b"calibration-profile-v1";
        let calibration_digest = hex_digest(calibration_bytes);
        let context_bytes = b"process-context-v1";
        let context_digest = hex_digest(context_bytes);

        let artifacts = vec![
            source_artifact,
            related_source_artifact,
            ResolvedEvidenceArtifact {
                declared_digest: source_observation_digest(&envelope.reference),
                bytes: canonical_source_observation_bytes(&envelope.reference),
                kind: EvidenceKind::SourceObservation,
            },
            ResolvedEvidenceArtifact {
                declared_digest: source_observation_digest(&envelope.related[0]),
                bytes: canonical_source_observation_bytes(&envelope.related[0]),
                kind: EvidenceKind::SourceObservation,
            },
            ResolvedEvidenceArtifact {
                declared_digest: calibration_digest,
                bytes: calibration_bytes.to_vec(),
                kind: EvidenceKind::CalibrationProfile,
            },
            ResolvedEvidenceArtifact {
                declared_digest: context_digest,
                bytes: context_bytes.to_vec(),
                kind: EvidenceKind::ProcessContext,
            },
        ];

        RegistrationQualificationInput {
            registration_envelope_digest,
            envelope,
            artifacts,
        }
    }

    #[test]
    fn structural_profile_is_deterministically_identified() {
        let profile = RegistrationQualificationProfile::STRUCTURAL_V1;
        assert_eq!(profile.digest(), RegistrationQualificationProfile::STRUCTURAL_V1.digest());
        assert_eq!(profile.digest().len(), SHA256_HEX_LEN);
    }

    #[test]
    fn exact_evidence_is_qualified_without_wall_clock_or_dht_state() {
        let input = input_with_exact_artifacts();
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::QualifiedForProfile
        );
        assert!(qualification.reasons.is_empty());
    }

    #[test]
    fn missing_committed_evidence_cannot_qualify() {
        let mut input = input_with_exact_artifacts();
        input.artifacts.pop();
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InsufficientEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::MissingCommittedArtifact));
    }

    #[test]
    fn tampered_artifact_bytes_are_invalid_not_missing() {
        let mut input = input_with_exact_artifacts();
        input.artifacts[0].bytes = b"tampered".to_vec();
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::ArtifactDigestMismatch));
    }

    #[test]
    fn cross_kind_digest_confusion_is_rejected() {
        let mut input = input_with_exact_artifacts();
        input.artifacts[0].kind = EvidenceKind::CalibrationProfile;
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::MissingCommittedArtifact));
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::UnexpectedArtifact));
    }

    #[test]
    fn envelope_substitution_is_rejected_by_exact_digest() {
        let mut input = input_with_exact_artifacts();
        input.envelope.related[0].source_sequence = 11;
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::EnvelopeDigestMismatch));
    }

    #[test]
    fn unresolved_clock_transform_cannot_qualify_even_with_committed_evidence() {
        let mut input = input_with_exact_artifacts();
        let transform_bytes = b"clock-transform-v1";
        let transform_digest = hex_digest(transform_bytes);
        input.envelope.alignment_method = Some(AlignmentMethod::DeclaredClockTransform {
            transform_digest: transform_digest.clone(),
        });
        input.registration_envelope_digest = input.envelope.digest().expect("envelope digest");
        input.artifacts.push(ResolvedEvidenceArtifact {
            declared_digest: transform_digest,
            bytes: transform_bytes.to_vec(),
            kind: EvidenceKind::AlignmentEvidence,
        });

        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InsufficientEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::RegistrationUnknown));
    }

    #[test]
    fn evidence_manifest_digest_is_order_independent() {
        let input = input_with_exact_artifacts();
        let mut reversed = input.artifacts.clone();
        reversed.reverse();

        assert_eq!(
            evidence_manifest_digest(&input.artifacts),
            evidence_manifest_digest(&reversed)
        );
    }

    #[test]
    fn empty_verifier_identity_cannot_qualify() {
        let input = input_with_exact_artifacts();
        let verifier = RegistrationQualificationVerifier {
            verifier_id: "   ".into(),
            verifier_version: "1".into(),
            verifier_implementation_digest: digest('d'),
        };

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::EmptyVerifierIdentity));
    }

    #[test]
    fn participant_payload_binding_rejects_metadata_relabeling() {
        let mut input = input_with_exact_artifacts();
        let thermal_artifact = input
            .artifacts
            .iter_mut()
            .find(|artifact| artifact.kind == EvidenceKind::SourceObservation)
            .expect("thermal source-observation artifact");
        thermal_artifact.bytes = canonical_source_observation_bytes(&input.envelope.related[0]);

        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::ArtifactDigestMismatch));
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::ArtifactBindingMismatch));
    }

    #[test]
    fn participant_payload_binding_rejects_replayed_source_identity() {
        let mut input = input_with_exact_artifacts();
        let original = input.envelope.reference.clone();
        let mut replay = original.clone();
        replay.source_id = "thermal-replay".into();

        let artifact = input
            .artifacts
            .iter_mut()
            .find(|artifact| {
                artifact.kind == EvidenceKind::SourceObservation
                    && artifact.declared_digest == source_observation_digest(&original)
            })
            .expect("reference source-observation artifact");
        artifact.bytes = canonical_source_observation_bytes(&replay);

        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::ArtifactDigestMismatch));
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::ArtifactBindingMismatch));
    }

    #[test]
    fn participant_payload_binding_is_deterministic_and_exact() {
        let input = input_with_exact_artifacts();
        let records = canonical_source_observation_records(&input.envelope);
        assert_eq!(records.len(), 2);

        for (digest, bytes) in records {
            assert_eq!(hex_digest(&bytes), digest);
            assert_eq!(source_observation_digest(
                if digest == source_observation_digest(&input.envelope.reference) {
                    &input.envelope.reference
                } else {
                    &input.envelope.related[0]
                }
            ), digest);
        }
    }

    #[test]
    fn qualification_basis_binds_envelope_evidence_and_profile() {
        let input = input_with_exact_artifacts();
        let verifier = verifier();

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.qualification_basis_digest,
            qualification_basis_digest(
                &qualification.registration_envelope_digest,
                &qualification.evidence_manifest_digest,
                &qualification.profile_digest,
            )
        );
    }

    #[test]
    fn invalid_verifier_implementation_digest_cannot_qualify() {
        let input = input_with_exact_artifacts();
        let verifier = RegistrationQualificationVerifier {
            verifier_id: "fpm.registration.qualifier".into(),
            verifier_version: "1".into(),
            verifier_implementation_digest: "not-a-digest".into(),
        };

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::InvalidEvidence
        );
        assert!(qualification
            .reasons
            .contains(&RegistrationQualificationReason::InvalidVerifierImplementationDigest));
    }

    #[test]
    fn verifier_identity_is_part_of_the_record_but_not_an_authentication_claim() {
        let input = input_with_exact_artifacts();
        let verifier = RegistrationQualificationVerifier {
            verifier_id: "untrusted-label".into(),
            verifier_version: "1".into(),
            verifier_implementation_digest: digest('d'),
        };

        let qualification =
            qualify_registration(RegistrationQualificationProfile::STRUCTURAL_V1, &verifier, &input);

        assert_eq!(
            qualification.status,
            RegistrationQualificationStatus::QualifiedForProfile
        );
        assert_eq!(qualification.verifier_id, "untrusted-label");
    }
}
