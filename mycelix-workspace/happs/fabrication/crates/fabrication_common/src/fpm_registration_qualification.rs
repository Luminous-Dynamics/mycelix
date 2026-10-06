//! Deterministic structural qualification for FPM registration metadata.
//!
//! This profile qualifies only the internal structure of a registration envelope.
//! It does not qualify source authenticity, sensor correctness, clock accuracy,
//! calibration correctness, or physical part quality.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::fpm_registration::{
    RegistrationEnvelope, RegistrationState, FPM_REGISTRATION_SCHEMA_VERSION,
};

pub const FPM_STRUCTURAL_QUALIFICATION_SCHEMA_VERSION: &str = "fpm.registration.qualification.v1";
pub const FPM_STRUCTURAL_QUALIFICATION_PROFILE_ID: &str = "fpm.registration.structure";
pub const FPM_STRUCTURAL_QUALIFICATION_PROFILE_VERSION: &str = "1";
const MAX_VERIFIER_ID_BYTES: usize = 128;
const SHA256_HEX_LEN: usize = 64;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum StructuralQualificationOutcome {
    Qualified,
    Rejected,
    InsufficientEvidence,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct StructuralQualification {
    pub schema_version: String,
    pub profile_id: String,
    pub profile_version: String,
    /// Identity of the agent/tool that authored this qualification statement.
    pub verifier_id: String,
    pub registration_schema_version: String,
    pub registration_digest: String,
    pub registration_state: RegistrationState,
    pub outcome: StructuralQualificationOutcome,
    /// Digest over the exact qualification inputs and outcome.
    pub qualification_digest: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationError {
    InvalidVerifierId,
    RegistrationDigest(String),
    Serialization(String),
}

impl std::fmt::Display for QualificationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidVerifierId => write!(f, "verifier_id is empty or too long"),
            Self::RegistrationDigest(reason) => {
                write!(f, "failed to digest registration: {reason}")
            }
            Self::Serialization(reason) => {
                write!(f, "failed to serialize qualification: {reason}")
            }
        }
    }
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

fn registration_digest(
    registration: &RegistrationEnvelope,
) -> Result<String, QualificationError> {
    let bytes = serde_json::to_vec(registration).map_err(|e| {
        QualificationError::RegistrationDigest(e.to_string())
    })?;
    Ok(hex_digest(&bytes))
}

fn qualification_digest(
    qualification: &StructuralQualification,
) -> Result<String, QualificationError> {
    let mut unsigned = qualification.clone();
    unsigned.qualification_digest.clear();
    let bytes = serde_json::to_vec(&unsigned).map_err(|e| {
        QualificationError::Serialization(e.to_string())
    })?;
    Ok(hex_digest(&bytes))
}

fn outcome_for(state: RegistrationState) -> StructuralQualificationOutcome {
    match state {
        RegistrationState::Consistent => StructuralQualificationOutcome::Qualified,
        RegistrationState::Unregistered | RegistrationState::Unknown => {
            StructuralQualificationOutcome::InsufficientEvidence
        }
        RegistrationState::Conflicting | RegistrationState::Invalid => {
            StructuralQualificationOutcome::Rejected
        }
    }
}

/// Deterministically qualify only the structural consistency of an FPM
/// registration envelope.
///
/// Qualified means qualified against the structural profile. It is not a
/// claim that the underlying sources or clocks are independently verified.
pub fn qualify_registration_structure(
    registration: &RegistrationEnvelope,
    verifier_id: &str,
) -> Result<StructuralQualification, QualificationError> {
    if verifier_id.trim().is_empty()
        || verifier_id != verifier_id.trim()
        || verifier_id.len() > MAX_VERIFIER_ID_BYTES
    {
        return Err(QualificationError::InvalidVerifierId);
    }

    let registration_digest = registration_digest(registration)?;
    let state = registration.assess();
    let mut qualification = StructuralQualification {
        schema_version: FPM_STRUCTURAL_QUALIFICATION_SCHEMA_VERSION.into(),
        profile_id: FPM_STRUCTURAL_QUALIFICATION_PROFILE_ID.into(),
        profile_version: FPM_STRUCTURAL_QUALIFICATION_PROFILE_VERSION.into(),
        verifier_id: verifier_id.into(),
        registration_schema_version: registration.schema_version.clone(),
        registration_digest,
        registration_state: state,
        outcome: outcome_for(state),
        qualification_digest: String::new(),
    };
    qualification.qualification_digest = qualification_digest(&qualification)?;
    Ok(qualification)
}

impl StructuralQualification {
    /// Recompute the content digest with the stored digest field excluded.
    pub fn digest(&self) -> Result<String, QualificationError> {
        qualification_digest(self)
    }

    /// Verify that the stored qualification digest matches its content.
    pub fn verify_digest(&self) -> Result<bool, QualificationError> {
        Ok(self.digest()? == self.qualification_digest)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fpm_registration::AlignmentMethod;
    use crate::fpm_registration::ModalityObservationRef;

    fn digest(ch: char) -> String {
        std::iter::repeat(ch).take(SHA256_HEX_LEN).collect()
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
    fn exact_correlation_structure_can_be_qualified() {
        let registration = envelope(AlignmentMethod::ExactCorrelationId);
        let result =
            qualify_registration_structure(&registration, "verifier-1").expect("qualification");
        assert_eq!(result.registration_state, RegistrationState::Consistent);
        assert_eq!(result.outcome, StructuralQualificationOutcome::Qualified);
        assert!(!result.qualification_digest.is_empty());
    }

    #[test]
    fn incomplete_registration_is_not_qualified() {
        let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
        registration.alignment_method = None;
        let result =
            qualify_registration_structure(&registration, "verifier-1").expect("qualification");
        assert_eq!(result.outcome, StructuralQualificationOutcome::InsufficientEvidence);
    }

    #[test]
    fn conflicting_registration_is_rejected() {
        let mut registration = envelope(AlignmentMethod::ExactCorrelationId);
        registration.related[0].correlation_id = "other-frame".into();
        let result =
            qualify_registration_structure(&registration, "verifier-1").expect("qualification");
        assert_eq!(result.registration_state, RegistrationState::Conflicting);
        assert_eq!(result.outcome, StructuralQualificationOutcome::Rejected);
    }

    #[test]
    fn unverified_transform_remains_insufficient() {
        let registration = envelope(AlignmentMethod::DeclaredClockTransform {
            transform_digest: digest('d'),
        });
        let result =
            qualify_registration_structure(&registration, "verifier-1").expect("qualification");
        assert_eq!(result.registration_state, RegistrationState::Unknown);
        assert_eq!(result.outcome, StructuralQualificationOutcome::InsufficientEvidence);
    }

    #[test]
    fn verifier_id_is_required_and_canonical() {
        let registration = envelope(AlignmentMethod::ExactCorrelationId);
        assert_eq!(
            qualify_registration_structure(&registration, " "),
            Err(QualificationError::InvalidVerifierId)
        );
        assert_eq!(
            qualify_registration_structure(&registration, " verifier-1"),
            Err(QualificationError::InvalidVerifierId)
        );
    }

    #[test]
    fn qualification_digest_is_self_verifiable() {
        let registration = envelope(AlignmentMethod::ExactCorrelationId);
        let qualification =
            qualify_registration_structure(&registration, "verifier-1").expect("qualification");
        assert!(qualification.verify_digest().expect("digest verification"));

        let mut tampered = qualification;
        tampered.outcome = StructuralQualificationOutcome::Rejected;
        assert!(!tampered.verify_digest().expect("tampered digest verification"));
    }

    #[test]
    fn qualification_binds_registration_digest() {
        let a = envelope(AlignmentMethod::ExactCorrelationId);
        let mut b = a.clone();
        b.related[0].source_data_digest = digest('d');
        let qa = qualify_registration_structure(&a, "verifier-1").expect("qa");
        let qb = qualify_registration_structure(&b, "verifier-1").expect("qb");
        assert_ne!(qa.registration_digest, qb.registration_digest);
        assert_ne!(qa.qualification_digest, qb.qualification_digest);
    }

    #[test]
    fn verifier_identity_is_part_of_qualification() {
        let registration = envelope(AlignmentMethod::ExactCorrelationId);
        let a = qualify_registration_structure(&registration, "verifier-a").expect("a");
        let b = qualify_registration_structure(&registration, "verifier-b").expect("b");
        assert_ne!(a.qualification_digest, b.qualification_digest);
    }
}
