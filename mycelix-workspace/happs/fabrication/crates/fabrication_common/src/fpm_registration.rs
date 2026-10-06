//! Deterministic multimodal registration evidence for Fabrication Process Monitoring.
//!
//! This module describes registration evidence, not sensor truth. A Consistent
//! state means the supplied identity/alignment metadata is internally coherent;
//! it does not establish independent verification, sensor calibration correctness,
//! clock synchronization beyond the declared evidence, or a physical defect.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub const FPM_REGISTRATION_SCHEMA_VERSION: &str = "fpm.registration.v1";
const MAX_LABEL_BYTES: usize = 128;
const SHA256_HEX_LEN: usize = 64;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ModalityObservationRef {
    pub source_id: String,
    pub modality: String,
    pub clock_domain: String,
    pub source_sequence: u64,
    /// Producer-assigned correlation identifier shared by observations that
    /// are asserted to represent the same acquisition frame.
    pub correlation_domain: String,
    pub correlation_id: String,
    pub source_timestamp_micros: Option<u64>,
    pub calibration_profile_digest: String,
    pub process_context_digest: String,
    pub source_data_digest: String,
}

impl ModalityObservationRef {
    pub fn validate(&self) -> Result<(), RegistrationError> {
        validate_label(&self.source_id, "source_id")?;
        validate_label(&self.modality, "modality")?;
        validate_label(&self.clock_domain, "clock_domain")?;
        validate_label(&self.correlation_domain, "correlation_domain")?;
        validate_label(&self.correlation_id, "correlation_id")?;
        validate_digest(&self.calibration_profile_digest, "calibration_profile_digest")?;
        validate_digest(&self.process_context_digest, "process_context_digest")?;
        validate_digest(&self.source_data_digest, "source_data_digest")?;
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AlignmentMethod {
    ExactCorrelationId,
    ExactSourceTimestampMicros,
    DeclaredClockTransform { transform_digest: String },
    ExternalRegistrationEvidence { evidence_digest: String },
}

impl AlignmentMethod {
    fn validate(&self) -> Result<(), RegistrationError> {
        match self {
            Self::ExactCorrelationId | Self::ExactSourceTimestampMicros => Ok(()),
            Self::DeclaredClockTransform { transform_digest }
            | Self::ExternalRegistrationEvidence {
                evidence_digest: transform_digest,
            } => validate_digest(transform_digest, "alignment evidence digest"),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct RegistrationEnvelope {
    pub schema_version: String,
    pub reference: ModalityObservationRef,
    pub related: Vec<ModalityObservationRef>,
    pub alignment_method: Option<AlignmentMethod>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationState {
    /// Supplied registration metadata is internally consistent. This is not
    /// independent verification of the underlying sources or clocks.
    Consistent,
    Unregistered,
    Conflicting,
    Invalid,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistrationError {
    InvalidField(String),
}

impl std::fmt::Display for RegistrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidField(reason) => write!(f, "invalid FPM registration: {reason}"),
        }
    }
}

impl RegistrationEnvelope {
    /// Derive registration state only from the complete envelope.
    ///
    /// No state is stored as user-supplied input, preventing a serialized
    /// "Registered" label from bypassing the underlying evidence checks.
    pub fn assess(&self) -> RegistrationState {
        if self.schema_version != FPM_REGISTRATION_SCHEMA_VERSION {
            return RegistrationState::Invalid;
        }
        if self.related.is_empty() {
            return RegistrationState::Unregistered;
        }
        if self.reference.validate().is_err()
            || self.related.iter().any(|item| item.validate().is_err())
        {
            return RegistrationState::Invalid;
        }

        let participants = std::iter::once(&self.reference).chain(self.related.iter());
        let participants: Vec<&ModalityObservationRef> = participants.collect();

        for left in 0..participants.len() {
            for right in (left + 1)..participants.len() {
                let a = participants[left];
                let b = participants[right];
                if a.source_id == b.source_id && a.modality == b.modality {
                    return RegistrationState::Invalid;
                }
            }
        }

        let distinct_modalities = participants
            .iter()
            .map(|item| item.modality.as_str())
            .collect::<std::collections::BTreeSet<_>>();
        if distinct_modalities.len() < 2 {
            return RegistrationState::Unregistered;
        }

        let same_context = participants.iter().all(|item| {
            item.process_context_digest == self.reference.process_context_digest
        });
        let same_calibration = participants.clone().all(|item| {
            item.calibration_profile_digest == self.reference.calibration_profile_digest
        });
        if !same_context || !same_calibration {
            return RegistrationState::Conflicting;
        }

        let Some(method) = &self.alignment_method else {
            return RegistrationState::Unregistered;
        };
        if method.validate().is_err() {
            return RegistrationState::Invalid;
        }

        match method {
            AlignmentMethod::ExactCorrelationId => {
                if participants.clone().all(|item| {
                    item.correlation_domain == self.reference.correlation_domain
                        && item.correlation_id == self.reference.correlation_id
                }) {
                    RegistrationState::Consistent
                } else {
                    RegistrationState::Conflicting
                }
            }
            AlignmentMethod::ExactSourceTimestampMicros => {
                let Some(reference_ts) = self.reference.source_timestamp_micros else {
                    return RegistrationState::Unknown;
                };
                if participants.clone().all(|item| {
                    item.clock_domain == self.reference.clock_domain
                        && item.source_timestamp_micros == Some(reference_ts)
                }) {
                    RegistrationState::Consistent
                } else {
                    RegistrationState::Conflicting
                }
            }
            AlignmentMethod::DeclaredClockTransform { .. }
            | AlignmentMethod::ExternalRegistrationEvidence { .. } => {
                // A commitment to an external artifact is not the same as
                // independently validating that artifact. Keep this state
                // unknown until a separate verifier consumes the evidence.
                RegistrationState::Unknown
            }
        }
    }

    pub fn validate_for_use(&self) -> Result<(), RegistrationError> {
        match self.assess() {
            RegistrationState::Consistent => Ok(()),
            RegistrationState::Unregistered => Err(RegistrationError::InvalidField(
                "registration evidence is absent".into(),
            )),
            RegistrationState::Conflicting => Err(RegistrationError::InvalidField(
                "registration participants disagree on context, calibration, or alignment".into(),
            )),
            RegistrationState::Invalid => Err(RegistrationError::InvalidField(
                "registration envelope is malformed or unsupported".into(),
            )),
            RegistrationState::Unknown => Err(RegistrationError::InvalidField(
                "registration evidence is insufficient to establish alignment".into(),
            )),
        }
    }

    pub fn digest(&self) -> Result<String, RegistrationError> {
        let bytes = serde_json::to_vec(self).map_err(|e| {
            RegistrationError::InvalidField(format!("failed to serialize registration: {e}"))
        })?;
        Ok(hex_digest(&bytes))
    }
}

fn validate_label(value: &str, field: &str) -> Result<(), RegistrationError> {
    if value.trim().is_empty() {
        return Err(RegistrationError::InvalidField(format!(
            "{field} cannot be empty"
        )));
    }
    if value.len() > MAX_LABEL_BYTES {
        return Err(RegistrationError::InvalidField(format!(
            "{field} cannot exceed {MAX_LABEL_BYTES} bytes"
        )));
    }
    Ok(())
}

fn validate_digest(value: &str, field: &str) -> Result<(), RegistrationError> {
    if value.len() != SHA256_HEX_LEN || !value.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(RegistrationError::InvalidField(format!(
            "{field} must be a {SHA256_HEX_LEN}-character hexadecimal SHA-256 digest"
        )));
    }
    Ok(())
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat(ch).take(SHA256_HEX_LEN).collect()
    }

    fn sample(source: &str, sequence: u64) -> ModalityObservationRef {
        ModalityObservationRef {
            source_id: source.into(),
            modality: "thermal".into(),
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

    fn registered(method: AlignmentMethod) -> RegistrationEnvelope {
        RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: sample("thermal-1", 10),
            related: vec![ModalityObservationRef {
                modality: "vibration".into(),
                ..sample("vibration-1", 10)
            }],
            alignment_method: Some(method),
        }
    }

    #[test]
    fn exact_correlation_id_requires_shared_frame_identity() {
        assert_eq!(
            registered(AlignmentMethod::ExactCorrelationId).assess(),
            RegistrationState::Consistent
        );

        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related[0].correlation_id = "frame-11".into();
        assert_eq!(envelope.assess(), RegistrationState::Conflicting);

        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related[0].source_sequence = 11;
        assert_eq!(envelope.assess(), RegistrationState::Consistent);

        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related[0].clock_domain = "local-clock-2".into();
        assert_eq!(envelope.assess(), RegistrationState::Consistent);
    }

    #[test]
    fn exact_timestamp_requires_present_equal_source_timestamps() {
        assert_eq!(
            registered(AlignmentMethod::ExactSourceTimestampMicros).assess(),
            RegistrationState::Consistent
        );

        let mut envelope = registered(AlignmentMethod::ExactSourceTimestampMicros);
        envelope.related[0].source_timestamp_micros = Some(1_000_001);
        assert_eq!(envelope.assess(), RegistrationState::Conflicting);

        let mut envelope = registered(AlignmentMethod::ExactSourceTimestampMicros);
        envelope.reference.source_timestamp_micros = None;
        assert_eq!(envelope.assess(), RegistrationState::Unknown);
    }

    #[test]
    fn missing_alignment_is_unregistered() {
        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.alignment_method = None;
        assert_eq!(envelope.assess(), RegistrationState::Unregistered);
        assert!(envelope.validate_for_use().is_err());
    }

    #[test]
    fn conflicting_context_cannot_register() {
        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related[0].process_context_digest = digest('d');
        assert_eq!(envelope.assess(), RegistrationState::Conflicting);
    }

    #[test]
    fn conflicting_calibration_cannot_register() {
        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related[0].calibration_profile_digest = digest('d');
        assert_eq!(envelope.assess(), RegistrationState::Conflicting);
    }

    #[test]
    fn declared_transform_requires_commitment() {
        let method = AlignmentMethod::DeclaredClockTransform {
            transform_digest: digest('d'),
        };
        assert_eq!(registered(method).assess(), RegistrationState::Unknown);

        let invalid = AlignmentMethod::DeclaredClockTransform {
            transform_digest: "bad".into(),
        };
        assert_eq!(registered(invalid).assess(), RegistrationState::Invalid);

        let external = AlignmentMethod::ExternalRegistrationEvidence {
            evidence_digest: digest('e'),
        };
        assert_eq!(registered(external).assess(), RegistrationState::Unknown);
    }

    #[test]
    fn invalid_schema_is_invalid_not_unknown() {
        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.schema_version = "fpm.registration.v0".into();
        assert_eq!(envelope.assess(), RegistrationState::Invalid);
    }

    #[test]
    fn empty_related_set_is_unregistered() {
        let mut envelope = registered(AlignmentMethod::ExactCorrelationId);
        envelope.related.clear();
        assert_eq!(envelope.assess(), RegistrationState::Unregistered);
    }

    #[test]
    fn digest_changes_when_registration_changes() {
        let a = registered(AlignmentMethod::ExactCorrelationId);
        let mut b = a.clone();
        b.related[0].source_sequence = 11;
        assert_ne!(a.digest().expect("digest a"), b.digest().expect("digest b"));
    }
}
