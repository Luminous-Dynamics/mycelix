// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Authority-free attestation envelopes for planetary evidence.
//!
//! PEF-1 says what an environmental observation is. PEF-2 says how a
//! computational product was produced. This module answers a third, separate
//! question: *who is making a claim about that evidence, and what exact immutable
//! subject does the claim bind to?*
//!
//! An attestation is not authority by itself. A valid signature proves control
//! of a verification method, not regulatory standing, scientific correctness,
//! or permission to act. Authority/credential evidence therefore remains an
//! explicit, separate field for policy layers to interpret.
//!
//! This crate deliberately does not implement cryptographic verification or a
//! canonical serializer. `SignedEvidenceAttestation` carries the payload digest,
//! payload-encoding identifier, verification method, algorithm identifier and
//! detached signature required by a verifier, but `validate()` only checks the
//! structural contract.

use crate::ExternalEvidenceRef;
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const PLANETARY_ATTESTATION_SCHEMA_VERSION: u16 = 1;
pub const MAX_ATTESTATION_ID_BYTES: usize = 256;
pub const MAX_ATTESTOR_DID_BYTES: usize = 512;
pub const MAX_VERIFICATION_METHOD_BYTES: usize = 768;
pub const MAX_ENCODING_ID_BYTES: usize = 128;
pub const MAX_ALGORITHM_ID_BYTES: usize = 128;
pub const MAX_ATTESTATION_DIGEST_BYTES: usize = 256;
pub const MAX_ATTESTATION_SCOPES: usize = 32;
pub const MAX_ATTESTATION_EVIDENCE_REFS: usize = 64;
pub const MAX_SIGNATURE_BYTES: usize = 32 * 1024;
pub const MAX_CUSTOM_SCOPE_BYTES: usize = 256;

/// Immutable subject of an attestation.
///
/// Canonical observation/lineage IDs make the subject human-addressable while
/// digests bind the claim to one exact representation. A subject with only a
/// mutable URL or bare identifier would permit a valid signature to be replayed
/// against changed content, so every subject variant requires a content digest.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AttestationSubject {
    Observation {
        observation_id: String,
        observation_digest: String,
    },
    Lineage {
        output_observation_id: String,
        lineage_digest: String,
    },
    Product {
        observation_id: String,
        observation_digest: String,
        lineage_digest: String,
    },
    ExternalEvidence(ExternalEvidenceRef),
}

impl AttestationSubject {
    pub fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        match self {
            Self::Observation {
                observation_id,
                observation_digest,
            } => {
                require_text(
                    "attestation.subject.observation_id",
                    observation_id,
                    MAX_ATTESTATION_ID_BYTES,
                )?;
                validate_digest(
                    "attestation.subject.observation_digest",
                    observation_digest,
                )
            }
            Self::Lineage {
                output_observation_id,
                lineage_digest,
            } => {
                require_text(
                    "attestation.subject.output_observation_id",
                    output_observation_id,
                    MAX_ATTESTATION_ID_BYTES,
                )?;
                validate_digest("attestation.subject.lineage_digest", lineage_digest)
            }
            Self::Product {
                observation_id,
                observation_digest,
                lineage_digest,
            } => {
                require_text(
                    "attestation.subject.observation_id",
                    observation_id,
                    MAX_ATTESTATION_ID_BYTES,
                )?;
                validate_digest(
                    "attestation.subject.observation_digest",
                    observation_digest,
                )?;
                validate_digest("attestation.subject.lineage_digest", lineage_digest)
            }
            Self::ExternalEvidence(reference) => {
                reference.validate().map_err(|error| {
                    PlanetaryAttestationError::InvalidExternalEvidence(error.to_string())
                })?;
                if reference.content_digest.is_none() {
                    return Err(PlanetaryAttestationError::UnboundExternalSubject);
                }
                Ok(())
            }
        }
    }

    /// Canonical output observation identity, when this subject names one.
    pub fn observation_id(&self) -> Option<&str> {
        match self {
            Self::Observation { observation_id, .. }
            | Self::Product { observation_id, .. } => Some(observation_id),
            Self::Lineage {
                output_observation_id,
                ..
            } => Some(output_observation_id),
            Self::ExternalEvidence(_) => None,
        }
    }
}

/// Identity that made an attestation.
///
/// `verification_method` is intentionally distinct from the DID. The DID names
/// the claimed controller while the verification method identifies the exact key
/// (or other method) that a cryptographic verifier must resolve.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AttestorIdentity {
    pub did: String,
    pub verification_method: String,
}

impl AttestorIdentity {
    pub fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        require_text("attestation.attestor.did", &self.did, MAX_ATTESTOR_DID_BYTES)?;
        if !self.did.starts_with("did:") {
            return Err(PlanetaryAttestationError::MalformedField {
                field: "attestation.attestor.did",
                reason: "attestor identity must be a DID",
            });
        }
        require_text(
            "attestation.attestor.verification_method",
            &self.verification_method,
            MAX_VERIFICATION_METHOD_BYTES,
        )?;
        if !self.verification_method.starts_with(&self.did) {
            return Err(PlanetaryAttestationError::VerificationMethodControllerMismatch);
        }
        let suffix = &self.verification_method[self.did.len()..];
        if suffix.is_empty()
            || !matches!(suffix.as_bytes().first(), Some(b'#' | b'/' | b'?' | b';'))
        {
            return Err(PlanetaryAttestationError::VerificationMethodControllerMismatch);
        }
        Ok(())
    }
}

/// What aspect of the subject the attestor is addressing.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AttestationScope {
    SourceAuthenticity,
    MeasurementIntegrity,
    Methodology,
    Reproducibility,
    ScientificReview,
    OperationalVerification,
    RegulatoryCompliance,
    CommunityWitness,
    Custom(String),
}

impl AttestationScope {
    fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        if let Self::Custom(value) = self {
            require_text(
                "attestation.scope.custom",
                value,
                MAX_CUSTOM_SCOPE_BYTES,
            )?;
        }
        Ok(())
    }
}

/// Semantic position taken by an attestation.
///
/// Signature validity and semantic stance are deliberately orthogonal: a
/// cryptographically valid signature can still contain a dispute or an
/// inconclusive assessment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AttestationStance {
    Affirms,
    Qualifies,
    Disputes,
    Inconclusive,
}

/// Unsigned semantic attestation payload.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EvidenceAttestation {
    pub schema_version: u16,
    pub id: String,
    pub subject: AttestationSubject,
    pub attestor: AttestorIdentity,
    pub scopes: Vec<AttestationScope>,
    pub stance: AttestationStance,
    /// Unix seconds when the attestation was issued.
    pub issued_at: i64,
    /// Optional Unix-second expiry. An expired attestation is not deleted; it
    /// remains historical evidence whose current applicability must be judged.
    pub valid_until: Option<i64>,
    /// Digest of a detailed human/machine-readable statement stored elsewhere.
    pub statement_digest: Option<String>,
    /// Evidence supporting the substantive claim made by the attestor.
    pub supporting_evidence: Vec<ExternalEvidenceRef>,
    /// Evidence supporting the attestor's role/authority/credential, if any.
    /// Empty means no external authority claim is supplied, not that authority
    /// should be assumed from the DID.
    pub authority_evidence: Vec<ExternalEvidenceRef>,
}

impl EvidenceAttestation {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        subject: AttestationSubject,
        attestor: AttestorIdentity,
        scopes: Vec<AttestationScope>,
        stance: AttestationStance,
        issued_at: i64,
        valid_until: Option<i64>,
    ) -> Result<Self, PlanetaryAttestationError> {
        let attestation = Self {
            schema_version: PLANETARY_ATTESTATION_SCHEMA_VERSION,
            id: id.into(),
            subject,
            attestor,
            scopes,
            stance,
            issued_at,
            valid_until,
            statement_digest: None,
            supporting_evidence: Vec::new(),
            authority_evidence: Vec::new(),
        };
        attestation.validate()?;
        Ok(attestation)
    }

    pub fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        if self.schema_version != PLANETARY_ATTESTATION_SCHEMA_VERSION {
            return Err(PlanetaryAttestationError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("attestation.id", &self.id, MAX_ATTESTATION_ID_BYTES)?;
        self.subject.validate()?;
        self.attestor.validate()?;

        if self.scopes.is_empty() {
            return Err(PlanetaryAttestationError::MissingScopes);
        }
        if self.scopes.len() > MAX_ATTESTATION_SCOPES {
            return Err(PlanetaryAttestationError::TooManyScopes {
                actual: self.scopes.len(),
                max: MAX_ATTESTATION_SCOPES,
            });
        }
        let mut scopes = HashSet::with_capacity(self.scopes.len());
        for scope in &self.scopes {
            scope.validate()?;
            if !scopes.insert(scope) {
                return Err(PlanetaryAttestationError::DuplicateScope);
            }
        }

        if let Some(valid_until) = self.valid_until
            && valid_until < self.issued_at
        {
            return Err(PlanetaryAttestationError::ExpiryBeforeIssue {
                issued_at: self.issued_at,
                valid_until,
            });
        }
        if let Some(statement_digest) = &self.statement_digest {
            validate_digest("attestation.statement_digest", statement_digest)?;
        }

        validate_evidence_refs(
            "supporting",
            &self.supporting_evidence,
            MAX_ATTESTATION_EVIDENCE_REFS,
        )?;
        validate_evidence_refs(
            "authority",
            &self.authority_evidence,
            MAX_ATTESTATION_EVIDENCE_REFS,
        )?;
        Ok(())
    }

    pub fn is_expired_at(&self, unix_seconds: i64) -> bool {
        self.valid_until
            .is_some_and(|valid_until| unix_seconds > valid_until)
    }

    /// Whether this payload explicitly carries evidence for an authority or
    /// credential claim. This does not validate the credential itself.
    pub fn declares_authority_evidence(&self) -> bool {
        !self.authority_evidence.is_empty()
    }
}

/// Detached cryptographic signature bytes plus algorithm identifier.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DetachedAttestationSignature {
    pub algorithm: String,
    pub bytes: Vec<u8>,
}

impl DetachedAttestationSignature {
    pub fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        require_text(
            "attestation.signature.algorithm",
            &self.algorithm,
            MAX_ALGORITHM_ID_BYTES,
        )?;
        if self.bytes.is_empty() {
            return Err(PlanetaryAttestationError::EmptySignature);
        }
        if self.bytes.len() > MAX_SIGNATURE_BYTES {
            return Err(PlanetaryAttestationError::SignatureTooLarge {
                actual: self.bytes.len(),
                max: MAX_SIGNATURE_BYTES,
            });
        }
        Ok(())
    }
}

/// Signed transport envelope.
///
/// The detached signature is expected to authenticate `payload_digest` using
/// the attestor's declared verification method. `payload_encoding` identifies
/// the canonicalization/encoding used to produce that digest. This type checks
/// shape only; callers must recompute the digest and verify the signature.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SignedEvidenceAttestation {
    pub payload: EvidenceAttestation,
    pub payload_encoding: String,
    pub payload_digest: String,
    pub signature: DetachedAttestationSignature,
}

impl SignedEvidenceAttestation {
    pub fn validate(&self) -> Result<(), PlanetaryAttestationError> {
        self.payload.validate()?;
        require_text(
            "attestation.payload_encoding",
            &self.payload_encoding,
            MAX_ENCODING_ID_BYTES,
        )?;
        validate_digest("attestation.payload_digest", &self.payload_digest)?;
        self.signature.validate()
    }
}

/// Result of an external cryptographic/credential evaluation.
///
/// This is intentionally not embedded into the signed payload: verification
/// results are contextual observations made by a verifier at a point in time.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AttestationVerificationState {
    Unchecked,
    SignatureValid,
    SignatureInvalid,
    UnsupportedAlgorithm,
    VerificationMethodUnavailable,
    Revoked { notice_id: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlanetaryAttestationError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    MalformedField {
        field: &'static str,
        reason: &'static str,
    },
    MalformedDigest {
        field: &'static str,
        reason: &'static str,
    },
    InvalidExternalEvidence(String),
    UnboundExternalSubject,
    VerificationMethodControllerMismatch,
    MissingScopes,
    TooManyScopes {
        actual: usize,
        max: usize,
    },
    DuplicateScope,
    ExpiryBeforeIssue {
        issued_at: i64,
        valid_until: i64,
    },
    TooManyEvidenceRefs {
        class: &'static str,
        actual: usize,
        max: usize,
    },
    DuplicateEvidenceRef {
        class: &'static str,
    },
    EmptySignature,
    SignatureTooLarge {
        actual: usize,
        max: usize,
    },
}

impl fmt::Display for PlanetaryAttestationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported planetary attestation schema version {version}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::MalformedField { field, reason } => write!(f, "malformed {field}: {reason}"),
            Self::MalformedDigest { field, reason } => {
                write!(f, "malformed {field}: {reason}")
            }
            Self::InvalidExternalEvidence(error) => {
                write!(f, "invalid attestation evidence reference: {error}")
            }
            Self::UnboundExternalSubject => write!(
                f,
                "external-evidence attestation subject requires a content digest"
            ),
            Self::VerificationMethodControllerMismatch => write!(
                f,
                "verification method must be controlled by the attestor DID"
            ),
            Self::MissingScopes => write!(f, "attestation must declare at least one scope"),
            Self::TooManyScopes { actual, max } => {
                write!(f, "attestation has {actual} scopes; maximum is {max}")
            }
            Self::DuplicateScope => write!(f, "attestation contains a duplicate scope"),
            Self::ExpiryBeforeIssue {
                issued_at,
                valid_until,
            } => write!(
                f,
                "attestation expiry {valid_until} precedes issue time {issued_at}"
            ),
            Self::TooManyEvidenceRefs { class, actual, max } => write!(
                f,
                "attestation has {actual} {class} evidence refs; maximum is {max}"
            ),
            Self::DuplicateEvidenceRef { class } => {
                write!(f, "attestation contains duplicate {class} evidence references")
            }
            Self::EmptySignature => write!(f, "attestation signature cannot be empty"),
            Self::SignatureTooLarge { actual, max } => {
                write!(f, "attestation signature is {actual} bytes; maximum is {max}")
            }
        }
    }
}

impl std::error::Error for PlanetaryAttestationError {}

fn require_text(
    field: &'static str,
    value: &str,
    max_bytes: usize,
) -> Result<(), PlanetaryAttestationError> {
    if value.trim().is_empty() {
        return Err(PlanetaryAttestationError::EmptyField(field));
    }
    if value.len() > max_bytes {
        return Err(PlanetaryAttestationError::FieldTooLong {
            field,
            actual: value.len(),
            max: max_bytes,
        });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), PlanetaryAttestationError> {
    require_text(field, digest, MAX_ATTESTATION_DIGEST_BYTES)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(PlanetaryAttestationError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(PlanetaryAttestationError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(PlanetaryAttestationError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

fn validate_evidence_refs(
    class: &'static str,
    refs: &[ExternalEvidenceRef],
    max: usize,
) -> Result<(), PlanetaryAttestationError> {
    if refs.len() > max {
        return Err(PlanetaryAttestationError::TooManyEvidenceRefs {
            class,
            actual: refs.len(),
            max,
        });
    }
    let mut unique = HashSet::with_capacity(refs.len());
    for reference in refs {
        reference.validate().map_err(|error| {
            PlanetaryAttestationError::InvalidExternalEvidence(error.to_string())
        })?;
        if !unique.insert(reference) {
            return Err(PlanetaryAttestationError::DuplicateEvidenceRef { class });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(resource: &str, digest: Option<&str>) -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "example".into(),
            resource_id: resource.into(),
            content_digest: digest.map(str::to_string),
            retrieved_at: None,
            license: None,
        }
    }

    fn attestor() -> AttestorIdentity {
        AttestorIdentity {
            did: "did:key:z6Mktest".into(),
            verification_method: "did:key:z6Mktest#z6Mktest".into(),
        }
    }

    fn subject() -> AttestationSubject {
        AttestationSubject::Product {
            observation_id: "atlas:heat-risk:jhb:1".into(),
            observation_digest: "sha256:observation".into(),
            lineage_digest: "sha256:lineage".into(),
        }
    }

    fn payload() -> EvidenceAttestation {
        EvidenceAttestation::new(
            "att:1",
            subject(),
            attestor(),
            vec![
                AttestationScope::Methodology,
                AttestationScope::Reproducibility,
            ],
            AttestationStance::Affirms,
            1_788_825_600,
            Some(1_791_417_600),
        )
        .unwrap()
    }

    #[test]
    fn accepts_digest_bound_product_attestation() {
        let mut value = payload();
        value.supporting_evidence = vec![evidence("review/1", Some("sha256:review"))];
        value.authority_evidence = vec![evidence("credential/1", Some("sha256:credential"))];
        value.validate().unwrap();
        assert!(value.declares_authority_evidence());
    }

    #[test]
    fn rejects_external_subject_without_content_digest() {
        let mut value = payload();
        value.subject = AttestationSubject::ExternalEvidence(evidence("mutable/url", None));
        assert_eq!(
            value.validate(),
            Err(PlanetaryAttestationError::UnboundExternalSubject)
        );
    }

    #[test]
    fn rejects_verification_method_for_different_controller() {
        let mut value = payload();
        value.attestor.verification_method = "did:key:z6Mkother#key-1".into();
        assert_eq!(
            value.validate(),
            Err(PlanetaryAttestationError::VerificationMethodControllerMismatch)
        );
    }

    #[test]
    fn rejects_duplicate_scopes() {
        let mut value = payload();
        value.scopes = vec![
            AttestationScope::ScientificReview,
            AttestationScope::ScientificReview,
        ];
        assert_eq!(value.validate(), Err(PlanetaryAttestationError::DuplicateScope));
    }

    #[test]
    fn rejects_expiry_before_issue() {
        let mut value = payload();
        value.valid_until = Some(value.issued_at - 1);
        assert!(matches!(
            value.validate(),
            Err(PlanetaryAttestationError::ExpiryBeforeIssue { .. })
        ));
    }

    #[test]
    fn distinguishes_expiry_from_historical_validity() {
        let value = payload();
        assert!(!value.is_expired_at(value.issued_at));
        assert!(value.is_expired_at(value.valid_until.unwrap() + 1));
    }

    #[test]
    fn signed_envelope_requires_structural_signature_contract() {
        let signed = SignedEvidenceAttestation {
            payload: payload(),
            payload_encoding: "mycelix-jcs-v1".into(),
            payload_digest: "sha256:payload".into(),
            signature: DetachedAttestationSignature {
                algorithm: "ed25519".into(),
                bytes: vec![7; 64],
            },
        };
        signed.validate().unwrap();
    }

    #[test]
    fn rejects_empty_signature() {
        let signed = SignedEvidenceAttestation {
            payload: payload(),
            payload_encoding: "mycelix-jcs-v1".into(),
            payload_digest: "sha256:payload".into(),
            signature: DetachedAttestationSignature {
                algorithm: "ed25519".into(),
                bytes: Vec::new(),
            },
        };
        assert_eq!(signed.validate(), Err(PlanetaryAttestationError::EmptySignature));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_attestation() {
        let signed = SignedEvidenceAttestation {
            payload: payload(),
            payload_encoding: "mycelix-jcs-v1".into(),
            payload_digest: "sha256:payload".into(),
            signature: DetachedAttestationSignature {
                algorithm: "ml-dsa-65".into(),
                bytes: vec![1, 2, 3, 4],
            },
        };
        let encoded = serde_json::to_string(&signed).unwrap();
        let decoded: SignedEvidenceAttestation = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, signed);
    }
}
