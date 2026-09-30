//! AeroCommons evidence primitives.
//!
//! This crate intentionally models evidence and provenance without making
//! certification or airworthiness claims. It is the typed boundary between
//! engineering records and the Mycelix provenance layer.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

/// The epistemic origin of an evidence record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimKind {
    Measurement,
    Analysis,
    Simulation,
    Inspection,
    Test,
    Calculation,
    Qualification,
    Attestation,
    Observation,
    Derived,
}

/// Lifecycle state of an evidence record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceLifecycle {
    Active,
    Superseded,
    Disputed,
    Revoked,
    Withdrawn,
}

/// A reference to another artifact without embedding its payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArtifactRef {
    pub id: String,
    pub kind: String,
}

/// A bounded uncertainty statement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Uncertainty {
    pub representation: String,
    pub value: Option<f64>,
    pub unit: Option<String>,
}

/// Conditions under which an evidence record is intended to apply.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ValidityDomain {
    #[serde(default)]
    pub conditions: BTreeMap<String, String>,
}

/// Explicit lifecycle transition metadata.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LifecycleEvent {
    pub state: EvidenceLifecycle,
    pub reason: Option<String>,
    pub predecessor: Option<String>,
}

/// Minimal typed AeroEvidence record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AeroEvidenceV1 {
    pub evidence_id: String,
    pub schema: String,
    pub claim: String,
    pub claim_kind: ClaimKind,
    pub configuration: ArtifactRef,
    pub subject: ArtifactRef,
    #[serde(default)]
    pub basis: Vec<ArtifactRef>,
    pub method: Option<ArtifactRef>,
    #[serde(default)]
    pub inputs: Vec<ArtifactRef>,
    #[serde(default)]
    pub toolchain: Vec<ArtifactRef>,
    pub result: serde_json::Value,
    pub uncertainty: Option<Uncertainty>,
    pub validity_domain: ValidityDomain,
    pub producer: String,
    #[serde(default)]
    pub review: Vec<ArtifactRef>,
    pub lifecycle: LifecycleEvent,
    #[serde(default)]
    pub provenance: Vec<ArtifactRef>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum EvidenceValidationError {
    #[error("schema must be AeroEvidenceV1")]
    WrongSchema,
    #[error("evidence_id must not be empty")]
    EmptyEvidenceId,
    #[error("claim must not be empty")]
    EmptyClaim,
    #[error("producer must not be empty")]
    EmptyProducer,
    #[error("configuration id must not be empty")]
    EmptyConfiguration,
    #[error("subject id must not be empty")]
    EmptySubject,
    #[error("derived evidence requires at least one basis record")]
    DerivedWithoutBasis,
    #[error("superseded evidence requires a predecessor")]
    SupersededWithoutPredecessor,
}

/// Lightweight structural validation.
///
/// This deliberately does not infer engineering validity. It only checks
/// invariants that are safe to enforce at the data-contract boundary.
pub fn validate_evidence(
    evidence: &AeroEvidenceV1,
) -> Result<(), EvidenceValidationError> {
    if evidence.schema != "AeroEvidenceV1" {
        return Err(EvidenceValidationError::WrongSchema);
    }
    if evidence.evidence_id.is_empty() {
        return Err(EvidenceValidationError::EmptyEvidenceId);
    }
    if evidence.claim.trim().is_empty() {
        return Err(EvidenceValidationError::EmptyClaim);
    }
    if evidence.producer.is_empty() {
        return Err(EvidenceValidationError::EmptyProducer);
    }
    if evidence.configuration.id.is_empty() {
        return Err(EvidenceValidationError::EmptyConfiguration);
    }
    if evidence.subject.id.is_empty() {
        return Err(EvidenceValidationError::EmptySubject);
    }
    if evidence.claim_kind == ClaimKind::Derived && evidence.basis.is_empty() {
        return Err(EvidenceValidationError::DerivedWithoutBasis);
    }
    if evidence.lifecycle.state == EvidenceLifecycle::Superseded
        && evidence.lifecycle.predecessor.is_none()
    {
        return Err(EvidenceValidationError::SupersededWithoutPredecessor);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> AeroEvidenceV1 {
        AeroEvidenceV1 {
            evidence_id: "evidence-001".into(),
            schema: "AeroEvidenceV1".into(),
            claim: "measured diameter is 10.02 mm".into(),
            claim_kind: ClaimKind::Measurement,
            configuration: ArtifactRef {
                id: "fixture-v1".into(),
                kind: "configuration".into(),
            },
            subject: ArtifactRef {
                id: "fixture".into(),
                kind: "component".into(),
            },
            basis: vec![],
            method: Some(ArtifactRef {
                id: "caliper-procedure-v1".into(),
                kind: "method".into(),
            }),
            inputs: vec![],
            toolchain: vec![],
            result: serde_json::json!({"value": 10.02, "unit": "mm"}),
            uncertainty: Some(Uncertainty {
                representation: "absolute".into(),
                value: Some(0.02),
                unit: Some("mm".into()),
            }),
            validity_domain: ValidityDomain::default(),
            producer: "agent-a".into(),
            review: vec![],
            lifecycle: LifecycleEvent {
                state: EvidenceLifecycle::Active,
                reason: None,
                predecessor: None,
            },
            provenance: vec![],
        }
    }

    #[test]
    fn valid_measurement_passes_structural_validation() {
        assert_eq!(validate_evidence(&fixture()), Ok(()));
    }

    #[test]
    fn derived_evidence_requires_basis() {
        let mut value = fixture();
        value.claim_kind = ClaimKind::Derived;
        assert_eq!(
            validate_evidence(&value),
            Err(EvidenceValidationError::DerivedWithoutBasis)
        );
    }

    #[test]
    fn supersession_requires_predecessor() {
        let mut value = fixture();
        value.lifecycle.state = EvidenceLifecycle::Superseded;
        assert_eq!(
            validate_evidence(&value),
            Err(EvidenceValidationError::SupersededWithoutPredecessor)
        );
    }

    #[test]
    fn serde_round_trip_preserves_record() {
        let value = fixture();
        let encoded = serde_json::to_string(&value).expect("serialize");
        let decoded: AeroEvidenceV1 =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(value, decoded);
    }
}
