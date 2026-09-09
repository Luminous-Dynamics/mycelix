// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound authority evaluation for planetary response actions.
//!
//! This module deliberately does **not** mint capabilities, grants, signatures,
//! or actuator commands. It records the result of evaluating immutable evidence
//! against one authority requirement declared by a response option.
//!
//! ```text
//! ResponseProposal::AuthorityRequirement
//!                  ↓
//! immutable authority / policy / identity evidence
//!                  ↓
//! AuthorityEvaluationRecord
//!       Allowed / Denied / Indeterminate
//!                  ↓
//! separately authenticated + trusted by execution boundary
//!                  ↓
//! capability / execution system outside this module
//! ```
//!
//! An `Allowed` record is therefore evidence *about* authority, never authority
//! itself. Consumers still need to authenticate the evaluator, verify referenced
//! evidence and revocation state, and enforce the actual execution capability.

use crate::{AuthorityRequirement, ExternalEvidenceRef, ResponseProposal};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub const AUTHORITY_EVALUATION_SCHEMA_VERSION: u16 = 1;
pub const MAX_AUTHORITY_ID_BYTES: usize = 256;
pub const MAX_AUTHORITY_LABEL_BYTES: usize = 256;
pub const MAX_AUTHORITY_EVIDENCE: usize = 128;
pub const MAX_AUTHORITY_QUALIFICATIONS: usize = 64;

/// Exact response-option authority requirement being evaluated.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ResponseAuthorityRef {
    pub proposal_id: String,
    pub proposal_digest: String,
    pub option_id: String,
    pub requirement: AuthorityRequirement,
}

impl ResponseAuthorityRef {
    pub fn validate(&self) -> Result<(), AuthorityEvaluationError> {
        require_text(
            "authority.response.proposal_id",
            &self.proposal_id,
            MAX_AUTHORITY_ID_BYTES,
        )?;
        require_text(
            "authority.response.option_id",
            &self.option_id,
            MAX_AUTHORITY_ID_BYTES,
        )?;
        validate_digest(
            "authority.response.proposal_digest",
            &self.proposal_digest,
        )?;
        self.requirement
            .validate()
            .map_err(|error| AuthorityEvaluationError::InvalidRequirement(error.to_string()))
    }

    /// Check that this reference names an authority requirement actually present
    /// on the exact response option supplied by the caller.
    ///
    /// Digest recomputation is intentionally outside this dependency-light crate.
    pub fn validate_against_proposal(
        &self,
        proposal: &ResponseProposal,
    ) -> Result<(), AuthorityEvaluationError> {
        self.validate()?;
        proposal
            .validate()
            .map_err(|error| AuthorityEvaluationError::InvalidProposal(error.to_string()))?;
        if self.proposal_id != proposal.id {
            return Err(AuthorityEvaluationError::ProposalIdMismatch);
        }
        let option = proposal
            .options
            .iter()
            .find(|option| option.id == self.option_id)
            .ok_or(AuthorityEvaluationError::UnknownResponseOption)?;
        if !option.authority_requirements.contains(&self.requirement) {
            return Err(AuthorityEvaluationError::RequirementNotOnOption);
        }
        Ok(())
    }
}

/// Semantic role played by an immutable authority evidence artifact.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AuthorityEvidenceKind {
    Grant,
    Delegation,
    Credential,
    Ownership,
    Consent,
    Policy,
    Statute,
    EmergencyDeclaration,
    IdentityControl,
    RevocationStatus,
    Custom(String),
}

impl AuthorityEvidenceKind {
    fn validate(&self) -> Result<(), AuthorityEvaluationError> {
        if let Self::Custom(value) = self {
            require_text(
                "authority.evidence.kind.custom",
                value,
                MAX_AUTHORITY_LABEL_BYTES,
            )?;
        }
        Ok(())
    }
}

/// One immutable artifact used to support an authority evaluation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AuthorityEvidenceRef {
    pub kind: AuthorityEvidenceKind,
    pub reference: ExternalEvidenceRef,
}

impl AuthorityEvidenceRef {
    pub fn validate(&self) -> Result<(), AuthorityEvaluationError> {
        self.kind.validate()?;
        self.reference
            .validate()
            .map_err(|error| AuthorityEvaluationError::InvalidEvidence(error.to_string()))?;
        if self.reference.content_digest.is_none() {
            return Err(AuthorityEvaluationError::MutableAuthorityEvidence(
                self.reference.resource_id.clone(),
            ));
        }
        Ok(())
    }
}

/// Result of applying an explicit authority-evaluation procedure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AuthorityEvaluationResult {
    Allowed,
    Denied,
    Indeterminate,
    NotApplicable,
}

/// Evidence that the evaluator is qualified/authorized to perform this class of
/// evaluation. These are references only; this type does not verify them.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EvaluatorQualificationRef {
    pub role: String,
    pub reference: ExternalEvidenceRef,
}

impl EvaluatorQualificationRef {
    pub fn validate(&self) -> Result<(), AuthorityEvaluationError> {
        require_text(
            "authority.evaluator.qualification.role",
            &self.role,
            MAX_AUTHORITY_LABEL_BYTES,
        )?;
        self.reference
            .validate()
            .map_err(|error| AuthorityEvaluationError::InvalidQualification(error.to_string()))?;
        if self.reference.content_digest.is_none() {
            return Err(AuthorityEvaluationError::MutableEvaluatorQualification(
                self.reference.resource_id.clone(),
            ));
        }
        Ok(())
    }
}

/// Semantic authority-evaluation record.
///
/// Authenticity/signature verification is deliberately external so the record
/// does not confuse `result == Allowed` with a cryptographically trusted result.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AuthorityEvaluationRecord {
    pub schema_version: u16,
    pub id: String,
    pub response_authority: ResponseAuthorityRef,
    /// DID of the principal that would exercise the authority at execution.
    pub subject_did: String,
    /// DID claiming responsibility for this evaluation.
    pub evaluator_did: String,
    pub evaluator_qualifications: Vec<EvaluatorQualificationRef>,
    pub evidence: Vec<AuthorityEvidenceRef>,
    pub result: AuthorityEvaluationResult,
    pub evaluated_at: i64,
    pub valid_from: Option<i64>,
    pub valid_until: Option<i64>,
    /// Optional immutable detailed rationale / evaluation transcript.
    pub rationale_ref: Option<ExternalEvidenceRef>,
}

impl AuthorityEvaluationRecord {
    pub fn validate(&self) -> Result<(), AuthorityEvaluationError> {
        if self.schema_version != AUTHORITY_EVALUATION_SCHEMA_VERSION {
            return Err(AuthorityEvaluationError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        require_text("authority.id", &self.id, MAX_AUTHORITY_ID_BYTES)?;
        require_did("authority.subject_did", &self.subject_did)?;
        require_did("authority.evaluator_did", &self.evaluator_did)?;
        self.response_authority.validate()?;

        if self.evidence.is_empty() {
            return Err(AuthorityEvaluationError::MissingAuthorityEvidence);
        }
        if self.evidence.len() > MAX_AUTHORITY_EVIDENCE {
            return Err(AuthorityEvaluationError::TooMuchAuthorityEvidence {
                actual: self.evidence.len(),
                max: MAX_AUTHORITY_EVIDENCE,
            });
        }
        let mut evidence = HashSet::with_capacity(self.evidence.len());
        for reference in &self.evidence {
            reference.validate()?;
            if !evidence.insert(reference) {
                return Err(AuthorityEvaluationError::DuplicateAuthorityEvidence);
            }
        }

        if self.evaluator_qualifications.len() > MAX_AUTHORITY_QUALIFICATIONS {
            return Err(AuthorityEvaluationError::TooManyEvaluatorQualifications {
                actual: self.evaluator_qualifications.len(),
                max: MAX_AUTHORITY_QUALIFICATIONS,
            });
        }
        let mut qualifications = HashSet::with_capacity(self.evaluator_qualifications.len());
        for reference in &self.evaluator_qualifications {
            reference.validate()?;
            if !qualifications.insert(reference) {
                return Err(AuthorityEvaluationError::DuplicateEvaluatorQualification);
            }
        }

        if let Some(valid_from) = self.valid_from
            && let Some(valid_until) = self.valid_until
            && valid_from > valid_until
        {
            return Err(AuthorityEvaluationError::InvalidValidityWindow {
                valid_from,
                valid_until,
            });
        }
        if let Some(valid_until) = self.valid_until
            && self.evaluated_at > valid_until
        {
            return Err(AuthorityEvaluationError::EvaluationAfterExpiry {
                evaluated_at: self.evaluated_at,
                valid_until,
            });
        }

        if let Some(rationale) = &self.rationale_ref {
            rationale
                .validate()
                .map_err(|error| AuthorityEvaluationError::InvalidRationale(error.to_string()))?;
            if rationale.content_digest.is_none() {
                return Err(AuthorityEvaluationError::MutableRationale(
                    rationale.resource_id.clone(),
                ));
            }
        }

        Ok(())
    }

    pub fn validate_against_proposal(
        &self,
        proposal: &ResponseProposal,
    ) -> Result<(), AuthorityEvaluationError> {
        self.validate()?;
        self.response_authority.validate_against_proposal(proposal)
    }

    /// Whether the record claims authority is allowed at `unix_seconds`.
    ///
    /// This remains a semantic claim, not an execution capability or trust
    /// decision. Callers must separately authenticate/evaluate this record.
    pub fn claims_allowed_at(&self, unix_seconds: i64) -> bool {
        if self.result != AuthorityEvaluationResult::Allowed {
            return false;
        }
        if self
            .valid_from
            .is_some_and(|valid_from| unix_seconds < valid_from)
        {
            return false;
        }
        if self
            .valid_until
            .is_some_and(|valid_until| unix_seconds > valid_until)
        {
            return false;
        }
        true
    }

    /// An evaluation record is evidence about authority, never the capability.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum AuthorityEvaluationError {
    UnsupportedSchemaVersion(u16),
    EmptyField(&'static str),
    FieldTooLong {
        field: &'static str,
        actual: usize,
        max: usize,
    },
    MalformedDid(&'static str),
    MalformedDigest {
        field: &'static str,
        reason: &'static str,
    },
    InvalidRequirement(String),
    InvalidProposal(String),
    ProposalIdMismatch,
    UnknownResponseOption,
    RequirementNotOnOption,
    InvalidEvidence(String),
    MutableAuthorityEvidence(String),
    MissingAuthorityEvidence,
    TooMuchAuthorityEvidence {
        actual: usize,
        max: usize,
    },
    DuplicateAuthorityEvidence,
    InvalidQualification(String),
    MutableEvaluatorQualification(String),
    TooManyEvaluatorQualifications {
        actual: usize,
        max: usize,
    },
    DuplicateEvaluatorQualification,
    InvalidValidityWindow {
        valid_from: i64,
        valid_until: i64,
    },
    EvaluationAfterExpiry {
        evaluated_at: i64,
        valid_until: i64,
    },
    InvalidRationale(String),
    MutableRationale(String),
}

impl fmt::Display for AuthorityEvaluationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(v) => {
                write!(f, "unsupported authority-evaluation schema version {v}")
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::MalformedDid(field) => write!(f, "{field} must use a DID"),
            Self::MalformedDigest { field, reason } => {
                write!(f, "malformed {field}: {reason}")
            }
            Self::InvalidRequirement(error) => write!(f, "invalid authority requirement: {error}"),
            Self::InvalidProposal(error) => write!(f, "invalid response proposal: {error}"),
            Self::ProposalIdMismatch => write!(f, "authority evaluation references a different response proposal"),
            Self::UnknownResponseOption => write!(f, "authority evaluation references an unknown response option"),
            Self::RequirementNotOnOption => write!(f, "evaluated authority requirement is not declared by the response option"),
            Self::InvalidEvidence(error) => write!(f, "invalid authority evidence: {error}"),
            Self::MutableAuthorityEvidence(id) => write!(f, "authority evidence {id} must be content-digest bound"),
            Self::MissingAuthorityEvidence => write!(f, "authority evaluation requires at least one evidence artifact"),
            Self::TooMuchAuthorityEvidence { actual, max } => write!(f, "authority evaluation has {actual} evidence refs; maximum is {max}"),
            Self::DuplicateAuthorityEvidence => write!(f, "authority evaluation contains duplicate evidence"),
            Self::InvalidQualification(error) => write!(f, "invalid evaluator qualification: {error}"),
            Self::MutableEvaluatorQualification(id) => write!(f, "evaluator qualification {id} must be content-digest bound"),
            Self::TooManyEvaluatorQualifications { actual, max } => write!(f, "authority evaluation has {actual} evaluator qualifications; maximum is {max}"),
            Self::DuplicateEvaluatorQualification => write!(f, "authority evaluation contains duplicate evaluator qualifications"),
            Self::InvalidValidityWindow { valid_from, valid_until } => write!(f, "authority validity starts at {valid_from} after it ends at {valid_until}"),
            Self::EvaluationAfterExpiry { evaluated_at, valid_until } => write!(f, "authority evaluation at {evaluated_at} occurs after validity ended at {valid_until}"),
            Self::InvalidRationale(error) => write!(f, "invalid authority rationale reference: {error}"),
            Self::MutableRationale(id) => write!(f, "authority rationale {id} must be content-digest bound"),
        }
    }
}

impl std::error::Error for AuthorityEvaluationError {}

fn require_did(field: &'static str, value: &str) -> Result<(), AuthorityEvaluationError> {
    require_text(field, value, MAX_AUTHORITY_ID_BYTES)?;
    if !value.starts_with("did:") {
        return Err(AuthorityEvaluationError::MalformedDid(field));
    }
    Ok(())
}

fn require_text(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), AuthorityEvaluationError> {
    if value.trim().is_empty() {
        return Err(AuthorityEvaluationError::EmptyField(field));
    }
    if value.len() > max {
        return Err(AuthorityEvaluationError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

fn validate_digest(field: &'static str, digest: &str) -> Result<(), AuthorityEvaluationError> {
    require_text(field, digest, 256)?;
    let Some((algorithm, value)) = digest.split_once(':') else {
        return Err(AuthorityEvaluationError::MalformedDigest {
            field,
            reason: "digest must be algorithm-qualified",
        });
    };
    if algorithm.trim().is_empty() || value.trim().is_empty() {
        return Err(AuthorityEvaluationError::MalformedDigest {
            field,
            reason: "digest algorithm and value must both be non-empty",
        });
    }
    if !algorithm
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'+'))
    {
        return Err(AuthorityEvaluationError::MalformedDigest {
            field,
            reason: "digest algorithm contains unsupported characters",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EvidenceClass, GeoPoint, InterventionOption, ProjectedOutcomeRef, ProjectedOutcomeRole,
        ResponseProposalMode, Reversibility, RiskAssessmentMode, RiskTriggerRef, SpatialExtent,
        TemporalExtent,
    };

    fn immutable(resource_id: &str) -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "mycelix".into(),
            resource_id: resource_id.into(),
            content_digest: Some(format!("sha256:{resource_id}")),
            retrieved_at: None,
            license: None,
        }
    }

    fn requirement() -> AuthorityRequirement {
        AuthorityRequirement {
            domain: "municipal_emergency".into(),
            action: "open_cooling_center".into(),
            jurisdiction: Some("Johannesburg".into()),
            policy_ref: Some(immutable("policy:heat-response:v1")),
        }
    }

    fn proposal() -> ResponseProposal {
        ResponseProposal::new(
            "response:heat:1",
            ResponseProposalMode::OperationalRecommendation,
            RiskTriggerRef {
                assessment_id: "risk:heat:1".into(),
                assessment_digest: "sha256:risk".into(),
                risk_output_observation_id: "obs:risk:heat".into(),
                expected_mode: RiskAssessmentMode::Operational,
            },
            100,
            None,
            vec![InterventionOption {
                id: "cooling-centers".into(),
                intervention_type: "open_cooling_centers".into(),
                target: SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
                execution_window: TemporalExtent::new(110, 200).unwrap(),
                projected_outcomes: vec![ProjectedOutcomeRef {
                    observation_id: "obs:projected:1".into(),
                    expected_class: EvidenceClass::Scenario,
                    role: ProjectedOutcomeRole::Benefit,
                }],
                resources: Vec::new(),
                authority_requirements: vec![requirement()],
                reversibility: Reversibility::Reversible,
                assumptions: Vec::new(),
            }],
        )
        .unwrap()
    }

    fn evaluation(result: AuthorityEvaluationResult) -> AuthorityEvaluationRecord {
        AuthorityEvaluationRecord {
            schema_version: AUTHORITY_EVALUATION_SCHEMA_VERSION,
            id: "authority-eval:1".into(),
            response_authority: ResponseAuthorityRef {
                proposal_id: "response:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
                option_id: "cooling-centers".into(),
                requirement: requirement(),
            },
            subject_did: "did:mycelix:executor".into(),
            evaluator_did: "did:mycelix:authority-verifier".into(),
            evaluator_qualifications: vec![EvaluatorQualificationRef {
                role: "municipal_authority_verifier".into(),
                reference: immutable("credential:verifier:1"),
            }],
            evidence: vec![AuthorityEvidenceRef {
                kind: AuthorityEvidenceKind::Grant,
                reference: immutable("grant:cooling-center:1"),
            }],
            result,
            evaluated_at: 105,
            valid_from: Some(100),
            valid_until: Some(200),
            rationale_ref: None,
        }
    }

    #[test]
    fn allowed_evaluation_matches_declared_response_requirement() {
        let record = evaluation(AuthorityEvaluationResult::Allowed);
        record.validate_against_proposal(&proposal()).unwrap();
        assert!(record.claims_allowed_at(150));
        assert!(!record.grants_execution_authority());
    }

    #[test]
    fn allowed_claim_expires_without_becoming_capability() {
        let record = evaluation(AuthorityEvaluationResult::Allowed);
        assert!(!record.claims_allowed_at(201));
        assert!(!record.grants_execution_authority());
    }

    #[test]
    fn requirement_not_present_on_option_is_rejected() {
        let mut record = evaluation(AuthorityEvaluationResult::Allowed);
        record.response_authority.requirement.action = "release_funds".into();
        assert_eq!(
            record.validate_against_proposal(&proposal()),
            Err(AuthorityEvaluationError::RequirementNotOnOption)
        );
    }

    #[test]
    fn mutable_authority_evidence_is_rejected() {
        let mut record = evaluation(AuthorityEvaluationResult::Allowed);
        record.evidence[0].reference.content_digest = None;
        assert!(matches!(
            record.validate(),
            Err(AuthorityEvaluationError::MutableAuthorityEvidence(_))
        ));
    }

    #[test]
    fn denied_or_indeterminate_never_claim_allowed() {
        for result in [
            AuthorityEvaluationResult::Denied,
            AuthorityEvaluationResult::Indeterminate,
            AuthorityEvaluationResult::NotApplicable,
        ] {
            assert!(!evaluation(result).claims_allowed_at(150));
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn authority_evaluation_round_trips() {
        let original = evaluation(AuthorityEvaluationResult::Allowed);
        let encoded = serde_json::to_string(&original).unwrap();
        let decoded: AuthorityEvaluationRecord = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, original);
    }
}
