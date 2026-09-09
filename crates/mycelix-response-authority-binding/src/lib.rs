// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure semantic bridge from one selected response requirement to the exact
//! generic authority subjects that a later live verifier must resolve.
//!
//! This crate deliberately stops before current-authority qualification.
//! A successful [`PreparedResponseAuthorityResolution`] means only:
//!
//! - the response decision selected this exact option;
//! - the requirement is actually declared by that option;
//! - the mapping candidate names a bounded, duplicate-free set of structurally
//!   valid operational authority subjects; and
//! - the mapping basis is immutable/content-digest bound.
//!
//! It does **not** prove that an institution adopted the mapping, that any
//! subject is current, or that an executor may act. Live consumers must resolve
//! every prepared subject through the designated local current-authority
//! verifier and then apply the later domain/effect theorem.

use mycelix_authority_freshness::{AuthoritySubjectKind, AuthoritySubjectRef};
use mycelix_core_types::{
    validate_decision_binding, AuthorityRequirement, ExternalEvidenceRef, ResponseDecisionRecord,
    ResponseLifecycleLedger, ResponseProposal,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fmt};

pub const PROTOCOL_VERSION: &str = "mycelix-response-authority-binding-v0.1";
pub const MAX_AUTHORITY_SUBJECTS_PER_REQUIREMENT: usize = 32;
pub const MAX_MAPPING_EVIDENCE: usize = 32;

/// Transportable candidate mapping supplied by an institutional configuration,
/// policy registry, or other discovery boundary.
///
/// Deserialization is not proof that the mapping is adopted or authoritative.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponseAuthorityBindingCandidate {
    pub requirement: AuthorityRequirement,
    pub subjects: Vec<AuthoritySubjectRef>,
    /// Immutable evidence explaining why these exact authority subjects are
    /// associated with the response requirement. These references are audit
    /// provenance, not positive authority.
    pub mapping_basis: Vec<ExternalEvidenceRef>,
}

impl ResponseAuthorityBindingCandidate {
    pub fn validate(&self) -> Result<(), ResponseAuthorityBindingError> {
        self.requirement
            .validate()
            .map_err(|error| ResponseAuthorityBindingError::InvalidRequirement(error.to_string()))?;

        if self.subjects.is_empty()
            || self.subjects.len() > MAX_AUTHORITY_SUBJECTS_PER_REQUIREMENT
        {
            return Err(ResponseAuthorityBindingError::InvalidSubjectCount {
                actual: self.subjects.len(),
                max: MAX_AUTHORITY_SUBJECTS_PER_REQUIREMENT,
            });
        }
        for subject in &self.subjects {
            subject
                .validate()
                .map_err(|error| ResponseAuthorityBindingError::InvalidSubject(error.to_string()))?;
            if is_control_plane_subject(subject.kind) {
                return Err(ResponseAuthorityBindingError::ControlPlaneSubjectForbidden(
                    subject.subject_id.clone(),
                ));
            }
        }

        if self.mapping_basis.is_empty() || self.mapping_basis.len() > MAX_MAPPING_EVIDENCE {
            return Err(ResponseAuthorityBindingError::InvalidMappingEvidenceCount {
                actual: self.mapping_basis.len(),
                max: MAX_MAPPING_EVIDENCE,
            });
        }
        for evidence in &self.mapping_basis {
            evidence
                .validate()
                .map_err(|error| ResponseAuthorityBindingError::InvalidMappingEvidence(
                    error.to_string(),
                ))?;
            if evidence.content_digest.is_none() {
                return Err(ResponseAuthorityBindingError::MutableMappingEvidence(
                    evidence.resource_id.clone(),
                ));
            }
        }
        Ok(())
    }
}

/// Locally prepared resolution plan.
///
/// The type is intentionally serializable for audit but not deserializable as a
/// positive object. Even in-process existence is **not authority**: the output
/// only fixes which exact subjects a later verifier must re-resolve.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PreparedResponseAuthorityResolution {
    proposal_id: String,
    proposal_digest: String,
    decision_id: String,
    option_id: String,
    requirement: AuthorityRequirement,
    /// Canonically ordered by each subject's generic authority identity digest.
    subjects: Vec<AuthoritySubjectRef>,
    mapping_basis: Vec<ExternalEvidenceRef>,
}

impl PreparedResponseAuthorityResolution {
    pub fn proposal_id(&self) -> &str {
        &self.proposal_id
    }

    pub fn proposal_digest(&self) -> &str {
        &self.proposal_digest
    }

    pub fn decision_id(&self) -> &str {
        &self.decision_id
    }

    pub fn option_id(&self) -> &str {
        &self.option_id
    }

    pub fn requirement(&self) -> &AuthorityRequirement {
        &self.requirement
    }

    pub fn subjects(&self) -> &[AuthoritySubjectRef] {
        &self.subjects
    }

    pub fn mapping_basis(&self) -> &[ExternalEvidenceRef] {
        &self.mapping_basis
    }

    /// This object is a resolution plan, never an execution capability.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Prepare one exact selected response requirement for later live authority
/// resolution.
///
/// The selected option is derived from the concrete decision. The proposal
/// digest is taken from the lifecycle root, not independently caller-supplied.
pub fn prepare_response_authority_resolution(
    lifecycle: &ResponseLifecycleLedger,
    proposal: &ResponseProposal,
    decision: &ResponseDecisionRecord,
    candidate: &ResponseAuthorityBindingCandidate,
) -> Result<PreparedResponseAuthorityResolution, ResponseAuthorityBindingError> {
    candidate.validate()?;

    let decision_gate = validate_decision_binding(lifecycle, proposal, decision)
        .map_err(|error| ResponseAuthorityBindingError::DecisionBinding(error.to_string()))?;
    let option_id = decision_gate
        .selected_option_id()
        .ok_or(ResponseAuthorityBindingError::DecisionDidNotSelectOption)?;

    let option = proposal
        .options
        .iter()
        .find(|option| option.id == option_id)
        .ok_or_else(|| ResponseAuthorityBindingError::SelectedOptionMissing(option_id.into()))?;
    if !option.authority_requirements.contains(&candidate.requirement) {
        return Err(ResponseAuthorityBindingError::RequirementNotOnSelectedOption);
    }

    let subjects = canonicalize_subjects(&candidate.subjects)?;
    let mapping_basis = canonicalize_mapping_basis(&candidate.mapping_basis)?;

    Ok(PreparedResponseAuthorityResolution {
        proposal_id: lifecycle.proposal.id.clone(),
        proposal_digest: lifecycle.proposal.digest.clone(),
        decision_id: decision.id.clone(),
        option_id: option_id.into(),
        requirement: candidate.requirement.clone(),
        subjects,
        mapping_basis,
    })
}

fn canonicalize_subjects(
    subjects: &[AuthoritySubjectRef],
) -> Result<Vec<AuthoritySubjectRef>, ResponseAuthorityBindingError> {
    let mut by_identity = BTreeMap::<[u8; 32], AuthoritySubjectRef>::new();
    for subject in subjects {
        let Digest32(identity) = subject
            .identity_digest()
            .map_err(|error| ResponseAuthorityBindingError::InvalidSubject(error.to_string()))?;
        if by_identity.insert(identity, subject.clone()).is_some() {
            return Err(ResponseAuthorityBindingError::DuplicateAuthoritySubject);
        }
    }
    Ok(by_identity.into_values().collect())
}

fn canonicalize_mapping_basis(
    evidence: &[ExternalEvidenceRef],
) -> Result<Vec<ExternalEvidenceRef>, ResponseAuthorityBindingError> {
    let mut by_identity = BTreeMap::<(String, String, String), ExternalEvidenceRef>::new();
    for reference in evidence {
        let digest = reference
            .content_digest
            .clone()
            .ok_or_else(|| ResponseAuthorityBindingError::MutableMappingEvidence(
                reference.resource_id.clone(),
            ))?;
        let key = (
            reference.source_system.clone(),
            reference.resource_id.clone(),
            digest,
        );
        if by_identity.insert(key, reference.clone()).is_some() {
            return Err(ResponseAuthorityBindingError::DuplicateMappingEvidence);
        }
    }
    Ok(by_identity.into_values().collect())
}

fn is_control_plane_subject(kind: AuthoritySubjectKind) -> bool {
    matches!(
        kind,
        AuthoritySubjectKind::AuthorityCoveragePolicy
            | AuthoritySubjectKind::CoverageTrustContextPolicy
            | AuthoritySubjectKind::WitnessTrustPolicy
    )
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResponseAuthorityBindingError {
    InvalidRequirement(String),
    InvalidSubjectCount { actual: usize, max: usize },
    InvalidSubject(String),
    ControlPlaneSubjectForbidden(String),
    InvalidMappingEvidenceCount { actual: usize, max: usize },
    InvalidMappingEvidence(String),
    MutableMappingEvidence(String),
    DuplicateMappingEvidence,
    DecisionBinding(String),
    DecisionDidNotSelectOption,
    SelectedOptionMissing(String),
    RequirementNotOnSelectedOption,
    DuplicateAuthoritySubject,
}

impl fmt::Display for ResponseAuthorityBindingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRequirement(error) => write!(f, "invalid authority requirement: {error}"),
            Self::InvalidSubjectCount { actual, max } => write!(
                f,
                "response authority mapping has {actual} subjects; allowed range is 1..={max}"
            ),
            Self::InvalidSubject(error) => write!(f, "invalid authority subject: {error}"),
            Self::ControlPlaneSubjectForbidden(id) => write!(
                f,
                "control-plane authority subject {id} cannot satisfy an operational response requirement"
            ),
            Self::InvalidMappingEvidenceCount { actual, max } => write!(
                f,
                "response authority mapping has {actual} evidence refs; allowed range is 1..={max}"
            ),
            Self::InvalidMappingEvidence(error) => {
                write!(f, "invalid response authority mapping evidence: {error}")
            }
            Self::MutableMappingEvidence(id) => write!(
                f,
                "response authority mapping evidence {id} must be content-digest bound"
            ),
            Self::DuplicateMappingEvidence => {
                write!(f, "response authority mapping repeats an evidence identity")
            }
            Self::DecisionBinding(error) => write!(f, "invalid response decision binding: {error}"),
            Self::DecisionDidNotSelectOption => {
                write!(f, "response decision did not select an executable option")
            }
            Self::SelectedOptionMissing(id) => write!(f, "selected response option {id} is missing"),
            Self::RequirementNotOnSelectedOption => write!(
                f,
                "authority requirement is not declared by the selected response option"
            ),
            Self::DuplicateAuthoritySubject => {
                write!(f, "response authority mapping repeats an exact authority subject")
            }
        }
    }
}

impl std::error::Error for ResponseAuthorityBindingError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{ProfiledDigest, SUBJECT_IDENTITY_PROFILE};
    use mycelix_core_types::{
        DecisionMakerClaim, DecisionMechanism, DecisionProposalRef, EvidenceClass, GeoPoint,
        InterventionOption, ProjectedOutcomeRef, ProjectedOutcomeRole, ResponseArtifactKind,
        ResponseArtifactRef, ResponseDisposition, ResponseLifecycleStage, ResponseProposalMode,
        Reversibility, RiskAssessmentMode, RiskTriggerRef, SpatialExtent, TemporalExtent,
        RESPONSE_DECISION_SCHEMA_VERSION, RESPONSE_LIFECYCLE_SCHEMA_VERSION,
    };

    fn requirement() -> AuthorityRequirement {
        AuthorityRequirement {
            domain: "municipal_emergency".into(),
            action: "open_cooling_center".into(),
            jurisdiction: Some("Johannesburg".into()),
            policy_ref: None,
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

    fn decision(disposition: ResponseDisposition) -> ResponseDecisionRecord {
        ResponseDecisionRecord {
            schema_version: RESPONSE_DECISION_SCHEMA_VERSION,
            id: "decision:heat:1".into(),
            proposal: DecisionProposalRef {
                proposal_id: "response:heat:1".into(),
                proposal_digest: "sha256:proposal".into(),
            },
            decided_at: 105,
            mechanism: DecisionMechanism::EmergencyProcedure,
            decision_makers: vec![DecisionMakerClaim {
                did: "did:mycelix:official".into(),
                role: "incident_commander".into(),
                constituency: Some("Johannesburg".into()),
            }],
            governing_policies: Vec::new(),
            evidence_considered: Vec::new(),
            disposition,
            rationale_ref: None,
            alternate_positions: Vec::new(),
        }
    }

    fn lifecycle() -> ResponseLifecycleLedger {
        ResponseLifecycleLedger {
            schema_version: RESPONSE_LIFECYCLE_SCHEMA_VERSION,
            id: "loop:heat:1".into(),
            stage: ResponseLifecycleStage::Decided,
            proposal: ResponseArtifactRef::new(
                ResponseArtifactKind::Proposal,
                "response:heat:1",
                "sha256:proposal",
            )
            .unwrap(),
            decision: Some(
                ResponseArtifactRef::new(
                    ResponseArtifactKind::Decision,
                    "decision:heat:1",
                    "sha256:decision",
                )
                .unwrap(),
            ),
            authority_evaluations: Vec::new(),
            execution: None,
            outcome: None,
        }
    }

    fn subject(kind: AuthoritySubjectKind, byte: u8, id: &str) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind,
            namespace: "municipal:johannesburg:heat".into(),
            subject_id: id.into(),
            identity: ProfiledDigest {
                digest: Digest32([byte; 32]),
                profile: SUBJECT_IDENTITY_PROFILE.into(),
            },
        }
    }

    fn evidence() -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "mycelix-governance".into(),
            resource_id: "response-authority-map:jhb:heat:v1".into(),
            content_digest: Some("sha256:mapping".into()),
            retrieved_at: None,
            license: None,
        }
    }

    fn candidate() -> ResponseAuthorityBindingCandidate {
        ResponseAuthorityBindingCandidate {
            requirement: requirement(),
            subjects: vec![
                subject(AuthoritySubjectKind::AuthorityGrant, 1, "grant:cooling-centers"),
                subject(
                    AuthoritySubjectKind::ExecutorDesignation,
                    2,
                    "executor:municipal-heat-response",
                ),
                subject(
                    AuthoritySubjectKind::EffectSafetyPolicy,
                    3,
                    "effect-safety:facility-opening",
                ),
            ],
            mapping_basis: vec![evidence()],
        }
    }

    #[test]
    fn selected_requirement_prepares_exact_subject_set_without_authority() {
        let prepared = prepare_response_authority_resolution(
            &lifecycle(),
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &candidate(),
        )
        .unwrap();
        assert_eq!(prepared.option_id(), "cooling-centers");
        assert_eq!(prepared.subjects().len(), 3);
        assert!(!prepared.grants_execution_authority());
    }

    #[test]
    fn rejected_decision_cannot_prepare_live_authority_resolution() {
        let error = prepare_response_authority_resolution(
            &lifecycle(),
            &proposal(),
            &decision(ResponseDisposition::RejectAll),
            &candidate(),
        )
        .unwrap_err();
        assert!(matches!(error, ResponseAuthorityBindingError::DecisionBinding(_)));
    }

    #[test]
    fn requirement_must_belong_to_selected_option() {
        let mut candidate = candidate();
        candidate.requirement.action = "release_funds".into();
        let error = prepare_response_authority_resolution(
            &lifecycle(),
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &candidate,
        )
        .unwrap_err();
        assert_eq!(
            error,
            ResponseAuthorityBindingError::RequirementNotOnSelectedOption
        );
    }

    #[test]
    fn control_plane_subject_cannot_satisfy_operational_requirement() {
        let mut candidate = candidate();
        candidate.subjects = vec![subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            9,
            "coverage-policy:not-an-effect-grant",
        )];
        assert!(matches!(
            candidate.validate(),
            Err(ResponseAuthorityBindingError::ControlPlaneSubjectForbidden(_))
        ));
    }

    #[test]
    fn mutable_mapping_basis_is_rejected() {
        let mut candidate = candidate();
        candidate.mapping_basis[0].content_digest = None;
        assert!(matches!(
            candidate.validate(),
            Err(ResponseAuthorityBindingError::MutableMappingEvidence(_))
        ));
    }

    #[test]
    fn duplicate_subject_identity_is_rejected() {
        let mut candidate = candidate();
        candidate.subjects.push(candidate.subjects[0].clone());
        let error = prepare_response_authority_resolution(
            &lifecycle(),
            &proposal(),
            &decision(ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            }),
            &candidate,
        )
        .unwrap_err();
        assert_eq!(error, ResponseAuthorityBindingError::DuplicateAuthoritySubject);
    }
}
