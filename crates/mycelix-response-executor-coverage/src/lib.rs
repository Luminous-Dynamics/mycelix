// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Semantic coverage matching between one prepared planetary response authority
//! requirement and one current-executor provider projection.
//!
//! This crate is deliberately **not** an execution-admission theorem.
//!
//! It proves only that a provider receipt which is internally valid and live at
//! the supplied time describes executor semantics compatible with the selected
//! response requirement:
//!
//! - exact response proposal ID;
//! - exact capability scope (`AuthorityRequirement.action`);
//! - exact jurisdiction;
//! - exact generic AuthorityGrant subject;
//! - exact generic ExecutorDesignation subject; and
//! - exact ThresholdAuthorization subject when the prepared mapping names it.
//!
//! Provider receipt deserialization is not proof of provider origin. This layer
//! also does not bind governance action bytes to the selected response option,
//! prove final deployment/coordinator state, satisfy effect-safety policy, or
//! fence an execution attempt. Those remain later independent theorems.

use mycelix_authority_freshness::{
    AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest,
};
use mycelix_governance_executor_provider_contract::{
    CurrentExecutorAuthorityProjection, VerifiedCurrentExecutorAuthorityReceipt,
};
use mycelix_response_authority_binding::PreparedResponseAuthorityResolution;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-response-executor-coverage-v0.1";

/// Non-deserializable result of pure semantic matching.
///
/// This is evidence that two already-supplied semantic surfaces agree. It is not
/// evidence that the executor receipt came from the designated local provider,
/// and it is never an execution capability.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MatchedResponseExecutorCoverage {
    protocol_version: String,
    proposal_id: String,
    proposal_digest: String,
    decision_id: String,
    option_id: String,
    capability_scope: String,
    jurisdiction: Option<String>,
    executor_authority_ref: String,
    executor_principal: String,
    granting_institution: String,
    actions_digest: [u8; 32],
    actions_digest_profile: String,
    matched_executor_subjects: Vec<AuthoritySubjectRef>,
    pending_authority_subjects: Vec<AuthoritySubjectRef>,
    provider_verification_ref: String,
    provider_verified_at_ms: u64,
    provider_valid_until_ms: u64,
}

impl MatchedResponseExecutorCoverage {
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

    pub fn capability_scope(&self) -> &str {
        &self.capability_scope
    }

    pub fn jurisdiction(&self) -> Option<&str> {
        self.jurisdiction.as_deref()
    }

    pub fn executor_authority_ref(&self) -> &str {
        &self.executor_authority_ref
    }

    pub fn matched_executor_subjects(&self) -> &[AuthoritySubjectRef] {
        &self.matched_executor_subjects
    }

    /// Authority subjects named by the prepared response mapping that are not
    /// discharged by the current-executor semantic theorem. Effect-safety is a
    /// typical example and must remain visible for later admission.
    pub fn pending_authority_subjects(&self) -> &[AuthoritySubjectRef] {
        &self.pending_authority_subjects
    }

    pub fn provider_valid_until_ms(&self) -> u64 {
        self.provider_valid_until_ms
    }

    /// Semantic coverage is never execution authority.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    /// This pure matcher cannot prove that a deserializable receipt originated
    /// from the designated local authority provider.
    pub const fn provider_origin_verified_here(&self) -> bool {
        false
    }

    /// The active authority stack has separate deployment/coordinator theorems;
    /// this matcher does not consume them yet.
    pub const fn deployment_bound_here(&self) -> bool {
        false
    }

    /// Governance action bytes still require an exact response→execution-plan
    /// binding theorem.
    pub const fn action_bytes_bound_here(&self) -> bool {
        false
    }
}

/// Match one prepared response authority requirement to one current-executor
/// provider receipt.
///
/// `VerifiedCurrentExecutorAuthorityReceipt::validate_at` proves only the wire
/// contract's internal consistency/current lease. The caller of this function
/// must still obtain the receipt from the designated local provider at the live
/// admission boundary; arbitrary caller-supplied bytes are not authority.
pub fn match_response_executor_coverage(
    prepared: &PreparedResponseAuthorityResolution,
    receipt: &VerifiedCurrentExecutorAuthorityReceipt,
    now_ms: u64,
) -> Result<MatchedResponseExecutorCoverage, ResponseExecutorCoverageError> {
    receipt
        .validate_at(now_ms)
        .map_err(|error| ResponseExecutorCoverageError::InvalidExecutorReceipt(error.to_string()))?;
    let projection = &receipt.projection;

    if projection.proposal_id.as_str() != prepared.proposal_id() {
        return Err(ResponseExecutorCoverageError::ProposalMismatch {
            expected: prepared.proposal_id().into(),
            actual: projection.proposal_id.as_str().into(),
        });
    }

    if projection.capability_scope.as_str() != prepared.requirement().action {
        return Err(ResponseExecutorCoverageError::CapabilityScopeMismatch {
            expected: prepared.requirement().action.clone(),
            actual: projection.capability_scope.as_str().into(),
        });
    }

    let actual_jurisdiction = projection.jurisdiction.as_ref().map(|value| value.as_str());
    if actual_jurisdiction != prepared.requirement().jurisdiction.as_deref() {
        return Err(ResponseExecutorCoverageError::JurisdictionMismatch {
            expected: prepared.requirement().jurisdiction.clone(),
            actual: actual_jurisdiction.map(str::to_string),
        });
    }

    let grant_subject = grant_subject(projection);
    let threshold_subject = threshold_subject(projection);
    let executor_subject = executor_subject(projection);
    for subject in [&grant_subject, &threshold_subject, &executor_subject] {
        subject
            .validate()
            .map_err(|error| ResponseExecutorCoverageError::InvalidProjectedSubject(
                error.to_string(),
            ))?;
    }

    let prepared_subjects = prepared.subjects();
    require_exact_kind_subject(
        prepared_subjects,
        AuthoritySubjectKind::AuthorityGrant,
        &grant_subject,
        true,
    )?;
    // Threshold authority is an internal dependency of the executor theorem and
    // therefore need not be redundantly named by RESPONSE-AUTH-1. If it is named,
    // however, it must be the exact same subject.
    require_exact_kind_subject(
        prepared_subjects,
        AuthoritySubjectKind::ThresholdAuthorization,
        &threshold_subject,
        false,
    )?;
    require_exact_kind_subject(
        prepared_subjects,
        AuthoritySubjectKind::ExecutorDesignation,
        &executor_subject,
        true,
    )?;

    let mut matched = vec![grant_subject.clone(), executor_subject.clone()];
    if prepared_subjects.iter().any(|subject| {
        subject.kind == AuthoritySubjectKind::ThresholdAuthorization
    }) {
        matched.push(threshold_subject.clone());
    }

    let pending_authority_subjects = prepared_subjects
        .iter()
        .filter(|subject| !matched.contains(subject))
        .cloned()
        .collect::<Vec<_>>();

    Ok(MatchedResponseExecutorCoverage {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: prepared.proposal_id().into(),
        proposal_digest: prepared.proposal_digest().into(),
        decision_id: prepared.decision_id().into(),
        option_id: prepared.option_id().into(),
        capability_scope: projection.capability_scope.as_str().into(),
        jurisdiction: actual_jurisdiction.map(str::to_string),
        executor_authority_ref: projection.executor_authority_ref.clone(),
        executor_principal: projection.executor_principal.as_str().into(),
        granting_institution: projection.granting_institution.as_str().into(),
        actions_digest: projection.actions_digest.0,
        actions_digest_profile: projection.actions_digest_profile.clone(),
        matched_executor_subjects: matched,
        pending_authority_subjects,
        provider_verification_ref: receipt.verification_ref.clone(),
        provider_verified_at_ms: receipt.verified_at_ms,
        provider_valid_until_ms: receipt.valid_until_ms,
    })
}

fn grant_subject(projection: &CurrentExecutorAuthorityProjection) -> AuthoritySubjectRef {
    AuthoritySubjectRef {
        kind: AuthoritySubjectKind::AuthorityGrant,
        namespace: projection.granting_institution.as_str().into(),
        subject_id: projection.authority_grant_id.as_str().into(),
        identity: ProfiledDigest {
            digest: projection.authority_grant_identity_digest,
            profile: projection.authority_grant_identity_profile.clone(),
        },
    }
}

fn threshold_subject(projection: &CurrentExecutorAuthorityProjection) -> AuthoritySubjectRef {
    AuthoritySubjectRef {
        kind: AuthoritySubjectKind::ThresholdAuthorization,
        namespace: projection.granting_institution.as_str().into(),
        subject_id: projection.threshold_authorization_ref.clone(),
        identity: ProfiledDigest {
            digest: projection.threshold_authorization_identity_digest,
            profile: projection.threshold_authorization_identity_profile.clone(),
        },
    }
}

fn executor_subject(projection: &CurrentExecutorAuthorityProjection) -> AuthoritySubjectRef {
    AuthoritySubjectRef {
        kind: AuthoritySubjectKind::ExecutorDesignation,
        namespace: projection.granting_institution.as_str().into(),
        subject_id: projection.executor_designation_record_ref.clone(),
        identity: ProfiledDigest {
            digest: projection.semantic_executor_authority_digest,
            profile: projection.semantic_executor_authority_profile.clone(),
        },
    }
}

fn require_exact_kind_subject(
    prepared: &[AuthoritySubjectRef],
    kind: AuthoritySubjectKind,
    expected: &AuthoritySubjectRef,
    required: bool,
) -> Result<(), ResponseExecutorCoverageError> {
    let matches = prepared
        .iter()
        .filter(|subject| subject.kind == kind)
        .collect::<Vec<_>>();
    match matches.as_slice() {
        [] if required => Err(ResponseExecutorCoverageError::MissingPreparedSubject(kind)),
        [] => Ok(()),
        [actual] if *actual == expected => Ok(()),
        [actual] => Err(ResponseExecutorCoverageError::PreparedSubjectMismatch {
            kind,
            expected_id: expected.subject_id.clone(),
            actual_id: actual.subject_id.clone(),
        }),
        _ => Err(ResponseExecutorCoverageError::AmbiguousPreparedSubjectKind(kind)),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResponseExecutorCoverageError {
    InvalidExecutorReceipt(String),
    ProposalMismatch {
        expected: String,
        actual: String,
    },
    CapabilityScopeMismatch {
        expected: String,
        actual: String,
    },
    JurisdictionMismatch {
        expected: Option<String>,
        actual: Option<String>,
    },
    InvalidProjectedSubject(String),
    MissingPreparedSubject(AuthoritySubjectKind),
    PreparedSubjectMismatch {
        kind: AuthoritySubjectKind,
        expected_id: String,
        actual_id: String,
    },
    AmbiguousPreparedSubjectKind(AuthoritySubjectKind),
}

impl fmt::Display for ResponseExecutorCoverageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidExecutorReceipt(error) => write!(f, "invalid current executor receipt: {error}"),
            Self::ProposalMismatch { expected, actual } => {
                write!(f, "executor proposal {actual} does not match response proposal {expected}")
            }
            Self::CapabilityScopeMismatch { expected, actual } => write!(
                f,
                "executor capability scope {actual} does not match required response action {expected}"
            ),
            Self::JurisdictionMismatch { expected, actual } => write!(
                f,
                "executor jurisdiction {actual:?} does not match response jurisdiction {expected:?}"
            ),
            Self::InvalidProjectedSubject(error) => {
                write!(f, "executor projection produced invalid authority subject: {error}")
            }
            Self::MissingPreparedSubject(kind) => {
                write!(f, "prepared response mapping is missing required {kind:?} subject")
            }
            Self::PreparedSubjectMismatch {
                kind,
                expected_id,
                actual_id,
            } => write!(
                f,
                "prepared {kind:?} subject {actual_id} does not match executor authority subject {expected_id}"
            ),
            Self::AmbiguousPreparedSubjectKind(kind) => write!(
                f,
                "prepared response mapping contains multiple {kind:?} subjects"
            ),
        }
    }
}

impl std::error::Error for ResponseExecutorCoverageError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::ProfiledDigest;
    use mycelix_authority_identity::AUTHORITY_GRANT_IDENTITY_PROFILE;
    use mycelix_core_types::{
        AuthorityRequirement, DecisionMakerClaim, DecisionMechanism, DecisionProposalRef,
        EvidenceClass, ExternalEvidenceRef, GeoPoint, InterventionOption, ProjectedOutcomeRef,
        ProjectedOutcomeRole, ResponseArtifactKind, ResponseArtifactRef, ResponseDecisionRecord,
        ResponseDisposition, ResponseLifecycleLedger, ResponseLifecycleStage, ResponseProposal,
        ResponseProposalMode, Reversibility, RiskAssessmentMode, RiskTriggerRef, SpatialExtent,
        TemporalExtent, RESPONSE_DECISION_SCHEMA_VERSION, RESPONSE_LIFECYCLE_SCHEMA_VERSION,
    };
    use mycelix_governance_current_executor_authority::{
        CURRENT_EXECUTOR_AUTHORITY_PROFILE, THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE,
    };
    use mycelix_governance_executor_lineage::EXECUTOR_LINEAGE_AUTHORITY_PROFILE;
    use mycelix_governance_executor_provider_contract::{
        executor_authority_ref, CurrentExecutorAuthorityProjection, PROTOCOL_VERSION as PROVIDER_PROTOCOL,
    };
    use mycelix_institutional_core::{
        AuthorityGrantId, CapabilityId, Digest32, InstitutionId, JurisdictionId, PrincipalId,
        RulebookId, RulebookRef,
    };
    use mycelix_response_authority_binding::{
        prepare_response_authority_resolution, ResponseAuthorityBindingCandidate,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

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

    fn decision() -> ResponseDecisionRecord {
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
            disposition: ResponseDisposition::SelectOption {
                option_id: "cooling-centers".into(),
            },
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

    fn projection() -> CurrentExecutorAuthorityProjection {
        CurrentExecutorAuthorityProjection {
            proposal_id: mycelix_governance_authority::ProposalId::new("response:heat:1").unwrap(),
            actions_digest: d(1),
            actions_digest_profile: "actions-v1-blake3".into(),
            threshold_authorization_ref: "threshold:heat:1".into(),
            threshold_authorization_identity_digest: d(2),
            threshold_authorization_identity_profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.into(),
            executor_principal: PrincipalId::new("did:mycelix:executor").unwrap(),
            executor_authority_ref: executor_authority_ref(
                d(3),
                CURRENT_EXECUTOR_AUTHORITY_PROFILE,
            )
            .unwrap(),
            current_executor_authority_digest: d(3),
            current_executor_authority_profile: CURRENT_EXECUTOR_AUTHORITY_PROFILE.into(),
            semantic_executor_authority_digest: d(4),
            semantic_executor_authority_profile: EXECUTOR_LINEAGE_AUTHORITY_PROFILE.into(),
            executor_designation_record_ref: "designation:heat:1".into(),
            authority_grant_id: AuthorityGrantId::new("grant:heat:1").unwrap(),
            authority_grant_identity_digest: d(5),
            authority_grant_identity_profile: AUTHORITY_GRANT_IDENTITY_PROFILE.into(),
            granting_institution: InstitutionId::new("municipality:johannesburg").unwrap(),
            jurisdiction: Some(JurisdictionId::new("Johannesburg").unwrap()),
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:heat-response").unwrap(),
                version: "1".into(),
                digest: d(6),
            },
            capability_scope: CapabilityId::new("open_cooling_center").unwrap(),
            freshness_digest: d(7),
            freshness_profile: mycelix_authority_freshness::BUNDLE_IDENTITY_PROFILE.into(),
            semantic_valid_until_ms: 1_000,
        }
    }

    fn subject(kind: AuthoritySubjectKind, id: &str, digest: Digest32, profile: &str) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind,
            namespace: "municipality:johannesburg".into(),
            subject_id: id.into(),
            identity: ProfiledDigest {
                digest,
                profile: profile.into(),
            },
        }
    }

    fn prepared() -> PreparedResponseAuthorityResolution {
        let p = projection();
        let candidate = ResponseAuthorityBindingCandidate {
            requirement: requirement(),
            subjects: vec![
                subject(
                    AuthoritySubjectKind::AuthorityGrant,
                    p.authority_grant_id.as_str(),
                    p.authority_grant_identity_digest,
                    &p.authority_grant_identity_profile,
                ),
                subject(
                    AuthoritySubjectKind::ExecutorDesignation,
                    &p.executor_designation_record_ref,
                    p.semantic_executor_authority_digest,
                    &p.semantic_executor_authority_profile,
                ),
                subject(
                    AuthoritySubjectKind::EffectSafetyPolicy,
                    "effect-safety:cooling-center",
                    d(9),
                    "effect-safety-v1",
                ),
            ],
            mapping_basis: vec![ExternalEvidenceRef {
                source_system: "mycelix-governance".into(),
                resource_id: "response-authority-map:jhb:heat:v1".into(),
                content_digest: Some("sha256:mapping".into()),
                retrieved_at: None,
                license: None,
            }],
        };
        prepare_response_authority_resolution(&lifecycle(), &proposal(), &decision(), &candidate)
            .unwrap()
    }

    fn receipt() -> VerifiedCurrentExecutorAuthorityReceipt {
        VerifiedCurrentExecutorAuthorityReceipt {
            protocol: PROVIDER_PROTOCOL.into(),
            projection: projection(),
            verification_ref: "provider-run:heat:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 500,
        }
    }

    #[test]
    fn executor_semantics_match_and_effect_safety_remains_pending() {
        let matched = match_response_executor_coverage(&prepared(), &receipt(), 200).unwrap();
        assert_eq!(matched.capability_scope(), "open_cooling_center");
        assert_eq!(matched.jurisdiction(), Some("Johannesburg"));
        assert_eq!(matched.matched_executor_subjects().len(), 2);
        assert_eq!(matched.pending_authority_subjects().len(), 1);
        assert_eq!(
            matched.pending_authority_subjects()[0].kind,
            AuthoritySubjectKind::EffectSafetyPolicy
        );
        assert!(!matched.grants_execution_authority());
        assert!(!matched.provider_origin_verified_here());
        assert!(!matched.deployment_bound_here());
        assert!(!matched.action_bytes_bound_here());
    }

    #[test]
    fn capability_scope_substitution_is_rejected() {
        let mut receipt = receipt();
        receipt.projection.capability_scope = CapabilityId::new("release_funds").unwrap();
        assert!(matches!(
            match_response_executor_coverage(&prepared(), &receipt, 200),
            Err(ResponseExecutorCoverageError::CapabilityScopeMismatch { .. })
        ));
    }

    #[test]
    fn jurisdiction_substitution_is_rejected() {
        let mut receipt = receipt();
        receipt.projection.jurisdiction = Some(JurisdictionId::new("Pretoria").unwrap());
        assert!(matches!(
            match_response_executor_coverage(&prepared(), &receipt, 200),
            Err(ResponseExecutorCoverageError::JurisdictionMismatch { .. })
        ));
    }

    #[test]
    fn executor_designation_subject_substitution_is_rejected() {
        let p = projection();
        let candidate = ResponseAuthorityBindingCandidate {
            requirement: requirement(),
            subjects: vec![
                subject(
                    AuthoritySubjectKind::AuthorityGrant,
                    p.authority_grant_id.as_str(),
                    p.authority_grant_identity_digest,
                    &p.authority_grant_identity_profile,
                ),
                subject(
                    AuthoritySubjectKind::ExecutorDesignation,
                    "designation:other",
                    p.semantic_executor_authority_digest,
                    &p.semantic_executor_authority_profile,
                ),
            ],
            mapping_basis: vec![ExternalEvidenceRef {
                source_system: "mycelix-governance".into(),
                resource_id: "map:other".into(),
                content_digest: Some("sha256:other".into()),
                retrieved_at: None,
                license: None,
            }],
        };
        let prepared = prepare_response_authority_resolution(
            &lifecycle(),
            &proposal(),
            &decision(),
            &candidate,
        )
        .unwrap();
        assert!(matches!(
            match_response_executor_coverage(&prepared, &receipt(), 200),
            Err(ResponseExecutorCoverageError::PreparedSubjectMismatch {
                kind: AuthoritySubjectKind::ExecutorDesignation,
                ..
            })
        ));
    }

    #[test]
    fn expired_provider_receipt_is_rejected_before_semantic_match() {
        assert!(matches!(
            match_response_executor_coverage(&prepared(), &receipt(), 500),
            Err(ResponseExecutorCoverageError::InvalidExecutorReceipt(_))
        ));
    }
}
