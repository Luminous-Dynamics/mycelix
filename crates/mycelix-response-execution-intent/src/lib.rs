// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact-byte binding between a selected planetary response and the action plan
//! already committed by current executor authority.
//!
//! This layer closes one substitution seam only:
//!
//! ```text
//! non-deserializable selected response authority resolution
//!          +
//! internally valid current-executor provider receipt
//!          +
//! caller-supplied adapter payload candidate
//!          ↓
//! #513 semantic response/executor coverage
//!          ↓
//! canonical response execution action JSON
//!          ↓ exact-byte governance digest
//! must equal executor authority actions_digest/profile
//!          ↓
//! QualifiedResponseExecutionIntent
//! ```
//!
//! The caller is free to suggest payload bytes, but those bytes cannot qualify
//! unless the executor authority domain already committed the exact resulting
//! canonical action envelope. The translation artifact therefore cannot authorize
//! itself.
//!
//! Provider origin, deployment/coordinator state, effect-safety authority and
//! execution-attempt fencing remain independent later gates.

use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_execution_action_digest::{
    execution_authority_digest, ActionDigestError, ACTIONS_DIGEST_PROFILE_V1,
};
use mycelix_governance_executor_provider_contract::VerifiedCurrentExecutorAuthorityReceipt;
use mycelix_institutional_core::Digest32;
use mycelix_response_authority_binding::PreparedResponseAuthorityResolution;
use mycelix_response_executor_coverage::{
    match_response_executor_coverage, ResponseExecutorCoverageError,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-response-execution-intent-v0.1";
pub const ACTION_ENVELOPE_PROTOCOL: &str = "mycelix-response-execution-action-v0.1";
pub const MAX_ADAPTER_PROFILE_BYTES: usize = 128;
pub const MAX_ADAPTER_PAYLOAD_BYTES: usize = 2048;

/// Untrusted execution-specific payload candidate.
///
/// `payload_utf8` is deliberately opaque exact UTF-8. It may itself contain JSON,
/// shell-safe structured text, a domain DSL, or another adapter representation.
/// This layer does not parse or normalize it. `adapter_profile` defines the later
/// adapter interpretation and effect-safety theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponseExecutionIntentCandidate {
    pub adapter_profile: String,
    pub payload_utf8: String,
}

impl ResponseExecutionIntentCandidate {
    pub fn validate(&self) -> Result<(), ResponseExecutionIntentError> {
        validate_profile(&self.adapter_profile)?;
        if self.payload_utf8.is_empty() {
            return Err(ResponseExecutionIntentError::EmptyAdapterPayload);
        }
        if self.payload_utf8.len() > MAX_ADAPTER_PAYLOAD_BYTES {
            return Err(ResponseExecutionIntentError::AdapterPayloadTooLong {
                actual: self.payload_utf8.len(),
                max: MAX_ADAPTER_PAYLOAD_BYTES,
            });
        }
        Ok(())
    }
}

/// Non-deserializable proof that the exact canonical response action bytes equal
/// the action identity already carried by the current executor authority domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedResponseExecutionIntent {
    protocol_version: String,
    proposal_id: String,
    proposal_digest: String,
    decision_id: String,
    option_id: String,
    authority_domain: String,
    action_class: String,
    jurisdiction: Option<String>,
    adapter_profile: String,
    /// Exact JSON bytes whose digest matched the executor authority domain.
    actions_json: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    executor_authority_ref: String,
    provider_verification_ref: String,
    provider_verified_at_ms: u64,
    provider_valid_until_ms: u64,
    /// Subjects still not discharged by executor semantics, such as effect safety.
    pending_authority_subjects: Vec<AuthoritySubjectRef>,
}

impl QualifiedResponseExecutionIntent {
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

    pub fn action_class(&self) -> &str {
        &self.action_class
    }

    pub fn jurisdiction(&self) -> Option<&str> {
        self.jurisdiction.as_deref()
    }

    pub fn adapter_profile(&self) -> &str {
        &self.adapter_profile
    }

    pub fn actions_json(&self) -> &str {
        &self.actions_json
    }

    pub fn actions_digest(&self) -> Digest32 {
        self.actions_digest
    }

    pub fn actions_digest_profile(&self) -> &str {
        &self.actions_digest_profile
    }

    pub fn executor_authority_ref(&self) -> &str {
        &self.executor_authority_ref
    }

    pub fn provider_valid_until_ms(&self) -> u64 {
        self.provider_valid_until_ms
    }

    pub fn pending_authority_subjects(&self) -> &[AuthoritySubjectRef] {
        &self.pending_authority_subjects
    }

    /// This theorem really does bind the exact execution bytes.
    pub const fn exact_action_bytes_bound_here(&self) -> bool {
        true
    }

    /// Exact bytes are still not execution permission.
    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    /// A deserializable provider receipt cannot prove its own call provenance.
    pub const fn provider_origin_verified_here(&self) -> bool {
        false
    }

    /// Current deployment/coordinator composition is a separate authority theorem.
    pub const fn deployment_bound_here(&self) -> bool {
        false
    }

    /// Adapter enforceability/effect safety remains an independent requirement.
    pub const fn effect_safety_satisfied_here(&self) -> bool {
        false
    }

    /// No attempt ID, idempotency key, lock, claim or crash fence is created here.
    pub const fn attempt_fenced_here(&self) -> bool {
        false
    }
}

/// Bind one exact selected response to the exact action bytes already approved by
/// the executor authority domain.
pub fn qualify_response_execution_intent(
    prepared: &PreparedResponseAuthorityResolution,
    receipt: &VerifiedCurrentExecutorAuthorityReceipt,
    candidate: &ResponseExecutionIntentCandidate,
    now_ms: u64,
) -> Result<QualifiedResponseExecutionIntent, ResponseExecutionIntentError> {
    candidate.validate()?;

    // Re-run the complete #513 semantic theorem first. This proves proposal,
    // capability, jurisdiction and exact grant/designation subject continuity.
    let coverage = match_response_executor_coverage(prepared, receipt, now_ms)
        .map_err(ResponseExecutionIntentError::ExecutorCoverage)?;

    if receipt.projection.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1 {
        return Err(ResponseExecutionIntentError::UnsupportedActionsDigestProfile {
            actual: receipt.projection.actions_digest_profile.clone(),
        });
    }

    let actions_json = canonical_actions_json(
        prepared,
        coverage.capability_scope(),
        coverage.jurisdiction(),
        candidate,
    )?;
    let digest = execution_authority_digest(prepared.proposal_id(), &actions_json)
        .map_err(ResponseExecutionIntentError::ActionDigest)?;
    if digest != receipt.projection.actions_digest {
        return Err(ResponseExecutionIntentError::ActionsDigestMismatch {
            expected: receipt.projection.actions_digest,
            actual: digest,
        });
    }

    Ok(QualifiedResponseExecutionIntent {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: coverage.proposal_id().into(),
        proposal_digest: coverage.proposal_digest().into(),
        decision_id: coverage.decision_id().into(),
        option_id: coverage.option_id().into(),
        authority_domain: prepared.requirement().domain.clone(),
        action_class: coverage.capability_scope().into(),
        jurisdiction: coverage.jurisdiction().map(str::to_string),
        adapter_profile: candidate.adapter_profile.clone(),
        actions_json,
        actions_digest: digest,
        actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
        executor_authority_ref: coverage.executor_authority_ref().into(),
        provider_verification_ref: receipt.verification_ref.clone(),
        provider_verified_at_ms: receipt.verified_at_ms,
        provider_valid_until_ms: coverage.provider_valid_until_ms(),
        pending_authority_subjects: coverage.pending_authority_subjects().to_vec(),
    })
}

/// Deterministic JSON action envelope. The adapter payload is encoded as a JSON
/// string rather than parsed/re-emitted, so its exact bytes remain significant.
fn canonical_actions_json(
    prepared: &PreparedResponseAuthorityResolution,
    action_class: &str,
    jurisdiction: Option<&str>,
    candidate: &ResponseExecutionIntentCandidate,
) -> Result<String, ResponseExecutionIntentError> {
    #[derive(Serialize)]
    struct CanonicalAction<'a> {
        protocol: &'static str,
        response_proposal_id: &'a str,
        response_proposal_digest: &'a str,
        response_decision_id: &'a str,
        response_option_id: &'a str,
        authority_domain: &'a str,
        action_class: &'a str,
        jurisdiction: Option<&'a str>,
        adapter_profile: &'a str,
        payload_utf8: &'a str,
    }

    let action = CanonicalAction {
        protocol: ACTION_ENVELOPE_PROTOCOL,
        response_proposal_id: prepared.proposal_id(),
        response_proposal_digest: prepared.proposal_digest(),
        response_decision_id: prepared.decision_id(),
        response_option_id: prepared.option_id(),
        authority_domain: prepared.requirement().domain.as_str(),
        action_class,
        jurisdiction,
        adapter_profile: candidate.adapter_profile.as_str(),
        payload_utf8: candidate.payload_utf8.as_str(),
    };

    serde_json::to_string(&[action])
        .map_err(|error| ResponseExecutionIntentError::CanonicalSerialization(error.to_string()))
}

fn validate_profile(value: &str) -> Result<(), ResponseExecutionIntentError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_ADAPTER_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(ResponseExecutionIntentError::InvalidAdapterProfile);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResponseExecutionIntentError {
    InvalidAdapterProfile,
    EmptyAdapterPayload,
    AdapterPayloadTooLong { actual: usize, max: usize },
    ExecutorCoverage(ResponseExecutorCoverageError),
    UnsupportedActionsDigestProfile { actual: String },
    CanonicalSerialization(String),
    ActionDigest(ActionDigestError),
    ActionsDigestMismatch { expected: Digest32, actual: Digest32 },
}

impl fmt::Display for ResponseExecutionIntentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAdapterProfile => write!(f, "invalid response execution adapter profile"),
            Self::EmptyAdapterPayload => write!(f, "response execution adapter payload must not be empty"),
            Self::AdapterPayloadTooLong { actual, max } => write!(
                f,
                "response execution adapter payload is {actual} bytes; maximum is {max}"
            ),
            Self::ExecutorCoverage(error) => write!(f, "response executor coverage failed: {error}"),
            Self::UnsupportedActionsDigestProfile { actual } => write!(
                f,
                "executor action digest profile {actual} is not the registered exact-byte governance profile"
            ),
            Self::CanonicalSerialization(error) => {
                write!(f, "cannot serialize canonical response execution action: {error}")
            }
            Self::ActionDigest(error) => write!(f, "cannot digest response execution action: {error}"),
            Self::ActionsDigestMismatch { .. } => write!(
                f,
                "canonical response execution bytes do not match the executor-authorized action digest"
            ),
        }
    }
}

impl std::error::Error for ResponseExecutionIntentError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest, BUNDLE_IDENTITY_PROFILE,
    };
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
        executor_authority_ref, CurrentExecutorAuthorityProjection,
        PROTOCOL_VERSION as PROVIDER_PROTOCOL,
    };
    use mycelix_institutional_core::{
        AuthorityGrantId, CapabilityId, InstitutionId, JurisdictionId, PrincipalId, RulebookId,
        RulebookRef,
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

    fn option(id: &str) -> InterventionOption {
        InterventionOption {
            id: id.into(),
            intervention_type: "open_cooling_centers".into(),
            target: SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            execution_window: TemporalExtent::new(110, 200).unwrap(),
            projected_outcomes: vec![ProjectedOutcomeRef {
                observation_id: format!("obs:projected:{id}"),
                expected_class: EvidenceClass::Scenario,
                role: ProjectedOutcomeRole::Benefit,
            }],
            resources: Vec::new(),
            authority_requirements: vec![requirement()],
            reversibility: Reversibility::Reversible,
            assumptions: Vec::new(),
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
            vec![option("cooling-centers"), option("cooling-centers-alt")],
        )
        .unwrap()
    }

    fn decision(option_id: &str, decision_id: &str) -> ResponseDecisionRecord {
        ResponseDecisionRecord {
            schema_version: RESPONSE_DECISION_SCHEMA_VERSION,
            id: decision_id.into(),
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
                option_id: option_id.into(),
            },
            rationale_ref: None,
            alternate_positions: Vec::new(),
        }
    }

    fn lifecycle(decision_id: &str) -> ResponseLifecycleLedger {
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
                    decision_id,
                    "sha256:decision",
                )
                .unwrap(),
            ),
            authority_evaluations: Vec::new(),
            execution: None,
            outcome: None,
        }
    }

    fn projection(actions_digest: Digest32, actions_profile: &str) -> CurrentExecutorAuthorityProjection {
        CurrentExecutorAuthorityProjection {
            proposal_id: mycelix_governance_authority::ProposalId::new("response:heat:1").unwrap(),
            actions_digest,
            actions_digest_profile: actions_profile.into(),
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
            freshness_profile: BUNDLE_IDENTITY_PROFILE.into(),
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

    fn prepared_for(option_id: &str, decision_id: &str) -> PreparedResponseAuthorityResolution {
        let p = projection(d(1), ACTIONS_DIGEST_PROFILE_V1);
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
        prepare_response_authority_resolution(
            &lifecycle(decision_id),
            &proposal(),
            &decision(option_id, decision_id),
            &candidate,
        )
        .unwrap()
    }

    fn candidate(payload: &str) -> ResponseExecutionIntentCandidate {
        ResponseExecutionIntentCandidate {
            adapter_profile: "mycelix.municipal.cooling-center.v1".into(),
            payload_utf8: payload.into(),
        }
    }

    fn authorized_receipt(
        prepared: &PreparedResponseAuthorityResolution,
        intent: &ResponseExecutionIntentCandidate,
    ) -> VerifiedCurrentExecutorAuthorityReceipt {
        let actions = canonical_actions_json(
            prepared,
            "open_cooling_center",
            Some("Johannesburg"),
            intent,
        )
        .unwrap();
        let digest = execution_authority_digest(prepared.proposal_id(), &actions).unwrap();
        VerifiedCurrentExecutorAuthorityReceipt {
            protocol: PROVIDER_PROTOCOL.into(),
            projection: projection(digest, ACTIONS_DIGEST_PROFILE_V1),
            verification_ref: "provider-run:heat:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 500,
        }
    }

    #[test]
    fn exact_authorized_action_bytes_bind_selected_response() {
        let prepared = prepared_for("cooling-centers", "decision:heat:1");
        let candidate = candidate("{\"site\":\"library-12\",\"open\":true}");
        let receipt = authorized_receipt(&prepared, &candidate);
        let qualified = qualify_response_execution_intent(&prepared, &receipt, &candidate, 200)
            .unwrap();

        assert_eq!(qualified.proposal_id(), "response:heat:1");
        assert_eq!(qualified.option_id(), "cooling-centers");
        assert_eq!(qualified.action_class(), "open_cooling_center");
        assert_eq!(qualified.jurisdiction(), Some("Johannesburg"));
        assert_eq!(qualified.actions_digest(), receipt.projection.actions_digest);
        assert_eq!(qualified.actions_digest_profile(), ACTIONS_DIGEST_PROFILE_V1);
        assert_eq!(qualified.pending_authority_subjects().len(), 1);
        assert!(qualified.exact_action_bytes_bound_here());
        assert!(!qualified.grants_execution_authority());
        assert!(!qualified.provider_origin_verified_here());
        assert!(!qualified.deployment_bound_here());
        assert!(!qualified.effect_safety_satisfied_here());
        assert!(!qualified.attempt_fenced_here());
    }

    #[test]
    fn payload_mutation_is_rejected() {
        let prepared = prepared_for("cooling-centers", "decision:heat:1");
        let authorized = candidate("{\"site\":\"library-12\",\"open\":true}");
        let mutated = candidate("{\"site\":\"library-13\",\"open\":true}");
        let receipt = authorized_receipt(&prepared, &authorized);
        assert!(matches!(
            qualify_response_execution_intent(&prepared, &receipt, &mutated, 200),
            Err(ResponseExecutionIntentError::ActionsDigestMismatch { .. })
        ));
    }

    #[test]
    fn semantically_similar_json_reordering_is_still_an_identity_change() {
        let prepared = prepared_for("cooling-centers", "decision:heat:1");
        let authorized = candidate("{\"site\":\"library-12\",\"open\":true}");
        let reordered = candidate("{\"open\":true,\"site\":\"library-12\"}");
        let receipt = authorized_receipt(&prepared, &authorized);
        assert!(matches!(
            qualify_response_execution_intent(&prepared, &receipt, &reordered, 200),
            Err(ResponseExecutionIntentError::ActionsDigestMismatch { .. })
        ));
    }

    #[test]
    fn selected_option_substitution_changes_authorized_bytes() {
        let original = prepared_for("cooling-centers", "decision:heat:1");
        let substituted = prepared_for("cooling-centers-alt", "decision:heat:2");
        let intent = candidate("{\"site\":\"library-12\",\"open\":true}");
        let receipt = authorized_receipt(&original, &intent);
        assert!(matches!(
            qualify_response_execution_intent(&substituted, &receipt, &intent, 200),
            Err(ResponseExecutionIntentError::ActionsDigestMismatch { .. })
        ));
    }

    #[test]
    fn digest_profile_substitution_is_rejected_before_byte_match() {
        let prepared = prepared_for("cooling-centers", "decision:heat:1");
        let intent = candidate("{\"site\":\"library-12\"}");
        let mut receipt = authorized_receipt(&prepared, &intent);
        receipt.projection.actions_digest_profile = "some-other-profile".into();
        assert!(matches!(
            qualify_response_execution_intent(&prepared, &receipt, &intent, 200),
            Err(ResponseExecutionIntentError::UnsupportedActionsDigestProfile { .. })
        ));
    }

    #[test]
    fn canonical_envelope_is_valid_json_but_preserves_payload_as_exact_string() {
        let prepared = prepared_for("cooling-centers", "decision:heat:1");
        let intent = candidate("{ \"site\": \"library-12\" }");
        let actions = canonical_actions_json(
            &prepared,
            "open_cooling_center",
            Some("Johannesburg"),
            &intent,
        )
        .unwrap();
        let value: serde_json::Value = serde_json::from_str(&actions).unwrap();
        assert_eq!(value[0]["response_option_id"], "cooling-centers");
        assert_eq!(value[0]["payload_utf8"], intent.payload_utf8);
    }
}
