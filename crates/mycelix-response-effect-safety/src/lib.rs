// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure composition between one exact response execution intent and one exact
//! current effect-safety policy qualification.
//!
//! This layer discharges only the exact `EffectSafetyPolicy` subject that the
//! response authority mapping left pending. It does not infer safety from an
//! action label, does not accept a different policy with similar semantics and
//! does not create an execution attempt.

use mycelix_authority_freshness::{AuthoritySubjectKind, AuthoritySubjectRef};
use mycelix_effect_safety::{QualifiedEffectSafetyPolicy, CURRENT_QUALIFICATION_PROFILE};
use mycelix_institutional_core::Digest32;
use mycelix_response_execution_intent::QualifiedResponseExecutionIntent;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-response-effect-safety-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-response-effect-safety-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-response-effect-safety-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/response/effect-safety/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/response/effect-safety/evidence/v1";

/// Non-deserializable composition result. This proves that the exact current
/// safety policy requested by the response applies to the exact already-bound
/// action bytes and adapter profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedResponseEffectSafety {
    protocol_version: String,
    proposal_id: String,
    proposal_digest: String,
    decision_id: String,
    option_id: String,
    action_class: String,
    jurisdiction: Option<String>,
    actions_digest: Digest32,
    actions_digest_profile: String,
    adapter_profile: String,
    executor_authority_ref: String,
    effect_safety_subject: AuthoritySubjectRef,
    effect_safety_qualification_digest: Digest32,
    effect_safety_evidence_digest: Digest32,
    automatic_effects_allowed: bool,
    max_attempt_lease_ms: u64,
    pending_authority_subjects: Vec<AuthoritySubjectRef>,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedResponseEffectSafety {
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

    pub fn actions_digest(&self) -> Digest32 {
        self.actions_digest
    }

    pub fn actions_digest_profile(&self) -> &str {
        &self.actions_digest_profile
    }

    pub fn adapter_profile(&self) -> &str {
        &self.adapter_profile
    }

    pub fn executor_authority_ref(&self) -> &str {
        &self.executor_authority_ref
    }

    pub fn effect_safety_subject(&self) -> &AuthoritySubjectRef {
        &self.effect_safety_subject
    }

    pub fn effect_safety_qualification_digest(&self) -> Digest32 {
        self.effect_safety_qualification_digest
    }

    pub fn effect_safety_evidence_digest(&self) -> Digest32 {
        self.effect_safety_evidence_digest
    }

    pub fn automatic_effects_allowed(&self) -> bool {
        self.automatic_effects_allowed
    }

    pub fn max_attempt_lease_ms(&self) -> u64 {
        self.max_attempt_lease_ms
    }

    pub fn pending_authority_subjects(&self) -> &[AuthoritySubjectRef] {
        &self.pending_authority_subjects
    }

    pub fn all_declared_authority_subjects_satisfied(&self) -> bool {
        self.pending_authority_subjects.is_empty()
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        QUALIFICATION_PROFILE
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn exact_action_and_adapter_safety_bound_here(&self) -> bool {
        true
    }

    pub const fn effect_safety_subject_discharged_here(&self) -> bool {
        true
    }

    /// Receipt/attestation origin is still a later same-invocation runtime fact.
    pub const fn proof_origins_verified_here(&self) -> bool {
        false
    }

    pub const fn deployment_bound_here(&self) -> bool {
        false
    }

    pub const fn attempt_fenced_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Compose one exact response intent with one already-qualified current effect
/// safety policy.
pub fn qualify_response_effect_safety(
    intent: &QualifiedResponseExecutionIntent,
    safety: &QualifiedEffectSafetyPolicy,
    now_ms: u64,
) -> Result<QualifiedResponseEffectSafety, ResponseEffectSafetyError> {
    if now_ms == 0 {
        return Err(ResponseEffectSafetyError::InvalidVerificationTime);
    }
    if intent.provider_valid_until_ms() <= now_ms {
        return Err(ResponseEffectSafetyError::ExecutorAuthorityExpired);
    }
    if safety.verified_at_ms() > now_ms || safety.valid_until_ms() <= now_ms {
        return Err(ResponseEffectSafetyError::EffectSafetyExpired);
    }

    safety
        .subject()
        .validate()
        .map_err(|error| ResponseEffectSafetyError::InvalidEffectSafetySubject(
            error.to_string(),
        ))?;
    if safety.subject().kind != AuthoritySubjectKind::EffectSafetyPolicy {
        return Err(ResponseEffectSafetyError::WrongEffectSafetySubjectKind);
    }

    let exact_pending = intent
        .pending_authority_subjects()
        .iter()
        .any(|subject| subject == safety.subject());
    if !exact_pending {
        return Err(ResponseEffectSafetyError::EffectSafetySubjectNotPending);
    }

    let policy = safety.policy();
    if policy.action_class != intent.action_class() {
        return Err(ResponseEffectSafetyError::ActionClassMismatch {
            expected: intent.action_class().into(),
            actual: policy.action_class.clone(),
        });
    }
    if policy.actions_digest != intent.actions_digest() {
        return Err(ResponseEffectSafetyError::ActionsDigestMismatch);
    }
    if policy.actions_digest_profile != intent.actions_digest_profile() {
        return Err(ResponseEffectSafetyError::ActionsDigestProfileMismatch {
            expected: intent.actions_digest_profile().into(),
            actual: policy.actions_digest_profile.clone(),
        });
    }
    let policy_jurisdiction = policy.jurisdiction.as_ref().map(|value| value.as_str());
    if policy_jurisdiction != intent.jurisdiction() {
        return Err(ResponseEffectSafetyError::JurisdictionMismatch {
            expected: intent.jurisdiction().map(str::to_string),
            actual: policy_jurisdiction.map(str::to_string),
        });
    }
    if policy.adapter_profile != intent.adapter_profile() {
        return Err(ResponseEffectSafetyError::AdapterProfileMismatch {
            expected: intent.adapter_profile().into(),
            actual: policy.adapter_profile.clone(),
        });
    }

    let pending_authority_subjects = intent
        .pending_authority_subjects()
        .iter()
        .filter(|subject| *subject != safety.subject())
        .cloned()
        .collect::<Vec<_>>();

    let subject_identity = safety
        .subject()
        .identity_digest()
        .map_err(|error| ResponseEffectSafetyError::InvalidEffectSafetySubject(
            error.to_string(),
        ))?;
    let qualification_digest = qualification_digest(intent, safety, subject_identity);
    let verified_at_ms = now_ms;
    let valid_until_ms = intent
        .provider_valid_until_ms()
        .min(safety.valid_until_ms());
    if valid_until_ms <= now_ms {
        return Err(ResponseEffectSafetyError::NoUsableQualificationWindow);
    }
    let evidence_digest = evidence_digest(
        qualification_digest,
        safety.evidence_digest(),
        verified_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedResponseEffectSafety {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: intent.proposal_id().into(),
        proposal_digest: intent.proposal_digest().into(),
        decision_id: intent.decision_id().into(),
        option_id: intent.option_id().into(),
        action_class: intent.action_class().into(),
        jurisdiction: intent.jurisdiction().map(str::to_string),
        actions_digest: intent.actions_digest(),
        actions_digest_profile: intent.actions_digest_profile().into(),
        adapter_profile: intent.adapter_profile().into(),
        executor_authority_ref: intent.executor_authority_ref().into(),
        effect_safety_subject: safety.subject().clone(),
        effect_safety_qualification_digest: safety.qualification_digest(),
        effect_safety_evidence_digest: safety.evidence_digest(),
        automatic_effects_allowed: safety.automatic_effects_allowed(),
        max_attempt_lease_ms: policy.max_attempt_lease_ms,
        pending_authority_subjects,
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn qualification_digest(
    intent: &QualifiedResponseExecutionIntent,
    safety: &QualifiedEffectSafetyPolicy,
    subject_identity: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, intent.proposal_id().as_bytes());
    frame(&mut hasher, intent.proposal_digest().as_bytes());
    frame(&mut hasher, intent.decision_id().as_bytes());
    frame(&mut hasher, intent.option_id().as_bytes());
    frame(&mut hasher, &intent.actions_digest().0);
    frame(&mut hasher, intent.actions_digest_profile().as_bytes());
    frame(&mut hasher, intent.adapter_profile().as_bytes());
    frame(&mut hasher, intent.executor_authority_ref().as_bytes());
    frame(&mut hasher, &subject_identity.0);
    frame(&mut hasher, CURRENT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &safety.qualification_digest().0);
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    safety_evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, &safety_evidence_digest.0);
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResponseEffectSafetyError {
    InvalidVerificationTime,
    ExecutorAuthorityExpired,
    EffectSafetyExpired,
    InvalidEffectSafetySubject(String),
    WrongEffectSafetySubjectKind,
    EffectSafetySubjectNotPending,
    ActionClassMismatch { expected: String, actual: String },
    ActionsDigestMismatch,
    ActionsDigestProfileMismatch { expected: String, actual: String },
    JurisdictionMismatch { expected: Option<String>, actual: Option<String> },
    AdapterProfileMismatch { expected: String, actual: String },
    NoUsableQualificationWindow,
}

impl fmt::Display for ResponseEffectSafetyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVerificationTime => write!(f, "response effect-safety verification time must be positive"),
            Self::ExecutorAuthorityExpired => write!(f, "response executor authority lease is expired"),
            Self::EffectSafetyExpired => write!(f, "effect-safety qualification is expired or from the future"),
            Self::InvalidEffectSafetySubject(error) => write!(f, "invalid effect-safety subject: {error}"),
            Self::WrongEffectSafetySubjectKind => write!(f, "qualified safety object does not name an EffectSafetyPolicy subject"),
            Self::EffectSafetySubjectNotPending => write!(f, "qualified effect-safety policy is not the exact pending response safety subject"),
            Self::ActionClassMismatch { expected, actual } => write!(f, "effect-safety action class {actual} does not match response action {expected}"),
            Self::ActionsDigestMismatch => write!(f, "effect-safety policy does not bind the exact response execution action digest"),
            Self::ActionsDigestProfileMismatch { expected, actual } => write!(f, "effect-safety action digest profile {actual} does not match response profile {expected}"),
            Self::JurisdictionMismatch { expected, actual } => write!(f, "effect-safety jurisdiction {actual:?} does not match response jurisdiction {expected:?}"),
            Self::AdapterProfileMismatch { expected, actual } => write!(f, "effect-safety adapter profile {actual} does not match response adapter {expected}"),
            Self::NoUsableQualificationWindow => write!(f, "response effect-safety composition has no usable intersected validity window"),
        }
    }
}

impl std::error::Error for ResponseEffectSafetyError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, AuthorityFreshnessState, ProfiledDigest,
        VerifiedAuthorityFreshness, BUNDLE_IDENTITY_PROFILE,
        PROTOCOL_VERSION as FRESHNESS_PROTOCOL,
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
    use mycelix_effect_safety::{
        qualify_effect_safety_policy, EffectSafetyPolicy,
        VerifiedEffectAdapterQualification, VerifiedEffectSafetyPolicyAdoptionProof,
        VerifiedEffectSafetyPolicyRecordProof, ADAPTER_QUALIFICATION_PROTOCOL,
        POLICY_ADOPTION_PROOF_PROTOCOL, POLICY_IDENTITY_PROFILE, POLICY_RECORD_PROOF_PROTOCOL,
        PROTOCOL_VERSION as SAFETY_PROTOCOL,
    };
    use mycelix_execution_action_digest::ACTIONS_DIGEST_PROFILE_V1;
    use mycelix_governance_current_executor_authority::{
        CURRENT_EXECUTOR_AUTHORITY_PROFILE, THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE,
    };
    use mycelix_governance_executor_lineage::EXECUTOR_LINEAGE_AUTHORITY_PROFILE;
    use mycelix_governance_executor_provider_contract::{
        executor_authority_ref, CurrentExecutorAuthorityProjection,
        VerifiedCurrentExecutorAuthorityReceipt, PROTOCOL_VERSION as PROVIDER_PROTOCOL,
    };
    use mycelix_institutional_core::{
        AuthorityGrantId, CapabilityId, InstitutionId, JurisdictionId, PrincipalId, RulebookId,
        RulebookRef,
    };
    use mycelix_response_authority_binding::{
        prepare_response_authority_resolution, PreparedResponseAuthorityResolution,
        ResponseAuthorityBindingCandidate,
    };
    use mycelix_response_execution_intent::{
        qualify_response_execution_intent, ResponseExecutionIntentCandidate,
        ResponseExecutionIntentError,
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

    fn policy(actions_digest: Digest32, action: &str, jurisdiction: &str, adapter_profile: &str) -> EffectSafetyPolicy {
        EffectSafetyPolicy {
            protocol_version: SAFETY_PROTOCOL.into(),
            policy_id: "effect-safety:cooling-center:v1".into(),
            institution: InstitutionId::new("municipality:johannesburg").unwrap(),
            jurisdiction: Some(JurisdictionId::new(jurisdiction).unwrap()),
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:heat-response").unwrap(),
                version: "1".into(),
                digest: d(21),
            },
            action_class: action.into(),
            actions_digest,
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
            adapter_profile: adapter_profile.into(),
            adapter_release_digest: d(22),
            adapter_release_profile: "mycelix-adapter-release-v1-blake3".into(),
            automatic_effects_allowed: true,
            requires_idempotency: true,
            requires_attempt_fencing: true,
            requires_precondition_fence: true,
            requires_compensation: false,
            max_attempt_lease_ms: 10_000,
            valid_from_ms: 100,
            valid_until_ms: 1_000,
            authority_ref: "institutional-authority:jhb:emergency:v1".into(),
            policy_proof_ref: "policy-adoption:jhb:cooling:v1".into(),
        }
    }

    fn projection(actions_digest: Digest32) -> CurrentExecutorAuthorityProjection {
        CurrentExecutorAuthorityProjection {
            proposal_id: mycelix_governance_authority::ProposalId::new("response:heat:1").unwrap(),
            actions_digest,
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
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

    fn prepared(safety_policy: &EffectSafetyPolicy) -> PreparedResponseAuthorityResolution {
        let p = projection(d(1));
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
                safety_policy.subject_ref().unwrap(),
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

    fn receipt(actions_digest: Digest32) -> VerifiedCurrentExecutorAuthorityReceipt {
        VerifiedCurrentExecutorAuthorityReceipt {
            protocol: PROVIDER_PROTOCOL.into(),
            projection: projection(actions_digest),
            verification_ref: "provider-run:heat:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 500,
        }
    }

    fn intent_for_policy(
        safety_policy: &EffectSafetyPolicy,
        adapter_profile: &str,
    ) -> (QualifiedResponseExecutionIntent, Digest32) {
        let prepared = prepared(safety_policy);
        let candidate = ResponseExecutionIntentCandidate {
            adapter_profile: adapter_profile.into(),
            payload_utf8: "{\"site\":\"library-12\",\"open\":true}".into(),
        };
        let first = qualify_response_execution_intent(&prepared, &receipt(d(99)), &candidate, 200)
            .unwrap_err();
        let digest = match first {
            ResponseExecutionIntentError::ActionsDigestMismatch { actual, .. } => actual,
            other => panic!("unexpected first-pass intent error: {other:?}"),
        };

        // The canonical response envelope does not commit the safety-policy
        // identity, so rebuilding the pending subject with this exact digest does
        // not change the action bytes.
        let mut rebound_policy = safety_policy.clone();
        rebound_policy.actions_digest = digest;
        let rebound = prepared(&rebound_policy);
        let intent = qualify_response_execution_intent(&rebound, &receipt(digest), &candidate, 200)
            .unwrap();
        (intent, digest)
    }

    fn qualified_safety(policy: &EffectSafetyPolicy) -> QualifiedEffectSafetyPolicy {
        let digest = policy.identity_digest().unwrap();
        let record = VerifiedEffectSafetyPolicyRecordProof {
            protocol_version: POLICY_RECORD_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            policy_record_ref: "policy-record:cooling:v1".into(),
            record_proof_ref: "record-proof:cooling:v1".into(),
            record_verifier_ref: "record-verifier:1".into(),
            verification_ref: "record-verification:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 900,
        };
        let adoption = VerifiedEffectSafetyPolicyAdoptionProof {
            protocol_version: POLICY_ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            verified_authority_ref: policy.authority_ref.clone(),
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            authority_verifier_ref: "authority-verifier:1".into(),
            verification_ref: "adoption-verification:1".into(),
            verified_at_ms: 160,
            valid_until_ms: 800,
        };
        let fresh = VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL.into(),
                subject: policy.subject_ref().unwrap(),
                generation: 1,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: 100,
                status_record_ref: "effect-safety-status:1".into(),
            },
            authoritative_source_ref: "effect-safety-source:jhb".into(),
            verification_ref: "freshness-verification:1".into(),
            verified_at_ms: 170,
            lease_until_ms: 700,
        };
        let adapter = VerifiedEffectAdapterQualification {
            protocol_version: ADAPTER_QUALIFICATION_PROTOCOL.into(),
            adapter_profile: policy.adapter_profile.clone(),
            adapter_release_digest: policy.adapter_release_digest,
            adapter_release_profile: policy.adapter_release_profile.clone(),
            deployment_ref: "adapter-deployment:jhb:cooling:1".into(),
            enforcement_ref: "adapter-enforcement:jhb:cooling:1".into(),
            exact_action_digest_enforced: true,
            supports_idempotency: true,
            supports_attempt_fencing: true,
            supports_precondition_fence: true,
            supports_compensation: false,
            external_effects_enabled: true,
            adapter_verifier_ref: "adapter-verifier:1".into(),
            verification_ref: "adapter-verification:1".into(),
            verified_at_ms: 190,
            valid_until_ms: 450,
        };
        qualify_effect_safety_policy(
            policy,
            "policy-record:cooling:v1",
            &record,
            &adoption,
            &fresh,
            &adapter,
            200,
        )
        .unwrap()
    }

    fn exact_pair() -> (QualifiedResponseExecutionIntent, QualifiedEffectSafetyPolicy) {
        let seed = policy(
            d(1),
            "open_cooling_center",
            "Johannesburg",
            "mycelix.municipal.cooling-center.v1",
        );
        let (intent, digest) = intent_for_policy(&seed, &seed.adapter_profile);
        let mut exact = seed;
        exact.actions_digest = digest;
        (intent, qualified_safety(&exact))
    }

    #[test]
    fn exact_current_safety_discharges_only_the_pending_policy() {
        let (intent, safety) = exact_pair();
        let qualified = qualify_response_effect_safety(&intent, &safety, 210).unwrap();
        assert_eq!(qualified.action_class(), "open_cooling_center");
        assert_eq!(qualified.jurisdiction(), Some("Johannesburg"));
        assert_eq!(qualified.actions_digest(), intent.actions_digest());
        assert_eq!(qualified.adapter_profile(), intent.adapter_profile());
        assert!(qualified.pending_authority_subjects().is_empty());
        assert!(qualified.all_declared_authority_subjects_satisfied());
        assert!(qualified.automatic_effects_allowed());
        assert!(qualified.exact_action_and_adapter_safety_bound_here());
        assert!(qualified.effect_safety_subject_discharged_here());
        assert!(!qualified.proof_origins_verified_here());
        assert!(!qualified.deployment_bound_here());
        assert!(!qualified.attempt_fenced_here());
        assert!(!qualified.grants_execution_authority());
        assert_eq!(qualified.valid_until_ms(), 450);
    }

    #[test]
    fn safety_policy_must_be_the_exact_pending_subject() {
        let (intent, _) = exact_pair();
        let mut other = policy(
            intent.actions_digest(),
            "open_cooling_center",
            "Johannesburg",
            "mycelix.municipal.cooling-center.v1",
        );
        other.policy_id = "effect-safety:other".into();
        let safety = qualified_safety(&other);
        assert_eq!(
            qualify_response_effect_safety(&intent, &safety, 210).unwrap_err(),
            ResponseEffectSafetyError::EffectSafetySubjectNotPending
        );
    }

    #[test]
    fn action_class_substitution_denies_even_for_qualified_policy() {
        let seed = policy(
            d(1),
            "release_funds",
            "Johannesburg",
            "mycelix.municipal.cooling-center.v1",
        );
        let (intent, digest) = intent_for_policy(&seed, &seed.adapter_profile);
        let mut mismatched = seed;
        mismatched.actions_digest = digest;
        let safety = qualified_safety(&mismatched);
        assert!(matches!(
            qualify_response_effect_safety(&intent, &safety, 210),
            Err(ResponseEffectSafetyError::ActionClassMismatch { .. })
        ));
    }

    #[test]
    fn jurisdiction_substitution_denies() {
        let seed = policy(
            d(1),
            "open_cooling_center",
            "Pretoria",
            "mycelix.municipal.cooling-center.v1",
        );
        let (intent, digest) = intent_for_policy(&seed, &seed.adapter_profile);
        let mut mismatched = seed;
        mismatched.actions_digest = digest;
        let safety = qualified_safety(&mismatched);
        assert!(matches!(
            qualify_response_effect_safety(&intent, &safety, 210),
            Err(ResponseEffectSafetyError::JurisdictionMismatch { .. })
        ));
    }

    #[test]
    fn adapter_profile_substitution_denies() {
        let seed = policy(
            d(1),
            "open_cooling_center",
            "Johannesburg",
            "mycelix.municipal.other-adapter.v1",
        );
        let (intent, digest) = intent_for_policy(
            &seed,
            "mycelix.municipal.cooling-center.v1",
        );
        let mut mismatched = seed;
        mismatched.actions_digest = digest;
        let safety = qualified_safety(&mismatched);
        assert!(matches!(
            qualify_response_effect_safety(&intent, &safety, 210),
            Err(ResponseEffectSafetyError::AdapterProfileMismatch { .. })
        ));
    }

    #[test]
    fn stale_effect_safety_cannot_be_reused() {
        let (intent, safety) = exact_pair();
        assert_eq!(
            qualify_response_effect_safety(&intent, &safety, safety.valid_until_ms()).unwrap_err(),
            ResponseEffectSafetyError::EffectSafetyExpired
        );
    }
}
