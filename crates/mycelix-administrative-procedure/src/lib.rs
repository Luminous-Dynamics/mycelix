// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure administrative-procedure qualification for Mycelix.
//!
//! ADMIN-001 deliberately reuses `mycelix-institutional-core` for authority,
//! decisions, evidence, rulebooks, challenges/appeals/remedies vocabulary, and
//! principal/institution/jurisdiction identifiers. It does not create a second
//! authority system or a second generic decision type.
//!
//! v0.1 proves only:
//! - exact administrative case identity/context;
//! - monotonic pre-decision structural progression;
//! - exact case -> institutional Decision binding;
//! - exact decider -> AuthorityGrant holder binding;
//! - competent capability/role/evidence qualification at decision time; and
//! - decision issuance only from `ReadyForDecision`.
//!
//! Notice, opportunity-to-respond, required reasons, service, reconsideration,
//! appeal/stay/finality, runtime persistence, cryptographic proof verification,
//! and external effects remain later layers.

use mycelix_institutional_core::{
    AuthorityDecision, AuthorityGrant, AuthorityGrantId, AuthorityRequirement, CapabilityId,
    Decision, DecisionId, EvidenceRef, EvidenceRequirement, InstitutionId, JurisdictionId,
    PrincipalId, PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL_VERSION, RoleId, RulebookRef,
    evaluate_authority,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-administrative-procedure-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_TEXT_BYTES: usize = 4096;
const MAX_REASONS: usize = 256;
const MAX_DECISION_EVIDENCE: usize = 4096;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AdministrativeCaseId(String);

impl AdministrativeCaseId {
    pub fn new(value: impl Into<String>) -> Result<Self, AdministrativeProcedureError> {
        let value = value.into();
        validate_text(&value, "case_id", MAX_ID_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProcedureProfileId(String);

impl ProcedureProfileId {
    pub fn new(value: impl Into<String>) -> Result<Self, AdministrativeProcedureError> {
        let value = value.into();
        validate_text(&value, "procedure_profile", MAX_ID_BYTES)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Structural lifecycle only. A state value is not authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdministrativeCaseState {
    Filed,
    EvidenceOpen { opened_at_ms: u64 },
    ReadyForDecision { ready_at_ms: u64 },
    DecisionIssued {
        decision_id: DecisionId,
        issued_at_ms: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeCase {
    pub protocol_version: String,
    pub id: AdministrativeCaseId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub procedure_profile: ProcedureProfileId,
    pub subject_ref: String,
    pub filed_by: PrincipalId,
    pub filed_at_ms: u64,
    pub state: AdministrativeCaseState,
}

impl AdministrativeCase {
    pub fn validate(&self) -> Result<(), AdministrativeProcedureError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.id.as_str(), "case.id", MAX_ID_BYTES)?;
        validate_text(
            self.procedure_profile.as_str(),
            "case.procedure_profile",
            MAX_ID_BYTES,
        )?;
        validate_text(&self.subject_ref, "case.subject_ref", MAX_TEXT_BYTES)?;
        self.rulebook
            .validate()
            .map_err(|_| AdministrativeProcedureError::InvalidRulebook)?;
        if self.filed_at_ms == 0 {
            return Err(AdministrativeProcedureError::InvalidCaseTime);
        }

        match &self.state {
            AdministrativeCaseState::Filed => {}
            AdministrativeCaseState::EvidenceOpen { opened_at_ms } => {
                if *opened_at_ms < self.filed_at_ms {
                    return Err(AdministrativeProcedureError::StateTimeRegression);
                }
            }
            AdministrativeCaseState::ReadyForDecision { ready_at_ms } => {
                if *ready_at_ms < self.filed_at_ms {
                    return Err(AdministrativeProcedureError::StateTimeRegression);
                }
            }
            AdministrativeCaseState::DecisionIssued { issued_at_ms, .. } => {
                if *issued_at_ms < self.filed_at_ms {
                    return Err(AdministrativeProcedureError::StateTimeRegression);
                }
            }
        }
        Ok(())
    }
}

/// Candidate structural progression before a consequential decision.
///
/// This type cannot represent `ReadyForDecision -> DecisionIssued`; that edge is
/// owned exclusively by `issue_qualified_decision`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PreDecisionTransition {
    pub protocol_version: String,
    pub case_id: AdministrativeCaseId,
    pub next_state: PreDecisionState,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PreDecisionState {
    EvidenceOpen { opened_at_ms: u64 },
    ReadyForDecision { ready_at_ms: u64 },
}

/// Opaque structural proof. It grants no institutional authority.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedPreDecisionTransition {
    successor: AdministrativeCase,
}

impl QualifiedPreDecisionTransition {
    pub fn successor(&self) -> &AdministrativeCase {
        &self.successor
    }

    pub fn into_successor(self) -> AdministrativeCase {
        self.successor
    }

    pub fn grants_authority(&self) -> bool {
        false
    }
}

pub fn qualify_pre_decision_transition(
    current: &AdministrativeCase,
    transition: PreDecisionTransition,
) -> Result<QualifiedPreDecisionTransition, AdministrativeProcedureError> {
    current.validate()?;
    require_protocol(&transition.protocol_version)?;
    if transition.case_id != current.id {
        return Err(AdministrativeProcedureError::CaseMismatch);
    }

    let next_state = match (&current.state, transition.next_state) {
        (
            AdministrativeCaseState::Filed,
            PreDecisionState::EvidenceOpen { opened_at_ms },
        ) if opened_at_ms >= current.filed_at_ms => {
            AdministrativeCaseState::EvidenceOpen { opened_at_ms }
        }
        (
            AdministrativeCaseState::EvidenceOpen { opened_at_ms },
            PreDecisionState::ReadyForDecision { ready_at_ms },
        ) if ready_at_ms >= *opened_at_ms => {
            AdministrativeCaseState::ReadyForDecision { ready_at_ms }
        }
        _ => return Err(AdministrativeProcedureError::InvalidStructuralTransition),
    };

    let successor = AdministrativeCase {
        protocol_version: current.protocol_version.clone(),
        id: current.id.clone(),
        institution: current.institution.clone(),
        jurisdiction: current.jurisdiction.clone(),
        rulebook: current.rulebook.clone(),
        procedure_profile: current.procedure_profile.clone(),
        subject_ref: current.subject_ref.clone(),
        filed_by: current.filed_by.clone(),
        filed_at_ms: current.filed_at_ms,
        state: next_state,
    };
    successor.validate()?;
    Ok(QualifiedPreDecisionTransition { successor })
}

/// Reusable authority requirements for issuing one class of administrative
/// decision under one procedure profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeDecisionPolicy {
    pub protocol_version: String,
    pub procedure_profile: ProcedureProfileId,
    pub required_capability: CapabilityId,
    pub accepted_roles: Vec<RoleId>,
    pub authority_evidence: Vec<EvidenceRequirement>,
}

impl AdministrativeDecisionPolicy {
    pub fn validate(&self) -> Result<(), AdministrativeProcedureError> {
        require_protocol(&self.protocol_version)?;
        validate_text(
            self.procedure_profile.as_str(),
            "policy.procedure_profile",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.required_capability.as_str(),
            "policy.required_capability",
            MAX_ID_BYTES,
        )?;
        let mut roles = BTreeSet::new();
        for role in &self.accepted_roles {
            validate_text(role.as_str(), "policy.accepted_role", MAX_ID_BYTES)?;
            if !roles.insert(role.as_str()) {
                return Err(AdministrativeProcedureError::DuplicateAcceptedRole);
            }
        }
        for requirement in &self.authority_evidence {
            requirement
                .validate()
                .map_err(|_| AdministrativeProcedureError::InvalidAuthorityEvidenceRequirement)?;
        }
        Ok(())
    }
}

/// Administrative binding around the existing institutional `Decision`.
///
/// `Decision` remains the single generic decision vocabulary. This envelope adds
/// only the public-procedure facts that the core decision intentionally does not
/// carry: exact case, jurisdiction, decider, and authority-grant identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeDecisionEnvelope {
    pub protocol_version: String,
    pub case_id: AdministrativeCaseId,
    pub jurisdiction: Option<JurisdictionId>,
    pub decider: PrincipalId,
    pub authority_grant_id: AuthorityGrantId,
    pub decision: Decision,
}

impl AdministrativeDecisionEnvelope {
    pub fn validate_shape(&self) -> Result<(), AdministrativeProcedureError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.case_id.as_str(), "decision.case_id", MAX_ID_BYTES)?;
        if self.decision.protocol_version != INSTITUTIONAL_PROTOCOL_VERSION {
            return Err(AdministrativeProcedureError::InvalidInstitutionalDecision);
        }
        self.decision
            .rulebook
            .validate()
            .map_err(|_| AdministrativeProcedureError::InvalidRulebook)?;
        validate_text(
            &self.decision.subject_ref,
            "decision.subject_ref",
            MAX_TEXT_BYTES,
        )?;
        validate_text(
            &self.decision.outcome_code,
            "decision.outcome_code",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.decision.decision_proof_ref,
            "decision.proof_ref",
            MAX_ID_BYTES,
        )?;
        if self.decision.decided_at_ms == 0 {
            return Err(AdministrativeProcedureError::InvalidDecisionTime);
        }
        if self.decision.reasons.len() > MAX_REASONS {
            return Err(AdministrativeProcedureError::TooManyReasons);
        }
        for reason in &self.decision.reasons {
            validate_text(reason, "decision.reason", MAX_TEXT_BYTES)?;
        }
        if self.decision.evidence.len() > MAX_DECISION_EVIDENCE {
            return Err(AdministrativeProcedureError::TooMuchDecisionEvidence);
        }
        let mut evidence_ids = BTreeSet::new();
        for evidence in &self.decision.evidence {
            evidence
                .validate()
                .map_err(|_| AdministrativeProcedureError::InvalidDecisionEvidence)?;
            if !evidence_ids.insert(evidence.id.as_str()) {
                return Err(AdministrativeProcedureError::DuplicateDecisionEvidence);
            }
        }
        Ok(())
    }
}

/// Opaque positive decision qualification. It is intentionally not Deserialize,
/// Clone, or Copy, and it is not an external-effect capability.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAdministrativeDecision {
    case_id: AdministrativeCaseId,
    institution: InstitutionId,
    jurisdiction: Option<JurisdictionId>,
    rulebook: RulebookRef,
    procedure_profile: ProcedureProfileId,
    decider: PrincipalId,
    authority_grant_id: AuthorityGrantId,
    decision: Decision,
}

impl QualifiedAdministrativeDecision {
    pub fn case_id(&self) -> &AdministrativeCaseId {
        &self.case_id
    }

    pub fn decision(&self) -> &Decision {
        &self.decision
    }

    pub fn decider(&self) -> &PrincipalId {
        &self.decider
    }

    pub fn authority_grant_id(&self) -> &AuthorityGrantId {
        &self.authority_grant_id
    }

    pub fn institution(&self) -> &InstitutionId {
        &self.institution
    }

    pub fn jurisdiction(&self) -> Option<&JurisdictionId> {
        self.jurisdiction.as_ref()
    }

    pub fn rulebook(&self) -> &RulebookRef {
        &self.rulebook
    }

    pub fn procedure_profile(&self) -> &ProcedureProfileId {
        &self.procedure_profile
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_administrative_decision(
    case: &AdministrativeCase,
    envelope: AdministrativeDecisionEnvelope,
    grant: &AuthorityGrant,
    policy: &AdministrativeDecisionPolicy,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedAdministrativeDecision, AdministrativeProcedureError> {
    case.validate()?;
    envelope.validate_shape()?;
    policy.validate()?;
    grant
        .validate()
        .map_err(|_| AdministrativeProcedureError::InvalidAuthorityGrant)?;

    let ready_at_ms = match &case.state {
        AdministrativeCaseState::ReadyForDecision { ready_at_ms } => *ready_at_ms,
        _ => return Err(AdministrativeProcedureError::CaseNotReadyForDecision),
    };

    if envelope.case_id != case.id {
        return Err(AdministrativeProcedureError::CaseMismatch);
    }
    if envelope.jurisdiction != case.jurisdiction {
        return Err(AdministrativeProcedureError::JurisdictionMismatch);
    }
    if envelope.decision.institution != case.institution {
        return Err(AdministrativeProcedureError::InstitutionMismatch);
    }
    if envelope.decision.rulebook != case.rulebook {
        return Err(AdministrativeProcedureError::RulebookMismatch);
    }
    if envelope.decision.subject_ref != case.subject_ref {
        return Err(AdministrativeProcedureError::SubjectMismatch);
    }
    if policy.procedure_profile != case.procedure_profile {
        return Err(AdministrativeProcedureError::ProcedureProfileMismatch);
    }
    if envelope.decision.decided_at_ms < ready_at_ms {
        return Err(AdministrativeProcedureError::DecisionBeforeReadiness);
    }
    if envelope.authority_grant_id != grant.id {
        return Err(AdministrativeProcedureError::AuthorityGrantMismatch);
    }
    if envelope.decider != grant.holder {
        return Err(AdministrativeProcedureError::DeciderIsNotGrantHolder);
    }

    let requirement = AuthorityRequirement {
        institution: case.institution.clone(),
        jurisdiction: case.jurisdiction.clone(),
        required_capabilities: vec![policy.required_capability.clone()],
        accepted_roles: policy.accepted_roles.clone(),
        evidence: policy.authority_evidence.clone(),
        rulebook: case.rulebook.clone(),
    };

    match evaluate_authority(
        grant,
        &requirement,
        authority_evidence,
        envelope.decision.decided_at_ms,
    ) {
        AuthorityDecision::Allow(allowed) => {
            if allowed.grant_id != grant.id || allowed.holder != envelope.decider {
                return Err(AdministrativeProcedureError::AuthorityResultMismatch);
            }
        }
        AuthorityDecision::NeedsEvidence(_) => {
            return Err(AdministrativeProcedureError::MissingAuthorityEvidence);
        }
        AuthorityDecision::Deny(_) => {
            return Err(AdministrativeProcedureError::DecisionAuthorityDenied);
        }
    }

    Ok(QualifiedAdministrativeDecision {
        case_id: case.id.clone(),
        institution: case.institution.clone(),
        jurisdiction: case.jurisdiction.clone(),
        rulebook: case.rulebook.clone(),
        procedure_profile: case.procedure_profile.clone(),
        decider: envelope.decider,
        authority_grant_id: envelope.authority_grant_id,
        decision: envelope.decision,
    })
}

/// Semantic issuance result. This is still not an external effect or durable
/// runtime record; it is the pure successor produced by consuming a qualified
/// administrative decision.
#[derive(Debug, PartialEq, Eq)]
pub struct IssuedAdministrativeDecision {
    successor_case: AdministrativeCase,
    decision: Decision,
    decider: PrincipalId,
    authority_grant_id: AuthorityGrantId,
}

impl IssuedAdministrativeDecision {
    pub fn successor_case(&self) -> &AdministrativeCase {
        &self.successor_case
    }

    pub fn decision(&self) -> &Decision {
        &self.decision
    }

    pub fn decider(&self) -> &PrincipalId {
        &self.decider
    }

    pub fn authority_grant_id(&self) -> &AuthorityGrantId {
        &self.authority_grant_id
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn issue_qualified_decision(
    current: &AdministrativeCase,
    qualified: QualifiedAdministrativeDecision,
) -> Result<IssuedAdministrativeDecision, AdministrativeProcedureError> {
    current.validate()?;
    match &current.state {
        AdministrativeCaseState::ReadyForDecision { ready_at_ms } => {
            if qualified.decision.decided_at_ms < *ready_at_ms {
                return Err(AdministrativeProcedureError::DecisionBeforeReadiness);
            }
        }
        _ => return Err(AdministrativeProcedureError::CaseNotReadyForDecision),
    }

    if qualified.case_id != current.id {
        return Err(AdministrativeProcedureError::CaseMismatch);
    }
    if qualified.institution != current.institution {
        return Err(AdministrativeProcedureError::InstitutionMismatch);
    }
    if qualified.jurisdiction != current.jurisdiction {
        return Err(AdministrativeProcedureError::JurisdictionMismatch);
    }
    if qualified.rulebook != current.rulebook {
        return Err(AdministrativeProcedureError::RulebookMismatch);
    }
    if qualified.procedure_profile != current.procedure_profile {
        return Err(AdministrativeProcedureError::ProcedureProfileMismatch);
    }

    let successor_case = AdministrativeCase {
        protocol_version: current.protocol_version.clone(),
        id: current.id.clone(),
        institution: current.institution.clone(),
        jurisdiction: current.jurisdiction.clone(),
        rulebook: current.rulebook.clone(),
        procedure_profile: current.procedure_profile.clone(),
        subject_ref: current.subject_ref.clone(),
        filed_by: current.filed_by.clone(),
        filed_at_ms: current.filed_at_ms,
        state: AdministrativeCaseState::DecisionIssued {
            decision_id: qualified.decision.id.clone(),
            issued_at_ms: qualified.decision.decided_at_ms,
        },
    };
    successor_case.validate()?;

    Ok(IssuedAdministrativeDecision {
        successor_case,
        decision: qualified.decision,
        decider: qualified.decider,
        authority_grant_id: qualified.authority_grant_id,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdministrativeProcedureError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidRulebook,
    InvalidCaseTime,
    StateTimeRegression,
    InvalidStructuralTransition,
    CaseMismatch,
    DuplicateAcceptedRole,
    InvalidAuthorityEvidenceRequirement,
    InvalidInstitutionalDecision,
    InvalidDecisionTime,
    TooManyReasons,
    TooMuchDecisionEvidence,
    InvalidDecisionEvidence,
    DuplicateDecisionEvidence,
    InvalidAuthorityGrant,
    CaseNotReadyForDecision,
    JurisdictionMismatch,
    InstitutionMismatch,
    RulebookMismatch,
    SubjectMismatch,
    ProcedureProfileMismatch,
    DecisionBeforeReadiness,
    AuthorityGrantMismatch,
    DeciderIsNotGrantHolder,
    MissingAuthorityEvidence,
    DecisionAuthorityDenied,
    AuthorityResultMismatch,
}

impl fmt::Display for AdministrativeProcedureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong administrative procedure protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidRulebook => write!(f, "invalid rulebook"),
            Self::InvalidCaseTime => write!(f, "invalid case filing time"),
            Self::StateTimeRegression => write!(f, "administrative case state time regressed"),
            Self::InvalidStructuralTransition => write!(f, "invalid pre-decision structural transition"),
            Self::CaseMismatch => write!(f, "administrative case identity mismatch"),
            Self::DuplicateAcceptedRole => write!(f, "duplicate accepted decision role"),
            Self::InvalidAuthorityEvidenceRequirement => write!(f, "invalid authority evidence requirement"),
            Self::InvalidInstitutionalDecision => write!(f, "invalid institutional decision protocol"),
            Self::InvalidDecisionTime => write!(f, "invalid decision time"),
            Self::TooManyReasons => write!(f, "too many decision reasons"),
            Self::TooMuchDecisionEvidence => write!(f, "too much decision evidence"),
            Self::InvalidDecisionEvidence => write!(f, "invalid decision evidence"),
            Self::DuplicateDecisionEvidence => write!(f, "duplicate decision evidence identity"),
            Self::InvalidAuthorityGrant => write!(f, "invalid authority grant"),
            Self::CaseNotReadyForDecision => write!(f, "case is not ready for decision"),
            Self::JurisdictionMismatch => write!(f, "jurisdiction mismatch"),
            Self::InstitutionMismatch => write!(f, "institution mismatch"),
            Self::RulebookMismatch => write!(f, "rulebook mismatch"),
            Self::SubjectMismatch => write!(f, "decision subject does not match case subject"),
            Self::ProcedureProfileMismatch => write!(f, "procedure profile mismatch"),
            Self::DecisionBeforeReadiness => write!(f, "decision predates case readiness"),
            Self::AuthorityGrantMismatch => write!(f, "decision envelope references a different authority grant"),
            Self::DeciderIsNotGrantHolder => write!(f, "decider is not the authority grant holder"),
            Self::MissingAuthorityEvidence => write!(f, "required authority evidence is missing"),
            Self::DecisionAuthorityDenied => write!(f, "institutional authority evaluator denied decision authority"),
            Self::AuthorityResultMismatch => write!(f, "authority evaluator result does not match decision envelope"),
        }
    }
}

impl std::error::Error for AdministrativeProcedureError {}

fn require_protocol(version: &str) -> Result<(), AdministrativeProcedureError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(AdministrativeProcedureError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), AdministrativeProcedureError> {
    if value.trim().is_empty() {
        return Err(AdministrativeProcedureError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(AdministrativeProcedureError::TooLong(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{
        AuthorityGrantId, AuthoritySourceKind, AuthoritySourceRef, Digest32, EvidenceId, RulebookId,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn principal(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn institution(value: &str) -> InstitutionId {
        InstitutionId::new(value).unwrap()
    }

    fn jurisdiction(value: &str) -> JurisdictionId {
        JurisdictionId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityId {
        CapabilityId::new(value).unwrap()
    }

    fn role(value: &str) -> RoleId {
        RoleId::new(value).unwrap()
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:permits:v1").unwrap(),
            version: "1".into(),
            digest: digest(7),
        }
    }

    fn case(state: AdministrativeCaseState) -> AdministrativeCase {
        AdministrativeCase {
            protocol_version: PROTOCOL_VERSION.into(),
            id: AdministrativeCaseId::new("case:permit:001").unwrap(),
            institution: institution("institution:city"),
            jurisdiction: Some(jurisdiction("jurisdiction:city")),
            rulebook: rulebook(),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            subject_ref: "permit-application:001".into(),
            filed_by: principal("did:example:applicant"),
            filed_at_ms: 1_000,
            state,
        }
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            holder: principal("did:example:officer"),
            institution: institution("institution:city"),
            jurisdiction: Some(jurisdiction("jurisdiction:city")),
            roles: vec![role("role:permit-officer")],
            capabilities: vec![capability("administration.decide")],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "credential:office:permit-officer".into(),
                proof_ref: "proof:office".into(),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 10_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant".into(),
        }
    }

    fn policy() -> AdministrativeDecisionPolicy {
        AdministrativeDecisionPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            required_capability: capability("administration.decide"),
            accepted_roles: vec![role("role:permit-officer")],
            authority_evidence: vec![],
        }
    }

    fn decision() -> Decision {
        Decision {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: DecisionId::new("decision:permit:001").unwrap(),
            institution: institution("institution:city"),
            rulebook: rulebook(),
            subject_ref: "permit-application:001".into(),
            outcome_code: "granted".into(),
            reasons: vec!["Meets the v1 structural criteria".into()],
            evidence: vec![EvidenceRef {
                id: EvidenceId::new("evidence:application:001").unwrap(),
                evidence_type: "permit-application".into(),
                issuer: Some(principal("did:example:applicant")),
                digest: Some(digest(9)),
                observed_at_ms: 1_100,
                proof_ref: Some("proof:evidence".into()),
            }],
            advisory_inputs: vec![],
            decided_at_ms: 3_000,
            decision_proof_ref: "proof:decision".into(),
        }
    }

    fn envelope() -> AdministrativeDecisionEnvelope {
        AdministrativeDecisionEnvelope {
            protocol_version: PROTOCOL_VERSION.into(),
            case_id: AdministrativeCaseId::new("case:permit:001").unwrap(),
            jurisdiction: Some(jurisdiction("jurisdiction:city")),
            decider: principal("did:example:officer"),
            authority_grant_id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            decision: decision(),
        }
    }

    fn ready_case() -> AdministrativeCase {
        case(AdministrativeCaseState::ReadyForDecision { ready_at_ms: 2_000 })
    }

    #[test]
    fn structural_progression_cannot_skip_evidence_open() {
        let filed = case(AdministrativeCaseState::Filed);
        let attempted = PreDecisionTransition {
            protocol_version: PROTOCOL_VERSION.into(),
            case_id: filed.id.clone(),
            next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 2_000 },
        };
        assert_eq!(
            qualify_pre_decision_transition(&filed, attempted).unwrap_err(),
            AdministrativeProcedureError::InvalidStructuralTransition
        );
    }

    #[test]
    fn structural_progression_reaches_ready_without_granting_authority() {
        let filed = case(AdministrativeCaseState::Filed);
        let open = qualify_pre_decision_transition(
            &filed,
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: filed.id.clone(),
                next_state: PreDecisionState::EvidenceOpen { opened_at_ms: 1_500 },
            },
        )
        .unwrap();
        assert!(!open.grants_authority());

        let open_case = open.into_successor();
        let ready = qualify_pre_decision_transition(
            &open_case,
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: open_case.id.clone(),
                next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 2_000 },
            },
        )
        .unwrap();
        assert!(matches!(
            &ready.successor().state,
            AdministrativeCaseState::ReadyForDecision { .. }
        ));
    }

    #[test]
    fn competent_decider_can_qualify_and_issue_decision() {
        let ready = ready_case();
        let qualified = qualify_administrative_decision(
            &ready,
            envelope(),
            &grant(),
            &policy(),
            &[],
        )
        .unwrap();
        assert!(!qualified.grants_external_effect_authority());
        let issued = issue_qualified_decision(&ready, qualified).unwrap();
        assert!(matches!(
            &issued.successor_case().state,
            AdministrativeCaseState::DecisionIssued { .. }
        ));
        assert!(!issued.grants_external_effect_authority());
    }

    #[test]
    fn decision_before_ready_state_is_rejected() {
        let open = case(AdministrativeCaseState::EvidenceOpen { opened_at_ms: 1_500 });
        assert_eq!(
            qualify_administrative_decision(&open, envelope(), &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::CaseNotReadyForDecision
        );
    }

    #[test]
    fn wrong_institution_is_rejected() {
        let ready = ready_case();
        let mut candidate = envelope();
        candidate.decision.institution = institution("institution:other");
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::InstitutionMismatch
        );
    }

    #[test]
    fn wrong_jurisdiction_is_rejected() {
        let ready = ready_case();
        let mut candidate = envelope();
        candidate.jurisdiction = Some(jurisdiction("jurisdiction:other"));
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::JurisdictionMismatch
        );
    }

    #[test]
    fn wrong_subject_is_rejected() {
        let ready = ready_case();
        let mut candidate = envelope();
        candidate.decision.subject_ref = "permit-application:other".into();
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::SubjectMismatch
        );
    }

    #[test]
    fn wrong_decider_is_rejected_even_with_valid_grant() {
        let ready = ready_case();
        let mut candidate = envelope();
        candidate.decider = principal("did:example:intruder");
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::DeciderIsNotGrantHolder
        );
    }

    #[test]
    fn missing_decision_capability_is_rejected() {
        let ready = ready_case();
        let mut bad_grant = grant();
        bad_grant.capabilities = vec![capability("administration.observe")];
        assert_eq!(
            qualify_administrative_decision(&ready, envelope(), &bad_grant, &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::DecisionAuthorityDenied
        );
    }

    #[test]
    fn expired_grant_is_rejected_at_decision_time() {
        let ready = ready_case();
        let mut expired = grant();
        expired.expires_at_ms = 2_500;
        assert_eq!(
            qualify_administrative_decision(&ready, envelope(), &expired, &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::DecisionAuthorityDenied
        );
    }

    #[test]
    fn duplicate_decision_evidence_is_rejected() {
        let ready = ready_case();
        let mut candidate = envelope();
        let duplicate = candidate.decision.evidence[0].clone();
        candidate.decision.evidence.push(duplicate);
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::DuplicateDecisionEvidence
        );
    }

    #[test]
    fn decision_predating_readiness_is_rejected() {
        let ready = ready_case();
        let mut candidate = envelope();
        candidate.decision.decided_at_ms = 1_900;
        assert_eq!(
            qualify_administrative_decision(&ready, candidate, &grant(), &policy(), &[])
                .unwrap_err(),
            AdministrativeProcedureError::DecisionBeforeReadiness
        );
    }

    #[test]
    fn authority_evidence_requirement_fails_closed() {
        let ready = ready_case();
        let mut p = policy();
        p.authority_evidence.push(EvidenceRequirement {
            evidence_type: "office-appointment-current".into(),
            accepted_issuers: vec![],
        });
        assert_eq!(
            qualify_administrative_decision(&ready, envelope(), &grant(), &p, &[])
                .unwrap_err(),
            AdministrativeProcedureError::MissingAuthorityEvidence
        );
    }

    #[test]
    fn case_wire_round_trip_preserves_state() {
        let original = ready_case();
        let encoded = serde_json::to_string(&original).unwrap();
        let decoded: AdministrativeCase = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, original);
    }
}
