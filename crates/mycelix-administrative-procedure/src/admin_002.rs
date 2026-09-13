// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! ADMIN-002 procedural-completeness qualification.
//!
//! This module is exposed only by the `procedural-completeness` feature. Under
//! that feature the crate root hides ADMIN-001's raw consequential decision and
//! issuance functions and re-exports the completeness-gated functions here.

use super::{
    legacy, AdministrativeCase, AdministrativeCaseId, AdministrativeCaseState,
    AdministrativeDecisionEnvelope, AdministrativeProcedureError, AdministrativeQualificationError,
    IssuedAdministrativeDecision, ProcedureProfileId, QualifiedAdministrativeCaseLineage,
    QualifiedAdministrativeDecision,
};
use mycelix_institutional_core::{AuthorityGrant, Digest32, EvidenceRef, PrincipalId};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const ADMIN_002_PROTOCOL_VERSION: &str =
    "mycelix-administrative-procedure-completeness-v0.1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_PARTIES: usize = 64;
const MAX_EVIDENCE: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResponseMode {
    Written,
    Hearing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResponseModeRequirement {
    None,
    Written,
    Hearing,
    WrittenOrHearing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReasonsRequirement {
    NotRequiredByProfile,
    AtLeastOne,
}

/// Exact profile reference used to interpret procedural-completeness evidence.
///
/// `policy_digest` is an externally supplied immutable content identity. This
/// pure kernel retains the whole policy object but does not claim that the
/// caller proved it is the institution's current authoritative profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProceduralCompletenessPolicy {
    pub protocol_version: String,
    pub procedure_profile: ProcedureProfileId,
    pub policy_ref: String,
    pub policy_digest: Digest32,
    pub policy_digest_profile: String,
    pub required_notice_recipients: Vec<PrincipalId>,
    pub required_response_recipients: Vec<PrincipalId>,
    pub response_mode: ResponseModeRequirement,
    pub min_response_window_ms: u64,
    pub reasons: ReasonsRequirement,
}

impl ProceduralCompletenessPolicy {
    pub fn validate(&self) -> Result<(), ProceduralCompletenessError> {
        require_protocol(&self.protocol_version)?;
        validate_ref(&self.policy_ref)?;
        validate_profile(&self.policy_digest_profile)?;
        if self.policy_digest.is_zero() {
            return Err(ProceduralCompletenessError::ZeroPolicyDigest);
        }
        if self.required_notice_recipients.len() > MAX_PARTIES
            || self.required_response_recipients.len() > MAX_PARTIES
        {
            return Err(ProceduralCompletenessError::TooManyParties);
        }

        let notices = exact_principal_set(&self.required_notice_recipients)?;
        let responses = exact_principal_set(&self.required_response_recipients)?;
        if !responses.is_subset(&notices) {
            return Err(ProceduralCompletenessError::ResponseRecipientWithoutNotice);
        }

        match self.response_mode {
            ResponseModeRequirement::None => {
                if !responses.is_empty() || self.min_response_window_ms != 0 {
                    return Err(ProceduralCompletenessError::InconsistentResponsePolicy);
                }
            }
            _ => {
                if responses.is_empty() || self.min_response_window_ms == 0 {
                    return Err(ProceduralCompletenessError::InconsistentResponsePolicy);
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct NoticeReceipt {
    pub protocol_version: String,
    pub case_id: AdministrativeCaseId,
    pub recipient: PrincipalId,
    pub served_at_ms: u64,
    pub content_digest: Digest32,
    pub proof_ref: String,
}

impl NoticeReceipt {
    fn validate(&self) -> Result<(), ProceduralCompletenessError> {
        require_protocol(&self.protocol_version)?;
        validate_principal(&self.recipient)?;
        if self.served_at_ms == 0 {
            return Err(ProceduralCompletenessError::InvalidNoticeTime);
        }
        if self.content_digest.is_zero() {
            return Err(ProceduralCompletenessError::ZeroNoticeDigest);
        }
        validate_ref(&self.proof_ref)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResponseOpportunityReceipt {
    pub protocol_version: String,
    pub case_id: AdministrativeCaseId,
    pub recipient: PrincipalId,
    pub mode: ResponseMode,
    pub opened_at_ms: u64,
    pub closed_at_ms: u64,
    pub proof_ref: String,
}

impl ResponseOpportunityReceipt {
    fn validate(&self) -> Result<(), ProceduralCompletenessError> {
        require_protocol(&self.protocol_version)?;
        validate_principal(&self.recipient)?;
        if self.opened_at_ms == 0 || self.closed_at_ms <= self.opened_at_ms {
            return Err(ProceduralCompletenessError::InvalidResponseWindow);
        }
        validate_ref(&self.proof_ref)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceClosureReceipt {
    pub protocol_version: String,
    pub case_id: AdministrativeCaseId,
    pub closed_at_ms: u64,
    /// Exact decision-evidence cut. Vector order has no semantics.
    pub decision_evidence: Vec<EvidenceRef>,
    pub proof_ref: String,
}

impl EvidenceClosureReceipt {
    fn validate(&self) -> Result<(), ProceduralCompletenessError> {
        require_protocol(&self.protocol_version)?;
        if self.closed_at_ms == 0 {
            return Err(ProceduralCompletenessError::InvalidEvidenceClosureTime);
        }
        validate_ref(&self.proof_ref)?;
        let evidence = exact_evidence_map(&self.decision_evidence)?;
        if evidence
            .values()
            .any(|item| item.observed_at_ms > self.closed_at_ms)
        {
            return Err(ProceduralCompletenessError::EvidenceObservedAfterClosure);
        }
        Ok(())
    }
}

/// Opaque proof that one ADMIN-001 lineage satisfies one exact captured
/// procedural-completeness policy.
///
/// Deliberately not Clone/Serialize/Deserialize.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedProcedurallyCompleteCase {
    lineage: QualifiedAdministrativeCaseLineage,
    policy: ProceduralCompletenessPolicy,
    evidence_closure: EvidenceClosureReceipt,
}

impl QualifiedProcedurallyCompleteCase {
    pub fn case(&self) -> &AdministrativeCase {
        self.lineage.current_case()
    }

    pub fn policy(&self) -> &ProceduralCompletenessPolicy {
        &self.policy
    }

    pub fn evidence_closure(&self) -> &EvidenceClosureReceipt {
        &self.evidence_closure
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Composite positive result retaining both completeness and ADMIN-001 authority
/// qualification so completeness cannot be floated onto another decision.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedProceduralDecision {
    complete_case: QualifiedProcedurallyCompleteCase,
    inner: QualifiedAdministrativeDecision,
}

impl QualifiedProceduralDecision {
    pub fn complete_case(&self) -> &QualifiedProcedurallyCompleteCase {
        &self.complete_case
    }

    pub fn decision(&self) -> &mycelix_institutional_core::Decision {
        self.inner.decision()
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_procedural_completeness(
    lineage: QualifiedAdministrativeCaseLineage,
    policy: ProceduralCompletenessPolicy,
    notices: &[NoticeReceipt],
    responses: &[ResponseOpportunityReceipt],
    evidence_closure: EvidenceClosureReceipt,
) -> Result<QualifiedProcedurallyCompleteCase, ProceduralCompletenessError> {
    policy.validate()?;
    evidence_closure.validate()?;

    let case = lineage.current_case();
    case.validate()
        .map_err(ProceduralCompletenessError::Administrative)?;
    let ready_at_ms = match &case.state {
        AdministrativeCaseState::ReadyForDecision { ready_at_ms } => *ready_at_ms,
        _ => return Err(ProceduralCompletenessError::CaseNotReadyForCompleteness),
    };

    if policy.procedure_profile != case.procedure_profile {
        return Err(ProceduralCompletenessError::ProcedureProfileMismatch);
    }
    if evidence_closure.case_id != case.id {
        return Err(ProceduralCompletenessError::CaseMismatch);
    }
    if evidence_closure.closed_at_ms < case.filed_at_ms
        || evidence_closure.closed_at_ms > ready_at_ms
    {
        return Err(ProceduralCompletenessError::InvalidEvidenceClosureTime);
    }

    if notices.len() > MAX_PARTIES || responses.len() > MAX_PARTIES {
        return Err(ProceduralCompletenessError::TooManyParties);
    }

    let required_notices = exact_principal_set(&policy.required_notice_recipients)?;
    let required_responses = exact_principal_set(&policy.required_response_recipients)?;

    let mut notice_by_recipient = BTreeMap::<String, &NoticeReceipt>::new();
    for notice in notices {
        notice.validate()?;
        if notice.case_id != case.id {
            return Err(ProceduralCompletenessError::CaseMismatch);
        }
        if notice.served_at_ms < case.filed_at_ms || notice.served_at_ms > ready_at_ms {
            return Err(ProceduralCompletenessError::InvalidNoticeTime);
        }
        let key = notice.recipient.as_str().to_owned();
        if notice_by_recipient.insert(key, notice).is_some() {
            return Err(ProceduralCompletenessError::DuplicateNoticeRecipient);
        }
    }
    if notice_by_recipient.keys().cloned().collect::<BTreeSet<_>>() != required_notices {
        return Err(ProceduralCompletenessError::NoticeSetMismatch);
    }

    let mut response_by_recipient = BTreeMap::<String, &ResponseOpportunityReceipt>::new();
    for response in responses {
        response.validate()?;
        if response.case_id != case.id {
            return Err(ProceduralCompletenessError::CaseMismatch);
        }
        if response.opened_at_ms < case.filed_at_ms {
            return Err(ProceduralCompletenessError::InvalidResponseWindow);
        }
        if response.closed_at_ms > evidence_closure.closed_at_ms
            || response.closed_at_ms > ready_at_ms
        {
            return Err(ProceduralCompletenessError::ResponseAfterEvidenceClosure);
        }
        if response.closed_at_ms - response.opened_at_ms < policy.min_response_window_ms {
            return Err(ProceduralCompletenessError::ResponseWindowTooShort);
        }
        if !response_mode_satisfies(policy.response_mode, response.mode) {
            return Err(ProceduralCompletenessError::ResponseModeMismatch);
        }
        let notice = notice_by_recipient
            .get(response.recipient.as_str())
            .ok_or(ProceduralCompletenessError::ResponseRecipientWithoutNotice)?;
        if notice.served_at_ms > response.opened_at_ms {
            return Err(ProceduralCompletenessError::ResponseOpenedBeforeNotice);
        }
        let key = response.recipient.as_str().to_owned();
        if response_by_recipient.insert(key, response).is_some() {
            return Err(ProceduralCompletenessError::DuplicateResponseRecipient);
        }
    }
    if response_by_recipient
        .keys()
        .cloned()
        .collect::<BTreeSet<_>>()
        != required_responses
    {
        return Err(ProceduralCompletenessError::ResponseSetMismatch);
    }

    Ok(QualifiedProcedurallyCompleteCase {
        lineage,
        policy,
        evidence_closure,
    })
}

/// Completeness-gated consequential decision qualification.
pub fn qualify_administrative_decision(
    complete_case: QualifiedProcedurallyCompleteCase,
    envelope: AdministrativeDecisionEnvelope,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedProceduralDecision, ProceduralCompletenessError> {
    let closed_evidence = exact_evidence_map(&complete_case.evidence_closure.decision_evidence)?;
    let decision_evidence = exact_evidence_map(&envelope.decision.evidence)?;
    if closed_evidence != decision_evidence {
        return Err(ProceduralCompletenessError::DecisionEvidenceCutMismatch);
    }

    if matches!(complete_case.policy.reasons, ReasonsRequirement::AtLeastOne)
        && envelope.decision.reasons.is_empty()
    {
        return Err(ProceduralCompletenessError::RequiredReasonsMissing);
    }

    let inner = legacy::qualify_administrative_decision(
        complete_case.lineage.current_case(),
        envelope,
        grant,
        complete_case.lineage.decision_policy(),
        authority_evidence,
    )
    .map_err(|error| {
        ProceduralCompletenessError::Qualification(AdministrativeQualificationError::from(error))
    })?;

    Ok(QualifiedProceduralDecision {
        complete_case,
        inner,
    })
}

pub fn issue_qualified_decision(
    qualified: QualifiedProceduralDecision,
) -> Result<IssuedAdministrativeDecision, ProceduralCompletenessError> {
    let QualifiedProceduralDecision {
        complete_case,
        inner,
    } = qualified;
    legacy::issue_qualified_decision(complete_case.lineage.current_case(), inner).map_err(|error| {
        ProceduralCompletenessError::Qualification(AdministrativeQualificationError::from(error))
    })
}

fn exact_principal_set(
    principals: &[PrincipalId],
) -> Result<BTreeSet<String>, ProceduralCompletenessError> {
    let mut out = BTreeSet::new();
    for principal in principals {
        validate_principal(principal)?;
        if !out.insert(principal.as_str().to_owned()) {
            return Err(ProceduralCompletenessError::DuplicatePolicyRecipient);
        }
    }
    Ok(out)
}

fn exact_evidence_map(
    evidence: &[EvidenceRef],
) -> Result<BTreeMap<String, EvidenceRef>, ProceduralCompletenessError> {
    if evidence.len() > MAX_EVIDENCE {
        return Err(ProceduralCompletenessError::TooMuchEvidence);
    }
    let mut out = BTreeMap::new();
    for item in evidence {
        item.validate()
            .map_err(|_| ProceduralCompletenessError::InvalidEvidence)?;
        let key = item.id.as_str().to_owned();
        if out.insert(key, item.clone()).is_some() {
            return Err(ProceduralCompletenessError::DuplicateEvidenceIdentity);
        }
    }
    Ok(out)
}

fn response_mode_satisfies(requirement: ResponseModeRequirement, actual: ResponseMode) -> bool {
    match requirement {
        ResponseModeRequirement::None => false,
        ResponseModeRequirement::Written => actual == ResponseMode::Written,
        ResponseModeRequirement::Hearing => actual == ResponseMode::Hearing,
        ResponseModeRequirement::WrittenOrHearing => true,
    }
}

fn require_protocol(value: &str) -> Result<(), ProceduralCompletenessError> {
    if value == ADMIN_002_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProceduralCompletenessError::WrongProtocolVersion)
    }
}

fn validate_principal(principal: &PrincipalId) -> Result<(), ProceduralCompletenessError> {
    validate_ref(principal.as_str())
}

fn validate_ref(value: &str) -> Result<(), ProceduralCompletenessError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ProceduralCompletenessError::InvalidReference)
    } else {
        Ok(())
    }
}

fn validate_profile(value: &str) -> Result<(), ProceduralCompletenessError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ProceduralCompletenessError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProceduralCompletenessError {
    Administrative(AdministrativeProcedureError),
    Qualification(AdministrativeQualificationError),
    WrongProtocolVersion,
    InvalidReference,
    InvalidProfile,
    ZeroPolicyDigest,
    ZeroNoticeDigest,
    TooManyParties,
    TooMuchEvidence,
    DuplicatePolicyRecipient,
    DuplicateNoticeRecipient,
    DuplicateResponseRecipient,
    DuplicateEvidenceIdentity,
    InvalidEvidence,
    EvidenceObservedAfterClosure,
    ResponseRecipientWithoutNotice,
    InconsistentResponsePolicy,
    CaseNotReadyForCompleteness,
    ProcedureProfileMismatch,
    CaseMismatch,
    InvalidNoticeTime,
    InvalidResponseWindow,
    InvalidEvidenceClosureTime,
    NoticeSetMismatch,
    ResponseSetMismatch,
    ResponseModeMismatch,
    ResponseWindowTooShort,
    ResponseOpenedBeforeNotice,
    ResponseAfterEvidenceClosure,
    DecisionEvidenceCutMismatch,
    RequiredReasonsMissing,
}

impl fmt::Display for ProceduralCompletenessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Administrative(error) => write!(f, "{error}"),
            Self::Qualification(error) => write!(f, "{error}"),
            Self::WrongProtocolVersion => write!(f, "wrong ADMIN-002 protocol version"),
            Self::InvalidReference => write!(f, "invalid procedural evidence reference"),
            Self::InvalidProfile => write!(f, "invalid procedural policy digest profile"),
            Self::ZeroPolicyDigest => write!(f, "procedural policy digest must be non-zero"),
            Self::ZeroNoticeDigest => write!(f, "notice content digest must be non-zero"),
            Self::TooManyParties => write!(f, "procedural party count exceeds v0.1 bound"),
            Self::TooMuchEvidence => write!(f, "decision evidence count exceeds v0.1 bound"),
            Self::DuplicatePolicyRecipient => write!(f, "duplicate recipient in procedure policy"),
            Self::DuplicateNoticeRecipient => write!(f, "duplicate notice recipient"),
            Self::DuplicateResponseRecipient => write!(f, "duplicate response recipient"),
            Self::DuplicateEvidenceIdentity => write!(f, "duplicate evidence identity"),
            Self::InvalidEvidence => write!(f, "invalid decision evidence"),
            Self::EvidenceObservedAfterClosure => {
                write!(f, "closed evidence contains an observation after closure")
            }
            Self::ResponseRecipientWithoutNotice => {
                write!(f, "response recipient lacks required notice")
            }
            Self::InconsistentResponsePolicy => write!(f, "inconsistent response policy"),
            Self::CaseNotReadyForCompleteness => {
                write!(f, "case is not ready for procedural completeness")
            }
            Self::ProcedureProfileMismatch => write!(f, "procedure profile mismatch"),
            Self::CaseMismatch => write!(f, "procedural evidence belongs to another case"),
            Self::InvalidNoticeTime => write!(f, "invalid notice service time"),
            Self::InvalidResponseWindow => write!(f, "invalid response opportunity window"),
            Self::InvalidEvidenceClosureTime => write!(f, "invalid evidence closure time"),
            Self::NoticeSetMismatch => write!(f, "notice recipients do not match exact policy set"),
            Self::ResponseSetMismatch => {
                write!(f, "response recipients do not match exact policy set")
            }
            Self::ResponseModeMismatch => write!(f, "response mode does not satisfy policy"),
            Self::ResponseWindowTooShort => write!(f, "response window is shorter than policy"),
            Self::ResponseOpenedBeforeNotice => {
                write!(f, "response opportunity opened before notice")
            }
            Self::ResponseAfterEvidenceClosure => {
                write!(f, "response opportunity closes after evidence closure")
            }
            Self::DecisionEvidenceCutMismatch => {
                write!(f, "decision evidence differs from the exact closed evidence cut")
            }
            Self::RequiredReasonsMissing => write!(f, "required decision reasons are missing"),
        }
    }
}

impl std::error::Error for ProceduralCompletenessError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AdministrativeDecisionPolicy, PreDecisionState, PreDecisionTransition, qualify_case_lineage,
    };
    use mycelix_institutional_core::{
        AuthorityGrantId, AuthoritySourceKind, AuthoritySourceRef, CapabilityId, Decision,
        DecisionId, EvidenceId, InstitutionId, JurisdictionId,
        PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL_VERSION, RoleId, RulebookId, RulebookRef,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn principal(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn institution() -> InstitutionId {
        InstitutionId::new("institution:city").unwrap()
    }

    fn jurisdiction() -> JurisdictionId {
        JurisdictionId::new("jurisdiction:city").unwrap()
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administration:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(7),
        }
    }

    fn case_id() -> AdministrativeCaseId {
        AdministrativeCaseId::new("case:1").unwrap()
    }

    fn procedure_profile() -> ProcedureProfileId {
        ProcedureProfileId::new("procedure:permit:v1").unwrap()
    }

    fn filed_case() -> AdministrativeCase {
        AdministrativeCase {
            protocol_version: super::super::PROTOCOL_VERSION.into(),
            id: case_id(),
            institution: institution(),
            jurisdiction: Some(jurisdiction()),
            rulebook: rulebook(),
            procedure_profile: procedure_profile(),
            subject_ref: "permit:parcel:42".into(),
            filed_by: principal("did:example:applicant"),
            filed_at_ms: 1_000,
            state: AdministrativeCaseState::Filed,
        }
    }

    fn decision_policy() -> AdministrativeDecisionPolicy {
        AdministrativeDecisionPolicy {
            protocol_version: super::super::PROTOCOL_VERSION.into(),
            procedure_profile: procedure_profile(),
            required_capability: CapabilityId::new("administration.decide").unwrap(),
            accepted_roles: vec![RoleId::new("role:permit-officer").unwrap()],
            authority_evidence: vec![],
        }
    }

    fn lineage() -> QualifiedAdministrativeCaseLineage {
        let transitions = vec![
            PreDecisionTransition {
                protocol_version: super::super::PROTOCOL_VERSION.into(),
                case_id: case_id(),
                next_state: PreDecisionState::EvidenceOpen {
                    opened_at_ms: 2_000,
                },
            },
            PreDecisionTransition {
                protocol_version: super::super::PROTOCOL_VERSION.into(),
                case_id: case_id(),
                next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 5_000 },
            },
        ];
        qualify_case_lineage(filed_case(), &transitions, decision_policy()).unwrap()
    }

    fn evidence() -> EvidenceRef {
        EvidenceRef {
            id: EvidenceId::new("evidence:inspection:1").unwrap(),
            evidence_type: "inspection-report".into(),
            issuer: Some(principal("did:example:inspector")),
            digest: Some(d(9)),
            observed_at_ms: 3_000,
            proof_ref: Some("proof:evidence:1".into()),
        }
    }

    fn policy() -> ProceduralCompletenessPolicy {
        ProceduralCompletenessPolicy {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            procedure_profile: procedure_profile(),
            policy_ref: "procedure-policy:permit:v1".into(),
            policy_digest: d(4),
            policy_digest_profile: "mycelix-procedure-policy-v1-blake3".into(),
            required_notice_recipients: vec![principal("did:example:applicant")],
            required_response_recipients: vec![principal("did:example:applicant")],
            response_mode: ResponseModeRequirement::WrittenOrHearing,
            min_response_window_ms: 1_000,
            reasons: ReasonsRequirement::AtLeastOne,
        }
    }

    fn notice() -> NoticeReceipt {
        NoticeReceipt {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id(),
            recipient: principal("did:example:applicant"),
            served_at_ms: 1_500,
            content_digest: d(5),
            proof_ref: "proof:notice:1".into(),
        }
    }

    fn response() -> ResponseOpportunityReceipt {
        ResponseOpportunityReceipt {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id(),
            recipient: principal("did:example:applicant"),
            mode: ResponseMode::Written,
            opened_at_ms: 2_000,
            closed_at_ms: 4_000,
            proof_ref: "proof:response-window:1".into(),
        }
    }

    fn closure() -> EvidenceClosureReceipt {
        EvidenceClosureReceipt {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id(),
            closed_at_ms: 4_500,
            decision_evidence: vec![evidence()],
            proof_ref: "proof:evidence-closure:1".into(),
        }
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            holder: principal("did:example:officer"),
            institution: institution(),
            jurisdiction: Some(jurisdiction()),
            roles: vec![RoleId::new("role:permit-officer").unwrap()],
            capabilities: vec![CapabilityId::new("administration.decide").unwrap()],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "credential:permit-officer:1".into(),
                proof_ref: "proof:credential:1".into(),
            }],
            issued_at_ms: 500,
            expires_at_ms: 10_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant:1".into(),
        }
    }

    fn envelope() -> AdministrativeDecisionEnvelope {
        AdministrativeDecisionEnvelope {
            protocol_version: super::super::PROTOCOL_VERSION.into(),
            case_id: case_id(),
            jurisdiction: Some(jurisdiction()),
            decider: principal("did:example:officer"),
            authority_grant_id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            decision: Decision {
                protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                id: DecisionId::new("decision:1").unwrap(),
                institution: institution(),
                rulebook: rulebook(),
                subject_ref: "permit:parcel:42".into(),
                outcome_code: "granted".into(),
                reasons: vec!["requirements satisfied".into()],
                evidence: vec![evidence()],
                advisory_inputs: vec![],
                decided_at_ms: 6_000,
                decision_proof_ref: "proof:decision:1".into(),
            },
        }
    }

    fn complete() -> QualifiedProcedurallyCompleteCase {
        qualify_procedural_completeness(lineage(), policy(), &[notice()], &[response()], closure())
            .unwrap()
    }

    #[test]
    fn exact_procedure_can_reach_competent_decision() {
        let complete = complete();
        assert!(!complete.grants_authority());
        assert!(!complete.grants_external_effect_authority());
        let qualified = qualify_administrative_decision(complete, envelope(), &grant(), &[]).unwrap();
        assert!(!qualified.grants_external_effect_authority());
        let issued = issue_qualified_decision(qualified).unwrap();
        assert!(matches!(
            issued.successor_case().state,
            AdministrativeCaseState::DecisionIssued { .. }
        ));
    }

    #[test]
    fn missing_notice_fails_closed() {
        assert_eq!(
            qualify_procedural_completeness(lineage(), policy(), &[], &[response()], closure())
                .unwrap_err(),
            ProceduralCompletenessError::NoticeSetMismatch
        );
    }

    #[test]
    fn extra_notice_fails_closed() {
        let mut extra = notice();
        extra.recipient = principal("did:example:other");
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[notice(), extra],
                &[response()],
                closure(),
            )
            .unwrap_err(),
            ProceduralCompletenessError::NoticeSetMismatch
        );
    }

    #[test]
    fn short_response_window_fails_closed() {
        let mut short = response();
        short.closed_at_ms = 2_500;
        assert_eq!(
            qualify_procedural_completeness(lineage(), policy(), &[notice()], &[short], closure())
                .unwrap_err(),
            ProceduralCompletenessError::ResponseWindowTooShort
        );
    }

    #[test]
    fn response_before_notice_fails_closed() {
        let mut late_notice = notice();
        late_notice.served_at_ms = 2_500;
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[late_notice],
                &[response()],
                closure(),
            )
            .unwrap_err(),
            ProceduralCompletenessError::ResponseOpenedBeforeNotice
        );
    }

    #[test]
    fn response_after_evidence_closure_fails_closed() {
        let mut early_closure = closure();
        early_closure.closed_at_ms = 3_500;
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[notice()],
                &[response()],
                early_closure,
            )
            .unwrap_err(),
            ProceduralCompletenessError::ResponseAfterEvidenceClosure
        );
    }

    #[test]
    fn duplicate_notice_recipient_fails_closed() {
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[notice(), notice()],
                &[response()],
                closure(),
            )
            .unwrap_err(),
            ProceduralCompletenessError::DuplicateNoticeRecipient
        );
    }

    #[test]
    fn changed_decision_evidence_after_closure_fails_closed() {
        let complete = complete();
        let mut changed = envelope();
        changed.decision.evidence.clear();
        assert_eq!(
            qualify_administrative_decision(complete, changed, &grant(), &[]).unwrap_err(),
            ProceduralCompletenessError::DecisionEvidenceCutMismatch
        );
    }

    #[test]
    fn evidence_observed_after_closure_fails_closed() {
        let mut future_evidence = evidence();
        future_evidence.observed_at_ms = 4_600;
        let mut closed = closure();
        closed.decision_evidence = vec![future_evidence];
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[notice()],
                &[response()],
                closed,
            )
            .unwrap_err(),
            ProceduralCompletenessError::EvidenceObservedAfterClosure
        );
    }

    #[test]
    fn missing_required_reasons_fails_closed() {
        let complete = complete();
        let mut no_reasons = envelope();
        no_reasons.decision.reasons.clear();
        assert_eq!(
            qualify_administrative_decision(complete, no_reasons, &grant(), &[]).unwrap_err(),
            ProceduralCompletenessError::RequiredReasonsMissing
        );
    }

    #[test]
    fn duplicate_closed_evidence_identity_fails_closed() {
        let mut duplicate = closure();
        duplicate.decision_evidence.push(evidence());
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                policy(),
                &[notice()],
                &[response()],
                duplicate,
            )
            .unwrap_err(),
            ProceduralCompletenessError::DuplicateEvidenceIdentity
        );
    }

    #[test]
    fn wrong_response_mode_fails_closed() {
        let mut hearing_only = policy();
        hearing_only.response_mode = ResponseModeRequirement::Hearing;
        assert_eq!(
            qualify_procedural_completeness(
                lineage(),
                hearing_only,
                &[notice()],
                &[response()],
                closure(),
            )
            .unwrap_err(),
            ProceduralCompletenessError::ResponseModeMismatch
        );
    }
}
