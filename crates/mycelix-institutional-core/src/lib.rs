// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Wire-neutral institutional primitives for Mycelix.
//!
//! This crate deliberately separates **authority** from observations, scores,
//! recommendations, and other advisory signals. A model output may be evidence
//! considered by an institution, but it cannot itself become an authority grant.
//! Authority is produced only from explicit institutional sources such as a
//! credential, consent, agreement, governance decision, delegation, or bounded
//! emergency mandate.
//!
//! The crate has no Holochain, Xenia, Symthaea, signature, storage, or transport
//! dependency. Integrators verify proofs and persist events externally.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-institutional-core-v0.1";

const MAX_ID_BYTES: usize = 512;
const MAX_CODE_BYTES: usize = 256;

macro_rules! id_type {
    ($name:ident, $label:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ValidationError> {
                let value = value.into();
                validate_text(&value, $label, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(PrincipalId, "principal_id");
id_type!(InstitutionId, "institution_id");
id_type!(JurisdictionId, "jurisdiction_id");
id_type!(RoleId, "role_id");
id_type!(CapabilityId, "capability_id");
id_type!(RulebookId, "rulebook_id");
id_type!(AuthorityGrantId, "authority_grant_id");
id_type!(IntentId, "intent_id");
id_type!(ConsentId, "consent_id");
id_type!(AgreementId, "agreement_id");
id_type!(ActionId, "action_id");
id_type!(EvidenceId, "evidence_id");
id_type!(DecisionId, "decision_id");
id_type!(ChallengeId, "challenge_id");
id_type!(AppealId, "appeal_id");
id_type!(RemedyId, "remedy_id");
id_type!(AdvisorySignalId, "advisory_signal_id");

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Digest32(pub [u8; 32]);

impl Digest32 {
    pub fn is_zero(&self) -> bool {
        self.0.iter().all(|b| *b == 0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Purpose {
    pub code: String,
    pub description: String,
}

impl Purpose {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_text(&self.code, "purpose.code", MAX_CODE_BYTES)?;
        validate_text(&self.description, "purpose.description", 4096)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RulebookRef {
    pub id: RulebookId,
    pub version: String,
    /// Digest of the exact machine-readable rulebook used for evaluation.
    pub digest: Digest32,
}

impl RulebookRef {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_text(self.id.as_str(), "rulebook.id", MAX_ID_BYTES)?;
        validate_text(&self.version, "rulebook.version", 128)?;
        require_nonzero_digest(self.digest, "rulebook.digest")
    }
}

/// Authority-bearing source categories.
///
/// There is intentionally no `ModelScore`, `ReputationScore`, `Phi`, or
/// `AdvisorySignal` variant. Those may be referenced by a decision as evidence,
/// but cannot directly grant authority through this type.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthoritySourceKind {
    Credential,
    Consent,
    Agreement,
    GovernanceDecision,
    Delegation,
    EmergencyMandate,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthoritySourceRef {
    pub kind: AuthoritySourceKind,
    pub reference: String,
    /// Cryptographic verification/proof reference checked by the host.
    pub proof_ref: String,
}

impl AuthoritySourceRef {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_text(&self.reference, "authority_source.reference", MAX_ID_BYTES)?;
        validate_text(&self.proof_ref, "authority_source.proof_ref", MAX_ID_BYTES)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityGrant {
    pub protocol_version: String,
    pub id: AuthorityGrantId,
    pub holder: PrincipalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub roles: Vec<RoleId>,
    pub capabilities: Vec<CapabilityId>,
    pub rulebook: RulebookRef,
    pub sources: Vec<AuthoritySourceRef>,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    /// Optional parent grant when authority is delegated.
    pub delegated_from: Option<AuthorityGrantId>,
    /// Attestation over this exact grant, verified externally.
    pub grant_proof_ref: String,
}

impl AuthorityGrant {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.id.as_str(), "authority_grant.id", MAX_ID_BYTES)?;
        validate_text(self.holder.as_str(), "authority_grant.holder", MAX_ID_BYTES)?;
        validate_text(
            self.institution.as_str(),
            "authority_grant.institution",
            MAX_ID_BYTES,
        )?;
        if let Some(jurisdiction) = &self.jurisdiction {
            validate_text(
                jurisdiction.as_str(),
                "authority_grant.jurisdiction",
                MAX_ID_BYTES,
            )?;
        }
        if self.capabilities.is_empty() {
            return Err(ValidationError::NoCapabilities);
        }
        if self.sources.is_empty() {
            return Err(ValidationError::NoAuthoritySources);
        }
        if self.expires_at_ms <= self.issued_at_ms {
            return Err(ValidationError::InvalidTimeRange("authority grant"));
        }
        self.rulebook.validate()?;
        for source in &self.sources {
            source.validate()?;
        }
        validate_text(
            &self.grant_proof_ref,
            "authority_grant.grant_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.issued_at_ms <= now_ms && now_ms < self.expires_at_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRef {
    pub id: EvidenceId,
    pub evidence_type: String,
    pub issuer: Option<PrincipalId>,
    pub digest: Option<Digest32>,
    pub observed_at_ms: u64,
    pub proof_ref: Option<String>,
}

impl EvidenceRef {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_text(self.id.as_str(), "evidence.id", MAX_ID_BYTES)?;
        validate_text(&self.evidence_type, "evidence.type", MAX_CODE_BYTES)?;
        if let Some(digest) = self.digest {
            require_nonzero_digest(digest, "evidence.digest")?;
        }
        if let Some(proof_ref) = &self.proof_ref {
            validate_text(proof_ref, "evidence.proof_ref", MAX_ID_BYTES)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceRequirement {
    pub evidence_type: String,
    /// Optional issuer allow-list. Empty means the rulebook accepts any issuer
    /// whose credential is independently trusted by the host.
    pub accepted_issuers: Vec<PrincipalId>,
}

impl EvidenceRequirement {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_text(&self.evidence_type, "evidence_requirement.type", MAX_CODE_BYTES)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityRequirement {
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub required_capabilities: Vec<CapabilityId>,
    /// If non-empty, the holder must possess at least one accepted role.
    pub accepted_roles: Vec<RoleId>,
    pub evidence: Vec<EvidenceRequirement>,
    pub rulebook: RulebookRef,
}

impl AuthorityRequirement {
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.required_capabilities.is_empty() {
            return Err(ValidationError::NoCapabilities);
        }
        self.rulebook.validate()?;
        for requirement in &self.evidence {
            requirement.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GrantedAuthority {
    pub grant_id: AuthorityGrantId,
    pub holder: PrincipalId,
    pub rulebook: RulebookRef,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Denial {
    pub reason_code: String,
    pub detail: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MissingEvidence {
    pub requirements: Vec<EvidenceRequirement>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityDecision {
    Allow(GrantedAuthority),
    Deny(Denial),
    NeedsEvidence(MissingEvidence),
}

/// Evaluate an explicit institutional grant against a rulebook requirement.
///
/// This function intentionally accepts no `AdvisorySignal`, model score,
/// reputation score, or consciousness metric. Such signals can inform a
/// separate decision process but cannot silently become authority here.
pub fn evaluate_authority(
    grant: &AuthorityGrant,
    requirement: &AuthorityRequirement,
    evidence: &[EvidenceRef],
    now_ms: u64,
) -> AuthorityDecision {
    if let Err(err) = grant.validate() {
        return deny("invalid_grant", err.to_string());
    }
    if let Err(err) = requirement.validate() {
        return deny("invalid_requirement", err.to_string());
    }
    if !grant.is_active_at(now_ms) {
        return deny("grant_inactive", "authority grant is not active at evaluation time");
    }
    if grant.institution != requirement.institution {
        return deny("institution_mismatch", "grant institution does not match requirement");
    }
    if grant.jurisdiction != requirement.jurisdiction {
        return deny("jurisdiction_mismatch", "grant jurisdiction does not match requirement");
    }
    if grant.rulebook != requirement.rulebook {
        return deny("rulebook_mismatch", "grant and requirement bind different rulebooks");
    }
    if requirement
        .required_capabilities
        .iter()
        .any(|required| !grant.capabilities.contains(required))
    {
        return deny("missing_capability", "grant does not contain every required capability");
    }
    if !requirement.accepted_roles.is_empty()
        && !grant
            .roles
            .iter()
            .any(|role| requirement.accepted_roles.contains(role))
    {
        return deny("role_not_accepted", "grant does not contain an accepted role");
    }

    let missing: Vec<EvidenceRequirement> = requirement
        .evidence
        .iter()
        .filter(|required| !evidence_satisfies(required, evidence))
        .cloned()
        .collect();
    if !missing.is_empty() {
        return AuthorityDecision::NeedsEvidence(MissingEvidence {
            requirements: missing,
        });
    }

    AuthorityDecision::Allow(GrantedAuthority {
        grant_id: grant.id.clone(),
        holder: grant.holder.clone(),
        rulebook: grant.rulebook.clone(),
    })
}

fn evidence_satisfies(requirement: &EvidenceRequirement, evidence: &[EvidenceRef]) -> bool {
    evidence.iter().any(|item| {
        item.evidence_type == requirement.evidence_type
            && (requirement.accepted_issuers.is_empty()
                || item
                    .issuer
                    .as_ref()
                    .is_some_and(|issuer| requirement.accepted_issuers.contains(issuer)))
            && item.validate().is_ok()
    })
}

fn deny(reason_code: &str, detail: impl Into<String>) -> AuthorityDecision {
    AuthorityDecision::Deny(Denial {
        reason_code: reason_code.to_string(),
        detail: detail.into(),
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Intent {
    pub protocol_version: String,
    pub id: IntentId,
    pub actor: PrincipalId,
    pub action_type: String,
    pub target: Option<String>,
    pub purpose: Purpose,
    /// Digest of the exact proposed action payload.
    pub payload_digest: Digest32,
    pub requested_at_ms: u64,
    pub expires_at_ms: u64,
}

impl Intent {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        validate_text(&self.action_type, "intent.action_type", MAX_CODE_BYTES)?;
        self.purpose.validate()?;
        require_nonzero_digest(self.payload_digest, "intent.payload_digest")?;
        if self.expires_at_ms <= self.requested_at_ms {
            return Err(ValidationError::InvalidTimeRange("intent"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConsentReceipt {
    pub protocol_version: String,
    pub id: ConsentId,
    pub intent_id: IntentId,
    pub parties: Vec<PrincipalId>,
    /// Digest of the exact consent scope/terms.
    pub scope_digest: Digest32,
    pub granted_at_ms: u64,
    pub expires_at_ms: Option<u64>,
    pub revocable: bool,
    pub proof_ref: String,
}

impl ConsentReceipt {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        if self.parties.is_empty() {
            return Err(ValidationError::NoConsentParties);
        }
        require_nonzero_digest(self.scope_digest, "consent.scope_digest")?;
        if let Some(expires_at_ms) = self.expires_at_ms {
            if expires_at_ms <= self.granted_at_ms {
                return Err(ValidationError::InvalidTimeRange("consent"));
            }
        }
        validate_text(&self.proof_ref, "consent.proof_ref", MAX_ID_BYTES)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AgreementRef {
    pub id: AgreementId,
    pub terms_digest: Digest32,
    pub version: String,
}

impl AgreementRef {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_nonzero_digest(self.terms_digest, "agreement.terms_digest")?;
        validate_text(&self.version, "agreement.version", 128)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionRequest {
    pub protocol_version: String,
    pub id: ActionId,
    pub intent: Intent,
    pub authority_grant_id: AuthorityGrantId,
    pub consent_receipts: Vec<ConsentId>,
    pub agreements: Vec<AgreementRef>,
    /// Repeated separately so an adapter cannot silently execute a payload that
    /// differs from the payload whose intent was evaluated.
    pub exact_payload_digest: Digest32,
    /// Freshness/replay domain supplied by the execution environment.
    pub nonce: [u8; 16],
    pub not_before_ms: u64,
    pub expires_at_ms: u64,
}

impl ActionRequest {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        self.intent.validate()?;
        if self.exact_payload_digest != self.intent.payload_digest {
            return Err(ValidationError::ActionPayloadDrift);
        }
        if self.nonce.iter().all(|b| *b == 0) {
            return Err(ValidationError::ZeroNonce);
        }
        if self.expires_at_ms <= self.not_before_ms {
            return Err(ValidationError::InvalidTimeRange("action request"));
        }
        if self.not_before_ms < self.intent.requested_at_ms
            || self.expires_at_ms > self.intent.expires_at_ms
        {
            return Err(ValidationError::ActionOutsideIntentLifetime);
        }
        for agreement in &self.agreements {
            agreement.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActionOutcome {
    Succeeded,
    Denied { reason_code: String },
    Failed { error_code: String },
    Compensated { compensation_ref: String },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionEvent {
    pub protocol_version: String,
    pub action_id: ActionId,
    pub actor: PrincipalId,
    pub institution: InstitutionId,
    pub rulebook: RulebookRef,
    pub exact_payload_digest: Digest32,
    pub outcome: ActionOutcome,
    pub executed_at_ms: u64,
    pub evidence: Vec<EvidenceRef>,
    /// Optional append-only/hash-chain predecessor.
    pub previous_event_digest: Option<Digest32>,
    pub event_proof_ref: String,
}

impl ActionEvent {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        self.rulebook.validate()?;
        require_nonzero_digest(self.exact_payload_digest, "action_event.payload_digest")?;
        for evidence in &self.evidence {
            evidence.validate()?;
        }
        validate_text(&self.event_proof_ref, "action_event.proof_ref", MAX_ID_BYTES)
    }
}

/// Non-authoritative machine/human assessment that may support deliberation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdvisorySignal {
    pub protocol_version: String,
    pub id: AdvisorySignalId,
    pub source: PrincipalId,
    pub subject: Option<PrincipalId>,
    pub signal_type: String,
    /// 0..=10_000 basis points. This is confidence/strength, never civic power.
    pub confidence_bp: u16,
    pub value_digest: Digest32,
    pub evidence: Vec<EvidenceRef>,
    pub generated_at_ms: u64,
    pub expires_at_ms: Option<u64>,
}

impl AdvisorySignal {
    pub fn validate(&self) -> Result<(), ValidationError> {
        require_protocol(&self.protocol_version)?;
        validate_text(&self.signal_type, "advisory_signal.type", MAX_CODE_BYTES)?;
        if self.confidence_bp > 10_000 {
            return Err(ValidationError::ConfidenceOutOfRange);
        }
        require_nonzero_digest(self.value_digest, "advisory_signal.value_digest")?;
        if let Some(expires_at_ms) = self.expires_at_ms {
            if expires_at_ms <= self.generated_at_ms {
                return Err(ValidationError::InvalidTimeRange("advisory signal"));
            }
        }
        for evidence in &self.evidence {
            evidence.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Decision {
    pub protocol_version: String,
    pub id: DecisionId,
    pub institution: InstitutionId,
    pub rulebook: RulebookRef,
    pub subject_ref: String,
    pub outcome_code: String,
    pub reasons: Vec<String>,
    pub evidence: Vec<EvidenceRef>,
    /// Advisory inputs are recorded for transparency but are not authority.
    pub advisory_inputs: Vec<AdvisorySignalId>,
    pub decided_at_ms: u64,
    pub decision_proof_ref: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Challenge {
    pub protocol_version: String,
    pub id: ChallengeId,
    pub decision_id: DecisionId,
    pub challenger: PrincipalId,
    pub grounds_code: String,
    pub evidence: Vec<EvidenceRef>,
    pub filed_at_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Appeal {
    pub protocol_version: String,
    pub id: AppealId,
    pub challenge_id: ChallengeId,
    pub appellant: PrincipalId,
    pub forum: InstitutionId,
    pub rulebook: RulebookRef,
    pub filed_at_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Remedy {
    pub protocol_version: String,
    pub id: RemedyId,
    pub decision_id: DecisionId,
    pub remedy_type: String,
    pub description: String,
    pub authorized_by: AuthorityGrantId,
    pub issued_at_ms: u64,
    pub expires_at_ms: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ValidationError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    ZeroDigest(&'static str),
    InvalidTimeRange(&'static str),
    NoCapabilities,
    NoAuthoritySources,
    NoConsentParties,
    ActionPayloadDrift,
    ZeroNonce,
    ActionOutsideIntentLifetime,
    ConfidenceOutOfRange,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::InvalidTimeRange(field) => write!(f, "invalid time range for {field}"),
            Self::NoCapabilities => write!(f, "at least one capability is required"),
            Self::NoAuthoritySources => write!(f, "at least one authority source is required"),
            Self::NoConsentParties => write!(f, "consent must contain at least one party"),
            Self::ActionPayloadDrift => write!(f, "action payload differs from evaluated intent"),
            Self::ZeroNonce => write!(f, "action request nonce must not be all zero"),
            Self::ActionOutsideIntentLifetime => {
                write!(f, "action request lifetime exceeds its intent lifetime")
            }
            Self::ConfidenceOutOfRange => write!(f, "confidence must be <= 10,000 basis points"),
        }
    }
}

impl std::error::Error for ValidationError {}

fn require_protocol(version: &str) -> Result<(), ValidationError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ValidationError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), ValidationError> {
    if value.trim().is_empty() {
        return Err(ValidationError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(ValidationError::TooLong(field));
    }
    Ok(())
}

fn require_nonzero_digest(
    digest: Digest32,
    field: &'static str,
) -> Result<(), ValidationError> {
    if digest.is_zero() {
        Err(ValidationError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn principal(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn institution(value: &str) -> InstitutionId {
        InstitutionId::new(value).unwrap()
    }

    fn capability(value: &str) -> CapabilityId {
        CapabilityId::new(value).unwrap()
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:community:v1").unwrap(),
            version: "1.0.0".into(),
            digest: digest(7),
        }
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant-1").unwrap(),
            holder: principal("did:example:alice"),
            institution: institution("institution:example"),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            roles: vec![RoleId::new("role:member").unwrap()],
            capabilities: vec![capability("capability:vote")],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "vc:membership:alice".into(),
                proof_ref: "proof:membership".into(),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 10_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant".into(),
        }
    }

    fn requirement() -> AuthorityRequirement {
        AuthorityRequirement {
            institution: institution("institution:example"),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            required_capabilities: vec![capability("capability:vote")],
            accepted_roles: vec![RoleId::new("role:member").unwrap()],
            evidence: vec![],
            rulebook: rulebook(),
        }
    }

    #[test]
    fn valid_grant_allows_authority() {
        let result = evaluate_authority(&grant(), &requirement(), &[], 2_000);
        assert!(matches!(result, AuthorityDecision::Allow(_)));
    }

    #[test]
    fn expired_grant_is_denied() {
        let result = evaluate_authority(&grant(), &requirement(), &[], 10_000);
        assert!(matches!(
            result,
            AuthorityDecision::Deny(Denial { ref reason_code, .. }) if reason_code == "grant_inactive"
        ));
    }

    #[test]
    fn missing_required_capability_is_denied() {
        let mut required = requirement();
        required
            .required_capabilities
            .push(capability("capability:execute"));
        let result = evaluate_authority(&grant(), &required, &[], 2_000);
        assert!(matches!(
            result,
            AuthorityDecision::Deny(Denial { ref reason_code, .. }) if reason_code == "missing_capability"
        ));
    }

    #[test]
    fn missing_evidence_is_explicit_not_silent_denial() {
        let mut required = requirement();
        required.evidence.push(EvidenceRequirement {
            evidence_type: "proof-of-membership".into(),
            accepted_issuers: vec![],
        });
        let result = evaluate_authority(&grant(), &required, &[], 2_000);
        assert!(matches!(result, AuthorityDecision::NeedsEvidence(_)));
    }

    #[test]
    fn matching_evidence_completes_authorization() {
        let mut required = requirement();
        required.evidence.push(EvidenceRequirement {
            evidence_type: "proof-of-membership".into(),
            accepted_issuers: vec![principal("did:example:issuer")],
        });
        let evidence = vec![EvidenceRef {
            id: EvidenceId::new("evidence-1").unwrap(),
            evidence_type: "proof-of-membership".into(),
            issuer: Some(principal("did:example:issuer")),
            digest: Some(digest(9)),
            observed_at_ms: 1_500,
            proof_ref: Some("proof:evidence".into()),
        }];
        assert!(matches!(
            evaluate_authority(&grant(), &required, &evidence, 2_000),
            AuthorityDecision::Allow(_)
        ));
    }

    #[test]
    fn action_request_rejects_payload_drift() {
        let intent = Intent {
            protocol_version: PROTOCOL_VERSION.into(),
            id: IntentId::new("intent-1").unwrap(),
            actor: principal("did:example:alice"),
            action_type: "treasury.transfer".into(),
            target: Some("account:bob".into()),
            purpose: Purpose {
                code: "approved-budget".into(),
                description: "Execute approved budget transfer".into(),
            },
            payload_digest: digest(1),
            requested_at_ms: 1_000,
            expires_at_ms: 5_000,
        };
        let request = ActionRequest {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ActionId::new("action-1").unwrap(),
            intent,
            authority_grant_id: AuthorityGrantId::new("grant-1").unwrap(),
            consent_receipts: vec![],
            agreements: vec![],
            exact_payload_digest: digest(2),
            nonce: [1; 16],
            not_before_ms: 1_500,
            expires_at_ms: 4_000,
        };
        assert_eq!(request.validate(), Err(ValidationError::ActionPayloadDrift));
    }

    #[test]
    fn action_request_rejects_replay_domain_zero_nonce() {
        let intent = Intent {
            protocol_version: PROTOCOL_VERSION.into(),
            id: IntentId::new("intent-2").unwrap(),
            actor: principal("did:example:alice"),
            action_type: "event.emit".into(),
            target: None,
            purpose: Purpose {
                code: "audit".into(),
                description: "Emit an audited event".into(),
            },
            payload_digest: digest(3),
            requested_at_ms: 1_000,
            expires_at_ms: 5_000,
        };
        let request = ActionRequest {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ActionId::new("action-2").unwrap(),
            intent,
            authority_grant_id: AuthorityGrantId::new("grant-1").unwrap(),
            consent_receipts: vec![],
            agreements: vec![],
            exact_payload_digest: digest(3),
            nonce: [0; 16],
            not_before_ms: 1_500,
            expires_at_ms: 4_000,
        };
        assert_eq!(request.validate(), Err(ValidationError::ZeroNonce));
    }

    #[test]
    fn advisory_signal_is_explicitly_non_authoritative() {
        let signal = AdvisorySignal {
            protocol_version: PROTOCOL_VERSION.into(),
            id: AdvisorySignalId::new("signal-1").unwrap(),
            source: principal("did:example:symthaea"),
            subject: Some(principal("did:example:alice")),
            signal_type: "reputation-assessment".into(),
            confidence_bp: 9_500,
            value_digest: digest(8),
            evidence: vec![],
            generated_at_ms: 1_000,
            expires_at_ms: Some(2_000),
        };
        assert!(signal.validate().is_ok());

        // AuthoritySourceKind has no variant capable of accepting this signal.
        // The authority evaluator's API likewise accepts only AuthorityGrant,
        // AuthorityRequirement, and EvidenceRef inputs.
        assert_eq!(signal.confidence_bp, 9_500);
    }

    #[test]
    fn wire_types_round_trip() {
        let encoded = serde_json::to_string(&grant()).unwrap();
        let decoded: AuthorityGrant = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, grant());
    }
}
