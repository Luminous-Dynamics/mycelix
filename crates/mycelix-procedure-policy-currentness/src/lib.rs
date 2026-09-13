// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-qualified closed-world currentness for ADMIN-002 procedure policy.
//!
//! This crate deliberately separates four statements:
//!
//! 1. policy bytes have one canonical semantic identity;
//! 2. a provider is institutionally authorized to attest policy currentness;
//! 3. that provider closes its authoritative namespace through one queried time;
//! 4. an administrative actor has authority to decide under the policy.
//!
//! This crate composes (1) + (2) + (3). It never produces (4).

use mycelix_administrative_procedure::ProcedureProfileId;
use mycelix_institutional_core::{
    AuthorityDecision, AuthorityGrant, AuthorityGrantId, AuthorityRequirement, CapabilityId,
    Digest32, EvidenceRef, EvidenceRequirement, InstitutionId, JurisdictionId, PrincipalId, RoleId,
    RulebookRef, evaluate_authority,
};
use mycelix_procedure_policy_identity::{
    CanonicalProceduralPolicyIdentity, QualifiedProceduralPolicyIdentity,
};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-procedure-policy-currentness-v0.1";
const MAX_REF_BYTES: usize = 2048;
const MAX_NAMESPACE_BYTES: usize = 256;
const MAX_PROFILE_BYTES: usize = 192;
const MAX_ROLES: usize = 64;
const MAX_AUTHORITY_EVIDENCE_REQUIREMENTS: usize = 64;
const MAX_AUTHORITY_EVIDENCE: usize = 128;

/// Frozen qualification policy for one procedure-policy currentness provider.
///
/// The target administrative institution/rulebook is deliberately separate
/// from the institution/rulebook that authorizes the provider. A shared public
/// registry may therefore be designated by another institution without being
/// mistaken for the administrative decision-maker.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProcedurePolicyCurrentnessPolicy {
    pub protocol_version: String,

    pub target_institution: InstitutionId,
    pub target_jurisdiction: Option<JurisdictionId>,
    pub target_rulebook: RulebookRef,
    pub procedure_profile: ProcedureProfileId,

    pub provider_namespace: String,
    pub provider_authority_institution: InstitutionId,
    pub provider_authority_jurisdiction: Option<JurisdictionId>,
    pub provider_authority_rulebook: RulebookRef,
    pub required_provider_capability: CapabilityId,
    pub accepted_provider_roles: Vec<RoleId>,
    pub provider_authority_evidence: Vec<EvidenceRequirement>,
}

impl ProcedurePolicyCurrentnessPolicy {
    pub fn validate(&self) -> Result<(), ProcedurePolicyCurrentnessError> {
        require_protocol(&self.protocol_version)?;
        validate_namespace(&self.provider_namespace)?;
        self.target_rulebook
            .validate()
            .map_err(|error| ProcedurePolicyCurrentnessError::InvalidPolicy(error.to_string()))?;
        self.provider_authority_rulebook
            .validate()
            .map_err(|error| ProcedurePolicyCurrentnessError::InvalidPolicy(error.to_string()))?;

        if self.accepted_provider_roles.len() > MAX_ROLES {
            return Err(ProcedurePolicyCurrentnessError::TooManyProviderRoles);
        }
        if self.provider_authority_evidence.len() > MAX_AUTHORITY_EVIDENCE_REQUIREMENTS {
            return Err(ProcedurePolicyCurrentnessError::TooManyAuthorityEvidenceRequirements);
        }

        let mut roles = BTreeSet::new();
        for role in &self.accepted_provider_roles {
            if !roles.insert(role.as_str()) {
                return Err(ProcedurePolicyCurrentnessError::DuplicateProviderRole);
            }
        }

        let requirement = self.authority_requirement();
        requirement
            .validate()
            .map_err(|error| ProcedurePolicyCurrentnessError::InvalidPolicy(error.to_string()))
    }

    fn authority_requirement(&self) -> AuthorityRequirement {
        AuthorityRequirement {
            institution: self.provider_authority_institution.clone(),
            jurisdiction: self.provider_authority_jurisdiction.clone(),
            required_capabilities: vec![self.required_provider_capability.clone()],
            accepted_roles: self.accepted_provider_roles.clone(),
            evidence: self.provider_authority_evidence.clone(),
            rulebook: self.provider_authority_rulebook.clone(),
        }
    }
}

/// Candidate closed-world assertion emitted by an authoritative policy source.
///
/// `closed_through_ms` is the key closed-world boundary: the provider claims its
/// exact namespace/generation view includes every authoritative publication or
/// supersession relevant through that time. Local absence is never an input to
/// this theorem.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProcedurePolicyCurrentnessClaim {
    pub protocol_version: String,

    pub target_institution: InstitutionId,
    pub target_jurisdiction: Option<JurisdictionId>,
    pub target_rulebook: RulebookRef,
    pub procedure_profile: ProcedureProfileId,

    pub policy_digest: Digest32,
    pub policy_digest_profile: String,
    pub publication_ref: String,

    pub provider_namespace: String,
    pub provider: PrincipalId,
    pub provider_grant_id: AuthorityGrantId,
    pub provider_generation: u64,
    pub provider_state_digest: Digest32,

    pub effective_from_ms: u64,
    pub effective_until_ms: Option<u64>,
    pub closed_through_ms: u64,
    pub issued_at_ms: u64,

    /// Host-verified proof over this exact provider claim/currentness state.
    /// The pure kernel validates the reference and exact semantic bindings but
    /// intentionally does not implement signature or transport verification.
    pub currentness_proof_ref: String,
}

impl ProcedurePolicyCurrentnessClaim {
    pub fn validate(&self) -> Result<(), ProcedurePolicyCurrentnessError> {
        require_protocol(&self.protocol_version)?;
        validate_namespace(&self.provider_namespace)?;
        validate_ref(&self.publication_ref)?;
        validate_ref(&self.currentness_proof_ref)?;
        validate_profile(&self.policy_digest_profile)?;
        self.target_rulebook
            .validate()
            .map_err(|error| ProcedurePolicyCurrentnessError::InvalidClaim(error.to_string()))?;

        if self.policy_digest.is_zero() {
            return Err(ProcedurePolicyCurrentnessError::ZeroPolicyDigest);
        }
        if self.provider_state_digest.is_zero() {
            return Err(ProcedurePolicyCurrentnessError::ZeroProviderStateDigest);
        }
        if self.provider_generation == 0 {
            return Err(ProcedurePolicyCurrentnessError::ZeroProviderGeneration);
        }
        if self.effective_from_ms == 0 || self.issued_at_ms == 0 || self.closed_through_ms == 0 {
            return Err(ProcedurePolicyCurrentnessError::InvalidCurrentnessTime);
        }
        if let Some(until_ms) = self.effective_until_ms {
            if until_ms <= self.effective_from_ms {
                return Err(ProcedurePolicyCurrentnessError::InvalidEffectiveInterval);
            }
        }
        if self.closed_through_ms > self.issued_at_ms {
            return Err(ProcedurePolicyCurrentnessError::ClosureAfterClaimIssuance);
        }
        Ok(())
    }
}

/// Opaque proof that one canonical policy identity is current under one exact
/// authority-qualified provider claim at one exact `as_of_ms`.
///
/// Deliberately not Clone/Serialize/Deserialize. Persisted currentness must be
/// requalified from the identity, provider authority, and exact closed-world
/// claim rather than by deserializing a positive token.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedCurrentProcedurePolicy {
    identity: QualifiedProceduralPolicyIdentity,
    policy: ProcedurePolicyCurrentnessPolicy,
    claim: ProcedurePolicyCurrentnessClaim,
    as_of_ms: u64,
}

impl QualifiedCurrentProcedurePolicy {
    pub fn identity(&self) -> &CanonicalProceduralPolicyIdentity {
        self.identity.identity()
    }

    pub fn policy_ref(&self) -> &str {
        self.identity.policy_ref()
    }

    pub fn procedure_profile(&self) -> &ProcedureProfileId {
        self.identity.procedure_profile()
    }

    pub fn currentness_policy(&self) -> &ProcedurePolicyCurrentnessPolicy {
        &self.policy
    }

    pub fn claim(&self) -> &ProcedurePolicyCurrentnessClaim {
        &self.claim
    }

    pub fn current_as_of_ms(&self) -> u64 {
        self.as_of_ms
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_administrative_decision_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_current_procedure_policy(
    identity: QualifiedProceduralPolicyIdentity,
    policy: ProcedurePolicyCurrentnessPolicy,
    claim: ProcedurePolicyCurrentnessClaim,
    provider_grant: &AuthorityGrant,
    provider_authority_evidence: &[EvidenceRef],
    as_of_ms: u64,
) -> Result<QualifiedCurrentProcedurePolicy, ProcedurePolicyCurrentnessError> {
    policy.validate()?;
    claim.validate()?;

    if as_of_ms == 0 {
        return Err(ProcedurePolicyCurrentnessError::InvalidAsOfTime);
    }
    if provider_authority_evidence.len() > MAX_AUTHORITY_EVIDENCE {
        return Err(ProcedurePolicyCurrentnessError::TooMuchAuthorityEvidence);
    }
    for evidence in provider_authority_evidence {
        evidence
            .validate()
            .map_err(|error| ProcedurePolicyCurrentnessError::InvalidAuthorityEvidence(error.to_string()))?;
        if evidence.observed_at_ms > claim.issued_at_ms {
            return Err(ProcedurePolicyCurrentnessError::AuthorityEvidenceFromFuture);
        }
    }

    if claim.target_institution != policy.target_institution {
        return Err(ProcedurePolicyCurrentnessError::TargetInstitutionMismatch);
    }
    if claim.target_jurisdiction != policy.target_jurisdiction {
        return Err(ProcedurePolicyCurrentnessError::TargetJurisdictionMismatch);
    }
    if claim.target_rulebook != policy.target_rulebook {
        return Err(ProcedurePolicyCurrentnessError::TargetRulebookMismatch);
    }
    if claim.procedure_profile != policy.procedure_profile
        || identity.procedure_profile() != &policy.procedure_profile
    {
        return Err(ProcedurePolicyCurrentnessError::ProcedureProfileMismatch);
    }

    if claim.policy_digest != identity.identity().digest {
        return Err(ProcedurePolicyCurrentnessError::PolicyDigestMismatch);
    }
    if claim.policy_digest_profile != identity.identity().profile {
        return Err(ProcedurePolicyCurrentnessError::PolicyDigestProfileMismatch);
    }

    if claim.provider_namespace != policy.provider_namespace {
        return Err(ProcedurePolicyCurrentnessError::ProviderNamespaceMismatch);
    }
    if claim.provider_grant_id != provider_grant.id {
        return Err(ProcedurePolicyCurrentnessError::ProviderGrantMismatch);
    }
    if claim.provider != provider_grant.holder {
        return Err(ProcedurePolicyCurrentnessError::ProviderPrincipalMismatch);
    }

    if as_of_ms < claim.effective_from_ms
        || claim
            .effective_until_ms
            .is_some_and(|until_ms| as_of_ms >= until_ms)
    {
        return Err(ProcedurePolicyCurrentnessError::PolicyNotEffectiveAtAsOf);
    }
    if as_of_ms > claim.closed_through_ms {
        return Err(ProcedurePolicyCurrentnessError::CurrentnessDoesNotCoverAsOf);
    }

    let authority_requirement = policy.authority_requirement();
    match evaluate_authority(
        provider_grant,
        &authority_requirement,
        provider_authority_evidence,
        claim.issued_at_ms,
    ) {
        AuthorityDecision::Allow(_) => {}
        AuthorityDecision::Deny(denial) => {
            return Err(ProcedurePolicyCurrentnessError::ProviderAuthorityDenied {
                reason_code: denial.reason_code,
                detail: denial.detail,
            });
        }
        AuthorityDecision::NeedsEvidence(missing) => {
            return Err(ProcedurePolicyCurrentnessError::ProviderAuthorityNeedsEvidence(
                missing.requirements.len(),
            ));
        }
    }

    Ok(QualifiedCurrentProcedurePolicy {
        identity,
        policy,
        claim,
        as_of_ms,
    })
}

fn require_protocol(value: &str) -> Result<(), ProcedurePolicyCurrentnessError> {
    if value == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ProcedurePolicyCurrentnessError::WrongProtocolVersion)
    }
}

fn validate_ref(value: &str) -> Result<(), ProcedurePolicyCurrentnessError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ProcedurePolicyCurrentnessError::InvalidReference)
    } else {
        Ok(())
    }
}

fn validate_namespace(value: &str) -> Result<(), ProcedurePolicyCurrentnessError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_NAMESPACE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ProcedurePolicyCurrentnessError::InvalidProviderNamespace)
    } else {
        Ok(())
    }
}

fn validate_profile(value: &str) -> Result<(), ProcedurePolicyCurrentnessError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ProcedurePolicyCurrentnessError::InvalidPolicyDigestProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProcedurePolicyCurrentnessError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidProviderNamespace,
    InvalidPolicyDigestProfile,
    InvalidPolicy(String),
    InvalidClaim(String),
    ZeroPolicyDigest,
    ZeroProviderStateDigest,
    ZeroProviderGeneration,
    InvalidCurrentnessTime,
    InvalidEffectiveInterval,
    ClosureAfterClaimIssuance,
    InvalidAsOfTime,
    TooManyProviderRoles,
    DuplicateProviderRole,
    TooManyAuthorityEvidenceRequirements,
    TooMuchAuthorityEvidence,
    InvalidAuthorityEvidence(String),
    AuthorityEvidenceFromFuture,
    TargetInstitutionMismatch,
    TargetJurisdictionMismatch,
    TargetRulebookMismatch,
    ProcedureProfileMismatch,
    PolicyDigestMismatch,
    PolicyDigestProfileMismatch,
    ProviderNamespaceMismatch,
    ProviderGrantMismatch,
    ProviderPrincipalMismatch,
    PolicyNotEffectiveAtAsOf,
    CurrentnessDoesNotCoverAsOf,
    ProviderAuthorityDenied { reason_code: String, detail: String },
    ProviderAuthorityNeedsEvidence(usize),
}

impl fmt::Display for ProcedurePolicyCurrentnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong procedure-policy currentness protocol version"),
            Self::InvalidReference => write!(f, "invalid procedure-policy currentness reference"),
            Self::InvalidProviderNamespace => write!(f, "invalid policy-provider namespace"),
            Self::InvalidPolicyDigestProfile => write!(f, "invalid policy digest profile"),
            Self::InvalidPolicy(detail) => write!(f, "invalid currentness policy: {detail}"),
            Self::InvalidClaim(detail) => write!(f, "invalid currentness claim: {detail}"),
            Self::ZeroPolicyDigest => write!(f, "currentness claim policy digest must be non-zero"),
            Self::ZeroProviderStateDigest => write!(f, "provider state digest must be non-zero"),
            Self::ZeroProviderGeneration => write!(f, "provider generation must be non-zero"),
            Self::InvalidCurrentnessTime => write!(f, "invalid currentness claim time"),
            Self::InvalidEffectiveInterval => write!(f, "invalid policy effective interval"),
            Self::ClosureAfterClaimIssuance => write!(f, "provider closure extends beyond claim issuance"),
            Self::InvalidAsOfTime => write!(f, "invalid currentness as-of time"),
            Self::TooManyProviderRoles => write!(f, "provider role count exceeds v0.1 bound"),
            Self::DuplicateProviderRole => write!(f, "duplicate accepted provider role"),
            Self::TooManyAuthorityEvidenceRequirements => {
                write!(f, "provider authority evidence requirements exceed v0.1 bound")
            }
            Self::TooMuchAuthorityEvidence => write!(f, "provider authority evidence exceeds v0.1 bound"),
            Self::InvalidAuthorityEvidence(detail) => write!(f, "invalid provider authority evidence: {detail}"),
            Self::AuthorityEvidenceFromFuture => write!(f, "provider authority evidence postdates currentness claim"),
            Self::TargetInstitutionMismatch => write!(f, "currentness target institution mismatch"),
            Self::TargetJurisdictionMismatch => write!(f, "currentness target jurisdiction mismatch"),
            Self::TargetRulebookMismatch => write!(f, "currentness target rulebook mismatch"),
            Self::ProcedureProfileMismatch => write!(f, "currentness procedure profile mismatch"),
            Self::PolicyDigestMismatch => write!(f, "currentness policy digest differs from qualified semantic identity"),
            Self::PolicyDigestProfileMismatch => {
                write!(f, "currentness policy digest profile differs from qualified semantic identity")
            }
            Self::ProviderNamespaceMismatch => write!(f, "currentness provider namespace mismatch"),
            Self::ProviderGrantMismatch => write!(f, "currentness provider grant identity mismatch"),
            Self::ProviderPrincipalMismatch => write!(f, "currentness provider principal mismatch"),
            Self::PolicyNotEffectiveAtAsOf => write!(f, "policy is not effective at requested currentness time"),
            Self::CurrentnessDoesNotCoverAsOf => write!(f, "provider closed-world currentness does not cover requested time"),
            Self::ProviderAuthorityDenied { reason_code, detail } => {
                write!(f, "policy-provider authority denied ({reason_code}): {detail}")
            }
            Self::ProviderAuthorityNeedsEvidence(count) => {
                write!(f, "policy-provider authority requires {count} missing evidence item(s)")
            }
        }
    }
}

impl std::error::Error for ProcedurePolicyCurrentnessError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_administrative_procedure::{
        ADMIN_002_PROTOCOL_VERSION, ProceduralCompletenessPolicy, ReasonsRequirement,
        ResponseModeRequirement,
    };
    use mycelix_institutional_core::{
        AuthoritySourceKind, AuthoritySourceRef, PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL_VERSION,
        RulebookId,
    };
    use mycelix_procedure_policy_identity::{
        PROCEDURAL_POLICY_IDENTITY_PROFILE, procedural_policy_semantic_identity,
        qualify_procedural_policy_identity,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn principal(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn target_institution() -> InstitutionId {
        InstitutionId::new("institution:city").unwrap()
    }

    fn provider_institution() -> InstitutionId {
        InstitutionId::new("institution:records-office").unwrap()
    }

    fn jurisdiction() -> JurisdictionId {
        JurisdictionId::new("jurisdiction:city").unwrap()
    }

    fn target_rulebook() -> RulebookRef {
        RulebookRef {
            id: mycelix_institutional_core::RulebookId::new("rulebook:administration:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(4),
        }
    }

    fn provider_rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:records-office:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(5),
        }
    }

    fn profile() -> ProcedureProfileId {
        ProcedureProfileId::new("procedure:permit:v1").unwrap()
    }

    fn canonical_policy() -> ProceduralCompletenessPolicy {
        let mut policy = ProceduralCompletenessPolicy {
            protocol_version: ADMIN_002_PROTOCOL_VERSION.into(),
            procedure_profile: profile(),
            policy_ref: "registry:procedure-policy:permit:v1".into(),
            policy_digest: d(1),
            policy_digest_profile: PROCEDURAL_POLICY_IDENTITY_PROFILE.into(),
            required_notice_recipients: vec![principal("did:example:applicant")],
            required_response_recipients: vec![principal("did:example:applicant")],
            response_mode: ResponseModeRequirement::WrittenOrHearing,
            min_response_window_ms: 1_000,
            reasons: ReasonsRequirement::AtLeastOne,
        };
        policy.policy_digest = procedural_policy_semantic_identity(&policy).unwrap().digest;
        policy
    }

    fn identity() -> QualifiedProceduralPolicyIdentity {
        qualify_procedural_policy_identity(&canonical_policy()).unwrap()
    }

    fn currentness_policy() -> ProcedurePolicyCurrentnessPolicy {
        ProcedurePolicyCurrentnessPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            target_institution: target_institution(),
            target_jurisdiction: Some(jurisdiction()),
            target_rulebook: target_rulebook(),
            procedure_profile: profile(),
            provider_namespace: "registry:procedure-policy:city".into(),
            provider_authority_institution: provider_institution(),
            provider_authority_jurisdiction: Some(jurisdiction()),
            provider_authority_rulebook: provider_rulebook(),
            required_provider_capability: CapabilityId::new("administration.policy.currentness.attest").unwrap(),
            accepted_provider_roles: vec![RoleId::new("role:policy-registrar").unwrap()],
            provider_authority_evidence: vec![],
        }
    }

    fn provider_grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:policy-registrar:1").unwrap(),
            holder: principal("did:example:policy-registrar"),
            institution: provider_institution(),
            jurisdiction: Some(jurisdiction()),
            roles: vec![RoleId::new("role:policy-registrar").unwrap()],
            capabilities: vec![CapabilityId::new("administration.policy.currentness.attest").unwrap()],
            rulebook: provider_rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "governance:provider-designation:1".into(),
                proof_ref: "proof:provider-designation:1".into(),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 20_000,
            delegated_from: None,
            grant_proof_ref: "proof:provider-grant:1".into(),
        }
    }

    fn claim() -> ProcedurePolicyCurrentnessClaim {
        let qualified = identity();
        ProcedurePolicyCurrentnessClaim {
            protocol_version: PROTOCOL_VERSION.into(),
            target_institution: target_institution(),
            target_jurisdiction: Some(jurisdiction()),
            target_rulebook: target_rulebook(),
            procedure_profile: profile(),
            policy_digest: qualified.identity().digest,
            policy_digest_profile: qualified.identity().profile.clone(),
            publication_ref: "registry:procedure-policy:permit:v1:g7".into(),
            provider_namespace: "registry:procedure-policy:city".into(),
            provider: principal("did:example:policy-registrar"),
            provider_grant_id: AuthorityGrantId::new("grant:policy-registrar:1").unwrap(),
            provider_generation: 7,
            provider_state_digest: d(8),
            effective_from_ms: 2_000,
            effective_until_ms: Some(12_000),
            closed_through_ms: 8_000,
            issued_at_ms: 8_500,
            currentness_proof_ref: "proof:policy-currentness:g7".into(),
        }
    }

    fn qualify_at(as_of_ms: u64) -> Result<QualifiedCurrentProcedurePolicy, ProcedurePolicyCurrentnessError> {
        qualify_current_procedure_policy(
            identity(),
            currentness_policy(),
            claim(),
            &provider_grant(),
            &[],
            as_of_ms,
        )
    }

    #[test]
    fn exact_authorized_closed_world_claim_qualifies_current_policy() {
        let qualified = qualify_at(8_000).unwrap();
        assert_eq!(qualified.current_as_of_ms(), 8_000);
        assert_eq!(qualified.identity().digest, identity().identity().digest);
        assert_eq!(qualified.claim().provider_generation, 7);
        assert!(!qualified.grants_authority());
        assert!(!qualified.grants_administrative_decision_authority());
        assert!(!qualified.grants_external_effect_authority());
    }

    #[test]
    fn local_or_stale_view_cannot_cover_later_as_of_time() {
        assert_eq!(
            qualify_at(8_001).unwrap_err(),
            ProcedurePolicyCurrentnessError::CurrentnessDoesNotCoverAsOf
        );
    }

    #[test]
    fn altered_policy_identity_fails_closed() {
        let mut current_claim = claim();
        current_claim.policy_digest = d(99);
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                current_claim,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::PolicyDigestMismatch
        );
    }

    #[test]
    fn provider_namespace_substitution_fails_closed() {
        let mut current_claim = claim();
        current_claim.provider_namespace = "registry:other".into();
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                current_claim,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::ProviderNamespaceMismatch
        );
    }

    #[test]
    fn provider_principal_or_grant_substitution_fails_closed() {
        let mut wrong_provider = claim();
        wrong_provider.provider = principal("did:example:other");
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                wrong_provider,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::ProviderPrincipalMismatch
        );

        let mut wrong_grant = claim();
        wrong_grant.provider_grant_id = AuthorityGrantId::new("grant:other").unwrap();
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                wrong_grant,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::ProviderGrantMismatch
        );
    }

    #[test]
    fn missing_provider_capability_fails_closed() {
        let mut grant = provider_grant();
        grant.capabilities = vec![CapabilityId::new("administration.other").unwrap()];
        assert!(matches!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                claim(),
                &grant,
                &[],
                8_000,
            ),
            Err(ProcedurePolicyCurrentnessError::ProviderAuthorityDenied { .. })
        ));
    }

    #[test]
    fn inactive_provider_grant_fails_closed() {
        let mut grant = provider_grant();
        grant.expires_at_ms = 8_400;
        assert!(matches!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                claim(),
                &grant,
                &[],
                8_000,
            ),
            Err(ProcedurePolicyCurrentnessError::ProviderAuthorityDenied { .. })
        ));
    }

    #[test]
    fn target_scope_substitution_fails_closed() {
        let mut current_claim = claim();
        current_claim.target_institution = InstitutionId::new("institution:other").unwrap();
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                current_claim,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::TargetInstitutionMismatch
        );
    }

    #[test]
    fn closed_through_cannot_be_after_claim_issuance() {
        let mut current_claim = claim();
        current_claim.closed_through_ms = current_claim.issued_at_ms + 1;
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                current_claim,
                &provider_grant(),
                &[],
                8_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::ClosureAfterClaimIssuance
        );
    }

    #[test]
    fn effective_interval_is_half_open() {
        assert_eq!(
            qualify_at(1_999).unwrap_err(),
            ProcedurePolicyCurrentnessError::PolicyNotEffectiveAtAsOf
        );

        let mut current_claim = claim();
        current_claim.closed_through_ms = 12_000;
        current_claim.issued_at_ms = 12_500;
        let mut grant = provider_grant();
        grant.expires_at_ms = 20_000;
        assert_eq!(
            qualify_current_procedure_policy(
                identity(),
                currentness_policy(),
                current_claim,
                &grant,
                &[],
                12_000,
            )
            .unwrap_err(),
            ProcedurePolicyCurrentnessError::PolicyNotEffectiveAtAsOf
        );
    }

    #[test]
    fn provider_and_target_authority_rulebooks_may_be_distinct() {
        let qualified = qualify_at(8_000).unwrap();
        assert_ne!(
            qualified.currentness_policy().target_rulebook,
            qualified.currentness_policy().provider_authority_rulebook
        );
    }
}
