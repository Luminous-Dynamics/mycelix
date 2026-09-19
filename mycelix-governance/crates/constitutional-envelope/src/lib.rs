// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical constitutional authorization envelopes for Mycelix.
//!
//! This crate deliberately performs no host calls and chooses no signature or
//! hash algorithm. It defines the exact semantic bytes that a runtime may hash,
//! sign, verify, replay-protect, and persist as authorization evidence.

use constitutional_authority::{
    principal_can_exercise, AuthorityPrincipal, Branch, CapabilitySource,
    ConstitutionalEntitlement, ConstitutionalPower, Guardian,
};
use serde::{Deserialize, Serialize};

pub const ENVELOPE_SCHEMA_VERSION: u16 = 1;
pub const ENVELOPE_DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-CONSTITUTIONAL-AUTH\0V1\0";

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, Hash)]
pub struct DigestRef {
    pub algorithm: String,
    pub value: String,
}

impl DigestRef {
    pub fn validate(&self) -> Result<(), EnvelopeError> {
        if self.algorithm.trim().is_empty() {
            return Err(EnvelopeError::EmptyDigestAlgorithm);
        }
        if self.value.trim().is_empty() {
            return Err(EnvelopeError::EmptyDigestValue);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, Hash)]
pub struct MatterId {
    /// Stable namespace such as `election`, `procurement`, `case`, or
    /// `protocol-upgrade`. Runtime provenance decides whether two actions truly
    /// share a matter; callers must not be trusted to self-assert that fact.
    pub namespace: String,
    pub stable_id: String,
}

impl MatterId {
    pub fn validate(&self) -> Result<(), EnvelopeError> {
        if self.namespace.trim().is_empty() {
            return Err(EnvelopeError::EmptyMatterNamespace);
        }
        if !self
            .namespace
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || matches!(b, b'.' | b'_' | b'-'))
        {
            return Err(EnvelopeError::InvalidMatterNamespace);
        }
        if self.stable_id.trim().is_empty() {
            return Err(EnvelopeError::EmptyMatterId);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq, Hash)]
pub struct ResourceBinding {
    pub kind: String,
    pub id: String,
    /// Digest of the exact payload/parameters being authorized. This is opaque
    /// to the semantic crate; runtime policy chooses approved digest algorithms.
    pub payload_digest: DigestRef,
}

impl ResourceBinding {
    pub fn validate(&self) -> Result<(), EnvelopeError> {
        if self.kind.trim().is_empty() {
            return Err(EnvelopeError::EmptyResourceKind);
        }
        if self.id.trim().is_empty() {
            return Err(EnvelopeError::EmptyResourceId);
        }
        self.payload_digest.validate()
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorizationAction {
    Sovereign(ConstitutionalPower),
    Entitlement(ConstitutionalEntitlement),
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuthorityDomain {
    Constituent,
    Branch(Branch),
    Guardian(Guardian),
}

impl AuthorityDomain {
    pub fn from_principal(principal: AuthorityPrincipal) -> Option<Self> {
        match principal {
            AuthorityPrincipal::ConstituentSovereignty => Some(Self::Constituent),
            AuthorityPrincipal::Branch(branch) => Some(Self::Branch(branch)),
            AuthorityPrincipal::Guardian(guardian) => Some(Self::Guardian(guardian)),
            AuthorityPrincipal::AutomatedAgent => None,
        }
    }

    fn rank(self) -> u16 {
        match self {
            Self::Constituent => 0,
            Self::Branch(branch) => 100 + branch_tag(branch) as u16,
            Self::Guardian(guardian) => 200 + guardian_tag(guardian) as u16,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum UsePolicy {
    OneShot,
    Bounded { max_uses: u32 },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConcurrenceRequirement {
    pub min_approvals: u16,
    pub min_distinct_domains: u16,
    pub required_domains: Vec<AuthorityDomain>,
    pub require_unique_holders: bool,
    pub require_unique_actors: bool,
    pub excluded_holder_ids: Vec<String>,
    pub excluded_actor_ids: Vec<String>,
}

impl ConcurrenceRequirement {
    pub fn validate(&self) -> Result<(), EnvelopeError> {
        if self.min_approvals == 0 {
            return Err(EnvelopeError::InvalidConcurrenceRequirement);
        }
        if self.min_distinct_domains == 0 || self.min_distinct_domains > self.min_approvals {
            return Err(EnvelopeError::InvalidConcurrenceRequirement);
        }
        let mut domains = Vec::new();
        for domain in &self.required_domains {
            if domains.contains(domain) {
                return Err(EnvelopeError::DuplicateRequiredDomain);
            }
            domains.push(*domain);
        }
        if self.required_domains.len() > self.min_distinct_domains as usize {
            return Err(EnvelopeError::InvalidConcurrenceRequirement);
        }
        if self.excluded_holder_ids.iter().any(|id| id.trim().is_empty())
            || self.excluded_actor_ids.iter().any(|id| id.trim().is_empty())
        {
            return Err(EnvelopeError::EmptyConcurrenceExclusion);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationEnvelope {
    pub schema_version: u16,
    /// Constitutional class that owns the authority.
    pub holder_class: AuthorityPrincipal,
    /// Exact office/institution/constituent process that legally holds it.
    pub holder_id: String,
    /// Exact runtime actor invoking the holder's authority. This actor does not
    /// become sovereign merely by appearing here; authority remains with holder.
    pub actor_id: String,
    pub action: AuthorizationAction,
    pub jurisdiction: String,
    pub source: CapabilitySource,
    /// Optional commitment to exact source bytes/version beyond the source ID.
    pub source_digest: Option<DigestRef>,
    pub matter: MatterId,
    pub purpose: String,
    pub resource: ResourceBinding,
    pub issued_at_us: i64,
    pub valid_from_us: i64,
    pub expires_at_us: Option<i64>,
    pub nonce: String,
    pub use_policy: UsePolicy,
    /// Required for delegated sources; commits to the authenticated chain/root.
    pub delegation_chain_digest: Option<DigestRef>,
    pub concurrence: Option<ConcurrenceRequirement>,
    pub review_path: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum EnvelopeError {
    UnsupportedSchemaVersion,
    AutomatedAgentCannotHoldConstitutionalAuthority,
    UnauthorizedSovereignAction,
    EmptyHolderId,
    EmptyActorId,
    EmptyJurisdiction,
    InvalidSource,
    EmptyMatterNamespace,
    InvalidMatterNamespace,
    EmptyMatterId,
    EmptyPurpose,
    EmptyResourceKind,
    EmptyResourceId,
    EmptyDigestAlgorithm,
    EmptyDigestValue,
    IssueTimeAfterActivation,
    InvalidExpiry,
    MissingRequiredExpiry,
    EmptyNonce,
    InvalidUsePolicy,
    EmptyReviewPath,
    MissingDelegationCommitment,
    UnexpectedDelegationCommitment,
    IntrinsicPowerCannotUseDelegatedSource,
    InvalidConcurrenceRequirement,
    DuplicateRequiredDomain,
    EmptyConcurrenceExclusion,
    CanonicalFieldTooLarge,
}

impl AuthorizationEnvelope {
    pub fn validate(&self) -> Result<(), EnvelopeError> {
        if self.schema_version != ENVELOPE_SCHEMA_VERSION {
            return Err(EnvelopeError::UnsupportedSchemaVersion);
        }
        if self.holder_id.trim().is_empty() {
            return Err(EnvelopeError::EmptyHolderId);
        }
        if self.actor_id.trim().is_empty() {
            return Err(EnvelopeError::EmptyActorId);
        }
        if self.jurisdiction.trim().is_empty() {
            return Err(EnvelopeError::EmptyJurisdiction);
        }
        if matches!(self.holder_class, AuthorityPrincipal::AutomatedAgent) {
            return Err(EnvelopeError::AutomatedAgentCannotHoldConstitutionalAuthority);
        }
        match self.action {
            AuthorizationAction::Sovereign(power) => {
                if !principal_can_exercise(self.holder_class, power) {
                    return Err(EnvelopeError::UnauthorizedSovereignAction);
                }
                if power.requires_hard_expiry() && self.expires_at_us.is_none() {
                    return Err(EnvelopeError::MissingRequiredExpiry);
                }
                if power.is_intrinsically_nondelegable() && self.source.is_delegation() {
                    return Err(EnvelopeError::IntrinsicPowerCannotUseDelegatedSource);
                }
            }
            AuthorizationAction::Entitlement(_) => {}
        }
        if !self.source.is_well_formed() {
            return Err(EnvelopeError::InvalidSource);
        }
        if let Some(digest) = &self.source_digest {
            digest.validate()?;
        }
        self.matter.validate()?;
        if self.purpose.trim().is_empty() {
            return Err(EnvelopeError::EmptyPurpose);
        }
        self.resource.validate()?;
        if self.issued_at_us > self.valid_from_us {
            return Err(EnvelopeError::IssueTimeAfterActivation);
        }
        if let Some(expires_at) = self.expires_at_us {
            if expires_at <= self.valid_from_us {
                return Err(EnvelopeError::InvalidExpiry);
            }
        }
        if self.nonce.trim().is_empty() {
            return Err(EnvelopeError::EmptyNonce);
        }
        if matches!(self.use_policy, UsePolicy::Bounded { max_uses: 0 }) {
            return Err(EnvelopeError::InvalidUsePolicy);
        }
        if self.review_path.trim().is_empty() {
            return Err(EnvelopeError::EmptyReviewPath);
        }
        match (&self.source, &self.delegation_chain_digest) {
            (CapabilitySource::Delegation { .. }, Some(digest)) => digest.validate()?,
            (CapabilitySource::Delegation { .. }, None) => {
                return Err(EnvelopeError::MissingDelegationCommitment)
            }
            (_, Some(_)) => return Err(EnvelopeError::UnexpectedDelegationCommitment),
            (_, None) => {}
        }
        if let Some(requirement) = &self.concurrence {
            requirement.validate()?;
        }
        Ok(())
    }

    /// Deterministic, domain-separated bytes suitable for hashing/signing.
    /// This is intentionally not JSON serialization.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, EnvelopeError> {
        self.validate()?;
        let mut out = Vec::with_capacity(512);
        out.extend_from_slice(ENVELOPE_DOMAIN_SEPARATOR);
        push_u16(&mut out, self.schema_version);
        encode_principal(&mut out, self.holder_class);
        push_str(&mut out, &self.holder_id)?;
        push_str(&mut out, &self.actor_id)?;
        encode_action(&mut out, self.action);
        push_str(&mut out, &self.jurisdiction)?;
        encode_source(&mut out, &self.source)?;
        encode_optional_digest(&mut out, self.source_digest.as_ref())?;
        encode_matter(&mut out, &self.matter)?;
        push_str(&mut out, &self.purpose)?;
        encode_resource(&mut out, &self.resource)?;
        push_i64(&mut out, self.issued_at_us);
        push_i64(&mut out, self.valid_from_us);
        encode_optional_i64(&mut out, self.expires_at_us);
        push_str(&mut out, &self.nonce)?;
        encode_use_policy(&mut out, &self.use_policy);
        encode_optional_digest(&mut out, self.delegation_chain_digest.as_ref())?;
        encode_optional_concurrence(&mut out, self.concurrence.as_ref())?;
        push_str(&mut out, &self.review_path)?;
        Ok(out)
    }

    pub fn domain(&self) -> Result<AuthorityDomain, EnvelopeError> {
        AuthorityDomain::from_principal(self.holder_class)
            .ok_or(EnvelopeError::AutomatedAgentCannotHoldConstitutionalAuthority)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ConcurrenceApproval {
    pub holder_id: String,
    pub actor_id: String,
    pub domain: AuthorityDomain,
    /// Digest of the canonical envelope approved by this holder/actor.
    pub envelope_digest: DigestRef,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ConcurrenceError {
    InvalidRequirement(EnvelopeError),
    InsufficientApprovals,
    InsufficientDistinctDomains,
    MissingRequiredDomain(AuthorityDomain),
    DuplicateHolder,
    DuplicateActor,
    ExcludedHolder,
    ExcludedActor,
    EnvelopeDigestMismatch,
    InvalidApprovalDigest(EnvelopeError),
    EmptyApprovalHolder,
    EmptyApprovalActor,
}

pub fn validate_concurrence(
    requirement: &ConcurrenceRequirement,
    approvals: &[ConcurrenceApproval],
    expected_envelope_digest: &DigestRef,
) -> Result<(), ConcurrenceError> {
    requirement
        .validate()
        .map_err(ConcurrenceError::InvalidRequirement)?;
    expected_envelope_digest
        .validate()
        .map_err(ConcurrenceError::InvalidApprovalDigest)?;

    if approvals.len() < requirement.min_approvals as usize {
        return Err(ConcurrenceError::InsufficientApprovals);
    }

    let mut holders: Vec<&str> = Vec::new();
    let mut actors: Vec<&str> = Vec::new();
    let mut domains: Vec<AuthorityDomain> = Vec::new();

    for approval in approvals {
        if approval.holder_id.trim().is_empty() {
            return Err(ConcurrenceError::EmptyApprovalHolder);
        }
        if approval.actor_id.trim().is_empty() {
            return Err(ConcurrenceError::EmptyApprovalActor);
        }
        approval
            .envelope_digest
            .validate()
            .map_err(ConcurrenceError::InvalidApprovalDigest)?;
        if &approval.envelope_digest != expected_envelope_digest {
            return Err(ConcurrenceError::EnvelopeDigestMismatch);
        }
        if requirement
            .excluded_holder_ids
            .iter()
            .any(|id| id == &approval.holder_id)
        {
            return Err(ConcurrenceError::ExcludedHolder);
        }
        if requirement
            .excluded_actor_ids
            .iter()
            .any(|id| id == &approval.actor_id)
        {
            return Err(ConcurrenceError::ExcludedActor);
        }
        if requirement.require_unique_holders && holders.contains(&approval.holder_id.as_str()) {
            return Err(ConcurrenceError::DuplicateHolder);
        }
        if requirement.require_unique_actors && actors.contains(&approval.actor_id.as_str()) {
            return Err(ConcurrenceError::DuplicateActor);
        }
        if !holders.contains(&approval.holder_id.as_str()) {
            holders.push(&approval.holder_id);
        }
        if !actors.contains(&approval.actor_id.as_str()) {
            actors.push(&approval.actor_id);
        }
        if !domains.contains(&approval.domain) {
            domains.push(approval.domain);
        }
    }

    if domains.len() < requirement.min_distinct_domains as usize {
        return Err(ConcurrenceError::InsufficientDistinctDomains);
    }
    for required in &requirement.required_domains {
        if !domains.contains(required) {
            return Err(ConcurrenceError::MissingRequiredDomain(*required));
        }
    }
    Ok(())
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConflictSubject {
    ConstitutionalHolder,
    ExecutingActor,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SeparationScope {
    Static,
    SameMatter,
    CoolingOff { minimum_gap_us: i64 },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SeparationRule {
    pub left: AuthorizationAction,
    pub right: AuthorizationAction,
    pub subject: ConflictSubject,
    pub scope: SeparationScope,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AuthorityActivation {
    pub holder_id: String,
    pub actor_id: String,
    pub domain: AuthorityDomain,
    pub action: AuthorizationAction,
    pub matter: MatterId,
    pub activated_at_us: i64,
}

impl AuthorityActivation {
    pub fn from_envelope(envelope: &AuthorizationEnvelope) -> Result<Self, EnvelopeError> {
        envelope.validate()?;
        Ok(Self {
            holder_id: envelope.holder_id.clone(),
            actor_id: envelope.actor_id.clone(),
            domain: envelope.domain()?,
            action: envelope.action,
            matter: envelope.matter.clone(),
            activated_at_us: envelope.valid_from_us,
        })
    }
}

pub fn violates_separation(
    rule: &SeparationRule,
    first: &AuthorityActivation,
    second: &AuthorityActivation,
) -> bool {
    let actions_match = (first.action == rule.left && second.action == rule.right)
        || (first.action == rule.right && second.action == rule.left);
    if !actions_match {
        return false;
    }

    let same_subject = match rule.subject {
        ConflictSubject::ConstitutionalHolder => first.holder_id == second.holder_id,
        ConflictSubject::ExecutingActor => first.actor_id == second.actor_id,
    };
    if !same_subject {
        return false;
    }

    match rule.scope {
        SeparationScope::Static => true,
        SeparationScope::SameMatter => first.matter == second.matter,
        SeparationScope::CoolingOff { minimum_gap_us } => {
            if minimum_gap_us <= 0 {
                return true;
            }
            let delta = (first.activated_at_us as i128 - second.activated_at_us as i128).abs();
            delta < minimum_gap_us as i128
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReceiptDecision {
    Authorized,
    Denied,
    Consumed,
    Revoked,
    Expired,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationReceipt {
    pub envelope_digest: DigestRef,
    pub decision: ReceiptDecision,
    pub verifier_id: String,
    pub verifier_version: String,
    pub evidence_refs: Vec<DigestRef>,
    pub approval_refs: Vec<DigestRef>,
    pub timestamp_us: i64,
    pub action_output: Option<DigestRef>,
    pub review_ref: Option<String>,
}

impl AuthorizationReceipt {
    pub fn validate(&self) -> Result<(), ReceiptError> {
        self.envelope_digest
            .validate()
            .map_err(ReceiptError::InvalidDigest)?;
        if self.verifier_id.trim().is_empty() {
            return Err(ReceiptError::EmptyVerifierId);
        }
        if self.verifier_version.trim().is_empty() {
            return Err(ReceiptError::EmptyVerifierVersion);
        }
        for digest in self.evidence_refs.iter().chain(self.approval_refs.iter()) {
            digest.validate().map_err(ReceiptError::InvalidDigest)?;
        }
        if let Some(output) = &self.action_output {
            output.validate().map_err(ReceiptError::InvalidDigest)?;
        }
        if self.review_ref.as_ref().is_some_and(|v| v.trim().is_empty()) {
            return Err(ReceiptError::EmptyReviewRef);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReceiptError {
    InvalidDigest(EnvelopeError),
    EmptyVerifierId,
    EmptyVerifierVersion,
    EmptyReviewRef,
}

fn push_u8(out: &mut Vec<u8>, value: u8) {
    out.push(value);
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_i64(out: &mut Vec<u8>, value: i64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_bool(out: &mut Vec<u8>, value: bool) {
    push_u8(out, u8::from(value));
}

fn push_str(out: &mut Vec<u8>, value: &str) -> Result<(), EnvelopeError> {
    let len = u32::try_from(value.len()).map_err(|_| EnvelopeError::CanonicalFieldTooLarge)?;
    push_u32(out, len);
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn encode_optional_i64(out: &mut Vec<u8>, value: Option<i64>) {
    match value {
        Some(value) => {
            push_u8(out, 1);
            push_i64(out, value);
        }
        None => push_u8(out, 0),
    }
}

fn encode_digest(out: &mut Vec<u8>, digest: &DigestRef) -> Result<(), EnvelopeError> {
    push_str(out, &digest.algorithm)?;
    push_str(out, &digest.value)
}

fn encode_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&DigestRef>,
) -> Result<(), EnvelopeError> {
    match digest {
        Some(digest) => {
            push_u8(out, 1);
            encode_digest(out, digest)
        }
        None => {
            push_u8(out, 0);
            Ok(())
        }
    }
}

fn branch_tag(branch: Branch) -> u8 {
    match branch {
        Branch::Deliberative => 1,
        Branch::Stewardship => 2,
        Branch::Justice => 3,
        Branch::Integrity => 4,
        Branch::CivicMandate => 5,
    }
}

fn guardian_tag(guardian: Guardian) -> u8 {
    match guardian {
        Guardian::RightsDefender => 1,
        Guardian::PublicEvidence => 2,
        Guardian::FutureGenerations => 3,
        Guardian::FiscalObservatory => 4,
        Guardian::PublicService => 5,
        Guardian::ProsecutionService => 6,
    }
}

fn encode_domain(out: &mut Vec<u8>, domain: AuthorityDomain) {
    match domain {
        AuthorityDomain::Constituent => push_u8(out, 1),
        AuthorityDomain::Branch(branch) => {
            push_u8(out, 2);
            push_u8(out, branch_tag(branch));
        }
        AuthorityDomain::Guardian(guardian) => {
            push_u8(out, 3);
            push_u8(out, guardian_tag(guardian));
        }
    }
}

fn encode_principal(out: &mut Vec<u8>, principal: AuthorityPrincipal) {
    match principal {
        AuthorityPrincipal::ConstituentSovereignty => push_u8(out, 1),
        AuthorityPrincipal::Branch(branch) => {
            push_u8(out, 2);
            push_u8(out, branch_tag(branch));
        }
        AuthorityPrincipal::Guardian(guardian) => {
            push_u8(out, 3);
            push_u8(out, guardian_tag(guardian));
        }
        AuthorityPrincipal::AutomatedAgent => push_u8(out, 4),
    }
}

fn power_tag(power: ConstitutionalPower) -> u8 {
    match power {
        ConstitutionalPower::ProposeOrdinaryLaw => 1,
        ConstitutionalPower::EnactOrdinaryLaw => 2,
        ConstitutionalPower::AppropriatePublicFunds => 3,
        ConstitutionalPower::RatifyTreaty => 4,
        ConstitutionalPower::AuthorizeEmergency => 5,
        ConstitutionalPower::ConductLegislativeOversight => 6,
        ConstitutionalPower::ExecuteLaw => 7,
        ConstitutionalPower::ExecuteAppropriation => 8,
        ConstitutionalPower::AdministerPublicService => 9,
        ConstitutionalPower::DirectPublicAdministration => 10,
        ConstitutionalPower::DeclareProvisionalEmergency => 11,
        ConstitutionalPower::AdjudicateDispute => 12,
        ConstitutionalPower::ConductConstitutionalReview => 13,
        ConstitutionalPower::IssueJudicialRemedy => 14,
        ConstitutionalPower::AuditPublicExpenditure => 15,
        ConstitutionalPower::AuditAuthorityUse => 16,
        ConstitutionalPower::InvestigatePublicIntegrity => 17,
        ConstitutionalPower::PublishIntegrityFinding => 18,
        ConstitutionalPower::ReferForProsecution => 19,
        ConstitutionalPower::AdministerElection => 20,
        ConstitutionalPower::CertifyMandate => 21,
        ConstitutionalPower::AdministerRecall => 22,
        ConstitutionalPower::AdministerInitiative => 23,
        ConstitutionalPower::AdministerSortition => 24,
        ConstitutionalPower::VerifyCivicEligibility => 25,
        ConstitutionalPower::CallConstitutionalConvention => 26,
        ConstitutionalPower::RatifyStructuralConstitution => 27,
        ConstitutionalPower::RatifyFoundationalCovenant => 28,
        ConstitutionalPower::WithdrawConstituentDelegation => 29,
        ConstitutionalPower::InitiateRightsChallenge => 30,
        ConstitutionalPower::PublishEvidenceAssessment => 31,
        ConstitutionalPower::InitiateFutureGenerationsReview => 32,
        ConstitutionalPower::PublishFiscalAssessment => 33,
        ConstitutionalPower::CertifyPublicServiceQualification => 34,
        ConstitutionalPower::InitiatePublicProsecution => 35,
    }
}

fn entitlement_tag(entitlement: ConstitutionalEntitlement) -> u8 {
    match entitlement {
        ConstitutionalEntitlement::RequestLawfulRecord => 1,
        ConstitutionalEntitlement::AccessSubmittedEvidence => 2,
        ConstitutionalEntitlement::ReceiveDecisionNotice => 3,
        ConstitutionalEntitlement::ObtainDecisionReasons => 4,
        ConstitutionalEntitlement::SubmitEvidence => 5,
        ConstitutionalEntitlement::ChallengePublicAction => 6,
        ConstitutionalEntitlement::SeekJudicialReview => 7,
        ConstitutionalEntitlement::PublishProtectedOversightReport => 8,
        ConstitutionalEntitlement::ReceiveProtectedDisclosure => 9,
    }
}

fn encode_action(out: &mut Vec<u8>, action: AuthorizationAction) {
    match action {
        AuthorizationAction::Sovereign(power) => {
            push_u8(out, 1);
            push_u8(out, power_tag(power));
        }
        AuthorizationAction::Entitlement(entitlement) => {
            push_u8(out, 2);
            push_u8(out, entitlement_tag(entitlement));
        }
    }
}

fn encode_source(out: &mut Vec<u8>, source: &CapabilitySource) -> Result<(), EnvelopeError> {
    match source {
        CapabilitySource::Charter { charter_id, version } => {
            push_u8(out, 1);
            push_str(out, charter_id)?;
            push_u32(out, *version);
        }
        CapabilitySource::ConstituentRatification { event_id } => {
            push_u8(out, 2);
            push_str(out, event_id)?;
        }
        CapabilitySource::Statute { proposal_id } => {
            push_u8(out, 3);
            push_str(out, proposal_id)?;
        }
        CapabilitySource::JudicialOrder { case_id } => {
            push_u8(out, 4);
            push_str(out, case_id)?;
        }
        CapabilitySource::EmergencyProtocol { declaration_id } => {
            push_u8(out, 5);
            push_str(out, declaration_id)?;
        }
        CapabilitySource::Delegation {
            parent_capability_id,
        } => {
            push_u8(out, 6);
            push_str(out, parent_capability_id)?;
        }
    }
    Ok(())
}

fn encode_matter(out: &mut Vec<u8>, matter: &MatterId) -> Result<(), EnvelopeError> {
    push_str(out, &matter.namespace)?;
    push_str(out, &matter.stable_id)
}

fn encode_resource(out: &mut Vec<u8>, resource: &ResourceBinding) -> Result<(), EnvelopeError> {
    push_str(out, &resource.kind)?;
    push_str(out, &resource.id)?;
    encode_digest(out, &resource.payload_digest)
}

fn encode_use_policy(out: &mut Vec<u8>, policy: &UsePolicy) {
    match policy {
        UsePolicy::OneShot => push_u8(out, 1),
        UsePolicy::Bounded { max_uses } => {
            push_u8(out, 2);
            push_u32(out, *max_uses);
        }
    }
}

fn encode_optional_concurrence(
    out: &mut Vec<u8>,
    requirement: Option<&ConcurrenceRequirement>,
) -> Result<(), EnvelopeError> {
    let Some(requirement) = requirement else {
        push_u8(out, 0);
        return Ok(());
    };
    push_u8(out, 1);
    push_u16(out, requirement.min_approvals);
    push_u16(out, requirement.min_distinct_domains);
    push_bool(out, requirement.require_unique_holders);
    push_bool(out, requirement.require_unique_actors);

    let mut domains = requirement.required_domains.clone();
    domains.sort_by_key(|domain| domain.rank());
    push_u16(
        out,
        u16::try_from(domains.len()).map_err(|_| EnvelopeError::CanonicalFieldTooLarge)?,
    );
    for domain in domains {
        encode_domain(out, domain);
    }

    let mut excluded_holders = requirement.excluded_holder_ids.clone();
    excluded_holders.sort();
    excluded_holders.dedup();
    push_u16(
        out,
        u16::try_from(excluded_holders.len())
            .map_err(|_| EnvelopeError::CanonicalFieldTooLarge)?,
    );
    for id in excluded_holders {
        push_str(out, &id)?;
    }

    let mut excluded_actors = requirement.excluded_actor_ids.clone();
    excluded_actors.sort();
    excluded_actors.dedup();
    push_u16(
        out,
        u16::try_from(excluded_actors.len())
            .map_err(|_| EnvelopeError::CanonicalFieldTooLarge)?,
    );
    for id in excluded_actors {
        push_str(out, &id)?;
    }
    Ok(())
}
