// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Rulebook-bound verification policy for binding Mycelix governance.
//!
//! This crate answers two policy questions without performing network or
//! cryptographic I/O:
//!
//! 1. Which already-verified credentials may establish a civic right?
//! 2. Which independently authorized observers may establish ballot-set finality?
//!
//! A valid signature or a distinct observer ID is not authority by itself.
//! Issuer trust, credential semantics, verifier provenance, institution,
//! jurisdiction, rulebook, and observer independence are explicit policy inputs.
//!
//! There are intentionally no Phi, reputation, stake, wealth, model-score, or
//! advisory-signal fields in this API.

use mycelix_governance_rights::CivicRight;
use mycelix_institutional_core::{
    AuthorityGrantId, Digest32, InstitutionId, JurisdictionId, PrincipalId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-verifier-policy-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_RULES: usize = 128;
const MAX_PREDICATES_PER_RULE: usize = 128;
const MAX_OBSERVERS: usize = 1024;
const MAX_TRUST_DOMAINS: usize = 256;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, PolicyError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(EligibilityPolicyId, "eligibility_policy_id");
id_type!(CredentialRuleId, "credential_rule_id");
id_type!(PredicateId, "predicate_id");
id_type!(FinalityPolicyId, "finality_policy_id");

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssuerTrustRequirement {
    /// Credential must be issued by this exact DID.
    ExactIssuer { issuer_did: String },
    /// Credential issuer must be authorized by an external trust registry under
    /// this exact registry/profile/relationship tuple.
    TrustRegistry {
        registry_ref: String,
        registry_profile: String,
        relationship: String,
    },
}

impl IssuerTrustRequirement {
    fn validate(&self) -> Result<(), PolicyError> {
        match self {
            Self::ExactIssuer { issuer_did } => {
                validate_did(issuer_did, "issuer_trust.exact_issuer")
            }
            Self::TrustRegistry {
                registry_ref,
                registry_profile,
                relationship,
            } => {
                validate_text(registry_ref, "issuer_trust.registry_ref", MAX_ID_BYTES)?;
                validate_profile(registry_profile, "issuer_trust.registry_profile")?;
                validate_text(relationship, "issuer_trust.relationship", MAX_ID_BYTES)
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PredicateRequirement {
    pub id: PredicateId,
    /// Profile defining the predicate semantics/proof contract.
    pub evidence_profile: String,
}

impl PredicateRequirement {
    fn validate(&self) -> Result<(), PolicyError> {
        validate_text(self.id.as_str(), "predicate.id", MAX_ID_BYTES)?;
        validate_profile(&self.evidence_profile, "predicate.evidence_profile")
    }
}

/// One acceptable credential rule.
///
/// Types are all-of: every type in `required_credential_types` must be present.
/// Schema IDs are any-of: the credential must use one listed schema.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialRule {
    pub id: CredentialRuleId,
    pub required_credential_types: Vec<String>,
    pub accepted_schema_ids: Vec<String>,
    pub issuer_trust: IssuerTrustRequirement,
    pub required_predicates: Vec<PredicateRequirement>,
}

impl CredentialRule {
    pub fn validate(&self) -> Result<(), PolicyError> {
        validate_text(self.id.as_str(), "credential_rule.id", MAX_ID_BYTES)?;
        if self.required_credential_types.is_empty() {
            return Err(PolicyError::NoCredentialTypes);
        }
        ensure_unique_texts(
            &self.required_credential_types,
            "credential_rule.required_credential_types",
        )?;
        for value in &self.required_credential_types {
            validate_text(value, "credential_rule.credential_type", MAX_ID_BYTES)?;
        }
        if self.accepted_schema_ids.is_empty() {
            return Err(PolicyError::NoCredentialSchemas);
        }
        ensure_unique_texts(
            &self.accepted_schema_ids,
            "credential_rule.accepted_schema_ids",
        )?;
        for value in &self.accepted_schema_ids {
            validate_text(value, "credential_rule.schema_id", MAX_ID_BYTES)?;
        }
        self.issuer_trust.validate()?;
        if self.required_predicates.len() > MAX_PREDICATES_PER_RULE {
            return Err(PolicyError::TooManyPredicates);
        }
        let mut predicate_ids = BTreeSet::new();
        for predicate in &self.required_predicates {
            predicate.validate()?;
            if !predicate_ids.insert(predicate.id.clone()) {
                return Err(PolicyError::DuplicatePredicate);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceCombination {
    /// At least one credential rule must match.
    Any,
    /// Every credential rule must be satisfied.
    All,
    /// At least this many different rules must be satisfied.
    AtLeast(u16),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EligibilityVerifierPolicy {
    pub protocol_version: String,
    pub id: EligibilityPolicyId,
    pub policy_digest: Digest32,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub right: CivicRight,
    pub credential_rules: Vec<CredentialRule>,
    pub combination: EvidenceCombination,
    /// When true, one credential cannot satisfy multiple required rules.
    pub distinct_credentials_across_rules: bool,
    /// Maximum age of credential verification/revocation evidence at snapshot.
    pub max_evidence_age_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Institutional authority that adopted this policy.
    pub authorized_by: AuthorityGrantId,
    pub policy_proof_ref: String,
}

impl EligibilityVerifierPolicy {
    pub fn validate(&self) -> Result<(), PolicyError> {
        require_protocol(&self.protocol_version)?;
        require_digest(self.policy_digest, "eligibility_policy.policy_digest")?;
        self.rulebook
            .validate()
            .map_err(|_| PolicyError::InvalidRulebook)?;
        if self.credential_rules.is_empty() || self.credential_rules.len() > MAX_RULES {
            return Err(PolicyError::InvalidRuleCount);
        }
        let mut ids = BTreeSet::new();
        for rule in &self.credential_rules {
            rule.validate()?;
            if !ids.insert(rule.id.clone()) {
                return Err(PolicyError::DuplicateCredentialRule);
            }
        }
        match self.combination {
            EvidenceCombination::Any | EvidenceCombination::All => {}
            EvidenceCombination::AtLeast(required) => {
                if required == 0 || required as usize > self.credential_rules.len() {
                    return Err(PolicyError::InvalidEvidenceThreshold);
                }
            }
        }
        if self.max_evidence_age_ms == 0 {
            return Err(PolicyError::ZeroEvidenceAge);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(PolicyError::InvalidTimeRange("eligibility policy"));
        }
        validate_text(
            self.authorized_by.as_str(),
            "eligibility_policy.authorized_by",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.policy_proof_ref,
            "eligibility_policy.policy_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn active_at(&self, timestamp_ms: u64) -> bool {
        self.valid_from_ms <= timestamp_ms && timestamp_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssuerTrustEvidence {
    ExactIssuer,
    TrustRegistry {
        registry_ref: String,
        registry_profile: String,
        relationship: String,
        trust_receipt_ref: String,
    },
}

impl IssuerTrustEvidence {
    fn validate(&self) -> Result<(), PolicyError> {
        match self {
            Self::ExactIssuer => Ok(()),
            Self::TrustRegistry {
                registry_ref,
                registry_profile,
                relationship,
                trust_receipt_ref,
            } => {
                validate_text(registry_ref, "issuer_evidence.registry_ref", MAX_ID_BYTES)?;
                validate_profile(registry_profile, "issuer_evidence.registry_profile")?;
                validate_text(relationship, "issuer_evidence.relationship", MAX_ID_BYTES)?;
                validate_text(
                    trust_receipt_ref,
                    "issuer_evidence.trust_receipt_ref",
                    MAX_ID_BYTES,
                )
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedPredicateEvidence {
    pub id: PredicateId,
    pub satisfied: bool,
    pub evidence_profile: String,
    pub proof_ref: String,
}

impl VerifiedPredicateEvidence {
    fn validate(&self) -> Result<(), PolicyError> {
        validate_text(self.id.as_str(), "predicate_evidence.id", MAX_ID_BYTES)?;
        validate_profile(
            &self.evidence_profile,
            "predicate_evidence.evidence_profile",
        )?;
        validate_text(&self.proof_ref, "predicate_evidence.proof_ref", MAX_ID_BYTES)
    }
}

/// Output of a cryptographic credential-verification adapter.
///
/// The host is responsible for constructing this only after actually checking
/// the credential signature, holder/subject binding, status/revocation, schema,
/// proof format, and any trust-registry receipt. This struct never turns a raw
/// user assertion into verified evidence by itself.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCredentialEvidence {
    pub credential_id: String,
    pub subject: PrincipalId,
    pub issuer_did: String,
    pub credential_types: Vec<String>,
    pub schema_id: String,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub verified_at_ms: u64,
    pub revocation_checked_at_ms: u64,
    pub verification_receipt_ref: String,
    pub issuer_trust: IssuerTrustEvidence,
    pub predicates: Vec<VerifiedPredicateEvidence>,
}

impl VerifiedCredentialEvidence {
    pub fn validate(&self) -> Result<(), PolicyError> {
        validate_text(&self.credential_id, "credential.id", MAX_ID_BYTES)?;
        validate_text(self.subject.as_str(), "credential.subject", MAX_ID_BYTES)?;
        validate_did(&self.issuer_did, "credential.issuer_did")?;
        if self.credential_types.is_empty() {
            return Err(PolicyError::NoCredentialTypes);
        }
        ensure_unique_texts(&self.credential_types, "credential.types")?;
        for value in &self.credential_types {
            validate_text(value, "credential.type", MAX_ID_BYTES)?;
        }
        validate_text(&self.schema_id, "credential.schema_id", MAX_ID_BYTES)?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(PolicyError::InvalidTimeRange("credential"));
        }
        if self.verified_at_ms == 0 || self.revocation_checked_at_ms == 0 {
            return Err(PolicyError::ZeroVerificationTime);
        }
        validate_text(
            &self.verification_receipt_ref,
            "credential.verification_receipt_ref",
            MAX_ID_BYTES,
        )?;
        self.issuer_trust.validate()?;
        if self.predicates.len() > MAX_PREDICATES_PER_RULE {
            return Err(PolicyError::TooManyPredicates);
        }
        let mut ids = BTreeSet::new();
        for predicate in &self.predicates {
            predicate.validate()?;
            if !ids.insert(predicate.id.clone()) {
                return Err(PolicyError::DuplicatePredicate);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EligibilityPermitEvidence {
    pub policy_id: EligibilityPolicyId,
    pub policy_digest: Digest32,
    pub principal: PrincipalId,
    pub right: CivicRight,
    pub matched_rules: Vec<CredentialRuleId>,
    pub credential_ids: Vec<String>,
    pub evaluated_at_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EligibilityDecision {
    Eligible(EligibilityPermitEvidence),
    Ineligible(EligibilityDenial),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EligibilityDenial {
    PolicyInactive,
    NoEvidence,
    EvidenceInvalid,
    SubjectMismatch,
    EvidenceTooOld,
    CredentialInactive,
    InsufficientMatchingRules,
    DistinctCredentialRequirementUnsatisfied,
}

pub fn evaluate_eligibility(
    policy: &EligibilityVerifierPolicy,
    principal: &PrincipalId,
    snapshot_at_ms: u64,
    evidence: &[VerifiedCredentialEvidence],
) -> Result<EligibilityDecision, PolicyError> {
    policy.validate()?;
    if !policy.active_at(snapshot_at_ms) {
        return Ok(EligibilityDecision::Ineligible(
            EligibilityDenial::PolicyInactive,
        ));
    }
    if evidence.is_empty() {
        return Ok(EligibilityDecision::Ineligible(
            EligibilityDenial::NoEvidence,
        ));
    }

    for credential in evidence {
        if credential.validate().is_err() {
            return Ok(EligibilityDecision::Ineligible(
                EligibilityDenial::EvidenceInvalid,
            ));
        }
        if &credential.subject != principal {
            return Ok(EligibilityDecision::Ineligible(
                EligibilityDenial::SubjectMismatch,
            ));
        }
        if !fresh_at(
            credential.verified_at_ms,
            snapshot_at_ms,
            policy.max_evidence_age_ms,
        ) || !fresh_at(
            credential.revocation_checked_at_ms,
            snapshot_at_ms,
            policy.max_evidence_age_ms,
        ) {
            return Ok(EligibilityDecision::Ineligible(
                EligibilityDenial::EvidenceTooOld,
            ));
        }
        if !(credential.valid_from_ms <= snapshot_at_ms
            && snapshot_at_ms < credential.valid_until_ms)
        {
            return Ok(EligibilityDecision::Ineligible(
                EligibilityDenial::CredentialInactive,
            ));
        }
    }

    let mut matches: Vec<(CredentialRuleId, String)> = Vec::new();
    for rule in &policy.credential_rules {
        if let Some(credential) = evidence.iter().find(|credential| rule_matches(rule, credential)) {
            matches.push((rule.id.clone(), credential.credential_id.clone()));
        }
    }

    let required = match policy.combination {
        EvidenceCombination::Any => 1usize,
        EvidenceCombination::All => policy.credential_rules.len(),
        EvidenceCombination::AtLeast(required) => required as usize,
    };
    if matches.len() < required {
        return Ok(EligibilityDecision::Ineligible(
            EligibilityDenial::InsufficientMatchingRules,
        ));
    }

    if policy.distinct_credentials_across_rules {
        let distinct: BTreeSet<&str> = matches
            .iter()
            .map(|(_, credential_id)| credential_id.as_str())
            .collect();
        if distinct.len() < required {
            return Ok(EligibilityDecision::Ineligible(
                EligibilityDenial::DistinctCredentialRequirementUnsatisfied,
            ));
        }
    }

    matches.truncate(required.max(matches.len().min(required)));
    let matched_rules = matches.iter().map(|(id, _)| id.clone()).collect();
    let credential_ids = matches
        .iter()
        .map(|(_, credential_id)| credential_id.clone())
        .collect();
    Ok(EligibilityDecision::Eligible(EligibilityPermitEvidence {
        policy_id: policy.id.clone(),
        policy_digest: policy.policy_digest,
        principal: principal.clone(),
        right: policy.right.clone(),
        matched_rules,
        credential_ids,
        evaluated_at_ms: snapshot_at_ms,
    }))
}

fn rule_matches(rule: &CredentialRule, credential: &VerifiedCredentialEvidence) -> bool {
    if !rule
        .required_credential_types
        .iter()
        .all(|required| credential.credential_types.contains(required))
    {
        return false;
    }
    if !rule.accepted_schema_ids.contains(&credential.schema_id) {
        return false;
    }
    if !issuer_matches(&rule.issuer_trust, credential) {
        return false;
    }
    rule.required_predicates.iter().all(|required| {
        credential.predicates.iter().any(|evidence| {
            evidence.id == required.id
                && evidence.evidence_profile == required.evidence_profile
                && evidence.satisfied
        })
    })
}

fn issuer_matches(
    requirement: &IssuerTrustRequirement,
    credential: &VerifiedCredentialEvidence,
) -> bool {
    match (requirement, &credential.issuer_trust) {
        (
            IssuerTrustRequirement::ExactIssuer { issuer_did },
            IssuerTrustEvidence::ExactIssuer,
        ) => &credential.issuer_did == issuer_did,
        (
            IssuerTrustRequirement::TrustRegistry {
                registry_ref,
                registry_profile,
                relationship,
            },
            IssuerTrustEvidence::TrustRegistry {
                registry_ref: evidence_registry,
                registry_profile: evidence_profile,
                relationship: evidence_relationship,
                trust_receipt_ref,
            },
        ) => {
            registry_ref == evidence_registry
                && registry_profile == evidence_profile
                && relationship == evidence_relationship
                && !trust_receipt_ref.trim().is_empty()
        }
        _ => false,
    }
}

fn fresh_at(observed_at_ms: u64, target_ms: u64, max_age_ms: u64) -> bool {
    observed_at_ms <= target_ms && target_ms - observed_at_ms <= max_age_ms
}

// ============================================================================
// Ballot-set finality policy
// ============================================================================

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BallotFinalityPolicy {
    pub protocol_version: String,
    pub id: FinalityPolicyId,
    pub policy_digest: Digest32,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub ballot_set_digest_profile: String,
    pub min_observers: u32,
    /// Minimum number of independent trust domains represented.
    pub min_trust_domains: u32,
    /// Upper bound on how many observer attestations one trust domain can
    /// contribute toward the threshold.
    pub max_observers_per_trust_domain: u32,
    pub allowed_trust_domains: Vec<String>,
    /// Optional exact observer allowlist. Empty means observer identity is
    /// governed by the external domain authorization receipt.
    pub allowed_observer_ids: Vec<String>,
    pub min_observation_delay_ms: u64,
    pub max_finalization_delay_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub authorized_by: AuthorityGrantId,
    pub policy_proof_ref: String,
}

impl BallotFinalityPolicy {
    pub fn validate(&self) -> Result<(), PolicyError> {
        require_protocol(&self.protocol_version)?;
        require_digest(self.policy_digest, "finality_policy.policy_digest")?;
        self.rulebook
            .validate()
            .map_err(|_| PolicyError::InvalidRulebook)?;
        validate_profile(
            &self.ballot_set_digest_profile,
            "finality_policy.ballot_set_digest_profile",
        )?;
        if self.min_observers == 0
            || self.min_trust_domains == 0
            || self.max_observers_per_trust_domain == 0
        {
            return Err(PolicyError::InvalidObserverThreshold);
        }
        if self.min_trust_domains > self.min_observers {
            return Err(PolicyError::InvalidObserverThreshold);
        }
        if self.allowed_trust_domains.is_empty()
            || self.allowed_trust_domains.len() > MAX_TRUST_DOMAINS
        {
            return Err(PolicyError::InvalidTrustDomainCount);
        }
        ensure_unique_texts(
            &self.allowed_trust_domains,
            "finality_policy.allowed_trust_domains",
        )?;
        for domain in &self.allowed_trust_domains {
            validate_text(domain, "finality_policy.trust_domain", MAX_ID_BYTES)?;
        }
        if self.allowed_observer_ids.len() > MAX_OBSERVERS {
            return Err(PolicyError::TooManyObservers);
        }
        ensure_unique_texts(
            &self.allowed_observer_ids,
            "finality_policy.allowed_observer_ids",
        )?;
        for observer in &self.allowed_observer_ids {
            validate_did(observer, "finality_policy.observer_id")?;
        }
        if self.max_finalization_delay_ms < self.min_observation_delay_ms {
            return Err(PolicyError::InvalidFinalityWindow);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(PolicyError::InvalidTimeRange("finality policy"));
        }
        validate_text(
            self.authorized_by.as_str(),
            "finality_policy.authorized_by",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.policy_proof_ref,
            "finality_policy.policy_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn active_at(&self, timestamp_ms: u64) -> bool {
        self.valid_from_ms <= timestamp_ms && timestamp_ms < self.valid_until_ms
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObserverAttestation {
    pub observer_id: String,
    pub trust_domain_id: String,
    pub ballot_set_digest: Digest32,
    pub ballot_set_digest_profile: String,
    pub observed_at_ms: u64,
    /// Receipt proving this observer was authorized by this trust domain and
    /// signed this exact ballot-set observation.
    pub authorization_receipt_ref: String,
    pub attestation_receipt_ref: String,
}

impl ObserverAttestation {
    pub fn validate(&self) -> Result<(), PolicyError> {
        validate_did(&self.observer_id, "observer_attestation.observer_id")?;
        validate_text(
            &self.trust_domain_id,
            "observer_attestation.trust_domain_id",
            MAX_ID_BYTES,
        )?;
        require_digest(
            self.ballot_set_digest,
            "observer_attestation.ballot_set_digest",
        )?;
        validate_profile(
            &self.ballot_set_digest_profile,
            "observer_attestation.ballot_set_digest_profile",
        )?;
        if self.observed_at_ms == 0 {
            return Err(PolicyError::ZeroVerificationTime);
        }
        validate_text(
            &self.authorization_receipt_ref,
            "observer_attestation.authorization_receipt_ref",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.attestation_receipt_ref,
            "observer_attestation.attestation_receipt_ref",
            MAX_ID_BYTES,
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalityPermitEvidence {
    pub policy_id: FinalityPolicyId,
    pub policy_digest: Digest32,
    pub ballot_set_digest: Digest32,
    pub observer_ids: Vec<String>,
    pub trust_domains: Vec<String>,
    pub evaluated_at_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FinalityDecision {
    Final(FinalityPermitEvidence),
    NotFinal(FinalityDenial),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FinalityDenial {
    PolicyInactive,
    NoAttestations,
    InvalidAttestation,
    BallotSetMismatch,
    UnapprovedTrustDomain,
    UnapprovedObserver,
    ObservationTooEarly,
    ObservationTooLate,
    DuplicateObserver,
    TooManyObserversFromTrustDomain,
    InsufficientObservers,
    InsufficientIndependentTrustDomains,
}

pub fn evaluate_ballot_set_finality(
    policy: &BallotFinalityPolicy,
    ballot_set_digest: Digest32,
    ballot_closed_at_ms: u64,
    evaluated_at_ms: u64,
    attestations: &[ObserverAttestation],
) -> Result<FinalityDecision, PolicyError> {
    policy.validate()?;
    if !policy.active_at(evaluated_at_ms) {
        return Ok(FinalityDecision::NotFinal(FinalityDenial::PolicyInactive));
    }
    if attestations.is_empty() {
        return Ok(FinalityDecision::NotFinal(FinalityDenial::NoAttestations));
    }

    let earliest = ballot_closed_at_ms.saturating_add(policy.min_observation_delay_ms);
    let latest = ballot_closed_at_ms.saturating_add(policy.max_finalization_delay_ms);
    let mut observer_ids = BTreeSet::new();
    let mut domains: BTreeMap<String, u32> = BTreeMap::new();

    for attestation in attestations {
        if attestation.validate().is_err() {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::InvalidAttestation,
            ));
        }
        if attestation.ballot_set_digest != ballot_set_digest
            || attestation.ballot_set_digest_profile != policy.ballot_set_digest_profile
        {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::BallotSetMismatch,
            ));
        }
        if !policy
            .allowed_trust_domains
            .contains(&attestation.trust_domain_id)
        {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::UnapprovedTrustDomain,
            ));
        }
        if !policy.allowed_observer_ids.is_empty()
            && !policy.allowed_observer_ids.contains(&attestation.observer_id)
        {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::UnapprovedObserver,
            ));
        }
        if attestation.observed_at_ms < earliest {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::ObservationTooEarly,
            ));
        }
        if attestation.observed_at_ms > latest || attestation.observed_at_ms > evaluated_at_ms {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::ObservationTooLate,
            ));
        }
        if !observer_ids.insert(attestation.observer_id.clone()) {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::DuplicateObserver,
            ));
        }
        let count = domains.entry(attestation.trust_domain_id.clone()).or_insert(0);
        *count += 1;
        if *count > policy.max_observers_per_trust_domain {
            return Ok(FinalityDecision::NotFinal(
                FinalityDenial::TooManyObserversFromTrustDomain,
            ));
        }
    }

    if observer_ids.len() < policy.min_observers as usize {
        return Ok(FinalityDecision::NotFinal(
            FinalityDenial::InsufficientObservers,
        ));
    }
    if domains.len() < policy.min_trust_domains as usize {
        return Ok(FinalityDecision::NotFinal(
            FinalityDenial::InsufficientIndependentTrustDomains,
        ));
    }

    Ok(FinalityDecision::Final(FinalityPermitEvidence {
        policy_id: policy.id.clone(),
        policy_digest: policy.policy_digest,
        ballot_set_digest,
        observer_ids: observer_ids.into_iter().collect(),
        trust_domains: domains.into_keys().collect(),
        evaluated_at_ms,
    }))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PolicyError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    InvalidDid(&'static str),
    ZeroDigest(&'static str),
    InvalidRulebook,
    NoCredentialTypes,
    NoCredentialSchemas,
    TooManyPredicates,
    DuplicatePredicate,
    InvalidRuleCount,
    DuplicateCredentialRule,
    InvalidEvidenceThreshold,
    ZeroEvidenceAge,
    InvalidTimeRange(&'static str),
    ZeroVerificationTime,
    InvalidObserverThreshold,
    InvalidTrustDomainCount,
    TooManyObservers,
    InvalidFinalityWindow,
    DuplicateText(&'static str),
}

impl fmt::Display for PolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong governance verifier policy version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a canonical profile token"),
            Self::InvalidDid(field) => write!(f, "{field} must be a DID"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::InvalidRulebook => write!(f, "invalid rulebook reference"),
            Self::NoCredentialTypes => write!(f, "credential rule/evidence requires credential types"),
            Self::NoCredentialSchemas => write!(f, "credential rule requires accepted schemas"),
            Self::TooManyPredicates => write!(f, "too many predicate requirements/evidence items"),
            Self::DuplicatePredicate => write!(f, "duplicate predicate id"),
            Self::InvalidRuleCount => write!(f, "credential rule count is outside protocol bounds"),
            Self::DuplicateCredentialRule => write!(f, "duplicate credential rule id"),
            Self::InvalidEvidenceThreshold => write!(f, "invalid evidence combination threshold"),
            Self::ZeroEvidenceAge => write!(f, "maximum evidence age must be non-zero"),
            Self::InvalidTimeRange(field) => write!(f, "invalid time range for {field}"),
            Self::ZeroVerificationTime => write!(f, "verification/observation timestamp must be non-zero"),
            Self::InvalidObserverThreshold => write!(f, "invalid observer/trust-domain threshold"),
            Self::InvalidTrustDomainCount => write!(f, "invalid allowed trust-domain count"),
            Self::TooManyObservers => write!(f, "observer allowlist exceeds protocol bound"),
            Self::InvalidFinalityWindow => write!(f, "invalid ballot finality observation window"),
            Self::DuplicateText(field) => write!(f, "{field} contains duplicate values"),
        }
    }
}

impl std::error::Error for PolicyError {}

fn require_protocol(version: &str) -> Result<(), PolicyError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(PolicyError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), PolicyError> {
    if value.trim().is_empty() {
        return Err(PolicyError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(PolicyError::TooLong(field));
    }
    Ok(())
}

fn validate_did(value: &str, field: &'static str) -> Result<(), PolicyError> {
    validate_text(value, field, MAX_ID_BYTES)?;
    if !value.starts_with("did:") {
        return Err(PolicyError::InvalidDid(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), PolicyError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(PolicyError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), PolicyError> {
    if digest.is_zero() {
        Err(PolicyError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

fn ensure_unique_texts(values: &[String], field: &'static str) -> Result<(), PolicyError> {
    let unique: BTreeSet<&str> = values.iter().map(String::as_str).collect();
    if unique.len() != values.len() {
        return Err(PolicyError::DuplicateText(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, RulebookRef};

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:civic:v1").unwrap(),
            version: "1".into(),
            digest: digest(7),
        }
    }

    fn credential_rule(id: &str, schema: &str, issuer: &str) -> CredentialRule {
        CredentialRule {
            id: CredentialRuleId::new(id).unwrap(),
            required_credential_types: vec!["GovernanceMembershipCredential".into()],
            accepted_schema_ids: vec![schema.into()],
            issuer_trust: IssuerTrustRequirement::ExactIssuer {
                issuer_did: issuer.into(),
            },
            required_predicates: vec![],
        }
    }

    fn eligibility_policy() -> EligibilityVerifierPolicy {
        EligibilityVerifierPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            id: EligibilityPolicyId::new("eligibility:civic:v1").unwrap(),
            policy_digest: digest(8),
            institution: InstitutionId::new("institution:civic").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:za").unwrap()),
            rulebook: rulebook(),
            right: CivicRight::Vote,
            credential_rules: vec![credential_rule(
                "membership",
                "schema:governance-membership:v1",
                "did:mycelix:trusted-issuer",
            )],
            combination: EvidenceCombination::Any,
            distinct_credentials_across_rules: true,
            max_evidence_age_ms: 60_000,
            valid_from_ms: 1_000,
            valid_until_ms: 100_000,
            authorized_by: AuthorityGrantId::new("grant:adopt-eligibility-policy:v1").unwrap(),
            policy_proof_ref: "proof:eligibility-policy:v1".into(),
        }
    }

    fn credential(subject: &PrincipalId) -> VerifiedCredentialEvidence {
        VerifiedCredentialEvidence {
            credential_id: "urn:vc:membership:alice".into(),
            subject: subject.clone(),
            issuer_did: "did:mycelix:trusted-issuer".into(),
            credential_types: vec![
                "VerifiableCredential".into(),
                "GovernanceMembershipCredential".into(),
            ],
            schema_id: "schema:governance-membership:v1".into(),
            valid_from_ms: 1_000,
            valid_until_ms: 100_000,
            verified_at_ms: 9_900,
            revocation_checked_at_ms: 9_900,
            verification_receipt_ref: "receipt:vc:alice".into(),
            issuer_trust: IssuerTrustEvidence::ExactIssuer,
            predicates: vec![],
        }
    }

    #[test]
    fn trusted_exact_issuer_credential_grants_vote_eligibility() {
        let principal = PrincipalId::new("did:mycelix:alice").unwrap();
        let decision = evaluate_eligibility(
            &eligibility_policy(),
            &principal,
            10_000,
            &[credential(&principal)],
        )
        .unwrap();
        assert!(matches!(decision, EligibilityDecision::Eligible(_)));
    }

    #[test]
    fn valid_signature_from_wrong_issuer_does_not_grant_authority() {
        let principal = PrincipalId::new("did:mycelix:alice").unwrap();
        let mut evidence = credential(&principal);
        evidence.issuer_did = "did:mycelix:self-issued".into();
        let decision = evaluate_eligibility(
            &eligibility_policy(),
            &principal,
            10_000,
            &[evidence],
        )
        .unwrap();
        assert_eq!(
            decision,
            EligibilityDecision::Ineligible(EligibilityDenial::InsufficientMatchingRules)
        );
    }

    #[test]
    fn stale_revocation_check_fails_closed() {
        let principal = PrincipalId::new("did:mycelix:alice").unwrap();
        let mut evidence = credential(&principal);
        evidence.revocation_checked_at_ms = 1_000;
        let decision = evaluate_eligibility(
            &eligibility_policy(),
            &principal,
            70_000,
            &[evidence],
        )
        .unwrap();
        assert_eq!(
            decision,
            EligibilityDecision::Ineligible(EligibilityDenial::EvidenceTooOld)
        );
    }

    fn finality_policy() -> BallotFinalityPolicy {
        BallotFinalityPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            id: FinalityPolicyId::new("finality:civic:v1").unwrap(),
            policy_digest: digest(9),
            institution: InstitutionId::new("institution:civic").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:za").unwrap()),
            rulebook: rulebook(),
            ballot_set_digest_profile: "mycelix-ballot-set-v1".into(),
            min_observers: 3,
            min_trust_domains: 3,
            max_observers_per_trust_domain: 1,
            allowed_trust_domains: vec!["domain:a".into(), "domain:b".into(), "domain:c".into()],
            allowed_observer_ids: vec![],
            min_observation_delay_ms: 1_000,
            max_finalization_delay_ms: 60_000,
            valid_from_ms: 1_000,
            valid_until_ms: 200_000,
            authorized_by: AuthorityGrantId::new("grant:finality-policy:v1").unwrap(),
            policy_proof_ref: "proof:finality-policy:v1".into(),
        }
    }

    fn attestation(observer: &str, domain: &str) -> ObserverAttestation {
        ObserverAttestation {
            observer_id: format!("did:mycelix:{observer}"),
            trust_domain_id: domain.into(),
            ballot_set_digest: digest(11),
            ballot_set_digest_profile: "mycelix-ballot-set-v1".into(),
            observed_at_ms: 11_500,
            authorization_receipt_ref: format!("receipt:authorization:{observer}"),
            attestation_receipt_ref: format!("receipt:attestation:{observer}"),
        }
    }

    #[test]
    fn independent_trust_domains_can_establish_finality() {
        let evidence = vec![
            attestation("a", "domain:a"),
            attestation("b", "domain:b"),
            attestation("c", "domain:c"),
        ];
        let decision = evaluate_ballot_set_finality(
            &finality_policy(),
            digest(11),
            10_000,
            12_000,
            &evidence,
        )
        .unwrap();
        assert!(matches!(decision, FinalityDecision::Final(_)));
    }

    #[test]
    fn many_observer_ids_in_one_domain_do_not_create_independence() {
        let mut policy = finality_policy();
        policy.max_observers_per_trust_domain = 3;
        policy.allowed_trust_domains = vec!["domain:a".into(), "domain:b".into(), "domain:c".into()];
        let evidence = vec![
            attestation("a1", "domain:a"),
            attestation("a2", "domain:a"),
            attestation("a3", "domain:a"),
        ];
        let decision = evaluate_ballot_set_finality(
            &policy,
            digest(11),
            10_000,
            12_000,
            &evidence,
        )
        .unwrap();
        assert_eq!(
            decision,
            FinalityDecision::NotFinal(FinalityDenial::InsufficientIndependentTrustDomains)
        );
    }

    #[test]
    fn wrong_ballot_set_digest_fails_closed() {
        let evidence = vec![
            attestation("a", "domain:a"),
            attestation("b", "domain:b"),
            attestation("c", "domain:c"),
        ];
        let decision = evaluate_ballot_set_finality(
            &finality_policy(),
            digest(12),
            10_000,
            12_000,
            &evidence,
        )
        .unwrap();
        assert_eq!(
            decision,
            FinalityDecision::NotFinal(FinalityDenial::BallotSetMismatch)
        );
    }

    #[test]
    fn json_round_trip_preserves_policy_identity() {
        let policy = eligibility_policy();
        let encoded = serde_json::to_string(&policy).unwrap();
        let decoded: EligibilityVerifierPolicy = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, policy);
    }
}
