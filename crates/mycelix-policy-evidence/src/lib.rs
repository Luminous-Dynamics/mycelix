// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure cross-domain qualification for immutable policy-record authenticity and
//! institutional adoption evidence.
//!
//! Policy semantics are defined upstream by domain-specific canonical identity
//! theorems. This crate only joins independently shaped evidence for one exact
//! semantic policy identity while preserving verifier lifetimes through the
//! existing monotone evidence-lease algebra.
//!
//! Record authenticity, institutional adoption, generation currentness, policy
//! authority, and external-effect authority remain distinct facts.

use mycelix_authority_evidence_lease::{EvidenceLease, EvidenceLeaseError};
use mycelix_institutional_core::{
    Digest32, InstitutionId, JurisdictionId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const POLICY_RECORD_EVIDENCE_PROTOCOL: &str = "mycelix-policy-record-evidence-v0.1";
pub const POLICY_ADOPTION_EVIDENCE_PROTOCOL: &str = "mycelix-policy-adoption-evidence-v0.1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Domain-supplied canonical semantic policy identity.
///
/// This crate never computes this digest. The policy-owning domain must derive it
/// from its registered semantic profile before constructing evidence subjects.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyIdentityRef {
    pub digest: Digest32,
    pub profile: String,
}

impl PolicyIdentityRef {
    pub fn validate(&self) -> Result<(), PolicyEvidenceError> {
        if self.digest.is_zero() {
            return Err(PolicyEvidenceError::ZeroPolicyDigest);
        }
        validate_profile(&self.profile)
    }

    pub fn grants_authority(&self) -> bool {
        false
    }
}

/// Exact immutable-record authentication subject.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyRecordSubject {
    pub policy: PolicyIdentityRef,
    pub policy_record_ref: String,
}

impl PolicyRecordSubject {
    pub fn validate(&self) -> Result<(), PolicyEvidenceError> {
        self.policy.validate()?;
        validate_ref(&self.policy_record_ref)
    }
}

/// Exact institutional-adoption subject expected by the consuming domain.
///
/// The policy-owning domain decides which institutional context ought to govern
/// the policy. This crate only exact-matches evidence to that supplied context.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyAdoptionSubject {
    pub policy: PolicyIdentityRef,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub authority_ref: String,
    pub adoption_proof_ref: String,
}

impl PolicyAdoptionSubject {
    pub fn validate(&self) -> Result<(), PolicyEvidenceError> {
        self.policy.validate()?;
        validate_ref(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            validate_ref(jurisdiction.as_str())?;
        }
        self.rulebook
            .validate()
            .map_err(|_| PolicyEvidenceError::InvalidRulebook)?;
        validate_ref(&self.authority_ref)?;
        validate_ref(&self.adoption_proof_ref)
    }
}

/// Transport/evidence-shaped authentication of one exact immutable policy record.
///
/// Deserialization does not establish verifier provenance. The live caller must
/// obtain this receipt from its independently qualified record-verifier boundary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyRecordVerificationEvidence {
    pub protocol_version: String,
    pub subject: PolicyRecordSubject,
    pub record_proof_ref: String,
    pub verifier_ref: String,
    pub verification_ref: String,
    pub lease: EvidenceLease,
}

impl PolicyRecordVerificationEvidence {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), PolicyEvidenceError> {
        if self.protocol_version != POLICY_RECORD_EVIDENCE_PROTOCOL {
            return Err(PolicyEvidenceError::WrongRecordEvidenceProtocol);
        }
        self.subject.validate()?;
        validate_ref(&self.record_proof_ref)?;
        validate_ref(&self.verifier_ref)?;
        validate_ref(&self.verification_ref)?;
        self.lease.validate_at(now_ms).map_err(Into::into)
    }

    pub fn grants_authority(&self) -> bool {
        false
    }
}

/// Transport/evidence-shaped verification of institutional adoption for one
/// exact policy identity and exact institutional context.
///
/// Deserialization does not establish verifier provenance. The live caller must
/// obtain this receipt from its independently qualified adoption-verifier boundary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyAdoptionVerificationEvidence {
    pub protocol_version: String,
    pub subject: PolicyAdoptionSubject,
    pub verifier_ref: String,
    pub verification_ref: String,
    pub lease: EvidenceLease,
}

impl PolicyAdoptionVerificationEvidence {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), PolicyEvidenceError> {
        if self.protocol_version != POLICY_ADOPTION_EVIDENCE_PROTOCOL {
            return Err(PolicyEvidenceError::WrongAdoptionEvidenceProtocol);
        }
        self.subject.validate()?;
        validate_ref(&self.verifier_ref)?;
        validate_ref(&self.verification_ref)?;
        self.lease.validate_at(now_ms).map_err(Into::into)
    }

    pub fn grants_authority(&self) -> bool {
        false
    }
}

/// Process-local proof that record verification evidence exact-matched the
/// supplied immutable-record subject while its dynamic verifier lease was live.
///
/// This type intentionally does not implement `Deserialize`.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedPolicyRecordEvidence {
    subject: PolicyRecordSubject,
    evidence: PolicyRecordVerificationEvidence,
}

impl QualifiedPolicyRecordEvidence {
    pub fn subject(&self) -> &PolicyRecordSubject {
        &self.subject
    }

    pub fn evidence(&self) -> &PolicyRecordVerificationEvidence {
        &self.evidence
    }

    pub fn lease(&self) -> &EvidenceLease {
        &self.evidence.lease
    }

    pub fn verifier_origin_verified_here(&self) -> bool {
        false
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Process-local proof that adoption verification evidence exact-matched the
/// supplied institutional-adoption subject while its dynamic verifier lease was
/// live.
///
/// This type intentionally does not implement `Deserialize`.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedPolicyAdoptionEvidence {
    subject: PolicyAdoptionSubject,
    evidence: PolicyAdoptionVerificationEvidence,
}

impl QualifiedPolicyAdoptionEvidence {
    pub fn subject(&self) -> &PolicyAdoptionSubject {
        &self.subject
    }

    pub fn evidence(&self) -> &PolicyAdoptionVerificationEvidence {
        &self.evidence
    }

    pub fn lease(&self) -> &EvidenceLease {
        &self.evidence.lease
    }

    pub fn verifier_origin_verified_here(&self) -> bool {
        false
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Joined record-authenticity + institutional-adoption evidence for one exact
/// canonical policy identity.
///
/// The joined lease is the exact monotone intersection of both independent
/// verifier leases. The result is evidence for a later currentness/authority
/// theorem; it is not currentness or authority itself.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedPolicyEvidenceBundle {
    record: QualifiedPolicyRecordEvidence,
    adoption: QualifiedPolicyAdoptionEvidence,
    lease: EvidenceLease,
}

impl QualifiedPolicyEvidenceBundle {
    pub fn policy_identity(&self) -> &PolicyIdentityRef {
        &self.record.subject.policy
    }

    pub fn record(&self) -> &QualifiedPolicyRecordEvidence {
        &self.record
    }

    pub fn adoption(&self) -> &QualifiedPolicyAdoptionEvidence {
        &self.adoption
    }

    pub fn lease(&self) -> &EvidenceLease {
        &self.lease
    }

    pub fn generation_currentness_verified_here(&self) -> bool {
        false
    }

    pub fn verifier_origins_verified_here(&self) -> bool {
        false
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_administrative_decision_authority(&self) -> bool {
        false
    }

    pub fn grants_review_authority(&self) -> bool {
        false
    }

    pub fn grants_execution_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Exact-match immutable-record verification evidence to the expected subject.
pub fn qualify_policy_record_evidence(
    expected: PolicyRecordSubject,
    evidence: PolicyRecordVerificationEvidence,
    now_ms: u64,
) -> Result<QualifiedPolicyRecordEvidence, PolicyEvidenceError> {
    expected.validate()?;
    evidence.validate_at(now_ms)?;
    if evidence.subject != expected {
        return Err(PolicyEvidenceError::RecordSubjectMismatch);
    }
    Ok(QualifiedPolicyRecordEvidence {
        subject: expected,
        evidence,
    })
}

/// Exact-match institutional-adoption verification evidence to the expected
/// typed institutional context.
pub fn qualify_policy_adoption_evidence(
    expected: PolicyAdoptionSubject,
    evidence: PolicyAdoptionVerificationEvidence,
    now_ms: u64,
) -> Result<QualifiedPolicyAdoptionEvidence, PolicyEvidenceError> {
    expected.validate()?;
    evidence.validate_at(now_ms)?;
    if evidence.subject != expected {
        return Err(PolicyEvidenceError::AdoptionSubjectMismatch);
    }
    Ok(QualifiedPolicyAdoptionEvidence {
        subject: expected,
        evidence,
    })
}

/// Join independently qualified record and adoption evidence only when both
/// bind the same canonical policy identity and use distinct verifier identities.
pub fn join_policy_evidence(
    record: QualifiedPolicyRecordEvidence,
    adoption: QualifiedPolicyAdoptionEvidence,
    now_ms: u64,
) -> Result<QualifiedPolicyEvidenceBundle, PolicyEvidenceError> {
    if record.subject.policy != adoption.subject.policy {
        return Err(PolicyEvidenceError::CrossEvidencePolicyMismatch);
    }
    if record.evidence.verifier_ref == adoption.evidence.verifier_ref {
        return Err(PolicyEvidenceError::VerifierDomainCollision);
    }
    let lease = record
        .evidence
        .lease
        .intersect(&adoption.evidence.lease, now_ms)?;
    Ok(QualifiedPolicyEvidenceBundle {
        record,
        adoption,
        lease,
    })
}

fn validate_ref(value: &str) -> Result<(), PolicyEvidenceError> {
    if value.trim().is_empty()
        || value.len() > MAX_REF_BYTES
        || value.bytes().any(|byte| byte.is_ascii_control())
    {
        Err(PolicyEvidenceError::InvalidReference)
    } else {
        Ok(())
    }
}

fn validate_profile(value: &str) -> Result<(), PolicyEvidenceError> {
    if value.trim().is_empty()
        || value.len() > MAX_PROFILE_BYTES
        || value.bytes().any(|byte| byte.is_ascii_control())
    {
        Err(PolicyEvidenceError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PolicyEvidenceError {
    WrongRecordEvidenceProtocol,
    WrongAdoptionEvidenceProtocol,
    ZeroPolicyDigest,
    InvalidProfile,
    InvalidReference,
    InvalidRulebook,
    RecordSubjectMismatch,
    AdoptionSubjectMismatch,
    CrossEvidencePolicyMismatch,
    VerifierDomainCollision,
    EvidenceLease(EvidenceLeaseError),
}

impl From<EvidenceLeaseError> for PolicyEvidenceError {
    fn from(value: EvidenceLeaseError) -> Self {
        Self::EvidenceLease(value)
    }
}

impl fmt::Display for PolicyEvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongRecordEvidenceProtocol => write!(f, "wrong policy-record evidence protocol"),
            Self::WrongAdoptionEvidenceProtocol => {
                write!(f, "wrong policy-adoption evidence protocol")
            }
            Self::ZeroPolicyDigest => write!(f, "policy identity digest must be non-zero"),
            Self::InvalidProfile => write!(f, "invalid policy evidence profile"),
            Self::InvalidReference => write!(f, "invalid policy evidence reference"),
            Self::InvalidRulebook => write!(f, "invalid institutional rulebook"),
            Self::RecordSubjectMismatch => {
                write!(f, "policy-record evidence belongs to another exact subject")
            }
            Self::AdoptionSubjectMismatch => {
                write!(f, "policy-adoption evidence belongs to another exact subject")
            }
            Self::CrossEvidencePolicyMismatch => {
                write!(f, "record and adoption evidence bind different policy identities")
            }
            Self::VerifierDomainCollision => {
                write!(f, "record and adoption evidence use the same verifier identity")
            }
            Self::EvidenceLease(error) => write!(f, "invalid policy evidence lease: {error}"),
        }
    }
}

impl std::error::Error for PolicyEvidenceError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn identity() -> PolicyIdentityRef {
        PolicyIdentityRef {
            digest: d(7),
            profile: "mycelix-test-policy-v1-blake3-framed".into(),
        }
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:test:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(8),
        }
    }

    fn record_subject() -> PolicyRecordSubject {
        PolicyRecordSubject {
            policy: identity(),
            policy_record_ref: "registry:policy:test:v1".into(),
        }
    }

    fn adoption_subject() -> PolicyAdoptionSubject {
        PolicyAdoptionSubject {
            policy: identity(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:test").unwrap()),
            rulebook: rulebook(),
            authority_ref: "authority:governance-decision:7".into(),
            adoption_proof_ref: "proof:policy-adoption:7".into(),
        }
    }

    fn record_evidence() -> PolicyRecordVerificationEvidence {
        PolicyRecordVerificationEvidence {
            protocol_version: POLICY_RECORD_EVIDENCE_PROTOCOL.into(),
            subject: record_subject(),
            record_proof_ref: "proof:record:7".into(),
            verifier_ref: "verifier:immutable-policy-record".into(),
            verification_ref: "verification:record:7".into(),
            lease: EvidenceLease::new(100, 900, 200).unwrap(),
        }
    }

    fn adoption_evidence() -> PolicyAdoptionVerificationEvidence {
        PolicyAdoptionVerificationEvidence {
            protocol_version: POLICY_ADOPTION_EVIDENCE_PROTOCOL.into(),
            subject: adoption_subject(),
            verifier_ref: "verifier:institutional-policy-adoption".into(),
            verification_ref: "verification:adoption:7".into(),
            lease: EvidenceLease::new(150, 800, 200).unwrap(),
        }
    }

    fn qualified_record() -> QualifiedPolicyRecordEvidence {
        qualify_policy_record_evidence(record_subject(), record_evidence(), 200).unwrap()
    }

    fn qualified_adoption() -> QualifiedPolicyAdoptionEvidence {
        qualify_policy_adoption_evidence(adoption_subject(), adoption_evidence(), 200).unwrap()
    }

    #[test]
    fn exact_independent_evidence_joins_without_authority_amplification() {
        let joined = join_policy_evidence(qualified_record(), qualified_adoption(), 200).unwrap();
        assert_eq!(joined.policy_identity(), &identity());
        assert_eq!(joined.lease().verified_at_ms, 150);
        assert_eq!(joined.lease().valid_until_ms, 800);
        assert!(!joined.generation_currentness_verified_here());
        assert!(!joined.verifier_origins_verified_here());
        assert!(!joined.grants_authority());
        assert!(!joined.grants_administrative_decision_authority());
        assert!(!joined.grants_review_authority());
        assert!(!joined.grants_execution_authority());
        assert!(!joined.grants_external_effect_authority());
    }

    #[test]
    fn record_subject_substitution_fails_closed() {
        let mut evidence = record_evidence();
        evidence.subject.policy_record_ref = "registry:policy:other".into();
        assert_eq!(
            qualify_policy_record_evidence(record_subject(), evidence, 200).unwrap_err(),
            PolicyEvidenceError::RecordSubjectMismatch
        );
    }

    #[test]
    fn adoption_context_substitution_fails_closed() {
        let mut evidence = adoption_evidence();
        evidence.subject.institution = InstitutionId::new("institution:other").unwrap();
        assert_eq!(
            qualify_policy_adoption_evidence(adoption_subject(), evidence, 200).unwrap_err(),
            PolicyEvidenceError::AdoptionSubjectMismatch
        );

        let mut evidence = adoption_evidence();
        evidence.subject.rulebook.digest = d(99);
        assert_eq!(
            qualify_policy_adoption_evidence(adoption_subject(), evidence, 200).unwrap_err(),
            PolicyEvidenceError::AdoptionSubjectMismatch
        );

        let mut evidence = adoption_evidence();
        evidence.subject.adoption_proof_ref = "proof:other".into();
        assert_eq!(
            qualify_policy_adoption_evidence(adoption_subject(), evidence, 200).unwrap_err(),
            PolicyEvidenceError::AdoptionSubjectMismatch
        );
    }

    #[test]
    fn policy_identity_substitution_fails_before_join() {
        let mut evidence = record_evidence();
        evidence.subject.policy.digest = d(55);
        assert_eq!(
            qualify_policy_record_evidence(record_subject(), evidence, 200).unwrap_err(),
            PolicyEvidenceError::RecordSubjectMismatch
        );
    }

    #[test]
    fn independently_qualified_different_policies_cannot_join() {
        let record = qualified_record();
        let mut subject = adoption_subject();
        subject.policy.digest = d(44);
        let mut evidence = adoption_evidence();
        evidence.subject = subject.clone();
        let adoption = qualify_policy_adoption_evidence(subject, evidence, 200).unwrap();
        assert_eq!(
            join_policy_evidence(record, adoption, 200).unwrap_err(),
            PolicyEvidenceError::CrossEvidencePolicyMismatch
        );
    }

    #[test]
    fn one_verifier_identity_cannot_fill_both_roles() {
        let record = qualified_record();
        let mut evidence = adoption_evidence();
        evidence.verifier_ref = record.evidence().verifier_ref.clone();
        let adoption =
            qualify_policy_adoption_evidence(adoption_subject(), evidence, 200).unwrap();
        assert_eq!(
            join_policy_evidence(record, adoption, 200).unwrap_err(),
            PolicyEvidenceError::VerifierDomainCollision
        );
    }

    #[test]
    fn stale_or_future_dynamic_evidence_fails_through_shared_lease_theorem() {
        let mut stale = record_evidence();
        stale.lease = EvidenceLease {
            protocol_version: mycelix_authority_evidence_lease::PROTOCOL_VERSION.into(),
            verified_at_ms: 100,
            valid_until_ms: 150,
        };
        assert_eq!(
            qualify_policy_record_evidence(record_subject(), stale, 200).unwrap_err(),
            PolicyEvidenceError::EvidenceLease(EvidenceLeaseError::ExpiredLease)
        );

        let mut future = adoption_evidence();
        future.lease = EvidenceLease {
            protocol_version: mycelix_authority_evidence_lease::PROTOCOL_VERSION.into(),
            verified_at_ms: 250,
            valid_until_ms: 300,
        };
        assert_eq!(
            qualify_policy_adoption_evidence(adoption_subject(), future, 200).unwrap_err(),
            PolicyEvidenceError::EvidenceLease(EvidenceLeaseError::VerificationFromFuture)
        );
    }

    #[test]
    fn deserialized_transport_evidence_remains_non_authoritative() {
        let wire = serde_json::to_string(&record_evidence()).unwrap();
        let decoded: PolicyRecordVerificationEvidence = serde_json::from_str(&wire).unwrap();
        assert!(!decoded.grants_authority());
        assert_eq!(decoded.subject, record_subject());
    }

    #[test]
    fn malformed_identity_and_institutional_context_fail_closed() {
        let zero = PolicyIdentityRef {
            digest: Digest32([0; 32]),
            profile: "profile:v1".into(),
        };
        assert_eq!(zero.validate().unwrap_err(), PolicyEvidenceError::ZeroPolicyDigest);

        let malformed = PolicyIdentityRef {
            digest: d(1),
            profile: "\n".into(),
        };
        assert_eq!(malformed.validate().unwrap_err(), PolicyEvidenceError::InvalidProfile);

        // Keep the institutional protocol constant live in this test corpus so
        // dependency drift cannot silently remove the typed institutional base.
        assert!(!INSTITUTIONAL_PROTOCOL.is_empty());
    }
}
