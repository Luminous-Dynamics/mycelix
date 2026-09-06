// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Split proof qualification for one exact authority-state transition.
//!
//! An authentic immutable record and an institutionally authorized state change
//! are distinct facts. This pure kernel requires independent receipts for both,
//! plus an authoritative source identity supplied by a different runtime boundary,
//! before projecting the legacy `VerifiedAuthorityStateTransition` ABI.

use mycelix_authority_state_source::{
    AuthorityStateTransition, VerifiedAuthorityStateTransition, TRANSITION_IDENTITY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-transition-verifier-v0.1";
pub const RECORD_PROOF_PROTOCOL: &str =
    "mycelix-authority-state-transition-record-proof-v0.1";
pub const AUTHORITY_PROOF_PROTOCOL: &str =
    "mycelix-authority-state-transition-authority-proof-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-state-transition-verification-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-authority-state-transition-verification-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority-state/transition-verification/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority-state/transition-verification-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Evidence-shaped result from the immutable transition-record proof verifier.
/// Deserialization is transport only; authority is created only by the pure kernel.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedTransitionRecordProof {
    pub protocol_version: String,
    pub transition_digest: Digest32,
    pub transition_profile: String,
    pub transition_record_ref: String,
    pub verified_record_proof_ref: String,
    pub record_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedTransitionRecordProof {
    fn validate_against(
        &self,
        transition: &AuthorityStateTransition,
        transition_digest: Digest32,
        now_ms: u64,
    ) -> Result<(), TransitionVerificationError> {
        if self.protocol_version != RECORD_PROOF_PROTOCOL {
            return Err(TransitionVerificationError::WrongRecordProofProtocol);
        }
        if self.transition_profile != TRANSITION_IDENTITY_PROFILE
            || self.transition_digest.is_zero()
            || self.transition_digest != transition_digest
        {
            return Err(TransitionVerificationError::RecordTransitionIdentityMismatch);
        }
        require_profile(&self.transition_profile)?;
        for value in [
            self.transition_record_ref.as_str(),
            self.verified_record_proof_ref.as_str(),
            self.record_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.transition_record_ref != transition.status_record_ref {
            return Err(TransitionVerificationError::TransitionRecordRefMismatch);
        }
        if self.verified_record_proof_ref != transition.record_proof_ref {
            return Err(TransitionVerificationError::RecordProofRefMismatch);
        }
        validate_window(
            transition.effective_at_ms,
            self.verified_at_ms,
            self.valid_until_ms,
            now_ms,
        )
        .map_err(|_| TransitionVerificationError::InvalidRecordProofWindow)
    }
}

/// Evidence-shaped result from the institutional/rulebook authority verifier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedTransitionAuthorityProof {
    pub protocol_version: String,
    pub transition_digest: Digest32,
    pub transition_profile: String,
    pub verified_authority_ref: String,
    pub verified_authority_proof_ref: String,
    pub authority_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedTransitionAuthorityProof {
    fn validate_against(
        &self,
        transition: &AuthorityStateTransition,
        transition_digest: Digest32,
        now_ms: u64,
    ) -> Result<(), TransitionVerificationError> {
        if self.protocol_version != AUTHORITY_PROOF_PROTOCOL {
            return Err(TransitionVerificationError::WrongAuthorityProofProtocol);
        }
        if self.transition_profile != TRANSITION_IDENTITY_PROFILE
            || self.transition_digest.is_zero()
            || self.transition_digest != transition_digest
        {
            return Err(TransitionVerificationError::AuthorityTransitionIdentityMismatch);
        }
        require_profile(&self.transition_profile)?;
        for value in [
            self.verified_authority_ref.as_str(),
            self.verified_authority_proof_ref.as_str(),
            self.authority_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.verified_authority_ref != transition.authority_ref {
            return Err(TransitionVerificationError::TransitionAuthorityRefMismatch);
        }
        if self.verified_authority_proof_ref != transition.authority_proof_ref {
            return Err(TransitionVerificationError::AuthorityProofRefMismatch);
        }
        validate_window(
            transition.effective_at_ms,
            self.verified_at_ms,
            self.valid_until_ms,
            now_ms,
        )
        .map_err(|_| TransitionVerificationError::InvalidAuthorityProofWindow)
    }
}

/// Non-deserializable proof that one exact transition has both an authentic
/// immutable record and exact institutional authorization under one independently
/// supplied authoritative source identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedAuthorityStateTransitionVerification {
    verified_transition: VerifiedAuthorityStateTransition,
    transition_digest: Digest32,
    transition_profile: String,
    qualification_digest: Digest32,
    qualification_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedAuthorityStateTransitionVerification {
    pub fn transition(&self) -> &AuthorityStateTransition {
        &self.verified_transition.transition
    }

    pub fn transition_digest(&self) -> Digest32 {
        self.transition_digest
    }

    pub fn transition_profile(&self) -> &str {
        &self.transition_profile
    }

    pub fn authoritative_source_ref(&self) -> &str {
        &self.verified_transition.authoritative_source_ref
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    /// Compatibility projection consumed by the existing #91 source projector.
    pub fn to_verified_transition(&self) -> VerifiedAuthorityStateTransition {
        self.verified_transition.clone()
    }
}

/// Qualify one transition from two independently verified proof domains.
pub fn qualify_authority_state_transition(
    transition: &AuthorityStateTransition,
    record_proof: &VerifiedTransitionRecordProof,
    authority_proof: &VerifiedTransitionAuthorityProof,
    authoritative_source_ref: &str,
    now_ms: u64,
) -> Result<QualifiedAuthorityStateTransitionVerification, TransitionVerificationError> {
    if now_ms == 0 {
        return Err(TransitionVerificationError::InvalidVerificationTime);
    }
    require_ref(authoritative_source_ref)?;
    transition
        .validate()
        .map_err(|_| TransitionVerificationError::InvalidTransition)?;
    let transition_digest = transition
        .identity_digest()
        .map_err(|_| TransitionVerificationError::InvalidTransition)?;

    record_proof.validate_against(transition, transition_digest, now_ms)?;
    authority_proof.validate_against(transition, transition_digest, now_ms)?;

    let qualification_digest = qualification_digest(transition_digest, authoritative_source_ref);
    let evidence_digest = evidence_digest(
        qualification_digest,
        record_proof,
        authority_proof,
    );
    let verified_at_ms = record_proof
        .verified_at_ms
        .max(authority_proof.verified_at_ms);
    let valid_until_ms = record_proof
        .valid_until_ms
        .min(authority_proof.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(TransitionVerificationError::VerificationExpired);
    }

    let verified_transition = VerifiedAuthorityStateTransition {
        transition: transition.clone(),
        transition_record_ref: transition.status_record_ref.clone(),
        verified_record_proof_ref: transition.record_proof_ref.clone(),
        verified_authority_ref: transition.authority_ref.clone(),
        verified_authority_proof_ref: transition.authority_proof_ref.clone(),
        authoritative_source_ref: authoritative_source_ref.into(),
        verification_ref: format!(
            "authority-state-transition-evidence:{EVIDENCE_PROFILE}:{}",
            digest_hex(evidence_digest)
        ),
        verified_at_ms,
        lease_until_ms: valid_until_ms,
    };
    verified_transition
        .validate_at(now_ms)
        .map_err(|_| TransitionVerificationError::ProjectedTransitionInvalid)?;

    Ok(QualifiedAuthorityStateTransitionVerification {
        verified_transition,
        transition_digest,
        transition_profile: TRANSITION_IDENTITY_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn qualification_digest(
    transition_digest: Digest32,
    authoritative_source_ref: &str,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, TRANSITION_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &transition_digest.0);
    frame(&mut hasher, authoritative_source_ref.as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    record: &VerifiedTransitionRecordProof,
    authority: &VerifiedTransitionAuthorityProof,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, record.record_verifier_ref.as_bytes());
    frame(&mut hasher, record.verification_ref.as_bytes());
    frame(&mut hasher, &record.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &record.valid_until_ms.to_le_bytes());
    frame(&mut hasher, authority.authority_verifier_ref.as_bytes());
    frame(&mut hasher, authority.verification_ref.as_bytes());
    frame(&mut hasher, &authority.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &authority.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn validate_window(
    effective_at_ms: u64,
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), ()> {
    if effective_at_ms == 0
        || verified_at_ms < effective_at_ms
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms < verified_at_ms
    {
        Err(())
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), TransitionVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(TransitionVerificationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), TransitionVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_PROFILE_BYTES {
        Err(TransitionVerificationError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransitionVerificationError {
    InvalidTransition,
    WrongRecordProofProtocol,
    WrongAuthorityProofProtocol,
    RecordTransitionIdentityMismatch,
    AuthorityTransitionIdentityMismatch,
    TransitionRecordRefMismatch,
    RecordProofRefMismatch,
    TransitionAuthorityRefMismatch,
    AuthorityProofRefMismatch,
    InvalidRecordProofWindow,
    InvalidAuthorityProofWindow,
    InvalidReference,
    InvalidProfile,
    InvalidVerificationTime,
    VerificationExpired,
    ProjectedTransitionInvalid,
}

impl fmt::Display for TransitionVerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidTransition => "invalid authority-state transition",
            Self::WrongRecordProofProtocol => "wrong transition record-proof protocol",
            Self::WrongAuthorityProofProtocol => "wrong transition authority-proof protocol",
            Self::RecordTransitionIdentityMismatch => "record proof binds another transition",
            Self::AuthorityTransitionIdentityMismatch => "authority proof binds another transition",
            Self::TransitionRecordRefMismatch => "record proof binds another transition record",
            Self::RecordProofRefMismatch => "record proof reference mismatch",
            Self::TransitionAuthorityRefMismatch => "institutional authority reference mismatch",
            Self::AuthorityProofRefMismatch => "institutional authority proof reference mismatch",
            Self::InvalidRecordProofWindow => "transition record-proof window is invalid or stale",
            Self::InvalidAuthorityProofWindow => "transition authority-proof window is invalid or stale",
            Self::InvalidReference => "invalid proof/source reference",
            Self::InvalidProfile => "invalid digest profile",
            Self::InvalidVerificationTime => "qualification time must be positive",
            Self::VerificationExpired => "combined transition verification is stale",
            Self::ProjectedTransitionInvalid => "projected compatibility transition receipt is invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for TransitionVerificationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessState, AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest,
    };
    use mycelix_authority_state_source::{
        AuthorityStateTransitionKind, PROTOCOL_VERSION as SOURCE_PROTOCOL_VERSION,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn transition() -> AuthorityStateTransition {
        AuthorityStateTransition {
            protocol_version: SOURCE_PROTOCOL_VERSION.into(),
            transition_id: "transition:grant:1".into(),
            subject: AuthoritySubjectRef {
                kind: AuthoritySubjectKind::AuthorityGrant,
                namespace: "institution:test:authority".into(),
                subject_id: "grant:1".into(),
                identity: ProfiledDigest {
                    digest: d(1),
                    profile: "grant-semantic-v1".into(),
                },
            },
            generation: 1,
            kind: AuthorityStateTransitionKind::Establish,
            state: AuthorityFreshnessState::Active,
            previous_generation: None,
            previous_transition_digest: None,
            effective_at_ms: 100,
            status_record_ref: "record:transition:1".into(),
            reason_digest: d(2),
            authority_ref: "institution:authority:root".into(),
            authority_proof_ref: "proof:institution:transition:1".into(),
            record_proof_ref: "proof:record:transition:1".into(),
        }
    }

    fn record_proof(t: &AuthorityStateTransition, verified_at: u64) -> VerifiedTransitionRecordProof {
        VerifiedTransitionRecordProof {
            protocol_version: RECORD_PROOF_PROTOCOL.into(),
            transition_digest: t.identity_digest().unwrap(),
            transition_profile: TRANSITION_IDENTITY_PROFILE.into(),
            transition_record_ref: t.status_record_ref.clone(),
            verified_record_proof_ref: t.record_proof_ref.clone(),
            record_verifier_ref: "verifier:record:1".into(),
            verification_ref: format!("verification:record:{verified_at}"),
            verified_at_ms: verified_at,
            valid_until_ms: 500,
        }
    }

    fn authority_proof(
        t: &AuthorityStateTransition,
        verified_at: u64,
    ) -> VerifiedTransitionAuthorityProof {
        VerifiedTransitionAuthorityProof {
            protocol_version: AUTHORITY_PROOF_PROTOCOL.into(),
            transition_digest: t.identity_digest().unwrap(),
            transition_profile: TRANSITION_IDENTITY_PROFILE.into(),
            verified_authority_ref: t.authority_ref.clone(),
            verified_authority_proof_ref: t.authority_proof_ref.clone(),
            authority_verifier_ref: "verifier:institutional-authority:1".into(),
            verification_ref: format!("verification:authority:{verified_at}"),
            verified_at_ms: verified_at,
            valid_until_ms: 480,
        }
    }

    #[test]
    fn exact_split_proofs_qualify() {
        let t = transition();
        let q = qualify_authority_state_transition(
            &t,
            &record_proof(&t, 120),
            &authority_proof(&t, 130),
            "source:authority-state:primary",
            150,
        )
        .unwrap();
        let projected = q.to_verified_transition();
        assert_eq!(projected.transition, t);
        assert_eq!(projected.verified_at_ms, 130);
        assert_eq!(projected.lease_until_ms, 480);
        assert_eq!(projected.authoritative_source_ref, "source:authority-state:primary");
        projected.validate_at(150).unwrap();
    }

    #[test]
    fn record_proof_for_another_transition_denies() {
        let t = transition();
        let mut record = record_proof(&t, 120);
        record.transition_digest = d(9);
        assert_eq!(
            qualify_authority_state_transition(
                &t,
                &record,
                &authority_proof(&t, 130),
                "source:authority-state:primary",
                150,
            ),
            Err(TransitionVerificationError::RecordTransitionIdentityMismatch)
        );
    }

    #[test]
    fn wrong_institutional_authority_denies() {
        let t = transition();
        let mut authority = authority_proof(&t, 130);
        authority.verified_authority_ref = "institution:authority:other".into();
        assert_eq!(
            qualify_authority_state_transition(
                &t,
                &record_proof(&t, 120),
                &authority,
                "source:authority-state:primary",
                150,
            ),
            Err(TransitionVerificationError::TransitionAuthorityRefMismatch)
        );
    }

    #[test]
    fn authoritative_source_changes_stable_qualification() {
        let t = transition();
        let record = record_proof(&t, 120);
        let authority = authority_proof(&t, 130);
        let a = qualify_authority_state_transition(&t, &record, &authority, "source:a", 150).unwrap();
        let b = qualify_authority_state_transition(&t, &record, &authority, "source:b", 150).unwrap();
        assert_ne!(a.qualification_digest(), b.qualification_digest());
    }

    #[test]
    fn refreshed_verification_changes_evidence_not_stable_qualification() {
        let t = transition();
        let first = qualify_authority_state_transition(
            &t,
            &record_proof(&t, 120),
            &authority_proof(&t, 130),
            "source:authority-state:primary",
            150,
        )
        .unwrap();
        let second = qualify_authority_state_transition(
            &t,
            &record_proof(&t, 140),
            &authority_proof(&t, 145),
            "source:authority-state:primary",
            150,
        )
        .unwrap();
        assert_eq!(first.qualification_digest(), second.qualification_digest());
        assert_ne!(first.evidence_digest(), second.evidence_digest());
    }
}
