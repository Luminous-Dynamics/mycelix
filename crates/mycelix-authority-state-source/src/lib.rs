// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only authority-generation source and deterministic projection.
//!
//! `mycelix-authority-freshness` deliberately does not choose which generation is
//! current. This crate supplies the missing pure source/projection semantics.
//! It never selects by DHT arrival order or newest timestamp. Instead, it
//! reconstructs one exact contiguous generation chain from independently verified
//! append-only transition records and requires an independently verified source
//! coverage receipt proving that the supplied endpoint is the authoritative head.

use mycelix_authority_freshness::{
    AuthorityFreshnessSnapshot, AuthorityFreshnessState, AuthoritySubjectRef,
    VerifiedAuthorityFreshness, PROTOCOL_VERSION as FRESHNESS_PROTOCOL_VERSION,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-source-v0.1";
pub const TRANSITION_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-transition-v1-blake3-framed";
pub const LINEAGE_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-lineage-v1-blake3-framed";
pub const PROJECTION_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-projection-v1-blake3-framed";

const DOMAIN_TRANSITION: &[u8] = b"mycelix/authority-state/transition/v1";
const DOMAIN_LINEAGE: &[u8] = b"mycelix/authority-state/lineage/v1";
const DOMAIN_PROJECTION: &[u8] = b"mycelix/authority-state/projection/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_TRANSITIONS: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorityStateTransitionKind {
    /// Establish generation 1 as Active.
    Establish,
    /// Active -> Revoked.
    Revoke,
    /// Active -> Superseded. Superseded is terminal in v0.1.
    Supersede,
    /// Revoked -> Active under a new explicitly authorized generation.
    Reactivate,
}

impl AuthorityStateTransitionKind {
    fn code(self) -> u8 {
        match self {
            Self::Establish => 1,
            Self::Revoke => 2,
            Self::Supersede => 3,
            Self::Reactivate => 4,
        }
    }
}

/// Immutable candidate state transition for one exact authority subject.
///
/// This object is not authoritative merely because it exists. A host/source
/// boundary must independently verify both the record proof and the institutional
/// authority proof, producing `VerifiedAuthorityStateTransition`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityStateTransition {
    pub protocol_version: String,
    pub transition_id: String,
    pub subject: AuthoritySubjectRef,
    pub generation: u64,
    pub kind: AuthorityStateTransitionKind,
    pub state: AuthorityFreshnessState,
    /// Previous generation for non-root transitions.
    pub previous_generation: Option<u64>,
    /// Exact identity digest of the previous transition for non-root transitions.
    pub previous_transition_digest: Option<Digest32>,
    pub effective_at_ms: u64,
    /// Immutable record/content identity that becomes the freshness status ref.
    pub status_record_ref: String,
    /// Non-zero commitment to the governed reason/cause for this state change.
    pub reason_digest: Digest32,
    /// Exact institutional/rulebook authority that authorized this transition.
    pub authority_ref: String,
    /// Proof for the exact institutional authority decision above.
    pub authority_proof_ref: String,
    /// Proof binding the stored transition record itself.
    pub record_proof_ref: String,
}

impl AuthorityStateTransition {
    pub fn validate(&self) -> Result<(), AuthorityStateSourceError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AuthorityStateSourceError::WrongProtocolVersion);
        }
        self.subject
            .validate()
            .map_err(|_| AuthorityStateSourceError::InvalidSubject)?;
        require_ref(&self.transition_id)?;
        require_ref(&self.status_record_ref)?;
        require_ref(&self.authority_ref)?;
        require_ref(&self.authority_proof_ref)?;
        require_ref(&self.record_proof_ref)?;
        if self.generation == 0 {
            return Err(AuthorityStateSourceError::ZeroGeneration);
        }
        if self.generation > MAX_TRANSITIONS as u64 {
            return Err(AuthorityStateSourceError::GenerationOutOfRange);
        }
        if self.effective_at_ms == 0 {
            return Err(AuthorityStateSourceError::InvalidEffectiveTime);
        }
        if self.reason_digest.is_zero() {
            return Err(AuthorityStateSourceError::ZeroReasonDigest);
        }

        match self.kind {
            AuthorityStateTransitionKind::Establish => {
                if self.generation != 1
                    || self.state != AuthorityFreshnessState::Active
                    || self.previous_generation.is_some()
                    || self.previous_transition_digest.is_some()
                {
                    return Err(AuthorityStateSourceError::InvalidRootTransition);
                }
            }
            AuthorityStateTransitionKind::Revoke => {
                self.validate_non_root()?;
                if self.state != AuthorityFreshnessState::Revoked {
                    return Err(AuthorityStateSourceError::TransitionStateMismatch);
                }
            }
            AuthorityStateTransitionKind::Supersede => {
                self.validate_non_root()?;
                if self.state != AuthorityFreshnessState::Superseded {
                    return Err(AuthorityStateSourceError::TransitionStateMismatch);
                }
            }
            AuthorityStateTransitionKind::Reactivate => {
                self.validate_non_root()?;
                if self.state != AuthorityFreshnessState::Active {
                    return Err(AuthorityStateSourceError::TransitionStateMismatch);
                }
            }
        }
        Ok(())
    }

    fn validate_non_root(&self) -> Result<(), AuthorityStateSourceError> {
        let Some(previous_generation) = self.previous_generation else {
            return Err(AuthorityStateSourceError::MissingParent);
        };
        let Some(previous_digest) = self.previous_transition_digest else {
            return Err(AuthorityStateSourceError::MissingParent);
        };
        if previous_generation == 0
            || previous_generation.checked_add(1) != Some(self.generation)
        {
            return Err(AuthorityStateSourceError::GenerationDiscontinuity);
        }
        if previous_digest.is_zero() {
            return Err(AuthorityStateSourceError::ZeroParentDigest);
        }
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, AuthorityStateSourceError> {
        self.validate()?;
        let subject_digest = self
            .subject
            .identity_digest()
            .map_err(|_| AuthorityStateSourceError::InvalidSubject)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_TRANSITION);
        frame(&mut hasher, TRANSITION_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.transition_id.as_bytes());
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, &self.generation.to_le_bytes());
        frame(&mut hasher, &[self.kind.code()]);
        frame(&mut hasher, &[state_code(self.state)]);
        frame_optional_u64(&mut hasher, self.previous_generation);
        frame_optional_digest(&mut hasher, self.previous_transition_digest);
        frame(&mut hasher, &self.effective_at_ms.to_le_bytes());
        frame(&mut hasher, self.status_record_ref.as_bytes());
        frame(&mut hasher, &self.reason_digest.0);
        frame(&mut hasher, self.authority_ref.as_bytes());
        frame(&mut hasher, self.authority_proof_ref.as_bytes());
        frame(&mut hasher, self.record_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Host-attested verification of one immutable transition candidate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityStateTransition {
    pub transition: AuthorityStateTransition,
    /// Must equal `transition.status_record_ref`.
    pub transition_record_ref: String,
    pub verified_record_proof_ref: String,
    pub verified_authority_ref: String,
    pub verified_authority_proof_ref: String,
    /// Exact authoritative state-source/registry identity.
    pub authoritative_source_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    /// Reuse ceiling for this verification evidence.
    pub lease_until_ms: u64,
}

impl VerifiedAuthorityStateTransition {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), AuthorityStateSourceError> {
        self.transition.validate()?;
        for value in [
            self.transition_record_ref.as_str(),
            self.verified_record_proof_ref.as_str(),
            self.verified_authority_ref.as_str(),
            self.verified_authority_proof_ref.as_str(),
            self.authoritative_source_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.transition_record_ref != self.transition.status_record_ref {
            return Err(AuthorityStateSourceError::RecordRefMismatch);
        }
        if self.verified_record_proof_ref != self.transition.record_proof_ref {
            return Err(AuthorityStateSourceError::RecordProofMismatch);
        }
        if self.verified_authority_ref != self.transition.authority_ref {
            return Err(AuthorityStateSourceError::TransitionAuthorityMismatch);
        }
        if self.verified_authority_proof_ref != self.transition.authority_proof_ref {
            return Err(AuthorityStateSourceError::AuthorityProofMismatch);
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.verified_at_ms < self.transition.effective_at_ms
            || self.lease_until_ms <= now_ms
        {
            return Err(AuthorityStateSourceError::InvalidVerificationWindow);
        }
        Ok(())
    }
}

/// Independently verified proof that one logical authoritative source has been
/// completely observed through one exact transition head.
///
/// Without this receipt, a valid prefix (for example generations 1-2) could be
/// mistaken for current even when a generation-3 revocation exists. Coverage is
/// dynamic source evidence and is intentionally separate from transition identity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityStateCoverage {
    pub subject: AuthoritySubjectRef,
    pub authoritative_source_ref: String,
    pub head_generation: u64,
    pub head_transition_digest: Digest32,
    pub head_status_record_ref: String,
    pub coverage_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub lease_until_ms: u64,
}

impl VerifiedAuthorityStateCoverage {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), AuthorityStateSourceError> {
        self.subject
            .validate()
            .map_err(|_| AuthorityStateSourceError::InvalidSubject)?;
        require_ref(&self.authoritative_source_ref)?;
        require_ref(&self.head_status_record_ref)?;
        require_ref(&self.coverage_proof_ref)?;
        require_ref(&self.verification_ref)?;
        if self.head_generation == 0 || self.head_generation > MAX_TRANSITIONS as u64 {
            return Err(AuthorityStateSourceError::GenerationOutOfRange);
        }
        if self.head_transition_digest.is_zero() {
            return Err(AuthorityStateSourceError::ZeroCoverageHeadDigest);
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.lease_until_ms <= now_ms
        {
            return Err(AuthorityStateSourceError::InvalidCoverageWindow);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ProjectionMode {
    Current,
    AsOf(u64),
}

/// Non-forgeable pure projection over one exact verified transition lineage and
/// one exact authoritative coverage/head proof.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedAuthorityStateProjection {
    subject: AuthoritySubjectRef,
    selected_snapshot: AuthorityFreshnessSnapshot,
    selected_transition_digest: Digest32,
    lineage_digest: Digest32,
    lineage_profile: String,
    projection_digest: Digest32,
    projection_profile: String,
    authoritative_source_ref: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
    historical_as_of_ms: Option<u64>,
}

impl QualifiedAuthorityStateProjection {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn snapshot(&self) -> &AuthorityFreshnessSnapshot {
        &self.selected_snapshot
    }

    pub fn selected_transition_digest(&self) -> Digest32 {
        self.selected_transition_digest
    }

    pub fn lineage_digest(&self) -> Digest32 {
        self.lineage_digest
    }

    pub fn lineage_profile(&self) -> &str {
        &self.lineage_profile
    }

    pub fn projection_digest(&self) -> Digest32 {
        self.projection_digest
    }

    pub fn projection_profile(&self) -> &str {
        &self.projection_profile
    }

    pub fn authoritative_source_ref(&self) -> &str {
        &self.authoritative_source_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }

    pub fn historical_as_of_ms(&self) -> Option<u64> {
        self.historical_as_of_ms
    }

    /// Project this exact *current* qualified source result into the ABI consumed
    /// by `mycelix-authority-freshness`.
    ///
    /// Historical/as-of results intentionally cannot be converted into current
    /// freshness receipts, preventing historical validity from becoming live
    /// execution authority.
    pub fn to_current_freshness_receipt(
        &self,
    ) -> Result<VerifiedAuthorityFreshness, AuthorityStateSourceError> {
        if self.historical_as_of_ms.is_some() {
            return Err(AuthorityStateSourceError::HistoricalReceiptNotCurrentAuthority);
        }
        Ok(VerifiedAuthorityFreshness {
            snapshot: self.selected_snapshot.clone(),
            authoritative_source_ref: self.authoritative_source_ref.clone(),
            verification_ref: projection_ref(self.projection_digest),
            verified_at_ms: self.verified_at_ms,
            lease_until_ms: self.lease_until_ms,
        })
    }
}

/// Resolve the exact current endpoint of one complete verified transition chain.
///
/// `coverage` is mandatory: a valid transition prefix is not evidence that no
/// later authoritative transition exists.
pub fn project_current_authority_state(
    subject: &AuthoritySubjectRef,
    receipts: &[VerifiedAuthorityStateTransition],
    coverage: &VerifiedAuthorityStateCoverage,
    now_ms: u64,
) -> Result<QualifiedAuthorityStateProjection, AuthorityStateSourceError> {
    project(subject, receipts, coverage, ProjectionMode::Current, now_ms)
}

/// Resolve the exact state that was effective at `as_of_ms` after first proving
/// one complete, unambiguous transition lineage through the currently verified
/// authoritative source head.
///
/// This answers a historical audit question only. The returned projection cannot
/// be converted into a live `VerifiedAuthorityFreshness` receipt.
pub fn project_authority_state_as_of(
    subject: &AuthoritySubjectRef,
    receipts: &[VerifiedAuthorityStateTransition],
    coverage: &VerifiedAuthorityStateCoverage,
    as_of_ms: u64,
    verification_now_ms: u64,
) -> Result<QualifiedAuthorityStateProjection, AuthorityStateSourceError> {
    if as_of_ms == 0 || as_of_ms > verification_now_ms {
        return Err(AuthorityStateSourceError::InvalidHistoricalTime);
    }
    project(
        subject,
        receipts,
        coverage,
        ProjectionMode::AsOf(as_of_ms),
        verification_now_ms,
    )
}

fn project(
    subject: &AuthoritySubjectRef,
    receipts: &[VerifiedAuthorityStateTransition],
    coverage: &VerifiedAuthorityStateCoverage,
    mode: ProjectionMode,
    verification_now_ms: u64,
) -> Result<QualifiedAuthorityStateProjection, AuthorityStateSourceError> {
    subject
        .validate()
        .map_err(|_| AuthorityStateSourceError::InvalidSubject)?;
    coverage.validate_at(verification_now_ms)?;
    if &coverage.subject != subject {
        return Err(AuthorityStateSourceError::CoverageSubjectMismatch);
    }
    if verification_now_ms == 0 {
        return Err(AuthorityStateSourceError::InvalidVerificationWindow);
    }
    if receipts.is_empty() {
        return Err(AuthorityStateSourceError::MissingRoot);
    }
    if receipts.len() > MAX_TRANSITIONS {
        return Err(AuthorityStateSourceError::TooManyTransitions);
    }

    let mut by_generation = BTreeMap::<u64, NormalizedTransition>::new();
    let authoritative_source_ref = coverage.authoritative_source_ref.clone();
    let mut verified_at_ms = coverage.verified_at_ms;
    let mut lease_until_ms = coverage.lease_until_ms;

    for receipt in receipts {
        receipt.validate_at(verification_now_ms)?;
        if &receipt.transition.subject != subject {
            return Err(AuthorityStateSourceError::SubjectMismatch);
        }
        if receipt.authoritative_source_ref != authoritative_source_ref {
            return Err(AuthorityStateSourceError::AuthoritativeSourceMismatch);
        }

        let digest = receipt.transition.identity_digest()?;
        let normalized = NormalizedTransition {
            transition: receipt.transition.clone(),
            digest,
        };
        match by_generation.get(&receipt.transition.generation) {
            Some(existing) if existing != &normalized => {
                return Err(AuthorityStateSourceError::GenerationFork);
            }
            Some(_) => {
                // Exact duplicate stable transition evidence is harmless. Dynamic
                // verification metadata may only shrink the aggregate lease.
            }
            None => {
                by_generation.insert(receipt.transition.generation, normalized);
            }
        }
        verified_at_ms = verified_at_ms.max(receipt.verified_at_ms);
        lease_until_ms = lease_until_ms.min(receipt.lease_until_ms);
    }

    let root = by_generation
        .get(&1)
        .ok_or(AuthorityStateSourceError::MissingRoot)?;
    if root.transition.kind != AuthorityStateTransitionKind::Establish {
        return Err(AuthorityStateSourceError::InvalidRootTransition);
    }

    let max_generation = *by_generation
        .keys()
        .next_back()
        .ok_or(AuthorityStateSourceError::MissingRoot)?;
    if max_generation != by_generation.len() as u64 {
        return Err(AuthorityStateSourceError::GenerationDiscontinuity);
    }

    let mut previous: Option<&NormalizedTransition> = None;
    let mut ordered = Vec::<&NormalizedTransition>::with_capacity(by_generation.len());
    for generation in 1..=max_generation {
        let current = by_generation
            .get(&generation)
            .ok_or(AuthorityStateSourceError::GenerationDiscontinuity)?;
        match previous {
            None => {
                if generation != 1 {
                    return Err(AuthorityStateSourceError::MissingRoot);
                }
            }
            Some(parent) => validate_parent_child(parent, current)?,
        }
        ordered.push(current);
        previous = Some(current);
    }

    let endpoint = ordered
        .last()
        .copied()
        .ok_or(AuthorityStateSourceError::MissingRoot)?;
    if coverage.head_generation != endpoint.transition.generation
        || coverage.head_transition_digest != endpoint.digest
        || coverage.head_status_record_ref != endpoint.transition.status_record_ref
    {
        return Err(AuthorityStateSourceError::CoverageHeadMismatch);
    }
    if coverage.verified_at_ms < endpoint.transition.effective_at_ms {
        return Err(AuthorityStateSourceError::InvalidCoverageWindow);
    }

    let selected = match mode {
        ProjectionMode::Current => {
            if endpoint.transition.effective_at_ms > verification_now_ms {
                return Err(AuthorityStateSourceError::CurrentTransitionInFuture);
            }
            endpoint
        }
        ProjectionMode::AsOf(as_of_ms) => ordered
            .iter()
            .copied()
            .take_while(|item| item.transition.effective_at_ms <= as_of_ms)
            .last()
            .ok_or(AuthorityStateSourceError::NoStateAtHistoricalTime)?,
    };

    let lineage_digest = lineage_digest(subject, &ordered);
    let historical_as_of_ms = match mode {
        ProjectionMode::Current => None,
        ProjectionMode::AsOf(value) => Some(value),
    };
    let snapshot = AuthorityFreshnessSnapshot {
        protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
        subject: subject.clone(),
        generation: selected.transition.generation,
        state: selected.transition.state,
        effective_at_ms: selected.transition.effective_at_ms,
        status_record_ref: selected.transition.status_record_ref.clone(),
    };
    snapshot
        .validate()
        .map_err(|_| AuthorityStateSourceError::InvalidProjectedSnapshot)?;
    let snapshot_digest = snapshot
        .identity_digest()
        .map_err(|_| AuthorityStateSourceError::InvalidProjectedSnapshot)?;
    let projection_digest = projection_digest(
        subject,
        lineage_digest,
        snapshot_digest,
        historical_as_of_ms,
    );

    if lease_until_ms <= verification_now_ms || verified_at_ms > verification_now_ms {
        return Err(AuthorityStateSourceError::InvalidVerificationWindow);
    }

    Ok(QualifiedAuthorityStateProjection {
        subject: subject.clone(),
        selected_snapshot: snapshot,
        selected_transition_digest: selected.digest,
        lineage_digest,
        lineage_profile: LINEAGE_IDENTITY_PROFILE.into(),
        projection_digest,
        projection_profile: PROJECTION_IDENTITY_PROFILE.into(),
        authoritative_source_ref,
        verified_at_ms,
        lease_until_ms,
        historical_as_of_ms,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct NormalizedTransition {
    transition: AuthorityStateTransition,
    digest: Digest32,
}

fn validate_parent_child(
    parent: &NormalizedTransition,
    child: &NormalizedTransition,
) -> Result<(), AuthorityStateSourceError> {
    if child.transition.generation != parent.transition.generation + 1
        || child.transition.previous_generation != Some(parent.transition.generation)
        || child.transition.previous_transition_digest != Some(parent.digest)
    {
        return Err(AuthorityStateSourceError::ParentMismatch);
    }
    if child.transition.effective_at_ms <= parent.transition.effective_at_ms {
        return Err(AuthorityStateSourceError::EffectiveTimeRegression);
    }

    match (parent.transition.state, child.transition.kind) {
        (AuthorityFreshnessState::Active, AuthorityStateTransitionKind::Revoke) => {}
        (AuthorityFreshnessState::Active, AuthorityStateTransitionKind::Supersede) => {}
        (AuthorityFreshnessState::Revoked, AuthorityStateTransitionKind::Reactivate) => {}
        (AuthorityFreshnessState::Superseded, _) => {
            return Err(AuthorityStateSourceError::TransitionAfterSuperseded);
        }
        _ => return Err(AuthorityStateSourceError::IllegalStateTransition),
    }
    Ok(())
}

fn lineage_digest(subject: &AuthoritySubjectRef, ordered: &[&NormalizedTransition]) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_LINEAGE);
    frame(&mut hasher, LINEAGE_IDENTITY_PROFILE.as_bytes());
    let subject_digest = subject
        .identity_digest()
        .expect("validated authority subject must remain valid");
    frame(&mut hasher, &subject_digest.0);
    frame(&mut hasher, &(ordered.len() as u64).to_le_bytes());
    for transition in ordered {
        frame(&mut hasher, &transition.digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn projection_digest(
    subject: &AuthoritySubjectRef,
    lineage_digest: Digest32,
    snapshot_digest: Digest32,
    historical_as_of_ms: Option<u64>,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_PROJECTION);
    frame(&mut hasher, PROJECTION_IDENTITY_PROFILE.as_bytes());
    let subject_digest = subject
        .identity_digest()
        .expect("validated authority subject must remain valid");
    frame(&mut hasher, &subject_digest.0);
    frame(&mut hasher, &lineage_digest.0);
    frame(&mut hasher, &snapshot_digest.0);
    match historical_as_of_ms {
        Some(value) => {
            frame(&mut hasher, &[1]);
            frame(&mut hasher, &value.to_le_bytes());
        }
        None => frame(&mut hasher, &[0]),
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn projection_ref(digest: Digest32) -> String {
    format!(
        "authority-state:{}:{}",
        PROJECTION_IDENTITY_PROFILE,
        hex_digest(digest)
    )
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn state_code(state: AuthorityFreshnessState) -> u8 {
    match state {
        AuthorityFreshnessState::Active => 1,
        AuthorityFreshnessState::Revoked => 2,
        AuthorityFreshnessState::Superseded => 3,
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn frame_optional_u64(hasher: &mut blake3::Hasher, value: Option<u64>) {
    match value {
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, &value.to_le_bytes());
        }
        None => frame(hasher, &[0]),
    }
}

fn frame_optional_digest(hasher: &mut blake3::Hasher, value: Option<Digest32>) {
    match value {
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, &value.0);
        }
        None => frame(hasher, &[0]),
    }
}

fn require_ref(value: &str) -> Result<(), AuthorityStateSourceError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(AuthorityStateSourceError::InvalidReference)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorityStateSourceError {
    WrongProtocolVersion,
    InvalidSubject,
    InvalidReference,
    ZeroGeneration,
    GenerationOutOfRange,
    InvalidEffectiveTime,
    ZeroReasonDigest,
    InvalidRootTransition,
    TransitionStateMismatch,
    MissingParent,
    ZeroParentDigest,
    GenerationDiscontinuity,
    RecordRefMismatch,
    RecordProofMismatch,
    TransitionAuthorityMismatch,
    AuthorityProofMismatch,
    InvalidVerificationWindow,
    InvalidCoverageWindow,
    ZeroCoverageHeadDigest,
    CoverageSubjectMismatch,
    CoverageHeadMismatch,
    TooManyTransitions,
    SubjectMismatch,
    AuthoritativeSourceMismatch,
    GenerationFork,
    MissingRoot,
    ParentMismatch,
    EffectiveTimeRegression,
    IllegalStateTransition,
    TransitionAfterSuperseded,
    CurrentTransitionInFuture,
    InvalidHistoricalTime,
    NoStateAtHistoricalTime,
    InvalidProjectedSnapshot,
    HistoricalReceiptNotCurrentAuthority,
}

impl fmt::Display for AuthorityStateSourceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong authority-state source protocol version",
            Self::InvalidSubject => "invalid authority-state subject",
            Self::InvalidReference => "invalid authority-state reference",
            Self::ZeroGeneration => "authority-state generation must be non-zero",
            Self::GenerationOutOfRange => "authority-state generation exceeds v0.1 bound",
            Self::InvalidEffectiveTime => "authority-state effective time must be non-zero",
            Self::ZeroReasonDigest => "authority-state reason digest must be non-zero",
            Self::InvalidRootTransition => "authority-state root must establish generation 1 Active",
            Self::TransitionStateMismatch => "authority-state transition kind/state mismatch",
            Self::MissingParent => "authority-state transition is missing its exact parent",
            Self::ZeroParentDigest => "authority-state parent digest must be non-zero",
            Self::GenerationDiscontinuity => "authority-state generations are not contiguous",
            Self::RecordRefMismatch => "verified transition record ref mismatch",
            Self::RecordProofMismatch => "verified transition record proof mismatch",
            Self::TransitionAuthorityMismatch => "verified transition authority ref mismatch",
            Self::AuthorityProofMismatch => "verified transition authority proof mismatch",
            Self::InvalidVerificationWindow => "invalid authority-state verification window",
            Self::InvalidCoverageWindow => "invalid authority-state source coverage window",
            Self::ZeroCoverageHeadDigest => "authority-state source coverage head digest is zero",
            Self::CoverageSubjectMismatch => "authority-state source coverage belongs to another subject",
            Self::CoverageHeadMismatch => "supplied transition endpoint does not match authoritative source head",
            Self::TooManyTransitions => "authority-state transition fan-in exceeds v0.1 bound",
            Self::SubjectMismatch => "authority-state transition belongs to another subject",
            Self::AuthoritativeSourceMismatch => "authority-state receipts use different authoritative sources",
            Self::GenerationFork => "multiple different transitions claim the same generation",
            Self::MissingRoot => "authority-state lineage has no generation-1 root",
            Self::ParentMismatch => "authority-state child does not bind the exact previous transition",
            Self::EffectiveTimeRegression => "authority-state effective time does not advance causally",
            Self::IllegalStateTransition => "authority-state transition is illegal from parent state",
            Self::TransitionAfterSuperseded => "superseded authority is terminal in v0.1",
            Self::CurrentTransitionInFuture => "current authority-state endpoint is not yet effective",
            Self::InvalidHistoricalTime => "invalid authority-state historical query time",
            Self::NoStateAtHistoricalTime => "authority subject had no established state at requested time",
            Self::InvalidProjectedSnapshot => "projected authority freshness snapshot is invalid",
            Self::HistoricalReceiptNotCurrentAuthority => "historical projection cannot become current authority",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for AuthorityStateSourceError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthoritySubjectKind, ProfiledDigest as FreshnessProfiledDigest,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test".into(),
            subject_id: "grant:test".into(),
            identity: FreshnessProfiledDigest {
                digest: d(1),
                profile: "grant-v1-blake3".into(),
            },
        }
    }

    fn transition(
        generation: u64,
        kind: AuthorityStateTransitionKind,
        state: AuthorityFreshnessState,
        previous: Option<&AuthorityStateTransition>,
        effective_at_ms: u64,
    ) -> AuthorityStateTransition {
        let previous_generation = previous.map(|value| value.generation);
        let previous_transition_digest = previous.map(|value| value.identity_digest().unwrap());
        AuthorityStateTransition {
            protocol_version: PROTOCOL_VERSION.into(),
            transition_id: format!("transition:{generation}"),
            subject: subject(),
            generation,
            kind,
            state,
            previous_generation,
            previous_transition_digest,
            effective_at_ms,
            status_record_ref: format!("status:{generation}"),
            reason_digest: d(generation as u8 + 10),
            authority_ref: format!("authority:{generation}"),
            authority_proof_ref: format!("authority-proof:{generation}"),
            record_proof_ref: format!("record-proof:{generation}"),
        }
    }

    fn verified(transition: AuthorityStateTransition) -> VerifiedAuthorityStateTransition {
        VerifiedAuthorityStateTransition {
            transition_record_ref: transition.status_record_ref.clone(),
            verified_record_proof_ref: transition.record_proof_ref.clone(),
            verified_authority_ref: transition.authority_ref.clone(),
            verified_authority_proof_ref: transition.authority_proof_ref.clone(),
            authoritative_source_ref: "authority-state-source:test".into(),
            verification_ref: format!("verification:{}", transition.generation),
            verified_at_ms: 1_000,
            lease_until_ms: 2_000,
            transition,
        }
    }

    fn lineage() -> Vec<VerifiedAuthorityStateTransition> {
        let t1 = transition(
            1,
            AuthorityStateTransitionKind::Establish,
            AuthorityFreshnessState::Active,
            None,
            100,
        );
        let t2 = transition(
            2,
            AuthorityStateTransitionKind::Revoke,
            AuthorityFreshnessState::Revoked,
            Some(&t1),
            200,
        );
        let t3 = transition(
            3,
            AuthorityStateTransitionKind::Reactivate,
            AuthorityFreshnessState::Active,
            Some(&t2),
            300,
        );
        vec![verified(t1), verified(t2), verified(t3)]
    }

    fn coverage(receipts: &[VerifiedAuthorityStateTransition]) -> VerifiedAuthorityStateCoverage {
        let head = receipts
            .iter()
            .max_by_key(|receipt| receipt.transition.generation)
            .unwrap();
        VerifiedAuthorityStateCoverage {
            subject: subject(),
            authoritative_source_ref: "authority-state-source:test".into(),
            head_generation: head.transition.generation,
            head_transition_digest: head.transition.identity_digest().unwrap(),
            head_status_record_ref: head.transition.status_record_ref.clone(),
            coverage_proof_ref: "coverage-proof:test".into(),
            verification_ref: "coverage-verification:test".into(),
            verified_at_ms: 950,
            lease_until_ms: 1_900,
        }
    }

    #[test]
    fn current_projection_uses_unique_covered_chain_endpoint() {
        let transitions = lineage();
        let projection = project_current_authority_state(
            &subject(),
            &transitions,
            &coverage(&transitions),
            1_000,
        )
        .unwrap();
        assert_eq!(projection.snapshot().generation, 3);
        assert_eq!(projection.snapshot().state, AuthorityFreshnessState::Active);
        assert!(projection.to_current_freshness_receipt().is_ok());
    }

    #[test]
    fn input_order_does_not_change_projection_identity() {
        let a = lineage();
        let mut b = a.clone();
        b.reverse();
        let coverage = coverage(&a);
        let left = project_current_authority_state(&subject(), &a, &coverage, 1_000).unwrap();
        let right = project_current_authority_state(&subject(), &b, &coverage, 1_000).unwrap();
        assert_eq!(left.projection_digest(), right.projection_digest());
        assert_eq!(left.lineage_digest(), right.lineage_digest());
    }

    #[test]
    fn historical_projection_preserves_pre_revocation_truth() {
        let transitions = lineage();
        let coverage = coverage(&transitions);
        let before_revoke = project_authority_state_as_of(
            &subject(),
            &transitions,
            &coverage,
            150,
            1_000,
        )
        .unwrap();
        let after_revoke = project_authority_state_as_of(
            &subject(),
            &transitions,
            &coverage,
            250,
            1_000,
        )
        .unwrap();
        assert_eq!(before_revoke.snapshot().state, AuthorityFreshnessState::Active);
        assert_eq!(after_revoke.snapshot().state, AuthorityFreshnessState::Revoked);
        assert_eq!(
            before_revoke.to_current_freshness_receipt().unwrap_err(),
            AuthorityStateSourceError::HistoricalReceiptNotCurrentAuthority
        );
    }

    #[test]
    fn omitted_later_generation_fails_coverage() {
        let full = lineage();
        let full_coverage = coverage(&full);
        let prefix = vec![full[0].clone(), full[1].clone()];
        assert_eq!(
            project_current_authority_state(&subject(), &prefix, &full_coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::CoverageHeadMismatch
        );
    }

    #[test]
    fn generation_fork_denies_instead_of_selecting_by_time() {
        let mut transitions = lineage();
        let base_coverage = coverage(&transitions);
        let t1 = transitions[0].transition.clone();
        let fork = transition(
            2,
            AuthorityStateTransitionKind::Supersede,
            AuthorityFreshnessState::Superseded,
            Some(&t1),
            999,
        );
        transitions.push(verified(fork));
        assert_eq!(
            project_current_authority_state(&subject(), &transitions, &base_coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::GenerationFork
        );
    }

    #[test]
    fn missing_generation_denies() {
        let transitions = lineage();
        let full_coverage = coverage(&transitions);
        let broken = vec![transitions[0].clone(), transitions[2].clone()];
        assert_eq!(
            project_current_authority_state(&subject(), &broken, &full_coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::GenerationDiscontinuity
        );
    }

    #[test]
    fn parent_digest_mismatch_denies() {
        let mut transitions = lineage();
        let coverage = coverage(&transitions);
        transitions[1].transition.previous_transition_digest = Some(d(99));
        assert_eq!(
            project_current_authority_state(&subject(), &transitions, &coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::ParentMismatch
        );
    }

    #[test]
    fn effective_time_regression_denies() {
        let mut transitions = lineage();
        let coverage = coverage(&transitions);
        transitions[1].transition.effective_at_ms = 50;
        transitions[1].verified_at_ms = 1_000;
        assert_eq!(
            project_current_authority_state(&subject(), &transitions, &coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::EffectiveTimeRegression
        );
    }

    #[test]
    fn superseded_is_terminal() {
        let t1 = transition(
            1,
            AuthorityStateTransitionKind::Establish,
            AuthorityFreshnessState::Active,
            None,
            100,
        );
        let t2 = transition(
            2,
            AuthorityStateTransitionKind::Supersede,
            AuthorityFreshnessState::Superseded,
            Some(&t1),
            200,
        );
        let t3 = transition(
            3,
            AuthorityStateTransitionKind::Reactivate,
            AuthorityFreshnessState::Active,
            Some(&t2),
            300,
        );
        let receipts = vec![verified(t1), verified(t2), verified(t3)];
        let coverage = coverage(&receipts);
        assert_eq!(
            project_current_authority_state(&subject(), &receipts, &coverage, 1_000).unwrap_err(),
            AuthorityStateSourceError::TransitionAfterSuperseded
        );
    }

    #[test]
    fn unverified_authority_binding_denies() {
        let mut transitions = lineage();
        let coverage = coverage(&transitions);
        transitions[1].verified_authority_ref = "authority:wrong".into();
        assert_eq!(
            project_current_authority_state(&subject(), &transitions, &coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::TransitionAuthorityMismatch
        );
    }

    #[test]
    fn different_authoritative_sources_deny() {
        let mut transitions = lineage();
        let coverage = coverage(&transitions);
        transitions[2].authoritative_source_ref = "authority-state-source:other".into();
        assert_eq!(
            project_current_authority_state(&subject(), &transitions, &coverage, 1_000)
                .unwrap_err(),
            AuthorityStateSourceError::AuthoritativeSourceMismatch
        );
    }

    #[test]
    fn generation_above_protocol_bound_denies_without_arithmetic() {
        let transition = AuthorityStateTransition {
            protocol_version: PROTOCOL_VERSION.into(),
            transition_id: "transition:overflow".into(),
            subject: subject(),
            generation: u64::MAX,
            kind: AuthorityStateTransitionKind::Reactivate,
            state: AuthorityFreshnessState::Active,
            previous_generation: Some(u64::MAX),
            previous_transition_digest: Some(d(9)),
            effective_at_ms: 500,
            status_record_ref: "status:overflow".into(),
            reason_digest: d(8),
            authority_ref: "authority:overflow".into(),
            authority_proof_ref: "authority-proof:overflow".into(),
            record_proof_ref: "record-proof:overflow".into(),
        };
        assert_eq!(
            transition.validate().unwrap_err(),
            AuthorityStateSourceError::GenerationOutOfRange
        );
    }
}
