// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact causal-position historical projection over one fully covered authority lineage.
//!
//! PR #91 already proves a complete, unambiguous, currently covered authority-state
//! lineage. Its historical API selects by wall-clock `as_of` time. This crate adds the
//! complementary selector needed by cross-lineage governance protocols: select the exact
//! state identified by one already-signed causal coordinate `(generation, transition digest)`.
//!
//! The causal coordinate does not establish authority by itself. This theorem first
//! re-runs #91 current covered-lineage qualification and only then proves that the exact
//! target transition is a member of that complete lineage. It never selects by timestamp,
//! input order, highest locally observed generation, or newest record.

#![forbid(unsafe_code)]

use mycelix_authority_freshness::{
    AuthorityFreshnessSnapshot, AuthorityFreshnessState, AuthoritySubjectRef,
    PROTOCOL_VERSION as FRESHNESS_PROTOCOL_VERSION,
};
use mycelix_authority_state_source::{
    project_current_authority_state, AuthorityStateSourceError, VerifiedAuthorityStateCoverage,
    VerifiedAuthorityStateTransition,
};
use mycelix_institutional_core::Digest32;
use std::fmt;

pub const CAUSAL_PROJECTION_IDENTITY_PROFILE: &str =
    "mycelix-authority-causal-projection-v1-blake3-framed";
const DOMAIN_CAUSAL_PROJECTION: &[u8] = b"mycelix/authority-state/causal-projection/v1";

/// Non-forgeable historical projection selected by one exact causal authority-state anchor.
///
/// This result cannot be converted into current freshness. Current execution authority must
/// continue to pass through the current-only freshness/current-operational authority stack.
#[derive(Debug)]
pub struct QualifiedCausalAuthorityStateProjection {
    subject: AuthoritySubjectRef,
    selected_snapshot: AuthorityFreshnessSnapshot,
    selected_transition_digest: Digest32,
    full_lineage_digest: Digest32,
    current_head_generation: u64,
    current_head_transition_digest: Digest32,
    authoritative_source_ref: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
    projection_digest: Digest32,
}

impl QualifiedCausalAuthorityStateProjection {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn snapshot(&self) -> &AuthorityFreshnessSnapshot {
        &self.selected_snapshot
    }

    pub fn state(&self) -> AuthorityFreshnessState {
        self.selected_snapshot.state
    }

    pub fn generation(&self) -> u64 {
        self.selected_snapshot.generation
    }

    pub fn selected_transition_digest(&self) -> Digest32 {
        self.selected_transition_digest
    }

    pub fn full_lineage_digest(&self) -> Digest32 {
        self.full_lineage_digest
    }

    pub fn current_head_generation(&self) -> u64 {
        self.current_head_generation
    }

    pub fn current_head_transition_digest(&self) -> Digest32 {
        self.current_head_transition_digest
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

    pub fn projection_digest(&self) -> Digest32 {
        self.projection_digest
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CausalAuthorityProjectionError {
    Source(AuthorityStateSourceError),
    ZeroTargetGeneration,
    ZeroTargetTransitionDigest,
    TargetBeyondCoveredHead,
    TargetTransitionDigestMismatch,
    DigestCollision,
    InvalidProjectedSnapshot,
    InvalidSubjectIdentity,
}

impl From<AuthorityStateSourceError> for CausalAuthorityProjectionError {
    fn from(value: AuthorityStateSourceError) -> Self {
        Self::Source(value)
    }
}

/// Project the exact authority state named by one signed causal coordinate.
///
/// Security order is deliberate:
///
/// 1. re-run #91 current projection to prove complete current source coverage;
/// 2. prove the target generation is not beyond that covered head;
/// 3. require the exact target transition digest at that generation;
/// 4. derive one historical-only projection identity committing the full covered lineage.
///
/// `effective_at_ms` remains part of the underlying immutable authority-state semantics and
/// #91 causal validation, but it is never used here to choose the target state.
pub fn project_authority_state_at_causal_position(
    subject: &AuthoritySubjectRef,
    receipts: &[VerifiedAuthorityStateTransition],
    coverage: &VerifiedAuthorityStateCoverage,
    target_generation: u64,
    target_transition_digest: Digest32,
    verification_now_ms: u64,
) -> Result<QualifiedCausalAuthorityStateProjection, CausalAuthorityProjectionError> {
    if target_generation == 0 {
        return Err(CausalAuthorityProjectionError::ZeroTargetGeneration);
    }
    if target_transition_digest.is_zero() {
        return Err(CausalAuthorityProjectionError::ZeroTargetTransitionDigest);
    }

    // This is the critical completeness gate. A valid historical prefix is never enough.
    let current = project_current_authority_state(subject, receipts, coverage, verification_now_ms)?;
    if target_generation > current.snapshot().generation {
        return Err(CausalAuthorityProjectionError::TargetBeyondCoveredHead);
    }

    let mut selected = None;
    for receipt in receipts {
        if receipt.transition.generation != target_generation {
            continue;
        }
        let digest = receipt.transition.identity_digest()?;
        if digest != target_transition_digest {
            continue;
        }
        if let Some(existing) = selected {
            if existing != &receipt.transition {
                // The source projector already denies semantic forks. Keep an explicit
                // fail-closed collision guard here because a digest is a protocol identity.
                return Err(CausalAuthorityProjectionError::DigestCollision);
            }
        } else {
            selected = Some(&receipt.transition);
        }
    }

    let selected = selected.ok_or(CausalAuthorityProjectionError::TargetTransitionDigestMismatch)?;
    let snapshot = AuthorityFreshnessSnapshot {
        protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
        subject: subject.clone(),
        generation: selected.generation,
        state: selected.state,
        effective_at_ms: selected.effective_at_ms,
        status_record_ref: selected.status_record_ref.clone(),
    };
    snapshot
        .validate()
        .map_err(|_| CausalAuthorityProjectionError::InvalidProjectedSnapshot)?;

    let projection_digest = derive_projection_digest(
        subject,
        current.lineage_digest(),
        current.snapshot().generation,
        current.selected_transition_digest(),
        target_generation,
        target_transition_digest,
        &snapshot,
    )?;

    Ok(QualifiedCausalAuthorityStateProjection {
        subject: subject.clone(),
        selected_snapshot: snapshot,
        selected_transition_digest: target_transition_digest,
        full_lineage_digest: current.lineage_digest(),
        current_head_generation: current.snapshot().generation,
        current_head_transition_digest: current.selected_transition_digest(),
        authoritative_source_ref: current.authoritative_source_ref().to_string(),
        verified_at_ms: current.verified_at_ms(),
        lease_until_ms: current.lease_until_ms(),
        projection_digest,
    })
}

fn derive_projection_digest(
    subject: &AuthoritySubjectRef,
    full_lineage_digest: Digest32,
    current_head_generation: u64,
    current_head_transition_digest: Digest32,
    target_generation: u64,
    target_transition_digest: Digest32,
    snapshot: &AuthorityFreshnessSnapshot,
) -> Result<Digest32, CausalAuthorityProjectionError> {
    let subject_digest = subject
        .identity_digest()
        .map_err(|_| CausalAuthorityProjectionError::InvalidSubjectIdentity)?;
    let snapshot_digest = snapshot
        .identity_digest()
        .map_err(|_| CausalAuthorityProjectionError::InvalidProjectedSnapshot)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CAUSAL_PROJECTION);
    frame(&mut hasher, CAUSAL_PROJECTION_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &subject_digest.0);
    frame(&mut hasher, &full_lineage_digest.0);
    frame(&mut hasher, &current_head_generation.to_le_bytes());
    frame(&mut hasher, &current_head_transition_digest.0);
    frame(&mut hasher, &target_generation.to_le_bytes());
    frame(&mut hasher, &target_transition_digest.0);
    frame(&mut hasher, &snapshot_digest.0);
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

impl fmt::Display for CausalAuthorityProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::Source(error) => return write!(f, "authority source projection failed: {error}"),
            Self::ZeroTargetGeneration => "causal target generation must be non-zero",
            Self::ZeroTargetTransitionDigest => "causal target transition digest must be non-zero",
            Self::TargetBeyondCoveredHead => "causal target lies beyond the independently covered authority head",
            Self::TargetTransitionDigestMismatch => "causal target digest does not identify the covered transition at that generation",
            Self::DigestCollision => "different authority transition semantics share one target digest",
            Self::InvalidProjectedSnapshot => "causal projection produced an invalid authority snapshot",
            Self::InvalidSubjectIdentity => "causal projection subject identity is invalid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for CausalAuthorityProjectionError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthoritySubjectKind, ProfiledDigest as FreshnessProfiledDigest,
    };
    use mycelix_authority_state_source::{
        AuthorityStateTransition, AuthorityStateTransitionKind,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::SigningPolicy,
            namespace: "identity:time-policy:transition-authority".into(),
            subject_id: "identity:policy-authority:test#ed25519-1@generation:1".into(),
            identity: FreshnessProfiledDigest {
                digest: d(1),
                profile: "mycelix-identity-time-policy-transition-signer-authority-v2-sha256-framed"
                    .into(),
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
        AuthorityStateTransition {
            protocol_version: mycelix_authority_state_source::PROTOCOL_VERSION.into(),
            transition_id: format!("authority-transition:{generation}"),
            subject: subject(),
            generation,
            kind,
            state,
            previous_generation: previous.map(|value| value.generation),
            previous_transition_digest: previous.map(|value| value.identity_digest().unwrap()),
            effective_at_ms,
            status_record_ref: format!("authority-status:{generation}"),
            reason_digest: d(10 + generation as u8),
            authority_ref: format!("constitution-authority:{generation}"),
            authority_proof_ref: format!("constitution-proof:{generation}"),
            record_proof_ref: format!("authority-record-proof:{generation}"),
        }
    }

    fn verified(transition: AuthorityStateTransition) -> VerifiedAuthorityStateTransition {
        VerifiedAuthorityStateTransition {
            transition_record_ref: transition.status_record_ref.clone(),
            verified_record_proof_ref: transition.record_proof_ref.clone(),
            verified_authority_ref: transition.authority_ref.clone(),
            verified_authority_proof_ref: transition.authority_proof_ref.clone(),
            authoritative_source_ref: "authority-state-source:constitution".into(),
            verification_ref: format!("authority-verification:{}", transition.generation),
            verified_at_ms: 1_000,
            lease_until_ms: 2_000,
            transition,
        }
    }

    fn lineage() -> Vec<VerifiedAuthorityStateTransition> {
        let first = transition(
            1,
            AuthorityStateTransitionKind::Establish,
            AuthorityFreshnessState::Active,
            None,
            100,
        );
        let second = transition(
            2,
            AuthorityStateTransitionKind::Revoke,
            AuthorityFreshnessState::Revoked,
            Some(&first),
            200,
        );
        let third = transition(
            3,
            AuthorityStateTransitionKind::Reactivate,
            AuthorityFreshnessState::Active,
            Some(&second),
            300,
        );
        vec![verified(first), verified(second), verified(third)]
    }

    fn coverage(receipts: &[VerifiedAuthorityStateTransition]) -> VerifiedAuthorityStateCoverage {
        let head = receipts
            .iter()
            .max_by_key(|receipt| receipt.transition.generation)
            .unwrap();
        VerifiedAuthorityStateCoverage {
            subject: subject(),
            authoritative_source_ref: "authority-state-source:constitution".into(),
            head_generation: head.transition.generation,
            head_transition_digest: head.transition.identity_digest().unwrap(),
            head_status_record_ref: head.transition.status_record_ref.clone(),
            coverage_proof_ref: "coverage-proof:constitution".into(),
            verification_ref: "coverage-verification:constitution".into(),
            verified_at_ms: 950,
            lease_until_ms: 1_900,
        }
    }

    #[test]
    fn exact_pre_revocation_generation_is_preserved_under_later_current_history() {
        let receipts = lineage();
        let target = receipts[0].transition.identity_digest().unwrap();
        let projection = project_authority_state_at_causal_position(
            &subject(),
            &receipts,
            &coverage(&receipts),
            1,
            target,
            1_000,
        )
        .unwrap();

        assert_eq!(projection.generation(), 1);
        assert_eq!(projection.state(), AuthorityFreshnessState::Active);
        assert_eq!(projection.current_head_generation(), 3);
        assert_eq!(projection.selected_transition_digest(), target);
    }

    #[test]
    fn exact_revoked_generation_is_projectable_without_time_selection() {
        let receipts = lineage();
        let target = receipts[1].transition.identity_digest().unwrap();
        let projection = project_authority_state_at_causal_position(
            &subject(),
            &receipts,
            &coverage(&receipts),
            2,
            target,
            1_000,
        )
        .unwrap();
        assert_eq!(projection.state(), AuthorityFreshnessState::Revoked);
    }

    #[test]
    fn input_order_cannot_change_causal_projection_identity() {
        let receipts = lineage();
        let mut reversed = receipts.clone();
        reversed.reverse();
        let target = receipts[0].transition.identity_digest().unwrap();
        let coverage = coverage(&receipts);
        let left = project_authority_state_at_causal_position(
            &subject(),
            &receipts,
            &coverage,
            1,
            target,
            1_000,
        )
        .unwrap();
        let right = project_authority_state_at_causal_position(
            &subject(),
            &reversed,
            &coverage,
            1,
            target,
            1_000,
        )
        .unwrap();
        assert_eq!(left.projection_digest(), right.projection_digest());
    }

    #[test]
    fn target_digest_substitution_fails_closed() {
        let receipts = lineage();
        assert_eq!(
            project_authority_state_at_causal_position(
                &subject(),
                &receipts,
                &coverage(&receipts),
                1,
                d(99),
                1_000,
            )
            .unwrap_err(),
            CausalAuthorityProjectionError::TargetTransitionDigestMismatch
        );
    }

    #[test]
    fn truncated_historical_prefix_cannot_bypass_current_coverage() {
        let full = lineage();
        let coverage = coverage(&full);
        let target = full[0].transition.identity_digest().unwrap();
        let prefix = vec![full[0].clone(), full[1].clone()];
        assert_eq!(
            project_authority_state_at_causal_position(
                &subject(),
                &prefix,
                &coverage,
                1,
                target,
                1_000,
            )
            .unwrap_err(),
            CausalAuthorityProjectionError::Source(AuthorityStateSourceError::CoverageHeadMismatch)
        );
    }

    #[test]
    fn target_beyond_covered_head_fails_closed() {
        let receipts = lineage();
        assert_eq!(
            project_authority_state_at_causal_position(
                &subject(),
                &receipts,
                &coverage(&receipts),
                4,
                d(44),
                1_000,
            )
            .unwrap_err(),
            CausalAuthorityProjectionError::TargetBeyondCoveredHead
        );
    }

    #[test]
    fn result_exposes_no_live_freshness_conversion() {
        let source = include_str!("lib.rs");
        let start = source
            .index("impl QualifiedCausalAuthorityStateProjection")
            .unwrap();
        let end = source[start..]
            .index("pub enum CausalAuthorityProjectionError")
            .unwrap()
            + start;
        let api = &source[start..end];
        assert!(!api.contains("to_current_freshness_receipt"));
        assert!(!api.contains("VerifiedAuthorityFreshness"));
    }
}
