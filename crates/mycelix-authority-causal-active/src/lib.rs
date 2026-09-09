// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Historical positive-authority qualification over one exact causal authority-state projection.
//!
//! PR #429 proves that an exact `(AuthoritySubjectRef, generation, transition digest)`
//! belongs to one fully currently covered, unambiguous authority-state lineage. This crate
//! adds only the positive historical-state gate: the selected exact causal coordinate must
//! itself resolve to `Active`, and the current source-completeness evidence must still be
//! inside its bounded reuse window at qualification time.
//!
//! Success is historical authority only. It cannot become live current freshness.

#![forbid(unsafe_code)]

use mycelix_authority_causal_projection::QualifiedCausalAuthorityStateProjection;
use mycelix_authority_freshness::{
    AuthorityFreshnessSnapshot, AuthorityFreshnessState, AuthoritySubjectRef,
};
use mycelix_institutional_core::Digest32;
use std::fmt;

pub const ACTIVE_CAUSAL_AUTHORITY_IDENTITY_PROFILE: &str =
    "mycelix-authority-causal-active-v1-blake3-framed";
const DOMAIN_ACTIVE_CAUSAL_AUTHORITY: &[u8] = b"mycelix/authority-state/causal-active/v1";

/// Non-forgeable positive historical authority for one exact #429 causal projection.
///
/// This result deliberately has no conversion to current freshness/current operational authority.
#[derive(Debug)]
pub struct QualifiedActiveCausalAuthority {
    subject: AuthoritySubjectRef,
    selected_snapshot: AuthorityFreshnessSnapshot,
    selected_transition_digest: Digest32,
    causal_projection_digest: Digest32,
    full_lineage_digest: Digest32,
    current_head_generation: u64,
    current_head_transition_digest: Digest32,
    authoritative_source_ref: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
    active_authority_digest: Digest32,
}

impl QualifiedActiveCausalAuthority {
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn snapshot(&self) -> &AuthorityFreshnessSnapshot {
        &self.selected_snapshot
    }

    pub fn generation(&self) -> u64 {
        self.selected_snapshot.generation
    }

    pub fn selected_transition_digest(&self) -> Digest32 {
        self.selected_transition_digest
    }

    pub fn causal_projection_digest(&self) -> Digest32 {
        self.causal_projection_digest
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

    pub fn active_authority_digest(&self) -> Digest32 {
        self.active_authority_digest
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ActiveCausalAuthorityError {
    HistoricalStateNotActive(AuthorityFreshnessState),
    InvalidVerificationWindow,
    InvalidSubjectIdentity,
    InvalidSnapshotIdentity,
}

/// Promote one exact #429 historical causal projection only when the selected state is `Active`
/// and its complete-source verification evidence remains reusable at `verification_now_ms`.
///
/// The time argument gates evidence freshness only. It never selects the historical state; that
/// state was already fixed by #429's exact generation + transition-digest causal coordinate.
///
/// The qualifier accepts no loose subject/generation/digest/state fields. All stable authority
/// material is copied from the opaque #429 capability, preserving its full-current-coverage proof.
pub fn qualify_active_causal_authority_at(
    projection: &QualifiedCausalAuthorityStateProjection,
    verification_now_ms: u64,
) -> Result<QualifiedActiveCausalAuthority, ActiveCausalAuthorityError> {
    if verification_now_ms == 0
        || projection.verified_at_ms() > verification_now_ms
        || projection.lease_until_ms() <= verification_now_ms
    {
        return Err(ActiveCausalAuthorityError::InvalidVerificationWindow);
    }
    if projection.state() != AuthorityFreshnessState::Active {
        return Err(ActiveCausalAuthorityError::HistoricalStateNotActive(
            projection.state(),
        ));
    }

    let active_authority_digest = derive_active_authority_digest(projection)?;

    Ok(QualifiedActiveCausalAuthority {
        subject: projection.subject().clone(),
        selected_snapshot: projection.snapshot().clone(),
        selected_transition_digest: projection.selected_transition_digest(),
        causal_projection_digest: projection.projection_digest(),
        full_lineage_digest: projection.full_lineage_digest(),
        current_head_generation: projection.current_head_generation(),
        current_head_transition_digest: projection.current_head_transition_digest(),
        authoritative_source_ref: projection.authoritative_source_ref().to_string(),
        verified_at_ms: projection.verified_at_ms(),
        lease_until_ms: projection.lease_until_ms(),
        active_authority_digest,
    })
}

fn derive_active_authority_digest(
    projection: &QualifiedCausalAuthorityStateProjection,
) -> Result<Digest32, ActiveCausalAuthorityError> {
    let subject_digest = projection
        .subject()
        .identity_digest()
        .map_err(|_| ActiveCausalAuthorityError::InvalidSubjectIdentity)?;
    let snapshot_digest = projection
        .snapshot()
        .identity_digest()
        .map_err(|_| ActiveCausalAuthorityError::InvalidSnapshotIdentity)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_ACTIVE_CAUSAL_AUTHORITY);
    frame(
        &mut hasher,
        ACTIVE_CAUSAL_AUTHORITY_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, &subject_digest.0);
    frame(&mut hasher, &projection.projection_digest().0);
    frame(&mut hasher, &snapshot_digest.0);
    frame(&mut hasher, &projection.generation().to_le_bytes());
    frame(&mut hasher, &projection.selected_transition_digest().0);
    frame(&mut hasher, &projection.full_lineage_digest().0);
    frame(
        &mut hasher,
        &projection.current_head_generation().to_le_bytes(),
    );
    frame(&mut hasher, &projection.current_head_transition_digest().0);
    // Explicit positive-state commitment. Future state enum evolution cannot silently alias Active.
    frame(&mut hasher, &[1]);
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

impl fmt::Display for ActiveCausalAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::HistoricalStateNotActive(state) => {
                write!(f, "historical causal authority state is not Active: {state:?}")
            }
            Self::InvalidVerificationWindow => {
                write!(f, "historical causal authority source evidence is outside its reuse window")
            }
            Self::InvalidSubjectIdentity => write!(f, "historical causal authority subject is invalid"),
            Self::InvalidSnapshotIdentity => {
                write!(f, "historical causal authority snapshot is invalid")
            }
        }
    }
}

impl std::error::Error for ActiveCausalAuthorityError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_causal_projection::project_authority_state_at_causal_position;
    use mycelix_authority_freshness::{
        AuthoritySubjectKind, ProfiledDigest as FreshnessProfiledDigest,
    };
    use mycelix_authority_state_source::{
        AuthorityStateTransition, AuthorityStateTransitionKind, VerifiedAuthorityStateCoverage,
        VerifiedAuthorityStateTransition,
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
    fn old_active_generation_remains_historically_positive_under_later_history() {
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
        let qualified = qualify_active_causal_authority_at(&projection, 1_000).unwrap();

        assert_eq!(qualified.generation(), 1);
        assert_eq!(qualified.snapshot().state, AuthorityFreshnessState::Active);
        assert_eq!(qualified.current_head_generation(), 3);
        assert_eq!(qualified.selected_transition_digest(), target);
        assert!(!qualified.active_authority_digest().is_zero());
    }

    #[test]
    fn exact_revoked_coordinate_cannot_become_positive_after_reactivation() {
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
        assert_eq!(
            qualify_active_causal_authority_at(&projection, 1_000).unwrap_err(),
            ActiveCausalAuthorityError::HistoricalStateNotActive(
                AuthorityFreshnessState::Revoked
            )
        );
    }

    #[test]
    fn expired_complete_source_evidence_cannot_be_promoted() {
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
        assert_eq!(
            qualify_active_causal_authority_at(&projection, projection.lease_until_ms()).unwrap_err(),
            ActiveCausalAuthorityError::InvalidVerificationWindow
        );
    }

    #[test]
    fn complete_lineage_identity_remains_part_of_positive_history() {
        let full = lineage();
        let target = full[0].transition.identity_digest().unwrap();
        let full_projection = project_authority_state_at_causal_position(
            &subject(),
            &full,
            &coverage(&full),
            1,
            target,
            1_000,
        )
        .unwrap();
        let full_positive = qualify_active_causal_authority_at(&full_projection, 1_000).unwrap();

        let prefix = vec![full[0].clone()];
        let prefix_projection = project_authority_state_at_causal_position(
            &subject(),
            &prefix,
            &coverage(&prefix),
            1,
            target,
            1_000,
        )
        .unwrap();
        let prefix_positive = qualify_active_causal_authority_at(&prefix_projection, 1_000).unwrap();

        assert_ne!(
            full_positive.active_authority_digest(),
            prefix_positive.active_authority_digest()
        );
    }

    #[test]
    fn positive_historical_capability_has_no_live_freshness_conversion() {
        let source = include_str!("lib.rs");
        let start = source.index("impl QualifiedActiveCausalAuthority").unwrap();
        let end = source[start..]
            .index("pub enum ActiveCausalAuthorityError")
            .unwrap()
            + start;
        let api = &source[start..end];
        assert!(!api.contains("VerifiedAuthorityFreshness"));
        assert!(!api.contains("QualifiedCurrentOperationalAuthority"));
        assert!(!api.contains("to_current"));
    }
}
