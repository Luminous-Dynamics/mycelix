// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Identity trust-resolution adapter.
//!
//! This crate owns the semantic boundary between Identity's local credential
//! records and the cross-domain `TrustResolutionV1` protocol. It deliberately
//! does not perform network I/O or cryptographic proof verification.
//!
//! A coordinator/runtime caller gathers canonical credential roots, every
//! observed update record, delete counts, and authoritative Holochain observation
//! time. This crate then owns root-assertion sanity, update equivalence,
//! historical-update collapse, tier mapping, subject-level ambiguity handling,
//! and quarantine-only V1 output.

#![forbid(unsafe_code)]

use mycelix_trust_current_state_policy::{
    CredentialLineageObservation, CredentialStateEvidence, CredentialStateResolutionError,
    CredentialUpdateObservation, ObservedCredentialState, resolve_observed_credential_state,
};
use mycelix_trust_protocol::{
    StructuralTrustStateV1, StructuralTrustTierV1, TrustResolutionV1,
};
use trust_credential_integrity::{TrustCredential, TrustTier};

/// Structural defects that can exist in historical credential roots admitted
/// before stricter validation is deployed.
///
/// These are fail-closed because V1 may expose a structural tier diagnostically;
/// even a quarantined diagnostic tier must not be derived from NaN/∞ or a range
/// whose midpoint disagrees with the claimed tier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalRootAssertionError {
    NonFiniteTrustRange,
    OutOfBoundsTrustRange,
    ReversedTrustRange,
    TierDoesNotMatchRangeMidpoint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustResolutionRuntimeError {
    MalformedCanonicalRoot(CanonicalRootAssertionError),
    CurrentState(CredentialStateResolutionError),
    SubjectMalformedCanonicalRoot {
        credential_index: usize,
        source: CanonicalRootAssertionError,
    },
    SubjectLineage {
        credential_index: usize,
        source: CredentialStateResolutionError,
    },
    /// V1 cannot truthfully select one structural tier when more than one
    /// credential root is observed active and no qualified issuer/supersession
    /// policy exists to choose among them.
    AmbiguousObservedActiveCredentials { count: usize },
}

impl From<CredentialStateResolutionError> for TrustResolutionRuntimeError {
    fn from(value: CredentialStateResolutionError) -> Self {
        Self::CurrentState(value)
    }
}

/// One canonical credential root and the entry bodies of every update observed
/// for that root by the network traversal layer.
#[derive(Debug, Clone, Copy)]
pub struct CredentialLineageInput<'a> {
    pub root: &'a TrustCredential,
    pub observed_updates: &'a [TrustCredential],
    pub valid_delete_count: usize,
}

/// Re-check the historically relevant structural assertion rules for a
/// canonical credential creation root.
///
/// The current integrity zome historically used ordinary float comparisons and
/// a weaker `upper >= tier.min_score()` check. Non-finite values can evade
/// ordinary comparisons, and that upper-bound check does not prove the claimed
/// tier equals the coordinator's midpoint-derived tier. The adapter therefore
/// re-establishes the intended structural theorem before exposing any observed
/// tier through V1.
pub fn validate_canonical_root_assertion(
    root: &TrustCredential,
) -> Result<(), CanonicalRootAssertionError> {
    let lower = root.trust_score_range.lower;
    let upper = root.trust_score_range.upper;

    if !lower.is_finite() || !upper.is_finite() {
        return Err(CanonicalRootAssertionError::NonFiniteTrustRange);
    }

    if !(0.0..=1.0).contains(&lower) || !(0.0..=1.0).contains(&upper) {
        return Err(CanonicalRootAssertionError::OutOfBoundsTrustRange);
    }

    if lower > upper {
        return Err(CanonicalRootAssertionError::ReversedTrustRange);
    }

    let midpoint = (lower as f64 + upper as f64) / 2.0;
    let expected_tier = TrustTier::from_score(midpoint);
    if expected_tier != root.trust_tier {
        return Err(CanonicalRootAssertionError::TierDoesNotMatchRangeMidpoint);
    }

    Ok(())
}

/// Compare assertion-bearing fields of a credential update with its canonical
/// creation root.
///
/// Revocation metadata (`revoked`, `revocation_reason`, `revoked_at`) is the only
/// intentionally excluded state. Every field contributing to the credential
/// assertion itself must remain equivalent.
pub fn credential_assertion_equivalent(
    root: &TrustCredential,
    candidate: &TrustCredential,
) -> bool {
    candidate.id == root.id
        && candidate.subject_did == root.subject_did
        && candidate.issuer_did == root.issuer_did
        && candidate.kvector_commitment == root.kvector_commitment
        && candidate.range_proof == root.range_proof
        && candidate.trust_score_range == root.trust_score_range
        && candidate.trust_tier == root.trust_tier
        && candidate.issued_at == root.issued_at
        && candidate.expires_at == root.expires_at
        && candidate.supersedes == root.supersedes
}

pub fn observe_update(
    root: &TrustCredential,
    updated: &TrustCredential,
) -> CredentialUpdateObservation {
    CredentialUpdateObservation {
        revoked: updated.revoked,
        assertion_equivalent_to_root: credential_assertion_equivalent(root, updated),
    }
}

/// Explicitly map Identity's local structural tier into the shared V1 wire tier.
/// This is intentionally exhaustive; there is no ordinal/serde reinterpretation.
pub const fn map_structural_tier(tier: &TrustTier) -> StructuralTrustTierV1 {
    match tier {
        TrustTier::Observer => StructuralTrustTierV1::Observer,
        TrustTier::Basic => StructuralTrustTierV1::Basic,
        TrustTier::Standard => StructuralTrustTierV1::Standard,
        TrustTier::Elevated => StructuralTrustTierV1::Elevated,
        TrustTier::Guardian => StructuralTrustTierV1::Guardian,
    }
}

fn resolve_lineage_state(
    root: &TrustCredential,
    observed_updates: &[TrustCredential],
    valid_delete_count: usize,
    observed_at_micros: i64,
) -> Result<ObservedCredentialState, CredentialStateResolutionError> {
    let update_observations: Vec<_> = observed_updates
        .iter()
        .map(|updated| observe_update(root, updated))
        .collect();

    resolve_observed_credential_state(CredentialStateEvidence::Resolved(
        CredentialLineageObservation {
            updates: &update_observations,
            valid_delete_count,
            expires_at_micros: root.expires_at.as_ref().map(|ts| ts.as_micros()),
            observed_at_micros,
        },
    ))
}

/// Resolve one canonical credential lineage into observation-scoped,
/// permanently-quarantined V1 semantics.
pub fn resolve_to_v1(
    root: &TrustCredential,
    observed_updates: &[TrustCredential],
    valid_delete_count: usize,
    observed_at_micros: i64,
) -> Result<TrustResolutionV1, TrustResolutionRuntimeError> {
    validate_canonical_root_assertion(root)
        .map_err(TrustResolutionRuntimeError::MalformedCanonicalRoot)?;

    let observed_state = resolve_lineage_state(
        root,
        observed_updates,
        valid_delete_count,
        observed_at_micros,
    )?;

    let structural = match observed_state {
        ObservedCredentialState::ObservedStructurallyActive => {
            StructuralTrustStateV1::ObservedActiveTier(map_structural_tier(&root.trust_tier))
        }
        ObservedCredentialState::NoActiveCredential(_) => {
            StructuralTrustStateV1::NoActiveCredentialObserved
        }
    };

    Ok(TrustResolutionV1::quarantined(structural))
}

/// Resolve all canonical credential roots observed for one subject.
///
/// V1 intentionally does not contain an issuer-selection or credential-priority
/// policy. Therefore this function refuses to choose the highest tier when more
/// than one root is observed structurally active. A future qualified policy may
/// introduce explicit issuer/supersession semantics in a separately reviewed
/// protocol version or adapter policy.
pub fn resolve_subject_to_v1(
    lineages: &[CredentialLineageInput<'_>],
    observed_at_micros: i64,
) -> Result<TrustResolutionV1, TrustResolutionRuntimeError> {
    let mut observed_active_tier: Option<StructuralTrustTierV1> = None;
    let mut observed_active_count = 0usize;

    for (credential_index, lineage) in lineages.iter().enumerate() {
        validate_canonical_root_assertion(lineage.root).map_err(|source| {
            TrustResolutionRuntimeError::SubjectMalformedCanonicalRoot {
                credential_index,
                source,
            }
        })?;

        let state = resolve_lineage_state(
            lineage.root,
            lineage.observed_updates,
            lineage.valid_delete_count,
            observed_at_micros,
        )
        .map_err(|source| TrustResolutionRuntimeError::SubjectLineage {
            credential_index,
            source,
        })?;

        if state == ObservedCredentialState::ObservedStructurallyActive {
            observed_active_count += 1;
            observed_active_tier = Some(map_structural_tier(&lineage.root.trust_tier));
        }
    }

    let structural = match (observed_active_count, observed_active_tier) {
        (0, _) => StructuralTrustStateV1::NoActiveCredentialObserved,
        (1, Some(tier)) => StructuralTrustStateV1::ObservedActiveTier(tier),
        (count, _) => {
            return Err(
                TrustResolutionRuntimeError::AmbiguousObservedActiveCredentials { count },
            )
        }
    };

    Ok(TrustResolutionV1::quarantined(structural))
}

#[cfg(test)]
mod tests {
    use super::*;
    use holochain_zome_types::prelude::Timestamp;
    use mycelix_trust_current_state_policy::CredentialStateResolutionError;
    use mycelix_trust_protocol::{
        ProofVerificationStateV1, TrustAuthorityDispositionV1,
    };
    use trust_credential_integrity::TrustScoreRange;

    fn range_for_tier(tier: &TrustTier) -> TrustScoreRange {
        match tier {
            TrustTier::Observer => TrustScoreRange {
                lower: 0.10,
                upper: 0.20,
            },
            TrustTier::Basic => TrustScoreRange {
                lower: 0.30,
                upper: 0.35,
            },
            TrustTier::Standard => TrustScoreRange {
                lower: 0.40,
                upper: 0.50,
            },
            TrustTier::Elevated => TrustScoreRange {
                lower: 0.60,
                upper: 0.70,
            },
            TrustTier::Guardian => TrustScoreRange {
                lower: 0.80,
                upper: 0.90,
            },
        }
    }

    fn credential(id: &str, tier: TrustTier) -> TrustCredential {
        let trust_score_range = range_for_tier(&tier);
        TrustCredential {
            id: id.to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            issuer_did: format!("did:mycelix:issuer:{id}"),
            kvector_commitment: vec![7u8; 32],
            range_proof: vec![1, 2, 3, 4],
            trust_score_range,
            trust_tier: tier,
            issued_at: Timestamp::from_micros(100),
            expires_at: Some(Timestamp::from_micros(10_000)),
            revoked: false,
            revocation_reason: None,
            revoked_at: None,
            supersedes: None,
        }
    }

    fn revoked_from(root: &TrustCredential) -> TrustCredential {
        let mut updated = root.clone();
        updated.revoked = true;
        updated.revocation_reason = Some("key compromise".to_string());
        updated.revoked_at = Some(Timestamp::from_micros(1_000));
        updated
    }

    #[test]
    fn every_identity_tier_maps_explicitly() {
        let pairs = [
            (TrustTier::Observer, StructuralTrustTierV1::Observer),
            (TrustTier::Basic, StructuralTrustTierV1::Basic),
            (TrustTier::Standard, StructuralTrustTierV1::Standard),
            (TrustTier::Elevated, StructuralTrustTierV1::Elevated),
            (TrustTier::Guardian, StructuralTrustTierV1::Guardian),
        ];
        for (local, wire) in pairs {
            assert_eq!(map_structural_tier(&local), wire);
        }
    }

    #[test]
    fn valid_fixture_ranges_match_their_claimed_tiers() {
        for tier in [
            TrustTier::Observer,
            TrustTier::Basic,
            TrustTier::Standard,
            TrustTier::Elevated,
            TrustTier::Guardian,
        ] {
            let root = credential("fixture", tier);
            assert_eq!(validate_canonical_root_assertion(&root), Ok(()));
        }
    }

    #[test]
    fn nan_root_range_fails_closed() {
        let mut root = credential("nan", TrustTier::Guardian);
        root.trust_score_range.lower = f32::NAN;
        assert_eq!(
            resolve_to_v1(&root, &[], 0, 1_000),
            Err(TrustResolutionRuntimeError::MalformedCanonicalRoot(
                CanonicalRootAssertionError::NonFiniteTrustRange
            ))
        );
    }

    #[test]
    fn infinite_root_range_fails_closed() {
        let mut root = credential("inf", TrustTier::Guardian);
        root.trust_score_range.upper = f32::INFINITY;
        assert_eq!(
            validate_canonical_root_assertion(&root),
            Err(CanonicalRootAssertionError::NonFiniteTrustRange)
        );
    }

    #[test]
    fn out_of_bounds_root_range_fails_closed() {
        let mut root = credential("bounds", TrustTier::Guardian);
        root.trust_score_range.upper = 1.01;
        assert_eq!(
            validate_canonical_root_assertion(&root),
            Err(CanonicalRootAssertionError::OutOfBoundsTrustRange)
        );
    }

    #[test]
    fn reversed_root_range_fails_closed() {
        let mut root = credential("reversed", TrustTier::Guardian);
        root.trust_score_range.lower = 0.95;
        root.trust_score_range.upper = 0.85;
        assert_eq!(
            validate_canonical_root_assertion(&root),
            Err(CanonicalRootAssertionError::ReversedTrustRange)
        );
    }

    #[test]
    fn claimed_tier_must_equal_range_midpoint_tier() {
        let mut root = credential("mismatch", TrustTier::Guardian);
        root.trust_score_range = TrustScoreRange {
            lower: 0.40,
            upper: 0.50,
        };
        assert_eq!(
            validate_canonical_root_assertion(&root),
            Err(CanonicalRootAssertionError::TierDoesNotMatchRangeMidpoint)
        );
    }

    #[test]
    fn observed_guardian_remains_not_established_and_quarantined() {
        let root = credential("guardian", TrustTier::Guardian);
        let resolution = resolve_to_v1(&root, &[], 0, 1_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian)
        );
        assert_eq!(
            resolution.proof_verification(),
            ProofVerificationStateV1::NotEstablished
        );
        assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn conforming_revocation_maps_to_observed_no_active_credential() {
        let root = credential("elevated", TrustTier::Elevated);
        let revoked = revoked_from(&root);
        let resolution = resolve_to_v1(&root, &[revoked], 0, 1_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::NoActiveCredentialObserved
        );
    }

    #[test]
    fn delete_maps_to_observed_no_active_credential() {
        let root = credential("standard", TrustTier::Standard);
        let resolution = resolve_to_v1(&root, &[], 1, 1_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::NoActiveCredentialObserved
        );
    }

    #[test]
    fn expiration_maps_to_observed_no_active_credential_at_exact_boundary() {
        let root = credential("standard", TrustTier::Standard);
        let resolution = resolve_to_v1(&root, &[], 0, 10_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::NoActiveCredentialObserved
        );
    }

    #[test]
    fn historical_tier_mutation_fails_closed() {
        let root = credential("standard", TrustTier::Standard);
        let mut legacy = revoked_from(&root);
        legacy.trust_tier = TrustTier::Guardian;
        assert_eq!(
            resolve_to_v1(&root, &[legacy], 0, 1_000),
            Err(TrustResolutionRuntimeError::CurrentState(
                CredentialStateResolutionError::NonConformingUpdate { index: 0 }
            ))
        );
    }

    #[test]
    fn historical_proof_mutation_fails_closed() {
        let root = credential("standard", TrustTier::Standard);
        let mut legacy = revoked_from(&root);
        legacy.range_proof.push(9);
        assert_eq!(
            resolve_to_v1(&root, &[legacy], 0, 1_000),
            Err(TrustResolutionRuntimeError::CurrentState(
                CredentialStateResolutionError::NonConformingUpdate { index: 0 }
            ))
        );
    }

    #[test]
    fn revocation_metadata_is_the_only_permitted_update_difference() {
        let root = credential("standard", TrustTier::Standard);
        let revoked = revoked_from(&root);
        assert!(credential_assertion_equivalent(&root, &revoked));
    }

    #[test]
    fn two_observed_active_credentials_are_ambiguous_not_highest_wins() {
        let standard = credential("standard", TrustTier::Standard);
        let guardian = credential("guardian", TrustTier::Guardian);
        let lineages = [
            CredentialLineageInput {
                root: &standard,
                observed_updates: &[],
                valid_delete_count: 0,
            },
            CredentialLineageInput {
                root: &guardian,
                observed_updates: &[],
                valid_delete_count: 0,
            },
        ];

        assert_eq!(
            resolve_subject_to_v1(&lineages, 1_000),
            Err(TrustResolutionRuntimeError::AmbiguousObservedActiveCredentials {
                count: 2
            })
        );
    }

    #[test]
    fn one_active_and_one_revoked_reports_only_the_observed_active_tier() {
        let standard = credential("standard", TrustTier::Standard);
        let guardian = credential("guardian", TrustTier::Guardian);
        let revoked_guardian = revoked_from(&guardian);
        let lineages = [
            CredentialLineageInput {
                root: &standard,
                observed_updates: &[],
                valid_delete_count: 0,
            },
            CredentialLineageInput {
                root: &guardian,
                observed_updates: core::slice::from_ref(&revoked_guardian),
                valid_delete_count: 0,
            },
        ];

        let resolution = resolve_subject_to_v1(&lineages, 1_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Standard)
        );
        assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn no_observed_active_roots_reports_observation_scoped_absence() {
        let standard = credential("standard", TrustTier::Standard);
        let revoked = revoked_from(&standard);
        let lineages = [CredentialLineageInput {
            root: &standard,
            observed_updates: core::slice::from_ref(&revoked),
            valid_delete_count: 0,
        }];
        let resolution = resolve_subject_to_v1(&lineages, 1_000).unwrap();
        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::NoActiveCredentialObserved
        );
    }

    #[test]
    fn malformed_subject_lineage_identifies_credential_index() {
        let standard = credential("standard", TrustTier::Standard);
        let guardian = credential("guardian", TrustTier::Guardian);
        let mut malformed_guardian = revoked_from(&guardian);
        malformed_guardian.range_proof.push(9);
        let lineages = [
            CredentialLineageInput {
                root: &standard,
                observed_updates: &[],
                valid_delete_count: 1,
            },
            CredentialLineageInput {
                root: &guardian,
                observed_updates: core::slice::from_ref(&malformed_guardian),
                valid_delete_count: 0,
            },
        ];

        assert_eq!(
            resolve_subject_to_v1(&lineages, 1_000),
            Err(TrustResolutionRuntimeError::SubjectLineage {
                credential_index: 1,
                source: CredentialStateResolutionError::NonConformingUpdate { index: 0 },
            })
        );
    }

    #[test]
    fn malformed_subject_root_identifies_credential_index() {
        let standard = credential("standard", TrustTier::Standard);
        let mut guardian = credential("guardian", TrustTier::Guardian);
        guardian.trust_score_range.lower = f32::NAN;
        let lineages = [
            CredentialLineageInput {
                root: &standard,
                observed_updates: &[],
                valid_delete_count: 1,
            },
            CredentialLineageInput {
                root: &guardian,
                observed_updates: &[],
                valid_delete_count: 0,
            },
        ];

        assert_eq!(
            resolve_subject_to_v1(&lineages, 1_000),
            Err(TrustResolutionRuntimeError::SubjectMalformedCanonicalRoot {
                credential_index: 1,
                source: CanonicalRootAssertionError::NonFiniteTrustRange,
            })
        );
    }
}
