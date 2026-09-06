// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Identity trust-resolution adapter.
//!
//! This crate owns the semantic boundary between Identity's local credential
//! records and the cross-domain `TrustResolutionV1` protocol. It deliberately
//! does not perform network I/O or cryptographic proof verification.
//!
//! A coordinator/runtime caller is responsible only for gathering the canonical
//! credential root, every observed update record, valid delete count, and
//! authoritative Holochain observation time. This crate then:
//!
//! 1. compares every update against the canonical root assertion;
//! 2. delegates CRUD/expiry collapse to the fail-closed current-state policy;
//! 3. maps Identity-local trust tiers explicitly into the shared wire tier;
//! 4. emits only `TrustResolutionV1::quarantined(...)`.
//!
//! No function in this crate can produce verified proof state or elevated
//! consequential authority.

#![forbid(unsafe_code)]

use mycelix_trust_current_state_policy::{
    CredentialLineageObservation, CredentialStateEvidence, CredentialStateResolutionError,
    CredentialUpdateObservation, ObservedCredentialState, resolve_observed_credential_state,
};
use mycelix_trust_protocol::{
    StructuralTrustStateV1, StructuralTrustTierV1, TrustResolutionV1,
};
use trust_credential_integrity::{TrustCredential, TrustTier};

/// Adapter failures are fail-closed. Callers must not convert these into
/// `NoActiveCredential`, because unavailable or nonconforming lineage evidence is
/// not equivalent to proving that no active credential exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustResolutionRuntimeError {
    CurrentState(CredentialStateResolutionError),
}

impl From<CredentialStateResolutionError> for TrustResolutionRuntimeError {
    fn from(value: CredentialStateResolutionError) -> Self {
        Self::CurrentState(value)
    }
}

/// Compare the assertion-bearing fields of a credential update with its
/// canonical creation root.
///
/// Revocation metadata (`revoked`, `revocation_reason`, `revoked_at`) is
/// intentionally excluded: those are the only fields permitted to change by the
/// revocation-only integrity contract. Everything that contributes to the
/// credential assertion remains immutable.
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

/// Convert one fetched update record into the pure policy observation required
/// for historical/current-lineage collapse.
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
///
/// This function is intentionally exhaustive so adding a new Identity tier
/// cannot silently reuse an unrelated consumer enum representation.
pub const fn map_structural_tier(tier: &TrustTier) -> StructuralTrustTierV1 {
    match tier {
        TrustTier::Observer => StructuralTrustTierV1::Observer,
        TrustTier::Basic => StructuralTrustTierV1::Basic,
        TrustTier::Standard => StructuralTrustTierV1::Standard,
        TrustTier::Elevated => StructuralTrustTierV1::Elevated,
        TrustTier::Guardian => StructuralTrustTierV1::Guardian,
    }
}

/// Collapse fetched credential lineage records into the permanently quarantined
/// V1 cross-domain protocol.
///
/// `observed_updates` must contain the actual credential entry body for every
/// update reported by network-aware `RecordDetails`. Supplying only update counts
/// is insufficient because historical updates may predate the revocation-only
/// integrity contract.
pub fn resolve_to_v1(
    root: &TrustCredential,
    observed_updates: &[TrustCredential],
    valid_delete_count: usize,
    observed_at_micros: i64,
) -> Result<TrustResolutionV1, TrustResolutionRuntimeError> {
    let update_observations: Vec<_> = observed_updates
        .iter()
        .map(|updated| observe_update(root, updated))
        .collect();

    let evidence = CredentialStateEvidence::Resolved(CredentialLineageObservation {
        updates: &update_observations,
        valid_delete_count,
        expires_at_micros: root.expires_at.as_ref().map(|ts| ts.as_micros()),
        observed_at_micros,
    });

    let observed_state = resolve_observed_credential_state(evidence)?;
    let structural = match observed_state {
        ObservedCredentialState::ObservedStructurallyActive => {
            StructuralTrustStateV1::ActiveTier(map_structural_tier(&root.trust_tier))
        }
        ObservedCredentialState::NoActiveCredential(_) => {
            StructuralTrustStateV1::NoActiveCredential
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

    fn credential(tier: TrustTier) -> TrustCredential {
        TrustCredential {
            id: "cred-1".to_string(),
            subject_did: "did:mycelix:subject".to_string(),
            issuer_did: "did:mycelix:issuer".to_string(),
            kvector_commitment: vec![7u8; 32],
            range_proof: vec![1, 2, 3, 4],
            trust_score_range: TrustScoreRange {
                lower: 0.8,
                upper: 0.9,
            },
            trust_tier: tier,
            issued_at: Timestamp::from_micros(100),
            expires_at: Some(Timestamp::from_micros(10_000)),
            revoked: false,
            revocation_reason: None,
            revoked_at: None,
            supersedes: Some("cred-0".to_string()),
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
    fn observed_guardian_remains_not_established_and_quarantined() {
        let root = credential(TrustTier::Guardian);
        let resolution = resolve_to_v1(&root, &[], 0, 1_000).unwrap();

        assert_eq!(
            resolution.structural(),
            StructuralTrustStateV1::ActiveTier(StructuralTrustTierV1::Guardian)
        );
        assert_eq!(
            resolution.proof_verification(),
            ProofVerificationStateV1::NotEstablished
        );
        assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn conforming_revocation_maps_to_no_active_credential() {
        let root = credential(TrustTier::Elevated);
        let revoked = revoked_from(&root);
        let resolution = resolve_to_v1(&root, &[revoked], 0, 1_000).unwrap();

        assert_eq!(resolution.structural(), StructuralTrustStateV1::NoActiveCredential);
        assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn delete_maps_to_no_active_credential() {
        let root = credential(TrustTier::Standard);
        let resolution = resolve_to_v1(&root, &[], 1, 1_000).unwrap();
        assert_eq!(resolution.structural(), StructuralTrustStateV1::NoActiveCredential);
    }

    #[test]
    fn expiration_maps_to_no_active_credential_at_exact_boundary() {
        let root = credential(TrustTier::Standard);
        let resolution = resolve_to_v1(&root, &[], 0, 10_000).unwrap();
        assert_eq!(resolution.structural(), StructuralTrustStateV1::NoActiveCredential);
    }

    #[test]
    fn historical_tier_mutation_fails_closed() {
        let root = credential(TrustTier::Standard);
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
        let root = credential(TrustTier::Standard);
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
        let root = credential(TrustTier::Standard);
        let revoked = revoked_from(&root);
        assert!(credential_assertion_equivalent(&root, &revoked));
    }
}
