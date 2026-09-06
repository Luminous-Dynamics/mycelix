// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed current-state policy for Identity trust credentials.
//!
//! This crate intentionally contains no HDK/HDI dependency. Network traversal is
//! performed by the coordinator; this crate only collapses an observed credential
//! lineage into a presentation-safe structural state.
//!
//! The word **observed** is deliberate. A successful network read is not a proof
//! of global DHT completeness, so `ObservedStructurallyActive` must never be used
//! as cryptographic verification or elevated authority.

#![forbid(unsafe_code)]

/// One valid update observed in a credential lineage.
///
/// New updates are constrained by the revocation-only integrity rule, but
/// historical actions may predate that rule. The resolver must therefore inspect
/// every update entry rather than infer revocation from update existence alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CredentialUpdateObservation {
    /// Whether the update explicitly marks the credential revoked.
    pub revoked: bool,
    /// Whether every assertion-bearing field matches the canonical creation root.
    pub assertion_equivalent_to_root: bool,
}

/// Network-derived current-state evidence supplied to the pure policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CredentialStateEvidence<'a> {
    /// The canonical root/details could not be obtained with the required network read.
    Unavailable,
    /// Holochain returned a details shape other than the expected record details.
    UnexpectedDetailsKind,
    /// The canonical creation root and its observed CRUD metadata were resolved.
    Resolved(CredentialLineageObservation<'a>),
}

/// Observed CRUD state for one canonical credential creation root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CredentialLineageObservation<'a> {
    /// All valid update records currently observed from the canonical root.
    pub updates: &'a [CredentialUpdateObservation],
    /// Number of valid deletes currently observed against the canonical root.
    pub valid_delete_count: usize,
    /// Root credential expiration, expressed in Holochain timestamp microseconds.
    pub expires_at_micros: Option<i64>,
    /// Authoritative Holochain observation time from `sys_time()`.
    pub observed_at_micros: i64,
}

/// Structural state safe to expose after collapsing observed lineage evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedCredentialState {
    /// No revocation/delete/expiration was observed in the resolved lineage.
    ///
    /// This is diagnostic structural state only; it is not proof verification and
    /// must not by itself raise authority.
    ObservedStructurallyActive,
    /// The credential is definitely non-active under the observed evidence.
    NoActiveCredential(NoActiveCredentialReason),
}

/// Why a resolved credential is non-active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoActiveCredentialReason {
    /// A valid delete is observed for the canonical root.
    Deleted,
    /// At least one conforming revocation update is observed.
    Revoked,
    /// The credential expiration is at or before the observation time.
    Expired,
}

/// Fail-closed resolution errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CredentialStateResolutionError {
    /// Required network/current-state evidence was unavailable.
    EvidenceUnavailable,
    /// Holochain returned a details variant that cannot prove credential lineage state.
    UnexpectedDetailsKind,
    /// A historical or otherwise valid update does not satisfy the revocation-only
    /// assertion-equivalence theorem required by the current resolver.
    NonConformingUpdate { index: usize },
}

/// Collapse one network observation into structural current-state semantics.
///
/// Security ordering:
/// 1. every observed update must first satisfy the revocation-only theorem;
/// 2. a delete makes the credential non-active;
/// 3. any conforming update makes the credential revoked;
/// 4. expiration makes the credential non-active;
/// 5. only then may the root be described as *observed structurally active*.
pub fn resolve_observed_credential_state(
    evidence: CredentialStateEvidence<'_>,
) -> Result<ObservedCredentialState, CredentialStateResolutionError> {
    let lineage = match evidence {
        CredentialStateEvidence::Unavailable => {
            return Err(CredentialStateResolutionError::EvidenceUnavailable)
        }
        CredentialStateEvidence::UnexpectedDetailsKind => {
            return Err(CredentialStateResolutionError::UnexpectedDetailsKind)
        }
        CredentialStateEvidence::Resolved(lineage) => lineage,
    };

    for (index, update) in lineage.updates.iter().enumerate() {
        if !update.revoked || !update.assertion_equivalent_to_root {
            return Err(CredentialStateResolutionError::NonConformingUpdate { index });
        }
    }

    if lineage.valid_delete_count > 0 {
        return Ok(ObservedCredentialState::NoActiveCredential(
            NoActiveCredentialReason::Deleted,
        ));
    }

    if !lineage.updates.is_empty() {
        return Ok(ObservedCredentialState::NoActiveCredential(
            NoActiveCredentialReason::Revoked,
        ));
    }

    if lineage
        .expires_at_micros
        .is_some_and(|expires_at| lineage.observed_at_micros >= expires_at)
    {
        return Ok(ObservedCredentialState::NoActiveCredential(
            NoActiveCredentialReason::Expired,
        ));
    }

    Ok(ObservedCredentialState::ObservedStructurallyActive)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn resolved<'a>(
        updates: &'a [CredentialUpdateObservation],
        valid_delete_count: usize,
        expires_at_micros: Option<i64>,
        observed_at_micros: i64,
    ) -> CredentialStateEvidence<'a> {
        CredentialStateEvidence::Resolved(CredentialLineageObservation {
            updates,
            valid_delete_count,
            expires_at_micros,
            observed_at_micros,
        })
    }

    #[test]
    fn unavailable_evidence_fails_closed() {
        assert_eq!(
            resolve_observed_credential_state(CredentialStateEvidence::Unavailable),
            Err(CredentialStateResolutionError::EvidenceUnavailable)
        );
    }

    #[test]
    fn unexpected_details_kind_fails_closed() {
        assert_eq!(
            resolve_observed_credential_state(CredentialStateEvidence::UnexpectedDetailsKind),
            Err(CredentialStateResolutionError::UnexpectedDetailsKind)
        );
    }

    #[test]
    fn historical_non_revocation_update_never_becomes_active() {
        let updates = [CredentialUpdateObservation {
            revoked: false,
            assertion_equivalent_to_root: true,
        }];
        assert_eq!(
            resolve_observed_credential_state(resolved(&updates, 0, None, 100)),
            Err(CredentialStateResolutionError::NonConformingUpdate { index: 0 })
        );
    }

    #[test]
    fn assertion_mutating_historical_update_never_becomes_active() {
        let updates = [CredentialUpdateObservation {
            revoked: true,
            assertion_equivalent_to_root: false,
        }];
        assert_eq!(
            resolve_observed_credential_state(resolved(&updates, 0, None, 100)),
            Err(CredentialStateResolutionError::NonConformingUpdate { index: 0 })
        );
    }

    #[test]
    fn conforming_revocation_update_is_non_active() {
        let updates = [CredentialUpdateObservation {
            revoked: true,
            assertion_equivalent_to_root: true,
        }];
        assert_eq!(
            resolve_observed_credential_state(resolved(&updates, 0, None, 100)),
            Ok(ObservedCredentialState::NoActiveCredential(
                NoActiveCredentialReason::Revoked
            ))
        );
    }

    #[test]
    fn delete_is_non_active() {
        assert_eq!(
            resolve_observed_credential_state(resolved(&[], 1, None, 100)),
            Ok(ObservedCredentialState::NoActiveCredential(
                NoActiveCredentialReason::Deleted
            ))
        );
    }

    #[test]
    fn malformed_update_is_reported_even_when_delete_exists() {
        let updates = [CredentialUpdateObservation {
            revoked: false,
            assertion_equivalent_to_root: false,
        }];
        assert_eq!(
            resolve_observed_credential_state(resolved(&updates, 1, None, 100)),
            Err(CredentialStateResolutionError::NonConformingUpdate { index: 0 })
        );
    }

    #[test]
    fn expiration_is_inclusive_at_boundary() {
        assert_eq!(
            resolve_observed_credential_state(resolved(&[], 0, Some(100), 100)),
            Ok(ObservedCredentialState::NoActiveCredential(
                NoActiveCredentialReason::Expired
            ))
        );
    }

    #[test]
    fn future_expiration_is_observed_structurally_active() {
        assert_eq!(
            resolve_observed_credential_state(resolved(&[], 0, Some(101), 100)),
            Ok(ObservedCredentialState::ObservedStructurallyActive)
        );
    }

    #[test]
    fn no_expiration_and_no_crud_is_observed_structurally_active() {
        assert_eq!(
            resolve_observed_credential_state(resolved(&[], 0, None, 100)),
            Ok(ObservedCredentialState::ObservedStructurallyActive)
        );
    }

    #[test]
    fn observed_active_is_not_named_verified_or_authorized() {
        let name = format!("{:?}", ObservedCredentialState::ObservedStructurallyActive);
        assert!(!name.contains("Verified"));
        assert!(!name.contains("Authorized"));
        assert!(!name.contains("Current"));
    }
}
