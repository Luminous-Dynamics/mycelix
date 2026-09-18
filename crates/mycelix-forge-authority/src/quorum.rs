// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural threshold evaluation over already-authenticated principals.
//!
//! This module does not authenticate signatures or sessions. Callers must
//! establish principal authenticity before supplying identities here. The
//! result means only that distinct, authenticated principals satisfy the
//! structural membership/threshold policy of one exact authority epoch.

use crate::{AuthorityEpoch, Capability, PrincipalId};
use serde::Serialize;
use thiserror::Error;

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct StructuralQuorum {
    capability: Capability,
    threshold: u16,
    eligible_authenticated: Vec<PrincipalId>,
    satisfied: bool,
}

impl StructuralQuorum {
    pub const fn capability(&self) -> Capability {
        self.capability
    }

    pub const fn threshold(&self) -> u16 {
        self.threshold
    }

    pub fn eligible_authenticated(&self) -> &[PrincipalId] {
        &self.eligible_authenticated
    }

    pub const fn satisfied(&self) -> bool {
        self.satisfied
    }
}

/// Evaluate a capability threshold after authentication has already occurred.
///
/// Duplicate principals are rejected rather than silently deduplicated so one
/// signature/session cannot be accidentally counted multiple times by an
/// adapter. Ineligible authenticated principals are ignored for the requested
/// capability but remain outside the quorum count.
pub fn evaluate_structural_quorum(
    epoch: &AuthorityEpoch,
    capability: Capability,
    unix_ms: u64,
    authenticated_principals: &[PrincipalId],
) -> Result<StructuralQuorum, QuorumError> {
    if !epoch.is_valid_at(unix_ms) {
        return Err(QuorumError::EpochNotValidAt(unix_ms));
    }

    let threshold = epoch
        .threshold_for(capability)
        .ok_or(QuorumError::CapabilityNotDefined(capability))?;

    let mut principals = authenticated_principals.to_vec();
    principals.sort();
    for pair in principals.windows(2) {
        if pair[0] == pair[1] {
            return Err(QuorumError::DuplicateAuthenticatedPrincipal(
                pair[0].clone(),
            ));
        }
    }

    let eligible_authenticated = principals
        .into_iter()
        .filter(|principal| epoch.is_principal_eligible(principal, capability, unix_ms))
        .collect::<Vec<_>>();
    let satisfied = eligible_authenticated.len() >= usize::from(threshold);

    Ok(StructuralQuorum {
        capability,
        threshold,
        eligible_authenticated,
        satisfied,
    })
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum QuorumError {
    #[error("authority epoch is not valid at {0}")]
    EpochNotValidAt(u64),
    #[error("capability {0:?} is not defined by this authority epoch")]
    CapabilityNotDefined(Capability),
    #[error("authenticated principal was supplied more than once: {0}")]
    DuplicateAuthenticatedPrincipal(PrincipalId),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AuthorityEpochParts, CapabilityRule, Digest, DigestAlgorithm, PrincipalGrant,
        ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn epoch() -> AuthorityEpoch {
        let seed = ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22));
        let project = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms: 100,
            valid_until_unix_ms: Some(200),
            grants: vec![
                PrincipalGrant::new(
                    principal(1),
                    [Capability::ManageAuthority, Capability::ReviewSource],
                )
                .unwrap(),
                PrincipalGrant::new(
                    principal(2),
                    [Capability::ManageAuthority, Capability::ReviewSource],
                )
                .unwrap(),
                PrincipalGrant::new(principal(3), [Capability::ManageAuthority]).unwrap(),
            ],
            thresholds: vec![
                CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 2).unwrap(),
            ],
            revoked_principals: vec![],
        })
        .unwrap()
    }

    #[test]
    fn distinct_eligible_principals_satisfy_threshold() {
        let result = evaluate_structural_quorum(
            &epoch(),
            Capability::ReviewSource,
            150,
            &[principal(2), principal(1)],
        )
        .unwrap();
        assert!(result.satisfied());
        assert_eq!(result.threshold(), 2);
        assert_eq!(result.eligible_authenticated().len(), 2);
    }

    #[test]
    fn duplicate_principal_cannot_be_double_counted() {
        assert_eq!(
            evaluate_structural_quorum(
                &epoch(),
                Capability::ReviewSource,
                150,
                &[principal(1), principal(1)],
            )
            .unwrap_err(),
            QuorumError::DuplicateAuthenticatedPrincipal(principal(1))
        );
    }

    #[test]
    fn authenticated_but_ineligible_principal_does_not_count() {
        let result = evaluate_structural_quorum(
            &epoch(),
            Capability::ReviewSource,
            150,
            &[principal(1), principal(3)],
        )
        .unwrap();
        assert!(!result.satisfied());
        assert_eq!(result.eligible_authenticated(), &[principal(1)]);
    }

    #[test]
    fn invalid_epoch_time_fails_closed() {
        assert_eq!(
            evaluate_structural_quorum(
                &epoch(),
                Capability::ReviewSource,
                200,
                &[principal(1), principal(2)],
            )
            .unwrap_err(),
            QuorumError::EpochNotValidAt(200)
        );
    }
}
