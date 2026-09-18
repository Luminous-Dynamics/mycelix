// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Binding between stable project identity and the exact initial authority policy.

use crate::{
    AuthorityEpoch, AuthorityEpochParts, AuthorityError, Capability, CapabilityRule, Digest,
    DigestAlgorithm, ForgeCoreError, PrincipalGrant, ProjectIdentity, ProjectIdentitySeed,
    ProtocolVersion,
};
use serde::Serialize;
use std::collections::BTreeSet;
use thiserror::Error;

const ROOT_AUTHORITY_POLICY_DOMAIN_V1: &[u8] = b"mycelix-forge/root-authority-policy/v1\0";

/// Project-independent initial authority policy.
///
/// The policy is committed into [`ProjectIdentitySeed`] before the project ID is
/// derived. Keeping the policy independent of the project ID avoids a circular
/// commitment (`project -> authority -> project`) while still making the
/// project's identity commit to its initial authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RootAuthorityPolicy {
    version: ProtocolVersion,
    grants: Vec<PrincipalGrant>,
    thresholds: Vec<CapabilityRule>,
}

impl RootAuthorityPolicy {
    /// Normalize and validate root authority material.
    pub fn new(
        mut grants: Vec<PrincipalGrant>,
        mut thresholds: Vec<CapabilityRule>,
    ) -> Result<Self, AuthorityError> {
        if grants.is_empty() {
            return Err(AuthorityError::NoPrincipals);
        }

        grants.sort_by(|a, b| a.principal().cmp(b.principal()));
        for pair in grants.windows(2) {
            if pair[0].principal() == pair[1].principal() {
                return Err(AuthorityError::DuplicatePrincipal(
                    pair[0].principal().clone(),
                ));
            }
        }

        thresholds.sort_by_key(|rule| rule.capability().code());
        for pair in thresholds.windows(2) {
            if pair[0].capability() == pair[1].capability() {
                return Err(AuthorityError::DuplicateCapabilityRule(
                    pair[0].capability(),
                ));
            }
        }

        let granted_capabilities = grants
            .iter()
            .flat_map(|grant| grant.capabilities().iter().copied())
            .collect::<BTreeSet<_>>();

        for capability in &granted_capabilities {
            let Some(rule) = thresholds
                .iter()
                .find(|rule| rule.capability() == *capability)
            else {
                return Err(AuthorityError::MissingCapabilityRule(*capability));
            };

            let eligible = grants
                .iter()
                .filter(|grant| grant.has(*capability))
                .count();
            if usize::from(rule.threshold()) > eligible {
                return Err(AuthorityError::ThresholdExceedsEligible {
                    capability: *capability,
                    threshold: rule.threshold(),
                    eligible,
                });
            }
        }

        for rule in &thresholds {
            if !granted_capabilities.contains(&rule.capability()) {
                return Err(AuthorityError::RuleWithoutEligiblePrincipal(
                    rule.capability(),
                ));
            }
        }

        if !granted_capabilities.contains(&Capability::ManageAuthority) {
            return Err(AuthorityError::MissingManageAuthority);
        }

        Ok(Self {
            version: ProtocolVersion::CURRENT,
            grants,
            thresholds,
        })
    }

    pub const fn version(&self) -> ProtocolVersion {
        self.version
    }

    pub fn grants(&self) -> &[PrincipalGrant] {
        &self.grants
    }

    pub fn thresholds(&self) -> &[CapabilityRule] {
        &self.thresholds
    }

    /// Deterministic project-independent bytes committed by the project seed.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AuthorityError> {
        let mut out = Vec::new();
        out.extend_from_slice(ROOT_AUTHORITY_POLICY_DOMAIN_V1);
        out.extend_from_slice(&self.version.get().to_be_bytes());

        push_u32_len(&mut out, self.grants.len(), "root_grants")?;
        for grant in &self.grants {
            push_digest(&mut out, grant.principal().commitment())?;
            push_u16_len(
                &mut out,
                grant.capabilities().len(),
                "root_capabilities",
            )?;
            for capability in grant.capabilities() {
                out.extend_from_slice(&capability.code().to_be_bytes());
            }
        }

        push_u16_len(&mut out, self.thresholds.len(), "root_thresholds")?;
        for rule in &self.thresholds {
            out.extend_from_slice(&rule.capability().code().to_be_bytes());
            out.extend_from_slice(&rule.threshold().to_be_bytes());
        }

        Ok(out)
    }

    pub fn commitment(&self, algorithm: DigestAlgorithm) -> Result<Digest, AuthorityError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

/// Positive proof object that an epoch-zero authority state is bound to the
/// same root-policy commitment used to derive the project identity.
///
/// A raw [`AuthorityEpoch`] with `sequence == 0` is only a structurally valid
/// epoch. Consumers that need identity-bound genesis authority should require
/// this type (or equivalent independently verified evidence), not merely a raw
/// epoch.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BoundGenesisAuthority {
    root_policy: RootAuthorityPolicy,
    root_policy_commitment: Digest,
    epoch: AuthorityEpoch,
}

impl BoundGenesisAuthority {
    pub fn new(
        seed: &ProjectIdentitySeed,
        project: ProjectIdentity,
        root_policy: RootAuthorityPolicy,
        valid_from_unix_ms: u64,
        valid_until_unix_ms: Option<u64>,
    ) -> Result<Self, GenesisBindingError> {
        let expected_root = seed.root_authority_commitment();
        let actual_root = root_policy.commitment(expected_root.algorithm())?;
        if &actual_root != expected_root {
            return Err(GenesisBindingError::RootPolicyCommitmentMismatch {
                expected: expected_root.clone(),
                actual: actual_root,
            });
        }

        let derived_project = ProjectIdentity::derive(seed, project.digest().algorithm())?;
        if derived_project != project {
            return Err(GenesisBindingError::ProjectIdentityMismatch);
        }

        let epoch = AuthorityEpoch::new(AuthorityEpochParts {
            project,
            sequence: 0,
            previous: None,
            valid_from_unix_ms,
            valid_until_unix_ms,
            grants: root_policy.grants().to_vec(),
            thresholds: root_policy.thresholds().to_vec(),
            revoked_principals: vec![],
        })?;

        Ok(Self {
            root_policy,
            root_policy_commitment: expected_root.clone(),
            epoch,
        })
    }

    pub fn root_policy(&self) -> &RootAuthorityPolicy {
        &self.root_policy
    }

    pub fn root_policy_commitment(&self) -> &Digest {
        &self.root_policy_commitment
    }

    pub fn epoch(&self) -> &AuthorityEpoch {
        &self.epoch
    }
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GenesisBindingError {
    #[error("root authority policy commitment does not match the project seed")]
    RootPolicyCommitmentMismatch { expected: Digest, actual: Digest },
    #[error("project identity does not derive from the supplied project seed")]
    ProjectIdentityMismatch,
    #[error(transparent)]
    Authority(#[from] AuthorityError),
    #[error(transparent)]
    Core(#[from] ForgeCoreError),
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), AuthorityError> {
    let algorithm = digest.algorithm().id().as_bytes();
    push_u16_len(out, algorithm.len(), "root_digest_algorithm")?;
    out.extend_from_slice(algorithm);
    push_u32_len(out, digest.as_bytes().len(), "root_digest_bytes")?;
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_u16_len(out: &mut Vec<u8>, len: usize, field: &'static str) -> Result<(), AuthorityError> {
    let value = u16::try_from(len).map_err(|_| AuthorityError::CanonicalCollectionTooLarge {
        field,
        len,
        max: u16::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn push_u32_len(out: &mut Vec<u8>, len: usize, field: &'static str) -> Result<(), AuthorityError> {
    let value = u32::try_from(len).map_err(|_| AuthorityError::CanonicalCollectionTooLarge {
        field,
        len,
        max: u32::MAX as usize,
    })?;
    out.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PrincipalId, GENESIS_NONCE_LEN};

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn principal(byte: u8) -> PrincipalId {
        PrincipalId::new(digest(byte))
    }

    fn grant(principal: PrincipalId, capabilities: &[Capability]) -> PrincipalGrant {
        PrincipalGrant::new(principal, capabilities.iter().copied()).unwrap()
    }

    fn root_policy() -> RootAuthorityPolicy {
        RootAuthorityPolicy::new(
            vec![
                grant(
                    principal(1),
                    &[Capability::ManageAuthority, Capability::ReviewSource],
                ),
                grant(principal(2), &[Capability::ManageAuthority]),
            ],
            vec![
                CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
        )
        .unwrap()
    }

    fn fixture() -> (RootAuthorityPolicy, ProjectIdentitySeed, ProjectIdentity) {
        let policy = root_policy();
        let commitment = policy.commitment(DigestAlgorithm::Sha256).unwrap();
        let seed = ProjectIdentitySeed::new([0x44; GENESIS_NONCE_LEN], commitment);
        let project = ProjectIdentity::derive(&seed, DigestAlgorithm::Sha256).unwrap();
        (policy, seed, project)
    }

    #[test]
    fn root_policy_canonicalization_is_order_independent() {
        let a = root_policy();
        let b = RootAuthorityPolicy::new(
            vec![
                grant(principal(2), &[Capability::ManageAuthority]),
                grant(
                    principal(1),
                    &[Capability::ReviewSource, Capability::ManageAuthority],
                ),
            ],
            vec![
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
                CapabilityRule::new(Capability::ManageAuthority, 2).unwrap(),
            ],
        )
        .unwrap();
        assert_eq!(a.canonical_bytes().unwrap(), b.canonical_bytes().unwrap());
        assert_eq!(
            a.commitment(DigestAlgorithm::Sha256).unwrap(),
            b.commitment(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn identity_bound_genesis_accepts_exact_root_policy() {
        let (policy, seed, project) = fixture();
        let bound = BoundGenesisAuthority::new(&seed, project.clone(), policy, 1_000, None)
            .unwrap();
        assert_eq!(bound.epoch().project(), &project);
        assert_eq!(bound.epoch().sequence(), 0);
        assert_eq!(
            bound.root_policy_commitment(),
            seed.root_authority_commitment()
        );
    }

    #[test]
    fn mutated_root_policy_cannot_launder_into_genesis() {
        let (_policy, seed, project) = fixture();
        let mutated = RootAuthorityPolicy::new(
            vec![
                grant(
                    principal(1),
                    &[Capability::ManageAuthority, Capability::ReviewSource],
                ),
                grant(principal(2), &[Capability::ManageAuthority]),
            ],
            vec![
                CapabilityRule::new(Capability::ManageAuthority, 1).unwrap(),
                CapabilityRule::new(Capability::ReviewSource, 1).unwrap(),
            ],
        )
        .unwrap();

        assert!(matches!(
            BoundGenesisAuthority::new(&seed, project, mutated, 1_000, None),
            Err(GenesisBindingError::RootPolicyCommitmentMismatch { .. })
        ));
    }

    #[test]
    fn wrong_project_identity_is_rejected() {
        let (policy, seed, _project) = fixture();
        let wrong_seed = ProjectIdentitySeed::new(
            [0x45; GENESIS_NONCE_LEN],
            seed.root_authority_commitment().clone(),
        );
        let wrong_project =
            ProjectIdentity::derive(&wrong_seed, DigestAlgorithm::Sha256).unwrap();

        assert_eq!(
            BoundGenesisAuthority::new(&seed, wrong_project, policy, 1_000, None).unwrap_err(),
            GenesisBindingError::ProjectIdentityMismatch
        );
    }
}
