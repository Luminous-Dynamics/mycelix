// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-bearing bindings layered over the portable repository contract.

use crate::contract::{
    structurally_qualify_observation, AdapterObservation, RepositoryAdoption,
    RepositoryPolicyState, RepositoryVerificationError, RepositoryVerificationRequest,
    StructurallyQualifiedRepositoryVerification, VerificationCapability, VerificationProfile,
};
use mycelix_forge_core::Digest;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

/// Positive proof that a sequence-zero repository policy state is the exact
/// external policy committed by the repository adoption statement.
///
/// A structurally valid sequence-zero [`RepositoryPolicyState`] is not enough:
/// consumers that need an accepted lineage root should require this type (or
/// independently reproduce the same checks).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundRepositoryPolicyGenesis {
    adoption: RepositoryAdoption,
    genesis: RepositoryPolicyState,
}

impl BoundRepositoryPolicyGenesis {
    pub fn new(
        adoption: RepositoryAdoption,
        genesis: RepositoryPolicyState,
    ) -> Result<Self, RepositoryEvidenceError> {
        if adoption.project() != genesis.project() {
            return Err(RepositoryEvidenceError::GenesisProjectMismatch);
        }
        if genesis.sequence() != 0 {
            return Err(RepositoryEvidenceError::ExpectedGenesisPolicyState(
                genesis.sequence(),
            ));
        }
        if adoption.repository_policy() != genesis.policy_digest() {
            return Err(RepositoryEvidenceError::AdoptionPolicyMismatch);
        }

        Ok(Self { adoption, genesis })
    }

    pub fn adoption(&self) -> &RepositoryAdoption {
        &self.adoption
    }

    pub fn genesis(&self) -> &RepositoryPolicyState {
        &self.genesis
    }
}

/// An adapter observation plus commitments for semantics that must not be
/// accepted as unbacked capability flags.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EvidenceBackedAdapterObservation {
    observation: AdapterObservation,
    policy_lineage_commitment: Option<Digest>,
}

impl EvidenceBackedAdapterObservation {
    pub fn new(
        observation: AdapterObservation,
        policy_lineage_commitment: Option<Digest>,
    ) -> Result<Self, RepositoryEvidenceError> {
        if observation
            .capabilities()
            .contains(&VerificationCapability::PolicyLineageMonotonic)
            && policy_lineage_commitment.is_none()
        {
            return Err(RepositoryEvidenceError::MissingPolicyLineageCommitment);
        }

        Ok(Self {
            observation,
            policy_lineage_commitment,
        })
    }

    pub fn observation(&self) -> &AdapterObservation {
        &self.observation
    }

    pub fn policy_lineage_commitment(&self) -> Option<&Digest> {
        self.policy_lineage_commitment.as_ref()
    }
}

impl<'de> Deserialize<'de> for EvidenceBackedAdapterObservation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct WireEvidenceBackedObservation {
            observation: AdapterObservation,
            policy_lineage_commitment: Option<Digest>,
        }

        let wire = WireEvidenceBackedObservation::deserialize(deserializer)?;
        Self::new(wire.observation, wire.policy_lineage_commitment).map_err(D::Error::custom)
    }
}

/// Positive runtime result that preserves both the structural verification
/// result and the exact commitment to the monotonic repository-policy lineage.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedRepositoryVerification {
    structural: StructurallyQualifiedRepositoryVerification,
    policy_lineage_commitment: Option<Digest>,
}

impl QualifiedRepositoryVerification {
    pub fn structural(&self) -> &StructurallyQualifiedRepositoryVerification {
        &self.structural
    }

    pub fn policy_lineage_commitment(&self) -> Option<&Digest> {
        self.policy_lineage_commitment.as_ref()
    }
}

/// Qualify one evidence-backed adapter observation against the exact request
/// and required profile.
///
/// The inner contract validates request/tip/policy-state binding, adapter
/// outcome, and required capabilities. This layer additionally guarantees
/// that a monotonic-policy claim has an explicit lineage commitment.
pub fn qualify_evidence_backed_observation(
    profile: &VerificationProfile,
    request: &RepositoryVerificationRequest,
    observation: EvidenceBackedAdapterObservation,
) -> Result<QualifiedRepositoryVerification, RepositoryEvidenceError> {
    let EvidenceBackedAdapterObservation {
        observation,
        policy_lineage_commitment,
    } = observation;

    let structural = structurally_qualify_observation(profile, request, observation)?;

    if structural
        .capabilities()
        .contains(&VerificationCapability::PolicyLineageMonotonic)
        && policy_lineage_commitment.is_none()
    {
        // Constructor/deserializer already enforces this. Keep the check at the
        // positive-type boundary as defense in depth if construction evolves.
        return Err(RepositoryEvidenceError::MissingPolicyLineageCommitment);
    }

    Ok(QualifiedRepositoryVerification {
        structural,
        policy_lineage_commitment,
    })
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RepositoryEvidenceError {
    #[error(transparent)]
    Contract(#[from] RepositoryVerificationError),
    #[error("repository adoption and policy genesis belong to different projects")]
    GenesisProjectMismatch,
    #[error("expected repository policy genesis state at sequence 0, got {0}")]
    ExpectedGenesisPolicyState(u64),
    #[error("repository adoption policy commitment does not match policy genesis")]
    AdoptionPolicyMismatch,
    #[error("PolicyLineageMonotonic capability requires a policy-lineage commitment")]
    MissingPolicyLineageCommitment,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contract::{
        AdapterIdentity, AdapterOutcome, GitObjectAlgorithm, GitObjectId, RepositoryRef,
        RepositoryTip,
    };
    use mycelix_forge_core::{
        DigestAlgorithm, ProjectIdentity, ProjectIdentitySeed, GENESIS_NONCE_LEN,
    };

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn project() -> ProjectIdentity {
        ProjectIdentity::derive(
            &ProjectIdentitySeed::new([0x11; GENESIS_NONCE_LEN], digest(0x22)),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn git_sha1(byte: u8) -> GitObjectId {
        GitObjectId::new(GitObjectAlgorithm::Sha1, vec![byte; 20]).unwrap()
    }

    fn adoption() -> RepositoryAdoption {
        RepositoryAdoption::new(
            project(),
            RepositoryTip::new(
                RepositoryRef::new("refs/heads/main").unwrap(),
                git_sha1(0x33),
            ),
            digest(0x44),
            digest(0x55),
            digest(0x66),
            1_000,
        )
    }

    fn genesis() -> RepositoryPolicyState {
        RepositoryPolicyState::new(project(), 0, None, digest(0x66)).unwrap()
    }

    fn request() -> RepositoryVerificationRequest {
        RepositoryVerificationRequest::new(
            &adoption(),
            git_sha1(0x33),
            git_sha1(0x77),
            digest(0x44),
            digest(0x55),
            &genesis(),
            DigestAlgorithm::Sha256,
        )
        .unwrap()
    }

    fn full_observation() -> AdapterObservation {
        let request = request();
        AdapterObservation::new(
            AdapterIdentity::new("gittuf", "0.16.0").unwrap(),
            request.digest(DigestAlgorithm::Sha256).unwrap(),
            request.to().clone(),
            request.repository_policy_state().clone(),
            Some(digest(0x88)),
            Some(digest(0x89)),
            VerificationProfile::m0_protected_source()
                .required()
                .iter()
                .copied(),
            AdapterOutcome::Verified,
        )
        .unwrap()
    }

    #[test]
    fn policy_genesis_must_match_adoption_commitment() {
        let bound = BoundRepositoryPolicyGenesis::new(adoption(), genesis()).unwrap();
        assert_eq!(bound.genesis().sequence(), 0);

        let wrong = RepositoryPolicyState::new(project(), 0, None, digest(0x67)).unwrap();
        assert_eq!(
            BoundRepositoryPolicyGenesis::new(adoption(), wrong).unwrap_err(),
            RepositoryEvidenceError::AdoptionPolicyMismatch
        );
    }

    #[test]
    fn non_genesis_policy_state_cannot_bind_as_adoption_root() {
        let root = genesis();
        let successor = RepositoryPolicyState::new(
            project(),
            1,
            Some(root.digest(DigestAlgorithm::Sha256).unwrap()),
            digest(0x67),
        )
        .unwrap();
        assert_eq!(
            BoundRepositoryPolicyGenesis::new(adoption(), successor).unwrap_err(),
            RepositoryEvidenceError::ExpectedGenesisPolicyState(1)
        );
    }

    #[test]
    fn monotonic_policy_capability_requires_lineage_commitment() {
        assert_eq!(
            EvidenceBackedAdapterObservation::new(full_observation(), None).unwrap_err(),
            RepositoryEvidenceError::MissingPolicyLineageCommitment
        );
    }

    #[test]
    fn evidence_backed_observation_qualifies_exact_request() {
        let request = request();
        let lineage = digest(0x90);
        let observation =
            EvidenceBackedAdapterObservation::new(full_observation(), Some(lineage.clone()))
                .unwrap();
        let qualified = qualify_evidence_backed_observation(
            &VerificationProfile::m0_protected_source(),
            &request,
            observation,
        )
        .unwrap();
        assert_eq!(qualified.policy_lineage_commitment(), Some(&lineage));
        assert_eq!(qualified.structural().observed_tip(), request.to());
    }

    #[test]
    fn serde_cannot_assert_monotonic_policy_without_lineage_commitment() {
        let malformed = serde_json::json!({
            "observation": full_observation(),
            "policy_lineage_commitment": null
        });
        assert!(serde_json::from_value::<EvidenceBackedAdapterObservation>(malformed).is_err());
    }
}
