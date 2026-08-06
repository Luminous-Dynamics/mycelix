// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Independently operated transparency-checkpoint mirror observations.
//!
//! A governed transparency witness may also operate a public mirror. The
//! observation proves that a particular checkpoint was retrievable at a stable
//! HTTPS location at a stated time. Mirror evidence is supplemental: it does
//! not replace checkpoint publication or organization-diverse witness signing.

use crate::scientific_credential_governance::CredentialGovernanceProjection;
use crate::{ActorId, ContentHash, Error, OrganizationId, Result};
use chrono::{DateTime, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde::{Deserialize, Serialize};
use url::Url;
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CheckpointMirrorObservationId(pub Uuid);

impl CheckpointMirrorObservationId {
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for CheckpointMirrorObservationId {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedCheckpointMirrorObservation {
    pub observation_id: CheckpointMirrorObservationId,
    pub checkpoint_hash: ContentHash,
    pub actor: ActorId,
    pub organization: OrganizationId,
    pub mirror_uri: String,
    pub observed_at: DateTime<Utc>,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedCheckpointMirrorObservation {
    pub fn sign(
        checkpoint_hash: ContentHash,
        actor: ActorId,
        organization: OrganizationId,
        mirror_uri: impl Into<String>,
        observed_at: DateTime<Utc>,
        signing_key: &SigningKey,
    ) -> Result<Self> {
        let mut observation = Self {
            observation_id: CheckpointMirrorObservationId::new(),
            checkpoint_hash,
            actor,
            organization,
            mirror_uri: mirror_uri.into(),
            observed_at,
            signer_public_key: signing_key.verifying_key().to_bytes(),
            signature: Vec::new(),
        };
        observation.validate_common()?;
        observation.signature = signing_key
            .sign(&observation.signing_bytes()?)
            .to_bytes()
            .to_vec();
        Ok(observation)
    }

    pub fn verify(&self) -> Result<()> {
        self.validate_common()?;
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify_strict(&self.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn verify_against_governance(
        &self,
        governance: &CredentialGovernanceProjection,
    ) -> Result<()> {
        self.verify()?;
        let checkpoint = governance
            .checkpoints
            .iter()
            .find(|checkpoint| checkpoint.checkpoint_hash() == self.checkpoint_hash)
            .ok_or_else(|| {
                Error::NotFound(
                    "mirrored checkpoint is not present in governed checkpoint history".to_string(),
                )
            })?;
        if self.observed_at < checkpoint.issued_at {
            return Err(Error::VerificationFailed(
                "mirror observation predates checkpoint publication".to_string(),
            ));
        }
        let witness = governance
            .active_transparency_witness(&self.actor, &self.signer_public_key, &self.observed_at)
            .ok_or_else(|| {
                Error::VerificationFailed(
                    "mirror signer is not an active uncompromised transparency witness".to_string(),
                )
            })?;
        if witness.organization != self.organization {
            return Err(Error::VerificationFailed(
                "mirror observation organization does not match governed witness authority"
                    .to_string(),
            ));
        }
        Ok(())
    }

    pub fn observation_hash(&self) -> Result<ContentHash> {
        let mut bytes = self.signing_bytes()?;
        bytes.extend_from_slice(&self.signature);
        Ok(ContentHash::digest(&bytes))
    }

    fn validate_common(&self) -> Result<()> {
        self.actor.validate()?;
        self.organization.validate()?;
        let uri = Url::parse(&self.mirror_uri).map_err(|error| {
            Error::Validation(format!("invalid checkpoint mirror URI: {error}"))
        })?;
        if uri.scheme() != "https" || uri.host_str().is_none() {
            return Err(Error::Validation(
                "checkpoint mirror URI must be an absolute HTTPS URL".to_string(),
            ));
        }
        if self.mirror_uri.len() > 2_048
            || uri.username() != ""
            || uri.password().is_some()
            || uri.query().is_some()
            || uri.fragment().is_some()
        {
            return Err(Error::Validation(
                "checkpoint mirror URI is too long or contains user information, a query, or a fragment"
                    .to_string(),
            ));
        }
        if !uri
            .path()
            .to_ascii_lowercase()
            .contains(&self.checkpoint_hash.to_hex())
        {
            return Err(Error::Validation(
                "checkpoint mirror URI must contain the immutable checkpoint hash".to_string(),
            ));
        }
        Ok(())
    }

    fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate_common()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CHECKPOINT-MIRROR-OBSERVATION\0");
        bytes.extend_from_slice(self.observation_id.0.as_bytes());
        bytes.extend_from_slice(&self.checkpoint_hash.0);
        push_string(&mut bytes, self.actor.as_str())?;
        push_string(&mut bytes, self.organization.as_str())?;
        push_string(&mut bytes, &self.mirror_uri)?;
        bytes.extend_from_slice(&self.observed_at.timestamp().to_be_bytes());
        bytes.extend_from_slice(&self.observed_at.timestamp_subsec_nanos().to_be_bytes());
        bytes.extend_from_slice(&self.signer_public_key);
        Ok(bytes)
    }
}

fn push_string(bytes: &mut Vec<u8>, value: &str) -> Result<()> {
    let length = u32::try_from(value.len())
        .map_err(|_| Error::Validation("canonical mirror field exceeds u32".to_string()))?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scientific_credential_governance::{
        AuthorizedCredentialTransparencyWitness, CredentialTransparencyCheckpoint,
        TransparencyWitnessCompromiseInterval,
    };
    use std::collections::BTreeMap;

    #[test]
    fn mirror_uri_must_be_hash_addressed_in_the_path() {
        let key = SigningKey::from_bytes(&[40; 32]);
        let checkpoint_hash = ContentHash::digest(b"checkpoint");
        let result = SignedCheckpointMirrorObservation::sign(
            checkpoint_hash,
            ActorId::new("did:key:mirror-latest").unwrap(),
            OrganizationId::new("org:mirror-latest").unwrap(),
            format!(
                "https://mirror.example/checkpoints/latest.json?hash={}",
                checkpoint_hash.to_hex()
            ),
            Utc::now(),
            &key,
        );
        assert!(result.is_err());
    }

    #[test]
    fn compromise_interval_invalidates_only_affected_observations() {
        let key = SigningKey::from_bytes(&[41; 32]);
        let actor = ActorId::new("did:key:mirror-one").unwrap();
        let organization = OrganizationId::new("org:mirror-one").unwrap();
        let issued_at = Utc::now();
        let checkpoint = CredentialTransparencyCheckpoint {
            registry_event_count: 1,
            registry_head: ContentHash::digest(b"registry-head"),
            registry_merkle_root: ContentHash::digest(b"registry-root"),
            governance_event_count: 1,
            governance_head: ContentHash::digest(b"governance-head"),
            governance_merkle_root: ContentHash::digest(b"governance-root"),
            issued_at,
        };
        let checkpoint_hash = checkpoint.checkpoint_hash();
        let mirror_uri = format!(
            "https://mirror.example/checkpoints/{}.json",
            checkpoint_hash.to_hex()
        );
        let observation = SignedCheckpointMirrorObservation::sign(
            checkpoint_hash,
            actor.clone(),
            organization.clone(),
            mirror_uri,
            issued_at + chrono::Duration::minutes(5),
            &key,
        )
        .unwrap();
        let mut projection = CredentialGovernanceProjection::default();
        projection.checkpoints.push(checkpoint);
        projection.transparency_witnesses.insert(
            actor.clone(),
            AuthorizedCredentialTransparencyWitness {
                actor: actor.clone(),
                organization,
                public_key: key.verifying_key().to_bytes(),
                valid_from: issued_at - chrono::Duration::minutes(1),
                valid_until: None,
                revoked_at: None,
            },
        );
        observation.verify_against_governance(&projection).unwrap();

        projection.transparency_witness_compromises = BTreeMap::from([(
            actor,
            vec![TransparencyWitnessCompromiseInterval {
                compromised_from: issued_at + chrono::Duration::minutes(2),
                restored_at: Some(issued_at + chrono::Duration::minutes(10)),
                reason: "forensic compromise window".to_string(),
            }],
        )]);
        assert!(observation.verify_against_governance(&projection).is_err());
    }
}
